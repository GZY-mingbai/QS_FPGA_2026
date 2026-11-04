//==============================================================================
//  spi_flash_read.v  —— SPI Flash（W25Q64）读取控制器
//==============================================================================
//
//  【这个文件干什么的】
//  用 SPI 总线从开发板上那颗 W25Q64 Flash 里读数据。
//  专门为「读字库」这类用法设计：一次请求 = 给一个起始地址 + 要读多少字节，
//  然后数据就一个接一个地吐出来。
//
//  【硬件连接（HX4S20C 开发板）】
//  ┌──────────────┬────────────┬──────────────┬──────────────────────────┐
//  │ 本模块端口   │ Flash 引脚 │ FPGA 物理引脚│ 说明                     │
//  ├──────────────┼────────────┼──────────────┼──────────────────────────┤
//  │ flash_cs_n   │ CS#  (1)   │ P8           │ 片选，低有效             │
//  │ flash_miso   │ DO   (2)   │ N8           │ 输入：Flash → FPGA       │
//  │ flash_wp_n   │ WP#  (3)   │ P9           │ 恒输出 1（不写保护）     │
//  │ flash_mosi   │ DI   (5)   │ P7           │ 输出：FPGA → Flash       │
//  │ flash_sclk   │ CLK  (6)   │ M9           │ SPI 时钟                 │
//  │ flash_hold_n │ HOLD#(7)   │ R9           │ 恒输出 1（不进入 HOLD）  │
//  └──────────────┴────────────┴──────────────┴──────────────────────────┘
//  ★ WP# 和 HOLD# 都是「低有效」，平时必须给高电平，否则芯片不工作。
//    本模块直接把它们恒驱动为 1，最省事。
//
//  【SPI 模式 = Mode 0】
//  W25Q64 支持 Mode 0 (CPOL=0,CPHA=0) 和 Mode 3 (CPOL=1,CPHA=1)。本模块用 Mode 0：
//    · SCLK 空闲时是低电平
//    · 数据在 SCLK 的「上升沿」被采样（收数据）
//    · 数据在 SCLK 的「下降沿」改变（发数据）
//
//  【读命令 = 03h (Read Data)】
//  一次读事务的线上时序：
//      CS#  ‾‾\___________________________________________________/‾‾‾
//      SCLK __/‾\_/‾\_/‾\_ ... _/‾\_/‾\_/‾\_/‾\_ ... _/‾\__________
//      MOSI     [  0x03   ][  A23..A0（24bit）  ][  全 0（无所谓） ]
//      MISO     [   忽略   ][      忽略         ][ D7..D0 ][ D7..D0 ]...
//                                                ↑ 从地址后的第一个上升沿开始出数据
//  · 地址发完后，Flash 会在每个 SCLK 周期移出 1 个数据 bit（高位先出）
//  · Flash 内部地址自动递增，所以只要 CS 保持低就能一直读下去
//  · 03h 指令最高支持 50MHz，本模块默认 12.5MHz，留足余量
//    （以后要跑更快就换成 0Bh 指令，并在地址后再补 1 个 dummy 字节）
//
//  【怎么用】
//      rd_ready  ─────┐                        ┌──────
//                     └────────────────────────┘
//      rd_req    ──┐  ┌─────────────────────────────
//                  └──┘
//      rd_valid  ────┐  ┌┐  ┌┐  ┌┐         ┌┐
//                    └──┘└──┘└──┘└─────────┘└─────────
//      rd_data   ──────<D0><D1><D2>.......<Dn>───────
//      rd_done   ───────────────────────────────┐  ┌──
//                                               └──┘
//      1. 等 rd_ready == 1
//      2. 摆好 rd_addr / rd_len，把 rd_req 拉高「一个时钟」再拉低
//      3. 每当 rd_valid == 1，rd_data 就是下一个字节（按地址递增顺序）
//      4. rd_done 拉高一拍 = 这次读完了，可以发下一次请求
//
//  【例化模板】
//      spi_flash_read #(
//          .CLK_FREQ_HZ (50000000),
//          .SCLK_DIV    (2),          // SCLK = 50M / (2*2) = 12.5MHz
//          .POR_CYCLES  (50000)       // 上电等 1ms
//      ) u_flash (
//          .clk          (clk),
//          .rst_n        (rst_n),
//          .rd_req       (flash_rd_req),
//          .rd_addr      (flash_rd_addr),    // 24bit
//          .rd_len       (flash_rd_len),     // 16bit，要读几个字节
//          .rd_ready     (flash_rd_ready),
//          .rd_data      (flash_rd_data),
//          .rd_valid     (flash_rd_valid),
//          .rd_done      (flash_rd_done),
//          .flash_cs_n   (flash_cs_n),
//          .flash_sclk   (flash_sclk),
//          .flash_mosi   (flash_mosi),
//          .flash_miso   (flash_miso),
//          .flash_wp_n   (flash_wp_n),
//          .flash_hold_n (flash_hold_n)
//      );
//
//  【注意】
//  · 地址范围：W25Q64 = 64Mbit = 8MB，所以有效地址是 0x000000 ~ 0x7FFFFF。
//    超出范围时 Flash 会从 0 重新开始（回绕），不会报错。
//  · rd_len 一次最多 65535 字节；要读更多就分多次请求。
//  · ★ rd_req 必须是「单拍脉冲」。若一直保持高电平，读完后会立刻又读一遍。
//  · ★ rd_done 和「最后一个 rd_valid」出现在同一拍 —— 收到 rd_done 时，
//    那一拍的数据也在总线上，照常用 rd_valid 采样就行，不要漏掉最后一个字节。
//  · 读速度：SCLK=12.5MHz 时约 1.5MB/s（含命令开销）。如果下游是实时视频
//    显示，请先把字库读出来缓存到 SDRAM，不要一边显示一边读 Flash。
//
//  【已验证】（iverilog 行为级仿真，W25Q64 从机模型）
//  · 单字节 / 8 字节 / 32 字节（跨 256 边界）/ 1000 字节连续读，数据全部正确
//  · 背靠背两次请求互不干扰
//  · SCLK_DIV = 1 / 2 / 5 / 10 四种分频，实测 SCLK 周期 40/80/200/400ns，
//    与 CLK_FREQ_HZ/(2*SCLK_DIV) 完全一致；MOSI 不在 SCLK 高电平期间变化
//
//==============================================================================

module spi_flash_read #(
    parameter CLK_FREQ_HZ = 50000000,   // 系统时钟频率（仅供参考，不影响逻辑）
    parameter SCLK_DIV    = 2,          // 半周期分频，必须 >= 1：SCLK = CLK_FREQ_HZ/(2*SCLK_DIV)
                                        //   1 → 25MHz   2 → 12.5MHz（默认）  5 → 5MHz  10 → 2.5MHz
    parameter POR_CYCLES  = 50000       // 上电等待周期数（50MHz 下 50000 ≈ 1ms）
)(
    //------------------------------------------------------------------
    //  系统
    //------------------------------------------------------------------
    input  wire        clk,             // 系统时钟（本工程是 50MHz 晶振，引脚 R7）
    input  wire        rst_n,           // 复位，低有效

    //------------------------------------------------------------------
    //  用户读接口
    //------------------------------------------------------------------
    input  wire        rd_req,          // 读请求，单拍脉冲
    input  wire [23:0] rd_addr,         // 起始地址（24bit）
    input  wire [15:0] rd_len,          // 要读的字节数（1 ~ 65535）
    output wire        rd_ready,        // 空闲、可以接受新请求
    output reg  [7:0]  rd_data,         // 读出的数据
    output reg         rd_valid,        // rd_data 有效，单拍脉冲
    output wire        rd_done,         // 一次读事务完成，单拍脉冲

    //------------------------------------------------------------------
    //  Flash 物理接口
    //------------------------------------------------------------------
    output wire        flash_cs_n,      // 片选   → P8
    output wire        flash_sclk,      // 时钟   → M9
    output wire        flash_mosi,      // 主机出 → P7  (DI)
    input  wire        flash_miso,      // 主机入 ← N8  (DO)
    output wire        flash_wp_n,      // 写保护 → P9  （恒 1）
    output wire        flash_hold_n     // HOLD   → R9  （恒 1）
);

//==============================================================================
//  1. 常量与状态编码
//==============================================================================
localparam [7:0] CMD_READ_DATA = 8'h03;   // 「读数据」指令

localparam [1:0] ST_IDLE = 2'd0;          // 空闲，等请求
localparam [1:0] ST_TX   = 2'd1;          // 发送：1 字节命令 + 3 字节地址 = 32bit
localparam [1:0] ST_RX   = 2'd2;          // 接收：连续读数据
localparam [1:0] ST_DONE = 2'd3;          // 收尾：拉高 CS，给出完成脉冲

//==============================================================================
//  2. 寄存器
//==============================================================================
reg  [1:0]  state;      // 当前状态
reg  [15:0] tick_cnt;   // 半周期计时器
reg         sclk_r;     // SCLK 寄存输出
reg  [2:0]  bit_cnt;    // 当前字节内的 bit 序号（0~7，3bit 会在 7→0 自动回绕）
reg  [2:0]  tx_byte;    // TX 阶段已发字节数（0~3）
reg  [15:0] rx_cnt;     // RX 阶段已收字节数
// ★★★ 2026-10-27：本次事务的长度必须**在收单那一拍锁存** ★★★
//  【上板现象】OSD 导航图标只有纯色色块（5 张 4bpp 覆盖率图没有形状）。
//  【根因】原来 `rx_last` 直接比较**端口上的 rd_len**：
//        wire rx_last = (state == ST_RX) && (rx_cnt == (rd_len - 16'd1));
//    rd_len 由 flash_arb 组合给出，而 arbiter 的 sel_len 是"看当前谁在请求"：
//    图标客户发完单拍 req 就进接收态、req 撤销 ⇒ `fl_len` 立刻回落成
//    **字库客户的 32**。于是图标这次 2560 字节的传输在第 32 字节就
//    "收满"了：spi_flash_read 提前给 done，osd_icon_rom 只写完前 8 个字
//    （32B），其余 632 个字保持上电值（BRAM=0 ⇒ 覆盖率 0 ⇒ 整块底板色）
//    ⇒ 5 个图标全部是纯色块，而且 data_pgm=1（首字节 0x00 非 0xFF）
//    所以"未编程检测"也拦不住。
//  【为什么字库没暴露】字库每次就是 32 字节，回落后仍是 32，恰好一致。
//  【改法】收单时把 rd_len 锁进 len_r，整段事务只用 len_r。地址本来就已
//    在 ST_IDLE 同一拍装进 tx_buf，长度现在也一致了。
reg  [15:0] len_r;      // 本次事务的字节数（收单时锁存）
reg  [31:0] tx_buf;     // 32bit 发送移位寄存器 {命令, 地址}
reg  [7:0]  rx_shift;   // 接收移位寄存器
reg  [31:0] por_cnt;    // 上电延时计数器

//==============================================================================
//  3. 组合信号
//==============================================================================
// tick：半个 SCLK 周期到了。SCLK 频率 = CLK_FREQ_HZ / (2*SCLK_DIV)
wire tick       = (tick_cnt == (SCLK_DIV - 1));

// 正在和 Flash 通信（CS 拉低期间）
wire spi_active = (state == ST_TX) || (state == ST_RX);

// 上电延时结束
wire por_done   = (por_cnt >= POR_CYCLES);

// byte_done：一个字节（8 个 bit）收发完毕。
// 发生在第 8 个 SCLK 的「下降沿」——此时 bit_cnt 刚回绕到 0。
wire byte_done  = spi_active && tick && (sclk_r == 1'b1) && (bit_cnt == 3'd0);

// 这是本次事务要读的最后一个字节
// ★ 用锁存后的 len_r，不能用端口上的 rd_len（见 §2 的说明）
wire rx_last    = (state == ST_RX) && (rx_cnt == (len_r - 16'd1));

//==============================================================================
//  4. 上电延时计数器
//==============================================================================
// Flash 上电后需要一小段时间（tVSL）才能接受命令。这里数 POR_CYCLES 拍再放行。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        por_cnt <= 32'd0;
    else if (!por_done)
        por_cnt <= por_cnt + 32'd1;
end

//==============================================================================
//  5. 主状态机
//==============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= ST_IDLE;
        len_r <= 16'd0;
    end else
        case (state)
            // 空闲：等请求 + 上电延时结束 + 长度合法
            ST_IDLE:
                if (rd_req && por_done && (rd_len != 16'd0)) begin
                    len_r <= rd_len;          // ★ 长度随事务一起锁存
                    state <= ST_TX;
                end

            // 发完 4 个字节（命令 + 3 字节地址）就转接收
            ST_TX:
                if (byte_done && (tx_byte == 3'd3))
                    state <= ST_RX;

            // 收满 rd_len 个字节就收尾
            ST_RX:
                if (byte_done && rx_last)
                    state <= ST_DONE;

            // 收尾一拍，然后回空闲
            ST_DONE:
                state <= ST_IDLE;

            default:
                state <= ST_IDLE;
        endcase
end

//==============================================================================
//  6. 半周期计时器
//==============================================================================
// 通信期间从 0 数到 SCLK_DIV-1；数满（tick）就清零重新数。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        tick_cnt <= 16'd0;
    else if (!spi_active || tick)
        tick_cnt <= 16'd0;
    else
        tick_cnt <= tick_cnt + 16'd1;
end

//==============================================================================
//  7. SCLK 生成
//==============================================================================
// 每个 tick 翻转一次，得到 50% 占空比的 SPI 时钟。
// 不通信时（IDLE / DONE）强制拉低 —— 这就是 Mode 0 要求的「空闲低」。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        sclk_r <= 1'b0;
    else if (!spi_active)
        sclk_r <= 1'b0;
    else if (tick)
        sclk_r <= ~sclk_r;
end

//==============================================================================
//  8. 字节内 bit 计数
//==============================================================================
// 在 SCLK 上升沿（tick 且 sclk_r==0）采样完一位后 +1。
// 3bit 计数器，7 再 +1 自动回绕到 0，正好表示「一个字节收完了」。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bit_cnt <= 3'd0;
    else if (!spi_active)
        bit_cnt <= 3'd0;
    else if (tick && (sclk_r == 1'b0))
        bit_cnt <= bit_cnt + 3'd1;
end

//==============================================================================
//  9. 发送移位寄存器（MOSI）
//==============================================================================
// 装载：{命令, 24bit地址} 一共 32bit，MSB 先出。
// 移位：在 SCLK 下降沿（tick 且 sclk_r==1）左移一位。
//   ★ 为什么在下降沿移位：Mode 0 要求「数据在下降沿改变、在上升沿被采样」，
//     这样从机在下一个上升沿采样 MOSI 时，数据已经稳定了半个周期。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        tx_buf <= 32'd0;
    else if ((state == ST_IDLE) && rd_req && por_done && (rd_len != 16'd0))
        tx_buf <= {CMD_READ_DATA, rd_addr};       // 装载命令 + 地址
    else if (spi_active && tick && (sclk_r == 1'b1))
        tx_buf <= {tx_buf[30:0], 1'b0};           // 下降沿左移一位
end

//==============================================================================
//  10. 接收移位寄存器（MISO）
//==============================================================================
// 在 SCLK 上升沿采样 MISO，移入最低位（高位先入 → 最终 rx_shift 就是完整字节）。
// TX 阶段也照常采样（数据没用，但保持时序连续，方便以后换 0Bh 指令）。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rx_shift <= 8'd0;
    else if (!spi_active)
        rx_shift <= 8'd0;
    else if (tick && (sclk_r == 1'b0))
        rx_shift <= {rx_shift[6:0], flash_miso};
end

//==============================================================================
//  11. TX 阶段已发字节计数
//==============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        tx_byte <= 3'd0;
    else if (state != ST_TX)
        tx_byte <= 3'd0;
    else if (byte_done)
        tx_byte <= tx_byte + 3'd1;
end

//==============================================================================
//  12. RX 阶段已收字节计数
//==============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rx_cnt <= 16'd0;
    else if (state != ST_RX)
        rx_cnt <= 16'd0;
    else if (byte_done)
        rx_cnt <= rx_cnt + 16'd1;
end

//==============================================================================
//  13. 数据输出
//==============================================================================
// byte_done 发生在 SCLK 下降沿，此时 rx_shift 已经装好了完整的 8bit。
// 所以下一拍输出 rd_data，同时把 rd_valid 拉高一拍。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_data  <= 8'd0;
        rd_valid <= 1'b0;
    end else begin
        rd_data  <= rx_shift;
        rd_valid <= (state == ST_RX) && byte_done;
    end
end

//==============================================================================
//  14. 输出连线
//==============================================================================
assign rd_ready     = (state == ST_IDLE) && por_done;
assign rd_done      = (state == ST_DONE);

assign flash_cs_n   = !spi_active;    // 通信期间拉低，其余时间拉高
assign flash_sclk   = sclk_r;
assign flash_mosi   = tx_buf[31];     // 总是输出移位寄存器的最高位

// WP# 和 HOLD# 低有效 → 恒给高电平
assign flash_wp_n   = 1'b1;
assign flash_hold_n = 1'b1;

endmodule
