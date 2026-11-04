//==============================================================================
//  spi_flash_ctrl.v —— W25Q64 读 / 写 / 整片擦除控制器
//==============================================================================
//
//  【职责】
//  把「读 N 字节 / 写 N 字节 / 整片擦除 / 读 ID」这几个高层操作，
//  翻译成 W25Q64 要求的 SPI 命令序列。
//
//  【W25Q64 的硬规矩（决定了本模块的内部结构）】
//    · 读数据     03h + 24bit 地址，之后连续吐数据，地址自动递增
//    · 写使能     06h（每个写/擦事务之前都必须先发一次，CS 拉高后生效）
//    · 页编程     02h + 24bit 地址 + 最多 256 字节；**不能跨页**（页 = 256B 对齐）
//    · 整片擦除   C7h（要先写使能）
//    · 读状态     05h，返回的 bit0 = WIP（1 = 芯片正在忙）
//    · 每个事务都要 CS 拉低→发命令→CS 拉高，事务之间 CS 必须回到高
//
//  【数据通路】
//    · 读：本模块输出 rdata/rvalid，由上层写进缓冲
//    · 写：本模块输出 waddr，上层用组合逻辑把 buf[waddr] 送回 wdata
//      （这样本模块不用自己管缓冲，职责更单一）
//
//  【SPI 时序】Mode 0：SCLK 空闲低，上升沿采样，下降沿改变。
//    SCLK 频率 = CLK_FREQ / (2*SCLK_DIV)，默认 12.5MHz。
//
//==============================================================================

module spi_flash_ctrl #(
    parameter CLK_FREQ = 50000000,
    parameter SCLK_DIV = 2              // SCLK = CLK_FREQ/(2*SCLK_DIV)
)(
    input  wire        clk,
    input  wire        rst_n,

    //------------------------------------------------------------------
    //  命令接口
    //------------------------------------------------------------------
    input  wire [2:0]  op,              // 操作码
    input  wire        start,           // 启动脉冲
    input  wire [23:0] addr,            // 起始地址
    input  wire [15:0] len,             // 字节数（读/写用）
    output reg         busy,            // 忙
    output reg         done,            // 完成脉冲

    //------------------------------------------------------------------
    //  读数据输出
    //------------------------------------------------------------------
    output reg  [7:0]  rdata,
    output reg         rvalid,          // rdata 有效（单拍）

    //------------------------------------------------------------------
    //  写数据输入（上层把 buf[waddr] 接过来）
    //    waddr 是"预取地址"：指向下一个要锁存的字节。
    //    BRAM 读有 1 拍延迟，所以地址必须比当前发送的字节提前一格。
    //------------------------------------------------------------------
    output wire [11:0] waddr,           // 预取地址
    input  wire [7:0]  wdata,           // 缓冲返回的数据（= buf[waddr]，晚 1 拍）

    //------------------------------------------------------------------
    //  状态寄存器当前值（给上层做调试显示用）
    //------------------------------------------------------------------
    output reg  [7:0]  sr_val,

    //------------------------------------------------------------------
    //  Flash 物理接口
    //------------------------------------------------------------------
    output wire        flash_cs_n,
    output wire        flash_sclk,
    output wire        flash_mosi,
    input  wire        flash_miso,
    output wire        flash_wp_n,
    output wire        flash_hold_n
);

//==============================================================================
//  操作码
//==============================================================================
localparam [2:0] OP_ID    = 3'd0;   // 读 JEDEC ID（3 字节）
localparam [2:0] OP_READ  = 3'd1;   // 读数据
localparam [2:0] OP_WRITE = 3'd2;   // 写数据
localparam [2:0] OP_ERASE = 3'd3;   // 整片擦除

//==============================================================================
//  SPI 命令码
//==============================================================================
localparam [7:0] CMD_WREN  = 8'h06;  // 写使能
localparam [7:0] CMD_RDSR  = 8'h05;  // 读状态寄存器
localparam [7:0] CMD_READ  = 8'h03;  // 读数据
localparam [7:0] CMD_PP    = 8'h02;  // 页编程
localparam [7:0] CMD_CE    = 8'hC7;  // 整片擦除
localparam [7:0] CMD_RDID  = 8'h9F;  // 读 JEDEC ID

//==============================================================================
//  状态编码
//==============================================================================
localparam [3:0] ST_IDLE      = 4'd0;
localparam [3:0] ST_ID_CMD    = 4'd1;   // 发 9Fh
localparam [3:0] ST_ID_RD     = 4'd2;   // 收 3 字节 ID
localparam [3:0] ST_RD_CMD    = 4'd3;   // 发 03h
localparam [3:0] ST_RD_ADDR   = 4'd4;   // 发 24bit 地址
localparam [3:0] ST_RD_DATA   = 4'd5;   // 收 len 字节
localparam [3:0] ST_WREN      = 4'd6;   // 发 06h
localparam [3:0] ST_GAP       = 4'd7;   // 事务间隙（CS 拉高）
localparam [3:0] ST_PP_CMD    = 4'd8;   // 发 02h
localparam [3:0] ST_PP_ADDR   = 4'd9;   // 发 24bit 地址
localparam [3:0] ST_PP_DATA   = 4'd10;  // 发本页数据
localparam [3:0] ST_RDSR_CMD  = 4'd11;  // 发 05h
localparam [3:0] ST_RDSR_RD   = 4'd12;  // 收 1 字节状态
localparam [3:0] ST_CE_CMD    = 4'd13;  // 发 C7h
localparam [3:0] ST_DONE      = 4'd14;

//==============================================================================
//  寄存器
//==============================================================================
reg  [3:0]  state;
reg  [3:0]  gap_next;      // ST_GAP 结束后去哪
reg  [2:0]  op_r;

reg  [23:0] addr_r;        // 原始起始地址
reg  [23:0] cur_addr;      // 当前写页地址
reg  [15:0] len_r;         // 原始长度
reg  [15:0] remain;        // 剩余待写字节数
reg  [15:0] page_left;     // 本页还能写几个字节

reg  [7:0]  byte_idx;      // 阶段内已完成的字节数
reg  [15:0] rx_cnt;        // 读阶段已收字节数

reg  [15:0] tick_cnt;
reg         sclk_r;
reg  [2:0]  bit_cnt;
reg  [7:0]  tx_shift;      // 发送移位寄存器
reg  [7:0]  rx_shift;      // 接收移位寄存器

reg  [7:0]  gap_cnt;       // ST_GAP 计时

// 写数据预取（补偿 BRAM 的 1 拍读延迟）
reg  [11:0] pp_cnt;        // 下一个要锁存的字节偏移
reg  [7:0]  pp_data;       // 已锁存、当前要发送的字节

//==============================================================================
//  SPI 位时序
//==============================================================================
wire spi_active = (state != ST_IDLE) && (state != ST_GAP) && (state != ST_DONE);
wire tick       = (tick_cnt == (SCLK_DIV - 1));
wire byte_done  = spi_active && tick && (sclk_r == 1'b1) && (bit_cnt == 3'd0);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        tick_cnt <= 16'd0;
    else if (!spi_active || tick)
        tick_cnt <= 16'd0;
    else
        tick_cnt <= tick_cnt + 16'd1;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        sclk_r <= 1'b0;
    else if (!spi_active)
        sclk_r <= 1'b0;
    else if (tick)
        sclk_r <= ~sclk_r;
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        bit_cnt <= 3'd0;
    else if (!spi_active)
        bit_cnt <= 3'd0;
    else if (tick && (sclk_r == 1'b0))
        bit_cnt <= bit_cnt + 3'd1;
end

//==============================================================================
//  发送 / 接收移位
//==============================================================================
// 当前阶段要发送的字节（组合）
reg [7:0] tx_byte;
always @(*) begin
    case (state)
        ST_ID_CMD:   tx_byte = CMD_RDID;
        ST_RD_CMD:   tx_byte = CMD_READ;
        ST_WREN:     tx_byte = CMD_WREN;
        ST_PP_CMD:   tx_byte = CMD_PP;
        ST_RDSR_CMD: tx_byte = CMD_RDSR;
        ST_CE_CMD:   tx_byte = CMD_CE;
        ST_RD_ADDR:  tx_byte = (byte_idx == 8'd0) ? addr_r[23:16] :
                               (byte_idx == 8'd1) ? addr_r[15:8]  : addr_r[7:0];
        ST_PP_ADDR:  tx_byte = (byte_idx == 8'd0) ? cur_addr[23:16] :
                               (byte_idx == 8'd1) ? cur_addr[15:8]  : cur_addr[7:0];
        ST_PP_DATA:  tx_byte = pp_data;
        default:     tx_byte = 8'h00;
    endcase
end

// 装载时机：进入某个 SPI 状态的第一拍（bit_cnt==0 且还没开始发）
wire tx_load = spi_active && (bit_cnt == 3'd0) && (tick_cnt == 16'd0);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        tx_shift <= 8'h00;
    else if (tx_load)
        tx_shift <= tx_byte;
    else if (spi_active && tick && (sclk_r == 1'b1))
        tx_shift <= {tx_shift[6:0], 1'b0};
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rx_shift <= 8'd0;
    else if (!spi_active)
        rx_shift <= 8'd0;
    else if (tick && (sclk_r == 1'b0))
        rx_shift <= {rx_shift[6:0], flash_miso};
end

//==============================================================================
//  字节内 / 阶段内计数
//==============================================================================
// 每个"命令字节"状态都把 byte_idx 归零（下一阶段从 0 开始计）；
// 地址段发完也要归零——否则 ST_PP_DATA 会从 3 开始计数，每页少写 3 字节。
wire bidx_clr = (state == ST_IDLE)     || (state == ST_GAP)      ||
                (state == ST_ID_CMD)   || (state == ST_RD_CMD)   ||
                (state == ST_WREN)     || (state == ST_PP_CMD)   ||
                (state == ST_RDSR_CMD) || (state == ST_CE_CMD)   ||
                (state == ST_RD_ADDR && byte_done && (byte_idx == 8'd2)) ||
                (state == ST_PP_ADDR && byte_done && (byte_idx == 8'd2));

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        byte_idx <= 8'd0;
    else if (bidx_clr)
        byte_idx <= 8'd0;
    else if (byte_done)
        byte_idx <= byte_idx + 8'd1;
end

//==============================================================================
//  CS 控制：SPI 事务内拉低，间隙拉高
//==============================================================================
assign flash_cs_n = ~spi_active;

//==============================================================================
//  读数据输出
//==============================================================================
//  【极其重要】rvalid 只能在"真的在读数据"时拉高：
//    · ST_RD_DATA —— 读 Flash 数据（要进缓冲）
//    · ST_ID_RD   —— 读 JEDEC ID（要进缓冲，上层从缓冲里取 3 字节发回去）
//  **绝对不能包含 ST_RDSR_RD**。等 WIP 清零时，控制器每轮询一次就读回一个
//  状态字节（0x03=忙 / 0x00=空闲），而页编程一次要轮询 ~280 次。上层是按
//  递增下标把 rvalid 字节写进缓冲的，这些状态字节会把**后面还没烧的页**的
//  数据整片覆盖掉。症状是：每页开头一截被写成 0x03，且第 0 页永远正常
//  （还没轮到它被覆盖就已经烧完了）——非常隐蔽。
//==============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rdata  <= 8'd0;
        rvalid <= 1'b0;
    end else begin
        rdata  <= rx_shift;
        rvalid <= (state == ST_RD_DATA || state == ST_ID_RD) && byte_done;
    end
end

//==============================================================================
//  写数据预取
//==============================================================================
//  BRAM 的同步读有 1 拍延迟：这一拍给地址，下一拍才出数据。
//  所以这里做一级"预取"：
//    · ST_PP_CMD 时把指针归零
//    · ST_PP_ADDR 发完（本页第 0 字节要发了）→ 锁存 buf[0]
//    · ST_PP_DATA 每完成一个字节 → 锁存下一个字节
//  tx_byte 直接用 pp_data，不再实时取，彻底避开时序错位。
assign waddr = pp_cnt;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pp_cnt  <= 12'd0;
        pp_data <= 8'd0;
    end else if (state == ST_IDLE) begin
        pp_cnt  <= 12'd0;       // 每次请求从头开始。跨页时绝不能清零，
                                // 否则第 2 页会重复读 buf[0..] 而不是 buf[256..]
    end else if (state == ST_PP_ADDR && byte_done && (byte_idx == 8'd2)) begin
        pp_data <= wdata;                                   // 锁存本页第 0 字节
        pp_cnt  <= pp_cnt + 12'd1;
    end else if (state == ST_PP_DATA && byte_done &&
                 (({8'd0, byte_idx} + 16'd1) < page_left)) begin
        pp_data <= wdata;                                   // 锁存下一字节
        pp_cnt  <= pp_cnt + 12'd1;                          // 末字节不再递增，
                                                            // 否则下一页起点会偏 1
    end
end

//==============================================================================
//  主状态机
//==============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state     <= ST_IDLE;
        gap_next  <= ST_IDLE;
        op_r      <= 3'd0;
        addr_r    <= 24'd0;
        cur_addr  <= 24'd0;
        len_r     <= 16'd0;
        remain    <= 16'd0;
        page_left <= 16'd0;
        rx_cnt    <= 16'd0;
        gap_cnt   <= 8'd0;
        sr_val    <= 8'd0;
        busy      <= 1'b0;
        done      <= 1'b0;
    end else begin
        done <= 1'b0;

        case (state)
            //----------------------------------------------------------
            ST_IDLE: begin
                busy <= 1'b0;
                gap_cnt <= 8'd0;
                rx_cnt  <= 16'd0;
                if (start) begin
                    op_r     <= op;
                    addr_r   <= addr;
                    cur_addr <= addr;
                    len_r    <= len;
                    remain   <= len;
                    busy     <= 1'b1;
                    // len=0 的读/写直接完成（否则 len_r-1 会下溢成 65535）
                    if ((op == OP_READ || op == OP_WRITE) && (len == 16'd0))
                        state <= ST_DONE;
                    else case (op)
                        OP_ID:    state <= ST_ID_CMD;
                        OP_READ:  state <= ST_RD_CMD;
                        OP_WRITE: state <= ST_WREN;      // 写前先写使能
                        OP_ERASE: state <= ST_WREN;      // 擦前也先写使能
                        default:  state <= ST_IDLE;
                    endcase
                end
            end

            //----------------------------------------------------------
            //  读 JEDEC ID
            //----------------------------------------------------------
            ST_ID_CMD: if (byte_done) state <= ST_ID_RD;

            ST_ID_RD:  if (byte_done && (byte_idx == 8'd2)) state <= ST_DONE;

            //----------------------------------------------------------
            //  读数据
            //----------------------------------------------------------
            ST_RD_CMD:  if (byte_done) state <= ST_RD_ADDR;

            ST_RD_ADDR: if (byte_done && (byte_idx == 8'd2)) state <= ST_RD_DATA;

            ST_RD_DATA: if (byte_done) begin
                rx_cnt <= rx_cnt + 16'd1;
                if (rx_cnt == (len_r - 16'd1)) state <= ST_DONE;
            end

            //----------------------------------------------------------
            //  写使能（写/擦共用入口）
            //----------------------------------------------------------
            ST_WREN: if (byte_done) begin
                gap_next <= (op_r == OP_WRITE) ? ST_PP_CMD : ST_CE_CMD;
                state    <= ST_GAP;
            end

            //----------------------------------------------------------
            //  事务间隙：CS 拉高若干拍
            //----------------------------------------------------------
            ST_GAP: begin
                if (gap_cnt == 8'd7) begin
                    gap_cnt <= 8'd0;
                    state   <= gap_next;
                end else begin
                    gap_cnt <= gap_cnt + 8'd1;
                end
            end

            //----------------------------------------------------------
            //  页编程
            //----------------------------------------------------------
            ST_PP_CMD:  if (byte_done) state <= ST_PP_ADDR;

            ST_PP_ADDR: if (byte_done && (byte_idx == 8'd2)) begin
                // 本页还能写多少字节 = 256 - (地址低 8 位)
                page_left <= 16'd256 - {8'd0, cur_addr[7:0]};
                state     <= ST_PP_DATA;
            end

            ST_PP_DATA: if (byte_done) begin
                if (({8'd0, byte_idx} + 16'd1) >= page_left) begin
                    // 本页写满
                    gap_next <= ST_RDSR_CMD;
                    state    <= ST_GAP;
                end
            end

            //----------------------------------------------------------
            //  读状态寄存器，等 WIP 清零
            //----------------------------------------------------------
            ST_RDSR_CMD: if (byte_done) state <= ST_RDSR_RD;

            ST_RDSR_RD: if (byte_done) begin
                sr_val <= rx_shift;
                if (rx_shift[0] == 1'b1) begin
                    // 还在忙，隔一会儿再查
                    gap_next <= ST_RDSR_CMD;
                    state    <= ST_GAP;
                end else begin
                    // 本次页编程/擦除完成
                    if (op_r == OP_WRITE) begin
                        cur_addr <= cur_addr + page_left;
                        remain   <= remain - page_left;
                        if (remain > page_left) begin
                            gap_next <= ST_WREN;      // 还有数据，写下一页
                            state    <= ST_GAP;
                        end else begin
                            state <= ST_DONE;
                        end
                    end else begin
                        state <= ST_DONE;             // 擦除完成
                    end
                end
            end

            //----------------------------------------------------------
            ST_CE_CMD: if (byte_done) begin
                gap_next <= ST_RDSR_CMD;
                state    <= ST_GAP;
            end

            //----------------------------------------------------------
            ST_DONE: begin
                done  <= 1'b1;
                busy  <= 1'b0;
                state <= ST_IDLE;
            end

            default: state <= ST_IDLE;
        endcase
    end
end

//==============================================================================
//  输出连线
//==============================================================================
assign flash_sclk    = sclk_r;
assign flash_mosi    = tx_shift[7];
assign flash_wp_n    = 1'b1;      // WP#  低有效 → 恒高
assign flash_hold_n  = 1'b1;      // HOLD# 低有效 → 恒高

endmodule
