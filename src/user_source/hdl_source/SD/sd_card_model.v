//==============================================================================
//  sd_card_model.v —— SD/TF 卡的 SPI 从机行为模型（★ 只用于仿真 ★）
//==============================================================================
//
//  ★★★ 这个文件【只用于仿真】，绝对不要加进 .al 工程 ★★★
//  （.al 里要标 AutoExcluded="true"，和 w25q64_model_24b.v 一样）
//
//  【它解决什么问题】
//  工程里 sd_card_top 驱动的是**真卡**，仿真里没有卡 → 整条
//      SD → bmp_read → sd_bmp_loader → SDRAM
//  链路一步都跑不了。这个模型就是一张"电子假卡"：主机（spi_master）怎么发命令，
//  它就怎么响应，数据从一个真实的 BMP 卡镜像文件里取。
//
//  【模拟哪一部分协议】
//  只实现本工程真正用到的那几条命令 —— 够用就好，不做完整 SD 协议栈：
//      CMD0  (GO_IDLE_STATE)      → R1 = 0x01（还在 idle）
//      CMD8  (SEND_IF_COND)       → R1 = 0x01 + 4 字节回显 0x000001AA（R7）
//      CMD55 (APP_CMD)            → R1 = 0x01
//      ACMD41(SD_SEND_OP_COND)    → R1 = 0x00（立刻"初始化完成"）
//      CMD16 (SET_BLOCKLEN)       → R1 = 0x00
//      CMD17 (READ_SINGLE_BLOCK)  → R1 = 0x00 → 2×0xFF → 0xFE + 512 字节 + 2 字节 CRC
//      CMD24 (WRITE_BLOCK)        → R1 = 0x00 → 收 512 字节 → 数据响应 0x05 → 0xFF
//      其它命令                    → R1 = 0x00（不报非法，省事）
//
//  【为什么 ACMD41 直接回 0x00】
//  真卡初始化要几百毫秒到几秒，仿真里没必要等。sd_card_sec_read_write 收到 0x00
//  就认为"卡准备好了"，直接往下走（见该文件 S_CMD41 的分支 1）。
//
//  【SPI 时序】本工程 spi_master 用 CPOL=1 / CPHA=1（模式 3）：
//      · 空闲时 DCLK = 1
//      · 主机在**下降沿**更新 MOSI、在**上升沿**采样 MISO
//      · 所以从机反过来：**上升沿采样 MOSI、下降沿更新 MISO**
//  一个字节 = 8 个上升沿 + 8 个下降沿，字节边界正好落在下降沿上。
//  nCS 为高时（上电那 11 个 0xFF、以及每条命令前那个 0xFF）只当"字节边界
//  分隔符"，把字节级状态复位 —— 这样每条命令都从干净的 6 字节开始。
//
//  【卡的内容从哪来】
//  parameter INIT_FILE 指向一个"卡镜像"文件（由 tools/gen_sd_image.py 生成：
//  一张或多张真 BMP 按 512 字节对齐摆进去），用 $fread 一次性读进内部存储。
//  没写到的位置填 0xFF（真卡空白区也是 0xFF），所以"偏移填错"时会老老实实
//  读到 0xFF，而不是让仿真读到 x 崩掉。
//
//  【和真实卡的差别（重要）】
//      · 没有容量/CSD/CID 寄存器，不响应 CMD9/CMD10/CMD58（本工程不问）
//      · 不校验 CRC（真卡初始化阶段会校验 CMD0/CMD8 的 CRC —— 好在
//        sd_card_sec_read_write 填的 0x95 / 0x87 本来就是对的）
//      · 读写延迟 = 0（没有"卡内部写几毫秒"这回事）
//      · 不支持多块读（CMD18）
//
//  【★ 一个容易踩的坑：不要在两个 always 块里写同一批寄存器 ★】
//  本模型只有一个"字节级主时序块"（DCLK 下降沿 + nCS 上升沿），接收移位寄存器
//  rx 单独放在上升沿块里、只被那一个块写。这样 tools/check_verilog_drivers.py
//  的多驱动检查不会报警（同一批状态只出现在一个块里）。
//
//==============================================================================

module sd_card_model #(
    parameter integer MEM_BYTES = 2097152,      // 卡容量（字节）= 2MB，够放 2 张 640×480 BMP
    parameter         INIT_FILE = ""            // 卡镜像文件（相对工程根目录）
)(
    input  wire nCS,                            // 片选（低有效）
    input  wire DCLK,                           // SPI 时钟（空闲高）
    input  wire MOSI,                           // 主机 → 卡
    output reg  MISO                            // 卡 → 主机
);

//==============================================================================
//  0. 卡内存储
//==============================================================================
reg [7:0] mem [0:MEM_BYTES-1];

integer f_i, f_fd, f_n;
initial begin
    // 【为什么要逐个初始化】空位置必须是确定值 0xFF（真卡擦除态）。
    // 不初始化会让上层读到 x —— 那会污染整个仿真却又看不见原因。
    for (f_i = 0; f_i < MEM_BYTES; f_i = f_i + 1)
        mem[f_i] = 8'hFF;

    if (INIT_FILE != "") begin
        f_fd = $fopen(INIT_FILE, "rb");
        if (f_fd == 0) begin
            $display("  [sd_card_model] !! 打不开卡镜像 %s（卡内全 0xFF）", INIT_FILE);
        end else begin
            f_n = $fread(mem, f_fd);
            $fclose(f_fd);
            $display("  [sd_card_model] 卡镜像已装载 %0d 字节：%s", f_n, INIT_FILE);
        end
    end
end

//==============================================================================
//  1. 字节级状态
//==============================================================================
localparam CMD_LEN = 6;                     // 一条命令固定 6 字节

reg [7:0] cmd_buf [0:CMD_LEN-1];            // 命令拼装缓冲
integer   cmd_cnt;                          // 已收到几个命令字节（0..6）
reg       card_idle;                        // 卡是否还在 idle
integer   blk_len;                          // CMD16 设的块长度（本模型固定按 512 处理）
reg [31:0] rd_sector;                       // CMD17/CMD24 的扇区号

reg [7:0] resp [0:1039];                    // 响应队列（最长：R1 + 2 填充 + 0xFE + 512 + 2 CRC）
integer   resp_len;                         // 队列里有多少字节待发
integer   resp_idx;                         // 已经发出几个

integer   wr_cnt;                           // CMD24 已收字节数
reg       wr_busy;                          // CMD24 数据阶段

reg [2:0] npos;                             // 本字节内已过的下降沿数（0..7）
reg [7:0] tx;                               // 发送移位
reg [7:0] rx;                               // 接收移位（上升沿块独占）
reg [7:0] nt;                               // 本字节要发的数据（主时序块里用的临时寄存器）
reg       first;                            // CS 拉低后的第一个下降沿：还没有收到过一个字节
integer   rx_cnt;                           // 已收字节数（只用于限制调试打印的频率）

// ---- 本字节要发什么 ----
// ★★★ 必须用 function，不能用 always @(*) 的组合信号 ★★★
//  【踩过的坑】do_byte 里是**阻塞赋值**改 resp/resp_idx/resp_len，紧接着在同一个
//  时间步里要"立刻"把这个新队首发出去。如果写成
//        always @(*) nt = (resp_idx < resp_len) ? resp[resp_idx] : 8'hFF;
//  那么负沿块的 `tx <= nt` 读到的是**更新前**的老值（always @(*) 在同一个时间步里
//  可能排在负沿块之后才被唤醒）——表现就是：卡把 R1 响应发出去了，但主机永远收不到，
//  主机就一直在那里发 0xFF 空转（仿真表现为"初始化永远完不成"）。
//  function 是在调用的那一刻求值，读到的就是刚更新过的值 ✓
function [7:0] peek_tx;
    input dummy;                        // Verilog-2001 要求函数至少有一个输入端口
    begin
        dummy    = 1'b0;
        peek_tx  = (resp_idx < resp_len) ? resp[resp_idx] : 8'hFF;
    end
endfunction

//==============================================================================
//  2. 字节级动作（task，从主时序块调用）
//==============================================================================

// ---- 把某个扇区的 512 字节压进响应队列（CMD17 用）----
task push_sector;
    input [31:0] sec;
    input        with_token;                // 是否先发 2×0xFF + 0xFE
    integer      p;
    integer      base;
    begin
        base = sec * 512;
        // ★ 从 resp_len 处**追加**，不要从 0 开始覆盖 ★
        //   （覆盖会把我方刚排好的 R1 = 0x00 冲掉，主机就会看到 0xFF，
        //     进而把数据字节当成"错误响应"，表现为命令被无限重发）
        if (with_token) begin
            resp[resp_len] = 8'hFF; resp_len = resp_len + 1;   // 填充（真卡要花时间找地址）
            resp[resp_len] = 8'hFF; resp_len = resp_len + 1;
            resp[resp_len] = 8'hFE; resp_len = resp_len + 1;   // 数据起始令牌
        end
        for (p = 0; p < 512; p = p + 1) begin
            if ((base + p) < MEM_BYTES) resp[resp_len] = mem[base + p];
            else                        resp[resp_len] = 8'hFF;
            resp_len = resp_len + 1;
        end
        resp[resp_len] = 8'hFF; resp_len = resp_len + 1;   // CRC 低字节（本工程不校验）
        resp[resp_len] = 8'hFF; resp_len = resp_len + 1;   // CRC 高字节
    end
endtask

// ---- 收完一个字节：拼命令 / 收写数据 ----
task do_byte;
    input [7:0] b;
    integer      arg;
    begin
        if (cmd_cnt < CMD_LEN) begin
            // ---------- 阶段 1：拼命令 ----------
            cmd_buf[cmd_cnt] = b;
            cmd_cnt = cmd_cnt + 1;

            if (cmd_cnt == CMD_LEN) begin
                resp_len = 0;
                resp_idx = 0;

                // 参数（4 字节，大端）
                arg = (cmd_buf[1] << 24) | (cmd_buf[2] << 16) |
                      (cmd_buf[3] <<  8) |  cmd_buf[4];

                case (cmd_buf[0][5:0])
                // ---- CMD0：回 idle，回 0x01 ----
                6'd0:  begin
                    card_idle = 1'b1;
                    resp[0] = 8'h01;  resp_len = 1;
                    $display("  [SD] CMD0   →R1=01");
                end
                // ---- CMD8：回 0x01 + 4 字节回显（R7）----
                6'd8:  begin
                    resp[0] = 8'h01;
                    resp[1] = 8'h00; resp[2] = 8'h00;
                    resp[3] = 8'h01; resp[4] = 8'hAA;
                    resp_len = 5;
                    $display("  [SD] CMD8   →R1=01 + 000001AA");
                end
                // ---- CMD55：APP_CMD 前缀，回 0x01 ----
                6'd55: begin
                    resp[0] = 8'h01;  resp_len = 1;
                    $display("  [SD] CMD55  →R1=01");
                end
                // ---- ACMD41：直接"初始化完成"，回 0x00 ----
                6'd41: begin
                    card_idle = 1'b0;
                    resp[0] = 8'h00;  resp_len = 1;
                    $display("  [SD] ACMD41 →R1=00（卡就绪）");
                end
                // ---- CMD16：设置块长度，回 0x00 ----
                6'd16: begin
                    blk_len = arg;
                    resp[0] = 8'h00;  resp_len = 1;
                    $display("  [SD] CMD16  块长=%0d →R1=00", arg);
                end
                // ---- CMD17：单块读 ----
                6'd17: begin
                    rd_sector = arg[31:0];
                    resp[0]   = 8'h00;
                    resp_len  = 1;
                    push_sector(arg[31:0], 1'b1);
                    $display("  [SD] CMD17  扇区 %0d →R1=00 + 0xFE + 512 字节", arg);
                end
                // ---- CMD24：单块写（本工程不用，给个正确响应免得挂死）----
                6'd24: begin
                    rd_sector = arg[31:0];
                    wr_cnt    = 0;
                    wr_busy   = 1'b1;
                    resp[0]   = 8'h00;  resp_len = 1;
                    $display("  [SD] CMD24  扇区 %0d →R1=00（等数据）", arg);
                end
                // ---- 其它：一律回当前 R1 ----
                default: begin
                    resp[0] = card_idle ? 8'h01 : 8'h00;
                    resp_len = 1;
                    $display("  [SD] CMD%0d（未实现）→R1=%02x", cmd_buf[0][5:0], resp[0]);
                end
                endcase
            end
        end else if (wr_busy) begin
            // ---------- 阶段 2：CMD24 的数据（512 字节 + 2 字节 CRC）----------
            if (wr_cnt < 512) begin
                if ((rd_sector * 512 + wr_cnt) < MEM_BYTES)
                    mem[rd_sector * 512 + wr_cnt] = b;
                wr_cnt = wr_cnt + 1;
                if (wr_cnt == 512) begin
                    // 接下来主机发 2 字节 CRC，我回"数据响应 0x05 + 忙 + 空闲"
                    resp[0] = 8'hFF;                   // 对应主机的 CRC 低字节
                    resp[1] = 8'hFF;                   // 对应主机的 CRC 高字节
                    resp[2] = 8'h05;                   // 数据响应：0101 = 被接受
                    resp[3] = 8'h00;                   // 忙
                    resp[4] = 8'hFF;                   // 忙结束
                    resp_len = 5;
                end
            end else if (wr_cnt < 514) begin
                wr_cnt = wr_cnt + 1;
                if (wr_cnt == 514) wr_busy = 1'b0;
            end
        end
        // 否则：纯握手阶段（读数据时主机一直发 0xFF），响应队列自己往前走就行
    end
endtask

//==============================================================================
//  3. 物理时序
//==============================================================================
// 下降沿块（字节级主时序）：
//   npos==0 → 先把上一字节周期收满的那个字节处理掉，再装载下一个要发的字节
//   npos>0  → 发送移位
//   每个字节正好 8 个下降沿，npos 是 3 位自然回绕（7 → 0）
//==============================================================================
always @(negedge DCLK or posedge nCS) begin
    if (nCS) begin
        // 片选拉高 = 字节流断开：复位字节级状态
        // （上电那 11 个 0xFF、以及每条命令前那个 0xFF 都落在这里）
        npos     <= 3'd0;
        tx       <= 8'hFF;
        MISO     <= 1'b1;
        first    <= 1'b1;
        cmd_cnt  <= 0;
        resp_len = 0;   // ★ 2026-10-31：与其余赋值统一阻塞（Verilator 禁止混用）
        resp_idx <= 0;
        wr_busy  <= 1'b0;
        wr_cnt   <= 0;
        rx_cnt   <= 0;
    end else if (npos == 3'd0) begin
        if (!first) begin
            // 前若干个字节打一份原始字节流水（调试用；全量打会拖慢 1.8MB 的仿真）
            if (rx_cnt < 64) begin
`ifndef SD_SILENT
                $display("  [SD模型] rx#%0d = %02x", rx_cnt, rx);
`endif
                $fflush;
            end
            do_byte(rx);            // 上一个字节周期收到的字节（rx 已满 8 位）
            rx_cnt <= rx_cnt + 1;
        end
        first <= 1'b0;

        // 【注意】不能写 peek_tx()[7]（Verilog-2001 不允许对函数返回值做位选），
        // 先阻塞赋值给一个临时寄存器再取位。
        nt   = peek_tx(1'b0);
        tx   <= nt;                 // 本字节要发的数据（do_byte 之后的最新队列头）
        MISO <= nt[7];
        if (resp_idx < resp_len)
            resp_idx <= resp_idx + 1;
        npos <= 3'd1;
    end else begin
        tx   <= {tx[6:0], 1'b1};
        MISO <= tx[6];
        npos <= npos + 3'd1;
    end
end

//==============================================================================
//  4. 接收移位（上升沿采样 MOSI）
//==============================================================================
// 【为什么单独一个块】rx 只被这里写、被上面那个块读 —— 单一驱动，
//  check_verilog_drivers.py 的多驱动检查不会误报。
//==============================================================================
always @(posedge DCLK or posedge nCS) begin
    if (nCS) rx <= 8'hFF;
    else     rx <= {rx[6:0], MOSI};
end

endmodule
