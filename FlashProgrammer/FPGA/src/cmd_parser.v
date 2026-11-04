//==============================================================================
//  cmd_parser.v —— 串口命令解析 / 执行 / 响应组帧
//==============================================================================
//
//  【职责】
//  解析上位机发来的帧 → 驱动 spi_flash_ctrl 干活 → 把结果组帧发回去。
//  内部带一块 4KB 缓冲（综合器推断为 BRAM）：
//    · 写操作：先收进缓冲，再喂给 Flash 控制器
//    · 读操作：先把 Flash 读出的数据存进缓冲，再慢慢通过串口发出去
//      （Flash 读出约 1.5MB/s，串口只有 92KB/s，必须缓冲）
//
//  【帧格式】（多字节一律小端）
//    请求： AA 55 | CMD(1) | ADDR(4) | LEN(4) | DATA(LEN) | CRC16(2)
//    响应： AA 55 | STATUS(1) | LEN(4) | DATA(LEN) | CRC16(2)
//    CRC16 = CRC-16/CCITT-FALSE，初值 0xFFFF，多项式 0x1021，覆盖 CMD..DATA
//
//  【命令表】
//    0x01 READ_ID     读 JEDEC ID  → 3 字节
//    0x02 READ        读 Flash     → LEN 字节
//    0x03 WRITE       写 Flash     ← LEN 字节
//    0x04 CHIP_ERASE  整片擦除     → 0 字节（典型 20~100 秒）
//    0x05 PING        回环测试     → 1 字节 0x55
//
//  【STATUS】0x00 OK / 0x01 CRC 错 / 0x02 命令不支持 / 0x03 长度非法
//
//  【注意】LEN 上限 = 缓冲大小 4096 字节，上位机需自行分块。
//
//==============================================================================

module cmd_parser #(
    parameter CLK_FREQ = 50000000,
    parameter BUF_AW   = 12              // 缓冲地址位宽：12 → 4096 字节
)(
    input  wire        clk,
    input  wire        rst_n,

    //------------------------------------------------------------------
    //  UART 接收（接 uart_rx 的 AXI-Stream 输出）
    //------------------------------------------------------------------
    input  wire        rx_tvalid,
    input  wire [7:0]  rx_tdata,
    output wire        rx_tready,

    //------------------------------------------------------------------
    //  UART 发送（接 uart_tx 的 AXI-Stream 输入）
    //------------------------------------------------------------------
    output reg         tx_tvalid,
    output reg  [7:0]  tx_tdata,
    input  wire        tx_tready,

    //------------------------------------------------------------------
    //  Flash 控制器
    //------------------------------------------------------------------
    output reg  [2:0]  fl_op,
    output reg         fl_start,
    output reg  [23:0] fl_addr,
    output reg  [15:0] fl_len,
    input  wire        fl_busy,
    input  wire        fl_done,
    input  wire [7:0]  fl_rdata,
    input  wire        fl_rvalid,
    input  wire [11:0] fl_waddr,        // 输入：控制器告诉我们它要第几个字节
    output wire [7:0]  fl_wdata,        // 输出：缓冲把该字节还给它
    input  wire [7:0]  fl_sr,

    //------------------------------------------------------------------
    //  调试状态灯
    //------------------------------------------------------------------
    output reg  [3:0]  led_state
);

//==============================================================================
//  命令 / 状态编码
//==============================================================================
localparam [7:0] CMD_READ_ID    = 8'h01;
localparam [7:0] CMD_READ       = 8'h02;
localparam [7:0] CMD_WRITE      = 8'h03;
localparam [7:0] CMD_CHIP_ERASE = 8'h04;
localparam [7:0] CMD_PING       = 8'h05;

localparam [7:0] ST_OK      = 8'h00;
localparam [7:0] ST_CRC_ERR = 8'h01;
localparam [7:0] ST_BAD_CMD = 8'h02;
localparam [7:0] ST_BAD_LEN = 8'h03;

//==============================================================================
//  CRC-16/CCITT-FALSE
//==============================================================================
function [15:0] crc16_byte;
    input [15:0] crc;
    input [7:0]  din;
    reg   [15:0] c;
    integer i;
    begin
        c = crc ^ {din, 8'h00};
        for (i = 0; i < 8; i = i + 1)
            c = c[15] ? ((c << 1) ^ 16'h1021) : (c << 1);
        crc16_byte = c;
    end
endfunction

//==============================================================================
//  状态编码
//==============================================================================
localparam [2:0] S_RX   = 3'd0;
localparam [2:0] S_EXEC = 3'd1;
localparam [2:0] S_WAIT = 3'd2;
localparam [2:0] S_TX   = 3'd3;

localparam [3:0] R_H0    = 4'd0;
localparam [3:0] R_H1    = 4'd1;
localparam [3:0] R_CMD   = 4'd2;
localparam [3:0] R_ADDR  = 4'd3;
localparam [3:0] R_LEN   = 4'd4;
localparam [3:0] R_DATA  = 4'd5;
localparam [3:0] R_CRCL  = 4'd6;
localparam [3:0] R_CRCH  = 4'd7;

localparam [3:0] T_H0    = 4'd0;
localparam [3:0] T_H1    = 4'd1;
localparam [3:0] T_ST    = 4'd2;
localparam [3:0] T_LEN   = 4'd3;
localparam [3:0] T_PRE   = 4'd4;
localparam [3:0] T_DATA  = 4'd5;
localparam [3:0] T_CRCL  = 4'd6;
localparam [3:0] T_CRCH  = 4'd7;
localparam [3:0] T_DONE  = 4'd8;    // 等 uart_tx 取走 CRC 高字节后再撤 tvalid

//==============================================================================
//  4KB 缓冲（简单双口：1 写口 + 1 读口）
//==============================================================================
localparam BUF_DEPTH = (1 << BUF_AW);
reg  [7:0] sbuf [0:BUF_DEPTH-1];        // 注意：不能叫 buf，buf 是 Verilog 保留原语

reg  [BUF_AW-1:0] buf_waddr;
reg  [7:0]        buf_wdata;
reg               buf_we;
reg  [BUF_AW-1:0] buf_raddr;
reg  [7:0]        buf_rdata;

// 读地址：写操作期间看控制器的请求，其余时间看发送指针
wire [BUF_AW-1:0] rd_addr_mux;      // 赋值在 main_state 声明之后

always @(posedge clk) begin
    if (buf_we)
        sbuf[buf_waddr] <= buf_wdata;
    buf_rdata <= sbuf[rd_addr_mux];
end

assign fl_wdata = buf_rdata;

//==============================================================================
//  寄存器
//==============================================================================
reg [2:0]  main_state;
reg [3:0]  rx_state;
reg [3:0]  tx_state;

// 读地址选择：写操作期间看控制器的预取请求，其余时间看发送指针
assign rd_addr_mux = (main_state == S_WAIT) ? fl_waddr : buf_raddr;

// 请求帧
reg [7:0]  req_cmd;
reg [31:0] req_addr;
reg [31:0] req_len;
reg [3:0]  rx_byte_idx;
reg [11:0] rx_data_cnt;
reg [15:0] rx_crc;
reg [7:0]  rx_crc_lo;
reg        frame_ready;
reg        rx_crc_pass;

// 响应帧
reg [7:0]  resp_status;
reg [31:0] resp_len;
reg [3:0]  tx_byte_idx;
reg [15:0] tx_crc;
reg [11:0] tx_data_cnt;

// Flash 读结果计数
reg [11:0] fl_rd_cnt;

//==============================================================================
//  UART 接收：永远准备好
//==============================================================================
assign rx_tready = 1'b1;
wire   rx_hs = rx_tvalid & rx_tready;

//==============================================================================
//  接收状态机
//==============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state    <= R_H0;
        req_cmd     <= 8'd0;
        req_addr    <= 32'd0;
        req_len     <= 32'd0;
        rx_byte_idx <= 4'd0;
        rx_data_cnt <= 12'd0;
        rx_crc      <= 16'hFFFF;
        rx_crc_lo   <= 8'd0;
    end else if (rx_hs) begin
        case (rx_state)
            R_H0: begin
                if (rx_tdata == 8'hAA) rx_state <= R_H1;
            end

            R_H1: begin
                if      (rx_tdata == 8'h55) rx_state <= R_CMD;
                else if (rx_tdata == 8'hAA) rx_state <= R_H1;
                else                        rx_state <= R_H0;
            end

            R_CMD: begin
                req_cmd     <= rx_tdata;
                rx_crc      <= crc16_byte(16'hFFFF, rx_tdata);
                rx_byte_idx <= 4'd0;
                rx_state    <= R_ADDR;
            end

            R_ADDR: begin
                req_addr[8*rx_byte_idx +: 8] <= rx_tdata;
                rx_crc <= crc16_byte(rx_crc, rx_tdata);
                if (rx_byte_idx == 4'd3) begin
                    rx_byte_idx <= 4'd0;
                    rx_state    <= R_LEN;
                end else begin
                    rx_byte_idx <= rx_byte_idx + 4'd1;
                end
            end

            R_LEN: begin
                req_len[8*rx_byte_idx +: 8] <= rx_tdata;
                rx_crc <= crc16_byte(rx_crc, rx_tdata);
                if (rx_byte_idx == 4'd3) begin
                    rx_byte_idx <= 4'd0;
                    rx_data_cnt <= 12'd0;
                    // 关键：只有 WRITE 命令的请求帧才带数据段。
                    // READ 的 LEN 是"要读多少字节"，帧里没有 DATA；
                    // PING / READ_ID / CHIP_ERASE 的 LEN 恒为 0。
                    // 若不加这个判断，READ 会傻等 LEN 个字节，直接把状态机卡死。
                    if ((req_cmd == CMD_WRITE) &&
                        !((rx_tdata == 8'd0) && (req_len[23:0] == 24'd0)))
                        rx_state <= R_DATA;
                    else
                        rx_state <= R_CRCL;
                end else begin
                    rx_byte_idx <= rx_byte_idx + 4'd1;
                end
            end

            R_DATA: begin
                rx_crc <= crc16_byte(rx_crc, rx_tdata);
                // 必须用 13 位比较：4096 是合法长度，但它的低 12 位正好是 0，
                // 若只比 req_len[11:0]，收到第 1 个字节就会误判成"收完了"。
                if (({1'b0, rx_data_cnt} + 13'd1) >= req_len[12:0]) begin
                    rx_data_cnt <= 12'd0;
                    rx_state    <= R_CRCL;
                end else begin
                    rx_data_cnt <= rx_data_cnt + 12'd1;
                end
            end

            R_CRCL: begin
                rx_crc_lo <= rx_tdata;
                rx_state  <= R_CRCH;
            end

            R_CRCH: begin
                rx_state <= R_H0;
            end

            default: rx_state <= R_H0;
        endcase
    end
end

// CRC 校验：在收到 CRC 高字节那一拍锁存结果。
// 注意必须用寄存器锁存——frame_ready 是寄存器（晚一拍），若这里用组合逻辑，
// 下一帧的首字节 AA 一到就会把 rx_tdata 冲掉，导致 CRC 误判。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rx_crc_pass <= 1'b0;
    else if (rx_hs && (rx_state == R_CRCH))
        rx_crc_pass <= (rx_crc == {rx_tdata, rx_crc_lo});
end

// 帧结束脉冲
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        frame_ready <= 1'b0;
    else if (main_state == S_RX && rx_state == R_CRCH && rx_hs)
        frame_ready <= 1'b1;
    else if (main_state == S_TX && tx_state == T_CRCH && tx_tvalid && tx_tready)
        frame_ready <= 1'b0;
end

//==============================================================================
//  缓冲写口（接收数据 / Flash 读结果，两者不会同时发生）
//==============================================================================
//  只有"读类"操作的结果才允许进缓冲：读 Flash 数据(fl_op=1) / 读 ID(fl_op=0)。
//  写(2) 和擦除(3) 期间 Flash 控制器本不该产生数据 —— 但万一产生了（例如
//  内部轮询状态寄存器时把状态字节也当成数据吐出来），这里也要挡掉。
//  否则这些字节会按递增下标把**待写入的数据**覆盖掉，而且覆盖位置随时间
//  线性推进，症状是"每页开头一截变错、第 0 页永远正常"，极难排查。
//==============================================================================
wire fl_buf_ok = (fl_op == 3'd1) || (fl_op == 3'd0);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        buf_we    <= 1'b0;
        buf_waddr <= 0;
        buf_wdata <= 8'd0;
    end else begin
        buf_we <= 1'b0;
        if (rx_hs && (rx_state == R_DATA)) begin
            buf_we    <= 1'b1;
            buf_waddr <= rx_data_cnt[BUF_AW-1:0];
            buf_wdata <= rx_tdata;
        end else if (fl_rvalid && fl_buf_ok) begin
            buf_we    <= 1'b1;
            buf_waddr <= fl_rd_cnt[BUF_AW-1:0];
            buf_wdata <= fl_rdata;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        fl_rd_cnt <= 12'd0;
    else if (main_state == S_EXEC)
        fl_rd_cnt <= 12'd0;
    else if (fl_rvalid && fl_buf_ok)
        fl_rd_cnt <= fl_rd_cnt + 12'd1;
end

//==============================================================================
//  主流程状态机
//==============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        main_state <= S_RX;
        fl_op      <= 3'd0;
        fl_start   <= 1'b0;
        fl_addr    <= 24'd0;
        fl_len     <= 16'd0;
        resp_status<= ST_OK;
        resp_len   <= 32'd0;
    end else begin
        fl_start <= 1'b0;

        case (main_state)
            //----------------------------------------------------------
            S_RX: if (frame_ready) begin
                if (!rx_crc_pass) begin
                    resp_status <= ST_CRC_ERR;
                    resp_len    <= 32'd0;
                    main_state  <= S_TX;
                end else begin
                    resp_status <= ST_OK;
                    fl_addr     <= req_addr[23:0];
                    fl_len      <= req_len[15:0];
                    case (req_cmd)
                        CMD_READ_ID: begin
                            fl_op      <= 3'd0;
                            resp_len   <= 32'd3;
                            main_state <= S_EXEC;
                        end
                        CMD_READ: begin
                            // 合法长度：1 ~ 4096（缓冲大小）。
                            // 注意不能用 req_len[31:12] 判上限——4096 的高位就是 1，
                            // 那样会把合法的一整块也拒掉。
                            if ((req_len == 32'd0) || (req_len > 32'd4096)) begin
                                resp_status <= ST_BAD_LEN;
                                resp_len    <= 32'd0;
                                main_state  <= S_TX;
                            end else begin
                                fl_op      <= 3'd1;
                                resp_len   <= req_len;
                                main_state <= S_EXEC;
                            end
                        end
                        CMD_WRITE: begin
                            if ((req_len == 32'd0) || (req_len > 32'd4096)) begin
                                resp_status <= ST_BAD_LEN;
                                resp_len    <= 32'd0;
                                main_state  <= S_TX;
                            end else begin
                                fl_op      <= 3'd2;
                                resp_len   <= 32'd0;
                                main_state <= S_EXEC;
                            end
                        end
                        CMD_CHIP_ERASE: begin
                            fl_op      <= 3'd3;
                            resp_len   <= 32'd0;
                            main_state <= S_EXEC;
                        end
                        CMD_PING: begin
                            resp_len   <= 32'd1;
                            main_state <= S_TX;
                        end
                        default: begin
                            resp_status <= ST_BAD_CMD;
                            resp_len    <= 32'd0;
                            main_state  <= S_TX;
                        end
                    endcase
                end
            end

            //----------------------------------------------------------
            S_EXEC: begin
                fl_start   <= 1'b1;
                main_state <= S_WAIT;
            end

            //----------------------------------------------------------
            S_WAIT: if (fl_done) main_state <= S_TX;

            //----------------------------------------------------------
            //  响应发完即退出。
            //  注意：不能写成 (tx_state==T_H0 && !tx_tvalid)，否则 tx 状态机
            //  刚回到 T_H0 时 main_state 还是 S_TX，T_H0 会误判为"新一轮"，
            //  把同一帧响应再发一遍。这里用 T_CRCH 的握手作为结束标志。
            //----------------------------------------------------------
            S_TX: if (tx_state == T_DONE && tx_tvalid && tx_tready)
                      main_state <= S_RX;

            default: main_state <= S_RX;
        endcase
    end
end

//==============================================================================
//  发送状态机
//==============================================================================
wire tx_hs = tx_tvalid & tx_tready;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_state    <= T_H0;
        tx_tvalid   <= 1'b0;
        tx_tdata    <= 8'd0;
        tx_byte_idx <= 4'd0;
        tx_crc      <= 16'hFFFF;
        tx_data_cnt <= 12'd0;
        buf_raddr   <= 0;
    end else begin
        case (tx_state)
            // 等主流程进入 S_TX
            T_H0: if (main_state == S_TX) begin
                tx_tvalid <= 1'b1;
                tx_tdata  <= 8'hAA;
                tx_crc    <= 16'hFFFF;
                tx_state  <= T_H1;
            end

            T_H1: if (tx_hs) begin
                tx_tdata <= 8'h55;
                tx_state <= T_ST;
            end

            T_ST: if (tx_hs) begin
                tx_tdata    <= resp_status;
                tx_crc      <= crc16_byte(16'hFFFF, resp_status);
                tx_byte_idx <= 4'd0;
                tx_state    <= T_LEN;
            end

            T_LEN: if (tx_hs) begin
                tx_tdata <= resp_len[8*tx_byte_idx +: 8];
                tx_crc   <= crc16_byte(tx_crc, resp_len[8*tx_byte_idx +: 8]);
                if (tx_byte_idx == 4'd3) begin
                    tx_byte_idx <= 4'd0;
                    tx_data_cnt <= 12'd0;
                    buf_raddr   <= 0;
                    tx_state    <= (resp_len == 32'd0) ? T_CRCL : T_PRE;
                end else begin
                    tx_byte_idx <= tx_byte_idx + 4'd1;
                end
            end

            // 预取第一个字节（同步读 BRAM 需要 1 拍）
            T_PRE: tx_state <= T_DATA;

            T_DATA: if (tx_hs) begin
                tx_tdata  <= (req_cmd == CMD_PING) ? 8'h55 : buf_rdata;
                tx_crc    <= crc16_byte(tx_crc, (req_cmd == CMD_PING) ? 8'h55 : buf_rdata);
                buf_raddr <= tx_data_cnt[BUF_AW-1:0] + 1'b1;
                // 同 R_DATA：必须用 13 位比较。resp_len=4096 时低 12 位为 0，
                // 若只比 resp_len[11:0]，发完第 1 个字节就会提前结束，上位机永远收不齐。
                if (({1'b0, tx_data_cnt} + 13'd1) >= resp_len[12:0]) begin
                    tx_data_cnt <= 12'd0;
                    tx_state    <= T_CRCL;
                end else begin
                    tx_data_cnt <= tx_data_cnt + 12'd1;
                end
            end

            T_CRCL: if (tx_hs) begin
                tx_tdata <= tx_crc[7:0];
                tx_state <= T_CRCH;
            end

            // 注意：T_CRCH 期间 tx_tdata 还是 CRC 低字节（uart_tx 在这拍才锁存它），
            // 所以这里装的是"下一个"字节 = CRC 高字节。必须再多等一个状态，
            // 否则 tx_tvalid 会跟着一起撤销，CRC 高字节就永远发不出去。
            T_CRCH: if (tx_hs) begin
                tx_tdata <= tx_crc[15:8];
                tx_state <= T_DONE;
            end

            // 等 uart_tx 把 CRC 高字节取走，再撤销 tvalid
            T_DONE: if (tx_hs) begin
                tx_tvalid <= 1'b0;
                tx_state  <= T_H0;
            end

            default: tx_state <= T_H0;
        endcase
    end
end

//==============================================================================
//  状态灯
//==============================================================================
always @(*) begin
    case (main_state)
        S_RX:    led_state = 4'b0001;
        S_EXEC:  led_state = 4'b0010;
        S_WAIT:  led_state = 4'b0100;
        S_TX:    led_state = 4'b1000;
        default: led_state = 4'b0000;
    endcase
end

endmodule
