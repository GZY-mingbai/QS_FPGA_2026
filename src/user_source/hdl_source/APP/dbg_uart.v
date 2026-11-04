//==============================================================================
//  dbg_uart.v —— 像素轨迹串口回传（帧缓存写入侧）
//==============================================================================
//  【用途】把"真正要写进帧缓存的那一拍"整条抓下来，通过串口回传。
//      一条 64 位 = 一个像素的全部上下文：
//          [63:48] px       屏幕列
//          [47:32] py       屏幕行
//          [31:8]  data     ps_data[23:0]（要写进帧缓存的 RGB）
//          [7]     take     本拍接受了一个像素
//          [6]     ps_valid
//          [5]     write_en
//          [4]     pipe_ce
//          [3:0]   保留
//
//  【协议】115200 8N1，每帧一段文本：
//      N=<2 位十六进制>          ← 本次抓到几条（0 也发，验证串口通）
//      <16 位十六进制>           ← 每条一个像素，高位在前
//
//  【为什么用环形 64 深】一帧 640 个像素，RAM 只有 64 格。用环形指针
//      （6 位，自然回绕 0..63），帧末 RAM 里就是**这一行最后 64 个像素**。
//      不需要任何"边界/窗口"计算（那类写法错过好几次，且失败时一条都采不到）。
//      读的时候按 px 出现"回落"的位置旋转即可。
//
//  【为什么每帧都发】帧长 16.7ms，而 64 条 × 17 字符 ≈ 94ms，追不上一帧。
//      ⇒ 发送期间 dumping=1 封锁写入，实际是"采一帧、发一帧"交替。
//      空帧（没采到东西）只发 N=00，一行，不占时间。
//==============================================================================

module dbg_uart #(
    parameter integer CLK_FREQ = 50_000_000
)(
    input  wire        clk,                 // 50MHz（采样与发送同域）
    input  wire        rst,                 // 高有效
    input  wire        frame,               // = ps_start
    input  wire        smp_en,              // 采一个样本
    input  wire [63:0] smp_data,            // 见文件头格式
    output wire        uart_tx
);

localparam integer DEPTH = 64;
localparam [7:0]   CR = 8'h0D;
localparam [7:0]   LF = 8'h0A;

//==============================================================================
//  ① 帧下降沿
//==============================================================================
reg fr_d;
always @(posedge clk or posedge rst) begin
    if (rst) fr_d <= 1'b0;
    else     fr_d <= frame;
end
wire frame_end = fr_d && !frame;

// ★★ 2026-10-24：**与帧无关的定时触发**，保证串口一定有输出 ★★
//  上板反复出现"完全无输出"，说明 frame_end 这条链不可靠。
//  现在：2^22 拍 ≈ 84ms 自动发一块（即使 N=00 也能确认串口活着）。
reg [21:0] tdiv;
always @(posedge clk or posedge rst) begin
    if (rst) tdiv <= 22'd0;
    else     tdiv <= tdiv + 22'd1;
end
wire dump_start = frame_end | (tdiv == 22'd0);

//==============================================================================
//  ② 环形抓取
//==============================================================================
(* ram_style = "block" *) (* ramstyle = "block" *) (* syn_ramstyle = "block" *)
reg [63:0] mem [0:DEPTH-1];

reg [5:0] wptr;
reg [7:0] cnt;          // 帧末锁存的待发条数
reg       any;          // 本帧抓到过东西
reg       dumping;      // 正在发送（期间禁止写）

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wptr <= 6'd0;
        cnt  <= 8'd0;
        any  <= 1'b0;
    end else if (frame_end) begin
        cnt  <= any ? 8'd64 : 8'd0;     // 抓到过就发满 64 条
        any  <= 1'b0;
        wptr <= 6'd0;
    end else if (frame && !dumping && smp_en) begin
        mem[wptr] <= smp_data;
        wptr      <= wptr + 6'd1;       // 6 位自然回绕 0..63
        any       <= 1'b1;
    end
end

//==============================================================================
//  ③ 发送 FSM
//==============================================================================
localparam [1:0] S_IDLE = 2'd0, S_HDR = 2'd1, S_LINE = 2'd2;

reg [1:0] state;
reg [5:0] ent;
reg [4:0] k;

wire [63:0] mdata = mem[ent];

function [7:0] hx;
    input [3:0] v;
    begin
        hx = (v < 4'd10) ? (8'h30 + {4'd0, v}) : (8'h37 + {4'd0, v});
    end
endfunction

reg [7:0] tbyte;
always @(*) begin
    tbyte = LF;
    if (state == S_HDR) begin
        case (k)
            5'd0: tbyte = "N";
            5'd1: tbyte = "=";
            5'd2: tbyte = hx(cnt[7:4]);
            5'd3: tbyte = hx(cnt[3:0]);
            5'd4: tbyte = CR;
            default: tbyte = LF;
        endcase
    end else if (state == S_LINE) begin
        if (k < 5'd16)       tbyte = hx(mdata[4 * (4'd15 - k[3:0]) +: 4]);
        else if (k == 5'd16) tbyte = CR;
        else                 tbyte = LF;
    end
end

wire tx_ready;
wire tx_done = tx_ready && (state != S_IDLE);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state   <= S_IDLE;
        ent     <= 6'd0;
        k       <= 5'd0;
        dumping <= 1'b0;
    end else begin
        dumping <= (state != S_IDLE);
        case (state)
            S_IDLE: begin
                if (dump_start) begin
                    ent   <= 6'd0;
                    k     <= 5'd0;
                    state <= S_HDR;                 // 0 条也发，验证串口通
                end
            end
            S_HDR: begin
                if (tx_done) begin
                    if (k == 5'd5) begin
                        k     <= 5'd0;
                        ent   <= 6'd0;
                        state <= (cnt == 8'd0) ? S_IDLE : S_LINE;
                    end else k <= k + 5'd1;
                end
            end
            S_LINE: begin
                if (tx_done) begin
                    if (k == 5'd17) begin
                        k <= 5'd0;
                        // ★★ ★ 2026-10-24：不能写 `cnt[5:0]`——cnt=64 截成 6 位 = 0
                        //   ⇒ (ent+1)>=0 恒真 ⇒ 每块只发 1 行（上板就是 N=40 后面只跟 1 行）。
                        if (({2'b00, ent} + 8'd1) >= cnt) state <= S_IDLE;
                        else                          ent   <= ent + 6'd1;
                    end else k <= k + 5'd1;
                end
            end
            default: state <= S_IDLE;
        endcase
    end
end

//==============================================================================
//  ④ 串口发送（搬运自 FlashProgrammer，GPLv3，见同目录 LICENSE）
//==============================================================================
uart_tx #(
    .CLK_FREQ   (CLK_FREQ),
    .BAUD_RATE  (115200),
    .PARITY     ("NONE"),
    .STOP_BITS  (1),
    .BYTE_WIDTH (1),
    .FIFO_EA    (0)
) u_tx (
    .rstn      (~rst),
    .clk       (clk),
    .i_tready  (tx_ready),
    .i_tvalid  (state != S_IDLE),
    .i_tdata   (tbyte),
    .i_tkeep   (1'b1),
    .i_tlast   (1'b0),
    .o_uart_tx (uart_tx)
);

endmodule
