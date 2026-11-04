//==============================================================================
//  tb_view.v —— bmp_view 的逐行对齐回归（专抓"2 倍放大时同一源行的两次冲刷不一致"）
//==============================================================================
//  源：320×240，每行相同（竖条纹，周期 8）。目标：640×480（step=512，2 倍放大）。
//  因此**输出每两行必须完全相同**（同一源行被输出两次）。
//
//  判据：
//    · 每一行的内容列起点必须相同（= 窗口左沿）
//    · 行 2k 与行 2k+1 必须逐字相同
//
//  跑法：
//    iverilog -g2012 -o tv.vvp -s tb_view tb_view.v bmp_view.v bmp_view_fill.v
//    vvp tv.vvp
//==============================================================================
`timescale 1ns / 1ps
module tb_view;

localparam integer SW = 320;      // 源宽
localparam integer SH = 240;      // 源高
localparam integer CW = 640;      // 画布宽（888，1 像素 1 字）
localparam integer CH = 480;

parameter integer FMT   = 1;      // 1 = 888
parameter integer STALL = 0;      // 非 0：每 STALL 拍拉一次 stall

reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg         start = 0;
reg  [1:0]  mode  = 2'd0;
reg  [20:0] dst_base = 21'd0;
reg  [10:0] dst_stride = 11'd0;
reg  [7:0]  zoom  = 8'h10;        // 1.0 = fit
reg  [9:0]  ax = 10'd320, ay = 10'd240;
reg         stall = 0;

wire        pix_rd;
reg         pix_vld = 1;
wire [11:0] row_req, win_x0;
wire [12:0] win_w;
wire        row_go, cont1;
wire        we;
wire [20:0] wa;
wire [31:0] wdata;
wire        busy, done;

reg [23:0] pix;

// 源像素：周期 8 的竖条纹（每行都相同）
function [23:0] src_px;
    input integer c;
    begin
        src_px = ((c % 8) < 4) ? 24'hFFFFFF : 24'h000000;
    end
endfunction

// 填充计数：fill_start 清零，每个 pix_rd 前进一格
integer fcnt = 0;
always @(posedge clk or posedge rst) begin
    if (rst) fcnt <= 0;
    else if (dut.fill_start) fcnt <= 0;
    else if (pix_rd) fcnt <= fcnt + 1;
end
always @(*) pix = src_px(dut.x_le + fcnt);

// 反压
integer scnt = 0;
always @(posedge clk) begin
    scnt <= scnt + 1;
    stall <= (STALL != 0) && ((scnt % STALL) == (STALL-1));
end

bmp_view dut (
    .clk(clk), .rst(rst),
    .start(start), .src_w(SW[15:0]), .src_h(SH[15:0]),
    .mode(mode), .fmt(FMT[0]), .dst_base(dst_base), .dst_stride(dst_stride),
    .zoom(zoom), .ax(ax), .ay(ay),
    .pix_vld(pix_vld), .pix(pix), .pix_rd(pix_rd),
    .row_req(row_req), .win_x0(win_x0), .win_w(win_w), .row_go(row_go), .cont1(cont1),
    .stall(stall), .we(we), .wa(wa), .wdata(wdata), .busy(busy), .done(done)
);

// 收写口
localparam integer MAXW = 400000;
reg [31:0] mem [0:MAXW-1];
integer i;
initial for (i = 0; i < MAXW; i = i + 1) mem[i] = 32'hDEAD_BEEF;
always @(posedge clk) if (we && wa < MAXW) mem[wa] <= wdata;

integer wn = 0;
always @(posedge clk) if (we) wn <= wn + 1;

integer k;
initial begin
    $display("== tb_view: %0dx%0d -> %0dx%0d, FMT=%0d STALL=%0d ==", SW, SH, CW, CH, FMT, STALL);
    repeat (5) @(posedge clk);
    rst = 0;
    repeat (5) @(posedge clk);
    start = 1;
    @(posedge clk);
    start = 0;
    for (k = 0; k < 3_000_000; k = k + 1) begin
        @(posedge clk);
        if (done) k = 3_000_000;
    end
    $display("  [tb] done=%b 写 %0d 字", done, wn);
    begin : cmp
        integer r, c, bad, badrow, w0, wexp;
        reg [31:0] a, b;
        bad = 0; badrow = -1;
        // 1) 相邻两行是否逐字相同
        for (r = 0; r + 1 < CH; r = r + 2) begin
            for (c = 0; c < CW; c = c + 1) begin
                a = mem[r*CW + c];
                b = mem[(r+1)*CW + c];
                if (a !== b) begin
                    bad = bad + 1;
                    if (badrow < 0) begin
                        badrow = r;
                        $display("  [首个不同] 行 %0d/%0d 列 %0d: %h vs %h", r, r+1, c, a, b);
                    end
                end
            end
        end
        $display("  [逐对] 行 2k 与 2k+1 不同的字数 = %0d（首行 %0d）", bad, badrow);
        // 2) 每行的内容起点（第一个非黑字）
        w0 = -1;
        for (r = 0; r < 8; r = r + 1) begin
            wexp = -1;
            for (c = 0; c < CW; c = c + 1)
                if (wexp < 0 && mem[r*CW+c][23:0] != 24'd0) wexp = c;
            $display("    行 %0d 首个非黑列 = %0d", r, wexp);
            if (r == 0) w0 = wexp;
            else if (wexp != w0 && bad >= 0) bad = bad + 1;
        end
        if (bad == 0) $display("  PASS：所有 2k/2k+1 行对逐字相同");
        else          $display("  FAIL：%0d 处不一致", bad);
    end
    $finish;
end

endmodule
