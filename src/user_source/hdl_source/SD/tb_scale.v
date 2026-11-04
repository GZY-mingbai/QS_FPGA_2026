//==============================================================================
//  tb_scale.v —— bmp_scale 逐行一致性（专抓"装载时把纹理写歪"）
//==============================================================================
//  源图：每行都是同一条竖条纹（周期 32 源像素），因此**输出每一行必须逐字相同**。
//  任何一行写出不同的字序列 ⇒ 缩放器把纹理写歪了 ⇒ 屏幕上就是"抖动/锯齿"。
//
//  两种模式都测：fmt=1（888，640×480 一像素一字）和 fmt=0（565，320×240 打包）。
//==============================================================================
`timescale 1ns / 1ps
module tb_scale;

localparam integer SW = 320;      // 源宽
localparam integer SH = 240;      // 源高

reg clk = 0, rst = 1;
always #10 clk = ~clk;

parameter integer FMT = 1;        // 1 = 888  0 = 565
parameter integer GAP = 0;        // !=0：每 GAP 拍断一拍 pix_vld（模拟读卡断续）
parameter integer STRETCH = 1;

reg         start = 0;
reg  [15:0] src_w = SW, src_h = SH;
reg         pix_vld = 0;
reg  [23:0] pix = 0;
wire        pix_rd;
reg         stall = 0;
wire        we;
wire [20:0] woff;
wire [31:0] wdata;
wire        busy, done;

bmp_scale #(.CW(640), .CH(480)) dut (
    .clk(clk), .rst(rst), .start(start),
    .src_w(src_w), .src_h(src_h), .stretch(1'b1), .fmt(FMT[0]),
    .pix_vld(pix_vld), .pix(pix), .pix_rd(pix_rd),
    .stall(stall), .we(we), .woff(woff), .wdata(wdata),
    .busy(busy), .done(done)
);

// 源行：竖条纹（第 0..15 白，16..31 黑）
function [23:0] src_px;
    input integer x;
    begin
        src_px = ((x % 32) < 16) ? 24'hFFFFFF : 24'h000000;
    end
endfunction

// 源扫描（对应 bmp_read 的 pix_vld/pix）
//  GAP != 0 ⇒ 每 GAP 拍断一拍 pix_vld（模拟真实读卡"不是每拍都有字"）。
//  断拍时 pix **保持不变**（真实 bmp_read 也是保持），consumed 才前进。
integer sx, sy;
integer gcnt = 0;
reg     src_done = 0;
always @(posedge clk) if (!rst) gcnt <= (gcnt == 999_999) ? 0 : gcnt + 1;
wire vld_ok = (GAP == 0) ? 1'b1 : ((gcnt % GAP) != (GAP-1));

always @(posedge clk) begin
    if (rst) begin
        pix_vld <= 0; sx <= 0; sy <= 0; pix <= src_px(0); src_done <= 0;
    end else begin
        pix_vld <= ~src_done && vld_ok;
        if (pix_vld && pix_rd) begin
            if (sx == SW-1) begin
                sx <= 0;
                if (sy == SH-1) src_done <= 1;
                else begin sy <= sy + 1; pix <= src_px(0); end
            end else begin
                sx <= sx + 1;
                pix <= src_px(sx + 1);
            end
        end
    end
end

// 写口收集：按行记录
localparam integer MAXW = 400000;
reg [31:0] mem [0:MAXW-1];
integer wn = 0;
always @(posedge clk) begin
    if (we) begin
        mem[woff] <= wdata;
        if (wn < MAXW) wn <= wn + 1;
    end
end

integer i, k;
initial begin
    $display("== tb_scale FMT=%0d STRETCH=%0d ==", FMT, STRETCH);
    for (i = 0; i < MAXW; i = i + 1) mem[i] = 32'hDEAD_BEEF;
    repeat (5) @(posedge clk);
    rst = 0;
    repeat (5) @(posedge clk);
    start = 1;
    @(posedge clk);
    start = 0;
    pix_vld <= 1;
    for (k = 0; k < 4_000_000; k = k + 1) begin
        @(posedge clk);
        if (done) k = 4_000_000;
    end
    $display("  [tb] done, 写 %0d 字", wn);
    // 逐行比较（只比较前若干行，输出行数依 FMT）
    begin : cmp
        integer rowsz, rows, r, c, bad, badrow;
        reg [31:0] refw;
        rowsz = (FMT != 0) ? 640 : 160;   // 每行字数
        rows  = (FMT != 0) ? 480 : 240;
        bad = 0; badrow = -1;
        for (r = 1; r < rows; r = r + 1) begin
            for (c = 0; c < rowsz; c = c + 1) begin
                refw = mem[c];
                if (mem[r*rowsz + c] !== refw) begin
                    bad = bad + 1;
                    if (badrow < 0) badrow = r;
                end
            end
        end
        $display("  [逐行] 首行与其它行不同的字数 = %0d，首个不同行 = %0d", bad, badrow);
        if (bad == 0) $display("  PASS：所有输出行逐字相同");
        else begin
            $display("  FAIL：第 %0d 行与第 0 行不同", badrow);
            for (c = 0; c < 16; c = c + 1)
                $display("      c=%0d ref=%h got=%h", c, mem[c], mem[badrow*rowsz + c]);
        end
    end
    $finish;
end

endmodule
