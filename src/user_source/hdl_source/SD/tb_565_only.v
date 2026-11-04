`timescale 1ns/1ps
// 只跑 565 一行的最小台子：看 lbuf 内容和第 0 行的写入序列
module tb_565_only;
reg clk = 0;
always #10 clk = ~clk;

reg         rst = 1, start = 0;
reg  [15:0] src_w = 640, src_h = 480;
reg         fmt = 0;
wire        pix_rd, we, busy, done;
wire [20:0] woff;
wire [31:0] wdata;

reg  [15:0] fx = 0, fy = 0;
reg         pix_vld = 0;
reg  [23:0] pix = 0;
integer     nw = 0;

function [23:0] srcpix;
    input integer x, y;
    begin
        srcpix = {x[7:0], y[7:0], (x[7:0] ^ y[7:0])};
    end
endfunction

bmp_scale dut (
    .clk(clk), .rst(rst), .start(start),
    .src_w(src_w), .src_h(src_h), .stretch(1'b0), .fmt(fmt),
    .pix_vld(pix_vld), .pix(pix), .pix_rd(pix_rd),
    .we(we), .woff(woff), .wdata(wdata), .busy(busy), .done(done)
);

// 顺序源
always @(posedge clk or posedge rst) begin
    if (rst) begin fx <= 0; fy <= 0; pix_vld <= 0; pix <= 0; end
    else begin
        if (pix_rd) begin
            if (fx + 1 >= src_w) begin fx <= 0; fy <= fy + 1; end
            else fx <= fx + 1;
        end
        pix_vld <= (fy < src_h);
        pix     <= srcpix(fx, fy);
    end
end

// 只抓关注地址
reg [31:0] v100, v0, v319;
always @(posedge clk) begin
    if (we) begin
        if (woff == 21'd100)  v100 = wdata;
        if (woff == 21'd0)    v0   = wdata;
        if (woff == 21'd319)  v319 = wdata;
        nw = nw + 1;
    end
end

always @(posedge clk) begin
    if (dut.st == 4'd10 && dut.ox == 11'd2 && dut.oy == 12'd478)
        $display("  [row478 fill] lbuf[200]=%h lbuf[201]=%h sy=%0d sy_t=%0d",
                 dut.lbuf[200], dut.lbuf[201], dut.sy, dut.sy_t);
end

initial begin
    $display("=== 565 single-row dump ===");
    rst = 1; repeat (5) @(posedge clk); rst = 0; repeat (5) @(posedge clk);
    @(negedge clk); start = 1; @(negedge clk); start = 0;
    for (integer t = 0; t < 2000000; t = t + 1) begin
        @(posedge clk);
        if (done) t = 2000000;
    end
    repeat (10) @(posedge clk);
    $display("  done=%0d writes=%0d", done, nw);
    $display("  addr    0 = %h  (row k=239, n=0)", v0);
    $display("  addr  100 = %h  (row k=239, n=100)", v100);
    $display("  addr  319 = %h  (row k=239, n=319)", v319);
    $display("  expect    100 = {to565(src201,478),to565(src200,478)} = %h",
             {8'h0, 5'h19, 6'h37, 5'h2, 8'h0, 5'h19, 6'h37, 5'h2});
    $finish;
end
endmodule
