`timescale 1ns/1ps
//==============================================================================
//  tb_hd_stream.v —— S_HD（888 图层）送进帧缓存的像素流是否逐点精确
//==============================================================================
//  【为什么需要这个台】
//    上板现象：HD 模式**右边有一列 1px 黑边**，左边正常（⇒ 不是位移，是一列黑）。
//    纹理侧已证明逐点精确（tb_bmp_read640 / tb_scale_shift 全 307200 字对），
//    所以黑列只能出在"图层 → 合并"这段像素流里。而这段唯一被改过的是
//    viewer_scene（OSD 逐像素叠加）。
//
//  【做法】直接把 viewer_scene 放在 a_hd=1、满屏、b_en=0、osd_on=0 的
//    S_HD 姿态下跑一帧，纹理模型用**坐标编码**：
//        HD 槽第 k 个字 = {8'hFF, k%640, k/640, 8'h5A}
//    S_HD 时 HD 图层是 1:1 顺序读 ⇒ 第 k 个像素必须正好等于
//        {k%640, k/640, 8'h5A}
//    任何一列黑/错位都会直接显形，并且能定位到具体 (x,y)。
//==============================================================================
module tb_hd_stream;

reg clk = 0;
always #10 clk = ~clk;                  // 50MHz 像素域
wire mclk = clk;                        // 本台把 SDRAM 侧同频，CDC 天然安全

localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam integer NPX   = IMG_W * IMG_H;
localparam [20:0] HD_BASE = 21'd729600;

reg  rst = 1;
reg  ps_start = 0;
reg  fmt565 = 0;
reg  wr_stall = 0;
reg  a_hd = 1;
reg  a_en = 1;
reg  [31:0] a_pos  = 32'd0;
reg  [31:0] a_size = {16'd640, 16'd480};
reg  [39:0] a_k    = {20'd16384, 20'd32768};
reg  [20:0] a_base = HD_BASE;
reg  [31:0] b_pos  = 32'd0;
reg  [31:0] b_size = {16'd640, 16'd480};
reg  [39:0] b_k    = {20'd16384, 20'd32768};
reg  [20:0] b_base = 21'd614400;
reg  b_en = 0;
reg  [7:0] alpha_b = 8'd0;
reg  osd_on = 0;
reg  [7:0]  osd_fade = 8'd0;
reg  [31:0] osd_pix  = 32'd0;

wire pixel_ready, pipe_ce;
wire [23:0] ps_dat;
wire [15:0] scan_x, scan_y;
wire [1:0]  tex_req_vld;
wire [41:0] tex_req_addr;
wire [7:0]  tex_req_mask;
reg  [1:0]  tex_req_ack = 0;
reg  [1:0]  tex_pix_wr  = 0;
reg  [63:0] tex_pix_data = 0;

viewer_scene u_scene (
    .clk(clk), .rst(rst), .mclk(mclk),
    .ps_start(ps_start), .pixel_ready(pixel_ready), .ps_data(ps_dat), .pipe_ce(pipe_ce),
    .fmt565(fmt565), .wr_stall(wr_stall),
    .a_hd(a_hd), .a_pos(a_pos), .a_size(a_size), .a_k(a_k), .a_ori(20'd0), .a_base(a_base), .a_en(a_en),
    .quad_mode(1'b0),
    .b_pos(b_pos), .b_size(b_size), .b_k(b_k), .b_base(b_base), .b_en(b_en), .alpha_b(alpha_b),
    .scan_x(scan_x), .scan_y(scan_y),
    .osd_on(osd_on), .osd_fade(osd_fade), .osd_pix(osd_pix),
    .tex_req_vld(tex_req_vld), .tex_req_addr(tex_req_addr), .tex_req_mask(tex_req_mask),
    .tex_req_ack(tex_req_ack), .tex_pix_wr(tex_pix_wr), .tex_pix_data(tex_pix_data)
);

//------------------------------------------------------------------------------
//  纹理模型：HD 槽第 k 个字 = {8'hFF, k%640, k/640, 8'h5A}
//  每收到一个请求，回 4 个字（req_mask=4'hF）
//------------------------------------------------------------------------------
function [31:0] tex_word;
    input [20:0] a;
    integer w, col, row;
    reg [7:0] cx, cy;
    begin
        w   = a - HD_BASE;
        if ((w < 0) || (w >= NPX)) tex_word = 32'hDEAD_BEEF;
        else begin
            cx = w % 640;
            cy = w / 640;
            tex_word = {8'hFF, cx, cy, 8'h5A};
        end
    end
endfunction

reg [1:0] srv = 0;
reg [1:0] kk  = 0;
reg [20:0] ca = 0;
reg [7:0]  cm = 0;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        srv <= 0; kk <= 0; ca <= 0; cm <= 0;
        tex_req_ack <= 0; tex_pix_wr <= 0; tex_pix_data <= 0;
    end else begin
        tex_req_ack <= 0;
        tex_pix_wr  <= 0;
        if (srv == 2'd0) begin
            if (tex_req_vld[0]) begin
                tex_req_ack[0] <= 1'b1;
                ca <= tex_req_addr[20:0];
                cm <= tex_req_mask[3:0];
                kk <= 0;
                srv <= 2'd1;
            end
        end else if (srv == 2'd1) begin
            if (cm[kk]) begin
                tex_pix_wr[0]   <= 1'b1;
                tex_pix_data[31:0] <= tex_word(ca + {19'd0, kk});
            end
            if (kk == 2'd3) srv <= 2'd2; else kk <= kk + 2'd1;
        end else begin
            srv <= 2'd0;
        end
    end
end

//------------------------------------------------------------------------------
//  抓像素流
//------------------------------------------------------------------------------
reg [23:0] cap [0:307199];
integer    nk;
integer    q, bad;
integer    bad_shift;
reg [7:0]  ex, ey;
reg [23:0] expw;

initial begin
    repeat (20) @(posedge clk);
    rst = 0;
    repeat (10) @(posedge clk);

    $display("");
    $display("==================================================================");
    $display("  S_HD 姿态：viewer_scene 送进帧缓存的像素流（a_hd=1, 满屏, 888）");
    $display("==================================================================");

    nk = 0;
    @(negedge clk);
    ps_start = 1'b1;
    while (nk < NPX) begin
        if (pipe_ce) begin
            cap[nk] = ps_dat;
            nk = nk + 1;
        end
        @(negedge clk);
    end
    ps_start = 1'b0;
    repeat (10) @(negedge clk);

    $display("  抓到的像素数 = %0d (应为 %0d)", nk, NPX);

    bad_shift = 0;
    bad = 0;
    for (q = 0; q < NPX; q = q + 1) begin
        ex = q % 640;
        ey = q / 640;
        expw = {ex, ey, 8'h5A};
        if (cap[q] !== expw) begin
            if (bad < 10)
                $display("        [%0d] (x=%0d,y=%0d) got=%h exp=%h",
                         q, ex, ey, cap[q], expw);
            bad = bad + 1;
        end
    end
    $display("  逐点不匹配 = %0d / %0d", bad, NPX);

    // ★ 我这一版是按"每个 pipe_ce 拍取一个"抓的，没有模拟 frame_writer 的
    //   `write_en = take 延迟 PIPE 拍`。所以先按 PIPE 拍补偿再比一次：
    //   若补偿后全对 ⇒ 这段像素流本来就是对的，之前的"错"是我台的相位。
    //   ★ 2026-10-30 [DSP-PIPE]：混合模块加了一级输入寄存器 ⇒ PIPE 4→5。
    begin : shifted
        integer d, sb;
        sb = 0;
        for (d = 0; d < NPX - 5; d = d + 1) begin
            ex = d % 640;
            ey = d / 640;
            expw = {ex, ey, 8'h5A};
            if (cap[d+5] !== expw) sb = sb + 1;
        end
        $display("  按 5 拍补偿后不匹配 = %0d / %0d", sb, NPX-5);
        bad_shift = sb;
    end

    // 每行最后一列单独统计（上板就是"右边一列黑"）
    begin : edgechk
        integer y, eb;
        eb = 0;
        for (y = 0; y < 480; y = y + 1) begin
            ex = 8'h7F; ey = y;
            if (cap[y*640 + 639] !== {ex, ey, 8'h5A}) eb = eb + 1;
        end
        $display("  [未补偿视图，仅供参考] 最后一列 (x=639) 错的行数 = %0d / 480", eb);
        for (y = 0; y < 480; y = y + 160)
            $display("        row %0d : x=638 %h | x=639 %h (exp %h)",
                     y, cap[y*640+638], cap[y*640+639],
                     {8'h7F, y, 8'h5A});
    end

    $display("");
    if (bad_shift == 0) $display("  ALL PASS  —— 像素流逐点精确（已按 PIPE=5 对齐 frame_writer 的 write_en）");
    else                $display("  FAILED：像素流有 %0d 个错点", bad_shift);
    $display("==================================================================");
    $finish;
end

initial begin
    #50_000_000;
    $display("TIMEOUT nk=%0d ps_start=%b pipe_ce=%b", nk, ps_start, pipe_ce);
    $finish;
end

endmodule
