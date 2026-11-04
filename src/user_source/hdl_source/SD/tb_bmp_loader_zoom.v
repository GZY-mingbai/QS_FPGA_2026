`timescale 1ns/1ps
//==============================================================================
//  tb_bmp_loader_zoom.v —— 缩放矩阵 + PNG 目检（一次仿真出全部图）
//==============================================================================
//  目的：把 bmp_loader 整链在 **不同画布档 / 888-565 / zoom>1 / zoom<1 /
//        异常宽高比** 下的真实输出拍成图，人眼直接看裁剪、黑边、放大是否对。
//
//  源图：一张 640×480 的"测试靶"（R 随 x 渐变、G 随 y 渐变、64px 网格、
//        主对角线、中心十字、圆环）—— 任何偏移/翻转/比例错都一眼看出来。
//        小尺寸源 = 这张图左上角裁一块（图案是 (x,y) 的纯函数，便于逐点比对）。
//
//  每个用例做三件事：
//    ① 逐点比对（期望 = 同一组几何公式 + 同一张源图，含黑边）
//    ② 地址覆盖（矩形内每字恰好写一次、矩形外零写）
//    ③ 存 PPM（按"图像自然行序"输出 ⇒ 888 的纹理是自下而上，这里倒过来）
//  PPM 用 ppm2png.py 转 PNG。
//==============================================================================
module tb_bmp_loader_zoom;

reg clk = 0;
always #10 clk = ~clk;
wire mclk = clk;
reg  rst = 1;

//----------------------------- DUT 端口 ---------------------------------------
reg         start = 0;
reg  [31:0] file_offset = 0;
reg  [31:0] file_length = 0;
reg  [20:0] dst_base = 0;
reg  [1:0]  mode = 0;
reg         fmt = 1;
reg  [10:0] dst_stride = 0;
reg  [7:0]  zoom = 8'h10;
reg  [9:0]  ax = 10'd320, ay = 10'd240;   // ★ 锚点 = 屏幕中心（等价于原 cam=图心）

wire        ready, busy, done, err;
wire [15:0] src_w_o, src_h_o;

wire        sd_sec_read;
wire [31:0] sd_sec_read_addr;
reg  [7:0]  sd_data = 0;
reg         sd_dv = 0, sd_end = 0;

wire        wr_vld;
wire [9:0]  wr_used;
wire [20:0] wr_addr;
wire [31:0] wr_data;
reg         wr_ack = 0;

bmp_loader #(.AFIFO_AW(9)) dut (
    .clk(clk), .rst(rst), .mclk(mclk),
    .start(start), .abort(1'b0), .file_offset(file_offset), .file_length(file_length),
    .dst_base(dst_base),
    .mode(mode), .fmt(fmt), .dst_stride(dst_stride),
    .zoom(zoom), .ax(ax), .ay(ay),
    .ready(ready), .busy(busy), .done(done), .err(err),
    .src_w_o(src_w_o), .src_h_o(src_h_o),
    .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
    .sd_sec_read_data(sd_data), .sd_sec_read_data_valid(sd_dv),
    .sd_sec_read_end(sd_end), .sd_init_done(1'b1),
    .wr_vld(wr_vld), .wr_used(wr_used), .wr_addr(wr_addr), .wr_data(wr_data),
    .wr_ack(wr_ack), .wr_settled(1'b1)
);
always @(*) wr_ack = wr_vld && !rst;

//------------------------------------------------------------------------------
//  源图：640×480 测试靶（图案是 (x,y) 纯函数）
//------------------------------------------------------------------------------
localparam integer MW = 640, MH = 480;
reg [23:0] simg [0:MW*MH-1];

function [23:0] pat;
    input integer x, y;
    integer r, g, b, dx, dy, d2, dg;
    begin
        r = (x * 255) / 639;
        g = (y * 255) / 479;
        b = 64 + ((x ^ y) & 63);
        if ((x % 64) == 0) begin r = 255; g = 255; b = 255; end   // 竖网格
        if ((y % 64) == 0) begin r = 224; g = 224; b = 224; end   // 横网格
        dg = (x * 479) / 639;
        if ((y >= dg - 1) && (y <= dg + 1)) begin r = 255; g = 40; b = 40; end
        if ((x >= 318) && (x <= 321)) begin r = 40; g = 255; b = 40; end
        if ((y >= 238) && (y <= 241)) begin r = 40; g = 40; b = 255; end
        dx = x - 320; dy = y - 240;
        d2 = dx * dx + dy * dy;
        if ((d2 >= 9604) && (d2 <= 10404)) begin r = 255; g = 40; b = 255; end
        pat = {r[7:0], g[7:0], b[7:0]};
    end
endfunction

integer ix, iy;
initial begin
    for (iy = 0; iy < MH; iy = iy + 1)
        for (ix = 0; ix < MW; ix = ix + 1)
            simg[iy * MW + ix] = pat(ix, iy);
end

//------------------------------------------------------------------------------
//  BMP 字节（自底向上，按绝对地址现算）
//------------------------------------------------------------------------------
integer IMGW = 640, IMGH = 480;
localparam integer PIXOFF = 54;

function [7:0] byte_at;
    input [31:0] a;
    integer off, row, col, b, y, stride;
    reg [31:0] flen;
    reg [23:0] p;
    begin
        flen = PIXOFF + IMGW * IMGH * 3;
        stride = (IMGW * 3 + 3) & (~3);
        if (a < PIXOFF) begin
            case (a)
                32'd0:  byte_at = "B";
                32'd1:  byte_at = "M";
                32'd2:  byte_at = flen[7:0];
                32'd3:  byte_at = flen[15:8];
                32'd4:  byte_at = flen[23:16];
                32'd5:  byte_at = flen[31:24];
                32'd10: byte_at = PIXOFF[7:0];
                32'd11: byte_at = PIXOFF[15:8];
                32'd12: byte_at = PIXOFF[23:16];
                32'd13: byte_at = PIXOFF[31:24];
                32'd14: byte_at = 8'd40;
                32'd18: byte_at = IMGW[7:0];
                32'd19: byte_at = IMGW[15:8];
                32'd20: byte_at = IMGW[23:16];
                32'd21: byte_at = IMGW[31:24];
                32'd22: byte_at = IMGH[7:0];
                32'd23: byte_at = IMGH[15:8];
                32'd24: byte_at = IMGH[23:16];
                32'd25: byte_at = IMGH[31:24];
                32'd26: byte_at = 8'd1;
                32'd28: byte_at = 8'd24;
                default: byte_at = 8'd0;
            endcase
        end else begin
            off = a - PIXOFF;
            row = off / stride;
            col = (off % stride) / 3;
            b   = (off % stride) % 3;
            y   = IMGH - 1 - row;
            p   = simg[y * MW + col];
            case (b)
                0: byte_at = p[7:0];
                1: byte_at = p[15:8];
                default: byte_at = p[23:16];
            endcase
        end
    end
endfunction

//------------------------------------------------------------------------------
//  SD 扇区从机
//------------------------------------------------------------------------------
reg  [31:0] cur_sec = 0;
reg         rd_act = 0;
integer     bi = 0;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_act <= 1'b0; bi <= 0; sd_dv <= 1'b0; sd_end <= 1'b0; sd_data <= 8'd0;
    end else begin
        sd_dv  <= 1'b0;
        sd_end <= 1'b0;
        if (!rd_act) begin
            if (sd_sec_read) begin
                rd_act  <= 1'b1;
                cur_sec <= sd_sec_read_addr;
                bi      <= 0;
            end
        end else begin
            sd_data <= byte_at(cur_sec * 512 + bi[31:0]);
            sd_dv   <= 1'b1;
            if (bi == 511) begin
                sd_end <= 1'b1;
                rd_act <= 1'b0;
            end
            bi <= bi + 1;
        end
    end
end

//------------------------------------------------------------------------------
//  写口捕获
//------------------------------------------------------------------------------
localparam integer MAXW = 307200;
reg [31:0] mem [0:MAXW-1];
reg [7:0]  cnt [0:MAXW-1];
reg [31:0] wmax = 0;
integer    nwr = 0;
integer    k;

task mem_clr;
    begin
        for (k = 0; k < MAXW; k = k + 1) begin
            mem[k] = 32'd0;
            cnt[k] = 8'd0;
        end
        wmax = 0; nwr = 0;
    end
endtask

always @(posedge clk) begin
    if (wr_vld && wr_ack) begin
        mem[wr_addr] <= wr_data;
        cnt[wr_addr] <= cnt[wr_addr] + 8'd1;
        nwr = nwr + 1;
        if (wr_addr > wmax) wmax <= wr_addr;
    end
end

//------------------------------------------------------------------------------
//  断言 / 运行
//------------------------------------------------------------------------------
integer fails = 0, checks = 0;
task chk;
    input [8*48:1] name;
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("   *** FAIL %0s : got %0d, expect %0d", name, got, exp);
            fails = fails + 1;
        end
    end
endtask

reg done_seen = 1'b1;
reg err_seen  = 1'b0;
always @(posedge clk) begin
    if (done) done_seen <= 1'b1;
    if (err)  err_seen  <= 1'b1;
end

task run_one;
    input integer tmo;
    integer n;
    begin
        done_seen = 1'b0;
        err_seen  = 1'b0;
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;
        n = 0;
        while (!done_seen && !err_seen && (n < tmo)) begin
            @(posedge clk);
            n = n + 1;
        end
        @(negedge clk);
        if (!done_seen && !err_seen) begin
            $display("   *** FAIL timeout (st=%0d)", dut.st);
            fails = fails + 1;
        end
        checks = checks + 1;
    end
endtask

//------------------------------------------------------------------------------
//  几何（与 bmp_view 同一组公式）
//------------------------------------------------------------------------------
integer g_w, g_h, g_md, g_fm, g_cw, g_ch, g_step, g_q, g_sx0, g_sy0;
integer g_base, g_stride, g_avg2, g_wpr;

task geom;
    input integer w, h, md, fm, zm;
    integer sf, z;
    begin
        g_w = w; g_h = h; g_md = md; g_fm = fm;
        if ((3*w) >= (4*h)) sf = (w * 1024) / 640;
        else                sf = (h * 1024) / 480;
        z = (zm < 4) ? 4 : zm;     // RTL: zoom 下限 0x04
        g_q    = (sf * 16) / z;
        g_step = g_q << md;
        g_cw   = 640 >> md;
        g_ch   = 480 >> md;
        g_sx0  = ((w >> 1) << 10) - 320 * g_q;
        g_sy0  = ((h >> 1) << 10) - 240 * g_q;
        g_avg2 = (!fm) && (g_step == 2048) && (g_sx0 == 0);
        g_wpr  = fm ? g_cw : (g_cw >> 1);
    end
endtask

//------------------------------------------------------------------------------
//  PPM 输出（图像自然行序：888 的纹理自下而上，这里倒过来）
//------------------------------------------------------------------------------
integer fd;
reg [8*64:1] fn;
reg [8*24:1] tag;

task dump_ppm;
    input integer base, strd, nrows, wpx, fm;
    integer m, x, srow;
    reg [31:0] w;
    reg [7:0] r, g, b;
    reg [15:0] h16;
    begin
        fd = $fopen(fn, "wb");
        $fwrite(fd, "P6\n%0d %0d\n255\n", wpx, nrows);
        for (m = 0; m < nrows; m = m + 1) begin
            srow = fm ? (nrows - 1 - m) : m;
            for (x = 0; x < wpx; x = x + 1) begin
                if (fm) begin
                    w = mem[base + srow * strd + x];
                    if (w[31:24] == 8'd0) begin r = 8'd0; g = 8'd0; b = 8'd0; end
                    else                  begin r = w[23:16]; g = w[15:8]; b = w[7:0]; end
                end else begin
                    w   = mem[base + srow * strd + (x >> 1)];
                    h16 = x[0] ? w[31:16] : w[15:0];
                    r   = {h16[15:11], h16[15:13]};
                    g   = {h16[10:5],  h16[10:9]};
                    b   = {h16[4:0],   h16[4:2]};
                end
                $fwrite(fd, "%c%c%c", r, g, b);
            end
        end
        $fclose(fd);
    end
endtask

//------------------------------------------------------------------------------
//  逐点比对
//------------------------------------------------------------------------------
task chk_pix;
    integer m, x, bad, sy, sx, t, rbad, rfirst, rowsbad, shown;
    reg [23:0] p0, p1, pav;
    reg [8:0]  r9, g9, b9;
    reg [31:0] gotw, expw;
    reg [15:0] got16, exp16;
    reg        blk;
    begin
        bad = 0; rowsbad = 0; shown = 0;
        for (m = 0; m < g_ch; m = m + 1) begin
            if (g_fm) t = g_sy0 + (g_ch - 1 - m) * g_step;
            else      t = g_sy0 + m * g_step;
            if (t < 0) sy = -1; else sy = t / 1024;
            rbad = 0; rfirst = -1;
            for (x = 0; x < g_cw; x = x + 1) begin
                t = g_sx0 + x * g_step;
                if (t < 0) sx = -1; else sx = t / 1024;
                blk = (sy < 0) || (sy >= g_h) || (sx < 0) || (sx >= g_w);
                if (blk) pav = 24'd0;
                else if (g_avg2) begin
                    p0 = simg[sy * MW + sx];
                    p1 = simg[sy * MW + sx + 1];
                    r9 = {1'b0, p0[23:16]} + {1'b0, p1[23:16]};
                    g9 = {1'b0, p0[15:8]}  + {1'b0, p1[15:8]};
                    b9 = {1'b0, p0[7:0]}   + {1'b0, p1[7:0]};
                    pav = {r9[8:1], g9[8:1], b9[8:1]};
                end else
                    pav = simg[sy * MW + sx];

                if (g_fm) begin
                    gotw = mem[g_base + m * g_stride + x];
                    expw = blk ? {8'hFF, 24'd0} : {8'hFF, pav};   // 黑边也是 A=FF
                end else begin
                    gotw  = mem[g_base + m * g_stride + (x >> 1)];
                    got16 = x[0] ? gotw[31:16] : gotw[15:0];
                    exp16 = blk ? 16'd0 : {pav[23:19], pav[15:10], pav[7:3]};
                    expw  = {16'd0, exp16};
                    gotw  = {16'd0, got16};
                end
                if (gotw !== expw) begin
                    if (rfirst < 0) begin
                        rfirst = x;
                        if (shown < 6) begin
                            shown = shown + 1;
                            $display("      (row %0d x %0d) got %h exp %h  src(%0d,%0d)%0s",
                                     m, x, gotw, expw, sx, sy, blk ? " <black>" : "");
                        end
                    end
                    rbad = rbad + 1;
                end
            end
            if (rbad != 0) begin rowsbad = rowsbad + 1; bad = bad + rbad; end
        end
        chk("content", bad, 0);
        if (bad == 0)
            $display("     content ok (%0d×%0d)", g_cw, g_ch);
        else
            $display("     content BAD %0d pixels in %0d rows", bad, rowsbad);
    end
endtask

//------------------------------------------------------------------------------
//  地址覆盖（矩形内一次、矩形外零次、总数相符）
//------------------------------------------------------------------------------
task chk_addr;
    input integer base, strd, nrows;
    integer i, j, bad;
    begin
        bad = 0;
        for (i = 0; i < nrows; i = i + 1)
            for (j = 0; j < g_wpr; j = j + 1)
                if (cnt[base + i * strd + j] != 8'd1) bad = bad + 1;
        chk("A1 one write per word", bad, 0);
        chk("A2 total writes",       nwr, nrows * g_wpr);
        chk("A3 max addr",           (wmax >= base) ? (wmax - base) : -1,
                                     (nrows - 1) * strd + g_wpr - 1);
    end
endtask

//------------------------------------------------------------------------------
//  一个用例
//------------------------------------------------------------------------------
task run_case;
    input integer id;
    input [8*24:1] nm;
    input integer w, h;
    input [1:0]  md;
    input        fm;
    input [7:0]  zm;
    input [10:0] strd;
    input [20:0] base;
    begin
        mem_clr;
        rst = 1;
        repeat (4) @(posedge clk);
        rst = 0;
        @(negedge clk);
        IMGW = w; IMGH = h;
        file_offset = 32'd0;
        file_length = PIXOFF + w * h * 3;
        dst_base = base; mode = md; fmt = fm; dst_stride = strd; zoom = zm;
        ax = 10'd320; ay = 10'd240;      // 锚点 = 屏幕中心（原 cam=图心等价）
        geom(w, h, md, fm, zm);
        g_base = base;
        g_stride = (strd == 11'd0) ? g_wpr : strd;

        $display("--- %0d. %0s : src %0dx%0d, mode%0d/%0s, zoom %0d.%02d ---",
                 id, nm, w, h, md, fm ? "888" : "565", zm >> 4, (zm[3:0] * 100) / 16);
        $display("     geom step=%0d sx0=%0d sy0=%0d canvas %0dx%0d win %0d..%0d",
                 g_step, g_sx0, g_sy0, g_cw, g_ch,
                 (g_sx0 < 0) ? 0 : (g_sx0 >> 10),
                 ((g_sx0 + g_cw * g_step) >> 10));
        run_one(40000000);
        if (err_seen) begin
            $display("   *** FAIL unexpected err");
            fails = fails + 1;
        end
        chk_addr(base, g_stride, g_ch);
        chk_pix;
        $sformat(fn, "out/z%02d_%0s.ppm", id, nm);
        dump_ppm(base, g_stride, g_ch, g_cw, fm);
    end
endtask

//------------------------------------------------------------------------------
//  用例表
//------------------------------------------------------------------------------
initial begin
    $display("======================================================");
    // 源图先存一张，便于对照
    begin : src_dump
        integer m, x;
        reg [23:0] c;
        $sformat(fn, "out/z00_source.ppm");
        fd = $fopen(fn, "wb");
        $fwrite(fd, "P6\n%0d %0d\n255\n", MW, MH);
        for (m = 0; m < MH; m = m + 1)
            for (x = 0; x < MW; x = x + 1) begin
                c = simg[m * MW + x];
                $fwrite(fd, "%c%c%c", c[23:16], c[15:8], c[7:0]);
            end
        $fclose(fd);
    end

    //                     id  tag              W    H   md fm zoom  stride  base
    run_case( 1, "888_full_1x",      640, 480, 2'd0, 1'b1, 8'h10, 11'd0, 21'd0);
    run_case( 2, "888_full_2x",      640, 480, 2'd0, 1'b1, 8'h20, 11'd0, 21'd0);
    run_case( 3, "888_full_1p25x",   640, 480, 2'd0, 1'b1, 8'h14, 11'd0, 21'd0);
    run_case( 4, "888_full_0p75x",   640, 480, 2'd0, 1'b1, 8'h0C, 11'd0, 21'd0);
    run_case( 5, "565_2to1_1x",      640, 480, 2'd1, 1'b0, 8'h10, 11'd0, 21'd0);
    run_case( 6, "565_2to1_2x",      640, 480, 2'd1, 1'b0, 8'h20, 11'd0, 21'd0);
    run_case( 7, "565_2to1_0p5x",    640, 480, 2'd1, 1'b0, 8'h08, 11'd0, 21'd0);
    run_case( 8, "888_small_4x",     640, 480, 2'd2, 1'b1, 8'h40, 11'd0, 21'd0);
    run_case( 9, "888_wide_1x",      320, 200, 2'd0, 1'b1, 8'h10, 11'd0, 21'd0);
    run_case(13, "888_full_0p25x",   640, 480, 2'd0, 1'b1, 8'h04, 11'd0, 21'd0);
    run_case(10, "565_tall_1x",      240, 480, 2'd1, 1'b0, 8'h10, 11'd0, 21'd0);
    run_case(11, "565_small_8x",     640, 480, 2'd2, 1'b0, 8'h80, 11'd0, 21'd0);
    run_case(12, "565_tile_1x",      160, 120, 2'd2, 1'b0, 8'h10, 11'd160, 21'd19280);

    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

initial begin
    #4000000000;
    $display("  *** GLOBAL TIMEOUT");
    $finish;
end

endmodule
