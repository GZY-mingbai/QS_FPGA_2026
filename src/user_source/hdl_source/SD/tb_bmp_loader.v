`timescale 1ns/1ps
//==============================================================================
//  tb_bmp_loader.v —— 整链：BMP 头 + 扇区流 → bmp_view → 写口
//==============================================================================
//  验证目标（都是不变量，先让它们 FAIL 再改 RTL）：
//    ① 写口字数 = 每行字数 × 行数，且 `dst_base..+N-1` 每个字恰好写一次
//    ② 内容 = 源图按几何映射回来的像素（整数比逐点精确）
//    ③ 565 行序翻转 + 偶数/奇数半字不串位
//    ④ zoom>1 时 seek + 行首跳过生效（内容 = 中心窗口，不是整图）
//    ⑤ done/err、src_w_o/src_h_o
//
//  源图 = 坐标编码图：R=x>>2, G=y>>2, B={x[1:0],y[1:0],4'h0}
//  BMP 头与像素都按"绝对字节地址"现算，不占存储（同 tb_bmp_read640 的做法）。
//
//  ★ 2026-10-29 重写期望模型：不再手写 x0/y0/step，**从输入参数按 RTL 的同一组
//    公式算**（step_fit→q→step<<mode、sx0=cam-320q）。原来 case3 手写成
//    step=512/x0=280，与 RTL 的 2048/[160,480) 不符，19199 个"失败"是期望错。
//    另加：逐行诊断（前 6 个坏行打印首个坏列）+ 失败时把整片写到 out/dump_N.txt。
//==============================================================================
module tb_bmp_loader;

reg clk = 0;
always #10 clk = ~clk;              // 50MHz
wire mclk = clk;

reg rst = 1;

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

// 写口消费者：永远当场取走
always @(*) wr_ack = wr_vld && !rst;

//------------------------------------------------------------------------------
//  源图尺寸（每个用例前设置）
//------------------------------------------------------------------------------
integer IMGW = 640, IMGH = 480;
integer PIXOFF = 54;

function [23:0] enc;
    input integer x, y;
    reg [7:0] r, g;
    begin
        r = x >> 2;
        g = y >> 2;
        enc = {r, g, x[1:0], y[1:0], 4'h0};       // 定宽拼接（pitfall #22）
    end
endfunction

//------------------------------------------------------------------------------
//  BMP 字节（按绝对地址现算）
//------------------------------------------------------------------------------
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
                32'd26: byte_at = 8'd1;          // planes
                32'd28: byte_at = 8'd24;         // bpp
                default: byte_at = 8'd0;
            endcase
        end else begin
            off = a - PIXOFF;
            row = off / stride;                   // 文件行（自底向上）
            col = (off % stride) / 3;
            b   = (off % stride) % 3;
            y   = IMGH - 1 - row;
            p   = enc(col, y);
            case (b)
                0: byte_at = p[7:0];              // B
                1: byte_at = p[15:8];             // G
                default: byte_at = p[23:16];      // R
            endcase
        end
    end
endfunction

//------------------------------------------------------------------------------
//  SD 扇区从机：收到请求后按字节吐 512 个
//------------------------------------------------------------------------------
reg  [31:0] cur_sec = 0;
reg         rd_act = 0;
reg         gap    = 1'b0;      // ★ 突发之间必须有一拍空隙（= 仲裁器的 A_GAP）
integer     bi = 0;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_act <= 1'b0; bi <= 0; sd_dv <= 1'b0; sd_end <= 1'b0; sd_data <= 8'd0;
        gap <= 1'b0;
    end else begin
        sd_dv  <= 1'b0;
        sd_end <= 1'b0;
        if (!rd_act) begin
            gap <= 1'b0;
            // 真链路上裁决器每收完一个扇区要经 A_GAP 一拍才回 A_IDLE，且控制器
            // 也要回 WAIT 才重新锁地址 ⇒ 请求在这两拍里是被"看不见"的。
            // 装载器的请求是**电平**，所以这里按"必须有一拍空隙"建模。
            if (sd_sec_read && !gap) begin
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
                gap    <= 1'b1;
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
integer k;

task mem_clr;
    begin
        for (k = 0; k < MAXW; k = k + 1) begin
            mem[k] = 32'd0;
            cnt[k] = 8'd0;
        end
    end
endtask

always @(posedge clk) begin
    if (wr_vld && wr_ack) begin
        mem[wr_addr] <= wr_data;
        cnt[wr_addr] <= cnt[wr_addr] + 8'd1;
    end
end

//------------------------------------------------------------------------------
//  断言
//------------------------------------------------------------------------------
integer fails = 0, checks = 0;
task chk;
    input [8*64:1] name;
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %0d", name, got);
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
            $display("  *** FAIL  timeout (st=%0d)", dut.st);
            fails = fails + 1;
        end
        checks = checks + 1;
    end
endtask

//------------------------------------------------------------------------------
//  A：地址覆盖（按行/字列描述矩形，565 的 stride 可能大于每行字数）
//------------------------------------------------------------------------------
task chk_addr;
    input integer base, stride, nrows, wpr;
    integer a, i, j, bad, extra, inr, off;
    begin
        bad = 0; extra = 0;
        for (i = 0; i < nrows; i = i + 1)
            for (j = 0; j < wpr; j = j + 1)
                if (cnt[base + i * stride + j] != 8'd1) begin
                    if (bad < 4) $display("    (miss addr %0d row %0d col %0d)",
                                          base + i * stride + j, i, j);
                    bad = bad + 1;
                end
        for (a = 0; a < MAXW; a = a + 1) begin
            off = a - base;
            inr = (off >= 0) && (off < nrows * stride) && ((off % stride) < wpr) &&
                  ((off / stride) < nrows);
            if (!inr && (cnt[a] != 8'd0)) extra = extra + 1;
        end
        chk("A1 every word written once", bad, 0);
        chk("A2 no write outside rect",   extra, 0);
    end
endtask

//------------------------------------------------------------------------------
//  几何：与 bmp_view 完全同一组公式（step_fit→q→step<<mode、sx0=cam-320q）
//------------------------------------------------------------------------------
integer g_w, g_h, g_md, g_fm, g_cw, g_ch, g_step, g_q, g_sx0, g_sy0;
integer g_base, g_stride, g_avg2;

task geom;
    input integer w, h, md, fm, zm;
    integer sf, z;
    begin
        g_w = w; g_h = h; g_md = md; g_fm = fm;
        if ((3*w) >= (4*h)) sf = (w * 1024) / 640;   // [C2] 只比一次大小
        else                sf = (h * 1024) / 480;
        z  = (zm < 16) ? 16 : zm;
        g_q    = (sf * 16) / z;
        g_step = g_q << md;
        g_cw   = 640 >> md;
        g_ch   = 480 >> md;
        g_sx0  = ((w >> 1) << 10) - 320 * g_q;       // cam_auto = 图心
        g_sy0  = ((h >> 1) << 10) - 240 * g_q;
        g_avg2 = (!fm) && (g_step == 2048) && ((g_sx0 & 2047) == 0);   // RTL: sx0[10:0]==0
    end
endtask

//------------------------------------------------------------------------------
//  B：内容
//------------------------------------------------------------------------------
integer dcase = 0;
integer fd;
reg [8*40:1] dname;

task dump_region;
    input integer base, n;
    integer j;
    begin
        $sformat(dname, "out/dump_%0d.txt", dcase);
        fd = $fopen(dname, "w");
        for (j = 0; j < n; j = j + 1) $fwrite(fd, "%08x\n", mem[base+j]);
        $fclose(fd);
        $display("   (dump -> %0s, %0d words)", dname, n);
    end
endtask

task chk_pix;
    integer m, x, bad, tot, sy, sx, t, rowsbad, shown;
    reg [23:0] p0, p1, pav;
    reg [8:0]  r9, g9, b9;
    reg [31:0] gotw, expw;
    reg [15:0] got16, exp16;
    reg        blk;
    integer    rbad, rfirst;
    reg [31:0] rgot, rexp;
    begin
        bad = 0; tot = 0; rowsbad = 0; shown = 0;
        for (m = 0; m < g_ch; m = m + 1) begin
            // 该 mem 行对应的源行（888 正序 / 565 反序）
            if (g_fm) t = g_sy0 + (g_ch - 1 - m) * g_step;
            else      t = g_sy0 + m * g_step;
            if (t < 0) sy = -1; else sy = t / 1024;
            rbad = 0; rfirst = -1; rgot = 0; rexp = 0;
            for (x = 0; x < g_cw; x = x + 1) begin
                t = g_sx0 + x * g_step;
                if (t < 0) sx = -1; else sx = t / 1024;
                blk = (sy < 0) || (sy >= g_h) || (sx < 0) || (sx >= g_w);
                if (blk) pav = 24'd0;
                else if (g_avg2) begin
                    p0 = enc(sx, sy);
                    p1 = enc(sx + 1, sy);
                    r9 = {1'b0, p0[23:16]} + {1'b0, p1[23:16]};
                    g9 = {1'b0, p0[15:8]}  + {1'b0, p1[15:8]};
                    b9 = {1'b0, p0[7:0]}   + {1'b0, p1[7:0]};
                    pav = {r9[8:1], g9[8:1], b9[8:1]};
                end else
                    pav = enc(sx, sy);

                if (g_fm) begin
                    gotw = mem[g_base + m * g_stride + x];
                    expw = blk ? 32'd0 : {8'hFF, pav};
                    tot  = tot + 1;
                    if (gotw !== expw) begin
                        if (rfirst < 0) begin rfirst = x; rgot = gotw; rexp = expw; end
                        rbad = rbad + 1;
                    end
                end else begin
                    gotw = mem[g_base + m * g_stride + (x >> 1)];
                    got16 = x[0] ? gotw[31:16] : gotw[15:0];
                    exp16 = blk ? 16'd0 : {pav[23:19], pav[15:10], pav[7:3]};
                    if (x[0] == 1'b0) tot = tot + 1;
                    if (got16 !== exp16) begin
                        if (rfirst < 0) begin rfirst = x; rgot = got16; rexp = exp16; end
                        rbad = rbad + 1;
                    end
                end
            end
            bad = bad + rbad;
            if (rbad != 0) begin
                rowsbad = rowsbad + 1;
                if (shown < 6) begin
                    shown = shown + 1;
                    $display("    row %0d (sy=%0d, base=%0d): %0d/%0d bad, first x=%0d got %h exp %h",
                             m, sy, g_base + m * g_stride, rbad, g_cw, rfirst, rgot, rexp);
                end
            end
        end
        $display("   bad %0d / %0d pixels, %0d bad rows", bad, tot, rowsbad);
        chk("B1 content matches source", bad, 0);
        if (bad != 0) dump_region(g_base, g_ch * g_stride);
    end
endtask

//------------------------------------------------------------------------------
//  用例准备
//------------------------------------------------------------------------------
task setup;
    input integer w, h;
    input [1:0]  md;
    input        fm;
    input [20:0] base;
    input [7:0]  zm;
    input [10:0] strd;
    input integer tagno;
    begin
        mem_clr;
        rst = 1;
        repeat (4) @(posedge clk);
        rst = 0;
        @(negedge clk);
        IMGW = w; IMGH = h;
        file_offset = 32'd0;
        file_length = PIXOFF + w * h * 3;
        dst_base    = base;
        mode = md; fmt = fm; dst_stride = strd;
        zoom = zm;
        ax = 10'd320; ay = 10'd240;      // 锚点 = 屏幕中心（原 cam=图心等价）
        dcase = tagno;
        geom(w, h, md, fm, zm);
        g_base = base; g_stride = (strd == 11'd0) ? (fm ? g_cw : (g_cw >> 1)) : strd;
    end
endtask

initial begin
    $display("======================================================");

    //--------------------------------------------------------------
    // 1) 640×480 → mode0/888，zoom 1.0：整幅 1:1（step=1024）
    //--------------------------------------------------------------
    $display("--- 1. 640x480 -> mode0/888 full (step=1024) ---");
    setup(640, 480, 2'd0, 1'b1, 21'd0, 8'h10, 11'd0, 1);
    $display("    geom: step=%0d sx0=%0d sy0=%0d cw=%0d ch=%0d avg2=%0d",
             g_step, g_sx0, g_sy0, g_cw, g_ch, g_avg2);
    run_one(9000000);
    chk("1 src_w", src_w_o, 640);
    chk("1 src_h", src_h_o, 480);
    chk_addr(0, g_stride, g_ch, g_cw);
    chk_pix;

    //--------------------------------------------------------------
    // 2) 640×480 → mode1/565，zoom 1.0：2:1 + 逐通道平均 + 行翻转
    //    窗口 = 整幅，step=2048
    //--------------------------------------------------------------
    $display("--- 2. 640x480 -> mode1/565 (step=2048, 2:1 avg, flip) ---");
    setup(640, 480, 2'd1, 1'b0, 21'd0, 8'h10, 11'd0, 2);
    $display("    geom: step=%0d sx0=%0d sy0=%0d cw=%0d ch=%0d avg2=%0d",
             g_step, g_sx0, g_sy0, g_cw, g_ch, g_avg2);
    run_one(9000000);
    chk_addr(0, g_stride, g_ch, g_cw >> 1);
    chk_pix;

    //--------------------------------------------------------------
    // 3) 640×480 → mode2/888，zoom 2.0：中心 320×240 窗口
    //    q = 1024*16/32 = 512 ⇒ step = 512<<2 = 2048
    //    sx0 = 320<<10 - 320*512 = 163840 ⇒ 源 x[160,480)，y[120,360)
    //--------------------------------------------------------------
    $display("--- 3. 640x480 -> mode2/888 zoom 2x (step=2048, win 320x240) ---");
    setup(640, 480, 2'd2, 1'b1, 21'd0, 8'h20, 11'd0, 3);
    $display("    geom: step=%0d sx0=%0d sy0=%0d cw=%0d ch=%0d avg2=%0d",
             g_step, g_sx0, g_sy0, g_cw, g_ch, g_avg2);
    run_one(9000000);
    chk_addr(0, g_stride, g_ch, g_cw);
    chk_pix;

    //--------------------------------------------------------------
    // 4) 160×120 → mode2/565 单格（写进 320×240 纹理的某一格，stride=160）
    //    step=1024 1:1，无平均
    //--------------------------------------------------------------
    $display("--- 4. 160x120 -> mode2/565 tile (stride 160, base 19280) ---");
    setup(160, 120, 2'd2, 1'b0, 21'd19280, 8'h10, 11'd160, 4);
    $display("    geom: step=%0d sx0=%0d sy0=%0d cw=%0d ch=%0d avg2=%0d",
             g_step, g_sx0, g_sy0, g_cw, g_ch, g_avg2);
    run_one(9000000);
    chk_addr(19280, g_stride, g_ch, g_cw >> 1);
    chk_pix;

    //--------------------------------------------------------------
    // 5) 非 BMP：头非法必须报 err，且不写任何字
    //--------------------------------------------------------------
    $display("--- 5. invalid header -> err ---");
    setup(640, 480, 2'd0, 1'b1, 21'd0, 8'h10, 11'd0, 5);
    begin : t5
        integer i, nz;
        // 把 header_0 改成 'X'
        force dut.hb0 = "X";
        run_one(200000);
        release dut.hb0;
        nz = 0;
        for (i = 0; i < 1024; i = i + 1) if (cnt[i] != 0) nz = nz + 1;
        chk("5 err asserted", err_seen, 1);
        chk("5 no write",     nz, 0);
    end

    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

initial begin
    #2000000000;
    $display("  *** GLOBAL TIMEOUT");
    $finish;
end

endmodule
