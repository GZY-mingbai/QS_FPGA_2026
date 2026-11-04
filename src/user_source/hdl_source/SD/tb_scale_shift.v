`timescale 1ns/1ps
//==============================================================================
//  tb_scale_shift.v —— 源图进画布，直接量"整体位移量 N"
//==============================================================================
//  【为什么要单写一个台】
//    tb_bmp_scale 的期望函数和 RTL 共用同一套坐标假设（srcpix(a%640, a/640)），
//    两边一起偏就互相抵消 —— 上次 avg2pix 抄 RTL 就是这么假绿的。
//    这里换一种**不依赖任何假设**的量法：
//      源图做成"坐标编码图"：R = x>>2、G = y>>2、B = {x[1:0], y[1:0]}
//      ⇒ 24 位像素里带着它自己的 (x,y)
//      画布抓全量，然后反过来问："画布第 (x,y) 格里的像素，说的是源的哪一点？"
//      ⇒ 直接得出横向位移 N / 纵向位移 V，不需要知道 RTL 想干什么。
//
//  【量法】对每一行 y，把 N 从 -16 扫到 +16，取"不匹配数最少"的那个 N。
//      画布内容整体右移 N ⇒ 画布(x) == 源(x-N) ⇒ 最佳匹配出现在 N = -N_shift…
//      所以下面同时打印两个方向的解释，避免符号搞反。
//==============================================================================
module tb_scale_shift;

reg clk = 0;
always #10 clk = ~clk;

reg         rst = 1;
reg         start = 0;
reg  [15:0] src_w = 0, src_h = 0;
reg         stretch = 0;
reg         fmt = 1;
reg         stall_tb = 0;   // ★ 下游写 FIFO 反压（真机上时有时无）
reg         pix_vld = 0;
reg  [23:0] pix = 0;
wire        pix_rd;
wire        we;
wire [20:0] woff;
wire [31:0] wdata;
wire        busy, done;

bmp_scale #(.CW(640), .CH(480)) dut (
    .clk(clk), .rst(rst),
    .start(start), .src_w(src_w), .src_h(src_h),
    .stretch(stretch), .fmt(fmt),
    .pix_vld(pix_vld), .pix(pix), .pix_rd(pix_rd),
    .we(we), .woff(woff), .wdata(wdata),
    .stall(stall_tb),
    .busy(busy), .done(done)
);

reg prd;
always @(posedge clk) prd <= pix_rd;

// ★ 2026-10-28：stall 发生器。真机上 SDRAM 写口是共享的，`scale_stall`
//   时有时无；888 冲刷 640 字/行（565 只有 160 字），所以 888 撞到 stall
//   的机会是 565 的 4 倍 —— 这是"只有 888 出问题"最合理的一处不对称。
reg [3:0] stg = 0;
always @(posedge clk) stg <= stg + 4'd1;
always @* stall_tb = (stg[3:1] == 3'b000);   // 每 8 拍堵 2 拍

//------------------------------------------------------------------------------
//  坐标编码源：像素自己带着 (x,y)
//------------------------------------------------------------------------------
// 【坑】不能写 {x[9:2], y[8:2], ...}：integer 表达式在拼接里按 32 位自决定宽度，
//   整体会被截断错位（实测 x=4 得到 0x008000 而不是 0x010000）。
//   必须先在函数内落成定宽 reg 再拼。
function [23:0] src2;
    input integer x, y;
    reg [7:0] xh, yh;
    reg [1:0] xl, yl;
    begin
        xh = x >> 2;  yh = y >> 2;
        xl = x & 3;   yl = y & 3;
        src2 = {xh, yh, xl, yl, 4'h0};
    end
endfunction

function integer dec_x;             // 从像素里解回 x
    input [23:0] c;
    begin
        dec_x = {c[23:16], c[7:6]};
    end
endfunction

function integer dec_y;
    input [23:0] c;
    begin
        dec_y = {c[15:8], c[5:4]};
    end
endfunction

//------------------------------------------------------------------------------
//  流式源驱动
//------------------------------------------------------------------------------
reg [15:0] fx, fy;
integer    FDW, FDH;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        fx <= 16'd0; fy <= 16'd0; pix_vld <= 1'b0; pix <= 24'd0;
    end else begin
        if (pix_rd) begin
            if (fx + 16'd1 >= FDW) begin fx <= 16'd0; fy <= fy + 16'd1; end
            else                         fx <= fx + 16'd1;
        end
        pix_vld <= (fy < FDH);
        pix     <= src2(fx, fy);
    end
end

//------------------------------------------------------------------------------
//  画布捕获（全量）
//------------------------------------------------------------------------------
reg  [31:0] cmem [0:307199];
reg         seen [0:307199];
integer     wcnt;
integer     k;

task cap_clr;
    begin
        for (k = 0; k < 307200; k = k + 1) begin
            cmem[k] = 32'hDEAD_DEAD;
            seen[k] = 1'b0;
        end
        wcnt = 0;
    end
endtask

// ★ 抓前 800 次写的地址，找第一个"地址不 +1"的位置
reg [20:0] aseq [0:799];
integer    na = 0;
always @(posedge clk) begin
    if (we && (na < 800)) begin aseq[na] = woff; na = na + 1; end
end
task dump_seq;
    integer i, hit;
    begin
        hit = -1;
        for (i = 1; i < na; i = i + 1)
            if (aseq[i] !== (aseq[i-1] + 21'd1)) begin hit = i; i = na; end
        $display("      前 %0d 次写，第一个不连续位置 = %0d", na, hit);
        if (hit > 0)
            for (i = (hit > 4 ? hit - 4 : 0); i < hit + 4 && i < na; i = i + 1)
                $display("        #%0d woff=%0d", i, aseq[i]);
    end
endtask

always @(posedge clk) begin
    if (we) begin
        if (woff < 307200) begin
            cmem[woff] = wdata;
            seen[woff] = 1'b1;
        end
        wcnt = wcnt + 1;
    end
end

//------------------------------------------------------------------------------
//  量测
//------------------------------------------------------------------------------
integer fails = 0, checks = 0;
task chk;
    input [8*56:1] name;
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %0d", name, got);
        $fflush;
    end
endtask

// 横向：画布(y,x) 的内容 == 源(x-N, y) ⇒ 说明画布内容整体右移了 N
integer bh_n, bh_bad;
task meas_h;
    input integer y;
    integer N, x, bad;
    begin
        bh_n = 0; bh_bad = 999999;
        for (N = -16; N <= 16; N = N + 1) begin
            bad = 0;
            for (x = 20; x < 620; x = x + 1)
                if (cmem[y*640 + x][23:0] !== src2(x - N, y)) bad = bad + 1;
            if (bad < bh_bad) begin bh_bad = bad; bh_n = N; end
        end
        $display("      row %0d : 画布内容相对源**右移 %0d 列**  (不匹配 %0d/600)",
                 y, bh_n, bh_bad);
    end
endtask

// 纵向
integer bv_n, bv_bad;
task meas_v;
    input integer x;
    integer V, y, bad;
    begin
        bv_n = 0; bv_bad = 999999;
        for (V = -8; V <= 8; V = V + 1) begin
            bad = 0;
            for (y = 10; y < 470; y = y + 1)
                if (cmem[y*640 + x][23:0] !== src2(x, y - V)) bad = bad + 1;
            if (bad < bv_bad) begin bv_bad = bad; bv_n = V; end
        end
        $display("      col %0d : 画布内容相对源**下移 %0d 行**  (不匹配 %0d/460)",
                 x, bv_n, bv_bad);
    end
endtask

// 直接按 dec_x/dec_y 读回若干点（更直观的证据）
task dump_pts;
    input integer y;
    integer x;
    begin
        $display("      --- 画布 row %0d 的前 8 个像素解出来的源坐标 ---", y);
        for (x = 0; x < 8; x = x + 1)
            $display("        canvas(x=%0d) = %h  ⇒ 源(%0d, %0d)",
                     x, cmem[y*640 + x][23:0],
                     dec_x(cmem[y*640 + x][23:0]),
                     dec_y(cmem[y*640 + x][23:0]));
    end
endtask

// ★ 2026-10-28：**边缘列专项** —— 上板"右边一列 1px 黑边"
//   把每行最右 4 列和左 2 列原样打出来，并与源逐点比。
integer edge_bad;
task dump_edge;
    input integer y;
    integer x;
    begin
        for (x = 0; x < 2; x = x + 1)
            $display("        row %0d  x=%0d : canvas=%h  src=%h",
                     y, x, cmem[y*640 + x][23:0], src2(x, y));
        for (x = 636; x < 640; x = x + 1)
            $display("        row %0d  x=%0d : canvas=%h  src=%h",
                     y, x, cmem[y*640 + x][23:0], src2(x, y));
    end
endtask

// 全列逐点比（含边缘）
integer col_bad;
task chk_all_cols;
    input integer y;
    integer x;
    begin
        col_bad = 0;
        for (x = 0; x < 640; x = x + 1)
            if (cmem[y*640 + x][23:0] !== src2(x, y)) begin
                if (col_bad < 12)
                    $display("        列 %0d : got=%h exp=%h",
                             x, cmem[y*640+x][23:0], src2(x, y));
                col_bad = col_bad + 1;
            end
        $display("      row %0d : 全 640 列不匹配 %0d 个", y, col_bad);
    end
endtask

//------------------------------------------------------------------------------
//  跑一次
//------------------------------------------------------------------------------
integer miss_cnt;
task run_case;
    input integer W, H;
    input         str;
    input         f;
    begin
        cap_clr;
        @(negedge clk);
        FDW = W; FDH = H;
        fx = 16'd0; fy = 16'd0;
        src_w = W; src_h = H; stretch = str; fmt = f;
        start = 1'b1;
        @(negedge clk);
        start = 1'b0;

        begin : waitd
            integer guard;
            guard = 0;
            while (!done && guard < 12000000) begin
                @(posedge clk);
                guard = guard + 1;
            end
            if (guard >= 12000000) $display("  *** TIMEOUT waiting done ***");
        end
        repeat (20) @(posedge clk);

        miss_cnt = 0;
        // 888 覆盖 0..307199；565 只覆盖 0..38399（行翻转后仍落在这一段里）
        for (k = 0; k < ((f == 1) ? 307200 : 38400); k = k + 1)
            if (!seen[k]) miss_cnt = miss_cnt + 1;
    end
endtask

initial begin
    if ($test$plusargs("vcd")) begin
        $dumpfile("tb_scale_shift.vcd");
        $dumpvars(0, tb_scale_shift);
    end

    repeat (10) @(posedge clk);
    rst = 0;
    repeat (5) @(posedge clk);

    //====================================================================
    $display("");
    $display("==================================================================");
    $display("  A. 640x480 源 → 888 画布（HD 纹理），stretch=0（等比 contain）");
    $display("==================================================================");
    run_case(640, 480, 0, 1);
    $display("  写入字数 = %0d (应为 307200)", wcnt);
    chk("A0 写入字数", wcnt, 307200);
    chk("A1 没有洞",   miss_cnt, 0);
    meas_h(0);   meas_h(1);   meas_h(120);
    meas_h(239); meas_h(240); meas_h(479);
    meas_v(0);   meas_v(320);
    dump_pts(0);
    $display("      === 边缘列专项（上板现象：右边一列 1px 黑边）===");
    dump_edge(0); dump_edge(240);
    chk_all_cols(0);
    chk("A2 row0 全 640 列逐点", col_bad, 0);
    dump_seq;
    chk_all_cols(479);
    chk("A3 row479 全 640 列逐点", col_bad, 0);

    //====================================================================
    $display("");
    $display("==================================================================");
    $display("  A'. 640x480 源 → 888 画布，stretch=0，**下游 stall 每 8 拍堵 2 拍**");
    $display("==================================================================");
    run_case(640, 480, 0, 1);
    $display("  写入字数 = %0d (应为 307200)", wcnt);
    chk("A'0 写入字数", wcnt, 307200);
    chk("A'1 没有洞",   miss_cnt, 0);
    meas_h(0);   meas_h(240); meas_h(479);
    chk_all_cols(0);
    chk("A'2 row0 全 640 列逐点", col_bad, 0);
    chk_all_cols(479);
    chk("A'3 row479 全 640 列逐点", col_bad, 0);
    dump_edge(0);

    //====================================================================
    $display("");
    $display("==================================================================");
    $display("  B. 640x480 源 → 888 画布，stretch=1（拉伸铺满）");
    $display("==================================================================");
    run_case(640, 480, 1, 1);
    $display("  写入字数 = %0d (应为 307200)", wcnt);
    chk("B0 写入字数", wcnt, 307200);
    chk("B1 没有洞",   miss_cnt, 0);
    meas_h(0);   meas_h(240); meas_h(479);
    meas_v(320);

    //====================================================================
    $display("");
    $display("==================================================================");
    $display("  C. 640x480 源 → 565 画布（缩略图纹理），stretch=0");
    $display("==================================================================");
    run_case(640, 480, 0, 0);
    $display("  写入字数 = %0d (应为 38400)", wcnt);
    chk("C0 写入字数", wcnt, 38400);
    chk("C1 没有洞(前38400字)", miss_cnt, 0);
    // 565 的行/列相位：直接看前 8 个字落在哪个地址、内容说什么
    begin : c565
        integer c, m;
        for (m = 0; m < 2; m = m + 1) begin
            $display("      --- 565 内存 row %0d 的前 4 个字 ---", m);
            for (c = 0; c < 4; c = c + 1)
                $display("        [%0d*160+%0d] = %h", m, c, cmem[m*160 + c]);
        end
    end

    $display("");
    $display("==================================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED: %0d / %0d checks", fails, checks);
    $display("==================================================================");
    $finish;
end

endmodule
