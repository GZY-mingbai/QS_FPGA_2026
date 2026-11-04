`timescale 1ns/1ps
//==============================================================================
//  tb_bmp_view.v —— 画布渲染器（几何 / 地址 / 内容 / 黑边 / 行序 / 打包）
//==============================================================================
//  纪律（skills/osd-sim-verify）：
//    ① 期望值从**规格**写，不从 RTL 翻译（pitfall #21/#22 的教训）；
//    ② 断言用不变量：地址恰好覆盖一次、内容按整数比精确、565 半字不串位；
//    ③ 先让它们 FAIL 过（用 -DBUG_* 注入），再相信它们。
//
//  源图 = 坐标编码图：R=x>>2, G=y>>2, B={x[1:0],y[1:0],4'h0}
//  ⇒ 画布上任意一格都能反查出"它来自源的哪一点"，不需要另存参考图。
//==============================================================================
module tb_bmp_view;

reg clk = 0;
always #10 clk = ~clk;              // 50MHz

reg rst = 1;

reg         start = 0;
reg  [15:0] src_w = 0, src_h = 0;
reg  [1:0]  mode = 0;
reg         fmt = 1;
reg  [20:0] dst_base = 21'd0;
reg  [10:0] dst_stride = 11'd0;
reg  [7:0]  zoom = 8'h10;
reg  [9:0]  ax = 10'd320, ay = 10'd240;   // ★ 锚点 = 屏幕中心（等价于原 cam=图心）

reg         pix_vld;
reg  [23:0] pix;
wire        pix_rd;
wire [11:0] row_req, win_x0;
wire [12:0] win_w;
reg         stall = 0;
reg         stall_en = 0;
reg  [3:0]  stg = 4'd0;
always @(posedge clk) stg <= stg + 4'd1;
always @(*) stall = stall_en && (stg[3:1] == 3'b000);   // 每 8 拍堵 2 拍

wire        we;
wire [20:0] wa;
wire [31:0] wdata;
wire        busy, done;

bmp_view dut (
    .clk(clk), .rst(rst),
    .start(start),
    .src_w(src_w), .src_h(src_h),
    .mode(mode), .fmt(fmt),
    .dst_base(dst_base), .dst_stride(dst_stride),
    .zoom(zoom), .ax(ax), .ay(ay),
    .pix_vld(pix_vld), .pix(pix), .pix_rd(pix_rd),
    .row_req(row_req), .win_x0(win_x0), .win_w(win_w),
    .stall(stall),
    .we(we), .wa(wa), .wdata(wdata),
    .busy(busy), .done(done)
);

//------------------------------------------------------------------------------
//  源图（坐标编码）与"按需供行"的从机
//------------------------------------------------------------------------------
function [23:0] enc;
    input integer x, y;
    reg [7:0] r, g;
    begin
        r = x >> 2;
        g = y >> 2;
        enc = {r, g, x[1:0], y[1:0], 4'h0};      // 定宽拼接（pitfall #22）
    end
endfunction

reg  [11:0] srv_row = 12'hFFF;
reg         srv_on  = 1'b0;
reg  [13:0] srv_col = 14'd0;

wire [23:0] srv_pix = enc(srv_col[11:0], srv_row);

always @(*) begin
    pix_vld = srv_on;
    pix     = srv_pix;
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        srv_on <= 1'b0; srv_col <= 14'd0; srv_row <= 12'hFFF;
    end else if (!srv_on) begin
        if ((row_req != srv_row) && (win_w != 13'd0)) begin
            srv_on  <= 1'b1;
            srv_row <= row_req;
            srv_col <= {2'd0, win_x0};
        end
    end else begin
        if (row_req != srv_row) begin          // 请求换了（含复位后的第一次）
            srv_row <= row_req;
            srv_col <= {2'd0, win_x0};
        end else if (pix_vld && pix_rd) begin
            if ((srv_col + 14'd1) >= ({1'b0, win_x0} + {1'b0, win_w})) begin
                srv_on  <= 1'b0;
                srv_col <= 14'd0;
            end else
                srv_col <= srv_col + 14'd1;
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
    if (we) begin
        mem[wa] <= wdata;
        cnt[wa] <= cnt[wa] + 8'd1;
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

//------------------------------------------------------------------------------
//  跑一幅：等 done
//------------------------------------------------------------------------------
reg done_seen = 1'b0;
always @(posedge clk) if (done) done_seen <= 1'b1;

task run_one;
    input integer tmo;
    integer n;
    begin
        done_seen = 1'b0;
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;
        n = 0;
        while (!done_seen && (n < tmo)) begin
            @(posedge clk);
            n = n + 1;
        end
        @(negedge clk);
        if (!done_seen) begin
            $display("  *** FAIL  timeout waiting done  (st=%0d ox=%0d oy=%0d)",
                     dut.st, dut.ox, dut.oy);
            fails = fails + 1;
        end
        checks = checks + 1;
    end
endtask

//------------------------------------------------------------------------------
//  地址覆盖：恰好 0..N-1 各写一次
//------------------------------------------------------------------------------
task chk_addr;
    input integer n;
    integer i, bad, extra;
    begin
        bad = 0; extra = 0;
        for (i = 0; i < n; i = i + 1)
            if (cnt[i] != 8'd1) bad = bad + 1;
        for (i = n; i < MAXW; i = i + 1)
            if (cnt[i] != 8'd0) extra = extra + 1;
        chk("A1 every word written once", bad, 0);
        chk("A2 no write beyond rect",     extra, 0);
    end
endtask

//------------------------------------------------------------------------------
//  内容：按"源 = 输出 × kn/kd"的整数比映射逐点核对
//    flip = 1（565）：内存行 m ← 源行 m*kn/kd
//    flip = 0（888）：内存行 m ← 源行 (CH-1-m)*kn/kd
//    avg  = 1：565 的精确 2:1，像素 = 逐通道平均
//------------------------------------------------------------------------------
// ★ 2026-10-30：加 (x0,y0) 源起点 —— 锚点不在屏幕中心时视口左上角不是 (0,0)
task chk_img;
    input integer cwpx, chpx, kn, kd, flip, avgen, x0, y0;
    integer m, x, e, g, y, sx, bad;
    reg [23:0] p0, pav;
    reg [31:0] p1;
    reg [8:0]  r9, g9, b9;
    begin
        bad = 0;
        for (m = 0; m < chpx; m = m + 1) begin
            y = y0 + (flip ? (m * kn / kd) : ((chpx - 1 - m) * kn / kd));
            for (x = 0; x < cwpx; x = x + 1) begin
                sx = x0 + x * kn / kd;
                p0 = enc(sx, y);
                if (avgen) begin
                    p1  = enc(sx + 1, y);
                    r9  = {1'b0, p0[23:16]} + {1'b0, p1[23:16]};
                    g9  = {1'b0, p0[15:8]}  + {1'b0, p1[15:8]};
                    b9  = {1'b0, p0[7:0]}   + {1'b0, p1[7:0]};
                    pav = {r9[8:1], g9[8:1], b9[8:1]};
                end else
                    pav = p0;

                if (flip) begin
                    e = mem[m * (cwpx / 2) + (x >> 1)];
                    if (x[0]) g = e[31:16]; else g = e[15:0];
                    p1 = {16'h0000, pav[23:19], pav[15:10], pav[7:3]};
                end else begin
                    e  = mem[m * cwpx + x];
                    g  = e[31:0];
                    p1 = {8'hFF, pav};
                end

                if (g !== p1) begin
                    if (bad < 4)
                        $display("    (m=%0d x=%0d got %h exp %h src(%0d,%0d))",
                                 m, x, g[31:0], p1[31:0], sx, y);
                    bad = bad + 1;
                end
            end
        end
        chk("B1 content matches source", bad, 0);
    end
endtask

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
integer cw, chh, words;

task case_begin;
    input [15:0] w, h;
    input [1:0]  md;
    input        fm;
    input [20:0] base;
    begin
        mem_clr;
        rst = 1;
        repeat (4) @(posedge clk);
        rst = 0;
        @(negedge clk);
        src_w = w; src_h = h;
        mode  = md; fmt = fm;
        dst_base = base;
        dst_stride = 11'd0;
        stall_en = 1'b0;
        zoom = 8'h10;
        ax = 10'd320;
        ay = 10'd240;
    end
endtask

initial begin
    $display("======================================================");

    //--------------------------------------------------------------
    // 1) 640×480 → 640×480 888：1:1 逐点精确
    //--------------------------------------------------------------
    $display("--- 1. 640x480 -> mode0/888 (1:1) ---");
    case_begin(640, 480, 2'd0, 1'b1, 21'd0);
    run_one(4000000);
    chk_addr(307200);
    chk_img(640, 480, 1, 1, 0, 0, 0, 0);

    //--------------------------------------------------------------
    // 2) 640×480 → 320×240 565：2:1 + 逐通道平均 + 行翻转
    //--------------------------------------------------------------
    $display("--- 2. 640x480 -> mode1/565 (2:1 avg) ---");
    case_begin(640, 480, 2'd1, 1'b0, 21'd0);
    run_one(4000000);
    chk_addr(38400);
    chk_img(320, 240, 2, 1, 1, 1, 0, 0);

    //--------------------------------------------------------------
    // 3) 320×240 → 640×480 888：放大 2×（源像素重复）
    //--------------------------------------------------------------
    $display("--- 3. 320x240 -> mode0/888 (2x up) ---");
    case_begin(320, 240, 2'd0, 1'b1, 21'd0);
    run_one(4000000);
    chk_addr(307200);
    chk_img(640, 480, 1, 2, 0, 0, 0, 0);

    //--------------------------------------------------------------
    // 4) 640×480 → 160×120 888：4:1
    //--------------------------------------------------------------
    $display("--- 4. 640x480 -> mode2/888 (4:1) ---");
    case_begin(640, 480, 2'd2, 1'b1, 21'd0);
    run_one(4000000);
    chk_addr(19200);
    chk_img(160, 120, 4, 1, 0, 0, 0, 0);

    //--------------------------------------------------------------
    // 5) 四宫格单格：160×120 565 写进 320×240 纹理的右上格
    //    stride=160（纹理行跨度），base = qy*120*160 + qx*80
    //--------------------------------------------------------------
    $display("--- 5. 160x120 -> mode2/565 tile (stride 160) ---");
    case_begin(160, 120, 2'd2, 1'b0, 21'd19280);
    dst_stride = 11'd160;
    run_one(4000000);
    begin : t5
        integer m, x, bad, e, g, y, sx;
        reg [23:0] p0, p1;
        bad = 0;
        for (m = 0; m < 120; m = m + 1) begin
            y = m;                                   // 565：行序正
            for (x = 0; x < 160; x = x + 1) begin
                sx  = x;
                p0  = enc(sx, y);
                e   = mem[19280 + m*160 + (x >> 1)];
                g   = x[0] ? e[31:16] : e[15:0];
                if (g !== {8'h00, p0[23:19], p0[15:10], p0[7:3]}) bad = bad + 1;
            end
        end
        chk("E1 tile content (stride 160)", bad, 0);
        p0 = enc(1, 0); p1 = enc(0, 0);
        chk("E2 tile row0 word0",
            mem[19280] & 32'hFFFF,
            {1'b0, p1[23:19], p1[15:10], p1[7:3]});
        chk("E3 tile row0 word0 hi",
            (mem[19280] >> 16) & 32'hFFFF,
            {1'b0, p0[23:19], p0[15:10], p0[7:3]});
    end

    //--------------------------------------------------------------
    // 6) 背压：888 每 8 拍堵 2 拍（只冻前半段会"塌一格"⇒ 每行漏一列）
    //--------------------------------------------------------------
    $display("--- 6. 640x480 -> mode0/888 with stall ---");
    case_begin(640, 480, 2'd0, 1'b1, 21'd0);
    stall_en = 1'b1;
    run_one(6000000);
    chk_addr(307200);
    chk_img(640, 480, 1, 1, 0, 0, 0, 0);

    //--------------------------------------------------------------
    // 7) 背压：565 同理
    //--------------------------------------------------------------
    $display("--- 7. 640x480 -> mode1/565 with stall ---");
    case_begin(640, 480, 2'd1, 1'b0, 21'd0);
    stall_en = 1'b1;
    run_one(6000000);
    chk_addr(38400);
    chk_img(320, 240, 2, 1, 1, 1, 0, 0);


    //--------------------------------------------------------------
    //  8) 锚点 ≠ 屏幕中心（mode0/888，zoom 2×，ax=160/ay=120）
    //     cam  = (W/2)<<10 + (ax-320)*(step_fit - q)
    //          = 320*1024 + (-160)*(1024-512) = 245760  ⇒ cam_px=240
    //     sx0  = 245760 - (640*512)/2 = 81920 ⇒ 80，step=512
    //     sy0  = 184320 - (480*512)/2 = 61440 ⇒ 60
    //     ⇒ 输出 (x,oy) 取源 (80 + x/2, 60 + (479-m)/2)
    //--------------------------------------------------------------
    $display("--- 8. 640x480 -> mode0/888 anchor(160,120) zoom 2x ---");
    case_begin(640, 480, 2'd0, 1'b1, 21'd0);
    zoom = 8'h20; ax = 10'd160; ay = 10'd120;
    run_one(6000000);
    chk_addr(307200);
    chk_img(640, 480, 1, 2, 0, 0, 80, 60);

    //--------------------------------------------------------------
    //  9) 锚点 ≠ 中心 + 更大倍率（mode0/888，zoom 4×，ax=500/ay=400）
    //     cam = 320*1024 + 180*(1024-256) = 465920 ⇒ 455
    //     sx0 = 465920 - (640*256)/2 = 384000 ⇒ 375；step=256
    //     sy0 = 245760 + 160*768 - (480*256)/2 = 368640-61440 = 307200 ⇒ 300
    //--------------------------------------------------------------
    $display("--- 9. 640x480 -> mode0/888 anchor(500,400) zoom 4x ---");
    case_begin(640, 480, 2'd0, 1'b1, 21'd0);
    zoom = 8'h40; ax = 10'd500; ay = 10'd400;
    run_one(6000000);
    chk_addr(307200);
    chk_img(640, 480, 1, 4, 0, 0, 375, 300);

    //--------------------------------------------------------------
    // 10) ★ 锚点 + mode1/565（这条专门抓 "dstep 用错 step 还是 q" 的 bug）
    //     正确：dstep = step_fit - q = 1024-512 = 512 ⇒ cam = 245760 ⇒ 240
    //     错误（用 step_n=q<<mode=1024）：cam = 327680 ⇒ 320，整幅偏 80 列
    //     mode1 下 step=1024 ⇒ 1:1；565 行序翻转 ⇒ y = 60 + m
    //--------------------------------------------------------------
    $display("--- 10. 640x480 -> mode1/565 anchor(160,120) zoom 2x ---");
    case_begin(640, 480, 2'd1, 1'b0, 21'd0);
    zoom = 8'h20; ax = 10'd160; ay = 10'd120;
    run_one(6000000);
    chk_addr(38400);
    chk_img(320, 240, 1, 1, 1, 0, 80, 60);

    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

// 超时保护
initial begin
    #200000000;
    $display("  *** GLOBAL TIMEOUT");
    $finish;
end

endmodule
