`timescale 1ns/1ps
//==============================================================================
//  tb_pp_equiv.v —— video_postproc 面积优化前后逐拍等价
//==============================================================================
//  2026-10-31 新建。白平衡建表从"一拍写三通道（3 台乘法）"改成"一拍一通道（1 台）"，
//  建表从 256 拍变 768 拍。对比改动前版本（video_postproc_ref，由 run_all_tb.sh 从
//  固定提交 7e6ad32 生成）与现版的全部输出。
//  激励：真实形状的同步（消隐远长于 768 拍）、随机像素、每隔几帧在帧首随机改一次
//  亮度/对比度/色温/饱和度/风格（OSD 改参数的时机），两版都应在同一个消隐期建完表。
//==============================================================================
module tb_pp_equiv;

localparam integer HACT = 64, HTOT = 96, VACT = 16, VTOT = 40;   // 一帧 3840 拍，消隐 ≫ 768

reg clk = 0, rst = 1;
always #20 clk = ~clk;

reg         hs = 0, vs = 0, de = 0;
reg  [23:0] rgb = 0;
reg  [6:0]  bright = 50, contrast = 50, temp = 50, sat = 50;
reg  [1:0]  style = 0;
reg         tgl = 0;

wire        n_hs, n_vs, n_de, r_hs, r_vs, r_de;
wire [23:0] n_rgb, r_rgb;

video_postproc     u_new (.video_clk(clk), .rst(rst), .in_hs(hs), .in_vs(vs), .in_de(de), .in_rgb(rgb),
                          .pic_bright(bright), .pic_contrast(contrast), .pic_temp(temp), .pic_sat(sat),
                          .pic_style(style), .pic_tgl(tgl),
                          .out_hs(n_hs), .out_vs(n_vs), .out_de(n_de), .out_rgb(n_rgb));
video_postproc_ref u_ref (.video_clk(clk), .rst(rst), .in_hs(hs), .in_vs(vs), .in_de(de), .in_rgb(rgb),
                          .pic_bright(bright), .pic_contrast(contrast), .pic_temp(temp), .pic_sat(sat),
                          .pic_style(style), .pic_tgl(tgl),
                          .out_hs(r_hs), .out_vs(r_vs), .out_de(r_de), .out_rgb(r_rgb));

// ---- 扫描 + 随机像素 ----
integer hc = 0, vc = 0, frame = 0, seed = 5;
always @(posedge clk) begin
    if (rst) begin hc <= 0; vc <= 0; end
    else begin
        if (hc == HTOT - 1) begin
            hc <= 0;
            if (vc == VTOT - 1) begin vc <= 0; frame <= frame + 1; end
            else vc <= vc + 1;
        end else hc <= hc + 1;
        de  <= (hc < HACT) && (vc < VACT);
        hs  <= (hc >= HACT + 8) && (hc < HACT + 20);
        vs  <= (vc >= VACT + 4) && (vc < VACT + 6);
        rgb <= $random(seed);
    end
end

// ---- 帧首改参数（每 3 帧一次）----
integer last_f = -1;
always @(posedge clk) if (!rst && hc == 2 && vc == 0 && frame != last_f) begin
    last_f <= frame;
    if (frame % 3 == 1) begin
        bright   <= $random(seed) % 101;  if (bright > 100) bright <= 100;
        contrast <= ($random(seed) & 32'h7FFF_FFFF) % 101;
        temp     <= ($random(seed) & 32'h7FFF_FFFF) % 101;
        sat      <= ($random(seed) & 32'h7FFF_FFFF) % 101;
        style    <= $random(seed);
        tgl      <= ~tgl;
    end
end

// ---- 逐拍比较 ----
integer errors = 0, cyc = 0, npx = 0;
// 帧 0 不比：上电第一张表还在建（新版 768 拍 / 旧版 256 拍），这期间读到的是未建好的表项
// （板上 = 上电后最初 ~30µs，那时还没有任何图像上屏）。从帧 1 起逐拍比较。
always @(posedge clk) if (!rst && frame >= 1) begin
    cyc = cyc + 1;
    if (r_de) npx = npx + 1;
    if ({n_hs, n_vs, n_de} !== {r_hs, r_vs, r_de} || (r_de && (n_rgb !== r_rgb))) begin
        errors = errors + 1;
        if (errors <= 20)
            $display("  *** FAIL  cyc %0d frame %0d: new=%b%b%b %h  ref=%b%b%b %h",
                     cyc, frame, n_hs, n_vs, n_de, n_rgb, r_hs, r_vs, r_de, r_rgb);
    end
end

initial begin
    repeat (10) @(posedge clk); rst = 0;
    wait (frame == 60);
    $display("== %0d cycles, %0d active pixels, %0d frames compared ==", cyc, npx, frame);
    if (npx < 10000) begin errors = errors + 1; $display("  *** FAIL  stimulus too weak"); end
    if (errors == 0) $display("  ALL PASS  (outputs identical every cycle)");
    else             $display("  FAILED    %0d mismatches", errors);
    $finish;
end

endmodule
