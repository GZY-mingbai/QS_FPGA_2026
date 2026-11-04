`timescale 1ns/1ps
//==============================================================================
//  tb_osd_style.v —— 样式表 / 圆角 的纯组合回归（毫秒级，不含像素通路）
//==============================================================================
//  为什么单独做一个台：
//    "分割线有杂像素""页面板有多余圆角"这两个问题上板才有、出图看不出来，
//    但它们的**根因都在本模块的两个常量/判据里**，完全可以用组合探测钉死：
//
//    ① 相邻的细特征必须**同透过程度**（level 相同），只允许颜色差固定一档。
//       否则它和旁边面板的颜色差会随底图明暗反号 ⇒ 看起来一会儿暗一会儿亮。
//    ② grad=0 时四个角**必须实心**（on=1），不能有任何切角。
//    ③ grad≠0 时又要能正确切角（确认判据没被"修死"）。
//
//  用法：iverilog -g2012 -o s.vvp -s tb_osd_style tb_osd_style.v osd_widget.v
//        vvp s.vvp
//==============================================================================
module tb_osd_style;

reg  [3:0]  wtype, wstyle, wrad;
reg  [1:0]  wmask;
reg  [9:0]  v0, lx, ly, w, h;
wire        on;
wire [3:0]  lv;
wire [23:0] rgb;

integer fails = 0, checks = 0;
integer lv_panel, lv_div, lv_bar;
integer rgb_panel, rgb_div;

osd_widget dut (
    .wtype(wtype), .wstyle(wstyle), .wrad(wrad), .wmask(wmask), .v0(v0),
    .lx(lx), .ly(ly), .w(w), .h(h),
    // ★ 2026-10-29：widget 新增绝对坐标输入。这里取 gx0=gy0=0、gx1=w、gy1=h、
    //   px=lx、py=ly ⇒ 与改动前逐位等价（lxr=w-1-lx、lyb=h-1-ly、cya=h>>1、
    //   hix=tx1、v0a=v0）。穷举台靠这组代入保持"测的是同一件事"。
    .px(lx), .py(ly), .gx0(10'd0), .gx1(w), .gy0(9'd0), .gy1(h),
    .on(on), .o_level(lv), .o_rgb(rgb)
);

task chk;
    input [8*72:1] name;
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

// 取某个样式在 Solid 下的 level / 颜色（矩形内部，非边框、无圆角）
task probe_style;
    input [3:0] s;
    begin
        wtype = 4'd0; wstyle = s; wrad = 4'd0; wmask = 2'd0; v0 = 10'd0;
        w = 10'd8; h = 10'd8; lx = 10'd3; ly = 10'd3;
        #1;
    end
endtask

initial begin
    $display("======================================================");
    $display("  tb_osd_style : 样式表 / 圆角 组合回归");

    //------------------------------------------------------------ ① 同透过程度
    $display("--- A. 分隔线必须和左右两侧（面板/左栏）同 level ---");
    probe_style(4'd1);  lv_bar = lv;  rgb_panel = rgb;
    probe_style(4'd10); lv_div = lv;  rgb_div   = rgb;
    $display("      style1 (面板/左栏) level=%0d rgb=%06h", lv_bar,   rgb_panel);
    $display("      style10(分隔线)    level=%0d rgb=%06h", lv_div,   rgb_div);
    // ★ 核心不变量：level 必须相同（都 6 = 75%），只有颜色差
    chk("A1 divider level == panel level", lv_div, lv_bar);
    chk("A2 divider level == 6 (75%)",     lv_div, 6);
    chk("A3 divider brighter than panel",  (rgb_div > rgb_panel) ? 1 : 0, 1);
    // 亮度差要够明显（>= 24 阶），否则看不出是"线"
    chk("A4 brightness gap >= 24", (((rgb_div >> 16) - (rgb_panel >> 16)) >= 24) ? 1 : 0, 1);

    //------------------------------------------------------------ ② grad=0 无圆角
    //  ★ 不能用 on 判：圆角外侧是**故意** on=1 并继承下层颜色的（否则角上会漏底图）。
    //    所以"有没有被切"要看**颜色**是不是样式色。
    $display("--- B. grad=0 时四角必须画成样式色（页面板不能有圆角）---");
    wtype = 4'd0; wstyle = 4'd3; wrad = 4'd0; wmask = 2'd0; v0 = 10'd0;   // style3 = F0F0F0
    w = 10'd64; h = 10'd64;
    lx = 10'd0;  ly = 10'd0;  #1; chk("B1 TL is style colour", (rgb == 24'hF0F0F0) ? 1 : 0, 1);
    lx = 10'd63; ly = 10'd0;  #1; chk("B2 TR is style colour", (rgb == 24'hF0F0F0) ? 1 : 0, 1);
    lx = 10'd0;  ly = 10'd63; #1; chk("B3 BL is style colour", (rgb == 24'hF0F0F0) ? 1 : 0, 1);
    lx = 10'd63; ly = 10'd63; #1; chk("B4 BR is style colour", (rgb == 24'hF0F0F0) ? 1 : 0, 1);
    lx = 10'd1;  ly = 10'd1;  #1; chk("B5 (1,1) also solid",  (rgb == 24'hF0F0F0) ? 1 : 0, 1);
    lx = 10'd0;  ly = 10'd32; #1; chk("B6 left edge solid",   (rgb == 24'hF0F0F0) ? 1 : 0, 1);

    //------------------------------------------------------------ ③ grad≠0 会切角
    $display("--- C. grad=8 时角上必须是\"继承色\"而不是样式色（判据没被修死）---");
    wrad = 4'd8;
    lx = 10'd0; ly = 10'd0; #1;
    chk("C1 TL cut (inherits, not style)", (rgb != 24'hF0F0F0) ? 1 : 0, 1);
    chk("C2 TL inherits panel 0E0E0E",     (rgb == 24'h0E0E0E) ? 1 : 0, 1);
    lx = 10'd32; ly = 10'd32; #1;
    chk("C3 centre still style colour",    (rgb == 24'hF0F0F0) ? 1 : 0, 1);
    wmask = 2'd1; lx = 10'd0; ly = 10'd0; #1;
    chk("C4 corner inherits mask level", (lv == 4'd4) ? 1 : 0, 1);

    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

endmodule
