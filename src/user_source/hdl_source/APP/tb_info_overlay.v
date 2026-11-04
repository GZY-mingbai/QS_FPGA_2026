//==============================================================================
//  tb_info_overlay.v —— OSD 与 B 层"逐像素叠加"的单元回归
//==============================================================================
//
//  【要证的事】切换动画期间（b_en=1）信息条必须能叠在动画上，而不是把
//    B 层整层替换掉。判据是 viewer_scene 里的两个内部量：
//        pix_b_eff : 送进 alpha 合并的前景
//        alpha_eff : 该前景的整体不透明度
//
//  【旧行为（bug）】
//        pix_b_eff = osd_on ? osd_pix : pix_b;
//    只要 OSD 层开（= 信息条开），**整幅画面**都变成 osd_pix：
//      · 信息条矩形外 osd_pix=0 ⇒ B 层动画被擦成透明 → 只剩 A 层；
//      · 信息条矩形内虽然是 OSD，但 B 层在那一块也没了。
//    所以仅把 info_show 放宽到过渡态，动画会被"吃掉"。
//
//  【新行为（本台守住，2026-11-05 改）】
//      · osd_pix 的 A==0 ⇒ 该像素仍用 B 层（动画不受影响）；
//      · A!=0 ⇒ 用 OSD 像素本身（不再预混），透明度交给合并级：
//            alpha_eff = 192（面板）/ osd_fade（文字），合并级再算
//            a9 = a1*alpha/256（a1=192 时 = 144 → 面板实际 56.25%）；
//      · 背景（s2）只看"B 是否完全落屏"：s2_b = in_box_b_p & (alpha_q==0xFF)
//        —— 与 OSD 像素是什么无关。这样面板在 A 区混 A、在 B 区混 B，
//        两侧同一种混法，不会再出现"被切成深浅两块"。
//
//  【旧的预混方案（已删，留档）】
//      A==192 且 B 在框内 ⇒ pix_b_eff = {255,(3*O+B)/4}，期望合并级直通。
//      但 alpha_eff 是 192，a9 = alpha = 192 ≠ 256 ⇒ 又被 A 层混了 25%；
//      动画期间该处 A 为空(0) ⇒ 面板背景混进 57% 的黑。这才是上板
//      "信息条底部/右半边偏深"的根因。
//
//  【怎么跑】工作目录 = APP/
//    iverilog -g2012 -s tb_info_overlay -o /tmp/tb_ov.vvp \
//        tb_info_overlay.v viewer_scene.v viewer_fx.v app_scan_gen.v \
//        img_layer_ctrl.v img_layer_hd.v ps_gen_image.v img_afifo.v \
//        ps_merge_alpha_pp.v ps_to_rgb24.v && vvp /tmp/tb_ov.vvp
//
//  说明：本台不让场景真正跑帧（ps_start 恒 0），只 force 两个内部 wire，
//    直接检查合并选择表达式本身。这样与纹理/SDRAM 无关，毫秒级。
//==============================================================================
`timescale 1ns / 1ps

module tb_info_overlay;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

// 场景输入（全部静态）
reg  [31:0] a_pos = 0, a_size = {16'd640, 16'd480}, b_pos = 0, b_size = {16'd640, 16'd480};
reg  [39:0] a_k = {20'd16384, 20'd32768}, b_k = {20'd16384, 20'd32768};
reg  b_en = 1;
reg  [7:0] alpha_b = 8'd77;
reg  osd_on = 0;
reg  [7:0] osd_fade = 8'd255;
reg  [31:0] osd_pix = 32'd0;

wire pixel_ready;
wire [23:0] ps_data;
wire pipe_ce;
wire [15:0] scan_x, scan_y;
wire [1:0] tex_req_vld, tex_pix_wr;
wire [41:0] tex_req_addr;
wire [7:0] tex_req_mask;
wire [63:0] tex_pix_data;

viewer_scene #(.IMG_W(640), .IMG_H(480)) dut (
    .clk(clk), .rst(rst), .mclk(clk),
    .ps_start(1'b0), .pixel_ready(pixel_ready),
    .ps_data(ps_data), .pipe_ce(pipe_ce),
    .fmt565(1'b1), .wr_stall(1'b0),
    .a_hd(1'b0), .a_pos(a_pos), .a_size(a_size), .a_k(a_k),
    .a_base(21'd614400), .a_en(1'b1),
    .quad_mode(1'b0),
    .b_pos(b_pos), .b_size(b_size), .b_k(b_k),
    .b_base(21'd652800), .b_en(b_en), .alpha_b(alpha_b),
    .scan_x(scan_x), .scan_y(scan_y),
    .osd_on(osd_on), .osd_fade(osd_fade), .osd_pix(osd_pix),
    .tex_req_vld(tex_req_vld), .tex_req_addr(tex_req_addr),
    .tex_req_mask(tex_req_mask), .tex_req_ack(2'b00),
    .tex_pix_wr(tex_pix_wr), .tex_pix_data(tex_pix_data)
);

//------------------------------------------------------------------------------
//  force 内部量：B 层像素 / B 的框内标志（★ 2026-11-05：改用与 pix_b 同拍的 _p 版本）
//------------------------------------------------------------------------------
reg [31:0] b_pix_f = 32'h00_80_40_20;
reg        b_inbox_f = 1'b1;

initial begin
    force dut.pix_b       = b_pix_f;
    force dut.in_box_b_p  = b_inbox_f;
end

wire [31:0] eff   = dut.pix_b_eff;
wire [7:0]  alf   = dut.alpha_eff;
wire [31:0] s2pix = dut.pix_a_eff;

//------------------------------------------------------------------------------
//  断言
//------------------------------------------------------------------------------
integer fails = 0, checks = 0;
task chk32;
    input [8*56:1] name;
    input [31:0]   got;
    input [31:0]   exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %08h, expect %08h", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %08h", name, got);
    end
endtask
task chk8;
    input [8*56:1] name;
    input [7:0]    got;
    input [7:0]    exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %0d", name, got);
    end
endtask
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
    end
endtask

initial begin
    repeat (4) @(posedge clk);
    rst = 0;
    repeat (4) @(posedge clk);

    //----------------------------------------------------------
    // A. 无 OSD：必须原样透传 B
    //----------------------------------------------------------
    $display("=== A. osd_on=0 ===");
    osd_on = 0; osd_pix = 32'hFF112233;
    #1;
    chk32("A1 B passes through", eff, 32'h00_80_40_20);
    chk8 ("A2 alpha_b passes through", alf, 8'd77);

    //----------------------------------------------------------
    // B. OSD 开但该像素透明：B 必须继续可见
    //    （旧代码在这里整层替换成 osd_pix=0 ⇒ 动画消失）
    //----------------------------------------------------------
    $display("=== B. osd_on=1, osd pixel A=0 ===");
    osd_on = 1; osd_pix = 32'h00000000;
    #1;
    chk32("B1 B survives transparent OSD pixel", eff, 32'h00_80_40_20);
    chk8 ("B2 alpha still alpha_b", alf, 8'd77);

    //----------------------------------------------------------
    // C. OSD 不透明文字：整像素用 OSD
    //----------------------------------------------------------
    $display("=== C. osd_opaque text ===");
    osd_pix = 32'hFF112233;
    #1;
    chk32("C1 opaque OSD wins", eff, 32'hFF112233);
    chk8 ("C2 alpha = fade", alf, 8'd255);

    //----------------------------------------------------------
    // D. 面板 + B 在框内 + B 完全落屏：s1 只给 osd_pix，背景取 B
    //    合并级随后算 a9 = 192*192/256 = 144 ⇒ 0.5625P + 0.4375B
    //----------------------------------------------------------
    $display("=== D. panel + B in box (alpha_b=255) ===");
    alpha_b = 8'd255;
    b_inbox_f = 1'b1; b_pix_f = 32'h00_80_40_20;
    osd_pix = 32'hC0_20_20_20;
    #1;
    chk32("D1 s1 = osd_pix (no pre-blend)", eff, 32'hC0_20_20_20);
    chk  ("D2 s2_b = 1 (background is B)",  dut.s2_b, 1);
    chk32("D3 s2 = pix_b",                  s2pix, 32'h00_80_40_20);
    chk8 ("D4 alpha = 192 (panel's own)",   alf, 8'd192);

    //----------------------------------------------------------
    // E. 面板但 B 不在框内：背景仍是 A
    //----------------------------------------------------------
    $display("=== E. panel outside B box ===");
    b_inbox_f = 1'b0;
    #1;
    chk32("E1 s1 = osd_pix", eff, 32'hC0_20_20_20);
    chk  ("E2 s2_b = 0",      dut.s2_b, 0);
    chk8 ("E3 alpha = 192",   alf, 8'd192);

    //----------------------------------------------------------
    // F. B 正在淡入（alpha_b<255）：面板不能提前混 B
    //----------------------------------------------------------
    $display("=== F. panel during B fade (alpha_b=77) ===");
    alpha_b = 8'd77;
    b_inbox_f = 1'b1;
    #1;
    chk32("F1 s1 = osd_pix", eff, 32'hC0_20_20_20);
    chk  ("F2 s2_b = 0 (B not opaque yet)", dut.s2_b, 0);

    //----------------------------------------------------------
    // G. B 框内 + 不透明文字：s2_b 照样为 1（选择器与 OSD 类型无关），
    //    但 a1=255 且 alpha=osd_fade=255 ⇒ a9=256，输出仍是文字本身。
    //    这条守住"把 B 接到 s2 不会改变任何不透明像素"。
    //----------------------------------------------------------
    $display("=== G. opaque text inside B box ===");
    alpha_b = 8'd255;
    b_inbox_f = 1'b1; osd_pix = 32'hFF445566;
    #1;
    chk  ("G1 s2_b = 1",                 dut.s2_b, 1);
    chk32("G2 s1 = text",                eff, 32'hFF445566);
    chk8 ("G3 alpha = fade (=256 path)", alf, 8'd255);

    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

endmodule
