`timescale 1ns/1ps
//==============================================================================
//  tb_osd_smoke.v —— OSD 图形层定点探针（无状态图层 ⇒ 直接给坐标即可）
//==============================================================================
//  1) 用一次 ps_start 下降沿触发布局（tick），等 FSM 把 16 条记录写完
//  2) 逐个坐标探测：给 (x,y) + ps_start=1，等 3 拍取 osd_pix
//  比全屏扫描快 3 个数量级，且判据更直接。
//==============================================================================
module tb_osd_smoke;

reg clk = 0;
always #10 clk = ~clk;

reg         rst = 1;
reg         ps_start = 0;
reg         pixel_ready = 1;
reg  [15:0] px = 0, py = 0;
reg  [3:0]  key_press = 0;
reg  [3:0]  key_level = 4'hF;
reg         menu_on = 0;

wire        enter_req, exit_req, osd_layer_en;
wire [7:0]  fade;

wire [31:0] osd_pix;
wire        f_cs, f_sclk, f_mosi, f_wp, f_hold;

osd_top #(.NL(8), .MAXLEN(32), .NSLOT(21), .NSLOT_F(63), .SELFTEST(0)) dut (
    .clk(clk), .rst(rst),
    .ps_start(ps_start), .pixel_ready(pixel_ready),
    .x(px), .y(py),
    .key_press(key_press), .key_level(key_level), .bright_eff(7'd50),
    .menu_active(menu_on), .osd_allow(1'b1), .info_show(1'b0),
    .bp_req(1'b0), .bp_addr(24'd0), .bp_len(16'd0),
    .bp_rdy(), .bp_data(), .bp_valid(), .bp_done(),
    .flash_cs_n(f_cs), .flash_sclk(f_sclk), .flash_mosi(f_mosi),
    .flash_wp_n(f_wp), .flash_hold_n(f_hold), .flash_miso(1'b0),
    .enter_req(enter_req), .exit_req(exit_req),
    .osd_layer_en(osd_layer_en), .fade(fade), .osd_pix(osd_pix),
    .fx_sel(), .auto_mode(),.info_on(),
    .album_prev(), .album_next(),
    .cur_w(16'd640), .cur_h(16'd480), .auto_remain(6'd0), .auto_bar(9'd0),
    .txt_trig()
);

//------------------------------------------------------------------------------
//  一次"帧"，只为产生 tick → 布局
//------------------------------------------------------------------------------
task do_layout;
    integer i;
    begin
        @(negedge clk); ps_start = 1; px = 0; py = 0;
        repeat (4) @(posedge clk);
        @(negedge clk); ps_start = 0;              // 落沿 → tick
        repeat (80) @(posedge clk);                // 等 FSM 写完 16 条
    end
endtask

//------------------------------------------------------------------------------
//  定点探测
//------------------------------------------------------------------------------
reg [31:0] got;
task probe;
    input [15:0] ax;
    input [15:0] ay;
    begin
        @(negedge clk); px = ax; py = ay; ps_start = 1;
        repeat (4) @(posedge clk);
        got = osd_pix;
        @(negedge clk); ps_start = 0;
        repeat (8) @(posedge clk);
    end
endtask

integer errs = 0;
task ck;
    input [15:0] ax;
    input [15:0] ay;
    input [23:0] exp_rgb;
    input [7:0]  exp_a;
    input [127:0] tag;
    begin
        probe(ax, ay);
        if ((got[23:0] !== exp_rgb) || (got[31:24] !== exp_a)) begin
            $display("FAIL %0s (%0d,%0d): got %06x/%0d exp %06x/%0d",
                     tag, ax, ay, got[23:0], got[31:24], exp_rgb, exp_a);
            errs = errs + 1;
        end else
            $display("ok   %0s (%0d,%0d) = %06x/%0d", tag, ax, ay, exp_rgb, exp_a);
    end
endtask

initial begin
    #200 rst = 0;
    #100;
    menu_on = 1;

    repeat (14) do_layout;       // fade 动画 12 帧到位

    $display("---- 探针 fade=%0d ----", fade);
    $display("val0..4 = %0d %0d %0d %0d %0d", dut.u_menu.val0, dut.u_menu.val1, dut.u_menu.val2, dut.u_menu.val3, dut.u_menu.val4);
    $display("c9 v0=%0d type=%0d", dut.u_gfx.u_mem.cell_r[9][52:39], dut.u_gfx.u_mem.cell_r[9][66:63]);
    $display("c10v0=%0d type=%0d", dut.u_gfx.u_mem.cell_r[10][52:39], dut.u_gfx.u_mem.cell_r[10][66:63]);
    $display("c11v0=%0d type=%0d", dut.u_gfx.u_mem.cell_r[11][52:39], dut.u_gfx.u_mem.cell_r[11][66:63]);
    $display("cell9 geo =%b", dut.u_gfx.u_mem.cell_r[9][38:0]);

    $display("geo0=%b geo1=%b", dut.u_gfx.u_mem.cell_r[0], dut.u_gfx.u_mem.cell_r[1]);
    $display("attr0=%b", dut.u_gfx.u_mem.cell_r[0]);
    probe(16'd50, 16'd240);
    $display("probe(50,240) got=%b  hit=%b idx=%0d rdgeo=%b rdat=%b",
             got, dut.u_gfx.hit_v, dut.u_gfx.hit_idx, dut.u_gfx.rd_geo, dut.u_gfx.rd_attr);
    ck(50,  240, 24'h0E0E0E, 8'd192, "navbg  ");   // 左栏底
    // 页面板现在从 y=0 开始（原来 gy0=12，屏幕最上面 12px 只有 50% 遮罩，
    // 上板看着就是"顶端一条更亮的横带、像没有 alpha 层"）⇒ 顶部这点必须是面板色。
    ck(500, 10,  24'h0E0E0E, 8'd192, "pagetop");   // 顶部 12px 也属于页面板
    ck(300, 300, 24'h0E0E0E, 8'd192, "pagebg ");   // 页面空白
    // 分隔线和面板必须**同透过程度**（都 75%），否则同一列上颜色差会随底图反号
    // ⇒ 上板"分割线有杂像素/虚线"。
    ck(97,  240, 24'h3A3A3A, 8'd192, "divider");
    // ★ 2026-10-27：本台只灌字库、没有图标覆盖率图 ⇒ icon_ready=0
    //   ⇒ type12 整槽不画（A=0）。原来这里期望 5A5A5A/255 —— 那正是
    //   "图标显示成纯色色块"的旧行为，属于把 bug 写成了断言。
    //   真实图标的逐像素验证见 tb_osd_icons.v。
    ck(32, 120,  24'h000000, 8'd0,   "icon no flash");
    ck(28,  40,  24'hFFFFFF, 8'd255, "cursor ");   // 游标外框（套住图标）

    if (errs == 0) $display("PASS tb_osd_smoke");
    else           $display("FAIL tb_osd_smoke (%0d)", errs);
    $finish;
end

endmodule
