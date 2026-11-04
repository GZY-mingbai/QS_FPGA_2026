`timescale 1ns/1ps
//==============================================================================
//  tb_menu_equiv.v —— osd_menu 重构的【逐像素等价】验证
//==============================================================================
//  【为什么这样做】`txt_xy` / `txt_col` 从"每行存 32 位"改成"紧凑存储 + 组合展开"，
//  是纯重构，但影响的是**可见文字的位置和颜色**。断言抓不到"整体偏 4px"这类错，
//  只有逐像素比对才算证据。
//
//  【做法】同一份激励同时喂给两个 osd_top：
//      dut_new : 当前 src 里的 osd_menu
//      dut_ref : .backup/hdl_source_P0 里的 osd_menu（改动前，模块名改成 osd_menu_ref）
//  每拍比较 `osd_pix`（32 位合成像素）与 `osd_layer_en`。
//  两者逐拍完全相同 ⇒ 行为逐像素等价。
//
//  【激励】菜单开关 / 翻页 / 上下滚动 / 打开弹层 / 进出编辑 / 加减值 / 切换页面
//==============================================================================
module tb_menu_equiv;

localparam integer NL = 8;

reg clk = 0;
always #10 clk = ~clk;

reg rst = 1;
reg ps_start = 0;
reg pixel_ready = 1;
reg [15:0] px = 0, py = 0;
reg [3:0] key_press = 0;
reg [3:0] key_level = 4'hF;
reg menu_on = 0;
reg info_show = 0;
reg [15:0] cur_w = 16'd640, cur_h = 16'd480;
reg [5:0] auto_remain = 6'd0;
reg [8:0] auto_bar = 9'd0;

// ---- 新实现 ----
wire        n_enter, n_exit, n_osd_en;
wire [7:0]  n_fade;
wire [31:0] n_pix;
wire        n_fcs, n_fsclk, n_fmosi, n_fwp, n_fhold, n_fmiso;
wire [2:0]  n_fx, n_auto;
wire        n_info, n_prev, n_next;
wire        n_bpr, n_bpv, n_bpd;
wire [7:0]  n_bpd8;

osd_top #(.NL(NL), .MAXLEN(32), .NSLOT(20), .NSLOT_F(63), .SELFTEST(0),
          .RPT_FIRST(2), .RPT_EVERY(1)) dut_new (
    .clk(clk), .rst(rst), .ps_start(ps_start), .pixel_ready(pixel_ready),
    .x(px), .y(py), .key_press(key_press), .key_level(key_level), .bright_eff(7'd50),
    .menu_active(menu_on), .osd_allow(1'b1), .info_show(info_show),
    .flash_cs_n(n_fcs), .flash_sclk(n_fsclk), .flash_mosi(n_fmosi),
    .flash_wp_n(n_fwp), .flash_hold_n(n_fhold), .flash_miso(n_fmiso),
    .bp_req(1'b0), .bp_addr(24'd0), .bp_len(16'd0),
    .bp_rdy(n_bpr), .bp_data(n_bpd8), .bp_valid(n_bpv), .bp_done(n_bpd),
    .enter_req(n_enter), .exit_req(n_exit),
    .osd_layer_en(n_osd_en), .fade(n_fade), .osd_pix(n_pix),
    .fx_sel(n_fx), .auto_mode(n_auto),.info_on(n_info),
    .album_prev(n_prev), .album_next(n_next),
    .cur_w(cur_w), .cur_h(cur_h), .auto_remain(auto_remain), .auto_bar(auto_bar)
);
w25q64_model_24b #(.MEM_BYTES(262144), .INIT_HEX("tools/font.hex")) u_fl_new (
    .cs_n(n_fcs), .sclk(n_fsclk), .mosi(n_fmosi), .miso(n_fmiso));

// ---- 参考实现（改动前）----
wire        r_enter, r_exit, r_osd_en;
wire [7:0]  r_fade;
wire [31:0] r_pix;
wire        r_fcs, r_fsclk, r_fmosi, r_fwp, r_fhold, r_fmiso;
wire [2:0]  r_fx, r_auto;
wire        r_info, r_prev, r_next;
wire        r_bpr, r_bpv, r_bpd;
wire [7:0]  r_bpd8;

osd_top_ref #(.NL(NL), .MAXLEN(32), .NSLOT(20), .NSLOT_F(63), .SELFTEST(0),
              .RPT_FIRST(2), .RPT_EVERY(1)) dut_ref (
    .clk(clk), .rst(rst), .ps_start(ps_start), .pixel_ready(pixel_ready),
    .x(px), .y(py), .key_press(key_press), .key_level(key_level),
    .menu_active(menu_on), .osd_allow(1'b1), .info_show(info_show),
    .flash_cs_n(r_fcs), .flash_sclk(r_fsclk), .flash_mosi(r_fmosi),
    .flash_wp_n(r_fwp), .flash_hold_n(r_fhold), .flash_miso(r_fmiso),
    .bp_req(1'b0), .bp_addr(24'd0), .bp_len(16'd0),
    .bp_rdy(r_bpr), .bp_data(r_bpd8), .bp_valid(r_bpv), .bp_done(r_bpd),
    .enter_req(r_enter), .exit_req(r_exit),
    .osd_layer_en(r_osd_en), .fade(r_fade), .osd_pix(r_pix),
    .fx_sel(r_fx), .auto_mode(r_auto),.info_on(r_info),
    .album_prev(r_prev), .album_next(r_next),
    .cur_w(cur_w), .cur_h(cur_h), .auto_remain(auto_remain), .auto_bar(auto_bar)
);
w25q64_model_24b #(.MEM_BYTES(262144), .INIT_HEX("tools/font.hex")) u_fl_ref (
    .cs_n(r_fcs), .sclk(r_fsclk), .mosi(r_fmosi), .miso(r_fmiso));

//==============================================================================
//  逐拍比对
//==============================================================================
integer n_cmp = 0, n_bad = 0;
integer n_osd_on = 0;      // OSD 有内容（en=1）的拍数，确保真的比到了东西
always @(posedge clk) begin
    if (!rst) begin
        n_cmp = n_cmp + 1;
        if (n_osd_en !== r_osd_en || n_pix !== r_pix) begin
            n_bad = n_bad + 1;
            if (n_bad <= 10)
                $display("   DIFF t=%0t  xy=(%0d,%0d) new={en=%b,%h} ref={en=%b,%h}",
                         $time, px, py, n_osd_en, n_pix, r_osd_en, r_pix);
        end
        if (n_osd_en && (n_pix !== 32'h0000_0000)) n_osd_on = n_osd_on + 1;
    end
end

//==============================================================================
//  扫描 + 按键
//==============================================================================
task scan_frame;
    integer yy, xx;
    begin
        ps_start = 1;
        for (yy = 479; yy >= 0; yy = yy - 1) begin
            for (xx = 0; xx < 640; xx = xx + 1) begin
                px = xx[15:0]; py = yy[15:0];
                @(posedge clk);
            end
        end
        ps_start = 0;
        repeat (30) @(posedge clk);
    end
endtask

task kpress;
    input [3:0] k;
    begin
        @(negedge clk); key_press = k; menu_on = 1'b1;
        @(posedge clk);
        @(negedge clk); key_press = 4'h0;
        repeat (3) @(posedge clk);
    end
endtask

// 只扫 OSD 所在区域：全帧扫描 × 2 个 DUT 太慢（>8min），只扫 y 479..300
// （布局 FSM 在帧间跑，扫描区域不影响布局；扫描只用于比对像素）
task scan_menu_area;
    integer yy, xx;
    begin
        ps_start = 1;
        for (yy = 479; yy >= 300; yy = yy - 1) begin
            for (xx = 0; xx < 640; xx = xx + 1) begin
                px = xx[15:0]; py = yy[15:0];
                @(posedge clk);
            end
        end
        ps_start = 0;
        repeat (20) @(posedge clk);
    end
endtask

integer i;
initial begin
    $display("======================================================");
    $display("  tb_menu_equiv : osd_menu 重构逐像素等价");
    $display("======================================================");
    rst = 1; repeat (10) @(posedge clk); rst = 0;
    repeat (50) @(posedge clk);
    // 预排几帧让字库就绪
    for (i = 0; i < 2; i = i + 1) scan_frame;

    $display("--- A. 导航栏 ---");
    scan_frame;

    $display("--- B. 进 page0（显示页）+ 逐项上下 ---");
    kpress(4'h2);
    for (i = 0; i < 6; i = i + 1) begin kpress(4'h4); scan_menu_area; end
    for (i = 0; i < 6; i = i + 1) begin kpress(4'h8); scan_menu_area; end

    $display("--- C. 进编辑 + 加减 ---");
    kpress(4'h2); scan_menu_area;
    kpress(4'h4); scan_menu_area;
    kpress(4'h8); scan_menu_area;
    kpress(4'h1); scan_menu_area;

    $display("--- D. 打开 ComboBox 弹层 ---");
    for (i = 0; i < 4; i = i + 1) kpress(4'h4);
    kpress(4'h2); scan_menu_area;
    kpress(4'h4); scan_menu_area;
    kpress(4'h1); scan_menu_area;

    $display("--- E. 滚到底 + 回导航栏 + 换页 ---");
    for (i = 0; i < 8; i = i + 1) kpress(4'h4);
    scan_menu_area;
    kpress(4'h1); scan_menu_area;
    kpress(4'h4); kpress(4'h2); scan_menu_area;
    kpress(4'h4); kpress(4'h4); kpress(4'h2); scan_menu_area;

    $display("--- F. 信息条模式（menu_on=0）---");
    menu_on = 0; info_show = 1;
    repeat (200) @(posedge clk);
    scan_frame;

    $display("\n======================================");
    $display("  比对 %0d 拍，其中 OSD 非空 %0d 拍", n_cmp, n_osd_on);
    if (n_bad == 0) $display("  ALL PASS  (逐像素完全一致)");
    else            $display("  FAIL      (%0d 拍不同)", n_bad);
    $display("======================================");
    $finish;
end

endmodule
