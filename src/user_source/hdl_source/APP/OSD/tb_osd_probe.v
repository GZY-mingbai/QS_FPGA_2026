`timescale 1ns/1ps
//==============================================================================
//  tb_osd_probe.v —— 定点探针：给定 (x,y) 直接打印 osd_pix 的原始 {A,RGB}
//                    以及命中的控件槽号 / 该槽的几何与属性
//==============================================================================
module tb_osd_probe;

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
wire        f_cs, f_sclk, f_mosi, f_wp, f_hold, f_miso;

osd_top #(.NL(8), .MAXLEN(32), .NSLOT(21), .NSLOT_F(63), .SELFTEST(0)) dut (
    .clk(clk), .rst(rst),
    .ps_start(ps_start), .pixel_ready(pixel_ready),
    .x(px), .y(py),
    .key_press(key_press), .key_level(key_level), .bright_eff(7'd50),
    .menu_active(menu_on), .osd_allow(1'b1), .info_show(1'b0),
    .bp_req(1'b0), .bp_addr(24'd0), .bp_len(16'd0),
    .bp_rdy(), .bp_data(), .bp_valid(), .bp_done(),
    .flash_cs_n(f_cs), .flash_sclk(f_sclk), .flash_mosi(f_mosi),
    .flash_wp_n(f_wp), .flash_hold_n(f_hold), .flash_miso(f_miso),
    .enter_req(enter_req), .exit_req(exit_req),
    .osd_layer_en(osd_layer_en), .fade(fade), .osd_pix(osd_pix),
    .fx_sel(), .auto_mode(),.info_on(),
    .album_prev(), .album_next(),
    .cur_w(16'd640), .cur_h(16'd480), .auto_remain(6'd0), .auto_bar(9'd0),
    .txt_trig()
);

w25q64_model_24b #(.MEM_BYTES(262144), .INIT_HEX("tools/font.hex")) u_flash (
    .cs_n(f_cs), .sclk(f_sclk), .mosi(f_mosi), .miso(f_miso)
);

task nframe;
    input integer n;
    integer i;
    begin
        for (i = 0; i < n; i = i + 1) begin
            @(negedge clk); ps_start = 1'b1; px = 0; py = 0;
            repeat (3000) @(posedge clk);
            @(negedge clk); ps_start = 1'b0;
            repeat (4000) @(posedge clk);
        end
    end
endtask

task key;
    input [3:0] k;
    begin
        @(negedge clk);
        key_level = 4'h0; key_press = k;
        @(posedge clk);
        @(negedge clk);
        key_press = 4'h0;
        @(posedge clk);
        @(negedge clk);
        key_level = 4'hF;
        repeat (6) @(posedge clk);
    end
endtask

// 只打印命中的文字像素（A==255）
task scan;
    input integer x0, x1, yv, st;
    input [8*20:1] tag;
    integer xx;
    reg [31:0] gp;
    begin
        $write("  %0s :", tag);
        for (xx = x0; xx <= x1; xx = xx + st) begin
            @(negedge clk); px = xx; py = yv; ps_start = 1'b1;
            repeat (4) @(posedge clk);
            gp = osd_pix;
            if (gp[31:24] == 8'd255) $write(" %0d:%02x%02x%02x", xx, gp[23:16], gp[15:8], gp[7:0]);
        end
        $write("\n");
        @(negedge clk); ps_start = 1'b0;
        repeat (2) @(posedge clk);
    end
endtask

reg [31:0] gp;
task pb;
    input [15:0] ax;
    input [15:0] ay;
    input [8*20:1] tag;
    begin
        @(negedge clk); px = ax; py = ay; ps_start = 1'b1;
        repeat (4) @(posedge clk);
        gp = osd_pix;
        @(negedge clk); ps_start = 1'b0;
        repeat (2) @(posedge clk);
        $display("  %-18s (%3d,%3d) A=%3d RGB=%02x%02x%02x | hit=%b idx=%0d ty=%0d sty=%0d geo=(%0d,%0d)-(%0d,%0d)",
            tag, ax, ay, gp[31:24], gp[23:16], gp[15:8], gp[7:0],
            dut.u_gfx.hit_v, dut.u_gfx.hit_idx,
            dut.u_gfx.rd_attr[27:24], dut.u_gfx.rd_attr[23:20],
            dut.u_gfx.rd_geo[18:9], dut.u_gfx.rd_geo[8:0],
            dut.u_gfx.rd_geo[37:28], dut.u_gfx.rd_geo[27:19]);
    end
endtask

integer i;
initial begin
    #200 rst = 0;
    #100 menu_on = 1'b1;
    nframe(40);

    $display("==== A. 打开菜单，焦点在左栏 ====");
    pb(300, 300, "page empty");
    pb(500, 10,  "scrim top");
    pb(50,  240, "nav bar bg");
    pb(97,  240, "divider");
    pb(150, 20,  "title area");
    pb(300, 44,  "title line");
    pb(32,  40,  "nav icon 0 edge");
    pb(40,  40,  "nav icon 0 in");
    pb(120, 40,  "nav icon 1?");
    pb(56,  40,  "cursor right edge");
    pb(28,  40,  "cursor left edge");
    pb(320, 66,  "ctrl0 edge");
    pb(340, 71,  "ctrl0 fill(60%)");
    pb(560, 71,  "ctrl0 track");
    pb(492, 71,  "ctrl0 knob");
    pb(350, 231, "ctrl2 combo in");
    pb(330, 311, "ctrl3 check in");
    pb(350, 391, "ctrl4 prog fill");
    pb(368, 311, "ctrl3 val txt");
    pb(368, 231, "ctrl2 val txt");
    pb(296, 71,  "ctrl0 num txt");

    $display("==== B. 进 page（KEY4）后 ====");
    key(4'h8);
    nframe(14);
    pb(300, 300, "page empty");
    pb(320, 66,  "ctrl0 edge");
    pb(340, 71,  "ctrl0 fill");
    pb(350, 231, "ctrl2 combo");
    pb(330, 311, "ctrl3 check");
    pb(455, 71,  "row0 highlight?");
    pb(110, 60,  "cursor top edge");

    $display("==== B2. 下拉值文字（第16格 = x368..384，行2 y224..240）====");
    $display("  line_ready=%b  line_len=%h", dut.txt_line_ready, dut.txt_line_len);
    pb(372, 232, "combo char16 a");
    pb(376, 232, "combo char16 b");
    pb(380, 236, "combo char16 c");
    pb(376, 228, "combo char16 d");
    pb(368, 232, "combo char16 e");
    pb(272, 232, "row0 num tens");
    pb(288, 232, "row0 num ones");
    pb(304, 232, "row0 num pct");
    pb(132, 232, "row2 label c0");
    pb(148, 232, "row2 label c1");

    $display("  ---- 逐行扫描：只打印 A==255(文字) 的像素 ----");
    scan(112, 190, 232, 2, "row2 label y232");
    scan(112, 190, 236, 2, "row2 label y236");
    scan(360, 392, 232, 2, "row2 char16 y232");
    scan(360, 392, 236, 2, "row2 char16 y236");
    scan(112, 190,  72, 2, "row0 label y72");
    scan(260, 330,  72, 2, "row0 num y72");

    $display("==== C. 控件表 dump ====");
    $display(" c0 ty=%0d sty=%0d geo=(%0d,%0d)-(%0d,%0d)",
        dut.u_gfx.u_mem.cell_r[0][66:63], dut.u_gfx.u_mem.cell_r[0][62:59],
        dut.u_gfx.u_mem.cell_r[0][18:9], dut.u_gfx.u_mem.cell_r[0][8:0],
        dut.u_gfx.u_mem.cell_r[0][37:28], dut.u_gfx.u_mem.cell_r[0][27:19]);
    $display(" c1 ty=%0d sty=%0d geo=(%0d,%0d)-(%0d,%0d)",
        dut.u_gfx.u_mem.cell_r[1][66:63], dut.u_gfx.u_mem.cell_r[1][62:59],
        dut.u_gfx.u_mem.cell_r[1][18:9], dut.u_gfx.u_mem.cell_r[1][8:0],
        dut.u_gfx.u_mem.cell_r[1][37:28], dut.u_gfx.u_mem.cell_r[1][27:19]);
    $display(" c3 ty=%0d sty=%0d geo=(%0d,%0d)-(%0d,%0d)",
        dut.u_gfx.u_mem.cell_r[3][66:63], dut.u_gfx.u_mem.cell_r[3][62:59],
        dut.u_gfx.u_mem.cell_r[3][18:9], dut.u_gfx.u_mem.cell_r[3][8:0],
        dut.u_gfx.u_mem.cell_r[3][37:28], dut.u_gfx.u_mem.cell_r[3][27:19]);
    $display(" c9 ty=%0d sty=%0d v0=%0d geo=(%0d,%0d)-(%0d,%0d)",
        dut.u_gfx.u_mem.cell_r[9][66:63], dut.u_gfx.u_mem.cell_r[9][62:59], dut.u_gfx.u_mem.cell_r[9][52:39],
        dut.u_gfx.u_mem.cell_r[9][18:9], dut.u_gfx.u_mem.cell_r[9][8:0],
        dut.u_gfx.u_mem.cell_r[9][37:28], dut.u_gfx.u_mem.cell_r[9][27:19]);
    $display(" c15(cursor) geo=(%0d,%0d)-(%0d,%0d) cx_c=%0d cw_c=%0d cy_c=%0d ch_c=%0d",
        dut.u_gfx.u_mem.cell_r[15][18:9], dut.u_gfx.u_mem.cell_r[15][8:0],
        dut.u_gfx.u_mem.cell_r[15][37:28], dut.u_gfx.u_mem.cell_r[15][27:19],
        dut.u_menu.cx_c, dut.u_menu.cw_c, dut.u_menu.hl_c, dut.u_menu.ch_c);

    $finish;
end

endmodule
