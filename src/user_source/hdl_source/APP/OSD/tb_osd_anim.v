`timescale 1ns/1ps
//==============================================================================
//  tb_osd_anim.v —— 带真实帧时序的 OSD 行为探针
//==============================================================================
//  不动像素扫描（布局 FSM 与 x/y 无关），只跑"帧节奏"：
//      ps_start=1 持续 ACTIVE 拍（产一帧），然后 0 持续 BLANK 拍（帧间空隙）。
//  每帧末尾打印：游标各通道 / 滚动 / 焦点状态 / cell 表关键条目。
//  用来回答：动画到底推了几步、滚到底有没有重置、控件表有没有被重写。
//==============================================================================
module tb_osd_anim;

parameter integer ACTIVE = 40000;    // 帧内（ps_start=1）
parameter integer BLANK  = 8000;     // 帧间（ps_start=0）

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
//  一帧
//------------------------------------------------------------------------------
integer fno = 0;
task frame;
    begin
        @(negedge clk); ps_start = 1'b1;
        repeat (ACTIVE) @(posedge clk);
        @(negedge clk); ps_start = 1'b0;
        repeat (BLANK)  @(posedge clk);
        fno = fno + 1;
    end
endtask

//------------------------------------------------------------------------------
//  按一下键（1 拍 press，level 全程为 0=按住）
//------------------------------------------------------------------------------
task key;
    input [3:0] k;
    begin
        @(negedge clk);
        key_level  = 4'h0;
        key_press  = k;
        @(posedge clk);
        @(negedge clk);
        key_press  = 4'h0;
        @(posedge clk);
        @(negedge clk);
        key_level  = 4'hF;
        repeat (4) @(posedge clk);
    end
endtask

//------------------------------------------------------------------------------
//  状态打印
//------------------------------------------------------------------------------
task dump;
    input [127:0] tag;
    reg [9:0] gx0, gy0, gx1, gy1;
    begin
        gx1 = dut.u_gfx.u_mem.cell_r[15][37:28];
        gy1 = dut.u_gfx.u_mem.cell_r[15][27:19];
        gx0 = dut.u_gfx.u_mem.cell_r[15][18:9];
        gy0 = dut.u_gfx.u_mem.cell_r[15][8:0];
        $display("f%0d %-10s | cx=%4d cw=%4d cy=%4d ch=%4d sc=%4d | pg=%0d nav=%0d it=%0d scr=%0d foc=%0d ed=%0d dirty=%0d | cur(%0d,%0d)-(%0d,%0d) lr=%b",
            fno, tag,
            dut.u_menu.cx_c, dut.u_menu.cw_c, dut.u_menu.cy_c, dut.u_menu.ch_c, dut.u_menu.sc_c,
            dut.u_menu.page, dut.u_menu.nav_idx, dut.u_menu.item, dut.u_menu.scroll_row,
            dut.u_menu.focus, dut.u_menu.edit, dut.u_menu.dirty,
            gx0, gy0, gx1, gy1, dut.txt_line_ready);
    end
endtask

initial begin
    #200 rst = 0;
    #100 menu_on = 1'b1;

    $display("==== 打开菜单，跑 6 帧看游标是否到位 ====");
    dump("t0");
    repeat (6) begin frame; dump("open"); end

    $display("==== 按 KEY4(右) 进入 page，逐帧看游标动画 ====");
    key(4'h8);
    repeat (14) begin frame; dump("->page"); end

    $display("==== 按 KEY2(下) 5 次，逐帧看滚动 ====");
    key(4'h4); repeat (3) begin frame; dump("dn1"); end
    key(4'h4); repeat (3) begin frame; dump("dn2"); end
    key(4'h4); repeat (3) begin frame; dump("dn3"); end
    key(4'h4); repeat (4) begin frame; dump("dn4"); end
    key(4'h4); repeat (6) begin frame; dump("dn5"); end

    $display("==== 再进一格（滚动窗口应跟着走） ====");
    key(4'h4); repeat (6) begin frame; dump("dn6"); end

    $display("---- 字符串 RAM（行 0..5，前 8 字符） ----");
    $display("  L0 len=%0d rdy=%0d : %04x %04x %04x %04x %04x %04x %04x %04x",
        dut.u_fc.line_len_r[0], dut.u_fc.line_rdy_r[0],
        dut.u_fc.str_mem[0],  dut.u_fc.str_mem[1],  dut.u_fc.str_mem[2],  dut.u_fc.str_mem[3],
        dut.u_fc.str_mem[4],  dut.u_fc.str_mem[5],  dut.u_fc.str_mem[6],  dut.u_fc.str_mem[7]);
    $display("  L1 len=%0d rdy=%0d : %04x %04x %04x %04x %04x %04x %04x %04x",
        dut.u_fc.line_len_r[1], dut.u_fc.line_rdy_r[1],
        dut.u_fc.str_mem[16], dut.u_fc.str_mem[17], dut.u_fc.str_mem[18], dut.u_fc.str_mem[19],
        dut.u_fc.str_mem[20], dut.u_fc.str_mem[21], dut.u_fc.str_mem[22], dut.u_fc.str_mem[23]);
    $display("  L2 len=%0d rdy=%0d : %04x %04x %04x %04x %04x %04x %04x %04x",
        dut.u_fc.line_len_r[2], dut.u_fc.line_rdy_r[2],
        dut.u_fc.str_mem[32], dut.u_fc.str_mem[33], dut.u_fc.str_mem[34], dut.u_fc.str_mem[35],
        dut.u_fc.str_mem[36], dut.u_fc.str_mem[37], dut.u_fc.str_mem[38], dut.u_fc.str_mem[39]);
    $display("  L3 len=%0d rdy=%0d : %04x %04x %04x %04x %04x %04x %04x %04x",
        dut.u_fc.line_len_r[3], dut.u_fc.line_rdy_r[3],
        dut.u_fc.str_mem[48], dut.u_fc.str_mem[49], dut.u_fc.str_mem[50], dut.u_fc.str_mem[51],
        dut.u_fc.str_mem[52], dut.u_fc.str_mem[53], dut.u_fc.str_mem[54], dut.u_fc.str_mem[55]);
    $display("  L4 len=%0d rdy=%0d : %04x %04x %04x %04x %04x %04x %04x %04x",
        dut.u_fc.line_len_r[4], dut.u_fc.line_rdy_r[4],
        dut.u_fc.str_mem[64], dut.u_fc.str_mem[65], dut.u_fc.str_mem[66], dut.u_fc.str_mem[67],
        dut.u_fc.str_mem[68], dut.u_fc.str_mem[69], dut.u_fc.str_mem[70], dut.u_fc.str_mem[71]);
    $display("  L5 len=%0d rdy=%0d : %04x %04x %04x %04x %04x %04x %04x %04x",
        dut.u_fc.line_len_r[5], dut.u_fc.line_rdy_r[5],
        dut.u_fc.str_mem[80], dut.u_fc.str_mem[81], dut.u_fc.str_mem[82], dut.u_fc.str_mem[83],
        dut.u_fc.str_mem[84], dut.u_fc.str_mem[85], dut.u_fc.str_mem[86], dut.u_fc.str_mem[87]);

    $display("---- 控件表 9..13（行控件）----");
    $display("  c9  ty=%0d sty=%0d v0=%0d geo=(%0d,%0d)-(%0d,%0d)",
        dut.u_gfx.u_mem.cell_r[9][60:57],  dut.u_gfx.u_mem.cell_r[9][56:53],  dut.u_gfx.u_mem.cell_r[9][48:39],
        dut.u_gfx.u_mem.cell_r[9][18:9],   dut.u_gfx.u_mem.cell_r[9][8:0],
        dut.u_gfx.u_mem.cell_r[9][37:28],  dut.u_gfx.u_mem.cell_r[9][27:19]);
    $display("  c10 ty=%0d sty=%0d v0=%0d geo=(%0d,%0d)-(%0d,%0d)",
        dut.u_gfx.u_mem.cell_r[10][60:57], dut.u_gfx.u_mem.cell_r[10][56:53], dut.u_gfx.u_mem.cell_r[10][48:39],
        dut.u_gfx.u_mem.cell_r[10][18:9],  dut.u_gfx.u_mem.cell_r[10][8:0],
        dut.u_gfx.u_mem.cell_r[10][37:28], dut.u_gfx.u_mem.cell_r[10][27:19]);
    $display("  c11 ty=%0d sty=%0d v0=%0d geo=(%0d,%0d)-(%0d,%0d)",
        dut.u_gfx.u_mem.cell_r[11][60:57], dut.u_gfx.u_mem.cell_r[11][56:53], dut.u_gfx.u_mem.cell_r[11][48:39],
        dut.u_gfx.u_mem.cell_r[11][18:9],  dut.u_gfx.u_mem.cell_r[11][8:0],
        dut.u_gfx.u_mem.cell_r[11][37:28], dut.u_gfx.u_mem.cell_r[11][27:19]);
    $display("  c12 ty=%0d sty=%0d v0=%0d geo=(%0d,%0d)-(%0d,%0d)",
        dut.u_gfx.u_mem.cell_r[12][60:57], dut.u_gfx.u_mem.cell_r[12][56:53], dut.u_gfx.u_mem.cell_r[12][48:39],
        dut.u_gfx.u_mem.cell_r[12][18:9],  dut.u_gfx.u_mem.cell_r[12][8:0],
        dut.u_gfx.u_mem.cell_r[12][37:28], dut.u_gfx.u_mem.cell_r[12][27:19]);
    $display("  c13 ty=%0d sty=%0d v0=%0d geo=(%0d,%0d)-(%0d,%0d)",
        dut.u_gfx.u_mem.cell_r[13][60:57], dut.u_gfx.u_mem.cell_r[13][56:53], dut.u_gfx.u_mem.cell_r[13][48:39],
        dut.u_gfx.u_mem.cell_r[13][18:9],  dut.u_gfx.u_mem.cell_r[13][8:0],
        dut.u_gfx.u_mem.cell_r[13][37:28], dut.u_gfx.u_mem.cell_r[13][27:19]);

    $display("==== 切子菜单：KEY3(左) 回 NAV，KEY2 换页，再看 scroll ====");
    key(4'h2); repeat (4) begin frame; dump("<-nav"); end
    key(4'h4); repeat (8) begin frame; dump("pg1"); end

    $finish;
end

endmodule
