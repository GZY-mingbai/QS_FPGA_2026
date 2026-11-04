//==============================================================================
//  tb_auto_popup.v —— 相册页"自动切换"下拉弹层的出图（1:1）
//==============================================================================
//  目的：把 6 项下拉滚到末页（sel=5，音乐结束），看屏幕上有几处"音乐结束"。
//  跑法（工作目录 = APP/OSD/）：
//    iverilog -g2012 -o /tmp/ta.vvp -s tb_auto_popup tb_auto_popup.v \
//        osd_top.v osd_menu.v osd_gfx.v osd_cellmem.v osd_widget.v \
//        osd_icon_rom.v flash_arb.v ../osd_font_cache.v ../ps_gen_text.v \
//        ../gb_addr.v ../../spi_flash_read.v ../w25q64_model_24b.v
//    vvp /tmp/ta.vvp
//==============================================================================
`timescale 1ns/1ps
module tb_auto_popup;

reg clk = 0; always #10 clk = ~clk;

reg         rst = 1;
reg         ps_start = 0;
reg         pixel_ready = 1;
reg  [15:0] px = 0, py = 0;
reg  [3:0]  key_press = 0;
reg  [3:0]  key_level = 4'hF;
reg         menu_on = 0;
reg         info_show_tb = 0;
reg  [5:0]  auto_remain = 6'd0;
integer     k;

wire enter_req, exit_req, osd_layer_en;
wire [7:0] fade;
wire [31:0] osd_pix;
wire f_cs, f_sclk, f_mosi, f_wp, f_hold, f_miso;

//------------------------------------------------------------------------------
//  名字 RAM 模型（★ 2026-11-02：信息条第 6 行现在是真实文件名）
//------------------------------------------------------------------------------
//  这里故意取**最长**的 8.3（主名 8 + 扩展名 3 = 12 字 "IMG_0001.BMP"）：
//  出图同时验证"面板 8..328 放得下 12 字"和"点插在主名后面"。
reg [7:0] nm_name [0:15];
integer nn;
initial begin
    for (nn = 0; nn < 16; nn = nn + 1) nm_name[nn] = 8'h20;
    nm_name[0] = "I"; nm_name[1] = "M"; nm_name[2] = "G"; nm_name[3] = "_";
    nm_name[4] = "0"; nm_name[5] = "0"; nm_name[6] = "0"; nm_name[7] = "1";
    nm_name[8] = "B"; nm_name[9] = "M"; nm_name[10] = "P";
end
wire [7:0] nm_char = nm_name[dut.u_menu.O_nm_idx[3:0]];

osd_top #(.NL(8), .MAXLEN(32), .NSLOT(20), .NSLOT_F(63), .SELFTEST(0)) dut (
    .clk(clk), .rst(rst),
    .ps_start(ps_start), .pixel_ready(pixel_ready),
    .x(px), .y(py),
    .key_press(key_press), .key_level(key_level), .bright_eff(7'd50),
    .menu_active(menu_on), .osd_allow(1'b1), .info_show(info_show_tb),
    .bp_req(1'b0), .bp_addr(24'd0), .bp_len(16'd0),
    .bp_rdy(), .bp_data(), .bp_valid(), .bp_done(),
    .flash_cs_n(f_cs), .flash_sclk(f_sclk), .flash_mosi(f_mosi),
    .flash_wp_n(f_wp), .flash_hold_n(f_hold), .flash_miso(f_miso),
    .enter_req(enter_req), .exit_req(exit_req),
    .osd_layer_en(osd_layer_en), .fade(fade), .osd_pix(osd_pix),
    .fx_sel(), .auto_mode(),.info_on(),
    .album_prev(), .album_next(),
    .cur_w(16'd640), .cur_h(16'd480), .auto_remain(auto_remain), .auto_bar(9'd0),
    .txt_trig(),
    .pic_bright(), .pic_contrast(), .pic_temp(), .pic_sat(),
    .pic_style(), .pic_tgl(),
    // ★ 2026-11-02 信息条文件名（8.3 名 + 名字 RAM 状态位）
    .I_nm_char(nm_char), .I_nm_blen(4'd8), .I_nm_elen(4'd3),
    .I_nm_len4(20'd0), .I_nm_vld(1'b1),
    .cur_rel(2'd0), .img_go(1'b0), .nm_upd(1'b0), .nm_hold(1'b0),
    // ★★ 2026-11-02【出图全黑的根因】缩放 OSD 的输入**必须显式接 0**。
    //   悬空时 `zoom_on` 是 Z ⇒ osd_top 末级 `chit_v ? zcol : ...` 的选择子变 X
    //   ⇒ m_pix 的 RGB 全 X ⇒ 4 张 ppm 全是纯黑（本台一直这样，只是没人看）。
    .zoom_on(1'b0), .ax(10'd0), .ay(10'd0), .zoom_sel(8'h10), .zoom_mode(2'd0)
);

w25q64_model_24b #(.MEM_BYTES(393216), .INIT_HEX("tools/font.hex"),
                   .INIT_HEX_ICONS("tools/osd_bitmaps.hex"),
                   .ICON_ADDR(24'h050000)) u_flash (
    .cs_n(f_cs), .sclk(f_sclk), .mosi(f_mosi), .miso(f_miso)
);

function [23:0] bg_of;
    input [15:0] bx, by;
    reg [7:0] r, g, b;
    begin
        r = bx[7:0]; g = by[7:0]; b = bx[7:0] ^ by[7:0];
        if (bx[5] ^ by[5]) begin r = r >> 1; g = g >> 1; b = b >> 1; end
        bg_of = {r, g, b};
    end
endfunction

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
        @(negedge clk); key_level = 4'h0; key_press = k;
        @(posedge clk); @(negedge clk); key_press = 4'h0;
        @(posedge clk); @(negedge clk); key_level = 4'hF;
        repeat (8) @(posedge clk);
    end
endtask

function [8:0] amul_a9;
    input [7:0] a;
    reg   [16:0] p;
    begin
        p = {9'd0, a} * 17'd255;
        amul_a9 = (a == 8'd255) ? 9'd256 : p[16:8];
    end
endfunction

task shot_rect;
    input [8*24:1] fname;
    input integer x0, y0, nx, ny, st;
    integer i, j, x, y, fd;
    reg [7:0]  a8;
    reg [8:0]  a9, i9;
    reg [23:0] fg, bg;
    reg [17:0] mr, mg, mb;
    begin
        fd = $fopen(fname, "wb");
        $fwrite(fd, "P6\n%0d %0d\n255\n", nx, ny);
        for (j = 0; j < ny; j = j + 1) begin
            for (i = 0; i < nx; i = i + 1) begin
                x = x0 + i * st; y = y0 + j * st;
                @(negedge clk); px = x; py = y; ps_start = 1'b1;
                repeat (4) @(posedge clk);
                a8 = osd_pix[31:24]; fg = osd_pix[23:0]; bg = bg_of(x, y);
                a9 = amul_a9(a8); i9 = 9'd256 - a9;
                mr = {2'b0, fg[23:16]} * {1'b0, a9} + {2'b0, bg[23:16]} * {1'b0, i9};
                mg = {2'b0, fg[15:8]}  * {1'b0, a9} + {2'b0, bg[15:8]}  * {1'b0, i9};
                mb = {2'b0, fg[7:0]}   * {1'b0, a9} + {2'b0, bg[7:0]}   * {1'b0, i9};
                $fwrite(fd, "%c%c%c", mr[15:8], mg[15:8], mb[15:8]);
            end
        end
        @(negedge clk); ps_start = 1'b0;
        repeat (3000) @(posedge clk);
        $fclose(fd);
        $display("shot %0s (%0dx%0d @%0d,%0d)", fname, nx, ny, x0, y0);
    end
endtask

`define K_UP    4'h8
`define K_DN    4'h4
`define K_LEFT  4'h1
`define K_RIGHT 4'h2

initial begin
    #200 rst = 0;
    #100 menu_on = 1'b1;
    nframe(40);                                  // NAV
    key(`K_DN); nframe(16);                      // nav 0 -> 1（相册）
    key(`K_RIGHT); nframe(20);                   // 进入页面 item0
    key(`K_DN); nframe(16);
    key(`K_DN); nframe(16);
    key(`K_DN); nframe(16);                      // item3 自动切换
    key(`K_RIGHT); nframe(20);                   // 打开下拉
    $fflush; $display("open: pop_sel=%0d pop_w0=%0d pop_a=%0d sel_k=%0d",
             dut.u_menu.pop_sel, dut.u_menu.pop_w0, dut.u_menu.pop_a, dut.u_menu.sel_k);
    for (k = 0; k < 6; k = k + 1) begin
        key(`K_DN); nframe(16);                  // KEY3 下：0->1->...->5
        $display("sel=%0d w0=%0d a=%0d la=%0d lb=%0d v6=%0d v7=%0d",
                 dut.u_menu.pop_sel, dut.u_menu.pop_w0, dut.u_menu.pop_a,
                 dut.u_menu.pop_la, dut.u_menu.pop_lb,
                 dut.u_menu.pop_v6, dut.u_menu.pop_v7);
    end
    nframe(20);
    shot_rect("out/a_popup5.ppm", 280, 200, 360, 240, 1);   // 弹层+菜单行
    $fflush;
    // 再按一次 KEY4 上：5 -> 4
    key(`K_UP); nframe(20);
    $display("after up: sel=%0d w0=%0d a=%0d la=%0d lb=%0d",
             dut.u_menu.pop_sel, dut.u_menu.pop_w0, dut.u_menu.pop_a,
             dut.u_menu.pop_la, dut.u_menu.pop_lb);
    shot_rect("out/a_popup4.ppm", 280, 200, 360, 240, 1);

    // 信息条：文件名/分辨率 + 圆角 + 倒计时右对齐
    menu_on = 0; info_show_tb = 1;
    force dut.info_on = 1'b1;
    force dut.u_menu.page_r = 3'd1;     // 相册页（自动切换在 item3）
    force dut.u_menu.val3    = 7'd1;    // 3s ⇒ auto_mode=1，倒计时才显示
    auto_remain = 6'd30;  nframe(30);
    shot_rect("out/a_infobar.ppm", 0, 0, 360, 100, 1);    // "640x480     30"
    auto_remain = 6'd7;   nframe(30);
    shot_rect("out/a_infobar1.ppm", 0, 0, 360, 100, 1);   // "640x480      7"
    release dut.u_menu.page_r;
    release dut.u_menu.val3;
    auto_remain = 6'd0;
    release dut.info_on;
    $finish;
end
endmodule
