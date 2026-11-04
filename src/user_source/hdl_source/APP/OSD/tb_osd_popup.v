`timescale 1ns/1ps
//==============================================================================
//  tb_osd_popup.v —— 只出"下拉弹层"那几张图（快速迭代用，约 20s）
//==============================================================================
//  用法（工作目录 = 本文件所在目录，INIT_HEX 相对它）：
//      iverilog -g2012 -o p.vvp -s tb_osd_popup tb_osd_popup.v osd_top.v \
//          osd_menu.v osd_gfx.v osd_cellmem.v osd_widget.v \
//          ../osd_font_cache.v ../ps_gen_text.v ../gb_addr.v \
//          ../../spi_flash_read.v ../w25q64_model_24b.v
//      vvp p.vvp
//==============================================================================
module tb_osd_popup;

reg clk = 0;
always #10 clk = ~clk;

reg         rst = 1;
reg         ps_start = 0;
reg         pixel_ready = 1;
reg  [15:0] px = 0, py = 0;
reg  [3:0]  key_press = 0;
reg  [3:0]  key_level = 4'hF;
reg         menu_on = 0;
integer     k;

wire        enter_req, exit_req, osd_layer_en;
wire [7:0]  fade;
wire [31:0] osd_pix;
wire        f_cs, f_sclk, f_mosi, f_wp, f_hold, f_miso;

osd_top #(.NL(8), .MAXLEN(32), .NSLOT(8), .NSLOT_F(63), .SELFTEST(0)) dut (
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
    // 2026-10-31: tie off ports added after this bench was written (were floating Z)
    , .zoom_on(1'b0), .ax(10'd0), .ay(10'd0), .zoom_sel(8'h10), .zoom_mode(2'd0),
    .I_nm_char(8'd0), .I_nm_blen(4'd0), .I_nm_elen(4'd0), .I_nm_len4(20'd0), .I_nm_vld(1'b0),
    .I_fp_vcnt(3'd0), .I_fp_has_more(1'b0), .I_fp_gen(1'b0), .I_fp_has_prev(1'b0), .O_fp_open()
);

w25q64_model_24b #(.MEM_BYTES(262144), .INIT_HEX("tools/font.hex")) u_flash (
    .cs_n(f_cs), .sclk(f_sclk), .mosi(f_mosi), .miso(f_miso)
);

function [23:0] bg_of;
    input [15:0] bx;
    input [15:0] by;
    reg [7:0] r, g, b;
    begin
        r = bx[7:0];
        g = by[7:0];
        b = bx[7:0] ^ by[7:0];
        if (bx[5] ^ by[5]) begin r = r >> 1; g = g >> 1; b = b >> 1; end
        bg_of = {r, g, b};
    end
endfunction

task nframe;
    input integer n;
    integer i;
    begin
        for (i = 0; i < n; i = i + 1) begin
            @(negedge clk); ps_start = 1'b1;
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
        repeat (6) @(posedge clk);
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
                x = x0 + i * st;
                y = y0 + j * st;
                @(negedge clk); px = x; py = y; ps_start = 1'b1;
                repeat (4) @(posedge clk);
                a8 = osd_pix[31:24];
                fg = osd_pix[23:0];
                bg = bg_of(x, y);
                a9 = amul_a9(a8);
                i9 = 9'd256 - a9;
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

// 面板 KEY1..KEY4 = key_press[0..3]：KEY4 上 / KEY3 下 / KEY2 进入 / KEY1 返回
`define K_UP    4'h8
`define K_DN    4'h4
`define K_LEFT  4'h1        // KEY1 = 返回（导航栏按下 = 关 OSD）
`define K_RIGHT 4'h2        // KEY2 = 进入 / 确认

initial begin
    #200 rst = 0;
    #100 menu_on = 1'b1;
    nframe(40);
    key(`K_RIGHT); nframe(16);                 // 进 page0
    key(`K_DN); nframe(20);
    key(`K_DN); nframe(20);
    key(`K_DN); nframe(20);
    key(`K_DN); nframe(20);                    // item=4 风格（ComboBox，16 字选项）
    key(`K_RIGHT); nframe(20);                 // 打开弹层
    $display("pop=%0d pop_sel=%0d sel_k=%0d",   // ★ 2026-10-31：格式符与参数个数对齐（Verilator 要求）
        dut.u_menu.pop_open, dut.u_menu.pop_sel, dut.u_menu.sel_k);
    $display("      [DN] pop_y=%0d pop_y6=%0d pop_y7=%0d pop_sel=%0d la=%0d lb=%0d",
             dut.u_menu.pop_y, dut.u_menu.pop_y6, dut.u_menu.pop_y7,
             dut.u_menu.pop_sel, dut.u_menu.pop_la, dut.u_menu.pop_lb);
    for (k = 0; k < 8; k = k + 1)
        $display("      [DN] txt line%0d  y=%0d x=%0d", k,
                 dut.u_menu.txt_xy[k*32+16 +: 16], dut.u_menu.txt_xy[k*32 +: 16]);
    // ★ item4 弹层在屏幕下部（pop_y≈378）：裁剪窗对准 200..480
    shot_rect("out/p_popup.ppm", 100, 200, 528, 280, 1);  // 弹层（含左标签列）
    shot_rect("out/p_zoom.ppm", 100, 330, 540, 148, 1);   // ★ 弹层 1:1 特写
    // ★ 页面板左上角 + 分割线：验"没有圆角 / 顶端没有亮带 / 分割线均匀"
    shot_rect("out/f_corner.ppm", 88, 0, 60, 64, 1);
    shot_rect("out/f_seam.ppm", 90, 60, 24, 200, 1);
    // ★ 2026-10-29 [S1]：cell_r 已在 10-28 拆成 geo_hi/geo_lo/attr_r，这里跟着改
    $display("      geom: combo_y=%0d pop_y=%0d panel=(%0d..%0d) opt0_y=%0d opt1_y=%0d opt2_y=%0d",
             dut.u_menu.combo_y, dut.u_menu.pop_y,
             dut.u_menu.pop_y - 12, dut.u_menu.pop_y + 2*34 + 30 + 6,
             dut.u_gfx.u_mem.geo_lo[6][8:0], dut.u_gfx.u_mem.geo_lo[7][8:0],
             dut.u_gfx.u_mem.geo_lo[8][8:0]);
    shot_rect("out/p_full.ppm", 0, 0, 320, 240, 2);       // 整屏（看被隐藏的行）
    // ★ 选中"黑白"再拍一张：锚定行写 2 字（黑白），验证收起态多字排布
    key(`K_UP); nframe(20);
    shot_rect("out/p_popup_bw.ppm", 100, 200, 528, 280, 1);

    // 靠底部的 ComboBox：page2 item3 → sel_k=3 → 向上翻
    key(`K_LEFT); nframe(16);
    key(`K_LEFT); nframe(20);                  // 回 NAV
    key(`K_DN); nframe(16); key(`K_DN); nframe(16);   // nav -> 2（系统）
    key(`K_RIGHT); nframe(20);
    key(`K_DN); nframe(16); key(`K_DN); nframe(16); key(`K_DN); nframe(16);  // item=3
    key(`K_RIGHT); nframe(20);
    // ★ 再把选中项推到第 2 项 ⇒ 弹层整体上移（选中项仍压在控件行上）
    key(`K_UP); nframe(16);
    key(`K_UP); nframe(16);
    $display("pop=%0d pop_sel=%0d sel_k=%0d",
        dut.u_menu.pop_open, dut.u_menu.pop_sel, dut.u_menu.sel_k);
    $display("      [UP] pop_y=%0d pop_y6=%0d pop_y7=%0d pop_sel=%0d la=%0d lb=%0d",
             dut.u_menu.pop_y, dut.u_menu.pop_y6, dut.u_menu.pop_y7,
             dut.u_menu.pop_sel, dut.u_menu.pop_la, dut.u_menu.pop_lb);
    for (k = 0; k < 8; k = k + 1)
        $display("      [UP] txt line%0d  y=%0d x=%0d", k,
                 dut.u_menu.txt_xy[k*32+16 +: 16], dut.u_menu.txt_xy[k*32 +: 16]);
    // ★ 2026-10-29 [S1]：换成 geo_hi/geo_lo；iverilog 不支持对层次数组用变量下标，逐条写死
    $display("      [UP] opt cell15 y=%0d..%0d", dut.u_gfx.rd_geo[8:0], dut.u_gfx.rd_geo[27:19]);   // [S6] 15 是常量槽
    $display("      [UP] opt cell16 y=%0d..%0d", dut.u_gfx.u_mem.geo_lo[6][8:0], dut.u_gfx.u_mem.geo_hi[6][8:0]);
    $display("      [UP] opt cell17 y=%0d..%0d", dut.u_gfx.u_mem.geo_lo[7][8:0], dut.u_gfx.u_mem.geo_hi[7][8:0]);
    shot_rect("out/p_popup_up.ppm", 100, 140, 528, 250, 1);
    $finish;
end

endmodule
