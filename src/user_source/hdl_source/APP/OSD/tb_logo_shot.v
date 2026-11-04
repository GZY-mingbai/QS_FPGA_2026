//==============================================================================
//  tb_logo_shot.v —— 出图专用（只看 logo + 导航栏的观感）
//==============================================================================
//
//  目的：把 OSD 层按 A8 合成到一张"渐变 + 棋盘"的假底图上，输出 PPM，
//  用眼睛确认三件事：
//    ① 左上角 logo 没有白框（背景和左栏面板同色）
//    ② 导航栏确实被 logo 顶下去了 48px
//    ③ 标题 / 页面行没被 logo 挤走
//
//  出图：
//    out/g_full.ppm      640×480 全屏，采样步长 2 ⇒ 320×240 总览
//    out/g_topleft.ppm   左上 0..339 × 0..139，1:1
//    out/g_page0.ppm     进"显示"页后的左栏 + 前两行，1:1
//
//  【怎么跑】工作目录 = APP/OSD/
//    iverilog -g2012 -DIVERILOG_SIM -s tb_logo_shot -o /tmp/ls.vvp tb_logo_shot.v \
//        osd_top.v osd_menu.v osd_gfx.v osd_cellmem.v osd_widget.v \
//        osd_icon_rom.v flash_arb.v ../osd_font_cache.v ../ps_gen_text.v \
//        ../gb_addr.v ../../spi_flash_read.v ../w25q64_model_24b.v
//    vvp /tmp/ls.vvp && python tools/ppm2png.py out
//==============================================================================
`timescale 1ns / 1ps

module tb_logo_shot;

localparam integer NL = 8, MAXLEN = 32, NSLOT = 20;

reg clk = 0;
always #10 clk = ~clk;

reg         rst = 1;
reg         ps_start = 0;
reg         pixel_ready = 1;
reg  [15:0] px = 0, py = 0;
reg  [3:0]  key_press = 0;
reg  [3:0]  key_level = 4'hF;
reg         menu_on = 1;
reg         info_show_tb = 0;
reg  [5:0]  auto_remain = 6'd0;

wire        enter_req, exit_req, osd_layer_en;
wire [7:0]  fade;
wire [31:0] osd_pix;
wire        f_cs, f_sclk, f_mosi, f_wp, f_hold, f_miso;

osd_top #(.NL(NL), .MAXLEN(MAXLEN), .NSLOT(NSLOT), .NSLOT_F(63), .SELFTEST(0)) dut (
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
    .cur_w(16'd640), .cur_h(16'd480),
    .auto_remain(auto_remain), .auto_bar(9'd0),
    .txt_trig(),
    .pic_bright(), .pic_contrast(), .pic_temp(), .pic_sat(),
    .pic_style(), .pic_tgl()
);

w25q64_model_24b #(
    // ≥ 0x050000 + 25728 = 353408，否则 $readmemh 静默不灌
    .MEM_BYTES     (393216),
    .INIT_HEX      ("tools/font.hex"),
    .INIT_HEX_ICONS("tools/osd_bitmaps.hex"),
    .ICON_ADDR     (24'h050000)
) u_flash (
    .cs_n(f_cs), .sclk(f_sclk), .mosi(f_mosi), .miso(f_miso)
);

//------------------------------------------------------------------------------
//  假底图：渐变 + 棋盘（棋盘是为了让"半透明层"和"不透明层"一眼分得开）
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
//  8×8 → 必须落显式 17 位中间量（写成 (a*255)>>8 会被截成 9 位）
//------------------------------------------------------------------------------
function [8:0] amul_a9;
    input [7:0] a;
    reg   [16:0] p;
    begin
        p = {9'd0, a} * 17'd255;
        amul_a9 = (a == 8'd255) ? 9'd256 : p[16:8];
    end
endfunction

//------------------------------------------------------------------------------
//  帧 / 按键
//------------------------------------------------------------------------------
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
        @(negedge clk); key_press = 4'h0;
        @(posedge clk);
        @(negedge clk); key_level = 4'hF;
        repeat (6) @(posedge clk);
    end
endtask

//------------------------------------------------------------------------------
//  扫一个矩形 → P6 PPM（先攒再写，不要逐像素 $fwrite 格式化）
//------------------------------------------------------------------------------
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
        $display("  shot %0s  (%0d x %0d, st=%0d)", fname, nx, ny, st);
    end
endtask

//------------------------------------------------------------------------------
integer wait_cnt;
initial begin
    repeat (4) @(posedge clk);
    rst = 0;

    wait_cnt = 0;
    while ((dut.icon_ready !== 1'b1) && (wait_cnt < 4_000_000)) begin
        @(posedge clk);
        wait_cnt = wait_cnt + 1;
    end
    $display("=== 位图 ROM ready after %0d cycles ===", wait_cnt);
    nframe(40);                              // 让布局/文字都稳定

    $display("=== 1. NAV 态：logo + 左栏 ===");
    shot_rect("out/g_full.ppm",    0,   0, 320, 240, 2);
    shot_rect("out/g_topleft.ppm", 0,   0, 340, 140, 1);

    $display("=== 2. 进 PAGE0：标题/行控件与 logo 的相对位置 ===");
    key(4'h2);                               // KEY2 进入
    nframe(60);                              // 等文字重绑
    shot_rect("out/g_page0.ppm",   0,   0, 640, 200, 1);

    $display("=== done ===");
    $finish;
end

endmodule
