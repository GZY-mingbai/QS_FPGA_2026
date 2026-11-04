`timescale 1ns/1ps
//==============================================================================
//  tb_osd_render.v —— OSD 整帧渲染器（仿真出图，替代拍照）
//==============================================================================
//  作用：把 osd_top 的输出**合成到一张合成底图**上，逐像素写成 PPM，
//  再用 tools/ppm2png.py 转 PNG 直接看。这样每一处配色/透明度/几何都能
//  在仿真里逐像素核对，不用反复上板拍照。
//
//  合成公式与 ps_merge_alpha_pp（alpha=255 时）完全一致：
//      a9   = (A==255) ? 256 : (A*255)>>8
//      out  = (a9*fg + (256-a9)*bg) >> 8
//  底图 = 彩色渐变 + 32px 棋盘，便于一眼看出"控件半透明了多少"。
//
//  用法：
//      iverilog -g2012 -o r.vvp -s tb_osd_render tb_osd_render.v osd_top.v \
//          osd_menu.v osd_gfx.v osd_cellmem.v osd_widget.v \
//          ../osd_font_cache.v ../ps_gen_text.v ../gb_addr.v \
//          ../../spi_flash_read.v ../w25q64_model_24b.v
//      vvp r.vvp            （工作目录必须是本文件所在目录，INIT_HEX 相对它）
//==============================================================================
module tb_osd_render;

parameter integer W = 640;
parameter integer H = 480;
parameter integer NPIX = W*H;

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
wire        f_miso;

// ★ 参数必须与 App.v 里的一致：MAXLEN=32（2026-10-26 起收起态值占 14..29 格）、NSLOT=18
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
    .txt_trig(),
    // ★ 2026-10-31 [Q8 根因]：这些输入口是 10-30 以后加的，台子一直没接 ⇒ 悬空 Z。
    //   zoom_on=Z ⇒ chit_v=X；zoom_mode=Z ⇒ zcol = C0 + X ⇒ 合并级 `chit_v ? zcol : …`
    //   按位合并，只剩 α=C0，RGB 全 X ⇒ 7 张 PPM 全黑（`[dbg] pix=c0xxxxxx`）。
    .zoom_on(1'b0), .ax(10'd0), .ay(10'd0), .zoom_sel(8'h10), .zoom_mode(2'd0),
    .I_nm_char(8'd0), .I_nm_blen(4'd0), .I_nm_elen(4'd0), .I_nm_len4(20'd0), .I_nm_vld(1'b0),
    .I_fp_vcnt(3'd0), .I_fp_has_more(1'b0), .I_fp_gen(1'b0), .I_fp_has_prev(1'b0), .O_fp_open()
);

w25q64_model_24b #(
    .MEM_BYTES     (393216),
    .INIT_HEX      ("tools/font.hex"),
    // ★ 2026-10-27：图标也要灌进去，否则 f_icon/v_nav 里的 5 个图标是空白
    //   （osd_icon_rom 会判"未编程"而整块不画）。
    .INIT_HEX_ICONS("tools/osd_bitmaps.hex"),
    .ICON_ADDR     (24'h050000)
) u_flash (
    .cs_n(f_cs), .sclk(f_sclk), .mosi(f_mosi), .miso(f_miso)
);

//------------------------------------------------------------------------------
//  底图
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
//  坐标延迟链（osd_pix 相对 (x,y) 晚 3 拍：gfx 2 + merge 1）
//------------------------------------------------------------------------------
reg [15:0] qx0, qx1, qx2, qx3;
reg [15:0] qy0, qy1, qy2, qy3;
reg        qv0, qv1, qv2, qv3;
always @(posedge clk) begin
    qx0 <= px; qy0 <= py; qv0 <= ps_start & pixel_ready;
    qx1 <= qx0; qy1 <= qy0; qv1 <= qv0;
    qx2 <= qx1; qy2 <= qy1; qv2 <= qv1;
    qx3 <= qx2; qy3 <= qy2; qv3 <= qv2;
end

//------------------------------------------------------------------------------
//  短帧（只为产生一次 tick / 推一步动画）：x,y 不动
//------------------------------------------------------------------------------
task nframe;
    input integer n;
    integer i, j;
    begin
        for (i = 0; i < n; i = i + 1) begin
            @(negedge clk); ps_start = 1'b1; px = 0; py = 0;
            repeat (3000) @(posedge clk);
            @(negedge clk); ps_start = 1'b0;
            repeat (4000) @(posedge clk);
        end
    end
endtask

//------------------------------------------------------------------------------
//  按键
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
//  渲染一帧 → 裸 RGB 文件（640*480*3 字节）+ alpha 裸文件
//  ★ 不要逐像素 $fwrite：iverilog 的 %c 格式化极慢（一次 300k 像素要 60s），
//    先攒进内存、最后一次 $writememb 落盘，快 100 倍。
//------------------------------------------------------------------------------
// ★ 8×8 乘法必须先落到显式 17 位中间量：写 (a8*255)>>8 会被上下文截成 9 位，
//   结果恒为 0~1 ⇒ 仿真图里所有半透明层都"几乎全透"（踩过一次）。
function [8:0] amul_a9;
    input [7:0] a;
    reg   [16:0] p;
    begin
        p = {9'd0, a} * 17'd255;
        amul_a9 = (a == 8'd255) ? 9'd256 : p[16:8];
    end
endfunction

//------------------------------------------------------------------------------
//  只扫"关心的矩形"，直接写 P6 PPM
//    nx,ny = 输出像素数；st = 采样步长（总览用 2，细节用 1）
//  ★ OSD 是逐像素无状态的，扫描顺序任意 ⇒ 不必扫满 640×480。
//  ★ 不要用 $writememb：iverilog 写的是二进制**字符**，不是字节（文件会大 10 倍）。
//------------------------------------------------------------------------------
integer fno = 0;
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
        $display("shot %0s (%0dx%0d @%0d,%0d step%0d)", fname, nx, ny, x0, y0, st);
        fno = fno + 1;
    end
endtask

// 全屏总览（半采样）
task shot_full;
    input [8*24:1] fname;
    begin
        shot_rect(fname, 0, 0, 320, 240, 2);
    end
endtask

// KEY1=bit0 上 / KEY2=bit1 下 / KEY3=bit2 左 / KEY4=bit3 右
// 面板 KEY1..KEY4 = key_press[0..3]：KEY4 上 / KEY3 下 / KEY2 进入 / KEY1 返回
`define K_UP    4'h8
`define K_DN    4'h4
`define K_LEFT  4'h1        // KEY1 = 返回（导航栏按下 = 关 OSD）
`define K_RIGHT 4'h2        // KEY2 = 进入 / 确认

//------------------------------------------------------------------------------
//  状态探针：直接打印菜单状态 + 控件表，比看图更快定位逻辑问题
//------------------------------------------------------------------------------
task dump_state;
    input [8*12:1] tag;
    integer k;
    begin
        $display("[%0s] focus=%0d nav=%0d page=%0d item=%0d edit=%0d scroll=%0d pop=%0d popsel=%0d",
            tag,
            dut.u_menu.focus, dut.u_menu.nav_idx, dut.u_menu.page,
            dut.u_menu.item,  dut.u_menu.edit,  dut.u_menu.scroll_row,
            dut.u_menu.pop_open, dut.u_menu.pop_sel);
        $display("        cur=(%0d,%0d) %0dx%0d   sc_px=%0d  val=%0d %0d %0d %0d %0d %0d %0d",
            dut.u_menu.cur_x, dut.u_menu.cur_y, dut.u_menu.cur_w, dut.u_menu.cur_h,
            dut.u_menu.sc_px,
            dut.u_menu.val0, dut.u_menu.val1, dut.u_menu.val2,
            dut.u_menu.val3, dut.u_menu.val4, dut.u_menu.val5, dut.u_menu.val6);
        // ★ 2026-10-29 [S6]：常量槽（0..3/5..9/15/19）已不在 cellmem 里，
        //   动态槽按 {4→0, 10..14→1..5, 16..18→6..8} 折算；这里只打印动态 9 槽。
        for (k = 0; k < 8; k = k + 1)
            $display("        dyn%0d ty=%0d sty=%0d rad=%0d geo=(%0d,%0d)-(%0d,%0d)",
                k, dut.u_gfx.u_mem.cell_rd[k][23:20], dut.u_gfx.u_mem.cell_rd[k][19:16],
                dut.u_gfx.u_mem.cell_rd[k][15:12],
                dut.u_gfx.u_mem.geo_lo[k][18:9], dut.u_gfx.u_mem.geo_lo[k][8:0],
                dut.u_gfx.u_mem.geo_hi[k][18:9], dut.u_gfx.u_mem.geo_hi[k][8:0]);
    end
endtask

initial begin
    #200 rst = 0;
    #100 menu_on = 1'b1;

    // ★ 2026-10-31：nframe(40) 只有 40×7000 拍 ≈ 5.6 ms，而图标 ROM 的 POR 就要 8 ms
    //   ⇒ 前 5 张截图里图标/logo 还没装好（f_icon 是空的）。先等 ROM 就绪。
    wait (dut.icon_ready === 1'b1);
    nframe(40);
    shot_full("out/v_nav.ppm");
    dump_state("NAV");
    // ---- 全分辨率细节（半采样看不出 1~2px 的分割线）----
    shot_rect("out/f_seam.ppm",    88, 180,  48,  36, 1);   // 左栏/分隔/面板 交界面
    shot_rect("out/f_pagetop.ppm", 94,   0,  72,  32, 1);   // 页面板顶边 + 上方遮罩
    shot_rect("out/f_cursor.ppm",  16,   6,  72,  72, 1);   // 导航游标（圆角 + 内部色）
    shot_rect("out/f_icon.ppm",    24,  16,  56,  52, 1);   // 图标

    key(`K_RIGHT);                               // 进页面
    nframe(16);
    shot_full("out/v_page.ppm");
    dump_state("PAGE0");
    shot_rect("out/f_cursor2.ppm", 96,  50, 120,  52, 1);   // 页面游标左上角（圆角）
    shot_rect("out/f_slider.ppm", 100,  52, 340,  40, 1);   // 亮度行：标签+数值+轨道
    shot_rect("out/f_check.ppm",  100, 292, 520,  44, 1);   // 饱和度行：标签+滑条（原勾选框行位）

    key(`K_DN); nframe(16);                      // item=1
    key(`K_DN); nframe(16);                      // item=2
    key(`K_DN); nframe(16);                      // item=3
    key(`K_DN); nframe(16);                      // item=4 风格（下拉，16 字选项）
    shot_rect("out/f_combo.ppm",  100, 372, 520,  44, 1);   // 风格行：标签+收起态值
    shot_full("out/v_item2.ppm");
    dump_state("ITEM4");

    key(`K_RIGHT); nframe(12);                   // 进入编辑 ⇒ 弹层（pop_y≈378，屏幕下部）
    shot_full("out/v_popup.ppm");
    shot_rect("out/f_popup.ppm",  296, 340, 336, 140, 1);
    dump_state("POPUP");

    key(`K_DN); nframe(12);                      // 换选项（无 → 黑白）
    shot_full("out/v_popup2.ppm");
    dump_state("POPUP_DN");

    key(`K_LEFT); nframe(12);                    // 收起
    key(`K_RIGHT); nframe(12);                   // 再进编辑（应重开弹层）
    shot_full("out/v_popup3.ppm");
    dump_state("POPUP_RE");

    //---- 恢复默认按钮：KEY2 触发动作，全部回 DEF0（四滑条 50% / 风格无）----
    key(`K_LEFT); nframe(12);                    // 退出编辑（收起弹层）
    key(`K_DN); nframe(16);                      // item 4 -> 5（恢复默认）
    key(`K_RIGHT); nframe(16);                   // KEY2：恢复默认
    shot_rect("out/f_restored.ppm", 100,  52, 520, 340, 1);  // 控件列全景：全 50% / 风格无
    dump_state("RESTORED");

    //---- 靠底部的 ComboBox：弹层必须"向上翻"（page2 item3 → sel_k=3 → pop_r0=1）----
    key(`K_LEFT); nframe(16);                    // 回 NAV
    key(`K_DN); nframe(16);                      // nav 0 -> 1
    key(`K_DN); nframe(16);                      // nav 1 -> 2（系统）
    key(`K_RIGHT); nframe(20);                   // 进 page2
    begin : to3
        integer i;
        for (i = 0; i < 3; i = i + 1) begin key(`K_DN); nframe(16); end
    end
    shot_full("out/v_p2i3.ppm");
    dump_state("P2_I3");
    key(`K_RIGHT); nframe(20);                   // 打开弹层（应向上翻）
    shot_full("out/v_popup_up.ppm");
    shot_rect("out/f_popup_up.ppm", 296, 130, 336, 340, 1);
    dump_state("POPUP_UP");
    key(`K_LEFT); nframe(16);
    key(`K_LEFT); nframe(20);                    // 回 NAV

    //---- 四页文字全览：全分辨率扫"标签列 + 数值列"，专门看有没有字丢失 ----
    begin : tour
        integer p, i;
        reg [8*32:1] nm;
        for (p = 0; p < 4; p = p + 1) begin
            for (i = 0; i < 3; i = i + 1) begin key(`K_UP); nframe(10); end
            for (i = 0; i < p; i = i + 1) begin key(`K_DN); nframe(10); end
            key(`K_RIGHT); nframe(60);   // NL=8 之后重绑要更久，24 帧不够会拍到半新半旧
            $sformat(nm, "out/t_page%0d.ppm", p);
            shot_rect(nm, 108, 56, 232, 350, 1);
            dump_state("TOUR");
            key(`K_LEFT); nframe(16);
        end
    end

    //---- 滚到 page 最下方（scroll_row=2）：标签必须跟着行控件一起走 ----
    key(`K_UP); nframe(16); key(`K_LEFT); nframe(20);      // 回 NAV（page0）
    key(`K_RIGHT); nframe(20);
    begin : tob
        integer i;
        for (i = 0; i < 6; i = i + 1) begin key(`K_DN); nframe(16); end
    end
    nframe(30);
    shot_full("out/v_bottom.ppm");
    shot_rect("out/t_bottom.ppm", 108, 56, 232, 350, 1);
    dump_state("BOTTOM");

    //---- 最底下那一行的下拉（sel_k=4 → 弹层必须向上翻且不出屏）----
    key(`K_UP); nframe(16);                                // item 5
    key(`K_UP); nframe(16);                                // item 4
    key(`K_UP); nframe(16);                                // item 3
    nframe(30);
    dump_state("BOTTOM_I3");

    //---- ★ 滚到底 + 色温变成第 0 行 + 打开下拉（用户报的"顶端顶到标题栏"）----
    key(`K_UP); nframe(16);                      // item3 -> item2（色温，窗口第 0 行）
    nframe(20);
    dump_state("ROW0_I2");
    key(`K_RIGHT); nframe(24);                   // 打开弹层
    shot_full("out/v_popup_row0.ppm");
    shot_rect("out/f_popup_row0.ppm", 0, 0, 640, 200, 1);   // 标题区 + 弹层上半
    dump_state("POPUP_ROW0");
    // 值推到第 3 项，再确认一次仍然不进标题区
    key(`K_UP); nframe(16); key(`K_UP); nframe(16);
    shot_rect("out/f_popup_row0b.ppm", 0, 0, 640, 200, 1);
    dump_state("POPUP_ROW0B");
    key(`K_DN); nframe(12);
    key(`K_LEFT); nframe(20);                    // 收起弹层

    //---- ★ 同一屏的按钮行（滚到底时 item5/6 = 恢复默认/退出菜单）----
    key(`K_DN); nframe(30);
    shot_full("out/v_buttons.ppm");
    shot_rect("out/f_buttons.ppm", 300, 290, 336, 120, 1);
    dump_state("BUTTONS");

    //---- 滚到底 → 回导航栏 → 再进页面（用户报的"控件错位、文字不显示"）----
    key(`K_LEFT); nframe(20);                    // 页面 → 导航栏
    key(`K_RIGHT); nframe(30);                   // 再进页面
    shot_full("out/v_reenter.ppm");
    shot_rect("out/t_reenter.ppm", 108, 56, 232, 350, 1);
    dump_state("REENTER");

    $finish;
end

endmodule
