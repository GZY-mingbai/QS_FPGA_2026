//==============================================================================
//  tb_infobar_shot.v —— 信息条出图 + 三档自动切换口径 ★ 2026-11-02 / 11-05
//==============================================================================
//
//  【场景 1：文件名 / 分辨率 / 倒计时】出图 out/a_infobar_name.ppm
//    （2026-11-02 那次"文件名改成真实名字"的证据图，保持原样不动）
//
//  【场景 2/3：★ 2026-11-05 进度条 + 读秒的可见性】
//    用户口径：**关掉自动切换（mode 0）和随音乐（mode 5）都不显示进度条和数字**。
//    断言（每个场景都查，避免"只改一处"）：
//      · 轨道区 (100,58) 没有 OSD 像素（alpha == 0）—— 原来这里恒有一条
//        4px 灰线（osd_widget 的 gt=13 无条件画轨道 + 填充），就是用户看到的
//        "关掉自动切换还剩一条 0% 的进度条"。
//      · 面板在 y=60 处已经收掉（收到 52），y=40 处仍在。
//      · 倒计时数字两列（x 288..319, y 32..47）一个墨点都没有；
//        同时**文件名和分辨率必须还在** —— 防止"把整条信息条关掉"这种过度修改。
//    出图 out/a_infobar_off.ppm（mode 0）/ out/a_infobar_music.ppm（mode 5）。
//
//  【为什么另开一个台】tb_auto_popup 里的 `shot_rect` 在本轮改动之后拍出来
//  是纯黑（连**弹层**那两张也是黑的）。查下来是那个台例化 `osd_top` 时写的
//  `.NSLOT(20)` —— 它还是 [S6]/[LUT-2] 重构之前的口径，而本工程现在是
//  NSLOT=8、`wr_idx` 只写 dci 0..8 ⇒ 9..19 号槽从没被写过（读出 X）
//  ⇒ RGB=X、整幅按 X 落盘。这是**那个台自己的陈旧**，与本次改动无关
//  （弹层那两张同样黑，而弹层路径一行没动）。
//  本台用与 App.v 完全一致的 NSLOT=8，采样方式照抄 tb_osd_icons 的 `sample`
//  （那个台的像素级断言一直是准的）。
//
//  【跑法】工作目录 = APP/OSD/
//    iverilog -g2012 -DIVERILOG_SIM -s tb_infobar_shot -o /tmp/ibs.vvp \
//        tb_infobar_shot.v osd_top.v osd_menu.v osd_gfx.v osd_cellmem.v \
//        osd_widget.v osd_icon_rom.v flash_arb.v \
//        tools/bram32k_stub.v tools/mult18_stub.v \
//        ../osd_font_cache.v ../ps_gen_text.v ../gb_addr.v \
//        ../../spi_flash_read.v ../w25q64_model_24b.v
//    vvp /tmp/ibs.vvp
//    python tools/ppm2png.py out/a_infobar_name.ppm
//==============================================================================
`timescale 1ns / 1ps

module tb_infobar_shot;

localparam integer NL = 8, MAXLEN = 32;
localparam integer NSLOT = 8;          // ★ 与 App.v 一致（[LUT-2]）

reg clk = 0;
always #10 clk = ~clk;

reg         rst = 1;
reg         ps_start = 0;
reg         pixel_ready = 1;
reg  [15:0] px = 0, py = 0;
reg  [3:0]  key_press = 0, key_level = 4'hF;
reg         menu_on = 0;
reg         info_show_tb = 0;
reg  [5:0]  auto_remain = 6'd0;

wire        enter_req, exit_req, osd_layer_en;
wire [7:0]  fade;
wire [31:0] osd_pix;
wire        f_cs, f_sclk, f_mosi, f_wp, f_hold, f_miso;

// ---- 名字 RAM 模型：8.3 = "IMG_0001" + "BMP" ⇒ 显示 12 字（最长）----
reg [7:0] nm_name [0:15];
integer nn;
initial begin
    for (nn = 0; nn < 16; nn = nn + 1) nm_name[nn] = 8'h20;
    nm_name[0] = "I"; nm_name[1] = "M"; nm_name[2] = "G"; nm_name[3] = "_";
    nm_name[4] = "0"; nm_name[5] = "0"; nm_name[6] = "0"; nm_name[7] = "1";
    nm_name[8] = "B"; nm_name[9] = "M"; nm_name[10] = "P";
end

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
    .pic_style(), .pic_tgl(),
    // 信息条文件名 + 名字 RAM 状态
    .I_nm_char(nm_name[dut.u_menu.O_nm_idx[3:0]]),
    .I_nm_blen(4'd8), .I_nm_elen(4'd3), .I_nm_len4(20'd0), .I_nm_vld(1'b1),
    .cur_rel(2'd0), .img_go(1'b0), .nm_upd(1'b0), .nm_hold(1'b0),
    .I_fp_vcnt(3'd0), .I_fp_has_more(1'b0), .I_fp_gen(1'b0),
    .I_fp_has_prev(1'b0), .O_fp_open(),
    // ★ 缩放 OSD 的输入必须**显式接 0**（不能悬空）：悬空时 `zoom_on`=Z ⇒
    //   osd_top 末级 `chit_v ? zcol : ...` 的选择子变成 X ⇒ m_pix 的 RGB 全是 X
    //   ⇒ 出图纯黑。tb_auto_popup 那 4 张黑图就是这个原因（它没接这几根）。
    .zoom_on(1'b0), .ax(10'd0), .ay(10'd0), .zoom_sel(8'h10), .zoom_mode(2'd0),
    // ★ 字幕的字模 Flash 客户：不接会悬空成 Z，把 flash_arb 拉成 X ⇒ 字形装不进来
    //   ⇒ 文件名/分辨率全空白（S2/S3 的 filename ink / resolution ink 就是这么假 FAIL 的）。
    .sub_fl_req(1'b0), .sub_fl_addr(24'd0), .sub_fl_len(16'd0)
);

w25q64_model_24b #(.MEM_BYTES(393216), .INIT_HEX("tools/font.hex"),
                   .INIT_HEX_ICONS("tools/osd_bitmaps.hex"),
                   .ICON_ADDR(24'h050000)) u_flash (
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

//------------------------------------------------------------------------------
//  出图：抄 tb_osd_icons 的采样（ps_start 高电平期间 5 拍后取 osd_pix）
//------------------------------------------------------------------------------
localparam integer SH_W = 340, SH_H = 90;
reg [23:0] line_buf [0:SH_W-1];
integer x, y, fd, chk_alpha;

task shot;
    input [8*32:1] fname;
    integer i, j;
    reg [31:0] p;
    reg [7:0]  a8;
    reg [8:0]  a9, i9;
    reg [17:0] rr, gg, bb;
    begin
        fd = $fopen(fname, "wb");
        $fwrite(fd, "P6\n%0d %0d\n255\n", SH_W, SH_H);
        for (j = 0; j < SH_H; j = j + 1) begin
            for (i = 0; i < SH_W; i = i + 1) begin
                @(negedge clk);
                px = i; py = j; ps_start = 1'b1;
                repeat (5) @(posedge clk);
                p = osd_pix;
                @(negedge clk); ps_start = 1'b0;
                @(posedge clk);
                // 合成：OSD 的 A 逐像素叠在 40/40/56 的灰底上（与 ps_merge 同式）
                a8 = p[31:24];
                a9 = (a8 == 8'd255) ? 9'd256 : {1'b0, a8};
                i9 = 9'd256 - a9;
                rr = ({2'b0, p[23:16]} * a9) + ({2'b0, 8'd40} * i9);
                gg = ({2'b0, p[15:8]}  * a9) + ({2'b0, 8'd40} * i9);
                bb = ({2'b0, p[7:0]}   * a9) + ({2'b0, 8'd56} * i9);
                line_buf[i] = {rr[15:8], gg[15:8], bb[15:8]};
            end
            for (i = 0; i < SH_W; i = i + 1)
                $fwrite(fd, "%c%c%c", line_buf[i][23:16], line_buf[i][15:8], line_buf[i][7:0]);
        end
        $fclose(fd);
        $display("shot %0s (%0dx%0d)", fname, SH_W, SH_H);
    end
endtask

//------------------------------------------------------------------------------
//  采一个像素（osd_pix 相对 (x,y) 晚 3 拍）—— 同 tb_osd_icons
//------------------------------------------------------------------------------
reg [31:0] cap;
task sample;
    input [15:0] sx;
    input [15:0] sy;
    begin
        @(negedge clk);
        px = sx; py = sy; ps_start = 1'b1;
        repeat (5) @(posedge clk);
        cap = osd_pix;
        @(negedge clk); ps_start = 1'b0;
        repeat (2) @(posedge clk);
    end
endtask

integer fails = 0, checks = 0;
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

// 矩形内"有墨"的像素数 —— 逐像素采，不抽行（字形笔画可能只有 1px）。
// 【判据不能是"alpha > 0"】信息条的面板本身就盖在这块矩形上（A=0xC0、
//   RGB=0x202020，见 S1 的 probe），alpha 恒非 0 ⇒ 会数出整整 512。
//   所以按**颜色**判：RGB != 面板色 0x202020 才算字形墨迹。
integer ink_n;
task ink_scan;
    input integer ix0, ix1, iy0, iy1;
    integer i, j;
    begin
        ink_n = 0;
        for (j = iy0; j <= iy1; j = j + 1)
            for (i = ix0; i <= ix1; i = i + 1) begin
                sample(i[15:0], j[15:0]);
                if (cap[23:0] != 24'h202020) ink_n = ink_n + 1;
            end
    end
endtask

//------------------------------------------------------------------------------
//  主场景：mode 1（3s）/ mode 0（关）/ mode 5（随音乐）三连
//------------------------------------------------------------------------------
initial begin
    integer wait_cnt;
    repeat (10) @(posedge clk);
    rst = 0;

    // ★ 必须等位图 ROM 就绪：osd_icon_rom 的输出寄存器在它 ready 之前是 X，
    //   而 osd_gfx 的 RGB 选择会把 X 漏到**任何**像素上（出图全黑）。
    //   tb_osd_icons 也是这么等的。
    wait_cnt = 0;
    while ((dut.icon_ready !== 1'b1) && (wait_cnt < 4_000_000)) begin
        @(posedge clk); wait_cnt = wait_cnt + 1;
    end
    $display("icon_ready=%b after %0d cycles", dut.icon_ready, wait_cnt);

    menu_on = 0; info_show_tb = 1;
    force dut.info_on = 1'b1;          // 等价于菜单里勾上"信息条"
    force dut.u_menu.page_r = 3'd1;    // 相册页（自动切换在 item3）
    force dut.u_menu.val3    = 7'd1;   // 3s ⇒ 倒计时可见
    auto_remain = 6'd30;
    nframe(40);
    $display("info_on=%b info_show=%b layer_en=%b  cur_w=%0d cur_h=%0d",
             dut.info_on, info_show_tb, osd_layer_en, dut.cur_w, dut.cur_h);
    // 探针：面板内 (300,68) 只该有面板色；文字行 (20,14) 该有字形
    @(negedge clk); px = 300; py = 68; ps_start = 1'b1;
    repeat (5) @(posedge clk);
    $display("probe (300,68): osd_pix=%h gfx=%h txt=%h | hit=%b v=%b attr=%h w_on=%b w_lv=%0d w_rgb=%h draw=%b ic_ok=%b",
             osd_pix, dut.gfx_pix, dut.txt_pix, dut.u_gfx.s1_hit, dut.u_gfx.s1_v,
             dut.u_gfx.rd_attr, dut.u_gfx.w_on, dut.u_gfx.w_lv, dut.u_gfx.w_rgb,
             dut.u_gfx.draw_on, dut.u_gfx.icon_ok);
    @(negedge clk); ps_start = 1'b0;
    @(negedge clk); px = 200; py = 45; ps_start = 1'b1;
    repeat (5) @(posedge clk);
    $display("probe (200,45) : osd_pix=%h gfx=%h | hit=%b attr=%h w_on=%b w_lv=%0d w_rgb=%h",
             osd_pix, dut.gfx_pix, dut.u_gfx.s1_hit, dut.u_gfx.rd_attr,
             dut.u_gfx.w_on, dut.u_gfx.w_lv, dut.u_gfx.w_rgb);
    @(negedge clk); ps_start = 1'b0;
    shot("out/a_infobar_name.ppm");

    //==========================================================================
    //  场景 1 基线（mode 1 = 3s）：轨道在、数字在、面板到 y=74
    //==========================================================================
    chk("S1 auto_mode == 1 (3s)", dut.u_menu.auto_mode, 1);
    sample(16'd100, 16'd58);
    chk("S1 track visible @(100,58)", (cap[31:24] >= 8'd8), 1);
    sample(16'd320, 16'd60);
    chk("S1 panel reaches y=74 @(320,60)", (cap[31:24] >= 8'd8), 1);
    ink_scan(288, 319, 32, 47);
    $display("      S1 countdown ink = %0d / 512 px", ink_n);
    chk("S1 countdown ink > 0", (ink_n > 0), 1);
    chk("S1 countdown ink looks like 2 glyphs", (ink_n < 200), 1);

    //==========================================================================
    //  场景 2：mode 0（关闭自动切换）
    //==========================================================================
    release dut.u_menu.val3;
    force   dut.u_menu.val3 = 7'd0;
    nframe(20);
    chk("S2 auto_mode == 0 (off)", dut.u_menu.auto_mode, 0);
    sample(16'd100, 16'd58);
    chk("S2 track GONE @(100,58)", cap[31:24], 0);
    sample(16'd320, 16'd60);
    chk("S2 panel shrunk: y=60 empty", cap[31:24], 0);
    sample(16'd320, 16'd40);
    chk("S2 panel still there @(320,40)", (cap[31:24] >= 8'd8), 1);
    ink_scan(288, 319, 32, 47);
    chk("S2 countdown ink == 0", ink_n, 0);
    ink_scan(16, 200, 12, 27);
    chk("S2 filename ink > 0", (ink_n > 0), 1);
    ink_scan(16, 127, 32, 47);
    chk("S2 resolution ink > 0", (ink_n > 0), 1);
    shot("out/a_infobar_off.ppm");

    //==========================================================================
    //  场景 3：mode 5（随音乐）—— 与 mode 0 同款：不显示进度条和读秒
    //==========================================================================
    release dut.u_menu.val3;
    force   dut.u_menu.val3 = 7'd5;
    nframe(20);
    chk("S3 auto_mode == 5 (music)", dut.u_menu.auto_mode, 5);
    sample(16'd100, 16'd58);
    chk("S3 track GONE @(100,58)", cap[31:24], 0);
    sample(16'd320, 16'd60);
    chk("S3 panel shrunk: y=60 empty", cap[31:24], 0);
    ink_scan(288, 319, 32, 47);
    chk("S3 countdown ink == 0", ink_n, 0);
    ink_scan(16, 200, 12, 27);
    chk("S3 filename ink > 0", (ink_n > 0), 1);
    shot("out/a_infobar_music.ppm");

    $display("  ---- %0d checks, %0d fails", checks, fails);
    if (fails == 0) $display("  >>> ALL PASS");

    release dut.u_menu.page_r;
    release dut.u_menu.val3;
    release dut.info_on;
    $finish;
end

endmodule
