//==============================================================================
//  tb_osd_icons.v —— 导航图标 + logo（type 12 位图）端到端逐像素回归
//==============================================================================
//
//  【要证的事】
//    ① Flash 的 osd_bitmaps 分区有数据时，osd_icon_rom 必须 ready=1；
//    ② 5 个图标 32×32 × 2 态 的每一个像素都必须等于 osd_bitmaps.bin 里对应的
//       RGB565 展开值（同时守住：地址生成、act 进地址、半字选择、
//       565→888 展开、ROM 字节拼装方向）；
//    ③ 图标必须有"形状"（颜色种数 ≥ 8、墨迹 bbox 不贴边、墨迹像素数合理）；
//    ④ 64×41 的 logo 每一个像素也必须逐点相等；背景必须是硬编码 #0E0E0E
//       （与左栏面板同色 ⇒ 看不出矩形），并且 navy / red 两种墨色都真的存在；
//    ⑤ Flash 分区是擦除态（ready=0）时，图标和 logo **整块不画**（A=0），
//       不能退化成白块；
//    ⑥ 菜单关闭时 logo 不画。
//
//  【2026-10-28 改版】位图格式从「4bpp 覆盖率 + 调色板」改成「RGB565 直出」，
//    期望值不再是"覆盖率经 pal_rgb 查表"，而是"bin 里的 16bpp 值按
//    {R5,R5[4:2]} 展开"—— **从规格独立写**，不抄 RTL。
//
//  【怎么灌数据】w25q64_model_24b 的 INIT_HEX_ICONS：
//      .INIT_HEX       ("tools/font.hex")         // 字库（文字用）
//      .INIT_HEX_ICONS ("tools/osd_bitmaps.hex")  // 25728B 位图 @0x050000
//    osd_bitmaps.hex 由 tools/gen_osd_bitmaps.py 从 bin 逐字节转出。
//
//  【怎么跑】工作目录 = APP/OSD/
//    iverilog -g2012 -s tb_osd_icons -o /tmp/tb_ic.vvp tb_osd_icons.v \
//        osd_top.v osd_menu.v osd_gfx.v osd_cellmem.v osd_widget.v \
//        osd_icon_rom.v flash_arb.v ../osd_font_cache.v ../ps_gen_text.v \
//        ../gb_addr.v ../../spi_flash_read.v ../w25q64_model_24b.v
//    vvp /tmp/tb_ic.vvp
//==============================================================================
`timescale 1ns / 1ps

module tb_osd_icons;

localparam integer NL     = 8;
localparam integer MAXLEN = 32;
localparam integer NSLOT  = 8;          // 与 App.v 完全一致（[LUT-2] 动态字格 8 个）
localparam integer NBLOB  = 30976;

// 位图在屏幕上的落位（必须与 osd_menu 的 LOGO_* / 图标常量一致）
localparam integer NAV_X  = 32;
localparam integer NAV_Y0 = 72;
localparam integer LOGO_X = 16;
localparam integer LOGO_Y = 8;
localparam integer LOGO_W = 64;
localparam integer LOGO_H = 41;

reg clk = 0;
always #10 clk = ~clk;

reg         rst = 1;
reg         ps_start = 0;
reg         pixel_ready = 1;
reg  [15:0] px = 0, py = 0;
reg  [3:0]  key_press = 0;
reg  [3:0]  key_level = 4'hF;
reg         menu_on = 0;
reg         info_show_tb = 0;
reg  [5:0]  auto_remain = 6'd0;         // 倒计时剩余秒（E0-14..17 右对齐要用）

//------------------------------------------------------------------------------
//  名字 RAM 模型（信息条文件名，★ 2026-11-02）
//------------------------------------------------------------------------------
//  真机上 name_mem 是 fat_dir_lister 里 4 槽 × 16 字节的 8.3 名。本台只需要
//  "名字对不对、长度对不对、点插在哪"这三件事，所以用一个 16 字节的数组模拟
//  一个槽：主名 6 字 + 空格补到 8 + 扩展名 3 字。槽号（O_nm_slot）本台恒 0。
//  这里故意用 blen≠8（6）且 elen=3：能同时守住"点插在主名实际长度处"和
//  "按实际长度收行宽"（写死 8 的老版本会显示成 "IMG_01.B"）。
//  ★ 必须声明在 osd_top 例化之前 —— iverilog 对"先用后声明"是硬错。
reg [7:0] nm_name [0:15];
integer nn;
initial begin
    for (nn = 0; nn < 16; nn = nn + 1) nm_name[nn] = 8'h20;
    nm_name[0]  = "I"; nm_name[1]  = "M"; nm_name[2]  = "G"; nm_name[3]  = "_";
    nm_name[4]  = "0"; nm_name[5]  = "1"; nm_name[6]  = " "; nm_name[7]  = " ";
    nm_name[8]  = "B"; nm_name[9]  = "M"; nm_name[10] = "P"; nm_name[11] = " ";
end

// 期望：8.3 = "IMG_01" + "BMP" ⇒ blen=6、elen=3、显示 10 字 "IMG_01.BMP"
localparam [3:0] NM_BLEN = 4'd6;
localparam [3:0] NM_ELEN = 4'd3;
localparam [4:0] NM_USE  = 5'd10;

wire [7:0] nm_char = nm_name[dut.u_menu.O_nm_idx[3:0]];

wire        enter_req, exit_req, osd_layer_en;
wire [7:0]  fade;
wire [31:0] osd_pix;
wire        f_cs, f_sclk, f_mosi, f_wp, f_hold, f_miso;
wire        txt_trig;
wire [2:0]  fx_sel, auto_mode;
wire        info_on, album_prev, album_next;

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
    .fx_sel(fx_sel), .auto_mode(auto_mode),.info_on(info_on),
    .album_prev(album_prev), .album_next(album_next),
    .cur_w(16'd640), .cur_h(16'd480),
    .auto_remain(auto_remain), .auto_bar(9'd0),
    .txt_trig(txt_trig),
    // 画质参数（本台不关心）
    .pic_bright(), .pic_contrast(), .pic_temp(), .pic_sat(),
    .pic_style(), .pic_tgl()
    // 2026-10-31: tie off ports added after this bench was written (were floating Z)
    , .zoom_on(1'b0), .ax(10'd0), .ay(10'd0), .zoom_sel(8'h10), .zoom_mode(2'd0),
    // ★ 2026-11-02 信息条文件名：name_mem 由一个 16 字节的 8.3 名模拟
    .I_nm_char(nm_char), .I_nm_blen(NM_BLEN), .I_nm_elen(NM_ELEN),
    .I_nm_len4(20'd0), .I_nm_vld(1'b1),
    // ★ 2026-11-02：信息条的空闲/换图/名字可用位
    .cur_rel(2'd0), .img_go(1'b0), .nm_upd(1'b0), .nm_hold(1'b0),
    .I_fp_vcnt(3'd0), .I_fp_has_more(1'b0), .I_fp_gen(1'b0), .I_fp_has_prev(1'b0), .O_fp_open(),
    // ★ 字幕的字模 Flash 客户：不接会悬空成 Z，把 flash_arb 拉成 X ⇒ icon_ready 永远 0、
    //   图标/logo 全成纯色块（本台 A1/I/G 组就是这么假 FAIL 的）。
    .sub_fl_req(1'b0), .sub_fl_addr(24'd0), .sub_fl_len(16'd0)
);

//------------------------------------------------------------------------------
//  名字 RAM 模型（见上面的声明段）：本台只需要一个槽的 8.3 名
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//  信息条文字捕获（line6 文件名 / line7 分辨率）
//------------------------------------------------------------------------------
reg [15:0] ic6 [0:15];
reg [15:0] ic7 [0:19];
reg [7:0]  cl6 = 8'd0;          // 第 6 行的 ctl_len（行宽）
reg  [1:0] ci6 = 2'd0;          // 第 6 行的颜色号（信息条固定白色=1）
integer ii;
initial for (ii = 0; ii < 20; ii = ii + 1) begin ic6[ii] = 16'h0; if (ii < 20) ic7[ii] = 16'h0; end
always @(negedge clk) begin
    if (dut.u_menu.str_we) begin
        if (dut.u_menu.str_line == 4'd6 && dut.u_menu.str_idx < 16)
            ic6[dut.u_menu.str_idx] <= dut.u_menu.str_code;
        if (dut.u_menu.str_line == 4'd7 && dut.u_menu.str_idx < 20)
            ic7[dut.u_menu.str_idx] <= dut.u_menu.str_code;
    end
    if (dut.u_menu.ctl_we && (dut.u_menu.ctl_line == 4'd6)) begin
        cl6 <= dut.u_menu.ctl_len;
        ci6 <= dut.u_menu.txt_ci[6*2 +: 2];
    end
end

//------------------------------------------------------------------------------
//  ★ 菜单标签文字捕获（第 0..5 行的第 0 列）
//------------------------------------------------------------------------------
//  为什么加这个：label 表曾经从组合 LUT 换成 BRAM ROM 之后，上板出现
//  "每行第 1 个字来自上一行"（显示→显度、亮度→亮比度），而所有仿真台都是绿的。
//  根因是"地址比数据需求晚一拍"。这条断言守住的就是"col=0 必须是本行的第 1 个字"。
reg [15:0] lb0 [0:7];
initial for (ii = 0; ii < 8; ii = ii + 1) lb0[ii] = 16'h0;
always @(negedge clk) begin
    if (dut.u_menu.str_we && (dut.u_menu.str_idx == 8'd0))
        lb0[dut.u_menu.str_line] <= dut.u_menu.str_code;
end

//------------------------------------------------------------------------------
//  Flash：字库 + 位图（真实 osd_bitmaps.bin 的 25728 字节）
//------------------------------------------------------------------------------
w25q64_model_24b #(
    // ★ 必须 ≥ 0x050000 + 25728 = 353408，否则 $readmemh 报
    //   "Start address 327680 is out of bounds" 并**静默不灌**，
    //   仿真里 ROM 全是 X（看起来像"位图全错"，其实是台子的容量不够）。
    .MEM_BYTES     (393216),
    .INIT_HEX      ("tools/font.hex"),
    .INIT_HEX_ICONS("tools/osd_bitmaps.hex"),
    .ICON_ADDR     (24'h050000)
) u_flash (
    .cs_n(f_cs), .sclk(f_sclk), .mosi(f_mosi), .miso(f_miso)
);

//------------------------------------------------------------------------------
//  期望值：直接从 bin 的字节算（规格：16bpp 光栅序，低半字 = 偶像素）
//------------------------------------------------------------------------------
reg [7:0] blob [0:NBLOB-1];
initial $readmemh("tools/osd_bitmaps.hex", blob);

// 图标的 16bpp 像素：word = idx*1024 + act*512 + ly*16 + (lx>>1)
function [15:0] icon_px;
    input integer idx, act, lx, ly;
    integer w;
    reg [7:0] b0, b1, b2, b3;
    begin
        w  = idx * 1024 + act * 512 + ly * 16 + (lx >> 1);
        b0 = blob[w*4];
        b1 = blob[w*4 + 1];
        b2 = blob[w*4 + 2];
        b3 = blob[w*4 + 3];
        icon_px = lx[0] ? {b3, b2} : {b1, b0};
    end
endfunction

// logo 的 ARGB8888 像素：word = 5120 + ly*64 + lx（1 像素 1 字）
//   文件字节序 B,G,R,A ⇒ 字 = {A,R,G,B}
function [31:0] logo_argb;
    input integer lx, ly;
    integer w;
    begin
        w = 5120 + ly * 64 + lx;
        logo_argb = {blob[w*4 + 3], blob[w*4 + 2], blob[w*4 + 1], blob[w*4]};
    end
endfunction

// RGB565 → RGB888（{R5,R5[4:2],G6,G6[5:4],B5,B5[4:2]}），哨兵 0 ⇒ 面板色。
// ★ 独立按规格写，不抄 RTL。
function [23:0] exp_rgb;
    input [15:0] v;
    reg [4:0] r5;
    reg [5:0] g6;
    reg [4:0] b5;
    begin
        r5 = v[15:11];
        g6 = v[10:5];
        b5 = v[4:0];
        exp_rgb = (v == 16'h0000) ? 24'h0E0E0E
                                 : {r5, r5[4:2], g6, g6[5:4], b5, b5[4:2]};
    end
endfunction

//------------------------------------------------------------------------------
//  帧节拍（layout / tick 需要 ps_start 落沿）
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

//------------------------------------------------------------------------------
//  采一个像素（osd_pix 相对 (x,y) 晚 3 拍）
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

//------------------------------------------------------------------------------
//  断言
//------------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
//  逐图标检查
//------------------------------------------------------------------------------
integer ic, lx, ly;
integer pix_err, distinct, ink_n, bbox_x0, bbox_x1, bbox_y0, bbox_y1;
reg [23:0] got_rgb, base_rgb, exp_lo;
reg [15:0] v565;
reg        act;
integer    k2;
reg [15:0] seen [0:31];

task check_icon;
    input integer idx;
    begin
        pix_err  = 0;
        ink_n    = 0;
        bbox_x0  = 99; bbox_x1 = -1; bbox_y0 = 99; bbox_y1 = -1;
        act      = (idx == 0);              // 导航栏焦点在最上面那个
        base_rgb = exp_rgb(icon_px(idx, act, 0, 0));
        distinct = 0;
        for (ly = 0; ly < 32; ly = ly + 1) begin
            for (lx = 0; lx < 32; lx = lx + 1) begin
                sample(NAV_X + lx, NAV_Y0 + idx*80 + ly);
                got_rgb = cap[23:0];
                v565    = icon_px(idx, act, lx, ly);
                exp_lo  = exp_rgb(v565);
                if (cap[31:24] !== 8'hFF || got_rgb !== exp_lo) begin
                    pix_err = pix_err + 1;
                    if (pix_err <= 4)
                        $display("        pixel(%0d,%0d): got A=%02h RGB=%06h, expect A=FF RGB=%06h (565=%04h)",
                                 lx, ly, cap[31:24], got_rgb, exp_lo, v565);
                end
                if (exp_lo !== base_rgb) begin
                    ink_n = ink_n + 1;
                    if (lx < bbox_x0) bbox_x0 = lx;
                    if (lx > bbox_x1) bbox_x1 = lx;
                    if (ly < bbox_y0) bbox_y0 = ly;
                    if (ly > bbox_y1) bbox_y1 = ly;
                end
                // 颜色种数（只需要 ≥8，所以记满 8 个就不再扫，避免 O(n²) 拖慢仿真）
                if (distinct < 8) begin
                    for (k2 = 0; k2 < distinct; k2 = k2 + 1)
                        if (seen[k2] == v565) k2 = 99;
                    if (k2 != 100) begin
                        seen[distinct] = v565;
                        distinct = distinct + 1;
                    end
                end
            end
        end

        $display("  --- icon %0d (act=%0d): pix_err=%0d  distinct=%0d  ink=%0d  bbox=(%0d,%0d)-(%0d,%0d)",
                 idx, act, pix_err, distinct, ink_n, bbox_x0, bbox_y0, bbox_x1, bbox_y1);
        chk("I pixel exact", pix_err, 0);
        chk("I has shape (colors>=8)", (distinct >= 8), 1);
        chk("I ink count sane", ((ink_n > 80) && (ink_n < 600)), 1);
        chk("I bbox not full tile", ((bbox_x0 >= 2) && (bbox_y0 >= 2) &&
                                     (bbox_x1 <= 29) && (bbox_y1 <= 29)), 1);
    end
endtask

//------------------------------------------------------------------------------
//  逐像素检查 logo
//------------------------------------------------------------------------------
integer lg_err, lg_bg, lg_navy, lg_red;
reg [23:0] lg_rgb;
reg [31:0] exp_argb;

task check_logo;
    begin
        lg_err = 0; lg_bg = 0; lg_navy = 0; lg_red = 0;
        for (ly = 0; ly < LOGO_H; ly = ly + 1) begin
            for (lx = 0; lx < LOGO_W; lx = lx + 1) begin
                sample(LOGO_X + lx, LOGO_Y + ly);
                got_rgb  = cap[23:0];
                exp_argb = logo_argb(lx, ly);
                if (cap !== exp_argb) begin
                    lg_err = lg_err + 1;
                    if (lg_err <= 6)
                        $display("        logo(%0d,%0d): got %08h, expect %08h",
                                 lx, ly, cap, exp_argb);
                end
                // 纯背景 = A192 + #0E0E0E；纯墨 = A255
                if (exp_argb[31:24] == 8'd192) lg_bg = lg_bg + 1;
                // 蓝多 = navy（A 字），红多 = 红条纹
                if (got_rgb[23:16] > got_rgb[7:0] + 8) lg_red  = lg_red  + 1;
                if (got_rgb[7:0]   > got_rgb[23:16] + 8) lg_navy = lg_navy + 1;
            end
        end
        $display("  --- logo: pix_err=%0d  bg(A192)=%0d  navy=%0d  red=%0d",
                 lg_err, lg_bg, lg_navy, lg_red);
    end
endtask

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
integer wait_cnt;
initial begin
    repeat (4) @(posedge clk);
    rst = 0;
    menu_on = 1;

    // 等位图 ROM 搬完（POR 400k + 25728B SPI）
    wait_cnt = 0;
    while ((dut.icon_ready !== 1'b1) && (wait_cnt < 4_000_000)) begin
        @(posedge clk);
        wait_cnt = wait_cnt + 1;
    end
    $display("=== A. 位图 ROM 就绪 ===");
    chk("A1 icon_ready", dut.icon_ready, 1);
    $display("    等待 %0d 拍", wait_cnt);

    // 等菜单布局写完（多帧 tick）
    nframe(40);

    // ★ 2026-10-29 [S6]：图标（5..9）和 logo（19）已从 cellmem 搬到 osd_gfx 的
    //   常量表（几何是编译期常量），cellmem 里不再有这些表项，原来的"读 attr_r[5+ic]"
    //   断言没有对象了。它们的正确性由**逐像素比对**覆盖：
    //   A/B 组已经把 5 个图标各 1024 点、logo 2624 点全部对齐源文件解码值。
    // 导航栏统一下移 48px：y0 = 72+80i 现在由 osd_gfx 的常量矩形保证，
    //   C 组的 1024 点/图标逐像素比对已经把这 5 个原点全部钉死（错 1px 就会全红）。
    // 激活态掩码：NAV 态、nav_idx=0 ⇒ 只有 bit0
    chk("B ic_act_mask onehot", dut.ic_act_mask, 8'h01);

    $display("=== C. 逐像素检查 5 个图标 ===");
    for (ic = 0; ic < 5; ic = ic + 1)
        check_icon(ic);

    //--------------------------------------------------------------------------
    //  G. logo 逐像素 + 透明底 + 两种墨色
    //--------------------------------------------------------------------------
    $display("=== G. logo 64x41 逐像素 ===");
    check_logo;
    chk("G1 logo pixel exact", lg_err, 0);
    chk("G2 logo has 75% bg pixels", (lg_bg > 800), 1);
    chk("G3 logo has navy ink",    (lg_navy > 200), 1);
    chk("G4 logo has red ink",     (lg_red  > 100), 1);
    // 背景像素必须**逐位**等于面板色，否则会出现一个比面板深的矩形
    // ★ 角落 = A192(75%) + #0E0E0E，**和左栏面板逐位相同** ⇒ 看不出方框
    //   （写成 A=255 就会变成一块实心矩形，面板下面那 25% 视频透不上来）
    sample(LOGO_X, LOGO_Y);
    chk("G5 logo corner A=192",   cap[31:24], 192);
    chk("G6 logo corner = panel", cap[23:0], 24'h0E0E0E);
    // 和左栏面板（cell1）取同一个颜色，证明"没有白框"
    sample(60, 300);
    chk("G7 panel RGB same", cap[23:0], 24'h0E0E0E);
    chk("G7b panel A=192",   cap[31:24], 192);

    //--------------------------------------------------------------------------
    //  D. Flash 未编程：强制 ready=0，图标和 logo 都必须整块不画
    //--------------------------------------------------------------------------
    $display("=== D. icon_ready=0 时不留白块 ===");
    force dut.icon_ready = 1'b0;
    sample(NAV_X + 16, NAV_Y0 + 16);
    chk("D1 icon area transparent (A=0)", cap[31:24], 0);
    sample(NAV_X + 4,  NAV_Y0 + 5);
    chk("D2 icon edge transparent (A=0)", cap[31:24], 0);
    sample(LOGO_X + 30, LOGO_Y + 20);
    chk("D3 logo area transparent (A=0)", cap[31:24], 0);
    release dut.icon_ready;

    //--------------------------------------------------------------------------
    //  E. 信息条：真圆角 + 进度条无圆角
    //--------------------------------------------------------------------------
    $display("=== E. info bar rounded corners ===");
    menu_on = 0;
    info_show_tb = 1;
    force dut.info_on = 1'b1;          // 等价于菜单里勾上"信息条"
    nframe(30);
    // ★ 菜单关闭 ⇒ 布局走 `!menu_on` 覆盖段，把 cell19 的矩形清零。
    //   这里不能靠采像素判：信息条面板 (8,8)-(328,74) 正好盖住 logo 的 (16,8)-(80,49)，
    //   采到的是面板色不是"未画"。所以直接断言几何被清零。
    // ★ [S6]：logo 的矩形现在是 osd_gfx 里的常量，唯一开关是 menu_on ⇒
    //   直接断言"菜单关着时 logo 命中恒为 0"（比读表更贴近语义）。
    chk("G8 logo hit off when menu off", dut.u_gfx.c_hit_t19, 0);
    // 信息条文字：★ 2026-11-02 起是**真实文件名**（来自名字 RAM），不再是
    // 写死的 "TEST.BMP"。本台喂进去的 8.3 名 = "IMG_01" + "BMP"：
    //     显示 "IMG_01.BMP"（10 字），点插在主名长度（第 6 格）处。
    //  三重断言：① 逐字内容；② 行宽 = 主名+点+扩展名（写死 8 会截掉 "MP"）；
    //            ③ 颜色号 = 1（信息条固定白色），且第 10 格不许是 'I'（不循环）。
    $display("=== E0. info bar text ===");
    chk("E0-1 name I",        ic6[0], 16'h0049);
    chk("E0-2 name M",        ic6[1], 16'h004D);
    chk("E0-3 name G",        ic6[2], 16'h0047);
    chk("E0-4 name _",        ic6[3], 16'h005F);
    chk("E0-5 name 0",        ic6[4], 16'h0030);
    chk("E0-6 name 1",        ic6[5], 16'h0031);
    chk("E0-7 dot at blen(6)", ic6[6], 16'h002E);
    chk("E0-8 ext B",         ic6[7], 16'h0042);
    chk("E0-9 ext M",         ic6[8], 16'h004D);
    chk("E0-10 ext P",        ic6[9], 16'h0050);
    chk("E0-11 line width = nm_use(10)", {3'd0, cl6}, {3'd0, NM_USE});
    chk("E0-12 name colour = white", ci6, 2'd1);
    chk("E0-13 no wrap at col10", (ic6[10] == 16'h0049) ? 1 : 0, 0);
    chk("E0-14 res x at col3", ic7[3], 16'h0078);
    chk("E0-15 H starts col4", ic7[4], 16'h0034);   // 640x480 的 '4'
    chk("E0-16 no space before x", (ic7[2] == 16'h0020) ? 1 : 0, 0);
    chk("E0-17 W digits", {ic7[0][7:0], ic7[1][7:0], ic7[2][7:0]}, 24'h363430);
    // 倒计时右对齐：个位固定第 18 列，十位第 17 列
    force dut.u_menu.page_r = 3'd1;
    force dut.u_menu.val3   = 7'd1;      // 自动切换 3s
    auto_remain = 6'd3;  nframe(30);
    chk("E0-14 countdown ones at col18", ic7[18], 16'h0033);   // '3'
    chk("E0-15 countdown tens blank",    ic7[17], 16'h0000);
    auto_remain = 6'd30; nframe(30);
    chk("E0-16 countdown tens at col17", ic7[17], 16'h0033);   // '3'
    chk("E0-17 countdown ones at col18", ic7[18], 16'h0030);   // '0'
    release dut.u_menu.page_r;
    release dut.u_menu.val3;
    auto_remain = 6'd0;  nframe(20);
    // 面板：中心 75% 面板色；四角透明（露出底图）
    sample(300, 68);                     // 面板底部空白处（避开文件名/分辨率/进度条）
    chk("E1 panel centre A=192", cap[31:24], 192);
    chk("E2 panel centre RGB",   cap[23:0], 24'h202020);
    sample(8, 8);
    chk("E3 panel corner A=0",   cap[31:24], 0);
    sample(9, 9);
    chk("E4 panel corner(9,9) A=0", cap[31:24], 0);
    sample(327, 8);
    chk("E4b panel TR A=0", cap[31:24], 0);
    sample(8, 73);
    chk("E4c panel BL A=0", cap[31:24], 0);
    sample(327, 73);
    chk("E4d panel BR A=0", cap[31:24], 0);
    sample(14, 14);
    chk("E5 panel inside corner A=192", cap[31:24], 192);
    // 槽属性：面板 rad=6 / mask=3；进度条 rad=0
    chk("E6 panel rad=6",  dut.u_gfx.u_mem.cell_rd[6][15:12], 6);   // 16 → 动态槽 6
    chk("E7 panel mask=3", dut.u_gfx.u_mem.cell_rd[6][11:10], 3);
    chk("E8 progress rad=0", dut.u_gfx.u_mem.cell_rd[7][15:12], 0);  // 17 → 动态槽 7
    // 进度条轨道：类型 13，无圆角 ⇒ (16,52) 是 C_BOX（不是切角继承色）
    sample(16, 52);
    chk("E9 progress corner is C_BOX", cap[23:0], 24'h202020);
    // 轨道本体：x=20,y=56（lx=4,ly=4）应是 0x505050/255
    sample(20, 56);
    chk("E10 progress track A=255", cap[31:24], 255);
    chk("E11 progress track RGB",   cap[23:0], 24'h505050);
    // ---- F. 菜单标签文字（守住"col=0 是本行第一个字"）----
    //  期望码从 GB2312 规格抄（不是从 RTL 翻）：
    //    显示=CFD4 CABE 亮=C1C1 度=B6C8 对=B6D4 比=B1C8 色=C9AB 温=CEC2
    //    饱=B1A5 和=BACD 风=B7E7 格=B8F1
    $display("--- F. label first char of each row (page0) ---");
    chk("F0 row0 title 'xian'", lb0[0], 16'hCFD4);
    chk("F1 row1 'liang'",      lb0[1], 16'hC1C1);
    chk("F2 row2 'dui'",        lb0[2], 16'hB6D4);
    chk("F3 row3 'se'",         lb0[3], 16'hC9AB);
    chk("F4 row4 'bao'",        lb0[4], 16'hB1A5);
    chk("F5 row5 'feng'",       lb0[5], 16'hB7E7);

    release dut.info_on;

    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

endmodule
