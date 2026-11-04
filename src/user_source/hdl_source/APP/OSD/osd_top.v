//==============================================================================
//  osd_top.v —— OSD 叠加层总成（图形层 + 文字层 + 字库缓存 + 图标 ROM + Flash 仲裁）
//==============================================================================
//
//        x,y ──┬─► osd_gfx  (控件表, 2 拍) ─┐
//              │        ▲                  │
//              │    osd_icon_rom (图标)    ├─► merge (1 拍) ─► {A8,RGB}
//              └─► ps_gen_text (2 拍) ─────┘
//                        ▲
//                   osd_font_cache
//                        │ 客户口
//   Flash 引脚 ◄─ spi_flash_read ◄─ flash_arb ◄─┬─ 字库
//                                               ├─ 图标（上电一次）
//                                               └─ 提示音（顶层 beep_player）
//
//  【为什么两层同深】osd_gfx 与 ps_gen_text 都恰好 2 拍（命中/RAM 地址寄存器 +
//  像素寄存器），所以合并级只加 1 拍；再和 A 层的 3 拍在 ps_merge_alpha_pp 里
//  汇合（4 拍），**frame_writer.PIPE 仍然是 4，不需要改**。
//
//  ★ 2026-10-27：Flash 从"字库私有"改成三客户共享（见 flash_arb.v）。
//    控制器只留一个实例，引脚仍由本模块驱动（顶层不用改引脚分配）。
//
//==============================================================================
module osd_top #(
    parameter integer NL     = 8,
    parameter integer MAXLEN = 32,
    // 遮罩0 左栏1 分隔2 页面板3 游标框/弹层板4 图标5..9 行控件10..14
    // 标题线15 弹层选项16..18 ★logo19（2026-10-28 新增，最高槽位）
    parameter integer NSLOT  = 8,      // ★ [S6] 动态字格（[LUT-2] 9 → 8）
    // ★ 2026-10-27：255 → 127。
    //   【为什么能降】槽位的需求量 = **当前显示的不同字符数**（相同字符靠
    //     tag_word 命中共享），而不是"历史上出现过的字符总数"：
    //     一行换了内容，旧槽的 owner 位会被清掉、owner 归零就当空闲槽回收。
    //     实测：菜单模式 8 行里真正有字的约 40 格，信息条 2 行 38 格
    //     ⇒ 峰值 ~50 个不同字形，127 有 2.5 倍余量。
    //   【省多少】owner 256×8×3 份 dram 副本 → 128×8×3（约 -192 LUT）；
    //     glyph_mem 4096×16 → 2048×16（-4 个 BRAM，从 89% 降到 83%）。
    //   【为什么必须是 127】osd_font_cache 要求 NSLOT = 2^k − 1，
    //     这样 NROW = (NSLOT+1)×16 才是 2 的幂（128×16 = 2048），
    //     BRAM 地址端口位宽才匹配（见那里的 §5 说明）。
    parameter integer NSLOT_F= 127,
    parameter integer TEXT_EN = 1,
    parameter integer SELFTEST = 0,
    parameter integer RPT_FIRST = 30,
    parameter integer RPT_EVERY = 5
)(

    // ---- 文件页状态（给四宫格）----
    output wire [2:0]  O_page,

    // ---- 文件页文件名读口（放在最前，避开 TD 端口列表末尾解析问题）----
    output wire [1:0]  O_nm_slot,
    output wire [3:0]  O_nm_idx,
    input  wire [7:0]  I_nm_char,
    input  wire [3:0]  I_nm_blen,       // 当前格主名长度
    input  wire [3:0]  I_nm_elen,       // 当前格扩展名长度
    input  wire [19:0] I_nm_len4,       // 4 格的显示字符数（每格 5 位）
    input  wire        I_nm_vld,        // 当前格有没有内容

    // ---- 文件页光标（2026-11-01 P5）----
    output wire [1:0]  O_fp_cur,        // 当前光标格
    output wire        O_fp_flip_nx,    // 单拍：请求翻下一页
    output wire        O_fp_flip_pv,    // 单拍：请求翻上一页
    input  wire [2:0]  I_fp_vcnt,       // 本页有效格数
    input  wire        I_fp_has_more,   // 后面还有没有
    input  wire        I_fp_gen,        // 每扫完一页翻转一次
    input  wire        I_fp_has_prev,   // ★ 2026-11-04：前面还有上一页
    output wire        O_fp_open,       // ★ 2026-11-04：单拍，文件页 KEY2 打开光标格

    input  wire        clk,             // 50MHz，与场景图同域
    input  wire        rst,

    input  wire        ps_start,
    input  wire        pixel_ready,
    input  wire [15:0] x,
    input  wire [15:0] y,

    input  wire [3:0]  key_press,       // {k4,k3,k2,k1}
    input  wire [3:0]  key_level,       // 1 = 松开

    input  wire        menu_active,     // 来自 viewer_ctrl：S_OSD
    input  wire        osd_allow,       // 允许开菜单（S_HD / S_STUCK）
    // ★ 2026-10-27：现在也包含 S_ARM/S_TRANS/S_LOW —— 切换动画期间
    //   信息条必须保持可见（viewer_scene 已改成逐像素叠加，不再吃掉 B 层）。
    input  wire        info_show,       // 信息条可以显示（静止态 + 切换动画全程）

    // ★ 2026-10-30 缩放 OSD：十字准星 + 倍数文字
    input  wire        zoom_on,         // 缩放 OSD 打开
    input  wire [9:0]  ax, ay,          // 十字中心（屏幕坐标 = crx/cry）
    input  wire [7:0]  zoom_sel,        // Q4.4 当前倍数（0x10/0x20/0x40/0x80）
    input  wire [1:0]  zoom_mode,       // 当前功能：0=左右 1=上下 2=缩放

    // ---- 用户 SPI Flash 物理引脚 ----
    output wire        flash_cs_n,
    output wire        flash_sclk,
    output wire        flash_mosi,
    output wire        flash_wp_n,
    output wire        flash_hold_n,
    input  wire        flash_miso,

    // ---- 提示音的 Flash 客户口（透传到顶层 beep_player）----
    input  wire        bp_req,
    input  wire [23:0] bp_addr,
    input  wire [15:0] bp_len,
    output wire        bp_rdy,
    output wire [7:0]  bp_data,
    output wire        bp_valid,
    output wire        bp_done,

    // ---- 字幕字模的 Flash 客户口（透传到顶层 sub_loader，★2026-11-06）----
    input  wire        sub_fl_req,
    input  wire [23:0] sub_fl_addr,
    input  wire [15:0] sub_fl_len,
    output wire        sub_fl_rdy,
    output wire [7:0]  sub_fl_data,
    output wire        sub_fl_valid,
    output wire        sub_fl_done,

    // ---- 控制/状态 ----
    output wire        enter_req,
    output wire        exit_req,
    output wire        osd_layer_en,
    output wire [7:0]  fade,

    // ---- 像素输出 {A8,R,G,B} ----
    output wire [31:0] osd_pix,

    // ---- 画质参数 ----
    output wire [6:0]  pic_bright,
    output wire [6:0]  pic_contrast,
    output wire [6:0]  pic_temp,
    output wire [6:0]  pic_sat,
    output wire [1:0]  pic_style,
    output wire        pic_tgl,
    // ★ 2026-11-03：ALS 生效亮度（只用于显示页亮度行，不参与画质参数导出）
    input  wire [6:0]  bright_eff,

    // ---- 相册页导出（2026-10-27）----
    output wire [2:0]  fx_sel,
    output wire [2:0]  auto_mode,
    output wire        info_on,
    output wire        album_prev,
    output wire        album_next,

    // ---- 系统页导出（2026-10-29）→ 顶层音频链路 ----
    output wire        o_lang,          // 0=中文 1=English（同时决定标签 ROM 地址）
    output wire [6:0]  o_vol,           // 音量 0..100
    output wire        o_mute,          // 静音（与 SW2 串联）
    output wire        o_loop,          // 循环播放
    output wire        o_spc,           // 频谱条（预留）
    output wire        o_asy,           // ★ 2026-11-02 音随图切（相册页 item5 CheckBox）
    output wire        o_auto,          // ★ 2026-11-03 自动亮度（显示页 item6 CheckBox）

    // ---- 来自 viewer_ctrl 的信息条数据 ----
    input  wire [15:0] cur_w,
    input  wire [15:0] cur_h,
    input  wire [5:0]  auto_remain,
    input  wire [8:0]  auto_bar,

    // ---- ★ 2026-11-02 信息条文件名（本模块只透传，逻辑在 osd_menu 的 §6）----
    input  wire [1:0]  cur_rel,         // 当前图在相册窗口里的位置(= name_mem 槽号)
    input  wire        img_go,          // 单拍：换图成立 ⇒ 文字重写一次
    input  wire        nm_upd,          // 单拍：相册重扫完成 ⇒ 文件名重写一次
    input  wire        nm_hold,         // ★ [NM-HOLD] 名字读口被 match 借走 ⇒ 本行写空格

    // ---- 文字表已更新（信息条在"停帧"状态下补画一帧用）----
    output wire        txt_trig
);

localparam integer NCW = 32*NL;

//------------------------------------------------------------------------------
//  2. 帧边界窗口：ps_start 落沿后 (PIPE + 2) 拍
//------------------------------------------------------------------------------
localparam integer PIPE = 5;   // ★ 必须与场景流水线深度一致（[DSP-PIPE] 后 4→5）

reg        ps_start_d;
reg  [4:0] tt_cnt;
wire       tick_w = (tt_cnt == 5'd1);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        ps_start_d <= 1'b0;
        tt_cnt     <= 5'd0;
    end else begin
        ps_start_d <= ps_start;
        if (ps_start_d & ~ps_start) tt_cnt <= 5'd7;      // = PIPE(5) + 2（[DSP-PIPE] 后 4→5）
        else if (tt_cnt != 5'd0)    tt_cnt <= tt_cnt - 5'd1;
    end
end

//------------------------------------------------------------------------------
//  1. 菜单控制
//------------------------------------------------------------------------------
wire        wr_en;
wire [4:0]  wr_idx;
wire [38:0] wr_geo;
wire [23:0] wr_attr;

// ★ 2026-10-28：导航图标激活态掩码（osd_menu 组合导出，给 gfx 拍 0 的地址用）
//   必须声明在 u_menu 之前 —— iverilog 对"先用后声明"会直接报错。
wire [7:0]  ic_act_mask;

// ★ 2026-10-27：位宽收窄后两者不再等宽，必须分开声明
wire [22*NL-1:0] txt_xy;
wire [2*NL-1:0]  txt_ci;          // 每行 2 位颜色号（见 ps_gen_text）

wire        str_we, ctl_we, trig;
wire [3:0]  str_line, ctl_line;
wire [7:0]  str_idx, ctl_len, ctl_ver;
wire [15:0] str_code;
wire        ctl_active;

osd_menu #(.NL(NL), .MAXLEN(MAXLEN), .SELFTEST(SELFTEST),
           .RPT_FIRST(RPT_FIRST), .RPT_EVERY(RPT_EVERY)) u_menu (
    .clk        (clk),
    .rst        (rst),
    .tick       (tick_w),
    .ps_start   (ps_start),
    .menu_on    (menu_active),
    // ★ 2026-10-30 缩放 OSD 复用信息条的槽 16 + 文字行 6/7
    .zoom_on    (zoom_on),
    .zoom_val   (zoom_sel),
    .zoom_mode  (zoom_mode),
    .osd_allow  (osd_allow),
    .key_press  (key_press),
    .key_level  (key_level),
    .bright_eff (bright_eff),
    .enter_req  (enter_req),
    .exit_req   (exit_req),
    .fade       (fade),
    .wr_en      (wr_en),
    .wr_idx     (wr_idx),
    .wr_geo     (wr_geo),
    .wr_attr    (wr_attr),
    .txt_xy     (txt_xy),
    .txt_ci     (txt_ci),
    .O_page     (O_page),
    .O_nm_slot  (O_nm_slot),
    .O_nm_idx   (O_nm_idx),
    .I_nm_char  (I_nm_char),
    .I_nm_blen  (I_nm_blen),
    .I_nm_elen  (I_nm_elen),
    .I_nm_len4  (I_nm_len4),
    .I_nm_vld   (I_nm_vld),
    .O_fp_cur     (O_fp_cur),
    .O_fp_flip_nx (O_fp_flip_nx),
    .O_fp_flip_pv (O_fp_flip_pv),
    .I_fp_vcnt    (I_fp_vcnt),
    .I_fp_has_more(I_fp_has_more),
    .I_fp_gen     (I_fp_gen),
    .I_fp_has_prev(I_fp_has_prev),
    .O_fp_open    (O_fp_open),

    .str_we     (str_we),
    .str_line   (str_line),
    .str_idx    (str_idx),
    .str_code   (str_code),
    .ctl_we     (ctl_we),
    .ctl_line   (ctl_line),
    .ctl_active (ctl_active),
    .ctl_len    (ctl_len),
    .ctl_ver    (ctl_ver),
    .trig       (trig),
    .pic_bright  (pic_bright),
    .pic_contrast(pic_contrast),
    .pic_temp    (pic_temp),
    .pic_sat     (pic_sat),
    .pic_style   (pic_style),
    .pic_tgl     (pic_tgl),
    // 相册页导出
    .fx_sel      (fx_sel),
    .auto_mode   (auto_mode),
    .info_on     (info_on),
    .album_prev  (album_prev),
    .album_next  (album_next),
    // 系统页导出（2026-10-29）
    .o_lang      (o_lang),
    .o_vol       (o_vol),
    .o_mute      (o_mute),
    .o_loop      (o_loop),
    .o_spc       (o_spc),
    .o_asy       (o_asy),
    .o_auto      (o_auto),
    .ic_act_mask (ic_act_mask),
    // 信息条数据
    .src_w       (cur_w),
    .src_h       (cur_h),
    .auto_remain (auto_remain),
    .auto_bar    (auto_bar),
    // ★ 2026-11-02 信息条文件名（当前图槽号 / 换图单拍 / 重扫完成单拍）
    .nm_rel      (cur_rel),
    .img_go      (img_go),
    .nm_upd      (nm_upd),
    .nm_hold     (nm_hold)
);

// ★ 信息条与菜单共用同一个叠加层：两者互斥（菜单开时信息条隐藏）
// ★ 2026-10-30：缩放 OSD 也走这一层（十字条 + 倍数文字）
assign osd_layer_en = menu_active | (info_on & info_show) | zoom_on;
assign txt_trig     = trig;

//------------------------------------------------------------------------------
//  3. 图形层 + 位图 ROM（图标 + logo）
//------------------------------------------------------------------------------
// ★ 位图 ROM 就绪（搬完 + Flash 区域确实编程过）。
//   声明必须在 u_gfx 之前 —— iverilog 对"先用后声明"会直接报错。
wire        icon_ready;

wire [31:0] gfx_pix;
wire        gfx_v;
wire [12:0] icon_ra;
wire [31:0] icon_rd;

osd_gfx #(.NSLOT(NSLOT)) u_gfx (
    .clk       (clk),
    .rst       (rst),
    .pipe_ce   (pixel_ready),
    .adv       (ps_start & pixel_ready),
    .x         (x),
    .y         (y),
    .wr_en     (wr_en & osd_layer_en),
    .wr_idx    (wr_idx),
    .wr_geo    (wr_geo),
    .wr_attr   (wr_attr),
    .icon_raddr(icon_ra),
    .icon_word (icon_rd),
    .icon_ok   (icon_ready),        // ★ ROM 未就绪/未编程 ⇒ 图标和 logo 都不画
    .ic_act_mask(ic_act_mask),
    // ★ 2026-10-29 [S6]：常量字格（遮罩/左栏/分割线/页面板/图标/标题线/logo）
    //   的命中判定要用它 —— 菜单关掉时 osd_menu 会把这些矩形清零。
    .menu_on   (menu_active),
    .pix       (gfx_pix),
    .out_valid (gfx_v)
);

//------------------------------------------------------------------------------
//  4. 文字层
//------------------------------------------------------------------------------
wire [8*NL-1:0]  txt_slot_col;
wire [10*NL-1:0] txt_slot_tbl;
wire [13:0]      txt_glyph_addr;
wire [15:0]      txt_glyph_row;
wire [8*NL-1:0]  txt_line_len;
wire [NL-1:0]    txt_line_ready;
wire [NL-1:0]    txt_line_ovf;
wire [31:0]      txt_pix;
wire             txt_v;

// 字库的 Flash 客户口
wire        fc_req;
wire [23:0] fc_addr;
wire [15:0] fc_len;
wire        fc_rdy;
wire [7:0]  fc_data;
wire        fc_valid;
wire        fc_done;

// 图标加载的 Flash 客户口
wire        ic_req;
wire [23:0] ic_addr;
wire [15:0] ic_len;
wire        ic_rdy;
wire [7:0]  ic_data;
wire        ic_valid;
wire        ic_done;
// ★ 2026-10-27 清理：ic_we / ic_wa / ic_wd 是早期"图标可写"方案的残留，
//   osd_icon_rom 现在是只读（从 Flash 装载），这三根线既无驱动也无负载 ⇒ 删。

generate
if (TEXT_EN != 0) begin : g_text

ps_gen_text #(.IMG_W(640), .IMG_H(480), .NL(NL), .MAXLEN(MAXLEN)) u_text (
    .clk        (clk),
    .rst        (rst),
    .x          (x),
    .y          (y),
    .valid      (ps_start),
    .ce         (pixel_ready),
    .txt_xy     (txt_xy),
    .txt_ci     (txt_ci),
    .slot_tbl   (txt_slot_tbl),
    .line_len   (txt_line_len),
    .line_ready (txt_line_ready),
    .glyph_row  (txt_glyph_row),
    .slot_col   (txt_slot_col),
    .glyph_addr (txt_glyph_addr),
    .pix        (txt_pix),
    .out_valid  (txt_v)
);

osd_font_cache #(
    .NL        (NL),
    .MAXLEN    (MAXLEN),
    .NSLOT     (NSLOT_F),
    .SCLK_DIV  (1),
    .POR_CYCLES(200000)
) u_fc (
    .clk        (clk),
    .rst        (rst),

    .str_we     (str_we),
    .str_line   (str_line),
    .str_idx    (str_idx),
    .str_code   (str_code),
    .ctl_we     (ctl_we),
    .ctl_line   (ctl_line),
    .ctl_active (ctl_active),
    .ctl_len    (ctl_len),
    .ctl_ver    (ctl_ver),
    .trigger    (trig),

    .slot_rd_col (txt_slot_col),
    .slot_rd_data(txt_slot_tbl),
    .glyph_rd_addr(txt_glyph_addr),
    .glyph_rd_row (txt_glyph_row),

    .line_ready  (txt_line_ready),
    .line_overflow(txt_line_ovf),
    .line_len    (txt_line_len),

    // 客户口（不再是物理引脚）
    .fl_req      (fc_req),
    .fl_addr     (fc_addr),
    .fl_len      (fc_len),
    .fl_rdy      (fc_rdy),
    .fl_data     (fc_data),
    .fl_valid    (fc_valid),
    .fl_done     (fc_done)
);

end else begin : g_no_text
    assign txt_pix = 32'h0000_0000;
    assign fc_req  = 1'b0;
    assign fc_addr = 24'd0;
    assign fc_len  = 16'd0;
end
endgenerate

//------------------------------------------------------------------------------
//  5. 位图 ROM（导航图标 + logo，上电从 Flash 一次搬到片上）
//------------------------------------------------------------------------------
// ★ 2026-10-28：格式从「4bpp 覆盖率 + 调色板」改成「RGB565 直出」，
//   图标和 logo 合并成同一个分区（0x050000，30976 B）一次读完。
//  ★ 2026-11-06：字幕字模成为第 4 个客户（客户 3，优先级最低），
//    由顶层 sub_loader 直接驱动本模块的 sub_fl_* 透传口。
osd_icon_rom #(
    .BASE       (24'h050000),
    .NBYTES     (30976),
    .POR_CYCLES (400_000)
) u_icon (
    .clk      (clk),
    .rst      (rst),
    .fl_req   (ic_req),
    .fl_addr  (ic_addr),
    .fl_len   (ic_len),
    .fl_rdy   (ic_rdy),
    .fl_data  (ic_data),
    .fl_valid (ic_valid),
    .fl_done  (ic_done),
    .ra_addr  (icon_ra),
    .ra_data  (icon_rd),
    // ★ 2026-10-29：位图 ROM 的输出寄存器也要跟其它流水级一起冻结
    .ce       (pixel_ready),
    // ★ 原来这里是悬空的 `.ready()`，于是上电约 9ms 内（POR 8ms + 搬运）
    //   位图是未初始化 BRAM，而且 Flash 未编程时也照样画 ⇒ 纯色色块/白块。
    //   现在接出来做双重判据，见 osd_icon_rom.v 顶部说明。
    .ready    (icon_ready)
);

//------------------------------------------------------------------------------
//  6. Flash 仲裁 + 控制器
//------------------------------------------------------------------------------
wire        fl_req;
wire [23:0] fl_addr;
wire [15:0] fl_len;
wire        fl_ready;
wire [7:0]  fl_data;
wire        fl_valid;
wire        fl_done;

flash_arb u_arb (
    .clk    (clk),
    .rst    (rst),
    // 客户 0：字库
    .c0_req (fc_req), .c0_addr(fc_addr), .c0_len(fc_len), .c0_rdy(fc_rdy),
    .c0_data(fc_data), .c0_valid(fc_valid), .c0_done(fc_done),
    // 客户 1：图标
    .c1_req (ic_req), .c1_addr(ic_addr), .c1_len(ic_len), .c1_rdy(ic_rdy),
    .c1_data(ic_data), .c1_valid(ic_valid), .c1_done(ic_done),
    // 客户 2：提示音（透传）
    .c2_req (bp_req), .c2_addr(bp_addr), .c2_len(bp_len), .c2_rdy(bp_rdy),
    .c2_data(bp_data), .c2_valid(bp_valid), .c2_done(bp_done),
    // 客户 3：字幕字模（透传，★2026-11-06）
    .c3_req (sub_fl_req), .c3_addr(sub_fl_addr), .c3_len(sub_fl_len),
    .c3_rdy (sub_fl_rdy), .c3_data(sub_fl_data), .c3_valid(sub_fl_valid),
    .c3_done(sub_fl_done),
    // 控制器
    .fl_req  (fl_req),
    .fl_addr (fl_addr),
    .fl_len  (fl_len),
    .fl_ready(fl_ready),
    .fl_data (fl_data),
    .fl_valid(fl_valid),
    .fl_done (fl_done)
);

spi_flash_read #(
    .CLK_FREQ_HZ (50000000),
    .SCLK_DIV    (1),
    .POR_CYCLES  (200000)
) u_flash (
    .clk          (clk),
    .rst_n        (~rst),
    .rd_req       (fl_req),
    .rd_addr      (fl_addr),
    .rd_len       (fl_len),
    .rd_ready     (fl_ready),
    .rd_data      (fl_data),
    .rd_valid     (fl_valid),
    .rd_done      (fl_done),
    .flash_cs_n   (flash_cs_n),
    .flash_sclk   (flash_sclk),
    .flash_mosi   (flash_mosi),
    .flash_miso   (flash_miso),
    .flash_wp_n   (flash_wp_n),
    .flash_hold_n (flash_hold_n)
);

//------------------------------------------------------------------------------
//  6.5 ★ 2026-10-30 缩放十字条（独立旁路，不走 osd_menu 控件表）
//------------------------------------------------------------------------------
//  【为什么不塞进控件表】osd_menu 关掉时会把所有矩形槽清零（见 u_gfx 的
//    `menu_on` 注释），缩放十字条不属于菜单，塞进去会被一起清掉；
//    而且加一个槽要动 osd_cellmem 的位宽，远贵于这里两条比较。
//  【为什么要打 2 拍】osd_gfx / ps_gen_text 都是「命中寄存器 + 像素寄存器」
//    两级流水，输出相对 x/y 晚 2 拍。这里用 x/y 算命中后必须再打两拍才能与
//    m_pix 对齐，否则十字线整体右/下偏 2 像素（而且 50MHz 域 WNS 已 −4.6ns，
//    把 16 位比较直接串进 m_pix 组合路径也是找事）。
wire chit_raw = zoom_on & ((x == {6'd0, ax}) | (y == {6'd0, ay}));
reg  chit_d0, chit_d1;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        chit_d0 <= 1'b0;
        chit_d1 <= 1'b0;
    end else if (pixel_ready) begin
        chit_d0 <= chit_raw;
        chit_d1 <= chit_d0;
    end
end
wire chit_v = chit_d1;



// 十字条颜色 = 当前功能（红/绿/青），A=192 走 75% 混合
wire [31:0] zcol = (zoom_mode == 2'd2) ? 32'hC0_60_D0FF :   // 青：缩放
                   (zoom_mode == 2'd1) ? 32'hC0_80_FF80 :   // 绿：上下
                                         32'hC0_FF_8080;    // 红：左右

//------------------------------------------------------------------------------
//  7. 合并：文字盖在图形上（1 拍）
//------------------------------------------------------------------------------
// ★ 2026-10-29 诊断结论：位图不是原因。
//   上板实测「游标框最右 1px + 图标/logo 中间稍右一列」出现白/蓝竖向虚线，
//   用 `define TXT_OFF 1` 把文字整层屏蔽后**虚线依旧** ⇒ 与文字层无关。
//   （该开关已验证无效，已移除；如需复查可临时把下一行换成 `m_pix <= gfx_pix;`）
//   ⇒ 问题在图形层：所有受影响的都是**命中网络里被更高索引槽压住**的位置。
//      `osd_cellmem` 的命中是"最高索引独占"（见 osd_gfx 注释），
//      高索引槽一旦选中，低索引槽就画不了；若高索引槽在那个像素上
//      `w_on=0`（不在自己的 in_rect / 被 cut 掉），就会出现一个**洞**。
// ★★ 2026-11-01 深夜【仿真专用】把未知位清成 0 ★★
//  【为什么】文字层的字形存储在仿真里可能从未被写过（真实硬件由 flash 预置），
//    于是 txt_pix 的 RGB 是 X；而合并级用 `txt_pix[31:24] != 0` 做选择子，
//    条件是 X ⇒ 整个 m_pix 变成 X ⇒ 出图全黑，什么都看不到。
//    这里只在仿真里把"含 X 的像素"当透明，让图形层能正常出图；
//    综合时这一段整块不存在，硬件行为一位不变。
`ifdef IVERILOG_SIM
wire txt_unk = (^txt_pix === 1'bx);
wire [31:0] txt_pix_s = txt_unk ? 32'h0000_0000 : txt_pix;
`else
wire [31:0] txt_pix_s = txt_pix;
`endif

reg [31:0] m_pix;

always @(posedge clk or posedge rst) begin
    if (rst) m_pix <= 32'h0000_0000;
    else if (pixel_ready)
        // ★ 2026-10-30：十字条颜色 = 当前功能（没有这个，用户不知道按键会作用到哪）
        //     红 = 左右平移    绿 = 上下平移    青 = 缩放
        // ★ 2026-10-30：十字条 / 倍数数字优先级最高（A=192 → viewer_scene 走 75% 混合）
        // ★★ 缩放模式的菜单残留怎么治 ★★
        //   【上板现象】打开缩放十字时，整张 OSD 菜单也一起冒出来。
        //   【根因】`osd_layer_en` 加了 `zoom_on` 之后 osd_gfx 的写使能打开，
        //     而 osd_menu 在 menu_on=0 时只在**布局那一拍**清零；S_ZOOM 是连续
        //     出帧、退出菜单时又是停帧，于是残留矩形每帧都被画出来。
        //   【修法】不在这里粗暴屏蔽（那样连缩放面板也一起没了，踩过），
        //     而是让 osd_menu 的 `!menu_on && zoom_on` 分支把槽 16 改成缩放面板、
        //     其余槽全部清零，并且 zoom_on 自己触发布局（见 osd_menu 的
        //     `L_IDLE` 条件与 `zoom_rise/zoom_chg`）。
        //     ⇒ 清零一定执行，残留自然消失；缩放面板和文字照常输出。
        m_pix <= chit_v                            ? zcol :
                 (txt_pix_s[31:24] != 8'd0)        ? txt_pix_s : gfx_pix;
end

assign osd_pix = m_pix;

endmodule
