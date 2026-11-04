//==============================================================================
//  App.v —— 图片查看器应用层顶层
//==============================================================================
//
//  【它在整条链路里的位置】
//
//      ┌──────────┐  ps_new_frame   ┌───────────────┐
//      │          │ ──────────────► │               │
//      │   App    │  ps_data        │ frame_writer  │
//      │          │  ps_valid       │               │
//      │          │ ◄────────────── │               │
//      └──────────┘   ps_start      └───────────────┘
//
//  App 沿用像素源接口，并额外输出 pipe_ce，让 frame_writer 与像素流水线
//  在背压时同步停顿。
//
//  【内部结构】
//
//      viewer_ctrl ──(模式/几何/alpha/格式)──► viewer_scene ──► ps_data
//           │                                     ▲
//           │ 装载请求                            │ 纹理请求
//           ▼                                     ▼
//      sd_bmp_loader（在顶层）              sdr_port_sched（在顶层）
//
//  【帧节奏】
//      565（过渡/低清）：每 FRAME_PERIOD_CYCLES 请求一帧 → 60fps
//      888（高清静止）：只请求一帧，画完就停；显示端一直读同一块缓冲
//
//  【为什么高清只画一帧】
//  静态画面重复合成纯属浪费带宽（一帧 888 要写 307,200 字 + 读同样多的纹理）。
//  停帧之后 frame_writer 停在 IDLE、display_valid 保持 1，屏幕内容不变。
//
//==============================================================================

module App #(
    parameter integer IMG_W = 640,
    parameter integer IMG_H = 480,
    // ★ 过渡总帧数按时长分档（2026-10-26，用户指定）：
    //   T_END   = 180 帧 ≈ 3.0s → fx3/fx4（缩小交换，编排长）
    //   T_END_F =  60 帧 ≈ 1.0s → fx0/fx1/fx2（平移/淡入，动作简单）
    //   2026-10-25：**3600 → 180**（16.7ms × 180 = **3.0s**，用户指定）
    //   `t` 12 位（≤4095），180 远不溢出。
    //   ⚠ 过渡期间按键被忽略（原设计），播完自动结束。
    parameter integer T_END = 180,
    parameter integer T_END_F = 60,
    // 过渡前预热帧数（透传给 viewer_ctrl；仿真可设 0）
    parameter integer ARM_FRAMES = 3,
    // ★ 场景配额（透传给 viewer_scene）。上板必须用 48/96 才不为难写口；
    //   仿真里传 127 可以解除限速，帧时间从 819200 拍降到 307200 拍（约 2.6×）。
    parameter integer SCENE_QUOTA = 48,
    parameter integer QUOTA_565   = 96,
    // 565 模式下一帧的目标周期（50MHz）：835000 拍 ≈ 16.7ms ≈ 60fps
    // ★★ 2026-10-24：835000(16.7ms/60fps) → 925000(18.5ms/~54fps) ★★
    //  【依据】sdr_port_sched.v 的时间账：
    //         收像素 6.1 + 等写 8.23（不可压缩）+ 纹理 4.18 = 18.5ms > 16.7
    //  【后果】原来每帧都在欠债 ⇒ 请求侧没有喘息窗口
    //         ⇒ 像素 FIFO 永远填不满 ⇒ 每行到同一个 x 就断供
    //         ⇒ 帧内停顿 + 竖线。
    //  【现在】帧间留出 ~1.5ms 空闲（925000 等于需要时间、零余量，等于没改） ⇒ **请求侧可以提前灌满 FIFO**
    //         ⇒ 帧内断供大幅减少 ⇒ 掉帧与竖线都应缓解。
    //  【代价】动画目标帧率从 60 降到 ~54fps（动画略慢，但不再卡顿）。
    //  【想回到 60fps】只能把场景时钟提到 100MHz（需给 SD 接口另配 50MHz）。
    // ★ 2026-10-24：20ms → **30ms**（当时为防撕裂，正确性优先）。
    // ★★ 2026-10-25（第二步）：18ms → **16.7ms**（≈60fps，与显示场同步）。
    //   【依据】上板确认 18ms 下**无撕裂** ⇒ 真实帧时 ≤ 18ms，还可以再收。
    //   16.7ms 正好是一帧显示场（60Hz），再往下没有意义（不会更快）。
    //   【若出现撕裂】退回 900000（18ms/55fps）——那说明真实帧时在 16.7~18ms 之间。
    //   【若仍是 55fps 不涨】说明限制已经不是帧期，而是帧内停顿
    //     （wr_stall 写口反压 / 纹理缺料）⇒ 下一步调 QUOTA_565。
    parameter integer FRAME_PERIOD_CYCLES = 835000,    // 16.7ms @50MHz ≈ 60fps

    // ★ 2026-11-01：IMG0/1/2_OFFSET 与 IMGx_LENGTH **已删除**。
    //   相册的三张图现在由 FAT 目录决定（album_src → I_img_off0/1/2），
    //   换卡不用改代码——这正是"相册走 FAT"要解决的问题。

    // ---- SDRAM 布局 ----
    parameter [20:0] THUMB0_ADDR = 21'd614400,
    parameter [20:0] THUMB1_ADDR = 21'd652800,
    parameter [20:0] THUMB2_ADDR = 21'd691200,
    parameter [20:0] HD_ADDR     = 21'd729600
)(
    input  wire        clk,             // 50MHz（sd_card_clk）
    input  wire        I_quad_mode,     // 1 = 文件页四宫格（运行时切换）
    input  wire [3:0]  I_quad_vm,       // 本页哪几格有内容（与 I_quad_mode 同拍采样）

    // ---- 文件页状态 ----
    output wire [2:0]  O_page,
    output wire        O_file_page,     // 1 = 真的停在文件页（菜单开着 && page==2）

    // ---- 相册三张图的卡内字节偏移（来自 album_src 的窗口）----
    input  wire [31:0] I_img_off0,
    input  wire [31:0] I_img_off1,
    input  wire [31:0] I_img_off2,
    input  wire [31:0] I_img_off3,       // ★ next2（滑动窗口预取）
    output wire [15:0] O_alb_base,      // 窗口起点（第几张图）→ album_src
    input  wire        I_alb_win4,      // 窗口第 4 项也是图 ⇒ 后面还有
    // ★ 2026-11-04：`I_alb_have[1:0]` → 1 位 `I_alb_any`（本窗口至少有一张图）。
    //   "还有没有下一张"不再由窗口张数推 —— 见 img_switch.v 头注释。
    input  wire        I_alb_any,
    input  wire [2:0]  I_alb_nimg,      // ★ [NIMG] 本窗口实际图数（0..4）
    input  wire        I_alb_ready,     // ★ 相册重扫完成（base 变了要等它）

    // ---- 文件页文件名读口 ----
    output wire [1:0]  O_nm_slot,
    output wire [3:0]  O_nm_idx,
    input  wire [7:0]  I_nm_char,
    input  wire [3:0]  I_nm_blen,       // 当前格主名长度（OSD 居中/插点用）
    input  wire [3:0]  I_nm_elen,       // 当前格扩展名长度
    input  wire [19:0] I_nm_len4,       // 4 格的显示字符数（每格 5 位）
    input  wire        I_nm_vld,        // 当前格有没有内容

    // ---- 文件页光标（2026-11-01 P5）----
    output wire [1:0]  O_fp_cur,
    output wire        O_fp_flip_nx,
    output wire        O_fp_flip_pv,
    input  wire [2:0]  I_fp_vcnt,
    input  wire        I_fp_has_more,
    input  wire        I_fp_gen,
    // ★ 2026-11-04：文件页"打开"（KEY2）—— 顶层按光标格的类型分派：
    //   BMP ⇒ I_jump(I_jump_idx = 目录里第几张图) 回给本模块；WAV ⇒ 换曲；TXT ⇒ 不动
    input  wire        I_fp_has_prev,
    output wire        O_fp_open,
    input  wire        I_jump,
    input  wire [7:0]  I_jump_idx,
    output wire        O_ld_abort,      // 单拍：叫 bmp_loader 放弃当前高清装载（主页连切）
    input  wire        rst,             // 高有效
    input  wire        mclk,            // 100MHz（ext_mem_clk）：图片层仲裁侧

    // ---- 像素源接口 ----
    input  wire        ps_start,
    output reg         ps_new_frame,     // ★ 过程赋值 ⇒ 必须是 reg（Gowin HDL-8007）
    output wire [23:0] ps_data,
    output wire        ps_valid,
    output wire        pipe_ce,
    input  wire        busy,

    // ---- 图片层 ↔ SDRAM 读仲裁 ----
    output wire [1:0]  tex_req_vld,
    output wire [41:0] tex_req_addr,
    output wire [7:0]  tex_req_mask,
    input  wire [1:0]  tex_req_ack,
    input  wire [1:0]  tex_pix_wr,
    input  wire [63:0] tex_pix_data,

    // ---- 按键（单拍脉冲）----
    input  wire        key_prev,
    input  wire        key_next,
    input  wire        key_fx,
    input  wire        key_menu,        // KEY1：OSD 菜单开/关
    input  wire [3:0]  key_level,       // 消抖电平（1=松开），长按判定用

    // ---- 用户 SPI Flash（字库，OSD 文字用）----
    output wire        flash_cs_n,
    output wire        flash_sclk,
    output wire        flash_mosi,
    output wire        flash_wp_n,
    output wire        flash_hold_n,
    input  wire        flash_miso,

    // ---- SD 装载器 ----
    output wire        ld_req,
    output wire        ld_fmt,
    output wire [31:0] ld_off,
    output wire [31:0] ld_len,
    output wire [20:0] ld_base,
    input  wire        ld_can_load,
    input  wire        ld_done,
    input  wire        ld_err,
    input  wire        ld_sd_ready,     // ★ SD 初始化完成（没它就发单会被静默丢弃）
    input  wire [15:0] ld_src_w,        // ★ 2026-10-27：装载器解析出的源分辨率
    input  wire [15:0] ld_src_h,

    // ---- 音乐播完（audio_wav_path，翻转电平）----
    input  wire        music_done,

    // ---- 提示音的 Flash 客户口（透传到 osd_top 里的 flash_arb）----
    input  wire        bp_req,
    input  wire [23:0] bp_addr,
    input  wire [15:0] bp_len,
    output wire        bp_rdy,
    output wire [7:0]  bp_data,
    output wire        bp_valid,
    output wire        bp_done,
    // ★ 2026-11-06 字幕字模的 Flash 客户口（透传到 osd_top 的 flash_arb 客户 3）
    input  wire        sub_fl_req,
    input  wire [23:0] sub_fl_addr,
    input  wire [15:0] sub_fl_len,
    output wire        sub_fl_rdy,
    output wire [7:0]  sub_fl_data,
    output wire        sub_fl_valid,
    output wire        sub_fl_done,
    // ★ 2026-11-05 缩放烘焙快照：viewer_ctrl 在发单那拍锁存"本次高清装载的倍率/锚点"。
    //   原来装载器直接吃 oz_zoom 的**实时值**，而新架构的发单要排队（ldp/alb_ok），
    //   中间跨过档位变化就会把**别的档位**烘焙进 HD 画布 ⇒ 预览收敛值与最终清晰图不一致。
    output wire [7:0]  ld_zoom,         // Q4.4（缩略图装载恒 1x）
    output wire [9:0]  ld_ax, ld_ay,    // 屏幕锚点

    // ---- 帧缓存格式（给 frame_read_write）----
    output wire        fmt565,

    // ---- 写口背压 ----
    input  wire        wr_stall,

    // ---- 状态观测（数码管：只保留 帧率 / 图片号 / 模式 三项）----
    output wire [1:0]  st_img,
    output wire [2:0]  st_fx,      // ★ 2026-10-26：1→2 位（新增 fx4，取值 0..4）
    output wire [3:0]  fps_dbg,     // ★ 帧率/10（给数码管最左位）
    // ---- 调试读数（数码管第 5/6 位）----
    output wire [3:0]  dbg_state,   // viewer_ctrl 状态码（5=HD 8=OSD）
    output wire        dbg_osd,     // osd_on
    output wire        dbg_fmt,     // fmt565
    // （st_state / t_dbg / blk_dbg / duty_dbg / dbg_delta / dbg_evt /
    //   smp_bus / smp_win：2026-10-26 随数码管清理删除）

    // ---- 画质参数（2026-10-26 新增）----
    //  OSD 显示页 → 顶层 video_postproc。sd_card_clk 域慢变电平，
    //  跨域同步 + vs 边沿生效都在 postproc 内部完成。
    output wire [6:0]  pic_bright,   // 亮度 0..100（50=默认）
    output wire [6:0]  pic_contrast, // 对比度
    output wire [6:0]  pic_temp,     // 色温 0=冷 100=暖
    output wire [6:0]  pic_sat,      // 饱和度
    output wire [1:0]  pic_style,    // 风格 0无 1黑白 2反色
    output wire        pic_tgl,      // 参数更新翻转标志
    // ★ 2026-11-03：ALS 生效亮度（顶层 pic_bright_eff）→ OSD 显示页亮度行
    input  wire [6:0]  bright_eff,

    // ---- 系统页导出（2026-10-29）→ 顶层音频链路 ----
    output wire        o_lang,       // 0=中文 1=English
    output wire [6:0]  o_vol,        // 音量 0..100
    output wire        o_mute,       // 静音
    output wire        o_loop,       // 循环播放
    output wire        o_spc,        // 频谱条（预留）
    // ★ 2026-11-02【音随图切】相册页 item6 的 CheckBox（1 = 切图时自动找同名 WAV）
    output wire        o_asy,
    // ★ 2026-11-03 自动亮度：显示页 item6 的 CheckBox（1 = 亮度跟随环境光，顶层 als_bright 用）
    output wire        o_auto,
    // ★ 2026-11-02【音随图切】本次高清装载是相册窗口第几格
    //   （给顶层查 lister 的 name_mem，见 viewer_ctrl 的 O_hd_rel 注释）
    output wire [1:0]  O_hd_rel,
    // ★ [MM-EARLY]：换图后一拍的"提前扫 match"请求 + 目标槽号（顶层用它把
    //   音乐/字幕的定位扫描提前到过渡动画里，见 viewer_ctrl 的端口注释）
    output wire        O_mm_arm,
    output wire [1:0]  O_mm_rel,
    // ★ 2026-11-02【信息条文件名】相册重扫完成单拍（顶层 = lst_done_alb）。
    //   OSD 只用它触发一次文字表重写；名字的新鲜度由"关闭文件页 ⇒ 重扫一次"
    //   保证，本模块不做任何判断、也不参与装载。
    input  wire        I_nm_upd,
    // ★ [NM-HOLD]：名字读口被 match 比较器借走（顶层 = lst_busy & own==MM）
    input  wire        I_nm_hold,

    // ---- 缩放 OSD（2026-10-30）→ 顶层 bmp_loader 的几何 ----
    output wire        oz_on,        // 缩放 OSD 打开（顶层也用来判 565/888）
    output wire [7:0]  oz_zoom,      // Q4.4
    output wire [9:0]  oz_ax,        // 锚点 = 屏幕上的十字中心
    output wire [9:0]  oz_ay
);

//==============================================================================
//  0. 控制层 ↔ 场景层 的连线
//==============================================================================
wire        compose_en;
wire        a_hd, a_en, b_en;
// ★ [ZOOM-SEQ] "888 高清画布真的在显示"：上一次缩放的切回要等帧边界才提交，
//   那几帧里画面还是 565 预览 ⇒ zoom_osd 必须据此压住下一次缩放，否则跳变。
// ★ 2026-11-02 [ZOOM-EARLY]：缩放现在也能从 S_LOW 进（那时 a_hd=0、画面就是
//   565 预览、高清还在装）。若仍按 `a_hd & ~fmt565` 判，缩放键会被 [ZOOM-SEQ]
//   一直压住 ⇒ OSD 出来了但按不动。所以补一项"缩放 OSD 自己就在 565 预览里"。
//   ⚠ 必须带 `& oz_on`：不然 S_LOW/S_TRANS 里（a_hd=0、fmt565=1、缩放没开）
//     也会被当成"已稳定"，虽然眼下没有消费者，但语义就错了。
wire        z_hd_shown = a_hd | (fmt565 & oz_on);
wire [20:0] a_base, b_base;
wire [31:0] pos_a, size_a, pos_b, size_b;
wire [39:0] k_a, k_b;
wire [19:0] ori_a_w;                // ★ [ZOOM-EXACT] A 层锚点 {ORIX[9:0], ORIY[9:0]}
wire [7:0]  alpha_b;
wire        pixel_ready;

wire [15:0] scan_x, scan_y;
wire        osd_enter, osd_exit, osd_on, osd_allow, osd_layer_en;
// ★ [OSD-EARLY] 缩放 OSD 的"允许"与菜单分开：菜单在 S_LOW 也允许开，
//   缩放不行（那时 a_hd=0，画布是缩略图，k/锚点会算错）。
wire        zoom_allow;
wire [7:0]  osd_fade;
wire [31:0] osd_pix;

// ---- 相册 / 自动切换 / 信息条（2026-10-27）----
wire [2:0]  fx_sel, auto_mode;
wire        info_on, album_prev, album_next, osd_trig, info_show;
wire [15:0] res_w, res_h;
wire [5:0]  auto_remain;
wire [8:0]  auto_bar;
wire [7:0]  ld_zoom_w;      // ★ 本次高清装载的倍率快照（hd_take 锁存）
wire [9:0]  ld_ax_w, ld_ay_w;
// ★ 2026-11-02【信息条】viewer_ctrl → osd_top：当前图槽号 / 换图单拍。
//   两根都必须声明在 u_ctrl 之前（例化处就要连），0 LUT。
wire [1:0]  cur_rel_w;
wire        sw_go_w;

// 缩放 OSD 的两个请求（声明必须在 viewer_ctrl 例化之前 —— iverilog 不允许先用后声明）
wire        z_preview;      // 单拍：zoom 变了 → 切 565 预览
wire [1:0]  z_mode;         // 0=左右 1=上下 2=缩放（给 osd_top 染色）
wire        z_load;         // 单拍：停手 500ms → 踢重装

//==============================================================================
//  ★★ [ZOOM-RESTORE] 缩放 >1.0 时"开菜单 / 翻页"先动画恢复 1.0× ★★
//==============================================================================
//  【为什么】zoom 是**持久状态**：退出缩放 OSD 后画面仍是放大的。
//    但菜单排版/翻页过渡都是按 1:1 满屏算的，在放大态上直接执行会错位。
//  【做法】把这三个键先"吞掉"（z_swallow），交给 zoom_osd 暂存；
//    zoom_osd 用 565 预览把 zoom 动画降到 1×、重装完成、回到静止态后
//    用 act_go/act_sel 把键**补放**出来。所以对外看到的时序是
//    "先动画回 1× → 再开菜单/翻页"，而键通路本身不需要任何额外状态。
//  【成本】2 位行为码 = 两级 mux；掩蔽 3 个键 + 补放 3 个 OR，约 20 LUT。
wire        z_need_rst = ~oz_on & (oz_zoom != 8'h10);      // 已退出 OSD 且仍在放大
// 把"开菜单/翻页"压成一个 2 位行为码：1=上一页 2=下一页 3=打开 OSD
wire [1:0]  z_act_i    = key_menu ? 2'd3 :
                         key_next ? 2'd2 :
                         key_prev ? 2'd1 : 2'd0;
wire        z_swallow  = z_need_rst & (z_act_i != 2'd0);
wire        k_menu_m   = key_menu & ~z_swallow;
wire        k_prev_m   = key_prev & ~z_swallow;
wire        k_next_m   = key_next & ~z_swallow;
wire        z_act_go;
wire [1:0]  z_act_sel;
wire        z_rest;             // ★ 静默恢复中 ⇒ 合成级屏蔽 OSD 层
wire        k_menu_f   = k_menu_m | (z_act_go & (z_act_sel == 2'd3));
wire        k_prev_f   = k_prev_m | (z_act_go & (z_act_sel == 2'd1));
wire        k_next_f   = k_next_m | (z_act_go & (z_act_sel == 2'd2));

//==============================================================================
//  1. 控制层
//==============================================================================
viewer_ctrl #(
    .T_END       (T_END),
    .T_END_F     (T_END_F),
    .ARM_FRAMES  (ARM_FRAMES),
    .THUMB0_ADDR (THUMB0_ADDR),
    .THUMB1_ADDR (THUMB1_ADDR),
    .THUMB2_ADDR (THUMB2_ADDR),
    .HD_ADDR     (HD_ADDR),
    // ★ 过渡进度按时间步进用（t 每 16.7ms 走一步，与帧率无关）
    .FRAME_CYC   (FRAME_PERIOD_CYCLES)
) u_ctrl (
    .clk        (clk),
    .rst        (rst),
    .I_img_off0 (I_img_off0),
    .I_img_off1 (I_img_off1),
    .I_img_off2 (I_img_off2),
    .I_img_off3 (I_img_off3),
    .O_base     (O_alb_base),       // 窗口起点 → album_src
    .I_win4     (I_alb_win4),       // 后面还有（末张置灰）
    .I_any      (I_alb_any),        // ★ 本窗口至少有一张图（只用于空目录保护）
    .I_nimg     (I_alb_nimg),       // ★ [NIMG] 本窗口图数
    .I_alb_ready(I_alb_ready),      // ★ 新窗口的偏移已就绪
    .I_jump     (I_jump),
    .I_jump_idx (I_jump_idx),
    .I_file_page(O_file_page),      // ★ [FP-ABORT] 文件页一开就放弃在飞的装载
    .O_ld_abort (O_ld_abort),

    .key_prev   (k_prev_f),
    .key_next   (k_next_f),
    .key_fx     (key_fx),
    .key_osd_enter(osd_enter),
    .key_osd_exit (osd_exit),
    .osd_fade   (osd_fade),
    .osd_on     (osd_on),
    .osd_allow  (osd_allow),
    .zoom_allow (zoom_allow),

    .busy       (busy),
    .ps_start   (ps_start),
    .pipe_ce    (pixel_ready),

    .ld_req     (ld_req),
    .ld_fmt     (ld_fmt),
    .ld_off     (ld_off),
    .ld_len     (ld_len),
    .ld_base    (ld_base),
    .ld_can_load(ld_can_load),
    .ld_done    (ld_done),
    .ld_err     (ld_err),
    .ld_sd_ready(ld_sd_ready),

    .compose_en (compose_en),
    .fmt565     (fmt565),
    .a_hd       (a_hd),
    .a_en       (a_en),
    .a_base     (a_base),
    .b_en       (b_en),
    .b_base     (b_base),

    .pos_a      (pos_a),  .size_a (size_a), .k_a (k_a), .ori_a (ori_a_w),
    .pos_b      (pos_b),  .size_b (size_b), .k_b (k_b),
    .alpha_b    (alpha_b),

    .st_state   (dbg_state),
    .st_img     (st_img),
    .st_fx      (st_fx),
    .fps_dbg    (fps_dbg),

    // 相册 / 自动切换 / 信息条
    .fx_sel      (fx_sel),
    .auto_mode   (auto_mode),
    .info_on     (info_on),
    .album_prev  (album_prev),
    .album_next  (album_next),
    .osd_trig    (osd_trig),
    .music_done  (music_done),
    .ld_src_w    (ld_src_w),
    .ld_src_h    (ld_src_h),
    .res_w       (res_w),
    .res_h       (res_h),
    .auto_remain (auto_remain),
    .auto_bar    (auto_bar),
    .info_show   (info_show),
    .ld_zoom     (ld_zoom_w),
    .ld_ax       (ld_ax_w),
    .ld_ay       (ld_ay_w),
    // ★ 2026-11-02【音随图切】本次高清装载的窗口格号（顶层查图片名用）
    .O_hd_rel    (O_hd_rel),
    // ★ 2026-11-02【信息条】当前图槽号 + 换图单拍（给 osd_top 用）
    .O_cur_rel   (cur_rel_w),
    .O_sw_go     (sw_go_w),
    // ★ [MM-EARLY]：换图后一拍的提前 match 请求（顶层用来把音乐/字幕提前）
    .O_mm_arm    (O_mm_arm),
    .O_mm_rel    (O_mm_rel),

    // ---- 缩放 OSD（2026-10-30）----
    .zoom_on     (oz_on),
    .preview_req (z_preview),
    .load_req    (z_load),
    .zoom_val    (oz_zoom),
    .za_x        (oz_ax),           // ★ [ZOOM-ANCHOR] 预览锚点 = 十字中心
    .za_y        (oz_ay)
    // （st_state / t_dbg / duty_dbg：2026-10-26 随数码管清理删除）
);

//==============================================================================
//  1.5 OSD 叠加层（菜单：控件表图形 + 16×16 点阵文字）
//==============================================================================
// ★ 2026-11-01：osd_top 的输出先在**例化之前**声明。
//   原来声明在例化之后（449 行），TD 能过，但 iverilog 报
//   "Net page_w is not defined in this context" ⇒ 任何 App 级仿真都编不过。
//   这是纯声明位置调整，没有逻辑变化。
wire [2:0] page_w;
wire [1:0] nm_slot_w;
wire [3:0] nm_idx_w;

osd_top #(.NL(8), .MAXLEN(32), .NSLOT(8), .NSLOT_F(63)) u_osd (   // [S6] 8 个动态字格（[LUT-2] 弹层第 3 行借选中行控件槽）
    .clk        (clk),
    .rst        (rst),
    .ps_start   (ps_start),
    .pixel_ready(pixel_ready),
    .x          (scan_x),
    .y          (scan_y),
    .key_press  (oz_on ? 4'b0 : {k_prev_f, k_next_f, key_fx, k_menu_f}),
    .key_level  (key_level),
    .menu_active(osd_on),
    .osd_allow  (osd_allow),
    .info_show  (info_show),
    // 缩放 OSD（2026-10-30）
    .zoom_on    (oz_on),
    .ax         (oz_ax),
    .ay         (oz_ay),
    .zoom_sel   (oz_zoom),
    .zoom_mode  (z_mode),
    .flash_cs_n (flash_cs_n),
    .flash_sclk (flash_sclk),
    .flash_mosi (flash_mosi),
    .flash_wp_n (flash_wp_n),
    .flash_hold_n(flash_hold_n),
    .flash_miso (flash_miso),
    .bp_req     (bp_req),
    .bp_addr    (bp_addr),
    .bp_len     (bp_len),
    .bp_rdy     (bp_rdy),
    .bp_data    (bp_data),
    .bp_valid   (bp_valid),
    .bp_done    (bp_done),
    // ★ 2026-11-06 字幕字模（flash_arb 客户 3）
    .sub_fl_req (sub_fl_req),
    .sub_fl_addr(sub_fl_addr),
    .sub_fl_len (sub_fl_len),
    .sub_fl_rdy (sub_fl_rdy),
    .sub_fl_data(sub_fl_data),
    .sub_fl_valid(sub_fl_valid),
    .sub_fl_done(sub_fl_done),
    .enter_req  (osd_enter),
    .exit_req   (osd_exit),
    .osd_layer_en(osd_layer_en),
    .fade       (osd_fade),
    .osd_pix    (osd_pix),
    // 画质参数透传（2026-10-26）
    .pic_bright  (pic_bright),
    .pic_contrast(pic_contrast),
    .pic_temp    (pic_temp),
    .pic_sat     (pic_sat),
    .pic_style   (pic_style),
    .pic_tgl     (pic_tgl),
    .bright_eff  (bright_eff),
    // 相册 / 信息条
    .fx_sel      (fx_sel),
    .auto_mode   (auto_mode),
    .info_on     (info_on),
    .album_prev  (album_prev),
    .album_next  (album_next),
    .cur_w       (res_w),
    .cur_h       (res_h),
    .auto_remain (auto_remain),
    .auto_bar    (auto_bar),
    .txt_trig    (osd_trig),
    // ★ 2026-11-02 信息条文件名：当前图槽号 / 换图单拍 / 重扫完成单拍
    .cur_rel     (cur_rel_w),
    .img_go      (sw_go_w),
    .nm_upd      (I_nm_upd),
    .nm_hold     (I_nm_hold),
    // 系统页导出（2026-10-29）
    .o_lang      (o_lang),
    .o_vol       (o_vol),
    .o_mute      (o_mute),
    .o_loop      (o_loop),
    .o_spc       (o_spc),
    .o_asy       (o_asy),           // ★ 2026-11-02 相册"音随图切"开关
    .o_auto      (o_auto),          // ★ 2026-11-03 显示页"自动亮度"开关

    // ---- 文件页文件名读口（透传给 fat_dir_lister 的名字 RAM）----
    .O_page      (page_w),
    .O_nm_slot   (nm_slot_w),
    .O_nm_idx    (nm_idx_w),
    .I_nm_char   (I_nm_char),
    .I_nm_blen   (I_nm_blen),
    .I_nm_elen   (I_nm_elen),
    .I_nm_len4   (I_nm_len4),
    .I_nm_vld    (I_nm_vld),
    .O_fp_cur     (O_fp_cur),
    .O_fp_flip_nx (O_fp_flip_nx),
    .O_fp_flip_pv (O_fp_flip_pv),
    .I_fp_vcnt    (I_fp_vcnt),
    .I_fp_has_more(I_fp_has_more),
    .I_fp_gen     (I_fp_gen),
    .I_fp_has_prev(I_fp_has_prev),
    .O_fp_open    (O_fp_open)
);

// osd_top 的输出经内部 wire 转成 App 的输出端口（output 不能直接并 output）
// （page_w / nm_slot_w / nm_idx_w 已上移到 osd_top 例化之前声明）
assign O_page    = page_w;
// ★ 必须与 osd_menu 内部的 fp_mode 同义：`page` 是粘滞的，关掉菜单后仍是 2。
//   顶层若只看 page==2，装载器归属就永远不交还 ⇒ 相册切图卡死（实测）。
assign O_file_page = osd_on & (page_w == 3'd2);
assign O_nm_slot = nm_slot_w;
assign O_nm_idx  = nm_idx_w;

//==============================================================================
//  1.2 缩放 OSD（2026-10-30）
//==============================================================================
//  【为什么它只输出 zoom / ax / ay 三个量】
//    cam 不在这里算 —— bmp_view 内部用 `cam = W/2 + (ax-320)*(step_fit-step)`
//    直接推出来，所以本模块没有任何几何状态，也不跨装载保存东西。
zoom_osd #(.CLK_HZ(50_000_000), .DB_MS(500)) u_zoom (
    .clk        (clk),
    .rst        (rst),
    // 按键总线：{KEY4, KEY3, KEY2, KEY1}
    .key_press  ({key_prev, key_next, key_fx, key_menu}),
    .menu_on    (osd_on),           // 菜单开着 ⇒ 不许进缩放
    .allow      (zoom_allow),       // ★ [OSD-EARLY] 只在静止态（不含 S_LOW）
    .ld_done    (ld_done),
    .hd_shown   (z_hd_shown),       // ★ [ZOOM-SEQ] 888 已真正显示才允许下一次缩放
    .act_code   (z_act_i),          // ★ [ZOOM-RESTORE] 1=上一页 2=下一页 3=打开OSD
    .zoom_on    (oz_on),
    .zoom       (oz_zoom),
    .ax         (oz_ax),
    .ay         (oz_ay),
    .mode       (z_mode),           // 给 osd_top 染色
    .refresh    (),                 // S_ZOOM 连续出帧 ⇒ 不需要补帧请求
    .preview_req(z_preview),
    .load_req   (z_load),
    .act_go     (z_act_go),         // ★ [ZOOM-RESTORE] 恢复完成，放行暂存动作
    .act_sel    (z_act_sel),
    .restore    (z_rest)            // ★ 静默恢复：合成级屏蔽 OSD 层
);

//==============================================================================
//  2. 场景层
//==============================================================================
viewer_scene #(
    .IMG_W (IMG_W), .IMG_H (IMG_H),
    .SCENE_QUOTA (SCENE_QUOTA), .QUOTA_565 (QUOTA_565)
) u_scene (
    .clk          (clk),
    .rst          (rst),
    .mclk         (mclk),

    .ps_start     (ps_start),
    .pixel_ready  (pixel_ready),
    // （blk_dbg / duty_dbg / dbg_delta / dbg_evt / smp_bus / smp_win：
    //   诊断读数，2026-10-26 随数码管清理一起删除）
    .ps_data      (ps_data),
    .pipe_ce      (pipe_ce),

    .fmt565       (fmt565),
    .wr_stall     (wr_stall),

    .a_hd         (a_hd),
    .a_pos        (pos_a),
    .a_size       (size_a),
    .a_k          (k_a),
    .a_ori        (ori_a_w),        // ★ [ZOOM-EXACT] 锚点（不是相位）
    .a_base       (a_base),
    .a_en         (a_en),

    // ---- 文件页四宫格 ----
    //   槽基址是 viewer_scene 的 parameter（QUAD_BASE = 1048576），
    //   因为 quad_sel/ps_gen_image 里的槽偏移要常量折叠，不能接 port。
    .quad_mode    (I_quad_mode),
    .quad_vm      (I_quad_vm),

    .b_pos        (pos_b),
    .b_size       (size_b),
    .b_k          (k_b),
    .b_base       (b_base),
    .b_en         (b_en),
    .alpha_b      (alpha_b),

    .scan_x       (scan_x),
    .scan_y       (scan_y),
    .osd_on       (osd_layer_en & ~z_rest),   // ★ [ZOOM-RESTORE] 静默恢复时不出 OSD
    .osd_fade     (osd_fade),
    .osd_pix      (osd_pix),

    .tex_req_vld  (tex_req_vld),
    .tex_req_addr (tex_req_addr),
    .tex_req_mask (tex_req_mask),
    .tex_req_ack  (tex_req_ack),
    .tex_pix_wr   (tex_pix_wr),
    .tex_pix_data (tex_pix_data)
);

//==============================================================================
//  3. ps_valid：未延迟的取数节拍
//==============================================================================
assign ps_valid = ps_start & pixel_ready;

// ★ 2026-11-05：装载器的烘焙几何改用"本次请求的快照"，不再用 oz_zoom 的实时值
assign ld_zoom = ld_zoom_w;
assign ld_ax   = ld_ax_w;
assign ld_ay   = ld_ay_w;
assign dbg_osd = osd_on;
assign dbg_fmt = fmt565;

//==============================================================================
//  4. 帧请求：限速自由运行 + compose_en 门控
//==============================================================================
//  【为什么限速】565 一帧只要 8.2ms（写口产能），不限速会跑到 100fps+，
//  把 SDRAM 读口打满。835000 拍 = 16.7ms，正好 60fps。
//
//  【compose_en 上升沿】立刻请求一帧，不等周期到点 —— 切换高清时不必多等 16ms。
//  【compose_en=0】不再请求；已经提交的帧继续显示（display_valid 保持 1）。
//==============================================================================
reg        busy_d;
reg        pending;
reg [31:0] tick_cnt;
reg        compose_d;

wire compose_rise = compose_en & ~compose_d;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        busy_d       <= 1'b0;
        pending      <= 1'b0;
        tick_cnt     <= 32'd0;
        compose_d    <= 1'b0;
        ps_new_frame <= 1'b0;
    end else begin
        busy_d       <= busy;
        compose_d    <= compose_en;
        ps_new_frame <= 1'b0;

        if (tick_cnt < FRAME_PERIOD_CYCLES)
            tick_cnt <= tick_cnt + 32'd1;

        // 帧画完 → 挂起"要下一帧"
        if (busy_d & ~busy)
            pending <= 1'b1;

        // compose_en 刚拉高：立刻发一帧
        if (compose_rise) begin
            pending  <= 1'b1;
            tick_cnt <= FRAME_PERIOD_CYCLES;
        end

        // 周期到点 + 挂起 + 允许产帧 → 发请求
        if (compose_en && pending && (tick_cnt >= FRAME_PERIOD_CYCLES)) begin
            ps_new_frame <= 1'b1;
            tick_cnt     <= 32'd0;
            pending      <= 1'b0;
        end

        // 不允许产帧：清掉挂起，避免下次一开门就抢跑
        if (!compose_en)
            pending <= 1'b0;
    end
end

endmodule
