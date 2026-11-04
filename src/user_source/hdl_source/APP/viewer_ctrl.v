//==============================================================================
//  viewer_ctrl.v —— 图片查看器的"大脑"
//==============================================================================
//
//  【职责】（2026-11-04 起换图决策/缓存在 img_switch，本模块只管"怎么显示"）
//      · 主页换图播过渡（S_ARM → S_TRANS），菜单里换图不播（帧边界直接换源）
//      · 装载序列器：img_switch 的缩略图预取 + 本模块的高清（当前图），高清优先
//      · 帧格式：过渡/低清 = RGB565（60fps），高清静止 = RGB888（只合成一帧）
//      · 所有场景参数只在帧边界更新（viewer_fx 的 upd）
//
//  【状态机】
//
//      BOOT_T ──当前图缩略图好了──► LOW ──高清好了──► HDF ──一帧画完──► HD
//        ▲  (开机 / 文件页打开)      ▲ │                                │
//        │                           │ └── 再按 key3/key4：放弃高清 ──┐  │ key3 / key4
//        │                           │                               ▼  ▼
//        │                           └───────────── TRANS ◄──────── ARM
//      OSD ──文件页打开 BMP──────────►（BOOT_T）
//
//  【缓存】3 块缩略图是环：当前图 + 上一张 + 下一张；每次换图补装 1 张，
//    补装的那块此刻不在屏幕上 ⇒ 与过渡并行。见 img_switch.v。
//
//  【按键】主页：HD / STUCK / LOW（装高清中也收，直接打断）；过渡中忽略。
//  装载失败 ⇒ STUCK：保持低清、恢复按键，绝不把界面锁死。
//
//  【帧边界】ps_start 下降沿 + PIPE 拍排空 + 2 拍保护 = tick。
//  t 更新后**下一拍**才锁存 fx 参数（fx 采样的是寄存器）。
//
//==============================================================================

module viewer_ctrl #(
    // ★ 过渡总帧数按时长分档（2026-10-26，用户指定）：
    //   T_END   = 180 帧 ≈ 3.0s → fx3/fx4（缩小交换，编排长）
    //   T_END_F =  60 帧 ≈ 1.0s → fx0/fx1/fx2（平移/淡入，动作简单）
    // 过渡何时结束由当前效果决定（见 §3.4 的 t_end_cur）。
    parameter integer T_END = 300,
    parameter integer T_END_F = 60,
    // ★ 2026-10-28：切 888 之前，额外等几个帧边界让**几何落定**。
    //   viewer_fx 为了时序把进度量打了拍（q_f_r → q2_r → e_r 共 2 拍），
    //   于是"t 到终点"的那一拍，screen 上的几何还是 2 帧前的（实测 pos_x=3）。
    //   原来在这里立刻切 ⇒ 变清晰那一瞬整幅右移 3~8px（用户报的"切换瞬间横移"）。
    //   等 SETTLE_TICKS 个 tick（4×16.7ms ≈ 67ms）后几何已经精确到终点，再切。
    parameter integer SETTLE_TICKS = 4,
    // 过渡前预热多少帧（0 = 不等帧边界，只在第一个 tick 切）
    parameter integer ARM_FRAMES = 3,
    // ★ 2026-11-02：S_ONE / S_ONE_W 等"单张缩略图装载完成"的上限（拍）。
    //   0.67s @50MHz。超时就放弃这次换图、回到能按键的状态 ——
    //   原来是**完全没有兜底**，装载器一不给就永久停在 S_ONE（用户报的
    //   "切完缩略图不播动画、按键全死"）。
    //   抽成参数是为了让仿真能把 3300 万拍压到 2000 拍跑得动。
    parameter integer ONE_TO = 33_554_431,
    // ★ 一个"60fps 帧"等于多少拍（50MHz 下 835000 ≈ 16.7ms）。
    //   只用于把过渡进度从"按帧数"改成"按时间"，见 §3.4。
    parameter integer FRAME_CYC = 835000,
    // ★ 2026-10-27：自动切换"1 秒"等于多少拍。
    //   抽成参数的唯一目的是**让仿真能把 3s 压缩到几十微秒** ——
    //   原来写死 26'd49_999_999，跑一次 3s 档要 1.5 亿拍，根本没法回归。
    //   硬件上保持 50_000_000（= 50MHz 下 1.000s）。
    parameter integer AUTO_SEC_CYC = 50_000_000,
    // ★ 2026-11-01：三张图的位置不再是参数 —— 运行时由 album_src（读 FAT 目录）
    //   从端口给进来。参数是"这块卡"的绝对字节偏移，换卡/重新格式化就废了；
    //   而且上板时没人会去算它。长度同理删掉：bmp_loader 的 file_length 是
    //   死输入（内部 flen 只写不读，见 fat_dir_lister 的同名注释），
    //   这里直接给 0，省 3×32 位参数和 3:1 mux。
    parameter [20:0] THUMB0_ADDR = 21'd614400,
    parameter [20:0] THUMB1_ADDR = 21'd652800,
    parameter [20:0] THUMB2_ADDR = 21'd691200,
    parameter [20:0] HD_ADDR     = 21'd729600
)(
    input  wire        clk,
    input  wire        rst,

    // ---- 三张图在卡上的字节偏移（来自 album_src，随 FAT 目录变化）----
    input  wire [31:0] I_img_off0,
    input  wire [31:0] I_img_off1,
    input  wire [31:0] I_img_off2,
    input  wire [31:0] I_img_off3,       // ★ 第 4 个偏移 = next2（滑动窗口预取）
    // ★ 滑动窗口（PLAN §7B.3）：O_base = 窗口起点（目录里第几张图），
    //   I_win4 = 窗口第 4 项也是图 ⇒ "后面还有"，用于末张置灰
    output wire [15:0] O_base,
    input  wire        I_win4,
    // ★ 2026-11-04：`I_have[1:0]`（窗口里有几张）→ 1 位 `I_any`（至少有一张）。
    //   它只用于"空目录"判定（S_BOOT_T）；"还有没有下一张"已改由 img_switch 的
    //   tv/dv（装到了 / 试过没有）决定 —— 用窗口张数推那个判断是错的（见 img_switch 头注释）。
    input  wire        I_any,
    // ★ [NIMG]：本窗口实际图数（0..4）。只用来判"窗口第 cr+1 格有没有图"，
    //   不参与装载 —— 偏移发布和预取路径逐位不变（上一版动偏移导致换图/缩放全废）。
    input  wire [2:0]  I_nimg,
    input  wire        I_alb_ready,
    // ★ 2026-11-04：OSD 文件页按 KEY2 打开一张 BMP ⇒ 直接跳到目录里第 I_jump_idx 张图
    //   （单拍；只在菜单运行相位里收，收下后关菜单、不播动画）
    input  wire        I_jump,
    input  wire [7:0]  I_jump_idx,
    // ★ 2026-11-02 [FP-ABORT]：1 = OSD 正停在文件页（= App 的 O_file_page）。
    //   用途只有三条，全是"让文件页立刻可用"：
    //     ① 文件页一开就把**本模块在飞的装载放弃掉**（bmp_loader 的 abort）。
    //        原来四宫格要等相册这次装载自然收尾（大图 1~4s）才拿得到装载器，
    //        期间 4 个格子一格都不出 ⇒ "进文件页像卡住"。
    //     ② 文件页开着时不再发新单（hd_take/tl_take 都门掉），否则放弃 →
    //        重发 → 再放弃地打转。
    //     ③ 被放弃的那次缩略图装载要按"没发生"处理（见 img_switch 的 I_tl_abort），
    //        否则会把那个方向判成"没有图"。
    input  wire        I_file_page,

    // ---- 按键（单拍脉冲）----
    input  wire        key_prev,        // key4
    input  wire        key_next,        // key3
    input  wire        key_fx,          // key2
    // ---- OSD 菜单（来自 osd_top）----
    input  wire        key_osd_enter,   // 菜单关着时按 KEY1 → 开菜单
    input  wire        key_osd_exit,    // 菜单里**在导航栏**按 KEY1 → 关菜单（长按已取消）
    input  wire [7:0]  osd_fade,        // 菜单整层不透明度
    output reg         osd_on,          // 1 = S_OSD（菜单开启）
    output wire        osd_allow,       // 1 = 允许开菜单
    // ★ 2026-11-02 [OSD-EARLY]：菜单的"允许"与缩放的"允许"必须分开。
    //   osd_allow 现在多含 S_LOW（装高清中也能开菜单 —— 那 4s 里按 KEY1 原来
    //   完全没反应），但缩放 OSD 不能跟着放宽：S_LOW 里 a_hd=0（画布是缩略图），
    //   进去之后 k/锚点全按 888 算 ⇒ 画面错位。所以缩放仍只认静止态。
    output wire        zoom_allow,      // 1 = 允许进缩放 OSD（只在静止态）

    // ---- 缩放 OSD（来自 zoom_osd）★ 2026-10-30 ----
    input  wire        zoom_on,         // 缩放 OSD 打开（十字条显示）
    input  wire        preview_req,     // 单拍：zoom 变了，切 565 预览
    input  wire        load_req,        // 单拍：停手够久，踢一次重装
    input  wire [7:0]  zoom_val,        // Q4.4：缩放目标（算预览几何 k 用）
    input  wire [9:0]  za_x, za_y,      // ★ [ZOOM-ANCHOR] 缩放锚点 = 屏幕上的十字中心

    // ---- 帧写控制器 ----
    input  wire        busy,            // 正在画一帧
    input  wire        ps_start,
    input  wire        pipe_ce,

    // ---- 装载器 ----
    output reg         ld_req,
    output reg         ld_fmt,          // 0 = 320×240 RGB565；1 = 640×480 RGB888
    output reg  [31:0] ld_off,
    output reg  [31:0] ld_len,
    output reg  [20:0] ld_base,
    input  wire        ld_can_load,
    input  wire        ld_done,
    input  wire        ld_err,
    input  wire        ld_sd_ready,     // ★ SD 初始化完成（没它就发单会被静默丢掉）
    output wire        O_ld_abort,      // ★ abort：**电平**，保持到本次装载收尾（见 abort_req）
    // ★ 2026-11-02【音随图切】本次高清装载是相册窗口第几格（0..3）。
    //   和 ld_off 是**同一拍**锁存的（见下面 ldp==0 的 `ld_off <= img_off(cur_rel)`），
    //   所以拿它去查 lister 的 name_mem 就是"正在装的这张图"的名字。
    //   ⚠ 不要用 cur_rel 代替：装载途中 cur_rel 可能已经走到下一张。
    output wire [1:0]  O_hd_rel,

    // ★ 2026-11-02【信息条】当前图在相册窗口里的槽号 + "换图成立"单拍。
    //   · O_cur_rel → 信息条读 name_mem 的槽号（= 正在显示的那张图的名字）；
    //   · O_sw_go   → OSD 强制重写一次文字（两张图分辨率相同时 src_w 不变，
    //     光靠 info_chg 不会翻，名字就会停在上一张）。
    //   都不参与装载，纯观测口。
    output wire [1:0]  O_cur_rel,
    output wire        O_sw_go,

    // ★ [MM-EARLY]：换图后**一拍**的"请马上做一次 match（WAV/TXT 同名扫描）"请求。
    //   为什么要有它：音乐和字幕都靠这次扫描定位（顶层 mm_pend/mm_go 那一段）。
    //   原来这次扫描是在**高清装载发起**时才武装的 ⇒ 只能排在烘焙后面，
    //   音乐/字幕总比图片晚到。改成换图当拍就武装之后，扫描落在过渡动画
    //   （1~3 s）里跑完 ⇒ 图片出现时字幕和音乐已经就位，且**不延后图片**
    //   （动画本来就要等，扫描是搭它的车）。
    //   为什么要延一拍：`sw_go` 那一拍 `cur_rel` 还是旧值（img_switch 同拍才更新 cr），
    //   延一拍拿到的才是新图的窗口位置 = name_mem 槽号。
    output wire        O_mm_arm,
    output wire [1:0]  O_mm_rel,

    // ---- 场景控制 ----
    output reg         compose_en,   // 1 = 请求下一帧
    output reg  [3:0]  fps_dbg,      // ★ 诊断：当前帧率/10（6=60fps，3=30fps），给数码管最左位
    // （t_dbg / duty_in / duty_dbg：2026-10-26 随数码管清理删除）
    output reg         fmt565,          // 1 = 帧缓存 565
    output reg         a_hd,            // A 层用 hd（888）
    output reg         a_en,
    output reg  [20:0] a_base,
    output reg         b_en,
    output reg  [20:0] b_base,

    // ---- 过渡参数（来自 viewer_fx）----
    output wire [31:0] pos_a, size_a,
    output wire [39:0] k_a,
    output wire [19:0] ori_a,           // ★ [ZOOM-EXACT] {ORIX[9:0], ORIY[9:0]} 锚点（源坐标原点）
    output wire [31:0] pos_b, size_b,
    output wire [39:0] k_b,
    output wire [7:0]  alpha_b,

    // ---- 状态观测（数码管）----
    output wire [1:0]  st_img,
    // ★ 2026-10-26：1→2 位（新增 fx4）。原来是 2 位 ⇒ fx_id=4(3'b100) 被截成
    //   0，数码管的"当前模式"位在 fx4 时显示 0（就是用户报的那个现象）。
    output wire [2:0]  st_fx,
    output wire [3:0]  st_state,

    //--------------------------------------------------------------------------
    //  相册 / 自动切换 / 信息条（2026-10-27 新增）
    //--------------------------------------------------------------------------
    input  wire [2:0]  fx_sel,          // 菜单里选的切换效果
    input  wire [2:0]  auto_mode,       // 0关 1:3s 2:5s 3:10s 4:30s 5音乐结束
    input  wire        info_on,         // 信息条开关
    input  wire        album_prev,      // 菜单"上一张"单拍
    input  wire        album_next,      // 菜单"下一张"单拍
    input  wire        osd_trig,        // 文字已更新（信息条重合成用）
    input  wire        music_done,      // WAV 播完一拍（50MHz 域，已同步）
    input  wire [15:0] ld_src_w,        // 装载器解析出的源分辨率
    input  wire [15:0] ld_src_h,
    output wire [15:0] res_w,           // 当前图分辨率 → 信息条
    output wire [15:0] res_h,
    output wire [5:0]  auto_remain,     // 剩余秒数
    output wire [8:0]  auto_bar,        // 进度条像素宽
    output wire        info_show,       // 信息条此刻可以显示
    // ★ 2026-11-05 [LD-SNAP]：本次装载的烘焙几何快照（在 hd_take 那一拍锁存）
    //   【为什么必须快照】高清画布是 loader 用 `zoom/ax/ay` **烘焙**出来的
    //   （bmp_loader 在 start 那拍锁进 zoom_r/ax_r/ay_r），而 `zk` 只负责 565 预览的插值。
    //   新架构的发单要排队（ldp==0 && alb_ok），这段时间里 oz_zoom 完全可能已经变了
    //   ⇒ loader 用**别的档位**烘焙 ⇒ 预览收敛值与最终清晰图不一致（再叠下一轮越走越歪）。
    //   输出 = 缩略图装载恒 1x/屏幕中心，高清装载 = 发单那拍锁的档位与锚点。
    output wire [7:0]  ld_zoom,
    output wire [9:0]  ld_ax,
    output wire [9:0]  ld_ay
);

//==============================================================================
//  0. 常量与寄存器
//==============================================================================
localparam [3:0] S_BOOT_T = 4'd0;   // 开机 / 文件页打开图片：等当前图缩略图装好再上屏
localparam [3:0] S_BOOT_H = 4'd1;   // （已并入 S_LOW，编码保留给数码管/测试台）
localparam [3:0] S_LOW    = 4'd2;
localparam [3:0] S_TRANS  = 4'd3;
localparam [3:0] S_HDF    = 4'd4;
localparam [3:0] S_HD     = 4'd5;
localparam [3:0] S_STUCK  = 4'd6;
localparam [3:0] S_ARM    = 4'd7;   // ★ 过渡前的"备战"：预热 B 层 + 等帧边界
localparam [3:0] S_OSD    = 4'd8;   // ★ OSD 菜单：565 连续合成（静止高清之上）
localparam [3:0] S_ZOOM   = 4'd9;   // ★ 缩放 OSD：888/565 连续出帧 + 停手重装
// ★ 2026-11-04：S_ONE(10) / S_ONE_W(11) 删除 —— 滑窗补装改成后台预取（img_switch），
//   换图不再等补装，主页/菜单都不需要专门的"补装 1 张"状态。

// ★ 2026-10-30 [DSP-PIPE]：混合器加了一级输入寄存器 ⇒ 场景流水线 4→5。
//   `tick` 是"帧尾排空 + 2 拍保护"的窗口，**必须**跟流水线深度同步。
localparam integer PIPE = 5;

(* syn_encoding = "sequential" *) reg [3:0] state;
reg [1:0] osd_ph;                   // S_OSD 相位：0 等排空 / 1 运行 / 2 退出
// ★ 2026-11-01 S3b：退出请求**粘滞**（KEY1 是单拍脉冲，落在相位 0/2 会丢）
reg       exit_pend;
reg       jmp_x;                    // ★ 2026-11-04：退出菜单是因为文件页"打开"了一张图
// ★ 2026-10-30：S_ZOOM 相位：0 = 888 高清连续出帧；1 = 565 缩略图预览；2 = 装载中
reg [1:0] zph;
reg pv_pend, ld_pend;               // 缩放的预览/重装请求锁存（按键单拍，帧边界消费）
// ★★ [ZOOM-SWAP]：装载已完，等 SWT 个帧边界让写落盘，再在帧边界一次性提交
reg       sw_ok;
reg [1:0] sw_wait;
localparam [1:0] SWT = 2'd2;
// ★ hd_valid 由装载序列器唯一驱动；状态机只能发一拍 hd_kill 让它去清
reg       hd_kill;
// ★ 菜单内换图（不播动画）：目标已定，等帧边界把 a_base 换过去
reg        osd_pv;
// ★ 2026-10-24：12 位(≤4095)，T_END=3000 也不回绕
reg [11:0] t;
// fx_id：当前过渡效果；dir：1=下一张（画面向左/上滚），0=上一张
reg [2:0] fx_id;
reg       dir;

// ★ 2026-11-04：装载请求改成"电平 + 序列器择机收下"。
//   hd_want：状态机要一次**当前图**的高清（偏移/目标地址由序列器按当前窗口现取）。
//   缩略图由 img_switch 自己提（O_tl_req），高清优先。
reg        hd_want;
reg        ab_req;                  // 单拍：换图成立那一拍发出（**任何状态**，见 §2 公共区）
// ★ 2026-11-02 [ZOOM-RESTEP]：缩放中途换档 ⇒ 请求序列器放弃在飞的那次烘焙。
//   ⚠ 只能打这根内部事件，**不能直接写 abort_req / O_ld_abort** —— 那两个归 §2 的
//   序列器 always 块独占（两处写就是多驱动，TD 会报错、仿真取哪个块看运气）。
reg        zoom_abort;
// ★ [FP-ABORT]：一次"本模块主动放弃装载"的**单拍**，送给 img_switch 的 I_tl_abort
//   （缓存要知道"这次不算数"，既不置 tv 也不置 dv）。
//   与 `abort_req` 的分工：abort_req 是打给**装载器**的电平（只对已发单的那次有效）；
//   ab_pulse 是打给**缓存**的单拍（还包含"还没发单就作废"的 ldp==1 情况）。
reg        ab_pulse;
// ★ [ABORT-LATCH]：`O_ld_abort` 不再是一拍脉冲，而是"锁存到本次装载收尾"的电平。
//   为什么必须锁：装载器只在 `abort && busy` 那一拍锁存 ab_r（bmp_loader:344-348），
//   而 `ab_req` 是单拍、发送又只发生在 ldp==2 —— 脉冲只要错开一拍（或撞上归属切换）
//   就**永久丢失**；丢了以后旧装载会正常收尾（err=0），把旧画布重新验证成有效。
//   现在：只要 ldp==2 期间出现过任何放弃事件，就保持拉高，直到 ldp 自己退出。
reg        abort_req;
// ★ [TGT-STAMP]：HD 画布的"本次请求目标"戳。
//   旧判据只有 `hd_img == cur_rel`，而**换图时 cur_rel 通常不变**（img_switch 滑窗时
//   cr 恒为 1），于是"上一张装完"也能满足它 ⇒ 旧画布被当成新图接受（画面停在旧图）。
//   本戳：hd_take 时置 1（本单就是给当前图的），**换图（sw_go）**时清 0；
//   `hd_valid` 与 `hd_ready` 都必须带它。它才是"这次结果属于谁"的唯一凭据。
//   （不要挂 hd_kill：那是单拍，缩放换档时会与 hd_take 撞拍，把刚发的单判成过期。）
reg        ld_tgt_ok;
// 放弃条件：主页切图（ab_req）或 OSD 进了文件页（I_file_page，电平）。
// 文件页用**电平**：只要它还开着就一直作废（含"刚放弃完又冒出来的那一单"），
// 否则会出现 放弃 → 重发 → 再放弃 的抖动，反而更慢。
wire       ld_drop = ab_req | I_file_page | zoom_abort;
// ★ 2026-11-05 [LD-SNAP]：发单那一刻的烘焙几何（loader 在 start 拍锁存的就是这三个）
reg  [7:0]  hd_zoom_s;
reg  [9:0]  hd_ax_s, hd_ay_s;
// ★ 2026-11-05 [HDW-TO]：`hd_want` 的排队兜底。它只在 `hd_take` 时清零，而
//   `hd_take = (ldp==0) && hd_want && alb_ok` —— 相册一直不 ready 就永远收不下，
//   于是 S_ZOOM 卡在 zph=2（一直是模糊 565），且 `!hd_want` 把预取也一起挡住。
//   24 位 @50MHz ≈ 335ms，和 [ZOOM-NOWRIP] 的 sw_hard 同一量级。
reg  [23:0] hdw_cnt;
wire        hdw_to = (hdw_cnt == 24'hFFFFFF);

// 装载序列器（独占写这几个）
(* syn_encoding = "sequential" *) reg  [1:0] ldp;
reg        ld_fin;
reg        ld_fail;
reg        ld_fail_st;      // ★ 粘滞：本次**高清**装载失败过
reg        ld_fmt_r;        // 本次在装的格式（收下时锁存）
reg        hd_valid;        // 高清槽内容有效
// ★ 2026-11-04 还原 77c748c：高清槽的"内容标签"——装的是窗口第几格（rel）。
//   hd_ready = hd_valid && (hd_img == cur_rel)，换图后 cur_rel 一变就天然不成立，
//   不需要额外的 hd_stale（那次装载装完也认不出当前图）。
reg  [1:0] hd_img;
// ★ 2026-11-02【音随图切】导出"本次高清装的是窗口第几格"给顶层查图片名。
//   纯连线，0 LUT（2 根 FF 输出）。
assign O_hd_rel = hd_img;
// ★ 2026-11-02 [NO-LD-TO]：**装载完全取消超时**。
//   原来缩略图 0.335s、高清 5.37s 就到时放弃；现在 `ldp` 只在装载器真的回
//   `ld_done` / `ld_err` 时才退出。
//   【为什么撤】缩略图那一单要逐行读源图（320×240 输出、240 行，每行一次寻道
//   + 读满源行宽），640 宽的源就已经 ≈0.34s，正好压在 0.335s 线上；更大的源必然超。
//   而超时**不中止**装载器：图照样被写进 SDRAM（所以画面看着是好的），但
//   `ld_fail=1` ⇒ `I_tl_ok=0`；同时 `ld_to_f=1` ⇒ `I_tl_err=0` ⇒ img_switch 既
//   不置 `tv` 也不置 `dv` ⇒ `tv[0]` 永远为 0 ⇒ `nc` 无限重试 THUMB0，邻居永远
//   轮不到 ⇒ 上一张/下一张按键全部无反应，而且**没有任何错误音**。
//   ⇒ 超时在这里不但没用，还是"缩略图永远装不上"的直接原因。
//   【代价】若装载器真的永不返回（卡被拔掉等），`ldp` 会停住、后续装载发不出去；
//   兜底交给 S_LOW 的**状态级**时间看门狗（low_stall1/2，纯时间，不依赖 ld_fail_st）。
reg  [2:0] arm_cnt;         // ★ S_ARM 里数预热帧数
reg  [1:0] hdf_ph;          // ★ S_HDF 三相：0 等旧帧排空 / 1 等新帧开工 / 2 等新帧提交
reg  [1:0] arm_ph;          // ★ S_ARM 相位：0 切源 1 预热 B 2 等一帧
reg        low_retry;       // ★ S_LOW 已重试过一次装载

// ★ [HDW-TO] 只数"请求已发出、装载口也空闲、但一直收不下"的时间；
//   若正有一次装载在跑（ldp!=0），那是正常的排队，计时清零。
always @(posedge clk or posedge rst) begin
    if (rst)                                   hdw_cnt <= 24'd0;
    else if (!hd_want || (ldp != 2'd0))        hdw_cnt <= 24'd0;
    else if (hdw_cnt != 24'hFFFFFF)            hdw_cnt <= hdw_cnt + 24'd1;
end

// ★★★ 2026-10-27：3 个看门狗共用一个计数器（区间互斥，见 §3.5）★★★
reg [28:0] wd_cnt;

// ★ 2026-11-02 [NO-LD-TO]：装载超时已整体删除（见 §1 末尾的说明）。
//   原来这里有一个 28 位 `ld_to_cnt` + `LD_TO_HD` 比较器（缩略图 2^24 ≈ 0.335s、
//   高清 2^28−1 ≈ 5.37s）。删掉后省 28 FF + 1 比较器，`ld_fail` 现在等价于
//   `ld_err`（"失败"只可能是装载器真报错），img_switch 的 `dv` 语义也随之变干净。

wire [1:0] cur_rel;                     // 当前图在相册窗口里的位置（img_switch 给）
// ★ 2026-11-02：`sw_go` 必须在 §1.5（自动切换计时）之前声明 —— 那里要用它
//   做"换图当拍清零"。iverilog 对"先用后声明"是硬错。
wire       sw_go;                       // 本拍换图成立（img_switch 的 O_go）
// ★ 2026-11-02 [ZOOM-MATCH]：画布有效还要求"是**当前档位**烘焙的"。
//   zoom 是持久状态（关掉缩放 OSD 后画面仍是放大的），所以 `hd_img` 对得上
//   不代表画面对得上：比如 2× 烘焙完、用户把档位缩回 1×，那块画布仍是 2× 的
//   **裁切**，按 1:1 满屏显示 = 整屏放大错位。
//   加上 `hd_zoom_s == zoom_val` 之后，凡是"档位变了但画布没重装"的地方
//   （S_LOW 的升 888、[ZOOM-SWAP] 的提交）都会自动判成"没准备好"，
//   走重装而不是把错的画面推上去。
wire hd_ready = hd_valid && (hd_img == cur_rel) && (hd_zoom_s == zoom_val) && ld_tgt_ok;

// ★ 2026-11-02 [ZOOM-EXIT-FAST]：**马上就会"对"** —— 已有一单 HD 在跑/在排，
//   而且它装出来正是当前图 + 当前档位 ⇒ 不用再排一单（重排会重复烘焙，
//   等第一单结束、画面刚切上 888，第二单就开始往正在显示的那块写 = 撕裂）。
wire hd_pend  = ld_tgt_ok && (ldp != 2'd0) && ld_fmt_r && (hd_img == cur_rel) && (hd_zoom_s == zoom_val);

// ★ [ABORT-LATCH]：abort 输出 = 锁存后的电平（不再是单拍）
assign O_ld_abort = abort_req;

// ★ 2026-11-02【信息条】两个观测/触发口（见端口注释），纯连线 0 LUT。
assign O_cur_rel = cur_rel;
assign O_sw_go   = sw_go;

// ★ [MM-EARLY]：换图后一拍的"提前扫 match"请求（见端口注释）。
//   `cur_rel` 此时已经是新图的位置，所以直接连出去，不再打第二级寄存器。
reg  sw_go_d;
always @(posedge clk or posedge rst) begin
    if (rst) sw_go_d <= 1'b0;
    else     sw_go_d <= sw_go;
end
assign O_mm_arm = sw_go_d;
assign O_mm_rel = cur_rel;


// ★ 2026-11-05 [LD-SNAP]：装载器的烘焙几何
//   高清：用发单那拍锁的快照（`ld_fmt_r` 在收下时锁存，到 loader 的 start 拍已经是本次的值）
//   缩略图：恒 1x + 屏幕中心（缩略图本来就是 1:1 装 320×240，不该被缩放档位影响）
assign ld_zoom = ld_fmt_r ? hd_zoom_s               : 8'h10;
assign ld_ax   = ld_fmt_r ? hd_ax_s                 : 10'd320;
assign ld_ay   = ld_fmt_r ? hd_ay_s                 : 10'd240;

//==============================================================================
//  1. 图片资源选择：相册窗口第 rel 张的偏移（rel 由 img_switch 给）
//==============================================================================
function [31:0] img_off;
    input [1:0] i;
    begin
        case (i)
            2'd0:    img_off = I_img_off0;
            2'd1:    img_off = I_img_off1;
            default: img_off = I_img_off2;
        endcase
    end
endfunction


//==============================================================================
//  1.5 ★ 2026-10-27：自动切换计时 + 信息条倒计时 / 进度条
//==============================================================================
//  【规则（用户口径）】
//    · 只在"静止高清 + 菜单关闭 + 模式 1..4"时计时；进菜单/过渡/装载 ⇒ 清零暂停，
//      退出菜单后从 0 重新开始（`!auto_run` 分支每拍重装初值，天然实现"重来"）。
//    · 剩余秒数与进度条**只在整秒时递减**，不做运行期除法：
//        bar 每秒减一个常数（≈287/total），末秒夹到 0，单调且与秒数同步。
//    · 模式 5（音乐结束）不计数；每遍播完的单拍事件会锁存到 mus_pend，
//      到静止高清再点火一次（详见下面 §1.5 末尾的锁存逻辑）。
reg [25:0] auto_cnt;                // 1s 内计数（50MHz）
reg [5:0]  auto_sec;                // 已过秒数
reg [5:0]  auto_rem;                // 剩余秒数
reg [8:0]  auto_barp;               // 进度条像素（0..287）

// ★★★ 2026-10-27：不要把 integer 型 parameter 直接位选 ★★★
//   `AUTO_SEC_CYC[25:0]` 这种写法在 TD 里可能被算成 **0**
//   （本工程 osd_top 的 PIPE、osd_icon_rom 的 POR_CYCLES 都踩过）。
//   一旦算成 0：AUTO_LAST = 0-1 = 0x3FF_FFFF ⇒ auto_tick 永不成立
//   ⇒ 自动切换彻底失效（而且仿真里一切正常，只有在板子上才发现）。
//   正确做法：先装进一个**定宽 localparam**，再拿它参与运算。
localparam [25:0] AUTO_SEC_W = AUTO_SEC_CYC;
localparam [25:0] AUTO_LAST  = AUTO_SEC_W - 26'd1;

wire [5:0] auto_total = (auto_mode == 3'd1) ? 6'd3 :
                        (auto_mode == 3'd2) ? 6'd5 :
                        (auto_mode == 3'd3) ? 6'd10 : 6'd30;
wire [8:0] auto_step  = (auto_mode == 3'd1) ? 9'd96 :
                        (auto_mode == 3'd2) ? 9'd57 :
                        (auto_mode == 3'd3) ? 9'd29 : 9'd10;

// ★★★ 2026-10-27 修复：S_HDF 必须计入计时窗口 ★★★
//   【上板现象】打开信息条后自动切换失效：设成 3s，倒计时 3→2→3→2 无限循环，
//     永远等不到切图。
//   【为什么】信息条的内容（分辨率/倒计时秒数）每秒都会变 ⇒ `osd_trig` 翻转 ⇒
//     `inf_refresh` 成立 ⇒ 本模块切到 S_HDF 去补画一帧带新文字的 888 画面。
//     而计时窗口原来只认 S_HD/S_STUCK ⇒ 一进 S_HDF 就把 `auto_cnt/auto_sec`
//     全部重装成初值 ⇒ 每秒被清零一次 ⇒ `auto_sec` 永远到不了 auto_total-1。
//     倒计时显示的就是"3 → 2 → (清零) → 3"，与用户描述逐条吻合。
//   【为什么加 S_HDF 是安全的】S_HDF 只持续一帧（~16.7ms），计入后每秒最多
//     损失 16.7ms，3s 档的实际周期 3.02s，肉眼无差别。
//
// ★★★ 2026-11-02【读秒口径：从切图那一拍起算】★★★
//   【用户报的】"读秒要等高清装完才开始"。
//   【根因】上面的窗口只认 S_HD/S_STUCK/S_HDF ⇒ 整个过渡动画（S_ARM/S_TRANS）
//     和装高清（S_LOW）全在窗口外，每拍走 `!auto_run` 分支把 auto_rem 重装成
//     满值。等高清上屏，读数才从满值开始掉 ⇒ 3s 档的真实节奏 =
//     动画(1~3s) + 装载(0.5~4s) + 3s。
//   【现在】把"计时窗口"和"点火窗口"拆开：
//     · 计时：从**切图那一拍**起算（S_ARM/S_TRANS/S_LOW 都算），只在
//       "还没有图（S_BOOT_T）"、"用户正在操作 OSD（S_OSD/S_ZOOM）"处停表；
//     · 点火：仍然只允许在稳定态（S_HD/S_STUCK）——动画中途点火会被 home_ok
//       丢掉，而且会让画面"刚清晰就被切走"。
//   【为什么 auto_sec 必须饱和】点火判据原来是 `auto_tick && auto_sec ==
//     total-1`。计时窗口一放宽，auto_sec 会在动画里越过 total-1 ⇒ 等号永不成立
//     ⇒ 自动切换彻底失效（与上面 S_HDF 那个 bug 同一形态：靠"状态切换时清零"
//     顺手复位，一旦窗口变了就断）。饱和到 total-1 之后，"已经到期"这件事会被
//     记住：一进稳定态，下一个整秒就点火（最多晚 1s），期间倒计时显示 0。
wire auto_stop = (state == S_BOOT_T) || (state == S_OSD) || (state == S_ZOOM);
wire auto_run  = !auto_stop && !osd_on && (auto_mode != 3'd0) && (auto_mode != 3'd5);
wire auto_rdy  = (state == S_HD) || (state == S_STUCK);
wire auto_tick = auto_run && (auto_cnt == AUTO_LAST);
// 剩余秒数是否已经到底（>= ⇒ 动画里到期后再也不会被越过）
wire auto_exp  = (auto_sec >= (auto_total - 6'd1));
wire auto_fire = auto_run && auto_tick && auto_exp && auto_rdy;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        auto_cnt <= 26'd0; auto_sec <= 6'd0;
        auto_rem <= 6'd0;  auto_barp <= 9'd0;
    end else if (!auto_run || sw_go) begin
        // ★ 2026-11-02：`sw_go` 这一条是**必须**的。原来靠在"切图必进 S_ARM
        //   ⇒ auto_run 落下"来顺手清零；现在 S_ARM 也在计时窗口里，不显式清零
        //   的话，自动切换点火之后计数器不会重新开始，第二张之后就再也不切了。
        //   按键换图 / 菜单里换图 / 文件页跳转（都算一次 sw_go）也一并从头计时，
        //   与改造前"每次换图都重来"的行为一致。
        auto_cnt  <= 26'd0;
        auto_sec  <= 6'd0;
        auto_rem  <= auto_total;
        auto_barp <= 9'd287;
    end else if (auto_tick) begin
        auto_cnt  <= 26'd0;
        // 饱和：到期之后再 tick 也不加，否则 `auto_exp` 会被越过
        auto_sec  <= auto_exp ? auto_sec : (auto_sec + 6'd1);
        auto_rem  <= (auto_rem == 6'd0) ? 6'd0 : (auto_rem - 6'd1);
        auto_barp <= (auto_barp < auto_step) ? 9'd0 : (auto_barp - auto_step);
    end else
        auto_cnt <= auto_cnt + 26'd1;
end

// 音乐播完：顶层给的是**翻转电平**，这里打两拍取边沿（同域也防亚稳）
reg [1:0] mus_s;
reg       mus_d;
always @(posedge clk or posedge rst) begin
    if (rst) begin mus_s <= 2'b00; mus_d <= 1'b0; end
    else begin mus_s <= {mus_s[0], music_done}; mus_d <= mus_s[1]; end
end
// ★ 2026-11-08：music_tgl 是**翻转电平**，必须“任意跳变”都算一次事件。
//   旧写法 `mus_s[1] & ~mus_d` 只认上升沿 ⇒ 第 1、3、5… 遍生效，第 2、4… 遍
//   被吞掉（循环播放时正好一半不切，和“大多数时间不生效”完全吻合）。
wire music_edge = mus_s[1] ^ mus_d;

// ★★★ 2026-11-08【音乐结束→自动切图：事件必须锁存】★★★
//  【上板现象】"自动切换=音乐结束 大多数时候不生效"。
//  【根因】旧写法是单拍与：`music_edge && !osd_on && state∈{S_HD,S_STUCK}`。
//    这一拍如果落在 S_OSD（菜单开着）、S_ARM/S_TRANS/S_LOW（过渡/装高清）、
//    S_HDF（信息条每秒补画一帧）或 S_ZOOM，就直接丢了；loop=0 时歌已经播完，
//    不会再有第二次事件 ⇒ 之后永远不切。用户往往正是在菜单里等歌结束。
//  【改法】把事件锁进 mus_pend（跨这些窗口保持），到"能点火"的稳态再放一拍：
//    · music_fire 仍是**单拍**（下一拍 mus_pend 清零），否则 ev_nx 变电平，
//      img_switch 的 go_n 会在一个稳态里连切好几张；
//    · 任何一次 sw_go（手动/菜单/计时模式）都把 pend 消费掉，避免"手动切图
//      + 同一拍音乐结束"连切两张；
//    · 换档离开 5 立即清零，切回来不会补一个陈旧事件。
reg mus_pend;
wire music_fire = mus_pend && (auto_mode == 3'd5) && !osd_on &&
                  ((state == S_HD) || (state == S_STUCK)) &&
                  !zoom_on && !key_osd_enter;

always @(posedge clk or posedge rst) begin
    if (rst)                     mus_pend <= 1'b0;
    else if (auto_mode != 3'd5)  mus_pend <= 1'b0;   // 换档即清
    else if (music_fire)         mus_pend <= 1'b0;   // 已经点火一次
    else if (sw_go)              mus_pend <= 1'b0;   // 别的路径已切图 ⇒ 本次消费掉
    else if (music_edge)         mus_pend <= 1'b1;   // 收下“这一遍播完”
end

wire music_trig = music_fire;
wire auto_go    = auto_fire | music_trig;

assign auto_remain = ((auto_mode >= 3'd1) && (auto_mode <= 3'd4)) ? auto_rem  : 6'd0;
assign auto_bar    = ((auto_mode >= 3'd1) && (auto_mode <= 3'd4)) ? auto_barp : 9'd0;

//------------------------------------------------------------------------------
//  信息条可见性
//------------------------------------------------------------------------------
//  ★★★ 2026-10-27：原来的判据只列了"静止态"，切换动画一开始信息条就消失 ★★★
//  【上板现象】打开信息条后按切换键：S_ARM / S_TRANS / S_LOW 全程看不到信息条，
//    动画结束回到 S_HD 才重新出现（用户报的"切换动画进行时信息条会隐藏"）。
//  【根因】这里只允许 S_HD / S_STUCK / S_HDF 三个状态。
//  【为什么可以放开】OSD 不再"整体替换 B 层"了 —— viewer_scene 改成了
//    逐像素择优（见那里的 pix_b_eff）：A==0 的 OSD 像素仍然走 B 层，
//    所以信息条叠在动画上不会吃掉动画。
//    （2026-11-05：半透明面板的"底"由 s2_b 选 —— B 落屏就混 B，
//      不再在 pix_b_eff 里预混，避免面板被混两次而变暗。）菜单仍然只允许在静止态打开
//    （osd_allow 未改），所以这里放开不会让菜单在过渡中途弹出来。
//  【为什么 S_OSD 不列】S_OSD 时 menu_active=1，osd_top 走菜单几何；
//    此时 info_on 即使为 1，信息条的 16/17 号槽也会被菜单覆盖掉，
//    列进来只会让"信息条可见"这个观测值变得没有意义。
assign info_show = info_on & ((state == S_HD)    || (state == S_STUCK) ||
                              (state == S_HDF)   || (state == S_ARM)   ||
                              (state == S_TRANS) || (state == S_LOW));

//==============================================================================
//  ★★ 2026-11-04 【换图逻辑统一抽到 img_switch】★★
//==============================================================================
//  主页 / OSD 相册页 / 文件页打开 —— 三处换图共用这一份决策和缓存：
//    · 能不能换（目标缩略图已在缓存里）、换到哪（环形 3 槽）、谁该预取；
//    · 本模块只决定**怎么显示**：
//        anim = 1（主页）     → S_ARM / S_TRANS 播过渡，播完装高清
//        anim = 0（菜单里）   → 帧边界直接换 a_base（相册页），或关菜单后直接上屏（文件页）
//  输入按状态分流（菜单里 KEY3/KEY4 是光标，不能顺带换图）：
//    主页：S_HD / S_STUCK / S_LOW（★ 装高清中也能切，见 S_LOW）
//    菜单：相册页按钮 / 文件页打开
//==============================================================================
wire in_osd  = (state == S_OSD);
wire home_ok = ((state == S_HD) || (state == S_STUCK) || (state == S_LOW)) &&
               !zoom_on && !key_osd_enter;
wire osd_ok  = in_osd && (osd_ph == 2'd1) && !exit_pend && !key_osd_exit;
wire ev_nx   = home_ok ? (key_next | auto_go) : (osd_ok & album_next);
wire ev_pv   = home_ok ? key_prev             : (osd_ok & album_prev);
wire ev_jp   = osd_ok & I_jump;

wire        sw_dir, alb_ok, cur_ok;     // （sw_go 已在 §1.5 之前声明）
wire        tl_req;
wire [1:0]  tl_rel;
wire [20:0] tl_addr, cur_addr, old_addr;
wire [7:0]  cur_abs;
wire [15:0] sw_base;
wire        hd_take, tl_take;
// ★ 2026-11-02：img_switch 的 3 槽分辨率缓存（13 位/维，见那边的 res_ram）
wire [12:0] sw_res_w, sw_res_h;

img_switch #(
    .THUMB0_ADDR(THUMB0_ADDR), .THUMB1_ADDR(THUMB1_ADDR), .THUMB2_ADDR(THUMB2_ADDR)
) u_sw (
    .clk(clk), .rst(rst),
    .I_next(ev_nx), .I_prev(ev_pv), .I_jump(ev_jp), .I_jump_idx(I_jump_idx),
    // ★ 2026-11-04 还原 77c748c：缩放期间不做缩略图预取 —— 那时 `ld_fin` 只可能是
    //   高清装载，缩放的"装载完成"判据才成立（旧版没有后台预取）。
    .I_pf_en((state != S_BOOT_T) && (state != S_ZOOM)),
    .O_base(sw_base), .I_any(I_any), .I_nimg(I_nimg), .I_win4(I_win4), .I_alb_ready(I_alb_ready), .O_alb_ok(alb_ok),
    .O_tl_req(tl_req), .O_tl_rel(tl_rel), .O_tl_addr(tl_addr),
    // ★ 2026-11-02 [NO-LD-TO]：`I_tl_err` 现在就是装载器真的报错（`ld_fail` 已等价于
    //   `ld_err`，装载超时整体删除）。所以 img_switch 记 `dv`（"这一格没有图"）的判据
    //   第一次变干净：只有读到的确实不是 BMP 才判死，不会再因为"慢了"而锁死一个方向。
    .I_tl_take(tl_take), .I_tl_fin(ld_fin & ~ld_fmt_r),
    .I_tl_ok(~ld_fail), .I_tl_err(ld_fail),
    // ★ 2026-11-02 [ABORT-NOT-ERR]：这次结束是被放弃的 ⇒ 既不置 tv 也不置 dv。
    .I_tl_abort(ab_pulse),
    // ★ 2026-11-02【信息条分辨率】：缩略图装完就把源图 W/H 按环槽记下来
    .I_src_w(ld_src_w), .I_src_h(ld_src_h),
    .O_res_w(sw_res_w), .O_res_h(sw_res_h),
    .O_go(sw_go), .O_dir(sw_dir),
    .O_cur_rel(cur_rel), .O_cur_addr(cur_addr), .O_old_addr(old_addr),
    .O_cur_ok(cur_ok), .O_cur(cur_abs)
);
assign O_base = sw_base;


//------------------------------------------------------------------------------
//  1.6 当前图的源分辨率（信息条显示用）
//------------------------------------------------------------------------------
//  ★★★ 2026-11-02：改成 img_switch 里按**环槽**存的 3 份（LUTRAM）。
//  【原来】这里是一份 `rw/rh`，只在 `ld_done && ld_fmt_r`（**高清装完**）更新。
//  【上板现象】"分辨率要等高清装完才变"。因为高清装载是过渡动画播完才发的
//    （见 S_TRANS 收尾的 `hd_want <= 1`）⇒ 实际滞后 = 动画(1~3s) + 装载(0.5~4s)。
//  【为什么不能只在"缩略图装完"更新一份】`ld_src_w/h` 是"最近一次装载"的头，
//    而预取装的是邻居（img_switch 的 O_tl_req）⇒ 切图那一刻它常常已经是别的
//    图了。必须按槽记住，见 img_switch 的 res_ram 段（26 LUT / 0 FF）。
//  【效果】切图当拍（cr/cs 变）分辨率就跟着变，和名字、读秒的"当前图"口径一致。
assign res_w = {3'd0, sw_res_w};
assign res_h = {3'd0, sw_res_h};


//------------------------------------------------------------------------------
//  1.7 信息条重合成请求（文字变了 ⇒ 240µs 后补画一帧 888）
//------------------------------------------------------------------------------
reg [13:0] inf_wait;
reg        inf_req;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        inf_wait <= 14'd0;
        inf_req  <= 1'b0;
    end else begin
        if (osd_trig) begin
            inf_wait <= 14'd12000;          // 240µs：覆盖最坏"整行新字形"的 SPI 装载
            inf_req  <= 1'b1;
        end else if (inf_wait != 14'd0)
            inf_wait <= inf_wait - 14'd1;
        if (inf_req && (inf_wait == 14'd0) &&
            ((state == S_HD) || (state == S_STUCK)))
            inf_req <= 1'b0;
    end
end
wire inf_refresh = inf_req && (inf_wait == 14'd0) && !osd_on && info_on &&
                   ((state == S_HD) || (state == S_STUCK));

// 目标图号：★ 2026-11-02 已并入上面 §1.5 的 `swap_to`（换图决策共用一份），
//   这里原来的 `tgt_new = key_next ? idx_next(img_idx) : idx_prev(img_idx)` 删掉：
//   idx_next/idx_prev 的回绕分支在滑动窗口下永远不会命中，留着只是隐患。

//==============================================================================
//  2. 装载请求序列器（只写 ldp / ld_req / ld_* / ld_fin / hd_*）
//==============================================================================
//  ★ 2026-11-04：两个来源 —— 状态机的 hd_want（当前图高清）和 img_switch 的
//    tl_req（缓存预取）。高清优先；都要等相册偏移属于当前窗口（alb_ok）。
//    偏移 / 目标地址在**收下那一拍**按当前窗口现取 ⇒ 状态机里不再有十几处
//    `ld_off <= img_off(...)` 的散装赋值（一台 3:1 mux + 2 位选择）。
//  ab_req：主页在装高清时又切了图 ⇒ 还没发单就直接作废；已发单就叫装载器尽快收尾
//    （bmp_loader 的 abort：读完手上这个扇区就停，报 err）。
//  ★ 2026-11-02 [SD-GATE]：去掉这两处的 `&& ld_sd_ready`。
//    顶层的两条复位线已经保证"本模块出复位时 sd_init_done 一定为 1"
//    （rst_app = rst_all | ~alb_first，而 alb_first 来自 rst_card 放行后的
//     第一次目录扫描）⇒ 这里再门一次只是重复条件，白白多一个死锁点。
// ★ [BUSY-CHECK]：发单点必须显式看"装载器是否空闲"（ld_can_load = ld_ready & ~quad_own）。
//   原来只看 ldp==0 —— 那是"我自己没在装"，不等于"装载器空闲"（可能被四宫格占着，
//   或上一单还没收尾）。不看就会先发单、再在 ldp==1 里无限期等 ld_can_load。
//   ⇒ ldp==1 现在只是一拍的过路态，不再是"等别人"的状态。
assign hd_take = (ldp == 2'd0) && hd_want && alb_ok && !I_file_page && ld_can_load;
assign tl_take = (ldp == 2'd0) && !hd_want && tl_req && !I_file_page && ld_can_load;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        ldp        <= 2'd0;
        ld_req     <= 1'b0;
        ld_fmt     <= 1'b0;
        ld_off     <= 32'd0;
        ld_len     <= 32'd0;
        ld_base    <= 21'd0;
        ld_fin     <= 1'b0;
        ld_fail    <= 1'b0;
        ld_fail_st <= 1'b0;
        ld_fmt_r   <= 1'b0;
        hd_valid   <= 1'b0;
        hd_img     <= 2'd0;
        hd_zoom_s  <= 8'h10;
        hd_ax_s    <= 10'd320;
        hd_ay_s    <= 10'd240;
        abort_req  <= 1'b0;
        ld_tgt_ok  <= 1'b0;
        ab_pulse   <= 1'b0;
    end else begin
        ld_fin     <= 1'b0;
        ab_pulse   <= 1'b0;
        // ---- [ABORT-LATCH] 只锁"已经发单（ldp==2）"的那次；ldp 一退出就撤 ----
        if (ldp != 2'd2)      abort_req <= 1'b0;
        else if (ld_drop)     abort_req <= 1'b1;
        // ---- [TGT-STAMP]：hd_take 置"本单属于当前图"；**只有换图**才失效 ----
        //   ⚠ 不能挂 `hd_kill`：它是单拍，且在缩放换档时与 hd_take 同拍/前后拍出现，
        //     会让"刚发单"的那次也被判成过期 ⇒ 烘焙完了也 hd_valid=0，缩放永远等不到 888
        //     （tb_zoom_flow 的 T5 就是这么挂的）。
        //   `hd_kill` 本来就会清 `hd_valid`（见上面两行），画布该失效的地方已经失效了。
        if (hd_take)  ld_tgt_ok <= 1'b1;
        if (sw_go)    ld_tgt_ok <= 1'b0;

        // 换图 / fit 变化：高清槽里已经不是当前图
        if (hd_kill) hd_valid <= 1'b0;
        // 只有"给当前图装的那次"失败才算失败：被换图打断（ab_req）的那次不算，
        //   否则 S_LOW 会白重试一次、甚至误落到 S_STUCK。
        //   判据用 hd_img 内容标签 —— 这就是 77c748c 里 !hd_stale 的等价写法。
        if (ld_fin && ld_fail && ld_fmt_r && (hd_img == cur_rel)) ld_fail_st <= 1'b1;

        case (ldp)
        2'd0: begin
            ld_req <= 1'b0;
            if (hd_take || tl_take) begin
                ld_fmt   <= hd_take;
                ld_fmt_r <= hd_take;
                ld_off   <= img_off(hd_take ? cur_rel : tl_rel);
                ld_base  <= hd_take ? HD_ADDR : tl_addr;
                // ★ 还原 77c748c：收下时记下"这次高清装的是窗口第几格"
                if (hd_take) hd_img <= cur_rel;
                // ★ 2026-11-05 [LD-SNAP]：同一拍把烘焙几何锁死，loader 之后不管
                //   `zoom_val` 怎么变，装出来的都是这一档
                if (hd_take) begin
                    hd_zoom_s <= zoom_val;
                    hd_ax_s   <= za_x;
                    hd_ay_s   <= za_y;
                end
                if (hd_take) ld_fail_st <= 1'b0;
                ldp      <= 2'd1;
            end
        end
        2'd1: begin
            // ★ [NO-LD-TO]：这里不再判超时（原来 0.335s/5.37s 会误杀正常装载）。
            //   出口两条：被放弃（切图 / 进文件页），或装载器终于空了。
            // ★ 2026-11-02 [FP-ABORT]：不再带 `ld_fmt_r` —— 缩略图也要能作废。
            //   原来只放弃高清，于是"进文件页"要先等相册那张缩略图读完（1~4s）。
            if (ld_drop) begin
                ld_fin   <= 1'b1;
                ld_fail  <= 1'b1;
                ab_pulse <= 1'b1;          // 告诉缓存：这次不算"没有图"
                ldp      <= 2'd0;
            end else if (ld_can_load) begin
                ld_req <= 1'b1;
                ldp    <= 2'd2;
            end
        end
        2'd2: begin
            ld_req <= 1'b0;
            // ★ [ABORT-LATCH]：abort 电平由块顶统一锁存（ldp!=2 就撤），这里只给缓存打标。
            if (ld_drop) ab_pulse <= 1'b1;
            // ★ [NO-LD-TO]：只等装载器真正的结果，不再有超时分支
            if (ld_done || ld_err) begin
                ld_fin  <= 1'b1;
                ld_fail <= ld_err;
                // ★ 只有**高清**装载能动 hd_valid；而且必须带"目标戳"：
                //   abort 丢失时旧装载会 err=0 收尾，若只看 err 就会把旧画布验证成有效。
                if (ld_fmt_r) hd_valid <= !ld_err && ld_tgt_ok;
                ldp <= 2'd0;
            end
        end
        default: ldp <= 2'd0;
        endcase
    end
end


//==============================================================================
//  3. 帧边界脉冲
//==============================================================================
reg        ps_start_d;
reg  [4:0] tt_cnt;
reg        tick_d;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        ps_start_d <= 1'b0;
        tt_cnt     <= 5'd0;
    end else begin
        ps_start_d <= ps_start;
        // ★ 2026-10-23：改按【时钟】计数，不再按 pipe_ce。
        //   【为什么】场消隐期间各图层的 layer_ready 全是 0（ps_gen_image 的
        //   warmed 被 frame_active=0 清掉，img_layer_hd 的 FIFO 被 flush 清掉），
        //   所以 pipe_ce 在帧间根本不动。原来写 `&& pipe_ce` ⇒ tt_cnt 永远停在
        //   初值 ⇒ tick 永不产生 ⇒ S_LOW / S_TRANS 卡死（过渡动画一帧都不推进）。
        //   【为什么现在按拍数就够】帧尾的流水线由 viewer_scene 的 drain 强推
        //   PIPE 拍排空，正好落在下面这 PIPE+2 拍的窗口里。所以 6 拍之后
        //   流水线一定是干净的空状态，可以安全改场景参数/换帧。
        if (ps_start_d & ~ps_start) tt_cnt <= PIPE + 5'd2;
        else if (tt_cnt != 5'd0)     tt_cnt <= tt_cnt - 5'd1;
    end
end

wire tick = (tt_cnt == 5'd1);

//------------------------------------------------------------------------------
//  ★ [ZOOM-FREEZE] "等帧边界"的兜底超时
//------------------------------------------------------------------------------
//  [ZOOM-SWAP] 的提交必须发生在帧边界（tick），否则场景参数在帧中间被改会撕裂。
//  但 tick 依赖"这一帧真的扫完了"——任何一次 SDRAM 卡顿都会让 tick 缺席，
//  于是 zph 永久停在 2（一直显示 565 模糊预览）⇒ 上板"动画放一半卡死"。
//  这里给一个按**拍数**的上限。
//  ★★★ 2026-10-30 [ZOOM-NOWRIP]：超时**绝不能落在帧中间** ★★★
//  【上板现象】缩放时偶尔一帧"上半是缩略图、下半是白色碎块"，像撕裂。
//  【根因】原来兜底写成 `tick || sw_to`，而 sw_to 是自由计数的：
//      20 位 ≈ 21ms ≈ **1.25 帧** ⇒ 只要这一帧稍微慢一点（装载写画布抢了
//      SDRAM、帧时 >21ms），sw_to 就会在**扫描进行中**拉高 ⇒ 当拍改掉
//      a_hd / a_base / fmt565。这一帧于是"上半走 565 缩略图、下半走 888 画布"；
//      888 的 ARGB 字按 565 解码 ⇒ 屏幕下半就是一片白色碎块（上板截图）。
//  【现在】分两级，都不允许落在扫描中间：
//      sw_to   = 21ms  ⇒ **允许**提交，但只准在 `!ps_start`（帧间空隙）落地；
//      sw_hard = 335ms ⇒ 真冻死（ps_start 一直高）时的最后手段，无条件提交。
//    正常情况下永远走 tick；帧慢一点就等它扫完再提交，一帧都不会撕裂。
reg [23:0] sw_cnt;
always @(posedge clk or posedge rst) begin
    if (rst)                       sw_cnt <= 24'd0;
    else if (!sw_ok || tick)       sw_cnt <= 24'd0;
    else if (sw_cnt != 24'hFFFFFF) sw_cnt <= sw_cnt + 24'd1;
end
wire sw_to   = (sw_cnt >= 24'h0FFFFF);      // 21ms：可以提交，但只能在帧间
wire sw_hard = (sw_cnt == 24'hFFFFFF);      // 335ms：彻底冻死兜底
wire sw_point = tick || (sw_to && !ps_start) || sw_hard;

//------------------------------------------------------------------------------
//  ★ 2026-10-30 回退记录：装载启动**不做**帧同步
//------------------------------------------------------------------------------
//  曾试过把 `ld_kick` 推到帧边界（`wire ld_point = tick || 42ms 兜底`），
//  理由是"别在扫描中途往 SDRAM 灌写流"。**上板实测：这一版撕裂更严重**
//  （照片那种下半白碎块），退回"按键当拍就踢"反而干净。
//  结论：撕裂的真正来源是 [ZOOM-NOWRIP] 那两个提交/切换点（它们直接改
//  a_hd/a_base/fmt565，会在帧中间换纹理源）；装载只是往 HD_ADDR 灌数据，
//  和当前显示的缩略图不在同一块区域，晚踢/早踢不影响那一帧的正确性。
//  ⇒ 保留 [ZOOM-SEAMLESS] + [ZOOM-NOWRIP]，删除 ld_point。见计划文件 9.7。
//------------------------------------------------------------------------------

// （t_dbg：2026-10-26 随数码管清理删除，原来 assign t_dbg = t）

//==============================================================================
//  3.4 过渡进度改成"按时间"（2026-10-23）
//==============================================================================
//  【问题】`t` 原来是"每帧 +1"，于是过渡时长 = T_END × **实际**帧时间。
//    fx2（淡入淡出）两层都满屏、每帧要取两份纹理 ⇒ 帧时间约为别种效果的两倍
//    ⇒ 同样 300 帧，fx2 走 10s，fx0/fx1/fx3 走 5s。用户实测就是"只有 fx2 是 10s"。
//
//  【做法】`t` 不再数帧，改成数"过了多少个 1/60s"：
//    · 自由计数器 tcyc 每 FRAME_CYC 拍（16.7ms）产生一步，累积到 tstep；
//    · 只在帧边界（tick）把 tstep 一次性加到 t 上。
//    ⇒ 时长恒定 = T_END × 16.7ms = 5s，与帧率无关；
//    ⇒ 几何/alpha 只在帧边界变，绝不会"帧中间换几何"造成撕裂。
//
//  【代价】帧率低于 60fps 时 t 一次跳 2 步（fx2 约 150 帧走完 5s），
//    动画帧数变少但时长正确 —— 这正是想要的（"慢"比"少几帧"更刺眼）。
//==============================================================================
localparam [19:0] FCYC_LAST = FRAME_CYC[19:0] - 20'd1;
// ★ 2026-10-26：按时长分档 —— fx0/1/2 走 T_END_F（1s），fx3/fx4 走 T_END（3s）。
//   fx_id 在按键切图时已锁存，整个过渡期间不变 ⇒ 结束判据稳定。
wire        fx_slow    = (fx_id >= 3'd3);
// ★ 2026-10-27：同样不要对 integer 参数位选（见上面 AUTO_SEC_W 的说明）。
//   原来写 `T_END[11:0]` / `T_END_F[11:0]`，TD 若算成 0 会让
//   `t >= t_end_cur` 一拍就成立 ⇒ 过渡直接跳到终点（动画一帧都没有）。
//   （实际板子上过渡是正常的，说明 TD 这里算对了；但改成定宽写法零风险，
//     顺便把没人用的 T_END10 死代码删掉 —— 省一条 12 位常量比较。）
localparam [11:0] T_END_W   = T_END;
localparam [11:0] T_END_F_W = T_END_F;
wire [11:0] t_end_cur  = fx_slow ? T_END_W : T_END_F_W;

reg [19:0] tcyc;        // 0..FRAME_CYC-1 自由计数
reg  [8:0] tstep;       // 距上个帧边界累积了多少个 1/60s（饱和 255）
// ★ 2026-10-28：到终点后已经等了几个 tick（几何落定用，见 SETTLE_TICKS）
localparam [7:0] SETTLE_W = SETTLE_TICKS;
reg  [7:0] geom_settle;
wire       geom_ok = (geom_settle >= SETTLE_W);

always @(posedge clk or posedge rst) begin
    if (rst)             geom_settle <= 8'd0;
    else if (state != S_TRANS) geom_settle <= 8'd0;
    else if (tick && (t >= t_end_cur) && (geom_settle != SETTLE_W))
        geom_settle <= geom_settle + 8'd1;
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        tcyc  <= 20'd0;
        tstep <= 9'd0;
    end else if (state != S_TRANS) begin
        tcyc  <= 20'd0;
        tstep <= 9'd0;                          // 只在过渡里累积
    end else begin
        tcyc <= (tcyc == FCYC_LAST) ? 20'd0 : (tcyc + 20'd1);
        if (tick)
            tstep <= 9'd0;                      // 帧边界：已经交给 t
        else if ((tcyc == FCYC_LAST) && (tstep != 9'd255))
            tstep <= tstep + 9'd1;
    end
end

// ★ 单帧最多走 4 步：正常情况下 tstep=1（fx0/1/3）或 2（fx2 帧时间翻倍）。
//   万一某帧异常慢（纹理断供等），也绝不让 t 一次跳到终点 —— 上板表现会变成
//   "画面静止 → 稍微动一下 → 闪到目标图"，比慢下来更糟。封顶后最坏只是慢。
wire [8:0] tadd  = (tstep == 9'd0) ? 9'd1 : ((tstep > 9'd4) ? 9'd4 : tstep);
wire [12:0] tnext = {1'b0, t} + {1'b0, tadd};        // 13 位，防 12 位溢出

//==============================================================================
//  3.5 看门狗计数（三个状态共用一个计数器，必须放在 tick 之后）
//==============================================================================
//  计数区间互斥（见顶部 §1.4 的说明）：
//    · S_TRANS        没有一帧收满 ⇒ 纹理通路卡死，降级到 S_LOW
//    · S_ARM  相位1/2 没有 tick    ⇒ 帧压根没跑起来，硬切到 S_TRANS
//    · S_LOW          等不到 hd_ready ⇒ 两级阈值：先重试装载，再认输进 STUCK
//  这是"绝不停在某个状态里不动"的兜底。
wire wd_in_trans = (state == S_TRANS);
// ★ 2026-11-02：**去掉 `arm_ph != 0`**。相位 0 在等 `!busy`，原来完全没兜底；
//   只要帧引擎因为任何原因结束不了当前帧，就永久停在这儿。
//   加进来之后相位 0 也受 arm_stall（0.67s）保护，代价零。
wire wd_in_arm   = (state == S_ARM);
wire wd_in_low   = (state == S_LOW);
// ★★ 2026-11-02 【单张补装段原来没有任何看门狗】★★
//   S_ONE / S_ONE_W 只等 `ld_fin`，而装载器一旦不给（`ld_can_load=0`、
//   读卡异常、归属被四宫格占住），这里就**永久停住**：
//     · 主页路径：屏幕留在缩略图上，按键全无反应 —— "永久卡死"
//     · 菜单路径：菜单停在原处，动不了
//   原来唯一的兜底是序列器的装载超时（`ld_to`），但它在 2026-11-02 被整体删除
//   —— 因为它会误杀正常的缩略图装载（逐行读源图本来就要 0.3~1s）。
//   现在装载"卡住"只能靠下面这几个 S_ARM/S_TRANS/S_LOW 的**状态级**时间看门狗兜住。
wire wd_in_one   = 1'b0;      // ★ 2026-11-04：S_ONE/S_ONE_W 已删除
wire wd_run      = wd_in_trans | wd_in_arm | wd_in_low | wd_in_one;

// 各状态收到"进展事件"就重数（帧边界 / 高清就绪）
// ★ 2026-11-02：S_TRANS 的"进展"多一个条件 —— 相册必须已经扫完。
//   滑动窗口换了 base 之后 album_src 会重扫，重扫期间 I_img_off* 还是**旧窗口**的。
//   过渡跑完时如果拿这份旧偏移去发高清单，装回来的就是上一张（甚至别的槽）的图。
//   正常情况下重扫只有 1ms 级、过渡 1s 级，碰不上；但"碰不上"不是"不会碰上"
//   ⇒ 干脆把它写成条件：没扫完就不许退出过渡。同时这里不再让 tick 无脑清
//   看门狗，否则相册真挂了会永久停在过渡里（用户报过的"卡死"形态）。
// ★ 2026-11-04：过渡不再等相册（高清装载自己等 alb_ok）；S_LOW 里切图直接进 S_ARM ⇒ sw_go 也清零
wire wd_clr = (wd_in_trans & tick) | (wd_in_arm & tick) | (wd_in_low & hd_ready) | sw_go;

// 饱和在最长阈值，不回绕（回绕会让超时永远不成立 = 永久卡死）
wire wd_sat = (wd_cnt == 29'h1FF_FFFFF);

always @(posedge clk or posedge rst) begin
    if (rst)               wd_cnt <= 29'd0;
    else if (!wd_run)      wd_cnt <= 29'd0;
    else if (wd_clr)       wd_cnt <= 29'd0;
    else if (!wd_sat)      wd_cnt <= wd_cnt + 29'd1;
end

// 各阈值。★ 都带状态门控：防止某个状态的阈值在别的状态里误触发
//   trans : 2^27 @50MHz ≈ 2.68s   过渡里没有一帧收满
//   arm   : 2^25 @50MHz ≈ 0.67s   预热/等帧一直没有 tick
//   low1  : 2^28 @50MHz ≈ 5.37s   还没好，重试装载一次
//   low2  : 2^29 @50MHz ≈ 10.7s   重试也没好，认输进 S_STUCK
//   （两级阈值的历史原因：重试那拍若靠"清零重数"，清零后的第一拍
//     low_stall 仍为 1（组合逻辑读的是旧值）会被误判成"重试也失败"。
//     改成两级阈值就没有这个时序陷阱。）
wire trans_stall  = wd_in_trans && (wd_cnt == 27'h7FF_FFFF);
wire arm_stall    = wd_in_arm   && (wd_cnt == 25'h1FF_FFFF);
// ★ 2026-11-02 时序：ONE_TO 默认 33_554_431 = 2^25-1 ⇒ "≥ 2^25" 等价于
//   高 4 位非零，1 个 LUT。写成 `wd_cnt >= ONE_TO` 是 29 位比较器（≈8 LUT），
//   在已经拥塞的 clk0 上不值得。仿真要缩短时加 -DSIM_FAST_TO 走比较器那条。
`ifdef SIM_FAST_TO
wire one_stall = wd_in_one && (wd_cnt >= ONE_TO);
`else
wire one_stall = wd_in_one && (|wd_cnt[28:25]);
`endif
wire low_stall1   = wd_in_low   && (wd_cnt == 29'hFFF_FFFF);
wire low_stall2   = wd_in_low   && (wd_cnt == 29'h1FF_FFFFF);

//==============================================================================
//  4. 过渡参数发生器
//==============================================================================
reg load_fx, load_fx_d;

always @(posedge clk or posedge rst) begin
    if (rst) begin tick_d <= 1'b0; load_fx_d <= 1'b0; end
    else     begin tick_d <= tick; load_fx_d <= load_fx; end
end

wire fx_upd = tick_d | load_fx_d;

wire [7:0]  fx_alpha;
wire [31:0] fx_pos_a, fx_size_a, fx_pos_b, fx_size_b;
wire [39:0] fx_k_a;

viewer_fx #(
    .IMG_W(640), .IMG_H(480), .SRC_W(160), .SRC_H(240),
    .T_END_F(T_END_F), .T_END_S(T_END)
) u_fx (
    .clk(clk), .rst(rst),
    .fx_id(fx_id), .t(t),
    .upd(fx_upd),
    .dir(dir),
    .pos_a(fx_pos_a), .size_a(fx_size_a), .k_a(fx_k_a),
    .pos_b(fx_pos_b), .size_b(fx_size_b), .k_b(k_b),
    .alpha_b(fx_alpha)
);

// ★ 备战期间把 alpha 强制成 0。
//   原因：S_ARM 里为了预热 B 层已经把 b_en 打开了，而 B 是"前景"——
//   alpha=255 的话它会直接盖住 A（屏幕上会突然冒出下一张图）。
//   压成 0 后 ps_merge_alpha8 的 i9=256 ⇒ 输出恒等于 A，B 完全不可见，
//   但 B 层的请求侧 / 像素 FIFO 照样在跑，warmed 和预取水位都建起来了。
assign alpha_b = (state == S_ARM) ? 8'd0 : (osd_on ? osd_fade : fx_alpha);

// OSD 只允许在"静止高清 / 降级"里打开 —— 加上 S_LOW（等高清那 4s）。
// ★ 2026-11-02 [OSD-EARLY]：S_LOW 也允许开菜单。
//   原来只认 S_HD/S_STUCK ⇒ 切完图要等高清装完（大图 4s）KEY1 才有反应，
//   上板就是"切完图想开菜单，按了没动静"。
//   为什么这里可以放：高清装载只往 HD_ADDR 写，与菜单显示完全无关 ——
//   让它**在菜单背后继续装**，退出菜单时 hd_ready 往往已经为 1，直接切 888。
//   （对比 S_TRANS/S_ARM：那里 a/b 两层在动，菜单会盖在过渡上，仍不放。）
// ★ 2026-11-02 [ZOOM-EARLY]：缩放同理 —— 切图后不必等高清装完才能按 KEY2。
//   但缩放**不能**像菜单那样直接放：S_LOW 里 a_hd=0（画布是缩略图），
//   而 S_ZOOM 的 zph==0 会无条件宣布"888 画布有效"（a_hd<=1/a_base<=HD_ADDR），
//   那样进去就是半张画布花屏。所以 S_LOW→S_ZOOM 从 **zph=1（565 预览）**进，
//   见下面 S_LOW 里的那个分支。
wire osd_quiet = (state == S_HD) || (state == S_STUCK);
assign zoom_allow = osd_quiet || (state == S_LOW);
assign osd_allow  = osd_quiet || (state == S_LOW);

//==============================================================================
//  ★★ 2026-10-25：S_ARM 期间把 **A 的几何也强制成"起始静止帧"的满屏几何 ★★
//==============================================================================
//  【上板现象】按一下切换键，屏幕**黑一下**（约一帧 16.7ms），然后才开始动。
//  【根因】相位 0→1 的那一拍 `compose_en` 0→1，App 下一拍就发帧请求
//    （compose_rise ⇒ pending，tick_cnt 直接置满）—— 而此刻 `viewer_fx` 的
//    `pos_a/size_a` 还是**上一轮动画最后一帧**的几何：
//        t ≈ T_END、q ≈ 256 ⇒ fx0 的 A 是 X0 = −640（整块在屏幕外）
//    而 `alpha_b` 在 S_ARM 里被压成 0（B 完全不可见）
//    ⇒ 这一帧 = "A 在屏幕外 + B 透明" = **整帧纯黑**，写满就提交、场边界就上屏
//    ⇒ 屏幕上正好黑一帧。等到这帧结束来 tick，t=0 的几何才被算出来，
//      下一帧就恢复正常 —— 所以现象是"黑一下"而不是持续黑。
//  【为什么以前没有】QMUL 之前被截成 16 位，q 最多只走到 76/256，
//    A 只滑出屏幕 30%（左边一条黑边，不明显）。QMUL 修好后 q 真到 256
//    ⇒ A 整块滑出 = 整屏黑，问题才暴露。
//  【为什么可以这样修】四种效果的 t=0 都是"A 满屏 (0,0) 640×480"
//    （fx0/fx1/fx3 的 A 只是纯平移、尺寸恒 640×480；fx2 两层都满屏），
//    所以 S_ARM 期间把 A 钉在满屏，就等于"起始静止帧"，与预热语义完全一致：
//    屏幕上应当是"旧图的放大版"，而不是黑。
//    k_a 一并钉成满屏对应的 {16384, 32768}（= 65536*160/640、65536*240/480），
//    避免"新尺寸 + 旧 k"混搭（这正是 2026-10-24 那条 f_k 陈旧的同类问题）。
//  【谁采样】ps_gen_image 只在 frame_active 上升沿采样 f_x0/f_rw/f_kx，
//    所以 S_ARM 结束切回真实几何时不会撕裂：下一帧整体换过去。
//  ★ 2026-10-25 补：**S_LOW 也要钉住满屏** —— 动画结束后进 S_LOW 装高清时，
//    A 的几何还是动画最后一帧的（fx0 是 X0=−640，A 整块在屏幕外）
//    而这一相位 A 已经换成目标图缩略图 ⇒ 不钉住就是"动画一结束整屏变黑"。
//    S_LOW 的语义本来就是"目标缩略图满屏、等高清"，钉住正好一致。
//  ★ 2026-10-26 补：**S_OSD 也要钉住** —— 上板现象"主页播过切换动画后，
//    再打开 OSD，菜单背景是黑的"。
//    【为什么】S_OSD 相位 0 会把 a_hd 拉成 0（见下面 S_OSD 的赋值）⇒ A 从
//      u_hd（没有 cfg_pos，永远满屏）切回 u_ta（吃 a_pos/a_size/a_k）
//      ⇒ 用的是动画最后一帧的几何：fx0 X0=±640、fx1 Y0=±480、fx3/fx4 后半段
//      也是整块出屏 ⇒ ps_gen_image 框外恒输出 0 ⇒ pix_a 全黑，
//      而 OSD 是 75% 半透明 ⇒ 菜单底下就是黑的。
//    【为什么只有"播过动画之后"才黑】t 停在 T_END；开机没跑过动画时 t=0，
//      fx 几何正好是 t=0 的姿势（A 满屏），所以开机会话里打开菜单是正常的。
//    【为什么 fx2 不黑】淡入淡出的 c_pos_a 恒为 (0,0) 满屏（viewer_fx 的 fx2 分支）
//      ⇒ A 从不出屏 ⇒ 这也是上板确认本诊断的判据。
//    【等价不变量】只要 A 走缩略图通路、又不在过渡中，几何就必须是"目标图满屏"：
//        (a_hd == 0) && (state != S_TRANS)
//     这里仍用状态列表写法，和上面两条修补保持同一风格。
//==============================================================================
// ★ 2026-11-04：+S_BOOT_T —— 从文件页打开图片时 S_BOOT_T 里仍在出旧图的 565，
//   几何不钉住就是上一轮动画最后一帧的姿态（可能整块在屏幕外 ⇒ 黑屏）。
wire arm_geom = (state == S_ARM) || (state == S_LOW) || (state == S_STUCK) ||
                (state == S_OSD) || (state == S_ZOOM) || (state == S_BOOT_T);

//==============================================================================
//  ★ 2026-10-30 缩放预览的几何：A 满屏，k 按档位插值 ⇒ 这才是"动画"
//==============================================================================
//  【上板现象】按放大只闪一下黑然后跳到结果，没有过渡。
//  【为什么】S_ZOOM 不在 arm_geom 里，A 层吃的是 viewer_fx 的几何 —— 而 fx 在
//    非过渡态时 t 停在某个值，几何是错位的；而且 k 恒为满屏值，本来就不缩放。
//  【现在】S_ZOOM 时 A 固定满屏 (0,0)640×480，k 每帧向目标逼近：
//      565 缩略图 320×240 铺满 640×480 ⇒ 基准 k = 0.5（Q16 = 32768）
//      zoom=2/4/8 ⇒ 目标 k = 0.25 / 0.125 / 0.0625（纯右移）
//    每帧走剩余量的 1/4 ⇒ 约 8~10 帧到位（130~170ms），和 fx 的缓动观感一致。
wire zoom_g = (state == S_ZOOM);

// ★ 2026-10-30：改成连续赋值。原来写成 `always @(*) case(zoom_val)`，
//   在 iverilog 里出现过 zk_tgt 恒为 X（同一模块里另一段结构完全一样的
//   always @(*) 却正常）⇒ zk 永不收敛、2× 预览不缩放。
//   语义完全等价（四选一常量 mux），20 位 ×3 级 mux，面积可忽略。
wire [19:0] zk_tgt = (zoom_val == 8'h20) ? 20'd16384 :   // 2.0x
                     (zoom_val == 8'h40) ? 20'd8192  :   // 4.0x
                     (zoom_val == 8'h80) ? 20'd4096  :   // 8.0x
                                           20'd32768;    // 1.0x（= 缩略图铺满）

//==============================================================================
//  ★★ 2026-10-30 [ZOOM-SEAMLESS]：插值**不能在"888→565 切换那一拍"走步** ★★
//==============================================================================
//  【上板现象】刚按下缩放的那一瞬间画面"撕裂/跳一下"，然后才是流畅动画。
//  【根因】`tick`（帧边界）那一拍同时干了两件事：
//      ① `pv_pend && tick` → a_hd/a_base/fmt565 三件一起翻（切到 565 预览）
//      ② `zoom_g && tick`  → zk / pgx / pgy 各向目标走一步
//    于是"预览第一帧"的几何**已经不是上一帧那个 1:1 了**：
//      zk 从 32768 一步走到 28672 ⇒ 32768/28672 = **1.143×**
//    也就是说按下缩放的那一帧，画面整体突然放大 14.3%（或缩小同样的量）。
//    再叠上"888 清晰 → 565 模糊"的清晰度突变，看起来就是撕裂。
//  【修法】切换那一拍（此刻 zph 还是 0）**不推进插值** ⇒ 预览第一帧的
//    zk 仍是 32768、相位仍是 0 ⇒ 几何与上一帧的 888 **逐位相同**，
//    只有清晰度变化；动画从下一帧才真正开始。
//  【为什么用一个与门就够】`zph` 只有 0/1/2：
//      zph==0 → 888 相位，此时 k_a 被强制成 65536，插值走不走都看不见；
//      zph!=0 → 565 相位，正常插值。
//    所以"zph!=0 才走"不但修掉跳变，逻辑还比原来少一项。
//==============================================================================
wire zstep = zoom_g && tick && (zph != 2'd0);

reg [19:0] zk;
// ★ 2026-11-03 [AREA]：原来 2 个比较 + 2 组差值 + 2 个 >3 判断 + 加/减各一台（≈6 条 20 位链）。
//   改成：一台 21 位有符号差 zd → 取模 |zd| → 一台"加或减 |zd|>>2"。逐值相同：
//     zk > tgt：zd>0，|zd| = zk−tgt，下一值 = zk − (|zd|>>2)（|zd|>3 时）否则 tgt
//     zk < tgt：zd<0，|zd| = tgt−zk，下一值 = zk + (|zd|>>2)（|zd|>3 时）否则 tgt
//     zk = tgt：|zd|=0 ⇒ 写 tgt（= zk，不变）
//   两个都是 20 位无符号 ⇒ |zd| ≤ 2^20−1，20 位装得下。
wire [20:0] zd   = {1'b0, zk} - {1'b0, zk_tgt};
wire        zneg = zd[20];
wire [19:0] zabs = zneg ? (20'd0 - zd[19:0]) : zd[19:0];
wire [19:0] zst  = zabs >> 2;
wire [19:0] zk_n = (zabs > 20'd3) ? (zneg ? (zk + zst) : (zk - zst)) : zk_tgt;
always @(posedge clk or posedge rst) begin
    if (rst)                zk <= 20'd32768;
    // ★ 差值 <4 时 >>2 会归零 ⇒ 永远差几个 LSB 到不了目标（k 差 → 画面轻微错位）。
    //   这里在小于 4 时直接吸附到目标（见 zk_n）。
    else if (zstep)         zk <= zk_n;
end

assign pos_a  = zoom_g   ? 32'd0              :
                arm_geom ? 32'd0              : fx_pos_a;
assign size_a = zoom_g   ? {16'd640, 16'd480} :
                arm_geom ? {16'd640, 16'd480} : fx_size_a;
// ★★★ KX 与 KY 不相等，别写等比 ★★★
//   565 缩略图是「每字打包 2 像素」：320×240 在 SDRAM 里是 160 字 × 240 行。
//   ps_gen_image 的 k = 源宽/RW，所以满屏时 KX = 160/640 = 0.25、
//   KY = 240/480 = 0.5 —— 上板实测写成 {zk,zk} 时 X 方向整片取不到像素，全黑。
// ★ 888 与 565 的 k 完全不同，必须按相位选：
//     888 高清（zph=0）：源 640×480 ⇒ k = 1.0 = 65536
//     565 缩略图（zph≥1）：源 160 字 ×240 行 ⇒ k = {0.25, 0.5}
//   漏掉这一条会让"刚打开缩放"那段的 888 画面按 4× 取左上角，看着像错位/花屏。
wire [19:0] kx_z = (zph == 2'd0) ? 20'd65536 : {1'b0, zk[19:1]};
wire [19:0] ky_z = (zph == 2'd0) ? 20'd65536 : zk;

assign k_a    = zoom_g   ? {kx_z, ky_z} :
                arm_geom ? {20'd16384, 20'd32768} : fx_k_a;

//==============================================================================
//  ★★ 2026-10-30 [ZOOM-EXACT]：锚点直接进源坐标，删掉整条相位累加器 ★★
//==============================================================================
//  【原来】维护 pgx/pgy（13 位，1/4 逼近）跟踪 PH = a·(k_fit−k)，送到
//    ps_gen_image 里做 mx = lx·kx + PH。代价：两套累加器 + 4 档目标生成
//    + 48 位输出 mux，实测 u_ctrl 233 LUT、u_ta 23 LUT。
//  【现在】同一个式子的恒等变形：
//        sx = (x·kx + PH) >> 16 ，PH = a·(k_fit − k)
//        ⇒ sx = ((x − a)·kx + a·k_fit) >> 16
//    k_fit 是常数（x:16384 / y:32768）⇒ `a·k_fit` 只是**移位**；
//    `x − a` 由 map_src 里一个减法完成（ps_gen_image 的 f_ox/f_oy）。
//    于是整条相位链全删，而且：
//      · 锚点从"±0.25 屏像素的量化近似"变成**逐位精确**；
//      · PH 和 k 不再各自插值 ⇒ 不存在"两者不同步导致锚点漂移"。
//  【下游拿到的是什么】不再是相位，而是**锚点本身**（10 位 × 2）；
//    它的语义是"源坐标原点"，与 k 无关，所以不需要跟着 tick 插值。
//  【门控】必须与 k_a 严格同条件（zph != 0）：zph==2 装载中仍在显示 565，
//    那时若把锚点清 0 就会跳回"锚在左上角"（上板"播完闪一下"）。
//==============================================================================
assign ori_a = (zoom_g && (zph != 2'd0)) ? {za_x, za_y} : 20'd0;

//==============================================================================
//  ★ S_ARM 期间把 B 的几何强制成全屏 —— 让"预热"真的有效
//==============================================================================
//  【为什么】fx0 在 t=0 时 B 位于 X0=640（完全在屏幕右侧外），fx1 时 Y0=480
//  （完全在下边外）。这两个位置 B 的框内一个像素都没有 ⇒ 请求侧无事可做
//  ⇒ warmed 只能靠 rq_scan_end 兜（要扫满 307,200 个屏幕位置，约 6ms）
//  ⇒ **预热基本没起作用**，B 进 TRANS 第一帧还是冷的。
//  上板表现就是"上电后第一次切换特别容易卡"（thumbB 从开机起就没跑过）。
//
//  强制成全屏 (0,0) 640×480 后，B 会真的把整屏像素取一遍，warmed 和 FIFO
//  水位都建起来。alpha 在这期间被压成 0，所以完全看不见。
//  注意：k_b 仍是按 viewer_fx 的真实几何算的（不是全屏的），所以这段时间
//  B 的采样率和画框不完全匹配 —— 反正不可见，到边界就切回真实几何。
// ★ 2026-11-03：S_OSD 也钉成全屏 —— 文件页四宫格画在 B 层（viewer_scene）。
//   四宫格的取址/框判定不看 B 的几何，但 ps_gen_image 的**每帧字数预算**看 f_x0：
//   开机没播过动画时 B 停在 fx0 的起始姿态 X0=−640 ⇒ 预算 = 0 ⇒ B 一个请求都发不出
//   ⇒ 该层永远不就绪 ⇒ 帧卡住（仿真实测）。菜单里 B 平时是关的，钉住没有可见影响。
wire   pin_b  = (state == S_ARM) || (state == S_OSD);
assign pos_b  = pin_b ? 32'd0                : fx_pos_b;
assign size_b = pin_b ? {16'd640, 16'd480}   : fx_size_b;

//==============================================================================
//  5. 主状态机
//==============================================================================
//  ★ 2026-11-04 重写要点（换图/缓存已移到 img_switch，见 §1.5'）：
//    · 不再有 img_idx/tgt_idx/rot/base —— 当前图的缩略图地址 = cur_addr，
//      过渡的旧图 = old_addr，高清的偏移由序列器按 cur_rel 现取。
//    · 装高清 = `hd_want <= 1`（序列器择机收下），不再在各状态里散装 ld_* 赋值。
//    · S_LOW（播完动画、等高清）里也收换图键：正在装的高清直接放弃（ab_req），
//      立刻开始下一次过渡 —— 主页连按不用等高清装完。
//    · 开机 / 文件页打开一张图都走 S_BOOT_T：等当前图缩略图装好，帧边界上屏，
//      再进 S_LOW 装高清；邻居（下一张/上一张）在高清之后才预取。
//==============================================================================
reg busy_d;

// ★ 2026-11-02 [BOOT-EXIT]：S_BOOT_T 的"必然推进"计时器
//   计数到 bit27 置起为止 = 2^27 拍 ≈ 2.68s @50MHz
//   S_BOOT_T 是全设计唯一一个"出口只挂在装载结果上"的状态，装不上就永远不出帧。
//   这里给它一个纯时间的出口：到了就按"低清先上屏"处理，高清交给 S_LOW 重试链。
reg [27:0] boot_to;

// 主页换图：进 S_ARM 播过渡（compose_en 由 S_ARM 相位 0 控制）
task start_anim;
    begin
        t          <= 12'd0;
        arm_cnt    <= 3'd0;
        arm_ph     <= 2'd0;
        low_retry  <= 1'b0;
        compose_en <= 1'b0;
        state      <= S_ARM;
    end
endtask

always @(posedge clk or posedge rst) begin
    if (rst) begin
        state       <= S_BOOT_T;
        t           <= 12'd0;
        fx_id       <= 3'd0;
        dir         <= 1'b1;
        compose_en  <= 1'b0;
        fmt565      <= 1'b1;
        a_hd        <= 1'b0;
        a_en        <= 1'b0;
        a_base      <= THUMB0_ADDR;
        b_en        <= 1'b0;
        b_base      <= THUMB1_ADDR;
        busy_d      <= 1'b0;
        boot_to     <= 28'd0;       // ★ [BOOT-EXIT] 上电从 0 起算
        arm_cnt     <= 3'd0;
        hdf_ph      <= 2'd0;
        arm_ph      <= 2'd0;
        low_retry   <= 1'b0;
        load_fx     <= 1'b0;
        osd_ph      <= 2'd0;
        exit_pend   <= 1'b0;
        jmp_x       <= 1'b0;
        zph         <= 2'd0;
        sw_ok       <= 1'b0;
        sw_wait     <= 2'd0;
        pv_pend     <= 1'b0;
        ld_pend     <= 1'b0;
        osd_on      <= 1'b0;
        hd_kill     <= 1'b0;
        osd_pv      <= 1'b0;
        hd_want     <= 1'b0;
        ab_req      <= 1'b0;
        zoom_abort  <= 1'b0;
    end else begin
        busy_d  <= busy;
        load_fx <= 1'b0;
        // hd_kill / ab_req / zoom_abort：单拍
        hd_kill    <= 1'b0;
        ab_req     <= 1'b0;
        zoom_abort <= 1'b0;
        // ★ 2026-11-02 [FP-ABORT]：这里**不能**用 I_file_page 清 hd_want。
        //   hd_take 已经被 `!I_file_page` 门住了，文件页开着时一单都发不出去；
        //   而清掉 hd_want 会让"过渡正好在文件页开着时结束"这条路上的高清请求
        //   永久丢失（出了文件页也不会再补装，画面永远停在 565）。
        //   ⇒ 只门发单，不动请求本身。
        if (hd_take) hd_want <= 1'b0;
        // ★ 2026-11-05 [HDW-TO]：排队条件长期不满足（相册不 ready / 装载口一直被占）
        //   就把请求丢掉，否则 `hd_want` 永久挂住：缩放卡在 zph=2，预取也被它挡住。
        else if (hdw_to) hd_want <= 1'b0;
        // 每次换图成立：高清槽里不再是当前图；同时记一次"放弃在飞装载"的请求
        //   ★ [ABORT-ANY-STATE]：原来只在 S_LOW 里置 ab_req ⇒ 在别的状态打断
        //     （S_HD 里的预取装载、S_TRANS 收尾那一拍）根本不会发 abort。
        //     现在统一在这里发；是否真的锁存由 abort_req 的 ldp==2 条件决定。
        if (sw_go) begin
            hd_kill <= 1'b1;
            ab_req  <= 1'b1;
        end

        // ★ [BOOT-EXIT]：只有"停在 S_BOOT_T 且还没等到 cur_ok"时才计时，
        //   离开本态或 cur_ok 一到就清零（不会误伤正常路径）
        if ((state != S_BOOT_T) || cur_ok) boot_to <= 28'd0;
        else if (!boot_to[27])             boot_to <= boot_to + 28'd1;

        case (state)
        //------------------------------------------------------------------
        //  开机 / 文件页打开：等当前图缩略图装好，帧边界上屏，然后装高清
        //------------------------------------------------------------------
        S_BOOT_T: begin
            //==================================================================
            //  ★ 2026-11-02 [BOOT-EXIT]：唯一出口加了"必然推进"的兜底。
            //  【为什么】本态原来两个出口都挂在装载结果上（cur_ok / !I_any）：
            //     · any=1 但缩略图没装成 ⇒ cur_ok 恒 0
            //     · alb_ok=0（相册没 ready / bpend 卡住）⇒ 两条分支都不成立
            //    两者都让 compose_en 永远为 0 ⇒ display_valid 恒 0 ⇒ 全黑。
            //  【实测教训：a_en 必须是 1】只把 compose_en 拉高、把 a_en 关掉
            //    （想显示"干净黑底"）**屏幕上仍然全黑**：A 层不使能时纹理层的
            //    warmed/layer_ready 链停在未预热态，pixel_ready 拉不起来 ⇒
            //    frame_writer 收不满 307200 ⇒ display_valid 恒 0。
            //    ⇒ 出帧的前提是"**有一个使能的层**"，不是 compose_en 单独够。
            //    所以这里和能亮的那版一样，a_en 一并置 1。
            //  【超时 ≈2.7s】等不到 cur_ok 就先上屏（低清＝当前槽的全屏
            //    缩略图；没装好就是那块 SDRAM 的内容），高清交给 S_LOW 的重试链。
            //==================================================================
            if (alb_ok && !I_any) begin
                // 空目录（或相册一格图都没扫出来）：不装图，但**必须继续出帧**
                // ★ 同一条教训：a_en 必须为 1，否则 pixel_ready 拉不起来、整屏黑、
                //   OSD 也看不见（S_STUCK 的另一个入口从 S_LOW 进来时 a_en 本来就是 1）。
                a_en       <= 1'b1;
                b_en       <= 1'b0;
                a_hd       <= 1'b0;
                a_base     <= cur_addr;
                fmt565     <= 1'b1;
                compose_en <= 1'b1;
                state      <= S_STUCK;
            end else if (cur_ok && (tick || !compose_en)) begin
                // 开机时还没出帧（compose_en=0）⇒ 直接切；
                // 从菜单打开图片时屏幕上正在出旧图的 565 ⇒ 等帧边界再换源
                compose_en <= 1'b1;
                a_en       <= 1'b1;
                a_hd       <= 1'b0;
                a_base     <= cur_addr;
                b_en       <= 1'b0;
                fmt565     <= 1'b1;
                hd_want    <= 1'b1;
                low_retry  <= 1'b0;
                state      <= S_LOW;
            end else if (boot_to[27]) begin
                // ★ 兜底：等不到 cur_ok 也先上屏（这一步与之前"能亮"的版本逐字一致）
                compose_en <= 1'b1;
                a_en       <= 1'b1;
                a_hd       <= 1'b0;
                a_base     <= cur_addr;
                b_en       <= 1'b0;
                fmt565     <= 1'b1;
                hd_want    <= 1'b1;
                low_retry  <= 1'b0;
                state      <= S_LOW;
            end
        end

        //------------------------------------------------------------------
        //  低清等待：目标缩略图全屏，等高清装完（帧照常出，tick 才会来）
        //------------------------------------------------------------------
        S_LOW: begin
            // ★ B 层必须在这里关掉（从 S_TRANS 看门狗降级过来时 B 还是开的）
            b_en <= 1'b0;
            if (sw_go) begin
                // ★ 2026-11-04：装高清途中又切图 —— 不等，放弃这次高清，直接开始下一次过渡。
                //   画面上此刻是当前图的缩略图（= 下一次过渡的 A），无缝。
                //   ★ abort 请求已在公共区（`if (sw_go)`）统一发出，这里不再重复置 ab_req。
                hd_want <= 1'b0;
                start_anim();
            end else if (hd_ready && tick) begin
                a_hd       <= 1'b1;
                a_base     <= HD_ADDR;
                fmt565     <= 1'b0;
                compose_en <= 1'b1;
                hdf_ph     <= 2'd0;
                low_retry  <= 1'b0;
                state      <= S_HDF;
            end else if (key_osd_enter) begin
                // ★ 2026-11-02 [OSD-EARLY]：等高清这 4s 里也能开菜单。
                //   注意**不 abort** 这次装载：它只写 HD_ADDR，跟菜单显示无关，
                //   让它继续跑 —— 退出菜单时 hd_ready 往往已经为 1，直接切 888，
                //   比"进菜单先放弃、出菜单再重装 4s"好得多。
                //   （真进了文件页才需要放弃，那条在序列器的 I_file_page 里。）
                compose_en <= 1'b0;
                osd_ph     <= 2'd0;
                exit_pend  <= 1'b0;
                osd_pv     <= 1'b0;
                state      <= S_OSD;
            end else if (zoom_on) begin
                // ★ 2026-11-02 [ZOOM-EARLY]：等高清这 4s 里也能进缩放 OSD。
                //   进 **zph=1（565 预览）**，不是 zph=0：
                //     · zph==0 会无条件 a_hd<=1 / a_base<=HD_ADDR / fmt565<=0 ⇒
                //       把还没装完的画布当高清显示，花屏（[ZOOM-NOWRIP] 同一类错）；
                //     · zph==1 什么都不改（当前就是 a_hd=0 / cur_addr / 565），
                //       只把几何源从 arm_geom 换成 zoom_g —— 两者在 1× 时逐位相同
                //       （pos(0,0)、size 640×480、k={16384,32768}），所以进场那一帧
                //       不会有任何跳变，只是十字条出现。
                //   **不 abort** 在飞的那次装载：它就是当前图的高清，让它继续跑；
                //   用户真按了缩放才会在 zph==1 里 hd_kill 掉它重装（见 S_ZOOM）。
                compose_en  <= 1'b1;
                zph         <= 2'd1;
                state       <= S_ZOOM;
            end else if ((ld_fail_st || low_stall1) && !low_retry) begin
                // 第一次失败/超时：重试一次（低清继续出帧，画面不冻）
                low_retry  <= 1'b1;
                hd_want    <= 1'b1;
                compose_en <= 1'b1;
            end else if (low_stall2 && low_retry) begin
                // 重试也等不到：认输，保持低清、恢复按键
                state <= S_STUCK;
            end
        end

        //------------------------------------------------------------------
        //  ★ S_ARM：过渡前的备战（切源 → 预热 B → 等一帧让 fx 几何生效）
        //------------------------------------------------------------------
        S_ARM: begin
            if (arm_ph == 2'd0) begin
                compose_en <= 1'b0;              // 等最后一帧排空
                if (!busy) begin
                    a_hd       <= 1'b0;          // A：旧图缩略图（放大）= 动画第一帧
                    a_en       <= 1'b1;
                    a_base     <= old_addr;
                    b_en       <= 1'b1;          // B：新图缩略图（alpha 为 0，先看不见）
                    b_base     <= cur_addr;
                    fmt565     <= 1'b1;
                    dir        <= sw_dir;
                    t          <= 12'd0;
                    load_fx    <= 1'b1;
                    arm_ph     <= 2'd1;
                    arm_cnt    <= 3'd0;
                    compose_en <= 1'b1;
                end
            end else if (arm_ph == 2'd1) begin
                if (tick || arm_stall) begin
                    if ((arm_cnt >= ARM_FRAMES) || arm_stall) begin
                        t          <= 12'd0;
                        load_fx    <= 1'b1;
                        compose_en <= 1'b1;
                        arm_ph     <= 2'd2;
                    end else begin
                        arm_cnt <= arm_cnt + 3'd1;
                    end
                end
            end else begin
                // 相位 2：等一帧，让 viewer_fx 的 cfg_apply（120 拍）完成
                if (tick || arm_stall) begin
                    arm_ph <= 2'd0;
                    state  <= S_TRANS;
                end
            end
        end

        //------------------------------------------------------------------
        //  过渡：双缩略图，t = 0..t_end_cur
        //------------------------------------------------------------------
        S_TRANS: begin
            if (trans_stall || (tick && (t >= t_end_cur) && geom_ok)) begin
                // 播完（或纹理通路卡住 2.68s）：落到新图缩略图满屏，开始装高清。
                //   S_LOW 的画面（A = 新图满屏、B 关）与动画最后一帧逐像素相同。
                a_en       <= 1'b1;
                a_hd       <= 1'b0;
                a_base     <= cur_addr;
                b_en       <= 1'b0;
                fmt565     <= 1'b1;
                compose_en <= 1'b1;
                low_retry  <= 1'b0;
                if (hd_ready) begin
                    a_hd   <= 1'b1;
                    a_base <= HD_ADDR;
                    fmt565 <= 1'b0;
                    hdf_ph <= 2'd0;
                    state  <= S_HDF;
                end else begin
                    hd_want <= 1'b1;
                    state   <= S_LOW;
                end
            end else if (tick) begin
                // ★ 夹紧到终点：保证终点帧用的是精确的 t_end_cur 几何（见 2026-10-27）
                t <= (tnext > t_end_cur) ? t_end_cur : tnext[11:0];
            end
        end

        //------------------------------------------------------------------
        //  ★ S_HDF：等"本次切换之后新合成的那一帧"提交（三相握手）
        //------------------------------------------------------------------
        S_HDF: begin
            // ★ [HDF-KEEP3]：保持三相 —— 这里**不能**简化成"等一次 busy 下降沿"：
            //   ph0 的退出条件就是 `!busy`，所以进入 ph1 的第一拍 `busy_d` 必然是 1
            //   ⇒ 两相版会把这个"上一帧的下降沿"当成"本帧提交"，提前跳 S_HD，
            //     把没写好的画布当 888 推上去（tb_zoom_flow 的 T5 就是这样失配的）。
            //   ph1 必须真的看到**新的** busy 上升，ph2 再等它落下。
            if (hdf_ph == 2'd0) begin
                if (!busy) hdf_ph <= 2'd1;
            end else if (hdf_ph == 2'd1) begin
                if (busy) hdf_ph <= 2'd2;
            end else begin
                if (busy_d && !busy) begin
                    hdf_ph      <= 2'd0;
                    arm_ph      <= 2'd0;
                    low_retry   <= 1'b0;
                    compose_en  <= 1'b0;        // 停帧：显示端保持这块缓冲
                    osd_ph      <= 2'd0;
                    state       <= S_HD;
                end
            end
        end

        //------------------------------------------------------------------
        //  ★ OSD 菜单：565 连续合成（A = 当前图缩略图，B 层关，OSD 走 B 通路）
        //------------------------------------------------------------------
        S_OSD: begin
            if (key_osd_exit) exit_pend <= 1'b1;
            case (osd_ph)
                2'd0: begin
                    compose_en <= 1'b0;                 // 先让旧帧排空
                    jmp_x      <= 1'b0;
                    if (!busy) begin
                        a_hd       <= 1'b0;
                        a_en       <= 1'b1;
                        a_base     <= cur_addr;
                        b_en       <= 1'b0;
                        fmt565     <= 1'b1;
                        osd_on     <= 1'b1;
                        osd_ph     <= 2'd1;
                        compose_en <= 1'b1;
                    end
                end
                2'd1: begin
                    compose_en <= 1'b1;
                    // ★ 菜单里换图**不播动画**：相册页按钮 ⇒ 帧边界直接换 a_base；
                    //   文件页打开一张 BMP ⇒ 关菜单，S_BOOT_T 等它的缩略图装好再上屏。
                    if (sw_go) begin
                        if (ev_jp) begin
                            jmp_x  <= 1'b1;
                            osd_ph <= 2'd2;
                        end else
                            osd_pv <= 1'b1;
                    end
                    // 帧边界提交（门控 osd_pv：与 sw_go 同拍时下一个 tick 才落地）
                    if (tick && osd_pv) begin
                        a_base <= cur_addr;
                        osd_pv <= 1'b0;
                    end
                    if (key_osd_exit || exit_pend) osd_ph <= 2'd2;
                end
                default: begin
                    if (jmp_x) begin
                        // 文件页打开：帧照常出（旧图 565），帧边界撤菜单 → S_BOOT_T
                        osd_pv <= 1'b0;
                        if (tick) begin
                            osd_on    <= 1'b0;
                            osd_ph    <= 2'd0;
                            exit_pend <= 1'b0;
                            jmp_x     <= 1'b0;
                            state     <= S_BOOT_T;
                        end
                    end else begin
                        compose_en <= 1'b0;
                        if (!busy) begin
                            a_en      <= 1'b1;
                            osd_on    <= 1'b0;
                            b_en      <= 1'b0;
                            osd_ph    <= 2'd0;
                            exit_pend <= 1'b0;
                            osd_pv    <= 1'b0;
                            // ★ 2026-11-02 [OSD-EXIT-SAFE]：**统一走 S_LOW**，不再在这里直切 888。
                            //   【为什么】原来 hd_ready 为真时直接进 S_HDF，而 S_HDF 的三相握手
                            //   （等 busy 抬 → 等 busy 落）是这条链里**唯一没有超时**的等待。
                            //   关菜单那一拍若正撞上四宫格在装 160×120（quad_loader 正在写槽），
                            //   新帧请求可能卡在 frame_writer 的 buf_free 上打不出来 ⇒ busy 不再抬
                            //   ⇒ 停在 S_HDF：屏幕定格在"菜单已撤、四宫格还在、第一张只装了半张"
                            //   的那一帧上，按键也不响应。
                            //   S_LOW 是这条链里唯一"必然往前走"的路：照常出 565 帧，下一个 tick
                            //   看到 hd_ready 就升回 S_HDF；装不到还有 low_stall1/2 兜底退到 S_STUCK。
                            //   代价只有 1 帧 565（16ms，同一张图，肉眼无感）。
                            a_hd       <= 1'b0;
                            a_base     <= cur_addr;
                            fmt565     <= 1'b1;
                            hd_want    <= ~hd_ready;   // 已经在 888 上就别白装一次
                            low_retry  <= 1'b0;
                            compose_en <= 1'b1;
                            state      <= S_LOW;
                        end
                    end
                end
            endcase
        end

        //------------------------------------------------------------------
        //  ★ 2026-10-30  S_ZOOM：缩放 OSD（见 [ZOOM-SWAP]/[ZOOM-NOWRIP]）
        //------------------------------------------------------------------
        S_ZOOM: begin
            compose_en <= 1'b1;
            if (preview_req) pv_pend <= 1'b1;
            if (load_req)    ld_pend <= 1'b1;
            if (pv_pend && tick && (zph == 2'd0)) pv_pend <= 1'b0;
            if (ld_pend && (zph == 2'd1))         ld_pend <= 1'b0;
            // （原 [ZOOM-EARLY] 的 z_from_low 已删除：退出时"档位=1× 且插值收尾
            //   ⇒ 直接回 S_LOW、不重装"这条已由下面的 [ZOOM-EXIT-FAST] 统一覆盖，
            //   连"从 S_LOW 进来又原样退出"也算在内，省 1 个 FF。）

            if (!zoom_on) begin
                // ★★ 2026-11-02 [ZOOM-EXIT-FAST]：退出缩放不再"等这次烘焙" ★★
                //   【上板现象】缩放到 2× → 关掉缩放 OSD → 再按翻页 / 开菜单：
                //     [ZOOM-RESTORE] 会先把档位动画降回 1×，然后**等一整次 1× 烘焙**
                //     （12MP 源 ~4s）才把动作放出去 ⇒ "缩放动画结束后还要等装高清"。
                //   【本质】`zph==1/2` 时画面本来就是 565 预览（a_hd=0 / fmt565=1），
                //     它和 S_LOW 的画面**逐像素相同**；而 1× 的烘焙只是为了让画面
                //     变清晰，跟"能不能接受下一个按键"没有任何关系。
                //   【现在】只要档位已经回到 1×（zoom_val==0x10）且 zk 插值收尾
                //     （zk==zk_tgt ⇒ 恢复动画真的播完了），就直接落到 S_LOW：
                //       · 照常出 565 帧，画面不跳（同一张图、同一几何）；
                //       · 高清在后台继续装，hd_ready 一到就升 888；
                //       · S_LOW 本来就是"按键全活"的状态（翻页/菜单/缩放都行）
                //         ⇒ [ZOOM-RESTORE] 暂存的那个动作当拍就能放行。
                //   ⚠ 只有 1× 才能这么走：>1× 时 S_LOW 的满屏几何会把放大裁切
                //     的 565 当 1× 铺开（画面整个错位），那条仍走下面的"补装再退"。
                //   ⚠ 也不能在这里 hd_kill：正在装的可能就是 1× 那一单。
                if ((zoom_val == 8'h10) && (zk == zk_tgt) && (zph != 2'd0)) begin
                    a_hd       <= 1'b0;
                    a_base     <= cur_addr;
                    fmt565     <= 1'b1;
                    // 正在装的那单不是"当前图 + 1×"⇒ 直接砍了，别让它再占着 SD
                    // （否则用户下一步（翻页/选图）的缩略图要排在它后面）
                    zoom_abort <= (ldp != 2'd0) && ld_fmt_r && ~hd_pend;
                    hd_want    <= ~hd_ready & ~hd_pend;   // 没有一单在路上才补发
                    sw_ok      <= 1'b0;
                    pv_pend    <= 1'b0;                   // 恢复会话的预览请求已无用
                    ld_pend    <= 1'b0;
                    low_retry  <= 1'b0;
                    compose_en <= 1'b1;
                    state      <= S_LOW;
                end else if (pv_pend && (zph == 2'd0)) begin
                    // ★★ 2026-11-02 [ZOOM-EXIT-FAST] 恢复动画的**起点**：888 → 565 ★★
                    //   【上板现象】"关了缩放 OSD 再按翻页/开菜单，动画没了，直接跳过去"。
                    //   【根因】恢复请求 preview_req 是在 OSD 关掉**之后**才到的（Z_HD
                    //     收到 act_code 才发），而这时 viewer_ctrl 已经因为 zoom_on=0
                    //     走下面的 `zph==0 → S_HDF` 静止出口 —— 那一拍就跳到 S_HD，
                    //     zk 一次都没走，等于把"降回 1×"那段动画整段跳过。
                    //     （改之前不会露这个洞：Z_LOAD 会一直等到烘焙完，zoom_on 还
                    //       是 1，pv_pend 走的是正常那条 `pv_pend && tick`。）
                    //   【现在】只要还有 preview 请求，**就在这里等帧边界**（不能漏到
                    //     下面那个 S_HDF 出口去），到 tick 切到 565 —— 和正常缩放时
                    //     那条路径逐字相同，zk 才有画面可插值，动画照播。
                    //   ⚠ 必须等 tick：a_hd / fmt565 / a_base 三个一起翻，帧内翻就是闪黑。
                    if (tick) begin
                        a_hd    <= 1'b0;
                        a_base  <= cur_addr;
                        fmt565  <= 1'b1;
                        pv_pend <= 1'b0;
                        zph     <= 2'd1;
                    end
                end else if (zph == 2'd0) begin
                    // 先补一帧（十字条已撤），等它提交再停帧
                    hdf_ph     <= 2'd0;
                    compose_en <= 1'b1;
                    state      <= S_HDF;
                end else if (zph == 2'd1) begin
                    // 565 预览中退出且档位还 >1×：把当前 zoom 补装一次，装完 zph 回 0 再退
                    hd_kill <= 1'b1;
                    hd_want <= 1'b1;
                    sw_ok   <= 1'b0;
                    zph     <= 2'd2;
                end else if (hdw_to) begin
                    // ★ [HDW-TO] 补装请求收不下（相册不 ready / 装载口被占）：别再等，
                    //   补一帧后照常退出（画面停在当前 565，至少不会把状态机卡死）。
                    hdf_ph     <= 2'd0;
                    compose_en <= 1'b1;
                    state      <= S_HDF;
                end else if (ld_fin && !hd_want) begin
                    sw_ok   <= 1'b1;
                    sw_wait <= hd_ready ? SWT : 2'd0;   // 失败则不等待，直接回 0
                end
            end else begin
                case (zph)
                2'd0: begin
                    a_hd   <= 1'b1;
                    a_en   <= 1'b1;
                    a_base <= HD_ADDR;
                    b_en   <= 1'b0;
                    fmt565 <= 1'b0;
                    if (pv_pend && tick) begin
                        a_hd   <= 1'b0;
                        a_base <= cur_addr;
                        fmt565 <= 1'b1;
                        zph    <= 2'd1;
                    end
                    // ★ 2026-11-02 [ZOOM-RESTEP]：还有一步没消化（键按在"刚提交完"那一拍）
                    //   ⇒ 直接回预览态，由 zph==1 去起新的一单。没有这一条，ld_pend 会
                    //   在 zph==0 里永远挂着（只有 zph==1 才消费它）⇒ 那次缩放丢失。
                    //   ⚠ 必须同时把 a_hd 拉回 0：否则 zph==1/2 期间画布还在 888 上，
                    //     新的烘焙会往正在显示的那块写 ⇒ 撕裂。
                    //   ⚠⚠ 必须等 `tick`（帧边界）。zoom_osd 的 Z_PRE 是**当拍**发 load_req 的，
                    //     所以这条一定比 pv_pend 那条更早满足；不门 tick 就会在**扫描中途**
                    //     改 a_hd/a_base/fmt565 —— 当前帧一半 888 一半 565，上板表现正是
                    //     "按放大先闪一下黑，然后动画才开始"。这是 [ZOOM-NOWRIP] 同一类错。
                    else if (ld_pend && tick) begin
                        a_hd    <= 1'b0;
                        a_base  <= cur_addr;
                        fmt565  <= 1'b1;
                        pv_pend <= 1'b0;
                        zph     <= 2'd1;
                    end
                end
                2'd1: begin
                    if (ld_pend) begin
                        // ★ 必须作废高清槽（否则 hd_ready 仍为真，切回旧 zoom 的画面）
                        hd_kill <= 1'b1;
                        hd_want <= 1'b1;
                        sw_ok   <= 1'b0;
                        zph     <= 2'd2;
                    end
                end
                default: begin
                    // 装载中：继续出 565；用 ld_fin 做门控（不是 hd_ready）——
                    //   进 zph=2 那一拍 hd_img 还是上一次装载的，hd_ready 可能仍为真。
                    //   ★ 还原 77c748c：缩放期间已禁预取（见 img_switch 的 I_pf_en），
                    //     再加 `!hd_want`（请求还没被收下之前，ld_fin 只可能是别的那次
                    //     装载在收尾）⇒ 这里的 ld_fin 一定是高清装载结束。
                    // ★★ 2026-11-02 [ZOOM-RESTEP]：装载中又按了缩放 ⇒ **砍掉这次烘焙** ★★
                    //   【原来】zoom_osd 在 Z_LOAD 里只记 1 步、且要等 ld_done 才执行 ⇒
                    //     1→2→4→8 连按 3 下要串行跑 3 次整幅 888 烘焙（大图 4s/次 = 12s），
                    //     而且每次都先用旧档位装完再装新的。
                    //   【现在】新的一步一进来就把飞着的那次放弃，**原地**用最新档位重发：
                    //     · 画面本来就停在 565（zph==2），不需要回 zph==1；
                    //     · `hd_want<=1` 让装载器一空下来就用新的 hd_zoom_s 起单
                    //       （hd_zoom_s 在 hd_take 那拍从 zoom_val 现取，所以一定是新档位）；
                    //     · `ld_pend<=0` 是关键：不清的话 zph==1 会再发一单，白装一次。
                    //     中间的档位本来就不会被看到 ⇒ N 次按键只装 1 次。
                    //   ⚠ 绝不能进 S_HDF —— 那边会无条件 `a_hd<=1`，把只写了一半的
                    //     HD 画布当高清显示出来。
                    if (hdw_to) begin
                        // ★ [HDW-TO] 请求一直收不下：退回 565 预览（k 仍是缩放值，
                        //   画面不会错位，只是模糊），下一次按键会重新发请求。
                        zph <= 2'd1;
                    end else if (ld_pend) begin
                        zoom_abort <= 1'b1;     // 让序列器砍掉在飞的那次烘焙
                        hd_kill    <= 1'b1;     // 画布要重写 ⇒ 旧内容作废
                        hd_want    <= 1'b1;     // 用新档位重发（下一拍就会被 hd_take 收下）
                        sw_ok      <= 1'b0;
                        ld_pend    <= 1'b0;     // 已消化，别再经 zph==1 发第二单
                        pv_pend    <= 1'b0;
                    end else if (ld_fin && !hd_want) begin
                        sw_ok   <= 1'b1;
                        sw_wait <= hd_ready ? SWT : 2'd0;
                    end
                end
                endcase
            end

            // ★★ [ZOOM-SWAP] 切回 888 的唯一提交点（帧边界，带 [ZOOM-NOWRIP] 兜底）
            if (sw_ok && sw_point) begin
                if ((sw_wait != 2'd0) && tick) sw_wait <= sw_wait - 2'd1;
                else if (hd_ready) begin
                    zph    <= 2'd0;
                    a_hd   <= 1'b1;
                    a_base <= HD_ADDR;
                    fmt565 <= 1'b0;
                    sw_ok  <= 1'b0;
                end else begin
                    // ★ [ZOOM-MATCH] 兜底：这一单装完了，但画布不是"当前档位"的
                    //   （烘焙途中档位变了 / 被 abort 掉的那单收尾）。原来这里照样
                    //   把 zph 推回 0 ⇒ 状态机以为 888 好了，实际是半张画布。
                    //   现在落 S_LOW：565 满屏 + 后台重装，按键全活。
                    a_hd       <= 1'b0;
                    a_base     <= cur_addr;
                    fmt565     <= 1'b1;
                    hd_want    <= 1'b1;
                    low_retry  <= 1'b0;
                    compose_en <= 1'b1;
                    sw_ok      <= 1'b0;
                    state      <= S_LOW;
                end
            end
        end

        //------------------------------------------------------------------
        //  高清静止 / 降级：响应按键
        //------------------------------------------------------------------
        S_HD, S_STUCK: begin
            // fx_id 在静止态每拍跟随菜单设置（过渡期间锁死）
            fx_id <= fx_sel;

            if (zoom_on) begin
                compose_en <= 1'b1;
                zph        <= 2'd0;
                state      <= S_ZOOM;
            end else if (key_osd_enter) begin
                compose_en <= 1'b0;
                osd_ph     <= 2'd0;
                exit_pend  <= 1'b0;
                osd_pv     <= 1'b0;
                state      <= S_OSD;
            end else if (sw_go) begin
                start_anim();
            end else if (inf_refresh) begin
                // 信息条内容变了：补画一帧 888
                hdf_ph     <= 2'd0;
                compose_en <= 1'b1;
                state      <= S_HDF;
            end
        end

        default: state <= S_BOOT_T;
        endcase
    end
end



//==============================================================================
//  ★★ 占空比"整段最小值"保持 —— 已随 2026-10-26 数码管清理删除 ★★
//==============================================================================
//  【原功能】进入 S_TRANS 后只往下记最差帧时间，出过渡复位成 F，
//    让 SEL3 在整个动画期间显示一个稳定的数（决定观感的那个下限）。
//  【为什么删】数码管现在只保留 帧率/图片号/模式/装载进度 四项，
//    帧时间读数已无消费者；viewer_scene 的 duty 输出、本模块的
//    duty_in/duty_dbg 端口和 fr_cyc/fr_busy_d 一并移除。
//  【读法留档】C=75%（被配额限满，8.2ms/帧，正常）  8=50%  6=37%
//               3=19%（32ms，严重）   F=100% 或非过渡期
//==============================================================================

assign st_img   = cur_abs[1:0];
assign st_fx    = fx_id;
// ★ st_state 端口保留：顶层数码管已不显示它（2026-10-26 清理），
//   但 tb_ctrl 回归依赖它观测状态机 —— 删端口会连带删测试台，先留着。
assign st_state = state;

//==============================================================================
//  ★★ 2026-10-23：帧率计（1 秒窗口）—— 给数码管最左位 ★★
//==============================================================================
//  【为什么要它】"过渡看着像掉帧"是主观描述，必须先量化：
//      60 ⇒ 正常   30 ⇒ 掉一半   10 ⇒ 严重   0 ⇒ 一帧都没有
//  【怎么数】一次"帧完成"= frame_writer 的 busy 下降沿（和 tick 同一个源），
//    在 50MHz 上开 1 秒窗口（50,000,000 拍），窗口结束时把计数锁存。
//  【为什么显示 /10】数码管只给一位。十进制：6=60fps、3=30fps、1=10fps；
//    ≥100 显示 F（不该出现）。
//  【静止 HD 页显示 0 是正常的】那里 compose_en=0，本来就不产帧。
//  【注意】除数 10 综合成一个小查找链（8 位），面积可忽略。
//==============================================================================
//==============================================================================
//  ★★ 2026-10-27：帧率计重写 —— 用 BCD 计数器替代 8 位除法器 ★★
//==============================================================================
//  【为什么要重写】上一版是 `fps_tens = fps_cnt / 8'd10`，TD 会生成一个
//    8 位除法器（约 30~50 LUT，还要多级进位），再配 8 位 fps_cnt + 8 位
//    fps_val（后者**没有任何消费者**）。这三块加起来是本模块最白烧的一块。
//  【为什么现在必须省】2026-10-27 恢复数码管读数页之后，这条链从"被优化掉"
//    变回"活的"（`fps_dbg` 重新有了消费者），mslice 因此涨上去、编不过。
//  【改法】不做任何除法：直接用一个 4 位个位 + 一个 4 位十位的**十进制计数器**，
//    每来一个 frame_done 就 +1（个位满 9 进位到十位，十位饱和在 15）。
//    窗口结束时把**十位**直接送给数码管 —— 它天生就是 "fps/10"。
//    · 省掉：8 位除法器、fps_cnt(8 FF)、fps_val(8 FF)
//    · 代价：fps_one(4 FF)、fps_ten(4 FF)
//    · 净省约 8 FF + 一整个除法器，且没有多级进位链。
//  【读法不变】6≈60fps  3≈30fps  1≈10fps  0=<10fps   F=≥150（不该出现）
//==============================================================================
reg [25:0] fps_win;         // 0 .. 49_999_999 @50MHz = 1s
reg [3:0]  fps_one;         // 本窗口已过帧数的个位 0..9
reg [3:0]  fps_ten;         // 本窗口已过帧数的十位 0..15（饱和）

wire       frame_done = busy_d & ~busy;                       // 一帧完成，一个脉冲

always @(posedge clk or posedge rst) begin
    if (rst) begin
        fps_win <= 26'd0;
        fps_one <= 4'd0;
        fps_ten <= 4'd0;
        fps_dbg <= 4'd0;
    end else if (fps_win == 26'd49_999_999) begin
        fps_win <= 26'd0;
        fps_dbg <= fps_ten;          // 十位 = fps/10，直接上数码管
        fps_one <= 4'd0;
        fps_ten <= 4'd0;
    end else begin
        fps_win <= fps_win + 26'd1;
        if (frame_done) begin
            if (fps_one == 4'd9) begin
                fps_one <= 4'd0;
                if (fps_ten != 4'd15) fps_ten <= fps_ten + 4'd1;   // 饱和，不回绕
            end else
                fps_one <= fps_one + 4'd1;
        end
    end
end

endmodule
