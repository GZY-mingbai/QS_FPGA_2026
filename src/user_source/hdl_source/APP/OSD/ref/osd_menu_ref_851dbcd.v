//==============================================================================
//  ★★★ 冻结参考模型：仅供 tb_equiv_menu_geom.v 做逐位等价比对，不要修改 ★★★
//==============================================================================
//  来源：commit 851dbcd 的 src/user_source/hdl_source/APP/OSD/osd_menu.v
//  生成方式（模块名改成 osd_menu_ref）：
//      git show 851dbcd:src/user_source/hdl_source/APP/OSD/osd_menu.v \
//        | sed 's/^module osd_menu /module osd_menu_ref /' > ref/osd_menu_ref_851dbcd.v
//  用途：LUT 释放重构（[LUT-1] 起）每一步都拿它跟当前 osd_menu 并列跑，
//        逐位比对 9 个动态槽写进 cellmem 的 (geo, attr) 以及文字/导出量。
//  注意：这是**历史快照**，不是活代码；工程不编译它（默认 flow 只读
//        src/user_source/hdl_source 下的文件，本文件在 OSD/ref 子目录且顶层无引用）。
//==============================================================================
//  osd_menu.v —— 多级菜单：按键 → 焦点树 → 动画 → 布局（控件表 + 文字串）
//==============================================================================
//
//  【它负责什么】每帧一次（帧间静默窗口）：
//    ① 读键并更新焦点状态（NAV/PAGE、页、行、编辑态、每页独立的值）
//    ② 推进 6 条动画通道（游标 x/y/w/h、滚动、数值）
//    ③ 按动画后的坐标重排 18 条控件记录（绝对屏幕坐标）
//    ④ 状态一变就重写文字串（6 行 × MAXLEN 字符）并给字体缓存一个 trigger
//
//  【动画为什么必须"每帧一步、目标一变就归零"】
//  每个通道三个寄存器：_f 进度(0..65535)、_t 上拍目标(用于检测目标变化)、
//  _c 当前显示值。三者缺一不可：
//      · 目标变 → _f 归零 → 从头插值（否则 _f 早饱和，永远直接跳到目标）
//      · 每帧一步 → _f 递增（否则一步不动）
//      · _c = tween(_c, _t, _f) → 平滑逼近
//  ★ 2026-10-29：tween 不再是函数，乘积落在一个显式 EG_PHY_MULT18 上（16×9）。
//  原来的写法 `{9'd0,(to-from)} * {16'd0,e}` 会被按 **25×25** 综合 ⇒ 4 个 DSP；
//  现在 16 位 × 9 位 = 1 个 DSP，算术逐位不变。详见 §0 末尾与 §4 的说明。
//  另外：乘积**不要**写成 from + ((to-from)*e)>>8 —— 上下文位宽会把积截成 16 位
//  （(to-from) ≤ 640、e ≤ 256 ⇒ 积 163840），必须落到 25 位中间量。
//
//  【游标 y 为什么要单独一条通道】
//  游标的主运动是上下换行（80px），x/w/h 只在 NAV↔PAGE 切换时变。
//  早期版本 cy_c <= tgt_cy（直接跟随）⇒ 上板"游标上下跳变 = 没有动画"。
//  现在 cy 通道插值"选中行在窗口里的位置"，游标 y = cy_c-6。
//
//  【下拉弹层】edit && 当前行是 ComboBox 时（紧凑列表，行距 34 / 行高 30）：
//    · cell4   → 不透明 Panel 底板（x 312..624，左右各 8px、上下各 6px）
//    · cell15..17 → 三个选项行，**按当前选中项锚定**：
//                    pop_y = combo_y - pop_sel*34 - 3
//                    于是选中项永远压在 ComboBox 那一行（和桌面系统一致）；
//                    选中第 0 项时整体下移、第 2 项时整体上移
//    · 圆角外侧用 gpm=2 继承"弹层底板 161616"（OSD 每像素只合成一层，
//      继承色填错就会在四个角透出底图 —— 截图里那圈发绿的缺口）
//    · ★ 三项选项字的落位（"一行都不能少"的关键）：
//        ① **选中项**压在 ComboBox 那一行，写在该行第 14 列（x=336）⇒
//           标签照旧留在第 0..3 列，于是读作"色温 中"，不会认错当前值；
//        ② 另外两项写到第 6、7 行 —— 这两行是 NL 6→8 新增的**备用行**，
//           没有标签也没有控件，借用它们不会让任何菜单行消失
//           （pop_sel=0 → 都在下方；=1 → 一上一下；=2 → 都在上方）；
//        ③ 行控件 9..13 只有**被面板盖住的那 1~2 行**不画，其余照常。
//           被盖住几行由 pop_wx0/pop_wx1 给出（与 pop_sel 有关）。
//    · 选中态只由选项行的蓝色底表达
//
//  【布局常量】标题 (112,24)；可视 5 行，行距 80，首行 y=64；
//  控件区 x=320 w=296（320..616）；左栏图标 32×32 @x=32，第 k 个 y=72+80k；
//  ★ 2026-10-28：左栏顶部插入 logo 64×41 @(16,8)，导航栏整体下移 48px。
//  文字 16px/格：标签占 0..3 格（112..176），数值占 9..12 格（256..320），
//  控件内值占第 14 格（下拉，336）或第 16 格（勾选的"开/关"，368）。
//
//  【按键】KEY1 开/关 OSD（导航栏按下即关）+ 返回；KEY2 进入/确认；
//          KEY3 下；KEY4 上。**没有长按退出**（2026-10-26 按用户要求取消）。
//==============================================================================
module osd_menu_ref #(
    parameter integer NL     = 6,
    // ★ 2026-10-26：16 → 32。显示页"风格"下拉收起态要显示完整选项串
    //   （最多 16 字，占第 14..29 列），每行字格数必须 ≥30。
    //   osd_font_cache / ps_gen_text 全部按参数推导（NL8×32=256 恰为 2 的幂），
    //   只需同步改 osd_top / App 例化处的传参和本文件的 LASTC / ctl_len。
    parameter integer MAXLEN = 32,
    // ★ 自检：1 = 把 cell0 画成全屏纯红。上板看屏幕变红 ⇒ 写表/命中/合成整条通路都通。
    parameter integer SELFTEST = 0,
    // ★ 长按连发阈值（单位 = 60Hz tick，local_tick）：板上默认 30/5 =
    //   按住 500ms 出第一发、之后每 83ms 一发。仿真可改小加速。
    //   【注意】RPT_FIRST 必须 < 64（rpt_cnt 只有 6 位）且 > RPT_EVERY。
    parameter integer RPT_FIRST = 30,
    parameter integer RPT_EVERY = 5
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        tick,
    input  wire        ps_start,        // 帧间空隙判据（布局只在 ps_start==0 时启动）
    input  wire        menu_on,
    // ★ 2026-10-30 缩放 OSD：复用槽 16（圆角面板）+ 文字行 6/7
    input  wire        zoom_on,
    input  wire [7:0]  zoom_val,        // Q4.4
    input  wire [1:0]  zoom_mode,
    input  wire        osd_allow,

    input  wire [3:0]  key_press,
    input  wire [3:0]  key_level,

    output reg         enter_req,
    output reg         exit_req,
    output reg  [7:0]  fade,

    output reg         wr_en,
    output reg  [4:0]  wr_idx,
    output reg  [38:0] wr_geo,
    output reg  [23:0] wr_attr,         // {type,style,rad(4+4+4), mask[1:0], v0[9:0]}

    // ★ 2026-10-27：位宽收窄（原来每行 32 位，实际用不到）
    //   txt_xy : 每行 {Y0[10:0], X0[10:0]} = 22 位。11 位而不是 10 位是**算过的**：
    //            y 在滚动动画中可到 616（scroll_row=3、sc_px 还没跟上时），
    //            在未滚动时又可为 −184（sc_px 大于 srow_px）⇒ 需要 −1024..1023。
    //   txt_col: 每行 {R,G,B} = 24 位。原来高 8 位是 A（alpha），
    //            但本文件的所有赋值都写死 8'hFF（文字永远不透明），
    //            ⇒ 存 24 位、在 ps_gen_text 里补回常量 8'hFF，省 8 个 FF/行。
    //   合计省 (32-22 + 32-24) × 8 = 144 个触发器。
    output reg  [22*NL-1:0] txt_xy,
    output reg  [24*NL-1:0] txt_col,

    output reg         str_we,
    output reg  [3:0]  str_line,
    output reg  [7:0]  str_idx,
    output reg  [15:0] str_code,
    output reg         ctl_we,
    output reg  [3:0]  ctl_line,
    output reg         ctl_active,
    output reg  [7:0]  ctl_len,
    output reg  [7:0]  ctl_ver,
    output reg         trig,

    // ---- 画质参数导出（2026-10-26 新增）----
    //  显示页 val 的实时值 → 顶层 video_postproc（跨域同步在 postproc 内部做）。
    //  组合导出：在显示页（page_r==0）用 val 活跃值，不在显示页时用 b0 存档
    //  （val 里装的是别的页的值）。字段序见下面 1c 节的 vpack 注释。
    output wire [6:0]  pic_bright,     // 亮度 0..100（50=默认）
    output wire [6:0]  pic_contrast,   // 对比度
    output wire [6:0]  pic_temp,       // 色温 0=冷 100=暖
    output wire [6:0]  pic_sat,        // 饱和度 0=黑白 50=原色
    output wire [1:0]  pic_style,      // 风格 0无 1黑白 2反色
    output reg         pic_tgl,        // 参数更新翻转标志（写回/恢复默认时翻转）

    //--------------------------------------------------------------------------
    //  系统页导出（2026-10-29 新增）→ 顶层音频链路
    //--------------------------------------------------------------------------
    //  与 pic_* 同一套规则：在本页时用 val 活跃值，不在本页时用 b3 存档。
    //  ★ o_lang 还要参与**标签 ROM 的地址**（英文表 +40）和文字取字，
    //    所以它必须是纯组合、随 page 立刻生效 —— 不能用 page_r 判。
    output wire        o_lang,         // 0=中文 1=English
    output wire [6:0]  o_vol,          // 音量 0..100
    output wire        o_mute,         // 静音（与 SW2 串联，任一关即静音）
    output wire        o_loop,         // 循环播放 1=播完回开头
    output wire        o_spc,          // 频谱条（预留，本期不接任何逻辑）

    //--------------------------------------------------------------------------
    //  相册页导出（2026-10-27 新增）
    //--------------------------------------------------------------------------
    output wire [2:0]  fx_sel,         // 切换效果 0..4（viewer_ctrl 直接用）
    output wire [2:0]  auto_mode,      // 自动切换 0关 1:3s 2:5s 3:10s 4:30s 5音乐结束
    output wire        fit_scr,        // 适应屏幕 1=拉伸 0=contain
    output wire        info_on,        // 信息条开关
    output reg         album_prev,     // 菜单里"上一张"单拍脉冲
    output reg         album_next,     // 菜单里"下一张"单拍脉冲

    //--------------------------------------------------------------------------
    //  导航图标"激活态"掩码（2026-10-28 新增）→ osd_gfx
    //--------------------------------------------------------------------------
    //  【为什么要这根线】图标改成 RGB565 直出后，普通态/激活态是**两份独立位图**，
    //  act 必须进 ROM 地址；而地址在**拍 0** 就要算，那时只有 hit_idx，
    //  `rd_attr` 里的 style 要到拍 1 才出来。
    //  ⇒ 这里在拍 0 之前把"哪个导航图标是激活态"组合导出（一帧之内不变）。
    //  【为什么是 8 位】osd_gfx 用 `ic_act_mask[ic_i]` 查表，而 ic_i =
    //  hit_idx[2:0]−5，非图标槽会算成 5/6/7 ⇒ 掩码必须覆盖 0..7，
    //  否则仿真里会取到 X 污染地址（bit5..7 恒 0）。
    output wire [7:0]  ic_act_mask,

    //--------------------------------------------------------------------------
    //  来自 viewer_ctrl 的信息（信息条显示用）
    //--------------------------------------------------------------------------
    input  wire [15:0] src_w,          // 当前图源分辨率（★ 不能叫 cur_w：与游标几何重名）
    input  wire [15:0] src_h,
    input  wire [5:0]  auto_remain,    // 剩余秒数
    input  wire [8:0]  auto_bar        // 进度条像素宽 0..287
);

//------------------------------------------------------------------------------
//  常量
//------------------------------------------------------------------------------
localparam integer VIS   = 5;
localparam integer NITEM = 7;
localparam integer NOPT  = 3;          // 下拉选项数
localparam integer LASTN = NOPT-1;
localparam [9:0]   ROW0  = 10'd64;
localparam [9:0]   PITCH = 10'd80;
localparam [9:0]   CTRL_X= 10'd320;
localparam [9:0]   CTRL_W= 10'd296;    // 320..616
localparam [9:0]   TXT_X = 10'd112;

//------------------------------------------------------------------------------
//  ★ 2026-10-28：logo（左上角）与导航栏下移
//------------------------------------------------------------------------------
//  logo 放在左栏顶部，64×41（SVG 48:31 ⇒ 64/1.5484 = 41.3），左右各留 16px。
//  宽度取 64 是**算过的**：logo 字地址 = 5120 + {ly[5:0], lx[5:1]}，
//  6 位行 × 5 位列正好对齐，osd_gfx 里只花 2 个 LUT（见那里的推导）。
//
//  导航栏整体下移 48px（图标 24+80i → 72+80i，游标 20+80i → 68+80i）：
//  logo 底边 49，第一个游标顶边 68，间距 19；第 5 个游标底边 428，屏高 480 还剩 52。
//  【标题和页面行不动】logo 只占左栏（x 16..80），与 x≥112 的标签/控件不重叠。
localparam [9:0]   LOGO_X  = 10'd16;
localparam [8:0]   LOGO_Y  = 9'd8;
localparam [9:0]   LOGO_W  = 10'd64;
localparam [8:0]   LOGO_H  = 9'd41;
localparam [9:0]   NAV_Y0  = 10'd72;   // 第 0 个导航图标的 y
localparam [9:0]   NAV_CY0 = 10'd68;   // 第 0 个导航游标的 y（图标 −4）

// ★ 滑条步进（2026-10-26：1 → 5，用户指定）：postproc 增益有量化（相邻 1~2%
//   可能同输出），5 一档手感更好；连发 83ms/步 ⇒ 0..100 全程约 1.7s。
localparam [6:0]   TB_STEP = 7'd5;

// ★ 每页一套初值（{v6,v5,v4,v3,v2,v1,v0}，最低 7 位是 val0）
// ★ 2026-10-26 显示页重排后：v0 亮度50 v1 对比50 v2 色温50 v3 饱和50
//   v4 风格0(无) v5/v6 恒 0（按钮/空行不存值）。全 50 = video_postproc 的
//   恒等直通点 ⇒ "恢复默认"后画面回到无处理状态。
// ★ 2026-10-27 页序改为：0 显示 / 1 相册 / 2 文件 / 3 系统 / 4 关于
localparam [48:0] DEF0 = {7'd0, 7'd0, 7'd0, 7'd50, 7'd50, 7'd50, 7'd50}; // 显示
// 相册：v0 上一张(按钮) v1 下一张(按钮) v2 效果0 v3 自动0(关) v4 适应屏1 v5 信息条0
localparam [48:0] DEF1 = {7'd0, 7'd0, 7'd1, 7'd0, 7'd0, 7'd0, 7'd0};
// DEF2（文件页）已删：整页无编辑项，值不存不留
// ★ 2026-10-29 系统页重排（5 项）：
//   v0 语言0(中文) v1 音量100 v2 静音0 v3 循环1(保持现状) v4 频谱0
//   音量默认 100 = 单位增益，听感与改造前完全一致（audio_vol 在 100% 时逐位直通）。
localparam [48:0] DEF3 = {7'd0, 7'd0, 7'd0, 7'd1, 7'd0, 7'd100, 7'd0};  // 系统
// DEF4（关于页）已删：整页全是 Label，值不存不留

// ★ L_TWN：把 6 条动画通道的 tween **时间复用**（一个乘法器分时服务 6 条）。
//   原来 6 套并行的 tween 是 osd_menu 里最贵的一块（14 个 DSP + 大量进位链）。
//   动画每帧只走一步，帧间有 83 万个 50MHz 周期，多花 5 拍毫无代价。
localparam [2:0] L_IDLE = 3'd0, L_ANIM = 3'd1, L_TWN  = 3'd6, L_CELL = 3'd2,
                 L_TXY  = 3'd3, L_TXT  = 3'd4, L_FIN  = 3'd5;

localparam [3:0]  LASTL = NL-1;
// ★ 2026-10-26：MAXLEN 20 → 32（风格下拉收起态要写满 16 字，占第 14..29 列）。
//   【勿单独改】LASTC = MAXLEN-1、ctl_len = MAXLEN，三处必须同步；
//   不写成 MAXLEN 位选表达式是因为 TD 对 integer 参数做位选会算错（踩过）。
localparam [4:0]  LASTC = 5'd31;       // MAXLEN-1（MAXLEN 固定 32）

// 缓动步长（每帧一次）
localparam [15:0] STEP_C = 16'd5461;   // 12 帧 ≈ 200ms（位置）
localparam [15:0] STEP_S = 16'd4096;   // 16 帧 ≈ 266ms（滚动）
localparam [15:0] STEP_V = 16'd6554;   // 10 帧 ≈ 166ms（数值）

//------------------------------------------------------------------------------
//  0. 纯函数（必须在任何使用点之前声明）
//------------------------------------------------------------------------------
// ease = 二次 in-out：x<128 ? 2x²/256 : 256-2(256-x)²/256
function [8:0] ease_of;
    input [7:0] x;
    reg   [8:0]  d;
    reg   [17:0] sq;
    begin
        d  = (x < 8'd128) ? {1'b0, x} : (9'd256 - {1'b0, x});
        sq = d * d;                       // ≤ 128*128 = 16384
        ease_of = (x < 8'd128) ? sq[16:7] : (9'd256 - sq[16:7]);
    end
endfunction

// ★★★ 2026-10-29：tween 的乘法从「推断 25×25」改成「显式 EG_PHY_MULT18 16×9」★★★
//  【为什么必须显式例化】原写法
//        p = {9'd0, (to-from)} * {16'd0, e};
//    两个操作数被零扩展成 25 位，赋值上下文也是 25 位 ⇒ TD 按 **25×25** 综合；
//    18×18 的 DSP 装 25 位要 2 片/操作数 ⇒ 2×2 = **4 个 DSP**。
//    （`syn_1/*_gate.area` 层次报告实测 u_menu dsp = 4，与该映射完全吻合。）
//    改成 `p = d * e;`（d 16 位、e 9 位）**没有用**：赋值上下文仍把两个操作数
//    拉到 25 位，TD 不做"高位恒 0"的位宽收缩 —— 这一条已被 25×25 的实测证伪。
//    所以只能显式例化，让乘法的两个操作数**物理上就是 16 位和 9 位**。
//  【等价性】16 位 × 9 位 = 25 位，与原来的 25×25 乘积逐位相同（高 9 位本来就是 0）
//    ⇒ tween 输出逐拍不变，动画曲线一个像素都不变。
//  【代价/收益】osd_menu 的 DSP 4 → 2（另一个在 ease_of 的 d*d），腾出 2 个。
//  【sourcea/sourceb 必须接 1】见 eg_phy_mult18.v:314 ——
//    sourcea=0 选 acin、=1 选 a。接 0 会乘 acin×bcin = 0（静默出错，仿真才发现）。
//    ★ 实现放在 §4 动画通道处（tw_o 的生成点），因为要接 tw_c/tw_t/tw_f。

function [15:0] sat;
    input [15:0] x;
    input [15:0] step;
    begin
        sat = ((x + step) < x) ? 16'd65535 : (x + step);
    end
endfunction

// ★★★ 控件类型表：类型必须跟"语义"对上，否则就是"控件逻辑混乱"的根源 ★★★
//   改过的四处（原来都是错的）：
//   ① page0 item4 锐度：ProgressBar(8) → TrackBar(4)
//      进度条**不可编辑**，而"锐度"是要能调的 ⇒ 上板看到一条拖不动的滑条。
//   ② page1 item5 适应屏幕：ComboBox(6) → CheckBox(5) —— 它就是个开关。
//   ③ page2 item0 语言：NumericUpDown(10) → ComboBox(6)
//      数值框会把"语言"显示成 "60%"（和亮度共用 val0 的值），完全说不通。
//   ④ page2 item1..3 分辨率/帧率/色彩空间：TrackBar(4) → ComboBox(6)
//      "分辨率 = 50%" 没有意义，三档下拉才说得通。
//   ⑤ page2 item4 版本：Button(7) → Label(1) —— 版本号是只读显示，不是按钮。
function [3:0] cell_type;
    input [2:0] pg;
    input [2:0] it;
    begin
        case (pg)
            3'd0: case (it)                        // 显示
                // ★ 2026-10-26 重排：色温 ComboBox→TrackBar（0~100 连续调），
                //   高动态 CheckBox→饱和度 TrackBar，锐度 TrackBar→风格 ComboBox，
                //   退出菜单按钮删除（item6 空行，光标停在 item5）。
                3'd0, 3'd1: cell_type = 4'd4;      // 亮度 / 对比度  TrackBar
                3'd2, 3'd3: cell_type = 4'd4;      // 色温 / 饱和度  TrackBar
                3'd4:       cell_type = 4'd6;      // 风格            ComboBox（无/黑白/反色）
                3'd5:       cell_type = 4'd7;      // 恢复默认        Button
                default:    cell_type = 4'd1;      // 空行（Label：不画控件、不留光标）
            endcase
            3'd1: case (it)                        // 相册（2026-10-27 重排，6 项）
                3'd0, 3'd1: cell_type = 4'd7;      // 上一张 / 下一张 Button
                3'd2:       cell_type = 4'd6;      // 切换效果        ComboBox（5 项）
                3'd3:       cell_type = 4'd6;      // 自动切换        ComboBox（6 项）
                3'd4:       cell_type = 4'd5;      // 适应屏幕        CheckBox
                3'd5:       cell_type = 4'd5;      // 信息条          CheckBox
                default:    cell_type = 4'd1;
            endcase
            3'd2:       cell_type = 4'd1;          // 文件：整页空白（只有标题）
            3'd3: case (it)                        // 系统（2026-10-29 重排，5 项）
                // ★ 原来这 4 个下拉（语言/分辨率/帧率/色彩空间）全是占位 ——
                //   pop_set/row_set 都落到 set 0（低/中/高），选哪个都没实际作用。
                //   现在只留"语言"一个下拉，其余换成音频相关的 1 滑条 + 3 勾选。
                3'd0:       cell_type = 4'd6;      // 语言            ComboBox（中/EN）
                3'd1:       cell_type = 4'd4;      // 音量            TrackBar 0..100
                3'd2:       cell_type = 4'd5;      // 静音            CheckBox
                3'd3:       cell_type = 4'd5;      // 循环播放        CheckBox
                3'd4:       cell_type = 4'd5;      // 频谱条          CheckBox（预留）
                default:    cell_type = 4'd1;      // 空行（本页已无 Button）
            endcase
            default:    cell_type = 4'd1;          // 关于页全是 Label
        endcase
    end
endfunction

// ★★ 三张表都按"实际最长字数"给位宽，不再一律 160 位。
//   原来 label_of 返回 160 位（= 10 字），但最长标签只有 4 字（恢复默认/退出菜单/
//   切换效果/适应屏幕/色彩空间/升级固件/恢复出厂/编译日期），后 96 位恒为 0，
//   却仍要给每个输出位生成一套 28:1 的选择逻辑 ⇒ 白烧几百 LUT。
//   收成 64 位（4 字）后，取字下标从 txt_c[3:0] 变成 txt_c[1:0]，
//   标签仍占第 0..3 格（x 112..176），值仍在第 9..12 / 第 16 格 —— 布局不变。

// 下拉选项字符表（2026-10-27 扩到 4 组）
//   set=0 → 低 / 中 / 高（各页通用三档，1 字）
//   set=1 → 无 / 黑白 / 反色（显示页"风格"，最长 2 字）
//   set=2 → 左右 / 上下 / 淡入 / 缩放 / 移门（相册"切换效果"，5 项 2 字）
//   set=3 → 关 / 3s / 5s / 10s / 30s / 音乐结束（相册"自动切换"，6 项）
// ci = 字符序号 0..15，超出串长返回 0。收起态与弹层共用本表。
// ★ 2026-10-30：加 lang —— 英文模式下**选项值**也要是英文（"纯英文"）。
//   不另开一套 opt_char_en（那样两条 mux 都要算，LUT 翻倍），而是在每个
//   (set,k) 分支里就地按 lang 选：TD 会把它折成一张按 {set,k,lang,ci}
//   索引的常数表，实测代价约 +50 LUT。
//   ★ 3s/5s/10s/30s 两种语言完全一样，不加 lang 项（省 4 个 2:1）。
function [15:0] opt_char;
    input [1:0]  set;
    input [2:0]  k;
    input [4:0]  ci;
    input        lang;
    begin
        case (set)
            2'd1: case (k)                                // 无 / 黑白 / 反色
                3'd0:    opt_char = lang
                    ? ((ci == 5'd0) ? 16'h004E : (ci == 5'd1) ? 16'h006F :
                       (ci == 5'd2) ? 16'h006E : (ci == 5'd3) ? 16'h0065 : 16'h0000)  // None
                    : ((ci == 5'd0) ? 16'hCEDE : 16'h0000);                           // 无
                3'd1:    opt_char = lang
                    ? ((ci == 5'd0) ? 16'h0047 : (ci == 5'd1) ? 16'h0072 :
                       (ci == 5'd2) ? 16'h0061 : (ci == 5'd3) ? 16'h0079 : 16'h0000)  // Gray
                    : ((ci == 5'd0) ? 16'hBADA : (ci == 5'd1) ? 16'hB0D7 : 16'h0000); // 黑白
                default: opt_char = lang
                    ? ((ci == 5'd0) ? 16'h0049 : (ci == 5'd1) ? 16'h006E :
                       (ci == 5'd2) ? 16'h0076 : (ci == 5'd3) ? 16'h0065 :
                       (ci == 5'd4) ? 16'h0072 : (ci == 5'd5) ? 16'h0074 : 16'h0000)  // Invert
                    : ((ci == 5'd0) ? 16'hB7B4 : (ci == 5'd1) ? 16'hC9AB : 16'h0000); // 反色
            endcase
            2'd2: case (k)                                // 左右 / 上下 / 淡入 / 缩放 / 移门
                3'd0:    opt_char = lang
                    ? ((ci == 5'd0) ? 16'h004C : (ci == 5'd1) ? 16'h002D :
                       (ci == 5'd2) ? 16'h0052 : 16'h0000)                            // L-R
                    : ((ci == 5'd0) ? 16'hD7F3 : (ci == 5'd1) ? 16'hD3D2 : 16'h0000); // 左右
                3'd1:    opt_char = lang
                    ? ((ci == 5'd0) ? 16'h0055 : (ci == 5'd1) ? 16'h002D :
                       (ci == 5'd2) ? 16'h0044 : 16'h0000)                            // U-D
                    : ((ci == 5'd0) ? 16'hC9CF : (ci == 5'd1) ? 16'hCFC2 : 16'h0000); // 上下
                3'd2:    opt_char = lang
                    ? ((ci == 5'd0) ? 16'h0046 : (ci == 5'd1) ? 16'h0061 :
                       (ci == 5'd2) ? 16'h0064 : (ci == 5'd3) ? 16'h0065 : 16'h0000)  // Fade
                    : ((ci == 5'd0) ? 16'hB5AD : (ci == 5'd1) ? 16'hC8EB : 16'h0000); // 淡入
                3'd3:    opt_char = lang
                    ? ((ci == 5'd0) ? 16'h005A : (ci == 5'd1) ? 16'h006F :
                       (ci == 5'd2) ? 16'h006F : (ci == 5'd3) ? 16'h006D : 16'h0000)  // Zoom
                    : ((ci == 5'd0) ? 16'hCBF5 : (ci == 5'd1) ? 16'hB7C5 : 16'h0000); // 缩放
                default: opt_char = lang
                    ? ((ci == 5'd0) ? 16'h0057 : (ci == 5'd1) ? 16'h0069 :
                       (ci == 5'd2) ? 16'h0070 : (ci == 5'd3) ? 16'h0065 : 16'h0000)  // Wipe
                    : ((ci == 5'd0) ? 16'hD2C6 : (ci == 5'd1) ? 16'hC3C5 : 16'h0000); // 移门
            endcase
            2'd3: case (k)                                // 关 / 3s / 5s / 10s / 30s / 音乐结束
                3'd0:    opt_char = lang
                    ? ((ci == 5'd0) ? 16'h004F : (ci == 5'd1) ? 16'h0066 :
                       (ci == 5'd2) ? 16'h0066 : 16'h0000)                            // Off
                    : ((ci == 5'd0) ? 16'hB9D8 : 16'h0000);                           // 关
                3'd1:    opt_char = (ci == 5'd0) ? 16'h0033 :                            // 3s
                                    (ci == 5'd1) ? 16'h0073 : 16'h0000;
                3'd2:    opt_char = (ci == 5'd0) ? 16'h0035 :                            // 5s
                                    (ci == 5'd1) ? 16'h0073 : 16'h0000;
                3'd3:    opt_char = (ci == 5'd0) ? 16'h0031 :                            // 10s
                                    (ci == 5'd1) ? 16'h0030 :
                                    (ci == 5'd2) ? 16'h0073 : 16'h0000;
                3'd4:    opt_char = (ci == 5'd0) ? 16'h0033 :                            // 30s
                                    (ci == 5'd1) ? 16'h0030 :
                                    (ci == 5'd2) ? 16'h0073 : 16'h0000;
                default: opt_char = lang
                    ? ((ci == 5'd0) ? 16'h0045 : (ci == 5'd1) ? 16'h006E :
                       (ci == 5'd2) ? 16'h0064 : (ci == 5'd3) ? 16'h004D :
                       (ci == 5'd4) ? 16'h0075 : (ci == 5'd5) ? 16'h0073 :
                       (ci == 5'd6) ? 16'h0069 : (ci == 5'd7) ? 16'h0063 : 16'h0000)  // EndMusic
                    : ((ci == 5'd0) ? 16'hD2F4 : (ci == 5'd1) ? 16'hC0D6 :
                       (ci == 5'd2) ? 16'hBDE1 : (ci == 5'd3) ? 16'hCAF8 : 16'h0000); // 音乐结束
            endcase
            // ★ 2026-10-29：set 0 原来是给系统页 4 个占位下拉用的（低/中/高）。
            //   系统页改完后它没有任何别的用户，直接改成"语言"的 2 个选项。
            //   ★ 语言名用「中 / EN」而不是「中文 / English」：后者条目会涨到 14 条，
            //     而这两个选项本身是"用哪种语言写的就用哪种语言"，不需要翻译。
            default: case (k)                             // 中 / EN
                3'd0:    opt_char = (ci == 5'd0) ? 16'hD6D0 : 16'h0000;  // 中
                default: opt_char = (ci == 5'd0) ? 16'h0045 :            // E
                                    (ci == 5'd1) ? 16'h004E : 16'h0000;  // N
            endcase
        endcase
    end
endfunction

// 各选项组的项数（弹层锚定 / 选项值夹紧用它，不再写死 NOPT）
function [2:0] nopt_of;
    input [1:0] set;
    begin
        case (set)
            2'd0:    nopt_of = 3'd2;        // 语言：中 / EN（2026-10-29）
            2'd2:    nopt_of = 3'd5;
            2'd3:    nopt_of = 3'd6;
            default: nopt_of = 3'd3;        // set 1：无 / 黑白 / 反色
        endcase
    end
endfunction

// ===== 标签/标题表已改为 BRAM ROM（见下方 AUTO-GEN 段）=====
//   ★ 2026-10-29：这里原来还有一个 28 分支的 label_of() 函数，2026-10-28 改
//     BRAM 之后就没有任何调用点了（纯死代码）。虽然 elaborator 不会实例化它，
//     但留着会让人以为改标签要去改这里 —— 直接删掉。
//   ★ 现在要改标签，改 tools/gen_label_rom.py 的表，再跑 --write。

//------------------------------------------------------------------------------
//  1. 状态
//------------------------------------------------------------------------------
reg       focus;          // 0 = NAV, 1 = PAGE
reg [2:0] nav_idx;        // 2026-10-27：2 → 3 位（5 个导航项）
reg [2:0] page;
reg [2:0] item;
reg       edit;
reg [2:0] scroll_row;

reg [6:0] val0, val1, val2, val3, val4, val5, val6;



//------------------------------------------------------------------------------
//  1a. 每页独立的"值"（换页时存档/读档）
//------------------------------------------------------------------------------
//  ★ 原来这 7 个寄存器被 4 个页**按行号共用**：在"显示"页把亮度拖到 80，
//    切到"系统"页，分辨率（也是第 1 行）就跟着变成 80 —— 上板看着就是
//    "控件逻辑混乱"。现在每页各存一份（4×49 位 = 196 个触发器，很便宜），
//    换页那一拍同时做"存旧页 + 读新页"：
//        非阻塞赋值右边取的都是**本拍之前**的值 ⇒ 同一拍完成，不会互相覆盖。
//    读档用的是 49 位 4 选 1 选择器（约 100 个 LUT），只在换页那一拍有用。
// ★ 2026-10-27：只备份"有可编辑项"的三页（显示/相册/系统）。
//   文件页与关于页全是 Label，值没有任何读者 ⇒ 省掉 2×49 = 98 个触发器。
reg [48:0] b0, b1, b3;               // 3 个页的值备份（低位 = val0）
reg [2:0]  page_r;                  // val0..6 里当前装的是哪一页

// ★ 上电确定值（**不靠复位**：顶层 rst_all = ~rst_n | ~audio_pll_lock，
//   板上 rst_n 上拉 + PLL 上电即锁 ⇒ 复位可能一次都不拉高）。
//   没有初值时 val*/b0..b3/page_r 停在上电值 0：
//     · 菜单一打开全是 0%，后处理被压成平表（画面发灰/发黑）；
//     · 换页回来用 b0=0 覆盖 val*，把显示页的值清零。
//   ★ 为什么用 initial 而不是自造复位：TD 的异步复位块里 if 条件必须直接对应
//     敏感列表上的边沿信号，写 `if (rst | por_rst)` 会报
//     "if-condition does not match any sensitivity list edge"（实测）。
//     本工程 osd_font_cache 的 owner 数组用的就是同款 initial 初值，已上板验证。
initial begin
    {val6, val5, val4, val3, val2, val1, val0} = DEF0;
    b0 = DEF0;  b1 = DEF1;  b3 = DEF3;
    page_r = 3'd0;
end
wire [48:0] vpack = {val6, val5, val4, val3, val2, val1, val0};

//------------------------------------------------------------------------------
//  1d. 相册页导出（2026-10-27）→ viewer_ctrl
//------------------------------------------------------------------------------
//  与 pic_* 同一套规则：在本页时用 val 活跃值，不在本页时用 b1 存档。
//  字段序同 vpack：v0=[6:0] v1=[13:7] v2=[20:14] v3=[27:21] v4=[34:28] v5=[41:35]
assign fx_sel    = (page_r == 3'd1) ? val2[2:0] : b1[16:14];
assign auto_mode = (page_r == 3'd1) ? val3[2:0] : b1[23:21];
assign fit_scr   = (page_r == 3'd1) ? val4[0]   : b1[28];
assign info_on   = (page_r == 3'd1) ? val5[0]   : b1[35];

//------------------------------------------------------------------------------
//  1e. 导航图标激活态掩码（2026-10-28）→ osd_gfx 的拍 0 地址
//------------------------------------------------------------------------------
//  图标改成 RGB565 直出后，普通/激活是两份独立位图，act 必须进 ROM 地址；
//  而地址在拍 0 就算，那时只有 hit_idx。所以这里**组合**导出 one-hot 掩码。
//  它在一帧之内不变（focus/nav_idx 只在帧间布局时改），组合使用是安全的。
//  ★ 必须是 8 位：osd_gfx 用 ic_act_mask[ic_i] 查表，ic_i = hit_idx[2:0]−5
//    对非图标槽会算成 5/6/7，位宽不够会取到 X 污染地址。
//  ★ 2026-10-28（用户要求）：**不再看 focus** —— 进页面后左侧图标要继续显示
//    激活态（变蓝），作为"当前在哪一页"的指示。原来写成
//      (focus == 0) ? one-hot : 0
//    一按 KEY2 进页面图标就全部变灰，看不出当前页。
assign ic_act_mask = (8'b0000_0001 << nav_idx);

//------------------------------------------------------------------------------
//  1c. 画质参数导出（2026-10-26 新增）→ 顶层 video_postproc
//------------------------------------------------------------------------------
//  val0..6 是 page_r 当前页的**活跃值**，b0 是显示页的存档：不在显示页时
//  val 里装的是别的页的值，此时用 b0（显示页参数保持不变）。
//  【字段序】vpack = {v6..v0} ⇒ v0=[6:0] v1=[13:7] v2=[20:14] v3=[27:21]
//  v4=[34:28]。编辑/恢复默认改变显示页值的那一拍会翻转 pic_tgl（见主状态机）。
//  （端口已在端口表声明为 output wire/reg，这里只做连续赋值。）
assign pic_bright   = (page_r == 3'd0) ? val0 : b0[6:0];
assign pic_contrast = (page_r == 3'd0) ? val1 : b0[13:7];
assign pic_temp     = (page_r == 3'd0) ? val2 : b0[20:14];
assign pic_sat      = (page_r == 3'd0) ? val3 : b0[27:21];
assign pic_style    = (page_r == 3'd0) ? val4[1:0] : b0[29:28];

//------------------------------------------------------------------------------
//  1c. 系统页导出（2026-10-29）
//------------------------------------------------------------------------------
//  【为什么用 page_r 而不是 page】与 pic_* 保持同一套规则。换页那一拍 page
//  已经变了、val* 还是旧页的值，若用 page 会在那一拍把"旧页的 val0 最低位"
//  当成语言（page0 的亮度 50 → bit0=0），虽然下一拍就自愈、且布局机还没跑，
//  但没有理由留这个毛刺 —— b3 里存的就是真正的语言值。
assign o_lang = (page_r == 3'd3) ? val0[0]   : b3[0];
assign o_vol  = (page_r == 3'd3) ? val1    : b3[13:7];
assign o_mute = (page_r == 3'd3) ? val2[0] : b3[14];
assign o_loop = (page_r == 3'd3) ? val3[0] : b3[21];
assign o_spc  = (page_r == 3'd3) ? val4[0] : b3[28];

//------------------------------------------------------------------------------
//  1b. 取值：★ 必须用"显式 case + always @(*)"，不能用 function
//------------------------------------------------------------------------------
//  原来写的是 function val_of(item)，函数**体内**读 val0..val6 —— iverilog
//  生成敏感表时只看调用处写出来的标识符（item），函数体内读到的 val0..val6
//  **不会**进敏感表 ⇒ 仿真里 val 已经改了、取出来的还是旧值（实测：val2 改成
//  0 了，item_val 仍是 1）。上板综合出来是纯 MUX 所以行为是对的，但这样
//  "仿真验证"就整个失效了 —— 根本没验到真实行为。
//  改成 always @(*) 里直接写 case，val0..val6 是块内直接读的 ⇒ 敏感表正确。
//  综合结果完全相同（8:1 MUX），不增加面积。
reg [6:0] sel_val;                    // 选中行（item）的值
always @(*) begin
    case (item)
        3'd0: sel_val = val0;
        3'd1: sel_val = val1;
        3'd2: sel_val = val2;
        3'd3: sel_val = val3;
        3'd4: sel_val = val4;
        3'd5: sel_val = val5;
        default: sel_val = val6;
    endcase
end

// ★ 只有 6 条通道（cx/cy/cw/ch/sc/sv）：行高亮和游标共用同一条 cy，
//   原来另有 hl（选中行 y）与 ny（导航 y）两条 ⇒ 7 条 tween ⇒ 7 个 16×9 乘法
//   + 7 个 9×9 平方，直接把 mslice 顶到 4957（上限 4900）。
reg [15:0] cx_f, cx_t, cx_c;
reg [15:0] cy_f, cy_t, cy_c;
reg [15:0] cw_f, cw_t, cw_c;
reg [15:0] ch_f, ch_t, ch_c;
reg [15:0] sc_f, sc_t, sc_c;
reg [15:0] sv_f, sv_t, sv_c;
wire [9:0] sc_px  = sc_c[15:4];   // 当前滚动像素（整数部分）
wire [9:0] srow_px= {7'd0, scroll_row} * 10'd80;   // 滚动行数 × 行距
wire [7:0] sv_disp= sv_c[11:4];   // 数值动画的显示值（0..100）

// 布局状态机
reg [2:0] lay_st;
reg [4:0] cell_i;
reg [3:0] txt_l;
reg       dirty;
reg       did_write;
reg       menu_d;
wire      menu_rise = menu_on & ~menu_d;

//------------------------------------------------------------------------------
//  2c. 信息条内容变化检测（菜单关着时也要让布局/文字重写一次）
//------------------------------------------------------------------------------
reg [15:0] cw_d, ch_d;
reg [5:0]  rem_d;
reg [2:0]  am_d;
reg        inf_d;
wire       info_rise = info_on & ~inf_d;
// ★ 2026-10-30 缩放 OSD 的重排触发
reg        zoom_d = 1'b0;
reg  [7:0] zoomv_d = 8'h10;
reg  [1:0] zoomm_d = 2'd0;
wire       zoom_rise = zoom_on & ~zoom_d;
wire       zoom_chg  = zoom_on & ((zoom_val != zoomv_d) || (zoom_mode != zoomm_d));
wire       info_chg  = (cw_d != src_w) || (ch_d != src_h) ||
                       (rem_d != auto_remain) || (am_d != auto_mode) ||
                       (inf_d != info_on);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        cw_d <= 16'd0; ch_d <= 16'd0; rem_d <= 6'd0; am_d <= 3'd0; inf_d <= 1'b0;
    end else begin
        cw_d <= src_w; ch_d <= src_h; rem_d <= auto_remain; am_d <= auto_mode; inf_d <= info_on;
        zoom_d <= zoom_on; zoomv_d <= zoom_val; zoomm_d <= zoom_mode;
    end
end

//------------------------------------------------------------------------------
//  2d. 相册页"上一张/下一张"的脉冲在按键声明之后（见 §2 末尾）
//------------------------------------------------------------------------------
// ★ txt_c 必须是 6 位：L_TXT 的退出条件是 txt_c 越过 LASTC(=31)。
//   若用 5 位，写完第 31 列后 txt_c+1 回绕成 0 ⇒ 永远退不出 L_TXT ⇒
//   布局状态机死循环：cell 表/文字表在首帧之后再也不会更新
//   （2026-10-26 实测：tb_osd_keys 的 R2/R15/V8/V9/W2 全因这个挂掉）。
reg [5:0] txt_c;


//==============================================================================
//  ★★ 布局/动画节拍 ★★
//==============================================================================
//  内部 60Hz 自由节拍（833333 拍 @50MHz）+ 外部 tick 作为"额外触发"。
//  请求锁存，只在 ps_start==0（帧间空隙）时真正启动布局 ⇒ 不撕裂。
reg [19:0] loc_cnt;
reg        lay_req;
wire       local_tick = (loc_cnt >= 20'd833_332);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        loc_cnt <= 20'd0;
        lay_req <= 1'b1;                 // 上电先排一次
    end else begin
        loc_cnt <= local_tick ? 20'd0 : (loc_cnt + 20'd1);
        // ★ 2026-10-30：缩放 OSD 复用槽 16 + 行 6/7，它不属于菜单/信息条，
        //   所以"开关/换档/换功能"都必须自己触发一次重排，否则槽和文字不更新。
        if (local_tick || tick || menu_rise || info_rise ||
            zoom_rise || zoom_chg) lay_req <= 1'b1;
        if (lay_st == L_ANIM)                lay_req <= 1'b0;
    end
end
reg [7:0] txt_ver_r;

// 只有 L_ANIM 这一拍推进动画（每帧一次）
wire      anim_step = (lay_st == L_ANIM);

//------------------------------------------------------------------------------
//  2. 按键
//------------------------------------------------------------------------------
//  ★★ 按键映射（按面板丝印编号）★★
//  顶层 top_tf_hdmi_audio 里 key_press[3:0] = {KEY4, KEY3, KEY2, KEY1}，
//  所以下标就是面板编号 - 1。菜单用如下四键：
//
//      KEY1 → OSD 关着时：打开 OSD
//              OSD 开着时：返回。**在导航栏（focus=0）按下 = 直接关掉 OSD**，
//                          在页面里按下 = 回到导航栏，编辑中按下 = 取消编辑
//      KEY2 → 进入：导航栏 → 页面；页面 → 编辑；编辑中再按 = 确认（退出编辑）
//      KEY3 → 下：换下一行 / 数值减
//      KEY4 → 上：换上一行 / 数值加
//
//  ★ 长按 800ms 退出**已取消**（用户要求）。原来那套 hold_cnt/long_fired 不只是
//    多余逻辑，它还逼着"上"键必须避开长按那一下（k_up 要 & ~long_fired），
//    现在 KEY1 专职退出，这 26 位计数器 + 比较器可以整块删掉。
wire k_enter = key_press[0] & ~menu_on & osd_allow;   // KEY1：打开 OSD
wire k_back  = key_press[0] &  menu_on;               // KEY1：返回 / 退出 OSD
wire k_ok    = key_press[1] &  menu_on;               // KEY2：进入 / 确认
wire k_down  = key_press[2] &  menu_on;               // KEY3：下
wire k_up    = key_press[3] &  menu_on;               // KEY4：上

always @(posedge clk or posedge rst) begin
    if (rst) begin
        exit_req  <= 1'b0;
        enter_req <= 1'b0;
    end else begin
        // KEY1 在导航栏按下 ⇒ 关 OSD（单拍脉冲给 App）；
        // 在页面里按下时 focus=1，这里不产生 exit_req，由状态机"回上一级"。
        exit_req  <= k_back & ~focus;
        enter_req <= k_enter;
    end
end

//------------------------------------------------------------------------------
//  2b. 长按连发（2026-10-26 新增）
//------------------------------------------------------------------------------
//  编辑态下按住 KEY4（加）/ KEY3（减）：500ms（30 个 60Hz tick）后开始连发，
//  之后每 5 拍（≈83ms）注入一个合成脉冲。导航态**不连发**（换行不需要）。
//  key_level：1=松开（已消抖）——长按退出取消后一直闲置，正好派上用场。
//  合成脉冲只进"值加减"路径（k_up_e/k_dn_e），不进导航换行（那边用原始 k_up/k_down）。
reg [5:0] rpt_cnt;                 // 60Hz tick 计数（到 30 触发首次连发）
reg       rpt_up;                  // 连发方向：1=加（KEY4）0=减（KEY3）
reg       rpt_vld;                 // 本拍注入一个合成脉冲（单拍）

always @(posedge clk or posedge rst) begin
    if (rst) begin
        rpt_cnt <= 6'd0;
        rpt_up  <= 1'b0;
        rpt_vld <= 1'b0;
    end else begin
        rpt_vld <= 1'b0;
        if (edit && menu_on && ((key_level[3] == 1'b0) || (key_level[2] == 1'b0))) begin
            rpt_up <= (key_level[3] == 1'b0);          // KEY4 优先
            if (local_tick) begin
                if (rpt_cnt >= RPT_FIRST) begin
                    rpt_vld <= 1'b1;
                    rpt_cnt <= RPT_FIRST - RPT_EVERY;   // 回退 ⇒ 之后每 RPT_EVERY 连发一次
                end else
                    rpt_cnt <= rpt_cnt + 6'd1;
            end
        end else
            rpt_cnt <= 6'd0;                           // 松开 / 退出编辑 ⇒ 重新计持按时间
    end
end

// 有效"加/减"：真实按键 ∨ 连发脉冲（连发只在编辑态有效）
wire k_up_e = k_up   || (rpt_vld &&  rpt_up && edit);
wire k_dn_e = k_down || (rpt_vld && ~rpt_up && edit);

// （2026-10-30 删除 fx_next：主画面的 KEY2 改成打开缩放 OSD，
//   换过渡效果只剩菜单里那一条路径，这个快捷轮换的中间量没有读者了）

//------------------------------------------------------------------------------
//  2d. 相册页"上一张/下一张"：KEY2 在按钮行上只发**一拍**
//------------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        album_prev <= 1'b0;
        album_next <= 1'b0;
    end else begin
        album_prev <= k_ok & focus & (page == 3'd1) & (item == 3'd0) & ~edit;
        album_next <= k_ok & focus & (page == 3'd1) & (item == 3'd1) & ~edit;
    end
end

//------------------------------------------------------------------------------
//  3. 目标值（组合）
//------------------------------------------------------------------------------
// 选中行在 5 行窗口里的位置（0..4，越界当 0）
wire [2:0] sel_k  = (item >= scroll_row) ? (item - scroll_row) : 3'd0;

// ★ 每页实际最后一项（2026-10-26）：显示页删除"退出菜单"按钮后只有 6 项
//   （item 0..5），其余页仍是 7 项。k_down 的边界用它，不再用 NITEM-1。
// ★ 每页实际最后一项（2026-10-27）：相册改成 6 项（0..5），文件页空白（0 项），
//   显示页 0..5，关于页 0..6。
// ★ 2026-10-29：系统页改成 5 项（0..4），正好占满 5 行窗口 ⇒ 这一页永不滚动。
wire [2:0] item_last = (page == 3'd0) ? 3'd5 :
                       (page == 3'd1) ? 3'd5 :
                       (page == 3'd2) ? 3'd0 :
                       (page == 3'd3) ? 3'd4 : 3'd6;
// ★ 2026-10-27：`page_empty` 已删除。
//   原来它有两个用途：① 整页 Label（cell_type 已经覆盖，与它无关）；
//   ② 在 cell4 处把游标槽清空 ⇒ 进文件页后**没有任何选中指示**
//   （用户报的就是"文件选项没有 cursor"）。
//   文件页目前是空白页，但光标本身仍有意义：它表示"键盘焦点在页面里，
//   KEY1 回导航栏"，与其它页保持一致 ⇒ 现在和其它页共用同一套游标绘制。

// ★ cy 的目标用"窗口内行号"而不是"绝对行 y"：
//   绝对 y = ROW0 + item*80 - sc_px 会在滚动过程中逐帧变化 ⇒ 进度被反复清零
//   ⇒ 游标在滚动时**一步都不动**。用 sel_k 后滚动期间目标恒定，游标稳稳停在
//   窗口里该行的位置，列表在它下面滑过去 —— 这才是正确的观感。
wire [9:0] tgt_cy  = focus ? (ROW0 + ({7'd0, sel_k} * PITCH) - 10'd6)
                           : (NAV_CY0 + ({7'd0, nav_idx} * PITCH));
wire [9:0] tgt_cx  = focus ? 10'd104 : 10'd28;
wire [9:0] tgt_cw  = focus ? 10'd522 : 10'd40;
wire [9:0] tgt_ch  = focus ? 10'd28  : 10'd40;
// ★★★ 滚动目标 = scroll_row × 行距，直接跟着 scroll_row 走 ★★★
//   原来写成"item 越出窗口才算新目标，否则保持 sc_t 不变" —— 可是进页面时
//   item/scroll_row 被归零，item=0 落在窗口内 ⇒ 走"保持"分支 ⇒ sc_t 死守
//   上一次的 160，而行索引已经是 0。于是 行 y = ROW0 + 0 - 160 = -96、
//   文字 y 同样 -96，整页控件和文字全被顶到屏幕外
//   （上板现象："滚到最底部→回导航栏→再进入，控件错位、文字不显示"）。
//   改成直接跟 scroll_row 之后，归零这一路自然也被覆盖。
wire [9:0] tgt_scroll = {7'd0, scroll_row} * PITCH;

//------------------------------------------------------------------------------
//  4. 动画通道
//------------------------------------------------------------------------------
//  ★★ 6 条通道共用**一套** tween/ease，靠 ch_i 在 L_TWN 里分时服务（6 拍）。
//     原来 6 套并行：6×(16×9 乘法 + 9×9 平方 + 16 位加减) ⇒ osd_menu 里
//     14 个 DSP + 617 条进位链，是本模块最贵的一块。
//     动画每帧只走一步、帧间有 83 万个 50MHz 周期，多花 5 拍毫无代价。
//     代价只是"进度用新值" ⇒ 整条曲线提前一帧起步，观感逐帧相同。
reg [2:0] ch_i;
// ★ 两个"立刻到位"用的边沿/标志（全部只在本 always 块里驱动，避免多驱动）：
//   · edit_d / focus_d：用来取上升沿
//   · sc_snap：进页面那一下把滚动偏移直接归位，不做动画
reg         edit_d, focus_d, sc_snap;

wire [15:0] tw_c = (ch_i == 3'd0) ? cx_c :
                   (ch_i == 3'd1) ? cy_c :
                   (ch_i == 3'd2) ? cw_c :
                   (ch_i == 3'd3) ? ch_c :
                   (ch_i == 3'd4) ? sc_c : sv_c;
wire [15:0] tw_t = (ch_i == 3'd0) ? cx_t :
                   (ch_i == 3'd1) ? cy_t :
                   (ch_i == 3'd2) ? cw_t :
                   (ch_i == 3'd3) ? ch_t :
                   (ch_i == 3'd4) ? sc_t : sv_t;
wire [15:0] tw_f = (ch_i == 3'd0) ? cx_f :
                   (ch_i == 3'd1) ? cy_f :
                   (ch_i == 3'd2) ? cw_f :
                   (ch_i == 3'd3) ? ch_f :
                   (ch_i == 3'd4) ? sc_f : sv_f;
//------------------------------------------------------------------------------
//  ★ 2026-10-29：tween 的乘法 = 一个显式 EG_PHY_MULT18（16 位 × 9 位）
//------------------------------------------------------------------------------
//  原实现是 `tween()` 函数里的 25×25 推断乘法（4 个 DSP，见 §0 的说明）。
//  这里把函数拆成"差值 / 缓动 / 乘积 / 叠加"四段，乘积落在原语上：
//      d = |to − from| ≤ 65535        （16 位）
//      e = ease_of(f[15:8]) ≤ 256     （9 位）
//      p = d × e ≤ 16776960           （25 位）
//      out = to ≥ from ? from + p>>8 : from − p>>8
//  **算术与原来逐位相同**，只是乘法器变窄 ⇒ 输出不变、DSP 从 4 降到 1。
//  ★ 组合逻辑（INPUTREG/OUTPUTREG 全 DISABLE）：与原函数的零延迟一致，
//    L_TWN 的 6 拍节奏不需要改。
wire        tw_up = (tw_t >= tw_c);                 // 与原 tween() 同一判据（无符号）
wire [15:0] tw_d  = tw_up ? (tw_t - tw_c) : (tw_c - tw_t);
wire [8:0]  tw_e  = ease_of(tw_f[15:8]);
wire [35:0] tw_p36;

EG_PHY_MULT18 #(
    .INPUTREGA ("DISABLE"),      // 组合：不加输入寄存器
    .INPUTREGB ("DISABLE"),
    .OUTPUTREG ("DISABLE"),      // 组合：不加输出寄存器（L_TWN 节奏不变）
    .SRMODE    ("ASYNC"),
    .MODE      ("MULT18X18C")
) u_tw_mult (
    .acout   (),
    .bcout   (),
    .p       (tw_p36),
    .signeda (1'b0),             // 无符号 × 无符号
    .signedb (1'b0),
    .a       ({2'b00, tw_d}),    // 16 位 → 18 位零扩展
    .b       ({9'd0,  tw_e}),    // 9 位  → 18 位零扩展
    .acin    (18'd0),
    .bcin    (18'd0),
    .cea     (1'b1),
    .ceb     (1'b1),
    .cepd    (1'b1),
    .clk     (clk),
    .rstan   (1'b1),
    .rstbn   (1'b1),
    .rstpdn  (1'b1),
    .sourcea (1'b1),             // ★ 1 = 选 a（0 会选 acin，乘出 0）
    .sourceb (1'b1)              // ★ 1 = 选 b
);

wire [24:0] tw_p = tw_p36[24:0];
wire [15:0] tw_o = tw_up ? (tw_c + tw_p[23:8]) : (tw_c - tw_p[23:8]);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        cx_f<=16'd0; cw_f<=16'd0; ch_f<=16'd0;
        cy_f<=16'd0; sc_f<=16'd0; sv_f<=16'd0;
        cx_c<=16'd28; cw_c<=16'd40; ch_c<=16'd40;
        cy_c<=16'd20; sc_c<=16'd0; sv_c<=16'd0;
        edit_d<=1'b0; focus_d<=1'b0; sc_snap<=1'b0;
        // ★ ch_i 的复位**不在这里**：它属于布局状态机（下面那个 always 块）。
        //   两个 always 块同时给同一个 reg 赋值 ⇒ TD 报
        //   "net 'ch_i[2]' is constantly driven from multiple places"。
    end else begin
        if (tgt_cx != cx_t) cx_f <= 16'd0;
        else if (anim_step) cx_f <= sat(cx_f, STEP_C);

        if (tgt_cw != cw_t) cw_f <= 16'd0;
        else if (anim_step) cw_f <= sat(cw_f, STEP_C);

        if (tgt_ch != ch_t) ch_f <= 16'd0;
        else if (anim_step) ch_f <= sat(ch_f, STEP_C);

        if (tgt_cy != cy_t) cy_f <= 16'd0;
        else if (anim_step) cy_f <= sat(cy_f, STEP_C);

        if ({tgt_scroll, 4'd0} != sc_t) sc_f <= 16'd0;
        else if (anim_step) sc_f <= sat(sc_f, STEP_S);

        if ({5'd0, sel_val, 4'd0} != sv_t) sv_f <= 16'd0;
        else if (anim_step) sv_f <= sat(sv_f, STEP_V);

        // ★ 进页面（NAV→PAGE）：滚动偏移直接归位，不做 16 帧滑行。
        //   否则重新进页面时 item/scroll_row 已归零、sc_px 还停在上一次的 160，
        //   控件先被顶出屏幕再滑回来，观感就是"错位/文字消失"。
        focus_d <= focus;
        if (focus & ~focus_d)                                        sc_snap <= 1'b1;
        if (lay_st == L_TWN && ch_i == 3'd4)                         sc_snap <= 1'b0;

        // 插值：L_TWN 每拍算一条通道，6 拍走完（tw_o 只有一份硬件）
        if (lay_st == L_TWN) begin
            case (ch_i)
                3'd0: cx_c <= tw_o;
                3'd1: cy_c <= tw_o;
                3'd2: cw_c <= tw_o;
                3'd3: ch_c <= tw_o;
                3'd4: sc_c <= sc_snap ? {tgt_scroll, 4'd0} : tw_o;
                default: sv_c <= tw_o;
            endcase
        end

        // ★ 进编辑（edit 上升沿）：值动画直接对齐到当前值。
        //   不然刚按下"进入"的那十几帧，数字/滑条会从上次遗留的 sv_c 滑过来。
        edit_d <= edit;
        if (edit & ~edit_d) sv_c <= {5'd0, sel_val, 4'd0};
    end
end

// 游标当前几何
wire [9:0] cur_x = cx_c[9:0];
wire [9:0] cur_w = cw_c[9:0];
wire [9:0] cur_y = cy_c[9:0];
wire [9:0] cur_h = ch_c[9:0];

wire [3:0] sit_type = cell_type(page, item);
wire       pop_open = focus & edit & (sit_type == 4'd6);
wire [2:0] pop_sel  = sel_val[2:0];
// 当前打开的下拉属于哪个选项组（0=低中高，1=风格，2=切换效果，3=自动切换）
wire [1:0] pop_set  = ((page == 3'd0) && (item == 3'd4)) ? 2'd1 :
                      ((page == 3'd1) && (item == 3'd2)) ? 2'd2 :
                      ((page == 3'd1) && (item == 3'd3)) ? 2'd3 : 2'd0;
wire [2:0] pop_nopt = nopt_of(pop_set);

// ★★★ 加减一个值：必须**看类型**，不能看行号 ★★★
//   原来写成 case(item)，行号 3 走翻转、行号 6 走 0..100 ⇒ page1 第 7 行（信息条，
//   CheckBox）按上会一路数到 100，勾选框却一直是勾 —— 又一个"控件无法操作"。
//   现在按 sit_type 定规则：CheckBox 翻转、ComboBox 0..选项组项数-1、其余 0..100，
//   再按 item 写回对应的值寄存器。类型表改了也不会再错。
reg [6:0] nval;
always @(*) begin
    if (sit_type == 4'd5) begin                       // CheckBox：上/下都是翻转
        nval = (sel_val == 7'd0) ? 7'd1 : 7'd0;
    end else if (sit_type == 4'd6) begin              // ComboBox：0..nopt-1（按所属选项组）
        // ★★★ 2026-10-27：KEY4=上、KEY3=下（用户明确要求）★★★
        //   原来的方向反了：KEY4 让选项值 +1（向下走），KEY3 让值 -1。
        //   在列表末尾按 KEY4 会被夹紧 ⇒ 看起来"KEY4 无效"。
        nval = k_up_e ? ((sel_val > 7'd0)                ? sel_val - 7'd1 : sel_val)
             : k_dn_e ? ((sel_val < (pop_nopt - 3'd1))  ? sel_val + 7'd1 : sel_val)
             :          sel_val;
    end else begin                                    // TrackBar / NumericUpDown：0..100
        // ★ 步进 TB_STEP=5：两端夹紧到 0/100（起点 50 是 5 的倍数 ⇒ 值恒为 5 的倍数）
        // ★ 2026-10-26 按键语义：**滑条反着来** —— KEY3(k_dn_e) 增大、KEY4(k_up_e) 减小。
        //   导航仍然是 KEY4 上 / KEY3 下（那是"光标往上走"，符合直觉）；
        //   但调值时用户预期"上面的键加、下面的键减"，所以这里单独反向。
        nval = k_dn_e ? ((sel_val + TB_STEP > 7'd100) ? 7'd100 : sel_val + TB_STEP)
             : k_up_e ? ((sel_val < TB_STEP)         ? 7'd0   : sel_val - TB_STEP)
             :          sel_val;
        // ★ 对比度硬下限 20：值为 0 时后处理表会是"全 128 平表"，连 OSD 一起看不见
        //   （后处理在 OSD 合成之后）。UI 这层先挡住，postproc 里还有一道同样的下限。
        if ((page_r == 3'd0) && (item == 3'd1) && (nval < 7'd20))
            nval = 7'd20;
    end
end

// ★★★ 下拉弹层：紧凑列表（行距 34 / 行高 30），锚在 ComboBox 那一行 ★★★
//   几何：第 k 项的 y = pop_y + k*34，行高 30（行间留 4px）。
//
//   ★ 位置由"**当前选中项**"决定，不是由"行在窗口里的位置"决定：
//       pop_y = combo_y - pop_sel*34 - 3
//     于是**选中项永远压在 ComboBox 那一行上**（和桌面系统一致）。
//     原来按 sel_k≤2 决定向下/向上翻，导致压在控件行上的永远是"第一项"
//     （或向上翻时的"最后一项"）⇒ 屏幕上读成"色温 低"，可当前值其实是"中"，
//     而高亮的"中"却在下一行 —— 这就是"下拉看起来对不上"的主因。
//
//   ★ 为什么 -3：菜单行文字 y = 控件顶边 + 4；选项字在 30px 行里居中
//     ⇒ 偏移 (30-16)/2 = 7。要让选中项和菜单文字**同一基线**：
//        pop_y + pop_sel*34 + 7 = combo_y + 4
//     ⇒ pop_y = combo_y - pop_sel*34 - 3
//     （不加这个 -3 时实测选中项比其他两项高 3px。）
// ★★★ 行距必须是 32，不能是 34 ★★★
//   文字层的一个字格是 16px 高，而菜单行距 80。选项字格的位置是
//       comboText + (k - pop_sel) * POP_PITCH
//   最外侧那一项离控件行 2*POP_PITCH。要和相邻菜单行的字格**不重叠**，必须
//       2*POP_PITCH <= 80 - 16 = 64   ⇒  POP_PITCH <= 32
//   原来用 34：2*34 = 68，离菜单行只剩 80-68 = 12 < 16 ⇒ 两个 16px 字格
//   在垂直方向重叠 4 行，后画的那一行会把前一行**擦掉 4 行像素**。
//   上板/仿真现象就是"低"的最上面 4 行没了（实测字格 236..251，墨迹只剩
//   240..251，缺的正好是 236..239 = 与菜单行 224..239 的重叠区）。
//   down 方向同理："高"缺最下面 4 行。
//   取 32 时 2*32 = 64 = 80-16，两个字格**正好相切**，一行都不缺。
localparam [9:0] POP_PITCH = 10'd32;
localparam [9:0] POP_H     = 10'd28;          // 行距 32 - 4px 间隙
localparam [9:0] POP_TY    = 10'd6;           // 字在行内居中：(28-16)/2

// ★★★ 锚定项数 pop_a：上方放不下就整列下移 ★★★
//   菜单窗口第 0 行的控件顶边是 y=56（combo_y=60），上面就是标题(24)和标题线(44)。
//   如果照 pop_sel 往上展开：sel_k=0 且 pop_sel=1 → pop_y = 26（面板 20..124），
//   直接压住标题；pop_sel=2 → pop_y = -6，整个第一项跑到屏幕外。
//   上板现象就是"滚到底部再开色温的下拉，弹层顶端顶到标题栏"。
//   ⇒ 上方能放的项数 = floor((combo_y-56)/32)：sel_k=0 时是 0，sel_k≥1 时 ≥2。
//   ★ 必须按 32 的**整数倍**收，不能按像素夹：三项文字间距是 32、菜单行距是 80，
//     两者都是 16 的倍数 ⇒ 只要偏移是 32 的整数倍，文字永远不会和菜单行字格相交。
//   副作用：sel_k=0 且 pop_sel>0 时"选中项"不再压在控件行上（被下移到第一/第二项），
//   这时控件行的标签也一并清空，避免读成"色温 低"而高亮在"中"。
// ★★ 2026-10-27：多项下拉（5/6 项）的**滚动窗口** ★★
//   弹层仍然只有 3 行（cell 16..18 / 文字第 6、7 行），窗口跟着选中项走：
//      pop_a = 选中项在窗口里的位置（0..2，上方放不下时强制 0）
//      pop_w0 = 窗口首项的项号；第 k 行显示第 pop_w0+k 项，越界则不画
wire [2:0] pop_a  = (sel_k == 3'd0) ? 3'd0 : ((pop_sel < 3'd2) ? pop_sel : 3'd2);
wire [2:0] pop_w0 = pop_sel - pop_a;
wire [3:0] pop_row  = {1'b0, scroll_row} + {1'b0, sel_k};                   // 控件绝对行号
wire [9:0] combo_y  = ROW0 + ({6'd0, pop_row} * PITCH) - sc_px - 10'd4;     // 该行控件顶边
//   -2 = 让"锚定项的字"和菜单行的字同基线：
//         pop_y + pop_a*32 + 6 = combo_y + 4  ⇒  pop_y = combo_y - pop_a*32 - 2
wire [9:0] pop_y    = combo_y - ({7'd0, pop_a} * POP_PITCH) - 10'd2;        // 第 0 项 y

// ★★★ 三项文字怎么落到文字行上 ★★★
//   NL 6→8 新增的第 6、7 行是**弹层专用备用行**（没有标签、没有控件），
//   选中项写控件行自己的第 14 列（x=336，标签仍在第 0..3 列），
//   另外两项写到第 6、7 行 ⇒ 一条菜单行的标签都不用让。
//   哪些项去第 6 行、哪些去第 7 行，随 pop_sel 变（保证"上去的在上、下来的在下"）：
//     pop_a=0 → 6 行=第1项, 7 行=第2项      （两项都在下方）
//     pop_a=1 → 6 行=第0项, 7 行=第2项      （一上一下）
//     pop_a=2 → 6 行=第0项, 7 行=第1项      （两项都在上方）
wire [2:0] pop_la   = (pop_a == 3'd0) ? 3'd1 : 3'd0;        // 第 6 行装窗口第几行
wire [2:0] pop_lb   = (pop_a == 3'd2) ? 3'd1 : 3'd2;        // 第 7 行装窗口第几行
wire [9:0] pop_y6   = pop_y + ({7'd0, pop_la} * POP_PITCH);
wire [9:0] pop_y7   = pop_y + ({7'd0, pop_lb} * POP_PITCH);
// 窗口行是否真的有项（末页项数不足时，下方/上方留空）
wire       pop_v6   = ((pop_w0 + pop_la) < pop_nopt);
wire       pop_v7   = ((pop_w0 + pop_lb) < pop_nopt);

// ★★★ 面板盖住了哪几行 ⇒ 那几行的控件不画、文字不写 ★★★
//   行的控件范围 = combo_y + (r-sel_k)*80 .. +24；面板 = pop_y-6 .. pop_y+104。
//   代入 pop_y 后判重叠，得到（与 pop_sel 有关）：
//     pop_a=0 → sel_k, sel_k+1     pop_a=1 → 只有 sel_k
//     pop_a=2 → sel_k-1, sel_k
//   等价写法：wx0 = (pop_sel==2) ? sel_k-1 : sel_k
//             wx1 = (pop_sel==0) ? sel_k+1 : sel_k
//   （结果 >4 时自然落空，不用额外判断。）
//  ★ 2026-10-29：弹层实际用了几行（1..3）。语言下拉只有 2 项 ⇒ nrow=2，
//    面板必须跟着变矮：写死 3 行会让面板底下多出一条**空行**，
//    并且把下面那一行菜单（音量）盖住、逼着 pop_wx1 把它清空 ⇒ 屏幕上"缺一行"。
//    nopt≥4 时 nrow 恒为 3，与改造前逐像素一致。
wire [3:0] pop_fill = {1'b0, pop_nopt} - {1'b0, pop_w0};
wire [2:0] pop_nrow = (pop_fill > 4'd3) ? 3'd3 : pop_fill[2:0];
wire [3:0] pop_below = {1'b0, pop_nrow} - {1'b0, pop_a};

wire [2:0] pop_wx0  = (pop_a == 3'd2) ? (sel_k - 3'd1) : sel_k;
wire [2:0] pop_wx1  = (pop_below >= 4'd3) ? (sel_k + 3'd1) : sel_k;

// 第 l 行文字的 y / x：
//   · 第 6、7 行 = 弹层备用行（菜单模式）/ 信息条两行（信息条模式）
//   · 其余行按菜单行距（第 0 行是标题，固定 y=24）
// ★ 2026-10-27：返回值从 16 位收成 11 位，和 txt_xy 的 22 位字段对齐。
//   （Verilog 不允许 `ty_of(x)[10:0]` 这种"函数调用后位选"的写法，
//     所以只能直接改返回类型。）
//   【为什么是 11 位而不是 10 位】实测范围是 −176 .. 624：
//     · 滚动动画中（scroll_row=3、sc_px 还没跟上）可到 64+240+320 = 624
//     · sc_px 大于 srow_px 时可到 64+0+0−240 = −176
//     ⇒ 需要 −1024..1023 的补码范围。10 位只有 −512..511，装不下 624。
function [10:0] ty_of;
    input [3:0] l;
    begin
        if (l == 4'd6)      ty_of = menu_on ? (pop_y6 + POP_TY) : 11'd12;
        else if (l == 4'd7) ty_of = menu_on ? (pop_y7 + POP_TY) : 11'd32;
        else if (l == 4'd0) ty_of = 11'd24;
        else                ty_of = ROW0 + srow_px + ((l - 4'd1) * PITCH) - sc_px;
    end
endfunction

function [10:0] tx_of;
    input [3:0] l;
    begin
        // ★ 2026-10-30：缩放 OSD 的面板从 x=8 起，而 ASCII 字形是半宽、在 16px
        //   格内偏左，起点 16 会把第一个字符左半切掉（实测显示成 "-OOM"）。
        //   缩放时整体右移 8px；信息条不受影响（它面板 8..328，容得下）。
        if ((l == 4'd6) || (l == 4'd7))
            tx_of = menu_on ? 11'd336 : (zoom_on ? 11'd24 : 11'd16);
        else                            tx_of = 11'd112;
    end
endfunction

//------------------------------------------------------------------------------
//  5. 控件表内容
//------------------------------------------------------------------------------
reg [9:0] gx0, gy0, gx1, gy1;
reg [3:0] gt, gs, grad;
reg [1:0] gpm;          // 圆角外侧继承哪一层：0=面板/左栏 1=遮罩 2=弹层底板
reg [9:0] gv;
reg [2:0] icx;

// 信息条进度条填充宽（0..287）：只有"自动切换"选了 3s/5s/10s/30s 才显示
wire [9:0] info_bar_v = ((auto_mode >= 3'd1) && (auto_mode <= 3'd4))
                        ? {1'b0, auto_bar} : 10'd0;

// ★ ridx 用 wire 表达，避免"在 always@* 里先赋值、再被组合逻辑回头引用"
wire [2:0] ridx_ctrl = (cell_i >= 5'd10 && cell_i <= 5'd14) ? (scroll_row + (cell_i - 5'd10))
                                                           : (scroll_row + cell_i[2:0]);
// 当前正在排的那一行（ridx_ctrl）的值（同上，显式 case 让敏感表带上 val*）
reg [6:0] row_val;
always @(*) begin
    case (ridx_ctrl)
        3'd0: row_val = val0;
        3'd1: row_val = val1;
        3'd2: row_val = val2;
        3'd3: row_val = val3;
        3'd4: row_val = val4;
        3'd5: row_val = val5;
        default: row_val = val6;
    endcase
end

//------------------------------------------------------------------------------
//  ★★★ 2026-10-27：`cell_type` 三处调用合并成两处（省一整套 5×8 case 树）★★★
//------------------------------------------------------------------------------
//  【原来】`cell_type(page, X)` 被例化了三次：
//      · sit_type = cell_type(page, item)        —— 按键判定 / 弹层几何（任何时刻）
//      · gt       = cell_type(page, ridx_ctrl)   —— 控件表，只在 L_CELL 有意义
//      · vtyp     = cell_type(page, vidx)        —— 文字表，只在 L_TXT  有意义
//    每套都是一个 (5 页 × 8 行) 的 case 树，综合出来是三条并行的比较/选择链。
//  【为什么能合】`gt` 与 `vtyp` **永远不会同时被消费**：
//      `wr_en = (lay_st == L_CELL)`，所以 L_TXT 期间 gt 无意义；
//      而 `vtyp` 只喂给 `txt_ch`，`str_code <= txt_ch` 只在 L_TXT 发生。
//    ⇒ 用一个 mux 按 `lay_st` 选行号，两处共用同一个 cell_type 结果。
//  【为什么必须把 vidx 提到这里】verilog 要求先用后声明在 TD 里是 warning、
//    在 iverilog 里是硬错；`gt` 在下面的 always 里就要用 ct_sel，
//    所以 vidx / ct_row / ct_sel 必须定义在它之前。
//  【sit_type 不能合】它用的是 `item`（当前选中行），而布局阶段 ct_row 是
//    另一回事，两者必须并存。
// ★ 2026-10-28：T1 的 ROM 是"地址组合 + 1 拍输出"，而 txt_l 在**行末那拍**
//   才递增（非阻塞）⇒ 那一拍的 tsrc_a 还指向旧行 ⇒ rom_q64 在 col=0 那拍
//   输出的是**上一行**的数据，col=1 起才追上。
//   上板症状：每行第 1 个字来自上一行（"显示"→"显度"、"亮度"→"亮比度"）。
//   修法：地址用"下一拍的行号"（行末那拍提前切到新行），col=0 就拿到新行。
wire [3:0] txt_l_pre = (lay_st == L_TXT && txt_c > LASTC && txt_l != LASTL)
                     ? (txt_l + 4'd1) : txt_l;
wire [2:0] vidx   = (txt_l_pre == 4'd0) ? 3'd0 : (scroll_row + txt_l_pre[2:0] - 3'd1);
wire [2:0] ct_row = (lay_st == L_TXT) ? vidx : ridx_ctrl;
wire [3:0] ct_sel = cell_type(page, ct_row);

// ★★★ 只有**正在编辑**的那一行才用动画值 ★★★
//   原来写的是 (focus && 选中行) ⇒ 一换行，新选中行就顶着**上一行**的值做
//   16 帧过渡：滑条填充和数字从 60 一路缩到 50，看着像"换行把值改了"。
//   值动画的唯一用途是"按住上下键调值时数字平滑走"，所以判据必须是 edit，
//   不是 focus。（edit 上升沿还会把 sv_c 直接对齐，见动画块。）
wire [6:0]  vdisp = (edit && (ridx_ctrl == item)) ? sv_disp[6:0] : row_val;
// 轨道实际跨度 = CTRL_W - 9 = 287 px ⇒ 0..100 → 0..287（×737 后 >>8）。
// ★ 737 = 512+128+64+32+1 ⇒ 用移位加代替乘法，省一个 10×10 乘法器（约 100 LUT）。
wire [16:0] vprod = ({3'd0, vdisp} << 9) + ({3'd0, vdisp} << 7)
                  + ({3'd0, vdisp} << 6) + ({3'd0, vdisp} << 5) + {3'd0, vdisp};
wire [9:0] vfill = vprod[16:8];                      // >>8 → 0..287（9 位够）

// ★ [S6] 这一格是不是"动态槽"，以及它进 cellmem 的固定下标
//   常量槽（0..3 / 5..9 / 15 / 19）不写 cellmem —— 由 osd_gfx 用常量矩形命中。
wire cell_is_dyn = (cell_i == 5'd4)
                || (cell_i >= 5'd10 && cell_i <= 5'd14)
                || (cell_i >= 5'd16 && cell_i <= 5'd18);
wire [4:0] cell_dyn_idx = (cell_i == 5'd4)  ? 5'd0 :
                          (cell_i <= 5'd14) ? (cell_i - 5'd9) :   // 10..14 → 1..5
                                              (cell_i - 5'd10);   // 16..18 → 6..8

always @(*) begin
    gx0 = 10'd0; gy0 = 9'd0; gx1 = 10'd0; gy1 = 9'd0;
    gt = 4'd0; gs = 4'd0; grad = 4'd0; gpm = 2'd0; gv = 10'd0;
    icx = 3'd0;
    // 层级（索引大者在上）：遮罩0 → 左栏1 → 分隔2 → 页面板3 → ★游标框/弹层板4
    //   → 图标5..8 → 行控件/选项行9..13 → 标题线14        （共 15 槽，原 24 槽）
    // ★ [S6b] 试过在这里加 `if (cell_is_dyn)` 把常量槽的分支跳过（想让 45 位 mux
    //   从 20 路缩到 9 路）—— 实测 17048→17084（**+36，变差**）：
    //   TD 会给整条 case 加一个旁路使能，"跳过的分支"并没有真的消失。
    //   已回退，保留原来的 20 路 case（wr_en 已经把常量槽挡住，不写进去）。
    case (cell_i)
        5'd0: begin gx0=0;   gy0=0;  gx1=10'd640; gy1=9'd480; gt=4'd0;
                    gs  = (SELFTEST != 0) ? 4'd15 : 4'd2;   // 15=纯红(自检) / 2=50% 遮罩
                    grad=0; end
        5'd1: begin gx0=0;   gy0=0;  gx1=10'd96;  gy1=9'd480; gt=4'd0; gs=4'd1;  grad=0; end
        // ★ 分割线加宽到 3px（96..99），并且**只有这一个槽**画它：
        //   页面板改成 Solid（不再有 1px 边框）⇒ 这条竖带里只有一个颜色、
        //   只有一个矩形边界，"两个 1px 相邻特征对不齐"造成的杂纹从源头消失。
        5'd2: begin gx0=96;  gy0=0;  gx1=10'd99;  gy1=9'd480; gt=4'd0; gs=4'd10; grad=0; end
        // ★ gx0 从 100 收到 98：原来 96..98 是分隔线、98..100 是 2px 空隙
        //   （只由遮罩覆盖）⇒ 上板看到"亮线 + 暗缝 + 面板边框"三条竖带，
        //   就是"左右侧分割线显示异常"。收到 98 后面板边框正好接上分隔线，
        //   96/97/98 三列同色（3A3A3A）连成一条 3px 的分割线。
        // ★ gpm=1：它的四个圆角外侧要继承**遮罩**，不是面板色（否则顶部两角
        //   会在遮罩上凸出两块不透明角 ⇒ "Page 框上方外部区域实心填充"）。
        // ★ 圆角去掉（grad=0）：上板反馈"额外圆角，不需要"。页面板上边/左边是贴着
        //   分隔线和屏幕上沿的，圆角反而在分隔线顶端切出一个缺口 + 一段 mask 色，
        //   就是那圈"杂像素"。去掉圆角后既没有缺口，也不再需要 gpm 继承遮罩色。
        // ★★★ 页面板：gy0 12 → 0，gt Panel → Solid，grad=0 ★★★
        //   ① 原来 gy0=12，屏幕最上面 12px 只被全屏遮罩（50%）覆盖，而面板是 75%
        //      ⇒ 顶端一条明显更亮的横带，上板看起来就是"顶端没有 alpha 层、
        //      只有全透明"。现在面板从 y=0 开始，这条带子彻底不存在。
        //   ② 改成 Solid：页面板原来用 Panel（自带 1px 3A3A3A 边框），而它的左边框
        //      正好压在分隔线旁边 ⇒ 同一条竖带里有两个矩形、两种取色路径，
        //      只要 1px 对不齐就出现"亮线/杂像素"。现在边框由分割线槽统一负责。
        //   ③ grad 必须是 0：圆角只在 rq≠0 时才会切（见 osd_widget 的 incorn），
        //      所以 grad=0 就绝对不会有圆角。gpm 也随之无意义。
        5'd3: begin gx0=99;  gy0=0;   gx1=10'd640; gy1=9'd480; gt=4'd0; gs=4'd1;  grad=4'd0;
                    gpm=2'd0; end
        5'd4: begin
            // ★★★ 游标 + 行高亮合并成一个槽（原来 1 个高亮槽 + 4 个空心边槽）
            //   type 11 会把整个矩形填满：2px 白边 + 内部 = 样式表颜色。
            //   内部色取"它下面那一层"的颜色即可逐像素无缝：
            //     focus=1 → 下面曾是行高亮 style9(404040@50%) ⇒ gs = 9
            //     focus=0 → 下面就是左栏   style1(0E0E0E@75%) ⇒ gs = 1
            //   之所以能压到索引 4（在图标/行控件**之下**），是因为游标边框与
            //   它们在几何上根本不相交（逐项核对见 osd_widget.v 头部注释）。
            if (pop_open) begin
                // ★ 弹层底板：正好包住三个紧凑选项行（行距 34、行高 30），
                //   左右各 8px、上下各 6px ⇒ 面板高 = 2*34 + 30 + 12 = 104。
                // ★ 左右各留 8px：原来右边贴着 616，选项行直接顶到面板边缘，
                //   看着"缺右 padding"。现在面板 312..624，选项行 320..616 ⇒ 左右对称。
                gx0 = CTRL_X - 10'd8;
                gx1 = CTRL_X + CTRL_W + 10'd8;
                // 上下各 6px（对称）。被弹层压住的那两行控件已经整行不画，
                // 所以不需要靠"多留 12px"去盖住它们的边框。
                gy0 = pop_y - 10'd6;
                // ★ 2026-10-29：高度按"实际用了几行"算（原来写死 3 行）。
                //   2 项的语言下拉 → 面板只包 2 行，不再多出一条空行、也不再
                //   把下面的音量行顶掉。nrow=3 时 (nrow-1)*32 = 64，与原来逐像素相同。
                gy1 = pop_y + ({7'd0, pop_nrow - 3'd1} * POP_PITCH) + POP_H + 10'd6;
                // ★ 改成"无边实心"：原来用 Panel（1px 4A4A4A 边框），
                //   那条边框离最后一个选项只有 6px，看着像选项下面多一根亮线。
                gt = 4'd0;                             // Solid：整块一个颜色，无边框
                gs = 4'd14; grad = 4'd6;
            end else begin
                gx0 = cur_x;           gy0 = cur_y;
                gx1 = cur_x + cur_w;   gy1 = cur_y + cur_h;
                gt  = 4'd11;
                gs  = focus ? 4'd9 : 4'd1;
                grad= 4'd8;
            end
        end
        5'd5, 5'd6, 5'd7, 5'd8, 5'd9: begin          // 5 个导航图标位
            icx = cell_i - 5'd5;
            // ★ 2026-10-28：24+80i → 72+80i（给左上角 logo 让出 48px）
            gx0 = 10'd32; gy0 = 9'd72 + ({6'd0, icx} * 9'd80);
            gx1 = 10'd64; gy1 = gy0 + 9'd32;
            gt  = 4'd12;                             // Icon：RGB565 直出位图
            // ★ 这里也去掉 focus 判断，和 ic_act_mask 保持一致。
            //   注意：type12 已经不看 style（颜色来自位图），这一行纯粹是
            //   "让属性表和实际显示对上"，避免以后有人照着 gs 去判断状态。
            gs  = (icx == nav_idx) ? 4'd6 : 4'd5;
            grad= 4'd0;                              // 图标是方形贴图，不切圆角
        end
        // ★★★ 5 行控件**永远**照常画，下拉打开时也不隐藏 ★★★
        //   原来 pop_open 时把这 5 个槽全改成"选项行 + 隐藏"，结果下拉一打开，
        //   整个菜单（5 行控件、标签、游标）全没了，只剩一个灰板 —— 上板看到的就是
        //   "下拉内容显示异常"。现在弹层用 15/16/17 三个**更高**的槽，压在这几行
        //   之上，行控件/标签原样留着。
        5'd10, 5'd11, 5'd12, 5'd13, 5'd14: begin      // 行控件
            icx = cell_i - 5'd10;
            gt  = ct_sel;               // = cell_type(page, ridx_ctrl)，见上面说明
            // 三种情况整行不画控件：
            //   ① Label（type 1）行 —— 只留文字，画成 Box 会像"输入框没字"；
            //   ② 弹层**完整盖住**的那两行 —— 弹层已经替代了它们，
            //      不画才能避免"边框从弹层底下探出来"的杂线。
            if ((gt == 4'd1) ||
                (pop_open && (({2'd0, icx} == {2'd0, pop_wx0}) || ({2'd0, icx} == {2'd0, pop_wx1})))) begin
                gx0 = 10'd0; gy0 = 9'd0; gx1 = 10'd0; gy1 = 9'd0;
            end else begin
                gx0 = CTRL_X;          gy0 = ROW0 + ({7'd0, ridx_ctrl} * PITCH) - sc_px - 10'd4;
                gx1 = CTRL_X + CTRL_W; gy1 = gy0 + 10'd24;
                gs  = 4'd11;                        // 75% 半透明底
                grad= 4'd5;
                // ★ TrackBar 和 ProgressBar 都要按百分比填充；Numeric 直接给数值；
                //   勾选/按钮只给"非零"标志。
                if (gt == 4'd4 || gt == 4'd8) gv = vfill;
                else if (gt == 4'd10)         gv = {3'd0, vdisp};
                else                          gv = {9'd0, (row_val != 7'd0)};
            end
        end
        5'd15: begin                                  // 标题下划线
            gx0 = 10'd112; gy0 = 9'd44; gx1 = 10'd624; gy1 = 9'd45;
            gt = 4'd9; gs = 4'd10; grad = 4'd0;
        end
        // ★ 下拉选项：紧凑列表（行距 32 / 行高 28），锚在 ComboBox 那一行。
        //   2026-10-27：窗口滚动 —— 第 k 行显示第 pop_w0+k 项，越界不画。
        5'd16, 5'd17, 5'd18: begin
            icx = cell_i - 5'd16;
            if (pop_open && (({1'b0, pop_w0} + {2'd0, icx}) < {1'b0, pop_nopt})) begin
                gx0 = CTRL_X;
                gy0 = pop_y + ({7'd0, icx} * POP_PITCH);
                gx1 = CTRL_X + CTRL_W;
                gy1 = gy0 + POP_H;
                gt  = 4'd0;
                gs  = (icx == pop_a) ? 4'd13 : 4'd12;
                // 圆角 6 与底板一致：30px 高的行用 8 会像"药丸"，和底板对不上。
                grad= 4'd6;
                gpm = 2'd2;      // 圆角外侧继承"弹层底板"(161616 不透明)
            end else begin                            // 没打开 ⇒ 空矩形（不命中）
                gx0 = 0; gy0 = 0; gx1 = 0; gy1 = 0;
            end
        end
        // ★★★ 2026-10-28：cell19 = 左上角 logo（最高槽位，永远在最上层）★★★
        //   几何固定 (16,8)-(80,49)，和 osd_gfx 拍 0 的地址公式硬绑定：
        //     logo 字地址 = 5120 + {(y-8)[5:0], (x-16)[5:1]}
        //   ⇒ 改这里的 LOGO_X/LOGO_Y/LOGO_W/LOGO_H 必须同步改 osd_gfx 的常量。
        //   它和任何其它槽都不重叠（图标 x≥32 且 y≥72；标题线 x≥112；
        //   弹层 x≥312），所以放在最高索引不会盖掉任何东西。
        //   `!menu_on` 的覆盖段会把 19 号也清零 ⇒ 菜单关掉时 logo 自动消失。
        5'd19: begin
            gx0 = LOGO_X;              gy0 = LOGO_Y;
            gx1 = LOGO_X + LOGO_W;     gy1 = LOGO_Y + LOGO_H;
            gt  = 4'd14;                             // logo：ARGB8888 直出（不同于图标的 12）
            gs  = 4'd5;                              // type14 不用 style，填个合法值
            grad= 4'd0;
        end
        // ★ 信息条两格（面板/进度条）**复用**弹层的 16/17：
        //   菜单开 ⇒ 16/17 是弹层行（信息条本来就要隐藏）；
        //   菜单关 ⇒ 下面覆盖段把 16/17 改写成信息条、其余格清零。
        //   两者永远不同时可见 ⇒ 省 2 格存储（2×63 触发器）+ 2 路命中比较。
        default: begin gx0 = 0; gy0 = 0; gx1 = 0; gy1 = 0; end
    endcase

    // ---- 覆盖 1：信息条模式（菜单关着、信息条开着）----
    //   除了 19/20，其余全部置零矩形。这样 osd_gfx 的命中网络只会命中信息条，
    //   不会被残留的菜单几何"抢"走像素。
    if (!menu_on) begin
        if (zoom_on) begin
            // ★ 2026-10-30 缩放 OSD：只留槽 16 当圆角面板，几何缩到左上角。
            //   圆角参数与信息条完全相同（gt=0/gs=11/grad=6/gpm=3）⇒ 同款观感、
            //   零额外 LUT（本来就是这一个槽在画）。
            if (cell_i == 5'd16) begin
                gx0 = 10'd8; gy0 = 9'd8; gx1 = 10'd160; gy1 = 9'd52;
                gt = 4'd0; gs = 4'd11; grad = 4'd6; gpm = 2'd3; gv = 10'd0;
            end else begin
                gx0 = 10'd0; gy0 = 9'd0; gx1 = 10'd0; gy1 = 9'd0;
                gt = 4'd0; gs = 4'd0; grad = 4'd0; gpm = 2'd0; gv = 10'd0;
            end
        end else if (cell_i == 5'd16) begin
            gx0 = 10'd8; gy0 = 9'd8; gx1 = 10'd328; gy1 = 9'd74;
            // gpm=3：圆角外侧**透明**（露出底图），信息条才有真正的圆角；
            //   菜单面板仍用 0/1/2（继承遮罩/面板色），不受影响。
            gt = 4'd0; gs = 4'd11; grad = 4'd6; gpm = 2'd3; gv = 10'd0;
        end else if (cell_i == 5'd17) begin
            gx0 = 10'd16; gy0 = 9'd52; gx1 = 10'd312; gy1 = 9'd64;
            // grad=0：进度条不要圆角 —— 原来 4 会在轨道两端切出
            //   一圈"面板色的小圆角"，用户报的"多余圆角"。
            gt = 4'd13; gs = 4'd11; grad = 4'd0; gpm = 2'd0; gv = info_bar_v;
        end else begin
            gx0 = 10'd0; gy0 = 9'd0; gx1 = 10'd0; gy1 = 9'd0;
            gt = 4'd0; gs = 4'd0; grad = 4'd0; gpm = 2'd0; gv = 10'd0;
        end
    end
    // ---- 覆盖 2：★ 2026-10-27 删除"文件页不画游标" ----
    //   原来这里有一段
    //       else if (page_empty && (cell_i == 5'd4)) 把游标矩形清零
    //   于是进入"文件"页后屏幕上没有任何"当前选中"的指示（用户报的
    //   "文件选项没有 cursor"）。现在文件页与其它页共用同一套游标绘制。

    // 自检模式下只留 cell0
    if (SELFTEST != 0 && cell_i != 5'd0) begin
        gx0 = 10'd0; gy0 = 9'd0; gx1 = 10'd0; gy1 = 9'd0;
    end

    // ★★ 2026-10-29 [S6]：只把「动态字格」写进 cellmem ★★
    //  20 个槽里有 11 个的矩形是编译期常量（0 遮罩 / 1 左栏 / 2 分割线 /
    //  3 页面板 / 5..9 图标 / 15 标题线 / 19 logo），它们只随 menu_on 开关，
    //  由 osd_gfx 用常量矩形直接命中，不再占 cellmem 的存储和命中比较
    //  （实测每槽约 100 LUT）。剩下 9 个每帧重算的按下面固定映射进 0..8：
    //      4 → 0       10..14 → 1..5       16..18 → 6..8
    //  ★ 这个映射与 osd_gfx 的分组合并、osd_cellmem 的 .NSLOT(9) 必须一致 ★
    wr_en   = (lay_st == L_CELL) && cell_is_dyn;
    wr_idx  = cell_dyn_idx;
    wr_geo  = {1'b1, gx1, gy1[8:0], gx0, gy0[8:0]};
    wr_attr = {gt, gs, grad, gpm, gv};        // 4+4+4+2+10 = 24
end

//------------------------------------------------------------------------------
//  6. 文字行内容
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//  ★ 2026-10-27：行可见性（决定这一行的 ctl_active / ctl_len）
//------------------------------------------------------------------------------
//  【用户报的现象】"信息条显示时会一起显示 OSD 的文字"。
//  【根因】信息条和菜单共用同一张 8 行文字表（同一个 osd_font_cache，
//    NL=8 行、每行 32 格）。`menu_on` 落下后，布局机确实改走"信息条两行"
//    分支去算第 6/7 行的内容，但**第 0..5 行的标题/标签从来没有被关掉** ——
//    每次布局收尾都无条件写 `ctl_active <= 1'b1; ctl_len <= 8'd32;`，
//    于是 ps_gen_text 照样把上一轮菜单的"相册 / 切换效果 / 自动切换 …"
//    渲染在信息条旁边，两层文字叠在同一屏上。
//  【改法】按"这一行此刻有没有内容"给 active/len：
//    · 菜单打开   ：8 行全亮（0=标题，1..5=标签+值，6/7=弹层备用行）
//    · 菜单关闭   ：只有 6/7 两行（文件名 / 分辨率+倒计时）
//    `ctl_len = 0` 会让 ps_gen_text 的 `LHIT = (LX < len_px)` 恒假
//    ⇒ 整行一个字都不画；同时 osd_font_cache 的 `pend` 逻辑会把该行
//    占用的字形槽释放掉（`active=0 且 rdy=1` ⇒ 需要释放），顺带减轻槽位压力。
wire row_vis = menu_on | (txt_l >= 4'd6);

// （vidx 已提到上面的 cell_type 复用段，这里不再重复声明）
// ★★★ vrow = 这一行在**窗口里**是第几行（0..4）★★★
//   弹层几何（sel_k / pop_wx0 / pop_wx1）全是窗口行号，而 vidx 是**绝对 item 号**；
//   两者只在 scroll_row==0 时相等。之前拿 vidx 去比 pop_wx ⇒ 只要滚动过，
//   "被弹层盖住的行"就一条都清不掉（上板：滚到底开下拉，被盖住那行的标签和
//   数值从弹层底下透出来）。文字这一侧一律用 vrow 比。
wire [2:0] vrow = txt_l[2:0] - 3'd1;
wire [3:0] vtyp = ct_sel;           // = cell_type(page, vidx)，见上面说明
// 文字行的值（同样显式 case）
reg [6:0] txt_val;
always @(*) begin
    case (vidx)
        3'd0: txt_val = val0;
        3'd1: txt_val = val1;
        3'd2: txt_val = val2;
        3'd3: txt_val = val3;
        3'd4: txt_val = val4;
        3'd5: txt_val = val5;
        default: txt_val = val6;
    endcase
end
wire [7:0] vsel = (edit && (vidx == item)) ? sv_disp : txt_val;   // 同上：只有在编辑才用动画值
wire [7:0] q10  = ((vsel * 16'd205) >> 11);   // vsel/10

// 该行所属的选项组（收起态值显示用）
//   显示页 item4"风格" → set 1；相册 item2"切换效果" → set 2；item3"自动切换" → set 3
wire [1:0] row_set = ((page == 3'd0) && (vidx == 3'd4)) ? 2'd1 :
                     ((page == 3'd1) && (vidx == 3'd2)) ? 2'd2 :
                     ((page == 3'd1) && (vidx == 3'd3)) ? 2'd3 : 2'd0;


reg [15:0] txt_ch;
// ★ 2026-10-28 T1-v2：txt_src 改为 wire（接 BRAM 输出 rom_q64），见下方 T1 段

//------------------------------------------------------------------------------
//  6b. 信息条用的十进制数字（源分辨率 4 位 BCD + 倒计时 2 位）
//------------------------------------------------------------------------------
//  【为什么用"顺序减法"而不是乘倒数除法】
//  分辨率只在换图时变一次，慢一点无所谓；顺序减法只要一个 16 位减法器 +
//  三个比较器（约 40 LUT），而且对 0..4095 是精确的。
//  W/H 串行复用同一套减法器：先转 W，转完再转 H（约 30 拍）。
localparam [2:0] BS_IDLE=3'd0, BS_TH=3'd1, BS_HU=3'd2, BS_TE=3'd3,
                 BS_ON=3'd4, BS_LAT=3'd5;

reg [2:0]  bs;
reg [15:0] brem;
reg [3:0]  d3, d2, d1, d0;
reg        b_is_h;
reg [15:0] wdig, hdig;
// ★ 2026-10-27：分辨率按"实际位数"紧凑排版（用户报文字间隙太大）
wire [3:0] wlen = (wdig[15:12] != 4'd0) ? 4'd4 :
                  (wdig[11:8]  != 4'd0) ? 4'd3 :
                  (wdig[7:4]   != 4'd0) ? 4'd2 : 4'd1;
wire [3:0] hlen = (hdig[15:12] != 4'd0) ? 4'd4 :
                  (hdig[11:8]  != 4'd0) ? 4'd3 :
                  (hdig[7:4]   != 4'd0) ? 4'd2 : 4'd1;
reg [15:0] last_w, last_h;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bs <= BS_IDLE; brem <= 16'd0;
        d3 <= 4'd0; d2 <= 4'd0; d1 <= 4'd0; d0 <= 4'd0;
        b_is_h <= 1'b0; wdig <= 16'd0; hdig <= 16'd0;
        last_w <= 16'hFFFF; last_h <= 16'hFFFF;
    end else begin
        case (bs)
            BS_IDLE: begin
                if (src_w != last_w) begin
                    last_w <= src_w; brem <= src_w; b_is_h <= 1'b0;
                    d3 <= 4'd0; d2 <= 4'd0; d1 <= 4'd0; d0 <= 4'd0;
                    bs <= BS_TH;
                end else if (src_h != last_h) begin
                    last_h <= src_h; brem <= src_h; b_is_h <= 1'b1;
                    d3 <= 4'd0; d2 <= 4'd0; d1 <= 4'd0; d0 <= 4'd0;
                    bs <= BS_TH;
                end
            end
            BS_TH: if (brem >= 16'd1000) begin brem <= brem - 16'd1000; d3 <= d3 + 4'd1; end
                   else bs <= BS_HU;
            BS_HU: if (brem >= 16'd100)  begin brem <= brem - 16'd100;  d2 <= d2 + 4'd1; end
                   else bs <= BS_TE;
            BS_TE: if (brem >= 16'd10)   begin brem <= brem - 16'd10;   d1 <= d1 + 4'd1; end
                   else bs <= BS_ON;
            BS_ON: begin d0 <= brem[3:0]; bs <= BS_LAT; end
            default: begin
                if (b_is_h) hdig <= {d3, d2, d1, d0};
                else        wdig <= {d3, d2, d1, d0};
                bs <= BS_IDLE;
            end
        endcase
    end
end

// 倒计时两位（0..30）：十位只有 0..3，直接比较比除法便宜
wire [3:0] cnt_tens = (auto_remain >= 6'd30) ? 4'd3 :
                      (auto_remain >= 6'd20) ? 4'd2 :
                      (auto_remain >= 6'd10) ? 4'd1 : 4'd0;
// 个位 = 剩余 − 十位×10（×10 = ×8 + ×2，纯移位加）
wire [6:0] t10      = {cnt_tens, 3'b000} + {2'b00, cnt_tens, 1'b0};
wire [6:0] ones7    = {1'b0, auto_remain} - {1'b0, t10[5:0]};
wire [3:0] cnt_ones = ones7[3:0];
// 三个来源都"左对齐"放进 64 位：标题 2 字 / 标签 ≤4 字 / 选项 1 字，
// 于是取字公式统一成 txt_src[16*(3-txt_c[1:0]) +: 16]
//==============================================================================
//  T1（2026-10-28 v2）：标签/标题表 → 4 个 EG_PHY_BRAM32K
//==============================================================================
//  【v1 为什么失败】用 `(* ram_style="bram" *)` 让工具自己挑，它塞进了 9Kb 池：
//  syn 报 62/64 看不出问题，phy 才报
//      PHY-9013 ERROR: Design's emb number = 66, exceeds the limit 64.
//  64 位宽 × 40 项 = 2560 位，9Kb 块（1024×9）要 4 块，62+4 = 66。
//
//  【v2】器件有 16 个 **32Kb 块**（2048×16）全空着，TD 不会主动用它。这里显式
//  例化 4 个 EG_PHY_BRAM32K（每个存 16 位），走 32Kb 池，9Kb 池一个都不碰。
//  初值写 INIT_00/01/02（每 256 位 = 16 个地址）。
//  ★ 2026-10-28 上板修正：位序是 **最低 16 位 = 最低地址**（LSB-first，
//    与 Xilinx 惯例一致），不是最初假设的 MSB-first。上板现象（标签每 16 项
//    一组倒序、低地址大面积空白）与 MSB-first 的解释完全吻合。
//
//  【时序】地址组合送入 b 口、REGMODE_B="NOREG" ⇒ 1 拍出数据，与原来
//  「地址组合 + 输出寄存」完全一致。行末 txt_l 递增那拍地址立刻指向新行，
//  下一拍 rom_q64 就是新行数据，与写第一列同拍 —— L_TXT 节奏一拍未变。
//
//  【地址】0..34 = label(page*7 + item)，35..39 = title(page)，其余恒 0。
//==============================================================================

// page*7 = (page<<3) - page，省一个乘法器
wire [6:0] lab_base = ({4'd0, page} << 3) - {4'd0, page};
// 边界保护：vidx 是 3 位，scroll_row + txt_l[2:0] - 1 会溢出到 7。
//   页最多 7 项（item 0..6），vidx=7 属越界。原 label_of 对越界组合走 default
//   返回 0（空行）；线性地址会越到下一页甚至 title 区（page4+7=35 正好是 title0）。
//   这里显式把越界地址指到 ROM 空区。
// ★ 2026-10-29 双语：地址从 6 位扩到 7 位，英文表整体 +40
//   （中文 0..39、英文 40..79），BRAM 有 2048 项，只用了 80 项。
//   ★ 空区常量从 63 改成 80：原来 40..63 是没用的零区，现在 40..79 全被英文
//     表占了，只有 80 以后才是空的（+40 后是 120，同样为空）。
wire [6:0] tsrc_a   = (((txt_l_pre == 4'd0) ? (7'd35 + {4'd0, page})
                                            : ((vidx > 3'd6) ? 7'd80
                                                             : (lab_base + {4'd0, vidx}))))
                      + (o_lang ? 7'd40 : 7'd0);

// ==== BEGIN AUTO-GEN: LABEL_ROM (tools/gen_label_rom.py) ====
`ifdef IVERILOG_SIM
// ---- 仿真行为模型（iverilog 没有 TD 原语）----
//   时序与 BRAM 一致：地址组合、clk 上升沿采样、1 拍后数据到 rom_q64。
reg [63:0] rom_q64;
reg [63:0] txt_rom [0:127];
integer ti;
initial begin
    for (ti = 0; ti < 128; ti = ti + 1) txt_rom[ti] = 64'd0;
    txt_rom[ 0] = 64'hC1C1B6C800000000;
    txt_rom[ 1] = 64'hB6D4B1C8B6C80000;
    txt_rom[ 2] = 64'hC9ABCEC200000000;
    txt_rom[ 3] = 64'hB1A5BACDB6C80000;
    txt_rom[ 4] = 64'hB7E7B8F100000000;
    txt_rom[ 5] = 64'hBBD6B8B4C4ACC8CF;
    txt_rom[ 6] = 64'h0000000000000000;
    txt_rom[ 7] = 64'hC9CFD2BBD5C50000;
    txt_rom[ 8] = 64'hCFC2D2BBD5C50000;
    txt_rom[ 9] = 64'hC7D0BBBBD0A7B9FB;
    txt_rom[10] = 64'hD7D4B6AFC7D0BBBB;
    txt_rom[11] = 64'hCACAD3A6C6C1C4BB;
    txt_rom[12] = 64'hD0C5CFA2CCF50000;
    txt_rom[13] = 64'h0000000000000000;
    txt_rom[14] = 64'h0000000000000000;
    txt_rom[15] = 64'h0000000000000000;
    txt_rom[16] = 64'h0000000000000000;
    txt_rom[17] = 64'h0000000000000000;
    txt_rom[18] = 64'h0000000000000000;
    txt_rom[19] = 64'h0000000000000000;
    txt_rom[20] = 64'h0000000000000000;
    txt_rom[21] = 64'hD3EFD1D400000000;
    txt_rom[22] = 64'hD2F4C1BF00000000;
    txt_rom[23] = 64'hBEB2D2F400000000;
    txt_rom[24] = 64'hD1ADBBB7B2A5B7C5;
    txt_rom[25] = 64'hC6B5C6D7CCF50000;
    txt_rom[26] = 64'h0000000000000000;
    txt_rom[27] = 64'h0000000000000000;
    txt_rom[28] = 64'hD0CDBAC500000000;
    txt_rom[29] = 64'hD0BEC6AC00000000;
    txt_rom[30] = 64'hB1E0D2EBC8D5C6DA;
    txt_rom[31] = 64'hD7F7D5DF00000000;
    txt_rom[32] = 64'hD0EDBFC900000000;
    txt_rom[33] = 64'hCEC4B5B500000000;
    txt_rom[34] = 64'hCDCBB3F600000000;
    txt_rom[35] = 64'hCFD4CABE00000000;
    txt_rom[36] = 64'hCFE0B2E100000000;
    txt_rom[37] = 64'hCEC4BCFE00000000;
    txt_rom[38] = 64'hCFB5CDB300000000;
    txt_rom[39] = 64'hB9D8D3DA00000000;
    txt_rom[40] = 64'h4272696768740000;
    txt_rom[41] = 64'h436F6E7472617374;
    txt_rom[42] = 64'h54656D7000000000;
    txt_rom[43] = 64'h5361747572617465;
    txt_rom[44] = 64'h5374796C65000000;
    txt_rom[45] = 64'h44656661756C7400;
    txt_rom[46] = 64'h0000000000000000;
    txt_rom[47] = 64'h5072657600000000;
    txt_rom[48] = 64'h4E65787400000000;
    txt_rom[49] = 64'h4566666563740000;
    txt_rom[50] = 64'h4175746F506C6179;
    txt_rom[51] = 64'h4669745363720000;
    txt_rom[52] = 64'h496E666F00000000;
    txt_rom[53] = 64'h0000000000000000;
    txt_rom[54] = 64'h0000000000000000;
    txt_rom[55] = 64'h0000000000000000;
    txt_rom[56] = 64'h0000000000000000;
    txt_rom[57] = 64'h0000000000000000;
    txt_rom[58] = 64'h0000000000000000;
    txt_rom[59] = 64'h0000000000000000;
    txt_rom[60] = 64'h0000000000000000;
    txt_rom[61] = 64'h4C616E6775616765;
    txt_rom[62] = 64'h566F6C756D650000;
    txt_rom[63] = 64'h4D75746500000000;
    txt_rom[64] = 64'h4C6F6F7000000000;
    txt_rom[65] = 64'h537065637472756D;
    txt_rom[66] = 64'h0000000000000000;
    txt_rom[67] = 64'h0000000000000000;
    txt_rom[68] = 64'h4D6F64656C000000;
    txt_rom[69] = 64'h4368697000000000;
    txt_rom[70] = 64'h4275696C64000000;
    txt_rom[71] = 64'h417574686F720000;
    txt_rom[72] = 64'h4C6963656E736500;
    txt_rom[73] = 64'h446F637300000000;
    txt_rom[74] = 64'h4578697400000000;
    txt_rom[75] = 64'h446973706C617900;
    txt_rom[76] = 64'h416C62756D000000;
    txt_rom[77] = 64'h46696C6500000000;
    txt_rom[78] = 64'h53797374656D0000;
    txt_rom[79] = 64'h41626F7574000000;
end
always @(posedge clk) rom_q64 <= txt_rom[tsrc_a];
`else
// ---- 综合：4 个 32Kb 块并联成 64 位宽 ----
//   ★ 本段由 tools/gen_label_rom.py 自动生成，不要手改。
//   条目 0..39 = 中文标签/标题，40..79 = 英文（同序号 + 40）。
wire [63:0] rom_q64;
EG_PHY_BRAM32K #(
    .MODE("DP16K"), .DATA_WIDTH_A("16"), .DATA_WIDTH_B("16"),
    .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
    .INIT_00(256'h0000000000000000C4BBBBBBB9FB000000000000C8CF00000000000000000000),
    .INIT_01(256'h0000C6DA00000000000000000000B7C500000000000000000000000000000000),
    .INIT_02(256'h0000000074000000746500007374000000000000000000000000000000000000),
    .INIT_03(256'h0000000067650000000000000000000000000000000000000000617900000000),
    .INIT_04(256'h00000000000000007900000000006500000000000000000000000000756D0000)
) u_txt_rom0 (
    .clka(clk), .csa(1'b1), .wea(1'b0), .addra(11'd0), .dia(16'd0),
    .bytea(1'b0), .bytewea(1'b0), .rsta(1'b0), .ocea(1'b1),
    .clkb(clk), .csb(1'b1), .web(1'b0), .addrb({4'd0, tsrc_a}),
    .dib(16'd0), .byteb(1'b0), .byteweb(1'b0), .rstb(1'b0), .oceb(1'b1),
    .dob(rom_q64[15:0])
);
EG_PHY_BRAM32K #(
    .MODE("DP16K"), .DATA_WIDTH_A("16"), .DATA_WIDTH_B("16"),
    .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
    .INIT_00(256'h000000000000CCF5C6C1C7D0D0A7D5C5D5C50000C4AC0000B6C80000B6C80000),
    .INIT_01(256'h0000C8D50000000000000000CCF5B2A500000000000000000000000000000000),
    .INIT_02(256'h00000000756C6500726100007261687400000000000000000000000000000000),
    .INIT_03(256'h00006D6575610000000000000000000000000000000000006372506C63740000),
    .INIT_04(256'h7400656D00006D006C61000000006E736F72640000006C000000000074720000)
) u_txt_rom1 (
    .clka(clk), .csa(1'b1), .wea(1'b0), .addra(11'd0), .dia(16'd0),
    .bytea(1'b0), .bytewea(1'b0), .rsta(1'b0), .ocea(1'b1),
    .clkb(clk), .csb(1'b1), .web(1'b0), .addrb({4'd0, tsrc_a}),
    .dib(16'd0), .byteb(1'b0), .byteweb(1'b0), .rstb(1'b0), .oceb(1'b1),
    .dob(rom_q64[31:16])
);
EG_PHY_BRAM32K #(
    .MODE("DP16K"), .DATA_WIDTH_A("16"), .DATA_WIDTH_B("16"),
    .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
    .INIT_00(256'h000000000000CFA2D3A6B6AFBBBBD2BBD2BB0000B8B4B8F1BACDCEC2B1C8B6C8),
    .INIT_01(256'hD5DFD2EBC6ACBAC500000000C6D7BBB7D2F4C1BFD1D400000000000000000000),
    .INIT_02(256'h657600006661796C74756D706E746967D3DACDB3BCFEB2E1CABEB3F6B5B5BFC9),
    .INIT_03(256'h74656C756E6700000000000000000000000000000000666F7453746F66657874),
    .INIT_04(256'h6F7573746C65627573706974637363657468696C697064650000000065636F70)
) u_txt_rom2 (
    .clka(clk), .csa(1'b1), .wea(1'b0), .addra(11'd0), .dia(16'd0),
    .bytea(1'b0), .bytewea(1'b0), .rsta(1'b0), .ocea(1'b1),
    .clkb(clk), .csb(1'b1), .web(1'b0), .addrb({4'd0, tsrc_a}),
    .dib(16'd0), .byteb(1'b0), .byteweb(1'b0), .rstb(1'b0), .oceb(1'b1),
    .dob(rom_q64[47:32])
);
EG_PHY_BRAM32K #(
    .MODE("DP16K"), .DATA_WIDTH_A("16"), .DATA_WIDTH_B("16"),
    .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
    .INIT_00(256'h000000000000D0C5CACAD7D4C7D0CFC2C9CF0000BBD6B7E7B1A5C9ABB6D4C1C1),
    .INIT_01(256'hD7F7B1E0D0BED0CD00000000C6B5D1ADBEB2D2F4D3EF00000000000000000000),
    .INIT_02(256'h507200004465537453615465436F4272B9D8CFB5CEC4CFE0CFD4CDCBCEC4D0ED),
    .INIT_03(256'h4D75566F4C6100000000000000000000000000000000496E4669417545664E65),
    .INIT_04(256'h416253794669416C44694578446F4C694175427543684D6F0000000053704C6F)
) u_txt_rom3 (
    .clka(clk), .csa(1'b1), .wea(1'b0), .addra(11'd0), .dia(16'd0),
    .bytea(1'b0), .bytewea(1'b0), .rsta(1'b0), .ocea(1'b1),
    .clkb(clk), .csb(1'b1), .web(1'b0), .addrb({4'd0, tsrc_a}),
    .dib(16'd0), .byteb(1'b0), .byteweb(1'b0), .rstb(1'b0), .oceb(1'b1),
    .dob(rom_q64[63:48])
);
`endif
// ==== END AUTO-GEN: LABEL_ROM ====

wire [63:0] txt_src = rom_q64;

//------------------------------------------------------------------------------
//  ★ 2026-10-29 双语取字（中文 4 字 / 英文 8 字，共用同一个 4:1 16 位 mux）
//------------------------------------------------------------------------------
//  【打包】64 位 = 4 个 16 位槽。中文 1 槽 1 个 GB 码；英文 2 个 ASCII 挤 1 槽
//  （高字节是前一个字）。于是"取第 c 个字"在两种语言下都只是
//  「选槽 → 选字节」，而**选槽的 4:1 mux 是原来就有的那份**，没有新增第二套。
//
//  【为什么要拆成 lab_word / lab_ch 两级】
//    原来三处（标题 / 弹层里保标签 / 普通标签）各写了一遍
//    `txt_src[16*(3-txt_c[1:0]) +: 16]`。表达式完全相同，但分散在三个分支里，
//    靠综合器自己 CSE。现在提成一个 wire，三处共用，且便于加语言分支。
//
//  【成本】选槽 2 位 2:1（+2）、字节/语种合并的 16 位 3:1（+16，每输出位 1 个 LUT）。
//    ★ 这里的 3:1 已经是最省的写法：把"选中/英"和"选高/低字节"合并成一个
//      每输出位只有 1 个 LUT 的函数，而不是两个串联的 16 位 2:1。
wire [1:0]  lab_slot = o_lang ? (2'd3 - txt_c[2:1]) : (2'd3 - txt_c[1:0]);
wire [15:0] lab_word = txt_src[{lab_slot, 4'd0} +: 16];
wire [15:0] lab_ch   = o_lang ? {8'd0, (txt_c[0] ? lab_word[7:0] : lab_word[15:8])}
                              : lab_word;
//  可见列数：中文标签 4 列、英文 8 列；中文标题 2 列、英文标题 7 列（Display）
//  ★ 必须写成"高位全 0"的 NOR，不能只判低位两三位：
//    原来 `txt_c < 4` 是 6 位比较，而 txt_c 会一直走到 31（MAXLEN），
//    只判 txt_c[3:2] 的话第 16 列也会命中 ⇒ 标签在 16..23 列又画一遍。
//    txt_c<8 ⟺ !|c[5:3]   txt_c<4 ⟺ !|c[5:2]
//    txt_c<7 ⟺ !|c[5:3] && !(c2&c1&c0)    txt_c<2 ⟺ !|c[5:1]
wire        lab_vis  = o_lang ? ~(|txt_c[5:3]) : ~(|txt_c[5:2]);
wire        ttl_vis  = o_lang ? (~(|txt_c[5:3]) & ~(txt_c[2] & txt_c[1] & txt_c[0]))
                              : ~(|txt_c[5:1]);

always @(*) begin
    txt_ch = 16'h0000;          // 0 = 空白码
    if (txt_l >= 4'd6) begin
        // ★ 第 6、7 行有三种身份：
        //   缩放 OSD  = 倍数（行6）+ 当前功能（行7）
        //   菜单打开  = 弹层备用行
        //   其它      = 信息条两行（文件名 / 分辨率 + 倒计时）
        if (zoom_on) begin
            // 行6 "2.0x"（数字按档位选），行7 "X-PAN"/"Y-PAN"/"ZOOM"
            if (txt_l == 4'd6) begin
                case (txt_c)
                    5'd0: txt_ch = (zoom_val == 8'h10) ? 16'h0031 :
                                   (zoom_val == 8'h20) ? 16'h0032 :
                                   (zoom_val == 8'h40) ? 16'h0034 : 16'h0038;
                    5'd1: txt_ch = 16'h002E;        // '.'
                    5'd2: txt_ch = 16'h0030;        // '0'
                    5'd3: txt_ch = 16'h0078;        // 'x'
                    default: txt_ch = 16'h0000;
                endcase
            end else begin
                case (txt_c)
                    // ZOOM / X-PAN / Y-PAN（都是 5 个字符，共用同一套列位置）
                    5'd0: txt_ch = (zoom_mode == 2'd2) ? 16'h005A :
                                   (zoom_mode == 2'd1) ? 16'h0059 : 16'h0058;
                    5'd1: txt_ch = 16'h002D;        // '-'
                    5'd2: txt_ch = (zoom_mode == 2'd2) ? 16'h004F : 16'h0050;
                    5'd3: txt_ch = (zoom_mode == 2'd2) ? 16'h004F : 16'h0041;
                    5'd4: txt_ch = (zoom_mode == 2'd2) ? 16'h004D : 16'h004E;
                    default: txt_ch = 16'h0000;
                endcase
            end
        end else if (menu_on) begin
            if (pop_open && ((txt_l == 4'd7) ? pop_v7 : pop_v6))
                txt_ch = opt_char(pop_set,
                                  (txt_l == 4'd7) ? (pop_w0 + pop_lb) : (pop_w0 + pop_la),
                                  txt_c, o_lang);
        end else if (txt_l == 4'd6) begin
            // "TEST.BMP"（固定文件名）
            // ★★★ 2026-10-27 根因修复：原来 case 的是 txt_c[2:0] ★★★
            //   ⇒ 第 8..31 列会把同一个 8 字文件名**再循环 2~4 遍**
            //   （用户看到 "TEST.BMPTEST.BMP"）。改成按完整 txt_c 判，
            //   越界就是空白；行宽也在布局收尾处收到 8。
            case (txt_c)
                5'd0: txt_ch = 16'h0054;   // T
                5'd1: txt_ch = 16'h0045;   // E
                5'd2: txt_ch = 16'h0053;   // S
                5'd3: txt_ch = 16'h0054;   // T
                5'd4: txt_ch = 16'h002E;   // .
                5'd5: txt_ch = 16'h0042;   // B
                5'd6: txt_ch = 16'h004D;   // M
                5'd7: txt_ch = 16'h0050;   // P
                default: txt_ch = 16'h0000;
            endcase
        end else begin
            // 第 7 行：左边按实际位数紧凑排 "640x480"，倒计时**右对齐**到面板内沿。
            //   面板几何 gx0=8 / gx1=328（osd_widget 的 cell16），文字原点 x0=16、
            //   每格 16px ⇒ 第 c 格占 x = 16+16c .. 31+16c：
            //       c=18 → 304..319（右边留 8px，与左边 8..15 的留白对称）
            //       c=19 → 320..335（越过 gx1=328）⇒ 最后一格只能到 18。
            //   所以倒计时钉在**第 17/18 列**：个位永远贴右，十位为 0 时留空，
            //   0/5/10/30 不论几位都往右靠齐。
            //
            //   ★ 注意 tc 只有 4 位（txt_c[3:0]），拿 4'd17 去比会被截成 1，
            //     和第 1 列（W 的某一位）撞车 ⇒ 这两支必须用 5 位的 txt_c 判。
            begin : resfmt
                reg [3:0] tc;
                reg [3:0] idx;
                tc = txt_c[3:0];
                if (txt_c == 5'd18) begin
                    // 个位（永远写，哪怕 0 —— 它就是右对齐的锚）
                    txt_ch = ((auto_mode >= 3'd1) && (auto_mode <= 3'd4))
                             ? ({12'd0, cnt_ones} + 16'h0030) : 16'h0000;
                end else if (txt_c == 5'd17) begin
                    // 十位（0 时留空 ⇒ 0..9 只看见一个个位数字贴在右边）
                    txt_ch = ((auto_mode >= 3'd1) && (auto_mode <= 3'd4) && (cnt_tens != 4'd0))
                             ? ({12'd0, cnt_tens} + 16'h0030) : 16'h0000;
                end else if ((txt_c < 5'd12) && (tc < wlen)) begin
                    // W 的某一位（右对齐到 wlen 内）
                    idx = wlen - 4'd1 - tc;
                    case (idx)
                        4'd0: txt_ch = {12'd0, wdig[3:0]}  + 16'h0030;
                        4'd1: txt_ch = {12'd0, wdig[7:4]}  + 16'h0030;
                        4'd2: txt_ch = {12'd0, wdig[11:8]} + 16'h0030;
                        default: txt_ch = {12'd0, wdig[15:12]} + 16'h0030;
                    endcase
                end else if ((txt_c < 5'd12) && (tc == wlen)) begin
                    txt_ch = 16'h0078;                       // x
                end else if ((txt_c < 5'd12) && (tc > wlen) && (tc <= (wlen + hlen))) begin
                    // H 的某一位（从 x 后面开始，左对齐）
                    idx = wlen + hlen - tc;
                    case (idx)
                        4'd0: txt_ch = {12'd0, hdig[3:0]}  + 16'h0030;
                        4'd1: txt_ch = {12'd0, hdig[7:4]}  + 16'h0030;
                        4'd2: txt_ch = {12'd0, hdig[11:8]} + 16'h0030;
                        default: txt_ch = {12'd0, hdig[15:12]} + 16'h0030;
                    endcase
                end
            end
        end
    end else if (txt_l == 4'd0) begin
        // ★ 2026-10-29：标题也双语。中文 2 字、英文最多 7 字（Display），
        //   取字与标签共用 lab_ch（同一份 4:1 mux + 字节选择）。
        if (ttl_vis) txt_ch = lab_ch;
    end else begin
        // ★ 优先级：先判"压在控件行上的那个选项字"，再判"这行是否被弹层盖住"。
        if (pop_open && (txt_c >= 5'd14) && (txt_c <= 5'd29) &&
            (txt_l == ({1'b0, sel_k} + 4'd1)))
            // 压在控件行上的那一项（夹紧后不一定是选中项）：2026-10-26 起
            // 从第 14 列起写完整选项串（16 字，x336..591，▽箭头在 x594 不冲突）
            txt_ch = opt_char(pop_set, pop_sel, txt_c - 5'd14, o_lang);
        // ★★ 被弹层压住的"另一行"：整行不写（它完全在面板底下，本来也看不见）。
        else if (pop_open && (vrow != sel_k) &&
                 ((vrow == pop_wx0) || (vrow == pop_wx1)))
            txt_ch = 16'h0000;
        // ★★ 但 ComboBox **自己那一行**必须保住标签 ★★
        //   原来是"被盖住的行连标签一起不写"，于是控件行的"色彩空间/色温"也没了，
        //   屏幕上就只剩一个悬空的列表 —— 看着像挂在**上一行**（分辨率）的
        //   ComboBox 下面，而且根本分不清在改哪一项。现在：
        //     · 第 0..3 列还剩标签（x 112..176），选项字在第 14 列（x 336），
        //       两者天然不重叠 ⇒ 视觉上就是"标签  当前选项"，和桌面系统一致；
        //     · 其余列（数值/▽箭头）都不写，因为它们的位置被选项行占着。
        else if (pop_open && (vrow == sel_k)) begin
            // ★ 只有"选中项正好压在控件行"时才留标签（读作"色温 中"）。
            //   被夹紧过（pop_a≠pop_sel）时整行清空 —— 否则会读成
            //   "色温 低"而高亮的却是"中"，比没有标签更糟。
            if ((pop_a == pop_sel) && lab_vis)
                txt_ch = lab_ch;
        end
        else if (lab_vis)
            txt_ch = lab_ch;
        // 勾选框开/关：第 15 列（x=352，正好在方框 332..348 右侧）
        //   ★ 2026-10-29：英文写 On / Off。Off 是 3 个字，占第 15..17 列
        //     （x 352/368/384）；只写到第 16 列会被截成 "Of"（出图时抓到过）。
        else if (vtyp == 4'd5 &&
                 (txt_c == 5'd15 || (o_lang && (txt_c == 5'd16 || txt_c == 5'd17)))) begin
            if (!o_lang)
                txt_ch = (vsel != 8'd0) ? 16'hBFAA : 16'hB9D8;          // 开 / 关
            else if (txt_c == 5'd15)
                txt_ch = 16'h004F;                                      // O
            else if (vsel != 8'd0)
                txt_ch = (txt_c == 5'd16) ? 16'h006E : 16'h0000;        // n / 空
            else
                txt_ch = 16'h0066;                                      // f
        end
        // ★ Button（type 7）：圆角矩形里写"执行"（GB2312 执=D6B4 行=D0D0）
        //   第 14/15 列 = x 336/352，离按钮左边 16px，与勾选框的"开/关"（第 15 列）
        //   和 ComboBox 的值（第 14 列）同一套位置，视觉上是一条线。
        //   原来按钮只有 1px 边框 + 半透明底，里面**什么都没有** —— 上板看着是
        //   一个空框，完全看不出是按钮。
        else if (vtyp == 4'd7) begin
            // ★ 2026-10-30：英文写 "OK"（第 14/15 列），中文仍是「执行」。
            //   按钮文字也是界面文字，不译的话英文模式下会孤零零剩两个汉字。
            if (!o_lang) begin
                if      (txt_c == 5'd14) txt_ch = 16'hD6B4;   // 执
                else if (txt_c == 5'd15) txt_ch = 16'hD0D0;   // 行
            end else begin
                if      (txt_c == 5'd14) txt_ch = 16'h004F;   // O
                else if (txt_c == 5'd15) txt_ch = 16'h004B;   // K
            end
        end
        else if ((txt_c >= 5'd14) && (txt_c <= 5'd29) && vtyp == 4'd6)
            // ★★★ 第 14..29 列（x=336..591）：这一行**自己**的值（2026-10-26 起
            //   最多 16 字，原为第 14 列 1 字）★★★
            //   原来的教训仍然有效：这里对所有 ComboBox 行都成立 ⇒ 只能写"这一行
            //   自己"的值，正在编辑的那一行由上面的锚定分支单独处理。
            txt_ch = opt_char(row_set, vsel[2:0], txt_c - 5'd14, o_lang);
        else if (vtyp == 4'd4 || vtyp == 4'd8 || vtyp == 4'd10) begin
            // 数值类：第 9..12 格右对齐 NNN%（在控件左侧，不压轨道）
            if (txt_c == 5'd12)
                txt_ch = 16'h0025;                          // %
            else if (txt_c == 5'd11)
                txt_ch = 16'h0030 + {8'd0, (vsel - q10[6:0] * 7'd10)};
            else if (txt_c == 5'd10)
                txt_ch = (vsel >= 8'd100) ? 16'h0030 :
                         (q10  != 8'd0)   ? (16'h0030 + {12'd0, q10[3:0]}) : 16'h0020;
            else if (txt_c == 5'd9)
                txt_ch = (vsel >= 8'd100) ? 16'h0031 : 16'h0020;
        end
    end
end

//------------------------------------------------------------------------------
//  7. 主状态机
//------------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        lay_st   <= L_IDLE;
        cell_i   <= 5'd0;
        ch_i     <= 3'd0;        // ★ 唯一驱动点（另一个 always 块里也有过一份，TD 报多驱动）
        txt_l    <= 4'd0;
        txt_c    <= 6'd0;
        dirty    <= 1'b1;
        did_write<= 1'b0;
        txt_ver_r<= 8'd1;
        str_we   <= 1'b0;
        ctl_we   <= 1'b0;
        trig     <= 1'b0;
        focus    <= 1'b0;
        nav_idx  <= 3'd0;
        page     <= 3'd0;
        item     <= 3'd0;
        edit     <= 1'b0;
        scroll_row <= 3'd0;
        {val6, val5, val4, val3, val2, val1, val0} <= DEF0;
        b0 <= DEF0; b1 <= DEF1; b3 <= DEF3;
        page_r <= 3'd0;
        pic_tgl <= 1'b0;                 // 画质参数提交标志（postproc 侧边沿检测不依赖初值，但 X 会传染仿真）
        cx_t<=16'd28;  cw_t<=16'd40;  ch_t<=16'd40;
        cy_t<=16'd20;  sc_t<=16'd0;  sv_t<=16'd0;
        txt_xy  <= {(22*NL){1'b0}};
        txt_col <= {(24*NL){1'b0}};
        fade    <= 8'd0;
        menu_d  <= 1'b0;
    end else begin
        str_we    <= 1'b0;
        ctl_we    <= 1'b0;
        trig      <= 1'b0;
        did_write <= did_write & ~(lay_st == L_FIN);
        menu_d    <= menu_on;

        // ---------------- 按键 ----------------
        // ★ 任何一次按键都把 dirty 置 1：标签/数值/选项/页标题一定跟着变，
        //   漏置会让"滚进来的行 / 换页后的行"没有文字。
        if (menu_on && (|key_press)) dirty <= 1'b1;

        if (menu_on) begin
            if (edit) begin
                // 加/减/翻转统一由 nval 算好（它按 sit_type 决定规则），
                // 这里只负责"写回哪一行对应的寄存器"。
                // ★ 原来两段 case(item) 里，3 号行写的是 val3 <= ~val3[0] ——
                //   一元 ~ 的操作数会被上下文位宽撑到 7 位再取反 ⇒ 1 → 126 → 127，
                //   val3 永远非零 ⇒ 勾选框恒为勾、按键像没反应
                //   （上板"checkbox 无法操作"的真正原因）。现在这条路已经不存在了。
                if (k_up_e || k_dn_e) begin
                    case (item)
                        3'd0: val0 <= nval;
                        3'd1: val1 <= nval;
                        3'd2: val2 <= nval;
                        3'd3: val3 <= nval;
                        3'd4: val4 <= nval;
                        3'd5: val5 <= nval;
                        default: val6 <= nval;
                    endcase
                    dirty <= 1'b1;
                    // ★ 画质参数变了才翻 tgl（连发顶到边界时 nval==sel_val，不翻，
                    //   避免每 83ms 空触发一次 postproc 的参数重载）
                    if ((page_r == 3'd0) && (nval != sel_val))
                        pic_tgl <= ~pic_tgl;
                end
                // 编辑中：KEY2（确认）或 KEY1（返回）都退出编辑
                if (k_back || k_ok) edit <= 1'b0;
            end else begin
                if (k_up) begin
                    if (!focus) nav_idx <= (nav_idx == 3'd0) ? 3'd0 : nav_idx - 3'd1;
                    else        item    <= (item == 3'd0) ? 3'd0 : item - 3'd1;
                end
                if (k_down) begin
                    if (!focus) nav_idx <= (nav_idx == 3'd4) ? 3'd4 : nav_idx + 3'd1;
                    else        item    <= (item == item_last) ? item : item + 3'd1;
                end
                // KEY1：页面 → 回导航栏（在导航栏再按就是关 OSD，见上面 exit_req）
                if (k_back && focus) focus <= 1'b0;
                // KEY2：导航栏 → 进页面
                if (k_ok && !focus) begin
                    focus <= 1'b1;
                    item  <= 3'd0;
                    scroll_row <= 3'd0;      // 进入页面从头开始
                end
                // ★ KEY2：CheckBox 直接反选，不进编辑态（用户要求"点一下就行"）
                //   其它可编辑类型（TrackBar / NumericUpDown / ComboBox）才进编辑。
                if (k_ok && focus) begin
                    if (sit_type == 4'd5) begin
                        case (item)                    // nval 对 CheckBox 就是"取反"
                            3'd0: val0 <= nval;
                            3'd1: val1 <= nval;
                            3'd2: val2 <= nval;
                            3'd3: val3 <= nval;
                            3'd4: val4 <= nval;
                            3'd5: val5 <= nval;
                            default: val6 <= nval;
                        endcase
                        dirty <= 1'b1;                 // 开/关文字要重写
                    end else if (sit_type == 4'd4 || sit_type == 4'd10 || sit_type == 4'd6) begin
                        edit <= 1'b1;
                    end else if (sit_type == 4'd7) begin
                        // ★★ 恢复默认（2026-10-26 新增）：原来 type7 按了没有任何动作。
                        //   显示页：7 个值全部装 DEF0（四滑条 50 + 风格无），
                        //   b0 不用写 —— val 就是显示页活跃值，换页存档机制会自动归档。
                        //   page2 的"恢复出厂"仍无动作（不在本期范围）。
                        if (page == 3'd0) begin
                            {val6, val5, val4, val3, val2, val1, val0} <= DEF0;
                            pic_tgl <= ~pic_tgl;       // 参数整体变化 → 通知 postproc
                            dirty <= 1'b1;             // 数值/选项文字全部重写
                        end
                    end
                end
            end

            if (!focus && (nav_idx != page)) begin
                page  <= nav_idx;
                item  <= 3'd0;
                scroll_row <= 3'd0;          // ★ 子菜单切换必须重置滚动
                dirty <= 1'b1;
            end
        end else begin
            edit <= 1'b0;
        end

        // ---------------- 换页：存档旧页 + 读档新页（同一拍） ----------------
        //  ★★★ 2026-10-27：KEY2 快捷轮换并进这里的 `else if` ★★★
        //    【为什么】原来是两段独立的 `if`，而且写的是 `val2[2:0] <= ...` /
        //      `b1[16:14] <= ...` —— **部分位写入**。
        //      TD 打包 FF 的前提是同一组 FF 共享"时钟+复位+使能"（控制集）。
        //      只给 val2 的 3 个 bit、b1 的 3 个 bit 换一份使能，综合器就必须
        //      把这 6 个 bit 从原组里拆出去，各成一套控制集 ⇒ 那几个 FF 变成
        //      "孤儿"，还要连带把同组剩余的 FF 重新分组 ⇒ **打包效率塌方**。
        //      这是本轮 mslice 从"能过"涨到 5091 的主要来源之一。
        //    【改法】改成**整字写**（49 位一起写，只把中间 3 位换掉），
        //      和换页那条路合并成一个 if/else ⇒ 仍然只有"一个"使能信号。
        //      代价是多几个 mux（LUT），换来的是控制集不增加（slice 数量）。
        //      LUT 富余、slice 紧张时，这个交换永远是对的。
        if (page_r != page) begin
            // 只对"有可编辑项"的页存档/读档；文件(2)/关于(4) 页没有值，
            // 进这两页时保持 val* 不变（没人读），省 98 个触发器 + 一条 49 位 5 选 1。
            if      (page_r == 3'd0) b0 <= vpack;
            else if (page_r == 3'd1) b1 <= vpack;
            else if (page_r == 3'd3) b3 <= vpack;
            if      (page == 3'd0) {val6, val5, val4, val3, val2, val1, val0} <= b0;
            else if (page == 3'd1) {val6, val5, val4, val3, val2, val1, val0} <= b1;
            else if (page == 3'd3) {val6, val5, val4, val3, val2, val1, val0} <= b3;
            page_r <= page;
        end

        // 滚动跟随
        if (focus) begin
            if (item >= scroll_row + VIS) begin scroll_row <= item - VIS + 1; dirty <= 1'b1; end
            else if (item < scroll_row)   begin scroll_row <= item;           dirty <= 1'b1; end
        end

        // 动画目标
        cx_t <= tgt_cx; cy_t <= tgt_cy; cw_t <= tgt_cw; ch_t <= tgt_ch;
        sc_t <= {tgt_scroll, 4'd0};
        sv_t <= {5'd0, sel_val, 4'd0};
        fade <= 8'd255;

        // ---------------- 布局 ----------------
        case (lay_st)
            L_IDLE: if (lay_req && (menu_on || info_on || zoom_on) && !ps_start) lay_st <= L_ANIM;

            L_ANIM: begin
                ch_i  <= 3'd0;          // 进度推进已在 L_ANIM 完成，接下来插值
                lay_st <= L_TWN;
            end

            L_TWN: begin                // ★ 6 拍，每拍插值一条通道（共用一套 tween）
                if (ch_i == 3'd5) begin
                    cell_i <= 5'd0;
                    lay_st <= L_CELL;
                end else ch_i <= ch_i + 3'd1;
            end

            L_CELL: begin
                if (cell_i == 5'd19) begin      // NSLOT = 20，最后一条是 19 号（logo）
                    txt_l  <= 4'd0;
                    txt_c  <= 5'd0;
                    lay_st <= L_TXY;
                end else cell_i <= cell_i + 5'd1;
            end

            L_TXY: begin
                // 格式必须是 {Y0[15:0], X0[15:0]}：两边都要显式 16 位
                //
                // ★★★ 这里必须加 scroll_row*PITCH ★★★
                //   第 l 行文字显示的是**第 vidx = scroll_row + l - 1 行**的标签，
                //   而该行控件的 y = ROW0 + vidx*PITCH - sc_px - 4。
                //   原来只写了 ROW0 + (l-1)*PITCH - sc_px，漏掉 scroll_row*PITCH
                //   ⇒ 一旦滚动（scroll_row ≥ 1），标签就和控件错开整整 scroll_row 行，
                //   最下面那一行甚至没有字 —— 上板就是"滚动以后文字乱位/缺字"。
                //   加上之后文字与控件用**同一个** y（差 4px 基线），滚动动画期间
                //   两者一起被 sc_px 拖着走，永远不脱节。
                txt_xy[0*22 +: 22] <= {ty_of(4'd0), tx_of(4'd0)};
                txt_xy[1*22 +: 22] <= {ty_of(4'd1), tx_of(4'd1)};
                txt_xy[2*22 +: 22] <= {ty_of(4'd2), tx_of(4'd2)};
                txt_xy[3*22 +: 22] <= {ty_of(4'd3), tx_of(4'd3)};
                txt_xy[4*22 +: 22] <= {ty_of(4'd4), tx_of(4'd4)};
                txt_xy[5*22 +: 22] <= {ty_of(4'd5), tx_of(4'd5)};
                // ★ 2026-10-30：缩放 OSD 时行 6/7 固定在左上角面板里。
                //   不能只改 tx_of 函数——iverilog 的 `always @(*)` 敏感表不含
                //   函数体内读到的全局信号（zoom_on），改了也不重算（实测无位移）。
                txt_xy[6*22 +: 22] <= zoom_on ? {11'd12, 11'd24} : {ty_of(4'd6), tx_of(4'd6)};
                txt_xy[7*22 +: 22] <= zoom_on ? {11'd32, 11'd24} : {ty_of(4'd7), tx_of(4'd7)};

                // 文字颜色：
                //   · 第 6、7 行（弹层备用行）→ 固定白色，不跟 pop_sel 变
                //     （选项的选中态只由蓝色底表达；否则同一行的标签也会跟着亮）
                //   · 第 1..5 行（对应窗口行 l-1）→ focus 且 sel_k==l-1 时白色
                // ★ 全部展开写死，不用 for + 变量位选（TD 前端不保证能展开）。
                txt_col[0*24 +: 24] <= 24'hF0F0F0;
                txt_col[1*24 +: 24] <= (focus && (sel_k == 3'd0)) ? 24'hFFFFFF : 24'hC0C0C0;
                txt_col[2*24 +: 24] <= (focus && (sel_k == 3'd1)) ? 24'hFFFFFF : 24'hC0C0C0;
                txt_col[3*24 +: 24] <= (focus && (sel_k == 3'd2)) ? 24'hFFFFFF : 24'hC0C0C0;
                txt_col[4*24 +: 24] <= (focus && (sel_k == 3'd3)) ? 24'hFFFFFF : 24'hC0C0C0;
                txt_col[5*24 +: 24] <= (focus && (sel_k == 3'd4)) ? 24'hFFFFFF : 24'hC0C0C0;
                txt_col[6*24 +: 24] <= 24'hFFFFFF;
                txt_col[7*24 +: 24] <= 24'hFFFFFF;
                lay_st <= L_TXT;
            end

            L_TXT: begin
                if (!dirty) begin
                    lay_st <= L_FIN;
                end else if (txt_c <= LASTC) begin
                    str_we   <= 1'b1;
                    str_line <= txt_l;
                    str_idx  <= {2'd0, txt_c};
                    str_code <= txt_ch;
                    txt_c    <= txt_c + 6'd1;
                    did_write<= 1'b1;
                end else begin
                    ctl_we     <= 1'b1;
                    ctl_line   <= txt_l;
                    // ★ 2026-10-27：不再无条件 active=1 / len=32。
                    //   信息条模式下第 0..5 行必须关掉，否则菜单文字会和
                    //   信息条一起显示（用户报的现象）。见上面 `row_vis`。
                    ctl_active <= row_vis;
                    // 菜单模式：整行 32 格（弹层选项串最长 16 字）
                    // 信息条模式：第 6 行 = 文件名 8 字；第 7 行 = 分辨率+倒计时 12 字
                    //   ★ 原来两行都写 32 ⇒ 文件名被 txt_c[2:0] 循环成
                    //     "TEST.BMPTEST.BMP"，分辨率也被拖得很长。
                    ctl_len    <= row_vis ? (menu_on ? 8'd32 :
                                             ((txt_l == 4'd6) ? 8'd8 : 8'd19))
                                          : 8'd0;
                    txt_c      <= 5'd0;
                    if (txt_l == LASTL) begin
                        txt_ver_r <= txt_ver_r + 8'd1;
                        dirty     <= 1'b0;
                        lay_st    <= L_FIN;
                    end else txt_l <= txt_l + 4'd1;
                end
            end

            L_FIN: begin
                trig   <= did_write;
                lay_st <= L_IDLE;
            end

            default: lay_st <= L_IDLE;
        endcase

        // ★★★ 值动画在飞的时候，文字里的数字必须**每帧重写** ★★★
        //   值寄存器按下键就变了、滑条每帧重排也跟着动，但文字只在 dirty 时才写，
        //   而值动画要走十几帧 ⇒ 数字停在按前的进度：上板现象就是
        //   "调大一次数值不变、按确定才变；调大两次只涨一次"。
        //   这里放在 case 之后 ⇒ 压过 L_TXT 里的 dirty<=0。
        if (sv_c != sv_t) dirty <= 1'b1;

        // ★ 菜单开/关：第 6/7 行要在"弹层备用行"和"信息条两行"之间切换
        if (menu_on != menu_d) dirty <= 1'b1;
        // ★ 信息条内容变化（换图 / 倒计时 / 模式 / 开关）
        // ★ 2026-10-30：缩放 OSD 的倍数/模式文字也要靠 dirty 才会在 L_TXT 重写。
        //   只触发 lay_req 不够 —— L_TXT 开头是 `if (!dirty) lay_st <= L_FIN;`，
        //   直接把文字阶段跳过去了（上板现象：左上角文字永远停在第一次的值）。
        if (info_chg || zoom_chg || zoom_rise) dirty <= 1'b1;

        ctl_ver <= txt_ver_r;
    end
end

//------------------------------------------------------------------------------
//  ★ 2026-10-28 文字输出探针（仅仿真，`ifdef IVERILOG_SIM）
//------------------------------------------------------------------------------
//  用途：label 文字从来没有被断言过（tb_osd_keys 全是几何、tb_osd_icons 的
//  E0 组测的是信息条 txt_l>=6），"文字错位"这类问题只能靠肉眼。这里把布局
//  引擎真正写进文字表的 (行, 列, 码) 打出来，用来区分"RTL 就错了"和
//  "BRAM 上板时序不对"——仿真是行为模型，如果仿真输出就对，说明 RTL 无误。
`ifdef IVERILOG_SIM
always @(posedge clk) begin
    if (str_we && (str_idx < 8'd8))
        $display("[TXT] line=%0d col=%0d code=%04X", str_line, str_idx, str_code);
end
`endif

endmodule
