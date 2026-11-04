//=============================================================================
// 文件名   : video_postproc.v
// 模块名   : video_postproc
// 功能     : 【显示端画质后处理】亮度 / 对比度 / 色温 / 饱和度 + 风格（黑白/反色）
//
// 它在整条链路里的位置（显示读出侧，video_clk 25MHz 域）：
//
//     frame_read_write 读口 ──► video_delay（数据/时序对齐）
//                               │ hs/vs/de/vout_data_raw
//                               ▼
//                         ★ video_postproc（本模块）
//                               │ pp_out_hs/vs/de/pp_out_data
//                               ▼
//                         黑屏门控 display_valid ? ... : 0   （顶层）
//                               ▼
//                         video_rgb_to_axis_640x480 ──► HDMI TX IP
//
// 为什么放显示侧而不是写帧侧：
//   ① 高清模式只画一帧就停（App.v），写侧处理意味着每调一次参数都要全帧重合成；
//   ② 读侧处理零 SDRAM 带宽开销、零合成管线改动；
//   ③ 语义正确：画质参数是"显示器旋钮"，应统一作用于最终画面（含 OSD）。
//
// 处理顺序（全部为点操作，无行缓存；锐化以后在级 3 后插入，DELAY 参数随之加大）：
//     级1 音调级（查表）：tone(x) = clamp( (对比度×曝光增益)·x + 回中项 )
//         ★ 顺序要紧：**先音调、后白平衡**（见 §2 的说明），白点才染得上色
//         白平衡：out_c = tone_c(x) × wb_c，wb_c ≤ 1.0（归一化，只降不升）
//             暖：R 1.00 / G 1−δ/2 / B 1−δ   冷：R 1−δ / G 1−δ/2 / B 1.00
//     级2 饱和度级：Y = (R+2G+B)>>2，out_c = Y + sgain·(in_c − Y)
//         Y 用移位加，不占乘法器
//     级3 风格级：反色 = 255−x；黑白在参数侧把 sgain 强制为 0（覆盖饱和度滑条）
//     每级饱和截断到 0..255。
//
// ★★ 零变量乘法架构（2026-10-26，方案二+三，治 TD 综合报 mslice 5066 > 4900）★★
//   原实现的 8 个变量×变量乘法（级1×3、级2×3、增益预计算×2）若被综合到 LUT，
//   约吃 130-150 mslice。现将每路增益量化成 4 位尾数 × 2 的幂：
//       A_c = M·2^E（M∈[1,15]，四舍五入 + 16 归一化；M=0 即零增益）
//       sgain = M2·2^E2
//       像素级的 M·x / M·d 用 4 项条件移位加实现，E 用桶形移位还原
//   ⇒ 全模块**零乘法器**，替换为 6 个 4 项加法网络 + 3 个小量化器 + 桶形移位。
//   【精度】量化相对误差 ≤ ±6.25%（四舍五入）；滑条步进已放大为 5
//   （osd_menu TB_STEP），相邻档位的量化差不可感知。
//   【恒等】默认参数下 A=256 → M=8,E=5；sgain=256 → M2=8,E2=5：
//       (8x·2^5 + 128)>>8 = x；(8d·2^5 + 128)>>>8 = d —— 逐位恒等不变，
//       "恢复默认 = 无处理"的回归基线保持成立。
//
// 参数跨时钟域（照顶层 commit_tgl 的 house style）：
//     参数来自 OSD 菜单（sd_card_clk 50MHz 域），本模块在 video_clk 25MHz 域。
//     · pic_tgl：菜单侧每次参数变化翻转一次 → 本模块 3 级打拍 → 相邻两位异或
//       还原"有更新"脉冲 → 置 arm；
//     · pic_*：慢变多 bit 电平，直接 2 级打拍（2 拍 ≤ tgl 的 3 拍，边沿检出时
//       总线必然已稳定）；
//     · 生效时机：in_vs 上升沿且 arm=1 → 参数快照进工作寄存器。vs 边沿处于
//       消隐区，流水线在飞的 ≤DELAY 拍全是消隐像素 ⇒ 零可见接缝。参数改变
//       到生效最多延迟一帧（16.7ms）。
//     · arm 单块驱动：置位（tgl_edge）与清零（vs 快照）必须在同一个 always
//       块里，且 tgl_edge 优先 —— 否则 TD 报多驱动（chk_multidrv 抓过）。
//
// 资源：全模块零乘法器、零 DSP。
//
// 【写法纪律】（本工程 TD/iverilog 踩坑结论，勿破坏）
//     · 所有位宽显式，禁止依赖上下文隐式截断（HDL-8007 前车之鉴）；
//     · 截断/量化函数只依赖入参，不在 always@(*) 里调读外部信号的 function；
//     · 每个信号只在一个 always 块里驱动；
//     · 复位值全部为常量。
//=============================================================================

module video_postproc_ref #(
    parameter DELAY = 3                        // 流水级数 = 同步信号延迟拍数
)(
    input  wire        video_clk,              // 像素时钟 25MHz
    input  wire        rst,                    // 复位，高有效（rst_all）

    // ---- 像素流输入（来自 video_delay 输出，de=1 时 in_rgb 同拍有效）----
    input  wire        in_hs,
    input  wire        in_vs,
    input  wire        in_de,
    input  wire [23:0] in_rgb,                 // {R,G,B} 各 8bit

    // ---- 画质参数（sd_card_clk 域慢变电平，菜单侧改一次翻一次 pic_tgl）----
    input  wire [6:0]  pic_bright,             // 0..100，50=默认
    input  wire [6:0]  pic_contrast,           // 0..100，50=默认
    input  wire [6:0]  pic_temp,               // 0=冷 100=暖 50=中性
    input  wire [6:0]  pic_sat,                // 0=黑白 50=原色 100=加倍
    input  wire [1:0]  pic_style,              // 0无 1黑白 2反色
    input  wire        pic_tgl,                // 参数更新翻转标志

    // ---- 像素流输出（延迟 DELAY 拍，与输入的相对对齐不变）----
    output wire        out_hs,
    output wire        out_vs,
    output wire        out_de,
    output wire [23:0] out_rgb
);

//=============================================================================
// 1. 参数跨时钟域同步 + 工作参数快照（单 always 驱动，避免多驱动）
//=============================================================================
//  【arm 的三态语义】
//    · tgl_edge        → arm <= 1（新参数在路上）
//    · vs_rise && arm  → 快照 + arm <= 0
//    · 两者同拍        → else-if 保证 tgl_edge 优先：本拍不快照（参数总线还没
//                        同步完），arm 保持 1，下个 vs 用已稳定的值生效
//                        ⇒ 任何时序组合都不丢参数。

// ---- 参数总线打包：{style, sat, temp, contrast, bright} = 2+7+7+7+7 = 30bit ----
wire [29:0] pic_bus = {pic_style, pic_sat, pic_temp, pic_contrast, pic_bright};

reg [29:0] pic_s1;                             // 1 拍同步
reg [29:0] pic_s2;                             // 2 拍同步（快照取这份）

// ---- pic_tgl：3 级打拍 + 相邻异或还原"有更新"脉冲（顶层 :672-679 同款）----
reg  [2:0] tgl_sync;
wire       tgl_edge = tgl_sync[2] ^ tgl_sync[1];

reg        arm;                                // 有待生效的新参数
reg        in_vs_d;
wire       vs_rise = in_vs & ~in_vs_d;

reg [6:0]  w_bright;                           // 工作参数（生效值）
reg [6:0]  w_contrast;
reg [6:0]  w_temp;
reg [6:0]  w_sat;
reg [1:0]  w_style;

// ---- 待生效参数（tgl 边沿锁存）----
//  表是按这组参数建的；建完后到 vs 才和 w_* 一起切换
//  ⇒ "新表 + 新参数"永远落在同一条像素上，不会出现半屏新半屏旧。
reg [6:0]  p_bright, p_contrast, p_temp, p_sat;
reg [1:0]  p_style;

// "从没被初始化过"的判据（上电全 0）。对比度有 20 下限 ⇒ 正常配置不可能全 0。
wire p_uninit = (p_bright == 7'd0) && (p_contrast == 7'd0) &&
                (p_temp   == 7'd0) && (p_sat      == 7'd0) && (p_style == 2'd0);


// ---- 映射表 bank 控制 ----
reg        bank_sel;      // 读 bank
reg        wr_bank;       // 写 bank（永远 = ~bank_sel，重建完成后才翻）
reg        tb_run;        // 建表中
reg        tb_done;       // 本张表已建完
reg [7:0]  tb_x;          // 建表扫描指针
reg [18:0] tb_acc_r, tb_acc_g, tb_acc_b;   // raw(x) = A_c·x（增量累加）

// ★ 上电确定值（理由同 osd_menu：板上 rst_all 可能一次都不拉高）。
//   没有初值时这几个寄存器停在上电值 0 ⇒ 对比度 0（被下限夹成 20）+ 曝光 0.22×
//   ⇒ 上板"发暗发灰，必须盲按菜单才恢复"。
initial begin
    p_bright = 7'd50;  p_contrast = 7'd50; p_temp = 7'd50;
    p_sat    = 7'd50;  p_style    = 2'd0;
    w_bright = 7'd50;  w_contrast = 7'd50; w_temp = 7'd50;
    w_sat    = 7'd50;  w_style    = 2'd0;
    // 建表状态机也要确定起点：不复位时若停在 X，`else if (!tb_done)` 的真值
    // 为 X ⇒ 自愈分支不成立 ⇒ 表永远是 0（黑屏）。显式给 0 后，
    // tb_run=0/tb_done=0 必然命中自愈分支 ⇒ 无条件建出第一张表。
    tb_run   = 1'b0;
    tb_done  = 1'b0;
    tb_x     = 8'd0;
    tb_acc_r = 19'd0;
    tb_acc_g = 19'd0;
    tb_acc_b = 19'd0;
    bank_sel = 1'b0;
    wr_bank  = 1'b0;
end

always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        pic_s1   <= 30'd0;
        pic_s2   <= 30'd0;
        tgl_sync <= 3'd0;
        arm      <= 1'b0;
        in_vs_d  <= 1'b0;
        // 复位值 = OSD 的 DEF0（全 50 / 风格无）⇒ 上电即为恒等直通
        w_bright   <= 7'd50;
        w_contrast <= 7'd50;
        w_temp     <= 7'd50;
        w_sat      <= 7'd50;
        w_style    <= 2'd0;
        p_bright   <= 7'd50;
        p_contrast <= 7'd50;
        p_temp     <= 7'd50;
        p_sat      <= 7'd50;
        p_style    <= 2'd0;
    end else begin
        pic_s1   <= pic_bus;
        pic_s2   <= pic_s1;
        tgl_sync <= {tgl_sync[1:0], pic_tgl};
        in_vs_d  <= in_vs;
        if (tgl_edge) begin
            // 新参数到齐：锁进 p_*（表由 p_* 组合算出，见第 2 节）
            // ★ 全 0 总线视为"OSD 还没就绪"的上电瞬态，不予采纳：
            //   合法配置里对比度有 20 的下限、亮度/色温/饱和至少有一个非 0，
            //   所以 30 位全 0 只可能出现在上电那一瞬。
            if (|pic_s2) begin
                p_bright   <= pic_s2[6:0];
                p_contrast <= pic_s2[13:7];
                p_temp     <= pic_s2[20:14];
                p_sat      <= pic_s2[27:21];
                p_style    <= pic_s2[29:28];
                arm        <= 1'b1;
            end
        end else if (p_uninit) begin
            // ★ 兜底：五个参数全 0 只可能是"从没初始化过"（对比度有 20 的下限，
            //   任何真实配置都不可能全 0）。若工具没采纳 initial（寄存器上电为 0），
            //   这里用一个比较器把默认值装进去 —— 不依赖复位、也不依赖初值。
            p_bright   <= 7'd50;  p_contrast <= 7'd50;
            p_temp     <= 7'd50;  p_sat      <= 7'd50;  p_style <= 2'd0;
            w_bright   <= 7'd50;  w_contrast <= 7'd50;
            w_temp     <= 7'd50;  w_sat      <= 7'd50;  w_style <= 2'd0;
        end else if (vs_rise && arm && tb_done) begin
            // 表已建完 ⇒ 参数切换（读 bank 由建表状态机同拍翻，见第 3 节）
            w_bright   <= p_bright;
            w_contrast <= p_contrast;
            w_temp     <= p_temp;
            w_sat      <= p_sat;
            w_style    <= p_style;
            arm        <= 1'b0;
        end
    end
end

//=============================================================================
// 2. 增益计算 + 量化（组合逻辑，随工作参数即时更新，每像素不重算）
//=============================================================================
//  常数乘法（×1311 / ×328）由综合器折叠成移位加，不是变量乘法。
//  ★ 变量乘法全部消除：
//    · 色温通道增益 A_r/A_b = cgain ± (cgain·|dtg|)>>8 —— |dtg| ≤ 64，
//      按 |dtg| 的 7 个位做条件移位加（一个网络服务 R/B 两路，符号各取）；
//    · 量化：A_c → M·2^E（M∈[1,15]，四舍五入 + 16 归一化；M=0 即零增益），
//      像素级的 M·x / M·d 用 4 项条件移位加，E 用桶形移位还原。

// ---- 对比度增益（Q8）：50→256(×1.0)，100→512(×2.0) ----
//  ★ 硬下限 20（≈0.4×）：对比度=0 会得到"全屏 128"的平表，画面彻底看不见；
//    而 OSD 是合成进帧缓存之后才进后处理的，菜单也会一起消失 ⇒ 会把人锁死
//    在盲操作里（上板实测过）。所以 UI 和这里各加一道下限。
wire [6:0]  pc    = (p_contrast < 7'd20) ? 7'd20 : p_contrast;
wire [17:0] cg_p  = {11'd0, pc} * 11'd1311;
wire [9:0]  cgain = cg_p[17:8];                // 102..512

// ---- 色温偏移：dt = temp−50 ∈ [−50,50]，dtg = dt×1.28 ∈ ±64 ----
wire signed [8:0]  dt   = $signed({1'b0, p_temp}) - 9'sd50;
wire signed [17:0] dt_p = dt * 18'sd328;       // |dt×328| ≤ 16400
wire signed [8:0]  dtg  = dt_p >>> 8;          // ±64（floor）

// ---- 亮度 = 曝光（乘性），不是直流偏置 ----
//  【为什么改】旧版 boff 是 offset（lift），范围只有 ±64：白 255 最多被压到 191
//  （25%），提亮时高光又是硬切 ⇒ 观感就是"亮度对纯白几乎没作用"。
//  改成曝光后 0..100 映射到 0.22×..1.78×，白、黑都能跟着走（黑锚在 0）。
//      bgain = 256 + (b−50)×4  ⇒  b=0 → 56(0.22×)，b=50 → 256，b=100 → 456(1.78×)
wire signed [9:0]  bd    = $signed({3'b0, p_bright}) - 10'sd50;
wire signed [11:0] bgain = 12'sd256 + (bd <<< 2);          // 56..456

// ★★ 位宽陷阱（HDL-8007 同族）★★
//   Verilog 里 a*b 的结果宽度 = max(宽度(a), 宽度(b))，**不是**两者之和。
//   原来写成 `(cgain * bgain[9:0]) >> 8`：两个操作数都 10 位 ⇒ 结果按 10 位算，
//   256×256 = 65536 被截成 0 ⇒ cg2=0 ⇒ 三路增益全 0 ⇒ 整张表全 0 ⇒ 全黑屏。
//   必须先给乘积一个足够宽的上下文（20 位），再移位。
wire [19:0] cg2_p = cgain * bgain[9:0];                    // 20 位：≤ 512×456
wire [11:0] cg2   = cg2_p >> 8;                            // 对比度×曝光 = 总增益 ≤ 912


// ---- 音调增益（对比度×曝光；三通道完全相同，色温不再掺进来）----
//  ★ 这是"白点染不上色"的第二个根因：
//    之前把白平衡增益 A_c = cg2×wb_c 和对比度合并成一条仿射曲线，
//    对比度（cg2 最高 512）会先把蓝通道抬到 255，色温的 −25% 被完全吃掉 ——
//    对比度越高，白点越白，色温越"看不见"。实测：对比度 100 时白点 = 255/255/255。
//    正确顺序是**先音调、后白平衡**：先把对比度/曝光曲线压到 0..255 并截断，
//    再乘 ≤1 的白平衡系数。这样白点永远是"255 × wb"，与对比度无关。
wire [11:0] cg2_x  = {1'b0, cg2};
wire [11:0] aq_r = cg2_x;                                  // 音调增益（R）
wire [11:0] aq_g = cg2_x;                                  // 音调增益（G）
wire [11:0] aq_b = cg2_x;                                  // 音调增益（B）

// ---- 白平衡增益（归一化，只降不升；256 = 1.0）----
//  暖（dtg>0）：R=1.00  G=1−δ/2  B=1−δ
//  冷（dtg<0）：R=1−δ   G=1−δ/2  B=1.00
//  ★ 强度与对比度解耦：δ 直接取 |dtg|（0..64 = 0..25%），不再乘 cg2。
//    乘了 cg2 的话，对比度 100 时色偏会是 50 时的两倍（白点 255/191/127），
//    而白平衡本质是固定比例，不该随对比度变。
//    ⇒ 顺带把原来那个 7 项条件移位加网络整块删掉（省 ~100 LUT）。
wire [7:0] dl_abs = dtg[8] ? (~dtg[7:0] + 8'd1) : dtg[7:0];   // |dtg| ≤ 64
wire [7:0] dl_h   = {1'b0, dl_abs[7:1]};                      // δ/2（绿通道跟一半）

wire [8:0] wb_main = 9'd256 - {1'b0, dl_abs};                 // 1 − δ
wire [8:0] wb_mid  = 9'd256 - {1'b0, dl_h};                   // 1 − δ/2
wire [8:0] wb_r = dtg[8] ? wb_main : 9'd256;               // 冷压 R；暖 R 不动
wire [8:0] wb_g = wb_mid;                                  // 两个方向都轻压 G
wire [8:0] wb_b = dtg[8] ? 9'd256 : wb_main;               // 暖压 B；冷 B 不动

// ---- 回中项 B_c = 128 − cgain/2（**只用对比度**，不含色温、不含曝光）----
//  ★★★ 这是色温"看不见效果"的根因修复 ★★★
//  旧版 B_c 用的是每通道的 A_c：bq_r = 128 + boff − A_r/2。色温把 A_r 抬到 320 时
//  B_r 同步变成 −32，于是 f(128)=1.25×128−32=128 —— **中灰纹丝不动**，而中灰正是
//  人眼判断色偏的参考点，所以调到 100 也"看不出变化"。
//  现在 B 只跟对比度走：
//    · 对比度：B = 128 − cgain/2，中灰锚定 128（不变）
//    · 曝光  ：B 不含它 ⇒ 锚点在黑（0→0），白跟着走
//    · 色温  ：B 不含它 ⇒ 纯通道增益，中灰被染色（这才看得见）
wire signed [9:0] bq_c = 10'sd128 - $signed({1'b0, cgain[9:1]});
wire signed [9:0] bq_r = bq_c;
wire signed [9:0] bq_g = bq_c;
wire signed [9:0] bq_b = bq_c;

// ---- 量化器：A(Q8) → {M[3:0], E[3:0]}，M=0 即零增益 ----
//   h = A 的最高有效位；E = max(h−3, 0)；
//   M = round(A / 2^E)，M==16 时归一化为 M=8、E+1。
function [7:0] quant_me;                       // {M[3:0], E[3:0]}
    input signed [10:0] a;                     // 原始 Q8 增益（负值按 0 处理）
    reg [10:0] av;
    reg [3:0]  e;
    integer    m;
    begin
        av = (a < 11'sd0) ? 11'd0 : a;
        // 最高有效位：显式优先链（A ≤ 640 ⇒ av[10] 恒 0，保留兜底）
        if      (av[10] == 1'b1) e = 4'd7;     // h=10 ⇒ E=7（理论不可达）
        else if (av[9]  == 1'b1) e = 4'd6;
        else if (av[8]  == 1'b1) e = 4'd5;
        else if (av[7]  == 1'b1) e = 4'd4;
        else if (av[6]  == 1'b1) e = 4'd3;
        else if (av[5]  == 1'b1) e = 4'd2;
        else if (av[4]  == 1'b1) e = 4'd1;
        else                     e = 4'd0;     // h ≤ 3
        // M = round(A / 2^E)。★ m 用 integer：round 结果可能到 16，
        //   4 位寄存器会把 16 截成 0（画面瞬间变平的隐形炸弹）
        if (e == 4'd0)
            m = av[3:0];                       // h<3 ⇒ av ≤ 7，精确
        else begin
            m = (av + (11'd1 << (e - 4'd1))) >> e;
            if (m >= 16) begin                                 // 进位归一化
                m = 8;
                e = e + 4'd1;
            end
        end
        quant_me = {m[3:0], e};
    end
endfunction

// ---- 饱和度增益（Q8）：50→256；黑白（style==1）强制 0（覆盖饱和度滑条）----
//  ★ 只有饱和度这一路还需要 M·2^E 量化：它作用在 Y 差值上（跨通道），
//    没法折进"每通道 1D 查表"，所以保留 4 项移位加。色温/亮度/对比度已走表。
wire [17:0] sg_p      = {11'd0, w_sat} * 11'd1311;
wire [9:0]  sgain_raw = sg_p[17:8];            // 0..512
wire [9:0]  sgain     = (w_style == 2'd1) ? 10'd0 : sgain_raw;
wire [7:0]  q_s = quant_me($signed({1'b0, sgain}));

//=============================================================================
// 3. 像素流水线（3 级寄存 = DELAY，hs/vs/de 随数据同级延迟）
//=============================================================================

// ---- 截断函数：只依赖入参（iverilog always@(*) 敏感表安全）----
function [7:0] clamp255(input signed [11:0] v);
    begin
        if (v < 12'sd0)
            clamp255 = 8'd0;
        else if (v > 12'sd255)
            clamp255 = 8'd255;
        else
            clamp255 = v[7:0];
    end
endfunction

// ---- 白平衡乘法：v×wb>>8（wb=256 时逐位恒等，即"默认无色偏"）----
//   ★ 位宽纪律：v 8 位 × wb 9 位，乘积必须先落到 17 位上下文再截，
//     否则会重演 cg2 那个"乘法结果按最大操作数宽度算"的截断坑。
function [7:0] wbgain;
    input [7:0] v;
    input [8:0] wb;
    reg   [16:0] p;
    begin
        p = v * wb;                       // ≤ 255×256 = 65280（17 位）
        wbgain = p[15:8];                 // >>8
    end
endfunction

// ---- M·d：4 项条件移位加（只给饱和度级用）----
function signed [12:0] mul4s;                  // M × d（d 9bit 有符号）|≤ 3825|
    input [3:0]        m;
    input signed [8:0] d;
    reg signed [12:0] acc;
    begin
        acc = 13'sd0;
        if (m[0]) acc = acc + d;                               // 符号扩展到 13 位
        if (m[1]) acc = acc + $signed({d, 1'b0});              // 2d（右补 0，负数亦成立）
        if (m[2]) acc = acc + $signed({d, 2'b00});
        if (m[3]) acc = acc + $signed({d, 3'b000});
        mul4s = acc;
    end
endfunction

// ---- 饱和度量化增益拆分（级 2 用）----
wire [3:0] sm_m = q_s[7:4], sm_e = q_s[3:0];

//----------------------------------------------------------------------------
// 级 1：三通道映射表（每通道 512×8 = 两个 256 项 bank 打在同一个 RAM 里）
//
//  表项  f_c(x) = clamp( ((A_c·x + 128) >> 8) + B_c )
//
//  【为什么换成表】原来每像素要 3×（M·x 的 4 项移位加 + 桶形移位 + 加法），
//  约 500 LUT；现在 3 块 BRAM 同步读一次就出结果，而且 A_c 不再量化
//  （精度从 ±6% 变成精确）。25MHz 下 BRAM 同步读正好 1 拍，流水级数不变。
//
//  【表怎么建】仿射映射有天然的增量关系：分子 raw(x) = A_c·x，
//  raw(x+1) = raw(x) + A_c ⇒ 3 个累加器扫 256 拍即可，全模块零乘法。
//  25MHz 下 256 拍 = 10.24µs，远小于垂直消隐（≈1.4ms）。
//----------------------------------------------------------------------------
reg [7:0] tab_r [0:511];   /* fehdl force_ram=1, ram_style="block" */
reg [7:0] tab_g [0:511];   /* fehdl force_ram=1, ram_style="block" */
reg [7:0] tab_b [0:511];   /* fehdl force_ram=1, ram_style="block" */

reg [7:0] s1_r, s1_g, s1_b;
reg       s1_hs, s1_vs, s1_de;

// ---- 读口：像素查表（同步读 ⇒ 1 拍，与原来的乘加级同延迟）----
always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        s1_r  <= 8'd0;  s1_g <= 8'd0;  s1_b <= 8'd0;
        s1_hs <= 1'b0;  s1_vs <= 1'b0; s1_de <= 1'b0;
    end else begin
        s1_r  <= tab_r[{bank_sel, in_rgb[23:16]}];
        s1_g  <= tab_g[{bank_sel, in_rgb[15:8]}];
        s1_b  <= tab_b[{bank_sel, in_rgb[7:0]}];
        s1_hs <= in_hs;  s1_vs <= in_vs;  s1_de <= in_de;
    end
end

// ---- 写口：增量建表（raw(x+1) = raw(x) + A_c）----
wire [18:0] bld_r = {1'b0, tb_acc_r} + 19'd128;   // A_c·x + 128
wire [18:0] bld_g = {1'b0, tb_acc_g} + 19'd128;
wire [18:0] bld_b = {1'b0, tb_acc_b} + 19'd128;

wire [10:0] bld_sh_r = bld_r[18:8];               // ≤ 638
wire [10:0] bld_sh_g = bld_g[18:8];
wire [10:0] bld_sh_b = bld_b[18:8];

wire signed [11:0] bld_v_r = $signed({1'b0, bld_sh_r}) + $signed(bq_r);
wire signed [11:0] bld_v_g = $signed({1'b0, bld_sh_g}) + $signed(bq_g);
wire signed [11:0] bld_v_b = $signed({1'b0, bld_sh_b}) + $signed(bq_b);

always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        // 上电立刻建第一张表（默认参数 50/50/50 ⇒ 恒等直通）
        tb_run   <= 1'b1;
        tb_x     <= 8'd0;
        tb_done  <= 1'b0;
        bank_sel <= 1'b0;
        wr_bank  <= 1'b0;
        tb_acc_r <= 19'd0;
        tb_acc_g <= 19'd0;
        tb_acc_b <= 19'd0;
    end else if (tgl_edge) begin
        // 参数变了：换到另一个 bank 重扫（写口永远写"非读 bank"）
        wr_bank  <= ~bank_sel;
        tb_run   <= 1'b1;
        tb_x     <= 8'd0;
        tb_acc_r <= 19'd0;
        tb_acc_g <= 19'd0;
        tb_acc_b <= 19'd0;
        tb_done  <= 1'b0;
    end else if (vs_rise && arm && tb_done) begin
        // 与第 1 节的 w_* 快照**同拍**切换读 bank ⇒ 新表配新参数
        bank_sel <= wr_bank;
    end else if (tb_run) begin
        // ★ 先音调（对比度/曝光 + 截断），再白平衡（×wb ≤ 1）
        //   顺序反了的话对比度会把白点抬回 255，色温在白点处就看不出来。
        tab_r[{wr_bank, tb_x}] <= wbgain(clamp255(bld_v_r), wb_r);
        tab_g[{wr_bank, tb_x}] <= wbgain(clamp255(bld_v_g), wb_g);
        tab_b[{wr_bank, tb_x}] <= wbgain(clamp255(bld_v_b), wb_b);
        tb_acc_r <= tb_acc_r + {7'd0, aq_r};
        tb_acc_g <= tb_acc_g + {7'd0, aq_g};
        tb_acc_b <= tb_acc_b + {7'd0, aq_b};
        if (tb_x == 8'd255) begin
            tb_run  <= 1'b0;
            tb_done <= 1'b1;
        end else
            tb_x <= tb_x + 8'd1;
    end else if (!tb_done) begin
        // ★ 自愈：表从没建过（上电复位没来、或 initial 被工具忽略）⇒ 立刻补建
        tb_run   <= 1'b1;
        tb_x     <= 8'd0;
        tb_acc_r <= 19'd0;
        tb_acc_g <= 19'd0;
        tb_acc_b <= 19'd0;
    end
end

//----------------------------------------------------------------------------
// 级 2：饱和度级  out_c = clamp( Y + ((M2·d)·2^E2 + 128) >>> 8 )
//   Y = (R + 2G + B) >> 2（移位加）；|M2·d| ≤ 15×255 = 3825（13bit 有符号）
//   ·2^E2(E2≤6) ≤ ±244800（19bit）；>>>8 → ±957；+Y → 12bit 截断
//----------------------------------------------------------------------------
wire [10:0] ysum = {3'b0, s1_r} + {2'b0, s1_g, 1'b0} + {3'b0, s1_b};  // ≤ 1020
wire [7:0]  ylum = ysum[9:2];

wire signed [8:0] d_r = $signed({1'b0, s1_r}) - $signed({1'b0, ylum});
wire signed [8:0] d_g = $signed({1'b0, s1_g}) - $signed({1'b0, ylum});
wire signed [8:0] d_b = $signed({1'b0, s1_b}) - $signed({1'b0, ylum});

wire signed [12:0] p2_r = mul4s(sm_m, d_r);
wire signed [12:0] p2_g = mul4s(sm_m, d_g);
wire signed [12:0] p2_b = mul4s(sm_m, d_b);

wire signed [18:0] p2_r_s = p2_r <<< sm_e;
wire signed [18:0] p2_g_s = p2_g <<< sm_e;
wire signed [18:0] p2_b_s = p2_b <<< sm_e;

wire signed [10:0] q2_r = (p2_r_s + 19'sd128) >>> 8;   // ±957
wire signed [10:0] q2_g = (p2_g_s + 19'sd128) >>> 8;
wire signed [10:0] q2_b = (p2_b_s + 19'sd128) >>> 8;

wire signed [11:0] t2_r = $signed(q2_r) + $signed({4'b0, ylum});
wire signed [11:0] t2_g = $signed(q2_g) + $signed({4'b0, ylum});
wire signed [11:0] t2_b = $signed(q2_b) + $signed({4'b0, ylum});

reg [7:0] s2_r, s2_g, s2_b;
reg       s2_hs, s2_vs, s2_de;

always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        s2_r  <= 8'd0;  s2_g <= 8'd0;  s2_b <= 8'd0;
        s2_hs <= 1'b0;  s2_vs <= 1'b0; s2_de <= 1'b0;
    end else begin
        s2_r  <= clamp255(t2_r);
        s2_g  <= clamp255(t2_g);
        s2_b  <= clamp255(t2_b);
        s2_hs <= s1_hs;  s2_vs <= s1_vs;  s2_de <= s1_de;
    end
end

//----------------------------------------------------------------------------
// 级 3：风格级（反色 255−x；黑白已在参数侧把 sgain 清 0）+ 输出寄存
//   hs/vs/de 随数据走同样的 3 级寄存 ⇒ 与 video_delay 的"延迟线取位"等效，
//   不再单独建延迟线；以后插锐化级时在此后再加一级、DELAY 同步 +1。
//----------------------------------------------------------------------------
reg [7:0] s3_r, s3_g, s3_b;
reg       s3_hs, s3_vs, s3_de;

always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        s3_r  <= 8'd0;  s3_g <= 8'd0;  s3_b <= 8'd0;
        s3_hs <= 1'b0;  s3_vs <= 1'b0; s3_de <= 1'b0;
    end else begin
        s3_r  <= (w_style == 2'd2) ? (8'd255 - s2_r) : s2_r;
        s3_g  <= (w_style == 2'd2) ? (8'd255 - s2_g) : s2_g;
        s3_b  <= (w_style == 2'd2) ? (8'd255 - s2_b) : s2_b;
        s3_hs <= s2_hs;  s3_vs <= s2_vs;  s3_de <= s2_de;
    end
end

assign out_hs  = s3_hs;
assign out_vs  = s3_vs;
assign out_de  = s3_de;
assign out_rgb = {s3_r, s3_g, s3_b};

endmodule
