//==============================================================================
//  sdr_as_ram_sim.v —— SDRAM 控制器（sdr_as_ram）的【行为级仿真替身】
//==============================================================================
//
//  ★★★ 这个文件【只用于仿真】，绝对不要加进 .al 工程 ★★★
//
//  【为什么需要它】
//  工程里真正用的 SDRAM 控制器是安路的加密 IP：
//      include/sdr_as_ram.enc.v            —— `pragma protect 加密，源码不可见
//      TD 自带的 eg_phy_sdram_2m_32.v       —— 同样加密
//  实测：iverilog 编译 sdr_as_ram.enc.v 会直接报
//      "sdr_as_ram.enc.v:76: syntax error"
//  而 TD 本身不带仿真器（它的仿真流程外包给 ModelSim，官方参考设计的
//  sim_do/sim.bat 里写死了 D:\modeltech64_10.5\win64\modelsim.exe）。
//  本机没装 ModelSim，所以**真实的 SDRAM 控制器无法在本机仿真**。
//
//  于是这里写一个行为级替身，让"控制器以上"的所有逻辑都能仿真：
//      · frame_fifo_read / frame_fifo_write 的读搬运、写搬运、读写仲裁
//      · 新增的纹理读仲裁（计划阶段 4）
//      · 全链路的帧时序（计划阶段 7）
//
//  【它模拟不了什么（重要）】
//      · 真实的读延迟 / 吞吐 —— 这里只是一个固定延迟的移位链
//      · 地址 → (row, bank, col) 的映射 —— 这里是一块平坦内存，没有 bank/row 概念
//      · 行切换开销、刷新停顿
//  这三样只能上板用探针实测。好在**本设计不依赖 bank 映射**
//  （顺序布局，不做 bank 规划），所以缺这三样不影响功能验证。
//
//  【对外契约（照抄真实 IP + 沿用工程既有约定）】
//      · 写：App_wr_en + App_wr_addr + App_wr_din + App_wr_dm（dm 位=1 屏蔽该字节）
//      · 读：App_rd_en + App_rd_addr
//            → 恰好 RD_LATENCY 拍后 Sdr_rd_en 拉高一拍，同一拍 Sdr_rd_dout 有效
//      · 上电 INIT_CYCLES 拍后 Sdr_init_done 拉高
//      · 读返回【严格保序】（与官方参考设计 app_wrrd.v:134-154 的用法一致）
//
//  【★ 延迟约定（与 SD/tb_frame_rw.v 的 SDR_LAT 完全一致）★】
//      Sdr_rd_en / Sdr_rd_dout 是【组合输出】，抽自移位链的最后一抽头。
//      定义：在 posedge E 采样到 App_rd_en=1，则在 posedge E+RD_LATENCY
//            采样到 Sdr_rd_en=1。即延迟恰好 RD_LATENCY 拍。
//      （RD_LATENCY 的默认值 10 来自工程实测：frame_fifo_read.v:130 的
//        `rd_delay==10` 就是真硬件上跑通的那个数。）
//
//  【读写冲突】
//  真实 SDRAM 是单口，同一拍不能既读又写。上层由 frame_fifo_read 的
//  App_rd_busy 与 frame_fifo_write 的 App_wr_busy 互斥。本模型用 rw_conflict
//  端口把"同一拍既读又写"报出来，供 tb 断言。
//
//  【已验证】配套 SD/tb_sdram_probe.v，2026-10-16 实测通过（886 次比对，0 错误）
//      · 顺序写 256 字 → 顺序读回：数据全对
//      · 跨行散列地址（256 对齐 / 间隔 1024 / 间隔 4096）写读：数据全对
//      · 随机地址写读 512 次：数据全对
//      · 四个真实区域基址（BUF0/BUF1/TEX0/TEX1）+ 区域末尾：互不混叠
//      · 读延迟实测 = 10 拍（= RD_LATENCY，与 frame_fifo_read.v:130 的
//        `rd_delay==10` 完全一致）
//      · 读返回保序：背靠背连读 256 个字，顺序逐字正确
//      · 背靠背连读吞吐 = 稳态 1 命令/拍（125MHz 下 ≈ 125M 次读/秒，无间隙）
//      · 同拍既读又写 → rw_conflict 正确拉高
//
//  【★ 这些数字里哪些是"真的"、哪些只是模型的假设 ★】
//      · 读延迟 10 拍 —— 模型里的假设值，但**与工程真硬件实测吻合**
//        （frame_fifo_read.v:130 就是用 10 拍跑通的）
//      · 保序 —— 与官方参考设计 app_wrrd.v:134-154 的用法一致，可信
//      · 吞吐 1 命令/拍 —— **只是模型的假设**。真实控制器在行切换/刷新时
//        会慢下来，这个数只能上板实测。它是"仲裁批次大小"的依据，务必上板确认。
//      · 地址 → (row,bank,col) 映射 —— **模型测不出来**（平坦内存）。
//        本设计不依赖它（顺序布局，不做 bank 规划），所以不影响。
//
//==============================================================================

module sdr_as_ram_sim #(
    // 读命令 → 数据 的固定延迟（拍）。默认 10，来自工程实测。
    parameter integer RD_LATENCY  = 10,
    // 上电到 Sdr_init_done 的拍数（真实 IP 内部要跑预充电/8 次刷新/模式寄存器）
    parameter integer INIT_CYCLES = 200,
    // 内存容量（字）。2^21 = 2M 字 × 32bit = 8MB，与 IS42s32200 一致
    parameter integer MEM_WORDS   = 2097152,
    //--------------------------------------------------------------------------
    //  ★★ 严格单口模式（默认 0 = 关，老行为）★★
    //--------------------------------------------------------------------------
    //  0 = 温和：同一拍既读又写时，两个命令都照做（只在 rw_conflict 上报）
    //  1 = 严格：同一拍既读又写时，**读命令被丢弃**（写优先）
    //
    //  【为什么要有这个开关】
    //  真 SDRAM 是**单口**器件，同一拍不可能既读又写，这是上层必须遵守的契约
    //  （原设计就是靠 frame_fifo_read 的 `~App_wr_busy` 与 frame_fifo_write 的
    //    `~App_rd_busy` 互斥来保证的）。
    //  温和模式会"照做"，于是**违反契约也照样跑通** —— 上板才炸。
    //  打开严格模式，违反契约就会真的丢读命令，从而在本机复现上板现象：
    //      丢一个纹理读 → 图片图层少一个像素 → 显示侧永远等不到 →
    //      整条场景流水线冻结 → 一帧都写不完 → frame_writer 的 display_valid
    //      永远上不去 → 顶层 vout_data 被强制拉 0 → **全屏黑**
    parameter integer STRICT_SINGLE_PORT = 0,
    //--------------------------------------------------------------------------
    //  ★★ 厂商 4 字分组规则（2026-10-17 新增，默认 0 = 老行为不变）★★
    //--------------------------------------------------------------------------
    //  厂商文档（APUG011 §2.3.4）原文：
    //      "每次读/写的数据以 4 个为单位，每 4 个中第 1 个地址即为首地址，
    //        第 2~4 地址依据首地址做累加；每 4 个时钟可以任意跳跃 1 次地址。"
    //  ⇒ 组内第 2~4 条命令的地址会被**忽略**，控制器用"组首+偏移"自己算。
    //     我们给错地址它不报错，只是悄悄取回别处的数据。
    //
    //  【三种相位假设】文档只说"每 4 个时钟"，没说流中断后相位是否复位，
    //  这个区别决定了"突发中间被打断"到底致不致命。所以做成开关，用仿真判：
    //      0 = 不模拟（老行为，永远 PASS —— 这就是我之前被它骗了 8 版的原因）
    //      1 = 相位随**时钟**自由运行（"每 4 个时钟"的字面理解）
    //          ⇒ 任何一段流若在组中间停顿，恢复后相位就错 ⇒ 前 1~3 个字取错
    //      2 = 相位随**命令**计数、跨空隙不重置（停顿无害，只有组内跳址有害）
    //          ⇒ 只有"连续流里地址跳变"才会错
    parameter integer PHASE_MODE = 0,
    //  写口是否也受分组规则约束。
    //    1 = 是（读写共用一条 4 拍流水线，最悲观）
    //    0 = 否（读写是两套独立的突发引擎 —— 厂商文档写"不支持读写同时操作"，
    //            暗示它们是分开的；真实结构无法从文档确认，所以做成开关）
    parameter integer PHASE_WR_EN = 1,
    //--------------------------------------------------------------------------
    //  ★★ 总线换向代价（2026-10-17 新增，默认 0 = 老行为完全不变）★★
    //--------------------------------------------------------------------------
    //  【真机为什么会"凭空少一拍数据"】
    //  本模型原来假设"每拍都能收下一条命令"，于是"发出去的读命令"永远等于
    //  "回来的读数据"。真控制器做不到：SDRAM 是单口器件，读→写 / 写→读 换向要
    //  等 tWTR / tRTW，还要等上一个方向的突发排空。来不及接的命令，控制器只能
    //  拒收（把端口标忙，或直接丢）。而上层调度器**不看 Sdr_busy**：它已经把这条
    //  命令记进标签环、把突发计数 +1 —— 于是从这一刻起
    //      · 该回来的数据少一拍 ⇒ 显示读 FIFO 欠载（板上 d2=2 就是这个）
    //      · 标签环整体错一位 ⇒ 显示数据和纹理数据互相串（= 整屏乱码/雪花）
    //  这就是"时隙表把口切成 4 拍一段、方向每 6 拍翻一次"的代价。
    //
    //  【本模型的换向模型（取最坏情况）】
    //    · 上一条**真正执行**的命令方向与本条不同，且两条之间端口空闲 < TURN_CYCLES
    //      拍 ⇒ 本条命令被丢弃，并开启 TURN_CYCLES 拍的"忙"窗口（Sdr_busy=1）
    //    · 忙窗口内递进来的命令一律丢弃（对应"上层不看 busy 硬发"）
    //    · 若两条命令之间端口空闲 ≥ TURN_CYCLES 拍（上一个方向已彻底结束）
    //      ⇒ 换向不再收费、命令照收
    //      ★ 这一条正是"同方向长段 + 换向前留空闲槽"能救命的原因
    //  TURN_CYCLES = 0 时整个模块透明（老行为、老 TB 一个数都不变）。
    parameter integer TURN_CYCLES = 0,
    //--------------------------------------------------------------------------
    //  ★★ 周期性刷新忙窗口（2026-10-18 新增，默认 0 = 老行为完全不变）★★
    //--------------------------------------------------------------------------
    //  【为什么必须补这一条（这是"图片层花屏 + 整屏明暗闪烁"的最后一环）】
    //  TURN_CYCLES 只模拟"换向"。而本项目时隙表已经给每次换向留了 16 拍 ID 槽，
    //  换向在有 ID 槽的表里几乎免费 —— 可是真机上还有一样躲不开的东西：
    //  **刷新**。SDRAM 每隔 SELF_REFRESH_INTERVAL 拍必须刷新一次，刷新期间控制器
    //  不收新命令。这个窗口是**周期性**的，跟时隙表转到哪、谁在跑都无关，
    //  所以它必然周期性地落在纹理槽/显示读段中间。
    //  上层若不看 busy 硬发命令（BUSY_HOLD=0）：
    //      丢一条命令 ⇒ 标签环少弹一位 ⇒ 此后 显示/纹理 数据互相串
    //      ⇒ 显示里混进纹理像素（整屏明暗/横带）+ 纹理里混进显示像素（花屏）
    //
    //  【参数怎么取（全部来自可读的官方源码，不是猜的）】
    //    include/global_def.v:
    //        `SELF_REFRESH_INTERVAL = 64000000/`SDR_CLK_PERIOD/2**(`ROW_WIDTH)
    //                              = 64000000/8/2048 = **3906 拍**（125MHz、11 位行地址）
    //    include/sdr_init_ref.enc.v（未加密的头部）:
    //        init_cnt==SELF_REFRESH_INTERVAL-10 ⇒ ref_req 一拍 ⇒ Sdr_ref_req
    //        （即每 3906 拍来一次刷新，一个显示行 3975 拍≈刷新 1 次，
    //          一帧 15.36ms ≈ 刷新 492 次）
    //        一次刷新 = 预充电 + tRP + 自动刷新 + tRFC ≈ 14 拍
    //
    //  REF_PERIOD = 0 ⇒ 不模拟（老行为，所有老 TB 一个数都不变）
    //  REF_BUSY       刷新忙窗口拍数（窗口内递进来的命令一律丢弃 = 控制器不收）
    //  REF_LAG    = 0 忙信号与拒收同拍（控制器提前/同拍给出 busy ⇒ 上层看得见）
    //             = 1 忙信号比拒收晚一拍（上层看不见的那一拍躲不掉 —— 悲观口径）
    parameter integer REF_PERIOD = 0,
    parameter integer REF_BUSY   = 16,
    parameter integer REF_LAG    = 0,
    //  Sdr_init_ref_vld（"刷新进行中"）的脉冲长度。
    //   真机里这是 sdr_init_ref.enc.v 的 `init_vld | ref_vld`，ref_vld 由 Sdr_ref_ack
    //   拉起、保持 20 拍（sdr_ref_ack_sft[19] 才清）⇒ 覆盖整个刷新窗口。
    //   本模型：忙窗口第一拍拉起，保持 REF_VLD 拍。0 = 不产生（输出恒 0）。
    parameter integer REF_VLD    = 24
)(
    input  wire        Sdr_clk,           // 125MHz 主时钟
    input  wire        Sdr_clk_sft,       // 相移时钟（真实 IP 要用；本模型不用）
    input  wire        Rst,               // 高有效复位

    output reg         Sdr_init_done,     // 初始化完成
    output wire        Sdr_init_ref_vld,  // 刷新进行中（REF_PERIOD>0 时每周期一次脉冲）
    output wire        Sdr_busy,          // 控制器忙（换向/刷新窗口内拉高，否则恒 0）

    input  wire        App_ref_req,       // 上层请求刷新（本模型忽略）

    // ---- 写口 ----
    input  wire        App_wr_en,
    input  wire [20:0] App_wr_addr,
    input  wire [3:0]  App_wr_dm,         // 位=1 表示屏蔽该字节（不写）
    input  wire [31:0] App_wr_din,

    // ---- 读口 ----
    input  wire        App_rd_en,
    input  wire [20:0] App_rd_addr,
    output wire        Sdr_rd_en,         // 读数据有效（1 拍）
    output wire [31:0] Sdr_rd_dout,

    // ---- 给 tb 用的观测口 ----
    output reg         rw_conflict,       // 同一拍既读又写 → 拉高（粘滞）
    output reg [31:0]  n_rd_drop,         // 严格模式下被丢掉的读命令数
    output reg [31:0]  n_rd_seen,         // 收到过多少读命令（分母）
    // ★ 4 字分组观测 ★
    output reg [31:0]  n_grp_bad,         // 地址被"组首自动累加"覆盖掉的命令数
    output reg [31:0]  n_grp_seen,        // 分组内的命令总数（分母）
    // ★ 换向观测 ★
    output reg [31:0]  n_turn_drop,       // 换向忙窗口内被丢掉的命令数（TURN_CYCLES=0 时恒 0）
    // ★ 刷新观测 ★
    output reg [31:0]  n_ref_drop         // 刷新忙窗口内被丢掉的命令数（REF_PERIOD=0 时恒 0）
);

//==============================================================================
//  0. 存储体
//==============================================================================
// 2M × 32bit = 8MB。平坦寻址，没有 row/bank/col 概念（见文件头"模拟不了什么"）。
//
// ★ 故意不做上电清零 ★
//   · 清零要用 initial 块写这块数组，而写口在 always 块里 —— 两处驱动同一块
//     存储会被 tools/check_verilog_drivers.py 判成"多驱动"。
//   · 而且保留 x 更有价值：一旦有"没写就读"的 bug，仿真会立刻炸出来，
//     比静默读出 0 好得多。
//   本文件配套的 tb 里所有读都先写过，所以不会读到 x。
//==============================================================================
reg [31:0] mem [0:MEM_WORDS-1];

//==============================================================================
//  1. 上电初始化
//==============================================================================
reg [31:0] init_cnt;

always @(posedge Sdr_clk or posedge Rst) begin
    if (Rst) begin
        init_cnt         <= 32'd0;
        Sdr_init_done    <= 1'b0;
    end else begin
        if (init_cnt < INIT_CYCLES)
            init_cnt <= init_cnt + 32'd1;
        else
            Sdr_init_done <= 1'b1;
        // 刷新由 2.6 节（Sdr_busy / Sdr_init_ref_vld）模拟
    end
end

//==============================================================================
//  2.5 厂商 4 字分组引擎（PHASE_MODE=0 时完全透明）
//==============================================================================
//  ph      : 当前这条命令落在 4 字组的第几个位置
//  grp_head: 本组第 1 条命令的地址（只有组首地址有效）
//  有效地址: ph==0 ? 本命令地址 : grp_head + ph
//==============================================================================
reg [1:0]  ph;
reg [20:0] grp_head;
reg        prev_en;

wire any_en = App_rd_en | App_wr_en;

// 有效地址（PHASE_MODE=0 时 ph 恒 0 ⇒ 等于原地址，行为与老模型一致）
wire [20:0] rd_eff_addr = (ph == 2'd0) ? App_rd_addr : (grp_head + {19'd0, ph});
wire [20:0] wr_eff_addr = (PHASE_WR_EN == 0 || ph == 2'd0) ? App_wr_addr
                                                          : (grp_head + {19'd0, ph});

wire rd_grp_bad = App_rd_en && (ph != 2'd0) && (App_rd_addr != rd_eff_addr);
wire wr_grp_bad = (PHASE_WR_EN != 0) && App_wr_en && (ph != 2'd0) && (App_wr_addr != wr_eff_addr);

always @(posedge Sdr_clk or posedge Rst) begin
    if (Rst) begin
        ph       <= 2'd0;
        grp_head <= 21'd0;
        prev_en  <= 1'b0;
        n_grp_bad  <= 32'd0;
        n_grp_seen <= 32'd0;
    end else begin
        prev_en <= any_en;

        case (PHASE_MODE)
            1: ph <= ph + 2'd1;                    // 按拍自由运行
            2: if (any_en) ph <= ph + 2'd1;        // 按命令计数，空隙不重置
            default: ph <= 2'd0;                   // 不模拟
        endcase

        // 组首地址：相位 0 的那条命令说了算
        if ((PHASE_WR_EN != 0 ? any_en : App_rd_en) && (ph == 2'd0))
            grp_head <= App_rd_en ? App_rd_addr : App_wr_addr;

        if (any_en) n_grp_seen <= n_grp_seen + 32'd1;
        if (rd_grp_bad || wr_grp_bad) n_grp_bad <= n_grp_bad + 32'd1;
    end
end

//==============================================================================
//  2.6 ★ 周期性刷新忙窗口（REF_PERIOD，默认 0 = 不模拟，老行为不变）★
//==============================================================================
//  详细说明见文件头参数表。要点：
//    · 每 REF_PERIOD 拍来一次刷新，窗口 REF_BUSY 拍
//    · 窗口内递进来的命令（读或写）一律丢弃 —— 这就是真机上"凭空少掉的那拍数据"
//    · ref_win 是组合量：BUSY_HOLD=1 且 REF_LAG=0 时上层同拍就能看见并按住 ⇒ 0 丢失
//    · REF_LAG=1 时忙信号晚一拍 ⇒ 每次刷新仍有 1 条命令躲不掉（悲观口径）
//    · 方向/空闲记账只看**真正执行**的命令，被丢的命令不改变总线方向
//==============================================================================
reg  [15:0] ref_cnt;
reg         ref_win_d1;
reg  [7:0]  ref_vld_cnt;

wire        ref_win      = (REF_PERIOD != 0) && (ref_cnt < REF_BUSY[15:0]);
wire        ref_win_vis  = (REF_LAG != 0) ? ref_win_d1 : ref_win;
wire        ref_drop     = ref_win && any_en;
// 刷新进行中（真机 = init_vld | ref_vld）：忙窗口第一拍拉起，保持 REF_VLD 拍
wire        ref_vld_now  = (REF_VLD != 0) && (ref_win && !ref_win_d1);

assign      Sdr_init_ref_vld = (ref_vld_cnt != 8'd0) | ref_vld_now;

always @(posedge Sdr_clk or posedge Rst) begin
    if (Rst) begin
        ref_cnt     <= 16'd0;
        ref_win_d1  <= 1'b0;
        ref_vld_cnt <= 8'd0;
        n_ref_drop  <= 32'd0;
    end else begin
        if (REF_PERIOD != 0) begin
            if (ref_cnt >= REF_PERIOD[15:0] - 16'd1)
                ref_cnt <= 16'd0;
            else
                ref_cnt <= ref_cnt + 16'd1;
        end
        ref_win_d1 <= ref_win;
        if (ref_vld_now)
            ref_vld_cnt <= REF_VLD[7:0];
        else if (ref_vld_cnt != 8'd0)
            ref_vld_cnt <= ref_vld_cnt - 8'd1;
        if (ref_drop)
            n_ref_drop <= n_ref_drop + 32'd1;
    end
end

//==============================================================================
//  2.7 ★ 总线换向代价（TURN_CYCLES，默认 0 = 不模拟，老行为不变）★
//==============================================================================
//  详细说明见文件头参数表。这里只强调三点：
//    · 只有"上一条**真正执行**的命令"才记方向；被丢的命令不改变总线方向
//    · 空闲 ≥ TURN_CYCLES 拍之后换向免费（上一个方向已经彻底结束）
//    · Sdr_busy 比"换向那一拍"晚一拍拉高 ⇒ 上层有一拍反应时间
//      （看得见 busy 的那一拍可以把命令按住；看不见的那一拍只能丢，
//        这正是真机上"上层不看 busy"必然的损失）
//==============================================================================
reg        last_dir;              // 上一条真正执行的命令方向：0=读 1=写
reg        dir_vld;               // 还没执行过任何命令
reg  [7:0] turn_left;             // 忙窗口倒计时
reg  [7:0] idle_cnt;              // 距上一条真正执行的命令过了几拍（饱和 255）
reg        turn_busy_r;           // 换向忙（打一拍，= 老版的 Sdr_busy 换向部分）

wire       dir_now     = App_wr_en;      // 本拍命令的方向（读写永不同拍）
wire       turn_busy   = (turn_left != 8'd0);
wire       turn_change = (TURN_CYCLES != 0) && any_en && dir_vld && !turn_busy
                         && (dir_now != last_dir) && (idle_cnt < TURN_CYCLES);
wire       turn_drop   = (TURN_CYCLES != 0) && any_en && (turn_busy || turn_change);

// 真正执行的命令（严格单口模式里写永远执行，与老模型一致）
//  ★ 刷新忙窗口内递进来的命令同样被丢弃（见 2.6 节）★
wire       wr_exec = App_wr_en && !turn_drop && !ref_win;
wire       rd_exec = App_rd_en && !(STRICT_SINGLE_PORT == 1 && App_wr_en) && !turn_drop && !ref_win;

always @(posedge Sdr_clk or posedge Rst) begin
    if (Rst) begin
        last_dir    <= 1'b0;
        dir_vld     <= 1'b0;
        turn_left   <= 8'd0;
        idle_cnt    <= 8'd0;
        turn_busy_r <= 1'b0;
        n_turn_drop <= 32'd0;
    end else begin
        // ① 忙窗口倒计时：换向那一拍 + 之后 (TURN_CYCLES-1) 拍都不能收命令
        if (turn_change)
            turn_left <= (TURN_CYCLES > 1) ? (TURN_CYCLES - 1) : 8'd1;
        else if (turn_busy)
            turn_left <= turn_left - 8'd1;

        // ② 被丢掉的命令（= 真机上"凭空少掉的那一拍数据"）
        if (turn_drop)
            n_turn_drop <= n_turn_drop + 32'd1;

        // ③ 方向 / 空闲记账
        if (wr_exec || rd_exec) begin
            last_dir <= dir_now;
            dir_vld  <= 1'b1;
            idle_cnt <= 8'd0;
        end else if (idle_cnt != 8'hFF) begin
            idle_cnt <= idle_cnt + 8'd1;
        end

        // ④ 忙信号（换向部分打一拍；刷新部分见 2.6 节）
        turn_busy_r <= turn_busy || turn_change;
    end
end

// ★ 控制器忙 = 换向忙（打一拍）| 刷新忙窗口（组合，见 2.6 节）★
//   换向部分与老版逐拍一致（老 TB 数值不变）；刷新部分默认关（REF_PERIOD=0 ⇒ 恒 0）。
assign Sdr_busy = turn_busy_r | ref_win_vis;

//==============================================================================
//  2. 写通路
//==============================================================================
// App_wr_dm[i] = 1 表示屏蔽第 i 个字节（SDRAM 的 DM 是高有效屏蔽）。
// 本工程顶层把 App_wr_dm 恒接 4'b0000，即"全部写"，所以四个分支都会走。
//==============================================================================
always @(posedge Sdr_clk) begin
    if (wr_exec && Sdr_init_done) begin
        if (!App_wr_dm[0]) mem[wr_eff_addr][ 7: 0] <= App_wr_din[ 7: 0];
        if (!App_wr_dm[1]) mem[wr_eff_addr][15: 8] <= App_wr_din[15: 8];
        if (!App_wr_dm[2]) mem[wr_eff_addr][23:16] <= App_wr_din[23:16];
        if (!App_wr_dm[3]) mem[wr_eff_addr][31:24] <= App_wr_din[31:24];
    end
end

//==============================================================================
//  3. 读通路：固定延迟移位链（约定与 SD/tb_frame_rw.v 的 SDR_LAT 一致）
//==============================================================================
//
//  【延迟是怎么凑成恰好 RD_LATENCY 拍的】
//  设 posedge E 采样到 App_rd_en=1。
//      E+1 之后 : rsp_v[0] = 1
//      E+2 之后 : rsp_v[1] = 1
//      ...
//      E+k 之后 : rsp_v[k-1] = 1
//      E+RD_LATENCY 之后 : rsp_v[RD_LATENCY-1] = 1
//  rsp_v[RD_LATENCY-1] 就是 Sdr_rd_en（组合输出），所以它在
//  [E+RD_LATENCY-1, E+RD_LATENCY) 这一拍为高，
//  下游在 posedge E+RD_LATENCY 采样到它 → 延迟恰好 RD_LATENCY 拍 ✓
//
//  数组要 RD_LATENCY 个元素（0 .. RD_LATENCY-1）。要求 RD_LATENCY >= 1。
//==============================================================================
reg        rsp_v [0:RD_LATENCY-1];
reg [31:0] rsp_d [0:RD_LATENCY-1];

integer k;

// ★ 严格单口模式：同拍有写 → 这个读命令被丢弃 ★
//   ★ 换向忙窗口内递进来的读命令同样被丢弃（见 2.7 节）★
//   （TURN_CYCLES=0 且非严格模式时 rd_exec 恒等于 App_rd_en，行为与老模型完全一样）
always @(posedge Sdr_clk or posedge Rst) begin
    if (Rst) begin
        for (k = 0; k < RD_LATENCY; k = k + 1) begin
            rsp_v[k] <= 1'b0;
            rsp_d[k] <= 32'd0;
        end
        rw_conflict <= 1'b0;
        n_rd_drop   <= 32'd0;
        n_rd_seen   <= 32'd0;
    end else begin
        // 第 0 级：从命令口取新值（地址在这一拍就被读出，之后只随链传递）
        if (rd_exec) begin
            rsp_v[0] <= 1'b1;
            rsp_d[0] <= mem[rd_eff_addr];
        end else begin
            rsp_v[0] <= 1'b0;
        end

        // 第 1 .. RD_LATENCY-1 级：逐级传递
        for (k = 1; k < RD_LATENCY; k = k + 1) begin
            rsp_v[k] <= rsp_v[k-1];
            rsp_d[k] <= rsp_d[k-1];
        end

        // 单口冲突观测（粘滞）+ 严格模式下的丢弃计数
        // 【注意】计数**不能**再带 Sdr_init_done 条件：命令在初始化期间照发，
        //   带了条件会把开头那几十拍漏掉，丢命令却报 0（真踩过一次）。
        if (App_rd_en && App_wr_en) begin
            rw_conflict <= 1'b1;
            if (STRICT_SINGLE_PORT == 1)
                n_rd_drop <= n_rd_drop + 32'd1;
        end
        if (App_rd_en)
            n_rd_seen <= n_rd_seen + 32'd1;
    end
end

// 组合输出（与 SD/tb_frame_rw.v:105-108 的写法一致）
assign Sdr_rd_en   = rsp_v[RD_LATENCY-1];
assign Sdr_rd_dout = rsp_d[RD_LATENCY-1];

endmodule
