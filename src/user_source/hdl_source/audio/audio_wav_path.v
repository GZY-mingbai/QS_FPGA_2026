//=============================================================================
//  audio_wav_path.v —— SD 卡 WAV 播放通道（音频应用层）
//=============================================================================
//
//  【整条链路】
//
//     sd_bus_arbiter ──▶ sd_audio_reader ──▶ img_afifo(4KB) ──▶ wav_player
//        (50MHz)            (50MHz)          50M → 25M           (25MHz)
//                                                                   │
//                                          audio_src_sel(SRC_SEL=3) │
//                                                                   ▼
//                                                     ACR → HDMI 1.4b 核
//
//  【为什么用一个异步 FIFO 隔开】
//  SD 侧 50MHz、音频输出侧 25MHz，两者没有相位关系。FIFO 同时干三件事：
//    1. 跨时钟域（灰度码指针，见 img_afifo）
//    2. 吸收 SD 的突发（一个扇区 2.67ms 的量一次性灌进来）
//    3. 吸收仲裁等待（图像装载时最坏等一个扇区 ~360us）
//
//  【为什么用 img_afifo 而不是新写一个】
//  它已经是工程里验证过的参数化双时钟 FIFO（1024 深会落 BRAM），
//  `dout` 是 show-ahead（队首直出），正好满足 wav_player 的 FWFT 语义。
//  ★ 它是**独立例化**，不复用图片图层那个实例，互不影响。
//
//  【预灌（prefill）】
//  开关刚打开时 FIFO 是空的，立刻开播必然欠载（爆音）。
//  所以先等 rcount ≥ PREFILL（默认 512 字 ≈ 10.7ms）再放行 wav_player。
//  读侧看到的 rcount 是**保守值**（写指针同步晚两拍），只会晚不会早。
//
//  【关闭 = 双端复位 FIFO】
//  SW2 关闭时同时拉住 FIFO 的 wrst / rrst（两个指针都回 0），
//  并让 reader 把当前扇区吃完、计数器归零。再次打开时：
//  空 FIFO → 重新预灌 → 从 BASE_SECTOR 第一个样本开始。
//
//  【时钟域】
//    I_clk_sd  50MHz：reader + FIFO 写侧
//    I_clk_v   25MHz：FIFO 读侧 + wav_player
//    I_play_async 来自按键/拨码（clk 50MHz 域），两个域各打两拍。
//=============================================================================

module audio_wav_path #(
    parameter integer BASE_SECTOR = 0,          // WAV data 起始扇区
    parameter integer SKIP_BYTES  = 44,         // 首扇区跳过的字节（< 512）
    parameter integer TOTAL_WORDS = 0,          // 要播的 32bit 样本字数
    parameter integer FIFO_AW     = 10,         // 1024 字 = 4KB
    parameter integer PREFILL     = 512,        // 预灌水位（字）
    parameter integer REQ_LEVEL   = 768,        // 预取门槛
    parameter integer URGENT_LEVEL= 256         // 抢占门槛
)(
    input  wire        I_clk_sd,                // sd_card_clk 50MHz
    input  wire        I_rst_sd,
    input  wire        I_clk_v,                 // video_clk 25MHz
    input  wire        I_rst_v,
    input  wire        I_play_async,            // 1 = 播放（来自 SW2，异步）
    input  wire        I_loop,                  // ★ 1 = 播完回开头；0 = 播完停住
    // ★ 2026-11-04：换曲（I_clk_sd 域，单拍）。I_trk_sec = 文件首扇区；
    //   之后的 WAV 头由 reader 自己找 "data" 块（不再用编译期的 SKIP/TOTAL）
    input  wire        I_trk_set,
    input  wire [31:0] I_trk_sec,

    //------------------------------------------------------------------
    //  SD 客户口（接 sd_bus_arbiter 的音频口）
    //------------------------------------------------------------------
    output wire        O_sd_req,
    output wire [31:0] O_sd_addr,
    output wire        O_sd_urgent,
    input  wire [7:0]  I_sd_data,
    input  wire        I_sd_dv,
    input  wire        I_sd_end,

    //------------------------------------------------------------------
    //  音频输出（video_clk 域，接 audio_src_sel 的 I_ext_*）
    //------------------------------------------------------------------
    output wire        O_audio_valid,
    output wire [23:0] O_audio_left,
    output wire [23:0] O_audio_right,
    output wire        O_underrun,              // 粘滞：曾经缺样本
    output wire        O_drop,                  // 粘滞：FIFO 溢出错（不该出现）
    output wire        O_playing,              // 1 = 已预灌完成、正在出声
    output wire [10:0] O_fifo_level,           // FIFO 现有字数（调试/数码管用）
    output wire        O_pass_end,              // ★ 2026-10-27：整首**播完**（含 FIFO 排空，仅 loop=0）
    output wire        O_pass_tick              // ★ 2026-11-08：每遍播完单拍（loop=1 也有）
);

//=============================================================================
//  播放开关：两个域各打两拍
//=============================================================================
reg [2:0] p_sd;
reg [2:0] p_v;

always @(posedge I_clk_sd or posedge I_rst_sd) begin
    if (I_rst_sd) p_sd <= 3'd0;
    else          p_sd <= {p_sd[1:0], I_play_async};
end

always @(posedge I_clk_v or posedge I_rst_v) begin
    if (I_rst_v) p_v <= 3'd0;
    else         p_v <= {p_v[1:0], I_play_async};
end

wire play_sd = p_sd[2];         // 50MHz 域
wire play_v  = p_v[2];          // 25MHz 域

// 关闭时把 FIFO 两端都按在复位里：两个指针同时回 0，安全且干净。
wire fifo_wrst = I_rst_sd | ~play_sd;
wire fifo_rrst = I_rst_v  | ~play_v;

//=============================================================================
//  样本 FIFO（独立实例，绝不与图片图层的 FIFO 共用）
//=============================================================================
wire [31:0]         fifo_dout;
wire                fifo_empty;
wire [FIFO_AW:0]    fifo_rcount;
wire [FIFO_AW:0]    fifo_wlevel;
wire                fifo_wfull;
wire                smp_rd;
wire                fifo_we;
wire [31:0]         fifo_din;

img_afifo #(
    .DW (32),
    .AW (FIFO_AW)
) u_fifo (
    // ---- 写侧（SD 50MHz）----
    .wclk   (I_clk_sd),
    .wrst   (fifo_wrst),
    .we     (fifo_we),
    .din    (fifo_din),
    .wfull  (fifo_wfull),
    .wlevel (fifo_wlevel),

    // ---- 读侧（video 25MHz）----
    .rclk   (I_clk_v),
    .rrst   (fifo_rrst),
    .re     (smp_rd),
    .dout   (fifo_dout),
    .rempty (fifo_empty),
    .rcount (fifo_rcount)
);

//=============================================================================
//  扇区读取器
//=============================================================================
sd_audio_reader #(
    .BASE_SECTOR  (BASE_SECTOR),
    .SKIP_BYTES   (SKIP_BYTES),
    .TOTAL_WORDS  (TOTAL_WORDS),
    .FIFO_AW      (FIFO_AW),
    .PREFILL      (PREFILL),        // ★ 必须与下面的 started 门槛一致
    .REQ_LEVEL    (REQ_LEVEL),
    .URGENT_LEVEL (URGENT_LEVEL)
) u_reader (
    .I_clk        (I_clk_sd),
    .I_rst        (I_rst_sd),
    .I_en         (play_sd),
    .I_loop       (I_loop),             // ★ 2026-10-29：OSD 系统页"循环播放"
    .I_trk_set    (I_trk_set),
    .I_trk_sec    (I_trk_sec),

    .O_sd_req     (O_sd_req),
    .O_sd_addr    (O_sd_addr),
    .O_sd_urgent  (O_sd_urgent),
    .I_sd_data    (I_sd_data),
    .I_sd_dv      (I_sd_dv),
    .I_sd_end     (I_sd_end),

    .I_fifo_level (fifo_wlevel),
    .I_fifo_full  (fifo_wfull),
    .O_fifo_we    (fifo_we),
    .O_fifo_data  (fifo_din),
    .O_drop       (O_drop),
    .O_pass_done  (pass_done_w),
    // ★ 2026-11-08：“播完”的两种口径全部由 reader 产出，本模块只透传
    //   （原来这里还有一份 pass_end_lvl 本地检测，和 reader 里的定义重复；
    //    合到一处后 loop=0 逐拍等价、loop=1 也有每遍单拍可用了）。
    .O_pass_end   (O_pass_end),
    .O_pass_tick  (O_pass_tick)
);

//=============================================================================
//  预灌门控
//=============================================================================
//  【为什么用 rcount 而不是写域水位】
//  读侧要用它做决定，用写域水位就得跨时钟域传多比特。rcount 本来就在读域，
//  而且它是**保守值**（写指针同步晚两拍）⇒ 只会让我们多等，不会提前放行。
reg started;

always @(posedge I_clk_v or posedge I_rst_v) begin
    if (I_rst_v)
        started <= 1'b0;
    else if (!play_v)
        started <= 1'b0;
    else if (fifo_rcount >= PREFILL)
        started <= 1'b1;
end

wire player_play = play_v && started;

//=============================================================================
//  播放内核
//=============================================================================
wav_player #(
    .CLK_HZ      (25_000_000),
    .SAMPLE_RATE (48_000),
    .TEST_SRC    (0),                   // 走真实样本口
    .TEST_TONE_HZ(1_000),
    .TEST_AMP16  (16'sd12000)
) u_player (
    .I_clk              (I_clk_v),
    .I_rst              (I_rst_v),
    .I_play             (player_play),

    .I_smp_valid        (~fifo_empty),  // FWFT：dout 就是队首
    .O_smp_rd           (smp_rd),
    .I_smp_data         (fifo_dout),

    .O_audio_valid      (O_audio_valid),
    .O_audio_left_data  (O_audio_left),
    .O_audio_right_data (O_audio_right),
    .O_underrun         (O_underrun)
);

// 播放状态
assign O_playing    = player_play;
assign O_fifo_level = fifo_wlevel;   // 写域水位，调试显示用

endmodule
