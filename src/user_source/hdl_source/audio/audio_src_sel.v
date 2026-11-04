//=============================================================================
//  audio_src_sel.v —— 音频源选择（音频应用层的统一入口）
//=============================================================================
//
//  【这一层解决什么问题】
//  以前顶层的音频支路是"写死"的一条链：
//      hdmi_audio_tone_i2s_64fs → I2S 三线 → I2S_receiver → audio_valid/L/R
//  想换音源（例如以后的 SD 卡 wav 播放器），必须回顶层改连线。
//
//  本模块把"音源"收敛成一个统一接口：
//      O_audio_valid + O_audio_left_data[23:0] + O_audio_right_data[23:0]
//  下游（audio_arc_calculate / HDMI 1.4b 核）只认这三个信号，永远不用改。
//
//  【SRC_SEL 选什么】
//      0 = PCM 音阶直出（★ 默认 ★）：hdmi_audio_tone_pcm_scale
//          在 video_clk(25MHz) 域用"分数分频"直接生成 48kHz 的 24bit PCM。
//          相比旧方案省掉 I2S 发送 + 接收一对模块，也不再依赖 audio_mclk
//          的频率精度（旧 PLL 实际给的是 12.295MHz，音高偏 0.06%）。
//      1 = I2S 回环（旧方案）：hdmi_audio_tone_i2s_64fs + I2S_receiver
//          保留是为了继续验证 I2S 时序（pin.adc 里 J13/F13/L14 三根线的约束
//          目前是注释状态，要用时放开即可）。
//      2 = WAV 播放内核（step 1）：wav_player
//          本步先用它内部的 1kHz 测试样本源（TEST_SRC=1）验证节拍与位宽；
//          step 2/3 把 I_smp_* 接到 Flash / SD 的样本 FIFO 后，
//          改成 TEST_SRC=0 就是真正的 wav 播放，本文件其它地方不用动。
//      3 = 外部音源（step 3）：SD 卡 WAV 播放通道（audio_wav_path）
//          整条链路（SD → FIFO → wav_player）都在 audio_wav_path 里，
//          它直接把 48kHz 的 24bit L/R + valid 送进来，本模块只做三选一。
//          ★ 顶层 AUDIO_SRC=3 时用这个。
//
//  【时钟域】
//  四种源最终输出的 valid 都对齐在 I_clk（= video_clk 25MHz）：
//    · SRC_SEL=0：源本身就在 I_clk 域，valid 直接输出；
//    · SRC_SEL=1：I2S 在 I_mclk 域产生，但 I2S_receiver 在 I_clk 域打三拍采样、
//                 解串，输出同样落在 I_clk 域；
//    · SRC_SEL=2：wav_player 本身就跑在 I_clk 域；
//    · SRC_SEL=3：audio_wav_path 的 wav_player 也在 I_clk 域。
//  所以下游 ACR 计数器（也在 video_clk 域）可以直接用，不需要再做跨域。
//
//  【step 2/3 要改哪里】
//  只改 SRC_SEL=2 分支里的 wav_player 例化：
//      · TEST_SRC 改 0；
//      · I_smp_valid / I_smp_data 接样本 FIFO 的 (empty 取反) / dout；
//      · O_smp_rd 接 FIFO 的 rd_en；
//      · 想加"播放/暂停"就把 I_play 引出到本模块端口。
//  顶层仍然一行都不用动。
//
//  【注意】本模块不含任何存储访问；SD 读口在 audio_wav_path / 顶层仲裁器里。
//=============================================================================

module audio_src_sel #(
    parameter integer        SRC_SEL           = 0,             // 0=PCM直出；1=I2S回环；2=内部测试；3=SD WAV
    parameter signed [23:0]  AMP               = 24'sd2000000,  // 幅度（两路源共用）
    parameter integer        NOTE_HOLD_SAMPLES  = 24_000        // 每个音 0.5s @48kHz
)(
    input  wire        I_clk,               // video_clk 25MHz（输出域）
    input  wire        I_rst,               // 复位，高有效
    input  wire        I_mclk,              // 12.288MHz，只有 SRC_SEL=1 的 I2S 源用
    input  wire        I_play,              // 播放开关（SW2，已同步到本域）；SRC_SEL=2/3 用

    //------------------------------------------------------------------
    //  外部音源（SRC_SEL=3）：由 audio_wav_path 在 video_clk 域直接产出
    //------------------------------------------------------------------
    input  wire        I_ext_valid,
    input  wire [23:0] I_ext_left,
    input  wire [23:0] I_ext_right,

    output wire        O_audio_valid,       // 一个采样点有效（单拍脉冲）
    output wire [23:0] O_audio_left_data,   // 左声道 24bit 有符号
    output wire [23:0] O_audio_right_data   // 右声道 24bit 有符号
);

generate

//-----------------------------------------------------------------------------
//  SRC_SEL = 0：PCM 音阶直出（默认）
//    在 video_clk 域直接产 48kHz 的 valid + 24bit PCM，经 HDMI 核直接打包，
//    不经过 I2S 串行/解串这一环。
//-----------------------------------------------------------------------------
if (SRC_SEL == 0) begin : g_pcm_tone

    hdmi_audio_tone_pcm_scale #(
        .CLK_FREQ_HZ       (25_000_000),        // 与 video_clk 一致
        .SAMPLE_RATE_HZ    (48_000),
        .AMP               (AMP),
        .NOTE_HOLD_SAMPLES (NOTE_HOLD_SAMPLES)
    ) u_pcm_tone (
        .I_clk              (I_clk),
        .I_rst              (I_rst),
        .O_audio_valid      (O_audio_valid),
        .O_audio_left_data  (O_audio_left_data),
        .O_audio_right_data (O_audio_right_data)
    );

    // 本分支不需要音频主时钟。显式"吃掉"这个端口，避免综合/仿真报 unused。
    wire unused_mclk = I_mclk;

end

//-----------------------------------------------------------------------------
//  SRC_SEL = 1：I2S 回环（旧方案，保留用于验证 I2S 时序）
//    audio_mclk 域产生 BCLK/LRCK/DOUT → video_clk 域三拍同步解串 → PCM。
//-----------------------------------------------------------------------------
else if (SRC_SEL == 1) begin : g_i2s_loop

    wire i2s_bclk;
    wire i2s_lrck;
    wire i2s_dout;

    hdmi_audio_tone_i2s_64fs #(
        .PHASE_INC        (32'd39370534),       // 兼容旧顶层的参数，模块内部不使用
        .AMP              (AMP),
        .NOTE_HOLD_FRAMES (NOTE_HOLD_SAMPLES)   // 0.5s @48kHz
    ) u_tone (
        .I_mclk     (I_mclk),
        .I_rst      (I_rst),
        .O_i2s_BCLK (i2s_bclk),
        .O_i2s_LRCK (i2s_lrck),
        .O_i2s_DOUT (i2s_dout)
    );

    I2S_receiver u_i2s_rx (
        .I_clk              (I_clk),
        .I_rst              (I_rst),
        .I_i2s_BCLK         (i2s_bclk),
        .I_i2s_LRCK         (i2s_lrck),
        .I_i2s_DOUT         (i2s_dout),
        .O_audio_valid      (O_audio_valid),
        .O_audio_left_data  (O_audio_left_data),
        .O_audio_right_data (O_audio_right_data)
    );

end

//-----------------------------------------------------------------------------
//  SRC_SEL = 2：WAV 播放内核（内部 1kHz 测试源）
//    wav_player 自身在 I_clk 域跑 48kHz 节拍，输出 24bit L/R。
//    TEST_SRC=1 时不需要上游数据，用来单独验证节拍/位宽/开关。
//-----------------------------------------------------------------------------
else if (SRC_SEL == 2) begin : g_wav_player

    wav_player #(
        .CLK_HZ       (25_000_000),     // 与 video_clk 一致
        .SAMPLE_RATE  (48_000),
        .TEST_SRC     (1),              // 内部 1kHz；真实播放走 SRC_SEL=3
        .TEST_TONE_HZ (1_000),
        .TEST_AMP16   (16'sd12000)      // 约 37% 满量程（≈ -8.7 dBFS）
    ) u_wav_player (
        .I_clk              (I_clk),
        .I_rst              (I_rst),
        .I_play             (I_play),   // SW2 开关：关掉即静音（仍按 48kHz 送 0）
        .I_smp_valid        (1'b0),     // TEST_SRC=1 时用不到
        .O_smp_rd           (),
        .I_smp_data         (32'd0),
        .O_audio_valid      (O_audio_valid),
        .O_audio_left_data  (O_audio_left_data),
        .O_audio_right_data (O_audio_right_data),
        .O_underrun         ()
    );

    // 本分支不需要音频主时钟。
    wire unused_mclk = I_mclk;

end

//-----------------------------------------------------------------------------
//  SRC_SEL = 3：外部音源（SD 卡 WAV）
//    整条 SD → FIFO → wav_player 链路在 audio/audio_wav_path.v 里，
//    它已经按 48kHz 产出 valid + 24bit L/R，这里只做旁路。
//-----------------------------------------------------------------------------
else begin : g_ext_src

    assign O_audio_valid      = I_ext_valid;
    assign O_audio_left_data  = I_ext_left;
    assign O_audio_right_data = I_ext_right;

    // 本分支不需要音频主时钟。
    wire unused_mclk = I_mclk;

end

endgenerate

endmodule
