//=============================================================================
// 文件名   : design_top_i2s_test_pll.v
// 模块名   : design_top_i2s_test_single_pll_fix64   ★★ 顶层测试模块 ★★
//
// 功能     : 【音频链路自测顶层】—— 这是一个"独立测试用"的顶层，
//            不是工程最终的主顶层（主顶层是 top_tf_hdmi_audio.v）。
//            它把"音频发生器 -> I2S 接收器"直接对起来做回环，
//            再把收到的音频送给 HDMI 发送器，输出到 HDMI 接口。
//
// 用途     : 用来验证"音频产生 -> I2S 时序 -> ACR 计算 -> HDMI 打包"这一整条链路。
//            因为它不涉及 SD 卡、SDRAM、图像，所以问题范围很小、很好定位。
//            —— 建议初学者先跑通这个顶层，看到屏幕上出图案、听到声音，
//               再去跑带 SD 卡的主顶层。这是"分而治之"的调试策略。
//
// 和主顶层的三点不同：
//   ① 没有 SD 卡 / SDRAM，视频源用的是 video_source_test（递增图案）；
//   ② 分辨率参数写的是 1080P（注意：但 PLL 实际给的像素时钟是 74.25MHz，
//      而 video_source_test 的参数和 HDMI 发送器参数不一致——
//      这属于"测试代码没对齐"，实际跑起来画面可能不正常，
//      说明这个测试顶层主要是拿来验证音频的，不是验证视频的）；
//   ③ 音频用的是固定 1000Hz 单音（便于用耳朵/示波器判断）。
//
// 数据流图：
//     50MHz 晶振
//        │
//        └─ PLL_HDMI_AUDIO ──┬─ 74.25MHz  pixel  ──► I2S_receiver / ACR / HDMI发送器
//                            ├─ 371.25MHz serial ──► hdmi_phy_wrapper
//                            └─ 12.288MHz audio  ──► i2s_test_tone_gen
//                                                          │ I2S 三线
//                                                          ▼
//                                                    I2S_receiver
//                                                          │ 左右声道 PCM
//                                                          ▼
//                                    audio_arc_calculate ──► HDMI 发送器 ──► PHY ──► 引脚
//
// 如何使用 ?
//   把本文件设为工程顶层（或做成一个单独的 TD 工程）编译下载即可。
//   按键 I_key_in 会触发一次 EDID 读取（读显示器的能力信息），属于附加功能。
//=============================================================================

module design_top_i2s_test_single_pll_fix64 (
    input  wire I_sys_clk,        // 系统时钟输入（板上 50MHz 晶振）
    input  wire I_key_in,         // 按键输入（用于触发 EDID 读取）

    output wire O_ddc_scl,        // DDC（I2C）时钟 -> HDMI 接口
    inout  wire IO_ddc_sda,       // DDC（I2C）数据 -> HDMI 接口（双向）

    output wire O_tmds_ch0_p,     // TMDS 通道 0 差分输出
    output wire O_tmds_ch1_p,     // TMDS 通道 1
    output wire O_tmds_ch2_p,     // TMDS 通道 2
    output wire O_tmds_clk_p      // TMDS 时钟通道
);

    wire       S_pll_lock;        // PLL 锁定标志（锁定了才放行复位）
    wire       S_rst;             // 内部复位
    wire       S_pixel_clk;       // 像素时钟
    wire       S_serial_clk;      // 串行时钟
    wire       S_audio_mclk;      // 音频主时钟

    wire       S_i2s_BCLK;        // I2S 位时钟
    wire       S_i2s_LRCK;        // I2S 声道时钟
    wire       S_i2s_DOUT;        // I2S 数据

    wire       S_axis_s_user;     // 视频流 SOF
    wire       S_axis_s_valid;    // 视频流 valid
    wire       S_axis_s_last;     // 视频流 EOL
    wire[23:0] S_axis_s_data;     // 视频流数据
    wire       S_axis_s_ready;    // 下游 ready

    wire       S_audio_valid;      // 音频采样有效
    wire[23:0] S_audio_left_data;  // 左声道
    wire[23:0] S_audio_right_data; // 右声道

    wire[9:0]  S_ch0_tmds_data;    // TMDS 编码 CH0
    wire[9:0]  S_ch1_tmds_data;    // TMDS 编码 CH1
    wire[9:0]  S_ch2_tmds_data;    // TMDS 编码 CH2
    wire[9:0]  S_clk_tmds_data;    // TMDS 编码 时钟通道

    wire       S_acr_valid;        // ACR 有效
    wire[19:0] S_acr_cts;          // ACR CTS
    wire[19:0] S_acr_n;            // ACR N

    wire       S_key_trig;         // 按键触发脉冲
    wire       S_edid_valid;       // EDID 读回有效
    wire[7:0]  S_edid_data;        // EDID 数据

    //--------------------------------------------------------------------
    // PLL：一个 PLL 出三路时钟（本模块因此叫 "single_pll"）
    //   refclk = 50MHz 输入
    //   clk0 = 74.25MHz  像素时钟（1080P@60Hz 的标准频率）
    //   clk1 = 371.25MHz 串行时钟（= 74.25 × 5）
    //   clk2 = 12.288MHz 音频主时钟（= 48000 × 256，音频行业标准值）
    //--------------------------------------------------------------------
    PLL_HDMI_AUDIO U_pll_hdmi_audio(
        .refclk   ( I_sys_clk    ),
        .reset    ( 1'b0         ),      // PLL 自身不复位
        .extlock  ( S_pll_lock   ),
        .clk0_out ( S_pixel_clk  ),
        .clk1_out ( S_serial_clk ),
        .clk2_out ( S_audio_mclk )
    );

    // PLL 没锁定时保持复位 —— 这是标准做法，
    // 因为 PLL 锁定前输出时钟不稳，此时跑逻辑会出各种诡异问题。
    assign S_rst = ~S_pll_lock;

    //--------------------------------------------------------------------
    // 音频源：产生固定 1000Hz 单音，输出 I2S 三线信号
    //--------------------------------------------------------------------
    i2s_test_tone_gen_single_pll_fix64 #(
        .SAMPLE_RATE ( 48000 ),
        .SAMPLE_BITS ( 24    ),
        .TONE_HZ     ( 1000  )
    ) u_i2s_test_tone_gen_single_pll_fix64 (
        .I_mclk      ( S_audio_mclk ),
        .I_rst       ( S_rst        ),
        .O_i2s_BCLK  ( S_i2s_BCLK   ),
        .O_i2s_LRCK  ( S_i2s_LRCK   ),
        .O_i2s_DOUT  ( S_i2s_DOUT   )
    );

    //--------------------------------------------------------------------
    // I2S 接收：把刚发出去的 I2S 位流再收回来（回环测试）
    // 用像素时钟来采样，因为 I2S_receiver 内部已经做了跨时钟域同步。
    //--------------------------------------------------------------------
    I2S_receiver u_I2S_receiver(
        .I_clk              ( S_pixel_clk        ),
        .I_rst              ( S_rst              ),
        .I_i2s_BCLK         ( S_i2s_BCLK         ),
        .I_i2s_LRCK         ( S_i2s_LRCK         ),
        .I_i2s_DOUT         ( S_i2s_DOUT         ),
        .O_audio_valid      ( S_audio_valid      ),
        .O_audio_left_data  ( S_audio_left_data  ),
        .O_audio_right_data ( S_audio_right_data )
    );

    //--------------------------------------------------------------------
    // ACR 参数计算：产生 HDMI 音频包必需的 CTS / N
    // 注意用的是"像素时钟"来数 CTS（见 audio_arc_calculate.v 注释）
    //--------------------------------------------------------------------
    audio_arc_calculate #(
        .ACR_N         ( 6144 )
    ) u_audio_arc_calculate (
        .I_clk         ( S_pixel_clk    ),
        .I_rst         ( S_rst          ),
        .I_audio_valid ( S_audio_valid  ),
        .O_acr_valid   ( S_acr_valid    ),
        .O_acr_cts     ( S_acr_cts      ),
        .O_acr_n       ( S_acr_n        )
    );

    //--------------------------------------------------------------------
    // 按键消抖：输出的脉冲用来触发一次 EDID 读取
    // （注意 I_rst_n 接的是 S_pll_lock，即"锁定=不复位"）
    //--------------------------------------------------------------------
    key_remove_shakes u_key_remove_shakes(
        .I_clk          ( S_pixel_clk ),
        .I_rst_n        ( S_pll_lock  ),
        .I_key_in       ( I_key_in    ),
        .O_key_trig_out ( S_key_trig  )
    );

    //--------------------------------------------------------------------
    // 视频源：测试图案发生器（这里配的是 1080P 参数）
    //--------------------------------------------------------------------
    video_source_test #(
        .HTOTAL        ( 2200 ),
        .HACTIVE       ( 1920 ),
        .HFP           ( 88   ),
        .HSA           ( 44   ),
        .HBP           ( 148  ),
        .VTOTAL        ( 1125 ),
        .VACTIVE       ( 1080 ),
        .VFP           ( 4    ),
        .VSA           ( 5    ),
        .VBP           ( 36   )
    ) U_video_source_test(
        .I_clk         ( S_pixel_clk    ),
        .I_rst         ( S_rst          ),
        .I_tpg_trig_en ( 1'b1           ),   // 常开，一上电就出图
        .O_video_user  ( S_axis_s_user  ),
        .O_video_valid ( S_axis_s_valid ),
        .O_video_last  ( S_axis_s_last  ),
        .O_video_data  ( S_axis_s_data  ),
        .I_video_ready ( S_axis_s_ready )
    );

    //--------------------------------------------------------------------
    // HDMI 1.4b 发送核心（官方 IP 的包装）
    // 它负责：TMDS 编码、音频包打包、EDID 读取、DDC 通信
    // 注意这里的视频参数（1280x720）和上面 video_source_test 的参数（1920x1080）不一致，
    //      这是测试代码遗留问题；实际正式工程请看 top_tf_hdmi_audio.v 的配置。
    //--------------------------------------------------------------------
    hdmi_1_4b_transmitter_core_wrapper #(
        .DEVICE                 ( "EG"     ),
        .HTOTAL                 ( 1650     ),
        .HSA                    ( 40       ),
        .HFP                    ( 110      ),
        .HBP                    ( 220      ),
        .HACTIVE                ( 1280     ),
        .VTOTAL                 ( 750      ),
        .VSA                    ( 5        ),
        .VFP                    ( 5        ),
        .VBP                    ( 20       ),
        .VACTIVE                ( 720      ),
        .VIDEO_VIC              ( 69       ),      // 视频识别码（VIC 69 = 1280x720@60Hz）
        .VIDEO_TPG              ( "Enable" ),
        .VIDEO_FORMAT           ( "RGB"    ),
        .AUDIO_SAMPLE_RATE      ( "48K"    ),
        .IIC_SCL_DIV            ( 125      )       // DDC 时钟分频
    ) u_hdmi_1_4b_transmitter_core_wrapper(
        .I_pixel_clk        ( S_pixel_clk        ),
        .I_rst              ( S_rst              ),
        .I_edid_read_trig   ( S_key_trig         ),   // 按键触发读 EDID
        .O_edid_read_valid  ( S_edid_valid       ),
        .O_edid_read_data   ( S_edid_data        ),
        .I_axis_s_user      ( S_axis_s_user      ),
        .I_axis_s_valid     ( S_axis_s_valid     ),
        .I_axis_s_last      ( S_axis_s_last      ),
        .I_axis_s_data      ( S_axis_s_data      ),
        .O_axis_s_ready     ( S_axis_s_ready     ),
        .I_audio_valid      ( S_audio_valid      ),
        .I_audio_left_data  ( S_audio_left_data  ),
        .I_audio_right_data ( S_audio_right_data ),
        .I_acr_valid        ( S_acr_valid        ),
        .I_acr_cts          ( S_acr_cts          ),
        .I_acr_n            ( S_acr_n            ),
        .O_video_locked     (                    ),   // 输出未使用
        .O_ddc_scl          ( O_ddc_scl          ),
        .IO_ddc_sda         ( IO_ddc_sda         ),
        .O_ch0_tmds_data    ( S_ch0_tmds_data    ),
        .O_ch1_tmds_data    ( S_ch1_tmds_data    ),
        .O_ch2_tmds_data    ( S_ch2_tmds_data    ),
        .O_clk_tmds_data    ( S_clk_tmds_data    )
    );

    //--------------------------------------------------------------------
    // HDMI 物理层：TMDS 数据 -> 差分引脚
    //--------------------------------------------------------------------
    hdmi_phy_wrapper #(
        .DEVICE ( "EG" )
    ) u_hdmi2phy_wrapper(
        .I_pixel_clk        ( S_pixel_clk     ),
        .I_serial_clk       ( S_serial_clk    ),
        .I_rst              ( S_rst           ),
        .I_tmds_channel_0   ( S_ch0_tmds_data ),
        .I_tmds_channel_1   ( S_ch1_tmds_data ),
        .I_tmds_channel_2   ( S_ch2_tmds_data ),
        .I_tmds_channel_clk ( S_clk_tmds_data ),
        .O_tmds_ch0_p       ( O_tmds_ch0_p    ),
        .O_tmds_ch1_p       ( O_tmds_ch1_p    ),
        .O_tmds_ch2_p       ( O_tmds_ch2_p    ),
        .O_tmds_clk_p       ( O_tmds_clk_p    )
    );

endmodule
