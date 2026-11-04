//=============================================================================
// 文件名   : i2s_test_tone_gen_pll.v
// 模块名   : i2s_test_tone_gen_single_pll_fix64
// 功能     : 【固定频率单音发生器 + I2S 发送（专用测试版）】
//            产生一个指定频率（默认 1000Hz）的方波音，通过 I2S 送出。
//            是 hdmi_audio_tone_i2s_64fs.v 的"简化测试版"：
//            那个版本会循环播放 do-re-mi 音阶，这个版本只发一个固定音。
//
// 为什么要单发一个固定音？
//   测试音频链路时，变化的东西越少越好。
//   1000Hz 是音频测试的标准频率：
//     · 人耳能清楚听到（不会太高也不会太低）
//     · 用手机录音或者示波器都容易看出来
//     · 频率固定，如果听到变调就说明时钟有问题
//   —— 这就是"控制变量法"在硬件调试里的应用。
//
// ★ 本模块的 I2S 时序是"特别定制"的，注释里写得很清楚：★
//   它不是标准 32bit-slot I2S，而是按照配套的 I2S_receiver 的胃口做的：
//     - 每个声道发送 64 个 BCLK
//     - 接收端在声道切换那一刻取 shift_reg[63:40]
//   因此这里把 24bit 样本放在每个 64bit 声道字的【最高 24 位】：
//       [63:40] = 波形数据
//       [39:0]  = 补 0
//   如果你换了别的 I2S 接收芯片，这段时序很可能需要改。
//
// 时钟关系：
//   MCLK = 12.288MHz   （输入）
//   BCLK = 6.144MHz    = MCLK / 2
//   LRCK = 48kHz       = BCLK / 128   （每声道 64 个 BCLK × 2 声道）
//
// 波形是怎么产生方波的？
//   HALF_PERIOD_SAMPLES = TONE_PERIOD_SAMPLES / 2
//     TONE_PERIOD_SAMPLES = 48000 / 1000 = 48 （一个周期 48 个采样）
//     HALF_PERIOD_SAMPLES = 24                 （半周期 24 个采样）
//   tone_cnt 从 0 数到 47；前 24 个采样输出 +AMP，后 24 个输出 -AMP，
//   于是波形频率 = 48000 / 48 = 1000Hz ✓
//   —— 这是最直观的"用计数器造方波"的写法。
//
// 如何使用 ?
//   I_mclk 接 12.288MHz 音频时钟，输出三线接接收端（或 HDMI 的音频输入）。
//   想改频率就改 TONE_HZ 参数（例化时覆盖）。
//=============================================================================

module i2s_test_tone_gen_single_pll_fix64 #(
    parameter integer SAMPLE_RATE = 48000,   // 采样率
    parameter integer SAMPLE_BITS = 24,      // 采样位宽（保留参数，实际按 24bit 处理）
    parameter integer TONE_HZ     = 1000     // 输出音调频率（Hz）
)(
    input  wire I_mclk,      // 12.288MHz 音频主时钟
    input  wire I_rst,       // 复位，高有效
    output reg  O_i2s_BCLK,  // I2S 位时钟 = MCLK/2 = 6.144MHz
    output reg  O_i2s_LRCK,  // I2S 声道时钟 = 48kHz
    output reg  O_i2s_DOUT   // I2S 串行数据
);

    // 这版不是标准 32bit-slot I2S，而是专门匹配你给的 I2S_receiver：
    //   - 每个声道发送 64 个 BCLK
    //   - 接收端在声道切换时取 shift_reg[63:40]
    // 因此这里把 24bit 音频样本放在每个 64bit 声道字的最高 24bit：
    //   [63:40] = sample[23:0]
    //   [39:0]  = 0
    //
    // 时钟关系：
    //   MCLK = 12.288MHz
    //   BCLK = 6.144MHz = MCLK / 2
    //   LRCK = 48kHz    = BCLK / 128

    localparam integer CHANNEL_BITS        = 64;   // 每个声道的位数
    localparam integer FRAME_BITS          = 128;  // 一帧（左右）总位数 = 64 × 2
    localparam integer TONE_PERIOD_SAMPLES = SAMPLE_RATE / TONE_HZ;   // 一个音周期几个采样（1000Hz -> 48）
    localparam integer HALF_PERIOD_SAMPLES = TONE_PERIOD_SAMPLES / 2; // 半个周期（-> 24）
    localparam signed [23:0] AMP           = 24'sd2000000;            // 振幅（约 1/4 满量程）

    reg        mclk_div;      // MCLK 二分频标志
    reg [6:0]  bit_cnt;       // 帧内 bit 计数（0~127，需要 7bit）
    reg [63:0] left_word;     // 左声道 64bit 发送字
    reg [63:0] right_word;    // 右声道 64bit 发送字
    reg [15:0] tone_cnt;      // 音调周期计数器（数采样个数）

    always @(posedge I_mclk or posedge I_rst) begin
        if (I_rst) begin
            mclk_div   <= 1'b0;
            O_i2s_BCLK <= 1'b0;
            O_i2s_LRCK <= 1'b0;
            O_i2s_DOUT <= AMP[23];      // 复位时输出最高位（符号位）
            bit_cnt    <= 7'd0;
            tone_cnt   <= 16'd0;
            left_word  <= {AMP, 40'd0};      // 数据放高位，低 40bit 补 0
            right_word <= {AMP, 40'd0};
        end else begin
            mclk_div <= ~mclk_div;

            // 每 1 个 I_mclk 周期翻转一次 BCLK => BCLK = I_mclk / 2
            if (mclk_div == 1'b0) begin
                // 在 BCLK 下降沿更新 LRCK 和 DOUT，供接收端在 BCLK 上升沿采样
                // （这是 I2S 的常规做法：发送端在下降沿换数据，接收端在上升沿采样，
                //   这样数据在采样时刻是稳定的，不会出现"边沿打架"的问题）
                if (O_i2s_BCLK == 1'b1) begin
                    if (bit_cnt < 7'd63) begin
                        // ---- 左声道前 63 位 ----
                        O_i2s_LRCK <= 1'b0;                       // 低 = 左声道
                        O_i2s_DOUT <= left_word[63];              // 送最高位（MSB first）
                        left_word  <= {left_word[62:0], 1'b0};    // 左移一位
                        bit_cnt    <= bit_cnt + 7'd1;
                    end else if (bit_cnt == 7'd63) begin
                        // ---- 左声道第 64 位（最后一个）----
                        O_i2s_LRCK <= 1'b1;                       // 同时切换到右声道标志
                        O_i2s_DOUT <= left_word[63];
                        left_word  <= {left_word[62:0], 1'b0};
                        bit_cnt    <= 7'd64;
                    end else if (bit_cnt < 7'd127) begin
                        // ---- 右声道前 63 位 ----
                        O_i2s_LRCK <= 1'b1;                       // 高 = 右声道
                        O_i2s_DOUT <= right_word[63];
                        right_word <= {right_word[62:0], 1'b0};
                        bit_cnt    <= bit_cnt + 7'd1;
                    end else begin
                        // ---- 右声道最后一位：一帧结束，准备下一帧 ----
                        O_i2s_LRCK <= 1'b0;                       // 回到左声道
                        O_i2s_DOUT <= right_word[63];
                        bit_cnt    <= 7'd0;                       // 位计数清零

                        // 根据 tone_cnt 决定这一帧发 +AMP 还是 -AMP
                        // 前半个周期发正的，后半个周期发负的 —— 这就是方波
                        if (tone_cnt < HALF_PERIOD_SAMPLES) begin
                            left_word  <= { AMP, 40'd0};          // 正半周
                            right_word <= { AMP, 40'd0};
                        end else begin
                            left_word  <= {-AMP, 40'd0};          // 负半周
                            right_word <= {-AMP, 40'd0};
                        end

                        // 音调周期计数 0 ~ 47 循环
                        if (tone_cnt == TONE_PERIOD_SAMPLES - 1)
                            tone_cnt <= 16'd0;
                        else
                            tone_cnt <= tone_cnt + 16'd1;
                    end
                end

                O_i2s_BCLK <= ~O_i2s_BCLK;    // BCLK 翻转（二分频）
            end
        end
    end

endmodule
