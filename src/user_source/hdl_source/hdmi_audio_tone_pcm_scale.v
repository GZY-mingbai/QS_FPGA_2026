//=============================================================================
// 文件名   : hdmi_audio_tone_pcm_scale.v
// 模块名   : hdmi_audio_tone_pcm_scale
// 功能     : 【PCM 音频样本发生器】—— 直接产生 48kHz 的左右声道 PCM 采样值
//            和 hdmi_audio_tone_i2s_64fs.v 是"同一首歌的两个唱法"：
//              · i2s 版：自己产生 BCLK/LRCK 时钟，输出串行 I2S 位流；
//              · pcm 版：直接输出"并行 24bit 采样值 + valid 脉冲"，
//                        由后面的 HDMI 音频编码器负责打包。
//            HDMI IP 的音频输入通常就是 PCM 并行接口，所以本模块更"贴近"IP。
//
// 一、两版的关键区别：采样率是怎么来的？
//   i2s 版靠"数固定的 BCLK 个数"得到 48kHz，前提是 MCLK 恰好是 12.288MHz。
//   pcm 版靠"分数分频"：输入时钟是 25MHz（像素时钟），
//   25MHz 除不出整数的 48kHz（25e6/48000 = 520.83，不是整数），
//   所以用"累加器溢出法"：
//       每个时钟： S_sample_acc += 48000
//       如果 >= 25_000_000 就溢出，此时才算一个采样点，并把多余的部分减掉。
//   —— 这其实就是"DDS/分数分频"的标准写法，长期平均频率精确等于 48kHz，
//      而且不用换时钟，非常实用。初学者一定要掌握这个技巧。
//
// 二、音高是怎么算的？
//   和 i2s 版完全一样：相位累加器取最高位当方波输出。
//       S_phase_acc += W_note_inc
//       bit31 = 1 -> +AMP，否则 -AMP
//   音阶表换算公式： PHASE_INC = f / fs × 2^32
//   例如 la 440Hz: 440/48000 × 2^32 ≈ 39,370,534（表里 3'd5 的值）✓
//
// 三、音符时长
//   NOTE_HOLD_SAMPLES = 24000 个采样 @48kHz = 0.5 秒，和 i2s 版保持一致。
//
// 如何使用 ?
//   I_clk 接像素时钟（25MHz），输出 O_audio_valid / O_audio_left_data / O_audio_right_data
//   接 HDMI IP 的音频输入。想换旋律就改 note_inc_lut 表。
//=============================================================================

module hdmi_audio_tone_pcm_scale #(
    parameter integer CLK_FREQ_HZ       = 25_000_000, // 输入时钟频率（Hz）
    parameter integer SAMPLE_RATE_HZ    = 48_000,     // 目标音频采样率（Hz）
    parameter signed [23:0] AMP         = 24'sd2000000, // 振幅（24bit 有符号）
    parameter integer NOTE_HOLD_SAMPLES = 24_000      // 每个音持续多少个采样（24000 = 0.5s）
)(
    input  wire        I_clk,              // 输入时钟（本工程接 25MHz 像素时钟）
    input  wire        I_rst,              // 复位，高有效
    output reg         O_audio_valid,      // 采样有效脉冲（48kHz 速率）
    output reg  [23:0] O_audio_left_data,  // 左声道采样值
    output reg  [23:0] O_audio_right_data  // 右声道采样值
);

//-----------------------------------------------------------------------------
// 音阶相位增量查找表（与 i2s 版完全相同，注释见那一个文件）
// PHASE_INC = f / fs × 2^32 ，fs = 48kHz
//-----------------------------------------------------------------------------
function [31:0] note_inc_lut;
    input [2:0] idx;
    begin
        case (idx)
            3'd0: note_inc_lut = 32'd23409862; // do  C4 261.63Hz
            3'd1: note_inc_lut = 32'd26276681; // re  D4 293.66Hz
            3'd2: note_inc_lut = 32'd29494578; // mi  E4 329.63Hz
            3'd3: note_inc_lut = 32'd31248410; // fa  F4 349.23Hz
            3'd4: note_inc_lut = 32'd35075155; // so  G4 392.00Hz
            3'd5: note_inc_lut = 32'd39370534; // la  A4 440.00Hz
            3'd6: note_inc_lut = 32'd44191930; // si  B4 493.88Hz
            default: note_inc_lut = 32'd46819716; // do  C5 523.25Hz
        endcase
    end
endfunction

reg [31:0] S_sample_acc;        // 采样率累加器（用于分数分频出 48kHz）
reg [31:0] S_phase_acc;         // 相位累加器（决定音高）
reg [2:0]  S_note_idx;          // 当前音阶 0~7
reg [31:0] S_note_sample_cnt;   // 当前音已持续的采样数
reg signed [23:0] S_pcm_sample; // 当前 PCM 采样值
reg [31:0] S_acc_add;           // 累加中间结果

wire [31:0] W_note_inc = note_inc_lut(S_note_idx);   // 当前音的相位增量

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        O_audio_valid      <= 1'b0;
        O_audio_left_data  <= 24'd0;
        O_audio_right_data <= 24'd0;
        S_sample_acc       <= 32'd0;
        S_phase_acc        <= 32'd0;
        S_note_idx         <= 3'd0;
        S_note_sample_cnt  <= 32'd0;
        S_pcm_sample       <= 24'sd0;
        S_acc_add          <= 32'd0;
    end else begin
        O_audio_valid <= 1'b0;      // 默认无效，只有采样时刻才拉高
        S_acc_add = S_sample_acc + SAMPLE_RATE_HZ;   // 注意：用 "=" 而非 "<="
                                                     // （这里当组合逻辑用，立即得到结果）

        // 分数分频产生 48kHz 采样使能
        // 原理：每个时钟累加 48000，累到超过 25_000_000 就说明"该采下一个点了"
        if (S_acc_add >= CLK_FREQ_HZ) begin
            S_sample_acc  <= S_acc_add - CLK_FREQ_HZ;  // 把超出的部分留到下一轮（保证长期精度）
            O_audio_valid <= 1'b1;                     // 打出"采样有效"

            // ---- 计算下一个采样点 ----
            S_phase_acc <= S_phase_acc + W_note_inc;   // 相位累加
            if (S_phase_acc[31])                       // 看最高位
                S_pcm_sample <= AMP;                   // 正半周
            else
                S_pcm_sample <= -AMP;                  // 负半周

            O_audio_left_data  <= S_pcm_sample;        // 左右声道相同（单声道播放）
            O_audio_right_data <= S_pcm_sample;

            // ---- 音符切换：唱满 0.5 秒换下一个 ----
            if (S_note_sample_cnt == NOTE_HOLD_SAMPLES - 1) begin
                S_note_sample_cnt <= 32'd0;
                if (S_note_idx == 3'd7)
                    S_note_idx <= 3'd0;                // 到最高音后回最低音
                else
                    S_note_idx <= S_note_idx + 3'd1;
            end else begin
                S_note_sample_cnt <= S_note_sample_cnt + 32'd1;
            end
        end else begin
            S_sample_acc <= S_acc_add;   // 没到采样点，只累加
        end
    end
end

endmodule
