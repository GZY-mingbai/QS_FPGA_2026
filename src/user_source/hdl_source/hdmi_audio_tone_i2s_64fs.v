//=============================================================================
// 文件名   : hdmi_audio_tone_i2s_64fs.v
// 模块名   : hdmi_audio_tone_i2s_64fs
// 功能     : 【音频正弦波发生器 + I2S 发送器（64fs 格式）】
//            自己算出一个正弦波，按 do-re-mi-fa-so-la-si-do 依次循环播放，
//            并通过 I2S 时序串行送出，最终混进 HDMI 信号里从电视/显示器发声。
//
// ★ 这是本工程"声音"部分的源头。★
//
// 一、I2S 是什么？
//   I2S（Inter-IC Sound）是音频芯片之间传数据的标准串行协议，需要三根线：
//     BCLK (Bit Clock)  —— 位时钟，每来一个沿送 1 bit
//     LRCK (Left/Right) —— 声道选择：低=左声道，高=右声道
//     DOUT (Data Out)   —— 数据线，串行送出
//   "64fs" 的意思是：每个音频采样周期用 64 个 BCLK 时钟，
//   分给左右两个声道各 32 bit（32×2 = 64）。
//
// 二、本模块的时钟关系（关键！）
//   输入 I_mclk = 12.288 MHz
//   BCLK = I_mclk / 2 = 6.144 MHz   （代码里 O_i2s_BCLK <= ~O_i2s_BCLK 实现二分频）
//   LRCK = BCLK / 64 = 96 kHz？ 不对——代码里每 64 个 BCLK 翻转一次 LRCK，
//          于是 LRCK 的一个完整周期（高+低）= 128 个 BCLK = 48 kHz ✓
//          这就是标准音频采样率 48kHz 的由来。
//
// 三、怎么产生正弦波？（初学者最感兴趣的部分）
//   用"相位累加器（Phase Accumulator）"—— 数字产生任意频率的核心技巧：
//     ① 维持一个 32bit 的累加器 S_phase_acc，每个采样点加上一个固定值 PHASE_INC；
//     ② 32bit 累加到一半就溢出一次，所以溢出频率 f = PHASE_INC / 2^32 × fs。
//     ③ 本模块更简单粗暴：直接看 S_phase_acc 的最高位（第 31 位）
//          bit31 = 1  -> 输出 +AMP
//          bit31 = 0  -> 输出 -AMP
//        这实际产生的是"方波"，听感就是电子蜂鸣器的"哔哔"声。
//        （真正要出正弦波，需要把 phase_acc 高位拿去查正弦表 ROM。）
//     ④ 想要某个音高，就算出对应的 PHASE_INC 值，这就有了 note_inc_lut 那张表。
//
//   例：440Hz 的 la（A4）：
//        PHASE_INC = f / fs × 2^32 = 440 / 48000 × 4294967296 ≈ 39370534 ✓
//        正好等于 note_inc_lut 里 3'd5 的值，可以自己验算一下，很直观。
//
// 四、旋律怎么来的？
//   S_note_idx 是当前音阶序号，每 NOTE_HOLD_FRAMES = 24000 个采样（=0.5 秒 @48kHz）
//   就 +1，到 7 之后回 0，于是自动循环播放音阶。
//
// 如何使用 ?
//   直接例化，I_mclk 接 audio PLL 产生的 12.288MHz。
//   三个输出接 HDMI IP 的音频输入（经 audio_arc_calculate 计算 CTS/N 后送进 IP）。
//=============================================================================

module hdmi_audio_tone_i2s_64fs #(
    parameter [31:0] PHASE_INC = 32'd39370534,   // 兼容旧顶层，实际本版内部不用它
    parameter signed [23:0] AMP = 24'sd2000000,  // 振幅。24bit 有符号，最大 ±8388607；这里取约 1/4 音量
    parameter integer NOTE_HOLD_FRAMES = 24000   // 每个音持续 0.5s @ 48kHz（48000×0.5 = 24000）
)(
    input  wire I_mclk,      // 12.288MHz - 音频主时钟（MCLK）
    input  wire I_rst,       // 复位，高有效
    output reg  O_i2s_BCLK,  // I2S 位时钟（MCLK/2 = 6.144MHz）
    output reg  O_i2s_LRCK,  // I2S 左右声道时钟（48kHz）
    output reg  O_i2s_DOUT   // I2S 串行数据
);

//-----------------------------------------------------------------------------
// 音阶相位增量查找表（function 写法）
// do/re/mi/fa/so/la/si/do  （C4 D4 E4 F4 G4 A4 B4 C5）
// 每个值都按 48kHz 采样率、公式 PHASE_INC = f/fs × 2^32 算出来
//
// 关于 function（Verilog 里的"函数"）：
//   可以把一段组合逻辑封装起来反复调用，必须有返回值、必须有 input。
//   这里相当于"用索引查表"，比写一大段 case 更清爽。
//   注意它是组合逻辑，不能写时序（不能有时钟沿），调用时也是立即出结果。
//-----------------------------------------------------------------------------
function [31:0] note_inc_lut;
    input [2:0] idx;                 // 音阶索引 0~7
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

reg  [5:0]  S_bit_cnt;        // 一个声道内的 bit 计数（0~63）
reg  [63:0] S_shift_reg;      // 64bit 发送移位寄存器（装左右两个声道的数据）
reg  [31:0] S_phase_acc;      // 相位累加器（决定音高）
reg signed [23:0] S_sample_word;  // 当前正在播放的采样值
reg signed [23:0] S_sample_next;  // 下一个采样值（提前算好）
reg  [2:0]  S_note_idx;       // 当前音阶（0~7）
reg [15:0]  S_note_frame_cnt; // 当前音持续了多少个采样帧

wire [31:0] W_note_inc = note_inc_lut(S_note_idx);  // 当前音对应的相位增量

//-----------------------------------------------------------------------------
// 主状态机：靠"数 BCLK"驱动整个 I2S 时序
//   每拍翻转 BCLK（=> BCLK = MCLK/2）
//   在 BCLK 上升沿时送出一位数据
//   数满 64 位就切声道（左->右，右->下一帧）
//-----------------------------------------------------------------------------
always @(posedge I_mclk or posedge I_rst) begin
    if (I_rst) begin
        O_i2s_BCLK      <= 1'b0;
        O_i2s_LRCK      <= 1'b0;
        O_i2s_DOUT      <= 1'b0;
        S_bit_cnt       <= 6'd0;
        S_shift_reg     <= 64'd0;
        S_phase_acc     <= 32'd0;
        S_sample_word   <= 24'sd0;
        S_sample_next   <= AMP;         // 初值给正振幅
        S_note_idx      <= 3'd0;        // 从 do 开始
        S_note_frame_cnt<= 16'd0;
    end
    else begin
        // 保持和你现有接收端兼容的 64fs 发送时序
        O_i2s_BCLK <= ~O_i2s_BCLK;      // 二分频：得到 6.144MHz 的 BCLK

        if (O_i2s_BCLK == 1'b1) begin   // 此刻 BCLK 即将变低，等效于"上升沿采样点"
            // ---- 移位输出：取最高位送出去，然后整体左移一位（低位补 0）----
            O_i2s_DOUT  <= S_shift_reg[63];
            S_shift_reg <= {S_shift_reg[62:0], 1'b0};

            if (S_bit_cnt == 6'd63) begin    // 一个声道 64 位发完了
                S_bit_cnt <= 6'd0;

                if (O_i2s_LRCK == 1'b0) begin
                    // 左声道发完，右声道复用同一个样本
                    O_i2s_LRCK  <= 1'b1;                              // 切到右声道
                    S_shift_reg <= {S_sample_word[23:0], 40'd0};      // 重新装填数据（高 24bit 有效，后补 0）
                end
                else begin
                    // 右声道发完，进入下一帧样本
                    O_i2s_LRCK  <= 1'b0;                              // 切回左声道

                    // ---- 相位累加，产生新的采样值 ----
                    S_phase_acc <= S_phase_acc + W_note_inc;          // 加一个增量
                    if (S_phase_acc[31])                              // 取最高位判断"半周期"
                        S_sample_next <= AMP;                         // 正半周
                    else
                        S_sample_next <= -AMP;                        // 负半周
                                                                      // （所以实际是方波，不是正弦）

                    S_sample_word <= S_sample_next;
                    S_shift_reg   <= {S_sample_next[23:0], 40'd0};    // 装入新样本准备发送

                    // ---- 音符计时：唱满 0.5 秒就换下一个音 ----
                    if (S_note_frame_cnt == NOTE_HOLD_FRAMES - 1) begin
                        S_note_frame_cnt <= 16'd0;
                        if (S_note_idx == 3'd7)     // 已经唱到最高音 do
                            S_note_idx <= 3'd0;     // 回到最低音，循环
                        else
                            S_note_idx <= S_note_idx + 3'd1;
                    end
                    else begin
                        S_note_frame_cnt <= S_note_frame_cnt + 16'd1;
                    end
                end
            end
            else begin
                S_bit_cnt <= S_bit_cnt + 6'd1;   // 声道内继续数位
            end
        end
    end
end

endmodule
