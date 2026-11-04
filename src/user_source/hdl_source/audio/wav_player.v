//=============================================================================
//  wav_player.v —— WAV 播放内核（音频应用层）
//=============================================================================
//
//  【它在整条链路里的位置】
//
//      存储读取（step 2/3：Flash / SD）
//            │ 32bit 立体声样本 {L16,R16}
//            ▼
//      ┌───────────────┐
//      │  wav_player   │  ← 本模块：节拍 + 位宽转换 + 欠载保持
//      └───────┬───────┘
//              │ audio_valid + 24bit L/R（48kHz）
//              ▼
//      audio_src_sel → audio_arc_calculate → HDMI 1.4b 核
//
//  【这一版（step 1）为什么带一个内部测试源】
//  先把"接口形状"和"节拍/位宽"验证掉，再去碰存储：
//      TEST_SRC = 1 → 内部产生 1kHz 方波样本，永远"有数据"
//                     → 上板应该听到持续 1kHz，用耳朵/示波器都能判断
//      TEST_SRC = 0 → 走 I_smp_* 样本接口，等 step 2/3 接真实 FIFO
//  这样出问题时范围很小：要么是节拍/位宽（本模块），要么是存储（后续）。
//
//  【采样节拍怎么来（分数分频）】
//  25MHz 除不出整数 48kHz（25e6/48000 = 520.83），所以用累加器：
//      每个时钟: S_acc += 48000
//      若 S_acc >= 25_000_000 → 这一拍就是一个采样点，并把多出来的部分留下
//  长期平均频率精确等于 48kHz，而且不需要额外的音频时钟。
//  这和 hdmi_audio_tone_pcm_scale 里那段是同一个套路。
//
//  【位宽：16bit WAV → 24bit HDMI】
//  HDMI 核吃 24bit 有符号样本，WAV 常见是 16bit 有符号。
//  转换就是左移 8 位（低 8 位补 0），等价于 ×256：
//      24bit 样本 = {L16[15:0], 8'h00}
//  不做这步会小 48dB，基本听不见。
//
//  【样本接口为什么用 valid/rd 而不是 FIFO 信号】
//  wav_player 不该知道上游是 Flash、SD 还是测试源。约定：
//      · I_smp_valid = 1 表示 I_smp_data 当前有效（**FWFT**：数据必须已经就绪）
//      · O_smp_rd   = 1 表示本模块在这一拍"吃掉"了这个样本
//  上游可以直接接一个 FWFT FIFO 的 (empty, dout, rd_en)，也可以接寄存器输出。
//
//  【欠载（underrun）怎么处理】
//  该出样本时上游没数据 → **保持上一个样本**（零阶保持），
//  同时把 O_underrun 置 1 并粘住（直到复位）。
//  为什么不是输出 0：突然归零会产生"啪"的爆音；保持直流反而听不出来。
//  O_underrun 是给调试用的——上板看到它置 1，说明存储侧供数不够快。
//
//  【暂停】
//  I_play = 0 时仍然按 48kHz 输出 valid，但样本恒为 0（静音）。
//  为什么不停掉 valid：HDMI 的 ACR 靠 audio_valid 统计采样节拍，
//  停掉 valid 会让接收端失去音频时钟参考。
//=============================================================================

module wav_player #(
    parameter integer       CLK_HZ       = 25_000_000,  // 本模块工作时钟（video_clk）
    parameter integer       SAMPLE_RATE  = 48_000,      // 目标采样率
    parameter integer       TEST_SRC     = 1,           // 1 = 内部 1kHz 测试样本；0 = 走 I_smp_*
    parameter integer       TEST_TONE_HZ = 1_000,       // 测试音频率
    parameter signed [15:0] TEST_AMP16   = 16'sd12000    // 测试音幅度（16bit，约 37% 满量程）
)(
    input  wire        I_clk,               // video_clk 25MHz
    input  wire        I_rst,               // 复位，高有效
    input  wire        I_play,              // 1 = 播放；0 = 静音（仍按 48kHz 送 0）

    // ---- 样本输入（TEST_SRC=0 时使用）----
    input  wire        I_smp_valid,         // 上游有样本（FWFT：数据已就绪）
    output wire        O_smp_rd,            // 本模块取走一个样本（单拍）
    input  wire [31:0] I_smp_data,          // {L16, R16}

    // ---- 输出到 HDMI ----
    output reg         O_audio_valid,       // 一个采样点有效（单拍，48kHz）
    output reg  [23:0] O_audio_left_data,   // 左声道 24bit 有符号
    output reg  [23:0] O_audio_right_data,  // 右声道 24bit 有符号
    output reg         O_underrun           // 欠载粘滞标志（1 = 曾经缺样本）
);

//-----------------------------------------------------------------------------
// 采样节拍：分数分频出 48kHz
//-----------------------------------------------------------------------------
localparam integer SR = SAMPLE_RATE;
localparam integer CK = CLK_HZ;

reg  [31:0] S_acc;                          // 采样率累加器
wire        W_tick = (S_acc + SR >= CK);    // 这一拍该出一个采样点

//-----------------------------------------------------------------------------
// 测试样本源：1kHz 方波（周期 48 个样本 @48kHz，前半 +AMP、后半 -AMP）
//   计数器只在"取走样本"时才前进，所以波形与播放节拍严格同步，
//   等价于一个永远不空的 FIFO。
//-----------------------------------------------------------------------------
localparam integer PERIOD = (SAMPLE_RATE / TEST_TONE_HZ);   // 48
localparam integer HALF   = (PERIOD / 2);                   // 24

wire        W_test_valid;
wire [31:0] W_test_data;

generate
if (TEST_SRC != 0) begin : g_test_src
    reg [15:0] S_tone_cnt;
    wire signed [15:0] W_wave = (S_tone_cnt < HALF) ? TEST_AMP16 : -TEST_AMP16;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst)
            S_tone_cnt <= 16'd0;
        else if (O_smp_rd)
            S_tone_cnt <= (S_tone_cnt == PERIOD - 1) ? 16'd0 : S_tone_cnt + 16'd1;
    end

    assign W_test_valid = 1'b1;             // 永远有数据
    assign W_test_data  = {W_wave, W_wave}; // 左右相同（单声道测试音）
end
else begin : g_no_test
    assign W_test_valid = 1'b0;
    assign W_test_data  = 32'd0;
end
endgenerate

// 源选择：测试源 / 外部样本口
wire        W_src_valid = (TEST_SRC != 0) ? W_test_valid : I_smp_valid;
wire [31:0] W_src_data  = (TEST_SRC != 0) ? W_test_data  : I_smp_data;

// 取样本：该出样本 且 在播放 且 上游有数据
assign O_smp_rd = W_tick && I_play && W_src_valid;

//-----------------------------------------------------------------------------
// 输出寄存器
//-----------------------------------------------------------------------------
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        S_acc              <= 32'd0;
        O_audio_valid      <= 1'b0;
        O_audio_left_data  <= 24'sd0;
        O_audio_right_data <= 24'sd0;
        O_underrun         <= 1'b0;
    end
    else begin
        O_audio_valid <= 1'b0;              // 默认低，只在采样点拉高一拍

        if (W_tick) begin
            S_acc         <= S_acc + SR - CK;   // 保留小数余量，长期平均精确
            O_audio_valid <= 1'b1;

            if (!I_play) begin
                // 静音：照样送 valid，但样本为 0（保 ACR 节拍）
                O_audio_left_data  <= 24'sd0;
                O_audio_right_data <= 24'sd0;
            end
            else if (W_src_valid) begin
                // 16bit → 24bit：左移 8 位
                O_audio_left_data  <= {W_src_data[31:16], 8'h00};
                O_audio_right_data <= {W_src_data[15:0],  8'h00};
            end
            else begin
                // 欠载：保持上一采样（零阶保持），避免爆音
                O_underrun <= 1'b1;
            end
        end
        else begin
            S_acc <= S_acc + SR;
        end
    end
end

endmodule
