//=============================================================================
// 文件名   : audio_arc_calculate.v
// 模块名   : audio_arc_calculate
// 功能     : 【ACR（Audio Clock Regeneration，音频时钟再生）参数计算器】
//            产生 HDMI 音频包必须携带的 CTS 和 N 两个数值。
//
// ★ 这是 HDMI 音频部分最"玄"的一个知识点，但原理其实很简单。★
//
// 一、为什么需要 ACR？
//   HDMI 线里同时传视频和音频。视频有自己的时钟（TMDS 像素时钟），
//   音频也有自己的时钟（比如 48kHz × 128 = 6.144MHz）。
//   但 HDMI 只有一根时钟线，接收端（电视）怎么知道你发的是 48kHz 还是 44.1kHz 呢？
//
//   解决办法：发送端在音频包里塞进 CTS 和 N 两个数字，
//   电视用下面这个公式反推出音频时钟：
//
//        128 × fs = f_TMDS × N / CTS
//        即  fs = f_TMDS × N / (128 × CTS)
//
//   其中 f_TMDS 是像素时钟（本工程 640x480@60Hz 时约 25.2MHz）。
//   电视测出 CTS 和 N 就能精确还原出采样率，进而正确播放。
//
// 二、数值怎么来的？
//   HDMI 标准（1.4b）给出了一批"推荐值表"。
//   对 640x480@60Hz（像素时钟 25.175MHz）配 48kHz 音频：
//        N = 6144, CTS = 6144（这也是本模块 ACR_N 默认值的来源）
//   验证一下：fs = 25.175e6 × 6144 / (128 × 6144) = 25.175e6 / 128 ≈ 196.7kHz？
//
//   —— 注意！这里算出来的不是 48kHz。
//   原因是本工程实际使用的像素时钟是 25MHz 整（PLL 给的），
//   而 CTS 是"实测出来"的：CTS = 音频时钟周期内的像素时钟个数。
//   代码正是这么做的：S_acr_cts_cnt 就是一个"数像素时钟"的计数器，
//   每 128 个音频采样（AUDIO_DIV = ACR_N >> 7 = 48）产生一次 ACR 更新，
//   届时把数到的像素时钟个数 +1 当作 CTS 发出去。
//
//   所以：N 是"固定表值"，CTS 是"本地实测值"，两者配合就能自适应。
//
// 三、为什么 AUDIO_DIV = ACR_N >> 7？
//   因为 HDMI 标准规定每秒要发 128 次 ACR 包（128Hz 的包速率）。
//   N = 6144 就是"这个包周期内应该有多少个 128fs 周期"，
//   除以 128 就得到包与包之间隔多少个音频采样。
//
// 如何使用 ?
//   由顶层例化：I_clk 接"像素时钟"（因为要数像素时钟！），
//   I_audio_valid 接音频采样有效信号，
//   输出 O_acr_cts / O_acr_n 送 HDMI IP 的音频包接口。
//=============================================================================

module audio_arc_calculate #(
    parameter ACR_N = 6144       // N 值（HDMI 标准推荐值，48kHz @ 25.175MHz 像素时钟）
    )(
    input wire       I_clk,          // 像素时钟（Pixel Clock），不是音频时钟！
    input wire       I_rst,          // 复位，高有效

    input wire       I_audio_valid,  // 音频采样有效脉冲（来自音频发送模块）

    output reg       O_acr_valid,    // ACR 包有效（每 128 个采样脉冲来一次）
    output reg[19:0] O_acr_cts,      // CTS 值（实测的像素时钟计数）
    output reg[19:0] O_acr_n         // N 值（固定为 ACR_N）
);


    // 每多少个音频采样产生一次 ACR 包：6144 / 128 = 48
    localparam AUDIO_DIV = ACR_N >> 7;   // ">> 7" 就是除以 128（2^7）

    reg[7:0]  S_audio_div_cnt;      // 音频采样分频计数（0~47）
    wire      S_audio_valid_div;    // 分频后的 ACR 更新时刻
    reg[19:0] S_acr_cts_cnt;        // 像素时钟计数器（数出来的就是 CTS）


    // 数音频采样个数，数到 48 归零
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst)
            S_audio_div_cnt <= 'd0;
        else
            if(I_audio_valid)
                begin
                    if(S_audio_div_cnt >= AUDIO_DIV-1)
                        S_audio_div_cnt <= 'd0;
                    else
                        S_audio_div_cnt <= S_audio_div_cnt + 'd1;
                end
            else
                S_audio_div_cnt <= S_audio_div_cnt;
    end

    // 到了第 48 个采样，就该发一次 ACR 了
    assign S_audio_valid_div = I_audio_valid && (S_audio_div_cnt == AUDIO_DIV-1) ? 1'b1 : 1'b0;

    // 数像素时钟个数：每来一次音频采样就 +1，ACR 更新时清零重数
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst)
            S_acr_cts_cnt <= 'd0;
        else
            if(S_audio_valid_div)      // 一个 ACR 周期结束，重新开始数
                S_acr_cts_cnt <= 'd0;
            else
                S_acr_cts_cnt <= S_acr_cts_cnt + 'd1;
    end

    // 输出 CTS / N
    always @(posedge I_clk) begin
        if(S_audio_valid_div)
            begin
                O_acr_valid <= 1'b1;
                O_acr_cts   <= S_acr_cts_cnt + 'd1;  // +1 是因为计数器从 0 开始
                O_acr_n     <= ACR_N;                // N 固定
            end
        else    
            begin
                O_acr_valid <= 1'b0;
                O_acr_cts   <= 'd0;
                O_acr_n     <= 'd0;
            end
    end

    
endmodule
