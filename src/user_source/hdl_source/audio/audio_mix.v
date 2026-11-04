//=============================================================================
//  audio_mix.v —— 音乐 + 提示音的 24bit 饱和叠加
//=============================================================================
//
//  【为什么不是"直接相加"】两路各自可能接近满量程，和会溢出 24 位；
//  溢出回绕会把"响"变成"炸"（正峰突然变负峰）。所以按符号位判断饱和：
//      s = a + b（25 位）
//      s[24] != s[23] ⇒ 同号溢出 ⇒ 钳到 {s[24], 23{...}}（±满量程）
//  纯加法器 + 几个门，约 30 LUT/声道，0 DSP。
//
//  【提示音位宽】16bit → 24bit 左移 8 位（与 wav_player 的 16→24 一致）。
//
//=============================================================================
module audio_mix (
    input  wire        clk,
    input  wire        rst,
    input  wire        valid,               // 48kHz 节拍（来自音源）

    input  wire [23:0] mus_l,
    input  wire [23:0] mus_r,
    input  wire signed [15:0] bep,          // 提示音样本（0 = 无）
    input  wire        bep_act,             // 1 = 提示音正在出声

    output reg  [23:0] out_l,
    output reg  [23:0] out_r,
    output reg         o_valid
);

wire signed [23:0] b24 = {bep, 8'h00};

// 和（25 位有符号）
wire signed [24:0] sl = {mus_l[23], mus_l} + {b24[23], b24};
wire signed [24:0] sr = {mus_r[23], mus_r} + {b24[23], b24};

wire signed [23:0] sat_l = (sl[24] != sl[23]) ? {sl[24], {23{sl[23]}}} : sl[23:0];
wire signed [23:0] sat_r = (sr[24] != sr[23]) ? {sr[24], {23{sr[23]}}} : sr[23:0];

always @(posedge clk or posedge rst) begin
    if (rst) begin
        out_l   <= 24'sd0;
        out_r   <= 24'sd0;
        o_valid <= 1'b0;
    end else begin
        o_valid <= valid;
        if (valid) begin
            // 提示音不在出声时直通（省得给它加一层无意义的加法延时）
            out_l <= bep_act ? sat_l : mus_l;
            out_r <= bep_act ? sat_r : mus_r;
        end
    end
end

endmodule
