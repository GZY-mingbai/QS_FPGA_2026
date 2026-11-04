//==============================================================================
//  als_bright.v —— 手动亮度 / 环境光自动亮度 二选一（sd_card_clk = 50MHz 域）
//==============================================================================
//
//  【输入】
//    auto        : OSD 显示页"自动亮度"CheckBox（本域电平）
//    pic_bright  : OSD 手动亮度滑条（本域）
//    pic_tgl     : OSD 画质参数提交翻转（本域；亮度以外的参数变化也靠它）
//    als_*_a     : als_adc 的输出（audio_mclk 域）——这里做跨域
//
//  【输出】pic_bright_eff / pic_tgl_eff → video_postproc（替代原 pic_bright / pic_tgl）
//    eff = (auto & ok) ? ALS 目标 : pic_bright
//    tgl_eff 在以下任一情况翻转一次：
//      · pic_tgl 变了（任何画质参数改动，原样转发）；
//      · 正在用 ALS 且 ALS 目标更新；
//      · "是否使用 ALS"（auto & ok）本身变了（切换来源）。
//    多个事件同一拍到达时只翻一次（用"有事件"而不是逐个异或，避免互相抵消）。
//
//  【跨域】ok：2FF；tgt_tgl：3FF + 边沿检测，边沿那拍采 tgt（tgt 在 tgl 翻转前
//    就已稳定，且 ≥16ms 才可能再变 ⇒ 多比特准静态采样安全）。
//==============================================================================
module als_bright (
    input  wire       clk,
    input  wire       rst,
    input  wire       auto,
    input  wire [6:0] pic_bright,
    input  wire       pic_tgl,
    input  wire       als_ok_a,
    input  wire [6:0] als_tgt_a,
    input  wire       als_tgl_a,
    output wire [6:0] pic_bright_eff,
    output reg        pic_tgl_eff
);

reg [1:0] ok_s;
reg [2:0] tgl_s;
reg [6:0] tgt_r;
reg       pic_tgl_d;
reg       use_d;

wire ok_v    = ok_s[1];
wire tgt_new = tgl_s[2] ^ tgl_s[1];
wire use_als = auto & ok_v;

assign pic_bright_eff = use_als ? tgt_r : pic_bright;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        ok_s        <= 2'b00;
        tgl_s       <= 3'b000;
        tgt_r       <= 7'd50;
        pic_tgl_d   <= 1'b0;
        use_d       <= 1'b0;
        pic_tgl_eff <= 1'b0;
    end else begin
        ok_s      <= {ok_s[0], als_ok_a};
        tgl_s     <= {tgl_s[1:0], als_tgl_a};
        pic_tgl_d <= pic_tgl;
        use_d     <= use_als;
        if (tgt_new) tgt_r <= als_tgt_a;
        // tgt_r 在 tgt_new 那拍才更新 ⇒ 用 tgt_new 的下一拍（use & 已更新）也行，
        // 但 postproc 自己还要 3 拍同步 + 等 vs 边沿才采参数，这里同拍翻转足够。
        if ((pic_tgl ^ pic_tgl_d) | (use_als & tgt_new) | (use_als ^ use_d))
            pic_tgl_eff <= ~pic_tgl_eff;
    end
end

endmodule
