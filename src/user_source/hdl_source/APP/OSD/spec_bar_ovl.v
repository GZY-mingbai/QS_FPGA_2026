//=============================================================================
//  spec_bar_ovl.v —— 频谱条叠加（帧缓存之后、video_clk 域、像素通路纯组合）
//=============================================================================
//
//  方案见 FFT_Plan.md §3 / §5.6。★ 2026-11-03：插在 `vout_data_raw` 与
//  动态字幕/video_postproc 之间（后处理已移到叠加之后）：
//      · hs/vs/de **不经过本模块**，零延迟 ⇒ 不可能让画面错位；
//      · out_rgb = on ? 条色 : in_rgb，组合二选一。
//
//  【几何】16 条 × 32px（条宽 28px + 左右各 2px 间隙），x 64..575；
//          条区 y 352..479（底部 128 行），条高 0..127。
//
//  【计数器零减法】
//      xc 在行首置 960（= −64 mod 1024）⇒ x = 64..575 时 xc = 0..511：
//          区内 = ~xc[9]，条号 = xc[8:5]，条内列 = xc[4:0]
//      yc 在 vs 跳变时置 32 ⇒ y = 352..479 时 yc = 384..511：
//          区内 = ~yc[9] & yc[8] & yc[7]，区内行 r = yc[6:0]（0 = 顶）
//      亮点条件  r ≥ 128 − h  ⇔  ~r < h      （7 位比较，取反被 LUT 吸收）
//
//  【条高读取 = "预取"，不加像素延迟】
//      条高在 spec_fft 的工作 RAM 里（同步读，晚 1 拍）。读地址用 xc 的
//      **下一拍值** xc_nx[8:5] ⇒ 数据回来时正好对上当前像素，
//      比较 + 选色全是组合逻辑，de 不需要打拍。
//
//  【vs 极性无关】只用 vs 的"跳变"复位 yc（同 rgb_to_axis 的做法），
//      两次跳变都落在场消隐里。
//
//  【vbl】最后一个有效行（yc = 511）的 de 下降沿 ⇒ 单拍，告诉 spec_fft
//      可以开始 FFT（之后一直到下一帧首行都是消隐，≥ 11k 拍）。
//
//  【开关】en = sync2(osd_spc) & ~sync2(menu_on)，且只在 vbl 时采样
//      ⇒ 开/关、菜单开/合都从整帧边界生效，不会出现半帧条。
//      菜单面板覆盖 y 64..450 全宽，与条区重叠，所以菜单打开时隐藏。
//      信息条 / 缩放 OSD 在左上角，不冲突，不需要门控。
//=============================================================================
module spec_bar_ovl #(
    parameter [7:0] COL_G = 8'hE0,          // 条色：R 随高度渐变（底绿→顶黄）
    parameter [7:0] COL_B = 8'h30
)(
    input  wire        clk,                 // video_clk 25MHz
    input  wire        rst,                 // 高有效

    input  wire        in_vs,               // 与 vout_data_raw 对齐的 vs
    input  wire        in_de,               // 与 vout_data_raw 对齐的 de
    input  wire [23:0] in_rgb,              // vout_data_raw（后处理/门控之前）

    input  wire        spc_a,               // osd_spc（sd_card_clk 域，电平）
    input  wire        menu_a,              // osd_on（sd_card_clk 域，电平）

    output wire [3:0]  bar_idx,             // → spec_fft.scan_idx
    input  wire [6:0]  bar_h,               // ← spec_fft.scan_h（晚 1 拍）
    output reg         vbl,                 // → spec_fft.vbl

    output wire [23:0] out_rgb
);

reg  [9:0] xc;
reg  [9:0] yc;
reg        de_d;
reg        vs_d;
reg  [1:0] s_spc;
reg  [1:0] s_menu;
reg        en_f;

wire [9:0] xc_nx = in_de ? (xc + 10'd1) : 10'd960;
assign bar_idx = xc_nx[8:5];

wire de_fall = de_d & ~in_de;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        xc     <= 10'd960;
        yc     <= 10'd0;
        de_d   <= 1'b0;
        vs_d   <= 1'b0;
        vbl    <= 1'b0;
        s_spc  <= 2'b00;
        s_menu <= 2'b00;
        en_f   <= 1'b0;
    end else begin
        xc   <= xc_nx;
        de_d <= in_de;
        vs_d <= in_vs;

        if (vs_d != in_vs) yc <= 10'd32;
        else if (de_fall)  yc <= yc + 10'd1;

        vbl <= de_fall & (yc == 10'd511);

        s_spc  <= {s_spc[0],  spc_a};
        s_menu <= {s_menu[0], menu_a};
        if (vbl) en_f <= s_spc[1] & ~s_menu[1];
    end
end

//-----------------------------------------------------------------------------
//  像素通路（纯组合）
//-----------------------------------------------------------------------------
wire       in_rgn = ~xc[9] & ~yc[9] & yc[8] & yc[7];
wire       in_col = (xc[4:1] != 4'd0) & (xc[4:1] != 4'd15);   // 2px 间隙
wire       lit    = (~yc[6:0]) < bar_h;
wire       on     = en_f & in_rgn & in_col & lit;

wire [7:0] col_r  = {~yc[6:0], 1'b1};      // 区顶 255（黄）→ 区底 1（绿）

assign out_rgb = on ? {col_r, COL_G, COL_B} : in_rgb;

endmodule
