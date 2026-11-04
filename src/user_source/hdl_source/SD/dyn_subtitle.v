`timescale 1ns/1ps
//=============================================================================
// 文件名   : dyn_subtitle.v
// 模块名   : dyn_subtitle
// 功能     : 【动态字幕】在最终显示像素流上叠加一条单行、常速左移的滚动字幕
//
//  ★ 2026-11-06：文字由 sub_loader 从 SD 卡同名 TXT 解析、把 GB2312 字形写进字形 RAM。
//  ★ 2026-11-07：双缓冲 + 省 LUT 改版
//    · 字形 RAM 分两个 bank（BRAM32K 是 2048×16，原来只用了低 1024 字）：
//      显示读 front bank，sub_loader 往 back bank 装新字幕；装完首批后
//      sub_loader 翻转 commit_tgl ⇒ 本模块在下一个 vs 沿换 bank、换长度、相位归零。
//      ⇒ 换图时旧字幕一直滚，直到新字幕完全装好才被替换（没有空白/乱码）。
//    · 地址只算一份：命中标志 + 格内列号打一拍给输出级，不再在输出级重算
//      一遍加法器和两个 13 位比较器。
//    · 相位改存"相对文字起点"的 pm = phase − 640（13 位补码），文本内偏移
//      rel = x + pm 一个加法就够，省掉 −640 减法器和 ≥640 比较器。
//    · 回卷判定从"phase ≥ cycle−1"（减法+比较）改成相等判定。
//
// 它在整条链路里的位置（显示读出侧，video_clk 25MHz 域）：
//
//     频谱条 ──► ★ 本模块 ★ ──► video_postproc ──► 黑屏门控 ──► video_rgb_to_axis
//
//  ★ 2026-11-03：后处理已移到本模块之后，字幕也接受画质参数处理。
//
//------------------------------------------------------------------------------
//  【几何】16×16 点阵，字符格 16px，单行
//------------------------------------------------------------------------------
//   · 字幕顶行 TEXT_Y = 456，占 456..471（480 高，底部留 8px）；
//   · 文字宽度 = len × 16（最多 256 字 = 4096px）；len = 0 ⇒ 区间为空，不画；
//   · 屏幕像素 x 对应的文本内偏移 rel = x + pm，rel ∈ [0, len×16) 才画；
//   · pm 从 −640 起（文字整体在屏幕右侧之外），每帧 +1（60px/s）；
//   · pm 到 len×16 + 31 回卷到 −640 ⇒ 尾部 32 帧（≈0.5s）空白后再从右边进来。
//     即 cycle = 640 + len×16 + 32 帧，与 sub_loader 的帧计数一致。
//
//  【槽号】字形槽 64 个/bank，slot = 字符位置 % 64 = rel[9:4]。
//
//  【BRAM 用法】1 × EG_PHY_BRAM32K（DP16K = 2048 × 16）
//     · 地址 {bank, slot[5:0], row[3:0]}（11 位）；
//     · 端口 A：sd_card_clk 域写（sub_loader）；端口 B：video_clk 读，1 拍延迟（NOREG）。
//
//  【门控】show_en（sd 域电平：菜单没开）在 vs 沿采样 ⇒ 整帧一致。
//
//  【CDC】commit_tgl 2FF 同步后取边沿；bank/text_len 与 commit_tgl 在 sd 域同拍变化，
//     看到边沿时它们早已稳定 ⇒ 当拍锁进 nx_*，等下一个 vs 沿再生效。
//     两次 commit 之间至少隔一次 TXT 扇区读（~0.4ms）≫ 2 拍同步延迟。
//=============================================================================

module dyn_subtitle (
    input  wire        video_clk,   // 像素时钟 25MHz
    input  wire        rst,         // 复位，高有效

    input  wire        vs,          // 场同步（与 de/rgb 对齐）
    input  wire        de,          // 数据有效
    input  wire [23:0] rgb_in,      // 输入像素 {R,G,B}

    // ---- 显示门控 / 字幕参数（sd_card_clk 域）----
    input  wire        show_en,     // 1 = 允许叠加（菜单没开）
    input  wire        commit_tgl,  // 每提交一份新字幕翻转一次
    input  wire        bank,        // 新字幕所在 bank（与 commit_tgl 同拍更新）
    input  wire [8:0]  text_len,    // 新字幕字符数（0..256；0 = 无字幕）

    // ---- 字模写口（sd_card_clk 域，sub_loader 驱动）----
    input  wire        wr_clk,
    input  wire [10:0] wr_addr,     // {bank, slot[5:0], row[3:0]}
    input  wire [15:0] wr_data,
    input  wire        wr_we,

    // ---- 给 sd 域的帧脉冲（每帧翻转一次，用于重建帧计数）----
    output reg         vs_tgl,

    output wire        vs_out,      // 延迟 1 拍的场同步
    output wire        de_out,      // 延迟 1 拍的数据有效
    output wire [23:0] rgb_out      // 叠加字幕后的像素
);

//----------------------------------------------------------------------------
// 1. 几何常量
//----------------------------------------------------------------------------
localparam [9:0]  SCREEN_W   = 10'd640;
localparam [12:0] PM_START   = 13'h1D80;       // −640（13 位补码）
localparam [23:0] TEXT_COLOR = 24'hFFD700;     // 金色

//----------------------------------------------------------------------------
// 2. 参数锁存 / 帧相位（video_clk 域）
//----------------------------------------------------------------------------
reg        cm_ff1, cm_ff2, cm_ff3;
reg        show_ff1, show_ff2;
wire       cm_edge = cm_ff2 ^ cm_ff3;
reg        cm_pend;                            // 有一份新字幕等下一个 vs 生效

reg        nx_bank;                            // 待生效的 bank / 长度
reg [8:0]  nx_len;
reg        rd_bank;                            // 正在显示的 bank
reg [8:0]  len_r;                              // 正在显示的字符数
reg [8:0]  lenp1_r;                            // len_r + 1（回卷判定用）
reg        show_q;                             // 本帧是否叠加

reg [9:0]  x;                                  // 行内像素 0..639
reg [8:0]  y;                                  // 行号 0..479
reg        de_d;
reg        vs_d;
reg [12:0] pm;                                 // phase − 640

wire       pm_wrap = (pm[3:0] == 4'hF) && (pm[12:4] == lenp1_r);   // pm == len×16+31

always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        x        <= 10'd0;
        y        <= 9'd0;
        de_d     <= 1'b0;
        vs_d     <= 1'b0;
        pm       <= PM_START;
        cm_ff1   <= 1'b0;
        cm_ff2   <= 1'b0;
        cm_ff3   <= 1'b0;
        show_ff1 <= 1'b0;
        show_ff2 <= 1'b0;
        cm_pend  <= 1'b0;
        nx_bank  <= 1'b0;
        nx_len   <= 9'd0;
        rd_bank  <= 1'b0;
        len_r    <= 9'd0;
        lenp1_r  <= 9'd1;
        show_q   <= 1'b0;
        vs_tgl   <= 1'b0;
    end else begin
        cm_ff1   <= commit_tgl;
        cm_ff2   <= cm_ff1;
        cm_ff3   <= cm_ff2;
        show_ff1 <= show_en;
        show_ff2 <= show_ff1;

        // ---- 新字幕提交：参数先锁进 nx_*，等 vs 沿生效 ----
        if (cm_edge) begin
            nx_bank <= bank;
            nx_len  <= text_len;
            cm_pend <= 1'b1;
        end

        // ---- 行内计数：de 期间 0..639，消隐归 0 ----
        de_d <= de;
        if (de) x <= (x == (SCREEN_W - 10'd1)) ? 10'd0 : (x + 10'd1);
        else    x <= 10'd0;

        // ---- 场边界 ----
        vs_d <= vs;
        if (vs && !vs_d) begin
            y      <= 9'd0;
            vs_tgl <= ~vs_tgl;
            show_q <= show_ff2;
            if (cm_pend && !cm_edge) begin
                // 换 bank / 长度，文字从右边缘重新进来
                rd_bank <= nx_bank;
                len_r   <= nx_len;
                lenp1_r <= nx_len + 9'd1;
                pm      <= PM_START;
                cm_pend <= 1'b0;
            end else begin
                // ★ 相位无条件推进（隐藏期间也走）：sd 域的帧计数靠 vs_tgl 数出来，
                //   两边必须同一节奏，否则懒加载的调度点会漂。
                pm <= pm_wrap ? PM_START : (pm + 13'd1);
            end
        end else if (!de && de_d) begin
            // ---- 行末：y 推进（场沿会归零，不需要 479 回卷）----
            y <= y + 9'd1;
        end
    end
end

//----------------------------------------------------------------------------
// 3. 字模读地址 + 命中判定（只算一份）
//----------------------------------------------------------------------------
//   字幕带 456..471：y[8:3] ∈ {57, 58}；glyph_row = y[3:0] ^ 8 ⇒ 0..15
wire [12:0] rel       = {3'b0, x} + pm;                       // 文本内像素偏移
wire        in_text   = ~rel[12] && ({1'b0, rel[11:4]} < len_r);
wire        in_text_y = (y[8:3] == 6'd57) || (y[8:3] == 6'd58);
wire [3:0]  glyph_row = y[3:0] ^ 4'b1000;
wire [10:0] rom_addr  = {rd_bank, rel[9:4], glyph_row};       // 不命中时读到什么都无所谓

//----------------------------------------------------------------------------
// 4. 字模 RAM（双口：A 写（sd 域）/ B 读（video 域），读口 1 拍延迟）
//----------------------------------------------------------------------------
wire [15:0] rom_q;

`ifdef IVERILOG_SIM
reg [15:0] glyph_mem [0:2047];
reg [15:0] rom_q_r;
integer gi;
initial begin
    for (gi = 0; gi < 2048; gi = gi + 1) glyph_mem[gi] = 16'd0;
end
always @(posedge wr_clk) begin
    if (wr_we) glyph_mem[wr_addr] <= wr_data;
end
always @(posedge video_clk) rom_q_r <= glyph_mem[rom_addr];
assign rom_q = rom_q_r;
`else
// ---- 综合：1 × EG_PHY_BRAM32K（DP16K = 2048 × 16，真双口）----
//   ★ 没有 INIT_xx：内容全部由 sub_loader 在运行期写入。
EG_PHY_BRAM32K #(
    .MODE("DP16K"), .DATA_WIDTH_A("16"), .DATA_WIDTH_B("16"),
    .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
    .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
    .SRMODE("SYNC"), .CSAMUX("SIG"), .CSBMUX("SIG"),
    .OCEAMUX("SIG"), .OCEBMUX("SIG"), .RSTAMUX("SIG"), .RSTBMUX("SIG")
) u_glyph_ram (
    .clka(wr_clk), .csa(1'b1), .wea(wr_we), .addra(wr_addr), .dia(wr_data),
    .bytea(1'b0), .bytewea(1'b0), .rsta(1'b0), .ocea(1'b1),
    .clkb(video_clk), .csb(1'b1), .web(1'b0), .addrb(rom_addr),
    .dib(16'd0), .byteb(1'b0), .byteweb(1'b0), .rstb(1'b0), .oceb(1'b1),
    .dob(rom_q)
);
`endif

//----------------------------------------------------------------------------
// 5. 输出侧像素流水（与 BRAM 同 1 拍）：只打命中位 + 格内列号
//----------------------------------------------------------------------------
reg        hit_q;
reg [3:0]  cell_q;
reg [23:0] rgb_q;
reg        de_q;
reg        vs_q;

always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        hit_q  <= 1'b0;
        cell_q <= 4'd0;
        rgb_q  <= 24'd0;
        de_q   <= 1'b0;
        vs_q   <= 1'b0;
    end else begin
        hit_q  <= show_q && in_text && in_text_y;
        cell_q <= rel[3:0];
        rgb_q  <= rgb_in;
        de_q   <= de;
        vs_q   <= vs;
    end
end

//----------------------------------------------------------------------------
// 6. 字模取位 + 叠加：第 cell 列（0=最左）对应字模 bit 15−cell = bit ~cell
//----------------------------------------------------------------------------
wire glyph_on = hit_q && rom_q[~cell_q];

assign rgb_out = glyph_on ? TEXT_COLOR : rgb_q;
assign de_out  = de_q;
assign vs_out  = vs_q;

endmodule
