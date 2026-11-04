//==============================================================================
//  tb_scale_center.v —— 320×240 缩略图逐步放大到 640×480 时，内容中心漂移量测
//==============================================================================
//
//  【要回答的问题】用户报"320×240 切到 HD 的过程中，图片有微量左右位移"。
//  这里把"缩放映射"单独隔离出来量：
//    · 纹理只有一行（IMG_H=1），中间一个白色源像素（word 80 的高半字）；
//    · 框宽从 320 逐级加到 640，框中心固定在屏幕 320；
//    · 每一级跑几帧，取输出的亮度加权中心 x_c；
//  如果 x_c 随框宽变化超过 ±1px，就说明 `map_src` 的取整相位在"抖动"。
//
//  【口径】源 320 像素 = 160 字（RGB565 打包）。KX = 65536*160/RW，
//    与 viewer_fx 完全一致。白色标记在 word 80 的高半字 = 源像素 161。
//    理想屏幕位置 = 320（框中心）；允许 ±1px 的整数取整误差。
//
//  跑法（工作目录 = APP/）：
//    iverilog -g2012 -s tb_scale_center -o /tmp/tb_sc.vvp \
//        tb_scale_center.v img_layer_ctrl.v ps_gen_image.v img_afifo.v
//    vvp /tmp/tb_sc.vvp
//==============================================================================
`timescale 1ns / 1ps

module tb_scale_center;

localparam integer IMG_W = 640;
localparam integer IMG_H = 1;          // 只扫一行：横向映射与行数无关
localparam integer SRCW  = 160;
localparam integer SRCH  = 240;
localparam integer NPW   = SRCW * SRCH;
localparam [20:0] TEX_BASE = 21'd614400;

reg clk = 0, mclk = 0;
always #10 clk  = ~clk;                // 50MHz
always #4  mclk = ~mclk;               // 125MHz

reg rst = 1;

//------------------------------------------------------------------------------
//  纹理：第 0 行 word 80 = {白高半字, 黑低半字}，其余全黑
//------------------------------------------------------------------------------
reg [31:0] tex [0:NPW-1];
integer ti;
initial begin
    for (ti = 0; ti < NPW; ti = ti + 1) tex[ti] = 32'd0;
    // 两个关于源中心(160)对称的标记：源像素 40 与 280（word 20 / 140 的低半字）
    tex[20]  = 32'h0000_FFFF;          // 源像素 40  = 纯白
    tex[140] = 32'h0000_FFFF;          // 源像素 280 = 纯白
end

//------------------------------------------------------------------------------
//  配置 / 扫描
//------------------------------------------------------------------------------
reg [31:0] cfg_pos;
reg [31:0] cfg_size;
reg [39:0] cfg_k;
reg        layer_en = 1;

reg [15:0] scan_x = 0, scan_y = 0;
reg        ps_start = 0;
wire       layer_ready;
wire       adv = ps_start & layer_ready;
wire       pipe_ce = layer_ready;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        scan_x <= 0; scan_y <= 0;
    end else if (!ps_start) begin
        scan_x <= 0; scan_y <= 0;
    end else if (adv) begin
        if (scan_x == IMG_W - 1) scan_x <= 0;
        else                     scan_x <= scan_x + 16'd1;
    end
end

//------------------------------------------------------------------------------
//  DUT
//------------------------------------------------------------------------------
wire        req_vld;
wire [20:0] req_addr;
wire [3:0]  req_mask;
wire        req_ack;
reg         pix_wr;
reg  [31:0] pix_data;
wire [31:0] pix;
wire        out_valid;
wire [4:0]  dbg_delta;
wire [3:0]  dbg_evt;
wire [127:0] smp_bus;
wire        smp_win, pix_empty_o, in_box_o;

img_layer_ctrl #(
    .IMG_W    (IMG_W),
    .IMG_H    (IMG_H),
    .SRC_W    (SRCW),
    .LEAD     (384),
    .AFIFO_AW (10)
) dut (
    .clk         (clk),
    .rst         (rst),
    .pipe_ce     (pipe_ce),
    .adv         (adv),
    .frame_active(ps_start),
    .x           (scan_x),
    .y           (scan_y),
    .cfg_pos     (cfg_pos),
    .cfg_size    (cfg_size),
    .cfg_k       (cfg_k),
    .tex_base    (TEX_BASE),
    .layer_en    (layer_en),
    .pix         (pix),
    .out_valid   (out_valid),
    .layer_ready (layer_ready),
    .in_box_o    (in_box_o),
    .dbg_delta   (dbg_delta),
    .dbg_evt     (dbg_evt),
    .smp_bus     (smp_bus),
    .smp_win     (smp_win),
    .pix_empty_o (pix_empty_o),
    .no_underrun_fix(1'b0),
    .exp_mode    (4'd0),
    .mclk        (mclk),
    .req_vld     (req_vld),
    .req_addr    (req_addr),
    .req_mask    (req_mask),
    .req_ack     (req_ack),
    .pix_wr      (pix_wr),
    .pix_data    (pix_data)
);

//------------------------------------------------------------------------------
//  假 SDRAM：按掩码逐字回
//------------------------------------------------------------------------------
localparam integer QD = 32;
reg [20:0] qa [0:QD-1];
reg [3:0]  qm [0:QD-1];
integer    qw = 0, qr = 0;
reg        emit;
reg [1:0]  kk;
reg [20:0] cur_ad;
reg [3:0]  cur_m;
integer    gm;

assign req_ack = req_vld && ((qw - qr) < (QD - 4));

always @(posedge mclk or posedge rst) begin
    if (rst) begin
        qw <= 0; qr <= 0; emit <= 0; kk <= 0; cur_ad <= 0; cur_m <= 0;
        pix_wr <= 0; pix_data <= 0;
    end else begin
        pix_wr <= 0;
        if (req_vld && req_ack) begin
            qa[qw % QD] <= req_addr;
            qm[qw % QD] <= req_mask;
            qw <= qw + 1;
        end
        if (!emit) begin
            if (qw != qr) begin
                cur_ad <= qa[qr % QD];
                cur_m  <= qm[qr % QD];
                qr     <= qr + 1;
                kk     <= 0;
                emit   <= 1;
            end
        end else begin
            if (cur_m[kk]) begin
                pix_wr   <= 1;
                gm        = cur_ad - TEX_BASE + kk;
                pix_data <= (gm >= 0 && gm < NPW) ? tex[gm] : 32'd0;
            end
            if (kk == 3) emit <= 0;
            else         kk   <= kk + 1;
        end
    end
end

//------------------------------------------------------------------------------
//  一帧的扫描：收集 out_valid 的像素（顺序 = x 递增）
//------------------------------------------------------------------------------
reg [23:0] rowpix [0:2047];
integer    rn;
reg        cap_en = 0;
reg        cap_clr = 0;
always @(posedge clk or posedge rst) begin
    if (rst) rn <= 0;
    else if (cap_clr) rn <= 0;
    else if (cap_en && out_valid && (rn < 2048)) begin
        rowpix[rn] <= pix[23:0];
        rn <= rn + 1;
    end
end

//------------------------------------------------------------------------------
//  跑一帧：cfg 在 ps_start 上升沿被采样
//------------------------------------------------------------------------------
task run_frame;
    input integer wx;
    input integer wy;
    input integer px0;
    input integer py0;
    input integer kx;
    input integer ky;
    integer n;
    begin
        @(negedge clk);
        cfg_pos  = {px0[15:0], py0[15:0]};
        cfg_size = {wx[15:0], wy[15:0]};
        cfg_k    = {kx[19:0], ky[19:0]};
        repeat (8) @(posedge clk);
        cap_en = 0; cap_clr = 1;
        @(posedge clk); cap_clr = 0;
        @(negedge clk); ps_start = 1;
        cap_en = 1;
        repeat (6000) @(posedge clk);
        @(negedge clk); ps_start = 0;
        cap_en = 0;
        repeat (600) @(posedge clk);
        n = 0;
    end
endtask

//------------------------------------------------------------------------------
//  在 rowpix 里找两个标记的亮度中心
//------------------------------------------------------------------------------
integer i, c1_i, c2_i;
integer c1_min, c2_min;
real    c1_sum, c1_w, c2_sum, c2_w, c1_pos, c2_pos;
integer lum;
task measure;
    begin
        c1_sum = 0.0; c1_w = 0.0;
        c2_sum = 0.0; c2_w = 0.0;
        for (i = 0; i < 640; i = i + 1) begin
            if (i < rn) lum = rowpix[i][7:0];
            else        lum = 0;
            // 标记 1：源像素 40，屏幕位置 160→80
            if (i >= 50 && i <= 230) begin
                c1_sum = c1_sum + i * lum;
                c1_w   = c1_w + lum;
            end
            // 标记 2：源像素 280，屏幕位置 440→560
            if (i >= 400 && i <= 610) begin
                c2_sum = c2_sum + i * lum;
                c2_w   = c2_w + lum;
            end
        end
        c1_pos = (c1_w > 0.0) ? (c1_sum / c1_w) : -1.0;
        c2_pos = (c2_w > 0.0) ? (c2_sum / c2_w) : -1.0;
    end
endtask

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
integer ws [0:31];
integer NW, wi, W, H, PX, PY, KX, KY;
real    p1 [0:31];
real    p2 [0:31];
real    d1, d2, spread1, spread2;

initial begin
    // 框宽序列：覆盖 320→640 的整段，含奇数宽
    NW = 0;
    // fx3 后半段现在只走偶数宽（viewer_fx 的 g320 已偶数化）⇒ 按真实序列测
    ws[NW]=320; NW=NW+1; ws[NW]=322; NW=NW+1; ws[NW]=324; NW=NW+1;
    ws[NW]=328; NW=NW+1; ws[NW]=336; NW=NW+1; ws[NW]=352; NW=NW+1;
    ws[NW]=368; NW=NW+1; ws[NW]=384; NW=NW+1; ws[NW]=416; NW=NW+1;
    ws[NW]=448; NW=NW+1; ws[NW]=480; NW=NW+1; ws[NW]=512; NW=NW+1;
    ws[NW]=544; NW=NW+1; ws[NW]=576; NW=NW+1; ws[NW]=608; NW=NW+1;
    ws[NW]=624; NW=NW+1; ws[NW]=632; NW=NW+1; ws[NW]=640; NW=NW+1;

    repeat (10) @(posedge clk);
    rst = 0;
    repeat (20) @(posedge clk);

    for (wi = 0; wi < NW; wi = wi + 1) begin
        W  = ws[wi];
        H  = 240 + (W - 320) / 2;           // 与 fx3 后半段相同：等比放大
        PX = 160 - (W - 320) / 2;           // 中心固定在 320
        PY = 0;
        KX = 10485760 / W;
        KY = 65536;                          // 只取源第 0 行
        // 跑 3 帧：前两帧建 warmed/水位，第三帧测
        run_frame(W, 240, PX, 0, KX, KY);
        run_frame(W, 240, PX, 0, KX, KY);
        run_frame(W, 240, PX, 0, KX, KY);
        measure;
        p1[wi] = c1_pos;
        p2[wi] = c2_pos;
        $display("W=%0d  PX=%0d  KX=%0d  rn=%0d  m40=%7.2f  m280=%7.2f  mid=%7.2f",
                 W, PX, KX, rn, c1_pos, c2_pos, (c1_pos+c2_pos)/2.0);
    end

    // 统计漂移：相对于 320 宽那一帧
    spread1 = 0.0;
    for (wi = 0; wi < NW; wi = wi + 1) begin
        d1 = (p1[wi] + p2[wi]) / 2.0 - 320.0;
        if (d1 < 0.0) d1 = -d1;
        if (d1 > spread1) spread1 = d1;
    end
    $display("======================================================");
    $display("max |content mid - 320| = %.2f px", spread1);
    if (spread1 <= 1.0)
        $display("  PASS  缩放过程内容中心稳定（<=1px）");
    else
        $display("  FAIL  缩放过程存在 %.2f px 的横向漂移", spread1);
    $finish;
end

endmodule
