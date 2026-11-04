`timescale 1ns / 1ps
//==============================================================================
//  tb_fx0.v —— 单层复现：fx0 的"框逐帧左移 + 纹理限速"
//==============================================================================
//  【为什么要单独写这个台】
//    tb_viewer 用的是**固定几何 + 无限速**的纹理源，它证明了"稀疏框本身没问题"。
//    但板上还有两样它没有的东西：
//      ① 几何**每帧都在变**（X0 每帧 -几像素）⇒ ps_gen_image 的 idle_reset 每帧触发；
//      ② 纹理源是**限速**的（SDRAM 只有 4 个 TX 槽 ≈ 25M 字/s，两层还要分）。
//    这个台只保留这两样，其余全部最小化，所以跑得快、结论干净。
//
//  【判据】
//    每帧要求 adv（被接受的扫描像素）达到 NPX=307200 个。
//    若某一帧连续 WD 拍没有 adv ⇒ 判定卡死，打印该帧的
//      rq_idx / ds_idx / warmed / layer_ready / pop_want / pix_count，
//    以及当时的 X0（就能知道卡在"左移多少像素"的位置）。
//
//  【跑法】
//    iverilog -g2005 -s tb_fx0 -o /tmp/tb_fx0.vvp \
//        APP/img_afifo.v APP/ps_gen_image.v APP/img_layer_ctrl.v APP/app_scan_gen.v APP/tb_fx0.v
//    vvp /tmp/tb_fx0.vvp
//==============================================================================

module tb_fx0;

localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam integer NPX   = IMG_W * IMG_H;

// 每帧左移多少像素（fx0：640/300 ≈ 2.13，这里取 2 便于观察）
parameter integer PX_STEP   = 2;
// 跑多少帧（到 640 就是整段动画）
parameter integer NFRAME    = 20;
// 纹理服务：每 SRV_GAP 拍 mclk 出一个字（4 → 25M 字/s，等于板上 TX 总产能）
parameter integer SRV_GAP   = 4;
// 卡死判据：连续这么多拍没有 adv
parameter integer WD        = 2000000;   // 一帧至少要 307200 拍，判据必须大于它

reg clk = 0, mclk = 0;
always #10 clk  = ~clk;      // 50MHz
always #5  mclk = ~mclk;     // 100MHz

reg rst = 1;

//------------------------------------------------------------------------------
//  扫描 + 图层
//------------------------------------------------------------------------------
reg         ps_start = 0;
wire        pixel_ready;
wire [15:0] sx, sy;
wire        svalid;

reg  [31:0] cfg_pos  = 32'd0;
reg  [31:0] cfg_size = {16'd640, 16'd480};
reg  [39:0] cfg_k    = {20'd16384, 20'd32768};

wire        adv = ps_start & pixel_ready;

wire        req_vld;
wire [20:0] req_addr;
wire [3:0]  req_mask;
wire        req_ack;
reg         pix_wr;
reg  [31:0] pix_data;
wire [31:0] pix;
wire        out_valid;
wire        layer_ready;

app_scan_gen #(.IMG_W(IMG_W), .IMG_H(IMG_H)) u_scan (
    .clk(clk), .rst(rst),
    .ps_start(ps_start), .pixel_ready(pixel_ready),
    .x(sx), .y(sy), .valid(svalid)
);

img_layer_ctrl #(
    .IMG_W(IMG_W), .IMG_H(IMG_H), .SRC_W(160),
    .LEAD(64), .AFIFO_AW(8), .ALIGN_ONLY(0)
) u_lyr (
    .clk(clk), .rst(rst), .mclk(mclk),
    .pipe_ce(pixel_ready),
    .adv(adv), .frame_active(ps_start),
    .x(sx), .y(sy),
    .cfg_pos(cfg_pos), .cfg_size(cfg_size), .cfg_k(cfg_k),
    .tex_base(21'd614400), .layer_en(1'b1),
    .quad_en(1'b0),
    .pix(pix), .out_valid(out_valid), .layer_ready(layer_ready),
    .no_underrun_fix(1'b0), .exp_mode(4'd0),
    .req_vld(req_vld), .req_addr(req_addr), .req_mask(req_mask), .req_ack(req_ack),
    .pix_wr(pix_wr), .pix_data(pix_data)
);

assign pixel_ready = layer_ready | ~ps_start;

//------------------------------------------------------------------------------
//  限速纹理源：req 入队，按 SRV_GAP 逐字回（内容无所谓，只看流控）
//------------------------------------------------------------------------------
localparam integer QD = 64;
reg [20:0] qa [0:QD-1];
reg [3:0]  qm [0:QD-1];
integer    qw = 0, qr = 0;
reg        emit = 0;
reg [1:0]  kk = 0;
reg [20:0] cur_a = 0;
reg [3:0]  cur_m = 0;
integer    gapc = 0;

assign req_ack = req_vld && ((qw - qr) < (QD - 4));

always @(posedge mclk or posedge rst) begin
    if (rst) begin
        qw <= 0; qr <= 0; emit <= 0; kk <= 0; gapc <= 0;
        cur_a <= 0; cur_m <= 0;
    end else begin
        if (req_vld && req_ack) begin
            qa[qw % QD] <= req_addr;
            qm[qw % QD] <= req_mask;
            qw <= qw + 1;
        end

        if (!emit) begin
            if (qw != qr) begin
                cur_a <= qa[qr % QD];
                cur_m <= qm[qr % QD];
                qr    <= qr + 1;
                kk    <= 2'd0;
                emit  <= 1'b1;
            end
        end else if (gapc < SRV_GAP - 1) begin
            gapc <= gapc + 1;
        end else begin
            gapc <= 0;
            if (kk == 2'd3) emit <= 1'b0;
            else            kk   <= kk + 2'd1;
            // 只有 mask 里点到的字才回（一个字一个 pix_wr）
            if (cur_m[kk]) ;
        end
    end
end

// pix_wr：只在"该字被 mask 点到"的那一拍拉高
always @(*) begin
    pix_wr   = emit && (gapc == SRV_GAP - 1) && cur_m[kk];
    pix_data = {8'hFF, cur_a[7:0], cur_a[15:8], 8'h5A};
end

//------------------------------------------------------------------------------
//  主流程：逐帧左移
//------------------------------------------------------------------------------
integer f, i, g, px, bad = 0;
integer adv_cnt;

initial begin
    $display("=== tb_fx0：PX_STEP=%0d NFRAME=%0d SRV_GAP=%0d ===", PX_STEP, NFRAME, SRV_GAP);
    repeat (20) @(posedge clk);
    rst = 0;
    repeat (20) @(posedge clk);

    px = 0;
    for (f = 0; f < NFRAME; f = f + 1) begin
        // ---- 本帧几何：fx0 = A 层左移 px ----
        cfg_pos = {16'd0 - px[15:0], 16'd0};
        repeat (20) @(posedge clk);

        adv_cnt = 0;
        g       = 0;
        ps_start = 1'b1;

        // 每帧要收满 NPX 个扫描像素
        while ((adv_cnt < NPX) && (g < WD)) begin
            @(posedge clk);
            if (adv) adv_cnt = adv_cnt + 1;
            g = g + 1;
        end

        ps_start = 1'b0;

        if (adv_cnt < NPX) begin
            bad = bad + 1;
            $display("  [FAIL] 帧 %0d（X0=%0d）卡死：只剩 %0d 个像素没走，等了 %0d 拍",
                     f, -px, NPX - adv_cnt, g);
            $display("         rq_idx=%0d ds_idx=%0d warmed=%b layer_ready=%b",
                     u_lyr.u_gen.rq_idx, u_lyr.u_gen.ds_idx,
                     u_lyr.u_gen.warmed, layer_ready);
            $display("         pix_count=%0d committed(push-pop)=%0d/%0d",
                     u_lyr.u_gen.pix_count, u_lyr.u_gen.push_w, u_lyr.u_gen.pop_w);
        end else begin
            $display("  [ ok ] 帧 %0d（X0=%0d）收满 %0d 像素，用 %0d 拍", f, -px, NPX, g);
        end

        px = px + PX_STEP;
        if (px > 640) px = 640;
        repeat (2000) @(posedge clk);
    end

    $display("\n==== 卡死帧数 = %0d / %0d ====", bad, NFRAME);
    if (bad == 0) $display("PASS");
    else          $display("FAIL");
    $finish;
end

initial begin
    #800_000_000;
    $display("GLOBAL TIMEOUT");
    $finish;
end

endmodule
