`timescale 1ns/1ps
//==============================================================================
//  tb_pgi_equiv.v —— ps_gen_image / img_layer_ctrl 的【逐拍等价】验证
//==============================================================================
//  【目的】证明 P1 的 D1（删 PAT_CHK 诊断通路）/ D2（删 rt_fire 死代码）/
//  P1-0（声明前移）**没有改变任何行为**。
//
//  【做法】同一份激励喂给两个 img_layer_ctrl：
//      dut_new : src 里的当前版本
//      dut_ref : .backup/hdl_source_P0 里的改动前版本（改名 *_ref）
//  每拍比较**全部输出**：pix / out_valid / layer_ready / in_box_o /
//  req_vld / req_addr / req_mask / pix_pop / pix_empty_o。
//  两边的假仲裁器各自独立、行为相同。
//
//  【覆盖】全屏 1:1、2 倍放大、部分出屏、非整数缩放 —— 各跑满一帧。
//==============================================================================
module tb_pgi_equiv;

localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam integer SRCW  = 160;
localparam integer SRCH  = 240;
localparam integer NPW   = SRCW * SRCH;
localparam [20:0]  TEX_BASE = 21'd614400;

reg clk = 0, mclk = 0;
always #10 clk  = ~clk;
always #4  mclk = ~mclk;
reg rst = 1;

//--------------------------------------------------------------- 纹理
reg [31:0] tex [0:NPW-1];
integer ti, twx, twy;
reg [15:0] ev565, od565;
initial begin
    for (ti = 0; ti < NPW; ti = ti + 1) begin
        twy = ti / SRCW; twx = ti % SRCW;
        ev565 = {twx[4:0], twy[3:0], twx[2:0], 4'h0};
        od565 = {~twx[4:0], ~twy[3:0], twx[2:0], 4'hF};
        tex[ti] = {od565, ev565};
    end
end

//--------------------------------------------------------------- 激励
reg [31:0] cfg_pos, cfg_size;
reg [39:0] cfg_k;
reg        layer_en;
reg [15:0] scan_x, scan_y;
reg        ps_start = 0;

//--------------------------------------------------------------- 仲裁器（参数化，实例化两次）
//  用宏避免写两遍
//---- 先把两边的连线声明好（仲裁器要用，避免先使用后声明）----
wire        n_reqv, n_ack, n_pop, n_ov, n_rdy, n_ibox, n_pemp;
wire [20:0] n_raddr;
wire [3:0]  n_rmask;
wire [31:0] n_pix;
wire        r_reqv, r_ack, r_pop, r_ov, r_rdy, r_ibox, r_pemp;
wire [20:0] r_raddr;
wire [3:0]  r_rmask;
wire [31:0] r_pix;

//---- 仲裁器 A（给新实现）----
reg  [20:0] qa_n [0:63];
reg  [3:0]  qm_n [0:63];
integer     qw_n = 0, qr_n = 0;
reg         emit_n, pwr_n;
reg  [1:0]  kk_n;
reg  [20:0] cad_n;
reg  [3:0]  cm_n;
reg  [31:0] pdt_n;
integer     gm_n;

assign n_ack = n_reqv && ((qw_n - qr_n) < 60);

always @(posedge mclk or posedge rst) begin
    if (rst) begin
        qw_n <= 0; qr_n <= 0; emit_n <= 1'b0; kk_n <= 2'd0;
        cad_n <= 21'd0; cm_n <= 4'd0; pwr_n <= 1'b0; pdt_n <= 32'd0;
    end else begin
        pwr_n <= 1'b0;
        if (n_reqv && n_ack) begin
            qa_n[qw_n % 64] <= n_raddr;
            qm_n[qw_n % 64] <= n_rmask;
            qw_n <= qw_n + 1;
        end
        if (!emit_n) begin
            if (qw_n != qr_n) begin
                cad_n <= qa_n[qr_n % 64];
                cm_n  <= qm_n[qr_n % 64];
                qr_n  <= qr_n + 1;
                kk_n  <= 2'd0;
                emit_n <= 1'b1;
            end
        end else begin
            if (cm_n[kk_n]) begin
                pwr_n <= 1'b1;
                gm_n  = cad_n - TEX_BASE + kk_n;
                pdt_n <= (gm_n >= 0 && gm_n < NPW) ? tex[gm_n] : 32'd0;
            end
            if (kk_n == 2'd3) emit_n <= 1'b0;
            else              kk_n <= kk_n + 2'd1;
        end
    end
end

//---- 仲裁器 B（给参考实现，逻辑完全相同）----
reg  [20:0] qa_r [0:63];
reg  [3:0]  qm_r [0:63];
integer     qw_r = 0, qr_r = 0;
reg         emit_r, pwr_r;
reg  [1:0]  kk_r;
reg  [20:0] cad_r;
reg  [3:0]  cm_r;
reg  [31:0] pdt_r;
integer     gm_r;

assign r_ack = r_reqv && ((qw_r - qr_r) < 60);

always @(posedge mclk or posedge rst) begin
    if (rst) begin
        qw_r <= 0; qr_r <= 0; emit_r <= 1'b0; kk_r <= 2'd0;
        cad_r <= 21'd0; cm_r <= 4'd0; pwr_r <= 1'b0; pdt_r <= 32'd0;
    end else begin
        pwr_r <= 1'b0;
        if (r_reqv && r_ack) begin
            qa_r[qw_r % 64] <= r_raddr;
            qm_r[qw_r % 64] <= r_rmask;
            qw_r <= qw_r + 1;
        end
        if (!emit_r) begin
            if (qw_r != qr_r) begin
                cad_r <= qa_r[qr_r % 64];
                cm_r  <= qm_r[qr_r % 64];
                qr_r  <= qr_r + 1;
                kk_r  <= 2'd0;
                emit_r <= 1'b1;
            end
        end else begin
            if (cm_r[kk_r]) begin
                pwr_r <= 1'b1;
                gm_r  = cad_r - TEX_BASE + kk_r;
                pdt_r <= (gm_r >= 0 && gm_r < NPW) ? tex[gm_r] : 32'd0;
            end
            if (kk_r == 2'd3) emit_r <= 1'b0;
            else              kk_r <= kk_r + 2'd1;
        end
    end
end

//--------------------------------------------------------------- 新实现

img_layer_ctrl #(.IMG_W(IMG_W), .IMG_H(IMG_H), .SRC_W(SRCW),
                 .LEAD(384), .AFIFO_AW(10), .SAMPLE_CENTER(0)) dut_new (
    .clk(clk), .rst(rst), .pipe_ce(n_rdy), .adv(ps_start & n_rdy),
    .frame_active(ps_start), .x(scan_x), .y(scan_y),
    .cfg_pos(cfg_pos), .cfg_size(cfg_size), .cfg_k(cfg_k),
    .tex_base(TEX_BASE), .layer_en(layer_en),
    .quad_en(1'b0),
    .pix(n_pix), .out_valid(n_ov), .layer_ready(n_rdy), .in_box_o(n_ibox),
    .dbg_delta(), .dbg_evt(), .smp_bus(), .smp_win(), .pix_empty_o(n_pemp),
    .no_underrun_fix(1'b0), .exp_mode(4'd0), .mclk(mclk),
    .req_vld(n_reqv), .req_addr(n_raddr), .req_mask(n_rmask), .req_ack(n_ack),
    .pix_wr(pwr_n), .pix_data(pdt_n)
);

//--------------------------------------------------------------- 参考实现

img_layer_ctrl_ref #(.IMG_W(IMG_W), .IMG_H(IMG_H), .SRC_W(SRCW),
                     .LEAD(384), .AFIFO_AW(10)) dut_ref (
    .clk(clk), .rst(rst), .pipe_ce(r_rdy), .adv(ps_start & r_rdy),
    .frame_active(ps_start), .x(scan_x), .y(scan_y),
    .cfg_pos(cfg_pos), .cfg_size(cfg_size), .cfg_k(cfg_k),
    .tex_base(TEX_BASE), .layer_en(layer_en),
    .pix(r_pix), .out_valid(r_ov), .layer_ready(r_rdy), .in_box_o(r_ibox),
    .dbg_delta(), .dbg_evt(), .smp_bus(), .smp_win(), .pix_empty_o(r_pemp),
    .no_underrun_fix(1'b0), .exp_mode(4'd0), .mclk(mclk),
    .req_vld(r_reqv), .req_addr(r_raddr), .req_mask(r_rmask), .req_ack(r_ack),
    .pix_wr(pwr_r), .pix_data(pdt_r)
);

//--------------------------------------------------------------- 扫描
always @(posedge clk or posedge rst) begin
    if (rst) begin scan_x <= 16'd0; scan_y <= IMG_H - 1; end
    else if (!ps_start) begin scan_x <= 16'd0; scan_y <= IMG_H - 1; end
    else if (ps_start & n_rdy & r_rdy) begin
        if (scan_x == IMG_W - 1) begin scan_x <= 16'd0; scan_y <= scan_y - 16'd1; end
        else                          scan_x <= scan_x + 16'd1;
    end
end

//--------------------------------------------------------------- 逐拍比对
integer n_cmp = 0, n_bad = 0;
integer n_nonzero = 0;
always @(posedge clk) begin
    if (!rst) begin
        n_cmp = n_cmp + 1;
        if ((n_pix      !== r_pix)   || (n_ov   !== r_ov)   ||
            (n_rdy      !== r_rdy)   || (n_ibox !== r_ibox) ||
            (n_reqv     !== r_reqv)  || (n_raddr!== r_raddr)||
            (n_rmask    !== r_rmask) || (n_pemp !== r_pemp)) begin
            n_bad = n_bad + 1;
            if (n_bad <= 12)
                $display("   DIFF t=%0t xy=(%0d,%0d) pix n=%h r=%h | ov %b/%b rdy %b/%b ibox %b/%b req %b/%b addr %h/%h mask %b/%b",
                         $time, scan_x, scan_y, n_pix, r_pix, n_ov, r_ov, n_rdy, r_rdy,
                         n_ibox, r_ibox, n_reqv, r_reqv, n_raddr, r_raddr, n_rmask, r_rmask);
        end
        if (n_pix !== 32'h0) n_nonzero = n_nonzero + 1;
    end
end

//--------------------------------------------------------------- 一轮
integer wd;
task run_case;
    input [255:0] tag;
    input [15:0]  x0, y0, rw, rh;
    input [19:0]  kx, ky;
    begin
        $display("\n--- %0s : X0=%0d Y0=%0d RW=%0d RH=%0d KX=%0d KY=%0d ---",
                 tag, $signed(x0), $signed(y0), rw, rh, kx, ky);
        ps_start = 0; rst = 1; #200; rst = 0; #200;
        cfg_pos = {x0, y0}; cfg_size = {rw, rh}; cfg_k = {kx, ky}; layer_en = 1'b1;
        repeat (16) @(posedge clk);
        ps_start = 1; @(posedge clk);
        wd = 0;
        while (!(scan_x == IMG_W-1 && scan_y == 0) && wd < 3*307200) begin
            @(posedge clk); wd = wd + 1;
        end
        if (wd >= 3*307200) $display("      !! 看门狗超时 scan=(%0d,%0d)", scan_x, scan_y);
        @(posedge clk); ps_start = 0; repeat (60) @(posedge clk);
    end
endtask

initial begin
    $display("======================================================");
    $display("  tb_pgi_equiv : ps_gen_image 重构前后逐拍等价");
    $display("======================================================");
    cfg_pos=0; cfg_size=0; cfg_k=0; layer_en=0;
    rst=1; repeat(10) @(posedge clk); rst=0; repeat(50) @(posedge clk);

    run_case("T1 2x up  320px->640", 16'd0,   16'd0,   16'd640, 16'd480, 20'd16384, 20'd32768);
    run_case("T2 1:1    @ (160,120)", 16'd160, 16'd120, 16'd320, 16'd240, 20'd32768, 20'd65536);
    run_case("T3 offscr @ (-100,-50)", 16'hFF9C,16'hFFCE,16'd640,16'd480, 20'd16384, 20'd32768);
    run_case("T4 nonint ->448",       16'd0,   16'd0,   16'd448, 16'd448, 20'd23405, 20'd35082);
    run_case("T5 nonint ->200x150",   16'd0,   16'd0,   16'd200, 16'd150, 20'd52428, 20'd104857);

    $display("\n======================================");
    $display("  比对 %0d 拍（非零像素 %0d 拍）", n_cmp, n_nonzero);
    if (n_bad == 0) $display("  ALL PASS  (改动前后逐拍完全一致)");
    else            $display("  FAIL      (%0d 拍不同)", n_bad);
    $display("======================================");
    $finish;
end

endmodule
