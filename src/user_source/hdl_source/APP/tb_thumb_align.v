//==============================================================================
//  tb_thumb_align.v —— 565 缩略图 vs HD 的**内容对齐**测量（真实 RTL）
//==============================================================================
//
//  【要回答的问题】用户报：苹果图从 565（320×240）切到 HD 时，画面"左右移动"。
//    真值只有一个：**同一幅图，两种显示路径，内容中心应该落在同一个源坐标上**。
//
//  【怎么测】不走我自己写的公式，而是直接例化上板用的那两个模块：
//      img_layer_ctrl（含 ps_gen_image 的 map_src 乘法映射 + 565 半字拆包）
//    + 假 SDRAM（按 req_mask 逐字回 32 位打包字）
//    纹理就是 sd_bmp_loader 会写进 SDRAM 的那 38400 个 565 字
//      （tools/apple565.hex = 苹果图按"逐通道 2:1 平均 + 纵向取偶行 + 行翻转"算出来的）
//
//  然后按 fx3 后半段的几个真实几何各扫一整帧 640×480，各出一张 PPM：
//      w=320/240 @ (160,120)  K={32768,65536}   ← 静止的 320×240 状态
//      w=480/360 @ ( 80, 60)  K={21845,43690}   ← 放大途中
//      w=640/480 @ (  0,  0)  K={16384,32768}   ← fx3 终点（下一步就切 HD）
//    K 的取法与 viewer_fx 一致：KX = floor(65536*160/W)，KY = floor(65536*240/H)。
//
//  【怎么判】把每张图和 HD 参考（源图原样）比：
//    · 内容中心（苹果红色区域的质心）换算回源坐标，跨 3 个宽度是否恒定；
//    · 每行的最佳横向位移（SAD 最小），中位数应恒为 0。
//
//  【怎么跑】工作目录 = APP/
//    iverilog -g2012 -s tb_thumb_align -o /tmp/ta.vvp tb_thumb_align.v \
//        img_layer_ctrl.v ps_gen_image.v img_afifo.v && vvp /tmp/ta.vvp
//==============================================================================
`timescale 1ns / 1ps

module tb_thumb_align;

localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam integer SRCW  = 160;          // 纹理行宽（字）
localparam integer SRCH  = 240;
localparam integer NPW   = SRCW * SRCH;  // 38400
localparam integer NPIX  = IMG_W * IMG_H;
localparam [20:0]  TEX_BASE = 21'd614400;

reg clk = 0, mclk = 0;
always #10 clk  = ~clk;                  // 50MHz
always #4  mclk = ~mclk;                 // 125MHz

reg rst = 1;

reg [31:0] tex [0:NPW-1];
initial $readmemh("tools/apple565.hex", tex);

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
        if (scan_x == IMG_W - 1) begin
            scan_x <= 0;
            if (scan_y == IMG_H - 1) scan_y <= 0;
            else                     scan_y <= scan_y + 16'd1;
        end else scan_x <= scan_x + 16'd1;
    end
end

reg [31:0] adv_cnt;
always @(posedge clk or posedge rst) begin
    if (rst)              adv_cnt <= 0;
    else if (!ps_start)   adv_cnt <= 0;
    else if (adv)         adv_cnt <= adv_cnt + 32'd1;
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

// ★ 2026-10-28：给仲裁加**周期性停顿** —— 真板上 SDRAM 会被显示读/写口抢占，
//   请求侧会遇到"这一拍不给 ack"。原来假存储秒回，等于假设零竞争，
//   组丢失/多回一个字这类问题根本暴露不出来。
reg [3:0] jit = 0;
always @(posedge mclk or posedge rst)
    if (rst) jit <= 4'd0;
    else     jit <= jit + 4'd1;
wire ack_ok = (jit[3:1] != 3'b000);      // 每 8 拍空 1~2 拍
assign req_ack = req_vld && ((qw - qr) < (QD - 4)) && ack_ok;

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
//  整帧捕获（out_valid 的顺序 = 扫描顺序）
//------------------------------------------------------------------------------
reg [23:0] fbuf [0:NPIX-1];
integer    fidx;
reg        cap_clr = 0;
always @(posedge clk or posedge rst) begin
    if (rst)          fidx <= 0;
    else if (cap_clr) fidx <= 0;
    else if (ps_start && out_valid && (fidx < NPIX)) begin
        fbuf[fidx] <= pix[23:0];
        fidx <= fidx + 1;
    end
end

//------------------------------------------------------------------------------
//  跑一帧并出 PPM
//------------------------------------------------------------------------------
integer fd, i;
integer snap_push = 0, snap_pop = 0;
task run_frame;
    input integer wx;
    input integer wy;
    input integer px0;
    input integer py0;
    input integer kx;
    input integer ky;
    input [255:0] fname;      // 8*32 位，够放字符串
    begin
        @(negedge clk);
        cfg_pos  = {px0[15:0], py0[15:0]};
        cfg_size = {wx[15:0], wy[15:0]};
        cfg_k    = {kx[19:0], ky[19:0]};
        repeat (8) @(posedge clk);
        // 清捕获缓冲（必须在 ps_start 之前，且**之后不能再清**）
        @(negedge clk); cap_clr = 1;
        @(negedge clk); cap_clr = 0;
        @(negedge clk); ps_start = 1;
        // 等扫完一帧（adv 计数到 640*480，再多给 2k 拍排空）
        while (adv_cnt < NPIX) @(posedge clk);
        repeat (2000) @(posedge clk);
        // ★ 必须在 ps_start 落下**之前**抓：落下后 idle_reset 会把计数清零
        snap_push = dut.u_gen.push_cnt;
        snap_pop  = dut.u_gen.popw_cnt;
        @(negedge clk); ps_start = 0;
        repeat (200) @(posedge clk);

        fd = $fopen(fname, "wb");
        $fwrite(fd, "P6\n%0d %0d\n255\n", IMG_W, IMG_H);
        for (i = 0; i < NPIX; i = i + 1) begin
            if (i < fidx)
                $fwrite(fd, "%c%c%c", fbuf[i][23:16], fbuf[i][15:8], fbuf[i][7:0]);
            else
                $fwrite(fd, "%c%c%c", 8'h00, 8'h00, 8'h00);
        end
        $fclose(fd);
        $display("shot %0s  size=%0dx%0d k=(%0d,%0d) pos=(%0d,%0d)  pix=%0d",
                 fname, wx, wy, kx, ky, px0, py0, fidx);
        // ★ 请求侧记账：一帧应恰好推入 240*160 = 38400 个字
        $display("      push=%0d pop=%0d (expect 38400)  差=%0d",
                 snap_push, snap_pop,
                 (snap_push >= snap_pop) ? (snap_push - snap_pop)
                                         : (snap_pop - snap_push));
    end
endtask

integer wi;
reg [255:0] fn;
integer wlist [0:8];
initial begin
    wlist[0] = 320; wlist[1] = 480; wlist[2] = 640;
end

initial begin
    rst = 1; repeat (10) @(posedge clk); rst = 0; repeat (10) @(posedge clk);

    // fx3 后半段的完整宽度序列（K 与 viewer_fx 的 KX=65536*160/W 同算法）
    for (wi = 0; wi < 3; wi = wi + 1) begin
        $sformat(fn, "out_thumb_w%0d.ppm", wlist[wi]);
        run_frame(wlist[wi], (wlist[wi]*3)/4,
                  (640 - wlist[wi])/2, (480 - (wlist[wi]*3)/4)/2,
                  10485760 / wlist[wi], 15728640 / ((wlist[wi]*3)/4), fn);
    end

    $display("=== done ===");
    $finish;
end

endmodule
