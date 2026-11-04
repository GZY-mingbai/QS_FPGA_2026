//==============================================================================
//  tb_jitter.v —— 显示通路【逐行横向对齐】回归（专抓"抖动/锯齿"）
//==============================================================================
//  iverilog -g2012 -o jt.vvp -s tb_jitter tb_jitter.v viewer_scene.v quad_sel.v \
//      img_layer_ctrl.v img_layer_hd.v ps_gen_image.v ps_merge_alpha_pp.v \
//      ps_to_rgb24.v app_scan_gen.v
//
//  【判据】纹理是竖条纹（源像素列 c：c%16<8 白，否则黑）铺满 640×480。
//    正确时每行的上升沿位置必须**完全相同**（0,8,16,…）。
//    只要某一行整体左/右移了 1 像素，边缘位置就会变 ⇒ 那就是"抖动"。
//
//  【为什么这个判据比"和参考图 diff"更狠】
//    参考图 diff 只能告诉你"某一格错了"；本判据直接给**每行的相位**，
//    一眼能看出是"整行平移"（弹出错位）还是"个别像素错"（半字选错）。
//
//  【服务模型】照 sdr_port_sched 的真实节拍：一组 4 字、先 ack 再回流，
//    延迟可配（GAP），用来验证"延迟变大时画面会不会错位"。
//==============================================================================
`timescale 1ns / 1ps

module tb_jitter;

localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam integer NPX   = IMG_W * IMG_H;

localparam [20:0] ABASE = 21'h100000;      // 2^20

// ---- 可配：源尺寸 / 目标框 / 仲裁间隔 ----
parameter integer SRC_PX = 640;            // 源宽（像素）
parameter integer SRC_PY = 480;            // 源高（像素）
parameter integer BOX_X0 = 0;
parameter integer BOX_Y0 = 0;
parameter integer BOX_W  = 640;
parameter integer BOX_H  = 480;
parameter integer GAP    = 10;
parameter integer STRIPE = 16;             // 条纹周期（源像素）
parameter integer HD     = 0;              // 1 = 走高清层（1 字 = 1 像素 ARGB8888）

localparam integer SRC_W = HD ? SRC_PX : (SRC_PX / 2);   // 纹理字/行
localparam integer KXV   = 65536 * SRC_W / BOX_W;
localparam integer KYV   = 65536 * SRC_PY / BOX_H;

reg clk = 0, mclk = 0, rst = 1;
always #10 clk  = ~clk;      // 50MHz
always #5  mclk = ~mclk;     // 100MHz

//------------------------------------------------------------------------------
//  DUT
//------------------------------------------------------------------------------
reg         ps_start = 0;
wire        pixel_ready;
wire [23:0] ps_data;
wire        pipe_ce;
reg         fmt565   = 1'b1;
reg         wr_stall = 1'b0;
parameter integer STALL = 0;      // 1 = 周期性拉低 pixel_ready（模拟 SDRAM 争用）
integer st_cnt = 0;

reg         a_hd = 0;
initial a_hd = (HD != 0);
reg  [31:0] a_pos  = 32'd0;
reg  [31:0] a_size = {16'd640, 16'd480};
reg  [39:0] a_k    = {20'd32768, 20'd65536};
reg  [19:0] a_ori  = 20'd0;
reg  [20:0] a_base = ABASE;
reg         a_en   = 1;

reg  [31:0] b_pos   = {16'd640, 16'd0};
reg  [31:0] b_size  = {16'd640, 16'd480};
reg  [39:0] b_k     = {20'd32768, 20'd65536};
reg  [20:0] b_base  = 21'd0;
initial b_base = ABASE + 21'h80000;   // B 层纹理放远一点
reg         b_en    = 0;
parameter integer BEN = 0;   // 1 = 同时打开 B 层（两层合成）
initial b_en = (BEN != 0);
reg  [7:0]  alpha_b = 8'd255;

// 由参数决定框位置/尺寸/缩放比（必须在声明之后）
initial begin
    a_pos  = {BOX_Y0[15:0], BOX_X0[15:0]};
    a_size = {BOX_H[15:0],  BOX_W[15:0]};
    a_k    = {KXV[19:0],    KYV[19:0]};
end

wire [1:0]  tex_req_vld;
wire [41:0] tex_req_addr;
wire [7:0]  tex_req_mask;
wire [1:0]  tex_pix_wr;
wire [63:0] tex_pix_data;
reg  [1:0]  ack_r = 0;
reg  [1:0]  pwr_r = 0;
reg  [63:0] pd_r  = 0;
wire [1:0]  tex_req_ack = ack_r;
assign tex_pix_wr   = pwr_r;
assign tex_pix_data = pd_r;

wire [15:0] scan_x, scan_y;

viewer_scene #(.IMG_W(IMG_W), .IMG_H(IMG_H), .SRC_W(SRC_W)) dut (
    .clk(clk), .rst(rst), .mclk(mclk),
    .ps_start(ps_start), .pixel_ready(pixel_ready), .ps_data(ps_data), .pipe_ce(pipe_ce),
    .fmt565(fmt565), .wr_stall(wr_stall),
    .a_hd(a_hd), .a_pos(a_pos), .a_size(a_size), .a_k(a_k), .a_ori(a_ori),
    .a_base(a_base), .a_en(a_en),
    .quad_mode(1'b0),
    .b_pos(b_pos), .b_size(b_size), .b_k(b_k), .b_base(b_base), .b_en(b_en),
    .alpha_b(alpha_b),
    .scan_x(scan_x), .scan_y(scan_y),
    .osd_on(1'b0), .osd_fade(8'd0), .osd_pix(32'd0),
    .tex_req_vld(tex_req_vld), .tex_req_addr(tex_req_addr), .tex_req_mask(tex_req_mask),
    .tex_req_ack(tex_req_ack), .tex_pix_wr(tex_pix_wr), .tex_pix_data(tex_pix_data)
);

//------------------------------------------------------------------------------
//  纹理：竖条纹，周期 16（8 白 8 黑）。一个字 = 两个像素。
//------------------------------------------------------------------------------
function [31:0] tex_word;
    input [20:0] a;
    integer rel, row, col, c0, c1;
    reg [15:0] p0, p1;
    begin
        rel = a - ABASE;
        row = rel / SRC_W;
        col = rel % SRC_W;
        c0  = col * 2;
        c1  = col * 2 + 1;
        if (HD != 0) begin
            // 高清层：1 字 = 1 像素 {A8,R,G,B}
            tex_word = (col < (STRIPE/2)) ? 32'hFFFF_FFFF : 32'hFF00_0000;
        end else begin
            p0  = ((c0 % STRIPE) < (STRIPE/2)) ? 16'hFFFF : 16'h0000;
            p1  = ((c1 % STRIPE) < (STRIPE/2)) ? 16'hFFFF : 16'h0000;
            tex_word = {p1, p0};
        end
    end
endfunction

//------------------------------------------------------------------------------
//  假仲裁：GAP 拍后 ack，然后每次 4 字
//------------------------------------------------------------------------------
integer    srv_st0;
reg [20:0] srv_a0;
reg [3:0]  srv_k0;
reg [7:0]  gap0;

always @(posedge mclk or posedge rst) begin
    if (rst) begin
        srv_st0 <= 0; ack_r <= 0; pwr_r <= 0; pd_r <= 0;
        gap0 <= 0; srv_k0 <= 0; srv_a0 <= 0;
    end else begin
        pwr_r <= 2'b00;
        ack_r <= 2'b00;
        case (srv_st0)
        0: if (tex_req_vld[0]) begin
               srv_a0  <= tex_req_addr[20:0];
               srv_st0 <= 1;
               gap0    <= GAP[7:0];
           end
        1: if (gap0 != 0) gap0 <= gap0 - 1;
           else begin
               ack_r   <= ack_r | 2'b01;
               srv_k0  <= 0;
               srv_st0 <= 2;
           end
        2: begin
               pwr_r[0]   <= 1'b1;
               pd_r[31:0] <= tex_word(srv_a0 + srv_k0);
               if (srv_k0 == 3) srv_st0 <= 0;
               else             srv_k0 <= srv_k0 + 1;
           end
        default: srv_st0 <= 0;
        endcase
    end
end

//------------------------------------------------------------------------------
//  收帧
//------------------------------------------------------------------------------
reg [23:0] fbuf [0:NPX-1];
integer i, fd, frames;
reg [8*64-1:0] ppm_name;

task run_frame;
    integer g, n;
    begin
        for (n = 0; n < NPX; n = n + 1) fbuf[n] = 24'hFF00FF;
        ps_start = 1'b1;
        @(posedge clk);
        n = 0;
        while (n < NPX) begin
            g = 0;
            while (!(ps_start && pixel_ready) && (g < 200000)) begin
                @(posedge clk);
                g = g + 1;
            end
            if (g >= 200000) begin
                $display("  [FAIL] deadlock at pixel %0d (x=%0d y=%0d)", n, scan_x, scan_y);
                $finish;
            end
            @(posedge clk);
            // ps_data 在这一拍有效（与 frame_writer 的取数沿一致）
            fbuf[scan_y * IMG_W + scan_x] = ps_data;
            n = n + 1;
        end
        @(posedge clk);
        ps_start = 1'b0;
        repeat (20) @(posedge clk);
    end
endtask

task dump_ppm;
    input [8*64-1:0] fn;
    integer f, x, y;
    begin
        f = $fopen(fn, "wb");
        $fwrite(f, "P6\n%0d %0d\n255\n", IMG_W, IMG_H);
        for (y = 0; y < IMG_H; y = y + 1)
            for (x = 0; x < IMG_W; x = x + 1)
                $fwrite(f, "%c%c%c",
                        fbuf[y*IMG_W+x][23:16], fbuf[y*IMG_W+x][15:8], fbuf[y*IMG_W+x][7:0]);
        $fclose(f);
        $display("  [tb] 写出 %0s", fn);
    end
endtask

always @(posedge clk) begin
    if (STALL != 0) begin
        st_cnt <= st_cnt + 1;
        // 每 97 拍拉低 1 拍：与 640 不整除 ⇒ 每行落在不同列
        wr_stall <= ((st_cnt % 97) == 96);
    end else wr_stall <= 1'b0;
end

// 错拍探针：什么时候"扫描走了但图层没接"
integer n_slip = 0;
always @(posedge clk) begin
    if (rst==0 && ps_start && pixel_ready && (dut.u_ta.u_gen.adv_eff==0)) begin
        if (n_slip < 12)
            $display("  [slip] t=%0d at (%0d,%0d) pop_stall=%b s2_hold=%b avail=%b cnt=%0d",
                     $time, scan_x, scan_y, dut.u_ta.u_gen.pop_stall,
                     dut.u_ta.u_gen.s2_hold_w, dut.u_ta.u_gen.pix_avail,
                     dut.u_ta.u_gen.pix_count);
        n_slip = n_slip + 1;
    end
end

initial begin
    $display("== tb_jitter: src=%0dx%0d box=%0dx%0d@(%0d,%0d) KX=%0d KY=%0d GAP=%0d ==",
             SRC_PX, SRC_PY, BOX_W, BOX_H, BOX_X0, BOX_Y0, KXV, KYV, GAP);
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (10) @(posedge clk);
    run_frame;
    if (!$value$plusargs("ppm=%s", ppm_name)) ppm_name = "out/jitter.ppm";
    dump_ppm(ppm_name);
    $display("== done: slip=%0d ==", n_slip);
    $finish;
end

endmodule
