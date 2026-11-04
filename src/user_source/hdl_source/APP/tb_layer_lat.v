//==============================================================================
//  tb_layer_lat.v —— 量 A 层两条通路的**输出延迟**（565 / 888）
//==============================================================================
//
//  【要回答的问题】上板现象：565(320×240) 切到 HD 的一瞬，整幅图向右跳 >4px。
//    这两条通路是同一个 `sx` 扫描、在 `viewer_scene` 里按 a_hd 二选一
//        pix_a = a_hd ? hd_pix : ta_pix
//    只要两条通路的"输出相对 sx 的延迟"不相等，切换那一帧就会整幅平移
//    **延迟差** 个像素。`img_layer_hd` 的 LAT=3 注释写着"与 ps_gen_image 的
//    D_img 对齐"，但没有任何断言量过这件事。
//
//  【怎么量】给两条通路各自一块"只有一个白点"的纹理：
//      · 565 通道：纹理像素 50（字 25 的低半字）= 白 ⇒ 2× 放大后应出现在
//        屏幕 x = 100（框 640×480 @ (0,0)，kx=16384）
//      · 888 通道：纹理字 100 = 白 ⇒ 应出现在屏幕 x = 100
//    扫一整帧，记录每个通道 "像素有效且非黑" 时扫描器所在的 x。
//    ⇒ 两条通路的 x 之差 = 切换时会看到的整幅位移（正 = 画面右移）。
//
//  【跑法】工作目录 = APP/
//    iverilog -g2012 -s tb_layer_lat -o /tmp/ll.vvp tb_layer_lat.v \
//        img_layer_ctrl.v img_layer_hd.v ps_gen_image.v img_afifo.v && vvp /tmp/ll.vvp
//==============================================================================
`timescale 1ns / 1ps

module tb_layer_lat;

localparam integer IMG_W = 640, IMG_H = 480;
localparam integer SRCW  = 160, SRCH = 240;
localparam integer NPW565 = SRCW*SRCH;         // 38400
localparam [20:0] BASE565 = 21'd614400;
localparam [20:0] BASE888 = 21'd729600;

reg clk = 0, mclk = 0;
always #10 clk  = ~clk;
always #4  mclk = ~mclk;
reg rst = 1;

//------------------------------------------------------------------------------
//  扫描器（两条通路共用同一个 x/y）
//------------------------------------------------------------------------------
reg [15:0] scan_x = 0, scan_y = 0;
reg        ps_start = 0;
wire       adv565, adv888;
wire       adv = ps_start & adv565 & adv888;
wire       pipe_ce = adv565 & adv888;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        scan_x <= 0; scan_y <= 0;
    end else if (!ps_start) begin
        scan_x <= 0; scan_y <= 0;
    end else if (adv) begin
        if (scan_x == IMG_W-1) begin
            scan_x <= 0;
            scan_y <= (scan_y == IMG_H-1) ? 16'd0 : scan_y + 16'd1;
        end else scan_x <= scan_x + 16'd1;
    end
end

//------------------------------------------------------------------------------
//  假 SDRAM（两份，各自按掩码回字；纹理用函数生成，省内存）
//------------------------------------------------------------------------------
function [31:0] tex565;          // 只有字 25 的低半字 = 0xFFFF（纹理像素 50）
    input [20:0] a;
    begin
        tex565 = (a == BASE565 + 21'd25) ? 32'h0000_FFFF : 32'd0;
    end
endfunction
function [31:0] tex888;          // 只有字 100 = 白
    input [20:0] a;
    begin
        tex888 = (a == BASE888 + 21'd100) ? 32'h00FF_FFFF : 32'd0;
    end
endfunction

// ---- 两条 4 字突发通道（队列 + 逐字回；与 tb_thumb_align 同模型）----
localparam integer QD = 32;

// 通道 565
wire        a565_rv; wire [20:0] a565_ra; wire [3:0] a565_rm; reg a565_ack;
reg         a565_wr; reg [31:0] a565_dat;
reg  [20:0] qa5 [0:QD-1]; reg [3:0] qm5 [0:QD-1];
integer     qw5 = 0, qr5 = 0; reg emit5; reg [1:0] kk5; reg [20:0] ca5; reg [3:0] cm5;
assign a565_ack = a565_rv && ((qw5 - qr5) < (QD - 4));
always @(posedge mclk or posedge rst) begin
    if (rst) begin qw5<=0; qr5<=0; emit5<=0; kk5<=0; ca5<=0; cm5<=0; a565_wr<=0; a565_dat<=0; end
    else begin
        a565_wr <= 0;
        if (a565_rv && a565_ack) begin qa5[qw5%QD]<=a565_ra; qm5[qw5%QD]<=a565_rm; qw5<=qw5+1; end
        if (!emit5) begin
            if (qw5 != qr5) begin ca5<=qa5[qr5%QD]; cm5<=qm5[qr5%QD]; qr5<=qr5+1; kk5<=0; emit5<=1; end
        end else begin
            if (cm5[kk5]) begin a565_wr<=1; a565_dat <= tex565(ca5 + {19'd0, kk5}); end
            if (kk5 == 2'd3) emit5 <= 0; else kk5 <= kk5 + 2'd1;
        end
    end
end

// 通道 888
wire        a888_rv; wire [20:0] a888_ra; wire [3:0] a888_rm; reg a888_ack;
reg         a888_wr; reg [31:0] a888_dat;
reg  [20:0] qa8 [0:QD-1]; reg [3:0] qm8 [0:QD-1];
integer     qw8 = 0, qr8 = 0; reg emit8; reg [1:0] kk8; reg [20:0] ca8; reg [3:0] cm8;
assign a888_ack = a888_rv && ((qw8 - qr8) < (QD - 4));
always @(posedge mclk or posedge rst) begin
    if (rst) begin qw8<=0; qr8<=0; emit8<=0; kk8<=0; ca8<=0; cm8<=0; a888_wr<=0; a888_dat<=0; end
    else begin
        a888_wr <= 0;
        if (a888_rv && a888_ack) begin qa8[qw8%QD]<=a888_ra; qm8[qw8%QD]<=a888_rm; qw8<=qw8+1; end
        if (!emit8) begin
            if (qw8 != qr8) begin ca8<=qa8[qr8%QD]; cm8<=qm8[qr8%QD]; qr8<=qr8+1; kk8<=0; emit8<=1; end
        end else begin
            if (cm8[kk8]) begin a888_wr<=1; a888_dat <= tex888(ca8 + {19'd0, kk8}); end
            if (kk8 == 2'd3) emit8 <= 0; else kk8 <= kk8 + 2'd1;
        end
    end
end

//------------------------------------------------------------------------------
//  DUT 1：565 通路（img_layer_ctrl + ps_gen_image）
//------------------------------------------------------------------------------
wire [31:0] p565; wire v565, rdy565, inb565, emp565;
wire [4:0] d565; wire [3:0] e565; wire [127:0] s565; wire w565;
img_layer_ctrl #(
    .IMG_W(IMG_W), .IMG_H(IMG_H), .SRC_W(SRCW), .LEAD(384), .AFIFO_AW(10)
) u565 (
    .clk(clk), .rst(rst), .mclk(mclk),
    .pipe_ce(pipe_ce), .adv(adv), .frame_active(ps_start),
    .x(scan_x), .y(scan_y),
    .cfg_pos({16'd0, 16'd0}), .cfg_size({16'd640, 16'd480}),
    .cfg_k({20'd16384, 20'd32768}),
    .tex_base(BASE565), .layer_en(1'b1),
    .quad_en(1'b0),
    .pix(p565), .out_valid(v565), .layer_ready(rdy565), .in_box_o(inb565),
    .dbg_delta(d565), .dbg_evt(e565), .smp_bus(s565), .smp_win(w565),
    .pix_empty_o(emp565),
    .no_underrun_fix(1'b0), .exp_mode(4'd0),
    .req_vld(a565_rv), .req_addr(a565_ra), .req_mask(a565_rm), .req_ack(a565_ack),
    .pix_wr(a565_wr), .pix_data(a565_dat)
);
assign adv565 = rdy565;

//------------------------------------------------------------------------------
//  DUT 2：888 通路（img_layer_hd，LAT=3）
//------------------------------------------------------------------------------
wire [31:0] p888; wire v888, rdy888, inb888;
img_layer_hd #(
    .IMG_W(IMG_W), .IMG_H(IMG_H), .AFIFO_AW(6), .RQ_AW(5), .LEAD(40), .LAT(3)
) u888 (
    .clk(clk), .rst(rst), .mclk(mclk),
    .pipe_ce(pipe_ce), .adv(adv), .frame_active(ps_start),
    .x(scan_x), .y(scan_y),
    .tex_base(BASE888), .layer_en(1'b1),
    .pix(p888), .out_valid(v888), .layer_ready(rdy888), .in_box_o(inb888),
    .req_vld(a888_rv), .req_addr(a888_ra), .req_mask(a888_rm), .req_ack(a888_ack),
    .pix_wr(a888_wr), .pix_data(a888_dat)
);
assign adv888 = rdy888;

//------------------------------------------------------------------------------
//  记录"白点出现在哪个 scan_x"
//------------------------------------------------------------------------------
integer x565 = -1, x888 = -1;
integer n565 = 0, n888 = 0;
always @(posedge clk) begin
    if (ps_start) begin
        if (v565 && (p565[23:0] != 24'd0)) begin
            if (x565 < 0) x565 = scan_x;
            n565 = n565 + 1;
        end
        if (v888 && (p888[23:0] != 24'd0)) begin
            if (x888 < 0) x888 = scan_x;
            n888 = n888 + 1;
        end
    end
end

integer cyc = 0;
always @(posedge clk) if (ps_start) cyc = cyc + 1;

initial begin
    $display("=== 两条通路的输出延迟（白点应出现在 x=100）===");
    rst = 1; repeat (8) @(posedge clk); rst = 0; repeat (8) @(posedge clk);
    @(negedge clk); ps_start = 1;
    while (cyc < IMG_W*IMG_H + 4000) @(posedge clk);
    @(negedge clk); ps_start = 0;
    repeat (50) @(posedge clk);

    $display("  565 通路: 白点首次出现在 x=%0d（%0d 个白像素）", x565, n565);
    $display("  888 通路: 白点首次出现在 x=%0d（%0d 个白像素）", x888, n888);
    if (x565 >= 0 && x888 >= 0) begin
        $display("  ⇒ 切换时整幅位移 = %0d 像素（正 = 画面右移）", x888 - x565);
        if (x888 == x565) $display("  PASS: 两条通路延迟一致");
        else              $display("  FAIL: 延迟差 %0d 像素", x888 - x565);
    end else $display("  FAIL: 没采到白点");
    $finish;
end

endmodule
