//==============================================================================
//  tb_hd.v —— 单独验证 img_layer_hd 的逐行对齐（1 字 = 1 像素）
//==============================================================================
//  纹理：竖条纹（列 c：c%16<8 白，否则黑），640×480。
//  正确时 480 行的上升沿位置必须完全一致；任何一行整体平移 = 抖动。
//==============================================================================
`timescale 1ns / 1ps
module tb_hd;

localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam integer NPX   = IMG_W * IMG_H;
localparam [20:0] BASE   = 21'h100000;

parameter integer GAP = 20;         // 仲裁延迟（拍）
parameter integer LEAD = 40;

reg clk = 0, mclk = 0, rst = 1;
always #10 clk  = ~clk;    // 50MHz
always #5  mclk = ~mclk;   // 100MHz

reg         ps_start = 0;
reg  [15:0] sx = 0, sy = 0;
wire [31:0] pix;
wire        out_valid, layer_ready, in_box;
wire        req_vld;
wire [20:0] req_addr;
wire [3:0]  req_mask;
reg         req_ack = 0;
reg         pix_wr  = 0;
reg  [31:0] pix_data = 0;

wire pixel_ready = layer_ready;
wire pipe_ce     = layer_ready;
wire adv         = ps_start & pixel_ready;

img_layer_hd #(.IMG_W(IMG_W), .IMG_H(IMG_H), .LEAD(LEAD)) dut (
    .clk(clk), .rst(rst), .pipe_ce(pipe_ce), .adv(adv), .frame_active(ps_start),
    .x(sx), .y(sy), .tex_base(BASE), .layer_en(1'b1),
    .pix(pix), .out_valid(out_valid), .layer_ready(layer_ready), .in_box_o(in_box),
    .mclk(mclk), .req_vld(req_vld), .req_addr(req_addr), .req_mask(req_mask),
    .req_ack(req_ack), .pix_wr(pix_wr), .pix_data(pix_data)
);

// 纹理字：1 像素 = 1 字
function [31:0] tex_word;
    input [20:0] a;
    integer rel, col;
    begin
        rel = a - BASE;
        col = rel % IMG_W;
        tex_word = {8'hFF, rel[23:16], rel[15:8], rel[7:0]};   // 字序号")
    end
endfunction

// 仲裁：GAP 拍后 ack，然后 4 字
integer st;
reg [20:0] a0;
reg [3:0]  k0;
reg [7:0]  g0;
always @(posedge mclk or posedge rst) begin
    if (rst) begin
        st <= 0; req_ack <= 0; pix_wr <= 0; k0 <= 0; g0 <= 0; a0 <= 0;
    end else begin
        req_ack <= 1'b0; pix_wr <= 1'b0;
        case (st)
        0: if (req_vld) begin a0 <= req_addr; st <= 1; g0 <= GAP[7:0]; end
        1: if (g0 != 0) g0 <= g0 - 1;
           else begin req_ack <= 1'b1; k0 <= 0; st <= 2; end
        2: begin
               pix_wr   <= 1'b1;
               pix_data <= tex_word(a0 + k0);
               if (k0 == 3) st <= 0; else k0 <= k0 + 1;
           end
        endcase
    end
end

// 收帧
reg [31:0] fbuf [0:NPX-1];
integer n, g;
reg [8*64-1:0] ppm_name;
task dump_ppm;
    input [8*64-1:0] fn;
    integer f, xx, yy;
    begin
        f = $fopen(fn, "wb");
        $fwrite(f, "P6\n%0d %0d\n255\n", IMG_W, IMG_H);
        for (yy = 0; yy < IMG_H; yy = yy + 1)
            for (xx = 0; xx < IMG_W; xx = xx + 1)
                $fwrite(f, "%c%c%c", fbuf[yy*IMG_W+xx][23:16],
                        fbuf[yy*IMG_W+xx][15:8], fbuf[yy*IMG_W+xx][7:0]);
        $fclose(f);
    end
endtask

initial begin
    $display("== tb_hd GAP=%0d LEAD=%0d ==", GAP, LEAD);
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (10) @(posedge clk);

    for (n = 0; n < NPX; n = n + 1) fbuf[n] = 32'hFF00FF00;

    // 输出侧按 ps_valid 语义收像素（frame_writer 就是这么收的）
    ps_start = 1'b1;
    n = 0;
    g = 0;
    while ((n < NPX) && (g < 2_000_000)) begin
        @(posedge clk);
        g = g + 1;
        if (out_valid && pipe_ce) begin
            fbuf[n] = pix;
            n = n + 1;
        end
    end
    $display("  [tb] 收到 %0d 像素, 用了 %0d 拍", n, g);
    ps_start = 1'b0;
    repeat (20) @(posedge clk);
    if (!$value$plusargs("ppm=%s", ppm_name)) ppm_name = "out/hd.ppm";
    // 逐像素核对：第 k 个像素应当是第 k 个字
    begin : chk
        integer i, k, bad, first;
        bad = 0; first = -1;
        for (i = 0; i < NPX; i = i + 1) begin
            k = fbuf[i][23:0];
            if (k !== i[23:0]) begin
                if (first < 0) begin
                    first = i;
                    $display("  [首个不符] i=%0d 得 %0d 期望 %0d (行 %0d 列 %0d)",
                             i, k, i, i/IMG_W, i%IMG_W);
                end
                bad = bad + 1;
            end
        end
        $display("  [核对] 不符 %0d / %0d", bad, NPX);
    end
    dump_ppm(ppm_name);
    $display("== done ==");
    $finish;
end

endmodule
