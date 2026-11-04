//==============================================================================
//  tb_quad_render.v —— 四宫格渲染出图（导出 PPM 看画面）
//==============================================================================
//  iverilog -g2012 -o qr.vvp -s tb_quad_render tb_quad_render.v quad_sel.v
//  vvp qr.vvp                       # 生成 quad.ppm
//  python ../OSD/tools/ppm2png.py quad.ppm quad.png
//
//  【它验证什么】
//  不是测 ps_gen_image（那套预取逻辑复杂、有现成 tb），而是测**坐标映射**：
//  屏幕上 4 个格子是否各自读到 4 个不同的 SDRAM 槽、边界和间隙是否正确。
//
//  4 张测试图各有：
//    · 不同底色（红/绿/蓝/黄）
//    · 左上角 4px 白边（用来看格子原点是否对）
//    · 中心十字（用来看有没有上下/左右翻转）
//==============================================================================
`timescale 1ns/1ps

module tb_quad_render;

localparam integer SLOT  = 16384;
localparam integer W     = 160;
localparam integer H     = 120;

reg  [9:0]  x = 0, y = 0;
wire [9:0]  lx, ly;
wire [20:0] base;
wire        in;
wire [1:0]  region;

quad_sel #(
    .X0(10'd208), .Y0(10'd100),
    .COL_PITCH(10'd192), .ROW_PITCH(10'd160),
    .CW(10'd160), .CH(10'd120),
    .SLOT_BASE(21'd0), .SLOT_SHIFT(14)
) dut (
    .I_x(x), .I_y(y), .I_en(1'b1),
    .O_x(lx), .O_y(ly), .O_base(base), .O_in(in), .O_region(region)
);

//------------------------------------------------------------------------------
//  SDRAM 模型：4 槽 × 16384 字
//------------------------------------------------------------------------------
reg [31:0] mem [0:4*SLOT-1];   // 565：2 像素/字

function [15:0] mkcolor;
    input [1:0] s;
    input [8:0] px;
    input [8:0] py;
    reg [4:0] rr;
    reg [5:0] gg;
    reg [4:0] bb;
    begin
        case (s)
            2'd0: begin rr = 5'd10 + {1'b0,px[7:4]}; gg = 6'd2;              bb = 5'd2;             end
            2'd1: begin rr = 5'd2;                   gg = 6'd24 + {1'b0,py[7:4]}; bb = 5'd2;         end
            2'd2: begin rr = 5'd2;                   gg = 6'd2;              bb = 5'd10 + {1'b0,px[7:4]}; end
            default: begin
                  rr = 5'd10 + {1'b0,px[7:4]};       gg = 6'd40;             bb = 5'd2;             end
        endcase
        // 左上 4px 白边：验证格子原点
        if (px < 4 || py < 4) begin rr = 5'd31; gg = 6'd63; bb = 5'd31; end
        // 中心十字
        if ((px > 74 && px < 86) || (py > 54 && py < 66)) begin
            rr = 5'd31; gg = 6'd63; bb = 5'd31;
        end
        mkcolor = {rr, gg, bb};
    end
endfunction

integer s, px, py;
reg [15:0] plo, phi;
initial begin
    for (s = 0; s < 4; s = s + 1)
        for (py = 0; py < H; py = py + 1)
            for (px = 0; px < W; px = px + 2) begin
                plo = mkcolor(s[1:0], px[8:0],   py[8:0]);
                phi = mkcolor(s[1:0], px[8:0]+1, py[8:0]);
                mem[s*SLOT + py*80 + (px>>1)] = {phi, plo};
            end
    $display("  SDRAM 模型已填充 4 张测试图（每槽 9600 字，2 像素/字）");
end

//------------------------------------------------------------------------------
//  逐像素扫描 → PPM
//------------------------------------------------------------------------------
integer i, j, fd;
reg [15:0] v;
reg [7:0]  rr, gg, bb;
integer n_in = 0, n_out = 0;

initial begin
    #1;
    fd = $fopen("quad.ppm", "wb");
    if (fd == 0) begin
        $display("  FAIL: 打不开 quad.ppm");
        $finish;
    end
    $fwrite(fd, "P6\n%0d %0d\n255\n", 640, 480);

    for (j = 0; j < 480; j = j + 1) begin
        for (i = 0; i < 640; i = i + 1) begin
            x = i[9:0]; y = j[9:0]; #1;
            if (in) begin
                v  = lx[0] ? mem[base[15:0] + ly*80 + (lx>>1)][31:16]
                            : mem[base[15:0] + ly*80 + (lx>>1)][15:0];
                rr = {v[15:11], 3'b000};
                gg = {v[10:5],  2'b00};
                bb = {v[4:0],   3'b000};
                n_in = n_in + 1;
            end else begin
                rr = 8'h28; gg = 8'h30; bb = 8'h40;   // 间隙/背景：深蓝灰
                n_out = n_out + 1;
            end
            $fwrite(fd, "%c%c%c", rr, gg, bb);
        end
    end

    $fclose(fd);
    $display("  已写出 quad.ppm");
    $display("  命中格子 %0d 像素，未命中 %0d 像素", n_in, n_out);
    $display("  期望命中 = 4×160×120 = %0d", 4*160*120);
    if (n_in == 4*160*120) $display("  PASS: 命中面积正确");
    else                   $display("  FAIL: 命中面积不对");
    $finish;
end

endmodule
