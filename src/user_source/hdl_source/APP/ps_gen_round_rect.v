//==============================================================================
//  ps_gen_round_rect.v —— 像素源生成器：圆角矩形
//==============================================================================
//
//      扫描坐标 (x,y) ──► [ ps_gen_round_rect ] ──► pix {A,R,G,B}
//                                  ▲
//                         cfg {X0,Y0,RW,RH} + color
//
//  【思路】
//  圆角矩形 = 矩形 挖掉四个角，每个角用一个"四分之一圆"来裁。
//
//      ┌───────────────┐
//      │╭─────────────╮│
//      ││             ││
//      │╰─────────────╯│
//      └───────────────┘
//
//  对每个角，把坐标换算成"到该角圆心的距离分量" (a, b)：
//      在角内 ⟺ a² + b² ≤ R²
//
//  【★ 怎么做到零 DSP ★】
//  如果直接写 a*a + b*b ≤ R*R，会用到 2 个乘法器。
//  但这里 R 是**编译期常数**（CR 参数），所以改成**查表**：
//  预先算好"给定 b，a 最大能到多少"（= floor(sqrt(R²-b²))），存成一张 17 项的表。
//  判定就退化成 `a ≤ 表[b]` —— 只有比较器和一个小 mux，**一个乘法器都不用**。
//
//  【★ 为什么圆角半径 CR 保持 parameter ★】
//  改成运行时输入的话，查表就得变成真乘法（2 个 DSP/实例），而动画很少需要
//  动态改圆角半径。位置/尺寸/颜色则改成运行时输入（几乎免费）。
//
//  【表的生成】R=16 时，b = 0..16 对应的 max_a：
//      b:  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16
//    max_a:16 15 15 15 15 15 14 14 13 13 12 11 10  9  7  5  0
//  （= floor(sqrt(256 - b²))）
//  ★ 如果要用别的圆角半径，按同样公式重算这张表即可。
//
//  【cfg 总线格式】见 app_scene_cfg.v 顶部：
//      cfg   [63:0] = { X0[15:0], Y0[15:0], RW[15:0], RH[15:0] }
//      color [31:0] = { A[7:0],   R[7:0],  G[7:0],  B[7:0]  }
//
//  【延迟 = 1 拍】，非图形区域输出透明。
//
//==============================================================================

module ps_gen_round_rect #(
    parameter integer IMG_W = 640,      // 屏幕宽
    parameter integer IMG_H = 480,      // 屏幕高
    parameter integer CR    = 16        // 圆角半径（静态；表按 16 生成，支持 0~16）
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] x,
    input  wire [15:0] y,
    input  wire        valid,
    input  wire [63:0] cfg,             // {X0, Y0, RW, RH}
    input  wire [31:0] color,           // {A, R, G, B}
    output reg  [31:0] pix,
    output reg         out_valid
);

//==============================================================================
//  圆角半宽查表（纯组合，零乘法器）
//==============================================================================
// 输入 b（离角边的垂直距离 0~16），输出该行允许的最大 a。
// 这本质上是一张 ROM，但只有 17 项，综合器会直接做成组合逻辑（mux 树）。
function [4:0] corner_max_a;
    input [4:0] b;
    begin
        case (b)
            5'd0 : corner_max_a = 5'd16;
            5'd1 : corner_max_a = 5'd15;
            5'd2 : corner_max_a = 5'd15;
            5'd3 : corner_max_a = 5'd15;
            5'd4 : corner_max_a = 5'd15;
            5'd5 : corner_max_a = 5'd15;
            5'd6 : corner_max_a = 5'd14;
            5'd7 : corner_max_a = 5'd14;
            5'd8 : corner_max_a = 5'd13;
            5'd9 : corner_max_a = 5'd13;
            5'd10: corner_max_a = 5'd12;
            5'd11: corner_max_a = 5'd11;
            5'd12: corner_max_a = 5'd10;
            5'd13: corner_max_a = 5'd9;
            5'd14: corner_max_a = 5'd7;
            5'd15: corner_max_a = 5'd5;
            5'd16: corner_max_a = 5'd0;
            default: corner_max_a = 5'd0;
        endcase
    end
endfunction

//==============================================================================
//  拆总线
//==============================================================================
wire [15:0] X0 = cfg[63:48];
wire [15:0] Y0 = cfg[47:32];
wire [15:0] RW = cfg[31:16];
wire [15:0] RH = cfg[15:0];

localparam [15:0] CRU = CR;

//==============================================================================
//  矩形本体判定
//==============================================================================
wire in_bounds = (x < IMG_W) && (y < IMG_H);
wire [15:0] lx = x - X0;                  // 矩形内局部坐标（x<X0 时回绕成大数）
wire [15:0] ly = y - Y0;
wire in_rect = in_bounds && (lx < RW) && (ly < RH);

//==============================================================================
//  四个角
//==============================================================================
wire [15:0] rxu = RW - CRU;               // 右侧角区的起点
wire [15:0] ryu = RH - CRU;               // 下侧角区的起点

wire c_tl = (lx <  CRU) && (ly <  CRU);    // 左上
wire c_tr = (lx >= rxu) && (ly <  CRU);    // 右上
wire c_bl = (lx <  CRU) && (ly >= ryu);    // 左下
wire c_br = (lx >= rxu) && (ly >= ryu);    // 右下

// 到"角圆心"的距离分量（在角区内时都在 0~16 范围内）
wire [15:0] a_tl16 = CRU - lx;             // 左/右：a = 到该侧角圆心的水平距离
wire [15:0] b_tl16 = CRU - ly;             // 上/下：b = 到该侧角圆心的垂直距离
wire [15:0] a_tr16 = lx - rxu;
wire [15:0] a_bl16 = CRU - lx;
wire [15:0] b_bl16 = ly - ryu;
wire [15:0] a_br16 = lx - rxu;
wire [15:0] b_br16 = ly - ryu;

wire ok_tl = (a_tl16[4:0] <= corner_max_a(b_tl16[4:0]));
wire ok_tr = (a_tr16[4:0] <= corner_max_a(b_tl16[4:0]));
wire ok_bl = (a_bl16[4:0] <= corner_max_a(b_bl16[4:0]));
wire ok_br = (a_br16[4:0] <= corner_max_a(b_br16[4:0]));

// 只在落在某个角区时才做圆角裁剪；不在任何角区 → 直接算"在内"
wire corner_ok = c_tl ? ok_tl :
                 c_tr ? ok_tr :
                 c_bl ? ok_bl :
                 c_br ? ok_br : 1'b1;

wire inside = in_rect && corner_ok;

//==============================================================================
//  输出打一拍
//==============================================================================
always @(posedge clk or posedge rst) begin
    if (rst) begin
        pix       <= 32'h0000_0000;
        out_valid <= 1'b0;
    end else begin
        pix       <= inside ? color : 32'h0000_0000;
        out_valid <= valid;
    end
end

endmodule
