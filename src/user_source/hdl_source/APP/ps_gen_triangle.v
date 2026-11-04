//==============================================================================
//  ps_gen_triangle.v —— 像素源生成器：三角形（任意朝向）
//==============================================================================
//
//      扫描坐标 (x,y) ──► [ ps_gen_triangle ] ──► pix {A,R,G,B}
//                                ▲
//                              color
//
//  【判定方法：半平面法（half-plane test）】
//
//  三角形有三条边。对任意一条边 AB，点 P 在"边的哪一侧"可以用**叉积**的符号判断：
//
//      cross = (Bx-Ax)*(Py-Ay) - (By-Ay)*(Px-Ax)
//      cross > 0  → P 在 AB 的左侧
//      cross < 0  → P 在 AB 的右侧
//
//  P 在三角形内 ⟺ 对三条边都在同一侧（都 ≥0 或都 ≤0）。
//  所以判定就是三次叉积 + 符号一致性检查，**不需要任何除法、也不需要排序**。
//
//  【★ 顶点保持 parameter，只有颜色是运行时输入 ★】
//  三个顶点都是**编译期常数**，所以 (Bx-Ax)、(By-Ay) 也是常数。
//  常数 × 变量，综合器会展开成"移位 + 加法"，**一个 DSP 都不占**。
//  如果把顶点也改成运行时输入，这三个乘法就全变成真乘法（6 个 DSP），
//  而动画很少需要逐帧改三角形的顶点——所以这里保持静态最划算。
//  颜色改成输入则完全免费（纯接线）。
//
//  【cfg 总线格式】见 app_scene_cfg.v 顶部：
//      color [31:0] = { A[7:0], R[7:0], G[7:0], B[7:0] }
//
//  【延迟 = 1 拍】，三角形外输出透明。
//
//==============================================================================

module ps_gen_triangle #(
    parameter integer IMG_W = 640,      // 屏幕宽
    parameter integer IMG_H = 480,      // 屏幕高
    parameter integer AX    = 0,        // 顶点 A（静态）
    parameter integer AY    = 0,
    parameter integer BX    = 0,        // 顶点 B（静态）
    parameter integer BY    = 0,
    parameter integer CXp   = 0,        // 顶点 C（名字避开 CX，防止和圆心混淆）
    parameter integer CYp   = 0
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] x,
    input  wire [15:0] y,
    input  wire        valid,
    input  wire [31:0] color,           // {A, R, G, B}
    output reg  [31:0] pix,
    output reg         out_valid
);

//------------------------------------------------------------------------------
//  常数与符号化坐标
//------------------------------------------------------------------------------
// 叉积最大量级：边向量最长约 640，坐标差最长约 480，
// 单个乘积 ≤ 640×480 ≈ 3.1e5，三项相加 ≤ ~1e6 → 21 位有符号足够，用 26 位留余量。
//------------------------------------------------------------------------------
localparam signed [16:0] AXS = AX;
localparam signed [16:0] AYS = AY;
localparam signed [16:0] BXS = BX;
localparam signed [16:0] BYS = BY;
localparam signed [16:0] CXS = CXp;
localparam signed [16:0] CYS = CYp;

// ★★ 坐标收窄（时序修复，与 ps_gen_polygon 同款）：有效扫描坐标 < 1024，
//   取低 10 位放进 12 位有符号；顶点常数同步收窄到 11 位（part-select 丢符号，
//   外面补 $signed）。这样每条边"减法 → 常量乘 → 比较"跑在 12/23 位而不是
//   17/26 位，进位链缩短约 1/3。消隐期坐标 ≥1024 会回卷，但 valid=0
//   不进像素路径。
wire signed [11:0] px = $signed({1'b0, x[9:0]});
wire signed [11:0] py = $signed({1'b0, y[9:0]});

// 三条边的叉积（顶点是常数 → 常数乘法 → 零 DSP）
wire signed [25:0] e1 = (BXS - AXS) * (py - $signed(AYS[10:0])) - (BYS - AYS) * (px - $signed(AXS[10:0]));
wire signed [25:0] e2 = (CXS - BXS) * (py - $signed(BYS[10:0])) - (CYS - BYS) * (px - $signed(BXS[10:0]));
wire signed [25:0] e3 = (AXS - CXS) * (py - $signed(CYS[10:0])) - (AYS - CYS) * (px - $signed(CXS[10:0]));

wire in_bounds = (x < IMG_W) && (y < IMG_H);

// 三条边同号（都 ≥0 或都 ≤0）→ 在三角形内
wire all_pos = (e1 >= 0) && (e2 >= 0) && (e3 >= 0);
wire all_neg = (e1 <= 0) && (e2 <= 0) && (e3 <= 0);

wire inside = in_bounds && (all_pos || all_neg);

//------------------------------------------------------------------------------
//  输出打一拍
//------------------------------------------------------------------------------
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
