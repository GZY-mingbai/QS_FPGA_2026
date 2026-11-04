//==============================================================================
//  ps_gen_polygon.v —— 像素源生成器：自定义凸多边形
//==============================================================================
//
//      扫描坐标 (x,y) ──► [ ps_gen_polygon ] ──► pix {A,R,G,B}
//                                ▲
//                              color
//
//  【思路：和三角形完全一样，只是边数可变】
//
//  凸多边形 = 若干条边围成。对每条边做"半平面判定"（叉积符号），
//  点在所有边的同一侧 ⟺ 在多边形内。
//
//      e_i = (V[i+1]-V[i]) × (P - V[i])
//      在内部 ⟺ 所有 e_i ≥ 0 或 所有 e_i ≤ 0
//
//  【顶点怎么给】
//  用参数 PX0/PY0 ... PX7/PY7 给出最多 8 个顶点，VN 指定实际用几个。
//  顶点必须按**顺时针或逆时针顺序**给出（不能交叉），且必须是**凸**多边形。
//
//  【★ 顶点保持 parameter，只有颜色是运行时输入 ★】
//  顶点是编译期常数 → 常数 × 变量 → 移位 + 加法，**零 DSP**。
//  若把顶点改成运行时输入，2N 个乘法会变成真乘法（10 个 DSP 以上），
//  而动画很少需要逐帧改顶点形状，所以保持静态最划算。
//
//  【★ 踩过的坑：顶点表不能用 `always @(*)` ★】
//  顶点表只依赖参数、不依赖任何信号。如果写成 `always @(*)`，敏感列表是**空的**，
//  iverilog 里这个块**一次都不会执行**，vx/vy 会一直是未知值 x，
//  导致叉积全是 x、同号判定恒成立 → 整个屏幕都被判成"在多边形内"。
//  所以这里用 `generate` + `assign`（连续赋值永远有效）。
//
//  【cfg 总线格式】见 app_scene_cfg.v 顶部：
//      color [31:0] = { A[7:0], R[7:0], G[7:0], B[7:0] }
//
//  【延迟 = 1 拍】，多边形外输出透明。
//
//==============================================================================

module ps_gen_polygon #(
    parameter integer IMG_W = 640,      // 屏幕宽
    parameter integer IMG_H = 480,      // 屏幕高
    parameter integer VN    = 5,        // 实际顶点数（3~8）

    parameter integer PX0 = 0,  parameter integer PY0 = 0,
    parameter integer PX1 = 0,  parameter integer PY1 = 0,
    parameter integer PX2 = 0,  parameter integer PY2 = 0,
    parameter integer PX3 = 0,  parameter integer PY3 = 0,
    parameter integer PX4 = 0,  parameter integer PY4 = 0,
    parameter integer PX5 = 0,  parameter integer PY5 = 0,
    parameter integer PX6 = 0,  parameter integer PY6 = 0,
    parameter integer PX7 = 0,  parameter integer PY7 = 0
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

//==============================================================================
//  顶点取值的"常数函数"
//==============================================================================
// k = 0..8。k == VN 时返回顶点 0（实现多边形闭合，避免用取模）。
// 在 generate 里调用时 k 是 genvar（编译期常数），会被完全折叠成常量。
//==============================================================================
function signed [16:0] vtx_x;
    input integer k;
    begin
        if      (k == VN) vtx_x = PX0;   // 闭合点
        else if (k == 0)  vtx_x = PX0;
        else if (k == 1)  vtx_x = PX1;
        else if (k == 2)  vtx_x = PX2;
        else if (k == 3)  vtx_x = PX3;
        else if (k == 4)  vtx_x = PX4;
        else if (k == 5)  vtx_x = PX5;
        else if (k == 6)  vtx_x = PX6;
        else if (k == 7)  vtx_x = PX7;
        else              vtx_x = PX0;
    end
endfunction

function signed [16:0] vtx_y;
    input integer k;
    begin
        if      (k == VN) vtx_y = PY0;   // 闭合点
        else if (k == 0)  vtx_y = PY0;
        else if (k == 1)  vtx_y = PY1;
        else if (k == 2)  vtx_y = PY2;
        else if (k == 3)  vtx_y = PY3;
        else if (k == 4)  vtx_y = PY4;
        else if (k == 5)  vtx_y = PY5;
        else if (k == 6)  vtx_y = PY6;
        else if (k == 7)  vtx_y = PY7;
        else              vtx_y = PY0;
    end
endfunction

//==============================================================================
//  顶点表（连续赋值，永远有效）
//==============================================================================
wire signed [16:0] vx [0:8];
wire signed [16:0] vy [0:8];

genvar gi;
generate
for (gi = 0; gi <= 8; gi = gi + 1) begin : g_vtx
    assign vx[gi] = vtx_x(gi);
    assign vy[gi] = vtx_y(gi);
end
endgenerate

//==============================================================================
//  边常数项（时序优化：把叉积里的常数部分提前折叠掉）
//==============================================================================
//  ee = (vxj−vxi)(py−vyi) − (vyj−vyi)(px−vxi)
//     = (vxj−vxi)·py − (vyj−vyi)·px + [ (vyj−vyi)·vxi − (vxj−vyi)·vyi ]
//       └── 变量部分：两次常量乘并行 ──┘   └──────── 常数 K_i ─────────┘
//  K_i 在编译期折成常数 → 变量路径上不再有"先减法、再乘法"的串行进位链
//  （实测这条链是 sd_card_clk 最差路径的源头：−1.16ns @ 8 级）。
wire signed [22:0] k_e [0:8];

genvar gk;
generate
for (gk = 0; gk <= 7; gk = gk + 1) begin : g_ke
    localparam integer NJ = (gk + 1 == VN) ? 0 : (gk + 1);
    assign k_e[gk] = $signed(vy[NJ][10:0] - vy[gk][10:0]) * $signed(vx[gk][10:0])
                   - $signed(vx[NJ][10:0] - vx[gk][10:0]) * $signed(vy[gk][10:0]);
end
endgenerate

//==============================================================================
//  逐边叉积 + 同号判定
//==============================================================================
// ★★ 坐标收窄（时序修复）：屏幕只有 640×480，有效扫描坐标永远 < 1024，
//   所以只取低 10 位（放进 12 位有符号）。消隐期坐标可能 ≥1024 而回卷，
//   但那时 valid=0，计算结果不进像素路径。
//   实测（final_timing.rpt）：17 位的 (py−vy) → 常量乘法 → 26 位比较，
//   单条边 8 级逻辑 12.78ns，是 sd_card_clk 第二差路径（slack −3.03ns）；
//   收窄到 12/23 位后进位链缩短约 1/3。
wire signed [11:0] px = $signed({1'b0, x[9:0]});
wire signed [11:0] py = $signed({1'b0, y[9:0]});

reg  signed [22:0] ee;          // 当前边的叉积
                                  // ★ 23 位足够：px/py 恒在 [0,1023]（取低 10 位），
                                  //   |ee| ≤ 2·1023·1023 + |K| < 2^22 —— 收窄比较/
                                  //   求和的进位链（26→23 位）
reg  all_pos;
reg  all_neg;
integer i;
integer j;                      // 下一条边的目标顶点序号

always @(*) begin
    all_pos = 1'b1;
    all_neg = 1'b1;
    ee      = 26'sd0;

    for (i = 0; i <= 7; i = i + 1) begin
        if (i < VN) begin
            // 第 i 条边：顶点 i → 顶点 i+1；最后一条边回到顶点 0（闭合）
            j = (i + 1 == VN) ? 0 : (i + 1);

            // ★ 变量路径只剩"两次并行常量乘 + 一次三项求和"（K_i 已提出去，
            //   见上面的 g_ke）；数学上与原式完全等价（整数运算无舍入）
            ee = $signed(vx[j][10:0] - vx[i][10:0]) * py
               - $signed(vy[j][10:0] - vy[i][10:0]) * px
               + k_e[i];

            if (!(ee >= 0)) all_pos = 1'b0;
            if (!(ee <= 0)) all_neg = 1'b0;
        end
    end
end

wire in_bounds = (x < IMG_W) && (y < IMG_H);
wire inside = in_bounds && (all_pos || all_neg);

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
