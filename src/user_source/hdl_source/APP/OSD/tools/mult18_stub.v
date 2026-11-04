//==============================================================================
//  mult18_stub.v —— EG_PHY_MULT18 的行为桩（**仅供仿真**，不参与综合）
//==============================================================================
//  用途：osd_menu 的 tween 乘法现在显式例化 EG_PHY_MULT18（为了把 DSP 从 4 个
//        降到 1 个，见 osd_menu.v §0 末尾的说明）。iverilog 没有这个原语，
//        不加桩的话所有 OSD 仿真台都会在 elaborate 阶段直接报 unknown module。
//
//  语义（对齐 sim_release/eg4/eg_phy_mult18.v）：
//      · sourcea=0 选 acin、=1 选 a      （★ 接反会乘出 0，是最容易踩的坑）
//      · signeda/signedb=1 时按 18 位有符号解释，否则零扩展成无符号
//      · INPUTREGA/B、OUTPUTREG = "ENABLE" 时各多打一拍
//      · 本设计三处全用 "DISABLE" ⇒ 纯组合，与原 tween() 的零延迟一致
//
//  ⚠️ 限制：不实现 MODE="MULT9X9C" 的拆分语义（本设计只用 MULT18X18C）、
//     不实现 SRMODE / CE 的极性细节（本设计 CE 恒 1、复位恒释放）。
//==============================================================================
module EG_PHY_MULT18 (
    acout, bcout, p, signeda, signedb,
    a, b, acin, bcin,
    cea, ceb, cepd, clk, rstan, rstbn, rstpdn,
    sourcea, sourceb
);
    output [17:0] acout;
    output [17:0] bcout;
    output [35:0] p;
    input  signeda, signedb;
    input  [17:0] a, b, acin, bcin;
    input  cea, ceb, cepd;
    input  clk, rstan, rstbn, rstpdn;
    input  sourcea, sourceb;

    parameter INPUTREGA = "ENABLE";
    parameter INPUTREGB = "ENABLE";
    parameter OUTPUTREG = "ENABLE";
    parameter SRMODE    = "ASYNC";
    parameter MODE      = "MULT18X18C";

    // ---- 端口选择（sourcea=0 → acin；=1 → a）----
    wire [17:0] imux_a = sourcea ? a : acin;
    wire [17:0] imux_b = sourceb ? b : bcin;

    // ---- 可选输入寄存器 ----
    reg [17:0] ra, rb;
    always @(posedge clk) begin
        ra <= imux_a;
        rb <= imux_b;
    end
    wire [17:0] ea = (INPUTREGA == "ENABLE") ? ra : imux_a;
    wire [17:0] eb = (INPUTREGB == "ENABLE") ? rb : imux_b;

    // ---- 符号解释 + 乘法 ----
    wire signed [35:0] sa = signeda ? {{18{ea[17]}}, ea} : {18'd0, ea};
    wire signed [35:0] sb = signedb ? {{18{eb[17]}}, eb} : {18'd0, eb};
    wire signed [35:0] prod = sa * sb;

    // ---- 可选输出寄存器 ----
    reg [35:0] pr;
    always @(posedge clk) pr <= prod;

    assign p     = (OUTPUTREG == "ENABLE") ? pr : prod;
    assign acout = ea;      // 级联输出：桩里直通，本设计未使用
    assign bcout = eb;
endmodule
