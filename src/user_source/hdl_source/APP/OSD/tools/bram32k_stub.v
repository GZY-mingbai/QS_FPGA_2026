//==============================================================================
//  bram32k_stub.v —— EG_PHY_BRAM32K 的行为桩（**仅供仿真**，不参与综合）
//==============================================================================
//  用途：osd_icon_rom / osd_font_cache 里真正的存储是显式例化的
//        EG_PHY_BRAM32K（为了落 32Kb 池）。但 iverilog 没有这个原语，
//        代码里用 `ifdef IVERILOG_SIM 走行为 RAM、`else 走原语。
//        于是**综合分支从来没被仿真验证过** —— 地址译码、bank 选择、
//        写使能门控、输出 mux 全是没测过的代码。
//
//        这个桩把原语的行为补上，再把 RTL 的 ifdef 临时改名
//        （sed 's/ifdef IVERILOG_SIM/ifdef FORCE_SYNTH_RAM/'），
//        就能让**真实要综合的那份 RTL** 跑在 tb 里。
//
//  时序（对齐 eagle_macro.v 的定义，REGMODE="NOREG"）：
//      clka 上升沿采样 addra/wea/csa/dia  → 写
//      clkb 上升沿采样 addrb/csb          → 下一拍 dob 有效（1 拍延迟）
//  即与「mem[addr] 打一拍输出」逐拍一致。
//
//  ⚠️ 限制：不实现 bytea/bytewea（两位宽模式）、INIT_xx 初值、oce 门控。
//     本设计的两个 RAM 都是运行时从 Flash 装载、16 位整字写，用不到这些。
//     如果真的用到，这里会静默不对 —— 所以这个桩只用来查"接线/译码"，
//     不替代上板。
//==============================================================================
module EG_PHY_BRAM32K (
    doa, dia, addra, bytea, bytewea, csa, wea, clka, rsta, ocea,
    dob, dib, addrb, byteb, byteweb, csb, web, clkb, rstb, oceb
);
    output [15:0] doa;
    output [15:0] dob;
    input  [15:0] dia;
    input  [15:0] dib;
    input  [10:0] addra;
    input  [10:0] addrb;
    input  bytea, bytewea, csa, wea, clka, rsta, ocea;
    input  byteb, byteweb, csb, web, clkb, rstb, oceb;

    parameter MODE         = "DP16K";
    parameter DATA_WIDTH_A = "16";
    parameter DATA_WIDTH_B = "16";
    parameter REGMODE_A    = "NOREG";
    parameter REGMODE_B    = "NOREG";
    parameter WRITEMODE_A  = "NORMAL";
    parameter WRITEMODE_B  = "NORMAL";
    parameter SRMODE       = "SYNC";
    parameter CSAMUX       = "SIG";
    parameter CSBMUX       = "SIG";
    parameter OCEAMUX      = "SIG";
    parameter OCEBMUX      = "SIG";
    parameter RSTAMUX      = "SIG";
    parameter RSTBMUX      = "SIG";

    reg [15:0] mem [0:2047];
    reg [15:0] dob_r;

    integer i;
    initial begin
        for (i = 0; i < 2048; i = i + 1) mem[i] = 16'h0000;
        dob_r = 16'h0000;
    end

    // 写口 A（同步）
    always @(posedge clka) begin
        if (csa && wea) mem[addra] <= dia;
    end

    // 读口 B（同步，1 拍延迟；web 用于写透，本设计恒 0）
    always @(posedge clkb) begin
        if (web) mem[addrb] <= dib;
        else if (csb) dob_r <= mem[addrb];
    end

    assign dob = dob_r;
    assign doa = 16'h0000;
endmodule
