//==============================================================================
//  tb_gb_addr.v —— gb_addr 的模块级验证
//==============================================================================
//
//  被测：gb_addr.v（字符码 → Flash 字库偏移，★ 2 级流水，输出晚输入 2 拍）
//
//  【怎么跑】
//      cd src/user_source/hdl_source
//      iverilog -g2005 -s tb_gb_addr -o tb_gb_addr.vvp APP/tb_gb_addr.v APP/gb_addr.v
//      vvp tb_gb_addr.vvp
//
//  【采样方式】
//  加时钟、每个用例灌码后等 2 个上升沿再采样（第 2 拍结束时 addr 寄存完毕）。
//
//  【期望值从哪来】
//  全部由 Font/字库格式说明.md 的公式手工推导，并与 Python 独立算出的结果交叉核对：
//      · Font/字库格式说明.md §2.4   'A'  → 0x000820
//      · Font/字库格式说明.md §3.4   '中' → 0x023E20
//      · 末字形 row=80,wb=0xFE → 0x3C7A0，+32 正好等于字库文件大小 0x3C7C0  ← 边界铁证
//
//==============================================================================

`timescale 1ns/1ps

module tb_gb_addr;

//==============================================================================
//  时钟与 DUT
//==============================================================================
reg clk = 1'b0;
always #5 clk = ~clk;               // 10ns 周期，与 sd_card_clk 同量级

reg  [15:0] code;
wire [23:0] addr;

gb_addr u_dut (
    .clk  (clk),
    .code (code),
    .addr (addr)
);

//==============================================================================
//  测试表
//==============================================================================
localparam integer NC = 17;

reg [15:0] tc_code [0:NC-1];        // 输入字符码
reg [23:0] tc_exp  [0:NC-1];        // 期望偏移
integer    n_err;
integer    i;

initial begin
    //                       字符码      期望偏移        说明
    // ---- ASCII 路径 ----
    tc_code[ 0] = 16'h0041; tc_exp[ 0] = 24'h000820; // 'A'  可打印（文档 §2.4）
    tc_code[ 1] = 16'h0020; tc_exp[ 1] = 24'h000400; // ' '  空格（字库里本来就是全 0）
    tc_code[ 2] = 16'h0000; tc_exp[ 2] = 24'h000000; // 0x00 控制符（全 0）
    tc_code[ 3] = 16'h007F; tc_exp[ 3] = 24'h000FE0; // 0x7F DEL（全 0）
    // ---- ASCII 路径的非法输入（高字节 0 但低字节 >= 0x80）----
    tc_code[ 4] = 16'h00FF; tc_exp[ 4] = 24'h000000; // 兜底 NULL
    tc_code[ 5] = 16'h00A1; tc_exp[ 5] = 24'h000000; // 兜底 NULL
    // ---- GB2312 路径 ----
    tc_code[ 6] = 16'hD6D0; tc_exp[ 6] = 24'h023E20; // '中'（文档 §3.4）
    tc_code[ 7] = 16'hB2E2; tc_exp[ 7] = 24'h009960; // '测'
    tc_code[ 8] = 16'hCAD4; tc_exp[ 8] = 24'h01B1A0; // '试'
    tc_code[ 9] = 16'hD6F0; tc_exp[ 9] = 24'h024220; // '中' 同区 wb+0x20 → 偏移 +0x400
    // ---- 区码边界 ----
    tc_code[10] = 16'hA9A1; tc_exp[10] = 24'h006E00; // 符号区末区 0xA9 → row 8
    tc_code[11] = 16'hB0A1; tc_exp[11] = 24'h0079C0; // 汉字区首区 0xB0 → row 9
    tc_code[12] = 16'hF7FE; tc_exp[12] = 24'h03C7A0; // 末字形 row=80,wb=0xFE（+32 = 文件尾）
    // ---- 空区 / 非法 ----
    tc_code[13] = 16'hAAA1; tc_exp[13] = 24'h000000; // 空区 0xAA → 兜底 NULL
    tc_code[14] = 16'hF8A1; tc_exp[14] = 24'h000000; // 空区 0xF8 → 兜底 NULL
    tc_code[15] = 16'hB0A0; tc_exp[15] = 24'h000000; // 位码 0xA0 非法 → 兜底 NULL
    tc_code[16] = 16'hFFFF; tc_exp[16] = 24'h000000; // 区码 0xFF → 兜底 NULL

    n_err = 0;

    $display("");
    $display("============================================================");
    $display("  tb_gb_addr —— 字符码 -> Flash 字库偏移（2 级流水）");
    $display("============================================================");
    $display("  用例清单：");
    $display("   [0] ASCII 'A'            [1] ASCII ' '        [2] 0x00");
    $display("   [3] ASCII 0x7F           [4] 0x00FF 非法      [5] 0x00A1 非法");
    $display("   [6] '中' D6D0            [7] '测' B2E2        [8] '试' CAD4");
    $display("   [9] '中' 同区 wb+0x20      [10] 区码边界 A9A1   [11] 区码边界 B0A1");
    $display("  [12] 末字形 F7FE           [13] 空区 AAA1       [14] 空区 F8A1");
    $display("  [15] 位码非法 B0A0          [16] 区码 0xFFFF");
    $display("------------------------------------------------------------");

    // 先让流水线空转 2 拍（旧码冲出去），再逐用例灌码
    code = 16'h0000;
    @(posedge clk); @(posedge clk);

    for (i = 0; i < NC; i = i + 1) begin
        @(negedge clk);
        code = tc_code[i];              // 负沿改码，避开上升沿竞争
        @(posedge clk);                 // 第 1 拍末：stage1 寄存
        @(posedge clk);                 // 第 2 拍末：addr 寄存
        #1;                             // 稳定后再采样
        if (addr !== tc_exp[i]) begin
            n_err = n_err + 1;
            $display("  x [%0d] code=%04h -> %06h   期望 %06h", i, tc_code[i], addr, tc_exp[i]);
        end else begin
            $display("    [%0d] code=%04h -> %06h   OK", i, tc_code[i], addr);
        end
    end

    $display("------------------------------------------------------------");
    $display("  用例总数 = %0d", NC);
    $display("  不匹配   = %0d  (期望 0)", n_err);
    if (n_err == 0) $display("  RESULT: PASS");
    else            $display("  RESULT: FAIL");
    $display("============================================================");
    $display("");
    $finish;
end

//==============================================================================
//  看门狗
//==============================================================================
initial begin
    #1_000_000;
    $display("  RESULT: FAIL  (看门狗超时)");
    $finish;
end

endmodule
