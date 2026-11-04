`timescale 1ns/1ps
//==============================================================================
//  tb_tween_equiv.v —— 证明「25×25 推断乘法」与「16×9 显式 MULT18」逐位等价
//==============================================================================
//  左边：osd_menu 改之前的 tween() 函数（原样抄下来）
//  右边：改之后的四段式（d / e / MULT18 / 叠加），乘法走 mult18_stub
//  对同一组 (from,to,t) 同时求值，逐位比较。
//
//  扫描范围：
//    · t 的 256 个有效值（只有 t[15:8] 参与运算）全覆盖
//    · (from,to) 取随机 + 边界：位置类 0..1024、数值类 0..100、以及 65535 端点
//==============================================================================
module tb_tween_equiv;

//------------------------------ 旧实现（原样）------------------------------
function [8:0] ease_of_old;
    input [7:0] x;
    reg   [8:0]  d;
    reg   [17:0] sq;
    begin
        d  = (x < 8'd128) ? {1'b0, x} : (9'd256 - {1'b0, x});
        sq = d * d;
        ease_of_old = (x < 8'd128) ? sq[16:7] : (9'd256 - sq[16:7]);
    end
endfunction

function [15:0] tween_old;
    input [15:0] from;
    input [15:0] to;
    input [15:0] t;
    reg   [24:0] p;
    reg   [8:0]  e;
    begin
        e = ease_of_old(t[15:8]);
        if (to >= from) begin
            p     = {9'd0, (to - from)} * {16'd0, e};
            tween_old = from + p[23:8];
        end else begin
            p     = {9'd0, (from - to)} * {16'd0, e};
            tween_old = from - p[23:8];
        end
    end
endfunction

//------------------------------ 新实现 ------------------------------
function [8:0] ease_of_new;
    input [7:0] x;
    reg   [8:0]  d;
    reg   [17:0] sq;
    begin
        d  = (x < 8'd128) ? {1'b0, x} : (9'd256 - {1'b0, x});
        sq = d * d;
        ease_of_new = (x < 8'd128) ? sq[16:7] : (9'd256 - sq[16:7]);
    end
endfunction

reg  [15:0] from, to, t;
wire        tw_up = (to >= from);
wire [15:0] tw_d  = tw_up ? (to - from) : (from - to);
wire [8:0]  tw_e  = ease_of_new(t[15:8]);
wire [35:0] tw_p36;

EG_PHY_MULT18 #(
    .INPUTREGA("DISABLE"), .INPUTREGB("DISABLE"), .OUTPUTREG("DISABLE"),
    .SRMODE("ASYNC"), .MODE("MULT18X18C")
) u_mult (
    .acout(), .bcout(), .p(tw_p36),
    .signeda(1'b0), .signedb(1'b0),
    .a({2'b00, tw_d}), .b({9'd0, tw_e}),
    .acin(18'd0), .bcin(18'd0),
    .cea(1'b1), .ceb(1'b1), .cepd(1'b1), .clk(1'b0),
    .rstan(1'b1), .rstbn(1'b1), .rstpdn(1'b1),
    .sourcea(1'b1), .sourceb(1'b1)
);
wire [24:0] tw_p = tw_p36[24:0];
wire [15:0] tween_new = tw_up ? (from + tw_p[23:8]) : (from - tw_p[23:8]);

//------------------------------ 比对 ------------------------------
integer i, k, errs, cases;
reg [15:0] a, b;

task check;
    input [15:0] f, tt, tv;
    begin
        from = f; to = tt; t = tv;
        #1;
        cases = cases + 1;
        if (tween_old(f, tt, tv) !== tween_new) begin
            errs = errs + 1;
            if (errs <= 10)
                $display("MISMATCH from=%0d to=%0d t=%0d  old=%0d new=%0d",
                         f, tt, tv, tween_old(f, tt, tv), tween_new);
        end
    end
endtask

initial begin
    errs = 0; cases = 0;

    // 1) 位置类：0..1024 全对 × 256 个 t 值（只取稀疏组合，避免太慢）
    for (i = 0; i <= 1024; i = i + 7)
        for (k = 0; k <= 1024; k = k + 11)
            for (a = 0; a < 256; a = a + 1)
                check(i[15:0], k[15:0], {a[7:0], 8'd0});

    // 2) 数值类：0..100
    for (i = 0; i <= 100; i = i + 1)
        for (k = 0; k <= 100; k = k + 1)
            for (a = 0; a < 256; a = a + 3)
                check(i[15:0], k[15:0], {a[7:0], 8'd0});

    // 3) 随机 + 端点
    for (i = 0; i < 200000; i = i + 1) begin
        a = $random; b = $random;
        check(a, b, {$random} % 65536);
    end
    check(16'd0,     16'd65535, 16'hFFFF);
    check(16'd65535, 16'd0,     16'hFFFF);
    check(16'd0,     16'd0,     16'h8000);
    check(16'd65535, 16'd65535, 16'h8000);

    $display("--------------------------------------------------");
    $display("cases = %0d   mismatches = %0d", cases, errs);
    if (errs == 0) $display("ALL PASS: tween 新旧实现逐位等价");
    else           $display("FAIL: 存在不等价");
    $finish;
end

endmodule
