`timescale 1ns/1ps
//==============================================================================
//  tb_audio_vol.v —— 音量模块的功能与"音质"验证
//==============================================================================
//  三条硬指标（对应"音量设置不能影响音质"）：
//    ① vol=100 时输出与输入**逐位相同**（单位增益是恒等映射，不是"差 0.4%"）
//    ② vol=0   时输出恒 0
//    ③ 任意 vol：|out − x·g/256| ≤ 0.5，即"四舍五入到最近"，没有系统性偏置
//  另外扫一遍单调性：vol 变大，输出绝对值不应变小。
//==============================================================================
module tb_audio_vol;

reg clk = 0;
always #10 clk = ~clk;          // 50MHz 仿真时钟，时序与 25MHz 无关

reg         rst = 1;
reg         valid = 0;
reg  [6:0]  vol = 7'd100;
reg  [23:0] l_in = 0, r_in = 0;
wire [23:0] l_out, r_out;
wire        o_valid;

audio_vol dut (
    .clk(clk), .rst(rst), .valid(valid), .vol(vol),
    .l_in(l_in), .r_in(r_in),
    .l_out(l_out), .r_out(r_out), .o_valid(o_valid)
);

integer errs = 0, cases = 0;

// 参考模型：g9 = round(vol*41/16)；(x*g9 + 128) >> 8（补码下即 floor）
function [8:0] ref_g;
    input [6:0] v;
    reg [12:0] t;
    begin
        t = v * 41;
        ref_g = (t + 8) >> 4;
    end
endfunction

function signed [23:0] ref_y;
    input signed [23:0] x;
    input [8:0] g;
    reg signed [32:0] p;
    begin
        // ★ 必须把 g 显式转成 signed：有符号 × 无符号在 Verilog 里整体按无符号
        //   算，负样本会算成天文数字（参考模型自己先踩了一次）。
        p = x * $signed({1'b0, g});
        ref_y = (p + 33'sd128) >>> 8;
    end
endfunction

// 打一拍样本，等 o_valid
task step;
    input [6:0]  v;
    input [23:0] a;
    input [23:0] b;
    input        chk;                // 1 = 与参考模型比对
    input [8:0]  g;
    begin
        @(negedge clk);
        vol = v; l_in = a; r_in = b; valid = 1'b1;
        @(negedge clk);
        valid = 1'b0;
        // 等输出
        while (!o_valid) @(negedge clk);
        cases = cases + 1;
        if (chk) begin
            if (l_out !== ref_y($signed(a), g) || r_out !== ref_y($signed(b), g)) begin
                errs = errs + 1;
                if (errs <= 12)
                    $display("MISMATCH vol=%0d in=(%0d,%0d) got=(%0d,%0d) exp=(%0d,%0d)",
                             v, $signed(a), $signed(b), $signed(l_out), $signed(r_out),
                             ref_y($signed(a), g), ref_y($signed(b), g));
            end
        end
    end
endtask

integer i, v, g, k;
reg signed [23:0] x, y, prev;

initial begin
    repeat (4) @(posedge clk);
    rst = 0;
    repeat (2) @(posedge clk);

    //--------------------------------------------------------------- ① vol=100 逐位直通
    $display("--- A. vol=100 逐位直通 ---");
    for (i = 0; i < 400; i = i + 1) begin
        x = $random;
        step(7'd100, x, ~x, 1'b1, ref_g(7'd100));
        cases = cases + 1;
        if (l_out !== x) begin
            errs = errs + 1;
            if (errs <= 12) $display("  PASSTHRU FAIL in=%0d out=%0d", $signed(x), $signed(l_out));
        end
    end
    // 端点
    for (i = 0; i < 4; i = i + 1) begin
        case (i)
            0: x = 24'sh7FFFFF;
            1: x = 24'sh800000;
            2: x = 24'sd1;
            default: x = -24'sd1;
        endcase
        step(7'd100, x, x, 1'b1, ref_g(7'd100));
        if (l_out !== x) begin
            errs = errs + 1;
            $display("  PASSTHRU EDGE FAIL in=%0d out=%0d", $signed(x), $signed(l_out));
        end
    end
    if (ref_g(7'd100) != 9'd256) begin
        errs = errs + 1;
        $display("  g9(100)=%0d 应为 256（单位增益）", ref_g(7'd100));
    end

    //--------------------------------------------------------------- ② vol=0 恒 0
    $display("--- B. vol=0 输出恒 0 ---");
    for (i = 0; i < 50; i = i + 1) begin
        x = $random;
        step(7'd0, x, x, 1'b1, ref_g(7'd0));
        if (l_out !== 24'sd0) begin
            errs = errs + 1;
            $display("  MUTE FAIL in=%0d out=%0d", $signed(x), $signed(l_out));
        end
    end

    //--------------------------------------------------------------- ③ 全档位 × 随机样本
    $display("--- C. 0..100 每档 × 200 个随机样本 ---");
    for (v = 0; v <= 100; v = v + 5) begin
        g = ref_g(v[6:0]);
        for (i = 0; i < 200; i = i + 1) begin
            x = $random;
            step(v[6:0], x, $random, 1'b1, g);
        end
    end

    //--------------------------------------------------------------- ④ 单调性
    $display("--- D. 单调性：vol 增大，|out| 不减 ---");
    x = 24'sd8000000;            // 大正样本
    prev = 0;
    for (v = 0; v <= 100; v = v + 5) begin
        step(v[6:0], x, 24'sd0, 1'b0, 9'd0);
        y = l_out;
        if (y < prev) begin
            errs = errs + 1;
            $display("  MONO FAIL vol=%0d out=%0d < prev=%0d", v, $signed(y), $signed(prev));
        end
        prev = y;
    end

    $display("--------------------------------------------------");
    $display("cases = %0d   errors = %0d", cases, errs);
    if (errs == 0) $display("ALL PASS: 音量模块功能与音质指标全过");
    else           $display("FAIL");
    $finish;
end

endmodule
