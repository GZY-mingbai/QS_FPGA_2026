`timescale 1ns/1ps
//=============================================================================
//  tb_audio_mix.v —— 24bit 饱和叠加的边界断言
//=============================================================================
module tb_audio_mix;

reg clk = 0;
always #10 clk = ~clk;

reg         rst = 1;
reg         valid = 0;
reg  [23:0] mus_l = 0, mus_r = 0;
reg  signed [15:0] bep = 0;
reg         act = 0;
wire [23:0] out_l, out_r;
wire        o_valid;

audio_mix dut (
    .clk(clk), .rst(rst), .valid(valid),
    .mus_l(mus_l), .mus_r(mus_r),
    .bep(bep), .bep_act(act),
    .out_l(out_l), .out_r(out_r), .o_valid(o_valid)
);

integer fails = 0, checks = 0;
task chk24;
    input [8*40:1] name;
    input [23:0]   got;
    input [23:0]   exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %h, expect %h", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %h", name, got);
    end
endtask

task one;
    input [23:0] ml, mr;
    input signed [15:0] bp;
    input        a;
    begin
        @(negedge clk);
        mus_l = ml; mus_r = mr; bep = bp; act = a; valid = 1'b1;
        @(negedge clk);
        valid = 1'b0;
        repeat (2) @(posedge clk);
    end
endtask

initial begin
    $display("=========== audio_mix 饱和边界 ===========");
    rst = 1; repeat (5) @(posedge clk); rst = 0; repeat (3) @(posedge clk);

    // 提示音 16bit → 24bit 是左移 8 位，所以 bep=16'sd1 等于 +256
    one(24'h000000, 24'h000000, 16'sd1, 1'b1);
    chk24("bep 1  -> +256", out_l, 24'h000100);
    chk24("right same",     out_r, 24'h000100);

    one(24'h7FFFFF, 24'h7FFFFF, 16'sd1, 1'b1);
    chk24("pos saturate",   out_l, 24'h7FFFFF);

    one(24'h800000, 24'h800000, -16'sd1, 1'b1);
    chk24("neg saturate",   out_l, 24'h800000);

    one(24'h000100, 24'h000200, 16'sd2, 1'b1);   // 256 + 512 = 768 = 0x300
    chk24("normal add L",   out_l, 24'h000300);
    chk24("normal add R",   out_r, 24'h000400);

    one(24'h123456, 24'h654321, 16'sd0, 1'b0);   // 不激活 ⇒ 直通
    chk24("bypass L",       out_l, 24'h123456);
    chk24("bypass R",       out_r, 24'h654321);

    $display("======================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  %0d FAIL / %0d checks", fails, checks);
    $finish;
end

endmodule
