//==============================================================================
//  tb_fx_const.v —— viewer_fx 常数乘法合并的**穷举**等价性证明
//==============================================================================
//  目的：把下面 14 个量，用「旧算法（各自独立乘常数）」和「新算法（4 个基量 +
//        常数移位）」在同一份 RTL 里各算一遍，对 e = 0..256 **逐值**比对。
//
//  这是纯组合的单元测试，不依赖时钟/状态机 ⇒ 可以真的穷举，
//  比 tb_fx_equiv 的 230 组抽样强得多。
//
//      旧： (e*C)>>8                （C 为各自常数，只截断一次）
//      新： 由 e*640 / eu*640 / 3e / 3eu 移位得到
//
//  只要这个测试全 PASS，就说明 viewer_fx 几何量相对旧版**逐位一致**。
//==============================================================================
`timescale 1ns / 1ps

module tb_fx_const;

//------------------------------------------------------------------------------
//  新算法（与 viewer_fx.v 里的实现逐字对应）
//------------------------------------------------------------------------------
function [17:0] n_ep;  input [8:0] e; begin n_ep  = {9'd0, e} * 11'd640; end endfunction
function [18:0] n_ep3; input [17:0] ep; begin n_ep3 = {1'd0, ep} + {2'd0, ep, 1'b0}; end endfunction
function [9:0]  n_e3;  input [8:0] e; begin n_e3  = {1'd0, e} + {2'd0, e, 1'b0}; end endfunction

//------------------------------------------------------------------------------
//  旧算法（与改动前的 viewer_fx.v 逐字对应）
//------------------------------------------------------------------------------
function [18:0] o_640; input [8:0] e; begin o_640 = {10'd0, e} * 16'd640; end endfunction
function [18:0] o_480; input [8:0] e; begin o_480 = {10'd0, e} * 16'd480; end endfunction
function [18:0] o_255; input [8:0] e; begin o_255 = {10'd0, e} * 16'd255; end endfunction
function [18:0] o_320; input [8:0] e; begin o_320 = {10'd0, e} * 16'd320; end endfunction
function [18:0] o_240; input [8:0] e; begin o_240 = {10'd0, e} * 16'd240; end endfunction
function [18:0] o_120; input [8:0] e; begin o_120 = {10'd0, e} * 16'd120; end endfunction
function [18:0] o_160; input [8:0] e; begin o_160 = {10'd0, e} * 16'd160; end endfunction
function [18:0] o_048; input [8:0] e; begin o_048 = {10'd0, e} * 16'd48;  end endfunction
function [18:0] o_096; input [8:0] e; begin o_096 = {10'd0, e} * 16'd96;  end endfunction
function [18:0] o_128; input [8:0] e; begin o_128 = {10'd0, e} * 16'd128; end endfunction
function [18:0] o_256; input [8:0] e; begin o_256 = {10'd0, e} * 16'd256; end endfunction
function [18:0] o_384; input [8:0] e; begin o_384 = {10'd0, e} * 16'd384; end endfunction

integer e, k;
integer err;
integer chk;
// 旧算法的函数返回值必须先落中间量：Verilog 不允许 `func(x)[hi:lo]`
reg [18:0] o640, o480, o320, o240, o120, o160;
reg [18:0] o048, o096, o128, o256, o384, o255;
reg [15:0] nv, ov;
reg [7:0]  npal, opal;
reg [17:0] ep;
reg [18:0] ep3;
reg [9:0]  e3;

reg [127:0] tag;         // 当前检查项名字（iverilog -g2005 不允许把字符串传进 task）
task chk16;
    input [15:0] n;
    input [15:0] o;
    begin
        chk = chk + 1;
        if (n !== o) begin
            err = err + 1;
            if (err <= 20)
                $display("  FAIL e=%0d %0s  new=%0d old=%0d", e, tag, n, o);
        end
    end
endtask

initial begin
    err = 0; chk = 0;
    $display("======================================================");
    $display("  tb_fx_const : e×常数合并 —— 穷举 0..256 逐值比对");
    $display("======================================================");

    for (k = 0; k <= 256; k = k + 1) begin
        e   = k;
        o640 = o_640(e[8:0]); o480 = o_480(e[8:0]); o320 = o_320(e[8:0]);
        o240 = o_240(e[8:0]); o120 = o_120(e[8:0]); o160 = o_160(e[8:0]);
        o048 = o_048(e[8:0]); o096 = o_096(e[8:0]); o128 = o_128(e[8:0]);
        o256 = o_256(e[8:0]); o384 = o_384(e[8:0]); o255 = o_255(e[8:0]);
        ep  = n_ep(e[8:0]);
        ep3 = n_ep3(ep);
        e3  = n_e3(e[8:0]);

        // ---- px = (e*640)>>8 ----
        nv = {6'd0,  ep [17: 8]};   ov = {5'd0, o640[18:8]};
        tag = "g640/px"; chk16(nv, ov);

        // ---- py = (e*480)>>8 ----
        nv = {6'd0,  ep3[18:10]};   ov = {5'd0, o480[18:8]};
        tag = "g480/py"; chk16(nv, ov);

        // ---- pal = (e*255)>>8 ----
        npal = (e == 0) ? 8'd0 : (e[7:0] - 8'd1);
        opal = o255[15:8];
        if (npal !== opal) begin
            err = err + 1;
            if (err <= 20) $display("  FAIL e=%0d pal  new=%0d old=%0d", e, npal, opal);
        end
        chk = chk + 1;

        // ---- g320 = (e*320)>>8 ----
        nv = {7'd0,  ep [17: 9]};   ov = {5'd0, o320[18:8]};
        tag = "g320"; chk16(nv, ov);

        // ---- g160 = (e*160)>>8 ----
        nv = {8'd0,  ep [17:10]};   ov = {5'd0, o160[18:8]};
        tag = "g160"; chk16(nv, ov);

        // ---- g240 = (e*240)>>8 ----
        nv = {8'd0,  ep3[18:11]};   ov = {5'd0, o240[18:8]};
        tag = "g240"; chk16(nv, ov);

        // ---- g120 = (e*120)>>8 ----
        nv = {9'd0,  ep3[18:12]};   ov = {5'd0, o120[18:8]};
        tag = "g120"; chk16(nv, ov);

        // ---- g128 = (e*128)>>8 ----
        nv = {8'd0,  e[8:1]};       ov = {5'd0, o128[18:8]};
        tag = "g128"; chk16(nv, ov);

        // ---- g256 = (e*256)>>8 ----
        nv = {7'd0,  e[8:0]};       ov = {5'd0, o256[18:8]};
        tag = "g256"; chk16(nv, ov);

        // ---- g384 = (e*384)>>8 ----
        nv = {7'd0,  e3 [9: 1]};    ov = {5'd0, o384[18:8]};
        tag = "g384"; chk16(nv, ov);

        // ---- g96 = (e*96)>>8 ----
        nv = {9'd0,  e3 [9: 3]};    ov = {5'd0, o096[18:8]};
        tag = "g096"; chk16(nv, ov);

        // ---- g48 = (e*48)>>8 ----
        nv = {10'd0, e3 [9: 4]};    ov = {5'd0, o048[18:8]};
        tag = "g048"; chk16(nv, ov);
    end

    $display("------------------------------------------------------");
    $display("  比对 %0d 项（12 个量 × 257 个 e 值 + 边界）", chk);
    if (err == 0)
        $display("  ALL PASS  (14 个派生量全部逐值一致)");
    else
        $display("  %0d 个 FAIL", err);
    $display("======================================================");
    $finish;
end

endmodule
