`timescale 1ns/1ps
//==============================================================================
//  tb_corner_equiv.v —— osd_widget 圆角判据的**穷举等价**回归
//==============================================================================
//  【为什么要它】
//    phy 报告显示圆角算术把 `rd_geo → pix_reg` 拉成 20 级 / 34.4ns（Fmax 28.7MHz），
//    要改成查表。查表是"重写实现"，必须先有一条**能 FAIL 的等价断言**守着，
//    否则改完只能靠肉眼。
//
//  【判据】把 `dut.cut / dut.arc / dut.arc2` 与**规格公式**（本文件里独立写的
//    参考模型，不是抄 RTL）逐位比对，穷举：
//        wrad 0..15 × w 2..20 × h 2..20 × lx 0..w-1 × ly 0..h-1
//    = 698896 组。
//
//  【规格】rq = min(rad,8)；dxm/dym = 到最近边的距离；inc = rq≠0 && dxm<rq && dym<rq
//        cut  = inc && (r-dx)²+(r-dy)² > r²
//        arc  = inc && !cut && (r-dx+1)²+(r-dy+1)² > r²
//        arc2 = inc && !cut && (r-dx+2)²+(r-dy+2)² > r²
//
//  【用法】iverilog -g2012 -o c.vvp -s tb_corner_equiv tb_corner_equiv.v osd_widget.v
//          vvp c.vvp
//==============================================================================
module tb_corner_equiv;

reg  [3:0]  wtype = 4'd0, wstyle = 4'd0, wrad = 4'd0;
reg  [1:0]  wmask = 2'd0;
reg  [9:0]  v0 = 10'd0, lx = 10'd0, ly = 10'd0, w = 10'd8, h = 10'd8;
wire        on;
wire [3:0]  lv;
wire [23:0] rgb;

osd_widget dut (
    .wtype(wtype), .wstyle(wstyle), .wrad(wrad), .wmask(wmask), .v0(v0),
    .lx(lx), .ly(ly), .w(w), .h(h),
    // ★ 2026-10-29：widget 新增绝对坐标输入。这里取 gx0=gy0=0、gx1=w、gy1=h、
    //   px=lx、py=ly ⇒ 与改动前逐位等价（lxr=w-1-lx、lyb=h-1-ly、cya=h>>1、
    //   hix=tx1、v0a=v0）。穷举台靠这组代入保持"测的是同一件事"。
    .px(lx), .py(ly), .gx0(10'd0), .gx1(w), .gy0(9'd0), .gy1(h),
    .on(on), .o_level(lv), .o_rgb(rgb)
);

//------------------------------------------------------------------------------
//  参考模型：规格公式（独立写，不抄 RTL）
//------------------------------------------------------------------------------
function [2:0] ref_corner;         // {cut, arc, arc2}
    input [3:0] rad;
    input [9:0] ww, hh, xx, yy;
    reg [9:0] dxm, dym;
    reg [3:0] rq, dxc, dyc, uu, vv;
    reg [8:0] d2, r2;
    reg       inc, ct;
    begin
        dxm = (xx < (ww - 10'd1 - xx)) ? xx : (ww - 10'd1 - xx);
        dym = (yy < (hh - 10'd1 - yy)) ? yy : (hh - 10'd1 - yy);
        rq  = (rad > 4'd8) ? 4'd8 : rad;
        dxc = (dxm < 10'd15) ? dxm[3:0] : 4'd15;
        dyc = (dym < 10'd15) ? dym[3:0] : 4'd15;
        uu  = rq - dxc;
        vv  = rq - dyc;
        d2  = uu * uu + vv * vv;
        r2  = rq * rq;
        inc = (rq != 4'd0) && (dxm < {6'd0, rq}) && (dym < {6'd0, rq});
        ct  = inc && (d2 > r2);
        // arc/arc2：把 (u,v) 各内移 1 / 2 格再比
        ref_corner = { ct,
                       inc && !ct && ((uu+4'd1)*(uu+4'd1) + (vv+4'd1)*(vv+4'd1) > r2),
                       inc && !ct && ((uu+4'd2)*(uu+4'd2) + (vv+4'd2)*(vv+4'd2) > r2) };
    end
endfunction

//------------------------------------------------------------------------------
// ★ 穷举范围可调：-DWHMAX=48 覆盖到 48×48（含 32×32 logo / 信息条大面板）。
//   默认 20 覆盖设计里所有实际出现的控件尺寸（最大 w=312,h=104 的弹层板
//   半径只有 6，切角永远落在 ≤8 的角区内，用 20×20 的窗口就穷尽了所有
//   (rq, dx, dy) 组合 —— 之所以还能再放大到 48，是为了证明这条推理本身没错）。
`ifndef WHMAX
 `define WHMAX 20
`endif

//------------------------------------------------------------------------------
integer fails = 0, checks = 0, n = 0;
reg [2:0] got, exp;

initial begin
    $display("");
    $display("==================================================================");
    $display("  圆角判据穷举等价：dut.cut/arc/arc2  vs  规格公式");
    $display("==================================================================");
    for (integer rad = 0; rad < 16; rad = rad + 1) begin
        for (integer ww = 2; ww <= `WHMAX; ww = ww + 1) begin
            for (integer hh = 2; hh <= `WHMAX; hh = hh + 1) begin
                for (integer xx = 0; xx < ww; xx = xx + 1) begin
                    for (integer yy = 0; yy < hh; yy = yy + 1) begin
                        wrad = rad[3:0]; w = ww[9:0]; h = hh[9:0];
                        lx = xx[9:0]; ly = yy[9:0];
                        #1;
                        got = {dut.cut, dut.arc, dut.arc2};
                        exp = ref_corner(rad[3:0], ww[9:0], hh[9:0], xx[9:0], yy[9:0]);
                        n = n + 1;
                        if (got !== exp) begin
                            if (fails < 10)
                                $display("  *** FAIL  rad=%0d w=%0d h=%0d lx=%0d ly=%0d : got=%b exp=%b",
                                         rad, ww, hh, xx, yy, got, exp);
                            fails = fails + 1;
                        end
                    end
                end
            end
        end
    end
    checks = 1;
    $display("");
    $display("  穷举 %0d 组", n);
    $display("==================================================================");
    if (fails == 0) $display("  ALL PASS  —— 圆角判据逐位等价");
    else            $display("  FAILED: %0d / %0d 组不一致", fails, n);
    $display("==================================================================");
    $finish;
end

endmodule
