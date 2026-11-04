`timescale 1ns/1ps
//==============================================================================
//  tb_widget_slider_equiv.v —— 滑条/旋钮几何的**穷举等价**回归
//==============================================================================
//  【为什么单独做】2026-10-29 把 osd_widget 的几何从"相对坐标 + w/h"
//  改成"绝对坐标 + 边距"：
//      dxm = min(lx, w−1−lx)      ←  w−1−lx  ⇒  min(lx, lxr)      lxr = gx1−1−px
//      dym = min(ly, h−1−ly)      ←           ⇒  min(ly, lyb)      lyb = gy1−1−py
//      trk 右端 lx ≤ w−5          ←           ⇒  px ≤ hix          hix = (w>9)? gx1−5 : gx0+4
//      vx  = clamp(v0, 4, w−5)    ←           ⇒  vxa = clamp(x0+v0, lox, hix)
//      kdx = |lx − vx|            ←           ⇒  |px − vxa|
//      kdy = |ly − cy|, cy = h>>1 ←           ⇒  |py − cya|, cya = (y0+y1)>>1
//      edge1/edge2 的右/下边框    ←           ⇒  lxr/lyb == 小常数
//
//  `tb_corner_equiv` 只覆盖 cut/arc/arc2（dxm/dym 的**下游**），
//  `tb_osd_style` 只探 14 个点、且是 Solid 样式 —— **滑条/旋钮完全没人测**。
//  这个台把那段几何单独拎出来穷举。
//
//  【怎么保证"测的是真 RTL"】直接层次化读 dut 内部信号，参考值用**旧公式**
//  独立算出（不抄新 RTL）。代入 gx0=gy0=0、gx1=w、gy1=h、px=lx、py=ly，
//  正是 tb_osd_style / tb_corner_equiv 的驱动方式，所以两个台测的是同一件事。
//
//  用法：iverilog -g2012 -o s.vvp -s tb_widget_slider_equiv tb_widget_slider_equiv.v osd_widget.v
//        vvp s.vvp
//==============================================================================
module tb_widget_slider_equiv;

reg  [3:0]  wtype, wstyle, wrad;
reg  [1:0]  wmask;
reg  [9:0]  v0, lx, ly, w, h;
wire        on;
wire [3:0]  lv;
wire [23:0] rgb;

osd_widget dut (
    .wtype(wtype), .wstyle(wstyle), .wrad(wrad), .wmask(wmask), .v0(v0),
    .lx(lx), .ly(ly), .w(w), .h(h),
    .px(lx), .py(ly), .gx0(10'd0), .gx1(w), .gy0(9'd0), .gy1(h),
    .on(on), .o_level(lv), .o_rgb(rgb)
);

//------------------------------------------------------------------------------
//  参考：**改动前**的公式（原样抄旧实现，不是抄新 RTL）
//------------------------------------------------------------------------------
reg        r_dxm_ok, r_lyb_ok;
reg [9:0]  r_dxm, r_dym;
reg        r_edge1, r_edge2, r_trk, r_fil, r_kout, r_kin, r_knb, r_knb_e;
reg [9:0]  r_kdx, r_kdy;

task ref_geom;
    input [9:0] ww, hh, xx, yy, vv;
    reg [9:0] cy_, tx1_, vx_, lxr_, lyb_;
    reg       trk_;
    begin
        // --- 旧写法 ---
        r_dxm  = (xx < (ww - 10'd1 - xx)) ? xx : (ww - 10'd1 - xx);
        r_dym  = (yy < (hh - 10'd1 - yy)) ? yy : (hh - 10'd1 - yy);
        r_edge1 = (xx == 10'd0) || (yy == 10'd0) ||
                  (xx == ww - 10'd1) || (yy == hh - 10'd1);
        r_edge2 = (xx < 10'd2) || (yy < 10'd2) ||
                  (xx >= ww - 10'd2) || (yy >= hh - 10'd2);
        cy_   = hh >> 1;
        tx1_  = (ww > 10'd9) ? (ww - 10'd5) : 10'd4;
        trk_  = ((yy + 10'd2) >= cy_) && (yy < (cy_ + 10'd2)) &&
                (xx >= 10'd4) && (xx <= tx1_);
        vx_   = (vv > tx1_) ? tx1_ : ((vv < 10'd4) ? 10'd4 : vv);
        r_trk  = trk_;
        r_fil  = trk_ && (xx <= vx_) && (vv != 10'd0);
        r_kdx  = (xx > vx_) ? (xx - vx_) : (vx_ - xx);
        r_kdy  = (yy > cy_) ? (yy - cy_) : (cy_ - yy);
        r_kout = (r_kdx > 10'd6) || (r_kdy > 10'd6) || ((r_kdx + r_kdy) > 10'd9);
        r_kin  = (r_kdx <= 10'd4) && (r_kdy <= 10'd4) && ((r_kdx + r_kdy) <= 10'd6);
        r_knb  = !r_kout && (hh >= 10'd10);
        r_knb_e= r_knb && !r_kin;
        // 只为消除 lint 未用告警
        lxr_ = 10'd0; lyb_ = 10'd0;
        r_dxm_ok = (lxr_ == 10'd0); r_lyb_ok = (lyb_ == 10'd0);
    end
endtask

integer fails = 0, n = 0;
reg [15:0] got16, exp16;

task ck;
    input [8*10:1] nm;
    input [15:0]   g, e;
    begin
        n = n + 1;
        if (g !== e) begin
            fails = fails + 1;
            if (fails <= 12)
                $display("  *** FAIL %0s  w=%0d h=%0d lx=%0d ly=%0d v0=%0d : got %0d exp %0d",
                         nm, w, h, lx, ly, v0, g, e);
        end
    end
endtask

integer wi, hi, xi, yi, vi;
reg [9:0] vset [0:8];

initial begin
    wtype = 4'd7; wstyle = 4'd6; wrad = 4'd0; wmask = 2'b00;

    $display("");
    $display("==================================================================");
    $display("  滑条/旋钮几何穷举等价（绝对坐标改写 vs 旧公式）");
    $display("==================================================================");

    for (wi = 1; wi <= 24; wi = wi + 1) begin
        for (hi = 1; hi <= 24; hi = hi + 1) begin
            w = wi[9:0]; h = hi[9:0];
            for (xi = 0; xi < wi; xi = xi + 1) begin
                lx = xi[9:0];
                for (yi = 0; yi < hi; yi = yi + 1) begin
                    ly = yi[9:0];
                    // v0 取：0 / 边界 / 轨道两端附近 / 中间 / 越界
                    for (vi = 0; vi < 8; vi = vi + 1) begin
                        case (vi)
                            0: v0 = 10'd0;
                            1: v0 = 10'd1;
                            2: v0 = 10'd4;
                            3: v0 = (wi > 10) ? (wi[9:0] - 10'd5) : 10'd4;
                            4: v0 = (wi > 10) ? (wi[9:0] - 10'd6) : 10'd3;
                            5: v0 = {2'b00, wi[9:0]} >> 1;
                            6: v0 = (wi > 1) ? (wi[9:0] - 10'd1) : 10'd1;
                            default: v0 = wi[9:0] + 10'd3;
                        endcase
                        #1;
                        ref_geom(w, h, lx, ly, v0);
                        ck("dxm",  dut.dxm,   r_dxm);
                        ck("dym",  dut.dym,   r_dym);
                        ck("edge1",dut.edge1, r_edge1);
                        ck("edge2",dut.edge2, r_edge2);
                        ck("trk",  dut.trk,   r_trk);
                        ck("fil",  dut.fil,   r_fil);
                        ck("kdx",  dut.kdx,   r_kdx);
                        ck("kdy",  dut.kdy,   r_kdy);
                        ck("kout", dut.kout,  r_kout);
                        ck("kin",  dut.kin,   r_kin);
                        ck("knb",  dut.knb,   r_knb);
                        ck("knb_e",dut.knb_e, r_knb_e);
                    end
                end
            end
        end
    end

    $display("------------------------------------------------------------------");
    $display("  比较 %0d 项，失败 %0d", n, fails);
    if (fails == 0) $display("  ALL PASS  —— 滑条/旋钮几何逐位等价");
    else            $display("  FAILED");
    $display("==================================================================");
    $finish;
end

endmodule
