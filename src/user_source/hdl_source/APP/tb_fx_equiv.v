`timescale 1ns/1ps
//==============================================================================
//  tb_fx_equiv.v —— viewer_fx 加两级流水前后的【等价】验证
//==============================================================================
//  【为什么必须验】改的是动画几何（pos/size/k/alpha）。这些量直接决定画面，
//  差一个像素就是"过渡时图片跳一下"。断言抓不到，只能逐值对比。
//
//  【做法】旧版（viewer_fx_ref）和新版（viewer_fx）同激励并行跑：
//    · 每个 (fx_id, dir, t) 组合发一次 upd
//    · 等 400 拍（新版应晚 2 拍落定，除法器要 24 步，cfg_wait 120 拍）
//    · 比较 pos_a/size_a/k_a/pos_b/size_b/k_b/alpha_b 全部 7 个输出
//
//  【覆盖】fx_id 0..4 × dir 0/1 × t 全行程（0..T_END 含端点）+ T_END 之后
//==============================================================================
module tb_fx_equiv;

localparam integer T_END_F = 60;
localparam integer T_END_S = 180;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg  [2:0]  fx_id = 0;
reg  [11:0] t     = 0;
reg         upd   = 0;
reg         dir   = 0;

wire [31:0] n_pa, n_sa, n_pb, n_sb;
wire [39:0] n_ka, n_kb;
wire [7:0]  n_al;
wire [31:0] r_pa, r_sa, r_pb, r_sb;
wire [39:0] r_ka, r_kb;
wire [7:0]  r_al;

viewer_fx #(.IMG_W(640), .IMG_H(480), .SRC_W(160), .SRC_H(240),
            .T_END_F(T_END_F), .T_END_S(T_END_S),
            // ★ 2026-10-27：本台验的是"乘法合并优化"的逐值等价，
            //   fx3 偶数化是后来独立的功能改动 ⇒ 这里关掉，和旧版对齐。
            .FX3_EVEN(0)) u_new (
    .clk(clk), .rst(rst), .fx_id(fx_id), .t(t), .upd(upd), .dir(dir),
    .pos_a(n_pa), .size_a(n_sa), .k_a(n_ka),
    .pos_b(n_pb), .size_b(n_sb), .k_b(n_kb), .alpha_b(n_al)
);

viewer_fx_ref #(.IMG_W(640), .IMG_H(480), .SRC_W(160), .SRC_H(240),
                .T_END_F(T_END_F), .T_END_S(T_END_S)) u_ref (
    .clk(clk), .rst(rst), .fx_id(fx_id), .t(t), .upd(upd), .dir(dir),
    .pos_a(r_pa), .size_a(r_sa), .k_a(r_ka),
    .pos_b(r_pb), .size_b(r_sb), .k_b(r_kb), .alpha_b(r_al)
);

integer n_case = 0, n_bad = 0;
integer i, d, tt, nz;

task do_case;
    input [2:0]  fid;
    input        dr;
    input [11:0] tv;
    begin
        fx_id = fid; dir = dr; t = tv;
        @(negedge clk); upd = 1'b1;
        @(negedge clk); upd = 1'b0;
        repeat (400) @(posedge clk);
        n_case = n_case + 1;
        if ((n_pa !== r_pa) || (n_sa !== r_sa) || (n_ka !== r_ka) ||
            (n_pb !== r_pb) || (n_sb !== r_sb) || (n_kb !== r_kb) ||
            (n_al !== r_al)) begin
            n_bad = n_bad + 1;
            if (n_bad <= 10)
                $display("   DIFF fx=%0d dir=%0d t=%0d\n        new pa=%h sa=%h ka=%h pb=%h sb=%h kb=%h al=%h\n        ref pa=%h sa=%h ka=%h pb=%h sb=%h kb=%h al=%h",
                         fid, dr, tv,
                         n_pa, n_sa, n_ka, n_pb, n_sb, n_kb, n_al,
                         r_pa, r_sa, r_ka, r_pb, r_sb, r_kb, r_al);
        end
    end
endtask

initial begin
    $display("======================================================");
    $display("  tb_fx_equiv : viewer_fx 两级流水前后等价");
    $display("======================================================");
    rst = 1; repeat (10) @(posedge clk); rst = 0; repeat (10) @(posedge clk);

    for (i = 0; i < 5; i = i + 1) begin
        for (d = 0; d < 2; d = d + 1) begin
            // fx0/1/2 用 T_END_F=60，fx3/4 用 T_END_S=180；两端各覆盖 0 和 T_END 及越界
            if (i < 3) begin
                for (tt = 0; tt <= 60; tt = tt + 3)  do_case(i[2:0], d[0], tt[11:0]);
                do_case(i[2:0], d[0], 12'd61);
                do_case(i[2:0], d[0], 12'd200);
            end else begin
                for (tt = 0; tt <= 180; tt = tt + 9) do_case(i[2:0], d[0], tt[11:0]);
                do_case(i[2:0], d[0], 12'd181);
                do_case(i[2:0], d[0], 12'd400);
            end
        end
    end

    $display("\n======================================");
    $display("  比对 %0d 组 (fx_id, dir, t)", n_case);
    if (n_bad == 0) $display("  ALL PASS  (7 个输出全部逐值一致)");
    else            $display("  FAIL      (%0d 组不同)", n_bad);
    $display("======================================");
    $finish;
end

endmodule
