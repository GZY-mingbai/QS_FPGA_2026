`timescale 1ns/1ps
//==============================================================================
//  tb_cellmem_equiv.v —— osd_cellmem 读口等价性专项（★ 2026-10-29 [S1]）
//==============================================================================
//  背景：读口从「39 位 × NSLOT 选 1 的组合大 mux」改成「LUTRAM 异步读 + 打一拍」。
//        这个改动**一行像素语义都不该变**，所以本台只守一件事：
//              rd_geo / rd_attr 与"寄存器阵列按 rd_idx 取值"逐拍逐位完全一致，
//              且 ce=0 时严格保持。
//
//  用法（工作目录 = 本文件所在目录）：
//      iverilog -g2012 -s tb_cellmem_equiv -o /tmp/cm.vvp \
//          tb_cellmem_equiv.v osd_cellmem.v && vvp /tmp/cm.vvp
//
//  覆盖：
//    P  复位后全 0 读回（NSLOT 个槽逐个查）
//    S  逐槽写入唯一值 → 逐槽读回（守 LUTRAM 地址译码 / 写使能门控）
//    R  3000 拍随机写读（wr_idx 覆盖全槽、wr_geo 随机、rd_idx 随机）
//       —— 每拍两条判据：① 与影子模型一致 ② 与 DUT 自己的 geo_hi/geo_lo 一致
//    H  命中网络（en=1 的槽必须命中且取最高索引）
//    C  ce=0 背压停顿：地址乱动，输出必须一位不变
//==============================================================================
module tb_cellmem_equiv;

localparam integer NSLOT = 8;           // 与 App.v 里 osd_top 的实际例化一致（[LUT-2]）

reg clk = 0;
always #5 clk = ~clk;

reg         rst = 1;
reg         ce  = 1;
reg         wr_en = 0;
reg  [4:0]  wr_idx = 0;
reg  [4:0]  rd_idx = 0;
reg  [38:0] wr_geo = 39'd0;
reg  [23:0] wr_attr = 24'd0;
reg  [9:0]  px = 10'd0;
reg  [8:0]  py = 9'd0;

wire        hit_v;
wire [4:0]  hit_idx;
wire [38:0] rd_geo;
wire [23:0] rd_attr;

osd_cellmem #(.NSLOT(NSLOT)) dut (
    .clk(clk), .rst(rst), .ce(ce),
    .wr_en(wr_en), .wr_idx(wr_idx), .wr_geo(wr_geo), .wr_attr(wr_attr),
    .px(px), .py(py), .hit_v(hit_v), .hit_idx(hit_idx),
    .rd_idx(rd_idx), .rd_geo(rd_geo), .rd_attr(rd_attr)
);

//------------------------------------------------------------------------------
//  检查计数器
//------------------------------------------------------------------------------
integer checks = 0;
integer fails  = 0;

task chk;
    input [8*72:1] name;
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            fails = fails + 1;
        end
    end
endtask

//------------------------------------------------------------------------------
//  随机数：一律走 ridx()（0..NSLOT-1），避免 % 出的负数越界
//------------------------------------------------------------------------------
integer seed = 32'h1234_5678;

function [4:0] ridx;
    input integer dummy;
    integer r;
    begin
        r = $random(seed) % NSLOT;
        if (r < 0) r = -r;
        ridx = r[4:0];
    end
endfunction

function integer roll;
    input integer modn;
    integer r;
    begin
        r = $random(seed) % modn;
        if (r < 0) r = -r;
        roll = r;
    end
endfunction

//------------------------------------------------------------------------------
//  影子模型（与 DUT 同拍、同一份写口）
//------------------------------------------------------------------------------
reg [38:0] mgeo  [0:NSLOT-1];
reg [23:0] mattr [0:NSLOT-1];

integer mi;
initial
    for (mi = 0; mi < NSLOT; mi = mi + 1) begin
        mgeo[mi]  = 39'd0;
        mattr[mi] = 24'd0;
    end

always @(posedge clk) begin
    if (wr_en) begin
        mgeo[wr_idx]  <= wr_geo;
        mattr[wr_idx] <= wr_attr;
    end
end

//------------------------------------------------------------------------------
//  每拍的期望值（在 posedge 抓"阵列的旧值"，与 rd_geo 的采样时刻完全一致）
//------------------------------------------------------------------------------
reg [38:0] exp_geo;
reg [23:0] exp_attr;
reg [4:0]  idx_q;
reg        ce_q;
reg        wr_same;                     // 本拍写的就是 idx_q ⇒ 不能用 DUT 阵列当参考

always @(posedge clk) begin
    exp_geo  <= mgeo[rd_idx];
    exp_attr <= mattr[rd_idx];
    idx_q    <= rd_idx;
    ce_q     <= ce;
    wr_same  <= wr_en && (wr_idx == rd_idx);
end

// 停顿判据用：上一个 negedge 看到的输出
reg [38:0] geo_prev;
reg [23:0] attr_prev;
reg        have_prev = 0;

//------------------------------------------------------------------------------
//  单拍写
//------------------------------------------------------------------------------
task wr1;
    input [4:0]  idx;
    input [38:0] geo;
    input [23:0] attr;
    begin
        @(negedge clk);
        wr_en = 1'b1; wr_idx = idx; wr_geo = geo; wr_attr = attr;
        @(negedge clk);
        wr_en = 1'b0;
    end
endtask

//------------------------------------------------------------------------------
//  R 段：自由运行的随机激励 + 检查（同一 negedge 块，一个 negedge 一个 posedge）
//------------------------------------------------------------------------------
integer i, n;
reg [38:0] v;
reg     r_run = 1'b0;
integer rn = 0;

always @(negedge clk) begin
    if (r_run) begin
        // 1) 检查上一个 posedge 的结果
        if (rn > 0) begin
            if (ce_q) begin
                checks = checks + 2;
                if (rd_geo !== exp_geo || rd_attr !== exp_attr) begin
                    fails = fails + 1;
                    $display("  *** FAIL  R cycle %0d idx=%0d : geo got %h exp %h | attr got %h exp %h",
                             rn, idx_q, rd_geo, exp_geo, rd_attr, exp_attr);
                end
                // DUT 自己的寄存器阵列作第二参考（本拍没写这个槽时才成立）
                if (!wr_same) begin
                    if (rd_geo !== {dut.geo_hi[idx_q], dut.geo_lo[idx_q]}) begin
                        fails = fails + 1;
                        $display("  *** FAIL  R cycle %0d idx=%0d : geo != {geo_hi,geo_lo} (%h vs %h)",
                                 rn, idx_q, rd_geo, {dut.geo_hi[idx_q], dut.geo_lo[idx_q]});
                    end
                end
            end else begin
                checks = checks + 1;
                if (have_prev && (rd_geo !== geo_prev || rd_attr !== attr_prev)) begin
                    fails = fails + 1;
                    $display("  *** FAIL  R cycle %0d stall : geo %h -> %h, attr %h -> %h",
                             rn, geo_prev, rd_geo, attr_prev, rd_attr);
                end
            end
            geo_prev = rd_geo; attr_prev = rd_attr; have_prev = 1'b1;
        end

        // 2) 给下一个 posedge 下激励
        rd_idx  = ridx(0);
        ce      = (roll(8) != 0);              // 约 1/8 拍停顿
        wr_en   = (roll(3) == 0);              // 约 1/3 拍写
        wr_idx  = ridx(0);
        v       = {$random(seed), $random(seed)};
        v[38]   = (roll(4) != 0);              // 多数 en=1
        wr_geo  = v;
        wr_attr = {$random(seed)};

        rn = rn + 1;
        if (rn >= 3001) begin
            r_run = 1'b0;
            ce = 1'b1; wr_en = 1'b0;
        end
    end
end

//------------------------------------------------------------------------------
//  主激励
//------------------------------------------------------------------------------

initial begin
    //------------------------------------------------------------------
    // P：复位后全 0
    //------------------------------------------------------------------
    @(negedge clk); rst = 1'b1;
    repeat (4) @(negedge clk);
    @(negedge clk); rst = 1'b0;

    $display("--- P. 复位后全 0 ---");
    for (i = 0; i < NSLOT; i = i + 1) begin
        @(negedge clk); rd_idx = i[4:0];
        @(posedge clk); @(negedge clk);
        chk("P geo all zero", rd_geo, 39'd0);
        chk("P attr all zero", rd_attr, 24'd0);
    end

    //------------------------------------------------------------------
    // S：逐槽写唯一值 → 逐槽读回
    //------------------------------------------------------------------
    $display("--- S. 逐槽写读（地址译码） ---");
    for (i = 0; i < NSLOT; i = i + 1)
        wr1(i[4:0], {25'h1AB_0000 + i, 14'h3F00 + i[13:0]},
                    {8'hA0 + i[7:0], 8'h5A, 8'h00 + i[7:0]});

    for (i = 0; i < NSLOT; i = i + 1) begin
        @(negedge clk); rd_idx = i[4:0];
        @(posedge clk); @(negedge clk);
        chk("S geo slot", rd_geo, {25'h1AB_0000 + i, 14'h3F00 + i[13:0]});
        chk("S attr slot", rd_attr, {8'hA0 + i[7:0], 8'h5A, 8'h00 + i[7:0]});
    end

    //------------------------------------------------------------------
    // R：3000 拍随机（写读同拍、ce 随机停顿）
    //   驱动与检查**放在同一个 negedge 块**里（见下面 always），这样每个 negedge
    //   之间严格只有一个 posedge。先前用 @(negedge)...@(negedge) 手写等待时，
    //   循环回绕会多吃一个 posedge，上一拍的 ce 会在两次检查之间再推进一格，
    //   于是停顿判据假报（实测 5 条假 FAIL）。
    //------------------------------------------------------------------
    $display("--- R. 3000 拍随机等价 ---");
    wr_en = 1'b0;
    r_run = 1'b1;
    wait (r_run == 1'b0);
    wr_en = 1'b0; ce = 1'b1;

    //------------------------------------------------------------------
    // H：命中网络（en=1 的最高索引必须赢）
    //------------------------------------------------------------------
    $display("--- H. 命中网络 ---");
    for (i = 0; i < NSLOT; i = i + 1)
        wr1(i[4:0], 39'd0, 24'd0);
    wr1(5'd3, {1'b1, 10'd200, 9'd200, 10'd0, 9'd0}, 24'hC0_00_00);
    // ★ 2026-10-31：原来写死槽 9（NSLOT=21 时成立）；现在 NSLOT=8 ⇒ 用最高槽 NSLOT-1
    wr1(NSLOT-1, {1'b1, 10'd200, 9'd200, 10'd0, 9'd0}, 24'hD0_00_00);
    px = 10'd100; py = 9'd100;
    #1;
    chk("H hit_v",       hit_v,   1);
    chk("H hit_idx top", hit_idx, NSLOT-1);
    px = 10'd250;                 // x 超界
    #1;
    chk("H miss", hit_v, 0);
    px = 10'd0; py = 9'd0;

    //------------------------------------------------------------------
    // C：ce=0 停顿（地址乱动，输出必须不动）
    //------------------------------------------------------------------
    $display("--- C. ce=0 背压 ---");
    @(negedge clk); rd_idx = 5'd5;
    @(posedge clk); @(negedge clk);
    geo_prev = rd_geo; attr_prev = rd_attr;
    ce = 1'b0;
    for (n = 0; n < 32; n = n + 1) begin
        @(negedge clk);
        rd_idx = ridx(0);
        @(posedge clk); @(negedge clk);
        checks = checks + 1;
        if (rd_geo !== geo_prev || rd_attr !== attr_prev) begin
            fails = fails + 1;
            $display("  *** FAIL  C stall idx=%0d : geo %h -> %h", rd_idx, geo_prev, rd_geo);
        end
    end
    ce = 1'b1;

    //------------------------------------------------------------------
    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  %0d FAIL out of %0d checks", fails, checks);
    $display("======================================================");
    $finish;
end

endmodule
