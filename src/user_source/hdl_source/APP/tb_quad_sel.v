//==============================================================================
//  tb_quad_sel.v —— 四宫格坐标适配器单元测试
//==============================================================================
//  iverilog -g2012 -o qs.vvp -s tb_quad_sel tb_quad_sel.v quad_sel.v && vvp qs.vvp
//
//  布局（与 doc/osd_file_page_proto.html 一致）：
//      格0 x 208..367   格1 x 384..543        列间距 16
//      格2/3 y 260..379
//==============================================================================
`timescale 1ns/1ps

module tb_quad_sel;

localparam [20:0] SBASE = 21'h100000;      // 1048576
localparam [20:0] STEP  = 21'd16384;

reg  [9:0] x = 0, y = 0;
reg        en = 0;
wire [9:0] ox, oy;
wire [20:0] base;
wire in;
wire [1:0] region;

quad_sel #(
    .X0(10'd208), .Y0(10'd100), .COL_PITCH(10'd176), .ROW_PITCH(10'd160),
    .CW(10'd160), .CH(10'd120),
    .SLOT_BASE(SBASE), .SLOT_SHIFT(14)
) dut (
    .I_x(x), .I_y(y), .I_en(en),
    .O_x(ox), .O_y(oy), .O_base(base), .O_in(in), .O_region(region)
);

integer errors = 0, ncase = 0;
task chk;
    input cond;
    input [255:0] msg;
    begin
        ncase = ncase + 1;
        if (!cond) begin errors = errors + 1; $display("  FAIL [%0d] %0s", ncase, msg); end
    end
endtask

task at;
    input [9:0] px, py;
    begin x = px; y = py; #1; end
endtask

initial begin
    en = 1;

    //------------------------------------------------------------------
    $display("\n[T1] cell centers");
    at(288, 160);                                  // cell 0 center
    chk(in && region == 2'd0 && ox == 10'd80 && oy == 10'd60, "T1a cell0 center");
    chk(base == SBASE,                                        "T1a cell0 base");

    at(464, 160);                                  // cell 1 center = 384+80
    chk(in && region == 2'd1 && ox == 10'd80 && oy == 10'd60, "T1b cell1 center");
    chk(base == SBASE + STEP,                                 "T1b cell1 base");

    at(288, 320);                                  // cell 2 center
    chk(in && region == 2'd2 && ox == 10'd80 && oy == 10'd60, "T1c cell2 center");
    chk(base == SBASE + 2*STEP,                               "T1c cell2 base");

    at(464, 320);                                  // cell 3 center
    chk(in && region == 2'd3 && ox == 10'd80 && oy == 10'd60, "T1d cell3 center");
    chk(base == SBASE + 3*STEP,                               "T1d cell3 base");

    //------------------------------------------------------------------
    $display("[T2] corners");
    at(208, 100);
    chk(in && region == 2'd0 && ox == 10'd0 && oy == 10'd0,     "T2a cell0 TL");
    at(367, 219);
    chk(in && region == 2'd0 && ox == 10'd159 && oy == 10'd119, "T2b cell0 BR");
    at(543, 379);
    chk(in && region == 2'd3 && ox == 10'd159 && oy == 10'd119, "T2c cell3 BR");

    //------------------------------------------------------------------
    $display("[T3] gaps must miss");
    at(375, 160);   chk(!in, "T3a column gap");
    at(288, 240);   chk(!in, "T3b row gap (name area)");
    at(375, 240);   chk(!in, "T3c cross point");

    //------------------------------------------------------------------
    $display("[T4] screen edges");
    at(0, 0);       chk(!in, "T4a outside TL");
    at(639, 479);   chk(!in, "T4b outside BR");
    at(207, 160);   chk(!in, "T4c cell0 left-1");
    at(368, 160);   chk(!in, "T4d cell0 right+1 (=gap)");
    at(383, 160);   chk(!in, "T4e gap last col");
    at(384, 160);   chk(in && region == 2'd1 && ox == 10'd0, "T4f cell1 left edge");

    //------------------------------------------------------------------
    $display("[T5] en=0");
    en = 0; at(288, 160);
    chk(!in, "T5 en=0");

    //------------------------------------------------------------------
    $display("\n========================================");
    if (errors == 0) $display("  ALL PASS  (%0d checks)", ncase);
    else             $display("  FAIL: %0d / %0d", errors, ncase);
    $display("========================================\n");
    $finish;
end

endmodule
