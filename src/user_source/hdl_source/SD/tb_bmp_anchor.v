`timescale 1ns/1ps
//==============================================================================
//  tb_bmp_anchor.v —— 锚点几何断言台（zoom 时不动的那个点）
//==============================================================================
//  【为什么单独有这个台】
//    tb_bmp_loader_zoom 的 12 个用例锚点全在屏幕中心，验证不了
//    "cam = W/2 + (ax-320)·(step_fit-step)" 这条新公式。这里专测非中心锚点：
//
//      源 640×480，zoom=2（step=512），ax=200
//        ⇒ cam = 320 + (200-320)×(1024-512) = 260
//        ⇒ sx0 = 260 - 320×0.5 = 100
//        ⇒ 屏幕 (200,·) 必须显示源列 200（锚点不动）
//
//  【纪律】期望值按规格手算，不从 RTL 翻译（skills/osd-sim-verify #21/#22）。
//
//  运行：
//    iverilog -g2012 -o /tmp/ta.vvp -s tb_bmp_anchor tb_bmp_anchor.v \
//        bmp_view.v bmp_view_fill.v && vvp /tmp/ta.vvp | grep -aE "FAIL|ALL PASS"
//==============================================================================
module tb_bmp_anchor;

reg clk = 0;
always #10 clk = ~clk;

reg rst = 1;

reg         start = 0;
reg  [15:0] src_w = 640, src_h = 480;
reg  [1:0]  mode  = 2'd0;          // 640×480
reg         fmt   = 1'b1;          // 888，1px/字
reg  [20:0] dst_base = 21'd0;
reg  [10:0] dst_stride = 11'd0;
reg  [7:0]  zoom = 8'h20;          // 2.0×
reg  [9:0]  ax = 10'd200, ay = 10'd240;

reg         pix_vld;
reg  [23:0] pix;
wire        pix_rd;
wire [11:0] row_req, win_x0;
wire [12:0] win_w;
wire        stall = 1'b0;

wire        we;
wire [20:0] wa;
wire [31:0] wdata;
wire        busy, done;

bmp_view dut (
    .clk(clk), .rst(rst),
    .start(start),
    .src_w(src_w), .src_h(src_h),
    .mode(mode), .fmt(fmt),
    .dst_base(dst_base), .dst_stride(dst_stride),
    .zoom(zoom), .ax(ax), .ay(ay),
    .pix_vld(pix_vld), .pix(pix), .pix_rd(pix_rd),
    .row_req(row_req), .win_x0(win_x0), .win_w(win_w),
    .stall(stall),
    .we(we), .wa(wa), .wdata(wdata),
    .busy(busy), .done(done)
);

//------------------------------------------------------------------------------
//  源图：坐标编码（R=x/4, G=y/4, B={x[1:0],y[1:0],4'h0}）
//------------------------------------------------------------------------------
function [23:0] enc;
    input integer x, y;
    begin
        enc = {x[9:2], y[9:2], x[1:0], y[1:0], 4'h0};
    end
endfunction

reg  [11:0] srv_row = 12'hFFF;
reg         srv_on  = 1'b0;
reg  [13:0] srv_col = 14'd0;
reg         srv_rst = 1'b0;          // 每次 run 前强制复位从机（否则 row_req 与残留行号相同时不启动）

wire [23:0] srv_pix = enc(srv_col[11:0], srv_row);

always @(*) begin
    pix_vld = srv_on;
    pix     = srv_pix;
end

always @(posedge clk or posedge rst) begin
    if (rst || srv_rst) begin
        srv_on <= 1'b0; srv_col <= 14'd0; srv_row <= 12'hFFF;
    end else if (!srv_on) begin
        if ((row_req != srv_row) && (win_w != 13'd0)) begin
            srv_on  <= 1'b1;
            srv_row <= row_req;
            srv_col <= {2'd0, win_x0};
        end
    end else if (pix_rd) begin
        if (srv_col >= (win_x0 + win_w - 13'd1))
            srv_on <= 1'b0;
        else
            srv_col <= srv_col + 14'd1;
    end
end

//------------------------------------------------------------------------------
//  写缓冲 model（888：addr = base + y*640 + x，每字 {8'hFF,R,G,B}）
//------------------------------------------------------------------------------
localparam integer NPX = 640*480;
reg [7:0] mem_r [0:NPX-1];
reg [7:0] mem_g [0:NPX-1];
reg [7:0] mem_b [0:NPX-1];
reg       mem_v [0:NPX-1];
integer   i;

always @(posedge clk) begin
    if (we && (wa < NPX)) begin
        mem_r[wa] <= wdata[23:16];
        mem_g[wa] <= wdata[15:8];
        mem_b[wa] <= wdata[7:0];
        mem_v[wa] <= 1'b1;
    end
end

//------------------------------------------------------------------------------
//  检查
//------------------------------------------------------------------------------
integer fails = 0, checks = 0;
task chk;
    input        cond;
    input [511:0] name;
    begin
        checks = checks + 1;
        if (!cond) begin
            fails = fails + 1;
            $display("  FAIL  %0s", name);
        end else
            $display("  ok    %0s", name);
    end
endtask

// 从缓冲像素反解源坐标（enc 的逆运算）
function [11:0] dec_x;
    input [23:0] p;
    begin
        dec_x = {p[23:16], p[7:6]};      // R=x/4（8 位）+ x[1:0]
    end
endfunction
function [11:0] dec_y;
    input [23:0] p;
    begin
        dec_y = {p[15:8], p[5:4]};
    end
endfunction

// 读缓冲某点的源坐标
task get_src;
    input  integer ox, oy;
    output [11:0] sx, sy;
    output        valid;
    reg [23:0] p;
    begin
        p = {mem_r[oy*640+ox], mem_g[oy*640+ox], mem_b[oy*640+ox]};
        sx = dec_x(p);
        sy = dec_y(p);
        valid = mem_v[oy*640+ox];
    end
endtask

// 不变量：锚点 (ax,ay) 处必须显示源 (ax,ay)
task chk_anchor;
    input integer ox, oy;
    input [511:0] name;
    reg [11:0] sx, sy;
    reg        ok;
    begin
        get_src(ox, oy, sx, sy, ok);
        checks = checks + 1;
        if (!ok || (sx !== ox[11:0]) || (sy !== oy[11:0])) begin
            fails = fails + 1;
            $display("  FAIL  %0s  @(%0d,%0d) -> src(%0d,%0d)", name, ox, oy, sx, sy);
        end else
            $display("  ok    %0s  @(%0d,%0d) -> src(%0d,%0d)", name, ox, oy, sx, sy);
    end
endtask

// 不变量：竖直方向拖动时，锚点那一列整列都显示同一个源列
task chk_col_fixed;
    input integer ox, oy;
    input integer exp_x;
    input [511:0] name;
    reg [11:0] sx, sy;
    reg        ok;
    begin
        get_src(ox, oy, sx, sy, ok);
        checks = checks + 1;
        if (!ok || (sx !== exp_x[11:0])) begin
            fails = fails + 1;
            $display("  FAIL  %0s  @(%0d,%0d) src_x=%0d exp=%0d", name, ox, oy, sx, exp_x);
        end else
            $display("  ok    %0s  @(%0d,%0d) src_x=%0d", name, ox, oy, sx);
    end
endtask

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
initial begin
    for (i = 0; i < NPX; i = i + 1) begin
        mem_v[i] = 0; mem_r[i] = 0; mem_g[i] = 0; mem_b[i] = 0;
    end

    repeat (4) @(negedge clk);
    rst = 1'b0;
    repeat (4) @(negedge clk);

    //------------------------------------------------------------------
    $display("--- A. zoom=2.0, anchor=(200,240) ---");
    //------------------------------------------------------------------
    // 手算：step = 1024/2 = 512；cam_x = 320 + (200-320)*512/1024 = 260
    //       sx0 = 260 - 320*0.5 = 100；sy0 = 240 - 240*0.5 = 120
    //       屏幕 (200,240) → 源 (100 + 200*0.5, 120 + 240*0.5) = (200,240)
    rst = 1'b1; repeat (8) @(negedge clk); rst = 1'b0; repeat (8) @(negedge clk);
    @(negedge clk); srv_rst = 1'b1;
    @(negedge clk); srv_rst = 1'b0;
    @(negedge clk); start = 1'b1;
    @(negedge clk); start = 1'b0;
    wait (done);
    repeat (4) @(negedge clk);

    chk(done === 1'b0, "A0 run finished");
    chk_anchor(200, 240, "A1 anchor pixel stays put");
    chk_col_fixed(200, 0,   200, "A2 anchor column at row 0");
    chk_col_fixed(200, 479, 200, "A3 anchor column at row 479");
    chk_col_fixed(201, 240, 200, "A4 anchor+1 shares source col (2x)");
    chk_col_fixed(202, 240, 201, "A5 anchor+2 next source col");
    chk_col_fixed(199, 240, 199, "A6 anchor-1 prev source col");

    //------------------------------------------------------------------
    $display("--- B. zoom=4.0, 同一锚点 ---");
    //------------------------------------------------------------------
    // step=256；cam = 320 + (200-320)*768/1024 = 320 - 90 = 230
    // sx0 = 230 - 320*0.25 = 150；sy0 = 240 - 240*0.25 = 180
    // 屏幕 (200,240) → 源 (150 + 200*0.25, 180 + 240*0.25) = (200,240)
    for (i = 0; i < NPX; i = i + 1) mem_v[i] = 0;

    zoom = 8'h40;
    rst = 1'b1; repeat (8) @(negedge clk); rst = 1'b0; repeat (8) @(negedge clk);
    @(negedge clk); srv_rst = 1'b1;
    @(negedge clk); srv_rst = 1'b0;
    @(negedge clk); start = 1'b1;
    @(negedge clk); start = 1'b0;
    wait (done);
    repeat (4) @(negedge clk);

    $display("      DBG-B sx0=%0d sy0=%0d step=%0d q=%0d", dut.sx0, dut.sy0, dut.step, dut.q_step);
    chk_anchor(200, 240, "B1 anchor stays at 4.0x too");
    chk_col_fixed(200, 0,   200, "B2 4x: anchor column row 0");
    chk_col_fixed(201, 240, 200, "B3 4x: +1px still src 200");
    chk_col_fixed(203, 240, 200, "B4 4x: +3px still src 200");
    chk_col_fixed(204, 240, 201, "B5 4x: +4px -> src 201");

    //------------------------------------------------------------------
    $display("--- C. 锚点在左上角 (8,8)，zoom=2 ---");
    //------------------------------------------------------------------
    // cam = 320 + (8-320)*512/1024 = 320 - 156 = 164
    // sx0 = 164 - 160 = 4；sy0 = 240 - 120 = 120（ay=240 不变）
    for (i = 0; i < NPX; i = i + 1) mem_v[i] = 0;

    zoom = 8'h20; ax = 10'd8;
    rst = 1'b1; repeat (8) @(negedge clk); rst = 1'b0; repeat (8) @(negedge clk);
    @(negedge clk); srv_rst = 1'b1;
    @(negedge clk); srv_rst = 1'b0;
    @(negedge clk); start = 1'b1;
    @(negedge clk); start = 1'b0;
    wait (done);
    repeat (4) @(negedge clk);

    $display("      DBG-C sx0=%0d sy0=%0d step=%0d q=%0d", dut.sx0, dut.sy0, dut.step, dut.q_step);
    chk_anchor(8, 240, "C1 corner anchor stays put");
    chk_col_fixed(8, 240, 8, "C2 corner column src 8");
    chk_col_fixed(9, 240, 8, "C3 corner +1px still src 8 (2x)");

    //------------------------------------------------------------------
    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

initial begin
    #80_000_000;
    $display("  *** GLOBAL TIMEOUT");
    $finish;
end

endmodule
