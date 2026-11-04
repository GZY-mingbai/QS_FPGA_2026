//==============================================================================
//  tb_viewer.v —— 图片查看器场景层 / HD 图层 的功能验证
//==============================================================================
//
//  【验什么】
//    1. 全屏缩略图（565，2 倍放大）：屏幕 (x,y) 必须对应源像素 (x/2, y/2)
//    2. 全屏高清（888，1:1）：屏幕 (x,y) 必须等于纹理第 y 行第 x 个字
//    3. 两层同时开（过渡）：框内/框外裁剪正确
//    4. 每帧都能收满 307200 个像素（不死锁、不丢字）
//
//  【纹理是"算"出来的，不存数组】见 tex_word()
//
//  【PIPE】ps_data 的语义是"4 拍之前那个扫描位置的像素"。所以本 tb 在
//  第 i 个像素被取走的当拍，比较的是第 i-4 个位置的期望值。
//  如果对不上，说明实际延迟不是 4 ⇒ 改 frame_writer 的 PIPE 或 img_layer_hd 的 LAT。
//
//==============================================================================

`timescale 1ns / 1ps

module tb_viewer;

localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam integer NPX   = IMG_W * IMG_H;

localparam [20:0] T0_BASE = 21'd614400;
localparam [20:0] T1_BASE = 21'd652800;
localparam [20:0] HD_BASE = 21'd729600;

reg clk = 0, mclk = 0, rst = 1;
always #10 clk  = ~clk;      // 50MHz
always #5  mclk = ~mclk;     // 100MHz

//------------------------------------------------------------------------------
//  DUT
//------------------------------------------------------------------------------
reg         ps_start;
reg         NOCMP = 0;   // 1 = only check frame completion, skip pixel compare
wire        pixel_ready;
wire [23:0] ps_data;
wire        pipe_ce;
reg         fmt565   = 1'b1;
reg         wr_stall = 1'b0;

reg         a_hd = 0;
reg  [31:0] a_pos = 32'd0;
reg  [31:0] a_size = {16'd640, 16'd480};
reg  [39:0] a_k = {20'd16384, 20'd32768};
reg  [20:0] a_base = T0_BASE;
reg         a_en = 1;

reg  [31:0] b_pos = {16'd640, 16'd0};
reg  [31:0] b_size = {16'd640, 16'd480};
reg  [39:0] b_k = {20'd16384, 20'd32768};
reg  [20:0] b_base = T1_BASE;
reg         b_en = 0;
reg  [7:0]  alpha_b = 8'd255;

wire [1:0]  tex_req_vld;
wire [41:0] tex_req_addr;
wire [7:0]  tex_req_mask;
wire [1:0]  tex_pix_wr;
wire [63:0] tex_pix_data;

reg  [1:0]  ack_r = 0;
reg  [1:0]  pwr_r = 0;
reg  [63:0] pd_r  = 0;

wire [1:0]  tex_req_ack = ack_r;

assign tex_pix_wr   = pwr_r;
assign tex_pix_data = pd_r;

viewer_scene #(.IMG_W(IMG_W), .IMG_H(IMG_H)) dut (
    .clk(clk), .rst(rst), .mclk(mclk),
    .ps_start(ps_start), .pixel_ready(pixel_ready), .ps_data(ps_data), .pipe_ce(pipe_ce),
    .fmt565(fmt565), .wr_stall(wr_stall),
    .a_hd(a_hd), .a_pos(a_pos), .a_size(a_size), .a_k(a_k), .a_ori(20'd0), .a_base(a_base), .a_en(a_en),
    .quad_mode(1'b0),
    .b_pos(b_pos), .b_size(b_size), .b_k(b_k), .b_base(b_base), .b_en(b_en), .alpha_b(alpha_b),
    .tex_req_vld(tex_req_vld), .tex_req_addr(tex_req_addr), .tex_req_mask(tex_req_mask),
    .tex_req_ack(tex_req_ack), .tex_pix_wr(tex_pix_wr), .tex_pix_data(tex_pix_data)
);

//------------------------------------------------------------------------------
//  纹理内容模型
//------------------------------------------------------------------------------
function [31:0] tex_word;
    input [20:0] a;
    integer w, row, col, cw, p0, p1;
    reg [4:0] r5_0, r5_1, b5;
    reg [5:0] g6_0, g6_1;
    begin
        if (a >= HD_BASE) begin
            w   = a - HD_BASE;
            row = w / IMG_W;
            col = w % IMG_W;
            tex_word = {8'hFF, col[7:0], row[7:0], 8'h5A};
        end else begin
            w   = a - (a >= T1_BASE ? T1_BASE : T0_BASE);
            row = w / 160;
            cw  = w % 160;
            p0  = cw * 2;
            p1  = p0 + 1;
            r5_0 = (p0 >> 4) & 31;  g6_0 = p0 & 63;
            r5_1 = (p1 >> 4) & 31;  g6_1 = p1 & 63;
            b5   = (row >> 3) & 31;
            tex_word = {r5_1, g6_1, b5, r5_0, g6_0, b5};
        end
    end
endfunction

// 注意：y 是**扫描行**（0 = 屏幕上边）。图层按 px_y = 479-y 取源行，
// 所以纹理行 = (479-y)>>1（缩略图是翻转存的）。
function [23:0] exp_thumb;
    input integer x, y;
    integer p, row;
    reg [4:0] r5, b5;
    reg [5:0] g6;
    begin
        p   = x >> 1;
        row = (479 - y) >> 1;
        r5  = (p >> 4) & 31;
        g6  = p & 63;
        b5  = (row >> 3) & 31;
        exp_thumb = {{r5, r5[4:2]}, {g6, g6[5:4]}, {b5, b5[4:2]}};
    end
endfunction

function [23:0] exp_hd;
    input integer x, y;
    begin
        exp_hd = {x[7:0], y[7:0], 8'h5A};
    end
endfunction

integer errors = 0;
integer frames = 0;
integer DBG = 0;
integer SEL = 0;

//------------------------------------------------------------------------------
//  假仲裁器：每层 4 拍一组，先 ack 再回像素
//------------------------------------------------------------------------------
integer    srv_st0, srv_st1;
reg [20:0] srv_a0, srv_a1;
reg [3:0]  srv_k0, srv_k1;
reg [3:0]  gap0, gap1;

always @(posedge mclk or posedge rst) begin
    if (rst) begin
        srv_st0 <= 0; srv_st1 <= 0; ack_r <= 0; pwr_r <= 0; pd_r <= 0;
        gap0 <= 0; gap1 <= 0; srv_k0 <= 0; srv_k1 <= 0;
        srv_a0 <= 0; srv_a1 <= 0;
    end else begin
        pwr_r <= 2'b00;
        ack_r <= 2'b00;

        case (srv_st0)
        0: if (tex_req_vld[0]) begin
               srv_a0  <= tex_req_addr[20:0];
               srv_st0 <= 1;
               gap0    <= 6;
           end
        1: if (gap0 != 0) gap0 <= gap0 - 1;
           else begin
               ack_r   <= ack_r | 2'b01;
               srv_k0  <= 0;
               srv_st0 <= 2;
           end
        2: begin
               pwr_r[0]   <= 1'b1;
               pd_r[31:0] <= tex_word(srv_a0 + srv_k0);
               if (srv_k0 == 0 && frames == 0 && DBG)
                   $display("      [srv0] a=%0d w0=%08h", srv_a0, tex_word(srv_a0));
               if (srv_k0 == 0 && frames == 1 && DBG)
                   $display("      [srv0-hd] a=%0d w0=%08h", srv_a0, tex_word(srv_a0));
               if (srv_k0 == 3) srv_st0 <= 0;
               else             srv_k0 <= srv_k0 + 1;
           end
        default: srv_st0 <= 0;
        endcase

        case (srv_st1)
        0: if (tex_req_vld[1]) begin
               srv_a1  <= tex_req_addr[41:21];
               srv_st1 <= 1;
               gap1    <= 6;
           end
        1: if (gap1 != 0) gap1 <= gap1 - 1;
           else begin
               ack_r   <= ack_r | 2'b10;
               srv_k1  <= 0;
               srv_st1 <= 2;
           end
        2: begin
               pwr_r[1]     <= 1'b1;
               pd_r[63:32]  <= tex_word(srv_a1 + srv_k1);
               if (srv_k1 == 3) srv_st1 <= 0;
               else             srv_k1 <= srv_k1 + 1;
           end
        default: srv_st1 <= 0;
        endcase
    end
end

//------------------------------------------------------------------------------
//  检查
//------------------------------------------------------------------------------


task check_px;
    input integer x, y;
    input [23:0] got;
    reg [23:0] exp;
    begin
        if (a_hd) exp = exp_hd(x, y);
        else      exp = exp_thumb(x, y);
        if (got !== exp) begin
            errors = errors + 1;
            if (errors < 16)
                $display("  [FAIL] frame %0d (%0d,%0d) got %06h exp %06h | pa=%08h pb=%08h va=%b vb=%b rdy=%b%b",
                         frames, x, y, got, exp, dut.pix_a, dut.pix_b, dut.v_a, dut.v_b, dut.rdy_a, dut.rdy_b);
        end
    end
endtask

// 按"源坐标"校验（过渡帧的两层有位移，不能用屏幕坐标直接算）
task check_src;
    input integer sx, sy;
    input [23:0] got;
    reg [23:0] exp;
    begin
        exp = exp_thumb(sx, sy);
        if (got !== exp) begin
            errors = errors + 1;
            if (errors < 16)
                $display("  [FAIL] frame %0d src(%0d,%0d) got %06h exp %06h",
                         frames, sx, sy, got, exp);
        end
    end
endtask

//------------------------------------------------------------------------------
//  单帧：等 NPX 个像素被取走
//------------------------------------------------------------------------------
task run_frame;
    integer i, g;
    integer j;
    integer abort;
    begin
        abort = 0;
        ps_start = 1'b1;
        @(posedge clk);
        for (i = 0; (i < NPX) && !abort; i = i + 1) begin
            g = 0;
            while (!(ps_start && pixel_ready) && (g < 200000)) begin
                @(posedge clk);
                g = g + 1;
            end
            if (g >= 200000) begin
                $display("  [FAIL] frame %0d 在第 %0d 个像素上等不到 ready（死锁）", frames, i);
                $display("    A: warmed=%b rq_idx=%0d ds_idx=%0d ready=%b",
                         dut.u_ta.u_gen.warmed, dut.u_ta.u_gen.rq_idx,
                         dut.u_ta.u_gen.ds_idx, dut.u_ta.layer_ready);
                $display("    B: warmed=%b rq_idx=%0d ds_idx=%0d ready=%b",
                         dut.u_tb.u_gen.warmed, dut.u_tb.u_gen.rq_idx,
                         dut.u_tb.u_gen.ds_idx, dut.u_tb.layer_ready);
                errors = errors + 1;
                abort  = 1;
            end else begin
                // ps_data 此刻是第 i-4 个位置的像素
                if (DBG && i < 24)
                    $display("   i=%0d sx=%0d sy=%0d data=%06h pixa=%08h merge=%08h",
                             i, dut.sx, dut.sy, ps_data, dut.pix_a, dut.merge_pix);
                j = i - 4;
                if (j >= 0 && !NOCMP) begin
                    if (SEL) begin
                        // 过渡帧：A 左移 320（屏幕 x 取源 x+320），B 从 320 起（源 x-320）
                        if (j == 0)               check_src(320, 0, ps_data);
                        if (j == 50*IMG_W + 100)  check_src(420, 50, ps_data);
                        if (j == 240*IMG_W + 320) check_src(0, 240, ps_data);
                        if (j == NPX - 1)         check_src(319, 479, ps_data);
                    end else begin
                        if (j == 0)               check_px(0, 0, ps_data);
                        if (j == 50*IMG_W + 100)  check_px(100, 50, ps_data);
                        if (j == 240*IMG_W + 320) check_px(320, 240, ps_data);
                        if (j == NPX - 1)         check_px(IMG_W-1, IMG_H-1, ps_data);
                    end
                end
                @(posedge clk);
            end
        end
        ps_start = 1'b0;
        if (!abort) $display("  [ ok ] frame %0d 收满 307200 像素", frames);
        frames = frames + 1;
        repeat (3000) @(posedge clk);
    end
endtask

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
initial begin
    ps_start = 0;
    repeat (20) @(posedge clk);
    rst = 0;
    repeat (20) @(posedge clk);

    $display("\n== 1. 缩略图全屏 565 ==");
    DBG = 1;
    a_hd = 0; a_en = 1; b_en = 0; fmt565 = 1;
    a_base = T0_BASE; a_pos = 32'd0;
    a_size = {16'd640, 16'd480}; a_k = {20'd16384, 20'd32768};
    repeat (8) @(posedge clk);
    run_frame;

    $display("\n== 2. 高清全屏 888 ==");
    DBG = 1;
    a_hd = 1; a_en = 1; b_en = 0; fmt565 = 0;
    a_base = HD_BASE; a_pos = 32'd0;
    a_size = {16'd640, 16'd480}; a_k = {20'd16384, 20'd32768};
    repeat (8) @(posedge clk);
    run_frame;

    DBG = 0;
    SEL = 1;
    $display("\n== 3. 双缩略图过渡（A 左移 320，B 从 320 起）==");
    a_hd = 0; a_en = 1; b_en = 1; fmt565 = 1;
    a_base = T0_BASE; a_pos = {16'h0000 - 16'd320, 16'd0};
    b_base = T1_BASE; b_pos = {16'd320, 16'd0};
    repeat (8) @(posedge clk);
    run_frame;

    $display("\n== 4. sparse box: A shift 9, B only 9 columns visible ==");
    NOCMP = 1; DBG = 0; SEL = 0;
    a_hd = 0; a_en = 1; b_en = 1; fmt565 = 1;
    a_base = T0_BASE; a_pos = {16'h0000 - 16'd9, 16'd0};
    a_size = {16'd640, 16'd480}; a_k = {20'd16384, 20'd32768};
    b_base = T1_BASE; b_pos = {16'd631, 16'd0};
    b_size = {16'd640, 16'd480}; b_k = {20'd16384, 20'd32768};
    repeat (8) @(posedge clk);
    run_frame;

    $display("\n==== errors = %0d，frames = %0d ====", errors, frames);
    if (errors == 0) $display("PASS");
    else             $display("FAIL");
    $finish;
end

initial begin
    #400_000_000;
    $display("TIMEOUT");
    $finish;
end

endmodule
