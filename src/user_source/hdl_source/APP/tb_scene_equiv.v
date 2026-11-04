`timescale 1ns/1ps
//==============================================================================
//  tb_scene_equiv.v —— viewer_scene 改动前后的输出像素流逐像素等价
//==============================================================================
//  2026-11-01 新建（[FIFO-SHARE]：img_layer_hd 不再自带 FIFO，借用 u_ta 的）。
//  对比改动前版本（viewer_scene_ref / img_layer_ctrl_ref / img_layer_hd_ref，
//  由 run_all_tb.sh 从固定提交 d7f1cc6 生成）与现版。
//
//  两边各接一套独立的纹理存储模型（随机延迟、按掩码回字）和一个 frame_writer
//  行为模型（take = ps_start & pixel_ready，PIPE=5 延迟链，帧尾排水），
//  wr_stall 随机背压。时序允许不同（新版请求多过一级 skid），
//  但**每帧写出的 307200 个像素必须逐个相同**。
//
//  帧序列覆盖：HD ↔ 缩略图(A+B 混合) ↔ HD ↔ 四宫格 ↔ HD ↔ 放大缩略图 ↔ HD，
//  即每一次 a_hd 翻转（共享 FIFO 换主人）都至少发生两次。
//  另检查每帧确有内容（非零像素数）、没有走看门狗降级。
//==============================================================================
module tb_scene_equiv;

localparam integer NPX = 640 * 480;
localparam integer NFR = 12;

reg clk = 0, mclk = 0, rst = 1;
always #10 clk  = ~clk;     // 50 MHz
always #5  mclk = ~mclk;    // 100 MHz

// ---------------- 场景配置（两边共用，帧间才改） ----------------
reg        fmt565 = 0, a_hd = 0, a_en = 0, quad_mode = 0, b_en = 0;
reg  [3:0] qvm = 4'hF;            // 四宫格有效槽掩码（帧 3 用不满页）
reg [31:0] a_pos = 0, a_size = 0, b_pos = 0, b_size = 0;
reg [39:0] a_k = 0, b_k = 0;
reg [19:0] a_ori = 0;
reg [20:0] a_base = 0, b_base = 0;
reg [7:0]  alpha_b = 0;

// ---------------- 两套 DUT + 模型 ----------------
wire        n_ps_start, r_ps_start;
wire        n_prdy, r_prdy, n_ce, r_ce;
wire [23:0] n_data, r_data;
wire        n_stall, r_stall;
wire [1:0]  n_rv, r_rv, n_ack, r_ack, n_pw, r_pw;
wire [41:0] n_ra, r_ra;
wire [7:0]  n_rm, r_rm;
wire [63:0] n_pd, r_pd;

viewer_scene u_new (
    .clk(clk), .rst(rst), .mclk(mclk),
    .ps_start(n_ps_start), .pixel_ready(n_prdy), .ps_data(n_data), .pipe_ce(n_ce),
    .fmt565(fmt565), .wr_stall(n_stall),
    .a_hd(a_hd), .a_pos(a_pos), .a_size(a_size), .a_k(a_k), .a_ori(a_ori),
    .a_base(a_base), .a_en(a_en), .quad_mode(quad_mode), .quad_vm(qvm),
    .b_pos(b_pos), .b_size(b_size), .b_k(b_k), .b_base(b_base), .b_en(b_en), .alpha_b(alpha_b),
    .scan_x(), .scan_y(), .osd_on(1'b0), .osd_fade(8'd0), .osd_pix(32'd0),
    .tex_req_vld(n_rv), .tex_req_addr(n_ra), .tex_req_mask(n_rm), .tex_req_ack(n_ack),
    .tex_pix_wr(n_pw), .tex_pix_data(n_pd)
);
viewer_scene_ref u_ref (
    .clk(clk), .rst(rst), .mclk(mclk),
    .ps_start(r_ps_start), .pixel_ready(r_prdy), .ps_data(r_data), .pipe_ce(r_ce),
    .fmt565(fmt565), .wr_stall(r_stall),
    .a_hd(a_hd), .a_pos(a_pos), .a_size(a_size), .a_k(a_k), .a_ori(a_ori),
    .a_base(a_base), .a_en(a_en), .quad_mode(quad_mode), .quad_vm(qvm),
    .b_pos(b_pos), .b_size(b_size), .b_k(b_k), .b_base(b_base), .b_en(b_en), .alpha_b(alpha_b),
    .scan_x(), .scan_y(), .osd_on(1'b0), .osd_fade(8'd0), .osd_pix(32'd0),
    .tex_req_vld(r_rv), .tex_req_addr(r_ra), .tex_req_mask(r_rm), .tex_req_ack(r_ack),
    .tex_pix_wr(r_pw), .tex_pix_data(r_pd)
);

tse_mem #(.SEED(3))  m_n0 (.mclk(mclk), .rst(rst), .vld(n_rv[0]), .addr(n_ra[20:0]),  .mask(n_rm[3:0]),
                          .ack(n_ack[0]), .wr(n_pw[0]), .data(n_pd[31:0]));
tse_mem #(.SEED(5))  m_n1 (.mclk(mclk), .rst(rst), .vld(n_rv[1]), .addr(n_ra[41:21]), .mask(n_rm[7:4]),
                          .ack(n_ack[1]), .wr(n_pw[1]), .data(n_pd[63:32]));
tse_mem #(.SEED(7))  m_r0 (.mclk(mclk), .rst(rst), .vld(r_rv[0]), .addr(r_ra[20:0]),  .mask(r_rm[3:0]),
                          .ack(r_ack[0]), .wr(r_pw[0]), .data(r_pd[31:0]));
tse_mem #(.SEED(9))  m_r1 (.mclk(mclk), .rst(rst), .vld(r_rv[1]), .addr(r_ra[41:21]), .mask(r_rm[7:4]),
                          .ack(r_ack[1]), .wr(r_pw[1]), .data(r_pd[63:32]));

reg  go = 0;
wire n_done, r_done, n_we, r_we;
wire [18:0] n_idx, r_idx;
wire [31:0] n_cyc, r_cyc;

tse_fw #(.SEED(13)) f_n (.clk(clk), .rst(rst), .go(go), .ps_start(n_ps_start), .pixel_ready(n_prdy),
                         .pipe_ce(n_ce), .wr_stall(n_stall), .we(n_we), .idx(n_idx), .done(n_done), .cyc(n_cyc));
tse_fw #(.SEED(17)) f_r (.clk(clk), .rst(rst), .go(go), .ps_start(r_ps_start), .pixel_ready(r_prdy),
                         .pipe_ce(r_ce), .wr_stall(r_stall), .we(r_we), .idx(r_idx), .done(r_done), .cyc(r_cyc));

reg [23:0] fr_n [0:NPX-1];
reg [23:0] fr_r [0:NPX-1];
always @(posedge clk) begin
    if (n_we) fr_n[n_idx] <= n_data;
    if (r_we) fr_r[r_idx] <= r_data;
end

// ---------------- 帧配置 ----------------
task set_frame(input integer k);
begin
    quad_mode = 0; b_en = 0; alpha_b = 0; a_ori = 0; qvm = 4'hF;
    a_pos = 0; a_size = {16'd640, 16'd480}; a_k = {20'd32768, 20'd32768};
    case (k)
        0, 2, 4, 6, 11: begin                       // HD 888 满屏
            a_hd = 1; a_en = 1; fmt565 = 0;
            a_base = 21'h040000 + k * 21'h1000;
        end
        1: begin                                // 缩略图 A 满屏 + B 半透明小框
            a_hd = 0; a_en = 1; fmt565 = 1; a_base = 21'h010000;
            b_en = 1; b_pos = {16'd120, 16'd80}; b_size = {16'd320, 16'd240};
            b_k = {20'd65536, 20'd65536}; b_base = 21'h020000; alpha_b = 8'd128;
        end
        3: begin                                // 四宫格（a_hd=1 也必须被 quad 压掉）
            // ★ 2026-11-03：四宫格改画在 B 层、格子外透出 A 层（相册）——这是**有意**的行为变化，
            //   参考版格子外是黑的。A 关掉（a_en=0）后两边格子内外都应逐像素相同。
            a_hd = 1; a_en = 0; fmt565 = 1; quad_mode = 1; a_base = 21'h030000;
            qvm = 4'b1011;                      // 不满页：右上格空
        end
        // ---- 2026-11-01 [PGI-SLIM] 追加：fx 动画里会出现的几何 ----
        7: begin                                // fx0 横滑：A 左移出屏（X0<0）+ B 从右侧滑入
            a_hd = 0; a_en = 1; fmt565 = 1; a_base = 21'h010000;
            a_pos = {16'hFF38, 16'd0};          // X0 = −200
            b_en = 1; b_pos = {16'd440, 16'd0}; b_size = {16'd640, 16'd480};
            b_k = {20'd32768, 20'd32768}; b_base = 21'h020000; alpha_b = 8'd255;
        end
        8: begin                                // A 缩小 0.75 居中 + B 跨右/下边界
            a_hd = 0; a_en = 1; fmt565 = 1; a_base = 21'h018000;
            a_pos = {16'd80, 16'd60}; a_size = {16'd480, 16'd360}; a_k = {20'd43690, 20'd43690};
            b_en = 1; b_pos = {16'd500, 16'd400}; b_size = {16'd320, 16'd240};
            b_k = {20'd65536, 20'd65536}; b_base = 21'h028000; alpha_b = 8'd200;
        end
        9: begin                                // A 1.25× 放大 + 锚点；B 顶边伸出屏外（Y0<0）
            a_hd = 0; a_en = 1; fmt565 = 1; a_base = 21'h010000;
            a_k = {20'd26214, 20'd26214}; a_ori = {10'd40, 10'd30};
            b_en = 1; b_pos = {16'd100, 16'hFFC0}; b_size = {16'd200, 16'd150};
            b_k = {20'd52428, 20'd52428}; b_base = 21'h020000; alpha_b = 8'd96;
        end
        10: begin                               // 四宫格另一种不满页
            a_hd = 1; a_en = 0; fmt565 = 1; quad_mode = 1; a_base = 21'h030000;
            qvm = 4'b0110;
        end
        5: begin                                // 缩略图 2× 放大 + 锚点（S_ZOOM 那种几何）+ B 小框 1:1
            a_hd = 0; a_en = 1; fmt565 = 1; a_base = 21'h018000;
            a_k = {20'd16384, 20'd16384}; a_ori = {10'd80, 10'd60};
            b_en = 1; b_pos = {16'd400, 16'd300}; b_size = {16'd160, 16'd120};
            b_k = {20'd65536, 20'd65536}; b_base = 21'h028000; alpha_b = 8'd255;
        end
    endcase
end
endtask

// 降级/死层：帧末会被 !ps_start 清掉，所以帧内粘住
reg deg = 0;
always @(posedge clk) begin
    if (go) deg <= 0;
    else if (u_new.frame_degraded || u_ref.frame_degraded ||
             u_new.u_hd.dead || u_ref.u_hd.dead) deg <= 1;
end

integer k, i, errors = 0, nz, diffs;
reg [31:0] sum;
initial begin
    repeat (20) @(posedge clk); rst = 0;
    repeat (200) @(posedge clk);
`ifndef K0
`define K0 0
`endif
    for (k = `K0; k < NFR; k = k + 1) begin
        @(negedge clk); set_frame(k);
        repeat (3000) @(posedge clk);           // 配置在帧间稳定（含 k 的串行除法）
        @(negedge clk); go = 1; @(negedge clk); go = 0;
        wait (n_done && r_done);
        repeat (50) @(posedge clk);
        diffs = 0; nz = 0; sum = 0;
        for (i = 0; i < NPX; i = i + 1) begin
            if (fr_n[i] !== fr_r[i]) begin
                diffs = diffs + 1;
                if (diffs <= 5)
                    $display("  *** FAIL  frame %0d px %0d (x=%0d y=%0d): new=%h ref=%h",
                             k, i, i % 640, i / 640, fr_n[i], fr_r[i]);
            end
            if (fr_r[i] != 24'd0) nz = nz + 1;
            sum = sum + {8'd0, fr_r[i]} * (i + 1);
        end
        $display("  frame %0d  a_hd=%b quad=%b fmt565=%b  cyc new=%0d ref=%0d  nonzero=%0d  sum=%h  diffs=%0d",
                 k, a_hd, quad_mode, fmt565, n_cyc, r_cyc, nz, sum, diffs);
        if (diffs != 0) errors = errors + 1;
        if (nz < NPX / 20) begin
            $display("  *** FAIL  frame %0d is (almost) empty: nonzero=%0d", k, nz);
            errors = errors + 1;
        end
        if (deg) begin
            $display("  *** FAIL  frame %0d degraded (stall watchdog / hd dead)", k);
            errors = errors + 1;
        end
    end
    if (errors == 0) $display("  ALL PASS  (%0d frames bit-identical)", NFR);
    else             $display("  FAILED    %0d problems", errors);
    $finish;
end

// 每帧的看门狗快照：帧末 degraded 已被 !ps_start 清掉，所以在帧内粘住
initial begin : timeout
    #(64'd1_000_000_000);    // 1 s
    $display("  *** FAIL  global timeout: new done=%b idx=%0d ps=%b | ref done=%b idx=%0d ps=%b",
             n_done, n_idx, n_ps_start, r_done, r_idx, r_ps_start);
    $finish;
end
// ★ 2026-11-01 [PGI-SLIM]：参考版的"每帧字数预算"在 1.25× 放大 + 锚点（帧 9）下**偏紧**：
//   push_cnt 撞到 76800 后请求被裁 ⇒ 参考版卡 ~4M 拍、底部两行取错字（见 -DPROBE）。
//   新版已删预算（这是修 bug，不是等价改动），所以 run_all_tb.sh 生成参考版时把
//   `req_push = flush_do && budget_ok` 改成 `= flush_do`，其余改动仍逐像素对照。
//   （帧 0–8/10/11 带预算也逐像素相同：预算只在帧 9 撞到。）
`ifdef PROBE
reg pb_a = 0, pb_b = 0;
always @(posedge clk) begin
    if (go) begin pb_a <= 0; pb_b <= 0; end
    if (u_ref.u_ta.u_gen.flush_do && !u_ref.u_ta.u_gen.budget_ok && !pb_a) begin
        pb_a <= 1; $display("  [probe] ref A budget hit: push_cnt=%0d budget=%0d", u_ref.u_ta.u_gen.push_cnt, u_ref.u_ta.u_gen.budget); end
    if (u_ref.u_tb.u_gen.flush_do && !u_ref.u_tb.u_gen.budget_ok && !pb_b) begin
        pb_b <= 1; $display("  [probe] ref B budget hit: push_cnt=%0d budget=%0d f_x0=%h", u_ref.u_tb.u_gen.push_cnt, u_ref.u_tb.u_gen.budget, u_ref.u_tb.u_gen.f_x0); end
end
`endif
endmodule

//------------------------------------------------------------------------------
//  纹理存储模型：一次一个 4 字组；ack 一拍 → 随机延迟 → 按掩码逐字回写
//  内容 = 地址的确定性散列（两套模型内容相同，只是延迟不同）
//------------------------------------------------------------------------------
module tse_mem #(parameter integer SEED = 1) (
    input  wire        mclk, rst,
    input  wire        vld,
    input  wire [20:0] addr,
    input  wire [3:0]  mask,
    output reg         ack,
    output reg         wr,
    output reg  [31:0] data
);
reg [31:0] lf = SEED * 32'h9E37_79B9 + 1;
reg [20:0] a_q; reg [3:0] m_q;
reg [4:0]  wait_c;
reg [2:0]  wi;
reg        busy;
function [31:0] f; input [20:0] a; reg [31:0] h; begin
    h = {11'd0, a} * 32'h0101_9E37 + 32'h1234_5678;
    f = h ^ (h >> 13);
end endfunction
always @(posedge mclk) begin
    lf <= {lf[30:0], lf[31] ^ lf[21] ^ lf[1] ^ lf[0]};
    ack <= 1'b0; wr <= 1'b0;
    if (rst) begin busy <= 0; wi <= 0; end
    else if (!busy) begin
        if (vld && !ack) begin
            ack <= 1'b1; a_q <= addr; m_q <= mask; busy <= 1'b1;
            wait_c <= 2 + lf[3:0]; wi <= 0;
        end
    end else if (wait_c != 0) begin
        wait_c <= wait_c - 1'b1;
    end else if (wi == 3'd4) begin
        busy <= 1'b0;
    end else if (lf[5:4] != 2'b00) begin       // 偶尔空一拍
        if (m_q[wi[1:0]]) begin wr <= 1'b1; data <= f(a_q + wi[1:0]); end
        wi <= wi + 1'b1;
    end
end
endmodule

//------------------------------------------------------------------------------
//  frame_writer 行为模型（只保留与像素源握手相关的部分）
//------------------------------------------------------------------------------
module tse_fw #(parameter integer SEED = 1) (
    input  wire        clk, rst, go,
    output reg         ps_start,
    input  wire        pixel_ready,
    input  wire        pipe_ce,
    output wire        wr_stall,
    output wire        we,
    output reg  [18:0] idx,
    output reg         done,
    output reg  [31:0] cyc
);
localparam integer NPX = 640 * 480;
localparam integer PIPE = 5;
reg [31:0] lf = SEED * 32'h2545_F491 + 7;
reg [18:0] cnt;
reg [PIPE-1:0] en_d;
reg [3:0]  st_run;
always @(posedge clk) lf <= {lf[30:0], lf[31] ^ lf[21] ^ lf[1] ^ lf[0]};
// 随机背压：成串出现（模拟写 FIFO 快满）
always @(posedge clk) begin
    if (rst) st_run <= 0;
    else if (st_run != 0) st_run <= st_run - 1'b1;
    else if (lf[7:0] == 8'h00) st_run <= lf[11:8];
end
assign wr_stall = (st_run != 0);

wire take = ps_start & pixel_ready & pipe_ce;
assign we = en_d[PIPE-1] & pipe_ce;
always @(posedge clk) begin
    if (rst) begin
        ps_start <= 0; cnt <= 0; en_d <= 0; idx <= 0; done <= 1; cyc <= 0;
    end else begin
        if (go) begin ps_start <= 1; cnt <= 0; idx <= 0; done <= 0; cyc <= 0; end
        else if (take) begin
            cnt <= cnt + 1'b1;
            if (cnt == NPX - 1) ps_start <= 0;
        end
        if (!done) cyc <= cyc + 1;
        if (pipe_ce) en_d <= {en_d[PIPE-2:0], take};
        if (we) begin
            idx <= idx + 1'b1;
            if (idx == NPX - 1) done <= 1;
        end
    end
end
endmodule
