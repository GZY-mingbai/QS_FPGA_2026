`timescale 1ns/1ps
//==============================================================================
//  tb_sched_equiv.v —— sdr_port_sched 改动前后逐拍等价（面积优化回归）
//==============================================================================
//  2026-10-31 新建。用于"只改实现、不改行为"的面积优化（如 tag_mem 改落 BRAM）：
//  把改动前的 sdr_port_sched 改名为 sdr_port_sched_ref 与现版并排，喂**完全相同**的
//  随机激励（显示读写 / 两层纹理请求 / 装载写 / SDRAM 回拍 / busy / 刷新），
//  逐拍比较全部输出；任何一拍任何一位不同即 FAIL。
//
//  参考版怎么来：
//    git show HEAD:src/user_source/hdl_source/SD/sdr_port_sched.v \
//      | sed 's/^module sdr_port_sched/module sdr_port_sched_ref/' > ref.v
//  跑法：
//    iverilog -g2012 -s tb_sched_equiv -o eq.vvp tb_sched_equiv.v sdr_port_sched.v ref.v
//    vvp eq.vvp
//  参数与顶层例化一致（TAG_AW=8, BUSY_HOLD=1, BUSY_GATE=0；rd_lag/ph_off/exp_mode 恒 0）。
//==============================================================================
module tb_sched_equiv;

parameter integer NCYC   = 400000;
parameter integer RD_LAT = 11;          // App_rd_en → Sdr_rd_en 的回拍延迟（板上实测 ~11）

reg clk = 0, rst = 1;
always #5 clk = ~clk;

// ---- 共同激励 ----
reg         disp_rd_en = 0, disp_rd_want = 0;
reg  [20:0] disp_rd_addr = 0;
reg         disp_wr_en = 0;
reg  [20:0] disp_wr_addr = 0;
reg  [31:0] disp_wr_din = 0;
reg  [3:0]  disp_wr_dm = 0;
reg  [1:0]  tex_req_vld = 0;
reg  [41:0] tex_req_addr = 0;
reg  [7:0]  tex_req_mask = 0;
reg         ld_wr_vld = 0;
reg  [4:0]  ld_wr_used = 0;
reg  [20:0] ld_wr_addr = 0;
reg  [31:0] ld_wr_data = 0;
reg         Sdr_busy = 0, Sdr_init_ref_vld = 0, App_ref_req = 0;
wire        Sdr_rd_en;
wire [31:0] Sdr_rd_dout;

// ---- 两份 DUT 的输出 ----
`define OUTS(p) \
wire p``_disp_rd_ready, p``_disp_rd_vld, p``_disp_wr_ready, p``_ld_wr_ack, p``_ld_wr_settled; \
wire [31:0] p``_disp_rd_data; \
wire [1:0]  p``_tex_req_ack, p``_tex_pix_wr; \
wire [63:0] p``_tex_pix_data; \
wire        p``_App_rd_en, p``_App_wr_en; \
wire [20:0] p``_App_rd_addr, p``_App_wr_addr; \
wire [31:0] p``_App_wr_din; \
wire [3:0]  p``_App_wr_dm;
`OUTS(n)
`OUTS(r)

`define CONN(p) \
    .clk(clk), .rst(rst), \
    .disp_rd_en(disp_rd_en), .disp_rd_want(disp_rd_want), .disp_rd_addr(disp_rd_addr), \
    .disp_rd_ready(p``_disp_rd_ready), .disp_rd_vld(p``_disp_rd_vld), .disp_rd_data(p``_disp_rd_data), \
    .disp_wr_en(disp_wr_en), .disp_wr_addr(disp_wr_addr), .disp_wr_din(disp_wr_din), \
    .disp_wr_dm(disp_wr_dm), .disp_wr_ready(p``_disp_wr_ready), \
    .tex_req_vld(tex_req_vld), .tex_req_addr(tex_req_addr), .tex_req_mask(tex_req_mask), \
    .tex_req_ack(p``_tex_req_ack), .tex_pix_wr(p``_tex_pix_wr), .tex_pix_data(p``_tex_pix_data), \
    .ld_wr_vld(ld_wr_vld), .ld_wr_used(ld_wr_used), .ld_wr_addr(ld_wr_addr), .ld_wr_data(ld_wr_data), \
    .ld_wr_ack(p``_ld_wr_ack), .ld_wr_settled(p``_ld_wr_settled), \
    .App_rd_en(p``_App_rd_en), .App_rd_addr(p``_App_rd_addr), \
    .App_wr_en(p``_App_wr_en), .App_wr_addr(p``_App_wr_addr), .App_wr_din(p``_App_wr_din), .App_wr_dm(p``_App_wr_dm), \
    .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout), .Sdr_busy(Sdr_busy), \
    .Sdr_init_ref_vld(Sdr_init_ref_vld), .App_ref_req(App_ref_req), \
    .rd_lag(1'b0), .tx_ph_off(2'd0), .ph_off(2'd0), .exp_mode(4'd0), .dbg_wbusy(1'b0)

sdr_port_sched     #(.TAG_AW(8), .BUSY_HOLD(1), .BUSY_GATE(0)) u_new (`CONN(n));
sdr_port_sched_ref #(.TAG_AW(8), .BUSY_HOLD(1), .BUSY_GATE(0)) u_ref (`CONN(r));

// ---- SDRAM 读回模型（由参考版的 App_rd_en 驱动；两版若分叉会先在 App_rd_* 上报出来）----
reg [RD_LAT-1:0] rd_pipe = 0;
reg [20:0]       ra_pipe [0:RD_LAT-1];
integer k;
always @(posedge clk) begin
    rd_pipe <= {rd_pipe[RD_LAT-2:0], r_App_rd_en};
    ra_pipe[0] <= r_App_rd_addr;
    for (k = 1; k < RD_LAT; k = k + 1) ra_pipe[k] <= ra_pipe[k-1];
end
assign Sdr_rd_en   = rd_pipe[RD_LAT-1];
assign Sdr_rd_dout = {11'h5A5, ra_pipe[RD_LAT-1]} ^ 32'h1357_9BDF;

// ---- 随机激励（请求类信号保持到被接受，模拟真实握手）----
integer seed = 7;
always @(posedge clk) if (!rst) begin
    disp_rd_want <= ($random(seed) & 7) != 0;
    if (!disp_rd_en || r_disp_rd_ready) begin
        disp_rd_en   <= ($random(seed) & 3) == 0;
        disp_rd_addr <= $random(seed) % 614400;
    end
    if (!disp_wr_en || r_disp_wr_ready) begin
        disp_wr_en   <= ($random(seed) & 7) == 0;
        disp_wr_addr <= $random(seed) % 614400;
        disp_wr_din  <= $random(seed);
        disp_wr_dm   <= 4'h0;
    end
    if (tex_req_vld[0] && r_tex_req_ack[0]) tex_req_vld[0] <= 1'b0;
    else if (!tex_req_vld[0] && (($random(seed) & 3) == 0)) begin
        tex_req_vld[0]      <= 1'b1;
        tex_req_addr[20:0]  <= 21'd614400 + ($random(seed) & 21'h3FFFC);
        tex_req_mask[3:0]   <= ($random(seed) & 15) | 4'b0001;
    end
    if (tex_req_vld[1] && r_tex_req_ack[1]) tex_req_vld[1] <= 1'b0;
    else if (!tex_req_vld[1] && (($random(seed) & 3) == 0)) begin
        tex_req_vld[1]      <= 1'b1;
        tex_req_addr[41:21] <= 21'd652800 + ($random(seed) & 21'h3FFFC);
        tex_req_mask[7:4]   <= ($random(seed) & 15) | 4'b0001;
    end
    if (!ld_wr_vld || r_ld_wr_ack) begin
        ld_wr_vld  <= ($random(seed) & 15) == 0;
        ld_wr_used <= $random(seed) & 5'h1F;
        ld_wr_addr <= 21'd729600 + ($random(seed) & 21'hFFFF);
        ld_wr_data <= $random(seed);
    end
    Sdr_busy         <= ($random(seed) & 31) == 0;
    Sdr_init_ref_vld <= ($random(seed) & 255) == 0;
    App_ref_req      <= ($random(seed) & 255) == 1;
end

// ---- 逐拍比较 ----
integer errors = 0, cyc = 0, nrd = 0, ntx = 0;
`define CMP(sig) if (n_``sig !== r_``sig) begin errors = errors + 1; \
    if (errors <= 20) $display("  *** FAIL  cyc %0d  %0s : new=%h ref=%h", cyc, `"sig`", n_``sig, r_``sig); end
always @(posedge clk) if (!rst) begin
    cyc = cyc + 1;
    if (r_App_rd_en) nrd = nrd + 1;
    if (|r_tex_pix_wr) ntx = ntx + 1;
    `CMP(disp_rd_ready) `CMP(disp_rd_vld) `CMP(disp_wr_ready)
    `CMP(tex_req_ack) `CMP(tex_pix_wr)
    // 数据总线只在各自的有效拍有意义（无效拍是 don't-care，参考版仿真里会是 X）
    if (r_disp_rd_vld && (n_disp_rd_data !== r_disp_rd_data)) begin errors = errors + 1;
        if (errors <= 20) $display("  *** FAIL  cyc %0d  disp_rd_data : new=%h ref=%h", cyc, n_disp_rd_data, r_disp_rd_data); end
    if (r_tex_pix_wr[0] && (n_tex_pix_data[31:0] !== r_tex_pix_data[31:0])) begin errors = errors + 1;
        if (errors <= 20) $display("  *** FAIL  cyc %0d  tex_pix_data[0] : new=%h ref=%h", cyc, n_tex_pix_data[31:0], r_tex_pix_data[31:0]); end
    if (r_tex_pix_wr[1] && (n_tex_pix_data[63:32] !== r_tex_pix_data[63:32])) begin errors = errors + 1;
        if (errors <= 20) $display("  *** FAIL  cyc %0d  tex_pix_data[1] : new=%h ref=%h", cyc, n_tex_pix_data[63:32], r_tex_pix_data[63:32]); end
    `CMP(ld_wr_ack) `CMP(ld_wr_settled)
    `CMP(App_rd_en) `CMP(App_rd_addr) `CMP(App_wr_en) `CMP(App_wr_addr) `CMP(App_wr_din) `CMP(App_wr_dm)
end

initial begin
    repeat (20) @(posedge clk); rst = 0;
    repeat (NCYC) @(posedge clk);
    $display("== %0d cycles, %0d SDRAM reads, %0d texture beats compared ==", cyc, nrd, ntx);
    if (nrd < 1000 || ntx < 1000) begin
        errors = errors + 1;
        $display("  *** FAIL  stimulus too weak (reads=%0d tex beats=%0d)", nrd, ntx);
    end
    if (errors == 0) $display("  ALL PASS  (outputs identical every cycle)");
    else             $display("  FAILED    %0d mismatches", errors);
    $finish;
end

endmodule
