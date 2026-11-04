`timescale 1ns / 1ps
//==============================================================================
//  tb_wrblind.v —— 量化"帧缓存写在忙窗口里盲发"的比例
//==============================================================================
//
//  【量什么】`App_wr_en && Sdr_busy` 的拍数（记作 blind）。
//  写口没有重发机制，所以**每一条这样的命令 = 帧缓存里一个永久洞**
//  （读回来是 32'hFFFFFFFF = 纯白）。blind 就是"每帧会掉多少个像素"的量度。
//
//  【忙信号怎么给】随机忙窗口：长度 7 拍、间隔 4~7 拍随机。
//  ★ 为什么不用固定周期：用固定周期（例如每 37 拍高 6 拍）时，blind 完全由
//    "写流与忙窗口的相位对齐"决定，任何判据改动都只是换了个相位，
//    会得出"改了更好/更差"的假结论。随机化之后才比得出真实差异。
//
//  【实测结论（2026-10-23, N=512）】
//      原始（单拍前视预测 wr_blk_g）      blind = 205 / 512 = 40%
//      + 起头看 !Sdr_busy_q（S1）        blind = 205 / 512 = 40%   ← 中性
//      改成"连续 4 拍安静才放行"          blind = 425 / 512 = 83%   ← 更差
//  ⇒ **判据再怎么调都救不了**：忙占空 40% 时盲发率就是 40%，
//    因为 `Sdr_busy` 对未来 1~3 拍没有任何预测能力。
//  ⇒ 唯一出路是**重发**（写幂等，重发整组最省事），不是把判据做聪明。
//
//  跑法：
//      iverilog -g2005 -s tb_wrblind -o /tmp/t.vvp SD/sdr_port_sched.v SD/tb_wrblind.v
//      vvp /tmp/t.vvp        # 打印 RESULT issued=.. blind=.. wrgo=.. cyc=.. duty=..%
//
//==============================================================================

module tb_wrblind;

    localparam integer N    = 512;
    localparam [20:0]  BASE = 21'h1_0000;

    reg clk = 0, rst = 1;
    always #4 clk = ~clk;

    reg  [20:0] wcnt;
    reg         go;
    wire        disp_wr_ready;
    wire        disp_wr_en   = go && (wcnt < N[20:0]) && disp_wr_ready;
    wire [20:0] disp_wr_addr = BASE + wcnt;
    wire [31:0] disp_wr_din  = {11'd0, disp_wr_addr};
    wire [3:0]  disp_wr_dm   = 4'h0;

    always @(posedge clk or posedge rst) begin
        if (rst)             wcnt <= 21'd0;
        else if (disp_wr_en) wcnt <= wcnt + 21'd1;
    end

    // 忙：周期 37、高 6 拍（和 256 拍时隙轮 / 16 拍槽互质，覆盖各相位）
    // 随机忙窗口：长度 7 拍，间隔 4~15 拍随机 ⇒ 与 256 拍时隙轮去相关
    reg [4:0] bc;
    reg [3:0] gap;
    reg       sbusy;
    always @(posedge clk or posedge rst) begin
        if (rst) begin bc <= 5'd0; gap <= 4'd7; sbusy <= 1'b0; end
        else if (sbusy) begin
            if (bc == 5'd6) begin sbusy <= 1'b0; bc <= 5'd0; gap <= ($random & 4'hF) % 4 + 4; end
            else bc <= bc + 5'd1;
        end else begin
            if (gap == 4'd0) begin sbusy <= 1'b1; bc <= 5'd0; end
            else gap <= gap - 4'd1;
        end
    end

    wire disp_rd_ready, disp_rd_vld;
    wire [31:0] disp_rd_data;
    wire [1:0]  tex_req_ack, tex_pix_wr;
    wire [63:0] tex_pix_data;
    wire ld_wr_ack, ld_wr_settled;
    wire App_rd_en, App_wr_en;
    wire [20:0] App_rd_addr, App_wr_addr;
    wire [31:0] App_wr_din;
    wire [3:0]  App_wr_dm;
    wire Sdr_rd_en;
    wire [31:0] Sdr_rd_dout;

    sdr_port_sched #(.BUSY_HOLD(1), .BUSY_GATE(0)) u_dut (
        .clk(clk), .rst(rst),
        .disp_rd_en(1'b0), .disp_rd_want(1'b0), .disp_rd_addr(21'd0),
        .disp_rd_ready(disp_rd_ready), .disp_rd_vld(disp_rd_vld), .disp_rd_data(disp_rd_data),
        .disp_wr_en(disp_wr_en), .disp_wr_addr(disp_wr_addr),
        .disp_wr_din(disp_wr_din), .disp_wr_dm(disp_wr_dm), .disp_wr_ready(disp_wr_ready),
        .tex_req_vld(2'b00), .tex_req_addr(42'd0), .tex_req_mask(8'd0),
        .tex_req_ack(tex_req_ack), .tex_pix_wr(tex_pix_wr), .tex_pix_data(tex_pix_data),
        .ld_wr_vld(1'b0), .ld_wr_used(5'd0), .ld_wr_addr(21'd0), .ld_wr_data(32'd0),
        .ld_wr_ack(ld_wr_ack), .ld_wr_settled(ld_wr_settled),
        .App_rd_en(App_rd_en), .App_rd_addr(App_rd_addr),
        .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr),
        .App_wr_din(App_wr_din), .App_wr_dm(App_wr_dm),
        .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout),
        .Sdr_busy(sbusy), .Sdr_init_ref_vld(1'b0),
        .rd_lag(1'b0), .tx_ph_off(2'd0), .ph_off(2'd0), .exp_mode(4'd0),
        .dbg_wbusy(go && (wcnt < N[20:0]))
    );

    // 统计
    integer issued = 0;     // 发出去的写命令
    integer blind  = 0;     // 在忙窗口里发出去的（= 必然丢）
    integer wrgo_c = 0;     // wr_go 高多少拍
    integer cyc    = 0;

    always @(posedge clk) begin
        if (!rst) begin
            cyc = cyc + 1;
            if (u_dut.wr_go)       wrgo_c = wrgo_c + 1;
            if (App_wr_en)         issued = issued + 1;
            if (App_wr_en && sbusy) blind = blind + 1;
        end
    end

    initial begin
        go = 1'b0;
        repeat (20) @(posedge clk);
        rst = 0;
        repeat (10) @(posedge clk);
        go = 1'b1;
        wait (wcnt == N[20:0]);
        repeat (1200) @(posedge clk);
        $display("RESULT issued=%0d blind=%0d wrgo=%0d cyc=%0d duty=%0d%%",
                 issued, blind, wrgo_c, cyc, (wrgo_c*100)/cyc);
        $finish;
    end
endmodule
