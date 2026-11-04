`timescale 1ns/1ps
//==============================================================================
//  tb_frame_rw.v —— frame_read_write 端到端：写进去的字，读回来还是不是那个字
//==============================================================================
//  【为什么要这个台】
//    上板现象：**HD（888）右边有一整列 1px 黑线，565 没有**；烧回 d1b08f8
//    就没有了 ⇒ 是这次改动引入的回归。而这一轮已经把上游全测穿了：
//      · bmp_read 640×480            → 307200 点逐点精确
//      · bmp_scale 888 画布          → 307200 字、0 洞、0 位移、第 639 列正确
//      · viewer_scene 送出的像素流   → 按 PIPE=4 补偿后 0/307196
//      · frame_fifo_write 写地址/使能 → 888 307200、565 153600，逐字正确
//    剩下唯一没被测过的就是 **frame_read_write 内部**（两个厂商 FIFO 是加密 IP，
//    一直没有仿真台）。本台用 fifo_models.v 顶掉那两个 FIFO，把整帧写进去再读回来。
//
//  【判据】写第 k 个字 = {k[23:0] 的编码}，读回来的第 k 个像素必须等于它。
//    任何一个字丢失/重复/错位都会指到具体的 (x,y)。
//==============================================================================
//  ★ 状态：**未跑通（WIP）**。写引擎在本地 FIFO 模型上不发起突发
//    （`into_burst_r` 需要 `rdusedw > 128`，我的模型 rdusedw 语义还没对齐
//     厂商 IP 的保守欠报口径）。功能判据（逐字比对）已经写好，缺的是把
//     FIFO 模型的 `rdusedw/wrusedw/full/empty` 调到和 IP 一致。
//     在跑通之前，这个台的结论**不可引用**。
//==============================================================================
module tb_frame_rw888;

localparam integer NW = 307200;        // 888：一帧字数

reg clk = 0;
always #4 clk = ~clk;                  // 125MHz（三个域同频，CDC 天然安全）

reg  rst = 1;
reg  Sdr_init_done = 1;
reg  Sdr_init_ref_vld = 0;
reg  Sdr_busy = 0;
reg  rd_ready = 1;
reg  rd_busy_ext = 0;
reg  wr_ready = 1;
wire wr_busy_o;

wire        App_rd_en, App_rd_want;
wire [20:0] App_rd_addr;
reg         Sdr_rd_en = 0;
reg  [31:0] Sdr_rd_dout = 0;

reg         read_req = 0;
wire        read_req_ack;
wire        read_finish;
reg  [20:0] read_addr_0 = 21'd0;
reg  [20:0] read_addr_1 = 21'd307200;
reg  [20:0] read_addr_2 = 21'd0;
reg  [20:0] read_addr_3 = 21'd0;
reg  [1:0]  read_addr_index = 2'd0;
reg  [20:0] read_len = 21'd307200;
reg         read_en = 0;
wire [31:0] read_data;

wire        App_wr_en;
wire [20:0] App_wr_addr;
wire [31:0] App_wr_din;
wire [3:0]  App_wr_dm;

reg         write_req = 0;
wire        write_req_ack;
wire        write_finish;
reg  [20:0] write_addr_0 = 21'd0;
reg  [20:0] write_addr_1 = 21'd307200;
reg  [20:0] write_addr_2 = 21'd0;
reg  [20:0] write_addr_3 = 21'd0;
reg  [1:0]  write_addr_index = 2'd0;
reg  [20:0] write_len = 21'd307200;
reg         write_en = 0;
reg  [31:0] write_data = 0;

reg         fmt_wr = 0;
reg         fmt_rd = 0;

wire [9:0]  rd_avail;
wire        rd_fifo_empty, rd_fifo_full;
wire [9:0]  wr_fifo_used;
wire        wr_fifo_full;
wire [8:0]  rd_buf_used;
wire [9:0]  wr_inflight;
wire [1:0]  buf_fmt_o;
wire [4:0]  dbg_wr;

frame_read_write #(
    .MEM_DATA_BITS(32), .READ_DATA_BITS(32), .WRITE_DATA_BITS(32),
    .ADDR_BITS(21), .BURST_BITS(9), .BURST_SIZE(128),
    .WRITE_V_FLIP(1), .FRAME_WIDTH(640), .FRAME_HEIGHT(480),
    .INTERLOCK(1)
) dut (
    .rst(rst), .mem_clk(clk),
    .Sdr_init_done(Sdr_init_done), .Sdr_init_ref_vld(Sdr_init_ref_vld),
    .Sdr_busy(Sdr_busy),
    .App_rd_en(App_rd_en), .App_rd_want(App_rd_want), .App_rd_addr(App_rd_addr),
    .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout),
    .rd_ready(rd_ready), .rd_busy_ext(rd_busy_ext), .wr_ready(wr_ready),
    .wr_busy_o(wr_busy_o),
    .read_clk(clk), .read_req(read_req), .read_req_ack(read_req_ack),
    .read_finish(read_finish),
    .read_addr_0(read_addr_0), .read_addr_1(read_addr_1),
    .read_addr_2(read_addr_2), .read_addr_3(read_addr_3),
    .read_addr_index(read_addr_index), .read_len(read_len),
    .read_en(read_en), .read_data(read_data),
    .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr),
    .App_wr_din(App_wr_din), .App_wr_dm(App_wr_dm),
    .write_clk(clk), .write_req(write_req), .write_req_ack(write_req_ack),
    .write_finish(write_finish),
    .write_addr_0(write_addr_0), .write_addr_1(write_addr_1),
    .write_addr_2(write_addr_2), .write_addr_3(write_addr_3),
    .write_addr_index(write_addr_index), .write_len(write_len),
    .write_en(write_en), .write_data(write_data),
    .rd_avail(rd_avail), .rd_fifo_empty(rd_fifo_empty), .rd_fifo_full(rd_fifo_full),
    .wr_fifo_used(wr_fifo_used), .wr_fifo_full(wr_fifo_full),
    .rd_buf_used(rd_buf_used), .wr_inflight(wr_inflight),
    .fmt_wr(fmt_wr), .fmt_rd(fmt_rd), .buf_fmt_o(buf_fmt_o), .dbg_wr(dbg_wr)
);

//------------------------------------------------------------------------------
//  SDRAM 行为模型：写立即落地，读 2 拍延迟
//------------------------------------------------------------------------------
reg [31:0] sdram [0:2**21-1];
reg        rd_e1 = 0, rd_e2 = 0;
reg [20:0] rd_a1 = 0;
reg [31:0] rd_d  = 0;

always @(posedge clk) begin
    if (App_wr_en) sdram[App_wr_addr] <= App_wr_din;
    rd_e1 <= App_rd_en;
    if (App_rd_en) rd_a1 <= App_rd_addr;
    rd_e2 <= rd_e1;
    if (rd_e1) rd_d <= sdram[rd_a1];
end
always @(posedge clk) begin
    Sdr_rd_en   <= rd_e2;
    Sdr_rd_dout <= rd_d;
end

//------------------------------------------------------------------------------
//  图案：第 k 个字 = {8'hA5, k[23:16], k[15:8], k[7:0]}（任何错位都能指出来）
//------------------------------------------------------------------------------
function [31:0] pat;
    input integer k;
    reg [7:0] b0, b1, b2;
    begin
        b2 = k >> 16;
        b1 = k >> 8;
        b0 = k;
        pat = {8'hA5, b2, b1, b0};
    end
endfunction

integer nwr = 0;                    // 实际写进 SDRAM 的字数
always @(posedge clk) if (App_wr_en) nwr = nwr + 1;

integer fails = 0, checks = 0;
task chk;
    input [8*50:1] name;
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %0d", name, got);
        $fflush;
    end
endtask

//------------------------------------------------------------------------------
reg [31:0] rcap [0:NW-1];
integer    wk, rk, t, bad, q;
reg [7:0]  cx, cy;
reg [31:0] expw;

initial begin
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (20) @(posedge clk);

    $display("");
    $display("==================================================================");
    $display("  frame_read_write：888 整帧写入 → 读回（307200 字）");
    $display("==================================================================");

    // ---------------- 写 ----------------
    wk = 0;
    @(negedge clk);
    write_req = 1'b1;
    t = 0;
    while (!write_req_ack && t < 200) begin @(negedge clk); t = t + 1; end
    $display("  write_req_ack 来了（%0d 拍后）", t);
    @(negedge clk);
    write_req = 1'b0;

    // ★ 必须直接用 FIFO 的 full 标志门控。按水位（wr_inflight<400）门控
    //   在"检查→落地"之间还能挤进去，实测丢了 4 个字 ⇒ write_cnt=307072
    //   而 len_latch=307200 ⇒ 尾突发永远等不到 remaining ⇒ write_finish 不产生。
    while (wk < NW) begin
        if (!wr_fifo_full) begin
            write_en   <= 1'b1;
            write_data <= pat(wk);
            wk = wk + 1;
        end else begin
            write_en   <= 1'b0;
        end
        @(negedge clk);
    end
    write_en <= 1'b0;
    @(negedge clk);
    $display("  写完 %0d 字，等 write_finish ...", wk);

    t = 0;
    while (!write_finish && t < 2000000) begin @(negedge clk); t = t + 1; end
    $display("  write_finish = %b（等了 %0d 拍）", write_finish, t);
    $display("  实际写进 SDRAM = %0d 字；FIFO 剩 %0d；write_cnt=%0d len_latch=%0d",
             nwr, dut.rdusedw, dut.frame_fifo_write_m0.write_cnt,
             dut.frame_fifo_write_m0.write_len_latch);
    chk("W0 实际写入字数", nwr, NW);
    chk("W1 write_finish", write_finish, 1);

    // ---------------- 读 ----------------
    rk = 0;
    @(negedge clk);
    read_req = 1'b1;
    t = 0;
    while (!read_req_ack && t < 200) begin @(negedge clk); t = t + 1; end
    $display("  read_req_ack 来了（%0d 拍后）", t);
    @(negedge clk);
    read_req = 1'b0;

    // read_en = de_d[18]（每个有效像素都拉高），read_data 在**下一拍**被采样
    //   ★ 原来写成"每 2 拍记一个" ⇒ 每 2 拍弹了 2 个字却只记 1 个
    //     ⇒ 读回来的是隔一个的字（实测 got=a504ad83 对应 k=306563，步进 2）。
    read_en <= 1'b1;
    @(negedge clk);                     // t0：read_en 生效
    while (rk < NW) begin
        @(negedge clk);                 // 前进一拍
        rcap[rk] = read_data;           // 这一拍的 read_data 对应上一拍的 read_en
        rk = rk + 1;
    end
    read_en <= 1'b0;
    repeat (10) @(negedge clk);

    $display("  读回 %0d 像素", rk);
    chk("R0 像素数", rk, NW);

    bad = 0;
    for (q = 0; q < NW; q = q + 1) begin
        expw = pat(q);
        if (rcap[q] !== expw) begin
            if (bad < 8) begin
                cx = q % 640;
                cy = q / 640;
                $display("        [%0d] (x=%0d,y=%0d) got=%h exp=%h",
                         q, cx, cy, rcap[q], expw);
            end
            bad = bad + 1;
        end
    end
    chk("R1 逐字相等", bad, 0);

    // 每行最后一列单独统计
    begin : lastcol
        integer y, eb;
        eb = 0;
        for (y = 0; y < 480; y = y + 1)
            if (rcap[y*640 + 639] !== pat(y*640 + 639)) eb = eb + 1;
        chk("R2 每行第 639 列", eb, 0);
    end

    $display("");
    $display("==================================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED: %0d / %0d", fails, checks);
    $display("==================================================================");
    $finish;
end

// 诊断：每 200000 拍打一次内部状态
always @(posedge clk) begin
    if ((t % 200000) == 0 && t != 0)
        $display("   [dbg t=%0d] wk=%0d rdusedw=%0d wrused=%0d busy=%b state=%0d",
                 t, wk, dut.rdusedw, dut.wr_fifo_used, wr_busy_o,
                 dut.frame_fifo_write_m0.state);
end

initial begin
    #80_000_000;
    $display("TIMEOUT wk=%0d rk=%0d busy=%b", wk, rk, wr_busy_o);
    $finish;
end

endmodule
