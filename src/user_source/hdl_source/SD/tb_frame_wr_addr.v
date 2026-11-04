`timescale 1ns/1ps
//==============================================================================
//  tb_frame_wr_addr.v —— frame_fifo_write 的写使能 / 地址序列是否一字不差
//==============================================================================
//  【为什么要这个台】
//    上板现象：**HD（888）右边有一整列 1px 黑线，565 没有**（565 只有右上角
//    一个黑点，那是帧尾排水那条，另有其事）。
//
//    这个"只有 888 有"的形状有一个非常具体的解释：
//      · 888 一行 = 640 字 = **正好 5 个 128 字突发** ⇒ 行尾正好压在 burst 边界
//      · 565 一行 = 320 字 = 2.5 个突发     ⇒ 行尾落在 burst 中间
//    如果突发收尾丢一个字，就只有 888 会在**每行第 639 列**留一个洞 ——
//    而帧缓存从不预清，那个字永远保持上电值 0 ⇒ 一条永久黑列。
//
//  【做法】把 frame_fifo_write 单独拎出来跑：让 rdusedw 恒有货（数据永远够），
//    然后逐拍记录 App_wr_en / App_wr_addr，与"教科书地址序列"逐字比对。
//      · 期望字数：888 = 307200，565 = 153600
//      · 期望地址：row = 479 - k/行宽，col = k%行宽，addr = row*行宽 + col
//    任何一个字被丢/写错地址都会立刻报出来，并指出是第几行第几列。
//==============================================================================
module tb_frame_wr_addr;

reg clk = 0;
always #4 clk = ~clk;                  // 125MHz

reg rst = 1;
reg  Sdr_init_done = 1;
reg  Sdr_init_ref_vld = 0;
reg  Sdr_busy = 0;
reg  App_rd_busy = 0;
reg  wr_ready = 1;
wire O_wr_busy;
wire App_wr_en;
wire [20:0] App_wr_addr;
reg  write_req = 0;
wire write_req_ack;
wire write_finish;
reg  [20:0] write_addr_0 = 21'd0;
reg  [20:0] write_addr_1 = 21'd307200;
reg  [20:0] write_addr_2 = 21'd0;
reg  [20:0] write_addr_3 = 21'd0;
reg  [1:0]  write_addr_index = 2'd0;
reg  [20:0] write_len = 21'd307200;
wire fifo_aclr;
reg  [9:0]  rdusedw = 10'd500;         // 恒有货
reg         fmt565 = 0;
wire [4:0]  dbg_wr;

frame_fifo_write #(
    .MEM_DATA_BITS(32), .ADDR_BITS(21), .BURST_BITS(9), .BURST_SIZE(128),
    .WRITE_V_FLIP(1), .FRAME_WIDTH(640), .FRAME_HEIGHT(480)
) dut (
    .rst(rst), .mem_clk(clk),
    .Sdr_init_done(Sdr_init_done), .Sdr_init_ref_vld(Sdr_init_ref_vld),
    .Sdr_busy(Sdr_busy), .App_rd_busy(App_rd_busy), .O_wr_busy(O_wr_busy),
    .wr_ready(wr_ready),
    .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr),
    .write_req(write_req), .write_req_ack(write_req_ack), .write_finish(write_finish),
    .write_addr_0(write_addr_0), .write_addr_1(write_addr_1),
    .write_addr_2(write_addr_2), .write_addr_3(write_addr_3),
    .write_addr_index(write_addr_index),
    .write_len(write_len),
    .fifo_aclr(fifo_aclr),
    .rdusedw(rdusedw),
    .fmt565(fmt565),
    .dbg_wr(dbg_wr)
);

integer fails = 0, checks = 0;
task chk;
    input [8*52:1] name;
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
//  捕获写流
//------------------------------------------------------------------------------
reg  [20:0] acap [0:307199];
integer     nw;
integer     q, row, col, exp_addr, bad;

task run_frame;
    input         f565;
    input integer len_pix;
    input integer row_words;
    integer t;
    begin
        nw = 0;
        fmt565     = f565;
        write_len  = len_pix;
        @(negedge clk);
        write_req = 1'b1;
        // 等 ack 后拉低
        t = 0;
        while (!write_req_ack && t < 100) begin @(negedge clk); t = t + 1; end
        @(negedge clk);
        write_req = 1'b0;
        // 等 write_finish
        t = 0;
        while (!write_finish && t < 4000000) begin @(negedge clk); t = t + 1; end
        repeat (20) @(negedge clk);
    end
endtask

always @(posedge clk) begin
    if (App_wr_en && (nw < 307200)) begin
        acap[nw] = App_wr_addr;
        nw = nw + 1;
    end else if (App_wr_en) begin
        nw = nw + 1;
    end
end

initial begin
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (10) @(posedge clk);

    //==================================================================
    $display("");
    $display("==================================================================");
    $display("  A. 888：一行 640 字（= 正好 5 个 128 字突发）");
    $display("==================================================================");
    run_frame(1'b0, 307200, 640);
    $display("  写使能拍数 = %0d (应为 307200)", nw);
    chk("A0 字数", nw, 307200);
    if (nw == 307200) begin
        bad = 0;
        for (q = 0; q < 307200; q = q + 1) begin
            row = 479 - (q / 640);
            col = q % 640;
            exp_addr = row * 640 + col;
            if (acap[q] !== exp_addr[20:0]) begin
                if (bad < 10)
                    $display("        [%0d] (row=%0d,col=%0d) got=%0d exp=%0d",
                             q, row, col, acap[q], exp_addr);
                bad = bad + 1;
            end
        end
        chk("A1 地址逐字", bad, 0);
        // 每行最后一列单独看
        begin : aedge
            integer y, eb;
            eb = 0;
            // 注意 V-flip：第 q 个字属于内存行 (479 - q/640)
            for (y = 0; y < 480; y = y + 1)
                if (acap[y*640 + 639] !== ((479 - y)*640 + 639)) eb = eb + 1;
            chk("A2 每行第 639 字地址", eb, 0);
        end
    end

    //==================================================================
    $display("");
    $display("==================================================================");
    $display("  B. 565：一行 320 字（行尾落在突发中间）");
    $display("==================================================================");
    run_frame(1'b1, 307200, 320);
    $display("  写使能拍数 = %0d (应为 153600)", nw);
    chk("B0 字数", nw, 153600);
    if (nw == 153600) begin
        bad = 0;
        for (q = 0; q < 153600; q = q + 1) begin
            row = 479 - (q / 320);
            col = q % 320;
            exp_addr = row * 320 + col;
            if (acap[q] !== exp_addr[20:0]) begin
                if (bad < 10)
                    $display("        [%0d] (row=%0d,col=%0d) got=%0d exp=%0d",
                             q, row, col, acap[q], exp_addr);
                bad = bad + 1;
            end
        end
        chk("B1 地址逐字", bad, 0);
        begin : bedge
            integer y, eb;
            eb = 0;
            for (y = 0; y < 480; y = y + 1)
                if (acap[y*320 + 319] !== ((479 - y)*320 + 319)) eb = eb + 1;
            chk("B2 每行第 319 字地址", eb, 0);
        end
    end

    $display("");
    $display("==================================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED: %0d / %0d", fails, checks);
    $display("==================================================================");
    $finish;
end

endmodule
