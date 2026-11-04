`timescale 1ns/1ps
//==============================================================================
//  tb_showahead.v —— 钉死 wfifo_32_32_512（show-ahead）见底时序
//  问题：100M-23 的 fifo_has = (rdusedw != 0) 在见底那拍会不会把
//       "当前 dout 字"连同"它的命令"一起吞掉 / 冻结计数？
//  这里灌 3 个字，逐拍打印 rdusedw / dout / re / rd_en_s，观察消费最后一字
//  的下一拍 dout 是保持还是失效、rdusedw 何时归 0。
//==============================================================================
module tb_showahead;

    reg  clkr = 0, clkw = 0, rst = 1;
    reg  we = 0, re = 0;
    reg  [31:0] di = 0;
    wire [31:0] dout;
    wire [9:0]  rdusedw;
    wire [9:0]  wrusedw;
    wire        empty_flag, full_flag;

    // 125MHz 读 / 100MHz 写（与真机一致；这里纯看读侧行为，写侧先灌满）
    always #4 clkr = ~clkr;   // 125MHz
    always #5 clkw = ~clkw;   // 100MHz

    wfifo_32_32_512 dut (
        .clkr(clkr), .clkw(clkw), .rst(rst),
        .we(we), .re(re), .di(di),
        .empty_flag(empty_flag), .full_flag(full_flag),
        .wrusedw(wrusedw), .rdusedw(rdusedw), .dout(dout)
    );

    integer i;
    initial begin
        $display(" time | rst we re | rdusedw empty | dout       | note");
        $display("------+-----------+---------------+------------+------");
        // ---- 复位 ----
        rst = 1; we = 0; re = 0; di = 0;
        repeat (4) @(posedge clkr);
        rst = 0;
        @(posedge clkw);

        // ---- 灌 3 个字：A1 A2 A3 ----
        we = 1;
        di = 32'hA1A1_0001; @(posedge clkw);
        di = 32'hA2A2_0002; @(posedge clkw);
        di = 32'hA3A3_0003; @(posedge clkw);
        we = 0; di = 0;
        repeat (3) @(posedge clkr);   // 等跨时钟域稳定

        $display("=== 灌完 3 字，开始逐拍读（观察见底） ===");
        // ---- 逐拍读，连续 re，观察见底那几拍 ----
        for (i = 0; i < 8; i = i + 1) begin
            @(negedge clkr);          // 在 negedge 设 re，稳定到 posedge 采样
            re = (i < 4);             // 前 4 拍给 re（会读过头，正好触发见底）
            @(posedge clkr);
            $display(" %4t |  %b  %b  %b |    %3d     %b | %h | re=%b 第%0d拍",
                     $time, rst, we, re, rdusedw, empty_flag, dout, re, i);
        end
        re = 0;
        @(posedge clkr);
        $display("=== 停 re，看 dout/rdusedw 末态 ===");
        $display(" %4t |  %b  %b  %b |    %3d     %b | %h | 末态",
                 $time, rst, we, re, rdusedw, empty_flag, dout);
        $finish;
    end

endmodule
