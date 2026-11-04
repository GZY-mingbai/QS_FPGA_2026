`timescale 1ns/1ps
//==============================================================================
//  tb_fw_nolock.v —— 验证修复后写引擎【不再见底死锁】
//  用真实 wfifo + frame_fifo_write，灌一帧 307200 字（带停顿，模拟图层 stall），
//  判据：write_finish 必须来、发出的命令总数必须 == 307200（一条不多不少）。
//  旧 bug（100M-23 fifo_has=rdusedw!=0）会卡死在 S_WRITE_BURST，write_finish 永不来。
//==============================================================================
module tb_fw_nolock;

    // ---- 时钟 / 复位 ----
    reg mem_clk = 0, wr_clk = 0, rst = 1;
    always #4 mem_clk = ~mem_clk;   // 125MHz
    always #5 wr_clk   = ~wr_clk;   // 100MHz

    // ---- FIFO 侧 ----
    reg         we = 0;
    reg  [31:0] di = 0;
    wire [31:0] dout;
    wire [8:0]  rdusedw9, wrusedw9;
    wire        empty_f, full_f;

    // ---- frame_fifo_write 侧 ----
    reg         wr_ready = 1'b1;
    reg         write_req = 0;
    wire        write_req_ack, write_finish, fifo_aclr;
    wire        App_wr_en;
    wire [20:0] App_wr_addr;
    reg  [20:0] write_len = 21'd307200;

    localparam integer NPX = 307200;

    // 写 FIFO（show-ahead）
    wfifo_32_32_512 write_buf (
        .clkr(mem_clk), .clkw(wr_clk), .rst(fifo_aclr),
        .we(we), .re(App_wr_en), .di(di),
        .empty_flag(empty_f), .full_flag(full_f),
        .wrusedw(wrusedw9), .rdusedw(rdusedw9), .dout(dout)
    );

    // 写引擎
    frame_fifo_write #(
        .MEM_DATA_BITS(32), .ADDR_BITS(21), .BURST_BITS(9),
        .BURST_SIZE(128), .WRITE_V_FLIP(1),
        .FRAME_WIDTH(640), .FRAME_HEIGHT(480)
    ) dut (
        .rst(rst), .mem_clk(mem_clk),
        .Sdr_init_done(1'b1), .Sdr_init_ref_vld(1'b0),
        .Sdr_busy(1'b0), .App_rd_busy(1'b0), .O_wr_busy(),
        .wr_ready(wr_ready),
        .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr),
        .write_req(write_req), .write_req_ack(write_req_ack),
        .write_finish(write_finish),
        .write_addr_0(21'd0), .write_addr_1(21'd307200),
        .write_addr_2(21'd614400), .write_addr_3(21'd921600),
        .write_addr_index(2'd0), .write_len(write_len),
        .fifo_aclr(fifo_aclr), .rdusedw({1'b0, rdusedw9})
    );

    // ---- 命令计数（在 mem_clk 域数 App_wr_en）----
    integer cmd_cnt = 0;
    always @(posedge mem_clk) if (!rst && App_wr_en) cmd_cnt = cmd_cnt + 1;

    // ---- 灌像素（wr_clk 域，带随机停顿模拟图层 stall）----
    integer pushed = 0;
    integer seed = 1;
    integer i;
    initial begin
        $display("=== tb_fw_nolock：验证写引擎不死锁 ===");
        rst = 1; we = 0; di = 0; write_req = 0;
        repeat (10) @(posedge wr_clk);
        rst = 0;
        repeat (5) @(posedge wr_clk);

        // 发起写请求
        write_req = 1;
        $display("[%0t] write_req=1，等 write_req_ack...", $time);
        fork
            begin : hs
                wait (write_req_ack == 1'b1);
                $display("[%0t] 收到 write_req_ack=1，撤销请求", $time);
                @(posedge wr_clk);
                write_req = 0;                  // 撤销请求
                wait (write_req_ack == 1'b0);   // ack 落下 = FIFO 清完，可灌数据
                $display("[%0t] write_req_ack 落下，开始灌 %0d 像素", $time, NPX);
            end
            begin : hs_to
                #2_000_000;
                $display("[%0t] ❌ 握手超时！write_req_ack=%b state=%0d",
                         $time, write_req_ack, dut.state);
                $finish;
            end
        join_any
        disable hs_to;

        // 灌 NPX 个字，随机停顿（模拟图层 pixel_ready 抖动）
        for (i = 0; i < NPX; i = i + 1) begin
            @(posedge wr_clk);
            // 随机停顿：约 1/8 概率停 1~3 拍（制造 FIFO 见底/近底）
            if (($random(seed) % 8) == 0) begin
                we = 0;
                repeat (1 + ({$random(seed)} % 3)) @(posedge wr_clk);
            end
            we = 1;
            di = i[31:0];
            pushed = pushed + 1;
        end
        @(posedge wr_clk);
        we = 0;

        // ---- 等 write_finish（关键判据：必须来！）----
        fork
            begin : wait_fin
                wait (write_finish == 1'b1);
                $display(">>> write_finish 到达！命令总数 = %0d（期望 %0d）", cmd_cnt, NPX);
                if (cmd_cnt == NPX)
                    $display(">>> ✅ PASS：命令数精确 307200，写引擎未死锁、无多写/少写");
                else if (cmd_cnt < NPX)
                    $display(">>> ❌ FAIL：少写 %0d 条（写引擎提前收工）", NPX - cmd_cnt);
                else
                    $display(">>> ❌ FAIL：多写 %0d 条（写引擎超发）", cmd_cnt - NPX);
                $finish;
            end
            begin : timeout
                #40_000_000;  // 40ms 上限（一帧约 6ms，给足余量）
                $display(">>> ❌ FAIL：超时！write_finish 永远没来 —— 写引擎【死锁】了");
                $display(">>>   卡死时：cmd_cnt=%0d / %0d，state=%0d，App_wr_en_r=%b",
                         cmd_cnt, NPX, dut.state, dut.App_wr_en_r);
                $finish;
            end
        join
    end

endmodule
