`timescale 1ns/1ps
//==============================================================================
//  tb_frame_rw.v —— frame_read_write 的单元级回归
//
//  【为什么要写这个】
//  2026-10-15 为修"会移动的横条纹"改了 frame_fifo_write（into_burst 拆两级
//  浅流水寄存）和 frame_fifo_read（wrusedw 打一拍）。这两个模块原先**没有
//  任何仿真覆盖**（tb_app 只是用假下游复刻握手节奏），改完必须有一层功能网。
//
//  【测什么】
//    1. 写通路：3 次突发写完 768 个字，数据/地址逐个正确，write_finish 到来；
//    2. 读通路：读回同一缓冲区，read_data 顺序逐字正确，read_finish 到来；
//    3. ★ 仲裁：App_wr_en 和 App_rd_en 在同一个 mem_clk 沿**绝不同时为 1**
//       （读写突发重叠 = 单口 SDRAM 必然丢数据，这就是"写优先"的意义）；
//    4. 并发压力：写新帧的同时读旧帧（双缓冲的真实用法），两边都要完整跑完；
//    5. remaining_r 跨帧：每帧 S_ACK 重新锁长度，尾条件寄存器必须跟着更新。
//
//  【SDRAM 响应器】
//  行为级：App_wr_en 采样进 mem[]；App_rd_en 每个脉冲延迟 SDR_LAT 拍回一个
//  Sdr_rd_en + mem[命令时刻的地址]（真实 CAS 延迟 10 拍内，rd_delay 数到 10）。
//
//  【已知既有限制（不是本 TB 造出来的）】
//  写状态机的尾突发要求 write_len 是 BURST_SIZE(256) 的倍数，否则
//  burst_cnt 冻结在 256 以下、wr_burst_finish 永不成立、状态机卡死。
//  真实工程 write_len = 640×480 = 307200 = 1200×256，恰好安全。
//  所以本 TB 只用 256 的倍数当帧长。
//==============================================================================
module tb_frame_rw;

    // ---- 参数 ----
    localparam ADDR_BITS = 21;
    localparam FRAME_LEN = 768;                 // 3 × 256
    localparam BASE_A    = 21'h0_0000;          // 帧 A 缓冲区
    localparam BASE_B    = 21'h2_0000;          // 帧 B 缓冲区
    localparam SDR_LAT   = 8;                   // SDRAM 读延迟（拍）

    // ---- 时钟：写 83.33MHz（App 域现状）、mem 125MHz、读 25MHz ----
    reg mem_clk = 0, write_clk = 0, read_clk = 0;
    always #4  mem_clk   = ~mem_clk;
    always #6  write_clk = ~write_clk;
    always #20 read_clk  = ~read_clk;

    // ---- 控制/状态 ----
    reg rst = 1;
    reg Sdr_init_done = 0, Sdr_init_ref_vld = 0, Sdr_busy = 0;

    // ---- DUT 端口 ----
    wire        App_rd_en;
    wire [ADDR_BITS-1:0] App_rd_addr;
    reg         Sdr_rd_en = 0;
    reg  [31:0] Sdr_rd_dout = 0;
    reg         read_req = 0, read_en = 0;
    reg  [1:0]  rd_idx = 2'd0, wr_idx = 2'd0;
    wire        read_req_ack, read_finish;
    wire [31:0] read_data;
    wire        App_wr_en, write_req_ack, write_finish;
    wire [ADDR_BITS-1:0] App_wr_addr;
    wire [31:0] App_wr_din;
    wire [3:0]  App_wr_dm;
    reg         write_req = 0, write_en = 0;
    reg  [31:0] write_data = 0;

    frame_read_write #(.WRITE_V_FLIP(0)) dut (
        .rst(rst), .mem_clk(mem_clk),
        .Sdr_init_done(Sdr_init_done), .Sdr_init_ref_vld(Sdr_init_ref_vld), .Sdr_busy(Sdr_busy),
        .App_rd_en(App_rd_en), .App_rd_addr(App_rd_addr),
        .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout),
        .read_clk(read_clk), .read_req(read_req), .read_req_ack(read_req_ack),
        .read_finish(read_finish),
        .read_addr_0(BASE_A), .read_addr_1(BASE_B), .read_addr_2(21'd0), .read_addr_3(21'd0),
        .read_addr_index(rd_idx), .read_len(FRAME_LEN), .read_en(read_en), .read_data(read_data),
        .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr), .App_wr_din(App_wr_din), .App_wr_dm(App_wr_dm),
        .write_clk(write_clk), .write_req(write_req), .write_req_ack(write_req_ack),
        .write_finish(write_finish),
        .write_addr_0(BASE_A), .write_addr_1(BASE_B), .write_addr_2(21'd0), .write_addr_3(21'd0),
        .write_addr_index(wr_idx), .write_len(FRAME_LEN),
        .write_en(write_en), .write_data(write_data)
    );

    //==========================================================================
    //  行为级 SDRAM
    //==========================================================================
    reg [31:0] mem [0:(1<<ADDR_BITS)-1];

    // 写：采样进阵列
    always @(posedge mem_clk)
        if (!rst && App_wr_en) mem[App_wr_addr] <= App_wr_din;

    // 读：每个 App_rd_en 脉冲，延迟 SDR_LAT 拍后回一个数据
    reg        rsp_v [0:SDR_LAT];
    reg [31:0] rsp_d [0:SDR_LAT];
    integer ri;
    initial for (ri = 0; ri <= SDR_LAT; ri = ri + 1) rsp_v[ri] = 1'b0;
    always @(posedge mem_clk) begin
        if (!rst && App_rd_en) begin
            rsp_v[0] <= 1'b1;
            rsp_d[0] <= mem[App_rd_addr];
        end else
            rsp_v[0] <= 1'b0;
        for (ri = 1; ri <= SDR_LAT; ri = ri + 1) begin
            rsp_v[ri] <= rsp_v[ri-1];
            rsp_d[ri] <= rsp_d[ri-1];
        end
    end
    always @(*) begin
        Sdr_rd_en   = rsp_v[SDR_LAT];
        Sdr_rd_dout = rsp_d[SDR_LAT];
    end

    //==========================================================================
    //  检查器
    //==========================================================================
    integer errors = 0;

    // ① 仲裁：读写突发绝不能重叠（同一 mem_clk 沿两个使能都为 1）
    always @(posedge mem_clk)
        if (!rst && App_wr_en && App_rd_en) begin
            errors = errors + 1;
            $display("[%0t] FAIL 仲裁违规：App_wr_en 与 App_rd_en 同拍为 1！", $time);
        end

    // ② 写数据/地址：逐字校验（期望值由激励按帧重置）
    reg [ADDR_BITS-1:0] exp_waddr = 0;
    reg [31:0]          exp_wdata = 0;
    integer dbg_prints = 0;
    always @(posedge mem_clk)
        if (!rst && App_wr_en) begin
            if (App_wr_addr !== exp_waddr || App_wr_din !== exp_wdata) begin
                errors = errors + 1;
                if (dbg_prints < 6) begin
                    dbg_prints = dbg_prints + 1;
                    $display("[%0t] FAIL 写校验：addr 得 %h 期 %h；data 得 %h 期 %h（fifo rd_addr=%h rdusedw=%h）",
                             $time, App_wr_addr, exp_waddr, App_wr_din, exp_wdata,
                             dut.write_buf.rd_addr, dut.rdusedw);
                end
            end
            exp_waddr = exp_waddr + 1'b1;
            exp_wdata = exp_wdata + 1'b1;
        end

    // ③ 读数据：rfifo 是非 show-ahead（SHOW_AHEAD_EN=0）——re 那一拍的下一拍
    //    dout 才是要读的字。所以用 ren_q（上一拍的 re）对齐采样。
    reg [31:0] exp_rdata = 0;
    reg        ren_q = 0;
    always @(posedge read_clk) begin
        ren_q <= read_en;
        if (ren_q && !rst) begin
            if (read_data !== exp_rdata) begin
                errors = errors + 1;
                $display("[%0t] FAIL 读校验：期望 %h，得到 %h", $time, exp_rdata, read_data);
            end
            exp_rdata = exp_rdata + 1'b1;
        end
    end

    //==========================================================================
    //  激励
    //==========================================================================
    // 写一帧：灌 FRAME_LEN 个字（seed 起），等 write_finish。
    // 落哪个缓冲区由 wr_idx 决定（调用前设置），期望值也由调用方先摆好。
    // 【握手契约】ack 在 S_ACK 期间保持为高，而 S_ACK 同时拉着 fifo_aclr——
    // 必须等 ack 撤销（S_ACK 退出）之后才能开始灌数据，否则开头几个字
    // 会撞进 FIFO 复位窗口被丢掉。
    task do_write_frame(input [31:0] seed);
        integer k;
        begin
            @(posedge write_clk);
            write_req <= 1'b1;
            wait (write_req_ack === 1'b1);      // S_ACK 应答
            @(posedge write_clk);
            write_req <= 1'b0;                  // 撤请求 → FSM 进入搬运循环
            wait (write_req_ack === 1'b0);      // ★ 等 S_ACK 真正退出（aclr 释放）
            repeat (2) @(posedge write_clk);
            for (k = 0; k < FRAME_LEN; k = k + 1) begin
                @(posedge write_clk);
                write_en   <= 1'b1;
                write_data <= seed + k;
            end
            @(posedge write_clk);
            write_en <= 1'b0;
            wait (write_finish === 1'b1);
            @(posedge write_clk);
            $display("[%0t] 写帧完成（seed=%h）", $time, seed);
        end
    endtask

    // 读一帧：等第一批 SDRAM 数据回来后再持续拉 read_en，等 read_finish。
    // 读哪个缓冲区由 rd_idx 决定，期望值由调用方先摆好。
    // 【注意】read_finish 在"SDRAM 读命令发完"时就脉冲（单周期），
    // 常常早于显示侧拉完数据——不能事后 wait 它，用监视器抓。
    reg saw_rfinish = 0;
    always @(posedge mem_clk)
        if (read_finish) saw_rfinish = 1;

    task do_read_frame;
        integer k;
        begin
            saw_rfinish = 0;
            @(posedge read_clk);
            read_req <= 1'b1;
            wait (read_req_ack === 1'b1);
            repeat (20) @(posedge read_clk);    // 保持足够久，让 d2 同步链看到撤请求
            read_req <= 1'b0;
            wait (Sdr_rd_en === 1'b1);          // 第一批数据在路上/已到达
            // 数据写进 FIFO 是 mem_clk 域事件，empty_flag 经 2 级同步才在
            // read_clk 域可见——等它真正报"非空"再拉 re，否则开头几个 re
            // 会被 FIFO 忽略（空读不弹出），整条序列就错位了。
            wait (dut.read_buf.empty_flag === 1'b0);
            @(posedge read_clk);
            for (k = 0; k < FRAME_LEN; k = k + 1) begin
                @(posedge read_clk);
                read_en <= 1'b1;
            end
            @(posedge read_clk);
            read_en <= 1'b0;
            // 等 FIFO 排空（最后一个字也读出来了）
            wait (dut.read_buf.empty_flag === 1'b1);
            repeat (10) @(posedge read_clk);
            if (!saw_rfinish) begin
                errors = errors + 1;
                $display("[%0t] FAIL 读帧没等到 read_finish 脉冲", $time);
            end
            $display("[%0t] 读帧完成", $time);
        end
    endtask

    initial begin
        rst = 1;
        repeat (20) @(posedge mem_clk);
        rst = 0;
        Sdr_init_done = 1;
        repeat (10) @(posedge mem_clk);

        //--------------------------------------------------------------
        // 帧 A：写 BASE_A（index=0，seed 0）→ 读回 BASE_A
        //--------------------------------------------------------------
        $display("=== 帧 A：顺序写 + 读回（main_ready_r 反复触发）===");
        wr_idx = 2'd0;  rd_idx = 2'd0;
        exp_waddr = BASE_A;  exp_wdata = 32'h0;
        exp_rdata = 32'h0;
        do_write_frame(32'h0);
        begin : check_a
            integer k;
            for (k = 0; k < FRAME_LEN; k = k + 1)
                if (mem[BASE_A + k] !== k[31:0]) begin
                    errors = errors + 1;
                    $display("FAIL 帧 A mem[%h]=%h 期望 %h", BASE_A + k, mem[BASE_A + k], k);
                end
        end
        do_read_frame;

        //--------------------------------------------------------------
        // 帧 B：写 BASE_B（index=1，seed 0x10000）的同时读 BASE_A（并发压力）
        //--------------------------------------------------------------
        $display("=== 帧 B：写 BASE_B 与读 BASE_A 并发（写优先仲裁）===");
        wr_idx = 2'd1;
        exp_waddr = BASE_B;  exp_wdata = 32'h10000;
        exp_rdata = 32'h0;                       // 读的是帧 A 的旧数据
        fork
            do_write_frame(32'h10000);
            do_read_frame;
        join
        // 帧 B 写内容独立复核
        begin : check_b
            integer k;
            for (k = 0; k < FRAME_LEN; k = k + 1)
                if (mem[BASE_B + k] !== 32'h10000 + k) begin
                    errors = errors + 1;
                    $display("FAIL 帧 B mem[%h]=%h 期望 %h", BASE_B + k, mem[BASE_B + k], 32'h10000 + k);
                end
        end

        //--------------------------------------------------------------
        // 帧 C：读回 BASE_B，确认帧 B 内容
        //--------------------------------------------------------------
        $display("=== 帧 C：读回 BASE_B ===");
        rd_idx = 2'd1;
        exp_rdata = 32'h10000;
        do_read_frame;
        rd_idx = 2'd0;

        //--------------------------------------------------------------
        // 总结
        //--------------------------------------------------------------
        repeat (100) @(posedge mem_clk);
        if (errors == 0)
            $display("\n*** tb_frame_rw 全部通过 ***");
        else
            $display("\n*** tb_frame_rw 失败：%0d 个错误 ***", errors);
        $finish;
    end

    // 看门狗
    initial begin
        #2_000_000;
        $display("*** tb_frame_rw 超时未完成（2ms 仿真时间）***");
        $finish;
    end



endmodule
