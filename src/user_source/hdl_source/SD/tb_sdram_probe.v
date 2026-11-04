`timescale 1ns/1ps
//==============================================================================
//  tb_sdram_probe.v —— SDRAM 替身模型（sdr_as_ram_sim）的验证探针
//==============================================================================
//
//  【为什么要这个 tb】
//  真实的 SDRAM 控制器（sdr_as_ram.enc.v）是加密的，iverilog 编译不了，
//  本机也没有 ModelSim（TD 的仿真流程外包给 ModelSim）。所以工程里
//  **没有任何可跑的 SDRAM 功能仿真**。
//
//  这个 tb 验证我们自己的替身模型 sdr_as_ram_sim，并且把后续阶段要依赖的
//  三件事**钉死**：
//      · 读延迟实测值（后续 ps_gen_image / 仲裁器的延迟对齐要用它）
//      · 读返回是否保序（纹理 FIFO 的"两个计数器配对"依赖它）
//      · 连续读命令的吞吐（决定仲裁批次大小能不能撑 60fps）
//
//  【测什么】
//    1. 顺序写 256 字 → 顺序读回，逐字比对
//    2. 跨行散列地址写读（256 对齐 / 间隔 1024 / 间隔 4096）→ 比对
//    3. 随机地址写读 512 次 → 比对
//    4. 四个真实区域基址（BUF0/BUF1/TEX0/TEX1）互不混叠
//    5. 读延迟实测 = RD_LATENCY 拍
//    6. 读返回保序（背靠背连读 256 个字，顺序逐字比对）
//    7. 背靠背连读 256 个命令的总周期数 → 实测吞吐（命令/拍）
//    8. 同拍既读又写 → rw_conflict 拉高
//
//  【采样约定（沿用 SD/tb_frame_rw.v）】
//    · 激励：`@(posedge clk); sig <= val;`（非阻塞，保证下一个沿采到的是新值）
//    · 采样：`@(negedge clk);`（此时本拍所有 NBA 都已落定，无竞争）
//
//  【怎么跑】（在工程根目录）
//    iverilog -g2005 -o tb_sdram_probe.vvp \
//        src/user_source/hdl_source/SD/model/sdr_as_ram_sim.v \
//        src/user_source/hdl_source/SD/tb_sdram_probe.v
//    vvp tb_sdram_probe.vvp
//
//==============================================================================

module tb_sdram_probe;

//==============================================================================
//  参数
//==============================================================================
localparam RD_LAT     = 10;                 // 与 sdr_as_ram_sim 的 RD_LATENCY 一致
localparam INIT_CYC   = 200;
localparam MEM_WORDS  = 2097152;

// 真实内存布局（与计划 §2 一致）
localparam [20:0] BUF0 = 21'd0;             // 0 .. 307199
localparam [20:0] BUF1 = 21'd307200;        // 307200 .. 614399
localparam [20:0] TEX0 = 21'd614400;        // 614400 .. 921599
localparam [20:0] TEX1 = 21'd921600;        // 921600 .. 1228799

//==============================================================================
//  时钟 / 复位
//==============================================================================
reg clk = 0;
always #4 clk = ~clk;                        // 周期 8ns = 125MHz

reg rst = 1;

//==============================================================================
//  DUT
//==============================================================================
wire        Sdr_init_done, Sdr_init_ref_vld, Sdr_busy;
reg         App_wr_en   = 0;
reg  [20:0] App_wr_addr = 21'd0;
reg  [3:0]  App_wr_dm   = 4'b0000;
reg  [31:0] App_wr_din  = 32'd0;
reg         App_rd_en   = 0;
reg  [20:0] App_rd_addr = 21'd0;
wire        Sdr_rd_en;
wire [31:0] Sdr_rd_dout;
wire        rw_conflict;

sdr_as_ram_sim #(
    .RD_LATENCY  (RD_LAT),
    .INIT_CYCLES (INIT_CYC),
    .MEM_WORDS   (MEM_WORDS)
) dut (
    .Sdr_clk          (clk),
    .Sdr_clk_sft      (clk),                 // 本模型不用，接同一个时钟即可
    .Rst              (rst),
    .Sdr_init_done    (Sdr_init_done),
    .Sdr_init_ref_vld (Sdr_init_ref_vld),
    .Sdr_busy         (Sdr_busy),
    .App_ref_req      (1'b0),
    .App_wr_en        (App_wr_en),
    .App_wr_addr      (App_wr_addr),
    .App_wr_dm        (App_wr_dm),
    .App_wr_din       (App_wr_din),
    .App_rd_en        (App_rd_en),
    .App_rd_addr      (App_rd_addr),
    .Sdr_rd_en        (Sdr_rd_en),
    .Sdr_rd_dout      (Sdr_rd_dout),
    .rw_conflict      (rw_conflict)
);

//==============================================================================
//  周期计数与错误计数
//==============================================================================
integer cyc = 0;
always @(posedge clk) cyc = cyc + 1;

integer errors    = 0;
integer n_checked = 0;

//==============================================================================
//  诊断计数器（只在背靠背连读测试里使能）
//==============================================================================
//  按 posedge 采样，数的是"下游在时钟沿看到的"脉冲数 —— 和真实消费者一致。
//==============================================================================
integer dbg_cmd_cnt = 0;          // DUT 实际收到的读命令数
integer dbg_dat_cnt = 0;          // DUT 实际吐出的数据数
reg     dbg_en      = 1'b0;
reg     dbg_armed   = 1'b0;
always @(posedge clk) begin
    if (dbg_en && App_rd_en) begin
        dbg_cmd_cnt = dbg_cmd_cnt + 1;
        if (!dbg_armed) begin
            dbg_armed = 1'b1;
            $display("      [dbg] 首个被采样的读命令：addr=0x%05h（期望 0x%05h）",
                     App_rd_addr, TEX0 + 21'd1000);
        end
    end
    if (dbg_en && Sdr_rd_en) dbg_dat_cnt = dbg_dat_cnt + 1;
end

//==============================================================================
//  任务：单次写
//==============================================================================
task automatic sdr_write(input [20:0] a, input [31:0] d);
    begin
        @(posedge clk);
        App_wr_addr <= a;
        App_wr_din  <= d;
        App_wr_dm   <= 4'b0000;
        App_wr_en   <= 1'b1;
        @(posedge clk);
        App_wr_en   <= 1'b0;
    end
endtask

//==============================================================================
//  任务：单次读，返回数据 + 实测延迟（拍）
//==============================================================================
//  延迟定义：posedge E 采到 App_rd_en=1，posedge E+RD_LATENCY 采到 Sdr_rd_en=1。
//  这里在 negedge 上数拍，得到的 n 正好等于 RD_LATENCY（推导见 sdr_as_ram_sim
//  文件头"延迟约定"一节）。
//==============================================================================
task automatic sdr_read1(input [20:0] a, output [31:0] d, output integer lat);
    integer n;
    reg     found;
    begin
        @(posedge clk);
        App_rd_addr <= a;
        App_rd_en   <= 1'b1;
        @(posedge clk);
        App_rd_en   <= 1'b0;

        n     = 0;
        found = 1'b0;
        while (!found) begin
            @(negedge clk);
            n = n + 1;
            if (Sdr_rd_en === 1'b1)
                found = 1'b1;
            else if (n > 100) begin
                $display("  *** 读超时：addr=0x%05h ***", a);
                found = 1'b1;
                n     = -1;
            end
        end

        if (n < 0) begin
            d = 32'hDEAD_BEEF; lat = -1;
            errors = errors + 1;
        end else begin
            d = Sdr_rd_dout; lat = n;
        end
        @(posedge clk);
    end
endtask

//==============================================================================
//  任务：单次读 + 比对
//==============================================================================
task automatic read_check(input [20:0] a, input [31:0] exp, input [127:0] tag);
    reg [31:0] got;
    integer    lat;
    begin
        sdr_read1(a, got, lat);
        n_checked = n_checked + 1;
        if (got !== exp) begin
            errors = errors + 1;
            if (errors <= 20)
                $display("  [%0s] 地址 0x%05h 期望 0x%08h 实得 0x%08h  <<< 错",
                         tag, a, exp, got);
        end
    end
endtask

//==============================================================================
//  测试主体
//==============================================================================
integer    i;
integer    lat_meas;
reg [31:0] rdat;
reg [31:0] exp_buf [0:511];
reg [20:0] addr_buf[0:511];
integer    n_rd;
integer    t_start, t_end;
integer    lat_total;

initial begin
    $display("");
    $display("===================================================================");
    $display("  tb_sdram_probe —— SDRAM 替身模型验证 + 关键时序实测");
    $display("===================================================================");

    //--------------------------------------------------------------------------
    // 复位 + 等初始化
    //--------------------------------------------------------------------------
    rst = 1;
    repeat (10) @(posedge clk);
    rst = 0;

    t_start = cyc;
    while (!Sdr_init_done) begin
        @(posedge clk);
        if (cyc - t_start > 5000) begin
            $display("*** Sdr_init_done 超时 ***");
            $finish;
        end
    end
    $display("  [0] 初始化完成，用时 %0d 拍（期望约 %0d）", cyc - t_start, INIT_CYC);
    if ((cyc - t_start) < INIT_CYC - 2) begin
        $display("      <<< 初始化拍数偏少");
        errors = errors + 1;
    end

    //--------------------------------------------------------------------------
    // 1. 顺序写 256 字 → 顺序读回
    //--------------------------------------------------------------------------
    $display("");
    $display("  [1] 顺序写 256 字 → 顺序读回");
    for (i = 0; i < 256; i = i + 1)
        sdr_write(BUF0 + i[20:0], 32'hA500_0000 + i[31:0]);
    for (i = 0; i < 256; i = i + 1)
        read_check(BUF0 + i[20:0], 32'hA500_0000 + i[31:0], "顺序");

    //--------------------------------------------------------------------------
    // 2. 跨行散列地址（256 对齐 / 间隔 1024 / 间隔 4096）
    //--------------------------------------------------------------------------
    // 256 对齐 = 真实 SDRAM 一行的边界（bank 位 addr[9:8] 会在这里翻转）
    // 间隔 1024 / 4096 模拟"跳行"
    //--------------------------------------------------------------------------
    $display("  [2] 跨行散列地址写读");
    for (i = 0; i < 64; i = i + 1) begin
        addr_buf[i] = BUF1 + i * 256;
        sdr_write(addr_buf[i], 32'hB100_0000 + i[31:0]);
    end
    for (i = 0; i < 64; i = i + 1)
        read_check(addr_buf[i], 32'hB100_0000 + i[31:0], "跨行");

    for (i = 0; i < 32; i = i + 1) begin
        addr_buf[i] = TEX0 + i * 1024;
        sdr_write(addr_buf[i], 32'hC200_0000 + i[31:0]);
    end
    for (i = 0; i < 32; i = i + 1)
        read_check(addr_buf[i], 32'hC200_0000 + i[31:0], "跳1024");

    for (i = 0; i < 16; i = i + 1) begin
        addr_buf[i] = TEX1 + i * 4096;
        sdr_write(addr_buf[i], 32'hD300_0000 + i[31:0]);
    end
    for (i = 0; i < 16; i = i + 1)
        read_check(addr_buf[i], 32'hD300_0000 + i[31:0], "跳4096");

    //--------------------------------------------------------------------------
    // 3. 随机地址写读 512 次
    //--------------------------------------------------------------------------
    $display("  [3] 随机地址写读 512 次");
    for (i = 0; i < 512; i = i + 1) begin
        addr_buf[i] = {$random} % (MEM_WORDS - 1);
        // 避开前面测试用过的低地址区，防止互相覆盖
        if (addr_buf[i] < TEX0 + 65536)
            addr_buf[i] = addr_buf[i] + TEX1;
        sdr_write(addr_buf[i], 32'hE400_0000 + i[31:0]);
    end
    for (i = 0; i < 512; i = i + 1)
        read_check(addr_buf[i], 32'hE400_0000 + i[31:0], "随机");

    //--------------------------------------------------------------------------
    // 4. 四个真实区域基址互不混叠
    //--------------------------------------------------------------------------
    $display("  [4] 四个区域基址互不混叠");
    sdr_write(BUF0, 32'h1111_1111);
    sdr_write(BUF1, 32'h2222_2222);
    sdr_write(TEX0, 32'h3333_3333);
    sdr_write(TEX1, 32'h4444_4444);
    read_check(BUF0, 32'h1111_1111, "基址");
    read_check(BUF1, 32'h2222_2222, "基址");
    read_check(TEX0, 32'h3333_3333, "基址");
    read_check(TEX1, 32'h4444_4444, "基址");
    // 区域末尾（一帧最后一个字）
    sdr_write(BUF0 + 307199, 32'hAAAA_0000);
    sdr_write(BUF1 + 307199, 32'hBBBB_0000);
    read_check(BUF0 + 307199, 32'hAAAA_0000, "末尾");
    read_check(BUF1 + 307199, 32'hBBBB_0000, "末尾");

    //--------------------------------------------------------------------------
    // 5. 读延迟实测
    //--------------------------------------------------------------------------
    $display("");
    $display("  [5] 读延迟实测");
    lat_total = 0;
    for (i = 0; i < 8; i = i + 1) begin
        sdr_read1(TEX0 + 21'd100 + i[20:0], rdat, lat_meas);
        lat_total = lat_total + lat_meas;
    end
    $display("      8 次单读的延迟 = %0d 拍（模型参数 RD_LATENCY = %0d）",
             lat_total / 8, RD_LAT);
    if ((lat_total / 8) !== RD_LAT) begin
        $display("      <<< 延迟与参数不符");
        errors = errors + 1;
    end

    //--------------------------------------------------------------------------
    // 6 + 7. 背靠背连读 256 个命令：保序 + 吞吐
    //--------------------------------------------------------------------------
    $display("");
    $display("  [6][7] 背靠背连读 256 个命令：保序 + 吞吐");
    for (i = 0; i < 256; i = i + 1)
        sdr_write(TEX0 + 21'd1000 + i[20:0], 32'hF700_0000 + i[31:0]);

    @(posedge clk);
    t_start     = cyc;
    dbg_cmd_cnt = 0;
    dbg_dat_cnt = 0;
    dbg_armed   = 1'b0;
    dbg_en      = 1'b1;

    // ★ 边发边收 ★
    // 命令是流水发的，数据在 RD_LAT 拍后就陆续回来。必须在"发命令的同一个循环里"
    // 同步收数据 —— 否则等 256 个命令发完再去收，前面 246 个数据早就过去了。
    n_rd = 0;
    for (i = 0; i < 256 + RD_LAT + 8; i = i + 1) begin
        @(posedge clk);
        if (i < 256) begin
            App_rd_addr <= TEX0 + 21'd1000 + i[20:0];
            App_rd_en   <= 1'b1;
        end else begin
            App_rd_en   <= 1'b0;
        end
        @(negedge clk);                      // 半拍后采样：本拍所有 NBA 已落定
        if (Sdr_rd_en === 1'b1) begin
            if (Sdr_rd_dout !== (32'hF700_0000 + n_rd[31:0])) begin
                errors = errors + 1;
                if (errors <= 20)
                    $display("      [保序] 第 %0d 个数据 期望 0x%08h 实得 0x%08h  <<< 错",
                             n_rd, 32'hF700_0000 + n_rd[31:0], Sdr_rd_dout);
            end
            n_rd = n_rd + 1;
        end
    end
    dbg_en = 1'b0;
    t_end  = cyc;

    $display("      [dbg] DUT 收到 %0d 个读命令，吐出 %0d 个数据", dbg_cmd_cnt, dbg_dat_cnt);
    if (n_rd !== 256) begin
        $display("      *** 只收到 %0d 个数据（期望 256）***", n_rd);
        errors = errors + 1;
    end

    // 稳态吞吐：数据从第 RD_LAT 拍开始，之后每拍一个。
    // 256 个读命令的理想耗时 = 256 + RD_LAT（发命令 256 拍 + 延迟填充 RD_LAT 拍）。
    $display("      256 个读命令 + 收满 256 个数据，总耗时 = %0d 拍", t_end - t_start);
    $display("      理想值 = 256 + %0d = %0d 拍  →  稳态 %0d 命令/拍",
             RD_LAT, 256 + RD_LAT, 1);
    $display("      → 125MHz 下稳态 ≈ 125 M 次读/秒（读口 1 命令/拍，无间隙）");
    if ((t_end - t_start) > (256 + RD_LAT + 10)) begin
        $display("      <<< 耗时明显超出理想值，说明命令之间有气泡");
        errors = errors + 1;
    end

    //--------------------------------------------------------------------------
    // 8. 同拍既读又写 → rw_conflict
    //--------------------------------------------------------------------------
    $display("");
    $display("  [8] 同拍既读又写 → rw_conflict");
    if (rw_conflict !== 1'b0) begin
        $display("      <<< 前面的测试里不该出现冲突");
        errors = errors + 1;
    end
    @(posedge clk);
    App_wr_addr <= BUF0 + 21'd7;
    App_wr_din  <= 32'h1234_5678;
    App_wr_dm   <= 4'b0000;
    App_wr_en   <= 1'b1;
    App_rd_addr <= BUF0 + 21'd7;
    App_rd_en   <= 1'b1;
    @(posedge clk);
    App_wr_en <= 1'b0;
    App_rd_en <= 1'b0;
    repeat (4) @(posedge clk);
    if (rw_conflict !== 1'b1) begin
        $display("      <<< 同拍读写没有报冲突");
        errors = errors + 1;
    end else begin
        $display("      冲突已报出 ✓");
    end

    //--------------------------------------------------------------------------
    // 汇总
    //--------------------------------------------------------------------------
    $display("");
    $display("===================================================================");
    $display("  比对次数 = %0d", n_checked);
    $display("  错误数   = %0d", errors);
    if (errors == 0)
        $display("  RESULT: PASS");
    else
        $display("  RESULT: FAIL");
    $display("===================================================================");
    $display("");
    $finish;
end

//==============================================================================
//  看门狗
//==============================================================================
initial begin
    #20_000_000;                             // 20ms 仿真时间上限
    $display("*** 仿真超时（看门狗）***");
    $finish;
end

endmodule
