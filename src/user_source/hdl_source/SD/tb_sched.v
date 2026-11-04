//==============================================================================
//  tb_sched —— sdr_port_sched 的功能验证（带"会惩罚"的 SDRAM 模型）
//==============================================================================
//  验证四件事：
//    1. 显示读：连续 4096 个字，每个都得对（这是虚线/撕裂的直接指标）
//    2. 显示写：连续 1024 个字，落盘内容正确
//    3. 纹理读：两层各 200 个请求；build D 起一次 ack 投整组 4 字，
//       所以回程按"组首 + 组内序号"逐字校验（共 800 字/层）
//    4. 纪律：App_rd_en 与 App_wr_en 任何一拍都不同时为 1（单口器件底线）
//
//  模型 PHASE_MODE=1 = 控制器内部 4 拍相位随【时钟】自由运行（最严格的一种
//  理解）。只有"所有空隙都是 4 拍的整数倍"才能在这模式下全对 ——
//  这正是本设计要证明的不变量。
//
//  跑法：
//    iverilog -g2005 -s tb_sched -Ptb_sched.RD_LAT=20 -o a.vvp \
//        SD/sdr_port_sched.v SD/model/sdr_as_ram_sim.v SD/tb_sched.v
//    vvp a.vvp
//==============================================================================
`timescale 1ns/1ps

module tb_sched;
    parameter integer RD_LAT     = 20;
    parameter integer PHASE_MODE = 1;
    parameter integer PHASE_WR   = 1;

    //--------------------------------------------------------------------------
    //  时钟/复位
    //--------------------------------------------------------------------------
    reg clk = 1'b0;
    reg rst = 1'b1;
    always #4 clk = ~clk;              // 125MHz

    //--------------------------------------------------------------------------
    //  DUT 端口
    //--------------------------------------------------------------------------
    wire        disp_rd_ready, disp_rd_vld;
    wire [31:0] disp_rd_data;
    wire [20:0] disp_rd_addr;

    wire        disp_wr_ready;
    wire [20:0] disp_wr_addr;
    wire [31:0] disp_wr_din;
    wire [3:0]  disp_wr_dm;

    wire [1:0]  tex_req_vld, tex_req_ack, tex_pix_wr;
    wire [41:0] tex_req_addr;
    wire [7:0]  tex_req_mask;
    wire [63:0] tex_pix_data;

    wire        ld_wr_vld, ld_wr_ack;
    wire [4:0]  ld_wr_used;
    wire [20:0] ld_wr_addr;
    wire [31:0] ld_wr_data;

    wire        App_rd_en, App_wr_en;
    wire [20:0] App_rd_addr, App_wr_addr;
    wire [31:0] App_wr_din;
    wire [3:0]  App_wr_dm;
    wire        Sdr_rd_en;
    wire [31:0] Sdr_rd_dout;
    wire [3:0]  dbg_slot;

    //--------------------------------------------------------------------------
    //  显示读源（模拟 frame_fifo_read：一直想读，地址连续递增）
    //--------------------------------------------------------------------------
    localparam [20:0] DRD_BASE = 21'h0_1000;
    localparam integer DRD_NUM  = 4096;

    reg         go;                        // 预置内存完成前不发起任何访问
    reg  [20:0] drd_cnt;
    wire        drd_want = go && (drd_cnt < DRD_NUM[20:0]);
    assign      disp_rd_addr = DRD_BASE + drd_cnt;
    wire        disp_rd_en   = drd_want & disp_rd_ready;

    //--------------------------------------------------------------------------
    //  显示写源（模拟 frame_fifo_write：一直想写，数据 = 地址的函数）
    //--------------------------------------------------------------------------
    localparam [20:0] DWR_BASE = 21'h3_0000;
    localparam integer DWR_NUM  = 1024;

    reg  [20:0] dwr_cnt;
    wire        dwr_want = go && (dwr_cnt < DWR_NUM[20:0]);
    assign      disp_wr_addr = DWR_BASE + dwr_cnt;
    assign      disp_wr_din  = pat(disp_wr_addr);
    assign      disp_wr_dm   = 4'h0;
    wire        disp_wr_en   = dwr_want & disp_wr_ready;

    //--------------------------------------------------------------------------
    //  数据图样
    //--------------------------------------------------------------------------
    function [31:0] pat;
        input [20:0] a;
        begin
            pat = {11'd0, a} ^ 32'h5A5A_0000 ^ (32'h0000_0101 * a[7:0]);
        end
    endfunction

    //--------------------------------------------------------------------------
    //  纹理请求源：两层各发 NTX 个请求，地址故意用"带偏移的斜线"
    //    addr = base + k*3  ⇒ addr[1:0] 依次 0,3,2,1,0,... 覆盖组内 4 个位置
    //--------------------------------------------------------------------------
    localparam integer NTX = 200;
    localparam [20:0]  TX0_BASE = 21'h0_2000;
    localparam [20:0]  TX1_BASE = 21'h0_4000;

    //  ★ vld 必须【组合】来自请求号：若用寄存器，ack 之后会多出一个
    //    "地址已前进但 vld 仍为 1"的假请求周期（真实 FIFO 不会这样）。
    reg  [20:0] tx_k0, tx_k1;              // 已发出的请求号（下面声明在前）
    wire [20:0] tx_addr_c0 = TX0_BASE + tx_k0 * 21'd3;
    wire [20:0] tx_addr_c1 = TX1_BASE + tx_k1 * 21'd7;
    // ★ build F：每个请求要带 4 位组掩码。这里故意用两种稀疏掩码轮换，
    //   既覆盖"一组多字"，也覆盖"一组只有一个字"。
    wire [3:0]  tx_mask_c0 = tx_k0[0] ? 4'b0101 : 4'b1110;
    wire [3:0]  tx_mask_c1 = tx_k1[0] ? 4'b1000 : 4'b0011;
    wire [20:0] tx_gb0 = {tx_addr_c0[20:2], 2'b00};
    wire [20:0] tx_gb1 = {tx_addr_c1[20:2], 2'b00};
    integer     exp0, exp1;            // 期望回程字数（= Σ 掩码位）
    wire        tx_vld_c0  = go && (tx_k0 < NTX[20:0]);
    wire        tx_vld_c1  = go && (tx_k1 < NTX[20:0]);
    assign tex_req_vld  = {tx_vld_c1, tx_vld_c0};
    assign tex_req_addr = {tx_addr_c1, tx_addr_c0};
    assign tex_req_mask = {tx_mask_c1, tx_mask_c0};

    reg  [20:0] tx_ack0, tx_ack1;          // 已被应答的请求号
    //  ★★ build E 契约：1 个请求 = 1 个像素 ★★
    //    请求地址是【完整地址】（低 2 位 = 要组里的第几个字），
    //    调度器把该 4 字组整组读回，只把"组内偏移 == 请求地址低 2 位"的那个字投出来。
    //    ⇒ 期望值因此退化成一个字：**mem[请求地址]**。
    //    ack 之后要过 RD_LAT 拍数据才回来，期间同层可能已经连开好几个请求，
    //    所以已应答地址必须排成先进先出队列。
    localparam integer TQ = 16;            // 每层在飞请求数上限
    reg  [20:0] q0 [0:TQ-1];
    reg  [20:0] q1 [0:TQ-1];
    reg  [3:0]  qw0, qr0, qw1, qr1;
    wire [4:0]  qn0 = {1'b0, qw0} - {1'b0, qr0};   // 在飞数（5 位，防 TQ=16 溢出）
    wire [4:0]  qn1 = {1'b0, qw1} - {1'b0, qr1};
    integer     err_q;                     // 队列溢出（说明模型假设不成立）


    //--------------------------------------------------------------------------
    //  装载写源：连续写 LW_NUM 个字
    //--------------------------------------------------------------------------
    localparam [20:0] LD_BASE = 21'h5_0000;
    localparam integer LD_NUM  = 1024;

    reg  [20:0] ld_cnt;
    wire        ld_want = go && (ld_cnt < LD_NUM[20:0]);
    assign      ld_wr_vld  = ld_want;
    assign      ld_wr_used = 5'd8;
    assign      ld_wr_addr = LD_BASE + ld_cnt;
    assign      ld_wr_data = pat(ld_wr_addr);

    //--------------------------------------------------------------------------
    //  DUT
    //--------------------------------------------------------------------------
    sdr_port_sched #(.TAG_AW(6)) u_dut (
        .clk          (clk),
        .rst          (rst),
        .disp_rd_en   (disp_rd_en),
        .disp_rd_addr (disp_rd_addr),
        .disp_rd_ready(disp_rd_ready),
        .disp_rd_vld  (disp_rd_vld),
        .disp_rd_data (disp_rd_data),
        .disp_wr_en   (disp_wr_en),
        .disp_wr_addr (disp_wr_addr),
        .disp_wr_din  (disp_wr_din),
        .disp_wr_dm   (disp_wr_dm),
        .disp_wr_ready(disp_wr_ready),
        .tex_req_vld  (tex_req_vld),
        .tex_req_addr (tex_req_addr),
        .tex_req_mask (tex_req_mask),
        .tex_req_ack  (tex_req_ack),
        .tex_pix_wr   (tex_pix_wr),
        .tex_pix_data (tex_pix_data),
        .ld_wr_vld    (ld_wr_vld),
        .ld_wr_used   (ld_wr_used),
        .ld_wr_addr   (ld_wr_addr),
        .ld_wr_data   (ld_wr_data),
        .ld_wr_ack    (ld_wr_ack),
        .App_rd_en    (App_rd_en),
        .App_rd_addr  (App_rd_addr),
        .App_wr_en    (App_wr_en),
        .App_wr_addr  (App_wr_addr),
        .App_wr_din   (App_wr_din),
        .App_wr_dm    (App_wr_dm),
        .Sdr_rd_en    (Sdr_rd_en),
        .Sdr_rd_dout  (Sdr_rd_dout),
        .Sdr_busy     (1'b0),          // 本测试不带换向模型 ⇒ BUSY_HOLD 用不上
        .Sdr_init_ref_vld (1'b0),      // 同上（没有刷新模型）
        .rd_lag       (1'b0),
        // ★ 2026-10-21：这四个输入原来没接（悬空 = z）★
        //   tx_ph_off 悬空 ⇒ tx_ph_eff = z ⇒ `== 0` 永远为假 ⇒ **纹理一条命令都没发**
        //   （所以老 TB 打印"纹理0 0/0"却仍然 PASS —— 它根本没测纹理）。
        .tx_ph_off    (2'd0),
        .exp_mode     (4'd0),
        .dbg_frame    (1'b0),
        .dbg_sel      (4'd0),
        .dbg_lddrop   (20'd0),
        .dbg_txhit_p  (),          // 读数据同拍（仿真模型行为）
        .ph_off       (2'd0),          // build F：组相位偏移（仿真用 0）
        .dbg_slot     (dbg_slot)
    );

    //--------------------------------------------------------------------------
    //  SDRAM 模型（4 字分组惩罚）
    //--------------------------------------------------------------------------
    wire        Sdr_init_done, Sdr_init_ref_vld, Sdr_busy, rw_conflict;
    wire [31:0] n_rd_drop, n_rd_seen, n_grp_bad, n_grp_seen;

    sdr_as_ram_sim #(
        .RD_LATENCY  (RD_LAT),
        .INIT_CYCLES (50),
        .MEM_WORDS   (2097152),
        .PHASE_MODE  (PHASE_MODE),
        .PHASE_WR_EN (PHASE_WR)
    ) u_mem (
        .Sdr_clk          (clk),
        .Sdr_clk_sft      (clk),
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
        .rw_conflict      (rw_conflict),
        .n_rd_drop        (n_rd_drop),
        .n_rd_seen        (n_rd_seen),
        .n_grp_bad        (n_grp_bad),
        .n_grp_seen       (n_grp_seen)
    );

    //--------------------------------------------------------------------------
    //  出错计数
    //--------------------------------------------------------------------------
    integer err_rd, err_tx, err_con, n_rd, n_tx0, n_tx1, n_ld;

    // 显示读校验
    always @(posedge clk) begin
        if (disp_rd_en)
            drd_cnt <= drd_cnt + 21'd1;
        if (disp_rd_vld) begin
            n_rd <= n_rd + 1;
            if (disp_rd_data !== u_mem.mem[DRD_BASE + n_rd]) begin
                err_rd <= err_rd + 1;
                if (err_rd < 6)
                    $display("[ERR-RD] #%0d got %h exp %h", n_rd, disp_rd_data,
                             u_mem.mem[DRD_BASE + n_rd]);
            end
        end
    end

    // 显示写计数
    always @(posedge clk) begin
        if (disp_wr_en)
            dwr_cnt <= dwr_cnt + 21'd1;
    end

    // 纹理：请求/应答/回程
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            tx_k0 <= 0; tx_k1 <= 0;
            tx_ack0 <= 0; tx_ack1 <= 0;
        end else begin
            if (tex_req_ack[0]) begin tx_ack0 <= tx_k0; tx_k0 <= tx_k0 + 21'd1; end
            if (tex_req_ack[1]) begin tx_ack1 <= tx_k1; tx_k1 <= tx_k1 + 21'd1; end
        end
    end

    // 回程校验：ack 时把【完整请求地址】压进该层队列；每个 tex_pix_wr 弹一个，
    // 期望值就是 mem[队首]（因为调度器投的就是"组内偏移 == 请求低 2 位"的那个字）。
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            qw0 <= 4'd0; qr0 <= 4'd0;
            qw1 <= 4'd0; qr1 <= 4'd0;
            exp0 <= 0; exp1 <= 0;
        end else begin
            // ★ build F：一个 ack 换 1~4 个字 ⇒ 按掩码把具体地址逐个压进队列 ★
            //  用显式 case（不能用连续 4 个 if：非阻塞赋值会打到同一个下标）
            if (tex_req_ack[0]) begin
                case (tx_mask_c0)
                4'b1111: begin
                    q0[qw0[3:0]] <= tx_gb0 + 21'd0; q0[qw0[3:0]+4'd1] <= tx_gb0 + 21'd1;
                    q0[qw0[3:0]+4'd2] <= tx_gb0 + 21'd2; q0[qw0[3:0]+4'd3] <= tx_gb0 + 21'd3;
                    qw0 <= qw0 + 4'd4; exp0 <= exp0 + 4;
                end
                4'b1110: begin
                    q0[qw0[3:0]] <= tx_gb0 + 21'd1; q0[qw0[3:0]+4'd1] <= tx_gb0 + 21'd2;
                    q0[qw0[3:0]+4'd2] <= tx_gb0 + 21'd3;
                    qw0 <= qw0 + 4'd3; exp0 <= exp0 + 3;
                end
                4'b0101: begin
                    q0[qw0[3:0]] <= tx_gb0 + 21'd0; q0[qw0[3:0]+4'd1] <= tx_gb0 + 21'd2;
                    qw0 <= qw0 + 4'd2; exp0 <= exp0 + 2;
                end
                default: begin
                    q0[qw0[3:0]] <= tx_gb0; qw0 <= qw0 + 4'd1; exp0 <= exp0 + 1;
                end
                endcase
            end
            if (tex_pix_wr[0]) qr0 <= qr0 + 4'd1;

            if (tex_req_ack[1]) begin
                case (tx_mask_c1)
                4'b1111: begin
                    q1[qw1[3:0]] <= tx_gb1 + 21'd0; q1[qw1[3:0]+4'd1] <= tx_gb1 + 21'd1;
                    q1[qw1[3:0]+4'd2] <= tx_gb1 + 21'd2; q1[qw1[3:0]+4'd3] <= tx_gb1 + 21'd3;
                    qw1 <= qw1 + 4'd4; exp1 <= exp1 + 4;
                end
                4'b0011: begin
                    q1[qw1[3:0]] <= tx_gb1 + 21'd0; q1[qw1[3:0]+4'd1] <= tx_gb1 + 21'd1;
                    qw1 <= qw1 + 4'd2; exp1 <= exp1 + 2;
                end
                4'b1000: begin
                    q1[qw1[3:0]] <= tx_gb1 + 21'd3;
                    qw1 <= qw1 + 4'd1; exp1 <= exp1 + 1;
                end
                default: begin
                    q1[qw1[3:0]] <= tx_gb1; qw1 <= qw1 + 4'd1; exp1 <= exp1 + 1;
                end
                endcase
            end
            if (tex_pix_wr[1]) qr1 <= qr1 + 4'd1;
        end
    end

    // 像素比对
    always @(posedge clk) begin
        if (tex_pix_wr[0]) begin
            n_tx0 <= n_tx0 + 1;
            if (tex_pix_data[31:0] !== u_mem.mem[q0[qr0[3:0]]]) begin
                err_tx <= err_tx + 1;
                if (err_tx < 6)
                    $display("[ERR-TX0] #%0d got %h exp %h (req_addr=%h)", n_tx0,
                             tex_pix_data[31:0], u_mem.mem[q0[qr0[3:0]]], q0[qr0[3:0]]);
            end
        end
        if (tex_pix_wr[1]) begin
            n_tx1 <= n_tx1 + 1;
            if (tex_pix_data[63:32] !== u_mem.mem[q1[qr1[3:0]]]) begin
                err_tx <= err_tx + 1;
                if (err_tx < 6)
                    $display("[ERR-TX1] #%0d got %h exp %h (req_addr=%h)", n_tx1,
                             tex_pix_data[63:32], u_mem.mem[q1[qr1[3:0]]], q1[qr1[3:0]]);
            end
        end
    end

    // 装载写计数
    always @(posedge clk) begin
        if (ld_wr_ack)
            ld_cnt <= ld_cnt + 21'd1;
    end

    // 同拍读写（单口器件底线，必须恒 0）
    always @(posedge clk) begin
        if (App_rd_en && App_wr_en) begin
            err_con <= err_con + 1;
            if (err_con < 4) $display("[ERR-CONFLICT] 同拍读写 slot=%0d", dbg_slot);
        end
    end

    //--------------------------------------------------------------------------
    //  激励
    //--------------------------------------------------------------------------
    integer i;
    initial begin
        go = 1'b0; drd_cnt = 0; dwr_cnt = 0; ld_cnt = 0;
        err_rd = 0; err_tx = 0; err_con = 0;
        n_rd = 0; n_tx0 = 0; n_tx1 = 0; n_ld = 0; err_q = 0;

        rst = 1'b1;
        repeat (20) @(posedge clk);
        rst = 1'b0;
        repeat (100) @(posedge clk);      // 等模型初始化稳定再预置

        // 预置内存图样（覆盖所有会被读到的区域）
        for (i = 0; i < 8192; i = i + 1)
            u_mem.mem[DRD_BASE + i] = pat(DRD_BASE + i);
        for (i = 0; i < 2048; i = i + 1) begin
            u_mem.mem[TX0_BASE + i] = pat(TX0_BASE + i);
            u_mem.mem[TX1_BASE + i] = pat(TX1_BASE + i);
        end
        go = 1'b1;                        // 预置完成，开始发访问

        // 等所有活动结束。
        //  ★ build E：1 个请求 = 1 个像素，所以回程像素数 = 请求数
        wait (n_rd >= DRD_NUM);
        wait (n_tx0 >= exp0 && n_tx1 >= exp1);
        wait (dwr_cnt >= DWR_NUM);
        wait (ld_cnt >= LD_NUM);
        repeat (2000) @(posedge clk);   // 等最后 4 字的装载组真正落盘

        // 显示写落盘校验
        for (i = 0; i < DWR_NUM; i = i + 1)
            if (u_mem.mem[DWR_BASE + i] !== pat(DWR_BASE + i)) begin
                err_tx = err_tx + 1;
                if (err_tx < 10) $display("[ERR-WR] addr %h got %h", DWR_BASE+i, u_mem.mem[DWR_BASE+i]);
            end
        // 装载写落盘校验
        for (i = 0; i < LD_NUM; i = i + 1)
            if (u_mem.mem[LD_BASE + i] !== pat(LD_BASE + i)) begin
                err_tx = err_tx + 1;
                if (err_tx < 10) $display("[ERR-LD] addr %h got %h", LD_BASE+i, u_mem.mem[LD_BASE+i]);
            end

        $display("-------------------------------------------------------------");
        $display(" 显示读 %0d/%0d  错 %0d", n_rd, DRD_NUM, err_rd);
        $display(" 显示写 %0d/%0d", dwr_cnt, DWR_NUM);
        $display(" 纹理0  %0d/%0d   纹理1 %0d/%0d  (像素 = Σ掩码位，一个 ack 换 1~4 字)",
                 n_tx0, exp0, n_tx1, exp1);
        $display(" 装载写 %0d/%0d", ld_cnt, LD_NUM);
        $display(" 数据错 %0d   读写同拍 %0d   队列溢出 %0d", err_tx, err_con, err_q);
        $display(" 模型: 分组错 %0d / %0d 拍   读命令 %0d 丢 %0d",
                 n_grp_bad, n_grp_seen, n_rd_seen, n_rd_drop);
        if (err_rd == 0 && err_tx == 0 && err_con == 0 && err_q == 0 &&
            n_rd == DRD_NUM && n_tx0 == exp0 && n_tx1 == exp1 &&
            ld_cnt >= LD_NUM && dwr_cnt == DWR_NUM)
            $display(" === PASS ===");
        else
            $display(" === FAIL ===");
        $display("-------------------------------------------------------------");
        $finish;
    end

    initial begin
        #4_000_000;
        $display(" === TIMEOUT === rd=%0d tx0=%0d tx1=%0d dwr=%0d ld=%0d",
                 n_rd, n_tx0, n_tx1, dwr_cnt, ld_cnt);
        $finish;
    end

endmodule
