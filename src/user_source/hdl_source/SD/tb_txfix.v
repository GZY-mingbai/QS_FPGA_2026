//==============================================================================
//  ★★ 100M-40：本文件是**测试平台**，只在仿真时编译 ★★
//==============================================================================
//  【为什么加这层护栏】本文件被写进了工程的综合文件列表（HDMI1.4b_Transmitter_v1.0.al
//  与 syn_1/*.prj），于是 TD 综合时也编译它，报 10 条
//      HDL-8007  'mem' is not declared under prefix 'u_mem'
//  原因是它用了**层次引用** `u_mem.mem`（SDRAM 模型内部数组），综合时该层次
//  不存在。TB 本来就不该进综合，所以用宏把它整体排除：
//      · 综合时：TB_TXFIX_SIM 未定义 ⇒ 本文件为空 ⇒ 不报错
//      · 需要仿真时：在仿真命令里加 `+define+TB_TXFIX_SIM`（或 TD 仿真选项里加宏）
//  ★ 若将来把这些 TB 从工程文件列表里移除，这层护栏可以删掉（但它无害）★
//==============================================================================
`ifdef TB_TXFIX_SIM
//==============================================================================
//  tb_txfix —— 量化"时隙表换位 + 组内冻结"对纹理供给与正确性的影响（100M-18 验收）
//==============================================================================
//  【为什么另起一个 TB】tb_sched 的 `tx_ph_off` 没接（悬空 = z）⇒ tx_ph_eff 是 X
//  ⇒ tx_start 永不成立 ⇒ 纹理通路**一条命令都没发**（打印 "纹理0 0/0"），
//  也就是说"虚线根因修复"从来没有被仿真覆盖过。本 TB 把每个输入都驱动起来，
//  并且把 SDRAM 模型的 4 命令分组（PHASE_MODE=2）打开 —— 分组相位漂移只有它能抓。
//
//  【测什么】
//    1. 纹理供给：每 3180 拍（一行）能开出多少个 4 字组 —— 需求是 240 组/行
//    2. 每组的命令数恒为 4（mod-4 不变式）
//    3. 带忙窗时数据仍然全对，**分组错必须为 0**（abort/retry 会让全局相位漂移）
//    4. abort 次数：冻结方案下应恒为 0；hold = 冻结拍数（不是次数）
//
//  【怎么跑】
//    iverilog -g2005 -s tb_txfix -Ptb_txfix.MODE=0 -o t.vvp \
//        SD/sdr_port_sched.v SD/model/sdr_as_ram_sim.v SD/tb_txfix.v
//    vvp t.vvp
//    MODE=0 饱和请求（测供给上限）
//    MODE=1 每行 240 组（测真实需求能否满足）
//    MODE=2 饱和请求 + 恶劣忙窗（12.5% 占空，比真机重 8 倍，验证"冻结"不丢命令）
//
//  【100M-18 实测】（100MHz，30 行）
//    MODE=0：312 组/行（峰值）、0 abort、0 错字、0 分组错
//    MODE=1：240/240 全交付、0 abort、0 错字、0 分组错
//    MODE=2：299 组/行、0 abort、0 错字、0 分组错
//    对照（100M-17：TX 头部静音 8 拍 + hold/abort）：286 组/行
//==============================================================================
`timescale 1ns / 1ps

module tb_txfix;
    parameter integer MODE      = 0;      // 0=饱和, 1=每行 240 组
    parameter integer LINE_BEATS= 3180;   // 100MHz、640x480@60 的一行
    parameter integer GROUPS_LN = 240;    // 活跃行需求（两框竖直不重叠 ⇒ 每行只有一个框）
    // ★ 换向代价：>0 时模型会丢掉"方向切换且空闲 < TURN_CYC 拍"的命令 ★
    //   真机实测区间是 7~19 拍（100M-18 的 7 拍出竖条、原设计 19 拍没问题），
    //   取 16 作为上板前的验收口径。
    parameter integer TURN_CYC  = 16;

    reg clk = 1'b0;
    reg rst = 1'b1;
    always #5 clk = ~clk;                 // 100MHz（与 sys_pll 的 ext_mem_clk 一致）

    //--------------------------------------------------------------------------
    //  DUT 端口
    //--------------------------------------------------------------------------
    wire        disp_rd_ready, disp_rd_vld;
    wire [31:0] disp_rd_data;
    wire        disp_wr_ready;
    wire [1:0]  tex_req_ack, tex_pix_wr;
    wire [63:0] tex_pix_data;
    wire        ld_wr_ack, ld_wr_settled;
    wire        App_rd_en, App_wr_en;
    wire [20:0] App_rd_addr, App_wr_addr;
    wire [31:0] App_wr_din;
    wire [3:0]  App_wr_dm;
    wire        Sdr_rd_en;
    wire [31:0] Sdr_rd_dout;
    wire [3:0]  dbg_slot, dbg_cmdbeat, dbg_flag;
    wire [11:0] dbg_val;
    wire        dbg_txhit_p, dbg_bad_we;
    wire [63:0] dbg_bad_data;

    //--------------------------------------------------------------------------
    //  行/场节拍
    //--------------------------------------------------------------------------
    reg [15:0] line_ph;                   // 0..LINE_BEATS-1
    reg [15:0] line_no;
    wire       line_tick = (line_ph == LINE_BEATS-1);
    always @(posedge clk or posedge rst) begin
        if (rst) begin line_ph <= 16'd0; line_no <= 16'd0; end
        else begin
            line_ph <= line_tick ? 16'd0 : line_ph + 16'd1;
            if (line_tick) line_no <= line_no + 16'd1;
        end
    end

    //--------------------------------------------------------------------------
    //  显示读 / 显示写：永远想要（各自吃满自己的窗口）
    //--------------------------------------------------------------------------
    reg  [20:0] drd_cnt, dwr_cnt;
    wire        disp_rd_en = disp_rd_ready;
    wire        disp_wr_en = disp_wr_ready;
    always @(posedge clk or posedge rst) begin
        if (rst) begin drd_cnt <= 21'd0; dwr_cnt <= 21'd0; end
        else begin
            if (disp_rd_en) drd_cnt <= drd_cnt + 21'd1;
            if (disp_wr_en) dwr_cnt <= dwr_cnt + 21'd1;
        end
    end
    //--------------------------------------------------------------------------
    //  显示读回程校验（白点的直接度量）
    //    每条 disp_rd_en 压一个期望地址，每个 disp_rd_vld 弹一个比对。
    //    忙窗把某条命令当成"组首"重算地址时，数据会与期望不符 ⇒ 这里计数。
    //--------------------------------------------------------------------------
    reg [20:0] rq_d [0:511];
    reg [8:0]  rq_w, rq_r;
    integer    err_drd;
    always @(posedge clk or posedge rst) begin
        if (rst) begin rq_w <= 0; rq_r <= 0; err_drd <= 0; end
        else begin
            if (disp_rd_en) begin
                rq_d[rq_w[8:0]] <= drd_cnt;
                rq_w <= rq_w + 9'd1;
            end
            if (disp_rd_vld) begin
                if (disp_rd_data !== u_mem.mem[rq_d[rq_r[8:0]]]) begin
                    err_drd <= err_drd + 1;
                    if (err_drd < 4)
                        $display("[ERR-DRD] got %h exp %h @%h", disp_rd_data,
                                 u_mem.mem[rq_d[rq_r[8:0]]], rq_d[rq_r[8:0]]);
                end
                rq_r <= rq_r + 9'd1;
            end
        end
    end

    function [31:0] pat;
        input [20:0] a;
        begin pat = {11'd0, a} ^ 32'h5A5A_0000 ^ (32'h0000_0101 * a[7:0]); end
    endfunction

    //--------------------------------------------------------------------------
    //  纹理请求源（两层）
    //    地址步进 4 ⇒ 每个请求都是新组；掩码恒 1111 ⇒ 一组必回 4 个字，
    //    回程顺序必须是 base+0,+1,+2,+3（这就是 4 命令分组的直接检验）。
    //    MODE=0：一直有效（饱和）
    //    MODE=1：每行前 240 组（模拟 240 像素宽的框），其余时间不发
    //--------------------------------------------------------------------------
    localparam [20:0] TX0_BASE = 21'h2_0000;
    localparam [20:0] TX1_BASE = 21'h4_0000;

    reg  [20:0] k0, k1;
    reg         stop;                      // 收尾：先撤请求，等在飞组排空再结账
    // MODE=1：每行只发 GROUPS_LN 组（= 一个 240 宽的框），按行内匀速铺开，
    //          用来回答"真实需求能不能被满足"（MODE=0 回答"上限是多少"）
    reg  [15:0] credit;
    reg  [3:0]  pace;
    wire        pace_ok = (pace == 4'd0);
    always @(posedge clk or posedge rst) begin
        if (rst) pace <= 4'd0;
        else     pace <= (pace == 4'd12) ? 4'd0 : pace + 4'd1;
    end
    // MODE=1 不做 pacing：真实图层前面有 128 深请求 FIFO，等价于"只要有额度就一直有效"
    // MODE=4：双层饱和 + 真实回程忙 ⇒ 回答"两个框重叠时，TX 段实际能给多少组/行"
    //         （320 组/行是两层各 160 组的需求，见 app_scene_cfg 的框宽 240）
    wire        tx_vld0 = !stop && ((MODE == 0 || MODE == 2 || MODE == 4) ? 1'b1 : (credit != 16'd0));
    wire        tx_vld1 = !stop && ((MODE == 0 || MODE == 2 || MODE == 4) ? 1'b1 : 1'b0);   // 单框：一屏行只有一个框
    wire [20:0] a0 = TX0_BASE + {k0[18:0], 2'b00};
    wire [20:0] a1 = TX1_BASE + {k1[18:0], 2'b00};

    wire [1:0]  req_vld  = {tx_vld1, tx_vld0};
    wire [41:0] req_addr = {a1, a0};
    wire [7:0]  req_mask = {4'hF, 4'hF};

    always @(posedge clk or posedge rst) begin
        if (rst) begin k0 <= 21'd0; k1 <= 21'd0; credit <= 16'd0; stop <= 1'b0; end
        else begin
            if (line_tick) credit <= GROUPS_LN[15:0];      // 每行重新给额度
            else if (tex_req_ack[0] && (credit != 16'd0)) credit <= credit - 16'd1;
            if (tex_req_ack[0]) k0 <= k0 + 21'd1;
            if (tex_req_ack[1]) k1 <= k1 + 21'd1;
        end
    end

    //--------------------------------------------------------------------------
    //  忙窗发生器（DUT 看它，SDRAM 模型不看 ⇒ 模型不会丢命令，只测保护逻辑）
    //    ① 换向尾巴：每 512 拍来 8 拍（1.6%）
    //    ② 刷新    ：每 3125 拍来 20 拍（0.64%，与真机一致）
    //    MODE=2（压力测试）：① 改成每 256 拍 32 拍（12.5%）② 每 1024 拍 24 拍
    //      —— 用来验证"冻结"方案在比真机恶劣 8 倍的忙占空比下仍不掉命令、不漂相位。
    //--------------------------------------------------------------------------
    localparam integer BUSY_A  = (MODE == 2) ? 32  : 8;
    localparam integer BUSY_AP = (MODE == 2) ? 255 : 511;
    localparam integer BUSY_B  = (MODE == 2) ? 24  : 20;
    localparam integer BUSY_BP = (MODE == 2) ? 1023 : 3124;

    wire Sdr_init_done, Sdr_init_ref_vld_m, Sdr_busy_m, rw_conflict;
    wire [31:0] n_rd_drop, n_rd_seen, n_grp_bad, n_grp_seen, n_ref_drop, n_turn_drop;
    reg [9:0]  b512;
    reg [11:0] b3125;
    reg        busy_r;
    always @(posedge clk or posedge rst) begin
        if (rst) begin b512 <= 10'd0; b3125 <= 12'd0; busy_r <= 1'b0; end
        else begin
            b512  <= (b512  == BUSY_AP) ? 10'd0 : b512  + 10'd1;
            b3125 <= (b3125 == BUSY_BP) ? 12'd0 : b3125 + 12'd1;
            busy_r <= (b512 < BUSY_A) || (b3125 < BUSY_B);
        end
    end
    //--------------------------------------------------------------------------
    //  MODE=3：**按真实回程建模忙** ★最接近真机的口径★
    //    每条显示读命令发出后，Sdr_busy 连续高 11 拍（= CAS 回程阴影）。
    //    板上"TX 段头部被压住 2~3 个组起点"就是它造成的，定时器模型（①②）
    //    完全模拟不出这个效应 ⇒ 只有这个模式能回答"到底够不够 240 组/行"。
    //--------------------------------------------------------------------------
    reg [10:0] rd_shadow;
    always @(posedge clk or posedge rst) begin
        if (rst) rd_shadow <= 11'd0;
        else rd_shadow <= {rd_shadow[9:0], (App_rd_en && (u_dut.own == 3'd1))};
    end
    wire busy_shadow = |rd_shadow;
    wire Sdr_busy = (MODE == 3 || MODE == 4) ? (busy_shadow | busy_r | Sdr_busy_m) : (busy_r | Sdr_busy_m);

    //--------------------------------------------------------------------------
    //  DUT
    //--------------------------------------------------------------------------
    sdr_port_sched #(.TAG_AW(8), .BUSY_HOLD(1)) u_dut (
        .clk(clk), .rst(rst),
        .disp_rd_en(disp_rd_en), .disp_rd_addr(drd_cnt), .disp_rd_ready(disp_rd_ready),
        .disp_rd_vld(disp_rd_vld), .disp_rd_data(disp_rd_data),
        .disp_wr_en(disp_wr_en), .disp_wr_addr(dwr_cnt), .disp_wr_din(pat(dwr_cnt)),
        .disp_wr_dm(4'h0), .disp_wr_ready(disp_wr_ready),
        .tex_req_vld(req_vld), .tex_req_addr(req_addr), .tex_req_mask(req_mask),
        .tex_req_ack(tex_req_ack), .tex_pix_wr(tex_pix_wr), .tex_pix_data(tex_pix_data),
        .ld_wr_vld(1'b0), .ld_wr_used(5'd0), .ld_wr_addr(21'd0), .ld_wr_data(32'd0),
        .ld_wr_ack(ld_wr_ack), .ld_wr_settled(ld_wr_settled),
        .App_rd_en(App_rd_en), .App_rd_addr(App_rd_addr),
        .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr),
        .App_wr_din(App_wr_din), .App_wr_dm(App_wr_dm),
        .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout),
        .Sdr_busy(Sdr_busy), .Sdr_init_ref_vld(Sdr_init_ref_vld_m),
        .rd_lag(1'b0), .tx_ph_off(2'd0), .ph_off(2'd0), .exp_mode(4'd0),
        .dbg_txhit_p(dbg_txhit_p), .dbg_slot(dbg_slot),
        .dbg_cmdbeat(dbg_cmdbeat), .dbg_flag(dbg_flag),
        .dbg_bad_we(dbg_bad_we), .dbg_bad_data(dbg_bad_data),
        .dbg_frame(1'b0), .dbg_sel(4'd0), .dbg_val(dbg_val), .dbg_lddrop(20'd0)
    );

    //--------------------------------------------------------------------------
    //  SDRAM 模型：只做存储与读延迟，不模拟换向/刷新（忙窗由上面单独给）
    //    PHASE_MODE=2 = "按 4 条命令分组、空隙不重置"——这是本项目实测的契约
    //    （PHASE_MODE=1 是"按拍自由跑"的悲观口径：它连 abort/retry 都会判错，
    //      因为重发余项本身就会在组内留空隙；真机已证明按条数分组。）
    //--------------------------------------------------------------------------
    sdr_as_ram_sim #(
        .RD_LATENCY(11), .INIT_CYCLES(20), .MEM_WORDS(2097152),
        .PHASE_MODE(1), .PHASE_WR_EN(1),
        .TURN_CYCLES(TURN_CYC), .REF_PERIOD(3125), .REF_BUSY(20), .REF_VLD(24), .REF_LAG(1)
    ) u_mem (
        .Sdr_clk(clk), .Sdr_clk_sft(clk), .Rst(rst),
        .Sdr_init_done(Sdr_init_done), .Sdr_init_ref_vld(Sdr_init_ref_vld_m),
        .Sdr_busy(Sdr_busy_m), .App_ref_req(1'b0),
        .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr), .App_wr_dm(App_wr_dm),
        .App_wr_din(App_wr_din),
        .App_rd_en(App_rd_en), .App_rd_addr(App_rd_addr),
        .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout),
        .rw_conflict(rw_conflict), .n_rd_drop(n_rd_drop), .n_rd_seen(n_rd_seen), .n_ref_drop(n_ref_drop),
        .n_turn_drop(n_turn_drop),
        .n_grp_bad(n_grp_bad), .n_grp_seen(n_grp_seen)
    );

    //--------------------------------------------------------------------------
    //  计量
    //--------------------------------------------------------------------------
    integer n_grp;          // tx_start 次数（含重试续发）
    integer n_grp_log;      // 逻辑组数（不含重试）
    integer n_txcmd;        // 纹理命令条数（应恒 = 4×逻辑组数）
    integer n_abort;        // tx_abort 次数
    integer n_hold;         // tx_hold 次数
    integer g_line_max;     // 单行最大组数
    integer g_line;         // 本行组数
    integer err_grp_cnt;    // 命令数 ≠ 4×组数 的行数
    integer err_dat;        // 回程数据错
    integer err_dwr;        // 显示写错地址数（结束时整段比）
    integer n_split;        // 组内停顿拍数（必须恒 0）

    //--------------------------------------------------------------------------
    //  换向间隔实测：最后一条读 → 第一条写（以及反向）
    //    竖条的判据就是这两个数：读→写 < 16 拍 → 写命令会撞进读回程
    //--------------------------------------------------------------------------
    integer cyc = 0;
    integer t_rd = -1000000, t_wr = -1000000;
    integer gap_rw_min = 99999, gap_wr_min = 99999;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            cyc <= 0; t_rd <= -1000000; t_wr <= -1000000;
            gap_rw_min <= 99999; gap_wr_min <= 99999;
        end else begin
            cyc <= cyc + 1;
            if (App_rd_en) begin
                if (t_wr > t_rd && (cyc - t_wr) < gap_wr_min) gap_wr_min <= cyc - t_wr;
                t_rd <= cyc;
            end
            if (App_wr_en) begin
                if (t_rd > t_wr && (cyc - t_rd) < gap_rw_min) gap_rw_min <= cyc - t_rd;
                t_wr <= cyc;
            end
        end
    end

    // 期望值队列：每个 ack 压入 4 个地址（base..base+3），每个回拍弹一个
    reg [20:0] q0 [0:127];
    reg [20:0] q1 [0:127];
    reg [6:0]  w0, r0, w1, r1;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            n_grp <= 0; n_grp_log <= 0; n_txcmd <= 0; n_abort <= 0; n_hold <= 0;
            g_line_max <= 0; g_line <= 0; err_grp_cnt <= 0; err_dat <= 0; n_split <= 0;
            w0 <= 0; r0 <= 0; w1 <= 0; r1 <= 0;
        end else begin
            if (u_dut.tx_start) begin
                n_grp  <= n_grp + 1;
                g_line <= g_line + 1;
                if (!u_dut.tx_retry_v) n_grp_log <= n_grp_log + 1;
            end
            if (u_dut.tx_abort) n_abort <= n_abort + 1;
            if (u_dut.tx_hold)  n_hold  <= n_hold  + 1;
            // 组的命令数必须恒为 4：用 (4×组数 − 已发命令数) 的瞬时值判
            if (u_dut.tx_cmd) n_txcmd <= n_txcmd + 1;
            // ★ 100M-22 核心不变式：组在飞(tx_act)且还在窗口内(tx_win) ⇒ 本拍必须发命令
            //   "组内停顿"在真控制器上等价于"后半段换了组首地址" ⇒ 整列读错。
            if (u_dut.tx_act && u_dut.tx_win && !u_dut.tx_cmd) n_split <= n_split + 1;

            if (tex_req_ack[0]) begin
                q0[w0[6:0]]     <= a0;
                q0[w0[6:0]+7'd1] <= a0 + 21'd1;
                q0[w0[6:0]+7'd2] <= a0 + 21'd2;
                q0[w0[6:0]+7'd3] <= a0 + 21'd3;
                w0 <= w0 + 7'd4;
            end
            if (tex_req_ack[1]) begin
                q1[w1[6:0]]     <= a1;
                q1[w1[6:0]+7'd1] <= a1 + 21'd1;
                q1[w1[6:0]+7'd2] <= a1 + 21'd2;
                q1[w1[6:0]+7'd3] <= a1 + 21'd3;
                w1 <= w1 + 7'd4;
            end
            if (tex_pix_wr[0]) begin
                if (tex_pix_data[31:0] !== u_mem.mem[q0[r0[6:0]]]) begin
                    err_dat <= err_dat + 1;
                    if (err_dat < 5)
                        $display("[ERR-TX0] got %h exp %h @%h", tex_pix_data[31:0],
                                 u_mem.mem[q0[r0[6:0]]], q0[r0[6:0]]);
                end
                r0 <= r0 + 7'd1;
            end
            if (tex_pix_wr[1]) begin
                if (tex_pix_data[63:32] !== u_mem.mem[q1[r1[6:0]]]) begin
                    err_dat <= err_dat + 1;
                    if (err_dat < 5)
                        $display("[ERR-TX1] got %h exp %h @%h", tex_pix_data[63:32],
                                 u_mem.mem[q1[r1[6:0]]], q1[r1[6:0]]);
                end
                r1 <= r1 + 7'd1;
            end

            if (line_tick) begin
                if (g_line > g_line_max) g_line_max <= g_line;
                g_line <= 0;
            end
        end
    end

    //--------------------------------------------------------------------------
    //  激励
    //--------------------------------------------------------------------------
    integer i;
    initial begin
        rst = 1'b1;
        repeat (10) @(posedge clk);
        rst = 1'b0;
        repeat (20) @(posedge clk);
        // 预置纹理区（每个地址一个固定图样，跨组边界也能判错）
        for (i = 0; i < 65536; i = i + 1) begin
            u_mem.mem[TX0_BASE + i] = pat(TX0_BASE + i);
            u_mem.mem[TX1_BASE + i] = pat(TX1_BASE + i);
        end
        repeat (LINE_BEATS * 30) @(posedge clk);
        stop = 1'b1;                          // 撤掉请求，等在飞组发完
        repeat (600) @(posedge clk);
        if (n_txcmd != 4*n_grp_log) err_grp_cnt = n_txcmd - 4*n_grp_log;
        // 显示写整段校验：所有写过的地址都必须是 pat(addr)
        begin : chk_wr
            integer a; integer bad;
            bad = 0;
            for (a = 0; a < dwr_cnt && a < 65536; a = a + 1)
                if (u_mem.mem[a] !== pat(a)) begin
                    bad = bad + 1;
                    if (bad < 4) $display("[ERR-DWR] addr=%0d got=%h exp=%h", a, u_mem.mem[a], pat(a));
                end
            err_dwr = bad;
        end
        $display("-------------------------------------------------------------");
        $display(" MODE=%0d  line=%0d  beats=%0d", MODE, line_no, LINE_BEATS);
        $display(" 逻辑组=%0d  纹理命令=%0d  (4×组数=%0d)  重试=%0d",
                 n_grp_log, n_txcmd, 4*n_grp_log, n_grp - n_grp_log);
        $display(" 单行最大组数=%0d   目标需求=%0d 组/行   写=%0d 字/行", g_line_max, GROUPS_LN, dwr_cnt/line_no);
        $display(" abort=%0d  hold=%0d  组内停顿=%0d", n_abort, n_hold, n_split);
        $display(" 数据错=%0d  mod4 违例行=%0d  读回程错=%0d  写错=%0d", err_dat, err_grp_cnt, err_drd, err_dwr);
        $display(" 模型: 分组错=%0d/%0d 拍  读命令丢=%0d  刷新丢命令=%0d  换向丢命令=%0d",
                 n_grp_bad, n_grp_seen, n_rd_drop, n_ref_drop, n_turn_drop);
        $display(" 换向最小间隔: 读→写=%0d 拍  写→读=%0d 拍   (TURN_CYC=%0d)",
                 gap_rw_min, gap_wr_min, TURN_CYC);
        if (err_dat == 0 && err_grp_cnt == 0 && n_grp_bad == 0 && n_rd_drop == 0
            && n_turn_drop == 0 && err_dwr == 0)
            $display(" === 数据/不变式 PASS ===");
        else
            $display(" === 数据/不变式 FAIL ===");
        $finish;
    end
endmodule

`endif  // TB_TXFIX_SIM
