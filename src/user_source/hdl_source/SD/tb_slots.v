//==============================================================================
//  tb_slots —— 只验一件事：实验 J 改完之后，16 个槽的归属对不对
//==============================================================================
//  不测数据正确性（那个交给 tb_sched，但它已过期），只做静态拓扑核对：
//      期望  RD 0-3 │ TX 4-7 │ ID 8 │ WR 9-14 │ ID 15
//  owner 编码：0=ID 1=RD 2=WR 3=TX 4=LD
//  激励：所有"有需求"信号拉满 + Sdr_busy=0 + 不借槽（td_want=1、tx_req_vld=3）
//        ⇒ 时隙表自由跑，看到的就是 own_of 的真值。
//==============================================================================
`timescale 1ns/1ps
module tb_slots;
    // MODE 控制纹理需求的形态，用来覆盖"借槽"这条路径：
    //   1 = 恒有需求 ⇒ 不借（验证标准槽表 RD0-3/TX4-7/ID8/WR9-14）
    //   0 = 恒无需求 ⇒ 全程借（槽 5..7 应变成 WR，验证换向红线仍成立）
    //   2 = 方波（周期 40 拍，不是 16 的倍数）⇒ 时而借时而不借，
    //       专门打"槽4 ph15 发了命令、请求正好变空"那个致命边界
    parameter integer MODE = 1;

    reg clk = 0, rst = 1;
    always #5 clk = ~clk;

    integer cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;

    reg        disp_rd_en = 1, disp_rd_want = 1;
    reg [20:0] disp_rd_addr = 0;
    reg        disp_wr_en = 1;
    reg [20:0] disp_wr_addr = 0;
    reg [31:0] disp_wr_din = 0;
    reg [3:0]  disp_wr_dm = 4'hF;
    reg [1:0]  tex_req_vld = 2'b11;
    reg [41:0] tex_req_addr = 42'd0;
    reg [7:0]  tex_req_mask = 8'hFF;
    reg        ld_wr_vld = 0;
    reg [4:0]  ld_wr_used = 0;
    reg [20:0] ld_wr_addr = 0;
    reg [31:0] ld_wr_data = 0;
    reg        Sdr_rd_en = 1;
    reg [31:0] Sdr_rd_dout = 32'h1234_5678;
    reg        Sdr_busy = 0, Sdr_init_ref_vld = 0;
    reg        rd_lag = 0;
    reg [1:0]  tx_ph_off = 0, ph_off = 0;
    reg [3:0]  exp_mode = 0;
    reg        dbg_frame = 0, dbg_pvalid = 0, dbg_wstall = 0;
    // ★ 实验 L 第二版：这个必须驱动。悬空会变成 x ⇒ disp_wr_en_d 变 x
    //   ⇒ wr_go 变 x ⇒ 槽表和换向全都测不出来。
    //   设 1 = "写引擎一直有数据要写"，正好让写口满负荷，是换向最严苛的情形。
    reg        dbg_wbusy = 1;
    reg [3:0]  dbg_sel = 0;
    reg [19:0] dbg_lddrop = 0;

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

    sdr_port_sched dut (
        .clk(clk), .rst(rst),
        .disp_rd_en(disp_rd_en), .disp_rd_want(disp_rd_want), .disp_rd_addr(disp_rd_addr),
        .disp_rd_ready(disp_rd_ready), .disp_rd_vld(disp_rd_vld), .disp_rd_data(disp_rd_data),
        .disp_wr_en(disp_wr_en), .disp_wr_addr(disp_wr_addr), .disp_wr_din(disp_wr_din),
        .disp_wr_dm(disp_wr_dm), .disp_wr_ready(disp_wr_ready),
        .tex_req_vld(tex_req_vld), .tex_req_addr(tex_req_addr), .tex_req_mask(tex_req_mask),
        .tex_req_ack(tex_req_ack), .tex_pix_wr(tex_pix_wr), .tex_pix_data(tex_pix_data),
        .ld_wr_vld(ld_wr_vld), .ld_wr_used(ld_wr_used), .ld_wr_addr(ld_wr_addr),
        .ld_wr_data(ld_wr_data), .ld_wr_ack(ld_wr_ack), .ld_wr_settled(ld_wr_settled),
        .App_rd_en(App_rd_en), .App_rd_addr(App_rd_addr),
        .App_wr_en(App_wr_en), .App_wr_addr(App_wr_addr),
        .App_wr_din(App_wr_din), .App_wr_dm(App_wr_dm),
        .Sdr_rd_en(Sdr_rd_en), .Sdr_rd_dout(Sdr_rd_dout),
        .Sdr_busy(Sdr_busy), .Sdr_init_ref_vld(Sdr_init_ref_vld),
        .rd_lag(rd_lag), .tx_ph_off(tx_ph_off), .ph_off(ph_off), .exp_mode(exp_mode),
        .dbg_wbusy(dbg_wbusy)
    );

    integer   i;
    reg [2:0] seen [0:15];
    reg [2:0] exp_own;
    integer   errs;
    integer   n_rd, n_tx, n_id, n_wr;

    //==========================================================================
    //  换向红线实时检测
    //      读→写：拍号差 ≥ 16     （历史板上验证值）
    //      写→读：拍号差 ≥ 17
    //  ★ 注意：App_rd_en / App_wr_en 是"引擎愿望"（= disp_rd_en | tx_cmd 等），
    //    不是命令本体。真正的"这一拍发了命令"是 rd_go / tx_cmd / wr_go。
    //  ★ 判据换算：`since_xx` 在命令那一拍的**当拍值** = 拍号差 − 1。
    //    所以"拍号差 ≥ 16" ⇔ since_rd ≥ 15；"拍号差 ≥ 17" ⇔ since_wr ≥ 16。
    //    （第一版写成 <16 / <17，把自己差了 1 拍误报成违规。）
    //==========================================================================
    wire rd_cmd_s = dut.rd_go | dut.tx_cmd;
    wire wr_cmd_s = dut.wr_go;

    reg [7:0]  since_rd, since_wr;
    reg        rd_seen, wr_seen;
    integer    bad_rw, bad_wr, min_rw, min_wr;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            since_rd <= 8'd0; since_wr <= 8'd0;
            rd_seen  <= 1'b0; wr_seen <= 1'b0;
        end else begin
            if (rd_cmd_s) begin since_rd <= 8'd0; rd_seen <= 1'b1; end
            else if (since_rd != 8'd255) since_rd <= since_rd + 8'd1;
            if (wr_cmd_s) begin since_wr <= 8'd0; wr_seen <= 1'b1; end
            else if (since_wr != 8'd255) since_wr <= since_wr + 8'd1;
        end
    end

    always @(posedge clk) begin
        if (!rst) begin
            // 写命令发出时，距上一条读命令若拍号差 < 16 ⇒ 违反读→写红线
            if (wr_cmd_s && rd_seen) begin
                if (since_rd < 8'd15) begin
                    bad_rw = bad_rw + 1;
                    if (bad_rw <= 5)
                        $display("  [VIOL] read->write 拍号差 %0d (<16)  @ slot=%0d ph=%0d",
                                 since_rd + 8'd1, dut.slot, dut.ph);
                end
                if (since_rd < min_rw) min_rw = since_rd;
            end
            // 读命令发出时，距上一条写命令若拍号差 < 17 ⇒ 违反写→读红线
            if (rd_cmd_s && wr_seen) begin
                if (since_wr < 8'd16) begin
                    bad_wr = bad_wr + 1;
                    if (bad_wr <= 5)
                        $display("  [VIOL] write->read 拍号差 %0d (<17)  @ slot=%0d ph=%0d  rd_go=%b tx_cmd=%b",
                                 since_wr + 8'd1, dut.slot, dut.ph, dut.rd_go, dut.tx_cmd);
                end
                if (since_wr < min_wr) min_wr = since_wr;
            end
        end
    end

    // 期望表（作者结论：RD 4 / TX 4 / ID 8 / WR 9-14 是唯一稳定分配）
    //   MODE=1（恒有需求）⇒ 不借：RD 4 槽、TX 4 槽、WR 6 槽
    //   MODE=0（恒无需求）⇒ 借槽 5..7 给 WR ⇒ TX 1 槽、WR 9 槽
    //   MODE=2（方波）   ⇒ 动态，只验换向红线
    function [2:0] want;
        input integer s;
        begin
            // ★ 2026-10-23：时隙表改成 RD 6 / TX 4 / ID / WR 4 / ID|LD
            //   （RD 4→6 是为了给显示读留出真余量，消除 888 静止页的欠载青点）
            if (s <= 4)                     want = 3'd1;   // RD（5 槽，0..4）
            else if (s <= 8)                want = 3'd3;   // TX（4 槽，5..8）
            else if (s == 9)                want = 3'd0;   // ID（读→写 换向缓冲）
            else if (s <= 14)               want = 3'd2;   // WR（4 槽）
            else                            want = 3'd0;   // ID/LD
        end
    endfunction

    initial begin
        for (i = 0; i < 16; i = i + 1) seen[i] = 3'd7;   // 7 = 未观测到
        bad_rw = 0; bad_wr = 0;
        min_rw = 255; min_wr = 255;
        $display("");
        $display("  ===== MODE=%0d  %s =====", MODE,
                 (MODE == 0) ? "纹理恒无需求（全程借槽）" :
                 (MODE == 1) ? "纹理恒有需求（不借槽）" :
                               "方波需求（打借/不借的边界）");
        rst = 1;
        repeat (6) @(posedge clk);
        rst = 0;
        repeat (900) begin
            @(posedge clk);
            #1;
            if (dut.slot < 16) seen[dut.slot] = dut.own;
            // 纹理需求形态
            case (MODE)
                0       : tex_req_vld <= 2'b00;
                1       : tex_req_vld <= 2'b11;
                default : tex_req_vld <= ((cyc % 40) < 20) ? 2'b11 : 2'b00;
            endcase
        end

        $display("");
        $display("  slot   own(实读)  own(期望)   owner      判定");
        $display("  ----   ---------   ---------   --------   ----");
        errs = 0;
        for (i = 0; i < 16; i = i + 1) begin
            exp_own = want(i);
            if (seen[i] !== exp_own) errs = errs + 1;
            $display("   %2d       %0d           %0d        %-8s  %s",
                     i, seen[i], exp_own,
                     (exp_own == 3'd1) ? "RD" : (exp_own == 3'd2) ? "WR" :
                     (exp_own == 3'd3) ? "TX" : "ID",
                     (seen[i] === exp_own) ? "OK" : "*** MISMATCH ***");
        end
        $display("");
        n_rd = 0; n_tx = 0; n_id = 0; n_wr = 0;
        for (i = 0; i < 16; i = i + 1) begin
            if      (seen[i] == 3'd1) n_rd = n_rd + 1;
            else if (seen[i] == 3'd3) n_tx = n_tx + 1;
            else if (seen[i] == 3'd2) n_wr = n_wr + 1;
            else                      n_id = n_id + 1;
        end
        $display("  实读统计：RD=%0d TX=%0d ID=%0d WR=%0d", n_rd, n_tx, n_id, n_wr);
        if (errs == 0) $display("  === 槽归属 PASS ===");
        else           $display("  === 槽归属 FAIL：%0d 个槽不对 ===", errs);
        $display("");
        $display("  换向间隔（拍号差）：读->写 最小 %0d 拍（要求 ≥16），违规 %0d 次", min_rw + 8'd1, bad_rw);
        $display("                      写->读 最小 %0d 拍（要求 ≥17），违规 %0d 次", min_wr + 8'd1, bad_wr);
        if (bad_rw == 0 && bad_wr == 0) $display("  === 换向红线 PASS ===");
        else                            $display("  === 换向红线 FAIL ===");
        $finish;
    end
endmodule
