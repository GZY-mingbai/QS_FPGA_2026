//==============================================================================
//  tb_frame —— 整条显示存储链路的功能/吞吐仿真
//==============================================================================
//  目的：在本地复现"上板全黑（数码管 b = 没有一帧收满）"。
//  被测组合 = sdr_port_sched + frame_read_write + sdr_as_ram_sim（4 字分组惩罚）
//
//  真实工况用"占用率"来模拟（全部跑在 125MHz，靠节奏模拟 25MHz）：
//    · 写侧：每 5 拍送 1 个像素（= 25MHz 灌进 125MHz 域），共 307200 个
//    · 读侧：每 5 拍取 1 个像素，共 307200 个
//  判据：
//    · write_finish 必须出现（否则 display_valid 永远 0 = 纯黑）
//    · read_finish  必须出现
//    · 读 FIFO 不能见底（见底 = 画面撕裂）
//    · App_rd_en 与 App_wr_en 不能同拍
//
//  跑法：
//    iverilog -g2005 -s tb_frame -o f.vvp SD/sdr_port_sched.v SD/frame_read_write.v \
//        SD/frame_fifo_read.v SD/frame_fifo_write.v SD/model/sdr_as_ram_sim.v SD/tb_frame.v
//==============================================================================
`timescale 1ns/1ps

module tb_frame;
    parameter integer RD_LAT     = 20;
    parameter integer PHASE_MODE = 1;
    parameter integer PACE_W     = 5;      // 写侧每 N 拍 1 像素
    parameter integer PACE_R     = 5;      // 读侧每 N 拍 1 像素
    parameter integer NPIX       = 307200;

    //--------------------------------------------------------------------------
    //  时钟 / 复位
    //--------------------------------------------------------------------------
    reg clk = 1'b0;
    reg rst = 1'b1;
    always #4 clk = ~clk;                  // 125MHz

    //--------------------------------------------------------------------------
    //  调度器 <-> 引擎
    //--------------------------------------------------------------------------
    wire        fw_rd_en, fw_rd_vld;
    wire [20:0] fw_rd_addr;
    wire        fw_rd_ready;
    wire [31:0] fw_rd_data;

    wire        fw_wr_en, fw_wr_ready;
    wire [20:0] fw_wr_addr;
    wire [31:0] fw_wr_din;
    wire [3:0]  fw_wr_dm;
    wire        fw_wr_busy;

    wire [1:0]  tex_req_ack, tex_pix_wr;
    wire [63:0] tex_pix_data;
    wire        ld_wr_ack;

    wire        App_rd_en, App_wr_en;
    wire [20:0] App_rd_addr, App_wr_addr;
    wire [31:0] App_wr_din;
    wire [3:0]  App_wr_dm;
    wire        Sdr_rd_en;
    wire [31:0] Sdr_rd_dout;
    wire [3:0]  dbg_slot;

    wire [9:0]  rd_avail_w;
    wire        rd_empty_w, rd_full_w;
    wire [9:0]  wr_used_w;
    wire        wr_full_w;
    wire [8:0]  rd_used_w;

    //--------------------------------------------------------------------------
    //  上层：写一帧
    //--------------------------------------------------------------------------
    reg         w_req;
    wire        w_ack, w_finish;
    reg         w_en;
    reg  [31:0] w_data;
    reg  [31:0] w_cnt;          // 已送出的像素数
    reg  [3:0]  w_div;
    integer     fin_cnt;
    reg         w_done;

    assign fw_wr_dm = 4'h0;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            w_req  <= 1'b1;     // 上电就请求写一帧
            w_en   <= 1'b0;
            w_data <= 32'd0;
            w_cnt  <= 32'd0;
            w_div  <= 4'd0;
            w_done <= 1'b0;
        end else begin
            if (w_ack) w_req <= 1'b0;
            // 应答之后开始按节奏灌像素
            if (!w_req && w_ack === 1'b0 && w_cnt < NPIX && !w_done) begin
                if (w_div == PACE_W-1) begin
                    w_div <= 4'd0;
                    w_en  <= 1'b1;
                    w_data <= w_cnt;
                    w_cnt  <= w_cnt + 32'd1;
                end else begin
                    w_div <= w_div + 4'd1;
                    w_en  <= 1'b0;
                end
            end else begin
                w_en <= 1'b0;
            end
            if (w_cnt >= NPIX) w_done <= 1'b1;
        end
    end

    //--------------------------------------------------------------------------
    //  上层：读一帧
    //--------------------------------------------------------------------------
    reg         r_req;
    wire        r_ack, r_finish;
    reg         r_en;
    reg  [3:0]  r_div;
    reg  [31:0] r_cnt;
    wire [31:0] r_data;
    reg  [9:0]  r_min;
    reg         r_fin_seen;
    integer     r_stall;        // 视频端"该取数但 FIFO 空了"的次数 = 真欠载
    reg         r_first_done;
    integer     r_pace_due;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            r_req <= 1'b1;
            r_en  <= 1'b0;
            r_div <= 4'd0;
            r_cnt <= 32'd0;
            r_min <= 10'h3FF;
            r_stall <= 0;
            r_first_done <= 1'b0;
            r_pace_due <= 0;
            r_fin_seen <= 1'b0;
        end else begin
            if (r_ack) r_req <= 1'b0;
            if (r_finish) r_fin_seen <= 1'b1;
            // 节拍：每 PACE_R 拍想取一个；只有 FIFO 非空才真的取（模拟视频端）
            if (!r_req && r_cnt < NPIX) begin
                if (r_div == PACE_R-1) begin
                    r_div <= 4'd0;
                    r_pace_due <= 1;
                end else begin
                    r_div <= r_div + 4'd1;
                    r_pace_due <= 0;
                end
                if (r_pace_due) begin
                    if (rd_avail_w != 0) begin
                        r_en  <= 1'b1;
                        r_cnt <= r_cnt + 32'd1;
                    end else begin
                        r_en  <= 1'b0;
                        r_stall <= r_stall + 1;     // 该取却没有 → 欠载
                    end
                end else begin
                    r_en <= 1'b0;
                end
            end else begin
                r_en <= 1'b0;
                r_pace_due <= 0;
            end
            // 水位只统计"整帧取满一轮之后"的稳态
            if (r_cnt > 32'd2048) begin
                if (rd_avail_w < r_min) r_min <= rd_avail_w;
                r_first_done <= 1'b1;
            end
        end
    end

    //--------------------------------------------------------------------------
    //  调度器
    //--------------------------------------------------------------------------
    sdr_port_sched #(.TAG_AW(8)) u_sched (
        .clk          (clk),
        .rst          (rst),
        .disp_rd_en   (fw_rd_en),
        .disp_rd_addr (fw_rd_addr),
        .disp_rd_ready(fw_rd_ready),
        .disp_rd_vld  (fw_rd_vld),
        .disp_rd_data (fw_rd_data),
        .disp_wr_en   (fw_wr_en),
        .disp_wr_addr (fw_wr_addr),
        .disp_wr_din  (fw_wr_din),
        .disp_wr_dm   (fw_wr_dm),
        .disp_wr_ready(fw_wr_ready),
        // 纹理：本测试不请求（vld 恒 0）
        .tex_req_vld  (2'b00),
        .tex_req_addr (42'd0),
        .tex_req_ack  (tex_req_ack),
        .tex_pix_wr   (tex_pix_wr),
        .tex_pix_data (tex_pix_data),
        // 装载：本测试空闲
        .ld_wr_vld    (1'b0),
        .ld_wr_used   (5'd0),
        .ld_wr_addr   (21'd0),
        .ld_wr_data   (32'd0),
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
        .dbg_slot     (dbg_slot)
    );

    //--------------------------------------------------------------------------
    //  帧缓存搬运（关闭引擎间互锁，互斥交给时隙表）
    //--------------------------------------------------------------------------
    frame_read_write #(
        .WRITE_V_FLIP  (0),
        .FRAME_WIDTH   (640),
        .FRAME_HEIGHT  (480),
        .INTERLOCK     (0)
    ) u_frw (
        .rst              (rst),
        .mem_clk          (clk),
        .Sdr_init_done    (1'b1),
        .Sdr_init_ref_vld (1'b0),
        .Sdr_busy         (1'b0),
        .Sdr_rd_en        (fw_rd_vld),
        .Sdr_rd_dout      (fw_rd_data),
        .rd_ready         (fw_rd_ready),
        .rd_busy_ext      (1'b0),
        .wr_ready         (fw_wr_ready),
        .wr_busy_o        (fw_wr_busy),

        .read_clk         (clk),
        .read_req         (r_req),
        .read_req_ack     (r_ack),
        .read_finish      (r_finish),
        .read_addr_0      (21'd0),
        .read_addr_1      (NPIX),
        .read_addr_2      (21'd0),
        .read_addr_3      (21'd0),
        .read_addr_index  (2'd0),
        .read_len         (NPIX),
        .read_en          (r_en),
        .read_data        (r_data),

        .App_rd_en        (fw_rd_en),
        .App_rd_addr      (fw_rd_addr),
        .App_wr_en        (fw_wr_en),
        .App_wr_addr      (fw_wr_addr),
        .App_wr_din       (fw_wr_din),
        .App_wr_dm        (fw_wr_dm),

        .write_clk        (clk),
        .write_req        (w_req),
        .write_req_ack    (w_ack),
        .write_finish     (w_finish),
        .write_addr_0     (21'd0),
        .write_addr_1     (NPIX),
        .write_addr_2     (21'd0),
        .write_addr_3     (21'd0),
        .write_addr_index (2'd0),
        .write_len        (NPIX),
        .write_en         (w_en),
        .write_data       (w_data),

        .rd_avail         (rd_avail_w),
        .rd_fifo_empty    (rd_empty_w),
        .rd_fifo_full     (rd_full_w),
        .wr_fifo_used     (wr_used_w),
        .wr_fifo_full     (wr_full_w),
        .rd_buf_used      (rd_used_w)
    );

    //--------------------------------------------------------------------------
    //  SDRAM 模型
    //--------------------------------------------------------------------------
    wire        Sdr_init_done_m, Sdr_init_ref_vld_m, Sdr_busy_m, rw_conflict;
    wire [31:0] n_rd_drop, n_rd_seen, n_grp_bad, n_grp_seen;

    sdr_as_ram_sim #(
        .RD_LATENCY  (RD_LAT),
        .INIT_CYCLES (50),
        .MEM_WORDS   (1048576),
        .PHASE_MODE  (PHASE_MODE),
        .PHASE_WR_EN (1)
    ) u_mem (
        .Sdr_clk          (clk),
        .Sdr_clk_sft      (clk),
        .Rst              (rst),
        .Sdr_init_done    (Sdr_init_done_m),
        .Sdr_init_ref_vld (Sdr_init_ref_vld_m),
        .Sdr_busy         (Sdr_busy_m),
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
    //  统计
    //--------------------------------------------------------------------------
    integer con_cnt;
    always @(posedge clk) begin
        if (rst) con_cnt <= 0;
        else begin
            if (w_finish) fin_cnt <= fin_cnt + 1;
            if (App_rd_en && App_wr_en) con_cnt <= con_cnt + 1;
        end
    end

    initial begin
        fin_cnt = 0;
        con_cnt = 0;
        rst = 1'b1;
        repeat (20) @(posedge clk);
        rst = 1'b0;
        // 等写完成（最多 3 帧时间 ≈ 3×4000×480 拍）
        repeat (NPIX * 12) @(posedge clk);   // 约 3 帧余量

        $display("-------------------------------------------------------------");
        $display(" 写侧: 送 %0d/%0d 像素, write_finish 次数 %0d, 写FIFO满过 %b",
                 w_cnt, NPIX, fin_cnt, wr_full_w);
        $display(" 读侧: 取 %0d/%0d 像素, read_finish=%b, 稳态最小余量 %0d, 欠载次数 %0d",
                 r_cnt, NPIX, r_fin_seen, r_min, r_stall);
        $display(" 读写同拍 %0d | 模型分组错 %0d/%0d | 读命令丢 %0d",
                 con_cnt, n_grp_bad, n_grp_seen, n_rd_drop);
        if (fin_cnt > 0 && con_cnt == 0 && r_fin_seen && r_stall < 64)
            $display(" === PASS ===");
        else
            $display(" === FAIL ===");
        $display("-------------------------------------------------------------");
        $finish;
    end

endmodule
