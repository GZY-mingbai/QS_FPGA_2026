`timescale 1ns/1ps
//==============================================================================
//  tb_chain_img.v —— 真链路：卡镜像 → sd_card_model → sd_card_top
//                          → sd_bus_arbiter → bmp_loader → 写口
//==============================================================================
//  【为什么要有这个台】`tb_bmp_loader` 里的扇区从机是"收到 sd_sec_read 就吐 512
//  字节"，**没有 A_GAP、没有真实 end 时序**。而真链路里：
//    · 裁决器的请求口是电平语义，A_IDLE 才采样 ⇒ 客户端的单拍脉冲可能落空
//    · sd_sec_read_end 是**组合**输出，且地址只在"开始一次读"时锁存
//  所以"仿真过、上板挂"的差集只能在这里暴露。
//
//  用例：卡偏移 0 放一张真 BMP（640×480/24bit），装成 mode0/888/zoom1.0，
//        检查 src_w/src_h、done、以及纹理内容（逐像素，导出 PPM 给 Python 比）。
//==============================================================================
module tb_chain_img;

parameter SLD_FILE = "out/card_img.bin";
parameter integer IMG_W = 640, IMG_H = 480;
parameter integer FILE_LEN = 921654;
parameter integer MODE    = 0;        // 0=640x480 1=320x240 2=160x120
parameter integer FMT     = 1;        // 1=888 0=565
parameter integer ZOOM    = 8'h10;
parameter integer STOP_ROW = 0;       // >0：跑满这么多行就停并打印（快速统计用）
parameter integer T0_NS    = 0;

reg clk = 0;
always #10 clk = ~clk;                  // 50MHz
reg rst = 1;
reg mclk = 0;
always #4 mclk = ~mclk;                 // 125MHz 写口域（近似）

//----------------------------- 卡 + 控制器 ------------------------------------
wire ncs, dclk_sd, mosi, miso;

sd_card_model #(
    .MEM_BYTES (2097152),
    .INIT_FILE (SLD_FILE)
) u_card (
    .nCS (ncs), .DCLK (dclk_sd), .MOSI (mosi), .MISO (miso)
);

wire        sd_init_done, sd_dv, sd_end;
wire [7:0]  sd_data;
wire        sd_sec_read;                // 裁决器 → 控制器（电平）
wire [31:0] ctl_addr;                   // 裁决器 → 控制器（地址）
wire        img_req;                    // 装载器 → 裁决器（单拍脉冲）
wire [31:0] img_addr;                   // 装载器 → 裁决器（地址）

sd_card_top u_top (
    .clk                    (clk),
    .rst                    (rst),
    .SD_nCS                 (ncs),
    .SD_DCLK                (dclk_sd),
    .SD_MOSI                (mosi),
    .SD_MISO                (miso),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (ctl_addr),
    .sd_sec_read_data       (sd_data),
    .sd_sec_read_data_valid (sd_dv),
    .sd_sec_read_end        (sd_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (8'd0),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

//----------------------------- 仲裁器（音频客户常空）---------------------------
wire        img_dv, img_end;
wire [7:0]  img_data;
wire        arb_timeout;

//----------------------------- 假音频客户（模拟放音乐）-------------------------
//  FIFO 水位：每 1042 拍（48kHz@50MHz）消耗 1 字；收到一个扇区 +128 字。
//  req   = 水位 < 768（预取），urgent = 水位 <= 256（抢占）
parameter integer AUD_EN = 1;
reg  [10:0] aud_lvl = 11'd0;
reg  [31:0] aud_consume = 0;
// AUD_MODE=0：礼貌客户（按 48kHz 需求取）；1：**恶意客户**（一直抢、一直 urgent）
parameter integer AUD_MODE = 0;
wire        aud_req  = AUD_EN && ((AUD_MODE != 0) ? 1'b1 : (aud_lvl < 11'd768));
wire        aud_urg  = AUD_EN && ((AUD_MODE != 0) ? 1'b1 : (aud_lvl <= 11'd256));
wire [7:0]  aud_data;
wire        aud_dv, aud_end;
integer     aud_sec = 0;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        aud_lvl <= 11'd0; aud_consume <= 32'd0; aud_sec <= 0;
    end else begin
        aud_consume <= aud_consume + 32'd1;
        if (aud_consume >= 32'd1042) begin
            aud_consume <= 32'd0;
            if (aud_lvl != 11'd0) aud_lvl <= aud_lvl - 11'd1;   // 播放消耗
        end
        if (aud_end && (aud_lvl < 11'd896)) begin
            aud_lvl <= aud_lvl + 11'd128;                       // 收到一个扇区
            aud_sec <= aud_sec + 1;   // ★ 2026-10-31：<= 与复位分支统一（Verilator 禁止混用）
        end
    end
end

sd_bus_arbiter u_arb (
    .I_clk           (clk),
    .I_rst           (rst),
    .I_sd_init_done  (sd_init_done),
    .I_img_req       (img_req),
    .I_img_addr      (img_addr),
    .O_img_data      (img_data),
    .O_img_dv        (img_dv),
    .O_img_end       (img_end),
    .I_aud_req       (aud_req),
    .I_aud_addr      (32'd2000),
    .I_aud_urgent    (aud_urg),
    .O_aud_data      (aud_data),
    .O_aud_dv        (aud_dv),
    .O_aud_end       (aud_end),
    .O_sd_sec_read   (sd_sec_read),
    .O_sd_sec_addr   (ctl_addr),
    .I_sd_sec_data   (sd_data),
    .I_sd_sec_dv     (sd_dv),
    .I_sd_sec_end    (sd_end),
    .O_timeout       (arb_timeout),
    .O_owner         ()
);

//----------------------------- 装载器 ----------------------------------------
reg         start = 0;
reg  [20:0] hw_base = 21'd0;
integer     sec_req_cnt = 0, rowgo_cnt = 0, emit_cnt = 0, sec_sum_done = 0;
reg         req_q = 1'b0, go_q = 1'b0;
always @(posedge clk) begin
    req_q <= sd_sec_read;
    if (sd_sec_read && !req_q) sec_req_cnt = sec_req_cnt + 1;
    go_q  <= dut.row_go;
    if (dut.row_go && !go_q) begin
        rowgo_cnt = rowgo_cnt + 1;
        if ((STOP_ROW > 0) && (rowgo_cnt >= STOP_ROW) && (sec_sum_done == 0)) begin
            sec_sum_done = 1;
            $display("  [统计] 跑到第 %0d 行：扇区请求 %0d 次，用时 %0t，平均 %.2f 扇区/行",
                     rowgo_cnt, sec_req_cnt, $time, sec_req_cnt * 1.0 / rowgo_cnt);
            $display("  [统计] 折合 %.1f ms 每行（50MHz 时钟）", $time / 1000.0 / 50000.0 / rowgo_cnt * 1000.0);
            $finish;
        end
    end
end
integer     load_cnt = 0, load_cyc = 0, load_err = 0;
wire        ready, busy, done, err;
wire [15:0] src_w_o, src_h_o;

wire        wr_vld;
wire [9:0]  wr_used;
wire [20:0] wr_addr;
wire [31:0] wr_data;
wire        wr_ack = wr_vld;            // 写口永远当场取走

bmp_loader #(.AFIFO_AW(9)) dut (
    .clk(clk), .rst(rst), .mclk(mclk),
    .start(start), .abort(1'b0), .file_offset(32'd0), .file_length(FILE_LEN),
    .dst_base(hw_base),
    .mode(MODE[1:0]), .fmt(FMT[0]), .dst_stride(11'd0),
    .zoom(ZOOM[7:0]), .ax(10'd320), .ay(10'd240),
    .ready(ready), .busy(busy), .done(done), .err(err),
    .src_w_o(src_w_o), .src_h_o(src_h_o),
    .sd_sec_read(img_req), .sd_sec_read_addr(img_addr),
    .sd_sec_read_data(img_data), .sd_sec_read_data_valid(img_dv),
    .sd_sec_read_end(img_end), .sd_init_done(sd_init_done),
    .wr_vld(wr_vld), .wr_used(wr_used), .wr_addr(wr_addr), .wr_data(wr_data),
    .wr_ack(wr_ack), .wr_settled(1'b1)
);

//----------------------------- 写口捕获 --------------------------------------
localparam integer MAXW = 307200;
reg [31:0] mem [0:MAXW-1];
reg [7:0]  cnt [0:MAXW-1];
integer k, nwr = 0;

always @(posedge mclk) begin
    if (wr_vld && wr_ack) begin
        mem[wr_addr] <= wr_data;
        cnt[wr_addr] <= cnt[wr_addr] + 8'd1;
        nwr = nwr + 1;
    end
end

//----------------------------- 一次装载 --------------------------------------
task do_load;
    input [20:0] b;
    integer g;
    begin
        load_cnt = load_cnt + 1;
        hw_base = b;
        sec_req_cnt = 0; rowgo_cnt = 0;
        @(negedge clk); start = 1'b1;
        @(negedge clk); start = 1'b0;
        g = 0;
        while (!done && !err && g < 60000000) begin @(posedge clk); g = g + 1; end
        load_cyc = g;
        if (!done || err) load_err = load_err + 1;
        $display("  [2.%0d] base=%0d done=%b err=%b 用了 %0d 拍 st=%0d foff=%0d sec=%0d hcnt=%0d w32=%h hb=%h%h",
                 load_cnt, b, done, err, g, dut.st, dut.foff, dut.sec, dut.hcnt, dut.w32, dut.hb0, dut.hb1);
        $display("       ★ 扇区请求 %0d 次，row_go %0d 次，平均 %.2f 扇区/行",
                 sec_req_cnt, rowgo_cnt,
                 (rowgo_cnt != 0) ? (sec_req_cnt * 1.0 / rowgo_cnt) : 0.0);
    end
endtask

//----------------------------- 主流程 ----------------------------------------
integer guard;
integer fd, m, x;
reg [31:0] w0;
reg [7:0] r, g, b;

initial begin
    for (k = 0; k < MAXW; k = k + 1) begin mem[k] = 32'd0; cnt[k] = 8'd0; end

    $display("======================================================");
    repeat (8) @(posedge clk);
    rst = 0;

    // 等卡初始化
    guard = 0;
    while (!sd_init_done && guard < 4000000) begin @(posedge clk); guard = guard + 1; end
    $display("  [1] sd_init_done=%b（%0d 拍）", sd_init_done, guard);
    if (!sd_init_done) begin
        $display("  *** FAIL 卡初始化失败"); $display("  STOP"); $finish;
    end

    // 连续两次装载（第 2 次 = 切图；专门覆盖"请求落进仲裁器 A_GAP"那种丢请求）
    do_load(21'd0);

    $display("  [3] src_w=%0d src_h=%0d  arb_timeout=%b  写口 %0d 字",
             src_w_o, src_h_o, arb_timeout, nwr);
    $display("  [3b] 音频抢到 %0d 个扇区，最终水位 %0d", aud_sec, aud_lvl);
    if (load_err != 0) $display("  *** FAIL 有装载未完成");

    // 内容抽查：纹理行 m ← 图像行 (479-m)（888 自下而上）
    begin : spot
        integer bad, first;
        bad = 0; first = -1;
        for (m = 0; m < IMG_H; m = m + 1)
            for (x = 0; x < IMG_W; x = x + 1) begin
                w0 = mem[m * IMG_W + x];
                if (cnt[m * IMG_W + x] != 8'd1) bad = bad + 1;
                else if (w0[31:24] == 8'd0) begin
                    if (bad < 5) $display("     (m=%0d x=%0d) 黑字（没写/写 0）", m, x);
                    bad = bad + 1;
                end
            end
        $display("  [4] 第1张地址/透明度异常 %0d / %0d 字", bad, IMG_W * IMG_H);
        // ★ 2026-10-31：原来只打印数字、不给结论 ⇒ 跑批时看不出过没过。
        if ((load_err == 0) && (bad == 0) && (arb_timeout == 0) &&
            (src_w_o == IMG_W) && (src_h_o == IMG_H) && (nwr == IMG_W * IMG_H))
            $display("  ALL PASS  (chain: init/load/size/%0d words/alpha)", nwr);
        else
            $display("  *** FAIL  chain verdict (err=%0d bad=%0d to=%b size=%0dx%0d wr=%0d)",
                     load_err, bad, arb_timeout, src_w_o, src_h_o, nwr);
    end

    // 导出 PPM（图像自然行序：纹理行 m = 图像行 479-m）
    begin : dump
        $display("  [5] 导出 out/chain.ppm");
        fd = $fopen("out/chain.ppm", "wb");
        $fwrite(fd, "P6\n%0d %0d\n255\n", IMG_W, IMG_H);
        for (m = 0; m < IMG_H; m = m + 1)
            for (x = 0; x < IMG_W; x = x + 1) begin
                w0 = mem[(IMG_H - 1 - m) * IMG_W + x];
                $fwrite(fd, "%c%c%c", w0[23:16], w0[15:8], w0[7:0]);
            end
        $fclose(fd);
    end

    $display("  STOP");
    $finish;
end

initial begin
    #2500000000;                 // 2.5s（整幅 640×480 要 ~0.6s）
    $display("  *** GLOBAL TIMEOUT");
    $finish;
end

endmodule
