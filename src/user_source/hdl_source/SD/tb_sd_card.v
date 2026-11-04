`timescale 1ns/1ps
//==============================================================================
//  tb_sd_card.v —— SD 卡从机模型 + 驱动的"最小闭环"验证
//==============================================================================
//  只挂两个模块：sd_card_top（真驱动）+ sd_card_model（假卡）。
//  目的是把 SPI 协议对着跑通：CMD0/8/55/41/16 初始化 → CMD17 读一个扇区。
//  比 tb_full 快得多（模块少），用来调试协议细节。
//
//  【跑法】（工程根目录）
//    iverilog -g2005 -s tb_sd_card -o tb_sd_card.vvp \
//        src/user_source/hdl_source/SD/sd_card_top.v \
//        src/user_source/hdl_source/SD/sd_card_cmd.v \
//        src/user_source/hdl_source/SD/sd_card_sec_read_write.v \
//        src/user_source/hdl_source/SD/spi_master.v \
//        src/user_source/hdl_source/SD/sd_card_model.v \
//        src/user_source/hdl_source/SD/tb_sd_card.v
//    vvp tb_sd_card.vvp -P tb_sd_card.SLD_FILE=\"_sim/card_fast.bin\"
//==============================================================================
module tb_sd_card;

parameter SLD_FILE = "_sim/card_fast.bin";
parameter integer SEC_TO_READ = 0;

reg clk = 0;
reg rst = 1;
always #10 clk = ~clk;                  // 50MHz（和上板一致）

wire ncs, dclk, mosi, miso;

sd_card_model #(
    .MEM_BYTES (2097152),
    .INIT_FILE (SLD_FILE)
) u_card (
    .nCS  (ncs), .DCLK (dclk), .MOSI (mosi), .MISO (miso)
);

reg         sd_sec_read = 1'b0;
reg  [31:0] sd_sec_read_addr = 32'd0;
wire        sd_init_done, sd_sec_read_data_valid, sd_sec_read_end;
wire [7:0]  sd_sec_read_data;

sd_card_top u_top (
    .clk                    (clk),
    .rst                    (rst),
    .SD_nCS                 (ncs),
    .SD_DCLK                (dclk),
    .SD_MOSI                (mosi),
    .SD_MISO                (miso),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (8'd0),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

integer cyc = 0;
integer n_byte = 0;
integer n_err = 0;
integer guard;
reg     read_end_seen = 1'b0;

always @(posedge clk) cyc = cyc + 1;

// 读回的字节流水（前 24 个打出来看看）
always @(posedge clk) begin
    if (sd_sec_read_data_valid) begin
        if (n_byte < 24)
            $display("  [读回] #%0d = %02x", n_byte, sd_sec_read_data);
        n_byte = n_byte + 1;
    end
end

initial begin
    $display("");
    $display("===================================================================");
    $display("  tb_sd_card —— SD 卡模型 最小闭环（卡镜像 %0s，读扇区 %0d）",
             SLD_FILE, SEC_TO_READ);
    $display("===================================================================");

    repeat (20) @(posedge clk);
    rst = 0;
    repeat (20) @(posedge clk);

    // ---- 1. 等初始化 ----
    guard = 0;
    while (!sd_init_done && guard < 4000000) begin
        @(posedge clk);
        guard = guard + 1;
    end
    $display("  [1] 初始化：sd_init_done=%b（等了 %0d 拍 @50MHz）", sd_init_done, guard);
    $fflush;
    if (!sd_init_done) begin
        $display("  RESULT: FAIL —— 卡初始化没完成（SPI 协议对不上）");
        $display("===================================================================");
        $finish;
    end

    // ---- 2. 读一个扇区 ----
    // ★ sd_sec_read 要**一直举着**直到 sd_sec_read_end ★
    //   （协议层要先把 CMD16 收尾、回到 S_WAIT_READ_WRITE 才看得到这个请求。
    //     bmp_read.v 也是这么用的：置 1 后一直保持，直到 end 才撤。
    //     只给一拍脉冲会被漏掉。）
    @(posedge clk);
    sd_sec_read_addr = SEC_TO_READ[31:0];
    sd_sec_read      = 1'b1;

    guard = 0;
    read_end_seen = 1'b0;
    while (!sd_sec_read_end && guard < 4000000) begin
        @(posedge clk);
        guard = guard + 1;
        if (sd_sec_read_end) read_end_seen = 1'b1;
    end
    if (sd_sec_read_end) read_end_seen = 1'b1;   // end 只拉高 1 拍，必须锁存
    @(posedge clk);
    sd_sec_read = 1'b0;
    $display("  [2] 扇区 %0d 读完：end=%b，收到 %0d 字节（等了 %0d 拍）",
             SEC_TO_READ, read_end_seen, n_byte, guard);
    $display("  预期前 4 字节 = 卡镜像里 偏移 %0d 处的 4 个字节", SEC_TO_READ * 512);
    $display("-------------------------------------------------------------------");
    if (read_end_seen && n_byte == 512) $display("  RESULT: PASS");
    else                                   $display("  RESULT: FAIL");
    $display("===================================================================");
    $finish;
end

initial begin
    #100_000_000;                       // 100ms 仿真时间上限
    $display("  !! 看门狗超时（%0d 拍 @50MHz）", cyc);
    $finish;
end

endmodule
