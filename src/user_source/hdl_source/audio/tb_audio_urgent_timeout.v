`timescale 1ns/1ps
//==============================================================================
//  tb_audio_urgent_timeout.v —— 预灌 urgent 必须有时间上限（2026-10-30 上板故障）
//==============================================================================
//  【要证明什么】
//    播放使能（I_en=1）但 FIFO 水位一直填不满时，`O_sd_urgent` 不能永远为 1 ——
//    否则音频会把 SD 总线占死，图像装载器一个字节都拿不到
//    （上板现象：信息条 0×0、相册全是花屏；SW2 拨到不播放就一切正常）。
//
//  【断言】
//    A. 水位恒 0 + I_en=1：10ms（PRE_TMO）之后 urgent 必须落 0
//    B. 把水位抬到 PREFILL 之上：urgent 立刻是 0（正常路径不受影响）
//    C. I_en=0：urgent 恒 0（关播不抢总线）
//==============================================================================
module tb_audio_urgent_timeout;

reg clk = 0;
always #10 clk = ~clk;              // 50MHz

reg rst = 1;
reg en = 0;
reg [10:0] level = 0;

wire        req, urgent;
wire [31:0] addr;
wire [7:0]  data = 8'hFF;
wire        dv = 1'b0, endp = 1'b0;
wire        we;
wire [31:0] wdata;
wire        drop;
wire        pass_done;

sd_audio_reader #(.TOTAL_WORDS(8000)) dut (
    .I_clk(clk), .I_rst(rst), .I_en(en), .I_loop(1'b1),
    .O_sd_req(req), .O_sd_addr(addr), .O_sd_urgent(urgent),
    .I_sd_data(data), .I_sd_dv(dv), .I_sd_end(endp),
    .I_fifo_level(level), .I_fifo_full(1'b0),
    .O_fifo_we(we), .O_fifo_data(wdata), .O_drop(drop), .O_pass_done(pass_done)
);

integer fails = 0, cyc;
reg urgent_ever;

initial begin
    $display("======================================================");
    repeat (4) @(posedge clk);
    rst = 0;

    //------------------------------ A ------------------------------------
    en = 1'b1; level = 11'd0;
    cyc = 0; urgent_ever = 1'b0;
    while (cyc < 700_000) begin          // 14ms
        @(posedge clk);
        cyc = cyc + 1;
        if (urgent) urgent_ever = 1'b1;
    end
    $display("  A. 14ms 后 urgent=%b（中间出现过 urgent=%b）", urgent, urgent_ever);
    if (!urgent_ever) begin $display("   *** FAIL 预灌期间应当 urgent"); fails = fails + 1; end
    if (urgent)       begin $display("   *** FAIL 10ms 后 urgent 必须落 0"); fails = fails + 1; end
    else              $display("   ok  预灌超时后让出总线");

    //------------------------------ B ------------------------------------
    level = 11'd600;                     // >= PREFILL(512)
    repeat (4) @(posedge clk);
    $display("  B. 水位 600 时 urgent=%b", urgent);
    if (urgent) begin $display("   *** FAIL 灌满后不该 urgent"); fails = fails + 1; end
    else        $display("   ok  灌满后不抢总线");

    //------------------------------ C ------------------------------------
    en = 1'b0;
    repeat (4) @(posedge clk);
    $display("  C. I_en=0 时 urgent=%b", urgent);
    if (urgent) begin $display("   *** FAIL 关播还 urgent"); fails = fails + 1; end
    else        $display("   ok  关播不抢总线");

    $display("======================================================");
    if (fails == 0) $display("  ALL PASS");
    else            $display("  FAILED %0d", fails);
    $finish;
end

endmodule
