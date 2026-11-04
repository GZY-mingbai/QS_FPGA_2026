`timescale 1ns/1ps
//==============================================================================
//  tb_wav_trk.v —— sd_audio_reader 运行时换曲（2026-11-04）
//==============================================================================
//  默认曲目：扇区 10、跳 78 字节、40 个字（编译期参数，旧行为）。
//  换曲到扇区 20：标准 44 字节头（"RIFF…WAVEfmt …data" + 长度 72 = 18 个字），
//    样本字节 = 0,1,2,…；断言换曲后吐出的正好是 18 个字、内容按 {b1,b0,b3,b2} 排，
//    之后 pass_done。再换到扇区 30：78 字节头（data 块前面有 LIST），长度跨扇区也要对。
//  跑法：iverilog -g2012 -o t.vvp -s tb_wav_trk tb_wav_trk.v sd_audio_reader.v && vvp t.vvp
//==============================================================================
module tb_wav_trk;
reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg [7:0] card [0:64*512-1];
integer i, errors = 0;

reg        en = 0, trk_set = 0;
reg        loop_mode = 0;       // ★ 2026-11-08：0=单遍 / 1=循环，用来验“每遍结束”口径
reg [31:0] trk_sec = 0;
wire       sd_req;
wire [31:0] sd_addr;
reg  [7:0] sd_data = 0;
reg        sd_dv = 0, sd_end = 0;
wire       we;
wire [31:0] wd;
wire       pdone;
wire       pass_end, pass_tick;

integer tick_cnt = 0, pend_cnt = 0;
always @(posedge clk) begin
    if (pass_tick) tick_cnt = tick_cnt + 1;
    if (pass_end)  pend_cnt = pend_cnt + 1;
end

sd_audio_reader #(.BASE_SECTOR(10), .SKIP_BYTES(78), .TOTAL_WORDS(40), .FIFO_AW(10),
                  .PREFILL(4), .REQ_LEVEL(768), .URGENT_LEVEL(256)) dut (
    .I_clk(clk), .I_rst(rst), .I_en(en), .I_loop(loop_mode),
    .I_trk_set(trk_set), .I_trk_sec(trk_sec),
    .O_sd_req(sd_req), .O_sd_addr(sd_addr), .O_sd_urgent(),
    .I_sd_data(sd_data), .I_sd_dv(sd_dv), .I_sd_end(sd_end),
    .I_fifo_level(11'd0), .I_fifo_full(1'b0),
    .O_fifo_we(we), .O_fifo_data(wd), .O_drop(), .O_pass_done(pdone),
    .O_pass_end(pass_end), .O_pass_tick(pass_tick)
);

// 扇区模型：req 上升后稍等，连发 512 字节 + end
integer b; reg busy = 0; reg [31:0] cur; reg rel = 1;   // rel：req 落过一次才接下一单（同真仲裁器）
always @(posedge clk) begin
    sd_dv <= 0; sd_end <= 0;
    if (!sd_req) rel <= 1;
    if (!busy && sd_req && rel) begin busy <= 1; rel <= 0; cur <= sd_addr; b <= -5; end
    else if (busy) begin
        if (b >= 0 && b < 512) begin sd_dv <= 1; sd_data <= card[cur*512 + b]; end
        if (b == 512) begin sd_end <= 1; busy <= 0; end
        b <= b + 1;
    end
end

// 收到的字
reg [31:0] got [0:255];
integer ng = 0;
always @(posedge clk) if (we) begin got[ng] = wd; ng = ng + 1; end

task put32; input integer a; input [31:0] v;
    begin card[a] = v[7:0]; card[a+1] = v[15:8]; card[a+2] = v[23:16]; card[a+3] = v[31:24]; end
endtask
task put4c; input integer a; input [31:0] v;    // 4 个 ASCII，按书写顺序
    begin card[a] = v[31:24]; card[a+1] = v[23:16]; card[a+2] = v[15:8]; card[a+3] = v[7:0]; end
endtask

// 期望：样本字节序列 s0,s1,... ⇒ 第 k 个字 = {s(4k+1), s(4k), s(4k+3), s(4k+2)}
task chk_words;
    input integer n; input integer first;   // first = 第 0 个样本字节的值
    input [8*16-1:0] tag;
    integer k; reg [31:0] e;
    begin
        if (ng != n) begin errors = errors + 1; $display("  FAIL %0s: %0d words, want %0d", tag, ng, n); end
        for (k = 0; k < n && k < ng; k = k + 1) begin
            e = {8'(first+4*k+1), 8'(first+4*k), 8'(first+4*k+3), 8'(first+4*k+2)};
            if (got[k] !== e) begin
                errors = errors + 1;
                if (errors < 10) $display("  FAIL %0s word %0d = %h want %h", tag, k, got[k], e);
            end
        end
        if (!pdone) begin errors = errors + 1; $display("  FAIL %0s: pass_done not set", tag); end
        $display("  %0s: %0d words", tag, ng);
    end
endtask

initial begin
    for (i = 0; i < 64*512; i = i + 1) card[i] = 8'hEE;
    // 默认曲目：扇区 10，78 字节头（垃圾），然后样本 0x10,0x11,...
    for (i = 0; i < 400; i = i + 1) card[10*512 + 78 + i] = 8'h10 + i;
    // 曲目 A：扇区 20，44 字节标准头，data 长度 72
    put4c(20*512 + 0, "RIFF"); put32(20*512 + 4, 32'd108); put4c(20*512 + 8, "WAVE");
    put4c(20*512 + 12, "fmt "); put32(20*512 + 16, 32'd16);
    put4c(20*512 + 36, "data"); put32(20*512 + 40, 32'd72);
    for (i = 0; i < 200; i = i + 1) card[20*512 + 44 + i] = 8'h40 + i;
    // 曲目 B：扇区 30，"data" 放在 506 处 ⇒ 长度 4 字节跨到扇区 31 开头
    put4c(30*512 + 0, "RIFF"); put4c(30*512 + 8, "WAVE");
    put4c(30*512 + 70, "LIST");
    put4c(30*512 + 506, "data");
    card[30*512 + 510] = 8'd40; card[30*512 + 511] = 8'd0;   // 长度 40 = 10 个字（跨扇区）
    card[31*512 + 0] = 8'd0;    card[31*512 + 1] = 8'd0;
    for (i = 0; i < 200; i = i + 1) card[31*512 + 2 + i] = 8'h80 + i;

    repeat (5) @(posedge clk); rst = 0;
    $display("--- default track ---");
    en = 1;
    repeat (60000) @(posedge clk);
    chk_words(40, 8'h10, "default");

    $display("--- switch to track A (44-byte header) ---");
    ng = 0;
    @(negedge clk); trk_sec = 20; trk_set = 1; @(negedge clk); trk_set = 0;
    repeat (60000) @(posedge clk);
    chk_words(18, 8'h40, "trackA");

    $display("--- switch to track B (data chunk straddles a sector) ---");
    ng = 0;
    @(negedge clk); trk_sec = 30; trk_set = 1; @(negedge clk); trk_set = 0;
    repeat (60000) @(posedge clk);
    chk_words(10, 8'h80, "trackB");

    //--------------------------------------------------------------------------
    //  ★ 2026-11-08：每遍结束口径（自动切换=音乐结束 的事件源）
    //    L1 loop=1：每回卷一次就一个 tick，且绝不出 pass_end（“整首播完”）
    //    L2 loop=0：整首播完时 tick / pass_end 各恰好一个
    //--------------------------------------------------------------------------
    $display("--- L1 loop=1: tick per pass, no pass_end ---");
    loop_mode = 1;
    tick_cnt = 0; pend_cnt = 0;
    ng = 0;
    @(negedge clk); trk_sec = 20; trk_set = 1; @(negedge clk); trk_set = 0;
    repeat (20000) @(posedge clk);
    $display("    ticks=%0d pass_end=%0d", tick_cnt, pend_cnt);
    if (tick_cnt < 2) begin
        errors = errors + 1;
        $display("  FAIL L1: loop=1 应该每遍一个 tick（>=2），实际 %0d", tick_cnt);
    end
    if (pend_cnt != 0) begin
        errors = errors + 1;
        $display("  FAIL L1b: loop=1 不应有 pass_end，实际 %0d", pend_cnt);
    end

    $display("--- L2 loop=0: exactly one tick + one pass_end ---");
    loop_mode = 0;
    tick_cnt = 0; pend_cnt = 0;
    ng = 0;
    @(negedge clk); trk_sec = 20; trk_set = 1; @(negedge clk); trk_set = 0;
    repeat (60000) @(posedge clk);
    $display("    ticks=%0d pass_end=%0d", tick_cnt, pend_cnt);
    if (tick_cnt != 1) begin
        errors = errors + 1;
        $display("  FAIL L2: loop=0 应该恰好 1 个 tick，实际 %0d", tick_cnt);
    end
    if (pend_cnt != 1) begin
        errors = errors + 1;
        $display("  FAIL L2b: loop=0 应该恰好 1 个 pass_end，实际 %0d", pend_cnt);
    end

    if (errors == 0) $display("ALL PASS"); else $display("FAIL: %0d", errors);
    $finish;
end
endmodule
