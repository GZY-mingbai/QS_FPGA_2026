//=============================================================================
//  tb_spec_fft.v —— spec_fft 定点链 / 分带 / 刻度回归
//=============================================================================
//  覆盖点（对应 2026-11-03 的标定修正）：
//    1. 静音            ⇒ 16 条全 0
//    2. 300 Hz −12dBFS  ⇒ 亮段 1..3（Hann 主瓣 4 bin），段 15 必须为 0
//    3. 1 kHz  −12dBFS  ⇒ argmax = 8，且 **不削顶**（旧版每段偏置会顶到 127）
//    4. 3 kHz  −12dBFS  ⇒ argmax = 13
//    5. 5 kHz  −12dBFS  ⇒ argmax = 15，段 0 必须为 0
//    6. 1 kHz  −6dBFS   ⇒ argmax = 8，h ∈ [110,126]（顶格附近但不满）
//
//  运行：bash tools/run_all_tb.sh spec      （run_all_tb.sh 已自动挂 mult18_stub.v）
//  手工：iverilog -g2012 -o tb_spec_fft.vvp -s tb_spec_fft tb_spec_fft.v \
//            -y . ../APP/OSD/tools/mult18_stub.v && vvp tb_spec_fft.vvp
//
//  时序：25 MHz 时钟；48.077 kHz 采样节拍（520 拍）；60 Hz vbl（416667 拍）。
//        spec_fft 每帧采 128 个 12 kHz 样本（=512 个 48 kHz 节拍 ≈ 10.65 ms），
//        下一个 vbl 才跑 FFT+BAND，所以一个用例等 1 个 vbl + 7000 拍再读条高。
//=============================================================================
`timescale 1ns/1ps

module tb_spec_fft;

localparam integer SMPDIV = 520;          // 25e6 / 520 = 48076.9 Hz
localparam integer FRAME  = 416667;       // 25e6 / 60
localparam real    PI2    = 6.283185307179586;

reg clk = 1'b0;
always #20 clk = ~clk;                    // 25 MHz

reg        rst       = 1'b1;
reg        smp_valid = 1'b0;
reg [23:0] smp       = 24'd0;
reg        vbl       = 1'b0;
reg [3:0]  idx       = 4'd0;
wire [6:0] h;

spec_fft dut (
    .clk       (clk),
    .rst       (rst),
    .smp_valid (smp_valid),
    .smp       (smp),
    .vbl       (vbl),
    .scan_idx  (idx),
    .scan_h    (h)
);

real    freq  = 1000.0;
real    amp   = 0.0;
real    phase = 0.0;
real    inc   = 0.0;
integer smp_cnt   = 0;
integer frame_cnt = 0;
integer errors    = 0;
integer i, k, mx, wi;

reg [6:0] hh [0:15];

// wmem 是 256×32 的 BRAM，仿真里未初始化＝X；板上的 BRAM 上电为 0。
// 这里只给仿真把工作区清 0（含 128..143 的条高区，否则 d_old=X 会自锁）。
initial begin
    for (wi = 0; wi < 256; wi = wi + 1) dut.wmem[wi] = 32'd0;
end

always @* inc = PI2 * freq / 48076.923;

always @(posedge clk) begin
    if (rst) begin
        smp_cnt   <= 0;
        frame_cnt <= 0;
        smp_valid <= 1'b0;
        vbl       <= 1'b0;
        smp       <= 24'd0;
        phase     <= 0.0;
    end else begin
        // ---- 48 kHz 采样节拍（24bit 左对齐，和 wav_player 的输出格式一致）----
        smp_valid <= 1'b0;
        if (smp_cnt == SMPDIV-1) begin
            smp_cnt   <= 0;
            smp_valid <= 1'b1;
            smp       <= ($rtoi(amp * 32767.0 * $sin(phase)) <<< 8);
            phase     <= phase + inc;
            if (phase + inc >= PI2) phase <= phase + inc - PI2;
        end else
            smp_cnt <= smp_cnt + 1;

        // ---- 60 Hz vbl（单拍）----
        if (frame_cnt == FRAME-1) begin
            frame_cnt <= 0;
            vbl       <= 1'b1;
        end else begin
            frame_cnt <= frame_cnt + 1;
            vbl       <= 1'b0;
        end
    end
end

//-----------------------------------------------------------------------------
//  激励/检查任务
//-----------------------------------------------------------------------------
task begin_case(input real f, input real a);
    begin
        @(negedge clk);
        rst  = 1'b1;
        freq = f;
        amp  = a;
        repeat (10) @(negedge clk);
        // 条高区（128..143）不随 rst 清零；仿真里手工清，避免上一个用例的
        // 峰值保持残留把 argmax 带偏（板上 BRAM 上电为 0，且每帧都会写）。
        for (wi = 128; wi < 144; wi = wi + 1) dut.wmem[wi] = 32'd0;
        rst  = 1'b0;
    end
endtask

task wait_frame;
    begin
        @(posedge vbl);
        repeat (7000) @(posedge clk);      // FFT(3136) + BAND(≤2300) 之后回到 S_CAP
    end
endtask

task read_h;
    begin
        for (i = 0; i < 16; i = i + 1) begin
            @(negedge clk);
            idx = i[3:0];
            @(posedge clk);                // 同步读：地址打一拍
            @(negedge clk);                // 等 NBA 落定再采
            hh[i] = h;
        end
        @(negedge clk);
        idx = 4'd0;
    end
endtask

task fail(input [8*72-1:0] msg);
    begin
        errors = errors + 1;
        $display("FAIL: %0s", msg);
    end
endtask

task dump_h(input [8*12-1:0] name);
    begin
        $write("  %0s :", name);
        for (k = 0; k < 16; k = k + 1) $write("%4d", hh[k]);
        $write("\n");
    end
endtask

function integer argmax;
    integer j, best;
    begin
        best = 0;
        for (j = 1; j < 16; j = j + 1)
            if (hh[j] > hh[best]) best = j;
        argmax = best;
    end
endfunction

//-----------------------------------------------------------------------------
//  用例
//-----------------------------------------------------------------------------
initial begin
    $display("== tb_spec_fft ==");

    // ---- 1. 静音 ----
    begin_case(1000.0, 0.0);
    wait_frame; read_h; dump_h("silence");
    for (i = 0; i < 16; i = i + 1)
        if (hh[i] !== 7'd0) fail("silence: bars must all be 0");

    // ---- 2. 300 Hz −12 dBFS ----
    begin_case(300.0, 0.2512);
    wait_frame; read_h; dump_h("300Hz");
    k = argmax();
    if (!(k >= 1 && k <= 3)) fail("300Hz: argmax not in 1..3");
    if (hh[k] < 60)         fail("300Hz: peak too low");
    if (hh[15] > 40)        fail("300Hz: leaks into band 15");

    // ---- 3. 1 kHz −12 dBFS：新刻度必须不削顶 ----
    begin_case(1000.0, 0.2512);
    wait_frame; read_h; dump_h("1kHz -12");
    k = argmax();
    if (k != 8)             fail("1kHz: argmax != 8");
    if (hh[8] < 90)         fail("1kHz: peak too low");
    if (hh[8] >= 127)       fail("1kHz -12dB clips => per-band offset still there?");

    // ---- 4. 3 kHz −12 dBFS ----
    begin_case(3000.0, 0.2512);
    wait_frame; read_h; dump_h("3kHz -12");
    k = argmax();
    if (k != 13)            fail("3kHz: argmax != 13");
    if (hh[13] < 80)        fail("3kHz: peak too low");

    // ---- 5. 5 kHz −12 dBFS ----
    begin_case(5000.0, 0.2512);
    wait_frame; read_h; dump_h("5kHz -12");
    k = argmax();
    if (k != 15)            fail("5kHz: argmax != 15");
    if (hh[15] < 80)        fail("5kHz: peak too low");
    if (hh[0] > 40)         fail("5kHz: leaks into band 0");

    // ---- 6. 1 kHz −6 dBFS：应接近满格但不该削顶（旧版=127）----
    begin_case(1000.0, 0.5012);
    wait_frame; read_h; dump_h("1kHz -6");
    k = argmax();
    if (k != 8)             fail("1kHz -6dB: argmax != 8");
    if (hh[8] < 110)        fail("1kHz -6dB: peak too low");
    if (hh[8] > 126)        fail("1kHz -6dB: clips at 127");

    // ---- 汇总 ----
    mx = 0;
    for (i = 0; i < 16; i = i + 1) if (hh[i] > mx) mx = hh[i];
    if (mx > 127) fail("any bar > 127");

    if (errors == 0) $display("ALL PASS tb_spec_fft");
    else             $display("FAILED tb_spec_fft (%0d errors)", errors);
    $finish;
end

endmodule
