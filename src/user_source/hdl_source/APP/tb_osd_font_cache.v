//==============================================================================
//  tb_osd_font_cache.v —— osd_font_cache 模块级自检
//==============================================================================
//
//  【这条 tb 要证明什么】（对应计划 §3.3 搬运细节 1~7 与 §7 正确性清单）
//    1. 首次装载：全 miss → 去重后只读 4 次 Flash（A/B/C/D），重复的 A 不重读
//    2. 命中复用：同一字形跨行共享槽（owner 必须是"多行都置位"的位掩码）
//    3. 释放 + 重装：ver 改变触发释放；★ 释放"先清 owner 位、再整体判零"
//       → line1 还引用着的 'A' 不能被回收
//    4. tag 持久性：槽被回收（owner 归零）后字形仍在 BRAM → 再次需要时**不读 Flash**
//    5. active=0 释放：line_ready 拉低；重新 active=1 能再装回来
//    6. 溢出：槽位耗尽 → 该列留空 + line_overflow 粘滞，但 **line_ready 仍要拉高**
//    7. 重装期间 line_ready 拉低；装到一半 ver 又变 → 影子 ver 不得被判成"当前"
//    8. 行内字节组装必须是 {左, 右}（图案左右不对称，写反必报错）
//    9. 槽 0 = 空白槽：读口对槽 0 恒返回 0（字形 RAM 未初始化也不许画花屏）
//   10. rd_req 单拍脉冲、且只在 rd_ready==1 时发
//
//  【为什么用"合成图案"而不是真字库】
//  真字库只看得出"读到了字节"，看不出**地址算错**或**左右字节写反**。
//  这里灌的图案是：
//       offset 2r   （左半 8 列） = base[7:0] + r
//       offset 2r+1 （右半 8 列） = base[7:0] - r
//  于是期望的 16 位行 = {base[7:0]+r, base[7:0]-r}：
//    · 依赖 base → 地址算错就抓到；
//    · 左 != 右（r>0）→ 写成 {右,左} 就抓到；
//    · 逐行不同 → 行序颠倒也抓到。
//  （真字库路径由端到端的 tb_app + check_app_demo.py 负责。）
//
//  【运行】（在 APP 目录下）
//      iverilog -g2005 -o tb_cache.vvp tb_osd_font_cache.v osd_font_cache.v \
//          gb_addr.v ../spi_flash_read.v w25q64_model_24b.v && vvp tb_cache.vvp
//
//==============================================================================

`timescale 1ns/1ps

module tb_osd_font_cache;

//==============================================================================
//  参数（故意取小：NSLOT=8 让"溢出"在几步内就能跑到）
//==============================================================================
localparam integer NL        = 2;
localparam integer MAXLEN    = 8;
localparam integer NSLOT     = 8;
localparam integer MEM_BYTES = 16384;     // 仿真 Flash 容量（用到的地址都 < 0x1000）

// 状态编码（与 DUT 一致，仅用于观察进度）
localparam [3:0] S_IDLE = 4'd0, S_LDCOL = 4'd4;

// 测试用字符码
localparam [15:0] C_A = 16'h0041, C_B = 16'h0042, C_C = 16'h0043, C_D = 16'h0044,
                  C_E = 16'h0045, C_F = 16'h0046, C_G = 16'h0047, C_H = 16'h0048,
                  C_I = 16'h0049, C_J = 16'h004A;

//==============================================================================
//  时钟 / 复位
//==============================================================================
reg clk = 1'b0;
always #5 clk = ~clk;                     // 100 MHz

reg rst = 1'b1;

//==============================================================================
//  DUT 接口线（★ 必须声明在 DUT 例化之前）
//==============================================================================
reg         str_we   = 1'b0;
reg  [3:0]  str_line = 4'd0;
reg  [7:0]  str_idx  = 8'd0;
reg  [15:0] str_code = 16'd0;

reg         ctl_we     = 1'b0;
reg  [3:0]  ctl_line   = 4'd0;
reg         ctl_active = 1'b0;
reg  [7:0]  ctl_len    = 8'd0;
reg  [7:0]  ctl_ver    = 8'd0;

reg         trigger = 1'b0;

reg  [8*NL-1:0]  slot_rd_col = 0;
wire [10*NL-1:0] slot_rd_data;

reg  [13:0] glyph_rd_addr = 14'd0;
wire [15:0] glyph_rd_row;

wire [NL-1:0]   line_ready;
wire [NL-1:0]   line_overflow;
wire [8*NL-1:0] line_len;

wire flash_cs_n, flash_sclk, flash_mosi, flash_wp_n, flash_hold_n, flash_miso;

osd_font_cache #(
    .NL         (NL),
    .MAXLEN     (MAXLEN),
    .NSLOT      (NSLOT),
    .SCLK_DIV   (1),          // SCLK = 100MHz/2 = 50MHz
    .POR_CYCLES (200)         // 仿真只等 200 拍
) u_dut (
    .clk          (clk),
    .rst          (rst),
    .str_we       (str_we),
    .str_line     (str_line),
    .str_idx      (str_idx),
    .str_code     (str_code),
    .ctl_we       (ctl_we),
    .ctl_line     (ctl_line),
    .ctl_active   (ctl_active),
    .ctl_len      (ctl_len),
    .ctl_ver      (ctl_ver),
    .trigger      (trigger),
    .slot_rd_col  (slot_rd_col),
    .slot_rd_data (slot_rd_data),
    .glyph_rd_addr(glyph_rd_addr),
    .glyph_rd_row (glyph_rd_row),
    .line_ready   (line_ready),
    .line_overflow(line_overflow),
    .line_len     (line_len),
    .flash_cs_n   (flash_cs_n),
    .flash_sclk   (flash_sclk),
    .flash_mosi   (flash_mosi),
    .flash_wp_n   (flash_wp_n),
    .flash_hold_n (flash_hold_n),
    .flash_miso   (flash_miso)
);

//==============================================================================
//  Flash 从机模型（24 位寻址版）
//==============================================================================
w25q64_model_24b #(.MEM_BYTES(MEM_BYTES)) u_flash (
    .cs_n (flash_cs_n),
    .sclk (flash_sclk),
    .mosi (flash_mosi),
    .miso (flash_miso)
);

//==============================================================================
//  独立（照文档重写一遍）的"字符码 → Flash 地址"，作为期望值的来源
//==============================================================================
// 故意不复用 DUT 的 gb_addr：这样"地址算错"才会被抓到
function [23:0] exp_addr;
    input [15:0] c;
    reg [7:0]  qb, wb;
    reg [15:0] row;
    begin
        qb = c[15:8];
        wb = c[7:0];
        if (qb == 8'h00) begin
            exp_addr = {16'd0, wb} * 24'd32;                     // ASCII
        end else begin
            if (qb <= 8'hA9) row = {8'd0, qb} - 16'h00A1;        // 符号区
            else             row = {8'd0, qb} - 16'h00B0 + 16'd9;// 汉字区（跳过空区）
            exp_addr = 24'h001000
                     + ({8'd0, row} * 32'd94 + {8'd0, (wb - 8'hA1)}) * 32;
        end
    end
endfunction

// 第 r 行的期望 16 位（{左, 右}）
function [15:0] exp_row;
    input [15:0] c;
    input integer r;
    reg [23:0] b;
    reg [7:0]  rr;
    begin
        b  = exp_addr(c);
        rr = r;
        exp_row = {b[7:0] + rr, b[7:0] - rr};
    end
endfunction

//==============================================================================
//  把图案灌进 Flash 模型
//==============================================================================
task fill_glyph;
    input [15:0] c;
    reg [23:0] b;
    integer    r;
    begin
        b = exp_addr(c);
        for (r = 0; r < 16; r = r + 1) begin
            u_flash.mem[b + 2*r]     = b[7:0] + r[7:0];
            u_flash.mem[b + 2*r + 1] = b[7:0] - r[7:0];
        end
    end
endtask

initial begin
    #1;   // 等模型的 0xFF 初始化跑完
    fill_glyph(C_A);
    fill_glyph(C_B);
    fill_glyph(C_C);
    fill_glyph(C_D);
    fill_glyph(C_E);
    fill_glyph(C_F);
    fill_glyph(C_G);
    fill_glyph(C_H);
    fill_glyph(C_I);
    fill_glyph(C_J);
end

//==============================================================================
//  打分 / 违规计数
//==============================================================================
integer errors   = 0;
integer checks   = 0;
integer rd_tx    = 0;         // Flash 事务数（rd_req 上升沿）
integer rd_pulse = 0;         // rd_req 高电平拍数（单拍脉冲则与 rd_tx 相等）
integer rd_bad   = 0;         // rd_req 未在 rd_ready 时发

reg rd_req_d = 1'b0;

task chk;
    input         cond;
    input [255:0] msg;
    begin
        checks = checks + 1;
        if (cond !== 1'b1) begin
            errors = errors + 1;
            $display("[FAIL] %0t  %0s", $time, msg);
        end
    end
endtask

// rd_req 必须是单拍脉冲，且只在 rd_ready 时发
always @(posedge clk) begin
    rd_req_d <= u_dut.rd_req;
    if (u_dut.rd_req) rd_pulse = rd_pulse + 1;
    if (u_dut.rd_req && !rd_req_d) rd_tx = rd_tx + 1;
    if (u_dut.rd_req && !u_dut.rd_ready) rd_bad = rd_bad + 1;
end

//==============================================================================
//  激励小工具
//==============================================================================
task write_str;
    input [3:0]  ln;
    input [7:0]  idx;
    input [15:0] code;
    begin
        @(negedge clk);
        str_line = ln; str_idx = idx; str_code = code; str_we = 1'b1;
        @(negedge clk);
        str_we = 1'b0;
    end
endtask

task write_ctl;
    input [3:0] ln;
    input       act;
    input [7:0] len;
    input [7:0] ver;
    begin
        @(negedge clk);
        ctl_line = ln; ctl_active = act; ctl_len = len; ctl_ver = ver; ctl_we = 1'b1;
        @(negedge clk);
        ctl_we = 1'b0;
    end
endtask

task pulse_trigger;
    begin
        @(negedge clk);
        trigger = 1'b1;
        @(negedge clk);
        trigger = 1'b0;
    end
endtask

// 等 FSM 回到 IDLE（= 整趟搬运彻底结束）
// ★ 这一步是必须的：只等 line_ready 会"抢跑"——释放阶段刚把 ready 拉低，
//   槽位表/owner 还要再跑 MAXLEN 拍才清完，此时读到的还是旧值。
task wait_idle;
    integer n;
    begin
        n = 0;
        while ((u_dut.st !== S_IDLE) && (n < 400000)) begin @(negedge clk); n = n + 1; end
        chk(u_dut.st === S_IDLE, "TIMEOUT waiting FSM back to IDLE");
    end
endtask

// 发一个 trigger，等整趟跑完，再校验 line_ready
task do_pass;
    input [NL-1:0] mask;
    input          expect_run;
    integer n;
    reg     started;
    begin
        pulse_trigger();
        started = 1'b0;
        n = 0;
        @(negedge clk);
        while ((u_dut.st === S_IDLE) && (n < 4000)) begin @(negedge clk); n = n + 1; end
        if (u_dut.st !== S_IDLE) started = 1'b1;
        if (expect_run) chk(started, "trigger did not start a pass");
        wait_idle();
        chk(line_ready === mask, "line_ready mismatch after pass");
    end
endtask

// 检查某个槽的 16 行是否与 Flash 里的期望完全一致
task chk_slot;
    input [9:0]   slot;
    input [15:0]  code;
    input [255:0] tag;
    reg   [15:0]  got, exp;
    integer r;
    begin
        for (r = 0; r < 16; r = r + 1) begin
            got = u_dut.glyph_mem[slot*16 + r];
            exp = exp_row(code, r);
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("[FAIL] %0t  slot%0d row%0d exp %04h got %04h (%0s)",
                         $time, slot, r, exp, got, tag);
            end
        end
    end
endtask

//==============================================================================
//  看门狗
//==============================================================================
initial begin
    #50_000_000;
    $display("[FAIL] WATCHDOG timeout");
    $display("RESULT: FAIL (%0d checks, %0d errors)", checks, errors + 1);
    $finish;
end

//==============================================================================
//  主流程
//==============================================================================
integer i, n;
reg [15:0] got16;

initial begin
    $display("=== tb_osd_font_cache : NL=%0d MAXLEN=%0d NSLOT=%0d ===", NL, MAXLEN, NSLOT);
    repeat (10) @(posedge clk);
    rst = 1'b0;
    repeat (5) @(posedge clk);

    //================================================================
    // A. reset: 还没配置 -> 什么都不该就绪
    //================================================================
    chk(line_ready    === {NL{1'b0}}, "A: line_ready must be 0 after reset");
    chk(line_overflow === {NL{1'b0}}, "A: line_overflow must be 0 after reset");
    chk(rd_tx == 0,                   "A: no flash transaction after reset");

    //================================================================
    // B. first load: line0="ABCA"(4)  line1="AD"(2)
    //    expect: exactly 4 flash reads (A reused by line0[3] and line1[0])
    //================================================================
    write_str(4'd0, 8'd0, C_A);
    write_str(4'd0, 8'd1, C_B);
    write_str(4'd0, 8'd2, C_C);
    write_str(4'd0, 8'd3, C_A);
    write_str(4'd1, 8'd0, C_A);
    write_str(4'd1, 8'd1, C_D);
    write_ctl(4'd0, 1'b1, 8'd4, 8'd1);
    write_ctl(4'd1, 1'b1, 8'd2, 8'd1);

    // booted==0 时会立即开工，不需要 trigger
    n = 0;
    while ((line_ready !== {NL{1'b1}}) && (n < 400000)) begin @(negedge clk); n = n + 1; end
    chk(line_ready === {NL{1'b1}}, "B: line_ready must be all 1 after first load");
    wait_idle();

    chk(rd_tx == 4,        "B: exactly 4 flash reads (A/B/C/D, A deduped)");
    chk(rd_pulse == rd_tx, "B: rd_req must be single-cycle pulses");
    chk(rd_bad == 0,       "B: rd_req only when rd_ready==1");

    // slot allocation: lowest free slot first -> A=1 B=2 C=3 D=4
    chk(u_dut.owner[1] === 2'b11, "B: slot1(A) owner must be 2'b11 (shared by both lines)");
    chk(u_dut.owner[2] === 2'b01, "B: slot2(B) owner must be 2'b01 (line0 only)");
    chk(u_dut.owner[3] === 2'b01, "B: slot3(C) owner must be 2'b01 (line0 only)");
    chk(u_dut.owner[4] === 2'b10, "B: slot4(D) owner must be 2'b10 (line1 only)");
    chk(u_dut.owner[5] === 2'b00, "B: slot5 must still be free");
    chk(u_dut.tag_hi    === 10'd4, "B: tag_hi must be 4");

    chk(u_dut.g_slotpx[0].slot_px[0] === 10'd1, "B: line0[0] -> slot1");
    chk(u_dut.g_slotpx[0].slot_px[1] === 10'd2, "B: line0[1] -> slot2");
    chk(u_dut.g_slotpx[0].slot_px[2] === 10'd3, "B: line0[2] -> slot3");
    chk(u_dut.g_slotpx[0].slot_px[3] === 10'd1, "B: line0[3] -> slot1 (same glyph as col0)");
    chk(u_dut.g_slotpx[0].slot_px[4] === 10'd0, "B: line0[4] must be blank slot 0");
    chk(u_dut.g_slotpx[1].slot_px[0] === 10'd1, "B: line1[0] -> slot1 (cross-line share)");
    chk(u_dut.g_slotpx[1].slot_px[1] === 10'd4, "B: line1[1] -> slot4");

    chk(line_len[7:0]  === 8'd4, "B: line0 len must be 4");
    chk(line_len[15:8] === 8'd2, "B: line1 len must be 2");

    // glyph content (★ {left,right} assembly + all 16 rows)
    chk_slot(10'd1, C_A, "slot1=A");
    chk_slot(10'd2, C_B, "slot2=B");
    chk_slot(10'd3, C_C, "slot3=C");
    chk_slot(10'd4, C_D, "slot4=D");

    //================================================================
    // C. ver change -> release + reload; 'A' still referenced by line1
    //================================================================
    write_str(4'd0, 8'd0, C_A);
    write_ctl(4'd0, 1'b1, 8'd1, 8'd2);      // line0 becomes 1 char 'A', ver=2
    do_pass({NL{1'b1}}, 1'b1);

    chk(rd_tx == 4,        "C: 'A' still owned by line1 -> no new flash read");
    chk(u_dut.owner[1] === 2'b11, "C: slot1 still referenced by both lines");
    chk(u_dut.owner[2] === 2'b00, "C: slot2(B) must be reclaimed");
    chk(u_dut.owner[3] === 2'b00, "C: slot3(C) must be reclaimed");
    chk(u_dut.owner[4] === 2'b10, "C: slot4(D) belongs to line1, untouched");
    chk(u_dut.g_slotpx[0].slot_px[0] === 10'd1, "C: line0[0] -> slot1 after reload");
    chk(u_dut.g_slotpx[0].slot_px[1] === 10'd0, "C: line0[1] must be cleared");
    chk(line_len[7:0] === 8'd1, "C: line0 len must be 1");

    //================================================================
    // D. line1 deactivates -> release; line_ready[1] drops
    //================================================================
    write_ctl(4'd1, 1'b0, 8'd2, 8'd2);
    do_pass(2'b01, 1'b1);

    chk(u_dut.owner[1] === 2'b01, "D: slot1 owner must keep only line0's bit");
    chk(u_dut.owner[4] === 2'b00, "D: slot4(D) must be reclaimed");
    chk(u_dut.g_slotpx[1].slot_px[0] === 10'd0, "D: line1[0] must be cleared");
    chk(u_dut.g_slotpx[1].slot_px[1] === 10'd0, "D: line1[1] must be cleared");

    //================================================================
    // E. line1 reactivates: ★ tag is persistent -> 'D' hits, no flash read
    //================================================================
    write_ctl(4'd1, 1'b1, 8'd2, 8'd2);
    do_pass({NL{1'b1}}, 1'b1);

    chk(rd_tx == 4, "E: slot4 was reclaimed but glyph is still in BRAM -> 'D' hits tag");
    chk(u_dut.owner[1] === 2'b11, "E: slot1 must be referenced by both lines again");
    chk(u_dut.owner[4] === 2'b10, "E: slot4(D) must belong to line1 again");
    chk(u_dut.g_slotpx[1].slot_px[0] === 10'd1, "E: line1[0] -> slot1");
    chk(u_dut.g_slotpx[1].slot_px[1] === 10'd4, "E: line1[1] -> slot4");

    //================================================================
    // F. fill all 8 slots + overflow
    //    line0 = "ABCDEFGH"(8)  line1 = "IJ"(2) -> I/J find no free slot
    //================================================================
    write_str(4'd0, 8'd0, C_A);
    write_str(4'd0, 8'd1, C_B);
    write_str(4'd0, 8'd2, C_C);
    write_str(4'd0, 8'd3, C_D);
    write_str(4'd0, 8'd4, C_E);
    write_str(4'd0, 8'd5, C_F);
    write_str(4'd0, 8'd6, C_G);
    write_str(4'd0, 8'd7, C_H);
    write_str(4'd1, 8'd0, C_I);
    write_str(4'd1, 8'd1, C_J);
    write_ctl(4'd0, 1'b1, 8'd8, 8'd3);
    write_ctl(4'd1, 1'b1, 8'd2, 8'd3);
    do_pass({NL{1'b1}}, 1'b1);

    // A/B/C/D hit old tags; E/F/G/H are new -> 4 more reads
    chk(rd_tx == 8, "F: 4 hits + 4 new -> 8 flash reads total");
    for (i = 1; i <= 8; i = i + 1)
        chk(u_dut.owner[i] === 2'b01, "F: all 8 slots must be owned by line0 only");
    chk(u_dut.tag_hi === 10'd8, "F: tag_hi must reach 8");

    chk_slot(10'd5, C_E, "slot5=E");
    chk_slot(10'd6, C_F, "slot6=F");
    chk_slot(10'd7, C_G, "slot7=G");
    chk_slot(10'd8, C_H, "slot8=H");

    // ★ overflow: column left blank, but line_ready must still go high
    chk(line_overflow === 2'b10, "F: line1 must raise line_overflow (sticky)");
    chk(u_dut.g_slotpx[1].slot_px[0] === 10'd0, "F: overflowed 'I' must be blank");
    chk(u_dut.g_slotpx[1].slot_px[1] === 10'd0, "F: overflowed 'J' must be blank");
    chk(line_ready === {NL{1'b1}}, "F: overflow must still raise line_ready");

    //================================================================
    // G. line_ready drops during reload + ver bumped mid-load
    //================================================================
    write_ctl(4'd0, 1'b1, 8'd8, 8'd4);      // same text, bump ver -> force rebind
    pulse_trigger();
    n = 0;
    @(negedge clk);
    while ((u_dut.st === S_IDLE) && (n < 4000)) begin @(negedge clk); n = n + 1; end
    chk(u_dut.st !== S_IDLE, "G: trigger must start a pass");

    n = 0;
    while ((line_ready[0] !== 1'b0) && (n < 64)) begin @(negedge clk); n = n + 1; end
    chk(line_ready[0] === 1'b0, "G: line_ready[0] must drop when rebind starts");

    n = 0;
    while (!((u_dut.st === S_LDCOL) && (u_dut.cur_line === 4'd0)) && (n < 20000)) begin
        @(negedge clk); n = n + 1;
    end
    chk(u_dut.st === S_LDCOL,     "G: must reach line0 load phase");
    chk(line_ready[0] === 1'b0,   "G: line_ready[0] must stay 0 during load");

    // ★ bump ver again in the MIDDLE of the load (cur_ver was latched as 4 already)
    write_ctl(4'd0, 1'b1, 8'd8, 8'd5);

    wait_idle();
    chk(line_ready === {NL{1'b1}},        "G: all lines ready after pass1");
    chk(u_dut.line_ver_shad[0] === 8'd4,  "G: shad must be the ver latched at load start (4)");
    chk(u_dut.line_ver_r[0]    === 8'd5,  "G: current ver must be 5");
    chk(rd_tx == 8,                       "G: pass1 all-hit -> no new flash reads");

    // one more trigger -> shad catches up
    do_pass({NL{1'b1}}, 1'b1);
    chk(u_dut.line_ver_shad[0] === 8'd5, "G: shad must catch up to 5 after re-pass");
    chk(rd_tx == 8,                      "G: re-pass still all-hit");

    //================================================================
    // H. blank slot 0 guard + read-port timing
    //================================================================
    // slot 0 always reads back 0 (glyph RAM here was never initialised -> X)
    @(negedge clk); glyph_rd_addr = 14'd0;        // slot0 row0
    @(negedge clk);
    chk(glyph_rd_row === 16'd0, "H: slot 0 read port must always be 0");
    glyph_rd_addr = 14'd0 + 14'd7;                // slot0 row7
    @(negedge clk);
    chk(glyph_rd_row === 16'd0, "H: slot 0 any row must be 0");

    // slot1 row0 / row5: data appears 1 cycle after the address
    glyph_rd_addr = 14'd16;
    @(negedge clk);
    @(negedge clk);
    chk(glyph_rd_row === exp_row(C_A, 0), "H: slot1 row0 must equal flash A row0 (1-cycle)");
    glyph_rd_addr = 14'd16 + 14'd5;
    @(negedge clk);
    @(negedge clk);
    chk(glyph_rd_row === exp_row(C_A, 5), "H: slot1 row5 must equal flash A row5");

    // slot table combinational read
    slot_rd_col = {8'd1, 8'd0};                   // {line1, line0}
    #1;
    chk(slot_rd_data[9:0]   === 10'd1, "H: slot_rd_data[line0=0] must be 1");
    chk(slot_rd_data[19:10] === 10'd0, "H: slot_rd_data[line1=1] must be 0 (overflow blank)");

    // column index out of range (>= MAXLEN) -> must return 0
    slot_rd_col = {8'd0, 8'd8};                   // line0 col 8 is out of range
    #1;
    chk(slot_rd_data[9:0] === 10'd0, "H: out-of-range column must return blank slot 0");
    slot_rd_col = {8'd7, 8'd0};
    #1;
    chk(slot_rd_data[9:0] === 10'd1, "H: line0 col0 must still be 1");

    //================================================================
    // I. wrap up
    //================================================================
    chk(rd_bad == 0,       "I: rd_req must always be issued when rd_ready==1");
    chk(rd_pulse == rd_tx, "I: rd_req must always be a single-cycle pulse");

    $display("--------------------------------------------------------------");
    $display("flash transactions = %0d ; rd_req high cycles = %0d ; rd_req/ready violations = %0d",
             rd_tx, rd_pulse, rd_bad);
    if (errors == 0)
        $display("RESULT: PASS (%0d checks passed)", checks);
    else
        $display("RESULT: FAIL (%0d checks, %0d errors)", checks, errors);
    $finish;
end

endmodule
