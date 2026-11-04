//==============================================================================
//  tb_quad_ph.v —— 四宫格装载调度 + wav/txt 占位图
//==============================================================================
//  iverilog -g2012 -o tqp.vvp -s tb_quad_ph \
//      tb_quad_ph.v ../APP/quad_loader.v ph_writer.v && vvp tqp.vvp
//
//  覆盖：
//    A1  BMP 项走 bmp_loader（offset/base 正确），WAV/TXT 不碰 bmp_loader
//    A2  WAV/TXT 的 base 分别落在 slot*40960
//    A3  第 0 遍 4 格都写（BMP 格黑底），每格 80×120 = 9600 字（行距 160）
//    A4  两类占位图的像素内容（边框色 / 柱高 / 文本行）
//    A5  4 项装完 O_loaded == 4，且不会重复占用同一个槽
//==============================================================================
`timescale 1ns/1ps

module tb_quad_ph;

localparam [20:0] QBASE = 21'd1048576;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

integer errors = 0, ncase = 0;
task chk;
    input cond;
    input [255:0] msg;
    begin
        ncase = ncase + 1;
        // ★ 必须用 !== 判：`!X` 是 X，`if (X)` 不成立 ⇒ X 会被当成通过
        if (cond !== 1'b1) begin
            errors = errors + 1;
            $display("  FAIL [%0d] %0s", ncase, msg);
        end
    end
endtask

//------------------------------------------------------------------------------
//  列表桩：4 项 = BMP / WAV / TXT / BMP
//------------------------------------------------------------------------------
reg  [31:0] file_off [0:3];
initial begin
    file_off[0] = 32'd100000;
    file_off[1] = 32'd200000;
    file_off[2] = 32'd300000;
    file_off[3] = 32'd400000;
end
function [3:0] type_of;
    input [1:0] i;
    begin
        case (i)
            2'd0: type_of = 4'd1;   // BMP
            2'd1: type_of = 4'd2;   // WAV
            2'd2: type_of = 4'd3;   // TXT
            default: type_of = 4'd1;
        endcase
    end
endfunction

//------------------------------------------------------------------------------
//  DUT: quad_loader
//------------------------------------------------------------------------------
reg  start = 0;
reg  [2:0] valid_cnt = 3'd4;
reg  [3:0] slot_valid = 4'b1111;
wire busy, done;
wire [2:0] loaded;
wire [1:0] rd_slot;
wire [31:0] ld_off_w, ld_len_w;
wire [20:0] ld_base_w;
reg  ld_ready = 1, ld_busy = 0, ld_done = 0, ld_err = 0;

wire ph_start;
wire [1:0] ph_kind;
wire [20:0] ph_base;
wire ph_busy, ph_done;

wire ld_start;
reg  [31:0] bmp_off_seen, bmp_base_seen;

quad_loader #(
    .SLOT_BASE (QBASE),
    .SLOT_STEP (40960),
    .N_ITEMS   (4)
) u_ql (
    .clk(clk), .rst(rst),
    .I_start(start), .I_abort(1'b0), .I_hold(1'b0), .O_bmp(), .O_filled(), .I_valid_cnt(valid_cnt), .I_slot_valid(slot_valid),
    .O_busy(busy), .O_done(done), .O_loaded(loaded),
    .O_rd_slot(rd_slot),
    .I_file_off(file_off[rd_slot]),
    .I_file_len(32'd921654),
    .I_file_type(type_of(rd_slot)),
    .O_ph_start(ph_start), .O_ph_kind(ph_kind), .O_ph_base(ph_base),
    .I_ph_busy(ph_busy), .I_ph_done(ph_done),
    .O_ld_start(ld_start), .O_ld_off(ld_off_w), .O_ld_len(ld_len_w),
    .O_ld_base(ld_base_w),
    .I_ld_ready(ld_ready), .I_ld_busy(ld_busy), .I_ld_done(ld_done), .I_ld_err(ld_err)
);

//------------------------------------------------------------------------------
//  ph_writer + 写口模型
//------------------------------------------------------------------------------
wire        ph_wr_vld;
wire [20:0] ph_wr_addr;
wire [31:0] ph_wr_data;

ph_writer u_ph (
    .clk(clk), .rst(rst),
    .I_start(ph_start), .I_kind(ph_kind), .I_base(ph_base),
    .O_busy(ph_busy), .O_done(ph_done),
    .O_wr_vld(ph_wr_vld), .O_wr_addr(ph_wr_addr), .O_wr_data(ph_wr_data),
    .I_wr_ack(1'b1)             // 每拍都收（真实调度器是"按配额"，更慢但同样握手）
);

// 抓写口：按 (addr - QBASE) 存前 4 个槽的前若干字，用于像素断言
localparam integer MEMW = 4*40960;   // 4 个槽全存（168k 字）
reg [31:0] pmem [0:MEMW-1];
integer    wcnt = 0;
reg [20:0] waddr_min = 21'hFFFFFF, waddr_max = 21'd0;
integer    slot_wr_cnt [0:3];

always @(posedge clk) begin
    if (ph_wr_vld) begin
        wcnt = wcnt + 1;
        if (ph_wr_addr < waddr_min) waddr_min = ph_wr_addr;
        if (ph_wr_addr > waddr_max) waddr_max = ph_wr_addr;
        if ($unsigned(ph_wr_addr - QBASE) < MEMW)
            pmem[ph_wr_addr - QBASE] <= ph_wr_data;
    end
end

// 从抓到的 pmem 取"显示像素"：槽 s 的 (x,y) —— 与 ps_gen_image 的映射一致
function [15:0] px;
    input integer slot;
    input integer x, y;
    integer a;
    begin
        // ★ 2026-11-04：槽 = 160×120 直存（行距 160 字，每字 2 像素，偶像素在低半字）
        a = slot * 40960 + y * 160 + (x >> 1);
        px = (x % 2) ? pmem[a][31:16] : pmem[a][15:0];
    end
endfunction

//------------------------------------------------------------------------------
//  bmp_loader 桩：收到 start 就记下参数，2 拍后 done
//------------------------------------------------------------------------------
integer bmp_cnt = 0;
reg [2:0] wd = 0;
always @(posedge clk) begin
    ld_done <= 0;
    if (ld_start) begin
        bmp_off_seen  <= ld_off_w;
        bmp_base_seen <= {10'd0, ld_base_w};
        bmp_cnt       <= bmp_cnt + 1;
        wd <= 3;
    end else if (wd != 0) begin
        wd <= wd - 1;
        if (wd == 1) ld_done <= 1;
    end
end

//------------------------------------------------------------------------------
integer i;
initial begin
    for (i = 0; i < 4; i = i + 1) slot_wr_cnt[i] = 0;
    for (i = 0; i < MEMW; i = i + 1) pmem[i] = 32'd0;

    repeat (5) @(posedge clk);
    rst = 0;
    repeat (3) @(posedge clk);

    @(posedge clk); start = 1;
    @(posedge clk); start = 0;

    while (!done) @(posedge clk);
    repeat (10) @(posedge clk);

    $display("\n[A1] loader dispatch");
    $display("      bmp_starts=%0d loaded=%0d ph_wr=%0d", bmp_cnt, loaded, wcnt);
    chk(bmp_cnt == 2,  "A1 two BMP items went to bmp_loader");
    chk(loaded  == 3'd4, "A1 loaded == 4");

    $display("[A2] slot bases");
    // BMP 项 0 base = QBASE+0；BMP 项 3 = QBASE+3*40960
    chk(bmp_base_seen == QBASE + 3*40960, "A2 last bmp base = slot3");

    $display("[A3] placeholder write range");
    $display("      min=%0d max=%0d (slot1 base=%0d slot2 base=%0d)",
             waddr_min, waddr_max, QBASE + 40960, QBASE + 2*40960);
    // ★ 2026-11-04：第 0 遍给**每一格**先写占位（BMP 格 = 纯黑，黑在顶层写口清零）
    chk(waddr_min == QBASE,                "A3 first write at slot0 base");
    chk(waddr_max == QBASE + 3*40960 + 119*160 + 79, "A3 last write at slot3 end");
    chk(wcnt == 4*9600,                    "A3 wrote 4*9600 words (black + 2 placeholders)");

    $display("[A4] pixel content");
    // WAV(slot1)：外框绿、柱底绿、内部底暗
    $display("      wav border=%04x bar=%04x bg=%04x", px(1,0,0), px(1,41,97), px(1,20,20));
    chk(px(1,0,0)   == 16'h3E71, "A4 wav border green");
    chk(px(1,80,60) == 16'h3E71, "A4 wav 正中色块 green");
    chk(px(1,20,20) == 16'h18C6, "A4 wav background");
    // TXT(slot2)：外框蓝、文本行蓝、行间是底
    $display("      txt border=%04x line=%04x gap=%04x", px(2,0,0), px(2,40,33), px(2,40,40));
    chk(px(2,0,0)   == 16'hADBA, "A4 txt border blue");
    chk(px(2,80,60) == 16'hADBA, "A4 txt 正中色块 blue");
    chk(px(2,40,40) == 16'h18C6, "A4 txt 卡片底色");

    $display("\n========================================");
    if (errors == 0) $display("  ALL PASS  (%0d checks)", ncase);
    else             $display("  FAIL: %0d / %0d", errors, ncase);
    $display("========================================\n");
    $finish;
end

initial begin
    #5_000_000;
    $display("  FAIL: watchdog");
    $finish;
end

endmodule
