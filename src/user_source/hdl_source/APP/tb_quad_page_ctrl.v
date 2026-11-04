//==============================================================================
//  tb_quad_page_ctrl.v —— 文件页时序控制单元测试
//==============================================================================
//  iverilog -g2012 -o tqpc.vvp -s tb_quad_page_ctrl tb_quad_page_ctrl.v quad_page_ctrl.v
//  vvp tqpc.vvp
//
//  用一个"听话的假 lister / 假 loader"当被控对象：
//    · lister 收到 start 后 5 拍回 done，cnt=4（或按用例设 0）、has_more 可控
//    · loader  收到 start 后 8 拍回 done，loaded=4（或按用例设 0）
//  这样才能把"进入 / 翻页 / 退出 / 空页 / 全失败"几条路都走一遍。
//==============================================================================
`timescale 1ns/1ps

module tb_quad_page_ctrl;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg  file_page = 0, flip_next = 0, flip_prev = 0;
wire lst_start;
wire [15:0] lst_offset;
reg  lst_busy = 0, lst_done = 0, lst_has_more = 1;
reg  [2:0] lst_cnt = 3'd4;
wire ld_start;
reg  ld_busy = 0, ld_done = 0;
reg  [2:0] ld_loaded = 3'd4;
wire own, ready, has_more;
wire [15:0] offset;
wire [2:0] vcnt;

quad_page_ctrl #(.N_ITEMS(4), .STUCK_CYC(40)) dut (
    .clk(clk), .rst(rst),
    .I_file_page(file_page), .I_flip_next(flip_next), .I_flip_prev(flip_prev),
    .O_lst_start(lst_start), .O_lst_offset(lst_offset),
    .I_lst_busy(lst_busy), .I_lst_done(lst_done),
    .I_lst_cnt(lst_cnt), .I_lst_has_more(lst_has_more),
    .O_ld_start(ld_start), .I_ld_busy(ld_busy), .I_ld_done(ld_done),
    .I_ld_loaded(ld_loaded),
    // ★ 2026-11-02：两遍装载之后 ready 由 I_ld_filled（第 0 遍写完）驱动，
    //   这个台原来没接 ⇒ ready 恒 0（T1c/T2b/T7b 一直红）。这里用 done 代替。
    .I_ld_filled(ld_done & (ld_loaded != 3'd0)), .I_ld_bmp(4'b1111), .I_cur(2'd0),
    .O_own(own), .O_ready(ready), .O_has_more(has_more),
    .O_offset(offset), .O_valid_cnt(vcnt)
);

//------------------------------------------------------------------------------
//  假 lister
//------------------------------------------------------------------------------
integer lst_t = 0;
reg [7:0] lst_len = 8'd3;                       // 扫描时长（T12 要拉长才好"在扫描中"按键）
always @(posedge clk) begin
    lst_done <= 0;
    if (rst) begin lst_busy <= 0; lst_t <= 0; end
    else if (lst_start) begin
        lst_busy <= 1; lst_t <= lst_len;        // 默认 3 拍后 done
    end else if (lst_busy) begin
        if (lst_t != 0) lst_t <= lst_t - 1;
        else begin lst_busy <= 0; lst_done <= 1; end
    end
end

//------------------------------------------------------------------------------
//  假 loader
//------------------------------------------------------------------------------
integer ld_t = 0;
reg     wedged = 0;             // 1 = 永远不回 done（模拟读卡卡死）
always @(posedge clk) begin
    ld_done <= 0;
    if (rst) begin ld_busy <= 0; ld_t <= 0; end
    else if (ld_start) begin
        ld_busy <= 1; ld_t <= 5;
    end else if (ld_busy) begin
        if (wedged) begin
            // 保持 busy，永不 done
        end else if (ld_t != 0) ld_t <= ld_t - 1;
        else begin ld_busy <= 0; ld_done <= 1; end
    end
end

//------------------------------------------------------------------------------
integer errors = 0, ncase = 0;
task chk;
    input cond;
    input [255:0] msg;
    begin
        ncase = ncase + 1;
        if (!cond) begin errors = errors + 1; $display("  FAIL [%0d] %0s", ncase, msg); end
    end
endtask

// 等 n 拍后检查（简单的"给足时间"）
task idle_cycles;
    input integer n;
    integer i;
    begin for (i = 0; i < n; i = i + 1) @(posedge clk); end
endtask

initial begin
    repeat (5) @(posedge clk);
    rst = 0;
    repeat (3) @(posedge clk);

    //------------------------------------------------------------------
    $display("\n[T1] enter file page -> scan -> load -> ready");
    chk(own === 1'b0 && ready === 1'b0, "T1a idle: own/ready = 0");
    file_page = 1;
    idle_cycles(30);
    chk(own   === 1'b1, "T1b own taken");
    chk(ready === 1'b1, "T1c ready after load");
    chk(offset == 16'd0, "T1d offset = 0");
    chk(vcnt  == 3'd4,   "T1e valid_cnt = 4");

    //------------------------------------------------------------------
    $display("[T2] flip next -> offset += 4");
    @(posedge clk); flip_next = 1;
    @(posedge clk); flip_next = 0;
    idle_cycles(30);
    chk(offset == 16'd4, "T2a offset = 4");
    chk(ready  === 1'b1,  "T2b ready again");
    chk(own    === 1'b1,  "T2c own still held");

    //------------------------------------------------------------------
    $display("[T3] flip prev -> offset -= 4");
    @(posedge clk); flip_prev = 1;
    @(posedge clk); flip_prev = 0;
    idle_cycles(30);
    chk(offset == 16'd0, "T3a offset back to 0");

    //------------------------------------------------------------------
    $display("[T4] flip prev at offset 0 -> must be ignored");
    @(posedge clk); flip_prev = 1;
    @(posedge clk); flip_prev = 0;
    idle_cycles(20);
    chk(offset == 16'd0, "T4a offset stays 0");

    //------------------------------------------------------------------
    $display("[T5] no more pages -> flip next ignored");
    // has_more 是**每次扫描时锁存**的，所以先让 lst_has_more=0 再翻一次，
    // 这一次扫描会把 has_more_r 采成 0；之后再翻就必须被忽略。
    lst_has_more = 0;
    @(posedge clk); flip_next = 1;
    @(posedge clk); flip_next = 0;
    idle_cycles(30);
    chk(offset == 16'd4, "T5a offset advanced to 4");
    chk(has_more === 1'b0, "T5b has_more captured 0");
    @(posedge clk); flip_next = 1;
    @(posedge clk); flip_next = 0;
    idle_cycles(20);
    chk(offset == 16'd4, "T5c offset stays 4 (no more)");
    lst_has_more = 1;
    // 回到第 0 页给后面的用例一个干净的起点
    @(posedge clk); flip_prev = 1;
    @(posedge clk); flip_prev = 0;
    idle_cycles(30);

    //------------------------------------------------------------------
    $display("[T6] leave page -> own released");
    file_page = 0;
    idle_cycles(20);
    chk(own   === 1'b0, "T6a own released");
    chk(ready === 1'b0, "T6b ready cleared");

    //------------------------------------------------------------------
    $display("[T7] re-enter -> reload same offset");
    file_page = 1;
    idle_cycles(30);
    chk(own   === 1'b1, "T7a own re-taken");
    chk(ready === 1'b1, "T7b ready again");

    //------------------------------------------------------------------
    $display("[T8] all loads fail -> ready must stay 0 (show nothing)");
    file_page = 0;  idle_cycles(20);
    ld_loaded = 3'd0;
    file_page = 1;  idle_cycles(30);
    chk(ready === 1'b0, "T8a ready = 0 when nothing loaded");
    chk(own   === 1'b1, "T8b still owns loader");

    //------------------------------------------------------------------
    $display("[T9] empty page (cnt = 0) -> no load, no ready");
    file_page = 0;  idle_cycles(20);
    ld_loaded = 3'd4;
    lst_cnt   = 3'd0;
    file_page = 1;  idle_cycles(30);
    chk(ready === 1'b0, "T9a ready = 0 on empty page");
    chk(ld_start === 1'b0, "T9b no load issued");
    lst_cnt = 3'd4;

    //------------------------------------------------------------------
    $display("[T10] leave during scan -> no load, own released");
    file_page = 0;  idle_cycles(20);
    file_page = 1;
    idle_cycles(1);                 // 刚进 S_SCAN
    file_page = 0;                  // 立刻离开
    idle_cycles(30);
    chk(own === 1'b0, "T10a own released after scan");
    // 注：lister 会自然扫完，装载被跳过

    //------------------------------------------------------------------
    $display("[T11] loader wedged + leave page -> force release");
    file_page = 0;  idle_cycles(20);
    file_page = 1;  idle_cycles(30);
    chk(own === 1'b1, "T11a own held");
    // 把假 loader 粘住：不再回 done
    wedged = 1'b1;
    file_page = 0;
    idle_cycles(120);          // > STUCK_CYC(40)
    chk(own === 1'b0, "T11b own force-released after timeout");
    wedged = 1'b0;

    //------------------------------------------------------------------
    $display("[T12] flip during scan -> key not dropped (SCAN-FLIP)");
    file_page = 0;  idle_cycles(20);
    file_page = 1;  idle_cycles(40);
    chk(offset == 16'd0, "T12a start from offset 0");
    lst_len = 8'd12;                // 拉长扫描，留出按键窗口
    @(posedge clk); flip_next = 1;  // 0 -> 4，进 S_SCAN
    @(posedge clk); flip_next = 0;
    idle_cycles(4);                 // 扫描进行中
    @(posedge clk); flip_next = 1;  // 4 -> 8，本次扫描作废
    @(posedge clk); flip_next = 0;
    idle_cycles(80);
    $display("       T12 dump: offset=%0d ready=%b own=%b st=%0d rescan=%b has_more=%b",
             offset, ready, own, dut.st, dut.rescan, has_more);
    chk(offset == 16'd8, "T12b flip during scan not dropped");
    chk(ready  === 1'b1, "T12c 重扫后 ready 恢复");
    chk(own    === 1'b1, "T12d own 仍在");
    lst_len = 8'd3;

    $display("\n========================================");
    if (errors == 0) $display("  ALL PASS  (%0d checks)", ncase);
    else             $display("  FAIL: %0d / %0d", errors, ncase);
    $display("========================================\n");
    $finish;
end

initial begin
    #2_000_000;
    $display("  TIMEOUT");
    $finish;
end

endmodule
