`timescale 1ns/1ps
//==============================================================================
//  tb_zoom_osd.v —— 缩放 OSD 按键状态机断言台
//==============================================================================
//  zoom_osd 是纯控制逻辑（无像素通路），用断言就能覆盖，不用出图。
//
//  运行：
//    iverilog -g2012 -o /tmp/tz.vvp -s tb_zoom_osd tb_zoom_osd.v zoom_osd.v
//    vvp /tmp/tz.vvp
//
//  覆盖（2026-10-30 版，500ms 防抖已去掉）：
//    1  打开 / 切档
//    2  逐档放大（每档一次装载，按下即装）
//    3  >1.0 禁止切到 X-PAN / Y-PAN
//    4  逐档缩小回 1.0
//    5  1.0 时 KEY2 可轮转、锚点可动
//    6  退出 / 重开保留状态
//    7  装载期间按键不被丢掉（记一步，装完立刻执行）
//    8  退出后按 KEY1 ⇒ 完整 565 动画恢复 1.0× ⇒ 再放行动作
//==============================================================================
module tb_zoom_osd;

reg         clk = 1'b0;
reg         rst = 1'b1;
reg  [3:0]  key_press = 4'd0;       // {KEY4,KEY3,KEY2,KEY1}
reg         menu_on   = 1'b0;
reg         allow     = 1'b1;
reg         ld_done   = 1'b0;
reg  [1:0]  act_code  = 2'd0;       // 1=上页 2=下页 3=打开OSD
reg         hd_shown  = 1'b1;       // ★ [ZOOM-SEQ] 888 是否已在显示

wire        zoom_on;
wire [7:0]  zoom;
wire [9:0]  ax, ay;
wire [1:0]  mode;
wire        refresh, preview_req, load_req;
wire        act_go;
wire [1:0]  act_sel;
wire        restore;

always #10 clk = ~clk;              // 50MHz

integer fails  = 0;
integer checks = 0;
integer c, cc;

zoom_osd #(
    .CLK_HZ (50_000_000),
    .DB_MS  (1)                     // 1ms（只影响装载超时）
) dut (
    .clk        (clk),
    .rst        (rst),
    .key_press  (key_press),
    .menu_on    (menu_on),
    .allow      (allow),
    .ld_done    (ld_done),
    .hd_shown   (hd_shown),
    .act_code   (act_code),
    .zoom_on    (zoom_on),
    .zoom       (zoom),
    .ax         (ax),
    .ay         (ay),
    .mode       (mode),
    .refresh    (refresh),
    .preview_req(preview_req),
    .load_req   (load_req),
    .act_go     (act_go),
    .act_sel    (act_sel),
    .restore    (restore)
);

//------------------------------------------------------------------------------
//  工具
//------------------------------------------------------------------------------
task chk;
    input         cond;
    input [511:0] name;
    begin
        checks = checks + 1;
        if (!cond) begin
            fails = fails + 1;
            $display("  FAIL  %0s", name);
        end else
            $display("  ok    %0s", name);
    end
endtask

// 单拍按键
task press;
    input [3:0] k;
    begin
        @(negedge clk);
        key_press = k;
        @(negedge clk);
        key_press = 4'd0;
    end
endtask

// 数 N 拍，统计 load_req 次数（which==2）
task count_pulse;
    input integer n;
    input integer which;
    output integer cnt;
    begin
        cnt = 0;
        repeat (n) begin
            @(negedge clk);
            if ((which == 2) && load_req) cnt = cnt + 1;
        end
    end
endtask

// 把"当前这次装载"收尾：在 Z_LOAD 就喂 ld_done，直到回到 Z_HD
task settle_load;
    input integer to;
    integer m;
    begin
        m = 0;
        while ((dut.st != 2'd1) && (m < to)) begin
            if (dut.st == 2'd3) begin
                @(negedge clk); ld_done = 1'b1;
                @(negedge clk); ld_done = 1'b0;
            end
            @(negedge clk); m = m + 1;
        end
    end
endtask

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
initial begin
    repeat (4) @(negedge clk);
    rst = 1'b0;
    repeat (4) @(negedge clk);

    //==================================================================
    $display("--- 1. 打开 / 切档 ---");
    //==================================================================
    press(4'b0010);                        // KEY2 打开
    repeat (4) @(negedge clk);
    chk(zoom_on,       "T1 KEY2 opens zoom OSD");
    chk(zoom == 8'h10, "T2 starts at 1.0x");
    chk(mode == 2'd0,  "T3 mode starts at X-PAN");
    press(4'b0010); press(4'b0010);        // 切到 ZOOM
    repeat (4) @(negedge clk);
    chk(mode == 2'd2,  "T4 mode = ZOOM");

    //==================================================================
    $display("--- 2. 逐档放大（按下即装，无 500ms 防抖）---");
    //==================================================================
    press(4'b0100); settle_load(20000); chk(zoom == 8'h20, "T5 zoom 2.0x");
    press(4'b0100); settle_load(20000); chk(zoom == 8'h40, "T6 zoom 4.0x");
    press(4'b0100); settle_load(20000); chk(zoom == 8'h80, "T7 zoom 8.0x");
    press(4'b0100); settle_load(20000); chk(zoom == 8'h80, "T8 saturates at 8.0x");

    //==================================================================
    $display("--- 3. 缩放>1.0 时禁止切到 X-PAN / Y-PAN ---");
    //==================================================================
    press(4'b0010); repeat (6) @(negedge clk);
    chk(mode == 2'd2,  "T9 KEY2 locked to ZOOM at 8.0x");
    chk(ax == 10'd320, "T10 ax frozen at >1.0");

    //==================================================================
    $display("--- 4. 逐档缩小回 1.0 ---");
    //==================================================================
    press(4'b1000); settle_load(20000); chk(zoom == 8'h40, "T11 back to 4.0x");
    press(4'b1000); settle_load(20000); chk(zoom == 8'h20, "T12 back to 2.0x");
    press(4'b1000); settle_load(20000); chk(zoom == 8'h10, "T13 back to 1.0x");

    //==================================================================
    $display("--- 5. 1.0 时 KEY2 可轮转、锚点可动 ---");
    //==================================================================
    press(4'b0010); repeat (6) @(negedge clk);
    chk(mode == 2'd0,  "T14 KEY2 rotates again at 1.0 (ZOOM -> X-PAN)");
    press(4'b0100); repeat (6) @(negedge clk);
    chk(ax == 10'd328, "T15 KEY3 pans ax right at 1.0");
    press(4'b1000); repeat (6) @(negedge clk);
    chk(ax == 10'd320, "T16 KEY4 pans ax left at 1.0");

    //==================================================================
    $display("--- 6. 退出 / 重开保留状态 ---");
    //==================================================================
    press(4'b0001); repeat (6) @(negedge clk);
    chk(!zoom_on, "T17 KEY1 exits");
    press(4'b0010); repeat (6) @(negedge clk);
    chk(zoom_on && (zoom == 8'h10) && (mode == 2'd0), "T18 reopen keeps 1.0x / X-PAN");

    // 放大到 2× 再退出重开 ⇒ 必须还是 2.0x ZOOM
    press(4'b0010); press(4'b0010); repeat (6) @(negedge clk);
    chk(mode == 2'd2, "T19 mode ZOOM");
    press(4'b0100); settle_load(20000);
    chk(zoom == 8'h20, "T20 zoom 2.0x");
    press(4'b0001); repeat (10) @(negedge clk);
    chk(!zoom_on && (zoom == 8'h20), "T21 1st KEY1 exits, zoom persists 2.0x");
    press(4'b0010); repeat (6) @(negedge clk);
    chk(zoom_on && (zoom == 8'h20) && (mode == 2'd2),
        "T22 reopen shows 2.0x ZOOM (state preserved)");

    //==================================================================
    $display("--- 7. 装载期间按键不会被丢掉 ---");
    //==================================================================
    press(4'b0100);                        // 2 -> 4（立刻进 Z_LOAD）
    repeat (4) @(negedge clk);
    chk(dut.st == 2'd3,     "T23 in Z_LOAD right after zoom key (no debounce)");
    chk(dut.nxt_v === 1'b0, "T23b no pending yet");
    press(4'b0100);                        // 装载中再按
    repeat (2) @(negedge clk);
    chk(zoom == 8'h40, "T24 key during load does not act yet");
    chk(dut.nxt_v,     "T24b but remembered");
    @(negedge clk); ld_done = 1'b1;
    @(negedge clk); ld_done = 1'b0;
    repeat (4) @(negedge clk);
    chk(zoom == 8'h80, "T25 pending step applied right after load");
    settle_load(20000);
    chk(dut.st == 2'd1, "T25b back to Z_HD");

    //==================================================================
    $display("--- 7b. 888 未显示出来时不许开始下一次缩放（防跳变）---");
    //==================================================================
    hd_shown = 1'b0;
    press(4'b1000);                        // 想缩小 8x -> 4x
    repeat (4) @(negedge clk);
    chk(zoom == 8'h80, "T25c not applied while 888 not shown");
    chk(dut.nxt_v,     "T25d captured into pending");
    hd_shown = 1'b1;
    repeat (4) @(negedge clk);
    chk(zoom == 8'h40, "T25e applied once 888 is shown");
    chk(!dut.nxt_v,    "T25f pending consumed");
    settle_load(20000);
    chk(dut.st == 2'd1, "T25g settled");

    //==================================================================
    $display("--- 8. 退出后按 KEY1：完整 565 动画恢复 + 放行动作 ---");
    //==================================================================
    press(4'b0001); repeat (10) @(negedge clk);
    chk(!zoom_on && (zoom == 8'h40), "T26 exited, still 4.0x");

    @(negedge clk); act_code = 2'd3;       // App 把第二次 KEY1 压成"打开 OSD"
    @(negedge clk);
    chk(zoom_on,        "T27 restore re-enters zoom OSD");
    chk(dut.st == 2'd1, "T27b parked at Z_HD one cycle (S_ZOOM not entered yet)");
    chk(!preview_req,   "T27c preview_req NOT in the same cycle (would be dropped)");
    @(negedge clk);
    chk(preview_req,    "T28 preview requested ⇒ full 565 animation runs");
    chk(zoom == 8'h10,  "T29 target forced to 1.0x");
    chk(dut.rest,       "T30 in restore mode");
    chk(mode == 2'd2,   "T31 mode forced to ZOOM while restoring");
    chk(!act_go,        "T32 action not released yet");
    act_code = 2'd0;
    allow    = 1'b0;                       // 进 S_ZOOM ⇒ 不在静止态

    // 无防抖 ⇒ load_req 立刻发；乱按键也不能影响
    press(4'b0100); press(4'b1000);
    chk(dut.st == 2'd3, "T33 restore reached Z_LOAD immediately (no debounce)");
    chk(zoom == 8'h10,  "T33b zoom stays 1.0x during the restore");

    @(negedge clk); ld_done = 1'b1;
    @(negedge clk); ld_done = 1'b0;
    repeat (6) @(negedge clk);
    chk(!zoom_on, "T34 restore session closes zoom OSD");
    chk(!act_go,  "T35 action waits until allow (= S_HD)");

    allow = 1'b1;
    cc = 0;
    repeat (20) begin
        @(negedge clk);
        if (act_go) cc = cc + 1;
        if (act_go && (act_sel !== 2'd3)) cc = cc + 100;
    end
    chk(cc == 1, "T36 menu action released exactly once with sel=3 (open OSD)");

    //==================================================================
    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

initial begin
    #50_000_000;
    $display("  *** GLOBAL TIMEOUT");
    $finish;
end

endmodule
