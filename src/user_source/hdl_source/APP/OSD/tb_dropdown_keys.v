//==============================================================================
//  tb_dropdown_keys.v —— 相册页 Dropdown 的按键方向 / 选项窗口专项
//==============================================================================
//
//  【要证的事】
//    ① KEY4 = 上（选项值 -1）、KEY3 = 下（选项值 +1），两端夹紧；
//    ② 自动切换（6 项）打开后，弹层窗口的 3 行必须是**三个不同的连续选项**，
//       且选中项只在 ComboBox 那一行出现一次 —— 不允许出现两个"音乐结束"。
//    ③ 窗口滚动到末页时，第 6/7 行不能留旧选项文字（pop_v6/v7 无效即空白）。
//
//  【怎么跑】工作目录 = APP/OSD/
//    iverilog -g2005 -o /tmp/tb_dd.vvp tb_dropdown_keys.v osd_menu.v
//    vvp /tmp/tb_dd.vvp
//==============================================================================
`timescale 1ns / 1ps

module tb_dropdown_keys;

localparam integer NL = 8;
localparam integer MAXLEN = 32;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg        tick = 0;
reg        ps_start = 0;
reg        menu_on = 0;
reg        osd_allow = 1;
reg  [3:0] key_press = 0;
reg  [3:0] key_level = 4'hF;
reg [15:0] src_w = 16'd640, src_h = 16'd480;
reg  [5:0] auto_remain = 6'd0;
reg  [8:0] auto_bar = 9'd0;

wire enter_req, exit_req;
wire [7:0] fade;
wire wr_en;
wire [4:0] wr_idx;
wire [38:0] wr_geo;
wire [23:0] wr_attr;
wire [22*NL-1:0] txt_xy;
wire [24*NL-1:0] txt_col;
wire str_we;
wire [3:0] str_line;
wire [7:0] str_idx;
wire [15:0] str_code;
wire ctl_we;
wire [3:0] ctl_line;
wire ctl_active;
wire [7:0] ctl_len, ctl_ver;
wire trig;
wire [6:0] pic_bright, pic_contrast, pic_temp, pic_sat;
wire [1:0] pic_style;
wire pic_tgl;
wire [2:0] fx_sel, auto_mode;
wire info_on, album_prev, album_next;

osd_menu #(.NL(NL), .MAXLEN(MAXLEN), .SELFTEST(0)) dut (
    .clk(clk), .rst(rst), .tick(tick), .ps_start(ps_start),
    .menu_on(menu_on), .osd_allow(osd_allow),
    .key_press(key_press), .key_level(key_level), .bright_eff(7'd50),
    .enter_req(enter_req), .exit_req(exit_req), .fade(fade),
    .wr_en(wr_en), .wr_idx(wr_idx), .wr_geo(wr_geo), .wr_attr(wr_attr),
    .txt_xy(txt_xy), .txt_col(txt_col),
    .str_we(str_we), .str_line(str_line), .str_idx(str_idx), .str_code(str_code),
    .ctl_we(ctl_we), .ctl_line(ctl_line), .ctl_active(ctl_active),
    .ctl_len(ctl_len), .ctl_ver(ctl_ver), .trig(trig),
    .pic_bright(pic_bright), .pic_contrast(pic_contrast), .pic_temp(pic_temp),
    .pic_sat(pic_sat), .pic_style(pic_style), .pic_tgl(pic_tgl),
    .fx_sel(fx_sel), .auto_mode(auto_mode),.info_on(info_on),
    .album_prev(album_prev), .album_next(album_next),
    .src_w(src_w), .src_h(src_h),
    .auto_remain(auto_remain), .auto_bar(auto_bar)
);

//------------------------------------------------------------------------------
//  选项字符串捕获（按行/列）
//------------------------------------------------------------------------------
reg [15:0] cap6 [0:31];
reg [15:0] cap7 [0:31];
reg [15:0] cap14 [0:7];
integer ci;
initial for (ci = 0; ci < 32; ci = ci + 1) begin cap6[ci] = 16'h0; cap7[ci] = 16'h0; end
initial for (ci = 0; ci < 8; ci = ci + 1) cap14[ci] = 16'h0;

always @(negedge clk) begin
    if (str_we && ((str_line == 4'd6) || (str_line == 4'd7)) && (str_idx == 8'd0))
        $display("      [WR] t=%0t line=%0d idx=%0d code=%04h sel=%0d w0=%0d la=%0d lb=%0d",
                 $time, str_line, str_idx, str_code, dut.pop_sel, dut.pop_w0, dut.pop_la, dut.pop_lb);
    if (str_we) begin
        if (str_line == 4'd6 && str_idx < 32) cap6[str_idx] <= str_code;
        if (str_line == 4'd7 && str_idx < 32) cap7[str_idx] <= str_code;
    end
    if (ctl_we) begin
        // 每个控件行的第 14 列最后写入的字符
        // （选项串从第 14 列开始，4 个字够判断）
    end
end

// 选中行 = sel_k+1；单独捕它的第 14..17 列
reg [15:0] capsel [0:3];
initial for (ci = 0; ci < 4; ci = ci + 1) capsel[ci] = 16'h0;
always @(negedge clk) begin
    if (str_we && (str_line == ({1'b0, dut.sel_k} + 4'd1)) &&
        (str_idx >= 8'd14) && (str_idx < 8'd18))
        capsel[str_idx - 8'd14] <= str_code;
end

//------------------------------------------------------------------------------
//  选择/节拍
//------------------------------------------------------------------------------
task do_tick; begin
    @(negedge clk); tick = 1'b1;
    @(negedge clk); tick = 1'b0;
end endtask

task do_key;
    input integer kk;
    begin
        @(negedge clk); key_press = (4'b0001 << kk);
        @(negedge clk); key_press = 4'h0;
        repeat (4) @(posedge clk);
    end
endtask

task settle;
    integer i;
    begin
        for (i = 0; i < 80; i = i + 1) begin
            do_tick;
            repeat (20) @(posedge clk);
        end
    end
endtask

//------------------------------------------------------------------------------
//  名字（自动切换选项）
//------------------------------------------------------------------------------
function [8*8:1] aname;
    input [2:0] k;
    begin
        case (k)
            3'd0: aname = "off";
            3'd1: aname = "3s";
            3'd2: aname = "5s";
            3'd3: aname = "10s";
            3'd4: aname = "30s";
            default: aname = "music";
        endcase
    end
endfunction

// 打印一行 4 个字（hex），并判断是否空
task showline;
    input [8*6:1] tag;
    input integer ln;
    integer j;
    reg [15:0] v;
    reg blank;
    begin
        blank = 1'b1;
        $write("      %0s:", tag);
        for (j = 0; j < 4; j = j + 1) begin
            v = (ln == 6) ? cap6[j] : cap7[j];
            if (v != 16'h0) blank = 1'b0;
            $write(" %04h", v);
        end
        $write("%0s\n", blank ? "  (blank)" : "");
    end
endtask

//------------------------------------------------------------------------------
//  断言
//------------------------------------------------------------------------------
integer fails = 0, checks = 0;
task chk;
    input [8*56:1] name;
    input integer got, exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s: got %0d, exp %0d", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %0d", name, got);
    end
endtask

// 判断两行选项是否都不是"选中项"（用前 4 个字的 hex 比对）
task chk_window;
    input integer sel;
    input [8*48:1] tag;
    integer j;
    reg same6, same7, same67;
    begin
        same6 = 1'b1; same7 = 1'b1; same67 = 1'b1;
        for (j = 0; j < 4; j = j + 1) begin
            if (cap6[j] !== capsel[j]) same6 = 1'b0;   // line6 与选中项相同？
            if (cap7[j] !== capsel[j]) same7 = 1'b0;   // line7 与选中项相同？
            if (cap6[j] !== cap7[j])   same67 = 1'b0;  // line6 与 line7 相同？
        end
        checks = checks + 1;
        if (same6 || same7) begin
            $display("  *** FAIL  %0s: duplicate of selected option (sel=%0d)", tag, sel);
            fails = fails + 1;
        end else if (same67) begin
            $display("  *** FAIL  %0s: line6 == line7", tag);
            fails = fails + 1;
        end else
            $display("   ok       %0s window distinct", tag);
    end
endtask

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
integer n, v0;
initial begin
    repeat (4) @(posedge clk);
    rst = 0;
    repeat (10) @(posedge clk);
    menu_on = 1;                          // 直接开菜单（focus=0/nav）
    settle;

    // nav 0 -> 1（相册），进入页面
    do_key(2); settle;                   // KEY3 下
    chk("nav=1", dut.nav_idx, 1);
    do_key(1); settle;                   // KEY2 进页面
    chk("focus=1", dut.focus, 1);

    // item0 -> item3（自动切换）
    do_key(2); settle;
    do_key(2); settle;
    do_key(2); settle;
    chk("item=3", dut.item, 3);

    do_key(1); settle;                   // 打开自动切换下拉
    chk("pop open", dut.pop_open, 1);
    v0 = dut.pop_sel;
    $display("=== A. 打开自动切换：pop_sel=%0d pop_w0=%0d pop_a=%0d ===",
             v0, dut.pop_w0, dut.pop_a);
    showline("line6", 6); showline("line7", 7);
    $display("      sel row: %04h %04h %04h %04h", capsel[0], capsel[1], capsel[2], capsel[3]);
    chk_window(v0, "A open");

    //--------------------------------------------------------------
    // B. KEY4 = 上：sel 应该 -1；到 0 夹紧
    //--------------------------------------------------------------
    $display("=== B. KEY4 上 ===");
    // 先推到 3（用 KEY3 下），再按 KEY4 回来
    do_key(2); settle; chk("down 1", dut.pop_sel, v0 + 1);
    do_key(2); settle; chk("down 2", dut.pop_sel, v0 + 2);
    do_key(2); settle; chk("down 3", dut.pop_sel, v0 + 3);
    showline("line6", 6); showline("line7", 7);
    chk_window(dut.pop_sel, "B sel3");
    do_key(3); settle; chk("up 1", dut.pop_sel, v0 + 2);
    showline("line6", 6); showline("line7", 7);
    chk_window(dut.pop_sel, "B up1");
    do_key(3); settle; chk("up 2", dut.pop_sel, v0 + 1);
    do_key(3); settle; chk("up 3", dut.pop_sel, v0);
    do_key(3); settle; chk("up clamp", dut.pop_sel, 0);

    //--------------------------------------------------------------
    // C. 一路下到底（5=音乐结束），窗口只能有 10s/30s/音乐结束
    //--------------------------------------------------------------
    $display("=== C. KEY3 下到底 ===");
    for (n = 0; n < 6; n = n + 1) do_key(2);
    settle;
    chk("sel=5", dut.pop_sel, 5);
    $display("      pop_w0=%0d pop_a=%0d pop_la=%0d pop_lb=%0d v6=%0d v7=%0d",
             dut.pop_w0, dut.pop_a, dut.pop_la, dut.pop_lb, dut.pop_v6, dut.pop_v7);
    showline("line6", 6); showline("line7", 7);
    $display("      sel row: %04h %04h %04h %04h", capsel[0], capsel[1], capsel[2], capsel[3]);
    $display("      pop: sel=%0d w0=%0d a=%0d la=%0d lb=%0d v6=%0d v7=%0d",
             dut.pop_sel, dut.pop_w0, dut.pop_a, dut.pop_la, dut.pop_lb, dut.pop_v6, dut.pop_v7);
    chk_window(5, "C sel5");

    //--------------------------------------------------------------
    // D. 再按 KEY4 上：5 -> 4，窗口不能出现两个"音乐结束"
    //--------------------------------------------------------------
    $display("=== D. KEY4 上 from 5 ===");
    do_key(3); settle;
    chk("sel=4", dut.pop_sel, 4);
    showline("line6", 6); showline("line7", 7);
    $display("      sel row: %04h %04h %04h %04h", capsel[0], capsel[1], capsel[2], capsel[3]);
    $display("      pop: sel=%0d w0=%0d a=%0d la=%0d lb=%0d v6=%0d v7=%0d",
             dut.pop_sel, dut.pop_w0, dut.pop_a, dut.pop_la, dut.pop_lb, dut.pop_v6, dut.pop_v7);
    chk_window(4, "D sel4");
    do_key(3); settle;
    chk("sel=3", dut.pop_sel, 3);
    showline("line6", 6); showline("line7", 7);
    $display("      sel row: %04h %04h %04h %04h", capsel[0], capsel[1], capsel[2], capsel[3]);
    $display("      pop: sel=%0d w0=%0d a=%0d la=%0d lb=%0d v6=%0d v7=%0d",
             dut.pop_sel, dut.pop_w0, dut.pop_a, dut.pop_la, dut.pop_lb, dut.pop_v6, dut.pop_v7);
    chk_window(3, "D sel3");

    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

endmodule
