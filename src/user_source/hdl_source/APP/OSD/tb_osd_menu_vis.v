//==============================================================================
//  tb_osd_menu_vis.v —— osd_menu 的行可见性 / KEY2 轮换效果 专项验证
//==============================================================================
//
//  【验的是 2026-10-27 的两处修复】
//    ① `row_vis`：信息条模式下第 0..5 行必须关掉（ctl_active=0, ctl_len=0），
//       否则菜单的"相册/切换效果/自动切换…"会和信息条文字叠在一起
//       （用户报的"信息条显示时会一起显示 OSD 的文字"）。
//    ② `fx_next`：菜单关闭时按 KEY2 要能轮换"切换效果"（0→1→2→3→4→0）；
//       菜单打开时 KEY2 是"进入/确认"，绝不能改效果。
//
//  【怎么打开信息条】走真实按键：菜单 → 导航到"相册" → 进入 → 下移到第 6 行
//    （信息条）→ KEY2 反选。这样测的是端到端行为，不是伪造内部状态。
//
//  【怎么跑】
//    cd src/user_source/hdl_source/APP/OSD
//    iverilog -g2005 -o /tmp/tb_vis.vvp tb_osd_menu_vis.v osd_menu.v
//    vvp /tmp/tb_vis.vvp
//
//==============================================================================
`timescale 1ns / 1ps

module tb_osd_menu_vis;

localparam integer NL     = 8;
localparam integer MAXLEN = 32;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

//------------------------------------------------------------------------------
//  DUT 输入
//------------------------------------------------------------------------------
reg        tick       = 0;
reg        ps_start   = 0;
reg        menu_on    = 0;
reg        osd_allow  = 1;
reg  [3:0] key_press  = 4'h0;
reg  [3:0] key_level  = 4'hF;        // 1 = 松开
reg [15:0] src_w      = 16'd640;
reg [15:0] src_h      = 16'd480;
reg  [5:0] auto_remain= 6'd3;
reg  [8:0] auto_bar   = 9'd200;

//------------------------------------------------------------------------------
//  DUT 输出
//------------------------------------------------------------------------------
wire        enter_req, exit_req;
wire  [7:0] fade;
wire        wr_en;
wire  [4:0] wr_idx;
wire [38:0] wr_geo;
wire [23:0] wr_attr;
wire [22*NL-1:0] txt_xy;
wire [24*NL-1:0] txt_col;
wire        str_we;
wire  [3:0] str_line;
wire  [7:0] str_idx;
wire [15:0] str_code;
wire        ctl_we;
wire  [3:0] ctl_line;
wire        ctl_active;
wire  [7:0] ctl_len, ctl_ver;
wire        trig;
wire  [6:0] pic_bright, pic_contrast, pic_temp, pic_sat;
wire  [1:0] pic_style;
wire        pic_tgl;
wire  [2:0] fx_sel, auto_mode;
wire        info_on;
wire        album_prev, album_next;

osd_menu #(
    .NL        (NL),
    .MAXLEN    (MAXLEN),
    .SELFTEST  (0),
    .RPT_FIRST (30),
    .RPT_EVERY (5)
) dut (
    .clk        (clk),
    .rst        (rst),
    .tick       (tick),
    .ps_start   (ps_start),
    .menu_on    (menu_on),
    .osd_allow  (osd_allow),
    .key_press  (key_press),
    .key_level  (key_level), .bright_eff(7'd50),
    .enter_req  (enter_req),
    .exit_req   (exit_req),
    .fade       (fade),
    .wr_en      (wr_en),
    .wr_idx     (wr_idx),
    .wr_geo     (wr_geo),
    .wr_attr    (wr_attr),
    .txt_xy     (txt_xy),
    .txt_col    (txt_col),
    .str_we     (str_we),
    .str_line   (str_line),
    .str_idx    (str_idx),
    .str_code   (str_code),
    .ctl_we     (ctl_we),
    .ctl_line   (ctl_line),
    .ctl_active (ctl_active),
    .ctl_len    (ctl_len),
    .ctl_ver    (ctl_ver),
    .trig       (trig),
    .pic_bright (pic_bright),
    .pic_contrast(pic_contrast),
    .pic_temp   (pic_temp),
    .pic_sat    (pic_sat),
    .pic_style  (pic_style),
    .pic_tgl    (pic_tgl),
    .fx_sel     (fx_sel),
    .auto_mode  (auto_mode),
    .info_on    (info_on),
    .album_prev (album_prev),
    .album_next (album_next),
    .src_w      (src_w),
    .src_h      (src_h),
    .auto_remain(auto_remain),
    .auto_bar   (auto_bar)
);

//------------------------------------------------------------------------------
//  收集每次 ctl_we 写进去的行属性（后写的覆盖先写的）
//------------------------------------------------------------------------------
reg [7:0] got_len [0:NL-1];
reg       got_act [0:NL-1];
reg       got_hit [0:NL-1];
integer   k;

task clr_got;
begin
    for (k = 0; k < NL; k = k + 1) begin
        got_len[k] = 8'hxx;
        got_act[k] = 1'bx;
        got_hit[k] = 1'b0;
    end
end
endtask

// ---- 控件几何收集（cell 4 = 游标/弹层底板）----
reg [38:0] got_geo [0:18];
integer    gj;
initial for (gj = 0; gj < 19; gj = gj + 1) got_geo[gj] = 39'd0;

always @(posedge clk) begin
    if (ctl_we && (ctl_line < NL)) begin
        got_len[ctl_line] = ctl_len;
        got_act[ctl_line] = ctl_active;
        got_hit[ctl_line] = 1'b1;
    end
    if (wr_en && (wr_idx < 19)) got_geo[wr_idx] = wr_geo;
end

//------------------------------------------------------------------------------
//  检查工具
//------------------------------------------------------------------------------
integer errors = 0;

task chk;
    input [8*32-1:0] name;
    input integer    got;
    input integer    exp;
    begin
        if (got === exp)
            $display("   ok       %0s = %0d", name, got);
        else begin
            $display("  *** FAIL  %0s = %0d (exp %0d)", name, got, exp);
            errors = errors + 1;
        end
    end
endtask

//------------------------------------------------------------------------------
//  激励
//------------------------------------------------------------------------------
task do_tick;
begin
    @(negedge clk); tick = 1'b1;
    @(negedge clk); tick = 1'b0;
end
endtask

// k: 0=KEY1 1=KEY2 2=KEY3 3=KEY4
task do_key;
    input integer kk;
begin
    @(negedge clk); key_press = (4'b0001 << kk);
    @(negedge clk); key_press = 4'h0;
    @(negedge clk);
end
endtask

task wait_layout;
begin
    repeat (420) @(negedge clk);
end
endtask

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
integer n;

initial begin
    clr_got;
    rst = 1;
    repeat (5) @(negedge clk);
    rst = 0;
    repeat (5) @(negedge clk);

    //==================================================================
    //  M0：确认缺省状态
    //==================================================================
    $display("========================================");
    $display("=== M0：缺省状态 ===");
    chk("M0 info_on==0",   info_on,  0);
    chk("M0 fx_sel==0",    fx_sel,   0);

    //==================================================================
    //  M1：菜单打开，跑一轮布局 ⇒ 8 行全部 active / len=32
    //==================================================================
    $display("========================================");
    $display("=== M1：菜单模式 ⇒ 8 行全显示 ===");
    clr_got;
    @(negedge clk); menu_on = 1'b1;
    do_tick;
    wait_layout;

    for (k = 0; k < NL; k = k + 1) begin
        if (!got_hit[k]) begin
            $display("  *** FAIL  M1 line%0d 没有收到 ctl_we", k);
            errors = errors + 1;
        end else begin
            if (got_act[k] !== 1'b1) begin
                $display("  *** FAIL  M1 line%0d active=%b (exp 1)", k, got_act[k]);
                errors = errors + 1;
            end
            if (got_len[k] !== 8'd32) begin
                $display("  *** FAIL  M1 line%0d len=%0d (exp 32)", k, got_len[k]);
                errors = errors + 1;
            end
        end
    end
    if (errors == 0) $display("   ok       M1 8 行均 active/len=32");

    //==================================================================
    //  M2：走真实按键打开"信息条"，再关菜单 ⇒ 只有 6/7 行可见
    //==================================================================
    $display("========================================");
    $display("=== M2：相册页勾选\"信息条\" ===");
    // focus=0(nav)，nav_idx=0 → 按 KEY3 到 1（相册）
    do_key(2);                 // KEY3：nav_idx 0 -> 1
    repeat (20) @(negedge clk);
    do_key(1);                 // KEY2：进页面
    repeat (20) @(negedge clk);
    // item 0..4，下移到 4（信息条；删除“适应屏幕”后 6 项）
    repeat (4) do_key(2);
    repeat (20) @(negedge clk);
    // KEY2 反选 CheckBox
    do_key(1);
    repeat (30) @(negedge clk);

    chk("M2 info_on==1",   info_on,  1);

    // 再设一个"必须被 KEY2 保住"的值：自动切换 = 3s（item3 ComboBox）
    do_key(3);                 // KEY4：item4 -> item3
    repeat (10) @(negedge clk);
    do_key(1);                 // KEY2：打开 自动切换
    repeat (20) @(negedge clk);
    do_key(2);                 // KEY3：向下 -> 3s（新方向：KEY3 增大）
    repeat (10) @(negedge clk);
    do_key(1);                 // KEY2：确认
    repeat (20) @(negedge clk);
    chk("M2 auto_mode==1", auto_mode, 1);

    $display("=== M2：关掉菜单 ⇒ 只允许第 6/7 行 ===");
    clr_got;
    @(negedge clk); menu_on = 1'b0;
    do_tick;
    wait_layout;

    for (k = 0; k < 6; k = k + 1) begin
        if (!got_hit[k]) begin
            $display("  *** FAIL  M2 line%0d 没有收到 ctl_we", k);
            errors = errors + 1;
        end else begin
            if (got_act[k] !== 1'b0) begin
                $display("  *** FAIL  M2 line%0d active=%b (exp 0)", k, got_act[k]);
                errors = errors + 1;
            end
            if (got_len[k] !== 8'd0) begin
                $display("  *** FAIL  M2 line%0d len=%0d (exp 0)", k, got_len[k]);
                errors = errors + 1;
            end
        end
    end
    for (k = 6; k < NL; k = k + 1) begin
        if (!got_hit[k]) begin
            $display("  *** FAIL  M2 line%0d 没有收到 ctl_we", k);
            errors = errors + 1;
        end else begin
            if (got_act[k] !== 1'b1) begin
                $display("  *** FAIL  M2 line%0d active=%b (exp 1)", k, got_act[k]);
                errors = errors + 1;
            end
            // ★ 2026-10-27：信息条两行改成定长：文件名 8、分辨率+倒计时 12
            if (got_len[k] !== ((k == 6) ? 8'd8 : 8'd19)) begin
                $display("  *** FAIL  M2 line%0d len=%0d (exp %0d)",
                         k, got_len[k], (k == 6) ? 8 : 19);
                errors = errors + 1;
            end
        end
    end
    if (errors == 0)
        $display("   ok       M2 line0..5 全关、line6/7 全开");

    //==================================================================
    //  M3：菜单关闭时按 KEY2 ⇒ fx_sel 0→1→2→3→4→0 循环
    //==================================================================
    $display("========================================");
    $display("=== M3：菜单关闭时 KEY2 轮换效果 ===");
    // 注意：M2 结束时 page_r=1（相册页），fx_sel 取 val2。
    //       先把 val2 归零（它本来就是 DEF1 的 0）
    chk("M3 start fx_sel", fx_sel,   0);
    begin : m3_regs
        integer info0, auto0;
        info0 = info_on; auto0 = auto_mode;
        $display("      before KEY2: info_on=%0d auto=%0d", info0, auto0);
        for (n = 1; n <= 6; n = n + 1) begin
            do_key(1);                          // KEY2
            repeat (5) @(negedge clk);
            chk("M3 KEY2 x N -> fx_sel", fx_sel, (n % 5));
            // ★ 2026-10-27：KEY2 只许改 fx_sel，相册其它设置必须原样
            chk("M3 info_on kept",   info_on,   info0);
            chk("M3 auto_mode kept", auto_mode, auto0);
        end
    end

    //==================================================================
    //  M4：菜单打开时按 KEY2 ⇒ fx_sel 不变（KEY2 是"进入/确认"）
    //==================================================================
    $display("========================================");
    $display("=== M4：菜单打开时 KEY2 不改效果 ===");
    begin : m4
        integer fx_before;
        @(negedge clk); menu_on = 1'b1;
        repeat (40) @(negedge clk);
        fx_before = fx_sel;
        do_key(1);
        repeat (20) @(negedge clk);
        chk("M4 fx_sel stable", fx_sel, fx_before);
    end

    //==================================================================
    //  M7：文件页（空白页）也必须画游标
    //      修复前 osd_menu 里有一段 `else if (page_empty && cell_i==4)`
    //      把游标槽清成空矩形，进"文件"页后看不到任何选中指示。
    //==================================================================
    $display("========================================");
    $display("=== M7：文件页游标 ===");
    begin : m7
        integer gx0, gx1, gw;
        // 回导航栏 → 移到 nav_idx=2（文件）→ 进页面
        do_key(0);                       // KEY1：focus 1 -> 0
        repeat (20) @(negedge clk);
        do_key(2);                       // KEY3：nav_idx 1 -> 2
        repeat (20) @(negedge clk);
        do_key(1);                       // KEY2：进页面
        // ★ 游标是 12 帧缓动（STEP_C=5461 / 65536），只给一次 tick 会停在
        //   导航栏与页面框之间的中间值（实测 gx0=31、宽 74）。
        //   这里连推 20 帧，确保几何收敛到页面框（104 / 522）。
        repeat (20) begin
            do_tick;
            wait_layout;
        end

        gx0 = got_geo[4][18:9];
        gx1 = got_geo[4][37:28];
        gw  = gx1 - gx0;
        chk("M7 cursor gx0",  gx0, 104);
        chk("M7 cursor width", gw,  522);
    end

    $display("========================================");
    if (errors == 0) $display("=== tb_osd_menu_vis ALL PASS ===");
    else             $display("=== tb_osd_menu_vis %0d FAIL ===", errors);
    $finish;
end

endmodule
