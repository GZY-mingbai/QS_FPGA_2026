//==============================================================================
//  tb_cursor.v —— 文件页光标（osd_menu 单元级）
//==============================================================================
//  iverilog -g2012 -DIVERILOG_SIM -o tc.vvp -s tb_cursor tb_cursor.v \
//      osd_menu.v tools/bram32k_stub.v tools/mult18_stub.v && vvp tc.vvp
//
//  覆盖（对应 PLAN T4）：
//    C1  导航到 page2（KEY3×2 + KEY2 进入）
//    C2  KEY3 让光标 0→1→2→3
//    C3  第 4 格再按 KEY3 ⇒ 不移动、发 flip_nx（且 has_more=1 才发）
//    C4  第 1 格再按 KEY4 ⇒ 发 flip_pv，并落在末格
//    C5  本页只有 3 格（vcnt=3）时，光标 2 再按 KEY3 ⇒ 直接翻页
//    C6  has_more=0 时第 4 格再按 KEY3 ⇒ 不翻页、光标不动
//    C7  光标移动后动画坐标会朝目标走（fp_cx 变化）
//==============================================================================
`timescale 1ns/1ps

module tb_cursor;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

integer errors = 0, ncase = 0;
task chk;
    input cond;
    input [255:0] msg;
    begin
        ncase = ncase + 1;
        if (cond !== 1'b1) begin
            errors = errors + 1;
            $display("  FAIL [%0d] %0s", ncase, msg);
        end
    end
endtask

//------------------------------------------------------------------------------
//  DUT
//------------------------------------------------------------------------------
reg         menu_on   = 1'b1;
reg         ps_start  = 1'b0;
reg  [3:0]  key_press = 4'd0;
reg  [2:0]  fp_vcnt   = 3'd4;
reg         fp_more   = 1'b1;
reg         fp_prev   = 1'b1;
reg         fp_gen    = 1'b0;

wire [1:0]  fp_cur;
wire        flip_nx, flip_pv;

osd_menu #(.NL(8), .MAXLEN(32)) dut (
    .clk(clk), .rst(rst),
    .tick(1'b0), .ps_start(ps_start), .menu_on(menu_on),
    .zoom_on(1'b0), .zoom_val(8'h10), .zoom_mode(2'd0), .osd_allow(1'b1),
    .key_press(key_press), .key_level(4'hF), .bright_eff(7'd50),
    .enter_req(), .exit_req(), .fade(),
    .wr_en(), .wr_idx(), .wr_geo(), .wr_attr(),
    .txt_xy(), .txt_ci(),
    .O_nm_slot(), .O_nm_idx(), .I_nm_char(8'd65),
    .I_nm_blen(4'd8), .I_nm_elen(4'd3), .I_nm_len4(20'hC6310),
    .I_nm_vld(1'b1),
    .O_fp_cur(fp_cur), .O_fp_flip_nx(flip_nx), .O_fp_flip_pv(flip_pv),
    .I_fp_vcnt(fp_vcnt), .I_fp_has_more(fp_more), .I_fp_gen(fp_gen), .I_fp_has_prev(fp_prev), .O_fp_open(),
    .str_we(), .str_line(), .str_idx(), .str_code(),
    .ctl_we(), .ctl_line(), .ctl_active(), .ctl_len(), .ctl_ver(), .trig(),
    .pic_bright(), .pic_contrast(), .pic_temp(), .pic_sat(), .pic_style(), .pic_tgl(),
    .o_lang(), .o_vol(), .o_mute(), .o_loop(), .o_spc(),
    .O_page(),
    .fx_sel(), .auto_mode(),.info_on(),
    .album_prev(), .album_next(),
    .ic_act_mask()
    // 2026-10-31: tie off ports added after this bench was written
    , .src_w(16'd640), .src_h(16'd480), .auto_remain(6'd0), .auto_bar(9'd0)
);

// 按键：KEY1=bit0 KEY2=bit1 KEY3=bit2 KEY4=bit3
task press;
    input [3:0] k;
    begin
        @(posedge clk);
        key_press = k;
        @(posedge clk);
        key_press = 4'd0;
        repeat (4) @(posedge clk);
    end
endtask

integer i;
initial begin
    repeat (5) @(posedge clk);
    rst = 0;
    repeat (5) @(posedge clk);

    //------------------------------------------------------------------
    $display("\n[C1] 导航到文件页");
    press(4'h4);              // KEY3：nav 0 → 1
    press(4'h4);              // KEY3：nav 1 → 2
    chk(dut.page === 3'd2, "C1 page == 2");
    press(4'h2);              // KEY2：进入该页
    chk(dut.focus === 1'b1, "C1 focus");
    chk(fp_cur === 2'd0,    "C1 cursor starts at 0");

    //------------------------------------------------------------------
    $display("[C2] KEY3 让光标前进");
    press(4'h4);
    chk(fp_cur === 2'd1, "C2 cursor 1");
    press(4'h4);
    chk(fp_cur === 2'd2, "C2 cursor 2");
    press(4'h4);
    chk(fp_cur === 2'd3, "C2 cursor 3");
    chk(flip_nx === 1'b0, "C2 未翻页");

    //------------------------------------------------------------------
    $display("[C3] 第 4 格再按 KEY3 → 翻下一页");
    @(posedge clk);
    key_press = 4'h4;
    @(posedge clk);
    key_press = 4'd0;
    // 抓 flip_nx 的脉冲
    i = 0;
    while (flip_nx !== 1'b1 && i < 20) begin @(posedge clk); i = i + 1; end
    chk(flip_nx === 1'b1, "C3 flip_nx 脉冲");
    repeat (4) @(posedge clk);
    chk(fp_cur === 2'd0, "C3 翻页后光标回 0");

    //------------------------------------------------------------------
    $display("[C4] 第 1 格再按 KEY4 → 翻上一页并落在末格");
    @(posedge clk);
    key_press = 4'h8;
    @(posedge clk);
    key_press = 4'd0;
    i = 0;
    while (flip_pv !== 1'b1 && i < 20) begin @(posedge clk); i = i + 1; end
    chk(flip_pv === 1'b1, "C4 flip_pv 脉冲");
    repeat (4) @(posedge clk);
    chk(fp_cur === 2'd3, "C4 落在末格 3");

    //------------------------------------------------------------------
    $display("[C5] 本页 3 格：光标 2 再按 KEY3 直接翻页");
    fp_vcnt = 3'd3; @(negedge clk); fp_gen = ~fp_gen;   // 真实链路里格数只随"换页"（gen 翻转）变
    repeat (4) @(posedge clk);
    chk(fp_cur === 2'd2, "C5 空格不可停留（夹到 2）");
    @(posedge clk); key_press = 4'h4;
    @(posedge clk); key_press = 4'd0;
    i = 0;
    while (flip_nx !== 1'b1 && i < 20) begin @(posedge clk); i = i + 1; end
    chk(flip_nx === 1'b1, "C5 满 3 格就翻页");

    //------------------------------------------------------------------
    $display("[C6] has_more=0：第 4 格再按 KEY3 不翻页");
    fp_vcnt = 3'd4;
    fp_more = 1'b0;
    repeat (6) @(posedge clk);
    press(4'h4); press(4'h4); press(4'h4);      // 走到 3
    chk(fp_cur === 2'd3, "C6 cursor 3");
    @(posedge clk); key_press = 4'h4;
    @(posedge clk); key_press = 4'd0;
    i = 0;
    while (i < 20) begin @(posedge clk); i = i + 1;
        if (flip_nx === 1'b1) i = 99; end
    chk(i != 99, "C6 不发 flip_nx");
    chk(fp_cur === 2'd3, "C6 光标不动");

    //------------------------------------------------------------------
    //  C7 光标目标 = 格子坐标（光标改用 OSD 自带那套 tween 之后，
    //     这里只看"目标"和"动画确实在走"，具体像素由 tgt_* 决定）
    //------------------------------------------------------------------
    $display("[C7] 光标目标 = 格子坐标");
    fp_more = 1'b1;
    press(4'h8);                              // 回 2
    press(4'h8);                              // 回 1
    press(4'h8);                              // 回 0（左上）
    repeat (20) @(posedge clk);
    chk(dut.tgt_cx === 10'd192 && dut.tgt_cy === 10'd100, "C7 左上目标 (192,100)");
    chk(dut.tgt_cw === 10'd160 && dut.tgt_ch === 10'd120, "C7 格子尺寸 160x120");
    press(4'h4);                              // → 1（右上）
    repeat (20) @(posedge clk);
    chk(dut.tgt_cx === 10'd384 && dut.tgt_cy === 10'd100, "C7 右上目标 (384,100)");
    press(4'h4);                              // → 2（左下）
    repeat (20) @(posedge clk);
    chk(dut.tgt_cx === 10'd192 && dut.tgt_cy === 10'd260, "C7 左下目标 (192,260)");
    // 动画：50MHz 下 tween 每帧走一步，tick 期间 cx_c 必须真的在变
    begin : anim_chk
        integer t0v;
        t0v = dut.cx_c;
        repeat (1_800_000) @(posedge clk);
        chk(dut.cx_c != t0v, "C7 动画在走（cx_c 有变化）");
    end

    $display("\n========================================");
    if (errors == 0) $display("  ALL PASS  (%0d checks)", ncase);
    else             $display("  FAIL: %0d / %0d", errors, ncase);
    $display("========================================\n");
    $finish;
end

initial begin
    #120_000_000;
    $display("  FAIL: watchdog");
    $finish;
end

endmodule
