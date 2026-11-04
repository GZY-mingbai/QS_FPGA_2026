//==============================================================================
//  tb_abort_job.v —— abort 语义 + "旧装载不许验证画布" 单元台（viewer_ctrl）
//==============================================================================
//  【要证的三件事】
//   A1  换图打断在飞的 HD 装载时，`O_ld_abort` 必须被拉高**并保持到装载收尾**
//       （现状：单拍脉冲，且只在 ldp==2 发）
//   A2  装载器"没听 abort"（模拟脉冲丢失 / 状态不认 ab_r）时，
//       这次装载完成 **绝不能** 让旧画布被当成新图接受
//       —— 现状会被接受：`hd_img == cur_rel`（滑窗时 cr 恒为 1）⇒ hd_ready=1
//   A3  装载器忙（ld_busy=1）时不得再发新的 ld_req（hd_take 要先看 ld_busy）
//
//  【判据都是不变量，不依赖坐标/时序】
//   V1（每拍）：`hd_valid && (hd_img == cr)` ⇒ HD 槽里的内容必须就是当前这张
//   V2：abort 后 `view_busy`/`ld_busy` 必须都落，且 `ldp` 回到 0
//   V3：任何时刻 `ld_req` 都不许在 `ldbusy` 为 1 时出现
//
//  【场景】NIMG=3、窗口不滑动（I_win4=0）⇒ 只走"窗口内挪 cr"，把 abort 逻辑
//   和相册重扫彻底解耦，台子只盯装载/放弃这一条链。
//
//  跑法（在 APP/ 下）：
//    iverilog -g2012 -o ab.vvp -s tb_abort_job tb_abort_job.v viewer_ctrl.v \
//             viewer_fx.v img_switch.v && vvp ab.vvp
//  （run_all_tb.sh 用 -y 自动找模块，无需写全）
//==============================================================================
`timescale 1ns/1ps
`define SIM_FAST_TO       // viewer_ctrl 的 ONE_TO 在仿真里走比较器，否则跑不动

module tb_abort_job;

parameter integer LAT_T = 40;        // 缩略图装载延迟（拍）
parameter integer LAT_H = 200;       // 高清装载延迟（拍）
localparam integer ABORT_LAT = 6;    // 收到 abort 后多少拍收尾（模拟"扇区边界"）

localparam [20:0] T0 = 21'd614400, T1 = 21'd652800, T2 = 21'd691200, HD = 21'd729600;

localparam [3:0] ST_HDF  = 4'd4;
localparam [3:0] ST_HD   = 4'd5;
localparam [3:0] ST_STUK = 4'd6;
localparam [3:0] ST_LOW  = 4'd2;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

//------------------------------------------------------------------------------
//  激励
//------------------------------------------------------------------------------
reg  key_prev = 0, key_next = 0, key_fx = 0;
reg  busy = 0, ps_start = 0;
reg  [2:0] fx_sel = 3'd0;

//  DUT 端口
wire [15:0] O_base;
wire        ld_req, ld_fmt, ld_can_load, ld_done, ld_err;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
wire        compose_en, fmt565, a_hd, a_en, b_en;
wire [20:0] a_base, b_base;
wire [31:0] pos_a, size_a, pos_b, size_b;
wire [39:0] k_a, k_b;
wire [19:0] ori_a;
wire [7:0]  alpha_b;
wire [1:0]  st_img;
wire [2:0]  st_fx;
wire [3:0]  st_state;
wire        O_ld_abort, O_sw_go;
wire [1:0]  O_hd_rel, O_cur_rel;
// ★ [MM-EARLY]：换图后一拍的"提前扫 match"请求（音乐/字幕定位）
wire        O_mm_arm;
wire [1:0]  O_mm_rel;
wire        osd_on, osd_allow, zoom_allow, info_show;
wire [15:0] res_w, res_h;
wire [5:0]  auto_remain;
wire [8:0]  auto_bar;
wire [3:0]  fps_dbg;
wire [7:0]  ld_zoom;
wire [9:0]  ld_ax, ld_ay;

//  相册参数：3 张、窗口不滑（I_win4=0）⇒ O_base 恒 0
function [31:0] off_of;
    input integer idx;
    begin off_of = 32'h0080_0000 + idx * 32'h0000_4000; end
endfunction

//------------------------------------------------------------------------------
//  DUT
//------------------------------------------------------------------------------
viewer_ctrl #(
    .T_END(6), .T_END_F(3), .ARM_FRAMES(1), .FRAME_CYC(1000),
    .THUMB0_ADDR(T0), .THUMB1_ADDR(T1), .THUMB2_ADDR(T2), .HD_ADDR(HD),
    .ONE_TO(2000)
) dut (
    .clk(clk), .rst(rst),
    .I_img_off0(off_of(0)), .I_img_off1(off_of(1)),
    .I_img_off2(off_of(2)), .I_img_off3(off_of(3)),
    .O_base(O_base),
    .I_win4(1'b0), .I_any(1'b1), .I_nimg(3'd3), .I_alb_ready(1'b1),
    .I_jump(1'b0), .I_jump_idx(8'd0), .I_file_page(1'b0),
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(1'b0), .key_osd_exit(1'b0), .osd_fade(8'd255),
    .osd_on(osd_on), .osd_allow(osd_allow),
    .zoom_on(1'b0), .zoom_allow(zoom_allow), .preview_req(1'b0), .load_req(1'b0),
    .zoom_val(8'h10), .za_x(10'd320), .za_y(10'd240),
    .busy(busy), .ps_start(ps_start), .pipe_ce(1'b1),
    .ld_req(ld_req), .ld_fmt(ld_fmt), .ld_off(ld_off), .ld_len(ld_len), .ld_base(ld_base),
    .ld_can_load(ld_can_load), .ld_done(ld_done), .ld_err(ld_err), .ld_sd_ready(1'b1),
    .O_ld_abort(O_ld_abort), .O_hd_rel(O_hd_rel), .O_cur_rel(O_cur_rel), .O_sw_go(O_sw_go),
    .O_mm_arm(O_mm_arm), .O_mm_rel(O_mm_rel),
    .compose_en(compose_en), .fps_dbg(fps_dbg), .fmt565(fmt565),
    .a_hd(a_hd), .a_en(a_en), .a_base(a_base),
    .b_en(b_en), .b_base(b_base),
    .pos_a(pos_a), .size_a(size_a), .k_a(k_a), .ori_a(ori_a),
    .pos_b(pos_b), .size_b(size_b), .k_b(k_b), .alpha_b(alpha_b),
    .st_img(st_img), .st_fx(st_fx), .st_state(st_state),
    .fx_sel(fx_sel), .auto_mode(3'd0),.info_on(1'b0),
    .album_prev(1'b0), .album_next(1'b0), .osd_trig(1'b0), .music_done(1'b0),
    .ld_src_w(16'd640), .ld_src_h(16'd480),
    .res_w(res_w), .res_h(res_h),
    .auto_remain(auto_remain), .auto_bar(auto_bar),
    .info_show(info_show),    .ld_zoom(ld_zoom), .ld_ax(ld_ax), .ld_ay(ld_ay)
);

//------------------------------------------------------------------------------
//  装载器模型：接受 ld_req → LAT 拍后 done；可选"听 abort"或"装死不听"
//------------------------------------------------------------------------------
reg        abort_works = 1;   // 0 = 模拟 abort 丢失/被忽略（D2/D4 的现场）
reg        ldbusy = 0;
reg        ld_done_r = 0;
reg        ld_err_r  = 0;
reg [31:0] cap_off;
reg        cap_fmt;
reg [31:0] hd_off_r;          // HD 槽里现在到底是什么
reg        hd_any;
reg [15:0] ldcnt;
reg        abort_seen = 0;    // 本次装载收到过 abort
reg        ld_req_while_busy = 0;   // V3 违例计数

assign ld_done    = ld_done_r;
assign ld_err     = ld_err_r;
assign ld_can_load = !ldbusy;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        ldbusy <= 0; ld_done_r <= 0; ld_err_r <= 0; ldcnt <= 0;
        hd_off_r <= 32'hDEAD_BEEF; hd_any <= 0; abort_seen <= 0;
        ld_req_while_busy <= 0;
    end else begin
        ld_done_r <= 1'b0;
        ld_err_r  <= 1'b0;
        if (ld_req && !ldbusy) begin
            ldbusy    <= 1'b1;
            ldcnt     <= 16'd0;
            cap_off   <= ld_off;
            cap_fmt   <= ld_fmt;
            abort_seen <= 1'b0;
        end else if (ldbusy) begin
            ldcnt <= ldcnt + 16'd1;
            if (O_ld_abort) abort_seen <= 1'b1;
            if (abort_seen && abort_works && (ldcnt >= ABORT_LAT)) begin
                // 正常放弃：报 done+err，HD 槽内容记为"不可信"
                ld_done_r <= 1'b1;
                ld_err_r  <= 1'b1;
                ldbusy    <= 1'b0;
                if (cap_fmt) hd_any <= 1'b0;      // 画布作废（模型语义）
            end else if (ldcnt >= (cap_fmt ? LAT_H : LAT_T)) begin
                // 正常装完
                ld_done_r <= 1'b1;
                ldbusy    <= 1'b0;
                if (cap_fmt) begin
                    hd_off_r <= cap_off;
                    hd_any   <= 1'b1;
                end
            end
        end
        // V3：装载器忙时不许再发单（hd_take 必须先看 ld_busy）
        if (ld_req && ldbusy) ld_req_while_busy <= ld_req_while_busy + 1;
    end
end

//------------------------------------------------------------------------------
//  假帧源：compose_en=1 时每帧 500 拍（与 tb_switch 同款）
//------------------------------------------------------------------------------
integer fcnt;
always @(posedge clk or posedge rst) begin
    if (rst) begin ps_start <= 0; busy <= 0; fcnt <= 0; end
    else if (compose_en) begin
        if (ps_start) begin
            if (fcnt == 500) begin ps_start <= 0; busy <= 0; fcnt <= 0; end
            else fcnt <= fcnt + 1;
        end else if (!busy && fcnt == 0) fcnt <= -20;
        else if (fcnt < 0) begin
            fcnt <= fcnt + 1;
            if (fcnt == -1) begin ps_start <= 1; busy <= 1; end
        end
    end else begin
        ps_start <= 0; busy <= 0; fcnt <= 0;
    end
end

//------------------------------------------------------------------------------
//  计数与判据
//------------------------------------------------------------------------------
integer errors = 0;
integer tmo;
integer cur_abs;
reg [31:0] exp_off;

// V1：hd_valid 且 hd_img 指向当前 rel ⇒ HD 槽内容必须是当前这张
integer v1_err = 0;
always @(posedge clk) begin
    if (!rst && dut.hd_valid && (dut.hd_img == dut.u_sw.cr)) begin
        cur_abs = dut.u_sw.base + dut.u_sw.cr;
        if (hd_off_r !== off_of(cur_abs)) begin
            if (v1_err < 5)
                $display("  [FAIL] V1 旧画布被当成新图：hd_off=%h 期望 %h（base=%0d cr=%0d hd_img=%0d）t=%0t",
                         hd_off_r, off_of(cur_abs), dut.u_sw.base, dut.u_sw.cr, dut.hd_img, $time);
            v1_err = v1_err + 1;
        end
    end
end

// ★ [MM-EARLY]：每次换图都必须**在高清发单之前**武装一次 match（音乐/字幕定位）
integer n_arm = 0, arm_bad_rel = 0, arm_at_hd = 0;
always @(posedge clk) begin
    if (!rst && O_mm_arm) begin
        n_arm = n_arm + 1;
        if (O_mm_rel !== dut.u_sw.cr)  arm_bad_rel = arm_bad_rel + 1;  // 目标槽号 = 新图位置
        if (ld_req && ld_fmt)          arm_at_hd  = arm_at_hd  + 1;    // 武装早于高清发单
    end
end

task chk;
    input cond;
    input [255:0] msg;
    begin
        if (!cond) begin
            $display("  [FAIL] %0s", msg);
            errors = errors + 1;
        end else
            $display("  ok   %0s", msg);
    end
endtask

task wait_state;
    input [3:0] s;
    input integer limit;
    begin
        tmo = 0;
        while ((st_state !== s) && (tmo < limit)) begin
            @(posedge clk); tmo = tmo + 1;
        end
        if (st_state !== s) begin
            $display("  [FAIL] 等 state=%0d 超时（现在 %0d，%0d 拍）", s, st_state, limit);
            errors = errors + 1;
        end
    end
endtask

task wait_hd_busy;          // 等"高清装载在飞"
    input integer limit;
    begin
        tmo = 0;
        while (!(ldbusy && cap_fmt) && (tmo < limit)) begin
            @(posedge clk); tmo = tmo + 1;
        end
    end
endtask

task press_next;
    begin
        @(negedge clk); key_next = 1;
        @(negedge clk); key_next = 0;
    end
endtask

task press_prev;
    begin
        @(negedge clk); key_prev = 1;
        @(negedge clk); key_prev = 0;
    end
endtask

// 等回静止态并核对 HD 槽内容
task settle_chk;
    input integer exp_abs;
    input [255:0] tag;
    begin
        tmo = 0;
        while ((st_state != ST_HD) && (st_state != ST_STUK) && (tmo < 400_000)) begin
            @(posedge clk); tmo = tmo + 1;
        end
        repeat (80) @(posedge clk);
        if (st_state !== ST_HD) begin
            $display("  [FAIL] %0s：没回到 S_HD（state=%0d，等了 %0d 拍）", tag, st_state, tmo);
            errors = errors + 1;
        end else if (hd_off_r !== off_of(exp_abs)) begin
            $display("  [FAIL] %0s：HD 槽内容错 hd_off=%h 期望 %h（abs=%0d，实际 base+cr=%0d）",
                     tag, hd_off_r, off_of(exp_abs), exp_abs, dut.u_sw.base + dut.u_sw.cr);
            errors = errors + 1;
        end else if (ldbusy) begin
            $display("  [FAIL] %0s：静止态装载器还在忙", tag);
            errors = errors + 1;
        end else
            $display("  ok   %0s：abs=%0d hd_off=%h", tag, exp_abs, hd_off_r);
    end
endtask

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
integer n_abort_seen;
initial begin
    $display("== tb_abort_job：abort 语义 + 旧画布不许被接受 ==");
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (50) @(posedge clk);

    //--------------------------------------------------------------- S0 boot
    tmo = 0;
    while ((st_state != ST_HD) && (tmo < 400_000)) begin @(posedge clk); tmo = tmo + 1; end
    repeat (50) @(posedge clk);
    settle_chk(0, "S0 boot");

    //--------------------------------------------------------------- S1 正常 0→1
    $display("  -- S1 正常换图 0->1 --");
    press_next;
    settle_chk(1, "S1");

    //--------------------------------------------------------------- S2 正常 1→2
    $display("  -- S2 正常换图 1->2 --");
    press_next;
    settle_chk(2, "S2");

    // 回到 0（2->1->0）
    press_prev; settle_chk(1, "S2b");
    press_prev; settle_chk(0, "S2c");

    //------------------------------------------------ S3 abort 落地（装载器听话）
    $display("  -- S3 高清装载在飞时换图：abort 必须落地 --");
    press_next;                       // 0 -> 1，启动 HD 装载
    wait_hd_busy(60_000);
    if (!(ldbusy && cap_fmt)) begin
        $display("  [FAIL] S3 没能进入高清装载在飞的状态");
        errors = errors + 1;
    end else begin
        abort_works = 1;              // 装载器会听 abort
        n_abort_seen = 0;
        press_next;                   // 1 -> 2，打断在飞的 HD
        // 等 abort 电平（当前 RTL 是单拍；修好后应保持到 ld_fin）
        tmo = 0;
        while ((tmo < 200) && (n_abort_seen == 0)) begin
            @(posedge clk);
            if (O_ld_abort) n_abort_seen = 1;
            tmo = tmo + 1;
        end
        if (n_abort_seen == 0) begin
            $display("  [FAIL] S3 换图打断在飞装载，但 O_ld_abort 从未拉高");
            errors = errors + 1;
        end
        settle_chk(2, "S3 settle");
    end

    //------------------------------------------------ S4 abort 丢失（核心）
    //  装载器"没听 abort"⇒ 旧装载正常完成 err=0。当前 RTL 会：
    //    hd_valid<=1、hd_img(=1)==cur_rel(=1) ⇒ hd_ready=1
    //    ⇒ S_TRANS 结束时直接进 S_HDF，把 **图1 的画布** 当图2 显示。
    //  修好后（目标戳）：这次完成不得验证画布 ⇒ 必须重新装载 ⇒ HD 槽 = 图2。
    $display("  -- S4 abort 丢失：旧装载不许验证画布 --");
    press_prev; settle_chk(1, "S4a");     // 2 -> 1
    press_prev; settle_chk(0, "S4b");     // 1 -> 0
    v1_err = 0;                           // 本段单独计数
    press_next;                           // 0 -> 1
    wait_hd_busy(60_000);
    if (!(ldbusy && cap_fmt)) begin
        $display("  [FAIL] S4 没能进入高清装载在飞的状态");
        errors = errors + 1;
    end else begin
        abort_works = 0;                  // ← 模拟 abort 丢失/被忽略
        press_next;                       // 1 -> 2
        settle_chk(2, "S4 settle");
        abort_works = 1;
        if (v1_err != 0) begin
            $display("  [FAIL] S4 旧画布被接受 %0d 次（V1）—— abort 丢失后 err=0 的旧装载无效化失败",
                     v1_err);
            errors = errors + v1_err;
        end else
            $display("  ok   S4 旧画布未被接受（V1 0 次）");
    end

    //--------------------------------------------------------------- S5 V3
    $display("  -- S5 装载器忙时不得再发单（V3）--");
    if (ld_req_while_busy != 0) begin
        $display("  [FAIL] S5 出现 %0d 次忙时发单", ld_req_while_busy);
        errors = errors + ld_req_while_busy;
    end else
        $display("  ok   S5 没有忙时发单");

    //----------------------------------------------------------- S6 再走一遍
    $display("  -- S6 收尾回归：再来回切几次 --");
    press_prev; settle_chk(1, "S6a");
    press_next; settle_chk(2, "S6b");
    press_prev; settle_chk(1, "S6c");

    //------------------------------------------ S7 换图即武装 match（音乐/字幕提前）
    $display("  -- S7 arm match early--");
    chk(n_arm >= 8,       "S7 one arm per switch");
    chk(arm_bad_rel == 0, "S7 arm rel = new cur_rel");
    chk(arm_at_hd == 0,   "S7 arm before HD issue");
    $display("       （武装 %0d 次，槽号错 %0d 次，晚于高清发单 %0d 次）",
             n_arm, arm_bad_rel, arm_at_hd);

    $display("== errors=%0d %s ==", errors, (errors == 0) ? "PASS" : "FAIL");
    $finish;
end

initial begin
    #200_000_000;
    $display("TIMEOUT");
    $display("== errors=%0d FAIL ==", errors + 1);
    $finish;
end

endmodule
