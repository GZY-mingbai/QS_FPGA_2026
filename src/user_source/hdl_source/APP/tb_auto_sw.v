//==============================================================================
//  tb_auto_sw.v —— 自动切换"读秒口径"专项回归（★ 2026-11-02）
//==============================================================================
//
//  【要证的事】
//    A1 到了 S_HD、auto_mode=3s ⇒ **3 秒后**真的换图（sw_go）。
//    A2 换完图进入过渡（S_ARM/S_TRANS/S_LOW）之后，倒计时**继续往下走**
//       —— 这是本次改动的核心：改造前它被钉在满值上，要等高清上屏才开始掉。
//    A3 换图那一拍倒计时**归零重来**（sw_go 清零；改造前是靠"切图必进 S_ARM
//        ⇒ auto_run 落下"顺带清零，现在 S_ARM 也在计时窗口里，必须显式清）。
//    A4 动画里就到期的档位（这里故意把 AUTO_SEC_CYC 压到 400 拍 = 1 "秒"，
//       而过渡是 60 帧 ≈ 7.8 万拍）⇒ auto_sec 必须**饱和**：高清一上屏，
//       下一个整秒就点火，**不会**因为等号早被越过而永久失效。
//
//  【跑法】工作目录 = APP/
//    iverilog -g2012 -s tb_auto_sw -o /tmp/as.vvp tb_auto_sw.v viewer_ctrl.v \
//        img_switch.v viewer_fx.v && vvp /tmp/as.vvp | grep -aE "FAIL|ALL PASS"
//==============================================================================
`timescale 1ns / 1ps

module tb_auto_sw;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

// ---- 激励 ----
reg  key_prev = 0, key_next = 0, key_fx = 0;
reg  busy = 0, ps_start = 0, pipe_ce = 1;
reg  ld_can_load = 1, ld_done = 0, ld_err = 0;
reg  [2:0] fx_sel = 3'd0;
reg  [2:0] auto_mode = 3'd0;
reg  zoom_on = 0, preview_req = 0, load_req = 0;

// ---- DUT ----
wire ld_req, ld_fmt;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
wire compose_en, fmt565, a_hd, a_en, b_en;
wire [20:0] a_base, b_base;
wire [31:0] pos_a, size_a, pos_b, size_b;
wire [39:0] k_a, k_b;
wire [7:0]  alpha_b;
wire [1:0]  st_img, cur_rel_w;
wire [2:0]  st_fx;
wire [3:0]  st_state, dbg_state;
wire [15:0] res_w, res_h;
wire [5:0]  auto_remain;
wire [8:0]  auto_bar;
wire        sw_go_w, info_show;

localparam integer AUTO_CYC = 400;   // 1 "秒" = 400 拍（只给仿真压缩，板上是 50M）

viewer_ctrl #(
    .T_END       (300),
    .T_END_F     (60),              // fx0：60 帧 ≈ 7.8 万拍（远大于 3 "秒"）
    .FRAME_CYC   (835000),
    .AUTO_SEC_CYC(AUTO_CYC),
    .THUMB0_ADDR (21'd614400), .THUMB1_ADDR (21'd652800),
    .THUMB2_ADDR (21'd691200), .HD_ADDR     (21'd729600)
) dut (
    .clk(clk), .rst(rst),
    .I_img_off0(32'd1000), .I_img_off1(32'd2000), .I_img_off2(32'd3000),
    .I_img_off3(32'd4000),
    .O_base(), .I_win4(1'b1), .I_any(1'b1), .I_nimg(3'd4),
    .I_alb_ready(1'b1),
    .I_jump(1'b0), .I_jump_idx(8'd0),
    .I_file_page(1'b0),
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(1'b0), .key_osd_exit(1'b0), .osd_fade(8'd255),
    .osd_on(), .osd_allow(), .zoom_allow(),
    .zoom_on(zoom_on), .preview_req(preview_req), .load_req(load_req),
    .zoom_val(8'h10), .za_x(10'd320), .za_y(10'd240),
    .busy(busy), .ps_start(ps_start), .pipe_ce(pipe_ce),
    .ld_req(ld_req), .ld_fmt(ld_fmt), .ld_off(ld_off), .ld_len(ld_len),
    .ld_base(ld_base),
    .ld_can_load(ld_can_load), .ld_done(ld_done), .ld_err(ld_err),
    .ld_sd_ready(1'b1), .O_ld_abort(),
    .O_hd_rel(), .O_cur_rel(cur_rel_w), .O_sw_go(sw_go_w),
    .compose_en(compose_en), .fps_dbg(), .fmt565(fmt565),
    .a_hd(a_hd), .a_en(a_en), .a_base(a_base),
    .b_en(b_en), .b_base(b_base),
    .pos_a(pos_a), .size_a(size_a), .k_a(k_a),
    .pos_b(pos_b), .size_b(size_b), .k_b(k_b), .alpha_b(alpha_b),
    .st_img(st_img), .st_fx(st_fx), .st_state(st_state),
    .fx_sel(fx_sel), .auto_mode(auto_mode),.info_on(1'b1),
    .album_prev(1'b0), .album_next(1'b0), .osd_trig(1'b0), .music_done(1'b0),
    .ld_src_w(16'd640), .ld_src_h(16'd480),
    .res_w(res_w), .res_h(res_h),
    .auto_remain(auto_remain), .auto_bar(auto_bar), .info_show(info_show),
.ld_zoom(), .ld_ax(), .ld_ay()
);

//------------------------------------------------------------------------------
//  假装载器：收到 ld_req 后 200 拍回 done
//------------------------------------------------------------------------------
integer ldcnt = 0;
reg     ldbusy = 0;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        ld_done <= 0; ldcnt <= 0; ldbusy <= 0;
    end else begin
        ld_done <= 0;
        if (ld_req) begin
            ldbusy <= 1; ldcnt <= 0;
        end else if (ldbusy) begin
            ldcnt <= ldcnt + 1;
            if (ldcnt == 200) begin ld_done <= 1; ldbusy <= 0; end
        end
    end
end

//------------------------------------------------------------------------------
//  假帧源：compose_en=1 时周期性地给 ps_start（1000 拍一帧）
//------------------------------------------------------------------------------
integer fcnt;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        ps_start <= 0; busy <= 0; fcnt <= 0;
    end else begin
        if (compose_en) begin
            if (ps_start) begin
                if (fcnt == 1000) begin
                    ps_start <= 0; busy <= 0; fcnt <= 0;
                end else fcnt <= fcnt + 1;
            end else if (!busy && fcnt == 0) begin
                fcnt <= -300;
            end else if (fcnt < 0) begin
                fcnt <= fcnt + 1;
                if (fcnt == -1) begin ps_start <= 1; busy <= 1; end
            end
        end else begin
            ps_start <= 0; busy <= 0;
        end
    end
end

//------------------------------------------------------------------------------
//  观测
//------------------------------------------------------------------------------
integer cyc = 0;
integer t_set = 0, t_go1 = 0, t_go2 = 0;
// 过渡/装载期间（S_ARM=7 / S_TRANS=3 / S_LOW=2）出现过的 auto_remain 值（位图）
integer rem_mask = 0;
integer rem_after_go = -1;      // sw_go 之后一拍的 auto_remain
integer n_go = 0;
integer errors = 0;
integer st_prev = -1;
reg  [2:0] go_d = 3'b000;

always @(posedge clk) begin
    if (!rst) begin
        cyc = cyc + 1;

        // 设置 auto_mode 的时刻（只记第一次）
        if ((auto_mode == 3'd1) && (t_set == 0)) t_set = cyc;

        // 换图脉冲
        if (sw_go_w) begin
            n_go = n_go + 1;
            if (n_go == 1) t_go1 = cyc;
            if (n_go == 2) t_go2 = cyc;
        end
        // sw_go 之后一拍看倒计时（应该已经归零重来）。用移位寄存器而不是算拍数：
        //  本台有两个 always 都在 posedge 上跑，`cyc` 的可见时刻会错开一拍。
        go_d = {go_d[1:0], sw_go_w};
        if (go_d[1]) rem_after_go = auto_remain;

        // ★ 过渡（S_ARM/S_TRANS）与装高清（S_LOW）期间，倒计时必须真的在走。
        //   改造前这三个状态都在计时窗口外 ⇒ auto_remain 恒为 auto_total（3）。
        if ((st_state == 4'd7) || (st_state == 4'd3) || (st_state == 4'd2)) begin
            if (auto_remain <= 6'd5) rem_mask = rem_mask | (1 << auto_remain);
        end

        if (st_state !== st_prev) begin
            $display("      [state] %0d -> %0d  (cyc=%0d remain=%0d)", st_prev, st_state, cyc, auto_remain);
            st_prev = st_state;
        end
    end
end

task chk;
    input [8*40:1] name;
    input integer  got;
    input integer  exp;
    begin
        if (got !== exp) begin
            errors = errors + 1;
            $display("*** FAIL  %0s : got %0d, expect %0d", name, got, exp);
        end else
            $display("  ok     %0s : %0d", name, got);
    end
endtask

//------------------------------------------------------------------------------
//  主场景
//------------------------------------------------------------------------------
integer guard;
initial begin
    repeat (20) @(posedge clk);
    rst = 0;

    // 等上电序列走到静止高清（S_HD）
    guard = 0;
    while ((st_state != 4'd5) && (guard < 400000)) begin
        @(posedge clk); guard = guard + 1;
    end
    $display("--- 0. 上电完成：state=%0d（5=S_HD），用了 %0d 拍 ---", st_state, guard);
    chk("0  boot reached S_HD", st_state, 5);

    // 打开自动切换 3s 档
    @(negedge clk);
    auto_mode = 3'd1;

    // A1：第一次换图应该在 3 秒（3*400 拍）+ 一点余量内发生
    guard = 0;
    while ((n_go == 0) && (guard < 20 * AUTO_CYC)) begin
        @(posedge clk); guard = guard + 1;
    end
    chk("A1  auto switch fired", (n_go >= 1) ? 1 : 0, 1);
    $display("      t_set=%0d t_go1=%0d  ⇒ 间隔 %0d 拍（3 秒 = %0d 拍）",
             t_set, t_go1, t_go1 - t_set, 3 * AUTO_CYC);
    chk("A1b interval ~= 3s", ((t_go1 - t_set) > (AUTO_CYC / 2)) &&
                              ((t_go1 - t_set) <= (4 * AUTO_CYC)) ? 1 : 0, 1);

    // A3：换图那一拍倒计时归零重来（= 满值 3）
    //   ★ 必须等 3 拍：`n_go` 是与 sw_go 同一拍置的，chk 不能在同一步读采样值。
    repeat (3) @(posedge clk);
    chk("A3  countdown restarted at sw", rem_after_go, 3);
    begin : dbg1
        integer k;
        for (k = 0; k < 6; k = k + 1) begin
            @(posedge clk);
            $display("      [dbg] cyc=%0d st=%0d run=%b cnt=%0d sec=%0d rem=%0d remain=%0d tick=%b exp=%b",
                     cyc, st_state, dut.auto_run, dut.auto_cnt, dut.auto_sec,
                     dut.auto_rem, auto_remain, dut.auto_tick, dut.auto_exp);
        end
    end

    // 等过渡跑完（fx0 = 60 帧 ≈ 7.8 万拍）并等第二次点火
    guard = 0;
    while ((n_go < 2) && (guard < 300000)) begin
        @(posedge clk); guard = guard + 1;
    end
    $display("--- 1. 过渡/装载期间出现过的 remain 值 = %b（bit k = 显示过 k）---", rem_mask);
    // A2：过渡期间倒计时必须往下走（改造前恒为 3 ⇒ 只会出现 bit3）
    chk("A2  countdown ticks during transition", ((rem_mask & ~(1 << 3)) != 0) ? 1 : 0, 1);
    chk("A2b countdown reached 0",               (rem_mask & 1) ? 1 : 0, 1);

    // A4：动画里就到期 ⇒ 高清一上屏下一个整秒就点火（不会永久失效）
    chk("A4  auto switch fired again after HD", (n_go >= 2) ? 1 : 0, 1);
    if (t_go2 != 0)
        $display("      t_go2=%0d（第一次之后 %0d 拍）", t_go2, t_go2 - t_go1);

    // A5：换图之后状态机仍在正常轨道上（S_HD=5 / S_ARM=7 / S_TRANS=3 之一）
    chk("A5  state still healthy", ((st_state == 4'd5) || (st_state == 4'd7) ||
                                     (st_state == 4'd3)) ? 1 : 0, 1);

    $display("");
    if (errors == 0) $display("*** ALL PASS");
    else             $display("*** %0d FAIL", errors);
    $finish;
end

endmodule
