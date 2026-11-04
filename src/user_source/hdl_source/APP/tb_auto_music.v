//==============================================================================
//  tb_auto_music.v —— 「自动切换 = 音乐结束」事件口径回归（★ 2026-11-08）
//==============================================================================
//
//  【要证的事】（对应 viewer_ctrl 的 mus_pend 锁存）
//    M1  S_HD 里收到一次“音乐播完” ⇒ 恰好切一张
//    M2  再来一次 ⇒ 再切一张（可重复）
//    M3  自动切换=关 时收到事件 ⇒ 不切
//    M4  菜单（S_OSD）里收到两次事件 ⇒ 菜单里不切；退出后**只补切一张**
//    M5  过渡（S_ARM）里收到事件 ⇒ 过渡中途不切；回到 S_HD 后补切一张
//    M1b/M2b/M5c 单次事件后计数保持稳定 ⇒ mus_pend 只点火一拍，不会连切
//
//  【跑法】工作目录 = APP/
//    iverilog -g2012 -s tb_auto_music -o /tmp/am.vvp tb_auto_music.v \
//        viewer_ctrl.v img_switch.v viewer_fx.v && vvp /tmp/am.vvp
//==============================================================================
`timescale 1ns / 1ps

module tb_auto_music;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

// ---- 激励 ----
reg  key_prev = 0, key_next = 0, key_fx = 0;
reg  key_osd_enter = 0, key_osd_exit = 0;
reg  busy = 0, ps_start = 0, pipe_ce = 1;
reg  ld_can_load = 1, ld_done = 0, ld_err = 0;
reg  [2:0] fx_sel = 3'd0;
reg  [2:0] auto_mode = 3'd0;
reg  zoom_on = 0, preview_req = 0, load_req = 0;
reg  music_done = 0;            // 顶层 music_tgl：每遍播完翻转一次
reg  alb_ready = 1;             // ★ 相册重扫模型：换图后拉低 ~50 拍再放行
integer rdy_cnt = 0;

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
wire        sw_go_w, info_show, osd_on;

viewer_ctrl #(
    .T_END       (300),
    .T_END_F     (60),
    .FRAME_CYC   (835000),
    .AUTO_SEC_CYC(400),
    .THUMB0_ADDR (21'd614400), .THUMB1_ADDR (21'd652800),
    .THUMB2_ADDR (21'd691200), .HD_ADDR     (21'd729600)
) dut (
    .clk(clk), .rst(rst),
    .I_img_off0(32'd1000), .I_img_off1(32'd2000), .I_img_off2(32'd3000),
    .I_img_off3(32'd4000),
    .O_base(), .I_win4(1'b1), .I_any(1'b1), .I_nimg(3'd4),
    .I_alb_ready(alb_ready),
    .I_jump(1'b0), .I_jump_idx(8'd0),
    .I_file_page(1'b0),
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(key_osd_enter), .key_osd_exit(key_osd_exit), .osd_fade(8'd255),
    .osd_on(osd_on), .osd_allow(), .zoom_allow(),
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
    .fx_sel(fx_sel), .auto_mode(auto_mode),.info_on(1'b0),
    .album_prev(1'b0), .album_next(1'b0), .osd_trig(1'b0), .music_done(music_done),
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
//  相册重扫模型：真实 album_src 在换图/滑窗后会把 ready 拉低再抬起。
//  本台若恒接 1，img_switch 的 bpend（滑窗后等重扫）永远不会清 ⇒ 第二刀之后
//  HD 装载发不出去，状态机停在 S_LOW（tb_auto_sw 只切两刀所以没暴露）。
//------------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        alb_ready <= 1'b1;
        rdy_cnt   <= 0;
    end else if (sw_go_w) begin
        alb_ready <= 1'b0;
        rdy_cnt   <= 0;
    end else if (!alb_ready) begin
        if (rdy_cnt == 50) alb_ready <= 1'b1;
        else               rdy_cnt   <= rdy_cnt + 1;
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
            // ★ 必须连 fcnt 一起清：OSD 期间 compose_en=0 停在半帧，
            //   只清 ps_start/busy 会留下 fcnt=900 这种“非 0 非负”的中间值，
            //   退出菜单后帧源三个分支全不进 ⇒ S_HDF 的三相握手永远等不到 busy。
            ps_start <= 0; busy <= 0; fcnt <= 0;
        end
    end
end

//------------------------------------------------------------------------------
//  观测
//------------------------------------------------------------------------------
integer go_cnt = 0;
integer errors = 0;
always @(posedge clk) begin
    if (!rst && sw_go_w) go_cnt = go_cnt + 1;
end

// 状态/事件跟踪（调试用；只看跳变）
`ifdef AM_TRACE
integer st_prev = -1;
integer dbg_cyc = 0;
always @(posedge clk) begin
    if (!rst) begin
        dbg_cyc = dbg_cyc + 1;
        if ((dbg_cyc % 200000) == 0)
            $display("      [dbg] %0t st=%0d busy=%b ps=%b ce=%b fcnt=%0d hdf_ph=%0d pend=%b fire=%b",
                     $time, st_state, busy, ps_start, compose_en, fcnt,
                     dut.hdf_ph, dut.mus_pend, dut.music_fire);
        if (st_state !== st_prev) begin
            $display("      [st] %0t %0d -> %0d  busy=%b ce=%b alb=%b pend=%b fire=%b",
                     $time, st_prev, st_state, busy, compose_en, alb_ready,
                     dut.mus_pend, dut.music_fire);
            st_prev = st_state;
        end
    end
end
`endif

task chk;
    input [8*48:1] name;
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

task settle;
    input integer n;
    integer k;
    begin
        for (k = 0; k < n; k = k + 1) @(posedge clk);
    end
endtask

task wait_state;
    input integer s;
    integer g;
    begin
        g = 0;
        while ((st_state != s[3:0]) && (g < 4_000_000)) begin
            @(posedge clk); g = g + 1;
        end
        if (st_state != s[3:0]) begin
            errors = errors + 1;
            $display("*** TIMEOUT wait_state(%0d): now state=%0d", s, st_state);
        end
    end
endtask

task wait_go;
    input integer n;
    integer g;
    begin
        g = 0;
        while ((go_cnt < n) && (g < 4_000_000)) begin
            @(posedge clk); g = g + 1;
        end
        if (go_cnt < n) begin
            errors = errors + 1;
            $display("*** TIMEOUT wait_go(%0d): now go_cnt=%0d", n, go_cnt);
        end
    end
endtask

// 一次“音乐播完”：music_tgl 翻转（DUT 内部两拍同步后按**任意跳变**算事件）
task music_end_once;
    begin
        @(negedge clk); music_done = ~music_done;
    end
endtask

//------------------------------------------------------------------------------
//  主场景
//------------------------------------------------------------------------------
integer base_go;
initial begin
    repeat (20) @(posedge clk);
    rst = 0;

    wait_state(5);
    $display("=== M0 上电到 S_HD ===");
    chk("M0 in S_HD", st_state, 5);

    //------------------------------------------------------------------
    //  M1：S_HD 下收到一次事件 ⇒ 恰好切一张
    //------------------------------------------------------------------
    $display("=== M1 音乐结束 -> 切一张 ===");
    @(negedge clk); auto_mode = 3'd5;
    settle(20);
    music_end_once;
    wait_go(1);
    chk("M1 one event -> one switch", go_cnt, 1);
    wait_state(5);
    settle(5000);
    chk("M1b no extra switch", go_cnt, 1);

    //------------------------------------------------------------------
    //  M2：可重复
    //------------------------------------------------------------------
    $display("=== M2 第二次事件 -> 再切一张 ===");
    music_end_once;
    wait_go(2);
    chk("M2 second event -> second switch", go_cnt, 2);
    wait_state(5);
    settle(5000);
    chk("M2b no extra switch", go_cnt, 2);

    //------------------------------------------------------------------
    //  M3：自动切换=关 时不响应
    //------------------------------------------------------------------
    $display("=== M3 关闭自动切换 -> 不切 ===");
    @(negedge clk); auto_mode = 3'd0;
    settle(20);
    music_end_once;
    settle(20000);
    chk("M3 mode off ignores music", go_cnt, 2);
    @(negedge clk); auto_mode = 3'd5;
    settle(20);

    //------------------------------------------------------------------
    //  M4：菜单里收到两次事件 ⇒ 菜单里不切；退出后只补切一张
    //------------------------------------------------------------------
    $display("=== M4 菜单里结束 -> 退出后只补切一张 ===");
    wait_state(5);
    @(negedge clk); key_osd_enter = 1;
    @(negedge clk); key_osd_enter = 0;
    wait_state(8);
    chk("M4 in S_OSD", st_state, 8);
    music_end_once;
    settle(500);
    music_end_once;             // 两次事件应在退出后合并成一次
    settle(2000);
    chk("M4b no switch while menu open", go_cnt, 2);
    @(negedge clk); key_osd_exit = 1;
    @(negedge clk); key_osd_exit = 0;
    wait_go(3);
    chk("M4c exactly one catch-up switch", go_cnt, 3);
    wait_state(5);
    settle(5000);
    chk("M4d no double catch-up", go_cnt, 3);

    //------------------------------------------------------------------
    //  M5：过渡里结束 ⇒ 过渡中不切；回到 S_HD 后补切一张
    //------------------------------------------------------------------
    $display("=== M5 过渡里结束 -> 回 S_HD 后补切一张 ===");
    base_go = go_cnt;
    @(negedge clk); key_next = 1;
    @(negedge clk); key_next = 0;
    wait_go(base_go + 1);       // key_next 自己的一刀
    wait_state(7);              // S_ARM
    music_end_once;
    settle(2000);
    chk("M5 no switch during transition", go_cnt, base_go + 1);
    wait_state(5);
    wait_go(base_go + 2);       // 过渡结束后的补刀
    chk("M5b one catch-up after transition", go_cnt, base_go + 2);
    wait_state(5);
    settle(5000);
    chk("M5c no extra switch", go_cnt, base_go + 2);

    $display("");
    if (errors == 0) $display("*** ALL PASS");
    else             $display("*** %0d FAIL", errors);
    $finish;
end

endmodule
