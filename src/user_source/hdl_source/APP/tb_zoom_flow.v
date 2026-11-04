`timescale 1ns/1ps
//==============================================================================
//  tb_zoom_flow.v —— 缩放"冷却时间"端到端测量台
//==============================================================================
//  【要回答的问题】上板现象：按下缩放 → 动画播完 → 888 加载完成 →
//    **还要等一会**才能继续缩放。到底是哪一段在拖？
//
//  【做法】把 zoom_osd + viewer_ctrl + 假装载器串起来，用拍数计时：
//      T1  按 KEY3（放大）→ zph 变 1（565 预览出现）
//      T2  zph 回到 0 且 a_hd=1（888 清晰图回来）
//      T3  T2 之后**立刻**再按 KEY3 → zph 再次变 1 用了几拍   ← 这就是"冷却"
//  只要 T3 远大于 1~2 帧，就说明卡在 viewer_ctrl 的某个 pending/状态上；
//  如果 T3 只有 1~2 拍，那"冷却"其实来自 zoom_osd 的 500ms 防抖，
//  也就是用户感知到的是"等防抖 + 等装载"，不是按不动。
//==============================================================================
module tb_zoom_flow;

reg clk = 0;
always #10 clk = ~clk;              // 50MHz

reg rst = 1;

// ---- 按键（单拍）----
reg key_menu = 0, key_next = 0, key_prev = 0, key_fx = 0;
reg key_osd_enter = 0, key_osd_exit = 0;

// ---- 先声明给 zoom_osd 用的两个反馈（必须在例化之前）----
wire        osd_allow;
reg         ld_done_r;

// ---- zoom_osd ----
wire        oz_on;
wire [7:0]  oz_zoom;
wire [9:0]  oz_ax, oz_ay;
wire [1:0]  z_mode;
wire        z_refresh, z_prev_req, z_load;
wire        z_act_go;
wire [1:0]  z_act_sel;
wire        z_rest;
wire [7:0]  z_zoom_val = oz_zoom;
wire        z_act_code = 1'b0;

zoom_osd #(.CLK_HZ(50_000_000), .DB_MS(1)) u_zoom (   // 1ms 加速
    .clk(clk), .rst(rst),
    .key_press  ({key_prev, key_next, key_fx, key_menu}),
    .menu_on    (1'b0),
    .allow      (osd_allow),
    .ld_done    (ld_done),
    .hd_shown   (a_hd & ~fmt565),    // ★ [ZOOM-SEQ] 真实信号
    .act_code   ({1'b0, z_act_code}),
    .zoom_on    (oz_on),
    .zoom       (oz_zoom),
    .ax         (oz_ax),
    .ay         (oz_ay),
    .mode       (z_mode),
    .refresh    (z_refresh),
    .preview_req(z_prev_req),
    .load_req   (z_load),
    .act_go     (z_act_go),
    .act_sel    (z_act_sel),
    .restore    (z_rest)
);

// ---- viewer_ctrl ----
wire        compose_en, a_en, b_en;
wire [20:0] a_base, b_base;
wire [31:0] pos_a, size_a, pos_b, size_b;
wire [39:0] k_a, k_b;
wire [7:0]  alpha_b;
wire [1:0]  st_img;
wire [2:0]  st_fx;
wire [3:0]  st_state;
wire        busy, ps_start, pipe_ce;
wire        ld_req, ld_err = 1'b0, ld_can_load, ld_sd_ready = 1'b1;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
wire        ld_fmt;
wire        ld_abort;               // ★ [ZOOM-RESTEP] 缩放中途换档 ⇒ 砍掉在飞烘焙
integer     n_ab = 0;
always @(posedge clk) if (ld_abort) n_ab <= n_ab + 1;

// ★ 2026-11-02 [NO-MIDFRAME] 监视：fmt565 / a_hd 只允许在**帧间**（ps_start=0）变化。
//   这两个量直接决定当前帧的写格式与取源，扫描中途改 ⇒ 这一帧一半 888 一半 565，
//   上板就是"按放大先闪一下黑"。
reg fmt_d = 0, ahd_d = 0;
integer n_tear = 0;
always @(posedge clk) begin
    fmt_d <= fmt565; ahd_d <= a_hd;
    if (!rst && ps_start && ((fmt565 != fmt_d) || (a_hd != ahd_d))) begin
        n_tear <= n_tear + 1;
        $display("  *** [NO-MIDFRAME] 扫描中途改了 fmt565/a_hd（t=%0t）", $time);
    end
end

viewer_ctrl #(
    .THUMB0_ADDR(21'd614400), .THUMB1_ADDR(21'd652800),
    .THUMB2_ADDR(21'd691200), .HD_ADDR(21'd729600),
    .ARM_FRAMES(1)
) dut (
    .clk(clk), .rst(rst),
    // ★ 2026-10-31：这组相册端口原来误接在 u_zoom 上（zoom_osd 没有这些口 ⇒ 编译不过），
    //   它们属于 viewer_ctrl。偏移给固定值；I_win4=0 ⇒ 不滑窗（本台只关心缩放几何）。
    .I_img_off0 (32'd1000), .I_img_off1 (32'd2000), .I_img_off2 (32'd3000),
    .I_img_off3 (32'd0), .I_any(1'b1), .I_nimg(3'd3), .I_alb_ready(1'b1), .I_jump(1'b0), .I_jump_idx(8'd0), .O_ld_abort(ld_abort), .I_win4(1'b0),
    .I_file_page(1'b0),              // ★ [FP-ABORT] 本台不测文件页
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(key_osd_enter), .key_osd_exit(key_osd_exit), .osd_fade(8'd255),
    .osd_on(), .osd_allow(osd_allow),
    .zoom_on(oz_on), .preview_req(z_prev_req), .load_req(z_load),
    .zoom_val(z_zoom_val),
    .za_x(10'd320), .za_y(10'd240),
    .busy(busy), .ps_start(ps_start), .pipe_ce(pipe_ce),
    .ld_req(ld_req), .ld_fmt(ld_fmt), .ld_off(ld_off), .ld_len(ld_len), .ld_base(ld_base),
    .ld_can_load(ld_can_load), .ld_done(ld_done), .ld_err(ld_err),
    .ld_sd_ready(ld_sd_ready),
    .compose_en(compose_en),
    .fmt565(fmt565),
    .a_hd(a_hd), .a_en(a_en), .a_base(a_base),
    .b_en(b_en), .b_base(b_base),
    .pos_a(pos_a), .size_a(size_a), .k_a(k_a),
    .pos_b(pos_b), .size_b(size_b), .k_b(k_b), .alpha_b(alpha_b),
    .st_img(st_img), .st_fx(st_fx), .st_state(st_state),
    .fx_sel(3'd0), .auto_mode(3'd0),.info_on(1'b0),
    .album_prev(1'b0), .album_next(1'b0), .osd_trig(1'b0), .music_done(1'b0),
    .ld_src_w(16'd640), .ld_src_h(16'd480),
    .res_w(), .res_h(), .auto_remain(), .auto_bar(),
    .info_show(),    .fps_dbg()
);

//------------------------------------------------------------------------------
//  假帧源（compose_en 时每帧 ps_start 1000 拍）
//------------------------------------------------------------------------------
reg     ps_start_r = 0, busy_r = 0;
integer fcnt = 0;
assign ps_start = ps_start_r;
assign busy     = busy_r;
assign pipe_ce  = 1'b1;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        ps_start_r <= 0; busy_r <= 0; fcnt <= 0;
    end else if (compose_en) begin
        if (ps_start_r) begin
            if (fcnt == 1000) begin ps_start_r <= 0; busy_r <= 0; fcnt <= 0; end
            else fcnt <= fcnt + 1;
        end else if (!busy_r && fcnt == 0) begin
            fcnt <= -300;
        end else if (fcnt < 0) begin
            fcnt <= fcnt + 1;
            if (fcnt == -1) begin ps_start_r <= 1; busy_r <= 1; end
        end
    end else begin
        ps_start_r <= 0; busy_r <= 0; fcnt <= 0;
    end
end

//------------------------------------------------------------------------------
//  假装载器：ld_req 后 300 拍回 done（模拟 SD→SDRAM 的一小段）
//------------------------------------------------------------------------------
reg     ldbusy = 0;
integer ldcnt = 0;
always @(posedge clk or posedge rst) begin
    if (rst) begin ld_done_r <= 0; ldcnt <= 0; ldbusy <= 0; end
    else begin
        ld_done_r <= 0;
        if (ld_req) begin ldbusy <= 1; ldcnt <= 0; end
        else if (ldbusy) begin
            ldcnt <= ldcnt + 1;
            if (ldcnt == 300) begin ld_done_r <= 1; ldbusy <= 0; end
        end
    end
end
assign ld_done      = ld_done_r;
assign ld_can_load  = ~ldbusy;

//------------------------------------------------------------------------------
//  工具
//------------------------------------------------------------------------------
integer cyc = 0;
always @(posedge clk) cyc <= cyc + 1;

task press;
    input [3:0] k;      // {k4,k3,k2,k1} = {prev,next,fx,menu}
    begin
        @(negedge clk); {key_prev, key_next, key_fx, key_menu} = k;
        @(negedge clk); {key_prev, key_next, key_fx, key_menu} = 4'd0;
    end
endtask

// 等某个条件成立，返回用掉的拍数（上限 to）
task wait_cyc;
    input integer to;
    output integer n;
    reg cond;
    begin
        n = 0;
        cond = 1'b0;
        while (!cond && (n < to)) begin
            @(negedge clk); n = n + 1;
            cond = (dut.zph == 2'd0) && a_hd;     // "888 清晰图在显示"
        end
    end
endtask

integer c_start, c_sharp, c_next, t, c_t3 = 0;
integer i;
reg [7:0] zoom_at_sharp;

initial begin
    $display("");
    $display("===== tb_zoom_flow : 缩放冷却时间测量（1 帧 ≈ 1300 拍）=====");
    repeat (20) @(negedge clk);
    rst = 0;
    t = 0;
    while ((st_state != 4'd5) && (t < 300000)) begin @(negedge clk); t = t + 1; end
    $display("  上电到 S_HD 用了 %0d 拍 (state=%0d)", t, st_state);
    chk_state("A 上电进 S_HD", st_state == 4'd5);
    repeat (50) @(negedge clk);

    // 打开缩放 OSD（KEY2 = key_fx）
    press(4'b0010);
    repeat (20) @(negedge clk);
    $display("  打开缩放: oz_on=%b st=%0d zph=%0d", oz_on, dut.state, dut.zph);

    // 切到 ZOOM 档（mode 0 → 1 → 2）
    press(4'b0010);
    repeat (20) @(negedge clk);
    press(4'b0010);
    repeat (20) @(negedge clk);
    $display("  模式: mode=%0d (期望 2=ZOOM)", z_mode);

    // ---- 第一次放大：量 T1 ----
    // ★ 按键相位对齐到"帧刚开始"，把"帧内切换"这个错暴露出来（否则可能正好落在
    //   帧间空隙里，测不到）。
    @(posedge ps_start);
    c_start = cyc;
    press(4'b0100);                          // KEY3 = key_next，放大
    // 等预览出现（zph==1）
    t = 0;
    while ((dut.zph != 2'd1) && (t < 20000)) begin @(negedge clk); t = t + 1; end
    $display("  T1 按下→565预览出现: %0d 拍", t);

    // 等 888 清晰图回来
    c_sharp = 0;
    t = 0;
    while (!((dut.zph == 2'd0) && a_hd) && (t < 200000)) begin
        @(negedge clk); t = t + 1;
    end
    c_sharp = cyc;
    zoom_at_sharp = oz_zoom;
    $display("  T2 按下→888 清晰图回来: %0d 拍（zoom=%h）", cyc - c_start, oz_zoom);

    // ---- 立刻再按一次，量 T3（这就是"冷却"）----
    c_start = cyc;
    press(4'b0100);                          // KEY3 再放大
    t = 0;
    while ((dut.zph != 2'd1) && (t < 200000)) begin @(negedge clk); t = t + 1; end
    $display("  T3 清晰图后立刻再按→565预览出现: %0d 拍", t);
    c_t3 = t;
    $display("      （此时 zoom=%h → 期望 %h）", oz_zoom, (zoom_at_sharp == 8'h10) ? 8'h20 : 8'h40);

    // ---- 第二次缩放必须真的走到新档位（图片真的变成 4×）----
    t = 0;
    while (!((dut.zph == 2'd0) && a_hd) && (t < 300000)) begin @(negedge clk); t = t + 1; end
    $display("  T4 第二次按→888 清晰图回来: %0d 拍  zoom=%h  k_a=%h",
             t, oz_zoom, k_a);
    if (oz_zoom != 8'h40) $display("     *** 第二次缩放没生效：zoom 还是 %h", oz_zoom);
    // 注意：zph==0 时 k_a 恒为 1:1（65536），缩放是由 bmp_view 画进画布的，
    // 所以这里只看 zoom 档位，不看 k_a。

    //------------------------------------------------------------------
    // ★ [ZOOM-RESTEP] 装载中再按一档：必须立刻放弃在飞的那次烘焙
    //------------------------------------------------------------------
    $display("");
    c_start = cyc;
    press(4'b0100);                          // 4x -> 8x，起烘焙
`ifdef ZF_DEBUG
    begin : dbg2
        integer j;
        for (j = 0; j < 40; j = j + 1) begin
            @(negedge clk);
            $display("   [T5] st=%0d zph=%0d ld_pend=%b want=%b ldp=%0d busy=%b can=%b take=%b hdp=%b tgt=%b rdy=%b zv=%0d hzs=%0d",
                     dut.state, dut.zph, dut.ld_pend, dut.hd_want, dut.ldp, ldbusy, ld_can_load,
                     dut.hd_take, dut.hd_pend, dut.ld_tgt_ok, dut.hd_ready,
                     dut.zoom_val, dut.hd_zoom_s);
        end
    end
`endif
    t = 0;
    while ((ldbusy == 0) && (t < 20000)) begin @(negedge clk); t = t + 1; end
    $display("  T5 8x 烘焙已在飞（%0d 拍后）", t);
    repeat (60) @(negedge clk);
    n_ab = 0;
    press(4'b1000);                          // 装载中再按（KEY4 = 缩一档到 4x）
`ifdef ZF_DEBUG
    begin : dbg
        integer j;
        for (j = 0; j < 25; j = j + 1) begin
            @(negedge clk);
            $display("   [T6] st=%0d zph=%0d ld_pend=%b zoom_ab=%b ab=%b ldp=%0d busy=%b want=%b hdp=%b tgt=%b rdy=%b",
                     dut.state, dut.zph, dut.ld_pend, dut.zoom_abort, ld_abort,
                     dut.ldp, ldbusy, dut.hd_want, dut.hd_pend, dut.ld_tgt_ok, dut.hd_ready);
        end
    end
`endif
    t = 0;
    while ((n_ab == 0) && (t < 200)) begin @(negedge clk); t = t + 1; end
    $display("  T6 装载中按键 → %0d 拍后放弃在飞烘焙（abort=%0d，期望 ≥1）", t, n_ab);
    if (n_ab != 0) $display("     ok    装载期间按键不需要等烘焙结束");
    else           $display("     FAIL  装载期间按键被吞（ZOOM-RESTEP 失效）");

    $display("  T7 扫描中途改 a_hd/fmt565 的次数 = %0d（期望 0）", n_tear);
    if (n_tear != 0) $display("     FAIL  会闪黑");
    else             $display("     ok    没有帧内切换");

    $display("");
    // 注：565 切换现在**必须**等帧边界（tick），所以 T3 的下限就是 1 帧（≈1300 拍），
    //     这是正确行为（帧内切换会闪黑），不再是"冷却"。
    $display("  结论：T3 = %0d 拍 ≈ %0d 帧（帧边界提交，肉眼无感；帧内切换会闪黑）",
             c_t3, c_t3/1300);
    $display("=============================================================");
    $display("");
    $finish;
end

task chk_state;
    input [255:0] name;
    input         cond;
    begin
        if (cond) $display("  ok    %0s", name);
        else      $display("  FAIL  %0s", name);
    end
endtask

initial begin
    #200_000_000;
    $display("  *** GLOBAL TIMEOUT");
    $finish;
end

endmodule
