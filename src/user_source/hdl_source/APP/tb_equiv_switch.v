//==============================================================================
//  tb_equiv_switch.v —— 换图逻辑「改动前 vs 改动后」逐拍等价性验证
//==============================================================================
//  【为什么要这个台子】
//    2026-11-02 把 OSD 相册页的换图并进了主页那套决策（slide_r/slide_l/
//    stuck_*/swap_to），上板却报"主页点切换卡住、切缩略图后不播动画"。
//    而 tb_switch（同样的场景）全 PASS ⇒ 说明**仿真没有覆盖到差别**。
//    与其继续猜，不如把两个版本**并排跑同一份激励**，逐拍比关键信号：
//    只要有一拍不同，就说明改动确实改了行为，且能定位到第一处差异。
//
//  【比较的信号】状态机 + 全部换图相关寄存器 + 送出去的装载请求
//  【激励】主页连按 12 次 → 返回 12 次 → 进菜单按 8 次 → 退出 → 再主页 6 次
//
//  iverilog -g2012 -o eq.vvp -s tb_equiv_switch tb_equiv_switch.v \
//           viewer_ctrl.v /tmp/eq/vc_old.v viewer_fx.v
//==============================================================================
`timescale 1ns/1ps

module tb_equiv_switch;

localparam integer NIMG = 10;
localparam [20:0] T0 = 21'd614400;
localparam [20:0] T1 = 21'd652800;
localparam [20:0] T2 = 21'd691200;
localparam [20:0] HD = 21'd729600;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg  key_prev = 0, key_next = 0, key_fx = 0;
reg  album_prev = 0, album_next = 0;
reg  osd_enter = 0, osd_exit = 0;
reg  busy = 0, ps_start = 0, pipe_ce = 1;
reg  [2:0] fx_sel = 3'd0;
reg  zoom_on = 0, preview_req = 0, load_req = 0;
reg  ld_sd_ready = 0;

//------------------------------------------------------------------------------
//  相册模型（两版共用同一份输入）
//------------------------------------------------------------------------------
function [31:0] off_of;
    input integer idx;
    begin off_of = 32'h0080_0000 + idx * 32'h0000_4000; end
endfunction

wire [15:0] O_base_w;
reg  [1:0]  alb_have;
reg         alb_win4;
reg         alb_ready;

wire [31:0] a_off0 = (O_base_w + 0 < NIMG) ? off_of(O_base_w + 0) : 32'd0;
wire [31:0] a_off1 = (O_base_w + 1 < NIMG) ? off_of(O_base_w + 1) : 32'd0;
wire [31:0] a_off2 = (O_base_w + 2 < NIMG) ? off_of(O_base_w + 2) : 32'd0;
wire [31:0] a_off3 = (O_base_w + 3 < NIMG) ? off_of(O_base_w + 3) : 32'd0;

//------------------------------------------------------------------------------
//  两套 DUT 的输出
//------------------------------------------------------------------------------
wire [20:0] o_abase, n_abase, o_bbase, n_bbase;
wire        o_ldreq, n_ldreq, o_ldfmt, n_ldfmt;
wire [31:0] o_ldoff, n_ldoff;
wire [20:0] o_ldbase, n_ldbase;
wire        o_comp, n_comp, o_fmt565, n_fmt565, o_ahd, n_ahd, o_aen, n_aen, o_ben, n_ben;
wire [1:0]  o_stimg, n_stimg;
wire [3:0]  o_state, n_state;
wire        o_hdval, n_hdval;
wire [1:0]  o_hdimg, n_hdimg;
wire        n_can, n_done, o_can, o_done;

// 只用一个 DUT 驱动相册窗口起点（两版必须看到同一个 O_base，否则比不了）
viewer_ctrl #(
    .T_END(6), .T_END_F(3), .ARM_FRAMES(1), .FRAME_CYC(1000),
    .THUMB0_ADDR(T0), .THUMB1_ADDR(T1), .THUMB2_ADDR(T2), .HD_ADDR(HD)
) u_new (
    .clk(clk), .rst(rst),
    .I_img_off0(a_off0), .I_img_off1(a_off1), .I_img_off2(a_off2), .I_img_off3(a_off3),
    .O_base(O_base_w), .I_win4(alb_win4), .I_have(alb_have), .I_alb_ready(alb_ready),
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(osd_enter), .key_osd_exit(osd_exit), .osd_fade(8'd255),
    .osd_on(), .osd_allow(),
    .zoom_on(zoom_on), .preview_req(preview_req), .load_req(load_req), .zoom_val(8'h10),
    .busy(busy), .ps_start(ps_start), .pipe_ce(pipe_ce),
    .ld_req(n_ldreq), .ld_fmt(n_ldfmt), .ld_off(n_ldoff), .ld_len(), .ld_base(n_ldbase),
    .ld_can_load(n_can), .ld_done(n_done), .ld_err(1'b0), .ld_sd_ready(ld_sd_ready),
    .compose_en(n_comp), .fps_dbg(), .fmt565(n_fmt565),
    .a_hd(n_ahd), .a_en(n_aen), .a_base(n_abase), .b_en(n_ben), .b_base(n_bbase),
    .pos_a(), .size_a(), .k_a(), .pos_b(), .size_b(), .k_b(), .alpha_b(),
    .st_img(n_stimg), .st_fx(), .st_state(n_state),
    .fx_sel(fx_sel), .auto_mode(3'd0),.info_on(1'b0),
    .album_prev(album_prev), .album_next(album_next), .osd_trig(1'b0), .music_done(1'b0),
    .ld_src_w(16'd640), .ld_src_h(16'd480),
    .res_w(), .res_h(), .auto_remain(), .auto_bar(), .info_show());

viewer_ctrl_old #(
    .T_END(6), .T_END_F(3), .ARM_FRAMES(1), .FRAME_CYC(1000),
    .THUMB0_ADDR(T0), .THUMB1_ADDR(T1), .THUMB2_ADDR(T2), .HD_ADDR(HD)
) u_old (
    .clk(clk), .rst(rst),
    .I_img_off0(a_off0), .I_img_off1(a_off1), .I_img_off2(a_off2), .I_img_off3(a_off3),
    .O_base(), .I_win4(alb_win4), .I_have(alb_have), .I_alb_ready(alb_ready),
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(osd_enter), .key_osd_exit(osd_exit), .osd_fade(8'd255),
    .osd_on(), .osd_allow(),
    .zoom_on(zoom_on), .preview_req(preview_req), .load_req(load_req), .zoom_val(8'h10),
    .busy(busy), .ps_start(ps_start), .pipe_ce(pipe_ce),
    .ld_req(o_ldreq), .ld_fmt(o_ldfmt), .ld_off(o_ldoff), .ld_len(), .ld_base(o_ldbase),
    .ld_can_load(o_can), .ld_done(o_done), .ld_err(1'b0), .ld_sd_ready(ld_sd_ready),
    .compose_en(o_comp), .fps_dbg(), .fmt565(o_fmt565),
    .a_hd(o_ahd), .a_en(o_aen), .a_base(o_abase), .b_en(o_ben), .b_base(o_bbase),
    .pos_a(), .size_a(), .k_a(), .pos_b(), .size_b(), .k_b(), .alpha_b(),
    .st_img(o_stimg), .st_fx(), .st_state(o_state),
    .fx_sel(fx_sel), .auto_mode(3'd0),.info_on(1'b0),
    .album_prev(album_prev), .album_next(album_next), .osd_trig(1'b0), .music_done(1'b0),
    .ld_src_w(16'd640), .ld_src_h(16'd480),
    .res_w(), .res_h(), .auto_remain(), .auto_bar(), .info_show());

//------------------------------------------------------------------------------
//  各自的装载模型（延迟一致）
//------------------------------------------------------------------------------
reg [15:0] c_n, c_o;
reg        b_n, b_o;
reg        dn_n, dn_o, can_n, can_o;

always @(posedge clk or posedge rst) begin
    if (rst) begin b_n<=0; c_n<=0; dn_n<=0; can_n<=1; end
    else begin
        dn_n <= 1'b0;
        if (n_ldreq && !b_n) begin b_n<=1'b1; c_n<= n_ldfmt ? 16'd120 : 16'd60; can_n<=1'b0; end
        else if (b_n) begin
            if (c_n == 16'd0) begin b_n<=1'b0; dn_n<=1'b1; can_n<=1'b1; end
            else c_n <= c_n - 16'd1;
        end
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin b_o<=0; c_o<=0; dn_o<=0; can_o<=1; end
    else begin
        dn_o <= 1'b0;
        if (o_ldreq && !b_o) begin b_o<=1'b1; c_o<= o_ldfmt ? 16'd120 : 16'd60; can_o<=1'b0; end
        else if (b_o) begin
            if (c_o == 16'd0) begin b_o<=1'b0; dn_o<=1'b1; can_o<=1'b1; end
            else c_o <= c_o - 16'd1;
        end
    end
end

assign n_can = can_n; assign n_done = dn_n;
assign o_can = can_o; assign o_done = dn_o;

//------------------------------------------------------------------------------
//  假帧源（两版共用）
//------------------------------------------------------------------------------
integer fcnt;
always @(posedge clk or posedge rst) begin
    if (rst) begin ps_start<=0; busy<=0; fcnt<=0; end
    else if (n_comp) begin
        if (ps_start) begin
            if (fcnt == 500) begin ps_start<=0; busy<=0; fcnt<=0; end
            else fcnt <= fcnt + 1;
        end else if (!busy && fcnt == 0) fcnt <= -20;
        else if (fcnt < 0) begin
            fcnt <= fcnt + 1;
            if (fcnt == -1) begin ps_start<=1; busy<=1; end
        end
    end else begin ps_start<=0; busy<=0; fcnt<=0; end
end

//------------------------------------------------------------------------------
//  相册重扫模型
//------------------------------------------------------------------------------
reg [15:0] base_d = 16'hFFFF;
reg [15:0] rsc;
always @(posedge clk or posedge rst) begin
    if (rst) begin base_d<=16'hFFFF; rsc<=0; alb_ready<=0; end
    else if (O_base_w != base_d) begin
        base_d <= O_base_w; rsc <= 16'd0; alb_ready <= 1'b0;
    end else if (rsc != 16'd40) begin
        rsc <= rsc + 16'd1;
        if (rsc == 16'd39) alb_ready <= 1'b1;
    end
end

always @(*) begin
    alb_have = (O_base_w + 2 < NIMG) ? 2'd3 :
               ((O_base_w + 1 < NIMG) ? 2'd2 : ((O_base_w + 0 < NIMG) ? 2'd1 : 2'd0));
    alb_win4 = (O_base_w + 3 < NIMG);
end

//------------------------------------------------------------------------------
//  逐拍比较
//------------------------------------------------------------------------------
integer errs = 0;
integer cyc = 0;
reg [255:0] ph = "boot";

always @(posedge clk) begin
    if (!rst) begin
        cyc <= cyc + 1;
        if (o_state !== n_state) begin
            errs = errs + 1;
            if (errs <= 10) $display("  [DIFF] t=%0t %0s state old=%0d new=%0d", $time, ph, o_state, n_state);
        end
        if (o_stimg !== n_stimg) begin
            errs = errs + 1;
            if (errs <= 10) $display("  [DIFF] t=%0t %0s st_img old=%0d new=%0d", $time, ph, o_stimg, n_stimg);
        end
        if (o_abase !== n_abase) begin
            errs = errs + 1;
            if (errs <= 10) $display("  [DIFF] t=%0t %0s a_base old=%0d new=%0d", $time, ph, o_abase, n_abase);
        end
        if (o_bbase !== n_bbase) begin
            errs = errs + 1;
            if (errs <= 10) $display("  [DIFF] t=%0t %0s b_base old=%0d new=%0d", $time, ph, o_bbase, n_bbase);
        end
        if (o_comp !== n_comp) begin
            errs = errs + 1;
            if (errs <= 10) $display("  [DIFF] t=%0t %0s compose_en old=%0d new=%0d", $time, ph, o_comp, n_comp);
        end
        if (o_ldreq !== n_ldreq) begin
            errs = errs + 1;
            if (errs <= 10) $display("  [DIFF] t=%0t %0s ld_req old=%0d new=%0d (state old=%0d new=%0d)",
                                     $time, ph, o_ldreq, n_ldreq, o_state, n_state);
        end
        if (o_ldoff !== n_ldoff) begin
            errs = errs + 1;
            if (errs <= 10) $display("  [DIFF] t=%0t %0s ld_off old=%h new=%h", $time, ph, o_ldoff, n_ldoff);
        end
        if (o_ldbase !== n_ldbase) begin
            errs = errs + 1;
            if (errs <= 10) $display("  [DIFF] t=%0t %0s ld_base old=%0d new=%0d", $time, ph, o_ldbase, n_ldbase);
        end
        if (o_ahd !== n_ahd) begin
            errs = errs + 1;
            if (errs <= 10) $display("  [DIFF] t=%0t %0s a_hd old=%0d new=%0d", $time, ph, o_ahd, n_ahd);
        end
        if (o_ben !== n_ben) begin
            errs = errs + 1;
            if (errs <= 10) $display("  [DIFF] t=%0t %0s b_en old=%0d new=%0d", $time, ph, o_ben, n_ben);
        end
    end
end

//------------------------------------------------------------------------------
//  激励
//------------------------------------------------------------------------------
task press;
    input integer which;   // 0=prev 1=next 2=album_prev 3=album_next 4=osd_enter 5=osd_exit
    integer t0;
    begin
        @(negedge clk);
        case (which)
            0: key_prev = 1;
            1: key_next = 1;
            2: album_prev = 1;
            3: album_next = 1;
            4: osd_enter = 1;
            default: osd_exit = 1;
        endcase
        @(negedge clk);
        key_prev = 0; key_next = 0; album_prev = 0; album_next = 0;
        osd_enter = 0; osd_exit = 0;
        repeat (60) @(posedge clk);
        // 等两个都回到静止（或超时）
        t0 = 0;
        while (((n_state != 4'd5) && (n_state != 4'd8)) && (t0 < 200_000)) begin
            @(posedge clk); t0 = t0 + 1;
        end
        repeat (300) @(posedge clk);
    end
endtask

integer k;
initial begin
    $display("== tb_equiv_switch：改动前(old) vs 改动后(new) 逐拍比对 ==");
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (100) @(posedge clk);
    ld_sd_ready = 1;

    ph = "boot";
    k = 0;
    while ((n_state != 4'd5) && (k < 400_000)) begin @(posedge clk); k = k + 1; end
    repeat (100) @(posedge clk);

    ph = "home-next";
    for (k = 0; k < 12; k = k + 1) press(1);
    ph = "home-prev";
    for (k = 0; k < 12; k = k + 1) press(0);

    ph = "menu-in";
    press(4);
    repeat (1500) @(posedge clk);
    ph = "menu-next";
    for (k = 0; k < 8; k = k + 1) press(3);
    ph = "menu-prev";
    for (k = 0; k < 8; k = k + 1) press(2);
    ph = "menu-out";
    press(5);
    repeat (3000) @(posedge clk);

    ph = "home-again";
    for (k = 0; k < 6; k = k + 1) press(1);

    $display("  总拍数 %0d，差异 %0d 处", cyc, errs);
    $display("== %s ==", (errs == 0) ? "EQUIVALENT" : "DIFFERENT");
    $finish;
end

initial begin
    #200_000_000;
    $display("TIMEOUT");
    $finish;
end

endmodule
