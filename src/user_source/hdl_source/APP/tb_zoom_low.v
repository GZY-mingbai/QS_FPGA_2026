//==============================================================================
//  tb_zoom_low.v —— "切图后（S_LOW 等高清）按 KEY2 立刻进缩放 OSD" 的单元测试
//==============================================================================
//  iverilog -g2012 -o tzl.vvp -s tb_zoomlow tb_zoom_low.v -y ../../..
//  vvp tzl.vvp
//
//  查四件事：
//    T3 按 KEY2 后 1~2 拍就进 S_ZOOM，相位是 zph=1（**565 预览**，不是 zph=0）
//       —— zph=0 会无条件宣布 888 画布有效，而这时画布还没装完（会花屏）
//    T4 在飞的那次高清装载**不被打断**（它是当前图的 888，装完正好用上）
//    T5 退出缩放（一次都没按）→ 1 拍回 S_LOW 接着等，不 hd_kill、不重装
//    T6 真按了放大 → 立刻起烘焙（zph=2），不用等高清先装完
//==============================================================================
`timescale 1ns/1ps
module tb_zoomlow;
reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg key_prev=0, key_next=0, key_fx=0, key_menu=0;
reg key_menu_r=0, key_osd_exit=0;
reg file_page = 0;
wire osd_allow, zoom_allow, osd_on;
wire key_osd_enter = key_menu_r & osd_allow;

reg pipe_ce = 1;
wire busy, ps_start;

wire compose_en, fmt565, a_en, b_en, a_hd;
wire [20:0] a_base, b_base;
wire ld_req, ld_fmt, ld_done, ld_can_load, ld_abort, ld_err;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
wire [31:0] pos_a, size_a, pos_b, size_b;
wire [39:0] k_a, k_b;
wire [7:0] alpha_b;
wire [1:0] st_img; wire [2:0] st_fx; wire [3:0] st_state;
wire [15:0] O_base_w;

wire oz_on, z_prev_req, z_load;
wire [7:0] z_zoom, z_zoom_val;
wire [9:0] z_ax, z_ay; wire [1:0] z_mode;
wire z_act_go, z_rest;

viewer_ctrl #(
    .THUMB0_ADDR(21'd614400), .THUMB1_ADDR(21'd652800),
    .THUMB2_ADDR(21'd691200), .HD_ADDR(21'd729600), .ARM_FRAMES(1)
) dut (
    .clk(clk), .rst(rst),
    .I_img_off0(32'd1000), .I_img_off1(32'd2000), .I_img_off2(32'd3000),
    .I_img_off3(32'd0), .O_base(O_base_w), .I_win4(1'b0), .I_any(1'b1),
    .I_nimg(3'd2), .I_alb_ready(1'b1), .I_jump(1'b0), .I_jump_idx(8'd0),
    .I_file_page(file_page),
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(key_osd_enter), .key_osd_exit(key_osd_exit), .osd_fade(8'd255),
    .osd_on(osd_on), .osd_allow(osd_allow), .zoom_allow(zoom_allow),
    .zoom_on(oz_on), .preview_req(z_prev_req), .load_req(z_load),
    .zoom_val(z_zoom), .za_x(10'd320), .za_y(10'd240),
    .busy(busy), .ps_start(ps_start), .pipe_ce(pipe_ce),
    .ld_req(ld_req), .ld_fmt(ld_fmt), .ld_off(ld_off), .ld_len(ld_len), .ld_base(ld_base),
    .ld_can_load(ld_can_load), .ld_done(ld_done), .ld_err(ld_err),
    .ld_sd_ready(1'b1), .O_ld_abort(ld_abort), .O_hd_rel(),
    .compose_en(compose_en), .fps_dbg(), .fmt565(fmt565),
    .a_hd(a_hd), .a_en(a_en), .a_base(a_base), .b_en(b_en), .b_base(b_base),
    .pos_a(pos_a), .size_a(size_a), .k_a(k_a), .ori_a(),
    .pos_b(pos_b), .size_b(size_b), .k_b(k_b), .alpha_b(alpha_b),
    .st_img(st_img), .st_fx(st_fx), .st_state(st_state),
    .fx_sel(3'd0), .auto_mode(3'd0),.info_on(1'b0),
    .album_prev(1'b0), .album_next(1'b0), .osd_trig(1'b0), .music_done(1'b0),
    .ld_src_w(16'd640), .ld_src_h(16'd480),
    .res_w(), .res_h(), .auto_remain(), .auto_bar(), .info_show(),    .ld_zoom(), .ld_ax(), .ld_ay()
);

zoom_osd u_zoom (
    .clk(clk), .rst(rst),
    .key_press({key_prev, key_next, key_fx, key_menu}),
    .menu_on(osd_on), .allow(zoom_allow),
    .ld_done(ld_done), .hd_shown(a_hd | (fmt565 & oz_on)),
    .act_code(2'd0),
    .zoom_on(oz_on), .zoom(z_zoom), .ax(z_ax), .ay(z_ay), .mode(z_mode),
    .refresh(), .preview_req(z_prev_req), .load_req(z_load),
    .act_go(z_act_go), .act_sel(), .restore(z_rest)
);

reg ps_r=0, busy_r=0; integer fcnt=0;
always @(posedge clk or posedge rst) begin
    if (rst) begin ps_r<=0; busy_r<=0; fcnt<=0; end
    else if (compose_en) begin
        if (ps_r) begin
            if (fcnt==800) begin ps_r<=0; busy_r<=0; fcnt<=0; end else fcnt<=fcnt+1;
        end else if (!busy_r && fcnt==0) fcnt<=-200;
        else if (fcnt<0) begin fcnt<=fcnt+1; if (fcnt==-1) begin ps_r<=1; busy_r<=1; end end
    end else begin ps_r<=0; busy_r<=0; fcnt<=0; end
end
assign busy=busy_r; assign ps_start=ps_r;

reg ld_done_r=0, ldbusy=0, ld_err_r=0; integer ldcnt=0;
always @(posedge clk or posedge rst) begin
    if (rst) begin ld_done_r<=0; ldcnt<=0; ldbusy<=0; ld_err_r<=0; end
    else begin
        ld_done_r<=0;
        if (ld_req) begin ldbusy<=1; ldcnt<=0; end
        else if (ldbusy) begin
            ldcnt<=ldcnt+1;
            if ((ldcnt==3000)||ld_abort) begin
                ld_done_r<=1; ld_err_r<=ld_abort; ldbusy<=0;
            end
        end
    end
end
assign ld_done=ld_done_r; assign ld_err=ld_err_r; assign ld_can_load=~ldbusy;

integer n_ab=0, n_req=0;
always @(posedge clk) begin
    if (ld_abort) n_ab <= n_ab+1;
    if (ld_req) n_req <= n_req+1;
end

integer t;
task press; input [3:0] k; begin
    @(negedge clk); {key_prev,key_next,key_fx,key_menu}=k; key_menu_r = k[0];
    @(negedge clk); {key_prev,key_next,key_fx,key_menu}=4'd0; key_menu_r = 1'b0;
end endtask

reg e_t0=0, e_t1=0, e_t2=0, e_t3=0, e_t4=0;
integer ab0 = 0;

initial begin
    repeat(20) @(negedge clk);
    rst=0; t=0;
    while ((st_state!=4'd5) && (t<300000)) begin @(negedge clk); t=t+1; end
    $display("[T0] S_HD，用时 %0d 拍", t);
    // 等开机那三单（当前缩略图 / 高清 / 下一张预取）全部落地，否则 go_n 因为
    // vn=0 会把翻页键丢掉（img_switch 的换图资格是"目标块已装好"）
    t=0; while (((n_req<3) || ldbusy || (dut.u_sw.tv!==3'b011)) && (t<400000)) begin @(negedge clk); t=t+1; end
    $display("     开机装载收尾：req=%0d ldbusy=%b（%0d 拍）", n_req, ldbusy, t);

    $display("     [dbg] tv=%b dv=%b cr=%0d cs=%0d vn=%b dn=%b cur_ok=%b go_n=%b home_ok=%b",
             dut.u_sw.tv, dut.u_sw.dv, dut.u_sw.cr, dut.u_sw.cs,
             dut.u_sw.vn, dut.u_sw.dn, dut.cur_ok, dut.u_sw.go_n, dut.home_ok);
    press(4'b0100);                       // KEY3 下一张 → 过渡 → S_LOW
    t=0; while ((st_state!=4'd2) && (t<400000)) begin @(negedge clk); t=t+1; end
    $display("[T1] 进 S_LOW，用时 %0d 拍（zoom_allow=%b）", t, zoom_allow);
    e_t0 = (st_state==4'd2) && zoom_allow;

    // 等高清装载真的在飞
    t=0; while ((ldbusy==0) && (t<200000)) begin @(negedge clk); t=t+1; end
    $display("[T2] 高清在飞（ldbusy=%b req=%0d）", ldbusy, n_req);
    ab0 = n_ab;

    press(4'b0010);                       // KEY2 进缩放
    t=0; while (((oz_on==0)||(st_state!=4'd9)) && (t<200)) begin @(negedge clk); t=t+1; end
    $display("[T3] 按 KEY2 后 %0d 拍 oz_on=%b state=%0d zph=%0d a_hd=%b fmt565=%b",
             t, oz_on, st_state, dut.zph, a_hd, fmt565);
    e_t1 = (oz_on==1) && (st_state==4'd9) && (dut.zph==2'd1) && (a_hd==0) && (fmt565==1);

    repeat(40) @(negedge clk);
    $display("[T4] 进缩放后 40 拍：装载仍在飞=%b，abort 次数=%0d（期望 0）", ldbusy, n_ab-ab0);
    e_t2 = (n_ab==ab0);

    // 退出（一次缩放都没按）⇒ 应回 S_LOW 接着等，不重装
    press(4'b0001);
    t=0; while ((st_state!=4'd2) && (t<400)) begin @(negedge clk); t=t+1; end
    $display("[T5] 退出缩放 %0d 拍后 state=%0d  abort=%0d（期望 state=2 且 abort=0）",
             t, st_state, n_ab-ab0);
    e_t3 = (st_state==4'd2) && (n_ab==ab0);

    // 再进一次，这回按一下缩放：应该起烘焙
    press(4'b0010);                       // 进缩放
    repeat(30) @(negedge clk);
    press(4'b0010); repeat(30) @(negedge clk);   // mode 0->1
    press(4'b0010); repeat(30) @(negedge clk);   // mode 1->2
    press(4'b0100);                       // KEY3 放大
    t=0; while ((dut.zph!=2'd2) && (t<20000)) begin @(negedge clk); t=t+1; end
    $display("[T6] 按放大后 %0d 拍 zph=%0d zoom=%h（期望 zph=2 起烘焙）", t, dut.zph, z_zoom);
    e_t4 = (dut.zph==2'd2) && (z_zoom==8'h20);

    $display("\n  T3 进 OSD 立刻可用 : %s", e_t0&&e_t1 ? "PASS" : "FAIL");
    $display("  T4 不打断在飞装载   : %s", (e_t0&&e_t1&&e_t2) ? "PASS" : "FAIL");
    $display("  T5 退出不重装       : %s", e_t3 ? "PASS" : "FAIL");
    $display("  T6 按键能起烘焙     : %s", e_t4 ? "PASS" : "FAIL");
    $finish;
end
endmodule
