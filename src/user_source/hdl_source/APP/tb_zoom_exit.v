//==============================================================================
//  tb_zoom_exit.v —— "缩放后关掉缩放 OSD，再按翻页/菜单必须立刻生效" 的单元测试
//==============================================================================
//  iverilog -g2012 -o tze.vvp -s tb_zoomexit tb_zoom_exit.v -y ../../..
//  vvp tze.vvp
//
//  【被测行为】[ZOOM-EXIT-FAST]
//    1) 缩放到 2×（888 烘焙在飞）→ KEY1 关掉缩放 OSD
//    2) 立刻按 KEY3（翻页）/ KEY1（开菜单）
//    期望：不等待那次 2× 烘焙 —— 先把 2× 烘焙放弃、动画回 1×，然后立刻
//          落到 S_LOW（按键全活），暂存的动作当拍放行。
//    【改前】[ZOOM-RESTORE] 要等一整次 1× 烘焙（本台把单次装载拉到 LD_LEN 拍，
//          相当于上板大图 ~4s）才放行动作。
//
//  本台把 App.v 里那几根"恢复"粘合线也原样搬了进来（z_swallow / act_code /
//  act_go 补放），否则测不到真实通路。
//==============================================================================
`timescale 1ns/1ps
module tb_zoomexit;
reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg key_prev=0, key_next=0, key_fx=0, key_menu=0;
reg key_osd_exit=0;

wire osd_allow, zoom_allow, osd_on;
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
wire [7:0] z_zoom;
wire [9:0] z_ax, z_ay; wire [1:0] z_mode;
wire z_act_go; wire [1:0] z_act_sel;
wire z_rest;

//------------------------------------------------------------------------------
//  App.v 的 [ZOOM-RESTORE] 粘合（原样搬过来）
//------------------------------------------------------------------------------
wire        z_need_rst = ~oz_on & (z_zoom != 8'h10);        // 已退出 OSD 且仍在放大
wire [1:0]  z_act_i    = key_menu ? 2'd3 :
                         key_next ? 2'd2 :
                         key_prev ? 2'd1 : 2'd0;
wire        z_swallow  = z_need_rst & (z_act_i != 2'd0);
wire        k_menu_f   = (key_menu & ~z_swallow) | (z_act_go & (z_act_sel == 2'd3));
wire        k_next_f   = (key_next & ~z_swallow) | (z_act_go & (z_act_sel == 2'd2));
wire        k_prev_f   = (key_prev & ~z_swallow) | (z_act_go & (z_act_sel == 2'd1));
wire        key_osd_enter = k_menu_f & osd_allow;

viewer_ctrl #(
    .THUMB0_ADDR(21'd614400), .THUMB1_ADDR(21'd652800),
    .THUMB2_ADDR(21'd691200), .HD_ADDR(21'd729600), .ARM_FRAMES(1)
) dut (
    .clk(clk), .rst(rst),
    .I_img_off0(32'd1000), .I_img_off1(32'd2000), .I_img_off2(32'd3000),
    .I_img_off3(32'd0), .O_base(O_base_w), .I_win4(1'b0), .I_any(1'b1),
    .I_nimg(3'd2), .I_alb_ready(1'b1), .I_jump(1'b0), .I_jump_idx(8'd0),
    .I_file_page(1'b0),
    .key_prev(k_prev_f), .key_next(k_next_f), .key_fx(key_fx),
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
    .act_code(z_act_i),
    .zoom_on(oz_on), .zoom(z_zoom), .ax(z_ax), .ay(z_ay), .mode(z_mode),
    .refresh(), .preview_req(z_prev_req), .load_req(z_load),
    .act_go(z_act_go), .act_sel(z_act_sel), .restore(z_rest)
);

//------------------------------------------------------------------------------
//  帧 / 装载模型
//------------------------------------------------------------------------------
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

// 单次装载长度：**故意拉长**到 120000 拍 —— 相当于上板 12MP 源的一次整幅烘焙
// （~4s）。这样"有没有等烘焙"一眼就看出来（动画 ~10 帧 = 8000 拍）。
localparam integer LD_LEN = 120000;
reg ld_done_r=0, ldbusy=0, ld_err_r=0, ld_req_d=0; integer ldcnt=0;
always @(posedge clk or posedge rst) begin
    if (rst) begin ld_done_r<=0; ldcnt<=0; ldbusy<=0; ld_err_r<=0; ld_req_d<=0; end
    else begin
        ld_done_r<=0;
        ld_req_d<=ld_req;
        // ★ 只在 ld_req 的**上升沿**起一单（ld_req 是电平，装载期间一直为高；
        //   照 bmp_loader 的语义，起单后自己跑完，跟请求电平无关）
        if (ld_req && !ld_req_d && !ldbusy) begin ldbusy<=1; ldcnt<=0; end
        else if (ldbusy) begin
            ldcnt<=ldcnt+1;
            if ((ldcnt==LD_LEN)||ld_abort) begin
                ld_done_r<=1; ld_err_r<=ld_abort; ldbusy<=0;
            end
        end
    end
end
assign ld_done=ld_done_r; assign ld_err=ld_err_r; assign ld_can_load=~ldbusy;

integer n_ab=0, n_req=0;
always @(posedge clk) begin
    if (ld_abort) n_ab <= n_ab+1;
    if (ld_req)   n_req <= n_req+1;
end

integer t;
// 事件跟踪：state / zph / oz_on / zoom 任一变就打印
reg [3:0] s_d=15; reg [1:0] z_d=3; reg o_d=0; reg [7:0] zm_d=0;
always @(posedge clk) begin
    s_d<=st_state; z_d<=dut.zph; o_d<=oz_on; zm_d<=z_zoom;
    if ((st_state!=s_d)||(dut.zph!=z_d)||(oz_on!=o_d)||(z_zoom!=zm_d))
        $display("      [ev] state %0d->%0d zph %0d->%0d oz %b->%b zoom %h->%h zk=%0d tgt=%0d",
                 s_d, st_state, z_d, dut.zph, o_d, oz_on, zm_d, z_zoom, dut.zk, dut.zk_tgt);
end

task press; input [3:0] k; begin
    @(negedge clk); {key_prev,key_next,key_fx,key_menu}=k;
    @(negedge clk); {key_prev,key_next,key_fx,key_menu}=4'd0;
end endtask

// 进缩放 OSD 并把 mode 切到 ZOOM，再放大一档到 2×
//   ⚠ mode 跨会话保留（Z_OFF 重开不重置）⇒ 不能按固定次数切，得按当前值走到 2
task zoom_to_2x;
    integer i;
    begin
        if (!oz_on) begin press(4'b0010); repeat(20) @(negedge clk); end
        for (i=0; (i<4) && (z_mode!=2'd2); i=i+1) begin
            press(4'b0010); repeat(20) @(negedge clk);
        end
        // 档位跨会话保留（可能停在 4×/8×）⇒ 一路走到 2×，不能只会"按一下放大"
        for (i=0; (i<4) && (z_zoom!=8'h20); i=i+1) begin
            if (z_zoom < 8'h20) press(4'b0100);      // KEY3 放大
            else                press(4'b1000);      // KEY4 缩小
            repeat(20) @(negedge clk);
        end
        t=0; while ((dut.zph!=2'd2) && (t<20000)) begin @(negedge clk); t=t+1; end
        // 等这一单真的开始装（zph=2 只是"已发单"，ld_req 下一拍才抬）
        t=0; while ((ldbusy==0) && (t<2000)) begin @(negedge clk); t=t+1; end
        // 等 zk 插值真的走到目标档（否则退出时"动画"是零长度）
        t=0; while ((dut.zk != dut.zk_tgt) && (t<60000)) begin @(negedge clk); t=t+1; end
    end
endtask

// 等回到 S_HD（上一轮过渡/装载收尾）
task wait_hd;
    begin
        t=0; while ((st_state!=4'd5) && (t<2000000)) begin @(negedge clk); t=t+1; end
        // 等"下一张的缩略图"真的在缓存里（go_n 要求 vn）—— 不能只看 tv==3'b011，
        // 换过图之后 cur 会变，下一格就不是 slot1 了。
        t=0; while ((ldbusy||(dut.u_sw.vn!==1'b1)) && (t<2000000)) begin @(negedge clk); t=t+1; end
    end
endtask

reg p1=0, p2=0, p3=0, p4=0, p5=0;
integer zk0=0, zk1=0;

initial begin
    repeat(20) @(negedge clk);
    rst=0;
    wait_hd;
    $display("[T0] 开机到 S_HD");

    //==========================================================================
    //  T1/T2：缩放到 2× ⇒ 关 OSD ⇒ 按翻页
    //==========================================================================
    zoom_to_2x;
    $display("[T1] 2× 烘焙在飞：zph=%0d zoom=%h ldbusy=%b", dut.zph, z_zoom, ldbusy);
    $display("     dbg: ldp=%0d hd_want=%b hd_take=%b alb_ok=%b ld_req=%b ldfmt=%b req=%0d",
             dut.ldp, dut.hd_want, dut.hd_take, dut.alb_ok, ld_req, ld_fmt, n_req);

    press(4'b0001);                          // KEY1 关掉缩放 OSD
    repeat(5) @(negedge clk);
    $display("     关 OSD 后：oz_on=%b state=%0d zph=%0d（还在等这次烘焙 —— 正常）",
             oz_on, st_state, dut.zph);
    p1 = (oz_on==0) && (st_state==4'd9);

    n_ab = 0;
    zk0 = dut.zk;                            // 此刻应还是 2× 的插值值
    press(4'b0100);                          // KEY3 = 下一页
    t=0;
    while ((st_state!=4'd7)&&(t<120000)) begin @(negedge clk); t=t+1; end
    repeat(3) @(negedge clk);
    $display("[T2] 按翻页后 %0d 拍到 state=%0d（S_ARM=7），abort=%0d", t, st_state, n_ab);
    // 关键：远小于一次装载（%0d 拍），而且 2× 那一单确实被砍了
    zk1 = dut.zk;
    $display("     zk: 按下前=%0d（2×）→ 动作开始时=%0d（1×=32768）", zk0, zk1);
    // ★ 关键：动作必须发生在动画**播完之后**（zk 收尾到 1×），不是把动画砍掉
    p2 = (t < 50000) && (n_ab >= 1) && (st_state==4'd7) &&
         (zk0 < 20000) && (zk1 == 32768);
    $display("     参照：一次装载 = %0d 拍；这次只用了 %0d 拍", LD_LEN, t);

    //==========================================================================
    //  T3：缩放到 2× ⇒ 关 OSD ⇒ 按菜单
    //==========================================================================
    wait_hd;
    zoom_to_2x;
    press(4'b0001);                          // 关缩放 OSD
    repeat(5) @(negedge clk);
    n_ab = 0;
    press(4'b0001);                          // KEY1 = 开菜单
    t=0;
    while ((st_state!=4'd8)&&(t<120000)) begin @(negedge clk); t=t+1; end
    repeat(3) @(negedge clk);
    $display("[T3] 按菜单后 %0d 拍到 state=%0d（S_OSD=8），abort=%0d", t, st_state, n_ab);
    p3 = (st_state==4'd8) && (t < 50000);

    //==========================================================================
    //  T4：缩放 2× 后关 OSD，**不按任何键** ⇒ 仍要老老实实装完并切到 888
    //      （这条是原行为，不能被 [ZOOM-EXIT-FAST] 改坏）
    //==========================================================================
    // 先把 T3 打开的菜单关掉
    @(negedge clk); key_osd_exit = 1; @(negedge clk); key_osd_exit = 0;
    wait_hd;
    zoom_to_2x;
    press(4'b0001);                          // 关缩放 OSD
    t=0; while ((st_state!=4'd5) && (t<2000000)) begin @(negedge clk); t=t+1; end
    $display("[T4] 不按键：%0d 拍后 state=%0d a_hd=%b fmt565=%b zoom=%h",
             t, st_state, a_hd, fmt565, z_zoom);
    p4 = (st_state==4'd5) && (a_hd==1) && (fmt565==0) && (z_zoom==8'h20);

    //==========================================================================
    //  T5：**等 2× 烘焙装完**（画布已经切回 888）再关 OSD ⇒ 再翻页
    //      上板最常走的就是这条：用户看着清晰的 2× 图，关掉 OSD，然后翻页。
    //      这条曾经把动画整段吃掉（恢复请求到时 zoom_on 已经掉 0，直接落 S_HDF）。
    //==========================================================================
    wait_hd;
    zoom_to_2x;
    t=0; while (!((a_hd==1)&&(fmt565==0)&&(dut.zph==2'd0)) && (t<2000000)) begin @(negedge clk); t=t+1; end
    $display("[T5] 2× 已装完切 888（%0d 拍）：state=%0d a_hd=%b fmt565=%b zk=%0d",
             t, st_state, a_hd, fmt565, dut.zk);
    press(4'b0001);                          // 关缩放 OSD ⇒ S_HDF → S_HD
    t=0; while ((st_state!=4'd5) && (t<200000)) begin @(negedge clk); t=t+1; end
    n_ab = 0;
    zk0 = dut.zk;
    // 这里按 KEY1（开菜单）：菜单不依赖"下一格缩略图已缓存"，测的仍是
    // "动画播完 → 动作才发生" 这条链路（翻页那条见 T2）。
    press(4'b0001);                          // KEY1 = 开菜单
    t=0; while ((st_state!=4'd8) && (t<200000)) begin @(negedge clk); t=t+1; end
    zk1 = dut.zk;
    $display("[T5] 开菜单后 %0d 拍 state=%0d；zk %0d（2×）→ %0d（1×=32768），abort=%0d",
             t, st_state, zk0, zk1, n_ab);
    // 动作必须在动画播完之后（zk 回到 1×），而且这段动画真的花了时间（>5000 拍）
    p5 = (st_state==4'd8) && (zk0 < 20000) && (zk1 == 32768) && (t > 5000);

    $display("\n========================================");
    $display("  T1 关 OSD 后仍在 S_ZOOM 等烘焙 : %s", p1 ? "PASS" : "FAIL");
    $display("  T2 翻页不再等烘焙              : %s", p2 ? "PASS" : "FAIL");
    $display("  T3 开菜单不再等烘焙            : %s", p3 ? "PASS" : "FAIL");
    $display("  T4 不按键仍会装完并切 888      : %s", p4 ? "PASS" : "FAIL");
    $display("  T5 装完后关 OSD 仍有动画       : %s", p5 ? "PASS" : "FAIL");
    $display("========================================");
    $finish;
end
endmodule
