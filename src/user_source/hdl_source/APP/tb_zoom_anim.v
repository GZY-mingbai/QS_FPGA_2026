//==============================================================================
//  tb_zoom_anim.v —— 缩放动画（S_ZOOM 四相位）流程断言
//==============================================================================
//  【为什么单独开台】
//    tb_ctrl 的 `st_state` 输出只有 3 位，连 S_ZOOM(=9) 都表示不了；
//    而且它的主流程是围绕"切图过渡"写的，塞进来会把断言搅乱。
//    这里只做一件事：把 S_ZOOM 的 888 → 565 → 装载 → 888 走一遍，
//    每一步断言 a_hd / fmt565 / zph / state。
//
//  【被测的四相位】
//    zph=0  888 高清连续出帧   ← preview_req & tick ⇒ 切 565
//    zph=1  565 缩略图预览      ← load_req ⇒ 踢装载
//    zph=2  装载中（仍 565）    ← hd_ready ⇒ 切回 888、zph=0
//    退出：zoom_on=0 且 zph=0 ⇒ 回 S_HD
//
//  运行：
//    iverilog -g2012 -o /tmp/tza.vvp -s tb_zoom_anim tb_zoom_anim.v \
//        viewer_ctrl.v viewer_fx.v && vvp /tmp/tza.vvp
//==============================================================================
`timescale 1ns / 1ps

module tb_zoom_anim;

reg clk = 0;
always #10 clk = ~clk;              // 50MHz

reg rst = 1;
reg [7:0] zoom_val_r = 8'h20;       // ★ [ZOOM-ANCHOR] 可在测试中切档验证相位

// 按键
reg key_prev = 0, key_next = 0, key_fx = 0;
reg key_osd_enter = 0, key_osd_exit = 0;

// 缩放 OSD 驱动
reg zoom_on     = 0;
reg preview_req = 0;
reg load_req    = 0;

// 帧源 / 装载器
wire busy, ps_start, pipe_ce;
wire ld_req, ld_done, ld_err;
wire ld_can_load = 1'b1;
wire ld_sd_ready = 1'b1;
reg  ld_done_r = 0;
reg  ld_err_r  = 0;
assign ld_done = ld_done_r;
assign ld_err  = ld_err_r;

wire        compose_en, fmt565, a_hd, a_en, b_en;
wire [7:0]  alpha_b;
wire [20:0] a_base, b_base;
wire [31:0] pos_a, size_a, pos_b, size_b;
wire [39:0] k_a, k_b;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
wire        ld_fmt;
wire [1:0]  st_img;
wire [2:0]  st_fx;
wire [3:0]  st_state;

viewer_ctrl #(
    .THUMB0_ADDR(21'd614400), .THUMB1_ADDR(21'd652800),
    .THUMB2_ADDR(21'd691200), .HD_ADDR(21'd729600),
    .ARM_FRAMES(1)                  // 加速
) dut (
    .clk(clk), .rst(rst),
    // ★ 2026-11-01：IMG0/1/2_OFFSET 已从参数改成端口（相册走 FAT，
    //   这几个台只关心几何/过渡，偏移给固定值即可）
    .I_img_off0 (32'd1000), .I_img_off1 (32'd2000), .I_img_off2 (32'd3000),
    .I_img_off3(32'd0),
    .I_have(3'd3), .I_alb_ready(1'b1), .I_jump(1'b0), .I_jump_idx(8'd0), .O_ld_abort(),
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(key_osd_enter), .key_osd_exit(key_osd_exit), .osd_fade(8'd255),
    .osd_on(), .osd_allow(),
    .zoom_on(zoom_on), .preview_req(preview_req), .load_req(load_req),
    .zoom_val(zoom_val_r),
    .za_x(10'd320), .za_y(10'd240),     // ★ [ZOOM-ANCHOR] 锚点固定屏幕中心，便于手算断言
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
    // 2026-10-31: tie off port added after this bench was written
    , .I_win4(1'b0)
);

//------------------------------------------------------------------------------
//  ★ [ZOOM-SEAMLESS] 抓"切到 565 的那一帧"的几何
//------------------------------------------------------------------------------
//  按下缩放的那一帧必须和上一帧（888）几何**完全一样**，否则上板就是
//  "点一下画面跳/撕裂"。这里在 zph 0→1 的那一拍把 zk / ph_a 抓下来断言。
reg [1:0]  zph_d;
reg [19:0] zk_sw  = 20'd0;
// ★ [ZOOM-EXACT] 锚点常量（与下面的 .za_x/.za_y 端口连接保持一致）
localparam [9:0] ZA_X = 10'd320;
localparam [9:0] ZA_Y = 10'd240;
localparam [19:0] ZA_ORI = {ZA_X, ZA_Y};

reg [19:0] ori_sw = 20'd0;
reg        sw_seen = 1'b0;

// ★ [ZOOM-NOWRIP] "切回 888"绝不能在扫描进行中发生：
//   上板表现就是"上半缩略图、下半白碎块"的撕裂帧（下半取的是 888 画布）。
//   （装载启动不做这个限制——实测推到帧边界反而更差，见 viewer_ctrl 的回退记录）
reg        a_hd_d  = 1'b1;
reg        ahd_mid = 1'b0;

always @(posedge clk) begin
    zph_d  <= dut.zph;
    a_hd_d <= a_hd;
    if (!rst && a_hd && !a_hd_d && ps_start)
        ahd_mid <= 1'b1;
    if (!rst && (dut.zph == 2'd1) && (zph_d == 2'd0)) begin
        zk_sw   <= dut.zk;
        ori_sw  <= dut.ori_a;
        sw_seen <= 1'b1;
    end
end

//------------------------------------------------------------------------------
//  假帧源（同 tb_ctrl：compose_en 时每帧 ps_start 1000 拍 + busy）
//------------------------------------------------------------------------------
reg        ps_start_r = 0, busy_r = 0;
integer    fcnt = 0;
assign ps_start = ps_start_r;
assign busy     = busy_r;
assign pipe_ce  = 1'b1;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        ps_start_r <= 0; busy_r <= 0; fcnt <= 0;
    end else begin
        if (compose_en) begin
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
            // ★ compose_en 落下时 must 清 fcnt：否则帧中途停帧后 fcnt 残留 >0，
            //   下次拉起时 `fcnt==0` 和 `fcnt<0` 两条都不成立 ⇒ 永久不出帧（tb 卡死）。
            ps_start_r <= 0; busy_r <= 0; fcnt <= 0;
        end
    end
end

//------------------------------------------------------------------------------
//  假装载器：ld_req 后 200 拍回 done
//------------------------------------------------------------------------------
integer ldcnt = 0;
reg     ldbusy = 0;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        ld_done_r <= 0; ldcnt <= 0; ldbusy <= 0;
    end else begin
        ld_done_r <= 0;
        if (ld_req) begin ldbusy <= 1; ldcnt <= 0; end
        else if (ldbusy) begin
            ldcnt <= ldcnt + 1;
            if (ldcnt == 200) begin ld_done_r <= 1; ldbusy <= 0; end
        end
    end
end

//------------------------------------------------------------------------------
//  断言
//------------------------------------------------------------------------------
integer fails = 0, checks = 0;
task chk;
    input         cond;
    input [511:0] name;
    begin
        checks = checks + 1;
        if (!cond) begin fails = fails + 1;  $display("  FAIL  %0s", name); end
        else                                  $display("  ok    %0s", name);
    end
endtask

// 等状态（用层次引用看 4 位 state）
task wait_tick;
    input integer to;
    integer i;
    begin
        i = 0;
        while ((dut.tick !== 1'b1) && (i < to)) begin
            @(posedge clk); i = i + 1;
        end
        if (i >= to) $display("      WARN wait_tick timeout (compose_en=%b state=%0d)", dut.compose_en, dut.state);
    end
endtask

task wait_state;
    input [3:0] s;
    input integer to;
    integer i;
    begin
        i = 0;
        while ((dut.state !== s) && (i < to)) begin
            @(posedge clk); i = i + 1;
        end
    end
endtask

// ★ 监视：退出缩放时是否经过 S_HDF（补帧擦十字条）
reg saw_hdf = 1'b0;
always @(posedge clk) if (dut.state == 4'd4) saw_hdf <= 1'b1;

integer i;

initial begin
    // ★ 必须在时间 0 显式赋值：只靠声明处的 `= 8'h20` 时，iverilog 的
    //   `always @(*)`（viewer_ctrl 里的 zk_tgt case）可能拿不到这次变化事件，
    //   zk_tgt 会一直停在 X ⇒ zk 永不收敛（实测踩过）。
    zoom_val_r = 8'h20;
    repeat (10) @(negedge clk);
    rst = 1'b0;
    repeat (20) @(negedge clk);

    //------------------------------------------------------------------
    $display("--- 1. 开机走到 S_HD（state=5）---");
    //------------------------------------------------------------------
    wait_state(4'd5, 20_000_000);
    chk(dut.state === 4'd5, "Z1 reaches S_HD");
    chk(a_hd === 1'b1 && fmt565 === 1'b0, "Z2 HD 888 active in S_HD");

    //------------------------------------------------------------------
    $display("--- 2. zoom_on 拉高 ⇒ S_ZOOM (9)、zph=0、888 ---");
    //------------------------------------------------------------------
    zoom_on = 1'b1;
    wait_state(4'd9, 1000);
    repeat (10) @(negedge clk);
    chk(dut.state === 4'd9, "Z3 enters S_ZOOM");
    chk(dut.zph     === 2'd0, "Z4 zph=0 (888 phase)");
    chk(a_hd === 1'b1 && fmt565 === 1'b0, "Z5 still 888 after open");
    chk(compose_en === 1'b1, "Z6 compose_en=1 (continuous frames)");
    // ★ 几何数值断言（状态机对了不代表几何对，黑屏就是这么来的）
    chk(dut.k_a === {20'd65536, 20'd65536}, "Z6b 888 phase: k_a = 1.0");

    //------------------------------------------------------------------
    $display("--- 3. preview_req + tick ⇒ 切 565 ---");
    //------------------------------------------------------------------
    // preview_req 要落在 tick 上（viewer_ctrl 用 preview_req && tick）
    // 直接等一个 tick 再给，保证不丢
    wait_tick(20000);
    @(negedge clk); preview_req = 1'b1;
    @(negedge clk); preview_req = 1'b0;
    repeat (4000) @(negedge clk);      // pending 要等下一个 tick（~1300 拍一帧）
    chk(dut.zph   === 2'd1, "Z7 zph=1 after preview_req");
    chk(a_hd      === 1'b0, "Z8 a_hd=0 (thumbnail)");
    chk(fmt565    === 1'b1, "Z9 fmt565=1 (565 preview)");
    chk(dut.k_a === {1'b0, dut.zk[19:1], dut.zk}, "Z9b 565: KX = zk/2, KY = zk (KX != KY)");
    // ★★★ [ZOOM-SEAMLESS] 切换那一帧的几何必须还是 1:1（这是"按下就撕裂"的回归锁）★★★
    chk(sw_seen,                     "Z9b1 caught the 888->565 switch frame");
    chk(zk_sw === 20'd32768,         "Z9b2 switch frame keeps k at 1.0x (no 14% jump)");
    chk(ori_sw === {ZA_X, ZA_Y},     "Z9b3 switch frame keeps anchor unchanged");
    // 等插值收敛（2× ⇒ 目标 KY=16384, KX=8192）
    repeat (48000) @(negedge clk);
    chk(dut.k_a === {20'd8192, 20'd16384}, "Z9c 565: k converges to {0.125,0.25} at 2x");
    // 锚点保不变：2× 时 pos_x 必须收敛到 -ax = -200
    chk(dut.pos_a === 32'd0, "Z9d 565: pos=0 (full screen, no black edge)");

    //★ [ZOOM-ANCHOR] 锚点靠"源相位"实现，框不动：
    //   PH_x = ax·(16384 − kx) = 320·(16384−8192) = 2621440
    //   PH_y = ay·(32768 − ky) = 240·(32768−16384) = 3932160
    //   屏幕 (ax,ay) 映射回源 (ax,ay) ⇒ 缩放中心就是十字线。
    chk(dut.ori_a === {ZA_X, ZA_Y}, "Z9e 2x: ori = anchor (exact)");
    // 反过来验证几何：screen x=ax 时的源坐标必须等于 ax
    //   sx_word = (x·kx + PH)>>16 = (320·8192 + 2621440)>>16 = 5242880>>16 = 80
    //   ax=320 ⇒ 320/4 = 80 字 ✓（565 每字 2 像素 ⇒ 4 个 1× 屏幕像素/字）
    chk(((10'd320 * 20'd8192) + 24'd2621440) >> 16 === 16'd80, "Z9f 2x: screen(ax)->src(ax)");

    //★ 4× / 8× 的相位（x 用 q=12288/14336 ⇒ 3a/3.5a；y 用 q=24576/28672 ⇒ 6a/7a）
    zoom_val_r = 8'h40;
    repeat (48000) @(negedge clk);
    chk(dut.k_a === {20'd4096, 20'd8192}, "Z9g 4x: k converges");
    chk(dut.ori_a === {ZA_X, ZA_Y}, "Z9h 4x: ori = anchor (exact)");

    zoom_val_r = 8'h80;
    repeat (48000) @(negedge clk);
    chk(dut.k_a === {20'd2048, 20'd4096}, "Z9i 8x: k converges");
    chk(dut.ori_a === {ZA_X, ZA_Y}, "Z9j 8x: ori = anchor (exact)");
    // 回到 1×：相位必须精确归零（等价性关键）
    zoom_val_r = 8'h10;
    repeat (48000) @(negedge clk);
    // 1× 时仍要给出锚点（它和 k 无关）；只有 888 相位才清 0
    chk(dut.ori_a === {ZA_X, ZA_Y}, "Z9k 1x: ori still = anchor");
    zoom_val_r = 8'h20;
    repeat (48000) @(negedge clk);

    //------------------------------------------------------------------
    $display("--- 4. load_req ⇒ 踢装载、zph=2 ---");
    //------------------------------------------------------------------
    @(negedge clk); load_req = 1'b1;
    @(negedge clk); load_req = 1'b0;
    repeat (200) @(negedge clk);       // ld_pend 在 zph=1 当拍就消费（不做帧同步）
    chk(dut.zph === 2'd2, "Z10 zph=2 (loading)");
    chk(ld_fmt  === 1'b1, "Z11 loader asked for 888 HD");
    chk(ld_base === 21'd729600, "Z12 loading into HD_ADDR");
    //★ 装载期间（zph=2）仍在显示 565 缩略图，k_a 也还是 565 的 k
    //  ⇒ 相位必须保持，不能清 0（上板"闪一下错误位置"就是这个）
    chk(dut.k_a  === {1'b0, dut.zk[19:1], dut.zk}, "Z12b loading still uses 565 k");
    chk(dut.ori_a === {ZA_X, ZA_Y}, "Z12c loading keeps the anchor (no jump to corner)");

    //------------------------------------------------------------------
    $display("--- 5. ld_done ⇒ hd_ready ⇒ 切回 888 ---");
    //------------------------------------------------------------------
    wait (dut.hd_ready === 1'b1);
    //★ [ZOOM-SWAP] 切换必须等帧边界：hd_ready 之后还要等 SWT+1 个 tick
    //  ⇒ 这里先确认"还没切"（说明不会在帧中间切），再等 tick 到位。
    repeat (10) @(negedge clk);
    chk(dut.zph !== 2'd0, "Z12d not switched right after hd_ready (waits for tick)");
    repeat (8000) @(negedge clk);      // 约 2 个 frame（tb 里一帧 ~1300 拍）
    chk(dut.zph === 2'd0, "Z13 zph back to 0 after load");
    chk(a_hd    === 1'b1, "Z14 a_hd=1 (888 restored)");
    chk(fmt565  === 1'b0, "Z15 fmt565=0");

    //------------------------------------------------------------------
    $display("--- 6. zoom_on=0 ⇒ 回 S_HD ---");
    //------------------------------------------------------------------
    saw_hdf = 1'b0;
    zoom_on = 1'b0;
    wait_state(4'd5, 20000);
    repeat (10) @(negedge clk);
    chk(dut.state === 4'd5, "Z16 exits to S_HD");
    // ★ 必须经过 S_HDF 补一帧：停帧会把带十字条的最后一帧永久留在屏上
    chk(saw_hdf, "Z16b exit repaints via S_HDF (erases cross)");
    chk(compose_en === 1'b0, "Z17 compose_en=0 (freeze again)");

    //------------------------------------------------------------------
    $display("--- 7. 二次进入：预览态直接退出，应补装再走 ---");
    //------------------------------------------------------------------
    zoom_on = 1'b1;
    wait_state(4'd9, 1000);
    repeat (10) @(negedge clk);
    wait_tick(20000);
    @(negedge clk); preview_req = 1'b1;
    @(negedge clk); preview_req = 1'b0;
    repeat (4000) @(negedge clk);
    chk(dut.zph === 2'd1, "Z18 back to 565 preview");

    // 此时直接退出（zph=1）⇒ 应该补装一次再回 S_HD
    zoom_on = 1'b0;
    repeat (10) @(negedge clk);
    chk(dut.zph === 2'd2 || dut.state === 4'd5, "Z19 exit triggers load or leaves");

    wait_state(4'd5, 2_000_000);
    repeat (10) @(negedge clk);
    $display("      DBG exit2: state=%0d zph=%0d hd_valid=%b hd_ready=%b hd_want=%b ldp=%0d",
             dut.state, dut.zph, dut.hd_valid, dut.hd_ready, dut.hd_want, dut.ldp);
    chk(dut.state === 4'd5, "Z20 eventually back to S_HD");
    chk(a_hd    === 1'b1 && fmt565 === 1'b0, "Z21 ends in 888");
    // ★★★ [ZOOM-NOWRIP] 切回 888 绝不许落在扫描进行中（撕裂帧的根因）★★★
    chk(!ahd_mid, "Z23 565->888 swap never happens mid-scan");

    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

initial begin
    #400_000_000;
    $display("  *** GLOBAL TIMEOUT");
    $finish;
end

endmodule
