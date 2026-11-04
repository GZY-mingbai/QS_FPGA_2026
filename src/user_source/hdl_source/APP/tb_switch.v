//==============================================================================
//  tb_switch.v —— 相册切换矩阵：装载【内容 + 位置】全判据（viewer_ctrl 单元台）
//==============================================================================
//  【为什么要有这个台子】
//  tb_window 只查了 `abs = O_base + st_img`，也就是"序号对不对"。
//  序号对 ≠ 画面对：
//    · 缩略图装进了**错的那块缓冲**（ld_base 错）⇒ 序号对、画面花
//    · HD 槽里是**上一张**的内容却报 hd_valid ⇒ 序号对、切高清后看着还是上一张
//  这一版把三条独立证据都记下来逐个核对：
//      ① 每次装载的 (ld_fmt, ld_off, ld_base) —— 发了什么单、装到哪
//      ② 装载模型的缓冲镜像 buf_off[0..2] / hd_off —— 装完之后 SDRAM 里是什么
//      ③ 静止态不变式：
//           rot == base % 3                              （环映射没漂）
//           buf_off[thumb_at(rel,rot)] == off[base+rel]   （3 张缩略图各就各位）
//           hd_off == off[base+img_idx]                   （HD 槽就是当前这张）
//        ③ 是对"装载状态/位置"最直接的判据 —— 只要有一条不成立，
//        上板就是"缩略图花 / 切高清后是上一张"。
//
//  【覆盖的场景】
//    S1 N=10 正向走完 0→9            （含第 4 张起的每一次滑动）
//    S2 N=10 末张再按（不越界）
//    S3 N=10 反向走回 9→0            （含往回滑动）
//    S4 N=10 正反交替（1→2→1→2→3→2→3→4…）
//    S5 N=1  单张：prev/next 都必须原地不动
//    S6 N=2  两张：正反各走一遍
//    S7 N=3  三张：走完再回来（窗口一次都不滑）
//    S8 N=10 首张按 prev（不许跳到 rel2）
//
//  iverilog -g2012 -o sw.vvp -s tb_switch -P tb_switch.NIMG=10 \
//           tb_switch.v viewer_ctrl.v viewer_fx.v
//==============================================================================
`timescale 1ns/1ps

// ★ 2026-11-02：把 viewer_ctrl 的 ONE_TO 兜底阈值从 2^25（0.67s）降成参数比较。
//   合成路径用的是"高 4 位非零"（1 个 LUT），仿真里 3300 万拍根本跑不完，
//   所以仿真必须打开这个开关。综合时 tb 不参与，不影响硬件。
`define SIM_FAST_TO

module tb_switch;

`ifdef NIMG_VAL
parameter integer NIMG = `NIMG_VAL;
`else
parameter integer NIMG = 10;
`endif

localparam integer LAT_T = 60;      // 缩略图装载延迟（拍）
localparam integer LAT_H = 120;     // 高清装载延迟（拍）
// 相册重扫延迟（拍）。默认 40；`-DSCAN_VAL=5000` 可模拟"重扫比过渡还慢"
`ifdef SCAN_VAL
localparam integer SCAN  = `SCAN_VAL;
`else
localparam integer SCAN  = 40;
`endif

localparam [20:0] T0 = 21'd614400;
localparam [20:0] T1 = 21'd652800;
localparam [20:0] T2 = 21'd691200;
localparam [20:0] HD = 21'd729600;

localparam [3:0] ST_HD   = 4'd5;
localparam [3:0] ST_STUK = 4'd6;
localparam [3:0] ST_OSD  = 4'd8;    // ★ 2026-11-02：菜单通道判据用

reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg  key_prev = 0, key_next = 0, key_fx = 0;
// ★ 2026-11-02：菜单相册页通道（原来钉死 0，所以菜单那条路一直没被测过）
reg  album_prev = 0, album_next = 0;
reg  osd_enter = 0, osd_exit = 0;
reg  busy = 0, ps_start = 0, pipe_ce = 1;
wire ld_can_load, ld_done;
reg  ld_err = 0, ld_sd_ready = 0;
reg  [2:0] fx_sel = 3'd0;
reg  zoom_on = 0, preview_req = 0, load_req = 0;

//  DUT 端口网线（必须先于相册模型声明，否则 O_base 被当成隐式 1 位）
wire ld_req, ld_fmt;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
wire compose_en, fmt565, a_hd, a_en, b_en;
wire [20:0] a_base, b_base;
wire [31:0] pos_a, size_a, pos_b, size_b;
wire [39:0] k_a, k_b;
wire [7:0]  alpha_b;
wire [1:0]  st_img;
wire [2:0]  st_fx;
wire [3:0]  st_state;
wire [15:0] O_base;

reg  [15:0] base_d = 16'hFFFF;
reg  [15:0] rsc;
reg         alb_ready = 0;

//------------------------------------------------------------------------------
//  相册模型：给 off[base+0..3] + have + win4；O_base 变就重扫 SCAN 拍
//------------------------------------------------------------------------------
function [31:0] off_of;
    input integer idx;
    begin off_of = 32'h0080_0000 + idx * 32'h0000_4000; end
endfunction

wire [15:0] b0 = O_base;
wire [31:0] alb_off0 = (b0 + 0 < NIMG) ? off_of(b0 + 0) : 32'd0;
wire [31:0] alb_off1 = (b0 + 1 < NIMG) ? off_of(b0 + 1) : 32'd0;
wire [31:0] alb_off2 = (b0 + 2 < NIMG) ? off_of(b0 + 2) : 32'd0;
wire [31:0] alb_off3 = (b0 + 3 < NIMG) ? off_of(b0 + 3) : 32'd0;
// ★ 2026-11-02：重扫期间 I_have/I_win4 **无效（给 0）**，模拟真实 album_src
//   （它是寄存输出，扫完才更新）。原来这里是组合立即更新，
//   把"重扫期间 have=0 ⇒ have_n=1 ⇒ 边界判据全错"这条实机工况掩盖掉了。
wire [1:0]  alb_have_r = (b0 + 2 < NIMG) ? 2'd3 :
                         ((b0 + 1 < NIMG) ? 2'd2 : ((b0 + 0 < NIMG) ? 2'd1 : 2'd0));
wire [1:0]  alb_have = alb_ready ? alb_have_r : 2'd0;
wire        alb_win4 = alb_ready & (b0 + 3 < NIMG);

//------------------------------------------------------------------------------
//  DUT
//------------------------------------------------------------------------------
viewer_ctrl #(
    .T_END(6), .T_END_F(3), .ARM_FRAMES(1), .FRAME_CYC(1000),
    .THUMB0_ADDR(T0), .THUMB1_ADDR(T1), .THUMB2_ADDR(T2), .HD_ADDR(HD),
    .ONE_TO(2000)          // 仿真：0.67s → 2000 拍（40us），否则跑不动
) dut (
    .clk(clk), .rst(rst),
    .I_img_off0(alb_off0), .I_img_off1(alb_off1), .I_img_off2(alb_off2),
    .I_img_off3(alb_off3),
    .O_base(O_base), .I_win4(alb_win4), .I_have(alb_have), .I_alb_ready(alb_ready),
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(osd_enter), .key_osd_exit(osd_exit), .osd_fade(8'd255),
    .osd_on(), .osd_allow(),
    .zoom_on(zoom_on), .preview_req(preview_req), .load_req(load_req), .zoom_val(8'h10),
    .busy(busy), .ps_start(ps_start), .pipe_ce(pipe_ce),
    .ld_req(ld_req), .ld_fmt(ld_fmt), .ld_off(ld_off), .ld_len(ld_len), .ld_base(ld_base),
    .ld_can_load(ld_can_load), .ld_done(ld_done), .ld_err(ld_err), .ld_sd_ready(ld_sd_ready),
    .compose_en(compose_en), .fps_dbg(), .fmt565(fmt565),
    .a_hd(a_hd), .a_en(a_en), .a_base(a_base), .b_en(b_en), .b_base(b_base),
    .pos_a(pos_a), .size_a(size_a), .k_a(k_a),
    .pos_b(pos_b), .size_b(size_b), .k_b(k_b), .alpha_b(alpha_b),
    .st_img(st_img), .st_fx(st_fx), .st_state(st_state),
    .fx_sel(fx_sel), .auto_mode(3'd0),.info_on(1'b0),
    .album_prev(album_prev), .album_next(album_next), .osd_trig(1'b0), .music_done(1'b0),
    .ld_src_w(16'd640), .ld_src_h(16'd480),
    .res_w(), .res_h(), .auto_remain(), .auto_bar(), .info_show());

//------------------------------------------------------------------------------
//  相册重扫模型：O_base 变 → ready 拉低 SCAN 拍
//------------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        base_d <= 16'hFFFF; rsc <= 16'd0; alb_ready <= 1'b0;
    end else if (O_base != base_d) begin
        base_d <= O_base; rsc <= 16'd0; alb_ready <= 1'b0;
    end else if (rsc != SCAN) begin
        rsc <= rsc + 16'd1;
    end else
        alb_ready <= 1'b1;
end

//------------------------------------------------------------------------------
//  装载模型：记录每次发的单，装完更新 SDRAM 镜像
//------------------------------------------------------------------------------
reg [31:0] buf_off [0:2];        // THUMB0/1/2 里现在装的是哪个 off
reg [31:0] hd_off_r;             // HD 槽里现在装的是哪个 off
reg        hd_any;               // HD 槽至少被 fmt=1 装过一次
reg [2:0]  buf_any;              // 每位：该缓冲装过

integer nload = 0, nthumb = 0, nhd = 0;
integer ldcnt = 0;
reg     ldbusy = 0;
reg     ld_done_r = 0;
reg     noload    = 0;      // 1 = 模拟装载器不给（ld_can_load=0）
reg [31:0] cap_off;
reg [20:0] cap_base;
reg        cap_fmt;
reg        cap_img;

assign ld_done  = ld_done_r;
assign ld_can_load = !ldbusy && !noload;

function integer tidx;
    input [20:0] a;
    begin
        if (a == T0)      tidx = 0;
        else if (a == T1) tidx = 1;
        else if (a == T2) tidx = 2;
        else              tidx = -1;
    end
endfunction

integer ti;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        ld_done_r <= 0; ldcnt <= 0; ldbusy <= 0;
        nload <= 0; nthumb <= 0; nhd <= 0; hd_any <= 0; buf_any <= 3'b000;
        hd_off_r <= 32'hDEAD_BEEF;
        buf_off[0] <= 32'hDEAD_BEEF;
        buf_off[1] <= 32'hDEAD_BEEF;
        buf_off[2] <= 32'hDEAD_BEEF;
    end else begin
        ld_done_r <= 1'b0;
        if (ld_req && !ldbusy) begin
            ldbusy   <= 1'b1;
            ldcnt    <= 0;
            cap_off  <= ld_off;
            cap_base <= ld_base;
            cap_fmt  <= ld_fmt;
            cap_img  <= dut.ld_img_d;
            nload    <= nload + 1;
            if (ld_fmt) nhd <= nhd + 1; else nthumb <= nthumb + 1;
        end else if (ldbusy) begin
            ldcnt <= ldcnt + 1;
            if (ldcnt == (cap_fmt ? LAT_H : LAT_T)) begin
                ld_done_r <= 1'b1;
                ldbusy    <= 1'b0;
                // 装载落盘：按 ld_base 决定写进哪块缓冲
                ti = tidx(cap_base);
                if (cap_fmt) begin
                    hd_off_r <= cap_off;
                    hd_any   <= 1'b1;
                end else if (ti >= 0 && ti <= 2) begin
                    buf_off[ti] <= cap_off;
                    buf_any[ti] <= 1'b1;
                end
                $display("      [load] %s off=%h -> %s (img_d=%0d base=%0d rot=%0d st=%0d)",
                         cap_fmt ? "HD  " : "THUM", cap_off,
                         cap_fmt ? "HD   " :
                         (ti == 0) ? "THUMB0" : (ti == 1) ? "THUMB1" :
                         (ti == 2) ? "THUMB2" : "BAD!!",
                         cap_img, O_base, dut.rot, st_state);
            end
        end
    end
end

//------------------------------------------------------------------------------
//  假帧源：compose_en=1 时每帧 500 拍
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
        // ★ 2026-11-02：compose_en=0 时必须把 fcnt 一起清零。
        //   原来只清 ps_start/busy：如果 compose_en 是在"ps_start 高电平中途"
        //   掉下去的（S_OSD 退出就是这样），fcnt 会停在 1..500 的中间值；
        //   恢复后 `fcnt==0` 和 `fcnt<0` 两个分支都不成立 ⇒ **帧永久停住**，
        //   表现为 S_LOW 里 hd_ready=1 却永远等不到 tick（本轮就撞上了）。
        ps_start <= 0; busy <= 0; fcnt <= 0;
    end
end

//------------------------------------------------------------------------------
//  不变式：rot == base % 3（每拍查）
//------------------------------------------------------------------------------
integer rot_err = 0;
always @(posedge clk) begin
    if (!rst && ((dut.base % 3) != dut.rot)) begin
        if (rot_err < 5)
            $display("  [FAIL] rot 与 base 不同步: base=%0d rot=%0d (期望 %0d) t=%0t",
                     dut.base, dut.rot, dut.base % 3, $time);
        rot_err = rot_err + 1;
    end
end

//------------------------------------------------------------------------------
//  判据
//------------------------------------------------------------------------------
integer errors = 0;
integer t0c, rel, absd, adr, havem, bad;

function [1:0] ptidx;
    input integer r;
    input integer rotv;
    integer j;
    begin
        j = r + rotv;
        if (j >= 3) j = j - 3;
        ptidx = j[1:0];
    end
endfunction

// rel + rot → 该落在哪块缩略图缓冲（= DUT 里 thumb_at 的判据）
function [20:0] ptaddr;
    input integer r;
    input integer rotv;
    integer j;
    begin
        j = r + rotv;
        if (j >= 3) j = j - 3;
        case (j)
            0:       ptaddr = T0;
            1:       ptaddr = T1;
            default: ptaddr = T2;
        endcase
    end
endfunction

// 一条记录的完整核对
task chk;
    input integer exp_abs;
    input [255:0] tag;
    begin
        absd  = O_base + st_img;
        bad   = 0;
        // ① 状态必须回到静止高清
        if (st_state != ST_HD) begin
            $display("  [FAIL] %0s: 没回到 S_HD（state=%0d）", tag, st_state);
            bad = 1;
        end
        // ② 当前图号
        if (absd != exp_abs) begin
            $display("  [FAIL] %0s: 图号错 abs=%0d 期望 %0d (base=%0d img=%0d)",
                     tag, absd, exp_abs, O_base, st_img);
            bad = 1;
        end
        // ③ 显示的是高清，且 HD 槽内容 == 当前图
        if (!a_hd) begin
            $display("  [FAIL] %0s: 静止态却是低清（a_hd=0）", tag);
            bad = 1;
        end
        if (!hd_any || (hd_off_r != off_of(exp_abs))) begin
            $display("  [FAIL] %0s: HD 槽内容错 hd_off=%h 期望 %h（hd_valid=%0d hd_img=%0d）",
                     tag, hd_off_r, off_of(exp_abs), dut.hd_valid, dut.hd_img);
            bad = 1;
        end
        // ④ 三张缩略图各就各位
        havem = (NIMG - O_base > 3) ? 3 : (NIMG - O_base);
        for (rel = 0; rel < 3; rel = rel + 1) begin
            adr = ptidx(rel, dut.rot);
            if (rel < havem) begin
                if (buf_off[adr] != off_of(O_base + rel)) begin
                    $display("  [FAIL] %0s: 缩略图 rel%0d(buf%0d) = %h 期望 %h (=第%0d张)",
                             tag, rel, adr, buf_off[adr], off_of(O_base + rel), O_base + rel);
                    bad = 1;
                end
            end
        end
        if (bad == 0)
            $display("  ok   %0s: base=%0d rot=%0d img=%0d abs=%0d  (thumb %0d, hd %0d)",
                     tag, O_base, dut.rot, st_img, absd, nthumb, nhd);
        errors = errors + bad;
    end
endtask

//------------------------------------------------------------------------------
//  动作
//------------------------------------------------------------------------------
task key_edge;
    input integer is_next;
    integer n0, nt0, nh0;
    begin
        n0 = nload; nt0 = nthumb; nh0 = nhd;
        @(negedge clk);
        if (is_next) key_next = 1; else key_prev = 1;
        @(negedge clk);
        key_next = 0; key_prev = 0;
        // 等离开静止态（末张/首张的"置灰"不会有反应）
        t0c = 0;
        while ((st_state == ST_HD || st_state == ST_STUK) && t0c < 300) begin
            @(posedge clk); t0c = t0c + 1;
        end
        // 等回到静止态
        t0c = 0;
        while (st_state != ST_HD && st_state != ST_STUK && t0c < 400_000) begin
            @(posedge clk); t0c = t0c + 1;
        end
        repeat (60) @(posedge clk);
        $display("      -> 本次装载 %0d 次（缩略图 %0d / 高清 %0d）",
                 nload - n0, nthumb - nt0, nhd - nh0);
    end
endtask

task do_next; input dummy; begin key_edge(1); end endtask
task do_prev; input dummy; begin key_edge(0); end endtask

//------------------------------------------------------------------------------
//  ★ 2026-11-02 【菜单通道】OSD 相册页的"上一张/下一张"
//------------------------------------------------------------------------------
//  这一段是本轮新增的核心覆盖：以前台子把 album_prev/album_next 钉死 0，
//  菜单那条路**一次都没被测过**，所以"第 4 张之后逻辑混乱"能一直躺在代码里。
//  它和主页共用同一套 slide_*/swap_to，判据也共用 —— 只是"静止态"从 S_HD
//  换成 S_OSD，屏幕上那层是缩略图而不是高清。
task menu_swap;
    input integer is_next;
    integer n0, t1, h1;
    begin
        n0 = nload; t1 = nthumb; h1 = nhd;
        @(negedge clk);
        if (is_next) album_next = 1; else album_prev = 1;
        @(negedge clk);
        album_next = 0; album_prev = 0;
        // 滑窗会离开 S_OSD（S_ONE / S_ONE_W），先等回来
        t0c = 0;
        while ((st_state != ST_OSD) && (t0c < 400_000)) begin
            @(posedge clk); t0c = t0c + 1;
        end
        // 再等一个整帧周期 —— 窗口内移动是 tick 上提交的，少等就看不到结果
        repeat (700) @(posedge clk);
        $display("      -> 菜单换图：装载 %0d 次（缩略图 %0d / 高清 %0d）",
                 nload - n0, nthumb - t1, nhd - h1);
    end
endtask

task menu_chk;
    input integer exp_abs;
    input [255:0] tag;
    begin
        absd = O_base + st_img;
        bad  = 0;
        if (st_state != ST_OSD) begin
            $display("  [FAIL] %0s: 不在 S_OSD（state=%0d）", tag, st_state);
            bad = 1;
        end
        if (absd != exp_abs) begin
            $display("  [FAIL] %0s: 图号错 abs=%0d 期望 %0d (base=%0d img=%0d)",
                     tag, absd, exp_abs, O_base, st_img);
            bad = 1;
        end
        // 底层必须指向"当前这张"的缩略图缓冲
        if (a_base != ptaddr(st_img, dut.rot)) begin
            $display("  [FAIL] %0s: a_base=%h 期望 %h (rel=%0d rot=%0d)",
                     tag, a_base, ptaddr(st_img, dut.rot), st_img, dut.rot);
            bad = 1;
        end
        // 三张缩略图各就各位（和主页同一条判据）
        havem = (NIMG - O_base > 3) ? 3 : (NIMG - O_base);
        for (rel = 0; rel < 3; rel = rel + 1) begin
            adr = ptidx(rel, dut.rot);
            if (rel < havem) begin
                if (buf_off[adr] != off_of(O_base + rel)) begin
                    $display("  [FAIL] %0s: 缩略图 rel%0d(buf%0d) = %h 期望 %h (=第%0d张)",
                             tag, rel, adr, buf_off[adr], off_of(O_base + rel), O_base + rel);
                    bad = 1;
                end
            end
        end
        if (bad == 0)
            $display("  ok   %0s: base=%0d rot=%0d img=%0d abs=%0d (thumb %0d)",
                     tag, O_base, dut.rot, st_img, absd, nthumb);
        errors = errors + bad;
    end
endtask

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
integer k, e0, abs0;
integer zig [0:15];
integer nzig;

initial begin
    $display("== tb_switch：NIMG=%0d，装载内容/位置逐项核对 ==", NIMG);
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (100) @(posedge clk);
    ld_sd_ready = 1;

    $display("  -- 等开机装完 --");
    t0c = 0;
    while (st_state != ST_HD && t0c < 400_000) begin @(posedge clk); t0c = t0c + 1; end
    repeat (50) @(posedge clk);
    chk(0, "S0 开机");

    //---------------------------------------------------------------- S1 正向
    $display("  -- S1 正向走完 --");
    for (k = 1; k < NIMG; k = k + 1) begin
        $display("    next -> %0d", k);
        do_next(0);
        chk(k, "S1");
    end

    //---------------------------------------------------------------- S2 末张
    $display("  -- S2 末张再按（应原地不动）--");
    e0 = nload;
    do_next(0);
    if (O_base + st_img != NIMG - 1) begin
        $display("  [FAIL] S2 末张越界 abs=%0d", O_base + st_img);
        errors = errors + 1;
    end else
        $display("  ok   S2 末张保持 abs=%0d，装载 %0d 次", O_base + st_img, nload - e0);
    if (nload != e0) begin
        $display("  [FAIL] S2 末张还发了 %0d 次装载", nload - e0);
        errors = errors + 1;
    end

    //---------------------------------------------------------------- S3 反向
    $display("  -- S3 反向走回 --");
    for (k = NIMG - 2; k >= 0; k = k - 1) begin
        $display("    prev -> %0d", k);
        do_prev(0);
        chk(k, "S3");
    end

    //---------------------------------------------------------------- S8 首张 prev
    $display("  -- S8 首张再按 prev（应原地不动）--");
    e0 = nload;
    do_prev(0);
    if (O_base + st_img != 0) begin
        $display("  [FAIL] S8 首张按 prev 跳到了 abs=%0d（应停在 0）", O_base + st_img);
        errors = errors + 1;
    end else
        $display("  ok   S8 首张保持 abs=0，装载 %0d 次", nload - e0);
    if (nload != e0) begin
        $display("  [FAIL] S8 首张还发了 %0d 次装载", nload - e0);
        errors = errors + 1;
    end

    //---------------------------------------------------------------- S4 交替
    if (NIMG >= 6) begin
        $display("  -- S4 正反交替 --");
        do_next(0); chk(1, "S4a");
        do_next(0); chk(2, "S4b");
        do_prev(0); chk(1, "S4c");     // 2->1
        do_next(0); chk(2, "S4d");     // 1->2
        do_next(0); chk(3, "S4e");     // 2->3（第一次滑动）
        do_prev(0); chk(2, "S4f");
        do_next(0); chk(3, "S4g");
        do_next(0); chk(4, "S4h");
        do_prev(0); chk(3, "S4i");
        do_prev(0); chk(2, "S4j");
        do_prev(0); chk(1, "S4k");     // 往回滑动
        do_next(0); chk(2, "S4l");
    end

    //---------------------------------------------------------------- S9 菜单通道
    //  ★ 2026-11-02 新增：OSD 相册页"上一张/下一张"走的是**同一套**滑动窗口。
    //  这一段把用户报的"第 4 张之后 HD 和缩略图逻辑混乱"直接跑一遍：
    //  菜单里一路 next 到末张（含每一次滑窗）、再一路 prev 回到首张、
    //  最后退出菜单，验高清确实按当前图重装了。
    $display("  -- S9 OSD 菜单相册页换图（与主页共用同一套逻辑）--");
    osd_enter = 1; @(negedge clk); osd_enter = 0;
    t0c = 0;
    while ((st_state != ST_OSD) && (t0c < 40_000)) begin @(posedge clk); t0c = t0c + 1; end
    if (st_state != ST_OSD) begin
        $display("  [FAIL] S9 进不去 S_OSD（state=%0d）", st_state);
        errors = errors + 1;
    end
    repeat (1500) @(posedge clk);          // 等 osd_ph 走到 1（要一个 !busy 的窗口）
    menu_chk(O_base + st_img, "S9 进菜单");

    abs0 = O_base + st_img;
    for (k = 0; k < NIMG + 2; k = k + 1) begin
        if (abs0 < NIMG - 1) begin
            abs0 = abs0 + 1;
            $display("    menu next -> %0d", abs0);
            menu_swap(1);
            menu_chk(abs0, "S9n");
        end else begin
            e0 = nload;
            menu_swap(1);
            if (nload != e0) begin
                $display("  [FAIL] S9 末张贴边还发了 %0d 次装载", nload - e0);
                errors = errors + 1;
            end
            if ((O_base + st_img) != NIMG - 1) begin
                $display("  [FAIL] S9 末张越界 abs=%0d（应停在 %0d）", O_base + st_img, NIMG - 1);
                errors = errors + 1;
            end
        end
    end

    for (k = 0; k < NIMG + 2; k = k + 1) begin
        if (abs0 > 0) begin
            abs0 = abs0 - 1;
            $display("    menu prev -> %0d", abs0);
            menu_swap(0);
            menu_chk(abs0, "S9p");
        end else begin
            e0 = nload;
            menu_swap(0);
            if (nload != e0) begin
                $display("  [FAIL] S9 首张贴边还发了 %0d 次装载", nload - e0);
                errors = errors + 1;
            end
            if ((O_base + st_img) != 0) begin
                $display("  [FAIL] S9 首张按 prev 跳到 abs=%0d（应停在 0）", O_base + st_img);
                errors = errors + 1;
            end
        end
    end

    // 菜单里 KEY3/KEY4 是"光标上下"，绝不能顺带换图
    //   （App.v 只把它们对 zoom 做了屏蔽，没有对 OSD 屏蔽 ⇒ viewer_ctrl
    //     在 S_OSD 里照样收得到脉冲；换图判据必须按状态分流）
    $display("  -- S9b 菜单里按上下键（不许换图）--");
    e0 = nload;
    @(negedge clk); key_next = 1; @(negedge clk); key_next = 0;
    repeat (700) @(posedge clk);
    @(negedge clk); key_prev = 1; @(negedge clk); key_prev = 0;
    repeat (700) @(posedge clk);
    if ((O_base + st_img) != abs0) begin
        $display("  [FAIL] S9b 菜单里按上下键换了图：abs=%0d（应停在 %0d）", O_base + st_img, abs0);
        errors = errors + 1;
    end else
        $display("  ok   S9b 菜单里按上下键不换图（abs=%0d）", abs0);
    if (nload != e0) begin
        $display("  [FAIL] S9b 菜单里按上下键触发了 %0d 次装载", nload - e0);
        errors = errors + 1;
    end

    // 退出菜单：换过图 ⇒ HD 槽已作废 ⇒ 必须重装，且装完必须就是当前这张
    $display("  -- S9 退出菜单，验高清重装 --");
    osd_exit = 1; @(negedge clk); osd_exit = 0;
    t0c = 0;
    while ((st_state != ST_HD) && (t0c < 60_000)) begin
        @(posedge clk); t0c = t0c + 1;
        if ((t0c % 5000) == 0)
            $display("      [dbg] t=%0d state=%0d osd_ph=%0d hd_valid=%0d hd_img=%0d tgt=%0d img=%0d ld_go=%0d ld_fin=%0d hd_ready=%0d compose=%0d",
                     t0c, st_state, dut.osd_ph, dut.hd_valid, dut.hd_img, dut.tgt_idx, dut.img_idx,
                     dut.ld_go, dut.ld_fin, dut.hd_ready, compose_en);
    end
    repeat (80) @(posedge clk);
    chk(abs0, "S9 退出");

    //---------------------------------------------------------- S10 退出菜单后继续
    //  这一条是给"上板回归"用的：菜单里滑过窗、one_osd/osd_pv 都动过，
    //  退出后主页必须还能正常切（不能残留"直切"标志把 S_ONE 引到 S_OSD）。
    $display("  -- S10 退出菜单后主页继续切 --");
    abs0 = O_base + st_img;
    for (k = 0; k < 5; k = k + 1) begin
        if (abs0 < NIMG - 1) begin
            abs0 = abs0 + 1;
            $display("    home next -> %0d", abs0);
            do_next(0);
            chk(abs0, "S10");
        end
    end

    //---------------------------------------------------------- S11 兜底自愈
    //  ★ 2026-11-02：上板报"点切换卡住、不播动画、永久卡死"。
    //  最可能的一条是装载器不给（ld_can_load=0）时 S_ONE 里死等 ld_fin。
    //  这一段把它做成可复现的：走到末张 → 掐断装载器 → 按 next（触发右滑窗）
    //  → 必须能在兜底阈值内回到 S_HD，而不是永久卡住。
    if (NIMG >= 4) begin
        $display("  -- S11 装载器不给时必须自愈（不许永久卡）--");
        // 先回第 0 张，再走到窗口右端（abs=2，base=0）——
        // 只有这个位置按 next 才会触发**右滑窗**进 S_ONE
        k = 0;
        while (((O_base + st_img) > 0) && (k < NIMG + 4)) begin
            do_prev(0); k = k + 1;
        end
        do_next(0);        // abs=1
        do_next(0);        // abs=2
        abs0 = O_base + st_img;
        noload = 1;        // ← 从这里开始装载器不再给
        e0 = nload;
        @(negedge clk); key_next = 1; @(negedge clk); key_next = 0;
        t0c = 0;
        while ((st_state != ST_HD) && (t0c < 400_000)) begin @(posedge clk); t0c = t0c + 1; end
        noload = 0;
        if (st_state != ST_HD) begin
            $display("  [FAIL] S11 卡死：state=%0d，等了 %0d 拍", st_state, t0c);
            errors = errors + 1;
        end else
            $display("  ok   S11 自愈：%0d 拍 ≈ %0d ms 回到 S_HD（abs 保持 %0d）",
                     t0c, t0c / 50000, O_base + st_img);
        repeat (3000) @(posedge clk);
    end

    $display("  总计：装载 %0d 次（缩略图 %0d / 高清 %0d），rot 不同步 %0d 次", nload, nthumb, nhd, rot_err);
    if (rot_err != 0) errors = errors + rot_err;
    $display("== errors=%0d %s ==", errors, (errors == 0) ? "PASS" : "FAIL");
    $finish;
end

initial begin
    #400_000_000;
    $display("TIMEOUT");
    $finish;
end

endmodule
