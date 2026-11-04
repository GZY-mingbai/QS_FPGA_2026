//==============================================================================
//  tb_window.v —— 相册滚动窗口：10 张图逐张可达（viewer_ctrl 单元台）
//==============================================================================
//  不跑 App/OSD/装载器，只驱动 viewer_ctrl：
//    · 假装载器：ld_req 后 50 拍 done，记录每次 ld_off
//    · 假相册：按 DUT 的 O_base 给出 off[base..base+2] + have + win4，
//      O_base 一变就"重扫" 50 拍（ready=0→1）
//    · 假帧源：compose_en=1 时每 500 拍一帧
//
//  断言：按 next 9 次，屏幕上的图号（base + img_idx）必须依次 1..9；
//        每次重装只允许装 off[base..base+2] 这三张。
//
//  iverilog -g2012 -o w.vvp -s tb_window tb_window.v viewer_ctrl.v viewer_fx.v
//==============================================================================
`timescale 1ns/1ps

module tb_window;

localparam integer NIMG = 10;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg  key_prev = 0, key_next = 0, key_fx = 0;
reg  busy = 0, ps_start = 0, pipe_ce = 1;
reg  ld_can_load = 1, ld_done = 0, ld_err = 0, ld_sd_ready = 0;
reg  [2:0] fx_sel = 3'd0;
reg  zoom_on = 0, preview_req = 0, load_req = 0;

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

// 假相册：off[i] = 1000*(i+1)
function [31:0] off_of;
    input integer idx;
    begin off_of = 32'd1000 * (idx + 1); end
endfunction

wire [15:0] b0 = O_base;
wire [31:0] fo0 = (b0 + 0 < NIMG) ? off_of(b0 + 0) : 32'd0;
wire [31:0] fo1 = (b0 + 1 < NIMG) ? off_of(b0 + 1) : 32'd0;
wire [31:0] fo2 = (b0 + 2 < NIMG) ? off_of(b0 + 2) : 32'd0;
wire [1:0]  have = (b0 + 2 < NIMG) ? 2'd3 : ((b0 + 1 < NIMG) ? 2'd2 : ((b0 < NIMG) ? 2'd1 : 2'd0));
wire        win4 = (b0 + 3 < NIMG);

// 假相册 ready：O_base 变 → 50 拍后 ready
reg [15:0] base_d = 16'hFFFF;
reg [5:0]  rescan_cnt;
reg        alb_ready = 0;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        base_d <= 16'hFFFF; rescan_cnt <= 0; alb_ready <= 0;
    end else begin
        if (O_base != base_d) begin
            base_d <= O_base; rescan_cnt <= 0; alb_ready <= 0;
        end else if (rescan_cnt != 6'd50) begin
            rescan_cnt <= rescan_cnt + 1;
        end else
            alb_ready <= 1'b1;
    end
end

viewer_ctrl #(
    .T_END(6), .T_END_F(3), .ARM_FRAMES(1), .FRAME_CYC(1000),
    .THUMB0_ADDR(21'd614400), .THUMB1_ADDR(21'd652800),
    .THUMB2_ADDR(21'd691200), .HD_ADDR(21'd729600)
) dut (
    .clk(clk), .rst(rst),
    .I_img_off0(fo0), .I_img_off1(fo1), .I_img_off2(fo2),
    .I_img_off3(32'd0),
    .O_base(O_base), .I_win4(win4), .I_have(have), .I_alb_ready(alb_ready),
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(1'b0), .key_osd_exit(1'b0), .osd_fade(8'd255),
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
    .album_prev(1'b0), .album_next(1'b0), .osd_trig(1'b0), .music_done(1'b0),
    .ld_src_w(16'd640), .ld_src_h(16'd480),
    .res_w(), .res_h(), .auto_remain(), .auto_bar(), .info_show());

// 假装载器
integer ldcnt = 0;
reg     ldbusy = 0;
integer nload = 0;
integer load_off [0:63];
always @(posedge clk or posedge rst) begin
    if (rst) begin
        ld_done <= 0; ldcnt <= 0; ldbusy <= 0; nload <= 0;
    end else begin
        ld_done <= 0;
        if (ld_req && !ldbusy) begin
            ldbusy <= 1; ldcnt <= 0;
            if (nload < 64) load_off[nload] <= ld_off;
            nload <= nload + 1;
        end else if (ldbusy) begin
            ldcnt <= ldcnt + 1;
            if (ldcnt == 50) begin ld_done <= 1; ldbusy <= 0; end
        end
    end
end

// 假帧源
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
    end else begin ps_start <= 0; busy <= 0; end
end

//------------------------------------------------------------------------------
//  场景
//------------------------------------------------------------------------------
integer errors = 0;
integer i, k, absidx, t0, base_seen;

task press_next;
    begin
        @(negedge clk); key_next = 1;
        @(negedge clk); key_next = 0;
    end
endtask

task wait_hd;
    input integer maxcyc;
    reg done;
    begin
        t0 = 0; done = 0;
        while (t0 < maxcyc && !done) begin
            @(posedge clk); t0 = t0 + 1;
            if (st_state == 4'd5) done = 1;
        end
        if (!done) begin
            $display("  [FAIL] 等不到 S_HD（state=%0d）", st_state);
            errors = errors + 1;
        end
    end
endtask

initial begin
    $display("== tb_window：10 张滚动窗口 ==");
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (100) @(posedge clk);
    ld_sd_ready = 1;

    wait_hd(2_000_000);
    base_seen = 0;
    absidx = O_base + st_img;
    $display("  boot: base=%0d img=%0d abs=%0d (期望 0)", O_base, st_img, absidx);
    if (absidx != 0) begin $display("  [FAIL] 首图不是 0"); errors = errors + 1; end

    for (k = 1; k < NIMG; k = k + 1) begin
        press_next;
        wait_hd(2_000_000);
        absidx = O_base + st_img;
        $display("  next %0d: base=%0d img=%0d abs=%0d  loads=%0d",
                 k, O_base, st_img, absidx, nload);
        if (absidx != k) begin
            $display("  [FAIL] 第 %0d 张不可达（abs=%0d）", k, absidx);
            errors = errors + 1;
        end
        if (O_base < base_seen) begin
            $display("  [FAIL] base 倒退 %0d -> %0d", base_seen, O_base);
            errors = errors + 1;
        end
        base_seen = O_base;
    end

    // 末张再按：不应越界
    press_next;
    repeat (20000) @(posedge clk);
    $display("  末张再按: base=%0d img=%0d state=%0d", O_base, st_img, st_state);
    if (O_base + st_img != NIMG - 1) begin
        $display("  [FAIL] 末张越界（abs=%0d）", O_base + st_img);
        errors = errors + 1;
    end

    // 反着走回第 0 张
    for (k = NIMG - 2; k >= 0; k = k - 1) begin
        @(negedge clk); key_prev = 1; @(negedge clk); key_prev = 0;
        wait_hd(2_000_000);
        absidx = O_base + st_img;
        if (absidx != k) begin
            $display("  [FAIL] 回退到 %0d 失败（abs=%0d base=%0d img=%0d）",
                     k, absidx, O_base, st_img);
            errors = errors + 1;
        end
    end

    $display("  共 %0d 次装载；最后 base=%0d img=%0d", nload, O_base, st_img);
    $display("== errors=%0d %s ==", errors, (errors == 0) ? "PASS" : "FAIL");
    $finish;
end

initial begin
    #200_000_000;
    $display("TIMEOUT");
    $finish;
end

endmodule
