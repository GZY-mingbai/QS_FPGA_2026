//==============================================================================
//  tb_info_trans.v —— 信息条在"切换动画期间不能消失"的专项回归
//==============================================================================
//
//  【要证的两件事】
//    ① 静止高清（S_HD/S_STUCK/S_HDF）时 info_show 必须为 1；
//    ② 按下切换键之后，从 S_ARM → S_TRANS → S_LOW 的**全过程** info_show
//       必须保持 1（用户在动画里也要看到分辨率/倒计时信息条）。
//
//  【上板现象】打开信息条后按切换键，动画一开始信息条就消失，动画结束才回来。
//  【预期根因】viewer_ctrl 的
//        assign info_show = info_on & ((state==S_HD)||(state==S_STUCK)||(state==S_HDF));
//    过渡用的 S_ARM / S_TRANS / S_LOW 全都不在列表里。
//
//  【为什么不能只放宽这一行】
//    viewer_scene 里 OSD 是**整体替换 B 层**：
//        pix_b_eff = osd_on ? osd_pix : pix_b;
//    过渡期间 B 层正是动画前景，直接替换会把动画吃掉。所以本台只负责
//    "控制侧 status"，叠加结构由 tb_info_overlay.v 单独验证。
//
//  跑法（工作目录 = APP/）：
//    iverilog -g2012 -s tb_info_trans -o /tmp/tb_it.vvp \
//        tb_info_trans.v viewer_ctrl.v viewer_fx.v && vvp /tmp/tb_it.vvp
//==============================================================================
`timescale 1ns / 1ps

module tb_info_trans;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

localparam integer T_END   = 12;
localparam integer T_END_F = 6;

// ---- DUT 输入 ----
reg  key_prev = 0, key_next = 0, key_fx = 0;
reg  busy = 0, ps_start = 0, pipe_ce = 1;
wire ld_req, ld_fmt;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
reg  ld_can_load = 1, ld_done = 0, ld_err = 0, ld_sd_ready = 0;
reg  [2:0] fx_sel = 3'd3;              // 固定 fx3（缩小交换），最贴近用户描述
reg        info_on = 1'b1;             // ★ 信息条打开

wire compose_en, fmt565, a_hd, a_en, b_en;
wire [20:0] a_base, b_base;
wire [31:0] pos_a, size_a, pos_b, size_b;
wire [39:0] k_a, k_b;
wire [7:0]  alpha_b;
wire [1:0]  st_img;
wire [2:0]  st_fx;
wire [3:0]  st_state;
wire        info_show;
wire [15:0] res_w, res_h;
wire [5:0]  auto_remain;
wire [8:0]  auto_bar;

viewer_ctrl #(
    .T_END       (T_END),
    .T_END_F     (T_END_F),
    .ARM_FRAMES  (2),
    .FRAME_CYC   (500),                 // 仿真压缩：一"帧" 500 拍
    .AUTO_SEC_CYC(2000),
    .THUMB0_ADDR (21'd614400), .THUMB1_ADDR (21'd652800),
    .THUMB2_ADDR (21'd691200), .HD_ADDR     (21'd729600)
) dut (
    .clk(clk), .rst(rst),
    // ★ 2026-11-01：IMG0/1/2_OFFSET 已从参数改成端口（相册走 FAT，
    //   这几个台只关心几何/过渡，偏移给固定值即可）
    .I_img_off0 (32'd1000), .I_img_off1 (32'd2000), .I_img_off2 (32'd3000),
    .I_img_off3(32'd0),
    .I_have(3'd3), .I_alb_ready(1'b1), .I_jump(1'b0), .I_jump_idx(8'd0), .O_ld_abort(),
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(1'b0), .key_osd_exit(1'b0), .osd_fade(8'd255),
    .osd_on(), .osd_allow(),
    .busy(busy), .ps_start(ps_start), .pipe_ce(pipe_ce),
    .ld_req(ld_req), .ld_fmt(ld_fmt), .ld_off(ld_off), .ld_len(ld_len), .ld_base(ld_base),
    .ld_can_load(ld_can_load), .ld_done(ld_done), .ld_err(ld_err),
    .ld_sd_ready(ld_sd_ready),
    .compose_en(compose_en), .fps_dbg(), .fmt565(fmt565),
    .a_hd(a_hd), .a_en(a_en), .a_base(a_base),
    .b_en(b_en), .b_base(b_base),
    .pos_a(pos_a), .size_a(size_a), .k_a(k_a),
    .pos_b(pos_b), .size_b(size_b), .k_b(k_b), .alpha_b(alpha_b),
    .st_img(st_img), .st_fx(st_fx), .st_state(st_state),
    .fx_sel(fx_sel), .auto_mode(3'd0),.info_on(info_on),
    .album_prev(1'b0), .album_next(1'b0), .osd_trig(1'b0), .music_done(1'b0),
    .ld_src_w(16'd640), .ld_src_h(16'd480),
    .res_w(res_w), .res_h(res_h), .auto_remain(auto_remain), .auto_bar(auto_bar),
    .info_show(info_show), // 2026-10-31: tie off ports added after this bench was written (were floating Z)
    .I_win4(1'b0), .za_x(10'd320), .za_y(10'd240), .zoom_on(1'b0), .preview_req(1'b0), .load_req(1'b0), .zoom_val(8'h10)
);

//------------------------------------------------------------------------------
//  假装载器
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
            if (ldcnt == 200) begin
                ld_done <= 1; ldbusy <= 0;
            end
        end
    end
end

//------------------------------------------------------------------------------
//  假帧源：compose_en=1 时周期给一帧（1000 拍扫描 + 300 拍空闲）
//------------------------------------------------------------------------------
integer fcnt;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        ps_start <= 0; busy <= 0; fcnt <= 0;
    end else begin
        if (compose_en) begin
            if (ps_start) begin
                if (fcnt == 1000) begin ps_start <= 0; busy <= 0; fcnt <= 0; end
                else fcnt <= fcnt + 1;
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
//  断言工具
//------------------------------------------------------------------------------
integer fails = 0, checks = 0;
task chk;
    input [8*64:1] name;
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %0d", name, got);
    end
endtask

task kpress;
    begin
        @(negedge clk); key_next = 1'b1;
        @(negedge clk); key_next = 1'b0;
        repeat (6) @(posedge clk);
    end
endtask

//------------------------------------------------------------------------------
//  全程监视：状态一旦进入 S_ARM/S_TRANS/S_LOW，就检查 info_show
//------------------------------------------------------------------------------
integer vis_bad = 0;
integer n_arm = 0, n_trans = 0, n_low = 0;
reg     seen_trans_end = 0;
always @(posedge clk) begin
    if (!rst) begin
        if (st_state == 4'd7) begin            // S_ARM
            n_arm = n_arm + 1;
            if (info_show !== 1'b1 && !seen_trans_end) vis_bad = vis_bad + 1;
        end
        if (st_state == 4'd3) begin            // S_TRANS
            n_trans = n_trans + 1;
            if (info_show !== 1'b1) vis_bad = vis_bad + 1;
        end
        if (st_state == 4'd2) begin            // S_LOW
            n_low = n_low + 1;
            if (info_show !== 1'b1) vis_bad = vis_bad + 1;
        end
        if (st_state == 4'd5) seen_trans_end = 1;   // 回到 HD
        if (st_state != 4'd5) seen_trans_end = 0;
    end
end

initial begin
    $dumpfile("tb_info_trans.vcd");
    $dumpvars(0, tb_info_trans);

    repeat (4) @(posedge clk);
    rst = 0;
    ld_sd_ready = 1;

    // 上电：装 3 张缩略图 + HD0 → S_HD
    wait (st_state == 4'd5);
    repeat (50) @(posedge clk);
    $display("=== A. 静止高清 ===");
    chk("A1 info_show in HD", info_show, 1);

    // 按下一张，进入动画
    $display("=== B. 切换动画全程 ===");
    kpress;
    repeat (4) @(posedge clk);
    wait (st_state == 4'd5);                    // 等回到 HD
    repeat (20) @(posedge clk);

    $display("    计数：S_ARM=%0d 拍, S_TRANS=%0d 拍, S_LOW=%0d 拍",
             n_arm, n_trans, n_low);
    chk("B1 entered S_ARM",  (n_arm   > 0), 1);
    chk("B2 entered S_TRANS", (n_trans > 0), 1);
    chk("B3 info_show never low during animation", vis_bad, 0);
    chk("B4 info_show back in HD", info_show, 1);

    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED    %0d of %0d checks", fails, checks);
    $finish;
end

endmodule
