//==============================================================================
//  tb_ctrl.v —— 控制状态机的流程验证（不含真实场景）
//==============================================================================
//  用一个假帧源（ps_start/busy）和假装载器（ld_done）驱动 viewer_ctrl，
//  检查：上电依次装 3 张缩略图 + HD0 → 进 HD；按 next → TRANS(32 帧) → 回 HD。
//==============================================================================

`timescale 1ns / 1ps

module tb_ctrl;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

// ---- DUT 端口 ----
reg  key_prev = 0, key_next = 0, key_fx = 0;
reg  busy = 0, ps_start = 0, pipe_ce = 1;
wire ld_req, ld_fmt;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
reg  ld_can_load = 1, ld_done = 0, ld_err = 0, ld_sd_ready = 0;
wire compose_en, fmt565, a_hd, a_en, b_en;
wire [20:0] a_base, b_base;
wire [31:0] pos_a, size_a, pos_b, size_b;
wire [39:0] k_a, k_b;
wire [7:0]  alpha_b;
wire [1:0]  st_img;
wire [2:0]  st_fx;
wire [3:0]  st_state;

// ★ 2026-10-27：viewer_ctrl 新增的输入端口原来**一个都没连**。
//   iverilog 对未连接的 input 端口给 'z'；而 viewer_ctrl 现在有
//   `fx_id <= fx_sel`（每拍跟随菜单设置），z 会一路传进 viewer_fx
//   ⇒ pos/size/k 全变 x/z ⇒ 整个测试台大面积假 FAIL（实测踩过）。
//   这里全部显式给确定值。key2 改效果不再由 viewer_ctrl 自己做，
//   而是"改设置"（fx_sel），与真实架构一致。
reg  [2:0] fx_sel = 3'd0;
// ★ 2026-10-30 缩放 OSD 驱动（用例里再拉高）
reg        zoom_on     = 1'b0;
reg        preview_req = 1'b0;
reg        load_req    = 1'b0;

viewer_ctrl #(
    .THUMB0_ADDR(21'd614400), .THUMB1_ADDR(21'd652800),
    .THUMB2_ADDR(21'd691200), .HD_ADDR(21'd729600)
) dut (
    .clk(clk), .rst(rst),
    // ★ 2026-11-01：IMG0/1/2_OFFSET 已从参数改成端口（相册走 FAT，
    //   这几个台只关心几何/过渡，偏移给固定值即可）
    .I_img_off0 (32'd1000), .I_img_off1 (32'd2000), .I_img_off2 (32'd3000),
    .I_img_off3(32'd0),
    // ★ 补 I_nimg：4cf563c 把 I_have 拆成 I_any/I_nimg，本台没跟上 ⇒ I_nimg 悬空成 Z
    //   ⇒ img_switch 的 `n_ex = (cr+1) < I_nimg` 变 X ⇒ 预取/换图判据失效
    //   （原本"没进 TRANS / 按键被吞"就是这么假 FAIL 的）。
    .I_any(1'b1), .I_nimg(3'd3), .I_alb_ready(1'b1), .I_jump(1'b0), .I_jump_idx(8'd0), .O_ld_abort(),
    .I_file_page(1'b0),              // ★ [FP-ABORT] 本台不测文件页
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(1'b0), .key_osd_exit(1'b0), .osd_fade(8'd255),
    .osd_on(), .osd_allow(),
    // ★ 2026-10-30 缩放 OSD：默认关着，保持原有行为（不连会得到 z）
    .zoom_on(zoom_on), .preview_req(preview_req), .load_req(load_req),
    .zoom_val(8'h10),
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
    // ★ 2026-10-27：这些原来全没连（= z），见上面声明处的说明
    .fx_sel(fx_sel), .auto_mode(3'd0),.info_on(1'b0),
    .album_prev(1'b0), .album_next(1'b0), .osd_trig(1'b0), .music_done(1'b0),
    .ld_src_w(16'd640), .ld_src_h(16'd480),
    .res_w(), .res_h(), .auto_remain(), .auto_bar(),
    .info_show(), // 2026-10-31: tie off ports added after this bench was written (were floating Z)
    .I_win4(1'b0), .za_x(10'd320), .za_y(10'd240)
);

//------------------------------------------------------------------------------
//  假装载器：收到 ld_req 后 200 拍回 done
//------------------------------------------------------------------------------
integer ldcnt = 0;
reg     ldbusy = 0;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        ld_done <= 0;
        ldcnt   <= 0;
        ldbusy  <= 0;
    end else begin
        ld_done <= 0;
        if (ld_req) begin
            ldbusy <= 1;
            ldcnt  <= 0;
        end else if (ldbusy) begin
            ldcnt <= ldcnt + 1;
            if (ldcnt == 200) begin
                ld_done <= 1;
                ldbusy  <= 0;
            end
        end
    end
end

//------------------------------------------------------------------------------
//  假帧源：compose_en=1 时周期性地给 1000 拍 ps_start
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
                // 空闲 300 拍后开下一帧
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
integer errors = 0;
integer last_state = -1;
integer trans_frames = 0;
integer arm_cycles = 0;            // S_ARM 停留拍数
integer arm_hold_violation = 0;    // ★ 2026-10-25：S_ARM 里不许再等装载（>10 帧就算违规）
reg     low_from_trans = 0;        // 这次 S_LOW 是"动画结束后装高清"进来的
reg     low_kick_seen = 0;         // 进来之后有没有把装载踢出去
integer low_no_kick = 0;           // 进了 S_LOW 却没踢装载（会干等 5.37s 才超时重试）
integer arm_seen = 0;

always @(posedge clk) begin
    if (!rst) begin
        // ★★ 2026-10-25 改：装载从"按下之前"挪到"动画之后" ★★
        //  · 断言 1（新）：S_ARM 不再等 0.5~1s 的 888 装载，
        //    只预热 ARM_FRAMES+1 帧 ⇒ 必须很快进 S_TRANS。
        //  · 断言 2（新）：过渡结束进 S_LOW 是**正常路径**，而且进去那一拍
        //    装载脉冲（ld_kick）必须已经发出来了，否则要干等 5.37s 超时。
        if (dut.hd_want) low_kick_seen = 1;
        if (st_state !== last_state) begin
            if (last_state == 3'd7 && st_state == 3'd3) begin
                $display("  [info] S_ARM 只停留 %0d 拍（不装载，直接进动画）", arm_cycles);
                if (arm_cycles > 10 * 400000) arm_hold_violation = arm_hold_violation + 1;
            end
            if (last_state == 3'd3 && st_state == 3'd2) begin
                low_from_trans = 1;
                low_kick_seen  = dut.hd_want;      // ld_kick 与 state 同拍更新
            end
            if (last_state == 3'd2 && st_state != 3'd2) begin
                if (low_from_trans && !low_kick_seen) low_no_kick = low_no_kick + 1;
                low_from_trans = 0;
            end
            if (st_state == 3'd7) arm_seen = 1;
            if (st_state == 3'd5) arm_seen = 0;
            $display("  [state] %0d -> %0d  (img=%0d fx=%0d fmt565=%b a_hd=%b hd_ready=%b)",
                     last_state, st_state, st_img, st_fx, fmt565, a_hd, dut.hd_ready);
            last_state = st_state;
            arm_cycles = (st_state == 3'd7) ? 0 : arm_cycles;
        end
        if (st_state == 3'd7) arm_cycles = arm_cycles + 1;
        if (st_state == 3) trans_frames = trans_frames + 1;
    end
end


//==============================================================================
//  ★ 2026-10-28：抓"切 S_HDF 之前那几个帧"的 B 几何
//==============================================================================
//  【为什么要它】上板现象：565(320×240) → HD 的**那一瞬**整幅右移 >4px。
//    若最后一帧 565 的 B 几何不是 (640×480)@(0,0)、k 不是 {16384,32768}，
//    切到满屏 888 时就会"缩放跳一下" —— 这里把切换前 4 帧逐帧打出来。
reg [31:0] h_size [0:3];
reg [31:0] h_pos  [0:3];
reg [39:0] h_k    [0:3];
integer    hi = 0, jj;
reg  [3:0] st_d = 4'd0;
always @(posedge clk) begin
    st_d <= dut.state;
    if (dut.tick) begin
        h_size[hi % 4] = size_b;
        h_pos [hi % 4] = pos_b;
        h_k   [hi % 4] = k_b;
        hi = hi + 1;
    end
    if ((dut.state == 4'd4) && (st_d != 4'd4)) begin
        $display("  [geom] 进入 S_HDF 前最近 4 帧的 B 层几何：");
        for (jj = 0; jj < 4; jj = jj + 1)
            $display("         size=(%0d,%0d) pos=(%0d,%0d) k=(%0d,%0d)",
                     h_size[jj][31:16], h_size[jj][15:0],
                     h_pos[jj][31:16],  h_pos[jj][15:0],
                     h_k[jj][39:20],    h_k[jj][19:0]);
    end
end

initial begin
    repeat (20) @(posedge clk);
    rst = 0;
    // 模拟 SD 卡初始化：过 500 拍才 ready（真实是几百 ms）
    repeat (500) @(posedge clk);
    ld_sd_ready = 1;
    $display("  [info] SD ready");

    //---- 上电：等进 HD（state 5），最多 2,000,000 拍 ----
    fork : wait_hd
        begin
            wait (st_state == 3'd5);
            disable wait_hd;
        end
        begin
            repeat (2000000) @(posedge clk);
            $display("  [FAIL] 上电进不了 HD");
            errors = errors + 1;
            disable wait_hd;
        end
    join
    $display("  [ ok ] 上电进入 HD，img=%0d", st_img);

    //---- 按 key2 两次：fx 应 0→1→2 ----
    @(negedge clk); key_fx = 1; fx_sel = (fx_sel == 3'd4) ? 3'd0 : fx_sel + 3'd1;
                    @(negedge clk); key_fx = 0;
    repeat (5) @(posedge clk);
    if (st_fx !== 3'd1) begin $display("  [FAIL] fx 没加到 1（=%0d）", st_fx); errors=errors+1; end
    @(negedge clk); key_fx = 1; fx_sel = (fx_sel == 3'd4) ? 3'd0 : fx_sel + 3'd1;
                    @(negedge clk); key_fx = 0;
    repeat (5) @(posedge clk);
    if (st_fx !== 3'd2) begin $display("  [FAIL] fx 没加到 2（=%0d）", st_fx); errors=errors+1; end
    $display("  [ ok ] key2 轮换 fx，当前 %0d", st_fx);

    //---- 按 key3：应进 TRANS（3），32 帧后回 HD（5）----
    trans_frames = 0;
    // ★ 按键必须在 negedge 驱动：在 posedge 用阻塞赋值会和 DUT 的采样
    //   落在同一个时刻步里竞争，可能采到"上一拍的按键值"（曾导致 key_next
    //   被当成 key_prev ⇒ 图片号 0→2 而不是 0→1）。
    @(negedge clk); key_next = 1; @(negedge clk); key_next = 0;
    // 现在会先经过 S_ARM(7) 预热几帧、等帧边界，才进 S_TRANS(3)
    fork : wait_trans
        begin
            wait (st_state == 3'd7);
            $display("  [ ok ] 进入 S_ARM(7) 预热");
            wait (st_state == 3'd3);
            disable wait_trans;
        end
        begin
            repeat (400000) @(posedge clk);
            $display("  [FAIL] 没进 TRANS（停在 %0d）", st_state);
            errors = errors + 1;
            disable wait_trans;
        end
    join
    if (st_state === 3'd3) $display("  [ ok ] 进入 TRANS");
    else if (errors == 0)  $display("  [FAIL] 没进 TRANS（=%0d）", st_state);

    fork : wait_back
        begin
            wait (st_state == 3'd5);
            disable wait_back;
        end
        begin
            repeat (2000000) @(posedge clk);
            $display("  [FAIL] 过渡后回不到 HD");
            errors = errors + 1;
            disable wait_back;
        end
    join
    $display("  [ ok ] 过渡完成回到 HD，img=%0d，TRANS 持续 %0d 拍", st_img, trans_frames);
    if (st_img !== 2'd1) begin $display("  [FAIL] 图片号没变成 1（=%0d）", st_img); errors=errors+1; end

    if (arm_hold_violation != 0) begin
        $display("  [FAIL] S_ARM 停留太久（>10 帧）⇒ 又在 S_ARM 里等装载了");
        errors = errors + 1;
    end else $display("  [ ok ] S_ARM 不再等装载，按下后几帧内就进动画");

    if (low_no_kick != 0) begin
        $display("  [FAIL] 有 %0d 次进了 S_LOW 却没踢装载（会干等 5.37s）", low_no_kick);
        errors = errors + 1;
    end else $display("  [ ok ] 动画结束后 S_LOW 立即踢了高清装载");

    //------------------------------------------------------------------
    //  ★★ 2026-10-25：第二次切换 —— 验"按下瞬间黑一帧"的根因已封住 ★★
    //  【要验什么】上一轮动画结束时 viewer_fx 的 A 几何是 **X0=−640**（整块在屏外）。
    //    若 S_ARM 不把 A 钉成满屏，按下后的第一帧就是"A 在屏外 + B 透明" = 纯黑。
    //    ⇒ 这里等真的跑完一轮（几何停在 −640）再按一次，检查 S_ARM 期间
    //      pos_a/size_a/k_a 是否被强制成满屏，同时确认 fx_pos_a 确实是 −640
    //      （否则这个用例是假的，什么都没测到）。
    //------------------------------------------------------------------
    // 先把 fx 从 2 转回 0（2 → 3 → 4 → 0，**三次**按键 —— fx 现在有 5 种），
    // **再跑一整轮 fx0 动画**，这样动画结束时 A 的几何才是 X0=−640
    // （上面那轮是 fx2，A 恒满屏，测不到）。
    repeat (3) begin
        @(negedge clk); key_fx = 1; fx_sel = (fx_sel == 3'd4) ? 3'd0 : fx_sel + 3'd1;
                    @(negedge clk); key_fx = 0;
        repeat (10) @(posedge clk);
    end
    $display("  [info] fx 已切回 %0d，开始第二轮（fx0）", st_fx);

    @(negedge clk); key_next = 1; @(negedge clk); key_next = 0;
    fork : wait_tr2
        begin wait (st_state == 3'd3); disable wait_tr2; end
        begin
            repeat (400000) @(posedge clk);
            $display("  [FAIL] 第二轮没进 TRANS");
            errors = errors + 1; disable wait_tr2;
        end
    join
    fork : wait_back2
        begin wait (st_state == 3'd5); disable wait_back2; end
        begin
            repeat (2000000) @(posedge clk);
            $display("  [FAIL] 第二轮回不到 HD");
            errors = errors + 1; disable wait_back2;
        end
    join
    $display("  [info] 第二轮跑完，img=%0d fx=%0d，fx_pos_a.X0=%0d（应为 −640）",
             st_img, st_fx, $signed(dut.fx_pos_a[31:16]));

    //---- 第三次按下：检查 S_ARM 立刻把 A 钉回满屏 ----
    // ★ 2026-10-31：改按「上一张」。此刻停在末张（img=2，I_win4=0），而滑动窗口
    //   改版后末张再按「下一张」是**有意不动**（置灰，见 viewer_ctrl 末张分支），
    //   原断言按旧的回绕行为写，已过时。上一张走同一条 S_ARM 路径，判据不变。
    @(negedge clk); key_prev = 1; @(negedge clk); key_prev = 0;
    fork : wait_arm2
        begin wait (st_state == 3'd7); disable wait_arm2; end
        begin
            repeat (200000) @(posedge clk);
            $display("  [FAIL] 第三次没进 S_ARM");
            errors = errors + 1; disable wait_arm2;
        end
    join
    repeat (4) @(posedge clk);          // 相位 0 走完（busy=0，两拍就过）

    if (dut.fx_pos_a[31:16] == 16'd0) begin
        $display("  [FAIL] fx_pos_a 的 X0 = 0，这一轮没测到「陈旧几何」（用例失效）");
        errors = errors + 1;
    end
    if (dut.pos_a !== 32'd0 || dut.size_a !== {16'd640, 16'd480}) begin
        $display("  [FAIL] S_ARM 里 A 的几何没钉成满屏（pos=%h size=%h）⇒ 按下第一帧会整屏黑",
                 dut.pos_a, dut.size_a);
        errors = errors + 1;
    end else $display("  [ ok ] S_ARM 里 A 被钉在满屏（旧几何 X0=%0d 被屏蔽，不会黑一帧）",
                      $signed(dut.fx_pos_a[31:16]));
    if (dut.k_a !== {20'd16384, 20'd32768}) begin
        $display("  [FAIL] S_ARM 里 k_a 没钉成满屏值（=%h）", dut.k_a);
        errors = errors + 1;
    end

    $display("\n==== errors = %0d ====", errors);
    if (errors == 0) $display("PASS"); else $display("FAIL");
    $finish;
end

initial begin
    #500_000_000; $display("TIMEOUT"); $finish;
end

endmodule
