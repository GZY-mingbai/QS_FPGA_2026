//==============================================================================
//  tb_osd_entry.v —— viewer_ctrl 的 OSD / 相册 / 自动切换 专项验证
//==============================================================================
//
//  【为什么单独做一个】原来 tb_ctrl.v 完全没有覆盖这条路径：
//    · 它没有任何 key_osd_enter 用例；
//    · 它的 st_state 只有 3 位，连 S_OSD=8 都表示不了。
//    所以 2026-10-27 的"OSD 打不开 / 点复选框就关菜单"两个 bug
//    在旧测试台上**一次都没跑过**。
//
//  【覆盖的用例】
//    E0    上电进 S_HD，osd_allow=1、osd_on=0
//    E1/E2 按 KEY1 → 进 S_OSD(8)，osd_on=1 / fmt565=1 / a_hd=0
//    E3/E4 40 万拍内不被踢回 S_BOOT_T / S_LOW，之后仍在 S_OSD
//    E5    key_osd_exit 能正常退出到 S_HDF/S_HD
//    F     按 KEY3 切下一张的完整状态时间线
//    H     fx_sel 必须传给 fx_id（原来 fx_id 从不更新，永远是 fx0）
//    I     开信息条 + 自动切换 3s：信息条每秒刷新不能让倒计时归零
//
//  【2026-10-27 重建说明】
//    一次批量替换脚本误用了 `newline='\\n'`（非法值），Python 的 open()
//    在抛 ValueError 之前已经创建并截断了文件，本文件因此被清零。
//    这里按此前读到的内容重建，功能与原版一致，并合入 G/H/I 三个新用例。
//    ⇒ 教训：批量改写前先备份；newline 只允许 None / '' / '\n' / '\r' / '\r\n'。
//
//  【怎么跑】
//    cd src/user_source/hdl_source
//    iverilog -g2005 -o tb_osd_entry.vvp APP/tb_osd_entry.v APP/viewer_ctrl.v \
//             APP/viewer_fx.v APP/viewer_scene.v APP/app_scan_gen.v \
//             APP/img_layer_ctrl.v APP/img_layer_hd.v APP/ps_merge_alpha_pp.v \
//             APP/ps_to_rgb24.v APP/ps_merge_alpha.v APP/img_afifo.v \
//             APP/ps_gen_image.v APP/ps_merge_overlay.v
//    vvp tb_osd_entry.vvp
//
//==============================================================================
`timescale 1ns / 1ps

module tb_osd_entry;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

// ---- 按键 ----
reg key_prev = 0, key_next = 0, key_fx = 0;
reg key_osd_enter = 0, key_osd_exit = 0;

// ---- 帧写控制器 ----
reg busy = 0, ps_start = 0;
reg pipe_ce = 1;

// ---- 装载器 ----
wire        ld_req, ld_fmt;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
reg         ld_can_load = 1, ld_done = 0, ld_err = 0, ld_sd_ready = 0;

// ---- 场景 ----
wire        compose_en, fmt565, a_hd, a_en;
wire [20:0] a_base, b_base;
wire        b_en;
wire [31:0] pos_a, size_a, pos_b, size_b;
wire [39:0] k_a, k_b;
wire [7:0]  alpha_b;
wire [1:0]  st_img;
wire [2:0]  st_fx;
wire [3:0]  st_state;
wire        osd_on, osd_allow, info_show;
wire [15:0] res_w, res_h;
wire [5:0]  auto_remain;
wire [8:0]  auto_bar;

// ---- H/I 用例需要的可控输入（原来都硬接常量）----
reg [2:0] fx_sel    = 3'd0;      // 菜单选择的切换效果
reg       info_on   = 1'b0;      // 信息条开关
reg       osd_trig  = 1'b0;      // 文字已更新（信息条重合成）
reg [2:0] auto_mode = 3'd0;      // 自动切换档位

viewer_ctrl #(
    .THUMB0_ADDR(21'd614400), .THUMB1_ADDR(21'd652800),
    .THUMB2_ADDR(21'd691200), .HD_ADDR(21'd729600),
    // ★ 2026-10-27：把时间常数压到仿真量级。硬件全部用缺省值
    //   （FRAME_CYC=835000 / T_END_F=60 / T_END=300 / AUTO_SEC_CYC=50_000_000），
    //   否则过渡 1s = 83.5 万拍、自动切换 3s = 1.5 亿拍，根本跑不完。
    .FRAME_CYC    (1000),
    .T_END_F      (10),
    .T_END        (20),
    .AUTO_SEC_CYC (20000)
) dut (
    .clk(clk), .rst(rst),
    // ★ 2026-11-01：IMG0/1/2_OFFSET 已从参数改成端口（相册走 FAT，
    //   这几个台只关心几何/过渡，偏移给固定值即可）
    .I_img_off0 (32'd0), .I_img_off1 (32'd0), .I_img_off2 (32'd0),
    .I_img_off3(32'd0),
    .I_have(3'd3), .I_alb_ready(1'b1), .I_jump(1'b0), .I_jump_idx(8'd0), .O_ld_abort(),
    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx),
    .key_osd_enter(key_osd_enter), .key_osd_exit(key_osd_exit),
    .osd_fade(8'd255),
    .osd_on(osd_on), .osd_allow(osd_allow),
    .busy(busy), .ps_start(ps_start), .pipe_ce(pipe_ce),
    .ld_req(ld_req), .ld_fmt(ld_fmt), .ld_off(ld_off), .ld_len(ld_len), .ld_base(ld_base),
    .ld_can_load(ld_can_load), .ld_done(ld_done), .ld_err(ld_err),
    .ld_sd_ready(ld_sd_ready),
    .compose_en(compose_en), .fps_dbg(),
    .fmt565(fmt565), .a_hd(a_hd), .a_en(a_en), .a_base(a_base),
    .b_en(b_en), .b_base(b_base),
    .pos_a(pos_a), .size_a(size_a), .k_a(k_a),
    .pos_b(pos_b), .size_b(size_b), .k_b(k_b), .alpha_b(alpha_b),
    .st_img(st_img), .st_fx(st_fx), .st_state(st_state),
    .fx_sel(fx_sel), .auto_mode(auto_mode),.info_on(info_on),
    .album_prev(1'b0), .album_next(1'b0), .osd_trig(osd_trig), .music_done(1'b0),
    .ld_src_w(16'd640), .ld_src_h(16'd480),
    .res_w(res_w), .res_h(res_h),
    .auto_remain(auto_remain), .auto_bar(auto_bar),
    .info_show(info_show), // 2026-10-31: tie off ports added after this bench was written (were floating Z)
    .I_win4(1'b0), .za_x(10'd320), .za_y(10'd240), .zoom_on(1'b0), .preview_req(1'b0), .load_req(1'b0), .zoom_val(8'h10)
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
//  假帧源：compose_en=1 时周期性给 1000 拍的 ps_start
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
            // ★ 停帧时必须把 fcnt 一起清掉 ★
            //   否则 compose_en 拉低时若 fcnt 停在正值，恢复出帧后
            //   `!busy && fcnt==0` 和 `fcnt<0` 都不成立 ⇒ 帧源永久冻结，
            //   S_HDF 会一直等 busy 上升，看着像 RTL 卡死（其实是测试台问题）。
            ps_start <= 0; busy <= 0; fcnt <= 0;
        end
    end
end

//------------------------------------------------------------------------------
//  检查
//------------------------------------------------------------------------------
integer errors = 0;
integer k;
reg     hit_boot_t = 0;
reg     hit_low    = 0;

task chk;
    input [8*40-1:0] name;
    input integer    got;
    input integer    exp;
begin
    if (got !== exp) begin
        $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
        errors = errors + 1;
    end else
        $display("   ok       %0s = %0d", name, got);
end
endtask

initial begin
    $display("=== 上电：装 3 张缩略图 + HD0，进 S_HD ===");
    repeat (5) @(negedge clk);
    rst = 1;
    repeat (3) @(negedge clk);
    rst = 0;
    repeat (3) @(negedge clk);
    ld_sd_ready = 1;

    // 等进 S_HD(=5)，最多等 200 万拍
    k = 0;
    while ((st_state !== 4'd5) && (k < 2_000_000)) begin
        @(posedge clk);
        k = k + 1;
    end
    if (st_state !== 4'd5) begin
        $display("  *** FAIL  上电没能进入 S_HD（停在 %0d）", st_state);
        errors = errors + 1;
    end else begin
        $display("   ok       进入 S_HD（%0d 拍）", k);
    end
    repeat (20) @(posedge clk);

    chk("E0 osd_allow", osd_allow, 1);
    chk("E0 osd_on",    osd_on,    0);

    //======================================================================
    //  E1-E4：按 KEY1 → 稳定停在 S_OSD
    //======================================================================
    $display("=== E1/E2/E3/E4：按 KEY1 → 应稳定停在 S_OSD ===");
    @(negedge clk); key_osd_enter = 1;
    @(negedge clk); key_osd_enter = 0;

    k = 0;
    while ((st_state !== 4'd8) && (k < 200_000)) begin
        @(posedge clk);
        k = k + 1;
    end
    chk("E1 enter S_OSD", st_state, 8);
    if (st_state !== 4'd8)
        $display("        （没进去；state=%0d osd_on=%0d fmt565=%0d a_hd=%0d）",
                 st_state, osd_on, fmt565, a_hd);

    repeat (100) @(posedge clk);
    chk("E2 osd_on",  osd_on,  1);
    chk("E2 fmt565",  fmt565,  1);
    chk("E2 a_hd",    a_hd,    0);

    // E3/E4：监控 40 万拍，暴露"被踢回 S_BOOT_T"
    hit_boot_t = 0;
    hit_low    = 0;
    for (k = 0; k < 400_000; k = k + 1) begin
        @(posedge clk);
        if (st_state == 4'd0) hit_boot_t = 1;
        if (st_state == 4'd2) hit_low    = 1;
    end
    chk("E3 not S_BOOT_T", hit_boot_t, 0);
    chk("E3 not S_LOW",    hit_low,    0);
    chk("E4 still S_OSD",  st_state,   8);
    chk("E4 osd_on still", osd_on,     1);

    //======================================================================
    //  E5：退出
    //======================================================================
    $display("=== E5：按 KEY1 退出 → 应回到 S_HDF(4) 或 S_HD(5) ===");
    @(negedge clk); key_osd_exit = 1;
    @(negedge clk); key_osd_exit = 0;
    k = 0;
    while ((st_state !== 4'd4) && (st_state !== 4'd5) && (k < 400_000)) begin
        @(posedge clk);
        k = k + 1;
    end
    if ((st_state === 4'd4) || (st_state === 4'd5))
        $display("   ok       E5 退出 OSD，state=%0d", st_state);
    else begin
        $display("  *** FAIL  E5 退不出去（停在 %0d）", st_state);
        errors = errors + 1;
    end

    //======================================================================
    //  F：切下一张（KEY3）—— 记录完整状态时间线
    //======================================================================
    $display("=== F：按 KEY3 切下一张，看状态时间线 ===");
    k = 0;
    while ((st_state !== 4'd5) && (k < 2_000_000)) begin @(posedge clk); k = k + 1; end
    repeat (50) @(posedge clk);
    chk("F0 start at S_HD", st_state, 5);
    $display("       起始: img=%0d a_hd=%0d fmt565=%0d a_base=%0d",
             st_img, a_hd, fmt565, a_base);

    @(negedge clk); key_next = 1;
    @(negedge clk); key_next = 0;

    begin : f_mon
        integer cyc;
        integer prev;
        cyc  = 0;
        prev = -1;
        // 进入本段时状态已经是 S_ARM(7)，所以「st_state == S_HD」就是收工条件；
        // 不要再用 prev==4 去判 —— prev 会被更新成 5，那个条件永远不成立。
        while ((cyc < 1_200_000) && (st_state !== 4'd5)) begin
            @(posedge clk);
            cyc = cyc + 1;
            if (st_state !== prev) begin
                $display("        cyc=%0d  state %0d -> %0d   a_hd=%0d fmt565=%0d b_en=%0d a_base=%0d",
                         cyc, prev, st_state, a_hd, fmt565, b_en, a_base);
                prev = st_state;
            end
            if ((cyc % 50000) == 0) begin
                $display("        [dbg] cyc=%0d st=%0d hdf_ph=%0d compose_en=%0d busy=%0d ps_start=%0d",
                         cyc, st_state, dut.hdf_ph, compose_en, busy, ps_start);
            end
        end
        $display("       回到 S_HD 共 %0d 拍（@50MHz ≈ %0d ms）", cyc, cyc/50000);
        // ★ 2026-10-27：过渡终点必须精确踩在 t_end_cur 上（T_END_F=10、fx0）。
        //   修复前这里是 9（tnext 一步跨过终点就直接切状态），
        //   终点几何差 1 小步 ⇒ 变清晰那一瞬间整幅横移 1~2 像素。
        chk("F1 t==T_END_F", dut.t, 10);
        // fx0 在 t=T_END 时 B 的 X0 必须精确是 0（否则变清晰瞬间会横移）
        chk("F2 pos_b X0==0", dut.pos_b[31:16], 0);
        chk("F3 pos_a X0==-640", dut.pos_a[31:16], 16'hFD80);
        if (cyc >= 1_200_000) begin
            $display("  *** FAIL  F 切下一张之后没回到 S_HD（卡在 state=%0d）", st_state);
            errors = errors + 1;
        end else
            $display("   ok       F 切下一张后回到 S_HD");
    end

    //======================================================================
    //  H：fx_sel => fx_id（原来 fx_id 从来没人写过，永远是 fx0）
    //======================================================================
    $display("=== H：fx_sel -> fx_id ===");
    repeat (50) @(posedge clk);
    chk("H0 in S_HD", st_state, 5);

    @(negedge clk); fx_sel = 3'd3;
    repeat (10) @(posedge clk);
    chk("H1 fx_sel=3", st_fx, 3);

    @(negedge clk); fx_sel = 3'd1;
    repeat (10) @(posedge clk);
    chk("H2 fx_sel=1", st_fx, 1);

    @(negedge clk); fx_sel = 3'd4;
    repeat (10) @(posedge clk);
    chk("H3 fx_sel=4", st_fx, 4);

    @(negedge clk); fx_sel = 3'd0;
    repeat (10) @(posedge clk);
    chk("H4 fx_sel=0", st_fx, 0);

    //======================================================================
    //  I：开信息条 + 自动切换 3s
    //     信息条每秒刷新会让 state 进一次 S_HDF；修好之前 auto_cnt 每次
    //     被清零 ⇒ 倒计时永远 3->2->3，自动切换永不触发。
    //     AUTO_SEC_CYC=20000 ⇒ 1s=2 万拍、3s=6 万拍；
    //     osd_trig 每 15000 拍给一次（比 1 秒更密，确保每轮都打断）。
    //======================================================================
    $display("=== I：info bar + auto 3s ===");
    @(negedge clk); auto_mode = 3'd1;
    @(negedge clk); info_on   = 1'b1;
    repeat (50) @(posedge clk);

    begin : i_mon
        integer cyc;
        integer arm_seen;
        integer rem_min;
        integer hdf_seen;
        cyc = 0; arm_seen = 0; rem_min = 99; hdf_seen = 0;
        while (cyc < 300_000) begin
            @(posedge clk);
            cyc = cyc + 1;
            osd_trig = ((cyc % 15000) == 0) ? 1'b1 : 1'b0;
            if (st_state == 4'd7) arm_seen = 1;      // S_ARM = 自动切换真的触发
            if (st_state == 4'd4) hdf_seen = 1;      // 信息条刷新确实用过 S_HDF
            if (auto_remain < rem_min) rem_min = auto_remain;
        end
        chk("I1 HDF seen", hdf_seen, 1);
        chk("I2 ARM seen", arm_seen, 1);
        chk("I3 rem to 0", rem_min,  0);
    end

    $display("======================================");
    if (errors == 0) $display("  ALL PASS");
    else             $display("  %0d FAIL", errors);
    $display("======================================");
    $finish;
end

endmodule
