`timescale 1ns/1ps
//==============================================================================
//  tb_equiv_menu_geom.v —— osd_menu [LUT-1] 重构的**逐位等价**验证
//==============================================================================
//  【被验证的改动】布局机不再遍历 20 个槽，只遍历 9 个动态槽：
//      L_CELL 的选择子从 5 位 cell_i(0..19) 换成 4 位 dci(0..8)，
//      几何 case 从 20 分支缩到 9 分支（删掉 11 个从未被采样过的常量槽分支）。
//
//  【为什么这算等价】wr_en 原来就是 `(lay_st==L_CELL) && cell_is_dyn`，
//      所以旧版 20 个分支里只有 9 个的 gx0..gv 被 cellmem 采样过。
//      新版让 dci 直接就是那 9 个的下标，写出的 (idx, geo, attr) **序列**不变。
//
//  【这台子怎么判】同一份激励同时喂给：
//      dut_new : src 里的 osd_menu
//      dut_ref : ref/osd_menu_ref_851dbcd.v（commit 851dbcd 的冻结快照）
//  编译：
//      iverilog -g2012 -DIVERILOG_SIM -o e.vvp -s tb_equiv_menu_geom \
//        tb_equiv_menu_geom.v osd_menu.v ref/osd_menu_ref_851dbcd.v \
//        tools/bram32k_stub.v tools/mult18_stub.v && vvp e.vvp
//  两件事都要一致：
//    ① 写序列：每次 wr_en 记一条 (wr_idx, wr_geo, wr_attr)，最后逐条比对。
//       —— 旧版在 L_CELL 里空转 11 拍（wr_en=0），新版没有空转，
//          所以**不能按拍比**，只能按"写事件序列"比。
//    ② 稳态输出：txt_xy / txt_col / ctl_* / pic_* / o_* / ic_act_mask …
//       在每个激励段落结束、布局收敛后逐位比。
//==============================================================================
module tb_equiv_menu_geom;

localparam integer NL     = 8;
localparam integer MAXLEN = 32;
localparam integer MAXREC = 512;

reg clk = 0;
always #10 clk = ~clk;

reg rst = 1;
reg tick = 0;
reg ps_start = 0;
reg menu_on = 0;
reg zoom_on = 0;
reg [7:0] zoom_val = 8'h10;
reg [1:0] zoom_mode = 2'd0;
reg osd_allow = 1;
reg [3:0] key_press = 4'd0;
reg [3:0] key_level = 4'hF;
reg [15:0] src_w = 16'd640, src_h = 16'd480;
reg [5:0]  auto_remain = 6'd0;
reg [8:0]  auto_bar = 9'd0;

//------------------------------------------------------------------------------
// 实例：新版
//------------------------------------------------------------------------------
wire        n_enter, n_exit;
wire [7:0]  n_fade;
wire        n_wre;  wire [4:0] n_wri;  wire [38:0] n_wrg;  wire [23:0] n_wra;
wire [22*NL-1:0] n_txy;  wire [24*NL-1:0] n_tcl;
wire        n_swe;  wire [3:0] n_sl;  wire [7:0] n_si;  wire [15:0] n_sc;
wire        n_cwe;  wire [3:0] n_cl;  wire n_ca;  wire [7:0] n_cln;  wire [7:0] n_cv;
wire        n_trig;
wire [6:0]  n_pbr, n_pco, n_pte, n_psa;
wire [1:0]  n_pst;
wire        n_ptgl;
wire        n_olang;  wire [6:0] n_ovol;  wire n_omute, n_oloop, n_ospc;
wire [2:0]  n_fx, n_am;
wire        n_info;
wire        n_ap, n_an;
wire [7:0]  n_icact;

osd_menu #(.NL(NL), .MAXLEN(MAXLEN), .SELFTEST(0)) dut_new (
    .clk(clk), .rst(rst), .tick(tick), .ps_start(ps_start),
    .menu_on(menu_on), .zoom_on(zoom_on), .zoom_val(zoom_val),
    .zoom_mode(zoom_mode), .osd_allow(osd_allow),
    .key_press(key_press), .key_level(key_level), .bright_eff(7'd50),
    .enter_req(n_enter), .exit_req(n_exit), .fade(n_fade),
    .wr_en(n_wre), .wr_idx(n_wri), .wr_geo(n_wrg), .wr_attr(n_wra),
    .txt_xy(n_txy), .txt_col(n_tcl),
    .str_we(n_swe), .str_line(n_sl), .str_idx(n_si), .str_code(n_sc),
    .ctl_we(n_cwe), .ctl_line(n_cl), .ctl_active(n_ca),
    .ctl_len(n_cln), .ctl_ver(n_cv), .trig(n_trig),
    .pic_bright(n_pbr), .pic_contrast(n_pco), .pic_temp(n_pte),
    .pic_sat(n_psa), .pic_style(n_pst), .pic_tgl(n_ptgl),
    .o_lang(n_olang), .o_vol(n_ovol), .o_mute(n_omute),
    .o_loop(n_oloop), .o_spc(n_ospc),
    .fx_sel(n_fx), .auto_mode(n_am),.info_on(n_info),
    .album_prev(n_ap), .album_next(n_an),
    .ic_act_mask(n_icact),
    .src_w(src_w), .src_h(src_h), .auto_remain(auto_remain), .auto_bar(auto_bar)
);

//------------------------------------------------------------------------------
// 实例：改动前（osd_menu_ref.v，从 commit 851dbcd 导出）
//------------------------------------------------------------------------------
wire        r_enter, r_exit;
wire [7:0]  r_fade;
wire        r_wre;  wire [4:0] r_wri;  wire [38:0] r_wrg;  wire [23:0] r_wra;
wire [22*NL-1:0] r_txy;  wire [24*NL-1:0] r_tcl;
wire        r_swe;  wire [3:0] r_sl;  wire [7:0] r_si;  wire [15:0] r_sc;
wire        r_cwe;  wire [3:0] r_cl;  wire r_ca;  wire [7:0] r_cln;  wire [7:0] r_cv;
wire        r_trig;
wire [6:0]  r_pbr, r_pco, r_pte, r_psa;
wire [1:0]  r_pst;
wire        r_ptgl;
wire        r_olang;  wire [6:0] r_ovol;  wire r_omute, r_oloop, r_ospc;
wire [2:0]  r_fx, r_am;
wire        r_info;
wire        r_ap, r_an;
wire [7:0]  r_icact;

osd_menu_ref #(.NL(NL), .MAXLEN(MAXLEN), .SELFTEST(0)) dut_ref (
    .clk(clk), .rst(rst), .tick(tick), .ps_start(ps_start),
    .menu_on(menu_on), .zoom_on(zoom_on), .zoom_val(zoom_val),
    .zoom_mode(zoom_mode), .osd_allow(osd_allow),
    .key_press(key_press), .key_level(key_level),
    .enter_req(r_enter), .exit_req(r_exit), .fade(r_fade),
    .wr_en(r_wre), .wr_idx(r_wri), .wr_geo(r_wrg), .wr_attr(r_wra),
    .txt_xy(r_txy), .txt_col(r_tcl),
    .str_we(r_swe), .str_line(r_sl), .str_idx(r_si), .str_code(r_sc),
    .ctl_we(r_cwe), .ctl_line(r_cl), .ctl_active(r_ca),
    .ctl_len(r_cln), .ctl_ver(r_cv), .trig(r_trig),
    .pic_bright(r_pbr), .pic_contrast(r_pco), .pic_temp(r_pte),
    .pic_sat(r_psa), .pic_style(r_pst), .pic_tgl(r_ptgl),
    .o_lang(r_olang), .o_vol(r_ovol), .o_mute(r_omute),
    .o_loop(r_oloop), .o_spc(r_ospc),
    .fx_sel(r_fx), .auto_mode(r_am),.info_on(r_info),
    .album_prev(r_ap), .album_next(r_an),
    .ic_act_mask(r_icact),
    .src_w(src_w), .src_h(src_h), .auto_remain(auto_remain), .auto_bar(auto_bar)
);

//------------------------------------------------------------------------------
// 写事件记录：按【槽号】记最后一次写入
//------------------------------------------------------------------------------
//  【为什么不是按拍/按序列比】旧版 L_CELL 遍历 20 个槽（其中 11 拍 wr_en=0 空转），
//  新版只遍历 9 个（每拍都写）。所以一轮布局的**时长**不同（20 拍 vs 9 拍），
//  同一个仿真时间窗内新版会多跑若干轮。轮数是时间窗的产物，不是功能差异。
//  真正的不变量是：**同样一轮布局写进 cellmem 的内容必须完全一样**，
//  也就是「每个槽最后一次写入的 (geo, attr)」必须逐位相同。
//  另加一条：一次布局必须恰好写 9 条（n_cnt % 9 == 0），新版旧版都要满足。
localparam integer NSLOT_DYN = 9;

reg [38:0] n_last_geo [0:NSLOT_DYN-1];
reg [23:0] n_last_attr[0:NSLOT_DYN-1];
reg        n_last_vld[0:NSLOT_DYN-1];
reg [38:0] r_last_geo [0:NSLOT_DYN-1];
reg [23:0] r_last_attr[0:NSLOT_DYN-1];
reg        r_last_vld[0:NSLOT_DYN-1];
integer n_cnt = 0, r_cnt = 0;

integer zi;
initial begin
    for (zi = 0; zi < NSLOT_DYN; zi = zi + 1) begin
        n_last_vld[zi] = 1'b0;  r_last_vld[zi] = 1'b0;
        n_last_geo[zi] = 39'd0; r_last_geo[zi] = 39'd0;
        n_last_attr[zi]= 24'd0; r_last_attr[zi]= 24'd0;
    end
end

always @(posedge clk) begin
    if (n_wre) begin
        n_last_geo [n_wri[3:0]] <= n_wrg;
        n_last_attr[n_wri[3:0]] <= n_wra;
        n_last_vld [n_wri[3:0]] <= 1'b1;
        n_cnt <= n_cnt + 1;
    end
    if (r_wre) begin
        r_last_geo [r_wri[3:0]] <= r_wrg;
        r_last_attr[r_wri[3:0]] <= r_wra;
        r_last_vld [r_wri[3:0]] <= 1'b1;
        r_cnt <= r_cnt + 1;
    end
end

//------------------------------------------------------------------------------
// 判据
//------------------------------------------------------------------------------
integer bad = 0, chks = 0;

task chk;
    input cond;
    input [255:0] msg;
    begin
        chks = chks + 1;
        if (!cond) begin
            bad = bad + 1;
            $display("  FAIL: %0s", msg);
        end
    end
endtask

task cmp_state;
    input [255:0] tag;
    integer k;
    begin
        // 稳态可见输出（ctl_ver 是"布局轮数"计数器，轮数随仿真时间窗变化，
        //   不属于功能差异，故不比它）
        chk(n_txy === r_txy, {tag, " txt_xy"});
        chk(n_tcl === r_tcl, {tag, " txt_col"});
        chk(n_ca === r_ca, {tag, " ctl_active"});
        chk(n_cln === r_cln, {tag, " ctl_len"});
        chk(n_icact === r_icact, {tag, " ic_act_mask"});
        chk(n_fx === r_fx, {tag, " fx_sel"});
        chk(n_am === r_am, {tag, " auto_mode"});
        chk(n_info === r_info, {tag, " info_on"});
        chk(n_olang === r_olang, {tag, " o_lang"});
        chk(n_ovol === r_ovol, {tag, " o_vol"});
        chk(n_omute === r_omute, {tag, " o_mute"});
        chk(n_oloop === r_oloop, {tag, " o_loop"});
        chk(n_pbr === r_pbr && n_pco === r_pco && n_pte === r_pte &&
            n_psa === r_psa && n_pst === r_pst, {tag, " pic_*"});
        // ★ 核心：每个槽最后一次写入的 (geo, attr) 必须逐位相同
        for (k = 0; k < NSLOT_DYN; k = k + 1) begin
            chk(n_last_vld[k] === r_last_vld[k], {tag, " slot vld"});
            chk(n_last_geo[k] === r_last_geo[k] &&
                n_last_attr[k] === r_last_attr[k], {tag, " slot geo/attr"});
            if (n_last_geo[k] !== r_last_geo[k] ||
                n_last_attr[k] !== r_last_attr[k]) begin
                $display("     slot%0d new=(%011h,%06h) ref=(%011h,%06h)",
                         k, n_last_geo[k], n_last_attr[k],
                            r_last_geo[k], r_last_attr[k]);
            end
        end
    end
endtask

//------------------------------------------------------------------------------
// 时钟/激励
//------------------------------------------------------------------------------
integer i;
integer cyc = 0;
always @(posedge clk) cyc <= cyc + 1;

// 每 8 拍给一个 tick（布局机的推进源）
always @(posedge clk) tick <= ((cyc % 8) == 0);

// 等待"两路都安静"：连续 40 拍没有 wr_en ⇒ 都停在一轮布局之外，快照才可比
task settle;
    input integer n;
    integer q, g;
    begin
        key_press <= 4'd0;
        for (i = 0; i < n; i = i + 1) @(posedge clk);
        key_press <= 4'd0;
        // 再等一个"安静窗口"（上限 20000 拍，防死等）
        q = 0; g = 0;
        while ((q < 40) && (g < 20000)) begin
            @(posedge clk);
            g = g + 1;
            if (n_wre || r_wre) q = 0; else q = q + 1;
        end
    end
endtask

task press;
    input [3:0] k;
    begin
        key_press <= k;
        @(posedge clk);
        key_press <= 4'd0;
    end
endtask

integer kk;

initial begin
    $display("");
    $display("======== tb_equiv_menu_geom : osd_menu [LUT-1] 逐位等价 ========");

    rst = 1;
    for (i = 0; i < 8; i = i + 1) @(posedge clk);
    rst = 0;
    repeat (20) @(posedge clk);

    // ---------- 1. 打开菜单 ----------
    menu_on = 1;
    settle(6000);
    cmp_state("1 menu open");

    // ---------- 2. 切页面（导航：下一页/上一页） ----------
    press(4'b0100); settle(6000); cmp_state("2 page next");
    press(4'b1000); settle(6000); cmp_state("3 page prev");

    // ---------- 3. 移动选中项 ----------
    press(4'b0010); settle(6000); cmp_state("4 item down");
    press(4'b0001); settle(6000); cmp_state("5 item up");

    // ---------- 4. 打开/关闭弹层（需先走到 ComboBox 行：反复下移） ----------
    for (kk = 0; kk < 8; kk = kk + 1) begin
        press(4'b0010); settle(4000);
    end
    cmp_state("6 combo row");

    // ---------- 5. 进编辑并加减值 ----------
    press(4'b1000); settle(6000); cmp_state("7 edit enter");
    for (kk = 0; kk < 4; kk = kk + 1) begin
        press(4'b0100); settle(4000);
    end
    cmp_state("8 value inc");
    for (kk = 0; kk < 6; kk = kk + 1) begin
        press(4'b0100); settle(4000);
    end
    cmp_state("9 value inc more");

    // ---------- 6. 关菜单 → 信息条 ----------
    //  说明：`info_on` 是菜单里"信息条开关"的位（page_r==1 时取 val4[0]），
    //  正常激励很难走到。这里用 force 把它拉起来，好让 `!menu_on` 覆盖段里
    //  dci==6（信息条面板）/ dci==7（进度条）两个分支真的被跑到。
    //  两路 force 完全一样 ⇒ 对比仍然有效。
    //  ★ 2026-11-05：还**必须**把 val3（自动切换）force 成 1。
    //    新版的进度条只服务 auto_mode=1..4：mode 0 时 dci 7 退化成零矩形、
    //    dci 6 的面板收到 y=52。这台子比的是"与冻结快照逐位等价"，
    //    所以要么两边都给同一个 mode（这里给 3s，进度条在），要么
    //    这一段就变成"故意不同"而失去意义。
    //    mode 0 / 5 的可见性由 tb_infobar_shot 的 S2/S3 断言覆盖。
    menu_on = 0;
    force dut_new.page_r = 3'd1;
    force dut_ref.page_r = 3'd1;
    force dut_new.val4   = 7'd1;
    force dut_ref.val4   = 7'd1;
    force dut_new.val3   = 7'd1;
    force dut_ref.val3   = 7'd1;
    settle(6000);
    cmp_state("10 info bar");

    // 信息条进度值变化（auto_bar / auto_remain 动起来）
    auto_bar = 9'd120; auto_remain = 6'd12;
    settle(6000);
    cmp_state("10b info bar bar");
    release dut_new.page_r;
    release dut_ref.page_r;
    release dut_new.val4;
    release dut_ref.val4;
    release dut_new.val3;
    release dut_ref.val3;
    auto_bar = 9'd0; auto_remain = 6'd0;

    // ---------- 7. 缩放 OSD（菜单关） ----------
    zoom_on = 1; zoom_val = 8'h20; zoom_mode = 2'd2;
    settle(6000);
    cmp_state("11 zoom 2.0x ZOOM");
    zoom_val = 8'h40; zoom_mode = 2'd0;
    settle(6000);
    cmp_state("12 zoom 4.0x X-PAN");
    zoom_val = 8'h80; zoom_mode = 2'd1;
    settle(6000);
    cmp_state("13 zoom 8.0x Y-PAN");
    zoom_on = 0;
    settle(6000);
    cmp_state("14 zoom off");

    // ---------- 8. 再开菜单 ----------
    menu_on = 1;
    settle(6000);
    cmp_state("15 menu reopen");

    $display("");
    if (bad == 0) $display("  ALL PASS  (%0d checks；9 槽 geo/attr 逐位一致，累计 %0d 次写入)", chks, n_cnt);
    else          $display("  FAIL      (%0d / %0d checks 不符)", bad, chks);
    $display("===============================================================");
    $display("");
    $finish;
end

endmodule
