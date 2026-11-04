//==============================================================================
//  tb_file_page_card4.vh —— tb_file_page 的第二套剧本（+define+FP_CARD4）
//==============================================================================
//  卡上内容 = sim_data/fat32_4img.img：IMG01..IMG04.BMP + MUSIC01.WAV + NOTE01.TXT
//  （+ 隐藏 HIDE01.BMP）⇒ 文件页 2 页：[4 张图] / [WAV, TXT]。
//  复现上板报告：
//    C1  进文件页按一次 KEY2 就要出 4 张图；出图后底图（相册缩略图）不能变黑
//    C2  翻到第 2 页：2 个名字（不是 4 个）；WAV/TXT 有占位色块；KEY4 能翻回第 1 页
//  每一步都抓整屏 PPM：out/c4_*.ppm
//==============================================================================
// ★ 2026-11-04 探针：本页名字重排（gen 翻转）要早于本页装完（ql_done）
// （C2b 已经在 WAV 上按过一次 KEY2 ⇒ C5 按增量判）
integer t_gen = 0, t_qdone = 0, n_trk = 0, n_jump = 0, last_jump = -1;
reg gen_d = 0;
always @(posedge clk) if (!rst) begin
    gen_d <= fp_gen_w;
    if (fp_gen_w !== gen_d) t_gen = $time;
    if (ql_done) t_qdone = $time;
    if (trk_set_r) begin n_trk = n_trk + 1; $display("  [trk] t=%0t sec=%0d", $time, trk_sec_r); end
    if (jump_r) begin n_jump = n_jump + 1; last_jump = jump_idx_r; $display("  [jump] t=%0t idx=%0d", $time, jump_idx_r); end
end
reg [3:0] c4st_d = 0;
always @(posedge clk) if (!rst && dbg_state !== c4st_d) begin
    $display("  [st] t=%0t %0d -> %0d cur=%0d frames=%0d", $time, c4st_d, dbg_state, u_app.u_ctrl.cur_abs, frames);
    c4st_d = dbg_state;
end
// 格子最左一列附近不许出现"压暗后的黑"（070707）—— 上板那条 1px 黑线
function integer black_col;
    input integer x0, y;
    integer x;
    begin
        black_col = 0;
        for (x = x0 - 1; x <= x0 + 5; x = x + 1) if (fb[y*640 + x] == 24'h070707) black_col = black_col + 1;
    end
endfunction
function [1:0] hd_dom4;
    input integer dummy;
    reg [31:0] w;
    begin w = sdram[HD_ADDR + 400*640 + 20]; hd_dom4 = dom(w[23:0]); end
endfunction

task dump_names;
    begin
        $display("    vcnt=%0d len4=%h ready=%b quad_mode=%b own=%b off=%0d",
                 quad_vcnt, lst_len4, quad_ready, quad_mode_r, quad_own, quad_off);
        for (nl = 0; nl < 4; nl = nl + 1)
            $display("    row%0d tx_len=%0d str=\"%0s\"", nl, lst_len4[nl*5 +: 5], nm_str[nl]);
    end
endtask

// 背景像素：面板外、格子外的一个点（左栏下方空白处的右侧）—— 相册底图应透过来
function [23:0] bg_px;
    input integer dummy;
    begin bg_px = fb[460*640 + 600]; end
endfunction

initial begin
    clr_names();
    rst = 1; wait_cyc(20); rst = 0;

    $display("--- boot (card4) ---");
    wait_state(5, 200_000_000);
    $display("  booted t=%0t frames=%0d state=%0d", $time, frames, dbg_state);
    wait_cyc(200000);

    $display("--- open OSD ---");
    press(0);
    wait_state(8, 20_000_000);
    wait_frames(6);
    wait_frame_done();
    dump_ppm("out/c4_00_osd.ppm");
    bgd = bg_px(0);
    $display("  bg pixel (album behind OSD) = %06h", bgd);

    $display("--- nav to File ---");
    key_and_settle(2);
    key_and_settle(2);
    chk("C nav=2", m_nav, 2);
    wait_frames(10);
    wait_frame_done();
    dump_ppm("out/c4_01_nav_file.ppm");

    $display("--- C1: KEY2 once -> page 1 ---");
    key_and_settle(1);
    chk("C1 focus PAGE", m_focus, 1);
    wait_quad_ready();
    chk("C1 quad_ready after ONE KEY2", quad_ready, 1);
    chk("C1 loaded 4", ql_loaded, 4);
    wait_frames(20);
    wait_frame_done();
    dump_ppm("out/c4_02_page1.ppm");
    $display("  cells: %06h %06h %06h %06h  bg=%06h", cell_px(0), cell_px(1), cell_px(2), cell_px(3), bg_px(0));
    chk("C1 cell0 red",    dom(cell_px(0)), 0);
    chk("C1 cell1 green",  dom(cell_px(1)), 1);
    chk("C1 cell2 blue",   dom(cell_px(2)), 2);
    chk("C1 cell3 yellow", dom(cell_px(3)), 3);
    // 底图 = 相册缩略图 + 菜单遮罩：四宫格打开前后同一点必须一样（原来变成近黑 070707）
    chk("C1 background unchanged by grid", bg_px(0), bgd);
    // ★ 每格最左一列不能是黑线（in_box 比像素早一拍的老问题）
    chk("C1 no black first column, left cell",  black_col(192, 150), 0);
    chk("C1 no black first column, right cell", black_col(384, 150), 0);
    chk("C1 no black first column, row 2",      black_col(384, 300), 0);
    dump_names();

    $display("--- C2: KEY3 x4 -> page 2 (WAV, TXT) ---");
    key_and_settle(2); key_and_settle(2); key_and_settle(2);
    chk("C2 item 3", m_item, 3);
    clr_names();
    key_and_settle(2);
    wait_quad_ready_or(400_000_000);
    wait_frames(30);
    wait_frame_done();
    dump_ppm("out/c4_03_page2.ppm");
    dump_names();
    // ★ 名字在**扫完**就重排，不等装完（上板：第 2 页先显示 4 行旧名字）
    chk("C2 names relaid before the page finished loading", (t_gen < t_qdone) ? 1 : 0, 1);
    chk("C2 offset 4", quad_off, 4);
    chk("C2 vcnt 2", quad_vcnt, 2);
    // 空槽 lister 给 1（= 一个空格，看不见），不能是上一页的名字长度
    chk("C2 row2 blank", (lst_len4[14:10] <= 5'd1) ? 1 : 0, 1);
    chk("C2 row3 blank", (lst_len4[19:15] <= 5'd1) ? 1 : 0, 1);
    chk("C2 bg unchanged", bg_px(0), bgd);
    chk("C2 quad_mode on (placeholders shown)", quad_mode_r, 1);
    $display("  cells: %06h %06h %06h %06h  bg=%06h", cell_px(0), cell_px(1), cell_px(2), cell_px(3), bg_px(0));
    chk("C2 cell0 WAV placeholder not black", (cell_px(0) == 24'd0) ? 0 : 1, 1);
    chk("C2 cell1 TXT placeholder not black", (cell_px(1) == 24'd0) ? 0 : 1, 1);
    // 空格（第 3/4 格）不能还是第 1 页的 IMG03(蓝)/IMG04(黄)
    chk("C2 cell2 empty (not stale IMG03 blue)", (dom(cell_px(2)) == 2'd2) ? 0 : 1, 1);
    chk("C2 cell3 empty (not stale IMG04)", (cell_px(3) == 24'h633719 || dom(cell_px(3)) == 2'd3) ? 0 : 1, 1);

    $display("--- C2b: KEY2 on page 2 ---");
    key_and_settle(1);
    wait_frames(10);

    $display("--- C2c: KEY4 back to page 1 ---");
    // 翻页后光标在第 0 格；KEY2 不改光标 ⇒ 这里第一次 KEY4 就翻回上一页
    chk("C2c on item 0", m_item, 0);
    clr_names();
    key_and_settle(3);
    wait_quad_ready_or(400_000_000);
    wait_frames(30);
    wait_frame_done();
    dump_ppm("out/c4_04_back_page1.ppm");
    dump_names();
    chk("C2c back to offset 0", quad_off, 0);
    chk("C2c item = last cell of page 1", m_item, 3);
    chk("C2c vcnt 4", quad_vcnt, 4);
    show_names();
    chk_names(1);

    //--------------------------------------------------------------------------
    $display("--- C4: KEY2 on IMG03 opens it (menu closes, no animation) ---");
    key_and_settle(3);                         // item 3 -> 2
    chk("C4 on item 2", m_item, 2);
    press(1);                                  // KEY2 = open（菜单会关 ⇒ 不能等 OSD 布局计数）
    wait_state(5, 400_000_000);
    chk("C4 jumped to directory image index 2", last_jump, 2);
    chk("C4 back on the home page (S_HD)", dbg_state, 5);
    chk("C4 current image = IMG03", u_app.u_ctrl.cur_abs, 2);
    chk("C4 HD shows IMG03 (blue)", hd_dom4(0), 2);
    dump_ppm("out/c4_05_open_img03.ppm");      // 静止 HD 不再出帧：fb 里就是最后那帧 888

    //--------------------------------------------------------------------------
    $display("--- C5: reopen menu, flip to page 2, KEY2 on MUSIC01.WAV switches track ---");
    press(0);
    wait_state(8, 40_000_000);
    wait_quad_ready_or(400_000_000);
    wait_frames(10);
    chk("C5 menu reopens on the file page", m_focus, 1);
    key_and_settle(2);                         // item 2 -> 3
    key_and_settle(2);                         // item 3 -> flip to page 2
    wait_quad_ready_or(400_000_000);
    wait_frames(10);
    chk("C5 page 2", quad_off, 4);
    chk("C5 item 0", m_item, 0);
    d0 = n_trk; d1 = n_jump;
    key_and_settle(1);
    chk("C5 KEY2 on WAV -> track switch", n_trk - d0, 1);
    chk("C5 menu still open", dbg_state, 8);
    key_and_settle(2);                         // item 1 = TXT
    key_and_settle(1);
    chk("C5 KEY2 on TXT does nothing", (n_trk - d0) + (n_jump - d1), 1);

    //--------------------------------------------------------------------------
    $display("--- C6: flip back (page 1 starts loading), flip forward again mid-load ---");
    key_and_settle(3);                         // item 1 -> 0
    key_and_settle(3);                         // item 0 -> flip to page 1
    chk("C6 lands on cell 4", m_item, 3);
    // 四宫格一出现（第 0 遍刚写完），第 4 格的 BMP 还没轮到装 ⇒ 槽里必须是纯黑
    while (!(quad_ready && ql_busy)) @(posedge clk);
    $display("  grid shown: loaded=%0d slot3 word=%08h", ql_loaded, sdram[QUAD_BASE + 3*40960 + 60*160 + 40]);
    chk("C6 not-yet-loaded cell 4 is black in SDRAM", sdram[QUAD_BASE + 3*40960 + 60*160 + 40], 0);
    wait_frame_done();
    dump_ppm("out/c4_06_loading.ppm");
    $display("  loading: cells %06h %06h %06h %06h loaded=%0d", cell_px(0), cell_px(1), cell_px(2), cell_px(3), ql_loaded);
    s = $time;
    key_and_settle(2);                         // item 3 -> flip next while loading
    wait_quad_ready_or(400_000_000);
    $display("  flip during load took %0d us", ($time - s) / 1000);
    chk("C6 flipped to page 2 without waiting", quad_off, 4);
    chk("C6 page 1 load was cut short", (ql_loaded < 4) ? 1 : 0, 1);
    key_and_settle(3);                         // flip back to page 1 for the close test
    wait_quad_ready_or(400_000_000);

    $display("--- C3: KEY1 -> nav, KEY1 -> close ---");
    key_and_settle(0);
    chk("C3 KEY1 back to NAV", m_focus, 0);
    press(0);
    wait_state(5, 200_000_000);
    chk("C3 back to S_HD", dbg_state, 5);
    chk("C3 loader returned", quad_own, 0);

    $display("");
    if (errors == 0) $display("ALL PASS");
    else             $display("FAIL: %0d errors", errors);
    $finish;
end

task wait_quad_ready_or;     // 等本页扫描+装载结束（不要求 ready=1：全是非图片时 ready 可能为 0）
    input integer tmo;
    integer g;
    begin
        g = 0;
        // 先等离开 S_SHOW（翻页开始），再等回到 S_SHOW
        while (u_qpage.st == 2'd3 && g < 2_000_000) begin @(posedge clk); g = g + 1; end
        while (!(u_qpage.st == 2'd3 && !ql_busy && !lst_busy) && g < tmo) begin @(posedge clk); g = g + 1; end
        if (g >= tmo) $display("  [warn] page load did not finish (qp st=%0d)", u_qpage.st);
    end
endtask

initial begin
    #(64'd30_000_000_000);
    $display("FAIL: card4 timeout");
    $finish;
end
