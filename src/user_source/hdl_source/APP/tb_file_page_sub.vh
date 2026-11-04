//==============================================================================
//  tb_file_page_sub.vh —— 主页换图：动态字幕 + 音随图切（+define+FP_SUB）
//==============================================================================
//  卡 = sim_data/fat32_sub.img（tools/mk_fat32_sub.py）：
//      IMG01 红 + IMG01.TXT(短) + IMG01.WAV
//      IMG02 绿 + IMG02.TXT(>64 字，懒加载) + IMG02.WAV
//      IMG03 蓝、IMG04 黄：没有 TXT / WAV
//  剧本：开机(1) → KEY3(2) → KEY4(1) → KEY3(2) → KEY3(3)
//  每一步断言：
//    · 画面真的换了：cur_abs、当前缩略图槽、高清都 = 目标图
//    · 字幕：这一步恰好提交 1 次；长度 / 码表首字 / 字形槽 0 与该图的 TXT 一致；
//            没 TXT 的图提交空字幕（len=0）
//    · 音乐：trk_sec = 该图的 WAV（没有 WAV 就保持上一首）
//    · 不打断装载：字幕提交晚于该图高清 ld_done（报告 + 断言）
//==============================================================================
`include "../../../../sim_data/fat32_sub_sub.vh"

integer n_commit = 0, n_start = 0;
reg     cm_d = 1'b0;
time    t_commit = 0, t_hd = 0, t_press = 0;
reg [3:0] sst_d = 0;

function [1:0] slot_dom;
    input [20:0] b;
    reg [31:0] w; reg [15:0] p; reg [23:0] c;
    begin
        w = sdram[b + 200*160 + 10]; p = w[15:0];
        c = {p[15:11], p[15:13], p[10:5], p[10:9], p[4:0], p[4:2]};
        slot_dom = dom(c);
    end
endfunction
function [1:0] hd_dom;
    input integer dummy;
    reg [31:0] w;
    begin w = sdram[HD_ADDR + 400*640 + 20]; hd_dom = dom(w[23:0]); end
endfunction
function [20:0] taddr;
    input integer s;
    taddr = (s == 0) ? THUMB0_ADDR : (s == 1) ? THUMB1_ADDR : THUMB2_ADDR;
endfunction
// 字符码 → 字库偏移（与 gb_addr.v 同一公式）
function [23:0] gbaddr;
    input [15:0] c;
    reg [7:0] qb, wb;
    begin
        qb = c[15:8]; wb = c[7:0];
        if (qb == 8'h00)                       gbaddr = {11'd0, wb, 5'd0};
        else if (qb >= 8'hA1 && qb <= 8'hA9)   gbaddr = 24'h1000 + (((qb - 8'hA1) * 94 + (wb - 8'hA1)) * 32);
        else if (qb >= 8'hB0 && qb <= 8'hF7)   gbaddr = 24'h1000 + (((qb - 8'hB0 + 9) * 94 + (wb - 8'hA1)) * 32);
        else                                   gbaddr = 24'd0;
    end
endfunction

always @(posedge clk) if (!rst) begin
    cm_d <= sub_commit_tgl;
    if (sub_commit_tgl !== cm_d) begin
        n_commit = n_commit + 1; t_commit = $time;
        $display("  [sub] t=%0t commit #%0d len=%0d bank=%0d", $time, n_commit, sub_len, sub_bank);
    end
    if (sub_start) begin
        n_start = n_start + 1;
        $display("  [sub] t=%0t start found=%b sec=%0d", $time, sub_has, sub_sec);
    end
    if (ld_done && u_app.u_ctrl.ld_fmt_r) begin
        t_hd = $time;
        $display("  [hd] t=%0t HD done err=%b img=%0d", $time, ld_err, u_app.u_ctrl.hd_img);
    end
    if (ld_req) $display("  [ld] t=%0t req fmt=%b off=%0d", $time, ld_fmt, ld_off);
    if (mm_win) $display("  [mm] t=%0t scan rel=%0d", $time, mm_rel);
    if (mm_hit) $display("  [mm] t=%0t WAV hit sec=%0d same=%b", $time, lst_match_off, mm_same);
    if (lst_match_txt_hit && lst_own == OWN_MM) $display("  [mm] t=%0t TXT hit sec=%0d", $time, lst_match_off);
    if (dbg_state !== sst_d) begin
        $display("  [st] t=%0t %0d -> %0d cur=%0d", $time, sst_d, dbg_state, u_app.u_ctrl.cur_abs);
        sst_d = dbg_state;
    end
end

// 回到 S_HD、装载器（含预取）空闲、字幕不再变化
task settle_sub;
    integer g, st;
    begin
        g = 0; st = 0;
        while (st < 300000 && g < 300_000_000) begin
            @(posedge clk); g = g + 1;
            if (dbg_state == 5 && u_app.u_ctrl.ldp == 0 && !u_app.u_ctrl.u_sw.O_tl_req && !ld_busy
                && !lst_busy && !sub_sd_req && u_sub_loader.gst == 3'd0 && !u_sub_loader.new_rdy)
                st = st + 1;
            else st = 0;
        end
        if (st < 300000) $display("  [warn] did not settle (state=%0d ldp=%0d)", dbg_state, u_app.u_ctrl.ldp);
    end
endtask

task check_step;
    input integer exp_cur;
    input integer exp_len;
    input [15:0]  exp_c0;
    input [22:0]  exp_trk;
    input integer commits;
    input integer after_hd;       // 1 = 要求字幕提交晚于高清完成（扫描/读 TXT 不打断装载）
    integer c, cs, r, a;
    reg [15:0] g, f;
    begin
        c = u_app.u_ctrl.cur_abs; cs = u_app.u_ctrl.u_sw.cs;
        $display("  state=%0d cur=%0d slot=%0d hd=%0d len=%0d trk=%0d commits=%0d (t_commit=%0t t_hd=%0t)",
                 dbg_state, c, slot_dom(taddr(cs)), hd_dom(0), sub_len, trk_sec_r, commits, t_commit, t_hd);
        chk("image: cur", c, exp_cur);
        chk("image: current thumb slot", slot_dom(taddr(cs)), exp_cur);
        chk("image: HD = current", hd_dom(0), exp_cur);
        chk("sub: one commit this step", commits, 1);
        chk("sub: text_len", sub_len, exp_len);
        if (exp_len != 0) begin
            chk("sub: first code", u_sub_loader.code_mem[{sub_bank, 8'd0}], exp_c0);
            a = gbaddr(exp_c0);
            for (r = 0; r < 16; r = r + 1) begin
                g = u_dyn_subtitle.glyph_mem[{sub_bank, 6'd0, r[3:0]}];
                f = {u_flash.mem[a + 2*r], u_flash.mem[a + 2*r + 1]};
                if (g !== f) begin
                    errors = errors + 1;
                    $display("  [FAIL] glyph row %0d = %04h exp %04h", r, g, f);
                end
            end
        end
        chk("music: trk_sec", trk_sec_r, exp_trk);
        if (after_hd) begin
            if (t_commit > t_hd && t_hd > t_press) $display("  [ ok ] subtitle committed after HD done (load not interrupted)");
            else begin errors = errors + 1; $display("  [FAIL] subtitle not after HD (commit %0t, HD %0t)", t_commit, t_hd); end
        end
    end
endtask

integer c0;
initial begin
    rst = 1; wait_cyc(20); rst = 0;

    $display("--- S0 boot on IMG01 ---");
    wait_state(5, 300_000_000);
    settle_sub();
    check_step(0, SUB_LEN1, SUB_C1_0, WAV1_SEC, n_commit, 0);

    $display("--- S1 KEY3 -> IMG02 (long subtitle) ---");
    c0 = n_commit; t_press = $time; press(2);
    settle_sub();
    check_step(1, SUB_LEN2, SUB_C2_0, WAV2_SEC, n_commit - c0, 1);
    chk("sub: char 64 parsed", u_sub_loader.code_mem[{sub_bank, 8'd64}], SUB_C2_64);

    $display("--- S2 KEY4 -> back to IMG01 ---");
    c0 = n_commit; t_press = $time; press(3);
    settle_sub();
    check_step(0, SUB_LEN1, SUB_C1_0, WAV1_SEC, n_commit - c0, 1);

    $display("--- S3 KEY3 -> IMG02 again ---");
    c0 = n_commit; t_press = $time; press(2);
    settle_sub();
    check_step(1, SUB_LEN2, SUB_C2_0, WAV2_SEC, n_commit - c0, 1);

    $display("--- S4 KEY3 -> IMG03 (no TXT / WAV) ---");
    c0 = n_commit; t_press = $time; press(2);
    settle_sub();
    check_step(2, 0, 16'd0, WAV2_SEC, n_commit - c0, 0);

    // ---- 快速来回：高清刚排队 / 正在读卡时就切回去（上板"切不回去"的现场）----
    $display("--- S5 KEY4 -> IMG02, then KEY4 again right after its HD is queued ---");
    c0 = n_commit; t_press = $time; press(3);
    while (!u_app.u_ctrl.hd_take) @(posedge clk);
    wait_cyc(2000);
    $display("  second KEY4 at t=%0t ldp=%0d", $time, u_app.u_ctrl.ldp);
    press(3);
    settle_sub();
    check_step(0, SUB_LEN1, SUB_C1_0, WAV1_SEC, (n_commit - c0 >= 1) ? 1 : 0, 0);

    $display("--- S6 KEY3 -> IMG02, then KEY4 while its HD is reading the card ---");
    c0 = n_commit; t_press = $time; press(2);
    while (!(u_app.u_ctrl.ldp == 2 && u_app.u_ctrl.ld_fmt_r)) @(posedge clk);
    wait_cyc(200000);
    $display("  KEY4 at t=%0t ldp=%0d", $time, u_app.u_ctrl.ldp);
    press(3);
    settle_sub();
    check_step(0, SUB_LEN1, SUB_C1_0, WAV1_SEC, (n_commit - c0 >= 1) ? 1 : 0, 0);

    $display("");
    if (errors == 0) $display("ALL PASS");
    else             $display("FAIL: %0d errors", errors);
    $finish;
end

initial begin
    #(64'd8_000_000_000);
    $display("FAIL: global timeout");
    $finish;
end
