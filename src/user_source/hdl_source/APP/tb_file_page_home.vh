//==============================================================================
//  tb_file_page_home.vh —— tb_file_page 的主页换图剧本（+define+FP_HOME）
//==============================================================================
//  卡 = fat32_4img（+FP_BIG 换 640×480 版）：IMG01..04 = 红/绿/蓝/黄（中间有大号数字）。
//  2026-11-04 新口径（img_switch 环形缓存）：
//    H0  开机：当前图 = 1，缓存 = [1, 2, 空]（cur 在槽 cs，下一张在 cs+1）
//    H1  KEY3 ×3：每次换图后 上一张/当前/下一张 都在各自的槽里，高清 = 当前图
//    H2  末张再按 KEY3：不动
//    H3  KEY4 后在 S_LOW（装高清中）立刻再按 KEY4：不等高清，直接开始下一次过渡，
//        被打断的高清装载走 abort（bmp_loader 报 err），最后停在图 2、高清 = 图 2
//  每一步把 3 个缩略图槽 + 过渡中间帧抓成 PPM（out/h*_t*.ppm、out/h_*_trans.ppm）。
//==============================================================================
integer hn, hstage = 0, n_abort = 0, n_err = 0;

// 纯色图左侧一列（避开中间的数字）判主色：0=R 1=G 2=B 3=黄
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

// 把一个缩略图槽（160 字 × 240 行，每字 2 个 565 像素）写成 320×240 PPM
task dump_thumb;
    input [20:0] b;
    input [8*32-1:0] fname;
    integer fd, w; reg [15:0] p; reg [31:0] d;
    begin
        fd = $fopen(fname, "wb");
        $fwrite(fd, "P6\n320 240\n255\n");
        for (w = 0; w < 38400; w = w + 1) begin
            d = sdram[b + w];
            p = d[15:0];  $fwrite(fd, "%c%c%c", {p[15:11],p[15:13]}, {p[10:5],p[10:9]}, {p[4:0],p[4:2]});
            p = d[31:16]; $fwrite(fd, "%c%c%c", {p[15:11],p[15:13]}, {p[10:5],p[10:9]}, {p[4:0],p[4:2]});
        end
        $fclose(fd);
    end
endtask

// 静止判据：cur = exp；cur−1 / cur / cur+1（存在的）都在各自的槽里；高清 = cur
task check_cache;
    input integer exp_cur;
    input [8*8-1:0] tag;
    integer c, cs, d, j;
    begin
        c = u_app.u_ctrl.cur_abs; cs = u_app.u_ctrl.u_sw.cs;
        $display("  %0s: cur=%0d cs=%0d tv=%b slots=%0d/%0d/%0d hd=%0d", tag, c, cs, u_app.u_ctrl.u_sw.tv,
                 slot_dom(THUMB0_ADDR), slot_dom(THUMB1_ADDR), slot_dom(THUMB2_ADDR), hd_dom(0));
        chk("cur", c, exp_cur);
        for (d = -1; d <= 1; d = d + 1) begin
            j = c + d;
            if (j >= 0 && j < 4) chk("slot holds image", slot_dom(taddr((cs + d + 3) % 3)), j);
        end
        chk("HD = current", hd_dom(0), c);
        case (hstage)
          0: begin dump_thumb(THUMB0_ADDR, "out/h0_t0.ppm"); dump_thumb(THUMB1_ADDR, "out/h0_t1.ppm"); dump_thumb(THUMB2_ADDR, "out/h0_t2.ppm"); end
          1: begin dump_thumb(THUMB0_ADDR, "out/h1_t0.ppm"); dump_thumb(THUMB1_ADDR, "out/h1_t1.ppm"); dump_thumb(THUMB2_ADDR, "out/h1_t2.ppm"); end
          2: begin dump_thumb(THUMB0_ADDR, "out/h2_t0.ppm"); dump_thumb(THUMB1_ADDR, "out/h2_t1.ppm"); dump_thumb(THUMB2_ADDR, "out/h2_t2.ppm"); end
          3: begin dump_thumb(THUMB0_ADDR, "out/h3_t0.ppm"); dump_thumb(THUMB1_ADDR, "out/h3_t1.ppm"); dump_thumb(THUMB2_ADDR, "out/h3_t2.ppm"); end
          default: ;
        endcase
        hstage = hstage + 1;
    end
endtask

// 等"回到 S_HD 且装载器（含后台预取）闲下来"
task settle_home;
    integer g, st;
    begin
        g = 0; st = 0;
        while (st < 200000 && g < 400_000_000) begin
            @(posedge clk); g = g + 1;
            if (dbg_state == 5 && u_app.u_ctrl.ldp == 0 && !u_app.u_ctrl.u_sw.O_tl_req && !ld_busy)
                st = st + 1;
            else st = 0;
        end
        if (st < 200000) $display("  [warn] did not settle (state=%0d)", dbg_state);
    end
endtask

task wait_nfr;              // 等 n 帧（带超时：静止 HD 不出帧）
    input integer n;
    integer f0, g;
    begin
        f0 = frames; g = 0;
        while (frames < f0 + n && g < 20_000_000) begin @(posedge clk); g = g + 1; end
    end
endtask

// 状态变化 / 装载探针
reg [3:0] hst_d = 0;
always @(posedge clk) if (!rst) begin
    if (dbg_state !== hst_d) begin
        $display("  [st] t=%0t %0d -> %0d cur=%0d cs=%0d a_base=%0d b_base=%0d", $time, hst_d, dbg_state,
                 u_app.u_ctrl.cur_abs, u_app.u_ctrl.u_sw.cs, u_app.u_ctrl.a_base, u_app.u_ctrl.b_base);
        hst_d = dbg_state;
    end
    if (ld_req) $display("  [ld] t=%0t fmt=%b off=%0d base=%0d", $time, ld_fmt, ld_off, ld_base);
    if (app_ld_abort) begin n_abort = n_abort + 1; $display("  [abort] t=%0t", $time); end
    if (ld_err && !quad_own) n_err = n_err + 1;
end

initial begin
    rst = 1; wait_cyc(20); rst = 0;
    $display("--- H0 boot ---");
    wait_state(5, 300_000_000);
    settle_home();
    dump_ppm("out/h_00_boot.ppm");
    check_cache(0, "boot");
    // 开机只装当前 + 下一张：第 3 张那块不该被装过（槽里仍是 0）
    chk("boot: 3rd slot untouched", sdram[taddr((u_app.u_ctrl.u_sw.cs + 2) % 3) + 200*160 + 10], 0);

    for (hn = 1; hn <= 3; hn = hn + 1) begin
        $display("--- H1 switch %0d (KEY3) ---", hn);
        press(2);
        wait_state(3, 50_000_000);
        wait_nfr(4);
        case (hn)
            1: dump_ppm("out/h_01_trans.ppm");
            2: dump_ppm("out/h_02_trans.ppm");
            default: dump_ppm("out/h_03_trans.ppm");
        endcase
        settle_home();
        check_cache(hn, "next");
    end

    $display("--- H2 KEY3 at the last image ---");
    press(2);
    wait_cyc(2_000_000);
    chk("H2 stays in S_HD", dbg_state, 5);
    chk("H2 stays on image 4", u_app.u_ctrl.cur_abs, 3);

    $display("--- H3 KEY4, then KEY4 again while the HD is loading ---");
    press(3);
    wait_state(2, 300_000_000);                 // S_LOW：过渡播完，开始装高清
    while (!(u_app.u_ctrl.ldp == 2 && u_app.u_ctrl.ld_fmt_r)) @(posedge clk);
    wait_cyc(20000);                            // 高清已经在读卡了
    press(3);
    wait_cyc(10);
    chk("H3 next switch started at once (S_ARM)", dbg_state, 7);
    settle_home();
    chk("H3 HD load was aborted", (n_abort > 0) ? 1 : 0, 1);
    check_cache(1, "lowkey");

    $display("");
    if (errors == 0) $display("ALL PASS");
    else             $display("FAIL: %0d errors", errors);
    $finish;
end
initial begin
    #(64'd30_000_000_000);
    $display("FAIL: home timeout");
    $finish;
end
