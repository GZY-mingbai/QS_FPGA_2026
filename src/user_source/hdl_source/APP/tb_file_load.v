//==============================================================================
//  tb_file_load.v —— 文件页四宫格【装载链路】集成台（不打像素，跑得快）
//==============================================================================
//  【为什么单独做这个台子】
//  tb_chain_full 把整条链路（含 640×480 出帧）都跑了，但一帧就要几十万拍、
//  本机只有几千拍/秒 ⇒ 判"四宫格装没装上"要等半小时。
//  这里把 App / viewer_scene / OSD / ps_gen_image 全部拿掉，只留
//  **卡 → 仲裁器 → lister → quad_page_ctrl → quad_loader → bmp_loader → SDRAM**，
//  一次跑完几秒钟，直接回答三个问题：
//      ① 进文件页之后 quad_ready 到底有没有起来（没起来 ⇒ 四宫格永远不画）
//      ② 4 个槽有没有真的被写（sentinel 有没有被覆盖）
//      ③ 每个槽里装的是哪张图（和 .map 里的偏移对得上吗）
//
//  【判据】
//    · 槽 k 的首字 != SENTINEL（说明写过）
//    · 槽 k 里的像素颜色 == 镜像里第 k 张图的主色（.map 的 r/g/b）
//    · 翻页后槽内容变成第 5..8 张
//
//  跑法（工作目录 = APP/）：
//    iverilog -g2012 -o /tmp/fl.vvp -s tb_file_load tb_file_load.v \
//        album_src.v quad_loader.v quad_page_ctrl.v \
//        ../SD/fat_dir_lister.v ../SD/bmp_loader.v ../SD/bmp_scale.v \
//        ../SD/bmp_read.v ../SD/bmp_view.v ../SD/bmp_view_fill.v ../SD/ph_writer.v \
//        ../SD/sd_bus_arbiter4.v ../SD/sd_sector_file_model.v
//    vvp /tmp/fl.vvp
//==============================================================================
`timescale 1ns/1ps

module tb_file_load;

localparam [20:0] QBASE = 21'd1048576;
localparam integer SLOT_STEP = 40960;
localparam [31:0] SENT = 32'hDEAD_BEEF;

reg clk = 0, mclk = 0, rst = 1;
always #10 clk  = ~clk;      // 50MHz
always #5  mclk = ~mclk;     // 100MHz

integer cyc = 0;
always @(posedge clk) cyc <= cyc + 1;

//------------------------------------------------------------------------------
//  卡 + 仲裁器
//------------------------------------------------------------------------------
wire        sd_init_done;
wire        sd_sec_read, sd_sec_dv, sd_sec_end;
wire [31:0] sd_sec_addr;
wire [7:0]  sd_sec_data;

sd_sector_file_model #(
    .INIT_FILE ("../../../../sim_data/fat32_10img.img"),
    .LAT_CYC   (40),
    .MAX_LBA   (32'd131071)
) u_card (
    .clk(clk), .rst(rst),
    .O_init_done(sd_init_done),
    .I_sec_read(sd_sec_read), .I_sec_addr(sd_sec_addr),
    .O_sec_data(sd_sec_data), .O_sec_dv(sd_sec_dv), .O_sec_end(sd_sec_end),
    .O_rd_pulse(), .O_rd_lba()
);

wire        img_req, img_dv, img_end;
wire [31:0] img_addr;
wire [7:0]  img_data;
wire        fat_req, fat_dv, fat_end;
wire [31:0] fat_addr;
wire [7:0]  fat_data;

sd_bus_arbiter4 #(.ENA_TXT(0)) u_arb (
    .I_clk(clk), .I_rst(rst), .I_sd_init_done(sd_init_done),
    .I_img_req(img_req), .I_img_addr(img_addr),
    .O_img_data(img_data), .O_img_dv(img_dv), .O_img_end(img_end),
    .I_aud_req(1'b0), .I_aud_addr(32'd0), .I_aud_urgent(1'b0),
    .O_aud_data(), .O_aud_dv(), .O_aud_end(),
    .I_fat_req(fat_req), .I_fat_addr(fat_addr),
    .O_fat_data(fat_data), .O_fat_dv(fat_dv), .O_fat_end(fat_end),
    .I_txt_req(1'b0), .I_txt_addr(32'd0),
    .O_txt_data(), .O_txt_dv(), .O_txt_end(),
    .O_sd_sec_read(sd_sec_read), .O_sd_sec_addr(sd_sec_addr),
    .I_sd_sec_data(sd_sec_data), .I_sd_sec_dv(sd_sec_dv), .I_sd_sec_end(sd_sec_end),
    .O_timeout(), .O_owner()
);

//------------------------------------------------------------------------------
//  lister / 相册 / 四宫格
//------------------------------------------------------------------------------
wire        lst_start, lst_busy, lst_done, lst_has_more;
wire [2:0]  lst_cnt;
wire [1:0]  lst_rd_slot, lst_ld_slot;
wire [3:0]  lst_rd_idx;
wire [7:0]  lst_rd_char;
wire [31:0] lst_off, lst_size;
wire [3:0]  lst_blen, lst_elen, lst_type, lst_valid;
wire [19:0] lst_len4;
wire        lst_vld;
wire [31:0] lst_root, lst_fat;
wire [3:0]  lst_spc;
wire        lst_bpb_ok;
wire [15:0] lst_offset;

wire        alb_lst_start, alb_sel, alb_win4, alb_ready, alb_any;
wire [1:0]  alb_rd_slot;
wire [31:0] alb_off0, alb_off1, alb_off2, alb_off3;
wire [2:0]  alb_nimg;
// 本台没有 viewer_ctrl，窗口起点直接钉 0（相册只认前 3 张）
wire [15:0] alb_base = 16'd0;

wire        qp_lst_start;
wire [15:0] qp_lst_offset;
wire        qp_ld_start, quad_own, quad_ready, quad_has_more, fp_gen_w;
wire [15:0] quad_off;
wire [2:0]  quad_vcnt;

wire        ql_busy, ql_done;
wire [2:0]  ql_loaded;
wire [1:0]  ql_rd_slot;
wire [31:0] ql_off, ql_len;
wire [20:0] ql_base;
wire        ph_start, ph_busy, ph_done, ph_wr_vld;
wire [1:0]  ph_kind;
wire [20:0] ph_base, ph_wr_addr;
wire [31:0] ph_wr_data;

wire        ql_ld_start, ld_ready, ld_busy, ld_done, ld_err;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
wire [15:0] ld_src_w, ld_src_h;
wire        ld_wr_vld, ld_wr_ack, ld_wr_settled;
wire [9:0]  ld_wr_used;
wire [20:0] ld_wr_addr;
wire [31:0] ld_wr_data;

reg         file_page = 0;          // 本台自己当 OSD：直接给"是否停在文件页"
reg         flip_nx = 0, flip_pv = 0;

assign lst_start = file_page ? qp_lst_start : alb_lst_start;
assign lst_offset = file_page ? qp_lst_offset : alb_base;

reg lst_own_fp;
wire lst_done_fp = lst_done &  lst_own_fp;
wire lst_done_al = lst_done & ~lst_own_fp;
always @(posedge clk or posedge rst) begin
    if (rst)            lst_own_fp <= 1'b0;
    else if (lst_start) lst_own_fp <= file_page;
end

album_src u_album (
    .clk(clk), .rst(rst),
    .I_base(alb_base), .I_need(1'b1), .I_block(file_page), .I_rescan(1'b0),
    .O_lst_start(alb_lst_start), .O_lst_offset(), .O_rd_slot(alb_rd_slot),
    .I_lst_busy(lst_busy), .I_lst_done(lst_done_al),
    .I_ld_type(lst_type), .I_slot_valid(lst_valid), .I_file_off(lst_off),
    .O_off0(alb_off0), .O_off1(alb_off1), .O_off2(alb_off2), .O_off3(alb_off3),
    .O_any(alb_any), .O_nimg(alb_nimg), .O_ready(alb_ready), .O_win4(alb_win4),
    .O_sel(alb_sel)
);

fat_dir_lister #(
    .N_ITEMS(4), .MAX_SECS(64), .INCLUDE_DIR(0)
) u_fat (
    .clk(clk), .rst(rst),
    .I_start(lst_start), .I_offset(lst_offset),
    // 与顶层一致：相册扫描只数 BMP，文件页扫描按条目
    .I_list_all_types(file_page),
    // ★ 2026-11-02：match 口（音随图切）。本台不测该功能 ⇒ 显式接 0，
    //   否则 mm_on 变 x，want_this 跟着变 x，整个扫描结果都不可信。
    .I_match_en(1'b0), .I_match_rel(2'd0),
    .O_busy(lst_busy), .O_done(lst_done), .O_valid_cnt(lst_cnt), .O_has_more(lst_has_more),
    .I_rd_slot(lst_rd_slot), .I_rd_idx(lst_rd_idx), .O_rd_char(lst_rd_char),
    .I_ld_slot(lst_ld_slot),
    .O_ld_file_off(lst_off), .O_ld_size(lst_size), .O_ld_namelen(),
    .O_ld_blen(lst_blen), .O_ld_elen(lst_elen), .O_ld_len4(lst_len4),
    .O_ld_vld(lst_vld), .O_ld_type(lst_type), .O_slot_valid(lst_valid),
    .O_root_lba(lst_root), .O_fat_lba(lst_fat), .O_spc_shift(lst_spc), .O_bpb_ok(lst_bpb_ok),
    .O_sd_req(fat_req), .O_sd_addr(fat_addr),
    .I_sd_data(fat_data), .I_sd_dv(fat_dv), .I_sd_end(fat_end)
);

quad_page_ctrl #(.N_ITEMS(4), .STUCK_CYC(800000)) u_qpage (
    .clk(clk), .rst(rst),
    .I_file_page(file_page),
    .I_flip_next(flip_nx), .I_flip_prev(flip_pv),
    .O_lst_start(qp_lst_start), .O_lst_offset(qp_lst_offset),
    .I_lst_busy(lst_busy), .I_lst_done(lst_done_fp), .I_lst_cnt(lst_cnt),
    .I_lst_has_more(lst_has_more),
    .O_ld_start(qp_ld_start), .I_ld_busy(ql_busy), .I_ld_done(ql_done), .I_ld_filled(ql_filled),
    .I_ld_loaded(ql_loaded),
    .O_own(quad_own), .O_ready(quad_ready), .O_has_more(quad_has_more),
    .O_offset(quad_off), .O_gen(fp_gen_w), .O_valid_cnt(quad_vcnt)
);

quad_loader #(.SLOT_BASE(QBASE), .SLOT_STEP(SLOT_STEP), .N_ITEMS(4)) u_qload (
    .clk(clk), .rst(rst),
    .I_start(qp_ld_start), .I_valid_cnt(lst_cnt), .I_slot_valid(lst_valid),
    .O_busy(ql_busy), .O_done(ql_done), .O_loaded(ql_loaded), .O_filled(ql_filled),
    .O_rd_slot(ql_rd_slot),
    .I_file_off(lst_off), .I_file_len(lst_size), .I_file_type(lst_type),
    .O_ph_start(ph_start), .O_ph_kind(ph_kind), .O_ph_base(ph_base),
    .I_ph_busy(ph_busy), .I_ph_done(ph_done),
    .O_ld_start(ql_ld_start), .O_ld_off(ql_off), .O_ld_len(ql_len), .O_ld_base(ql_base),
    .I_ld_ready(ld_ready), .I_ld_busy(ld_busy), .I_ld_done(ld_done), .I_ld_err(ld_err)
);

// 元数据口分时（照抄 top）
assign lst_ld_slot = alb_sel ? alb_rd_slot : (ql_busy ? ql_rd_slot : lst_rd_slot);

//------------------------------------------------------------------------------
//  装载器 + 占位图 + SDRAM
//------------------------------------------------------------------------------
wire        bmp_start = quad_own ? ql_ld_start : 1'b0;
wire [31:0] bmp_off   = ql_ld_start ? ql_off : 32'd0;
wire [20:0] bmp_base  = ql_ld_start ? ql_base : 21'd0;
wire [1:0]  bmp_mode  = quad_own ? 2'd1 : 2'd1;
wire        bmp_fmt   = 1'b0;
wire [31:0] bmp_len   = 32'd0;

bmp_loader #(.AFIFO_AW(9)) u_loader (
    .clk(clk), .rst(rst), .mclk(mclk),
    .start(bmp_start), .file_offset(bmp_off), .file_length(bmp_len), .dst_base(bmp_base),
    .mode(bmp_mode), .fmt(bmp_fmt), .dst_stride(11'd0),
    .zoom(8'h10), .ax(10'd320), .ay(10'd240),
    .ready(ld_ready), .busy(ld_busy), .done(ld_done), .err(ld_err),
    .src_w_o(ld_src_w), .src_h_o(ld_src_h),
    .sd_sec_read(img_req), .sd_sec_read_addr(img_addr),
    .sd_sec_read_data(img_data), .sd_sec_read_data_valid(img_dv),
    .sd_sec_read_end(img_end), .sd_init_done(sd_init_done),
    .wr_vld(ld_wr_vld), .wr_used(ld_wr_used), .wr_addr(ld_wr_addr),
    .wr_data(ld_wr_data), .wr_ack(ld_wr_ack), .wr_settled(ld_wr_settled)
);

ph_writer u_ph (
    .clk(clk), .rst(rst),
    .I_start(ph_start), .I_kind(ph_kind), .I_base(ph_base),
    .O_busy(ph_busy), .O_done(ph_done),
    .O_wr_vld(ph_wr_vld), .O_wr_addr(ph_wr_addr), .O_wr_data(ph_wr_data),
    .I_wr_ack(ld_wr_ack)
);

wire        sdr_ld_vld  = ph_busy ? ph_wr_vld  : ld_wr_vld;
wire [20:0] sdr_ld_addr = ph_busy ? ph_wr_addr : ld_wr_addr;
wire [31:0] sdr_ld_data = ph_busy ? ph_wr_data : ld_wr_data;

assign ld_wr_ack = sdr_ld_vld;

// ★ 2026-11-02【补上缺失的 wr_settled 模型】
//   `bmp_loader` 的 L_DRN 必须等 `wr_settled`（= 写通道"已沉降"）才肯 done。
//   本台原来是 100M-15 之前的写法，这根线**从未被驱动**（Z→X）⇒ 装载器第一单
//   流完像素后永远卡在 L_DRN（上板现象就是"装载全挂住"：`ld_busy` 恒 1、
//   四宫格只写了第 0 格）。真机上它来自 `sdr_port_sched`：
//       ld_wr_settled = (ld_settle_cnt >= 32)，条件 = FIFO 空 + 无在发组 + 暂存空…
//   这里用一个等价模型：**连续 32 拍没有写请求**就报"已沉降"。
reg [7:0] ws_idle = 8'd0;
always @(posedge clk or posedge rst) begin
    if (rst)                 ws_idle <= 8'd0;
    else if (sdr_ld_vld)     ws_idle <= 8'd0;
    else if (ws_idle != 8'hFF) ws_idle <= ws_idle + 8'd1;
end
assign ld_wr_settled = (ws_idle >= 8'd32);

reg [31:0] dram [0:2097151];
integer di;
// 只把四宫格那 4×40960 字清成 sentinel（整块 2M 清一遍太慢）
initial for (di = QBASE; di < QBASE + 4*SLOT_STEP; di = di + 1) dram[di] = SENT;
always @(posedge clk) if (sdr_ld_vld) dram[sdr_ld_addr] <= sdr_ld_data;

//------------------------------------------------------------------------------
//  检查 / 统计
//------------------------------------------------------------------------------
integer errors = 0;
integer k;
reg [31:0] w;
integer rr, gg, bb;

// 期望主色（与 sim_data/fat32_10img.map 一致）
integer exp_r [0:11], exp_g [0:11], exp_b [0:11];
initial begin
    exp_r[0]=200; exp_g[0]= 40; exp_b[0]= 40;
    exp_r[1]= 40; exp_g[1]=200; exp_b[1]= 40;
    exp_r[2]= 40; exp_g[2]= 60; exp_b[2]=220;
    exp_r[3]=210; exp_g[3]=200; exp_b[3]= 40;
    exp_r[4]=200; exp_g[4]= 40; exp_b[4]=200;
    exp_r[5]= 40; exp_g[5]=200; exp_b[5]=200;
    exp_r[6]=230; exp_g[6]=120; exp_b[6]= 30;
    exp_r[7]=130; exp_g[7]= 50; exp_b[7]=230;
    exp_r[8]= 40; exp_g[8]=220; exp_b[8]=150;
    exp_r[9]=230; exp_g[9]= 50; exp_b[9]=110;
end

// 纯色图 ⇒ 取中心像素即可代表该槽装的是哪张
reg [31:0] pix;
integer r5, g6, b5;

task check_slot;
    input integer slot;
    input integer expect_img;    // 期望是第几张（从 0 数）
    begin
        // 取该槽"最有代表性"的颜色：统计 320×240 里出现最多的 (r,g,b)
        // 简化：直接看 src 是纯色图（每张图整幅同色，见 .map）
        // ⇒ 任意取中心像素即可
        pix = dram[QBASE + slot*SLOT_STEP + 120*160 + 80];
        r5  = (pix[31:27]);          // RGB565: r5[4:0] g6 b5
        g6  = (pix[26:21]);
        b5  = (pix[20:16]);
        rr  = (r5 * 255 + 15) / 31;
        gg  = (g6 * 255 + 31) / 63;
        bb  = (b5 * 255 + 15) / 31;
        $display("    槽%0d: %h  =>  RGB(%0d,%0d,%0d)   期望 RGB(%0d,%0d,%0d)",
                 slot, pix, rr, gg, bb, exp_r[expect_img], exp_g[expect_img], exp_b[expect_img]);
        if (pix === SENT) begin
            $display("    [FAIL] 槽%0d 从来没被写过（还是 sentinel）", slot);
            errors = errors + 1;
        end else if ((rr - exp_r[expect_img] > 12) || (exp_r[expect_img] - rr > 12) ||
                     (gg - exp_g[expect_img] > 12) || (exp_g[expect_img] - gg > 12) ||
                     (bb - exp_b[expect_img] > 12) || (exp_b[expect_img] - bb > 12)) begin
            $display("    [FAIL] 槽%0d 颜色对不上：装的很可能是别的图", slot);
            errors = errors + 1;
        end else
            $display("    ok   槽%0d == IMG%02d", slot, expect_img + 1);
    end
endtask

task dump_slots;
    input [8*24-1:0] tag;
    integer sx, sy, s, wd;
    reg [31:0] v;
    reg [7:0] r8, g8, b8;
    begin
        // 2×2 拼图，每格 320×240
        wd = $fopen(tag, "w");
        $fwrite(wd, "P6\n%0d %0d\n255\n", 640, 480);
        for (sy = 0; sy < 480; sy = sy + 1) begin
            for (sx = 0; sx < 640; sx = sx + 1) begin
                s  = (sy >= 240) ? ((sx >= 320) ? 3 : 2) : ((sx >= 320) ? 1 : 0);
                v  = dram[QBASE + s*SLOT_STEP + (sy % 240)*160 + ((sx % 320) / 2)];
                if ((sx % 320) % 2 == 0) begin
                    r8 = {v[31:27], v[31:29]}; g8 = {v[26:21], v[26:24]}; b8 = {v[20:16], v[20:18]};
                end else begin
                    r8 = {v[15:11], v[15:13]}; g8 = {v[10:5], v[10:8]};  b8 = {v[4:0], v[4:2]};
                end
                $fwrite(wd, "%c%c%c", r8, g8, b8);
            end
        end
        $fclose(wd);
        $display("    拼图写出：%0s", tag);
    end
endtask

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
integer t0, ok;
initial begin
    $display("== tb_file_load：四宫格装载链路 ==");
    $fflush();
    repeat (20) @(posedge clk);
    rst = 0;

    // 等卡初始化 + 相册扫完
    t0 = cyc;
    while (!sd_init_done && (cyc - t0) < 2_000_000) @(posedge clk);
    $display("  卡初始化完成（%0d 拍）", cyc - t0);
    t0 = cyc;
    while (!alb_ready && (cyc - t0) < 4_000_000) @(posedge clk);
    $display("  相册扫描完成 ready=%0d have=%0d win4=%0d off0=%h off1=%h off2=%h off3=%h",
             alb_ready, alb_nimg, alb_win4, alb_off0, alb_off1, alb_off2, alb_off3);
    $fflush();

    // ---- 进文件页 ----
    $display("  -- 进文件页（file_page=1）--");
    t0 = cyc;
    @(negedge clk); file_page = 1;
    ok = 0;
    while ((cyc - t0) < 8_000_000 && !ok) begin
        @(posedge clk);
        if (cyc - t0 == 500_000)   $display("    ...50万拍: lst_busy=%b cnt=%0d ql_busy=%b ld_busy=%b ready=%b own=%b",
                                            lst_busy, lst_cnt, ql_busy, ld_busy, quad_ready, quad_own);
        if (cyc - t0 == 2_000_000) $display("    ...200万拍: lst_busy=%b cnt=%0d ql_busy=%b ld_busy=%b ready=%b own=%b",
                                            lst_busy, lst_cnt, ql_busy, ld_busy, quad_ready, quad_own);
        if (quad_ready) ok = 1;
    end
    $display("  quad_ready=%0d 用了 %0d 拍；lst_cnt=%0d ql_loaded=%0d vcnt=%0d own=%0d src=%0dx%0d",
             quad_ready, cyc - t0, lst_cnt, ql_loaded, quad_vcnt, quad_own, ld_src_w, ld_src_h);
    if (!quad_ready) begin
        $display("  [FAIL] quad_ready 从来没起来 ⇒ 四宫格永远是透明/不画");
        errors = errors + 1;
    end
    $fflush();

    repeat (2000) @(posedge clk);
    for (k = 0; k < 4; k = k + 1) check_slot(k, k);
    dump_slots("out/quad_slots.ppm");

    // ---- 翻页 ----
    $display("  -- 翻页（flip_next）--");
    @(negedge clk); flip_nx = 1;
    @(negedge clk); flip_nx = 0;
    // 先等 ready 掉下来，再等它起来
    t0 = cyc;
    while (quad_ready && ((cyc - t0) < 8_000_000)) @(posedge clk);
    if (quad_ready) begin
        $display("  [FAIL] 翻页后 ready 一直不降（翻页没生效？）");
        errors = errors + 1;
    end else
        $display("  翻页后 ready 掉下（%0d 拍）", cyc - t0);
    t0 = cyc;
    while (!quad_ready && ((cyc - t0) < 8_000_000)) @(posedge clk);
    $display("  翻页后 ready 重新起来（%0d 拍）ready=%0d", cyc - t0, quad_ready);
    repeat (2000) @(posedge clk);
    $display("  翻页后：quad_off=%0d lst_cnt=%0d ql_loaded=%0d", quad_off, lst_cnt, ql_loaded);
    for (k = 0; k < 4; k = k + 1) check_slot(k, k + 4);
    dump_slots("out/quad_slots_p2.ppm");

    $display("== errors=%0d %s ==", errors, (errors == 0) ? "PASS" : "FAIL");
    $finish;
end

initial begin
    #4_000_000_000;
    $display("TIMEOUT");
    $finish;
end

endmodule
