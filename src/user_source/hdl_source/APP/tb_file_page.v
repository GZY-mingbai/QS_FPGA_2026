`timescale 1ns/1ps
//==============================================================================
//  tb_file_page.v —— OSD 文件页（菜单第 2 页）端到端回归
//==============================================================================
//  【为什么要这个台】文件页的单元台（tb_cursor / tb_osd_keys / tb_quad_ready）
//  都绕过了顶层胶水：按键直接进 osd_menu、lister/装载器是假的。四个上板故障
//  （导航停在"文件"就自己进页 / 四格透明 / 光标不动 / 文件名缺失或抖动）
//  全部出在"模块之间"，单元台一个都抓不到。
//
//  【链路】全是真 RTL，只有 SDRAM / 写引擎 / SD 卡是模型：
//      sd_sector_file_model(FAT32 镜像) → sd_bus_arbiter4
//          ├─ 客户 0：bmp_loader ──写口──► SDRAM 模型（数组）
//          └─ 客户 2：fat_dir_lister ─► album_src / quad_page_ctrl / quad_loader
//      App（viewer_ctrl + osd_top + viewer_scene + ps_gen_image …）
//          ◄── 纹理读（从同一个 SDRAM 数组）
//          ──► frame_writer ──► 写引擎模型 + 抓帧（PPM）
//  顶层 top_tf_hdmi_audio.v 里的胶水（按键、lister/装载器二选一、quad_mode_r）
//  在下面 **逐行照抄**，改顶层时这里必须同步。
//
//  【素材】sim_data/fat32_10img.img：IMG01..IMG10.BMP（320×240 纯色）+ MUSIC01.WAV
//          + NOTE01.TXT + 隐藏的 HIDE01.BMP ⇒ 文件页 3 页：4 / 4 / 4 项。
//
//  【第三套剧本】+define+FP_SUB（见 tb_file_page_sub.vh）：fat32_sub 卡（每张图配同名
//    TXT/WAV），主页来回换图，查字幕提交 / 音随图切 / 图真的换了。胶水在 tb_file_page_subglue.vh。
//  【第二套剧本】+define+FP_CARD4（见 tb_file_page_card4.vh）：4 张图 + WAV + TXT，
//    测 2 页（第 2 页只有 2 项）、占位图、空格、往回翻页、整屏抓帧 out/c4_*.ppm。
//    +define+FP_BIG 换 640×480 的图，+define+FP_LAT=18000 用真卡的扇区延迟（慢）。
//    +define+PH_OLD 还原 HEAD 的顶层接法（ph_writer 50MHz、used=1）⇒ 复现第 2 页卡死。
//    +define+FP_HB 每 1M 拍打印一次帧计数/图层就绪（查卡帧用）。
//  【装载写口】按 sdr_port_sched 的真实规则建模：used>=4 才取、一次连弹 4 个 ack。
//    原来写成"vld 就 ack"，ph_writer 永远拿不到 ack 的上板故障就被掩盖了。
//
//  【断言】
//    F1  导航栏移到"文件"：焦点仍在导航栏、游标仍是导航游标；KEY3/KEY4 仍能移走
//    F2  KEY2 进页 → 四格装满（loaded=4），quad_mode 打开，四格画面是各自的图色
//    F3  文件名：4 行 = IMG01.BMP..IMG04.BMP，居中、连续多帧坐标不变
//    F4  KEY3/KEY4 在 4 格间移动；末格 KEY3 翻下一页、首格 KEY4 翻回并落末格
//    F5  KEY1 回导航栏（游标回左栏），再 KEY1 关 OSD，回到主页（S_HD）
//==============================================================================
module tb_file_page;

localparam [20:0] THUMB0_ADDR = 21'd614400;
localparam [20:0] THUMB1_ADDR = 21'd652800;
localparam [20:0] THUMB2_ADDR = 21'd691200;
localparam [20:0] HD_ADDR     = 21'd729600;
localparam [20:0] QUAD_BASE   = 21'd1048576;
localparam integer FAT_MAX_SECS = 64;

reg clk = 0, mclk = 0;
always #10 clk  = ~clk;      // 50MHz
always #5  mclk = ~mclk;     // 100MHz
reg rst = 1;
wire rst_all = rst;
wire sd_card_clk = clk;

integer errors = 0;
task chk;
    input [8*48-1:0] name;
    input integer got, exp;
    begin
        if (got !== exp) begin
            errors = errors + 1;
            $display("  [FAIL] %0s : got %0d exp %0d", name, got, exp);
        end else
            $display("  [ ok ] %0s = %0d", name, got);
    end
endtask

//==============================================================================
//  SD 卡 + 仲裁器
//==============================================================================
wire        sd_init_done_w;
wire        sd_bus_read;
wire [31:0] sd_bus_addr;
wire [7:0]  sd_bus_data;
wire        sd_bus_dv, sd_bus_end;

`ifndef FP_LAT
`define FP_LAT 600
`endif
`ifdef FP_BIG
`define FP_IMG "../../../../sim_data/fat32_4big.img"
`elsif FP_CARD4
`define FP_IMG "../../../../sim_data/fat32_4img.img"
`elsif FP_HOME
`define FP_IMG "../../../../sim_data/fat32_4img.img"
`elsif FP_SUB
`define FP_IMG "../../../../sim_data/fat32_sub.img"
`else
`define FP_IMG "../../../../sim_data/fat32_10img.img"
`endif
sd_sector_file_model #(
    .INIT_FILE (`FP_IMG),
    .LAT_CYC   (`FP_LAT),
    .MAX_LBA   (32'd131071),
    .PRELOAD   (6*1024*1024)
) u_card (
    .clk(clk), .rst(rst), .O_init_done(sd_init_done_w),
    .I_sec_read(sd_bus_read), .I_sec_addr(sd_bus_addr),
    .O_sec_data(sd_bus_data), .O_sec_dv(sd_bus_dv), .O_sec_end(sd_bus_end),
    .O_rd_pulse(), .O_rd_lba()
);

wire        ld_sd_req;  wire [31:0] ld_sd_addr;
wire [7:0]  ld_sd_data; wire ld_sd_dv, ld_sd_end;
wire        lst_sd_req; wire [31:0] lst_sd_addr;
// ★ 2026-11-07 字幕 TXT 客户（FP_SUB 里由 sub_loader 驱动，其余剧本恒 0）
wire        sub_sd_req; wire [31:0] sub_sd_addr;
wire        fat_req_g, txt_req_g;   // 照抄顶层：装载器忙时 match / 字幕的请求让路（赋值在胶水里）
wire [7:0]  sub_sd_data; wire sub_sd_dv, sub_sd_end;
wire [7:0]  fat_sd_data; wire fat_sd_dv, fat_sd_end;

sd_bus_arbiter4 #(
    .TIMEOUT_CYC (1_000_000), .AUD_GAP_CYC (66_000), .FAT_QUOTA (8), .ENA_TXT (1)
) u_sd_arb (
    .I_clk(clk), .I_rst(rst), .I_sd_init_done(sd_init_done_w),
    .I_img_req(ld_sd_req), .I_img_addr(ld_sd_addr),
    .O_img_data(ld_sd_data), .O_img_dv(ld_sd_dv), .O_img_end(ld_sd_end),
    .I_aud_req(1'b0), .I_aud_addr(32'd0), .I_aud_urgent(1'b0),
    .O_aud_data(), .O_aud_dv(), .O_aud_end(),
    .I_fat_req(fat_req_g), .I_fat_addr(lst_sd_addr),
    .O_fat_data(fat_sd_data), .O_fat_dv(fat_sd_dv), .O_fat_end(fat_sd_end),
    .I_txt_req(txt_req_g), .I_txt_addr(sub_sd_addr),
    .O_txt_data(sub_sd_data), .O_txt_dv(sub_sd_dv), .O_txt_end(sub_sd_end),
    .O_sd_sec_read(sd_bus_read), .O_sd_sec_addr(sd_bus_addr),
    .I_sd_sec_data(sd_bus_data), .I_sd_sec_dv(sd_bus_dv), .I_sd_sec_end(sd_bus_end),
    .O_timeout(), .O_owner()
);

//==============================================================================
//  SDRAM 模型：一个数组，装载写进去、纹理读出来
//==============================================================================
reg [31:0] sdram [0:2097151];
integer mi;
initial for (mi = 0; mi < 2097152; mi = mi + 1) sdram[mi] = 32'h0;

//==============================================================================
//  顶层胶水（照抄 top_tf_hdmi_audio.v，改顶层时同步）
//==============================================================================
reg  [3:0]  key_press = 4'd0;           // {KEY4, KEY3, KEY2, KEY1}
wire [3:0]  key_level = 4'hF;

wire        osd_file_page;
wire        osd_fp_w;
wire        quad_own, quad_ready, quad_flip_nx, quad_flip_pv, fp_gen_w;
wire [1:0]  fp_cur_w;
wire [15:0] quad_off;
wire        quad_has_more;
wire [2:0]  quad_vcnt;
wire        lst_start, lst_busy, lst_done, lst_has_more;
wire        lst_done_fp, lst_done_alb;
wire        qp_lst_start, alb_lst_start, alb_sel;
wire [1:0]  alb_rd_slot;
wire [31:0] alb_off0, alb_off1, alb_off2, alb_off3;
wire        alb_any;            // ★ 2026-11-04 照抄顶层：O_have → O_any / O_nimg
wire [2:0]  alb_nimg;
wire [15:0] alb_base;
wire        alb_win4, alb_ready;
// ★ 2026-11-07 照抄顶层：lister 三个主人（相册 / 文件页 / match 扫描）
localparam [1:0] OWN_ALB = 2'd0, OWN_FP = 2'd1, OWN_MM = 2'd2;
reg  [1:0]  lst_own;
wire        lst_own_fp = (lst_own == OWN_FP);
wire        mm_go, mm_win, mm_hit;
wire [1:0]  mm_rel;
wire        lst_match_hit, lst_match_txt_hit;
wire [22:0] lst_match_off;
wire        nm_rescan, nm_upd, nm_hold;
reg         osd_fp_d;
wire        osd_asy, dbg_osd;
wire [1:0]  hd_rel_w;
wire        sub_fl_req, sub_fl_rdy, sub_fl_valid, sub_fl_done;
wire [23:0] sub_fl_addr;
wire [15:0] sub_fl_len;
wire [7:0]  sub_fl_data;
wire [7:0]  ld_snap_zoom;
wire [9:0]  ld_snap_ax, ld_snap_ay;
wire [2:0]  lst_cnt;
wire [3:0]  lst_valid;
wire [1:0]  lst_rd_slot;
wire [3:0]  lst_rd_idx;
wire [7:0]  lst_rd_char;
wire [1:0]  lst_ld_slot;
wire [31:0] lst_off, lst_size;
wire [3:0]  lst_blen, lst_elen, lst_type;
wire [19:0] lst_len4;
wire        lst_vld;
wire        ql_start, ql_busy, ql_done;
wire [2:0]  ql_loaded;
wire [1:0]  ql_rd_slot;
wire [31:0] ql_off, ql_len;
wire [20:0] ql_base;
wire        bmp_start, bmp_start_q;
wire [31:0] bmp_off, bmp_len;
wire [20:0] bmp_base;
wire [1:0]  bmp_mode;
wire        bmp_fmt;
// ★ 2026-11-04 照抄顶层：可打断装载 / 两遍装载 / 文件页"打开"
wire        qp_ld_abort, app_ld_abort, ql_filled, quad_has_prev, fp_open_w;
wire [3:0]  ql_bmp;
wire [7:0]  quad_rank;
reg         op_ph = 0, jump_r = 0, trk_set_r = 0;
reg  [7:0]  jump_idx_r = 0;
reg  [22:0] trk_sec_r = 0;

wire        ld_req, ld_fmt;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
wire        ld_ready, ld_can_load, ld_busy, ld_done, ld_err;
wire [15:0] ld_src_w, ld_src_h;
wire [7:0]  ld_zoom_w;
wire [9:0]  ld_ax_w, ld_ay_w;
wire        oz_on_w;

wire        ps_new_frame, ps_start;
reg quad_mode_r;
reg [3:0] quad_vm_r;
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        quad_mode_r <= 1'b0;
        quad_vm_r   <= 4'd0;
    end else if (ps_new_frame) begin
        quad_mode_r <= quad_ready & osd_file_page;
        quad_vm_r   <= {quad_vcnt > 3'd3, quad_vcnt > 3'd2, quad_vcnt > 3'd1, quad_vcnt != 3'd0};
    end
end
assign osd_file_page = osd_fp_w;

assign mm_win       = mm_go && !osd_file_page && !qp_lst_start && !alb_lst_start;
assign lst_start    = osd_file_page ? qp_lst_start : (alb_lst_start ? 1'b1 : mm_go);
assign lst_done_fp  = lst_done & (lst_own == OWN_FP);
assign lst_done_alb = lst_done & (lst_own == OWN_ALB);
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all)                       lst_own <= OWN_ALB;
    else if (lst_start)                lst_own <= osd_file_page ? OWN_FP :
                                                  (alb_lst_start ? OWN_ALB : OWN_MM);
end
// 照抄顶层：文件页退出 ⇒ 相册重扫；match 扫描借 name_mem 读口时 OSD 暂缓写字
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) osd_fp_d <= 1'b0;
    else         osd_fp_d <= osd_file_page;
end
assign nm_rescan = osd_fp_d & ~osd_file_page;
// ★ [NM-UPD]/[NM-HOLD]：与顶层同一条规则（ALB 重扫 + MM 扫完都重写；MM 期间写空格）
assign nm_upd    = lst_done & ((lst_own == OWN_ALB) | (lst_own == OWN_MM));
assign nm_hold   = lst_busy & (lst_own == OWN_MM);

`ifdef FP_SUB
// ★ 2026-11-07 字幕 + 音随图切的顶层胶水（match 扫描 / TXT / sub_loader / dyn_subtitle）
`include "tb_file_page_subglue.vh"
`else
assign mm_go = 1'b0;
assign mm_rel = 2'd0;
assign mm_hit = 1'b0;
assign sub_sd_req = 1'b0;
assign sub_sd_addr = 32'd0;
assign sub_fl_req = 1'b0;
assign sub_fl_addr = 24'd0;
assign sub_fl_len = 16'd0;
assign ld_can_load = ld_ready & ~quad_own;
assign fat_req_g = lst_sd_req;
assign txt_req_g = 1'b0;
`endif

assign lst_ld_slot = alb_sel ? alb_rd_slot :
                     op_ph   ? fp_cur_w    :
                     (ql_busy ? ql_rd_slot : lst_rd_slot);
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        op_ph <= 1'b0; jump_r <= 1'b0; jump_idx_r <= 8'd0; trk_set_r <= 1'b0; trk_sec_r <= 23'd0;
    end else begin
        jump_r    <= 1'b0;
        trk_set_r <= 1'b0;
        op_ph     <= fp_open_w & ~op_ph;
        if (op_ph && lst_vld) begin
            if (lst_type == 4'd1) begin jump_r    <= 1'b1; jump_idx_r <= quad_rank;     end
            if (lst_type == 4'd2) begin trk_set_r <= 1'b1; trk_sec_r  <= lst_off[31:9]; end
        end
`ifdef FP_SUB
        // 照抄顶层的音随图切（本台把 osd_asy 视为打开、没有音频 ⇒ trk_done=0）
        if (mm_hit && !mm_same) begin trk_set_r <= 1'b1; trk_sec_r <= lst_match_off; end
`endif
    end
end
assign bmp_start   = quad_own ? bmp_start_q : (ld_req & ~quad_own);
assign bmp_off     = quad_own ? ql_off   : ld_off;
assign bmp_len     = ld_len;
assign bmp_base    = quad_own ? ql_base  : ld_base;
assign bmp_mode    = quad_own ? 2'd2 : (ld_fmt ? 2'd0 : 2'd1);
assign bmp_fmt     = quad_own ? 1'b0 : ld_fmt;

`ifdef KEY_MASK_OLD
// 顶层原来的按键接法（文件页里屏蔽 KEY3/KEY4）
wire key_prev_t = key_press[3] & ~osd_file_page;
wire key_next_t = key_press[2] & ~osd_file_page;
`else
wire key_prev_t = key_press[3];
wire key_next_t = key_press[2];
`endif

album_src u_album (
    .clk(clk), .rst(rst), .I_base(alb_base), .I_need(1'b1), .I_block(osd_file_page), .I_rescan(nm_rescan),
    .O_lst_start(alb_lst_start), .O_lst_offset(), .O_rd_slot(alb_rd_slot),
    .I_lst_busy(lst_busy), .I_lst_done(lst_done_alb), .I_ld_type(lst_type),
    .I_slot_valid(lst_valid), .I_file_off(lst_off),
    .O_off0(alb_off0), .O_off1(alb_off1), .O_off2(alb_off2), .O_off3(alb_off3),
    .O_any(alb_any), .O_nimg(alb_nimg), .O_win4(alb_win4), .O_ready(alb_ready), .O_sel(alb_sel)
);

fat_dir_lister #(.N_ITEMS(4), .MAX_SECS(FAT_MAX_SECS), .INCLUDE_DIR(0)) u_fat (
    .clk(clk), .rst(rst),
    .I_start(lst_start),
    .I_offset(osd_file_page ? quad_off : alb_base),
    .I_list_all_types(osd_file_page),
    .O_busy(lst_busy), .O_done(lst_done), .O_valid_cnt(lst_cnt), .O_has_more(lst_has_more),
    .I_rd_slot(lst_rd_slot), .I_rd_idx(lst_rd_idx), .O_rd_char(lst_rd_char),
    .I_ld_slot(lst_ld_slot), .O_ld_file_off(lst_off), .O_ld_size(lst_size),
    .O_ld_namelen(), .O_ld_blen(lst_blen), .O_ld_elen(lst_elen), .O_ld_len4(lst_len4),
    .O_ld_vld(lst_vld), .O_ld_type(lst_type), .O_slot_valid(lst_valid),
    .I_match_en(mm_go), .I_match_rel(mm_rel),
    .O_match_hit(lst_match_hit), .O_match_off(lst_match_off), .O_match_txt_hit(lst_match_txt_hit),
    .O_sd_req(lst_sd_req), .O_sd_addr(lst_sd_addr),
    .I_sd_data(fat_sd_data), .I_sd_dv(fat_sd_dv), .I_sd_end(fat_sd_end)
);

quad_page_ctrl #(.N_ITEMS(4)) u_qpage (
    .clk(clk), .rst(rst),
    .I_file_page(osd_file_page), .I_flip_next(quad_flip_nx), .I_flip_prev(quad_flip_pv),
    .O_lst_start(qp_lst_start), .O_lst_offset(),
    .I_lst_busy(lst_busy), .I_lst_done(lst_done_fp), .I_lst_cnt(lst_cnt),
    .I_lst_has_more(lst_has_more),
    .O_ld_start(ql_start), .I_ld_busy(ql_busy), .I_ld_done(ql_done), .I_ld_loaded(ql_loaded),
    .I_ld_filled(ql_filled), .I_ld_bmp(ql_bmp), .O_ld_abort(qp_ld_abort), .I_cur(fp_cur_w),
    .O_has_prev(quad_has_prev), .O_rank(quad_rank),
    .O_own(quad_own), .O_ready(quad_ready), .O_has_more(quad_has_more),
    .O_offset(quad_off), .O_gen(fp_gen_w), .O_valid_cnt(quad_vcnt)
);

wire        ph_start, ph_busy, ph_done, ph_wr_vld;
wire [1:0]  ph_kind;
wire [20:0] ph_base, ph_wr_addr;
wire [31:0] ph_wr_data;

quad_loader #(.SLOT_BASE(QUAD_BASE), .SLOT_STEP(40960), .N_ITEMS(4)) u_qload (
    .clk(clk), .rst(rst),
    .I_start(ql_start), .I_abort(qp_ld_abort), .I_hold(op_ph), .O_bmp(ql_bmp), .O_filled(ql_filled),
    .I_valid_cnt(lst_cnt), .I_slot_valid(lst_valid),
    .O_busy(ql_busy), .O_done(ql_done), .O_loaded(ql_loaded), .O_rd_slot(ql_rd_slot),
    .I_file_off(lst_off), .I_file_len(lst_size), .I_file_type(lst_type),
    .O_ph_start(ph_start), .O_ph_kind(ph_kind), .O_ph_base(ph_base),
    .I_ph_busy(ph_busy), .I_ph_done(ph_done),
    .O_ld_start(bmp_start_q), .O_ld_off(ql_off), .O_ld_len(ql_len), .O_ld_base(ql_base),
    .I_ld_ready(ld_ready), .I_ld_busy(ld_busy), .I_ld_done(ld_done), .I_ld_err(ld_err)
);

wire        ld_wr_vld;
wire [9:0]  ld_wr_used_raw;
wire [20:0] ld_wr_addr;
wire [31:0] ld_wr_data;
wire        ld_wr_ack;

// ---- 装载写口：bmp_loader / ph_writer 二选一（照抄顶层）----
wire [4:0]  ld_wr_used_b = (ld_wr_used_raw > 10'd15) ? 5'd15 : ld_wr_used_raw[4:0];
wire        sdr_ld_vld  = ph_busy ? ph_wr_vld  : ld_wr_vld;
wire [20:0] sdr_ld_addr = ph_busy ? ph_wr_addr : ld_wr_addr;
wire        ph_black;
wire [31:0] sdr_ld_data = ph_busy ? (ph_wr_data & {32{~ph_black}}) : ld_wr_data;
`ifdef PH_OLD
// 顶层原来的接法：ph_writer 在 50MHz、used 恒 1
wire [4:0]  sdr_ld_used = ph_busy ? 5'd1 : ld_wr_used_b;
wire        ph_clk = clk;
`else
wire [4:0]  sdr_ld_used = ph_busy ? {1'b0, {4{ph_wr_vld}}} : ld_wr_used_b;
wire        ph_clk = mclk;
`endif

ph_writer u_ph (
    .clk(ph_clk), .rst(rst), .I_start(ph_start), .I_kind(ph_kind), .I_base(ph_base),
    .O_busy(ph_busy), .O_done(ph_done), .O_black(ph_black),
    .O_wr_vld(ph_wr_vld), .O_wr_addr(ph_wr_addr), .O_wr_data(ph_wr_data),
    .I_wr_ack(ld_wr_ack)
);

// ---- sdr_port_sched 装载写口的模型（mclk 域）----
//   与真调度器同一条取数规则：used >= 4 才取，一次连弹 4 拍 ack，每拍收一个队首字；
//   取完空 4 拍（发组）才允许下一次取。
reg [3:0] ldm_ph = 0;       // 0 = 空闲；1..4 = 取数第几拍；5..8 = 发组
assign ld_wr_ack = (ldm_ph >= 4'd1) && (ldm_ph <= 4'd4);
always @(posedge mclk) begin
    if (ldm_ph == 0) begin
        if (sdr_ld_used >= 5'd4) ldm_ph <= 4'd1;
    end else if (ldm_ph == 4'd8) ldm_ph <= 4'd0;
    else ldm_ph <= ldm_ph + 4'd1;
    if (ld_wr_ack) sdram[sdr_ld_addr] <= sdr_ld_data;
end

bmp_loader #(.AFIFO_AW(9)) u_loader (
    .clk(clk), .rst(rst), .mclk(mclk),
    .start(bmp_start), .abort(quad_own ? qp_ld_abort : app_ld_abort), .file_offset(bmp_off), .file_length(bmp_len), .dst_base(bmp_base),
    .mode(bmp_mode), .fmt(bmp_fmt), .dst_stride(quad_own ? 11'd160 : 11'd0),
    .zoom(quad_own ? ld_zoom_w : ld_snap_zoom), .ax(quad_own ? ld_ax_w : ld_snap_ax),
    .ay(quad_own ? ld_ay_w : ld_snap_ay),
    .ready(ld_ready), .busy(ld_busy), .done(ld_done), .err(ld_err),
    .src_w_o(ld_src_w), .src_h_o(ld_src_h),
    .wr_vld(ld_wr_vld), .wr_used(ld_wr_used_raw), .wr_addr(ld_wr_addr), .wr_data(ld_wr_data),
    .wr_ack(ld_wr_ack), .wr_settled(1'b1),
    .sd_sec_read(ld_sd_req), .sd_sec_read_addr(ld_sd_addr),
    .sd_sec_read_data(ld_sd_data), .sd_sec_read_data_valid(ld_sd_dv),
    .sd_sec_read_end(ld_sd_end), .sd_init_done(sd_init_done_w)
);

//==============================================================================
//  App
//==============================================================================
wire [23:0] ps_data;
wire        ps_valid, app_pipe_ce, app_busy;
wire [1:0]  tex_req_vld;
wire [41:0] tex_req_addr;
wire [7:0]  tex_req_mask;
reg  [1:0]  tex_req_ack = 2'b00;
reg  [1:0]  tex_pix_wr  = 2'b00;
reg  [63:0] tex_pix_data = 64'd0;
wire        f_cs, f_sclk, f_mosi, f_miso;
wire        fb_fmt565;
wire [3:0]  dbg_state;

App #(
    .IMG_W(640), .IMG_H(480),
    .THUMB0_ADDR(THUMB0_ADDR), .THUMB1_ADDR(THUMB1_ADDR),
    .THUMB2_ADDR(THUMB2_ADDR), .HD_ADDR(HD_ADDR),
    .ARM_FRAMES(0), .T_END(8), .T_END_F(8),
    .SCENE_QUOTA(127), .QUOTA_565(127),
    .FRAME_PERIOD_CYCLES(330000)
) u_app (
    .clk(clk), .rst(rst),
    .I_quad_mode(quad_mode_r), .O_page(), .O_file_page(osd_fp_w),
`ifndef PH_OLD
    .I_quad_vm(quad_vm_r),
`endif
    .O_nm_slot(lst_rd_slot), .O_nm_idx(lst_rd_idx), .I_nm_char(lst_rd_char),
    .I_nm_blen(lst_blen), .I_nm_elen(lst_elen), .I_nm_len4(lst_len4), .I_nm_vld(lst_vld),
    .I_img_off0(alb_off0), .I_img_off1(alb_off1), .I_img_off2(alb_off2), .I_img_off3(alb_off3),
    .O_alb_base(alb_base), .I_alb_win4(alb_win4), .I_alb_any(alb_any), .I_alb_nimg(alb_nimg),
    .I_alb_ready(alb_ready), .I_nm_upd(nm_upd), .I_nm_hold(nm_hold),
    .O_fp_cur(fp_cur_w), .O_fp_flip_nx(quad_flip_nx), .O_fp_flip_pv(quad_flip_pv),
    .I_fp_vcnt(quad_vcnt), .I_fp_has_more(quad_has_more), .I_fp_gen(fp_gen_w),
    .I_fp_has_prev(quad_has_prev), .O_fp_open(fp_open_w),
    .I_jump(jump_r), .I_jump_idx(jump_idx_r), .O_ld_abort(app_ld_abort),
    .mclk(mclk),
    .ps_start(ps_start), .ps_new_frame(ps_new_frame), .ps_data(ps_data), .ps_valid(ps_valid),
    .pipe_ce(app_pipe_ce), .busy(app_busy),
    .tex_req_vld(tex_req_vld), .tex_req_addr(tex_req_addr), .tex_req_mask(tex_req_mask),
    .tex_req_ack(tex_req_ack), .tex_pix_wr(tex_pix_wr), .tex_pix_data(tex_pix_data),
    .key_prev(key_prev_t), .key_next(key_next_t), .key_fx(key_press[1]), .key_menu(key_press[0]),
    .key_level(key_level), .bright_eff(7'd50),
    .flash_cs_n(f_cs), .flash_sclk(f_sclk), .flash_mosi(f_mosi), .flash_wp_n(), .flash_hold_n(),
    .flash_miso(f_miso),
    .ld_req(ld_req), .ld_fmt(ld_fmt), .ld_off(ld_off), .ld_len(ld_len), .ld_base(ld_base),
    .ld_can_load(ld_can_load), .ld_done(ld_done), .ld_err(ld_err), .ld_sd_ready(sd_init_done_w),
    .ld_src_w(ld_src_w), .ld_src_h(ld_src_h), .music_done(1'b0),
    .bp_req(1'b0), .bp_addr(24'd0), .bp_len(16'd0),
    .bp_rdy(), .bp_data(), .bp_valid(), .bp_done(),    .fmt565(fb_fmt565), .wr_stall(1'b0),
    .st_img(), .st_fx(), .fps_dbg(), .dbg_state(dbg_state), .dbg_osd(dbg_osd), .dbg_fmt(),
    .pic_bright(), .pic_contrast(), .pic_temp(), .pic_sat(), .pic_style(), .pic_tgl(),
    .o_lang(), .o_vol(), .o_mute(), .o_loop(), .o_spc(), .o_asy(osd_asy),
    .O_hd_rel(hd_rel_w),
    .ld_zoom(ld_snap_zoom), .ld_ax(ld_snap_ax), .ld_ay(ld_snap_ay),
    .sub_fl_req(sub_fl_req), .sub_fl_addr(sub_fl_addr), .sub_fl_len(sub_fl_len),
    .sub_fl_rdy(sub_fl_rdy), .sub_fl_data(sub_fl_data), .sub_fl_valid(sub_fl_valid),
    .sub_fl_done(sub_fl_done),
    .oz_on(oz_on_w), .oz_zoom(ld_zoom_w), .oz_ax(ld_ax_w), .oz_ay(ld_ay_w)
);

w25q64_model_24b #(.MEM_BYTES(262144), .INIT_HEX("OSD/tools/font.hex")) u_flash (
    .cs_n(f_cs), .sclk(f_sclk), .mosi(f_mosi), .miso(f_miso)
);

//------------------------------------------------------------------------------
//  纹理口：按 sdr_port_sched 的契约（4 字组 + 掩码，组内升序只回掩码里的字）
//  两层轮流服务，固定小延迟。
//------------------------------------------------------------------------------
integer tst [0:1];
reg [20:0] ta  [0:1];
reg [3:0]  tm  [0:1];
reg [2:0]  tk  [0:1];
integer    tg  [0:1];
reg        tturn = 0;
integer li;
initial begin tst[0] = 0; tst[1] = 0; tg[0] = 0; tg[1] = 0; end
always @(posedge mclk) begin
    tex_req_ack <= 2'b00;
    tex_pix_wr  <= 2'b00;
    for (li = 0; li < 2; li = li + 1) begin
        case (tst[li])
        0: if (tex_req_vld[li] && (tst[1-li] == 0) && (tturn == li || !tex_req_vld[1-li])) begin
               ta[li]  = (li == 0) ? {tex_req_addr[20:2], 2'b00} : {tex_req_addr[41:23], 2'b00};
               tm[li]  = (li == 0) ? tex_req_mask[3:0] : tex_req_mask[7:4];
               tg[li]  = 6;
               tst[li] = 1;
           end
        1: if (tg[li] != 0) tg[li] = tg[li] - 1;
           else begin tex_req_ack[li] <= 1'b1; tk[li] = 0; tst[li] = 2; end
        2: begin
               if (tm[li][tk[li][1:0]]) begin
                   tex_pix_wr[li] <= 1'b1;
                   if (li == 0) tex_pix_data[31:0]  <= sdram[ta[li] + tk[li]];
                   else         tex_pix_data[63:32] <= sdram[ta[li] + tk[li]];
               end
               if (tk[li] == 3) begin tst[li] = 0; tturn <= ~li[0]; end
               else tk[li] = tk[li] + 1;
           end
        endcase
    end
end

//------------------------------------------------------------------------------
//  frame_writer + 写引擎模型 + 抓帧
//------------------------------------------------------------------------------
wire        app_write_req, app_write_en, display_valid, fw_commit_tgl;
wire [1:0]  write_buf_idx, disp_buf_idx;
wire [31:0] app_write_data;
wire [15:0] fw_ox, fw_oy;
reg         app_write_req_ack = 0;
reg         write_finish_pulse = 0;
reg         disp_adopt_sync = 0;

frame_writer #(.IMG_W(640), .IMG_H(480), .NPX(307200), .PIPE(4)) u_fw (
    .clk(clk), .rst(rst),
    .ps_start(ps_start), .ps_new_frame(ps_new_frame),
    .ps_data(ps_data), .ps_valid(ps_valid), .pipe_ce(app_pipe_ce),
    .write_req(app_write_req), .write_buf_idx(write_buf_idx), .write_len(),
    .write_req_ack(app_write_req_ack),
    .write_en(app_write_en), .write_data(app_write_data),
    .write_finish_pulse(write_finish_pulse),
    .disp_buf_idx(disp_buf_idx), .display_valid(display_valid),
    .disp_commit_tgl(fw_commit_tgl), .disp_adopt(disp_adopt_sync),
    .busy(app_busy), .done(), .px_x(), .px_y(), .out_x(fw_ox), .out_y(fw_oy)
);

always @(posedge clk) disp_adopt_sync <= fw_commit_tgl;

reg [19:0] wcnt = 0;
reg        wr_fmt_l = 0, pk_ph = 0, fin_armed = 0;
reg [5:0]  fdelay = 0;
wire [19:0] words_target = wr_fmt_l ? 20'd153600 : 20'd307200;
integer frames = 0;
always @(posedge clk) begin
    write_finish_pulse <= 1'b0;
    if (app_write_req && !app_write_req_ack) begin
        app_write_req_ack <= 1'b1;
        wcnt <= 0; fdelay <= 0; pk_ph <= 0; fin_armed <= 0;
        wr_fmt_l <= fb_fmt565;
    end else if (app_write_req_ack && !app_write_req)
        app_write_req_ack <= 1'b0;
    if (app_write_en) begin
        if (wr_fmt_l) begin if (pk_ph) wcnt <= wcnt + 20'd1; pk_ph <= ~pk_ph; end
        else wcnt <= wcnt + 20'd1;
    end
    if (wcnt >= words_target) fin_armed <= 1'b1;
    if (fin_armed) begin
        if (fdelay == 6'd20) begin
            write_finish_pulse <= 1'b1; frames = frames + 1;
            fdelay <= 6'd21; fin_armed <= 1'b0; wcnt <= 0;
        end else if (fdelay < 6'd20) fdelay <= fdelay + 6'd1;
    end
end

// 抓帧：最近一帧的 RGB（frame_writer 的 out_x/out_y 与 write_data 同拍）
reg [23:0] fb [0:307199];
always @(posedge clk) if (app_write_en && fw_ox < 640 && fw_oy < 480)
    fb[fw_oy*640 + fw_ox] <= app_write_data[31:8];

task dump_ppm;
    input [8*64-1:0] fname;
    integer fd, p;
    begin
        fd = $fopen(fname, "wb");
        $fwrite(fd, "P6\n640 480\n255\n");
        for (p = 0; p < 307200; p = p + 1)
            $fwrite(fd, "%c%c%c", fb[p][23:16], fb[p][15:8], fb[p][7:0]);
        $fclose(fd);
        $display("  [ppm] %0s", fname);
    end
endtask

//==============================================================================
//  观测点
//==============================================================================
`define MENU  u_app.u_osd.u_menu
wire       m_focus = `MENU.focus;
wire [2:0] m_nav   = `MENU.nav_idx;
wire [2:0] m_page  = `MENU.page;
wire [2:0] m_item  = `MENU.item;

// 文件名跟踪：每次 L_TXT 写 0..3 行就拼字符串；每次 L_TXY 结束记录坐标
reg [8*16-1:0] nm_str [0:3];
reg [7:0]      nm_len [0:3];
integer        lay_cnt = 0;        // 完成了几次布局（L_FIN）
reg [21:0]     xy_hist [0:3];
integer        xy_jit = 0;         // 文件页里同一页 x/y 发生变化的次数
reg            xy_arm = 0;
reg [15:0]     fp_gen_seen;
integer        nl;
always @(posedge clk) begin
    if (`MENU.str_we && `MENU.fp_mode && `MENU.str_line < 4) begin
        nm_str[`MENU.str_line] = (`MENU.str_idx == 8'd0) ? {120'd0, `MENU.str_code[7:0]}
                               : {nm_str[`MENU.str_line][8*15-1:0], `MENU.str_code[7:0]};
    end
    if (`MENU.ctl_we && `MENU.fp_mode && `MENU.ctl_line < 4)
        nm_len[`MENU.ctl_line] = `MENU.ctl_len;
    if (`MENU.lay_st == 3'd5) lay_cnt = lay_cnt + 1;
end
task clr_names;
    begin for (nl = 0; nl < 4; nl = nl + 1) nm_str[nl] = 0; end
endtask

// x/y 抖动：文件页里、没有发生翻页/按键时，每次布局的 txt_xy[0..3] 必须一样
always @(posedge clk) begin
    if (`MENU.lay_st == 3'd5 && `MENU.fp_mode) begin
        if (xy_arm) begin
            for (nl = 0; nl < 4; nl = nl + 1)
                if (`MENU.txt_xy[nl*22 +: 22] !== xy_hist[nl]) xy_jit = xy_jit + 1;
        end
        for (nl = 0; nl < 4; nl = nl + 1) xy_hist[nl] = `MENU.txt_xy[nl*22 +: 22];
        xy_arm = 1;
    end
    if (!`MENU.fp_mode) xy_arm = 0;
end

// 事件探针：文件页链路的每一次握手 + txt_xy[0] 变化
reg [1:0] qp_st_d = 0;
reg [21:0] xy0_d = 0;
always @(posedge clk) if (!rst) begin
    if (u_qpage.st !== qp_st_d) begin
        $display("  [qp] t=%0t st %0d->%0d own=%b fp=%b lbusy=%b qlbusy=%b",
                 $time, qp_st_d, u_qpage.st, quad_own, osd_file_page, lst_busy, ql_busy);
        qp_st_d = u_qpage.st;
    end
    if (lst_start) $display("  [lst] t=%0t start fp=%b off=%0d all=%b busy=%b",
                            $time, osd_file_page, u_fat.I_offset, osd_file_page, lst_busy);
    if (lst_done)  $display("  [lst] t=%0t done own_fp=%b cnt=%0d more=%b", $time, lst_own_fp, lst_cnt, lst_has_more);
    if (qp_lst_start) $display("  [qp] t=%0t qp_lst_start", $time);
    if (ql_start)  $display("  [ql] t=%0t start", $time);
    if (ql_done)   $display("  [ql] t=%0t done loaded=%0d", $time, ql_loaded);
    if (bmp_start) $display("  [bmp] t=%0t start own=%b off=%0d base=%0d", $time, quad_own, bmp_off, bmp_base);
    if (ld_done)   $display("  [bmp] t=%0t done err=%b", $time, ld_err);
    if (`MENU.fp_mode && `MENU.txt_xy[21:0] !== xy0_d) begin
        $display("  [xy0] t=%0t x0=%0d len4=%h lst_ld_slot=%0d st=%0d", $time,
                 $signed(`MENU.txt_xy[10:0]), lst_len4, lst_ld_slot, `MENU.lay_st);
        xy0_d = `MENU.txt_xy[21:0];
    end
end

`ifdef FP_HB
// 心跳：每 1M 拍打印一次帧/布局计数与 A/B 层就绪（查"帧变慢/卡住"用）
integer hb_n = 0, hb_rq0 = 0, hb_rq1 = 0;
always @(posedge clk) begin
    if (tex_req_ack[0]) hb_rq0 = hb_rq0 + 1;
    if (tex_req_ack[1]) hb_rq1 = hb_rq1 + 1;
    hb_n = hb_n + 1;
    if (hb_n == 1_000_000) begin
        hb_n = 0;
        $display("  [hb] t=%0t fr=%0d lay=%0d qm=%b ps=%b rdyA=%b rdyB=%b boxA=%b boxB=%b deg=%b wd=%b ack0=%0d ack1=%0d",
                 $time, frames, lay_cnt, quad_mode_r, ps_start,
                 u_app.u_scene.rdy_a, u_app.u_scene.rdy_b,
                 u_app.u_scene.in_box_a, u_app.u_scene.in_box_b,
                 u_app.u_scene.frame_degraded, u_app.u_scene.wd_force, hb_rq0, hb_rq1);
        hb_rq0 = 0; hb_rq1 = 0;
    end
end
`endif

//==============================================================================
//  驱动
//==============================================================================
task press;
    input integer k;
    begin
        @(negedge clk); key_press = 4'd1 << k;
        @(negedge clk); key_press = 4'd0;
    end
endtask
task wait_cyc;
    input integer n;
    integer i;
    begin for (i = 0; i < n; i = i + 1) @(posedge clk); end
endtask
task wait_frames;           // 等 n 次布局（≈ n 帧）
    input integer n;
    integer t0;
    begin
        t0 = lay_cnt;
        while (lay_cnt < t0 + n) @(posedge clk);
    end
endtask
task key_and_settle;        // 按键后等 3 次布局，让动画/文字收敛
    input integer k;
    begin press(k); wait_frames(3); end
endtask

// 四格中心像素：quad 打开时每格应是本页那张图的颜色（经 OSD 遮罩压暗，比主色）
function [1:0] dom;          // 0=R 1=G 2=B 3=混（R≈G）
    input [23:0] c;
    begin
        if (c[23:16] > c[15:8] + 8'd20 && c[23:16] > c[7:0] + 8'd20) dom = 2'd0;
        else if (c[15:8] > c[23:16] + 8'd20 && c[15:8] > c[7:0] + 8'd20) dom = 2'd1;
        else if (c[7:0] > c[23:16] + 8'd20 && c[7:0] > c[15:8] + 8'd20) dom = 2'd2;
        else dom = 2'd3;
    end
endfunction
function [23:0] cell_px;
    input integer s;
    integer x, y;
    begin
        x = (s[0] ? 384 : 192) + 130;      // 避开格子中央的大号数字和左上角标记
        y = (s[1] ? 260 : 100) + 100;
        cell_px = fb[y*640 + x];
    end
endfunction

task wait_frame_done;       // 等一帧真正写完（抓到的 fb 是完整帧）
    integer f0;
    begin f0 = frames; while (frames < f0 + 2) @(posedge clk); end
endtask

task wait_state;
    input integer s;
    input integer tmo;
    integer g;
    begin
        g = 0;
        while (dbg_state !== s && g < tmo) begin @(posedge clk); g = g + 1; end
        if (dbg_state !== s) $display("  [warn] state %0d not reached (now %0d)", s, dbg_state);
    end
endtask

task wait_quad_ready;
    integer g;
    begin
        g = 0;
        while (!(quad_ready && !ql_busy && !lst_busy) && g < 400_000_000) begin @(posedge clk); g = g + 1; end
    end
endtask

task show_names;
    begin
        for (nl = 0; nl < 4; nl = nl + 1)
            $display("    name[%0d] len=%0d \"%0s\"  x0=%0d y0=%0d", nl, nm_len[nl], nm_str[nl],
                     $signed(xy_hist[nl][10:0]), $signed(xy_hist[nl][21:11]));
    end
endtask

task chk_names;
    input integer first;        // 第几张开始：1 → IMG01..IMG04
    reg [8*16-1:0] e;
    reg [7:0] dg1, dg0;
    integer n, ex;
    begin
        for (n = 0; n < 4; n = n + 1) begin
            dg1 = 8'h30 + ((first + n) / 10);
            dg0 = 8'h30 + ((first + n) % 10);
            e = {"IMG", dg1, dg0, ".BMP"};
            if (nm_str[n] !== e) begin
                errors = errors + 1;
                $display("  [FAIL] name[%0d] = \"%0s\" exp \"%0s\"", n, nm_str[n], e);
            end else $display("  [ ok ] name[%0d] = \"%0s\"", n, nm_str[n]);
            chk("name len", nm_len[n], 9);
            ex = (n[0] ? 464 : 272) - 9*8 + 2;
            chk("name x0 centred", xy_hist[n][10:0], ex);
            chk("name y0", xy_hist[n][21:11], n[1] ? 388 : 228);
        end
    end
endtask

integer s, d0, d1, d2, d3, bgd;
`ifdef FP_HOME
`include "tb_file_page_home.vh"
`elsif FP_CARD4
`include "tb_file_page_card4.vh"
`elsif FP_SUB
`include "tb_file_page_sub.vh"
`else
initial begin
    clr_names();
    rst = 1; wait_cyc(20); rst = 0;

    $display("--- boot ---");
    wait_state(5, 60_000_000);
    $display("  booted t=%0t frames=%0d state=%0d alb_nimg=%0d", $time, frames, dbg_state, alb_nimg);
    wait_cyc(200000);

    $display("--- open OSD ---");
    press(0);
    wait_state(8, 5_000_000);
    wait_frames(4);
    chk("osd state", dbg_state, 8);
    chk("nav 0", m_nav, 0);

    //--------------------------------------------------------------------------
    $display("--- F1: nav to File (KEY3 x2) ---");
    key_and_settle(2);
    key_and_settle(2);
    chk("F1 nav=2", m_nav, 2);
    chk("F1 focus stays NAV", m_focus, 0);
    chk("F1 page preview=2", m_page, 2);
    wait_frames(14);
    chk("F1 cursor x = nav column", `MENU.cx_c[9:0], 28);
    chk("F1 cursor w = nav icon", `MENU.cw_c[9:0], 40);
    // KEY3 must still move the nav cursor away from File, KEY4 back
    key_and_settle(2);
    chk("F1 KEY3 leaves File", m_nav, 3);
    key_and_settle(3);
    chk("F1 KEY4 back to File", m_nav, 2);
    chk("F1 still NAV", m_focus, 0);

    //--------------------------------------------------------------------------
    $display("--- F2: KEY2 enters file page, thumbnails load ---");
    key_and_settle(1);
    chk("F2 focus PAGE", m_focus, 1);
    chk("F2 item 0", m_item, 0);
    wait_quad_ready();
    chk("F2 quad_ready", quad_ready, 1);
    chk("F2 loaded 4", ql_loaded, 4);
    chk("F2 vcnt 4", quad_vcnt, 4);
    wait_frames(20);
    chk("F2 quad_mode on", quad_mode_r, 1);
    wait_frame_done();
    dump_ppm("out/fp_page1.ppm");
    d0 = dom(cell_px(0)); d1 = dom(cell_px(1)); d2 = dom(cell_px(2)); d3 = dom(cell_px(3));
    $display("  cells: %06h %06h %06h %06h", cell_px(0), cell_px(1), cell_px(2), cell_px(3));
    chk("F2 cell0 IMG01 red",   d0, 0);
    chk("F2 cell1 IMG02 green", d1, 1);
    chk("F2 cell2 IMG03 blue",  d2, 2);
    chk("F2 cell3 IMG04 yellow", d3, 3);

    //--------------------------------------------------------------------------
    $display("--- F3: file names ---");
    xy_jit = 0;
    wait_frames(30);
    show_names();
    chk_names(1);
    chk("F3 no x/y jitter while idle", xy_jit, 0);

    //--------------------------------------------------------------------------
    $display("--- F4: KEY3/KEY4 move the selection, page at the edges ---");
    key_and_settle(2);  chk("F4 KEY3 -> 1", m_item, 1);
    wait_frames(14);
    chk("F4 cursor on cell1 x", `MENU.cx_c[9:0], 384);
    chk("F4 cursor on cell1 y", `MENU.cy_c[9:0], 100);
    key_and_settle(2);  chk("F4 KEY3 -> 2", m_item, 2);
    key_and_settle(2);  chk("F4 KEY3 -> 3", m_item, 3);
    key_and_settle(3);  chk("F4 KEY4 -> 2", m_item, 2);
    key_and_settle(2);  chk("F4 KEY3 -> 3", m_item, 3);
    clr_names();
    key_and_settle(2);  chk("F4 KEY3 at edge flips -> item 0", m_item, 0);
    wait_quad_ready();
    chk("F4 page 2 offset", quad_off, 4);
    wait_frames(30);
    xy_jit = 0;
    wait_frames(10);
    show_names();
    chk_names(5);
    chk("F4 no jitter page 2", xy_jit, 0);
    wait_frame_done();
    dump_ppm("out/fp_page2.ppm");
    d0 = dom(cell_px(0));
    chk("F4 page2 cell0 IMG05 magenta(mixed)", d0, 3);
    d1 = dom(cell_px(1));
    chk("F4 page2 cell1 IMG06 cyan(mixed)", d1, 3);
    clr_names();
    key_and_settle(3);  chk("F4 KEY4 at first cell flips back -> item 3", m_item, 3);
    wait_quad_ready();
    chk("F4 back to offset 0", quad_off, 0);
    wait_frames(30);
    show_names();
    chk_names(1);

    //--------------------------------------------------------------------------
    $display("--- F5: KEY1 -> nav, KEY1 -> close ---");
    key_and_settle(0);
    chk("F5 KEY1 back to NAV", m_focus, 0);
    chk("F5 OSD still open", dbg_state, 8);
    wait_frames(14);
    chk("F5 cursor back on nav x", `MENU.cx_c[9:0], 28);
    press(0);
    wait_state(5, 40_000_000);
    chk("F5 OSD closed, back to S_HD", dbg_state, 5);
    chk("F5 loader returned to album", quad_own, 0);

    $display("");
    if (errors == 0) $display("ALL PASS");
    else             $display("FAIL: %0d errors", errors);
    $finish;
end
`endif

`ifndef FP_CARD4
`ifndef FP_HOME
`ifndef FP_SUB
`define FP_GTO
`endif
`endif
`endif
`ifdef FP_GTO
initial begin
    #(64'd4_000_000_000);
    $display("FAIL: global timeout");
    $finish;
end
`endif

endmodule
