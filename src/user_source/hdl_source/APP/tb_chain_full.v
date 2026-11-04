//==============================================================================
//  tb_chain_full.v —— 从 FAT32 卡镜像到 640×480 整帧的全链路仿真
//==============================================================================
//
//  链路（模块全部是真实 RTL，接线照抄 top_tf_hdmi_audio.v）：
//
//      sd_sector_file_model            ← sim_data/fat32_10img.img（真 FAT32）
//            │
//      sd_bus_arbiter4 ──┬── fat_dir_lister ──► album_src ──► viewer_ctrl
//                        └── bmp_loader ─────► SDRAM 模型
//                                                  ▲
//      App(viewer_ctrl + viewer_scene + osd_top) ──┘
//            │ ps_data/ps_valid
//            ▼
//      frame_writer 模型 ──► fbuf ──► out/chain_*.ppm
//
//  【和实机唯一的差别】
//    · SDRAM 用平坦数组 + 固定 0 延迟（不模拟 bank/刷新/换向）；
//    · 帧写口不落 SDRAM，直接进 fbuf（显示链路只关心 ps_data 序列）。
//    这两处只影响带宽/时序，不影响功能；带宽类问题由 tb_quad_page / 上板覆盖。
//
//  【跑法】（工作目录 = APP/）
//      iverilog -g2012 -DIVERILOG_SIM -o /tmp/ch.vvp -s tb_chain_full \
//          tb_chain_full.v App.v viewer_ctrl.v viewer_scene.v viewer_fx.v \
//          app_scan_gen.v img_layer_ctrl.v img_layer_hd.v ps_gen_image.v img_afifo.v \
//          ps_merge_alpha8.v ps_to_rgb24.v ps_gen_text.v osd_font_cache.v gb_addr.v \
//          ps_merge_alpha_pp.v \
//          OSD/osd_top.v OSD/osd_menu.v OSD/osd_gfx.v OSD/osd_cellmem.v OSD/osd_widget.v \
//          OSD/flash_arb.v OSD/osd_icon_rom.v OSD/tools/bram32k_stub.v OSD/tools/mult18_stub.v \
//          w25q64_model_24b.v ../spi_flash_read.v \
//          album_src.v quad_loader.v quad_page_ctrl.v \
//          ../SD/fat_dir_lister.v ../SD/bmp_loader.v ../SD/bmp_scale.v ../SD/bmp_read.v \
//          ../SD/bmp_view.v ../SD/bmp_view_fill.v ../SD/ph_writer.v \
//          ../SD/sd_bus_arbiter4.v ../SD/sd_sector_file_model.v
//      vvp /tmp/ch.vvp +SCEN=all
//==============================================================================
`timescale 1ns/1ps

module tb_chain_full;

localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam integer NPIX  = IMG_W * IMG_H;

localparam [20:0] THUMB0 = 21'd614400;
localparam [20:0] THUMB1 = 21'd652800;
localparam [20:0] THUMB2 = 21'd691200;
localparam [20:0] HD_A   = 21'd729600;
localparam [20:0] QBASE  = 21'd1048576;

// 仿真加速：真值 18000（360µs），复现阶段用 2000（40µs）
parameter integer LAT_CYC = 40;
parameter integer FRAME_CYC = 3000;
parameter integer T_END = 8;
parameter integer T_END_F = 4;
parameter integer ARM_FRAMES = 1;

reg clk = 0, mclk = 0, rst = 1;
always #10 clk  = ~clk;      // 50MHz
always #5  mclk = ~mclk;     // 100MHz

integer cyc = 0;
always @(posedge clk) cyc <= cyc + 1;

// ---- 先声明（iverilog 不允许先用后声明）----
wire        ld_req, ld_fmt, ld_ready, ld_busy, ld_done, ld_err, ld_can_load;
wire        ld_sd_ready;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
wire [15:0] ld_src_w, ld_src_h;
wire        ql_start;
wire        ld_wr_vld, ld_wr_ack, ld_wr_settled;
wire [9:0]  ld_wr_used;
wire [20:0] ld_wr_addr;
wire [31:0] ld_wr_data;
wire        fp_flip_nx, fp_flip_pv;
wire [1:0]  fp_cur_w;
reg  [8*24-1:0] tmp_tag;

//------------------------------------------------------------------------------
//  卡 + 仲裁器
//------------------------------------------------------------------------------
wire        sd_init_done;
wire        sd_sec_read, sd_sec_dv, sd_sec_end;
wire [31:0] sd_sec_addr;
wire [7:0]  sd_sec_data;
wire        rd_pulse;
wire [31:0] rd_lba;

sd_sector_file_model #(
    .INIT_FILE ("../../../../sim_data/fat32_10img.img"),
    .LAT_CYC   (LAT_CYC),
    .MAX_LBA   (32'd131071)
) u_card (
    .clk(clk), .rst(rst),
    .O_init_done(sd_init_done),
    .I_sec_read(sd_sec_read), .I_sec_addr(sd_sec_addr),
    .O_sec_data(sd_sec_data), .O_sec_dv(sd_sec_dv), .O_sec_end(sd_sec_end),
    .O_rd_pulse(rd_pulse), .O_rd_lba(rd_lba)
);

// ---- 客户口 ----
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
//  FAT 目录 / 相册 / 四宫格
//------------------------------------------------------------------------------
wire        lst_start, lst_busy, lst_done, lst_has_more;
wire [2:0]  lst_cnt;
wire [1:0]  lst_rd_slot;
wire [3:0]  lst_rd_idx;
wire [7:0]  lst_rd_char;
wire [1:0]  lst_ld_slot;
wire [31:0] lst_off, lst_size;
wire [3:0]  lst_blen, lst_elen, lst_type, lst_valid;
wire [19:0] lst_len4;
wire        lst_vld;
wire [31:0] lst_root, lst_fat;
wire [3:0]  lst_spc;
wire        lst_bpb_ok;

wire        alb_lst_start, alb_busy, alb_done, alb_sel, alb_win4;
wire [1:0]  alb_rd_slot;
wire [31:0] alb_off0, alb_off1, alb_off2;
wire [1:0]  alb_have, alb_win;
wire        alb_ready;
wire [15:0] alb_base;

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

wire        osd_fp_w;
wire [2:0]  osd_page_w;

// ★ 复现 S1：album 的偏移没接进 lister（当前 top 就是这样）
//   修完 D1 之后把这一行改成 `wire [15:0] lst_offset = osd_fp_w ? quad_off : alb_base;`
wire [15:0] lst_offset = osd_fp_w ? quad_off : alb_base;   // ★ D1 修复后

reg    lst_own_fp;
assign lst_start   = osd_fp_w ? qp_lst_start : alb_lst_start;
wire   lst_done_fp = lst_done &  lst_own_fp;
wire   lst_done_al = lst_done & ~lst_own_fp;

always @(posedge clk or posedge rst) begin
    if (rst)            lst_own_fp <= 1'b0;
    else if (lst_start) lst_own_fp <= osd_fp_w;
end

album_src u_album (
    .clk(clk), .rst(rst),
    .I_base(alb_base), .I_need(1'b1), .I_block(osd_fp_w), .I_rescan(1'b0),
    .O_lst_start(alb_lst_start), .O_lst_offset(), .O_rd_slot(alb_rd_slot),
    .I_lst_busy(lst_busy), .I_lst_done(lst_done_al),
    .I_ld_type(lst_type), .I_slot_valid(lst_valid), .I_file_off(lst_off),
    .O_off0(alb_off0), .O_off1(alb_off1), .O_off2(alb_off2),
    .O_have(alb_have), .O_ready(alb_ready), .O_win(alb_win), .O_win4(alb_win4),
    .O_sel(alb_sel)
);

fat_dir_lister #(
    .N_ITEMS(4), .MAX_SECS(64), .INCLUDE_DIR(0)
) u_fat (
    .clk(clk), .rst(rst),
    .I_start(lst_start), .I_offset(lst_offset),
    // 与顶层一致：文件页扫描列全部类型，相册扫描只数 BMP
    .I_list_all_types(osd_fp_w),
    // ★ 2026-11-02：match 端口显式接 0（不接会 z→x）
    .I_match_en(1'b0), .I_match_rel(2'd0), .O_match_hit(), .O_match_off(),
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

quad_page_ctrl #(.N_ITEMS(4), .STUCK_CYC(200000)) u_qpage (
    .clk(clk), .rst(rst),
    .I_file_page(osd_fp_w),
    .I_flip_next(fp_flip_nx), .I_flip_prev(fp_flip_pv),
    .O_lst_start(qp_lst_start), .O_lst_offset(qp_lst_offset),
    .I_lst_busy(lst_busy), .I_lst_done(lst_done_fp), .I_lst_cnt(lst_cnt),
    .I_lst_has_more(lst_has_more),
    .O_ld_start(qp_ld_start), .I_ld_busy(ld_busy), .I_ld_done(ld_done),
    .I_ld_loaded(ql_loaded),
    .O_own(quad_own), .O_ready(quad_ready), .O_has_more(quad_has_more),
    .O_offset(quad_off), .O_gen(fp_gen_w), .O_valid_cnt(quad_vcnt)
);

quad_loader #(.SLOT_BASE(QBASE), .SLOT_STEP(40960), .N_ITEMS(4)) u_qload (
    .clk(clk), .rst(rst),
    .I_start(qp_ld_start), .I_valid_cnt(lst_cnt), .I_slot_valid(lst_valid),
    .O_busy(ql_busy), .O_done(ql_done), .O_loaded(ql_loaded),
    .O_rd_slot(ql_rd_slot),
    .I_file_off(lst_off), .I_file_len(lst_size), .I_file_type(lst_type),
    .O_ph_start(ph_start), .O_ph_kind(ph_kind), .O_ph_base(ph_base),
    .I_ph_busy(ph_busy), .I_ph_done(ph_done),
    .O_ld_start(ql_start), .O_ld_off(ql_off), .O_ld_len(ql_len), .O_ld_base(ql_base),
    .I_ld_ready(ld_ready), .I_ld_busy(ld_busy), .I_ld_done(ld_done), .I_ld_err(ld_err)
);

// 元数据口分时（照抄 top）
assign lst_ld_slot = alb_sel ? alb_rd_slot : (ql_busy ? ql_rd_slot : lst_rd_slot);

//------------------------------------------------------------------------------
//  装载器 + 占位图 + SDRAM
//------------------------------------------------------------------------------
wire        bmp_start = quad_own ? ql_start : ld_req;
wire [31:0] bmp_off   = quad_own ? ql_off   : ld_off;
wire [31:0] bmp_len   = ld_len;
wire [20:0] bmp_base  = quad_own ? ql_base  : ld_base;
wire [1:0]  bmp_mode  = quad_own ? 2'd1 : (ld_fmt ? 2'd0 : 2'd1);
wire        bmp_fmt   = quad_own ? 1'b0 : ld_fmt;

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

// 占位图写口二选一（照抄 top）
wire        sdr_ld_vld  = ph_busy ? ph_wr_vld  : ld_wr_vld;
wire [20:0] sdr_ld_addr = ph_busy ? ph_wr_addr : ld_wr_addr;
wire [31:0] sdr_ld_data = ph_busy ? ph_wr_data : ld_wr_data;

ph_writer u_ph (
    .clk(clk), .rst(rst),
    .I_start(ph_start), .I_kind(ph_kind), .I_base(ph_base),
    .O_busy(ph_busy), .O_done(ph_done),
    .O_wr_vld(ph_wr_vld), .O_wr_addr(ph_wr_addr), .O_wr_data(ph_wr_data),
    .I_wr_ack(ld_wr_ack)
);

assign ld_wr_ack     = sdr_ld_vld;      // 平坦内存：当拍收、当拍 ack
assign ld_wr_settled = ~sdr_ld_vld;
assign ld_sd_ready   = sd_init_done;
assign ld_can_load   = ld_ready & ~quad_own;

// SDRAM：2M 字平坦数组
reg [31:0] dram [0:2097151];
always @(posedge clk) if (sdr_ld_vld) dram[sdr_ld_addr] <= sdr_ld_data;

// 纹理读：每层 4 字组，按 mask 依次回字
wire [1:0]  tex_req_ack, tex_pix_wr;
wire [41:0] tex_req_addr;
wire [7:0]  tex_req_mask;
wire [63:0] tex_pix_data;
wire [1:0]  tex_req_vld;
reg  [1:0]  tex_req_ack_r;
reg  [1:0]  tex_pix_wr_r;
reg  [63:0] tex_pix_data_r;

// 每层一个小队列（深度 8）
reg [20:0] q_base0 [0:7], q_base1 [0:7];
reg [3:0]  q_mask0 [0:7], q_mask1 [0:7];
reg [2:0]  q_w0, q_r0, q_w1, q_r1;
wire       q0_empty = (q_w0 == q_r0);
wire       q1_empty = (q_w1 == q_r1);
wire [2:0] q0_used  = q_w0 - q_r0;
wire [2:0] q1_used  = q_w1 - q_r1;

reg [3:0]  srv_k0, srv_k1;
reg [20:0] srv_b0, srv_b1;
reg [3:0]  srv_m0, srv_m1;
reg        srv_act0, srv_act1;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        tex_req_ack_r <= 2'd0; tex_pix_wr_r <= 2'd0; tex_pix_data_r <= 64'd0;
        q_w0 <= 0; q_r0 <= 0; q_w1 <= 0; q_r1 <= 0;
        srv_act0 <= 0; srv_act1 <= 0; srv_k0 <= 0; srv_k1 <= 0;
    end else begin
        tex_req_ack_r <= 2'd0;
        tex_pix_wr_r  <= 2'd0;

        // 入队
        if (tex_req_vld[0] && (q0_used < 7)) begin
            q_base0[q_w0[2:0]] <= tex_req_addr[20:0];
            q_mask0[q_w0[2:0]] <= tex_req_mask[3:0];
            q_w0 <= q_w0 + 3'd1;
            tex_req_ack_r[0] <= 1'b1;
        end
        if (tex_req_vld[1] && (q1_used < 7)) begin
            q_base1[q_w1[2:0]] <= tex_req_addr[41:21];
            q_mask1[q_w1[2:0]] <= tex_req_mask[7:4];
            q_w1 <= q_w1 + 3'd1;
            tex_req_ack_r[1] <= 1'b1;
        end

        // 服务 layer0
        if (!srv_act0 && !q0_empty) begin
            srv_b0 <= q_base0[q_r0[2:0]];
            srv_m0 <= q_mask0[q_r0[2:0]];
            q_r0   <= q_r0 + 3'd1;
            srv_k0 <= 0;
            srv_act0 <= 1'b1;
        end else if (srv_act0) begin
            if (srv_m0[srv_k0]) begin
                tex_pix_wr_r[0]  <= 1'b1;
                tex_pix_data_r[31:0] <= dram[srv_b0 + srv_k0];
            end
            if (srv_k0 == 3) srv_act0 <= 1'b0;
            else             srv_k0 <= srv_k0 + 1;
        end

        // 服务 layer1
        if (!srv_act1 && !q1_empty) begin
            srv_b1 <= q_base1[q_r1[2:0]];
            srv_m1 <= q_mask1[q_r1[2:0]];
            q_r1   <= q_r1 + 3'd1;
            srv_k1 <= 0;
            srv_act1 <= 1'b1;
        end else if (srv_act1) begin
            if (srv_m1[srv_k1]) begin
                tex_pix_wr_r[1]  <= 1'b1;
                tex_pix_data_r[63:32] <= dram[srv_b1 + srv_k1];
            end
            if (srv_k1 == 3) srv_act1 <= 1'b0;
            else             srv_k1 <= srv_k1 + 1;
        end
    end
end

assign tex_req_ack = tex_req_ack_r;
assign tex_pix_wr  = tex_pix_wr_r;
assign tex_pix_data= tex_pix_data_r;

//------------------------------------------------------------------------------
//  帧写口模型 + 抓帧
//------------------------------------------------------------------------------
reg        ps_start = 0, busy = 0;
wire       ps_new_frame;
wire [23:0] ps_data;
wire       ps_valid, pipe_ce;
reg  [23:0] fbuf [0:NPIX-1];
integer    pix_cnt = 0;
reg        in_frame = 0;
integer    frame_cnt = 0;
reg        frame_done = 0;
integer    scan_y, scan_x;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        ps_start <= 0; busy <= 0; pix_cnt <= 0; in_frame <= 0;
        frame_cnt <= 0; frame_done <= 0;
    end else begin
        frame_done <= 0;
        if (ps_new_frame && !in_frame) begin
            ps_start <= 1'b1;
            busy     <= 1'b1;
            pix_cnt  <= 0;
            in_frame <= 1'b1;
        end
        if (in_frame && ps_valid) begin
            scan_y = (IMG_H - 1) - (pix_cnt / IMG_W);
            scan_x = pix_cnt % IMG_W;
            fbuf[scan_y * IMG_W + scan_x] <= ps_data;
            if (pix_cnt == NPIX - 1) begin
                ps_start <= 1'b0;
                busy     <= 1'b0;
                in_frame <= 1'b0;
                frame_cnt <= frame_cnt + 1;
                frame_done <= 1'b1;
            end else
                pix_cnt <= pix_cnt + 1;
        end
    end
end

//------------------------------------------------------------------------------
//  App
//------------------------------------------------------------------------------
reg         key_prev = 0, key_next = 0, key_fx = 0, key_menu = 0;
reg  [3:0]  key_level = 4'hF;
wire        flash_cs_n, flash_sclk, flash_mosi, flash_wp_n, flash_hold_n, flash_miso;
wire [1:0]  st_img;
wire [2:0]  st_fx;
wire [3:0]  dbg_state;
wire [3:0]  fps_dbg;
wire        dbg_osd, dbg_fmt, fmt565;
wire [6:0]  pic_bright, pic_contrast, pic_temp, pic_sat;
wire [1:0]  pic_style;
wire        pic_tgl, o_lang, o_mute, o_loop, o_spc, oz_on;
wire [6:0]  o_vol;
wire [7:0]  oz_zoom;
wire [9:0]  oz_ax, oz_ay;

reg  quad_mode_r;

App #(
    .IMG_W(IMG_W), .IMG_H(IMG_H),
    .T_END(T_END), .T_END_F(T_END_F), .ARM_FRAMES(ARM_FRAMES),
    .SCENE_QUOTA(127), .QUOTA_565(127),
    .FRAME_PERIOD_CYCLES(FRAME_CYC),
    .THUMB0_ADDR(THUMB0), .THUMB1_ADDR(THUMB1), .THUMB2_ADDR(THUMB2),
    .HD_ADDR(HD_A)
) u_app (
    .clk(clk), .rst(rst), .mclk(mclk),
    .I_quad_mode(quad_mode_r),
    .O_page(osd_page_w), .O_file_page(osd_fp_w),

    .I_img_off0(alb_off0), .I_img_off1(alb_off1), .I_img_off2(alb_off2),
    .I_img_off3(32'd0),
    .O_alb_base(alb_base), .I_alb_win4(alb_win4), .I_alb_have(alb_have), .I_alb_ready(alb_ready),

    .O_nm_slot(lst_rd_slot), .O_nm_idx(lst_rd_idx), .I_nm_char(lst_rd_char),
    .I_nm_blen(lst_blen), .I_nm_elen(lst_elen), .I_nm_len4(lst_len4), .I_nm_vld(lst_vld),

    .O_fp_cur(fp_cur_w), .O_fp_flip_nx(fp_flip_nx), .O_fp_flip_pv(fp_flip_pv),
    .I_fp_vcnt(quad_vcnt), .I_fp_has_more(quad_has_more), .I_fp_gen(fp_gen_w),

    .ps_start(ps_start), .ps_new_frame(ps_new_frame),
    .ps_data(ps_data), .ps_valid(ps_valid), .pipe_ce(pipe_ce), .busy(busy),

    .tex_req_vld(tex_req_vld), .tex_req_addr(tex_req_addr), .tex_req_mask(tex_req_mask),
    .tex_req_ack(tex_req_ack), .tex_pix_wr(tex_pix_wr), .tex_pix_data(tex_pix_data),

    .key_prev(key_prev), .key_next(key_next), .key_fx(key_fx), .key_menu(key_menu),
    .key_level(key_level), .bright_eff(7'd50),

    .flash_cs_n(flash_cs_n), .flash_sclk(flash_sclk), .flash_mosi(flash_mosi),
    .flash_wp_n(flash_wp_n), .flash_hold_n(flash_hold_n), .flash_miso(flash_miso),

    .ld_req(ld_req), .ld_fmt(ld_fmt), .ld_off(ld_off), .ld_len(ld_len), .ld_base(ld_base),
    .ld_can_load(ld_can_load), .ld_done(ld_done), .ld_err(ld_err),
    .ld_sd_ready(ld_sd_ready), .ld_src_w(ld_src_w), .ld_src_h(ld_src_h),

    .music_done(1'b0),
    .bp_req(1'b0), .bp_addr(24'd0), .bp_len(16'd0),
    .bp_rdy(), .bp_data(), .bp_valid(), .bp_done(),

    .fmt565(fmt565), .wr_stall(1'b0),

    .st_img(st_img), .st_fx(st_fx), .fps_dbg(fps_dbg),
    .dbg_state(dbg_state), .dbg_osd(dbg_osd), .dbg_fmt(dbg_fmt),

    .pic_bright(pic_bright), .pic_contrast(pic_contrast), .pic_temp(pic_temp),
    .pic_sat(pic_sat), .pic_style(pic_style), .pic_tgl(pic_tgl),

    .o_lang(o_lang), .o_vol(o_vol), .o_mute(o_mute), .o_loop(o_loop), .o_spc(o_spc),

    .oz_on(oz_on), .oz_zoom(oz_zoom), .oz_ax(oz_ax), .oz_ay(oz_ay)
);

// 四宫格模式：只在帧边界切换（照抄 top）
always @(posedge clk or posedge rst) begin
    if (rst)              quad_mode_r <= 1'b0;
    else if (ps_new_frame) quad_mode_r <= quad_ready & osd_fp_w;
end

// 字库 flash
w25q64_model_24b #(
    .INIT_HEX("OSD/tools/font.hex"),
    .INIT_HEX_ICONS("OSD/tools/icons32.hex"),
    .ICON_ADDR(24'h03D000)
) u_flash (
    .cs_n(flash_cs_n), .sclk(flash_sclk), .mosi(flash_mosi),
    .miso(flash_miso)
);

//------------------------------------------------------------------------------
//  卡镜像映射表（每张图的 offset/size）
//------------------------------------------------------------------------------
integer map_off [0:15];
integer map_size[0:15];
reg [8*16-1:0] map_name [0:15];
integer map_n = 0;
integer map_fd, m_i;

// 由 tools/mk_fat32_img.py 生成：map_n / map_off[] / map_size[] / map_name[]
`include "../../../../sim_data/fat32_10img_map.vh"


task load_map;
    integer fd, n, it;
    reg [8*16-1:0] nm;
    integer o, s, w, h, r, g, b;
    begin
        fd = $fopen("../../../../sim_data/fat32_10img.map", "r");
        if (fd == 0) begin
            $display("  [tb] !! 打不开 map 文件");
            disable load_map;
        end
        n = $fgets(nm, fd);       // 跳过注释行
        for (it = 0; it < 32; it = it + 1) begin
            n = $fscanf(fd, "%s %d %d %d %d %d %d %d", nm, o, s, w, h, r, g, b);
            if (n != 8) it = 32;
            else begin
                map_name[map_n] = nm;
                map_off[map_n]  = o;
                map_size[map_n] = s;
                map_n = map_n + 1;
            end
        end
        $fclose(fd);
        $display("  [tb] map: %0d 个文件", map_n);
        $fflush();
    end
endtask

task show_map;
    integer i;
    begin
        for (i = 0; i < map_n; i = i + 1)
            $display("    %0s off=%0d size=%0d", map_name[i], map_off[i], map_size[i]);
    end
endtask

//------------------------------------------------------------------------------
//  按键 / 等待 / 抓帧
//------------------------------------------------------------------------------
task press;
    input [3:0] k;      // 1=KEY1 2=KEY2 4=KEY3 8=KEY4
    begin
        @(posedge clk);
        key_menu <= k[0]; key_fx <= k[1]; key_next <= k[2]; key_prev <= k[3];
        @(posedge clk);
        key_menu <= 0; key_fx <= 0; key_next <= 0; key_prev <= 0;
        @(posedge clk);
    end
endtask

task wait_frames;
    input integer n;
    integer i;
    begin
        for (i = 0; i < n; i = i + 1) @(posedge frame_done);
    end
endtask

task wait_state;
    input [3:0] s;
    input integer maxcyc;
    integer t0;
    reg ok;
    begin
        t0 = cyc; ok = 0;
        while ((cyc - t0) < maxcyc) begin
            @(posedge clk);
            if (u_app.u_ctrl.state == s) begin ok = 1; disable wait_state; end
        end
        $display("  [tb] !! wait_state(%0d) 超时（当前 %0d）", s, u_app.u_ctrl.state);
    end
endtask

integer ppm_fd;
task dump_ppm;
    input [8*24-1:0] tag;
    integer x, y;
    reg [8*64-1:0] fn;
    begin
        $sformat(fn, "out/chain_%0s.ppm", tag);
        ppm_fd = $fopen(fn, "wb");
        if (ppm_fd == 0) begin
            $display("  [tb] !! 打不开 %0s", fn);
        end else begin
            $fwrite(ppm_fd, "P6\n%0d %0d\n255\n", IMG_W, IMG_H);
            for (y = 0; y < IMG_H; y = y + 1)
                for (x = 0; x < IMG_W; x = x + 1) begin
                    $fwrite(ppm_fd, "%c%c%c",
                            fbuf[y*IMG_W+x][23:16],
                            fbuf[y*IMG_W+x][15:8],
                            fbuf[y*IMG_W+x][7:0]);
                end
            $fclose(ppm_fd);
            $display("  [tb] shot %0s  (frame %0d, state=%0d)", tag, frame_cnt, u_app.u_ctrl.state);
        end
    end
endtask

// 黑帧统计（每帧抽样 1/1024 像素）
integer lum_acc = 0, lum_n = 0, black_run = 0, black_max = 0;
always @(posedge frame_done) begin
    integer s, i;
    s = 0;
    for (i = 0; i < NPIX; i = i + 1024)
        s = s + fbuf[i][23:16] + fbuf[i][15:8] + fbuf[i][7:0];
    if (s < 3000) black_run = black_run + 1;
    else          black_run = 0;
    if (black_run > black_max) black_max = black_run;
end

//------------------------------------------------------------------------------
//  扇区统计
//------------------------------------------------------------------------------
integer sec_cnt [0:15];
integer sec_total = 0;
integer i0;
initial for (i0 = 0; i0 < 16; i0 = i0 + 1) sec_cnt[i0] = 0;

always @(posedge clk) begin
    integer j;
    if (rd_pulse) begin
        sec_total = sec_total + 1;
        for (j = 0; j < map_n; j = j + 1)
            if ((rd_lba * 512 >= map_off[j]) &&
                (rd_lba * 512 <  map_off[j] + map_size[j]))
                sec_cnt[j] = sec_cnt[j] + 1;
    end
end

// 周期性状态（排查卡在哪）
always @(posedge clk) begin
    if ((cyc % 500000) == 0) begin
        $display("  [t=%0d] st=%0d hv=%0d ard=%b lc=%0d lv=%b bpb=%b ast=%0d off0=%0d",
                 cyc, u_app.u_ctrl.state, alb_have, alb_ready, lst_cnt, lst_valid,
                 lst_bpb_ok, u_album.st, alb_off0);
        $fflush();
    end
end

//------------------------------------------------------------------------------
//  主场景
//------------------------------------------------------------------------------
integer k, t0, f0, fail = 0;

task show_sec;
    integer i;
    begin
        $write("  [tb] 各图读取扇区数：");
        for (i = 0; i < map_n; i = i + 1)
            $write(" %0s=%0d", map_name[i], sec_cnt[i]);
        $write("\n");
    end
endtask

initial begin
    $display("== tb_chain_full ==");
    $fflush();
    repeat (5) @(posedge clk);
    show_map;

    rst = 1;
    repeat (20) @(posedge clk);
    rst = 0;

    // 等开机装完（S_HD）
    wait_state(4'd5, 120_000_000);
    $display("  [tb] boot: state=%0d img=%0d tgt=%0d hd_valid=%0d quad=%0d",
             u_app.u_ctrl.state, u_app.u_ctrl.img_idx, u_app.u_ctrl.tgt_idx,
             u_app.u_ctrl.hd_valid, quad_ready);
    $fflush();
    repeat (1500) @(posedge clk);
    // ★ 采样前把几何/状态打出来：判定"这张图是不是在转场中途抓的"
    $display("  [geo] a_hd=%b a_en=%b fmt565=%b state=%0d t_fx=%0d",
             u_app.u_ctrl.a_hd, u_app.u_ctrl.a_en, u_app.u_ctrl.fmt565,
             u_app.u_ctrl.state, u_app.u_ctrl.t);
    $display("  [geo] pos=%h size=%h k=%h ori=%h",
             u_app.u_ctrl.pos_a, u_app.u_ctrl.size_a,
             u_app.u_ctrl.k_a, u_app.u_ctrl.ori_a);
    $display("  [geo] arm_geom=%b zoom_g=%b zph=%0d",
             u_app.u_ctrl.arm_geom, u_app.u_ctrl.zoom_g, u_app.u_ctrl.zph);
    $sformat(tmp_tag, "home_01"); dump_ppm(tmp_tag);
    $fflush();

    // 文件页：开菜单 → 下移两次（文件）→ 截图（应当还没进页）→ KEY2 → 截图
    press(4'h1);                               // KEY1 开菜单
    wait_frames(30);
    $sformat(tmp_tag, "nav"); dump_ppm(tmp_tag);
    press(4'h4); press(4'h4);                  // 下移两次
    wait_frames(30);
    $display("  [tb] nav at idx=%0d page=%0d (期望 page 仍为 0)",
             u_app.u_osd.u_menu.nav_idx, u_app.u_osd.u_menu.page);
    if (u_app.u_osd.u_menu.page != 3'd0) begin
        $display("  FAIL S3: 导航移动就切页了 page=%0d", u_app.u_osd.u_menu.page);
        fail = fail + 1;
    end
    $sformat(tmp_tag, "nav_file"); dump_ppm(tmp_tag);
    $fflush();

    press(4'h2);                               // KEY2 确认进入
    // 等四宫格装好（最多 2M 拍）
    t0 = cyc;
    while ((cyc - t0) < 2_000_000 && !quad_ready) @(posedge clk);
    wait_frames(40);
    $display("  [tb] enter file: page=%0d focus=%0d quad_ready=%0d vcnt=%0d ql_loaded=%0d own=%0d",
             u_app.u_osd.u_menu.page, u_app.u_osd.u_menu.focus,
             quad_ready, quad_vcnt, ql_loaded, quad_own);
    $fflush();
    $sformat(tmp_tag, "file_page"); dump_ppm(tmp_tag);
    $fflush();

    press(4'h4); wait_frames(20);
    press(4'h4); wait_frames(20);
    press(4'h4); wait_frames(20);
    $display("  [tb] cursor item=%0d sel_k=%0d", u_app.u_osd.u_menu.item, u_app.u_osd.u_menu.sel_k);
    $sformat(tmp_tag, "file_cur3"); dump_ppm(tmp_tag);
    $fflush();

    // 退出：KEY1 回导航栏，再 KEY1 关 OSD
    press(4'h1); wait_frames(10);
    $display("  [tb] after back: focus=%0d page=%0d", u_app.u_osd.u_menu.focus, u_app.u_osd.u_menu.page);
    press(4'h1);
    wait_state(4'd5, 40_000_000);
    if (u_app.u_ctrl.state != 4'd5) begin
        $display("  FAIL S3b: KEY1 两次仍在 state=%0d (osd_on=%0d quad_own=%0d)",
                 u_app.u_ctrl.state, u_app.u_ctrl.osd_on, quad_own);
        fail = fail + 1;
    end else begin
        $display("  [tb] exit ok: state=%0d ld_can_load=%0d", u_app.u_ctrl.state, ld_can_load);
        repeat (1500) @(posedge clk);
        $sformat(tmp_tag, "exit"); dump_ppm(tmp_tag);
    end

    show_sec;
    $display("  [tb] 黑帧最长连续 = %0d（>2 即算失败）", black_max);
    if (black_max > 2) fail = fail + 1;
    $display("== 结果：%0s（fail=%0d，frames=%0d，cyc=%0d）==",
             (fail == 0) ? "PASS" : "有失败", fail, frame_cnt, cyc);
    $finish;
end

// 超时兜底
initial begin
    #20_000_000_000;      // 20ms 仿真时间
    $display("  [tb] !! 全局超时（cyc=%0d state=%0d）", cyc, u_app.u_ctrl.state);
    $finish;
end

endmodule
