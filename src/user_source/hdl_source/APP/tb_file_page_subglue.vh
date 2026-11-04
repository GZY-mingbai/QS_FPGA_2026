//==============================================================================
//  tb_file_page_subglue.vh —— FP_SUB 的顶层胶水（逐行照抄 top_tf_hdmi_audio.v，改顶层时同步）
//==============================================================================
//  match 扫描（音随图切）+ 同一次扫描里的 TXT（字幕）+ sub_loader / dyn_subtitle。
//  与顶层唯一的差别：本台把 osd_asy 视为打开、没有音频 ⇒ trk_done=0（见 mm_same）。
//==============================================================================
reg         mm_go_r, mm_pend, mm_live, mm_stale, mm_sub_ok;
reg  [1:0]  mm_rel_r;
reg         txt_found_r, sub_start, sub_has;
reg  [22:0] txt_lba_r, sub_sec;
assign mm_go  = mm_go_r;
assign mm_rel = mm_rel_r;
wire mm_hit_raw  = lst_match_hit & (lst_own == OWN_MM);
wire lst_done_mm = lst_done     & (lst_own == OWN_MM);
assign mm_hit = mm_hit_raw & ~mm_stale;
wire mm_same  = (lst_match_off == trk_sec_r);
wire txt_same = (txt_found_r == sub_has) && (!txt_found_r || (txt_lba_r == sub_sec));

always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        mm_pend <= 1'b0; mm_go_r <= 1'b0; mm_rel_r <= 2'd0; mm_live <= 1'b0; mm_stale <= 1'b0;
        mm_sub_ok <= 1'b0; txt_found_r <= 1'b0; txt_lba_r <= 23'd0;
        sub_start <= 1'b0; sub_has <= 1'b0; sub_sec <= 23'd0;
    end else begin
        mm_go_r   <= 1'b0;
        sub_start <= 1'b0;
        if (ld_req && ld_fmt) begin
            mm_pend  <= 1'b1;
            mm_rel_r <= hd_rel_w;
            if (mm_live) begin mm_stale <= 1'b1; mm_sub_ok <= 1'b0; end
        end
        if (mm_pend && !lst_busy && !osd_file_page && !alb_lst_start && alb_ready && !ld_busy) mm_go_r <= 1'b1;
        if (mm_win) begin mm_pend <= 1'b0; mm_live <= 1'b1; mm_sub_ok <= 1'b1; txt_found_r <= 1'b0; end
        if (mm_hit_raw | lst_done_mm) begin mm_live <= 1'b0; mm_stale <= 1'b0; end
        if (lst_match_txt_hit && (lst_own == OWN_MM) && mm_sub_ok) begin
            txt_found_r <= 1'b1; txt_lba_r <= lst_match_off;
        end
        if (lst_done_mm && mm_sub_ok && !txt_same) begin
            sub_start <= 1'b1; sub_has <= txt_found_r; sub_sec <= txt_lba_r;
        end
    end
end

assign ld_can_load = ld_ready & ~quad_own;
assign fat_req_g = lst_sd_req & ~((lst_own == OWN_MM) & ld_busy);
assign txt_req_g = sub_sd_req & ~ld_busy;

//------------------------------------------------------------------------------
//  sub_loader + dyn_subtitle（视频侧只要 vs：每 VS_PER 个 25MHz 拍一帧）
//------------------------------------------------------------------------------
wire [10:0] sub_wr_addr;
wire [15:0] sub_wr_data;
wire        sub_wr_we, sub_commit_tgl, sub_bank, sub_vs_tgl;
wire [8:0]  sub_len;

sub_loader u_sub_loader (
    .clk(sd_card_clk), .rst(rst_all),
    .I_start(sub_start), .I_txt_found(sub_has), .I_txt_sec(sub_sec),
    .O_sd_req(sub_sd_req), .O_sd_addr(sub_sd_addr),
    .I_sd_data(sub_sd_data), .I_sd_dv(sub_sd_dv), .I_sd_end(sub_sd_end),
    .O_fl_req(sub_fl_req), .O_fl_addr(sub_fl_addr), .O_fl_len(sub_fl_len),
    .I_fl_rdy(sub_fl_rdy), .I_fl_data(sub_fl_data), .I_fl_valid(sub_fl_valid), .I_fl_done(sub_fl_done),
    .I_vs_tgl(sub_vs_tgl),
    .O_wr_addr(sub_wr_addr), .O_wr_data(sub_wr_data), .O_wr_we(sub_wr_we),
    .O_commit_tgl(sub_commit_tgl), .O_bank(sub_bank), .O_text_len(sub_len)
);

`ifndef VS_PER
`define VS_PER 165000
`endif
reg        vclk = 1'b0;
always #20 vclk = ~vclk;
integer    vcnt = 0;
reg        v_vs = 1'b0;
always @(posedge vclk) begin
    vcnt <= (vcnt == `VS_PER - 1) ? 0 : vcnt + 1;
    v_vs <= (vcnt < 100);
end

dyn_subtitle u_dyn_subtitle (
    .video_clk(vclk), .rst(rst_all),
    .vs(v_vs), .de(1'b0), .rgb_in(24'd0),
    .show_en(~dbg_osd), .commit_tgl(sub_commit_tgl), .bank(sub_bank), .text_len(sub_len),
    .wr_clk(sd_card_clk), .wr_addr(sub_wr_addr), .wr_data(sub_wr_data), .wr_we(sub_wr_we),
    .vs_tgl(sub_vs_tgl), .vs_out(), .de_out(), .rgb_out()
);
