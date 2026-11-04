//==============================================================================
//  tb_fat_10.v —— 真 FAT32 镜像 + 相册窗口：10 张图的偏移全部正确
//==============================================================================
//  只跑存储链（卡 → 仲裁器 → lister → album_src），不跑显示：
//    · 对 base = 0..9 逐个重扫，断言 album 的 off0 == map 里第 base 张的偏移
//    · 断言 have / win4 与剩余张数一致
//  显示/窗口逻辑由 tb_window（viewer_ctrl）覆盖；两者合起来 = "10 张都能被读到"。
//
//  iverilog -g2012 -DIVERILOG_SIM -o f10.vvp -s tb_fat_10 \
//      tb_fat_10.v album_src.v ../SD/fat_dir_lister.v \
//      ../SD/sd_sector_file_model.v ../SD/sd_bus_arbiter4.v
//==============================================================================
`timescale 1ns/1ps

module tb_fat_10;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

// ---- 卡 + 仲裁器 ----
wire        sd_init_done, sd_read, sd_dv, sd_end;
wire [31:0] sd_addr;
wire [7:0]  sd_data;
wire        rd_pulse;
wire [31:0] rd_lba;

sd_sector_file_model #(
    .INIT_FILE("../../../../sim_data/fat32_10img.img"),
    .LAT_CYC(40), .INIT_CYC(50), .MAX_LBA(32'd131071)
) u_card (
    .clk(clk), .rst(rst), .O_init_done(sd_init_done),
    .I_sec_read(sd_read), .I_sec_addr(sd_addr),
    .O_sec_data(sd_data), .O_sec_dv(sd_dv), .O_sec_end(sd_end),
    .O_rd_pulse(rd_pulse), .O_rd_lba(rd_lba)
);

wire        fat_req, fat_dv, fat_end;
wire [31:0] fat_addr;
wire [7:0]  fat_data;

sd_bus_arbiter4 #(.ENA_TXT(0)) u_arb (
    .I_clk(clk), .I_rst(rst), .I_sd_init_done(sd_init_done),
    .I_img_req(1'b0), .I_img_addr(32'd0), .O_img_data(), .O_img_dv(), .O_img_end(),
    .I_aud_req(1'b0), .I_aud_addr(32'd0), .I_aud_urgent(1'b0), .O_aud_data(), .O_aud_dv(), .O_aud_end(),
    .I_fat_req(fat_req), .I_fat_addr(fat_addr),
    .O_fat_data(fat_data), .O_fat_dv(fat_dv), .O_fat_end(fat_end),
    .I_txt_req(1'b0), .I_txt_addr(32'd0), .O_txt_data(), .O_txt_dv(), .O_txt_end(),
    .O_sd_sec_read(sd_read), .O_sd_sec_addr(sd_addr),
    .I_sd_sec_data(sd_data), .I_sd_sec_dv(sd_dv), .I_sd_sec_end(sd_end),
    .O_timeout(), .O_owner()
);

wire        lst_start, lst_busy, lst_done, lst_has_more;
wire [2:0]  lst_cnt;
wire [1:0]  lst_ld_slot, alb_rd_slot;
wire [31:0] lst_off, lst_size;
wire [3:0]  lst_type, lst_valid;
wire [31:0] lst_root, lst_fat;
wire [3:0]  lst_spc;
wire        lst_bpb_ok;

reg  [15:0] alb_base = 16'd0;
wire        alb_lst_start, alb_sel, alb_win4, alb_ready, alb_any;
wire [31:0] alb_off0, alb_off1, alb_off2;
wire [2:0]  alb_nimg;

fat_dir_lister #(.N_ITEMS(4), .MAX_SECS(64), .INCLUDE_DIR(0)) u_fat (
    .clk(clk), .rst(rst),
    .I_start(alb_lst_start), .I_offset(alb_base),
    .I_list_all_types(1'b0),          // 相册：只数 BMP
    // ★ 2026-11-02：match 端口显式接 0（不接会 z→x）
    .I_match_en(1'b0), .I_match_rel(2'd0), .O_match_hit(), .O_match_off(),
    .O_busy(lst_busy), .O_done(lst_done), .O_valid_cnt(lst_cnt), .O_has_more(lst_has_more),
    .I_rd_slot(2'd0), .I_rd_idx(4'd0), .O_rd_char(),
    .I_ld_slot(lst_ld_slot),
    .O_ld_file_off(lst_off), .O_ld_size(lst_size), .O_ld_namelen(),
    .O_ld_blen(), .O_ld_elen(), .O_ld_len4(), .O_ld_vld(), .O_ld_type(lst_type),
    .O_slot_valid(lst_valid),
    .O_root_lba(lst_root), .O_fat_lba(lst_fat), .O_spc_shift(lst_spc), .O_bpb_ok(lst_bpb_ok),
    .O_sd_req(fat_req), .O_sd_addr(fat_addr),
    .I_sd_data(fat_data), .I_sd_dv(fat_dv), .I_sd_end(fat_end)
);

album_src u_album (
    .clk(clk), .rst(rst),
    .I_base(alb_base), .I_need(1'b1), .I_block(1'b0), .I_rescan(1'b0),
    .O_lst_start(alb_lst_start), .O_lst_offset(), .O_rd_slot(alb_rd_slot),
    .I_lst_busy(lst_busy), .I_lst_done(lst_done),
    .I_ld_type(lst_type), .I_slot_valid(lst_valid), .I_file_off(lst_off),
    .O_off0(alb_off0), .O_off1(alb_off1), .O_off2(alb_off2),
    .O_any(alb_any), .O_nimg(alb_nimg), .O_ready(alb_ready), .O_win4(alb_win4),
    .O_sel(alb_sel)
);

assign lst_ld_slot = alb_sel ? alb_rd_slot : 2'd0;

// map（由 tools/mk_fat32_img.py 生成）
integer map_off [0:15];
integer map_size[0:15];
reg [8*16-1:0] map_name [0:15];
integer map_n = 0;
`include "../../../../sim_data/fat32_10img_map.vh"

integer errors = 0, i;
integer exp_have, exp_win4;

initial begin
    $display("== tb_fat_10 ==");
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (200) @(posedge clk);

    for (i = 0; i < 10; i = i + 1) begin
        alb_base = i;
        // 等重扫完成（ready 先掉再起）
        repeat (5) @(posedge clk);
        while (!alb_ready) @(posedge clk);
        repeat (5) @(posedge clk);

        exp_have = (10 - i >= 4) ? 4 : (10 - i);
        exp_win4 = ((i + 3) < 10) ? 1 : 0;

        if (i == 0) begin
            if (!lst_bpb_ok) begin $display("  [FAIL] BPB 解析失败"); errors = errors + 1; end
            else $display("  BPB ok: root=%0d fat=%0d spc_shift=%0d", lst_root, lst_fat, lst_spc);
        end
        $display("  base=%0d off0=%0d (期望 %0d) nimg=%0d (期望 %0d) win4=%b (期望 %b) any=%b",
                 i, alb_off0, map_off[i], alb_nimg, exp_have, alb_win4, exp_win4, alb_any);
        if (alb_off0 !== map_off[i]) begin
            $display("  [FAIL] base=%0d 偏移错", i);
            errors = errors + 1;
        end
        if (alb_nimg !== exp_have[2:0]) begin
            $display("  [FAIL] base=%0d have 错", i);
            errors = errors + 1;
        end
        if (alb_win4 !== exp_win4[0]) begin
            $display("  [FAIL] base=%0d win4 错", i);
            errors = errors + 1;
        end
    end

    $display("== errors=%0d %s ==", errors, (errors == 0) ? "PASS" : "FAIL");
    $finish;
end

initial begin
    #50_000_000;
    $display("TIMEOUT");
    $finish;
end

endmodule
