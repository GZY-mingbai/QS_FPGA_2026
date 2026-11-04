//==============================================================================
//  tb_quad_chain.v —— lister + quad_loader 集成（真实 RTL 对接）
//==============================================================================
//  iverilog -g2012 -o qc.vvp -s tb_quad_chain \
//      tb_quad_chain.v ../SD/fat_dir_lister.v quad_loader.v
//  vvp qc.vvp
//
//  【链路】
//      假 SD 卡（7 个目录项：4 可见 + LFN + 已删除 + 隐藏）
//          │
//          ▼
//      fat_dir_lister ──4 项 {off,len,blen,elen,type}──► quad_loader
//                                                             │
//                                                             ▼
//                                              (off, base) 4 次装载请求
//
//  【验证什么】
//  1. lister 从"脏目录"里只挑出 4 个可见项，LFN / 删除 / 隐藏全部跳过
//  2. loader 按顺序发 4 次装载，槽基址严格 = QUAD_BASE + k×40960（256 行×160 字）
//  3. loader 拿到的 (偏移, 长度) 与目录项里的簇号 / size 字段一致
//
//  【显示侧不在这里验】四宫格的像素生成已经做进 ps_gen_image（quad_en），
//  由 tb_quad_page 逐像素验证（4×19200 像素全对）。
//==============================================================================
`timescale 1ns/1ps

module tb_quad_chain;

// ★ 2026-11-01：lister 不再收 ROOT_LBA/DATA_LBA/SPC_SHIFT 参数，改成自己解析 BPB。
//   本台因此要给出"卡"的三段：MBR(0) / BPB(2048) / 根目录(4128)。
localparam [31:0]  PART_LBA = 32'd2048;
localparam [31:0]  DATA_LBA = 32'd4128;      // 2048 + 32(保留) + 2×1024(FAT)
localparam integer SPC_SH   = 3;             // spc = 8
localparam [20:0]  QBASE    = 21'd1048576;
localparam integer SLOT_SH  = 16;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

integer errors = 0, ncase = 0;
task chk;
    input cond;
    input [255:0] msg;
    begin
        ncase = ncase + 1;
        if (!cond) begin errors = errors + 1; $display("  FAIL [%0d] %0s", ncase, msg); end
    end
endtask

//==============================================================================
//  假 SD 卡（扇区模型，从 ROOT_LBA 开始线性映射）
//==============================================================================
reg [7:0] sd_mem [0:2047];      // 根目录（从 DATA_LBA 起）
reg [7:0] mbr_mem [0:511];
reg [7:0] bpb_mem [0:511];

task set_ent;
    input integer idx;
    input [7:0] b0,b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,b11;
    input [7:0] clo, chi, chi2;
    input [7:0] s0,s1,s2,s3;
    integer base;
    begin
        base = idx * 32;
        sd_mem[base+0]=b0;  sd_mem[base+1]=b1;  sd_mem[base+2]=b2;  sd_mem[base+3]=b3;
        sd_mem[base+4]=b4;  sd_mem[base+5]=b5;  sd_mem[base+6]=b6;  sd_mem[base+7]=b7;
        sd_mem[base+8]=b8;  sd_mem[base+9]=b9;  sd_mem[base+10]=b10; sd_mem[base+11]=b11;
        // FAT32：20/21 = 高 16 位，26/27 = 低 16 位
        sd_mem[base+20]=chi; sd_mem[base+21]=8'h00; sd_mem[base+26]=clo; sd_mem[base+27]=8'h00;
        sd_mem[base+28]=s0; sd_mem[base+29]=s1; sd_mem[base+30]=s2; sd_mem[base+31]=s3;
    end
endtask

task init_sd;
    integer i;
    begin
        for (i = 0; i < 2048; i = i + 1) sd_mem[i] = 8'h00;
        for (i = 0; i < 512; i = i + 1) begin
            mbr_mem[i] = 8'h00;
            bpb_mem[i] = 8'h00;
        end
        // MBR：第 1 个分区从 2048 开始
        mbr_mem[454] = PART_LBA[7:0];
        mbr_mem[455] = PART_LBA[15:8];
        // BPB：spc=8, reserved=32, nfat=2, fatsz32=1024
        bpb_mem[13] = 8'd8;
        bpb_mem[14] = 8'd32;
        bpb_mem[16] = 8'd2;
        bpb_mem[37] = 8'h04;
        // 0: APPLE.BMP     clus=3 size=1000
        set_ent(0,"A","P","P","L","E"," "," "," ","B","M","P",8'h20, 8'h03,8'h00,8'h00, 8'hE8,8'h03,8'h00,8'h00);
        // 1: LFN（跳过）
        set_ent(1,8'h41,8'h42,8'h0F,0,0,0,0,0,0,0,0,8'h0F, 0,0,0, 0,0,0,0);
        // 2: MUSIC001.WAV   clus=4 size=2000
        set_ent(2,"M","U","S","I","C","0","0","1","W","A","V",8'h20, 8'h04,8'h00,8'h00, 8'hD0,8'h07,8'h00,8'h00);
        // 3: NOTE0001.TXT   clus=5 size=300
        set_ent(3,"N","O","T","E","0","0","0","1","T","X","T",8'h20, 8'h05,8'h00,8'h00, 8'h2C,8'h01,8'h00,8'h00);
        // 4: PHOTO002.BMP   clus=6 size=4000
        set_ent(4,"P","H","O","T","O","0","0","2","B","M","P",8'h20, 8'h06,8'h00,8'h00, 8'hA0,8'h0F,8'h00,8'h00);
        // 5: 已删除（跳过）
        set_ent(5,8'hE5,"E","L","E","T","E","D"," ","B","M","P",8'h20, 8'h07,8'h00,8'h00, 0,0,0,0);
        // 6: HIDDEN.BMP attr=0x22（跳过）
        set_ent(6,"H","I","D","D","E","N"," "," ","B","M","P",8'h22, 8'h08,8'h00,8'h00, 0,0,0,0);
    end
endtask

//==============================================================================
//  DUT 1：lister
//==============================================================================
wire [1:0] q_rd_slot;               // loader 当前选的项（前向声明）
reg        ld_start = 0;
wire       ld_busy, ld_done;
wire [2:0] ld_valid_cnt;
wire       ld_has_more;
wire [7:0] ld_rd_char;
wire [31:0] ld_file_off, ld_file_len;
wire [3:0] ld_blen, ld_elen, ld_type, ld_slot_valid;
wire       lst_sd_req;
wire [31:0] lst_sd_addr;
reg  [7:0] sd_data = 0;
reg        sd_dv = 0, sd_end = 0;

fat_dir_lister #(
    .N_ITEMS(4), .MAX_SECS(8), .INCLUDE_DIR(0)
) u_lst (
    .clk(clk), .rst(rst),
    .I_start(ld_start), .I_offset(16'd0),
    .I_list_all_types(1'b1),          // 文件页：按条目
    .I_match_en(1'b0), .I_match_rel(2'd0),   // ★ match 口接 0（本台不测）
    .O_busy(ld_busy), .O_done(ld_done), .O_valid_cnt(ld_valid_cnt), .O_has_more(ld_has_more),
    .I_rd_slot(2'd0), .I_rd_idx(4'd0), .O_rd_char(ld_rd_char),
    .I_ld_slot(q_rd_slot),
    .O_ld_file_off(ld_file_off), .O_ld_size(ld_file_len),
    .O_ld_namelen(), .O_ld_blen(ld_blen), .O_ld_elen(ld_elen),
    .O_ld_type(ld_type), .O_slot_valid(ld_slot_valid),
    .O_sd_req(lst_sd_req), .O_sd_addr(lst_sd_addr),
    .I_sd_data(sd_data), .I_sd_dv(sd_dv), .I_sd_end(sd_end)
);

//==============================================================================
//  DUT 2：loader
//==============================================================================
reg  q_ld_start = 0;
wire q_busy, q_done;
wire [2:0] q_loaded;
wire bl_start;
wire [31:0] bl_off, bl_len;
wire [20:0] bl_base;
reg  bl_ready = 1, bl_busy = 0, bl_done = 0, bl_err = 0;

quad_loader #(
    .SLOT_BASE(QBASE), .SLOT_STEP(40960), .N_ITEMS(4)
) u_ld (
    .clk(clk), .rst(rst),
    .I_start(q_ld_start), .I_abort(1'b0), .I_hold(1'b0), .O_bmp(), .O_filled(), .I_valid_cnt(ld_valid_cnt), .I_slot_valid(ld_slot_valid),
    .O_busy(q_busy), .O_done(q_done), .O_loaded(q_loaded),
    .O_rd_slot(q_rd_slot), .I_file_off(ld_file_off), .I_file_len(ld_file_len),
    // ⚠ 本台全部按 BMP 处理（占位图通路由 tb_quad_ph.v 覆盖）
    .I_file_type(4'd1),
    .O_ph_start(), .O_ph_kind(), .O_ph_base(), .I_ph_busy(1'b0), .I_ph_done(1'b0),
    .O_ld_start(bl_start), .O_ld_off(bl_off), .O_ld_len(bl_len), .O_ld_base(bl_base),
    .I_ld_ready(bl_ready), .I_ld_busy(bl_busy), .I_ld_done(bl_done), .I_ld_err(bl_err)
);

//==============================================================================
//  SD 扇区应答（req → 2 拍 → 512 字节 → end）
//==============================================================================
reg        reading = 0;
reg [3:0]  waitc = 0;
reg [2:0]  settle = 3'd0;
integer    bc;
reg [1:0]  src;              // 0=MBR 1=BPB 2=目录
reg [31:0] cur_base;

always @(posedge clk) begin
    sd_dv  <= 0;
    sd_end <= 0;
    if (rst) begin
        reading <= 0;
        settle  <= 3'd0;
    end else if (!reading) begin
        // 地址要等 DUT 把新值写进 O_sd_addr 之后再锁（3 拍，见 tb_fat_dir_lister）
        if (lst_sd_req) settle <= settle + 3'd1;
        else            settle <= 3'd0;
        if (lst_sd_req && settle >= 3'd2) begin
            settle <= 3'd0;
            if (lst_sd_addr == 32'd0)          begin src <= 2'd0; cur_base <= 0; end
            else if (lst_sd_addr == PART_LBA)  begin src <= 2'd1; cur_base <= 0; end
            else                               begin src <= 2'd2; cur_base <= (lst_sd_addr - DATA_LBA) * 512; end
            reading  <= 1;
            waitc    <= 2;
            bc       <= 0;
        end
    end else begin
        if (waitc != 0) waitc <= waitc - 1;
        else if (bc < 512) begin
            sd_dv   <= 1;
            sd_data <= (src == 2'd0) ? mbr_mem[bc] :
                       (src == 2'd1) ? bpb_mem[bc] : sd_mem[cur_base + bc];
            bc      <= bc + 1;
        end else begin
            sd_end  <= 1;
            reading <= 0;
        end
    end
end

//==============================================================================
//  装载模型：收下 (off, base)，3 拍后 done
//==============================================================================
integer    n_load = 0;
reg [20:0] rec_base [0:3];
reg [31:0] rec_off  [0:3];
reg [31:0] rec_len  [0:3];
reg        ld_run = 0;
integer    ld_wait = 0;

always @(posedge clk) begin
    bl_done <= 0; bl_err <= 0;
    if (rst) begin
        ld_run <= 0; bl_busy <= 0;
    end else if (!ld_run) begin
        bl_busy <= 0;
        if (bl_start) begin
            ld_run  <= 1;
            bl_busy <= 1;
            ld_wait <= 3;
            if (n_load < 4) begin
                rec_base[n_load] <= bl_base;
                rec_off[n_load]  <= bl_off;
                rec_len[n_load]  <= bl_len;
            end
            n_load <= n_load + 1;
        end
    end else begin
        if (ld_wait != 0) ld_wait <= ld_wait - 1;
        else begin
            bl_done <= 1;
            bl_busy <= 0;
            ld_run  <= 0;
        end
    end
end

//==============================================================================
//  主流程
//==============================================================================
initial begin
    init_sd();
    $display("  fake SD ready: 7 entries (4 visible + LFN + deleted + hidden)");

    repeat (5) @(posedge clk);
    rst = 0;
    repeat (3) @(posedge clk);

    // ---- 扫目录 ----
    $display("\n[1] scan root dir");
    @(posedge clk); ld_start = 1;
    @(posedge clk); ld_start = 0;
    wait (ld_done);
    repeat (2) @(posedge clk);
    $display("      valid_cnt=%0d slot_valid=%b has_more=%b",
             ld_valid_cnt, ld_slot_valid, ld_has_more);
    chk(ld_valid_cnt == 4,      "1.1 found 4 visible files");
    chk(ld_slot_valid == 4'hF,  "1.2 all 4 slots valid");
    chk(ld_has_more == 1'b0,    "1.3 no more pages after 4");

    // ---- 装载 ----
    $display("[2] quad_loader");
    @(posedge clk); q_ld_start = 1;
    @(posedge clk); q_ld_start = 0;
    wait (q_done);
    repeat (2) @(posedge clk);
    $display("      n_load=%0d loaded=%0d", n_load, q_loaded);
    chk(n_load == 4,   "2.1 issued 4 load requests");
    chk(q_loaded == 4, "2.2 4 loads reported ok");

    // 槽基址必须严格 2^16 递增
    chk(rec_base[0] == QBASE,                    "2.3 slot0 base");
    chk(rec_base[1] == QBASE + 21'd40960,        "2.4 slot1 base");
    chk(rec_base[2] == QBASE + 21'd81920,       "2.5 slot2 base");
    chk(rec_base[3] == QBASE + 21'd122880,       "2.6 slot3 base");

    // 文件偏移：簇 3/4/5/6 → 扇区 1064/1072/1080/1088 → 字节偏移
    $display("      off order: %h %h %h %h", rec_off[0], rec_off[1], rec_off[2], rec_off[3]);
    chk(rec_off[0] == 32'd2117632, "2.7 off0 = clus3");
    chk(rec_off[1] == 32'd2121728, "2.8 off1 = clus4");
    chk(rec_off[2] == 32'd2125824, "2.9 off2 = clus5");
    chk(rec_off[3] == 32'd2129920, "2.10 off3 = clus6");

    // 长度：bmp_loader 的 file_length 是死输入（内部 flen 只写不读），
    // 所以 lister 的 O_ld_size 恒 0 —— 这里确认它确实是 0，别又长出逻辑
    chk(rec_len[0] == 32'd0, "2.11 len0 == 0 (dead input)");
    chk(rec_len[2] == 32'd0, "2.12 len2 == 0 (dead input)");

    $display("\n========================================");
    if (errors == 0) $display("  ALL PASS  (%0d checks)", ncase);
    else             $display("  FAIL: %0d / %0d", errors, ncase);
    $display("========================================\n");
    $finish;
end

initial begin
    #20_000_000;
    $display("  FAIL: watchdog");
    $finish;
end

endmodule
