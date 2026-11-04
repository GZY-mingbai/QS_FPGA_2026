//==============================================================================
//  tb_fat_dir_lister.v —— 目录扫描器单元测试（含动态 BPB）
//==============================================================================
//  iverilog -g2012 -o fdl.vvp -s tb_fat_dir_lister \
//      tb_fat_dir_lister.v fat_dir_lister.v && vvp fdl.vvp
//
//  这一段测试覆盖：
//    T1  MBR 两段式：0 号扇区是 MBR，BPB 在第 2048 扇区
//    T2  目录项过滤（LFN / 删除 / 隐藏 / 卷标）
//    T3  8.3 拆分（blen/elen）
//    T4  offset 翻页
//    T5  offset 越界
//    T6  superfloppy：0 号扇区就是 BPB（part_lba = 0）
//    T7  非法 BPB（spc 不是 2 的幂）→ 兜底常量 + 不卡死
//
//  造的目录（第 1 扇区）：
//      0  APPLE.BMP      attr=0x20 clus=3
//      1  LFN 项         attr=0x0F            ← 应跳过
//      2  MUSIC001.WAV   attr=0x20 clus=4
//      3  NOTE0001.TXT   attr=0x20 clus=5
//      4  已删除         attr=0x20 首字节0xE5  ← 应跳过
//      5  HIDDEN.BMP     attr=0x22            ← 隐藏，应跳过
//      6  0x00 结束
//==============================================================================
`timescale 1ns/1ps

module tb_fat_dir_lister;

// ---- MBR 布局 ----
localparam [31:0] PART_LBA = 32'd2048;
localparam [31:0] RESERVED = 32'd32;
localparam [31:0] NFAT     = 2;
localparam [31:0] FATSZ    = 1024;
// FAT_LBA = 2048+32 = 2080；DATA_LBA = 2080 + 2*1024 = 4128
localparam [31:0] EXP_FAT  = 32'd2080;
localparam [31:0] EXP_DATA = 32'd4128;
localparam        EXP_SH   = 3;             // spc = 8

reg clk = 0, rst = 1;
reg start = 0;
reg [15:0] offset = 0;
reg list_all = 1;                       // ★ 1 = wav/txt 也列（文件页语义，T1~T7 用这个）
// ★ 2026-11-02：match 模式端口（T9）
reg         m_en  = 0;
reg  [1:0]  m_rel = 0;
wire        m_hit;
wire [22:0] m_off;
wire        m_txt_hit;
wire [22:0] m_txt_off = m_off;      // ★ 2026-11-07：TXT 偏移与 WAV 共用 O_match_off
reg         hit_seen;
reg  [22:0] m_off_r;
reg         txt_seen;
reg  [22:0] txt_off_r;
wire busy, done;
wire [2:0] valid_cnt;
wire has_more;
wire [7:0] rd_char;
reg  [1:0] rd_slot = 2'd0;          // ★ T10：名字读口地址（由 OSD 给）
reg  [3:0] rd_idx  = 4'd0;
reg  [7:0] t10_ref;
wire [31:0] ld_off, ld_size;
wire [3:0] ld_len, ld_blen, ld_elen, ld_type, slot_valid;
wire sd_req;
wire [31:0] sd_addr;
reg [7:0] sd_data = 0;
reg sd_dv = 0, sd_end = 0;
wire [31:0] root_lba_w, fat_lba_w;
wire [3:0]  spc_sh_w;
wire        bpb_ok_w;

integer errors = 0, ncase = 0;
task chk;
    input cond;
    input [255:0] msg;
    begin
        ncase = ncase + 1;
        if (!cond) begin
            errors = errors + 1;
            $display("  FAIL [%0d] %0s", ncase, msg);
            $display("        got=%0d exp=%0d", cond, 1);
        end
    end
endtask

//------------------------------------------------------------------------------
//  三个扇区源：MBR / BPB / 目录
//------------------------------------------------------------------------------
reg [7:0] mbr_mem [0:511];
reg [7:0] bpb_mem [0:511];
reg [7:0] dir_mem [0:1023];

task init_mbr;
    integer i;
    begin
        for (i = 0; i < 512; i = i + 1) mbr_mem[i] = 8'h00;
        mbr_mem[0]   = 8'hFA;           // 随便一条 boot 指令，不是 "FAT32"
        mbr_mem[82]  = 8'h00;           // 关键：这里不是 "FAT32"
        mbr_mem[454] = PART_LBA[7:0];
        mbr_mem[455] = PART_LBA[15:8];
        mbr_mem[456] = PART_LBA[23:16];
        mbr_mem[457] = PART_LBA[31:24];
        mbr_mem[510] = 8'h55;
        mbr_mem[511] = 8'hAA;
    end
endtask

task init_bpb;
    input integer spc;
    input [31:0] reserved;
    input [31:0] nfat;
    input [31:0] fatsz;
    input         sf;                   // 1 = 写成 "FAT32" 签名（superfloppy）
    integer i;
    begin
        for (i = 0; i < 512; i = i + 1) bpb_mem[i] = 8'h00;
        bpb_mem[0]  = 8'hEB;
        bpb_mem[1]  = 8'h58;
        bpb_mem[2]  = 8'h90;
        bpb_mem[11] = 8'h00; bpb_mem[12] = 8'h02;      // 512 字节/扇区
        bpb_mem[13] = spc[7:0];
        bpb_mem[14] = reserved[7:0];
        bpb_mem[15] = reserved[15:8];
        bpb_mem[16] = nfat[7:0];
        bpb_mem[22] = 8'h00; bpb_mem[23] = 8'h00;      // fatsz16 = 0（FAT32）
        bpb_mem[36] = fatsz[7:0];
        bpb_mem[37] = fatsz[15:8];
        bpb_mem[38] = fatsz[23:16];
        bpb_mem[39] = fatsz[31:24];
        bpb_mem[44] = 8'h02; bpb_mem[45] = 0; bpb_mem[46] = 0; bpb_mem[47] = 0;
        if (sf) begin
            bpb_mem[82] = "F"; bpb_mem[83] = "A"; bpb_mem[84] = "T";
            bpb_mem[85] = "3"; bpb_mem[86] = "2";
        end
        bpb_mem[510] = 8'h55; bpb_mem[511] = 8'hAA;
    end
endtask

task set_entry;
    input integer idx;
    input [7:0] b0,b1,b2,b3,b4,b5,b6,b7,b8,b9,b10,b11;
    input integer clus;                 // ★ 一个整簇号，内部按 FAT32 顺序拆
    input [7:0] s0,s1,s2,s3;
    integer base;
    begin
        base = idx * 32;
        dir_mem[base+0]  = b0;  dir_mem[base+1]  = b1;
        dir_mem[base+2]  = b2;  dir_mem[base+3]  = b3;
        dir_mem[base+4]  = b4;  dir_mem[base+5]  = b5;
        dir_mem[base+6]  = b6;  dir_mem[base+7]  = b7;
        dir_mem[base+8]  = b8;  dir_mem[base+9]  = b9;
        dir_mem[base+10] = b10; dir_mem[base+11] = b11;
        // FAT32：20/21 = 高 16 位，26/27 = 低 16 位
        dir_mem[base+20] = clus[15:8]; dir_mem[base+21] = 8'h00;
        dir_mem[base+26] = clus[7:0];  dir_mem[base+27] = 8'h00;
        dir_mem[base+28] = s0; dir_mem[base+29] = s1;
        dir_mem[base+30] = s2; dir_mem[base+31] = s3;
    end
endtask

task init_dir;
    integer i;
    begin
        for (i = 0; i < 1024; i = i + 1) dir_mem[i] = 8'h00;
        // 0: APPLE.BMP  clus=3
        set_entry(0,"A","P","P","L","E"," "," "," ","B","M","P",8'h20, 3, 8'hE8,8'h03,8'h00,8'h00);
        // 1: LFN
        set_entry(1,8'h41,8'h42,8'h0F,0,0,0,0,0,0,0,0,8'h0F, 0, 0,0,0,0);
        // 2: MUSIC001.WAV clus=4
        set_entry(2,"M","U","S","I","C","0","0","1","W","A","V",8'h20, 4, 8'hD0,8'h07,8'h00,8'h00);
        // 3: NOTE0001.TXT clus=5
        set_entry(3,"N","O","T","E","0","0","0","1","T","X","T",8'h20, 5, 8'h2C,8'h01,8'h00,0);
        // 4: 已删除
        set_entry(4,8'hE5,"E","L","E","T","E","D"," ","B","M","P",8'h20, 6, 0,0,0,0);
        // 5: HIDDEN.BMP (attr=0x22)
        set_entry(5,"H","I","D","D","E","N"," "," ","B","M","P",8'h22, 7, 0,0,0,0);
        // 6: 结束（全 0）
    end
endtask

//------------------------------------------------------------------------------
//  T8 用的真卡布局：4 张图 + music.wav + 第 5 张图
//      0 B1.BMP  clus=3      1 B2.BMP  clus=4
//      2 B3.BMP  clus=5      3 B4.BMP  clus=6
//      4 MUSIC.WAV clus=7    5 B5.BMP  clus=8
//      6 结束（全 0）
//  ★ 这正是上板出问题的那张 D 盘：[图1..图4, WAV, 第5张]。
//------------------------------------------------------------------------------
task init_dir_wav_case;
    integer i;
    begin
        for (i = 0; i < 1024; i = i + 1) dir_mem[i] = 8'h00;
        set_entry(0,"B","1"," "," "," "," "," "," ","B","M","P",8'h20, 3, 0,0,0,0);
        set_entry(1,"B","2"," "," "," "," "," "," ","B","M","P",8'h20, 4, 0,0,0,0);
        set_entry(2,"B","3"," "," "," "," "," "," ","B","M","P",8'h20, 5, 0,0,0,0);
        set_entry(3,"B","4"," "," "," "," "," "," ","B","M","P",8'h20, 6, 0,0,0,0);
        set_entry(4,"M","U","S","I","C"," "," "," ","W","A","V",8'h20, 7, 0,0,0,0);
        set_entry(5,"B","5"," "," "," "," "," "," ","B","M","P",8'h20, 8, 0,0,0,0);
    end
endtask

//------------------------------------------------------------------------------
//  T9 用：同名 图 + WAV 的三种排布
//    A) WAV 排在图片**后面**（最常见：先拷图再配乐）
//    B) 有 WAV 但名字不同 / 同名但扩展名不是 WAV  ⇒ 不应该命中
//    C) WAV 排在图片**前面**  ⇒ 单遍扫描也应该命中（按名字匹配，与顺序无关）
//------------------------------------------------------------------------------
task init_dir_match_after;
    integer i;
    begin
        for (i = 0; i < 1024; i = i + 1) dir_mem[i] = 8'h00;
        set_entry(0,"B","1"," "," "," "," "," "," ","B","M","P",8'h20, 3, 0,0,0,0);
        set_entry(1,"B","1"," "," "," "," "," "," ","W","A","V",8'h20, 9, 0,0,0,0);
        set_entry(2,"B","2"," "," "," "," "," "," ","B","M","P",8'h20, 4, 0,0,0,0);
        set_entry(3,"B","2"," "," "," "," "," "," ","W","A","V",8'h20, 10, 0,0,0,0);
    end
endtask

task init_dir_match_miss;
    integer i;
    begin
        for (i = 0; i < 1024; i = i + 1) dir_mem[i] = 8'h00;
        set_entry(0,"B","1"," "," "," "," "," "," ","B","M","P",8'h20, 3, 0,0,0,0);
        set_entry(1,"O","T","H","E","R"," "," "," ","W","A","V",8'h20, 7, 0,0,0,0);
        set_entry(2,"B","1"," "," "," "," "," "," ","T","X","T",8'h20, 5, 0,0,0,0);
    end
endtask

task init_dir_match_before;
    integer i;
    begin
        for (i = 0; i < 1024; i = i + 1) dir_mem[i] = 8'h00;
        set_entry(0,"B","7"," "," "," "," "," "," ","W","A","V",8'h20, 11, 0,0,0,0);
        set_entry(1,"B","7"," "," "," "," "," "," ","B","M","P",8'h20, 6, 0,0,0,0);
    end
endtask

// 同名但扩展名不是 WAV 的干扰项排在前面 ⇒ 必须跳过它，命中后面的 B1.WAV
task init_dir_match_txt;
    integer i;
    begin
        for (i = 0; i < 1024; i = i + 1) dir_mem[i] = 8'h00;
        set_entry(0,"B","1"," "," "," "," "," "," ","B","M","P",8'h20, 3, 0,0,0,0);
        set_entry(1,"B","1"," "," "," "," "," "," ","T","X","T",8'h20, 4, 0,0,0,0);
        set_entry(2,"B","1"," "," "," "," "," "," ","W","A","V",8'h20, 12, 0,0,0,0);
    end
endtask

//------------------------------------------------------------------------------
//  DUT
//------------------------------------------------------------------------------
fat_dir_lister #(
    .N_ITEMS(4), .MAX_SECS(8), .INCLUDE_DIR(0)
) dut (
    .clk(clk), .rst(rst),
    .I_start(start), .I_offset(offset),
    .I_list_all_types(list_all),
    // ★ 2026-11-02：match 模式（T9 用）
    .I_match_en(m_en), .I_match_rel(m_rel),
    .O_match_hit(m_hit), .O_match_off(m_off),
    .O_match_txt_hit(m_txt_hit),
    .O_busy(busy), .O_done(done), .O_valid_cnt(valid_cnt), .O_has_more(has_more),
    .I_rd_slot(rd_slot), .I_rd_idx(rd_idx), .O_rd_char(rd_char),
    .I_ld_slot(2'd0), .O_ld_file_off(ld_off), .O_ld_size(ld_size),
    .O_ld_namelen(ld_len), .O_ld_blen(ld_blen), .O_ld_elen(ld_elen),
    .O_ld_len4(), .O_ld_vld(), .O_ld_type(ld_type), .O_slot_valid(slot_valid),
    .O_root_lba(root_lba_w), .O_fat_lba(fat_lba_w),
    .O_spc_shift(spc_sh_w), .O_bpb_ok(bpb_ok_w),
    .O_sd_req(sd_req), .O_sd_addr(sd_addr),
    .I_sd_data(sd_data), .I_sd_dv(sd_dv), .I_sd_end(sd_end)
);

always #10 clk = ~clk;

//------------------------------------------------------------------------------
//  SD 模型：req → 2 拍后吐 512 字节 → end
//  ★ 按地址选源：0 → MBR，PART_LBA → BPB，其余 → 目录（按 EXP_DATA 基址）
//------------------------------------------------------------------------------
reg        reading = 0;
reg [3:0]  waitc = 0;
integer    bc;
reg [31:0] cur_base;
reg [31:0] lat_addr;
reg [1:0]  src;             // 0=MBR 1=BPB 2=目录
reg [31:0] model_root = EXP_DATA;   // 本用例的根目录首扇区（测试自己设）
reg [2:0]  settle = 3'd0;           // 地址稳定计数

always @(posedge clk) begin
    sd_dv  <= 0;
    sd_end <= 0;
    if (rst) begin reading <= 0; end
    else if (!reading) begin
        // ★ 地址稳定要等几拍：DUT 在「看到 end」后还要过 S_CALC 那一拍才把新地址
        //   写进 O_sd_addr。这里留 3 拍再锁（真控制器也要经过 CMD17 的串行时序，
        //   3 拍远小于它）。同拍锁存会抓到旧地址 ⇒ 把 MBR 读两遍 / 拿 BPB 当目录。
        if (sd_req) settle <= settle + 3'd1;
        else        settle <= 3'd0;
        if (sd_req && (settle >= 3'd2)) begin
            settle <= 3'd0;
            lat_addr <= sd_addr;
            if (sd_addr == 32'd0) begin
                src <= 2'd0; cur_base <= 0;
            end else if (sd_addr == PART_LBA) begin
                src <= 2'd1; cur_base <= 0;
            end else begin
                src <= 2'd2; cur_base <= (sd_addr - model_root) * 512;
            end
            reading  <= 1;
            waitc    <= 2;
            bc       <= 0;
        end
    end else begin
        if (waitc != 0) waitc <= waitc - 1;
        else if (bc < 512) begin
            sd_dv   <= 1;
            sd_data <= (src == 2'd0) ? mbr_mem[bc] :
                       (src == 2'd1) ? bpb_mem[bc] : dir_mem[cur_base + bc];
            bc      <= bc + 1;
        end else begin
            sd_end  <= 1;
            reading <= 0;
        end
    end
end

//------------------------------------------------------------------------------
//  match 扫描（T9）：start 一拍 + m_en 保持到扫完；命中脉冲可能比 done 晚 1~8 拍
//------------------------------------------------------------------------------
task run_match;
    input [1:0] rel;
    integer g;
    reg seen;
    begin
        @(posedge clk);
        offset  = 16'd0;
        m_rel   = rel;
        m_en    = 1'b1;
        start   = 1'b1;
        @(posedge clk);
        start   = 1'b0;
        seen    = 1'b0;
        m_off_r = 23'h7FFFFF;
        txt_seen = 1'b0;
        txt_off_r = 23'h7FFFFF;
        g       = 0;
        while ((g < 200000) && !seen) begin
            @(negedge clk);
            if (m_hit) begin seen = 1'b1; m_off_r = m_off; end
            if (m_txt_hit) begin txt_seen = 1'b1; txt_off_r = m_txt_off; end
            g = g + 1;
        end
        hit_seen = seen;
        m_en     = 1'b0;
        repeat (600) @(posedge clk);
    end
endtask

//------------------------------------------------------------------------------
task run_scan;
    input [15:0] off;
    begin
        @(posedge clk);
        offset = off;
        start  = 1;
        @(posedge clk);
        start  = 0;
        while (!done) @(posedge clk);
        repeat (600) @(posedge clk);   // 等上一次传输彻底结束
    end
endtask

//------------------------------------------------------------------------------
initial begin
    init_mbr();
    init_bpb(8, RESERVED, NFAT, FATSZ, 0);
    init_dir();

    repeat (5) @(posedge clk);
    rst = 0;
    repeat (3) @(posedge clk);

    //======================================================================
    $display("\n[T1] MBR two-stage BPB parse");
    run_scan(0);
    $display("      root=%0d fat=%0d sh=%0d ok=%b valid=%0d",
             root_lba_w, fat_lba_w, spc_sh_w, bpb_ok_w, valid_cnt);
    chk(root_lba_w == EXP_DATA, "T1 ROOT_LBA == DATA_LBA(4128)");
    chk(fat_lba_w  == EXP_FAT,  "T1 FAT_LBA == 2080");
    chk(spc_sh_w   == EXP_SH,   "T1 SPC_SHIFT == 3");
    chk(bpb_ok_w   == 1'b1,     "T1 bpb_ok");

    //======================================================================
    $display("[T2] filtering + 8.3 split");
    chk(valid_cnt == 3,         "T2 valid_cnt == 3");
    chk(slot_valid == 4'b0111,  "T2 slot_valid == 0111");
    chk(ld_type == 4'd1,        "T2 item0 type == BMP");
    chk(dut.meta_blen[0] == 4'd5, "T2 item0 blen == 5 (APPLE)");
    chk(dut.meta_elen[0] == 4'd3, "T2 item0 elen == 3 (BMP)");
    // 偏移 = DATA_LBA<<9 + (3-2)<<(9+3) = 2113536 + 4096
    chk(ld_off == 32'd2117632,  "T2 item0 offset = data_base + 4096");

    //======================================================================
    $display("[T3] offset=1 skips APPLE");
    run_scan(1);
    chk(valid_cnt == 2,         "T3 valid_cnt == 2");
    chk(ld_type == 4'd2,        "T3 first item is WAV");
    chk(dut.meta_blen[0] == 4'd8, "T3 blen == 8 (MUSIC001)");
    chk(ld_off == 32'd2121728,  "T3 item0 offset (clus=4)");

    //======================================================================
    $display("[T4] offset=2");
    run_scan(2);
    chk(valid_cnt == 1,         "T4 valid_cnt == 1");
    chk(ld_off == 32'd2125824,  "T4 item0 offset (clus=5)");
    chk(ld_type == 4'd3,        "T4 type == TXT");

    //======================================================================
    $display("[T5] offset beyond end");
    run_scan(9);
    chk(valid_cnt == 0,         "T5 valid_cnt == 0");
    chk(has_more == 1'b0,       "T5 no more");

    //======================================================================
    //  T6 superfloppy：0 号扇区自己就是 BPB（sig = "FAT32"）
    //      reserved=64, nfat=1, fatsz=512, spc=64 → data = 0+64+512 = 576,
    //      FAT = 64, shift = 6（覆盖"最多 6 次串行移位"）
    //======================================================================
    $display("[T6] superfloppy + spc=64 (variable shift)");
    init_mbr();
    mbr_mem[82] = "F"; mbr_mem[83] = "A"; mbr_mem[84] = "T";
    mbr_mem[85] = "3"; mbr_mem[86] = "2";
    mbr_mem[13] = 8'd64;
    mbr_mem[14] = 8'd64; mbr_mem[15] = 8'd0;
    mbr_mem[16] = 8'd1;
    mbr_mem[36] = 8'd0; mbr_mem[37] = 8'd2;      // fatsz = 512
    mbr_mem[38] = 8'd0; mbr_mem[39] = 8'd0;
    model_root = 32'd576;
    run_scan(0);
    $display("      root=%0d fat=%0d sh=%0d ok=%b", root_lba_w, fat_lba_w, spc_sh_w, bpb_ok_w);
    chk(root_lba_w == 32'd576,  "T6 superfloppy ROOT_LBA == 576");
    chk(fat_lba_w  == 32'd64,   "T6 FAT_LBA == 64");
    chk(spc_sh_w   == 4'd6,     "T6 SPC_SHIFT == 6");
    chk(bpb_ok_w   == 1'b1,     "T6 bpb_ok");
    // clus=3 → 576*512 + (3-2)<<(9+6) = 294912 + 32768 = 327680
    //  （MODEL 里只认 EXP_DATA，这里只校验算出来的偏移，不看数据）
    chk(ld_off == 32'd327680,   "T6 item0 offset with spc=64");

    //======================================================================
    //  T7 非法 BPB：spc=3（不是 2 的幂）⇒ 兜底常量，且必须正常结束
    //======================================================================
    $display("[T7] illegal BPB (spc=3) -> LBA 照算 + 移位量夹兜底");
    init_mbr();
    mbr_mem[82] = "F"; mbr_mem[83] = "A"; mbr_mem[84] = "T";
    mbr_mem[85] = "3"; mbr_mem[86] = "2";
    mbr_mem[13] = 8'd3;                 // 非法
    mbr_mem[14] = 8'd32; mbr_mem[15] = 8'd0;
    mbr_mem[16] = 8'd2;
    mbr_mem[36] = 8'd0; mbr_mem[37] = 8'd2;   // fatsz=512
    // ★ 2026-11-01：非法 BPB **不再**套"那张 32GB 卡"的兜底 LBA（2080/17224）。
    //   那是特定卡的布局，换卡反而更错，而且 RTL 里是两台 32 位 2:1 mux（≈32 LUT）。
    //   现在 LBA 一律用卡上 BPB 算出来的值：part(0)+reserved(32)+2×512 = 1056
    //   —— 一个真实存在的扇区，读得到东西；只有**移位量**需要夹到兜底 4
    //   （spc=3 不是 2 的幂，不夹会得到非法移位量）。
    model_root = 32'd1056;
    run_scan(0);
    $display("      root=%0d sh=%0d ok=%b valid=%0d", root_lba_w, spc_sh_w, bpb_ok_w, valid_cnt);
    chk(bpb_ok_w   == 1'b0,     "T7 bpb_ok == 0");
    chk(root_lba_w == 32'd1056, "T7 LBA 用卡上 BPB 算（不套常量）");
    chk(spc_sh_w   == 4'd4,     "T7 移位量夹到兜底 4");
    chk(bpb_ok_w   == 1'b0,     "T7 finished (no hang)");

    //======================================================================
    //  T8 images-only 模式（2026-11-01 夜）
    //     【为什么加】相册把 I_offset 当"第几张图"，文件页当"第几个条目"。
    //     IMAGES_ONLY 原来是参数，顶层只能二选一（选了 0），于是卡上混一个
    //     MUSIC.WAV 就让相册的窗口变成 [图2,图3,图4,WAV] ⇒ album_src 的
    //     win4=0 ⇒ 滑动窗口卡死，第 5 张图永远翻不到（上板症状就是"读不到"）。
    //     现在 I_images_only 由顶层动态驱动，这里锁死两条语义都成立。
    //======================================================================
    $display("[T8] images-only offset (wav must not break the album window)");
    init_mbr();
    init_bpb(8, RESERVED, NFAT, FATSZ, 0);
    model_root = EXP_DATA;
    init_dir_wav_case();

    // ---- 相册语义：只数 BMP ----
    list_all = 0;
    run_scan(0);
    chk(valid_cnt == 4,           "T8.1 album window = 4 images");
    chk(slot_valid == 4'b1111,    "T8.1 all 4 slots valid");
    chk(dut.meta_type[3] == 4'd1, "T8.1 slot3 is BMP (win4=1, can slide)");
    chk(has_more == 1'b1,         "T8.1 there is a 5th image");

    run_scan(1);
    chk(valid_cnt == 4,           "T8.2 offset=1 -> B2..B5");
    chk(dut.meta_type[3] == 4'd1, "T8.2 slot3 is B5 (not WAV)");
    chk(has_more == 1'b0,         "T8.2 last window");

    run_scan(2);
    chk(valid_cnt == 3,           "T8.3 offset=2 -> B3,B4,B5");

    // ---- 文件页语义：按条目跳，wav 仍然要列出来 ----
    list_all = 1;
    run_scan(4);
    chk(valid_cnt == 2,           "T8.4 file page offset=4 -> WAV + B5");
    chk(ld_type == 4'd2,          "T8.4 first item is still WAV");

    //======================================================================
    //  T9 match 模式（2026-11-02【音随图切】）
    //     相册先扫一遍（list_all=0）把 4 个 BMP 的主名写进 name_mem，
    //     然后按 rel 发起 match 扫描：命中"主名 8 字节相同 + .WAV"就报偏移。
    //     ★ 这一步同时验证 match 不污染列表（valid_cnt 必须 0、slot_valid 不变）。
    //======================================================================
    $display("[T9] match mode: find WAV with the same 8.3 main name");
    init_mbr();
    init_bpb(8, RESERVED, NFAT, FATSZ, 0);
    model_root = EXP_DATA;

    // ---- T9.1 WAV 在图片后面 ----
    init_dir_match_after();
    list_all = 0;
    run_scan(0);                         // 相册扫描：slot0=B1 slot1=B2
    chk(valid_cnt == 2,          "T9.1 album window has 2 images");

    run_match(2'd0);                     // 找 B1.WAV (clus=9)
    chk(hit_seen,                "T9.1 B1 -> hit");
    // 4128 + (9-2)<<3 = 4184
    chk(m_off_r == 23'd4184,     "T9.1 B1.WAV sector = 4184");
    chk(valid_cnt == 0,          "T9.1 match 不填槽（valid_cnt==0）");
    chk(slot_valid == 4'b0011,   "T9.1 列表零污染（slot_valid 不变）");

    run_match(2'd1);                     // 找 B2.WAV (clus=10)
    chk(hit_seen,                "T9.1 B2 -> hit");
    chk(m_off_r == 23'd4192,     "T9.1 B2.WAV sector = 4192");

    // ---- T9.2 没有同名 WAV ⇒ 不命中 ----
    init_dir_match_miss();
    run_scan(0);                         // slot0=B1
    run_match(2'd0);
    chk(!hit_seen,               "T9.2 无同名 WAV ⇒ 不命中");

    // ---- T9.3 WAV 排在图片**前面** ⇒ 单遍扫描也要命中 ----
    init_dir_match_before();
    run_scan(0);                         // slot0=B7（目录第 2 项才是 BMP）
    chk(valid_cnt == 1,          "T9.3 相册只数 BMP（1 张）");
    run_match(2'd0);
    chk(hit_seen,                "T9.3 WAV 在前也能命中");
    // 4128 + (11-2)<<3 = 4200
    chk(m_off_r == 23'd4200,     "T9.3 B7.WAV sector = 4200");

    // ---- T9.4 同名 TXT 干扰项在前 ⇒ 跳过它，命中后面的同名 WAV ----
    //   B1.TXT 与目标同名，但扩展名不是 WAV ⇒ mm_ok 必须为 0、不能提前收工
    init_dir_match_txt();
    run_scan(0);                         // slot0=B1.BMP
    run_match(2'd0);
    chk(hit_seen,                "T9.4 同名 TXT 不拦截，仍能找到 WAV");
    // 4128 + (12-2)<<3 = 4208
    chk(m_off_r == 23'd4208,     "T9.4 B1.WAV sector = 4208");
    // ★ 字幕：同名的 B1.TXT（clus=4）也必须命中，且偏移正确
    //   4128 + (4-2)<<3 = 4144
    chk(txt_seen,                "T9.4 B1.TXT -> hit");
    chk(txt_off_r == 23'd4144,   "T9.4 B1.TXT sector = 4144");

    //----------------------------------------------------------------------
    //  T10 名字读口：match 扫描结束后必须**还给 OSD**（[NM-STICKY] 回归）
    //----------------------------------------------------------------------
    //  旧实现 `nm_addr = mm_on ? {mm_rel,ni} : {I_rd_slot,I_rd_idx}`，而 `mm_on`
    //  只在下一次 I_start 才清 ⇒ 扫完之后读口一直被比较器占着（ni 停在 0）
    //  ⇒ 信息条整行读成同一个字符（AAAAAA.AAA）。
    //  判据：同一个 (slot,idx) 在"没有 match 在飞"和"刚扫过一次 match"两种情况下
    //  必须读回**同一个字节**。
    $display("[T10] name read port released after a match scan");
    init_dir_match_after();
    run_scan(0);                       // 一次普通扫描 ⇒ mm_on 清 0、名字就位
    rd_slot = 2'd0; rd_idx = 4'd3;
    repeat (4) @(posedge clk);
    t10_ref = rd_char;
    $display("      ref (slot0,idx3) = %02h", t10_ref);

    run_match(2'd0);                   // match 扫描；扫完 mm_on 仍是 1（旧实现的坑）
    rd_slot = 2'd0; rd_idx = 4'd3;
    repeat (4) @(posedge clk);
    if (rd_char !== t10_ref)
        $display("      got %02h expect %02h（读口被比较器地址借走了）", rd_char, t10_ref);
    chk(rd_char === t10_ref, "T10 read port released after match scan");

    //======================================================================
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
