//==============================================================================
//  tb_fat_real.v —— 用**真卡数据**跑目录扫描器
//==============================================================================
//  iverilog -g2012 -o tfr.vvp -s tb_fat_real tb_fat_real.v fat_dir_lister.v
//  vvp tfr.vvp
//
//  【素材从哪来】sim_data/sd.hex（tools/dump_sd_fixture.py 从插入的 D: 卡导出）
//      格式：// SEG <lba> <nsec> 开一段，随后每行 16 字节十六进制
//      段：0 号扇区（MBR 或 superfloppy）+ 2048 号（BPB）+ 17224 起 16 个（根目录）
//
//  【为什么要这个台】单元测试的目录项是手搓的，簇号字段的顺序、BPB 的偏移、
//    "根目录不在 0 号扇区"这些**真卡才有的坑**它都测不到 —— 本轮就是靠这个台
//    抓到 0x14/0x1A 高低字写反（簇号 5 变成 0x50000，直接读到卡外）。
//
//  【断言】ROOT_LBA/FAT_LBA/SPC_SHIFT == 17224/2080/4；
//          第一页 4 个文件名 + 偏移；第二页 3 个；spc=64 的合成用例另外在
//          tb_fat_dir_lister.v 里（T6）。
//==============================================================================
`timescale 1ns/1ps

module tb_fat_real;

// ---- 期望值（来自 sim_data/expect.txt）----
localparam [31:0] EXP_ROOT = 32'd17224;
localparam [31:0] EXP_FAT  = 32'd2080;
localparam        EXP_SH   = 4;
localparam [31:0] EXP_DATA = 32'd17224;      // DATA_LBA == ROOT_LBA（根簇 2）
localparam integer SPC     = 16;

reg clk = 0, rst = 1;
reg start = 0;
reg [15:0] offset = 0;
wire busy, done;
wire [2:0] valid_cnt;
wire has_more;
wire [7:0] rd_char;
wire [31:0] ld_off, ld_size;
wire [3:0] ld_blen, ld_elen, ld_type, slot_valid;
wire [19:0] ld_len4;
wire ld_vld;
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
        end
    end
endtask

//==============================================================================
//  稀疏镜像：段表 + 数据池
//==============================================================================
localparam integer POOL_MAX = 32 * 1024;      // 够 16 段 × 512B
reg [7:0]  pool [0:POOL_MAX-1];
integer    seg_lba  [0:15];
integer    seg_nsec [0:15];
integer    seg_off  [0:15];
integer    nseg = 0;
integer    pool_wp = 0;

reg [8*80:1] line;
integer      fd, r, i, b;
integer      p_lba, p_nsec, pg, pcur, pin, pj;
reg [7:0]    c;

// ★ iverilog 的 $fgets 把字符串**右对齐**写进向量（最后一个字符在 bit 1），
//   而且行长随内容变化 ⇒ 必须先量出 len，再按 len 从行首取字符。
integer llen = 0;
function [7:0] ch;                     // 取 line 的第 j 个字符（0 基）
    input integer j;
    begin
        ch = line[8*(llen-j) -: 8];
    end
endfunction

function [3:0] hx;
    input [7:0] c;
    begin
        if      (c >= "0" && c <= "9") hx = c - "0";
        else if (c >= "a" && c <= "f") hx = c - "a" + 8'd10;
        else if (c >= "A" && c <= "F") hx = c - "A" + 8'd10;
        else                           hx = 4'd0;
    end
endfunction

task load_fixture;
    input [1023:0] path;
    begin
        fd = $fopen(path, "r");
        if (fd == 0) begin
            $display("  FATAL: 打不开 %0s", path);
            $finish;
        end
        nseg = 0; pool_wp = 0;
        while ($fgets(line, fd) != 0) begin
            llen = 0;
            for (i = 0; i < 80; i = i + 1)
                if (line[8*(i+1) -: 8] != 8'd0) llen = i + 1;
            if (ch(0) == "/" && ch(1) == "/") begin
                // ★ 不用 $sscanf：iverilog 对 "// SEG %d %d" 这种带注释前缀的
                //   格式串解析不出来（实测返回 0，段表全空）。手写"抓两个整数"。
                p_lba = 0; p_nsec = 0; pg = 0; pcur = 0; pin = 0;
                for (pj = 0; pj < 40; pj = pj + 1) begin
                    c = ch(pj);
                    if (c >= "0" && c <= "9") begin
                        pcur = pcur * 10 + (c - "0");
                        pin  = 1;
                    end else if (pin) begin
                        if (pg == 0) p_lba = pcur;
                        else if (pg == 1) p_nsec = pcur;
                        pg = pg + 1; pcur = 0; pin = 0;
                    end
                end
                if (pin && pg < 2) begin
                    if (pg == 0) p_lba = pcur; else p_nsec = pcur;
                    pg = pg + 1;
                end
                if (pg >= 2) begin
                    seg_lba[nseg]  = p_lba;
                    seg_nsec[nseg] = p_nsec;
                    seg_off[nseg]  = pool_wp;
                    nseg = nseg + 1;
                end
            end else begin
                // 32 个十六进制字符 = 16 字节
                for (b = 0; b < 16; b = b + 1) begin
                    if (pool_wp < POOL_MAX)
                        pool[pool_wp] = {hx(ch(2*b)), hx(ch(2*b+1))};
                    pool_wp = pool_wp + 1;
                end
            end
        end
        $fclose(fd);
        $display("  fixture: %0d 段, %0d 字节", nseg, pool_wp);
    end
endtask

// 找扇区所在段，返回池内字节偏移；找不到返回 -1
function integer seg_find;
    input integer lba;
    integer k;
    begin
        seg_find = -1;
        for (k = 0; k < 16; k = k + 1)
            if (k < nseg && lba >= seg_lba[k] && lba < seg_lba[k] + seg_nsec[k])
                seg_find = seg_off[k] + (lba - seg_lba[k]) * 512;
    end
endfunction

// 名字读口 / 列表读口由 TB 直接驱动（都是组合读）
reg  [1:0] rd_slot = 2'd0;
reg  [3:0] rd_idx  = 4'd0;
reg  [1:0] ld_slot = 2'd0;

//==============================================================================
//  DUT
//==============================================================================
fat_dir_lister #(
    .N_ITEMS(4), .MAX_SECS(16), .INCLUDE_DIR(0)
) dut (
    .clk(clk), .rst(rst),
    .I_start(start), .I_offset(offset),
    .I_list_all_types(1'b1),          // 真卡台：按住条目语义（原行为）
    .I_match_en(1'b0), .I_match_rel(2'd0),   // ★ match 口接 0（本台不测）
    .O_busy(busy), .O_done(done), .O_valid_cnt(valid_cnt), .O_has_more(has_more),
    .I_rd_slot(rd_slot), .I_rd_idx(rd_idx), .O_rd_char(rd_char),
    .I_ld_slot(ld_slot), .O_ld_file_off(ld_off), .O_ld_size(ld_size),
    .O_ld_namelen(), .O_ld_blen(ld_blen), .O_ld_elen(ld_elen),
    .O_ld_len4(ld_len4), .O_ld_vld(ld_vld), .O_ld_type(ld_type),
    .O_slot_valid(slot_valid),
    .O_root_lba(root_lba_w), .O_fat_lba(fat_lba_w),
    .O_spc_shift(spc_sh_w), .O_bpb_ok(bpb_ok_w),
    .O_sd_req(sd_req), .O_sd_addr(sd_addr),
    .I_sd_data(sd_data), .I_sd_dv(sd_dv), .I_sd_end(sd_end)
);

always #10 clk = ~clk;


//==============================================================================
//  SD 模型（与 tb_fat_dir_lister 同构：3 拍锁地址 → 512 字节 → end）
//==============================================================================
reg        reading = 0;
reg [3:0]  waitc = 0;
integer    bc;
integer    cur_base;
reg [2:0]  settle = 3'd0;
integer    miss_cnt = 0;

always @(posedge clk) begin
    sd_dv  <= 0;
    sd_end <= 0;
    if (rst) begin
        reading <= 0;
        settle  <= 3'd0;
    end else if (!reading) begin
        if (sd_req) settle <= settle + 3'd1;
        else        settle <= 3'd0;
        if (sd_req && (settle >= 3'd2)) begin
            settle   <= 3'd0;
            cur_base <= seg_find(sd_addr);
            if (seg_find(sd_addr) < 0) miss_cnt = miss_cnt + 1;
            reading  <= 1;
            waitc    <= 2;
            bc       <= 0;
        end
    end else begin
        if (waitc != 0) waitc <= waitc - 1;
        else if (bc < 512) begin
            sd_dv   <= 1;
            sd_data <= (cur_base >= 0) ? pool[cur_base + bc] : 8'h00;
            bc      <= bc + 1;
        end else begin
            sd_end  <= 1;
            reading <= 0;
        end
    end
end

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
        repeat (600) @(posedge clk);
    end
endtask

// 读一个槽的名字（8.3 + 点）到 name_buf，返回字符数
reg [8*16:1] name_buf;
task get_name;
    input integer slot;
    output integer n;
    integer k;
    reg [3:0] bl, el;
    begin
        // ★ 名字口和长度口是两个不同的选择器：名字用 I_rd_slot，
        //   长度用 I_ld_slot。只设前者会读到「上一次 ld_slot」那一格的长度
        //   （实测 MUSIC.WAV 的主名长度取了第 4 格的 8，显示成 "MUSIC   .WAV"）。
        rd_slot = slot[1:0];
        ld_slot = slot[1:0];
        #1;
        bl = ld_blen;
        el = ld_elen;
        n  = 0;
        for (k = 0; k < 16; k = k + 1) name_buf[8*(16-k) -: 8] = " ";
        for (k = 0; k < 8; k = k + 1) begin
            if (k < bl) begin
                rd_idx = k[3:0];
                #1;
                name_buf[8*(16-n) -: 8] = rd_char;
                n = n + 1;
            end
        end
        if (el != 0) begin
            name_buf[8*(16-n) -: 8] = ".";
            n = n + 1;
            for (k = 0; k < 3; k = k + 1) begin
                if (k < el) begin
                    rd_idx = 4'd8 + k[3:0];
                    #1;
                    name_buf[8*(16-n) -: 8] = rd_char;
                    n = n + 1;
                end
            end
        end
    end
endtask

function [31:0] clus_off;
    input integer clus;
    begin
        clus_off = (EXP_DATA + (clus - 2) * SPC) * 512;
    end
endfunction

integer n0, n1, n2, n3, n4;
reg [8*16:1] s0, s1, s2, s3;

//------------------------------------------------------------------------------
initial begin
    // 执行目录是 SD/，素材在仓库根的 sim_data/
    load_fixture("../../../../sim_data/sd.hex");

    repeat (5) @(posedge clk);
    rst = 0;
    repeat (3) @(posedge clk);

    //======================================================================
    $display("\n[R1] real card: BPB -> ROOT/FAT/SPC");
    run_scan(0);
    $display("      root=%0d fat=%0d sh=%0d ok=%b vcnt=%0d more=%b miss=%0d",
             root_lba_w, fat_lba_w, spc_sh_w, bpb_ok_w, valid_cnt, has_more, miss_cnt);
    chk(root_lba_w == EXP_ROOT, "R1 ROOT_LBA == 17224");
    chk(fat_lba_w  == EXP_FAT,  "R1 FAT_LBA  == 2080");
    chk(spc_sh_w   == EXP_SH,   "R1 SPC_SHIFT == 4");
    chk(bpb_ok_w   == 1'b1,     "R1 bpb_ok");
    chk(miss_cnt   == 0,        "R1 no read outside fixture");

    //======================================================================
    $display("[R2] page 1: 4 files");
    chk(valid_cnt == 3'd4,      "R2 valid_cnt == 4");
    chk(has_more  == 1'b1,      "R2 has_more");
    chk(slot_valid == 4'b1111,  "R2 slot_valid == 1111");

    get_name(0, n0); s0 = name_buf;
    get_name(1, n1); s1 = name_buf;
    get_name(2, n2); s2 = name_buf;
    get_name(3, n3); s3 = name_buf;
    $display("      [%0d] %0s", n0, s0);
    $display("      [%0d] %0s", n1, s1);
    $display("      [%0d] %0s", n2, s2);
    $display("      [%0d] %0s", n3, s3);
    chk(s0 == "_640X4~1.BMP    ", "R2 name0");
    chk(s1 == "_640X4~2.BMP    ", "R2 name1");
    chk(s2 == "LOGO_6~1.BMP    ", "R2 name2");
    chk(s3 == "_640X4~3.BMP    ", "R2 name3");

    // 偏移按簇号算：5 / 118 / 231 / 344
    ld_slot = 2'd0; #1; chk(ld_off == clus_off(5),   "R2 off0 (clus 5)");
    ld_slot = 2'd1; #1; chk(ld_off == clus_off(118), "R2 off1 (clus 118)");
    ld_slot = 2'd2; #1; chk(ld_off == clus_off(231), "R2 off2 (clus 231)");
    ld_slot = 2'd3; #1; chk(ld_off == clus_off(344), "R2 off3 (clus 344)");

    // 类型：前 4 个都是 BMP
    ld_slot = 2'd0; #1; chk(ld_type == 4'd1, "R2 type0 BMP");
    ld_slot = 2'd3; #1; chk(ld_type == 4'd1, "R2 type3 BMP");

    //======================================================================
    $display("[R3] page 2: offset=4");
    run_scan(4);
    $display("      vcnt=%0d more=%b", valid_cnt, has_more);
    chk(valid_cnt == 3'd3,      "R3 valid_cnt == 3");
    chk(has_more  == 1'b0,      "R3 no more");
    get_name(0, n0); s0 = name_buf;
    get_name(1, n1); s1 = name_buf;
    get_name(2, n2); s2 = name_buf;
    $display("      [%0d] %0s  [%0d] %0s  [%0d] %0s", n0, s0, n1, s1, n2, s2);
    chk(s0 == "GRID_R~1.BMP    ", "R3 name0");
    chk(s1 == "GRID_B~1.BMP    ", "R3 name1");
    chk(s2 == "MUSIC.WAV       ", "R3 name2");
    ld_slot = 2'd0; #1; chk(ld_off == clus_off(458), "R3 off0 (clus 458)");
    ld_slot = 2'd2; #1; chk(ld_off == clus_off(684), "R3 off2 (clus 684)");
    chk(ld_type == 4'd2, "R3 type2 == WAV");

    //======================================================================
    $display("\n========================================");
    if (errors == 0) $display("  ALL PASS  (%0d checks)", ncase);
    else             $display("  FAIL: %0d / %0d", errors, ncase);
    $display("========================================\n");
    $finish;
end

initial begin
    #40_000_000;
    $display("  FAIL: watchdog");
    $finish;
end

endmodule
