//==============================================================================
//  tb_album.v —— 相册取偏移（album_src + fat_dir_lister）
//==============================================================================
//  iverilog -g2012 -o ta.vvp -s tb_album tb_album.v ../APP/album_src.v \
//      fat_dir_lister.v && vvp ta.vvp
//
//  目录： [0] A.BMP  [1] B.WAV  [2] C.BMP  [3] D.BMP   ← 中间夹一个 wav
//  断言：off0/1/2 = 三个 BMP 的偏移（跳过 wav），O_have = 3，ready = 1
//==============================================================================
`timescale 1ns/1ps

module tb_album;

localparam [31:0] PART_LBA = 32'd2048;
localparam [31:0] DATA_LBA = 32'd4128;      // 2048 + 32(保留) + 2*1024(FAT)
localparam        SPC_SH   = 3;

reg clk = 0, rst = 1;
always #10 clk = ~clk;

integer errors = 0, ncase = 0, tmo = 0;
reg  [15:0] base = 16'd0;      // 窗口起点
wire [1:0]  win;
wire        win4;
task chk;
    input cond;
    input [255:0] msg;
    begin
        ncase = ncase + 1;
        if (cond !== 1'b1) begin
            errors = errors + 1;
            $display("  FAIL [%0d] %0s", ncase, msg);
        end
    end
endtask

//------------------------------------------------------------------------------
reg [7:0] mbr_mem [0:511];
reg [7:0] bpb_mem [0:511];
reg [7:0] dir_mem [0:1023];

task init_all;
    integer i;
    begin
        for (i = 0; i < 512; i = i + 1) begin
            mbr_mem[i] = 8'h00;
            bpb_mem[i] = 8'h00;
        end
        for (i = 0; i < 1024; i = i + 1) dir_mem[i] = 8'h00;
        mbr_mem[454] = PART_LBA[7:0];
        mbr_mem[455] = PART_LBA[15:8];
        bpb_mem[13] = 8'd8;                 // spc = 8
        bpb_mem[14] = 8'd32; bpb_mem[15] = 8'd0;
        bpb_mem[16] = 8'd2;
        bpb_mem[37] = 8'h04;                // fatsz = 1024
        // [0] A.BMP clus=3
        dir_mem[0]="A"; dir_mem[1]=" "; dir_mem[8]="B"; dir_mem[9]="M"; dir_mem[10]="P";
        dir_mem[11]=8'h20; dir_mem[26]=8'd3;
        // [1] B.WAV clus=4（要被跳过）
        dir_mem[32]="B"; dir_mem[40]="W"; dir_mem[41]="A"; dir_mem[42]="V";
        dir_mem[32+11]=8'h20; dir_mem[32+26]=8'd4;
        // [2] C.BMP clus=5
        dir_mem[64]="C"; dir_mem[72]="B"; dir_mem[73]="M"; dir_mem[74]="P";
        dir_mem[64+11]=8'h20; dir_mem[64+26]=8'd5;
        // [3] D.BMP clus=6
        dir_mem[96]="D"; dir_mem[104]="B"; dir_mem[105]="M"; dir_mem[106]="P";
        dir_mem[96+11]=8'h20; dir_mem[96+26]=8'd6;
        // [4] 结束（全 0）
    end
endtask

//------------------------------------------------------------------------------
//  SD 模型
//------------------------------------------------------------------------------
reg sd_dv = 0, sd_end = 0;
reg [7:0] sd_data = 0;
wire lst_req;
wire [31:0] lst_addr;
reg reading = 0;
reg [3:0] waitc = 0;
reg [2:0] settle = 3'd0;
integer bc;
reg [1:0] src;
integer cur_base;

always @(posedge clk) begin
    sd_dv <= 0; sd_end <= 0;
    if (rst) begin reading <= 0; settle <= 0; end
    else if (!reading) begin
        if (lst_req) settle <= settle + 3'd1; else settle <= 3'd0;
        if (lst_req && settle >= 3'd2) begin
            settle <= 0;
            if (lst_addr == 0)               begin src <= 2'd0; cur_base <= 0; end
            else if (lst_addr == PART_LBA)   begin src <= 2'd1; cur_base <= 0; end
            else                             begin src <= 2'd2; cur_base <= (lst_addr - DATA_LBA)*512; end
            reading <= 1; waitc <= 2; bc <= 0;
        end
    end else begin
        if (waitc != 0) waitc <= waitc - 1;
        else if (bc < 512) begin
            sd_dv <= 1;
            sd_data <= (src == 2'd0) ? mbr_mem[bc] :
                       (src == 2'd1) ? bpb_mem[bc] : dir_mem[cur_base + bc];
            bc <= bc + 1;
        end else begin
            sd_end <= 1; reading <= 0;
        end
    end
end

//------------------------------------------------------------------------------
//  DUT
//------------------------------------------------------------------------------
wire        alb_start;
wire [15:0] alb_off_x;
wire [1:0]  alb_rd_slot;
wire [31:0] o0, o1, o2;
wire [1:0]  have;
wire        ready, sel;

wire        lst_busy, lst_done, lst_more;
wire [2:0]  lst_cnt;
wire [1:0]  ld_slot = sel ? alb_rd_slot : 2'd0;
wire [31:0] ld_off;
wire [3:0]  ld_type, slot_valid;
wire [19:0] len4;

album_src u_alb (
    .clk(clk), .rst(rst),
    .I_base(base), .I_need(1'b1), .I_block(1'b0),
    .O_lst_start(alb_start), .O_lst_offset(alb_off_x), .O_rd_slot(alb_rd_slot),
    .I_lst_busy(lst_busy), .I_lst_done(lst_done),
    .I_ld_type(ld_type), .I_slot_valid(slot_valid), .I_file_off(ld_off),
    .O_off0(o0), .O_off1(o1), .O_off2(o2),
    .O_have(have), .O_win(win), .O_win4(win4), .O_ready(ready), .O_sel(sel)
);

fat_dir_lister #(.N_ITEMS(4), .MAX_SECS(4), .INCLUDE_DIR(0)) u_lst (
    .clk(clk), .rst(rst),
    .I_start(alb_start), .I_offset(alb_off_x),
    .I_list_all_types(1'b0),          // 相册：只数 BMP
    .I_match_en(1'b0), .I_match_rel(2'd0),   // ★ match 口接 0（本台不测）
    .O_busy(lst_busy), .O_done(lst_done), .O_valid_cnt(lst_cnt), .O_has_more(lst_more),
    .I_rd_slot(2'd0), .I_rd_idx(4'd0), .O_rd_char(),
    .I_ld_slot(ld_slot), .O_ld_file_off(ld_off), .O_ld_size(),
    .O_ld_namelen(), .O_ld_blen(), .O_ld_elen(), .O_ld_len4(len4), .O_ld_vld(),
    .O_ld_type(ld_type), .O_slot_valid(slot_valid),
    .O_root_lba(), .O_fat_lba(), .O_spc_shift(), .O_bpb_ok(),
    .O_sd_req(lst_req), .O_sd_addr(lst_addr),
    .I_sd_data(sd_data), .I_sd_dv(sd_dv), .I_sd_end(sd_end)
);

function [31:0] coff;
    input integer clus;
    begin
        coff = (DATA_LBA + (clus - 2) * 8) * 512;    // spc = 8
    end
endfunction

//------------------------------------------------------------------------------
initial begin
    init_all();
    repeat (5) @(posedge clk);
    rst = 0;

    // 最多等 200us（BPB 两段 + 目录 1 个扇区）
    // ★ 不能用 fork/join 做超时：join 会等**所有**分支，超时分支必然执行到
    //   200us ⇒ 即使 ready 早就到了也会打 FAIL。这里用拍数轮询。
    tmo = 0;
    while ((ready !== 1'b1) && (tmo < 200_000)) begin
        @(posedge clk);
        tmo = tmo + 1;
    end
    if (ready !== 1'b1) $display("  FAIL: album timeout");

    repeat (5) @(posedge clk);
    $display("\n[AL] off0=%0d off1=%0d off2=%0d have=%0d ready=%b",
             o0, o1, o2, have, ready);
    chk(ready === 1'b1, "AL ready");
    chk(have === 2'd3,  "AL have 3 BMPs");
    chk(o0 === coff(3), "AL off0 = A.BMP  (clus 3)");
    chk(o1 === coff(5), "AL off1 = C.BMP  (clus 5, 跳过 WAV)");
    chk(o2 === coff(6), "AL off2 = D.BMP  (clus 6)");

    $display("\n========================================");
    if (errors == 0) $display("  ALL PASS  (%0d checks)", ncase);
    else             $display("  FAIL: %0d / %0d", errors, ncase);
    $display("========================================\n");
    $finish;
end

initial begin
    #1_000_000;
    $display("  FAIL: watchdog");
    $finish;
end

endmodule
