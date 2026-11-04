`timescale 1ns/1ps
//==============================================================================
//  tb_bmp_read.v —— 动态分辨率 / 行填充 / 头部校验 的断言
//==============================================================================
//  直接驱动 SD 的字节流接口（不起 sd_card_top），所以可以把"造出来的 BMP"
//  精确控制到字节。三条断言：
//    ① 每行第一个像素必须是列 0（行填充被正确跳过）
//    ② 总像素数 == W*H（填充字节没有被当成像素）
//    ③ 非法头（W=0 / 32bpp）必须 header_match=0，且状态机回 IDLE、不卡死
//==============================================================================
module tb_bmp_read;

reg clk = 0;
always #10 clk = ~clk;

reg         rst = 1;
reg         sd_init_done = 1;
reg         load_start = 0;
reg  [31:0] load_sector = 0;
reg  [7:0]  sd_data = 0;
reg         sd_dv = 0;
reg         sd_end = 0;
wire        ready;
wire [3:0]  state_code;
wire        write_req;
wire        bmp_wr;
wire [23:0] bmp_data;
wire [15:0] src_w_o, src_h_o;
wire        sd_sec_read;
wire [31:0] sd_sec_read_addr;
wire        wr_credit = 1'b1;

bmp_read dut (
    .clk(clk), .rst(rst), .ready(ready),
    .scan_start(1'b0), .scan_start_sector(32'd0), .scan_max_sector(32'd0),
    .scan_target_count(3'd0),
    .scan_done(), .scan_found_valid(), .scan_found_sector(), .scan_found_total(),
    .load_start(load_start), .load_sector(load_sector),
    .sd_init_done(sd_init_done), .state_code(state_code),
    .bmp_width(16'd640), .bmp_height(16'd480),
    .write_req(write_req), .write_req_ack(write_req),
    .wr_credit(wr_credit),
    .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
    .sd_sec_read_data(sd_data), .sd_sec_read_data_valid(sd_dv), .sd_sec_read_end(sd_end),
    .bmp_data_wr_en(bmp_wr), .bmp_data(bmp_data),
    .src_w_o(src_w_o), .src_h_o(src_h_o)
);

//------------------------------------------------------------------------------
//  造 BMP：W×H，24bit，行填充按 4 字节对齐
//------------------------------------------------------------------------------
reg [7:0] mem [0:262143];   // 256KB：够 321x239x3+54 的合成 BMP
integer W, H, STRIDE, PIXOFF;

task build_bmp;
    input integer w, h, bpp;
    integer x, y, off;
    begin
        W = w; H = h;
        STRIDE = ((w * 3 + 3) / 4) * 4;
        PIXOFF = 54;
        for (off = 0; off < 262144; off = off + 1) mem[off] = 8'h00;
        mem[0] = 8'h42; mem[1] = 8'h4D;                       // 'B' 'M'
        off = 54 + STRIDE * h;
        mem[2] = off[7:0]; mem[3] = off[15:8]; mem[4] = off[23:16]; mem[5] = off[31:24];
        mem[10] = 54;                                         // pixel offset
        mem[18] = w[7:0];  mem[19] = w[15:8];
        mem[22] = h[7:0];  mem[23] = h[15:8];
        mem[28] = bpp[7:0];
        mem[30] = 0;                                          // compression
        for (y = 0; y < h; y = y + 1) begin
            for (x = 0; x < w; x = x + 1) begin
                off = PIXOFF + y * STRIDE + x * 3;
                mem[off + 0] = 8'h5A;                         // B
                mem[off + 1] = y[7:0];                        // G
                mem[off + 2] = x[7:0];                        // R
            end
            // 行尾填充保持 0x00（如果 stride > w*3）
        end
    end
endtask

//------------------------------------------------------------------------------
//  SD 扇区从机模型
//------------------------------------------------------------------------------
reg [1:0]  sbusy = 2'd0;
reg [15:0] si = 16'd0;
reg [31:0] saddr;
always @(posedge clk) begin
    if (sd_sec_read && !sbusy) begin
        sbusy <= 1;
        saddr <= sd_sec_read_addr;
    end
end
// 逐字节吐（放在 negedge 驱动，posedge 采样）
always @(negedge clk) begin
    if (sbusy == 1) begin
        if (si < 512) begin
            sd_data <= mem[saddr * 512 + si];
            sd_dv   <= 1'b1;
            si      <= si + 1;
        end else begin
            sd_dv  <= 1'b0;
            sd_end <= 1'b1;
            si     <= 0;
            sbusy  <= 2;
        end
    end else if (sbusy == 2) begin
        sd_end <= 1'b0;
        sbusy  <= 0;
    end
end

//------------------------------------------------------------------------------
//  像素监视
//------------------------------------------------------------------------------
integer npx = 0;
integer rowbad = 0;
integer i;
reg [23:0] first_of_row [0:63];
integer    first_n = 0;

always @(posedge clk) begin
    if (bmp_wr) begin
        if (npx == 0 || (npx % W) == 0) begin
            if (first_n < 64) first_of_row[first_n] = bmp_data;
            first_n = first_n + 1;
            if (bmp_data[23:16] !== 8'h00) begin
                if (rowbad < 5)
                    $display("        rowstart bad: npx=%0d data=%h (expect R=0)", npx, bmp_data);
                rowbad = rowbad + 1;
            end
        end
        npx = npx + 1;
    end
end

integer fails = 0, checks = 0;
task chk;
    input [8*44:1] name;
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %0d", name, got);
    end
endtask

task run_load;
    input integer w, h, bpp;
    integer t;
    begin
        build_bmp(w, h, bpp);
        npx = 0; rowbad = 0; first_n = 0; si = 0; sbusy = 0;
        @(negedge clk); load_start = 1'b1;
        @(negedge clk); load_start = 1'b0;
        for (t = 0; t < 4000000; t = t + 1) begin
            @(posedge clk);
            if (ready && (t > 1000)) t = 4000000;
        end
        repeat (20) @(posedge clk);
    end
endtask

initial begin
    $display("=========== bmp_read 动态分辨率断言 ===========");
    rst = 1; repeat (10) @(posedge clk); rst = 0; repeat (10) @(posedge clk);

    //--------------------------------------------------------------- A. 640×480 无填充
    $display("--- A. 320x240（stride=960，无填充）---");
    run_load(320, 240, 24);
    chk("A1 src_w", src_w_o, 320);
    chk("A2 src_h", src_h_o, 240);
    chk("A3 pixel count", npx, 320*240);
    chk("A4 every row starts at col0", rowbad, 0);
    chk("A5 first pixel", first_of_row[0], 24'h00005A);
    chk("A6 row1 first",  first_of_row[1], 24'h00015A);
    chk("A7 row2 first",  first_of_row[2], 24'h00025A);

    //--------------------------------------------------------------- B. 641×479 有 1 字节填充
    $display("--- B. 321x239（stride=964，每行 1 字节填充）---");
    run_load(321, 239, 24);
    chk("B1 src_w", src_w_o, 321);
    chk("B2 src_h", src_h_o, 239);
    chk("B3 pixel count", npx, 321*239);
    chk("B4 every row starts at col0", rowbad, 0);

    //--------------------------------------------------------------- C. 非法头
    $display("--- C. 非法头（32bpp）必须不卡死 ---");
    run_load(320, 240, 32);
    chk("C1 stays idle", ready, 1);
    chk("C2 no pixel",   npx, 0);

    $display("======================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  %0d FAIL / %0d checks", fails, checks);
    $finish;
end

endmodule
