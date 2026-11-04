`timescale 1ns/1ps
//==============================================================================
//  tb_bmp_read640.v —— bmp_read 在 640x480 源上的像素流是否逐点精确
//==============================================================================
//  【为什么要这个台】
//    tb_bmp_read 里 mem 只有 256KB，装不下 640×480 的 24bit BMP（≈900KB），
//    所以 640×480 这条路**从来没被 bmp_read 测过**；而 tb_bmp_scale 的源是
//    台内自己生成的 model，绕过了 bmp_read ⇒ "bmp_read 的像素流是否从源列 0
//    开始"这一环是空白。缩放改造把 header_match 从 width==640 放宽到 1..4096，
//    行首相位/行尾填充是新写的代码，必须单独钉死。
//
//  【做法】SD 卡扇区源改成"按绝对地址现算"，不占存储 ⇒ 任意尺寸都能测。
//    源图 = 坐标编码图：像素(x,y) = { x>>2, y>>2, x[1:0], y[1:0], 4'h0 }
//    ⇒ 24 位像素里带着自己的 (x,y)，任何位移/串行都立刻暴露。
//==============================================================================
module tb_bmp_read640;

reg clk = 0;
always #10 clk = ~clk;

reg rst = 1;
reg load_start = 0;
reg [31:0] load_sector = 0;
reg sd_init_done = 1;
reg wr_credit = 1;
reg bmp_ack = 1;                  // write_req_ack 恒 1（本台只关心像素流）

wire        ready;
wire [3:0]  state_code;
wire        write_req;
wire        sd_sec_read;
wire [31:0] sd_sec_read_addr;
reg  [7:0]  sd_data = 0;
reg         sd_dv = 0;
reg         sd_end = 0;
wire        bmp_wr;
wire [23:0] bmp_data;
wire [15:0] src_w_o, src_h_o;

bmp_read dut (
    .clk(clk), .rst(rst), .ready(ready),
    .scan_start(1'b0), .scan_start_sector(32'd0), .scan_max_sector(32'd0),
    .scan_target_count(3'd1), .scan_done(), .scan_found_valid(),
    .scan_found_sector(), .scan_found_total(),
    .load_start(load_start), .load_sector(load_sector),
    .sd_init_done(sd_init_done), .state_code(state_code),
    .bmp_width(16'd640), .bmp_height(16'd480),
    .write_req(write_req), .write_req_ack(bmp_ack),
    .wr_credit(wr_credit),
    .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
    .sd_sec_read_data(sd_data), .sd_sec_read_data_valid(sd_dv),
    .sd_sec_read_end(sd_end),
    .bmp_data_wr_en(bmp_wr), .bmp_data(bmp_data),
    .src_w_o(src_w_o), .src_h_o(src_h_o)
);

//------------------------------------------------------------------------------
//  源图定义（640x480，坐标编码）
//------------------------------------------------------------------------------
integer SW = 640, SH = 480;
integer STRIDE;
initial STRIDE = ((640*3 + 3) / 4) * 4;     // 1920

function [7:0] hdr_byte;
    input [31:0] a;
    integer flen;
    begin
        flen = 54 + 1920 * 480;
        case (a)
            32'd0 : hdr_byte = 8'h42;               // 'B'
            32'd1 : hdr_byte = 8'h4D;               // 'M'
            32'd2 : hdr_byte = flen[7:0];
            32'd3 : hdr_byte = flen[15:8];
            32'd4 : hdr_byte = flen[23:16];
            32'd5 : hdr_byte = flen[31:24];
            32'd10: hdr_byte = 8'd54;
            32'd18: hdr_byte = 8'h80;               // 640 = 0x0280
            32'd19: hdr_byte = 8'h02;
            32'd22: hdr_byte = 8'hE0;               // 480 = 0x01E0
            32'd23: hdr_byte = 8'h01;
            32'd28: hdr_byte = 8'd24;
            32'd30: hdr_byte = 8'd0;
            default: hdr_byte = 8'd0;
        endcase
    end
endfunction

// 像素(x,y) = { x>>2, y>>2, x[1:0], y[1:0], 4'h0 }
function [7:0] pix_byte;
    input integer x, y, b;
    begin
        case (b)
            0: pix_byte = 8'h00;                     // B 分量 = {x[1:0],y[1:0],4'h0}
            1: pix_byte = (y >> 2);                  // G
            default: pix_byte = (x >> 2);            // R
        endcase
    end
endfunction

function [7:0] byte_at;
    input [31:0] a;
    integer off, yy, xx, bb, r;
    begin
        if (a < 54) byte_at = hdr_byte(a);
        else begin
            off = a - 54;
            yy  = off / STRIDE;
            r   = off % STRIDE;
            if ((yy >= SH) || (r >= SW*3)) byte_at = 8'h00;
            else begin
                xx = r / 3;
                bb = r % 3;
                if (bb == 0) byte_at = {xx[1:0], yy[1:0], 4'h0};
                else         byte_at = pix_byte(xx, yy, bb);
            end
        end
    end
endfunction

// 【坑】不能写 {x[9:2], y[8:2], ...}：integer 表达式在拼接里按 32 位自决定宽度，
//   整体会被截断错位（实测 x=4 得到 0x008000 而不是 0x010000）。
//   必须先在函数内落成定宽 reg 再拼。
function [23:0] srcpix;
    input integer x, y;
    reg [7:0] xh, yh;
    reg [1:0] xl, yl;
    begin
        xh = x >> 2;  yh = y >> 2;
        xl = x & 3;   yl = y & 3;
        srcpix = {xh, yh, xl, yl, 4'h0};
    end
endfunction

//------------------------------------------------------------------------------
//  SD 扇区从机
//------------------------------------------------------------------------------
reg [1:0]  sbusy = 0;
reg [15:0] si = 0;
reg [31:0] saddr = 0;
always @(posedge clk) begin
    if (sd_sec_read && (sbusy == 2'd0)) begin
        sbusy <= 2'd1;
        saddr <= sd_sec_read_addr;
    end
end
always @(negedge clk) begin
    if (sbusy == 2'd1) begin
        if (si < 512) begin
            sd_data <= byte_at(saddr * 512 + si);
            sd_dv   <= 1'b1;
            si      <= si + 1;
        end else begin
            sd_dv  <= 1'b0;
            sd_end <= 1'b1;
            si     <= 0;
            sbusy  <= 2'd2;
        end
    end else if (sbusy == 2'd2) begin
        sd_end <= 1'b0;
        sbusy  <= 2'd0;
    end
end

//------------------------------------------------------------------------------
//  像素捕获
//------------------------------------------------------------------------------
reg [23:0] pcap [0:307199];
integer    npx;
integer    k;

always @(posedge clk) begin
    if (bmp_wr) begin
        if (npx < 307200) pcap[npx] = bmp_data;
        npx = npx + 1;
    end
end

integer fails = 0, checks = 0;
task chk;
    input [8*52:1] name;
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0d, expect %0d", name, got, exp);
            fails = fails + 1;
        end else
            $display("   ok       %0s = %0d", name, got);
        $fflush;
    end
endtask

// 画布第 k 个像素的内容，说的是源的哪一点？
function integer dec_x; input [23:0] c; begin dec_x = {c[23:16], c[7:6]}; end endfunction
function integer dec_y; input [23:0] c; begin dec_y = {c[15:8], c[5:4]}; end endfunction

integer bad_pix;
integer bh_n, bh_bad;
task meas_h;
    input integer y;
    integer N, x, bad;
    begin
        bh_n = 0; bh_bad = 999999;
        for (N = -16; N <= 16; N = N + 1) begin
            bad = 0;
            for (x = 20; x < 620; x = x + 1)
                if (pcap[y*640 + x] !== srcpix(x - N, y)) bad = bad + 1;
            if (bad < bh_bad) begin bh_bad = bad; bh_n = N; end
        end
        $display("      row %0d : 像素流相对源**右移 %0d 列**  (不匹配 %0d/600)",
                 y, bh_n, bh_bad);
    end
endtask

initial begin
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (10) @(posedge clk);

    $display("");
    $display("==================================================================");
    $display("  bmp_read：640x480 源，像素流逐点比对（307200 点）");
    $display("==================================================================");

    npx = 0;
    @(negedge clk); load_start = 1'b1;
    @(negedge clk); load_start = 1'b0;

    begin : waitdone
        integer t;
        for (t = 0; t < 12000000; t = t + 1) begin
            @(posedge clk);
            if (ready && (t > 2000)) t = 12000000;
        end
    end
    repeat (50) @(posedge clk);

    $display("  输出像素数 = %0d  (应为 307200)", npx);
    chk("D0 像素数", npx, 307200);
    chk("D1 src_w",  src_w_o, 640);
    chk("D2 src_h",  src_h_o, 480);

    bad_pix = 0;
    for (k = 0; k < 307200; k = k + 1)
        if (pcap[k] !== srcpix(k % 640, k / 640)) begin
            if (bad_pix < 6)
                $display("        [%0d] (%0d,%0d) got=%h exp=%h",
                         k, k % 640, k / 640, pcap[k], srcpix(k % 640, k / 640));
            bad_pix = bad_pix + 1;
        end
    chk("D3 逐点精确", bad_pix, 0);

    meas_h(0); meas_h(120); meas_h(240); meas_h(479);

    $display("");
    $display("  前 8 个像素解出的源坐标：");
    for (k = 0; k < 8; k = k + 1)
        $display("    [%0d] = %h ⇒ 源(%0d, %0d)", k, pcap[k], dec_x(pcap[k]), dec_y(pcap[k]));

    $display("");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAILED: %0d / %0d checks", fails, checks);
    $display("==================================================================");
    $finish;
end

endmodule
