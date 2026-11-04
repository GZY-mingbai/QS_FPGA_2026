//==============================================================================
//  tb_bmp_abort.v —— bmp_loader 的 abort 契约（小图，秒级）
//==============================================================================
//  为什么单独建台：tb_bmp_loader / tb_bmp_loader_zoom / tb_chain_img 里
//  `.abort` 全是硬接 1'b0 —— abort 这条路**从来没被仿真过**。
//
//  守的契约：
//    A1 忙的时候收到 abort ⇒ 必须上报 err（err_abort 打标）、busy 落、ready 回来
//    A2 空闲时收到 abort ⇒ 必须被忽略（下一单正常完成）
//    A3 被放弃的那一单不许影响下一单（ab_r 必须自清）
//    A4 abort 落在任何状态都要收敛（含 L_HDR / L_SEED / L_STRM / L_WROW）
//
//  跑法：iverilog -g2012 -o ba.vvp -s tb_bmp_abort tb_bmp_abort.v bmp_loader.v
//        bmp_view.v bmp_view_fill.v && vvp ba.vvp
//==============================================================================
`timescale 1ns/1ps

module tb_bmp_abort;

localparam integer IMGW = 64;
localparam integer IMGH = 64;
localparam integer PIXOFF = 54;

reg clk = 0;
always #10 clk = ~clk;
wire mclk = clk;

reg rst = 1;
reg start = 0, abort = 0;
reg [31:0] file_offset = 0, file_length = PIXOFF + IMGW * IMGH * 3;
reg [20:0] dst_base = 21'd0;
reg [1:0]  mode = 2'd2;              // 160x120 档，最小
reg        fmt  = 1'b0;              // 565
reg [10:0] dst_stride = 11'd0;
reg [7:0]  zoom = 8'h10;
reg [9:0]  ax = 10'd320, ay = 10'd240;

wire        ready, busy, done, err, err_abort;
wire [15:0] src_w_o, src_h_o;

wire        sd_sec_read;
wire [31:0] sd_sec_read_addr;
reg  [7:0]  sd_data = 0;
reg         sd_dv = 0, sd_end = 0;

wire        wr_vld;
wire [9:0]  wr_used;
wire [20:0] wr_addr;
wire [31:0] wr_data;
wire        wr_ack;
reg         wr_settled = 1'b1;

bmp_loader #(.AFIFO_AW(9)) dut (
    .clk(clk), .rst(rst), .mclk(mclk),
    .start(start), .abort(abort),
    .file_offset(file_offset), .file_length(file_length),
    .dst_base(dst_base), .mode(mode), .fmt(fmt), .dst_stride(dst_stride),
    .zoom(zoom), .ax(ax), .ay(ay),
    .ready(ready), .busy(busy), .done(done), .err(err), .err_abort(err_abort),
    .src_w_o(src_w_o), .src_h_o(src_h_o),
    .sd_sec_read(sd_sec_read), .sd_sec_read_addr(sd_sec_read_addr),
    .sd_sec_read_data(sd_data), .sd_sec_read_data_valid(sd_dv),
    .sd_sec_read_end(sd_end), .sd_init_done(1'b1),
    .wr_vld(wr_vld), .wr_used(wr_used), .wr_addr(wr_addr), .wr_data(wr_data),
    .wr_ack(wr_ack), .wr_settled(wr_settled)
);

// 写口：永远当场取走
assign wr_ack = wr_vld && !rst;
integer nwr = 0;
always @(posedge clk) if (wr_vld && wr_ack) nwr = nwr + 1;

//------------------------------------------------------------------------------
//  BMP 字节（现算，不占存储）
//------------------------------------------------------------------------------
function [7:0] enc_b;
    input integer x, y;
    begin enc_b = (x * 16 + y) % 256; end
endfunction

function [7:0] byte_at;
    input [31:0] a;
    integer off, row, col, b, y, stride;
    reg [31:0] flen;
    reg [23:0] p;
    begin
        flen   = PIXOFF + IMGW * IMGH * 3;
        stride = (IMGW * 3 + 3) & (~3);
        if (a < PIXOFF) begin
            case (a)
                32'd0:  byte_at = "B";
                32'd1:  byte_at = "M";
                32'd2:  byte_at = flen[7:0];
                32'd3:  byte_at = flen[15:8];
                32'd4:  byte_at = flen[23:16];
                32'd5:  byte_at = flen[31:24];
                32'd10: byte_at = PIXOFF[7:0];
                32'd14: byte_at = 8'd40;
                32'd18: byte_at = IMGW[7:0];
                32'd22: byte_at = IMGH[7:0];
                32'd26: byte_at = 8'd1;      // planes
                32'd28: byte_at = 8'd24;     // bpp
                default: byte_at = 8'd0;
            endcase
        end else begin
            off = a - PIXOFF;
            row = off / stride;
            col = (off % stride) / 3;
            b   = (off % stride) % 3;
            y   = IMGH - 1 - row;
            p   = {8'h40, enc_b(col, y), 8'h80};
            case (b)
                0: byte_at = p[7:0];
                1: byte_at = p[15:8];
                default: byte_at = p[23:16];
            endcase
        end
    end
endfunction

//------------------------------------------------------------------------------
//  SD 扇区从机（含 A_GAP 空隙）
//------------------------------------------------------------------------------
reg [31:0] cur_sec = 0;
reg        rd_act = 0, gap = 1'b0;
integer    bi = 0;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_act <= 0; bi <= 0; sd_dv <= 0; sd_end <= 0; sd_data <= 0; gap <= 0;
    end else begin
        sd_dv  <= 0;
        sd_end <= 0;
        if (!rd_act) begin
            gap <= 1'b0;
            if (sd_sec_read && !gap) begin
                rd_act  <= 1'b1;
                cur_sec <= sd_sec_read_addr;
                bi      <= 0;
            end
        end else begin
            sd_data <= byte_at(cur_sec * 512 + bi[31:0]);
            sd_dv   <= 1'b1;
            if (bi == 511) begin
                sd_end <= 1'b1;
                rd_act <= 1'b0;
                gap    <= 1'b1;
            end
            bi <= bi + 1;
        end
    end
end

//------------------------------------------------------------------------------
//  观测 / 判据
//------------------------------------------------------------------------------
integer fails = 0, checks = 0;
reg     err_seen, done_seen, abort_tag, ready_ok;
reg [3:0] st_at_done;

always @(posedge clk) begin
    if (err)  begin err_seen  <= 1'b1; abort_tag <= err_abort; end
    if (done) begin done_seen <= 1'b1; st_at_done <= dut.st; end
end

task chk;
    input cond;
    input [255:0] msg;
    begin
        checks = checks + 1;
        if (!cond) begin
            $display("  *** FAIL  %0s", msg);
            fails = fails + 1;
        end else
            $display("  ok       %0s", msg);
    end
endtask

task do_start;
    begin
        err_seen = 0; done_seen = 0; abort_tag = 0;
        nwr = 0;
        @(negedge clk); start = 1;
        @(negedge clk); start = 0;
    end
endtask

task wait_fin;                        // 等 done|err
    input integer tmo;
    integer n;
    begin
        n = 0;
        while (!(done_seen || err_seen) && (n < tmo)) begin
            @(posedge clk); n = n + 1;
        end
        if (!(done_seen || err_seen)) begin
            $display("  *** FAIL  timeout in wait_fin (st=%0d busy=%b rd_on=%b)",
                     dut.st, busy, dut.rd_on);
            fails = fails + 1;
        end
    end
endtask

task wait_ready;
    begin
        repeat (5) @(posedge clk);
        chk(ready, "ready returned");
    end
endtask

task pulse_abort;
    begin
        @(negedge clk); abort = 1;
        @(negedge clk); abort = 0;
    end
endtask

integer n;
initial begin
    $display("== tb_bmp_abort: abort contract ==");
    repeat (5) @(posedge clk);
    rst = 0;
    repeat (5) @(posedge clk);

    //------------------------------------------------------------------ A0 正常
    $display("  -- A0 normal load --");
    do_start;
    wait_fin(3_000_000);
    chk(done_seen && !err_seen, "A0 normal load: done and no err");
    chk(src_w_o == IMGW[15:0] && src_h_o == IMGH[15:0], "A0 src 8x8 parsed");
    wait_ready;

    //-------------------------------------------------- A1 空闲时 abort 被忽略
    $display("  -- A1 abort while idle must be ignored --");
    pulse_abort;
    do_start;
    wait_fin(3_000_000);
    chk(done_seen && !err_seen, "A1 idle abort ignored, next load ok");
    wait_ready;

    //---------------------------------------------- A2 abort 在头解析/启动附近
    $display("  -- A2 abort right after start --");
    do_start;
    repeat (4) @(negedge clk);
    pulse_abort;
    wait_fin(3_000_000);
    chk(err_seen, "A2 abort -> err");
    chk(!done_seen || (done_seen && err_seen), "A2 err reported");
    wait_ready;

    //-------------------------------------------------- A3 被放弃的不许污染下一单
    $display("  -- A3 aborted load must not poison the next --");
    do_start;
    wait_fin(3_000_000);
    chk(done_seen && !err_seen, "A3 next load clean (ab_r self-cleared)");
    wait_ready;

    //------------------------------------------------ A4 abort 落在 L_STRM
    $display("  -- A4 abort during streaming --");
    do_start;
    // 在"第一次进入 L_STRM 且还在忙"的那一拍就把 abort 拉起来（保持到收尾）。
    //   用 task 打单拍会和"这一帧正好收尾"抢时间 —— 那是台子的相位问题，不是 RTL。
    n = 0;
    abort = 0;
    while (!done_seen && !err_seen && (n < 400000)) begin
        @(negedge clk);
        if ((dut.st === 4'd6) && busy) abort = 1;
        n = n + 1;
    end
    abort = 0;
    wait_fin(1_000_000);
    chk(err_seen, "A4 streaming abort -> err");
    chk(abort_tag, "A4 err_abort tagged");
    wait_ready;

    //------------------------------------------------ A5 abort 落在 L_WROW
    //  L_WROW 等渲染器来要行；ab_r 不在该状态生效（现状），但必须仍能收敛到 err
    $display("  -- A5 abort while waiting for next row --");
    do_start;
    n = 0;
    while ((dut.st !== 4'd3) && (n < 200000)) begin @(posedge clk); n = n + 1; end
    chk(dut.st === 4'd3, "A5 reached L_WROW");
    pulse_abort;
    wait_fin(3_000_000);
    chk(err_seen, "A5 L_WROW abort -> err");
    wait_ready;

    $display("== checks=%0d fails=%0d %s ==", checks, fails, (fails == 0) ? "PASS" : "FAIL");
    $finish;
end

initial begin
    #50_000_000;
    $display("TIMEOUT");
    $display("== checks=%0d fails=%0d FAIL ==", checks, fails + 1);
    $finish;
end

endmodule
