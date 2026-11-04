`timescale 1ns/1ps
//==============================================================================
//  tb_fc_reload.v —— osd_font_cache 重装通路专项（★ 2026-10-29 [S4]）
//==============================================================================
//  新结构没有槽表/扫描/owner，"一行装完之后每一格都对"就是全部正确性。
//  本台直接驱动配置口 + 一个假 Flash 客户口，逐格核对 16 行：
//    ① 非空白字符：16 行必须等于"该字符码对应的 Flash 地址"读出来的 32 字节
//       （假 Flash 的内容 = (请求地址 + 字节序号) & 0xFF ⇒ 地址错/行错位/
//        左右字节反 都会当场显形）
//    ② 空白字符（码 0）：16 行必须全 0
//    ③ line_ready 只在装完之后才为 1，且重装期间为 0
//  用法：工作目录 = APP/OSD
//      iverilog -g2012 -DIVERILOG_SIM -s tb_fc_reload -o /tmp/fcrel.vvp \
//          tb_fc_reload.v ../osd_font_cache.v ../gb_addr.v && vvp /tmp/fcrel.vvp
//==============================================================================
module tb_fc_reload;

localparam integer NL     = 8;
localparam integer MAXLEN = 32;

reg clk = 0;
always #5 clk = ~clk;

reg rst = 1;

reg         str_we = 0;
reg  [3:0]  str_line = 0;
reg  [7:0]  str_idx = 0;
reg  [15:0] str_code = 0;
reg         ctl_we = 0;
reg  [3:0]  ctl_line = 0;
reg         ctl_active = 0;
reg  [7:0]  ctl_len = 0;
reg  [7:0]  ctl_ver = 0;
reg         trigger = 0;

wire [10*NL-1:0] slot_rd_data;

reg  [13:0] glyph_rd_addr = 0;
wire [15:0] glyph_rd_row;
wire [NL-1:0]   line_ready;
wire [NL-1:0]   line_overflow;
wire [8*NL-1:0] line_len;

wire        fl_req;
wire [23:0] fl_addr;
wire [15:0] fl_len;
reg         fl_rdy = 1;
wire [7:0]  fl_data;
reg         fl_valid = 0;
reg         fl_done = 0;

osd_font_cache #(
    .NL(NL), .MAXLEN(MAXLEN), .NSLOT(63), .SCLK_DIV(1), .POR_CYCLES(200000)
) dut (
    .clk(clk), .rst(rst),
    .str_we(str_we), .str_line(str_line), .str_idx(str_idx), .str_code(str_code),
    .ctl_we(ctl_we), .ctl_line(ctl_line), .ctl_active(ctl_active),
    .ctl_len(ctl_len), .ctl_ver(ctl_ver),
    .trigger(trigger),
    .slot_rd_col({8*NL{1'b0}}),
    .slot_rd_data(slot_rd_data),
    .glyph_rd_addr(glyph_rd_addr), .glyph_rd_row(glyph_rd_row),
    .line_ready(line_ready), .line_overflow(line_overflow), .line_len(line_len),
    .fl_req(fl_req), .fl_addr(fl_addr), .fl_len(fl_len),
    .fl_rdy(fl_rdy), .fl_data(fl_data), .fl_valid(fl_valid), .fl_done(fl_done)
);

//------------------------------------------------------------------------------
//  假 Flash：收到请求后连续 32 拍吐字节，内容 = (请求地址 + 字节序号) & 0xFF
//------------------------------------------------------------------------------
reg [23:0] cap_addr = 24'd0;
reg [7:0]  si = 8'd0;

assign fl_data = (cap_addr + {16'd0, si}) & 24'h0000FF;

always @(posedge clk) begin
    if (rst) begin
        fl_valid <= 1'b0;
        fl_done  <= 1'b0;
        si       <= 8'd0;
    end else if (fl_req && fl_rdy && !fl_valid) begin
        cap_addr <= fl_addr;
        si       <= 8'd0;
        fl_valid <= 1'b1;
        fl_done  <= 1'b0;
    end else if (fl_valid) begin
        if (si == 8'd31) begin
            fl_valid <= 1'b0;
            fl_done  <= 1'b1;
        end else begin
            si <= si + 8'd1;
        end
    end else begin
        fl_done <= 1'b0;
    end
end

//------------------------------------------------------------------------------
integer checks = 0, fails = 0;
task chk;
    input [8*64:1] name;
    input integer  got;
    input integer  exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            $display("  *** FAIL  %0s : got %0h expect %0h", name, got, exp);
            fails = fails + 1;
        end
    end
endtask

task rd_row;
    input [2:0]  line;
    input [4:0]  col;
    input [3:0]  row;
    begin
        glyph_rd_addr = {2'b00, line, col, row};
        @(negedge clk);
        @(negedge clk);          // 同步读：地址后 1 拍
    end
endtask

task wr_str;
    input [3:0]  line;
    input [7:0]  idx;
    input [15:0] code;
    begin
        @(negedge clk);
        str_we = 1'b1; str_line = line; str_idx = idx; str_code = code;
        @(negedge clk);
        str_we = 1'b0;
    end
endtask

task set_ctl;
    input [3:0] line;
    input       act;
    input [7:0] len;
    input [7:0] ver;
    begin
        @(negedge clk);
        ctl_we = 1'b1; ctl_line = line; ctl_active = act; ctl_len = len; ctl_ver = ver;
        @(negedge clk);
        ctl_we = 1'b0;
    end
endtask

// 独立算 GB2312 → Flash 偏移（**不调用 RTL 的 gb_addr**）
function [23:0] gb_off;
    input [15:0] code;
    integer qb, wb, rowi;
    begin
        if (code[15:8] == 8'h00) begin
            gb_off = {8'd0, code} * 32;
        end else begin
            qb = code[15:8];
            wb = code[7:0];
            if (qb <= 8'hA9) rowi = qb - 8'hA1;
            else             rowi = qb - 8'hB0 + 9;
            gb_off = 24'h001000 + (rowi * 94 + (wb - 8'hA1)) * 32;
        end
    end
endfunction

integer i, k, n;
reg [15:0] codes [0:5];
reg [23:0] a24;
reg [15:0] want;
reg [7:0]  b0, b1;

initial begin
    rst = 1;
    repeat (4) @(negedge clk);
    rst = 0;
    @(negedge clk);

    codes[0] = 16'hB5CD;   // 低
    codes[1] = 16'hD6D0;   // 中
    codes[2] = 16'hB8DF;   // 高
    codes[3] = 16'h0000;   // 空白
    codes[4] = 16'h0041;   // 'A'
    codes[5] = 16'h0031;   // '1'

    set_ctl(4'd0, 1'b1, 8'd6, 8'd1);
    for (i = 0; i < 6; i = i + 1) wr_str(4'd0, i[7:0], codes[i]);
    @(negedge clk); trigger = 1'b1; @(negedge clk); trigger = 1'b0;

    // 重装期间必须为 0
    n = 0;
    while (!line_ready[0] && (n < 40000)) begin
        @(negedge clk); n = n + 1;
    end
    $display("  line_ready[0]=%0d after %0d cycles", line_ready[0], n);
    chk("ready bit0", line_ready[0], 1);
    chk("len0", line_len[7:0], 6);
    chk("ovf0", line_overflow[0], 0);

    for (k = 0; k < 6; k = k + 1) begin
        if (codes[k] == 16'h0000) begin
            for (i = 0; i < 16; i = i + 1) begin
                rd_row(3'd0, k[4:0], i[3:0]);
                chk("blank row", glyph_rd_row, 16'h0000);
            end
        end else begin
            a24 = gb_off(codes[k]);
            for (i = 0; i < 16; i = i + 1) begin
                rd_row(3'd0, k[4:0], i[3:0]);
                b0 = (a24 + 2*i);
                b1 = (a24 + 2*i + 1);
                want = {b0, b1};
                chk("glyph row", glyph_rd_row, want);
            end
        end
    end

    //------------------------------------------------------------------
    //  第二行：只写 2 个字，确认别的格没被串味
    //------------------------------------------------------------------
    set_ctl(4'd1, 1'b1, 8'd2, 8'd3);
    wr_str(4'd1, 8'd0, 16'hCAC0);   // 世
    wr_str(4'd1, 8'd1, 16'hBDE7);   // 界
    @(negedge clk); trigger = 1'b1; @(negedge clk); trigger = 1'b0;
    n = 0;
    while (!line_ready[1] && (n < 40000)) begin
        @(negedge clk); n = n + 1;
    end
    chk("ready bit1", line_ready[1], 1);
    a24 = gb_off(16'hCAC0);
    for (i = 0; i < 16; i = i + 1) begin
        rd_row(3'd1, 5'd0, i[3:0]);
        b0 = (a24 + 2*i);
        b1 = (a24 + 2*i + 1);
        want = {b0, b1};
        chk("L1 col0 row", glyph_rd_row, want);
    end

    //------------------------------------------------------------------
    //  ★ 2026-10-29 [S7]：字形读口延迟必须**恰好 1 拍**，仿真/综合两套
    //    分支必须一致。
    //    S4 重写读口时在 BRAM 的 dob 后又打了一拍 ⇒ 综合出来 2 拍、仿真 1 拍。
    //    地址 = {行, 字格列, 字形行} 在一个字格的 16 个像素内**不变**，
    //    晚一拍的后果是"字格左边界前 2 个像素取到上一个字格的数据"
    //    ⇒ 上板现象就是"文字里随机冒出静态白点，且跟着文字走"。
    //    出图测试抓不到，因为出图走的是另一条（1 拍的）分支。
    //    这条断言防它复发：地址变后**一拍**就必须出数，早了晚了都算错。
    //------------------------------------------------------------------
    glyph_rd_addr = {2'b00, 3'd1, 5'd0, 4'd0};        // 稳定在"世 row0"
    @(negedge clk); @(negedge clk);
    a24 = gb_off(16'hBDE7);
    b0  = a24;  b1 = a24 + 1;
    want = {b0, b1};
    glyph_rd_addr = {2'b00, 3'd1, 5'd1, 4'd0};        // 切到"界 row0"
    @(negedge clk);                                    // 只等 1 拍
    chk("read latency exactly 1 cycle", glyph_rd_row, want);

    $display("======================================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  %0d FAIL out of %0d checks", fails, checks);
    $display("======================================================");
    $finish;
end

endmodule
