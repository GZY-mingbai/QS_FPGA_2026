`timescale 1ns/1ps
//==============================================================================
//  fifo_models.v —— 给 frame_read_write 用的两个厂商 FIFO 行为模型
//==============================================================================
//  厂商 IP（wfifo_32_32_512 / rfifo_32_32_512）是加密的、iverilog 跑不了，
//  所以 frame_read_write 一直没有仿真台。这两个 wrapper 用同一套行为把它顶掉：
//    端口名与厂商 IP 完全一致（clkr/clkw/rst/we/re/di/empty_flag/full_flag/
//    wrusedw/rdusedw/dout），内部就是一个标准灰度码异步 FIFO。
//  【注意】wrusedw / rdusedw 按厂商语义：10 位，表示"对面域已看到的个数"，
//    本模型直接给准确的二进制计数（比厂商保守值更乐观，但足够做功能验证）。
//==============================================================================

module fifo_async_model #(
    parameter integer DW = 32,
    parameter integer AW = 9          // 深度 = 2^AW
)(
    input  wire          clkr,        // 读侧时钟
    input  wire          clkw,        // 写侧时钟
    input  wire          rst,         // 高有效，两侧都复位
    input  wire          we,
    input  wire [DW-1:0] di,
    input  wire          re,
    output wire [DW-1:0] dout,
    output wire          empty_flag,
    output wire          full_flag,
    output wire [AW:0]   wrusedw,     // 写域水位
    output wire [AW:0]   rdusedw      // 读域可用数
);

localparam integer DEPTH = (1 << AW);

// ★ 格雷码 → 二进制。水位必须用**二进制**指针相减：
//   直接拿 wgray_s2（格雷码）减 rbin（二进制）得到的是无意义的值，
//   表现就是"FIFO 明明攒了几百个字，rdusedw 却报个小数"，
//   写引擎的 into_burst（要求 rdusedw > 128）永远不成立 ⇒ 一个突发都不发。
function [AW:0] gray2bin;
    input [AW:0] g;
    integer i;
    begin
        gray2bin[AW] = g[AW];
        for (i = AW-1; i >= 0; i = i - 1)
            gray2bin[i] = gray2bin[i+1] ^ g[i];
    end
endfunction

reg [DW-1:0] mem [0:DEPTH-1];

reg [AW:0] wbin, wgray, rbin, rgray;
reg [AW:0] wgray_s1, wgray_s2;
reg [AW:0] rgray_s1, rgray_s2;

wire        wen  = we && !full_flag;
wire        ren  = re && !empty_flag;

// ---- 写侧 ----
always @(posedge clkw or posedge rst) begin
    if (rst) begin
        wbin  <= 0;
        wgray <= 0;
        rgray_s1 <= 0;      // ★ 读指针 → 写域（full_flag / wrusedw 用）
        rgray_s2 <= 0;
    end else begin
        if (wen) begin
            mem[wbin[AW-1:0]] <= di;
            wbin  <= wbin + 1'b1;
            wgray <= ((wbin + 1'b1) >> 1) ^ (wbin + 1'b1);
        end
        rgray_s1 <= rgray;
        rgray_s2 <= rgray_s1;
    end
end

// ---- 读侧 ----
always @(posedge clkr or posedge rst) begin
    if (rst) begin
        rbin  <= 0;
        rgray <= 0;
        wgray_s1 <= 0;      // ★ 写指针 → 读域（empty_flag / rdusedw 用）
        wgray_s2 <= 0;
    end else begin
        if (ren) begin
            rbin  <= rbin + 1'b1;
            rgray <= ((rbin + 1'b1) >> 1) ^ (rbin + 1'b1);
        end
        wgray_s1 <= wgray;
        wgray_s2 <= wgray_s1;
    end
end

// ---- 标志 ----
// 满：写指针高两位与同步过来的读指针相反
assign full_flag  = (wgray == {~rgray_s2[AW:AW-1], rgray_s2[AW-2:0]});
// 空：读指针等于同步过来的写指针
assign empty_flag = (rgray == wgray_s2);
// 水位（都按二进制差给，足够做流控验证）
// ★ 本工程的仿真台三个时钟同频（clk = clkr = clkw），所以水位直接用
//   **精确二进制指针**相减。用两级同步后的 wgray_s2 会滞后 2 拍 ⇒
//   `rdusedw` 永远差 1~2 个到不了 `remaining` ⇒ 尾突发永远不发起 ⇒
//   `write_finish` 不产生（实测卡在剩 124 字）。真 IP 的水位语义不同，
//   这里只服务于"功能是否正确"的验证。
assign wrusedw    = wbin - rbin;
assign rdusedw    = wbin - rbin;

// ★ 非 show-ahead：`re` 那一拍取址，dout 在**下一拍**才有效（厂商 IP 的语义）
//   这一点必须对上：frame_read_write 的 rd_ph_d 相位、word_vld8 资格位
//   全都建立在"t 拍 re、t+1 拍 dout"这个假设上。
//   用 show-ahead 的模型会让整条 565 解包错半拍，测出来的全是假 FAIL。
reg [DW-1:0] dout_r;
always @(posedge clkr or posedge rst) begin
    if (rst)        dout_r <= {DW{1'b0}};
    else if (ren)   dout_r <= mem[rbin[AW-1:0]];
end
assign dout = dout_r;

endmodule

//==============================================================================
//  厂商 IP 同名 wrapper
//==============================================================================
module wfifo_32_32_512 (
    input  wire        clkr,
    input  wire        clkw,
    input  wire        rst,
    input  wire        we,
    input  wire        re,
    input  wire [31:0] di,
    output wire        empty_flag,
    output wire        full_flag,
    output wire [9:0]  wrusedw,
    output wire [9:0]  rdusedw,
    output wire [31:0] dout
);
fifo_async_model #(.DW(32), .AW(9)) u (
    .clkr(clkr), .clkw(clkw), .rst(rst),
    .we(we), .di(di), .re(re), .dout(dout),
    .empty_flag(empty_flag), .full_flag(full_flag),
    .wrusedw(wrusedw), .rdusedw(rdusedw)
);
endmodule

module rfifo_32_32_512 (
    input  wire        clkr,
    input  wire        clkw,
    input  wire        rst,
    input  wire        we,
    input  wire        re,
    input  wire [31:0] di,
    output wire        empty_flag,
    output wire        full_flag,
    output wire [9:0]  wrusedw,
    output wire [9:0]  rdusedw,
    output wire [31:0] dout
);
fifo_async_model #(.DW(32), .AW(9)) u (
    .clkr(clkr), .clkw(clkw), .rst(rst),
    .we(we), .di(di), .re(re), .dout(dout),
    .empty_flag(empty_flag), .full_flag(full_flag),
    .wrusedw(wrusedw), .rdusedw(rdusedw)
);
endmodule
