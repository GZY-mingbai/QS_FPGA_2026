//=============================================================================
//  beep_player.v —— 系统提示音（Flash 流式 → 48kHz 样本）
//=============================================================================
//
//  【数据在哪】Flash 0x040000 起，**裸 16bit 单声道 48kHz 小端，无文件头**：
//      click 0x040000  2400 样本（50ms）
//      ok    0x042000  5760 样本（120ms）
//      err   0x046000 19200 样本（400ms）
//  段表写在参数里（tools/gen_beep.py 按同一张表生成 bin）。
//
//  【为什么流式读】最长 5s = 480KB，片上放不下；Flash 只要 96KB/s，
//  占用率 6%，对字库几乎无感。
//
//  【跨域】50MHz 侧取数（Flash 客户口 + 打包），25MHz 侧按 48kHz 节拍取样本。
//  中间 512×16 异步 FIFO 吸收 Flash 的突发（128B = 41µs，而 64 个样本能撑 1.33ms）。
//
//  【播放节拍】直接用音乐路径的 audio_valid（wav_player 恒按 48kHz 出 valid，
//  哪怕 I_play=0 也照样出、只是样本为 0）⇒ 天然与音乐采样对齐，混音不用重采样。
//
//  【结束】取数取完 busy 落 0；FIFO 里剩下的样本照常播完 ⇒ 尾音不截断。
//
//=============================================================================
module beep_player #(
    parameter [23:0] SEG0_ADDR = 24'h040000,
    parameter [17:0] SEG0_LEN  = 18'd2400,
    parameter [23:0] SEG1_ADDR = 24'h042000,
    parameter [17:0] SEG1_LEN  = 18'd5760,
    parameter [23:0] SEG2_ADDR = 24'h046000,
    parameter [17:0] SEG2_LEN  = 18'd19200,
    parameter integer FIFO_AW  = 9                 // 512 样本
)(
    // ---- 50MHz：Flash 取数 ----
    input  wire        clk_sd,
    input  wire        rst_sd,
    input  wire        trig,            // 单拍：开始播（busy 期间忽略）
    input  wire [1:0]  seg,             // 0=click 1=ok 2=err

    output reg         fl_req,
    output wire [23:0] fl_addr,
    output wire [15:0] fl_len,
    input  wire        fl_rdy,
    input  wire [7:0]  fl_data,
    input  wire        fl_valid,
    input  wire        fl_done,

    // ---- 25MHz：播放 ----
    input  wire        clk_v,
    input  wire        rst_v,
    input  wire        smp_tick,        // 48kHz 节拍（= 音乐路径 audio_valid）
    output reg  signed [15:0] o_smp,    // 本拍样本（无样本时 0）
    output wire        o_act            // 1 = 还在出声（给混音器做旁路判断）
);

//------------------------------------------------------------------------------
//  段表
//------------------------------------------------------------------------------
reg [23:0] seg_addr;
reg [17:0] seg_len;

always @(*) begin
    case (seg)
        2'd1:    begin seg_addr = SEG1_ADDR; seg_len = SEG1_LEN; end
        2'd2:    begin seg_addr = SEG2_ADDR; seg_len = SEG2_LEN; end
        default: begin seg_addr = SEG0_ADDR; seg_len = SEG0_LEN; end
    endcase
end

//------------------------------------------------------------------------------
//  取数 FSM（50MHz）
//------------------------------------------------------------------------------
localparam [1:0] B_IDLE = 2'd0, B_REQ = 2'd1, B_RX = 2'd2;

reg [1:0]  bst;
reg        busy;
reg [23:0] baddr;
reg [17:0] bleft;
reg [17:0] bchunk;          // 本次请求还要收多少字节
reg [7:0]  bh;              // 高字节暂存
reg        bodd;
reg        fwe;
reg [15:0] fdin;

wire [FIFO_AW:0] flevel;
wire             ffull;
wire             fempty;
wire [15:0]      fdout;
wire             fpop;

// 本次要读的样本数（≤64）与字节数（≤128）—— 必须显式夹紧：
// 直接取 bleft 的低 7 位会在 bleft>127 时回绕，读多/读少都会把段内数据搞错。
wire [17:0] chunk = (bleft > 18'd64) ? 18'd64 : bleft;

assign fl_addr = baddr;
assign fl_len  = {chunk[6:0], 1'b0};      // 每次最多 128 字节 = 64 样本

always @(posedge clk_sd or posedge rst_sd) begin
    if (rst_sd) begin
        bst   <= B_IDLE;
        busy  <= 1'b0;
        baddr <= 24'd0;
        bleft <= 18'd0;
        bchunk<= 18'd0;
        bh    <= 8'd0;
        bodd  <= 1'b0;
        fwe   <= 1'b0;
        fdin  <= 16'd0;
        fl_req<= 1'b0;
    end else begin
        fwe    <= 1'b0;
        fl_req <= 1'b0;

        case (bst)
            B_IDLE: begin
                if (trig && !busy) begin
                    busy  <= 1'b1;
                    baddr <= seg_addr;
                    bleft <= seg_len;
                    bst   <= B_REQ;
                end
            end

            B_REQ: begin
                if ((bleft == 18'd0)) begin
                    busy <= 1'b0;
                    bst  <= B_IDLE;
                end else if (fl_rdy && (flevel < (512 - 80))) begin
                    fl_req <= 1'b1;
                    bchunk <= chunk;
                    bodd   <= 1'b0;
                    bst    <= B_RX;
                end
            end

            B_RX: begin
                if (fl_valid) begin
                    if (!bodd) begin
                        bh   <= fl_data;
                        bodd <= 1'b1;
                    end else begin
                        fdin <= {fl_data, bh};
                        fwe  <= 1'b1;
                        bodd <= 1'b0;
                        bchunk <= bchunk - 18'd1;
                    end
                end
                if (fl_done) begin
                    baddr <= baddr + {17'd0, chunk[6:0], 1'b0};   // 字节数
                    bleft <= bleft - chunk;
                    bst   <= B_REQ;
                end
            end

            default: bst <= B_IDLE;
        endcase
    end
end

//------------------------------------------------------------------------------
//  异步 FIFO（512 × 16）
//------------------------------------------------------------------------------
img_afifo #(.DW(16), .AW(FIFO_AW), .RCOUNT_EN(0)) u_bf (   // [AF-GATE] rcount 没人用
    .wclk   (clk_sd),
    .wrst   (rst_sd),
    .we     (fwe),
    .din    (fdin),
    .wfull  (ffull),
    .wlevel (flevel),
    .rclk   (clk_v),
    .rrst   (rst_v),
    .re     (fpop),
    .dout   (fdout),
    .rempty (fempty),
    .rcount ()
);

//------------------------------------------------------------------------------
//  播放（25MHz）：按 48kHz 节拍取样本
//------------------------------------------------------------------------------
reg act;

assign fpop = smp_tick & ~fempty;
assign o_act = act;

always @(posedge clk_v or posedge rst_v) begin
    if (rst_v) begin
        o_smp <= 16'sd0;
        act   <= 1'b0;
    end else if (smp_tick) begin
        if (!fempty) begin
            o_smp <= fdout;
            act   <= 1'b1;
        end else begin
            o_smp <= 16'sd0;
            act   <= 1'b0;
        end
    end
end

endmodule
