//==============================================================================
//  bmp_view_fill.v —— 一个源行的横向重采样（写进行缓冲，越界留 0）
//==============================================================================
//  【输入】一行源像素（win_w 个，源列 x_le .. x_le+win_w-1，按 x 递增到达）
//  【输出】lbuf[0..cw-1]：已缩放到画布宽度的这一行；越界/窗口外保持 0（黑）
//          ox_lo/ox_hi：内容列范围 [ox_lo, ox_hi)，供冲刷判"内容字 vs 黑边字"
//
//  【映射（与 bmp_scale 同约定：floor）】
//      输出列 ox  ←  源列 floor(sx0 + ox*step)
//    step = 1024 时是 1:1；>1024 缩小（部分源列被跳过）；<1024 放大（源像素重复）
//
//  【为什么单独一个模块】
//    BRAM 的输出 `? :` 是 TD 放弃块 RAM 的经典原因（sd_bmp_loader 的 bf_fifo
//    踩过：256×24 被推成 384 LUT 的分布式 RAM）。所以这里 RAM 读口只有
//    "地址 → 输出寄存器"一句话，所有选择都在外面做。
//==============================================================================
module bmp_view_fill (
    input  wire        clk,
    input  wire        rst,

    input  wire        start,           // 单拍：开始填这一行（下面参数已稳定）
    input  wire signed [26:0] sx0,      // Q10 源起点（可为负）
    input  wire [16:0] step,            // Q10 步进（17 位：支持缩小到 0.25×）
    input  wire [11:0] x_le,            // 本行第一个像素的源列
    input  wire [12:0] win_w,           // 本行像素数
    input  wire [10:0] cw,              // 画布宽（列数）
    input  wire        avg2,            // 1 = 精确 2:1 横向下采样，做逐通道平均

    input  wire        pix_vld,
    input  wire [23:0] pix,
    output wire        pix_rd,

    output reg  [10:0] ox_lo,
    output reg  [10:0] ox_hi,
    output wire        busy,
    output reg         done,

    // 行缓冲写口（RAM 实体在父模块，写读各占一个 always ⇒ 推断成简单双口块 RAM）
    output reg         lb_we,
    output reg  [10:0] lb_wad,
    output reg  [23:0] lb_din,

    // 行缓冲读口（与写口同在一个模块 ⇒ TD 才会推成块 RAM）
    input  wire [10:0] rd_addr,
    input  wire        rd_en,
    output reg  [23:0] rd_q
);

localparam [1:0] F_IDLE = 2'd0, F_CLR = 2'd1, F_FILL = 2'd2, F_END = 2'd3;

reg [1:0]  fs;
(* ram_style = "block" *) (* ramstyle = "block" *) (* syn_ramstyle = "block" *)
reg [23:0] lbuf [0:639];

// ★ 写口 + 读口必须在同一个 always 里、读地址走本地寄存器：
//   这正是 bmp_scale 的 lbuf / sd_bmp_loader 的 bf_fifo 能推成块 RAM 的形态。
//   读延迟因此 = 2 拍（地址 1 拍 + 输出 1 拍），父模块的 cd 相应多打一拍。
reg [10:0] rd_a;
// ★ [S8] 地址寄存和输出寄存必须**一起**受 rd_en 门控：
//   只冻下游（输出）不冻上游（地址），一次 2 拍反压就会让数据比列号超前 2 列。
always @(posedge clk) begin
    if (lb_we) lbuf[lb_wad] <= lb_din;
    if (rd_en) begin
        rd_a <= rd_addr;
        rd_q <= lbuf[rd_a];
    end
end

reg [10:0] clr_i;
reg [23:0] pix_hold;            // avg2：暂存偶数源列
reg [12:0] fill_i;              // 已消费的源像素数
reg [10:0] ox_w;                // 行缓冲写指针
reg signed [26:0] sx_acc;       // 当前输出列对应的源 x（Q10）

// 本像素的右边界（源列 x_le+fill_i 的下一列）
wire signed [26:0] c_end = ({1'b0, x_le} + {1'b0, fill_i} + 27'sd1) <<< 10;

// 这个源像素还要覆盖下一个输出列？
wire col_pending = (ox_w < cw) && (sx_acc < c_end);
// 该列已经在图外（左边）⇒ 保持 0，不写
wire col_valid   = (sx_acc >= 0);

// ★ 精确 2:1 必须逐通道平均（整字相加会进位串色，pitfall #21）
wire [8:0] avg_r = {1'b0, pix_hold[23:16]} + {1'b0, pix[23:16]};
wire [8:0] avg_g = {1'b0, pix_hold[15:8]}  + {1'b0, pix[15:8]};
wire [8:0] avg_b = {1'b0, pix_hold[7:0]}   + {1'b0, pix[7:0]};
wire [23:0] pix_avg = {avg_r[8:1], avg_g[8:1], avg_b[8:1]};

assign pix_rd = (fs == F_FILL) && pix_vld && (fill_i != win_w) &&
                (avg2 ? 1'b1 : !col_pending);
assign busy   = (fs != F_IDLE);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        fs <= F_IDLE; clr_i <= 11'd0; fill_i <= 13'd0; ox_w <= 11'd0;
        sx_acc <= 27'sd0; ox_lo <= 11'd0; ox_hi <= 11'd0;
        lb_din <= 24'd0; lb_wad <= 11'd0; lb_we <= 1'b0; done <= 1'b0;
    end else begin
        lb_we  <= 1'b0;
        done   <= 1'b0;

        case (fs)
            F_IDLE: begin
                if (start) begin
                    clr_i  <= 11'd0;
                    fill_i <= 13'd0;
                    ox_w   <= 11'd0;
                    sx_acc <= sx0;
                    ox_lo  <= 11'd0;
                    ox_hi  <= 11'd0;
                    fs     <= F_CLR;
                end
            end

            // 先整行清 0：黑边、纵向越界、窗口右侧都用它兜住
            F_CLR: begin
                lb_we  <= 1'b1;
                lb_wad <= clr_i;
                lb_din <= 24'd0;
                if (clr_i == (cw - 11'd1)) begin
                    clr_i <= 11'd0;
                    fs    <= (win_w == 13'd0) ? F_END : F_FILL;
                end else
                    clr_i <= clr_i + 11'd1;
            end

            F_FILL: begin
                if (pix_vld && avg2) begin
                    // 2:1：两个源像素 → 一个输出列（逐通道平均）
                    if (fill_i[0] == 1'b0) begin
                        pix_hold <= pix;
                        fill_i   <= fill_i + 13'd1;
                    end else begin
                        lb_we  <= 1'b1;
                        lb_wad <= ox_w;
                        lb_din <= pix_avg;
                        if ((ox_lo == 11'd0) && (ox_hi == 11'd0)) ox_lo <= ox_w;
                        ox_hi <= ox_w + 11'd1;
                        ox_w  <= ox_w + 11'd1;
                        fill_i <= fill_i + 13'd1;
                        if ((fill_i + 13'd1) >= win_w) fs <= F_END;
                    end
                end else if (pix_vld) begin
                    if (col_pending) begin
                        if (col_valid) begin
                            lb_we  <= 1'b1;
                            lb_wad <= ox_w;
                            lb_din <= pix;
                            // 第一个/最后一个内容列（用于"内容字 vs 黑边字"）
                            if ((ox_lo == 11'd0) && (ox_hi == 11'd0)) ox_lo <= ox_w;
                            ox_hi <= ox_w + 11'd1;
                        end
                        ox_w   <= ox_w + 11'd1;
                        sx_acc <= sx_acc + $signed({10'd0, step});
                    end else begin
                        // 这个源像素用完了，取下一个
                        fill_i <= fill_i + 13'd1;
                        if ((fill_i + 13'd1) >= win_w) fs <= F_END;
                    end
                end
            end

            F_END: begin
                done <= 1'b1;
                fs   <= F_IDLE;
            end

            default: fs <= F_IDLE;
        endcase
    end
end

endmodule
