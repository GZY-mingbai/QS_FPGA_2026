//==============================================================================
//  img_layer_hd.v —— 640×480 RGB888 专用图层（1 字 = 1 像素）
//==============================================================================
//
//  【为什么单独写一个，而不是复用 ps_gen_image】
//  ps_gen_image 是给"320×240 RGB565 打包纹理"设计的：一个 32 位字装 2 个像素，
//  还要做缩放、负坐标、4 字组挑字、亚像素选择。高清图是 640×480、**一像素一字、
//  和屏幕 1:1**，用那条通路既浪费又容易踩它的历史补丁。
//
//  本模块只做一件事：
//      · 按 4 字组顺序请求 tex_base + 0 .. 307,199
//      · 收到的字直接当 {A,R,G,B} 输出
//      · 输出延迟和 ps_gen_image 对齐（3 拍），这样两层的像素天然同拍
//
//  ★ 2026-11-01 [FIFO-SHARE]：本层**不再自带 FIFO**。请求/像素 FIFO 借用 thumbA
//    （u_ta = img_layer_ctrl）那一套：两者帧内互斥（a_hd 只在帧边界改），帧间
//    flush（~ps_start）把两个 FIFO 清空。省掉 2 个 img_afifo + 一套 flush 同步器。
//    接口改成直接面向 FIFO：q_* = 请求 FIFO 写口，p_* = 像素 FIFO 读口。
//
//  【请求协议】完全照抄 img_layer_ctrl：
//      req_vld 高 = 队首有一个请求，req_addr 就是它；req_ack 一拍 = 取走
//      一个请求 = 一个 4 字组（掩码恒 1111，所以不需要带掩码过 FIFO）
//
//  【流控】
//      · 请求侧只看 `ahead = 已请求像素 - 已消费像素`，保持在 LEAD 以内
//        ⇒ 像素 FIFO（借用 u_ta 的，1024 深）永远不会溢出，也不依赖跨时钟水位
//      · 像素侧 layer_ready = FIFO 非空；空了就让整条场景流水线停住
//      · 帧间（frame_active=0）两个 FIFO 都清空（异步置位、按域同步释放）
//
//  【看门狗】
//  万一 SDRAM 一直不返回（初始化失败/地址越界），本层会把整条场景锁死成黑屏。
//  所以加一个超时：一帧内等不到数据超过 ~2^21 拍就置 dead，输出透明、ready 拉高，
//  画面降级但不死锁（和 ps_gen_image 的做法一致）。
//
//==============================================================================

module img_layer_hd #(
    parameter integer IMG_W = 640,
    parameter integer IMG_H = 480,
    parameter integer LEAD     = 40,    // 预取领先量（像素）
    parameter integer LAT      = 3      // 输出延迟（要和 ps_gen_image 对齐）
)(
    // ---- 场景图侧（clk = 50MHz）----
    input  wire        clk,
    input  wire        rst,
    input  wire        pipe_ce,         // 场景流水线推进使能
    input  wire        adv,             // = ps_start & pixel_ready
    input  wire        frame_active,    // = ps_start
    input  wire [15:0] x,               // 未用（全屏 1:1），只为接口对齐
    input  wire [15:0] y,
    input  wire [20:0] tex_base,
    input  wire        layer_en,

    output wire [31:0] pix,             // {A,R,G,B}
    output wire        out_valid,
    output wire        layer_ready,
    // ★ 2026-10-23：HD 层永远是满屏 640×480（没有 cfg_pos/cfg_size），
    //   所以"这一列归我负责"恒为 1。viewer_scene 会再与上 `a_en & a_hd`。
    output wire        in_box_o,

    // ---- 借用的 FIFO（img_layer_ctrl 的 ext_* 口，均在 clk 域）----
    output wire        q_push,          // 推一个 4 字组请求
    output wire [20:0] q_addr,
    input  wire        q_full,
    input  wire        q_wrst,          // 请求 FIFO 写口还在复位 ⇒ 不许 push
    output wire        p_pop,
    input  wire [31:0] p_head,
    input  wire        p_empty
);

localparam integer NPX  = IMG_W * IMG_H;        // 307,200
localparam integer NGRP = NPX / 4;              // 76,800 个 4 字组

//------------------------------------------------------------------------------
//  请求侧（clk 域）：按 4 字组顺序推进
//------------------------------------------------------------------------------
reg  [16:0] rq_grp;                 // 0 .. NGRP-1（76,800 需要 17 位）
reg  [19:0] ds_px;                  // 本帧已消费像素数（0 .. 307,200）

wire        pix_rempty = p_empty;
wire [31:0] pix_head   = p_head;

wire [19:0] issued_px = {rq_grp, 2'b00};
wire [19:0] ahead     = issued_px - ds_px;
wire        rq_can     = frame_active && layer_en && (rq_grp < NGRP) && (ahead < LEAD);

wire        req_wfull = q_full;
// ★ 必须等请求 FIFO 的复位释放（q_wrst）再 push：
//   否则前 2 拍写进去的请求被 wrst 吃掉，rq_grp 却照涨
//   ⇒ 整幅图右移 8 像素（实测）。
wire        req_push = rq_can && !req_wfull && !q_wrst;

always @(posedge clk or posedge rst) begin
    if (rst)                 rq_grp <= 17'd0;
    else if (!frame_active)  rq_grp <= 17'd0;
    else if (req_push)       rq_grp <= rq_grp + 17'd1;
end

always @(posedge clk or posedge rst) begin
    if (rst)                 ds_px <= 20'd0;
    else if (!frame_active)  ds_px <= 20'd0;
    else if (adv && !pix_rempty) ds_px <= ds_px + 20'd1;
end

assign q_push = req_push;
assign q_addr = tex_base + {{2{1'b0}}, rq_grp, 2'b00};
assign p_pop  = adv && !pix_rempty;

//------------------------------------------------------------------------------
//  看门狗：一帧内一直没数据 ⇒ 降级（透明 + ready），不锁死场景
//------------------------------------------------------------------------------
reg [20:0] wd_cnt;
reg        dead;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wd_cnt <= 21'd0;
        dead   <= 1'b0;
    end else if (!frame_active) begin
        wd_cnt <= 21'd0;
        dead   <= 1'b0;
    end else if (!pix_rempty) begin
        wd_cnt <= 21'd0;                 // 有货就不算饿死
    end else if (wd_cnt != 21'h1F_FFFF) begin
        wd_cnt <= wd_cnt + 21'd1;
    end else begin
        dead <= 1'b1;
    end
end

assign layer_ready = (!layer_en) ? 1'b1 : (dead ? 1'b1 : ~pix_rempty);

// ★ HD 层满屏、永远在框内 ⇒ "这一列归我负责"恒 1（viewer_scene 会再与 a_hd/a_en）
assign in_box_o = 1'b1;

//------------------------------------------------------------------------------
//  输出流水线：LAT 级，与 ps_gen_image 的 D_img 对齐
//------------------------------------------------------------------------------
reg [31:0] p1, p2, p3;
reg        v1, v2, v3;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        p1 <= 32'd0; p2 <= 32'd0; p3 <= 32'd0;
        v1 <= 1'b0;  v2 <= 1'b0;  v3 <= 1'b0;
    end else if (pipe_ce) begin
        // ★ 2026-10-23：第 0 级必须用 adv 门控装载！
        //   帧尾排水（viewer_scene 的 drain）会把 pipe_ce 强推 PIPE 拍，
        //   那时 ps_start=0 ⇒ flush=1 已经把 FIFO 清掉 ⇒ pix_head 是垃圾。
        //   无脑 `p1 <= pix_head` 会把垃圾灌进流水线。
        //   用 adv 门控后：排水期间 p1 保持最后一个真像素，整条链把它搬出去，
        //   帧末 4 个像素的数据反而是**精确**的。
        if (adv && !pix_rempty) p1 <= pix_head;
        v1 <= adv && !pix_rempty;
        p2 <= p1;  v2 <= v1;
        p3 <= p2;  v3 <= v2;
    end
end

assign pix       = (!layer_en || dead) ? 32'h0000_0000 :
                   (LAT >= 3 ? p3 : (LAT == 2 ? p2 : p1));
assign out_valid = (!layer_en) ? 1'b1 :
                   (LAT >= 3 ? v3 : (LAT == 2 ? v2 : v1));

endmodule
