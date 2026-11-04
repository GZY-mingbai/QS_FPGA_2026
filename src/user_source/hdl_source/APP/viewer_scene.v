//==============================================================================
//  viewer_scene.v —— 图片查看器的场景图（黑底 + 两层图 + 一次混合）
//==============================================================================
//
//  【结构】
//
//      app_scan_gen ──(x,y)──┬─► thumbA (565, 160 字/行) ─┐
//                            ├─► thumbB (565, 160 字/行) ─┤
//                            └─► hd     (888, 640 字/行) ─┤
//                                                         │
//                       A 层三选一（a_hd 决定）───────────┤
//                                                         ▼
//                                              ps_merge_alpha8（B over A）
//                                                         ▼
//                                                    ps_to_rgb24
//
//  【为什么只有一级混合】
//  A 是底图，B 盖在 A 上；B 在框外透明 ⇒ 输出 A。最后 ps_to_rgb24 把结果
//  叠到纯黑上并丢掉 alpha。这样总延迟 = 3（图片层）+ 1（混合）= 4 拍。
//
//  【三块图层共享一条纹理请求总线】
//  SDRAM 调度器只有两路纹理口（layer0 / layer1）。layer0 按 a_hd 二选一
//  （2026-11-01 起 hd 直接借用 thumbA 的 FIFO，不再在总线上做 mux）：
//      · 过渡 / 低清：layer0 = thumbA，layer1 = thumbB
//      · 高清静止：  layer0 = hd，    layer1 = 关
//  a_hd 只在帧边界改；帧间 flush 会清掉在途请求，所以不会串层。
//
//  【流控】沿用 demo 的配额机制，任意一层没货 / 写口快满 / 配额用完
//  ⇒ pixel_ready=0 ⇒ 扫描停走，所有层一起冻结。
//
//==============================================================================

module viewer_scene #(
    parameter integer IMG_W   = 640,
    parameter integer IMG_H   = 480,
    parameter integer SRC_W   = 160,        // 缩略图行宽（字）= 320/2
    parameter integer LEAD    = 384,   // 2026-10-24: 64 太小 —— A 一行要 160 个字，预取不足会在同一 x 取空 = 固定 x 全高竖条
    parameter integer AFIFO_AW = 10,        // 1024 深（≥ 2×LEAD 512，build 100M-41 验证过的配置）
    parameter integer SCENE_QUOTA = 48,     // 888：写口只能吃 37.5%
    parameter integer QUOTA_565   = 96,     // 565：写口吞吐翻倍
    //--------------------------------------------------------------------------
    //  文件页四宫格：槽 0 的 SDRAM 基址（槽间距 2^16 = 65536 字）
    //  必须是常量：quad_sel 的 SLOT_BASE 是 parameter，不能接 port。
    //  4 槽共 4×65536 = 262144 字，占 1048576..1310719（SDRAM 21 位 = 2M 字）。
    //--------------------------------------------------------------------------
    parameter [20:0] QUAD_BASE   = 21'd1048576
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        mclk,

    // ---- 像素源接口（对 frame_writer）----
    input  wire        ps_start,
    output wire        pixel_ready,
    // （blk_dbg / duty_dbg / dbg_delta / dbg_evt / smp_bus / smp_win：
    //   2026-10-26 诊断读数，随数码管"只保留 4 项"清理一起删除）
    output wire [23:0] ps_data,
    output wire        pipe_ce,

    input  wire        fmt565,              // 本帧帧缓存是不是 565（决定配额）
    input  wire        wr_stall,            // 写 FIFO 快满

    // ---- A 层（底图）----
    input  wire        a_hd,                // 1 = 用 hd 层，0 = 用 thumbA
    input  wire [31:0] a_pos,
    input  wire [31:0] a_size,
    input  wire [39:0] a_k,
    input  wire [19:0] a_ori,           // ★ [ZOOM-EXACT] A 层锚点 {ORIX[9:0], ORIY[9:0]}
    input  wire [20:0] a_base,
    input  wire        a_en,

    // ---- 文件页四宫格（quad_mode=0 时下面这些信号全被综合器裁掉）----
    //  B 层改成"按屏幕坐标从 4 个 SDRAM 槽里选一个"（2026-11-03 起，原来是 A 层），
    //  A 层不动 ⇒ 格子以外透出相册底图。
    input  wire        quad_mode,           // 1 = B 层显示 4 宫格（基址见 QUAD_BASE），A 层照常画相册
    input  wire [3:0]  quad_vm,             // 本页哪几格有内容（帧内恒定）

    // ---- B 层（新图，盖在 A 上）----
    input  wire [31:0] b_pos,
    input  wire [31:0] b_size,
    input  wire [39:0] b_k,
    input  wire [20:0] b_base,
    input  wire        b_en,
    input  wire [7:0]  alpha_b,

    // ---- 扫描坐标导出（给 App 里的 osd_top 用）----
    output wire [15:0] scan_x,
    output wire [15:0] scan_y,

    // ---- OSD 图层（osd_top 输出；osd_on=0 时不参与，行为与原来完全一致）----
    input  wire        osd_on,
    input  wire [7:0]  osd_fade,
    input  wire [31:0] osd_pix,

    // ---- 纹理读仲裁 ----
    output wire [1:0]  tex_req_vld,
    output wire [41:0] tex_req_addr,
    output wire [7:0]  tex_req_mask,
    input  wire [1:0]  tex_req_ack,
    input  wire [1:0]  tex_pix_wr,
    input  wire [63:0] tex_pix_data
);

//==============================================================================
//  0. 内部连线（全部先声明 —— iverilog 不接受"先用后声明"）
//==============================================================================
wire [15:0] sx, sy;
wire        svalid;

wire [31:0] ta_pix, tb_pix, hd_pix;
wire        ta_v,   tb_v,   hd_v;
wire        ta_rdy, tb_rdy, hd_rdy;

wire [20:0] ta_ra, tb_ra;
wire [3:0]  ta_rm, tb_rm;
wire        ta_rv, tb_rv;
wire        ta_ack, tb_ack;

wire        ta_pix_wr, tb_pix_wr;
wire [31:0] ta_pix_data, tb_pix_data;

// ★ [FIFO-SHARE] HD 层 ↔ u_ta 的 FIFO（clk 域）
wire        hd_q_push, hd_q_full, hd_q_wrst;
wire [20:0] hd_q_addr;
wire        hd_p_pop,  hd_p_empty;
wire [31:0] hd_p_head;

wire        adv;
wire        pixel_ready_i;

assign adv = ps_start & pixel_ready;
assign pipe_ce = pixel_ready_i;
assign pixel_ready = pixel_ready_i;

//==============================================================================
//  1. 扫描坐标
//==============================================================================
app_scan_gen #(.IMG_W(IMG_W), .IMG_H(IMG_H)) u_scan_gen (
    .clk(clk), .rst(rst),
    .ps_start(ps_start),
    .pixel_ready(pixel_ready_i),
    .x(sx), .y(sy), .valid(svalid)
);

// ★ 2026-10-23：两层解耦用的"这一列归谁负责"（声明必须在例化之前，
//   iverilog / TD 都不允许先用后声明）
wire        ta_in_box, tb_in_box;
// ★ [INBOX-P]：与各层 `pix` 严格同拍的框内标志（in_box_o 早一个像素）
wire        ta_in_box_p, tb_in_box_p;
// ★ 2026-10-28 T6：6 根诊断线（dbg_delta / dbg_evt / smp_bus[127:0] /
//   smp_win / pix_empty_o）声明与连接一并删除。它们从 10-26 数码管清理后就
//   没有任何消费者，原注释指望"综合会把整条链剪掉"——实测见 commit 记录。
//   真要恢复观测口时，从 ps_gen_image 的端口表反查即可。

//==============================================================================
//  2. 三块图层
//==============================================================================
// ---- thumbA（layer0 的 565 通路）----
//==============================================================================
//  四宫格（quad_mode=0 时下面的选择器全部退化成原值）
//==============================================================================
//  【为什么这里不再做坐标变换】
//  第一版把屏幕坐标换成"格子内局部坐标"再喂给图层，结果上板/仿真双双死锁：
//  ps_gen_image 内部有**两套坐标**（请求侧自带整屏计数器 rq_x/rq_y，
//  显示侧用输入 x/y）。只换 x/y 会让两侧的框判定指向不同区域
//  ⇒ 请求侧 19200 个字就扫完全帧，显示侧要 76800 个 ⇒ 卡死。
//  ⇒ 正解是把四宫格做进 ps_gen_image（quad_en），两侧吃同一套整屏坐标。
//  所以这里只负责：框尺寸、基址、使能 三件事。
//
//  src 图 = 320×240（= 现有缩略图那一档，160 字/行，行宽正好等于 SRC_W），
//  所以**不需要给行补空**，bmp_view 的 cont1 连续读快路径照常可用。
//  1:1 与 2:1 抽样的换算都在 ps_gen_image 的 quad_map 里，见那边的注释。
//==============================================================================
// ★ 2026-11-03【四宫格改画在 B 层】原来四宫格占 A 层、B 层强制关：
//   格子以外 A 层框外恒 0 ⇒ 菜单底下的相册缩略图整片变黑（上板故障）。
//   现在 A 层照常画相册（S_OSD 下就是当前图的缩略图），四宫格画在 B 层，
//   B 框外 A=0 ⇒ 底图透出来。两层 565 同时读 = 过渡动画那条已验证 60fps 的路径。
// a_hd 仍被 quad 压掉（菜单里 a_hd 恒 0，这里只是守住"四宫格时 A 不走 888"这条不变量）
wire        a_hd_e = a_hd & ~quad_mode;
wire        a_en_e = a_en;

// A 层的输入：quad_mode=0 时原样透传，行为与改动前**逐位相同**
//  【为什么不给 pos/size/k/ori 加 mux】ps_gen_image 在 quad_en=1 时
//    · cfg_ok 恒真（不看 rw/rh）
//    · quad_map 只用常量几何（不看 x0/k/ori）
//  所以这四组在四宫格模式下**根本没有读者**，加 mux 只是白花 ~35 LUT。
//  只有 tex_base 必须换（它是请求地址的基点）。
wire [9:0]  ta_x    = sx[9:0];
wire [9:0]  ta_y    = sy[9:0];
wire [20:0] ta_base = a_base;
wire [31:0] ta_pos  = a_pos;
wire [31:0] ta_size = a_size;
wire [39:0] ta_k    = a_k;
wire [19:0] ta_ori  = a_ori;

// 四宫格 = B 层（S_OSD 里 b_en 恒 0，不会和过渡动画撞）；全不透明盖在 A 上
wire        b_en_q  = b_en | quad_mode;
wire [20:0] tb_base = quad_mode ? QUAD_BASE : b_base;
wire [7:0]  alpha_q = alpha_b | {8{quad_mode}};

img_layer_ctrl #(
    .IMG_W(IMG_W), .IMG_H(IMG_H), .SRC_W(SRC_W),
    .LEAD(LEAD), .AFIFO_AW(AFIFO_AW), .ALIGN_ONLY(0)
) u_ta (
    .clk(clk), .rst(rst), .mclk(mclk),
    .pipe_ce(pixel_ready_i),
    .adv(adv), .frame_active(ps_start),
    .x(ta_x), .y(ta_y),
    .cfg_pos(ta_pos), .cfg_size(ta_size), .cfg_k(ta_k), .cfg_ori(ta_ori),
    .tex_base(ta_base), .quad_en(1'b0), .quad_vm(4'd0), .layer_en(a_en_e & ~a_hd_e),
    .pix(ta_pix), .out_valid(ta_v), .layer_ready(ta_rdy), .in_box_o(ta_in_box),
    .in_box_p(ta_in_box_p),
    .dbg_delta(), .dbg_evt(), .smp_bus(), .smp_win(), .pix_empty_o(),
    .no_underrun_fix(1'b0),
    .exp_mode(4'd0),
    // ★ [FIFO-SHARE] HD 模式下这两个 FIFO 归 u_hd 用（ps_gen_image 此时 layer_en=0）
    .ext_sel(a_hd_e),
    .ext_push(hd_q_push), .ext_addr(hd_q_addr), .ext_wfull(hd_q_full), .ext_wrst(hd_q_wrst),
    .ext_pop(hd_p_pop), .ext_head(hd_p_head), .ext_rempty(hd_p_empty),
    .req_vld(ta_rv), .req_addr(ta_ra), .req_mask(ta_rm), .req_ack(ta_ack),
    .pix_wr(ta_pix_wr), .pix_data(ta_pix_data)
);

// ---- thumbB（layer1）----
img_layer_ctrl #(
    .IMG_W(IMG_W), .IMG_H(IMG_H), .SRC_W(SRC_W),
    .LEAD(LEAD), .AFIFO_AW(AFIFO_AW), .ALIGN_ONLY(0)
) u_tb (
    .clk(clk), .rst(rst), .mclk(mclk),
    .pipe_ce(pixel_ready_i),
    .adv(adv), .frame_active(ps_start),
    .x(sx), .y(sy),
    .cfg_pos(b_pos), .cfg_size(b_size), .cfg_k(b_k), .cfg_ori(20'd0),
    .tex_base(tb_base), .quad_en(quad_mode), .quad_vm(quad_vm), .layer_en(b_en_q),
    .pix(tb_pix), .out_valid(tb_v), .layer_ready(tb_rdy), .in_box_o(tb_in_box),
    .in_box_p(tb_in_box_p),
    .dbg_delta(), .dbg_evt(), .smp_bus(), .smp_win(), .pix_empty_o(),
    .no_underrun_fix(1'b0),
    .exp_mode(4'd0),
    .ext_sel(1'b0),
    .ext_push(1'b0), .ext_addr(21'd0), .ext_wfull(), .ext_wrst(),
    .ext_pop(1'b0), .ext_head(), .ext_rempty(),
    .req_vld(tb_rv), .req_addr(tb_ra), .req_mask(tb_rm), .req_ack(tb_ack),
    .pix_wr(tb_pix_wr), .pix_data(tb_pix_data)
);

// ---- hd（layer0 的 888 通路）----
//  ★ 2026-11-01 [FIFO-SHARE]：不再自带 FIFO，借用 u_ta 的那一套（见 img_layer_hd 文件头）
img_layer_hd #(
    .IMG_W(IMG_W), .IMG_H(IMG_H),
    .LEAD(40), .LAT(3)
) u_hd (
    .clk(clk), .rst(rst),
    .pipe_ce(pixel_ready_i),
    .adv(adv), .frame_active(ps_start & a_hd_e),
    .x(sx), .y(sy),
    .tex_base(a_base), .layer_en(a_en_e & a_hd_e),
    .pix(hd_pix), .out_valid(hd_v), .layer_ready(hd_rdy), .in_box_o(),
    .q_push(hd_q_push), .q_addr(hd_q_addr), .q_full(hd_q_full), .q_wrst(hd_q_wrst),
    .p_pop(hd_p_pop), .p_head(hd_p_head), .p_empty(hd_p_empty)
);

//==============================================================================
//  3. 纹理总线（layer0 = u_ta 的 FIFO；HD 模式下由 u_hd 借用，见 [FIFO-SHARE]）
//==============================================================================
//  （四宫格的槽基址由 ps_gen_image 的 quad_slot 直接在请求地址里加，
//    所以这里 ta_ra 原样出去，不需要额外加法器。）
//  ★ 2026-11-01 [FIFO-SHARE]：thumbA / hd 共用一套 FIFO ⇒ 这里不再需要 mux。
assign tex_req_vld[0]     = ta_rv;
assign tex_req_addr[20:0] = ta_ra;
assign tex_req_mask[3:0]  = ta_rm;
assign ta_ack = tex_req_ack[0];

assign tex_req_vld[1]      = tb_rv;
assign tex_req_addr[41:21] = tb_ra;
assign tex_req_mask[7:4]   = tb_rm;
assign tb_ack = tex_req_ack[1];

assign ta_pix_wr   = tex_pix_wr[0];
assign tb_pix_wr   = tex_pix_wr[1];
assign ta_pix_data = tex_pix_data[31:0];
assign tb_pix_data = tex_pix_data[63:32];

//==============================================================================
//  4. A / B 层输出选择
//==============================================================================
// ★ 2026-10-23：两层解耦用的"这一列归谁负责"
//   ta_in_box/tb_in_box 来自 ps_gen_image 的 in_box_o —— 它是**第 2 拍**，
//   比自己的 pix 早一个像素，所以只能用于"下一个要弹出的像素"这类流控判断
//   （layer_ok）；凡是"要挑这一拍的像素/背景"都必须用下面的 in_box_*_p
//   （见 [INBOX-P]）。HD 层满屏 ⇒ 恒 1。层被关掉时按"不负责"处理
//   （避免关掉的层拖住整幅）。
wire        in_box_a = !a_en_e ? 1'b0 : (a_hd_e ? 1'b1 : ta_in_box);
// （dbg_delta / dbg_evt / smp_bus / smp_win：2026-10-26 随数码管清理删除）
wire        in_box_b = !b_en_q ? 1'b0 : tb_in_box;

// ★ 2026-11-05 [INBOX-P]：**与 pix/v 同拍**的那一份（第 3 拍）。
//   in_box_o 是第 2 拍，比它自己的 pix 早一个像素 —— 用它"选层"就会：
//   格左沿那列选了 B 但 B 的 pix 还是框外值(0) ⇒ 左边一条黑；
//   源第 0 列落到 X0+1 ⇒ 整幅右移 1px；右沿那列判成框外 ⇒ 丢最后一列。
//   `layer_ok`/`layer_ready` 仍用上面第 2 拍的版本（判的是"下一个要弹出的像素"）。
wire        in_box_a_p = !a_en_e ? 1'b0 : (a_hd_e ? 1'b1 : ta_in_box_p);
wire        in_box_b_p = !b_en_q ? 1'b0 : tb_in_box_p;

wire [31:0] pix_a = !a_en_e ? 32'h0000_0000 : (a_hd_e ? hd_pix : ta_pix);
wire        v_a   = !a_en_e ? 1'b1          : (a_hd_e ? hd_v   : ta_v);
wire        rdy_a = !a_en_e ? 1'b1          : (a_hd_e ? hd_rdy : ta_rdy);

wire [31:0] pix_b = !b_en_q ? 32'h0000_0000 : tb_pix;
wire        v_b   = !b_en_q ? 1'b1          : tb_v;
wire        rdy_b = !b_en_q ? 1'b1          : tb_rdy;

//==============================================================================
//  5. 配额（照抄 demo：主动把产出摊平到写口产能）
//==============================================================================
localparam integer QUOTA_W = 7;
wire [QUOTA_W-1:0] quota_val = fmt565 ? QUOTA_565[QUOTA_W-1:0]
                                      : SCENE_QUOTA[QUOTA_W-1:0];

reg [6:0]         q_win;
reg [QUOTA_W-1:0] q_left;
reg               quota_ok_d;
wire              quota_ok = (q_left != {QUOTA_W{1'b0}});

always @(posedge clk or posedge rst) begin
    if (rst) begin
        // ★ 复位值必须是常量 0：原来写 `q_left <= quota_val`（quota_val 是运行时
        //   fmt565 选出来的 96/48），TD 会为了"复位成 1"再造一个异步置位，
        //   于是同一个触发器同时有异步复位和异步置位 ⇒ SYN-5065 警告，
        //   严重时综合器直接崩。第一个 128 拍窗口到了自然会重新装载。
        q_win      <= 7'd0;
        q_left     <= {QUOTA_W{1'b0}};
        quota_ok_d <= 1'b0;
    end else begin
        quota_ok_d <= quota_ok;
        if (q_win == 7'd127) begin
            q_win  <= 7'd0;
            q_left <= quota_val;
        end else begin
            q_win <= q_win + 7'd1;
            // 2026-10-24 fix: 原条件不检查"是否真的推进"，缺料/背压期间配额也被扣。
            if (ps_start && pixel_ready_i && quota_ok_d && (q_left != {QUOTA_W{1'b0}}))
                q_left <= q_left - 1'b1;
        end
    end
end

//==============================================================================
//  5.5 ★ 帧尾排水（2026-10-23：修"最后一帧永远写不满 ⇒ 全屏黑"）★
//==============================================================================
//  【上板现象】全屏黑，数码管状态码卡在 4（S_HDF），display_valid 恒 0。
//    仿真里量到：一帧 take 了 307200 个像素，但 write_en 只有 307196 个
//    —— 正好差 PIPE(4) 个。
//  【为什么差】frame_writer 的 write_en 是 take 延迟 PIPE 拍：
//      assign write_en = en_d[PIPE-1] & pipe_ce;
//    而 ps_start 在 cnt 数到 NPX 的那一拍就落下了。ps_start=0 之后：
//      · img_layer_hd 的 flush = ~(ps_start & a_hd) 立刻异步清 FIFO
//        （2026-11-01 起 hd 借用 u_ta 的 FIFO，flush = ~ps_start，结论不变）
//        ⇒ layer_ready=0
//      · ps_gen_image 的 warmed 也被 idle_reset 清掉 ⇒ layer_ready=0
//    于是 pipe_ce 立刻变 0，延迟链**冻结**，最后 4 个像素永远出不来
//    ⇒ 写引擎少 4 个字 ⇒ write_finish 永不产生 ⇒ busy 永远为 1
//    ⇒ viewer_ctrl 停在 S_HDF（数码管 4）、display_valid 恒 0 ⇒ 黑屏。
//  【修法】ps_start 落下后再强推 PIPE 拍的 pipe_ce，把延迟链排空。
//    · 排水期间 adv = ps_start & pixel_ready = 0 ⇒ 不会多取像素
//    · en_d 链和像素流水线用的是同一个 pipe_ce，一起冻一起走
//      ⇒ 相位永远保持，write_en 与 ps_data 依然对齐
//    · 各图层的第 0 级必须用 adv 门控装载（见 img_layer_hd.v），
//      否则排水期间会灌进垃圾（FIFO 已清零）
//  【代价】ps_gen_image 是 adv 驱动的，排水时它输出冻结 ⇒ 帧末最后 4 个像素
//    是前一像素的重复。4/307200，肉眼不可见，换来的是"帧能收满"。
//==============================================================================
localparam integer PIPE = 5;        // 必须与 frame_writer 的 PIPE 一致（[DSP-PIPE] 后 4→5）

reg        ps_start_d;
reg  [2:0] drain;
wire       draining = (drain != 3'd0);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        ps_start_d <= 1'b0;
        drain      <= 3'd0;
    end else begin
        ps_start_d <= ps_start;
        if (ps_start_d & ~ps_start) drain <= PIPE[2:0];   // 帧结束 ⇒ 排水 PIPE 拍
        else if (draining)          drain <= drain - 3'd1;
    end
end

//==============================================================================
//  ★★ 2026-10-24：帧内失速自愈（fail-safe）★★
//==============================================================================
//  【为什么加这个】`rdy_a & rdy_b & ~wr_stall` 里任何一项卡住，整帧就一个像素也
//    走不动。App 层自身已被三个仿真证明不会卡：
//        tb_viewer  稀疏框（a_pos=-9 / b_pos=+631，就是"只左移一点点"那点）PASS
//        tb_fx0     单层 X0 逐帧 0→-38、纹理限速 1字/4拍，20/20 帧全过
//        tb_trans   板上配额 48/96 + 835k 帧周期，帧间隔恰好 16.7ms、tstep=1
//    但板上多了一层仿真里**没有**的东西：真 SDRAM 控制器（IP 是加密的，
//    iverilog 跑不了，只能靠 sdr_as_ram_sim 近似）。只要有一个纹理字没回来
//    （标签环错位/换向窗口被吞/装载抢占），该层就永远 ready=0 ⇒ 整帧冻结 ⇒
//    tick 不来 ⇒ trans_to 看门狗 1.34s ⇒ 直接跳到目标高清图。
//    上板现象："只移动一点点 → 卡住 → 3s 内闪到图 2"——逐条对得上。
//
//  【做法】只数"连续多少拍没有任何像素推进"。真卡住时不再等，把**本帧降级**：
//    降级后不再要求两层 ready（纹理那一路放过），但**保留 ~wr_stall**（写口是
//    真流控，满了就该等，硬塞会丢更多字）。于是这一帧一定能走完，
//    t 继续走、过渡正常结束。代价是那一下可能出现少量重复/错色像素，
//    比"冻住 1.34s 然后跳图"好一个量级。帧界（ps_start 落下）清标志。
//
//  【★★ 2026-10-23 第二次修订：这个 fail-safe 自己就是"按键撕裂"和"交界竖条"
//      的来源，必须收紧到几乎不触发 ★★】
//    上板证据链：
//      ① `layer_ready` 的真实定义（ps_gen_image.v:1290）是
//             `~f_en | dead | (warmed && (~pop_want | pix_count != 0))`
//         —— 只在"本层在框内 && 要弹像素 && FIFO 空"时才为 0，**框外的层永远报就绪**。
//         所以 `rdy_a & rdy_b` 本身没问题，问题在下面的"降级"动作。
//      ② `layer_dead` 已被 100M-33 永久禁用（恒 0），`give_up` 也不再放行扫描
//         （build F）。⇒ **整个设计里唯一能把扫描推过"缺失像素"的东西，就是这里。**
//      ③ 一旦降级本帧就锁存 ⇒ 扫描全速前进，而缺数据的那一层 `adv` 照走、
//         pix 保持上一个像素 ⇒ 屏幕上就是**一条"该层颜色"的竖条**；
//         刚好缺在交界那一列（B 每行第一个框内像素最容易被吞）⇒ 位置正好在交界；
//         px 每帧变 ⇒ 闪烁。
//      ④ 按键那一刻 B 被关/要预热、A 从高清切缩略图，`rdy` 必掉 ⇒ 降级 ⇒
//         这一帧**注定被提交并在场边界被采纳** ⇒ 一帧撕裂。
//    ⇒ **收紧两处**：
//      (a) **只在 565 过渡帧降级**（`fmt565`）。888 是"静止高清"，那一帧就是用户
//          最终看到的图，绝不允许带着降级痕迹落屏 ⇒ 按键撕裂消失。
//      (b) 阈值 400k → **2,000,000（40ms）**。400k 只按"扫完整屏 307,200 拍"算，
//          没算 `rq_go_eff` 被节流后的实际耗时（实测能到 3~5 倍）；
//          40ms 仍比看门狗的 1.34s 快 33 倍，只在**真死锁**时才接管。
//==============================================================================
localparam integer STALL_BREAK = 2000000;      // 40ms @50MHz

reg [20:0] stall_cnt;                      // 21 位：够装 2,000,000
reg        frame_degraded;

// ★★ 2026-10-23：**两层解耦** ★★
//  【原来】`rdy_a & rdy_b` —— 任一层在框内缺数据，**整幅扫描一起停**。
//    横向平移时 B 的可见带很窄（每行几个列），它每个框入口都会把像素 FIFO
//    抽空一次 ⇒ 每行都停一次 ⇒ 一帧被拉长 ⇒ 帧时间超过 16.7ms ⇒ 掉到 30fps；
//    同一个停顿还会让 B 的窄带错一个字 ⇒ 交界处那条竖条。
//  【现在】只问"负责这一列的那一层"：
//    · 一列只归一层（两层的框相邻拼接）⇒ 另一层在框外、FIFO 空是**正常**的，不该等；
//    · fx2（淡入淡出）两层都满屏 ⇒ in_box_a=in_box_b=1 ⇒ 仍然两个都等（正确）；
//    · 某层被关掉 ⇒ in_box=0 ⇒ 不等它（它本来就不出像素）。
//  任一列都不归任何层时也不等 —— 那本来就没有像素可等。
wire layer_ok  = (in_box_a ? rdy_a : 1'b1) & (in_box_b ? rdy_b : 1'b1);

//==============================================================================
//  ★★ 2026-10-23：帧内"停顿归因"计数器（纯观测，不影响任何行为）★★
//==============================================================================
//  【要回答的问题】帧时间 20~50ms，但带宽算下来只要 6~8ms。
//    ⇒ 时间花在"停"上，必须知道停在谁身上：
//         rdy_a 低 ⇒ A 层纹理饿     rdy_b 低 ⇒ B 层纹理饿
//         wr_stall  ⇒ 写口满（frame_fifo_write 的 FIFO 顶到阈值）
//  【怎么算】每拍按优先级只记一个"当前阻挡源"，帧末比大小取主导，饱和到 2^18。
//    只统计 ≥ 2000 拍（0.04ms）的源，噪声不算。
//  【为什么优先级挡住 quota】quota 是**设计上的节流**，不是故障，不参与归因。
//==============================================================================
//  ★★ 帧内"停顿归因"计数器 —— 已随 2026-10-26 数码管清理删除 ★★
//  【原功能】每拍按优先级记一个"当前阻挡源"（写口 > A 饿 > B 饿），
//    帧末比大小取主导，输出到 blk_dbg 供数码管读数。
//  【为什么删】数码管只保留 帧率/图片号/模式/装载进度，该读数已无消费者。
//  【留档】BLK_MIN=2000（0.04ms）以下的源不算（噪声过滤）；饱和度 2^18。

// ★ 2026-11-03 [AREA]：stall_cnt 只喂 frame_degraded（粘滞，清它的条件与清计数相同）
//   ⇒ 只需"第一次到达 STALL_BREAK"那一拍：== 与 >= 等价，且计数饱和也不再需要
//   （回绕后再次命中只是重复置 1）。
wire stall_hit = (stall_cnt == STALL_BREAK[20:0]);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        stall_cnt      <= 21'd0;
        frame_degraded <= 1'b0;
    end else if (!ps_start) begin
        stall_cnt      <= 21'd0;              // 帧间清零
        frame_degraded <= 1'b0;
    end else if (layer_ok) begin
        stall_cnt      <= 21'd0;              // 有推进 ⇒ 正常
    end else begin
        stall_cnt <= stall_cnt + 21'd1;
        // ★ 2026-10-23（第三次）：**恢复对所有帧生效**。
        //   上一版只允许 565 降级，结果 888 那帧一旦真卡住就**永远收不满**
        //   ⇒ `busy` 不落 ⇒ `S_HDF` 相位 2 永远等不到提交 ⇒ 屏幕一直停在
        //   过渡最后一帧（放大的缩略图）⇒ 表现为"HD 图像显示有误"。
        //   现在靠"阈值 40ms 足够长"来保证它平时不触发，而不是靠 fmt565 关掉它。
        //   （按键撕裂已由时隙表 + 关掉 RD 借槽修掉，不再依赖这条闸门。）
        if (stall_hit)               frame_degraded <= 1'b1;
    end
end

// 降级时只放过"纹理 ready"，写口反压（wr_stall）仍然有效
wire tex_ok = frame_degraded ? 1'b1 : layer_ok;

// ★ pixel_ready 必须放在 draining 声明之后（TD: HDL-5373 used before declaration）
// 2026-10-24 fix: `| draining` 原来把 wr_stall 也绕过了，与上一行注释矛盾。
//   排水可绕过"纹理就绪"和"配额"，但不绕写口反压。
// 2026-10-24 **回退**：draining 必须能绕过 tex_ok，否则会死锁 ——
//   warmed=0 或某层未就绪时 tex_ok=0 ⇒ pixel_ready 永不拉高 ⇒ 帧收不满
//   ⇒ busy 不落 ⇒ 状态机卡住（实测：上电不加载 HD、按键无响应）。
//   跳过 pop 的问题改由 ps_gen_image 的 s2_hold 在层内解决（不动全局流控）。
//==============================================================================
//  ★★ 2026-11-01【帧级看门狗】任何一层永远不就绪，也不许把整帧卡死 ★★
//==============================================================================
//  【上板现象】进文件页后整屏停更：光标不动、KEY1 退不出、图全透明。
//    根因链：四宫格的图没装成 ⇒ 该层 `layer_ready` 长期 0 ⇒ `ps_start` 这一帧
//    永远收不满 ⇒ `frame_writer` 的 `busy` 不落 ⇒ `viewer_ctrl` 停在 S_OSD 等
//    帧边界 ⇒ **不再出新帧** ⇒ 屏幕冻结（按键其实都还在改内部状态）。
//  【本看门狗干什么】帧内若连续 WD_MAX 拍一个像素都没推进，就强制放行：
//    宁可这一帧少一层（该层按"重复上一像素/透明"降级），也绝不让整屏停更。
//    一帧的像素数是 307200，正常 60fps 一帧 ≈ 800k 拍；
//    WD_MAX = 2^22 ≈ 42ms @50MHz，比一个最慢的缩略图装载还长，正常路径不会误触。
//  【为什么放帧内】帧间（ps_start=0）本来就一直在等装载，那里不能计时。
reg  [22:0] wd_cnt;
reg         wd_force;
always @(posedge clk or posedge rst) begin
    if (rst)              begin wd_cnt <= 23'd0; wd_force <= 1'b0; end
    else if (!ps_start)   begin wd_cnt <= 23'd0; wd_force <= 1'b0; end
    else if (pixel_ready_i) wd_cnt <= 23'd0;
    else if (wd_cnt != 23'h7F_FFFF) wd_cnt <= wd_cnt + 23'd1;
    else                  wd_force <= 1'b1;
end

assign pixel_ready_i = (tex_ok & ~wr_stall & quota_ok) | draining | wd_force;

//==============================================================================
//  ★★ 推进占空比 / 帧时间读数 —— 已随 2026-10-26 数码管清理删除 ★★
//==============================================================================
//  【原功能】1024 拍窗口量 pixel_ready 占空比，或直接量 ps_start 高电平整数，
//    帧末锁存 `f_cyc >> 17`（单位 2.62ms/格）输出到 duty_dbg 供数码管读数。
//  【为什么删】数码管只保留 帧率/图片号/模式/装载进度 四项，帧时间已无消费者。
//  【读法留档】6 ≈ 16ms(60fps)  9 ≈ 24ms(42fps)  C ≈ 31ms(32fps)  F ≥ 39ms
//  【注意】f_cyc / ps_d 只被这段逻辑使用，一并移除；不影响 pixel_ready 任何行为。
//==============================================================================

//==============================================================================
//  ★★ 2026-10-23：SEAM_DBG —— B 层强制纯红（默认 0，只用于判定竖条归属）★★
//==============================================================================
//  烧一次就能三选一：
//    竖条变红   ⇒ 竖条在 B 的框内（B 的源字错一个）
//    竖条仍花   ⇒ 竖条是 A 的内容（A 右边界多一列）
//    竖条变黑   ⇒ 两层都无效（合并器输入相位）
//   ★ 用法：= 1 → B 纯红；= 2 → A 纯蓝；**= 0 才是正常画面**。
localparam integer SEAM_DBG = 0;

//==============================================================================
//  5.5 OSD 叠加：逐像素择优，而不是整层替换 B
//==============================================================================
//  【旧行为】pix_b_eff = osd_on ? osd_pix : pix_b;
//    只要 OSD 层开（菜单或信息条），**整幅画面**的前景都变成 osd_pix。
//    B 层在 OSD 矩形外是 "A=0 的黑"，于是那一大片直接透出 A 层
//    ⇒ 切换动画被擦掉，只剩底图。这也是"信息条只能在 B 关着时显示"的
//    历史原因（老代码为了不打架，干脆用 info_show 把两者做成互斥）。
//
//  ★★★ 2026-10-27：信息条要能在切换动画里显示 ⇒ 改成逐像素择优 ★★★
//    · OSD 像素 A==0  → 用 B 层（动画完全不受影响）
//    · OSD 像素 A!=0  → 用 OSD 像素，透明度交给合并级
//    · 背景（s2）由 s2_b 选：B 完全落屏的像素上，背景就是 B
//
//  ★★★ 2026-11-05：删掉「面板预混 B」，改成「把 B 接到 s2」★★★
//  【上板现象】开信息条后切换图片，面板被切成深浅两块：竖缝在右半边、
//    横缝在底部，分界线正好是当前帧 A/B 的接缝。
//  【根因】原来在 panel_on_b 时把面板预混成 {8'hFF,(3O+B)>>2} 喂给 s1，
//    指望"A=255 ⇒ 合并级直通"。但面板这档 alpha_eff 被钉成 192，合并级的
//    a9 = alpha = 192（≠256）⇒ 结果**又被 A 层混了 25%**：
//        A 区：0.5625P + 0.4375·A
//        B 区：0.5625P + 0.1875·B + 0.25·A
//    而动画期间某层的框外输出恒 0（ps_gen_image 的框外分支）⇒ B 区那 25%
//    的 A 是**纯黑**，面板背景掉到约一半亮度 ⇒ 深浅两块。
//  【改法】预混整段删掉，改为把 B 接到 s2（四宫格 2026-11-03 就是这么修的）：
//        s2 = B ⇒ 输出 = 0.5625P + 0.4375·B   （与 A 区同一种混法）
//    s1 只剩两选一，"透明 / 半透明 / 不透明"三种像素形态统一。
//  【门控为什么是 alpha_q==255】B 还没完全落屏时（S_ARM 的 alpha_b=0、
//    fx2 淡入 0..254）该像素的背景其实还是 A；此时切 B 会让"面板背景抢跑"。
//    四宫格 alpha_q = alpha_b | {8{quad_mode}} ≡ 0xFF ⇒ 条件自动成立，
//    所以不再单列 quad_mode。★ 不能放宽成 alpha_b[7]：fx2 后半段
//    非 OSD 像素的 s2 会变成 B，等于把交叉淡入硬切成 B。
//  【对齐】用 in_box_b_p（第 3 拍，与 pix_b 同拍）。旧码的 panel_on_b 用第 2 拍
//    的 in_box_b，在四宫格格子左沿 x0−1 那列会预混到 pix_b=0 ⇒ 那里有一列
//    偏暗；改用 _p 后消失。
//  【不变量】pix_a_eff 的 A 字节合并级不读（只用 24 位颜色），所以把 B
//    （框内 A=255 / 框外 A=0）接到 s2 不会引入 alpha 语义问题。
//  【代价】净省约 85 LUT（删 3 通道 (3O+B) 与一路 32 位 3:1 mux），0 寄存器。
//==============================================================================
wire [7:0]  osd_a   = osd_pix[31:24];
wire        osd_hit = osd_on & (osd_a != 8'd0);

// SEAM_DBG=1 的旧调试路径保留：没有 OSD 命中时把 B 强制纯红
wire [31:0] pix_b_dbg  = (SEAM_DBG == 1) ? 32'h00FF_0000 : pix_b;
// ★ 面板不再预混：所有 OSD 像素只输出自己（透明度由 alpha_eff 决定）
wire [31:0] pix_b_eff  = osd_hit ? osd_pix : pix_b_dbg;
// ★ 2026-10-30：A=192 的像素（信息条面板 / 缩放十字条）用**它自己的 alpha**。
//   【2026-11-05 复核】`osd_menu` 每拍无条件 `fade <= 255`（复位 0，一拍后
//    恒 255），所以"缩放 OSD 打开时 fade 还是 0"这条原始理由已经不成立了。
//    保留这一档的理由只剩观感：它让面板实际停在 56.25%（合并级 a9=144）
//    而不是 74.6%。★ 想回到设计意图的 75%，把 `(osd_a==8'd192)?192:` 删掉
//    即可（0 风险、省约 10 LUT），但菜单底板/页面板/十字条会一起变实，
//    必须重刷黄金图 ⇒ 单独一笔，别混在这次的信息条修复里。
wire [7:0]  alpha_eff = osd_hit ? ((osd_a == 8'd192) ? 8'd192 : osd_fade) : alpha_q;
// ★ 2026-11-03 四宫格 / 2026-11-05 信息条：格子里的"底"和信息条的"底"都是 B。
//   菜单遮罩（osd_fade 那一档）和未选中格的压暗都是"OSD 混到 s2 上"；s2 若仍是 A，
//   格子里就变成"遮罩 + 相册"，B 整个被 OSD 像素顶掉（仿真：图 2/3/4 透出相册）。
//   换成 B 后与原来"四宫格在 A 层"时逐像素同一种混法；格子外仍是相册。
//   （s2 的 alpha 字节合并器不读，所以实际是 24 位 + 1 位的选择。）
//   ★ 只有 B **完全落屏**（alpha_q=0xFF）时才能把 B 当背景：S_ARM 与 fx2 淡入
//     期间 B 还没盖住画面，此时背景仍是 A。四宫格 alpha_q 恒 0xFF ⇒ 自动成立。
wire        s2_b      = in_box_b_p & (alpha_q == 8'hFF);   // ★ [INBOX-P] 用与 pix 同拍的那一份
wire [31:0] pix_a_eff = (SEAM_DBG == 2) ? 32'h0000_00FF : (s2_b ? pix_b : pix_a);

//==============================================================================
//  6. 合并 + 输出
//==============================================================================
wire [31:0] merge_pix;
wire        merge_v;

ps_merge_alpha_pp u_merge (
    .clk(clk), .rst(rst), .ce(pixel_ready_i),
    // ★ valid 只进 out_valid，而 out_valid(=merge_v) 全工程没人消费 ⇒
    //   合成时被剪掉，改这些线不会影响任何一个像素。这里保留真实连接只为
    //   将来真有人用它时语义正确（原来那句"否则信息条会被挖个洞"是错的：
    //   合并级的 pix 每拍 ce 无条件更新，与 valid 无关）。
    .s1_pix(pix_b_eff), .s1_valid(v_b | osd_hit),
    .s2_pix(pix_a_eff), .s2_valid(v_a),
    .alpha(alpha_eff),
    .pix(merge_pix), .out_valid(merge_v)
);

ps_to_rgb24 u_out (
    .pix(merge_pix),
    .rgb24(ps_data)
);

// 扫描坐标（OSD 图层与其它图层吃同一根坐标总线，保证像素严格对齐）
assign scan_x = sx;
assign scan_y = sy;

endmodule
