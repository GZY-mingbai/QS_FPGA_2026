//==============================================================================
//  img_layer_ctrl.v —— 图片图层的封装：ps_gen_image + 两个异步 FIFO
//==============================================================================
//
//  【在架构里的位置】
//
//      场景图侧（50MHz）                      SDRAM 读仲裁侧（100MHz，2026-10-19 降频）
//      ─────────────────                     ──────────────────────
//      (x,y,adv) ──┐
//                  ├─► ps_gen_image ─ req_push/req_addr ─► 请求FIFO ─► req_vld/req_addr
//      cfg ────────┘        │                                  ◄─ req_ack
//                           └─ pix/pop ◄── 像素FIFO ◄─ pix_wr/pix_data
//
//  【为什么两个 FIFO 都要异步】
//  请求 FIFO：写侧 50MHz（图层产生请求）、读侧 100MHz（仲裁器取走）
//  像素 FIFO：写侧 100MHz（仲裁器写回）、读侧 50MHz（图层消费）
//  两边都是多比特数据流，跨时钟域必须用异步 FIFO。
//
//  【深度】像素 FIFO 的深度必须 **明显大于** ps_gen_image 的 LEAD（预取领先量），
//  否则预取出来的像素没地方放、被 wfull 静默丢掉。经验关系：深度 ≥ 2 × LEAD。
//  当前 深度 = 128（AFIFO_AW=7）、LEAD = 64，正好一倍余量。
//
//  【接口说明】
//  仲裁侧故意做成"show-ahead + 单拍 ack"的简单形式：
//      req_vld 高 = 队首有一个有效请求，req_addr 就是它
//      req_ack 拉高一拍 = 取走它
//  这样仲裁器不需要知道 FIFO 的内部结构。
//
//==============================================================================

module img_layer_ctrl_ref #(
    parameter integer IMG_W    = 640,
    parameter integer IMG_H    = 480,
    parameter integer SRC_W    = 640,
    // ★★ 2026-10-16 上板修复（第二版）★★
    //   第一版把 LEAD 和 FIFO 都做成 128，结果更糟 —— 因为 LEAD == 深度 时，
    //   预取刚好把像素 FIFO 填满，之后每个到达的像素都被 wfull 挡掉
    //   （本模块像素 FIFO 的 wfull 是悬空的 = **静默丢弃**），丢像素 → 图片层卡住。
    //   正确关系：LEAD ≤ 深度 / 2，所以 LEAD = 64、深度 = 128（AFIFO_AW=7）。
    //
    //  ★ 100M-16：此注释已过期（review 问题 10）—— app_demo_scene 的 IMG_GATE=1，
    //    图片层【参与】全局流控：任一层没货 ⇒ pixel_ready 拉低 ⇒ app_scan_gen
    //    停走 ⇒ 外部 x,y 一起冻住。ps_gen_image 的 pop_stall 只补 pixel_ready
    //    传播那 1~2 拍的窗口。两者是同一套假设（一次有效推进 = 所有层各进一步）；
    //    谁要是把 IMG_GATE 关成 0，弹空抑制会失效、画面必花。
    //
    //   资源：两个 FIFO 都是寄存器实现，64 深 ×(21+32) 位 ≈ 3.4K 触发器/层。
    parameter integer LEAD     = 384,  // 2026-10-24: 64 太小（一行 160 字），预取不足 = 固定 x 全高竖条
    parameter integer AFIFO_AW = 10,        // 2026-10-24: 1024 深（≥ 2×LEAD 512）
    parameter integer ALIGN_ONLY = 0,       // ★ 诊断：1 = 请求地址强制 4 字对齐（见 ps_gen_image）
    // ★ 2026-10-27：透传给 ps_gen_image 的采样中心开关（见那里的说明）
    parameter integer SAMPLE_CENTER = 1
)(
    // ---- 场景图侧（clk = sd_card_clk = 50MHz）----
    input  wire        clk,
    input  wire        rst,
    input  wire        pipe_ce,             // shared scene/output pipeline advance
    input  wire        adv,                 // = streaming
    input  wire        frame_active,        // = ps_start
    input  wire [15:0] x,
    input  wire [15:0] y,
    input  wire [31:0] cfg_pos,
    input  wire [31:0] cfg_size,
    input  wire [39:0] cfg_k,
    input  wire [19:0] cfg_ori,         // ★ [ZOOM-EXACT] {ORIX[9:0], ORIY[9:0]} 锚点（透传给 ps_gen_image）
    input  wire [20:0] tex_base,
    input  wire        layer_en,
    input  wire        quad_en,          // ★ 四宫格（透传给 ps_gen_image）
    input  wire [3:0]  quad_vm,          // ★ 四宫格有效槽（透传）
    output wire [31:0] pix,
    output wire        out_valid,
    output wire        layer_ready,
    output wire        in_box_o,        // ★ 这一列是否归本层负责（两层解耦用）
    output wire [4:0]  dbg_delta,       // ★ 帧末 (push−pop) 快照（诊断，5 位）
    output wire [3:0]  dbg_evt,         // ★ 帧内粘滞事件标志
    output wire [127:0] smp_bus,        // ★ 像素轨迹探针
    output wire        smp_win,         // ★ 采样窗口
    output wire        pix_empty_o,     // ★ 像素 FIFO 真实空标志（同 HD 层）
    // ★ 2026-10-17 虚线根修 A/B 开关（透传给 ps_gen_image，见其文件头）
    input  wire        no_underrun_fix,
    input  wire [3:0]  exp_mode,        // ★ 10 模式诊断选择（key1 轮换）★

    // ---- ★ 2026-11-01 [FIFO-SHARE]：外部客户（img_layer_hd）借用本层的两个 FIFO ----
    //  ext_sel=1 时请求 FIFO 的写口、像素 FIFO 的读口改由外部驱动（ps_gen_image
    //  此时 layer_en=0，不产生请求也不消费像素）。两者在帧内互斥（a_hd 只在帧边界改，
    //  帧间 flush 清空两个 FIFO），所以 HD 层不必再自带一套 FIFO。不用时 ext_sel 接 0。
    input  wire        ext_sel,
    input  wire        ext_push,        // 一个 4 字组请求（掩码恒 1111）
    input  wire [20:0] ext_addr,
    output wire        ext_wfull,       // 请求 FIFO 满
    output wire        ext_wrst,        // 请求 FIFO 写口仍在复位（此时不许 push）
    input  wire        ext_pop,
    output wire [31:0] ext_head,        // 像素 FIFO 队首（show-ahead）
    output wire        ext_rempty,

    // ---- SDRAM 读仲裁侧（mclk = ext_mem_clk = 100MHz）----
    input  wire        mclk,
    output wire        req_vld,
    output wire [20:0] req_addr,
    output wire [3:0]  req_mask,        // ★ build F：这个 4 字组里要哪几个字（跟着地址一起过 FIFO）★
    input  wire        req_ack,
    input  wire        pix_wr,
    input  wire [31:0] pix_data
);

//==============================================================================
//  ★ 帧边界必须把两个 FIFO 都清空 ★
//==============================================================================
//  踩过的坑：不清空的话，上一帧帧尾残留在像素 FIFO 里的像素（最多 LEAD 个）
//  会被下一帧当成"这一帧的前几个像素"用掉，整幅图错位。
//
//  用 frame_active(=ps_start) 的反相当清空信号：帧间空闲期它一直有效，
//  所以连"仲裁器里还在飞的请求回来后写进来的那几个像素"也会被反复冲掉，
//  到帧真正开始时 FIFO 一定是干净的。
//
//  电平信号跨时钟域直接当异步复位用是安全的 —— 它在帧间要持续几十万拍。
//==============================================================================
wire flush = ~frame_active;

// ★ 100M-16：flush 按各自时钟域同步释放（review 问题 13）
//   flush 置位保持异步（立即清 FIFO 指针），释放经过本域两级同步。
//   50MHz 的帧边界信号直接掀掉 100MHz 域 FIFO 的复位释放时序，指针/空满
//   标志的恢复可能不满足各自时钟域的恢复时间要求 ⇒ 偶发帧首脏数据、
//   请求/像素计数异常，而常规仿真不一定复现。
//   （标准结构：异步置位、按域同步释放；两个域各一套。）
reg fl_c1, fl_c2;      // clk 域（请求 FIFO 写侧 / 像素 FIFO 读侧）
reg fl_m1, fl_m2;      // mclk 域（请求 FIFO 读侧 / 像素 FIFO 写侧）
always @(posedge clk or posedge flush or posedge rst) begin
    if (rst || flush) begin fl_c1 <= 1'b1; fl_c2 <= 1'b1; end
    else               begin fl_c1 <= 1'b0; fl_c2 <= fl_c1; end
end
always @(posedge mclk or posedge flush or posedge rst) begin
    if (rst || flush) begin fl_m1 <= 1'b1; fl_m2 <= 1'b1; end
    else               begin fl_m1 <= 1'b0; fl_m2 <= fl_m1; end
end

//------------------------------------------------------------------------------
//  ★★ 2026-10-30 [RST-SYNC]：`rst` 本身也必须按域"同步释放" ★★
//------------------------------------------------------------------------------
//  【原来】`frst_c = rst | fl_c2`、`frst_m = rst | fl_m2`。
//     `fl_*` 是同步过的，但 **`rst` 这一项是原始异步信号**（顶层 rst_all 由
//     50MHz 的 21 位计数器比较产生，扇出 200+）⇒ 释放边沿直接落到
//     mclk(100MHz) 域的 BRAM `wea`/寄存器 `sr` 上 ⇒ 布线后 recovery 违例
//     −7.5ns × 390 个端点（TD 报的就是这一组）。
//  【现在】`rst` 也只做"异步置位、本域同步释放"，两个域各一套 3 级同步器：
//     第 1 级允许亚稳态（recovery 违例就止步于此），第 2/3 级把释放对齐到本域。
//     这既让 390 个 `wea` 端点变成"由本地寄存器驱动"，也让复位释放天然有序。
//  【代价】各域复位释放晚 3 拍（50MHz 域 60ns），对功能无影响。
//------------------------------------------------------------------------------
reg [2:0] rstc_sr, rstm_sr;
always @(posedge clk or posedge rst) begin
    if (rst) rstc_sr <= 3'b111;
    else     rstc_sr <= {rstc_sr[1:0], 1'b0};
end
always @(posedge mclk or posedge rst) begin
    if (rst) rstm_sr <= 3'b111;
    else     rstm_sr <= {rstm_sr[1:0], 1'b0};
end
wire rst_c = rstc_sr[2];      // clk  域：同步释放后的复位
wire rst_m = rstm_sr[2];      // mclk 域：同步释放后的复位

wire frst_c = rst_c | fl_c2;  // clk 域复位：异步置位、同步释放
wire frst_m = rst_m | fl_m2;  // mclk 域复位：异步置位、同步释放

//==============================================================================
//  内部连线（必须先声明 —— 否则会被当成 1 位隐式线网）
//==============================================================================
wire        req_push;
wire [19:0] push_w_o;      // ps_gen_image 的本帧提交字数
wire [20:0] req_addr_int;
wire [3:0]  req_mask_int;           // ★ build F：{掩码, 地址} 一起过请求 FIFO ★
wire        pix_pop;
wire        req_wfull;
wire [24:0] req_head;
wire        req_rempty;
wire [31:0] pix_head;
wire [AFIFO_AW:0] pix_count;        // ★ build C：跟 AFIFO_AW 走（256 需 9 位，8 位会在 256 处回绕成 0）★
wire        pix_wfull;              // ★ build E：像素 FIFO 真满（原来悬空 = 静默丢弃）
wire        pix_rempty;             // ★ build E：像素 FIFO 真空
assign pix_empty_o = pix_rempty;     // ★ 透传给 ps_gen_image（与 HD 层同源）

//==============================================================================
//  请求 FIFO：50MHz 写 → 100MHz 读
//==============================================================================
//  ★ build F：位宽 21 → 25（地址 21 位 + 组掩码 4 位）★
//  掩码必须和地址**一起过 FIFO**：否则仲裁器拿到地址时不知道要这组里的哪几个字。
//
//  ★★★ 2026-10-19 build E：请求 FIFO 加【寄存器输出】（skid buffer）★★★
//------------------------------------------------------------------------------
//  【原来】
//      wire [24:0] req_head = mem[rbin];     // 25 位 × 256 深，**异步（组合）读**
//      assign req_addr = req_head[20:0];     // → 调度器 t_pre_a
//      assign req_mask = req_head[24:21];    // → 调度器 t_pre_m
//  这是一条**无寄存的 25 位总线**，上面同时捆着两个语义完全不同的字段：
//      · 地址 [20:0]  —— "读哪里"
//      · 掩码 [24:21] —— "这一组要几个字"
//  布局布线报告实测这条路 slack = **+0.001ns**（`u_img1/u_req_fifo/rbin` →
//  `u_sched/t_pre_a`）—— 一皮秒。25 位每一位走线延迟不同，同一个寄存器沿
//  采样时可能出现"地址已经是新的、掩码还是旧的"这种半新半旧的组合：
//      · 地址位错 ⇒ 这一组 4 个字读到别处（纹理区外）⇒ 4~8 像素的**脏点块**
//      · 掩码位错 ⇒ 这一组"要几个字"算错 ⇒ 像素 FIFO 里多/少几个字
//                  ⇒ 显示侧从那一刻起**永久错位**，逐帧累积 ⇒ **彩色水平条带**
//  这是唯一一处把"读哪里"和"要几个"捆在同一条无寄存总线上的地方，
//  也正好是"脏点 + 条带"这两个现象能同源的那一处。
//
//  【改法】FIFO 本体不动，只在读侧加一级寄存器输出：
//      · 输出寄存器空着、且 FIFO 非空 ⇒ 把队首搬进来（同时弹 FIFO）
//      · 被调度器 req_ack 消费    ⇒ 清空，下一拍再搬
//  ⇒ 地址和掩码从此是**在同一个寄存沿上一起变**的，物理上不可能半新半旧。
//  ⇒ 代价：每个请求晚 1 拍出现。请求最快 4 拍一次（tx_start 只在 ph[1:0]==0），
//     多这 1 拍不会降低吞吐（实测路径见 PLAN §9.x 的 TX 带宽预算 1.3 倍余量）。
//==============================================================================
// ---- 输出寄存器（先声明：下面 FIFO 的 .re 要用它）----
reg  [24:0] head_q;
reg         head_v_q;
wire        req_fifo_re = (!head_v_q) && (!req_rempty);   // 搬进来（并弹 FIFO）

img_afifo_ref #(
    .DW (25),
    .AW (AFIFO_AW),
    // ★ 100M-29：只有**请求** FIFO 打开额外写流水级 —— 修这条实测保持违例：
    //     u_img1/u_req_fifo/waddr_q_reg[7] → u_req_fifo/ramread0.addra[11]
    //     HWNS −0.011ns / HTNS −0.022ns
    //   像素 FIFO 保持 WSTAGE=0：它的写延迟是**载荷语义**（pix_head 对齐），
    //   多一拍会让整幅图错位，不能顺手开。
    .WSTAGE (1)
) u_req_fifo (
    .wclk  (clk),
    .wrst  (frst_c),
    .we    (ext_sel ? ext_push : req_push),
    .din   (ext_sel ? {4'hF, ext_addr} : {req_mask_int, req_addr_int}),
    .wfull (req_wfull),

    .rclk  (mclk),
    .rrst  (frst_m),
    .re    (req_fifo_re),          // ★ 只在"把队首搬进输出寄存器"那一拍弹 ★
    .dout  (req_head),
    .rempty(req_rempty),
    .rcount()
);

//------------------------------------------------------------------------------
//  ★ 输出寄存器（skid buffer）：地址 + 掩码同沿稳定 ★
//------------------------------------------------------------------------------
always @(posedge mclk or posedge frst_m) begin
    if (frst_m) begin
        head_q   <= 25'd0;
        head_v_q <= 1'b0;
    end else if (!head_v_q) begin
        if (!req_rempty) begin
            head_q   <= req_head;      // 队首 → 寄存器
            head_v_q <= 1'b1;
        end
    end else if (req_ack) begin
        head_v_q <= 1'b0;              // 被消费，下一拍再搬
    end
end

assign req_vld  = head_v_q;
assign req_addr = head_q[20:0];
assign req_mask = head_q[24:21];

//==============================================================================
//  像素 FIFO：100MHz 写 → 50MHz 读（show-ahead）
//==============================================================================
img_afifo_ref #(
    .DW (32),
    .AW (AFIFO_AW)
) u_pix_fifo (
    .wclk  (mclk),
    .wrst  (frst_m),
    .we    (pix_wr),
    .din   (pix_data),
    .wfull (pix_wfull),

    .rclk  (clk),
    .rrst  (frst_c),
    .re    (ext_sel ? ext_pop : pix_pop),
    .dout  (pix_head),
    .rempty(pix_rempty),
    .rcount(pix_count)
);

assign ext_wfull  = req_wfull;
assign ext_wrst   = frst_c;
assign ext_head   = pix_head;
assign ext_rempty = pix_rempty;

//------------------------------------------------------------------------------
//  ★ build C：ovf 只保留"本帧实时"★
//  build B 的跨帧最大值已经回答了问题：**从复位起每帧最大丢字数 = 0**，
//  所以"写满丢字"不是病因，那个跨帧最大值就不必再占数码管。
//  保留本帧实时位作为回归哨兵：下次动深度/节流时它会第一个发言。
//------------------------------------------------------------------------------
reg ovf_w;
always @(posedge mclk or posedge rst_m) begin
    if (rst_m)       ovf_w <= 1'b0;
    else if (flush)  ovf_w <= 1'b0;
    else if (pix_wr && pix_wfull) ovf_w <= 1'b1;
end

//==============================================================================
//  ★★ 2026-10-24：把 ovf_w（本帧"像素 FIFO 写满拒绝写入"粘滞位）送上数码管 ★★
//==============================================================================
//  【为什么现在必须看它】用户的定性判断把方向钉死了：
//      画面"向左移 4px" ⇔ **FIFO 里少了一个字**（多一个字会向右移）
//    而 push_cnt（请求字数）是对的（左6=0），所以只能是**物理上少进了一个**。
//    唯一会"静默丢字"的地方就是 img_afifo 写满：`pix_wr && pix_wfull` 时写入被拒，
//    而我们的流控并不会因此回滚"已承诺字数" ⇒ 之后整条流永久错位。
//  【接线】ps_gen_image 的 dbg_evt（0~7 是 |push−budget|，最多到 7）
//    与 ovf_w 做 OR 放 bit3 ⇒ 数码管读到 ≥8 就说明**本帧确实丢过字**。
//  【历史】注释记录过"build B 跨帧最大值 = 0"，但那是旧配置（更小的闸门/更深的 FIFO）；
//    现在闸门 500、FIFO 512、余量只有 16 字，必须重新实测。
//==============================================================================
wire [3:0] dbg_evt_i;
assign dbg_evt = dbg_evt_i | {ovf_w, 3'b000};   // bit3 = 本帧丢过字
// ★ build F2：layer_dead 由 ps_gen_image 用**端口**送出来 ★
//   【踩过的坑】第一稿写的是 `u_gen.layer_dead`（跨模块层次引用）——
//   iverilog 允许，但 TD 综合直接报
//   "ERROR: external reference 'u_gen.layer_dead' remains unresolved"。
//   层次引用只在仿真里成立，综合必须走端口。
wire dead_r;                        // 本帧此刻是否 dead（实时）
wire dead_seen_r;                   // ★ 跨帧粘滞：是否被 give_up 放弃过
wire [7:0] byte_r;                  // ★ build 7：{[7:4]=传输偏移数, [3:0]=洞/纯垃圾数}★
wire       bneq_r;                  // ★ build 7：出现过任何错配（粘滞）★
// ★ build 7：diag[15:0]（每层 16 位）★
//  [15:12] = 状态位 {dead_seen, bneq, ovf, dead}（恒可见，与 rsel 无关）
//  [11:0]  = rsel 选中的读数
//  ★ 100M-16：低位四位的旧布局描述作废（从未实现），见 diag 端口注释
// rsel 0..3：本模块的四个字数读数；rsel 4..15：交给 ps_gen_image 的 16 档读数
// ★ 100M-16：高 4 位恒为状态位（review 问题 18）—— 端口注释里承诺的
//   [3]=dead_seen / [2]=bneq / [1]=ovf / [0]=dead 四位以前从未接过线，
//   读 0 不能证明"没发生过"。现在高 4 位固定输出这四位，低 12 位仍是读数。

//==============================================================================
//  图层本体
//==============================================================================
ps_gen_image_ref #(
    .IMG_W (IMG_W),
    .IMG_H (IMG_H),
    .SRC_W (SRC_W),
    .LEAD  (LEAD),
    .ALIGN_ONLY (ALIGN_ONLY),
    .SAMPLE_CENTER (SAMPLE_CENTER)
) u_gen (
    .clk         (clk),
    .rst         (rst_c),
    .pipe_ce     (pipe_ce),
    .adv         (adv),
    .frame_active(frame_active),

    .x           (x),
    .y           (y),

    .cfg_pos     (cfg_pos),
    .cfg_size    (cfg_size),
    .cfg_k       (cfg_k),
    .cfg_ori     (cfg_ori),
    .tex_base    (tex_base),
    .layer_en    (layer_en),
    .quad_en     (quad_en),
    .quad_vm     (quad_vm),

    .req_push    (req_push),
    .req_addr    (req_addr_int),
    .req_mask    (req_mask_int),
    // ★★★ 2026-10-25 根因修复：请求 FIFO 复位窗口内**绝不 push** ★★★
    //  【病根】`frst_c = rst | fl_c2` 是请求 FIFO 的写口复位（:118），
    //    `flush` 后还要保持 **2 拍**才释放（:111-112、:185）。
    //    而原来 `req_ready` 只看 `~req_wfull`，**完全没看复位**：
    //      帧起点 `ps_start` 上升后那 2 拍里，`flush_do` 可能刚好成立 ⇒
    //      `req_push` 拉高、FIFO 的 `we` 也拉高，但写口仍在 `wrst` 里
    //      ⇒ **这个写被静默吞掉**（img_afifo.v: 复位期间 `wbin` 不动），
    //      而 `cur_g/cur_mask` 照常前进、`push_cnt/push_w` 也照常累加。
    //    ⇒ 请求侧记了账、FIFO 里没有字 ⇒ `push−pop = +1`
    //    ⇒ 显示侧从帧首起每个字都早一位 ⇒ **整幅左移 1 个字（4 屏幕列）**
    //    ⇒ 行末取到下一行的字 0 = "图片最左边那一小段跑到框最右侧"
    //    ⇒ 该字"等 4ms 也不回来"（它从未进过请求队列，不是 SDRAM 丢命令）。
    //  【为什么只有 fx0 撞上】fx0 的 X0 = −px ⇒ 行首可见字常落在 4 字组的
    //    第 3/2/1 位 ⇒ 第一组只含 1~3 个字 ⇒ 组变化发生在 rq_x 很小的时候，
    //    正好落在复位那 2 拍里。fx1（X0=0）第一组是满的，要到 rq_x=16 才换组，
    //    早就出了复位窗口 ⇒ 从不触发。
    //  【参照】HD 层早就修过同一个 bug：img_layer_hd.v:101-105
    //    「否则前 2 拍写进去的请求被 wrst 吃掉，rq_grp 却照涨 ⇒ 整幅图右移 8 像素」。
    //  【修法】把复位并进 `req_ready`：`rq_go` 与 `flush_do` 都含它 ⇒
    //    请求扫描与 flush 都会等复位释放再动，一个掩码位都不会丢。
    .req_ready   (~req_wfull & ~frst_c),

    .pix_head    (pix_head),
    // ★★ 2026-10-23：FIFO **真的空**时，把 `pix_count` 强制报 0 ★★
    //  【病根】ps_gen_image 用 `pix_avail = (pix_count != 0)` 决定"这一拍能不能弹字"，
    //    而 `rcount` 是**跨时钟同步后的保守值（滞后约 2 拍）**（见 ps_gen_image.v:1179
    //    自己写的警告："千万别用 pix_count 判断这一拍的 pix_head 有没有效"）。
    //    B 层在横向平移时每行只有几个像素可见 —— 每个框入口它都会**恰好把 FIFO 抽空一次**，
    //    这 2 拍滞后就可能让它"看起来还有 1 个字" ⇒ 弹出一个**已经不存在**的字
    //    ⇒ B 的整条窄带错一个源字 ⇒ 屏幕上是"交界处一条该层颜色的竖条"，px 每帧变 ⇒ 闪烁。
    //  【改法】用与读域同拍的 `pix_rempty` 兜底：真空就报 0，滞后窗口消失。
    //    只影响"实际已空"的那几拍，有数据时逐位不变。
    .pix_count   (pix_rempty ? {(AFIFO_AW+1){1'b0}} : pix_count),
    .pix_pop     (pix_pop),

    .pix         (pix),
    .out_valid   (out_valid),
    .layer_ready (layer_ready),
    .in_box_o    (in_box_o),
    .dbg_delta   (dbg_delta),
    .dbg_evt     (dbg_evt_i),      // ★ 2026-10-24：先接内部，再与控制器的 ovf 合并
    .smp_bus     (smp_bus),
    .smp_win     (smp_win),
    .layer_dead_o(dead_r),
    .layer_diff_o(byte_r),       // ★ build 3：第一个错配字的低字节★
    .layer_div_o (bneq_r),       // ★ build 3：R≠G≠B 发生过★
    .layer_neq_o (dead_seen_r),  // ★ build 9：改为"是否死过（跨帧粘滞）"★
    .no_underrun_fix (no_underrun_fix),
    .pix_empty       (pix_empty_o),
    .exp_mode        (exp_mode),
    .push_w_o        (push_w_o)
);

endmodule
