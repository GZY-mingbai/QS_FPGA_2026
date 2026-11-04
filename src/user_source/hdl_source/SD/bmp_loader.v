//==============================================================================
//  bmp_loader.v —— SD 卡 BMP → 缩放 → SDRAM（读卡/头解析/自动裁剪/写口 一个模块）
//==============================================================================
//
//  【它替掉谁】`sd_bmp_loader.v`(2251 LUT) + `bmp_read.v` + `bmp_scale.v`。
//
//  【和旧版的三处结构性差别】
//    ① 没有任何"目标区地址加法"：`bmp_view` 直接吐**绝对字地址**，
//       本模块 `wr_addr = fifo_dout[52:32]` 原样送仲裁器。
//    ② 没有预清（旧版装载前把整片纹理刷成洋红"显影洞"）：新渲染器把
//       `dst_base .. dst_base+N-1` **每个字都写一次**（黑边写 0），洞不存在。
//    ③ 行是**按需**读的：`bmp_view` 说"我要第 k 行"，本模块 seek 到那一行、
//       只发 `[x_le, x_ri)` 这段像素；窗口外的行列不占带宽、也不写。
//
//  【内部流程】
//      start → 读扇区 0 → 解析头(W/H/pixel_offset) → 启动 bmp_view（算几何）
//            → 每收到一次 row_go：seek 到该行窗口首字节 → 喂到窗口末 → 回 L_WROW
//            → view done → 排空写 FIFO + 等写沉降 → done
//
//  【★ 为什么"每行都重新 seek"】
//      bmp_view 请求的行号是**按缩放比跳着来的**（放大跳行、缩小复用行），
//      所以不能像旧版那样 seek 一次然后顺序流到底。每行 seek 的代价是
//      `H-1-row_req` 次 `acc += stride`（480 行最坏 11.5 万拍 ≈ 2.3 ms），
//      换来的是任意缩放比/任意视口都成立。seek 之后字节流正好从 `x_le*3` 开始，
//      于是"窗口内"是构造保证的，不需要行/列两级比较器（省两个 16 位比较）。
//
//  【流控】一个扇区 ≈170 像素；`bf_fifo` 深 256 ⇒ 只在 `bf_cnt < 80` 时才发
//    下一个扇区请求，最坏水位 80+171 = 251 < 256，永不丢字。
//==============================================================================
module bmp_loader #(
    parameter integer AFIFO_AW = 9,
    parameter integer BF_AW    = 8
)(
    input  wire        clk,             // 50MHz（SD / 像素域）
    input  wire        rst,
    input  wire        mclk,            // 125MHz（写口域）

    // 装载请求
    input  wire        start,
    // ★ 2026-11-04：放弃本次装载（翻页 / 主页连切）。不再读卡，改用**黑像素快进**：
    //   渲染器照常把整幅写完（剩下的全是黑），然后正常排空 + done，并同拍报 err
    //   （调用方都把 err 当"这张没装成"）。为什么不当场停：SDRAM 装载写口
    //   攒够 4 个字才取，中途停下写 FIFO 里会剩 1~3 个字永远排不空（排空要 1s 超时）；
    //   整幅的字数都是 4 的倍数，写完就一定排得空。被放弃的那格顺带变成纯黑。
    input  wire        abort,
    input  wire [31:0] file_offset,     // BMP 在卡里的字节偏移（512 对齐）
    input  wire [31:0] file_length,     // 文件字节数（兜底）
    input  wire [20:0] dst_base,        // 目标区绝对首字地址

    // 画布与视口
    input  wire [1:0]  mode,            // 0=640×480 1=320×240 2=160×120
    input  wire        fmt,             // 1=888(1px/字) 0=565(2px/字)
    input  wire [10:0] dst_stride,      // 0 = 自动
    input  wire [7:0]  zoom,            // Q4.4
    // ★ 2026-10-30：cam/cam_auto 换成屏幕锚点 (ax,ay)。
    //   cam 由 bmp_view 内部用 `cam = W/2 + (ax-320)*(step_fit-step)` 推出，
    //   所以装载器不再需要"源图中心/指定点"二选一，也没有跨装载的几何状态。
    input  wire [9:0]  ax, ay,          // Q0 屏幕坐标（= crx/cry）

    // 状态
    output wire        ready,
    output reg         busy,
    output reg         done,
    output reg         err,
    // ★ 2026-11-02【BEEP-ABORT】：本次 err 是"被翻页/连切放弃"（ab_r）造成的，
    //   不是真的装载失败。语义上与 err 同拍，**只给提示音用**（顶层据此屏蔽错误音）；
    //   控制逻辑（viewer_ctrl / quad_page_ctrl）仍然只看 err，行为一位不变。
    output reg         err_abort,
    output wire [15:0] src_w_o, src_h_o,

    // SD 扇区客户口
    output reg         sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end,
    input  wire        sd_init_done,

    // SDRAM 写口
    output wire        wr_vld,
    output wire [AFIFO_AW:0] wr_used,
    output wire [20:0] wr_addr,
    output wire [31:0] wr_data,
    input  wire        wr_ack,
    input  wire        wr_settled
);

localparam [3:0] L_IDLE = 4'd0, L_HDR = 4'd1, L_SEED = 4'd2, L_WROW = 4'd3,
                 L_SEEK = 4'd4, L_STRM = 4'd5, L_DRN = 4'd6, L_DONE = 4'd7,
                 L_ERR  = 4'd8;

reg [3:0] st;

//------------------------------------------------------------------------------
//  0. 参数锁存
//------------------------------------------------------------------------------
reg [31:0] foff, flen;
reg [20:0] dbase;
reg [1:0]  mode_r;
reg        fmt_r;
reg [10:0] dstride;
reg [7:0]  zoom_r;
reg [9:0]  ax_r, ay_r;

//------------------------------------------------------------------------------
//  1. 文件头
//------------------------------------------------------------------------------
reg [5:0]  hcnt;            // 扇区内字节号 0..33
reg [7:0]  hb0, hb1;
reg [31:0] flen32, poff32, w32, h32, comp32;
reg [15:0] bits16;
reg [15:0] stride;          // 行字节数（含 4 字节填充），W=4096 时 12288

wire hdr_match = (hb0 == "B") && (hb1 == "M") &&
                 (w32 != 32'd0) && (w32 <= 32'd4096) &&
                 (h32 != 32'd0) && (h32 <= 32'd4096) &&
                 (bits16 == 16'd24) && (comp32 == 32'd0);

assign src_w_o = w32[15:0];
assign src_h_o = h32[15:0];

//------------------------------------------------------------------------------
//  2. 字节流状态
//------------------------------------------------------------------------------
reg [31:0] sec;             // 当前请求的扇区号
reg        rd_on;           // 已发请求、还没收到 end
reg [1:0]  bidx;            // 像素内字节序 0=B 1=G 2=R
reg [7:0]  pb, pg;
// ★ 2026-10-29：原来用 bsec(已收字节数) + first_sec(还是本行首扇区) 判"跳过行首
//   那截"。两者都不成立：`first_sec` 在 L_SEEK 等上一扇区结束时会被 end 清掉
//   （竞态，等于每行都跳过 0 字节 ⇒ 整行左移 skip/3 列，实测 0x781a/0x780a
//   正好是 skip=438 丢失后的那两个平均值）；`bsec` 靠 dv 间隙归零，真机上
//   dv 若有毛刺也会错位。改成**递减计数器**：进 L_STRM 时装载，每收一个字节减 1，
//   减到 0 就开始取像素。不依赖任何握手/间隙，且比原来少一个 10 位计数器 + 比较器。
reg [9:0]  skip_left;       // 还要丢掉的字节数（行首偏移）
reg [12:0] pcnt;            // 本行已组装的像素数（0..win_w）

reg [31:0] seek_acc;        // 目标行窗口首字节的文件内偏移（相对像素区）
reg [11:0] seek_k;          // 还要加几次 stride
reg        row_go_s;        // row_go 的粘滞标志（loader 可能晚几拍才回到 L_WROW）


reg         view_start;
// 写 FIFO 快满 → 冻结渲染器冲刷（留 12 字余量抵消 we/addr 的 1 拍滞后）
wire        fifo_stall;
// 视口中心改由 bmp_view 从锚点推出（本模块只透传 ax_r/ay_r）
wire        view_pix_rd;
wire [11:0] row_req, win_x0;
wire [12:0] win_w;
wire        row_go;
wire        view_busy, view_done;
wire        v_we;
wire [20:0] v_wa;
wire [31:0] v_wdata;

//------------------------------------------------------------------------------
//  3. 像素 FIFO（24bit × 256，块 RAM + 写旁路）
//------------------------------------------------------------------------------
(* ram_style = "block" *) (* ramstyle = "block" *) (* ram_syn = "block" *)
reg  [23:0] bf_mem [0:255];
reg  [23:0] bf_ram;
reg         bf_hit_r;
reg  [23:0] bf_dat_r;
reg         bf_dv;
reg  [8:0]  bf_wp, bf_rp;

wire [23:0] pix_in;
reg         pix_wr;

wire [8:0] bf_cnt  = bf_wp - bf_rp;
wire        bf_full = (bf_cnt >= 9'd256);
wire        bf_we   = pix_wr && !bf_full;
wire        bf_pop  = view_pix_rd && bf_dv;
// ★ 2026-10-30：连续流下 loader 会多读几行（不再逐行等 row_go），bmp_view 收工后
//   那些像素没人消费 ⇒ `bf_cnt == 0` 永远不成立 ⇒ L_DRN 卡到超时、done 不产生
//   （实测 tb 报 timeout(st=6)，但 307200 个像素内容全对）。
//   drain 期间把读指针直接追到写指针，等价于"丢弃多余预读"。
reg         bf_clr;
always @(posedge clk or posedge rst) begin
    if (rst) bf_clr <= 1'b0;
    else     bf_clr <= (st == L_DRN);
end
wire [8:0]  bf_rp_n = bf_clr ? bf_wp : (bf_rp + (bf_pop ? 9'd1 : 9'd0));
wire        bf_hit  = bf_we && (bf_wp == bf_rp_n);

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bf_wp <= 9'd0; bf_rp <= 9'd0; bf_ram <= 24'd0;
        bf_hit_r <= 1'b0; bf_dat_r <= 24'd0; bf_dv <= 1'b0;
    end else begin
        if (bf_we) begin
            bf_mem[bf_wp[7:0]] <= pix_in;
            bf_wp <= bf_wp + 9'd1;
        end
        bf_ram   <= bf_mem[bf_rp_n[7:0]];
        bf_hit_r <= bf_hit;
        bf_dat_r <= pix_in;
        bf_dv    <= (bf_wp != bf_rp_n);
        bf_rp    <= bf_rp_n;
    end
end

wire [23:0] bf_q = bf_hit_r ? bf_dat_r : bf_ram;

// ★ 2026-10-30：连续流。
//   bmp_view 说 cont1=1（整幅 1:1）时，BMP 文件里"本行末尾"正好就是"下一行行首"，
//   中间一个字节都不用跳。旧流程每行都 `st <= L_WROW` → 重新算 sec/skip_left →
//   发新 CMD17，于是：
//     ① 上一行末那个扇区被**整块重读一次**（实测 60 行里 59 次步长 0 的重复读）；
//     ② 每行多付一次卡的 CMD17 访问延迟（真卡 ~0.8ms ⇒ 480 行 ≈ 0.4s）；
//     ③ 行末预取的那个扇区数据被整段丢弃（又 ~0.4s）。
//   改成 cont1 时行末只把 pcnt 清零、留在 L_STRM 继续组装，一个 CMD17 都不用发。
wire        view_cont1;
// 连续安全阀：只有"行间间隔为 0"（stride == 3*win_w）才真的连续
wire [15:0] win3 = ({3'd0, win_w} << 1) + {3'd0, win_w};
wire        cont_ok = view_cont1 && (stride == win3);

//------------------------------------------------------------------------------
//  4. bmp_view
//------------------------------------------------------------------------------

reg ab_r;          // 见 §7 的 abort
bmp_view u_view (
    .clk(clk), .rst(rst),
    .start(view_start),
    .src_w(w32[15:0]), .src_h(h32[15:0]),
    .mode(mode_r), .fmt(fmt_r),
    .dst_base(dbase), .dst_stride(dstride),
    .zoom(zoom_r), .ax(ax_r), .ay(ay_r),
    .pix_vld(bf_dv), .pix(bf_q), .pix_rd(view_pix_rd),
    .row_req(row_req), .win_x0(win_x0), .win_w(win_w), .row_go(row_go),
    .cont1(view_cont1),
    .stall(fifo_stall),
    .we(v_we), .wa(v_wa), .wdata(v_wdata),
    .busy(view_busy), .done(view_done)
);

// 行首字节偏移（文件内）= 3*x_le
wire [15:0] lo3 = ({4'd0, win_x0} << 1) + {4'd0, win_x0};

//------------------------------------------------------------------------------
//  5. 写 FIFO（53bit × 512，{addr,data} 同包）
//------------------------------------------------------------------------------
wire        fifo_full, fifo_empty;
wire [52:0] fifo_dout;
wire [AFIFO_AW:0] fifo_level;

assign fifo_stall = (fifo_level >= ((1 << AFIFO_AW) - 12));
assign wr_vld  = ~fifo_empty;
assign wr_addr = fifo_dout[52:32];
assign wr_data = fifo_dout[31:0];

img_afifo #(
    .DW (53),
    .AW (AFIFO_AW)
) u_wfifo (
    .wclk  (clk),
    .wrst  (rst),
    .we    (v_we),
    .din   ({v_wa, v_wdata}),
    .wfull (fifo_full),
    .wlevel(fifo_level),
    .rclk  (mclk),
    .rrst  (rst),
    .re    (wr_ack),
    .dout  (fifo_dout),
    .rempty(fifo_empty),
    .rcount(wr_used)
);

//------------------------------------------------------------------------------
//  6. 空/沉降同步 + 排空超时
//------------------------------------------------------------------------------
reg fe_s1, fe_s2, ws_s1, ws_s2;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        fe_s1 <= 1'b0; fe_s2 <= 1'b0; ws_s1 <= 1'b0; ws_s2 <= 1'b0;
    end else begin
        fe_s1 <= fifo_empty; fe_s2 <= fe_s1;
        ws_s1 <= wr_settled; ws_s2 <= ws_s1;
    end
end

localparam [25:0] DRAIN_TO = 26'd49_999_999;
reg [25:0] drain_cnt;
always @(posedge clk or posedge rst) begin
    if (rst)                        drain_cnt <= 26'd0;
    else if (st != L_DRN)           drain_cnt <= 26'd0;
    else if (!(fe_s2 && ws_s2))     drain_cnt <= drain_cnt + 26'd1;
end

//------------------------------------------------------------------------------
//  4'. 扇区请求（**电平保持到拿到数据**，见 [S17]）
//------------------------------------------------------------------------------
//  ★ 2026-10-30：这里原来是"恰好一拍"的脉冲。仲裁器的 I_img_req 只在 A_IDLE
//    采样，而它每收完一个扇区要经 A_GAP 一拍才回 A_IDLE ⇒ 脉冲落进 A_GAP 就
//    **永久丢失**，而 rd_on 已经被置 1 ⇒ 装载器死等。
//    上板表现正是："第一张能出（开机时仲裁器在 A_IDLE），切图切到一半卡住"。
//    改成电平：只要"还想要这一扇区"就一直拉高，直到见到第一个字节。
//    授权一旦给出就锁存到 end，中途撤请求不会把授权收回去（仲裁器保证）。
wire img_issue  = (st == L_HDR) && !rd_on;
wire strm_issue = (st == L_STRM) && !rd_on && !view_done && (bf_cnt < 9'd80) && !ab_r;
wire issue      = img_issue || strm_issue;

// ★ 2026-10-30：请求必须**保持到拿到数据**，不能只打一拍。
//   【上板现象】只放图（SW2 关）一切正常；**一边放音乐**就 HD 不加载、卡在 565。
//   【原因】仲裁器的 I_img_req 只在 A_IDLE 采样，而音频在跑时它会长时间停在
//     A_AUD / A_GAP —— 单拍脉冲正好落进去就**永久丢失**，而 rd_on 已经被置 1，
//     装载器就永远等不到数据，最后超时 ⇒ 客户停在 565 图。
//     （音频静默时仲裁器几乎总在 A_IDLE，所以"关掉音乐就正常"，很难怀疑到它。）
//   【做法】`issue` 仍是单拍（rd_on 的语义、字节门控都不动），但输出 `sd_sec_read`
//     由 req_hold 保持：见到第一个字节或 end 才撤。
//     授权一旦给出就锁存到 end，所以晚撤不会多读一个扇区（仲裁器保证）。
reg req_hold;
always @(posedge clk or posedge rst) begin
    if (rst)                                        req_hold <= 1'b0;
    else if (sd_sec_read_data_valid || sd_sec_read_end) req_hold <= 1'b0;
    else if (issue)                                 req_hold <= 1'b1;
    else if (st == L_DRN || st == L_DONE || st == L_ERR) req_hold <= 1'b0;  // 收工必撤
end
always @(posedge clk or posedge rst) begin
    if (rst) sd_sec_read <= 1'b0;
    else     sd_sec_read <= req_hold;
end

always @(posedge clk) begin
    if (issue) sd_sec_read_addr <= img_issue ? {9'd0, foff[31:9]} : sec;
end

//------------------------------------------------------------------------------
//  7. 主状态机
//------------------------------------------------------------------------------
// 只在这里驱动（FSM 里不能再写，否则 TD 报多驱动）
reg view_done_s;
// 字节源：平时 = 卡上的字节；放弃后 = 每拍一个 0（bf 有空位才给，否则会挤掉像素）
wire       byte_v = ab_r ? (!rd_on && (bf_cnt < 9'd200)) : (sd_sec_read_data_valid && rd_on);
wire [7:0] byte_d = ab_r ? 8'd0 : sd_sec_read_data;
// ★ 2026-11-04 abort：粘滞到下一次 start —— **并且装载一结束立刻自清**。
//   为什么要多这一条：`ab_r` 原来只由 `start` 清，若 abort 在本次装载收尾之后
//   还留着一拍（`quad_page_ctrl` 的 O_ld_abort 在离开文件页时是**电平**，
//   会一直保持到状态机走开），`ab_r` 就会带着 1 进入下一次装载 ⇒
//   下一次直接 L_SEED 失败 / 整幅写成黑 + 报 err。
//   现在：`busy` 落下那一拍之后（L_DONE/L_IDLE）立即清 0，abort 不可能跨次生效。
always @(posedge clk or posedge rst) begin
    if (rst)        ab_r <= 1'b0;
    else if (start) ab_r <= 1'b0;
    else if (!busy) ab_r <= 1'b0;      // ★ 装载结束（含被放弃的那次）⇒ 立刻撤销
    else if (abort) ab_r <= 1'b1;      // 只在 busy 期间才允许置起
end
// row_go 是单拍脉冲，而 loader 可能还在流上一行的尾巴 ⇒ 用粘滞位兜住
always @(posedge clk or posedge rst) begin
    if (rst)                     row_go_s <= 1'b0;
    else if (row_go)             row_go_s <= 1'b1;
    else if ((st == L_WROW && row_go_s) || (st == L_IDLE)) row_go_s <= 1'b0;
end
always @(posedge clk or posedge rst) begin
    if (rst)           view_done_s <= 1'b0;
    else if (start)    view_done_s <= 1'b0;
    else if (view_done) view_done_s <= 1'b1;
end

// 像素组装：每 3 字节一个 {R,G,B}
// ★ 像素必须在**凑齐第三字节的那一拍**就锁存：`pix_wr` 是寄存器，等到下一拍
//   写进 FIFO 时 `sd_sec_read_data` 已经是下一个像素的 B 字节了（实测 R 通道错位）。
reg  [23:0] pix_hold;
assign pix_in = pix_hold;


always @(posedge clk or posedge rst) begin
    if (rst) begin
        st <= L_IDLE; busy <= 1'b0; done <= 1'b0; err <= 1'b0; err_abort <= 1'b0;
        foff <= 32'd0; flen <= 32'd0; dbase <= 21'd0;
        mode_r <= 2'd0; fmt_r <= 1'b0; dstride <= 11'd0;
        zoom_r <= 8'h10; ax_r <= 10'd320; ay_r <= 10'd240;
        hcnt <= 6'd0; hb0 <= 8'd0; hb1 <= 8'd0;
        flen32 <= 32'd0; poff32 <= 32'd0; w32 <= 32'd0; h32 <= 32'd0;
        comp32 <= 32'd0; bits16 <= 16'd0; stride <= 16'd0;         sec <= 32'd0; rd_on <= 1'b0;
        bidx <= 2'd0; pb <= 8'd0; pg <= 8'd0;
        skip_left <= 10'd0; seek_acc <= 32'd0; seek_k <= 12'd0;
        pcnt <= 13'd0;
        pix_wr <= 1'b0; view_start <= 1'b0;
    end else begin
        done       <= 1'b0;
        err        <= 1'b0;
        err_abort  <= 1'b0;
        pix_wr     <= 1'b0;
        view_start <= 1'b0;

        // ---- 扇区请求 / 收尾 ----
        // ★ [S15] 请求做成**单拍脉冲**。仲裁器头注释写明：授权一旦给出就锁存到
        //   `end`，控制器只在"开始一次读"时锁存地址。若 sd_sec_read 一直保持高，
        //   控制器在 end 之后会立刻用**旧地址**重读同一个扇区——实测表现为
        //   "loader 进 L_STRM 时突发已跑掉几十字节"，像素分组整体旋转。
        //   脉冲请求 + `end` 清 rd_on ⇒ 每次突发恰好一次、地址每次都是新的。
        // ★ [S16] `sec+1` 只在 L_STRM 里加：在 L_SEEK/L_WROW 收到的 end 属于
        //   "上一行没读完的尾巴"，那里既不该前进扇区号，也不该动跳过计数器。
        if (sd_sec_read_end) begin
            rd_on <= 1'b0;
            if (st == L_STRM) sec <= sec + 32'd1;   // 顺序续读
        end else if (issue) begin
            rd_on <= 1'b1;
        end

        case (st)
            //--------------------------------------------------------------
            L_IDLE: begin
                if (start) begin
                    foff    <= file_offset;
                    flen    <= file_length;
                    dbase   <= dst_base;
                    mode_r  <= mode;
                    fmt_r   <= fmt;
                    dstride <= (dst_stride == 11'd0) ? 11'd0 : dst_stride;
                    zoom_r  <= zoom;
                    ax_r    <= ax;
                    ay_r    <= ay;
                    hcnt    <= 6'd0;
                    hb0     <= 8'd0; hb1 <= 8'd0;
                    busy    <= 1'b1;
                    st      <= L_HDR;
                end
            end

            //--------------------------------------------------------------
            //  解析文件头（只在前 34 字节里）
            //--------------------------------------------------------------
            L_HDR: begin
                if (sd_sec_read_data_valid) begin
                    if (hcnt <= 6'd33) begin
                        case (hcnt)
                            6'd0:  hb0 <= sd_sec_read_data;
                            6'd1:  hb1 <= sd_sec_read_data;
                            6'd2:  flen32[7:0]   <= sd_sec_read_data;
                            6'd3:  flen32[15:8]  <= sd_sec_read_data;
                            6'd4:  flen32[23:16] <= sd_sec_read_data;
                            6'd5:  flen32[31:24] <= sd_sec_read_data;
                            6'd10: poff32[7:0]   <= sd_sec_read_data;
                            6'd11: poff32[15:8]  <= sd_sec_read_data;
                            6'd12: poff32[23:16] <= sd_sec_read_data;
                            6'd13: poff32[31:24] <= sd_sec_read_data;
                            6'd18: w32[7:0]   <= sd_sec_read_data;
                            6'd19: w32[15:8]  <= sd_sec_read_data;
                            6'd20: w32[23:16] <= sd_sec_read_data;
                            6'd21: w32[31:24] <= sd_sec_read_data;
                            6'd22: h32[7:0]   <= sd_sec_read_data;
                            6'd23: h32[15:8]  <= sd_sec_read_data;
                            6'd24: h32[23:16] <= sd_sec_read_data;
                            6'd25: h32[31:24] <= sd_sec_read_data;
                            6'd28: bits16[7:0]  <= sd_sec_read_data;
                            6'd29: bits16[15:8] <= sd_sec_read_data;
                            6'd30: comp32[7:0]   <= sd_sec_read_data;
                            6'd31: comp32[15:8]  <= sd_sec_read_data;
                            6'd32: comp32[23:16] <= sd_sec_read_data;
                            6'd33: comp32[31:24] <= sd_sec_read_data;
                            default: ;
                        endcase
                    end
                    if (hcnt != 6'd63) hcnt <= hcnt + 6'd1;
                end
                if (sd_sec_read_end) st <= L_SEED;
            end

            //--------------------------------------------------------------
            //  头校验 → 派生 stride → 启动渲染器
            //--------------------------------------------------------------
            L_SEED: begin
                if (!hdr_match || ab_r) begin
                    st <= L_ERR;
                end else begin
                    stride    <= ((w32[15:0] << 1) + w32[15:0] + 16'd3) & 16'hFFFC;
                    view_start <= 1'b1;
                    st <= L_WROW;
                end
            end

            //--------------------------------------------------------------
            //  等 bmp_view 要第一行（之后的请求由 L_STRM 自己处理）
            //--------------------------------------------------------------
            L_WROW: begin
                if (row_go_s) begin
                    seek_acc <= poff32 + lo3;          // pixel_offset + x_le*3
                    seek_k   <= h32[11:0] - 12'd1 - row_req;   // 要跳过的行数
                    pcnt      <= 13'd0;
                    bidx      <= 2'd0;
                    st <= L_SEEK;
                end else if (view_done_s) st <= L_DRN;
            end

            //--------------------------------------------------------------
            //  seek：acc += stride × k（迭代加，无乘法）
            //--------------------------------------------------------------
            L_SEEK: begin
                if (seek_k != 12'd0) begin
                    seek_acc <= seek_acc + {5'd0, stride};
                    seek_k   <= seek_k - 12'd1;
                end else if (!rd_on) begin
                    // 等上一行残留的扇区读完（不中途 abort，否则仲裁器会挂住）
                    // ★ 跳过计数器在这里装载：此刻才是"本行首扇区"真正开始。
                    sec       <= (foff + seek_acc) >> 9;
                    skip_left <= (foff + seek_acc) & 32'h1FF;
                    st        <= L_STRM;
                end
            end

            //--------------------------------------------------------------
            //  流式：喂像素 / 换行 / 收工
            //--------------------------------------------------------------
            L_STRM: begin
                if (view_done_s) st <= L_DRN;
                else if (byte_v) begin
                    if (skip_left != 10'd0) begin
                        // 行首那截（3*x_le 的低位余数）整字节丢掉
                        skip_left <= skip_left - 10'd1;
                    end else begin
                        // 窗口内：按 B / G / R 三字节组装
                        if (bidx == 2'd0)      pb <= byte_d;
                        else if (bidx == 2'd1) pg <= byte_d;
                        else begin
                            pix_hold <= {byte_d, pg, pb};
                            pix_wr   <= 1'b1;
                            pcnt     <= pcnt + 13'd1;
                            // ★ 最后一个像素凑齐就交还控制权：不再等"下一个字节"
                            //   来触发换行（旧写法要等字节，窗口收在扇区末尾时会
                            //   白读一整个扇区，且换行时机依赖从机节奏）。
                            if ((pcnt + 13'd1) >= win_w) begin
                                if (cont_ok) begin
                                    // 连续流：行末字节位置 == 下一行行首 ⇒ 直接接着组装
                                    pcnt <= 13'd0;
                                    bidx <= 2'd0;
                                end else begin
                                    st <= L_WROW;
                                end
                            end
                        end
                        bidx <= (bidx == 2'd2) ? 2'd0 : (bidx + 2'd1);
                    end
                end
            end

            //--------------------------------------------------------------
            L_DRN: begin
                if (drain_cnt == DRAIN_TO) st <= L_ERR;   // ★ [AREA] 从 0 逐一递增、每拍都判 ⇒ == 与 >= 等价
                // ★ 2026-10-30：**必须等最后一个扇区的 end（!rd_on）**。
                //   行尾为了流控会提前要下一扇区（bf_cnt<80），那个突发可能在
                //   本张图"写完了"的时候还在飞；如果这时就 done，客户马上下一次
                //   装载（切图），残余字节会灌进下一次的**头解析**（hb=ffff、
                //   w32=FFFFFFFF ⇒ hdr_match 失败 ⇒ err）。上板表现：
                //   "第一张正常，切图切一半卡住"。
                else if (fe_s2 && ws_s2 && (st == L_DRN) && (bf_cnt == 9'd0) && !rd_on) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    err  <= ab_r;          // 放弃的那次：done + err 同拍
                    err_abort <= ab_r;     // ★ 同拍打标：这次 err 是"放弃"而非"失败"
                    st   <= L_DONE;
                end
            end

            L_DONE: st <= L_IDLE;

            L_ERR: begin
                busy <= 1'b0;
                err  <= 1'b1;
                err_abort <= ab_r;      // ★ 头解析阶段被放弃（L_SEED）也算"放弃"
                st   <= L_IDLE;
            end

            default: st <= L_IDLE;
        endcase
    end
end

`ifdef SIM
reg [3:0] st_q;
always @(posedge clk) begin
    st_q <= st;
    if (st != st_q) $display("[ST] %0t %0d -> %0d  (k=%0d acc=%0d h=%0d rr=%0d go=%b)", $time, st_q, st, seek_k, seek_acc, h32, row_req, row_go);
end
`endif

assign ready = (st == L_IDLE) && !view_busy;

// 仿真断言：FIFO 满时还在推像素 = 流控协议被破坏
`ifdef SIM
always @(posedge clk) if (bf_we && bf_full)
    $display("[ASSERT] %0t bmp_loader: bf_we && bf_full", $time);
`endif

endmodule
