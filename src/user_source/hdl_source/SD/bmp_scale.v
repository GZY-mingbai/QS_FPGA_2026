//==============================================================================
//  bmp_scale.v —— SD 像素流 → 固定画布（640×480 888 / 320×240 565）
//==============================================================================
//
//  【它解决什么问题】
//  显示链路（ps_gen_image / img_layer_hd / viewer_fx）全部按固定画布写死：
//      888 = 640×480，1 字 1 像素，行宽 640 字
//      565 = 320×240，1 字 2 像素，行宽 320 字 + 行翻转
//  源 BMP 却是任意分辨率。所以**在装载阶段**把任意源图渲染进这两块画布，
//  显示侧一行都不用改。
//
//  【几何】
//      stretch=1：dw=640 dh=480 dx0=dy0=0
//      stretch=0：640*H < 480*W ?  dw=640, dh=H*640/W, dy0=(480-dh)/2   （源更宽）
//                                  dh=480, dw=W*480/H, dx0=(640-dw)/2   （源更高）
//  即"等比 contain，宽或高正好占满，其余补黑"。
//
//  【映射约定】最近邻、**向下取整**，与旧 1:2 采样（取偶列）完全一致：
//      输出列 c  ←  源列 floor((c-dx0)*sw/dw)
//      输出行 k  ←  源行 floor((k-dy0)*sh/dh)          （k 为内容区行号）
//  所以缩小的时候最后几行/列可能用不到（与旧行为一致），放大时同源像素重复。
//
//  ★ 2026-10-27 例外：**精确 2:1 横向**（565，sw == dw）改为 (偶+奇)>>1。
//    原因：565 冲刷本来只取画布偶列（源 0,2,4…），奇数源列被丢掉；
//    从 320×240 缩略图切到 HD 时，落在奇列的边缘会偏 1px（"微量左右位移"）。
//    平均后每个 565 像素 = 源 2c/2c+1 的均值，2× 放大与 HD 的 2×2 块对齐。
//    其它比例（1920→640、320→640 等）仍走最近邻，行为不变。
//
//  【★ 全程没有除法器、没有乘法器 ★】
//  尺寸只算一次：dw 或 dh = **分子连续减分母**（商 ≤ 640，最多 640 拍）。
//  映射用"余数累加"，天然是精确的 floor：
//      横向（源列驱动）：hn += dw;  while (hn >= sw) { hn -= sw; run++ }
//                        ⇒ 这个源像素覆盖 run 个输出列
//      纵向（输出行驱动）：vn += sh; while (vn >= dh) { vn -= dh; sy_t++ }
//                        ⇒ 这一输出行要用第 sy_t 个源行（同一行可重复冲刷）
//
//  【写地址】严格线性 0..307199（888）/ 行翻转 0..38399（565，160 字/行）
//  的地址流逐字一致 ⇒ SDRAM 调度、写仲裁、预清逻辑都不用改。
//
//  【冲刷流水（2026-10-27 重写，修掉 3 个相位 bug）】
//    lbuf 是"同步读"存储器（地址寄存一拍、数据下一拍出），所以**写必须比取址
//    晚一拍**，而且命中标志 inb 要和 lb_q 同一拍对齐：
//        cyc ox=k   : lb_ra <= f(xr(k));  inb <= inb(k)
//        cyc ox=k+1 : lb_q = lbuf[xr(k)]; inb = inb(k)  ⇒ 正是第 k 列的数据
//    旧代码用"上上拍的 xr"做门控、却在"上一拍"写地址，整幅图横向偏 1 列。
//
//==============================================================================
module bmp_scale #(
    parameter integer CW = 640,
    parameter integer CH = 480
)(
    input  wire        clk,
    input  wire        rst,

    input  wire        start,
    input  wire [15:0] src_w,
    input  wire [15:0] src_h,
    input  wire        stretch,
    input  wire        fmt,             // 1 = 888(640x480)  0 = 565(320x240, 行翻转)

    input  wire        pix_vld,
    input  wire [23:0] pix,
    output wire        pix_rd,

    // ★ 2026-10-27：下游满 → 冻结输出级。
    //   【为什么必须有】原来输出侧完全不看 FIFO 满不满：`wr_credit` 只保护了
    //   输入侧（bmp_read），而 bmp_scale 冲刷一行的速度远快于 SDRAM 写口排空，
    //   888 一帧 307200 字 ⇒ FIFO 溢出、写被静默丢弃，装载器却照常 done、
    //   err_code=0。实测：307200 字里丢了 185301 字（正好等于溢出次数）。
    //   565 只有 38400 字，来得及排空，所以"缩略图正常、高清不正常"。
    input  wire        stall,           // 1 = 下游满，冻结冲刷

    output reg         we,
    output reg  [20:0] woff,
    output reg  [31:0] wdata,

    output reg         busy,
    output reg         done
);

localparam [3:0] S_IDLE = 4'd0, S_GEO  = 4'd1, S_ROW  = 4'd2, S_VADV = 4'd3,
                 S_VCHK = 4'd4, S_VDONE= 4'd5, S_DISC = 4'd6, S_FGET = 4'd7,
                 S_HADV = 4'd8, S_FWR  = 4'd9, S_FL   = 4'd10, S_NEXT = 4'd11,
                 S_DONE = 4'd12;

reg [3:0] st;

reg [10:0] dw, dh, dx0, dy0;
reg [15:0] sw, sh;
reg        stretch_r, fmt_r;
reg        vbord;           // 本行是纵向黑边（整行写 0）

// ★ 强制落块 RAM（15,360 bit）：组合读会被打成分布式 RAM（≈950 LUT），
//   这里是"地址寄存 + 输出寄存"的标准简单双口写法，属性只是兜底。
(* ram_style = "block" *) (* ramstyle = "block" *) (* syn_ramstyle = "block" *)
reg [23:0] lbuf [0:639];    // 画布行缓冲（1 个 32Kb 块即可）
reg [23:0] lb_q;            // BRAM 输出（比地址晚一拍）
reg [9:0]  lb_ra;
reg        d1_inb;          // 级 1：该列是否在内容区
reg [11:0] d1_col;          // 级 1：列号
reg        d2_inb;          // 级 2：与 lb_q 同拍对齐的命中标志
reg [11:0] d2_col;          // 级 2：与 lb_q 同拍对齐的列号
reg [15:0] wd_lo;           // 565：偶数列算出的低半字

reg [11:0] oy;              // 输出行（两种格式都数 0..479）
reg [10:0] sy;              // 已消费的源行数（下一待消费行号）
reg [10:0] sy_t;            // 本输出行要用的源行
reg [15:0] sx;              // 源列 + 已消费像素计数
reg        vfirst;          // 内容区第一行：纵向累加器不前进（sy=0）
reg [15:0] hn, vn;          // 横向/纵向**余数**（跨像素/跨行保留）
reg [15:0] hac, vac;        // 本步的分子（hn+dw / vn+sh），循环里递减
reg [10:0] cprev;           // 当前源像素对应的输出列起点
reg [10:0] run, run_i;
reg [23:0] pix_lat;
reg [23:0] pix_even;         // ★ 2:1 横向平均：暂存偶数画布列
reg        filled;          // 行缓冲里已有可复用的整行
reg [10:0] ox;              // 冲刷：已取址到第几列

// 一次性尺寸计算（连续减）
reg [27:0] gnum;
reg [15:0] gden;
reg [10:0] gq;
reg [2:0]  gs;
wire [10:0] gq_ok = (gq == 11'd0) ? 11'd1 : gq;   // 极端长条图别算出 0

// 源像素消费：取像素 与 **丢整行** 都要推进源流（旧代码漏了后者 ⇒ 行列失步）
assign pix_rd = ((st == S_FGET) || (st == S_DISC)) && pix_vld;

// ★★ 2026-10-27：640×480 → 320×240 的 565 缩略图改成**横向 2:1 平均** ★★
//   【上板现象】第一张图从 320×240 缩略图切到 HD 时，画面有微量左右位移。
//   【根因】原来 565 走 floor 映射，只取偶数源列（src0,src2,src4…），
//     奇数源列被丢掉。2× 放大后，偶数特征对得上 HD，但奇数特征会偏 1px；
//     真实图片的边缘落在任意列，肉眼就是"整幅图轻轻横移"。
//   【改法】仅对**精确 2:1 横向**（sw == dw，本工程 640→640 画布）在
//     S_FWR 写行缓冲时做 (偶列+奇列)>>1，并写回偶列；565 冲刷本来
//     只取偶列 ⇒ 每个 565 像素 = 源 2c/2c+1 的平均。
//     其它比例仍走原来的最近邻（不改行为）。
//     纵向仍取偶数行：用户报的是左右；纵向平均需要整行缓冲，暂不做。
//   【为什么不影响 1:1】源 320 → 画布 640 ⇒ sw=320 != dw=640 ⇒ avg2=0。
//   ★ 2026-10-28：再加 `dx0[0]==0`。这个平均只写**偶内容列**，而 565 打包器
//     取的是"偶画布列 ⇒ 内容列 = 偶画布列 − dx0"。dx0 为奇数时被打包的全是
//     **从未写过的奇内容列**（读到上一行残值）⇒ 又是一次花屏。
//     dx0 为奇只会出现在 W×480 这类源（dw = W，dx0 = (640-W)/2）；
//     这种尺寸退回最近的最近邻即可（那 0.5px 偏移比花屏重要得多）。
wire avg2 = (!fmt_r) && (sw == dw) && (dx0[0] == 1'b0);

// ★★★ 2026-10-28 根因修复：平均必须**逐通道**做，不能整字相加 ★★★
//   【上板现象】640×480 源在 320×240（565）状态下整幅花屏，但轮廓认得出；
//     OSD 文字正常、HD（888）正常 ⇒ 只有"经过这级平均的 565 纹理"坏。
//   【病根】旧写法是
//       pix_avg_sum = {1'b0,pix_even} + {1'b0,pix_lat};   // 24 位整字加
//       lbuf[...] <= pix_avg_sum[24:1];                    // 再整体右移
//     R/G/B 是挤在同一个 24 位字里的，低位通道的和 >=256 就会**进位到高位通道**，
//     右移时高位通道的奇偶位又落进低位通道的 MSB：
//         result = (R_s<<16 + G_s<<8 + B_s) >> 1
//                = R_s<<15 + G_s<<7 + B_s>>1
//       ⇒ R = R_s>>1            （正确）
//         G = {R_s[0], G_s[8:1]}（MSB 被 R 的奇偶覆盖）
//         B = {G_s[0], B_s[7:1]}（MSB 被 G 的奇偶覆盖）
//     实测 avg(24'h010000, 24'h000000) 旧写法得 24'h008000 —— 一对纯红凭空
//     冒出 G=128。上板就是**满屏 ±128 的蓝/绿彩色麻点**（红色/亮度结构保留，
//     所以"能看出图像大概是什么"）。
//   【为什么仿真当时是绿的】tb 的 avg2pix() 抄了同一套整字加写法，把 bug
//     写成了期望。已同时改成逐通道，并加了 E3/E4 两条只看通道的断言。
//   【代价】3 个 8 位加法器，比原来 1 个 25 位加法器还省。
wire [8:0] avg_r = {1'b0, pix_even[23:16]} + {1'b0, pix_lat[23:16]};
wire [8:0] avg_g = {1'b0, pix_even[15:8]}  + {1'b0, pix_lat[15:8]};
wire [8:0] avg_b = {1'b0, pix_even[7:0]}   + {1'b0, pix_lat[7:0]};
wire [23:0] pix_avg = {avg_r[8:1], avg_g[8:1], avg_b[8:1]};

//------------------------------------------------------------------------------
//  行内列映射
//------------------------------------------------------------------------------
wire [11:0] oxz  = {1'b0, ox};
wire [11:0] dx0z = {1'b0, dx0};
wire [11:0] dx1z = {1'b0, dx0} + {1'b0, dw};
wire        inb_c = (oxz >= dx0z) && (oxz < dx1z);
wire [11:0] xrz   = oxz - dx0z;
wire [9:0]  xr    = (xrz < 12'd640) ? xrz[9:0] : 10'd0;

// ★ 565 的纵向分母按 **240 行** 算，不是 480 ★
//   565 只冲刷偶数输出行（奇数行直接跳过），纵向累加器也随之只走 240 次，
//   若仍用 dh=480 就会把 sh 行压到 240 行的一半里去 ⇒ 缩略图只显示源图上半截
//   （踩过：640x480 源的 565 缩略图 sy_t = k 而不是 2k）。
wire [10:0] dhv = fmt_r ? dh : (((dh >> 1) == 11'd0) ? 11'd1 : (dh >> 1));

// 纵向黑边判据（S_ROW 内组合使用）
wire vbord_c = (oy < {1'b0, dy0}) || (oy >= ({1'b0, dy0} + {1'b0, dh}));

// 888：640 字/行；565：**160 字/行**（1 字 = 2 个 RGB565 像素，与 ps_gen_image
// 的 SRC_W「字数」口径一致；240 行 × 160 = 38400 字）。行翻转只对 565 做。
wire [20:0] row_base = fmt_r ? ({10'd0, oy} * 21'd640)
                             : ({11'd0, (12'd239 - {2'd0, oy[11:1]})} * 21'd160);

//==============================================================================
//  ★★★ 2026-10-28 根因修复：`stall` 期间**整条冲刷延迟线必须一起冻结** ★★★
//==============================================================================
//  【上板现象】640×480 源在 HD（888）下**右边有一整列 1px 黑线**，565 没有；
//    烧回 d1b08f8（还没有缩放）就没有 ⇒ 是缩放这条线引入的。
//
//  【机制】S_FL 的冲刷是"取址 → BRAM 输出 → 写"两级流水：
//        cyc k   : lb_ra <= xr(ox=k)、d1_col <= k、ox <= k+1
//        cyc k+1 : lb_q = lbuf[xr(k)]、d2_col <= d1_col
//        cyc k+2 : 写：woff = row_base + d2_col（此时 d2_col == k，与 lb_q 配对）
//    原来 `ox`/`d1_*` 在 `if (!stall)` 里（会被冻结），而 `lb_q`/`d2_*` 在
//    always 块顶部**无条件**推进。于是每来一次 stall：
//        ox 停在 K，d1_col 停在 K-1，而 d2_col 继续吃 d1_col ⇒ 从 K-2 追到 K-1
//      ⇒ 流水线塌一格 ⇒ **第 K-2 列永远不写、第 K-1 列写两遍**。
//    实测（tb_scale_shift 加 stall 激励后）：写地址序列 #7=7、#8=9、#9=9，
//    每行 46 个列是"从未写入"的洞。
//
//  【为什么只有 888 看得见】冲刷循环对两种格式都是 ox=0..641（642 拍），
//    但 888 每行写 **640 字**、565 只写 **160 字**（1 字=2 像素）⇒ 888 的写
//    FIFO 水位是 565 的 4 倍，行末正好撞上 `scale_stall` ⇒ 第 639 列被跳过
//    ⇒ 一条贯穿全高的黑线。565 数据量只有 1/4，撞不到。
//
//  【修法】把 `lb_q`/`d2_inb`/`d2_col` 也放进"不冻结才推进"的条件里。
//------------------------------------------------------------------------------
wire fl_hold = (st == S_FL) && stall;      // 冲刷被下游冻结

always @(posedge clk or posedge rst) begin
    if (rst) begin
        st <= S_IDLE; busy <= 1'b0; done <= 1'b0;
        we <= 1'b0; woff <= 21'd0; wdata <= 32'd0;
        dw <= 11'd0; dh <= 11'd0; dx0 <= 11'd0; dy0 <= 11'd0;
        sw <= 16'd0; sh <= 16'd0; stretch_r <= 1'b0; fmt_r <= 1'b0;
        vbord <= 1'b0;
        oy <= 12'd0; sy <= 11'd0; sy_t <= 11'd0; sx <= 16'd0;
        hn <= 16'd0; vn <= 16'd0; hac <= 16'd0; vac <= 16'd0; cprev <= 11'd0;
        vfirst <= 1'b1;
        run <= 11'd0; run_i <= 11'd0; pix_lat <= 24'd0; pix_even <= 24'd0; filled <= 1'b0;
        ox <= 11'd0; d1_inb <= 1'b0; d1_col <= 12'd0;
        d2_inb <= 1'b0; d2_col <= 12'd0; wd_lo <= 16'd0;
        gs <= 3'd0; gnum <= 28'd0; gden <= 16'd0; gq <= 11'd0;
        lb_ra <= 10'd0; lb_q <= 24'd0;
    end else begin
        we <= 1'b0;
        // ★ stall 冻结期间，延迟线必须和 ox / d1_* 一起停住，否则会塌一格
        if (!fl_hold) begin
            lb_q   <= lbuf[lb_ra];
            d2_inb <= d1_inb;
            d2_col <= d1_col;
        end

        case (st)
            //------------------------------------------------------------------
            S_IDLE: begin
                done <= 1'b0;
                if (start) begin
                    sw <= src_w; sh <= src_h;
                    stretch_r <= stretch; fmt_r <= fmt;
                    busy <= 1'b1;
                    gs   <= 3'd0;
                    st   <= S_GEO;
                end
            end

            //------------------------------------------------------------------
            //  尺寸：一次连续减（商 ≤ 640）
            //------------------------------------------------------------------
            S_GEO: begin
                case (gs)
                    3'd0: begin
                        if (stretch_r) begin
                            dw <= 11'd640; dh <= 11'd480;
                            dx0 <= 11'd0;  dy0 <= 11'd0;
                            oy <= 12'd0; sy <= 11'd0; sy_t <= 11'd0;
                            vn <= 16'd0; filled <= 1'b0; vfirst <= 1'b1;
                            st <= S_ROW;
                        // 判据：640*H < 480*W ⇒ 源**更宽**（16:9 那种）⇒ 以宽为准
                        end else if (({12'd0, sh} << 9) + ({12'd0, sh} << 7) <
                                     ({12'd0, sw} << 8) + ({12'd0, sw} << 7) +
                                     ({12'd0, sw} << 6) + ({12'd0, sw} << 5)) begin
                            gnum <= ({12'd0, sh} << 9) + ({12'd0, sh} << 7);   // H*640
                            gden <= sw;
                            gq   <= 11'd0;
                            gs   <= 3'd1;
                        end else begin
                            gnum <= ({12'd0, sw} << 8) + ({12'd0, sw} << 7) +
                                    ({12'd0, sw} << 6) + ({12'd0, sw} << 5);   // W*480
                            gden <= sh;
                            gq   <= 11'd0;
                            gs   <= 3'd2;
                        end
                    end
                    3'd1: begin
                        if (gnum >= {12'd0, gden}) begin
                            gnum <= gnum - {12'd0, gden};
                            gq   <= gq + 11'd1;
                        end else begin
                            dh  <= gq_ok;
                            dw  <= 11'd640;
                            dx0 <= 11'd0;
                            dy0 <= (11'd480 - gq_ok) >> 1;
                            oy <= 12'd0; sy <= 11'd0; sy_t <= 11'd0;
                            vn <= 16'd0; filled <= 1'b0; vfirst <= 1'b1;
                            st <= S_ROW;
                        end
                    end
                    default: begin
                        if (gnum >= {12'd0, gden}) begin
                            gnum <= gnum - {12'd0, gden};
                            gq   <= gq + 11'd1;
                        end else begin
                            dw  <= gq_ok;
                            dh  <= 11'd480;
                            dy0 <= 11'd0;
                            dx0 <= (11'd640 - gq_ok) >> 1;
                            oy <= 12'd0; sy <= 11'd0; sy_t <= 11'd0;
                            vn <= 16'd0; filled <= 1'b0; vfirst <= 1'b1;
                            st <= S_ROW;
                        end
                    end
                endcase
            end

            //------------------------------------------------------------------
            S_ROW: begin
                ox    <= 11'd0;
                vbord <= vbord_c;
                if (!fmt_r && oy[0]) begin
                    st <= S_NEXT;                                 // 565 只冲偶数行
                end else if (vbord_c) begin
                    st <= S_FL;                                   // 纵向黑边
                end else begin
                    sx    <= 16'd0;
                    hn    <= sw - 16'd1;        // ★ 见 S_VDONE：floor 约定
                    cprev <= 11'd0;
                    st    <= S_VADV;
                end
            end

            //------------------------------------------------------------------
            //  纵向：vn += sh；商 = 本输出行相对上一行的源行增量
            //------------------------------------------------------------------
            S_VADV: begin
                // 内容区第 0 行必须用源第 0 行：先不累加
                if (vfirst) begin
                    vac    <= vn;
                    vfirst <= 1'b0;
                end else
                    vac <= vn + sh;
                st <= S_VCHK;
            end

            // 循环必须用独立累加寄存器：写成 vn <= vn+sh-dh 会每拍拿旧的 vn
            // 重算同一个值，永远退不出循环。
            S_VCHK: begin
                if (vac >= {5'd0, dhv}) begin
                    vac  <= vac - {5'd0, dhv};
                    sy_t <= sy_t + 11'd1;
                end else begin
                    vn <= vac;
                    st <= S_VDONE;
                end
            end

            //------------------------------------------------------------------
            //  ★ 复用判据必须在 sy_t **更新之后** 才成立（这里），
            //    放到 S_ROW 会读到上一行的 sy_t ⇒ 纵向放大时每行都用第 0 行。
            //------------------------------------------------------------------
            S_VDONE: begin
                if (filled && (sy == (sy_t + 11'd1))) begin
                    st <= S_FL;                                   // 缓冲里就是这一行
                end else if (sy < sy_t) begin
                    sx <= 16'd0;
                    st <= S_DISC;                                 // 缩小：丢掉整行
                end else if (sy_t >= sh) begin
                    vbord <= 1'b1;                                // 源行被截断 ⇒ 画黑
                    st    <= S_FL;
                end else begin
                    sx    <= 16'd0;
                    // ★ 横向余数从 sw-1 起（不是 0）⇒ run_k = ceil 差分
                    //   ⇒ 输出列 c 取源列 floor(c*sw/dw)，与旧 1:2 采样（取偶列）
                    //     和测试断言一致。从 0 起会变成"取区间末像素"，整幅偏
                    //     (sw/dw-1) 列（1920→640 就是偏 2 列）。
                    hn    <= sw - 16'd1;
                    cprev <= 11'd0;
                    st    <= S_FGET;
                end
            end

            S_DISC: begin
                if (pix_vld) begin
                    sx <= sx + 16'd1;
                    if ((sx + 16'd1) >= sw) begin
                        sy <= sy + 11'd1;
                        st <= S_VDONE;
                    end
                end
            end

            //------------------------------------------------------------------
            //  取一个源像素
            //------------------------------------------------------------------
            S_FGET: begin
                if (pix_vld) begin
                    pix_lat <= pix;
                    run     <= 11'd0;
                    run_i   <= 11'd0;
                    sx      <= sx + 16'd1;
                    hac     <= hn + {5'd0, dw};
                    st      <= S_HADV;
                end
            end

            //------------------------------------------------------------------
            //  横向：hn += dw；商 = 这个源像素覆盖几列
            //------------------------------------------------------------------
            S_HADV: begin
                if (hac >= {5'd0, sw}) begin
                    hac <= hac - {5'd0, sw};
                    run <= run + 11'd1;
                end else begin
                    hn <= hac;
                    if (run == 11'd0) begin
                        if (sx >= sw) begin
                            sy <= sy_t + 11'd1;
                            st <= S_FL;                            // 缩小：整列被跳过
                        end else st <= S_FGET;
                    end else begin
                        run_i <= 11'd0;
                        st    <= S_FWR;
                    end
                end
            end

            S_FWR: begin
                // ★ 相对列号：行缓冲只存内容区（0..dw-1），黑边由冲刷按 dx0 补。
                //   旧代码写 dx0+cprev、冲刷读 xr ⇒ dx0≠0 时整幅错位 dx0 列。
                if (avg2) begin
                    // 精确 2:1（640 源 → 640 画布 → 565 只取偶列）：
                    //   偶列先存，奇列来时把逐通道均值写回**偶列**；
                    //   565 冲刷取偶列 ⇒ 每个 565 像素是源 2c/2c+1 的平均。
                    // ★ 2026-10-28：必须用 pix_avg（逐通道），不能用整字右移。
                    if (!cprev[0])
                        pix_even <= pix_lat;
                    else
                        lbuf[cprev - 11'd1] <= pix_avg;
                end else
                    lbuf[cprev + run_i] <= pix_lat;
                run_i <= run_i + 11'd1;
                if ((run_i + 11'd1) >= run) begin
                    cprev <= cprev + run;
                    if (sx >= sw) begin
                        sy <= sy_t + 11'd1;
                        st <= S_FL;
                    end else st <= S_FGET;
                end
            end

            //------------------------------------------------------------------
            //  冲刷一行
            //  【流水】ox = 正在取址的列号；地址 → BRAM 输出共 2 拍延迟，
            //  所以列号与命中标志也走两级延迟线，写用 d2_col/d2_inb/lb_q
            //  三者同拍 ⇒ 第 c 列的数据一定写到第 c 列（旧代码差 2 列）。
            //  写窗口：ox = 2..641 对应列 0..639；ox == 641 收尾。
            //------------------------------------------------------------------
            S_FL: begin
              if (!stall) begin          // ★ 下游满 → 整段冻结（含 ox 推进与收尾）
                if (ox <= 11'd640) begin
                    lb_ra  <= inb_c ? xr : 10'd0;
                    d1_inb <= inb_c;
                    d1_col <= oxz;
                    ox     <= ox + 11'd1;
                end
                if (ox >= 11'd2) begin
                    if (fmt_r) begin                       // 888：一拍一个字
                        we    <= 1'b1;
                        woff  <= row_base + {9'd0, d2_col};
                        wdata <= (d2_inb && !vbord) ? {8'hFF, lb_q} : 32'd0;
                    end else if (d2_col[1:0] == 2'b00) begin
                        // 565 像素 j = 画布列 2j ⇒ 只有**偶画布列**参与（2:1 抽列）
                        // j 为偶数 ⇒ 字的低半
                        wd_lo <= (d2_inb && !vbord) ? rgb565(lb_q) : 16'd0;
                    end else if (d2_col[1:0] == 2'b10) begin
                        // j 为奇数 ⇒ 字的**高半**，连同刚存的低半一起落字
                        // 地址 n = j>>1 = 列>>2
                        we           <= 1'b1;
                        woff         <= row_base + {10'd0, d2_col[11:2]};
                        wdata[31:16] <= (d2_inb && !vbord) ? rgb565(lb_q) : 16'd0;
                        wdata[15:0]  <= wd_lo;
                    end
                end
                if (ox == 11'd641) begin
                    filled <= 1'b1;
                    st     <= S_NEXT;
                end
              end                        // if (!stall)
            end

            //------------------------------------------------------------------
            S_NEXT: begin
                oy <= oy + 12'd1;
                if (oy == 12'd479) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    st   <= S_DONE;
                end else st <= S_ROW;
            end

            // 收工后必须回 IDLE：loader 一张图跑完还要跑下一张
            default: begin
                done <= 1'b1;
                busy <= 1'b0;
                st   <= S_IDLE;
            end
        endcase
    end
end

function [15:0] rgb565;
    input [23:0] c;
    begin
        rgb565 = {c[23:19], c[15:10], c[7:3]};
    end
endfunction

endmodule
