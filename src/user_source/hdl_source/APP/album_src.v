//==============================================================================
//  album_src.v —— 相册的三张图从 FAT 目录拿（不再用 IMGx_OFFSET 常量）
//==============================================================================
//
//  【为什么需要】
//  相册原来靠三个顶层参数 `IMG0/1/2_OFFSET`（绝对字节偏移）。那是"某一次
//  往卡里拷文件"的结果：换卡、重新格式化、甚至只是删掉一张图重拷，偏移就全变了。
//  上板表现是"相册不动 / 花屏 / 卡死"。改成开机扫一次根目录，取前 3 个 .BMP。
//
//  【与文件页共用 lister】
//  lister 只有一份（约 350 LUT）。时分复用的依据：
//    · 文件页只在"菜单开着 && page==2"时用，相册只在**开机**用一次；
//    · 两边都只在对方不用时才发 start（顶层用 owner 锁存 done 的归属）。
//  相册侧只有一条规则：**文件页开着就不开始**（顶层还会用 busy 挡住）。
//
//  【为什么只要 3 张】
//  viewer_ctrl 的过渡动画需要 3 张缩略图常驻 SDRAM（prev/cur/next 的 A/B 层），
//  这是 SDRAM 布局定死的上限（见顶层 SDRAM 规划）。所以相册就是"目录里的
//  前 3 个 BMP 循环"。**已知限制**：第 4 张以后要进文件页去看。
//  （要做"整个目录滑动窗口"，需要给 viewer_ctrl 加窗口重填，见 PLAN 的后续项。）
//
//  【没有 BMP 怎么办】
//  off 保持 0、ready=0。viewer_ctrl 会去读 0 号扇区（MBR，不是 BMP）⇒
//  bmp_loader 报 err ⇒ 停在 565 背景色，**不会卡死**（err 路径本来就有）。
//==============================================================================

module album_src (
    input  wire        clk,
    input  wire        rst,

    //------------------------------------------------------------------
    //  触发与互斥
    //------------------------------------------------------------------
    input  wire [15:0] I_base,          // 窗口起点：目录里第几张图（0/3/6/…）
    input  wire        I_need,          // 1 = 相册需要偏移（未就绪时置 1 即可，不用脉冲）
    input  wire        I_block,         // 1 = 文件页正在用 lister（此时不许发 start）
    // ★ 2026-11-02【信息条文件名】：本窗口要**重扫一次**（阻塞式刷新）。
    //   唯一来源是"关闭文件页"——文件页扫描会把 name_mem（4 槽 × 8.3 名）整槽
    //   覆盖成"第几个目录项"，而本模块原来只在 base 变化时重扫。
    //   走和"base 变了"同一条作废通道：清 tried/O_ready ⇒ `O_ready` 落下期间
    //   `alb_ok=0`，上层的一切发单（高清/缩略图/缩放恢复）都**挂住等这次重扫**；
    //   扫完 O_ready 回 1 才放行。同 base 重扫出来的偏移逐位不变，所以这段等待
    //   只刷新名字，不改变任何装载数据。
    input  wire        I_rescan,

    //------------------------------------------------------------------
    //  lister 请求
    //------------------------------------------------------------------
    output reg         O_lst_start,     // 单拍
    output wire [15:0] O_lst_offset,    // 恒 0：相册总是从目录头开始
    output wire [1:0]  O_rd_slot,       // 正在读第几槽（接 lister 的 I_ld_slot）
    input  wire        I_lst_busy,
    input  wire        I_lst_done,

    //------------------------------------------------------------------
    //  lister 结果（4 项）
    //------------------------------------------------------------------
    input  wire [3:0]  I_ld_type,       // 0=其他 1=BMP 2=WAV 3=TXT（跟 I_ld_slot）
    input  wire [3:0]  I_slot_valid,    // 4 个槽各自的 valid（按 O_rd_slot 选）
    input  wire [31:0] I_file_off,

    //------------------------------------------------------------------
    //  对外
    //------------------------------------------------------------------
    output reg  [31:0] O_off0,
    output reg  [31:0] O_off1,
    output reg  [31:0] O_off2,
    // ★ 2026-11-01 夜：第 4 个偏移（next2）。滑动窗口要"每切一次只重读一张"，
    //   靠的就是这个"下一张的下一张"提前扫出来。
    output reg  [31:0] O_off3,
    // ★ 2026-11-04：`O_have[1:0]`（窗口里有几张）缩成 1 位 `O_any`。
    //   原来 viewer_ctrl / img_switch 拿它当"还有没有下一张"的判据，那个口径是错的
    //   （窗口内张数 ≠ 后面还有图，而且 4 张会被截成 3）⇒ 换图资格已经彻底不看它。
    //   现在它只表示"本窗口至少有一张图"，仅用于：① 空目录判定（S_BOOT_T）
    //   ② `nc`（当前格无效就重装）的使能。
    output reg         O_any,           // 1 = 本窗口至少取到一张图
    // ★ 2026-11-05 [NIMG]：本窗口**实际取到的图数**（0..4）。
    //   只用于"窗口第 rel 格到底有没有图"这一个判断（img_switch 的 `nn`）。
    //   它**不参与装载**：偏移照旧原样发布，预取路径逐位不变。
    output reg  [2:0]  O_nimg,
    output reg         O_ready,         // 1 = 三张都齐了（或确认不够，不再扫）
    output wire        O_win4,          // 1 = 窗口第 4 项也是图（说明后面还有）
    output wire        O_sel            // 1 = 正在逐槽取偏移（此时元数据口归我）
);

localparam [1:0] A_IDLE = 2'd0;
localparam [1:0] A_RUN  = 2'd1;      // 等 lister done
localparam [1:0] A_COL  = 2'd2;      // 逐槽取 offset
localparam [1:0] A_FIN  = 2'd3;

reg [1:0] st;
reg [1:0] k;                          // 正在取第几槽
reg [1:0] got;                        // 已取到几个 BMP
reg       tried;                      // 本窗口已扫过（base 变了才重扫）
reg [15:0] base_r;                    // 上一次扫描用的窗口起点
// ★ 2026-11-01 D：O_win4 必须**寄存**。
//   原来它是组合输出 `I_slot_valid[3] && I_ld_type==1`，而扫描结束后
//   `lst_ld_slot` 已经切回 OSD 的名字口（slot 0）⇒ win4 永远按第 0 格算
//   ⇒ 末窗（第 4 项是 wav/txt）仍报"后面还有" ⇒ 相册会滑到非图片项上。
reg        win4_r;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        st          <= A_IDLE;
        k           <= 2'd0;
        got         <= 2'd0;
        tried       <= 1'b0;
        base_r      <= 16'd0;
        O_lst_start <= 1'b0;
        O_off0      <= 32'd0;
        O_off1      <= 32'd0;
        O_off2      <= 32'd0;
        O_off3      <= 32'd0;
        O_any       <= 1'b0;
        O_nimg      <= 3'd0;
        O_ready     <= 1'b0;
        win4_r      <= 1'b0;
    end else begin
        O_lst_start <= 1'b0;

        // ★ 窗口起点变了 ⇒ 作废上次结果，重新扫（滑动窗口就是靠这个）

        case (st)
        //----------------------------------------------------------------------
        A_IDLE: begin
            // 需要、没扫过、对方没用 lister ⇒ 发起一次扫描
            if (I_need && !tried && !O_ready && !I_block && !I_lst_busy) begin
                O_lst_start <= 1'b1;
                // ★ 2026-11-04："已扫"在**开扫**时置，扫描中途 base 又变 ⇒ 上面那段把它清 0，
                //   扫完不认（ready 不拉高）、回 IDLE 按新 base 重扫。
                //   原来在扫完时才置 tried/ready ⇒ 快速连切两次时，旧 base 的偏移被当成
                //   新窗口的发布出去（装错图）。
                tried       <= 1'b1;
                st          <= A_RUN;
            end
        end

        //----------------------------------------------------------------------
        A_RUN: begin
            if (I_lst_done) begin
                k   <= 2'd0;
                got <= 2'd0;
                st  <= A_COL;
            end
        end

        //----------------------------------------------------------------------
        //  A_COL：一次一台（I_ld_slot 是组合口，必须一个槽一个槽地取）
        //----------------------------------------------------------------------
        A_COL: begin
            if (I_slot_valid[k] && (I_ld_type == 4'd1)) begin
                case (got)
                    2'd0: O_off0 <= I_file_off;
                    2'd1: O_off1 <= I_file_off;
                    2'd2: O_off2 <= I_file_off;
                    default: O_off3 <= I_file_off;   // 第 4 张（预取用）
                endcase
                got <= got + 2'd1;
            end
            if (k == 2'd3) begin
                // ★ 2026-11-04：只发布"有没有图"，不再发布张数。
                //   原来这里是 `O_have <= (got==2 && slot3是图) ? 3 : got`，2 位封顶 3，
                //   而消费者（img_switch 的 n_ex）又把它当成"还有没有下一张"用 ——
                //   口径错位：窗口里有 1 张、但第 4 格是图时，它会判"没有下一张"，
                //   于是 cur+1 永远不预取、翻页彻底死掉。现在彻底解耦：
                //   "有没有下一张" 由每一格的 tv/dv（装到了 / 试过没有）决定。
                //   got 在 k==3 这一拍统计的正是槽 0..2，再加上槽 3 的特判。
                O_any   <= (got != 2'd0) || (I_slot_valid[3] && (I_ld_type == 4'd1));
                // ★ [NIMG]：图数 = 已取到的 got + 槽 3 是不是图
                O_nimg  <= {1'b0, got} + {2'b0, (I_slot_valid[3] && (I_ld_type == 4'd1))};
                // 第 4 项是不是图：在 A_COL 的最后一拍按 slot 3 采一次
                win4_r  <= I_slot_valid[3] && (I_ld_type == 4'd1);
                O_ready <= tried;          // 扫完就认（不够 3 张也用已有的）；中途换过 base 则不认
                st      <= A_FIN;
            end else begin
                k <= k + 2'd1;
            end
        end

        A_FIN: begin
            st <= A_IDLE;                  // 回 IDLE 但 tried=1，不会再扫
        end

        default: st <= A_IDLE;
        endcase

        // ★ 放在 case 之后：与开扫/扫完同拍时，"base 变了"必须压过它们
        // ★ 2026-11-02：`I_rescan`（文件页退出）走同一条作废通道 —— 清 tried/ready
        //   之后 A_IDLE 会在 lister 空下来时重扫；扫描途中收到它也只是"这次结果
        //   不认、再扫一次"，不会死锁（tried 是新扫描自己置的）。
        if ((I_base != base_r) || I_rescan) begin
            base_r  <= I_base;
            tried   <= 1'b0;
            O_ready <= 1'b0;
        end
    end
end

// ★ 列表起点 = 窗口起点。lister 的 I_offset 语义是"跳过前 N 个有效项"，
//   本模块扫描时顶层把 `I_list_all_types` 拉 0 ⇒ lister 只收/只数 BMP，
//   I_offset 正好是"第几张图"。窗口里 4 格因此全是图，win4 不会被夹在
//   中间的 wav/txt 打断（2026-11-01 夜修的就是这个）。
assign O_lst_offset = I_base;
assign O_win4       = win4_r;
assign O_rd_slot    = k;
// ★ 不用 `u_album.st` 这种跨模块层次引用：TD 综合会报
//   "external reference ... remains unresolved"（见 ps_gen_image 的同名注释）。
assign O_sel        = (st == A_COL);

endmodule
