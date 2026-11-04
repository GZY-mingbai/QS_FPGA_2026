//==============================================================================
//  quad_page_ctrl.v —— 文件页四宫格的时序控制（进入 / 翻页 / 退出）
//==============================================================================
//
//  【它管三件事】
//  1. **装载器归属**：bmp_loader 只有一个，相册和四宫格共用。
//     进入文件页时把归属抢过来（`O_own=1`），离开时还回去。
//     抢的条件是"装载器空闲"，所以永远不会打断相册正在进行的装载。
//  2. **扫描 → 装载的握手**：lister 扫完 4 项之后才发装载启动。
//  3. **翻页**：光标走到第 4 格再往下，offset += N_ITEMS，重新扫 + 重装。
//
//  【为什么单独做一个模块】
//  这段逻辑是纯时序、可穷举的；放在顶层会被 2000 行别的代码埋掉，
//  出问题只能靠上板。独立出来就能写单元测试（tb_quad_page_ctrl.v）。
//
//  【状态机】
//
//      S_IDLE ──进入文件页 && 装载器空闲──► S_SCAN
//      S_SCAN ──lister done──► S_LOAD
//      S_LOAD ──loader done──► S_SHOW
//      S_SHOW ──翻页──► S_SCAN          （offset 先改，再重扫）
//             └─离开页面 && 都空闲──► S_IDLE
//
//  【为什么"离开页面"要等 !busy】
//  扫描/装载进行到一半就放手，别人（相册）会拿到一个半成品状态的共用外设：
//  lister 那边正在收扇区，bmp_loader 那边正在写 SDRAM。等它们自然收尾
//  再交还，代价最多几十毫秒，换来的是"任何时刻只有一个主人"这个不变量。
//
//  【O_ready 为什么要单独一根】
//  4 个槽里可能一格都没装成功（空卡 / 全是 txt）。这时候继续显示四宫格
//  会把 SDRAM 里的陈货（相册帧缓存）画上去 —— 上板表现就是"花屏"。
//  所以只有装过至少一张才让上层打开四宫格。
//
//==============================================================================

module quad_page_ctrl #(
    parameter integer N_ITEMS = 4,          // 一页几格
    parameter integer MAX_OFF = 65535,      // offset 上限（16 位够用）
    //------------------------------------------------------------------
    //  兜底超时：离开文件页后，装载器还忙多久就强制交还归属
    //  2^25 @50MHz ≈ 0.67s（4 张 320×240 正常约 0.65s，所以只会命中
    //  "真卡住"的情况，比如 FAT 参数填错导致 bmp_loader 去读卡外地址）
    //------------------------------------------------------------------
    //  ★ 2026-11-03：必须 < 2^25。计数器和比较都是 25 位（STUCK_CYC[24:0]），
    //    原来写 (1 << 25) ⇒ [24:0] 截成 **0** ⇒ `scan_cnt >= 0` 恒真：每次扫描发出
    //    下一拍就"超时"、交还归属、回 IDLE 再扫 —— 无限重扫，永远装不了图
    //    （上板：四格全透明；lister 元数据反复清零 ⇒ 文件名空白/左右乱跳）。
    //    同理 force_rel 恒为 1。单元台 tb_quad_ready 覆盖成 1<<20，所以没暴露。
    //    取全 1 还让 ≥ 比较退化成一个与门。
    parameter integer STUCK_CYC = (1 << 25) - 1
)(
    input  wire        clk,
    input  wire        rst,

    //------------------------------------------------------------------
    //  页面状态（来自 OSD 层）
    //------------------------------------------------------------------
    input  wire        I_file_page,         // 1 = OSD 停在文件页
    input  wire        I_flip_next,         // 单拍：下一页
    input  wire        I_flip_prev,         // 单拍：上一页

    //------------------------------------------------------------------
    //  lister
    //------------------------------------------------------------------
    output reg         O_lst_start,         // 单拍
    output reg  [15:0] O_lst_offset,        // 本页从第几个有效项开始
    input  wire        I_lst_busy,
    input  wire        I_lst_done,          // 单拍
    input  wire [2:0]  I_lst_cnt,           // 本页有效项数
    input  wire        I_lst_has_more,

    //------------------------------------------------------------------
    //  quad_loader
    //------------------------------------------------------------------
    output reg         O_ld_start,          // 单拍
    input  wire        I_ld_busy,
    input  wire        I_ld_done,           // 单拍
    input  wire [2:0]  I_ld_loaded,         // 装成功几张（保留，不再参与判定）
    // ★ 2026-11-04：quad_loader 的第 0 遍（黑底/占位）写完 ⇒ 马上可以显示四宫格
    input  wire        I_ld_filled,
    input  wire [3:0]  I_ld_bmp,            // 本页哪几格是 BMP
    output reg         O_ld_abort,          // 单拍：翻页/离开时放弃本页的装载
    input  wire [1:0]  I_cur,               // OSD 光标格（算"打开"的图序号）

    //------------------------------------------------------------------
    //  对外
    //------------------------------------------------------------------
    output reg         O_own,               // 1 = bmp_loader 归四宫格
    output reg         O_ready,             // 1 = 可以显示四宫格
    output wire        O_has_more,          // 本页后面还有没有
    output wire [15:0] O_offset,
    // ★ 2026-11-01：每装完一页翻转一次。OSD 用它触发"文件名重排"——
    //   翻页后两边页的项数可能相同（都是 4），靠 vcnt 变化是检测不出来的。
    output reg         O_gen,
    output reg  [2:0]  O_valid_cnt,         // 本页几格有内容（名字显示用）
    output wire        O_has_prev,          // ★ 2026-11-04：前面还有上一页
    output wire [7:0]  O_rank               // ★ 光标那格是目录里第几张**图**（只对 BMP 格有意义）
);

localparam [1:0] S_IDLE = 2'd0;
localparam [1:0] S_SCAN = 2'd1;
localparam [1:0] S_LOAD = 2'd2;
localparam [1:0] S_SHOW = 2'd3;

reg [1:0]  st;
reg [15:0] offset_r;
reg        has_more_r;
reg        leave_wait;                  // 扫描/装载期间用户已经离开页面
reg        rescan;                      // ★ [SCAN-FLIP] 本次扫描被翻页作废，扫完按新 offset 重扫
// ★ [SCAN-FLIP] 重扫的 start 必须**晚一拍**再打。
//   lister 的 S_DONE 是 `if (!I_start) st <= S_IDLE` —— done 那一拍若同拍打
//   I_start，它看到的是高电平、原地不动，这个脉冲就被吞了（然后要等 0.67s 超时）。
//   晚一拍照发：T 拍 done → T+1 拉 start（lister 这拍还在 S_DONE、看到的是低）
//   → T+2 它已在 S_IDLE、正好吃下这个 start。
reg        resc_go;
reg  [24:0] stuck_cnt;                  // 离开页面后"还忙"的拍数
// ★ 2026-11-01 S3b：S_SCAN 自己的超时。
//   原来只有 S_LOAD 有 force_rel，而 S_SCAN 的出口只有 I_lst_done ——
//   一旦 done 因为归属竞争丢掉（lister 正被相册占用 / start 被丢），
//   本机永久停在 S_SCAN 且 O_own=1 ⇒ ld_can_load 永久 0 ⇒ 相册再也装不了图，
//   屏幕看起来就是"进文件页后彻底卡死、按键全无反应"。
reg  [24:0] scan_cnt;
wire        force_rel = (stuck_cnt >= STUCK_CYC[24:0]);

wire [15:0] off_next = offset_r + N_ITEMS[15:0];
wire [15:0] off_prev = offset_r - N_ITEMS[15:0];

//------------------------------------------------------------------------------
//  ★ 2026-11-04：翻页随时可用（不再等 4 张装完）
//------------------------------------------------------------------------------
//  S_SHOW 里直接扫新页；S_LOAD 里先叫装载器放弃（当前那张由 bmp_loader 尽快收尾），
//  等 quad_loader 收工再扫新页（fl_pend）。第 0 遍（黑底，≈3ms）没写完之前不收翻页，
//  O_ready 恰好就是"第 0 遍完成"，拿它门控。
wire flip_n  = I_flip_next && has_more_r;
wire flip_p  = I_flip_prev && (offset_r >= N_ITEMS[15:0]);
wire [15:0] off_new = flip_n ? off_next : off_prev;
reg  fl_pend;
reg  fill_d;
wire fill_rise = I_ld_filled & ~fill_d;
assign O_has_prev = (offset_r >= N_ITEMS[15:0]);

//------------------------------------------------------------------------------
//  ★ 2026-11-04：图序号（文件页"打开"BMP ⇒ 相册跳到第几张图）
//------------------------------------------------------------------------------
//  rk = 本页之前有几张 BMP。下一页：rk += 本页 BMP 数；上一页：等新页第 0 遍写完
//  （BMP 掩码就绪）再 rk -= 新页 BMP 数（往前翻那一页一定是满页、掩码完整）。
//  光标格的序号 = rk + 本页光标之前的 BMP 数。
reg  [7:0] rk;
reg        rk_sub;
wire [2:0] bm_all = {2'b0, I_ld_bmp[0]} + {2'b0, I_ld_bmp[1]} + {2'b0, I_ld_bmp[2]} + {2'b0, I_ld_bmp[3]};
wire [1:0] bm_lo  = {1'b0, I_ld_bmp[0] & (I_cur != 2'd0)} + {1'b0, I_ld_bmp[1] & I_cur[1]} +
                    {1'b0, I_ld_bmp[2] & (I_cur == 2'd3)};
assign O_rank = rk + {6'd0, bm_lo};

assign O_has_more = has_more_r;
assign O_offset   = offset_r;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        st           <= S_IDLE;
        offset_r     <= 16'd0;
        has_more_r   <= 1'b0;
        leave_wait   <= 1'b0;
        stuck_cnt    <= 25'd0;
        scan_cnt     <= 25'd0;
        O_lst_start  <= 1'b0;
        O_lst_offset <= 16'd0;
        O_ld_start   <= 1'b0;
        O_own        <= 1'b0;
        O_ready      <= 1'b0;
        O_valid_cnt  <= 3'd0;
        O_gen        <= 1'b0;
        O_ld_abort   <= 1'b0;
        fl_pend      <= 1'b0;
        fill_d       <= 1'b0;
        rk           <= 8'd0;
        rk_sub       <= 1'b0;
        rescan       <= 1'b0;
        resc_go      <= 1'b0;
    end else begin
        O_lst_start <= 1'b0;
        O_ld_start  <= 1'b0;
        O_ld_abort  <= 1'b0;
        fill_d      <= I_ld_filled;

        // S_SCAN 超时计数（与是否离开页面无关）
        if (st != S_SCAN)                     scan_cnt <= 25'd0;
        else if (scan_cnt != STUCK_CYC[24:0]) scan_cnt <= scan_cnt + 25'd1;

        // 离开页面后还在忙 -> 计时；一旦不忙就清零
        if (!I_file_page && (I_ld_busy || I_lst_busy)) begin
            if (stuck_cnt != STUCK_CYC[24:0]) stuck_cnt <= stuck_cnt + 25'd1;
        end else begin
            stuck_cnt <= 25'd0;
        end

        // ★ [SCAN-FLIP] 重扫的 start：done 之后一拍才打（理由见 resc_go 的声明）
        if (resc_go) begin
            O_lst_start <= 1'b1;
            resc_go     <= 1'b0;
            scan_cnt    <= 25'd0;           // 新的一轮，给它完整的超时窗口
        end

        case (st)
        //======================================================================
        S_IDLE: begin
            O_ready <= 1'b0;
            // ★ 还要等 lister 空闲：开机时相册要先扫一次目录（取 3 张图），
            //   那时用户可能已经进文件页。若此时发 start，lister 正忙会**丢掉**
            //   这个脉冲，而本机已经进了 S_SCAN 等 done ⇒ 永久等待（页面卡死）。
            if (I_file_page && !I_ld_busy && !I_lst_busy) begin
                O_own        <= 1'b1;
                O_lst_offset <= offset_r;
                O_lst_start  <= 1'b1;
                leave_wait   <= 1'b0;
                rescan       <= 1'b0;
                st           <= S_SCAN;
            end else begin
                // 没进文件页 / 装载器还在忙 -> 交还归属（忙的时候先不放）
                if (!I_ld_busy) O_own <= 1'b0;
            end
        end

        //======================================================================
        S_SCAN: begin
            if (!I_file_page) leave_wait <= 1'b1;

            // ★★ 2026-11-02 [SCAN-FLIP]：扫描期间按翻页**不再丢键** ★★
            //   【原来】这个状态只认 I_lst_done / 超时，flip 一根线都没接 ⇒
            //     扫描那 5~20ms 里按的翻页被直接吞掉（用户只能再按一次）。
            //   【现在】把新 offset 记下来、本次扫描结果作废（rescan），扫完立刻
            //     按新 offset 重发单。lister 一行都不用改（它本来就在扇区末收工）。
            //   为什么不直接 abort lister：它中途撤 req 会让下次扫描读到残余字节，
            //     只能等扇区末；而一次扫描本来就只有几个扇区，等它更简单。
            if (flip_n || flip_p) begin
                offset_r <= off_new;
                O_ready  <= 1'b0;
                if (flip_n) rk     <= rk + {5'd0, bm_all};
                else        rk_sub <= 1'b1;
                rescan   <= 1'b1;
            end

            if (I_lst_done) begin
                if (rescan || flip_n || flip_p) begin
                    // 本次结果是旧 offset 的 ⇒ 丢掉，按新 offset 重扫（晚一拍发 start）
                    rescan       <= 1'b0;
                    O_lst_offset <= (flip_n || flip_p) ? off_new : offset_r;
                    resc_go      <= 1'b1;
                end else begin
                O_valid_cnt <= I_lst_cnt;
                has_more_r  <= I_lst_has_more;
                if (leave_wait || !I_file_page) begin
                    // 已经不看了：不装载，直接交还（卡住时靠 force_rel 兜底）
                    if ((!I_ld_busy && !I_lst_busy) || force_rel) begin
                        O_own <= 1'b0;
                        st    <= S_IDLE;
                    end
                end else if (I_lst_cnt == 3'd0) begin
                    // 空页：没有可装的，也算"完成"（显示 4 个空位）
                    O_ready <= 1'b0;
                    O_gen   <= ~O_gen;
                    st      <= S_SHOW;
                end else begin
                    // ★ 2026-11-04：扫完就通知 OSD 重排名字（原来要等 4 张全装完 ⇒
                    //   上板"第 2 页先显示上一页的 4 个名字，装完才变 2 个"）
                    O_gen      <= ~O_gen;
                    O_ld_start <= 1'b1;
                    st         <= S_LOAD;
                end
                end
            end else if (scan_cnt >= STUCK_CYC[24:0]) begin
                // 扫描 never came back：强制交还归属，回 IDLE 重新来过
                O_own   <= 1'b0;
                O_ready <= 1'b0;
                st      <= S_IDLE;
            end
        end

        //======================================================================
        S_LOAD: begin
            // 离开页面：叫装载器尽快收尾（不用再等 4 张装完才交还）
            if (!I_file_page) begin
                leave_wait <= 1'b1;
                O_ld_abort <= 1'b1;
            end
            // 第 0 遍写完 ⇒ 格子里全是黑底/占位，可以显示了；往前翻的话现在才知道新页 BMP 数
            if (fill_rise && !fl_pend) begin
                O_ready <= 1'b1;
                if (rk_sub) rk <= rk - {5'd0, bm_all};
                rk_sub  <= 1'b0;
            end
            if (I_ld_done) begin
                if (fl_pend) begin
                    fl_pend     <= 1'b0;
                    rescan      <= 1'b0;
                    O_lst_start <= 1'b1;
                    st          <= S_SCAN;
                end else
                    st <= S_SHOW;
            end else if ((flip_n || flip_p) && O_ready && !fl_pend) begin
                offset_r   <= off_new;
                O_ready    <= 1'b0;
                O_ld_abort <= 1'b1;
                fl_pend    <= 1'b1;
                if (flip_n) rk <= rk + {5'd0, bm_all};
                else        rk_sub <= 1'b1;
            end else if (force_rel) begin
                // 装载器卡死（读卡外地址等）：放弃这一页，交还归属
                O_ready <= 1'b0;
                O_own   <= 1'b0;
                fl_pend <= 1'b0;
                st      <= S_IDLE;
            end
        end

        //======================================================================
        S_SHOW: begin
            if (flip_n || flip_p) begin
                offset_r    <= off_new;
                O_lst_start <= 1'b1;
                O_ready     <= 1'b0;          // 翻页期间先不显示（避免画陈货）
                rescan      <= 1'b0;          // 这一单是新的，别被上一轮的作废标志带偏
                if (flip_n) rk <= rk + {5'd0, bm_all};
                else        rk_sub <= 1'b1;
                st          <= S_SCAN;
            end else if (!I_file_page && ((!I_ld_busy && !I_lst_busy) || force_rel)) begin
                O_own   <= 1'b0;
                O_ready <= 1'b0;
                st      <= S_IDLE;
            end
        end

        default: st <= S_IDLE;
        endcase
    end
end

endmodule
