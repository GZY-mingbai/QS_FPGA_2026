//==============================================================================
//  flash_arb.v —— 用户 SPI Flash 的四客户仲裁（2026-10-27 重写，2026-11-06 加客户 3）
//==============================================================================
//
//  【为什么需要】
//  同一颗 W25Q64 上要读四样东西：
//      · 图标覆盖率图（上电一次性 2560 B @0x03D000）
//      · 系统提示音（按键时按需 128 B 一段 @0x040000）
//      · 字库字形（菜单布局时 32 B 一个字，最频繁）
//      · 字幕字模（动态字幕装载时 32 B 一个字，★2026-11-06 新增客户 3）
//  四个客户不能同时驱动引脚，所以控制器只留一个实例，前面加这层仲裁。
//
//==============================================================================
//  ★★★ 2026-10-27：旧版有一个**会永久挂死**的结构性缺陷，本版重写 ★★★
//==============================================================================
//  【旧版的写法】
//      assign c1_rdy = free;
//      assign c2_rdy = free & ~c1_req;
//      assign c0_rdy = free & ~c1_req & ~c2_req;   // free = fl_ready & ~busy
//      assign fl_req  = free & any_req;
//      assign fl_addr = c1_req ? c1_addr : (c2_req ? c2_addr : c0_addr);
//
//  【为什么它会挂死】三个客户的请求都是**"看到 rdy → 下一拍才拉 req"**：
//      · osd_font_cache : if (rd_ready) begin rd_req_r <= 1'b1; st <= S_FRX; end
//      · osd_icon_rom   : if (fl_rdy)   begin fl_req   <= 1'b1; st <= S_RX;  end
//      · beep_player    : if (fl_rdy && ...) fl_req <= 1'b1;
//  而 `free=1` 且没人请求时，**三根 rdy 同时为 1**。于是两个客户能在同一拍 T
//  都看到 rdy，在 T+1 拍同时拉 req；仲裁器按优先级只认一个（req_owner 是个
//  三选一 mux），另一个的 req 被**静默吞掉**。
//  被吞的那个客户已经离开请求态、进了"等数据"态（如 S_FRX），
//  而 c*_valid / c*_done 只发给 owner ⇒ **它永远等不到，永久挂死**。
//
//  典型触发场景：按 KEY1 开菜单的同时会响提示音（顶层 beep_trig = |key_press），
//  于是"提示音要 Flash"和"字库要 Flash"在同一拍撞上 ⇒ 字库挂死 ⇒ line_ready
//  恒 0 ⇒ 菜单一个字都画不出来。
//
//  【本版的三条对策】
//   ① **rdy 用令牌 + 预约**(tok/offered)：同一拍只有一个人能看到 rdy，
//      而且"发出 rdy 到 req 到货"这一拍之内**不再给别人发 rdy**，
//      所以 req 到货时总线一定还空着 ⇒ 请求永不丢失。
//      令牌按 0→1→2→0 轮转，天然无饥饿（不需要优先级，也就没有优先级反转）。
//   ② **显式状态机 + 授权锁存**（照 sd_bus_arbiter 的骨架）：
//      st = F_IDLE / F_BUSY / F_ZL，owner 在授权那一拍锁进 own，
//      之后所有 valid/done 都按 own 分发，不再依赖组合优先级。
//   ③ **超时兜底**（sd_bus_arbiter 有这个，旧版没有）：
//      等不到 fl_done 就强制释放 busy，并给 owner 一个"假 done"，
//      保证任何一个客户都不会被永久卡住。
//
//  【客户协议（与原来完全一致，客户侧一行都不用改）】
//      · c*_rdy 为高 ⇒ 本拍可以发 req（单拍脉冲），且**保证被服务**
//      · req 发出后，data/valid 逐字节回来，done 与最后一个 valid 同拍
//      · 超时/空长度那一拍也会给 done（不给 valid），客户据此离开等待态
//
//  【数据分发为什么是组合的】
//      spi_flash_read 的 rd_data/rd_valid/rd_done 三者是同沿寄存的，已经对齐；
//      这里再打一拍只会多 3×8 个 FF，而且 mslice 正紧张。
//      组合直通与原 sd_bus_arbiter 的做法一致（O_img_dv = I_sd_sec_dv && st==A_IMG）。
//
//==============================================================================
module flash_arb #(
    //--------------------------------------------------------------------------
    //  超时：最长的一次是图标 2560B。SCLK=25MHz（SCLK_DIV=1）时 ≈0.82ms，
    //  即约 41k 拍 @50MHz。这里给 20ms（1e6 拍），约 24 倍余量。
    //  ★约束：必须 ≤ 1048576★ —— to_cnt / TO_MAX 是 20 位（最大 1048575）。
    //    改大之前先把 to_cnt 位宽一起加宽，否则 TO_MAX 会被静默截断。
    //--------------------------------------------------------------------------
    parameter integer TIMEOUT_CYC = 1_000_000
)(
    input  wire        clk,
    input  wire        rst,

    //------------------------------------------------------------------
    //  客户 0：字库（osd_font_cache）
    //------------------------------------------------------------------
    input  wire        c0_req,
    input  wire [23:0] c0_addr,
    input  wire [15:0] c0_len,
    output wire        c0_rdy,
    output wire [7:0]  c0_data,
    output wire        c0_valid,
    output wire        c0_done,

    //------------------------------------------------------------------
    //  客户 1：图标上电加载（osd_icon_rom）
    //------------------------------------------------------------------
    input  wire        c1_req,
    input  wire [23:0] c1_addr,
    input  wire [15:0] c1_len,
    output wire        c1_rdy,
    output wire [7:0]  c1_data,
    output wire        c1_valid,
    output wire        c1_done,

    //------------------------------------------------------------------
    //  客户 2：系统提示音（beep_player）
    //------------------------------------------------------------------
    input  wire        c2_req,
    input  wire [23:0] c2_addr,
    input  wire [15:0] c2_len,
    output wire        c2_rdy,
    output wire [7:0]  c2_data,
    output wire        c2_valid,
    output wire        c2_done,

    //------------------------------------------------------------------
    //  客户 3：字幕字模（sub_loader，★2026-11-06）
    //------------------------------------------------------------------
    input  wire        c3_req,
    input  wire [23:0] c3_addr,
    input  wire [15:0] c3_len,
    output wire        c3_rdy,
    output wire [7:0]  c3_data,
    output wire        c3_valid,
    output wire        c3_done,

    //------------------------------------------------------------------
    //  控制器口（接 spi_flash_read 的用户侧）
    //------------------------------------------------------------------
    output wire        fl_req,
    output wire [23:0] fl_addr,
    output wire [15:0] fl_len,
    input  wire        fl_ready,
    input  wire [7:0]  fl_data,
    input  wire        fl_valid,
    input  wire        fl_done
);

//==============================================================================
//  1. 状态 / 常量
//==============================================================================
localparam [1:0] F_IDLE = 2'd0;   // 空闲：发 rdy、收单
localparam [1:0] F_BUSY = 2'd1;   // 已授权：等 fl_done（或超时）
localparam [1:0] F_ZL   = 2'd2;   // 收到"空长度"请求：给一拍 done，不碰控制器

localparam [19:0] TO_MAX = TIMEOUT_CYC - 1;   // 计数器饱和值（20 位够用）

reg  [1:0]  st;
reg  [1:0]  own;         // 本段归谁（0/1/2）—— 授权那一拍锁存，之后唯一真相
reg         busy;        // 1 = 有事务在途
reg  [1:0]  tok;         // 令牌：本拍把 rdy 给谁
reg         offered;     // 1 = 上一拍已经把 rdy 发出去了，本拍等 req 到货
reg  [19:0] to_cnt;      // 等 fl_done 的计时

//==============================================================================
//  2. 授权握手
//==============================================================================
//  客户请求线先声明（下面的 rdy_en 要用）
wire        any_req  = c0_req | c1_req | c2_req | c3_req;

//  free    ：控制器真的能接新活（内部空闲 + 本层没有在途事务）
//  rdy_en  ：本拍允许**发出**新的 rdy。两个"与"缺一不可：
//     · ~offered —— 上一拍已经把 rdy 发出去了，这一拍是"等 req 到货"的预约
//                   窗口，绝不能再给别人，否则到货的 req 就没人认领了。
//     · ~any_req —— 这一拍已经有人在请求、马上就要 take 了，也不能再发 rdy，
//                   否则那个客户下一拍拉 req 时 busy 已置 1 ⇒ 它照样挂着。
//                   （★这一条是最容易漏掉的：只有 ~offered 还不够★）
wire free   = fl_ready & ~busy;
wire rdy_en = free & ~offered & ~any_req;

assign c0_rdy = rdy_en & (tok == 2'd0);
assign c1_rdy = rdy_en & (tok == 2'd1);
assign c2_rdy = rdy_en & (tok == 2'd2);
assign c3_rdy = rdy_en & (tok == 2'd3);

// 客户请求。★因为有令牌 + 预约 + 互斥，同一拍最多只会有一个 req 有效★
// 这里仍保留 c1>c2>c3>c0 的取值顺序，纯属防御（真有同拍多请求时行为可预期）。
// c3 排在最后：字幕字模晚一拍没影响（每 16 帧才装一个），不能拖住句柄菜单。
wire [1:0]  sel      = c1_req ? 2'd1 : (c2_req ? 2'd2 : (c3_req ? 2'd3 : 2'd0));
wire [23:0] sel_addr = (sel == 2'd1) ? c1_addr :
                       (sel == 2'd2) ? c2_addr :
                       (sel == 2'd3) ? c3_addr : c0_addr;
wire [15:0] sel_len  = (sel == 2'd1) ? c1_len  :
                       (sel == 2'd2) ? c2_len  :
                       (sel == 2'd3) ? c3_len  : c0_len;

// 长度 0 的请求：spi_flash_read 的 ST_IDLE 要求 rd_len != 0，
// 发了也不会启动、更不会回 done ⇒ 必须自己给 done（走 F_ZL），
// 既不能发给控制器，也不能把客户晾着。
wire sel_zl = (sel_len == 16'd0);

// 收单条件。fl_req 必须是单拍脉冲：本拍 busy 被置 1 ⇒ 下一拍 free=0 ⇒ 自动收回。
wire take = free & any_req;

// 发给控制器：地址/长度在请求这一拍组合给出。
// ★这里不需要额外锁存★ —— spi_flash_read 在 ST_IDLE 认请求的同一个时钟沿
//   就把 rd_addr 装进 tx_buf（见 spi_flash_read.v 第 268 行），
//   之后客户的 addr 再变也不影响本段事务。
assign fl_req  = take & ~sel_zl;
assign fl_addr = sel_addr;
assign fl_len  = sel_len;

// 令牌轮转：0 → 1 → 2 → 3 → 0
wire [1:0] nxt_tok = (sel == 2'd3) ? 2'd0 : (sel + 2'd1);

//==============================================================================
//  3. 超时
//==============================================================================
wire to_hit = busy && (to_cnt == TO_MAX);

//==============================================================================
//  4. 主状态机
//==============================================================================
always @(posedge clk or posedge rst) begin
    if (rst) begin
        st      <= F_IDLE;
        own     <= 2'd0;
        busy    <= 1'b0;
        tok     <= 2'd0;
        offered <= 1'b0;
        to_cnt  <= 20'd0;
    end else begin
        case (st)
        //----------------------------------------------------------------------
        //  F_IDLE：发 rdy / 收单
        //----------------------------------------------------------------------
        F_IDLE: begin
            to_cnt <= 20'd0;

            if (take) begin
                // ---- 收单：锁 owner；本拍 fl_req/addr/len 已同时给到控制器 ----
                own     <= sel;
                busy    <= 1'b1;
                offered <= 1'b0;            // 预约已兑现
                tok     <= nxt_tok;         // 下一个轮到谁（轮转 ⇒ 无饥饿）
                st      <= sel_zl ? F_ZL : F_BUSY;

            end else if (free) begin
                // ---- 没有请求：令牌轮转 ----
                //  每给出一次 rdy，必须**空等一拍**等 req 到货，才允许换人。
                //  这一拍就是"预约窗口"，缺了它就会出现"发了 rdy 却把总线
                //  给了别人"⇒ 客户挂着等 done 永不返回（旧版就是这么挂的）。
                if (offered) begin
                    offered <= 1'b0;
                    tok     <= (tok == 2'd3) ? 2'd0 : (tok + 2'd1);
                end else begin
                    offered <= 1'b1;        // 本拍 rdy 已亮出，下一拍等 req
                end
            end else begin
                // 控制器忙 / 本层忙：撤销预约，别让客户以为拿到了授权
                offered <= 1'b0;
            end
        end

        //----------------------------------------------------------------------
        //  F_BUSY：等 done，超时强制放行
        //----------------------------------------------------------------------
        F_BUSY: begin
            if (fl_done) begin
                busy   <= 1'b0;
                st     <= F_IDLE;
                to_cnt <= 20'd0;
            end else if (to_hit) begin
                // ★超时兜底★：控制器没回 done（SPI 异常 / 请求丢失），
                //  这里主动释放，并在同一拍由下面的组合逻辑给 owner 一个假 done。
                //  没有这一段，busy 会永久为 1 ⇒ free 恒 0 ⇒ 三个客户全部饿死。
                busy   <= 1'b0;
                st     <= F_IDLE;
                to_cnt <= 20'd0;
            end else begin
                to_cnt <= to_cnt + 20'd1;   // 到 TO_MAX 停住，不回绕
            end
        end

        //----------------------------------------------------------------------
        //  F_ZL：空长度请求，给一拍 done 就走（不占控制器）
        //----------------------------------------------------------------------
        F_ZL: begin
            busy   <= 1'b0;
            st     <= F_IDLE;
            to_cnt <= 20'd0;
        end

        default: begin
            st      <= F_IDLE;
            busy    <= 1'b0;
            offered <= 1'b0;
        end
        endcase
    end
end

//==============================================================================
//  5. 响应分发（全组合，与 fl_valid / fl_data 天然同拍）
//==============================================================================
//  ★为什么用 own 而不是只靠 busy★
//    busy 只说明"有事务"，own 才说明"归谁"。两者在 F_BUSY/F_ZL 期间同时为真，
//    所以下面统一用 busy & (own == n) 作门控；复位后 own=0、busy=0 ⇒ 全静默。
wire zl = (st == F_ZL);

assign c0_data = fl_data;
assign c1_data = fl_data;
assign c2_data = fl_data;
assign c3_data = fl_data;

assign c0_valid = busy & (own == 2'd0) & fl_valid & ~zl;
assign c1_valid = busy & (own == 2'd1) & fl_valid & ~zl;
assign c2_valid = busy & (own == 2'd2) & fl_valid & ~zl;
assign c3_valid = busy & (own == 2'd3) & fl_valid & ~zl;

// done：正常收工 / 超时兜底 / 空长度，三种情况都要让客户脱离等待态
assign c0_done = busy & (own == 2'd0) & (fl_done | to_hit | zl);
assign c1_done = busy & (own == 2'd1) & (fl_done | to_hit | zl);
assign c2_done = busy & (own == 2'd2) & (fl_done | to_hit | zl);
assign c3_done = busy & (own == 2'd3) & (fl_done | to_hit | zl);

endmodule
