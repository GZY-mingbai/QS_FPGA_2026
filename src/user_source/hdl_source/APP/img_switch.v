//==============================================================================
//  img_switch.v —— 换图决策 + 缩略图滑动窗口缓存（全工程唯一的一份）
//==============================================================================
//  【谁用它】viewer_ctrl 里只例化这一份：
//      主页        KEY3/KEY4/自动切换 → I_next / I_prev，带过渡动画（viewer_ctrl 走 S_ARM）
//      OSD 相册页  上一张/下一张      → I_next / I_prev，**不播动画**（帧边界直接换 a_base）
//      OSD 文件页  KEY2 打开某张 BMP  → I_jump(I_jump_idx)，不播动画
//    "要不要动画"由调用方决定（anim = !in_osd）；本模块只管
//    "换不换得了、换到哪、缓存里谁该装"。
//
//  【缓存：3 块缩略图是一个环】
//    当前图 cur 住在槽 cs，下一张住 cs+1，上一张住 cs−1（mod 3）。
//    窗口 = [cur−1, cur, cur+1]（第 1 张时 = [cur, cur+1]）。
//    开机：只装 cur、再装 cur+1（用户口径：图1、图2、空）。
//    下一张：cs+1 变成当前；原来 cs−1 那块（上一张的上一张）作废，改装新的 cur+1：
//        [1 2 _] → [1 2 3] → [4 2 3] → [4 5 3] → [4 5 6] → [7 5 6] …
//    上一张对称。**每次换图正好补装 1 张**，而且补装的那块此刻不在屏幕上
//    （过渡只画旧 cur 和新 cur）⇒ 可以和过渡动画并行装。
//
//  【★★ 2026-11-04：换图资格不再看"窗口里有几张" ★★】
//    原来用 album_src 的 `O_have`（窗口前 3 格里有几张 BMP）推"还有没有下一张"：
//        n_ex = (cr==0) ? have[1] : (cr==1) ? (have==3) : 0
//    这个口径是错的 —— `have` 描述的是"窗口里装了几张"，不是"后面还有没有图"，
//    而且它 2 位封顶 3。上板实测：窗口里只有 1 张、但第 4 格是图（have=1、win4=1）时
//    `n_ex` 恒 0 ⇒ cur+1 一张都不预取 ⇒ tv[sn] 永远为 0 ⇒ **下一张按了没反应**。
//    （77c748c 没这个问题：它不看 have，按键就装，装完再转场。）
//
//    现在改成"**试了再算**"，和 77c748c 同一种行为：
//      · 只要目标槽没装过（~tv）、也没被判过"这里没有图"（~dv），就发一次预取；
//      · 装成功 ⇒ tv 置位，可以换过去；
//      · 装载器报硬错（文件不存在 / 头校验失败）⇒ dv 置位，"这个方向没有图"；
//      · 超时不算硬错（不置 dv），下次还会再试 —— 瞬态卡顿不会把翻页永久锁死。
//    `dv` 在三种情况下清 0：该槽被改派（换图）、开始往里装、跳转清空全部。
//    ⇒ 走到相册最后一张时只会多一次失败装载（约 0.3s），换来的是一条**不需要
//      任何看门狗、也不会被相册计数卡死**的换图通路。
//
//  【相册窗口】album_src 按 O_base 给出 4 个偏移（窗口 [base, base+3]）。
//    cr = cur 在窗口里的位置 0..2：平时 cur 在 rel1（base = cur−1）；
//    第 1 张在 rel0；**最后一张在 rel2**（I_win4=0 ⇒ 后面没图了，不再滑 base，
//    省一次重扫）。base 变了要等 album_src 重扫完（O_alb_ok）才能用偏移。
//    为什么 I_win4 可以直接用：换图要求目标块已装好，而那次装载本身就等过
//    本窗口扫完 ⇒ 能换图的那一拍，win4 一定是当前窗口的。
//
//  【装载中途换图】正在装的那块如果在这次换图里被"改派"了（下一张时的旧 cs−1、
//    上一张时的旧 cs+1、跳转时全部），装完也不能算有效（tstale）。
//==============================================================================
module img_switch #(
    parameter [20:0] THUMB0_ADDR = 21'd614400,
    parameter [20:0] THUMB1_ADDR = 21'd652800,
    parameter [20:0] THUMB2_ADDR = 21'd691200
)(
    input  wire        clk,
    input  wire        rst,

    // ---- 换图请求（单拍；调用方已按状态门控）----
    input  wire        I_next,
    input  wire        I_prev,
    input  wire        I_jump,
    input  wire [7:0]  I_jump_idx,       // 目录里第几张图（0 起）
    input  wire        I_pf_en,          // 1 = 允许预取邻居（开机/跳转装当前图时关）

    // ---- 相册（album_src）----
    output wire [15:0] O_base,
    input  wire        I_any,            // 1 = 本窗口至少有一张图（只用于空目录保护）
    // ★ [NIMG]：本窗口实际图数。`nn` 用它判"窗口第 cr+1 格到底有没有图" ——
    //   原来只看 `cr != 2`，末尾窗口里那格是**上个窗口的残留偏移** ⇒ 预取装成功
    //   ⇒ 末张之后还能再切一步（还是同一张）。这里只做"少发一单"，不碰装载。
    input  wire [2:0]  I_nimg,
    input  wire        I_win4,           // 窗口第 4 项也是图（后面还有）
    input  wire        I_alb_ready,
    output wire        O_alb_ok,         // 1 = 偏移属于当前窗口

    // ---- 缩略图装载请求（给 viewer_ctrl 的序列器）----
    output wire        O_tl_req,
    output wire [1:0]  O_tl_rel,         // 用相册第几个偏移
    output wire [20:0] O_tl_addr,
    input  wire        I_tl_take,        // 序列器收下（单拍）
    input  wire        I_tl_fin,         // 缩略图装载结束（单拍）
    input  wire        I_tl_ok,          // 1 = 装成功
    input  wire        I_tl_err,         // 1 = 装载器报硬错（不是超时）
    // ★ 2026-11-02 [ABORT-NOT-ERR]：这次结束是**被主动放弃**的（bmp_loader 的 abort）。
    //   bmp_loader 把 abort 和"头校验失败"都报成 err，但两者对缓存的含义完全相反：
    //     · 真错 ⇒ dv=1（"这格没有图"，这个方向不该再试）
    //     · 放弃 ⇒ 什么都不记（下次还要装）
    //   原来没有这根，任何 abort 都会把目标方向的 dv 置起来 ⇒ 那个方向永久翻不动
    //   （go_n 要求 vn，而 nn 又被 dn 挡掉），这是"abort 不能用在缩略图上"的唯一原因。
    //   本信号与 I_tl_fin 同拍（或更早，见 viewer_ctrl 的 ab_pulse），等效于 tstale。
    input  wire        I_tl_abort,

    // ---- 源图分辨率（信息条用）★ 2026-11-02 ----
    //   本次装载解析出的源图 W/H（= bmp_loader 的 w32/h32）。缩略图和高清读的是
    //   同一个 BMP 头，所以在**缩略图装完那一拍**就拿到了当前图的答案。
    input  wire [15:0] I_src_w,
    input  wire [15:0] I_src_h,
    output wire [12:0] O_res_w,          // 当前图（环槽 cs）的源宽
    output wire [12:0] O_res_h,          // 当前图（环槽 cs）的源高

    // ---- 结果 ----
    output wire        O_go,             // 本拍换图成立
    output reg         O_dir,            // 最近一次：1 = 下一张
    output wire [1:0]  O_cur_rel,        // 当前图在相册窗口里的位置（高清用这个偏移）
    output wire [20:0] O_cur_addr,       // 当前图缩略图
    output wire [20:0] O_old_addr,       // 换图前那张（过渡的 A）
    output wire        O_cur_ok,         // 当前图缩略图已装好
    output wire [7:0]  O_cur             // 当前图序号（观测用）
);

reg  [7:0] base;
reg  [1:0] cr;            // cur 在窗口里的位置 0..2
reg  [1:0] cs;            // cur 所在的槽 0..2
reg  [2:0] tv;            // 各槽内容有效（装好了）
reg  [2:0] dv;            // 各槽"试过、这里没有图"（硬错）—— 与 tv 互斥
reg        ld_on;         // 有一张缩略图在装
reg  [1:0] ts;            // 正在装哪个槽
reg        tstale;        // 正在装的那块已被改派
reg        bpend;         // base 刚变，等 album_src 把 ready 拉低再拉高

wire [1:0] sn = (cs == 2'd2) ? 2'd0 : cs + 2'd1;     // 下一张的槽
wire [1:0] sp = (cs == 2'd0) ? 2'd2 : cs - 2'd1;     // 上一张的槽
wire       vc = tv[cs];
wire       vn = tv[sn];
wire       vp = tv[sp];
wire       dn = dv[sn];                              // 下一张那块已确认"没有图"
wire       dp = dv[sp];                              // 上一张那块已确认"没有图"

function [20:0] taddr;
    input [1:0] s;
    taddr = (s == 2'd0) ? THUMB0_ADDR : (s == 2'd1) ? THUMB1_ADDR : THUMB2_ADDR;
endfunction

//------------------------------------------------------------------------------
//  ★ 2026-11-02【信息条分辨率】：按**环槽**记住每张缩略图的源分辨率
//------------------------------------------------------------------------------
//  【为什么必须存 3 份】缩略图装载时 `ld_src_w/h` 就解析出源图 W/H 了，但那是
//  **一份活寄存器**，内容 = "最近一次装载"。而 O_tl_req 装的是**邻居**（预取），
//  切图那一刻它通常已经不是目标图 ⇒ 想"切图当拍就显示当前图的分辨率"，只能按槽记。
//  【为什么按环槽(cs)而不是 rel】滑窗时 rel 会整体平移（base±1、cr 不变），
//  环槽不会 —— rel 域存的东西过一次滑窗就得搬一次家。
//  【为什么是 LUTRAM 而不是 78 个 FF + 3:1 mux】取值靠地址就够了：深度 4 的
//  分布式 RAM，每个 LUT4 存一个 bit 列（4 个槽用掉 16 格里的 4 格），读口
//  **不额外花 mux** ⇒ 约 26 LUT / 0 FF。写成寄存器 + 3:1 是 52 LUT + 78 FF
//  （同一份数据贵 5 倍，而且 78 FF 占 slices），别改回去。
//  【写法】与 fat_dir_lister 的 name_mem 完全一致（组合读 + 注释式 pragma）：
//  TD 只认 `none | dram | bram | bram_144k`，`ram_style="block"` 会被静默忽略。
//  【宽度】bmp_loader 的 hdr_match 把 W/H 限在 1..4096 ⇒ 13 位/维足够。
//  【上电初值】LUTRAM 没有复位。正常路径上信息条最早也要等首张缩略图装完才可能
//  可见（viewer_ctrl 的 info_show 不含 S_BOOT_T），所以不会显示没写过的槽；
//  仿真里在第一次装载之前读到 X 是预期的。
//  【为什么不用 tstale 之外的门控】`~tstale` 是必要的：被改派/放弃的那次装载
//  写的不是本槽的内容，写进去就是"别人的分辨率"。
reg [25:0] res_ram [0:3];       /* fehdl force_ram=1, ram_style="dram" */

always @(posedge clk) begin
    if (I_tl_fin && I_tl_ok && !tstale)
        res_ram[ts] <= {I_src_w[12:0], I_src_h[12:0]};
end

// 组合读：地址 = 当前图所在环槽 ⇒ 切图当拍（cs 变）就跟着变，不需要任何握手。
// （全模块只允许这一个 res_ram 读表达式，否则综合会复制成两份 LUTRAM。）
wire [25:0] res_q = res_ram[cs];
assign O_res_w = res_q[25:13];
assign O_res_h = res_q[12:0];

// 能换过去 = 目标块装好了、且没被判过"这里没有图"。
// cr 的边界：窗口只有 rel 0..3 四个偏移，cur 在 rel2（cr=2）时再往后没有 rel3 可指
//   （3 槽环也表达不了）⇒ 直接判"没有下一张"，不靠任何计数。
wire go_n = I_next & vn & ~dn & (cr != 2'd2);
wire go_p = I_prev & vp & ~dp & (cr != 2'd0);
assign O_go = go_n | go_p | I_jump;

// 跳转后的窗口：base = J−1（J=0 时 0），cr = (J != 0) ? 1 : 0
wire       j_nz = (I_jump_idx != 8'd0);
wire [7:0] j_base = j_nz ? (I_jump_idx - 8'd1) : 8'd0;

assign O_alb_ok = I_alb_ready & ~bpend;

//------------------------------------------------------------------------------
//  本窗口有没有下一张 / 上一张
//------------------------------------------------------------------------------
// 不再由"窗口里有几张"推：没装过就试一次（nn/np），装失败由 I_tl_err 记 dv。
// c_ex 只表示"相册非空"，空目录时一单都不发（否则会去读 0 号扇区的 MBR）。
wire p_ex = (cr != 2'd0);
wire c_ex = I_any;
// ★ [NIMG]：窗口第 cr+1 格真的有图吗（cr=0 时要 nimg≥2，cr=1 时要 nimg≥3…）
wire n_ex = ({1'b0, cr} + 3'd1) < I_nimg;
wire nc   = ~vc & c_ex;
wire nn   = c_ex & ~vn & ~dn & I_pf_en & n_ex;
wire np   = c_ex & ~vp & ~dp & I_pf_en & p_ex;
// 换图那一拍不发新单（否则"收下"与"改派"同拍，tstale 判不到）
assign O_tl_req  = O_alb_ok & ~ld_on & (nc | nn | np) & ~(I_next | I_prev | I_jump);
assign O_tl_rel  = nc ? cr : (nn ? cr + 2'd1 : cr - 2'd1);
wire [1:0] tl_s  = nc ? cs : (nn ? sn : sp);
assign O_tl_addr = taddr(tl_s);

assign O_base     = {8'd0, base};
assign O_cur_rel  = cr;
assign O_cur_addr = taddr(cs);
assign O_old_addr = taddr(O_dir ? sp : sn);
assign O_cur_ok   = vc;
assign O_cur      = base + {6'd0, cr};

always @(posedge clk or posedge rst) begin
    if (rst) begin
        base <= 8'd0; cr <= 2'd0; cs <= 2'd0; tv <= 3'd0; dv <= 3'd0;
        ld_on <= 1'b0; ts <= 2'd0; tstale <= 1'b0; bpend <= 1'b0;
        O_dir <= 1'b1;
    end else begin
        if (!I_alb_ready) bpend <= 1'b0;

        // ---- 开始装：清掉这块的"没有图"标记（这次重新试）----
        if (I_tl_take) begin
            ld_on <= 1'b1; ts <= tl_s; tstale <= 1'b0; dv[tl_s] <= 1'b0;
        end
        // ---- 被主动放弃：结果作废（既不置 tv、也不置 dv）----
        //   粘滞到下一次 I_tl_take：放弃之后 tstale 一直保持着，等那次装载真正收尾。
        if (I_tl_abort && ld_on) tstale <= 1'b1;
        // ---- 装载结束：成功置 tv；硬错置 dv（超时不算，下次还试）----
        if (I_tl_fin) begin
            ld_on <= 1'b0;
            if (!tstale) begin
                if (I_tl_ok) begin
                    tv[ts] <= 1'b1; dv[ts] <= 1'b0;
                end else if (I_tl_err) begin
                    tv[ts] <= 1'b0; dv[ts] <= 1'b1;
                end
            end
        end

        // ---- 换图 ----
        if (go_n) begin
            O_dir <= 1'b1;
            cs    <= sn;
            tv[sp] <= 1'b0;                          // 旧上一张那块改派给新的下一张
            dv[sp] <= 1'b0;
            if (ld_on && ts == sp) tstale <= 1'b1;
            // 在 rel1 且后面还有图 ⇒ 窗口滑一格（cur 仍在 rel1）；否则 cur 在窗口里右移
            if ((cr == 2'd1) && I_win4) begin base <= base + 8'd1; bpend <= 1'b1; end
            else                         cr <= cr + 2'd1;
        end else if (go_p) begin
            O_dir <= 1'b0;
            cs    <= sp;
            tv[sn] <= 1'b0;                          // 旧下一张那块改派给新的上一张
            dv[sn] <= 1'b0;
            if (ld_on && ts == sn) tstale <= 1'b1;
            // 在 rel1 且前面还有窗口 ⇒ 滑回一格；否则 cur 在窗口里左移
            if ((cr == 2'd1) && (base != 8'd0)) begin base <= base - 8'd1; bpend <= 1'b1; end
            else                                 cr <= cr - 2'd1;
        end else if (I_jump) begin
            cs     <= sn;                            // 落在一块"现在不在屏幕上"的槽
            tv     <= 3'd0;
            dv     <= 3'd0;
            tstale <= ld_on;
            base   <= j_base;
            cr     <= {1'b0, j_nz};
            if (j_base != base) bpend <= 1'b1;
        end
    end
end

endmodule
