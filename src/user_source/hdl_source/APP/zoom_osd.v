//==============================================================================
//  zoom_osd.v —— 缩放 OSD：按键状态机 + 档位 + 锚点 + 停手防抖
//==============================================================================
//
//  【它在链路里的位置】
//      key ─► 本模块 ─┬─ zoom ─────────────► 顶层 bmp_loader（几何）
//                     ├─ ax / ay ──────────► 同上（锚点 = 屏幕上的十字中心）
//                     ├─ refresh ──────────► App：补一帧 888（十字条要重画）
//                     ├─ preview_req ──────► App：切 565 缩略图（缩放动画用）
//                     └─ load_req ─────────► App：停手够久，踢一次真正重装
//
//  【几何为什么不用 cam】
//    `bmp_view` 收到的是屏幕锚点 (ax,ay)，内部自己推 cam：
//        cam = W/2 + (ax − 320)·(step_fit − step)
//    锚点固定时 cam 对 step 线性 ⇒ 多档缩放累积与一次直算等价，
//    所以本模块**没有任何 cam 寄存器**，也没有跨装载的几何状态。
//
//  【键位（面板丝印）】
//      KEY2  菜单关着时 → 打开；打开后 → 轮转功能：左右 / 上下 / 缩放
//      KEY4  按当前功能：左移 / 上移 / 缩小一档
//      KEY3  按当前功能：右移 / 下移 / 放大一档
//      KEY1  退出
//
//  【为什么 ax/ay 只在 zoom=1.0 时可动】
//    cam 公式的前提是"锚点在缩放期间不变"。zoom=1 时 cam 恒等于图心、
//    画面不动 ⇒ 挪锚点只是挪准星（零成本）；一旦离开 1.0 就锁死锚点，
//    否则 cam 会随锚点漂移，公式不再成立。
//
//  【防抖】
//    改 zoom 才进 Z_PRE 并开始数 500ms；任何按键清零。
//    挪 ax/ay 不触发装载（zoom=1 时画面本来就不动），只发 refresh 补帧。
//
//  【状态】
//      Z_OFF   关闭
//      Z_HD    已打开，画面是 888 停帧，十字条静止
//      Z_PRE   已切 565 预览，等停手
//      Z_LOAD  已发 load_req，等 ld_done
//==============================================================================
module zoom_osd #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer DB_MS  = 500,          // 停手多久后真正重装
    parameter [7:0]   Z_FIT  = 8'h10         // 1.0×
)(
    input  wire        clk,
    input  wire        rst,

    input  wire [3:0]  key_press,       // {KEY4,KEY3,KEY2,KEY1}，单拍
    input  wire        menu_on,         // OSD 菜单开着 ⇒ 不许进入
    input  wire        allow,           // 允许进入（S_HD / S_STUCK）
    input  wire        ld_done,         // 装载完成（单拍）
    // ★★ [ZOOM-SEQ]：888（高清画布）此刻**真的在显示**
    //  = a_hd & ~fmt565（viewer_ctrl 的输出）。上一次缩放的"切回 888"要等
    //  帧边界才提交，那几帧里画面还是 565 预览 ⇒ 这时候接受新的缩放会跳变。
    //  所以缩放/平移的按键门控里都要带上它；提前按的键记在 nxt_v 里，等它就绪再执行。
    input  wire        hd_shown,
    // ★★ 2026-10-30 [ZOOM-RESTORE]：缩放 >1.0 时"开菜单 / 翻页"先恢复 1.0× ★★
    //  App 侧把这三个键压成**一个 2 位行为码**（只有"需要先恢复"时才非零）：
    //      2'd0 = 无   2'd1 = 上一页   2'd2 = 下一页   2'd3 = 打开 OSD
    //  本模块原样寄存这个码，恢复 + 重装完成、回到静止态后由 act_go/act_sel 放行。
    //  （不存 3 个按键位 = 省掉一层"3 位 → 2 位"的编码逻辑）
    input  wire [1:0]  act_code,        // 1=上一页 2=下一页 3=打开OSD

    output reg         zoom_on,
    output reg  [7:0]  zoom,            // Q4.4：0x10=1× 0x20=2× 0x40=4× 0x80=8×
    output reg  [9:0]  ax, ay,          // 屏幕锚点（十字中心），0..639 / 0..479
    output reg  [1:0]  mode,            // 0=左右 1=上下 2=缩放

    output reg         refresh,         // 单拍：十字条变了，App 补一帧 888
    output reg         preview_req,     // 单拍：zoom 变了，App 切 565
    output reg         load_req,        // 单拍：停手够久，App 踢重装
    // ★ [ZOOM-RESTORE] 恢复完成，放行被暂存的动作
    output reg         act_go,          // 单拍
    output reg  [1:0]  act_sel,         // 1=上一页 2=下一页 3=打开OSD
    // ★ [ZOOM-RESTORE] "这是一次静默恢复"：App 用它把 OSD 层在合成级屏蔽掉
    //   （要复用 S_ZOOM 的 565 预览做动画，但用户不该看到十字线/缩放面板）。
    output wire        restore
);

//------------------------------------------------------------------------------
//  0. 常量
//------------------------------------------------------------------------------
localparam [1:0] Z_OFF  = 2'd0,
                 Z_HD   = 2'd1,
                 Z_PRE  = 2'd2,
                 Z_LOAD = 2'd3;

localparam [26:0] DB_CYC = CLK_HZ / 1000 * DB_MS;   // 500ms @50MHz = 25_000_000
// ★ 2026-10-30：DB_MS 只用来推导装载超时 LD_TO（原来的 500ms 防抖已去掉）。
// ★ 装载等待超时：装载最长 ~0.6s，给到 1.5s 富余。
//   没有它，一旦装载失败（ld_err）就永远等不到 ld_done ⇒ 卡在 Z_LOAD，
//   除 KEY1 外全部按键失效 —— 上板就是"动画播完直接死机"。
localparam [26:0] LD_TO  = DB_CYC * 3;

localparam [9:0] X_MID = 10'd320;
localparam [9:0] Y_MID = 10'd240;
localparam [9:0] X_LO  = 10'd8;
localparam [9:0] X_HI  = 10'd632;
localparam [9:0] Y_LO  = 10'd8;
localparam [9:0] Y_HI  = 10'd472;

reg [1:0]  st;
reg [26:0] db_cnt;
// ★ [ZOOM-RESTORE]
reg        rest;                    // 本次会话是为了"恢复 1.0×"而开着的
reg        pend_v;                  // 有被暂存的动作
reg  [1:0] pend_sel;                // 行为码：1=上页 2=下页 3=打开OSD
// ★★ 2026-11-02 [ZOOM-RESTEP]：装载期间按下的缩放键"立刻生效"★★
//  【原来（ZOOM-NOCOOL）】Z_LOAD 里只记 1 步（nxt_v），而且要等 ld_done、
//    回到 Z_HD 才执行 ⇒ 1×→2×→4×→8× 连按 3 下要**串行跑 3 次整幅 888 烘焙**
//    （12MP 源每次约 4s ⇒ 12s），用户体感就是"每次都要等清晰图好了才能按下一档"。
//  【现在】装载中按键 ⇒ 直接改档位 + 发 load_req；viewer_ctrl 收到后**放弃**
//    飞着的那次烘焙、回 565 预览，再用最新档位重新起单。中间的档位本来就不会
//    被看到 ⇒ N 次按键只装 1 次。
//  【为什么还要 out_n】放弃会产生一个"额外的 ld_done"（bmp_loader 收尾时照报
//    done+err）。原来用"记一步、等 done"回避了这个问题，现在必须自己记账：
//    out_n = 已发出、还没见到 done 的装载数（0..3）。只有"这次 done 把账清零"
//    才算这一轮烘焙结束 —— 否则会把被砍掉那次的 done 当成结束，直接跳到 Z_HD。
//    代价 2 个 FF；换掉 nxt_v/nxt_up 正好抵扣。
reg  [1:0]  out_n;                  // 在飞装载数（饱和 3）

// 静默恢复标志直接导出给 App（合成级屏蔽 OSD 层）
assign restore = rest;

wire k1 = key_press[0];
wire k2 = key_press[1];
wire k3 = key_press[2];
wire k4 = key_press[3];

// 只有 1.0× 时才允许挪锚点
wire at_fit = (zoom == Z_FIT) && zoom_on;

// ★ [ZOOM-RESTEP] 两个组合判据（放在 k1..k4 之后 —— iverilog 要求先声明后用）
//   step_ok：这一拍是一次有效的"换档"（放大到顶 / 缩到 1× 时不再响应）
//   req_now：这一拍真的会发一单（与 Z_PRE / Z_LOAD 两处 load_req 赋值一一对应）
wire step_ok = (k3 && (zoom != 8'h80)) || (k4 && (zoom != Z_FIT));
// ★ [ZOOM-RESTEP]：req_now 必须与"真正发 load_req 的地方"严格一致。
//   ★ 2026-11-02 [ZOOM-EXIT-FAST]：Z_PRE 在恢复会话（rest）里**不再发单**
//     ⇒ 这里也要把 rest 排掉，否则 out_n 会多记一笔，Z_LOAD 的出口判据偏掉。
wire req_now = ((st == Z_PRE) && !k1 && !rest) ||
               ((st == Z_LOAD) && step_ok && !rest);

//------------------------------------------------------------------------------
//  1. 档位：1 / 2 / 4 / 8（全是 2 的幂，bmp_view 侧只需移位）
//------------------------------------------------------------------------------
function [7:0] z_up;
    input [7:0] z;
    case (z)
        8'h10:   z_up = 8'h20;
        8'h20:   z_up = 8'h40;
        default: z_up = 8'h80;       // 0x40 / 0x80 都停在 8×
    endcase
endfunction

function [7:0] z_dn;
    input [7:0] z;
    case (z)
        8'h80:   z_dn = 8'h40;
        8'h40:   z_dn = 8'h20;
        default: z_dn = Z_FIT;       // 0x20 / 0x10 都回到 1×
    endcase
endfunction

//------------------------------------------------------------------------------
//  2. 锚点步进候选值（组合）
//------------------------------------------------------------------------------
wire [9:0] ax_p = (ax >= X_HI) ? X_HI : (ax + 10'd8);
wire [9:0] ax_m = (ax <= X_LO) ? X_LO : (ax - 10'd8);
wire [9:0] ay_p = (ay >= Y_HI) ? Y_HI : (ay + 10'd8);
wire [9:0] ay_m = (ay <= Y_LO) ? Y_LO : (ay - 10'd8);

//------------------------------------------------------------------------------
//  3. 按键能不能落到某个功能上（必须声明在 always 之前，iverilog 不允许先用后声明）
//------------------------------------------------------------------------------
wire pan_x_ok = at_fit && (mode == 2'd0) && (st != Z_LOAD) && hd_shown;
wire pan_y_ok = at_fit && (mode == 2'd1) && (st != Z_LOAD) && hd_shown;
wire zoom_ok  = zoom_on && (mode == 2'd2) && (st != Z_LOAD) && hd_shown;

//------------------------------------------------------------------------------
//  4. 主状态机
//------------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        st          <= Z_OFF;
        zoom_on     <= 1'b0;
        zoom        <= Z_FIT;
        ax          <= X_MID;
        ay          <= Y_MID;
        mode        <= 2'd0;
        refresh     <= 1'b0;
        preview_req <= 1'b0;
        load_req    <= 1'b0;
        db_cnt      <= 26'd0;
        rest        <= 1'b0;
        pend_v      <= 1'b0;
        pend_sel    <= 2'd0;
        act_go      <= 1'b0;
        act_sel     <= 2'd0;
        out_n       <= 2'd0;
    end else begin
        refresh     <= 1'b0;
        preview_req <= 1'b0;
        load_req    <= 1'b0;
        act_go      <= 1'b0;
        // ---- 在飞装载计数：发出 +1，收到 done −1（同拍有按有 done ⇒ 净不变）----
        //   req_now 必须与下面真正发 load_req 的两处严格一致：Z_PRE（不为退出时）
        //   和 Z_LOAD 的"再走一步"。写在别处就会漏计，Z_LOAD 的出口判据就偏了。
        if (req_now) begin
            if (!ld_done && (out_n != 2'd3)) out_n <= out_n + 2'd1;
        end else if (ld_done) begin
            if (out_n != 2'd0) out_n <= out_n - 2'd1;
        end

        // ---- 计时器：现在只剩"装载超时"一个用途 ----
        // ★★ 2026-10-30：500ms 防抖已按需求去掉（进 Z_PRE 立刻重装），
        //   所以 db_cnt 只在 Z_LOAD 里计数，到达 LD_TO 就兜底退出，
        //   保证装载失败/卡住时状态机也不会死等。
        if (st != Z_LOAD)
            db_cnt <= 27'd0;
        else if (db_cnt != LD_TO)           // ★ [AREA] 从 0 逐一递增、停在 LD_TO ⇒ != 与 < 等价
            db_cnt <= db_cnt + 27'd1;

        case (st)
            //--------------------------------------------------------------
            Z_OFF: begin
                // ★ [ZOOM-RESTEP] 收工态清账：超时/KEY1 退出时可能还有没等到 done 的
                //   装载（那次装载的 done 已经没人收），不清会把下一轮的计数带偏。
                out_n <= 2'd0;
                if (k2 && !menu_on && allow) begin
                    // ★★ 2026-10-30：重开时**保留上次状态**（zoom / ax / ay / mode）★★
                    //  原来是 `zoom<=Z_FIT; ax<=中心; mode<=0` 全部重置 ⇒ 上板
                    //  "缩放到 2.0x，关掉再打开，左上角变回 1.0x X-PAN"，而且
                    //  内部 zoom 与实际显示的放大画面不一致（画面还是 2×，
                    //  但按 KEY3 是从 1× 再跳 2×）。现在一律保留：
                    //    · 画面本来就是当前 zoom 的重装结果，不需要重装；
                    //    · OSD 文字由 zoom_rise 触发布局，会用保留的 zoom/mode 重画。
                    //  上电首次打开时这四个量都是复位值（1.0x / 中心 / X-PAN），
                    //  与原来的行为一致。
                    zoom_on <= 1'b1;
                    refresh <= 1'b1;        // 补一帧让十字条出现
                    st      <= Z_HD;
                end else if ((act_code != 2'd0) && (zoom != Z_FIT)) begin
                    //----------------------------------------------------------
                    // ★★ [ZOOM-RESTORE] 缩放 >1.0 时按了"开菜单/翻页" ★★
                    //----------------------------------------------------------
                    //  【为什么必须回到 S_ZOOM】"动画恢复"要复用 565 预览 →
                    //    zk 从当前档插值回 1×，这条通路只有 S_ZOOM 有。
                    //  【动作怎么放行】先把哪个键记下来（pend_sel），等恢复 + 重装
                    //    完成、viewer_ctrl 回到静止态（allow=1）再发 act_go。
                    //    直接透传不行：那一刻还在 S_ZOOM/S_HDF，菜单/翻页会丢。
                    //  ★ 恢复要跑**一次完整的 565 预览动画**（zk 插值回 1×）★
                    //    同拍不能发 preview_req：viewer_ctrl 收它的语句在 S_ZOOM
                    //    分支里，这一拍它还在 S_HD ⇒ 请求会被丢掉。
                    //    所以先进 Z_HD 停一拍，等它真的进了 S_ZOOM 再发（见 Z_HD）。
                    //  ★ 卡死问题不在这里，而在"恢复期间按键仍然把防抖清零"——
                    //    已在下面的 db_cnt 逻辑里用 `!rest` 堵掉。
                    zoom_on     <= 1'b1;
                    zoom        <= Z_FIT;       // 目标 1× ⇒ viewer_ctrl 的 zk 往下走
                    mode        <= 2'd2;        // 恢复期间只允许 ZOOM
                    rest        <= 1'b1;
                    pend_v      <= 1'b1;
                    pend_sel    <= act_code;
                    st          <= Z_HD;        // 下一拍发 preview_req
                end else if (pend_v && allow) begin
                    // 恢复完成且已回到静止态 ⇒ 放行被暂存的那个键（单拍）
                    act_go  <= 1'b1;
                    act_sel <= pend_sel;
                    pend_v  <= 1'b0;
                end
            end

            //--------------------------------------------------------------
            //  888 停帧态
            //--------------------------------------------------------------
            Z_HD: begin
                if (rest) begin
                    // ★ 恢复会话：viewer_ctrl 这一拍已在 S_ZOOM ⇒ 现在发预览请求，
                    //   跑一次完整的 565 预览动画。
                    preview_req <= 1'b1;
                    st          <= Z_PRE;
                end else if (!hd_shown && (k3 || k4)) begin
                    // ★ [ZOOM-SEQ] 上一次缩放的 888 还没真正显示出来 ⇒ 这一拍不动。
                    //   原来会记进 nxt_v 等 hd_shown；现在这一步由 Z_LOAD 的
                    //   "立刻生效 + 重新起单"覆盖（真正被卡住的只有提交那 2 帧），
                    //   丢掉一下换掉 2 个 FF + 一组 mux，值。
                end else if (k1) begin
                    zoom_on <= 1'b0;
                    rest    <= 1'b0;
                    pend_v  <= 1'b0;
                    st      <= Z_OFF;
                end else if (k2) begin
                    // ★ [ZOOM-RESTORE] 缩放 >1.0 时**禁止**切到 X-PAN / Y-PAN：
                    //   锚点必须锁死（cam 公式的前提），而且 >1.0 时 pan_*_ok 恒 0，
                    //   切过去只会让 OSD 写着一个按不动的功能。
                    mode    <= (zoom != Z_FIT) ? 2'd2
                                               : ((mode == 2'd2) ? 2'd0 : (mode + 2'd1));
                    refresh <= 1'b1;        // 模式变了，十字条外观不变但也补一帧（便宜）
                end else if (k4) begin
                    if (pan_x_ok && (ax != X_LO)) begin
                        ax <= ax_m; refresh <= 1'b1;
                    end else if (pan_y_ok && (ay != Y_LO)) begin
                        ay <= ay_m; refresh <= 1'b1;
                    end else if (zoom_ok && (zoom != Z_FIT)) begin
                        zoom <= z_dn(zoom);
                        preview_req <= 1'b1;
                        st   <= Z_PRE;
                    end
                end else if (k3) begin
                    if (pan_x_ok && (ax != X_HI)) begin
                        ax <= ax_p; refresh <= 1'b1;
                    end else if (pan_y_ok && (ay != Y_HI)) begin
                        ay <= ay_p; refresh <= 1'b1;
                    end else if (zoom_ok && (zoom != 8'h80)) begin
                        zoom <= z_up(zoom);
                        preview_req <= 1'b1;
                        st   <= Z_PRE;
                    end
                end
            end

            //--------------------------------------------------------------
            //  565 预览态：继续收键，停手 500ms 才重装
            //--------------------------------------------------------------
            Z_PRE: begin
                if (k1) begin
                    zoom_on <= 1'b0;
                    rest    <= 1'b0;
                    pend_v  <= 1'b0;
                    st      <= Z_OFF;
                end else if (rest) begin
                    // ★★ 2026-11-02 [ZOOM-EXIT-FAST]：恢复会话不再踢烘焙、也不再等它 ★★
                    //   【原来】Z_HD(rest) 发 preview_req 后进 Z_PRE，Z_PRE 无条件
                    //     `load_req` 起一单 1× 烘焙，然后 Z_LOAD 一直等到 ld_done
                    //     （12MP 源 ~4s）才退出缩放 ⇒ 用户按了翻页/菜单，动画演完了
                    //     还要再等一整套高清装载，才轮到这个键生效。
                    //   【现在】恢复会话在 Z_PRE 就收工：只把 OSD 关掉（zoom_on=0），
                    //     **不**发 load_req。档位已经是 1×，viewer_ctrl 会让 zk 插值
                    //     跑完（动画照旧）后落到 S_LOW —— 那是个"按键全活"的状态，
                    //     1× 的高清在后台装，pend_v 暂存的动作当拍就通过 act_go 放行。
                    //   ⚠ 不能在这里发 load_req：S_LOW 自己会判断"要不要补一单"
                    //     （[ZOOM-EXIT-FAST] 里的 hd_want<=~hd_ready&~hd_pend），
                    //     两边都发就会重复烘焙，第二单会写正在显示的那块画布 = 撕裂。
                    zoom_on <= 1'b0;
                    rest    <= 1'b0;
                    st      <= Z_OFF;
                end else begin
                    if (k2) begin
                        // ★ [ZOOM-RESTORE] >1.0 时只允许 ZOOM
                        mode <= (zoom != Z_FIT) ? 2'd2
                                                : ((mode == 2'd2) ? 2'd0 : (mode + 2'd1));
                    end else if (k4) begin
                        if (pan_x_ok && (ax != X_LO))      ax <= ax_m;
                        else if (pan_y_ok && (ay != Y_LO)) ay <= ay_m;
                        else if (zoom_ok && (zoom != Z_FIT)) zoom <= z_dn(zoom);
                    end else if (k3) begin
                        if (pan_x_ok && (ax != X_HI))      ax <= ax_p;
                        else if (pan_y_ok && (ay != Y_HI)) ay <= ay_p;
                        else if (zoom_ok && (zoom != 8'h80)) zoom <= z_up(zoom);
                    end

                    // ★★ 2026-10-30：**去掉 500ms 防抖** ★★
                    //  原来要等"停手 500ms"才踢重装 ⇒ 动画早演完了还要干等，
                    //  手感像卡住。现在进 Z_PRE 立刻踢重装：
                    //    · 动画（zk 插值）在装载期间继续跑，装完正好切清晰图；
                    //    · 装载期间按键由 Z_LOAD **立即生效并重新起单**，不会丢；
                    //    · 装载期间按键不再参与任何计时（见上面 db_cnt 的说明）。
                    load_req <= 1'b1;
                    db_cnt   <= 27'd0;      // Z_LOAD 的超时从"发出请求"起算
                    st       <= Z_LOAD;
                end
            end

            //--------------------------------------------------------------
            //  装载中：KEY3/KEY4 立刻生效（重新起单），KEY1 退出
            //--------------------------------------------------------------
            Z_LOAD: begin
                // ★★ 2026-11-02 [ZOOM-RESTEP] ★★
                //  以前这里是"记一步、等 ld_done 再执行"⇒ 连按 N 下 = N 次整幅烘焙。
                //  现在直接改档位 + load_req：viewer_ctrl 会把飞着那次放弃掉，
                //  再用最新档位重装。db_cnt 顺带清零 —— 每重起一单就给一个新的
                //  LD_TO 窗口，否则连按时会被上一单的超时提前踢出去。
                if (!rest && step_ok) begin
                    zoom    <= k3 ? z_up(zoom) : z_dn(zoom);
                    load_req <= 1'b1;
                    db_cnt   <= 27'd0;
                end

                if (k1) begin
                    zoom_on <= 1'b0;
                    rest    <= 1'b0;
                    pend_v  <= 1'b0;
                    st      <= Z_OFF;
                end else if (ld_done && !step_ok && (out_n <= 2'd1)) begin
                    // out_n<=1 ⇒ 这次 done 把账清零：这一轮烘焙真的结束了。
                    // （被我们砍掉的那次也会报 done，它只把 out_n 减 1，不会走到这里。）
                    // ★ [ZOOM-RESTORE] 恢复会话装完就退出缩放（回到 1× 的画面），
                    //   被暂存的动作在外面的 Z_OFF 里等 allow 再放行。
                    if (rest) begin
                        zoom_on <= 1'b0;
                        rest    <= 1'b0;
                        st      <= Z_OFF;
                    end else st <= Z_HD;
                end else if (db_cnt == LD_TO) begin   // ★ [AREA] 饱和在 LD_TO ⇒ == 与 >= 等价
                    // ★ 超时兜底：装载失败/卡住时不能让状态机死等
                    out_n <= 2'd0;          // ★ [ZOOM-RESTEP] 账已经对不上了，清掉
                    if (rest) begin
                        zoom_on <= 1'b0;
                        rest    <= 1'b0;
                        st      <= Z_OFF;
                    end else st <= Z_HD;
                end
            end

            default: st <= Z_OFF;
        endcase
    end
end

endmodule
