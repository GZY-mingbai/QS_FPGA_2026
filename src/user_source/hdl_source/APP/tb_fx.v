//==============================================================================
//  tb_fx.v —— 过渡几何发生器 viewer_fx 的逐帧断言
//==============================================================================
//
//  【为什么需要它】
//  这几条不变式在板上"看不出来"，只能靠断言：
//    1. q=0   的几何 == 起始静止帧           （否则过渡一开始就跳一下）
//    2. q=T_END 的几何 == 目标静止帧         （否则过渡末尾"啪"地放大 —— 旧 fx3 的坑）
//    3. 两层并集盖满屏幕、且零缝隙零重叠       （否则中间漏黑缝/叠影）
//    4. 相邻帧位移有限、方向与 dir 一致        （否则看起来跳帧/卡顿/方向反了）
//    5. k*RW == 65536*160 ± RW              （否则纹理采样率错 ⇒ 图像缩放不对/出杂色）
//
//  【2026-10-26：dir 维度】
//    dir=1（next，"下一张"）⇒ 画面向左/向上滚；dir=0（prev）⇒ 全部镜像。
//    不变式 1/2/5 与 dir 无关；3/4 改写成"方向中性 + 按 dir 查运动方向"。
//    fx3/fx4 是缩小交换，中间帧故意露黑底，不查"并集盖满"，改查：
//      · 两层尺寸 ∈ [320×240, 640×480]（约束一）
//      · 中点（q=128）B 的落点：fx3 必须在屏幕正中心；fx4 必须偏置 80px
//      · 中点 A 必须已完全出屏，且出屏侧与 dir 匹配（next 出左、prev 出右）
//
//  【不依赖 q 的写法】
//  本 tb **不去重算 q**，只从 RTL 的输出检查几何关系 —— 这样连"q 算错了"
//  也能被抓到（比如 q 在 t=T_END 没到 256，端点断言立刻失败）。
//
//  跑法：
//    iverilog -g2005 -s tb_fx -o /tmp/tb_fx.vvp APP/tb_fx.v APP/viewer_fx.v
//    vvp /tmp/tb_fx.vvp
//
//==============================================================================

`timescale 1ns / 1ps

module tb_fx;

localparam integer T_END_F = 60;       // fx0/1/2：1.0s @60fps（必须和 App.v 一致）
localparam integer T_END_S = 180;      // fx3/fx4：3.0s @60fps（必须和 App.v 一致）
localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam [23:0]  NUM_X = 24'd10485760;   // 65536*160

reg clk = 0, rst = 1;
always #10 clk = ~clk;

reg  [2:0] fx_id = 0;
reg  [8:0] t     = 0;
reg        upd   = 0;
reg        dir   = 1;      // 1=next(左/上滚)  0=prev(右/下滚)

wire [31:0] pos_a, size_a, pos_b, size_b;
wire [39:0] k_a, k_b;
wire [7:0]  alpha_b;

viewer_fx #(
    .IMG_W(IMG_W), .IMG_H(IMG_H), .SRC_W(160), .SRC_H(240),
    .T_END_F(T_END_F), .T_END_S(T_END_S)
) dut (
    .clk(clk), .rst(rst),
    .fx_id(fx_id), .t(t), .upd(upd), .dir(dir),
    .pos_a(pos_a), .size_a(size_a), .k_a(k_a),
    .pos_b(pos_b), .size_b(size_b), .k_b(k_b),
    .alpha_b(alpha_b)
);

integer errors = 0;

// 有符号视图
wire signed [15:0] ax = pos_a[31:16];
wire signed [15:0] ay = pos_a[15:0];
wire        [15:0] aw = size_a[31:16];
wire        [15:0] ah = size_a[15:0];
wire signed [15:0] bx = pos_b[31:16];
wire signed [15:0] by = pos_b[15:0];
wire        [15:0] bw = size_b[31:16];
wire        [15:0] bh = size_b[15:0];

//------------------------------------------------------------------------------
//  推一帧：设 t / fx_id / dir，打一拍 upd，等除法算完
//------------------------------------------------------------------------------
task step;
    input [2:0] fx;
    input       dd;
    input integer tt;
    begin
        @(negedge clk);
        fx_id = fx;
        dir   = dd;
        t     = tt[8:0];
        upd   = 1'b1;
        @(negedge clk);
        upd   = 1'b0;
        repeat (150) @(posedge clk);     // 4 次除法 × 24 拍 = 96，留余量
    end
endtask

//------------------------------------------------------------------------------
//  k 检查：kx 必须等于 10485760/RW（±1）
//------------------------------------------------------------------------------
task chk_k;
    input integer fx;
    input integer tt;
    input [19:0] kx;
    input [15:0] rw;
    input [19:0] ky;
    input [15:0] rh;
    integer ex, ey;
    begin
        ex = NUM_X / rw;
        ey = 24'd15728640 / rh;
        if (kx > ex + 1 || kx + 1 < ex) begin
            $display("  [FAIL] fx%0d t=%0d KX=%0d 期望 %0d (RW=%0d)", fx, tt, kx, ex, rw);
            errors = errors + 1;
        end
        if (ky > ey + 1 || ky + 1 < ey) begin
            $display("  [FAIL] fx%0d t=%0d KY=%0d 期望 %0d (RH=%0d)", fx, tt, ky, ey, rh);
            errors = errors + 1;
        end
    end
endtask

//------------------------------------------------------------------------------
//  单步几何断言
//------------------------------------------------------------------------------
integer i, fx, last_ax;
integer lx, ly;      // 上一帧的 X0A / Y0A

task check_frame;
    input [2:0]   f;
    input         d;
    input integer tt;
    integer lim;      // 相邻帧位移上限。q 只有 0..256（Q8），qu 每帧跳 2 或 4
                      // ⇒ 步进是"块状"的：fx3/fx4 prev 的 A 左沿 90 帧移 640px，
                      // 实测峰值 15px/帧（avg 7.1 的 ~2.1 倍）。
                      // 取 1920/tend+12：快档 44、慢档 22 —— 既能抓住真正的故障
                      // （陈旧几何/半帧切换会是 640px 级跳变），又不会被量化步进误触。
    begin
        lim = (f >= 3'd3) ? (1920 / T_END_S) + 12 : (1920 / T_END_F) + 12;
        // ★ 2026-10-28 已否决的优化（留个记号，别再试）：
        //   「KY = 2×KX，省掉 KY_A/KY_B 两个除法通道」——
        //   前提是画框宽高比恒为 4:3，但实测不成立：
        //   g320=(eup*640)>>9 与 g240=(eup3*640)>>11 是两处独立取整，
        //   fx3 t=7 得到 638×480、t=10 得到 636×478，偏差 ±2px（≈0.4%）。
        //   强行 2× 会让画面纵向拉伸，5 秒动画里能看出来。此项作废。
        // ---- 3. 接缝 / 覆盖（fx3、fx4 除外：缩小交换故意露黑底）----
        //   fx1 是**竖向**平移：A 从下沿滑出、B 从下方进入，接缝是横向的
        //   fx0 是**横向**平移：接缝是纵向的
        //   方向中性的判据（next/prev 都成立）：
        //     ① 并集盖满屏幕：min(A左,B左) ≤ 0 且 max(A右,B右) ≥ 屏幕宽/高
        //     ② 零缝隙零重叠：A右沿 == B左沿  或  B右沿 == A左沿
        // ---- 3. 接缝 / 覆盖（fx3、fx4 除外：缩小交换故意露黑底）----
        //   fx1 是**竖向**平移：A 从下沿滑出、B 从下方进入，接缝是横向的
        //   fx0 是**横向**平移：接缝是纵向的
        //   fx2 是淡入淡出：两层都满屏重叠（B 半透明盖 A）⇒ 只查覆盖，不查接缝
        //   方向中性的判据（next/prev 都成立）：
        //     ① 并集盖满屏幕：min(A左,B左) ≤ 0 且 max(A右,B右) ≥ 屏幕宽/高
        //     ② 零缝隙零重叠：A右沿 == B左沿  或  B右沿 == A左沿
        if (f != 3'd3 && f != 3'd4) begin
            if (f == 3'd1) begin
                if (!((ay <= 0 || by <= 0) && (ay + ah >= IMG_H || by + bh >= IMG_H))) begin
                    $display("  [FAIL] fx%0d t=%0d 竖向漏缝: A[%0d,%0d] B[%0d,%0d]",
                             f, tt, ay, ay + ah, by, by + bh);
                    errors = errors + 1;
                end
                if (!(ay + ah == by || by + bh == ay)) begin
                    $display("  [FAIL] fx%0d t=%0d 竖向接缝不精确: A[%0d,%0d] B[%0d,%0d]",
                             f, tt, ay, ay + ah, by, by + bh);
                    errors = errors + 1;
                end
            end else if (f == 3'd0) begin
                if (!((ax <= 0 || bx <= 0) && (ax + aw >= IMG_W || bx + bw >= IMG_W))) begin
                    $display("  [FAIL] fx%0d t=%0d 横向漏缝: A[%0d,%0d] B[%0d,%0d]",
                             f, tt, ax, ax + aw, bx, bx + bw);
                    errors = errors + 1;
                end
                if (!(ax + aw == bx || bx + bw == ax)) begin
                    $display("  [FAIL] fx%0d t=%0d 横向接缝不精确: A[%0d,%0d] B[%0d,%0d]",
                             f, tt, ax, ax + aw, bx, bx + bw);
                    errors = errors + 1;
                end
            end else begin
                // fx2：两层都满屏 ⇒ 覆盖必然成立，查一下尺寸就够（见第 4 组）
                if (ax != 0 || ay != 0 || bx != 0 || by != 0) begin
                    $display("  [FAIL] fx2 t=%0d 两层不在原点: A(%0d,%0d) B(%0d,%0d)",
                             tt, ax, ay, bx, by);
                    errors = errors + 1;
                end
            end
        end else begin
            // ---- fx3/fx4 专用：两层尺寸必须在允许范围内 ----
            //   B：两种效果都 ≥ 320×240、≤ 640×480
            //   A：fx3 ∈ [320×240, 640×480]；
            //      fx4 缩放档 80% 且后半段不再缩（用户要求"左图最终缩放 ==
            //      右图初始缩放 == 80%"）⇒ A ∈ [512×384, 640×480]
            //      （全程 KX/KY 都 < 65536，不会触发"整字抽稀"）
            if (f == 3'd3) begin
                if (aw < 320 || aw > IMG_W || ah < 240 || ah > IMG_H) begin
                    $display("  [FAIL] fx3 t=%0d A 尺寸越界: %0dx%0d（要求 320..%0d x 240..%0d）",
                             tt, aw, ah, IMG_W, IMG_H);
                    errors = errors + 1;
                end
            end else begin
                if (aw < 512 || aw > IMG_W || ah < 384 || ah > IMG_H) begin
                    $display("  [FAIL] fx4 t=%0d A 尺寸越界: %0dx%0d（要求 512..%0d x 384..%0d）",
                             tt, aw, ah, IMG_W, IMG_H);
                    errors = errors + 1;
                end
            end
            if (bw < 320 || bw > IMG_W || bh < 240 || bh > IMG_H) begin
                $display("  [FAIL] fx%0d t=%0d B 尺寸越界: %0dx%0d（要求 320..%0d x 240..%0d）",
                         f, tt, bw, bh, IMG_W, IMG_H);
                errors = errors + 1;
            end
            // ---- fx3/fx4 专用：中点（q_s=128 ⇒ t=T_END_S/2）断言 ----
            if (tt == T_END_S/2) begin
                if (f == 3'd3) begin
                    // fx3：B 必须在屏幕正中心，A 已完全出屏
                    if (bx != (IMG_W-320)/2 || by != (IMG_H-240)/2 || bw != 320 || bh != 240) begin
                        $display("  [FAIL] fx3 中点 B 不是 320x240@(160,120): (%0d,%0d) %0dx%0d",
                                 bx, by, bw, bh);
                        errors = errors + 1;
                    end else $display("  [ ok ] t=T_END/2  fx3 B = 320x240 @ (160,120)（图2 到达屏幕中心）");
                    // fx3：A 必须已完全出屏，出屏侧与 dir 匹配（next 出左 / prev 出右）
                    // ★ 必须显式 $signed：ax 是有符号、aw 是无符号，混用会把整个
                    //   表达式按无符号算（-320+320 变成 65024）⇒ 永远误判。
                    if (d ? (($signed(ax) + $signed({1'b0, aw})) > 0) : ($signed(ax) < IMG_W)) begin
                        $display("  [FAIL] fx3 中点 A 未完全出屏: X0=%0d W=%0d（dir=%0d）",
                                 ax, aw, d);
                        errors = errors + 1;
                    end
                end else begin
                    // fx4 v5（80% 档 + 移门式交接）：A、B 都是 512×384，
                    //   接缝恒等相接（零缝隙零重叠），并集盖满屏幕
                    //   next：A@(-256,48) 管左 256px、B@(256,48) 管右 384px
                    //   prev：镜像 —— A@(384,48) 管右、B@(-128,48) 管左（A 左沿 ≡ B 右沿 ≡ 384）
                    if (aw != 512 || ah != 384 || bw != 512 || bh != 384) begin
                        $display("  [FAIL] fx4 中点两层不是 512x384: A %0dx%0d B %0dx%0d",
                                 aw, ah, bw, bh);
                        errors = errors + 1;
                    end else if (d ? (ax != -256 || bx != 256) : (ax != 384 || bx != -128)) begin
                        $display("  [FAIL] fx4 中点两层位置不对: A(%0d,%0d) B(%0d,%0d)",
                                 ax, ay, bx, by);
                        errors = errors + 1;
                    end else $display("  [ ok ] t=T_END/2  fx4 A/B 都是 512x384 移门式相接铺满");
                    // fx4 中点：接缝必须恒等（next: B 左沿 == A 右沿；prev: A 左沿 == B 右沿）
                    if (!(d ? (bx == ($signed(ax) + $signed({1'b0, aw})))
                            : (ax == ($signed(bx) + $signed({1'b0, bw}))))) begin
                        $display("  [FAIL] fx4 中点接缝不恒等: A[%0d,%0d] B[%0d,%0d]",
                                 ax, $signed(ax) + $signed({1'b0, aw}), bx,
                                 $signed(bx) + $signed({1'b0, bw}));
                        errors = errors + 1;
                    end
                    if (!(($signed(ax) <= 0 || bx <= 0) &&
                          (($signed(ax) + $signed({1'b0, aw})) >= IMG_W ||
                           ($signed(bx) + $signed({1'b0, bw})) >= IMG_W))) begin
                        $display("  [FAIL] fx4 中点并集未盖满屏幕: A[%0d,%0d] B[%0d,%0d]",
                                 ax, $signed(ax) + $signed({1'b0, aw}), bx,
                                 $signed(bx) + $signed({1'b0, bw}));
                        errors = errors + 1;
                    end
                end
            end
        end
        // ---- 4. A 永远满屏尺寸；B 必须满高、不超宽（fx3/fx4 除外）----
        //   A 一旦比屏幕小/大就会露出黑底或超框；
        //   B 比屏幕矮的话，在 A 已经滑走的那一侧会露出黑底（旧 fx3 踩过）。
        //   fx3/fx4 故意让两层缩到 50%，这两条不适用（见上面 fx3/fx4 专用段）。
        if (f != 3'd3 && f != 3'd4) begin
            if (aw != IMG_W || ah != IMG_H) begin
                $display("  [FAIL] fx%0d t=%0d A 不是满屏尺寸: %0dx%0d", f, tt, aw, ah);
                errors = errors + 1;
            end
            if (bh != IMG_H || bw > IMG_W) begin
                $display("  [FAIL] fx%0d t=%0d B 尺寸不合规: %0dx%0d（要求高=%0d、宽<=%0d）",
                         f, tt, bw, bh, IMG_H, IMG_W);
                errors = errors + 1;
            end
        end
        // ---- 尺寸合理 ----
        if (aw == 0 || ah == 0 || bw == 0 || bh == 0) begin
            $display("  [FAIL] fx%0d t=%0d 尺寸为 0", f, tt);
            errors = errors + 1;
        end
        // ---- ★ 约束一：画框面积绝不能超过 640×480 ----
        //   ps_gen_image 的 rq_can_adv 里 rq_idx 上限是框内位置数，
        //   一超就会出现"请求侧停走 ⇒ 帧收不满 ⇒ 卡在 S_TRANS"（旧 fx3 踩过）。
        if (aw * ah > IMG_W * IMG_H) begin
            $display("  [FAIL] fx%0d t=%0d A 画框超屏: %0dx%0d = %0d > %0d",
                     f, tt, aw, ah, aw * ah, IMG_W * IMG_H);
            errors = errors + 1;
        end
        if (bw * bh > IMG_W * IMG_H) begin
            $display("  [FAIL] fx%0d t=%0d B 画框超屏: %0dx%0d = %0d > %0d",
                     f, tt, bw, bh, bw * bh, IMG_W * IMG_H);
            errors = errors + 1;
        end

        // ---- 6. k 与尺寸一致 ----
        chk_k(f, tt, k_a[39:20], aw, k_a[19:0], ah);
        chk_k(f, tt, k_b[39:20], bw, k_b[19:0], bh);
        // ---- 5. 相邻帧位移有限 + 运动方向与 dir 一致 ----
        //   峰值速度 ≈ 1.5×平均（ease-in-out）；上限按最坏情况 640px/(tend/2) 帧取
        if (tt > 0) begin
            if ((lx - ax > lim) || (ax - lx > lim)) begin
                $display("  [FAIL] fx%0d t=%0d X0A 跳变: %0d -> %0d", f, tt, lx, ax);
                errors = errors + 1;
            end
            if ((ly - ay > lim) || (ay - ly > lim)) begin
                $display("  [FAIL] fx%0d t=%0d Y0A 跳变: %0d -> %0d", f, tt, ly, ay);
                errors = errors + 1;
            end
            // 运动轴的方向必须与 dir 一致：
            //   fx0/fx3/fx4 水平运动：next(dir=1) 只许左移(ax 递减)，prev 只许右移
            //   fx1 竖直运动：        next 只许上移(ay 递减)，prev 只许下移
            if (f == 3'd1) begin
                if (d && (ay > ly)) begin
                    $display("  [FAIL] fx1 t=%0d Y0A 应上移却下移: %0d -> %0d", tt, ly, ay);
                    errors = errors + 1;
                end
                if (!d && (ay < ly)) begin
                    $display("  [FAIL] fx1 t=%0d Y0A 应下移却上移: %0d -> %0d", tt, ly, ay);
                    errors = errors + 1;
                end
            end else if (f != 3'd2) begin
                if (d && (ax > lx)) begin
                    $display("  [FAIL] fx%0d t=%0d X0A 应左移却右移: %0d -> %0d", f, tt, lx, ax);
                    errors = errors + 1;
                end
                if (!d && (ax < lx)) begin
                    $display("  [FAIL] fx%0d t=%0d X0A 应右移却左移: %0d -> %0d", f, tt, lx, ax);
                    errors = errors + 1;
                end
            end
        end
        lx = ax;
        ly = ay;
    end
endtask

//------------------------------------------------------------------------------
//  单种效果整段扫一遍（指定 dir）
//------------------------------------------------------------------------------
task sweep;
    input [2:0] f;
    input       d;
    integer tt;
    integer tend;      // 本效果的总帧数：fx3/fx4 用慢档，其余用快档
    integer lim;       // 相邻帧位移上限（随档位变）
    begin
        tend = (f >= 3'd3) ? T_END_S : T_END_F;
        lim  = (1440 / tend) + 2;
        $display("\n== fx%0d dir=%0d (%s) tend=%0d ==", f, d,
                 d ? "next 左/上滚" : "prev 右/下滚", tend);
        lx = 0; ly = 0;

        // ---- 起点 ----
        step(f, d, 0);
        if (!(ax == 0 && ay == 0 && aw == IMG_W && ah == IMG_H)) begin
            $display("  [FAIL] fx%0d t=0 的 A 不是满屏原点: (%0d,%0d) %0dx%0d",
                     f, ax, ay, aw, ah);
            errors = errors + 1;
        end else $display("  [ ok ] t=0  A = 640x480 @ (0,0)");

        // ---- 终点 ----
        step(f, d, tend);
        if (!(bx == 0 && by == 0 && bw == IMG_W && bh == IMG_H)) begin
            $display("  [FAIL] fx%0d t=%0d 的 B 不是满屏原点: (%0d,%0d) %0dx%0d",
                     f, tend, bx, by, bw, bh);
            errors = errors + 1;
        end else $display("  [ ok ] t=%0d B = 640x480 @ (0,0)（与静止帧无缝）", tend);
        if (f == 3'd2 && alpha_b !== 8'd255) begin
            $display("  [FAIL] fx2 末帧 alpha=%0d，应为 255", alpha_b);
            errors = errors + 1;
        end

        // ---- 全程逐帧 ----
        lx = 0; ly = 0;
        for (tt = 0; tt <= tend; tt = tt + 1) begin
            step(f, d, tt);
            check_frame(f, d, tt);
        end

        // ---- alpha 单调（仅 fx2）----
        if (f == 3'd2) begin
            step(f, d, 0);
            if (alpha_b !== 8'd0) begin
                $display("  [FAIL] fx2 t=0 alpha=%0d，应为 0", alpha_b);
                errors = errors + 1;
            end
            step(f, d, tend / 2);
            if (alpha_b < 8'd100 || alpha_b > 8'd156) begin
                $display("  [FAIL] fx2 中点 alpha=%0d，应约 128", alpha_b);
                errors = errors + 1;
            end else $display("  [ ok ] fx2 中点 alpha=%0d", alpha_b);
        end
        $display("  [ ok ] fx%0d dir=%0d 全程 %0d 帧，接缝/覆盖/单调/k 全部通过", f, d, tend + 1);
    end
endtask

//------------------------------------------------------------------------------
//  主流程：dir=1(next) 与 dir=0(prev) 各扫一遍 fx0..fx4
//------------------------------------------------------------------------------
initial begin
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (10) @(posedge clk);

    sweep(3'd0, 1'b1);
    sweep(3'd1, 1'b1);
    sweep(3'd2, 1'b1);
    sweep(3'd3, 1'b1);
    sweep(3'd4, 1'b1);

    sweep(3'd0, 1'b0);
    sweep(3'd1, 1'b0);
    sweep(3'd2, 1'b0);
    sweep(3'd3, 1'b0);
    sweep(3'd4, 1'b0);

    $display("\n==== errors = %0d ====", errors);
    if (errors == 0) $display("PASS");
    else             $display("FAIL");
    $finish;
end

initial begin
    #2_000_000_000;
    $display("TIMEOUT");
    $finish;
end

endmodule
