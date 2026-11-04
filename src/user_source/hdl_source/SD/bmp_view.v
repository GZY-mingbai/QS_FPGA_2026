//==============================================================================
//  bmp_view.v —— 源像素矩形窗 → 固定画布（绕点缩放 + 黑边 + 888/565 打包）
//==============================================================================
//
//  【它替掉谁】`bmp_scale.v`（1130 LUT）。旧版把画布写死 640×480 / 320×240，
//  几何用"宽优先/高优先"两套分支 + dx0/dy0 + inb/d1/d2 三级黑边掩码 + 逐源像素
//  的 run 循环。新版只保留**一个 step**：
//
//      step_fit = max( (W<<10)/640 , (H<<10)/480 ) << mode     // 整图可见（fit）
//      step     = (step_fit << 4) / zoom                       // zoom=0x10 ⇒ step_fit
//                                                              // 0x04..0xFF = 0.25×..16×
//      cam      = W/2 + (ax − 320)·(step_fit − step)            // ★ 锚点反推（Q12.10）
//      sx0      = cam − (CW/2)*step
//      sy0      = cam − (CH/2)*step
//      输出列 ox ← 源列 floor(sx0 + ox*step)   （Q6.10，越界写黑）
//      输出行 oy ← 源行 floor(sy0 + oy*step)   （Q6.10，越界整行黑）
//
//  ★ 2026-10-30：输入从「绝对 cam」改成「屏幕锚点 (ax,ay)」。
//    锚点 = 屏幕上那个"缩放时不动的点"（滚轮语义：指针下的像素不动）。
//    调用方不用维护任何 cam 状态 —— cam 由 (ax, zoom) 直接算出，因为锚点固定
//    时 cam 对 step 是线性的，多档累积与一次直算完全等价（计划 §3 有验算）。
//
//  ⇒ 等比 / 居中 / 不裁剪 / 放大后自动裁，全部由这一组数表达。
//
//  【数据流】源是流、输出是光栅，所以：
//      一个源行 → 横向重采样一次（bmp_view_fill）→ 行缓冲 CW×24bit
//               → 纵向重复 R 次（R = 映射到该源行的输出行数）
//    缩小：同一源行被复用 R 次（**每行只读一次卡**）；放大：同一源像素写多列。
//
//  【行请求协议（给 loader）】
//      row_req : 要的源行（图像坐标；随 oy 单调递减 = 文件顺序递增）
//      win_x0  : 该行起始源列（自动裁剪左边界）
//      win_w   : 该行要发的像素数
//    loader 负责"跳到 row_req、只发 [win_x0, win_x0+win_w) 这段像素"。
//    本模块按 win_w 自己计数判断收完，不需要额外的行结束信号。
//
//  【三条不变量】
//    ① `dst_base .. dst_base + wpr*CH - 1` 每个字写一次（黑边写 0）
//       ⇒ loader 可以删掉"装载前预清洋红"。
//    ② 行序：888 正序、565 反序 —— 与 bmp_scale 一致，显示侧不用改。
//    ③ 888 内容字 = `{8'hFF,R,G,B}`、黑边字 = `32'd0`；565 偶列=低半、奇列=高半。
//
//  【stall】冻结**整条**冲刷延迟线（地址、数据、列号、命中位、BRAM 输出寄存器）。
//    只冻前半段会"塌一格"⇒ 每行漏一列（pitfall #23）。
//==============================================================================
module bmp_view (
    input  wire        clk,
    input  wire        rst,

    input  wire        start,
    input  wire [15:0] src_w,
    input  wire [15:0] src_h,
    input  wire [1:0]  mode,            // 0=640×480  1=320×240  2=160×120
    input  wire        fmt,             // 1=888(1px/字)  0=565(2px/字)
    input  wire [20:0] dst_base,
    input  wire [10:0] dst_stride,      // 0 = 自动（= 每行字数）
    input  wire [7:0]  zoom,            // Q4.4，0x10 = 1.0（fit）
    // ★ 2026-10-30：几何输入从「绝对 cam」改成「屏幕锚点 (ax,ay)」。
    //   缩放语义是"鼠标指针下的像素不动"，锚点就是屏幕上的那个点（= crx/cry）。
    //   内部按下面这条推出视口中心：
    //       cam = W/2 + (ax − 320)·(step_fit − step)        // 全 Q12.10
    //   锚点固定时 cam 对 step 是线性的 ⇒ 多档缩放累积与一次直算等价
    //   （见 plans/swift-pulse-tesla-VV5Q3p5S.md §3 的验算），
    //   所以**不需要跨装载保存 cam**，也就没有"上一次缩放"的状态。
    input  wire [9:0]  ax, ay,          // 缩放锚点（屏幕坐标，0..639 / 0..479）

    input  wire        pix_vld,
    input  wire [23:0] pix,
    output wire        pix_rd,

    output reg  [11:0] row_req,
    output reg  [11:0] win_x0,
    output reg  [12:0] win_w,
    output reg         row_go,          // 单拍：row_req 刚刚更新（loader 用它锁存目标行）
    // ★ 2026-10-30：连续流许可。整幅 1:1 时（step==1024、窗口就是整行），
    //   BMP 文件里行与行**本来就是连续的**（stride == win_w*3），loader 完全
    //   可以不停顿地读下去，不必每行重新 seek。
    //   上板实测：每行 seek 会让 SD 卡多付一次 CMD17 的访问延迟（真卡 ~0.8ms），
    //   而一张 640×480 有 480 行 ⇒ 白扔 0.4s 以上，且每行还会重复读同一个扇区。
    output wire        cont1,

    input  wire        stall,

    output reg         we,
    output reg  [20:0] wa,
    output reg  [31:0] wdata,

    output reg         busy,
    output reg         done
);

//------------------------------------------------------------------------------
//  常量 / 状态
//------------------------------------------------------------------------------
localparam [3:0] V_IDLE = 4'd0, V_GEO = 4'd1, V_DIV = 4'd2, V_CALC = 4'd3,
                 V_ADDR = 4'd4, V_NEXT = 4'd5, V_FILL = 4'd6, V_EMIT = 4'd7;
// ★ 2026-10-30：新增 V_CAM（取 8，不重排上面已有的编码）
//   【为什么必须拆出来】锚点→cam 是一次 10×16 乘法。写成组合后，综合报告
//   实测 LUT 从 17338 涨到 18149（+811）、WNS 从 −4.6ns 恶化到 −11.6ns ——
//   一条乘法链直接把 50MHz 域打穿。这里改成 10 拍串行移位加，
//   装载一次 0.4s，多 200ns 完全无所谓。
localparam [3:0] V_CAM  = 4'd8;

reg [3:0] st;

reg [1:0]  mode_r;
reg        fmt_r;
reg [15:0] sw_r, sh_r;
reg [9:0]  ax_r, ay_r;      // ★ 2026-10-30：屏幕锚点（原 camx_r/camy_r 的 22 位绝对坐标）
reg [20:0] base_r;
reg [10:0] stride_r;

// 画布尺寸（mode_r 常数展开，不占寄存器；TD 折叠成常量）
wire [10:0] cw  = (mode_r == 2'd0) ? 11'd640 : (mode_r == 2'd1) ? 11'd320 : 11'd160;
wire [10:0] ch  = (mode_r == 2'd0) ? 11'd480 : (mode_r == 2'd1) ? 11'd240 : 11'd120;
// ★ 2026-10-29：这里原来写 `fmt_r ? cw[9:0] : ...`，10 位装不下 640！
//   640 = 10_1000_0000（11 位），取 [9:0] 变成 128 ⇒ 888 每行只推 128 个字、
//   行指针每行只前进 128 ⇒ 高清整幅被压成 128 宽的斜条纹，表现就是"HD 888 加载不出来"。
//   565 的 320 刚好在 10 位内，所以只有 888 出问题。
wire [10:0] wpr = fmt_r ? cw : (cw >> 1);           // 每行字数（888=640 / 565=320）
// start 那一拍 mode_r/fmt_r 还没更新，用端口值另算一份
wire [10:0] cw_i  = (mode == 2'd0) ? 11'd640 : (mode == 2'd1) ? 11'd320 : 11'd160;
wire [10:0] wpr_i = fmt ? cw_i : (cw_i >> 1);
wire [10:0] ch_i  = (mode == 2'd0) ? 11'd480 : (mode == 2'd1) ? 11'd240 : 11'd120;

//------------------------------------------------------------------------------
//  几何寄存器（start 后算一次）
//------------------------------------------------------------------------------
// ★ 17 位：支持 zoom<1（缩小）后 step = q<<mode 会超过 65535
//   （例：4096 宽源、mode2、zoom=0.25 ⇒ q=26212、step=104848）
reg [16:0] step;            // Q6.10 源像素 / 输出像素
reg signed [26:0] sx0, sy0; // Q12.10 视口左上角对应的源坐标
reg signed [26:0] y_acc;    // 当前输出行的源 y（Q10，每行 -step）
reg [11:0] x_le, x_ri;
reg [10:0] oy;
reg [20:0] row_ptr;
reg [11:0] last_row;        // 行缓冲里装的是哪个源行
reg        buf_ok;          // 行缓冲里有有效内容

// ★ [C1] 4 根 26 位"按 mode 变速移位"合成 1 根 16 位移位
//   旧写法 p_cw = 5*step << (7-mode_r) 等都是 26 位桶形移位器（两级 2:1 mux
//   ≈ 52 LUT/根，四根 ≈ 208 LUT）。但 CANVAS 是 4:3 等比缩小的：
//        CW*step = (640>>mode) * (q<<mode) = 640*q     ← mode 自动抵消
//   所以只要拿"未按档位移位的商 q（= step>>mode）"去乘，四个变速移位全消失，
//   只剩 step = q<<mode 这一个 16 位移位。数值与原来逐位相同。
reg  [15:0] q_step;         // q = step_fit*16/zoom，与画布档无关
wire [19:0] q5  = {4'd0, q_step} + ({4'd0, q_step} << 2);       // 5*q
wire [19:0] q15 = ({4'd0, q_step} << 4) - {4'd0, q_step};       // 15*q
wire [25:0] p_cw  = {6'd0, q5}  << 7;       // 640*q（定值移位 = 纯连线）
wire [25:0] p_ch  = {6'd0, q15} << 5;       // 480*q
wire [25:0] p_cw2 = p_cw >> 1;              // CW/2 = CW>>1（定值）
wire [25:0] p_ch2 = p_ch >> 1;
// step 本拍的值（寄存器 step 要到下一拍才更新，算 y_acc 必须用这一个）
wire [16:0] step_n = {1'b0, q_step} << mode_r;
// ★ 提前声明（原在除法器那段）：锚点→cam 的组合块要用它，iverilog 要求先声明后使用
reg [15:0] step_fit;        // max(W/CW, H/CH) 的 Q10（未按档位移位）

//------------------------------------------------------------------------------
//  视口中心：由屏幕锚点反推（Q12.10）—— V_CAM 里 10 拍串行乘
//------------------------------------------------------------------------------
//  映射关系：屏幕列 ox 对应源列 sx0 + ox·step，而 ox=320 处恒等于 cam。
//  ⇒ 要让屏幕 ax 处那个源点保持不动，需要
//        cam = W/2 + (ax − 320)·(step_fit − step)
//  锚点固定时 cam 对 step 线性 ⇒ 多档累积 == 一次直算，无需保存历史。
//
//  ★ 不要写成 `ax_off * dstep` 组合乘：实测 LUT +811、WNS −4.6 → −11.6ns。
//    这里拆成 V_CAM 的 10 拍移位加，累加器和移位链都留在寄存器后面。
reg  [4:0]  cm_i;                   // 0..9（ax 只有 10 位）
reg         cm_sx, cm_sy;           // ax_off / ay_off 的符号
reg  [13:0] cm_dx, cm_dy;           // |ax_off| / |ay_off|
reg  [24:0] cm_shx;                 // 当前位移后的 step（每拍左移一位；x/y 同值，共用一份）
reg  [24:0] cm_ax, cm_ay;           // 累加结果
// W/2 << 10：sw_r/sh_r 在整个 V_CAM 期间不变，直接接线，不另存寄存器
wire [24:0] cm_base_x = {4'd0, sw_r[11:1], 10'd0};
wire [24:0] cm_base_y = {4'd0, sh_r[11:1], 10'd0};
reg signed [24:0] camx_q_r, camy_q_r;   // 串行乘完的 cam（Q12.10）

wire [10:0] ax_off = $signed({1'b0, ax_r}) - 11'sd320;
wire [10:0] ay_off = $signed({1'b0, ay_r}) - 11'sd240;
// ★ 2026-10-30：这里必须用**未按档位移位的 q_step**，不能用 step_n(=q<<mode)。
//   推导：屏幕列 ax 处的源像素 = (cam + (ax-320)·q)/1024 —— mode 因
//   `ox = ax/2^mode` 与 `step = q·2^mode` 相乘而自动抵消。所以保持锚点不动要
//       cam = (W/2)<<10 + (ax-320)·(step_fit - q)
//   写成 step_n 后，mode1/mode2 的锚点偏移会被放大 2×/4×（实测 mode1 整幅偏
//   80 列、76800 个像素全错）。zoom<1 时 q>step_fit ⇒ 这里钳 0（保持居中）。
wire [13:0] dstep  = (step_fit >= q_step) ? (step_fit[13:0] - q_step[13:0]) : 14'd0;

wire signed [26:0] sx0_c  = $signed(camx_q_r) - $signed({1'b0, p_cw2});
wire signed [26:0] sy0_c  = $signed(camy_q_r) - $signed({1'b0, p_ch2});
wire signed [26:0] sx1_c  = sx0_c + $signed({1'b0, p_cw});    // 复用 sx0_c，省一台 27 位减法
wire signed [26:0] sx1_r  = sx1_c + 27'sd1023;               // 上取整用，只算一次

// 列裁剪窗口（组合算好，V_CALC 一次锁存；win_w 必须用它们算，不能用还没更新的寄存器）
wire [11:0] x_le_c = (sx0_c < 0)                          ? 12'd0 :
                     ((sx0_c >>> 10) >= {1'd0, 11'd0} + {4'd0, sw_r}) ? (sw_r[11:0] - 12'd1) :
                     sx0_c[21:10];
wire [11:0] x_ri_c = (sx1_c <= 0)                         ? 12'd1 :
                     ((sx1_r >>> 10) >= {4'd0, sw_r}) ? sw_r[11:0] :
                     (sx1_r >>> 10);

//------------------------------------------------------------------------------
//  串行除法器（24 拍/次，三次共用：/640、/480、/zoom）
//------------------------------------------------------------------------------
reg [23:0] dv_a;
reg [12:0] dv_b;
reg [23:0] dv_q;
reg [12:0] dv_r;
reg [5:0]  dv_i;
reg [1:0]  dv_sel;
// （step_fit 已在几何块前提前声明）

// ★ [C2] 4:3 恒定 ⇒ W/640 > H/480 ⟺ 3W > 4H。先比大小，只做一次除法，
//   原来"W/640、H/480 各除一次再取 max"白花了一整趟（24 拍 + 一套商/余数寄存）。
wire [15:0] w3 = {14'd0, src_w[11:0]} + {13'd0, src_w[11:0], 1'b0};
wire [15:0] h4 = {12'd0, src_h[11:0], 2'b00};
wire [22:0] w_q10 = src_w[12:0] << 10;
wire [22:0] h_q10 = src_h[12:0] << 10;

wire dv_done = (dv_i == 6'd24);

// 折半（24 拍移位减；r 最大 2b-1 ≤ 8191 ⇒ 13 位够）
wire [12:0] dv_r_sh = {dv_r[11:0], dv_a[23]};

//------------------------------------------------------------------------------
//  行缓冲填（子模块）
//------------------------------------------------------------------------------
// 精确 2:1 才做平均：565 + step 正好 2048 + **视口左沿就在源图第 0 列**。
// ★ 2026-10-30：这里原来只判 `sx0[10:0]==0`（= 1024 对齐），于是 240×480 这种
//   "比画布窄"的源（sx0 = -200×1024）也满足，走了 avg2 分支 —— 而 avg2 分支
//   从 ox_w=0 开始写、完全不看 sx_acc/col_valid ⇒ 整幅贴左边、右侧全黑。
//   正确的判据是"窗口正好等于整幅图"，即 sx0 恰为 0。
wire        avg2_on = (!fmt_r) && (step == 17'd2048) && (sx0 == 27'sd0);

reg         fill_start;
wire        fill_busy, fill_done;
wire [10:0] f_ox_lo, f_ox_hi;
wire        f_lb_we;
wire [10:0] f_lb_wad;
wire [23:0] f_lb_din;

// ★ 行缓冲（CW × 24bit = 15,360bit）：写口在 u_fill，读口在本模块。
//   两个 always 各管一个口 ⇒ 标准简单双口块 RAM（不要把 ? : 放进读口，
//   那会让 TD 退回分布式 RAM，sd_bmp_loader 的 bf_fifo 踩过）。
// 行缓冲本体在 u_fill 里（写读同模块，TD 才会推成块 RAM）

reg  [10:0] ox;                 // 冲刷列号（读口地址源）
wire        fl_hold = (st == V_EMIT) && stall;
wire [10:0] raddr   = (ox >= cw) ? (cw - 11'd1) : ox;
wire [23:0] lb_q;
bmp_view_fill u_fill (
    .clk     (clk),
    .rst     (rst),
    .start   (fill_start),
    .sx0     (sx0),
    .step    (step),
    .x_le    (x_le),
    .win_w   (win_w),
    .cw      (cw),
    .avg2    (avg2_on),
    .pix_vld (pix_vld),
    .pix     (pix),
    .pix_rd  (pix_rd),
    .ox_lo   (f_ox_lo),
    .ox_hi   (f_ox_hi),
    .busy    (fill_busy),
    .done    (fill_done),
    .lb_we   (f_lb_we),
    .lb_wad  (f_lb_wad),
    .lb_din  (f_lb_din),
    .rd_addr (raddr),
    .rd_en   (!fl_hold),
    .rd_q    (lb_q)
);

reg [10:0] ox_lo, ox_hi;

//------------------------------------------------------------------------------
//  冲刷流水（地址 → BRAM → 写；两级延迟）
//------------------------------------------------------------------------------
reg [10:0] cd1, cd2;        // 两级：cd2 与 lb_q 同拍（比 ox 晚 2 拍）
reg [15:0] wd_lo;

// 命中位由列号直接译码（不再多打一拍，否则会比 lb_q 晚一拍 ⇒ 第一列写 0）
wire inb = (cd2 >= ox_lo) && (cd2 < ox_hi);
// ★ [C3] 黑边只在 24bit 像素上打一次零，再拼成 888/565：
//   原来三处各打一次（wdata 32 位 + wd_lo 16 位 + 高半字 16 位 = 64 个与门），
//   现在一处 24 位（24 个与门），输出逐位相同。
wire [23:0] rgb_g = inb ? lb_q : 24'd0;

wire        row_ok  = (y_acc >= 0) && ((y_acc >>> 10) < {15'd0, sh_r});
wire [11:0] src_y   = (y_acc < 0) ? 12'd0 : (y_acc >>> 10);


function [15:0] rgb565;
    input [23:0] c;
    begin
        rgb565 = {c[23:19], c[15:10], c[7:3]};
    end
endfunction

//------------------------------------------------------------------------------
//  主状态机
//------------------------------------------------------------------------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        st <= V_IDLE; busy <= 1'b0; done <= 1'b0;
        mode_r <= 2'd0; fmt_r <= 1'b0; sw_r <= 16'd0; sh_r <= 16'd0;
        ax_r <= 10'd320; ay_r <= 10'd240; base_r <= 21'd0; stride_r <= 11'd0;
        cm_i <= 5'd0; cm_sx <= 1'b0; cm_sy <= 1'b0; cm_dx <= 14'd0; cm_dy <= 14'd0;
        cm_shx <= 25'd0;
        cm_ax <= 25'd0; cm_ay <= 25'd0; camx_q_r <= 25'sd0; camy_q_r <= 25'sd0;
        step <= 16'd0; sx0 <= 27'sd0; sy0 <= 27'sd0; y_acc <= 27'sd0;
        x_le <= 12'd0; x_ri <= 12'd0; oy <= 11'd0; row_ptr <= 21'd0;
        last_row <= 12'd0; buf_ok <= 1'b0;
        dv_a <= 24'd0; dv_b <= 13'd0; dv_q <= 24'd0; dv_r <= 13'd0;
        dv_i <= 6'd0; dv_sel <= 2'd0; step_fit <= 16'd0; q_step <= 16'd0;
        ox <= 11'd0; cd1 <= 11'd0; cd2 <= 11'd0;
        wd_lo <= 16'd0;
        we <= 1'b0; wa <= 21'd0; wdata <= 32'd0;
        row_req <= 12'd0; win_x0 <= 12'd0; win_w <= 13'd0; row_go <= 1'b0;
        fill_start <= 1'b0; ox_lo <= 11'd0; ox_hi <= 11'd0;
    end else begin
        we <= 1'b0;
        fill_start <= 1'b0;
        row_go <= 1'b0;


        case (st)
            //--------------------------------------------------------------
            V_IDLE: begin
                done <= 1'b0;
                if (start) begin
                    mode_r   <= mode;
                    fmt_r    <= fmt;
                    sw_r     <= src_w;
                    sh_r     <= src_h;
                    ax_r     <= ax;
                    ay_r     <= ay;
                    base_r   <= dst_base;
                    stride_r <= (dst_stride == 11'd0) ? wpr_i : dst_stride;
                    busy     <= 1'b1;
                    buf_ok   <= 1'b0;
                    dv_sel   <= 2'd0;
                    st       <= V_GEO;
                end
            end

            //--------------------------------------------------------------
            //  几何：W/640、H/480、step_fit/zoom
            //--------------------------------------------------------------
            V_GEO: begin
                dv_a <= (w3 >= h4) ? {1'b0, w_q10} : {1'b0, h_q10};
                dv_b <= (w3 >= h4) ? 13'd640 : 13'd480;
                dv_q <= 24'd0;
                dv_r <= 13'd0;
                dv_i <= 6'd0;
                st   <= V_DIV;
            end

            V_DIV: begin
                if (!dv_done) begin
                    dv_a <= {dv_a[22:0], 1'b0};
                    dv_q <= {dv_q[22:0], 1'b0};
                    dv_r <= dv_r_sh;
                    if (dv_r_sh >= {1'b0, dv_b}) begin
                        dv_r <= dv_r_sh - {1'b0, dv_b};
                        dv_q <= {dv_q[22:0], 1'b1};
                    end
                    dv_i <= dv_i + 6'd1;
                end else begin
                    case (dv_sel)
                        2'd0: begin
                            // 第 1 趟的商 = step_fit（Q10）
                            step_fit <= dv_q[15:0];
                            dv_a     <= {8'd0, dv_q[15:0]} << 4;      // step_fit<<4
                            // zoom 下限 0x04 = 0.25×：再小 q 会超过 16 位
                            dv_b     <= (zoom < 8'd4) ? 13'd4 : {5'd0, zoom};
                            dv_q     <= 24'd0;
                            dv_r     <= 13'd0;
                            dv_i     <= 6'd0;
                            dv_sel   <= 2'd1;
                        end
                        default: begin
                            // 第 2 趟的商 = q = step >> mode（与档位无关）
                            q_step <= (dv_q[15:0] == 16'd0) ? 16'd1 : dv_q[15:0];
                            // ---- 启动 V_CAM 的串行乘（初始化累加器/移位链）----
                            cm_i      <= 5'd0;
                            cm_sx     <= ax_off[10];
                            cm_sy     <= ay_off[10];
                            cm_dx     <= ax_off[10] ? (~ax_off[9:0] + 10'd1) : ax_off[9:0];
                            cm_dy     <= ay_off[10] ? (~ay_off[9:0] + 10'd1) : ay_off[9:0];
                            // ★ 移位链**不在这里**初始化：本拍 q_step 还是旧值，
                            //   dstep 会算错（实测 4× 时 cam 偏 30 像素）。
                            //   改到 V_CAM 首拍，那时 step_n 已是新值。
                            // ★ 2026-11-03 [AREA]：累加器直接从 base 起步，逐位"加或减"，
                            //   收尾不再需要 25 位取负 + 25 位加 base（x/y 各一套）。
                            //   模 2^25：base − Σ ≡ base + (−Σ)，与原式逐位相同。
                            //   sw_r/sh_r 在本次 start 时已锁存，V_CAM 期间不变。
                            cm_ax     <= cm_base_x;
                            cm_ay     <= cm_base_y;
                            st        <= V_CAM;
                        end
                    endcase
                end
            end

            //--------------------------------------------------------------
            //  ★ 锚点 → cam：10 拍串行移位加（换掉原来的组合乘法）
            //--------------------------------------------------------------
            V_CAM: begin
                if (cm_i == 5'd0) begin
                    // 首拍：此时 q_step 已是本趟的新值 ⇒ dstep 才正确
                    cm_shx <= {11'd0, dstep};
                    cm_i   <= 5'd1;
                end else if (cm_i != 5'd11) begin
                    // 累加 bit(cm_i-1)，移位链每拍左移一位
                    if (cm_dx[cm_i - 5'd1]) cm_ax <= cm_sx ? (cm_ax - cm_shx) : (cm_ax + cm_shx);
                    if (cm_dy[cm_i - 5'd1]) cm_ay <= cm_sy ? (cm_ay - cm_shx) : (cm_ay + cm_shx);
                    cm_shx <= cm_shx << 1;
                    cm_i   <= cm_i + 5'd1;
                end else begin
                    // cm_ax/cm_ay 此时已含 bit0..9 的全部贡献
                    camx_q_r <= cm_ax;          // = base ± Σ（见初始化处）
                    camy_q_r <= cm_ay;
                    st       <= V_CALC;
                end
            end

            //--------------------------------------------------------------
            //  窗口：sx0/sy0、列裁剪窗口 [x_le,x_ri)、y 累加器
            //--------------------------------------------------------------
            V_CALC: begin
                step <= step_n;                 // [C1] step = q << mode
                sx0 <= sx0_c;
                sy0 <= sy0_c;
                // y_acc = sy0 + (CH-1)*step = sy0 + CH*step - step
                // ★ 必须用 step_n：本拍 step 寄存器还是旧值（首帧是 0），
                //   用旧值会让首个 y_acc 大 one step ⇒ 整幅画面下移一个源行。
                y_acc <= sy0_c + $signed({1'b0, p_ch}) - $signed({1'b0, step_n});
                x_le   <= x_le_c;
                x_ri   <= x_ri_c;
                win_x0 <= x_le_c;
                win_w  <= ({1'b0, x_ri_c} > {1'b0, x_le_c}) ?
                          ({1'b0, x_ri_c} - {1'b0, x_le_c}) : 13'd1;
                oy      <= 11'd0;
                row_ptr <= base_r;
                st      <= V_ADDR;
            end

            //--------------------------------------------------------------
            //  565 行序翻转：row_ptr 先加到 (CH-1)*stride（迭代加，无乘法）
            //--------------------------------------------------------------
            V_ADDR: begin
                if (fmt_r) begin
                    row_ptr <= base_r;
                    st      <= V_NEXT;
                end else if (oy == (ch - 11'd1)) begin
                    oy      <= 11'd0;
                    st      <= V_NEXT;
                end else begin
                    row_ptr <= row_ptr + {10'd0, stride_r};
                    oy      <= oy + 11'd1;
                end
            end

            //--------------------------------------------------------------
            //  本输出行：黑行 / 复用 / 去要一个源行
            //--------------------------------------------------------------
            V_NEXT: begin
                if (oy == ch) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    st   <= V_IDLE;
                end else begin
                    oy <= oy + 11'd1;
                    if (!row_ok) begin
                        // 纵向越界：整行黑，不需要源像素
                        ox_lo <= 11'd0;
                        ox_hi <= 11'd0;
                        st    <= V_EMIT;
                    end else if (buf_ok && (last_row == src_y)) begin
                        st <= V_EMIT;                       // 缩小：复用上一行
                    end else begin
                        row_req    <= src_y;
                        row_go     <= 1'b1;
                        fill_start <= 1'b1;
                        st         <= V_FILL;
                    end
                    y_acc <= y_acc - $signed({1'b0, step});
                end
            end

            //--------------------------------------------------------------
            V_FILL: begin
                if (fill_done) begin
                    ox_lo    <= f_ox_lo;
                    ox_hi    <= f_ox_hi;
                    last_row <= row_req;
                    buf_ok   <= 1'b1;
                    st       <= V_EMIT;
                end
            end

            //--------------------------------------------------------------
            //  冲刷一行：CW 拍取址 + 两级流水
            //--------------------------------------------------------------
            V_EMIT: begin
                if (!stall) begin
                    if (ox <= cw + 11'd1) begin
                        cd1 <= ox;
                        cd2 <= cd1;
                        ox  <= ox + 11'd1;
                    end
                    // lb_q 比 ox 晚 2 拍 ⇒ cd2 就是这一拍数据所属的列号
                    if ((ox >= 11'd2) && (ox <= cw + 11'd1)) begin
                        // ★ [C4] 888 用列号、565 用字列号：先把索引选好，
                        //   全模块只留一根 21 位行内加法器（原来 888/565 各一根）。
                        if (fmt_r) begin
                            we    <= 1'b1;
                            wa    <= row_ptr + {10'd0, cd2};
                            wdata <= {8'hFF, rgb_g};
                        end else if (cd2[0] == 1'b0) begin
                            wd_lo <= rgb565(rgb_g);
                        end else begin
                            we           <= 1'b1;
                            wa           <= row_ptr + {11'd0, cd2[10:1]};
                            wdata[31:16] <= rgb565(rgb_g);
                            wdata[15:0]  <= wd_lo;
                        end
                    end
                    if (ox == cw + 11'd2) begin
                        ox      <= 11'd0;
                        row_ptr <= fmt_r ? (row_ptr + {10'd0, stride_r})
                                         : (row_ptr - {10'd0, stride_r});
                        st      <= V_NEXT;
                    end
                end
            end

            default: st <= V_IDLE;
        endcase
    end
end

// 连续流许可：水平 1:1（窗口=整行、左沿=0）且垂直 1:1（step 正好 1024）
//   ⇒ 每输出行恰好对应相邻的一个源行，行末字节位置 = 下一行行首。
assign cont1 = (step == 17'd1024) && (win_x0 == 12'd0) && (win_w == {1'b0, src_w[11:0]});

endmodule
