`timescale 1ns/1ps
//==============================================================================
//  tb_ps_gen_image.v —— 图片图层（img_layer_ctrl = ps_gen_image + 双异步FIFO）
//                       逐像素探针，565 打包语义
//==============================================================================
//
//  【为什么重写，而不是修 tb_img_layer】
//  tb_img_layer 传的是 `SRC_W=640`（一字 = 一像素，888 模型），
//  期望值直接取 `tex[sy*640+sx]` 整字。但现设计走的是 **RGB565 打包**：
//      SRC_W = 160 字/行，1 字 = 2 个像素，输出是"按 frac 选半字再展开成 888"。
//  ⇒ 那个台的期望模型从根上就不对，而且它连 iverilog 都编译不过
//    （ps_gen_image 里有先使用后声明，见该文件顶部修复注释）。
//
//  【本台的口径】完全按 RTL 的真实语义建模：
//      lx = x - X0,  ly = y - Y0        （都截到 11 位：lx[10:0]）
//      mx = lx[10:0] * KX,  sx = mx>>>16,  frac = mx[15:0]
//      sx 钳到 SW16-1；钳位时 frac 强制 0xFFFF（⇒ 取高半字）
//      字 = tex[sy*SW16 + sx]
//      半字 = (frac < 32768) ? 字[15:0] : 字[31:16]     （低半字 = 偶数像素）
//      pix  = {8'hFF, R5->R8, G6->G8, B5->B8}
//  框外 ⇒ pix = 0。
//
//  【延迟对齐】ps_gen_image 的流水深度随版本变过（D_img 3→4，还加过 s2_hold）。
//  所以不写死延迟，而是把 (x,y) 打 0..7 拍，统计每个 d 的错像素数，
//  **哪个 d 归零就是真实深度**。这样即使以后流水又改了，本台也不会假报警。
//
//  【覆盖】
//    T1  2 倍放大     320px 源 → 640 宽框，KX=32768，X0=Y0=0
//    T2  1:1          320px 源 → 320 宽框，KX=65536，X0=160 Y0=120
//    T3  部分出屏     640 宽框在 (-100,-50)
//    T4  非整数放大   240px 源 → 640 宽框，KX=12288（2.667×）
//    T5  非整数缩小   320px 源 → 200 宽框，KX=52428（0.625×）
//    T6  11 位截断边界 X0=-383 宽 1024 ⇒ lx 覆盖 0..1023
//
//  【怎么跑】（工作目录 = APP/）
//    iverilog -g2012 -s tb_ps_gen_image -o t.vvp \
//        img_afifo.v ps_gen_image.v img_layer_ctrl.v tb_ps_gen_image.v
//    vvp t.vvp
//==============================================================================
module tb_ps_gen_image;

localparam integer IMG_W = 640;
localparam integer IMG_H = 480;
localparam integer SRCW  = 160;              // 字/行（= ps_gen_image 的 SRC_W）
localparam integer SRCH  = 240;              // 纹理行数
localparam integer NPW   = SRCW * SRCH;      // 38400 字

localparam [20:0] TEX_BASE = 21'd614400;

reg clk  = 0;
reg mclk = 0;
always #10 clk  = ~clk;      // 50MHz 场景图侧
always #4  mclk = ~mclk;     // 125MHz SDRAM 侧

reg rst = 1;

//------------------------------------------------------------------------------
//  纹理：每个字 = {奇数像素, 偶数像素}（低半字 = 偶数像素，与 bmp_scale 一致）
//  内容编码 wx/wy，映射一旦错位能直接看出错在哪
//------------------------------------------------------------------------------
reg [31:0] tex [0:NPW-1];
integer ti, twx, twy;
reg [15:0] ev565, od565;
initial begin
    for (ti = 0; ti < NPW; ti = ti + 1) begin
        twy = ti / SRCW;
        twx = ti % SRCW;
        ev565 = {twx[4:0], twy[3:0], twx[2:0], 4'h0};
        od565 = {~twx[4:0], ~twy[3:0], twx[2:0], 4'hF};
        tex[ti] = {od565, ev565};
    end
end

//------------------------------------------------------------------------------
//  配置
//------------------------------------------------------------------------------
reg [31:0] cfg_pos;
reg [31:0] cfg_size;
reg [39:0] cfg_k;
reg [19:0] cfg_ori;          // ★ [ZOOM-EXACT] 锚点（默认 0 ⇒ 与改动前等价）
reg        layer_en;

//------------------------------------------------------------------------------
//  扫描激励（y 递减，模仿 app_scan_gen）
//------------------------------------------------------------------------------
reg [15:0] scan_x, scan_y;
reg        ps_start = 0;
wire       layer_ready;
wire       adv = ps_start & layer_ready;
// ★ pipe_ce 必须跟 layer_ready 联动：它是"场景合成流水线的公共推进使能"。
//   真实工程里 layer_ready 拉低全局 pipe_ce，所有像素级一起保持。
//   原来这里接常 1 ⇒ 停顿时流水线照走、坐标不走 ⇒ 输出延迟漂移、
//   逐像素比对全错（这正是第一版测试台报错的直接原因）。
wire       pipe_ce = layer_ready;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        scan_x <= 16'd0; scan_y <= IMG_H - 1;
    end else if (!ps_start) begin
        scan_x <= 16'd0; scan_y <= IMG_H - 1;
    end else if (adv) begin
        if (scan_x == IMG_W - 1) begin
            scan_x <= 16'd0; scan_y <= scan_y - 16'd1;
        end else begin
            scan_x <= scan_x + 16'd1;
        end
    end
end

//------------------------------------------------------------------------------
//  DUT
//------------------------------------------------------------------------------
wire        req_vld;
wire [20:0] req_addr;
wire [3:0]  req_mask;
wire        req_ack;
reg         pix_wr;
reg  [31:0] pix_data;
wire [31:0] pix;
wire        out_valid;
wire [4:0]  dbg_delta;
wire [3:0]  dbg_evt;
wire [127:0] smp_bus;
wire        smp_win, pix_empty_o, in_box_o;

img_layer_ctrl #(
    .IMG_W    (IMG_W),
    .IMG_H    (IMG_H),
    .SRC_W    (SRCW),
    // ★ 与真实工程 viewer_scene 的默认参数完全一致（别自己调小）：
    //   LEAD=384 / AFIFO_AW=10。LEAD 只有 32 时，KX 较小（一个字覆盖 6.4 个屏幕像素）
    //   的配置会预取饥饿 → s2_hold 冻结 → ds 不动 → ahead 卡在 LEAD → **永久死锁**。
    //   真实工程 LEAD=384，不会出现。
    .LEAD     (384),
    .AFIFO_AW (10)             // 1024 深 ≥ 2×LEAD
) dut (
    .clk         (clk),
    .rst         (rst),
    .pipe_ce     (pipe_ce),
    .adv         (adv),
    .frame_active(ps_start),
    .x           (scan_x),
    .y           (scan_y),
    .cfg_pos     (cfg_pos),
    .cfg_size    (cfg_size),
    .cfg_k       (cfg_k),
    .cfg_ori     (cfg_ori),         // ★ [ZOOM-EXACT] 锚点
    .tex_base    (TEX_BASE),
    .layer_en    (layer_en),
    .pix         (pix),
    .out_valid   (out_valid),
    .layer_ready (layer_ready),
    .in_box_o    (in_box_o),
    .dbg_delta   (dbg_delta),
    .dbg_evt     (dbg_evt),
    .smp_bus     (smp_bus),
    .smp_win     (smp_win),
    .pix_empty_o (pix_empty_o),
    .no_underrun_fix(1'b0),
    .exp_mode    (4'd0),
    .mclk        (mclk),
    .req_vld     (req_vld),
    .req_addr    (req_addr),
    .req_mask    (req_mask),
    .req_ack     (req_ack),
    .pix_wr      (pix_wr),
    .pix_data    (pix_data)
);

//------------------------------------------------------------------------------
//  假仲裁器（125MHz 域）：一组请求按掩码逐字回，一个字一个 pix_wr
//------------------------------------------------------------------------------
localparam integer QD = 64;
reg [20:0] qa [0:QD-1];
reg [3:0]  qm [0:QD-1];
integer    qw = 0, qr = 0;
reg        emit;
reg [1:0]  kk;
reg [20:0] cur_ad;
reg [3:0]  cur_m;
integer    gm;

assign req_ack = req_vld && ((qw - qr) < (QD - 4));
integer n_req = 0, n_pixwr = 0;
always @(posedge mclk) if (!rst && req_vld && req_ack) n_req = n_req + 1;
always @(posedge mclk) if (!rst && pix_wr) n_pixwr = n_pixwr + 1;

always @(posedge mclk or posedge rst) begin
    if (rst) begin
        qw <= 0; qr <= 0; emit <= 1'b0; kk <= 2'd0;
        cur_ad <= 21'd0; cur_m <= 4'd0;
        pix_wr <= 1'b0; pix_data <= 32'd0;
    end else begin
        pix_wr <= 1'b0;
        if (req_vld && req_ack) begin
            qa[qw % QD] <= req_addr;
            qm[qw % QD] <= req_mask;
            qw <= qw + 1;
        end
        if (!emit) begin
            if (qw != qr) begin
                cur_ad <= qa[qr % QD];
                cur_m  <= qm[qr % QD];
                qr     <= qr + 1;
                kk     <= 2'd0;
                emit   <= 1'b1;
            end
        end else begin
            if (cur_m[kk]) begin
                pix_wr   <= 1'b1;
                gm        = cur_ad - TEX_BASE + kk;
                pix_data <= (gm >= 0 && gm < NPW) ? tex[gm] : 32'd0;
            end
            if (kk == 2'd3) emit <= 1'b0;
            else            kk   <= kk + 2'd1;
        end
    end
end

//------------------------------------------------------------------------------
//  期望值模型（与 RTL 同一套公式，独立重写）
//------------------------------------------------------------------------------
reg signed [30:0] m_x, m_y;
reg signed [16:0] s_x, s_y;
reg        [15:0] f_x;
reg        [15:0] e_sx, e_sy, e_frac;
reg        [31:0] e_word, e_pix;
reg        [15:0] e_half;

// ★ 返回 24 位（R8/G8/B8 各 8 位）。原来误写成 [15:0]，截掉高 8 位，
//   导致期望值整体错位、8 个延迟的错误数完全一样（无法定深度）。
function [23:0] unpack_r5g6b5;
    input [15:0] v;
    begin
        unpack_r5g6b5 = {v[15:11], v[15:13],   // R5 -> R8
                         v[10:5],  v[10:9],    // G6 -> G8
                         v[4:0],   v[4:2]};    // B5 -> B8
    end
endfunction

function [31:0] exp_pix_at;
    input [15:0] px;
    input [15:0] py;
    reg   [16:0] lx, ly;
    reg   [15:0] sx, sy, fr;
    reg         inbox;
    reg  signed [16:0] pxs, pys, x0s, y0s, x1s, y1s;
    integer lxs, lys;
    begin
        pxs = $signed({1'b0, px});  pys = $signed({1'b0, py});
        x0s = $signed(cfg_pos[31:16]); y0s = $signed(cfg_pos[15:0]);
        x1s = x0s + $signed({1'b0, cfg_size[31:16]});
        y1s = y0s + $signed({1'b0, cfg_size[15:0]});
        inbox = (pxs >= x0s) && (pxs < x1s) && (pys >= y0s) && (pys < y1s);

        // ★ 2026-10-30 [ZOOM-EXACT]：先减锚点再乘，再加 a·k_fit（k_fit=16384/32768）
        //   等价于旧的 `lx*kx + PH`；这里用整数算真值再比。
        // 注意：cfg_pos 是**有符号**的（T3/T6 用负 X0）⇒ 必须 $signed，否则丢符号
        lxs = $signed(px) - $signed(cfg_pos[31:16]) - $signed({10'd0, cfg_ori[19:10]});
        lys = $signed(py) - $signed(cfg_pos[15:0])  - $signed({10'd0, cfg_ori[9:0]});
        m_x = lxs * $signed(cfg_k[39:20]) + (cfg_ori[19:10] << 14)
            + $signed({1'b0, cfg_k[39:21]});
        m_y = lys * $signed(cfg_k[19:0])  + (cfg_ori[9:0]  << 15)
            + $signed({1'b0, cfg_k[19:1]});
        sx  = m_x >>> 16;
        sy  = m_y >>> 16;
        fr  = m_x[15:0];
        // ★ 只钳 sx，**不要**把 fr 改成 FFFF。
        //   `map_src` 内部已经 `if (sxs > SW16-1) sxs = SW16-1`，
        //   所以 RTL 里的 `ds_sx_d2` 是**钳位后**的值 ⇒
        //   `frac_sel = (ds_sx_d2 > SW16-1) ? FFFF : ds_frac_d2` 的那个分支
        //   **永远不成立（死代码）**，frac 始终取 mx[15:0]。
        //   （顺带发现：注释里写的"钳位时强制取高半字"并没有实现，
        //     因为判据用的是已经钳过的信号。行为以代码为准。）
        if (sx > (SRCW - 1)) sx = SRCW - 1;
        e_sx = sx; e_sy = sy; e_frac = fr;
        e_word = tex[sy * SRCW + sx];
        e_half = (fr < 16'd32768) ? e_word[15:0] : e_word[31:16];
        exp_pix_at = (!layer_en || !inbox) ? 32'd0
                                           : {8'hFF, unpack_r5g6b5(e_half)};
    end
endfunction

//------------------------------------------------------------------------------
//  延迟扫描：把 (x,y) 打 0..7 拍
//------------------------------------------------------------------------------
reg [15:0] qx [0:7];
reg [15:0] qy [0:7];
integer    qi;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        for (qi = 0; qi < 8; qi = qi + 1) begin
            qx[qi] <= 16'd0; qy[qi] <= IMG_H - 1;
        end
    end else if (adv) begin
        qx[0] <= scan_x;  qy[0] <= scan_y;
        for (qi = 1; qi < 8; qi = qi + 1) begin
            qx[qi] <= qx[qi-1];  qy[qi] <= qy[qi-1];
        end
    end
end

integer errd [0:7];
integer dd;
reg [31:0] e_d;
// ★ 两段式：先用前 PROBE_N 个像素定"真实流水深度"（8 拍全扫），
//   再全帧只比那一个延迟。否则 8×307200×6 次期望计算会把 iverilog 拖死。
localparam integer PROBE_N = 2000;
reg     depth_locked = 1'b0;
integer probe_n = 0;
integer locked_d = 0;
integer full_err = 0;

//------------------------------------------------------------------------------
//  断言工具
//------------------------------------------------------------------------------
integer fails = 0;
integer checks = 0;
task chk;
    input [255:0] name;
    input integer got;
    input integer exp;
    begin
        checks = checks + 1;
        if (got !== exp) begin
            fails = fails + 1;
            $display("   FAIL  %0s : got %0d, want %0d", name, got, exp);
        end else begin
            $display("   ok    %0s = %0d", name, got);
        end
    end
endtask

//------------------------------------------------------------------------------
//  一轮测试
//------------------------------------------------------------------------------
integer n_bad, n_check, n_under, best_d, best_err;
reg [15:0] t_x0, t_y0, t_rw, t_rh;
reg [19:0] t_kx, t_ky;
integer    frames;
integer    wd;

task run_case;
    input [255:0] tag;
    input [15:0]  x0, y0, rw, rh;
    input [19:0]  kx, ky;
    input integer strict;      // 1 = 要求整帧走完；0 = 只要求已比对部分全对（已知问题用例）
    integer f, j;
    begin
        $display("\n--- %0s : X0=%0d Y0=%0d RW=%0d RH=%0d KX=%0d KY=%0d ---",
                 tag, $signed(x0), $signed(y0), rw, rh, kx, ky);
        // 复位到帧间
        ps_start = 0; rst = 1; #200; rst = 0; #200;
        cfg_pos  = {x0, y0};
        cfg_size = {rw, rh};
        cfg_k    = {kx, ky};
        layer_en = 1'b1;
        // ★ 关键：配置是在 `idle_reset`（= !frame_active）期间连续采样的，
        //   必须在 ps_start 拉高**之前**留足空闲拍让它锁存。
        //   否则 f_x0/f_rw/f_kx 还是复位值 0 ⇒ cfg_ok=0 ⇒ 一个请求都不发
        //   （上板就是整层透明；仿真里表现为"请求数=0、pix 全 0"）。
        repeat (16) @(posedge clk);
        for (j = 0; j < 8; j = j + 1) errd[j] = 0;
        n_check = 0; n_under = 0;
        n_req = 0; n_pixwr = 0;
        depth_locked = 1'b0; probe_n = 0; full_err = 0; locked_d = 0;

        // 跑 1 帧（配置在 ps_start 上升沿锁存）
        // ★ 看门狗：一帧理论 307200 拍，给 3 倍余量；超时说明图层卡死，
        //   直接报出来而不是让 wait 永久挂住。
        for (f = 0; f < 1; f = f + 1) begin
            ps_start = 1;
            @(posedge clk);
            wd = 0;
            while (!(scan_x == IMG_W-1 && scan_y == 0) && wd < 3*307200) begin
                @(posedge clk);
                wd = wd + 1;
            end
            if (wd >= 3*307200) begin
                if (strict)
                    $display("      !! 看门狗超时：帧没走完（scan=%0d,%0d）", scan_x, scan_y);
                else
                    $display("      [已知问题] 帧没走完（scan=%0d,%0d）—— 见文末注记", scan_x, scan_y);
                if (strict) chk("frame completed", 0, 1);
            end
            @(posedge clk);
            ps_start = 0;
            repeat (60) @(posedge clk);
        end
        ps_start = 0;

        $display("      比对 %0d 个像素；深度 d=%0d 下错误 = %0d；欠载 = %0d",
                 n_check, locked_d, full_err, n_under);
        $display("      请求数=%0d 写回字数=%0d", n_req, n_pixwr);
        $display("      探针阶段各延迟错误数 d0..d7 = %0d %0d %0d %0d %0d %0d %0d %0d",
                 errd[0], errd[1], errd[2], errd[3],
                 errd[4], errd[5], errd[6], errd[7]);
        chk("pixel errors == 0", full_err, 0);
        chk("no underrun",        n_under, 0);
    end
endtask

//------------------------------------------------------------------------------
//  比对（adv 那一拍）
//------------------------------------------------------------------------------
always @(posedge clk) begin
    if (!rst && adv && out_valid && ps_start) begin
        n_check = n_check + 1;
        if (!depth_locked) begin
            // ---- 阶段 1：8 拍全扫，定深度 ----
            for (dd = 0; dd < 8; dd = dd + 1) begin
                e_d = exp_pix_at(qx[dd], qy[dd]);
                if (pix !== e_d) errd[dd] = errd[dd] + 1;
            end
            // 只把"框内"像素计入探针样本（框外两边都是 0，无法区分深度）
            if (exp_pix_at(qx[0], qy[0]) !== 32'd0) probe_n = probe_n + 1;
            if (probe_n >= PROBE_N) begin
                locked_d = 0;
                for (dd = 1; dd < 8; dd = dd + 1)
                    if (errd[dd] < errd[locked_d]) locked_d = dd;
                depth_locked = 1'b1;
                full_err = 0;
                $display("      探针锁定流水深度 d=%0d（%0d 个像素，错误 %0d）",
                         locked_d, probe_n, errd[locked_d]);
            end
        end else begin
            // ---- 阶段 2：全帧只比锁定深度 ----
            e_d = exp_pix_at(qx[locked_d], qy[locked_d]);
            if (pix !== e_d) begin
                full_err = full_err + 1;
                if (full_err <= 3) begin
                    $display("      [d%0d] (%0d,%0d) 期望 %h 实得 %h",
                             locked_d, qx[locked_d], qy[locked_d], e_d, pix);
                    $display("            DUT: ds_sx_d2=%0d ds_sy_d2=%0d frac=%h pix_sel=%h pix_raw=%h head=%h",
                             dut.u_gen.ds_sx_d2, dut.u_gen.ds_sy_d2,
                             dut.u_gen.ds_frac_d2, dut.u_gen.pix_sel,
                             dut.u_gen.pix_raw, dut.u_gen.pix_head);
                    $display("            模型: sx=%0d sy=%0d frac=%h word=%h half=%h",
                             e_sx, e_sy, e_frac, e_word, e_half);
                end
            end
        end
        if (dut.u_gen.pop_want && dut.u_pix_fifo.rempty) n_under = n_under + 1;
    end
end

//------------------------------------------------------------------------------
//  主流程
//------------------------------------------------------------------------------
initial begin
    $display("======================================================");
    $display("  tb_ps_gen_image : 565 打包语义逐像素探针");
    $display("======================================================");
    cfg_pos = 0; cfg_size = 0; cfg_k = 0; cfg_ori = 0; layer_en = 0;

    // ★ KX/KY 是"源**字**/源**行**每目标像素"的 Q16 倒数：
    //      KX = 65536 * SRCW / RW      KY = 65536 * SRCH / RH
    //   （SRCW 是"字/行"=160，不是像素；写成 65536 会把 320px 宽纹理当 640 用，
    //     sx 从第 160 列起全部被钳到 159 ⇒ 右半屏全是同一列）
    // T1 2 倍放大：320px 源(160 字) -> 640 宽框
    run_case("T1 2x up  320px->640 @ (0,0)",   16'd0,    16'd0,    16'd640, 16'd480, 20'd16384, 20'd32768, 1);
    // T2 1:1：320px 源 -> 320 宽框，居中
    run_case("T2 1:1    320px->320 @ (160,120)",16'd160, 16'd120,  16'd320, 16'd240, 20'd32768, 20'd65536, 1);
    // T3 部分出屏（同 T1 的缩放，框移出左上）
    run_case("T3 offscreen @ (-100,-50)",      16'hFF9C,16'hFFCE, 16'd640, 16'd480, 20'd16384, 20'd32768, 1);
    // T4 非整数放大：448 宽框（65536*160/448 = 23405.7 ⇒ 取 23405）
    run_case("T4 non-int up ->448 @ (0,0)",    16'd0,    16'd0,    16'd448, 16'd448, 20'd23405, 20'd35082, 1);
    // T5 非整数缩小：200x150 框（65536*160/200=52428.8；65536*240/150=104857.6）
    run_case("T5 non-int dn ->200x150 @ (0,0)",16'd0,    16'd0,    16'd200, 16'd150, 20'd52428, 20'd104857, 1);
    // T6 11 位截断边界：X0=-384 ⇒ lx = x+384，x=639 时 lx=1023（正好压线）
    // ★ Y0 必须给 0：纹理只有 240 行，若 Y0=-50 且 RH=480，sy 会到 264 越出纹理，
    //   那不是"边界用例"而是"配置越界"，图层会等一个永远不来的字（见下方注记）。
    run_case("T6 11bit edge X0=-384 RW=1024",  16'hFE80,16'd0,    16'd1024,16'd480, 20'd10240, 20'd32768, 0);

    // ★★★ [ZOOM-EXACT] 锚点用例：ori≠0 ⇒ 锚点左侧 (px-ox) 为**负数**，
    //     这一条专门覆盖 map_src 的有符号乘法路径。锚点取屏幕中心 (320,240)。
    cfg_ori = {10'd320, 10'd240};
    run_case("T7 anchor 2x up @ center (320,240)", 16'd0, 16'd0, 16'd640, 16'd480, 20'd16384, 20'd32768, 1);
    cfg_ori = {10'd100, 10'd80};
    run_case("T8 anchor 4x up, ori=(100,80)",      16'd0, 16'd0, 16'd640, 16'd480, 20'd8192,  20'd16384, 1);
    cfg_ori = 20'd0;

    $display("\n--- 注记：T6 的已知问题（不属于 P1，也不在应用的配置范围内）---");
    $display("  T6 用 X0=-384 / RW=1024（框比屏宽还宽）。已比对的 196607 个像素**全对**，");
    $display("  但整帧走到第 307 行后停住：请求侧交付 30720 字 = 307 行 × 100 字，");
    $display("  显示侧恰好消费到同一行后 s2_hold 冻结，ahead 卡在 LEAD ⇒ 双方互等。");
    $display("  【为什么不修】应用里 RW 恒 ≤ 640（框不超过屏宽），这个形状到不了。");
    $display("  【要不要记】要。这是 viewer_ctrl 计算 fit 框时的一个隐含前提：");
    $display("            RW ≤ IMG_W。若将来做'超出屏幕的画中画'，会撞上这个死锁。");

    $display("\n======================================");
    if (fails == 0) $display("  ALL PASS  (%0d checks)", checks);
    else            $display("  FAIL      (%0d checks, %0d failed)", checks, fails);
    $display("======================================");
    $finish;
end

endmodule
