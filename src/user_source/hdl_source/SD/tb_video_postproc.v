//=============================================================================
// 文件名   : tb_video_postproc.v
// 模块名   : tb_video_postproc
// 功能     : 【video_postproc 单元自检查】
//
//  被测：SD/video_postproc.v（显示端画质后处理）
//
//  激励：行为级 25MHz 像素流。小帧制（非真 640x480，后处理不关心时序规格）：
//        每行 80 拍 = 64 个 de 像素 + 16 消隐，每帧 10 行，vs 在帧首拉高 4 拍。
//        像素值覆盖 0 / 255 / 中间值（r: {0,255,x*4}，g: y*28，b: 2x+y）。
//
//  检查（全流程逐像素比较，错误计数器汇总）：
//    A. 身份回归：全默认参数（50/50/50/50/风格0）输出 ≡ 3 拍前输入（位精确，
//       不经过期望值函数，独立直比）
//    B. 公式回归：参数组合逐像素对照期望值函数（与 RTL 同一套整数运算；
//       RTL 的 >>> 是 floor 语义，负数向下取整 —— 期望函数用有符号整数的
//       >>> 保持一致）
//    C. 生效时机：参数在帧正中（xc=32,yc=4）改 + tgl 翻转，本帧剩余部分必须
//       仍是旧参数输出，下一个帧首之后才是新参数（checker 的期望参数组在
//       vs 上升沿切换 —— 与 DUT 的快照协议同语义，DUT 提前/推迟生效即 FAIL）
//    D. 对齐：out_de 必须等于 3 拍前的 in_de（DELAY=3）
//    E. 风格：黑白 = 饱和度强制 0（覆盖滑条），反色 = 255−x 最后一级
//
//  【采样语义】（务必保持）
//    · 移位链在 posedge 里用"沿前值"采样 in_rgb/xc/yc（Verilog NBA 语义下
//      读组合线 = 本拍 DUT 实际采到的像素），深度 4 拍，sh[3] 即 3 级流水
//      入口的像素；
//    · checker 在 posedge 后 #1 读 post-NBA 值：out_* 与 sh_*[3] 同拍对应；
//    · 激励用 (xc==0 && yc==0) 定位帧首，避免 vs 检测竞态。
//
//  运行：
//    cd src/user_source/hdl_source
//    iverilog -g2012 -o pp.vvp -s tb_video_postproc SD/video_postproc.v SD/tb_video_postproc.v
//    vvp pp.vvp
//  期望输出：ALL PASS
//=============================================================================

`timescale 1ns / 1ps

module tb_video_postproc;

//=============================================================================
// 0. 时钟 / 复位
//=============================================================================
reg video_clk;
reg rst;

initial begin
    video_clk = 1'b0;
    forever #20 video_clk = ~video_clk;    // 25MHz：40ns 周期
end

//=============================================================================
// 1. 被测模块
//=============================================================================
reg        in_hs, in_vs, in_de;
reg [23:0] in_rgb;
reg [6:0]  pic_bright, pic_contrast, pic_temp, pic_sat;
reg [1:0]  pic_style;
reg        pic_tgl;

wire        out_hs, out_vs, out_de;
wire [23:0] out_rgb;

video_postproc #(.DELAY(3)) dut (
    .video_clk   (video_clk),
    .rst         (rst),
    .in_hs       (in_hs),
    .in_vs       (in_vs),
    .in_de       (in_de),
    .in_rgb      (in_rgb),
    .pic_bright  (pic_bright),
    .pic_contrast(pic_contrast),
    .pic_temp    (pic_temp),
    .pic_sat     (pic_sat),
    .pic_style   (pic_style),
    .pic_tgl     (pic_tgl),
    .out_hs      (out_hs),
    .out_vs      (out_vs),
    .out_de      (out_de),
    .out_rgb     (out_rgb)
);

//=============================================================================
// 2. 像素流发生器：每行 80 拍（64 de + 16 消隐）× 每帧 10 行
//=============================================================================
reg [6:0] xc;                              // 0..79
reg [3:0] yc;                              // 0..9

always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        xc <= 7'd0;
        yc <= 4'd0;
    end else if (xc == 7'd79) begin
        xc <= 7'd0;
        yc <= (yc == 4'd9) ? 4'd0 : yc + 4'd1;
    end else begin
        xc <= xc + 7'd1;
    end
end

always @(*) begin
    in_de = (xc < 7'd64);
    // vs 上升沿必须落在消隐区（真实时序 vs 在垂直消隐）：放在第 9 行 xc=67，
    // 此刻流水线在飞的 3 个像素是 xc=64/65/66（消隐）⇒ 参数切换零可见接缝。
    // 若把 vs 与 de 重叠（如 yc=0 且 xc<4），快照边沿恰好命中首个有效像素，
    // 该像素会用旧增益处理 —— 上板等价于"帧首一根 1px 彩线"。
    in_vs = (yc == 4'd9) && (xc >= 7'd67);
    in_hs = (xc < 7'd4);
end

// 像素值：覆盖 0 / 255 / 中间值
function [7:0] gen_r(input [6:0] xx);
    begin
        if (xx == 7'd0)       gen_r = 8'd0;
        else if (xx == 7'd63) gen_r = 8'd255;
        else                  gen_r = {xx[5:0], 2'b00};   // x*4：4..248
    end
endfunction

function [7:0] gen_g(input [3:0] yy);
    begin
        gen_g = yy * 8'd28;                                    // 0..252
    end
endfunction

function [7:0] gen_b(input [6:0] xx, input [3:0] yy);
    begin
        gen_b = {1'b0, xx[5:0], 1'b0} + {4'b0, yy};            // 2x+y：0..135
    end
endfunction

always @(*) begin
    in_rgb = (in_de) ? {gen_r(xc), gen_g(yc), gen_b(xc, yc)} : 24'd0;
end

//=============================================================================
// 3. 输入侧 3 拍移位链：checker 在边沿 m 后读到的 sh[j] = I(m−1−j)，
//    而 out = f(I(m−3)) ⇒ 比对下标取 sh[2]（深度 3 恰好，勿改 4 取 [3]）
//=============================================================================
reg [23:0] sh_rgb [0:2];
reg        sh_de  [0:2];
reg [6:0]  sh_xc  [0:2];
reg [3:0]  sh_yc  [0:2];

always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        sh_rgb[0] <= 24'd0; sh_rgb[1] <= 24'd0; sh_rgb[2] <= 24'd0;
        sh_de[0]  <= 1'b0;  sh_de[1]  <= 1'b0;  sh_de[2]  <= 1'b0;
        sh_xc[0]  <= 7'd0;  sh_xc[1]  <= 7'd0;  sh_xc[2]  <= 7'd0;
        sh_yc[0]  <= 4'd0;  sh_yc[1]  <= 4'd0;  sh_yc[2]  <= 4'd0;
    end else begin
        // 全部读"沿前值"：sh[0] 即本拍 DUT 采到的像素
        sh_rgb[0] <= in_rgb;  sh_rgb[1] <= sh_rgb[0];  sh_rgb[2] <= sh_rgb[1];
        sh_de[0]  <= in_de;   sh_de[1]  <= sh_de[0];   sh_de[2]  <= sh_de[1];
        sh_xc[0]  <= xc;      sh_xc[1]  <= sh_xc[0];   sh_xc[2]  <= sh_xc[1];
        sh_yc[0]  <= yc;      sh_yc[1]  <= sh_yc[0];   sh_yc[2]  <= sh_yc[1];
    end
end

//=============================================================================
// 4. 期望参数组（vs 上升沿切换，与 DUT 快照协议同语义）
//=============================================================================
reg [6:0] exp_bright, exp_contrast, exp_temp, exp_sat;
reg [1:0] exp_style;
reg [6:0] pend_bright, pend_contrast, pend_temp, pend_sat;
reg [1:0] pend_style;

reg vs_d2;
always @(posedge video_clk or posedge rst) begin
    if (rst) begin
        vs_d2       <= 1'b0;
        exp_bright  <= 7'd50;  exp_contrast <= 7'd50;
        exp_temp    <= 7'd50;  exp_sat     <= 7'd50;
        exp_style   <= 2'd0;
    end else begin
        vs_d2 <= in_vs;                        // 沿前值语义：检测本拍上升沿
        if (in_vs & ~vs_d2) begin
            exp_bright  <= pend_bright;   exp_contrast <= pend_contrast;
            exp_temp    <= pend_temp;     exp_sat      <= pend_sat;
            exp_style   <= pend_style;
        end
    end
end

//=============================================================================
// 5. 期望值函数（与 RTL 同一套整数运算；>>> 保持 floor 语义）
//=============================================================================
function integer clampi(input integer v);
    begin
        if (v < 0)        clampi = 0;
        else if (v > 255) clampi = 255;
        else              clampi = v;
    end
endfunction

// ★ 增益量化：与 RTL quant_me 同一套阈值/舍入/归一化（A → M·2^E，M=0 即零增益）
//   返回打包 {M[3:0], E[3:0]}（function 不能调 task，拆包用 LHS 拼接赋值）
function [7:0] quantq;
    input integer a;
    integer av, m, e;
    begin
        av = (a < 0) ? 0 : a;
        if      (av >= 1024) e = 7;
        else if (av >= 512)  e = 6;
        else if (av >= 256)  e = 5;
        else if (av >= 128)  e = 4;
        else if (av >= 64)   e = 3;
        else if (av >= 32)   e = 2;
        else if (av >= 16)   e = 1;
        else                 e = 0;
        if (e == 0)
            m = av;
        else begin
            m = (av + (1 << (e - 1))) >> e;
            if (m >= 16) begin
                m = 8;
                e = e + 1;
            end
        end
        quantq = {m[3:0], e[3:0]};
    end
endfunction

function [23:0] exp_pixel(input [7:0] ir, input [7:0] ig, input [7:0] ib);
    integer cgain, dtg, Ad;
    integer ar, ag, ab, mr, er, mg, eg, mb, eb;
    integer boff, Br, Bg, Bb, a128;
    integer sg_m, sg_e;
    integer s1r, s1g, s1b, Y, rr, rg, rb;
    integer b_int, t_int, s_int, sg_raw, bgain, cg2, bq, dl, dla, wbr, wbg, wbb;
    reg [7:0] tq;
    begin
        // 【坑】exp_bright/exp_temp 是 unsigned reg，直接写 (exp_bright-50)
        //  会把整个表达式拖成无符号（0−50 = 2^32−50），>>> 变逻辑移位。
        //  先转 integer（有符号）再做减法。
        b_int = exp_bright;
        t_int = exp_temp;
        s_int = exp_sat;
        // ---- 增益（与 RTL 2026-10-26 版同式）----
        //   对比度 cgain：围绕中灰的增益，下限 20（=102）
        //   亮度  bgain：曝光（乘性）0.22x..1.78x，锚在黑
        //   色温  Ad   ：纯通道增益，锚在黑 ⇒ 中灰会被染色
        cgain = (exp_contrast * 1311) >> 8;
        if (cgain < 102) cgain = 102;
        bgain = 256 + (b_int - 50) * 4;
        if (bgain < 0) bgain = 0;
        cg2   = (cgain * bgain) >> 8;
        dtg   = ((t_int - 50) * 328) >>> 8;
        // ---- 音调（对比度×曝光，三通道同一曲线）+ 白平衡（只降不升，乘在音调之后）----
        //   暖(dtg>0)：R=1.00 G=1−δ/2 B=1−δ；冷(dtg<0)：R=1−δ G=1−δ/2 B=1.00
        dla   = (dtg < 0) ? -dtg : dtg;      // δ = |dtg|（与对比度解耦）
        wbr   = (dtg < 0) ? (256 - dla) : 256;
        wbg   = 256 - (dla >> 1);
        wbb   = (dtg < 0) ? 256 : (256 - dla);
        bq    = 128 - (cgain >> 1);          // 音调回中项（不含色温/曝光）
        s1r = clampi((((cg2 * ir) + 128) >> 8) + bq);   // 先音调 + 截断
        s1g = clampi((((cg2 * ig) + 128) >> 8) + bq);
        s1b = clampi((((cg2 * ib) + 128) >> 8) + bq);
        s1r = (s1r * wbr) >> 8;                        // 再白平衡（≤1）
        s1g = (s1g * wbg) >> 8;
        s1b = (s1b * wbb) >> 8;
        // ---- 级 2：饱和度（★ 量化的对象是 Q8 增益 256，不是滑条原值 50；黑白 = 0）----
        sg_raw = (s_int * 1311) >> 8;
        if (exp_style == 2'd1) sg_raw = 0;
        tq = quantq(sg_raw); sg_m = tq[7:4]; sg_e = tq[3:0];
        Y     = (s1r + 2 * s1g + s1b) >> 2;
        rr    = ((((sg_m * (s1r - Y)) * (2 ** sg_e)) + 128) >>> 8) + Y;
        rg    = ((((sg_m * (s1g - Y)) * (2 ** sg_e)) + 128) >>> 8) + Y;
        rb    = ((((sg_m * (s1b - Y)) * (2 ** sg_e)) + 128) >>> 8) + Y;
        s1r   = clampi(rr);  s1g = clampi(rg);  s1b = clampi(rb);
        // ---- 级 3：风格 ----
        if (exp_style == 2'd2) begin
            s1r = 255 - s1r;  s1g = 255 - s1g;  s1b = 255 - s1b;
        end
        exp_pixel = {s1r[7:0], s1g[7:0], s1b[7:0]};
    end
endfunction

//=============================================================================
// 6. 错误计数
//=============================================================================
integer errs;
integer checks;

task chk24(input [23:0] got, input [23:0] expv, input [8*24:1] name);
    begin
        checks = checks + 1;
        if (got !== expv) begin
            errs = errs + 1;
            if (errs <= 20) begin
                $display("FAIL %0s t=%0t got=%06h exp=%06h", name, $time, got, expv);
                if (errs <= 3)
                    $display("   dbg: w(b/c/t/s/st)=%0d/%0d/%0d/%0d/%0d arm=%b tb_done=%b bank=%b | exp(b/c/t/s/st)=%0d/%0d/%0d/%0d/%0d in=%h",
                             dut.w_bright, dut.w_contrast, dut.w_temp, dut.w_sat, dut.w_style,
                             dut.arm, dut.tb_done, dut.bank_sel,
                             exp_bright, exp_contrast, exp_temp, exp_sat, exp_style,
                             sh_rgb[2]);
            end
        end
    end
endtask

task chk1(input got, input expv, input [8*24:1] name);
    begin
        checks = checks + 1;
        if (got !== expv) begin
            errs = errs + 1;
            if (errs <= 20)
                $display("FAIL %0s t=%0t got=%b exp=%b", name, $time, got, expv);
        end
    end
endtask

//=============================================================================
// 7. 主 checker（posedge 后 #1 读 post-NBA 值）
//=============================================================================
reg identity_mode;                             // 1=与延迟输入直比；0=用期望函数
reg [9:0] warmup;                              // 复位后等移位链填满 + 映射表建完（256 拍）

always @(posedge video_clk) begin
    #1;
    if (!rst) begin
        if (warmup != 10'd400)
            warmup <= warmup + 10'd1;
        else begin
            // D. 对齐：out_de 必须等于 3 拍前 in_de（sh[2] = I(m−3)，见移位链注释）
            chk1(out_de, sh_de[2], "D de-align");
            if (out_de) begin
                if (identity_mode)
                    chk24(out_rgb, sh_rgb[2], "A identity");
                else
                    chk24(out_rgb,
                          exp_pixel(sh_rgb[2][23:16], sh_rgb[2][15:8], sh_rgb[2][7:0]),
                          "B formula");
            end
        end
    end
end

//=============================================================================
// 8. 激励序列
//=============================================================================
// 等到帧首（xc==0 && yc==0 的那一拍之后）
task wait_frame_start;
    begin
        @(posedge video_clk); #1;
        while (!(xc == 7'd0 && yc == 4'd0)) begin
            @(posedge video_clk); #1;
        end
    end
endtask

task wait_frames(input integer n);
    integer i;
    begin
        for (i = 0; i < n; i = i + 1)
            wait_frame_start;
    end
endtask

// 在帧正中（xc=32, yc=4）改参数 + 翻 tgl：保证 arm 在帧首前建立，
// 且"改动当帧剩余部分仍按旧参数比对"有足量像素可查（C 项检查）
task apply_params(input [6:0] b, input [6:0] c, input [6:0] t,
                  input [6:0] s, input [1:0] stl);
    begin
        @(posedge video_clk); #1;
        while (!(xc == 7'd32 && yc == 4'd4)) begin
            @(posedge video_clk); #1;
        end
        pend_bright  = b;    pend_contrast = c;
        pend_temp    = t;    pend_sat      = s;
        pend_style   = stl;
        pic_bright   = b;    pic_contrast  = c;
        pic_temp     = t;    pic_sat       = s;
        pic_style    = stl;
        pic_tgl      = ~pic_tgl;
    end
endtask

initial begin
    errs    = 0;
    checks  = 0;
    rst     = 1'b1;
    warmup  = 10'd0;
    identity_mode = 1'b1;
    pic_bright = 7'd50;  pic_contrast = 7'd50;
    pic_temp   = 7'd50;  pic_sat      = 7'd50;
    pic_style  = 2'd0;   pic_tgl      = 1'b0;
    pend_bright = 7'd50; pend_contrast = 7'd50;
    pend_temp   = 7'd50; pend_sat      = 7'd50;
    pend_style  = 2'd0;

    repeat (4) @(posedge video_clk);
    #1 rst = 1'b0;

    //---------------------------------------------------------------
    // A. 身份回归：默认参数跑 3 帧，输出 ≡ 3 拍前输入（位精确）
    //---------------------------------------------------------------
    $display("--- A. identity (defaults) ---");
    identity_mode = 1'b1;
    wait_frames(3);

    //---------------------------------------------------------------
    // B/C. 公式回归 + 生效时机（改动当帧剩余部分仍按旧参数比对）
    //---------------------------------------------------------------
    identity_mode = 1'b0;

    $display("--- B1. bright=100 ---");
    apply_params(7'd100, 7'd50, 7'd50, 7'd50, 2'd0);
    wait_frames(2);

    $display("--- B2. bright=0 ---");
    apply_params(7'd0, 7'd50, 7'd50, 7'd50, 2'd0);
    wait_frames(2);

    $display("--- B3. contrast=100 temp=100 ---");
    apply_params(7'd50, 7'd100, 7'd100, 7'd50, 2'd0);
    wait_frames(2);

    $display("--- B4. contrast=0（全图 128） ---");
    apply_params(7'd50, 7'd0, 7'd50, 7'd50, 2'd0);
    wait_frames(2);

    $display("--- B5. sat=0 ---");
    apply_params(7'd50, 7'd50, 7'd50, 7'd0, 2'd0);
    wait_frames(2);

    $display("--- B6. sat=100 ---");
    apply_params(7'd50, 7'd50, 7'd50, 7'd100, 2'd0);
    wait_frames(2);

    $display("--- B7. style=黑白（覆盖 sat=100） ---");
    apply_params(7'd50, 7'd50, 7'd50, 7'd100, 2'd1);
    wait_frames(2);

    $display("--- B8. style=反色（其余默认） ---");
    apply_params(7'd50, 7'd50, 7'd50, 7'd50, 2'd2);
    wait_frames(2);

    $display("--- B9. 组合压力：bright=0 contrast=100 temp=0 sat=100 ---");
    apply_params(7'd0, 7'd100, 7'd0, 7'd100, 2'd0);
    wait_frames(2);

    $display("--- B10. 反色+bright=100 ---");
    apply_params(7'd100, 7'd50, 7'd50, 7'd50, 2'd2);
    wait_frames(2);

    //---------------------------------------------------------------
    // 回到默认：验证"改回去"的路径也恒等
    //---------------------------------------------------------------
    $display("--- B11. 恢复默认参数 → 恒等 ---");
    apply_params(7'd50, 7'd50, 7'd50, 7'd50, 2'd0);
    wait_frames(2);
    identity_mode = 1'b1;
    wait_frames(2);

    //---------------------------------------------------------------
    // 汇总
    //---------------------------------------------------------------
    $display("=========================================");
    if (errs == 0)
        $display("ALL PASS (%0d checks)", checks);
    else
        $display("FAILED: %0d / %0d checks", errs, checks);
    $display("=========================================");
    $finish;
end

// 超时看门狗
initial begin
    #5000000;                                  // 5ms 仿真时间（远超所需）
    $display("TIMEOUT");
    $finish;
end

endmodule
