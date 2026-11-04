`timescale 1ns/1ps
//==============================================================================
//  tb_img_layer.v —— 图片图层（img_layer_ctrl = ps_gen_image + 双异步FIFO）单测
//==============================================================================
//
//  【为什么要先单测】
//  ps_gen_image 是这次改造里最复杂的新模块（双扫描 + 预取领先 + 跨时钟域 FIFO）。
//  先用"假仲裁器"把它单独验通，再接进场景图 —— 否则一旦画面不对，
//  分不清是图层错、坐标链错、还是 SDRAM 读仲裁错。
//
//  【假仲裁器】
//  125MHz 域，每拍接受一个请求（req_ack = req_vld），延迟 REQ_LAT 拍后把
//  纹理数组里的对应像素写回像素 FIFO。往返延迟 ≈ REQ_LAT+1 拍 = 88ns，
//  正好落在 LEAD(12 拍@50MHz=240ns) 的预算内。
//
//  【测什么】
//    T1  1:1 全屏（640×480 画在 (0,0)）—— 验证基本映射 + 预取 + 流控
//    T2  缩放（缩到 320×240 画在 (160,120)）—— 验证缩放映射 + 横向复用
//    T3  部分出屏（画在 (-100,-50) 且尺寸超出屏幕）—— 验证包围盒预判
//    另外统计：请求数、像素数、是否发生欠载（欠载 = 流控失效）
//
//  【怎么跑】（工程根目录）
//    iverilog -g2005 -s tb_img_layer -o tb_img_layer.vvp \
//        src/user_source/hdl_source/APP/img_afifo.v \
//        src/user_source/hdl_source/APP/ps_gen_image.v \
//        src/user_source/hdl_source/APP/img_layer_ctrl.v \
//        src/user_source/hdl_source/APP/tb_img_layer.v
//    vvp tb_img_layer.vvp
//
//==============================================================================

module tb_img_layer;

//==============================================================================
//  参数
//==============================================================================
localparam integer IMG_W    = 640;
localparam integer IMG_H    = 480;
localparam integer NPX      = IMG_W * IMG_H;      // 307200
localparam [20:0]  TEX_BASE = 21'd614400;         // TEX0
localparam integer REQ_LAT  = 10;                 // 假仲裁器往返延迟（拍 @125MHz）

//==============================================================================
//  时钟 / 复位
//==============================================================================
reg clk  = 0;
reg mclk = 0;
always #10 clk  = ~clk;        // 周期 20ns = 50MHz（场景图侧）
always #4  mclk = ~mclk;       // 周期  8ns = 125MHz（SDRAM 侧）

reg rst = 1;

//==============================================================================
//  纹理内容（等价于 SDRAM 里 TEX0 的数据）
//==============================================================================
// 用"地址相关"的可辨识图案，这样一旦映射错位，比对会立刻报出具体坐标。
reg [31:0] tex [0:NPX-1];

integer ti;
initial begin
    for (ti = 0; ti < NPX; ti = ti + 1)
        tex[ti] = {8'hFF, ti[7:0], ti[15:8], ti[23:16]};
end

//==============================================================================
//  配置（每轮测试前改）
//==============================================================================
reg [31:0] cfg_pos;        // {X0, Y0}
reg [31:0] cfg_size;       // {RW, RH}
reg [39:0] cfg_k;          // {KX, KY}
reg        layer_en;

//==============================================================================
//  扫描激励：模仿 app_scan_gen
//==============================================================================
reg [15:0] scan_x, scan_y;
reg        ps_start = 0;

wire       layer_ready;
wire       adv = ps_start & layer_ready;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        scan_x <= 16'd0;
        scan_y <= IMG_H - 1;
    end else if (!ps_start) begin
        scan_x <= 16'd0;
        scan_y <= IMG_H - 1;
    end else if (adv) begin
        if (scan_x == IMG_W - 1) begin
            scan_x <= 16'd0;
            scan_y <= scan_y - 16'd1;
        end else begin
            scan_x <= scan_x + 16'd1;
        end
    end
end

//==============================================================================
//  DUT
//==============================================================================
wire        req_vld;
wire [20:0] req_addr;
wire [3:0]  req_mask;
// ★ 应答必须是【组合】的：寄存一拍会让同一个请求被应答两次
//   （req_vld 在 FIFO 真正空掉之前会保持两拍），从而多写出垃圾像素。
wire        req_ack;   // ★ 由仲裁器按队列余量反压（见下）
reg         pix_wr;
reg  [31:0] pix_data;
wire [31:0] pix;
wire        out_valid;

img_layer_ctrl #(
    .IMG_W    (IMG_W),
    .IMG_H    (IMG_H),
    .SRC_W    (640),
    .LEAD     (12),
    .AFIFO_AW (4)
) dut (
    .clk         (clk),
    .rst         (rst),
    .pipe_ce     (1'b1),
    .adv         (adv),
    .frame_active(ps_start),
    .x           (scan_x),
    .y           (scan_y),
    .cfg_pos     (cfg_pos),
    .cfg_size    (cfg_size),
    .cfg_k       (cfg_k),
    .tex_base    (TEX_BASE),
    .layer_en    (layer_en),
    .pix         (pix),
    .out_valid   (out_valid),
    .layer_ready (layer_ready),

    .no_underrun_fix(1'b0),
    .exp_mode(4'd0),
    .mclk        (mclk),
    .req_vld     (req_vld),
    .req_addr    (req_addr),
    .req_mask    (req_mask),
    .req_ack     (req_ack),
    .pix_wr      (pix_wr),
    .pix_data    (pix_data)
);

//==============================================================================
//  假仲裁器（125MHz 域）
//==============================================================================
// ★ 当前 RTL 的契约：req_addr 是 **4 字组基址**，req_mask 说明组内要哪几个字，
//   一组请求要回 popcount(mask) 个字（按地址升序，一个字一个 pix_wr 脉冲）。
//   原来这里是"一请求回一字"的老约定 ⇒ T1 1:1 也会全错。
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

always @(posedge mclk or posedge rst) begin
    if (rst) begin
        qw <= 0; qr <= 0; emit <= 1'b0; kk <= 2'd0;
        cur_ad <= 21'd0; cur_m <= 4'd0;
        pix_wr <= 1'b0; pix_data <= 32'd0;
    end else begin
        pix_wr <= 1'b0;

        // ---- 请求入队 ----
        if (req_vld && req_ack) begin
            qa[qw % QD] <= req_addr;
            qm[qw % QD] <= req_mask;
            qw <= qw + 1;
        end

        // ---- 出队：按掩码逐字回 ----
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
                pix_data <= (gm >= 0 && gm < NPX) ? tex[gm] : 32'd0;
            end
            if (kk == 2'd3) emit <= 1'b0;
            else            kk   <= kk + 2'd1;
        end
    end
end

//==============================================================================
//  统计
//==============================================================================
integer n_req    = 0;      // 图层发出的请求数（组数）
integer n_pix    = 0;      // 假仲裁器写回的像素（字）数
integer n_adv    = 0;      // 实际推进的像素数
integer n_check  = 0;
integer n_bad    = 0;
integer n_under  = 0;      // 欠载（本版用层次引用，名字不符时恒 0）

// 兼容旧打印口的诊断量（本版不再逐项统计）
integer d_push = 0, d_pop = 0, d_rqgo = 0, d_full = 0, d_maxcnt = 0;

always @(posedge mclk) if (!rst && req_vld) n_req = n_req + 1;
always @(posedge mclk) if (!rst && pix_wr)  n_pix = n_pix + 1;
always @(posedge clk)  if (!rst && adv)     n_adv = n_adv + 1;

//==============================================================================
//  期望值计算（和 ps_gen_image 用同一套公式，独立重写一遍做交叉验证）
//==============================================================================
function [15:0] exp_sx;
    input [15:0] px;
    reg   [16:0] lx;
    reg signed [30:0] mx;
    begin
        lx = px - cfg_pos[31:16];
        mx = $signed({1'b0, lx[10:0]}) * $signed(cfg_k[39:20]);
        exp_sx = (mx >>> 16);
    end
endfunction

function [15:0] exp_sy;
    input [15:0] py;
    reg   [16:0] ly;
    reg signed [30:0] my;
    begin
        ly = py - cfg_pos[15:0];
        my = $signed({1'b0, ly[10:0]}) * $signed(cfg_k[19:0]);
        exp_sy = (my >>> 16);
    end
endfunction

function exp_inbox;
    input [15:0] px;
    input [15:0] py;
    reg signed [16:0] pxs, pys, x0s, y0s, x1s, y1s;
    begin
        pxs = $signed({1'b0, px});
        pys = $signed({1'b0, py});
        x0s = $signed(cfg_pos[31:16]);
        y0s = $signed(cfg_pos[15:0]);
        x1s = x0s + $signed({1'b0, cfg_size[31:16]});
        y1s = y0s + $signed({1'b0, cfg_size[15:0]});
        exp_inbox = (pxs >= x0s) && (pxs < x1s) &&
                    (pys >= y0s) && (pys < y1s);
    end
endfunction

// 输出坐标延迟 3 个 adv 拍，用来和 pix 对齐（D_img = 3）
// ★ 2026-10-23：流水深度不再是固定的 3 拍，改成 8 级延迟链 + 自动扫描
//   （每个用例同时统计 d=0..7 各自的错误数，谁归零就是真实深度）。
reg [15:0] qx [0:7];
reg [15:0] qy [0:7];
integer    qi;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        for (qi = 0; qi < 8; qi = qi + 1) begin qx[qi] <= 16'd0; qy[qi] <= IMG_H-1; end
    end else if (adv) begin
        qx[0] <= scan_x;      qy[0] <= scan_y;
        for (qi = 1; qi < 8; qi = qi + 1) begin
            qx[qi] <= qx[qi-1];   qy[qi] <= qy[qi-1];
        end
    end
end

integer errd [0:7];
integer dd, esx_d, esy_d;
reg [31:0] exp_d;

//==============================================================================
//  比对（在 adv 那一拍比较 pix 与 qx[2]/qy[2] 的期望）
//==============================================================================
reg [31:0] exp_pix;
integer    esx, esy;

// ★ 只在 out_valid=1 时比对：图层流水线有 3 拍延迟，帧起点前 3 拍 pix 还是
//   无效值（out_valid=0），拿它比会误报。
always @(posedge clk) begin
    if (!rst && adv && out_valid) begin
        n_check = n_check + 1;

        // ---- 延迟自动扫描：d=0..7 各自累计错误数 ----
        for (dd = 0; dd < 8; dd = dd + 1) begin
            if (exp_inbox(qx[dd], qy[dd]) && layer_en) begin
                esx_d = exp_sx(qx[dd]);
                esy_d = exp_sy(qy[dd]);
                exp_d = tex[esy_d * 640 + esx_d];
            end else begin
                exp_d = 32'h0000_0000;
            end
            if (pix !== exp_d) errd[dd] = errd[dd] + 1;
        end

        // 先用 d=2 作为"官方口径"（保持原有统计口径不变）
        if (exp_inbox(qx[2], qy[2]) && layer_en) begin
            esx = exp_sx(qx[2]);
            esy = exp_sy(qy[2]);
            exp_pix = tex[esy * 640 + esx];
        end else begin
            exp_pix = 32'h0000_0000;
        end
        if (pix !== exp_pix) begin
            n_bad = n_bad + 1;
            if (n_bad <= 3)
                $display("    [%0d] 坐标(%0d,%0d) 期望 %h 实得 %h  <<< 错",
                         n_check, qx[2], qy[2], exp_pix, pix);
        end

        // 欠载检测：要弹像素但 FIFO 空
        if (dut.u_gen.pop_want && dut.u_pix_fifo.rempty) n_under = n_under + 1;
    end
end

//==============================================================================
//  逐拍 trace（临时诊断）
//==============================================================================
integer trc = 0;
always @(posedge clk) begin
    if (!rst && ps_start && trc < 6) begin
        trc = trc + 1;
        $display("    [trc%0d] tbcfg(%0d,%0d,%0d,%0d) dutcfg(%0d,%0d,%0d,%0d) | adv=%b warm=%b x=%0d y=%0d ds_sx=%0d ds_sy=%0d new=%b s1n=%b s2n=%b pix=%h head=%h cnt=%0d rqi=%0d dsi=%0d",
                 trc, $signed(cfg_pos[31:16]), $signed(cfg_pos[15:0]),
                 $signed(cfg_k[39:20]), $signed(cfg_k[19:0]),
                 $signed(dut.u_gen.X0), $signed(dut.u_gen.Y0),
                 $signed(dut.u_gen.KX), $signed(dut.u_gen.KY),
                 adv, dut.u_gen.warmed, scan_x, scan_y,
                 dut.u_gen.ds_sx, dut.u_gen.ds_sy, dut.u_gen.ds_new,
                 dut.u_gen.s1_is_new, dut.u_gen.s2_is_new,
                 pix, dut.u_pix_fifo.dout, dut.u_pix_fifo.rcount,
                 dut.u_gen.rq_idx, dut.u_gen.ds_idx);
    end
end

//==============================================================================
//  单轮测试
//==============================================================================
integer guard;

task run_case;
    input [255:0] name;
    input [31:0]  pos;
    input [31:0]  size;
    input [39:0]  k;
    begin
        $display("");
        $display("  --- %0s ---", name);
        $display("      目标矩形 (X0,Y0)=(%0d,%0d)  RW×RH=%0d×%0d",
                 pos[31:16], pos[15:0], size[31:16], size[15:0]);

        // 摆配置
        ps_start  = 1'b0;
        cfg_pos   = pos;
        cfg_size  = size;
        cfg_k     = k;
        layer_en  = 1'b1;
        repeat (4) @(posedge clk);

        n_req   = 0;
        n_pix   = 0;
        n_adv   = 0;
        n_check = 0;
        n_bad   = 0;
        for (dd = 0; dd < 8; dd = dd + 1) errd[dd] = 0;
        n_under = 0;
        d_push   = 0;
        d_pop    = 0;
        d_rqgo   = 0;
        d_full   = 0;
        d_maxcnt = 0;
        trc      = 0;

        // 起帧
        ps_start = 1'b1;

        // 等走完一帧（带 guard）
        guard = 0;
        while (n_adv < NPX && guard < 4000000) begin
            @(posedge clk);
            guard = guard + 1;
        end
        if (n_adv < NPX) $display("      !! 超时：只推进了 %0d / %0d 个像素", n_adv, NPX);

        ps_start = 1'b0;
        repeat (20) @(posedge clk);

        $display("      请求数 = %0d，写回像素数 = %0d，推进 = %0d",
                 n_req, n_pix, n_adv);
        $display("      延迟扫描 d0..d7 错误数 = %0d %0d %0d %0d %0d %0d %0d %0d",
                 errd[0],errd[1],errd[2],errd[3],errd[4],errd[5],errd[6],errd[7]);
        $display("      比对 = %0d，错误 = %0d，欠载 = %0d",
                 n_check, n_bad, n_under);
        $display("      [dbg] req_push=%0d  rq_go=%0d  req_ack(pop)=%0d  reqfifo_full=%0d  pixfifo_max=%0d",
                 d_push, d_rqgo, d_pop, d_full, d_maxcnt);
    end
endtask

//==============================================================================
//  主流程
//==============================================================================
localparam [39:0] K_1_1  = {20'd65536, 20'd65536};                  // 1:1
// 缩到 320×240：KX = 65536×640/320 = 131072，KY = 65536×480/240 = 131072
localparam [39:0] K_HALF = {20'd131072, 20'd131072};

initial begin
    $display("");
    $display("===================================================================");
    $display("  tb_img_layer —— 图片图层单测（假仲裁器）");
    $display("===================================================================");

    cfg_pos  = 32'd0;
    cfg_size = 32'd0;
    cfg_k    = 40'd0;
    layer_en = 1'b0;

    rst = 1;
    repeat (10) @(posedge clk);
    rst = 0;
    repeat (10) @(posedge clk);

    // T1：1:1 全屏
    run_case("T1  1:1 full screen 640x480 @ (0,0)",
             32'h0000_0000,
             32'h0280_01E0,          // RW=640, RH=480
             K_1_1);

    // T2：缩到 320×240，画在 (160,120)
    run_case("T2  scaled to 320x240 @ (160,120)",
             32'h00A0_0078,          // X0=160, Y0=120
             32'h0140_00F0,          // RW=320, RH=240
             K_HALF);

    // T3：部分出屏（左上角在屏幕外，尺寸超出屏幕）
    run_case("T3  partially off-screen 640x480 @ (-100,-50)",
             32'hFF9C_FFCE,          // X0=-100, Y0=-50（16位补码）
             32'h0280_01E0,
             K_1_1);

    $display("");
    $display("===================================================================");
    if (n_bad == 0 && n_under == 0)
        $display("  RESULT: PASS");
    else
        $display("  RESULT: FAIL  (最后一轮：错误 %0d，欠载 %0d)", n_bad, n_under);
    $display("===================================================================");
    $display("");
    $finish;
end

// ---- 看门狗 ----
initial begin
    #400_000_000;
    $display("*** 看门狗超时 ***");
    $finish;
end

endmodule
