//==============================================================================
//  app_scan_gen.v —— 应用层：扫描坐标发生器
//==============================================================================
//
//  【它产生整条场景图的"时间轴"】
//
//      ps_start ──► [ app_scan_gen ] ──► (x, y, valid) ──► 所有像素源生成器
//
//  每来一拍，就吐出一个屏幕坐标。所有生成器都拿同一个坐标去求值，
//  所以它们天然对齐——这是"坐标驱动"模型的地基。
//
//  【★ 扫描顺序必须和 frame_writer 的 px_x/px_y 完全一致 ★】
//
//  frame_writer 内部维护的坐标是：
//      起始 (0, 479)  →  x 先加到 639，再回 0 且 y 减 1  →  最后 (639, 0)
//  它输出的 px_y 就是**屏幕行号**（0 = 屏幕顶行），因为它下游的 frame_fifo_write
//  会做一次行翻转。所以本模块照抄同样的推进方式，就能保证：
//
//      · 顺序一致（一帧 307200 个坐标，一个不差）
//      · 方向一致（不会上下颠倒）
//
//  换句话说：本模块的 (x,y) 和 frame_writer 的 (px_x,px_y) 是**同一时刻同一个值**。
//
//  【valid 为什么就是 ps_start】
//  ps_start 高 = frame_writer 正在收数据 = 本拍要给一个像素。所以：
//      ps_valid = ps_start        （见 App.v）
//      x/y 每拍推进（只要 ps_start 高）
//  这样一帧正好 307200 拍，一拍不多一拍不少。
//
//  【不在取数时坐标停在起点】
//  只要 ps_start = 0，坐标就一直复位到 (0, IMG_H-1)。
//  这样进入取数状态的第一拍，坐标天然就是起点，不需要额外的"复位脉冲"，
//  也不会出现"复位脉冲和第一个像素撞在同一拍"的边界问题。
//  （这个技巧是从 frame_writer.v 里学来的，见那边的注释。）
//
//  【延迟 = 0（本模块自己就是坐标源，不是流水线的一级）】
//
//==============================================================================

module app_scan_gen #(
    parameter integer IMG_W = 640,      // 屏幕宽
    parameter integer IMG_H = 480       // 屏幕高
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        ps_start,        // 来自 frame_writer："开始给这一帧的数据"
    //--------------------------------------------------------------------------
    //  ★ 像素就绪（图片图层引入的流控）★
    //--------------------------------------------------------------------------
    //  图片图层的像素要从 SDRAM 读回来，偶尔会来不及。这时本模块必须**停住**
    //  坐标。场景图、frame_writer 延迟链也用相同的推进使能，保证一起冻结。
    //
    //  pixel_ready = 所有图片图层都"要么不需要新像素、要么像素 FIFO 里有货"。
    //  没有图片图层时它恒为 1，行为与改造前完全一致。
    input  wire        pixel_ready,
    output reg  [15:0] x,               // 当前扫描坐标
    output reg  [15:0] y,
    output wire        valid            // 该坐标有效（= 本拍真的取走了一个像素）
);

//==============================================================================
//  ★★★ 2026-10-16 上板修复：复位的判据必须是 ps_start，不能是 streaming ★★★
//==============================================================================
//  【踩过的坑（上板"门控一开整屏全乱"的真因）】
//  原代码是：
//      end else if (!streaming) begin x <= 0; y <= IMG_H-1; end   // ← 复位到帧起点
//  这里 streaming = ps_start & pixel_ready，所以**图片层只要停一拍**
//  （pixel_ready=0），坐标就跳回 (0,479) —— 而这一帧可能已经画到第 15 万个像素了。
//
//  更致命的是 **frame_writer 那边是"保持"**：
//      frame_writer.v:  wire streaming = (state == S_STREAM);   // 与 pixel_ready 无关
//                       if (!streaming) 复位  else if (take) 推进
//      停顿时 state 仍是 S_STREAM → 既不复位也不推进 → **保持**
//
//  于是"场景坐标复位 / frame_writer 保持" → 两边彻底错位 →
//  一帧里的内容被写到错误的位置 → **整屏全乱**。
//
//  【为什么以前没暴露】
//  这段代码是图片图层引入**之前**写的，那时 pixel_ready 恒 1、streaming ≡ ps_start，
//  所以"停"只能发生在帧间（ps_start=0）——复位恰好是对的。加上门控后这个前提被打破，
//  但复位判据没跟着改。所以"整屏被拖累"从来不是门控的错，是这一行。
//
//  【修法】判据改成 **!ps_start**（只认"帧间空闲"），帧内停顿一律保持：
//      !ps_start        → 帧间空闲 → 复位到 (0,479)，等下一帧
//      ps_start & ready → 推进
//      ps_start & !ready→ **保持不动**（和 frame_writer 完全对称）
//  注意 valid 仍然 = streaming：停顿时必须给 0，否则 frame_writer 会多算像素。
//==============================================================================
wire streaming = ps_start & pixel_ready;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        x <= 16'd0;
        y <= IMG_H - 1;
    end else if (!ps_start) begin
        // ★ 帧间空闲：坐标停在起点，等着 ps_start 第一拍来取 ★
        //   （判据是 ps_start，不是 streaming —— 否则帧内停顿会把坐标打回去）
        x <= 16'd0;
        y <= IMG_H - 1;
    end else if (streaming) begin
        // 和 frame_writer 的 px_x/px_y 推进逻辑逐字一致
        if (x == IMG_W - 1) begin
            x <= 16'd0;
            y <= y - 16'd1;
        end else begin
            x <= x + 16'd1;
        end
    end
    // else：ps_start=1 且 pixel_ready=0 → 帧内停顿 → **保持**（什么都不做）
end

assign valid = streaming;

endmodule
