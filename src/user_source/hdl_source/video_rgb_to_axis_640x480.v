//=============================================================================
// 文件名   : video_rgb_to_axis_640x480.v
// 模块名   : video_rgb_to_axis_640x480
// 功能     : 【RGB 视频流 → AXI-Stream 视频流 格式转换】
//            把传统的"并行 RGB + 行场同步"信号，转换成带 SOF/EOL 标记的
//            流式视频接口（类似 AXI-Stream Video）。
//
// 为什么要转换 ?
//   HDMI 发送 IP（APUG092）用的是 AXI-Stream 风格的接口，
//   它不是靠 hs/vs 来定位画面，而是靠三个"标记位"来描述视频流：
//
//     O_video_user  (SOF, Start Of Frame) —— "这一拍是整帧的第一个像素"
//     O_video_valid                     —— "这一拍的数据是有效像素"（等价于 de）
//     O_video_last  (EOL, End Of Line)   —— "这一拍是这一行的最后一个像素"
//
//   打个比方：hs/vs 像是"尺子上的刻度"，而 AXI-Stream 像是"在一个数据流里
//   打上'这是开头''这是结尾'的标签"。后者更适合流水线传输，接收端不需要自己数格子。
//
// 核心思路：
//   ① 用 I_de 的上升沿（!S_de_1d && I_de）判断"一行的第一个像素"；
//      行内用 S_x_cnt 数到 639 判断"最后一个像素" -> 给出 last。
//   ② 用 S_frame_arm 当作"待发射"旗子：只要检测到 vs 跳变，就插旗；
//      等到下一次视频真正开始时（行首像素），把这旗子变成 user 脉冲。
//      这样保证 SOF 一定出现在真正的帧首，而不是出现在消隐区里。
//
// 如何使用 ?
//   输入 I_rgb（24bit）+ I_vs + I_de，输出四个 AXI-Stream 信号接 HDMI IP。
//   本模块是 SD 图片通路（640x480）专用的，所以 last 的判据写死为 639。
//=============================================================================

module video_rgb_to_axis_640x480(
    input  wire        I_clk,          // 像素时钟
    input  wire        I_rst,          // 复位，高有效
    input  wire        I_vs,           // 输入场同步
    input  wire        I_de,           // 输入数据有效
    input  wire [23:0] I_rgb,          // 输入 RGB888 像素数据

    output reg         O_video_user,   // SOF：整帧第一个像素（单周期脉冲）
    output reg         O_video_valid,  // 数据有效（跟随 I_de）
    output reg         O_video_last,   // EOL：一行最后一个像素（单周期脉冲）
    output reg  [23:0] O_video_data    // 输出的像素数据
);

reg        S_vs_1d;        // I_vs 打一拍，用于检测跳变
reg        S_de_1d;        // I_de 打一拍，用于检测行首/行中
reg        S_frame_arm;    // "本帧待发射 SOF"标志（插旗）
reg [10:0] S_x_cnt;        // 行内像素计数器（0~639，11bit 够用）

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        S_vs_1d       <= 1'b0;
        S_de_1d       <= 1'b0;
        S_frame_arm   <= 1'b1;      // 复位后先插旗，保证第一帧也有 SOF
        S_x_cnt       <= 11'd0;
        O_video_user  <= 1'b0;
        O_video_valid <= 1'b0;
        O_video_last  <= 1'b0;
        O_video_data  <= 24'd0;
    end
    else begin
        // 打一拍，为下面的边沿判断做准备
        S_vs_1d <= I_vs;
        S_de_1d <= I_de;

        // ---- 默认输出（先按"普通像素"输出，下面再针对特殊情况覆盖）----
        O_video_user  <= 1'b0;      // 默认不是帧首
        O_video_valid <= I_de;      // valid 直接跟随 de
        O_video_last  <= 1'b0;      // 默认不是行尾
        O_video_data  <= I_rgb;     // 数据直通

        // 任意一次 VS 跳变后，把"下一次 active video 的首像素"当成 SOF
        // 用"跳变"而不是"高低电平"，是因为正/负极性都能兼容
        if (S_vs_1d != I_vs)
            S_frame_arm <= 1'b1;    // 插旗：本帧的第一个有效像素要打 SOF

        if (I_de) begin             // 当前是有效像素
            if (!S_de_1d) begin
                // 一行首像素（de 刚从 0 变 1）
                S_x_cnt <= 11'd0;   // 像素计数清零
                if (S_frame_arm) begin
                    O_video_user <= 1'b1;   // 打出 SOF
                    S_frame_arm  <= 1'b0;   // 旗子收掉，这一帧不再重复打
                end
            end
            else begin
                S_x_cnt <= S_x_cnt + 11'd1; // 行内计数递增
            end

            // 640x480 最后一个有效像素（第 639 个，从 0 数起）
            if (S_x_cnt == 11'd639)
                O_video_last <= 1'b1;       // 打出 EOL
        end
        else begin
            S_x_cnt <= 11'd0;       // 消隐区里计数保持 0
        end
    end
end

endmodule
