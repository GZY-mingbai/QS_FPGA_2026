//=============================================================================
// 文件名   : video_source_test.v
// 模块名   : video_source_test
// 功能     : 【测试图案发生器（Test Pattern Generator, TPG）】
//            自己产生视频时序 + 一串递增的数据当作像素，用来测试 HDMI 发送通路。
//            默认参数是 1920x1080（1080P），和工程主体用的 640x480 不同。
//
// 为什么要写这个模块？
//   和 color_bar.v 是同一类东西 —— 都是"不依赖外部存储的纯视频源"。
//   区别在于：
//     color_bar.v      —— 出彩条图案（给人眼看），用于验证时序；
//     video_source_test—— 出"递增的灰度数据"（给机器看），
//                         而且带 SOF/valid/EOL 标记，直接对接 AXI-Stream 接口。
//   它常用于：接一个 HDMI 发送器，看屏幕上是不是一条从黑到白的渐变带 ——
//   如果是，说明视频发送通路完全正常；如果屏幕黑的，说明发送端有问题。
//
// 本模块的两个关键设计：
//   ① S_tpg_en（使能）：由外部 I_tpg_trig_en 触发才开始出图。
//      这样你可以精确控制"什么时候开始"。
//   ② 三级计数器：S_col_cnt（数像素）/ S_line_cnt（数行），
//      再用它们和 HBLANK/VBLANK 比较，产生 valid / user / last 三个标记。
//      —— 这是"用计数器直接生成 AXI-Stream 标记"的典型写法。
//
// parameter 与 localparam 的区别（初学者小知识）：
//   parameter  —— 可以被上层例化时覆盖（比如改成 640x480）；
//   localparam —— 模块内部自己算出来的，不允许覆盖（如 HBLANK）。
//
// 如何使用 ?
//   本模块在 IP 例程里可能没有被顶层例化（它是备用/参考模块）。
//   要使用的话：I_clk 接像素时钟，I_tpg_trig_en 给一个触发脉冲即可；
//   输出接 AXI-Stream 视频接收端。想改分辨率就改 parameter。
//=============================================================================

module video_source_test #(
    // 默认参数按 1080P 配置；如果要产生 640x480 的图案，例化时覆盖即可
    parameter HTOTAL  = 2200,      // 一行总像素数（含消隐）
    parameter HACTIVE = 1920,      // 行有效像素数
    parameter HFP     = 88,        // 行前肩
    parameter HSA     = 44,        // 行同步宽度
    parameter HBP     = 148,       // 行后肩

    parameter VTOTAL  = 1125,      // 一帧总行数
    parameter VACTIVE = 1080,      // 场有效行数
    parameter VFP     = 4,         // 场前肩
    parameter VSA     = 5,         // 场同步宽度
    parameter VBP     = 36         // 场后肩
) 
(
    input wire         I_clk,          // 像素时钟
    input wire         I_rst,          // 复位，高有效

    input wire         I_tpg_trig_en,  // 触发使能：给一拍脉冲，TPG 开始工作

    output reg         O_video_user,   // SOF：整帧第一个像素
    output reg         O_video_valid,  // 数据有效
    output reg         O_video_last,   // EOL：一行最后一个像素
    output wire[23:0]  O_video_data,   // 像素数据输出
    input wire         I_video_ready   // 下游是否准备好（本模块未使用，保留接口）

);


    // localparam：由上面参数算出来的，不允许外部覆盖
    localparam VBLANK = VTOTAL - VACTIVE;   // 场消隐行数
    localparam HBLANK = HTOTAL - HACTIVE;   // 行消隐像素数


    reg        S_tpg_en;       // TPG 工作使能
    reg[13:0]  S_col_cnt;      // 列（水平）计数器，14bit 足以数十千像素
    reg[13:0]  S_line_cnt;     // 行（垂直）计数器
	reg[7:0]   S_tpg_data;     // 测试图案数据（8bit 递增）


    //--------------------------------------------------------------------
    // 使能锁存：I_tpg_trig_en 来一次就永久打开（直到复位）
    //--------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst)
            S_tpg_en <= 1'b0;
        else
            if(I_tpg_trig_en)
                S_tpg_en <= 1'b1;
            else
                S_tpg_en <= S_tpg_en;   // 保持
    end


    //--------------------------------------------------------------------
    // 列计数器：0 ~ HTOTAL-1 循环
    //--------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst || !S_tpg_en)
            S_col_cnt <= 'd0;
        else
            if(S_col_cnt == HTOTAL-1)
                S_col_cnt <= 'd0;
            else
                S_col_cnt <= S_col_cnt + 1'b1;
    end


    //--------------------------------------------------------------------
    // 行计数器：每到"一整行结束"（列计数满）才 +1
    //--------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst || !S_tpg_en)
            S_line_cnt <= 'd0;
        else
            if(S_col_cnt == HTOTAL-1)       // 行末
                begin
                    if(S_line_cnt == VTOTAL-1)  // 帧末
                        S_line_cnt <= 'd0;
                    else
                        S_line_cnt <= S_line_cnt + 1'b1;
                end
            else
                S_line_cnt <= S_line_cnt;
    end


    //--------------------------------------------------------------------
    // valid：进入"有效显示区"（行、列都越过消隐）时拉高
    //--------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst || !S_tpg_en)
            O_video_valid <= 1'b0;
        else
            if(S_line_cnt >= VBLANK && S_col_cnt >= HBLANK)
                O_video_valid <= 1'b1;
            else
                O_video_valid <= 1'b0;
    end

    //--------------------------------------------------------------------
    // user（SOF）：有效区第一个像素的准确时刻 = 行、列同时刚到边界
    //--------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst || !S_tpg_en)
            O_video_user <= 1'b0;
        else
            if(S_line_cnt == VBLANK && S_col_cnt == HBLANK)
                O_video_user <= 1'b1;
            else
                O_video_user <= 1'b0;
    end

    //--------------------------------------------------------------------
    // last（EOL）：有效行内的最后一个像素（列走到 HTOTAL-1）
    //--------------------------------------------------------------------
    always @(posedge I_clk or posedge I_rst) begin
        if(I_rst || !S_tpg_en)
            O_video_last <= 1'b0;
        else
            if(S_line_cnt >= VBLANK && S_col_cnt == HTOTAL-1)
                O_video_last <= 1'b1;
            else
                O_video_last <= 1'b0;
    end

	
    //--------------------------------------------------------------------
    // 测试图案数据：在每个有效像素上不断 +1，到 255 后自然回绕到 0
    // 效果：屏幕从左到右、从上到下是一条"灰色渐变/条纹"。
    // 注意这里没有复位分支（代码风格不严谨，但功能上依赖回绕也能工作）。
    //--------------------------------------------------------------------
    always @ (posedge I_clk) begin
    	if(O_video_valid)
        	S_tpg_data <= S_tpg_data + 1'b1;
       	else	
           	S_tpg_data <= 'd0;      // 消隐区清零
    end


    // 只把 8bit 数据放在最低字节，高 16bit 补 0（所以只有蓝色通道有变化）
    assign O_video_data = {8'd0,8'h0,S_tpg_data};

    
endmodule
