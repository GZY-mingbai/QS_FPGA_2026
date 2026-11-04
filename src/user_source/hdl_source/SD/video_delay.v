//=============================================================================
// 文件名   : video_delay.v
// 模块名   : video_delay
// 功能     : 【视频数据延迟对齐器】
//            把 SDRAM 读出的 24bit 像素数据、以及配套的 hs/vs/de 时序信号，
//            统一延迟若干个像素时钟，使"数据"和"时序"在时间上严格对齐。
//
// 为什么要写这个模块 ?（这是本工程里最容易出 bug 的地方，务必看懂）
//   视频通路是一条长长的流水线：
//        SDRAM 发出读命令 ──► 等 10 拍 ──► 数据出来 ──► FIFO ──► …
//   而 hs/vs/de 是直接从 color_bar "现在"产生的。
//   如果两者不对齐，结果就是：屏幕上的画面整体向左/上偏、或者边缘出现一条彩边、
//   或者图片被"撕"成两半。
//   解决办法：把 hs/vs/de 也延后同样的拍数，让它们在时间轴上重新对齐。
//
//   本模块用了一个很经典的技巧 —— 用"移位寄存器"当延迟线：
//        de_d <= {de_d[19:0], de};    // 20 位移位寄存器，de 从最低位进、最高位出
//    de_d[18] 就是"18 拍之前的 de"，de_d[20] 就是"20 拍之前的 de"。
//   想延迟几拍，就取第几位，非常灵活、几乎不耗资源（就是一个寄存器）。
//
// 如何使用 ?
//   由顶层例化，输入 read_data（SDRAM 出来的数据）和 hs/vs/de（时序），
//   输出 hs_r/vs_r/de_r/vout_data（对齐后的结果），直接送 HDMI 编码。
//   如果发现画面位置不对，就调 de_d[18]/[19]/[20] 的下标数字，改延迟量。
//
// 为什么写三个不同的下标（18 / 19 / 20）？
//   因为这三条路径的延迟不一样：
//     - de_d[19] 取数据：SDRAM 数据出来比 de 早/晚一些，用 19 拍对齐
//     - de_d[18] 给 read_en：读使能要比数据更早 1 拍（提前一拍准备）
//     - de_d[20] 给 hs_r/vs_r/de_r：时序信号再多延一拍，正好包住数据窗口
//   这三个数字是调出来的，不是算出来的 —— 调试时就是靠示波器/看画面调的。
//=============================================================================

module video_delay
#(
	parameter DATA_WIDTH = 24                       // Video data one clock data width - 像素数据位宽，640x480 24bit = RGB888
)
(
	input                       video_clk,          // Video pixel clock - 像素时钟
	input                       rst,                // 复位
	output                      read_en,            // Read data enable - 读数据使能（提前一拍的 de）
	input[DATA_WIDTH - 1:0]     read_data,          // Read data - 从 SDRAM 读出的像素数据
	input                      hs,                 // horizontal synchronization - 输入行同步
	input                      vs,                 // vertical synchronization - 输入场同步
	input                      de,                 // video valid - 输入数据有效

	output                      hs_r,                 // horizontal synchronization - 输出行同步（已对齐）
	output                      vs_r,                 // vertical synchronization - 输出场同步（已对齐）
	output                      de_r,                 // video valid - 输出数据有效（已对齐）
	output[DATA_WIDTH - 1:0]    vout_data           // video data - 输出像素数据

);

// 21 位移位寄存器当延迟线，用"位下标"表示延迟拍数
reg [20:0] hs_d;
reg [20:0] vs_d;
reg [20:0] de_d;
reg[DATA_WIDTH - 1:0]  vout_data_r;   // 输出像素数据寄存器

assign read_en = de_d[18];       // 读使能取 18 拍前的 de（比数据早一拍）
assign hs_r = hs_d[20];          // 行同步取 20 拍前的
assign vs_r = vs_d[20];          // 场同步取 20 拍前的
assign de_r = de_d[20];          // 数据有效取 20 拍前的
assign vout_data = vout_data_r;

//-----------------------------------------------------------------------------
// 像素数据寄存：只在 de_d[19] 有效时采集 read_data
// 注意：这里用 de_d[19]（19 拍前的 de）作选通，
//       因为 SDRAM 数据到达的时刻比 de 晚 19 个时钟。
//       这就是"用时序信号当数据选通"的经典做法——相当于给数据配一个"时间戳"。
//-----------------------------------------------------------------------------
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(de_d[19])                // 数据到达窗口
		vout_data_r <= read_data;
	else
		vout_data_r <= {DATA_WIDTH{1'b0}};   // 窗口外清零，避免显示杂色
end

//-----------------------------------------------------------------------------
// 三条延迟线：每个时钟把新输入挤进最低位，老数据往高位推
//   {hs_d[19:0], hs} 意思是：取 hs_d 的低 20 位放在左边，
//   hs 放在最低位，拼成 21 位再赋回去 —— 整体左移一位效果。
//   所以 hs_d[0] 是上一拍的 hs，hs_d[20] 是 20 拍前的 hs。
//-----------------------------------------------------------------------------
always @(posedge video_clk or posedge rst)begin
	if(rst)begin
    	hs_d <= 20'b0;
        vs_d <= 20'b0;
        de_d <= 20'b0;
    end
	else begin
    	
    	hs_d <= {hs_d[19:0],hs};     // 行同步延迟线
        vs_d <= {vs_d[19:0],vs};     // 场同步延迟线
        de_d <= {de_d[19:0],de};     // 数据有效延迟线
    end
end
endmodule
