//=============================================================================
// 文件名   : video_timing_data.v
// 模块名   : video_timing_data
// 功能     : 【视频时序发生器 + 帧读请求发生器】
//            1) 例化 color_bar 产生一套完整的视频时序（hs / vs / de）；
//            2) 根据 vs 的边沿，产生"请给我读一帧数据"的请求信号 read_req；
//            3) 把 hs/vs/de 往后延迟 2 拍再输出，用来和 SDRAM 出来的数据对齐。
//
// 它在整个工程里的位置 :
//      video_clk 域
//      ┌────────────────┐   hs/vs/de    ┌──────────────┐
//      │  color_bar     ├──────────────►│ video_delay  │──► 最终 hs/vs/de (给HDMI)
//      │ (只当时序源)   │               └──────────────┘
//      └────────────────┘
//              │ vs 边沿
//              ▼
//         read_req ──► 去 sd_card_bmp，请求把下一帧从SD卡读到SDRAM
//
// 为什么要写这个模块 ?
//   把"产生时序"和"决定何时读数据"这两件相关但不相同的事放在一起管，
//   上层的 top 模块就不用关心这些细节了。而且 color_bar 本来就能独立出图，
//   这里只是借用它的时序信号。
//
// 如何使用 ?
//   由 top 层直接例化，接 video_clk（像素时钟）和视频域复位。
//   read_req / read_req_ack 是一对"请求-应答"握手信号，
//   接到 sd_card_bmp 的对应端口上。
//
// 为什么这样写 ?  （初学者重点）
//   1) 为什么要延迟 2 拍？因为 SDRAM 读数据有固定的流水线延迟（见 frame_fifo_read.v
//      里 rd_delay 计数），如果时序信号不跟着延后，就会出现"图上来了但位置错乱"，
//      俗称"画面整体偏移/撕裂"。延迟量要和数据通路的延迟精确匹配。
//   2) 用 vs 的边沿产生 read_req：一帧显示完、下一帧刚开始时（场消隐期间）去搬数据，
//      这样不会打断正在显示的画面。源码里注释写"上升沿下降沿都行"，
//      是因为它用的是"v_cnt 归零那一瞬间"这个事件，取的是沿而不是电平。
//=============================================================================

module video_timing_data
#(
	parameter DATA_WIDTH = 16                       // Video data one clock data width - 视频数据位宽（此模块实际未使用，保留接口）
)
(
	input                       video_clk,          // Video pixel clock - 视频像素时钟
	input                       rst,                // 复位，高有效
	output reg                  read_req,           // Start reading a frame of data - 请求读取一帧（去 sd_card_bmp）
	input                       read_req_ack,       // Read request response - 请求已被接受（来自 sd_card_bmp）

	//--------------------------------------------------------------------------
	//  ★ 场同步换页（修"边写边显示"的滚动横缝 / 撕裂）★
	//--------------------------------------------------------------------------
	//  写口 31.25M 字/s ≈ 102fps，显示 60fps ⇒ 写比读快 1.7 倍，写指针会转回来
	//  重写"屏幕正在读"的那块 ⇒ 屏幕上一条横缝从下往上滚动（缝两侧是同一动画
	//  的两个不同时刻）。修法：显示指针只在**自己的场边界**（vs 下降沿）切换，
	//  换完再回告写侧"你可以写另一块了"（翻转电平，跨时钟域最稳）。
	//  这些信号都由 top 同步好再送进来 / 送出去。
	//--------------------------------------------------------------------------
	input      [1:0]            buf_commit_idx,     // 写侧"已提交可显示"的块号（已同步到本时钟域）
	input                       buf_commit_tgl,     // 写侧每提交一帧翻转一次（已同步到本时钟域）
	output reg [1:0]            disp_rd_idx,        // 本模块真正在读的块号（只在 vs 下降沿换）
	output reg                  disp_adopt_tgl,     // "已采纳一帧新的"事件（翻转电平，回给写侧）
	//output                      read_en,            // Read data enable   （已注释，本版本不用）
	//input[DATA_WIDTH - 1:0]     read_data,          // Read data          （已注释，本版本不用）
	output                      hs,                 // horizontal synchronization - 行同步
	output                      vs,                 // vertical synchronization - 场同步
	output                      de                  // video valid - 数据有效
	//output[DATA_WIDTH - 1:0]    vout_data           // video data         （已注释，本版本不用）
);

// color_bar 输出的原始时序（组合 wire）
wire                   video_hs;
wire                   video_vs;
wire                   video_de;

//delay video_hs video_vs  video_de 2 clock cycles
// 两级延迟寄存器：d0 是第一拍，d1 是第二拍
reg                    video_hs_d0;
reg                    video_vs_d0;
reg                    video_de_d0;
reg                    video_hs_d1;
reg                    video_vs_d1;
reg                    video_de_d1;

//reg[DATA_WIDTH - 1:0]  vout_data_r;
//assign read_en = video_de;
// 输出经过 2 拍延迟的时序信号，与 SDRAM 数据延迟对齐
assign hs = video_hs_d1;
assign vs = video_vs_d1;
assign de = video_de_d1;
//assign vout_data = vout_data_r;

//-----------------------------------------------------------------------------
// 两级延迟移位：video_xx -> d0 -> d1
// 写在一起"批量移位"是 Verilog 常见写法，综合器会拆成两组独立触发器。
//-----------------------------------------------------------------------------
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		video_hs_d0 <= 1'b0;
		video_vs_d0 <= 1'b0;
		video_de_d0 <= 1'b0;
	end
	else
	begin
		//delay video_hs video_vs  video_de 2 clock cycles
		video_hs_d0 <= video_hs;      // 第一拍
		video_vs_d0 <= video_vs;
		video_de_d0 <= video_de;
		video_hs_d1 <= video_hs_d0;   // 第二拍：把 d0 再延一拍
		video_vs_d1 <= video_vs_d0;
		video_de_d1 <= video_de_d0;		
	end
end

/*
// 原版本把 SDRAM 读数据打一拍再输出，本工程改用 video_delay.v 统一处理，
// 这段就注释掉保留了。
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vout_data_r <= {DATA_WIDTH{1'b0}};
	else if(video_de_d0)
		vout_data_r <= read_data;
	else
		vout_data_r <= {DATA_WIDTH{1'b0}};
end
*/

//-----------------------------------------------------------------------------
//  ★ 场同步换页：显示指针只在 vs 下降沿（场边界）切换 ★
//-----------------------------------------------------------------------------
//  【为什么这么做】原来显示指针（disp_buf_idx）由写侧在"整帧写完"时直接切换。
//  但写口 31.25M 字/s（一帧 9.83ms ≈ 102fps）比显示（60fps）快 1.7 倍，写指针
//  转一圈又回来重写"屏幕正在读"的那块 ⇒ 屏幕上一条横缝从下往上滚动，缝两侧是
//  同一动画的两个不同时刻（图形在两处同时出现）—— 这就是撕裂。
//  现在：写侧提交后只翻转 buf_commit_tgl；本模块在**自己的场边界**才把
//  disp_rd_idx 切到新块，并翻转 disp_adopt_tgl 通知写侧"可以写另一块了"。
//  ⇒ 显示永远在读一块"完整且不再被写"的缓冲。
//
//  【为什么 disp_rd_idx 要比 read_req 早一拍更新】
//  read_addr_index 在 frame_fifo_read 里要打 2 拍同步后才在 S_ACK 被锁存；
//  如果同一拍才变，极端相位下读引擎可能锁到旧块号。早一拍（40ns）就够稳。
//-----------------------------------------------------------------------------
wire vs_fall = video_vs_d0 & ~video_vs;   // vs 下降沿（原来 read_req 的触发条件）
reg  vs_fall_d1;                          // 打一拍：read_req 用它，保证比换页晚一拍

always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		vs_fall_d1 <= 1'b0;
	else
		vs_fall_d1 <= vs_fall;
end

reg disp_adopted_tgl;   // 本模块"已经采纳到"的提交号，用来识别"有新帧可换"

always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		disp_rd_idx      <= 2'd0;
		disp_adopt_tgl   <= 1'b0;
		disp_adopted_tgl <= 1'b0;
	end
	else if(vs_fall && (buf_commit_tgl != disp_adopted_tgl))
	begin
		disp_rd_idx      <= buf_commit_idx;    // ★ 换页（场边界）
		disp_adopted_tgl <= buf_commit_tgl;    // 记下已采纳到哪一帧
		disp_adopt_tgl   <= ~disp_adopt_tgl;   // 回告写侧：可以开下一帧了
	end
end

// 帧读请求发生器
// 逻辑：
//   vs 出现下降沿（video_vs_d0=1 且 video_vs=0，即从高变低的那一拍）
//       -> 拉高 read_req，告诉 sd_card_bmp "准备搬下一帧"
//   对方接受后回 read_req_ack=1
//       -> 拉低 read_req，一次握手完成
// 这就是最简单的"请求-应答"握手（handshake），
// 比固定延时更可靠：不管对方多慢，只要不回 ack 我就不撤销请求。
//-----------------------------------------------------------------------------
always@(posedge video_clk or posedge rst)
begin
	if(rst == 1'b1)
		read_req <= 1'b0;
	else if(vs_fall_d1)              // ★ 比换页晚一拍：让 disp_rd_idx 先稳定
		                             //vertical synchronization edge (the rising or falling edges are OK)
		                             // 检测 vs 的下降沿：上一拍是 1，这一拍是 0
		read_req <= 1'b1;
	else if(read_req_ack)            // 对方已接受请求，撤销
		read_req <= 1'b0;
end

//-----------------------------------------------------------------------------
// 例化彩条发生器，只取它的 hs/vs/de 作为时序基准
// （rgb_r/g/b 端口留空不接，因为本工程真正要显示的是 SD 卡图片）
//-----------------------------------------------------------------------------
color_bar color_bar_m0(
	.clk(video_clk),
	.rst(rst),
	.hs(video_hs),
	.vs(video_vs),
	.de(video_de),
	.rgb_r(),      // 悬空：颜色数据不使用
	.rgb_g(),
	.rgb_b()
);
endmodule
