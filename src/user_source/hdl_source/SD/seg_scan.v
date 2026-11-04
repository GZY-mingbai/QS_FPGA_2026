//=============================================================================
// 文件名   : seg_scan.v
// 模块名   : seg_scan
// 功能     : 【6 位数码管动态扫描显示驱动】
//            轮流点亮 6 位数码管的每一位，每位的段码来自对应的输入端口。
//
// 什么是"动态扫描"？（初学者必懂）
//   如果有 6 个数码管，要让它们显示不同的内容，有两种做法：
//     ① 静态驱动：每位都配一套独立的段选线 —— 引脚太多（6×8 = 48 根），浪费。
//     ② 动态扫描：所有数码管共用同一组段选线（8 根），再用 6 根"位选线"（seg_sel）
//                 决定"现在点亮第几位"。快速轮流切换，利用人眼视觉暂留，
//                 看起来就像 6 位同时亮着。本模块用的就是 ②。
//   —— 所以你会看到 seg_data 是"共享的"，seg_sel 才是"指哪打哪"。
//
// 本模块的两级逻辑：
//   第一级：一个定时器 scan_timer，每数满 SCAN_COUNT 就产生一次"切换位"的动作，
//           让 scan_sel 在 0~5 之间循环。
//   第二级：根据 scan_sel，输出对应的位选 seg_sel 和该位的段码 seg_data。
//
// 扫描频率怎么定？
//   SCAN_COUNT = CLK_FREQ / (SCAN_FREQ * 6) - 1
//   含义：一秒钟要完整扫完 SCAN_FREQ 轮（每轮扫 6 位），
//         所以每切换一位需要 CLK_FREQ/(SCAN_FREQ*6) 个时钟。
//   本例 50MHz 时钟、200Hz 扫描，得到 SCAN_COUNT = 41。
//   为什么是 200Hz？太低会看到闪烁，太高数码管亮度会下降（每次点亮时间变短）。
//   一般取 100Hz~1000Hz 比较合适。
//
// 如何使用 ?
//   6 个输入 seg_data_0 ~ seg_data_5 分别是要显示的 6 位内容（段码，最高位=小数点）。
//   输出 seg_sel 接数码管位选、seg_data 接段选。
//   注意复位是低有效（rst_n），和工程里其它模块（高有效）不同，接入时要留意。
//=============================================================================

module seg_scan(
	input           clk,             // 系统时钟
	input           rst_n,           // 复位，★低电平有效★（注意和其它模块相反）
	output reg[5:0] seg_sel,      //digital led chip select - 位选信号（选第几个数码管）
	output reg[7:0] seg_data,     //eight segment digital tube output,MSB is the decimal point - 段码，最高位是小数点
	input[7:0]      seg_data_0,   // 第 0 位要显示的内容
	input[7:0]      seg_data_1,   // 第 1 位
	input[7:0]      seg_data_2,   // 第 2 位
	input[7:0]      seg_data_3,   // 第 3 位
	input[7:0]      seg_data_4,   // 第 4 位
	input[7:0]      seg_data_5    // 第 5 位
);
parameter SCAN_FREQ = 200;     //scan frequency - 扫描频率 200Hz
parameter CLK_FREQ = 50000000; //clock frequency - 输入时钟 50MHz

// 每切换一位需要的时钟数（减 1 是因为计数器从 0 开始数）
parameter SCAN_COUNT = CLK_FREQ /(SCAN_FREQ * 6) - 1;

// ★ 2026-10-27 P1-L8：32 位 → 16 位
//   SCAN_COUNT = CLK_FREQ/(SCAN_FREQ*6)-1 = 50_000_000/(200*6)-1 = 41666
//   ⇒ 16 位（0..65535）足够，原来的 32 位白占 16 个 FF + 加法器高 16 位。
//   ⚠ 改 CLK_FREQ / SCAN_FREQ 使 SCAN_COUNT ≥ 65536 时必须改回宽位。
reg[15:0] scan_timer;  //scan time counter - 扫描定时器
reg[3:0] scan_sel;     //Scan select counter - 当前扫描到第几位

//-----------------------------------------------------------------------------
// 扫描定时器：数满 SCAN_COUNT 就把 scan_sel 加 1，到 5 之后回到 0
//-----------------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(rst_n == 1'b0)          // 低电平复位
	begin
		scan_timer <= 16'd0;
		scan_sel <= 4'd0;
	end
	else if(scan_timer >= SCAN_COUNT)
	begin
		scan_timer <= 16'd0;   // 计数器清零，准备下一轮
		if(scan_sel == 4'd5)   // 已经扫到最后一位
			scan_sel <= 4'd0;  // 回到第一位
		else
			scan_sel <= scan_sel + 4'd1;
	end
	else
		begin
			scan_timer <= scan_timer + 16'd1;
		end
end

//-----------------------------------------------------------------------------
// 输出级：按 scan_sel 选通对应的数码管
//   seg_sel 是"独热码"（one-hot），只有一个 bit 是 0，其余为 1。
//   因为数码管位选也是低有效：某位为 0 表示"选中这个数码管"。
//   例如 6'b11_1110 -> 最低位为 0 -> 选中最右边的第 0 位。
//-----------------------------------------------------------------------------
always@(posedge clk or negedge rst_n)
begin
	if(rst_n == 1'b0)
	begin
		seg_sel <= 6'b111111;   // 全 1 = 全部熄灭
		seg_data <= 8'hff;      // 段码全 1 = 全灭
	end
	else
	begin
		case(scan_sel)
			//first digital led - 第 0 位
			4'd0:
			begin
				seg_sel <= 6'b11_1110;
				seg_data <= seg_data_0;
			end
			//second digital led - 第 1 位
			4'd1:
			begin
				seg_sel <= 6'b11_1101;
				seg_data <= seg_data_1;
			end
			//...（原理同上，逐个往下选）
			4'd2:
			begin
				seg_sel <= 6'b11_1011;
				seg_data <= seg_data_2;
			end
			4'd3:
			begin
				seg_sel <= 6'b11_0111;
				seg_data <= seg_data_3;
			end
			4'd4:
			begin
				seg_sel <= 6'b10_1111;
				seg_data <= seg_data_4;
			end
			4'd5:
			begin
				seg_sel <= 6'b01_1111;
				seg_data <= seg_data_5;
			end
			default:                // 防呆分支：scan_sel 不该出现 6~15
			begin
				seg_sel <= 6'b11_1111;
				seg_data <= 8'hff;
			end
		endcase
	end
end

endmodule
