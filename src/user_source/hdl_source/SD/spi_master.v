
//==============================================================================
//  spi_master.v  —— SPI 主机硬件控制器（物理层）
//==============================================================================
//
//  【这个文件干什么的】
//  它是 SPI 协议的"硬件实现"，负责最底层的事情：产生时钟、把 8 位数据
//  一位一位地移出去、把线上的一位一位数据移进来。它完全不关心
//  对面接的是 SD 卡还是 Flash——是个通用的 SPI 收发器。
//
//  【SPI 是什么】
//  SPI = Serial Peripheral Interface，串行外设接口。四根线：
//    nCS   片选：拉低表示"我要跟你说话了"
//    DCLK  时钟：由主机产生，数据跟着时钟走
//    MOSI  主机输出、从机输入（Master Out Slave In）
//    MISO  主机输入、从机输出（Master In Slave Out）
//
//  【CPOL 和 CPHA 是什么（这两个参数决定了时钟的"相位"）】
//  SPI 有 4 种模式，由两个参数组合而成：
//    CPOL = Clock Polarity（时钟极性）
//      0 → 空闲时时钟是低电平
//      1 → 空闲时时钟是高电平     ← 本工程用 1
//    CPHA = Clock Phase（时钟相位）
//      0 → 在第一个边沿采样（输出也在第一个边沿）
//      1 → 在第二个边沿采样       ← 本工程用 1
//  CPOL=1 + CPHA=1 就是常说的"SPI 模式 3"。
//  SD 卡在 SPI 模式下要求模式 0 或模式 3。
//
//  【时钟分频的计算】★ 2026-11-02：分两档
//
//    div > 0（初始化 / 慢速）：
//        SPI 时钟 = clk / ((clk_div + 2) × 2)
//        每个边沿 = DCLK_IDLE 等 (div+1) 拍 + DCLK_EDGE 1 拍
//        例：50MHz、div=248 → 50M/(250×2) = 100kHz（SD 规范要求的初始化时钟）
//
//    div = 0（正常读写 / 高速）：
//        每个边沿 = 1 拍（EDGE 直接回 EDGE，DCLK 每个时钟沿翻一次）
//        SPI 时钟 = clk / 2
//        例：50MHz → 25MHz（= 参考例程 100MHz/div=0 的同一速率）
//
//  【为什么以前 div=0 只有 12.5MHz】
//  旧状态机每个边沿固定要两拍（DCLK_IDLE 1 拍 + DCLK_EDGE 1 拍），
//  50MHz 下 1 bit = 4 拍 = 80ns ⇒ 12.5MHz。高速档把 DCLK_IDLE 整个跳过，
//  1 bit = 2 拍 = 40ns ⇒ 25MHz，字节级吞吐 1.389 → 2.63 MB/s。
//
//  【状态机流程（一个字节的完整过程）】
//    IDLE           ← 等 wr_req
//      ↓
//    ┌ div>0：DCLK_IDLE ← 等半个时钟周期（clk_cnt 数到 clk_div）
//    └ div=0：直接去 DCLK_EDGE（省掉这 1 拍）
//      ↓
//    DCLK_EDGE      ← 翻转 DCLK，移出一位、移入一位，计数 +1
//      ↑______________|
//      │  div>0：每次回 DCLK_IDLE 等半周期
//      └  div=0：EDGE→EDGE，一拍一个边沿（重复 16 次）
//      ↓ (数到 15)
//    div>0：LAST_HALF_CYCLE ← 最后半个周期
//    div=0：直接去 ACK（DCLK 在 ACK/ACK_WAIT/IDLE 三拍里保持高，
//           对卡来说只是 tCH 变长，不影响建立/保持）
//      ↓
//    ACK            ← 拉高 wr_ack 一拍，告诉上层"字节发完了"
//      ↓
//    ACK_WAIT       ← ★必须保留★ 上层就是在这一拍把下一个 send_data 摆好的
//      ↓              （跳过它，S_CMD 会把上一个命令字节重发一遍）
//    IDLE
//
//==============================================================================
module spi_master
(
	input                       sys_clk,   // 系统时钟
	input                       rst,       // 复位，高有效
	output                      nCS,       // 片选输出
	output                      DCLK,      // SPI 时钟输出
	output                      MOSI,      // 主机输出
	input                       MISO,      // 主机输入
	input                       CPOL,      // 时钟极性（0/1）
	input                       CPHA,      // 时钟相位（0/1）
	input                       nCS_ctrl,  // 片选控制（来自命令层）
	input[15:0]                 clk_div,   // 时钟分频值
	input                       wr_req,    // 请求发送一个字节
	output                      wr_ack,    // 字节发送完成应答
	input[7:0]                  data_in,   // 要发送的字节
	output[7:0]                 data_out   // 接收到的字节
);

//==============================================================================
//  状态编码
//==============================================================================
localparam                   IDLE            = 0;   // 空闲
localparam                   DCLK_EDGE       = 1;   // 时钟边沿（翻转 DCLK + 移位）
localparam                   DCLK_IDLE       = 2;   // 等待半个时钟周期
localparam                   ACK             = 3;   // 应答
localparam                   LAST_HALF_CYCLE = 4;   // 最后半个周期
localparam                   ACK_WAIT        = 5;   // 等一拍再回空闲

reg                          DCLK_reg;      // DCLK 寄存输出
reg[7:0]                     MOSI_shift;    // 发送移位寄存器
reg[7:0]                     MISO_shift;    // 接收移位寄存器
reg[2:0]                     state;
reg[2:0]                     next_state;    // 次态（用于两段式状态机）
reg [15:0]                   clk_cnt;       // 半周期计时器
reg[4:0]                     clk_edge_cnt;  // 时钟边沿计数器（0~15）

//==============================================================================
//  组合逻辑输出
//==============================================================================
assign MOSI = MOSI_shift[7];        // 总是输出移位寄存器的最高位
assign DCLK = DCLK_reg;
assign data_out = MISO_shift;       // 接收完的字节直接输出
assign wr_ack = (state == ACK);     // 到达 ACK 状态 = 字节发完
assign nCS = nCS_ctrl;              // 片选直接透传

//==============================================================================
//  【两段式状态机：第一段 - 状态寄存】
//==============================================================================
// 【什么是两段式状态机】把"次态计算"和"状态更新"分开写：
//   第一段（这里）：时序逻辑，只在时钟边沿把 next_state 复制到 state
//   第二段（下面）：组合逻辑，根据当前 state 和输入算出 next_state
// 【为什么要分开】这样次态逻辑全是组合逻辑，容易检查和仿真；
// 而且避免了"在时序块里既读又写同一个信号"的常见错误。
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		state <= IDLE;
	else
		state <= next_state;
end

//==============================================================================
//  【两段式状态机：第二段 - 次态计算】
//==============================================================================
// ★ 2026-11-02 高速档判据：div=0 时"一拍一个边沿"
//   【为什么做成一根 wire】次态逻辑里只用一次 2:1，综合就是几个 LUT，
//   不进任何寄存器，也不改变慢速档的时序。
//   【div 在字节中途不会变】sd_card_cmd 只在 S_IDLE / S_WAIT 里改 clk_div，
//   那两个状态一定在字节边界之后，所以这里可以安全地组合使用。
wire fast = (clk_div == 16'd0);

always@(*)
begin
	case(state)
		IDLE:
			if(wr_req == 1'b1)
				next_state <= fast ? DCLK_EDGE : DCLK_IDLE;  // 高速档直插边沿
			else
				next_state <= IDLE;         // 否则继续等

		DCLK_IDLE:
			// 只有 div>0 才会进来：等半个 SPI 时钟周期
			if(clk_cnt == clk_div)
				next_state <= DCLK_EDGE;    // 时间到，去产生边沿
			else
				next_state <= DCLK_IDLE;

		DCLK_EDGE:
			// 一个字节需要 16 个边沿（8 位 × 2）
			if(clk_edge_cnt == 5'd15)
				next_state <= fast ? ACK : LAST_HALF_CYCLE;   // 最后一位发完了
			else
				next_state <= fast ? DCLK_EDGE : DCLK_IDLE;   // 高速档：下一拍继续翻

		LAST_HALF_CYCLE:
			// 最后半个周期，等时钟走完
			if(clk_cnt == clk_div)
				next_state <= ACK;
			else
				next_state <= LAST_HALF_CYCLE;

		ACK:
			next_state <= ACK_WAIT;     // 拉高 wr_ack 一拍

		ACK_WAIT:
			next_state <= IDLE;         // 再等一拍，让上层撤销 wr_req / 摆好下一个字节

		default:
			next_state <= IDLE;
	endcase
end

//==============================================================================
//  DCLK 时钟生成
//==============================================================================
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		DCLK_reg <= 1'b0;
	else if(state == IDLE)
		DCLK_reg <= CPOL;                   // 空闲时回到 CPOL 电平
	else if(state == DCLK_EDGE)
		DCLK_reg <= ~DCLK_reg;              // 每个边沿状态翻转一次
	// 【注意】这里没有 else 分支，意味着其他状态保持原值。
	// 在时序 always 块里，不写 else 就是"保持"（综合出来是带使能的触发器）。
end

//==============================================================================
//  半周期计时器
//==============================================================================
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		clk_cnt <= 16'd0;
	else if(state == DCLK_IDLE || state == LAST_HALF_CYCLE)
		clk_cnt <= clk_cnt + 16'd1;         // 这两个状态在计时，递增
	else
		clk_cnt <= 16'd0;                   // 其他状态归零
end

//==============================================================================
//  边沿计数器
//==============================================================================
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		clk_edge_cnt <= 5'd0;
	else if(state == DCLK_EDGE)
		clk_edge_cnt <= clk_edge_cnt + 5'd1;  // 每个边沿 +1
	else if(state == IDLE)
		clk_edge_cnt <= 5'd0;                 // 回到空闲就清零，准备下一个字节
end

//==============================================================================
//  ★ MOSI 数据输出（移位发出）★
//==============================================================================
// 【移位方向】MOSI 总是取 MOSI_shift[7]（最高位）输出，
// 然后每发一位就把整个寄存器左移一位，所以是"高位先出"（MSB First）。
// 这是 SPI 和 SD 卡的标准做法。
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		MOSI_shift <= 8'd0;
	else if(state == IDLE && wr_req)
		// 空闲状态下收到发送请求 → 把要发的数据装进移位寄存器
		MOSI_shift <= data_in;
	else if(state == DCLK_EDGE)
		// 移位。具体在哪个边沿移位由 CPHA 决定：
		if(CPHA == 1'b0 && clk_edge_cnt[0] == 1'b1)
			// CPHA=0：在第 2、4、6... 个边沿（奇数序号）移位
			MOSI_shift <= {MOSI_shift[6:0],MOSI_shift[7]};
		else if(CPHA == 1'b1 && (clk_edge_cnt != 5'd0 && clk_edge_cnt[0] == 1'b0))
			// CPHA=1：在第 1、3、5... 个边沿（偶数序号）移位
			// 【clk_edge_cnt[0] == 0 表示偶数】用最低位判断奇偶是个小技巧
			// 【为什么要 clk_edge_cnt != 0】因为第 0 个边沿时数据刚装上，
			// 还没发出去，不能立刻移位。
			MOSI_shift <= {MOSI_shift[6:0],MOSI_shift[7]};
end

//==============================================================================
//  ★ MISO 数据输入（移位接收）★
//==============================================================================
always@(posedge sys_clk or posedge rst)
begin
	if(rst)
		MISO_shift <= 8'd0;
	else if(state == IDLE && wr_req)
		MISO_shift <= 8'h00;        // 新字节开始，清空接收寄存器
	else if(state == DCLK_EDGE)
		// 采样时机和发送错开半个周期（这是 SPI 的标准要求：
		// 一个边沿发、另一个边沿收），所以这里的 CPHA 判断条件是反过来的。
		if(CPHA == 1'b0 && clk_edge_cnt[0] == 1'b0)
			MISO_shift <= {MISO_shift[6:0],MISO};
		else if(CPHA == 1'b1 && (clk_edge_cnt[0] == 1'b1))
			MISO_shift <= {MISO_shift[6:0],MISO};
end
endmodule
