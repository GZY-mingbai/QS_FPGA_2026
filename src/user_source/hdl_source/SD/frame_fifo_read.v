`timescale 1ns/1ps
//==============================================================================
//  frame_fifo_read.v  —— 读方向搬运状态机（SDRAM 突发读 → FIFO → 视频端）
//==============================================================================
//
//  【这个文件干什么的】
//  和 frame_fifo_write 正好相反的方向：
//  从 SDRAM 里以"突发"的方式把像素数据一批批读出来，
//  压进读 FIFO，视频端（25MHz）再慢慢一个一个取走显示。
//
//  【为什么视频端不能直接读 SDRAM】
//  因为 SDRAM 的读是"突发"式的——一次给你 256 个数据，速度很快，
//  而且是 125MHz 时钟域的。视频端只需要 25MHz，一拍取一个像素。
//  直接在两边对接，视频端根本来不及取，数据就丢了。
//  所以中间必须要有个 FIFO 做"缓冲池"，让快的一边先灌满，慢的一边慢慢喝。
//
//  【为什么读的时候要检查 App_wr_busy】
//  SDRAM 是单口的，同一时刻只能读或只能写。
//  本模块在发起突发读之前会检查"写方向是否在忙"，
//  如果写正在忙就先等等——这就是"写优先于读"的实现。
//
//  【rd_delay 这个 10 拍的延时是干什么的】
//  SDRAM 控制器收到读命令后，需要若干个时钟才能把数据准备好
//  （包括行激活 tRCD、列延迟 tCL 等时序）。本模块用一个延时计数器
//  等 10 拍，确认"数据应该已经全部回来了"，才认为这次突发真正结束。
//  这样比逐个数据计数更简单，是 SDRAM 控制器的常见简化写法。
//
//==============================================================================
module frame_fifo_read
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter ADDR_BITS              = 21,
	parameter BURST_BITS             = 9,
	parameter FIFO_DEPTH             = 512,     // FIFO 深度（用于判断剩余空间）
	parameter BURST_SIZE             = 128,     // 一次突发读多少个
	// ★ 2026-10-17（第二次修订）：INFLIGHT_MAX 由 8 加深到 **16** ★
	//  【为什么】发现 rfifo IP 的水位读数是 **9 位差值**（ADDR_WIDTH=9）：
	//  512 深的 FIFO 水位打到 512 时，wrusedw/rdusedw 回绕成 0 ——
	//  发突发判据 wrusedw_r < 阈值 会把"满"看成"全空"而继续发突发，
	//  写口（we=Sdr_rd_en，无满保护）把落在满 FIFO 上的数据全部丢掉
	//  → 大幅横移。这解释了上上轮的"整屏左右抖动"（burst 256 时阈值 248，
	//  峰值 248+256+在途 = 512+，正好踩线）。
	//  burst 128 + INFLIGHT_MAX=8 时稳态峰值 ≈ 376+128+在途(≤8) = 512 ——
	//  仍然贴线！取 16：阈值 512-128-16 = 368，峰值 ≤ 368+128+8 = 504，
	//  正常工况永远碰不到 512，回绕地雷排除。
	//  【欠载侧代价】余量 368 字 ≈ 14.7µs，与 376 几乎无差。
	//  （另外：数码管第 2 位已改为 FIFO 自身的 empty/full 粘滞标志，
	//   不再用会回绕的 rdusedw —— 见 top_tf_hdmi_audio.v。）
	parameter INFLIGHT_MAX           = 16
)
(
	input                            rst,
	input                            mem_clk,      // SDRAM 侧时钟 125MHz
	input							 Sdr_init_done,
	input							 Sdr_init_ref_vld,
    input							 Sdr_busy,
    input							 Sdr_rd_en,    // SDRAM 读数据有效（每个数据来一拍）
	input							 App_wr_busy,  // 写方向是否忙（写优先！）
    // ★ 读口仲裁器放行（0 = 暂停本次突发，不是丢弃）★
    //   加了 SDRAM 纹理读之后，读口要和图片图层共用。仲裁器在服务纹理期间
    //   把 rd_ready 拉低，本模块的 App_rd_en 被挡住 —— burst_cnt 不前进、
    //   rd_delay 也不跑，所以突发长度一点都不丢，只是被拉长了。
    input                            rd_ready,
    output							 O_rd_busy,    // 本模块是否忙

	output 							 App_rd_en,    // 读使能（每读一个数据拉高一拍，已被 rd_ready 门控）
	output 							 App_rd_want,  // ★ 未门控的"我想读"（给仲裁器判优先级用）
	output  [ADDR_BITS - 1:0]		 App_rd_addr,  // 读地址

    // ---- 上层握手接口 ----
	input                            read_req,         // 读请求
	output reg                       read_req_ack,     // 应答
	output                           read_finish,      // 整帧读完
	input[ADDR_BITS - 1:0]           read_addr_0,      // 缓冲区0 基地址
	input[ADDR_BITS - 1:0]           read_addr_1,      // 缓冲区1 基地址
	input[ADDR_BITS - 1:0]           read_addr_2,      // 缓冲区2
	input[ADDR_BITS - 1:0]           read_addr_3,      // 缓冲区3
	input[1:0]                       read_addr_index,  // 选哪块（双缓冲关键）
	input[ADDR_BITS - 1:0]           read_len,         // 一帧多少个【像素】
	input                            fmt565,           // ★ build FB16a：1 = 565（字数 = 像素数/2）
	output reg                       fifo_aclr,        // 通知 FIFO 清零
	input[9:0]                      wrusedw,          // FIFO 里已有多少数据
	// ★ 100M-23：读 FIFO 的"真满"标志。wrusedw 是 9/10 位，**512 时回绕成 0**，
	//   所以 `wrusedw < 阈值` 在"真满"时反而成立 ⇒ 状态机判定"空间够" ⇒
	//   再发 128 条 ⇒ 落到满 FIFO 上的数据被**静默丢掉**（写口 we=Sdr_rd_en
	//   无满保护，见文件头 §39-49 的实测记录）⇒ 丢一个显示像素 = 该行余下部分
	//   横向错一位 ⇒ 画面上是竖线状的错位缝。full_flag 是 IP 自己的真满标志，
	//   不回绕，拿它做最后一道闸门最稳。
	input                           wr_full

);

localparam ONE                       = 256'd1;
localparam ZERO                      = 256'd0;

//==============================================================================
//  状态编码
//==============================================================================
localparam S_IDLE                    = 0;    // 空闲，等读请求
localparam S_ACK                     = 1;    // 应答：清 FIFO、锁地址、锁长度
localparam S_CHECK_FIFO              = 2;    // 检查 FIFO 空间够不够
localparam S_READ_BURST              = 3;    // 正在发起突发读
localparam S_READ_BURST_END          = 4;    // 一次突发结束
localparam S_END                     = 5;    // 整帧读完

//==============================================================================
//  内部寄存器
//==============================================================================
reg                                  read_req_d0;      // 跨时钟域同步第 1 级
reg                                  read_req_d1;      // 第 2 级
reg                                  read_req_d2;      // 第 3 级（最终使用）
reg[ADDR_BITS - 1:0]                 read_len_d0;
reg[ADDR_BITS - 1:0]                 read_len_d1;
reg[ADDR_BITS - 1:0]                 read_len_latch;   // 锁存的帧长度
reg[ADDR_BITS - 1:0]                 read_cnt;         // 已读出的数据总数
reg[3:0]                             state;
reg[1:0]                             read_addr_index_d0;
reg[1:0]                             read_addr_index_d1;
reg [ADDR_BITS - 1:0]	 App_rd_addr_r;

reg [BURST_BITS - 1:0]				burst_cnt;       // 本次突发发了多少个读命令
wire								rd_burst_finish;
reg App_rd_en_r;      // 突发进行中标志
reg App_rd_en_d0;     // 输出使能

// ★ 时序修复（2026-10-15，"会移动的横条纹"）：
//   S_CHECK_FIFO 里的空间检查 wrusedw < (FIFO_DEPTH-BURST_SIZE) 原来直接用
//   FIFO IP 的格雷码转二进制指针（secondary_addr_o）做 10 位比较，与写侧
//   O_wr_busy 串联后是 App_rd_en_r 违规路径的末端。这里打一拍再比较：
//   【为什么晚一拍安全】S_CHECK_FIFO 期间读 FIFO 没有数据流入（Sdr_rd_en
//   只在突发数据返回时才有），只有显示侧在往外抽——水位只降不升，
//   晚一拍看到的值只会更保守（空间只会更多），绝不会溢出。
//   每次突发多花 1 拍决策，吞吐无感。
reg [9:0] wrusedw_r;  // 读 FIFO 水位（打一拍后的版本）

always @(posedge mem_clk or posedge rst) begin
    if (rst)
        wrusedw_r <= 10'd0;
    else
        wrusedw_r <= wrusedw;
end

wire rd_vld;              // 突发发完了（内部中间信号）
reg [3:0] rd_delay;       // 等数据回来的延时计数器

assign App_rd_addr = {App_rd_addr_r[ADDR_BITS - 1:0]};

// 突发命令已经全部发出（发了 256 个）
assign rd_vld = (state == S_READ_BURST && burst_cnt >= BURST_SIZE);

// 本模块忙标志
assign O_rd_busy = (state == S_READ_BURST);

// ★ 突发真正结束 = 命令发完了 且 数据延时够了 10 拍
// burst_cnt 代表"已发出的读指令数"，但 SDRAM 数据要等若干拍才回来，
// 所以还要再等 rd_delay 数到 10，才能确认数据都进 FIFO 了。
assign rd_burst_finish = (rd_vld && rd_delay == 4'd10);

assign read_finish = (state == S_END) ? 1'b1 : 1'b0;
assign App_rd_want = App_rd_en_d0;              // 未门控：给仲裁器
assign App_rd_en   = App_rd_en_d0 & rd_ready;   // 门控后：真正发给 SDRAM

//==============================================================================
//  跨时钟域同步：read_req / read_len / read_addr_index
//==============================================================================
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		read_req_d0    <=  1'b0;
		read_req_d1    <=  1'b0;
		read_req_d2    <=  1'b0;
		read_len_d0    <=  ZERO[ADDR_BITS - 1:0];
		read_len_d1    <=  ZERO[ADDR_BITS - 1:0];
		read_addr_index_d0 <= 2'b00;
		read_addr_index_d1 <= 2'b00;
	end
	else
	begin
		read_req_d0    <=  read_req;
		read_req_d1    <=  read_req_d0;
		read_req_d2    <=  read_req_d1;
		read_len_d0    <=  read_len;
		read_len_d1    <=  read_len_d0;
		read_addr_index_d0 <= read_addr_index;
		read_addr_index_d1 <= read_addr_index_d0;
	end
end

//==============================================================================
//  数据回来延时计数器
//==============================================================================
// 【为什么是 10】SDRAM 从收到读命令到数据出现在总线上，大约需要
// 10 个 125MHz 时钟周期（行激活 + CAS 延迟）。数 10 拍是个经验值。
// 重置条件：App_rd_en 拉高（说明有新的读操作）就清零重新数。
always @(posedge mem_clk or posedge rst)
begin
	if(rst || App_rd_en)begin
        rd_delay <= 4'd0;
    end
    else if(rd_delay < 4'd10)begin
    	rd_delay <= rd_delay + 1'b1;
    end
end

//==============================================================================
//  地址生成与突发计数
//==============================================================================
always @(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		burst_cnt <= ZERO[BURST_BITS - 1:0];
		App_rd_addr_r <= ZERO[ADDR_BITS - 1:0];
		App_rd_en_d0 <= 1'b0;
	end
	else begin

		// ---- 突发计数器 ----
		if(state == S_CHECK_FIFO)
			burst_cnt <= ZERO[BURST_BITS - 1:0];
		else if(App_rd_en)
			burst_cnt <= burst_cnt + 1'b1;

		// ---- 读地址生成 ----
		if(state == S_ACK)
			begin
				// 根据索引选择缓冲区基地址
				if(read_addr_index_d1 == 2'd0)
					App_rd_addr_r <= read_addr_0;
				else if(read_addr_index_d1 == 2'd1)
					App_rd_addr_r <= read_addr_1;
				else if(read_addr_index_d1 == 2'd2)
					App_rd_addr_r <= read_addr_2;
				else if(read_addr_index_d1 == 2'd3)
					App_rd_addr_r <= read_addr_3;
				// 【这里不需要行翻转】因为翻转已经在写入的时候做过了，
				// 内存里存的就是正序的，读的时候直接顺序读就行。
			end
		else if(App_rd_en)
			App_rd_addr_r <= App_rd_addr_r + 1'b1;    // 顺序递增

		// ---- 输出使能 ----
		// ★ App_rd_en_d0 保持【未门控】★
		//   它是"我想发命令"的意图，仲裁器用它判优先级；真正发出去的命令
		//   再由 rd_ready 门控（见下面的 assign App_rd_en）。
		//   burst_cnt / rd_delay 都数【门控后】的 App_rd_en，所以被暂停时
		//   突发长度一点都不丢，只是被拉长。
		if(App_rd_en_r && burst_cnt + App_rd_en < BURST_SIZE)
			App_rd_en_d0 <= 1'b1;
		else
			App_rd_en_d0 <= 1'b0;

	end
end

//==============================================================================
//  ★ 主状态机 ★
//==============================================================================
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		state <= S_IDLE;
		read_len_latch <= ZERO[ADDR_BITS - 1:0];
		App_rd_en_r <= 1'b0;
		read_cnt <= ZERO[ADDR_BITS - 1:0];
		fifo_aclr <= 1'b0;
		read_req_ack <= 1'b0;
	end
	else
		case(state)

			// ============================================================
			//  S_IDLE：等读请求
			// ============================================================
			S_IDLE:
			begin
				if(read_req_d2 == 1'b1 && Sdr_init_done)
				begin
					state <= S_ACK;
				end
				read_req_ack <= 1'b0;
			end

			// ============================================================
			//  S_ACK：应答 + 清 FIFO + 锁存参数
			// ============================================================
			// 和写方向完全对称：应答上层、清 FIFO、锁长度、地址复位。
			// 【为什么读方向也要清 FIFO】因为上一帧的数据可能还有残留，
			// 不清掉的话，新的一帧开头会混进上一帧的尾巴，画面会错位。
			S_ACK:
			begin
				if(read_req_d2 == 1'b0)
				begin
					state <= S_CHECK_FIFO;
					fifo_aclr <= 1'b0;
					read_req_ack <= 1'b0;
				end
				else
				begin
					read_req_ack <= 1'b1;
					fifo_aclr <= 1'b1;
					read_len_latch <= fmt565 ? (read_len_d1 >> 1) : read_len_d1;
				end
				read_cnt <= ZERO[ADDR_BITS - 1:0];
			end

			// ============================================================
			//  S_CHECK_FIFO：检查 FIFO 空间 + 写是否忙
			// ============================================================
			S_CHECK_FIFO:
			begin
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;         // 新请求优先
				end
				// ★ 条件（100M-24 起）：FIFO 剩余空间够放一次突发 + 上一轮在途数据
				//   （"写方向不忙"已删除，理由见下面 100M-24 的说明）
				//   （时序修复：水位用打一拍的 wrusedw_r，见上面的说明；
				//     App_wr_busy 仍是实时值——让读侧看到的是写侧最新状态）
				//
				// ★★ 2026-10-16 上板修复：必须再扣掉 INFLIGHT_MAX ★★
				//   上面第 106 行原来断言"S_CHECK_FIFO 期间读 FIFO 没有数据流入，
				//   水位只降不升"—— 那个断言**只在延迟 ≤ rd_delay(10 拍) 时成立**。
				//   真 SDRAM 的行激活/刷新/写口抢占会让回程延迟大得多，于是：
				//       进入 S_CHECK_FIFO 时，上一轮最后几十个像素还在路上
				//       → wrusedw 被低估 → 判定"空间够" → 再发 128 条
				//       → 在途数据一齐落地 → **FIFO 溢出丢像素**
				//   → 丢一个显示像素 = 该行余下部分横向错一位
				//   → 画面上出现**竖直线状的错位缝 / 散点**（上板实测现象）
				//
				//   所以阈值要再扣掉"最大在途量"（INFLIGHT_MAX=64）。
				//   扣太多会把水位压低 → 视频端欠载 → 左右晃动，所以只能扣 64。
				//   ★ 100M-23 再加一道：~wr_full。wrusedw 在 512 处回绕成 0，
				//     "真满"时 `wrusedw < 阈值` 反而成立 ⇒ 会再发 128 条 ⇒
				//     满 FIFO 上的数据被静默丢掉（丢像素 = 该行余下部分错一位）。
				//     full_flag 不回绕，是最后一道闸门。
				//   ★ 100M-24：**去掉 ~App_wr_busy** ★
				//     读写本来就在不同时隙（读 0~3、写 11~13），"写优先"在时隙表
				//     层面已经保证；但在突发**起跑**这一拍再卡一道 App_wr_busy，
				//     代价是被写引擎挡掉就是整整一轮（256 拍 = 2.56µs）。
				//     两图层重叠时纹理把读口压满，这种"整轮错过"会连着发生几次
				//     ⇒ 读 FIFO 见底 ⇒ 视频端取到空 FIFO ⇒ 那一列变成白齿。
				//     去掉后读突发的起跑机会只增不减，不会和写突发撞（时隙互斥）。
				else if(wrusedw_r < (FIFO_DEPTH - BURST_SIZE - INFLIGHT_MAX) && ~wr_full)
				begin
					state <= S_READ_BURST;
					App_rd_en_r <= 1'b1;
				end
			end

			// ============================================================
			//  S_READ_BURST：正在发突发读命令
			// ============================================================
			S_READ_BURST:
			begin
				if(rd_burst_finish == 1'b1)
				begin
					App_rd_en_r <= 1'b0;
					state <= S_READ_BURST_END;
					read_cnt <= read_cnt + BURST_SIZE[ADDR_BITS - 1:0];   // 计数 +256
				end
			end

			// ============================================================
			//  S_READ_BURST_END：判断整帧读完没有
			// ============================================================
			S_READ_BURST_END:
			begin
				if(read_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				else if(read_cnt < read_len_latch)
				begin
					state <= S_CHECK_FIFO;   // 还没读完，继续
				end
				else
				begin
					state <= S_END;          // 读够了
				end
			end

			// ============================================================
			//  S_END：整帧读完
			// ============================================================
			S_END:
			begin
				state <= S_IDLE;
			end

			default:
				state <= S_IDLE;
		endcase
end
endmodule
