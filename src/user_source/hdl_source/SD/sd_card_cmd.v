
//==============================================================================
//  sd_card_cmd.v  —— SD 命令协议层（负责"命令怎么发、数据怎么搬"）
//==============================================================================
//
//  【这个文件干什么的】
//  它夹在"协议层"（sd_card_sec_read_write，决定发什么命令）和
//  "物理层"（spi_master，只管收发字节）之间，负责三件事：
//
//    1. 上电时先发 11 个 0xFF 字节 —— 见 S_INIT 状态
//    2. 把 48bit 命令拆成 6 个字节，通过 SPI 一个个发出去，
//       然后不停地读响应字节，直到读到期望的值 —— 见 S_CMD 状态
//    3. 命令成功之后，按 cmd_data_len 发/收附带的数据（如果有）
//
//  【★ 上电为什么要发 11 个 0xFF（至少 74 个 SPI 时钟）★】
//  这是 SD 卡规范的硬性要求。卡上电后内部要跑一段初始化程序，
//  在这期间必须给它提供至少 74 个时钟脉冲，让它完成自我配置。
//  11 个字节 = 88 个时钟，满足"至少 74 个"的要求。
//  这期间片选（nCS）必须保持高（不选中），让卡"听"时钟但不响应命令。
//
//  【★ 命令的 48bit 格式 ★】
//    bit[47:46]  起始位，固定是 0b01
//    bit[45:40]  命令码（0~63）
//    bit[39:8]   参数（4 字节）
//    bit[7:1]    CRC7 校验
//    bit[0]      结束位，固定 1
//  【代码里怎么处理的】看这一行：
//      send_data <= (cmd[47:40] | 8'h40);
//  0x40 = 0b0100_0000，正好把 bit[47:46] 置成 0b01，
//  同时保留低 6 位的命令码。这样上层只需要在 cmd[47:40] 里填命令码，
//  起始位由这里自动补上。
//
//  【★ 响应字节（R1）★】
//  卡对命令的响应是一个字节，格式是：
//    bit[7] = 0 表示这是有效响应（bit[7]=1 表示还在忙，是"无效响应"）
//    bit[6] = 参数错误
//    bit[5] = 地址错误
//    bit[4] = 擦除序列错误
//    bit[3] = CRC 错误
//    bit[2] = 非法命令
//    bit[1] = 擦除复位
//    bit[0] = 处于空闲状态（1 = 还在空闲，初始化没完成）
//  【关键判断（第 135 行）】
//    if(data_recv != cmd_r1 && data_recv[7] == 1'b0)
//  意思是：收到的字节 bit[7]=0（是有效响应）但又不是我期望的值 → 出错
//  如果 bit[7]=1，说明卡还在忙，继续读下一个字节（不等它）。
//
//  【★ 数据块的结构（读写扇区时）★】
//  读：
//    卡先发一个字节 0xFE（数据起始令牌）→ 然后 512 字节数据 → 然后 2 字节 CRC
//    所以 S_READ_WAIT 在等 0xFE，S_READ 在收 513 个字节（512 数据 + 1 CRC？）
//    【注意代码里是 byte_cnt == 16'd513】，实际是收到 512 字节数据后
//    再多读几个字节把 CRC 吃掉再结束。
//  写：
//    主机发 0xFE（起始令牌）→ 512 字节数据 → 2 字节 CRC → 读一个字节看卡的反应
//    卡的反应字节 bit[4:1] = 0b0101（5）表示"数据写入成功"
//    然后还要等卡把数据真正写进 Flash（这段时间 MISO 一直是 0，写完变 0xFF）
//
//==============================================================================
module sd_card_cmd(
	input                       sys_clk,        // 系统时钟
	input                       rst,            // 复位，高有效
	input[15:0]                 spi_clk_div,    // SPI 分频设置（来自协议层）
	input                       cmd_req,        // 请求发命令
	output                      cmd_req_ack,    // 命令完成应答
	output reg                  cmd_req_error,  // 命令出错
	input[47:0]                 cmd,            // 命令内容
	input[7:0]                  cmd_r1,         // 期望的响应字节
	input[15:0]                 cmd_data_len,   // 命令附带数据长度

	// ---- 扇区数据搬运 ----
	input                       block_read_req,     // 请求读一块
	output reg                  block_read_valid,   // 读到的字节有效
	output reg[7:0]             block_read_data,    // 读到的字节
	output                      block_read_req_ack, // 读完成应答
	input                       block_write_req,    // 请求写一块
	input[7:0]                  block_write_data,   // 要写的字节
	output                      block_write_data_rd,// 请求下一个写字节
	output                      block_write_req_ack,// 写完成应答

	// ---- 给物理层的接口 ----
	output                      nCS_ctrl,       // 片选控制
	output reg[15:0]            clk_div,        // 实际使用的分频值
	output reg                  spi_wr_req,     // 请求发一个字节
	input                       spi_wr_ack,     // 字节发送完成
	output[7:0]                 spi_data_in,    // 要发的字节
	input[7:0]                  spi_data_out    // 收到的字节
);

//==============================================================================
//  状态编码
//==============================================================================
// 命名规律：
//   S_IDLE/S_WAIT/S_INIT       → 初始化准备
//   S_CMD_PRE/S_CMD/S_CMD_DATA → 发命令 + 收响应
//   S_READ_WAIT/S_READ/S_READ_ACK       → 读流程
//   S_WRITE_TOKEN/S_WRITE_DATA_0/1/S_WRITE_CRC/S_WRITE_SUC/S_WRITE_BUSY/S_WRITE_ACK → 写流程
//   S_ERR/S_END                → 收尾
parameter S_IDLE         = 0;    // 空闲（复位后起点）
parameter S_WAIT         = 1;    // 待命：等命令或数据请求
parameter S_INIT         = 2;    // 初始化：发 11 个 0xFF
parameter S_CMD_PRE      = 3;    // 发命令前先发一个 0xFF（提供时钟）
parameter S_CMD          = 4;    // 发命令 + 收响应
parameter S_CMD_DATA     = 5;    // 命令附带的数据
parameter S_READ_WAIT    = 6;    // 读：等数据起始令牌 0xFE
parameter S_READ         = 7;    // 读：收 512 字节数据
parameter S_READ_ACK     = 8;    // 读：完成应答
parameter S_WRITE_TOKEN  = 9;    // 写：发数据起始令牌 0xFE
parameter S_WRITE_DATA_0 = 10;   // 写：准备一个字节（拉高 data_rd 通知上层给数据）
parameter S_WRITE_DATA_1 = 11;   // 写：发字节
parameter S_WRITE_CRC    = 12;   // 写：发 2 字节 CRC
parameter S_WRITE_SUC    = 13;   // 写：读卡的数据响应
parameter S_WRITE_BUSY   = 14;   // 写：等卡内部写入完成
parameter S_WRITE_ACK    = 15;   // 写：完成应答
parameter S_ERR          = 16;   // 出错
parameter S_END          = 17;   // 结束（产生 cmd_req_ack）

reg[4:0]                      state;
reg                           CS_reg;        // 片选寄存器
reg[15:0]                     byte_cnt;      // 字节计数器
reg[7:0]                      send_data;     // 当前要发送的字节
wire[7:0]                     data_recv;     // 当前收到的字节
reg[9:0]                      wr_data_cnt;   // 已写的字节数

//==============================================================================
//  组合逻辑输出
//==============================================================================
// 这几个信号都是"用当前状态直接组合出来"的，不需要额外的寄存器。
// 【这样写的好处】省触发器；【坏处】输出会跟着状态变化有毛刺，
// 但如果下游只在时钟边沿采样，就没问题。
assign cmd_req_ack = (state == S_END);                            // 到达 END 状态 = 命令流程完成
assign block_read_req_ack = (state == S_READ_ACK);                // 读完成
assign block_write_req_ack= (state == S_WRITE_ACK);               // 写完成
assign block_write_data_rd = (state == S_WRITE_DATA_0);           // 请求下一个写字节
assign spi_data_in = send_data;                                   // 要发的字节
assign data_recv = spi_data_out;                                  // 收到的字节
assign nCS_ctrl = CS_reg;                                         // 片选

//==============================================================================
//  ★ 主状态机 ★
//==============================================================================
always@(posedge sys_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		// ---------- 复位 ----------
		CS_reg <= 1'b1;             // 片选拉高（不选中）
		spi_wr_req <= 1'b0;
		byte_cnt <= 16'd0;
		clk_div <= 16'd0;
		send_data <= 8'hff;
		state <= S_IDLE;
		cmd_req_error <= 1'b0;
		wr_data_cnt <= 10'd0;
	end
	else
		case(state)

			// ================================================================
			//  S_IDLE → 设置 SPI 分频 → 进入初始化
			// ================================================================
			S_IDLE:
			begin
				state <= S_INIT;
				clk_div <= spi_clk_div;     // 采用协议层指定的分频
				CS_reg <= 1'b1;             // 确保片选是高的
			end

			// ================================================================
			//  S_INIT：上电初始化，发 11 个 0xFF
			// ================================================================
			// 【为什么是 11 个】至少需要 74 个时钟，11 字节 = 88 个时钟 ✓
			// 【为什么片选要保持高】见文件头注释。这期间卡"只收时钟不响应命令"。
			// 【握手是怎么走的】
			//   拉高 spi_wr_req → 物理层发完一个字节 → 拉高 spi_wr_ack → 计数 +1
			//   数到 10（即发了 11 个）就结束
			S_INIT:
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(byte_cnt >= 16'd10)
					begin
						byte_cnt <= 16'd0;
						spi_wr_req <= 1'b0;
						state <= S_WAIT;        // 初始化准备完成，待命
					end
					begin
						byte_cnt <= byte_cnt + 16'd1;   // 继续数
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;     // 举手：请发一个字节
					send_data <= 8'hff;     // 发的内容是 0xFF
				end
			end

			// ================================================================
			//  S_WAIT：待命，看有命令还是有数据要搬
			// ================================================================
			S_WAIT:
			begin
				cmd_req_error <= 1'b0;      // 清错误标志，准备新任务
				wr_data_cnt <= 10'd0;
				if(cmd_req == 1'b1)
					state <= S_CMD_PRE;                  // 有命令 → 先发个 0xFF
				else if(block_read_req == 1'b1)
					state <= S_READ_WAIT;                // 有读请求 → 等数据令牌
				else if(block_write_req == 1'b1)
					state <= S_WRITE_TOKEN;              // 有写请求 → 发数据令牌
				clk_div <= spi_clk_div;                  // 每次待命都重设分频
			end

			// ================================================================
			//  S_CMD_PRE：命令前发一个 0xFF
			// ================================================================
			// 【为什么需要】SD 卡在收到命令前需要一个"空闲时钟"，
			// 用来区分"上一个命令的尾巴"和"新命令的开头"。
			// 发 0xFF 就是给卡提供一个完整的时钟周期。
			S_CMD_PRE:
			begin
				if(spi_wr_ack == 1'b1)
				begin
					state <= S_CMD;
					spi_wr_req <= 1'b0;
					byte_cnt <= 16'd0;          // 字节计数清零，准备发命令的 6 个字节
				end
				else
				begin
					spi_wr_req <= 1'b1;
					CS_reg <= 1'b1;             // 【注意】发 0xFF 期间片选还是高的
					send_data <= 8'hff;
				end
			end

			// ================================================================
			//  ★ S_CMD：发 6 字节命令，然后轮询读取响应 ★
			// ================================================================
			// 这个状态干两件事：
			//   byte_cnt 0~5：把命令的 6 个字节依次发出去
			//   byte_cnt 6~ ：发 0xFF（提供时钟）并读取响应字节
			S_CMD:
			begin
				if(spi_wr_ack == 1'b1)
				begin
					// ★ 收到一个字节了，判断它是不是有效响应
					if((byte_cnt == 16'hffff) || (data_recv != cmd_r1 && data_recv[7] == 1'b0))
					begin
						// ---- 出错：要么读超时了，要么收到有效响应但值不对 ----
						state <= S_ERR;
						spi_wr_req <= 1'b0;
						byte_cnt <= 16'd0;
					end
					else if(data_recv == cmd_r1)
					begin
						// ---- 收到期望的响应了！----
						spi_wr_req <= 1'b0;
						if(cmd_data_len != 16'd0)
						begin
							// 命令还有附带数据要处理
							state <= S_CMD_DATA;
							byte_cnt <= 16'd0;
						end
						else
						begin
							// 命令流程结束
							state <= S_END;
							byte_cnt <= 16'd0;
						end
					end
					else
						byte_cnt <=  byte_cnt + 16'd1;   // 还没收到有效响应，继续读
				end
				else
				begin
					// ★ 准备并发送下一个字节
					spi_wr_req <= 1'b1;
					CS_reg <= 1'b0;     // 【关键】发命令时片选要拉低（选中卡）
					if(byte_cnt == 16'd0)
						// 第 0 字节：命令码。| 8'h40 就是自动补上起始位 0b01
						send_data <= (cmd[47:40] | 8'h40);
					else if(byte_cnt == 16'd1)
						send_data <= cmd[39:32];   // 参数第 1 字节
					else if(byte_cnt == 16'd2)
						send_data <= cmd[31:24];   // 参数第 2 字节
					else if(byte_cnt == 16'd3)
						send_data <= cmd[23:16];   // 参数第 3 字节
					else if(byte_cnt == 16'd4)
						send_data <= cmd[15:8];    // 参数第 4 字节
					else if(byte_cnt == 16'd5)
						send_data <= cmd[7:0];     // CRC 字节
					else
						send_data <= 8'hff;        // 第 6 个以后：发 0xFF 提供时钟
						// 【为什么要一直发 0xFF】SPI 是"发时钟才能收数据"的协议。
						// 想读卡的响应，就必须持续给时钟，所以不停地发 0xFF。
				end
			end

			// ================================================================
			//  S_CMD_DATA：命令的附带数据（比如 CMD8 的 4 字节响应）
			// ================================================================
			S_CMD_DATA:
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(byte_cnt == cmd_data_len - 16'd1)
					begin
						state <= S_END;         // 收够了
						spi_wr_req <= 1'b0;
						byte_cnt <= 16'd0;
					end
					else
					begin
						byte_cnt <= byte_cnt + 16'd1;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;         // 只读不发，所以发 0xFF
				end
			end

			// ================================================================
			//  S_READ_WAIT：等卡发数据起始令牌 0xFE
			// ================================================================
			// 发 CMD17 之后，卡不会立刻给数据，它要先花点时间去找地址。
			// 这期间它一直发 0xFF（或者 0x00），直到准备好后才发 0xFE。
			// 【0xFE 是什么】Single/Multiple Block Read 的数据起始令牌，
			// 收到它就表示"数据马上来了"。
			S_READ_WAIT:
			begin
				if(spi_wr_ack == 1'b1 && data_recv == 8'hfe)
				begin
					spi_wr_req <= 1'b0;
					state <= S_READ;            // 收到令牌，开始收数据
					byte_cnt <= 16'd0;
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;         // 持续给时钟
				end
			end

			// ================================================================
			//  S_READ：接收 512 字节数据
			// ================================================================
			S_READ:
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(byte_cnt == 16'd513)
					begin
						state <= S_READ_ACK;    // 收够了
						spi_wr_req <= 1'b0;
						byte_cnt <= 16'd0;
					end
					else
					begin
						byte_cnt <= byte_cnt + 16'd1;
					end
					// 【注意】数据的有效性是由下面单独的 always 块产生的
					// （block_read_valid = state==S_READ && byte_cnt < 512），
					// 保证只有前 512 字节被当作数据，后面的 CRC 被忽略。
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end

			// ================================================================
			//  S_WRITE_TOKEN：写数据前先发起始令牌 0xFE
			// ================================================================
			S_WRITE_TOKEN:
				if(spi_wr_ack == 1'b1)
				begin
					state <= S_WRITE_DATA_0;
					spi_wr_req <= 1'b0;
					byte_cnt <= 16'd0;
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hfe;      // 起始令牌
				end

			// ================================================================
			//  S_WRITE_DATA_0：拉高 data_rd，通知协议层"给我下一个字节"
			// ================================================================
			S_WRITE_DATA_0:
			begin
				state <= S_WRITE_DATA_1;     // 【注意】无条件跳到下一个状态，只待一拍
				wr_data_cnt <= wr_data_cnt + 10'd1;
				// 【为什么只待一拍】因为 block_write_data_rd 是用状态组合出来的
				// （assign block_write_data_rd = state == S_WRITE_DATA_0），
				// 待一拍就能给协议层一个"取数据"的窗口。
			end

			// ================================================================
			//  S_WRITE_DATA_1：把字节发出去
			// ================================================================
			S_WRITE_DATA_1:
			begin
				if(spi_wr_ack == 1'b1 && wr_data_cnt == 10'd512)
				begin
					// 512 字节都发完了 → 去发 CRC
					state <= S_WRITE_CRC;
					spi_wr_req <= 1'b0;
				end
				else if(spi_wr_ack == 1'b1)
				begin
					// 发完了但还没到 512 → 回落去取下一个字节
					state <= S_WRITE_DATA_0;
					spi_wr_req <= 1'b0;
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= block_write_data;   // 发上层给的字节
				end
			end

			// ================================================================
			//  S_WRITE_CRC：发 2 个字节的 CRC
			// ================================================================
			S_WRITE_CRC:
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(byte_cnt == 16'd1)
					begin
						state <= S_WRITE_SUC;
						spi_wr_req <= 1'b0;
						byte_cnt <= 16'd0;
					end
					else
					begin
						byte_cnt <= byte_cnt + 16'd1;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;     // 发 0xFF 当 CRC（卡通常不校验写时的 CRC）
				end
			end

			// ================================================================
			//  S_WRITE_SUC：读卡的数据响应，确认写入被接受
			// ================================================================
			// 【响应格式】卡回一个字节，其中 bit[4:1] 是数据响应码：
			//   0101 (5) = 数据被接受
			//   1011 (B) = CRC 错误
			//   1101 (D) = 写错误
			// 代码里检查 data_recv[4:0] == 5'b00101，就是检查"接受"。
			S_WRITE_SUC :
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(data_recv[4:0] == 5'b00101)
					begin
						state <= S_WRITE_BUSY;      // 被接受了，等卡内部写完
						spi_wr_req <= 1'b0;
					end
					// 【注意】如果响应不是"接受"，这里什么都不做，
					// 会一直停在这个状态反复读，直到读到"接受"为止。
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end

			// ================================================================
			//  S_WRITE_BUSY：等卡把数据真正写进 Flash
			// ================================================================
			// 【卡怎么表示"我还在写"】这段时间 MISO 一直是低电平（0x00）。
			// 写完了，MISO 会变回高电平（0xFF）。
			// 所以这里不停地读，直到读到 0xFF 才算写完。
			// 【要等多久】SD 卡写入一个扇区通常需要几毫秒，
			// 按 25MHz SPI 时钟算，一个字节 0.32us，要读几千次。
			S_WRITE_BUSY :
			begin
				if(spi_wr_ack == 1'b1)
				begin
					if(data_recv == 8'hff)
					begin
						state <= S_WRITE_ACK;       // 卡写完了
						spi_wr_req <= 1'b0;
					end
				end
				else
				begin
					spi_wr_req <= 1'b1;
					send_data <= 8'hff;
				end
			end

			// ================================================================
			//  S_ERR / S_END：收尾状态
			// ================================================================
			S_ERR:
			begin
				state <= S_END;
				cmd_req_error <= 1'b1;      // 报错
			end
			// 这三个状态做的事一样：跳回待命。
			// cmd_req_ack 会在 S_END 时拉高一拍（见上面的 assign），
			// 协议层看到它就知道了"这条命令处理完了"。
			S_READ_ACK,S_WRITE_ACK,S_END:
			begin
				state <= S_WAIT;
			end

			default:
				state <= S_IDLE;
		endcase
end

//==============================================================================
//  产生"读数据有效"信号
//==============================================================================
// 【为什么单独写一个 always 块】
// 因为它的时序和主状态机略有不同，写在一起容易乱。
// 条件：处于 S_READ 状态 且 计数还在 512 以内（排除后面的 CRC 字节）。
always@(posedge sys_clk or posedge rst)
begin
	if(rst == 1'b1)
		block_read_valid <= 1'b0;
	else if(state == S_READ && byte_cnt < 16'd512)
		block_read_valid <= spi_wr_ack;     // 每读完一个字节，有效一拍
	else
		block_read_valid <= 1'b0;
end

//==============================================================================
//  取出读到的数据字节
//==============================================================================
always@(posedge sys_clk or posedge rst)
begin
	if(rst == 1'b1)
		block_read_data <= 8'd0;
	else if(state == S_READ && spi_wr_ack == 1'b1)
		block_read_data <= data_recv;       // 字节接收完成时锁存
	// 【注意】这里没有 else，意味着不在读状态时数据保持上一次的值。
	// 但因为 block_read_valid 在那个时刻是 0，下游不会采信这个数据。
end

endmodule
