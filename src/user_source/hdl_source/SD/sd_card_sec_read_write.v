
//==============================================================================
//  sd_card_sec_read_write.v  —— SD 卡初始化与扇区读写协议层
//==============================================================================
//
//  【这个文件干什么的】
//  这是 SD 卡驱动里"最懂协议"的模块。它按 SD 卡规范规定的顺序，
//  一步一步地把卡初始化好，之后响应"读扇区 / 写扇区"的请求。
//
//  【★ SD 卡 SPI 模式的初始化流程 ★ —— 这是本文件最重要的内容】
//  上电后必须按这个顺序发命令，一步都不能错、不能乱：
//
//    步骤1  CMD0  (GO_IDLE_STATE)
//           让卡进入空闲状态。这是"复位"卡的第一步。
//           注意：CMD0 的响应期望是 0x01，表示"我还在空闲状态"。
//
//    步骤2  CMD8  (SEND_IF_COND)
//           询问卡的电压支持情况。参数里的 0x1AA 是"我是 2.7~3.6V 的高容量卡"。
//           期望响应 0x01。只有 CMD8 有响应，才能确定这是 SDHC/SDXC 卡。
//
//    步骤3  CMD55 (APP_CMD)
//           声明"下一条命令是应用类命令（ACMD）"。
//           SD 卡规范里，一些命令需要先发 CMD55 做前缀。
//           期望响应 0x01。
//
//    步骤4  CMD41 (SD_SEND_OP_COND, 属于 ACMD41)
//           让卡开始初始化，并询问它准备好了没有。
//           参数 0x40000000 表示"我支持高容量卡（HCS）"。
//           期望响应 0x00 表示"初始化完成"。
//           ★ 如果响应还是 0x01，说明卡还在忙，必须**重复** CMD55 + CMD41，
//             一直等到响应变成 0x00 为止。代码里 142 行那个
//             "else if(cmd_req_ack) state <= S_CMD55;" 就是实现这个循环。
//
//    步骤5  CMD16 (SET_BLOCKLEN)
//           设置读写块的长度为 512 字节（扇区大小）。
//           参数 32'd512。期望响应 0x00。
//
//    步骤6  进入待命状态 S_WAIT_READ_WRITE，等上层的读写请求。
//
//  【之后的常规操作】
//    读一个扇区：CMD17 (READ_SINGLE_BLOCK)，参数 = 扇区号
//                期望响应 0x00，然后卡会发一个 0xFE 数据起始令牌，
//                接着就是 512 字节数据。
//    写一个扇区：CMD24 (WRITE_BLOCK)，参数 = 扇区号
//                期望响应 0x00，然后发 0xFE + 512 字节数据 + 2 字节 CRC。
//
//  【关键设计点：速度切换】
//  初始化阶段用慢速 SPI（200kHz），CMD41 成功之后立刻切到快速（25MHz）。
//  看代码里 spi_clk_div 的三次赋值：
//    - 复位/S_IDLE：设成 SPI_LOW_SPEED_DIV（慢）
//    - S_CMD41 成功：设成 SPI_HIGH_SPEED_DIV（快）
//    - S_WAIT_READ_WRITE：设成 16'd0（也是快）
//
//  【注意本模块的复位极性】rst = 1 时复位，高电平有效。
//
//==============================================================================
module sd_card_sec_read_write
#(
	parameter  SPI_LOW_SPEED_DIV = 248,         // 慢速分频（初始化用）
	parameter  SPI_HIGH_SPEED_DIV = 0           // 快速分频（正常读扇区用）
)
(
	input            clk,                       // 100MHz 时钟
	input            rst,                       // 复位，高有效
	output reg       sd_init_done,              // 初始化完成标志
	input            sd_sec_read,               // 上层请求读扇区
	input[31:0]      sd_sec_read_addr,          // 读哪个扇区
	output[7:0]      sd_sec_read_data,          // 读出的字节
	output           sd_sec_read_data_valid,    // 字节有效
	output           sd_sec_read_end,           // 扇区读完
	input            sd_sec_write,              // 上层请求写扇区
	input[31:0]      sd_sec_write_addr,         // 写哪个扇区
	input[7:0]       sd_sec_write_data,         // 要写的字节
	output           sd_sec_write_data_req,     // 请求下一个写字节
	output           sd_sec_write_end,          // 扇区写完

	// ---- 给命令层的接口 ----
	output reg[15:0] spi_clk_div,               // SPI 速度设置
	output reg       cmd_req,                   // 请求发命令
	input            cmd_req_ack,               // 命令发完
	input            cmd_req_error,             // 命令出错
	output reg[47:0] cmd,                       // 命令内容（48bit）
	output reg[7:0]  cmd_r1,                    // 期望的响应
	output reg[15:0] cmd_data_len,              // 附带数据长度

	// ---- 扇区数据搬运接口 ----
	output reg       block_read_req,            // 请求读一块
	input            block_read_valid,          // 数据有效
	input[7:0]       block_read_data,           // 读到的数据
	input            block_read_req_ack,        // 读完成
	output reg       block_write_req,           // 请求写一块
	output[7:0]      block_write_data,          // 要写的数据
	input            block_write_data_rd,       // 请求下一个字节
	input            block_write_req_ack        // 写完成
);
reg[7:0] read_data;
reg[31:0] timer;      // 一个备用定时器（本版本未使用）

//==============================================================================
//  状态编码
//==============================================================================
// 命名很有规律，看名字就知道在哪一步：
//   S_CMD0/8/55/41/16   → 初始化阶段的各条命令
//   S_CMD17 / S_READ    → 读扇区流程
//   S_CMD24 / S_WRITE   → 写扇区流程
//   S_READ_END/S_WRITE_END → 流程收尾
localparam S_IDLE               = 0;     // 空闲（复位后的起点）
localparam S_CMD0               = 1;     // 发 CMD0
localparam S_CMD8               = 2;     // 发 CMD8
localparam S_CMD55              = 3;     // 发 CMD55
localparam S_CMD41              = 4;     // 发 CMD41
localparam S_CMD17              = 5;     // 发 CMD17（单扇区读）
localparam S_READ               = 6;     // 读数据中
localparam S_CMD24              = 7;     // 发 CMD24（单扇区写）
localparam S_WRITE              = 8;     // 写数据中
localparam S_ERR                = 14;    // 出错
localparam S_WRITE_END          = 15;    // 写扇区结束
localparam S_READ_END           = 16;    // 读扇区结束
localparam S_WAIT_READ_WRITE    = 17;    // 待命：等上层读写请求
localparam S_CMD16              = 18;    // 发 CMD16（设置块长度）

reg[4:0]                       state;
reg[31:0]                      sec_addr;    // 当前操作的扇区号（锁存）

//==============================================================================
//  组合逻辑输出
//==============================================================================

// 数据有效：只有在 S_READ 状态、且命令层说"这个字节有效"时才有效
assign sd_sec_read_data_valid = (state == S_READ) && block_read_valid;

// 读出的数据直接来自命令层
assign sd_sec_read_data = block_read_data;

// 扇区读完：进入 S_READ_END 状态时拉高
assign sd_sec_read_end = (state == S_READ_END);

// 请求下一个写字节（写流程专用）
assign sd_sec_write_data_req = (state == S_WRITE) && block_write_data_rd;

// 要写的数据来自上层
assign block_write_data = sd_sec_write_data;

// 扇区写完：进入 S_WRITE_END 状态时拉高
assign sd_sec_write_end = (state == S_WRITE_END);

//==============================================================================
//  ★ 主状态机 ★
//==============================================================================
always@(posedge clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		// ---------- 复位 ----------
		state <= S_IDLE;
		cmd_req <= 1'b0;
		cmd_data_len <= 16'd0;
		cmd_r1 <= 8'd0;
		cmd <= 48'd0;
		spi_clk_div <= SPI_LOW_SPEED_DIV[15:0];    // 复位时用慢速
		block_write_req <= 1'b0;
		block_read_req <= 1'b0;
		sec_addr <= 32'd0;
		sd_init_done <= 1'b0;
	end
	else
		case(state)

			// ================================================================
			//  S_IDLE → 立即进入 CMD0，开始初始化
			// ================================================================
			S_IDLE:
			begin
				state <= S_CMD0;
				sd_init_done <= 1'b0;
				spi_clk_div <= SPI_LOW_SPEED_DIV[15:0];   // 确保慢速
			end

			// ================================================================
			//  ◆ CMD0：让卡进入空闲状态
			// ================================================================
			// 【状态机的通用模式（后面每个 CMD 状态都一样）】
			//   if (cmd_req_ack & ~cmd_req_error)  → 命令成功发完了，去下一个状态
			//   else → 把命令内容摆好、把 cmd_req 拉高，等命令层来取
			// 这种"摆好数据 + 举手请求"的写法叫"握手"，是 FPGA 里最常见的接口方式。
			S_CMD0:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD8;        // 成功了，去下一步
					cmd_req <= 1'b0;        // 撤销请求
				end
				else
				begin
					cmd_req <= 1'b1;                       // 举手：请发命令
					cmd_data_len <= 16'd0;                 // CMD0 没有附带数据
					cmd_r1 <= 8'h01;                       // 期望响应 0x01（还在 idle）
					cmd <= {8'd0,8'h00,8'h00,8'h00,8'h00,8'h95};
					//  ↑ 48bit 命令的组成：{命令码, 参数(4字节), CRC}
					//    CMD0 的 CRC 是固定值 0x95（规范里 CMD0 和 CMD8 必须用正确 CRC）
				end
			end

			// ================================================================
			//  ◆ CMD8：查询电压，确认是 SDHC/SDXC 卡
			// ================================================================
			S_CMD8:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD55;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd4;                 // CMD8 有 4 字节响应数据
					cmd_r1 <= 8'h01;
					cmd <= {8'd8,8'h00,8'h00,8'h01,8'haa,8'h87};
					//  ↑ 参数 0x000001AA：VHS=1（2.7~3.6V）+ 校验图案 0xAA
					//    CRC 固定 0x87
				end
			end

			// ================================================================
			//  ◆ CMD55：声明"下一条是应用命令"
			// ================================================================
			// 【为什么要这个前缀】SD 卡的 ACMD41 必须通过 CMD55 来"解锁"。
			// 这是规范的设计，防止误触发。
			S_CMD55:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_CMD41;
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h01;
					cmd <= {8'd55,8'h00,8'h00,8'h00,8'h00,8'hff};
					//  ↑ CMD55 之后 CRC 就可以用 0xff 了（不校验）
				end
			end

			// ================================================================
			//  ◆ CMD41：启动初始化，并等待卡准备好
			// ================================================================
			// ★ 这个状态有三条分支，是初始化流程的关键：
			//   分支1（第 135 行）：响应正确（0x00）→ 初始化完成，切高速，去 CMD16
			//   分支2（第 142 行）：cmd_req_ack 但响应不对 → 说明卡还在忙，
			//                       回到 CMD55 重新走一遍（这就是"轮询等待"）
			//   分支3（第 146 行）：还没发出去 → 摆好命令
			S_CMD41:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					// ---- 分支1：初始化成功！----
					state <= S_CMD16;
					cmd_req <= 1'b0;
					sd_init_done <= 1'b1;                              // 宣布卡可用了
					spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];           // ★ 切到高速
				end
				else if(cmd_req_ack)
				begin
					// ---- 分支2：卡还没准备好，重新轮询 ----
					// 【注意这里状态跳回 S_CMD55】因为 ACMD41 每次都要
					// 先发 CMD55 做前缀，所以必须回到 CMD55 而不是 S_CMD41。
					state <= S_CMD55;
				end
				else
				begin
					// ---- 分支3：发送命令 ----
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;                        // 期望响应 0x00（准备好了）
					cmd <= {8'd41,8'h40,8'h00,8'h00,8'h00,8'hff};
					//  ↑ 参数 0x40000000 = HCS 位为 1，表示"我是高容量卡"
				end
			end

			// ================================================================
			//  ◆ CMD16：设置扇区大小为 512 字节
			// ================================================================
			// 【其实现在的 SD 卡默认块大小就是 512】但规范建议显式设置一下，
			// 保证兼容性。参数就是 512。
			S_CMD16:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_WAIT_READ_WRITE;
					cmd_req <= 1'b0;
					sd_init_done <= 1'b1;
					spi_clk_div <= SPI_HIGH_SPEED_DIV[15:0];
				end
				else if(cmd_req_ack)
				begin
					// 出错了，从头再来一遍初始化
					state <= S_CMD55;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd16,32'd512,8'hff};
					//  ↑ 参数直接写 32'd512，Verilog 会自动拆成 4 字节
				end
			end

			// ================================================================
			//  S_WAIT_READ_WRITE：待命，等上层读写请求
			// ================================================================
			S_WAIT_READ_WRITE:
			begin
				if(sd_sec_write ==  1'b1)
				begin
					state <= S_CMD24;                  // 有写请求 → 发 CMD24
					sec_addr <= sd_sec_write_addr;     // 锁存扇区号
				end
				else if(sd_sec_read == 1'b1)
				begin
					state <= S_CMD17;                  // 有读请求 → 发 CMD17
					sec_addr <= sd_sec_read_addr;
				end

				spi_clk_div <= 16'd0;    // 保持最高速度（分频 0 = 最快）
				// 【为什么反复设置 spi_clk_div】因为命令层在发命令时
				// 可能会用 spi_clk_div 来配置物理层，某些状态下它会被改成别的值，
				// 所以待命时重新设一次，保证后续操作是全速的。
			end

			// ================================================================
			//  ◆ CMD24：写单个扇区
			// ================================================================
			S_CMD24:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_WRITE;          // 命令成功 → 开始发数据
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd24,sec_addr,8'hff};
					//  ↑ 参数就是扇区号（SDHC 卡用"块地址"而不是字节地址）
				end
			end

			// ================================================================
			//  S_WRITE：搬运 512 字节数据到卡里
			// ================================================================
			// 握手方式：本模块把 block_write_req 拉高，命令层去取数据，
			// 取完一轮（512 字节）后把 block_write_req_ack 拉高。
			S_WRITE:
			begin
				if(block_write_req_ack == 1'b1)
				begin
					block_write_req <= 1'b0;
					state <= S_WRITE_END;
				end
				else
					block_write_req <= 1'b1;
			end

			// ================================================================
			//  ◆ CMD17：读单个扇区
			// ================================================================
			S_CMD17:
			begin
				if(cmd_req_ack & ~cmd_req_error)
				begin
					state <= S_READ;           // 命令成功 → 开始收数据
					cmd_req <= 1'b0;
				end
				else
				begin
					cmd_req <= 1'b1;
					cmd_data_len <= 16'd0;
					cmd_r1 <= 8'h00;
					cmd <= {8'd17,sec_addr,8'hff};
				end
			end

			// ================================================================
			//  S_READ：接收 512 字节数据
			// ================================================================
			S_READ:
			begin
				if(block_read_req_ack)
				begin
					state <= S_READ_END;
					block_read_req <= 1'b0;
				end
				else
				begin
					block_read_req <= 1'b1;
				end
			end

			// ================================================================
			//  结束状态：产生"完成"标志，然后回待命
			// ================================================================
			// 【为什么要有这两个单独的结束状态】
			// 因为 sd_sec_read_end / sd_sec_write_end 这两个输出信号，
			// 是用"当前状态"直接组合出来的（见上面的 assign 语句）。
			// 必须有一个独立的状态存在，才能让这两个信号拉高一个时钟，
			// 上层才能收到"完成"通知。
			S_WRITE_END:
			begin
				state <= S_WAIT_READ_WRITE;
			end
			S_READ_END:
			begin
				state <= S_WAIT_READ_WRITE;
			end

			// ================================================================
			//  出错了 → 回空闲（会重新走一遍完整初始化）
			// ================================================================
			default:
				state <= S_IDLE;
		endcase
end
endmodule
