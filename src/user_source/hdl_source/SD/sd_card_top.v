
//==============================================================================
//  sd_card_top.v  —— SD 卡驱动层顶层（把命令层和物理层组装起来）
//==============================================================================
//
//  【这个文件干什么的】
//  它自己不做任何逻辑，只是一个"装配工"：把三个子模块连起来，
//  然后对外提供一个非常简单的接口。你可以把它理解成"SD 卡驱动的门面"。
//
//  【对外提供的接口只有 5 类】
//    1. sd_init_done        → 告诉你"卡初始化好了，可以用了"
//    2. sd_sec_read + 地址  → "帮我读第 N 个扇区"
//    3. sd_sec_read_data(_valid/_end) → 数据一个字节一个字节吐出来
//    4. sd_sec_write + 地址 + 数据 → "帮我写第 N 个扇区"（本工程禁用）
//    5. SD_nCS/DCLK/MOSI/MISO → 直接连到 TF 卡的物理引脚
//
//  【三个子模块的分工（自顶向下）】
//    sd_card_sec_read_write  ← 协议层：发什么命令、按什么顺序发
//         ↓ (通过 spi_wr_req / spi_data_in 等)
//    sd_card_cmd             ← 命令层：把命令拆成字节、校验响应
//         ↓ (通过 nCS_ctrl / clk_div / spi_* )
//    spi_master              ← 物理层：真正产生时钟、移出/移入比特
//         ↓
//      TF 卡引脚
//
//  【SD 卡的两种工作模式】
//  SD 卡支持 SD 总线模式（4 线并行）和 SPI 模式（串行）。
//  本工程用 SPI 模式，因为引脚少、时序简单，FPGA 实现起来容易。
//  代价是速度慢一些（本工程 SPI 时钟 12.5MHz 左右，
//  按 512 字节一个扇区算，读一个扇区约 330us）。
//
//  【SPI 时钟分频参数说明】
//  文档注释里给了公式：SPI 时钟 = clk / ((DIV + 2) × 2)
//    SPI_LOW_SPEED_DIV  = 248 → 100MHz / (250 × 2) = 200kHz
//      【为什么初始化要用这么慢的速度】因为 SD 卡规范规定初始化阶段
//      （CMD0~CMD41）SPI 时钟必须在 100kHz ~ 400kHz 之间。
//      卡还没配置好之前跑太快会通信失败。
//    SPI_HIGH_SPEED_DIV = 0   → 100MHz / (2 × 2) = 25MHz
//      【为什么初始化后就能跑这么快】CMD41 之后卡进入了 SPI 模式，
//      已经"认识"了主机，就可以全速跑了。不过 25MHz 对 SD 卡来说
//      已经接近上限，实际受走线质量影响，可能会降速。
//
//==============================================================================
module sd_card_top
#(
	parameter  SPI_LOW_SPEED_DIV = 248,         // 初始化阶段的 SPI 分频（慢速 200kHz）
	parameter  SPI_HIGH_SPEED_DIV = 0           // 初始化完成后的分频（快速 25MHz）
)
(
	input            clk,                       // 时钟，100MHz
	input            rst,                       // 复位，高有效
	output           SD_nCS,                    // TF 卡片选（SPI 模式）
	output           SD_DCLK,                   // TF 卡 SPI 时钟
	output           SD_MOSI,                   // 主机→卡 数据
	input            SD_MISO,                   // 卡→主机 数据
	output           sd_init_done,              // 初始化完成
	input            sd_sec_read,               // 请求读扇区
	input[31:0]      sd_sec_read_addr,          // 要读哪个扇区
	output[7:0]      sd_sec_read_data,          // 读出的字节
	output           sd_sec_read_data_valid,    // 字节有效
	output           sd_sec_read_end,           // 整个扇区读完
	input            sd_sec_write,              // 请求写扇区
	input[31:0]      sd_sec_write_addr,         // 写哪个扇区
	input[7:0]       sd_sec_write_data,         // 要写的字节
	output           sd_sec_write_data_req,     // 请求下一个写字节
	output           sd_sec_write_end           // 写扇区完成
);

//==============================================================================
//  内部连线：连接三个子模块
//==============================================================================

// ---- spi_master 的时钟分频参数（由命令层动态切换）----
wire[15:0]           spi_clk_div;               // SPI 分频值

// ---- 命令层 ↔ 协议层：命令请求与响应 ----
wire                 cmd_req;                   // 请求发出一条命令
wire                 cmd_req_ack;               // 命令发送完成
wire                 cmd_req_error;             // 命令执行出错
wire[47:0]           cmd;                       // 48bit 命令内容（命令码 + 参数 + CRC）
wire[7:0]            cmd_r1;                    // 期望收到的响应字节
wire[15:0]           cmd_data_len;              // 命令附带数据的长度

// ---- 协议层 ↔ 命令层：扇区数据搬运 ----
wire                 block_read_req;            // 请求读一块数据
wire                 block_read_valid;          // 读到的字节有效
wire[7:0]            block_read_data;           // 读到的字节
wire                 block_read_req_ack;        // 读完成应答
wire                 block_write_req;           // 请求写一块数据
wire[7:0]            block_write_data;          // 要写的字节
wire                 block_write_data_rd;       // 请求下一个写字节
wire                 block_write_req_ack;       // 写完成应答

// ---- 命令层 ↔ 物理层：SPI 收发 ----
wire                 nCS_ctrl;                  // 片选控制
wire                 spi_wr_req;                // 请求发送一个字节
wire                 spi_wr_ack;                // 字节发送完成
wire[7:0]            spi_data_in;               // 要发送的字节
wire[7:0]            spi_data_out;              // 收到的字节
wire[15:0]           clk_div;                   // 物理层实际使用的分频值

//==============================================================================
//  子模块 1：协议层（负责"发什么命令"）
//==============================================================================
// 这个模块里有一个大状态机，按 SD 卡规范要求的顺序完成初始化，
// 之后响应上层"读扇区/写扇区"的请求。
sd_card_sec_read_write
#(
	.SPI_LOW_SPEED_DIV(SPI_LOW_SPEED_DIV),
	.SPI_HIGH_SPEED_DIV(SPI_HIGH_SPEED_DIV)
)
sd_card_sec_read_write_m0(
	.clk                            (clk                    ),
	.rst                            (rst                    ),
	.sd_init_done                   (sd_init_done           ),
	.sd_sec_read                    (sd_sec_read            ),
	.sd_sec_read_addr               (sd_sec_read_addr       ),
	.sd_sec_read_data               (sd_sec_read_data       ),
	.sd_sec_read_data_valid         (sd_sec_read_data_valid ),
	.sd_sec_read_end                (sd_sec_read_end        ),
	.sd_sec_write                   (sd_sec_write           ),
	.sd_sec_write_addr              (sd_sec_write_addr      ),
	.sd_sec_write_data              (sd_sec_write_data      ),
	.sd_sec_write_data_req          (sd_sec_write_data_req  ),
	.sd_sec_write_end               (sd_sec_write_end       ),
	.spi_clk_div                    (spi_clk_div            ),   // 输出：要用的 SPI 速度
	.cmd_req                        (cmd_req                ),   // 输出：发命令
	.cmd_req_ack                    (cmd_req_ack            ),   // 输入：发完了
	.cmd_req_error                  (cmd_req_error          ),   // 输入：出错了
	.cmd                            (cmd                    ),   // 输出：命令内容
	.cmd_r1                         (cmd_r1                 ),   // 输出：期望响应
	.cmd_data_len                   (cmd_data_len           ),   // 输出：附带数据长度
	.block_read_req                 (block_read_req         ),
	.block_read_valid               (block_read_valid       ),
	.block_read_data                (block_read_data        ),
	.block_read_req_ack             (block_read_req_ack     ),
	.block_write_req                (block_write_req        ),
	.block_write_data               (block_write_data       ),
	.block_write_data_rd            (block_write_data_rd    ),
	.block_write_req_ack            (block_write_req_ack    )
);

//==============================================================================
//  子模块 2：命令层（负责"命令怎么发"）
//==============================================================================
// 把 48bit 命令按字节拆开、通过 SPI 一个个发出去，
// 然后不断读响应字节，直到读到期望的响应值为止。
sd_card_cmd sd_card_cmd_m0(
	.sys_clk                        (clk                    ),
	.rst                            (rst                    ),
	.spi_clk_div                    (spi_clk_div            ),
	.cmd_req                        (cmd_req                ),
	.cmd_req_ack                    (cmd_req_ack            ),
	.cmd_req_error                  (cmd_req_error          ),
	.cmd                            (cmd                    ),
	.cmd_r1                         (cmd_r1                 ),
	.cmd_data_len                   (cmd_data_len           ),
	.block_read_req                 (block_read_req         ),
	.block_read_req_ack             (block_read_req_ack     ),
	.block_read_data                (block_read_data        ),
	.block_read_valid               (block_read_valid       ),
	.block_write_req                (block_write_req        ),
	.block_write_data               (block_write_data       ),
	.block_write_data_rd            (block_write_data_rd    ),
	.block_write_req_ack            (block_write_req_ack    ),
	.nCS_ctrl                       (nCS_ctrl               ),   // 输出：片选
	.clk_div                        (clk_div                ),   // 输出：给物理层的分频
	.spi_wr_req                     (spi_wr_req             ),   // 输出：请求发字节
	.spi_wr_ack                     (spi_wr_ack             ),   // 输入：发完了
	.spi_data_in                    (spi_data_in            ),   // 输出：发什么
	.spi_data_out                   (spi_data_out           )    // 输入：收到什么
);

//==============================================================================
//  子模块 3：物理层 SPI 主机
//==============================================================================
// 真正产生时钟边沿、移出/移入比特的地方。与 SD 卡无关，是个通用 SPI 收发器。
// 【CPOL=1, CPHA=1 是什么】这是 SPI 的"模式 3"：
//   CPOL（Clock Polarity）= 1 → 空闲时时钟为高电平
//   CPHA（Clock Phase）= 1   → 在时钟的第二个边沿（上升沿）采样数据
// SD 卡在 SPI 模式下要求的就是模式 0 或模式 3，本工程用模式 3。
spi_master spi_master_m0(
	.sys_clk                        (clk                    ),
	.rst                            (rst                    ),
	.nCS                            (SD_nCS                 ),   // → TF 卡引脚
	.DCLK                           (SD_DCLK                ),
	.MOSI                           (SD_MOSI                ),
	.MISO                           (SD_MISO                ),
	.clk_div                        (clk_div                ),
	.CPOL                           (1'b1                   ),   // 空闲高电平
	.CPHA                           (1'b1                   ),   // 第二边沿采样
	.nCS_ctrl                       (nCS_ctrl               ),
	.wr_req                         (spi_wr_req             ),
	.wr_ack                         (spi_wr_ack             ),
	.data_in                        (spi_data_in            ),
	.data_out                       (spi_data_out           )
);
endmodule
