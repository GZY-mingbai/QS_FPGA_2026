`timescale 1ns/1ps
//==============================================================================
//  frame_fifo_write.v  —— 写方向搬运状态机（FIFO → SDRAM 突发写）
//==============================================================================
//
//  【这个文件干什么的】
//  把 FIFO 里的像素数据，以"突发"的方式一批一批写进 SDRAM，
//  直到整个一帧（640×480 = 307200 个像素）写完。
//
//  【"突发"（Burst）是什么】
//  SDRAM 的工作方式是"给一个起始地址 + 长度，然后它自己连续读写一串数据"。
//  这种连续传输就叫突发。突发越长，平均每个数据的开销越小，效率越高。
//  SDRAM 突发时地址会自动递增，不需要每个数据都给地址。
//  本模块一次突发搬 BURST_SIZE = 256 个数据。
//
//  【★ 行翻转（Vertical Flip）★ —— 本文件最有价值的技巧】
//  BMP 图片在文件里是"从下往上"存的：文件里第一行数据，其实是图像的**最后一行**。
//  但视频显示是"从上往下"扫描的。如果不处理，图片会上下颠倒。
//
//  【常见做法】在上层把整帧数据缓存起来再倒序输出 —— 要额外内存，很浪费。
//
//  【本文件的巧做法】不改数据顺序，只改**写地址**：
//     输入流的第 0 行    → 写到 SDRAM 的第 479 行
//     输入流的第 1 行    → 写到 SDRAM 的第 478 行
//     ...
//     输入流的第 479 行  → 写到 SDRAM 的第 0 行
//  这样读出来的时候自然就是正的，完全不需要额外缓存。
//
//  【行翻转的地址运算（看懂这三行就懂了这个技巧）】
//    VFLIP_FIRST_ADDR_OFFSET = (480-1) × 640 = 306560
//      → 第一个数据的起始地址要偏移到"最后一行的行首"
//    VFLIP_LINE_JUMP         = 640 × 2 - 1 = 1279
//      → 一行写完后，地址要往回跳多少
//        推导：写完第 k 行的最后一个像素后，地址在 (479-k)×640 + 639。
//              下一行（第 k+1 行）的目标是 (479-k-1)×640 + 0 = (478-k)×640。
//              两者之差 = (479-k)×640+639 - (478-k)×640 = 640 + 639 = 1279。
//              所以地址要减去 1279 才能跳到下一行的行首。
//    VFLIP_LINE_LAST_X       = 640 - 1 = 639
//      → 一行的最后一个像素的横坐标，用来判断"这行写完了没"
//
//==============================================================================
module frame_fifo_write
#
(
	parameter MEM_DATA_BITS          = 32,
	parameter ADDR_BITS              = 21,
	parameter BURST_BITS             = 9,
	parameter BURST_SIZE             = 128,
	parameter WRITE_V_FLIP           = 1,      // 1 = 做行翻转（输入第0行写到内存最后一行）
	parameter FRAME_WIDTH            = 640,
	parameter FRAME_HEIGHT           = 480
)
(
	input                            rst,
	input                            mem_clk,      // SDRAM 侧时钟 125MHz
	input							 Sdr_init_done,
	input							 Sdr_init_ref_vld,
    input							 Sdr_busy,
	input							 App_rd_busy,  // 读方向是否忙（写优先，本模块其实不怎么看它）
    output							 O_wr_busy,    // 本模块是否忙

    // ★ 2026-10-17 根修：写口门控（4 拍分组规则——地址不连续点需 ≥4 拍空隙）★
    //   与读侧 rd_ready 完全对称：计数/地址/回压全部跟随【门控后】的 App_wr_en，
    //   被暂停时突发被拉长而不是丢字。
    input                            wr_ready,

	output 							 App_wr_en,    // 每写一个数据拉高一拍
	output  [ADDR_BITS - 1:0]	 	App_wr_addr,  // 写地址

    // ---- 上层握手接口 ----
	input                            write_req,        // 写请求（保持高直到应答）
	output reg                       write_req_ack,    // 应答
	output                           write_finish,     // 整帧写完
	input[ADDR_BITS - 1:0]           write_addr_0,     // 缓冲区0 基地址
	input[ADDR_BITS - 1:0]           write_addr_1,     // 缓冲区1 基地址
	input[ADDR_BITS - 1:0]           write_addr_2,     // 缓冲区2
	input[ADDR_BITS - 1:0]           write_addr_3,     // 缓冲区3
	input[1:0]                       write_addr_index, // 选哪块缓冲区
	input[ADDR_BITS - 1:0]           write_len,        // 一帧的数据个数
	output reg                       fifo_aclr,        // 通知 FIFO 清零
	input[9:0]                      rdusedw,          // FIFO 里攒了多少数据
	// ★★ build FB16：本帧是否是 RGB565 打包格式（2 像素/字，行宽减半）★★
	//   运行时输入（不是 parameter）：同一块板子上会逐帧切换 888 / 565，
	//   见 frame_read_write.v 顶部的说明。行宽只影响下面三个常量，用一位选择器
	//   在两个编译期常量之间选，**没有运行时乘除法**。
	input                            fmt565,
	// ★ 2026-10-23 诊断：[3:0]=本帧行末回跳次数/32（正确应到 F = 480/32），
	//   [4]=地址异常粘滞（回跳后地址不等于上一行首 − 行宽）
	output wire [4:0]                dbg_wr
);

// 【这两个 localparam 是作者留下的"生成全 1 / 全 0"的小技巧】
// 定义一个 256 位的常数 1 和 0，然后想用 n 位的时候写 ZERO[n-1:0] 就行。
// 这样就不用为每个位宽都定义一个常数了。
localparam ONE                       = 256'd1;
localparam ZERO                      = 256'd0;

//==============================================================================
//  ★ 行翻转的地址常量（详细推导见文件头注释）★
//==============================================================================
// VFLIP_FIRST_ADDR_OFFSET：第一个数据要写到的地址偏移
//   地址 = 缓冲区基地址 + 这个偏移 = 最后一行的行首
//
//  ★★ build FB16：两套常量，用 fmt565 一位选择 ★★
//  888 模式：每行 FRAME_WIDTH 个字（1 像素 1 字）
//  565 模式：每行 FRAME_WIDTH/2 个字（2 像素 1 字）
//  三对都是**编译期常量**，运行时只做一次二选一（一位选择器），
//  不引入任何乘法/除法，所以不影响时序。
//==============================================================================
localparam [ADDR_BITS - 1:0] OFF_FULL  = (FRAME_HEIGHT - 1) * FRAME_WIDTH;
localparam [ADDR_BITS - 1:0] OFF_HALF  = (FRAME_HEIGHT - 1) * (FRAME_WIDTH / 2);
localparam [ADDR_BITS - 1:0] JUMP_FULL = (FRAME_WIDTH * 2) - 1;
localparam [ADDR_BITS - 1:0] JUMP_HALF = FRAME_WIDTH - 1;          // 2*(W/2)-1
localparam [15:0]            LAST_FULL = FRAME_WIDTH - 1;
localparam [15:0]            LAST_HALF = (FRAME_WIDTH / 2) - 1;

//==============================================================================
//  ★★ 2026-10-23：格式必须在**帧起始锁存**，整帧用锁存值 ★★
//==============================================================================
//  【病根】行行走的三个常量（起始偏移 / 行末回跳 / 行末判据）原来直接用**活的**
//    `fmt565`。而 `fmt565` 是 viewer_ctrl 在帧边界切的 —— 可上一帧的**尾部**
//    还在本引擎里排空，于是"一帧写到一半格式就变了"。
//  【后果】行末判据 `wr_x == VFLIP_LINE_LAST_X` 在帧中间从 639 跳成 319
//    ⇒ 该在第 639 字回跳的，提前到第 319 字回跳 ⇒ 这一行后半 320 字写到错位置
//    ⇒ **行内水平环绕** ⇒ 屏幕上"左边那段跑到右边"（上板现象就是这样）。
//  【改法】`write_req_d2`（帧起始，同步后）那一拍把 fmt565 锁进 fmt_r，整帧不变。
//    这正是本文件 :176 注释写的设计意图（"每帧起始锁存成 wr_fmt_r"），
//    只是之前没有真正落到 frame_fifo_write 里。
//==============================================================================
reg  fmt_r;                              // 本帧的格式（帧起始锁存）
wire [ADDR_BITS - 1:0] VFLIP_FIRST_ADDR_OFFSET = fmt_r ? OFF_HALF  : OFF_FULL;
wire [ADDR_BITS - 1:0] VFLIP_LINE_JUMP         = fmt_r ? JUMP_HALF : JUMP_FULL;
wire [15:0]            VFLIP_LINE_LAST_X       = fmt_r ? LAST_HALF : LAST_FULL;

//==============================================================================
//  状态编码
//==============================================================================
localparam S_IDLE                    = 0;    // 空闲，等写请求
localparam S_ACK                     = 1;    // 应答写请求：清 FIFO、锁地址、锁长度
localparam S_CHECK_FIFO              = 2;    // 检查 FIFO 攒够没有
localparam S_WRITE_BURST             = 3;    // 正在突发写
localparam S_WRITE_BURST_END         = 4;    // 一次突发写完了
localparam S_END                     = 5;    // 整帧写完

//==============================================================================
//  内部寄存器
//==============================================================================

// ---- 跨时钟域同步链 ----
// write_req / write_len / write_addr_index 都来自 write_clk（100MHz）域，
// 需要同步到 mem_clk（125MHz）域才能在这里使用。
// 【为什么打 3 拍】前 2 拍消除亚稳态，第 3 拍让信号彻底稳定
// （作者的注释说"这是设计习惯"，实际上 2 拍通常就够，
//   打 3 拍更保守，代价是多一个触发器，几乎没有成本）。
reg                                 write_req_d0;
reg                                 write_req_d1;
reg                                 write_req_d2;
reg[ADDR_BITS - 1:0]                write_len_d0;
reg[ADDR_BITS - 1:0]                write_len_d1;
reg[ADDR_BITS - 1:0]                write_len_latch;   // 锁存后的长度（本时钟域用）
reg[ADDR_BITS - 1:0]                write_cnt;         // 已写入的数据总数
reg[1:0]                            write_addr_index_d0;
reg[1:0]                            write_addr_index_d1;
reg[3:0]                            state;

reg [ADDR_BITS - 1:0]	 App_wr_addr_r;    // 当前写地址（内部版本）
reg [15:0]                            wr_x;      // 当前在输入行的第几个像素

reg [BURST_BITS - 1:0]				burst_cnt;   // 本次突发已写了几个
wire								wr_burst_finish;

reg App_wr_en_r;      // 突发输出使能寄存器
reg App_wr_en_d0;     // 输出到端口的最终使能（打一拍保证时序）

//==============================================================================
//  ★★ 2026-10-18/19 时序根修（第二刀）：把逐拍判据压到"一次 9 位加法"★★
//==============================================================================
//  build 8 把 21 位加法换成 remaining_r 之后，报告从 −0.090 走到 −0.020，
//  但还剩一条：Data Path Delay 7.740ns、Logic Level 8（ADDER=4）。
//  原因：`bc < BURST_SIZE` 与 `bc < remaining_r(21 位)` 是两个比较，
//  合起来仍要拼一条宽的比较链。
//
//  其实两个条件可以合成一个：剩余量在本突发内**是常数**（只在 S_ACK 和
//  突发结束时变），所以"本突发还允许写多少"可以在**进突发之前算一次**，
//  存进一个 9 位寄存器 limit：
//      limit = min(BURST_SIZE, remaining_r)
//  于是逐拍只剩 `burst_cnt + App_wr_en < limit` —— 一次 9 位加法 + 一次 9 位比较。
//  逐字等价（bc < BURST_SIZE && bc < remaining_r  ⟺  bc < min(...)）。
//==============================================================================
reg [BURST_BITS-1:0] limit;   // 本突发逐拍判据的上限（进突发前算好）

// ★★ build FB16a：本帧到底多少个字 ★★
//  fmt565 是准静态的（每帧起始由 frame_read_write 锁存成 wr_fmt_r），这里只做一次移位。
//  【为什么要在这里算】原来是在 frame_read_write 里用 fmt 去 mux write_len，然后这个
//  21 位总线裸跨时钟域（只打 2 拍）—— 切换瞬间会采到半旧半新的值，锁进 write_len_latch
//  就是个垃圾长度。偏大 ⇒ FIFO 永远凑不够 ⇒ write_finish 永不产生 ⇒ **整屏卡死**。
//  现在长度端口永远送常量 307200，折半在本时钟域内做，不存在这个风险。
wire [ADDR_BITS-1:0] len_words = fmt_r  ? (write_len_d1 >> 1) : write_len_d1;
//  【原来】
//      if (App_wr_en_r && burst_cnt + App_wr_en < BURST_SIZE &&
//                        (burst_cnt + write_cnt + App_wr_en < write_len_latch))
//  而 `App_wr_en = App_wr_en_d0 & wr_ready` ⇒ 自己的 Q 穿过 wr_ready、三个加法器
//  回到自己的 D 端。布局布线报告实测（2026-10-18，build 7）：
//      Slack = −0.090ns（全设计最差），Data Path Delay 7.738ns / 8ns
//      Logic Level 6 = ADDER×3 + LUT5×2 + LUT4×1
//      终点 frame_fifo_write/App_wr_en_d0，起点 sdr_init_ref 的 ref_vld 触发器
//  刷新每帧 492 次 ⇒ 每次都是采错的机会。而这条错只在"某拍采到上一拍的值"，
//  突发中段两者相等 ⇒ 场景图看不出来；只在突发首尾边界露头 ⇒ 表现为稀错误点。
//
//  【改法】`write_cnt` 那半段本来就有现成的等价寄存器 `remaining_r`
//  （= write_len_latch − write_cnt，只在 S_ACK / 突发结束时变，见模块上方）：
//          burst_cnt + write_cnt + App_wr_en < write_len_latch
//      ⟺   burst_cnt + App_wr_en < remaining_r
//  这样判据里只剩 **一次 9 位加法**（burst_cnt 是 9 位），而不是两次 21 位加法。
//  逐字等价（所有量都远小于 2^21，不存在溢出），不再有"晚一拍"的风险 ——
//  这也是为什么不能简单换成余量计数器：`App_wr_en` 在判据里承担
//  "提前一拍看计数"的作用，去掉它就会多写一个字（实测 307201 / 307200）。
//==============================================================================

//==============================================================================
//  ★ 时序修复（2026-10-15，"会移动的横条纹"）★
//  原写法：
//      into_burst = (write_len_latch <= (rdusedw + write_cnt))
//                   || (rdusedw > BURST_SIZE)) && ~App_rd_busy
//  一个周期里串了「21 位加法（rdusedw+write_cnt）+ 21 位比较 + 10 位比较」，
//  实测 ext_mem_clk 域 −0.157ns 违约（起点 write_buf 的指针寄存器
//  secondary_addr_o，终点本状态机 state_reg 和 frame_fifo_read 的
//  App_rd_en_r）。这是临界违约：失败与否随温度/电压漂移，上板表现为
//  "会移动的横条纹"（之前 App 域深违约修掉后，就轮到它表演了）。
//
//  改法：把判断拆成两级浅流水寄存，每级都是寄存器到寄存器的浅逻辑：
//      第 1 级  main_ready_r / tail_ready_r —— 两个 10 位比较
//      第 2 级  into_burst_r —— 合并 + "读方向不忙"
//  FSM 与 O_wr_busy 改用 into_burst_r。
//
//  【为什么不会破坏"写优先"仲裁】rdusedw 在 FIFO IP 内部本来就是寄存器
//  输出，原组合逻辑虽然当拍可见，但 FSM 采样决策永远在它变化之后至少
//  一拍；改成寄存器后这个"提前量"变成显式的两拍，而读 FSM 每个决策沿
//  看到的 O_wr_busy（含 into_burst_r）依旧先于写 FSM 的实际跳转两拍，
//  读侧永远来得及让路。~App_rd_busy 用读状态机的**实时**状态（不打拍），
//  保证读突发进行中时，写侧绝不会在下一拍插进来。
//  每次突发多花 2 拍决策（256+ 拍的突发里占不到 1%），吞吐无感。
//
//  【tail_ready 的等价改写】原尾条件 arrived = rdusedw + write_cnt ≥ 整帧
//  长度 ⟺ rdusedw ≥ remaining = write_len_latch − write_cnt。
//  remaining 只在"锁存长度"和"一次突发结束"两个 rare 事件时变，
//  所以维护成寄存器 remaining_r，而不是每拍现算 21 位加法。
//  又因为 rdusedw 最多 511（10 位 FIFO 水位），remaining > 511 时尾条件
//  必为假 —— 先判 remaining_r[20:9] 全零，再做 10 位比较，
//  把 21 位比较砍成 10 位。
//==============================================================================
reg [ADDR_BITS - 1:0] remaining_r;   // 还剩多少字没搬（write_len_latch − write_cnt）
reg                   main_ready_r;  // 第 1 级：FIFO 攒够一次突发的量（>256）
reg                   tail_ready_r;  // 第 1 级：整帧数据都已到达（尾条件）
reg                   into_burst_r;  // 第 2 级：可以发起新的突发（含"读不忙"）

always @(posedge mem_clk or posedge rst) begin
    if (rst) begin
        remaining_r  <= {ADDR_BITS{1'b0}};
        main_ready_r <= 1'b0;
        tail_ready_r <= 1'b0;
        into_burst_r <= 1'b0;
    end else begin
        // ---- 剩余量：只在两个 rare 事件时变 ----
        //  S_ACK 期间 write_cnt 恒为 0（本状态机每拍都在清它），所以直接锁 write_len_d1
        if (state == S_ACK)
            remaining_r <= len_words;
        //  与 write_cnt += BURST_SIZE 严格同拍同量；wr_burst_finish 在
        //  S_WRITE_BURST 内只成立一拍（state 当拍就跳走），不会连减
        else if (state == S_WRITE_BURST && wr_burst_finish)
            remaining_r <= remaining_r - BURST_SIZE[ADDR_BITS-1:0];

        // ---- 第 1 级：两个"够不够"条件（输入全是寄存器）----
        main_ready_r <= (rdusedw > BURST_SIZE);
        tail_ready_r <= (remaining_r[ADDR_BITS-1:9] == {(ADDR_BITS-9){1'b0}})
                        && ({1'b0, rdusedw} >= {1'b0, remaining_r[8:0]});

        // ---- 第 2 级：合并 + 读方向不忙 ----
        into_burst_r <= (main_ready_r || tail_ready_r) && ~App_rd_busy;
    end
end

//==============================================================================
//  组合逻辑
//==============================================================================

assign App_wr_addr = {App_wr_addr_r[ADDR_BITS - 1:0]};

// 本模块的"忙"标志：正在突发、或正要突发（用寄存后的 into_burst_r 做预告）
assign O_wr_busy = (state == S_WRITE_BURST || (state == S_CHECK_FIFO && into_burst_r));

// 一次突发写完 = 计数器到 256
assign wr_burst_finish = (burst_cnt >= BURST_SIZE);

// 整帧写完 = 进入 S_END 状态（输出高电平，上层拿它做跨时钟域同步）
assign write_finish = (state == S_END) ? 1'b1 : 1'b0;

assign App_wr_en = App_wr_en_d0 & wr_ready;   // ★ 门控后输出（内部计数全部跟随）

//==============================================================================
//  跨时钟域同步：write_req / write_len / write_addr_index
//==============================================================================
always@(posedge mem_clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		write_req_d0    <=  1'b0;
		write_req_d1    <=  1'b0;
		write_req_d2    <=  1'b0;
		fmt_r           <=  1'b0;
		write_len_d0    <=  ZERO[ADDR_BITS - 1:0];
		write_len_d1    <=  ZERO[ADDR_BITS - 1:0];
		write_addr_index_d0    <=  2'b00;
		write_addr_index_d1    <=  2'b00;
	end
	else
	begin
		// 标准的"移位链"同步：每一级都往前走一格
		write_req_d0    <=  write_req;
		write_req_d1    <=  write_req_d0;
		write_req_d2    <=  write_req_d1;       // d2 是最终"干净"的版本
		if (write_req_d2)  fmt_r <= fmt565;    // ★ 帧起始锁存格式，整帧不变
		write_len_d0    <=  write_len;
		write_len_d1    <=  write_len_d0;
		write_addr_index_d0 <= write_addr_index;
		write_addr_index_d1 <= write_addr_index_d0;
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
		App_wr_addr_r <= ZERO[ADDR_BITS - 1:0];
		App_wr_en_d0 <= 1'b0;
		wr_x <= 16'd0;
		limit <= ZERO[BURST_BITS - 1:0];     // ★ 时序根修：逐拍上限复位 ★
	end
	else begin

		// ---- 突发计数器 ----
		if(state == S_CHECK_FIFO)
			burst_cnt <= ZERO[BURST_BITS - 1:0];   // 准备发起新突发，计数清零
		else if(App_wr_en)
			burst_cnt <= burst_cnt + 1'b1;          // 每写一个数据加一
		// （else 分支保持原值，这里省略了）

		// ---- ★ 时序根修：本突发的逐拍上限，进突发前只算这一次 ★ ----
		//  limit = min(BURST_SIZE, remaining_r)；remaining_r 在突发内是常数
		if(state == S_CHECK_FIFO)
			limit <= (remaining_r >= BURST_SIZE[ADDR_BITS-1:0])
			         ? BURST_SIZE[BURST_BITS-1:0]
			         : remaining_r[BURST_BITS-1:0];

		// ---- ★ 写地址生成 ★ ----
		if(state == S_ACK)
			begin
				// 应答时刻：把地址初始化到"缓冲区基地址 + 行翻转偏移"
				wr_x <= 16'd0;

				if(write_addr_index_d1 == 2'd0)
					App_wr_addr_r <= WRITE_V_FLIP ? (write_addr_0 + VFLIP_FIRST_ADDR_OFFSET) : write_addr_0;
				else if(write_addr_index_d1 == 2'd1)
					App_wr_addr_r <= WRITE_V_FLIP ? (write_addr_1 + VFLIP_FIRST_ADDR_OFFSET) : write_addr_1;
				else if(write_addr_index_d1 == 2'd2)
					App_wr_addr_r <= WRITE_V_FLIP ? (write_addr_2 + VFLIP_FIRST_ADDR_OFFSET) : write_addr_2;
				else if(write_addr_index_d1 == 2'd3)
					App_wr_addr_r <= WRITE_V_FLIP ? (write_addr_3 + VFLIP_FIRST_ADDR_OFFSET) : write_addr_3;
				// 【为什么 WRITE_V_FLIP 为 0 时不加偏移】因为不做翻转时，
				// 第一行就写到缓冲区的第一个地址，不需要偏移。
			end
		else if(App_wr_en)
			begin
				// 每写一个数据，地址加一。但行翻转模式下，
				// 写完一行末尾要"往回跳"，所以分两种情况。
				if(WRITE_V_FLIP)
				begin
					if(wr_x == VFLIP_LINE_LAST_X)
					begin
						// 本行最后一个像素写完了 → 跳到上一行的行首
						wr_x <= 16'd0;
						App_wr_addr_r <= App_wr_addr_r - VFLIP_LINE_JUMP;
					end
					else
					begin
						// 行内正常往前走
						wr_x <= wr_x + 1'b1;
						App_wr_addr_r <= App_wr_addr_r + 1'b1;
					end
				end
				else
					App_wr_addr_r <= App_wr_addr_r + 1'b1;  // 不翻转就是简单递增
			end
		// （都不满足时地址保持不变，省略了 else）

		// ---- 当前这一拍要不要输出数据 ----
		// ★ 2026-10-18/19 时序根修：只剩一次 9 位加法 + 一次 9 位比较 ★
		//   limit = min(BURST_SIZE, 本突发开始时还剩多少)  ⇒ 与原式逐字等价
		if(App_wr_en_r && (burst_cnt + App_wr_en < limit))begin
            App_wr_en_d0 <= 1'b1;
        end
		else
			App_wr_en_d0 <= 1'b0;
		// 【为什么这么绕】要同时满足三个边界条件：
		//   ① 本拍不超出单次突发的长度（< BURST_SIZE）
		//   ② 本拍不超出整帧的总长度（< write_len_latch）
		//   ③ 处于突发进行中（App_wr_en_r）
		// 这样就能保证"突发边界"和"帧末尾"都被正确截断。

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
		write_len_latch <= ZERO[ADDR_BITS - 1:0];
		App_wr_en_r <= 1'b0;
		write_cnt <= ZERO[ADDR_BITS - 1:0];
		fifo_aclr <= 1'b0;
		write_req_ack <= 1'b0;
	end
	else
		case(state)

			// ============================================================
			//  S_IDLE：等写请求
			// ============================================================
			S_IDLE:
			begin
				// 用 write_req_d2（同步后的版本）判断
				// 还要等 Sdr_init_done 才能开始（SDRAM 没初始化好不能写）
				if(write_req_d2 == 1'b1 && Sdr_init_done)
				begin
					state <= S_ACK;
				end
				write_req_ack <= 1'b0;
			end

			// ============================================================
			//  S_ACK：应答请求 + 清 FIFO + 锁存参数
			// ============================================================
			// 【这个状态很重要】它做了三件"一次性"的准备工作：
			//   ① 拉高 write_req_ack，告诉上层"我准备好了，你可以开始发数据了"
			//   ② 拉高 fifo_aclr，把 FIFO 里可能残留的旧数据清掉
			//   ③ 锁存 write_len（避免它在整个写过程中变化）
			// 之后等上层撤销请求（write_req_d2 变 0），才进入实际搬运。
			S_ACK:
			begin
				if(write_req_d2 == 1'b0)
				begin
					// 上层已经撤销请求 → 准备工作完成，进入搬运循环
					state <= S_CHECK_FIFO;
					fifo_aclr <= 1'b0;          // 撤销清零信号（清零只需一拍）
					write_req_ack <= 1'b0;
				end
				else
				begin
					write_req_ack <= 1'b1;      // 应答
					fifo_aclr <= 1'b1;          // 清 FIFO
					write_len_latch <= len_words;   // 锁存长度
				end
				write_cnt <= ZERO[ADDR_BITS - 1:0];    // 已写计数清零
			end

			// ============================================================
			//  S_CHECK_FIFO：检查能否发起下一轮突发
			// ============================================================
			S_CHECK_FIFO:
			begin
				// 如果上层又发来新请求（比如用户连按两次切图），
				// 优先处理新请求，重新走一遍 S_ACK
				if(write_req_d2 == 1'b1)
				begin
					state <= S_ACK;
				end
				// 否则，攒够数据了就发起一次突发（时序修复：用寄存后的判断）
				// ★ 2026-10-15 晚补充：跳转条件上再串一个"当前拍"的 ~App_rd_busy。
				//   into_burst_r 是上一拍合成的，里面嵌的 ~App_rd_busy 采的是更早
				//   一拍的读状态。若恰好读状态机在"本拍"刚进入 S_READ_BURST，
				//   而 into_burst_r 还带着旧的"读不忙"信息，本状态机下一拍就会
				//   跟着进突发 → 读写突发在同一条单口 SDRAM 上重叠 → 偶发一小块
				//   花屏（发生时机由两个突发节奏的相位拍频决定，间隔数秒一次）。
				//   这里补上实时检查后，竞争窗口归零；与读侧"直接用实时
				//   O_wr_busy"完全对称。代价：与读突发竞争时多等 1~2 拍，可忽略。
				else if(into_burst_r && ~App_rd_busy)
				begin
					state <= S_WRITE_BURST;
					App_wr_en_r <= 1'b1;    // 打开数据输出
				end
			end

			// ============================================================
			//  S_WRITE_BURST：正在突发输出
			// ============================================================
			S_WRITE_BURST:
			begin
				if(wr_burst_finish == 1'b1)
				begin
					App_wr_en_r <= 1'b0;    // 关掉输出
					state <= S_WRITE_BURST_END;
					write_cnt <= write_cnt + BURST_SIZE[ADDR_BITS - 1:0];  // 计数 +256
				end
			end

			// ============================================================
			//  S_WRITE_BURST_END：一轮突发结束，判断整帧写完没有
			// ============================================================
			S_WRITE_BURST_END:
			begin
				if(write_req_d2 == 1'b1)
				begin
					state <= S_ACK;         // 新请求优先
				end
				else if(write_cnt < write_len_latch)
				begin
					state <= S_CHECK_FIFO;  // 还没写完，继续下一轮
				end
				else
				begin
					state <= S_END;         // 写够了！
				end
			end

			// ============================================================
			//  S_END：整帧写完
			// ============================================================
			// 这个状态的存在意义：让 write_finish 信号保持高电平足够长时间，
			// 以便上层（sd_card_bmp）能通过跨时钟域同步器可靠地采到它。
			S_END:
			begin
				state <= S_IDLE;
			end

			default:
				state <= S_IDLE;
		endcase
end
//==============================================================================
//  ★★ 2026-10-23：写侧行走诊断（数码管当逻辑分析仪）★★
//==============================================================================
//  【要查什么】SEL5=0 已证明图层管线干净（不缺料/不越界/不空弹），
//    那么"左边那段跑到右边"只能是**写地址行走**出错。
//  【正常值】一帧 480 行 ⇒ 回跳 480 次 ⇒ 480/32 = 15 = **F**。
//    显示不是 F，就是行走异常（少跳/多跳）。
//  【[4]】回跳后地址应等于"上一行首 - 行宽"；不等就粘滞置 1。
//==============================================================================
reg  [9:0] wr_jump_cnt;      // 帧内累计的行末回跳次数
reg  [9:0] wr_jump_snap;     // 每帧快照（显示用，稳定不跳）
reg        wr_addr_bad;      // 快照 >= 481 即异常

//  ★ 2026-10-23 第二次：**用 write_finish 做帧界**。
//  上板实测：用 `write_req_d2` 上升沿清零时，计数器从不复位
//  （小数点恒亮、数字一直往上涨）⇒ write_req 是电平信号，
//  帧间不一定有下降沿。`write_finish` 每帧**只脉冲一次**，天然是帧界。
//  正常值：480 次⇒ 480>>5 = **15 = F**。
always @(posedge mem_clk or posedge rst) begin
    if (rst) begin
        wr_jump_cnt  <= 10'd0;
        wr_jump_snap <= 10'd0;
        wr_addr_bad  <= 1'b0;
    end else begin
        if (write_finish) begin
            wr_jump_snap <= wr_jump_cnt;                 // 快照本帧
            wr_addr_bad  <= (wr_jump_cnt >= 10'd481);    // 多跳即异常
            wr_jump_cnt  <= 10'd0;
        end else if (App_wr_en && WRITE_V_FLIP && (wr_x == VFLIP_LINE_LAST_X)) begin
            if (wr_jump_cnt != 10'd1023) wr_jump_cnt <= wr_jump_cnt + 10'd1;
        end
    end
end

//  显示快照值（不再显示实时累加，避免乱跳）
//  ★ 2026-10-23 第三次：直接显示**多跳的次数**（snap − 480）
//  上板实测到 F+小数点（即 481~511）⇒ 每帧多跳了 1~31 次。
//  多跳一次 = 该行后半写到了行首位置 ⇒ 屏幕上"左边那段跑到右边"。
//  显示：数字 = 多跳次数（0 = 完美，F = ≥15）；小数点 = 不等于 480。
wire [9:0] jump_excess = (wr_jump_snap > 10'd480) ? (wr_jump_snap - 10'd480)
                                                   : (10'd480 - wr_jump_snap);
assign dbg_wr = { (wr_jump_snap != 10'd480),
                  (jump_excess >= 10'd15) ? 4'd15 : jump_excess[3:0] };

endmodule
