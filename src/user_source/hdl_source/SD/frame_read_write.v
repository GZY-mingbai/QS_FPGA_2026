`timescale 1ns/1ps
//==============================================================================
//  frame_read_write.v  —— 帧缓存读写控制器（视频链路的"交通枢纽"）
//==============================================================================
//
//  【这个文件干什么的】
//  它是一个"翻译层"。上层的 sd_card_bmp 和 video_timing_data 只会说简单的话：
//     "我要写一帧"、"给我读一帧"
//  但 SDRAM 控制器需要的是"突发传输"（一次连续搬 256 个数据），
//  而且读写两端跑在不同频率的时钟上（写侧 100MHz、读侧 25MHz、SDRAM 侧 125MHz）。
//  这个模块负责把"简单的话"翻译成"SDRAM 能听懂的具体操作"。
//
//  【它内部有什么】
//    1. 两个异步 FIFO
//       - write_buf（wfifo_32_32_512）：写方向，把 100MHz 域的数据搬到 125MHz 域
//       - read_buf （rfifo_32_32_512）：读方向，把 125MHz 域的数据搬到 25MHz 域
//       【为什么必须用异步 FIFO】因为两端时钟频率不同、相位无关联。
//       用 FIFO 是最标准的跨时钟域大数据量传输方案（比打拍同步可靠得多）。
//    2. 两个搬运行状态机
//       - frame_fifo_write：从写 FIFO 取数据 → 突发写进 SDRAM
//       - frame_fifo_read ：从 SDRAM 突发读出 → 压进读 FIFO
//
//  【读写优先级】
//  板卡上的 SDRAM 是单口的——同一时刻只能读或者只能写，不能同时。
//  所以本模块让"写优先于读"：当两边同时想访问时，先让写。
//  这也是 App_rd_busy / App_wr_busy 两个信号存在的意义。
//
//  【双缓冲是怎么实现的】
//  注意 read_addr_index 和 write_addr_index 两个输入：
//  它们分别决定"读哪块缓冲区"和"写哪块缓冲区"。
//  上层（sd_card_bmp）保证这两个索引总是不同的，就实现了双缓冲。
//
//  【参数说明】
//  BURST_SIZE = 256 表示每次突发搬 256 个数据。
//  【为什么是 256】突发太长会占用 SDRAM 太久导致另一边饥饿；
//  太短则每次突发的开销（行激活、命令周期）占比太高，效率低。
//  256 是个经验折中值。
//
//==============================================================================
module frame_read_write
#
(
	parameter MEM_DATA_BITS          = 32,      // SDRAM 侧数据位宽
	parameter READ_DATA_BITS         = 32,      // 读侧数据位宽
	parameter WRITE_DATA_BITS        = 32,      // 写侧数据位宽
	parameter ADDR_BITS              = 21,      // 地址位宽（2^21 个 32bit 字 = 8MB）
	parameter BURST_BITS             = 9,       // 突发计数器的位宽
	parameter BURST_SIZE             = 128,     // 一次突发搬多少个数据
	// ★ 2026-10-17：BURST_SIZE 由厂商的 256 改为 128（读写两侧同时生效）★
	//  【上板证据】数码管第 2 位（读 FIFO 历史最小余量）读到 0，而该位只在
	//  有效像素期采样（read_en = de_d[18]，消隐期不取数）—— 排除消隐假象，
	//  **读 FIFO 在有效显示期真的被读空过**，画面上就是那条固定 x、每 4 行
	//  一次的"变亮"虚线（读空那一拍视频拿到存储体里的旧数据）。
	//  【为什么会读空】App 用 frame_writer 产帧 → 写突发连绵（O_wr_busy 频繁
	//  拉高），读突发的发起（S_CHECK_FIFO 要求 ~App_wr_busy）和执行
	//  （仲裁器 wr_act 单口纪律，写口每拍优先）都被写侧切碎。厂商的 256
	//  突发 / 512 深 FIFO 只给读侧留 512-256=256 字 ≈ 10µs 的视频量，
	//  跨不过"等写空隙 + 突发中途被写打断"的最坏组合。
	//  【为什么 128 能治】读阈值升到 512-128-INFLIGHT_MAX(8) = 376 字 ≈ 15µs；
	//  写突发时间也减半（wr_act 独占窗口缩短），读的呼吸空间双倍改善。
	//  【副作用核对】307200 % 128 == 0 ✓；读 FIFO 峰值 376+128+在途≤2 =
	//  506 < 512 ✓（INFLIGHT_MAX=8 见 frame_fifo_read）；写侧攒 128 就发，
	//  写 FIFO 峰值反而更低 ✓；SDRAM 行 2048 字，128 字突发不跨行 ✓。
	//  （厂商例程的写侧来自 SD 卡慢速读取，占空比低，所以 256 能活；
	//    本工程的写侧是 50MHz 全速 frame_writer，占空比翻倍以上。）
	parameter WRITE_V_FLIP           = 1,       // 写 SDRAM 时是否做行翻转（BMP 需要）
	parameter FRAME_WIDTH            = 640,     // 帧宽，行翻转计算要用
	parameter FRAME_HEIGHT           = 480,     // 帧高，行翻转计算要用

    //--------------------------------------------------------------------------
    //  ★★ 引擎间互锁开关（2026-10-17 v3）★★
    //  1 = 旧行为：写引擎要等"读方向不忙"、读引擎要等"写方向不忙"。
    //      这是当年【只有一个读客户端】时的做法（厂商原始契约）。
    //  0 = 关闭：互斥改由外部 sdr_port_sched 的时隙表保证
    //      （RD/TX 与 WR/LD 是不同槽，物理上不可能同拍读写）。
    //
    //  【为什么必须关】留着互锁时，读突发（128 字 = 32 个 RD 槽 = 512 拍）
    //  期间写被完全挡住，随后写 128 字又要 512 拍 ⇒ 128+128 字要 1024 拍。
    //  而每行只有 3975 拍、需要 640 读 + 640 写 ⇒ 要 5120 拍，永远追不上
    //  ⇒ 写引擎收不满一帧 ⇒ write_finish 不产生 ⇒ display_valid 永远 0
    //  ⇒ 屏幕纯黑（数码管最右位 b）。
    //--------------------------------------------------------------------------
	parameter INTERLOCK              = 1
)
(
	input                            rst,
	input                            mem_clk,          // SDRAM 控制器侧时钟（125MHz）
	input							 Sdr_init_done,    // SDRAM 初始化完成
	input							 Sdr_init_ref_vld, // SDRAM 自刷新请求（本层直接透传）
    input							 Sdr_busy,         // SDRAM 正忙

	// ---- SDRAM 用户读接口 ----
	output							 App_rd_en,        // 读使能（每读一个数据拉高一拍，已被 rd_ready 门控）
	output							 App_rd_want,      // ★ 未门控的"我想读"（给仲裁器判优先级）
	output[ADDR_BITS - 1:0]			 App_rd_addr,      // 读地址
	input							 Sdr_rd_en,		   // SDRAM 读数据有效
	input[MEM_DATA_BITS - 1:0]		 Sdr_rd_dout,      // SDRAM 读出的数据
    // ★ 读口仲裁放行（透传给 frame_fifo_read）★
    //   加了 SDRAM 纹理读之后，读口要和图片图层共用；仲裁器在服务纹理期间
    //   把 rd_ready 拉低，本模块的读突发就被"暂停"而不是"丢弃"（burst_cnt 和
    //   rd_delay 都不前进，突发长度一点都不丢）。
    //   没有仲裁器时接 1'b1 即可，行为与改造前完全一致。
    input                            rd_ready,
    // ★ 外部读口占用（纹理读回合）★
    //   接读仲裁器的 tex_busy。它必须算进 App_rd_busy，否则写引擎会认为
    //   "读方向不忙"，在纹理读命令期间发起写突发 —— 单口 SDRAM 同拍读写
    //   = 丢命令/错数据（厂商文档：不支持读写同时操作）。
    input                            rd_busy_ext,
    // ★ 写口放行（透传给 frame_fifo_write；装载 4 字组期间拉低 4 拍）★
    input                            wr_ready,
    // ★ 写方向忙（= App_wr_busy）引出给写仲裁器 ★
    //   装载开组前要确认显示写引擎"没在突发、也没正要突发"，
    //   否则会把显示突发的头几拍切掉。
    output                           wr_busy_o,

	// ---- 上层读接口（video_clk = 25MHz 域）----
	input                            read_clk,         // 读侧时钟
	input                            read_req,         // 请求读一帧（保持高直到应答）
	output                           read_req_ack,     // 请求应答
	output                           read_finish,      // 一帧读完
	input[ADDR_BITS - 1:0]           read_addr_0,      // 缓冲区0 基地址
	input[ADDR_BITS - 1:0]           read_addr_1,      // 缓冲区1 基地址
	input[ADDR_BITS - 1:0]           read_addr_2,      // 缓冲区2 基地址（本工程未用）
	input[ADDR_BITS - 1:0]           read_addr_3,      // 缓冲区3 基地址（本工程未用）
	input[1:0]                       read_addr_index,  // 选哪块缓冲区（0~3）
	input[ADDR_BITS - 1:0]           read_len,         // 一帧多少个数据
	input                            read_en,          // 取一个数据（拉高一拍）
	output[READ_DATA_BITS  - 1:0]    read_data,        // 送出的数据

	// ---- SDRAM 用户写接口 ----
	output							 App_wr_en,        // 写使能
	output[ADDR_BITS - 1:0]			 App_wr_addr,      // 写地址
	output[MEM_DATA_BITS - 1:0]	 App_wr_din,       // 写数据
	output[3:0]						 App_wr_dm,        // 写字节掩码

	// ---- 上层写接口（sd_card_clk = 100MHz 域）----
	input                            write_clk,        // 写侧时钟
	input                            write_req,        // 请求写一帧
	output                           write_req_ack,    // 请求应答
	output                           write_finish,     // 一帧写完
	input[ADDR_BITS - 1:0]           write_addr_0,     // 缓冲区0 基地址
	input[ADDR_BITS - 1:0]           write_addr_1,     // 缓冲区1 基地址
	input[ADDR_BITS - 1:0]           write_addr_2,     // 缓冲区2（未用）
	input[ADDR_BITS - 1:0]           write_addr_3,     // 缓冲区3（未用）
	input[1:0]                       write_addr_index, // 选哪块缓冲区
	input[ADDR_BITS - 1:0]           write_len,        // 一帧多少个数据
	input                            write_en,         // 写一个数据（拉高一拍）
	input[WRITE_DATA_BITS - 1:0]     write_data,       // 写入的数据

	//------------------------------------------------------------------
	//  ★ 诊断输出：读 FIFO 里当前可读的字数（视频端看得见的余量）★
	//------------------------------------------------------------------
	//  【为什么要引出来】上板有一条**动态闪烁**的竖直虚线伪影（固定 x、
	//  每 4 行一次、小块变亮）。排掉"幽灵数据脉冲"和"图片层读口共用"之后，
	//  最像的机制是 **读 FIFO 被读空**：
	//      · 读空那一拍，视频端拿到的是 FIFO 存储体里的旧内容（多半是画面里
	//        大片的白底）→ 看上去就是一小块"变亮"的斑点；
	//      · 余量是不是够，取决于"读突发相对视频行的相位"，而这个相位每帧都在
	//        漂（App 写帧 59.88Hz vs 显示 59.52Hz）→ 所以**同一列会闪烁**。
	//  把水位引到数码管第 2 位：读到 F = 余量充足（≥15 字）；读到小数字 = 余量很薄；
	//  读到 0 = 确实读空过。这样"是不是读空"一次烧录就能定。
	output wire [9:0]                rd_avail,

	//------------------------------------------------------------------
	//  ★ 2026-10-17 新增诊断：读 FIFO 自身的空/满标志（原本悬空）★
	//  【为什么】rfifo IP 的 rdusedw/wrusedw 是 9 位差值：512 深的 FIFO
	//  水位打到 512 时回绕成 0 —— 用 rdusedw 判"欠载"会把"满"误判成"空"
	//  （数码管第 2 位两轮读 0 就是这么来的）。empty_flag / full_flag 是
	//  IP 自己算的硬标志，没有回绕歧义：
	//      rd_fifo_empty（video_clk 域）= FIFO 真空（视频会拿到旧数据）
	//      rd_fifo_full （mem_clk  域）= FIFO 真满（再写就丢字 = 回绕地雷）
	//  顶层把这两个做成粘滞标志上数码管，当场分清虚线到底是哪种。
	//------------------------------------------------------------------
	output wire                      rd_fifo_empty,
	output wire                      rd_fifo_full,
	//------------------------------------------------------------------
	//  ★ 2026-10-18：写 FIFO 水位引出来给读仲裁器做"让路"判据 ★
	//  【为什么必须让路】写 FIFO 只有 512 深，像素以 25MHz 灌进来；
	//  纹理回合一旦连跑 4096 拍（33µs），这期间写引擎拿不到口，
	//  FIFO 会被灌进 825 个字 ⇒ **必然溢出丢字** ⇒ 写引擎永远写不满
	//  write_len ⇒ write_finish 永不产生 ⇒ display_valid 永不置位 ⇒ 黑屏。
	//  所以：水位过阈值就让读仲裁器提前收工，把口还给写。
	//------------------------------------------------------------------
	output wire [9:0]                wr_fifo_used,
	output wire                      wr_fifo_full,
	//------------------------------------------------------------------
	//  ★ 显示读 FIFO 的占用（mem_clk 侧视图，9 位，满 512 时回绕成 0）★
	//  读仲裁器用它判断"显示侧快断粮了"：见底就立刻结束纹理回合。
	//  【为什么用它而不是 rd_avail】rd_avail 在 video_clk（25MHz）域，
	//  跨到 125MHz 仲裁器里要多比特同步；wrusedw 本来就在 mem_clk 域。
	//  回绕只会造成"满的时候被误判成见底"⇒ 多让一次路，无害。
	//------------------------------------------------------------------
	output wire [8:0]                rd_buf_used,
	//------------------------------------------------------------------
	//  ★ 2026-10-18：写 FIFO 的【写侧】占用 → 给像素源做背压 ★
	//  【为什么用写侧 wrusedw 而不是读侧 rdusedw】wfifo 是双时钟 FIFO：
	//      · rdusedw（读侧，mem_clk 域）= 写引擎待消化的量（已有 wr_fifo_used）
	//      · wrusedw（写侧，write_clk 域）= FIFO 里真正的占用，**就在
	//        App / frame_writer 的 sd_card_clk 域**，不需要任何跨时钟域处理。
	//  【为什么要背压】像素源是"1 像素/拍 @50MHz"（50M 字/s），而写口在 16
	//  时隙表里只分到 4 个 WR 槽 = 16 字/64 拍 = 1/4 占空比 @125MHz ≈ 31.25M 字/s。
	//  512 深的写 FIFO 会在 ~27µs 内被灌满 ⇒ 溢出丢字 ⇒ 写引擎永远数不满
	//  write_len ⇒ write_finish 永不产生 ⇒ display_valid 恒 0 ⇒ 全屏黑。
	//  把水位引出去卡住像素源，生产自然降到写口的平均速率（自适应节流）。
	//------------------------------------------------------------------
	output wire [9:0]                wr_inflight,

	//------------------------------------------------------------------
	//  ★★ build FB16：帧缓存格式（888 / RGB565）的动态切换 ★★
	//------------------------------------------------------------------
	//  【为什么要动态】一帧的时间账（板上实测，SDRAM 侧 100MHz）：
	//      收像素 6.14ms + 等写 8.23ms + 等纹理 4.18ms + 帧内开销 2.3ms = 20.9ms
	//    而 60fps 只要 16.667ms ⇒ 差 25%。槽表/标签/忙守卫/提频全都试过，都是挪缺口。
	//    唯一还能省的就是"每帧搬多少个 32 位字"：565 打包让**写/读的字数减半**。
	//  【为什么是"动态"】静止画面 30fps 和 60fps 看起来完全一样，没必要为它牺牲色深。
	//    ⇒ 动画中 = 565（60fps），静止 = 888（全彩，30fps，但画面不动看不出差别）。
	//  【怎么做到无缝】格式是**每块缓冲自己的属性**，不是全局开关：
	//      写侧：帧起始（write_req）把 fmt_wr 记进 buf_fmt[write_addr_index]
	//      读侧：显示在场边界换块时（disp_rd_idx 同拍）才换 fmt_rd
	//    ⇒ 永远不会出现"这块是 565、却按 888 去读"的混读；也不需要重写旧帧，
	//      所以切换不闪黑、不撕裂、不重建。
	//------------------------------------------------------------------
	input  wire                      fmt_wr,     // 写侧：本帧要不要 565（来自 anim_active）
	input  wire                      fmt_rd,     // 读侧：正在显示那块缓冲的格式
	output wire [1:0]                buf_fmt_o,  // 两块缓冲各自的格式（bit=1 表示 565）
	// ★ 2026-10-23 写侧行走诊断：[3:0]=回跳次数/32（正确应到 F），[4]=地址异常粘滞
	output wire [4:0]                dbg_wr
);

// ---- FIFO 中的已用字数（用来判断攒够没有、够不够空）----
wire [9:0] wrusedw;        // 读 FIFO 的【写入侧】占用（rfifo.wrusedw，mem_clk 域）
wire       wfifo_full;     // ★写 FIFO 真满（出现过就说明丢字了）★
wire [9:0] rdusedw;        // 写 FIFO 的【读侧】可用字数 = 写引擎待消化的量（mem_clk 域）
wire [9:0] wfifo_wrusedw;  // 写 FIFO 的【写侧】占用（write_clk 域，给像素源背压）
// 读侧（mem_clk）能看到的字数 = 写引擎待消化的量（真满时它也会停在 511 附近）
assign wr_fifo_used = rdusedw;
assign wr_fifo_full = wfifo_full;
assign rd_buf_used  = wrusedw[8:0];   // 读 FIFO 写入侧占用（mem_clk 域）
assign wr_inflight  = wfifo_wrusedw;  // 写 FIFO 写侧占用（write_clk 域 → 像素源背压）

// ---- 两个搬运状态机的"忙"标志 ----
wire 								 App_rd_busy;        // 读方向正在搬
wire								 App_wr_busy;        // 写方向正在搬

// ---- FIFO 异步清零信号 ----
// 【为什么要清零】每次开始新的一帧搬运，都要把 FIFO 清空，
// 防止上一帧的残留数据混进来（尤其是上一次被中途打断的情况）。
wire                                 read_fifo_aclr;
wire                                 write_fifo_aclr;

wire O_wr_busy;     // 写状态机输出的忙标志
wire O_rd_busy;     // 读状态机输出的忙标志

// ---- 忙碌信号的传递（这是"写优先于读"的实现）----
// ★★ 2026-10-18 v2：App_rd_busy 现在还要算上【纹理读回合】★★
//  【为什么改了】v1 里读口靠 wr_act 逐拍让写口，结果是显示读突发被切碎；
//  v2 改成：读口不再看写口脸色，写口改回厂商原始契约 ——
//      frame_fifo_write 决定是否发突发读/写时检查对方的 busy。
//  可这样一来，"纹理读"必须也出现在 App_rd_busy 里，否则写引擎以为读口空闲、
//  在纹理读命令中间插入写突发 ⇒ 同拍读写 ⇒ 丢命令 ⇒ 错数据。
//  互锁可以关：时隙表已经保证读写不同拍，引擎之间不需要再互相等
assign App_wr_busy = INTERLOCK ? O_wr_busy                  : 1'b0;
assign App_rd_busy = INTERLOCK ? (O_rd_busy | rd_busy_ext)  : 1'b0;
assign wr_busy_o   = O_wr_busy;                 // 观察点保持真实值

// 写字节掩码固定为 0：0 表示"这一字节允许写入"（和通常的直觉相反，注意）
// 【为什么全 0】因为我们要写完整的 32bit 数据，不需要屏蔽任何字节。
assign App_wr_dm = 4'b0000;

//==============================================================================
//  ★★ build FB16：写侧 RGB565 打包（把 1 像素/拍变成 2 像素/字）★★
//==============================================================================
//  【输入】write_en / write_data 仍是"每拍一个像素、{R,G,B,8'h00}"
//  【输出】wfifo_we / wfifo_di 变成"每两拍一个字、{后像素565, 前像素565}"
//  【为什么在这里做，而不是在 frame_writer 里】frame_writer 的 NPX / cnt /
//    write_len 全是"像素"语义（一帧 307200 个），在那里改会连锁改一大片；
//    在 FIFO 边界打包，上游一行都不用动。
//  【相位对齐】打包相位 pk_ph 在 write_req 期间清零 ⇒ 每帧第一个像素一定是
//    "前像素"，不会出现半帧错位（一帧 307200 是偶数，必然配成 153600 个字）。
//==============================================================================
reg        wr_fmt_r;     // 本帧的格式（帧起始锁存，整帧恒定）
reg        pk_ph;        // 0 = 等前像素，1 = 等后像素
reg  [15:0] pk_lo;       // 前像素的 RGB565
reg  [1:0]  buf_fmt;     // 两块缓冲各自的格式（bit=1 ⇒ 565）

wire [15:0] w_pix565 = {write_data[31:27], write_data[23:18], write_data[15:11]};

wire        pk_we    = wr_fmt_r & write_en & pk_ph;        // 只在"后像素"到达时写
wire [31:0] pk_di    = {w_pix565, pk_lo};                  // {后, 前}
wire        wfifo_we = wr_fmt_r ? pk_we : write_en;
wire [31:0] wfifo_di = wr_fmt_r ? pk_di : write_data;

always @(posedge write_clk or posedge rst) begin
    if (rst) begin
        wr_fmt_r <= 1'b0;
        pk_ph    <= 1'b0;
        pk_lo    <= 16'd0;
        buf_fmt  <= 2'b00;
    end else if (write_req) begin
        // ---- 帧起始：锁格式、对齐打包相位、把格式记到"这一块"名下 ----
        wr_fmt_r <= fmt_wr;
        pk_ph    <= 1'b0;
        pk_lo    <= 16'd0;
        buf_fmt[write_addr_index] <= fmt_wr;
    end else if (write_en) begin
        pk_ph <= ~pk_ph;
        if (!pk_ph) pk_lo <= w_pix565;                      // 前像素：先存着
    end
end

assign buf_fmt_o = buf_fmt;

//==============================================================================
//  ★★ 2026-10-23 build FB16b：帧起始当拍的格式必须是"新"格式 ★★
//==============================================================================
//  【上板现象】全屏黑，数码管状态码卡在 4，display_valid 永远是 0。
//  【根因】wr_fmt_r 是**寄存器**：write_req 那一拍它还是上一帧的值，下一拍才更新。
//    而 frame_fifo_write 在 write_req 当拍就要用 fmt565 去算本帧的总字数
//    （len_words = fmt565 ? write_len>>1 : write_len）。
//      ⇒ 凡"这一帧的格式和上一帧不同"，锁进 write_len_latch 的就是**上一帧的格式**。
//      ⇒ 切到 565 的第一帧：实际只进 153600 字，写引擎却在等 307200 字
//        ⇒ write_finish 永不产生 ⇒ frame_writer 的 busy 永远为 1
//        ⇒ viewer_ctrl 停在 S_HDF（数码管 4）、display_valid 恒 0 ⇒ 全屏黑。
//  【为什么以前没炸】demo 上电 fmt565=0（888），和 wr_fmt_r 的复位值一致，
//    第一帧天然对齐；只有"上电第一帧就是 565"（查看器的过渡/低清方案）才会踩到。
//  【修法】只给 frame_fifo_write 的 fmt565 换成"当拍值"：
//    write_req 高 ⇒ 用 fmt_wr（新格式）；其余时候 ⇒ 用 wr_fmt_r（已更新、整帧恒定）。
//    打包通路 wfifo_we/wfifo_di 仍然用 wr_fmt_r —— 它在 write_req 之后一拍就正确了，
//    而且必须整帧恒定，不能跟着 fmt_wr 抖。
//==============================================================================
wire wr_fmt_now = write_req ? fmt_wr : wr_fmt_r;

//==============================================================================
//  例化 1：写方向的异步 FIFO
//==============================================================================
// 数据从 write_clk（100MHz）写进来，从 mem_clk（125MHz）读出去。
wfifo_32_32_512 write_buf
	(
	.clkr                      	(mem_clk                  ),  // 读侧时钟（SDRAM 侧）
	.clkw                      	(write_clk                ),  // 写侧时钟（SD 卡侧）
	.rst                       	(write_fifo_aclr          ),  // 异步清零
	.we                      	(wfifo_we                 ),  // ★ build FB16：可能是每 2 拍一次
	.re                      	(App_wr_en        		  ),  // 读请求（状态机去取数据时）
	.di                       	(wfifo_di                 ),  // ★ build FB16：可能是打包后的字
	.empty_flag                 (                         ),  // 空标志，未用
	.full_flag                  (wfifo_full              ),  // ★引出：真满（=丢了字）★
	.wrusedw                	(wfifo_wrusedw          	  ),  // ★写侧占用 → 引出做像素源背压★
	.rdusedw                	(rdusedw                  ),  // 读侧可用数据数（状态机要看）
	.dout                      	(App_wr_din		          )   // 读出的数据 → 直接送 SDRAM
);

//==============================================================================
//  例化 2：写方向的搬运状态机
//==============================================================================
// 工作流程：
//   ① 等 write_req（上层说"要写一帧"）
//   ② 应答（write_req_ack = 1），同时清空 FIFO、锁存写地址和数据长度
//   ③ 反复：等 FIFO 里攒够 256 个数据 → 向 SDRAM 发一次 256 长的突发写 → 循环
//   ④ 搬够 write_len 个数据 → 输出 write_finish 脉冲
frame_fifo_write
#
(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BURST_BITS                 (BURST_BITS               ),
	.BURST_SIZE                 (BURST_SIZE               ),
	.WRITE_V_FLIP               (WRITE_V_FLIP             ),
	.FRAME_WIDTH                (FRAME_WIDTH              ),
	.FRAME_HEIGHT               (FRAME_HEIGHT             )
)
frame_fifo_write_m0
(
	.rst                        (rst                      ),
	.mem_clk                    (mem_clk                  ),
    .Sdr_init_done				(Sdr_init_done),
	.Sdr_init_ref_vld			(Sdr_init_ref_vld),
    .Sdr_busy					(Sdr_busy),
	.App_rd_busy				(App_rd_busy),      // 输入：读方向是否在忙
    .O_wr_busy					(O_wr_busy),        // 输出：本方向是否忙
    .wr_ready                   (wr_ready),         // ★ 写口分组门控

	.App_wr_en					(App_wr_en),
    .App_wr_addr				(App_wr_addr),
	.write_req                  (write_req                ),
	.write_req_ack              (write_req_ack            ),
	.write_finish               (write_finish             ),
	.write_addr_0               (write_addr_0             ),  // 4 个候选基地址
	.write_addr_1               (write_addr_1             ),
	.write_addr_2               (write_addr_2             ),
	.write_addr_3               (write_addr_3             ),
	.write_addr_index           (write_addr_index         ),  // 选哪个（双缓冲的关键）
	// ★ build FB16a：长度不再跨时钟域携多比特（切换瞬间会采到半旧半新的错长度
	//   ⇒ write_len_latch 变成垃圾 ⇒ 若大于实际供给则 write_finish 永不产生 ⇒ 卡死）。
	//   这里始终送像素数，由 frame_fifo_write 在自己的时钟域里按 fmt565 折半。
	.write_len                  (write_len                ),
	.fifo_aclr                  (write_fifo_aclr          ),  // 输出：通知 FIFO 清零
	.rdusedw                 	(rdusedw                  ),  // 输入：看 FIFO 攒了多少
	.fmt565                     (wr_fmt_now                ),  // ★ FB16b：帧起始用当拍格式算字数
	.dbg_wr                     (dbg_wr                    )   // ★ 写侧行走诊断
);

//==============================================================================
//  例化 3：读方向的异步 FIFO
//==============================================================================
// 数据从 mem_clk（125MHz）写进来，从 read_clk（25MHz）读出去。
// 【注意和写 FIFO 的对称性】两边的 clkr/clkw 是反过来的：
//   写 FIFO：clkw = 慢时钟，clkr = 快时钟
//   读 FIFO：clkw = 快时钟，clkr = 慢时钟
// ★ 100M-23：FIFO 的 dout 先落到 fifo_dout，再做欠载保持（见例化之后的说明）
wire [READ_DATA_BITS-1:0] fifo_dout;

//==============================================================================
//  ★★ build FB16：读侧 RGB565 解包（一个字 → 两个像素）★★
//==============================================================================
//  【FIFO 的时序（关键）】rfifo 是"非 show-ahead"模式：
//      第 t 拍给 re ⇒ 第 t+1 拍 dout 才是那个字，并保持到下次 re。
//  而视频端在 read_en 之后**再一拍**才采样 read_data（video_delay 用 de_d[19]）。
//  所以：
//      t   : read_en=1, rd_ph=0 ⇒ 弹字（rd_pop=1），dout 在 t+1 变成这个字
//      t+1 : rd_ph_d=0（上一拍的相位）⇒ 输出该字**低半** = 前像素 ✓
//      t+2 : rd_ph_d=1            ⇒ 输出该字**高半** = 后像素 ✓
//  ⇒ 用"上一拍的相位"来选半字，这正是 rd_ph_d 存在的唯一原因。
//==============================================================================
reg        rd_ph;      // 当前像素在字内是前半还是后半
reg        rd_ph_d;    // 打一拍（对齐 dout 的 1 拍延迟）
reg [23:0] rgb_hold;   // 欠载/无效时保持的上一像素
reg        word_vld;   // ★ fifo_dout 当前是否装着一个"真的弹出过"的字

wire        rd_pop   = fmt_rd ? (read_en & ~rd_ph & ~rd_fifo_empty) : read_en;
wire [15:0] r_pix565 = rd_ph_d ? fifo_dout[31:16] : fifo_dout[15:0];
wire [23:0] r_rgb565 = {r_pix565[15:11], r_pix565[15:13],   // R5 -> R8（低位补高位）
                        r_pix565[10:5],  r_pix565[10:9],    // G6 -> G8
                        r_pix565[4:0],   r_pix565[4:2]};    // B5 -> B8

// ★★ 2026-10-22 build FB16a：白色脏像素的根因就在这里 ★★
//  【现象】565 模式下图片层出现少量白色脏点。
//  【根因】rfifo 的 dout 在"还没弹出过任何字"时是**内存里的值**（复位后全 1）。
//    888 路径有 `rd_fifo_empty ? dout_hold : fifo_dout` 挡着，所以看不出来；
//    而 565 路径的"字内后半"那一拍（rd_ph_d=1）用的是**已经弹出来的字**，
//    本来不需要看 empty —— 但"弹出来"这件事必须真的发生过。
//    帧起始 FIFO 被 aclr 之后、第一个字到达之前，rd_ph_d 可能已经是 1，
//    这时 dout 还是全 1 ⇒ 0xFFFF ⇒ **纯白**。
//  【修法】加一个"这个字确实弹出来过"的资格位 word_vld：没有资格就保持上一像素。
//    这样无论空 FIFO、清 FIFO、还是上电未初始化，都不可能再把 0xFFFF 当像素。
wire [23:0] r_rgb_sel = word_vld ? r_rgb565 : rgb_hold;

always @(posedge read_clk or posedge rst) begin
    if (rst) begin
        rd_ph <= 1'b0; rd_ph_d <= 1'b0; rgb_hold <= 24'd0; word_vld <= 1'b0;
    end else if (read_req) begin
        rd_ph <= 1'b0; rd_ph_d <= 1'b0; word_vld <= 1'b0;   // 帧起始：FIFO 即将被清
    end else begin
        rd_ph_d <= rd_ph;                        // 恒定延迟一拍
        if (rd_pop) word_vld <= 1'b1;            // 真的弹出一个字 ⇒ 之后 dout 有效
        if (read_en) begin
            rd_ph    <= ~rd_ph;
            rgb_hold <= r_rgb_sel;               // 只记"实际显示过的"像素
        end
    end
end

rfifo_32_32_512 read_buf
	(
	.clkr                     	(read_clk                   ),  // 读侧时钟（视频侧 25MHz）
	.clkw                     	(mem_clk                    ),  // 写侧时钟（SDRAM 侧 125MHz）
	.rst                      	(read_fifo_aclr             ),  // 异步清零
	.we                     	(Sdr_rd_en       			),  // 写请求（SDRAM 数据有效）
	.re                     	(rd_pop                     ),  // ★ build FB16：565 时两拍弹一个字
	.di                      	(Sdr_rd_dout                ),  // 写数据（SDRAM 读出的）
	.empty_flag					(rd_fifo_empty              ),  // ★ 2026-10-17 引出：真空标志（原来悬空）
	.full_flag					(rd_fifo_full               ),  // ★ 2026-10-17 引出：真满标志（原来悬空）
	.wrusedw                	(wrusedw         	  	  	 ),  // 写侧已用（状态机要看；★9 位，512 时回绕成 0★）
	.rdusedw                	(rd_avail        			 ),  // ★ 引出（9 位，512 时回绕成 0；诊断已改用空/满标志）
	.dout						(fifo_dout                )   // 送出的像素数据（下面再做欠载保持）
);

//==============================================================================
//  ★★ 100M-23：显示读 FIFO 欠载保护（板上"纯白竖线"的另一条来源）★★
//  【现象】白线只在两个图片层重叠时出现、且落在图片框之外，贯穿全高。
//    两图层重叠 ⇒ 纹理读需求翻倍 ⇒ 显示读突发被推迟（刷新/换向/写口优先都会
//    断流）⇒ 读 FIFO（512 深）在某一列被抽干 ⇒ 视频端 read_en 是**无条件**的
//    （read_en = de_d[18]，只看时序不看 FIFO）⇒ 弹到空 FIFO。
//    空 FIFO 的 dout 是内存里的值（复位后是全 1）⇒ video_read_data[31:8] = 纯白
//    ⇒ 画面上就是一条白线；欠载点每行固定在同一相位 ⇒ 贯穿全高；
//    它在显示通路上，所以把那一列原本的文字/图形全盖成白 —— 落在图片框之外。
//  【修法】标准的"欠载保持上一像素"：FIFO 空时不弹、不换数据，只把上一个有效
//    像素保持住。视频时序不受影响（de 照常出像素），最多那一两个像素被拉长，
//    绝不出现垃圾色。
//  【为什么不直接给 read_en 加 ~empty】那样会少弹一个 ⇒ 整帧像素流错位，
//    后面所有内容横移，比一条白线更糟。
//==============================================================================
reg  [READ_DATA_BITS-1:0] dout_hold;
always @(posedge read_clk or posedge rst) begin
    if (rst)                            dout_hold <= {READ_DATA_BITS{1'b0}};
    else if (read_en && !rd_fifo_empty) dout_hold <= fifo_dout;   // 只在真取到数据时更新
end
//==============================================================================
//  ★★ 2026-10-26：888 路径的"弹字资格位"（根治右下角单像素伪影）★★
//==============================================================================
//  【根因】不是场边界清 FIFO（旧注释误判，时序上不可能——见下面更正段），
//    而是**帧尾 FIFO 自然排空**：视频侧每帧正好弹 307,200 个字，最后一个
//    read_en 把 FIFO 排空；rfifo 是非 show-ahead（见上面 415 行），排空后
//    **下一拍 dout 仍保留最后一个有效字**，但 empty_flag 同时变 1。
//    旧代码先看 empty ⇒ 把"刚好排空、dout 还有效"误判成欠载 ⇒ 每帧最后一个
//    像素（屏幕右下角）拿到左邻居/标记色。565 路径早有 word_vld 资格位防
//    这个（见 r_rgb_sel），888 路径一直没有等价物——不对称的根源就在这。
//  【修法】补一个与 565 同构的资格位：本帧成功弹过字 ⇒ dout 一定是
//    "最近一次真弹出的字"（FIFO 空时 dout 保持，见 415 行注释）。
//    · 帧尾最后一个像素：word_vld8=1 ⇒ 显示 fifo_dout = **真正的最后一个字**
//      （比旧行为"重复左邻居"更正确，连 1px 拉伸都没有）
//    · 真欠载（本帧还没成功弹过任何字，如帧起始断粮/上电未初始化）：
//      word_vld8=0 ⇒ 保持上一像素（dout_hold），绝不出垃圾色
//  【为什么 read_req 清资格位是安全的】read_req 在 vs 下降沿后 2 拍产生，
//    而本帧最后一个 read_en 在那之前约 13 行（color_bar: V_FP+V_SYNC=12 行）
//    就结束了，两者永不相邻；且那期间视频侧在消隐区、没有 read_en。
//  【为什么不会漏判 mid-frame 欠载】资格位是"本帧弹过字"的粘滞标志；
//    mid-frame 断粮时 FIFO 空、dout 保持最后一个好字 ⇒ 显示它就是
//    "保持上一像素"，与 dout_hold 行为一致，保护不丢。
//==============================================================================
reg word_vld8;      // 888 路径：本帧真的从 FIFO 弹出过字（dout 有效）
always @(posedge read_clk or posedge rst) begin
    if (rst)                              word_vld8 <= 1'b0;
    else if (read_req)                    word_vld8 <= 1'b0;   // 帧起始：FIFO 即将被 aclr
    else if (rd_pop && ~rd_fifo_empty)    word_vld8 <= 1'b1;   // 真弹出了一个字
end
// ★★ 100M-24：欠载"染色"——一次烧板就能判定 ★★
//   上一版只是保持上一像素；但保持出来的仍然是"背景白"，在文字上看起来还是
//   一颗白齿 ⇒ 从画面上分不清"欠载"和"内容缺失"。改成染洋红：
//     · 梳齿变成**洋红** ⇒ 确认是显示读欠载（读引擎/FIFO 供给问题）
//     · 梳齿**仍是白色** ⇒ 与显示读无关，白是帧缓存里真写进去的（查场景/写流）
//   代价：欠载点显示洋红而不是"拉长"——**仅用于这一版定位**；
//   定位完把 UNDERRUN_MARK 改成 0，即恢复"保持上一像素"。
//==============================================================================
//  ★★ 2026-10-26：更正 2026-10-22 的错误归因 ★★
//  【旧诊断错在哪】旧注释说"帧尾那 1 个像素是 read_fifo_aclr 压在最后一个
//    read_en 上"——时序上不可能：最后活跃像素在 v_cnt=523/h_cnt=798
//    （color_bar.v:85-96），vs 脉冲在下一轮计数的 v_cnt=10~11，vs_fall
//    约在最后活跃像素之后 ~10,400 个视频周期；read_req 还要再经 3 级
//    mem_clk 同步 + S_ACK 才拉 aclr ⇒ 距最后一个 read_en（= 最后活跃
//    de+18 拍，video_delay.v:60）还有约一万拍。**aclr 碰不到最后一个
//    像素，延迟/修改 aclr 对这个 bug 是无效功。**
//  【真机制】帧尾 FIFO 自然排空 + 888 路径缺"弹字资格位"（见上面
//    word_vld8 段）：最后一个 read_en 排空 FIFO，排空的下一拍 dout 仍是
//    最后一个有效字，但 empty_flag 已变 1 ⇒ 旧代码误判欠载。
//  【颜色更正】旧注释按 32'hFFFF_00FF 推算"黄"；实际 UNDERRUN_RGB 是
//    **24 位**常量，赋给 32 位 read_data 时被零扩展成 32'h00FF_FF00，
//    顶层取 [31:8]（top_tf_hdmi_audio.v:1238）⇒ 0x00FFFF = R0 G255 B255
//    = **浅蓝**。所以 2026-10-22 板上看到的"右下角一个像素"一直是浅蓝，
//    不是黄；UNDERRUN_MARK=1 时每帧必现，UNDERRUN_MARK=0 时退化成
//    "重复左邻居"（仍有 1px 拉伸，只是肉眼难辨）。

//==============================================================================
//  ★ 2026-10-23 诊断开关：把"显示读欠载"染成黄色（用完改回 0）★
//==============================================================================
//  【为什么要临时打开】上板出现"斜向接缝"（越往下偏得越多）。这个形状只有一种
//    成因：显示读的像素流**每行少一个字**，误差逐行累积。而少字只有两条路：
//      ① FIFO 欠载（读完的时候字还没到）⇒ 该字永久缺失 ⇒ 从这点起整体前移；
//      ② 存进去的内容本身就是斜的（合成/写侧问题）。
//    两种情况的修法完全不同，所以必须先分开。打开本开关后：
//      · 斜缝变成**黄色**（或缝上出现黄色像素）⇒ 是①欠载 ⇒ 去加显示读带宽；
//      · 斜缝颜色不变、没有一个黄色像素        ⇒ 是②内容 ⇒ 去查合成/写入。
//  【代价】欠载点显示黄色而不是"保持上一像素"。**仅用于这一版定位**。
//    （注意：顶层取 [31:8]，所以 32'hFFFF_00FF 显示为 FF,FF,00 = 黄）
//==============================================================================
localparam integer UNDERRUN_MARK = 0;      // 1 = 欠载染色（诊断）／0 = 保持上一像素
                                             // ★ 2026-10-26：配 word_vld8 后此开关
                                             //   只会标记**真欠载**（结构性误报已除），
                                             //   生产保持 0。

localparam [23:0] UNDERRUN_RGB = 24'hFF_FF_00;   // 注意：24 位常量零扩展后经顶层
                                                 // [31:8] 取数显示为 0x00FFFF 浅蓝

assign read_data = fmt_rd
    // 565：r_rgb_sel 已经处理了"没弹过字 / 欠载 / 上电未初始化"三种情况
    //      （这时输出上一像素，绝不出白色），这里不再需要额外的 empty 判断。
    ? {r_rgb_sel, 8'h00}
    // 888：★ 2026-10-26 根治 ★ 不能再像旧版那样先看 rd_fifo_empty——
    //   帧尾最后一个 read_en 会把 FIFO 自然排空，排空的下一拍 dout 仍是
    //   最后一个有效字（非 show-ahead），但 empty_flag 已变 1 ⇒ 旧版把
    //   它误判成欠载（右下角单像素伪影的根因）。
    //   改成"本帧成功弹过字"资格位 word_vld8：
    //     =1 ⇒ dout 一定是最近一次真弹出的字（含帧尾最后一个字）⇒ 用它；
    //     =0（帧起始断粮/上电未初始化，本帧还没弹出过任何字）⇒ 保持/标记。
    : (word_vld8 ? fifo_dout
                 : ((UNDERRUN_MARK != 0) ? UNDERRUN_RGB : dout_hold));

//==============================================================================
//  例化 4：读方向的搬运状态机
//==============================================================================
// 工作流程：
//   ① 等 read_req（视频说"准备读一帧"）
//   ② 应答，清空 FIFO，锁存读地址和长度
//   ③ 反复：检查读 FIFO 有没有足够空间 + 写方向是否在忙
//            → 不忙且有空位 → 向 SDRAM 发一次 256 长的突发读
//   ④ 读够 read_len 个 → 输出 read_finish
frame_fifo_read
#
(
	.MEM_DATA_BITS              (MEM_DATA_BITS            ),
	.ADDR_BITS                  (ADDR_BITS                ),
	.BURST_BITS                 (BURST_BITS               ),
	.BURST_SIZE                 (BURST_SIZE               )
)
frame_fifo_read_m0
(
	.rst                        (rst                      ),
	.mem_clk                    (mem_clk                  ),
    .Sdr_init_done				(Sdr_init_done),
	.Sdr_init_ref_vld			(Sdr_init_ref_vld),
    .Sdr_busy					(Sdr_busy),
    .Sdr_rd_en					(Sdr_rd_en),        // 输入：SDRAM 数据有效
    .rd_ready                   (rd_ready),         // 输入：读口仲裁放行
	.App_wr_busy				(App_wr_busy),      // 输入：写方向是否在忙（写优先！）
    .O_rd_busy					(O_rd_busy),        // 输出：本方向是否忙

    .App_rd_en					(App_rd_en),
    .App_rd_want                (App_rd_want),
    .App_rd_addr				(App_rd_addr),
	.read_req                   (read_req                 ),
	.read_req_ack               (read_req_ack             ),
	.read_finish                (read_finish              ),
	.read_addr_0                (read_addr_0              ),
	.read_addr_1                (read_addr_1              ),
	.read_addr_2                (read_addr_2              ),
	.read_addr_3                (read_addr_3              ),
	.read_addr_index            (read_addr_index          ),  // 双缓冲的关键
	.fmt565                     (fmt_rd                   ),  // ★ build FB16a：本帧是不是 565
	// ★ build FB16a：同上，长度送常量，折半在读引擎自己的时钟域里做
	.read_len                   (read_len                 ),
	.fifo_aclr                  (read_fifo_aclr           ),
	.wrusedw                	(wrusedw                  ),
	.wr_full                    (rd_fifo_full             )   // ★ 100M-23：真满闸门（wrusedw 会回绕）
);

endmodule
