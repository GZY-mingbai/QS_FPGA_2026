`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 文件名   : sdram.v
// 模块名   : sdram
// 公司     : anlogic（安路科技）
// 作者     : xg
// 功能     : 【SDRAM 内存控制器顶层封装】
//            把安路官方提供的 SDRAM 控制器 IP（sdr_as_ram）和 SDRAM 器件模型
//            例化在一起，对上提供简单的"读写接口"。
//
// 为什么要有一个 SDRAM ?
//   本工程的用途是把 SD 卡里的大图（640x480x24bit ≈ 900KB）搬到 HDMI 显示。
//   FPGA 片内 RAM 只有几 MB 甚至更少，装不下整帧图片；
//   外部 SDRAM 有 2M x 32bit = 64Mbit = 8MB，专门用来当"显存"。
//
//   数据流向：
//      SD 卡 ──► SDRAM（写入一帧）──► 读出 ──► HDMI 显示
//                ↑ 用双缓冲：一块在显示、一块在写入
//
// 两层结构（本模块做的事）：
//   ┌───────────────────────────────────────────────┐
//   │  用户接口（好用的）                            │
//   │    App_wr_en / App_wr_addr / App_wr_din       │  ← 上层直接调这个
//   │    App_rd_en / App_rd_addr / Sdr_rd_dout      │
//   ├───────────────────────────────────────────────┤
//   │  sdr_as_ram（官方 IP，处理复杂时序）           │
//   │    刷新/预充电/行激活/读写命令/延迟……          │
//   ├───────────────────────────────────────────────┤
//   │  EG_PHY_SDRAM_2M_32（器件物理模型）            │
//   │    真正的引脚：RAS/CAS/WE/BA/ADDR/DQ/DM        │
//   └───────────────────────────────────────────────┘
//
// 如何使用 ?
//   由 frame_read_write.v 例化，读写地址由上层算好（一帧 = 640*480 = 307200 个地址）。
//   注意：这里必须给两个时钟 —— Clk（100MHz 主时钟）和 Clk_sft（相移时钟），
//   SDRAM 控制器需要用到时钟相移来保证建立/保持时间。
//
// 为什么这样写 ?（初学者重点）
//   1) SDRAM 的时序极其复杂（上电要 200us 等待、预充电、8 次刷新、模式寄存器配置…），
//      手写几乎不可能，所以直接用厂商 IP。这份文件就是"包装层"。
//   2) `ifdef SIMULATION 的用意：仿真时引脚要暴露出来方便观察，
//      综合成实际电路时引脚由器件模型内部接管，所以用编译宏区分两种写法。
//   3) App_ref_req 原来是固定接 1'b0（全交给 IP 内部刷新）。2026-10-23 改成
//      每 1562 拍（15.6us）主动请求一次：板测发现"只有永不重写的区域（THUMB）
//      随时间掉数据"，怀疑内部刷新不到位 ⇒ 用 IP 公开接口补齐刷新率。
//      详见下面第 97 行附近的说明。
//////////////////////////////////////////////////////////////////////////////////

`define DEBUG

// 引入全局参数定义（ADDR_WIDTH / DATA_WIDTH / BA_WIDTH / ROW_WIDTH / DM_WIDTH 等都在里面）
`include "../user_source/hdl_source/include/global_def.v"

module sdram(
    // 仿真模式下才需要暴露这些 SDRAM 物理引脚
    `ifdef SIMULATION
	output					SDRAM_CLK,     // SDRAM 时钟
	output					SDR_CKE,       // 时钟使能
	output					SDR_RAS,       // 行地址选通（低有效）
	output					SDR_CAS,       // 列地址选通（低有效）
	output					SDR_WE,        // 写使能（低有效）
	output	[`BA_WIDTH-1:0]	SDR_BA,        // Bank 地址
	output	[`ROW_WIDTH-1:0]SDR_ADDR,      // 行/列地址总线
	output	[`DM_WIDTH-1:0]	SDR_DM,        // 数据掩码（写时按字节屏蔽）
	inout	[`DATA_WIDTH-1:0]	SDR_DQ,	   // 双向数据总线
	`endif
    input   					Clk,           // 系统主时钟（100MHz）
    input						Clk_sft,       // 相移时钟（SDRAM 控制器内部要用）
    input						Rst,           // 复位，高有效
    
    output 						Sdr_init_done,   // SDRAM 初始化完成标志
    output						Sdr_init_ref_vld,// 初始化刷新完成标志
    output						Sdr_busy,        // SDRAM 忙标志
    // ★ 2026-10-24：把"即将发出 REF"这一拍引给调度器。
    //   纹理组是"4 条命令一组、控制器按组首地址+相位重算"，
    //   若 REF 与本组首拍撞在同一拍，IP 会先做刷新 ⇒ 该组被切开/丢一条
    //   ⇒ 控制器把后几条当新组首、按 head+ph 重排 ⇒ 读回 base+1..base+4
    //   ⇒ 少一个字、多一个字 ⇒ 顺序错位一个字 = 整幅左移 4px（320×240 上的 2px）。
    output						App_ref_req,     // = ref_pend 且已等到空闲窗口（即将发 REF，1 拍）
    
    // ---- 写接口 ----
    input						App_wr_en,       // 写使能
	input [`ADDR_WIDTH-1:0]		App_wr_addr,    // 写地址（单位：一个 32bit 字）
	input [`DM_WIDTH-1:0]		App_wr_dm,      // 写数据掩码，本工程固定为 0（不屏蔽）
	input [`DATA_WIDTH-1:0]		App_wr_din,	//synthesis keep - 写数据
    // ---- 读接口 ----
    input 						App_rd_en,       // 读使能
	input [`ADDR_WIDTH-1:0]		App_rd_addr,     // 读地址
	output 						Sdr_rd_en,       // 读数据有效（数据总线上的值此时才有效）
	output [`DATA_WIDTH-1:0]	Sdr_rd_dout	//synthesis keep - 读出的数据
);

// 非仿真模式下，这些信号在本模块内部自己产生，不引到端口
`ifndef SIMULATION
wire  			SDRAM_CLK;
wire 			SDR_RAS;
wire  			SDR_CAS;
wire  			SDR_WE;
wire  [`BA_WIDTH-1:0]	SDR_BA; 
wire  [`ROW_WIDTH-1:0]	SDR_ADDR;
wire  [`DATA_WIDTH-1:0]	SDR_DQ ;
wire  [`DM_WIDTH-1:0]	SDR_DM; 
`endif

//-----------------------------------------------------------------------------
//  ★ 2026-10-23：显式周期刷新（App_ref_req）
//-----------------------------------------------------------------------------
//  【为什么要加】
//    上板现象：THUMB 上的杂点"随时间增长、位置固定"，而帧缓存（每帧被整片读
//    + 每帧重写）从不显脏，HD（每次切换都被重写）也看不到。这是 DRAM 漏电
//    累积的典型特征 —— 只有"永不重写"的区域会暴露出来。
//    物理上：读不会修复数据（读回的是当前值，哪怕已经错了），只有重写才行。
//    帧缓存每帧被整片读一遍，等于每 16.7ms 把所有行"激活+预充"一次，天然免疫。
//    THUMB 谁也不碰，只能靠自动刷新撑着 ⇒ 刷新不到位就慢慢掉。
//
//  【为什么原来那条路不够】
//    ① 刷新到底有没有在执行，我们其实**无法确认**：sdr_port_sched.v 里的
//       ref_max 实测记录说"Sdr_init_ref_vld 几乎从不拉高"，而 top 的另一处记录
//       又写着"dbg_sel=3 每帧刷新脉冲期望 380~400"——两条记录互相矛盾，
//       说明这个指示信号不可信（它只反映 ref_vld，不反映 REF 命令是否发出）；
//    ② sdr_init_ref 的周期常量按 2048 行算（3125 拍 = 31.25us），
//       而同一个 IP 的端口注释要求 "4096 refresh in 64ms"（15.625us）—— 差 2 倍。
//    既然无法观测、又有一处 2 倍的疑点，就先用公开接口把刷新率钉到规格值。
//
//  【做法】用它公开的接口 App_ref_req 补一个周期请求：
//      sdr_init_ref.enc.v:166   Sdr_ref_req = ref_req | App_ref_req
//    多打一拍就多发一次 REF，不会影响原本的内部刷新路径。
//    REF_DIV = 1562 拍 @100MHz = 15.6us ⇒ 与 IP 注释的 15.625us 对齐
//    （若内部刷新本来就没在执行，这一路正好补齐规格；若本来在执行，
//      相当于 3 倍余量，只有好处）。
//
//  【代价】一次刷新控制器占用约 20 拍 ⇒ 每 1562 拍多 20 拍 ≈ 1.3% 带宽。
//          刷新期间 sdr_port_sched 会因 BUSY_HOLD 冻结时隙表，不会往刷新
//          窗口里发命令，所以不会带来"写被吞"。
//
//  【怎么调】REF_DIV：1562 = 1 倍（规格）、780 ≈ 2 倍、390 ≈ 4 倍。
//            改小 = 刷得更勤。寄存器 12 位，上限 4095。
//
//  ★ 2026-10-23 追加：刷新请求等"控制器空闲"再发 ★
//  【为什么】第一版是"到点就打"，结果上板出现**斜向接缝**（越往下偏得越多）——
//    这是显示读的像素流被"每行少一个字"累积出来的典型形状。
//    机制：REF 命令一旦插入，控制器会把正在飞的读突发切断；而显示读的地址是
//    逐字推进的（`rd_run` 期间不看 Sdr_busy），被切断的那条命令若控制器不接，
//    这个字就永久缺失 ⇒ 从该点起后面所有像素整体前移一位 ⇒ 一行一条缝、
//    多行累积成斜切。
//  【做法】到点后先挂起（ref_pend），等 `Sdr_busy` 连续 4 拍为 0 再打一拍请求。
//    这样 REF 落在突发与突发之间的空档里，不会切断任何在飞的读/写组。
//    挂起期间最多等几十拍（时隙表每 256 拍必然有换向空档），不影响保持时间。
//  【代价】刷新时刻比标称稍晚（几十拍 ≈ 0.x µs），相对 15.6µs 周期可忽略。
//-----------------------------------------------------------------------------
localparam [11:0] REF_DIV = 12'd1562;

reg [11:0] ref_div;
reg        ref_pend;            // 到点了，等一个空闲窗口再发
reg  [2:0] ref_quiet;           // 连续空闲拍数（饱和到 3）
reg [12:0] ref_wait;            // 挂起等待拍数（超时兜底，防止刷新被无限推迟）

always @(posedge Clk or posedge Rst) begin
    if (Rst) begin
        ref_div   <= 12'd0;
        ref_pend  <= 1'b0;
        ref_quiet <= 3'd0;
        ref_wait  <= 13'd0;
    end else begin
        // ---- 周期计数 ----
        if (ref_div == REF_DIV - 12'd1)
            ref_div   <= 12'd0;
        else
            ref_div   <= ref_div + 12'd1;

        // ---- 到点挂起 / 打出去后清掉（保证只发一拍）----
        if (ref_div == REF_DIV - 12'd1)
            ref_pend  <= 1'b1;
        else if (ref_pend && ((ref_quiet == 3'd3) || (ref_wait == 13'h1FFF)))
            ref_pend  <= 1'b0;

        // ---- 空闲窗口检测：Sdr_busy 连续 4 拍为 0 才算"真的空" ----
        if (Sdr_busy)
            ref_quiet <= 3'd0;
        else if (ref_quiet != 3'd3)
            ref_quiet <= ref_quiet + 3'd1;

        // ---- 兜底：挂起超过 8191 拍就强行发，绝不让刷新被饿死 ----
        if (ref_pend && ref_div != REF_DIV - 12'd1)
            ref_wait  <= (ref_wait == 13'h1FFF) ? ref_wait : (ref_wait + 13'd1);
        else
            ref_wait  <= 13'd0;
    end
end

wire app_ref_req = ref_pend && ((ref_quiet == 3'd3) || (ref_wait == 13'h1FFF));
assign App_ref_req = app_ref_req;   // ★ 引出给 sdr_port_sched，用于让纹理组首避开这一拍

//-----------------------------------------------------------------------------
// 官方 SDRAM 控制器 IP
//   self_refresh_open = 1'b1：打开自刷新，SDRAM 会自己定期刷新电容，数据不会掉
//                             （DRAM 靠电容存数据，必须每隔 64ms 刷一遍）
//   参数和端口都是官方定义好的，不要改，改了可能对不上 IP 内部逻辑。
//-----------------------------------------------------------------------------
sdr_as_ram  #( .self_refresh_open(1'b1))
	u2_ram( 
		.Sdr_clk(Clk),                  // 主时钟
		.Sdr_clk_sft(Clk_sft),          // 相移时钟
		.Rst(Rst),
			  			  
		.Sdr_init_done(Sdr_init_done),
		.Sdr_init_ref_vld(Sdr_init_ref_vld),
		.Sdr_busy(Sdr_busy),
		
		.App_ref_req(app_ref_req),      // ★ 周期请求刷新，每 1562 拍一次（见上面那段）
		
        .App_wr_en(App_wr_en), 
        .App_wr_addr(App_wr_addr),  	
		.App_wr_dm(App_wr_dm),
		.App_wr_din(App_wr_din),

		.App_rd_en(App_rd_en),
		.App_rd_addr(App_rd_addr),
		.Sdr_rd_en	(Sdr_rd_en),
		.Sdr_rd_dout(Sdr_rd_dout),
	
		// 以下直接连到 SDRAM 物理引脚
		.SDRAM_CLK(SDRAM_CLK),
		.SDR_RAS(SDR_RAS),
		.SDR_CAS(SDR_CAS),
		.SDR_WE(SDR_WE),
		.SDR_BA(SDR_BA),
		.SDR_ADDR(SDR_ADDR),
		.SDR_DM(SDR_DM),
		.SDR_DQ(SDR_DQ)	
	);

// CKE 时钟使能直接拉高：SDRAM 一直处于使能状态
assign SDR_CKE=1'b1;

//-----------------------------------------------------------------------------
// SDRAM 器件模型：EG_PHY_SDRAM_2M_32 表示 2M x 32bit
//   2M 个地址 × 32bit = 64Mbit = 8MB
//   这行代码不是"逻辑"，而是告诉综合工具"我这个 FPGA 上挂的是这么一颗 SDRAM"，
//   工具会据此把信号分配到正确的引脚上。
//
//   注意 addr 只连了 [10:0]：SDRAM 的行/列地址是分时复用的，
//   11 根地址线通过 RAS/CAS 分两次送行地址和列地址。
//-----------------------------------------------------------------------------
//`ifndef SIMULATION
	EG_PHY_SDRAM_2M_32 sdram(
		.clk(SDRAM_CLK),
		.ras_n(SDR_RAS),        // 行地址选通，低有效
		.cas_n(SDR_CAS),        // 列地址选通，低有效
		.we_n(SDR_WE),          // 写使能，低有效
		.addr(SDR_ADDR[10:0]),  // 11 位复用地址线
		.ba(SDR_BA),            // Bank 选择（4 个 bank）
		.dq(SDR_DQ),            // 双向数据
		.cs_n(1'b0),            // 片选常低 = 一直选中
		.dm0(SDR_DM[0]),        // 4 个字节的写掩码
		.dm1(SDR_DM[1]),
		.dm2(SDR_DM[2]),
		.dm3(SDR_DM[3]),
		.cke(1'b1)              // 时钟使能常高
		);
//`endif

endmodule
