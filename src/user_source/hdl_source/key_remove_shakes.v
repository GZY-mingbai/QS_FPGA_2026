/*
===============================================================================
文件   : key_remove_shakes.v
作者   : xiong guo qiang
日期   : 2020/12/3 15:50

功能   : 【按键消抖 + 脉冲输出】（另一套实现，和 ax_debounce.v 是一类东西）

描述   :
    对按键操作进行消抖操作，每次按下产生一个时钟周期的高脉冲。

和 ax_debounce.v 的区别（两份都能用，用途略不同）：
    ax_debounce.v    —— 输出消抖后的电平 + 上升/下降沿脉冲，判据是"电平稳定 20ms"；
    key_remove_shakes.v —— 只输出一个触发脉冲，判据是"检测到按下沿后，等 10 万个时钟
                          再确认按键确实松开了"。写法更轻量，适合"按一下做一件事"。

为什么需要消抖？
    机械按键按下/松开的一瞬间，触点会来回弹跳十几次（几毫秒），
    直接接到逻辑里会一次按下被识别成好多次。必须等电平稳定后才认账。

本模块的四个步骤：
    ① 打四拍：把异步的按键信号同步进 I_clk 时钟域（防亚稳态）。
       —— 为什么打四拍比三拍多一拍？因为要同时拿到 "3d" 和 "4d" 两个值来比较。
    ② 检测按下沿（S_key_in_n_edge）：上一拍是 1、这一拍是 0 —— 即按键被按下。
       （注释里说"低有效"，所以按下 = 电平从 1 变 0。）
    ③ 检测到按下后，把 S_delay_en 拉高，开始数 100000 个时钟；
       数满就拉低 S_delay_en —— 这段时间就是"消抖等待期"。
       @50MHz 时 100000 个时钟 = 2ms，足够跳过抖动期。
    ④ 在 S_delay_en 的下降沿（消抖期结束那一刻），
       如果此时按键仍是按下状态，就输出一个 O_key_trig_out 高脉冲。
       —— 这就是"确认为一次有效按键"的信号。

如何使用 ?
    I_clk 接系统时钟，I_rst_n 是★低有效复位★，I_key_in 接按键引脚。
    O_key_trig_out 是单周期高脉冲，可以直接当"按了一次"的触发用。
===============================================================================
*/


module key_remove_shakes( 
	input wire      I_clk,          // 系统时钟
	input wire      I_rst_n,        // ★低有效复位★
	
	input wire      I_key_in,       // 原始按键输入（低有效：按下为 0）
	
	output reg      O_key_trig_out  // 有效按键触发脉冲（单周期高电平）
);

	///同步到处理时钟域，打三拍 —— 实际打了四拍（1d~4d）
	reg       S_key_in_sync_1d;      
	reg       S_key_in_sync_2d;      
	reg       S_key_in_sync_3d;      
	reg	      S_key_in_sync_4d;
	wire      S_key_in_n_edge;       // 按键下降沿（按下瞬间）
			  
	reg       S_delay_en;            // 消抖等待期使能
	reg       S_delay_en_1d;         // 打一拍，用于检测下降沿
	wire      S_delay_en_n_edge;     // 等待期结束的瞬间
	reg[16:0] S_delay_cnt;           // 消抖计数器（100000 需要 17bit）
	
	
	//--------------------------------------------------------------------
	// ① 四级同步：把按键信号安全地搬进本时钟域
	//--------------------------------------------------------------------
	always @ (posedge I_clk) begin
		S_key_in_sync_1d <= I_key_in;
		S_key_in_sync_2d <= S_key_in_sync_1d;
		S_key_in_sync_3d <= S_key_in_sync_2d;
		S_key_in_sync_4d <= S_key_in_sync_3d;
	end
	

	// ② 下降沿检测：上一拍还是 1，这一拍变成 0 —— 说明按键被按下
	assign S_key_in_n_edge = ~S_key_in_sync_3d & S_key_in_sync_4d;
	
	
	//--------------------------------------------------------------------
	// ③ 消抖等待期：检测到按下就开启等待，数满 100000 个时钟就关闭
	//--------------------------------------------------------------------
	always @ (posedge I_clk or negedge I_rst_n) begin
		if(!I_rst_n)
			S_delay_en <= 1'b0;
		else
			if(S_key_in_n_edge)             // 刚按下，开启等待期
				S_delay_en <= 1'b1;
			else if(S_delay_cnt == 'd100000) // 等够了，关闭等待期
				S_delay_en <= 1'b0;
			else
				S_delay_en <= S_delay_en;
	end
	
	// 等待期计数器
	always @ (posedge I_clk or negedge I_rst_n) begin
		if(!I_rst_n)
			S_delay_cnt <= 'd0;
		else
			if(S_delay_en)
				S_delay_cnt <= S_delay_cnt + 1'b1;
			else
				S_delay_cnt <= 'd0;      // 等待期结束就清零
	end
	
	
	// ④ S_delay_en 打一拍，用于检测它的下降沿
	always @ (posedge I_clk) begin
		S_delay_en_1d <= S_delay_en;
	end
	
	
	// 检测等待期结束（S_delay_en 从 1 变 0）的那一刻
	assign S_delay_en_n_edge = ~S_delay_en & S_delay_en_1d;
	
	
	// ⑤ 等待期结束 + 按键仍然是按下状态 -> 确认这是一次有效按键
	always @ (posedge I_clk) begin
		if(S_delay_en_n_edge && !S_key_in_sync_4d)
			O_key_trig_out <= 1'b1;
		else
			O_key_trig_out <= 1'b0;      // 只出一个时钟周期的高脉冲
	end
	

endmodule


//==============================================================================
//  board_input_debounce.v —— 【多路】按键/开关消抖（一套逻辑管 N 路）
//==============================================================================
//
//  【为什么又写一个】工程里已经有 key_remove_shakes / key_press_debounce /
//  ax_debounce 三份消抖了，但都是"一个模块管一根线"，板上 4 个键 + 4 个开关
//  要例化 8 次、8 套计数器。这个模块把"位宽"做成参数，一次例化管一整组。
//
//  【算法：1ms 采样 + 连续 N 次一致】
//    ① 先把引脚打两拍同步进 clk 域（按键/开关是异步信号，防亚稳态）；
//    ② 每 1ms 产生一个 tick，只在 tick 那一拍才看输入；
//    ③ 若采样值和当前稳定值不同 → 计数器 +1；一旦又相同 → 计数器清 0；
//    ④ 计数器攒满 DEBOUNCE_MS（20）→ 承认这次变化，更新稳定值并出脉冲。
//    ⇒ 只有"连续 20ms 都没变"才认账，抖动期间 count 反复清零，永远攒不满。
//    ⇒ 20ms 是机械按键的常规值：人手最快连击也在 30ms 以上，不会误吞。
//
//  【输出三路（都是本时钟域，clk = 50MHz）】
//    level   : 消抖后的稳定电平。**极性不变** —— 输入 0 就是 0。
//              本工程按键是"上拉 + 按下接地"，所以 level = 0 表示按下；
//              乒乓开关同理，0 表示拨到 ON（若实测反了，外面取反即可）。
//    press   : 稳定值从 1→0 的那一拍，输出**单周期高脉冲**（= 按下了）
//    rls     : 稳定值从 0→1 的那一拍，输出**单周期高脉冲**（= 松开了）
//
//  【为什么要 press 脉冲而不是只给电平】
//    电平在"按住"期间一直是 0，拿它直接驱动"按一下切换一次"的功能，
//    会在按住时每拍都切一次。用 press 就是天然的一键一次。
//
//  【复位】rst 高有效。注意：不要接"由本模块输入自己产生的复位"（会成环）。
//
//  【资源】每路：2 个同步 FF + 8 位计数器 + 3 个输出 FF ≈ 15 个触发器；
//          8 路一共 120 个左右，外加一个 16 位 tick 计数器，可忽略不计。
//==============================================================================
module board_input_debounce #(
    parameter integer WIDTH       = 4,          // 通道数（4 = 4 个按键）
    parameter integer CLK_HZ      = 50_000_000, // 时钟频率
    parameter integer DEBOUNCE_MS = 20          // 消抖窗口（毫秒）
)(
    input  wire             clk,                // 采样时钟（本模块唯一时钟域）
    input  wire             rst,                // 复位，★高有效★

    input  wire [WIDTH-1:0] din,                // 原始引脚（按下/ON = 0）

    output reg  [WIDTH-1:0] level,              // 消抖后的稳定电平
    output reg  [WIDTH-1:0] press,              // 按下沿：单周期高脉冲
    output reg  [WIDTH-1:0] rls                 // 松开沿：单周期高脉冲（release 是关键字，不能当端口名）
);

// 1ms 的时钟数：50MHz / 1000 = 50000
localparam integer TICK_DIV = (CLK_HZ / 1000);
// 攒够多少个 1ms 采样才认账（减 1 是因为计数器从 0 开始数）
localparam integer HOLD      = (DEBOUNCE_MS - 1);

//------------------------------------------------------------------
//  1ms 采样节拍：计数到 TICK_DIV-1 的那一拍 tick = 1
//------------------------------------------------------------------
reg [15:0] tick_cnt;
wire       tick = (tick_cnt == (TICK_DIV - 1));

always @(posedge clk or posedge rst) begin
    if (rst)
        tick_cnt <= 16'd0;
    else if (tick)
        tick_cnt <= 16'd0;
    else
        tick_cnt <= tick_cnt + 16'd1;
end

//------------------------------------------------------------------
//  每路一份独立的同步/计数/输出逻辑
//------------------------------------------------------------------
genvar i;
generate
    for (i = 0; i < WIDTH; i = i + 1) begin : g_ch

        reg [1:0] sync;     // 两级同步：第 1 级防亚稳态，第 2 级拿来用
        // ★ 2026-11-03 [AREA]：cnt 从 0 逐一数到 HOLD 就清零 ⇒ 5 位够（HOLD=19），
        //   且 `>= HOLD` 与 `== HOLD` 等价。
        reg [4:0] cnt;      // 连续"和稳定值不一致"的毫秒数

        always @(posedge clk or posedge rst) begin
            if (rst) begin
                // 复位默认当成"松开/OFF"（上拉电平 = 1），和板上实际状态一致
                sync       <= 2'b11;
                cnt        <= 5'd0;
                level[i]   <= 1'b1;
                press[i]   <= 1'b0;
                rls[i] <= 1'b0;
            end else begin
                sync <= {sync[0], din[i]};

                // 脉冲默认拉低：只有真正发生跳变那一拍才被下面置 1，
                // 所以宽度恒为 1 个时钟
                press[i]   <= 1'b0;
                rls[i] <= 1'b0;

                if (tick) begin
                    if (sync[1] == level[i]) begin
                        cnt <= 5'd0;                    // 没变化，重新计数
                    end else if (cnt == HOLD[4:0]) begin
                        cnt      <= 5'd0;
                        level[i] <= sync[1];            // 连续 20ms 都变过 = 认账
                        if (sync[1])
                            rls[i] <= 1'b1;         // 1 = 松开
                        else
                            press[i]   <= 1'b1;         // 0 = 按下
                    end else begin
                        cnt <= cnt + 5'd1;              // 还在观察期
                    end
                end
            end
        end

    end
endgenerate

endmodule
