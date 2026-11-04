//=============================================================================
// 文件名   : ax_debounce.v
// 模块名   : ax_debounce
// 功能     : 【按键消抖模块（带边沿检测）】
//            把物理按键的抖动毛刺滤掉，输出稳定的按键电平，
//            并额外给出"按下瞬间"和"松开瞬间"两个单周期脉冲信号。
//
// 为什么需要消抖？（初学者必看）
//   机械按键在按下/松开的一瞬间，金属触点并不是"啪"地一下就接触好了，
//   而是在几毫秒内会来回弹跳十几次。用示波器看就是一段密集的方波毛刺。
//   如果直接把它接到 FPGA 的时钟敏感电路，一次按下会被当成十几次按下。
//   解决办法就是消抖：等电平稳定一段时间（本模块用 20ms）后才认账。
//
// 本模块的原理（经典三段式写法）：
//   ① DFF1/DFF2 两级触发器：把异步的按键信号同步到本时钟域（防亚稳态），
//      并让 DFF2 保存"上一拍的值"以便比较。
//   ② q_reset = DFF1 ^ DFF2：异或判断"电平是否发生了变化"。
//      只要还在变（抖动），q_reset 就为 1，计数器一直被清零，从头再数。
//   ③ 当电平稳定、q_reset 保持 0 并持续数满 TIMER_MAX_VAL（20ms），
//      才认为这是一个"有效按键"，把稳定的电平写入 button_out。
//
// 为什么是 20ms 这个数？
//   TIMER_MAX_VAL = MAX_TIME(ms) * 1000 * FREQ(MHz)
//   = 20 × 1000 × 50 = 1,000,000 个时钟 @50MHz = 20ms。
//   机械按键抖动一般 <10ms，取 20ms 有安全余量。
//   （注意：这需要 32bit 计数器，因为 100 万超过了 16bit 的范围。）
//
// 如何使用 ?
//   由顶层例化，button_in 接按键引脚，clk 接 50MHz 系统时钟。
//   三个输出按需使用：
//     button_out      —— 消抖后的电平（按住为 1，松开为 0）
//     button_posedge  —— 按下瞬间的单周期脉冲（常用于"按一下执行一次"）
//     button_negedge  —— 松开瞬间的单周期脉冲
//   本工程中 SD 卡的"换图按键"用的就是这套。
//=============================================================================

module  ax_debounce 
(
    input       clk, 
    input       rst, 
    input       button_in,            // 原始按键输入（未消抖）
    output reg  button_posedge,       // 按下瞬间脉冲（单周期）
    output reg  button_negedge,       // 松开瞬间脉冲（单周期）
    output reg  button_out            // 消抖后的稳定电平
);
//// ---------------- internal constants --------------
parameter N = 32 ;           // debounce timer bitwidth - 消抖计数器位宽（20ms@50MHz 需要 20bit 以上，留 32bit 富余）
parameter FREQ = 50;         //model clock :Mhz - 时钟频率，单位 MHz
parameter MAX_TIME = 20;     //ms - 消抖时间，单位 ms
localparam TIMER_MAX_VAL =   MAX_TIME * 1000 * FREQ;   // 换算成时钟个数 = 1_000_000
////---------------- internal variables ---------------
reg  [N-1 : 0]  q_reg;      // timing regs - 消抖计数器当前值
reg  [N-1 : 0]  q_next;     // 消抖计数器下一拍值（组合逻辑算出来）
reg DFF1, DFF2;             // input flip-flops - 两级同步触发器
wire q_add;                 // control flags - 计数器"继续加"标志
wire q_reset;               // 计数器"清零"标志
reg button_out_d0;          // button_out 打一拍，用于边沿检测
//// ------------------------------------------------------

////contenious assignment for counter control
assign q_reset = (DFF1  ^ DFF2);          // xor input flip flops to look for level chage to reset counter
                                          // 异或：两个值不同 = 电平刚变化（还没稳定）=> 计数器清零
assign q_add = ~(q_reg == TIMER_MAX_VAL); // add to counter when q_reg msb is equal to 0
                                          // 没数到头就继续加（数到 20ms 就停住不再加）
    
//// combo counter to manage q_next 
// 组合逻辑计算计数器的下一个值（真值表思路）：
//   q_reset=1 -> 清零（电平还在跳）
//   q_reset=0 且未数满 -> 加 1
//   q_reset=0 且已数满 -> 保持
always @ ( q_reset, q_add, q_reg)
begin
    case( {q_reset , q_add})
        2'b00 :                     // 电平稳定 + 已数满 -> 保持
                q_next <= q_reg;
        2'b01 :                     // 电平稳定 + 没数满 -> 继续数
                q_next <= q_reg + 1;
        default :                   // q_reset=1 -> 清零
                q_next <= { N {1'b0} };
    endcase     
end

//// Flip flop inputs and q_reg update
// 时序逻辑：同步输入、更新计数器
always @ ( posedge clk or posedge rst)
begin
    if(rst == 1'b1)
    begin
        DFF1 <= 1'b0;
        DFF2 <= 1'b0;
        q_reg <= { N {1'b0} };
    end
    else
    begin
        DFF1 <= button_in;      // 第一级同步（防亚稳态）
        DFF2 <= DFF1;           // 第二级：保存上一拍的稳定值，用于和 DFF1 比较
        q_reg <= q_next;        // 更新计数器
    end
end

//// counter control
// 只有计数器数满（即已经稳定持续 20ms）时，才把 DFF2 这个"稳定值"输出
always @ ( posedge clk or posedge rst)
begin
	if(rst == 1'b1)
		button_out <= 1'b1;              // 复位时当作"松开"（按键通常低有效，具体看硬件）
    else if(q_reg == TIMER_MAX_VAL)      // 已稳定 20ms，认账
        button_out <= DFF2;
    else
        button_out <= button_out;        // 未稳定，保持原值不变
end

//-----------------------------------------------------------------------------
// 边沿检测：把 button_out 打一拍得 button_out_d0，然后比较"现在"和"上一拍"
//   上升沿（按下）：上一拍是 0、这一拍是 1  -> ~button_out_d0 & button_out
//   下降沿（松开）：上一拍是 1、这一拍是 0  -> button_out_d0 & ~button_out
//   这样按一下只产生"一个时钟周期"的脉冲，方便当触发信号用。
//   这是 FPGA 里最常用的边沿检测写法，务必记住。
//-----------------------------------------------------------------------------
always @ ( posedge clk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		button_out_d0 <= 1'b1;
		button_posedge <= 1'b0;
		button_negedge <= 1'b0;
	end
	else
	begin
		button_out_d0 <= button_out;
		button_posedge <= ~button_out_d0 & button_out;   // 检测上升沿（按下）
		button_negedge <= button_out_d0 & ~button_out;   // 检测下降沿（松开）
	end	
end
endmodule

