//=============================================================================
// 文件名   : startup_pulse.v
// 模块名   : startup_pulse
// 功能     : 【上电延时脉冲发生器】
//            复位释放后，延迟 CNT_MAX 个时钟，产生一个单周期的高脉冲，
//            整个系统运行期间只产生这一次（"上电一次"信号）。
//
// 为什么要这样一个模块？
//   FPGA 刚配置完成、复位刚释放的时候，外部器件（比如 SD 卡、HDMI 接收端）
//   还没有准备好。如果立刻就开始发命令，对方可能不响应，导致初始化失败。
//   所以需要一个"先等一会儿"的信号，等系统稳定后再正式开始工作。
//   本工程用它来启动 SD 卡初始化。
//
// 工作原理（很简单，用 S_done 当"已经发过"的锁）：
//   ① 复位时 S_done=0；
//   ② 每拍计数器 +1；
//   ③ 数到 CNT_MAX-1 时，输出一个高脉冲，同时把 S_done 置 1 并锁存计数器，
//      之后再也不会进入这个分支 —— 保证整个运行期只发一次。
//
// 默认 CNT_MAX = 100000，@50MHz 就是 2ms 的延时。
//
// 如何使用 ?
//   例化后把 O_pulse 接到需要"上电启动"的模块使能端即可。
//   觉得等得不够久就调大 CNT_MAX（例化时用 #(.CNT_MAX(20'd500000)) 覆盖）。
//=============================================================================

module startup_pulse #(
    parameter [19:0] CNT_MAX = 20'd100000    // 延时时钟数，默认 100000（@50MHz = 2ms）
)(
    input  wire I_clk,      // 系统时钟
    input  wire I_rst,      // 复位，高有效
    output reg  O_pulse     // 上电脉冲（单周期高电平，只出现一次）
);

reg [19:0] S_cnt;      // 延时计数器
reg        S_done;     // "已经发过脉冲"标志，防止重复触发

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        S_cnt   <= 20'd0;
        S_done  <= 1'b0;
        O_pulse <= 1'b0;
    end
    else begin
        O_pulse <= 1'b0;        // 默认输出低，只有下面那一拍才拉高

        if (!S_done) begin      // 还没发过
            if (S_cnt == CNT_MAX - 1'b1) begin
                S_cnt   <= S_cnt;   // 计数器停住
                S_done  <= 1'b1;    // 标记已发过
                O_pulse <= 1'b1;    // 发出脉冲（只此一拍）
            end
            else begin
                S_cnt <= S_cnt + 1'b1;   // 继续数
            end
        end
        // S_done 之后什么都不做，O_pulse 始终保持 0
    end
end

endmodule
