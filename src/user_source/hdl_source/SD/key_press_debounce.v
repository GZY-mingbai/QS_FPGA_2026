//==============================================================================
//  key_press_debounce.v —— 按键消抖模块
//==============================================================================
//
//  【这个模块干什么的】
//  机械按键在按下和松开的瞬间，金属触点会"弹跳"几下（专业叫"抖动"），
//  持续几毫秒。如果直接把按键信号接到计数器上，按一次可能被数成按了 5 次。
//  这个模块的作用就是：等信号稳定一段时间后才承认它真的变了，
//  并且"每次真正按下"只输出一个时钟周期的高脉冲。
//
//  【按键的默认电平】这个模块假设：松开 = 1，按下 = 0（低电平有效）。
//  这是因为板卡上的按键通常一端接地，按下时把引脚拉低。
//
//  【实现原理：延迟确认法】
//    1. 先用两级触发器把异步的按键信号同步到本时钟域（消除亚稳态）；
//    2. 把"同步后的当前值"和"已确认的稳定值 button_stable"比较：
//       - 两者相同 → 说明没变化，计数器清零（不用数了）；
//       - 两者不同 → 说明信号刚变，开始数数（20ms）；
//    3. 数满 20ms 后信号还是不同的 → 承认这次变化是真的：
//       如果是从 1 变 0（按下），就输出一个脉冲，并更新稳定值。
//
//  【为什么第一级要打两拍】
//  按键是异步信号，跟时钟没有任何关系，随时可能变化。
//  直接在任何逻辑里用会有亚稳态风险。标准做法就是先打两拍同步。
//
//  【历史】这个模块原来定义在 sd_card_bmp.v 文件末尾。重构后 sd_card_bmp
//  被拆成了 frame_writer（写帧时序）+ tf_bmp_pixel_source（像素源），
//  消抖功能归像素源用，所以抽出来单独成一个文件。
//
//==============================================================================
module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,  // 时钟频率（Hz）
    parameter integer DEBOUNCE_MS = 20            // 消抖窗口（毫秒）
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 按键输入：松开=1，按下=0
    output reg  press_pulse   // 输出：按下一次产生一个时钟周期的高脉冲
);

// 计算需要数多少个时钟。
// 100_000_000 / 1000 = 100_000 → 1ms 的时钟数；再 × 20 = 200 万个时钟。
// 【为什么用 localparam 而不是 parameter】localparam 不能被例化时覆盖，
// 它是纯粹的内部常量，表示"由参数计算出来的中间结果"。
localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;      // 同步第 1 级
reg button_sync1;      // 同步第 2 级（这级的值可以作为"当前采样值"使用）
reg button_stable;     // 已确认的稳定电平
reg [31:0] cnt;        // 消抖计数器

always @(posedge clk or posedge rst) begin
    if (rst) begin
        // 复位时都设成 1（"松开"状态），因为按键默认就是松开的
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        // ---- 第 1 步：两级同步 ----
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;

        // ---- 第 2 步：脉冲默认拉低（只有一个时钟宽度）----
        press_pulse  <= 1'b0;

        // ---- 第 3 步：判断信号是否稳定 ----
        if (button_sync1 == button_stable) begin
            // 当前值和已确认的稳定值一致 → 没有变化，计数器清零
            cnt <= 32'd0;
        end else begin
            // 检测到差异，开始/继续数数
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                // 数满 20ms 后差异依然存在 → 承认这次变化
                // 如果是从 1 变 0（即按下），输出一个脉冲
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;

                // 更新稳定值，计数器清零
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                // 还没数满，继续数
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
