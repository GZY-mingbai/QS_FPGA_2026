//==============================================================================
//  key_debounce9.v —— 紧凑型多路消抖（共享 2ms tick + 每路 4 位计数）
//==============================================================================
//
//  【为什么要它】板上 4 个按键 + 五向键 5 个 + 4 个拨码开关 = 13 路。
//    原 board_input_debounce 每路 5 位计数、1ms tick（16 位分频）。
//    这里把采样间隔放到 2ms，连续 10 次一致（= 20ms，窗口与原来相同）⇒
//    每路计数只要 4 位；13 路共用**一个** 17 位 tick 分频器。
//    模块名沿用计划里的 key_debounce9（默认 9 路），顶层按 13 路例化：
//    拨码开关也并进来，省掉第二套分频器（比"拨码保留原实例"更省）。
//
//  【行为与原 board_input_debounce 一致】
//    · 输入先 2FF 同步；
//    · 只在 tick 拍看输入：与稳定值不同 → 计数 +1；相同 → 清 0；
//      攒满 HOLD+1 次（10 × 2ms = 20ms）才认账；
//    · level 极性不变（按下 / ON = 0）；press = 1→0 单拍，rls = 0→1 单拍；
//    · 复位时 level 全 1（= 松开 / OFF）。
//    唯一差别：采样粒度从 1ms 变成 2ms（20ms 窗口不变，人手感觉不到）。
//
//  【资源】tick 17 位 + 每路（2 同步 + 4 计数 + 3 输出）FF。
//==============================================================================
module key_debounce9 #(
    parameter integer WIDTH       = 9,
    parameter integer CLK_HZ      = 50_000_000,
    parameter integer TICK_MS     = 2,          // 采样间隔
    parameter integer DEBOUNCE_MS = 20          // 消抖窗口（必须是 TICK_MS 的整数倍，且 /TICK_MS ≤ 16）
)(
    input  wire             clk,
    input  wire             rst,                // 高有效（顶层接 rst_input，不能接由本模块输入产生的复位）
    input  wire [WIDTH-1:0] din,                // 原始引脚（按下 / ON = 0）
    output reg  [WIDTH-1:0] level,
    output reg  [WIDTH-1:0] press,
    output reg  [WIDTH-1:0] rls
);

localparam integer TICK_DIV = (CLK_HZ / 1000) * TICK_MS;     // 50MHz × 2ms = 100,000
localparam integer HOLD     = (DEBOUNCE_MS / TICK_MS) - 1;   // 9 ⇒ 第 10 次认账
localparam [16:0]  TICK_TOP = TICK_DIV - 1;
localparam [3:0]   HOLD4    = HOLD;

//------------------------------------------------------------------
//  共享 tick
//------------------------------------------------------------------
reg  [16:0] tick_cnt;
wire        tick = (tick_cnt == TICK_TOP);

always @(posedge clk or posedge rst) begin
    if (rst)       tick_cnt <= 17'd0;
    else if (tick) tick_cnt <= 17'd0;
    else           tick_cnt <= tick_cnt + 17'd1;
end

//------------------------------------------------------------------
//  每路：同步 + 4 位计数 + 输出
//------------------------------------------------------------------
genvar i;
generate
    for (i = 0; i < WIDTH; i = i + 1) begin : g_ch
        reg [1:0] sync;
        reg [3:0] cnt;

        always @(posedge clk or posedge rst) begin
            if (rst) begin
                sync     <= 2'b11;
                cnt      <= 4'd0;
                level[i] <= 1'b1;
                press[i] <= 1'b0;
                rls[i]   <= 1'b0;
            end else begin
                sync     <= {sync[0], din[i]};
                press[i] <= 1'b0;
                rls[i]   <= 1'b0;
                if (tick) begin
                    if (sync[1] == level[i]) begin
                        cnt <= 4'd0;
                    end else if (cnt == HOLD4) begin
                        cnt      <= 4'd0;
                        level[i] <= sync[1];
                        if (sync[1]) rls[i]   <= 1'b1;
                        else         press[i] <= 1'b1;
                    end else begin
                        cnt <= cnt + 4'd1;
                    end
                end
            end
        end
    end
endgenerate

endmodule
