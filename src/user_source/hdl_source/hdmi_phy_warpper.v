//=============================================================================
// 文件名   : hdmi_phy_wrapper.v
// 模块名   : hdmi_phy_wrapper
// 功能     : 【HDMI 物理层（PHY）顶层封装】
//            把 4 路 TMDS 数据（3 路颜色 + 1 路时钟）分别送去 4 个串行器，
//            变成 4 对高速差分信号输出到 HDMI 接口。
//
// 一、HDMI 的物理结构（初学者必看）
//   HDMI 线里有 4 对差分线（TMDS 通道）：
//     CH0 = 蓝色 + 行/场同步
//     CH1 = 绿色 + 控制信号
//     CH2 = 红色 + 音频/辅助数据
//     CLK = 像素时钟（也要用差分方式传）
//   每一对线每个像素周期要传 10 bit（这就是"TMDS 编码"，用 10bit 保证直流平衡）。
//
//   于是：像素时钟 25MHz，每个时钟要串出 10 bit，
//        串行时钟就需要 25MHz × 10 = 250MHz。
//   —— 这就是本模块为什么需要两个时钟：
//        I_pixel_clk  = 25MHz  （并行域，10bit 一起走）
//        I_serial_clk = 250MHz （串行域，一位一位走）
//
// 二、本模块做了什么"看起来没用"的事？
//   你会发现上面那 4 个 assign 把 I_tmds_channel_0[0]~[9] 重新排了一遍顺序：
//        S_tmds_data_ch0 = {I[0], I[1], ..., I[9]}
//   即整体"倒序"。这不是多余的 ——
//   因为后面的移位寄存器是"从最高位开始往左移"送出的，
//   而 IP 输出的 bit 顺序恰好是反的，所以要在这里掰正，
//   否则屏幕上颜色会完全错乱（比如红色显示成蓝色）。
//   （实践里如果颜色不对，第一件事就是检查这里的位序。）
//
// 三、为什么叫 "wrapper"（包装）？
//   因为它自己不干活，只是"把 4 个一模一样的模块摆开、连好线"。
//   4 个通道逻辑完全一样，所以只写一个 lane_lvds_10_1 然后例化 4 次即可 ——
//   这就是模块化设计的价值：同样的活儿干 4 遍，代码只写 1 份。
//
// 如何使用 ?
//   I_pixel_clk 接视频 PLL 的 25MHz，I_serial_clk 接 250MHz（要相位对齐！），
//   四个 I_tmds_channel_x 接 HDMI IP 输出的 TMDS 编码数据，
//   四个输出 O_tmds_chX_p 直接接 FPGA 的 HDMI 差分输出引脚。
//=============================================================================

module hdmi_phy_wrapper#(
    parameter DEVICE = "EG"      // 目标器件系列，EG 表示安路 Eagle 系列（本板 EG4S20 属于这一族）
    )(
    input wire      I_pixel_clk,   // 像素时钟（并行域，25MHz）
    input wire      I_serial_clk,  // 串行时钟（250MHz = 像素时钟 × 10）
    input wire      I_rst,         // 复位，高有效

    input wire[9:0] I_tmds_channel_0,      // CH0 的 10bit TMDS 编码数据
    input wire[9:0] I_tmds_channel_1,      // CH1
    input wire[9:0] I_tmds_channel_2,      // CH2
    input wire[9:0] I_tmds_channel_clk,    // 时钟通道
    
    output wire     O_tmds_ch0_p,   // CH0 差分输出（正端）
    output wire     O_tmds_ch1_p,   // CH1 差分输出
    output wire     O_tmds_ch2_p,   // CH2 差分输出
    output wire     O_tmds_clk_p    // 时钟差分输出
);
    
    wire[9:0] S_tmds_data_ch0;
    wire[9:0] S_tmds_data_ch1;
    wire[9:0] S_tmds_data_ch2;
//------------------------------------------------------------------------------
//  ★★ 2026-10-30 [RST-SYNC]：复位按串行时钟域"同步释放" ★★
//------------------------------------------------------------------------------
//  【问题】I_rst 来自顶层 rst_all（50MHz 域的 21 位计数器组合比较，扇出 200+）。
//    直接接到 4 条 lane 的 ODDR/移位寄存器异步复位上 ⇒ 布线后 recovery 违例
//    −13.7ns（起点 rst_rel_cnt、终点 HDMI_D0~D2/CLK_P 的 .rst），是整份时序
//    报告里最差的一条，也把布线资源从真正关键路径上挤走了。
//  【做法】标准结构"异步置位、同步释放"：3 级同步器把释放边沿对齐到串行时钟。
//    第 1 级允许亚稳态（recovery 检查就止步于此），第 2/3 级给出干净释放。
//  【代价】复位释放晚 3 个串行时钟（= 3/125MHz ≈ 24ns），对功能无影响。
//------------------------------------------------------------------------------
reg [2:0] rst_ser_sr;
always @(posedge I_serial_clk or posedge I_rst) begin
    if (I_rst) rst_ser_sr <= 3'b111;
    else       rst_ser_sr <= {rst_ser_sr[1:0], 1'b0};
end
wire I_rst_s = rst_ser_sr[2];      // 串行域同步后的复位

    wire[9:0] S_tmds_data_clk;

    // ---- 位序翻转（bit swap）：把 [0] 放到最高位 ----
    // 用拼接 {a,b,c...} 手工重排，等价于"倒序"。
    // 不这么写的话，输出颜色会错乱。这是适配 IP 位序的常规手段。
    assign S_tmds_data_ch0 = {I_tmds_channel_0[0],
                              I_tmds_channel_0[1],
                              I_tmds_channel_0[2],
                              I_tmds_channel_0[3],
                              I_tmds_channel_0[4],
                              I_tmds_channel_0[5],
                              I_tmds_channel_0[6],
                              I_tmds_channel_0[7],
                              I_tmds_channel_0[8],
                              I_tmds_channel_0[9]};

    assign S_tmds_data_ch1 = {I_tmds_channel_1[0],
                              I_tmds_channel_1[1],
                              I_tmds_channel_1[2],
                              I_tmds_channel_1[3],
                              I_tmds_channel_1[4],
                              I_tmds_channel_1[5],
                              I_tmds_channel_1[6],
                              I_tmds_channel_1[7],
                              I_tmds_channel_1[8],
                              I_tmds_channel_1[9]};

    assign S_tmds_data_ch2 = {I_tmds_channel_2[0],
                              I_tmds_channel_2[1],
                              I_tmds_channel_2[2],
                              I_tmds_channel_2[3],
                              I_tmds_channel_2[4],
                              I_tmds_channel_2[5],
                              I_tmds_channel_2[6],
                              I_tmds_channel_2[7],
                              I_tmds_channel_2[8],
                              I_tmds_channel_2[9]};

    assign S_tmds_data_clk = {I_tmds_channel_clk[0],
                              I_tmds_channel_clk[1],
                              I_tmds_channel_clk[2],
                              I_tmds_channel_clk[3],
                              I_tmds_channel_clk[4],
                              I_tmds_channel_clk[5],
                              I_tmds_channel_clk[6],
                              I_tmds_channel_clk[7],
                              I_tmds_channel_clk[8],
                              I_tmds_channel_clk[9]};


    // ---- 4 个通道的串行器（结构完全相同，只是接的信号不同）----
    // 每个串行器把 10bit 并行数据变成 1 路 10:1 的高速串行位流
    lane_lvds_10_1 #(
        .DEVICE ( DEVICE )    
    )u0_lane_lvds_8_1(          // 模块名叫 8_1 是历史遗留，实际是 10:1
        .I_pixel_clk  ( I_pixel_clk     ),
        .I_serial_clk ( I_serial_clk    ),
        .I_rst        ( I_rst_s         ),

        .I_data_in    ( S_tmds_data_ch0 ),
        .O_serial_out ( O_tmds_ch0_p    )
    );


    lane_lvds_10_1 #(
        .DEVICE ( DEVICE )    
    )u1_lane_lvds_8_1(
        .I_pixel_clk  ( I_pixel_clk     ),
        .I_serial_clk ( I_serial_clk    ),
        .I_rst        ( I_rst_s         ),
        
        .I_data_in    ( S_tmds_data_ch1 ),
        .O_serial_out ( O_tmds_ch1_p    )
    );


    lane_lvds_10_1 #(
        .DEVICE ( DEVICE )    
    )u2_lane_lvds_8_1(
        .I_pixel_clk  ( I_pixel_clk     ),
        .I_serial_clk ( I_serial_clk    ),
        .I_rst        ( I_rst_s         ),
        
        .I_data_in    ( S_tmds_data_ch2 ),
        .O_serial_out ( O_tmds_ch2_p    )
    );


    lane_lvds_10_1 #(
        .DEVICE ( DEVICE )    
    )u3_lane_lvds_8_1(
        .I_pixel_clk  ( I_pixel_clk     ),
        .I_serial_clk ( I_serial_clk    ),
        .I_rst        ( I_rst_s         ),
        
        .I_data_in    ( S_tmds_data_clk ),
        .O_serial_out ( O_tmds_clk_p    )
    );


endmodule
