//=============================================================================
// 文件名   : I2S_receiver.v
// 模块名   : I2S_receiver
// 功能     : 【I2S 接收器（把串行音频位流还原成左右声道采样值）】
//            这是 hdmi_audio_tone_i2s_64fs.v 的"反过程"：
//            发送端把数据一位一位串出去，接收端就得一位一位收回来，
//            拼成 24bit 的左右声道样本。
//
// 为什么工程里既有发送又有接收？
//   典型用途是"自测"：把发送端直接接到接收端，
//   看收到的数据和你发出的音频是否一致，从而验证音频链路是否正常。
//   这在 FPGA 开发里很常见，叫"回环测试（loopback test）"。
//
// 一、接收 I2S 的三个关键难点（初学者容易踩坑的地方）
//
//   ① 跨时钟域（CDC）
//      I2S 的 BCLK/LRCK/DOUT 来自"音频那边"，而 I_clk 是本地的系统时钟。
//      两者频率不同、相位无关，直接采样会不稳。
//      解决办法：用三级触发器（_1d/_2d/_3d）逐级同步，这叫"打三拍"。
//      —— 为什么要三级？一级不够可靠（亚稳态），三级可以把出错概率降到极低。
//
//   ② 边沿检测
//      S_i2s_bclk_p_edge = ~S_i2s_bclk_3d & S_i2s_bclk_2d
//      意思是"上一拍是 0、这一拍是 1" —— 即检测 BCLK 的上升沿。
//      每次检测到 BCLK 上升沿，就采样一位数据。
//      （用 _3d 和 _2d 比较，而不是和 I_i2s_BCLK 直接比，
//        是因为 _3d 是对齐到本地时钟域的那个稳定值。）
//
//   ③ LRCK 与 BCLK 的相位配合
//      代码里用 S_i2s_lrck_sync 在"BCLK 上升沿"上重新锁存一次 LRCK，
//      这样判断左右声道时，LRCK 的值和当前的数据位是同一时刻的，不会错位。
//      —— 这个技巧非常巧妙，是异步接口设计的常见手法。
//
// 二、数据是怎么拼回来的？
//   每个 BCLK 上升沿，把 DOUT 的值"挤"进移位寄存器的最低位，整体左移：
//       S_xxx_shift_data <= {S_xxx_shift_data[62:0], S_i2s_dout_3d};
//   因为 I2S 是"高位先发（MSB first）"，所以收完 32 位后，
//   数据就"靠左对齐"在 64bit 寄存器的高位，取 [63:40] 就是 24bit 有效数据。
//
// 三、输出时机
//   左声道数据在"LRCK 上升沿"这一刻收完 -> 锁存到 S_left_data_lock；
//   右声道数据在"LRCK 下降沿"这一刻收完 -> 此时左声道也早就锁好了，
//   于是同时输出 O_audio_left_data 和 O_audio_right_data，并打一个 valid 脉冲。
//=============================================================================

module I2S_receiver (
    input wire       I_clk,        // 本地系统时钟（不是音频时钟！）
    input wire       I_rst,        // 复位，高有效
  
    input wire       I_i2s_BCLK,   // I2S 位时钟（异步输入）
    input wire       I_i2s_LRCK,   // I2S 左右声道时钟（异步输入）
    input wire       I_i2s_DOUT,   // I2S 串行数据（异步输入）

    output reg       O_audio_valid,     // 一帧（左右各一个采样）数据有效脉冲
    output reg[23:0] O_audio_left_data, // 左声道 24bit 采样
    output reg[23:0] O_audio_right_data // 右声道 24bit 采样
);


    // ---- 三级同步触发器：把异步信号安全地搬进 I_clk 时钟域 ----
    reg       S_i2s_bclk_1d;
    reg       S_i2s_bclk_2d;
    reg       S_i2s_bclk_3d;
  
    wire      S_i2s_bclk_p_edge;      // BCLK 上升沿（本地域）
  
    reg       S_i2s_lrck_1d;
    reg       S_i2s_lrck_2d;
    reg       S_i2s_lrck_3d;
    reg       S_i2s_lrck_sync;        // 在 BCLK 沿上重锁存的 LRCK
    wire      S_i2s_lrck_p_edge;      // LRCK 上升沿
    wire      S_i2s_lrck_n_edge;      // LRCK 下降沿
  
    reg       S_i2s_dout_1d;
    reg       S_i2s_dout_2d;
    reg       S_i2s_dout_3d;

    reg[63:0] S_left_shift_data;      // 左声道移位寄存器
    reg[63:0] S_right_shift_data;     // 右声道移位寄存器

    reg       S_left_data_valid;      // 左声道收完标志
    reg       S_right_data_valid;     // 右声道收完标志

    reg[63:0] S_left_data_lock;       // 左声道数据锁存（等右声道收完一起输出）

    //-----------------------------------------------------------------
    // 三级同步：三根信号各打三拍
    //-----------------------------------------------------------------
    always @(posedge I_clk) begin
        S_i2s_bclk_1d <= I_i2s_BCLK;
        S_i2s_bclk_2d <= S_i2s_bclk_1d;
        S_i2s_bclk_3d <= S_i2s_bclk_2d;

        S_i2s_lrck_1d <= I_i2s_LRCK;
        S_i2s_lrck_2d <= S_i2s_lrck_1d;
        S_i2s_lrck_3d <= S_i2s_lrck_2d;

        S_i2s_dout_1d <= I_i2s_DOUT;
        S_i2s_dout_2d <= S_i2s_dout_1d;
        S_i2s_dout_3d <= S_i2s_dout_2d;
    end

    // BCLK 上升沿检测（在本地时钟域里看）
    assign S_i2s_bclk_p_edge = ~S_i2s_bclk_3d & S_i2s_bclk_2d;


    //-----------------------------------------------------------------
    // 在 BCLK 上升沿的瞬间，重新锁存 LRCK
    // 目的：让"判断左右声道"和"接收当前数据位"发生在同一时刻，
    //       避免因为同步延迟导致的声道错位。
    //-----------------------------------------------------------------
    always @(posedge I_clk) begin
        if(S_i2s_bclk_p_edge)
            S_i2s_lrck_sync <= S_i2s_lrck_3d;
        else
            S_i2s_lrck_sync <= S_i2s_lrck_sync;
    end

    // LRCK 的上升沿（左->右切换点）和 下降沿（右->左切换点）
    assign S_i2s_lrck_p_edge = ~S_i2s_lrck_sync & S_i2s_lrck_3d;

    assign S_i2s_lrck_n_edge = ~S_i2s_lrck_3d & S_i2s_lrck_sync;

    //-----------------------------------------------------------------
    // 数据接收：BCLK 上升沿时把 DOUT 挤进对应声道的移位寄存器
    //   S_i2s_lrck_sync = 1 -> 当前是右声道
    //   S_i2s_lrck_sync = 0 -> 当前是左声道
    //   切声道时顺便把另一路的寄存器清零，避免残留数据干扰。
    //-----------------------------------------------------------------
    always @(posedge I_clk) begin
        if(S_i2s_bclk_p_edge)
            if(S_i2s_lrck_sync)
                begin
                    S_right_shift_data <= {S_right_shift_data[62:0],S_i2s_dout_3d};
                    S_left_shift_data  <= 'd0;
                end
            else
                begin
                    S_right_shift_data <= 'd0;
                    S_left_shift_data  <= {S_left_shift_data[62:0],S_i2s_dout_3d};
                end
        else
            begin
                S_right_shift_data <= S_right_shift_data;
                S_left_shift_data  <= S_left_shift_data;
            end
    end


    // 左声道接收完成：BCLK 沿 + LRCK 上升沿同时出现
    always @(posedge I_clk) begin
        if(S_i2s_bclk_p_edge && S_i2s_lrck_p_edge)
            S_left_data_valid <= 1'b1;
        else
            S_left_data_valid <= 1'b0;
    end

    // 右声道接收完成：BCLK 沿 + LRCK 下降沿同时出现
    always @(posedge I_clk) begin
        if(S_i2s_bclk_p_edge && S_i2s_lrck_n_edge)
            S_right_data_valid <= 1'b1;
        else
            S_right_data_valid <= 1'b0;
    end

    //-----------------------------------------------------------------
    // 左声道先收到，先锁存起来；等右声道也收完再一起输出
    // （I2S 的一"帧"必须是左右声道同时给出，不能一前一后）
    //-----------------------------------------------------------------
    always @(posedge I_clk) begin
        if(S_left_data_valid)
            S_left_data_lock <= S_left_shift_data;
        else
            S_left_data_lock <= S_left_data_lock;
    end

    //-----------------------------------------------------------------
    // 输出：右声道收完的那一刻，把左右两个声道一起送出去
    //   取 [63:40] 是 24 位有效数据（因为 I2S 高位先发、数据靠左对齐）
    //-----------------------------------------------------------------
    always @(posedge I_clk) begin
        if(S_right_data_valid)
            begin
                O_audio_valid      <= 1'b1;
                O_audio_left_data  <= S_left_data_lock[63:40];
                O_audio_right_data <= S_right_shift_data[63:40];
            end
        else
            begin
                O_audio_valid      <= 1'b0;
                O_audio_left_data  <= 'd0;
                O_audio_right_data <= 'd0;
            end
    end

    // 下面是原来用来数声道内 bit 数的计数器，改成用移位寄存器后就不需要了，
    // 保留注释供参考。
    // always @(posedge I_clk) begin
    //     if(S_i2s_bclk_p_edge)
    //         begin
    //             if(S_i2s_lrck_3d)
    //                 begin
    //                     S_right_channel_cnt <= S_right_channel_cnt + 1'b1;
    //                     S_left_channel_cnt  <= 'd0;
    //                 end
    //             else    
    //                 begin
    //                     S_right_channel_cnt <= 'd0;
    //                     S_left_channel_cnt  <= S_left_channel_cnt + 1'b1;
    //                 end
    //         end
    //     else
    //         begin
    //             S_right_channel_cnt <= S_right_channel_cnt;
    //             S_left_channel_cnt  <= S_left_channel_cnt;
    //         end
    // end

endmodule
