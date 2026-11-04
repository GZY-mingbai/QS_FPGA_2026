//=============================================================================
// 文件名   : i2s_rom_player_single_pll_fix64_rom_ip.v
// 模块名   : i2s_rom_player_single_pll_fix64
// 功能     : 【ROM 波表音乐播放器】—— 用一张正弦波表 + 变速读表，合成 8 个音阶
//
// ★ 这是本工程音频部分"最讲究"的一个模块，也是从"方波哔哔声"升级到"真正乐音"的关键。★
//
// 一、和 hdmi_audio_tone_i2s_64fs.v 的本质区别
//   那个模块：相位累加器取最高位 -> 输出 ±AMP -> 只能出方波（音色硬、像报警声）。
//   本模块： 相位累加器的高位拿去做【地址】-> 查正弦波表 -> 输出真正的正弦波采样。
//   正弦波听起来就光滑柔和多了，接近真正的乐器声。
//
//   这叫做"DDS（直接数字频率合成）"或"波表合成（Wavetable Synthesis）"，
//   是几乎所有电子琴、合成器的工作原理，非常值得理解。
//
// 二、核心原理：怎么用一张表弹出不同的音？
//   表里只存了【一个周期的正弦波】，只有 48 个点（ROM_DEPTH=48），
//   地址用 6 位（ADDR_WIDTH=6，2^6=64 足够覆盖 48）。
//
//   关键技巧：改变"读表的步进速度"就能改变音高！
//     走得慢 -> 一个正弦周期要很多个采样才读完 -> 频率低 -> 低音
//     走得快 -> 很快读完整张表           -> 频率高 -> 高音
//
//   具体做法（定点数 Q16.16 格式）：
//     S_phase_acc_q16 是一个 32bit 定点数，高 16 位是整数部分（也就是表地址），
//     低 16 位是小数部分（表示"读表读到了第几个点的几分之几"）。
//     每个采样点： S_phase_acc_q16 += S_phase_step_q16
//     然后用【高 16 位】作为 ROM 地址： S_rom_addr = S_phase_acc_q16[31:16]
//     —— 因为地址只取整数部分，小数部分自然被"忽略"，产生了小数步进的平滑效果。
//
//   步进值怎么算？
//     phase_step = round(freq × ROM_DEPTH / SAMPLE_RATE × 2^16)
//     例如 la（A4, 440Hz）：
//       440 × 48 / 48000 × 65536 = 28835.8 ≈ 28836 （正对应 STEP_LA4）✓
//     验证另一个：do（C4, 261.63Hz）：
//       261.63 × 48 / 48000 × 65536 = 17145.9 ≈ 17146 （对应 STEP_DO4）✓
//     —— 这两个数字都对上了，说明公式和代码一致，你可以放心修改。
//
//   相位回绕：正弦是周期函数，地址超过表长就要绕回来：
//     if (S_phase_sum_q16 >= TABLE_SIZE_Q16)   // 超过 48.0（整数部分超了）
//         S_phase_next_q16 = S_phase_sum_q16 - TABLE_SIZE_Q16;   // 减掉一个周期
//     TABLE_SIZE_Q16 = ROM_DEPTH << 16 = 48 × 65536 = 48.0（Q16.16 表示的 48）
//
// 三、为什么有一段 S_init_step 的初始化流程？
//   因为 ROM 是【同步读】的：给地址后要等一个时钟数据才出来。
//   模块一上电时不能直接用表数据（那会是无效值），所以用 3 个步骤：
//     step0: 清所有寄存器，发地址 0
//     step1: 等一拍，锁存 ROM 输出的第一个样本
//     step2: 把第一个样本装进发送寄存器
//     default: 正式进入正常播放
//   —— 这是处理"同步 ROM 上电取数"的标准套路，避免开头发出"咔"的爆音。
//
// 四、S_capture_pending 是干什么的？
//   ROM 数据要一个时钟才出来。每当相位更新、新地址发出后，
//   就把 capture_pending 置 1；下一个时钟再把 ROM 输出取回来存进 S_sample_next。
//   —— 这叫"流水线对齐"：因为取数有一拍延迟，所以要用标志记住"这一拍该收货了"。
//
// 五、旋律与时长
//   NOTE_LEN_SAMPLES = 12000，@48kHz = 0.25 秒一个音（比前两个版本快一倍，显得更活泼）。
//   循环顺序：do re mi fa so la si do -> 回到 do。
//
// 如何使用 ?
//   依赖一个 ROM IP：AUDIO_SAMPLE_ROM（端口 doa[23:0] / addra / clka），
//   里面用 .mif 文件填了 48 个 24bit 的正弦波样本。
//   I_mclk 接 12.288MHz，输出 I2S 三线。想换音色就换 ROM 里的波表内容。
//=============================================================================

module i2s_rom_player_single_pll_fix64 #(
    parameter integer SAMPLE_RATE       = 48000,   // 采样率
    parameter integer SAMPLE_BITS       = 24,      // 样本位宽
    parameter integer ROM_DEPTH         = 48,      // 波表深度（一个正弦周期 48 个点）
    parameter integer ADDR_WIDTH        = 6,       // 波表地址位宽（2^6 = 64 > 48）
    parameter integer NOTE_LEN_SAMPLES  = 12000    // 每个音 0.25s @ 48kHz
)(
    input  wire I_mclk,      // 12.288MHz 音频主时钟
    input  wire I_rst,       // 复位，高有效
    output reg  O_i2s_BCLK,  // I2S 位时钟 = 6.144MHz
    output reg  O_i2s_LRCK,  // I2S 声道时钟 = 48kHz
    output reg  O_i2s_DOUT   // I2S 串行数据
);

    // ============================================================
    // 单端口 ROM IP 版：24bit / 48 深度正弦波表 + 8 音阶旋律发生器
    //
    // ROM IP 端口：module AUDIO_SAMPLE_ROM ( doa, addra, clka );
    // doa[23:0] = 单声道 24bit PCM 波表样本
    //
    // I2S 时序与当前 I2S_receiver 兼容：
    //   MCLK = 12.288MHz
    //   BCLK = 6.144MHz = MCLK / 2
    //   LRCK = 48kHz    = BCLK / 128
    //   每声道发送 64bit，其中 [63:40] 为 24bit 采样，低 40bit 补 0
    //
    // 旋律：do re mi fa so la si do (C4 D4 E4 F4 G4 A4 B4 C5)
    // 实现方式：ROM 只存一个 48 点正弦单周期，通过改变读表步进值来合成音高
    // ============================================================

    // 一个完整正弦周期对应的 Q16.16 定点值 = 48 × 65536
    localparam [31:0] TABLE_SIZE_Q16 = (ROM_DEPTH << 16);

    // phase_step = round(freq * ROM_DEPTH / SAMPLE_RATE * 2^16)
    // 每一个数字都可以按这个公式自己验算 → 见文件头注释
    localparam [31:0] STEP_DO4 = 32'd17146; // C4  261.63Hz
    localparam [31:0] STEP_RE4 = 32'd19245; // D4  293.66Hz
    localparam [31:0] STEP_MI4 = 32'd21603; // E4  329.63Hz
    localparam [31:0] STEP_FA4 = 32'd22887; // F4  349.23Hz
    localparam [31:0] STEP_SO4 = 32'd25690; // G4  392.00Hz
    localparam [31:0] STEP_LA4 = 32'd28836; // A4  440.00Hz
    localparam [31:0] STEP_SI4 = 32'd32367; // B4  493.88Hz
    localparam [31:0] STEP_DO5 = 32'd34292; // C5  523.25Hz

    reg        S_mclk_div;    // MCLK 二分频标志
    reg [6:0]  S_bit_cnt;     // 帧内位计数（0~127）
    reg [63:0] S_left_word;   // 左声道发送字
    reg [63:0] S_right_word;  // 右声道发送字

    reg [ADDR_WIDTH-1:0] S_rom_addr;   // ROM 读地址
    wire [23:0]          S_rom_q;      // ROM 读出数据（有一拍延迟）

    reg [23:0]  S_sample_next;     // 下一个要播放的样本
    reg         S_capture_pending; // "该接收 ROM 输出了"标志
    reg [1:0]   S_init_step;       // 初始化步骤（0~2，之后进入 3 = 正常运行）

    reg [31:0]  S_phase_acc_q16;   // 相位累加器（Q16.16 定点）
    reg [31:0]  S_phase_step_q16;  // 当前音对应的步进值
    reg [15:0]  S_note_cnt;        // 当前音已持续的采样数
    reg [2:0]   S_note_sel;        // 当前音阶 0~7

    // 组合逻辑算出的"下一拍值"（这样 ROM 地址可以提前一拍准备好）
    reg [31:0]  S_phase_next_q16;
    reg [31:0]  S_phase_sum_q16;
    reg [31:0]  S_phase_step_next_q16;
    reg [15:0]  S_note_cnt_next;
    reg [2:0]   S_note_sel_next;
    reg [ADDR_WIDTH-1:0] S_rom_addr_next;

    // ROM 例化：同步读，clka = I_mclk；给出 addra 后下一个 clk 才出 doa
    AUDIO_SAMPLE_ROM u_audio_sample_rom (
        .doa   ( S_rom_q    ),
        .addra ( S_rom_addr ),
        .clka  ( I_mclk     )
    );

    // 音阶 -> 步进值 的查找函数
    function [31:0] note_step;
        input [2:0] note_idx;
        begin
            case (note_idx)
                3'd0: note_step = STEP_DO4;
                3'd1: note_step = STEP_RE4;
                3'd2: note_step = STEP_MI4;
                3'd3: note_step = STEP_FA4;
                3'd4: note_step = STEP_SO4;
                3'd5: note_step = STEP_LA4;
                3'd6: note_step = STEP_SI4;
                default: note_step = STEP_DO5;
            endcase
        end
    endfunction

    //--------------------------------------------------------------------
    // 组合逻辑：提前算出"下一拍"的所有值（尤其是下一个 ROM 地址）
    // 这样 ROM 可以在这一拍就开始取数，等下一拍正好能用到。
    // 这叫"时序前推"，是处理同步 ROM/IP 的标准手法。
    //--------------------------------------------------------------------
    always @(*) begin
        // 默认：保持不变
        S_phase_step_next_q16 = S_phase_step_q16;
        S_note_cnt_next       = S_note_cnt;
        S_note_sel_next       = S_note_sel;
        // 相位先加上当前步进
        S_phase_sum_q16       = S_phase_acc_q16 + S_phase_step_q16;

        if (S_note_cnt == NOTE_LEN_SAMPLES - 1) begin
            // ---- 当前音唱满了，换下一个音 ----
            if (S_note_sel == 3'd7)
                S_note_sel_next = 3'd0;          // 最高音后回到最低音
            else
                S_note_sel_next = S_note_sel + 3'd1;

            // 计算新音的步进值（用下一拍的音阶索引）
            S_phase_step_next_q16 = note_step((S_note_sel == 3'd7) ? 3'd0 : (S_note_sel + 3'd1));
            S_note_cnt_next       = 16'd0;       // 计数器清零
            S_phase_next_q16      = 32'd0;       // 相位从 0 开始（保证从波表开头起音）
        end else begin
            // ---- 正常推进 ----
            S_note_cnt_next = S_note_cnt + 16'd1;

            // 相位回绕：超过一个完整周期就减去一个周期
            if (S_phase_sum_q16 >= TABLE_SIZE_Q16)
                S_phase_next_q16 = S_phase_sum_q16 - TABLE_SIZE_Q16;
            else
                S_phase_next_q16 = S_phase_sum_q16;
        end

        // ★ 取 Q16.16 的整数部分（高 16 位）作为 ROM 地址 ★
        S_rom_addr_next = S_phase_next_q16[31:16];
    end

    //--------------------------------------------------------------------
    // 主时序逻辑：初始化 + I2S 发送 + 相位推进
    //--------------------------------------------------------------------
    always @(posedge I_mclk or posedge I_rst) begin
        if (I_rst) begin
            S_mclk_div         <= 1'b0;
            S_bit_cnt          <= 7'd0;
            S_left_word        <= 64'd0;
            S_right_word       <= 64'd0;
            S_rom_addr         <= {ADDR_WIDTH{1'b0}};
            S_sample_next      <= 24'd0;
            S_capture_pending  <= 1'b0;
            S_init_step        <= 2'd0;

            S_phase_acc_q16    <= 32'd0;
            S_phase_step_q16   <= STEP_DO4;      // 从 do 开始
            S_note_cnt         <= 16'd0;
            S_note_sel         <= 3'd0;

            O_i2s_BCLK         <= 1'b0;
            O_i2s_LRCK         <= 1'b0;
            O_i2s_DOUT         <= 1'b0;
        end else begin
            // 收货：ROM 数据到了一拍，取回来存下
            if (S_capture_pending) begin
                S_sample_next     <= S_rom_q;
                S_capture_pending <= 1'b0;
            end

            case (S_init_step)
                // ---- 步骤 0：全部清干净，发地址 0 ----
                2'd0: begin
                    S_rom_addr        <= {ADDR_WIDTH{1'b0}};
                    S_phase_acc_q16   <= 32'd0;
                    S_phase_step_q16  <= STEP_DO4;
                    S_note_cnt        <= 16'd0;
                    S_note_sel        <= 3'd0;
                    S_init_step       <= 2'd1;
                    O_i2s_BCLK        <= 1'b0;
                    O_i2s_LRCK        <= 1'b0;
                    O_i2s_DOUT        <= 1'b0;
                    S_mclk_div        <= 1'b0;
                    S_bit_cnt         <= 7'd0;
                end

                // ---- 步骤 1：等 ROM 出数据（同步 ROM 有一拍延迟）----
                2'd1: begin
                    S_sample_next     <= S_rom_q;   // 这时候 S_rom_q 就是波表第 0 个点
                    S_rom_addr        <= {ADDR_WIDTH{1'b0}};
                    S_capture_pending <= 1'b1;
                    S_init_step       <= 2'd2;
                end

                // ---- 步骤 2：把第一个样本装进左右声道发送字 ----
                2'd2: begin
                    S_left_word       <= {S_sample_next, 40'd0};
                    S_right_word      <= {S_sample_next, 40'd0};
                    S_init_step       <= 2'd3;      // 初始化完成，进入正常播放
                end

                // ---- 步骤 3（default）：正常 I2S 播放 + 推相位 ----
                default: begin
                    S_mclk_div <= ~S_mclk_div;

                    if (S_mclk_div == 1'b0) begin
                        if (O_i2s_BCLK == 1'b1) begin
                            // —— 与 i2s_test_tone_gen 相同的发送时序 ——
                            if (S_bit_cnt < 7'd63) begin
                                // 左声道
                                O_i2s_LRCK <= 1'b0;
                                O_i2s_DOUT <= S_left_word[63];
                                S_left_word <= {S_left_word[62:0], 1'b0};
                                S_bit_cnt   <= S_bit_cnt + 7'd1;
                            end else if (S_bit_cnt == 7'd63) begin
                                // 左声道最后一位，切到右声道
                                O_i2s_LRCK <= 1'b1;
                                O_i2s_DOUT <= S_left_word[63];
                                S_left_word <= {S_left_word[62:0], 1'b0};
                                S_bit_cnt   <= 7'd64;
                            end else if (S_bit_cnt < 7'd127) begin
                                // 右声道
                                O_i2s_LRCK   <= 1'b1;
                                O_i2s_DOUT   <= S_right_word[63];
                                S_right_word <= {S_right_word[62:0], 1'b0};
                                S_bit_cnt    <= S_bit_cnt + 7'd1;
                            end else begin
                                // 一帧结束：装载新样本 + 推进相位 + 更新 ROM 地址
                                O_i2s_LRCK        <= 1'b0;
                                O_i2s_DOUT        <= S_right_word[63];
                                S_left_word       <= {S_sample_next, 40'd0};   // 新样本（24bit 放高位）
                                S_right_word      <= {S_sample_next, 40'd0};
                                S_bit_cnt         <= 7'd0;

                                // 把组合逻辑算好的"下一拍值"提交
                                S_phase_acc_q16   <= S_phase_next_q16;
                                S_phase_step_q16  <= S_phase_step_next_q16;
                                S_note_cnt        <= S_note_cnt_next;
                                S_note_sel        <= S_note_sel_next;
                                S_rom_addr        <= S_rom_addr_next;   // 提前给 ROM 地址
                                S_capture_pending <= 1'b1;              // 下一拍收货
                            end
                        end

                        O_i2s_BCLK <= ~O_i2s_BCLK;   // BCLK 二分频
                    end
                end
            endcase
        end
    end

endmodule
