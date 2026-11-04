//=============================================================================
// 文件名   : lane_lvds_10_1.v
// 模块名   : lane_lvds_10_1
// 功能     : 【10:1 串行器（Serializer）】—— 把 10bit 并行数据变成 1 路高速串行位流
//
// ★ 这是整个 HDMI 通路里速度要求最高的地方，也是初学者最该看懂的"跨时钟域技巧"。★
//
// 一、要解决什么问题？
//   像素域：25MHz，一次给你 10 bit（一个像素的 TMDS 编码）
//   串行域：250MHz，一次只能往外吐 1 bit
//   于是需要"把 10bit 拆成 10 拍，每拍吐 1 bit"，这个动作就叫"串行化"。
//
// 二、为什么用 ODDR 而不是普通寄存器？（关键！）
//   FPGA 里普通寄存器都挂在同一个时钟的上升沿上，一个周期只能输出 1 位。
//   但我们需要的串行时钟是 250MHz，而 ODDR 是"DDR 输出寄存器"，
//   它在时钟的【上升沿】和【下降沿】各输出一位 ——
//   也就是说，用 125MHz 的时钟就能输出 250Mbps 的数据！
//   （DDR = Double Data Rate，双倍数据率，就是内存条上说的那个 DDR。）
//   这行代码的作用：把数据准备好，ODDR 自己负责按下沿/上沿交替送出。
//
// 三、enclk 是怎么产生的？（本模块最巧妙的地方）
//   问题：S_odd_even 是 25MHz 域的信号，要在 250MHz 域里用它，属于跨时钟域。
//        而且我们希望它每 10 个串行时钟产生一个"装载"脉冲。
//
//   做法（经典技巧：让信号"抖动"一下，再用边沿检测"抓"它）：
//     ① S_odd_even 在 25MHz 域每拍翻转一次 —— 也就是 12.5MHz 的方波。
//     ② 拿到 250MHz 域打两拍（S_odd_even_1d / _2d）—— 相当于采样这个慢信号。
//     ③ S_load_en = S_odd_even_1d ^ S_odd_even_2d ——
//        异或检测"变化"，每次方波跳变就得到一个 250MHz 域的窄脉冲。
//        因为方波是 12.5MHz（周期 80ns），每 80ns 跳变一次；
//        再算上打了两拍和后面又打两拍，最后 S_load_en_2d 的脉冲正好是
//        "每 10 个串行时钟来一次" —— 也就是 250MHz / 10 = 25MHz。
//        ★ 这正好就是像素时钟！★
//     ④ 于是 S_load_en_2d 就可以当作"每 10 拍装载一次新数据"的使能信号。
//
//   —— 这个技巧叫"脉冲同步 + 边沿检测"，是跨时钟域产生慢速使能的标准手法，
//      几乎在每个高速接口里都会遇到。理解它，你就跨过了 FPGA 的一道大坎。
//
// 四、移位怎么做的？
//   S_shift_data <= S_shift_data << 2;
//   装载时把 10bit 并行数据装进来，然后每个串行时钟左移 2 位，
//   因为 ODDR 每个时钟要吃掉 2 位（上面说了 DDR 一周期出 2 位）。
//     - 2 位出到 ODDR
//     - 10 位数据移 5 次就发完了（5 × 2 = 10）✓
//
// 五、generate 和 parameter 的用法
//   `generate if(DEVICE == "EF2") ... else if ...` 相当于"编译期的 if"，
//   根据传进来的器件型号参数，只保留对应的那一段代码例化。
//   安路各系列 FPGA 的 ODDR 原语名不一样（EG_LOGIC_ODDR / EF2_LOGIC_ODDR ...），
//   所以需要这样写一份代码适配多个器件。
//   ★ 本板是 EG4S20，所以实际生效的是最后一个 else if(DEVICE == "EG") 分支 ★
//=============================================================================


module lane_lvds_10_1 #(
    parameter DEVICE = "EG"       // 目标器件系列，本板用 "EG"
    )(
    input wire      I_pixel_clk,   // 像素时钟（25MHz，并行域）
    input wire      I_serial_clk,  // 串行时钟（250MHz，串行域）
    input wire      I_rst,         // 复位，高有效

    input wire[9:0] I_data_in,     // 10bit 并行输入（一个像素的 TMDS 编码）

    output wire     O_serial_out   // 串行输出（1 bit 位流）
);

    reg      S_odd_even;      // 25MHz 域的翻转标志（产生装载使能的源头）
    reg      S_odd_even_1d;   // 同步到 250MHz 域第一拍
    reg      S_odd_even_2d;   // 第二拍
    reg      S_load_en;       // 检测到变化 -> 脉冲
    reg      S_load_en_1d;    // 再延两拍，用作真正装载信号
    reg      S_load_en_2d;
    reg[9:0] S_shift_data;    // 10bit 移位寄存器
    reg[1:0] S_oddr_data;     // ODDR 的两个数据输入


    //--------------------------------------------------------------------
    // ① 在像素域产生翻转标志：每拍取反 => 12.5MHz 方波
    //--------------------------------------------------------------------
    always @(posedge I_pixel_clk or posedge I_rst) begin
        if(I_rst)
            S_odd_even <= 1'b0;
        else
            S_odd_even <= ~S_odd_even;
    end

    //--------------------------------------------------------------------
    // ② 同步到串行域（打两拍，防亚稳态）
    //--------------------------------------------------------------------
    always @(posedge I_serial_clk) begin
        S_odd_even_1d <= S_odd_even;
        S_odd_even_2d <= S_odd_even_1d;
    end

    //--------------------------------------------------------------------
    // ③ 边沿检测 + 再打两拍 => 得到频率为像素时钟的装载脉冲
    //    S_load_en    = 1d ^ 2d  （检测变化）
    //    S_load_en_1d / 2d        （对齐时序，让脉冲落点正确）
    //--------------------------------------------------------------------
    always @(posedge I_serial_clk) begin
        S_load_en    <= S_odd_even_1d ^ S_odd_even_2d;
        S_load_en_1d <= S_load_en;
        S_load_en_2d <= S_load_en_1d;
    end

    //--------------------------------------------------------------------
    // ④ 移位寄存器：
    //    装载脉冲来 -> 把 10bit 输入整体装进来
    //    否则       -> 左移 2 位（因为 ODDR 每个时钟消耗 2 位）
    //--------------------------------------------------------------------
    always @(posedge I_serial_clk or posedge I_rst) begin
        if(I_rst)
            S_shift_data <= 'd0;
        else
            if(S_load_en_2d)
                S_shift_data <= I_data_in;          // 装载新像素
            else 
                S_shift_data <= S_shift_data << 2;  // 左移 2 位
    end

    //--------------------------------------------------------------------
    // ⑤ 把最高 2 位交给 ODDR（DDR 会在上/下两个沿分别送出）
    //--------------------------------------------------------------------
    always @(posedge I_serial_clk) begin
        S_oddr_data <= S_shift_data[9:8];
    end

    //--------------------------------------------------------------------
    // ⑥ 按器件系列例化对应的 ODDR 原语（编译期选择，只保留一个）
    //    ODDR 的 d1 在时钟上升沿输出，d0 在下降沿输出。
    //    —— 所以一个 125MHz 时钟就能产生 250Mbps 的数据率。
    //    ★ 本板 DEVICE="EG"，实际走第 5 个分支 ★
    //--------------------------------------------------------------------
    generate
        if(DEVICE == "EF2")
            begin
                EF2_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "EF3")
            begin
                EF3_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "EF4")
            begin
                EF4_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "SF1")
            begin
                SF1_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "EG")              // ★ 本板走这一支 ★
            begin
                EG_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "PH1A")
            begin
                PH1_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "PH1P")
            begin
                PH1P_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
        else if(DEVICE == "DR1")
            begin
                DR1_LOGIC_ODDR U_EG4_ODDR(
                    .clk   ( I_serial_clk   ),
                    .rst   ( I_rst          ),

                    .d1    ( S_oddr_data[0] ),
                    .d0    ( S_oddr_data[1] ),

                    .q     ( O_serial_out   )
                );
            end
    endgenerate
    
endmodule
