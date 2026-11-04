//=============================================================================
//  sd_audio_reader.v —— SD 卡 WAV 裸数据读取器（音频应用层）
//=============================================================================
//
//  【它在整条链路里的位置】
//
//      SD 卡 ──sd_card_top──▶ sd_bus_arbiter ──▶ 本模块 ──▶ FIFO ──▶ wav_player
//                                ▲                 （扇区 → 32bit 样本）
//                          图像装载器
//
//  【它干的活：把"扇区流"变成"样本字流"】
//  WAV 的 PCM 数据在卡上是**连续的裸字节**：
//      L15 L14 ... L0 R15 R14 ... R0 | L15 ... （小端，每 2 字节一个声道）
//  本模块按扇区顺序读，每 4 个字节拼成一个 32bit 字 `{L16, R16}` 推进 FIFO，
//  **正好是 wav_player 的 I_smp_data 需要的格式**（它再左移 8 位补成 24bit）。
//
//  【为什么 HDL 里不解析 RIFF/FAT】
//  和 BMP 一样：偏移由 PC 端脚本（tools/wav_fat_offset.py）算好，写进顶层
//  parameter。HDL 只做"从第 N 扇区连续读 M 个扇区"，逻辑量最小、也最容易验。
//
//  【三个偏移量】
//    BASE_SECTOR  文件（或直接是 data chunk）所在的首扇区
//    SKIP_BYTES   首扇区开头要丢掉的字节数（= data_offset - BASE_SECTOR*512）
//                 必须 < 512；每循环一轮只在第一个扇区生效一次
//    TOTAL_WORDS  一共要播多少个 32bit 样本字（= data_size / 4）
//
//  【流控：为什么"够放一整扇区"才敢发请求】
//  SD 卡一旦开始吐一个扇区，中途是停不下来的（512 字节连续时钟）。
//  如果 FIFO 只剩 10 个字，这一扇区的 128 个字必然溢出。
//  所以发请求的门槛是"剩余空间 ≥ 128 字"，也就是水位 ≤ REQ_LEVEL。
//  这样**一个扇区一定能整段放进去**，永远不需要丢样本。
//
//  【循环】
//  播完 TOTAL_WORDS 个字后不停止，而是回到 BASE_SECTOR 重新开始 ——
//  中间 FIFO 里还压着 ~20ms 的存量，接缝听不出来。
//
//  【关闭（I_en=0）】
//    · 立刻停止往 FIFO 写字（FIFO 由外层同时复位）；
//    · 如果正在读一个扇区，**必须把它吃完**（否则仲裁器等不到 end）；
//      所以进 abort，收完 end 才真正回空闲；
//    · 计数器（扇区/样本/跳字节）在关闭的那一拍就归零，
//      下次打开一定从 BASE_SECTOR 的第一个样本开始。
//
//  【时钟域】sd_card_clk（50MHz），与 sd_card_top / 仲裁器同域。
//=============================================================================

module sd_audio_reader #(
    parameter integer BASE_SECTOR  = 0,         // WAV data 起始扇区
    parameter integer SKIP_BYTES   = 44,        // 首扇区要跳过的字节（< 512）
    parameter integer TOTAL_WORDS  = 0,         // 要播的 32bit 样本字数
    parameter integer FIFO_AW      = 10,        // FIFO 地址位宽（深度 = 2^AW）
    //--------------------------------------------------------------------------
    //  水位门槛（单位：字）
    //    PREFILL      ≥ 这个值算"预灌完成"，播放器才放行（必须与 audio_wav_path 一致）
    //    REQ_LEVEL    ≤ 这个值 → 继续预取（深度 1024 时留 256 字余量）
    //    URGENT_LEVEL ≤ 这个值 → 抢占 SD 总线（图像让路）
    //--------------------------------------------------------------------------
    parameter integer PREFILL      = 512,
    parameter integer REQ_LEVEL    = 768,
    parameter integer URGENT_LEVEL = 256
)(
    input  wire             I_clk,              // sd_card_clk 50MHz
    input  wire             I_rst,              // 高有效
    input  wire             I_en,               // 1 = 播放（已同步到本域）
    // ★ 2026-10-29：OSD 系统页"循环播放"
    //   1 = 本轮播完回到 BASE_SECTOR 继续（原行为）；0 = 播完停住，等下次开播
    input  wire             I_loop,
    //  ★ 2026-11-04：运行时换曲（文件页打开 WAV）。I_trk_sec = 文件首扇区。
    //    换曲后不再用 SKIP_BYTES/TOTAL_WORDS：从文件头逐字节找 "data" 块，
    //    其后 4 字节就是数据长度（字节）⇒ 样本字数 = 长度/4。44 字节标准头、
    //    带 LIST 的 78 字节头都适用。开机默认曲目仍按编译期参数，行为不变。
    input  wire             I_trk_set,
    input  wire [31:0]      I_trk_sec,

    //------------------------------------------------------------------
    //  SD 客户口（接 sd_bus_arbiter）
    //------------------------------------------------------------------
    output reg              O_sd_req,
    output reg  [31:0]      O_sd_addr,
    output wire             O_sd_urgent,
    input  wire [7:0]       I_sd_data,
    input  wire             I_sd_dv,            // 字节有效
    input  wire             I_sd_end,           // 本扇区结束（单拍）

    //------------------------------------------------------------------
    //  FIFO 写口（接 img_afifo 的写侧）
    //------------------------------------------------------------------
    input  wire [FIFO_AW:0] I_fifo_level,       // 写域已用字数
    input  wire             I_fifo_full,
    output reg              O_fifo_we,
    output reg  [31:0]      O_fifo_data,
    output reg              O_drop,             // 粘滞：发生过溢出丢弃（不该出现）
    output wire             O_pass_done,        // ★ 2026-10-27：本轮样本已取完（电平）
    // ★ 2026-11-08【音乐结束→自动切图】：把“每遍播完”做成两个干净的单拍。
    //   O_pass_end  = 真·播完：loop=0 时 FIFO 排空后的单拍（原语义，trk_done 用）
    //   O_pass_tick = 每一遍数据结束都发一拍（loop=1 也有）
    //     · 上层“自动切换=音乐结束”用 O_pass_tick：循环播放时不存在“整首结束”，
    //       只能按“每一遍结束”算。旧实现直接等 O_pass_end，loop=1 时永不成立。
    output wire             O_pass_end,
    output wire             O_pass_tick
);

//=============================================================================
//  状态编码
//=============================================================================
localparam S_IDLE = 2'd0;       // 等播放
localparam S_REQ  = 2'd1;       // 已发请求，正在收一个扇区
localparam S_GAP  = 2'd2;       // 扇区之间：req 先拉低，等 FIFO 有空间

reg  [1:0]  st;
reg  [31:0] sec;                // 下一个要读的扇区号
reg  [9:0]  skip;               // 本扇区开头还要丢几个字节
reg  [1:0]  lane;               // 组字进度（0..3）
reg  [23:0] shf;                // 组字移位寄存器（前 3 个字节）
reg  [31:0] left;               // 还剩多少个 32bit 字没产出
reg         pass_done;          // 本轮样本已取完（只等把当前扇区吃完）
reg         abort;              // 收到关闭请求，吃完当前扇区就回空闲
reg         en_d;               // 上一拍的 I_en（下降沿检测）
reg         pre_ok;             // 预灌完成过（写域）；清零前一直算"急需数据"

reg  [31:0] base_r;             // 起始扇区（开机 = BASE_SECTOR，换曲后 = 新文件）
reg         user;               // 1 = 运行时选的曲（头要现解析）
reg         hdr;                // 1 = 正在找 "data" 块
reg         trk_d;              // 换曲后一拍：按"关播"同一条路径重来（用新的 base_r）
// 换曲 = 一次"关播 + 重开"：正在收的扇区收完再回到新起点
wire en_fall = (en_d && !I_en) || trk_d;

//=============================================================================
//  【抢占信号 —— 这里踩过一个致命死锁，务必看懂再改】
//=============================================================================
//  最初的写法是：urgent = I_en && (level <= URGENT_LEVEL)
//  【为什么会死锁】播放器要"FIFO 里有 PREFILL(512) 个字"才肯开播；
//  而开播之前**没人读 FIFO**，水位只会涨不会跌。
//  板上开机时图像装载器一直在抢总线，音频只抢到 3 个扇区（364 字）就被
//  "图像优先"切走；此时 364 > 256，urgent=0 ⇒ 再也抢不到总线 ⇒
//  水位永远停在 364 ⇒ 永远不满足 512 ⇒ 永远不开播 ⇒ **永久静音**。
//  （无图像负载时音频能连续灌满 512，所以单元仿真看不出来；
//    只有"上电即播放 + 图像持续装载"的整链仿真才复现。）
//
//  【修法】预灌没完成之前，音频**始终** urgent —— 这段时间它必须拿总线，
//  否则永远填不满。灌满过一次之后 pre_ok 锁存，回到正常的低水位抢占逻辑
//  （此时播放器在消费，水位会自然下降）。
//=============================================================================
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst)          pre_ok <= 1'b0;
    else if (!I_en)     pre_ok <= 1'b0;             // 关播 → 下次重新预灌
    else if (I_fifo_level >= PREFILL) pre_ok <= 1'b1;
end

// ★ 2026-10-30：给"预灌期间一直 urgent"加**时间上限**（10ms @50MHz）。
//   【为什么必须有】预灌只需 ~4 个扇区（1.5ms）就能到 512；但如果预灌因为任何
//   原因填不满（FIFO 写侧/水位 CDC 出问题），urgent 就永远为 1 ⇒ 音频把 SD 总线
//   **占死** ⇒ 图像装载器一个字节都拿不到 ⇒ 信息条 0×0、相册全是 SDRAM 花屏。
//   上板实测：SW2 拨到"不播放"（I_en=0）图片立刻正常，就是这个原因。
//   加了上限之后：预灌仍有 10ms 的绝对优先（是正常所需 1.5ms 的 6 倍），
//   超过就认输让图像走 —— 音频自己的 bug 不该把整机拖死。
localparam [18:0] PRE_TMO = 19'd500_000;     // 10ms @50MHz
reg [18:0] pre_tmo;
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst)                     pre_tmo <= 19'd0;
    else if (!I_en)                pre_tmo <= 19'd0;
    else if (!pre_ok && (pre_tmo < PRE_TMO)) pre_tmo <= pre_tmo + 19'd1;
end

assign O_sd_urgent = I_en && ((!pre_ok && (pre_tmo < PRE_TMO)) ||
                              (I_fifo_level <= URGENT_LEVEL));

//=============================================================================
//  主逻辑
//=============================================================================
always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        st          <= S_IDLE;
        sec         <= BASE_SECTOR;
        skip        <= SKIP_BYTES;
        lane        <= 2'd0;
        shf         <= 24'd0;
        left        <= TOTAL_WORDS;
        pass_done   <= 1'b0;
        abort       <= 1'b0;
        en_d        <= 1'b0;
        O_sd_req    <= 1'b0;
        O_sd_addr   <= BASE_SECTOR;
        O_fifo_we   <= 1'b0;
        O_fifo_data <= 32'd0;
        O_drop      <= 1'b0;
        base_r      <= BASE_SECTOR;
        user        <= 1'b0;
        hdr         <= 1'b0;
        trk_d       <= 1'b0;
    end else begin
        O_fifo_we <= 1'b0;          // 默认单拍
        en_d      <= I_en;
        trk_d     <= I_trk_set;
        if (I_trk_set) begin
            base_r <= I_trk_sec;
            user   <= 1'b1;
        end

        //---------------------------------------------------------------------
        //  关闭：计数立刻归零（下次打开从第一个样本开始）
        //    如果正在收扇区，不能马上停 —— 要让仲裁器拿到 end。
        //---------------------------------------------------------------------
        if (en_fall) begin
            left      <= user ? 32'd1 : TOTAL_WORDS;   // 换曲：长度由头里的 data 块给
            skip      <= user ? 10'd0 : SKIP_BYTES;
            hdr       <= user;
            lane      <= 2'd0;
            shf       <= 24'd0;
            pass_done <= 1'b0;
            if (st == S_REQ)
                abort <= 1'b1;
            else begin
                sec <= base_r;
                st  <= S_GAP;
            end
        end

        case (st)
        //---------------------------------------------------------------------
        //  S_IDLE：等播放开关
        //---------------------------------------------------------------------
        S_IDLE: begin
            if (I_en && (left != 32'd0)) begin
                O_sd_addr <= sec;
                O_sd_req  <= 1'b1;
                st        <= S_REQ;
            end
            // ★ 2026-10-29：上一轮"不循环"播完后停在这里。如果用户此时把
            //   "循环播放"重新勾上，就从头再播一遍（否则要拨 SW2 才能重启）。
            else if (I_en && pass_done && I_loop) begin
                sec       <= base_r;
                skip      <= user ? 10'd0 : SKIP_BYTES;
                hdr       <= user;
                lane      <= 2'd0;
                shf       <= 24'd0;
                left      <= user ? 32'd1 : TOTAL_WORDS;   // 换曲：长度由头里的 data 块给
                pass_done <= 1'b0;
                st        <= S_GAP;
            end
        end

        //---------------------------------------------------------------------
        //  S_REQ：收一个扇区（512 字节），边收边拼字
        //---------------------------------------------------------------------
        S_REQ: begin
            // ---- 字节处理：先跳 header，再每 4 字节拼一个字 ----
            if (I_sd_dv) begin
                if (hdr) begin
                    // 换曲：逐字节找 "data"（shf 存前 3 个字节，正好拼成 4 字节比较）
                    shf <= {shf[15:0], I_sd_data};
                    if ({shf, I_sd_data} == 32'h6461_7461) begin
                        hdr  <= 1'b0;
                        skip <= 10'd4;              // 再跳 4 个长度字节，顺路把长度收进 left
                    end
                end else if (skip != 10'd0) begin
                    skip <= skip - 10'd1;           // 文件头 / chunk 头
                    shf  <= {shf[15:0], I_sd_data};
                    // 长度（小端 4 字节）的最后一个字节到了：left = 长度 / 4
                    if (user && (skip == 10'd1))
                        left <= {2'b00, I_sd_data, shf[7:0], shf[15:8], shf[23:18]};
                end else begin
                    shf  <= {shf[23:0], I_sd_data}; // 小端：先来的字节在高位
                    lane <= lane + 2'd1;
                    if (lane == 2'd3) begin
                        if (!pass_done && (left != 32'd0)) begin
                            if (I_en && !I_fifo_full) begin
                                O_fifo_we   <= 1'b1;
                                //--------------------------------------------------
                                //  【字节序：WAV 是小端，必须两两换位】
                                //  四条字节按到达顺序记为 b0 b1 b2 b3：
                                //      b0 = L 的低字节，b1 = L 的高字节
                                //      b2 = R 的低字节，b3 = R 的高字节
                                //  wav_player 要求 I_smp_data = {L16, R16}，
                                //  所以正确排布是 {b1,b0,b3,b2}：
                                //      L16 = b1<<8 | b0 ，R16 = b3<<8 | b2
                                //  shf 里此时是 {b0,b1,b2}（先来的在高位），
                                //  因此输出要按 8bit 为单位重排，不能直接拼接。
                                //  （曾经直接拼成 {b0,b1,b2,b3}，上板会是刺耳的噪声：
                                //    实测真卡数据 word1116 得到 0000_0100，应为 0000_0001）
                                //--------------------------------------------------
                                O_fifo_data <= {shf[15:8], shf[23:16], I_sd_data, shf[7:0]};
                            end else if (I_en) begin
                                // 理论上不会发生（发请求时已保证放得下一整扇区）
                                O_drop <= 1'b1;
                            end
                            if (left == 32'd1) pass_done <= 1'b1;
                            left <= left - 32'd1;
                        end
                    end
                end
            end

            // ---- 扇区结束 ----
            //  【关键：lane / shf 必须跨扇区保持】
            //  当 SKIP_BYTES 不是 4 的倍数时（例如 data 前面有 LIST chunk，
            //  实际偏移 = 78），(512 - SKIP_BYTES) 就不是 4 的整数倍，
            //  于是一个 32bit 样本会被扇区边界劈成两半：前 2 字节在上一扇区末尾，
            //  后 2 字节在下一扇区开头。
            //  所以这里**不能**把 lane 清零，否则每跨一个扇区就丢 2 个字节、
            //  从第 108 个字起全部错位（实测 5f606162 vs 应为 56575f60）。
            //  只有在"回到起点重开一轮"或"关播/中止"时才清零。
            if (I_sd_end) begin
                O_sd_req <= 1'b0;
                st       <= S_GAP;

                if (abort || !I_en) begin
                    // 关播：彻底归零（相位必须一起清，否则下次开头会错位）
                    sec       <= base_r;
                    skip      <= user ? 10'd0 : SKIP_BYTES;
                    hdr       <= user;
                    lane      <= 2'd0;
                    shf       <= 24'd0;
                    left      <= user ? 32'd1 : TOTAL_WORDS;   // 换曲：长度由头里的 data 块给
                    pass_done <= 1'b0;
                    abort     <= 1'b0;
                end else if (pass_done || (left == 32'd0)) begin
                    // 本轮播完 → 从头循环：回到同一个起始字节，相位重新从 0 开始
                    // ★ 2026-10-29：I_loop=0 时不重来，停在 S_IDLE。
                    //   保留 pass_done=1 / left=0 ⇒ S_GAP 的"继续请求"条件
                    //   (left!=0 && !pass_done) 恒假，不会再发请求。
                    if (I_loop) begin
                        sec       <= base_r;
                        skip      <= user ? 10'd0 : SKIP_BYTES;
                        hdr       <= user;
                        lane      <= 2'd0;
                        shf       <= 24'd0;
                        left      <= user ? 32'd1 : TOTAL_WORDS;   // 换曲：长度由头里的 data 块给
                        pass_done <= 1'b0;
                    end else begin
                        st        <= S_IDLE;
                    end
                end else begin
                    // 继续下一扇区：相位（lane/shf）原样带过去。
                    // ★ 2026-11-04：skip 不再清零 —— 换曲时 "data" 的长度字节可能跨扇区；
                    //   开机默认曲目的 SKIP_BYTES < 512，到这里本来就是 0。
                    sec  <= sec + 32'd1;
                end
            end
        end

        //---------------------------------------------------------------------
        //  S_GAP：扇区之间拉低 req 至少一拍，等 FIFO 腾出空间
        //---------------------------------------------------------------------
        S_GAP: begin
            if (I_en && !abort && (left != 32'd0) && !pass_done &&
                (I_fifo_level <= REQ_LEVEL)) begin
                O_sd_addr <= sec;
                O_sd_req  <= 1'b1;
                st        <= S_REQ;
            end
        end

        default: st <= S_IDLE;
        endcase
    end
end

// 本轮样本取完（left 归零后置起，直到下一次 I_en 上升沿/复位）
assign O_pass_done = pass_done;

//=============================================================================
//  ★ 2026-11-08【音乐结束→自动切图】“每遍播完”的单拍口径
//=============================================================================
//  【为什么必须分成两个信号】
//    · O_pass_end（原口径）= pass_done && FIFO 排空。loop=0 时它是对的：
//      最后一个样本真的放完了，trk_done 可以在这一刻置位。
//      但 loop=1 时 reader 在数据尾**立刻回卷并清 pass_done**（见 S_REQ 收尾），
//      FIFO 从来不会掉到 0 ⇒ 这个电平永远不会出现。旧版把它当“音乐结束”
//      的唯一来源 ⇒ 默认打开“循环播放”时自动切换永远不触发。
//    · O_pass_tick = 每遍数据结束的单拍（pass_done 的上升沿），
//      与 I_loop 无关。loop=1 时每回卷一次就发一次；loop=0 时改用排空后的
//      drain_rise，保证和 O_pass_end 同一时刻、且不会提前 20ms 把 trk_done 抢跑。
//  【为什么用打拍找边沿】pass_done / drain 都是电平，直接给上层会在整个
//    扇区尾保持高电平 ⇒ music_tgl 每个时钟翻转一次（旧注释里踩过这个坑）。
reg pass_done_d;
reg drain_d;
wire pass_rise  = pass_done & ~pass_done_d;
wire drain_lvl  = pass_done & (I_fifo_level == 0);
wire drain_rise = drain_lvl & ~drain_d;

always @(posedge I_clk or posedge I_rst) begin
    if (I_rst) begin
        pass_done_d <= 1'b0;
        drain_d     <= 1'b0;
    end else begin
        pass_done_d <= pass_done;
        drain_d     <= drain_lvl;
    end
end

assign O_pass_end  = ~I_loop & drain_rise;             // “真·播完”：只有单曲不循环才有
assign O_pass_tick =  I_loop ? pass_rise : drain_rise; // 每遍结束都有，且只一拍

endmodule
