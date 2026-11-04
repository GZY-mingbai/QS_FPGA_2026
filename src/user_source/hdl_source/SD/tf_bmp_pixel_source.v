//==============================================================================
//  tf_bmp_pixel_source.v —— 像素源：从 TF 卡读 BMP，一个像素一拍地吐出来
//==============================================================================
//
//  【在架构里的位置】
//
//      ┌──────────────────────┐        ┌──────────────────────┐
//      │ tf_bmp_pixel_source  │        │    frame_writer      │
//      │  （本模块，像素源）   │        │   （应用层，写帧）    │
//      │                      │        │                      │
//      │  扫描 TF 卡          │        │  和显存握手           │
//      │  解析 BMP            │        │  把像素写进 SDRAM     │
//      │  响应按键            │        │  管双缓冲 / 切显示    │
//      └──────────┬───────────┘        └──────────┬───────────┘
//                 │                               │
//                 │  ps_new_frame ──────────────► │   "我要画一帧"
//                 │  ◄────────────── ps_start     │   "开始给数据"
//                 │  ps_data / ps_valid ────────► │   一个像素一拍
//                 │                               │
//              SD_nCS/DCLK/MOSI/MISO              ▼
//                                            显存控制器
//
//  【职责边界（重要）】
//  本模块**只管产生像素**，不知道像素会被写到哪、什么时候上屏。
//  它也不碰 write_req / write_en / disp_buf_idx 这些显存信号 —— 那些全是
//  frame_writer 的事。
//
//  这样拆的好处：想换个像素来源（测试图案、摄像头、运算结果），
//  只要按同样的接口产像素，frame_writer 一行都不用改。
//
//  【内部有什么】
//    1. key_press_debounce × 2   —— 两个按键消抖
//    2. bmp_read                 —— 扫描 BMP + 解析 + 输出像素
//    3. sd_card_top              —— SD 卡 SPI 驱动
//    4. 一个状态机                —— 决定"什么时候画哪张图"
//
//  【状态机（4 个状态）】
//
//      P_IDLE ──触发──► P_WAIT ──ps_start──► P_FEED_START ──► P_FEED
//        ▲                                                      │
//        └──────────────── bmp_read 吐完了 ─────────────────────┘
//
//      P_IDLE       : 空闲。做两件事：上电扫描一次；根据按键/自动轮播决定下一帧
//      P_WAIT       : 已经告诉 frame_writer"我要画一帧"，等它说"开始给数据"
//      P_FEED_START : 刚启动 bmp_read，等它真的忙起来
//      P_FEED       : bmp_read 正在吐像素，等它吐完
//
//  【★ 为什么需要 P_FEED_START 这个状态 ★】
//  bmp_read 的 ready 信号表示"我空闲"。在 P_WAIT 里我们发 load_start 脉冲，
//  但 bmp_read 要下一拍才离开 IDLE，ready 要再下一拍才变低。
//  如果直接进 P_FEED 并判断 "ready == 1 就算吐完了"，会在刚启动的那一刻
//  立刻误判成"已经吐完"。所以必须先等 ready 掉下去（确认真的开工了），
//  再等它升上来（确认真的干完了）。
//
//  【★ 一个内部的小机关：bmp_read 的写握手 ★】
//  bmp_read 内部有一个 write_req / write_req_ack 握手（它以为自己在跟显存打交道）。
//  重构后写握手归 frame_writer 管了，所以本模块内部给它一个"自应答"：
//  收到它的 write_req 就打一拍回 ack，让它继续往下走。
//  时序上这是安全的，因为 frame_writer 的写握手早就做完了，FIFO 是干净的。
//
//==============================================================================

module tf_bmp_pixel_source #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,  // 本模块工作时钟频率
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,        // 扫描起点扇区
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,   // 扫描终点扇区
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4,         // 最多找几张图
    parameter [15:0]  BMP_WIDTH         = 16'd640,      // 期望图片宽度（判合法性用）
    parameter [15:0]  BMP_HEIGHT        = 16'd480       // 期望图片高度
)(
    input  wire        clk,             // 时钟，接 sd_card_clk（100 MHz）
    input  wire        rst,             // 复位，高电平有效（接 rst_all）

    input  wire        key_next,        // 按键1：手动下一张
    input  wire        key_auto,        // 按键2：自动轮播开关

    //--------------------------------------------------------------------------
    //  对 frame_writer 的接口
    //--------------------------------------------------------------------------
    input  wire        ps_start,        // frame_writer："开始给这一帧的数据"
                                        // 它做完写握手才会拉高，所以看到它就可以开工
    output reg         ps_new_frame,    // 本模块："我有新的一帧要画"（1 拍脉冲）
    output wire [23:0] ps_data,         // 像素数据 {R, G, B}
    output wire        ps_valid,        // 这个像素有效（与 ps_data 同拍）

    //--------------------------------------------------------------------------
    //  SD 卡物理接口（直通到顶层）
    //--------------------------------------------------------------------------
    output wire        SD_nCS,
    output wire        SD_DCLK,
    output wire        SD_MOSI,
    input  wire        SD_MISO,

    //--------------------------------------------------------------------------
    //  状态码输出（给数码管看，调试用）
    //--------------------------------------------------------------------------
    output wire [3:0]  state_code
);

//==============================================================================
//  状态编码
//==============================================================================
localparam P_IDLE       = 2'd0;   // 空闲：扫描 / 等触发
localparam P_WAIT       = 2'd1;   // 已发出 ps_new_frame，等 frame_writer 的 ps_start
localparam P_FEED_START = 2'd2;   // 刚启动 bmp_read，等它忙起来
localparam P_FEED       = 2'd3;   // bmp_read 正在吐像素

reg [1:0] state;

//==============================================================================
//  内部信号
//==============================================================================

// ---- 按键消抖后的单拍脉冲 ----
wire key_next_press;
wire key_auto_press;

// ---- 与 bmp_read 之间的接口 ----
wire             sd_sec_read;             // 请求读一个扇区
wire [31:0]      sd_sec_read_addr;        // 要读的扇区号
wire [7:0]       sd_sec_read_data;        // 读回的字节数据
wire             sd_sec_read_data_valid;  // 该字节有效
wire             sd_sec_read_end;         // 整个扇区读完（512 字节）
wire             bmp_data_wr_en;          // bmp_read 输出的"像素数据有效"
wire [23:0]      bmp_data;                // bmp_read 输出的 24bit 像素
wire             sd_init_done;            // SD 卡初始化完成
wire             bmp_ready;               // bmp_read 处于空闲、可以接收新命令
wire             scan_done;               // 扫描流程结束
wire             scan_found_valid;        // 发现一张合法 BMP（脉冲）
wire [31:0]      scan_found_sector;       // 该 BMP 的起始扇区
wire [2:0]       scan_found_total;        // 目前已找到的图片总数
wire             bmp_write_req;           // bmp_read 内部发出的写请求（本模块自应答）

// ---- 本模块自己维护的状态 ----
reg              scan_start_pulse;        // 启动扫描的脉冲
reg              load_start_pulse;        // 启动加载某张图的脉冲
reg  [31:0]      load_sector;             // 要加载的图，其起始扇区
reg              scan_kicked;             // 是否已经发起过扫描（防止重复发起）
reg              first_frame_committed;   // 第一张图是否已经画过
reg              auto_play_en;            // 自动轮播是否开启
reg  [31:0]      auto_cnt;                // 自动轮播计时器
reg  [2:0]       img_found_count;         // 扫到了几张图
reg  [1:0]       img_idx;                 // 当前"正在显示"的图片编号
reg  [1:0]       next_img_idx;            // 下一帧要画的图片编号
reg  [31:0]      img_sector0;             // 第 0 张图的扇区
reg  [31:0]      img_sector1;             // 第 1 张图的扇区
reg  [31:0]      img_sector2;             // 第 2 张图的扇区
reg  [31:0]      img_sector3;             // 第 3 张图的扇区
reg              next_req_pending;        // "用户按了下一张"的待处理标记
reg              bmp_ack;                 // 给 bmp_read 的写应答（自应答）

wire             auto_tick;               // 自动轮播计时到点（1 秒）
wire [1:0]       next_from_current;       // 当前图的"下一张"编号

//==============================================================================
//  组合逻辑
//==============================================================================

// 1 秒到点判断：计数器数到 CLK_FREQ_HZ - 1 就是正好 1 秒
// （100MHz 时钟数 1 亿次 = 1 秒）
assign auto_tick = (auto_cnt == (CLK_FREQ_HZ - 1));

// 计算"下一张"的编号（受已找到图片总数限制，不会越界）
assign next_from_current = next_index_limited(img_idx, img_found_count);

// 把 bmp_read 的输出直接接到对 frame_writer 的像素接口上。
// 【为什么是组合直连】bmp_read 的 bmp_data_wr_en 和 bmp_data 本来就是同拍有效的
// （内部用非阻塞赋值，两者一起在下一拍生效），所以直接透传即可，一拍都不用加。
assign ps_data  = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0]};
assign ps_valid = bmp_data_wr_en;

//==============================================================================
//  工具函数
//==============================================================================
// Verilog 的 function 可以理解成"可综合的纯组合逻辑子程序"，
// 只能有输入、一个输出（用函数名返回），不能有时序（不能有 posedge），
// 内部不能例化模块。用它可以避免写重复的 case 语句。

// ---- 计算"循环意义下的下一张图编号" ----
// 假设找到了 count 张图（编号 0 ~ count-1），求 cur 的下一张。
// 到末尾就绕回 0。这样自动轮播才能无限循环。
function [1:0] next_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;                    // 没有图
            3'd1: next_index_limited = 2'd0;                    // 只有 1 张，永远是自己
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
            // 为什么不用简单的 (cur+1) % count？
            // 因为对非 2 的幂做取模在硬件里要除法器，很浪费资源，
            // 而这种"比较 + 加一"的写法综合出来只是一堆比较器和加法器，便宜得多。
        endcase
    end
endfunction

// ---- 根据图片编号查出它存在 TF 卡的哪个扇区 ----
// 这是个"查找表"（Look-Up Table, LUT）：输入编号，输出扇区地址。
function [31:0] sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: sector_lut = img_sector0;
            2'd1: sector_lut = img_sector1;
            2'd2: sector_lut = img_sector2;
            2'd3: sector_lut = img_sector3;
            default: sector_lut = img_sector0;              // 兜底：返回第 0 张
        endcase
    end
endfunction

//==============================================================================
//  按键消抖例化
//==============================================================================
key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)                   // 20ms 内的抖动都忽略
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)        // 输出：按下一次 = 一个脉冲
);

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_auto (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_auto),
    .press_pulse(key_auto_press)
);

//==============================================================================
//  bmp_read 的写握手"自应答"
//==============================================================================
// bmp_read 内部以为自己在跟显存控制器握手：它拉高 write_req，等对方回 ack。
// 现在写握手归 frame_writer 管了，所以这里给它打一拍应答，让它继续往下走。
//
// 时序：bmp_read 拉高 write_req → 下一拍 bmp_ack 变高 → bmp_read 看到 ack，
//       撤销 write_req 并进入"吐数据"状态 → 再下一拍 bmp_ack 跟着落下去。
//
// 为什么这样是安全的：frame_writer 的写握手早就完成了（它做完握手才拉 ps_start，
// 本模块也是看到 ps_start 才启动 bmp_read 的），所以下游 FIFO 已经清干净、等着收数据。
always @(posedge clk or posedge rst) begin
    if (rst) bmp_ack <= 1'b0;
    else     bmp_ack <= bmp_write_req;   // 打一拍应答
end

//==============================================================================
//  主逻辑：决定"什么时候画哪张图"
//==============================================================================
// 注意本模块的主 always 块不是典型的"case 状态机"，
// 而是"一堆并行的 if 条件判断 + 一个三态 case"。
// 为什么这样写？因为这些事件（收到扫描结果 / 按键按下 / 自动计时到）
// 在时间上不互斥，可能同时发生，用状态机会把逻辑搞得很复杂。
//
// 【高电平异步复位】rst = 1 时复位，和顶层 rst_all 一致。
//==============================================================================
always @(posedge clk or posedge rst) begin
    if (rst) begin
        // ---------- 复位：所有状态清零 ----------
        state                 <= P_IDLE;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        first_frame_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 2'd0;
        next_img_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        next_req_pending      <= 1'b0;
        ps_new_frame          <= 1'b0;
    end else begin
        // ---------- 每个时钟都执行的动作 ----------

        // 这两个是"脉冲"，默认拉低、只在需要的那一拍拉高
        // 【为什么这么写】先默认赋 0，后面的逻辑里如果需要发脉冲就覆盖成 1。
        // 这样就不用在每个分支里都写一遍"不满足条件时清零"，代码更简洁，
        // 而且综合出来是一样的（这叫"默认赋值"技巧）。
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;
        ps_new_frame     <= 1'b0;

        // ---------- 分支 1：SD 卡还没初始化完，全部清空 ----------
        if (!sd_init_done) begin
            state                 <= P_IDLE;
            scan_kicked           <= 1'b0;
            first_frame_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            img_idx               <= 2'd0;
            next_img_idx          <= 2'd0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            next_req_pending      <= 1'b0;
        end else begin
            // ---------- 分支 2：SD 卡已就绪，正常干活 ----------

            // ---- 动作 A：扫描阶段，发现一张合法 BMP 就记下它的扇区 ----
            // scan_found_valid 是 bmp_read 输出的脉冲，每次发现一张图来一下。
            // img_found_count 从 0 开始，所以第 1 张存进 img_sector0，以此类推。
            if (scan_found_valid) begin
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector;
                    3'd1: img_sector1 <= scan_found_sector;
                    3'd2: img_sector2 <= scan_found_sector;
                    3'd3: img_sector3 <= scan_found_sector;
                    default: ;          // 已经存满 4 张，后面的忽略
                endcase

                if (img_found_count < 3'd4)
                    img_found_count <= img_found_count + 3'd1;
            end

            // ---- 动作 B：上电后自动发起一次扫描 ----
            // 条件：还没发起过扫描 且 bmp_read 空闲
            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;      // 发一个脉冲启动扫描
                scan_kicked           <= 1'b1;      // 标记已发起，不会再重复
                // 顺便把所有状态初始化一遍，保证从干净状态开始
                first_frame_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_idx               <= 2'd0;
                next_img_idx          <= 2'd0;
                next_req_pending      <= 1'b0;
            end

            // ---- 动作 C：按键与自动轮播（随时都在响应，和状态无关）----
            else begin

                // C-1. 记录"下一张"请求
                // 【为什么要用 next_req_pending 这个中间变量，而不是直接启动加载？】
                // 因为按键可能按在"系统正忙"的时候。先记下"用户按过一次"，
                // 等空闲了再执行。这样可以保证"忙的时候连按 5 次也只会跳 1 张"。
                if (key_next_press && scan_done && (img_found_count > 3'd0))
                    next_req_pending <= 1'b1;

                // C-2. 自动轮播开关（key2）
                // 只有扫描完成、且图不少于 2 张时才允许开启
                // （只有 1 张图的话轮播没意义，切换了还是同一张）
                if (key_auto_press && scan_done && (img_found_count > 3'd1)) begin
                    auto_play_en <= ~auto_play_en;      // 取反：开→关，关→开
                    auto_cnt     <= 32'd0;              // 重新计时
                end

                // C-3. 自动轮播计时器
                // 条件：扫描完成 + 自动播开启 + 首图已画过 + 当前空闲 + 图数 > 1
                if (scan_done && auto_play_en && first_frame_committed &&
                    (state == P_IDLE) && (img_found_count > 3'd1)) begin
                    if (auto_tick)
                        auto_cnt <= 32'd0;              // 到 1 秒，清零重新数
                    else
                        auto_cnt <= auto_cnt + 32'd1;   // 继续数
                end else begin
                    auto_cnt <= 32'd0;                  // 条件不满足就一直清零（不计时）
                end
                // 【为什么条件里要有 state == P_IDLE】正在画一帧的时候计时器不该跑，
                // 否则可能在画的中间"到点又启动下一次"，逻辑会打架。
            end

            // ---- 动作 D：状态机 ----
            case (state)

            //==================================================================
            // P_IDLE：空闲。决定"要不要开始画下一帧"
            //==================================================================
            // 三条触发路径，优先级从高到低：
            //   ① 首图自动画（用户还在等第一张图出来，最紧急）
            //   ② 用户按了 key1
            //   ③ 自动轮播 1 秒到点
            // 三条路径干的事一样：记下"要画第几张"，发 ps_new_frame，转 P_WAIT。
            //==================================================================
            P_IDLE: begin
                if (scan_done && !first_frame_committed && bmp_ready &&
                    (img_found_count != 3'd0)) begin
                    // ① 首图
                    next_img_idx <= 2'd0;
                    ps_new_frame <= 1'b1;
                    state        <= P_WAIT;
                end
                else if (scan_done && bmp_ready && next_req_pending &&
                         (img_found_count != 3'd0)) begin
                    // ② 手动下一张
                    next_img_idx     <= next_from_current;
                    next_req_pending <= 1'b0;           // 用掉了这个请求
                    ps_new_frame     <= 1'b1;
                    state            <= P_WAIT;
                end
                else if (scan_done && bmp_ready && auto_play_en && auto_tick &&
                         (img_found_count > 3'd1)) begin
                    // ③ 自动轮播
                    next_img_idx <= next_from_current;
                    ps_new_frame <= 1'b1;
                    state        <= P_WAIT;
                end
                // 都不满足就待在 P_IDLE
            end

            //==================================================================
            // P_WAIT：已经告诉 frame_writer 了，等它说"开始给数据"
            //==================================================================
            // frame_writer 会先做写握手（清 FIFO、锁地址），做完才拉 ps_start。
            // 看到 ps_start 就启动 bmp_read，让它开始从 TF 卡读图。
            //==================================================================
            P_WAIT: begin
                if (ps_start) begin
                    load_sector      <= sector_lut(next_img_idx);  // 查表得扇区地址
                    load_start_pulse <= 1'b1;                      // 启动加载
                    state            <= P_FEED_START;
                end
            end

            //==================================================================
            // P_FEED_START：刚启动 bmp_read，等它真的忙起来
            //==================================================================
            // 为什么要这一步？见文件头"为什么需要 P_FEED_START"。
            // 简单说：bmp_ready 现在还是 1（bmp_read 还没看到 load_start），
            // 如果直接去 P_FEED 判断"ready==1 就算完事"，会立刻误判。
            //==================================================================
            P_FEED_START: begin
                if (!bmp_ready)                     // bmp_read 离开空闲了 = 真的开工了
                    state <= P_FEED;
            end

            //==================================================================
            // P_FEED：bmp_read 正在吐像素，等它吐完
            //==================================================================
            // 数据是通过 ps_data / ps_valid 组合直连出去的，这里不用做任何搬运。
            // bmp_ready 重新变高 = bmp_read 回到 IDLE = 这一帧的像素全吐完了。
            //==================================================================
            P_FEED: begin
                if (bmp_ready) begin
                    first_frame_committed <= 1'b1;  // 首图已经画过了
                    img_idx               <= next_img_idx;  // 记下"现在显示的是第几张"
                    state                 <= P_IDLE;
                end
            end

            default: state <= P_IDLE;
            endcase
        end
    end
end

//==============================================================================
//  例化 BMP 解析与加载模块
//==============================================================================
bmp_read bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),            // 输出：空闲可用

    // ---- 扫描命令接口 ----
    .scan_start             (scan_start_pulse),     // 启动扫描
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),            // 输出：扫描结束
    .scan_found_valid       (scan_found_valid),     // 输出：发现一张（脉冲）
    .scan_found_sector      (scan_found_sector),    // 输出：那张图的扇区
    .scan_found_total       (scan_found_total),     // 输出：累计找到几张

    // ---- 加载命令接口 ----
    .load_start             (load_start_pulse),     // 启动加载
    .load_sector            (load_sector),          // 要加载哪张

    // ---- 状态与配置 ----
    .sd_init_done           (sd_init_done),
    .state_code             (state_code),           // 状态码 → 数码管
    .bmp_width              (BMP_WIDTH),
    .bmp_height             (BMP_HEIGHT),

    // ---- 写握手（本模块内部自应答，见上面"自应答"一节）----
    .write_req              (bmp_write_req),
    .write_req_ack          (bmp_ack),

    // ---- SD 卡扇区读接口 ----
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),

    // ---- 像素数据输出 ----
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data)
);

//==============================================================================
//  例化 SD 卡驱动层
//==============================================================================
// 注意这里的 .sd_sec_write 接的是常数 1'b0 —— 本工程只需要"读"，
// 不需要"写"SD 卡，所以把写接口全部禁用，避免误写卡里的数据。
sd_card_top sd_card_top_m0(
    .clk                    (clk),
    .rst                    (rst),
    .SD_nCS                 (SD_nCS),
    .SD_DCLK                (SD_DCLK),
    .SD_MOSI                (SD_MOSI),
    .SD_MISO                (SD_MISO),
    .sd_init_done           (sd_init_done),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .sd_sec_write           (1'b0),                 // 禁用写功能
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (8'd0),                 // ★ 这是【输入】不是输出！
                                                    //   原 sd_card_bmp.v 里写的是
                                                    //   ".sd_sec_write_data ()" 悬空，
                                                    //   仿真时会是未知值 x。
                                                    //   因为 sd_sec_write 恒为 0，
                                                    //   功能上没暴露问题，但属于隐患。
    .sd_sec_write_data_req  (),                     // 未使用的【输出】可以留空
    .sd_sec_write_end       ()
);

endmodule
