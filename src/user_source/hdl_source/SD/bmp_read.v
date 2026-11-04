
//==============================================================================
//  bmp_read.v  —— BMP 文件解析 + 多图扫描状态机
//==============================================================================
//
//  【这个文件干什么的】
//  它有两个工作模式，由两个"启动脉冲"分别触发：
//
//  模式一：扫描（scan）
//    从 scan_start_sector 开始，一个扇区一个扇区地往后看，
//    每读到一个扇区，就检查它的开头 54 字节是不是"合法的 BMP 文件头"。
//    如果是，就记下这个扇区号（通知上层"我找到一张图"），
//    然后跳过整个文件（按文件长度算），继续往后找下一张。
//    找到 scan_target_count 张、或者扫到 scan_max_sector 还没找够，就结束。
//
//  模式二：加载（load）
//    上层说"把第 N 个扇区那张图读出来"，本模块就去读那个文件：
//    先读文件头验证一下，然后逐扇区把像素数据读出来，
//    剥掉文件头部分、把 3 个字节（B、G、R）拼成一个 24bit 像素输出。
//
//  【BMP 文件格式速览（本模块要解析的部分）】
//
//    偏移量   长度   含义
//    ------   ----   ------------------------------------------
//    0x00      2    文件标识，必须是 'B' 'M' 两个字母
//    0x02      4    文件总长度（字节），小端序
//    0x0A      4    像素数据起始偏移（通常是 54）
//    0x12      4    图像宽度（像素），小端序
//    0x16      4    图像高度（像素），小端序
//    0x1C      2    每像素位数，24 表示 RGB888
//    0x1E      4    压缩方式，0 表示不压缩
//    ------   ----   ------------------------------------------
//    54 字节之后就是像素数据，按 B、G、R 顺序，每像素 3 字节
//
//  【什么是"小端序"（Little Endian）】
//  一个 4 字节的数，在文件里是"低位字节在前"存放的。
//  比如文件长度 0x00012345，文件里存的顺序是：45 23 01 00。
//  所以代码里要写成 file_len[7:0] 先收、file_len[31:24] 最后收。
//  看下面第 150 行附近的 case 语句，就是按偏移量逐个字节还原的。
//
//  【状态机的五个状态】
//    ST_IDLE       空闲，等命令
//    ST_SCAN       扫描模式：读一个扇区、判断、决定下一个读哪里
//    ST_LOAD_HDR   加载模式第一步：先读文件头做校验
//    ST_LOAD_WAIT  等帧缓存模块应答写请求
//    ST_LOAD_DATA  逐扇区读像素数据
//
//  【注意】本模块的高电平复位 rst 与 sd_init_done 的关系：
//  只要 SD 卡没初始化完（sd_init_done = 0），就强制回到 ST_IDLE 并清空所有输出。
//  这样即使卡被拔掉再插上，也能从干净状态重新开始。
//
//==============================================================================

module bmp_read(
    input                       clk,            // 时钟，100MHz
    input                       rst,            // 复位，高有效
    output                      ready,          // 输出：本模块空闲、可以接受新命令

    // ---------------- 扫描模式接口 ----------------
    input                       scan_start,          // 启动扫描（脉冲）
    input  [31:0]               scan_start_sector,   // 从哪个扇区开始扫
    input  [31:0]               scan_max_sector,     // 扫到哪个扇区就停
    input  [2:0]                scan_target_count,   // 想找几张
    output reg                  scan_done,           // 输出：扫描结束
    output reg                  scan_found_valid,    // 输出：发现一张合法图（脉冲）
    output reg [31:0]           scan_found_sector,   // 输出：那张图的起始扇区
    output reg [2:0]            scan_found_total,    // 输出：累计找到几张

    // ---------------- 加载模式接口 ----------------
    input                       load_start,     // 启动加载（脉冲）
    input  [31:0]               load_sector,    // 要加载的图，其起始扇区

    // ---------------- SD 卡状态与配置 ----------------
    input                       sd_init_done,   // SD 卡初始化完成
    output reg [3:0]            state_code,     // 状态码（给数码管）
    input  [15:0]               bmp_width,      // 期望宽度
    input  [15:0]               bmp_height,     // 期望高度

    // ---------------- 与帧缓存的写握手 ----------------
    output reg                  write_req,      // 写请求（保持高直到 ack）
    input                       write_req_ack,  // 写请求应答

    //--------------------------------------------------------------------------
    //  ★★★ build 100M-12：扇区间流控信用 ★★★
    //--------------------------------------------------------------------------
    //  【为什么需要】原来的 `write_req/ack` 只在装载**开始时**握手一次，
    //  进了 ST_LOAD_DATA 之后就是"一个扇区接一个扇区地拼命读"，**没有流控**。
    //  下游 FIFO 一满就静默丢字（`we_q <= we && !wfull`），而装载器的
    //  行/列计数照走 ⇒ 纹理地址永久留洞。板上实测每张图丢一半以上。
    //
    //  【怎么用】`sd_sec_read` 是电平请求：拉高读一个扇区、读完拉低。
    //  所以在**两个扇区之间**等 `wr_credit`=1 再拉高，就能把平均产速
    //  精确压到下游排水速率以下。扇区 = 512B ≈ 170 像素，是天然的节流单位。
    //--------------------------------------------------------------------------
    input                       wr_credit,      // 1 = 下游还有空间，可以再读一个扇区

    // ---------------- SD 卡扇区读接口 ----------------
    output reg                  sd_sec_read,          // 请求读一个扇区
    output reg [31:0]           sd_sec_read_addr,     // 要读的扇区号
    input  [7:0]                sd_sec_read_data,     // 读回的字节
    input                       sd_sec_read_data_valid,// 字节有效
    input                       sd_sec_read_end,      // 扇区读完（512 字节完）

    // ---------------- 像素输出 ----------------
    output reg                  bmp_data_wr_en,  // 像素有效（拼满 3 字节后拉高一次）
    output reg [23:0]           bmp_data,        // 24bit 像素，顺序是 {R,G,B}
    // ★ build 100M-15：头部解析出的“本文件实际能提供多少像素”
    //   【为什么给下游】装载器的 len_pix 原来用 load_length/3 粗估（54 字节
    //   文件头也计成了像素），截断/长度不一致的文件会让上限大于真实像素数
    //   ⇒ bmp_read 提前读完后装载器永远等不到 len_pix（L_FEED 死锁）。
    //   文件头里的 file_len / pixel_offset 才是权威值，这里除 3 交出去。
    //   header_match 已强制 width==640 ⇒ 640×3=1920 是 4 的倍数、无行填充，
    //   所以这个 /3 对合法文件是精确的。
    // ★ 2026-10-27：给装载器/信息条用的源分辨率（头部解析完即有效）
    output wire [15:0]          src_w_o,
    output wire [15:0]          src_h_o
);

//==============================================================================
//  状态编码
//==============================================================================
// 【为什么用 localparam 给状态编码起名字而不是直接用 0/1/2/3？】
// 可读性。看到 ST_LOAD_DATA 就知道在干什么，看到 3 就得回去翻注释。
// 综合工具会把名字转成二进制编码，不占任何额外资源。
localparam ST_IDLE      = 3'd0;     // 空闲
localparam ST_SCAN      = 3'd1;     // 扫描中
localparam ST_LOAD_HDR  = 3'd2;     // 加载：读文件头校验
localparam ST_LOAD_WAIT = 3'd3;     // 加载：等写握手
localparam ST_LOAD_DATA = 3'd4;     // 加载：搬运像素

//==============================================================================
//  内部寄存器
//==============================================================================
reg [2:0]  state;       // 当前状态
reg [9:0]  rd_cnt;      // 当前扇区内已读的字节序号（0 ~ 511）

// ---- 文件头解析出来的字段 ----
reg [7:0]  header_0;    // 第 0 字节，应该是 'B'
reg [7:0]  header_1;    // 第 1 字节，应该是 'M'
reg [31:0] file_len;    // 文件总长度（偏移 0x02）
reg [31:0] pixel_offset;// 像素数据偏移（偏移 0x0A），通常 54
reg [31:0] width;       // 图像宽度（偏移 0x12）
reg [31:0] height;      // 图像高度（偏移 0x16）
reg [15:0] bit_count;   // 每像素位数（偏移 0x1C）
reg [31:0] compression; // 压缩方式（偏移 0x1E）

// ---- 扫描/加载过程变量 ----
reg [31:0] scan_sector;         // 当前扫描到的扇区号
reg [31:0] load_sector_latched; // 锁存的待加载扇区
reg [31:0] bmp_len_cnt;         // 加载时已处理的字节数
reg [1:0]  bmp_byte_idx;        // 像素内字节计数 0/1/2（B、G、R）

// ---- 组合逻辑辅助信号 ----
wire header_match;              // 文件头校验是否通过
wire bmp_data_valid;            // 当前字节是否属于像素数据区（该输出）
wire [31:0] file_sector_count;  // 这个文件一共占几个扇区
wire [31:0] next_scan_sector_if_match;  // 命中后应该跳到哪个扇区
wire [31:0] next_scan_sector_if_miss;   // 没命中，下一个扇区

//==============================================================================
//  组合逻辑
//==============================================================================

// ready = 只有在 ST_IDLE 时才算空闲，可以接受新的 scan_start / load_start
assign ready = (state == ST_IDLE);

// ============================================================================
//  ★★★ 合法性判据 ★★★
// ============================================================================
// 这行表达式是"什么样的文件才算一张能用图"的完整定义。六个条件全部满足才算合法：
//   1. header_0 == "B"     文件标识第一个字节
//   2. header_1 == "M"     文件标识第二个字节
//   3. width  == bmp_width   宽度必须正好是 640
//   4. height == bmp_height  高度必须正好是 480
//   5. bit_count == 24       每像素 24 位（RGB888）
//   6. compression == 0      不压缩
// 【注意】"B" 和 "M" 在这里是字符字面量，Verilog 会自动转成 ASCII 码
//        （'B' = 0x42，'M' = 0x4D）。所以要写 8 位比较。
// 【为什么不做更宽松的判断】因为 FPGA 端的帧缓存、HDMI 时序都按死 640×480 设计的，
//        一张 800×600 的图读进来会错位、花屏，不如干脆判为非法跳过。
// ★★ 2026-10-27：不再要求 640×480 ★★
//   画布缩放交给 bmp_scale（装载阶段），所以这里只做"能不能安全解析"的检查：
//   宽高非零、≤4096（行缓冲/计数器都按 12 位设计）、24bit、不压缩。
//   top-down（height<0）判非法：它要额外处理行序，本期不做。
assign header_match = (header_0 == "B") &&
                      (header_1 == "M") &&
                      (width  != 32'd0) && (width  <= 32'd4096) &&
                      (height != 32'd0) && (height <= 32'd4096) &&
                      (bit_count    == 16'd24) &&
                      (compression  == 32'd0);

// ★★ 2026-10-27：BMP 每行按 4 字节对齐，宽×3 不是 4 的倍数时行尾有 0~3 字节填充。
//   stride = (W*3 + 3) & ~3；只有 row_byte < W*3 的字节才是像素。
wire [15:0] stride   = ((width[15:0] << 1) + width[15:0] + 16'd3) & 16'hFFFC;
wire [15:0] row_pix3 = (width[15:0] << 1) + width[15:0];

reg [15:0] row_byte;        // 当前行内已读字节（0..stride-1）
always @(posedge clk or posedge rst) begin
    if (rst)
        row_byte <= 16'd0;
    else if ((state == ST_LOAD_DATA) && sd_sec_read_data_valid) begin
        if (bmp_len_cnt == pixel_offset)      row_byte <= 16'd1;   // 第一个像素字节
        else if (row_byte >= (stride - 16'd1)) row_byte <= 16'd0;  // 一行读完
        else                                   row_byte <= row_byte + 16'd1;
    end
end

// 当前读到的字节，是否位于"像素数据区"？
// 条件是：字节有效 且 已超过文件头 且 还没超文件末尾 且 **不在行尾填充里**
assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1) &&
                        (bmp_len_cnt >= pixel_offset) &&
                        (bmp_len_cnt <  file_len) &&
                        (row_byte < row_pix3);

// 本文件实际能提供的像素数 = (file_len - pixel_offset) / 3
// （只在 ST_LOAD_DATA 之后有效——那时文件头已经解析完；
//   装载器在收到第 1 个像素时采样它，时序上一定晚于头部解析）
// ★ 2026-10-27：pix_avail 已删（装载器不再需要；32 位 ÷3 在 DSP 满时会落成大乘法器）
assign src_w_o = width[15:0];
assign src_h_o = height[15:0];

// 这个文件占多少个扇区 = 向上取整(文件长度 / 512)
// (x + 511) >> 9 就是 ceil(x/512) 的经典写法（>>9 等于除以 512）
// 【为什么要向上取整】文件长度一般不是 512 的整数倍，最后那个扇区还是要读的。
assign file_sector_count = (file_len == 32'd0) ? 32'd1 : ((file_len + 32'd511) >> 9);

// 命中一张图后，下一个要检查的扇区 = 当前扇区 + 整个文件占用的扇区数
// （跳过这个文件，直接看它后面的内容）
assign next_scan_sector_if_match = scan_sector + file_sector_count;

// 没命中，就往后挪一个扇区，继续找
assign next_scan_sector_if_miss  = scan_sector + 32'd1;

//==============================================================================
//  字节计数器 rd_cnt
//==============================================================================
// 用来记录"当前这个扇区里已经读到第几个字节"，从而按偏移量解析文件头。
// 每个扇区 512 字节，所以需要 10 位（2^9=512 其实 9 位就够，但 10 位更保险）。
//
// 【为什么每读一个字节就 +1，而不等扇区读完？】
// 因为要在读到第 0 个字节的当拍就把它存进 header_0，
// 所以计数器必须跟数据同步走。
always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR)) begin
        // 两个状态都需要解析文件头，所以共用这个计数器
        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;       // 来一个有效字节就加一
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;                // 一个扇区读完，计数器归零准备下一个
    end else begin
        rd_cnt <= 10'd0;                    // 其他状态保持归零
    end
end

//==============================================================================
//  文件头字段解析
//==============================================================================
// 按偏移量把每个字节"拼装"到对应的字段里。
// 【小端序】低字节先到，所以 file_len[7:0] 对应偏移 2，[15:8] 对应偏移 3，以此类推。
// 【为什么是这些数字】看文件开头的格式表：
//   偏移 0,1   → header_0, header_1
//   偏移 2~5   → file_len
//   偏移 10~13 → pixel_offset
//   偏移 18~21 → width
//   偏移 22~25 → height
//   偏移 28~29 → bit_count
//   偏移 30~33 → compression
always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        pixel_offset <= 32'd54;         // 默认值 54（标准 BMP 文件头长度）
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
    end else if (((state == ST_SCAN) || (state == ST_LOAD_HDR)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            // ---- 文件标识 'B' 'M' ----
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;

            // ---- 文件总长度（4 字节小端）----
            10'd2 : file_len[7:0]   <= sd_sec_read_data;
            10'd3 : file_len[15:8]  <= sd_sec_read_data;
            10'd4 : file_len[23:16] <= sd_sec_read_data;
            10'd5 : file_len[31:24] <= sd_sec_read_data;

            // ---- 像素数据偏移（4 字节小端）----
            10'd10: pixel_offset[7:0]   <= sd_sec_read_data;
            10'd11: pixel_offset[15:8]  <= sd_sec_read_data;
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;

            // ---- 图像宽度（4 字节小端）----
            10'd18: width[7:0]   <= sd_sec_read_data;
            10'd19: width[15:8]  <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;

            // ---- 图像高度（4 字节小端）----
            10'd22: height[7:0]   <= sd_sec_read_data;
            10'd23: height[15:8]  <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;

            // ---- 每像素位数（2 字节小端）----
            10'd28: bit_count[7:0]  <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;

            // ---- 压缩方式（4 字节小端）----
            10'd30: compression[7:0]   <= sd_sec_read_data;
            10'd31: compression[15:8]  <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;

            default: ;      // 其他偏移的字节不关心
        endcase
    end
end

//==============================================================================
//  加载模式：字节计数器 bmp_len_cnt
//==============================================================================
// 记录加载过程中已经"看过"多少字节，用来判断：
//   ① 什么时候进入像素区（>= pixel_offset）
//   ② 什么时候文件读完（>= file_len）
always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else begin
        bmp_len_cnt <= 32'd0;       // 不在加载状态就归零
    end
end

//==============================================================================
//  像素内字节计数 bmp_byte_idx
//==============================================================================
// 一个像素 = 3 个字节 = B, G, R。
// 这个计数器在 0→1→2→0 之间循环，指示"当前字节是像素的第几个分量"。
always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_byte_idx <= 2'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid)
            // 0→1→2→回0 循环
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end else begin
        bmp_byte_idx <= 2'd0;
    end
end

//==============================================================================
//  像素拼装（把 3 个字节合成 24bit）
//==============================================================================
// 【BMP 里像素的字节顺序是 B、G、R】，但 HDMI 需要的是 {R, G, B} 顺序，
// 所以拼装的时候要注意"倒过来放"：
//   第 0 个字节（B）→ 存到 bmp_data[23:16]（R 的位置）...
//
// 【等等，这里有个细节】看代码发现第 0 个字节存到了 bmp_data[7:0]，
// 第 2 个字节存到了 bmp_data[23:16]。这看起来和上面说的相反，为什么？
// 因为这里的"第 0 个字节在文件里的位置"其实对应的是 R 分量——
// 这个工程在生成 BMP 的 Python 脚本里做了一次特殊处理
// （或者依赖于 BMP 的 BGR 存储顺序 + 后续模块的连接方式），
// 最终让 bmp_read 输出的 bmp_data = {R, G, B}。
// 【实践建议】如果你换了一批新图片发现颜色不对（红蓝颠倒），
// 就来改这个 case 里的赋值顺序，把 [7:0] 和 [23:16] 对调即可。
//
// 【为什么要 wr_en 在第 2 个字节时才拉高】
// 因为要等 3 个字节都收齐了，这个 24bit 数据才完整。
// 前两个字节只更新数据寄存器，不拉使能；第三个字节到齐后拉高使能一拍，
// 下游看到这一拍就知道"bmp_data 有效了，可以取走"。
always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid) begin
            case (bmp_byte_idx)
                2'd0: begin
                    bmp_data_wr_en <= 1'b0;         // 第 1 个字节，先不发使能
                    bmp_data[7:0]  <= sd_sec_read_data;
                end
                2'd1: begin
                    bmp_data_wr_en <= 1'b0;         // 第 2 个字节，还不发
                    bmp_data[15:8] <= sd_sec_read_data;
                end
                2'd2: begin
                    bmp_data_wr_en  <= 1'b1;        // 第 3 个字节到了，拉高使能！
                    bmp_data[23:16] <= sd_sec_read_data;
                end
                default: begin
                    bmp_data_wr_en <= 1'b0;
                end
            endcase
        end else begin
            // 不在像素区（还在文件头里）→ 不给使能
            bmp_data_wr_en <= 1'b0;
        end
    end else begin
        bmp_data_wr_en <= 1'b0;
    end
end

//==============================================================================
//  ★ 主状态机 ★
//==============================================================================
always @(posedge clk or posedge rst) begin
    if (rst) begin
        // ---------- 复位 ----------
        state               <= ST_IDLE;
        state_code          <= 4'd0;
        sd_sec_read         <= 1'b0;
        sd_sec_read_addr    <= 32'd0;
        write_req           <= 1'b0;
        scan_done           <= 1'b0;
        scan_found_valid    <= 1'b0;
        scan_found_sector   <= 32'd0;
        scan_found_total    <= 3'd0;
        scan_sector         <= 32'd0;
        load_sector_latched <= 32'd0;
    end else if (!sd_init_done) begin
        // ---------- SD 卡没初始化好 → 强制空闲 ----------
        state               <= ST_IDLE;
        state_code          <= 4'd0;        // 状态码 0 = 未初始化
        sd_sec_read         <= 1'b0;
        sd_sec_read_addr    <= 32'd0;
        write_req           <= 1'b0;
        scan_done           <= 1'b0;
        scan_found_valid    <= 1'b0;
        scan_found_sector   <= 32'd0;
        scan_found_total    <= 3'd0;
        scan_sector         <= 32'd0;
        load_sector_latched <= 32'd0;
    end else begin
        // scan_found_valid 是个"脉冲"，默认拉低，只在发现图片的那一拍拉高
        scan_found_valid <= 1'b0;

        case (state)

            // ================================================================
            //  ST_IDLE —— 空闲，等命令
            // ================================================================
            ST_IDLE: begin
                state_code  <= 4'd1;        // 状态码 1 = 空闲
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;

                if (scan_start) begin
                    // 收到扫描命令：初始化扫描状态
                    scan_done        <= 1'b0;
                    scan_found_total <= 3'd0;                       // 找到的图片数清零
                    scan_sector      <= scan_start_sector;          // 从起点开始
                    sd_sec_read_addr <= scan_start_sector;
                    state            <= ST_SCAN;
                end else if (load_start) begin
                    // 收到加载命令：锁存目标扇区，进入"先读文件头校验"状态
                    load_sector_latched <= load_sector;             // 先锁存，防止中途变化
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

            // ================================================================
            //  ST_SCAN —— 扫描模式：读扇区 → 判断 → 决定下一个
            // ================================================================
            ST_SCAN: begin
                state_code  <= 4'd2;        // 状态码 2 = 正在读扇区
                sd_sec_read <= 1'b1;        // 请求读当前扇区

                // 一个扇区（512 字节）读完，文件头也已经解析完毕，可以判断了
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;

                    if (header_match) begin
                        // ---- 命中了！这是一张合法的图 ----
                        scan_found_valid  <= 1'b1;                          // 发脉冲通知上层
                        scan_found_sector <= scan_sector;                   // 报告扇区号
                        scan_found_total  <= scan_found_total + 3'd1;       // 计数加一

                        // 判断是否该结束扫描：找够了 或 下一个扇区已经超出范围
                        if ((scan_found_total + 3'd1 >= scan_target_count) ||
                            (next_scan_sector_if_match > scan_max_sector)) begin
                            // 结束：回到空闲，把读地址预置到"跳过这张图之后"的位置
                            scan_done        <= 1'b1;
                            state            <= ST_IDLE;
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end else begin
                            // 还没找够：跳过整个文件，继续往后找
                            sd_sec_read_addr <= next_scan_sector_if_match;
                            scan_sector      <= next_scan_sector_if_match;
                        end
                    end else begin
                        // ---- 没命中，这不是合法 BMP（或者根本不是 BMP）----
                        if (scan_sector >= scan_max_sector) begin
                            // 扫到头了，结束
                            scan_done <= 1'b1;
                            state     <= ST_IDLE;
                        end else begin
                            // 往后挪一个扇区继续找
                            sd_sec_read_addr <= next_scan_sector_if_miss;
                            scan_sector      <= next_scan_sector_if_miss;
                        end
                    end
                end
            end

            // ================================================================
            //  ST_LOAD_HDR —— 加载第一步：读文件头做最后校验
            // ================================================================
            // 【为什么要在这里再校验一次】
            // 因为扫描时的信息可能已经过期（用户中途换了卡），
            // 而且加载时要用的 pixel_offset / file_len 需要重新解析出来。
            ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;

                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (header_match) begin
                        // 校验通过：发起写请求，让帧缓存模块准备接收数据
                        write_req        <= 1'b1;
                        sd_sec_read_addr <= load_sector_latched;    // 从文件开头重新读
                        state            <= ST_LOAD_WAIT;
                    end else begin
                        // 校验失败：直接放弃，回空闲
                        // 【这就是"SD 卡读取防死锁机制"的一部分】
                        // 如果图片损坏/截断/分辨率不对，这里会直接退出，
                        // 不会卡在加载状态里出不来。
                        state <= ST_IDLE;
                    end
                end
            end

            // ================================================================
            //  ST_LOAD_WAIT —— 等帧缓存应答写请求
            // ================================================================
            // 这是一个"握手"过程：本模块把 write_req 拉高，
            // 下游（frame_read_write）看到后把 write_req_ack 拉高，表示"我准备好了"。
            // 【为什么要握手】因为下游需要时间清空 FIFO、锁存写地址，
            // 不是马上就能接收数据的。
            ST_LOAD_WAIT: begin
                state_code <= 4'd3;         // 状态码 3 = 等写握手
                if (write_req_ack) begin
                    write_req <= 1'b0;      // 收到应答，撤销请求
                    state     <= ST_LOAD_DATA;
                end
            end

            // ================================================================
            //  ST_LOAD_DATA —— 逐扇区搬运像素数据
            // ================================================================
            ST_LOAD_DATA: begin
                state_code  <= 4'd4;        // 状态码 4 = 搬运数据中

                // ★ build 100M-12：扇区间流控
                //  原来这里是无条件 `sd_sec_read <= 1'b1`（一个扇区接一个扇区地猛读），
                //  下游一满就静默丢字。现在改成"读完一个扇区先拉低，等下游信用"：
                //      · 读完一个扇区（sd_sec_read_end）→ 拉低、地址 +1
                //      · 还没读完                             → 保持高，继续收字节
                //      · 已经拉低且 wr_credit=1                → 拉高，读下一个扇区
                //  信用=1 的那一拍行为与原来完全一致（下一拍就重新拉高），
                //  只有信用=0 时才多等几拍 ⇒ 不会破坏原来的时序假设。
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;

                    if (bmp_len_cnt >= file_len) begin
                        // 整个文件读完了 → 结束
                        state <= ST_IDLE;
                    end else begin
                        // 还没读完 → 继续读下一个扇区
                        // 【注意】这里只 +1，是因为"跳过文件头"不需要跳扇区，
                        // 文件头在第一个扇区里，bmp_len_cnt 的计数已经自动
                        // 帮我们过滤掉了（见 bmp_data_valid 的判断）。
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end else if (!sd_sec_read && wr_credit) begin
                    sd_sec_read <= 1'b1;        // 下游有空间 → 读下一个扇区
                end
            end

            // ================================================================
            //  兜底：不该出现的状态 → 回空闲
            // ================================================================
            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule
