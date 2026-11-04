//==============================================================================
//  osd_font_cache.v —— 字形存储控制器（外部 Flash → 片上 RAM）
//==============================================================================
//
//  【它解决什么问题】
//  字库在外部 W25Q64 里（font_gb2312.bin @ 0x000000），每个字形 32 字节。
//  一次 SPI 读一个字约 21µs，而一行 640 像素只要 6.4µs —— **边显示边读 Flash
//  根本不可能**。所以必须把字形搬到片上 RAM，渲染时只读片上 RAM。
//
//  【★★ 2026-10-29 [S4]：从"缓存"改成"每格一份" ★★】
//  旧实现是一套真正的缓存：tag 表去重 + owner 位掩码回收 + 63 步顺序扫描 +
//  空槽扫描 + 两阶段释放。为了在不淘汰的前提下复用字形，它必须知道
//  "这个槽还被哪几行引用"。**这套机制实测占 1237 LUT**（把状态机冻住后
//  1327→90，见 P1 探针），而像素侧只值 90 LUT。
//
//  新实现换了一个更简单的坐标：**屏幕上每个字格固定拥有自己的一份字形**。
//      slot = {行号, 列号}          （8 行 × 32 列 = 256 格）
//  于是：
//    · 不需要分配/回收/扫描/去重 —— 一个状态都不用，索引是纯连线；
//    · 槽位表整块消失（旧版 slot_fs + 8 份 slot_px ≈ 5 Kbit LUTRAM）；
//    · tag_word / owner / tag_hi / find_ptr / scan_slot 全部删除；
//    · 释放阶段（S_RELCHK/S_RELSTP）整段删除 —— 行失活就是 line_ready 归零。
//
//  代价与不变量：
//    · 字形 RAM 从 2048 行涨到 4096 行（256 格 × 16 行）⇒ 2 个 32Kb 块；
//    · 同一字符出现在多行时会被重复读 Flash（旧版靠 tag 去重）。
//      实测代价可接受：一次行重装 = 32 列 × 32 字节 = 1 KB ≈ 0.3 ms，
//      而旧版每次未命中就要等 21 µs、一页切换常有上百个未命中。
//    · 每次行重装都会把该行**全部字符**重读一遍（不再判断"字形是否已在片内"），
//      所以重装期间 line_ready=0 的窗口比以前略长一点，但绝对时间更短。
//
//  【端口语义一条没变】
//    · slot_rd_data 仍是"每行 10 位的槽号"，只是高位恒 0、低位 = {行, 列}；
//    · glyph_rd_addr 仍是 {slot, row}，所以 ps_gen_text 一行都不用改；
//    · line_ready / line_len / line_overflow / Flash 客户口协议全部照旧。
//    · `line_overflow`：新结构不存在"槽位耗尽"，端口保留但恒 0。
//    · NSLOT / SCLK_DIV / POR_CYCLES 三个参数保留兼容（本实现不再使用）。
//
//  【关键正确性约定】
//   ① line_ready 在整个重装期间保持 0 → 画面只可能"这一行暂时透明"，
//      不会出现"半个旧字 + 半个新字"。（ps_gen_text 用 line_ready 门控绘制）
//   ② 空白字符（码 0）不读 Flash，而是把该格的 16 行**清成 0**
//      ⇒ 空白格永远画出空点阵，不需要"槽 0 空白保护"那一套。
//   ③ 装载期间目标 ver 又变 → 影子 ver 与当前 ver 不符 → 下一帧自动重来
//      （pending 是从状态组合出来的，丢 trigger 也不丢 pending）。
//   ④ Flash 地址（gb_addr）有 3 拍流水，所以 cur_code 稳定后要等 4 拍才发请求
//      （S_PRE 状态），否则会读到上一条字形。
//
//  【存储走向】
//    str_mem    NL × MAXLEN × 16bit = 4 Kbit  → BRAM（配置口写、装载口读）
//    glyph_mem  NL*MAXLEN × 16 × 16bit = 64 Kbit → 2 × EG_PHY_BRAM32K（2048×16）
//
//==============================================================================

module osd_font_cache #(
    parameter integer NL        = 4,        // 文本行数（<= 15）
    parameter integer MAXLEN    = 32,       // 每行最多字符数
    parameter integer NSLOT     = 512,      // ☆ 兼容保留：新结构不使用槽表
    parameter integer SCLK_DIV  = 1,        // ☆ 兼容保留（Flash 控制器已上提）
    parameter integer POR_CYCLES = 200000   // ☆ 兼容保留
)(
    input  wire        clk,
    input  wire        rst,                 // 高有效、异步（与其它 APP 模块一致）

    //------------------------------------------------------------------
    //  配置写口 1：写字符串（字符码）
    //------------------------------------------------------------------
    input  wire        str_we,              // 单拍脉冲
    input  wire [3:0]  str_line,            // 第几行
    input  wire [7:0]  str_idx,             // 第几个字符
    input  wire [15:0] str_code,            // 字符码：高字节 0 = ASCII，否则 {qb,wb}

    //------------------------------------------------------------------
    //  配置写口 2：写每行的 active / len / ver
    //------------------------------------------------------------------
    input  wire        ctl_we,              // 单拍脉冲
    input  wire [3:0]  ctl_line,
    input  wire        ctl_active,          // 1 = 这行要显示
    input  wire [7:0]  ctl_len,             // 这行几个字符
    input  wire [7:0]  ctl_ver,             // 文本版本号（内容变就 +1）

    //------------------------------------------------------------------
    //  触发：在帧边界发一个单拍脉冲，本模块才动手搬运
    //------------------------------------------------------------------
    input  wire        trigger,

    //------------------------------------------------------------------
    //  给 ps_gen_text：槽号读口（组合，纯连线）
    //------------------------------------------------------------------
    input  wire [ 8*NL-1:0] slot_rd_col,    // 每行的字符列
    output wire [10*NL-1:0] slot_rd_data,   // 每行对应的槽号（= {行, 列}）

    //------------------------------------------------------------------
    //  给 ps_gen_text：字形 RAM 读口（同步，地址后 1 拍出数据）
    //------------------------------------------------------------------
    input  wire [13:0] glyph_rd_addr,       // = {slot, row}（★ 组合输出，不要打拍）
    output wire [15:0] glyph_rd_row,

    //------------------------------------------------------------------
    //  状态回给上层
    //------------------------------------------------------------------
    output wire [NL-1:0]   line_ready,      // 该行绑定已就绪（重装期间为 0）
    output wire [NL-1:0]   line_overflow,   // 新结构无槽位耗尽：恒 0
    output wire [8*NL-1:0] line_len,        // 该行字符数

    //------------------------------------------------------------------
    //  Flash 客户口（控制器在 osd_top 的 flash_arb，协议与 spi_flash_read 一致）
    //------------------------------------------------------------------
    output wire        fl_req,
    output wire [23:0] fl_addr,
    output wire [15:0] fl_len,

    input  wire        fl_rdy,
    input  wire [7:0]  fl_data,
    input  wire        fl_valid,
    input  wire        fl_done
);

//==============================================================================
//  0. 常量
//==============================================================================
// 常量函数：clog2（iverilog / TD 都支持常量函数在 elaboration 期折叠）
function integer clog2;
    input integer value;
    integer v;
    begin
        v = value - 1;
        for (clog2 = 0; v > 0; clog2 = clog2 + 1) v = v >> 1;
    end
endfunction

localparam integer NCELL = NL * MAXLEN;                     // 字格总数（8×32 = 256）
localparam integer CBW   = (MAXLEN > 1) ? clog2(MAXLEN) : 1; // 列号位宽（32 → 5）
localparam integer LBW   = (NL     > 1) ? clog2(NL)     : 1; // 行号位宽（8  → 3）
// 字形 RAM 地址 = {行, 列, 行内像素行(4)}；深度必须是 2 的幂（RAM 推断硬要求）
localparam integer GAW   = LBW + CBW + 4;                   // 12
localparam integer GDEPTH= 1 << GAW;                        // 4096
localparam integer NBANK = (GDEPTH + 2047) / 2048;          // 2 个 32Kb 块
// 字符串表：NL × MAXLEN 项，深度圆整到 2 的幂
localparam integer STRD  = 1 << clog2(NCELL);               // 256
localparam integer SAW   = clog2(STRD);                     // 8
localparam [3:0]   NL4   = NL[3:0];

//==============================================================================
//  1. 存储体
//==============================================================================
// ---- 字符串表：配置口写、装载口读（都是单地址，落 BRAM）----
reg [15:0] str_mem [0:STRD-1];      /* fehdl force_ram=1, ram_style="block" */

// ---- 字形 RAM：每个字格 16 行 × 16 位 ----
`ifdef IVERILOG_SIM
reg [15:0] glyph_mem [0:GDEPTH-1];
`endif

//==============================================================================
//  5. 行状态寄存器（配置口与 FSM 都要用 → 必须声明在使用之前）
//==============================================================================
reg        line_act_r    [0:NL-1];
reg [7:0]  line_len_r    [0:NL-1];
reg [7:0]  line_ver_r    [0:NL-1];
reg [7:0]  line_ver_shad [0:NL-1];  // 已装载完成的那一版 ver（影子）
reg        line_rdy_r    [0:NL-1];

reg          booted;                // 首帧立即装载：上电后不等 trigger 先装一次

genvar gi;

//==============================================================================
//  2. 配置写口
//==============================================================================
wire        str_we_ok  = str_we && (str_line < NL4) && (str_idx < MAXLEN[7:0]);
wire [15:0] str_waddr  = str_line * MAXLEN + str_idx;
wire        ctl_we_ok  = ctl_we && (ctl_line < NL4);

always @(posedge clk) begin
    if (str_we_ok) str_mem[str_waddr[SAW-1:0]] <= str_code;
end

// 行属性寄存器：只由配置口写
integer ri;
always @(posedge clk or posedge rst) begin
    if (rst) begin
        for (ri = 0; ri < NL; ri = ri + 1) begin
            line_act_r[ri] <= 1'b0;
            line_len_r[ri] <= 8'd0;
            line_ver_r[ri] <= 8'd0;
        end
    end else if (ctl_we_ok) begin
        line_act_r[ctl_line] <= ctl_active;
        line_len_r[ctl_line] <= ctl_len;
        line_ver_r[ctl_line] <= ctl_ver;
    end
end

//==============================================================================
//  3. 槽号 = 纯连线（新结构的核心）
//==============================================================================
//  旧版这里是 8 份 LUTRAM 槽位表（每行一份 32×10）+ 一份 FSM 读副本。
//  新结构里"第 gi 行第 c 列的槽号"恒等于 {gi, c}，不需要任何存储。
//  地址 = {slot[9:0], row[3:0]} = {2'b00, gi, c, row} ⇒ 与字形 RAM 布局逐位对应。
generate
for (gi = 0; gi < NL; gi = gi + 1) begin : g_slot
    localparam [LBW-1:0] LI = gi;           // genvar 不能直接位选 → 先落成定宽常量
    wire [7:0]  c8 = slot_rd_col[8*gi +: 8];
    assign slot_rd_data[10*gi +: 10] = { {(10-LBW-CBW-1){1'b0}}, 1'b0,
                                          LI, c8[CBW-1:0] };
end
endgenerate

//==============================================================================
//  4. 待处理判定（组合导出，丢 trigger 也不丢 pending）
//==============================================================================
//  行 L 需要动一动的条件：
//    · active=1 且（还没就绪 或 已绑定的 ver 与当前 ver 不符）
//    · active=0 且目前还标着就绪（→ 只需把 ready 拉低，没有东西要"释放"了）
wire [NL-1:0] pend;
generate
for (gi = 0; gi < NL; gi = gi + 1) begin : g_pend
    assign pend[gi] = line_act_r[gi]
                    ? (~line_rdy_r[gi] | (line_ver_shad[gi] != line_ver_r[gi]))
                    : line_rdy_r[gi];
end
endgenerate

wire pend_any = |pend;

//==============================================================================
//  6. 主状态机
//==============================================================================
//  状态：IDLE → (LDCHK → LDCOL → LDCOL2 → [CLR | PRE → FREQ → FRX → FEND] → NCOL)*
//        → LDFIN → LDCHK ... → DONE → IDLE
//  每个字符：2 拍取码 + (空白 ? 16 拍清零 : 4 拍等地址 + 32 字节 Flash)
localparam [3:0] S_IDLE   = 4'd0;
localparam [3:0] S_LDCHK  = 4'd1;   // 选下一行
localparam [3:0] S_LDCOL  = 4'd2;   // 取本列字符码（地址已稳定，本拍采样）
localparam [3:0] S_LDCOL2 = 4'd3;   // 判空白 / 非空白
localparam [3:0] S_CLR    = 4'd4;   // 空白格：清 16 行（每拍一行）
localparam [3:0] S_PRE    = 4'd5;   // 等 gb_addr 3 拍流水 + rd_addr 打拍
localparam [3:0] S_FREQ   = 4'd6;   // 发 Flash 读请求
localparam [3:0] S_FRX    = 4'd7;   // 收 32 字节
localparam [3:0] S_FEND   = 4'd8;
localparam [3:0] S_NCOL   = 4'd9;   // 下一列
localparam [3:0] S_LDFIN  = 4'd10;  // 本行装完
localparam [3:0] S_DONE   = 4'd11;

reg [3:0]  st;
reg [3:0]  cur_line;
reg [7:0]  cur_col;
reg [7:0]  cur_ver;
reg [15:0] cur_code;
reg [4:0]  rx_cnt;          // 0..31（收字节计数）
reg [7:0]  byte_lo;
reg [2:0]  pre_cnt;
reg        rd_req_r;

// 字符串表读（同步 1 拍）：地址用当前 cur_col
wire [SAW-1:0] str_raddr = (cur_line * MAXLEN + cur_col);   // 截到 SAW
reg  [15:0]    str_rdata;

always @(posedge clk) begin
    str_rdata <= str_mem[str_raddr[SAW-1:0]];
end

// ---- 字形 RAM 写口（独立 always：TD 推断 BRAM 的硬要求）----
//   · S_FRX：每收满 2 字节写一行（先来的是左 8 列 → 高位）
//   · S_CLR：空白格逐拍写 0
wire        gm_we    = ((st == S_FRX) && fl_valid && rx_cnt[0]) || (st == S_CLR);
wire [GAW-1:0] gm_waddr = {cur_line[LBW-1:0], cur_col[CBW-1:0],
                           (st == S_CLR) ? rx_cnt[3:0] : rx_cnt[4:1]};
wire [15:0] gm_wdata = (st == S_CLR) ? 16'd0 : {byte_lo, fl_data};

`ifdef IVERILOG_SIM
// 仿真：行为数组的写口（综合走下面显式的 BRAM 原语，写口在各自的端口上）
always @(posedge clk) begin
    if (gm_we) glyph_mem[gm_waddr] <= gm_wdata;
end
`endif

// ---- 字形 RAM 读口（同步 1 拍，无复位：给复位端会让 TD 放弃 BRAM）----
// ★ 读数据寄存器**只能在唯一一个 always 里赋值**：TD 对"同一个 reg 出现在
//   两个 generate 分支里"会报 HDL-8007（并发赋值），哪怕只有一个分支被展开。
//   所以这里把"选 bank + 打拍"统一写成一个 always，generate 里只放 BRAM 原语。
//
// ★★★ 2026-10-29 [S7]：**不能再给 dob 补一级寄存器** ★★★
//   BRAM 原语（REGMODE_B="NOREG"）本身就是"addrb 采样后下一拍 dob 有效"
//   = 1 拍延迟（见 tools/bram32k_stub.v 头部时序说明）。S4 重写时在 dob 后面
//   又加了一个 `always @(posedge clk) glyph_rd_row_r <= gdob` ⇒ 综合出来是
//   **2 拍**，而 IVERILOG_SIM 分支和 ps_gen_text 的流水线都按 **1 拍**校准。
//
//   为什么"出图 7 张全绿"没发现：两套分支走的是不同代码 —— 出图验证的是
//   `glyph_mem[addr]` 那条 1 拍路径，上板跑的是 BRAM 这条 2 拍路径。
//
//   2 拍时的具体错法（也就是上板看到的"文字里的静态白点"）：
//     地址 = {行, 字格列, 字形行}，**在一个字格的 16 个像素内不变**，
//     所以晚一拍只在"字格左边界"的前 2 个像素上取到**上一个字格**的数据：
//         bit 索引 15-col 用的是新字格 ⇒ 取到上一字格 col0/col1 的墨点。
//     字库里大多数字形 col0 是空白（1px 边距），所以只在"上一个字恰好
//     在左边缘有笔画"时冒出一个白点 ⇒ 看起来随机、但始终跟着文字走。
//  ★ 注意：`glyph_rd_row_r` 在仿真分支是 reg、在综合分支是 wire（纯 mux 输出），
//    所以**必须分别在两个分支里声明**。放在公共段会让综合报
//    "concurrent assignment to a non-net 'glyph_rd_row_r'"（TD 已实测）。
wire [15:0] gdob0, gdob1;

`ifdef IVERILOG_SIM
reg  [15:0] glyph_rd_row_r;
always @(posedge clk) glyph_rd_row_r <= glyph_mem[glyph_rd_addr[GAW-1:0]];
assign glyph_rd_row = glyph_rd_row_r;
assign gdob0 = 16'd0;
assign gdob1 = 16'd0;
`else
// bank 选择位必须与 dob 同拍：地址高位打一拍，再用它选 bank（纯 mux，0 拍）
reg         bank_q;
wire [15:0] glyph_rd_row_r;
always @(posedge clk) bank_q <= (GAW > 11) ? glyph_rd_addr[GAW-1] : 1'b0;
assign glyph_rd_row_r = ((GAW > 11) && bank_q) ? gdob1 : gdob0;
assign glyph_rd_row   = glyph_rd_row_r;

generate
if (NBANK == 1) begin : g_bank1
    assign gdob1 = 16'd0;
    EG_PHY_BRAM32K #(
        .MODE("DP16K"), .DATA_WIDTH_A("16"), .DATA_WIDTH_B("16"),
        .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
        .SRMODE("SYNC"), .CSAMUX("SIG"), .CSBMUX("SIG"),
        .OCEAMUX("SIG"), .OCEBMUX("SIG"), .RSTAMUX("SIG"), .RSTBMUX("SIG")
    ) u_glyph_ram (
        .clka(clk), .csa(1'b1), .wea(gm_we), .addra(gm_waddr[10:0]), .dia(gm_wdata),
        .bytea(1'b0), .bytewea(1'b0), .rsta(1'b0), .ocea(1'b1),
        .clkb(clk), .csb(1'b1), .web(1'b0), .addrb(glyph_rd_addr[10:0]), .dib(16'd0),
        .byteb(1'b0), .byteweb(1'b0), .rstb(1'b0), .oceb(1'b1),
        .dob(gdob0)
    );
end else begin : g_bank2
    EG_PHY_BRAM32K #(
        .MODE("DP16K"), .DATA_WIDTH_A("16"), .DATA_WIDTH_B("16"),
        .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
        .SRMODE("SYNC"), .CSAMUX("SIG"), .CSBMUX("SIG"),
        .OCEAMUX("SIG"), .OCEBMUX("SIG"), .RSTAMUX("SIG"), .RSTBMUX("SIG")
    ) u_glyph_ram0 (
        .clka(clk), .csa(1'b1),
        .wea(gm_we && (gm_waddr[GAW-1] == 1'b0)), .addra(gm_waddr[10:0]), .dia(gm_wdata),
        .bytea(1'b0), .bytewea(1'b0), .rsta(1'b0), .ocea(1'b1),
        .clkb(clk), .csb(1'b1), .web(1'b0), .addrb(glyph_rd_addr[10:0]), .dib(16'd0),
        .byteb(1'b0), .byteweb(1'b0), .rstb(1'b0), .oceb(1'b1),
        .dob(gdob0)
    );
    EG_PHY_BRAM32K #(
        .MODE("DP16K"), .DATA_WIDTH_A("16"), .DATA_WIDTH_B("16"),
        .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
        .SRMODE("SYNC"), .CSAMUX("SIG"), .CSBMUX("SIG"),
        .OCEAMUX("SIG"), .OCEBMUX("SIG"), .RSTAMUX("SIG"), .RSTBMUX("SIG")
    ) u_glyph_ram1 (
        .clka(clk), .csa(1'b1),
        .wea(gm_we && (gm_waddr[GAW-1] == 1'b1)), .addra(gm_waddr[10:0]), .dia(gm_wdata),
        .bytea(1'b0), .bytewea(1'b0), .rsta(1'b0), .ocea(1'b1),
        .clkb(clk), .csb(1'b1), .web(1'b0), .addrb(glyph_rd_addr[10:0]), .dib(16'd0),
        .byteb(1'b0), .byteweb(1'b0), .rstb(1'b0), .oceb(1'b1),
        .dob(gdob1)
    );
end
endgenerate
`endif
// 注：glyph_rd_row 已在上面两个分支里各自 assign（见 [S7] 注释）

//------------------------------------------------------------------------------
//  Flash 地址：字符码 → 偏移（gb_addr 内部 2 拍 + 这里 1 拍 = 3 拍）
//------------------------------------------------------------------------------
wire        rd_req   = rd_req_r;
wire [23:0] code_addr;
wire [23:0] rd_addr_r;

gb_addr u_gb_addr (
    .clk  (clk),
    .code (cur_code),
    .addr (code_addr)
);

reg [23:0] rd_addr_q;
always @(posedge clk) rd_addr_q <= code_addr;
assign rd_addr_r = rd_addr_q;

wire rd_ready = fl_rdy;
wire [7:0] rd_data = fl_data;
wire rd_valid = fl_valid;
wire rd_done  = fl_done;

//------------------------------------------------------------------------------
//  主 FSM
//------------------------------------------------------------------------------
integer rj;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        st        <= S_IDLE;
        cur_line  <= 4'd0;
        cur_col   <= 8'd0;
        cur_ver   <= 8'd0;
        cur_code  <= 16'd0;
        rx_cnt    <= 5'd0;
        byte_lo   <= 8'd0;
        pre_cnt   <= 3'd0;
        rd_req_r  <= 1'b0;
        booted    <= 1'b0;
        for (rj = 0; rj < NL; rj = rj + 1) begin
            line_ver_shad[rj] <= 8'd0;
            line_rdy_r   [rj] <= 1'b0;
        end
    end else begin
        rd_req_r <= 1'b0;                   // 单拍脉冲

        case (st)
        //--------------------------------------------------------------
        S_IDLE: begin
            if ((trigger || ~booted) && pend_any) begin
                cur_line <= 4'd0;
                st       <= S_LDCHK;
            end
        end

        //--------------------------------------------------------------
        //  选下一行：失活的直接丢 ready，没有"释放"这一步了
        //--------------------------------------------------------------
        S_LDCHK: begin
            if (cur_line == NL4) begin
                st <= S_DONE;
            end else if (pend[cur_line] && !line_act_r[cur_line]) begin
                line_rdy_r[cur_line] <= 1'b0;      // 这一行不再显示
                cur_line <= cur_line + 4'd1;
            end else if (pend[cur_line]) begin
                cur_ver  <= line_ver_r[cur_line];
                cur_col  <= 8'd0;
                rx_cnt   <= 5'd0;
                // ★ 约定⑦：重装期间本行必须透明。新结构里字形直接写在**本行自己
                //   那几格**上，若这里不拉低 ready，就会看到"一半旧字 + 一半新字"。
                line_rdy_r[cur_line] <= 1'b0;
                st       <= S_LDCOL;
            end else begin
                cur_line <= cur_line + 4'd1;
            end
        end

        //--------------------------------------------------------------
        //  取本列字符码：地址在本拍稳定，str_mem 同步读 ⇒ 下一拍拿到
        //--------------------------------------------------------------
        S_LDCOL: begin
            st <= S_LDCOL2;
        end

        S_LDCOL2: begin
            if (cur_col >= line_len_r[cur_line]) begin
                st <= S_LDFIN;                      // 本行字符都装完了
            end else begin
                cur_code <= str_rdata;
                rx_cnt   <= 5'd0;
                pre_cnt  <= 3'd0;
                st       <= (str_rdata == 16'd0) ? S_CLR : S_PRE;
            end
        end

        //--------------------------------------------------------------
        //  空白格：把这一格的 16 行清 0（写口在 S_CLR 期间每拍写一行）
        //--------------------------------------------------------------
        S_CLR: begin
            if (rx_cnt[3:0] == 4'd15) st <= S_NCOL;
            else                      rx_cnt <= rx_cnt + 5'd1;
        end

        //--------------------------------------------------------------
        //  等 gb_addr(2 拍) + rd_addr 打拍(1 拍) 稳定后再发请求
        //--------------------------------------------------------------
        S_PRE: begin
            if (pre_cnt == 3'd3) st <= S_FREQ;
            else                 pre_cnt <= pre_cnt + 3'd1;
        end

        //--------------------------------------------------------------
        S_FREQ: begin
            // rd_req 必须是单拍脉冲，且只在 rd_ready 时发
            if (rd_ready) begin
                rd_req_r <= 1'b1;
                st       <= S_FRX;
            end
        end

        //--------------------------------------------------------------
        //  收 32 字节 → 每 2 字节写一行（先来的是左 8 列 → 高位）
        //--------------------------------------------------------------
        S_FRX: begin
            if (rd_valid) begin
                if (rx_cnt[0] == 1'b0) byte_lo <= rd_data;
                if (rx_cnt == 5'd31)   st <= S_FEND;
                else                   rx_cnt <= rx_cnt + 5'd1;
            end
        end

        S_FEND: begin
            st <= S_NCOL;
        end

        //--------------------------------------------------------------
        S_NCOL: begin
            if ((cur_col + 8'd1) >= line_len_r[cur_line]) st <= S_LDFIN;
            else begin cur_col <= cur_col + 8'd1; st <= S_LDCOL; end
        end

        //--------------------------------------------------------------
        //  本行装完：绑定生效（用启动时记下的 ver）
        //--------------------------------------------------------------
        S_LDFIN: begin
            line_ver_shad[cur_line] <= cur_ver;
            line_rdy_r   [cur_line] <= 1'b1;
            cur_line <= cur_line + 4'd1;
            st       <= S_LDCHK;
        end

        S_DONE: begin
            booted <= 1'b1;
            st     <= S_IDLE;
        end

        default: st <= S_IDLE;
        endcase
    end
end

//==============================================================================
//  7. 输出
//==============================================================================
generate
for (gi = 0; gi < NL; gi = gi + 1) begin : g_out
    assign line_ready   [gi]        = line_rdy_r[gi];
    assign line_overflow[gi]        = 1'b0;          // 新结构无槽位耗尽
    assign line_len     [8*gi +: 8] = line_len_r[gi];
end
endgenerate

//==============================================================================
//  8. Flash 客户口
//==============================================================================
assign fl_req  = rd_req;
assign fl_addr = rd_addr_r;
assign fl_len  = 16'd32;                // 一个字 = 16 行 × 2 字节

endmodule
