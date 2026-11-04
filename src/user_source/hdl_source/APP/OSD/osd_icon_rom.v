//==============================================================================
//  osd_icon_rom.v —— OSD 位图 ROM（导航图标 + logo，上电从 Flash 一次搬进片上）
//==============================================================================
//
//  【★★★ 2026-10-28：从「4bpp 覆盖率 + 调色板」改成 RGB565 直出 ★★★】
//
//  【为什么要改】原来每个像素只存 0..15 的覆盖率，颜色靠 osd_gfx 里的
//    pal_rgb() 查表 —— 那是 32 项 × 24bit 的常量 case，综合出来 ≈180 个 LUT，
//    比它省下来的 BRAM 贵得多（本设计 LUT 已经到 95.38%）。
//    改成直出色彩后：像素通路只剩「选半字 + 拼回 888」（纯连线），
//    调色板整块删掉，净省约 140 个 LUT。代价是 ROM 从 640 字涨到 6432 字
//    （32Kb 块 1 → 7，池子 4/16 → 11/16，够用）。
//
//  【数据从哪来】icons/osd_bitmaps.bin（25728 B），由 tools/gen_osd_bitmaps.py
//    从 icons/png32/*.png（10 张 32×32）+ icons/logo_ph1.svg 生成。
//    Flash 基址 0x050000（partitions.json 的 osd_bitmaps 分区）。
//
//  【格式】16bpp RGB565，光栅序，**2 像素 / 32bit 字**（低半字 = 偶像素）：
//      word 0    .. 511   icon0 普通态 32×32
//      word 512  .. 1023  icon0 激活态 32×32
//      ...                （5 图标 × 2 态 = 5120 字）
//      word 5120 .. 6431  logo 64×41
//    地址公式见 osd_gfx.v：图标 {idx,act,ly,lx[4:1]} 是纯连线；
//    logo 5120 + {ly,lx[5:1]} 只花 2 个 LUT。
//
//  【字内布局】一个 32bit 字 = 连续 4 个文件字节，**低字节是文件里第一个字节**：
//      word[7:0]=b0  word[15:8]=b1  word[23:16]=b2  word[31:24]=b3
//      所以拼装写 {fl_data, wdata[31:8]}（右移进），4 个字节写一次。
//    ⇒ 文件里就是 16bpp 光栅序（小端），b0/b1 = 偶像素，b2/b3 = 奇像素。
//
//  【读口】地址由 osd_gfx 在**拍 0** 组合给出，本模块的地址寄存器就是流水第 1 级，
//      数据在拍 1 正好可用 ⇒ 零附加延迟。
//
//  【一次性】上电搬完就再也不碰 Flash，不影响稳态性能。
//
//  【★★ 一次搬完、不分段 ★★】
//    图标和 logo 现在在**同一个分区、同一段连续地址**里，所以整个 ROM 只需要
//    一次 fl_req（25728 B ≈ 8.2ms @25MHz SCLK，flash_arb 的超时是 20ms）。
//    ⇒ flash_arb 不用加第 4 个客户，仲裁器的令牌/预约机制一行都不动。
//
//==============================================================================
module osd_icon_rom #(
    parameter [23:0] BASE       = 24'h050000,
    parameter integer NBYTES    = 30976,           // 7744 字 × 4B
    parameter integer POR_CYCLES= 400_000          // ≈8ms @50MHz（等 Flash 上电）
)(
    input  wire        clk,
    input  wire        rst,

    // ---- Flash 客户口（接 flash_arb 的 c1_*）----
    output reg         fl_req,
    output wire [23:0] fl_addr,
    output wire [15:0] fl_len,
    input  wire        fl_rdy,
    input  wire [7:0]  fl_data,
    input  wire        fl_valid,
    input  wire        fl_done,

    // ---- 读口（osd_gfx 拍 0 送地址，拍 1 出数据）----
    input  wire [12:0] ra_addr,
    output wire [31:0] ra_data,
    // ★★ 2026-10-29：流水推进使能（= pixel_ready）★★
    //   读口的输出寄存器必须与其它流水级一起冻结。背压停顿一拍时，坐标
    //   `s1_px` 被冻住而 BRAM 内部输出寄存器照常采样地址 ⇒ 下一拍那个像素
    //   是"坐标 N、位图 N+1"，用错一个 32bit 字 ⇒ 图标上出现 1px 竖虚线。
    //   （同一类问题在 osd_cellmem 的 `rd_geo` 上已经修过一次，cursor 好了；
    //     位图这条是唯一还没被冻的一条。）
    input  wire        ce,

    output wire        ready
);

localparam integer NWORD = NBYTES/4;               // 7744
// ★ 不要对 integer 型参数做位选（TD 会算成 0，osd_top 的 PIPE 踩过）：
//   先把参数装进一个定宽 localparam，再拿它去比较。
localparam [18:0]  POR_MAX = POR_CYCLES;

//------------------------------------------------------------------------------
//  "这块 Flash 到底编程了没有"的判定
//------------------------------------------------------------------------------
//  【为什么必须有】未编程态是 0xFF：16bpp 全 1 ⇒ RGB565 = 0xFFFF = 纯白。
//    不判的话，没烧分区时屏幕上会出现 10 个白块 + 一个白色 logo 矩形。
//  【做法】搬运动程中只看**一个字节**：只要出现非 0xFF 的字节，就认为该区
//    已经编程；全程 0xFF 则判为空白。代价 1 个比较器 + 1 个触发器。
//  【后果】ready=0 时 osd_gfx 不画 type12（见那里的 icon_ok）⇒ 图标和 logo
//    都不出现，不会出现难看的色块。烧好 osd_bitmaps 分区后自动恢复。
//  【★ 注意】图标和 logo 现在共用一次搬运、一个 data_pgm，所以**要么一起有、
//    要么一起没有**。改格式后必须重烧 osd_bitmaps 分区，否则导航图标也会消失。
//------------------------------------------------------------------------------
reg data_pgm;                                      // 1 = 至少读到一个非 0xFF 字节

//------------------------------------------------------------------------------
//  存储：7744 × 32（简单双口：写口 = 加载，读口 = 像素）
//------------------------------------------------------------------------------
// ★ 247,808 bit：必须落块 RAM（否则 7744 个 LUT）。
//
// ★★ 2026-10-28：为什么不能靠 `ram_style` 让它自己选 32Kb 块 ★★
//   · 第一版写的是 `ram_style = "block"` —— 不在 TD 的合法值表里
//     （`td.exe` 里只有 `[9K, 9K(FAST), 32K]`），被当成非法值忽略。
//   · 第二版改成 `ram_style = "32K"` —— **TD 静默忽略**：综合日志里没有告警，
//     但 `.area` 里 `u_icon` 仍是 29 个 9Kb 块、`#bram32k` 仍是 4。
//     同一个结论在 `osd_menu.v` 里已经踩过一次（“TD 不会主动用它”）。
//   ⇒ 结论：**32Kb 池只能靠显式例化 `EG_PHY_BRAM32K` 才用得上。**
//
//   例化形状：EG_PHY_BRAM32K = 2048 × 16。
//     7744 字 / 2048 = 3.78 ⇒ 4 个深度 bank（每个 2048 字）
//     32 位 / 16 = 2 片并联
//     ⇒ **8 块**（9Kb 池一口气让出 29 − 8 = 21 块）
//
//   读延迟：REGMODE_B="NOREG" ⇒ 地址组合送入、**1 拍后 dob 有效**，
//     与原「`ra_q <= mem[ra_addr]`（BRAM 输出寄存器）」**完全一致**，
//     `osd_gfx` 的像素节拍一拍未动。
//     代价只是 dob 之后多一级 4:1 mux（32 位 ≈ 96 LUT、2 级），
//     而它的下落点 `pix <= ...`（osd_gfx:237）是寄存器，不构成新长路径。
//   写口：只置起目标 bank 的 `wea`，其余 bank 写使能为 0（`csa` 恒 1）。
//   32K 块不支持 read-before-write，这里是标准 SDP，不涉及同址同拍读写。
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//  状态机
//------------------------------------------------------------------------------
localparam [1:0] S_POR = 2'd0, S_REQ = 2'd1, S_RX = 2'd2, S_DONE = 2'd3;

reg [1:0]  st;
reg [18:0] por_cnt;
reg [14:0] bcnt;                 // 已收字节数 0..25727（15 位）
reg [31:0] shf;                  // 拼装：低字节 = 文件里第一个字节
reg        we_p;                 // 上一字节是"行内第 4 个" ⇒ 本拍写
reg [12:0] wa_p;

assign fl_addr = BASE;
assign fl_len  = NBYTES[15:0];
assign ready   = (st == S_DONE) && data_pgm;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        st      <= S_POR;
        por_cnt <= 19'd0;
        fl_req  <= 1'b0;
        bcnt    <= 15'd0;
        shf     <= 32'd0;
        we_p    <= 1'b0;
        wa_p    <= 13'd0;
        data_pgm<= 1'b0;
    end else begin
        fl_req <= 1'b0;
        we_p   <= 1'b0;

        case (st)
            // 等 Flash 上电稳定（SPI 控制器自己也有 POR，这里只是别太早发请求）
            S_POR: begin
                if (por_cnt == POR_MAX) st <= S_REQ;   // ★ [AREA] 从 0 逐一递增、到点即停 ⇒ == 与 >= 等价
                else                          por_cnt <= por_cnt + 19'd1;
            end

            // 发一次整段读请求
            S_REQ: begin
                if (fl_rdy) begin
                    fl_req <= 1'b1;
                    st     <= S_RX;
                end
            end

            // 收 NBYTES 字节，每 4 字节写一个字
            // ★★★ 拼装必须是 `{fl_data, shf[31:8]}` ★★★
            //   【踩过的坑】曾经写成 `{fl_data, shf[23:0]}` —— 少看了 8 位。
            //   这是"把新字节塞进最高字节"的移位写法，旧值必须整体右移 8 位
            //   （也就是取 `shf[31:8]`）。写成 `shf[23:0]` 相当于每收一个字节
            //   就把**最低字节丢掉**、同时**最高字节被新数据覆盖**：
            //       收 b0 → 0x00_00_00_00
            //       收 b1 → 0x01_00_00_00
            //       收 b2 → 0x02_00_00_00   ← b1 已经被冲到最低字节又丢掉了
            //       收 b3 → 0x03_00_00_00
            //   ⇒ 一个 32 位字里只有【最高字节】有数据，其余三个字节恒为 0。
            //   【造成的视觉】当时图标是 4bpp 覆盖率图，一行 32 像素 = 16 字节 = 4 个字，
            //   只有每 8 个像素里的最后 2 个有明暗变化 ⇒ 整块看上去是一片同色
            //   （用户说的"只有纯色色块"）。
            //   【文件头注释本来写的就是对的】"拼装写 {fl_data, wdata[31:8]}"，
            //   是下面这行的实现写错了 —— 文档与代码不一致的典型案例。
            //   【tb_icon_rom.v 覆盖】T1 验证未编程判定、T2 用可预测 pattern 逐字节
            //   比对拼装方向、T3 用真实 osd_bitmaps.bin 抽查三个字。
            S_RX: begin
                if (fl_valid) begin
                    // ★ 只要有一个字节不是擦除态 0xFF，就说明这片区域真的编程过
                    if (fl_data != 8'hFF) data_pgm <= 1'b1;
                    shf <= {fl_data, shf[31:8]};
                    if (bcnt[1:0] == 2'd3) begin
                        we_p <= 1'b1;
                        wa_p <= bcnt[14:2];
                    end
                    bcnt <= bcnt + 15'd1;
                end
                if (fl_done) st <= S_DONE;
            end

            default: st <= S_DONE;      // 加载完成，永久停在这里
        endcase
    end
end

//------------------------------------------------------------------------------
//  读地址的"停顿冻结"
//------------------------------------------------------------------------------
//  ★★ 2026-10-29：位图这条路**没有有效的 pixel valid**，改成冻地址 ★★
//   背压停顿那一拍：`app_scan_gen` 已经把 x/y 推进到"下一个待消费像素"，
//   而 `osd_gfx` 的 `s1_px` 冻在当前像素。BRAM 的输出寄存器没有 valid 端可用
//   （`oceb` 在 REGMODE_B="NOREG" 下被旁路，实测接 `ce` 毫无效果），
//   于是 `dob` 会多走一格 ⇒ 那个像素"坐标 N、位图 N+1"。
//    ⇒ 改成**冻地址**：ce=0 时继续送上一拍的地址，BRAM 重复采到同一个字，
//      输出自然保持不变。**不增加任何延迟**（正常 ce=1 时是直通）。
reg  [12:0] ra_hold;
always @(posedge clk) if (ce) ra_hold <= ra_addr;   // ★ 只记「被接收」那一拍的地址
wire [12:0] ra_eff = ce ? ra_addr : ra_hold;       // 停顿拍用旧地址

//------------------------------------------------------------------------------
//  存储实现（两种，靠 IVERILOG_SIM 切换；地址/FSM/拼装全共用）
//------------------------------------------------------------------------------
`ifdef IVERILOG_SIM
// ---- 仿真：行为 RAM（延迟与 BRAM 一致：地址打一拍出数据）----
reg [31:0] mem [0:NWORD-1];
reg  [31:0] ra_q;
always @(posedge clk) begin
    if (we_p) mem[wa_p] <= shf;
end
always @(posedge clk) begin
    ra_q <= mem[ra_eff];               // ★ 停顿拍地址被冻住 ⇒ 输出自然不变
end
assign ra_data = ra_q;

`else
// ---- 综合：4 个深度 bank × 低/高 16 位 = 8 个 EG_PHY_BRAM32K ----
//   bank = addr[12:11]，bank 内地址 = addr[10:0]（2048 字）
//   NWORD 必须 ≤ 8192（4 × 2048）；当前 7744。
wire [31:0] dob_b [0:3];

genvar gb;
generate
for (gb = 0; gb < 4; gb = gb + 1) begin : g_bank
    localparam [1:0] BIDX = gb;
    // 写使能只给目标 bank（csa 恒 1，靠 wea 区分）
    wire we_b = we_p && (wa_p[12:11] == BIDX);

    EG_PHY_BRAM32K #(
        .MODE("DP16K"), .DATA_WIDTH_A("16"), .DATA_WIDTH_B("16"),
        .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
        .SRMODE("SYNC"), .CSAMUX("SIG"), .CSBMUX("SIG"),
        .OCEAMUX("SIG"), .OCEBMUX("SIG"),
        .RSTAMUX("SIG"), .RSTBMUX("SIG")
    ) u_lo (
        .clka(clk), .csa(1'b1), .wea(we_b), .addra(wa_p[10:0]), .dia(shf[15:0]),
        .bytea(1'b0), .bytewea(1'b0), .rsta(1'b0), .ocea(1'b1),
        .clkb(clk), .csb(1'b1), .web(1'b0), .addrb(ra_eff[10:0]), .dib(16'd0),
        .byteb(1'b0), .byteweb(1'b0), .rstb(1'b0), .oceb(1'b1),
        .dob(dob_b[gb][15:0])
    );

    EG_PHY_BRAM32K #(
        .MODE("DP16K"), .DATA_WIDTH_A("16"), .DATA_WIDTH_B("16"),
        .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),
        .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),
        .SRMODE("SYNC"), .CSAMUX("SIG"), .CSBMUX("SIG"),
        .OCEAMUX("SIG"), .OCEBMUX("SIG"),
        .RSTAMUX("SIG"), .RSTBMUX("SIG")
    ) u_hi (
        .clka(clk), .csa(1'b1), .wea(we_b), .addra(wa_p[10:0]), .dia(shf[31:16]),
        .bytea(1'b0), .bytewea(1'b0), .rsta(1'b0), .ocea(1'b1),
        .clkb(clk), .csb(1'b1), .web(1'b0), .addrb(ra_eff[10:0]), .dib(16'd0),
        .byteb(1'b0), .byteweb(1'b0), .rstb(1'b0), .oceb(1'b1),
        .dob(dob_b[gb][31:16])
    );
end
endgenerate

// dob 比 ra_addr 晚 1 拍 ⇒ bank 选择也要晚 1 拍
reg [1:0] ra_bank_q;
always @(posedge clk) ra_bank_q <= ra_eff[12:11];

reg [31:0] ra_mux;
always @(*) begin
    case (ra_bank_q)
        2'd0:    ra_mux = dob_b[0];
        2'd1:    ra_mux = dob_b[1];
        2'd2:    ra_mux = dob_b[2];
        default: ra_mux = dob_b[3];
    endcase
end
assign ra_data = ra_mux;
`endif

endmodule
