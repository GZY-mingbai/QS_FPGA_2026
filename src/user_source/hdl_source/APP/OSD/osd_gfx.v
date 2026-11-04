//==============================================================================
//  osd_gfx.v —— OSD 图形层（控件表 → 像素），2 拍
//==============================================================================
//
//  拍 0：x,y → 并行命中 → idx（组合）→ 直接当 cellmem 的地址
//        同时用 x,y + 命中槽号算出**位图 ROM 地址**（组合送出去）
//  拍 1：cellmem 吐出该条的几何/属性 → 算局部坐标 → 取半字 → 拼回 RGB888 →
//        osd_widget → 打拍输出
//
//  ★ 为什么只有 2 拍：命中结果**组合**送进 RAM 地址，RAM 自己的地址寄存器
//    就是第 1 级；第 2 级是控件像素寄存器。这样它和 ps_gen_text（2 拍）同深，
//    两路可以直接进同一个合并级 —— 不需要额外的对齐寄存器。
//
//  ★★ 位图（导航图标 / logo）为什么也是 2 拍 ★★
//    位图 ROM 的地址在**拍 0**（用未打拍的 x,y）就算好并送进它的地址寄存器，
//    数据在拍 1 开头就稳定；拍 1 里只有"半字选择 + 565 拼回 888"两段**纯组合**
//    （都是连线，零 LUT），和 osd_widget 的求值在同一拍完成 ⇒ 总延迟不变。
//
//==============================================================================
//  ★★★ 2026-10-28：删掉调色板，改成 RGB565 直出 ★★★
//==============================================================================
//  【原来】ROM 里存 4bpp 覆盖率，拍 1 做「字节选择 → 半字节选择 → pal_rgb()」。
//    pal_rgb 是 32 项 × 24bit 的常量 case，综合出来 ≈180 个 LUT —— 这是本模块
//    最贵的一块，而且还在像素通路上压了 4~5 级 LUT。
//  【现在】ROM 里直接存 RGB565（2 像素/字），拍 1 只做：
//        选半字（lx[0]）→ {R5,R5[4:2]} 拼回 R8（纯连线）
//    ⇒ 省下 ≈200 LUT（pal_rgb + 字节/半字节选择），只多花一个 16bit 2:1 mux。
//    代价在存储侧：ROM 640 字 → 6432 字（32Kb 块 1 → 7）。
//
//  【地址为什么几乎不花钱】
//    图标：{idx[2:0], act, ly[4:0], lx[4:1]} —— 纯拼接，**0 LUT**
//          （idx 0..4、act 0/1 ⇒ word 0..5119，正好是 ROM 的前半段）
//    logo：5120 + {ly[5:0], lx[5:0]}（ARGB8888，1 像素/字）
//          5120 = 0x1400 = bit12 + bit10，而 {ly,lx} ≤ 0xA3F
//          ⇒ 只有 bit10..12 会进位，写成
//             {~(x[11]&x[10]), x[11]^x[10], ~x[10], x[9:0]}，**4 LUT**
//    两者再做一个 13 位 2:1 mux（≈13 LUT）。
//
//  【★ act（普通/激活态）为什么必须在拍 0 就知道】
//    现在两态是**两份独立位图**，act 要进地址。而 style 在 rd_attr 里、拍 1 才有。
//    ⇒ 由 osd_menu 导出 ic_act_mask[7:0]（one-hot 标出哪个导航图标是激活态），
//    拍 0 直接查表。它在一帧之内不变，所以组合使用是安全的。
//    掩码给 8 位而不是 5 位，是为了让 ic_i（= hit_idx[2:0]−5，非图标槽会是
//    5/6/7）永远落在范围内 —— 否则仿真里会取到 X 污染地址。
//
//  【★ 两种格式：图标 RGB565 / logo ARGB8888 ★】
//    图标是"不透明贴片"（底板 + 墨色），alpha 恒 255 ⇒ 16bpp 就够。
//    logo 必须**逐像素 alpha**：它的背景要和左栏面板一模一样（#0E0E0E @ 75%），
//    写成不透明的 0x0E0E0E 就是一块实心矩形（面板下面还有 25% 的视频透上来，
//    实心的没有 ⇒ 出图一眼看到一个方框）。所以 logo 存 32bpp，由生成工具
//    按 A = 192+63c 预合成好，RTL 直接整字输出。
//    ★ 这一条是**出图才发现的**：最初以为 ps_merge_alpha_pp 只用全局 fade，
//      实际它算的是 a9 = A*alpha/256 —— 像素自带的 A 是参与混合的。
//==============================================================================
module osd_gfx #(
    parameter integer NSLOT = 24
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        pipe_ce,         // 流水推进（= pixel_ready）
    input  wire        adv,             // 本拍是否装入新像素（= ps_start & pixel_ready）
    input  wire [15:0] x,
    input  wire [15:0] y,

    // 控件表写口（帧间窗口，来自布局引擎）
    input  wire        wr_en,
    input  wire [4:0]  wr_idx,
    input  wire [38:0] wr_geo,
    input  wire [23:0] wr_attr,         // {type,style,rad, mask[1:0], v0[9:0]}

    // 位图 ROM 读口（地址拍 0 送，数据拍 1 回）
    output wire [12:0] icon_raddr,
    input  wire [31:0] icon_word,
    // ★ 位图 ROM 就绪（数据搬完 **且** Flash 那片区域确实编程过）
    //   为 0 时 type12（Icon / Logo）不画 —— 否则未编程的 0xFF 会渲染成白块。
    input  wire        icon_ok,
    // ★ 导航激活态掩码（one-hot，bit i = 第 i 个导航图标处于激活态）
    input  wire [7:0]  ic_act_mask,
    // ★ 2026-10-29 [S6]：菜单开关。11 个"几何固定"的字格（遮罩/左栏/分割线/
    //   页面板/5 个图标/标题线/logo）只在菜单打开时有效（菜单关掉时 osd_menu
    //   会把它们的矩形清零），所以它们的命中判定 = menu_on && 常量矩形。
    input  wire        menu_on,

    // 像素输出 {A8, R, G, B}
    output reg  [31:0] pix,
    output reg         out_valid
);

//------------------------------------------------------------------------------
//  动态字格（只有 9 个）：cellmem 只装这些
//------------------------------------------------------------------------------
//  ★★ 2026-10-29 [S6]：常量槽从 cellmem 里搬走 ★★
//  20 个字格里有 11 个的矩形在 osd_menu 的布局 case 里是**编译期常量**：
//      cell0 全屏遮罩、cell1 左栏、cell2 分割线、cell3 页面板、
//      cell5..9 五个导航图标、cell15 标题下划线、cell19 logo
//  它们唯一的"变量"是 menu_on（菜单关掉时 osd_menu 把矩形清零）。
//  实测（探针 P3：NSLOT 20→10）每少一个槽全片掉约 100 LUT ⇒ 这 11 个槽
//  值 ~1100 LUT，而它们需要的只是一个常量矩形比较（下面 §"常量槽"）。
//  剩下的 9 个才是真正每帧重算的：4（游标/弹层底板）、10..14（行控件）、
//  16..18（弹层行，菜单关时复用成信息条）。
wire        dyn_hit;
wire [4:0]  dyn_idx;
wire [38:0] dyn_geo;
wire [23:0] dyn_attr;

osd_cellmem #(.NSLOT(NSLOT)) u_mem (
    .clk     (clk),
    .rst     (rst),
    .wr_en   (wr_en),
    .wr_idx  (wr_idx),
    .wr_geo  (wr_geo),
    .wr_attr (wr_attr),
    .px      (x[9:0]),
    .py      (y[8:0]),
    .hit_v   (dyn_hit),
    .hit_idx (dyn_idx),
    .rd_idx  (dyn_idx),
    .rd_geo  (dyn_geo),
    .rd_attr (dyn_attr),
    // ★ 2026-10-29：读口输出寄存器必须与其它流水级一起冻结（见 osd_cellmem 注释）
    .ce      (pipe_ce)
);

//------------------------------------------------------------------------------
//  拍 0：位图 ROM 地址
//------------------------------------------------------------------------------
//  ★ 槽位约定（必须和 osd_menu.v 的控件表一致）
//      cell 5..9   = 导航图标 i = 0..4，固定放在 (32, 72+80i) - (64, 104+80i)
//      cell 19     = logo，固定放在 (16, 8) - (80, 49)
//  两者都是**固定原点**，所以拍 0 只有 hit_idx 就能算局部坐标，不用读 cellmem。
// ★★ 2026-10-29：ic_i / ic_odd 改成**并行译码**（原来是一条 4 级 mux 链）★★
//  【为什么】上板现象：左侧导航图标出现**竖向**的像素抖动，虚线是白/蓝
//    （不是黑 ⇒ 不是"没画"，是**取错了位图字**）。
//    icon_waddr = {ic_i[2:0], ic_act, ic_ly5[4:0], x[4:1]}
//                                 └ 行 ┘        └ 列 ┘
//    列那 4 位是 `x[4:1]`，纯寄存器切片（0 级逻辑），不可能错；
//    行那 5 位要经过 `y → ic_i(4 级 mux 链) → ic_odd(+1) → xor(+1)` = **6 级**。
//    行地址错 ⇒ 整行取到相邻行 ⇒ 边缘那 1px 白描边变成"有的行 2px、有的行 0px"
//    ⇒ **看着就是竖向虚线**（横向描边因为错位方向与自身平行，不明显），
//    与观察完全一致。
//
//  【怎么改】把阈值比较全部**并行**化，再用"数个数"代替 mux 链：
//    ic_i   = (y≥152)+(y≥232)+(y≥312)+(y≥392)   ← 4 个 1bit 相加，2 级加法树
//    ic_odd = (y∈[152,232)) | (y∈[312,392))      ← 比较器之后只 1 级
//    ⇒ y → addrb[8:4] 从 6 级降到 **3 级**（比较 1 + ic_odd 1 + xor 1）
//    ★ 等价性：ic_i 的"数阈值个数"与原 mux 链逐值相同（见 tb_osd_icons 5120 点）；
//      ic_odd 展开后 = ic_i∈{1,3}，而 ic_i=1 ↔ y∈[152,232)、ic_i=3 ↔ y∈[312,392)。
wire y_ge_152 = (y >= 9'd152);
wire y_ge_232 = (y >= 9'd232);
wire y_ge_312 = (y >= 9'd312);
wire y_ge_392 = (y >= 9'd392);

wire [2:0]  ic_i  = {2'b00, y_ge_152} + {2'b00, y_ge_232} +
                    {2'b00, y_ge_312} + {2'b00, y_ge_392};

// logo 在 (16,8)-(80,49)：判定只需要 y < 49（图标从 y=72 才开始，不冲突）
wire        is_logo0 = (y < 9'd49);

// ★★ 2026-10-28：拍 0 的局部坐标算术化简（**逐位等价**，见下）★★
//  【化简 1：lx 的 10 位减法整个删掉】
//    原来 `ic_lx = x - 32`，但只取 `ic_lx[4:1]`。
//    32 是 32 的倍数 ⇒ (x−32) mod 32 == x mod 32 ⇒ **ic_lx[4:1] ≡ x[4:1]**。
//    穷举 x=0..1023 全部命中（0 不匹配）。
//
//  【化简 2：5 选 1 的 ic_y0 + 9 位减法 → 一个 5 位减法】
//    五个图标 y0 = 72 + 80i，且 `default` = 392；只取 `ic_ly[4:0]`：
//        (72+80i) mod 32 = 8 + 16·(i 为奇数且 i<4)      （default 392 ≡ 8）
//    ⇒ ic_ly[4:0] = y[4:0] − 8 − {sel16, 4'b0}
//    穷举 hit_idx 0..31 × y 0..511 全部命中（0 不匹配）。
//    ★ 注意 sel16 的条件是 `ic_i[0] & ~ic_i[2]`，**不是** `ic_i[0]`：
//      ic_i=5/6/7 走的是 case 的 default（392），若按"奇偶"判就会错 4096 组
//      —— 这条是穷举逼出来的，差点写错。
//    ★ 再进一步：模 32 下 −16 ≡ +16 ≡ 异或 bit4 ⇒ 那一位用 XOR 而不是减法，
//      于是「5 位减法」与 ic_i 无关（可以和比较并行算），
//      与 ic_i 相关的只剩**一个异或门**。
// ic_odd：ic_i∈{1,3} ⇒ y 落在 [152,232) 或 [312,392) 这两段
//   ★ 直接从比较结果算，**不再等 ic_i 的加法树**（少 2 级）
wire        ic_odd = (y_ge_152 && !y_ge_232) || (y_ge_312 && !y_ge_392);
wire [4:0]  ic_ly8 = y[4:0] - 5'd8;               // 与比较器并行，不进关键路径
wire [4:0]  ic_ly5 = ic_ly8 ^ {ic_odd, 4'b0000};  // 只异或 bit4
wire        ic_act = ic_act_mask[ic_i];

// 图标字地址：{idx[2:0], act, ly[4:0], lx[4:1]}  ⇒ 0..5119，纯拼接 0 LUT
//   （RGB565，2 像素/字，所以 lx 只用 [4:1]）
wire [12:0] icon_waddr = {ic_i, ic_act, ic_ly5, x[4:1]};

// logo 字地址：5120 + {ly[5:0], lx[5:0]}（ARGB8888，**1 像素/字**）
//   5120 = 0x1400 = bit12 + bit10，x ≤ 0xA3F ⇒ 只有 bit10..12 进位：
//     5120 + x = {~(x[11]&x[10]), x[11]^x[10], ~x[10], x[9:0]}
// ★ 2026-10-28：与上面同理，只取低 6 位 ⇒ 减法从 10/9 位缩到 6 位
//   （(x−16) mod 64 == x[5:0]−16，(y−8) mod 64 == y[5:0]−8）
wire [5:0]  lg_lx  = x[5:0] - 6'd16;
wire [5:0]  lg_ly  = y[5:0] - 6'd8;
wire [11:0] lg_x   = {lg_ly, lg_lx};
wire [12:0] logo_waddr = {~(lg_x[11] & lg_x[10]), lg_x[11] ^ lg_x[10],
                          ~lg_x[10], lg_x[9:0]};

// is_logo0 已在上面按 y 译码得到（不再用 hit_idx）
assign icon_raddr = is_logo0 ? logo_waddr : icon_waddr;

//------------------------------------------------------------------------------
//  常量槽（11 个）的命中 + 最终优先级合并 ★ 2026-10-29 [S6]
//------------------------------------------------------------------------------
//  这 11 个槽的矩形来自 osd_menu 布局 case 的**编译期常量**，唯一的变量是
//  menu_on（菜单关掉时 osd_menu 会把它们的矩形清零）。所以不需要在 cellmem
//  里占 63 位存储 + 一套命中比较，一个常量矩形比较就够。
//  ★ 改 osd_menu 的布局必须同步改下面这些常量 ★（同 icon/logo 地址公式的纪律）
//
//  优先级（索引大者在上）与改造前逐位一致：
//      bg(0..3) < 4(游标) < icon(5..9) < ctrl(10..14) < t15(15) < pop(16..18) < logo(19)
wire [9:0] cpx = x[9:0];
wire [8:0] cpy = y[8:0];

// ---- cell0..3：整屏竖向切分（只有 x 三段）----
wire cb3 = (cpx >= 10'd99);
wire cb2 = (cpx < 10'd99) && (cpx >= 10'd96);
wire cb1 = (cpx < 10'd96);
wire [4:0] idx_bg = cb3 ? 5'd3 : cb2 ? 5'd2 : cb1 ? 5'd1 : 5'd0;

// ---- cell5..9：五个 32×32 图标，x∈[32,64)，y 落在 5 条 80 间距的带里 ----
wire ic_band = (cpy >= 9'd72  && cpy < 9'd104) || (cpy >= 9'd152 && cpy < 9'd184)
            || (cpy >= 9'd232 && cpy < 9'd264) || (cpy >= 9'd312 && cpy < 9'd344)
            || (cpy >= 9'd392 && cpy < 9'd424);
wire c_hit_icon = menu_on && (cpx >= 10'd32) && (cpx < 10'd64) && ic_band;

// ---- cell15：标题下划线 (112,44)-(624,45)；cell19：logo (16,8)-(80,49) ----
wire c_hit_t15 = menu_on && (cpx >= 10'd112) && (cpx < 10'd624) && (cpy == 9'd44);
wire c_hit_t19 = menu_on && (cpx >= 10'd16)  && (cpx < 10'd80)
              && (cpy >= 9'd8) && (cpy < 9'd49);

// ---- 动态组内部分段（cellmem 的优先编码器已按索引给出组内最高者）----
wire dyn_pop  = dyn_hit && (dyn_idx >= 5'd6);                     // → 16..17（[LUT-2] 第 3 行走选中行控件槽）
wire dyn_ctrl = dyn_hit && (dyn_idx >= 5'd1) && (dyn_idx <= 5'd5); // → 10..14
wire dyn_low  = dyn_hit && (dyn_idx == 5'd0);                     // → 4

wire       hit_v   = c_hit_t19 | dyn_pop | c_hit_t15 | dyn_ctrl
                   | c_hit_icon | dyn_low | menu_on;
wire [4:0] hit_idx = c_hit_t19  ? 5'd19 :
                     dyn_pop    ? (5'd16 + (dyn_idx - 5'd6)) :
                     c_hit_t15  ? 5'd15 :
                     dyn_ctrl   ? (5'd10 + (dyn_idx - 5'd1)) :
                     c_hit_icon ? (5'd5 + {2'b00, ic_i}) :
                     dyn_low    ? 5'd4 :
                                  idx_bg;
wire       sel_const = c_hit_t19  ? 1'b1 :
                       dyn_pop    ? 1'b0 :
                       c_hit_t15  ? 1'b1 :
                       dyn_ctrl   ? 1'b0 :
                       c_hit_icon ? 1'b1 :
                       dyn_low    ? 1'b0 : 1'b1;      // 落到背景组（常量）

reg        sel_const_q;
reg [4:0]  ridx_q;
reg        ic_act_q;
always @(posedge clk) begin
    if (pipe_ce) begin
        sel_const_q <= sel_const;
        ridx_q      <= hit_idx;
        ic_act_q    <= ic_act;          // 图标样式位要跟数据同拍
    end
end

// ---- 常量槽的矩形与属性：用 3 位"类别码"驱动一个**并行 case** ----
//  ⚠ 不要写成 "(idx==0)?A:(idx==1)?B:..." 的链：那是 6 级 39 位串联 mux，
//    实测比下面的并行 case 多花约 80 LUT。
wire [2:0]  icon_i  = ridx_q[2:0] - 3'd5;               // 5..9 → 0..4
wire [8:0]  icon_y0 = 9'd72 + ({6'd0, icon_i} << 6) + ({6'd0, icon_i} << 4);
wire        is_icon_slot = (ridx_q >= 5'd5) && (ridx_q <= 5'd9);

wire [2:0] ccls = (ridx_q == 5'd0)  ? 3'd0 :    // 全屏遮罩
                  (ridx_q == 5'd1)  ? 3'd1 :    // 左栏
                  (ridx_q == 5'd2)  ? 3'd2 :    // 分割线
                  (ridx_q == 5'd3)  ? 3'd3 :    // 页面板
                  is_icon_slot      ? 3'd4 :    // 导航图标
                  (ridx_q == 5'd15) ? 3'd5 :    // 标题下划线
                                      3'd6;     // logo

reg [38:0] cgeo;
reg [23:0] cattr;
always @(*) begin
    // 这 11 个槽的 rad/mask/v0 全是 0 ⇒ 属性只需要 type + style 8 位，
    // 其余 16 位直接接常数 0（比整条 24 位 mux 省一半）。
    case (ccls)
        3'd0: begin cgeo = {1'b1, 10'd640, 9'd480, 10'd0,   9'd0};
                    cattr = {4'd0,  4'd2,  16'd0}; end
        3'd1: begin cgeo = {1'b1, 10'd96,  9'd480, 10'd0,   9'd0};
                    cattr = {4'd0,  4'd1,  16'd0}; end
        3'd2: begin cgeo = {1'b1, 10'd99,  9'd480, 10'd96,  9'd0};
                    cattr = {4'd0,  4'd10, 16'd0}; end
        3'd3: begin cgeo = {1'b1, 10'd640, 9'd480, 10'd99,  9'd0};
                    cattr = {4'd0,  4'd1,  16'd0}; end
        3'd4: begin cgeo = {1'b1, 10'd64,  icon_y0 + 9'd32, 10'd32, icon_y0};
                    // style 的值 type12 不消费（图标激活色走 ic_act_mask），
                    // 这里仍按"激活 6 / 常态 5"填，保持属性表语义一致
                    cattr = {4'd12, (ic_act_q ? 4'd6 : 4'd5), 16'd0}; end
        3'd5: begin cgeo = {1'b1, 10'd624, 9'd45,  10'd112, 9'd44};
                    cattr = {4'd9,  4'd10, 16'd0}; end
        default: begin cgeo = {1'b1, 10'd80, 9'd49, 10'd16, 9'd8};
                    cattr = {4'd14, 4'd5,  16'd0}; end
    endcase
end

wire [38:0] rd_geo  = sel_const_q ? cgeo  : dyn_geo;
wire [23:0] rd_attr = sel_const_q ? cattr : dyn_attr;

//------------------------------------------------------------------------------
//  第 1 级：坐标随像素一起延迟 1 拍
//------------------------------------------------------------------------------
reg [9:0] s1_px, s1_py;
reg       s1_v, s1_hit;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        s1_px  <= 10'd0;
        s1_py  <= 9'd0;
        s1_v   <= 1'b0;
        s1_hit <= 1'b0;
    end else if (pipe_ce) begin
        s1_px  <= x[9:0];
        s1_py  <= y[8:0];
        s1_v   <= adv;
        s1_hit <= hit_v;
    end
end

//------------------------------------------------------------------------------
//  第 2 级：局部坐标 + 控件像素
//------------------------------------------------------------------------------
wire [9:0] g_x1 = rd_geo[37:28];
wire [8:0] g_y1 = rd_geo[27:19];
wire [9:0] g_x0 = rd_geo[18:9];
wire [8:0] g_y0 = rd_geo[8:0];

wire [9:0] lx = s1_px - g_x0;
wire [9:0] ly = {1'b0, s1_py} - {1'b0, g_y0};
wire [9:0] cw = g_x1 - g_x0;
wire [9:0] ch = {1'b0, g_y1} - {1'b0, g_y0};

//---- 位图取色（纯组合）----
//  图标：RGB565，2 像素/字，低半字 = 偶像素 ⇒ 选半字后拼回 888（纯连线）
//  logo：ARGB8888，1 像素/字 ⇒ **整个字就是最终像素**，不选半字、不查表
wire [15:0] bm16 = lx[0] ? icon_word[31:16] : icon_word[15:0];

// RGB565 → RGB888：低位补高位（{R5,R5[4:2]} 等），纯连线、零 LUT
wire [23:0] ic_rgb = {bm16[15:11], bm16[15:13],
                      bm16[10:5],  bm16[10:9],
                      bm16[4:0],   bm16[4:2]};

wire        w_on;
wire [3:0]  w_lv;
wire [23:0] w_rgb;

osd_widget u_w (
    // attr[23:0] = {type[23:20], style[19:16], rad[15:12], mask[11:10], v0[9:0]}
    .wtype  (rd_attr[23:20]),
    .wstyle (rd_attr[19:16]),
    .wrad   (rd_attr[15:12]),
    .wmask  (rd_attr[11:10]),
    .v0     (rd_attr[9:0]),
    .ic_rgb (ic_rgb),
    .lx     (lx),
    .ly     (ly),
    .w      (cw),
    .h      (ch),
    // ★ 2026-10-29：绝对坐标直通（纯切片，零逻辑），给 widget 把 w/h 从关键路径摘掉
    .px     (s1_px),
    .py     (s1_py),
    .gx0    (g_x0),
    .gx1    (g_x1),
    .gy0    (g_y0),
    .gy1    (g_y1),
    .on     (w_on),
    .o_level(w_lv),
    .o_rgb  (w_rgb)
);

// level(0..8) → 8 位 alpha
wire [7:0] a8 = (w_lv == 4'd8) ? 8'd255 : {w_lv, 5'd0};

// ★ 位图未就绪（ROM 还没搬完 / Flash 那片没烧）时，type12/14 整槽不画。
//   放在最后一级与门，不影响前面的命中/寻址时序。
wire is_icon = (rd_attr[23:20] == 4'd12);
wire is_logo1 = (rd_attr[23:20] == 4'd14);
// ★ 两种位图（12/14）都必须被 icon_ok 门住：
//   漏掉 14 的话，"没烧 Flash"时 logo 会被 osd_widget 的 default 分支
//   画成一块不透明的垃圾色（实测 D3 断言 A=255）。
wire is_bmp  = is_icon | is_logo1;
wire draw_on = w_on & s1_hit & s1_v & (icon_ok | ~is_bmp);

// ★★ logo：type 14，**整个 32bit 字就是 {A8,R,G,B}**，直接当像素输出 ★★
//   为什么不能走 osd_widget：它的 o_level 只有 4 位（alpha 只有 0/32/64…/255
//   九档），而 logo 背景要和面板一样是 A=192（75%）、墨色是 255、边缘是
//   192..255 之间的连续值。走 4 位 level 就只剩 192/224/255 三档，
//   抗锯齿的过渡全丢。所以这里绕过 widget，直接把字送出去。
//   【LUT 代价 0】末级本来就是 `draw_on ? {a8,w_rgb} : 0`（2 输入 AND，
//   每比特 1 个 LUT4 且无法两两打包），加一路变 4 输入仍是 1 个 LUT4/bit。
wire logo_draw = s1_hit & s1_v & is_logo1 & icon_ok;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        pix       <= 32'h0000_0000;
        out_valid <= 1'b0;
    end else if (pipe_ce) begin
        pix       <= logo_draw ? icon_word :
                     (draw_on   ? {a8, w_rgb} : 32'h0000_0000);
        out_valid <= s1_v;
    end
end

endmodule
