//==============================================================================
//  osd_widget.v —— 控件像素生成（纯组合）
//==============================================================================
//
//  【铁律】每个控件必须把**自己矩形内的每个像素**都画上（先铺底再画内容）。
//  命中网络是"最高索引独占"，没画到的像素会直接透出底图（不是透出下层控件）。
//  唯一例外是圆角外侧：那里本来就在矩形之外，必须 on=0（否则四个角是方块）。
//
//  【圆角】真正的圆弧，不是 45° 直切。
//  早期版本用 dxm+dym < rad 判角 ⇒ 上板四个角是"斜切的八边形"，很粗糙。
//  现在用 dx²+dy² > rad²：多花两个 4×4 乘法（clamp 到 4 位 ⇒ 8 位积，约 30 LUT），
//  换来正确的圆弧，且在 50MHz 下只有 2 级 LUT。
//
//  【配色】Solid/Panel/控件底 走样式表（可半透明），内容（填充/旋钮/勾/箭头/文字）
//  一律 level 8 = 不透明，保证清晰。
//
//      底   #202020     槽/框内  #141414     边   #808080
//      轨道 #505050     填充    #3080FF     白   #FFFFFF
//
//  局部坐标 (lx,ly) ∈ [0,w)×[0,h)，w/h ≥ 2。
//
//  【省资源的两次合并（2026-10-26，mslice 4957 > 4900）】
//  ① 游标原本是 4 个槽（上下左右 4 条边，因为命中网络"最高索引独占"，
//     中间留洞就会漏底图）。但它底下的"行高亮"cell4 是**同一个矩形**，
//     所以改成 type 11「框」：2px 白边 + 内部 = 样式表颜色（与下层同色
//     ⇒ 逐像素无缝），一个槽顶四个槽 ⇒ NSLOT 24 → 15，省 9 路命中比较器。
//     前提是游标边框与行控件/文字/图标在几何上**不相交**（已逐项核对：
//     行控件 y = row-4..row+20，游标上下边在 row-6..row-4 / row+20..row+22；
//     图标 x = 32..64 y = 24+80k..56+80k，导航游标四边在它外面一圈）。
//  ② v0 从 14 位收到 10 位（填充 0..287 只需 9 位、数值 0..100 只需 7 位）；
//     删掉从未被本模块引用的 wthick（2 位）。
//==============================================================================
module osd_widget (
    input  wire [3:0]  wtype,
    input  wire [3:0]  wstyle,
    input  wire [3:0]  wrad,
    // ★ 圆角外侧该"继承"哪一层（2 位，因为现在有三层可能）：
    //   0 = 面板/左栏（0E0E0E @75%）—— 游标、图标、行控件都属于这种
    //   1 = 全屏遮罩（000000 @50%）  —— 只有页面板 cell3 是直接坐在遮罩上的
    //   2 = 弹层底板（161616 @100%）—— 下拉选项行坐在弹层 Panel 上
    //   ★ 为什么要三位：OSD 每个像素**只合成一层**（ps_merge_alpha 只叠一次），
    //     所以"圆角外侧"必须手工填成下面那层看起来的颜色。填错就会露出原图
    //     （上板现象：下拉选项行四个角有小缺口、缺口里透出底图发绿）。
    input  wire [1:0]  wmask,
    input  wire [9:0]  v0,
    // ★ 图标像素（type 12 专用，由 osd_gfx 在拍 1 组合算好：字节/半字节/调色板）
    input  wire [23:0] ic_rgb,
    input  wire [9:0]  lx,
    input  wire [9:0]  ly,
    input  wire [9:0]  w,
    input  wire [9:0]  h,
    // ★★ 2026-10-29：绝对坐标（都是 rd_geo / 坐标寄存器的**纯切片，零逻辑**）★★
    //  【为什么加】phy 最差路径 `rd_geo_reg[11] → pix_reg[11]`：19 级、ADDER=5。
    //    根因是几何链里「先算 w = x1−x0，再算 w−1−lx / w−5 / h>>1」——
    //    这些量用绝对坐标写出来**根本不需要 w**：
    //        w − 1 − lx = x1 − 1 − px          （右边距）
    //        lx ≤ w − 5 ⟺ px ≤ x1 − 5
    //        y0 + (h>>1) = (y0 + y1) >> 1
    //    于是每个关键比较/求绝对值都少串一个加法器。
    input  wire [9:0]  px,
    input  wire [8:0]  py,
    input  wire [9:0]  gx0,
    input  wire [9:0]  gx1,
    input  wire [8:0]  gy0,
    input  wire [8:0]  gy1,

    output reg         on,
    output reg  [3:0]  o_level,
    output reg  [23:0] o_rgb
);

localparam [23:0] C_BOX   = 24'h202020;   // 控件底（配合半透明样式）
localparam [23:0] C_INNER = 24'h141414;   // 框内（下拉/勾选框）
localparam [23:0] C_EDGE  = 24'h808080;
localparam [23:0] C_TRACK = 24'h505050;
localparam [23:0] C_FILL  = 24'h3080FF;
localparam [23:0] C_WHITE = 24'hFFFFFF;
// ★ 文件页光标的主色（原型里的 --accent #4A8CFF）
localparam [23:0] C_ACCENT = 24'h4A8CFF;
localparam [23:0] C_BLACK = 24'h000000;

//------------------------------------------------------------------------------
//  样式（level[27:24] + 颜色[23:0]）
//------------------------------------------------------------------------------
function [27:0] style_of;
    input [3:0] s;
    begin
        case (s)
            4'd1 : style_of = {4'd6, 24'h0E0E0E};   // 左栏/页面板：75%
            4'd2 : style_of = {4'd4, 24'h000000};   // 全屏遮罩：50%
            4'd3 : style_of = {4'd8, 24'hF0F0F0};
            4'd5 : style_of = {4'd8, 24'h5A5A5A};   // 导航图标未选中
            4'd6 : style_of = {4'd8, 24'hFFFFFF};   // 导航图标选中 / 游标
            // ★ 行高亮原来是 404040@50%：OSD 只合成一层，50% 就等于把**原图**
            //   透一半上来 ⇒ 选中行（尤其"关于"页只有 Label、没有控件压住的
            //   标签区）看起来"背景变纯透明、失去遮罩"。
            //   改成 303030@75%：透过程度和页面板(0E0E0E@75%)完全一致，
            //   只是把亮度抬高固定一档 ⇒ 既看得出选中，又保持同样的压暗。
            4'd9 : style_of = {4'd6, 24'h303030};   // 行高亮：75%，亮度 +一档
            // ★★★ 分隔线/标题线必须是**半透明**，不能是 level 8 ★★★
            //   它左右两边（左栏、页面板）都是 style1 = 0E0E0E@75%，颜色随底图变。
            //   而这里是 level 8 不透明 3A3A3A，颜色固定 58 ⇒ 同一列上：
            //     底图亮 → 面板 0.75*14+0.25*255 = 74 > 58  ⇒ 线看着**发暗**
            //     底图暗 → 面板 0.75*14+0.25*20  = 16 < 58  ⇒ 线看着**发亮**
            //   于是沿底图明暗变化的方向，这根线一会儿暗一会儿亮 = 上板"杂像素/
            //   虚线"。仿真里看不出来，是因为测试底图是平滑渐变、很少明暗突变。
            //   改成 level 6（和面板同透过程度）+ 固定亮度差
            //   ⇒ 线永远比面板亮 0.75*(58-14) = 33 阶，任何底图下都均匀。
            4'd10: style_of = {4'd6, 24'h3A3A3A};   // 分隔线/标题线：75%，比面板亮一档
            4'd11: style_of = {4'd6, 24'h202020};   // ★ 控件底：75% 半透明
            4'd12: style_of = {4'd8, 24'h282828};   // ★ 弹层选项（未选中）—— 不透明
            4'd13: style_of = {4'd8, 24'h2F6FD0};   // ★ 弹层选项（选中）
            4'd14: style_of = {4'd8, 24'h161616};   // ★ 弹层底板内部 —— 不透明
            // ★ 2026-11-01：原来这里是 {8, FF0000}（红），全工程没人用。
            //   改成 **level=0（alpha 0 = 完全透明）**，专门给文件页游标用：
            //   游标画类型 11（2px 白边 + 内部填色），文件页把内部填成"透明"，
            //   框里就能透出四宫格的图 —— 不用再加一个控件类型。
            4'd15: style_of = {4'd0, 24'h000000};
            default: style_of = {4'd8, 24'h101010};
        endcase
    end
endfunction

wire [27:0] st  = style_of(wstyle);
wire [3:0]  slv = st[27:24];
wire [23:0] sc  = st[23:0];

//------------------------------------------------------------------------------
//  圆角（真圆弧）
//------------------------------------------------------------------------------
wire       in_rect = (lx < w) && (ly < h);

// ★★ 2026-10-29：四个"到边距离"改成**绝对坐标**算，把 w/h 从关键路径上摘掉 ★★
//   w−1−lx = (x1−x0)−1−(px−x0) = **x1−1−px**      （模 1024 恒成立，无边沿特例）
//   于是 dxm 变成两个**并行**减法器 + 一个 2 选 1，不再"先算 w 再算 w−1−lx"。
//   lxr/lyb 就是右边距/下边距，`edge1` 的右/下边框判据也直接用 `lxr==0 / lyb==0`。
wire [9:0] lxr = gx1 - 10'd1 - px;                  // 右边距 = w−1−lx
wire [8:0] lyb = gy1 - 9'd1 - py;                   // 下边距 = h−1−ly
wire [9:0] dxm = (lx < lxr) ? lx : lxr;
wire [9:0] dym = (ly < {1'b0, lyb}) ? ly : {1'b0, lyb};

// ★★★ 圆角判据的基准是**圆心**，不是角点。
//   错误写法（踩过）：dxm²+dym² > r²  → 角点 (0,0) 满足 0>r² 为假 ⇒ 四个角永远是方的。
//   正确写法：(r-dxm)² + (r-dym)² > r² ⇒ (0,0) 处 2r²>r² 成立 ⇒ 切掉。
//
// ★★★ 2026-10-28：算术 → **查表** ★★★
//  【为什么改】phy 报告（`final_timing.rpt`）：`u_mem/rd_geo → pix_reg` 是
//    **20 级逻辑 / 34.4ns**（Fmax 28.7MHz，要求 50MHz），SWNS −14.784ns，
//    424 个违例端点全在 `u_app/u_osd/u_gfx`。这条链是
//    dxm/dym（减+取小）→ clamp → uu/vv（减）→ 2 个 4×4 乘法 → 加 → 比较
//    → 再加 → 再比较，8 个加法器 + 3 个乘法全串在一起。
//  【怎么改】三个判据只取决于 (rq, dx, dy)，都是 ≤8 的小整数 ⇒ 查表：
//    **一次变址取值 + 几个门**，深度从 20 级降到 ~5 级。
//  【表的语义】表里存**纯算术结果**的带序，**不含 inc**。
//    inc 必须用**未 clamp** 的 dxm/dym 判 —— clamp 到 7 会把 dym=8 当成 dy=7
//    （实测多切一圈角）。表项 2 位，索引缩放就是移位 {idx,1'b0}。
//  【等价性】
//    · 生成器：`tools/gen_corner_lut.py`（内置自检：698896 组 0 不匹配）
//    · 断言：`tb_corner_equiv.v` 穷举 rad 0..15 × w 2..20 × h 2..20 × lx × ly
//      = 698896 组，与规格公式逐位比对；改前改后都 ALL PASS，
//      且把半径上限 8→7 会让它立刻 FAIL（证明断言有效）。
//  【表的组织 v2 = 1024 bit（v1 的 1/4）】
//    v1 用 {rq[3:0],dx,dy} × 3 位 = 4096 bit，两处浪费：
//      · rq 只有 1..8 共 8 个有效值，用 4 位寻址**浪费一半表**；
//      · (cut,arc,arc2) 3 位里绝不会有两位同时为 1，非法组合也占了地方。
//    v2：半径用 **rq-1**（3 位）寻址 ⇒ 512 项；三项折成 **2 位带序**
//      band = 0:none / 1:arc2 / 2:arc+arc2 / 3:cut
//
//  【为什么能折成 4 个状态】三条判据阈值同为 r²，LHS 逐级变大：
//      d2 < (uu+1)²+(vv+1)² < (uu+2)²+(vv+2)²      (inc 区域 uu,vv ≥ 1)
//    LHS 越大越容易越阈 ⇒ 集合关系是 **cut ⊆ arc ⊆ arc2**（★ 方向别搞反，
//    是 arc2 比 arc 更容易成立）。而 arc/arc2 都被 !cut 门控，于是 inc 区域内
//    只剩 4 个合法状态，2 位**无损**：
//      band 0 = 000   band 1 = 001   band 2 = 011   band 3 = 100(cut)
//    解码：cut = &band；arc = band==2；arc2 = !cut && band!=0。
//    （踩过：把 arc2 解码写成 band==1，漏掉 band==2，96812 组不匹配 ——
//      靠生成器的等价自检当场抓住。）
//
//  【等价性】
//    · 生成器：`tools/gen_corner_lut.py`（内置三重自检：698896 组 0 不匹配 +
//      inc 区域合法状态恰好 4 种 + band 递增）
//    · 断言：`tb_corner_equiv.v` 穷举 rad 0..15 × w 2..20 × h 2..20 × lx × ly
//      = 698896 组，与规格公式逐位比对；v1/v2 都 ALL PASS，
//      且把半径上限 8→7 会让它立刻 FAIL（证明断言有效）。
//  【代价】表 1024 bit ≈ 32 LUT5 + 变址选择 ≈ 10 LUT5 ≈ **42 LUT5**；
//    换掉 2 个 clamp(12) + 2 个减法(8) + 2 个 4×4 乘法(32) + rq²(16) + 加法(8)
//    + 比较(8) ≈ 84 LUT5 ⇒ **净省约 40 LUT5**。
//    深度：dxm/dym(2) + clamp(1) + 表选择(4) + 门(1) ≈ **8 级**（原 20 级）。
//    **延迟 0 拍变化** ⇒ osd_top 的 4 拍（gfx 2 + 文字 2 + 合并 1）不变。
//------------------------------------------------------------------------------
localparam [1023:0] CORN_LUT = {
    256'h001b005b006b016f15af5abfabffffff001a001b005b016b05af56bfaaffbfff,
    256'h0017001a001b005b016b55af6abfefff000b0017001a001b005b156b9aaffbff,
    256'hc01f000b0017001a005b456be6affeffffffff3ff00bf017c01af15bf9abffbf,
    256'hffffffffffffffcfff17fc5aff6bffeffffffffffffffffffffffff7ffdafffb };

wire [3:0] rq  = (wrad > 4'd8) ? 4'd8 : wrad;        // 半径上限 8
wire [2:0] dx3 = (dxm < 10'd8) ? dxm[2:0] : 3'd7;    // 表内只用 0..7
wire [2:0] dy3 = (dym < 10'd8) ? dym[2:0] : 3'd7;

// ★ 半径用 rq-1 寻址（rq=1..8 → 0..7）。rq=8 时 rq[2:0]=000，减 1 自然回绕成 111
//   ⇒ 不需要额外的"等于 8"判断。rq=0 时地址无意义（incorn 已为假）。
wire [2:0] r3  = rq[2:0] - 3'd1;

wire       incorn = (rq != 4'd0) && (dxm < {6'd0, rq}) && (dym < {6'd0, rq});
wire [8:0] cidx  = {r3, dx3, dy3};                  // 512 项
wire [9:0] cbit  = {cidx, 1'b0};                    // 2 位/项 ⇒ 移位
wire [1:0] cband = CORN_LUT[cbit +: 2];

wire       cut    = incorn && (&cband);              // 圆外侧 ⇒ 不画
wire       arc    = incorn &&  cband[1] && !cband[0]; // 贴圆弧的那 1px
// ★ arc2 是 arc 的**超集**（阈值同为 r²，LHS 更大）⇒ band 1 和 band 2 都算 arc2
wire       arc2   = incorn && !(&cband) && (|cband);

// 1px 边框（直边 + 圆弧）
// ★ 右/下边框改用边距判据：lx == w−1 ⟺ lxr == 0，ly == h−1 ⟺ lyb == 0
//   （省掉 w−1 / h−1 两个减法器，而且 dxm/dym 已经算过同样的量）
wire       edge1 = (lx == 10'd0) || (ly == 10'd0) ||
                   (lxr == 10'd0) || (lyb == 9'd0) || arc;
// ★ 这里原本写的是 `arc || arc2`。因为 arc ⊆ arc2（tb_corner_equiv 穷举证明），
//   arc 是冗余项，删掉不影响任何一层像素，少一个或门。
wire       edge2 = (lx < 10'd2) || (ly < 10'd2) ||
                   (lxr < 10'd2) || (lyb < 9'd2) || arc2;

//------------------------------------------------------------------------------
//  内容几何
//------------------------------------------------------------------------------
//  cy = h>>1 = (y1−y0)>>1；绝对坐标下就是中线的 y：**cya = (y0+y1)>>1**
//  （floor 除法下 y0+((y1−y0)>>1) ≡ (y0+y1)>>1，见 tb_widget_equiv 穷举）
wire [9:0] cy  = h >> 1;                       // 仍给 ay（下拉箭头）用
wire [9:0] cya = ({1'b0, gy0} + {1'b0, gy1}) >> 1;

// 轨道：竖直居中 4px，左右各留 4px 不压边框
//   右端 lx ≤ w−5 ⟺ **px ≤ x1−5**（不再先算 w−5）
wire [9:0] tx0 = 10'd4;
wire [9:0] hix = (w > 10'd9) ? (gx1 - 10'd5) : (gx0 + 10'd4);   // 轨道右端（绝对）
wire [9:0] lox = gx0 + 10'd4;                                    // 轨道左端（绝对）
wire       trk = ((ly + 10'd2) >= cy) && (ly < (cy + 10'd2)) &&
                 (px >= lox) && (px <= hix);

// 填充：轨道左端 → v0（v0 已按控件宽算好，单位像素）
//   vx = clamp(v0, 4, w−5) ⇒ 绝对坐标 vxa = clamp(x0+v0, x0+4, x1−5)
//   ★ x0+v0 与 x1−5 都由寄存器直接算，**互相并行**，不再串在 w 后面
wire [9:0] v0a = gx0 + v0;
wire [9:0] vxa = (v0a > hix) ? hix : ((v0a < lox) ? lox : v0a);
wire       fil = trk && (px <= vxa) && (v0 != 10'd0);   // 0 值时完全不填充

// 旋钮：以 (vxa,cya) 为心的八边形（视觉等同圆点，只用加减比较）
//   kdx = |lx − vx| = |px − vxa|，kdy = |ly − cy| = |py − cya|
// ★ 2026-11-03 [AREA]：kout/kin 只关心"每个距离是否 ≤4 / ≤6 / ≥7"，以及两个都 ≤6 时的和
//   ⇒ 距离饱和到 3 位（≥7 一律记 7）结果逐位相同：
//     · kdx>6 ⟺ sat==7；kdx≤4 ⟺ sat≤4（sat=7 时同为假）
//     · 和的两项判据只在两轴都 ≤6 时才起作用（否则 kout 已真 / kin 已假）⇒ 此时 sat 即真值
//   |d| 不再"先比后减两路"：一台 11 位减法给出有符号差，再从高位判"|d| ≤ 7"。
//   cya ≤ 510（两个 9 位之和右移）⇒ cya[9] 恒 0，与原式的 10 位减法同值。
wire [10:0] kdx_d = {1'b0, px} - {1'b0, vxa};
wire [10:0] kdy_d = {2'b0, py} - {1'b0, cya};
function [2:0] abs_sat7;                         // min(|d|, 7)，d 为 11 位补码
    input [10:0] d;
    begin
        if (!d[10] && (d[9:3] == 7'd0))                     abs_sat7 = d[2:0];
        else if (d[10] && (&d[9:3]) && (d[2:0] != 3'd0))    abs_sat7 = 3'd0 - d[2:0];
        else                                                abs_sat7 = 3'd7;
    end
endfunction
wire [2:0] kdx = abs_sat7(kdx_d);
wire [2:0] kdy = abs_sat7(kdy_d);
wire [3:0] kds = {1'b0, kdx} + {1'b0, kdy};
wire       kout  = (kdx == 3'd7) || (kdy == 3'd7) || (kds > 4'd9);
wire       kin   = (kdx <= 3'd4) && (kdy <= 3'd4) && (kds <= 4'd6);
wire       knb   = !kout && (h >= 10'd10);
wire       knb_e = knb && !kin;

//------------------------------------------------------------------------------
//  勾选框：16×16 方框（2px 边 + 2px 斜切角）
//------------------------------------------------------------------------------
//  ★ 上板反馈"checkbox 左侧距离外框没有间隙"：原来 in_box 判 lx < 16，
//    即方框紧贴控件矩形的左边缘（x = CTRL_X = 320），与 2px 外框之间一点空隙都没有。
//    现在整体右移 BOX_X0 = 12 ⇒ 离控件外框内侧留 10px 呼吸位。
localparam [9:0] BOX_X0 = 10'd12;

wire [9:0] by  = (h > 10'd16) ? ((h - 10'd16) >> 1) : 10'd0;
wire       in_box = (lx >= BOX_X0) && (lx < BOX_X0 + 10'd16) &&
                    (ly >= by) && (ly < (by + 10'd16));
// ★ 2026-11-03 [AREA]：blx/bly 只在 in_box 时被读（box_edge/ck_on 都在 `if (in_box)` 里）
//   ⇒ 那时两者都在 0..15，5 位足够（低 5 位减法模 32 与 10 位结果的低位相同）。
//   下面所有中间量的真值 ≤ 30（blx+bly、blx+15−bly）或 ≥0（17−blx 等），
//   5 位里不溢出 ⇒ 判据逐位相同，进位链从 10 位缩到 5 位。
wire [4:0] blx = lx[4:0] - BOX_X0[4:0];   // 框内局部坐标（仅在 in_box 时有效）
wire [4:0] bly = ly[4:0] - by[4:0];
wire [4:0] bsum = blx + bly;
wire [4:0] bdif = blx + 5'd15 - bly;
wire       box_edge = (blx < 5'd2) || (blx >= 5'd14) || (bly < 5'd2) || (bly >= 5'd14)
                      || (bsum < 5'd3) || (bsum > 5'd26)
                      || (bdif < 5'd3) || (bdif > 5'd26);

// 勾：短下笔 (3,8)→(6,11)，长上笔 (6,11)→(13,4)，各 2px 粗
wire ck1 = (blx >= 5'd3) && (blx <= 5'd6)  && (bly == blx + 5'd5);
wire ck2 = (blx >= 5'd3) && (blx <= 5'd6)  && (bly == blx + 5'd6);
wire ck3 = (blx >= 5'd6) && (blx <= 5'd13) && (bly == 5'd17 - blx);
wire ck4 = (blx >= 5'd6) && (blx <= 5'd13) && (bly == 5'd18 - blx);
wire ck_on = ck1 || ck2 || ck3 || ck4;

//------------------------------------------------------------------------------
//  下拉箭头：右侧实心倒三角 10×6
//------------------------------------------------------------------------------
wire [9:0] ax = lx - ((w > 10'd22) ? (w - 10'd22) : 10'd0);
wire [9:0] ay = ly - ((cy > 10'd3) ? (cy - 10'd3) : 10'd0);
wire       chev = (ax < 10'd10) && (ay < 10'd6) && (ax >= ay) && ((ax + ay) <= 10'd9);

//------------------------------------------------------------------------------
//  主判定（先铺底，再覆盖）
//------------------------------------------------------------------------------
always @(*) begin
    on      = 1'b0;
    o_level = slv;         // 默认取样式表（控件底 = 75% 半透明）
    o_rgb   = C_BLACK;

    // ★★ 圆角外侧必须"继承它下面那一层的外观"，不能 on=0。
    //   命中网络是"最高索引独占"：一旦这个槽被选中，下面那些槽就没机会画了。
    //   若这里 on=0，角上会漏出**原始底图**（用户看到的就是"角上是纯透明"）。
    //   继承目标由 wmask 指定（原来硬编码成面板色，害得页面板的两个上圆角
    //   在遮罩上凸出两块不透明角 ⇒ 上板看到"Page 框上方外部区域实心填充"）。
    if (in_rect && cut) begin
        on = 1'b1;
        case (wmask)
            2'd1:    begin o_level = 4'd4; o_rgb = 24'h000000; end   // 全屏遮罩 50%
            2'd2:    begin o_level = 4'd8; o_rgb = 24'h161616; end   // 弹层底板（不透明）
            // ★ 2'd3：圆角外侧**透明**。信息条面板专用 —— 它下面就是
            //   A/B 图片层，on=0 才能真正"切出圆角"；菜单面板必须继承
            //   遮罩/底板色（0/1/2），否则会透出底图。
            2'd3:    begin on = 1'b0; o_level = 4'd0; o_rgb = 24'h000000; end
            default: begin o_level = 4'd6; o_rgb = 24'h0E0E0E; end   // 面板/左栏 75%
        endcase
    end else if (in_rect) begin
        o_rgb = C_BOX;                       // 统一铺底，不留洞

        // ★★ 上板反馈"面板边框发虚 / 框上方像实心"：这里原来只有部分分支显式
        //   给了 o_level = 8，其余分支把"边框、内部深底、轨道"也套用了样式的
        //   半透明等级（面板 = 75%）。于是 4A4A4A 的边框是 75% 混色，看上去
        //   既不像线也不像面 —— 在暗底上就是一条"发灰的实心横条"。
        //   现在统一：**硬内容（边/深底/轨道/填充/旋钮/勾/箭头）全 8 级不透明**，
        //   只有"底/面板/遮罩/高亮"（sc）沿用样式表给的等级。
        case (wtype)
            4'd0: o_rgb = sc;                                    // Solid
            4'd1: begin                                          // Box：边=sc，内=深底
                if (edge1) begin o_rgb = sc;      o_level = slv;  end
                else       begin o_rgb = C_INNER; o_level = 4'd8; end
            end
            // ★ 2026-11-01：类型 2/8/9/10 全工程**没有任何产生者**（见下），删掉分支。
            //   `cell_type()` 只会产出 {1,4,5,6,7}；`osd_menu` 的 gt 只写 {0,11,13}；
            //   静态槽由 osd_gfx 写 {12,14}。这 4 个 case 分支的 rgb/level mux 输入
            //   是纯浪费（每个 ~10~15 LUT），删掉不影响任何画面。
            4'd4: begin                                          // TrackBar
                if (edge1)      begin o_rgb = C_EDGE;  o_level = 4'd8; end
                else if (knb)   begin o_rgb = knb_e ? 24'h8A8A8A : C_WHITE;
                                        o_level = 4'd8; end
                else if (fil)   begin o_rgb = C_FILL;  o_level = 4'd8; end
                else if (trk)   begin o_rgb = C_TRACK; o_level = 4'd8; end
            end
            4'd5: begin                                          // CheckBox
                if (in_box) begin
                    if (box_edge)                       begin o_rgb = C_EDGE;  o_level = 4'd8; end
                    else if ((v0 != 10'd0) && ck_on)    begin o_rgb = C_FILL;  o_level = 4'd8; end
                    else                                begin o_rgb = C_INNER; o_level = 4'd8; end
                end else if (edge1) begin o_rgb = C_EDGE; o_level = 4'd8; end
            end
            4'd6: begin                                          // ComboBox
                if (edge1)     begin o_rgb = C_EDGE;  o_level = 4'd8; end
                else if (chev) begin o_rgb = C_WHITE; o_level = 4'd8; end
                else                 begin o_rgb = C_INNER; o_level = 4'd8; end
            end
            4'd7: begin                                          // Button
                if (edge1) begin o_rgb = C_EDGE; o_level = 4'd8; end
                else       begin o_rgb = sc;                         end
            end
            4'd11: begin                                         // ★ 游标框（合并 4 槽 → 1 槽）
                // 2px 白边不透明；内部取样式表颜色，且必须**与它下面那层同色**
                //   PAGE 态下面 = 行高亮 style9(404040@50%)  ⇒ gs = 9
                //   NAV  态下面 = 左栏   style1(0E0E0E@75%)  ⇒ gs = 1
                // 这样"一个填满的槽"画出来和原来"高亮 + 4 条空心边"逐像素相同。
                if (edge2) begin o_rgb = C_WHITE; o_level = 4'd8; end
                else       begin o_rgb = sc;      o_level = slv;  end
            end
            // ★ 4'd12：导航图标（32×32，4bpp 覆盖率图 + 调色板）
            //   ic_rgb 已经把"覆盖度 → RGB"算完了（0 级就是底板色 #C0C0C0），
            //   所以这里必须**满铺**：整块矩形都画，一个像素都不能留洞，
            //   否则会透出下面的左栏（命中网络是最高索引独占）。
            4'd12: begin
                o_rgb = ic_rgb;
                o_level = 4'd8;
            end
            // ★ 4'd14：logo（ARGB8888）。实际像素由 osd_gfx 末级直接取 ROM 字
            //   （需要 8 位 alpha，而本模块的 o_level 只有 4 位），
            //   这里给个合法输出只为让 w_on=1，不会被用到。
            4'd14: begin
                o_rgb = ic_rgb;
                o_level = 4'd8;
            end
            // ★ 4'd13：信息条进度条 —— 轨道只有 4px 高、无旋钮（与 8 的区别是
            //   没有外框），用于信息条里那条细条。
            4'd13: begin
                if (fil) begin o_rgb = C_FILL;  o_level = 4'd8; end
                else if (trk) begin o_rgb = C_TRACK; o_level = 4'd8; end
            end
            default: o_rgb = C_BOX;
        endcase
        on = 1'b1;
    end
end

endmodule
