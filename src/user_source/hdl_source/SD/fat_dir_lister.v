//==============================================================================
//  fat_dir_lister.v —— FAT32 根目录扫描器（流式，占用与文件数无关）
//==============================================================================
//
//  【它干什么】
//  从根目录第一个扇区开始线性扫，把命中项压缩成一张 **4 项文件列表**，
//  供文件页（2×2 四宫格）和主页（滑动窗口）共用。扫完就停，不缓存整个目录。
//
//  【为什么是流式】
//  需求是"1000 个文件和 1 个文件占用一样"。所以只能持有 **N_ITEMS 项**
//  （固定 4），内存与文件数无关。代价是跳页要重扫（45ms@1000 文件，可接受）。
//
//  ★★★ 2026-11-01：动态 BPB（不再有 ROOT_LBA / FAT_LBA / SPC_SHIFT 参数）★★★
//------------------------------------------------------------------------------
//  【为什么必须动态】那三个参数是**这块卡**的布局，换卡 / 重新格式化立刻失效：
//      · 参数错 ⇒ 从错误扇区开始扫 ⇒ 读到一堆垃圾当目录项
//      · 垃圾项里的簇号随便换算 ⇒ 去读卡外地址 ⇒ SD 总线卡死
//    上板现象就是"名字乱码 + 偶尔卡死"。所以改成每次扫描前先解析 BPB。
//
//  【两段式：BPB 不一定在 0 号扇区】
//    ① 读 LBA 0
//       · byte[82..86] == "FAT32" ⇒ superfloppy：0 号扇区**就是** BPB，part_lba = 0
//       · 否则 ⇒ MBR：part_lba = u32le(byte[454..457])（第 1 个分区项），
//                再读 part_lba 号扇区当 BPB
//    ② 从 BPB 算：
//         FAT_LBA  = part_lba + reserved
//         DATA_LBA = part_lba + reserved + nfat × fatsz32
//         ROOT_LBA = DATA_LBA            （FAT32 根目录簇恒为 2）
//
//  【换扇区不用撤 req】`sd_card_sec_read_write` 每次进 S_WAIT_READ_WRITE 都重新
//    锁存当前地址，所以只要 req 保持高、地址换成下一个即可（见 PLAN §7H）。
//
//  【非法 BPB 的兜底】spc 不是 2 的幂 / nfat=0 / fatsz=0 / 算出 0
//    ⇒ 落到 FB_* 常量（不是"什么都不做"：宁可显示旧卡的内容，也不要卡死）。
//
//  【★ 变长移位不用 barrel ★】
//    `(clus-2) << (9 + spc_shift)` 里 spc_shift 是运行时值，并行 barrel 要 ~96 LUT4。
//    但**一条目录项有 32 拍**：clus 在 byte27 就收全了，之后还有一整条目录项
//    （下一条的 32 拍）可以用 ⇒ 用 32 位 sh_acc 逐位左移，1 位/拍，最多 7 拍。
//    移位没完成前先不写 meta_off（`slot_valid` 与 meta_off 同拍写），
//    读者只认 slot_valid ⇒ 不会看到"偏移还是半成品"的槽。
//
//  【目录项格式（32 字节）】
//      0x00        名字前 8 字符
//      0x08        扩展名 3 字符
//      0x0B        属性（0x0F=LFN, 0x08=卷标, 0x10=目录, 0x02=隐藏）
//      0x14/0x15   起始簇 **高** 16 位（FstClusHI）
//      0x1A/0x1B   起始簇 **低** 16 位（FstClusLO）
//      0x1C..0x1F  文件大小（字节）
//
//  【★ 写后回退：为什么不要临时 buffer ★】
//  读到 byte 0..10 时还不知道这项是否有效（attr 在 byte 11）。
//  常规做法是缓存 11 字节再决定 —— 88 个 FF，太贵。
//  这里改成"先写进 BRAM，无效就不推进 slot"：
//      · 无效项写的垃圾会被**下一个有效项从头覆盖**（同一个 slot 地址）
//      · 唯一漏洞是"最后一个 slot 写脏后再没有有效项"，用 slot_valid 标志挡住
//  成本 = 4 个 FF，0 个 buffer。
//
//  【跳过规则】
//      0xE5  已删除
//      0x0F  LFN（长文件名碎片）
//      0x08  卷标
//      0x02  隐藏（占位图用，不显示在列表里）
//      0x10  子目录（INCLUDE_DIR=0 时跳过）
//      I_list_all_types=0 时，非 .BMP 的项也一并跳过
//
//  ★★★ 2026-11-01 夜：IMAGES_ONLY 从 parameter 改成输入 I_list_all_types ★★★
//  【为什么必须改成运行时】相册把 I_offset 当成"第几张图"用（album_src 的
//    O_lst_offset = alb_base），而文件页把 I_offset 当成"第几个目录项"用
//    （quad_page_ctrl 的 offset += 4）。两种语义只能靠"非 BMP 项算不算一项"
//    来区分，而原来这个开关是综合期参数 ⇒ 顶层只能选一种：
//      · IMAGES_ONLY=0（文件页要列 wav/txt）⇒ 相册的 base 其实在数**条目**；
//        根目录里混进一个 MUSIC.WAV 之后，窗口 [图2,图3,图4,WAV] 的第 4 格
//        不是图 ⇒ album_src 的 win4=0 ⇒ 滑动窗口提前卡死，第 5 张图永远
//        翻不到（上板实测：D 盘 4 张图 + music.wav + 第 5 张 PixPin 截图）。
//    现在由顶层按 osd_file_page 动态切：相册扫描=0，文件页扫描=1。
//    于是本端口=0 时 seen_cnt 只数 BMP ⇒ offset 语义自然回到"第几张图"，
//    窗口里 4 格全是图 ⇒ win4 正确，滑窗不会被打断。
//
//  【LUT 代价】极性取"列全部类型"（而不是"只要图"）是刻意的：顶层直接接
//    osd_file_page，不需要额外反相器。模块内只多一个 2 输入或 `keep_type`
//    （1 个 LUT4）+ 1 个锁存 FF —— 判定挂到 entry_valid 那一级而不是并进
//    attr_skip，就是为了不把 4 输入或门挤成两级（详见 attr_skip 处的注释）。
//
//  【终止】
//      · 读到 byte0 == 0x00（目录结束）
//      · 收满 offset + N_ITEMS 个有效项
//      · 扫到 MAX_SECS 个扇区（兜底，防目录损坏时无限扫）
//
//==============================================================================
//  ★★★ 2026-11-02：match 模式（按名字找同名 WAV）★★★
//------------------------------------------------------------------------------
//  【干什么】相册勾上"音随图切"后，每次高清装载发起都调一次本模块，让它
//    在根目录里找 **主名 8 字节与当前图片完全相同、扩展名为 .WAV** 的文件，
//    命中就把它的文件首扇区输出去换曲；找不到就什么都不发（原曲继续）。
//
//  【为什么不需要"先定位图片"】
//    目标是直接从 `name_mem[{I_match_rel, 0..7}]` 读出来的 —— 那个槽里
//    就是当前窗口第 rel 张图的 8.3 名（相册扫描按 wr_slot 顺序写，
//    而 rel 正是相册窗口序号）。所以：
//      · 不做 23 位偏移比较器（省 ~9 LUT）
//      · 不做"WAV 排在图片前面要补扫"的相位（名字一开始就知道）
//      · 不做 64 位名字快照寄存器（读口组合读，逐字节比即可，省 64 FF）
//    前提由上层保证：**文件页扫描会覆盖 name_mem**，所以 match 只在
//    !osd_file_page 时发起；且相册窗口一变就重扫，装载又等 alb_ok。
//
//  【为什么必须停槽（want_this 里那个 ~mm_on 不是可选项）】
//    match 要 list_all=1（看得见 wav），于是前 4 个有效项就会填满槽、
//    第 5 项触发 drain —— 扫描在第 4 个文件处结束，后面的 WAV 永远扫不到。
//    `want_this = ... && ~mm_on` 一处同时管住 name_wr / 槽推进 / meta_off：
//    既不污染列表，也顺带保护了 name_mem（否则自己把自己写脏）。
//
//  【命中判据】ei 0..7 逐字节与目标名比（不等就置 mm_bad），ei==11 那一拍
//    `mm_ok <= ~mm_bad && f_wav`；移位器 sh_cnt==0 那拍（byte 34 前后，
//    仍在同一目录项内）把 `data_lba_r + sh_acc` 输出到 O_match_off。
//    ⚠ 命中脉冲可能比 O_done 晚 1~8 拍（移位器要跑完），上层要能收迟到的 hit。
//
//  【读口复用】name_mem 只有一路组合读。这里**改地址 mux**（nm_addr），
//    绝不写第二个 name_mem[...] 读表达式 —— 那会被工具复制成两份 LUTRAM
//    （+40 LUT），把整个优化吃回去。match 期间 OSD 不画文件名（相册页没有
//    文件名），地址被抢走无害。
//
//  【时钟域】sd_card_clk（50MHz），与仲裁器同域。
//==============================================================================

module fat_dir_lister #(
    //--------------------------------------------------------------------------
    //  行为
    //--------------------------------------------------------------------------
    parameter integer N_ITEMS     = 4,          // 一次输出几个（固定 4）
    parameter integer MAX_SECS    = 64,         // 最多扫多少个扇区（兜底）
    parameter         INCLUDE_DIR = 0           // 1 = 子目录也列
)(
    input  wire        clk,
    input  wire        rst,

    //------------------------------------------------------------------
    //  控制
    //------------------------------------------------------------------
    input  wire        I_start,                 // 单拍：开始一次扫描
    input  wire [15:0] I_offset,                // 跳过前 N 个有效项
    // ★ 1 = wav/txt 也列（文件页：offset 语义 = "第几个条目"）
    //   0 = 只列/只数 .BMP（相册：offset 语义 = "第几张图"）
    //   扫描开始时锁存，中途变化不影响本次扫描
    input  wire        I_list_all_types,
    //------------------------------------------------------------------
    //  ★ match 模式（按名字找同名 WAV，见头注释）
    //------------------------------------------------------------------
    input  wire        I_match_en,              // 1 = 本次扫描是 match（与 I_start 同拍有效）
    input  wire [1:0]  I_match_rel,             // 目标名在 name_mem 的槽号（= 相册窗口 rel）
    output reg         O_match_hit,             // 单拍：找到同名 WAV（可能晚于 O_done）
    output reg  [22:0] O_match_off,             // 命中 WAV 的文件首扇区（字节偏移 >> 9）
    // ★ 2026-11-06 字幕：同一次扫描里**同时**找同名 TXT
    //   两个命中各自独立输出；只有 WAV 与 TXT 都命中（或扫到目录末尾）才收工。
    //   ★ 2026-11-07：TXT 的偏移与 WAV 共用 O_match_off（两者同拍写同一个值，
    //     原来的 O_match_txt_off 是 23 位重复寄存器，删掉）。
    output reg         O_match_txt_hit,         // 单拍：找到同名 TXT（偏移见 O_match_off）
    // ★ [NM-BUSY]：名字读口此刻真的被 match 比较器借走（= 内部 mm_rd）。
    //   OSD 用它做 nm_hold，必须和 nm_addr 的选择子逐拍一致；
    //   不能用 O_busy：O_busy 在 S_SCAN 结束那拍就落，而 mm_rd 还要晚一拍。
    output wire        O_nm_busy,
    output reg         O_busy,
    output reg         O_done,                  // 单拍：列表就绪
    output reg  [2:0]  O_valid_cnt,             // 本页实际拿到 0..N_ITEMS
    output reg         O_has_more,              // 后面还有没有

    //------------------------------------------------------------------
    //  名字读口（给 OSD 文本用）
    //------------------------------------------------------------------
    input  wire [1:0]  I_rd_slot,
    input  wire [3:0]  I_rd_idx,
    output wire [7:0]  O_rd_char,
    // ★ 2026-11-05 [NM-PRIO]：名字读口自带的长度/有效位。
    //   OSD 画文件名必须用这一组（按 I_rd_slot 选），不能用下面按 I_ld_slot
    //   选的那一组 —— 那个口在四宫格装载期间归 quad_loader，OSD 会读到别格。
    output wire [3:0]  O_rd_blen,
    output wire [3:0]  O_rd_elen,
    output wire        O_rd_vld,

    //------------------------------------------------------------------
    //  列表内容（给装载用）
    //------------------------------------------------------------------
    input  wire [1:0]  I_ld_slot,               // 读第几项
    output wire [31:0] O_ld_file_off,           // 字节偏移（直接给 bmp_loader）
    output wire [31:0] O_ld_size,               // 文件字节数
    output wire [3:0]  O_ld_namelen,            // 名字总长（0..11，中间不含点）
    output wire [3:0]  O_ld_blen,               // 主名长度（0..8）—— OSD 居中与插点用
    output wire [3:0]  O_ld_elen,               // 扩展名长度（0..3）
    // ★ 四格的"显示字符数"打包（每格 5 位，0..3 格依次放低到高）：
    //   OSD 的 tx_of() 是**一次把 4 行坐标全算出来**的，只给"当前格"的长度
    //   会让后面 3 行用错宽度（实测第 1 行的名字挤到第 0 行中心）。
    output wire [19:0] O_ld_len4,               // 4 × 5 位：每格 blen+elen+点
    output wire        O_ld_vld,                // 当前 I_ld_slot 这一格有没有内容
    output wire [3:0]  O_ld_type,               // 0=其他 1=BMP 2=WAV 3=TXT 4=DIR
    output wire [3:0]  O_slot_valid,            // 哪几项有效

    //------------------------------------------------------------------
    //  ★ 调试 / 测试口：解析出来的 BPB 结果（顶层不接，TB 断言用）
    //------------------------------------------------------------------
    output wire [31:0] O_root_lba,
    output wire [31:0] O_fat_lba,
    output wire [3:0]  O_spc_shift,
    output wire        O_bpb_ok,                // 0 = 用了兜底常量

    //------------------------------------------------------------------
    //  SD 客户口
    //------------------------------------------------------------------
    output reg         O_sd_req,
    output wire [31:0] O_sd_addr,
    input  wire [7:0]  I_sd_data,
    input  wire        I_sd_dv,
    input  wire        I_sd_end
);

//==============================================================================
//  常量
//==============================================================================
localparam [2:0] S_IDLE = 3'd0;
localparam [2:0] S_BPB0 = 3'd1;     // 读 LBA 0（MBR 或 superfloppy BPB）
localparam [2:0] S_BPB1 = 3'd2;     // MBR 分支：读 part_lba 号扇区（BPB）
localparam [2:0] S_CALC = 3'd3;     // 算 DATA_LBA / FAT_LBA / SPC_SHIFT
localparam [2:0] S_SCAN = 3'd4;
localparam [2:0] S_DONE = 3'd5;

localparam [3:0] T_OTHER = 4'd0, T_BMP = 4'd1, T_WAV = 4'd2, T_TXT = 4'd3, T_DIR = 4'd4;

// ---- 非法 BPB 时的兜底**移位量**（只这一项）----
//  原来还兜底了 FAT_LBA / DATA_LBA（2080 / 17224）。这两个常量是"这张卡"的，
//  换卡就是错的，而且在 RTL 里是两台 32 位 2:1 mux（≈32 LUT + 32 FF）。
//  现在 LBA 一律用卡上 BPB 算出来的值 —— 非法 BPB 时 nfat/fatsz 通常为 0，
//  算出来是 part_lba+reserved（一个真实存在的扇区，读得到东西），
//  加上 MAX_SECS 兜底，既不会指向天文数字也不会卡死。
localparam [3:0]  FB_SPC_SH   = 4'd4;

//==============================================================================
//  BPB 捕获寄存器（68 位量级，全部用寄存器：REG 才 55%，BRAM 只剩 9 块）
//==============================================================================
reg  [7:0]  spc_r;              // byte 13
reg  [15:0] reserved_r;         // byte 14..15
reg  [7:0]  nfat_r;             // byte 16
reg  [22:0] fatsz_r;            // byte 36..38（23 位够：c_prod 只用 [22:0]）
reg  [22:0] part_lba_r;         // 分区起始 LBA（superfloppy = 0），只需 23 位
// ★ 不再单独存 mbr_part：S_BPB0 直接把 byte454..457 抓进 part_lba_r，
//   MBR 分支下它就是分区起始；superfloppy 分支在 S_CALC 前清零。
//   省 32 个 FF + 一个 32 位 mux（原来 `part_lba_r <= mbr_part`）。
// ★ "FAT32" 签名也不再存 5 个字节再比：**边收边与**，只留 1 位。
//   省 40 个 FF + 5 个 8 位比较器（≈15 LUT）。
reg         sf_ok;              // 0 号扇区 byte 82..86 是否等于 "FAT32"

reg  [22:0] data_lba_r;         // = ROOT_LBA（根目录簇 = 2），扇区号
reg  [22:0] fat_lba_r;
// ★ 不再存 data_base_r（= data_lba<<9）：它的低 9 位恒 0，存 32 位等于
//   白扔 9 个 FF + 9 位加法器 + 9 位读 mux。偏移一律按"扇区号"算，
//   输出时再补 9 个 0（见 meta_off 的写口）。
// ★ 同理 sh_acc 只留 [22:0]：0 号簇偏移 = (clus-2)<<spc 个**扇区**。
reg  [3:0]  spc_shift_r;
reg         bpb_ok_r;

//==============================================================================
//  BPB → 三个量（纯组合，S_CALC 那一拍采样）
//==============================================================================
wire [7:0]  c_spc    = spc_r;
wire        c_pow2   = (c_spc != 8'd0) && ((c_spc & (c_spc - 8'd1)) == 8'd0);
// log2 优先编码器（合法输入只有 1/2/4/8/16/32/64/128）
wire [3:0]  c_shift  = (c_spc >= 8'd128) ? 4'd7 :
                       (c_spc >= 8'd64)  ? 4'd6 :
                       (c_spc >= 8'd32)  ? 4'd5 :
                       (c_spc >= 8'd16)  ? 4'd4 :
                       (c_spc >= 8'd8)   ? 4'd3 :
                       (c_spc >= 8'd4)   ? 4'd2 :
                       (c_spc >= 8'd2)   ? 4'd1 : 4'd0;
// nfat × fatsz：FAT32 规范允许 1..4 份 FAT，实际卡上**只可能是 1 或 2**
//   （Windows/linux 的 mkfs 从不写 3/4）。这里只认 1/2，省掉一个 32 位加法器
//   （32 位加法 ≈ 16 LUT + 32 条进位链，在这个工程里很贵）。
// ★ 全部收在 23 位：下游只要 23 位扇区号（meta_off 就 23 位），
//   32 位加法的上 9 位是白算的 —— 两台 32 位加法器变 23 位，省 18 LUT。
//   代价：卡上分区起点/文件位置必须在**前 4GB**（字节偏移本来就是 32 位，
//   等价限制），32GB 卡正常格式化后的分区都在 LBA 2048 附近，够用。
wire [22:0] c_prod   = (nfat_r == 8'd1) ? fatsz_r[22:0] :
                       (nfat_r == 8'd2) ? {fatsz_r[21:0], 1'b0} : 23'd0;

// ★ 试过把两台 23 位加法器"串行共用一台"：**不省**。
//   共用一台加法器要加两组 23 位操作数 mux（≈46 LUT），比省下的加法器（23）还贵。
//   这个器件上"并行转串行"只在**操作数已经在寄存器里**时才有意义（对比 fat 里的
//   串行移位器：那里操作数就是 sh_acc 自己，不需要 mux）。
wire [22:0] c_fat_lba  = part_lba_r + {7'd0, reserved_r};
wire [22:0] c_data_lba = c_fat_lba + c_prod;

// 合法性：spc 是 2 的幂、nfat/fatsz 非 0、乘法结果非 0。
// ★ 刻意**不**做 `fatsz < 0x0080_0000` 这类 32 位区间比较：多 30 多个 LUT，
//   而 fatsz 荒谬时算出来的 DATA_LBA 一定会远大于卡容量，读卡自然报错（不卡死）。
wire        c_ok = c_pow2 && (nfat_r != 8'd0) &&
                   (c_prod != 23'd0);   // c_prod≠0 已蕴含 fatsz≠0

wire [22:0] d_lba   = c_data_lba;

wire [3:0]  d_shift = c_pow2 ? c_shift : FB_SPC_SH;

assign O_root_lba   = {9'd0, data_lba_r};   // 调试口仍给 32 位（TB 断言用）
assign O_fat_lba    = {9'd0, fat_lba_r};
assign O_spc_shift  = spc_shift_r;
assign O_bpb_ok     = bpb_ok_r;

//==============================================================================
//  状态
//==============================================================================
reg  [2:0]  st;
// 扇区号只需 23 位（与 part/data LBA 同宽），对外补 0 成 32 位
reg  [22:0] sd_sec;
assign O_sd_addr = {9'd0, sd_sec};
reg         drain;              // 1 = 本扇区收完就停（不能中途撤 req，见下）
// ★ 不再维护 cur_sec：它就是"正在读的扇区号"，等于 O_sd_addr。
//   两份 32 位寄存器 + 两台 32 位加法器（cur_sec+1 和 O_sd_addr<=cur_sec+1）
//   现在合并成一台：O_sd_addr <= O_sd_addr + 1（省 32 FF + 32 LUT）。
reg  [6:0]  sec_left;           // 还可扫几个扇区（MAX_SECS ≤ 127）
reg  [8:0]  byte_cnt;           // 扇区内字节 0..511
// ★ 11 位够用：最多扫 MAX_SECS(64) 个扇区 × 每扇区 16 项 = 1024 项。
reg  [10:0] seen_cnt;           // 已见有效项数（含被 offset 跳过的）
reg  [10:0] offset_r;
// ★ I_list_all_types 在扫描开始时锁存：本次扫描期间语义不变（I_offset 也这么锁）
reg         list_all_r;
// ★ 2026-11-02 match 模式：4 个标志就够（没有相位机、没有两遍扫描）
reg         mm_on;              // 本次扫描是 match（S_IDLE 锁存 I_match_en）
reg  [1:0]  mm_rel;             // 目标名槽号（锁存 I_match_rel）
reg         mm_bad;             // 本项目录项主名与目标不符（ei 0..7 累积）
reg         mm_ok;              // 本项是"同名 WAV"（ei==11 采样，保持到移位器完成）
reg         mm_txt_ok;          // 本项是"同名 TXT"（同上）
// ★ 2026-11-06 字幕：一次扫描同时找 WAV 和 TXT ⇒ 已拿到哪个，各自记一位
reg         mm_got_wav;
reg         mm_got_txt;
// ★ 「后面还有没有」不能靠"收满了就停"来推断 —— 收满 4 个并不代表没有第 5 个。
//   做法：收满之后**继续往下看一项**，见到下一个有效项就置 got_more 并收工；
//   扫到目录结束标记（byte0==0）就说明真没有了。最多多看一个扇区。
reg         got_more;
reg         dir_end;
reg  [2:0]  slot;               // 已填了几个
reg  [1:0]  wr_slot;            // 当前写哪个槽
reg         entry_dead;         // 本项已删
reg         entry_skip;         // 本项要跳过（LFN/卷标/目录/隐藏）
reg         entry_valid;        // attr 判定结果（留到 byte31 提交用）
reg  [3:0]  blen_this;          // 本项主名长度
reg  [3:0]  elen_this;          // 本项扩展名长度
// ★ 扩展名不再存 24 位再比：**边收边与**，只留 3 个标志
//   省 24 个 FF + 3 台 24 位比较器（≈30 LUT），代价是 9 个 8 位比较
reg         f_bmp, f_wav, f_txt;
reg  [27:0] clus;               // 起始簇

// 串行移位器（簇号 → 扇区偏移，见头部说明）
reg  [22:0] sh_acc;
reg  [3:0]  sh_cnt;
reg         sh_busy;
reg  [1:0]  sh_slot;

//==============================================================================
//  4 槽列表（meta 用寄存器，名字用 LUTRAM）
//==============================================================================
// ★ 只存 [31:9]（扇区号）—— 低 9 位恒 0（字节偏移总落在扇区边界上）
reg  [22:0] meta_off  [0:3];
reg  [3:0]  meta_blen [0:3];       // 主名字节数（第一个空格往前的长度，0..8）
reg  [3:0]  meta_elen [0:3];       // 扩展名字节数（0..3）
reg  [3:0]  meta_type [0:3];
// ★ 显示字符数 = blen + elen + (有扩展名？1:0)。原来用 generate 对 4 个槽
//   **各算一遍**（4 台 5 位加法器 + 4 个比较器 ≈ 30 LUT）。现在在 byte31 提交时
//   算一次存下来（4×5 个 FF 比 3 台加法器便宜），输出直接拼，零逻辑。
reg  [4:0]  meta_len  [0:3];
reg  [3:0]  slot_valid;

// 名字：4 槽 × 16 字节（只用 11）= 64 字
// ★ 工具约定写法（照抄 osd_cellmem / sdr_port_sched 的既有约定）：注释式 pragma。
//   TD 只认 `none | dram | bram | bram_144k`（SWUG106 §4.2.6），
//   `(* ram_style="block" *)` 这种**不在合法值表里、会被静默忽略**。
//   读口是**组合读**（见下面 O_rd_char），只能用 dram（SLICE 里的 LUTRAM）。
//   ⚠ 但按 `#reg` 反推（u_fat 444 FF << 512），它**本来就已经是 LUTRAM**了，
//     这条属性是"写对了但没收益"，别指望它省 LUT。
reg [7:0] name_mem [0:63];      /* fehdl force_ram=1, ram_style="dram" */

//==============================================================================
//  条目内偏移
//==============================================================================
wire [4:0] ei = byte_cnt[4:0];          // 0..31
wire [3:0] ni = byte_cnt[3:0];          // 0..15（名字字节 0..10）

// byte27 那一拍：{clus[27:24]+clus[23:16](byte20/21), 当拍到达的 clus[15:8](byte27), clus[7:0](byte26)}
wire [27:0] clus_full = {clus[27:16], I_sd_data, clus[7:0]};

//==============================================================================
//  本项是否可能被收下（预判，用于决定要不要写名字/要不要起移位）
//==============================================================================
wire want_this = (seen_cnt >= offset_r) && (slot < N_ITEMS) && !mm_on;
//                              ★ ~mm_on：match 不填槽、不推进 slot、不写 meta_off、
//                                也不写 name_mem（name_wr 里用了 want_this）。见头注释。

//==============================================================================
//  名字 RAM 写
//==============================================================================
// ★ 2026-11-03【删除项后首字符丢失】`entry_dead` 是寄存器，ei==0 那一拍还是
//   上一条目的值。若上一条是 0xE5 删除项：
//     · 删除项自己的 byte0 看到旧值 0 ⇒ 把 0xE5 写进 name_mem[slot][0]；
//     · 下一条可见项的 byte0 看到旧值 1 ⇒ 真正的首字符写不进去。
//   于是名字第 0 格留下 0xE5，OSD 画成空白，看起来像 "ACC_01.TXT" 少了 A。
//   这里改成"当前条目"的死亡标志：ei==0 直接用当拍数据判，其余拍用寄存器。
wire nm_dead = (ei == 5'd0) ? (I_sd_data == 8'hE5) : entry_dead;
wire name_wr = (st == S_SCAN) && I_sd_dv && (ei < 5'd11) && !nm_dead && want_this;
always @(posedge clk) begin
    if (name_wr)
        name_mem[{wr_slot, ni}] <= I_sd_data;
end

//==============================================================================
//  名字读：**组合读**（LUTRAM）
//==============================================================================
//  OSD 写字符串是逐拍推进的，BRAM 的 1 拍延迟会让字符整体错位一格。
//  44 个 8 位字用 LUTRAM 实现约 44 LUT，换掉时序对齐的麻烦，值得。
//  ★ 2026-11-02：match 期间地址借给比较器（读目标名的第 ni 字节）。
//    ⚠ 全模块**只允许这一个** name_mem 读表达式，写成两条会被综合成两份
//    LUTRAM（+40 LUT）—— 那正是本方案要避免的。
//==============================================================================
// ★ [NM-STICKY]：`mm_on` 只在下一次 I_start 才清 ⇒ 扫完之后读口会一直借给比较器
//   （ni=0 ⇒ 信息条整串变成"首字母重复"，如 AAAAAA.AAA）。不能在 S_DONE 清 mm_on：
//   命中脉冲可能比 O_done 晚 1~8 拍，移位器靠 mm_on 区分"打 O_match_off"和写 meta。
//   ⇒ 只把**地址选择子**收窄到 S_SCAN，并且打一拍（免得在 LUTRAM 地址前多插组合级）。
//   进/出各晚一拍都不会漏比较：第一扇区的数据要几百拍后才到；退出后比较器已不再采样。
reg         mm_rd;
always @(posedge clk or posedge rst) begin
    if (rst) mm_rd <= 1'b0;
    else     mm_rd <= mm_on & (st == S_SCAN);
end
wire [5:0]  nm_addr = mm_rd ? {mm_rel, ni[3:0]} : {I_rd_slot, I_rd_idx};
assign O_rd_char = name_mem[nm_addr];
// [NM-BUSY] 对外就是 mm_rd：只这一个窗口名字读口不属于 OSD。
assign O_nm_busy = mm_rd;

// ★ [NM-PRIO]：与实际读出的字符**同槽**的长度/有效位（各 1 台 4:1 mux）
assign O_rd_blen = meta_blen[I_rd_slot];
assign O_rd_elen = meta_elen[I_rd_slot];
assign O_rd_vld  = slot_valid[I_rd_slot];

//==============================================================================
//  attr 判定（byte 11 那一拍组合）
//==============================================================================
wire [7:0] attr = I_sd_data;
wire is_lfn    = (attr == 8'h0F);
wire is_volume = attr[3] && !attr[4];
wire is_dir    = attr[4];
wire is_hidden = attr[1];
wire is_bmp    = f_bmp;                          // byte8..10 边收边与的结果
wire is_wav    = f_wav;
wire is_txt    = f_txt;

wire attr_skip = is_lfn || is_volume || is_hidden ||
                 (is_dir && !INCLUDE_DIR);

// ★ 相册扫描（list_all_r=0）时，非 .BMP 的项也不收：不占槽、不进 seen_cnt，
//   于是 I_offset 的语义回到"第几张图"，窗口 4 格全是图（win4 不再被 wav 打断）。
//   【为什么放在这里而不是并进 attr_skip】attr_skip 已经是 4 输入或（1 个 LUT4），
//   再塞一项就要多一级或门（+1 LUT4）；挂到 entry_valid 这一级只多这一个 2 输入
//   或（+1 LUT4），后面那个 3 输入与仍能塞进同一个 LUT4。整处改动合计 +1 LUT4。
// ★ match 模式也要让 WAV 成为"有效项"（否则 entry_valid=0 ⇒ 移位器不起、
//   条目全被当垃圾跳过 ⇒ 永远扫不到音频）。所以把 mm_on 并进来，而不是要求
//   调用方配套把 I_list_all_types 拉高 —— 少一条隐式约定，1 个 LUT4 输入。
wire keep_type = list_all_r || is_bmp || mm_on;

//==============================================================================
//  列表输出
//==============================================================================
assign O_ld_file_off = {meta_off[I_ld_slot], 9'd0};
// ★ 文件长度对外恒 0：bmp_loader 的 `file_length` 端口在它内部只写进 `flen`
//   而 `flen` 没有任何读者（已验证：grep flen 只有赋值行）⇒ 白占 4×32 寄存器
//   + 一个 32 位 4:1 读 mux（~40 LUT）。这里输出 0，接口保持兼容。
assign O_ld_size     = 32'd0;
assign O_ld_namelen  = 4'd0;   // 同样没有读者（OSD 用 blen/elen 自己算）
assign O_ld_blen     = meta_blen[I_ld_slot];
assign O_ld_elen     = meta_elen[I_ld_slot];

// 每格显示字符数 = 主名 + 扩展名 + (有扩展名 ? 1 个点 : 0)
//  ★ 无效槽一律按 1 个字符 ⇒ 显示成空格（meta_len 在扫描开始时清零）。
assign O_ld_len4 = {meta_len[3], meta_len[2], meta_len[1], meta_len[0]};

assign O_ld_vld = slot_valid[I_ld_slot];
assign O_ld_type     = meta_type[I_ld_slot];
assign O_slot_valid  = slot_valid;

//==============================================================================
//  主状态机
//==============================================================================
integer k;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        st          <= S_IDLE;
        sec_left    <= 7'd0;
        byte_cnt    <= 9'd0;
        seen_cnt    <= 11'd0;
        offset_r    <= 11'd0;
        list_all_r  <= 1'b0;
        mm_on       <= 1'b0;
        mm_rel      <= 2'd0;
        mm_bad      <= 1'b0;
        mm_ok       <= 1'b0;
        mm_txt_ok   <= 1'b0;
        mm_got_wav  <= 1'b0;
        mm_got_txt  <= 1'b0;
        O_match_hit <= 1'b0;
        O_match_off <= 23'd0;
        O_match_txt_hit <= 1'b0;
        got_more    <= 1'b0;
        dir_end     <= 1'b0;
        slot        <= 3'd0;
        wr_slot     <= 2'd0;
        O_sd_req    <= 1'b0;
        sd_sec      <= 23'd0;
        O_busy      <= 1'b0;
        O_done      <= 1'b0;
        O_valid_cnt <= 3'd0;
        O_has_more  <= 1'b0;
        slot_valid  <= 4'd0;
        entry_dead  <= 1'b0;
        entry_skip  <= 1'b0;
        entry_valid <= 1'b0;
        drain       <= 1'b0;
        f_bmp       <= 1'b0; f_wav <= 1'b0; f_txt <= 1'b0;
        clus        <= 28'd0;
        spc_r       <= 8'd0;
        reserved_r  <= 16'd0;
        nfat_r      <= 8'd0;
        fatsz_r     <= 23'd0;
        part_lba_r  <= 23'd0;
        sf_ok       <= 1'b0;
        data_lba_r  <= 23'd0;
        fat_lba_r   <= 23'd0;
        spc_shift_r <= 4'd0;
        bpb_ok_r    <= 1'b0;
        sh_acc      <= 23'd0;
        sh_cnt      <= 4'd0;
        sh_busy     <= 1'b0;
        sh_slot     <= 2'd0;
        for (k = 0; k < 4; k = k + 1) begin
            meta_off[k]  <= 23'd0;
            meta_blen[k] <= 4'd0;
            meta_elen[k] <= 4'd0;
            meta_type[k] <= 4'd0;
            meta_len[k]  <= 5'd1;
        end
    end else begin
        O_done <= 1'b0;
        O_match_hit <= 1'b0;        // ★ 单拍脉冲（见头注释：可能晚于 O_done）
        O_match_txt_hit <= 1'b0;    // ★ 同上，字幕 TXT 的命中脉冲

        //----------------------------------------------------------------------
        //  串行移位：簇号 → 字节偏移（1 位/拍，最多 7 拍）
        //  ★ 写完 meta_off 才置 slot_valid：读侧不会看到半成品
        //  ★ match 模式：这一拍不写 meta_off/slot_valid（列表零污染），
        //    改成把结果打到 O_match_off；`mm_ok` 在 ei==11 就锁好了，
        //    移位器完成（byte 34 前后）时它还是本项目录项的值。
        //----------------------------------------------------------------------
        if (sh_busy) begin
            if (sh_cnt == 4'd0) begin
                // 字节偏移 = (data_lba + sh_acc) 个扇区，低 9 位补 0
                if (mm_on) begin
                    O_match_off     <= data_lba_r + sh_acc;
                    O_match_hit     <= mm_ok;
                    O_match_txt_hit <= mm_txt_ok;
                end else begin
                    meta_off[sh_slot]   <= data_lba_r + sh_acc;
                    slot_valid[sh_slot] <= 1'b1;
                end
                sh_busy             <= 1'b0;
            end else begin
                sh_acc  <= sh_acc << 1;
                sh_cnt  <= sh_cnt - 4'd1;
            end
        end

        //----------------------------------------------------------------------
        //  BPB 字段捕获（S_BPB0 / S_BPB1 共用一套译码）
        //----------------------------------------------------------------------
        //  ★ 原来 S_BPB0 和 S_BPB1 各有一份一模一样的 9 位 case 树（13/14/15/16/
        //    36..39），两份约 48 LUT。字段在这两个状态里偏移完全相同，合并成一份。
        //    （0 号扇区特有的 byte 82..86 / 454..457 仍在 S_BPB0 里单独收。）
        if (I_sd_dv && ((st == S_BPB0) || (st == S_BPB1))) begin
            case (byte_cnt)
                9'd13:  spc_r            <= I_sd_data;
                9'd14:  reserved_r[7:0]  <= I_sd_data;
                9'd15:  reserved_r[15:8] <= I_sd_data;
                9'd16:  nfat_r           <= I_sd_data;
                9'd36:  fatsz_r[7:0]     <= I_sd_data;
                9'd37:  fatsz_r[15:8]    <= I_sd_data;
                9'd38:  fatsz_r[22:16]   <= I_sd_data[6:0];
                default: ;
            endcase
        end

        case (st)
        //----------------------------------------------------------------------
        S_IDLE: begin
            O_busy <= 1'b0;
            if (I_start) begin
                sec_left   <= MAX_SECS;
                byte_cnt   <= 9'd0;
                seen_cnt   <= 11'd0;
                offset_r   <= I_offset[10:0];
                list_all_r <= I_list_all_types;
                mm_on      <= I_match_en;       // ★ match 模式开关（本次扫描有效）
                mm_rel     <= I_match_rel;
                mm_bad     <= 1'b0;
                mm_ok      <= 1'b0;
                mm_txt_ok  <= 1'b0;
                mm_got_wav <= 1'b0;
                mm_got_txt <= 1'b0;
                got_more   <= 1'b0;
                dir_end    <= 1'b0;
                slot       <= 3'd0;
                wr_slot    <= 2'd0;
                // ★ match 扫描**不碰**列表：slot_valid 留着，相册/文件页那 4 格
                //   的偏移仍然可读（否则匹配期间 quad_loader 会看到"空格子"）。
                if (!I_match_en) slot_valid <= 4'd0;
                // 空槽显示 1 个字符（空格）：扫描开始时清掉上一次的残值
                meta_len[0] <= 5'd1; meta_len[1] <= 5'd1;
                meta_len[2] <= 5'd1; meta_len[3] <= 5'd1;
                drain      <= 1'b0;
                sh_busy    <= 1'b0;
                sf_ok      <= 1'b0;
                O_busy     <= 1'b1;
                O_sd_req   <= 1'b1;
                sd_sec     <= 23'd0;        // ① 先读 0 号扇区
                st         <= S_BPB0;
            end
        end

        //----------------------------------------------------------------------
        //  S_BPB0：收 0 号扇区。可能本身就是 BPB（superfloppy），也可能是 MBR。
        //          这里只收"0 号扇区特有"的两组：FAT32 签名 + 分区项。
        //----------------------------------------------------------------------
        S_BPB0: begin
            if (I_sd_dv) begin
                case (byte_cnt)
                    9'd82:  sf_ok <= (I_sd_data == "F");
                    9'd83:  sf_ok <= sf_ok && (I_sd_data == "A");
                    9'd84:  sf_ok <= sf_ok && (I_sd_data == "T");
                    9'd85:  sf_ok <= sf_ok && (I_sd_data == "3");
                    9'd86:  sf_ok <= sf_ok && (I_sd_data == "2");
                    // ★ 只留 23 位（前 4GB 内的分区）：byte454/455 全收，
                    //   byte456 只取低 7 位，byte457 不要（+2^31 的卡不适用）。
                    9'd454: part_lba_r[7:0]   <= I_sd_data;
                    9'd455: part_lba_r[15:8]  <= I_sd_data;
                    9'd456: part_lba_r[22:16] <= I_sd_data[6:0];
                    default: ;
                endcase
                byte_cnt <= byte_cnt + 9'd1;
            end
            if (I_sd_end) begin
                byte_cnt <= 9'd0;
                if (sf_ok) begin
                    // superfloppy：0 号扇区就是 BPB（part_lba 必须是 0 —— 上面
                    // 那段 454..457 抓的是 BPB 里的垃圾，这里覆盖掉）
                    part_lba_r <= 23'd0;
                    st         <= S_CALC;
                end else begin
                    // MBR：part_lba_r 已经是 byte454..457 抓到的分区起始，
                    //      直接拿它当地址去读 BPB
                    sd_sec     <= part_lba_r;
                    st         <= S_BPB1;
                end
            end
        end

        //----------------------------------------------------------------------
        //  S_BPB1：收 BPB（part_lba 号扇区）。字段捕获在公共块里。
        //----------------------------------------------------------------------
        S_BPB1: begin
            if (I_sd_dv) byte_cnt <= byte_cnt + 9'd1;
            if (I_sd_end) begin
                byte_cnt <= 9'd0;
                st       <= S_CALC;
            end
        end

        //----------------------------------------------------------------------
        //  S_CALC：算三个量，然后把地址切到根目录首扇区
        //----------------------------------------------------------------------
        S_CALC: begin
            data_lba_r  <= d_lba;
            fat_lba_r   <= c_fat_lba;
            spc_shift_r <= d_shift;
            bpb_ok_r    <= c_ok;
            sd_sec      <= d_lba;
            st          <= S_SCAN;
        end

        //----------------------------------------------------------------------
        S_SCAN: begin
            if (I_sd_dv && !drain) begin
                // ---- 条目开始：清临时状态 ----
                if (ei == 5'd0) begin
                    entry_dead <= (I_sd_data == 8'hE5);
                    entry_skip <= (I_sd_data == 8'hE5);
                    blen_this  <= 4'd0;
                    elen_this  <= 4'd0;
                    f_bmp      <= 1'b0;
                    f_wav      <= 1'b0;
                    f_txt      <= 1'b0;
                    clus       <= 28'd0;
                    // ★ match：主名首字节在这里就比掉（0xE5/0x00 天然不等，
                    //   所以显式判一下，别让"目标名恰好等于 0xE5"这种荒谬情况成立）
                    mm_bad     <= (I_sd_data != O_rd_char) || (I_sd_data == 8'hE5);
                    // 目录结束：只能记下来，必须收完当前扇区
                    // （中途撤 req 会让下一次扫描读到上次的残余字节）
                    if (I_sd_data == 8'h00) begin
                        drain   <= 1'b1;
                        dir_end <= 1'b1;
                    end
                end

                // ---- 名字字节：记录长度 ----
                //   8.3 是"主名 8 字节空格补齐 + 扩展名 3 字节空格补齐"，
                //   OSD 显示要在主名末尾插一个 '.'，所以两个长度必须分开记：
                //       "APPLE   BMP"  → blen=5, elen=3 → 显示 "APPLE.BMP"（9 字）
                if (ei < 5'd8 && !nm_dead && I_sd_data != 8'h20)
                    blen_this <= ni + 4'd1;
                if (ei >= 5'd8 && ei < 5'd11 && !nm_dead && I_sd_data != 8'h20)
                    elen_this <= ni - 4'd7;

                // ---- match：主名 8 字节逐字节与目标名比 ----
                //   目标名从 name_mem[{mm_rel, ni}] 组合读出（见 nm_addr 的注释）。
                //   不等就锁死 mm_bad 到下一项开头（ei==0 重新装载）。
                if (mm_on && (ei >= 5'd1) && (ei < 5'd8) && (I_sd_data != O_rd_char))
                    mm_bad <= 1'b1;

                // ---- 扩展名：byte8 起头，byte9/10 继续与 ----
                if (ei == 5'd8) begin
                    f_bmp <= (I_sd_data == "B");
                    f_wav <= (I_sd_data == "W");
                    f_txt <= (I_sd_data == "T");
                end
                if (ei == 5'd9) begin
                    f_bmp <= f_bmp && (I_sd_data == "M");
                    f_wav <= f_wav && (I_sd_data == "A");
                    f_txt <= f_txt && (I_sd_data == "X");
                end
                if (ei == 5'd10) begin
                    f_bmp <= f_bmp && (I_sd_data == "P");
                    f_wav <= f_wav && (I_sd_data == "V");
                    f_txt <= f_txt && (I_sd_data == "T");
                end

                // ---- 起始簇 ----
                //   ★ FAT32 的短名目录项里 **高字在前**：
                //       offset 0x14(20) = FstClusHI 的高/低字节
                //       offset 0x1A(26) = FstClusLO 的高/低字节
                //     （FAT12/16 只有 offset 26 那一个 16 位字段，很容易记反。
                //       真卡实测：_640X4~1.BMP 在 20/21 = 00 00、26/27 = 05 00，
                //       簇号是 5 —— 按"20 当低字"拼出来是 0x50000，直接读到卡外。）
                if (ei == 5'd20) clus[23:16] <= I_sd_data;
                if (ei == 5'd21) clus[27:24] <= I_sd_data[3:0];
                if (ei == 5'd26) clus[7:0]   <= I_sd_data;
                if (ei == 5'd27) clus[15:8]  <= I_sd_data;

                // ---- attr(byte11)：只记录判定，**不提交** ----
                if (ei == 5'd11)
                    entry_valid <= !entry_dead && !attr_skip && keep_type;

                // ---- match：byte11 那一拍 f_wav 已经收全 ⇒ 本项结论定型 ----
                //   ⚠ 不能在这里置 drain：drain 会让 `I_sd_dv && !drain` 立刻失效，
                //     本项 byte27 的移位器就起不来了 ⇒ 拿不到命中项的文件偏移。
                //     提前收工放到 byte31（那时移位器已经起跑）。
                if (ei == 5'd11) begin
                    mm_ok     <= mm_on && !mm_bad && f_wav;
                    mm_txt_ok <= mm_on && !mm_bad && f_txt;
                end

                // ---- byte27：簇号收全 → 起串行移位（9 + spc_shift 位）----
                //   ★ 当拍到达的 byte27 里就有 clus[27:24]，直接拼，不等寄存器
                //   ★ 这里用的 wr_slot 就是本项的槽（byte31 才推进）
                //   ★ match 模式不填槽（want_this=0），但仍要为**每个**有效项起
                //     移位 —— 否则拿不到命中项的文件偏移（代价只是多 7 拍）
                if ((ei == 5'd27) && entry_valid && (want_this || mm_on)) begin
                    // (clus-2) 个簇 = (clus-2)<<spc 个扇区；字节层面的 <<9 留到写口
                    sh_acc  <= (clus_full >= 28'd2)
                               ? (clus_full[22:0] - 23'd2)
                               : 23'd0;
                    sh_cnt  <= spc_shift_r;
                    sh_slot <= wr_slot;
                    sh_busy <= 1'b1;
                end

                // ---- byte31：本项收全，提交（meta_off 由移位器写）----
                if (ei == 5'd31) begin
                    if (entry_valid) begin
                        // ★ 用 want_this（含 ~mm_on）：match 模式不提交、不推进槽。
                        //   原来这里写的是 `slot < N_ITEMS && seen_cnt >= offset_r`
                        //   —— 与 want_this 少一个 ~mm_on，match 时会白填 4 个槽
                        //   并在第 5 项 drain，后面的 WAV 永远扫不到。
                        if (want_this) begin
                            meta_blen[wr_slot] <= blen_this;
                            meta_elen[wr_slot] <= elen_this;
                            // 显示字符数：主名 + 扩展名 + 点（提交时算一次）
                            meta_len[wr_slot]  <= (({1'b0, blen_this} + {1'b0, elen_this} +
                                                    ((elen_this != 4'd0) ? 5'd1 : 5'd0)) == 5'd0)
                                                  ? 5'd1
                                                  : ({1'b0, blen_this} + {1'b0, elen_this} +
                                                     ((elen_this != 4'd0) ? 5'd1 : 5'd0));
                            meta_type[wr_slot] <= is_dir ? T_DIR :
                                                  is_bmp ? T_BMP :
                                                  is_wav ? T_WAV :
                                                  is_txt ? T_TXT : T_OTHER;
                            wr_slot             <= wr_slot + 2'd1;
                            slot                <= slot + 3'd1;
                        end else if (slot >= N_ITEMS) begin
                            // 本页已满，又见到一个有效项 ⇒ 后面确实还有
                            got_more <= 1'b1;
                            drain    <= 1'b1;
                        end
                        seen_cnt <= seen_cnt + 11'd1;
                    end
                    // ★ match 命中：本项 byte27 的移位器已经起跑 ⇒ 现在可以收工了
                    //   （drain 只在扇区末生效，不能中途撤 req）
                    //   ★ 字幕：WAV 与 TXT **都**拿到才收工；只拿到一个就继续扫，
                    //     否则排在后面的另一个永远找不到（TXT 可能排在 WAV 之后）。
                    if (mm_on && entry_valid) begin
                        if ((mm_got_wav | mm_ok) && (mm_got_txt | mm_txt_ok))
                            drain <= 1'b1;
                        if (mm_ok)     mm_got_wav <= 1'b1;
                        if (mm_txt_ok) mm_got_txt <= 1'b1;
                    end
                end

                byte_cnt <= byte_cnt + 9'd1;
            end

            // ---- 扇区结束 ----
            if (I_sd_end) begin
                byte_cnt <= 9'd0;
                sec_left <= sec_left - 7'd1;
                sd_sec   <= sd_sec + 23'd1;   // ★ 一台加法器（原来 cur_sec/地址各一台）

                if (drain || sec_left <= 7'd1) begin
                    O_sd_req    <= 1'b0;
                    O_busy      <= 1'b0;
                    O_done      <= 1'b1;
                    O_valid_cnt <= slot;
                    // 见到过第 N+1 项 ⇒ 有；扫到目录结束 ⇒ 没有；
                    // 被 MAX_SECS 截断 ⇒ 保守认为有（翻页时再确认）。
                    O_has_more  <= got_more | ~dir_end;
                    st          <= S_DONE;
                end
            end
        end

        //----------------------------------------------------------------------
        S_DONE: begin
            O_busy <= 1'b0;
            if (!I_start) st <= S_IDLE;
        end

        default: st <= S_IDLE;
        endcase
    end
end

endmodule
