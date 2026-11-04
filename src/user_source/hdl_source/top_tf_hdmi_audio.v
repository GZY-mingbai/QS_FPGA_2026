
//==============================================================================
//  top_tf_hdmi_audio.v  —— 整个工程的顶层模块
//==============================================================================
//
//  【这个文件干什么的】
//  这是整个系统的"总装配图"。它本身不做任何具体的数据处理，只负责三件事：
//    1. 把板卡输入的 50MHz 时钟，通过 3 个 PLL 变成系统需要的各种时钟；
//    2. 定义统一的复位信号 rst_all（芯片复位 或 音频 PLL 没锁定时，系统保持复位）；
//    3. 把所有子模块"例化"进来，并用 wire 把它们的输入输出连起来。
//
//  【数据是怎么流的】
//
//   ┌──────────────────────┐
//   │        App           │  ← 像素源：应用层场景图（模块化拼装）
//   │   （像素从哪来）      │     app_scan_gen + ps_gen_* + ps_offset/scale
//   └──────────┬───────────┘     + ps_merge_* + ps_to_rgb24
//              │ ps_new_frame / ps_data / ps_valid
//              │ ◄── ps_start / busy
//              ▼
//   ┌──────────────────────┐
//   │    frame_writer      │  ← 写帧控制器：写一帧的时序
//   │   （怎么送上去）      │     管握手、写像素、双缓冲、切显示
//   └──────────┬───────────┘
//              │ write_req / write_en / write_data / write_buf_idx
//              ▼
//        frame_read_write（内部有写FIFO、读FIFO）
//                    │
//                    ▼
//                  sdram（帧缓存，双缓冲 BUF0/BUF1）
//                    │ 读像素数据
//                    ▼
//        video_timing_data（产生行/场同步）
//                    │
//                    ▼
//           video_delay（把数据与同步信号对齐）
//                    │ RGB888 + DE/VS
//                    ▼
//      video_rgb_to_axis_640x480（打包成 AXI-Stream）
//                    │
//                    ▼
//      hdmi_1_4b_transmitter_core_wrapper（HDMI 1.4b IP，视频+音频组包 + TMDS 编码）
//                    │ TMDS 10bit × 4 通道
//                    ▼
//           hdmi_phy_wrapper（10:1 并串转换 → LVDS 差分输出）
//                    │
//                    ▼
//                开发板 HDMI_B 接口
//
//  【应用层为什么拆成"像素源 + 写帧控制器"两块】
//  原来是一个大模块 sd_card_bmp 全包了。拆开之后：
//    · 像素源只负责"像素从哪来"，写帧控制器只负责"怎么送上去"，职责清晰
//    · 想换像素来源，只要按同样接口（ps_start / ps_new_frame / ps_data / ps_valid）
//      产像素，frame_writer 一行都不用改
//
//  【现在像素源这一侧进一步模块化了（APP/ 目录）】
//  像素源不再是一个写死的大模块，而是由一组可组合的模块拼出来：
//    · app_scan_gen   产生扫描坐标（整条流水线的时间轴）
//    · ps_gen_*       几何图形生成器（矩形/圆/三角/圆角矩形/多边形/纯色）
//    · ps_offset      坐标偏移（零成本：只改"源看哪个坐标"，不动像素）
//    · ps_scale       坐标缩放（最近邻）
//    · ps_merge_*     合并器（直接叠加 / 透明度叠加）
//    · ps_to_rgb24    叠加到纯黑 + 舍弃 alpha
//  所有模块统一打 1 拍，用一条"坐标延迟链"对齐；
//  场景图总延迟 = 1（首个生成器）+ 合并器级数，这个数要填进 frame_writer 的 PIPE。
//
//  ★ 注意：原来的 tf_bmp_pixel_source.v / bmp_read.v 等文件**没有删**，
//    仍然在工程里（只是暂时没被例化）。后续任务要用随时可以接回来。
//
//  音频支路（2026-10-26 step 0 改造后）：
//      audio_src_sel（音频应用层，在 hdl_source/audio/，用 AUDIO_SRC 选源）
//        ├─ AUDIO_SRC=0（默认）：hdmi_audio_tone_pcm_scale
//        │      在 video_clk 域用分数分频直接产 48kHz/24bit PCM
//        └─ AUDIO_SRC=1（旧）  ：hdmi_audio_tone_i2s_64fs → I2S_receiver
//               I2S 回环，用于继续验证 I2S 时序
//                    │ audio_valid + 24bit 左/右声道
//                    ▼
//         audio_arc_calculate（算 ACR 包的 CTS/N）
//                    │
//                    └──→ 送进 hdmi_1_4b_transmitter_core_wrapper
//
//  外设（2026-11-03，Keypad_ALS_Plan）：
//      4 键 + 五向键(5) + 4 拨码 ── key_debounce9（13 路共享 2ms tick）
//          └─ 4 键 / 五向键 ── key_router（查看器 / 缩放 / 菜单 三种上下文）
//                                  └─► key_press / key_level ─► App / beep（原样）
//      ALS(M10) ── als_adc（audio_mclk：EG_PHY_ADC CH1 + 256×7 BRAM 曲线 + EMA）
//          └─► als_bright（sd_card_clk）── OSD"自动亮度" ──► video_postproc 的亮度
//
//  【建议的阅读顺序】
//  先看端口列表和第 112~140 行（复位与时钟），再看第 150 行开始的 TF 部分，
//  然后是第 198 行开始的视频部分，最后是第 291 行开始的音频部分。
//
//==============================================================================

module top #(
// ---- 音频源选择（2026-10-26 step 0 新增，step 1 增加 2，step 3 增加 3）----
//   0 = PCM 音阶直出（默认）：hdmi_audio_tone_pcm_scale 在 video_clk 域
//       直接产 48kHz/24bit PCM + valid，不经过 I2S 收发，省一对模块。
//   1 = I2S 回环（旧方案）：I2S 发送 → I2S 接收 → PCM，用于验证 I2S 时序。
//   2 = WAV 播放内核（内部 1kHz 测试音）：wav_player 自带测试样本，
//       不需要存储，用来单独验证节拍/位宽/开关。
//   3 = SD 卡 WAV 播放（step 3）：SD → 仲裁器 → FIFO → wav_player，
//       播什么由下面三个 WAV_* 参数决定（tools/wav_fat_offset.py 自动回写）。
//   改这里就能来回切，内部连线不用动；所有源输出接口完全一致。
parameter integer AUDIO_SRC = 3

// ★ 2026-11-02：原来这里有三个 WAV_BASE_SECTOR / WAV_SKIP_BYTES /
//   WAV_TOTAL_WORDS（tools/wav_fat_offset.py 回写的"待机默认曲"）。
//   **已删除**——待机不再硬编码一首曲子。现在的规则：
//     · 开机 / 待机 / 没有图的时候：完全不出声（reader 的 left=0 就不发请求）
//     · 第一张图装载完 → 音随图切找到同名 WAV ⇒ trk_set 指定曲目 ⇒ 才开始播
//     · 那张图没有同名 WAV ⇒ 一直静音，直到某张有配乐的图（或文件页打开 WAV）
//   曲目信息一律来自 FAT 目录（运行期解析 WAV 头得到长度与偏移），
//   换卡/换文件不用重新综合。
)(
    // ---------- 板卡基础信号 ----------
    input                       clk,            // 板卡输入时钟，50MHz（管脚 R7）

    // ---------- 按键 KEY1~KEY4（板上 4 个独立按键，按下 = 0）----------
    //  引脚表：KEY1=A2  KEY2=B2  KEY3=B1  KEY4=C1
    //  ★ 旧版的 rst_n(A2) / key1(B2) / key2(C1) 已按丝印名重排 ★
    //      A2  原来是复位键 → 现在叫 key1（复位已挪到 SW1）
    //      B2  原来叫 key1  → 现在叫 key2（本来就没人用，无影响）
    //      C1  原来叫 key2  → 现在叫 key4（"冻结动画"仍在这个脚上，手感不变）
    //      B1               → 新增 key3
    input                       key1,           // KEY1 → A2
    input                       key2,           // KEY2 → B2
    input                       key3,           // KEY3 → B1
    input                       key4,           // KEY4 → C1（按一下 = 冻结/恢复动画）

    // ---------- 五向键（2026-11-03，外接，按下 = 0，内部上拉）----------
    //  引脚表：OK=F14  UP=G16  LEFT=K15  RIGHT=H15  DOWN=K16
    //  与 KEY1~4 共用一套消抖，按上下文映射到同一条按键总线（见 key_router）。
    input                       bt_ok,
    input                       bt_up,
    input                       bt_lt,
    input                       bt_rt,
    input                       bt_dn,

    // ---------- 乒乓开关 SW1~SW4（拨到 ON = 0）----------
    //  引脚表：SW1=C8  SW2=C7  SW3=C6  SW4=C5
    //  ★ SW1 被"复位"占用（见下方复位小节）⇒ 开关只有 SW2~SW4 可自由用 ★
    input                       sw1,            // SW1 → C8（复位开关：ON = 复位）
    input                       sw2,            // SW2 → C7
    input                       sw3,            // SW3 → C6
    input                       sw4,            // SW4 → C5

    // ---------- 串口 TX（调试功能已移除，脚位保留，固定高电平 = UART 空闲）----------
    //  内部 dbg_uart 实例已删除；端口保留是因为 pin.adc 仍有 D12 约束。
    output                      uart_tx,

    // ---------- 数码管（显示当前诊断页与 16 位读数）----------
    output [5:0]                seg_sel,        // 6 位数码管的"片选"，决定点亮哪一位
    output [7:0]                seg_data,       // 8 段段码（最高位是小数点），决定显示什么数字

    // ---------- HDMI TMDS 差分输出（4 路：3 路数据 + 1 路时钟）----------
    // 为什么这里只有一个 _P（正端）？因为 LVDS 差分对只需要在约束文件里
    // 指定正端管脚，工具会自动把相邻的负端管脚分配好。
    output                      HDMI_CLK_P,     // TMDS 时钟通道
    output                      HDMI_D2_P,      // TMDS 数据通道 2（承载 R + 控制信号）
    output                      HDMI_D1_P,      // TMDS 数据通道 1（承载 G + 控制信号）
    output                      HDMI_D0_P,      // TMDS 数据通道 0（承载 B + 控制信号）

    // ---------- HDMI DDC 通道（I2C，用来读取显示器的 EDID 信息）----------
    output                      HDMI_DDC_SCL,   // DDC 时钟（I2C SCL）
    inout                       HDMI_DDC_SDA,   // DDC 数据（I2C SDA），双向所以用 inout

    // ---------- TF 卡 SPI 接口 ----------
    output                      sd_ncs,         // 片选，低有效
    output                      sd_dclk,        // SPI 时钟
    output                      sd_mosi,        // 主机发、从机收（Master Out Slave In）
    input                       sd_miso,        // 主机收、从机发（Master In Slave Out）

    // ---------- 用户 SPI Flash（W25Q64，字库 font_gb2312.bin 就在这里）----------
    // 【为什么要单列出来】板上有两颗 Flash：
    //   · boot flash（MSPI / 专用配置脚）—— 存 FPGA 配置数据，**绝对不能占用**，
    //     绑上去会破坏配置链路；
    //   · 用户 Flash（就是这 6 根线）—— 普通 IO，可以随便用。
    //   WP#/HOLD# 低有效 → 平时必须为高，所以模块内部恒输出 1，且约束里挂上拉。
    output                      flash_cs_n,     // 片选，低有效   → P8
    output                      flash_sclk,     // SPI 时钟       → M9
    output                      flash_mosi,     // FPGA → Flash   → P7
    input                       flash_miso,     // Flash → FPGA   → N8
    output                      flash_wp_n,     // 写保护，恒 1   → P9
    output                      flash_hold_n    // HOLD，恒 1     → R9
);

//==============================================================================
//  参数定义
//==============================================================================
// Verilog 的 parameter 相当于"编译期常量"，可以在例化时用 #(...) 覆盖。
parameter MEM_DATA_BITS = 32;           // SDRAM 数据位宽（32bit = 4 字节）
parameter ADDR_BITS     = 21;           // SDRAM 地址位宽（2^21 = 2M 个 32bit 字 = 8MB）
parameter BUSRT_BITS    = 10;           // 突发长度位宽
parameter FRAME_PIXELS  = 24'd307200;   // 一帧的像素数 = 640 × 480 = 307200
parameter BUF0_ADDR     = 24'd0;        // 第 0 块缓冲区的起始地址
parameter BUF1_ADDR     = FRAME_PIXELS; // 第 1 块缓冲区的起始地址（紧接着 BUF0）
// ---- 纹理槽 ----
//  ★ build A（隔离实验）：TEX0 从 614400 搬到 1703936 ★
//  【为什么搬】build 9 的分类染色证明：坏像素大多既不是绿也不是青 ⇒
//    读回来的字**来自纹理区之外**。而 614400 紧贴 BUF1 的末尾 614399，
//    地址哪怕只往小偏一点点，就掉进**正在被每帧重写的帧缓存** ⇒
//    读到合成画面（既非绿也非青、而且逐帧变化 = "点会动"）。
//  【搬到哪】1703936（0x1A0000）。原来 614400~1703935 这一整段（约 1.09M 字）
//    变成"从没写过"的区域：
//      · 若错点变成**静止**的杂色 ⇒ 坐实"地址跑到了纹理区外"
//      · 若错点**照样动** ⇒ 数据不是从帧缓存来的（是读回的字本身被改）
//      · 若错点**变成红色** ⇒ 地址只错一点点、仍在纹理区内
//    选值还考虑了：即使地址丢掉最高位（−1048576）也落在 655360，
//    仍在"没写过"的区域里，不会掉回 BUF。
//  ★ 必须与 app_scene_cfg.v 的 img0_base 同步改 ★
//==============================================================================
//  ★★ 2026-10-22 build N：SDRAM 布局重排（中清槽启用，高清槽预留）★★
//==============================================================================
//  【布局】21 位地址空间 = 2,097,152 字
//      BUF0           0          ~ 307,200      帧缓存 0（双缓冲）
//      BUF1           307,200    ~ 614,400      帧缓存 1
//      TEX_MID0       614,400    ~ 691,200      320×240 中清 0  ← 本版层0
//      TEX_MID1       691,200    ~ 768,000      320×240 中清 1  ← 本版层1
//      TEX_MID2       768,000    ~ 844,800      320×240 中清 2（预留：下一张）
//        (空闲)       844,800    ~ 1,703,940    859K 字，将来扩展用
//      TEX_FULL       1,703,940  ~ 2,011,140    640×480 高清（预留，本版不启用）
//      ────────────────────────────────────────────────────────
//      已用 1,152,000 / 2,097,152 ⇒ 余量 45%
//
//  【为什么不再用 1703940 那个"隔离实验"地址】那是 build A 为了把纹理搬离
//  帧缓存 BUF1 末尾做的隔离，问题早已定性并修好。现在帧缓存是双缓冲
//  （0 / 307,200），614,400 之后整段都是空的，正好用来放中清槽。
//
//  【为什么中清槽放在帧缓存后面】紧贴但 4 对齐（614400 % 4 == 0）⇒ 4 字组规则
//  不受影响；且与帧缓存之间无重叠。
//  【回退】TEX0_ADDR/TEX1_ADDR 改回 1703940 / 921600，
//          并把 app_scene_cfg 的 img0_base/img1_base 同步改回。
//==============================================================================
parameter [23:0] TEX0_ADDR = 24'd614400;    // TEX_MID0（层0，320×240）
parameter [23:0] TEX1_ADDR = 24'd691200;    // TEX_MID1（层1，320×240）
// ---- 两张 BMP 在 SD 卡里的位置（字节偏移必须 512 对齐 + 文件字节长度）----
//   ★ 这两个数要按你的卡填 ★ —— 用 tools/sd_fat_offset.py 算：
//        python tools/sd_fat_offset.py            （自动选唯一的可移动盘，需管理员）
//        python tools/sd_fat_offset.py --apply src/user_source/hdl_source/top_tf_hdmi_audio.v
//     它会解析卡上的 FAT 目录表算出物理偏移，并逐字节自检（还会查图片是否碎片）。
//   ★ 下面这几个数是 2026-10-16 在实测卡上量出来的 ★
//     卡：物理盘 2 第 1 分区（起始 LBA 2048 = 1048576 字节）、FAT32、簇 8192 字节
//     （换卡 / 重新格式化 / 重新拷图之后都要重新算！）
//  ★★ 2026-10-19 21:50：两张图换成【网格检测图】，偏移重新实测并写回 ★★
//     图由 tools/gen_grid_bmp.py 生成（640×480 / 24bit / 不压缩 / 自底向上，921654 字节）：
//         grid_red_640x480.bmp    16×16 红色格线，浅灰底
//         grid_blue_640x480.bmp   16×16 蓝色格线，浅灰底
//     完整流程（换图必走一遍，缺一步就会花屏）：
//         python tools/gen_grid_bmp.py
//         copy tools\out\grid_*.bmp  D:\
//         python tools/sd_fat_offset.py --dev \\.\D: --pick grid_red,grid_blue ^
//                --apply src/user_source/hdl_source/top_tf_hdmi_audio.v
//     本次实测（卡上 6 个文件，两张新图都是单段连续、113 簇）：
//         苹果 8843264   西瓜 9768960   logo 10694656   草莓 11620352
//         grid_red 12554240 ← IMG0        grid_blue 13479936 ← IMG1
//
//  ★ 一个踩过的坑：2026-10-18 那次"更正"把 IMG1 改成 9768960，理由是"草莓紧跟苹果"。
//    但后来卡上又插进了西瓜，9768960 实际是**西瓜** —— 注释写着草莓、代码指着西瓜，
//    数码管照样报 E（E 只证明"这是一张合法的 640×480 BMP"，不证明"是你要的那张"）
//    ⇒ 屏幕上是"一张没问题的错图"，看起来像取数错了。
//    ⇒ **每次往卡里加/删文件，都必须重跑 sd_fat_offset.py，别信旧注释。**
//
//  ★★ 偏移过期会怎么表现（花了很久才定位，记在这里）★★
//     装载器是"从偏移开始一个扇区一个扇区线性搬"，它不认识 FAT：
//       · 偏移对不上时**文件头校验多半失败** ⇒ loader 报 err ⇒ **这一槽的纹理根本
//         没写** ⇒ 而顶层 2026-10-16 把 tex_ready 门控删了（见 u_app 处注释：门控会
//         把整条像素流水线锁死成全屏黑）⇒ **图片框显示 SDRAM 里的陈货 = 花屏**。
//       · 它跟着动画在屏幕上走 ⇒ 看起来像"整行彩带 / 整屏明暗闪烁"；
//         而 key2 关掉图片层闪烁立刻消失（因为那个框没了）—— 全是同一个原因。
//     判据：数码管第 3/4 位 = 每槽装载结果（E=成功、F=失败）。**看到 F 就是这一类问题。**
// ★★ 2026-11-01：IMG0/1/2_OFFSET 与 IMGx_LENGTH **已删除** ★★
//   它们是"某一次往卡里拷文件"的绝对字节偏移，换卡/重新格式化立刻失效。
//   相册现在开机扫一次 FAT 根目录，取前 3 个 .BMP（见 album_src.v）——
//   上一段注释里"偏移对不上 ⇒ 文件头校验失败 ⇒ 花屏"这条整类故障随之消失。

//==============================================================================
//  ★★★ 2026-11-01 文件页四宫格：FAT32 ★★★
//==============================================================================
//  【★ 原来的 ROOT_LBA / DATA_LBA / SPC_SHIFT 三个参数已经删掉了 ★】
//  它们是"这块卡"的布局，换卡/重新格式化立刻失效；填错的表现是
//  "文件名乱码 + 偶尔卡死"（从错扇区扫到垃圾项 ⇒ 簇号换算越界 ⇒ 读到卡外，
//  控制器等不到 end ⇒ 总线卡住）。现在改成 `fat_dir_lister` 每次扫描前
//  自己解析 BPB（MBR 两段式），代码里不再有卡相关的常量。
//
//  仍然保留的只有"最多扫几个扇区"这个兜底值：根目录一般 1 个簇（16 扇区），
//  64 个扇区足够，且目录损坏时不至于无限扫。
//------------------------------------------------------------------------------
parameter integer FAT_MAX_SECS  = 64;           // 最多扫多少个扇区（兜底）

//------------------------------------------------------------------------------
//  SDRAM 四宫格槽布局
//------------------------------------------------------------------------------
//  每格装 320×240 RGB565（= 160 字/行 × 240 行 = 38400 字），
//  槽间距 **40960 字**（= 160 行宽 × 256 行），base = QUAD_BASE + slot*40960。
//  【为什么不是 2^16】槽号被折进了 ps_gen_image 的 sy（sy = slot*256 + 2ly+1），
//  于是地址 = sy*160 + x = slot*40960 + ...，槽间距必须是 160*256。这样请求
//  地址上就**不需要再加一个加法器**（那个加法器正好在最差路径的锥里）。
//
//  【为什么一定要 320×240 而不是 160×120】
//  ps_gen_image 的行跨距写死等于 SRC_W（=160 字/行）。装 160×120 的话
//  每行只有 80 字，就得给行补空（dst_stride=160），而 bmp_view 的 cont1
//  连续读快路径要求 stride == 3*win_w ⇒ 补空会让它退化成"每行一次 CMD17"，
//  240 行白付 240 次卡访问。装 320×240 时行宽天然等于 160 字，
//  连续读照常可用，显示侧 2:1 抽样（见 ps_gen_image 的 quad_map）。
//
//  SDRAM 布局：1048576..1179647 原本是空闲区（见上面的 21 位规划），
//  4 槽共 262144 字，不动任何既有区域。
//------------------------------------------------------------------------------
parameter [20:0] QUAD_BASE = 21'd1048576;

//==============================================================================
//  ★ 2026-10-22 图片查看器：SDRAM 布局（21 位 = 2,097,152 字）
//==============================================================================
//  【为什么要重新规划】查看器要同时容纳：
//      · 2 块合成帧缓存（888 时一帧 307,200 字）
//      · 3 张 320×240 RGB565 缩略图（过渡动画的双层素材，必须常驻）
//      · 1 张 640×480 RGB888 高清纹理（1 字/像素）
//
//      0         .. 307,199   BUF0      合成帧缓存 0（888/565 逐帧可切）
//      307,200   .. 614,399   BUF1      合成帧缓存 1
//      614,400   .. 652,799   THUMB0    图片 0 缩略图（160×240 字）
//      652,800   .. 691,199   THUMB1    图片 1 缩略图
//      691,200   .. 729,599   THUMB2    图片 2 缩略图
//      729,600   .. 1,036,799 HD        当前图 640×480 RGB888
//      1,036,800 .. 2,097,151 空闲（50%）
//
//  ★ 四条 4 字对齐约束：0/307200/614400/652800/691200/729600 全是 4 的倍数。
//  ★ 显示读侧没有越界保护（21 位回绕），HD_ADDR + 307,200 = 1,036,800 < 2^21。
//==============================================================================
// BUF0_ADDR / BUF1_ADDR 已在上面的参数区定义（0 / 307200），这里不再重复
parameter [20:0] THUMB0_ADDR = 21'd614400;
parameter [20:0] THUMB1_ADDR = 21'd652800;
parameter [20:0] THUMB2_ADDR = 21'd691200;
parameter [20:0] HD_ADDR     = 21'd729600;
parameter [20:0] THUMB_WORDS = 21'd38400;     // 320×240 RGB565 打包后 160×240 字
parameter [20:0] HD_WORDS    = 21'd307200;    // 640×480，1 字/像素
// ↑ 双缓冲原理：BUF0 装一帧、BUF1 装另一帧。
//   显示 BUF0 的时候往 BUF1 写新图，写完了再切换去显示 BUF1，
//   这样切换图片的瞬间不会出现"半新半旧"的撕裂画面。

//==============================================================================
//  内部信号声明（wire = 连线，用来把模块之间连起来；reg = 寄存器）
//==============================================================================

// ---- SDRAM 状态 ----
wire Sdr_init_done;         // SDRAM 初始化完成标志
wire Sdr_init_ref_vld;      // SDRAM 自刷新请求有效
wire App_ref_req;           // ★ 2026-10-24：即将发出 REF（给调度器让纹理组首避开）
wire Sdr_busy;              // SDRAM 正忙（此时不能发起新的访问）

// ---- 各时钟域时钟 ----
wire sd_card_clk;           // 50MHz，App（像素源/写帧控制器）侧时钟
wire ext_mem_clk;           // 125MHz，SDRAM 侧时钟
wire ext_mem_clk_sft;       // 125MHz、相移 180°，给 SDRAM 控制器采样用
wire video_clk;             // 25MHz，640×480@60Hz 的像素时钟
wire hdmi_5x_clk;           // 125MHz = 25MHz × 5，TMDS 串行化用的 5 倍频时钟

// ---- 视频时序 ----
wire hs;                    // 行同步（Horizontal Sync）
wire vs;                    // 场同步（Vertical Sync）
wire de;                    // 数据有效（Data Enable）：这拍像素是要显示的
wire [23:0] vout_data_raw;  // 从 SDRAM 读出的原始像素（还没做"首图未就绪即黑屏"处理）
wire [23:0] vout_data;      // 最终送给 HDMI 的像素
wire        display_valid;  // 首图提交完成标志：为 0 时强制黑屏

// ---- 显示端画质后处理（video_postproc，详见其文件头）----
// ★ 2026-11-03：后处理移到频谱/字幕之后，最终链路：
//   vout_data_raw → 频谱条 → 动态字幕（+1 拍）→ video_postproc（+3 拍）
//   → 黑屏门控 → vout_data → rgb_to_axis
//   频谱/字幕/菜单统一受画质参数处理；总延迟仍是 1+3=4 拍。
//   黑屏门控必须在后处理【之后】：否则亮度/对比度会把"无画面"抬成灰色而非纯黑。
wire [23:0] pp_out_data;    // 后处理输出的像素（延迟 3 拍，与 pp_out_hs/vs/de 对齐）
wire        pp_out_hs;
wire        pp_out_vs;
wire        pp_out_de;

// ---- 频谱/动态字幕输出（叠加发生在后处理之前）----
wire [23:0] vout_ovl;       // 频谱条叠加后的像素
wire        sub_vs;         // 动态字幕：vs/de/rgb 一起延迟 1 拍
wire        sub_de;
wire [23:0] sub_data;
// 画质参数（sd_card_clk 域，OSD 显示页输出，经 u_app 端口穿出）：
// 默认值 = 全 50 / 风格无 = video_postproc 的恒等直通点；OSD"恢复默认"也回到
// 这组值。跨域同步（tgl 3 拍 + 边沿检测）与 vs 边沿生效都在 postproc 内部。
wire [6:0]  pic_bright;
wire [6:0]  pic_contrast;
wire [6:0]  pic_temp;
wire [6:0]  pic_sat;
wire [1:0]  pic_style;
wire        pic_tgl;
// ★ 2026-11-03 自动亮度：OSD 显示页 item6 CheckBox + 选择后的亮度（→ video_postproc）
wire        osd_auto;
wire [6:0]  pic_bright_eff;
wire        pic_tgl_eff;

// ---- 系统页导出（2026-10-29，sd_card_clk 域，OSD 系统页输出）----
//  语言只影响 OSD 文字，不进音频；音量/静音/循环要跨到 video_clk（见音频段）。
wire        osd_lang;
wire [6:0]  osd_vol;
wire        osd_mute;
wire        osd_loop;
wire        osd_spc;             // 频谱条开关 → spec_bar_ovl（2FF 同步进 video_clk）

// ---- 数码管：只保留驱动（seg_scan），6 个数据口固定全灭（见文件后段）----

// ---- 帧缓存读接口（video_clk 域 ↔ mem_clk 域）----
wire        video_read_req;      // 视频端请求读一帧
wire        video_read_req_ack;  // 帧缓存端应答
wire        video_read_en;       // 每读出一个像素拉高一次
wire [31:0] video_read_data;     // 读出的像素数据（32bit，高 24bit 是 RGB）

// ---- 帧缓存写接口（sd_card_clk 域 ↔ mem_clk 域）----
// 【命名说明】前缀原来叫 sd_card_，因为以前是 SD 卡模块直接驱动的。
// 重构后由应用层 frame_writer 驱动，所以改名为 app_，名副其实。
wire        app_write_req;        // 写请求（保持高直到收到 ack）
wire        app_write_req_ack;    // 写请求应答
wire        app_write_en;         // 写入使能（每像素一拍，一帧共 307200 拍）
wire [31:0] app_write_data;       // 写入的像素数据 {R,G,B,8'h00}
wire        frame_write_finish;   // 整帧写完（在 mem_clk 域的脉冲）
reg         frame_write_toggle_mem; // 把上面的脉冲转成"翻转电平"以便跨时钟域传输

// ---- 显示读 / 显示写 与调度器的握手 ----
wire        fw_rd_en, fw_rd_want;
wire [20:0] fw_rd_addr;
wire        fw_rd_ready, fw_rd_vld;
wire        fw_wr_ready;               // 调度器放行显示写（WR 槽）
wire        fw_wr_busy;                // 写引擎忙（只作观察点）
localparam [3:0] BUILD_TAG = 4'h0;     // 本版 = 0【16色周相 + 真实地址】(TEX_PAT=5, FREEZE_ADDR=0, PAT_CHK=0)
                                       //   看画面即可：绿/青分界干净、无会动的错点 ⇒ 坐实时序。
                                       //   第3位=传输偏移数，第4位=洞/纯垃圾数。

// 写/读 FIFO 水位曾经用来做"告急让路"，现在调度器是固定时隙表，
// 不再需要任何动态让路 ⇒ 下面这三根线只保留端口连接，逻辑未使用。
// （唯一在用的水位是 wr_inflight —— 写 FIFO 的写侧占用，用来做像素源背压。）
wire [9:0]  wr_fifo_used;
wire        wr_fifo_full;
wire [8:0]  rd_buf_used;

//==============================================================================
//  ★ 2026-10-18：写 FIFO 水位 → 像素源背压（修"纯黑"第三因）★
//==============================================================================
//  【为什么必须加】像素源是"1 像素/拍 @50MHz"（50M 字/s），而写口在 16 时隙表里
//  只分到 4 个 WR 槽 = 16 字/64 拍 = 1/4 占空比 @125MHz ≈ 31.25M 字/s。
//  512 深的写 FIFO 会在 ~27µs 内被灌满 ⇒ 溢出丢字 ⇒ 写引擎永远数不满
//  write_len ⇒ write_finish 永不产生 ⇒ display_valid 恒 0 ⇒ 全屏黑（数码管最右位 b）。
//  【为什么能自适应】背压只"卡住生产者"；写口一有空水位就落下来、立刻放行，
//  产出自然被压到写口的平均速率，不需要任何固定降速/缓冲。
//  【为什么要留余量】frame_writer 的 write_en 比取数晚 PIPE=13 拍，松手后还会
//  再灌进 ≤13 个字；阈值 448 留了 64 个字的余量。
//  【为什么阈值远低于 512】wfifo 的水位是 9 位差值，512 时回绕成 0 —— 阈值
//  必须 < 512 才不会被回绕骗到。
//  【为什么不需要跨时钟域】wr_inflight 就是 wfifo 的【写侧】wrusedw，本来就在
//  sd_card_clk（50MHz）域，和 App / frame_writer 同域。
//  【为什么比较要打一拍】水位是 FIFO 内部"两个寄存器相减"的组合结果，直接接进
//  App 的 pixel_ready → ps_valid → frame_writer 会拉出一条长组合路径；打一拍既
//  隔断它，又让 wr_stall 在复位后有确定的 0（比较器见下面 App 例化前）。
//==============================================================================
wire [9:0]  wr_inflight;

// frame_writer 诊断
wire [31:0] fw_rd_data;
wire [1:0]  tex_req_vld, tex_req_ack, tex_pix_wr;
wire [41:0] tex_req_addr;
wire [7:0]  tex_req_mask;   // ★ build F：组掩码 [3:0]=层0 / [7:4]=层1 ★
wire [63:0] tex_pix_data;
                            //   ∴ 花屏不可能来自"多投/少投一个字"这类长度错配

// ---- 写口仲裁：显示写帧 ↔ 纹理装载 ----
wire        fw_wr_en;
wire [20:0] fw_wr_addr;
wire [31:0] fw_wr_din;
wire [3:0]  fw_wr_dm;
wire        ld_wr_vld, ld_wr_ack;
// ★ build 100M-10：AFIFO_AW 4→9，装载写 FIFO 的 wr_used 从 5 位变 10 位。
//   下游 `sdr_port_sched` 只用它判 ">=4"，不需要全量程，所以在这里饱和到 15，
//   避免为了一个水位信号去改调度器的端口宽度和一堆测试台。
wire [9:0]  ld_wr_used_raw;            // 装载写 FIFO 真实字数（0..512）
wire [4:0]  ld_wr_used_b;              // 饱和到 15（给写仲裁器组包 / ld_occ_max）
assign ld_wr_used_b = (ld_wr_used_raw > 10'd15) ? 5'd15 : ld_wr_used_raw[4:0];

//==============================================================================
//  ★ 2026-11-01 P4b：wav / txt 的占位图（程序生成，不读卡）
//==============================================================================
//  【为什么要二选一装载写口】bmp_loader 和 ph_writer 都往 SDRAM 槽里写，
//  同一时刻只能有一个在驱动 {vld,addr,data,used}（SDRAM 只有一个写口）。
//  二选一的条件是 ph_busy：占位图写入占 768µs，期间 bmp_loader 一定空闲
//  （两边的启动都在 quad_loader 里串行，不会重叠）。
//  成本：21+32+1+5 位 2:1 mux ≈ 30 LUT。
//==============================================================================
wire        ph_start, ph_busy, ph_done, ph_wr_vld;
wire [1:0]  ph_kind;
wire [20:0] ph_base, ph_wr_addr;
wire [31:0] ph_wr_data;

wire        sdr_ld_vld  = ph_busy ? ph_wr_vld  : ld_wr_vld;
wire [20:0] sdr_ld_addr = ph_busy ? ph_wr_addr : ld_wr_addr;
wire        ph_black;
// ★ 2026-11-04：ph_black = 纯黑占位（BMP 还没装好的格子）。并进已有的 2 选 1：每位 4 输入，不多占 LUT
wire [31:0] sdr_ld_data = ph_busy ? (ph_wr_data & {32{~ph_black}}) : ld_wr_data;
// ★★ 2026-11-03【占位图永远写不进去 ⇒ 含 wav/txt 的页卡死】★★
//   sdr_port_sched 的装载写口**攒够 4 个字才取**（ld_fifo_ok = used >= 4，一次连弹 4 个 ack）。
//   原来这里给的是常数 1 ⇒ 调度器一个字都不取 ⇒ ph_writer 第一个字就等不到 ack，
//   ph_busy 永远 1 ⇒ quad_loader 停在 S_PH、quad_page_ctrl 停在 S_LOAD：
//   第 2 页不出图、按键翻页被吞（只在 S_SHOW 收翻页）、名字不重排（还是上一页的 4 个），
//   而且 ph_busy 卡住这个二选一 ⇒ bmp_loader 的写也再进不了 SDRAM。
//   占位图是现算的，有 vld 就"要多少有多少" ⇒ 报 15（只看 >=4）；vld=0 时报 0，
//   保证每组 4 个 ack 都落在 ph_writer 身上（38400 = 9600 组，正好整除）。
//   单元台 tb_quad_ph / 旧的 tb_file_page 都是"vld 就 ack"，所以没暴露。
wire [4:0]  sdr_ld_used = ph_busy ? {1'b0, {4{ph_wr_vld}}} : ld_wr_used_b;

// ★ 2026-10-31：rst_all 必须在第一次使用（下面的 u_ph）之前声明，
//   否则 TD 先把它当隐式 wire，再在原声明处报 HDL-7225 CRITICAL-WARNING。赋值仍在下方复位段。
wire rst_all;

// ★ 2026-11-03：ph_writer 改在 ext_mem_clk（100MHz）—— 它的写口 / ack 和
//   bmp_loader 的写口一样直接对 sdr_port_sched（mclk 域）。原来跑在 50MHz：
//   调度器一组连弹 4 拍 ack，50MHz 只能看到一半。
//   控制口（start / busy / done）留给 50MHz 的 quad_loader：两个时钟同一个 PLL、2:1 同相，
//   start 单拍在 100MHz 下是 2 拍（run 后第二拍被忽略）；done 只有 1 个 100MHz 拍，
//   50MHz 可能采不到 ⇒ quad_loader 的 S_PH 以 busy 下降为准（见 quad_loader.v）。
ph_writer u_ph (
    .clk       (ext_mem_clk),
    .rst       (rst_all),
    .I_start   (ph_start),
    .I_kind    (ph_kind),
    .I_base    (ph_base),
    .O_busy    (ph_busy),
    .O_black   (ph_black),
    .O_done    (ph_done),
    .O_wr_vld  (ph_wr_vld),
    .O_wr_addr (ph_wr_addr),
    .O_wr_data (ph_wr_data),
    .I_wr_ack  (ld_wr_ack)
);
wire [20:0] ld_wr_addr;
wire [31:0] ld_wr_data;

// ---- SD 卡装载器 ----
wire        ld_sd_ready, ld_ready, ld_can_load, ld_busy, ld_done, ld_err;
// ★ 2026-11-02【BEEP-ABORT】：与 ld_err 同拍，标记"这次 err 只是被翻页放弃"
wire        ld_err_abort;
wire        ld_wr_settled;      // ★ build 100M-15：仲裁器“装载写已沉降”→ loader 的 L_DRAIN
wire [3:0]  ld_state_code;   // ★ 2026-10-27 诊断恢复
wire [3:0]  ld_err_code;     // ★ 2026-10-27 诊断：粘滞错误码
wire [3:0]  ld_bmp_state;    // ★ 2026-10-27 诊断：bmp_read 状态码
// ★ 2026-10-22 查看器：装载请求改由 App 内部状态机产生
wire        ld_req, ld_fmt;
wire [31:0] ld_off, ld_len;
wire [20:0] ld_base;
wire [1:0]  st_img;
wire [2:0]  st_fx;        // ★ 2026-10-26：当前过渡模式 0..4（fx0~fx4）
wire [3:0]  fps_dbg;               // ★ 帧率/10，显示在数码管最左位（SEL0）
wire [3:0]  dbg_state;             // 状态码（数码管第 5 位）
wire        dbg_osd, dbg_fmt;      // {osd_on, fmt565}（数码管第 6 位）
// ★ 2026-10-26：SDRAM 装载进度 0..F（sd_bmp_loader → sd_card_clk 域）
// ★ 2026-10-27 P1-L7：`ld_prog_s1/s2` 两拍同步链已删除 —— 它唯一的消费者
//   是 `u_seg_ldp`（装载进度译码器），而数码管已经换成音频调试页，
//   那个译码器没有任何消费者。见下方 seg_decoder 段。
wire [3:0]  ld_prog;

// ---- 双缓冲索引 ----
wire [1:0]  write_buf_idx;  // 当前正在写入哪块缓冲区（0=BUF0, 1=BUF1）
wire [1:0]  disp_buf_idx;   // 写侧"已提交待显示"的块号（不再直接接读指针！）

// ---- ★ 场同步换页握手（2026-10-17 修"滚动横缝/撕裂"）★ ----
//  原来 disp_buf_idx 直接接读指针，写侧一提交显示就切；但写口 ~102fps 比显示
//  60fps 快 1.7 倍，写指针会转回来重写"屏幕正在读"的那块 ⇒ 屏幕上一条横缝从
//  下往上滚动（缝两侧是同一动画的两个时刻）。
//  现在：写侧只翻转 commit；显示侧在**自己的场边界**采纳并翻转 adopt；
//        写侧等 adopt 追平 commit 才开下一帧 ⇒ 写侧被节流到 ≤60fps，永不碰
//        显示正在读的那块。两组信号都是慢速电平，跨时钟域打拍即可。
wire [1:0]  disp_rd_idx;        // 显示真正在读的块号（video_clk 域，帧边界才变）
wire        disp_adopt_tgl;     // 显示侧"已换页"翻转电平（video_clk 域）
wire        fw_commit_tgl;      // 写侧"已提交"翻转电平（sd_card_clk 域）
reg  [3:0]  commit_idx_v;       // disp_buf_idx      → video_clk 域（2 级打拍）
reg  [2:0]  commit_tgl_v;       // fw_commit_tgl     → video_clk 域（3 级打拍）
reg  [2:0]  adopt_tgl_v;        // disp_adopt_tgl    → sd_card_clk 域（3 级打拍）
wire [1:0]  disp_commit_idx_v = commit_idx_v[3:2];  // 同步后稳定的提交块号
wire        disp_commit_tgl_v = commit_tgl_v[2];    // 同步后稳定的提交翻转电平
wire        disp_adopt_sync   = adopt_tgl_v[2];     // 同步后稳定的采纳翻转电平

// ---- SDRAM 用户接口 ----
// ★ 2026-10-27 清理：地址规则检查器（rc_rd/wr_mis|jmp|tail）诊断已停用 ⇒ 删声明。

wire App_rd_en;                              // 读使能
wire [ADDR_BITS-1:0] App_rd_addr;            // 读地址
wire Sdr_rd_en;                              // SDRAM 读数据有效
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;        // SDRAM 读出的数据
wire App_wr_en;                              // 写使能
wire [ADDR_BITS-1:0] App_wr_addr;            // 写地址
wire [MEM_DATA_BITS-1:0] App_wr_din;         // 写数据
wire [3:0] App_wr_dm;                        // 写数据掩码（Data Mask），4 个字节各一位

// ---- 视频同步信号的一级缓存（color_bar 直接输出，还没经过 video_delay）----
wire hs_0;
wire vs_0;
wire de_0;

// ---- HDMI 音频相关 ----
//  ★ 2026-10-26 step 0：I2S 三线（bclk/lrck/dout）已经收进 audio_src_sel 内部，
//    顶层不再有这三根线——只有 AUDIO_SRC=1 时它们才存在于那个模块里。
wire        audio_pll_lock;     // 音频 PLL 锁定标志
wire        audio_mclk;         // 12.288MHz 音频主时钟（仅 AUDIO_SRC=1 的 I2S 源用）
wire        audio_valid;        // 音频样本有效（一拍一个采样点）
wire [23:0] audio_left_data;    // 左声道 24bit PCM
wire [23:0] audio_right_data;   // 右声道 24bit PCM
wire        acr_valid;          // ACR 参数有效
wire [19:0] acr_cts;            // ACR 的 CTS 参数
wire [19:0] acr_n;              // ACR 的 N 参数（固定 6144）

// ---- SD 卡总线（2026-10-27 step 3.1：控制器上移 + 扇区级仲裁）----
//  sd_card_top → 仲裁器 → 两个客户（图像装载器 / 音频读取器）
wire        sd_init_done_w;     // 卡初始化完成（仲裁器授权前提）
wire        sd_bus_read;        // 仲裁器 → sd_card_top
wire [31:0] sd_bus_addr;
wire [7:0]  sd_bus_data;        // sd_card_top → 仲裁器
wire        sd_bus_dv;
wire        sd_bus_end;

wire        ld_sd_req;          // 客户 0：图像（sd_bmp_loader）
wire [31:0] ld_sd_addr;
wire [7:0]  ld_sd_data;
wire        ld_sd_dv;
wire        ld_sd_end;

wire        au_sd_req;          // 客户 1：音频（audio_wav_path）
wire [31:0] au_sd_addr;
wire        au_sd_urgent;
wire [7:0]  au_sd_data;
wire        au_sd_dv;
wire        au_sd_end;
wire        sd_arb_timeout;     // 粘滞：仲裁器等 end 超时（不该出现）
wire [1:0]  sd_arb_owner;

//==============================================================================
//  ★ 2026-11-01 文件页四宫格：lister / 页面时序 / 装载调度
//==============================================================================
wire        osd_file_page;      // 1 = 真的停在文件页（菜单开着 && page==2）
wire        osd_fp_w;
wire        quad_own;           // 1 = bmp_loader 归四宫格
wire        quad_ready;         // 1 = 至少一格装好了（可以显示）
wire        quad_flip_nx;       // 单拍：下一页（由 OSD 的光标逻辑产生）
wire        quad_flip_pv;       // 单拍：上一页
wire        fp_gen_w;           // 每装完一页翻转一次（OSD 重排名字用）
wire [1:0]  fp_cur_w;           // 当前光标格
// 本页有效格数直接用 quad_vcnt（声明见下方四宫格段）
wire [15:0] quad_off;           // 当前页的列表偏移
wire        quad_has_more;
wire [2:0]  quad_vcnt;

// lister
wire        lst_start, lst_busy, lst_done, lst_has_more;
wire        lst_nm_busy;        // ★ [NM-BUSY] 名字读口真被 match 借走（= lister 的 mm_rd）
// ★ 2026-11-01：lister 在「文件页」和「相册取偏移」之间时分复用。
//   归属在**发起扫描那一刻**锁存，done/cnt 按归属分发 —— 否则两边的扫描
//   会互相"认领"对方的结果（文件页拿到相册的 4 个槽，名字全错）。
wire        lst_done_fp, lst_done_alb;
wire        qp_lst_start, alb_lst_start, alb_sel;
// ★ 2026-11-02【音随图切】：lister 的**第三个**客户（match 扫描）。
//   归属编码从 1 位 `lst_own_fp` 扩成 2 位 —— 两边共用一个 `O_done` 脉冲，
//   不锁存归属就会互相认领对方的结果（这个坑文件页那边已经踩过一次）。
localparam [1:0] OWN_ALB = 2'd0, OWN_FP = 2'd1, OWN_MM = 2'd2;
reg  [1:0]  lst_own;
reg         mm_go;              // 单拍：给 lister 的 I_start（match 用，always 块里赋值）
wire        mm_win;             // 这一拍 match 的 start 真的被 lister 收下了
wire        lst_match_hit;      // lister 的命中脉冲
wire [22:0] lst_match_off;      // 命中文件首扇区（WAV / TXT 共用）
wire        mm_hit;             // 命中且确认是 match 那次扫描（assign 在下面）
reg         mm_pend;            // 有一次换图还没做过匹配
reg  [1:0]  mm_rel;             // 目标名在 name_mem 的槽号（= hd_img）
reg         mm_live;            // 一次 match 扫描正在飞（mm_win 置，命中/done 清）
reg         mm_stale;           // 飞着的那次目标已过期 ⇒ 丢弃其命中（连击保护）
// ★ [MM-EARLY]：当前这张图已经扫过一次（换图时清）。用来给"装载发起"那条兜底
//   路径去重 —— 提前扫成功过就不必再扫一遍（省一次几十毫秒的 SD 往返）。
reg         mm_seen;
wire        osd_asy;            // 相册页 item6：音随图切开关（sd_card_clk 域，OSD 同域）
wire [1:0]  hd_rel_w;           // App：本次高清装载的窗口格号
// ★ [MM-EARLY]：App 的"换图后一拍：请马上扫一次 match"（音乐/字幕定位）
wire        mm_arm_w;
wire [1:0]  mm_arm_rel;

//==============================================================================
//  ★ 2026-11-06【动态字幕】同名 TXT → 字模装载 → 滚动叠加
//  ★ 2026-11-07：与音随图切共用同一次 match 扫描（同一个 always 块）；旧字幕一直滚到新字幕装好
//==============================================================================
//  链路：高清装载发起（记下目标）→ 装载器空了 → match 扫描（同一次找 WAV / TXT）
//          → TXT 文件变了 ⇒ sub_loader 读 TXT、字模走 Flash
//          → 首批字模装进 back bank → 提交 → dyn_subtitle 换 bank 重新滚
//  门控：菜单没开（dbg_osd=0）就显示；换图时不撤旧字幕（双缓冲，新的装好才换）。
//==============================================================================
wire        lst_match_txt_hit;  // lister：同一次 match 扫描命中的同名 TXT（偏移 = lst_match_off）

// sub_loader ↔ SD 客户3
wire        sub_sd_req;
// ★ 2026-11-07 不打断图片装载：装载器忙时，match 扫描（音乐 / 字幕匹配）和字幕 TXT 读
//   的扇区请求先压住（已经授权的那个扇区照常读完，仲裁器只在空闲时采请求）。
//   相册 / 文件页的目录扫描不受影响（装载本身要等它们）。
// ★ [FAT-QUOTA]：`fat_req_g` 已改由下方的 u_fat_gate（配额）产生，这里不再直接屏蔽。
//   （原式：lst_sd_req & ~((lst_own == OWN_MM) & ld_busy) —— 全屏蔽会让 lister 无上界卡住）
// ★ [MM-EARLY]：字幕读取只让"高清烘焙"、不让缩略图预取 ——
//   提前扫描之后，字幕装的时机正好落在动画里（那时多半在预取），
//   若还按 `!ld_busy` 让路，字幕又会被推到烘焙之后。
wire        txt_req_g = sub_sd_req & ~(ld_busy & ld_fmt);
wire [31:0] sub_sd_addr;
wire [7:0]  sub_sd_data;
wire        sub_sd_dv, sub_sd_end;
// sub_loader ↔ Flash 客户3（经 App → osd_top → flash_arb）
wire        sub_fl_req;
wire [23:0] sub_fl_addr;
wire [15:0] sub_fl_len;
wire        sub_fl_rdy;
wire [7:0]  sub_fl_data;
wire        sub_fl_valid;
wire        sub_fl_done;
// sub_loader ↔ 字形 RAM / dyn_subtitle
wire [10:0] sub_wr_addr;
wire [15:0] sub_wr_data;
wire        sub_wr_we;
wire        sub_commit_tgl;     // 新字幕提交（翻转）
wire        sub_bank;           // 正在显示的 bank
wire [8:0]  sub_len;            // 正在显示的字符数
wire        sub_vs_tgl;
reg         mm_sub_ok;          // 本次 match 扫描的目标没被换掉（字幕用，只在 mm_win 才置回）
reg         txt_found_r;        // 本次扫描见到同名 TXT
reg  [22:0] txt_lba_r;          // 该 TXT 的文件首扇区
reg         sub_start;          // 单拍：TXT 变了 ⇒ 装新字幕
reg         sub_has;            // 当前字幕有没有 TXT（= 送给 sub_loader 的）
reg  [22:0] sub_sec;            // 当前字幕的 TXT 首扇区
wire        sub_show_en;        // → dyn_subtitle.show_en
wire [1:0]  alb_rd_slot;
wire [31:0] alb_off0, alb_off1, alb_off2, alb_off3;
wire        alb_any;            // ★ 2026-11-04：本窗口至少有一张图（原 alb_have[1:0]）
wire [2:0]  alb_nimg;           // ★ [NIMG] 本窗口实际图数（0..4），只给 img_switch 判"有没有下一张"
wire [15:0] alb_base;           // 滑动窗口起点（第几张图）
wire        alb_win4;           // 窗口第 4 项也是图
wire        alb_ready;
// ★ [MM-EARLY] 安全门：换图当拍必须确认"相册这一次不会再重扫"。
//   为什么要连续 3 拍：`alb_ready` 是被动掉下去的（album_src 的 FSM 要 1~2 拍才反应），
//   而滑窗换图会改 base ⇒ 它马上要重扫。若在它掉下去之前就武装，match 会拿**上一窗**
//   的旧名字去匹配 ⇒ 配到上一张的 WAV/TXT（比"晚一点"严重得多）。
//   滑窗那一次识别出来就不提前，退回"高清装载发起时再扫"（原时机，结果正确）。
reg         alb_rdy_d1, alb_rdy_d2;
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        alb_rdy_d1 <= 1'b0;
        alb_rdy_d2 <= 1'b0;
    end else begin
        alb_rdy_d1 <= alb_ready;
        alb_rdy_d2 <= alb_rdy_d1;
    end
end
wire        mm_arm_ok = alb_ready & alb_rdy_d1 & alb_rdy_d2;
wire [2:0]  lst_cnt;
wire [3:0]  lst_valid;
wire [1:0]  lst_rd_slot;        // 名字读口：OSD 选槽
wire [3:0]  lst_rd_idx;
wire [7:0]  lst_rd_char;
wire [1:0]  lst_ld_slot;        // 元数据读口：装载时给 loader，否则给 OSD
wire [31:0] lst_off, lst_size;
wire [3:0]  lst_blen, lst_elen, lst_type;
// ★ [NM-PRIO]：OSD 专用的名字长度/有效位（按名字口 lst_rd_slot 选，不经装载口）
wire [3:0]  lst_rd_blen, lst_rd_elen;
wire        lst_rd_vld;
wire [19:0] lst_len4;

//------------------------------------------------------------------------------
//  ★ 2026-11-02【信息条文件名】关闭文件页 ⇒ **先重扫、再放行**
//------------------------------------------------------------------------------
//  【根因】name_mem 是 4 槽 × 16 字节的 8.3 名缓存，被两个消费者共用：
//    相册扫描（只列 BMP，槽 = 窗口第几张图）和文件页扫描（全类型，槽 = 第几个
//    目录项）。文件页一扫，相册那 4 个名字就没了；而 album_src 只在 base 变化
//    时重扫 ⇒ 退出文件页后信息条会读到**文件页的名字**。
//  【做法】固定规则，只有这一条：**关闭文件页（osd_file_page 下降沿）⇒ 让相册
//    重扫一次**。这次重扫是**阻塞式**的：album_src 里清 `tried/O_ready`
//    ⇒ `alb_ok=0` ⇒ `hd_take/tl_take` 全部挂住（装载、换图、缩放恢复都等它），
//    重扫完成 `O_ready` 回 1 才放行。同 base 重扫出来的偏移逐位不变，所以这段
//    等待只是"把名字刷回来"，不改变任何装载数据。
//  【重扫完成】lst_done_alb 这一拍（nm_upd）透给 OSD：文字表在**放行之后**
//    才重写一次。名字不需要任何"可信/不可信"判断，就这一条触发链。
wire        nm_rescan;          // 单拍：文件页退出 ⇒ 请相册重扫一次（并挂住装载）
wire        nm_upd;             // 单拍：相册重扫完成 / match 扫完（OSD 重写一次文件名）
// ★ [NM-HOLD]：match 扫描正在借用名字读口（见 fat_dir_lister 的 mm_rd）
wire        nm_hold;
reg         osd_fp_d;           // osd_file_page 的上一拍（取下降沿）

//==============================================================================
//  ★ 2026-11-02 [SD-GATE]：卡不在位（没插 / 运行中拔掉）⇒ 整机复位 = SW1 拨到 ON
//------------------------------------------------------------------
//  【需求】"卡不在位"要和"SW1 拨到 ON（复位）"表现完全一致：整机不走、屏幕什么
//    都不显示；**卡插回去要能自动恢复**（不用按 SW1）。
//    这样"忘了插卡 / 卡松了"不会被误判成"程序跑飞了"——今天就是这个问题：
//    卡没插紧 ⇒ 全黑但 OSD 能开 ⇒ 看起来像装载逻辑坏了。
//  【做法】把 `sd_ok`（卡在位）并进 rst_all 的或项，见下面【复位源】小节：
//        rst_all = rst_src | ~sd_ok        （sd_ok = sd_init_done & ~sd_lost）
//    相册 / lister / 显示层 / OSD / 音频 / 视频全部用 rst_all ⇒ 一起停在复位。
//  【★ 关键】`sd_card_top u_sd_card` 自己的复位**绝不能接 rst_all**，否则成环：
//      主机在复位 → sd_init_done=0 → rst_all=1 → 主机继续在复位 ⇒ 自锁死，
//      插上卡也恢复不了。因此它走另外两条线：
//        · 正常的硬件复位      ：rst_src
//        · 卡丢失/未初始化后的重试：rst_sd_host（周期脉冲，见下面【复位源】）
//  【历史】(1) 曾只门控应用层 `rst_app = rst_all | ~sd_init_done_w`，实测屏幕全黑，
//    当时被误判成"应用层没跑"于是撤掉；现在反过来 —— 全黑正是要的语义。
//  (2) 曾额外等"第一次目录扫描完成"再放显示层，那是**没有出口的等待**，已撤。
//==============================================================================
wire rst_card = rst_all;        // 4 处沿用旧名（现在都含"卡不在位"门控）
wire rst_app  = rst_all;        // 7 处沿用旧名

wire        lst_vld;            // 当前格（lst_ld_slot 选的）有没有内容
wire        lst_sd_req;
wire [31:0] lst_sd_addr;
wire [7:0]  fat_sd_data;
wire        fat_sd_dv, fat_sd_end;

//==============================================================================
//  ★ [FAT-QUOTA] FAT 客户在"MM 扫描 + 装载器忙"期间的限流（原来是全屏蔽）
//==============================================================================
//  全屏蔽会让 lister 连一个扇区都拿不到，而它的上界是按扇区递减的 ⇒ O_busy 无上界
//  ⇒ 相册永远重扫不了 ⇒ alb_ready 永久 0 ⇒ 之后再也发不出装载（画面定死）。
//  现在改成"每 8 个 IMG 扇区放 1 个 FAT 扇区"，让扫描一定能推进。
//  只挡"高清烘焙"在飞的窗口：那段时间 FAT 客户按 1/8 配额走（保证 lister 能推进）。
//  ★ [MM-EARLY]：原来连缩略图预取也挡 —— 现在提前扫描要在动画里跑，
//    那会儿预取正忙，若照样挡就把扫描又推回烘焙之后了。
wire        fat_block_w = (lst_own == OWN_MM) & ld_busy & ld_fmt;
wire        fat_req_g;
fat_quota_gate #(.N(8)) u_fat_gate (
    .clk       (sd_card_clk),
    .rst       (rst_all),
    .i_block   (fat_block_w),
    .i_req     (lst_sd_req),
    .i_img_end (ld_sd_end),
    .i_fat_end (fat_sd_end),
    .o_req     (fat_req_g)
);

// quad_loader
wire        ql_start, ql_busy, ql_done;
wire [2:0]  ql_loaded;
wire [1:0]  ql_rd_slot;
wire [31:0] ql_off, ql_len;
wire [20:0] ql_base;

// bmp_loader 控制口最终值（相册 / 四宫格 二选一）
wire        bmp_start;
wire        bmp_start_q;            // quad_loader 的启动脉冲
wire [31:0] bmp_off, bmp_len;
wire [20:0] bmp_base;
wire [1:0]  bmp_mode;
wire        bmp_fmt;

// ★ 2026-11-04：翻页/切图可打断装载 + 两遍装载（先黑底） + 文件页"打开"
wire        qp_ld_abort;            // quad_page_ctrl：翻页/离开页面 ⇒ 放弃本页装载
wire        app_ld_abort;           // viewer_ctrl：主页连切 ⇒ 放弃当前高清
wire        ql_filled;              // quad_loader 第 0 遍（黑底/占位）写完
wire [3:0]  ql_bmp;                 // 本页哪几格是 BMP
wire        quad_has_prev;
wire [7:0]  quad_rank;              // 光标格是目录里第几张图
wire        fp_open_w;              // OSD：文件页 KEY2 打开光标格

//------------------------------------------------------------------------------
//  文件页"打开"：借列表元数据口**一拍**读光标那格（类型 / 偏移），按类型分派
//    BMP ⇒ 相册跳到这张（App 关菜单、直接上屏，不播动画）
//    WAV ⇒ 换曲（audio_wav_path 从这个文件头开始重新解析）
//    TXT / 其它 ⇒ 不动
//  借口那一拍 quad_loader 被 I_hold 停住（它的 S_NEXT 正好在读同一个口）。
//------------------------------------------------------------------------------
reg         op_ph;
reg         jump_r;
reg  [7:0]  jump_idx_r;
reg         trk_set_r;
reg  [22:0] trk_sec_r;              // 文件首扇区（文件偏移 >> 9）
// ★ 2026-11-02【音随图切】：当前曲目已经**播完**（单曲、loop=0）。
//   sd_card_clk 域：wav_pass_end 就是这个域的 L 拍脉冲，不用跨域。
//   用途只有一处 —— 判断"同名 WAV 要不要跳过换曲"时区分两种情况：
//     · 还在播（trk_done=0）⇒ 跳过，不打断
//     · 已经播完（trk_done=1）⇒ 照常重放，否则「音乐结束→换图→换新曲」
//       这条闭环在两图共用一首歌时会断（新图无声、pass_done 不再翻转）。
reg         trk_done;
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all)            trk_done <= 1'b0;
    // ★ trk_set_r 优先：换曲当拍 pass_end 可能还是旧曲的尾巴（reader 下一拍
    //   才清 pass_done），此时应记"新一轮开始"而不是"已播完"。
    else if (trk_set_r)     trk_done <= 1'b0;   // 换曲 ⇒ 新一轮开始
    else if (wav_pass_end)  trk_done <= 1'b1;   // 本轮播到尾巴
end
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        op_ph <= 1'b0; jump_r <= 1'b0; jump_idx_r <= 8'd0;
        trk_set_r <= 1'b0; trk_sec_r <= 23'd0;
    end else begin
        jump_r    <= 1'b0;
        trk_set_r <= 1'b0;
        op_ph     <= fp_open_w & ~op_ph;
        if (op_ph && lst_vld) begin
            if (lst_type == 4'd1) begin jump_r    <= 1'b1; jump_idx_r <= quad_rank;     end
            if (lst_type == 4'd2) begin trk_set_r <= 1'b1; trk_sec_r  <= lst_off[31:9]; end
        end
        // ★ 2026-11-02【音随图切】：目录里找到同名 WAV ⇒ 走同一条换曲通路。
        //   放在 op_ph 之后 ⇒ 万一同拍（文件页打开 WAV 的同时命中）以文件页为准。
        //   与上面共用一个 trk_set_r/trk_sec_r 寄存器组，音频侧零改动。
        // ★ 2026-11-02【不打断正在播的那首】：命中曲目 == 当前曲目 且**没播完**
        //   ⇒ 什么都不做（既不换曲、也不动 trk_sec_r），音频照原样接着播。
        //   典型场景：几张图配上同一首 BGM，来回切图不该每次都从开头重放。
        //   ★ 2026-11-06：match 扫描现在**无条件**发起（字幕也要靠它找同名 TXT），
        //     但"换曲"这个动作仍然只听 osd_asy 开关 ⇒ 这里补上 && osd_asy。
        if (mm_hit && !mm_same && osd_asy) begin trk_set_r <= 1'b1; trk_sec_r <= lst_match_off; end
    end
end

// ---- 音频播放开关（SW2）----
wire        sw2_play;           // clk（50MHz 晶振）域：板上 ON = 0 → 取反后为"播放"
wire        audio_play_v;       // video_clk 域（两拍同步）
wire        wav_pass_end;       // ★ 整首播完（含排空，仅 loop=0；50MHz 域，一拍）
wire        wav_pass_tick;      // ★ 2026-11-08：每一遍播完（loop=1 也有；一拍）
reg         music_tgl;          // 播完翻转电平 → App（auto_mode=音乐结束 时切图）
wire [15:0] ld_src_w, ld_src_h; // 装载器解析出的源分辨率

wire        bp_req, bp_rdy, bp_valid, bp_done;
wire [23:0] bp_addr;
wire [15:0] bp_len;
wire [7:0]  bp_data;
wire signed [15:0] beep_smp;
wire        beep_act;
wire [23:0] src_valid_w, src_l_w, src_r_w;

wire        wav_valid;          // audio_wav_path 输出（video_clk 域）
wire [23:0] wav_left;
wire [23:0] wav_right;
wire        wav_underrun;
wire        wav_drop;
wire        wav_playing;        // 预灌完成、正在出声
wire [10:0] wav_level;          // 音频 FIFO 现有字数（0..1024）→ 数码管调试

// ---- 数码管音频调试页（2026-10-27）----
//  为什么要有：音频是"无声"类故障，看不到内部状态只能猜。
//  把下面 4 个芯粒摆到数码管上，一眼就能判断卡在哪一段：
//    d5 = {sw2_play, 音频已获总线, sd_init_done, 正在出声}
//         典型读数：B = 播放中且初始化完成；A = 数据没跟上；8 = 开关没被读到
//    d4 = FIFO 水位 >> 6      ┐ 合起来 = 水位/4 的十六进制
//    d3 = (FIFO 水位 >> 2) & F┘   例：4b ≈ 300 字
//    d2 = {仲裁超时, 欠载, 溢出, 图像已获总线}
wire [3:0]  dbg_aud0, dbg_aud1, dbg_aud2, dbg_aud3;
wire [9:0]  wav_level_c = (wav_level > 11'd1023) ? 10'd1023 : wav_level[9:0];
assign dbg_aud0 = {sw2_play, (sd_arb_owner == 2'd2), sd_init_done_w, wav_playing};
assign dbg_aud1 = wav_level_c[9:6];
assign dbg_aud2 = wav_level_c[5:2];
assign dbg_aud3 = {sd_arb_timeout, wav_underrun, wav_drop, (sd_arb_owner == 2'd1)};

// ---- AXI-Stream 视频流（HDMI IP 要求的输入格式）----
wire        axis_s_user;        // 帧起始标志（Start of Frame）
wire        axis_s_valid;       // 数据有效
wire        axis_s_last;        // 行结束标志（End of Line）
wire [23:0] axis_s_data;        // 像素数据 RGB888
wire        axis_s_ready;       // 下游是否准备好接收（反压）

// ---- EDID 读取 ----
wire        edid_trig;          // EDID 读取触发（上电后自动打一拍）
wire        edid_valid;         // EDID 数据有效
wire [7:0]  edid_data;          // EDID 字节数据

// ---- TMDS 编码后的 10bit 并行数据 ----
wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

//==============================================================================
//  复位源：SW1（C8）—— 异步置位 / 同步释放（≈42ms）+ 音频 PLL 锁定
//==============================================================================
// 【为什么复位从 KEY1 挪到 SW1】
//   · KEY1(A2) 让出来当普通按键 ⇒ KEY1~KEY4 四个键全都能用（还能出按下脉冲）
//   · SW1 是乒乓开关：拨到 ON 就一直按住复位，正好当"维护复位 / 强制重启"
//
// 【为什么不是一句 wire rst_n = sw1】
//   开关拨动的瞬间触点会抖几 ms，直接当复位 = 系统在抖动期间反复重启。
//   这里用的是教科书式 "异步置位 / 同步释放"：
//     · SW1 拨到 0（ON）→ rst_n 立刻变 0，**不等时钟**（按住多久就复位多久）
//     · SW1 拨回 1（OFF）→ 计数器数满 2^21 个周期（@50MHz ≈ 42ms）才放行
//       期间再抖一下 → 计数器被异步清零、重新数 ⇒ 抖动被彻底吃掉
//   · reg 声明时给初值 0 ⇒ 上电后必然先等满 42ms 才释放复位，
//     顺带把上电电源爬升 / PLL 锁定的时间一起覆盖掉（本身就是 POR）。
//
// rst_all 的极性：高电平有效（1 = 复位中）。
//   SW1 = ON      → rst_n = 0 → ~rst_n = 1 → rst_all = 1（复位）
//   音频 PLL 没锁 → audio_pll_lock = 0 → ~audio_pll_lock = 1 → rst_all = 1（复位）
//
// 下面这一行是"上电防抖复位（POR）机制"的核心：确保所有 PLL 完全锁定
// 且稳定后才放系统跑起来，防止上电时钟抖动导致花屏或死机。
//==============================================================================
reg [20:0] rst_rel_cnt = 21'd0;

always @(posedge clk or negedge sw1) begin
    if (!sw1)                                   // SW1 = ON：异步清零 ⇒ 立即复位
        rst_rel_cnt <= 21'd0;
    else if (rst_rel_cnt != 21'h1F_FFFF)        // 已松开：慢慢数到满
        rst_rel_cnt <= rst_rel_cnt + 21'd1;
end

wire rst_n = (rst_rel_cnt == 21'h1F_FFFF);      // 数满才放行（低有效）

//==============================================================================
//  ★ 2026-11-02 [SD-GATE]：复位源 = SW1 + 音频 PLL + **卡不在位**（含热插拔）
//==============================================================================
//  rst_src：真正的硬件复位源（SW1 按住 / 音频 PLL 没锁）。
wire rst_src = ~rst_n | ~audio_pll_lock;

//------------------------------------------------------------------
//  卡在位判定：两条判据，任一成立就算"不在位"
//------------------------------------------------------------------
//  ① 从没初始化成功：sd_init_done_w = 0（上电时没卡 / 卡没插紧）
//  ② 运行中被拔掉：下一次读卡时仲裁器等不到 end（TIMEOUT_CYC=1e6 拍 ≈20ms）
//     ⇒ sd_arb_timeout 粘滞置位。
//     ★ 板上**没有卡检测(CD)引脚**（pin.adc 里 SD 只有 dclk/miso/mosi/ncs），
//       所以只能靠"访问失败"来发现拔卡；发现延迟 = 下一次读卡 + 20ms。
//  sd_lost 粘滞，唯一的清除条件是"重新初始化成功"（sd_init_done_w 上升沿）。
reg sd_lost;
reg sd_done_d;
always @(posedge sd_card_clk or posedge rst_src) begin
    if (rst_src) begin
        sd_lost   <= 1'b0;
        sd_done_d <= 1'b0;
    end else begin
        sd_done_d <= sd_init_done_w;
        if (sd_arb_timeout)                        sd_lost <= 1'b1;
        else if (sd_init_done_w & ~sd_done_d)      sd_lost <= 1'b0;
    end
end

wire sd_ok = sd_init_done_w & ~sd_lost;

//  设计侧复位：卡不在位 ⇒ 与"SW1 拨到 ON"完全等效（整机停在复位）。
//  ⚠ 释放相对 video_clk / ext_mem_clk 是异步的（与原有的 ~audio_pll_lock 同性质）。
//  ⚠ 所有参与本门控的寄存器一律用 rst_src 复位，**不能用 rst_all**（否则自锁死）。
assign rst_all = rst_src | ~sd_ok;

//------------------------------------------------------------------
//  自动恢复：拔卡后给 **SD 主机** 一个复位脉冲，让它从 CMD0 重跑初始化
//------------------------------------------------------------------
//  ★ 为什么"一次"脉冲就够，不需要周期性重试：
//    `sd_card_sec_read_write` 的初始化状态机在命令失败时**原地重发同一条命令**
//    （S_CMD0 / S_CMD8 / S_CMD55 / S_CMD41 的 else 分支都是"摆好命令、继续等"），
//    所以：
//      · 上电时没卡   → 主机一直重发 CMD0，卡插上就自然往下走（不需要我们干预）
//      · 运行中拔卡   → 主机停在 S_READ 等数据，必须复位一次才能回到 CMD0
//    因此这里只做"读超时 → 复位 SD 主机一次"。卡插回来后由主机自己完成初始化，
//    初始化成功（sd_init_done_w 上升沿）自动清 sd_lost ⇒ 整机放行。
//  ★ 因为没有周期性复位，也就不会把一次合法的慢初始化（ACMD41 最长约 1s）打断。
reg [3:0] sd_rst_cnt;

always @(posedge sd_card_clk or posedge rst_src) begin
    if (rst_src)                 sd_rst_cnt <= 4'd0;
    else if (sd_arb_timeout)     sd_rst_cnt <= 4'd15;      // 展宽 16 拍
    else if (sd_rst_cnt != 4'd0) sd_rst_cnt <= sd_rst_cnt - 4'd1;
end

//  只喂给 sd_card_top（它下面含协议层/块层/SPI 物理层，整条一起重来最干净）。
wire rst_sd_host = rst_src | (sd_rst_cnt != 4'd0);

//==============================================================================
//  ★ 预留接口：板上 4 个按键 + 4 个乒乓开关，8 路全部 20ms 消抖 ★
//==============================================================================
//  【位序】（和引脚表一一对应）
//      bit0 = KEY1 / SW1     bit1 = KEY2 / SW2
//      bit2 = KEY3 / SW3     bit3 = KEY4 / SW4
//
//  【输出怎么用】
//      key_level / sw_level : 消抖后的稳定电平（1 = 松开 / OFF，0 = 按下 / ON）
//      key_press / key_release : 按下沿 / 松开沿，**单周期高脉冲**（clk = 50MHz 域）
//      ⇒ 将来要"按一下做一件事"，直接用 key_press[n]，不用自己再判边沿
//      ⚠ 这些信号在 **clk（50MHz 晶振）域**。要拿到 sd_card_clk / video_clk
//        域去用，必须先打两拍同步；直接跨域会漏脉冲（1 拍 = 20ns，比采样点还窄）。
//
//  【现在没人用，为什么还要例化】
//      这就是"预留"。综合工具如果发现输出悬空，会把整块逻辑优化掉 —— 无所谓，
//      顶层端口和 pin.adc 约束还在，将来一接上就自动带出来。
//      但为了**现在就能上板验证**（顺便防优化），下面把状态打到了数码管上，
//      见 "数码管自检" 一节。
//
//  【为什么消抖模块不复位到 rst_all】
//      rst_all 由 SW1 产生，而 SW1 自己也要消抖 ⇒ 拿 rst_all 去复位"读 SW1 的
//      逻辑"会成环。所以这里用 ~audio_pll_lock（上电/PLL 未锁）当它的复位：
//      既保证上电有确定的初值，又不会让 SW1 把自己读成一片 1。
//==============================================================================
wire rst_input;                                 // 消抖模块专用复位（高有效）
assign rst_input = ~audio_pll_lock;

// key_press / key_level：**路由之后**的按键总线（板载 4 键 ∪ 五向键映射），
//   由下面 App 例化前的 key_router 驱动（它要用 dbg_osd / oz_on_w，声明在那之后）。
wire [3:0] key_level, key_press;
wire [3:0] key_level_hw, key_press_hw, key_release;     // 板载 4 键（消抖后、路由前）
wire [3:0] sw_level,  sw_press,  sw_release;
wire [4:0] bt_level,  bt_press,  bt_release;            // 五向键 {OK,DN,RT,LT,UP}

// ★ 2026-11-03 [Keypad]：13 路（五向键 5 + 拨码 4 + 板载键 4）共用**一个**紧凑消抖：
//   共享 2ms tick，每路 4 位计数（10 × 2ms = 20ms 窗口，与原 board_input_debounce 相同），
//   极性 / press / rls 语义不变。替换原来的 8 路 board_input_debounce 实例。
key_debounce9 #(
    .WIDTH       (13),
    .CLK_HZ      (50_000_000),
    .TICK_MS     (2),
    .DEBOUNCE_MS (20)
) u_key_deb (
    .clk     (clk),
    .rst     (rst_input),
    .din     ({bt_ok, bt_dn, bt_rt, bt_lt, bt_up, sw4, sw3, sw2, sw1, key4, key3, key2, key1}),
    .level   ({bt_level,   sw_level,   key_level_hw}),
    .press   ({bt_press,   sw_press,   key_press_hw}),
    .rls     ({bt_release, sw_release, key_release})
);


//==============================================================================
//  ★★ build FB16：帧缓存格式（RGB888 / RGB565）动态切换 ★★
//==============================================================================
//  【为什么要切】一帧 20.9ms（100MHz），60fps 只要 16.667ms ⇒ 差 25%。
//    槽表/标签环/忙守卫/提频全试过，都是把缺口在端口间挪；唯一能省的是
//    "每帧搬多少个 32 位字"，而 565 打包让它减半（307200 → 153600）。
//  【为什么不一直 565】静止画面 30fps 和 60fps 看起来一样，没必要为它牺牲色深。
//    ⇒ 动画中 = 565（60fps）；静止 = 888（全彩，30fps，画面不动看不出）
//    ⇒ 板上 key2 = 手动强制 888，用来肉眼验证"切换是否无缝"
//  【怎么做到无缝】
//      ① 格式是**每块缓冲自己的属性**（fb_buf_fmt[1:0]），写侧在帧起始写入
//      ② 显示侧只在**场边界换块**时（disp_rd_idx 同一拍）才换格式
//      ⇒ 永远不会"这块是 565 却按 888 读"；也不需要重写旧帧
//      ⇒ 不闪黑、不撕裂、不重建
//==============================================================================
wire        fb_fmt565;          // 动画中 = 1（写侧选 565）
wire [1:0]  fb_buf_fmt;         // 两块缓冲各自的格式（bit=1 ⇒ 565）
reg  [1:0]  fb_buf_fmt_v0, fb_buf_fmt_v1;   // 同步到 video_clk
wire [1:0]  fb_buf_fmt_v = fb_buf_fmt_v1;
wire        fb_fmt_disp = fb_buf_fmt_v[disp_rd_idx];   // 选"正在读的那块"的格式
//  fb_buf_fmt 只在"某块写完一帧"时变，属于准静态多比特 ⇒ 两级同步足够；
//  且 fb_fmt_disp 会跟着 disp_rd_idx 在同一拍切换（此时是场消隐，read_en=0）。
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        fb_buf_fmt_v0 <= 2'b00;
        fb_buf_fmt_v1 <= 2'b00;
    end else begin
        fb_buf_fmt_v0 <= fb_buf_fmt;
        fb_buf_fmt_v1 <= fb_buf_fmt_v0;
    end
end


//==============================================================================
//  【时钟生成】三个 PLL
//==============================================================================
// PLL = Phase Locked Loop，锁相环。它的作用是"倍频/分频/移相"。
// 板卡只给了一个 50MHz 时钟，但系统需要 25/100/125/12.288MHz 好几种，
// 所以用 3 个 PLL 分别产生。

// ---- 系统 PLL：产生 SD 卡侧和 SDRAM 侧时钟 ----
// 输入 50MHz，输出：
//   clk0 = 50MHz   → sd_card_clk：App（像素源 + 写帧控制器）用这个时钟
//   clk1 = 125MHz  → ext_mem_clk：SDRAM 控制器的主时钟
//   clk2 = 125MHz (相移180°) → ext_mem_clk_sft：给 SDRAM 做数据采样窗口调整
// .reset(1'b0) 表示不使用该 PLL 的复位端口（PLL 会自己锁定）。
sys_pll sys_pll_m0(
    .refclk     (clk),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft),
    .reset      (1'b0)
);

// ---- 视频 PLL：产生像素时钟和 TMDS 串行时钟 ----
// 输入 50MHz，输出：
//   clk0 = 25MHz   → video_clk：640×480@60Hz 的像素时钟（25.175MHz 的近似值）
//   clk1 = 125MHz  → hdmi_5x_clk：像素时钟的 5 倍频，TMDS 需要把 10bit
//                                用双沿（DDR）在 5 个时钟周期内串出去
video_pll video_pll_m0(
    .refclk     (clk),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk),
    .reset      (1'b0)
);

// ---- 音频 PLL：只取 12.288MHz 音频主时钟 ----
// 12.288MHz = 48000 × 256，是音频行业的标准主时钟频率（256fs）。
// 有了它才能精确生成 48kHz 采样率。
// 这个 PLL 是复用 I2S→HDMI 测试工程里的配置，clk0/clk1 用不上所以留空 ()。
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (audio_pll_lock),   // 锁定标志，用来参与 rst_all 的生成
    .clk0_out   (),                 // 未使用
    .clk1_out   (),                 // 未使用
    .clk2_out   (audio_mclk)        // 12.288MHz
);

//==============================================================================
//  跨时钟域信号：把"写完成脉冲"从 mem_clk 域送到 sd_card_clk 域
//==============================================================================
// 【为什么要这么做】
// frame_write_finish 是 mem_clk（125MHz）域的一个单拍脉冲。
// 如果直接把这个脉冲接到 sd_card_clk（50MHz）域去用，会出问题：
//   - 两个时钟频率不同、相位无关联，脉冲可能被采到、也可能被完全漏掉；
//   - 就算采到了，也是"亚稳态"（信号处于 0 和 1 之间），后续逻辑行为不确定。
//
// 【标准的跨时钟域做法：脉冲 → 电平翻转（toggle）→ 打拍同步 → 边沿检测】
//   第1步（这里）：每次收到 write_finish 脉冲，就把 frame_write_toggle_mem 翻转一次。
//                 翻转是个"电平变化"，能保持足够长时间被另一侧采到。
//   第2步（在 sd_card_bmp.v 里）：用 3 级移位寄存器打拍同步，再异或相邻两位，
//                 还原出"另一侧的脉冲"。这就是经典的"脉冲同步器"。
//
// 【为什么不用两拍同步器直接同步脉冲？】
// 因为脉冲可能只有 1 个时钟宽，而同步器需要采样，1 拍宽的信号在
// 不同频时钟域之间可能"整个漏掉"（另一侧采样点刚好都在脉冲之外）。
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all)
        frame_write_toggle_mem <= 1'b0;          // 复位时清零
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;  // 来一个脉冲翻一次
end

// ---- 第 3 步（在 sd_card_clk 域）：打 3 拍同步，再异或还原出脉冲 ----
// 【为什么是 3 拍而不是 2 拍】前两拍用来消除亚稳态，第三拍才能和第一拍做比较
// （第 1 拍可能还在亚稳态，拿它做逻辑运算不可靠）。
// 【异或为什么能还原出脉冲】两位不同 = 这两拍之间发生过一次翻转 = 正好是那个脉冲。
// 下面这段原来在 sd_card_bmp.v 内部，重构后应用层拆成了 frame_writer +
// tf_bmp_pixel_source，跨时钟域这件事属于"顶层接线"，所以搬到 top 里来。
reg [2:0] wrfin_tgl_sync;
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) wrfin_tgl_sync <= 3'b000;
    else         wrfin_tgl_sync <= {wrfin_tgl_sync[1:0], frame_write_toggle_mem};
end

// wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1]：相邻两位不同 → 这一拍产生 1 拍宽的脉冲
wire write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];

//==============================================================================
//  ★ 场同步换页的跨时钟域（2026-10-17 修撕裂）★
//==============================================================================
//  两组方向相反的**慢速电平**（每帧才变一次，约 16.7ms ⇒ 打 3 拍足够）：
//     正向：写侧提交块号 + 提交事件（sd_card_clk → video_clk）
//     反向：显示侧采纳事件        （video_clk    → sd_card_clk）
//  为什么慢速电平可以直接打拍同步？因为它每次变化都保持整整一帧，
//  同步器采样点无论落在哪里都能采到，不会像"单拍脉冲"那样整个漏掉。
//  （对比上面 write_finish：那是 8ns 单拍脉冲，必须先转成翻转电平才能过域。）
//==============================================================================
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        commit_idx_v <= 4'd0;
        commit_tgl_v <= 3'd0;
    end else begin
        commit_idx_v <= {commit_idx_v[1:0], disp_buf_idx};    // 2 位数据：2 级足够
        commit_tgl_v <= {commit_tgl_v[1:0], fw_commit_tgl};   // 1 位事件：3 级
    end
end

always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) adopt_tgl_v <= 3'd0;
    else         adopt_tgl_v <= {adopt_tgl_v[1:0], disp_adopt_tgl};
end

//==============================================================================
//  【应用层】像素源 + 写帧控制器
//==============================================================================
// 原来这一块是一个大模块 sd_card_bmp 全包了（扫描 + 解析 + 写帧时序 + 双缓冲）。
// 现在拆成两个职责清晰的模块：
//
//   tf_bmp_pixel_source（像素源）—— "像素从哪来"
//       · 上电扫描 TF 卡、找出最多 4 张合法 BMP
//       · 解析 BMP、响应按键（key1 手动下一张 / key2 自动轮播开关）
//       · 输出 ps_data / ps_valid（一个像素一拍）
//
//   frame_writer（应用层）—— "怎么把一帧送上去"
//       · 和显存控制器握手（write_req / write_req_ack）
//       · 把像素写进 SDRAM（write_en / write_data）
//       · 管双缓冲索引，整帧写完才切显示（避免撕裂）
//
// 两者之间还共享 pipe_ce：背压时坐标、场景流水线和写口延迟链同步保持。
// 这样拆的好处：想换个像素来源（测试图案、摄像头、运算结果），
// 只要按同样的接口产像素，frame_writer 一行都不用改。

// ---- 像素源 ↔ 写帧控制器 之间的信号 ----
wire        ps_start;        // 写帧控制器 → 像素源："开始给这一帧的数据"
wire        ps_new_frame;    // 像素源 → 写帧控制器："我有新的一帧要画"
wire [23:0] ps_data;         // 像素数据 {R,G,B}
wire        ps_valid;        // 像素有效（与 ps_data 同拍）
wire        app_pipe_ce;     // App 场景流水线推进使能（与 frame_writer 写口同步）

// ---- 帧节奏反馈（写帧控制器 → App）----
wire        app_busy;        // 正在画一帧（电平）
wire        app_done;        // 画完一帧（1 拍脉冲）

//==============================================================================
//  【应用层】App —— 用模块拼出来的场景图
//==============================================================================
// 【这里原来接的是什么】
// 原来接的是 tf_bmp_pixel_source：上电扫描 TF 卡、解析 BMP、按键切换图片。
// 问题是不便于二次开发：想加个叠加层 / 画个图形 / 做个动画，都得动那个大模块。
//
// 【现在换成 App】
// App 把"画面"拆成一组可组合的模块：
//     app_scan_gen   —— 扫描坐标发生器（产生整条流水线的时间轴）
//     ps_gen_*       —— 图形生成器（矩形 / 圆 / 三角 / 圆角矩形 / 多边形 / 纯色）
//     ps_offset      —— 坐标偏移（偏移不改像素，只改"源看哪个坐标"，零成本）
//     ps_scale       —— 坐标缩放（最近邻）
//     ps_merge_*     —— 合并器（直接叠加 / 透明度叠加）
//     ps_to_rgb24    —— 叠加到纯黑 + 舍弃 alpha
//
// 【★ 接口和原来完全一样 ★】
// PIPE 与场景图总延迟同步设为 13，pipe_ce 与 ps_valid 一并连接到 frame_writer。
//
// 【SD 卡那套代码没有删】
// tf_bmp_pixel_source.v / bmp_read.v / sd_card_top.v 等仍然在工程里，
// 只是暂时没有被例化。后续任务（比如从 SD 卡读 BMP 当纹理）还会用到。
//==============================================================================
//==============================================================================
//  ★ 图片层自检开关：key1（B2）或 key2（C1）每按一次翻转，带消抖 ★
//==============================================================================
//  【为什么加这个】上板后图片层始终不出图，而仿真里所有能造的场景都 PASS ——
//  需要把"层链路（框判定/缩放坐标/叠加/时序）"和"纹理数据 / 读回路径"
//  一刀切开。有了它，**烧一次、按一下按键**就能判读：
//
//      · 框内出现彩色渐变（G 随列、B 随行）→ 层链路完全正确
//                                              ⇒ 问题在纹理数据或读回路径
//      · 框内一点变化都没有                    → 层链路本身有问题
//
//  自检时图片层不依赖 SDRAM 数据、也不参与全局流控（见 app_demo_scene.v）。
//  两个按键都接上是为了避免"极性/位置猜错"白跑一趟：
//      · 假设按键有上拉、按下为低（绝大多数开发板如此）。若两个键都没反应，
//        把下面 `&& !key1_stb` 改成 `&& key1_stb`（key2 同理）即可。
//
//  ★★ 2026-10-16 补充：上电后【自动】自检 5 秒，不按按键也能拿到答案 ★★
//   【为什么】如果只能靠按键，万一按键极性/位置不对，这一趟烧录就白跑了。
//   所以改成：上电后 5 秒内 img_selftest 自动 = 1（图片框里是彩色渐变），
//   5 秒后自动回到 0（正常模式，图片框里应该是苹果/草莓）。按键仍可随时翻转。
//   ⇒ 一次烧录、**什么都不用按**，就能看到"层对不对"和"纹理数据有没有"两组答案：
//        前 5 秒  框内是渐变、5 秒后框内是空白 → 层完全正确，问题在纹理数据/读回
//        前 5 秒  框内也是空白                 → 层本身有问题（框判定/叠加/时序）
//==============================================================================
//  ★ build 100M-9：上电自动自检**关掉**（清零）★
//  以前是 5 秒：上电后图片框里先放彩色渐变、5 秒后才切回真纹理。
//  现在要一眼看"真图显示得对不对"，那 5 秒的渐变只会让人以为没生效，所以清 0。
//  想临时看渐变：短按 key1（st_manual 翻转）随时能调出来。
//  ★ 2026-10-22：key2/key4 的"冻结动画/自检"逻辑随 demo 一起删除。
//    按键现在直接由 board_input_debounce 的 press 脉冲送进 App：
//        key4 → 上一张   key3 → 下一张   key2 → 换过渡效果
//==============================================================================

//==============================================================================
//  ★ 串口调试已整体移除（dbg_uart 实例删除）★
//  端口 uart_tx 保留声明（pin.adc 仍有 D12 约束），固定驱动为高电平（UART 空闲态）。
//==============================================================================
//==============================================================================
//  ★★ 2026-10-24：串口回传 A 层像素轨迹（数码管当逻辑分析仪的延伸）★★
//==============================================================================
//  只抓 Y_SEL 行、x 在 [X_SEL, X_SEL+XW) 的 XW 个像素，每个像素 2 条行８16 位十六进制）：
//    行 A：{sx[15:0], y[15:0], x[15:0], flags[5:0], 10'd0}
//    行 B：{head[31:0], raw[23:0], 8'd0}
//  打印形式：每帧一块，先 "N=xx"（0x40=64 条）再逐行输出。
//==============================================================================
// ★ 2026-10-24：串口探针已整体移除。诊断信息已够用：
//   图层输出正确 / 写侧握手 F0 / 像素数 307200 / 写侧回跳 480。
//   探针本身多次引入侧效应，所以直接取消。
assign uart_tx = 1'b1;



// ---- 下面这两块（"每层有没有收到纹理像素"粘滞位、调度器 cmd/beat 计数同步）
//      都只是给数码管看的观测量，随调试读数一起删除 ----

//==============================================================================
//  ★★ build 1：组相位位移已定论为 0，扫描拆掉 ★★
//==============================================================================
//  build F 实验结果：ph_off=0 时基线画面正常；ph_off=1/2/3 时**整屏变花**
//  （因为 rd_go/wr_go 的起搏相位也被一起挪了，显示流被破坏）。
//  ⇒ **0 就是控制器真实的组边界**，组相位假设排除。端口保留，接常量 0。
//  这一段代码保留说明，不再占资源。

// ---- 写 FIFO 水位比较（打一拍；复位后确定为 0）----
//  wr_inflight 是 wfifo 的写侧 wrusedw，已在 sd_card_clk 域，无需跨时钟域。
//  阈值必须 < 512（9 位水位在 512 处回绕），并给 frame_writer 的 PIPE=13 在途写留余量。
//
//  ★★ 实验 W2（2026-10-22）：384/320 ★★
//  【为什么又抬回去】W1 把写口加了一组"挡忙"，写 FIFO 会被削；W2 又把 TX 空闲的
//     槽 5..9 借给了写 ⇒ 借槽回合里一轮最多能吃掉 **144 字**。
//     阈值 192 时，FIFO 在一轮中途就会掉到 BURST_SIZE(128) 以下 ⇒ 写引擎凑不够
//     一次突发 ⇒ 写口反而吃不满，借来的槽等于白借。384 留出 384-144=240 字余量，
//     一轮吃完仍 ≥128 ⇒ 突发永不等待。
//  【安全性】上限 = 384 + PIPE13 的在途写 13 = 397 < 512 ✓ 不会触发 wfifo 静默丢字。
//  【回退】改回 `wr_stall_r <= (wr_inflight >= 10'd448);` 单行即可。
wire        wr_stall;
reg         wr_stall_r;
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all)                     wr_stall_r <= 1'b0;
    // ★ build Q48：阈值 384/320 → 192/160（代码 :1107 早就建议过这个值）
    //   贴 384 时一旦顶到就是长停顿；降到 192 让反压更早、更平滑，
    //   配合 SCENE_QUOTA=48 基本不会触发。
    else if (wr_inflight >= 10'd192) wr_stall_r <= 1'b1;
    else if (wr_inflight <= 10'd160) wr_stall_r <= 1'b0;
end
assign wr_stall = wr_stall_r;

//==============================================================================
wire [2:0] osd_page_w;   // OSD 当前页号（2 = 文件页）
// ★ 文件页 = 菜单开着 && page==2（page 是粘滞寄存器，关菜单后仍为 2）
assign osd_file_page = osd_fp_w;
//  App #(
//==============================================================================
//  ★ 层0 纹理基址：线性布局，固定 TEX0_ADDR（模式 5 不动布局，所以只有一个值）
//------------------------------------------------------------------------------
//  上一版有一条"稀疏模式把纹理挪到 1228800"的分支（12/16 布局要 409600 字，
//  放在 1703940 会顶到 2^21 之外）。稀疏布局已被证伪，这条分支连同
//  sd_bmp_loader / ps_gen_image 里配套的地址映射一起删除。
//  现在纹理就是一块连续的 640×480，地址 = TEX0_ADDR + 像素序号。
//  声明放在 App 例化之前：iverilog 不允许"先用后声明"。
// （tex0_base_cur 已随 App 的 tex0_base_ovr 端口一起删除）

// ★ 2026-10-30：缩放 OSD 的几何口（声明放 App 例化之前，iverilog 不允许先用后声明）
wire [7:0] ld_zoom_w;                 // Q4.4，1×/2×/4×/8×
wire [9:0] ld_ax_w;                   // 锚点（屏幕坐标）
wire [9:0] ld_ay_w;
// ★ 2026-11-05 [LD-SNAP]：viewer_ctrl 在发单那拍锁存的烘焙几何
//   （与上面三个"实时值"的区别见 viewer_ctrl 的端口注释）
wire [7:0] ld_snap_zoom;
wire [9:0] ld_snap_ax, ld_snap_ay;
wire       oz_on_w;                   // 缩放 OSD 是否打开（key_router 用来判上下文）

//==============================================================================
//  ★ 2026-11-03 [Keypad]：按键路由（板载 4 键恒等 + 五向键按上下文映射）
//==============================================================================
//  上下文：缩放界面（oz_on_w）> 菜单（dbg_osd = osd_on，含文件页）> 查看器。
//  映射表见 peripherals/key_router.v 文件头。纯组合，不改变原 4 键的任何时序。
key_router u_key_router (
    .hw_press  (key_press_hw),
    .hw_level  (key_level_hw),
    .bt_press  (bt_press),
    .bt_level  (bt_level),
    .menu_on   (dbg_osd),
    .zoom_on   (oz_on_w),
    .key_press (key_press),
    .key_level (key_level)
);

//==============================================================================
//  四宫格模式：**只在帧边界切换**
//==============================================================================
//  【为什么不能直接接组合值】quad_mode 一进 viewer_scene 就同时影响
//    A 层使能、tex_base、以及 ps_gen_image 内部的框判定与地址映射。
//    帧中间翻转会让"已经发出去的请求"和"正在弹的显示"落在两套映射上
//    （这套流水线本来就要求 a_hd 等几何量只在帧边界改）。
//    ⇒ 用 ps_new_frame 采样一次，与既有约定保持一致。
//==============================================================================
reg quad_mode_r;
// ★ 2026-11-03：本页有效格掩码，和 quad_mode 同一拍采样（帧内恒定）。
//   末页不满 4 格时，空格不画（否则露出上一页留在槽里的图）。
reg [3:0] quad_vm_r;
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        quad_mode_r <= 1'b0;
        quad_vm_r   <= 4'd0;
    end else if (ps_new_frame) begin
        quad_mode_r <= quad_ready & osd_file_page;
        quad_vm_r   <= {quad_vcnt > 3'd3, quad_vcnt > 3'd2, quad_vcnt > 3'd1, quad_vcnt != 3'd0};
    end
end

// ★ 2026-11-07 字幕显示门控：菜单没开就显示（文件页也在菜单里，一并隐藏）。
//   不再等"高清已上屏"：字幕在缩略图预览阶段就出来；换图时旧字幕不撤，
//   由 sub_loader 的双缓冲在新字幕装好后原子替换。
//   dbg_osd = App 的 osd_on（sd_card_clk 电平），dyn_subtitle 内部 2FF + vs 沿采样。
assign sub_show_en = ~dbg_osd;

// ★ 2026-11-01 P5：翻页不再直接由按键产生 —— 按键先动光标，
//   光标走到第 4 格再按（或末页空格）才由 OSD 发出翻页请求。
//   这样"翻页"和"移光标"共用同一套边界判断，不会出现"翻页了但光标还在半路"。
//   （原来这里直接把 key3/key4 接到 quad_page_ctrl，等于按一次翻一页、
//     光标概念根本不存在。）

App #(
    .IMG_W (640),
    .IMG_H (480),
    .THUMB0_ADDR (THUMB0_ADDR),
    .THUMB1_ADDR (THUMB1_ADDR),
    .THUMB2_ADDR (THUMB2_ADDR),
    .HD_ADDR     (HD_ADDR)
) u_app (
    .clk          (sd_card_clk),        // 50MHz
    .rst          (rst_app),            // ★ [SD-GATE] 同 rst_all（含"没卡"门控）
    // 四宫格：只有"停在文件页 && 至少一格已装好"才打开，
    // 否则会把 SDRAM 里的陈货（相册帧缓存）画到格子里 = 花屏
    // ★ 2026-11-02 [QUAD-OFF-NOW]：再与一个 osd_file_page（**1 个 LUT，纯组合**）。
    //   【为什么】quad_mode_r 只在 ps_new_frame 采样 ⇒ 关菜单之后至少还有一帧是
    //   "菜单已经没了、四宫格还在画"。万一这一帧之后新帧没接上（退出路径卡在
    //   S_HDF 等 busy），屏幕就**定格**在那一帧：整屏黑 + 0 号格只有半张第一张图。
    //   【为什么安全】viewer_ctrl 只在 `!busy` 那一拍清 osd_on，那一拍没有帧在合成，
    //   所以这里中途落 quad_mode 切不出"半帧四宫格"；帧内恒定只对**开**的方向有意义。
    .I_quad_mode   (quad_mode_r & osd_file_page),
    .I_quad_vm     (quad_vm_r),
    .O_page        (osd_page_w),
    .O_file_page   (osd_fp_w),
    // 文件页名字读口 → fat_dir_lister 的名字 BRAM
    .O_nm_slot     (lst_rd_slot),
    .O_nm_idx      (lst_rd_idx),
    .I_nm_char     (lst_rd_char),
    .I_nm_blen     (lst_rd_blen),
    .I_nm_elen     (lst_rd_elen),
    .I_nm_len4     (lst_len4),
    .I_nm_vld      (lst_rd_vld),
    // 文件页光标（P5）与相册偏移（P6）
    .I_img_off0    (alb_off0),
    .I_img_off1    (alb_off1),
    .I_img_off2    (alb_off2),
    .I_img_off3    (alb_off3),
    .O_alb_base    (alb_base),
    .I_alb_win4    (alb_win4),
    .I_alb_any     (alb_any),
    .I_alb_nimg    (alb_nimg),
    .I_alb_ready   (alb_ready),
    .O_fp_cur      (fp_cur_w),
    .O_fp_flip_nx  (quad_flip_nx),
    .O_fp_flip_pv  (quad_flip_pv),
    .I_fp_vcnt     (quad_vcnt),
    .I_fp_has_more (quad_has_more),
    .I_fp_gen      (fp_gen_w),
    .I_fp_has_prev (quad_has_prev),
    .O_fp_open     (fp_open_w),
    .I_jump        (jump_r),
    .I_jump_idx    (jump_idx_r),
    .O_ld_abort    (app_ld_abort),
    .mclk         (ext_mem_clk),        // 100MHz：图片层的 SDRAM 读仲裁侧

    .ps_start     (ps_start),
    .ps_new_frame (ps_new_frame),
    .ps_data      (ps_data),
    .ps_valid     (ps_valid),
    .pipe_ce      (app_pipe_ce),
    .busy         (app_busy),

    .tex_req_vld  (tex_req_vld),
    .tex_req_addr (tex_req_addr),
    .tex_req_mask (tex_req_mask),
    .tex_req_ack  (tex_req_ack),
    .tex_pix_wr   (tex_pix_wr),
    .tex_pix_data (tex_pix_data),

    // ---- 按键：key4 上一张 / key3 下一张 / key2 换过渡效果 / key1 开 OSD 菜单 ----
    // ★ 文件页里 key3/key4 用来翻页，同时**屏蔽**相册的上一张/下一张
    //   （否则相册会在背后偷偷重装图，虽然看不见但会抢 SD 带宽）
    // ★ 2026-11-03：**不再**用 ~osd_file_page 屏蔽 KEY3/KEY4。
    //   这两根同时是 OSD 的"上/下"键（App 把它们并进 osd_top 的 key_press），
    //   屏蔽之后文件页里光标一步都走不动，导航栏停在"文件"上也再也移不开。
    //   相册在菜单里本来就不吃它们（viewer_ctrl S_OSD 只认 album_next/prev），
    //   缩放 OSD 在菜单开着时也被 menu_on 挡住 ⇒ 去掉屏蔽不影响别的页。
    .key_prev     (key_press[3]),
    .key_next     (key_press[2]),
    .key_fx       (key_press[1]),
    .key_menu     (key_press[0]),
    .key_level    (key_level),

    // ---- 用户 SPI Flash（OSD 字库 font_gb2312.bin）----
    .flash_cs_n   (flash_cs_n),
    .flash_sclk   (flash_sclk),
    .flash_mosi   (flash_mosi),
    .flash_wp_n   (flash_wp_n),
    .flash_hold_n (flash_hold_n),
    .flash_miso   (flash_miso),

    // ---- SD 装载器 ----
    .ld_req       (ld_req),
    .ld_fmt       (ld_fmt),
    .ld_off       (ld_off),
    .ld_len       (ld_len),
    .ld_base      (ld_base),
    .ld_can_load  (ld_can_load),
    .ld_done      (ld_done),
    .ld_err       (ld_err),
    .ld_sd_ready  (ld_sd_ready),        // ★ 卡初始化完成才允许发第一单
    .ld_src_w     (ld_src_w),
    .ld_src_h     (ld_src_h),
    .music_done   (music_tgl),
    .bp_req       (bp_req),
    .bp_addr      (bp_addr),
    .bp_len       (bp_len),
    .bp_rdy       (bp_rdy),
    .bp_data      (bp_data),
    .bp_valid     (bp_valid),
    .bp_done      (bp_done),
    // ★ 2026-11-06 字幕字模：flash_arb 的第 4 个客户（透传 App → osd_top）
    .sub_fl_req   (sub_fl_req),
    .sub_fl_addr  (sub_fl_addr),
    .sub_fl_len   (sub_fl_len),
    .sub_fl_rdy   (sub_fl_rdy),
    .sub_fl_data  (sub_fl_data),
    .sub_fl_valid (sub_fl_valid),
    .sub_fl_done  (sub_fl_done),
    // ★ 2026-11-05 [LD-SNAP]：本次装载的烘焙几何快照（缩略图装载恒 1x/中心）
    .ld_zoom      (ld_snap_zoom),
    .ld_ax        (ld_snap_ax),
    .ld_ay        (ld_snap_ay),

    .fmt565       (fb_fmt565),
    .wr_stall     (wr_stall),

    .st_img       (st_img),
    .st_fx        (st_fx),
    .fps_dbg      (fps_dbg),
    .dbg_state    (dbg_state),
    .dbg_osd      (dbg_osd),
    .dbg_fmt      (dbg_fmt),       // ★ 帧率/10 → 数码管最左位
    // （.st_state / .t_dbg / .blk_dbg / .duty_dbg / .dbg_delta / .dbg_evt /
    //   .smp_bus / .smp_win：随 2026-10-26 数码管清理一起删除，
    //   现在只保留 帧率 / 图片号 / 模式 / 装载进度 四项）

    // ---- 画质参数（2026-10-26 新增）→ video_postproc ----
    .pic_bright   (pic_bright),
    .pic_contrast (pic_contrast),
    .pic_temp     (pic_temp),
    .pic_sat      (pic_sat),
    .pic_style    (pic_style),
    .pic_tgl      (pic_tgl),
    // ★ 2026-11-03：ALS 生效亮度 → OSD 显示页亮度行
    .bright_eff   (pic_bright_eff),

    // ---- 系统页导出（2026-10-29）→ 音频链路 ----
    .o_lang       (osd_lang),
    .o_vol        (osd_vol),
    .o_mute       (osd_mute),
    .o_loop       (osd_loop),
    .o_spc        (osd_spc),
    // ★ 2026-11-02【音随图切】相册页 item6 的 CheckBox（与 OSD 同在 sd_card_clk 域，直连）
    .o_asy        (osd_asy),
    // ★ 2026-11-03 显示页"自动亮度" CheckBox → als_bright
    .o_auto       (osd_auto),
    // ★ 2026-11-02：本次高清装载的窗口格号（顶层查 lister 的名字 RAM 用）
    .O_hd_rel     (hd_rel_w),
    // ★ [MM-EARLY]：换图后一拍的提前 match 请求（音乐/字幕提前到动画里扫）
    .O_mm_arm     (mm_arm_w),
    .O_mm_rel     (mm_arm_rel),
    // ★ 2026-11-02【信息条文件名】相册重扫完成单拍（OSD 用它重写一次行 6）
    .I_nm_upd     (nm_upd),
    .I_nm_hold    (nm_hold),            // ★ [NM-HOLD] 名字读口被借走 ⇒ 本行写空格
    // 缩放 OSD（2026-10-30）→ 下面直接进 bmp_loader 的几何口
    .oz_on        (oz_on_w),
    .oz_zoom      (ld_zoom_w),
    .oz_ax        (ld_ax_w),
    .oz_ay        (ld_ay_w)
);

//==============================================================================
//  SD 卡引脚：应用层不再读卡，静态拉死
//==============================================================================
// 【为什么必须显式赋值】顶层端口是 output，如果没有任何东西驱动它，
// 综合工具会报"输出端口未驱动"。这里给一个安全的静态电平：
// nCS 拉高 = 片选无效（卡不会被选中），时钟/数据线拉低。
//==============================================================================
// 【2026-10-16 改造】SD 卡现在由 sd_bmp_loader 驱动，用来把 BMP 搬进 SDRAM 纹理区。
//
// key1 选择诊断页；key2 切换图片层显隐。

// ---- 应用层：写一帧的时序控制器 ----
frame_writer #(
    .IMG_W (640),
    .IMG_H (480),
    .NPX   (FRAME_PIXELS),              // 307200
    .PIPE  (5)                          // ★ 场景图总延迟 = 5 ★
                                        //   图片层 3 拍 + 混合 2 拍
                                        //   （混合模块 2026-10-30 加了 DSP 输入寄存器，
                                        //     ps_to_rgb24 是组合逻辑，0 拍）
                                        //   改场景结构时这个数必须同步改
) u_frame_writer (
    .clk          (sd_card_clk),        // 50MHz，与 App 同域（已回退）
    .rst          (rst_app),            // ★ [SD-GATE] 同 rst_all

    // ---- 像素源 ----
    .ps_start     (ps_start),
    .ps_new_frame (ps_new_frame),
    .ps_data      (ps_data),
    .ps_valid     (ps_valid),
    .pipe_ce      (app_pipe_ce),

    // ---- 写通道（接 frame_read_write 的写侧）----
    .write_req     (app_write_req),
    .write_buf_idx (write_buf_idx),
    .write_len     (),                  // 内部固定 = NPX，不需要引出来
    .write_req_ack (app_write_req_ack),
    .write_en      (app_write_en),
    .write_data    (app_write_data),

    // ---- 整帧写完回执（从 ext_mem_clk 域同步过来的脉冲）----
    .write_finish_pulse (write_finish_pulse),

    // ---- 提交上屏 ----
    .disp_buf_idx  (disp_buf_idx),
    .display_valid (display_valid),
    // （.px_dbg / .fw_smp / .fw_win：诊断探针，2026-10-26 随数码管清理删除）

    // ---- ★ 场同步换页（修撕裂）★ ----
    .disp_commit_tgl (fw_commit_tgl),    // 输出：每提交一帧翻转一次
    .disp_adopt      (disp_adopt_sync),  // 输入：显示侧已换页（已同步到本时钟域）

    // ---- 状态与坐标（给 App 做帧节奏反馈）----
    .busy          (app_busy),
    .done          (app_done),
    .px_x          (),
    .px_y          (),
    .out_x         (),
    .out_y         ()
);

// ---- 数码管显示：把状态码翻译成段码，再动态扫描输出 ----
// 【为什么要这个 mux】上板排查时最想知道的就是"装载到底怎么了"：
//   装载期间显示 sd_bmp_loader 的状态码
//       0 = 空闲（还没开始 / SD 卡没就绪）
//       1 = 读文件头校验
//       2~5 = 正在搬像素（来自 bmp_read 的子状态）
//       E = 这一槽装载完成
//       F = 装载失败（偏移填错 / 卡里那个位置不是 640×480 24bit BMP）
//   进入运行（B_RUN）后显示"帧有没有上屏"：
//       A = display_valid=1 → 已经有整帧写进 SDRAM 并上屏了，**屏幕应该是亮的**
//       b = display_valid=0 → 一帧都还没画完（像素流水线卡住 / 帧写没回来）
//           → 全屏黑就是这个状态，问题在"生成帧"这一侧，不在 HDMI 读出侧
//   ★ 为什么要把 A / b 分开 ★
//   顶层有一句 `vout_data = display_valid ? pp_out_data : 24'd0;`
//   —— display_valid=0 时**强制黑屏**。所以"全屏黑"其实有两种完全不同的原因：
//       ① 帧没画完（display_valid=0）→ 数码管 b
//       ② 帧画完了但读不出来（display_valid=1）→ 数码管 A
//   看一眼数码管就能分清，不用猜。
// （数码管"写值"已删除：原先这里给出 A=有帧 / b=无帧 的运行码）

//  ★ 2026-10-22：上电装载时序已移进 App（viewer_ctrl 的状态机）。
//    顶层只负责把 App 的 ld_* 接到 sd_bmp_loader，不再有自己的 boot FSM。

// （数码管第 5 位的"装载状态码 / 运行码"已删除，译码器改接常量）

// ★★ 装载结果：两张图分别锁存 ★★
//   【为什么要分开】IMG0(苹果→TEX0) 和 IMG1(草莓→TEX1) 是**同一个装载器顺序执行**的，
//   以前只锁存"最后一个值"，于是数码管上的 E **只能证明 IMG1 成功**，
//   IMG0 成没成完全看不见 —— 而你盯的正是 TEX0（苹果）。
//   【为什么在 done/err 那一刻显式写 E/F】err_p 是单拍脉冲，它拉高时 st 还停在
//   L_FEED，直接跟踪 state_code 会显示成 2~5（"还在搬"），看起来像没出错。
//   显式写 E/F 才可信。
// （ld_code_img0 / ld_code_img1：装载结果的数码管显示值 —— 随调试读数一起删除）

// ★★ 纹理读通道的"粘滞"观测（数码管第 0 位）★★
//   【为什么要三个位】"图片不显示"有三种完全不同的断点，必须一次分开：
//       · 层根本没发请求      → 问题在图片层（layer_en / rq_go / req_push / in_box）
//       · 发了请求、像素没回来 → 问题在仲裁 / SDRAM 读回这一侧
//       · 像素回来了、但**全是 0** → 纹理区里没数据（装载没写进去 / 地址不对）
//   ★ 第三位（tex_dat_seen）是最关键的一位，也是这一版新加的 ★
//   为什么"像素全 0"必须单独判：纹理字是 {8'hFF,R,G,B}，**只要写进去了，
//   高字节一定是 0xFF**。所以读回来是 0 就等于"这个字从没被写过"。
//   而 0 的 alpha=0 → ps_merge_overlay 判定"全透明" → 图片像素直接消失。
//   → 这一位能把"读回路径坏了"和"纹理区是空的"一刀切开。
//
//   数码管第 0 位读数（3 位粘滞标志 {req_seen, pix_seen, dat_seen}）：
//      0 = 什么都没发生（图片层没发过读请求）→ 查 layer_en / rq_go / in_box
//      4 = 请求发过了、但一个像素都没回来  → 查仲裁 / SDRAM 读回
//      6 = 像素回来了、但**全是 0**          → 纹理区里没数据（装载/地址）
//      7 = ★ 请求、像素、非 0 数据全齐 = 纹理读通道完全正常 ★
//          此时图片还不显示 ⇒ 问题在"层本身"或"叠加"（用自检模式再判一次）
//      （1/2/3/5 不会单独出现：dat_seen 蕴含 pix_seen，pix_seen 蕴含 req_seen）
// （tex_req_seen / tex_pix_seen / tex_dat_seen / drain_to_seen：
//   纹理读通道的四个粘滞观测位 —— 随调试读数一起删除）

// ★★ 读 FIFO 空/满粘滞标志（诊断用）—— 随调试读数一起删除 ★★
//  【为什么换掉 rdmin】上一版用 rdusedw 取历史最小值，但 **rfifo IP 的
//  rdusedw 是 9 位差值**（ADDR_WIDTH=9）：512 深的 FIFO 水位打到 512 时
//  回绕成 0 —— 第 2 位读 0 分不清是"读空"还是"打满回绕"，两轮判读都被
//  它晃点了。现在改用 FIFO 自己的硬标志（原本悬空的端口，本轮引出）：
//      rd_fifo_empty（video_clk 域）  → 视频真的会拿到旧数据
//      rd_fifo_full （ext_mem_clk 域）→ 再写就丢字（= 水位判据回绕地雷）
//  各自做粘滞位，第 2 位显示 {empty_seen, full_seen}：
//      0 = 两者都没发生过（好）
//      1 = 打到过满（full）  → 水位顶到 512：回绕 → 重复发突发 → 丢字横移
//      2 = 读空过（empty）   → 真欠载：视频拿到旧数据 = 那条"变亮"虚线
//      3 = 两个都发生过      → 水位在两轨间撞来撞去（节奏病理）
//  注意 full 在 125MHz 域、empty 在 25MHz 域，都是粘滞位，打两拍同步到
//  50MHz 域给数码管。
wire rd_fifo_empty, rd_fifo_full;       // ← frame_read_write 引出的端口（保留声明）

// （rd_empty_seen / rd_full_seen / rf_s1,s2 / rdmin_stat：全部随调试读数删除）

// ★★ 写 FIFO 真满粘滞位（诊断用）—— 随调试读数一起删除 ★★
//   【为什么必须看它】wfifo 的写使能在满时被 IP 内部**静默丢弃**
//   （afifo_*_256.v 里 `wr_en_s = (!full_flag) & we`），丢一个显示字 =
//   该行余下部分**永久横移一位** → 满屏横向错位缝 / 散点。
//   它就在 50MHz 域（wfifo 的写侧时钟 = sd_card_clk），不需要跨时钟域同步。
//   判读：第 1 位 = 0 ⇒ 显示写这一侧从未丢过字；= 1 ⇒ 丢过（横移/雪花坐实）。
// （wfifo_full_seen / wfifo_full_stat：删除）

// ★★ 第四轮实验（2026-10-17）：欠载事件计数器 ★★
//  第三轮结论：模式1（数据纯橙、流量不变）虚线依旧 ⇒ 虚线不在纹理数据里；
//  模式2（零纹理读）大幅缓解 ⇒ 跟随纹理读流量。
//  头号机制：显示读 FIFO 欠载（只错值不错位 ✓、随负载涨 ✓、相位锁定 ✓）。
//  本计数器直接数 rd_fifo_empty 事件（video_clk 域）：显示 [19:16] 位——
//  帧尾结构性空每帧 1 次（~60/s）→ [19:16] 每 ~76s 进 1，几乎不动；
//  若虚线=欠载，每帧几十上百次 → 读数每秒滚动多次，肉眼立判。
// （uf_ev_cnt / uf_nib_s1,s2：欠载事件计数器 —— 随调试读数一起删除）

//==============================================================================
//  数码管读数（极简版）
//==============================================================================
//  旧版这里有一大堆统计：纹理通道三标志、排空超时、写口违规现场、写命令总数、
//  write_finish 次数、FIFO 满标志、仲裁器内部快照 …… 全部删除。
//  原因：① 它们把 ext_mem_clk(125MHz) 域的逻辑撑到 114 个端点违例；
//        ② 那些指标都是给"旧仲裁器"做诊断的，仲裁器已经不存在了。
//  现在只留 5 个真正有用的读数，而且全部在 clk(50MHz) 域：
//    第0位 BUILD_TAG     固件版本（本版 = 7）
//    第1位 img_diag       每层 4 位诊断（见下）
//    第2位 seen_pix       {层1有请求,层0有请求,层1收到像素,层0收到像素}
//    第3位 层0 |want-pop| 一帧内"投进像素 FIFO 的字数"与"弹出字数"之差（饱和 0..15）
//    第4位 层1 |want-pop| 同上
//    第5位 seg_code      运行中 A=有帧 / b=无帧（装载中是实时状态码）
//  【本版(E)新增】场同步换页修"滚动横缝/撕裂"：写侧必须等显示侧在自己的场边界
//      采纳上一帧（buf_free）才允许开下一帧 ⇒ 写侧被节流到 ≤ 显示帧率(60fps)，
//      永远不会重写屏幕正在读的那块缓冲。判据：屏幕上那条"从下往上滚动的横缝、
//      图形在两处出现"应当消失；动画仍以 60fps 平滑播放。
//  【本版(0)新增】修"图片层纹理带宽饿死"（照片里那些整行黄/红/白横带）：
//      sdr_port_sched 的 tx_start 判据从 `ph==0` 改成 `ph[1:0]==0`（一个 16 拍
//      TX 槽从"只开 1 组"恢复成"每 4 拍开 1 组 = 4 组"），并把它没在内的
//      "在飞组队列"从 4 加深到 8。实测（_sim/tb_texrate.v）：
//        修前 63 组/行 → 修后 250 组/行，活跃行需求 240 组/行。
//      判据：图片层（苹果/草莓）应当**整幅清晰、无整行横带**；横带消失 = 修好。
//==============================================================================
// （bin_d0 / bin_d1x：key1 模式号、key2 读数编号 —— 随调试读数一起删除）

//  ★ build C：第 1 位 = {层1 dead 过, 层0 dead 过, 层1 此刻丢字, 层0 此刻丢字}  应恒 0 ★
//      这两件事都是"静默杀画面"：
//        dead → 该层从那一刻起全透明 ⇒ **底下场景透上来**
//               （图片区里出现"不属于这张图的颜色"就是这个）
//        ovf  → 像素 FIFO 满时丢字 ⇒ 从那里起永久错位
//      两个都改成**跨帧只置不清**，所以读数不再每帧翻转。
//      正常应恒为 0；按位读：1=层0ovf 2=层0dead 4=层1ovf 8=层1dead
// （bin_d1：删）

//  ★ build F6：第 3/4 位显示"长度差值"（饱和 0..15），取代原来的装载码 ★
//      0 = 配平；1 = 多/少一个字（正好解释"插入一个杂像素 + 后移一格"）。
//  ★ build 7：第 3/4 位改成直接盯"命令数 vs 回拍数"——这才是本轮要验的东西★
//    第3位 = {[3]环空丢过回拍, [2]幽灵回拍过, [1]命令被吞过, [0]=0}  粘滞
//             读到 0   ⇒ cmd 与 beat 严格 1:1（我的假设被证否）
//             读到 2   ⇒ 有命令被吞（数据没回来）⇒ 标签环从此永久错位
//             读到 4   ⇒ 有幽灵回拍（没发命令却回了数据）
//             读到 8   ⇒ 标签环空、回拍被挡掉
//    第4位 = (cmd - beat) 的低 4 位，实时。**稳态应恒定在一个数字**
//             （固定读延迟 ≈ 10 → 应多半显示 A）；
//             只要它单调增长、直到卡在 F，就是命令在被吞。
//  ★ build 3：第1位 = {层1dead过, 层0dead过, 层1R≠G, 层0R≠G}；第3/4位 = 层0 第一个坏字节的 高/低 半字节 ★
//    第3位+第4位 拼起来 = 收到的那个字节：00/55/AA/FF = 合法等级（取错地址）；其他值 = 不是纹理内容。
//    配合第 1 位看：第 1 位说"死过没有"，这一位说"从哪一段开始坏"。
//==============================================================================
//  ★★★ build K：数码管读数【跨域打两拍】—— 修掉唯一的时序违例 ★★★
//------------------------------------------------------------------------------
//  【实测依据】`phy_1/final_timing.rpt`（2026-10-19 03:52）：
//      Path Group : ext_mem_clk -> clk        SWNS = −4.692ns，7 个违例端点
//      Begin      : u_app/u_scene/u_img0/txhit_lat_reg[15]   ← 125MHz 域
//      End        : seg_data[4]_syn_4.do[0]                  ← 数码管第 4 段引脚
//      Logic Level: 7（LUT4×4 + LUT5×3），Data Path 8.279ns，预算只有 4.0ns
//  ⇒ **7 个违例端点全是数码管的 7 个段**。数据通路本身是过的
//    （sd_card_clk +0.229、ext_mem_clk +0.106、video +28.942、hdmi_5x +5.546）。
//
//  【为什么原来会这样】`img_diag` 是 125MHz 域里一组寄存器的**组合 mux 输出**，
//  直接连到 50MHz 域的 `seg_decoder / seg_scan`，中间只有组合逻辑 ⇒
//  7 级 LUT + 8.3ns 走线，跨到另一个时钟域的输出寄存器上。
//  这是"多比特慢变信号跨域"，功能上不会错（一帧才变一次，数码管 1kHz 扫描），
//  但 STA 会一直报，而且**掩盖真正要看的东西**。
//
//  【修法】在 50MHz 域里先打两拍（标准两级同步），把那条长组合链切断：
//      txhit_lat(mclk) → diag mux（1~2 级 LUT）→ diag_s1 → diag_s2 → 译码/扫描
//  ⇒ 跨域段只剩 1~2 级 LUT，4ns 预算绰绰有余；且读数本身也不再是亚稳态采样。
//==============================================================================
// （diag_s1/diag_s2：图层诊断读数的两级同步 —— 随调试读数一起删除）

// （诊断页 / 读数同步 / FIFO 粘滞位：已随调试接口整体删除）
wire [1:0]  ph_off_sel = 2'd0;              // 组相位已扫过（0..3 全 FFF），固定 0
// ★ 实验 2 结论（2026-10-21）：tx_ph_off 扫描 0..3 —— 0 最好，2/3 只是把
//   "帧缓存串入"换成"邻居字丢线"，无一完美 ⇒ 相位不是根因，根因是组命令数
//   mod 4 漂移（tx_retry 全量重发，已在 sdr_port_sched 修复）。
//   ⇒ tx_ph_off 固化 0；诊断页 0x31..0x34 的扫描接线保留但恒 0（不再影响）。
wire [1:0]  tx_ph_off_sel = 2'd0;

// （bin_d3 / bin_d4 / bin_d2 / bin_d5：丢字标志等诊断读数，随调试接口一起删除）
//  ★ 2026-10-26：诊断页选择 / 读数同步 / FIFO 粘滞位 / 按键·开关位图
//    全部随"只保留 4 项读数"的清理一起删除，见下面数码管数据段。
//----------------------------------------------------------------------
//  数码管：2026-10-28 整体移除（T2 资源释放）
//----------------------------------------------------------------------
//  【删了什么】seg_scan_m0 + 4 个 seg_decoder（fps/img/fx/ldp）+ ld_prog 两拍
//    跨域同步链，共约 74 LUT。原读数页（SEL0 帧率/10、SEL1 图片号、
//    SEL2 过渡模式、SEL3 装载进度、SEL4/5 全灭）随之失效。
//
//  【为什么能删】本工程全部靠仿真台验证（skills/osd-sim-verify：7 个台 +
//    出图工具 + 断言），板载读数不再是唯一观测手段；LUT 已到 97.4%，
//    优先让给功能。fps_dbg 因此成为无消费者的死逻辑，TD 会连同帧率计数链
//    一起剪掉（源码注释估约 38 FF + 45 LUT）。
//
//  【怎么恢复】git revert 本次提交即可 —— 整段在一个 commit 内。
//    恢复要点：ld_prog 必须跨域两拍同步（sd_card_clk → clk），否则第 4 位随机抖。
//
//  【端口保留】seg_sel / seg_data 在 pin.adc 里有物理约束，不能从端口表删。
//    段码全 1 = 全灭（共阳极），位选取什么都不影响。
//----------------------------------------------------------------------
assign seg_sel  = 6'b111111;
assign seg_data = 8'hFF;

//==============================================================================
//  【视频链路】时序生成与帧缓存读取
//==============================================================================

// ---- 第 1 步：产生行/场同步信号 ----
// 这个模块内部调用 color_bar 产生 hs/vs/de 时序（颜色部分丢弃不用）。
// 它还会在"场同步边沿"产生一次 read_req，通知帧缓存"准备下一帧数据"。
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_req          (video_read_req),      // 输出：请求读一帧
    .read_req_ack      (video_read_req_ack),  // 输入：帧缓存已开始准备

    // ---- ★ 场同步换页（修撕裂）：显示指针只在场边界切换 ★ ----
    .buf_commit_idx    (disp_commit_idx_v),   // 写侧已提交的块号（已同步）
    .buf_commit_tgl    (disp_commit_tgl_v),   // 写侧提交事件（已同步）
    .disp_rd_idx       (disp_rd_idx),         // 输出：显示真正在读的块号
    .disp_adopt_tgl    (disp_adopt_tgl),      // 输出：已换页事件（回给写侧同步）

    .hs                (hs_0),                // 输出：行同步（还没延迟对齐）
    .vs                (vs_0),                // 输出：场同步
    .de                (de_0)                 // 输出：数据有效
);

// ---- 第 2 步：数据与同步信号对齐（延迟 20 拍）----
// 【为什么要延迟】
// 像素数据从 SDRAM 经过 FIFO 读出来是有延迟的（几百 ns 级别）。
// 如果直接把同步信号接到 HDMI 上，会出现"同步信号已经进入有效区，
// 但数据还没出来"的错位，画面就会整体偏移或撕裂。
// 所以这里把 hs/vs/de 也延迟同样的拍数，让它们和像素数据重新对齐。
//
// 模块内部用 21bit 移位寄存器（hs_d/vs_d/de_d）记录历史，
//   read_en = de_d[18]  → 提前 18 拍就去 FIFO 取数据（给 FIFO 留出延迟余量）
//   vout_data 在 de_d[19] 时采数据
//   hs_r/vs_r/de_r 输出 de_d[20]，即延迟 20 拍
video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_en           (video_read_en),          // 输出给读 FIFO：取一个像素
    .read_data         (video_read_data[31:8]),  // 输入像素：只取高 24bit（RGB888）
    .hs                (hs_0),                   // 输入未延迟的同步信号
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs),                     // 输出延迟对齐后的同步信号
    .vs_r              (vs),
    .de_r              (de),
    .vout_data         (vout_data_raw)           // 输出延迟对齐后的像素数据
);

// ---- 第 2.5 步：显示端画质后处理（2026-10-26 新增）----
// 【它在链路里的位置】video_delay 对齐之后、黑屏门控与 rgb_to_axis 之前。
// 【做什么】亮度/对比度/色温/饱和度 + 风格（黑白/反色），全部点操作、无行缓存。
// 【参数】pic_* 来自 OSD 菜单（sd_card_clk 域，目前接常量默认值 = 恒等直通），
//         模块内部自带跨域同步 + vs 边沿快照（改参数下一帧才生效，零接缝）。
// 【对齐】hs/vs/de 与数据一起在模块内延迟 3 拍，相对对齐关系保持不变，
//         不接触 video_delay 的 read_en（FIFO 读时序零影响）。
// 【回归基线】全默认参数下逐位恒等（单元仿真已验证）——上板画面必须与
//         改动前完全一致，若不一致立刻回退本实例。
//------------------------------------------------------------------------------
//  ★ 2026-11-03 [ALS]：环境光自动亮度
//------------------------------------------------------------------------------
//  als_adc（audio_mclk 12.288MHz，不新增 PLL）：ADC CH1(M10) → 曲线 → EMA → 目标亮度
//  als_bright（sd_card_clk）：OSD"自动亮度"勾选且传感器在位 ⇒ 用 ALS 目标，
//    否则用手动滑条 pic_bright；其余画质参数不经过它（仍是 pic_* 直连）。
wire [6:0] als_tgt;
wire       als_tgl;
wire       als_ok;

als_adc #(
    .CLK_HZ (12_288_000),
    .ADC_CH (1)
) u_als_adc (
    .clk     (audio_mclk),
    .rst     (rst_all),
    .tgt     (als_tgt),
    .tgt_tgl (als_tgl),
    .ok      (als_ok)
);

als_bright u_als_bright (
    .clk            (sd_card_clk),
    .rst            (rst_all),
    .auto           (osd_auto),
    .pic_bright     (pic_bright),
    .pic_tgl        (pic_tgl),
    .als_ok_a       (als_ok),
    .als_tgt_a      (als_tgt),
    .als_tgl_a      (als_tgl),
    .pic_bright_eff (pic_bright_eff),
    .pic_tgl_eff    (pic_tgl_eff)
);

// ---- 第 3 步：频谱条叠加（2026-11-02，FFT_Plan.md）----
// 【插在哪】帧缓存之后、video_postproc 之前，video_clk 域。
//   · 音频样本（src_l_w）、FFT、像素扫描同在 video_clk ⇒ 零 CDC（只有 2 个 1 位同步器）。
//   · 50MHz 域一根线都不加（该域 WNS 已为负）。
//   · hs/vs/de 不经过叠加模块、零延迟；像素只做组合二选一 ⇒ 不可能错位。
// 【取样点】src_l_w（audio_src_sel 输出的纯音乐），不受音量/静音/提示音影响。
// 【开关】osd_spc（OSD CheckBox，默认关）& ~osd_on（菜单打开时隐藏）。
wire        spc_vbl;                // 最后有效行结束单拍 → FFT 开工
wire [3:0]  spc_idx;                // 扫描侧条号（预取）
wire [6:0]  spc_h;                  // 条高 0..127

spec_fft u_spec_fft (
    .clk        (video_clk),
    .rst        (rst_all),
    .smp_valid  (src_valid_w[0]),   // ★ src_valid_w 声明成了 24 位，只有 bit0 有驱动
    .smp        (src_l_w),
    .vbl        (spc_vbl),
    .scan_idx   (spc_idx),
    .scan_h     (spc_h)
);

spec_bar_ovl u_spec_ovl (
    .clk        (video_clk),
    .rst        (rst_all),
    .in_vs      (vs),
    .in_de      (de),
    .in_rgb     (vout_data_raw),
    .spc_a      (osd_spc),          // sd_card_clk 电平，模块内 2FF 同步
    .menu_a     (dbg_osd),          // = App 的 osd_on（菜单打开），模块内 2FF 同步
    .bar_idx    (spc_idx),
    .bar_h      (spc_h),
    .vbl        (spc_vbl),
    .out_rgb    (vout_ovl)
);

// ---- 第 4 步：动态字幕叠加（2026-11-02 新增，2026-11-07 双缓冲）----
// 【位置】频谱条之后、video_postproc 之前 —— 显示链路的最后一个叠加点，
//         字幕画在频谱条上面（字幕行 456..471）。
// 【对齐】spec_bar_ovl 的 vs/de 零延迟、像素组合二选一 ⇒ vout_ovl 与 vs/de 同拍；
//         本模块把 vs/de/rgb 一起再延迟 1 拍（字模 BRAM 读延迟），下游进入后处理。
// 【零影响】只改字形命中的像素，其余像素逐位直通。
dyn_subtitle u_dyn_subtitle (
    .video_clk  (video_clk),
    .rst        (rst_all),
    .vs         (vs),
    .de         (de),
    .rgb_in     (vout_ovl),
    // ---- 门控 / 参数（sd_card_clk 域）----
    .show_en    (sub_show_en),
    .commit_tgl (sub_commit_tgl),
    .bank       (sub_bank),
    .text_len   (sub_len),
    // ---- 字形 RAM 写口（sd_card_clk 域，sub_loader 驱动）----
    .wr_clk     (sd_card_clk),
    .wr_addr    (sub_wr_addr),
    .wr_data    (sub_wr_data),
    .wr_we      (sub_wr_we),
    // ---- 给 sd 域的帧脉冲 ----
    .vs_tgl     (sub_vs_tgl),
    .vs_out     (sub_vs),
    .de_out     (sub_de),
    .rgb_out    (sub_data)
);

// ---- 第 5 步：显示端画质后处理（2026-11-03 移到频谱/字幕之后）----
// 数据通路：sub_data（频谱+字幕已叠加）→ video_postproc → 黑屏门控 → rgb_to_axis
// 总延迟 = 字幕 1 拍 + 后处理 3 拍 = 4 拍，与移动前完全相同。
video_postproc #(
    .DELAY             (3)
) video_postproc_m0 (
    .video_clk         (video_clk),
    .rst               (rst_all),
    .in_hs             (hs),
    .in_vs             (sub_vs),
    .in_de             (sub_de),
    .in_rgb            (sub_data),
    .pic_bright        (pic_bright_eff),     // ★ 2026-11-03：手动 / 自动亮度二选一
    .pic_contrast      (pic_contrast),
    .pic_temp          (pic_temp),
    .pic_sat           (pic_sat),
    .pic_style         (pic_style),
    .pic_tgl           (pic_tgl_eff),        // ★ 含 pic_tgl 的全部事件 + ALS 目标变化
    .out_hs            (pp_out_hs),
    .out_vs            (pp_out_vs),
    .out_de            (pp_out_de),
    .out_rgb           (pp_out_data)
);

// ---- 第 6 步：首图未就绪时强制黑屏（最终输出级）----
// 【为什么需要】系统刚上电时，SDRAM 里还没有任何图片数据（内容是随机的）。
// 如果直接显示，屏幕上会出现雪花噪点，观感很差。
// 所以用 display_valid 做门控：首图提交完成前，一律输出黑色（24'd0）。
// 【为什么放在最后】门控必须在后处理【之后】，否则亮度/对比度会把"无画面"
// 抬成灰色而非纯黑；同时频谱/字幕也一起被压黑。
assign vout_data = display_valid ? pp_out_data : 24'd0;

// ---- 帧缓存读写控制器（视频链路的"交通枢纽"）----
// 这个模块内部有两个异步 FIFO + 两个搬运行状态机，把"简单的读写握手"
// 翻译成"SDRAM 的突发读写"。用户只需要说"我要读一帧 / 我要写一帧"，
// 具体每 256 个数据打一次突发这些细节都在内部处理。
frame_read_write #(
    // ★ 100M-37：关掉行翻转这个实验**无效**，已恢复 1。
    //   原因：坏条是"固定 x、全屏高"，而行翻转只把行号换成 H-1-r，
    //   **x 仍然映射到同一个列偏移** ⇒ 翻转开/关，坏列位置都不会变。
    //   （顺带说明：`frame_fifo_write` 的默认参数被本文件覆盖，改那边的默认值
    //     不会生效；要改只能改这里。）
    .WRITE_V_FLIP     (1),    // 1 = 行翻转（BMP 需要）
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480),
    //  ★ 关掉引擎间互锁 ★ 互斥由 u_sched 的时隙表保证。
    //    留着它会让读写轮流霸占时间（每 1024 拍才走 128+128 字），
    //    每行 3975 拍根本喂不饱 640+640 ⇒ 帧永远收不满 ⇒ 纯黑。
    .INTERLOCK        (0)
) frame_read_write_m0(
    // ---- SDRAM 侧接口（mem_clk = 125MHz 域）----
    .mem_clk           (ext_mem_clk),
    .rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    // ★ 读口不再直连 SDRAM，而是经过 sdr_rd_arb 仲裁 ★
    .App_rd_en         (fw_rd_en),
    .App_rd_want       (fw_rd_want),
    .App_rd_addr       (fw_rd_addr),
    .Sdr_rd_en         (fw_rd_vld),
    .Sdr_rd_dout       (fw_rd_data),
    .rd_ready          (fw_rd_ready),
    // ★ 读写互斥由调度器的时隙表保证（RD/TX 与 WR/LD 是不同槽），
    //   两个引擎之间不再需要互相"看脸色"，这里恒接 0。★
    .rd_busy_ext       (1'b0),
    .wr_ready          (fw_wr_ready),         // ★ 写口放行：装载 4 字组期间拉低
    .wr_busy_o         (fw_wr_busy),          // 保留观察点（无逻辑使用）
    .wr_fifo_used      (wr_fifo_used),        // ★ 写 FIFO 水位 → 读仲裁器让路判据 ★
    .wr_fifo_full      (wr_fifo_full),        // ★ 写 FIFO 真满（丢字）★
    .rd_buf_used       (rd_buf_used),         // ★ 显示读 FIFO 占用 → 读仲裁器让路 ★

    // ---- 读侧（video_clk = 25MHz 域）----
    .read_clk          (video_clk),             // 读侧时钟
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),                      // 不需要，留空
    .read_addr_0       (BUF0_ADDR),             // 缓冲区0 起始地址
    .read_addr_1       (BUF1_ADDR),             // 缓冲区1 起始地址
    .read_addr_2       (24'd0),                 // 未使用的缓冲区2（本工程只用双缓冲）
    .read_addr_3       (24'd0),                 // 未使用的缓冲区3
    .read_addr_index   (disp_rd_idx),           // ★ 显示真正在读的块号（场边界才换）
    .read_len          (FRAME_PIXELS),          // 读多长 = 一帧像素数
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    // ---- 写侧（sd_card_clk = 50MHz 域）----
    // ★ 写口也不再直连 SDRAM，而是经过 sdr_wr_arb 仲裁 ★
    .App_wr_en         (fw_wr_en),
    .App_wr_addr       (fw_wr_addr),
    .App_wr_din        (fw_wr_din),
    .App_wr_dm         (fw_wr_dm),

    .write_clk         (sd_card_clk),           // 写侧时钟
    .write_req         (app_write_req),
    .write_req_ack     (app_write_req_ack),
    .write_finish      (frame_write_finish),    // 输出：整帧写完的脉冲
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (24'd0),
    .write_addr_3      (24'd0),
    .write_addr_index  (write_buf_idx),         // 用哪个索引 → 决定写哪块（双缓冲切换点！）
    .write_len         (FRAME_PIXELS),
    .write_en          (app_write_en),
    .write_data        (app_write_data),

    // ★ 诊断：读 FIFO 当前可读字数（视频端余量）
    //   ★ 注意：rfifo 的 rdusedw 是 9 位差值，水位 512 时回绕成 0 ——
    //     别再用它判欠载！判读改用下面两个硬标志（第 2 位 = {空,满} 粘滞）。

    // ★ 2026-10-17 新增：读 FIFO 自身的空/满硬标志 → 数码管第 2 位 ★
    //   empty（video_clk 域）= 真空（视频拿到旧数据 = "变亮"虚线）
    //   full （mem_clk  域） = 打满（水位判据 9 位回绕 → 丢字横移的地雷）
    .rd_fifo_empty     (rd_fifo_empty),
    .rd_fifo_full      (rd_fifo_full),

    // ★ 2026-10-18：写 FIFO 写侧水位 → App 的 wr_stall（背压）★
    .wr_inflight       (wr_inflight),

    // ★ build FB16：帧缓存格式（888 / RGB565）动态切换 ★
    .fmt_wr            (fb_fmt565),           // 写侧：本帧要不要 565
    .fmt_rd            (fb_fmt_disp),         // 读侧：正在显示那块缓冲的格式
    .buf_fmt_o         (fb_buf_fmt)           // 两块缓冲各自的格式（写侧记录）
    // （.dbg_wr：写侧行走诊断，2026-10-26 随数码管清理一起删除）
);

//==============================================================================
//  【SDRAM 读口仲裁】显示读 ↔ 纹理读（★2026-10-18 v2：厂商地址规则合规★）
//==============================================================================
//  【v2 结论从哪来】见 SD/sdr_rule_check.v 里抄录的厂商文档（APUG011 §2.3.4/§2.4）：
//      每次读/写的数据以 4 个为单位，每 4 个中第 1 个地址即为首地址，
//      第 2~4 地址依据首地址做累加；每 4 个时钟可以任意跳跃 1 次地址，
//      跳跃目标必须 4 字对齐。
//  违反它控制器不会报错，只会悄悄拿"自动累加地址"取数 ⇒ 显示错值（虚线）、
//  纹理错值（花屏）。v1 的"逐拍看写口脸色 + owner 切换"把显示突发切碎在组中间，
//  纹理又是"一像素一命令"的乱跳地址 ⇒ 两头都违规。
//
//  【v2 改法】
//    · 显示读突发：只在自己的回合放行，回合只在 want 掉 0（128 条命令发完）后
//      才切换 ⇒ 突发永远整段；起点 4 字对齐（BURST_SIZE=128、帧基址对齐）。
//    · 读写单口纪律：改回厂商契约（写引擎看 App_rd_busy，其中含 tex_busy）；
//      装载写靠 rd_active 让路 ⇒ 读口不再需要 wr_act 逐拍暂停。
//    · 纹理读：改成"4 字一组取数"（组首 4 字对齐、4 连发），同组请求共享一次
//      取回 ⇒ 地址流天然合规，且不需要行缓存 BRAM、不预缩放（缩放动画随便做）。
//    · 那一堆 exp_* 实验开关全部删除（结论已出，留着只会误开）。
//==============================================================================
//==============================================================================
//  【SDRAM 用户端口调度器】固定 16 时隙表，每槽 16 拍（同方向成段）
//==============================================================================
//  这是整条存储通路上唯一的调度逻辑，全部内容 = 一张常数表 + 两个计数器：
//      槽号 :  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15
//      owner: RD  RD  RD  RD  TX  TX  TX  TX  TX  ID  WR  WR  WR  WR  LD  ID
//  读段（RD×4 + TX×5 = 144 拍）→ ID（16 拍）→ 写段（WR×4 + LD = 80 拍）→ ID
//  ⇒ 每 256 拍只换向 2 次（旧表换向约 43 次），且每次换向前都有 16 拍空闲给
//    SDRAM 做 tWTR/tRTW 和突发排空。这就是修"全屏雪花"的那一刀。
//  ★ 本版（build 1）把 build F 的 RD5/TX4 换成 RD4/TX5 ★
//    读段总长仍是 9 槽 = 144 拍、换向仍只有 2 次 ⇒ 雪花那一刀原样保留；
//    换 1 个槽的目的是补纹理带宽（"开图片掉帧"的那一刀）：
//        显示读 4 槽 = 64 字/256 拍 ⇒ 1000 字/行 ≥ 需求 640 ✓
//        纹理   5 槽 = 20 组/256 拍 ⇒  312 组/行 ≥ 需求 240 ✓（原 250 只有 1.04×）
//
//  带宽核算（每行 3180 拍 @100MHz）：RD 4槽=60字/256拍(需求640/3180)、WR 4槽=64字/256拍、
//  TX 6槽=24组/256拍(峰值需求240组/行)、LD 1槽(只在开机)、ID 2槽(换向缓冲+刷新)。
//  每个源的需求都小于分给它的时间 ⇒ 不会饿死。
//  ★ 本版把 1 个显示读槽挪给纹理（RD 5→4 / TX 4→5）★ —— 治"开图片掉帧"：
//    纹理 250 → 312 组/行（需求 240）。忙停止（BUSY_HOLD）build 1 试过，板上
//    全线花屏 ⇒ 已改回 0，见下面 u_sched 的参数注释。
//
//  ★★ 2026-10-21 build 100M-19：把"看 Sdr_busy"从纹理通路里彻底拿掉 ★★
//    板上事实链（每一格都是整版烧录的结果）：
//      100M-16  守卫开（忙即 abort）      → 虚线消失、开始掉帧
//      100M-17  守卫开 + TX 头部静音 8 拍 → 掉帧依旧
//      100M-18  时隙表换位、TX 头部全干净 → 掉帧依旧 + **白色竖条**
//      125MHz   产能 +25%                 → 掉帧只"好一点" + 花屏
//    ⇒ ① 掉帧与纹理产能无关：100M-18 供给 312 组/行，实际需求只有 ~160 组/行
//         （两图 y 不重叠，每条线只有一个 240×180 框，K=2.667 ⇒ 640 字 = 160 组），
//         余量 1.9 倍，照样掉。
//       ② 白色竖条 = 100M-18 的换位把**读→写换向从 36 拍压到 20 拍**（唯一变紧处），
//         写命令压进读回程 ⇒ 每条线同一时刻坏一批字 ⇒ 屏幕上一条竖的。
//    ⇒ 本版：① 时隙表**回退**到 RD 0-3 │ TX 4-9 │ ID │ WR 11-14 │ LD|ID（红线复位）；
//             ② TX_BUSY_GUARD = **0**（纹理通路完全不再看 Sdr_busy）；
//             ③ 虚线改由**确定性静音**兜：TX_LEAD_GAP = 8
//                （末条读命令 ≤ 槽3 ph11，回程 11 拍 ⇒ 槽4 ph0~7 必在忙里，直接静音）。
//    产能：TX 88 拍/轮 ≈ 273 组/行 vs 需求 ~160 ⇒ 1.7 倍，够。
//    ⚠ 复测点：dbg_sel=0 的 tex_bad。0 = 虚线根因就是那个回程阴影（本版收工）；
//       几百 = 只剩"刷新落在组中间"的残余（概率 4/3125），可再决定要不要回守卫。
//
//  ★★ 2026-10-21 build 100M-20：TX_BUSY_GUARD 重新打开（两图层重叠必需）★★
//    新增事实：两框竖直**不重叠**时，两层共用同一套 (x,y) 扫描，永远不会同时有
//    纹理请求在飞 ⇒ TX 段只用掉一半，组间有空档，忙窗掉进空档里，守卫确实不必开。
//    一旦有**哪怕一行**重叠，两层同时要货 ⇒ TX 段被填满、组与组首尾相接 ⇒ 任何忙窗
//    （读回程尾巴 / 写换向 / 内部刷新）都必然落在组中间。此时守卫关着 ⇒ 这一组照发，
//    控制器按"显示组首+相位"重排 ⇒ 纹理命令读回**帧缓存内容** ⇒ 图片框里混进屏幕
//    别处的像素（板上现象：任意一行重叠就"干扰"，而且错的是框外内容）。
//    ⇒ 本版：TX_BUSY_GUARD = 1（组内撞忙冻结整表；abort/retry 仍不参与，mod-4 不漂）。
//    仿真（tb_txfix，双层饱和 + 真实回程忙）：286 组/行不变，abort=0，数据错 0，
//    分组错 0 ⇒ 守卫的代价在本设计里几乎为零（冻结不消耗时隙表时间）。
//
//  ★★ 2026-10-21 build 100M-21：框外白点的根因（刷新冻结把 RD/WR 的组打断）★★
//    现象：图片框内干净了，但框外仍随机出现白色像素（白点/白块），与图片位置无关。
//    根因（仿真已复现）：顶层 BUSY_HOLD=1 ⇒ 刷新期间冻结整张时隙表。冻结时 ph 停住
//      是对的，但 `rd_run`/`wr_run` 被清 0 ⇒ 解冻后 RD/WR 从"下一个 4 拍边界"重新
//      起突发；而控制器的 4 命令分组相位并没有复位 ⇒ 新突发的前几条被当成上一组的
//      余项，地址按 grp_head+ph 重算 ⇒ **读回帧缓存别处的内容**（= 框外随机白点），
//      写同理（= 白色竖条）。
//    ⇒ 本版：冻结期间 `rd_run`/`wr_run` **保持**（组状态原地不动），解冻后接着发。
//    --- 仿真对照（tb_txfix 打开控制器刷新窗口 REF_PERIOD=3125 / REF_BUSY=20）---
//      修前：读回程错 605   写错 627   分组错 14883 → FAIL
//      修后：读回程错 0     写错 0     分组错 0     → PASS（REF_LAG=0/1 都过）
//    另加 WR_HEAD_GAP=4：TX_TAIL 允许组尾进槽 10 ph0~3，其回程到 ph14，而 WR 原来从
//      槽 11 ph0 立刻起跑 ⇒ 写命令压进读回程（白竖条）。静音 4 拍后实测
//      TX→WR 最小间隔 21 拍 ≥ CAS(11)+2；写供给 739 字/行（需求 640）。
//    ⚠ 上板复测：dbg_sel=3 = 每帧 Sdr_init_ref_vld 脉冲数，期望 380~400。
//      读到 0 ⇒ 控制器运行期不报刷新 ⇒ BUSY_HOLD 没生效，白点残余只能靠降低
//      RD/WR 命令密度或提频，届时再看。
//    叠加重叠：双层需求 320 组/行，实测供给 283 组/行（缺口 11.7%）。纸面帧时
//      13.2ms（写口限速）+ 180 行×8.5µs ≈ 14.7ms < 16.7ms；若上板仍 30fps，
//      读 dbg_sel=15 的 hold 占比 —— 那是真机 Sdr_busy 占空比比模型高的证据，
//      此时唯一的解是降需求（源 320×240 / 框宽 200）。
//
//  ★★ 2026-10-21 build 100M-22：白竖带 + 30fps 的定稿修法 ★★
//    【100M-20/21 的错】把"组内撞忙"做成**冻结整张时隙表**。两处代价都致命：
//      ① 真控制器的 4 命令分组 = "读口上连续的 4 条命令"。冻结把一组拆成两段发，
//         后半段被当成新组组首（用自己的地址当 head，再按 head+ph 重算后 3 条）
//         ⇒ 读回 base+1..base+4（错位一组）⇒ 落到帧缓存区，而背景是白的
//         ⇒ 板上就是"整列纯白"，且冻结相位每轮固定 ⇒ 同一列每行都错 ⇒ 竖带。
//      ② Sdr_busy 在真机是"控制器正在干活"（读回程/刷新/换向都拉高），占空比
//         远高于仿真的 0.4%。组内一撞就停整表 ⇒ RD/WR 一起停 ⇒ 帧时超期 ⇒ 30fps。
//    【本版规则】一个组的 4 条命令必须 **4 拍连续发完**，中间不许任何停顿：
//      · tx_cmd 去掉 !tx_busy_blk（忙只在 tx_start 决定"能不能开这一组"）
//      · hold 去掉 tx_stall（不再因忙冻结整表；只保留刷新冻结，手册硬性要求）
//      · TX_LEAD_GAP=8 继续承担"显示读回程阴影"的确定性静默
//      · TX_TAIL=4 不动；WR_HEAD_GAP 改回 0（读→写=17 拍，正是原设计跑过多年
//        的值）⇒ 写口回到 64 字/轮 = 12.3ms/帧（原 60 字/轮是 13.1ms），
//        帧期余量从 3.6ms 回到 4.4ms —— 重叠行 320 组/行 才有地方借。
//    仿真（tb_txfix，含 TURN_CYCLES=16 换向丢弃模型 + 刷新窗口）：
//      MODE=0/4 饱和 283 组/行、MODE=1/3 需求 240/240，全部 0 错、0 分组错、0 换向丢。
//    【新增读数】dbg_sel=12 = 每帧"显示读回程里纯白字(FFFFFFFF)"的个数。
//      ⚠ 100M-23 更正：**背景本身就是纯白**（bg_rect 铺满全屏、bg_col=FF FF FF FF），
//        所以这个数基线极大，只能纵向对比，不能当"帧缓存变白"的判据。
//        真正的判据仍是 dbg_sel=0 (tex_bad) 与 dbg_sel=14 (last_sig)。
//    【100M-23 已验证 / 已否决】
//      · TB 改用严格口径 PHASE_MODE=1（按拍分组，即厂家规则）后，本版仍然
//        0 组错 / 0 读回程错 / 0 写错 —— 说明 **调度器的地址与数据通路是干净的**，
//        "组内停顿 140~196 次/30 行"都是 4 拍的整数倍，对控制器无害。
//      · "组在飞时冻结 ph4" 试过 ⇒ **死锁**（ph4 停 ⇒ stall 永远等不到相等 ⇒
//        整表停死，组数掉到 1）。已回退，注释留在 sdr_port_sched.v 的 ph4 处。
//      ⇒ 白带不在 SDRAM 调度路径上，下一步应查应用层（图形层输出/合成/写使能）。
//==============================================================================
sdr_port_sched #(
    .DIAG_KNOBS (0),               // 诊断口全接常数 0 ⇒ 锁存/选择逻辑消掉
    //  标签环 256 深：显示读突发是 128 条命令背靠背发出的，都会同时在途；
    //  再加纹理 4 组 + FIFO 余量 ⇒ 64 深不够，256 深有 1.6 倍余量。
    .TAG_AW (8),
    .TEX0_BASE (TEX0_ADDR[20:0]),
    .TEX1_BASE (TEX1_ADDR[20:0]),
    //  ★★ 忙停止 = 1，但**只按 Sdr_init_ref_vld**（build 6 起）★★
    //  【build 1 的教训】当时是 `Sdr_busy | Sdr_init_ref_vld` 一起按 ⇒ 全屏乱码。
    //  原因是 `Sdr_busy` 是"控制器正在干活"的状态位（经常为高），
    //  手册表 3-1 对它写的是 **"暂不使用"**；把它当门用会把用户口饿死。
    //
    //  【现在按的是谁】只按刷新标志。手册 §2.3 原文：
    //      "在用户对虚拟 ERAM 做操作前……如果确认忙，
    //        请等待 Sdr_init_ref_vld 信号变为低电平后，再做读写操作。"
    //  刷新窗口 20 拍 / 3906 拍 = 0.5% 时间，冻结期间时隙表不消耗时间，
    //  客户端不丢配额 ⇒ 代价可以忽略。
    .BUSY_HOLD (1),
    //  ★★ build F4（甲）：发命令前确认控制器"能收" ★★
    //  【和 BUSY_HOLD 完全不同，别混】
    //      BUSY_HOLD：忙时**冻结整张时隙表** —— build 1 就是死在"Sdr_busy 常为高
    //                 ⇒ 表走不动 ⇒ 显示读/写饿死 ⇒ 全屏乱码"。
    //      BUSY_GATE：**只让四个"启动时刻"等一等**（纹理开组 / 显示读起突发 /
    //                 显示写起突发 / 装载起组），时隙表照常走。
    //                 ⇒ 不会饿死任何人，最坏只是某个突发晚几个 4 拍边界启动。
    //  【为什么能减少拒收】手册 §7 第 3 条要求"每次读写操作前先判断 Sdr_busy"。
    //  被拒收的命令没有任何回程拍 ⇒ 标签环整体错一位 ⇒ 数据互相串且不可检测。
    //  在"起头"那一刻避开忙，就能让整串/整组都落在控制器能收的窗口里。
    //  ★★ 板上已实测：=1 会让纹理塌成横条纹，故定稿 =0 ★★
    //  原因：手册表 3-1 对 Sdr_busy 写的是"暂不使用"；参考设计 app_wrrd.v 里
    //  它连应用层端口都没进，只按 Sdr_init_ref_vld 停机。用它门控"启动"会把
    //  纹理开组的机会大量挡掉 ⇒ 供给塌陷。开关保留，便于以后 A/B。
    .BUSY_GATE (0)
    //  ★ 读数据错拍补偿改成了【运行时端口】rd_lag（见下面 .rd_lag 的连接）★
) u_sched (
    .clk          (ext_mem_clk),
    .rst          (rst_all),

    // ---- 显示读 ----
    .disp_rd_en   (fw_rd_en),
    // ★ 2026-10-22：未门控的"想读"，专供 sdr_port_sched 的借槽判闲
    //   `fw_rd_en` 是 `App_rd_en_d0 & rd_ready`，拿它判闲会顺序锁死（满屏黄）。
    .disp_rd_want (fw_rd_want),
    .disp_rd_addr (fw_rd_addr),
    .disp_rd_ready(fw_rd_ready),
    .disp_rd_vld  (fw_rd_vld),
    .disp_rd_data (fw_rd_data),

    // ---- 显示写 ----
    .disp_wr_en   (fw_wr_en),
    .disp_wr_addr (fw_wr_addr),
    .disp_wr_din  (fw_wr_din),
    .disp_wr_dm   (fw_wr_dm),
    .disp_wr_ready(fw_wr_ready),

    // ---- 纹理读 ----
    .tex_req_vld  (tex_req_vld),
    .tex_req_addr (tex_req_addr),
    .tex_req_mask (tex_req_mask),
    .tex_req_ack  (tex_req_ack),
    .tex_pix_wr   (tex_pix_wr),
    .tex_pix_data (tex_pix_data),

    // ---- 装载写（bmp_loader / 占位图 二选一，见 u_ph 段）----
    .ld_wr_vld    (sdr_ld_vld),
    .ld_wr_used   (sdr_ld_used),
    .ld_wr_addr   (sdr_ld_addr),
    .ld_wr_data   (sdr_ld_data),
    .ld_wr_ack    (ld_wr_ack),
    .ld_wr_settled(ld_wr_settled),   // ★ build 100M-15：装载写“已沉降”标志

    // ---- SDRAM 用户口 ----
    .App_rd_en    (App_rd_en),
    .App_rd_addr  (App_rd_addr),
    .App_wr_en    (App_wr_en),
    .App_wr_addr  (App_wr_addr),
    .App_wr_din   (App_wr_din),
    .App_wr_dm    (App_wr_dm),
    .Sdr_rd_en    (Sdr_rd_en),
    .Sdr_rd_dout  (Sdr_rd_dout),
    .Sdr_busy     (Sdr_busy),              // 控制器忙（BUSY_HOLD=1 时冻结时隙表）
    .Sdr_init_ref_vld (Sdr_init_ref_vld),  // 刷新进行中（同上，第二个"别发"信号）
    .App_ref_req      (App_ref_req),       // ★ 纹理组首避开 REF 那一拍
    .rd_lag        (1'b0),         // 同拍取数（板上已定论，见文件头）
    // ★ 实验 2：纹理组相位实时扫描 —— 诊断页 0x31..0x34 = 0..3 ★
    .tx_ph_off     (tx_ph_off_sel),
    .exp_mode     (4'd0),          // ★ build N：同上 ★
    //  【为什么必须硬接】这原来接的是 key1 翻转的寄存器（`rd_lag_m`）。
    //  `rd_lag=1` 时「回拍使能」迟 2 拍、「数据」迟 1 拍 ⇒ 每一条命令配到的
    //  都是**下一条命令**的数据：
    //      · 4 字组内 → 拿到相邻字，仍是纹理 ⇒ 只表现为"红点"
    //      · 组里第 4 个字 → 下一条命令是**显示读**（帧缓存）⇒ 拿到非纹理数据
    //        ⇒ **白点**，而且**每 4 个像素一个有规律地出现** ⇒ "全屏都有条纹"
    //  ⇒ 也就是说：**只要有人按过一次 key1，之后所有结构性修复都会显得"毫无作用"**。
    //  实测（build 8 注释）rd_lag=1 会让整屏左移 1 像素，说明 1 才是"数据晚一拍"；
    //  而本设计从 SDRAM IP 取数时 en 与 dout 是**同拍**的，正确值就是 0。
    //  这个开关是调试遗留，留在固件里等于埋了一颗"按一下就全错"的雷，故删除。
    // ★ build T：组相位改成【按键实时扫】—— key1 = 0/1/2/3 就是 ph_off = 0/1/2/3 ★
    //   4 字组相位和控制器内部 4 拍边界差 k，控制器就会用"组首+它自己的相位"重算地址
    //   ⇒ 取回别处的数据。停掉显示读后仍有 ~1930 个字错，这里就是唯一的旋钮。
    .ph_off       (ph_off_sel),

    // 调度器只保留一根有逻辑用途的观测口（借槽判据要用）；
    // 其余 dbg_* 读数口已随调试接口一起删除。
    .dbg_wbusy    (fw_wr_busy)
);

//==============================================================================
//  【SD 卡 → SDRAM 纹理装载器】
//==============================================================================
//  上电后由 App/viewer_ctrl 用"album_src 从 FAT 目录取到的偏移"把 BMP 搬进
//  缩略图槽；搬完就空闲，运行时换图只要给它一个新的偏移即可
//  （写口由 sdr_wr_arb 仲裁，显示画面不会被打断）。
//==============================================================================
//  【SD 卡总线】控制器 + 扇区级仲裁（2026-10-27 step 3.1）
//==============================================================================
//  【为什么把 sd_card_top 从装载器里提出来】
//  以前它埋在 sd_bmp_loader 内部，SD 四个引脚也从装载器出 ⇒ 音频想读卡就必须
//  改装载器。现在它归顶层，两个客户挂在仲裁器上：
//      图像：sd_bmp_loader  （切图/上电装载，扇区连读）
//      音频：audio_wav_path （WAV 播放，按需预取）
//  仲裁粒度 = 一个扇区，因为 sd_card_top 的读请求本来就是"电平请求 → 512 字节
//  → 单拍 end"的语义，end 之后控制器回到 WAIT 重新锁地址，换人没有任何残留。
//==============================================================================
sd_card_top u_sd_card (
    .clk                    (sd_card_clk),      // 50MHz
    // ★ 2026-11-02 [SD-GATE]：这里绝不能用 rst_all（含 ~sd_ok，会自锁死）。
    //   用 rst_sd_host = rst_src + "卡丢失后自动重试"的复位脉冲，
    //   这样整机在复位时 SD 主机照样能重跑初始化，卡插回去就能自动恢复。
    .rst                    (rst_sd_host),
    .SD_nCS                 (sd_ncs),
    .SD_DCLK                (sd_dclk),
    .SD_MOSI                (sd_mosi),
    .SD_MISO                (sd_miso),
    .sd_init_done           (sd_init_done_w),
    .sd_sec_read            (sd_bus_read),
    .sd_sec_read_addr       (sd_bus_addr),
    .sd_sec_read_data       (sd_bus_data),
    .sd_sec_read_data_valid (sd_bus_dv),
    .sd_sec_read_end        (sd_bus_end),
    .sd_sec_write           (1'b0),             // 只读，禁写
    .sd_sec_write_addr      (32'd0),
    .sd_sec_write_data      (8'd0),
    .sd_sec_write_data_req  (),
    .sd_sec_write_end       ()
);

//==============================================================================
//  ★ 2026-10-31 P1：仲裁器换成 4 客户版 ★
//  FAT / TXT 两个口先打桩（req=0），等 P2 / P7 再接。
//  行为与旧版 2 客户完全一致：IMG 最高、AUD urgent 抢占、A_GAP 不变。
//==============================================================================
sd_bus_arbiter4 #(
    .TIMEOUT_CYC (1_000_000),
    .AUD_GAP_CYC (66_000),
    .FAT_QUOTA   (8),
    .ENA_TXT     (1)
) u_sd_arb (
    .I_clk          (sd_card_clk),
    .I_rst          (rst_all),
    .I_sd_init_done (sd_init_done_w),

    // ---- 客户 0：图像 ----
    .I_img_req      (ld_sd_req),
    .I_img_addr     (ld_sd_addr),
    .O_img_data     (ld_sd_data),
    .O_img_dv       (ld_sd_dv),
    .O_img_end      (ld_sd_end),

    // ---- 客户 1：音频 ----
    .I_aud_req      (au_sd_req),
    .I_aud_addr     (au_sd_addr),
    .I_aud_urgent   (au_sd_urgent),
    .O_aud_data     (au_sd_data),
    .O_aud_dv       (au_sd_dv),
    .O_aud_end      (au_sd_end),

    // ---- 客户 2：FAT 目录扫描 ----
    .I_fat_req      (fat_req_g),
    .I_fat_addr     (lst_sd_addr),
    .O_fat_data     (fat_sd_data),
    .O_fat_dv       (fat_sd_dv),
    .O_fat_end      (fat_sd_end),

    // ---- 客户 3：字幕 TXT 读取（★ 2026-11-06 接 sub_loader）----
    .I_txt_req      (txt_req_g),
    .I_txt_addr     (sub_sd_addr),
    .O_txt_data     (sub_sd_data),
    .O_txt_dv       (sub_sd_dv),
    .O_txt_end      (sub_sd_end),

    // ---- 到控制器 ----
    .O_sd_sec_read  (sd_bus_read),
    .O_sd_sec_addr  (sd_bus_addr),
    .I_sd_sec_data  (sd_bus_data),
    .I_sd_sec_dv    (sd_bus_dv),
    .I_sd_sec_end   (sd_bus_end),

    // ---- 调试 ----
    .O_timeout      (sd_arb_timeout),
    .O_owner        (sd_arb_owner)
);

//==============================================================================
//  ★★ 2026-11-01：文件页四宫格（FAT32 目录 → 4 个 SDRAM 槽）★★
//==============================================================================
//  【链路】
//      SD 卡 ──仲裁器客户2──► fat_dir_lister ──4 项{偏移,长度}──► quad_loader
//                  │                                              │
//                  └──客户0──► bmp_loader ◄──控制口在这里二选一─────┘
//                                   │
//                                   └──► SDRAM 槽 0..3
//
//  【装载器为什么能共用】bmp_loader 是"给偏移和目的基址，把 BMP 搬过去"，
//  它不关心调用者是相册还是文件页。归属由 quad_page_ctrl 用 quad_own 决定，
//  抢之前一定等它空闲（!ld_busy），所以不会打断相册的装载。
//
//  【名字读口】lister 的元数据口只有一套，用 lst_ld_slot 分时：
//      装载期间 → 给 quad_loader（它要按槽号取 offset/length）
//      其余时间 → 给 OSD（文件页要按格号取 名字/主名长度/扩展名长度）
//  两者在时间上互斥（装的时候还没到显示阶段），所以不需要双口 BRAM。
//==============================================================================
//==============================================================================
//  ★★ lister 时分复用 + 相册取偏移 ★★
//==============================================================================
//  【为什么锁存归属】两边的 start 都被 !lst_busy 挡住，理论上不会重叠；
//  但 done 是**共用的电平/脉冲**，若不锁存归属，"相册扫完"会被文件页当成
//  "我扫完了"，于是拿相册的列表去装载（名字全错、还可能读到 .wav）。
//  归属在 start 那一拍锁存，从此 done/cnt 只发给发起方。
//==============================================================================
//==============================================================================
//  ★★ 2026-11-02【音随图切】切图 → 找同名 WAV → 换曲 ★★
//==============================================================================
//  【触发点】`ld_req && ld_fmt` —— 这就是"高清装载发起"那一拍。
//    ld_req 在 viewer_ctrl 里是**单拍脉冲**（ldp 1→2，`ld_can_load` 那次），
//    ld_fmt 是本次装载的格式位，所以两者相与天然是一次上升沿，不需要边沿检测。
//    用"装载发起"而不是按键/切图那一刻，是因为换图时窗口偏移要等 album_src
//    重扫完成才是新值；高清装载被 `alb_ok`（bpend）挡在重扫之后 ⇒ 天然安全。
//    ★ 2026-11-07：同一次扫描顺带找同名 TXT（字幕），见下面 mm 块里的字幕部分。
//    副作用（已知并接受）：在相册菜单里按"上一张/下一张"时，音频要等退菜单
//    （高清装载发出）才切；主页按键切换与自动切换是即时的。
//
//  【目标名】`hd_rel_w` = viewer_ctrl 的 hd_img，与 ld_off 同一拍锁存
//    ⇒ 它就是"这次装的这张图"在相册窗口里的第几格。
//
//  【为什么不与相册/文件页抢】lister 只有一个 start/done。这里给 match
//    **最低优先级**：mux 里让 alb / 文件页先过；match 的 start 若被抢走，
//    `mm_win` 为 0 ⇒ 保留 mm_pend 下一轮再试，**绝不会让相册丢掉它的 start**
//    （那会让 album_src 的 tried=1 却永远等不到 done ⇒ 相册卡死）。
//
//  【时序套路】和 album_src 完全一样：看到 !busy 的**下一拍**出单拍 start
//    （lister 的 S_DONE 会在 start 拉低后一拍回 S_IDLE，正好对上）。
//==============================================================================
assign mm_win   = mm_go && !osd_file_page && !qp_lst_start && !alb_lst_start;
assign lst_start = osd_file_page ? qp_lst_start :
                   (alb_lst_start ? 1'b1 : mm_go);
assign lst_done_fp  = lst_done & (lst_own == OWN_FP);
assign lst_done_alb = lst_done & (lst_own == OWN_ALB);
// 命中：必须确认"这次扫描是我发的"，否则可能把相册扫描里碰巧的命中当自己的
wire mm_hit_raw  = lst_match_hit & (lst_own == OWN_MM);
wire lst_done_mm = lst_done     & (lst_own == OWN_MM);
// ★ 连击保护：扫描途中又来了新目标 ⇒ 这次飞着的扫描目标已过期，
//   它的命中必须丢弃（否则会按上一张图的曲子换曲），由 mm_pend 重扫最新目标。
assign mm_hit = mm_hit_raw & ~mm_stale;

//------------------------------------------------------------------------------
//  ★ 2026-11-02【信息条文件名】"关闭文件页 ⇒ 重扫" 的两个单拍
//------------------------------------------------------------------------------
//  ① 下降沿：请相册重扫一次（album_src 会在里面清 tried/O_ready ⇒ 装载挂住）。
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) osd_fp_d <= 1'b0;
    else         osd_fp_d <= osd_file_page;
end
assign nm_rescan = osd_fp_d & ~osd_file_page;

//  ② 相册扫描完成（= 放行那一拍）：让 OSD 把行 6 重写一遍。
//     ⚠ 必须用 lst_done_alb 而不是 O_ready 的上升沿：`O_ready` 要再晚几拍才置位
//     （A_COL 逐槽取完偏移之后），而名字在 lst_done 那一刻就已经是新的了 ——
//     早一拍重写，赶上 16.7ms 一次的布局节拍，肉眼看是"和放行同时更新"。
// ★ [NM-UPD]：相册重扫完成（OWN_ALB）**和** match 扫描完成（OWN_MM）都要重写一次文件名 ——
//   match 扫描期间名字那一行写的是空格（[NM-HOLD]），不补写就会一直空着。
// ★ [NM-HOLD]：match 扫描正在借名字读口（lister 的 mm_rd），这段时间 OSD 别读它。
//   ★ 2026-11-03：必须用 lister 的 O_nm_busy（= mm_rd），不能用 lst_busy。
//   O_busy 在 S_SCAN 结束那拍就落，而 mm_rd 是打了一拍的，还会多借一拍；
//   这一拍空档里 OSD 读到的是比较器地址，首字符会被写成空格。
assign nm_upd  = lst_done & ((lst_own == OWN_ALB) | (lst_own == OWN_MM));
assign nm_hold = lst_nm_busy;

// ★ 2026-11-02【不打断正在播的那首】：命中的 WAV 就是当前这首、而且还在播
//   ⇒ 跳过换曲（否则来回切图会把同一首 BGM 反复从头播）。
//   `!trk_done` 这一项不能省：曲已经播完（loop=0）时同一首要能重放，
//   否则「音乐结束→自动换图→换新曲」的闭环会断（见 trk_done 的注释）。
wire mm_same = (lst_match_off == trk_sec_r) && ~trk_done;

always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all)                       lst_own <= OWN_ALB;
    else if (lst_start)                lst_own <= osd_file_page ? OWN_FP :
                                                 (alb_lst_start ? OWN_ALB : OWN_MM);
end

// ★ 2026-11-07 字幕：TXT 与当前字幕是同一份（或两边都没有）⇒ 不重装（同 mm_same 的思路）
wire txt_same = (txt_found_r == sub_has) && (!txt_found_r || (txt_lba_r == sub_sec));

always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all) begin
        mm_pend     <= 1'b0;
        mm_go       <= 1'b0;
        mm_rel      <= 2'd0;
        mm_live     <= 1'b0;
        mm_stale    <= 1'b0;
        mm_seen     <= 1'b0;
        mm_sub_ok   <= 1'b0;
        txt_found_r <= 1'b0;
        txt_lba_r   <= 23'd0;
        sub_start   <= 1'b0;
        sub_has     <= 1'b0;
        sub_sec     <= 23'd0;
    end else begin
        mm_go     <= 1'b0;
        sub_start <= 1'b0;
        //==========================================================================
        //  ★ [MM-EARLY] 换图后一拍就武装这次 match（音乐/字幕定位）
        //==========================================================================
        //  原来只在"高清装载发起"时武装 ⇒ 扫描只能排在烘焙后面，音乐/字幕总比
        //  图片晚到。现在换图当拍（动画开始前）就武装：
        //    · 扫描落在 S_ARM/S_TRANS 那段 1~3 s 的动画里跑完
        //      ⇒ 图片出现时 `sub_start` 早就发过、`trk_set_r` 早就切过曲
        //    · **不延后图片**：动画本来就要等，扫描是搭它的车，不是插在它前面
        //  目标槽号用 `O_mm_rel`（换图后一拍的 cur_rel，已经是新图的位置）。
        //  `mm_arm_ok` 是安全门：滑窗换图（base 变、相册要重扫）时它不成立，
        //  这一次就不提前，交给下面的 ld_req 兜底 —— 宁可晚一次，不能配错一张。
        if (mm_arm_w && mm_arm_ok) begin
            mm_pend <= 1'b1;
            mm_rel  <= mm_arm_rel;
            mm_seen <= 1'b0;
            // 只有当"在飞的那次扫的是别的图"才作废旧目标；
            //   同一张图的重复武装（见下面 ld_req 兜底）不能把结果作废。
            if (mm_live && (mm_arm_rel != mm_rel)) begin
                mm_stale  <= 1'b1;
                mm_sub_ok <= 1'b0;
            end
        end
        // 高清装载发起时再兜底一次：开机（没有 sw_go）和"提前那次因为相册没 ready
        // 而没跑成"都靠它。当前图已经扫过（mm_seen）就不重复扫了。
        // ★ 2026-11-07：字幕也靠这次扫描 ⇒ 扫描不再看 osd_asy；
        //   "音随图切"开关只门掉换曲动作（见 trk_set_r 那处）。
        if (ld_req && ld_fmt && !mm_seen) begin
            mm_pend <= 1'b1;
            mm_rel  <= hd_rel_w;
            // 已有一次 match 扫描在飞 ⇒ 它的目标作废（命中要丢弃，见 mm_hit）
            if (mm_live && (hd_rel_w != mm_rel)) begin
                mm_stale  <= 1'b1;
                mm_sub_ok <= 1'b0;
            end
        end
        // 发起：空闲的下一拍出单拍 start
        // ★ `alb_ready` / `!alb_lst_start` 不是多余的：
        //   ① album_src 的 O_lst_start 是**寄存**输出——它在 T 拍看到 !busy 就
        //      在 T+1 拍拉高。若 match 也在 T 拍发起，T+1 拍 lister 已忙，
        //      而 album 的 start 这时才到 ⇒ lst_own 被改写成 ALB，mm 的扫描
        //      被当成相册的 done（相册拿到空槽 ⇒ 画面空白、还得等下次滑窗才自愈）。
        //   ② album 只在 `!O_ready` 时才发起扫描 ⇒ 要求 alb_ready 就等于
        //      "相册这拍不会启动"，两条一起把错位彻底封死。
        // ★ [MM-EARLY]：让路条件从"装载器忙"收窄成"**高清烘焙**在飞"。
        //   提前扫描正好撞上动画里的缩略图预取，若还按 `!ld_busy` 让路，
        //   这次扫描又会被推回烘焙之后 —— 那这次改动就白做了。
        //   高清烘焙仍然享受百分百的优先（它决定画面多久出来）；
        //   预取只是"下一张的缩略图"，晚几十毫秒无感。
        if (mm_pend && !lst_busy && !osd_file_page && !alb_lst_start && alb_ready &&
            !(ld_busy && ld_fmt)) mm_go <= 1'b1;
        // 这一拍真的被收下才清 pend；被 alb/qp 抢走就留着下轮再试
        if (mm_win) begin
            mm_pend     <= 1'b0;
            mm_live     <= 1'b1;
            mm_sub_ok   <= 1'b1;
            txt_found_r <= 1'b0;
        end
        // 扫描收尾（命中或扫完）⇒ 复位在飞/过期标志
        if (mm_hit_raw | lst_done_mm) begin
            mm_live  <= 1'b0;
            mm_stale <= 1'b0;
            // 只有"没被作废"的那次扫完才算"这张图扫过了"
            if (!mm_stale) mm_seen <= 1'b1;
        end

        // ---- 字幕：同一次扫描里的同名 TXT ----
        //   mm_stale 在第一个命中（常常是 WAV）就清了，挡不住后到的 TXT ⇒
        //   字幕用自己的 mm_sub_ok：目标被换掉后直到下一次 mm_win 都不认。
        if (lst_match_txt_hit && (lst_own == OWN_MM) && mm_sub_ok) begin
            txt_found_r <= 1'b1;
            txt_lba_r   <= lst_match_off;
        end
        // 扫完：TXT 变了才装（同一张图的高清重装 ⇒ 同一份 TXT ⇒ 字幕继续滚）
        if (lst_done_mm && mm_sub_ok && !txt_same) begin
            sub_start <= 1'b1;
            sub_has   <= txt_found_r;
            sub_sec   <= txt_lba_r;
        end
    end
end

// 旧的两客户仲裁（保留注释，便于对照）
// assign lst_start    = osd_file_page ? qp_lst_start : alb_lst_start;
// assign lst_done_fp  = lst_done &  lst_own_fp;
// assign lst_done_alb = lst_done & ~lst_own_fp;

album_src u_album (
    .clk          (sd_card_clk),
    .rst          (rst_card),           // ★ [SD-GATE] 同 rst_all（含"没卡"门控）
    .I_base       (alb_base),           // 滑动窗口起点（PLAN §7B.3）
    .I_need       (1'b1),               // ★ [SD-GATE-OFF] 恒 1：让相册**无论如何都扫一次**。
                                        //   理由：如果卡初始化真的没完成（这次实验的结论），
                                        //   把扫描也挂到 sd_init_done 上会让状态机停在 S_BOOT_T、
                                        //   连 OSD 都进不去 ⇒ 又一次"全黑且不可观察"。
                                        //   让它扫：扫到 0 张会走 S_STUCK（那里 compose_en=1、菜单可用），
                                        //   扫到图就继续装载 —— 两条路都是可观察的。
    .I_block      (osd_file_page),
    // ★ 2026-11-02【信息条文件名】：文件页退出 ⇒ 重扫本窗口，把 name_mem 抢回来
    .I_rescan     (nm_rescan),
    .O_lst_start  (alb_lst_start),
    .O_lst_offset (),
    .O_rd_slot    (alb_rd_slot),
    .I_lst_busy   (lst_busy),
    .I_lst_done   (lst_done_alb),
    .I_ld_type    (lst_type),
    .I_slot_valid (lst_valid),
    .I_file_off   (lst_off),
    .O_off0       (alb_off0),
    .O_off1       (alb_off1),
    .O_off2       (alb_off2),
    .O_off3       (alb_off3),
    .O_any        (alb_any),
    .O_nimg       (alb_nimg),
    .O_win4       (alb_win4),
    .O_ready      (alb_ready),
    .O_sel        (alb_sel)
);

fat_dir_lister #(
    .N_ITEMS     (4),
    .MAX_SECS    (FAT_MAX_SECS),
    .INCLUDE_DIR (0)
) u_fat (
    .clk(sd_card_clk), .rst(rst_card),        // ★ [SD-GATE] 同 rst_all
    .I_start        (lst_start),
    // ★ 2026-11-01 D：相册也要能指定窗口起点（原来恒 0 ⇒ 窗口永远停在头 4 项，
    //   第 4 张以后根本扫不到）。album_src 的 O_lst_offset 就是 alb_base。
    // ★ 2026-11-02：match 扫描不跳项（offset=0），要扫整张目录
    .I_offset       (mm_go ? 16'd0 : (osd_file_page ? quad_off : alb_base)),
    // ★★ 2026-11-01 夜：offset 语义随消费者切换 ★★
    //   相册（!osd_file_page）：只列/只数 BMP ⇒ alb_base 真的是"第几张图"，
    //     窗口 4 格全是图 ⇒ album_src 的 win4 不会被混在中间的 wav/txt 打断。
    //     原来固定 IMAGES_ONLY=0，卡上只要有 music.wav，相册滑到
    //     [图2,图3,图4,WAV] 就 win4=0 卡死，第 5 张图永远翻不到（上板症状）。
    //   文件页（osd_file_page）：保留 wav/txt ⇒ quad_off 仍按"目录条目"翻页。
    // ★ 2026-11-02：match 扫描必须列全部类型（否则 WAV 不算有效项、扫不到）
    .I_list_all_types(osd_file_page | mm_go),
    // ★ 2026-11-02【音随图切】match 口：只在 mm_go 那一拍意义有效
    .I_match_en     (mm_go),
    .I_match_rel    (mm_rel),
    .O_match_hit    (lst_match_hit),
    .O_match_off    (lst_match_off),
    // ★ 2026-11-06 字幕：同一次扫描顺带找同名 TXT（与 WAV 各自独立）
    .O_match_txt_hit(lst_match_txt_hit),
    .O_nm_busy      (lst_nm_busy),
    .O_busy         (lst_busy),
    .O_done         (lst_done),
    .O_valid_cnt    (lst_cnt),
    .O_has_more     (lst_has_more),
    .I_rd_slot      (lst_rd_slot),
    .I_rd_idx       (lst_rd_idx),
    .O_rd_char      (lst_rd_char),
    .O_rd_blen      (lst_rd_blen),
    .O_rd_elen      (lst_rd_elen),
    .O_rd_vld       (lst_rd_vld),
    .I_ld_slot      (lst_ld_slot),
    .O_ld_file_off  (lst_off),
    .O_ld_size      (lst_size),
    .O_ld_namelen   (),
    .O_ld_blen      (lst_blen),
    .O_ld_elen      (lst_elen),
    .O_ld_len4      (lst_len4),
    .O_ld_vld       (lst_vld),
    .O_ld_type      (lst_type),
    .O_slot_valid   (lst_valid),
    .O_sd_req       (lst_sd_req),
    .O_sd_addr      (lst_sd_addr),
    .I_sd_data      (fat_sd_data),
    .I_sd_dv        (fat_sd_dv),
    .I_sd_end       (fat_sd_end)
);

//==============================================================================
//  ★ 2026-11-06【动态字幕装载器】TXT 首扇区 → 字符码 → Flash 字模 → 字形 RAM
//==============================================================================
//  【启动时机】match 扫描结束、且 TXT 文件与当前字幕不同（mm 块里的 sub_start）。
//  【双缓冲】新字幕装进 back bank，首批装完自己提交（O_commit_tgl），旧字幕滚到那一刻。
//  【端口分工】SD 客户 3 读 TXT；Flash 客户 3 读字库（经 App → osd_top → flash_arb）；
//    `I_vs_tgl` 是 dyn_subtitle 每帧翻转的脉冲（跨回 sd 域数帧，调度懒加载）。
//  【复位】rst_all（与 lister / 音频同域同复位）。
sub_loader u_sub_loader (
    .clk         (sd_card_clk),
    .rst         (rst_all),

    // ---- 启动 ----
    .I_start     (sub_start),
    .I_txt_found (sub_has),
    .I_txt_sec   (sub_sec),

    // ---- SD 客户 3（扇区号）----
    .O_sd_req    (sub_sd_req),
    .O_sd_addr   (sub_sd_addr),
    .I_sd_data   (sub_sd_data),
    .I_sd_dv     (sub_sd_dv),
    .I_sd_end    (sub_sd_end),

    // ---- Flash 客户 3（字库）----
    .O_fl_req    (sub_fl_req),
    .O_fl_addr   (sub_fl_addr),
    .O_fl_len    (sub_fl_len),
    .I_fl_rdy    (sub_fl_rdy),
    .I_fl_data   (sub_fl_data),
    .I_fl_valid  (sub_fl_valid),
    .I_fl_done   (sub_fl_done),

    // ---- 帧脉冲 ----
    .I_vs_tgl    (sub_vs_tgl),

    // ---- 字模写口 → dyn_subtitle 的 BRAM 端口 A ----
    .O_wr_addr   (sub_wr_addr),
    .O_wr_data   (sub_wr_data),
    .O_wr_we     (sub_wr_we),

    // ---- 提交 → dyn_subtitle ----
    .O_commit_tgl(sub_commit_tgl),
    .O_bank      (sub_bank),
    .O_text_len  (sub_len)
);

quad_page_ctrl #(.N_ITEMS(4)) u_qpage (
    .clk            (sd_card_clk), .rst(rst_app),   // ★ [SD-GATE] 同 rst_all
    .I_file_page    (osd_file_page),
    .I_flip_next    (quad_flip_nx),
    .I_flip_prev    (quad_flip_pv),
    .O_lst_start    (qp_lst_start),
    .O_lst_offset   (),
    .I_lst_busy     (lst_busy),
    .I_lst_done     (lst_done_fp),
    .I_lst_cnt      (lst_cnt),
    .I_lst_has_more (lst_has_more),
    .O_ld_start     (ql_start),
    // ★★ 2026-10-31 [Q4 根因]：这两根必须是 quad_loader 的**整页** busy/done ★★
    //  原来接的是 bmp_loader 的 ld_busy / ld_done（**每张**一次）。第 1 张装完那拍
    //  u_qpage 就进 S_SHOW 并采 `O_ready <= (I_ld_loaded != 0)`，而 ql_loaded
    //  恰好在同一个沿才 +1（寄存器）⇒ 采到 0 ⇒ quad_ready 永远 0 ⇒ quad_mode_r
    //  永远 0 ⇒ A 层一直读相册那块 ⇒ 文件页四格全透明（上板现象）。
    //  ld_busy 在两张之间也会掉 0，S_SHOW 还会借这个空档提前交还归属。
    //  回归：APP/tb_quad_ready.v（-DOLD_WIRING 复现旧接法：ready 起不来）。
    .I_ld_busy      (ql_busy),
    .I_ld_done      (ql_done),
    .I_ld_loaded    (ql_loaded),
    .I_ld_filled    (ql_filled),
    .I_ld_bmp       (ql_bmp),
    .O_ld_abort     (qp_ld_abort),
    .I_cur          (fp_cur_w),
    .O_has_prev     (quad_has_prev),
    .O_rank         (quad_rank),
    .O_own          (quad_own),
    .O_ready        (quad_ready),
    .O_has_more     (quad_has_more),
    .O_offset       (quad_off),
    .O_gen          (fp_gen_w),
    .O_valid_cnt    (quad_vcnt)
);

quad_loader #(
    .SLOT_BASE  (QUAD_BASE),
    .SLOT_STEP  (40960),        // = 160 字/行 × 256 行（见 ps_gen_image 的 quad_map）
    .N_ITEMS    (4)
) u_qload (
    .clk        (sd_card_clk), .rst(rst_app),    // ★ [SD-GATE] 同 rst_all
    .I_start    (ql_start),
    .I_abort    (qp_ld_abort),
    .I_hold     (op_ph),
    .O_bmp      (ql_bmp),
    .O_filled   (ql_filled),
    .I_valid_cnt(lst_cnt),
    .I_slot_valid(lst_valid),
    .O_busy     (ql_busy),
    .O_done     (ql_done),
    .O_loaded   (ql_loaded),
    .O_rd_slot  (ql_rd_slot),
    .I_file_off (lst_off),
    .I_file_len (lst_size),
    .I_file_type(lst_type),
    .O_ph_start (ph_start),
    .O_ph_kind  (ph_kind),
    .O_ph_base  (ph_base),
    .I_ph_busy  (ph_busy),
    .I_ph_done  (ph_done),
    .O_ld_start (bmp_start_q),
    .O_ld_off   (ql_off),
    .O_ld_len   (ql_len),
    .O_ld_base  (ql_base),
    .I_ld_ready (ld_ready),
    .I_ld_busy  (ld_busy),
    .I_ld_done  (ld_done),
    .I_ld_err   (ld_err)
);

// 元数据口分时 + 装载器控制口二选一
// 元数据口三方分时：相册取偏移（只在开机一次）> 四宫格装载 > OSD 名字
assign lst_ld_slot = alb_sel ? alb_rd_slot :
                     op_ph   ? fp_cur_w    :          // ★ 2026-11-04：文件页"打开"借一拍
                     (ql_busy ? ql_rd_slot : lst_rd_slot);
assign bmp_start   = quad_own ? bmp_start_q : (ld_req & ~quad_own);
assign bmp_off     = quad_own ? ql_off   : ld_off;
// ★ 长度不做二选一：`bmp_loader` 的 `file_length` 是**死输入**（内部 flen 只写不读），
//   为它保留一个 32 位 mux 是纯浪费（32 LUT）。这里直接把相册的长度透过去，
//   两种模式都不影响行为；`ql_len` 现在就没人用了。
assign bmp_len     = ld_len;
assign bmp_base    = quad_own ? ql_base  : ld_base;
// ★ 2026-11-04：四宫格直接装 160×120（行距 160 字，见 ps_gen_image 的 quad_map）
//   ——只读源图 1/4 的行，比原来的 320×240 快约一倍
assign bmp_mode    = quad_own ? 2'd2 : (ld_fmt ? 2'd0 : 2'd1);
assign bmp_fmt     = quad_own ? 1'b0 : ld_fmt;      // 四宫格恒 565

//==============================================================================
//  ★★ 2026-10-22 build N：降采样到 320×240（1/2 抽样）★★
//  【为什么最终选 320×240 而不是 240×180】
//    · 320→640 是**精确 2 倍**放大：每个源像素固定覆盖 2 个屏幕像素，
//      宽度完全均匀 ⇒ 不存在"2/3 交替"造成的网格线粗细不匀（那种不均匀
//      在最近邻放大下是数学必然，看起来就像脏像素）。
//    · 240→640 是 2.67 倍，宽度在 2 和 3 之间循环 ⇒ 网格必然歪扭。
//  【loader 已支持 1/2】sample_c/sample_r 增加了 IS_HALF 分支，
//    用 `OW*2 == IMG_W` 判断（不能用 IMG_W/OW，整数除法会把 640/240 也算成 2）。
//  【注意】降采样发生在**装载时**，改了尺寸必须重新上电装载才生效。
//  【回退】OUT_W/OUT_H 改回 0（不下采样，源 640×480），并把 app_scene_cfg 的
//          SRC_W/img0_k/img1_* 一起改回。
//==============================================================================
//------------------------------------------------------------------
//  ★ 2026-10-29：装载链换成 bmp_loader（读卡+头解析+内部自动裁剪+缩放+写口）
//  默认参数与旧行为等价：mode/fmt 由 ld_fmt 决定、zoom=1.0(fit)、
//  视口中心自动取源图中心 ⇒ 几何与旧版 contain 完全一致。
//------------------------------------------------------------------
// ★ 2026-10-30：几何输入改成「锚点」，由 App 里的 zoom_osd 实时给
//   （声明在 App 例化之前，见上）。默认 zoom=1.0、锚点=屏幕中心，
//   bmp_view 内部算出的 cam 正好是源图中心 —— 与原来的 cam_auto=1 逐像素等价。

bmp_loader #(.AFIFO_AW(9)) u_loader (
    .clk          (sd_card_clk),
    .rst          (rst_app),            // ★ [SD-GATE] 同 rst_all
    .mclk         (ext_mem_clk),

    // ★ 控制口二选一（相册 / 文件页四宫格），见上面的 u_qload 段
    .start        (bmp_start),
    // ★ [ABORT-OR]：abort 不能按归属二选一 —— 归属只是"谁发单"的问题，
    //   而"能不能放弃"是安全属性。原写法在 quad_own 上升/下降的那几拍会把 App 的
    //   abort 断线（quad_own 只决定 start/addr/base 的归属）。这里改成或。
    .abort        (app_ld_abort | qp_ld_abort),
    .file_offset  (bmp_off),
    .file_length  (bmp_len),
    .dst_base     (bmp_base),

    .mode         (bmp_mode),
    .fmt          (bmp_fmt),            // 0 = 565 缩略图；1 = 888 高清
    .dst_stride   (quad_own ? 11'd160 : 11'd0),   // 四宫格 80 字/行放在 160 字行距里；其余自动
    // ★ 2026-11-05 [LD-SNAP]：相册装载用"发单快照"，四宫格沿用原来的实时值
    //   （四宫格的 mode=2 不走缩放烘焙，保持原行为不动）
    .zoom         (quad_own ? ld_zoom_w : ld_snap_zoom),
    .ax           (quad_own ? ld_ax_w   : ld_snap_ax),
    .ay           (quad_own ? ld_ay_w   : ld_snap_ay),

    .ready        (ld_ready),
    .busy         (ld_busy),
    .done         (ld_done),
    .err          (ld_err),
    .err_abort    (ld_err_abort),        // ★ 纯诊断：只用于屏蔽"被放弃"那次的错误音
    .src_w_o      (ld_src_w),
    .src_h_o      (ld_src_h),

    .wr_vld       (ld_wr_vld),
    .wr_used      (ld_wr_used_raw),     // 10 位原值（AFIFO_AW=9）
    .wr_addr      (ld_wr_addr),
    .wr_data      (ld_wr_data),
    .wr_ack       (ld_wr_ack),
    .wr_settled   (ld_wr_settled),

    // ---- SD 扇区读客户口 ----
    .sd_sec_read            (ld_sd_req),
    .sd_sec_read_addr       (ld_sd_addr),
    .sd_sec_read_data       (ld_sd_data),
    .sd_sec_read_data_valid (ld_sd_dv),
    .sd_sec_read_end        (ld_sd_end),
    .sd_init_done           (sd_init_done_w)
);

// 旧调试口已下线：只保留"错误位"，其余接 0（顶层已无消费者）
assign ld_sd_ready   = sd_init_done_w;
// 四宫格占用装载器时不放行相册的装载请求（否则两边会抢同一个 start）
assign ld_can_load   = ld_ready & ~quad_own;
assign ld_prog       = 4'd0;
assign ld_state_code = 4'd0;
assign ld_bmp_state  = 4'd0;
assign ld_err_code   = {3'd0, ld_err};

//==============================================================================
//  【上电装载】已由 App 内部的 viewer_ctrl 驱动（ld_req / ld_fmt / ld_off / ld_len / ld_base）
//==============================================================================
//  顺序（2026-11-04）：当前图缩略图 → 当前图高清 → 下一张缩略图；之后每次切图
//    补装 1 张缩略图（与过渡并行）+ 目标图高清（过渡后，可被下一次切图打断）。
//  装载失败不会锁死：viewer_ctrl 进 STUCK，保持低清并恢复按键。
//==============================================================================

// ---- SDRAM 控制器 ----
// 【为什么要两个时钟】sdr_as_ram 内部用 Clk 产生命令、用 Clk_sft（180°相移）
// 去采样 SDRAM 返回的数据。SDRAM 的数据输出有 tAC 延迟，用相移时钟采样
// 才能正好落在数据稳定窗口的中间（这是 SDRAM 控制器的经典做法）。
sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_ref_req       (App_ref_req),           // ★ 2026-10-24：即将发 REF（给调度器让组首避开）
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);



//==============================================================================
//  【播放开关】SW2（板上拨到 ON = 0，所以取反后才是"播放"）+ OSD 静音
//==============================================================================
//  【为什么取反】板卡丝印的 ON 位置对应低电平，这是硬件决定的；
//  为了让"拨上去 = 播放"符合直觉，这里统一取反一次。
//  【为什么要同步】sw_level 在 clk（50MHz 晶振）域，video_clk 是 PLL 出来的
//  另一个 50MHz，两者相位无关 ⇒ 必须打两拍再用。
//  audio_wav_path 内部对 sd_card_clk 侧也有自己的两拍同步。
//
//  ★ 2026-10-29：OSD 系统页的"静音"与 SW2 **串联**（用户要求 sw2 保留）：
//        play = SW2 ∧ ¬静音
//    静音位在 sd_card_clk 域（OSD 时钟），先打两拍进 clk 域再与 sw2 相与。
//    这样两个开关任何一个"关"都能停播，语义与改造前 SW2 单独工作时完全一致
//    （wav_player 在 !I_play 时仍按 48kHz 送 0 样本，ACR 不会断）。
//==============================================================================
assign sw2_play = ~sw_level[1];

reg [1:0] mute_c_sync;                  // 静音：sd_card_clk → clk
always @(posedge clk or posedge rst_all) begin
    if (rst_all) mute_c_sync <= 2'd0;
    else         mute_c_sync <= {mute_c_sync[0], osd_mute};
end

// 真正送进音频链路的播放请求（两个域都用这一个，保证语义一致）
wire play_async = sw2_play & ~mute_c_sync[1];

reg [1:0] play_v_sync;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) play_v_sync <= 2'd0;
    else         play_v_sync <= {play_v_sync[0], play_async};
end
assign audio_play_v = play_v_sync[1];

//==============================================================================
//  【音乐播完】→ 翻转电平（给 OSD 的"音乐结束后切换"用）
//==============================================================================
//  ★ 2026-11-08：改用 wav_pass_tick（每遍结束一拍），而不是 wav_pass_end。
//    · 循环播放（出厂默认）时 reader 在数据尾立刻回卷，FIFO 永远不会排空
//      ⇒ wav_pass_end 恒为 0 ⇒ 旧实现下「自动切换=音乐结束」永远不触发。
//    · wav_pass_tick 与 I_loop 无关：loop=1 每回卷一次、loop=0 排空时各一拍。
//    · trk_done 仍用 wav_pass_end（“真的停了”），语义不变，见上面的 trk_done 块。
always @(posedge sd_card_clk or posedge rst_all) begin
    if (rst_all)            music_tgl <= 1'b0;
    else if (wav_pass_tick) music_tgl <= ~music_tgl;
end

//==============================================================================
//  【SD 卡 WAV 通道】只有 AUDIO_SRC=3 时才进综合
//==============================================================================
generate
if (AUDIO_SRC == 3) begin : g_wav_sd

    audio_wav_path #(
        // ★ 2026-11-02：不再有"待机默认曲"。三个都填 0 的语义是：
        //   TOTAL_WORDS=0 ⇒ reader 复位后 left=0 ⇒ S_IDLE 不发请求 ⇒ 静音；
        //   一旦 trk_set（音随图切命中 / 文件页打开 WAV）到来，user=1、
        //   left=1、hdr=1 ⇒ 从新起点现解析 WAV 头的 data 块拿真实长度。
        //   BASE_SECTOR / SKIP_BYTES 只在 user=0 时才会被用到，现在走不到。
        .BASE_SECTOR  (0),
        .SKIP_BYTES   (0),
        .TOTAL_WORDS  (0),
        .FIFO_AW      (10),                 // 1024 字 = 4KB ≈ 21.3ms
        // 没有"编译期已知总长"，一律按常规 512 字预灌
        //   （原来这里是 (TOTAL_WORDS==0)?1:... ，那个 1 是给"默认曲"用的，
        //     现在默认曲没了，1 字预灌会在换曲刚起头就欠载）
        .PREFILL      (512),
        .REQ_LEVEL    (768),
        .URGENT_LEVEL (256)
    ) u_wav_path (
        .I_clk_sd      (sd_card_clk),       // 50MHz：reader + FIFO 写侧
        .I_rst_sd      (rst_all),
        .I_clk_v       (video_clk),         // 25MHz：FIFO 读侧 + wav_player
        .I_rst_v       (rst_all),
        .I_play_async  (play_async),        // 内部各自打两拍（已含静音门控）
        .I_loop        (osd_loop),          // ★ 与 reader 同在 sd_card_clk 域，直接接
        .I_trk_set     (trk_set_r),         // ★ 2026-11-04：文件页打开 WAV ⇒ 换曲
        .I_trk_sec     ({9'd0, trk_sec_r}),

        .O_sd_req      (au_sd_req),
        .O_sd_addr     (au_sd_addr),
        .O_sd_urgent   (au_sd_urgent),
        .I_sd_data     (au_sd_data),
        .I_sd_dv       (au_sd_dv),
        .I_sd_end      (au_sd_end),

        .O_audio_valid (wav_valid),
        .O_audio_left  (wav_left),
        .O_audio_right (wav_right),
        .O_underrun    (wav_underrun),
        .O_drop        (wav_drop),
        .O_playing     (wav_playing),
        .O_fifo_level  (wav_level),
        // ★ 2026-10-29：这根线**原来一直是悬空的** —— wav_pass_end 声明了、
        //   music_tgl 也在用（第 1935 行），但实例上根本没接 ⇒ "音乐结束后切换"
        //   从来不会触发。顺手接上（同时 audio_wav_path 把 O_pass_end 改成了
        //   单拍脉冲，否则 I_loop=0 时它会常高，music_tgl 会每个时钟翻转）。
        .O_pass_end    (wav_pass_end),
        // ★ 2026-11-08：每遍播完一拍（loop=1 也发）→ music_tgl → 自动切换
        .O_pass_tick   (wav_pass_tick)
    );

end else begin : g_no_wav_sd

    // 不选 3 时把音频客户口和它的输出全部摁住 —— 仲裁器不会给音频授权，
    // 图像装载器的时序与改造前完全一致。
    assign au_sd_req    = 1'b0;
    assign au_sd_addr   = 32'd0;
    assign au_sd_urgent = 1'b0;
    assign wav_valid    = 1'b0;
    assign wav_left     = 24'sd0;
    assign wav_right    = 24'sd0;
    assign wav_underrun = 1'b0;
    assign wav_drop     = 1'b0;
    assign wav_playing  = 1'b0;
    assign wav_level    = 11'd0;
    assign wav_pass_end = 1'b0;         // ★ 不选 SD WAV 时也要有驱动（它是内部 wire）
    assign wav_pass_tick = 1'b0;        // ★ 同上

end
endgenerate

//==============================================================================
//  【音频链路】音源 → PCM → ACR → HDMI
//==============================================================================
//  ★ 2026-10-26 step 0/1：音源全部收敛到音频应用层 audio/ ★
//
//  所有音源的输出接口完全一致（audio_valid + 24bit 左/右），下游 ACR 与 HDMI
//  组包一行都不用改：
//    AUDIO_SRC=0（默认）→ hdmi_audio_tone_pcm_scale
//        在 video_clk 域用分数分频直接产 48kHz/24bit PCM；
//        省掉 I2S 发送+接收一对模块，也不再受 audio_mclk 频率精度影响
//        （旧 PLL 实际给 12.295MHz，音高偏 +0.06%）。
//    AUDIO_SRC=1（旧方案）→ hdmi_audio_tone_i2s_64fs + I2S_receiver
//        I2S 自环，用于继续单独验证 I2S 时序（引脚 J13/F13/L14 约束可随时放开）。
//    AUDIO_SRC=2（step 1）→ wav_player
//        播放内核 + 48kHz 节拍 + 16bit→24bit + 欠载保持；
//        当前 TEST_SRC=1，输出内部 1kHz 测试音，用来验证"样本接口"这条新路。
//    AUDIO_SRC=3（step 3）→ audio_wav_path（SD 卡 WAV）
//        SD → 仲裁器 → 4KB FIFO → wav_player；SW2 控制播放/停止。
//
//  选源逻辑放在 audio/audio_src_sel.v（音频应用层入口）；
//  存储访问（SD 读口 + 仲裁）在顶层，与图像装载器分时复用同一条 SPI 总线。
//==============================================================================
audio_src_sel #(
    .SRC_SEL           (AUDIO_SRC),
    .AMP               (24'sd2000000),  // 幅度：约满量程 24%（≈ -12.5 dBFS）
    .NOTE_HOLD_SAMPLES (24_000)         // 每个音 0.5s @48kHz
) u_audio_src_sel (
    .I_clk              (video_clk),        // 输出与解串都在像素时钟域
    .I_rst              (rst_all),
    .I_mclk             (audio_mclk),       // 仅 AUDIO_SRC=1 的 I2S 源使用
    .I_play             (audio_play_v),     // 仅 AUDIO_SRC=2 用（3 由 wav_path 内部管）
    .I_ext_valid        (wav_valid),        // 仅 AUDIO_SRC=3 用
    .I_ext_left         (wav_left),
    .I_ext_right        (wav_right),
    .O_audio_valid      (src_valid_w),      // 每产出一个采样点拉高一次
    .O_audio_left_data  (src_l_w),
    .O_audio_right_data (src_r_w)
);

//------------------------------------------------------------------------------
//  ★ 2026-10-27：音乐 + 系统提示音（24bit 饱和叠加）
//    提示音按同一个 48kHz 节拍（src_valid_w）取样，天然与音乐样本对齐。
//------------------------------------------------------------------------------
wire [23:0] mix_l, mix_r;
wire        mix_valid;

audio_mix u_audio_mix (
    .clk      (video_clk),
    .rst      (rst_all),
    .valid    (src_valid_w),
    .mus_l    (src_l_w),
    .mus_r    (src_r_w),
    .bep      (beep_smp),
    .bep_act  (beep_act),
    .out_l    (mix_l),
    .out_r    (mix_r),
    .o_valid  (mix_valid)
);

//------------------------------------------------------------------------------
//  ★ 2026-10-29：全局数字音量（OSD 系统页"音量"滑条）
//------------------------------------------------------------------------------
//  【为什么放在 mix 之后】"全局音量"= 音乐和提示音一起变小。
//  【为什么不会伤音质】24bit × 9bit 全精度、四舍五入、增益 ≤1 不溢出；
//    100% 时 g9=256 ⇒ 输出与输入**逐位相同**。详见 audio/audio_vol.v 头注释。
//  【时钟域】vol 来自 sd_card_clk（OSD），慢变电平 ⇒ 打两拍进 video_clk。
//    ★ 7 位一起变，理论上可能采到"半新半旧"的中间值；但音量只在按键时变，
//      且中间值最多差 1 个 LSB 增益、只作用一个样本（20µs）—— 听不出来，
//      不值得为此加握手。
//  【延迟】6 拍（240ns）；ACR 只数 valid 个数，不受影响。
//------------------------------------------------------------------------------
reg [6:0] vol_v_sync1, vol_v_sync2;
always @(posedge video_clk or posedge rst_all) begin
    if (rst_all) begin
        vol_v_sync1 <= 7'd100;          // 上电 = 单位增益（与 OSD 默认值一致）
        vol_v_sync2 <= 7'd100;
    end else begin
        vol_v_sync1 <= osd_vol;
        vol_v_sync2 <= vol_v_sync1;
    end
end

audio_vol u_audio_vol (
    .clk     (video_clk),
    .rst     (rst_all),
    .valid   (mix_valid),
    .vol     (vol_v_sync2),
    .l_in    (mix_l),
    .r_in    (mix_r),
    .l_out   (audio_left_data),
    .r_out   (audio_right_data),
    .o_valid (audio_valid)
);

//------------------------------------------------------------------------------
//  ★ 2026-10-27：系统提示音（Flash 0x040000 起，裸 16bit 单声道 48kHz）
//    KEY1 开菜单 → ok；其它按键 → click；装载失败 → err。
//    优先级靠"busy 期间忽略"实现（不打岔，最长 400ms）。
//  ★ 2026-11-02【BEEP-ABORT】：**被翻页/连切放弃的那次装载不算失败**。
//    bmp_loader 对 abort 的处理是"黑像素快进 + 正常 done + 同拍 err"（见其端口注释），
//    于是每翻一页（打断在飞的缩略图/高清预装）都会冒一个 err ⇒ 每一页都"嘟"一声，
//    而画面其实完全正常。这里用 err_abort 把它挡掉：
//        beep_err = ld_err & ~ld_err_abort
//    真正的失败（头非法 / 排空超时 / 越界 / 偏移未对齐）ab_r=0 ⇒ 照旧响。
//------------------------------------------------------------------------------
wire       beep_err  = ld_err & ~ld_err_abort;
wire [1:0] beep_seg  = beep_err ? 2'd2 : (key_press[0] ? 2'd1 : 2'd0);
wire       beep_trig = (|key_press) | beep_err;

beep_player u_beep (
    .clk_sd  (sd_card_clk),
    .rst_sd  (rst_all),
    .trig    (beep_trig),
    .seg     (beep_seg),
    .fl_req  (bp_req),
    .fl_addr (bp_addr),
    .fl_len  (bp_len),
    .fl_rdy  (bp_rdy),
    .fl_data (bp_data),
    .fl_valid(bp_valid),
    .fl_done (bp_done),
    .clk_v   (video_clk),
    .rst_v   (rst_all),
    .smp_tick(src_valid_w),
    .o_smp   (beep_smp),
    .o_act   (beep_act)
);

// ---- 第 3 步：计算 ACR 参数 ----
// ACR = Audio Clock Regeneration（音频时钟重建）。
// HDMI 音频不是单独走线的，而是嵌在 TMDS 的视频消隐期里传输。
// 接收端（显示器）需要知道"音频采样时钟和 TMDS 时钟的比例关系"，
// 才能从 TMDS 时钟恢复出准确的 48kHz 采样时钟——这个比例就是 CTS/N。
//   标准公式：128 × fs = fTMDS × N / CTS
// 本工程固定 N = 6144（48kHz 音频的标准值），CTS 由本模块动态计算。
audio_arc_calculate #(
    .ACR_N         (6144)               // 48kHz 对应的标准 N 值
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_audio_valid (audio_valid),       // 统计采样节拍
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

//==============================================================================
//  【视频打包】RGB + DE/VS → AXI-Stream
//==============================================================================
// HDMI 1.4b IP 不接受"hs/vs/de + 并行像素"这种传统格式，
// 它要求 AXI-Stream 视频流：每个像素一拍，同时用 user/valid/last 三个标志
// 标明"这是帧首像素 / 这是有效像素 / 这是行末像素"。
// 这个模块就负责做格式转换。
video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    // 【2026-11-03】同步信号换用最终后处理输出（pp_out_*）：频谱/字幕先叠加、
    // 后处理再延迟 3 拍，vs/de 必须用同样延迟过的版本，否则数据与行首错位。
    .I_vs          (pp_out_vs),
    .I_de          (pp_out_de),
    .I_rgb         (vout_data),         // 频谱 + 字幕 + 画质后处理 + 黑屏门控后的最终像素
    .O_video_user  (axis_s_user),       // → 帧首
    .O_video_valid (axis_s_valid),      // → 数据有效
    .O_video_last  (axis_s_last),       // → 行末
    .O_video_data  (axis_s_data)
);

//==============================================================================
//  【EDID 读取触发】上电后自动打一拍
//==============================================================================
// EDID 是显示器里的一块小 ROM，记录了"这台显示器支持哪些分辨率/刷新率/音频格式"。
// HDMI 标准要求发射端先读 EDID，确认对方支持的模式再发送，否则可能不被识别。
//
// 【为什么要延时 100000 个时钟（约 4ms @25MHz）】
// 显示器上电后需要一段时间准备内部的 I2C 从机电路。如果 FPGA 一上电就去读，
// 显示器还没准备好，读取会失败，导致 HDMI 握手不成功、黑屏。
// 等 4ms 是给显示器留出准备时间。
//
// 这个模块只产生"一次的"脉冲（内部有 S_done 标志，产生后不再重复）。
//------------------------------------------------------------------------------
//  ★ 2026-11-01 [EDID-OFF]：默认不读 EDID（省 ~216 LE + 2 BRAM9K）
//------------------------------------------------------------------------------
//  读回的 EDID 字节本工程从来没用（O_edid_read_* 悬空），IP 内的 hdmi_edid_wrapper
//  （I²C 主机 + 1024 深 FIFO）只是因为驱动着 DDC 两根脚才没被裁掉。
//  这里不再把 IP 的 DDC 口接到管脚、触发拉 0 ⇒ 它没有任何可观察输出 ⇒ TD 整块裁掉。
//  DDC 管脚：SCL 保持 1（I²C 空闲电平），SDA 高阻（外部上拉）。
//  显示器不要求发射端去读 EDID；如遇个别显示器不认，定义 HDMI_EDID_EN 即可恢复原样。
//------------------------------------------------------------------------------
`ifdef HDMI_EDID_EN
startup_pulse #(
    .CNT_MAX(20'd100000)                // 数 100000 个 video_clk ≈ 4ms
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)               // → 送给 HDMI IP 触发 EDID 读取
);
`else
assign edid_trig    = 1'b0;
assign HDMI_DDC_SCL = 1'b1;
assign HDMI_DDC_SDA = 1'bz;
`endif

//==============================================================================
//  【HDMI 1.4b 发射核】视频 + 音频组包，TMDS 编码
//==============================================================================
// 这是安路官方的加密 IP，看不到内部实现。它做这些事：
//   1. 读取 EDID，决定输出什么模式；
//   2. 在视频消隐期插入数据岛包（Data Island Packet），把音频样本和 ACR 参数塞进去；
//   3. 把视频/音频/控制信号做 TMDS 10bit 编码，输出 4 路 10bit 并行数据。
//
// 参数说明（这些都是 VESA 标准里定死的时序数字）：
//   分辨率 640×480@60Hz，VIC=1（VIC 是 HDMI 规范里给每种视频模式编的号）
//   行：HACTIVE=640 有效像素 + HFP=16 前肩 + HSA=96 同步脉宽 + HBP=48 后肩 = HTOTAL=800
//   场：VACTIVE=480 有效行   + VFP=10 前肩 + VSA=2 同步脉宽  + VBP=33 后肩 = VTOTAL=525
//   （回忆一下：800 × 525 × 60Hz ≈ 25.2MHz，正是像素时钟的频率）
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),   // 目标器件系列：EG4 系列
    .HTOTAL                 ( 800        ),   // 一行总像素数（含消隐）
    .HSA                    ( 96         ),   // 行同步脉宽
    .HFP                    ( 16         ),   // 行前肩
    .HBP                    ( 48         ),   // 行后肩
    .HACTIVE                ( 640        ),   // 一行有效像素数
    .VTOTAL                 ( 525        ),   // 一场总行数（含消隐）
    .VSA                    ( 2          ),   // 场同步脉宽
    .VFP                    ( 10         ),   // 场前肩
    .VBP                    ( 33         ),   // 场后肩
    .VACTIVE                ( 480        ),   // 一场有效行数
    .VIDEO_VIC              ( 1          ),   // HDMI 视频模式编号（1 = 640×480@60）
    .VIDEO_TPG              ( "Disable"  ),   // 关闭内部测试图案（我们用真实的 TF 卡图片）
    .VIDEO_FORMAT           ( "RGB"      ),   // 色彩格式：RGB888
    .AUDIO_SAMPLE_RATE      ( "48K"      ),   // 音频采样率：48kHz
    .IIC_SCL_DIV            ( 250        )    // DDC I2C 时钟分频（决定读 EDID 的速率）
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),          // 像素时钟
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),          // 上电后自动触发一次 EDID 读取
    .O_edid_read_valid  (edid_valid),         // EDID 读出的数据（本工程未使用）
    .O_edid_read_data   (edid_data),

    // 视频输入：AXI-Stream 从接口
    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),       // IP 的反压信号（本工程未做流控）

    // 音频输入
    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),                   // 视频锁定标志，未使用
`ifdef HDMI_EDID_EN
    .O_ddc_scl          (HDMI_DDC_SCL),       // DDC I2C 时钟 → 直接连到顶层端口
    .IO_ddc_sda         (HDMI_DDC_SDA),       // DDC I2C 数据 → 双向端口
`else
    .O_ddc_scl          (),                   // [EDID-OFF] 不接 ⇒ edid_wrapper 被裁
    .IO_ddc_sda         (),
`endif

    // TMDS 编码输出（4 路 10bit 并行）
    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

//==============================================================================
//  【HDMI 物理层】TMDS 10bit 并行 → LVDS 串行差分
//==============================================================================
// TMDS 每个通道是 10bit 并行数据，但 HDMI 线上跑的是串行差分信号。
// 这个模块把每路 10bit 用 5 倍频时钟（hdmi_5x_clk = 125MHz）双沿输出，
// 5 个时钟周期 × 2 个沿 = 10 个比特，正好串完。
// 位速率 = 125MHz × 2 = 250Mbps，乘 10bit 还原 = 25MHz 像素率。
hdmi_phy_wrapper #(
    .DEVICE ( "EG" )                    // 器件系列，决定用哪个 ODDR 原语
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),          // 像素时钟（用于产生 load 信号）
    .I_serial_clk       (hdmi_5x_clk),        // 5 倍频串行时钟
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),      // R 通道（含控制信号）
    .I_tmds_channel_1   (tmds_ch1_data),      // G 通道（含控制信号）
    .I_tmds_channel_2   (tmds_ch2_data),      // B 通道（含控制信号）
    .I_tmds_channel_clk (tmds_clk_data),      // 时钟通道
    .O_tmds_ch0_p       (HDMI_D0_P),          // → 板卡 HDMI_B 接口
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule
