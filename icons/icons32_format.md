# icons32.bin 读取指南

> ⚠️ **已作废（2026-10-28）**：OSD 位图已改成
> 「图标 RGB565 + logo ARGB8888」的单一文件 `osd_bitmaps.bin`，
> 不再读取本文件。本文只作为旧格式的存档保留。
> 新格式见 **`icons/osd_bitmaps_format.md`**。


导航图标（显示调整 / 相册操作 / 文件管理 / 系统设置 / 关于信息）的打包格式与解码方法。

| 项 | 值 |
|---|---|
| 文件 | `icons/icons32.bin` |
| 大小 | **2656 字节（0xA60）**，无文件头、无压缩、无填充位 |
| 烧录位置 | Flash `0x03D000`（见 `FlashProgrammer/host/partitions.json`） |
| 图像 | 32×32 像素，每像素 4bit 色阶（0..15） |
| 图标数 | 5 张覆盖率图 × 2 组调色板 = 10 个图标（5 个 × 普通/激活） |

---

## 1. 为什么是 4bpp 而不是 RGB

每个图标只有两种颜色（底板色 + 墨色），抗锯齿产生的一切中间色都落在「底板 → 墨色」这一条
直线上。所以一个像素不必存 RGB，只要存一个 **0..15 的色阶（覆盖率）**：

- `0` = 纯底板色，`15` = 纯墨色，中间值 = 两者线性插值。
- 32 像素一行 = 16 字节，整除，**没有跨行填充**；半字节紧凑，**没有无效位**。
- 普通态和激活态的几何完全一致，只有底板/墨色不同 —— 所以**共用同一张覆盖率图**，只切调色板。

体积对比（同一套 10 个图标）：

| 方案 | 每图标 | 合计 | 备注 |
|---|---|---|---|
| ARGB8888 | 4096 B | 40 KB | 带 alpha，浪费 |
| RGB565 | 2048 B | 20 KB | 无 alpha |
| 8bpp 色阶 | 1024 B | 10.2 KB | |
| **4bpp 色阶（本方案）** | **512 B** | **2.6 KB** | 16 级，最大误差 6/255 |
| 1bpp 二值 | 128 B | 1.3 KB | 无灰度，笔画发毛 |

---

## 2. 内存布局

```
偏移(文件)      长度     内容
0x0000         512B    图标 0 覆盖率图  display
0x0200         512B    图标 1 覆盖率图  album
0x0400         512B    图标 2 覆盖率图  files
0x0600         512B    图标 3 覆盖率图  settings
0x0800         512B    图标 4 覆盖率图  about
0x0A00          48B    调色板 · 普通态  16 × RGB888（3 字节/级）
0x0A30          48B    调色板 · 激活态  16 × RGB888
0x0A60          ——     文件结束
```

覆盖 Flash 基址 `0x03D000` 后的绝对地址：

| 内容 | 文件偏移 | Flash 绝对地址 |
|---|---|---|
| display 覆盖率图 | 0x0000 | `0x03D000` |
| album 覆盖率图 | 0x0200 | `0x03D200` |
| files 覆盖率图 | 0x0400 | `0x03D400` |
| settings 覆盖率图 | 0x0600 | `0x03D600` |
| about 覆盖率图 | 0x0800 | `0x03D800` |
| 调色板 · 普通 | 0x0A00 | `0x03DA00` |
| 调色板 · 激活 | 0x0A30 | `0x03DA30` |

> 分区按 4KB 扇区预留 `0x03D000 ~ 0x03DFFF`，文件只占前 2656 字节，其余为空闲余量
> （一个扇区可存 32 张覆盖率图，后续加图标不用挪分区）。

---

## 3. 每个图标的 offset（与界面索引一一对应）

导航栏索引 `idx = 0..4`，`focus = 0/1`：

| idx | 图标 | 覆盖率图偏移 | 普通态调色板 | 激活态调色板 |
|---|---|---|---|---|
| 0 | 显示调整 | `0x0000` | `0x0A00` | `0x0A30` |
| 1 | 相册操作 | `0x0200` | `0x0A00` | `0x0A30` |
| 2 | 文件管理 | `0x0400` | `0x0A00` | `0x0A30` |
| 3 | 系统设置 | `0x0600` | `0x0A00` | `0x0A30` |
| 4 | 关于信息 | `0x0800` | `0x0A00` | `0x0A30` |

对应的 10 个源文件（`icons/` 与 `icons/png32/`）与偏移的映射：

| 源文件 | 覆盖率图 | 调色板 |
|---|---|---|
| `ico-display.svg/.png` | `0x0000` | 普通 `0x0A00` |
| `ico-display-active.svg/.png` | `0x0000` | 激活 `0x0A30` |
| `ico-album.svg/.png` | `0x0200` | 普通 `0x0A00` |
| `ico-album-active.svg/.png` | `0x0200` | 激活 `0x0A30` |
| `ico-files.svg/.png` | `0x0400` | 普通 `0x0A00` |
| `ico-files-active.svg/.png` | `0x0400` | 激活 `0x0A30` |
| `ico-settings.svg/.png` | `0x0600` | 普通 `0x0A00` |
| `ico-settings-active.svg/.png` | `0x0600` | 激活 `0x0A30` |
| `ico-about.svg/.png` | `0x0800` | 普通 `0x0A00` |
| `ico-about-active.svg/.png` | `0x0800` | 激活 `0x0A30` |

---

## 4. 像素寻址与位序

- 覆盖率图区：**行优先**，每行 32 像素 = 16 字节，第 y 行起始字节 = `y × 16`。
- 一个字节装两个像素：**高半字节 = 左边（偶数 x）像素，低半字节 = 右边（奇数 x）像素**。
- 色阶 0..15 直接就是调色板索引，**没有符号位、没有 alpha 通道**。

```
文件偏移 = 图标偏移 + y*16 + (x >> 1)
lvl      = (x 为偶数) ? (byte >> 4) : (byte & 0x0F)
RGB      = 调色板基址 + lvl*3 + { 0,1,2 }        // R,G,B 各 1 字节
```

例：`idx=0, x=7, y=12` → `0x0000 + 12*16 + 3 = 0x00C3`，读到 `0x50` → x=7 是奇数，
取低半字节 `0x0` → 调色板第 0 级 = `#C0C0C0`（底板）。

---

## 5. 调色板（线性插值，16 级）

| 组 | 基址 | 级 0（底板） | 级 15（墨色） |
|---|---|---|---|
| 普通 | `0x0A00` | `#C0C0C0` | `#161616` |
| 激活 | `0x0A30` | `#3080FF` | `#FFFFFF` |

每级 3 字节，顺序为 **R, G, B**（无 alpha）。完整 96 字节：

```
普通 0x0A00:
c0c0c0 b5b5b5 a9a9a9 9e9e9e 939393 878787 7c7c7c 717171
656565 5a5a5a 4f4f4f 434343 383838 2d2d2d 212121 161616

激活 0x0A30:
3080ff 3e88ff 4c91ff 5999ff 67a2ff 75aaff 83b3ff 91bbff
9ec4ff acccff bad5ff c8ddff d6e6ff e3eeff f1f7ff ffffff
```

> 调色板已经展开成表，FPGA 侧**不需要做乘加混合**，直接查表即可（16 项 × 2 组，
> 一个 32×24bit 的小 ROM 就够）。换配色只改这 96 字节，覆盖率图不用动。

---

## 6. 解码伪代码

### 6.1 Verilog

```verilog
localparam [23:0] ICON_BASE = 24'h3D000;   // 分区基址
localparam [23:0] PAL_BASE  = 24'h3DA00;   // 0x3D000 + 0x0A00

// 取一个像素的 RGB（组合逻辑示意，实际要按 flash 读时序打拍）
//   idx    : 0..4     导航索引
//   focus  : 1=激活态
//   x, y   : 0..31    像素坐标
addr     = ICON_BASE + (idx << 9) + (y << 4) + (x >> 1);   // 512 = 1<<9, 16 = 1<<4
byte     = flash_rd8(addr);
lvl      = x[0] ? byte[3:0] : byte[7:4];
pal_addr = PAL_BASE + (focus ? 24'd48 : 24'd0) + lvl*3;    // 48 = 0x30
rgb      = { flash_rd8(pal_addr), flash_rd8(pal_addr+1), flash_rd8(pal_addr+2) };
alpha    = 8'hFF;                                          // 底板不透明
```

要点：

1. **按 32bit 读更划算**：一次读 4 字节 = 8 个像素，行内只需 8 次读
   （`行基址 + (x>>3)*4`，再用 `x[2:0]` 选字节）。
2. `lvl*3` 用 `lvl + (lvl<<1)` 合成，不占 DSP。
3. 调色板地址只随 `focus` 和 `lvl` 变，同一行内可以缓存；覆盖率图每行 16 字节，
   整张图 512 字节，完全可以按行预取进一行 BRAM。
4. 换图标只需改 `idx`（`idx<<9`），换状态只需改 `focus` —— 两者互不影响。

### 6.2 Python（用于校验 bin）

```python
BASE_PAL = 0x0A00

def pixel(blob, idx, x, y, active=False):
    off = idx * 512 + y * 16 + (x >> 1)
    b = blob[off]
    lvl = (b & 0x0F) if (x & 1) else (b >> 4)
    p = BASE_PAL + (0x30 if active else 0) + lvl * 3
    return tuple(blob[p:p + 3])          # (R, G, B)

blob = open("icons/icons32.bin", "rb").read()
assert len(blob) == 0xA60
assert pixel(blob, 0, 0, 0) == (0xC0, 0xC0, 0xC0)      # 左上角是底板色
assert pixel(blob, 0, 0, 0, active=True) == (0x30, 0x80, 0xFF)
```

---

## 7. 校验样本

烧录后可以直接比对这些字节确认写对了：

| 位置 | 期望值 |
|---|---|
| 文件长度 | `2656`（`0x0A60`） |
| `0x0A00` 前 12B（普通级 0..3） | `c0 c0 c0 b5 b5 b5 a9 a9 a9 9e 9e 9e` |
| `0x0A2D` 前 3B（普通级 15） | `16 16 16` |
| `0x0A30` 前 3B（激活级 0） | `30 80 ff` |
| `0x0A5D` 后 3B（激活级 15） | `ff ff ff` |

各图 `y = 12` 行的前 8 字节（`y*16 = +192`）：

| 图标 | 行起始偏移 | 前 8 字节 |
|---|---|---|
| display | `0x00C0` | `00 00 af 50 00 03 fd 10` |
| album | `0x02C0` | `00 00 af 50 4f ff 90 00` |
| files | `0x04C0` | `00 00 af 50 00 00 00 00` |
| settings | `0x06C0` | `00 00 00 00 9f 70 05 ce` |
| about | `0x08C0` | `00 00 0f f4 00 00 00 06` |

---

## 8. 重新生成

链路：`icons/*.svg`（24 网格矢量）→ 32×32 PNG（`icons/png32/`）→ `icons/icons32.bin`。

```bash
# 1. 矢量 → PNG（浏览器/设计工具导出，或复用现有 png32/）
# 2. PNG → bin
python tools/gen_icons_bin.py --check
```

`--check` 会把「原始 PNG / bin 解码结果」拼成对比图存到
`tools/out/icons_decode_check.png`，并打印每张图的往返一致性与误差：

```
icon      半调像素   往返一致   对原始 PNG 误差 最大/平均
display   191       是        6 / 0.6
album     215       是        6 / 0.5
files     104       是        6 / 0.2
settings  215       是        6 / 0.7
about     139       是        6 / 0.5
```

> 误差上限 6/255 来自 16 级量化（170/15 ≈ 11 级差，半级 ≈ 5.7），属于不可见范围。

---

## 9. 烧录

`FlashProgrammer/host/partitions.json` 已经把 `icons` 分区指向本文件：

```json
{ "name": "icons", "offset": "0x3D000", "file": "../../icons/icons32.bin", "size": "0x1000" }
```

改完图标重新生成 bin 后，直接按原流程烧录 `icons` 分区即可，字体分区（`0x000000`）不受影响。
