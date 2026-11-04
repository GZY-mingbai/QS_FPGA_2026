#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_font.py —— 生成 FPGA 用 GB2312 点阵字库（.bin，无压缩）

数据来源
    ascii.png            ASCII 0x21~0x7E 的 8x8 点阵（工程自带，风格统一）
                         提取后 2 倍放大到 16x16，与汉字块尺寸一致
    unifont-17.0.01.bmp  其余全部字符的 16x16 点阵（含 GB2312 全部汉字与符号区）

输出
    font_gb2312.bin      扁平二进制，无压缩、无表头，地址可直接算
                         全部字形统一 16x16 / 32 字节步进
    （格式细节见同目录 字库格式说明.md）

依赖
    pip install pillow

用法
    python gen_font.py
    python gen_font.py --out font_gb2312.bin --preview font_preview.png
"""

import argparse
import hashlib
import os
import sys

from PIL import Image

# ----------------------------------------------------------------------------
# 源图几何参数（实测确认，勿随意改动）
# ----------------------------------------------------------------------------
# ascii.png：128x128，P 模式（调色板损坏，必须读「原始索引」而非 RGB/L 值）
#   · 索引 1 = 笔画，索引 0 = 背景
#   · 单元格 8x8，横向 16 格，纵向 6 格，从 y=16 开始
#   · 字符 c 的格子 = ((c-0x20) % 16, (c-0x20) // 16)
ASCII_IMG = "ascii.png"
A_W, A_H = 128, 128
A_CELL = 8          # 单元格边长
A_COLS = 16         # 每行格数
A_YOFF = 16         # 顶部留白（首行点阵的 y 起点）
A_INK = 1           # 笔画对应的调色板索引

# unifont-17.0.01.bmp：4128x4160，1 位图，0=黑=笔画
#   · 单元格 16x16，整图 256 列 x 256 行
#   · 码位 U 的格子 = (U & 0xFF, U >> 8)，像素原点 = (格列*16+32, 格行*16+64)
UNI_IMG = "unifont-17.0.01.bmp"
U_CELL = 16
U_DX, U_DY = 32, 64

# ----------------------------------------------------------------------------
# 输出布局参数
# ----------------------------------------------------------------------------
ASCII_COUNT = 128               # 覆盖 0x00~0x7F，直接索引 = c * 32
ASCII_BYTES = (U_CELL // 8) * U_CELL             # 放大到 16x16 后 → 32 字节
ASCII_BLOCK = ASCII_COUNT * ASCII_BYTES          # = 4096 字节

GB_ROWS = list(range(0xA1, 0xAA)) + list(range(0xB0, 0xF8))   # 区1-9 + 区16-87
GB_COLS = 94                    # 每区 94 个位
GB_BYTES = (U_CELL // 8) * U_CELL                # 16x16 → 32 字节
GB_BLOCK = len(GB_ROWS) * GB_COLS * GB_BYTES

GB_BASE = ASCII_BLOCK           # GB2312 区在文件中的起始偏移
TOTAL_SIZE = ASCII_BLOCK + GB_BLOCK


# ----------------------------------------------------------------------------
# 加载源图
# ----------------------------------------------------------------------------
def load_ascii_sheet(path):
    """读 ascii.png，返回 (像素索引列表, 宽, 高)。

    注意：该 PNG 调色板是坏的（所有项都是白色），用 convert('L') 或 RGB
    取值会得到全白。必须直接用 getdata() 拿调色板索引。
    """
    im = Image.open(path)
    if im.size != (A_W, A_H):
        raise ValueError("ascii.png 尺寸应为 %dx%d，实际 %s" % (A_W, A_H, im.size))
    idx = list(im.getdata())          # P 模式 → 调色板索引
    uniq = set(idx)
    if not uniq.issubset({0, 1}):
        raise ValueError("ascii.png 出现意外的调色板索引: %s" % sorted(uniq)[:8])
    return idx, im.size[0], im.size[1]


def load_unifont(path):
    """读 unifont BMP，返回 (灰度字节缓冲, 宽)。1 字节/像素，0=黑=笔画。"""
    im = Image.open(path).convert("L")
    w, h = im.size
    if (w, h) != (256 * U_CELL + U_DX, 256 * U_CELL + U_DY):
        raise ValueError("unifont 尺寸异常: %s" % (im.size,))
    return im.tobytes(), w


# ----------------------------------------------------------------------------
# 点阵提取
# ----------------------------------------------------------------------------
def ascii_glyph(sheet, width, ch, shift_x=0, ascii_fill=13):
    """取 8x8 ASCII 点阵，2 倍放大到 16x16，返回 32 字节。

    放大方式为「像素复制」（最近邻）：一个源像素 → 2x2 目标像素，
    保证点阵锐利、不引入插值灰阶，也不改变字形形状。
    输出格式与汉字块完全一致：每行 2 字节，先左半后右半，bit7 = 最左。

    shift_x：放大后整体右移的像素数（0~8）。
        源图字形在 8 列格内是左对齐的（笔画约占左侧 5~6 列），
        放大到 16 列后仍是左对齐。若用 16px 等宽排版，看起来会偏左，
        可用 shift_x=2 左右平衡；若用 8px 半宽排版，保持 0 即可。
    """
    if not (0x21 <= ch <= 0x7E):
        return bytes(U_CELL * 2)                  # 空格/控制符/0x7F 一律留空
    n = ch - 0x20
    x0 = (n % A_COLS) * A_CELL
    y0 = (n // A_COLS) * A_CELL + A_YOFF

    # 1) 先按源图取 8x8（每字节一行，bit7 = 最左）
    rows8 = []
    for r in range(A_CELL):
        base = (y0 + r) * width + x0
        b = 0
        for i in range(A_CELL):
            if sheet[base + i] == A_INK:
                b |= 1 << (7 - i)
        rows8.append(b)

    # 2) 纵向放大 2 倍（每源行 → 2 目标行）；横向按 scale_x 等比放大墨迹
    #
    #  ★ 2026-10-30：原来横向也是 2 倍（源 5 列 → 目标 10 列），右边固定留 6 列空白。
    #    16px 等宽步进下字距就是 6px，英文看起来像 "L a n g u a g e"。
    #    现在：取字形墨迹列范围 [c0,c1]，按 scale_x 等比放大（默认 2.6）后居中。
    #      普通字母墨迹 5 列 → 13 列 ⇒ 字距 3px（16px 字号的 0.19em，正常）
    #      'i'/'l'（1 列）→ 3 列、't'（3 列）→ 8 列 ⇒ **保持宽窄比例**，
    #      不会出现"i 和 m 一样宽"的怪样子。
    #    scale_x=2.0 就是改造前的效果；纵向始终 2 倍，所以竖笔 2px、横笔 2-3px。
    shift_x = max(0, min(8, shift_x))
    sxx = max(2.0, min(3.4, float(ascii_fill) / 5.0))   # ascii_fill=13 → 2.6

    used = [i for i in range(A_CELL) if any(rows8[r] & (1 << (7 - i)) for r in range(A_CELL))]
    if used:
        c0, c1 = min(used), max(used)
    else:
        c0, c1 = 0, 0
    ws = c1 - c0 + 1
    tw = max(2, min(U_CELL, int(round(ws * sxx))))      # 目标墨迹宽度（列）
    xoff = (U_CELL - tw) // 2 + shift_x                 # 居中

    out = bytearray()
    for r in range(U_CELL):
        src = rows8[r // 2]
        w16 = 0
        for k in range(tw):
            j = c0 + (k * ws) // tw                     # 目标第 k 列取源第 j 列（最近邻）
            if (src >> (7 - j)) & 1:
                x = xoff + k
                if 0 <= x < U_CELL:
                    w16 |= 1 << (15 - x)
        out.append((w16 >> 8) & 0xFF)
        out.append(w16 & 0xFF)
    return bytes(out)


def unifont_glyph(buf, width, cp):
    """取 16x16 点阵，返回 32 字节（每行 2 字节，先左半后右半，bit7 = 最左）。"""
    cx, cy = cp & 0xFF, (cp >> 8) & 0xFF
    x0, y0 = cx * U_CELL + U_DX, cy * U_CELL + U_DY
    out = bytearray()
    for r in range(U_CELL):
        base = (y0 + r) * width + x0
        hi = lo = 0
        for i in range(8):
            if buf[base + i] == 0:                # 黑 = 笔画
                hi |= 1 << (7 - i)
            if buf[base + 8 + i] == 0:
                lo |= 1 << (7 - i)
        out.append(hi)
        out.append(lo)
    return bytes(out)


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def build(ascii_path, unifont_path, ascii_shift_x=0, ascii_fill=13):
    sheet, aw, _ = load_ascii_sheet(ascii_path)
    ubuf, uw = load_unifont(unifont_path)

    data = bytearray(TOTAL_SIZE)

    # ---- 块 0：ASCII 8x8 → 放大到 16x16 ----
    for c in range(ASCII_COUNT):
        data[c * ASCII_BYTES:(c + 1) * ASCII_BYTES] = \
            ascii_glyph(sheet, aw, c, ascii_shift_x, ascii_fill)

    # ---- 块 1：GB2312 16x16 ----
    stats = {"ok": 0, "blank": 0, "blank_list": []}
    for row, qb in enumerate(GB_ROWS):
        for wb in range(0xA1, 0xFF):
            off = GB_BASE + (row * GB_COLS + (wb - 0xA1)) * GB_BYTES
            try:
                ch = bytes([qb, wb]).decode("gb2312")
            except UnicodeDecodeError:
                stats["blank"] += 1
                stats["blank_list"].append((qb, wb))
                continue
            data[off:off + GB_BYTES] = unifont_glyph(ubuf, uw, ord(ch))
            stats["ok"] += 1

    return data, stats


def preview(data, path, sample):
    """从生成的 bin 反渲染抽样字形，用于肉眼校验。全部按 16x16 统一排版。"""
    CELL_W, CELL_H = 16, 16
    cols = 12
    pad = 2
    rows = (len(sample) + cols - 1) // cols
    img = Image.new("L", (cols * (CELL_W + pad) + pad,
                          rows * (CELL_H + pad) + pad), 255)
    px = img.load()
    for n, (label, off, nbytes) in enumerate(sample):
        gx = (n % cols) * (CELL_W + pad) + pad
        gy = (n // cols) * (CELL_H + pad) + pad
        for r in range(16):
            hi, lo = data[off + r * 2], data[off + r * 2 + 1]
            for i in range(8):
                if hi & (1 << (7 - i)):
                    px[gx + i, gy + r] = 0
                if lo & (1 << (7 - i)):
                    px[gx + 8 + i, gy + r] = 0
    img = img.resize((img.size[0] * 3, img.size[1] * 3), Image.NEAREST)
    img.save(path)
    return img.size


def gb_offset(qb, wb):
    """演示寻址公式：GB2312 双字节 → 文件内字节偏移。"""
    row = (qb - 0xA1) if qb <= 0xA9 else (qb - 0xB0 + 9)
    return GB_BASE + (row * GB_COLS + (wb - 0xA1)) * GB_BYTES


def main():
    ap = argparse.ArgumentParser(description="生成 GB2312 点阵字库")
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--ascii", default=os.path.join(here, ASCII_IMG))
    ap.add_argument("--unifont", default=os.path.join(here, UNI_IMG))
    ap.add_argument("--out", default=os.path.join(here, "font_gb2312.bin"))
    ap.add_argument("--preview", default=os.path.join(here, "font_preview.png"))
    ap.add_argument("--ascii-shift-x", type=int, default=0,
                    help="ASCII 居中后额外右移的像素数 0~8（一般 0 即可）")
    ap.add_argument("--ascii-fill", type=int, default=13,
                    help="ASCII 墨迹横向占满多少列（8~16）。10=改造前，13=推荐，16=贴死")
    args = ap.parse_args()

    data, stats = build(args.ascii, args.unifont, args.ascii_shift_x, args.ascii_fill)

    with open(args.out, "wb") as f:
        f.write(data)

    sha = hashlib.sha256(data).hexdigest()

    print("=" * 68)
    print("字库生成完成")
    print("=" * 68)
    print("ASCII 块 : 偏移 0x%06X, %d 字形(16x16) x %d 字节 = %d 字节"
          % (0, ASCII_COUNT, ASCII_BYTES, ASCII_BLOCK))
    print("GB2312 块: 偏移 0x%06X, %d 区 x %d 位(16x16) x %d 字节 = %d 字节"
          % (GB_BASE, len(GB_ROWS), GB_COLS, GB_BYTES, GB_BLOCK))
    print("总大小   : %d 字节 (%.2f KiB)" % (TOTAL_SIZE, TOTAL_SIZE / 1024))
    print("-" * 68)
    print("GB2312 有效字形 : %d 个" % stats["ok"])
    print("GB2312 未定义位 : %d 个（已填 0）" % stats["blank"])
    print("SHA-256        : %s" % sha)
    print("-" * 68)

    # 抽样回渲染：ASCII + 常用汉字 + GB2312 符号区
    sample = []
    for ch in "AaZz09!~":
        sample.append((ch, ord(ch) * ASCII_BYTES, ASCII_BYTES))
    for qb, wb in [(0xB0, 0xA1), (0xD6, 0xD0), (0xCE, 0xC4), (0xD3, 0xC3),
                   (0xB7, 0xFB), (0xCA, 0xFD), (0xA1, 0xA3), (0xA3, 0xB0),
                   (0xA4, 0xA1), (0xA6, 0xA1), (0xA8, 0xA1), (0xB5, 0xC4)]:
        label = bytes([qb, wb]).decode("gb2312")
        sample.append((label, gb_offset(qb, wb), GB_BYTES))
    w, h = preview(data, args.preview, sample)
    print("抽样预览 : %s (%dx%d)" % (args.preview, w, h))
    print("输出文件 : %s" % args.out)
    print("=" * 68)

    return 0


if __name__ == "__main__":
    sys.exit(main())
