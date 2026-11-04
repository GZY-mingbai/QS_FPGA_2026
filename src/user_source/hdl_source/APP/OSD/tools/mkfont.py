#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mkfont.py —— 生成 OSD 字形 ROM（osd_font.hex）

源：Font/unifont-17.0.01.bmp（1bpp，256 列 × 256 行，每格 16×16，原点 (col*16+32, row*16+64)）
输出：osd_font.hex —— 256 字形 × 16 行，每行 1 个 16bit 字（8 像素 × 2bpp）

字内编码（与 osd_text.v 严格一致）：
    bit[15:14] = 该行第 0 列像素（最左），bit[1:0] = 第 7 列
    2bpp 取值：0 = 透明（不着墨），3 = 满墨；本工具只产 0/3（源头是 1bpp）

为什么用 unifont：它的 ASCII 是原生 8×16 点阵（占 16×16 格子的左 8 列、中间 10 行），
不需要像 Font/gen_font.py 那样把 8×8 放大 2 倍（那会变成"拉长字"）。

用法：
    python mkfont.py                 # 生成 osd_font.hex
    python mkfont.py --preview A     # 顺便打印 A 的点阵（自检）
本脚本只用标准库，不依赖 Pillow。
"""
import argparse
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", "..", ".."))
BMP_DEFAULT = os.path.join(ROOT, "Font", "unifont-17.0.01.bmp")

CELL = 16          # unifont 格子边长
COLS = 256         # 整图 256 列
ORIGIN_X = 32
ORIGIN_Y = 64
NGLYPH = 256       # 覆盖 0x00~0xFF（0x80 以上填空，留给以后）
ROWS = 16


class Bmp1bit:
    """极简 1bpp BMP 读取（只支持未压缩、1bpp、自底向上/上向下）"""

    def __init__(self, path):
        with open(path, "rb") as f:
            self.d = f.read()
        if self.d[:2] != b"BM":
            raise ValueError("not a BMP")
        self.off = struct.unpack_from("<I", self.d, 10)[0]
        self.w = struct.unpack_from("<i", self.d, 18)[0]
        self.h = struct.unpack_from("<i", self.d, 22)[0]
        self.bpp = struct.unpack_from("<H", self.d, 28)[0]
        comp = struct.unpack_from("<I", self.d, 30)[0]
        if self.bpp != 1 or comp != 0:
            raise ValueError("expect 1bpp uncompressed, got bpp=%d comp=%d" % (self.bpp, comp))
        self.rowbytes = ((self.w * self.bpp + 31) // 32) * 4

    def get(self, x, y):
        yy = (self.h - 1 - y) if self.h > 0 else y
        base = self.off + yy * self.rowbytes + (x >> 3)
        return (self.d[base] >> (7 - (x & 7))) & 1


def glyph_rows(bmp, ch):
    """取一个 ASCII 字符的 16 行 × 8 列点阵（1 = 墨）"""
    u = ord(ch)
    ox = (u & 0xFF) * CELL + ORIGIN_X
    oy = (u >> 8) * CELL + ORIGIN_Y
    rows = []
    for y in range(ROWS):
        v = 0
        for x in range(8):
            # ★ unifont 是"黑字白底"的 1bpp：像素 0 才是笔画，必须取反
            v = (v << 1) | (1 - bmp.get(ox + x, oy + y))
        rows.append(v)          # 8 bit，bit7 = 最左
    return rows


def rows_to_word(r8):
    """8 像素 × 2bpp → 16bit（墨 = 3）"""
    w = 0
    for x in range(8):
        bit = (r8 >> (7 - x)) & 1
        w = (w << 2) | (3 if bit else 0)
    return w


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bmp", default=BMP_DEFAULT)
    ap.add_argument("--out", default=os.path.join(HERE, "osd_font.hex"))
    ap.add_argument("--preview", default="", help="打印这些字符的点阵后退出")
    a = ap.parse_args()

    bmp = Bmp1bit(a.bmp)

    if a.preview:
        for ch in a.preview:
            print("=== %r" % ch)
            for r in glyph_rows(bmp, ch):
                print("".join("#" if (r >> (7 - i)) & 1 else "." for i in range(8)))
        return

    if not os.path.isdir(os.path.dirname(a.out) or "."):
        os.makedirs(os.path.dirname(a.out), exist_ok=True)

    lines = []
    for g in range(NGLYPH):
        ch = chr(g) if 0x20 <= g < 0x7F else " "
        rows = glyph_rows(bmp, ch)
        for r in rows:
            lines.append("%04x" % rows_to_word(r))

    with open(a.out, "w", encoding="ascii", newline="\n") as f:
        for ln in lines:
            f.write(ln + "\n")

    print("wrote %s  (%d words = 256 glyphs x 16 rows)" % (a.out, len(lines)))
    # 自检：把 'A' 打出来
    for r in glyph_rows(bmp, "A"):
        print("".join("#" if (r >> (7 - i)) & 1 else "." for i in range(8)))


if __name__ == "__main__":
    sys.exit(main())
