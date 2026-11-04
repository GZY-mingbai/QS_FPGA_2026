#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_font.py —— 校验 font_gb2312.bin 与源图逐字节一致，并渲染混排样张

校验思路：用「独立提取路径」（PIL crop）重新从源图取点阵，
与「按文档公式从 bin 里寻址取出的点阵」逐字节比对。
两条路径的代码完全不同，能有效发现位序/字节序/寻址错误。

用法：python verify_font.py
"""

import os
import random
import sys

from PIL import Image

BIN = os.path.join(os.path.dirname(os.path.abspath(__file__)), "font_gb2312.bin")
ASCII_IMG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ascii.png")
UNI_IMG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "unifont-17.0.01.bmp")

ASCII_BYTES = 32
GB_BASE = 4096
GB_COLS = 94
GB_BYTES = 32
GB_ROWS = list(range(0xA1, 0xAA)) + list(range(0xB0, 0xF8))


# --------------------------------------------------------------------------
# 从 bin 取字形（严格按文档公式）
# --------------------------------------------------------------------------
def bin_ascii(data, ch):
    if not (0x21 <= ch <= 0x7E):
        return bytes(32)
    return data[ch * ASCII_BYTES: (ch + 1) * ASCII_BYTES]


def bin_gb(data, qb, wb):
    row = (qb - 0xA1) if qb <= 0xA9 else (qb - 0xB0 + 9)
    off = GB_BASE + (row * GB_COLS + (wb - 0xA1)) * GB_BYTES
    return data[off: off + GB_BYTES]


# --------------------------------------------------------------------------
# 从源图独立提取（PIL crop 路径）
# --------------------------------------------------------------------------
_sheet = list(Image.open(ASCII_IMG).getdata())
_uni = Image.open(UNI_IMG).convert("L")


ASCII_SHIFT_X = 0        # 与 gen_font.py --ascii-shift-x 保持一致
ASCII_SCALE_X = 2.6      # 与 gen_font.py --ascii-fill 13 对应（13/5 = 2.6）


def ref_ascii(ch):
    """独立提取：从 ascii.png 取 8x8，再按文档约定 2 倍放大到 16x16。"""
    if not (0x21 <= ch <= 0x7E):
        return bytes(32)
    n = ch - 0x20
    x0, y0 = (n % 16) * 8, (n // 16) * 8 + 16
    box = Image.open(ASCII_IMG).crop((x0, y0, x0 + 8, y0 + 8))
    p = list(box.getdata())
    rows8 = []
    for r in range(8):
        b = 0
        for i in range(8):
            if p[r * 8 + i] == 1:
                b |= 1 << (7 - i)
        rows8.append(b)
    # ★ 2026-10-30：与 gen_font.py 同步 —— 墨迹按 ASCII_SCALE_X 等比放大后居中
    #   （原来横向固定 2 倍，右边空 6 列，16px 步进下字距过大）。
    used = [i for i in range(8) if any(rows8[r] & (1 << (7 - i)) for r in range(8))]
    c0, c1 = (min(used), max(used)) if used else (0, 0)
    ws = c1 - c0 + 1
    tw = max(2, min(16, int(round(ws * ASCII_SCALE_X))))
    xoff = (16 - tw) // 2 + ASCII_SHIFT_X
    out = bytearray()
    for r in range(16):
        src = rows8[r // 2]
        w16 = 0
        for k in range(tw):
            j = c0 + (k * ws) // tw
            if (src >> (7 - j)) & 1:
                x = xoff + k
                if 0 <= x < 16:
                    w16 |= 1 << (15 - x)
        out.append((w16 >> 8) & 0xFF)
        out.append(w16 & 0xFF)
    return bytes(out)


def ref_gb(ch):
    cp = ord(ch)
    cx, cy = cp & 0xFF, (cp >> 8) & 0xFF
    x0, y0 = cx * 16 + 32, cy * 16 + 64
    p = list(_uni.crop((x0, y0, x0 + 16, y0 + 16)).getdata())
    out = bytearray()
    for r in range(16):
        hi = lo = 0
        for i in range(8):
            if p[r * 16 + i] == 0:
                hi |= 1 << (7 - i)
            if p[r * 16 + 8 + i] == 0:
                lo |= 1 << (7 - i)
        out.append(hi)
        out.append(lo)
    return bytes(out)


# --------------------------------------------------------------------------
def main():
    data = open(BIN, "rb").read()
    print("bin 大小: %d 字节" % len(data))
    fails = []

    # 1) ASCII 0x21~0x7E 全量比对
    n_ok = 0
    for ch in range(0x21, 0x7F):
        if bin_ascii(data, ch) == ref_ascii(ch):
            n_ok += 1
        else:
            fails.append("ASCII 0x%02X" % ch)
    print("ASCII 0x21~0x7E : %d/%d 一致" % (n_ok, 0x7E - 0x20))

    # 2) ASCII 保留位必须全 0
    blank_ok = all(bin_ascii(data, c) == bytes(32)
                   for c in list(range(0x00, 0x21)) + [0x7F])
    print("ASCII 保留位全 0 : %s" % ("是" if blank_ok else "否 ← 异常"))
    if not blank_ok:
        fails.append("ASCII 保留位非零")

    # 3) GB2312 全量比对
    total = 0
    for qb in GB_ROWS:
        for wb in range(0xA1, 0xFF):
            try:
                ch = bytes([qb, wb]).decode("gb2312")
            except UnicodeDecodeError:
                continue
            total += 1
            if bin_gb(data, qb, wb) != ref_gb(ch):
                fails.append("GB %02X%02X '%s'" % (qb, wb, ch))
    print("GB2312 全量比对 : %d 个字形，%d 个不一致" % (total, len(fails)))

    # 4) 未定义位必须全 0
    undef_bad = 0
    for qb in GB_ROWS:
        for wb in range(0xA1, 0xFF):
            try:
                bytes([qb, wb]).decode("gb2312")
            except UnicodeDecodeError:
                if bin_gb(data, qb, wb) != bytes(32):
                    undef_bad += 1
    print("未定义位全 0    : %s (%d 个异常)" % ("是" if undef_bad == 0 else "否", undef_bad))
    if undef_bad:
        fails.append("未定义位非零")

    # 5) 随机抽样再确认
    random.seed(42)
    samp = random.sample(range(0x21, 0x7F), 30)
    bad = [c for c in samp if bin_ascii(data, c) != ref_ascii(c)]
    print("随机抽样 30 ASCII: %s" % ("全部一致" if not bad else "异常 %s" % bad))

    print("-" * 60)
    if fails:
        print("校验失败 %d 项：%s" % (len(fails), fails[:10]))
        return 1
    print("校验通过：bin 与源图逐字节一致")

    # 6) 渲染混排样张
    render_sample(data)
    return 0


def render_sample(data):
    """用 bin 里的字形渲染一句 ASCII+中文 混排，输出 font_sample.png。"""
    text = "GB2312 字库 FPGA 读取测试：中文显示正常，汉字 0.5mm 精度。"
    CW, CH = 16, 16
    pen_x, pen_y = 0, 0
    maxw = 640
    lines = [[]]
    # 先把文本按 16px 步进排版
    for ch in text:
        if pen_x + CW > maxw:
            lines.append([])
            pen_x = 0
            pen_y += CH
        lines[-1].append((ch, pen_x))
        pen_x += CW
    W, H = maxw, pen_y + CH
    img = Image.new("L", (W, H), 255)
    px = img.load()
    for li, line in enumerate(lines):
        y0 = li * CH
        for ch, x0 in line:
            cp = ord(ch)
            g = bin_ascii(data, cp) if cp < 0x80 else bin_gb(data, *ch.encode("gb2312"))
            for r in range(16):
                hi, lo = g[r * 2], g[r * 2 + 1]
                for i in range(8):
                    if hi & (1 << (7 - i)):
                        px[x0 + i, y0 + r] = 0
                    if lo & (1 << (7 - i)):
                        px[x0 + 8 + i, y0 + r] = 0
    out = img.resize((W * 2, H * 2), Image.NEAREST)
    p = os.path.join(os.path.dirname(BIN), "font_sample.png")
    out.save(p)
    print("混排样张已输出: %s" % p)


if __name__ == "__main__":
    sys.exit(main())
