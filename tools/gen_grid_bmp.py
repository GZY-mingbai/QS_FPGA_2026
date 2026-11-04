#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
gen_grid_bmp.py —— 生成 640×480 / 24bit / 不压缩 / 自底向上的网格检测图

==============================================================================
【为什么要"生成"而不是随手画一张】
==============================================================================
bmp_read.v 的 header_match 是**严格**校验，六条全过才算合法文件：

    'B' 'M'  +  width==640  +  height==480  +  bit_count==24  +  compression==0

而且 height 只取低 16 位 ⇒ **负高度（top-down）会被判成 65504 而直接拒绝**，
所以必须写成标准自底向上。行宽 640×3 = 1920 字节，正好 4 对齐，不需要 padding。
文件总长固定 54 + 921600 = 921654，正好等于顶层的 IMG0_LENGTH / IMG1_LENGTH。
==============================================================================

【怎么用】
    python tools/gen_grid_bmp.py                       # 默认两张：红格 + 蓝格
    python tools/gen_grid_bmp.py --pitch 40            # 改格距（见下）
    python tools/gen_grid_bmp.py --line 4 --outdir D:/ # 指定线宽与输出目录

【关于格距的两个选择】
    纹理 640×480 在屏幕上被缩到 240×180（K = 65536×8/3 = 174763），缩小 8/3 倍。

    --pitch 16（默认，字面照做）
        屏幕上格距 = 16 × 3/8 = **6 px**，密。点采样会让 2px 线变成虚线，
        但当检测图用没问题：任何错位/错像素都会立刻破坏这个密网格的规律性。

    --pitch 40（推荐用于"看清楚"）
        640/40 = 16 格、480/40 = 12 格，屏幕格距 = 40 × 3/8 = **15 px 整**，
        采样点正好落在格子上，**没有摩尔纹**，最干净。
"""
import argparse
import os
import struct

W, H = 640, 480
BMP_HDR = 54


def build(pitch, line_w, bg, fg):
    """返回完整的 BMP 字节流（含 54 字节文件头）。

    bg/fg 是 (r, g, b)。行优先、自底向上写：文件第 0 行 = 图像最后一行。
    """
    # 先按图像行（y=0 在顶部）算出每一行的字节
    rows = []
    for y in range(H):
        on_row = (y % pitch) < line_w
        row = bytearray(W * 3)
        if on_row:
            # 整行都是线色，直接填
            row[:] = bytes((fg[2], fg[1], fg[0])) * W
        else:
            for x in range(W):
                if (x % pitch) < line_w:
                    row[x * 3:x * 3 + 3] = bytes((fg[2], fg[1], fg[0]))
                else:
                    row[x * 3:x * 3 + 3] = bytes((bg[2], bg[1], bg[0]))
        rows.append(bytes(row))

    pixels = b''.join(reversed(rows))          # 自底向上

    hdr = b'BM'
    hdr += struct.pack('<IHHI', BMP_HDR + len(pixels), 0, 0, BMP_HDR)
    hdr += struct.pack('<IiiHHIIiiII',
                       40,          # DIB 头大小
                       W, H,        # 宽、高（正数 = 自底向上）
                       1, 24,       # planes、每像素位数
                       0,           # compression = BI_RGB
                       len(pixels), # 图像字节数
                       2835, 2835,  # 像素/米（约 72 dpi，无所谓）
                       0, 0)        # 调色板数、重要色数
    assert len(hdr) == BMP_HDR, len(hdr)
    return hdr + pixels


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pitch', type=int, default=16, help='格距（像素），默认 16；40 可得到屏幕 15px 的干净网格')
    ap.add_argument('--line', type=int, default=2, help='线宽（像素），默认 2')
    ap.add_argument('--bg', default='C0C0C0', help='底色，十六进制 RGB，默认 C0C0C0 浅灰')
    ap.add_argument('--outdir', default='tools/out', help='输出目录，默认 tools/out')
    args = ap.parse_args()

    def rgb(s):
        s = s.lstrip('#')
        return (int(s[0:2], 16), int(s[2:4], 16), int(s[4:6], 16))

    bg = rgb(args.bg)
    os.makedirs(args.outdir, exist_ok=True)

    jobs = [
        ('grid_red_640x480.bmp',  (0xFF, 0x00, 0x00)),
        ('grid_blue_640x480.bmp', (0x00, 0x00, 0xFF)),
    ]
    for name, fg in jobs:
        data = build(args.pitch, args.line, bg, fg)
        path = os.path.join(args.outdir, name)
        with open(path, 'wb') as f:
            f.write(data)
        print('  %-28s %d 字节  格距=%d 线宽=%d 底色=%s 线色=#%02X%02X%02X'
              % (name, len(data), args.pitch, args.line, args.bg, fg[0], fg[1], fg[2]))

    exp = BMP_HDR + W * H * 3
    print('\n  期望字节数 = 54 + 640×480×3 = %d' % exp)
    print('  顶层 IMG0_LENGTH / IMG1_LENGTH 必须等于这个数')


if __name__ == '__main__':
    main()
