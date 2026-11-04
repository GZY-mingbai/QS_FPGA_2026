#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_text_init.py —— 生成文字层要用的仿真/初始化数据

用途三合一：
  1) 把一段文本（默认「测试 TEST」）转成字符串 RAM 的初始化值（16 位字符码）
     —— App.v 里那张 string RAM 的初值就用它；
  2) 打印每个字符的 Flash 地址 + 点阵（ASCII art），便于肉眼看对不对
     —— 与 gb_addr.v / tb_gb_addr.v 用的是同一套公式；
  3) 把整个字库导成 $readmemh 能吃的 hex 文件，供 w25q64_model 预置。

用法：
    python tools/gen_text_init.py                      # 默认「测试 TEST」
    python tools/gen_text_init.py --text "音量 TEST"
    python tools/gen_text_init.py --no-font-hex        # 不导整库（省时间）

产物（都在 tools/out/）：
    osd_str0.hex      每行一个 16 位字符码，给 string RAM $readmemh
    font256k.hex      247744 行、每行一个字节，给 flash 模型 $readmemh
"""

import argparse
import hashlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

FONT_BIN     = os.path.join(ROOT, "Font", "font_gb2312.bin")
OUT_DIR      = os.path.join(HERE, "out")

# ---- 与 Font/字库格式说明.md、gb_addr.v 完全一致的常量 ----
ASCII_BASE   = 0x000000
GB_BASE      = 0x001000
GLYPH_BYTES  = 32
GB_COLS      = 94
GB_ROWS      = list(range(0xA1, 0xAA)) + list(range(0xB0, 0xF8))   # 81 个有内容的区
FONT_SIZE    = ASCII_BASE + len(range(128)) * 0 + 4096 + len(GB_ROWS) * GB_COLS * GLYPH_BYTES
FLASH_SIZE   = 256 * 1024        # 仿真模型给 256KB


# ==============================================================
#  寻址（与 gb_addr.v 同一套公式）
# ==============================================================
def glyph_addr(code: int) -> int | None:
    """16 位字符码 → Flash 偏移；非法/空区返回 None（RTL 那边会兜底成 0）。"""
    qb, wb = (code >> 8) & 0xFF, code & 0xFF
    if qb == 0x00:                      # ASCII 路径
        if wb < 0x80:
            return ASCII_BASE + wb * GLYPH_BYTES
        return None
    if 0xA1 <= qb <= 0xA9:              # 符号区 → row 0..8
        row = qb - 0xA1
    elif 0xB0 <= qb <= 0xF7:            # 汉字区 → row 9..80
        row = qb - 0xB0 + 9
    else:                               # 空区 / 非法
        return None
    if not (0xA1 <= wb <= 0xFE):
        return None
    return GB_BASE + (row * GB_COLS + (wb - 0xA1)) * GLYPH_BYTES


def text_to_codes(text: str):
    """一段文本 → [(字符, 16位字符码, Flash偏移 or None)]，一个字 = 一个码。"""
    out = []
    for ch in text:
        try:
            enc = ch.encode("gb2312")
        except UnicodeEncodeError:
            out.append((ch, None, None, "无法用 GB2312 编码"))
            continue
        if len(enc) == 1:
            code = enc[0]                       # ASCII：高字节 0
        else:
            code = (enc[0] << 8) | enc[1]       # GB2312：{qb, wb}
        addr = glyph_addr(code)
        note = "" if addr is not None else "字库无此字形（RTL 会渲染成空白）"
        out.append((ch, code, addr, note))
    return out


def show_bitmap(buf: bytes, off: int, indent: str = "      "):
    """把 32 字节字形画成 16 行 ASCII art。"""
    lines = []
    for r in range(16):
        left, right = buf[off + 2 * r], buf[off + 2 * r + 1]
        row16 = (left << 8) | right
        s = "".join("#" if (row16 >> (15 - c)) & 1 else "." for c in range(16))
        lines.append(indent + s)
    return lines


# ==============================================================
#  主流程
# ==============================================================
def main():
    ap = argparse.ArgumentParser(description="生成文字层的初始化数据")
    ap.add_argument("--text", default="测试 TEST", help="要处理的文本（默认「测试 TEST」）")
    ap.add_argument("--font", default=FONT_BIN, help="字库 bin 路径")
    ap.add_argument("--out", default=OUT_DIR, help="产物输出目录")
    ap.add_argument("--show-bitmap", action="store_true", default=True,
                    help="打印每个字的点阵（默认开）")
    ap.add_argument("--font-hex", dest="font_hex", action="store_true", default=True,
                    help="导出整库 hex（默认开）")
    ap.add_argument("--no-font-hex", dest="font_hex", action="store_false",
                    help="不导出整库 hex")
    args = ap.parse_args()

    if not os.path.isfile(args.font):
        print("找不到字库: %s" % args.font, file=sys.stderr)
        return 1

    buf = open(args.font, "rb").read()
    os.makedirs(args.out, exist_ok=True)

    print("=" * 64)
    print("  字库: %s" % args.font)
    print("  大小: %d 字节 (0x%X)   sha256[:16] = %s"
          % (len(buf), len(buf), hashlib.sha256(buf).hexdigest()[:16]))
    print("  预期: 247744 字节 (0x3C7C0)   sha256[:16] = 1cae592f8c1bf615")
    if len(buf) != 247744:
        print("  !! 字库大小与文档不符，后续地址可能全错 !!")
    print("=" * 64)

    items = text_to_codes(args.text)
    print("  文本: %r   共 %d 个字符" % (args.text, len(items)))
    print("-" * 64)
    print("  %-4s %-10s %-10s %-10s %s" % ("序号", "字符", "字符码", "Flash偏移", "备注"))
    print("-" * 64)
    for i, (ch, code, addr, note) in enumerate(items):
        c_s = "0x%04X" % code if code is not None else "  --  "
        a_s = "0x%06X" % addr if addr is not None else "  --  "
        print("  %-4d %-10s %-10s %-10s %s" % (i, ch, c_s, a_s, note))
    print("-" * 64)
    print("  提示：RTL 里非法/空区返回 NULL_ADDR = 0x000000（内容全 0 → 空白）")
    print("-" * 64)

    if args.show_bitmap:
        for i, (ch, code, addr, note) in enumerate(items):
            if addr is None:
                continue
            print("  [%d] %r  0x%06X" % (i, ch, addr))
            for ln in show_bitmap(buf, addr):
                print(ln)
        print("-" * 64)

    # ---- 产物 1：string RAM 初值 ----
    p1 = os.path.join(args.out, "osd_str0.hex")
    with open(p1, "w") as f:
        for i, (ch, code, addr, note) in enumerate(items):
            f.write("%04x\n" % (code if code is not None else 0x0000))
    print("  已写出 %s  (%d 行，每行一个 16 位字符码)" % (p1, len(items)))

    # ---- 产物 2：整库 hex（给 flash 模型 $readmemh） ----
    if args.font_hex:
        p2 = os.path.join(args.out, "font256k.hex")
        with open(p2, "w") as f:
            for b in buf:
                f.write("%02x\n" % b)
        print("  已写出 %s  (%d 行；模型剩余区域默认 0xFF)" % (p2, len(buf)))

    print("=" * 64)
    print("  完成")
    print("=" * 64)
    return 0


if __name__ == "__main__":
    sys.exit(main())
