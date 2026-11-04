#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ppm2png.py —— P6(PPM) → PNG，纯标准库（zlib + struct），不依赖 Pillow。

用法:
    python ppm2png.py out            # 把 out/*.ppm 全部转成 out/*.png
    python ppm2png.py out/a_nav.ppm  # 只转一个
"""
import os
import re
import sys
import zlib
import struct


def write_png(path, w, h, rgb):
    """rgb: bytes, 长度 w*h*3"""
    raw = bytearray()
    stride = w * 3
    for y in range(h):
        raw.append(0)                       # filter type 0 (None)
        raw += rgb[y * stride:(y + 1) * stride]

    def chunk(tag, data):
        c = struct.pack(">I", len(data)) + tag + data
        c += struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        return c

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", ihdr)
           + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)


def read_ppm(path):
    with open(path, "rb") as f:
        data = f.read()
    # 解析 P6\n<w> <h>\n<max>\n
    assert data[:2] == b"P6", "not P6: %s" % path
    m = re.match(rb"P6\s+(\d+)\s+(\d+)\s+(\d+)\s", data[:64])
    w, h, mx = int(m.group(1)), int(m.group(2)), int(m.group(3))
    assert mx == 255
    hdr_len = m.end()
    rgb = data[hdr_len:hdr_len + w * h * 3]
    if len(rgb) != w * h * 3:
        print("WARN %s: expect %d bytes, got %d (补 0)" % (path, w * h * 3, len(rgb)))
        rgb = rgb + bytes(w * h * 3 - len(rgb))
    return w, h, rgb


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return
    files = []
    for a in args:
        if os.path.isdir(a):
            files += [os.path.join(a, f) for f in sorted(os.listdir(a))
                      if f.lower().endswith(".ppm")]
        else:
            files.append(a)
    for p in files:
        w, h, rgb = read_ppm(p)
        out = os.path.splitext(p)[0] + ".png"
        write_png(out, w, h, rgb)
        print("%s -> %s (%dx%d)" % (p, out, w, h))


if __name__ == "__main__":
    main()
