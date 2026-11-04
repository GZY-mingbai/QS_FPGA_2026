#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
crop.py —— 从 P6 PPM 裁一块并放大（纯标准库）。
用法: python crop.py in.ppm out.png x y w h [倍率]
"""
import sys, zlib, struct

def read_ppm(path):
    d = open(path, 'rb').read()
    assert d[:2] == b'P6'
    hdr = d[:64].split(b'\n')
    w, h = int(hdr[1].split()[0]), int(hdr[1].split()[1])
    off = len(b'\n'.join(hdr[:3])) + 1
    return w, h, d[off:off + w * h * 3]

def write_png(path, w, h, rgb):
    raw = bytearray(); stride = w * 3
    for y in range(h):
        raw.append(0); raw += rgb[y * stride:(y + 1) * stride]
    def ck(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    open(path, 'wb').write(b"\x89PNG\r\n\x1a\n" + ck(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                           + ck(b"IDAT", zlib.compress(bytes(raw), 6)) + ck(b"IEND", b""))

a = sys.argv[1:]
W, H, px = read_ppm(a[0])
assert len(px) == W * H * 3, "PPM payload %d != %d" % (len(px), W * H * 3)
x0, y0, cw, ch = int(a[2]), int(a[3]), int(a[4]), int(a[5])
k = int(a[6]) if len(a) > 6 else 6

# ★ 裁剪矩形必须夹进画面内：越界时 sx/sy 会落到图外，切片变短/为空，
#   而 IHDR 仍然按 ow×oh 声明 ⇒ 写出来的 PNG 头尾不一致，
#   读图工具会直接报 "unsupported image"（就是之前三次 400 的原因）。
x0 = max(0, min(x0, W - 1)); y0 = max(0, min(y0, H - 1))
cw = max(1, min(cw, W - x0)); ch = max(1, min(ch, H - y0))

ow, oh = cw * k, ch * k
out = bytearray(ow * oh * 3)
for y in range(oh):
    sy = y0 + y // k
    row = sy * W
    for x in range(ow):
        si = (row + x0 + x // k) * 3
        out[(y * ow + x) * 3:(y * ow + x) * 3 + 3] = px[si:si + 3]
assert len(out) == ow * oh * 3
write_png(a[1], ow, oh, bytes(out))
print("%s crop(%d,%d,%d,%d) -> %s x%d  [%dx%d]" % (a[0], x0, y0, cw, ch, a[1], k, ow, oh))
