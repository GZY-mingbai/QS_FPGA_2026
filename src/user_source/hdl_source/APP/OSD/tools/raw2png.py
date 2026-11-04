#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""raw2png.py —— 裸 RGB(640*480*3) → PNG。用法: python raw2png.py out  [W H]"""
import os, sys, zlib, struct

def write_png(path, w, h, rgb):
    raw = bytearray(); stride = w*3
    for y in range(h):
        raw.append(0); raw += rgb[y*stride:(y+1)*stride]
    def ck(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t+d) & 0xFFFFFFFF)
    open(path,'wb').write(b"\x89PNG\r\n\x1a\n" + ck(b"IHDR", struct.pack(">IIBBBBB", w,h,8,2,0,0,0))
                          + ck(b"IDAT", zlib.compress(bytes(raw),6)) + ck(b"IEND", b""))

W = int(sys.argv[2]) if len(sys.argv)>2 else 640
H = int(sys.argv[3]) if len(sys.argv)>3 else 480
d = sys.argv[1]
files = [os.path.join(d,f) for f in sorted(os.listdir(d)) if f.endswith('.raw')]
for p in files:
    b = open(p,'rb').read()
    need = W*H*3
    if len(b) < need:
        b = b + bytes(need-len(b))
    out = os.path.splitext(p)[0] + '.png'
    write_png(out, W, H, b[:need])
    print('%s -> %s' % (p, out))
