#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
zoom.py —— 把 PNG 用最近邻放大 N 倍，便于逐像素看细节（纯标准库）。

用法:  python zoom.py in.png out.png [倍率]
"""
import sys
import zlib
import struct


def read_png(path):
    data = open(path, "rb").read()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos = 8
    w = h = bitd = ctype = None
    idat = b""
    while pos < len(data):
        ln = struct.unpack(">I", data[pos:pos + 4])[0]
        tag = data[pos + 4:pos + 8]
        body = data[pos + 8:pos + 8 + ln]
        pos += 12 + ln
        if tag == b"IHDR":
            w, h, bitd, ctype = struct.unpack(">IIBB", body[:10])
        elif tag == b"IDAT":
            idat += body
        elif tag == b"IEND":
            break
    raw = zlib.decompress(idat)
    assert bitd == 8 and ctype in (2, 6), "only 8-bit RGB/RGBA"
    nch = 3 if ctype == 2 else 4
    stride = w * nch
    out = bytearray(w * h * 3)
    prev = bytearray(stride)
    p = 0
    for y in range(h):
        f = raw[p]; p += 1
        line = bytearray(raw[p:p + stride]); p += stride
        if f == 1:
            for i in range(nch, stride):
                line[i] = (line[i] + line[i - nch]) & 0xFF
        elif f == 2:
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif f == 3:
            for i in range(stride):
                a = line[i - nch] if i >= nch else 0
                line[i] = (line[i] + ((a + prev[i]) >> 1)) & 0xFF
        elif f == 4:
            for i in range(stride):
                a = line[i - nch] if i >= nch else 0
                b = prev[i]
                c = prev[i - nch] if i >= nch else 0
                pa = abs(b - c); pb = abs(a - c); pc = abs(a + b - 2 * c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                line[i] = (line[i] + pr) & 0xFF
        prev = line
        for x in range(w):
            out[(y * w + x) * 3 + 0] = line[x * nch + 0]
            out[(y * w + x) * 3 + 1] = line[x * nch + 1]
            out[(y * w + x) * 3 + 2] = line[x * nch + 2]
    return w, h, bytes(out)


def write_png(path, w, h, rgb):
    raw = bytearray()
    stride = w * 3
    for y in range(h):
        raw.append(0)
        raw += rgb[y * stride:(y + 1) * stride]

    def chunk(tag, d):
        return (struct.pack(">I", len(d)) + tag + d
                + struct.pack(">I", zlib.crc32(tag + d) & 0xFFFFFFFF))

    png = (b"\x89PNG\r\n\x1a\n"
           + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
           + chunk(b"IEND", b""))
    open(path, "wb").write(png)


def main():
    a = sys.argv[1:]
    if len(a) < 2:
        print(__doc__); return
    k = int(a[2]) if len(a) > 2 else 8
    w, h, rgb = read_png(a[0])
    ow, oh = w * k, h * k
    out = bytearray(ow * oh * 3)
    for y in range(oh):
        sy = y // k
        for x in range(ow):
            sx = x // k
            si = (sy * w + sx) * 3
            di = (y * ow + x) * 3
            out[di:di + 3] = rgb[si:si + 3]
    write_png(a[1], ow, oh, bytes(out))
    print("%s (%dx%d) -> %s (%dx%d) x%d" % (a[0], w, h, a[1], ow, oh, k))


if __name__ == "__main__":
    main()
