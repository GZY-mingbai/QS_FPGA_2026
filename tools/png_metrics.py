#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""png_metrics.py —— 用数值代替眼睛（纯标准库）

用法：
    python tools/png_metrics.py --check out            # 校验目录下所有 PNG（CRC + 解压长度）
    python tools/png_metrics.py --pix f.ppm 320 240     # 取一个像素
    python tools/png_metrics.py --bands f.ppm 112 0 640 480 60   # 文字行带（x0 y0 x1 y1 亮度阈值）
    python tools/png_metrics.py --bbox  f.ppm 208 220 368 260 60 # 墨迹 bbox
    python tools/png_metrics.py --diff  a.ppm b.ppm     # 两帧差异像素数与 bbox
    python tools/png_metrics.py --center f.ppm 208 220 368 260 60 # bbox 中心与区域中心

支持 PPM(P6) 与 PNG（8bit RGB/RGBA，非隔行）。
"""

import argparse
import glob
import os
import struct
import sys
import zlib


# ---------------------------------------------------------------------------
#  读图
# ---------------------------------------------------------------------------
def read_ppm(path):
    d = open(path, 'rb').read()
    if d[:2] != b'P6':
        raise SystemExit('%s 不是 P6 PPM' % path)
    # 解析头：P6 <w> <h> <max>\n
    i = 2
    vals = []
    while len(vals) < 3:
        while i < len(d) and d[i:i + 1].isspace():
            i += 1
        if d[i:i + 1] == b'#':
            while i < len(d) and d[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(d) and not d[j:j + 1].isspace():
            j += 1
        vals.append(int(d[i:j]))
        i = j
    i += 1
    w, h, mx = vals
    if mx != 255:
        raise SystemExit('只支持 8bit PPM')
    px = d[i:i + w * h * 3]
    if len(px) != w * h * 3:
        raise SystemExit('PPM 像素长度不对：%d != %d' % (len(px), w * h * 3))
    return w, h, px


def read_png(path):
    d = open(path, 'rb').read()
    assert d[:8] == b'\x89PNG\r\n\x1a\n', '不是 PNG'
    off = 8
    idat = b''
    w = h = bd = ct = None
    while off < len(d):
        ln = struct.unpack('>I', d[off:off + 4])[0]
        typ = d[off + 4:off + 8]
        body = d[off + 8:off + 8 + ln]
        crc = struct.unpack('>I', d[off + 8 + ln:off + 12 + ln])[0]
        assert zlib.crc32(typ + body) & 0xffffffff == crc, 'CRC 错：%s' % typ
        if typ == b'IHDR':
            w, h, bd, ct = struct.unpack('>IIBB', body[:10])
        elif typ == b'IDAT':
            idat += body
        elif typ == b'IEND':
            break
        off += 12 + ln
    ch = {2: 3, 6: 4, 0: 1}[ct]
    raw = zlib.decompress(idat)
    assert len(raw) == (w * ch + 1) * h, '解压长度不对'
    # 反滤波
    stride = w * ch
    out = bytearray(stride * h)
    prev = bytearray(stride)
    p = 0
    for y in range(h):
        f = raw[p]
        p += 1
        line = bytearray(raw[p:p + stride])
        p += stride
        if f == 1:
            for x in range(ch, stride):
                line[x] = (line[x] + line[x - ch]) & 0xFF
        elif f == 2:
            for x in range(stride):
                line[x] = (line[x] + prev[x]) & 0xFF
        elif f == 3:
            for x in range(stride):
                a = line[x - ch] if x >= ch else 0
                line[x] = (line[x] + ((a + prev[x]) >> 1)) & 0xFF
        elif f == 4:
            for x in range(stride):
                a = line[x - ch] if x >= ch else 0
                b = prev[x]
                c = prev[x - ch] if x >= ch else 0
                pa = abs(b - c)
                pb = abs(a - c)
                pc = abs(a + b - 2 * c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                line[x] = (line[x] + pr) & 0xFF
        out[y * stride:(y + 1) * stride] = line
        prev = line
    if ch == 4:
        rgb = bytearray(w * h * 3)
        for i in range(w * h):
            rgb[3 * i:3 * i + 3] = out[4 * i:4 * i + 3]
        out = rgb
    return w, h, bytes(out)


def read_img(path):
    if path.lower().endswith('.png'):
        return read_png(path)
    return read_ppm(path)


def get(px, w, x, y):
    i = (y * w + x) * 3
    return px[i], px[i + 1], px[i + 2]


def lum(c):
    return (c[0] * 299 + c[1] * 587 + c[2] * 114) // 1000


# ---------------------------------------------------------------------------
#  命令
# ---------------------------------------------------------------------------
def cmd_check(d):
    bad = []
    files = sorted(glob.glob(os.path.join(d, '*.png')))
    for p in files:
        try:
            read_png(p)
        except Exception as e:
            bad.append((p, str(e)))
    print('PNG 校验：%s（%d 张）' % ('全部通过' if not bad else '失败', len(files)))
    for p, e in bad:
        print('  !!', p, e)
    return 1 if bad else 0


def cmd_pix(args):
    w, h, px = read_img(args.file)
    c = get(px, w, args.x, args.y)
    print('%s (%d,%d) = RGB(%d,%d,%d) #%02X%02X%02X lum=%d'
          % (args.file, args.x, args.y, c[0], c[1], c[2], c[0], c[1], c[2], lum(c)))


def cmd_bbox(args):
    w, h, px = read_img(args.file)
    x0, y0, x1, y1 = args.rect
    xs, ys = [], []
    for y in range(max(0, y0), min(h, y1)):
        for x in range(max(0, x0), min(w, x1)):
            if lum(get(px, w, x, y)) > args.th:
                xs.append(x)
                ys.append(y)
    if not xs:
        print('  区域内没有墨迹（阈值 %d）' % args.th)
        return
    bx0, bx1, by0, by1 = min(xs), max(xs), min(ys), max(ys)
    print('  ink bbox x=%d..%d (w=%d)  y=%d..%d (h=%d)  cx=%.1f cy=%.1f  像素=%d'
          % (bx0, bx1, bx1 - bx0 + 1, by0, by1, by1 - by0 + 1,
             (bx0 + bx1) / 2.0, (by0 + by1) / 2.0, len(xs)))


def cmd_center(args):
    w, h, px = read_img(args.file)
    x0, y0, x1, y1 = args.rect
    xs = []
    for y in range(max(0, y0), min(h, y1)):
        for x in range(max(0, x0), min(w, x1)):
            if lum(get(px, w, x, y)) > args.th:
                xs.append(x)
    if not xs:
        print('  没有墨迹')
        return
    bx0, bx1 = min(xs), max(xs)
    want = (x0 + x1 - 1) / 2.0
    got = (bx0 + bx1) / 2.0
    print('  bbox x=%d..%d  center=%.1f  区域中心=%.1f  偏差=%+.1f px  %s'
          % (bx0, bx1, got, want, got - want,
             'OK' if abs(got - want) <= 2.0 else 'FAIL'))


def cmd_bands(args):
    w, h, px = read_img(args.file)
    x0, y0, x1, y1 = args.rect
    rows = []
    for y in range(max(0, y0), min(h, y1)):
        n = 0
        for x in range(max(0, x0), min(w, x1)):
            if lum(get(px, w, x, y)) > args.th:
                n += 1
        rows.append((y, n))
    bands = []
    cur = None
    for y, n in rows:
        if n > 0:
            if cur is None:
                cur = [y, y, n]
            else:
                cur[1] = y
                cur[2] += n
        else:
            if cur is not None:
                bands.append(cur)
                cur = None
    if cur is not None:
        bands.append(cur)
    print('  文字行带：%d 段' % len(bands))
    for b in bands:
        print('    y=%d..%d (h=%d) 墨迹=%d' % (b[0], b[1], b[1] - b[0] + 1, b[2]))


def cmd_diff(args):
    w1, h1, p1 = read_img(args.a)
    w2, h2, p2 = read_img(args.b)
    if (w1, h1) != (w2, h2):
        print('  尺寸不同：%dx%d vs %dx%d' % (w1, h1, w2, h2))
        return
    n = 0
    xs, ys = [], []
    for y in range(h1):
        for x in range(w1):
            i = (y * w1 + x) * 3
            if p1[i:i + 3] != p2[i:i + 3]:
                n += 1
                xs.append(x)
                ys.append(y)
    if n == 0:
        print('  两帧完全相同')
        return
    print('  差异像素=%d (%.3f%%)  bbox x=%d..%d y=%d..%d'
          % (n, 100.0 * n / (w1 * h1), min(xs), max(xs), min(ys), max(ys)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check')
    ap.add_argument('--pix', nargs=3, metavar=('FILE', 'X', 'Y'))
    ap.add_argument('--bbox', nargs=5, metavar=('FILE', 'X0', 'Y0', 'X1', 'Y1'))
    ap.add_argument('--center', nargs=5, metavar=('FILE', 'X0', 'Y0', 'X1', 'Y1'))
    ap.add_argument('--bands', nargs=5, metavar=('FILE', 'X0', 'Y0', 'X1', 'Y1'))
    ap.add_argument('--diff', nargs=2, metavar=('A', 'B'))
    ap.add_argument('--th', type=int, default=60)
    args = ap.parse_args()

    if args.check:
        return cmd_check(args.check)
    if args.pix:
        args.file, args.x, args.y = args.pix[0], int(args.pix[1]), int(args.pix[2])
        return cmd_pix(args)
    if args.bbox:
        args.file = args.bbox[0]
        args.rect = tuple(int(v) for v in args.bbox[1:])
        return cmd_bbox(args)
    if args.center:
        args.file = args.center[0]
        args.rect = tuple(int(v) for v in args.center[1:])
        return cmd_center(args)
    if args.bands:
        args.file = args.bands[0]
        args.rect = tuple(int(v) for v in args.bands[1:])
        return cmd_bands(args)
    if args.diff:
        args.a, args.b = args.diff
        return cmd_diff(args)
    ap.print_help()
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
