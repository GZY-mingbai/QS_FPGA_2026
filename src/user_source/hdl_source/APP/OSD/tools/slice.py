#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
slice.py —— 沿一条直线逐点采样 PPM，用于看"边界处的颜色台阶"。

用法:
    python slice.py out/v_nav.ppm --h y=200 --from 88 --to 112 [--step 2]
    python slice.py out/v_nav.ppm --v x=400 --from 0  --to 30  [--step 2]
"""
import sys


def read_ppm(path):
    d = open(path, 'rb').read()
    assert d[:2] == b'P6'
    pos = 2
    fields = []
    while len(fields) < 3:
        while d[pos:pos + 1].isspace():
            pos += 1
        if d[pos:pos + 1] == b'#':
            while d[pos:pos + 1] != b'\n':
                pos += 1
            continue
        s = pos
        while not d[pos:pos + 1].isspace():
            pos += 1
        fields.append(int(d[s:pos]))
    pos += 1
    w, h, _ = fields
    return w, h, d[pos:pos + w * h * 3]


def bg_of(x, y):
    r = x & 0xFF
    g = y & 0xFF
    b = (x & 0xFF) ^ (y & 0xFF)
    if ((x >> 5) & 1) ^ ((y >> 5) & 1):
        r >>= 1
        g >>= 1
        b >>= 1
    return r, g, b


def main():
    path = sys.argv[1]
    args = sys.argv[2:]
    step = 1
    if '--step' in args:
        i = args.index('--step')
        step = int(args[i + 1])
        args.pop(i)
        args.pop(i)
    horizon = True
    if '--h' in args:
        i = args.index('--h')
        y = int(args[i + 1].split('=')[1])
        args.pop(i)
        args.pop(i)
    else:
        i = args.index('--v')
        x = int(args[i + 1].split('=')[1])
        args.pop(i)
        args.pop(i)
        horizon = False
    i = args.index('--from')
    a0 = int(args[i + 1])
    args.pop(i)
    args.pop(i)
    i = args.index('--to')
    a1 = int(args[i + 1])
    # 图像可能只是原图的一块（shot_rect 的窗口），用 --ox/--oy 给窗口左上角的原图坐标
    ox = oy = 0
    if '--ox' in args:
        i = args.index('--ox')
        ox = int(args[i + 1])
        args.pop(i)
        args.pop(i)
    if '--oy' in args:
        i = args.index('--oy')
        oy = int(args[i + 1])
        args.pop(i)
        args.pop(i)

    w, h, rgb = read_ppm(path)

    def px(xx, yy):
        ix, iy = xx // step, yy // step
        if ix < 0 or iy < 0 or ix >= w or iy >= h:
            return None
        o = (iy * w + ix) * 3
        return rgb[o], rgb[o + 1], rgb[o + 2]

    print(f"{path} {w}x{h} step={step} " + (f"h-slice y={y}" if horizon else f"v-slice x={x}"))
    prev = None
    for a in range(a0, a1 + 1, step):
        xx, yy = (a, y) if horizon else (x, a)
        c = px(xx - ox, yy - oy)
        if c is None:
            continue
        bg = bg_of(xx, yy)
        tag = ""
        if prev is not None and max(abs(c[k] - prev[k]) for k in range(3)) > 3:
            tag = "  <-- step"
        print(f"  {'x' if horizon else 'y'}={xx if horizon else yy:4d}  rgb{c}  bg{bg}{tag}")
        prev = c


if __name__ == '__main__':
    main()
