#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
probe.py —— 在 PPM 里按"原图坐标"采样并打印 RGB，同时给出该点底图的应有值。

用法:
    python probe.py out/v_nav.ppm 50,200 97,200 99,200
    python probe.py out/v_nav.ppm --step 2 50,200      # 图是 step2 半采样
    python probe.py out/v_nav.ppm --rect x0,y0,w,h --step 2   # 打一块区域的所有不同色
"""
import sys


def read_ppm(path):
    d = open(path, 'rb').read()
    assert d[:2] == b'P6', "not P6"
    # 解析三行头
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
    """与 tb_osd_render.v 的 bg_of 完全一致。"""
    r = x & 0xFF
    g = y & 0xFF
    b = (x & 0xFF) ^ (y & 0xFF)
    if ((x >> 5) & 1) ^ ((y >> 5) & 1):
        r >>= 1
        g >>= 1
        b >>= 1
    return r, g, b


def a9_of(level):
    """osd_gfx: a8 = level==8 ? 255 : level<<5；再 (a8*255)>>8（255 取 256）"""
    a8 = 255 if level == 8 else (level << 5)
    if a8 == 255:
        return 256
    return (a8 * 255) >> 8


def blend(fg, a9, bg):
    return tuple(((f * a9 + b * (256 - a9)) >> 8) for f, b in zip(fg, bg))


STYLE = {
    1:  (6, (0x0E, 0x0E, 0x0E)),   # 左栏/页面板 75%
    2:  (4, (0x00, 0x00, 0x00)),   # 全屏遮罩 50%
    5:  (8, (0x5A, 0x5A, 0x5A)),   # 图标未选
    6:  (8, (0xFF, 0xFF, 0xFF)),   # 图标选中/游标
    9:  (4, (0x40, 0x40, 0x40)),   # 行高亮 50%
    10: (8, (0x3A, 0x3A, 0x3A)),   # 分隔线/标题线/弹层板
    11: (6, (0x20, 0x20, 0x20)),   # 控件底 75%
}


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return
    path = args[0]
    step = 1
    if '--step' in args:
        i = args.index('--step')
        step = int(args[i + 1])
        del args[i:i + 2]
    args.pop(0)

    w, h, rgb = read_ppm(path)
    print(f"{path}: {w}x{h} step={step}")

    def px(x, y):
        ix, iy = x // step, y // step
        if ix < 0 or iy < 0 or ix >= w or iy >= h:
            return None
        o = (iy * w + ix) * 3
        return rgb[o], rgb[o + 1], rgb[o + 2]

    if '--rect' in args:
        i = args.index('--rect')
        x0, y0, rw, rh = [int(v) for v in args[i + 1].split(',')]
        counts = {}
        for yy in range(y0, y0 + rh, step):
            for xx in range(x0, x0 + rw, step):
                c = px(xx, yy)
                if c is not None:
                    counts[c] = counts.get(c, 0) + 1
        for c, n in sorted(counts.items(), key=lambda kv: -kv[1]):
            print(f"  rgb{c}  x{n}")
        return

    for a in args:
        x, y = [int(v) for v in a.split(',')]
        c = px(x, y)
        bg = bg_of(x, y)
        print(f"({x:3d},{y:3d}) actual={c}  bg={bg}")
        for lv, (l, fgc) in STYLE.items():
            exp = blend(fgc, a9_of(l), bg)
            mark = "  <== MATCH" if c and max(abs(c[k] - exp[k]) for k in range(3)) <= 2 else ""
            print(f"           if style{lv:2d} (lv{l} {fgc}) -> {exp}{mark}")


if __name__ == '__main__':
    main()
