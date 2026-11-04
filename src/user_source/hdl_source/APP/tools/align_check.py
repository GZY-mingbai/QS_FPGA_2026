#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""align_check.py —— 把"565 缩略图渲染帧"反投影回 640×480 源坐标，再和 HD 比

为什么不能直接逐像素相减：320×240 / 480×360 的帧和 640×480 的 HD 是**不同尺度**，
直接减等于拿黑边和缩放差当信号（第一版就是这么错的，SAD 全被黑框带跑）。

做法：
  1. 按盒子几何 (x0,y0,W,H) 把渲染帧最近邻**反投影**到 640×480（源坐标）；
  2. 在这个公共坐标里做逐行 SAD 求最佳横向位移、逐列求纵向位移；
  3. 输出叠加图：R=HD、G=反投影后的被测帧 ⇒ 对齐处呈黄色，偏了出红/绿边。

用法（工作目录 = APP/）：
    python tools/align_check.py out_hd.ppm out_thumb_w320.ppm:160,120,320,240 ...
"""
import sys, zlib, struct

BOXES = {
    'w320': (160, 120, 320, 240),
    'w480': ( 80,  60, 480, 360),
    'w640': (  0,   0, 640, 480),
}

def read_ppm(p):
    d = open(p, 'rb').read()
    assert d[:2] == b'P6', p
    i, vals = 2, []
    while len(vals) < 3:
        while d[i:i+1].isspace(): i += 1
        j = i
        while not d[j:j+1].isspace(): j += 1
        vals.append(int(d[i:j])); i = j
    i += 1
    w, h, _ = vals
    return w, h, d[i:i+w*h*3]

def write_png(p, w, h, rgb):
    raw = b''.join(b'\x00' + rgb[y*w*3:(y+1)*w*3] for y in range(h))
    def chunk(t, data):
        c = t + data
        return struct.pack('>I', len(data)) + c + struct.pack('>I', zlib.crc32(c) & 0xffffffff)
    open(p, 'wb').write(b'\x89PNG\r\n\x1a\n' +
                        chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)) +
                        chunk(b'IDAT', zlib.compress(raw, 9)) +
                        chunk(b'IEND', b''))

def lum(b, w, x, y):
    o = (y*w + x)*3
    return (b[o]*77 + b[o+1]*150 + b[o+2]*29) >> 8

def unproject(buf, w, h, box, W=640, H=480):
    """把渲染帧按盒子几何反投影回 640×480（最近邻），返回 3 字节/像素"""
    x0, y0, bw, bh = box
    out = bytearray(W*H*3)
    for sy in range(H):
        yy = y0 + int((sy+0.5)*bh/H)      # 目标像素中心
        yy = min(max(yy, 0), h-1)
        for sx in range(W):
            xx = x0 + int((sx+0.5)*bw/W)
            xx = min(max(xx, 0), w-1)
            o = (yy*w + xx)*3
            d = (sy*W + sx)*3
            out[d:d+3] = buf[o:o+3]
    return out

def best_shift(hd, cur, W, H, axis, rng=4, step=2):
    """逐行(axis=0)/逐列(axis=1) SAD 求最佳位移，返回 (中位数, 分布)"""
    ys = range(0, H, 1) if axis == 0 else range(0, W, step)
    res = []
    for k in ys:
        if axis == 0:
            base = sum(lum(hd, W, x, k) for x in range(0, W, 8))
        else:
            base = sum(lum(hd, W, k, y) for y in range(0, H, 8))
        if base < 8000:                       # 整行/整列过暗（黑边）跳过
            continue
        best, bs = 0, None
        for s in range(-rng, rng+1):
            sad = 0
            if axis == 0:
                for x in range(4, W-4, step):
                    sad += abs(lum(cur, W, x, k) - lum(hd, W, x-s, k))
            else:
                for y in range(4, H-4, step):
                    sad += abs(lum(cur, W, k, y) - lum(hd, W, k, y-s))
            if bs is None or sad < bs:
                bs, best = sad, s
        res.append(best)
    if not res:
        return 0, {}
    hist = {}
    for v in res: hist[v] = hist.get(v, 0) + 1
    res.sort()
    return res[len(res)//2], hist

def main():
    if len(sys.argv) < 3:
        print(__doc__); return
    w, h, hd = read_ppm(sys.argv[1])
    print(f"HD 参考 {sys.argv[1]}  {w}x{h}（源坐标 = 屏幕坐标）\n")
    print(f"{'帧':<26}{'横向(源px)':>12}{'纵向(源px)':>12}   分布")
    print("-"*78)
    ovs = []
    for spec in sys.argv[2:]:
        p = spec
        key = p.split('_')[-1].split('.')[0]
        box = BOXES.get(key, (0, 0, w, h))
        w2, h2, cur = read_ppm(p)
        cur_u = unproject(cur, w2, h2, box)
        mdx, hx = best_shift(hd, cur_u, w, h, 0)
        mdy, hy = best_shift(hd, cur_u, w, h, 1)
        print(f"{p:<26}{mdx:>+12d}{mdy:>+12d}   dx{sorted(hx.items())} dy{sorted(hy.items())}")
        ov = bytearray(w*h*3)
        for i in range(0, w*h*3, 3):
            ov[i]   = hd[i]            # R = HD
            ov[i+1] = cur_u[i+1]       # G = 被测
            ov[i+2] = 0
        out = p.replace('.ppm', '_ovl.png')
        write_png(out, w, h, bytes(ov))
        ovs.append(out)
    print("-"*78)
    for o in ovs:
        print("叠加图(R=HD G=被测):", o)
    print("\n判读：中位数是内容整体偏移（源像素）；分布若同时出现 ±1 的两种值，")
    print("      说明采样相位在抖（最近邻 2× 放大只能落在 2 屏幕像素的网格上）。")

if __name__ == '__main__':
    main()
