#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""align_sweep.py —— fx3 放大过程中"内容相位"随宽度的变化（亚像素）

对每个宽度 w 的渲染帧：
  1. 按盒子几何反投影回 640×480（源坐标）；
  2. 与 HD 做整数位移 SAD，取最小点 + 相邻两点做**抛物线插值** ⇒ 亚像素位移；
  3. 只看有对比度的行（黑边/纯色跳过）。

输出：宽度 → dx(亚像素) / dy，以及"逐行位移的标准差"（抖动幅度）。

用法（工作目录 = APP/）： python tools/align_sweep.py
"""
import glob, re, statistics, zlib, struct, sys

def read_ppm(p):
    d = open(p,'rb').read(); assert d[:2]==b'P6', p
    i, vals = 2, []
    while len(vals) < 3:
        while d[i:i+1].isspace(): i += 1
        j = i
        while not d[j:j+1].isspace(): j += 1
        vals.append(int(d[i:j])); i = j
    i += 1
    w,h,_ = vals
    return w,h,d[i:i+w*h*3]

def lum(b,w,x,y):
    o=(y*w+x)*3
    return (b[o]*77+b[o+1]*150+b[o+2]*29)>>8

def unproject(buf,w,h,box,W=640,H=480):
    x0,y0,bw,bh = box
    out = bytearray(W*H*3)
    for sy in range(H):
        # ★ 取目标像素**中心**（+0.5），否则不同宽度会各带一个 -0.5*bw/W 的偏置
        yy = min(max(int(y0+(sy+0.5)*bh/H),0),h-1)
        row = yy*w
        for sx in range(W):
            xx = min(max(int(x0+(sx+0.5)*bw/W),0),w-1)
            o=(row+xx)*3; d=(sy*W+sx)*3
            out[d:d+3]=buf[o:o+3]
    return out

def sad(hd,cur,W,H,s,axis):
    t=0
    if axis==0:
        for y in range(4,H-4,2):
            for x in range(6,W-6,2):
                t += abs(lum(cur,W,x,y)-lum(hd,W,x-s,y))
    else:
        for y in range(6,H-6,2):
            for x in range(4,W-4,2):
                t += abs(lum(cur,W,x,y)-lum(hd,W,x,y-s))
    return t

def subpix(hd,cur,W,H,axis,rng=4):
    pts=[(s,sad(hd,cur,W,H,s,axis)) for s in range(-rng,rng+1)]
    s0,v0 = min(pts,key=lambda t:t[1])
    d = dict(pts)
    if (s0-1) in d and (s0+1) in d:
        a,b,c = d[s0-1],v0,d[s0+1]
        den = a-2*b+c
        off = 0.5*(a-c)/den if den else 0.0
        return s0+max(-0.5,min(0.5,off)), v0
    return float(s0), v0

def per_row_spread(hd,cur,W,H):
    vals=[]
    for y in range(4,H-4,4):
        base=sum(lum(hd,W,x,y) for x in range(0,W,8))
        if base < 8000: continue
        best,bs=0,None
        for s in range(-3,4):
            t=sum(abs(lum(cur,W,x,y)-lum(hd,W,x-s,y)) for x in range(6,W-6,4))
            if bs is None or t<bs: bs,best=t,s
        vals.append(best)
    return (statistics.pstdev(vals) if len(vals)>2 else 0.0), len(vals)

hdw,hdh,hd = read_ppm('out_hd.ppm')
print(f"{'W':>5}{'kx':>8}{'dx(px)':>9}{'dy(px)':>9}{'行抖动σ':>9}   备注")
print("-"*60)
rows=[]
for p in sorted(glob.glob('out_thumb_w*.ppm')):
    W = int(re.search(r'w(\d+)\.ppm',p).group(1))
    H = W*3//4
    box = ((640-W)//2, (480-H)//2, W, H)
    w,h,cur = read_ppm(p)
    cu = unproject(cur,w,h,box)
    dx,_ = subpix(hd,cu,640,480,0)
    dy,_ = subpix(hd,cu,640,480,1)
    sd,nr = per_row_spread(hd,cu,640,480)
    kx = 10485760//W
    rows.append((W,dx,dy,sd))
    print(f"{W:>5}{kx:>8}{dx:>+9.2f}{dy:>+9.2f}{sd:>9.2f}   盒={box}")
print("-"*60)
if rows:
    xs=[r[1] for r in rows]
    print(f"横向相位范围: {min(xs):+.2f} .. {max(xs):+.2f} px  "
          f"（峰峰值 {max(xs)-min(xs):.2f} px，源坐标）")
    print(f"换算到屏幕: 在 W=640 时 1 源px = 1 屏幕px；W=320 时 = 0.5 屏幕px")
