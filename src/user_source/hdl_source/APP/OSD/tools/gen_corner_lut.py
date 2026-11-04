#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_corner_lut.py —— 生成 osd_widget.v 里的圆角判定表 CORN_LUT

【为什么要有这张表】
    phy 报告：圆角算术把 `rd_geo(BRAM 输出) → pix_reg` 拉成
        20 级逻辑 / 34.4ns  (Fmax 28.7MHz，要求 50MHz)
    而圆角的三个判据只取决于 (rq, dx, dy)，三者都是 ≤ 8 的小整数
        ⇒ 完全可以查表，把 8 个加法器 + 3 个 4×4 乘法塌成 1 次查表。

【判据（与 osd_widget.v 原实现逐位等价）】
    rq  = min(wrad, 8)
    dxm = min(lx, w-1-lx)        到左右最近边的距离
    dym = min(ly, h-1-ly)
    inc = (rq != 0) && (dxm < rq) && (dym < rq)
    cut  = inc && (r-dx)² + (r-dy)²           > r²
    arc  = inc && !cut && (r-dx+1)² + (r-dy+1)² > r²
    arc2 = inc && !cut && (r-dx+2)² + (r-dy+2)² > r²

【表的组织（2.0：1024 bit，是 1.0 的 1/4）】
    1.0 用 index={rq[3:0],dx,dy} × 3 位 = 4096 bit，问题有两个：
      · rq 只有 1..8 共 8 个有效值，用 4 位寻址把表**浪费掉一半**；
      · (cut, arc, arc2) 三位里绝不会有两位同时为 1（见下面的单调性），
        存 3 位等于把 "10 / 01 / 00" 这些非法组合也当数据存。

    ⇒ 2.0：
      · 半径用 **rq-1**（3 位）寻址  ⇒ 512 项
      · 三项折成 **2 位带序** band ∈ {0=none, 1=arc2, 2=arc, 3=cut}
      ⇒ index = {rq-1, dx[2:0], dy[2:0]} = 9 位 → 512 项 × 2 位 = **1024 bit**

    **合法性（折叠的前提，本文件 check() 穷举验证）**
      三条判据的阈值**同为 r²**，只是 LHS 逐级变大：
        d2 = uu²+vv²  <  (uu+1)²+(vv+1)²  <  (uu+2)²+(vv+2)²     (inc 区域 uu,vv ≥ 1)
      LHS 越大越容易越过阈值 ⇒ 作为集合是 **cut ⊆ arc ⊆ arc2**。
      （★ 方向别搞反：是"arc2 比 arc 更容易成立"，不是反过来。
        踩过：误以为 arc ⇒ arc2 的补集关系，`band==1` 解 arc2 漏掉了 band==2。）

      RTL 里 arc 和 arc2 **都**被 !cut 门控，于是 inc 区域内只剩 4 个合法状态：
        band 0 = (0,0,0)   none
        band 1 = (0,0,1)   arc2 only      ← 弧内侧那第二圈
        band 2 = (0,1,1)   arc 与 arc2 同时为真（arc 蕴含 arc2）
        band 3 = (1,0,0)   cut
      ⇒ 2 位无损。解码：cut = &band；arc = band==2；arc2 = !cut && band!=0。

    **inc 不进表** —— 表里只存纯算术结果。inc 必须用**未 clamp** 的 dxm/dym
      判，clamp 到 7 会把 dym=8 当成 dy=7（实测多切一圈角）。
      inc 在表外与两个与门合成，只花 2 个比较器。

【面积账（Anlogic LUT5 = 32 存储位 + 5 输入函数）】
      表本身 1024 bit  ≈ 32 LUT5
      9→2 位变址选择    ≈ 10 LUT5
      inc 的两个比较器  ≈  4 LUT5
      ----------------------------------
      合计 ≈ 46 LUT5
    换掉的是：2 个 clamp(≈12) + 2 个减法(≈8) + 2 个 4×4 乘法(≈32)
             + 加法(≈16) + 比较(≈8) ≈ 76 LUT5，外加 14 级组合深度。
    ⇒ 面积净省约 30 LUT5，深度 20 级 → 约 5 级。

用法：
    python tools/gen_corner_lut.py            # 自检 + 打印常量
    python tools/gen_corner_lut.py --check    # 只做等价自检
"""
import sys

RMAX = 8

# band 编码
B_NONE, B_ARC2, B_ARC, B_CUT = 0, 1, 2, 3


def arith(rad, w, h, lx, ly):
    """原 RTL 的算术实现（规格模型 —— 不许拿 RTL 的新表达式回抄）"""
    dxm = min(lx, w - 1 - lx)
    dym = min(ly, h - 1 - ly)
    rq = min(rad, RMAX)
    dxc = min(dxm, 15)
    dyc = min(dym, 15)
    uu, vv = rq - dxc, rq - dyc
    d2 = uu * uu + vv * vv
    r2 = rq * rq
    inc = (rq != 0) and (dxm < rq) and (dym < rq)
    cut = inc and (d2 > r2)
    arc = inc and (not cut) and (d2 + 2 * uu + 2 * vv + 2 > r2)
    arc2 = inc and (not cut) and (d2 + 4 * uu + 4 * vv + 8 > r2)
    return cut, arc, arc2


def band_of(rq, dx, dy):
    """纯算术 → 带（不含 inc）"""
    r2 = rq * rq
    uu, vv = rq - dx, rq - dy
    d2 = uu * uu + vv * vv
    if d2 > r2:
        return B_CUT
    if d2 + 2 * uu + 2 * vv + 2 > r2:
        return B_ARC
    if d2 + 4 * uu + 4 * vv + 8 > r2:
        return B_ARC2
    return B_NONE


def build_table():
    """512 项 × 2 位，index = {(rq-1)[2:0], dx[2:0], dy[2:0]}"""
    t = {}
    for r3 in range(8):          # rq = 1..8
        for dx in range(8):
            for dy in range(8):
                t[(r3, dx, dy)] = band_of(r3 + 1, dx, dy)
    return t


def lut_lookup(t, rad, w, h, lx, ly):
    """RTL 的取表方式（逐位复刻）"""
    dxm = min(lx, w - 1 - lx)
    dym = min(ly, h - 1 - ly)
    rq = min(rad, RMAX)
    inc = (rq != 0) and (dxm < rq) and (dym < rq)
    r3 = ((rq - 1) & 7) if rq else 7          # rq=0 时地址无意义（inc 已为假）
    b = t[(r3, dxm if dxm < 8 else 7, dym if dym < 8 else 7)]
    cut = inc and b == B_CUT
    arc = inc and b == B_ARC
    arc2 = inc and (not cut) and b != B_NONE   # ★ band 1 和 band 2 都表示 arc2=1
    return (cut, arc, arc2)


def check():
    t = build_table()
    bad = n = 0
    for rad in range(16):
        for w in range(2, 21):
            for h in range(2, 21):
                for lx in range(w):
                    for ly in range(h):
                        n += 1
                        if arith(rad, w, h, lx, ly) != lut_lookup(t, rad, w, h, lx, ly):
                            bad += 1
                            if bad <= 5:
                                print("  MISMATCH rad=%d w=%d h=%d lx=%d ly=%d" % (rad, w, h, lx, ly))
    print("等价自检：%d 组，不匹配 %d 组" % (n, bad))

    # 折叠前提单独再验一遍：inc 区域内门控后的三元组必须只有 4 种取值
    legal = {(False, False, False), (False, False, True),
             (False, True, True), (True, False, False)}
    states = set()
    for rq in range(1, RMAX + 1):
        for dx in range(rq):
            for dy in range(rq):
                uu, vv = rq - dx, rq - dy
                d2, r2 = uu * uu + vv * vv, rq * rq
                cut = d2 > r2
                arc = (not cut) and (d2 + 2 * uu + 2 * vv + 2 > r2)
                arc2 = (not cut) and (d2 + 4 * uu + 4 * vv + 8 > r2)
                states.add((cut, arc, arc2))
    extra = states - legal
    print("状态自检：inc 区域合法状态 %d 种，越界 %d 种 %s"
          % (len(states), len(extra), sorted(extra) if extra else ""))
    return t, bad + len(extra)


def emit(t):
    v = 0
    for idx in range(511, -1, -1):
        r3, dx, dy = idx >> 6, (idx >> 3) & 7, idx & 7
        v = (v << 2) | t[(r3, dx, dy)]
    hx = "%0256x" % v
    print("localparam [1023:0] CORN_LUT = 1024'h")
    for i in range(0, 256, 64):
        sep = "," if i + 64 < 256 else ";"
        print("    " + hx[i:i + 64] + sep)


if __name__ == "__main__":
    t, bad = check()
    if "--check" not in sys.argv:
        print()
        emit(t)
    sys.exit(1 if bad else 0)
