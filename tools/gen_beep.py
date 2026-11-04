#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
gen_beep.py —— 生成系统提示音（裸 16bit 单声道 48kHz，无文件头）

    click  50ms   按键音
    ok     120ms  菜单开/关
    err    400ms  装载失败

输出 tools/out/beep.bin，烧录到 Flash 0x040000（partitions.json 的 beeps 分区）。
段表（地址/样本数）写在 RTL 参数里，所以文件里**不需要**任何头。
"""
import math
import os
import struct

SR = 48000
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out", "beep.bin")


def tone(freq, dur, amp=0.35, atk=0.004, rel=None, harm=0.45, square=0.0):
    """正弦（可叠二次谐波 / 方波成分）+ 起落包络"""
    if rel is None:
        rel = min(0.04, dur * 0.5)
    n = int(SR * dur)
    out = []
    for i in range(n):
        t = i / float(SR)
        env = min(1.0, t / atk) * min(1.0, max(0.0, (dur - t) / rel))
        s = math.sin(2 * math.pi * freq * t) + harm * math.sin(4 * math.pi * freq * t)
        if square:
            s = (1.0 - square) * s + square * (1.0 if math.sin(2 * math.pi * freq * t) >= 0 else -1.0)
        v = int(max(-32767, min(32767, s * amp * env * 32767)))
        out.append(v)
    return out


def sweep(f0, f1, dur, amp=0.30):
    """线性扫频（双音上行的柔和版）"""
    n = int(SR * dur)
    out = []
    ph = 0.0
    for i in range(n):
        t = i / float(SR)
        f = f0 + (f1 - f0) * (t / dur)
        ph += 2 * math.pi * f / SR
        env = min(1.0, t / 0.006) * min(1.0, max(0.0, (dur - t) / 0.05))
        v = int(max(-32767, min(32767, math.sin(ph) * amp * env * 32767)))
        out.append(v)
    return out


SEGS = [
    (0x0000, tone(880.0, 0.05, amp=0.32, atk=0.002, rel=0.03)),          # click
    (0x2000, sweep(1100.0, 1600.0, 0.12, amp=0.28)),                     # ok
    (0x6000, tone(220.0, 0.40, amp=0.30, atk=0.01, rel=0.20, harm=0.9, square=0.5)),  # err
]

blob = bytearray()
for off, smp in SEGS:
    if len(blob) > off:
        raise SystemExit("segment overlap at 0x%X" % off)
    blob += b"\x00" * (off - len(blob))
    blob += struct.pack("<%dh" % len(smp), *smp)

os.makedirs(os.path.dirname(OUT), exist_ok=True)
with open(OUT, "wb") as f:
    f.write(blob)

print("wrote %s  (%d bytes)" % (OUT, len(blob)))
for off, smp in SEGS:
    print("  seg @0x%05X  %6d samples  %5.1f ms" % (0x040000 + off, len(smp), len(smp) * 1000.0 / SR))
