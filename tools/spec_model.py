#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spec_model.py —— spec_fft 定点链的逐位黄金模型（回归用，不参与综合）

它把 src/user_source/hdl_source/audio/spec_fft.v 里的定点链重算一遍：
    24bit 左对齐 PCM ──>>8──▶ 16bit ──÷4 盒式抽取──▶ 12kHz
    ──Hann(ROM 128..255) × sample >> 17──▶ 位反转写入
    ──128 点 DIT（每级 >>1，W 取 ROM 0..63）──▶ re/im
    ──re²+im² ──串行归一化──▶ L = {ncnt, acc[27:25]}（1 单位 = 0.376dB）
    ──段内取 max（段尾由 ROM 64..127 的 bit8）──▶ h = sat127(L − off)
    ──峰值保持 DECAY/帧

用途：改窗、改分段、改每段偏置之前先跑它，看 16 条高度的形状/余量是否还成立。

用法：
    python tools/spec_model.py                       # 默认 1.wav
    python tools/spec_model.py some.wav --decay 3
    python tools/spec_model.py --quiet                # 只输出 PASS/FAIL

判定：
    · mean(h[0]) - mean(h[15]) >= 35 px   （形状要看得出来，旧版只有 23）
    · 每段 p99 < 127                       （不许削顶）
    · 每段 p99 > 30                        （不许整段贴底）
退出码：0 = PASS，1 = FAIL。
"""

import argparse
import os
import re
import sys
import wave

import numpy as np

N = 128
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SPEC = os.path.join(ROOT, "src", "user_source", "hdl_source", "audio", "spec_fft.v")
DEF_WAV = os.path.join(ROOT, "src", "user_source", "hdl_source", "python", "1.wav")


def parse_rom(spec_path):
    txt = open(spec_path, "r", encoding="utf-8", errors="ignore").read()
    rom = {}
    for m in re.finditer(r"rom\[\s*(\d+)\]\s*=\s*32'h([0-9A-Fa-f]{8})\s*;", txt):
        rom[int(m.group(1))] = int(m.group(2), 16)
    if len(rom) < 256:
        sys.exit("ROM_INIT parse failed (got %d words)" % len(rom))
    return [rom[i] for i in range(256)]


def bitrev(n, bits=7):
    r = 0
    for i in range(bits):
        r = (r << 1) | ((n >> i) & 1)
    return r


BREV = np.array([bitrev(n) for n in range(N)])


def fft_fixed(a, cos, negsin):
    """a 已按位反转地址存放；cos/negsin 是 64 点 Q1.15 ROM。"""
    re = np.array(a, dtype=np.int64)
    im = np.zeros(N, dtype=np.int64)
    h = 1
    while h < N:
        step = N // (2 * h)
        for k in range(0, N, 2 * h):
            for j in range(h):
                a0, b0 = int(re[k + j]), int(im[k + j])
                c, d = int(re[k + j + h]), int(im[k + j + h])
                w = (j * step) % 64
                wr, wi = cos[w], negsin[w]          # W = cos - j*sin
                tr = (c * wr - d * wi) >> 15
                ti = (c * wi + d * wr) >> 15
                re[k + j] = (a0 + tr) >> 1
                im[k + j] = (b0 + ti) >> 1
                re[k + j + h] = (a0 - tr) >> 1
                im[k + j + h] = (b0 - ti) >> 1
        h *= 2
    return re, im


def lg_of(p):
    if p <= 0:
        return 0
    acc, ncnt = int(p), 28
    while ncnt > 0 and not ((acc >> 28) & 1):
        acc <<= 1
        ncnt -= 1
    return (ncnt << 3) | ((acc >> 25) & 7)


def band_edges(words):
    """返回 16 个段的 (起始 bin, 结束 bin) 和每段偏置（ROM[64+bin] 的低 8 位）。"""
    last = [bin_ for bin_ in range(64) if (words[64 + bin_] >> 8) & 1]
    if len(last) != 16:
        sys.exit("band table is not 16 bands (got %d)" % len(last))
    edges, offs, start = [], [], 1
    for end in last:
        edges.append((start, end))
        offs.append(words[64 + end] & 0xFF)
        start = end + 1
    return edges, offs


def read_wav_left(path):
    w = wave.open(path, "rb")
    nch, sw, fs = w.getnchannels(), w.getsampwidth(), w.getframerate()
    raw = w.readframes(w.getnframes())
    if sw == 3:
        b = np.frombuffer(raw, dtype=np.uint8).reshape(-1, 3)
        v = (b[:, 0].astype(np.int32)
             | (b[:, 1].astype(np.int32) << 8)
             | (b[:, 2].astype(np.int32) << 16))
        v = np.where(v >= 2 ** 23, v - 2 ** 24, v)
    elif sw == 2:
        v = np.frombuffer(raw, dtype="<i2").astype(np.int32)
    else:
        v = np.frombuffer(raw, dtype="<i4").astype(np.int32)
    return v.reshape(-1, nch)[:, 0].astype(np.int64), fs, sw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("wav", nargs="?", default=DEF_WAV)
    ap.add_argument("--decay", type=int, default=2)   # 峰值保持每帧下落像素（RTL: DECAY=2）
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    words = parse_rom(SPEC)
    cos = np.array([(words[i] >> 16) & 0xFFFF for i in range(64)], dtype=np.int64)
    negsin = np.array([words[i] & 0xFFFF for i in range(64)], dtype=np.int64)
    cos = np.where(cos >= 2 ** 15, cos - 2 ** 16, cos)
    negsin = np.where(negsin >= 2 ** 15, negsin - 2 ** 16, negsin)
    hann = np.array([words[128 + n] & 0xFFFF for n in range(N)], dtype=np.int64)
    edges, offs = band_edges(words)

    x, fs, sw = read_wav_left(args.wav)
    if fs != 48000:
        sys.exit("expect 48 kHz wav, got %d" % fs)
    d = (x >> 8)                                    # smp[23:8]
    m = len(d) // 4 * 4
    dec = (d[:m].reshape(-1, 4).sum(axis=1)) >> 2   # ÷4 盒式抽取 → 12 kHz

    step = int(round(fs * 16.667 / 1000 / 4))       # 每帧前进的 12k 样本数
    nf = (len(dec) - N) // step
    heights, hold = [], np.zeros(16, dtype=int)
    for f in range(nf):
        seg = dec[f * step: f * step + N]
        w = (seg * hann) >> 17                       # 加窗：mpy[32:17]
        a = np.zeros(N, dtype=np.int64)
        a[BREV] = w
        re, im = fft_fixed(a, cos, negsin)
        lg = np.array([lg_of(int(p)) for p in re * re + im * im])
        # 条高 = sat127(段内最大 lg − 该段偏置)；16 段现在统一 off=48
        hs = np.array([min(127, max(0, int(lg[s:e + 1].max()) - off))
                       for (s, e), off in zip(edges, offs)])
        hold = np.maximum(hs, np.maximum(hold - args.decay, 0))
        heights.append(hold.copy())
    H = np.array(heights)

    mean, p99 = H.mean(0), np.percentile(H, 99, axis=0)
    if not args.quiet:
        print("file : %s  (%d-bit, %d frames, off=%s, DECAY=%d)"
              % (os.path.relpath(args.wav, ROOT), sw * 8, len(H),
                 sorted(set(offs)), args.decay))
        print("band :" + "".join("%5d" % i for i in range(16)))
        print("mean :" + "".join("%5.1f" % v for v in mean))
        print("p99  :" + "".join("%5.0f" % v for v in p99))
        print("max  :" + "".join("%5d" % v for v in H.max(0)))
        print("slope: mean(0)-mean(15) = %.1f px" % (mean[0] - mean[15]))

    errs = []
    if mean[0] - mean[15] < 35:
        errs.append("slope %.1f < 35 px (每段偏置没统一?)" % (mean[0] - mean[15]))
    if (p99 >= 127).any():
        errs.append("bands %s clip at 127" % list(np.where(p99 >= 127)[0]))
    if (p99 <= 30).any():
        errs.append("bands %s sit at bottom" % list(np.where(p99 <= 30)[0]))
    if errs:
        for e in errs:
            print("FAIL:", e)
        return 1
    print("ALL PASS spec_model")
    return 0


if __name__ == "__main__":
    sys.exit(main())
