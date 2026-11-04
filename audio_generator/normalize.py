#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
批量响度平衡（EBU R128 / ITU-R BS.1770），基于 ffmpeg loudnorm 两遍法。

为什么用两遍：loudnorm 单遍是"边分析边压"的动态模式，每条音频压法不同，
批量做出来响度反而不一致。两遍法先测出每条的真实响度，再以线性增益对齐，
只改音量不改动态，44 条听感上才整齐。

用法：
  python normalize.py                       # 输出到 out/normalized（不动原文件）
  python normalize.py --in-place             # 直接覆盖 out 里的原文件（会先备份）
  python normalize.py -t -14                 # 目标 -14 LUFS（默认 -16）
  python normalize.py --tp -1.0              # 真峰值上限（默认 -1.5 dBTP）
  python normalize.py --measure-only         # 只测不改，输出报告

依赖：仅标准库 + PATH 中的 ffmpeg
"""

import argparse
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

FFMPEG = "ffmpeg"

DEFAULT_TARGET_I = -16.0   # 综合响度目标（流媒体/短视频常用 -16 ~ -14 LUFS）
DEFAULT_TARGET_TP = -1.5   # 真峰值上限
DEFAULT_TARGET_LRA = 11.0  # 响度范围（只作为 loudnorm 的约束参考）

JSON_RE = re.compile(r"\{[\s\S]*?\}")


def run(args: list) -> subprocess.CompletedProcess:
    return subprocess.run(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE)


def measure(path: Path, target_i: float, target_tp: float, target_lra: float) -> dict:
    """第一遍：只分析，返回 loudnorm 的测量值（input_i / input_tp / ...）。"""
    af = f"loudnorm=I={target_i}:TP={target_tp}:LRA={target_lra}:print_format=json"
    proc = run([FFMPEG, "-hide_banner", "-nostats", "-i", str(path), "-af", af, "-f", "null", "-"])
    err = proc.stderr.decode("utf-8", "replace")
    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg 分析失败：{err.strip().splitlines()[-1] if err.strip() else ''}")

    block = None
    for m in JSON_RE.finditer(err):
        if "input_i" in m.group(0):
            block = m.group(0)
    if not block:
        raise RuntimeError("没解析到 loudnorm 输出")

    import json
    d = json.loads(block)
    return {
        "i": float(d["input_i"]),
        "tp": float(d["input_tp"]),
        "lra": float(d["input_lra"]),
        "thresh": float(d["input_thresh"]),
        "offset": float(d["target_offset"]),
    }


def apply_norm(src: Path, dst: Path, m: dict, target_i: float,
               target_tp: float, target_lra: float, sample_rate: int, channels: int) -> str:
    """第二遍：用第一遍的测量值做线性增益 + 真峰值限幅。"""
    af = (
        f"loudnorm=I={target_i}:TP={target_tp}:LRA={target_lra}"
        f":measured_I={m['i']}:measured_TP={m['tp']}:measured_LRA={m['lra']}"
        f":measured_thresh={m['thresh']}:offset={m['offset']}"
        f":linear=true:print_format=none"
    )
    proc = run([
        FFMPEG, "-y", "-v", "error", "-i", str(src), "-af", af,
        "-ar", str(sample_rate), "-ac", str(channels),
        "-sample_fmt", "s16", "-c:a", "pcm_s16le", str(dst),
    ])
    err = proc.stderr.decode("utf-8", "replace")
    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg 处理失败：{err.strip()[:300]}")
    return "linear" if "linear" in err else "linear"


def main() -> int:
    ap = argparse.ArgumentParser(description="批量响度平衡")
    ap.add_argument("-d", "--dir", default="out", help="音频目录，默认 out")
    ap.add_argument("-o", "--out-dir", default=None, help="输出目录，默认 <dir>/normalized")
    ap.add_argument("--in-place", action="store_true", help="原地覆盖（先备份到 _orig 子目录）")
    ap.add_argument("-t", "--target-i", type=float, default=DEFAULT_TARGET_I, help="目标综合响度 LUFS")
    ap.add_argument("--tp", type=float, default=DEFAULT_TARGET_TP, help="真峰值上限 dBTP")
    ap.add_argument("--lra", type=float, default=DEFAULT_TARGET_LRA, help="目标响度范围 LU")
    ap.add_argument("--sample-rate", type=int, default=48000)
    ap.add_argument("--channels", type=int, default=2)
    ap.add_argument("--measure-only", action="store_true", help="只测量输出报告")
    args = ap.parse_args()

    if shutil.which(FFMPEG) is None:
        print("找不到 ffmpeg", file=sys.stderr)
        return 2

    src_dir = Path(args.dir)
    files = sorted(src_dir.glob("*.WAV")) + sorted(src_dir.glob("*.wav"))
    files = [f for f in dict.fromkeys(files)]  # 去重（Windows 大小写不敏感）
    if not files:
        print(f"{src_dir} 里没有 WAV", file=sys.stderr)
        return 2

    # 输出位置
    if args.in_place:
        backup = src_dir / "_orig"
        backup.mkdir(exist_ok=True)
        out_dir = src_dir
    else:
        out_dir = Path(args.out_dir) if args.out_dir else src_dir / "normalized"
        out_dir.mkdir(parents=True, exist_ok=True)

    print(f"共 {len(files)} 个文件  目标 {args.target_i} LUFS / TP {args.tp} dBTP")
    if not args.measure_only:
        print(f"输出目录：{out_dir.resolve()}\n")

    rows = []
    fail = 0
    t0 = time.time()

    for idx, src in enumerate(files, 1):
        name = src.name
        try:
            m = measure(src, args.target_i, args.tp, args.lra)
            if m["i"] == float("-inf"):
                print(f"[{idx}/{len(files)}] {name} 全静音，跳过")
                rows.append((name, None, None, None))
                continue

            if args.measure_only:
                print(f"[{idx}/{len(files)}] {name}  {m['i']:6.1f} LUFS  peak {m['tp']:5.1f} dBTP  LRA {m['lra']:4.1f} LU")
                rows.append((name, m["i"], None, m["tp"]))
                continue

            dst = out_dir / name
            tmp = dst.with_suffix(".tmp.WAV")
            apply_norm(src, tmp, m, args.target_i, args.tp, args.lra,
                       args.sample_rate, args.channels)

            # 复测，确认真的到位
            after = measure(tmp, args.target_i, args.tp, args.lra)
            if args.in_place:
                shutil.copy2(src, backup / name)
            tmp.replace(dst)

            gain = args.target_i - m["i"]
            print(f"[{idx}/{len(files)}] {name}  {m['i']:6.1f} -> {after['i']:6.1f} LUFS "
                  f"(增益 {gain:+.1f} dB, peak {m['tp']:.1f} -> {after['tp']:.1f} dBTP)")
            rows.append((name, m["i"], after["i"], m["tp"]))
        except Exception as e:
            print(f"[{idx}/{len(files)}] {name}  失败：{e}", file=sys.stderr)
            fail += 1

    done = [r for r in rows if r[1] is not None and r[2] is not None]
    if done:
        before = [r[1] for r in done]
        after = [r[2] for r in done]
        print(f"\n处理前：{min(before):.1f} ~ {max(before):.1f} LUFS（极差 {max(before)-min(before):.1f} LU）")
        print(f"处理后：{min(after):.1f} ~ {max(after):.1f} LUFS（极差 {max(after)-min(after):.1f} LU）")
        print(f"耗时 {time.time()-t0:.0f}s，失败 {fail} 个")
    elif args.measure_only:
        vals = [r[1] for r in rows if r[1] is not None]
        if vals:
            print(f"\n当前：{min(vals):.1f} ~ {max(vals):.1f} LUFS（极差 {max(vals)-min(vals):.1f} LU），共 {len(vals)} 条")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
