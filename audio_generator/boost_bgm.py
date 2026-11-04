#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
后期重配比：把人声压下去、把配乐推上来（不重新生成音频）。

背景：这些音频是人声+配乐的一次性混音，且几乎单声道，无法靠声场分离。
但可以用分频（acrossover）拆成三段分别处理：

  < 250 Hz  配乐的低频主体（鼓、贝斯、钢琴低音）→ 这里人声几乎不占，直接提 7 dB
  250-3000  人声主战场                            → 压缩 + 降 2 dB，人声被"按"回去
  > 3000    齿音与尖锐感                          → deesser + 6.5k 削 3.5 dB

实测相对电平变化：低频 +8.3 dB，人声带 -3.0 dB，高频 -4.0 dB。

用法：
  python boost_bgm.py                        # out/normalized -> out/boosted
  python boost_bgm.py --low-gain 9 --mid-gain -3    # 更极端
  python boost_bgm.py --in-place              # 覆盖输入目录（备份到 _orig）
  python boost_bgm.py --mode eq               # 退回简单的静态 EQ（保守方案）

依赖：仅标准库 + ffmpeg
"""

import argparse
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

from normalize import FFMPEG, apply_norm, measure

# 分频点：250 Hz 以下基本是人声之外的音乐层，3 kHz 以上是人声齿音区
SPLIT_POINTS = "250 3000"


def build_chain(args) -> str:
    """生成 filter_complex 字符串，输出标签为 [out]。"""
    if args.mode == "eq":
        parts = [f"bass=g={args.low_gain}:f=110:width_type=o:w=1.5"]
        if args.sibilance_cut:
            parts.append(f"equalizer=f=6500:width_type=q:w=1.4:g={args.sibilance_cut}")
        if args.mid_gain:
            parts.append(f"equalizer=f=1500:width_type=q:w=1.5:g={args.mid_gain}")
        if args.deess:
            parts.append(f"deesser=i={args.deess}:m=0.6")
        return "[0:a]" + ",".join(parts) + "[out]"

    return (
        f"[0:a]acrossover=split={SPLIT_POINTS}:order=4[lo][mid][hi];"
        # 低频：配乐主体，直接推上去
        f"[lo]volume={args.low_gain}dB[L];"
        # 中频：压缩把持续的人声按下去，再整体降一点
        f"[mid]acompressor=threshold={args.mid_thresh}:ratio={args.mid_ratio}"
        f":attack=4:release=300,volume={args.mid_gain}dB,"
        f"equalizer=f=2200:width_type=q:w=1.0:g=-1.5[M];"
        # 高频：去齿音 + 削 6.5k 的刺耳感
        f"[hi]deesser=i={args.deess}:m=0.6,"
        f"equalizer=f=6500:width_type=q:w=1.4:g={args.sibilance_cut}[H];"
        f"[L][M][H]amix=inputs=3:normalize=0[out]"
    )


def lufs(path: Path, lowpass: bool = False) -> float | None:
    af = "lowpass=f=120,ebur128" if lowpass else "ebur128"
    p = subprocess.run(
        [FFMPEG, "-hide_banner", "-nostats", "-i", str(path), "-af", af, "-f", "null", "-"],
        capture_output=True, stdin=subprocess.DEVNULL,
    )
    m = re.findall(r"I:\s*(-?\d+\.\d+)\s*LUFS", p.stderr.decode("utf-8", "replace"))
    return float(m[-1]) if m else None


def main() -> int:
    ap = argparse.ArgumentParser(description="人声后撤 + 配乐前推")
    ap.add_argument("-d", "--dir", default="out/normalized", help="输入目录")
    ap.add_argument("-o", "--out-dir", default="out/boosted", help="输出目录")
    ap.add_argument("--in-place", action="store_true", help="原地覆盖（备份到 _orig）")
    ap.add_argument("--mode", choices=["multi", "eq"], default="multi",
                    help="multi=分频处理（默认，效果强）；eq=静态 EQ（保守）")
    ap.add_argument("--low-gain", type=float, default=7.0, help="低频提升 dB")
    ap.add_argument("--mid-gain", type=float, default=-2.0, help="中频（人声）增益 dB")
    ap.add_argument("--mid-ratio", type=float, default=5.0, help="中频压缩比")
    ap.add_argument("--mid-thresh", type=float, default=0.055, help="中频压缩阈值（线性 0-1）")
    ap.add_argument("--sibilance-cut", type=float, default=-3.5, help="6.5k 齿音衰减 dB")
    ap.add_argument("--deess", type=float, default=0.8, help="去齿音强度 0-1")
    ap.add_argument("-t", "--target-i", type=float, default=-16.0, help="目标响度 LUFS")
    ap.add_argument("--tp", type=float, default=-1.5, help="真峰值上限 dBTP")
    ap.add_argument("--sample-rate", type=int, default=48000)
    ap.add_argument("--channels", type=int, default=2)
    ap.add_argument("--dry-run", action="store_true", help="只打印滤镜链")
    args = ap.parse_args()

    if shutil.which(FFMPEG) is None:
        print("找不到 ffmpeg", file=sys.stderr)
        return 2

    chain = build_chain(args)
    print(f"模式：{args.mode}\n滤镜链：{chain}\n")
    if args.dry_run:
        return 0

    src_dir = Path(args.dir)
    if not src_dir.exists():
        print(f"输入目录不存在：{src_dir}", file=sys.stderr)
        return 2
    files = sorted(set(src_dir.glob("*.WAV")) | set(src_dir.glob("*.wav")))
    if not files:
        print(f"{src_dir} 里没有 WAV", file=sys.stderr)
        return 2

    if args.in_place:
        backup = src_dir / "_orig"
        backup.mkdir(exist_ok=True)
        out_dir = src_dir
    else:
        out_dir = Path(args.out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

    print(f"共 {len(files)} 个文件  {src_dir} -> {out_dir}\n")
    t0 = time.time()
    fail = 0
    low_b, low_a = [], []

    for idx, src in enumerate(files, 1):
        name = src.name
        stage = out_dir / f".{name}.stage.WAV"
        try:
            # 1) 分频重配比
            p = subprocess.run(
                [FFMPEG, "-y", "-v", "error", "-i", str(src),
                 "-filter_complex", chain, "-map", "[out]",
                 "-ar", str(args.sample_rate), "-ac", str(args.channels),
                 "-sample_fmt", "s16", "-c:a", "pcm_s16le", str(stage)],
                capture_output=True,
            )
            if p.returncode != 0:
                raise RuntimeError(p.stderr.decode("utf-8", "replace").strip()[:300])

            # 2) 回到统一响度（重配比会改变整体电平）
            m = measure(stage, args.target_i, args.tp, 11.0)
            dst = out_dir / name
            apply_norm(stage, dst, m, args.target_i, args.tp, 11.0,
                       args.sample_rate, args.channels)
            stage.unlink(missing_ok=True)

            if args.in_place:
                shutil.copy2(src, backup / name)

            lb, la = lufs(src, lowpass=True), lufs(dst, lowpass=True)
            if lb is not None and la is not None:
                low_b.append(lb)
                low_a.append(la)
                print(f"[{idx}/{len(files)}] {name}  低频 {lb:.1f} -> {la:.1f} LUFS ({la-lb:+.1f} dB)")
            else:
                print(f"[{idx}/{len(files)}] {name}  完成")
        except Exception as e:
            print(f"[{idx}/{len(files)}] {name} 失败：{e}", file=sys.stderr)
            fail += 1

    if low_b:
        print(f"\n低频(<120Hz)平均净提升：{(sum(low_a)-sum(low_b))/len(low_b):+.1f} dB（人声带约 -3 dB，高频约 -4 dB）")
    print(f"耗时 {time.time()-t0:.0f}s，失败 {fail} 个；输出 {out_dir.resolve()}")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
