#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scale_to_bmp.py —— 任意图片 → 等比缩放到不超过 1080p → 本工程能读的 BMP

与 keepres_to_bmp.py 的关系：
    keepres_to_bmp.py  保持原分辨率，只做色彩归一化（尺寸一根手指都不动）
    本脚本            先等比缩放（默认上限 1920x1080），再输出同样的 BMP

BMP 判据与 keepres_to_bmp.py 完全一致（RTL: SD/bmp_loader.v 的 hdr_match）：
    ① 签名 'B''M'   ② 24bit  ③ 无压缩(BI_RGB)  ④ 0 < 宽,高 <= 4096  ⑤ 高度为正(bottom-up)
Pillow 的 BMP 编码器默认就是 40B 头 + 54 偏移 + BGR + 4 字节行对齐 + bottom-up，全覆盖。

【缩放策略】
    · 等比：只按一个缩放比 s = min(maxW/w, maxH/h)，绝不拉伸变形
    · 不放大：原图比 1080p 小就原样输出（要放大加 --upscale）
    · 算法：默认 LANCZOS（Lanczos3，加窗 sinc）。照片类缩小的画质上限就是它——
      比 BICUBIC 更锐、细节保留更多，且 Pillow 会随缩放比例自动放大滤波核，
      大比例缩小时不会像 NEAREST/BILINEAR 那样出摩尔纹或糊成一片。
    · 不做锐化：默认不掺任何主观增强，像素可追溯；确需提亮细节可加 --sharpen

【用法】
    python scale_to_bmp.py                             # 脚本旁 input_images/ -> output_bmp/
    python scale_to_bmp.py pics/ -o out/               # 整个目录
    python scale_to_bmp.py a.jpg b.png -o out/         # 指定文件
    python scale_to_bmp.py pics/ -o out/ --max-h 720   # 换成 720p 上限
    python scale_to_bmp.py pics/ -o out/ --filter bicubic
    python scale_to_bmp.py pics/ -o out/ --report log.csv

【可选参数】
    --max-w N        宽上限，默认 1920
    --max-h N        高上限，默认 1080
    --filter NAME    lanczos(默认) / bicubic / box / hamming / bilinear / nearest
    --sharpen R      缩小后做一次 USM 锐化，R 为半径（如 0.6），默认关闭
    --upscale        允许把小图放大到上限
    --bg COLOR       透明区背景 black / white / #RRGGBB（默认 black）
    --overwrite      覆盖同名输出
    --short-names    输出 8.3 短名 img0001.bmp
    --report PATH    写一份 CSV 转换报表
    -o/--out DIR     输出目录
    -q/--quiet       只打错误
"""

from __future__ import annotations

import argparse
import csv
import re
import struct
import sys
from pathlib import Path

try:
    from PIL import Image, ImageFilter, ImageOps
except ImportError:  # pragma: no cover
    sys.exit("缺少 Pillow，请先执行：  pip install Pillow")


MAX_SIDE = 4096                 # RTL 判据：宽高 <= 4096
DEFAULT_MAX_W, DEFAULT_MAX_H = 1920, 1080
DEFAULT_IN = "input_images"
DEFAULT_OUT = "output_bmp"

IMG_EXTS = {
    ".bmp", ".dib", ".png", ".jpg", ".jpeg", ".jpe", ".jfif", ".webp", ".gif",
    ".tif", ".tiff", ".ppm", ".pgm", ".pbm", ".tga", ".ico", ".pcx",
    ".avif", ".heic", ".heif", ".jp2",
}

FILTERS = {
    "lanczos": Image.Resampling.LANCZOS,
    "bicubic": Image.Resampling.BICUBIC,
    "hamming": Image.Resampling.HAMMING,
    "box": Image.Resampling.BOX,
    "bilinear": Image.Resampling.BILINEAR,
    "nearest": Image.Resampling.NEAREST,
}

NAMED_BG = {
    "black": (0, 0, 0), "white": (255, 255, 255), "gray": (128, 128, 128),
    "grey": (128, 128, 128), "red": (255, 0, 0), "green": (0, 128, 0),
    "blue": (0, 0, 255),
}


# ----------------------------------------------------------------------------
# 小工具
# ----------------------------------------------------------------------------
def parse_bg(text: str) -> tuple[int, int, int]:
    s = str(text).strip().lower()
    if s in NAMED_BG:
        return NAMED_BG[s]
    if s.startswith("#"):
        s = s[1:]
    if len(s) == 3:
        s = "".join(c * 2 for c in s)
    if len(s) == 6:
        try:
            return tuple(int(s[i:i + 2], 16) for i in (0, 2, 4))  # type: ignore[return-value]
        except ValueError:
            pass
    raise SystemExit(f"--bg 无法识别：{text!r}（可用 black/white/#RRGGBB）")


def natural_key(p: Path):
    return [(0, int(t)) if t.isdigit() else (1, t) for t in re.split(r"(\d+)", p.name.lower())]


def fit_within(w: int, h: int, max_w: int, max_h: int, allow_upscale: bool) -> tuple[int, int]:
    """等比缩放到 max_w x max_h 之内；默认只缩不放。"""
    s = min(max_w / w, max_h / h)
    if s >= 1.0 and not allow_upscale:
        return w, h
    nw, nh = max(1, int(round(w * s))), max(1, int(round(h * s)))
    return min(nw, max_w), min(nh, max_h)


# ----------------------------------------------------------------------------
# 色彩模式归一化
# ----------------------------------------------------------------------------
def to_rgb(img: Image.Image, bg: tuple[int, int, int]) -> Image.Image:
    mode = img.mode
    if mode in ("I;16", "I;16B", "I;16L", "I;16N", "I"):
        return img.convert("I").point(lambda v: v / 257.0).convert("RGB")
    if mode == "F":
        lo, hi = img.getextrema()
        if hi <= lo:
            return img.convert("L").convert("RGB")
        span = float(hi - lo)
        return img.point(lambda v: (v - lo) * 255.0 / span).convert("L").convert("RGB")

    has_alpha = mode in ("RGBA", "LA", "La", "PA", "RGBa") or (
        mode == "P" and "transparency" in img.info
    )
    if has_alpha:
        rgba = img.convert("RGBA")
        canvas = Image.new("RGBA", rgba.size, bg + (255,))
        canvas.alpha_composite(rgba)
        return canvas.convert("RGB")

    if mode == "1":
        return img.convert("L").convert("RGB")
    return img.convert("RGB")


# ----------------------------------------------------------------------------
# BMP 头校验
# ----------------------------------------------------------------------------
def read_bmp_header(path: Path) -> dict:
    with open(path, "rb") as f:
        head = f.read(54)
    if len(head) < 54:
        raise ValueError("文件不足 54 字节")
    return {
        "sig": head[0:2],
        "data_off": struct.unpack("<I", head[10:14])[0],
        "w": struct.unpack("<i", head[18:22])[0],
        "h": struct.unpack("<i", head[22:26])[0],
        "bpp": struct.unpack("<H", head[28:30])[0],
        "comp": struct.unpack("<I", head[30:34])[0],
        "size": path.stat().st_size,
    }


def verify_bmp(path: Path, w: int, h: int) -> list[str]:
    info = read_bmp_header(path)
    stride = (w * 3 + 3) & ~3
    need = info["data_off"] + stride * h
    errs = []
    if info["sig"] != b"BM":
        errs.append("签名不是 BM")
    if info["bpp"] != 24:
        errs.append(f"位深 {info['bpp']} != 24")
    if info["comp"] != 0:
        errs.append(f"压缩方式 {info['comp']} != 0")
    if info["h"] <= 0:
        errs.append("高不为正（top-down）")
    if (info["w"], info["h"]) != (w, h):
        errs.append(f"尺寸变成 {info['w']}x{info['h']}（应为 {w}x{h}）")
    if info["size"] != need:
        errs.append(f"文件长度 {info['size']} != {need}（54 + stride*H）")
    return errs


# ----------------------------------------------------------------------------
# 单文件转换
# ----------------------------------------------------------------------------
def convert_one(src: Path, dst: Path, bg, max_w, max_h, resample, sharpen, upscale,
                overwrite) -> tuple[bool, str, dict | None]:
    try:
        with Image.open(src) as im:
            if getattr(im, "is_animated", False):
                im.seek(0)
            try:
                im = ImageOps.exif_transpose(im)
            except Exception:
                pass
            ow, oh = im.size
            if ow <= 0 or oh <= 0:
                return False, "尺寸为 0", None

            nw, nh = fit_within(ow, oh, max_w, max_h, upscale)
            rgb = to_rgb(im, bg)
            # 先缩再转 BMP；缩放与色彩归一化的先后不影响结果（都是逐通道线性/滤波）
            if (nw, nh) != (ow, oh):
                rgb = rgb.resize((nw, nh), resample)
                if sharpen and sharpen > 0:
                    rgb = rgb.filter(ImageFilter.UnsharpMask(radius=sharpen, percent=60, threshold=3))

            if dst.exists() and not overwrite:
                return False, f"{dst.name} 已存在（加 --overwrite 覆盖）", None
            dst.parent.mkdir(parents=True, exist_ok=True)
            rgb.save(dst, "BMP")
    except Exception as e:
        return False, f"{type(e).__name__}: {e}", None

    errs = verify_bmp(dst, nw, nh)
    if errs:
        return False, "头校验失败: " + "; ".join(errs), None

    kb = dst.stat().st_size / 1024
    ratio = (nw * nh) / (ow * oh)
    note = ""
    if nh == 4096:
        note = "  [注意] 高度正好 4096 会被 RTL 的 12 位行号截断"
    row = {"src": src.name, "dst": dst.name, "ow": ow, "oh": oh, "nw": nw, "nh": nh,
           "kb": round(kb), "ratio": round(ratio, 4)}
    return True, f"{ow}x{oh} -> {nw}x{nh}  ({ratio * 100:.1f}% 面积)  {kb:,.0f} KB{note}", row


# ----------------------------------------------------------------------------
# 输入收集
# ----------------------------------------------------------------------------
def collect(paths: list[str], recursive: bool) -> tuple[list[Path], list[Path]]:
    files, dirs = [], []
    for raw in paths:
        p = Path(raw)
        if p.is_dir():
            dirs.append(p)
            it = p.rglob("*") if recursive else p.glob("*")
            files += [f for f in it if f.is_file() and f.suffix.lower() in IMG_EXTS]
        elif p.is_file():
            files.append(p)
        else:
            print(f"[跳过] 找不到：{p}")

    seen, uniq = set(), []
    for f in files:
        key = str(f.resolve()).lower()
        if key not in seen:
            seen.add(key)
            uniq.append(f)
    uniq.sort(key=natural_key)
    return uniq, dirs


# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(
        description="任意图片 -> 等比缩放(默认上限 1920x1080) -> 工程 BMP(24bit/无压缩/bottom-up)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("inputs", nargs="*", help="图片文件或目录（留空=脚本旁 input_images/）")
    ap.add_argument("-o", "--out", help="输出目录")
    ap.add_argument("--max-w", type=int, default=DEFAULT_MAX_W, help="宽上限，默认 1920")
    ap.add_argument("--max-h", type=int, default=DEFAULT_MAX_H, help="高上限，默认 1080")
    ap.add_argument("--filter", default="lanczos", choices=sorted(FILTERS),
                    help="缩放算法，默认 lanczos")
    ap.add_argument("--sharpen", type=float, default=0.0, help="缩小后 USM 锐化半径，默认 0")
    ap.add_argument("--upscale", action="store_true", help="允许把小图放大到上限")
    ap.add_argument("--bg", default="black", help="透明区背景（默认 black）")
    ap.add_argument("--overwrite", action="store_true", help="覆盖同名输出")
    ap.add_argument("--short-names", action="store_true", help="输出 8.3 短名 img0001.bmp")
    ap.add_argument("--report", help="写 CSV 报表到指定路径")
    ap.add_argument("--recursive", action="store_true", help="目录递归")
    ap.add_argument("-q", "--quiet", action="store_true", help="只打错误")
    args = ap.parse_args()

    for v, name in ((args.max_w, "--max-w"), (args.max_h, "--max-h")):
        if not (0 < v <= MAX_SIDE):
            sys.exit(f"{name} 必须在 1..{MAX_SIDE} 之间（RTL 判据），当前 {v}")

    bg = parse_bg(args.bg)
    resample = FILTERS[args.filter]
    script_dir = Path(__file__).resolve().parent

    if not args.inputs:
        in_dir = script_dir / DEFAULT_IN
        if not in_dir.exists():
            in_dir.mkdir(parents=True, exist_ok=True)
            print("=== 图片 -> 等比缩放 -> 工程 BMP ===")
            print(f"已创建输入目录：{in_dir}")
            print("把图片丢进去，再运行一次本脚本即可。")
            return 0
        args.inputs = [str(in_dir)]
        if not args.out:
            args.out = str(script_dir / DEFAULT_OUT)

    files, dirs = collect(args.inputs, args.recursive)
    if not files:
        print("没有找到可转换的图片。")
        return 0

    if args.out:
        out_dir = Path(args.out)
    elif dirs:
        out_dir = Path(DEFAULT_OUT)
    else:
        out_dir = None

    if not args.quiet:
        print(f"=== 等比缩放到 <= {args.max_w}x{args.max_h} -> BMP(24bit 无压缩) ===")
        print(f"算法 {args.filter}，共 {len(files)} 个文件，输出到：{out_dir or '（源文件同目录）'}")
        print("-" * 72)

    ok = fail = 0
    used: set[str] = set()
    rows = []
    for idx, src in enumerate(files, 1):
        target_dir = out_dir if out_dir else src.parent
        if args.short_names:
            name = f"img{idx:04d}.bmp"
        else:
            name = src.stem + ".bmp"
            n = 2
            while name.lower() in used:
                name = f"{src.stem}_{n}.bmp"
                n += 1
        used.add(name.lower())
        dst = target_dir / name

        good, msg, row = convert_one(src, dst, bg, args.max_w, args.max_h, resample,
                                     args.sharpen, args.upscale, args.overwrite)
        if good:
            ok += 1
            if row:
                rows.append(row)
            if not args.quiet:
                print(f"[OK]   {src.name:28s} -> {name:16s} {msg}")
        else:
            fail += 1
            print(f"[失败] {src.name}: {msg}")

    if args.report and rows:
        with open(args.report, "w", encoding="utf-8-sig", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=["src", "dst", "ow", "oh", "nw", "nh", "kb", "ratio"])
            w.writeheader()
            w.writerows(rows)

    if not args.quiet:
        print("-" * 72)
        total = sum(r["kb"] for r in rows) / 1024
        print(f"完成：成功 {ok} 个，失败 {fail} 个，输出共 {total:.1f} MB。")
        if ok:
            print("拷贝到 TF 卡根目录即可；卡上文件名建议 8.3 短名（本脚本产物已是）。")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
