#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
keepres_to_bmp.py —— 任意图片 → 本工程 FPGA 能读的 BMP（保持原始分辨率，不缩放）

【为什么只做"色彩归一化"、不做 resize】
本工程的 BMP 判据（RTL: SD/bmp_loader.v 的 hdr_match，与旧 SD/bmp_read.v 一致）：
    ① 签名 'B''M'                 ② 位深 24（RGB888）
    ③ 压缩方式 0（BI_RGB 无压缩） ④ 0 < 宽,高 <= 4096
    ⑤ 高度为正（bottom-up；负数=top-down 会被判非法）
Pillow 的 BMP 编码器默认写的就是：BITMAPINFOHEADER(40B) + 数据偏移 54 + BGR 序
+ 每行 4 字节对齐 + bottom-up，正好全覆盖上面 5 条。所以这里**只改色彩模式**，
宽高原样保留 —— 分辨率零损失。

【用法】
    python keepres_to_bmp.py                          # 一键：脚本旁 input_images/ -> output_bmp/
    python keepres_to_bmp.py 1.jpg                    # 单文件 -> 同目录 1.bmp
    python keepres_to_bmp.py 1.jpg -o out/            # 指定输出目录
    python keepres_to_bmp.py pics/ -o out/            # 整个目录
    python keepres_to_bmp.py a.png b.webp c/ -o out/ --short-names
    python keepres_to_bmp.py pics/ -o out/ --bg white # 透明区垫白（默认垫黑）

【可选参数】
    -o/--out DIR      输出目录（默认：批量=output_bmp，单文件=源文件同目录）
    --bg COLOR        透明区背景：black / white / #RRGGBB（默认 black）
    --max-side N      边长上限，默认 4096（超过直接判非法，不偷偷缩）
    --overwrite       允许覆盖同名输出
    --short-names     输出改成 8.3 短名 img0001.bmp（FPGA 的 FAT 目录扫描只认 8.3）
    --recursive       目录递归
    -q/--quiet        只打错误
"""

from __future__ import annotations

import argparse
import re
import struct
import sys
from pathlib import Path

try:
    from PIL import Image, ImageOps
except ImportError:  # pragma: no cover
    sys.exit("缺少 Pillow，请先执行：  pip install Pillow")


# ----------------------------------------------------------------------------
# 常量
# ----------------------------------------------------------------------------
MAX_SIDE = 4096                 # RTL 判据：宽高 <= 4096
CANVAS_W, CANVAS_H = 640, 480   # HDMI 画布（只用来算"等比显示成多大"，不改图）
DEFAULT_IN = "input_images"
DEFAULT_OUT = "output_bmp"

IMG_EXTS = {
    ".bmp", ".dib", ".png", ".jpg", ".jpeg", ".jpe", ".jfif", ".webp", ".gif",
    ".tif", ".tiff", ".ppm", ".pgm", ".pbm", ".tga", ".ico", ".pcx",
    ".avif", ".heic", ".heif", ".jp2",
}

NAMED_BG = {
    "black": (0, 0, 0),
    "white": (255, 255, 255),
    "gray": (128, 128, 128),
    "grey": (128, 128, 128),
    "red": (255, 0, 0),
    "green": (0, 128, 0),
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
    """让 2.bmp 排在 10.bmp 前面。"""
    return [(0, int(t)) if t.isdigit() else (1, t) for t in re.split(r"(\d+)", p.name.lower())]


def canvas_fit(w: int, h: int) -> tuple[int, int]:
    """bmp_view 的 fit 几何：max(W/640, H/480) 为缩放比，等比塞进 640x480。"""
    step = max(w / CANVAS_W, h / CANVAS_H)
    return max(1, int(round(w / step))), max(1, int(round(h / step)))


# ----------------------------------------------------------------------------
# 色彩模式归一化（核心：尺寸一根手指都不动）
# ----------------------------------------------------------------------------
def to_rgb(img: Image.Image, bg: tuple[int, int, int]) -> Image.Image:
    """
    任意模式 → RGB，尺寸保持 img.size 不变。

    几个必须有专门处理的坑：
      · I;16 / I / F  —— 16/32 位灰度。直接 convert('RGB') 会把 >255 的值全截成
        纯白（Pillow 不做位深缩放），所以先按 65535 -> 255 线性压到 8 位。
      · P + transparency / RGBA / LA / RGBa —— 有 alpha 的必须合成到不透明底色，
        否则 convert('RGB') 会丢掉 alpha 直接把 RGB 原样拿来，半透明处会变脏。
      · CMYK —— 走 Pillow 的 CMYK->RGB（JPEG 印刷稿常见）。
    """
    mode = img.mode

    # --- 高位深灰度 ---
    if mode in ("I;16", "I;16B", "I;16L", "I;16N", "I"):
        return img.convert("I").point(lambda v: v / 257.0).convert("RGB")
    if mode == "F":
        lo, hi = img.getextrema()
        if hi <= lo:
            return img.convert("L").convert("RGB")
        span = float(hi - lo)
        return img.point(lambda v: (v - lo) * 255.0 / span).convert("L").convert("RGB")

    # --- 带 alpha ---
    has_alpha = mode in ("RGBA", "LA", "La", "PA", "RGBa") or (
        mode == "P" and "transparency" in img.info
    )
    if has_alpha:
        rgba = img.convert("RGBA")
        canvas = Image.new("RGBA", rgba.size, bg + (255,))
        canvas.alpha_composite(rgba)
        return canvas.convert("RGB")

    # --- 其余 ---
    if mode == "1":                       # 二值图：转灰度再转 RGB，别直接放大成 0/255
        return img.convert("L").convert("RGB")
    return img.convert("RGB")


# ----------------------------------------------------------------------------
# BMP 头校验（写完立刻回读，出问题当场报，不把坏文件丢给板子）
# ----------------------------------------------------------------------------
def read_bmp_header(path: Path) -> dict:
    with open(path, "rb") as f:
        head = f.read(54)
    if len(head) < 54:
        raise ValueError("文件不足 54 字节")
    return {
        "sig": head[0:2],
        "file_len": struct.unpack("<I", head[2:6])[0],
        "data_off": struct.unpack("<I", head[10:14])[0],
        "hdr_size": struct.unpack("<I", head[14:18])[0],
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
def convert_one(src: Path, dst: Path, bg: tuple[int, int, int],
                max_side: int, overwrite: bool) -> tuple[bool, str]:
    try:
        with Image.open(src) as im:
            if getattr(im, "is_animated", False):
                im.seek(0)                      # GIF/WebP 动图取第一帧
            try:
                im = ImageOps.exif_transpose(im)  # 手机照片的 EXIF 旋转
            except Exception:
                pass
            w, h = im.size
            if w <= 0 or h <= 0:
                return False, "尺寸为 0"
            if w > max_side or h > max_side:
                return False, (f"{w}x{h} 超过 {max_side} 上限 —— 本工程会判为非法 BMP"
                               f"（不偷偷缩小；确需缩小请用 convert_images_to_bmp.py）")
            rgb = to_rgb(im, bg)
            if rgb.size != (w, h):              # 理论上不会发生，保险
                rgb = rgb.resize((w, h), Image.Resampling.NEAREST)

            if dst.exists() and not overwrite:
                return False, f"{dst.name} 已存在（加 --overwrite 覆盖）"
            dst.parent.mkdir(parents=True, exist_ok=True)
            rgb.save(dst, "BMP")               # Pillow 默认：40B 头 + 54 偏移 + bottom-up + BGR
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"

    errs = verify_bmp(dst, w, h)
    if errs:
        return False, "头校验失败: " + "; ".join(errs)

    dw, dh = canvas_fit(w, h)
    kb = dst.stat().st_size / 1024
    note = ""
    if h == 4096:
        # bmp_loader 用 h32[11:0] 算 seek 行数，4096 = 0x1000 会被截成 0 ⇒ 板上会花屏
        note = "  [注意] 高度正好 4096 会被 RTL 的 12 位行号截断，建议降到 4095 以内"
    return True, f"{w}x{h} 24bit（画布内等比显示 {dw}x{dh}）{kb:,.0f} KB{note}"


# ----------------------------------------------------------------------------
# 输入收集
# ----------------------------------------------------------------------------
def collect(paths: list[str], recursive: bool) -> tuple[list[Path], list[Path]]:
    """返回 (要转的文件, 扫描用到的目录)。显式给出的文件不看后缀，扫目录才按后缀过滤。"""
    files: list[Path] = []
    dirs: list[Path] = []
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
        description="任意图片 -> 本工程要求的 BMP（24bit/无压缩/bottom-up，保持原分辨率）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("inputs", nargs="*", help="图片文件或目录（留空=脚本旁 input_images/）")
    ap.add_argument("-o", "--out", help="输出目录")
    ap.add_argument("--bg", default="black", help="透明区背景（默认 black）")
    ap.add_argument("--max-side", type=int, default=MAX_SIDE, help="边长上限（默认 4096）")
    ap.add_argument("--overwrite", action="store_true", help="覆盖同名输出")
    ap.add_argument("--short-names", action="store_true", help="输出 8.3 短名 img0001.bmp")
    ap.add_argument("--recursive", action="store_true", help="目录递归")
    ap.add_argument("-q", "--quiet", action="store_true", help="只打错误")
    args = ap.parse_args()

    bg = parse_bg(args.bg)
    script_dir = Path(__file__).resolve().parent

    # ---- 无参数 = 一键批处理：脚本旁 input_images -> output_bmp ----
    if not args.inputs:
        in_dir = script_dir / DEFAULT_IN
        if not in_dir.exists():
            in_dir.mkdir(parents=True, exist_ok=True)
            print("=== 任意图片 -> 工程 BMP（保持原分辨率）===")
            print(f"已创建输入目录：{in_dir}")
            print("把图片丢进去，再运行一次本脚本即可。")
            if sys.stdin and sys.stdin.isatty():
                try:
                    input("\n按回车键退出…")
                except (EOFError, KeyboardInterrupt):
                    pass
            return 0
        args.inputs = [str(in_dir)]
        if not args.out:
            args.out = str(script_dir / DEFAULT_OUT)

    files, dirs = collect(args.inputs, args.recursive)

    # ---- 输出目录 ----
    if args.out:
        out_dir = Path(args.out)
    elif dirs:
        out_dir = Path(DEFAULT_OUT)
    else:
        out_dir = None                                  # 单文件 -> 同目录

    if not files:
        print("没有找到可转换的图片。")
        return 0

    if not args.quiet:
        print(f"=== 任意图片 -> 工程 BMP（24bit 无压缩，保持原分辨率，边长 <= {args.max_side}）===")
        print(f"共 {len(files)} 个文件，输出到：{out_dir or '（源文件同目录）'}")
        print("-" * 72)

    ok = fail = 0
    used_names: set[str] = set()
    for idx, src in enumerate(files, 1):
        target_dir = out_dir if out_dir else src.parent
        if args.short_names:
            name = f"img{idx:04d}.bmp"
        else:
            name = src.stem + ".bmp"
            # 同名（不同目录下的 a.jpg / a.png）防撞
            n = 2
            while name.lower() in used_names:
                name = f"{src.stem}_{n}.bmp"
                n += 1
        used_names.add(name.lower())
        dst = target_dir / name

        good, msg = convert_one(src, dst, bg, args.max_side, args.overwrite)
        if good:
            ok += 1
            if not args.quiet:
                print(f"[OK]   {src.name}  ->  {name}   {msg}")
        else:
            fail += 1
            print(f"[失败] {src.name}: {msg}")

    if not args.quiet:
        print("-" * 72)
        print(f"完成：成功 {ok} 个，失败 {fail} 个。")
        if ok:
            print("拷贝到 TF 卡根目录即可（BMP 头必须落在扇区起始，正常拷贝天然满足）；")
            print("卡上文件名建议 8.3 短名，可用 --short-names 直接生成。")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
