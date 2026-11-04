# -*- coding: utf-8 -*-
"""[已作废 2026-10-28] 把 icons/png32/*.png 打包成 4bpp 覆盖率图 + 调色板的 bin。

新的打包脚本是 tools/gen_osd_bitmaps.py（图标 RGB565 + logo ARGB8888，
整块 ROM 直出色彩，删掉了 osd_gfx 里 ≈180 LUT 的调色板）。
本文件只为旧格式留档，不再参与构建。

打包思路
--------
每个图标只有两种颜色（底板色 + 墨色），抗锯齿像素都落在"底板→墨色"这条连线上，
所以一个像素只需要一个 0..15 的色阶（覆盖率）就能表示，不需要存 RGB。
普通/激活两态几何完全一致，只是调色板不同 —— 5 张覆盖率图 + 2 组调色板即可
表示 10 个图标。

文件布局（共 2656 字节 = 0xA60，无任何填充位）
----------------------------------------------
    0x0000  5 × 512B   覆盖率图，32×32，4bpp，行优先，高半字节 = 左像素
    0x0A00  96B        调色板，2 组 × 16 级 × RGB888
                       0x0A00 + 0..47  普通态（0=纯底板，15=纯墨色）
                       0x0A30 + 0..47  激活态

用法
----
    python tools/gen_icons_bin.py            # 只生成 bin + 打印校验
    python tools/gen_icons_bin.py --check    # 额外输出解码对比图到 tools/out/

依赖：Pillow（pip install pillow）
"""
import argparse
import os
import sys

from PIL import Image

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "icons", "png32")
OUT = os.path.join(REPO, "icons", "icons32.bin")

W = H = 32
NAMES = ["display", "album", "files", "settings", "about"]

PLATE_N = (0xC0, 0xC0, 0xC0)
INK_N = (0x16, 0x16, 0x16)
PLATE_A = (0x30, 0x80, 0xFF)
INK_A = (0xFF, 0xFF, 0xFF)

LEVELS = 16
MASK_BYTES = W * H // 2                      # 512
PAL_OFF = len(NAMES) * MASK_BYTES            # 0x0A00


def coverage(px, plate, ink):
    """把两色抗锯齿像素投影到 plate→ink 连线上，返回覆盖率 0..1（1 = 墨色）。"""
    v = [ink[i] - plate[i] for i in range(3)]
    n2 = sum(x * x for x in v)
    if n2 == 0:
        return 0.0
    d = [px[i] - plate[i] for i in range(3)]
    return max(0.0, min(1.0, sum(d[i] * v[i] for i in range(3)) / n2))


def load_cov(name, state, plate, ink):
    path = os.path.join(SRC, "ico-%s%s.png" % (name, state))
    img = Image.open(path).convert("RGB")
    if img.size != (W, H):
        raise SystemExit("%s 尺寸是 %s，要求 %d×%d" % (path, img.size, W, H))
    corner = img.getpixel((0, 0))
    if max(abs(corner[i] - plate[i]) for i in range(3)) > 1:
        raise SystemExit("%s 左上角不是底板色 %s，而是 %s" % (path, plate, corner))
    return img, [[coverage(img.getpixel((x, y)), plate, ink) for x in range(W)] for y in range(H)]


def quantize(cov_n, cov_a):
    """两态覆盖率取均值后量化到 0..15，保证两态共用同一张图时误差对半分。"""
    return [[int(round((cov_n[y][x] + cov_a[y][x]) / 2.0 * (LEVELS - 1)))
             for x in range(W)] for y in range(H)]


def pack(levels):
    out = bytearray()
    for y in range(H):
        for x in range(0, W, 2):
            out.append((levels[y][x] << 4) | (levels[y][x + 1] & 0xF))
    return bytes(out)


def unpack(buf):
    lv = [[0] * W for _ in range(H)]
    for y in range(H):
        for bx in range(W // 2):
            b = buf[y * (W // 2) + bx]
            lv[y][bx * 2] = b >> 4
            lv[y][bx * 2 + 1] = b & 0xF
    return lv


def ramp(plate, ink):
    return [tuple(int(round(plate[i] + (ink[i] - plate[i]) * k / (LEVELS - 1)))
                  for i in range(3)) for k in range(LEVELS)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="输出解码对比图到 tools/out/")
    args = ap.parse_args()

    masks, decoded, half = [], [], []
    for name in NAMES:
        img_n, cov_n = load_cov(name, "", PLATE_N, INK_N)
        _, cov_a = load_cov(name, "-active", PLATE_A, INK_A)
        lv = quantize(cov_n, cov_a)
        masks.append(pack(lv))
        decoded.append((name, img_n, lv))
        half.append(sum(1 for y in range(H) for x in range(W) if lv[y][x] not in (0, LEVELS - 1)))

    pal = bytes(b for rgb in (ramp(PLATE_N, INK_N) + ramp(PLATE_A, INK_A)) for b in rgb)
    blob = b"".join(masks) + pal
    with open(OUT, "wb") as fp:
        fp.write(blob)

    print("输出 %s" % OUT)
    print("  共 %d 字节 (0x%X) = %d 张覆盖率图 × %d B + 调色板 %d B"
          % (len(blob), len(blob), len(NAMES), MASK_BYTES, len(pal)))
    print("\n  %-9s %-10s %-8s %s" % ("icon", "半调像素", "往返一致", "对原始 PNG 误差 最大/平均"))
    for idx, (name, img, lv) in enumerate(decoded):
        back = unpack(masks[idx])
        same = all(back[y][x] == lv[y][x] for y in range(H) for x in range(W))
        pale = ramp(PLATE_N, INK_N)
        mx, sm = 0, 0.0
        for y in range(H):
            for x in range(W):
                want, got = pale[lv[y][x]], img.getpixel((x, y))
                mx = max(mx, max(abs(got[i] - want[i]) for i in range(3)))
                sm += sum(abs(got[i] - want[i]) for i in range(3)) / 3.0
        print("  %-9s %-10d %-8s %d / %.1f"
              % (name, half[idx], "是" if same else "否", mx, sm / (W * H)))

    if args.check:
        Z = 6
        out_dir = os.path.join(REPO, "tools", "out")
        os.makedirs(out_dir, exist_ok=True)
        sheet = Image.new("RGB", (len(NAMES) * (W * Z + 10) + 10, H * Z * 2 + 30), (18, 18, 18))
        for idx, (name, img, lv) in enumerate(decoded):
            x0 = 10 + idx * (W * Z + 10)
            sheet.paste(img.resize((W * Z, H * Z), Image.NEAREST), (x0, 10))
            dec = Image.new("RGB", (W, H))
            pale = ramp(PLATE_N, INK_N)
            for y in range(H):
                for x in range(W):
                    dec.putpixel((x, y), pale[lv[y][x]])
            sheet.paste(dec.resize((W * Z, H * Z), Image.NEAREST), (x0, 20 + H * Z))
        p = os.path.join(out_dir, "icons_decode_check.png")
        sheet.save(p)
        print("\n对比图（上=原始 PNG，下=bin 解码）：%s" % p)


if __name__ == "__main__":
    main()
