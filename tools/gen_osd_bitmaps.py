# -*- coding: utf-8 -*-
"""把导航图标 + PH1 logo 打包成「OSD 位图 ROM」的单一 bin。

为什么不再用 4bpp 覆盖率 + 调色板
----------------------------------
旧格式每个像素只存 0..15 的覆盖率，颜色靠 `osd_gfx.pal_rgb()` 查表。那个函数是
32 项 × 24bit 的常量 case，综合出来 ≈180 个 LUT —— 比它省下来的 BRAM 贵得多。
现在改成直出色彩：像素通路只剩「选半字 + 拼回 888」（纯连线），调色板整块删掉。

两种格式（故意的：图标不需要逐像素 alpha，logo 需要）
------------------------------------------------------
    图标  32×32 × 2 态 × 5 个，**RGB565，2 像素 / 32bit 字**（alpha 恒 255）
          —— 图标是一块不透明的"贴片"，本来就要盖住底下的面板
    logo  64×41，**ARGB8888，1 像素 / 32bit 字**
          —— 背景必须和左栏面板一样是 **75% (#0E0E0E, A=192)**。
             写成不透明的 0x0E0E0E 会变成一块实心矩形：面板下面有 25% 的视频透上来，
             实心的没有 —— 出图一眼就能看出一个方框（这一版就是这么被抓到的）。

文件布局（全部光栅序，低字节/低半字在前）
------------------------------------------
    word        内容
    0     .. 511   icon0 普通态  32×32（16bpp）
    512   .. 1023  icon0 激活态  32×32
    1024  .. 1535  icon1 普通态
    ...            （共 5 图标 × 2 态 = 5120 字）
    5120  .. 7743  logo 64×41（32bpp ARGB，2624 字）

地址（osd_gfx 里几乎全是连线）：
    图标 = {idx[2:0], act, ly[4:0], lx[4:1]}            → 0..5119，零 LUT
    logo = 5120 + {ly[5:0], lx[5:0]}                     → 5120..7743
         = {~(x[11]&x[10]), x[11]^x[10], ~x[10], x[9:0]}（x = ly*64+lx）
         5120 = 0x1400 = bit12 + bit10，而 x ≤ 0xA3F ⇒ 只有 t=1..3 位进位，**4 LUT**

logo 的 alpha 怎么来的
----------------------
目标观感 = "墨色按覆盖率 c 叠在左栏面板上"，而面板本身是 75% 的 #0E0E0E：

    P      = 0.75*0E0E0E + 0.25*V            （左栏面板，V = 视频）
    out    = c*ink + (1-c)*P
    等价单层 A/255*C + (1-A/255)*V
        ⇒ A = 192 + 63*c
        ⇒ C = [c*ink + (1-c)*0.75*0E0E0E] / (A/255)

c=0 ⇒ A=192、C=#0E0E0E（**和面板逐位相同**）；c=1 ⇒ A=255、C=ink。
中间值由 8× 超采样给出（64 级覆盖率，量化到 8 位）。

用法
----
    python tools/gen_osd_bitmaps.py            # 生成 bin + hex + 校验
    python tools/gen_osd_bitmaps.py --check    # 额外输出预览图到 tools/out/

依赖：Pillow
"""
import argparse
import os
import re

from PIL import Image, ImageDraw

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PNG32 = os.path.join(REPO, "icons", "png32")
SVG = os.path.join(REPO, "icons", "logo_ph1.svg")
OUT_BIN = os.path.join(REPO, "icons", "osd_bitmaps.bin")
# ★ hex 必须放在 OSD 的 tools/ 下 —— 仿真工作目录是 APP/OSD/，
#   testbench 用 `$readmemh("tools/osd_bitmaps.hex", ...)` 相对路径读它。
OUT_HEX = os.path.join(REPO, "src", "user_source", "hdl_source", "APP", "OSD",
                       "tools", "osd_bitmaps.hex")

ICON_NAMES = ["display", "album", "files", "settings", "about"]
ICON_W = ICON_H = 32
NICON_WORD = 512                       # 每张 32×32 = 1024 px = 512 字（16bpp）

LOGO_W, LOGO_H = 64, 41
LOGO_WORD_BASE = 5120                  # = 5 图标 × 2 态 × 512
NLOGO_WORD = LOGO_W * LOGO_H           # 2624（每像素 1 字，32bpp）

PANEL_RGB = (0x0E, 0x0E, 0x0E)         # 左栏 / 页面板（osd_widget style 1）
PANEL_A = 192                          # style 1 的 level 6 ⇒ 75%

SS = 8                                 # logo 超采样倍数（64 级覆盖率）


# ---------------------------------------------------------------------------
#  SVG：只支持本文件用到的 M / L / H / V / Z（绝对坐标）
# ---------------------------------------------------------------------------
def parse_svg(path):
    """返回 [(fill_rgb, [(x, y), ...]), ...]，按文档顺序（后者盖前者）。"""
    src = open(path, encoding="utf-8").read()
    out = []
    for m in re.finditer(r'<path\s+d="([^"]+)"\s+fill="(#[0-9A-Fa-f]{6})"', src):
        d, fill = m.group(1), m.group(2)
        pts, cur, start = [], (0.0, 0.0), (0.0, 0.0)
        for cmd, args in re.findall(r'([MLHVZmlhvz])([^MLHVZmlhvz]*)', d):
            nums = [float(v) for v in re.findall(r'-?\d*\.?\d+(?:[eE]-?\d+)?', args)]
            if cmd in "Mm":
                for i in range(0, len(nums), 2):
                    cur = (nums[i], nums[i + 1])
                    if i == 0:
                        start = cur
                    pts.append(cur)
            elif cmd in "Ll":
                for i in range(0, len(nums), 2):
                    cur = (nums[i], nums[i + 1])
                    pts.append(cur)
            elif cmd in "Hh":
                for v in nums:
                    cur = (v, cur[1])
                    pts.append(cur)
            elif cmd in "Vv":
                for v in nums:
                    cur = (cur[0], v)
                    pts.append(cur)
            elif cmd in "Zz":
                pts.append(start)
                cur = start
        rgb = tuple(int(fill[i:i + 2], 16) for i in (1, 3, 5))
        out.append((rgb, pts))
    if not out:
        raise SystemExit("%s 里没解析到 path" % path)
    return out


def render_logo():
    """超采样渲染 logo。

    返回 (cov, premul)：
        cov[y][x]   = 覆盖率 c（0..1）
        premul      = c*ink 的 RGB（把墨色画在黑底上再降采样，就是预乘色）
    """
    paths = parse_svg(SVG)
    W, H = LOGO_W * SS, LOGO_H * SS
    sx, sy = W / 48.0, H / 31.0                     # viewBox 0 0 48 31

    img_col = Image.new("RGB", (W, H), (0, 0, 0))    # 黑底画墨色 ⇒ 降采样 = c*ink
    img_msk = Image.new("RGB", (W, H), (0, 0, 0))    # 黑底画白  ⇒ 降采样 = c
    dcol, dmsk = ImageDraw.Draw(img_col), ImageDraw.Draw(img_msk)
    for rgb, pts in paths:
        poly = [(x * sx, y * sy) for (x, y) in pts]
        if len(poly) >= 3:
            dcol.polygon(poly, fill=rgb)
            dmsk.polygon(poly, fill=(255, 255, 255))

    pc = img_col.resize((LOGO_W, LOGO_H), Image.BOX)
    pm = img_msk.resize((LOGO_W, LOGO_H), Image.BOX)
    cov = [[pm.getpixel((x, y))[0] / 255.0 for x in range(LOGO_W)] for y in range(LOGO_H)]
    premul = [[pc.getpixel((x, y)) for x in range(LOGO_W)] for y in range(LOGO_H)]
    return cov, premul


# ---------------------------------------------------------------------------
#  RGB888 → RGB565（四舍五入，误差 ≤4/255，比旧的 4bpp 16 级还小）
# ---------------------------------------------------------------------------
def to565(rgb):
    r, g, b = rgb
    r5 = (r * 31 + 127) // 255
    g6 = (g * 63 + 127) // 255
    b5 = (b * 31 + 127) // 255
    return (r5 << 11) | (g6 << 5) | b5


def from565(v):
    r5, g6, b5 = (v >> 11) & 0x1F, (v >> 5) & 0x3F, v & 0x1F
    return (r5 * 255 // 31, g6 * 255 // 63, b5 * 255 // 31)


def logo_argb(cov, premul):
    """覆盖率 + 预乘墨色 → 源-over 合成到左栏面板后的 ARGB8888。

    A = 192 + 63*c
    C = [c*ink + (1-c)*0.75*panel] / (A/255)
    """
    out = []
    for y in range(LOGO_H):
        for x in range(LOGO_W):
            c = cov[y][x]
            a = int(round(PANEL_A + (255 - PANEL_A) * c))
            if a <= 0:
                a = 1
            k = a / 255.0
            rgb = []
            for i in range(3):
                num = premul[y][x][i] + (1.0 - c) * (PANEL_A / 255.0) * PANEL_RGB[i]
                rgb.append(max(0, min(255, int(round(num / k)))))
            out.append((a, rgb[0], rgb[1], rgb[2]))
    return out


# ---------------------------------------------------------------------------
#  打包
# ---------------------------------------------------------------------------
def pack_565(px):
    """16bpp 像素列表（光栅序）→ 32bit 字（低半字 = 偶像素）。"""
    if len(px) % 2:
        px = px + [0]
    return [px[i] | (px[i + 1] << 16) for i in range(0, len(px), 2)]


def build():
    icon_px, stat = [], []
    for name in ICON_NAMES:
        for act in (0, 1):
            p = os.path.join(PNG32, "ico-%s%s.png" % (name, "-active" if act else ""))
            img = Image.open(p).convert("RGB")
            if img.size != (ICON_W, ICON_H):
                raise SystemExit("%s 尺寸是 %s，要求 32×32" % (p, img.size))
            for y in range(ICON_H):
                for x in range(ICON_W):
                    icon_px.append(to565(img.getpixel((x, y))))
            stat.append((name + ("-act" if act else ""), img))
    assert len(icon_px) == LOGO_WORD_BASE * 2, len(icon_px)

    cov, premul = render_logo()
    logo = logo_argb(cov, premul)

    words = pack_565(icon_px)
    # logo：每像素 1 个 32bit 字，字 = {A,R,G,B}；文件字节序 B,G,R,A
    #   （loader 是 shf<={fl_data,shf[31:8]} ⇒ b0 落最低字节）
    for (a, r, g, b) in logo:
        words.append(b | (g << 8) | (r << 16) | (a << 24))
    assert len(words) == LOGO_WORD_BASE + NLOGO_WORD, len(words)

    blob = b"".join(w.to_bytes(4, "little") for w in words)
    return blob, words, icon_px, logo, stat, cov


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="输出预览图到 tools/out/")
    args = ap.parse_args()

    blob, words, icon_px, logo, stat, cov = build()

    with open(OUT_BIN, "wb") as fp:
        fp.write(blob)
    with open(OUT_HEX, "w", encoding="ascii") as fp:
        for b in blob:
            fp.write("%02x\n" % b)

    print("输出 %s" % OUT_BIN)
    print("  %d 字节 (0x%X) = %d 个 32bit 字" % (len(blob), len(blob), len(words)))
    print("  图标 %d 字（word 0..%d，RGB565 2px/字）" % (LOGO_WORD_BASE, LOGO_WORD_BASE - 1))
    print("  logo %d 字（word %d..%d，ARGB8888 1px/字）"
          % (NLOGO_WORD, LOGO_WORD_BASE, LOGO_WORD_BASE + NLOGO_WORD - 1))

    # ---- 关键统计 ----
    a_vals = [p[0] for p in logo]
    n_bg = sum(1 for a in a_vals if a == PANEL_A)
    n_op = sum(1 for a in a_vals if a == 255)
    print("\n  logo alpha：A=192（纯背景）%d，A=255（纯墨）%d，中间 %d"
          % (n_bg, n_op, len(a_vals) - n_bg - n_op))
    print("  背景像素颜色必须 = #0E0E0E：%s"
          % ("是" if all(tuple(logo[i][1:]) == PANEL_RGB
                         for i in range(len(logo)) if a_vals[i] == PANEL_A) else "否"))

    # ---- 往返自检 ----
    back = [int.from_bytes(blob[i:i + 4], "little") for i in range(0, len(blob), 4)]
    bad = 0
    for i in range(LOGO_WORD_BASE):
        if (back[i] & 0xFFFF) != icon_px[2 * i] or (back[i] >> 16) != icon_px[2 * i + 1]:
            bad += 1
    for i in range(NLOGO_WORD):
        a, r, g, b = logo[i]
        if back[LOGO_WORD_BASE + i] != (a << 24) | (r << 16) | (g << 8) | b:
            bad += 1
    print("  往返一致（图标 + logo 全区）：%s" % ("是" if bad == 0 else "否 %d" % bad))

    print("\n  关键样本")
    print("    word0        = 0x%08X  (icon0 普通态 px0/px1)" % words[0])
    print("    word%-8d= 0x%08X  (logo 左上角，应为 A=192 #0E0E0E)"
          % (LOGO_WORD_BASE, words[LOGO_WORD_BASE]))
    print("    word%-8d= 0x%08X  (logo 右下角)" % (LOGO_WORD_BASE + NLOGO_WORD - 1, words[-1]))

    if args.check:
        out_dir = os.path.join(REPO, "tools", "out")
        os.makedirs(out_dir, exist_ok=True)
        Z = 6
        sheet = Image.new("RGB", (10 * (32 * Z + 8) + 8, 32 * Z + LOGO_H * Z + 30),
                          PANEL_RGB)
        for i in range(10):
            sheet.paste(stat[i][1].resize((32 * Z, 32 * Z), Image.NEAREST),
                        (8 + i * (32 * Z + 8), 8))
        # logo：按"75% 面板 + 视频"的亮底色合成，验"没有方框"
        for mode, tag in ((0, "on_panel"), (1, "on_video")):
            canv = Image.new("RGB", (LOGO_W, LOGO_H))
            for y in range(LOGO_H):
                for x in range(LOGO_W):
                    v = (200, 140, 60) if mode else PANEL_RGB
                    a, r, g, b = logo[y * LOGO_W + x]
                    k = a / 255.0
                    canv.putpixel((x, y), tuple(
                        int(round(rr * k + vv * (1 - k)))
                        for rr, vv in zip((r, g, b), v)))
            sheet.paste(canv.resize((LOGO_W * Z, LOGO_H * Z), Image.NEAREST),
                        (8 + mode * (LOGO_W * Z + 8), 24 + 32 * Z))
        p = os.path.join(out_dir, "osd_bitmaps_check.png")
        sheet.save(p)
        print("\n预览图（下排：左=面板底、右=亮底 ⇒ 都不该出现矩形边界）：%s" % p)


if __name__ == "__main__":
    main()
