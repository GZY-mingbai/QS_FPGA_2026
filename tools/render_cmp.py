import os, sys
from PIL import Image

T = os.path.expandvars(r"%LOCALAPPDATA%\Temp")

TEXTS = [
    "Language  Contrast  Saturate",
    "Volume 100%   Mute Off  Loop On",
    "Bright  Temp  Style  Default",
    "Spectrum  FitScr  AutoPlay",
]


def render(buf, zoom=3, bg=(16, 16, 16)):
    W = max(len(t) for t in TEXTS) * 16 + 8
    H = len(TEXTS) * 20 + 8
    img = Image.new("RGB", (W, H), bg)
    px = img.load()
    for li, t in enumerate(TEXTS):
        for ci, ch in enumerate(t):
            if ch == ' ':
                continue
            off = ord(ch) * 32
            for r in range(16):
                row = (buf[off + 2 * r] << 8) | buf[off + 2 * r + 1]
                for c in range(16):
                    if (row >> (15 - c)) & 1:
                        x = 4 + ci * 16 + c
                        y = 4 + li * 20 + r
                        if 0 <= x < W and 0 <= y < H:
                            px[x, y] = (255, 255, 255)
    return img.resize((W * zoom, H * zoom), Image.NEAREST)


old_bin = open(os.path.join(os.path.dirname(os.path.abspath(sys.argv[0])), "..", "Font", "font_gb2312.bin"), "rb").read() \
    if False else open(r"C:\Users\mingbai\Documents\Code\QS_FPGA\Font\font_gb2312.bin", "rb").read()
new_bin = open(os.path.join(T, "font_s26.bin"), "rb").read()

a = render(old_bin)
b = render(new_bin)

# 上下拼成一张对比图（上=旧，下=新）
gap = 12
out = Image.new("RGB", (a.width, a.height + gap + b.height), (60, 60, 60))
out.paste(a, (0, 0))
out.paste(b, (0, a.height + gap))
dst = os.path.join(T, "cmp_spacing.png")
out.save(dst)
print("written", dst, out.size)
