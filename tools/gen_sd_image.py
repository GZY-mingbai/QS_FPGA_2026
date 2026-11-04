#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gen_sd_image.py —— 生成仿真用的「SD 卡镜像」和缩小版 BMP

【为什么需要它】
阶段 7 的全链路仿真（SD/tb_full.v）里，SD 卡模型要从一个文件里读"卡的内容"。
真卡上是 FAT32 文件系统，仿真里不需要文件系统 —— 直接按**物理字节偏移**把
几张 BMP 摆进一个镜像文件即可，FPGA 侧也正好是按偏移读的
（top_tf_hdmi_audio.v 的 IMG0_OFFSET / IMG1_OFFSET）。

【两种用法】

1) 缩小版 BMP（给"快速全链路"用：64×48 只有 9KB，仿真几秒就能跑完）
       python tools/gen_sd_image.py small 源图.bmp 输出.bmp 64 48

2) 拼装卡镜像
       python tools/gen_sd_image.py build 卡镜像.bin 0:图A.bmp 922112:图B.bmp
   偏移必须 512 对齐（FPGA 侧 load_offset[31:9] 直接当扇区号用）。

【为什么不用 PIL】
本机 Python 环境没有 Pillow。24bit 无压缩 BMP 的读写和最近邻缩放一共就
几十行，自己写反而没有依赖、结果可复现。
"""

import struct
import sys


# ----------------------------------------------------------------------------
# BMP 读写（只支持 24 位不压缩，也就是本工程唯一接受的格式）
# ----------------------------------------------------------------------------
def read_bmp24(path):
    """返回 (w, h, rows)，rows[0] 是**图像最上面一行**的 RGB 字节串"""
    with open(path, "rb") as f:
        data = f.read()

    if data[0:2] != b"BM":
        raise SystemExit("%s 不是 BMP（魔数不是 BM）" % path)

    (pix_off,) = struct.unpack_from("<I", data, 0x0A)
    w, h = struct.unpack_from("<ii", data, 0x12)
    (bpp,) = struct.unpack_from("<H", data, 0x1C)
    (comp,) = struct.unpack_from("<I", data, 0x1E)

    if bpp != 24:
        raise SystemExit("%s 位深 = %d，本工程只支持 24 位" % (path, bpp))
    if comp != 0:
        raise SystemExit("%s 压缩方式 = %d，本工程只支持不压缩" % (path, comp))

    bottom_up = h > 0                      # 高度为正 = 常规的"自底向上"存储
    ah      = abs(h)
    stride  = ((w * 3 + 3) // 4) * 4       # 行按 4 字节对齐

    rows = []
    for r in range(ah):
        base = pix_off + r * stride
        row = bytearray()
        for c in range(w):
            b, g, rr = data[base + c * 3], data[base + c * 3 + 1], data[base + c * 3 + 2]
            row += bytes((rr, g, b))       # BMP 存的是 BGR，转成 RGB 方便处理
        rows.append(bytes(row))

    if bottom_up:
        rows.reverse()                     # 统一成"第 0 行 = 图像顶行"
    return w, ah, rows


def write_bmp24(path, w, h, rows):
    """rows[0] 必须是图像最上面一行；写成标准 24 位不压缩 BMP（自底向上）"""
    stride = ((w * 3 + 3) // 4) * 4
    pad    = bytes(stride - w * 3)
    pix    = bytearray()
    for row in reversed(rows):
        # RGB → BGR
        for c in range(w):
            pix += bytes((row[c * 3 + 2], row[c * 3 + 1], row[c * 3]))
        pix += pad

    (file_size,) = (54 + len(pix),)
    hdr = bytearray()
    hdr += b"BM"
    hdr += struct.pack("<IHHI", file_size, 0, 0, 54)          # 文件头
    hdr += struct.pack("<IiiHHIIiiII",                         # 信息头
                       40, w, h, 1, 24, 0, len(pix), 2835, 2835, 0, 0)
    with open(path, "wb") as f:
        f.write(hdr)
        f.write(pix)
    return file_size


def downsample_nearest(w, h, rows, nw, nh):
    """最近邻缩放（和 FPGA 侧 ps_gen_image 的取点方式一致，便于对账）"""
    out = []
    for r in range(nh):
        sy = (r * h) // nh
        src = rows[sy]
        row = bytearray()
        for c in range(nw):
            sx = (c * w) // nw
            row += src[sx * 3: sx * 3 + 3]
        out.append(bytes(row))
    return out


# ----------------------------------------------------------------------------
# 卡镜像：按 512 对齐的物理偏移把若干文件摆进去，其余位置是 0xFF（擦除态）
# ----------------------------------------------------------------------------
def build_card(out_path, items, total_size=None):
    blob = bytearray(b"\xFF" * (total_size or 0))
    for off, path in items:
        if off % 512:
            raise SystemExit("偏移 %d 不是 512 的倍数" % off)
        with open(path, "rb") as f:
            data = f.read()
        need = off + len(data)
        if len(blob) < need:
            blob += b"\xFF" * (need - len(blob))
        blob[off:off + len(data)] = data
        print("  偏移 %-8d ← %-60s (%d 字节)" % (off, path, len(data)))
    with open(out_path, "wb") as f:
        f.write(blob)
    print("  卡镜像 %s：%d 字节（%.2f MB）" % (out_path, len(blob), len(blob) / 1048576.0))


def build_tex_hex(out_path, bmp, flip=False):
    """把一张 BMP 转成"纹理区应该有的一串 32bit 字"（$readmemh 用）

    这和 sd_bmp_loader + bmp_read 搬完之后 SDRAM 里的内容必须**完全一致**：
        字 = {8'hFF, R, G, B}，第 dst 行 = 图像第 dst 行

    ★ flip 默认 False（也就是不要翻转）★
      因为 BMP 文件里是"自底向上"存的，而装载器在写地址上做了行翻转，
      最终 SDRAM 里的纹理行序 = 图像的自然行序（第 0 行 = 图最上面一行）。
      这一条已由 tools/check_full_chain.py tex 实测确认（"行不翻+RGB" 100% 命中）。
      一开始想当然地又翻了一次，那才会得到上下颠倒的图。
    """
    w, h, rows = read_bmp24(bmp)
    with open(out_path, "w") as f:
        for r in range(h):
            src = rows[h - 1 - r] if flip else rows[r]
            for c in range(w):
                R, G, B = src[c * 3], src[c * 3 + 1], src[c * 3 + 2]
                f.write("%08X\n" % (0xFF000000 | (R << 16) | (G << 8) | B))
    print("  %s → %s（%dx%d = %d 字）" % (bmp, out_path, w, h, w * h))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    mode = sys.argv[1]

    if mode == "small":
        src, dst, nw, nh = sys.argv[2], sys.argv[3], int(sys.argv[4]), int(sys.argv[5])
        w, h, rows = read_bmp24(src)
        nrows = downsample_nearest(w, h, rows, nw, nh)
        size = write_bmp24(dst, nw, nh, nrows)
        print("  %s (%dx%d) → %s (%dx%d, %d 字节)" % (src, w, h, dst, nw, nh, size))
        return 0

    if mode == "tex":
        # tex 输出.hex 源图.bmp [--noflip]
        out_path, bmp = sys.argv[2], sys.argv[3]
        flip = ("--flip" in sys.argv[4:])
        build_tex_hex(out_path, bmp, flip)
        return 0

    if mode == "build":
        out_path = sys.argv[2]
        items = []
        for spec in sys.argv[3:]:
            off_s, path = spec.split(":", 1)
            items.append((int(off_s), path))
        build_card(out_path, items)
        return 0

    print(__doc__)
    return 1


if __name__ == "__main__":
    sys.exit(main())
