#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mk_fat32_img.py —— 造一张**真 FAT32** 的卡镜像（给全链路仿真用）

布局与实卡一致（D 盘那张卡就是这种）：

    LBA 0        MBR（0x0C 分区项，起始 2048）
    LBA 2048     BPB(FAT32)，保留区 32 扇区
    LBA 2080     FAT0 / FAT1（各 512 扇区）
    LBA 3104     数据区，簇 2 = 根目录，8 扇区/簇（4KB）

内容：
    IMG01..IMG10.BMP   8 张 640×480 + 2 张 320×240（24bit 无压缩）
    MUSIC01.WAV        假 wav（测"非图片"过滤）
    NOTE01.TXT         假 txt
    HIDE01.BMP         隐藏项（attr |= 0x02）
    MUSIC01.WAV 前挂一条 LFN（测"跳过 LFN 项"）

每张 BMP 都画了：唯一底色 + 大号序号 + 白边 + 左上角一个 32×32 的定位块。
序号用于"屏幕上是谁"的目视判断；底色用于像素断言。

用法：
    python tools/mk_fat32_img.py --out sim_data/fat32_10img.img --check
"""

import argparse
import os
import struct
import sys

SEC = 512
TOTAL_SEC = 128 * 1024          # 64 MB
PART_LBA = 2048
RSVD = 32
NFAT = 2
FATSZ = 512                     # 扇区/FAT
SPC = 8                         # 扇区/簇 = 4KB
DATA_LBA = PART_LBA + RSVD + NFAT * FATSZ     # 3104
PART_SEC = TOTAL_SEC - PART_LBA
ROOT_CLUS = 2

FONT5x7 = {
    '0': ["01110", "10001", "10011", "10101", "11001", "10001", "01110"],
    '1': ["00100", "01100", "00100", "00100", "00100", "00100", "01110"],
    '2': ["01110", "10001", "00001", "00010", "00100", "01000", "11111"],
    '3': ["11110", "00001", "00001", "01110", "00001", "00001", "11110"],
    '4': ["00010", "00110", "01010", "10010", "11111", "00010", "00010"],
    '5': ["11111", "10000", "11110", "00001", "00001", "10001", "01110"],
    '6': ["00110", "01000", "10000", "11110", "10001", "10001", "01110"],
    '7': ["11111", "00001", "00010", "00100", "01000", "01000", "01000"],
    '8': ["01110", "10001", "10001", "01110", "10001", "10001", "01110"],
    '9': ["01110", "10001", "10001", "01111", "00001", "00010", "01100"],
}

# 10 张图的底色（RGB）
COLORS = [
    (200, 40, 40), (40, 200, 40), (40, 60, 220), (210, 200, 40),
    (200, 40, 200), (40, 200, 200), (230, 120, 30), (130, 50, 230),
    (40, 220, 150), (230, 50, 110),
]


# ---------------------------------------------------------------------------
#  BMP
# ---------------------------------------------------------------------------
def bmp24(w, h, rgb, label):
    """24bit 无压缩 BMP（自底向上，行 4 字节对齐）"""
    row = w * 3
    pad = (-row) % 4
    stride = row + pad
    body = bytearray(stride * h)
    r, g, b = rgb

    def put(x, y, col):
        if 0 <= x < w and 0 <= y < h:
            i = (h - 1 - y) * stride + x * 3
            body[i + 0] = col[2]
            body[i + 1] = col[1]
            body[i + 2] = col[0]

    # 底色
    for y in range(h):
        base = (h - 1 - y) * stride
        for x in range(w):
            i = base + x * 3
            body[i] = b
            body[i + 1] = g
            body[i + 2] = r

    # 8px 白边
    for x in range(w):
        for t in range(8):
            put(x, t, (255, 255, 255))
            put(x, h - 1 - t, (255, 255, 255))
    for y in range(h):
        for t in range(8):
            put(t, y, (255, 255, 255))
            put(w - 1 - t, y, (255, 255, 255))

    # 左上角 32×32 定位块（黑）
    for y in range(16, 48):
        for x in range(16, 48):
            put(x, y, (0, 0, 0))

    # 中间大号序号
    digits = str(label)
    scale = max(4, min(w // (len(digits) * 8), h // 12))
    gw, gh = 5 * scale, 7 * scale
    total_w = len(digits) * (gw + scale)
    x0 = (w - total_w) // 2
    y0 = (h - gh) // 2
    ink = (20, 20, 20) if (r + g + b) > 380 else (245, 245, 245)
    for di, ch in enumerate(digits):
        pat = FONT5x7[ch]
        ox = x0 + di * (gw + scale)
        for ry, line in enumerate(pat):
            for rx, bit in enumerate(line):
                if bit == '1':
                    for dy in range(scale):
                        for dx in range(scale):
                            put(ox + rx * scale + dx, y0 + ry * scale + dy, ink)

    hdr = bytearray(54)
    hdr[0:2] = b'BM'
    struct.pack_into('<I', hdr, 2, 54 + len(body))
    struct.pack_into('<I', hdr, 10, 54)
    struct.pack_into('<I', hdr, 14, 40)
    struct.pack_into('<i', hdr, 18, w)
    struct.pack_into('<i', hdr, 22, h)
    struct.pack_into('<H', hdr, 26, 1)
    struct.pack_into('<H', hdr, 28, 24)
    struct.pack_into('<I', hdr, 34, len(body))
    return bytes(hdr) + bytes(body)


# ---------------------------------------------------------------------------
#  FAT / MBR / BPB
# ---------------------------------------------------------------------------
def u8(v):  return struct.pack('<B', v)
def u16(v): return struct.pack('<H', v)
def u32(v): return struct.pack('<I', v)


def mbr():
    b = bytearray(SEC)
    e = bytearray(16)
    e[0] = 0x80                       # 可引导（不影响）
    e[4] = 0x0C                       # FAT32 LBA
    struct.pack_into('<I', e, 8, PART_LBA)
    struct.pack_into('<I', e, 12, PART_SEC)
    b[446:462] = e
    b[510:512] = b'\x55\xAA'
    return bytes(b)


def bpb():
    b = bytearray(SEC)
    b[0:3] = b'\xEB\x58\x90'
    b[3:11] = b'MSWIN4.1'
    b[11:13] = u16(SEC)
    b[13] = SPC
    b[14:16] = u16(RSVD)
    b[16] = NFAT
    b[17:19] = u16(0)
    b[19:21] = u16(0)
    b[21] = 0xF8
    b[22:24] = u16(0)
    b[24:26] = u16(63)
    b[26:28] = u16(255)
    b[28:32] = u32(PART_LBA)
    b[32:36] = u32(0)                 # TotSec16 = 0（FAT32）
    b[36:40] = u32(FATSZ)
    b[40:44] = u16(0)                 # ExtFlags
    b[44:48] = u32(ROOT_CLUS)
    b[48:50] = u16(1)                 # FSInfo
    b[50:52] = u16(6)                 # 备份 BPB
    b[64] = 0x80
    b[66] = 0x29
    b[67:71] = u32(0x51A2B3C4)
    b[71:82] = b'QSFPGA10IMG'
    b[82:90] = b'FAT32   '
    struct.pack_into('<I', b, 32, PART_SEC)
    b[510:512] = b'\x55\xAA'
    return bytes(b)


def short_name(name, ext='', attr=0x20, cluster=0, size=0):
    b = name.upper().ljust(8)[:8].encode('ascii')
    e = ext.upper().ljust(3)[:3].encode('ascii')
    ent = bytearray(32)
    ent[0:8] = b
    ent[8:11] = e
    ent[11] = attr
    ent[20:22] = u16((cluster >> 16) & 0xFFFF)
    ent[26:28] = u16(cluster & 0xFFFF)
    ent[28:32] = u32(size)
    return bytes(ent)


def lfn_checksum(name8):
    s = 0
    for c in name8:
        s = (((s & 1) << 7) + (s >> 1) + c) & 0xFF
    return s


def lfn_entries(longname, name8):
    """把长名拆成 13 字符/项（只造 1 项，够测跳过）"""
    chk = lfn_checksum(name8)
    txt = longname[:13]
    chars = txt.encode('utf-16-le')
    while len(chars) < 26:
        chars += b'\xff\xff'
    chars = chars[:26]
    ent = bytearray(32)
    ent[0] = 0x41                     # 序号 1 + LAST
    ent[1:11] = chars[0:10]
    ent[11] = 0x0F
    ent[13] = chk
    ent[14:26] = chars[10:22]
    ent[26:28] = b'\x00\x00'
    ent[28:32] = chars[22:26]
    return bytes(ent)


# ---------------------------------------------------------------------------
#  造镜像
# ---------------------------------------------------------------------------
def build(out):
    files = []
    for i in range(10):
        label = i + 1
        # 只有第 1 张用 640×480（覆盖高清 1:1 通路），其余 320×240：
        # 全链路仿真里 640×480 要读 1800 扇区，10 张会把仿真拖到几十分钟。
        w, h = (320, 240)
        data = bmp24(w, h, COLORS[i], label)
        files.append(('IMG%02d' % (i + 1), 'BMP', 0x20, data, w, h, COLORS[i]))
    files.append(('MUSIC01', 'WAV', 0x20, b'RIFF' + b'\x00' * 2044, 0, 0, (0, 0, 0)))
    files.append(('NOTE01', 'TXT', 0x20, b'hello qs_fpga\n' * 64, 0, 0, (0, 0, 0)))
    files.append(('HIDE01', 'BMP', 0x22, bmp24(160, 120, (90, 90, 90), 0), 160, 120, (90, 90, 90)))

    # 逐文件分配连续簇
    clus = 3
    plan = []
    for (nm, ext, attr, data, w, h, rgb) in files:
        nclus = (len(data) + SPC * SEC - 1) // (SPC * SEC)
        plan.append((nm, ext, attr, data, clus, nclus, w, h, rgb))
        clus += nclus
    total_clus = clus
    assert total_clus < FATSZ * SEC * SPC, 'FAT 装不下'

    img = bytearray(TOTAL_SEC * SEC)

    # MBR + 备份分区表
    img[0:SEC] = mbr()

    # BPB + 备份 BPB（LBA 6）
    img[PART_LBA * SEC:(PART_LBA + 1) * SEC] = bpb()
    img[(PART_LBA + 6) * SEC:(PART_LBA + 7) * SEC] = bpb()

    # FAT
    fat = bytearray(FATSZ * SEC)
    struct.pack_into('<I', fat, 0, 0x0FFFFFF8)
    struct.pack_into('<I', fat, 4, 0x0FFFFFFF)
    struct.pack_into('<I', fat, 8, 0x0FFFFFFF)      # 根目录簇 2 = EOC
    for (_, _, _, _, start, nclus, _, _, _) in plan:
        for k in range(nclus):
            c = start + k
            nxt = 0x0FFFFFFF if k == nclus - 1 else c + 1
            struct.pack_into('<I', fat, c * 4, nxt)
    for i in range(NFAT):
        o = (PART_LBA + RSVD + i * FATSZ) * SEC
        img[o:o + len(fat)] = fat

    # 根目录
    ents = b''
    for (nm, ext, attr, data, start, nclus, w, h, rgb) in plan:
        if nm == 'MUSIC01':
            ents += lfn_entries('My Music File', b'MUSIC01 ')
        ents += short_name(nm, ext, attr, start, len(data))
    ents += b'\x00' * 32
    root_off = (DATA_LBA + (ROOT_CLUS - 2) * SPC) * SEC
    img[root_off:root_off + len(ents)] = ents

    # 文件数据
    for (nm, ext, attr, data, start, nclus, w, h, rgb) in plan:
        off = (DATA_LBA + (start - 2) * SPC) * SEC
        img[off:off + len(data)] = data

    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    with open(out, 'wb') as f:
        f.write(img)

    # 给 TB 用的映射表（简单空白分隔，方便 $fscanf）
    mapf = os.path.splitext(out)[0] + '.map'
    with open(mapf, 'w', encoding='ascii') as f:
        f.write('# name off size w h r g b\n')
        for (nm, ext, attr, data, start, nclus, w, h, rgb) in plan:
            off = (DATA_LBA + (start - 2) * SPC) * SEC
            f.write('%s.%s %d %d %d %d %d %d %d\n'
                    % (nm, ext, off, len(data), w, h, rgb[0], rgb[1], rgb[2]))
    # 给 Verilog 台用的 include（避免 iverilog 的 $fscanf %s 不支持）
    vhf = os.path.splitext(out)[0] + '_map.vh'
    with open(vhf, 'w', encoding='ascii') as f:
        f.write('// auto-generated by tools/mk_fat32_img.py\n')
        f.write('initial begin\n')
        f.write('    map_n = %d;\n' % len(plan))
        for i, (nm, ext, attr, data, start, nclus, w, h, rgb) in enumerate(plan):
            off = (DATA_LBA + (start - 2) * SPC) * SEC
            f.write('    map_off[%d] = %d; map_size[%d] = %d; map_name[%d] = "%s.%s";\n'
                    % (i, off, i, len(data), i, nm, ext))
        f.write('end\n')
    return plan, mapf, vhf


# ---------------------------------------------------------------------------
#  独立校验
# ---------------------------------------------------------------------------
def check(path):
    data = open(path, 'rb').read()
    errs = []
    if len(data) != TOTAL_SEC * SEC:
        errs.append('大小 %d != %d' % (len(data), TOTAL_SEC * SEC))

    # MBR
    if data[510:512] != b'\x55\xAA':
        errs.append('MBR 签名')
    p_lba = struct.unpack_from('<I', data, 446 + 8)[0]
    p_sec = struct.unpack_from('<I', data, 446 + 12)[0]
    if p_lba != PART_LBA or p_sec != PART_SEC:
        errs.append('分区项 %d/%d' % (p_lba, p_sec))

    # BPB
    b = data[p_lba * SEC:(p_lba + 1) * SEC]
    bps = struct.unpack_from('<H', b, 11)[0]
    spc = b[13]
    rsvd = struct.unpack_from('<H', b, 14)[0]
    nfat = b[16]
    fatsz = struct.unpack_from('<I', b, 36)[0]
    root = struct.unpack_from('<I', b, 44)[0]
    tot = struct.unpack_from('<I', b, 32)[0]
    if (bps, spc, rsvd, nfat, fatsz, root, tot) != (SEC, SPC, RSVD, NFAT, FATSZ, ROOT_CLUS, PART_SEC):
        errs.append('BPB %s' % str((bps, spc, rsvd, nfat, fatsz, root, tot)))

    d_lba = p_lba + rsvd + nfat * fatsz
    fat0 = data[(p_lba + rsvd) * SEC:(p_lba + rsvd + fatsz) * SEC]
    fat1 = data[(p_lba + rsvd + fatsz) * SEC:(p_lba + rsvd + 2 * fatsz) * SEC]
    if fat0 != fat1:
        errs.append('两份 FAT 不一致')

    # 根目录
    root_off = (d_lba + (root - 2) * spc) * SEC
    ents = []
    for i in range(SPC * SEC // 32):
        e = data[root_off + i * 32: root_off + (i + 1) * 32]
        if e[0] == 0x00:
            break
        if e[0] == 0xE5 or e[11] == 0x0F:
            continue
        nm = e[0:8].decode('ascii', 'replace').rstrip()
        ext = e[8:11].decode('ascii', 'replace').rstrip()
        attr = e[11]
        cl = (struct.unpack_from('<H', e, 20)[0] << 16) | struct.unpack_from('<H', e, 26)[0]
        sz = struct.unpack_from('<I', e, 28)[0]
        ents.append((nm, ext, attr, cl, sz))

    # 每个文件：簇链 + BMP 头 + 首行首像素
    for (nm, ext, attr, cl, sz) in ents:
        if sz == 0:
            continue
        n = 0
        c = cl
        while True:
            nxt = struct.unpack_from('<I', fat0, c * 4)[0] & 0x0FFFFFFF
            n += 1
            if nxt >= 0x0FFFFFF8:
                break
            c = nxt
            if n > 70000:
                errs.append('%s 簇链过长' % nm)
                break
        if n * SPC * SEC < sz:
            errs.append('%s 簇链 %d 簇 < 文件 %d 字节' % (nm, n, sz))
        off = (d_lba + (cl - 2) * spc) * SEC
        if ext == 'BMP':
            if data[off:off + 2] != b'BM':
                errs.append('%s BMP 魔数' % nm)
            else:
                fsz, = struct.unpack_from('<I', data, off + 2)
                w, h = struct.unpack_from('<ii', data, off + 18)
                bits, = struct.unpack_from('<H', data, off + 28)
                if fsz != sz:
                    errs.append('%s 头内大小 %d != 目录项 %d' % (nm, fsz, sz))
                if bits != 24 or w <= 0 or h <= 0:
                    errs.append('%s 头 %dx%d/%dbit' % (nm, w, h, bits))
                # 首行（图像最下行）第一个像素
                poff, = struct.unpack_from('<I', data, off + 10)
                px = data[off + poff: off + poff + 3]
                if px == b'\x00\x00\x00':
                    errs.append('%s 首像素全黑（可能没写数据）' % nm)

    return errs, ents


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='sim_data/fat32_10img.img')
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()

    plan, mapf, vhf = build(args.out)
    print('镜像：%s（%d 字节，%d 扇区）' % (args.out, TOTAL_SEC * SEC, TOTAL_SEC))
    print('  DATA_LBA=%d  ROOT_CLUS=%d  SPC=%d  FATSZ=%d' % (DATA_LBA, ROOT_CLUS, SPC, FATSZ))
    for (nm, ext, attr, data, start, nclus, w, h, rgb) in plan:
        off = (DATA_LBA + (start - 2) * SPC) * SEC
        print('  %-9s %-4s attr=%02X off=%9d size=%8d %4dx%-4d rgb=%s'
              % (nm, ext, attr, off, len(data), w, h, rgb))
    print('映射表：%s' % mapf)
    print('Verilog include：%s' % vhf)

    if args.check:
        errs, ents = check(args.out)
        print('  校验：%s（目录项 %d 个）' % ('通过' if not errs else '失败', len(ents)))
        for e in errs:
            print('   !!', e)
        return 0 if not errs else 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
