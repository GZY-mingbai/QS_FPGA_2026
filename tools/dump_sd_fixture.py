#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dump_sd_fixture.py —— 从真实的 SD 卡导出「仿真素材」

给 FPGA 的 fat_dir_lister 做**真实数据**的离线仿真用。导出的东西：

    sim_data/sd.hex        一张"稀疏卡镜像"：只放三个窗口
                             · 0 号扇区（MBR）
                             · 卷首扇区（BPB，绝对 LBA = 分区起始）
                             · 根目录那十几个扇区
                           不做全卡镜像（32GB 没法读）。
                           地址压缩方式见下，tb 里用同一个公式。

    sim_data/expect.txt    期望值：算出来的 ROOT_LBA / FAT_LBA / SPC_SHIFT，
                           以及前几个文件的名字 + 起始簇 + 起始扇区。

【地址压缩】
真实扇区号跨度太大（0、2048、17224..17239），全存会浪费。本脚本按**段**存：
格式（每行 = 16 字节的十六进制）：

    // SEG <lba> <nsec>        段头：这段从绝对 lba 开始，nsec 个扇区
    <16 字节一行...>

【★ 两个容易踩的坑（本版已修）★】
  ① **盘符不是整卡**：`\\.\D:` 的偏移 0 = 分区的 boot 扇区，不是卡的 0 号扇区。
     所以必须先用 IOCTL 问出"这个卷在物理盘上的起始字节"（不需要管理员），
     把它换算成 part_lba，再按"绝对 LBA"写段头。
  ② **FAT32 目录项的起始簇是高字在前**：offset 20 是**高** 16 位、offset 26 是
     **低** 16 位（与 FAT12/16 相反）。RTL 里也要按这个顺序拼。

【MBR 可能拿不到】
  读 `\\.\PhysicalDriveN` 需要管理员权限；拿不到时本脚本**合成**一个最小 MBR：
  分区表第 1 项（偏移 446，起始 LBA 在 454）填 IOCTL 给出的真实 part_lba，
  末尾 0x55AA。除了这两处，其余字节为 0 —— 对"是不是 FAT32 / part_lba 是多少"
  这两个判断完全够用（真实 MBR 的引导代码与这两件事无关）。
  expect.txt 里会写 `MBR_SYNTH 1` 标明这一点。

用法：
    python tools/dump_sd_fixture.py --dev D: -o sim_data
    python tools/dump_sd_fixture.py --dev \\.\D: -o sim_data
    python tools/dump_sd_fixture.py --dev \\.\PhysicalDrive2 -o sim_data   （要管理员）
"""
import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sd_fat_offset import win_volume_map, RawDev, u16, u32, decode_short_name   # noqa: E402

SEC = 512


def open_volume(dev):
    """打开卷/物理盘。返回 (RawDev, part_lba, synth_note)"""
    note = ''
    part_lba = 0

    if dev.startswith('\\\\.\\PhysicalDrive'):
        rd = RawDev(dev)
        # 物理盘：0 号扇区就是 MBR，part_lba 从分区表读
        mbr = rd.read(0, SEC)
        if len(mbr) == SEC and mbr[82:87] != b'FAT32':
            part_lba = u32(mbr, 454)
            note = '物理盘，MBR 是真读的'
        else:
            note = '物理盘，0 号扇区就是 BPB（superfloppy）'
        return rd, part_lba, note

    letter = dev.replace('\\\\.\\', '').replace(':', '')
    info = win_volume_map(letter)
    if info and info[2] is not None:
        part_lba = info[2] // SEC
    rd = RawDev('\\\\.\\%s:' % letter)
    note = '卷句柄（分区起始 %d 扇区，来自 IOCTL）' % part_lba
    return rd, part_lba, note


def make_mbr(part_lba):
    m = bytearray(SEC)
    m[0] = 0xFA
    # 分区项 1：偏移 446，起始 LBA 在 +8 = 454
    m[446 + 4] = 0x0C                      # 类型：FAT32 LBA
    struct.pack_into('<I', m, 454, part_lba)
    m[510] = 0x55
    m[511] = 0xAA
    return bytes(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dev', required=True)
    ap.add_argument('-o', '--out', default='sim_data')
    a = ap.parse_args()

    rd, part_lba, note = open_volume(a.dev)
    print('设备：%s  （%s）' % (a.dev, note))

    # ---- 卷首扇区 = BPB（卷内偏移 0）----
    vol0 = rd.read(0, SEC)
    if len(vol0) < SEC:
        sys.exit('读卷首扇区失败')

    # ---- MBR（绝对 0 号扇区）----
    #   ★ 判据是「分区起始 == 0」而不是「卷首扇区像不像 BPB」：
    #     卷句柄的 0 号扇区**永远是** BPB（Windows 已经把分区偏移扣掉了）。
    synth = 0
    if part_lba == 0:
        part_lba = 0
        mbr = vol0
        print('0 号扇区就是 BPB（superfloppy）⇒ part_lba = 0')
    else:
        mbr = None
        try:
            disk_no = None
            import re
            m = re.search(r'PhysicalDrive(\d+)', a.dev)
            if m:
                disk_no = int(m.group(1))
            if disk_no is not None:
                prd = RawDev('\\\\.\\PhysicalDrive%d' % disk_no)
                blk = prd.read(0, SEC)
                prd.close()
                if len(blk) == SEC and u32(blk, 454) != 0:
                    mbr = blk
                    part_lba = u32(blk, 454)
                    print('MBR 从物理盘读到：part_lba = %d' % part_lba)
        except OSError:
            mbr = None
        if mbr is None:
            mbr = make_mbr(part_lba)
            synth = 1
            print('拿不到物理盘（要管理员）⇒ 合成最小 MBR，part_lba = %d' % part_lba)

    # ---- 解析 BPB ----
    bps      = u16(vol0, 11)
    spc      = vol0[13]
    reserved = u16(vol0, 14)
    nfat     = vol0[16]
    fatsz16  = u16(vol0, 22)
    fatsz32  = u32(vol0, 36)
    rootclus = u32(vol0, 44)
    fatsz    = fatsz16 if fatsz16 else fatsz32
    if bps != SEC:
        print('⚠ 扇区大小 = %d（本工程按 512 走）' % bps)
    if fatsz16:
        print('⚠ fatsz16 != 0 ⇒ 这其实是 FAT12/16，不是 FAT32')

    # 卷内（相对分区）与卡上（绝对）两套编号
    fat_lba_v  = reserved
    data_lba_v = reserved + nfat * fatsz
    fat_lba    = part_lba + fat_lba_v
    data_lba   = part_lba + data_lba_v
    spc_shift  = spc.bit_length() - 1
    root_lba   = data_lba                     # FAT32 根目录簇 = 2

    print('扇区/簇 %d  簇大小 %d  保留 %d  FAT 数 %d  FAT 大小 %d'
          % (bps, spc, reserved, nfat, fatsz))
    print('part_lba %d → FAT_LBA %d  ROOT_LBA %d  SPC_SHIFT %d'
          % (part_lba, fat_lba, root_lba, spc_shift))
    if rootclus != 2:
        print('⚠ 根目录簇 = %d（不是 2）—— 本工程假设根目录簇恒为 2' % rootclus)

    # ---- 根目录：读一个簇（最多 16 扇区，够本工程 MAX_SECS 用）----
    nsec = min(spc, 16)
    root = rd.read(data_lba_v * SEC, nsec * SEC)
    if len(root) < nsec * SEC:
        print('⚠ 根目录只读到 %d 字节' % len(root))

    # ---- 解析可见项（规则与 RTL 一致）----
    files = []
    for i in range(0, len(root) - 31, 32):
        e = root[i:i + 32]
        if e[0] == 0x00:
            break
        if e[0] == 0xE5:
            continue
        attr = e[11]
        if attr == 0x0F or (attr & 0x08) or (attr & 0x10) or (attr & 0x02):
            continue
        fname = decode_short_name(e)          # "APPLE.BMP"
        clus = (u16(e, 20) << 16) | u16(e, 26)      # ★ FAT32：高字在 20、低字在 26
        sec = data_lba + (clus - 2) * spc if clus >= 2 else 0
        files.append((fname, clus, sec))
        if len(files) >= 8:
            break

    os.makedirs(a.out, exist_ok=True)

    # ---- 写稀疏镜像 ----
    segs = [(0, 1, mbr)]
    if part_lba:
        segs.append((part_lba, 1, vol0))
    segs.append((root_lba, nsec, root))

    with open(os.path.join(a.out, 'sd.hex'), 'w') as f:
        for lba, ns, data in segs:
            f.write('// SEG %d %d\n' % (lba, ns))
            for s in range(ns):
                blk = data[s * SEC:(s + 1) * SEC]
                for k in range(0, SEC, 16):
                    f.write(''.join('%02x' % c for c in blk[k:k + 16]) + '\n')

    lines = ['ROOT_LBA %d' % root_lba,
             'FAT_LBA %d' % fat_lba,
             'SPC_SHIFT %d' % spc_shift,
             'PART_LBA %d' % part_lba,
             'NFILES %d' % len(files),
             'MBR_SYNTH %d' % synth]
    for nm, c, sec in files:
        lines.append('FILE %s %d %d' % (nm, c, sec))
    with open(os.path.join(a.out, 'expect.txt'), 'w') as f:
        f.write('\n'.join(lines) + '\n')

    print('\n导出 %s/sd.hex（%d 段）' % (a.out, len(segs)))
    for nm, c, sec in files:
        print('  %-14s clus=%-6d 扇区 %d' % (nm, c, sec))
    print('导出 %s/expect.txt' % a.out)


if __name__ == '__main__':
    main()
