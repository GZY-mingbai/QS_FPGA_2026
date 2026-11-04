# -*- coding: utf-8 -*-
"""生成一个最小 FAT32 镜像，验证 sd_fat_offset.py --rootdir 的输出。

布局（全部按扇区计）：
    0        ~ 32        保留区（含 BPB）
    32       ~ 32+2*512  FAT0 / FAT1
    1056     ~ ...       数据区，簇 2 = 根目录

目录项故意造得"脏"：有 LFN、有已删、有隐藏、有结束标记。
期望：只数出 3 个可见文件，隐藏/已删/LFN 各自计数正确。
"""
import os
import struct
import sys

SEC = 512
TOTAL_SEC = 32768          # 16 MB
RSVD = 32
NFAT = 2
FATSZ = 512
SPC = 8                    # 8 扇区/簇 = 4 KB
ROOT_CLUS = 2
DATA_START = RSVD + NFAT * FATSZ       # 1056

def u8(v):  return struct.pack('<B', v)
def u16(v): return struct.pack('<H', v)
def u32(v): return struct.pack('<I', v)

def short_name(base, ext='', attr=0x20, first=0, size=0):
    """base ≤ 8 字符，ext ≤ 3 字符"""
    b = base.upper().ljust(8)[:8].encode('ascii')
    e = ext.upper().ljust(3)[:3].encode('ascii')
    ent = bytearray(32)
    ent[0:8] = b
    ent[8:11] = e
    ent[11] = attr
    ent[20:22] = u16((first >> 16) & 0xFFFF)
    ent[26:28] = u16(first & 0xFFFF)
    ent[28:32] = u32(size)
    return bytes(ent)

def lfn_entry(seq, text, last=False):
    """一条 LFN 项（13 个 UTF-16 字符，散在三段）"""
    chars = text.encode('utf-16-le')
    while len(chars) < 26:
        chars += b'\xff\xff'
    chars = chars[:26]
    ent = bytearray(32)
    ent[0] = (seq & 0x3F) | (0x40 if last else 0)
    ent[1:11] = chars[0:10]
    ent[11] = 0x0F
    ent[13] = 0
    ent[14:26] = chars[10:22]
    ent[26:28] = b'\x00\x00'
    ent[28:32] = chars[22:26]
    return bytes(ent)

def build_bpb():
    b = bytearray(SEC)
    b[0:3] = b'\xEB\x58\x90'
    b[3:11] = b'MSWIN4.1'
    b[11:13] = u16(SEC)
    b[13] = SPC
    b[14:16] = u16(RSVD)
    b[16] = NFAT
    b[17:19] = u16(0)              # root_ent = 0 → FAT32
    b[19:21] = u16(0)
    b[21] = 0xF8
    b[22:24] = u16(0)
    b[24:26] = u16(63)
    b[26:28] = u16(255)
    b[32:36] = u32(TOTAL_SEC)
    b[36:40] = u32(FATSZ)
    b[44:48] = u32(ROOT_CLUS)
    b[48:50] = u16(1)
    b[50:52] = u16(6)
    b[64] = 0x80
    b[66] = 0x29
    b[67:71] = u32(0x12345678)
    b[71:82] = b'TESTCARD   '
    b[82:90] = b'FAT32   '
    b[510:512] = b'\x55\xAA'
    return bytes(b)

def main(out='tools/out/test_fat32.img'):
    os.makedirs(os.path.dirname(out), exist_ok=True)
    img = bytearray(TOTAL_SEC * SEC)

    # ---- BPB ----
    img[0:SEC] = build_bpb()

    # ---- FAT0 / FAT1：簇 2（根目录）标记结束 ----
    fat = bytearray(FATSZ * SEC)
    fat[0:4] = u32(0x0FFFFFF8)      # 媒体描述
    fat[4:8] = u32(0x0FFFFFFF)      # 簇 1（保留）
    fat[8:12] = u32(0x0FFFFFFF)     # 簇 2（根目录，结束）
    fat[12:16] = u32(0x0FFFFFFF)    # 簇 3（APPLE）
    fat[16:20] = u32(0x0FFFFFFF)    # 簇 4（MUSIC001）
    fat[20:24] = u32(0x0FFFFFFF)    # 簇 5（NOTE0001）
    for i in range(NFAT):
        off = (RSVD + i * FATSZ) * SEC
        img[off:off + len(fat)] = fat

    # ---- 根目录（簇 2） ----
    ents = b''
    ents += short_name('APPLE', 'BMP', 0x20, 3, 1000)
    ents += lfn_entry(1, 'My Music File', last=True)
    ents += short_name('MUSIC001', 'WAV', 0x20, 4, 2000)
    ents += short_name('NOTE0001', 'TXT', 0x20, 5, 300)
    ents += b'\xE5' + short_name('DELETED', 'BMP')[1:]   # 已删
    ents += short_name('HIDDEN', 'BMP', 0x22, 6, 400)    # 隐藏
    ents += b'\x00' * 32                                  # 结束标记
    ents = ents.ljust(SPC * SEC, b'\x00')

    off = DATA_START * SEC
    img[off:off + len(ents)] = ents

    with open(out, 'wb') as f:
        f.write(img)

    print('镜像已生成：%s（%d 字节，%d 扇区）' % (out, len(img), TOTAL_SEC))
    print('  期望：ROOT_LBA=%d  FAT_LBA=%d  SPC_SHIFT=%d  CONTIG=1'
          % (DATA_START, RSVD, SPC.bit_length() - 1))
    print('  期望：文件 3（BMP 1 / WAV 1 / TXT 1），已删 1，LFN 1，隐藏 1')
    return out

if __name__ == '__main__':
    main(*sys.argv[1:])
