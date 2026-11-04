#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
sd_fat_offset.py —— 读 TF 卡上的 FAT/exFAT 目录表，算出 BMP 文件的**物理字节偏移**

==============================================================================
【它解决什么问题】
==============================================================================
顶层 top_tf_hdmi_audio.v 里这些参数现在是占位的 0：
        parameter [31:0] IMG0_OFFSET = 32'd0;
        parameter [31:0] IMG1_OFFSET = 32'd0;
        parameter [31:0] IMG2_OFFSET = 32'd0;
偏移不对 → 装载器读到的"第 0 扇区"其实是 MBR，文件头校验（必须 "BM" +
640×480 + 24bit + 不压缩）必然失败 → 报 err → 图片区显示 SDRAM 里的乱码。
本脚本把 2~3 张 BMP 在卡里的真实位置算出来，直接给出可以粘贴的参数值。

==============================================================================
【为什么要自己解析 FAT —— 为什么不直接问操作系统】
==============================================================================
Windows 的文件 API 只给"文件内容"，不给"这个文件在盘上是第几个扇区"。
而 FPGA 的 SD 驱动是按**原始扇区号**读卡的（扇区 0 = MBR 引导区），
它完全不认识文件系统。唯一可靠的办法：自己找到 FAT 表，顺着簇链把文件的
物理位置算出来。

==============================================================================
【★ 文件必须在卡上连续（不碎片）★】
==============================================================================
装载器 sd_bmp_loader 是**线性**读的：从 offset 开始，一个扇区一个扇区往后搬
load_length/3 个像素 —— 它根本不知道 FAT 的存在。所以文件必须在卡上占**一整段
连续的簇**，否则搬到一半就会读到别的文件的内容（花屏、串图、上下半张不一样）。
本脚本会检查簇链是否连续，不连续会大声警告并告诉你怎么处理。

==============================================================================
【怎么用】
==============================================================================
    # --- Windows（裸读设备要管理员权限，普通权限打不开 \\.\E:）---
    python tools/sd_fat_offset.py --list                  # 先看卡是哪个盘
    python tools/sd_fat_offset.py                         # 只有一个可移动盘时自动选它
    python tools/sd_fat_offset.py --dev \\.\E:            # 指定盘符（卷）
    python tools/sd_fat_offset.py --dev \\.\PhysicalDrive2 # 指定物理盘（含 MBR）

    # --- Linux / macOS ---
    sudo python3 tools/sd_fat_offset.py --dev /dev/sdb

    # --- 或者先导出镜像再离线分析（不需要任何权限）---
    python tools/sd_fat_offset.py --image card.img

    # --- 算完直接写回顶层 ---
    python tools/sd_fat_offset.py --apply src/user_source/hdl_source/top_tf_hdmi_audio.v

==============================================================================
【它会自己验算 —— 这一步才是让人敢信的地方】
==============================================================================
算完偏移之后，脚本会做**逐字节自检**：
    左边 = 顺着 FAT 簇链读出来的文件开头 512 字节
    右边 = 直接按"算出来的物理偏移"裸读的 512 字节
两者完全一致才敢说这个偏移是对的。这一步能同时挡住：FAT 解析写错、
分区起始算错、字节序搞反、簇大小算错…… 任何一环错了，这里都会对不上。

==============================================================================
"""
import argparse
import json
import os
import re
import struct
import sys

SECTOR = 512                    # 逻辑扇区（TF 卡一律 512）
BMP_HDR_LEN = 54                # 标准 BMP 文件头长度
EXPECT_W = 640                  # 工程期望的纹理宽（和 loader 的 IMG_W 一致）
EXPECT_H = 480                  # 工程期望的纹理高（和 loader 的 IMG_H 一致）
EXPECT_BPP = 24
MAX_DIR_BYTES = 64 << 20        # 目录/簇链走飞了的保护上限
ATTR_READ_ONLY, ATTR_HIDDEN, ATTR_SYSTEM = 0x01, 0x02, 0x04
ATTR_VOLUME_ID, ATTR_DIRECTORY, ATTR_ARCHIVE = 0x08, 0x10, 0x20
ATTR_LFN = 0x0F


#==============================================================================
#  小工具
#==============================================================================
def u8(b, o):
    return b[o]


def u16(b, o):
    return struct.unpack_from('<H', b, o)[0]


def u32(b, o):
    return struct.unpack_from('<I', b, o)[0]


def u64(b, o):
    return struct.unpack_from('<Q', b, o)[0]


def sizeof_str(n):
    """把字节数写成人类看得懂的样子"""
    for unit in ('B', 'KB', 'MB', 'GB'):
        if n < 1024 or unit == 'GB':
            return ('%d %s' % (n, unit)) if unit == 'B' else ('%.1f %s' % (n, unit))
        n /= 1024.0


def decode_short_name(raw):
    """8.3 短名 → 可读字符串（处理 0x05 → 0xE5 这个历史怪癖）"""
    base = bytearray(raw[0:8])
    ext = bytearray(raw[8:11])
    if base and base[0] == 0x05:
        base[0] = 0xE5
    s = bytes(base).decode('cp437', 'replace').rstrip(' ')
    e = bytes(ext).decode('cp437', 'replace').rstrip(' ')
    return (s + '.' + e) if e else s


#==============================================================================
#  裸设备读：按对齐读、带缓存
#==============================================================================
class RawDev(object):
    """能从"某个字节偏移"读若干字节的设备/文件。

    ★ 为什么要按 512 对齐读 ★
      Windows 上对卷/物理盘的裸读（ReadFile）要求偏移和长度都是扇区的整数倍，
      随便读 54 字节会直接失败。所以这里把请求向下/向上扩到 512 的整数倍，
      读完再把需要的那一段切出来。
    """

    def __init__(self, path, align=SECTOR):
        self.path = path
        self.align = align
        flags = os.O_RDONLY | getattr(os, 'O_BINARY', 0)
        self.fd = os.open(path, flags)
        self.size = None
        try:
            self.size = os.lseek(self.fd, 0, os.SEEK_END)
            os.lseek(self.fd, 0, os.SEEK_SET)
        except OSError:
            self.size = None            # 裸设备拿不到大小是正常的
        self._cache = {}

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass

    def read(self, off, n):
        if n <= 0:
            return b''
        a = self.align
        start = (off // a) * a
        end = ((off + n + a - 1) // a) * a
        if start in self._cache and len(self._cache[start]) >= (end - start):
            blk = self._cache[start]
        else:
            os.lseek(self.fd, start, os.SEEK_SET)
            blk = b''
            need = end - start
            while len(blk) < need:
                chunk = os.read(self.fd, need - len(blk))
                if not chunk:
                    break
                blk += chunk
            self._cache.clear()          # 只留一份，别把整卡吃进内存
            self._cache[start] = blk
        return blk[off - start: off - start + n]

    def read_sector(self, lba, nsec=1):
        return self.read(lba * SECTOR, nsec * SECTOR)

    def dev_size(self):
        return self.size


#==============================================================================
#  Windows：查"盘符 ↔ 物理盘 + 分区在盘上的字节偏移"
#==============================================================================
def win_volume_map(letter):
    r"""返回 (物理盘号, 分区号, 该分区在物理盘上的起始字节)

    【为什么要这个】卡上的物理偏移 = 分区在卡上的起始字节 + 分区内偏移。
      盘符 \\.\E: 的裸读地址是**分区内**的（扇区 0 就是 BPB），少算了分区起始；
      而 FPGA 按整卡扇区号读，必须把这段补上。
    实现走两个 IOCTL（都是查询，不需要管理员）：
      IOCTL_STORAGE_GET_DEVICE_NUMBER  → 这个卷在哪个 PhysicalDrive 的第几个分区
      FSCTL_VOLUME_GET_VOLUME_DISK_EXTENTS → 这个卷在物理盘上的起始字节
    """
    if os.name != 'nt':
        return None
    import ctypes
    from ctypes import wintypes
    k32 = ctypes.windll.kernel32
    INVALID = ctypes.c_void_p(-1).value
    k32.CreateFileW.restype = ctypes.c_void_p
    h = k32.CreateFileW(ctypes.c_wchar_p('\\\\.\\%s:' % letter), 0,
                        0x00000001 | 0x00000002, None, 3, 0, None)
    if h in (None, INVALID):
        return None
    try:
        # ---- 卷 → 物理盘号 / 分区号 ----
        buf = ctypes.create_string_buffer(16)
        got = wintypes.DWORD()
        ok = k32.DeviceIoControl(ctypes.c_void_p(h), 0x2D1080, None, 0,
                                 buf, 16, ctypes.byref(got), None)
        if not ok:
            return None
        disk_no = u32(buf.raw, 4)
        part_no = u32(buf.raw, 8)
        # ---- 卷在物理盘上的起始字节 ----
        buf2 = ctypes.create_string_buffer(8 + 24 * 8)
        ok = k32.DeviceIoControl(ctypes.c_void_p(h), 0x00560000, None, 0,
                                 buf2, len(buf2), ctypes.byref(got), None)
        if not ok:
            return (disk_no, part_no, None)
        n = u32(buf2.raw, 0)
        if n < 1:
            return (disk_no, part_no, 0)
        start_off = u64(buf2.raw, 8 + 8)      # Extents[0].StartingOffset
        return (disk_no, part_no, start_off)
    finally:
        k32.CloseHandle(ctypes.c_void_p(h))


def win_list_drives():
    """列出本机所有盘（重点标出可移动盘 = 读卡器里的卡）"""
    if os.name != 'nt':
        return []
    import ctypes
    k32 = ctypes.windll.kernel32
    k32.GetLogicalDrives.restype = ctypes.c_uint
    mask = k32.GetLogicalDrives()
    out = []
    for i in range(26):
        if not (mask >> i) & 1:
            continue
        letter = chr(ord('A') + i)
        root = '%s:\\' % letter
        dtype = k32.GetDriveTypeW(ctypes.c_wchar_p(root))   # 2 = 可移动
        label = ctypes.create_unicode_buffer(64)
        fsname = ctypes.create_unicode_buffer(32)
        k32.GetVolumeInformationW(ctypes.c_wchar_p(root), label, 64, None, None,
                                  None, fsname, 32)
        info = win_volume_map(letter)
        out.append({
            'letter': letter,
            'removable': (dtype == 2),
            'label': label.value,
            'fs': fsname.value,
            'dev': '\\\\.\\%s:' % letter,
            'disk': info[0] if info else None,
            'part': info[1] if info else None,
            'part_off': info[2] if info else None,
        })
    return out


#==============================================================================
#  分区表：MBR / GPT
#==============================================================================
def find_partition(dev, verbose=True):
    """在整盘（镜像）里找第一个数据分区，返回 (起始字节, 说明)

    三种可能：
      ① 扇区 0 本身就是 BPB（"超级软盘"格式，整卡没分区表，卡里很常见）
      ② MBR 分区表（0x1BE 处四个 16 字节表项）
      ③ GPT（保护型 MBR，类型 0xEE）→ 去 LBA1 读 GPT 头
    """
    sec0 = dev.read_sector(0, 1)
    if len(sec0) < SECTOR:
        raise SystemExit('读不到第 0 扇区（设备没插好 / 权限不够？）')

    # ---- ① 超级软盘：扇区 0 直接是 FAT 引导扇区 ----
    if look_like_fat_boot(sec0):
        return 0, '超级软盘（无分区表，整个卡就是一个卷）'

    # ---- ③ GPT ----
    if sec0[0x1BE + 4] == 0xEE:
        sec1 = dev.read_sector(1, 1)
        if sec1[0:8] == b'EFI PART':
            ent_lba = u64(sec1, 72)
            n_ent = u32(sec1, 80)
            ent_sz = u32(sec1, 84)
            if ent_sz == 0 or ent_sz > 512:
                ent_sz = 128
            for k in range(min(n_ent, 128)):
                raw = dev.read(ent_lba * SECTOR + k * ent_sz, ent_sz)
                if raw[0:16] == b'\x00' * 16:
                    continue
                first = u64(raw, 32)
                if first:
                    return first * SECTOR, 'GPT 分区表（第 %d 个表项）' % (k + 1)

    # ---- ② MBR ----
    for k in range(4):
        e = 0x1BE + 16 * k
        ptype = sec0[e + 4]
        lba = u32(sec0, e + 8)
        nsec = u32(sec0, e + 12)
        if ptype == 0 or lba == 0 or nsec == 0:
            continue
        return lba * SECTOR, 'MBR 分区表（第 %d 个分区，类型 0x%02X，LBA %d）' % (k + 1, ptype, lba)
    raise SystemExit('MBR/GPT 里没找到可用分区')


def look_like_fat_boot(s):
    """判断这 512 字节像不像一个 FAT 引导扇区（BPB）"""
    if len(s) < 512:
        return False
    bps = u16(s, 11)
    spc = s[13]
    nfat = s[16]
    if bps not in (512, 1024, 2048, 4096):
        return False
    if spc == 0 or (spc & (spc - 1)) != 0:
        return False
    if nfat not in (1, 2):
        return False
    if s[21] not in (0xF0, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0xFF):
        return False
    if s[510] != 0x55 or s[511] != 0xAA:
        return False
    return True


#==============================================================================
#  文件系统：FAT12/16/32 与 exFAT
#==============================================================================
class Entry(object):
    """目录里的一项（文件或子目录）"""

    def __init__(self, path, size, first_clus, is_dir, name_ok=True):
        self.path = path
        self.size = size
        self.first = first_clus
        self.is_dir = is_dir
        self.name_ok = name_ok
        self.runs = None            # [(起始簇, 簇数), ...]
        self.contiguous = None
        self.note = []

    def cluster_count(self, bytes_per_clus):
        if self.size == 0:
            return 0
        return (self.size + bytes_per_clus - 1) // bytes_per_clus


class Fs(object):
    """解析一个 FAT12/16/32 或 exFAT 卷。

    vol_base —— 卷在"设备地址空间"里的起始字节（盘符模式=0；整盘/镜像模式=分区起始）
    card_base —— 卷在"整张卡"上的起始字节（给 FPGA 用的物理偏移要从这里算起）
    """

    def __init__(self, dev, vol_base, card_base, part_desc):
        self.dev = dev
        self.vol_base = vol_base
        self.card_base = card_base
        self.part_desc = part_desc
        boot = dev.read(vol_base, SECTOR)
        self.boot = boot
        if boot[3:11] == b'EXFAT   ':
            self.kind = 'exFAT'
            self._init_exfat(boot)
        else:
            self.kind = None
            self._init_fat(boot)

    #--------------------------------------------------------------------------
    #  卷内偏移 ↔ 设备偏移 ↔ 卡上物理偏移
    #--------------------------------------------------------------------------
    def dev_off(self, vol_off):
        return self.vol_base + vol_off

    def card_off(self, vol_off):
        return self.card_base + vol_off

    def dev_addr_of_card(self, card_off):
        """★ 卡上物理偏移 → 用**当前这个设备句柄**去读时该用的偏移 ★

        【为什么必须换一下】两种打开方式，设备的地址空间不一样：
          · 整盘/镜像模式：句柄就是整张卡，vol_base == card_base，直接读没问题；
          · 盘符（卷）模式：句柄只覆盖那个分区，**卷内第 0 字节就是句柄的第 0 字节**，
            所以想读"卡上第 1048576+xxx 字节"，用这个句柄要读的是"卷内第 xxx 字节"。
        把物理偏移直接喂给卷句柄 → 读偏了一个分区的距离 → 自检必然对不上。
        （这个 bug 就是被自检抓出来的：卡上跑的时候报"不一致"，一查是这里。）
        """
        return card_off - self.card_base + self.vol_base

    def read_vol(self, vol_off, n):
        return self.dev.read(self.dev_off(vol_off), n)

    def clus_vol_off(self, clus):
        """簇号 → 卷内字节偏移"""
        return (self.data_start_sec + (clus - 2) * self.spc) * self.bps

    #--------------------------------------------------------------------------
    #  FAT12/16/32
    #--------------------------------------------------------------------------
    def _init_fat(self, boot):
        if not look_like_fat_boot(boot):
            raise SystemExit('这个位置不是 FAT/exFAT 引导扇区 —— 分区起始算错了吧？')
        self.bps = u16(boot, 11)
        self.spc = boot[13]
        self.rsvd = u16(boot, 14)
        self.nfat = boot[16]
        self.root_ent = u16(boot, 17)
        tot16 = u16(boot, 19)
        self.fatsz = u16(boot, 22)
        tot32 = u32(boot, 32)
        if self.fatsz == 0:
            self.fatsz = u32(boot, 36)
        self.total_sec = tot16 if tot16 else tot32
        self.root_clus = u32(boot, 44)
        self.oem = boot[3:11].decode('ascii', 'replace')
        self.label = boot[71:82].decode('cp437', 'replace').rstrip(' \x00') if self.root_ent == 0 \
            else boot[43:54].decode('cp437', 'replace').rstrip(' \x00')

        root_dir_sec = (self.root_ent * 32 + self.bps - 1) // self.bps
        self.root_dir_sec = root_dir_sec          # FAT12/16 的根目录区长度（扇区）
        self.data_start_sec = self.rsvd + self.nfat * self.fatsz + root_dir_sec
        data_sec = self.total_sec - self.data_start_sec
        self.clus_cnt = data_sec // self.spc
        if self.root_ent == 0 and self.fatsz == 0:
            raise SystemExit('BPB 里 FAT 大小为 0，这个卷不合理')
        if self.root_ent == 0:
            self.kind = 'FAT32'
        elif self.clus_cnt < 4085:
            self.kind = 'FAT12'
        else:
            self.kind = 'FAT16'
        self.fat_base = self.rsvd * self.bps
        self.bytes_per_clus = self.bps * self.spc
        self._fat_cache = {}

    def _fat_next(self, clus):
        """查 FAT 表，返回下一个簇号（0x0FFFFFFF 系 = 结束）"""
        if self.kind == 'FAT32':
            off = self.fat_base + clus * 4
            if off // self.bps not in self._fat_cache:
                self._fat_cache.clear()
                self._fat_cache[off // self.bps] = self.read_vol(off - off % self.bps, self.bps)
            v = u32(self._fat_cache[off // self.bps], off % self.bps) & 0x0FFFFFFF
            if v >= 0x0FFFFFF8:
                return None
            if v == 0x0FFFFFF7:
                return 'BAD'
            return v
        if self.kind == 'FAT16':
            off = self.fat_base + clus * 2
            if off // self.bps not in self._fat_cache:
                self._fat_cache.clear()
                self._fat_cache[off // self.bps] = self.read_vol(off - off % self.bps, self.bps)
            v = u16(self._fat_cache[off // self.bps], off % self.bps)
            if v >= 0xFFF8:
                return None
            return v
        # FAT12：1.5 字节一项
        off = self.fat_base + (clus * 3) // 2
        if off // self.bps not in self._fat_cache:
            self._fat_cache.clear()
            self._fat_cache[off // self.bps] = self.read_vol(off - off % self.bps, self.bps)
        blk = self._fat_cache[off // self.bps]
        i = off % self.bps
        b0 = blk[i]
        b1 = blk[i + 1] if i + 1 < len(blk) else 0
        v = ((b1 & 0x0F) << 8) | b0 if (clus & 1) == 0 else (b1 << 4) | (b0 >> 4)
        if v >= 0xFF8:
            return None
        return v

    def chain(self, first, limit_clusters=None):
        """顺 FAT 簇链，返回簇号列表（同时防跑飞）"""
        out = []
        c = first
        cap = limit_clusters if limit_clusters else (self.clus_cnt + 2)
        while c and c >= 2 and len(out) < cap:
            out.append(c)
            nxt = self._fat_next(c)
            if nxt is None or nxt == 'BAD':
                break
            if nxt == 0:
                break
            c = nxt
        return out

    #--------------------------------------------------------------------------
    #  exFAT
    #--------------------------------------------------------------------------
    def _init_exfat(self, boot):
        self.bps = 1 << boot[108]
        self.spc = 1 << boot[109]
        self.fat_off_sec = u32(boot, 80)
        self.fat_len_sec = u32(boot, 84)
        self.heap_sec = u32(boot, 88)
        self.clus_cnt = u32(boot, 92)
        self.root_clus = u32(boot, 96)
        self.nfat = boot[110] or 1
        self.bytes_per_clus = self.bps * self.spc
        self.fat_base = self.fat_off_sec * self.bps
        self.data_start_sec = self.heap_sec
        self.label = ''
        self._fat_cache = {}

    def _exfat_next(self, clus):
        off = self.fat_base + clus * 4
        if off // self.bps not in self._fat_cache:
            self._fat_cache.clear()
            self._fat_cache[off // self.bps] = self.read_vol(off - off % self.bps, self.bps)
        v = u32(self._fat_cache[off // self.bps], off % self.bps)
        if v >= 0xFFFFFFF8:
            return None
        return v

    def _exfat_chain(self, first, nofat):
        if nofat:
            return None             # 标记了"不用 FAT" = 本身就是连续的
        out = []
        c = first
        while c and c >= 2 and len(out) < self.clus_cnt + 2:
            out.append(c)
            nxt = self._exfat_next(c)
            if not nxt:
                break
            c = nxt
        return out

    #--------------------------------------------------------------------------
    #  读一个目录（返回 32 字节表项拼接成的字节串）
    #--------------------------------------------------------------------------
    def read_dir_clusters(self, first_clus, root_fixed=False):
        if root_fixed:
            # FAT12/16 的根目录是"固定区"，不占簇：位置 = 保留区 + 所有 FAT 表之后
            start_sec = self.data_start_sec - self.root_dir_sec
            return self.read_vol(start_sec * self.bps, self.root_ent * 32)
        if first_clus < 2:
            return b''
        if self.kind == 'exFAT':
            chain = self._exfat_chain(first_clus, False)
            if chain is None:                     # NoFatChain：按连续簇读
                chain = list(range(first_clus, first_clus + 128))
        else:
            chain = self.chain(first_clus)
        buf = b''
        for c in chain:
            buf += self.read_vol(self.clus_vol_off(c), self.bytes_per_clus)
            if len(buf) > MAX_DIR_BYTES:
                break
        return buf

    #--------------------------------------------------------------------------
    #  遍历：把整棵目录树走一遍，产出 Entry
    #--------------------------------------------------------------------------
    def walk(self):
        out = []
        if self.kind == 'exFAT':
            self._walk_exfat(self.root_clus, '', out, 0)
        else:
            root_fixed = (self.kind in ('FAT12', 'FAT16'))
            data = self.read_dir_clusters(self.root_clus, root_fixed)
            self._walk_fat(data, '', out, 0)
        return out

    def _walk_fat(self, data, prefix, out, depth):
        if depth > 8:
            return
        lfn = []
        for i in range(0, len(data) - 31, 32):
            e = data[i:i + 32]
            if e[0] == 0x00:
                break
            if e[0] == 0xE5:
                lfn = []
                continue
            attr = e[11]
            if attr == ATTR_LFN:
                # LFN 项：13 个 UTF-16 字符，分散在三段里
                chars = e[1:11] + e[14:26] + e[28:32]
                lfn.append((e[0] & 0x3F, chars.decode('utf-16-le', 'replace')))
                continue
            if attr & ATTR_VOLUME_ID and not (attr & ATTR_DIRECTORY):
                lfn = []
                continue
            name = None
            if lfn:
                name = ''.join(t for _, t in sorted(lfn))
                name = name.split('\x00')[0].replace('\uffff', '')
            if not name:
                name = decode_short_name(e[0:11])
            lfn = []
            size = u32(e, 28)
            first = (u16(e, 20) << 16) | u16(e, 26)
            is_dir = bool(attr & ATTR_DIRECTORY)
            path = name if not prefix else (prefix + '/' + name)
            ent = Entry(path, size, first, is_dir, name_ok=(name not in ('.', '..')))
            if name in ('.', '..'):
                continue
            out.append(ent)
            if is_dir:
                sub = self.read_dir_clusters(first, False)
                self._walk_fat(sub, path, out, depth + 1)

    def _walk_exfat(self, first_clus, prefix, out, depth):
        if depth > 8 or first_clus < 2:
            return
        data = self.read_dir_clusters(first_clus, False)
        i = 0
        while i + 31 < len(data):
            e = data[i:i + 32]
            t = e[0]
            if t == 0x00:
                break
            if t == 0x85:
                attr = u16(e, 4)
                name = ''
                size = 0
                fclus = 0
                nofat = False
                j = i + 32
                while j + 31 < len(data):
                    s = data[j:j + 32]
                    if s[0] == 0xC0:
                        nofat = bool(s[1] & 0x02)
                        nlen = s[3]
                        fclus = u32(s, 20)
                        size = u64(s, 24)
                    elif s[0] == 0xC1:
                        name += s[2:32].decode('utf-16-le', 'replace')
                    else:
                        break
                    j += 32
                name = name.split('\x00')[0]
                if name and name not in ('.', '..'):
                    is_dir = bool(attr & 0x10)
                    path = name if not prefix else (prefix + '/' + name)
                    ent = Entry(path, size, fclus, is_dir)
                    out.append(ent)
                    if is_dir:
                        self._walk_exfat(fclus, path, out, depth + 1)
                i = j
                continue
            i += 32

    #--------------------------------------------------------------------------
    #  簇链 → 连续段 [(起始簇, 簇数)]
    #--------------------------------------------------------------------------
    def runs_of(self, clus_list):
        runs = []
        for c in clus_list:
            if runs and runs[-1][0] + runs[-1][1] == c:
                runs[-1][1] += 1
            else:
                runs.append([c, 1])
        return [tuple(r) for r in runs]

    def fill_runs(self, ent, nofat=False):
        need = ent.cluster_count(self.bytes_per_clus)
        if need == 0:
            ent.runs = []
            ent.contiguous = True
            return
        chain = self._exfat_chain(ent.first, nofat) if self.kind == 'exFAT' else self.chain(ent.first)
        if chain is None:                       # exFAT 的 NoFatChain：天然连续
            ent.runs = [(ent.first, need)]
            ent.contiguous = True
            ent.note.append('exFAT NoFatChain 标记 → 天然连续')
            return
        ent.runs = self.runs_of(chain)
        ent.contiguous = (len(ent.runs) == 1 and len(chain) >= need)
        if len(chain) < need:
            ent.note.append('簇链只有 %d 个簇，文件需要 %d 个 —— 目录项或 FAT 表坏了'
                            % (len(chain), need))

    #--------------------------------------------------------------------------
    #  读文件（顺簇链，用于自检；max_bytes 用来"只要文件头"时少读一点）
    #--------------------------------------------------------------------------
    def read_file(self, ent, max_bytes=None):
        want = ent.size if max_bytes is None else min(max_bytes, ent.size)
        buf = b''
        for (c0, n) in (ent.runs or []):
            for k in range(n):
                buf += self.read_vol(self.clus_vol_off(c0 + k), self.bytes_per_clus)
                if len(buf) >= want:
                    return buf[:want]
        return buf[:want]


#==============================================================================
#  BMP 文件头校验（和 bmp_read.v 里的 header_match 保持同一套判据）
#==============================================================================
def check_bmp_header(h):
    """返回 (是否合法, 说明, 详情 dict)"""
    d = {}
    if len(h) < BMP_HDR_LEN:
        return False, '文件太短，读不满 54 字节文件头', d
    d['signature'] = bytes(h[0:2]).decode('latin1')
    d['file_len'] = u32(h, 2)
    d['pixel_offset'] = u32(h, 10)
    d['width'] = u32(h, 18)
    d['height'] = u32(h, 22)
    d['bpp'] = u16(h, 28)
    d['compression'] = u32(h, 30)
    if h[0:2] != b'BM':
        return False, "开头不是 'BM'（这是 %r，看起来不是 BMP）" % d['signature'], d
    if d['compression'] != 0:
        return False, '压缩方式 = %d（必须是 0 = 不压缩）' % d['compression'], d
    if d['bpp'] != EXPECT_BPP:
        return False, '位深 = %d（必须是 %d）' % (d['bpp'], EXPECT_BPP), d
    if d['width'] != EXPECT_W or d['height'] != EXPECT_H:
        return False, '分辨率 = %dx%d（必须是 %dx%d）' % (d['width'], d['height'],
                                                         EXPECT_W, EXPECT_H), d
    return True, 'OK（%dx%d %dbit 不压缩）' % (d['width'], d['height'], d['bpp']), d


def is_bmp_candidate(name, head):
    if name.lower().endswith('.bmp'):
        return True
    return head[0:2] == b'BM'


#==============================================================================
#  主流程
#==============================================================================
class Candidate(object):
    def __init__(self, ent, phy_off, sector, valid, why, hdr, selfcheck):
        self.ent = ent
        self.phy_off = phy_off
        self.sector = sector
        self.valid = valid
        self.why = why
        self.hdr = hdr
        self.selfcheck = selfcheck


def analyze(fs, verbose=True):
    ents = fs.walk()
    files = [e for e in ents if not e.is_dir]
    cands = []
    for e in files:
        # 先读文件头（顺簇链）判断要不要当候选；这里只要 54 字节，别整张读
        if e.size <= 0 or e.first < 2:
            continue
        try:
            fs.fill_runs(e)
            head = fs.read_file(e, BMP_HDR_LEN) if e.runs else b''
        except Exception as ex:                     # noqa: BLE001
            head = b''
            e.note.append('读文件头出错：%s' % ex)
        if len(head) < 2 or not is_bmp_candidate(e.path, head):
            continue

        valid, why, hdr = check_bmp_header(head) if len(head) >= BMP_HDR_LEN else (False, '文件头读不全', {})
        phy = None
        sec = None
        selfcheck = '未做'
        if e.runs:
            # 起始簇 → 卷内偏移 → 卡上物理偏移
            vol_off = fs.clus_vol_off(e.first)
            phy = fs.card_off(vol_off)
            sec = phy // SECTOR
            selfcheck = do_selfcheck(fs, e, phy)
        cands.append(Candidate(e, phy, sec, valid, why, hdr, selfcheck))
    return cands


def do_selfcheck(fs, ent, phy):
    """★ 核心验证 ★：'按 FAT 簇链读出来的前 512 字节' vs '按物理偏移裸读的 512 字节'

    fs.card_off() 算出来的物理偏移，是给 FPGA 的裸扇区读用的。如果这个偏移是对的，
    那么"直接从设备上这个位置读 512 字节"必然等于"文件真正的前 512 字节"。
    对不上 → 偏移算错了（分区起始/FAT 解析/簇大小 任一处），这里立刻暴露。
    """
    want = min(512, ent.size)
    a = fs.read_file(ent, want)
    # 【注意】这里要用 dev_addr_of_card 换算，不能把物理偏移直接喂给句柄：
    #   盘符（卷）模式下句柄只覆盖那个分区，差一个分区的距离。
    b = fs.dev.read(fs.dev_addr_of_card(phy), want)
    if len(a) != len(b) or len(a) == 0:
        return '读不到（磁盘读回 %d 字节，预期 %d 字节），跳过' % (len(b), len(a))
    if a == b:
        return 'OK（前 %d 字节与簇链读法逐字节一致）' % len(a)
    n_bad = sum(1 for i in range(len(a)) if a[i] != b[i])
    first = next(i for i in range(len(a)) if a[i] != b[i])
    return ('!! 不一致（%d/%d 字节不同，第一个不同在 +%d：链 %02X vs 盘 %02X）'
            '—— 偏移或簇起始算错了' % (n_bad, len(a), first, a[first], b[first]))


def dump_candidates(fs, cands, outdir):
    """把卡上每个 BMP 的**原样内容**导出到 outdir，并尽量和本地素材逐字节比对。

    ★ 2026-10-18 新增 ★
      动机：数码管的 E 只说明"读到了合法的 640×480 BMP 文件头"，
      它**不代表**这张图的内容和工程里那份一样 ——
      卡上可能是另一张图、也可能拷坏了（前 54 字节是好的，后面是垃圾）。
      那种情况下上板就是"图框里一片花"，而所有既有诊断位都正常。
      这个选项把"卡上那份到底是哪一份"变成可核对的事实：
        · 导出成 .bin（可以再拿来喂给仿真，让仿真在本地复现）
        · 本地有同名素材（doc/TF卡图片/）时，直接逐字节比对并报第一处不同
    """
    if not os.path.isdir(outdir):
        os.makedirs(outdir)
    local_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                             'doc', 'TF卡图片')
    man = []
    for c in cands:
        if not c.ent.runs:
            continue
        name = os.path.basename(c.ent.path)
        safe = re.sub(r'[^\w\u4e00-\u9fff.]+', '_', name)
        out = os.path.join(outdir, safe + '.bin')
        try:
            data = fs.read_file(c.ent)
        except Exception as ex:                                  # noqa: BLE001
            print('  !! 导出 %s 失败：%s' % (name, ex))
            continue
        with open(out, 'wb') as f:
            f.write(data)
        note = ''
        lp = os.path.join(local_dir, name)
        if os.path.isfile(lp):
            with open(lp, 'rb') as f:
                ref = f.read()
            if data == ref:
                note = '✓ 与本地素材逐字节一致'
            else:
                n = min(len(data), len(ref))
                d = next((k for k in range(n) if data[k] != ref[k]), n)
                note = ('✗ 与本地素材不同：卡上 %d 字节 / 本地 %d 字节，'
                        '第 %d 字节起不同（卡上 = 0x%02X，本地 = 0x%02X）'
                        % (len(data), len(ref), d,
                           data[d] if d < len(data) else -1,
                           ref[d] if d < len(ref) else -1))
        else:
            note = '（本地没有同名素材可比）'
        print('  导出 %-46s → %s' % (name, out))
        print('        %s' % note)
        man.append('%s\t%d\t%s\t%s\t%s' % (name, c.ent.size, c.phy_off,
                                           'C' if c.ent.contiguous else 'F',
                                           safe + '.bin'))
    with open(os.path.join(outdir, 'manifest.txt'), 'w', encoding='utf-8') as f:
        f.write('# 名字\tFAT大小\t物理偏移\t连续(C/F)\t导出文件\n')
        f.write('\n'.join(man) + '\n')
    print('  清单：%s' % os.path.join(outdir, 'manifest.txt'))
    print()


def main():
    ap = argparse.ArgumentParser(
        description='读 TF 卡 FAT/exFAT 目录表，算出 BMP 的物理字节偏移（给 IMG0/1/2_OFFSET）',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--list', action='store_true', help='列出本机所有盘（标出可移动盘 = 卡）')
    ap.add_argument('--dev', help=r'设备：Windows 用 \\.\E: 或 \\.\PhysicalDrive2；Linux 用 /dev/sdb')
    ap.add_argument('--image', help='卡镜像文件（离线分析用，不需要权限）')
    ap.add_argument('--apply', help='把算出的偏移直接写回这个 .v 文件（顶层）')
    ap.add_argument('--json', help='把结果另存为 JSON')
    ap.add_argument('--all', action='store_true', help='列出卡上所有文件，不只 BMP')
    ap.add_argument('--pick', help='指定哪几张当 IMG0/IMG1/IMG2（名字的一部分，逗号隔开，2~3 个），'
                                   '例如 --pick 苹果,草莓,西瓜')
    ap.add_argument('--dump', help='把卡上每个 BMP 的原样内容导出到这个目录，'
                                   '并和 doc/TF卡图片/ 里的同名素材逐字节比对')
    ap.add_argument('--allow-fragmented', action='store_true',
                    help='即使文件不连续也照样给出偏移（默认拒绝）')
    ap.add_argument('--sector', type=int, default=512, help='逻辑扇区大小（默认 512）')
    ap.add_argument('--rootdir', action='store_true',
                    help='只输出根目录参数（ROOT_LBA / FAT_LBA / SPC_SHIFT / ROOT_CONTIG），不找 BMP')
    ap.add_argument('--apply-verilog',
                    help='配合 --rootdir：把 4 个根目录参数写回这个 .v 文件')
    args = ap.parse_args()

    global SECTOR
    SECTOR = args.sector

    if args.list:
        rows = win_list_drives()
        if not rows:
            print('  这台机器上没查到盘（非 Windows 请直接用 --dev /dev/sdX）')
            return 0
        print('  %-6s %-4s %-18s %-8s %-8s %s' % ('盘符', '类型', '卷标', '文件系统', '物理盘', '分区起始字节'))
        for r in rows:
            print('  %-6s %-4s %-18s %-8s %-8s %s' % (
                r['letter'] + ':', '可移动' if r['removable'] else '固定', r['label'],
                r['fs'] or '-', r['disk'] if r['disk'] is not None else '-',
                r['part_off'] if r['part_off'] is not None else '-'))
        print('\n  卡一般是"可移动"。拿到盘符后：')
        print(r'      python tools/sd_fat_offset.py --dev \\.\E:      （要管理员权限）')
        return 0

    # ---- 没给 --dev/--image：自动找唯一的可移动盘 ----
    if not args.dev and not args.image:
        rows = [r for r in win_list_drives() if r['removable']]
        if len(rows) == 1:
            args.dev = rows[0]['dev']
            print('  自动选中可移动盘 %s（%s %s）' % (args.dev, rows[0]['label'], rows[0]['fs']))
        elif not rows:
            raise SystemExit('没找到可移动盘。请先插卡，或用 --dev/--image 指定。\n'
                             '  （也可以先 --list 看看）')
        else:
            raise SystemExit('检测到多个可移动盘，请用 --dev 指定其中一个：%s'
                             % ', '.join(r['dev'] for r in rows))

    # ---- 打开 ----
    try:
        if args.image:
            dev = RawDev(args.image, align=args.sector)
            vol_base = card_base = 0
            if look_like_fat_boot(dev.read_sector(0, 1)):
                part_desc = '镜像文件：卷直接在第 0 字节（无分区表）'
            else:
                off, part_desc = find_partition(dev)
                vol_base = card_base = off
            dev_desc = '镜像 %s' % args.image
        else:
            dev = RawDev(args.dev, align=args.sector)
            if args.dev.upper().startswith('\\\\.\\PHYSICALDRIVE') or args.dev.startswith('/dev/'):
                # 整盘：分区起始从分区表里读，物理偏移 = 分区起始 + 卷内偏移
                off, part_desc = find_partition(dev)
                vol_base = card_base = off
                dev_desc = '整盘 %s' % args.dev
            else:
                # 盘符/卷：裸读地址是**分区内**的，分区在卡上的起始字节要单独问
                vol_base = 0
                card_base = 0
                part_desc = '卷（盘符）模式'
                if os.name == 'nt':
                    letter = args.dev.rstrip(':').lstrip('\\\\.')
                    info = win_volume_map(letter)
                    if info and info[2] is not None:
                        card_base = info[2]
                        part_desc = '卷模式：物理盘 %d 第 %d 分区，分区起始字节 %d' % (
                            info[0], info[1], info[2])
                    else:
                        raise SystemExit(
                            '拿不到这个卷在物理盘上的起始字节 —— 这样算出来的偏移会少一段，\n'
                            '  不敢给你用。两个办法：\n'
                            r'   ① 改用它所在的物理盘：--dev \\.\PhysicalDriveN'
                            '（N 可以 --list 看）\n'
                            '   ② 直接把卡镜像导出成文件，用 --image 分析')
                dev_desc = '设备 %s' % args.dev
    except PermissionError as ex:
        raise SystemExit('打开 %s 被拒绝：%s\n'
                         '  Windows 下裸读磁盘必须**以管理员身份**运行（右键 → 以管理员身份运行）。'
                         % (args.dev, ex))
    except OSError as ex:
        raise SystemExit('打开 %s 失败：%s\n'
                         '  Windows 下裸读磁盘必须**以管理员身份**运行；'
                         '也可能是盘符写错了（先跑 --list）。' % (args.dev, ex))

    print('=' * 78)
    print('  设备      ：%s' % dev_desc)
    print('  分区      ：%s' % part_desc)
    if card_base:
        print('  卷在卡上起始：%d 字节（第 %d 扇区）' % (card_base, card_base // SECTOR))
    print('=' * 78)

    fs = Fs(dev, vol_base, card_base, part_desc)
    print('  文件系统  ：%s   扇区 %d 字节 × 每簇 %d 扇区 = 簇 %d 字节'
          % (fs.kind, fs.bps, fs.spc, fs.bytes_per_clus))
    print('  卷标      ：%s' % (fs.label or '（无）'))
    print('  数据区起点：卷内第 %d 扇区' % fs.data_start_sec)
    print()

    # ---- 根目录模式：只输出 lister 需要的参数，不找 BMP ----
    if args.rootdir:
        vals = rootdir_report(fs)
        if args.apply_verilog:
            apply_root_params(args.apply_verilog, vals)
        if args.json:
            slim = dict((k, v) for k, v in vals.items()
                        if k not in ('chain', 'files', 'dirs'))
            with open(args.json, 'w', encoding='utf-8') as f:
                json.dump(slim, f, ensure_ascii=False, indent=2)
            print('  已写入 JSON：%s' % args.json)
        return 0

    if args.all:
        print('  --- 卡上所有文件 ---')
        for e in fs.walk():
            if not e.is_dir:
                print('   %10s  %s' % (e.size, e.path))
        print()

    cands = analyze(fs)

    if not cands:
        print('  ✗ 卡上没找到任何 BMP（.bmp 结尾，或开头两字节是 "BM" 的文件）。')
        print('    请确认图片已经拷进卡里。要列全部文件加 --all。')
        return 1

    print('  --- 卡上的 BMP ---')
    print('  %-42s %-11s %-9s %-9s %s' % ('文件', '大小', '合法?', '连续?', '物理字节偏移'))
    for c in cands:
        e = c.ent
        size = sizeof_str(e.size)
        ok = '✓' if c.valid else '✗'
        cont = '✓' if e.contiguous else '✗'
        off = str(c.phy_off) if c.phy_off is not None else '-'
        print('  %-42s %-11s %-9s %-9s %s' % (e.path[:42], size, ok, cont, off))
        if not c.valid:
            print('      ↳ 为什么不能用：%s' % c.why)
        if c.phy_off is not None:
            if c.phy_off % SECTOR:
                print('      ↳ !! 偏移 %d 不是 %d 的整数倍，装载器按扇区换算会读错位置'
                      % (c.phy_off, SECTOR))
            print('      ↳ 扇区号 %d（loader 里 lat_sector = offset>>9 = %d）'
                  % (c.sector, c.phy_off >> 9))
            print('      ↳ 自检：%s' % c.selfcheck)
            if e.runs:
                print('      ↳ 簇链连续段 %d 段：%s' % (len(e.runs),
                      ', '.join('簇%d×%d' % (r[0], r[1]) for r in e.runs[:6])))
            for n in e.note:
                print('      ↳ 注意：%s' % n)
    print()

    if args.dump:
        # ★ 导出独立于 --pick ★：卡上多于两张图时算不出参数，但"导出 + 逐字节比对"
        #   照样是有效证据（而且不需要动 RTL / 不需要重新综合）。
        dump_candidates(fs, cands, args.dump)

    # ---- 挑出可用的：合法 + 连续 + 自检通过 ----
    good = [c for c in cands if c.valid and c.ent.contiguous and c.selfcheck.startswith('OK')]
    warn_frag = [c for c in cands if c.valid and not c.ent.contiguous]
    if warn_frag and not args.allow_fragmented:
        print('  !! 有 %d 张图是**碎片**（在卡上不连续）。' % len(warn_frag))
        print('     装载器是在卡上**线性**搬数据的，碎片图会搬出满屏花屏。')
        print('     处理办法（任选一）：')
        print('       ① 把卡格式化（FAT32）后，**先拷这几张图**再拷别的东西；')
        print('       ② 用工具把卡上的图片碎片整理掉；')
        print('       ③ 加 --allow-fragmented 强行输出（不推荐，画面会花）。')
        print()

    if len(good) < 2:
        print('  ✗ 只有 %d 张图同时满足「合法 + 连续 + 自检通过」，凑不齐两张。' % len(good))
        if good:
            print('     目前可用的：%s' % ', '.join(c.ent.path for c in good))
        return 1

    # ★ 2026-10-18：默认不再"盲取前两张" ★
    #   以前的写法是 pick = good[:2]（按**发现顺序**取前两张）。卡上 BMP 多于两张时
    #   这**不一定是你要的那两张** —— 上板表现是"图能正常显示，但是另一张图"，
    #   而且数码管照样读 E（E 只代表"找到了合法 640×480 BMP"，不代表"是你要的图"），
    #   极难察觉。2026-10-18 就因为这个，把西瓜的偏移当成草莓写进了顶层，白绕一圈。
    #   所以：只有 ≤3 张可用 → 直接用（没有歧义）；多于 3 张 → 拒绝，要求 --pick 点名。
    #   ★ 2026-10-22：App 层升级成 3 张图的查看器，上限从 2 张放宽到 3 张。
    if len(good) > 3 and not args.pick:
        print('  !! 卡上有 %d 张可用 BMP，脚本不知道你要哪几张（不替你做这个决定）:' % len(good))
        print('     请用 --pick 点名，例如：')
        print('       python tools/sd_fat_offset.py --pick 苹果,草莓,西瓜')
        print('     （顺序 = 参数顺序：--pick 里的第 1 个 → IMG0，第 2 个 → IMG1，第 3 个 → IMG2）')
        print('     卡上可用的有：')
        for c in good:
            print('       %-46s 偏移 %d' % (c.ent.path, c.phy_off))
        if args.dump:
            print('     （--dump 已经按你的要求导出完毕；要把参数也算出来就再加 --pick）')
            return 0
        return 1

    pick = good[:3]
    if args.pick:
        want = [w.strip() for w in args.pick.split(',') if w.strip()]
        if len(want) not in (2, 3):
            raise SystemExit('--pick 要写 2 或 3 个名字，用逗号隔开，例如 --pick 苹果,草莓,西瓜')
        picked = []
        for w in want:
            hit = None
            for c in good:
                if w.lower() in c.ent.path.lower():
                    hit = c
                    break
            if hit is None:
                # 名字对上了、但没进"可用"名单 → 说清楚为什么，别只说"没找到"
                near = [c for c in cands if w.lower() in c.ent.path.lower()]
                if near:
                    c = near[0]
                    why = []
                    if not c.valid:
                        why.append('文件头不合法：%s' % c.why)
                    if not c.ent.contiguous:
                        why.append('在卡上是碎片（簇链 %s）—— 装载器线性搬数据会花屏，'
                                   '请先整理/重新拷贝这张图' % c.ent.runs)
                    if not c.selfcheck.startswith('OK'):
                        why.append('偏移自检没过：%s' % c.selfcheck)
                    raise SystemExit('--pick 里的 %r 对应 %s，但它不能用：%s'
                                     % (w, c.ent.path, '；'.join(why) or '未知原因'))
                raise SystemExit('--pick 里写的 %r 没找到（可用的有：%s）'
                                 % (w, ', '.join(c.ent.path for c in good)))
            picked.append(hit)
        if len(set(id(c) for c in picked)) != len(picked):
            raise SystemExit('--pick 里有重复的名字指向同一张图，换一个')
        pick = picked
    print('  ' + '=' * 74)
    print('  可用（%s）：' % ('按 --pick 指定' if args.pick else '按发现顺序取前 %d 张' % len(pick)))
    for i, c in enumerate(pick):
        print('    IMG%d ← %s（大小 %d 字节）' % (i, c.ent.path, c.ent.size))
    print('  ' + '=' * 74)
    print('  ★ 上面第 1 行就是 IMG0、第 2 行 IMG1、第 3 行 IMG2（顺序即参数顺序）—— 请核对文件名！')
    print('  ★ 数码管的 E 只代表"找到了合法 640×480 BMP"，不代表"是你要的那张图"。')
    print('    要确认是哪张图，只能看这里打印的文件名。')
    print()
    print('  ---- 贴到 src/user_source/hdl_source/top_tf_hdmi_audio.v 的这几组参数 ----')
    for i, c in enumerate(pick):
        print('    parameter [31:0] IMG%d_OFFSET = 32\'d%d;   // %s' % (i, c.phy_off, c.ent.path))
        print('    parameter [31:0] IMG%d_LENGTH = 32\'d%d;' % (i, c.ent.size))
    print()
    for i, c in enumerate(pick):
        if c.ent.size == EXPECT_W * EXPECT_H * 3 + BMP_HDR_LEN:
            print('  顺便核对：IMG%d = %d 字节 = 640×480×3 + 54，标准 640×480 24bit 不压缩 BMP'
                  % (i, c.ent.size))
        else:
            print('  !! IMG%d = %d 字节，不是标准的 640×480×3 + 54（%d）'
                  % (i, c.ent.size, EXPECT_W * EXPECT_H * 3 + BMP_HDR_LEN))
    print()

    # ---- ★ 与工程里"当前配置的值"对照 ★ ----
    #  这是最容易出错、也最容易被忽略的一环：卡一旦被"拷过文件 / 删过文件 / 换过卡"，
    #  文件在卡上的位置就会平移，而工程里那几个 parameter 还是旧值 ——
    #  上板表现就是"装载失败/花屏"，但数码管上未必看得出来。
    top = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                       'src', 'user_source', 'hdl_source', 'top_tf_hdmi_audio.v')
    print('  ---- 与工程当前配置对照 ----')
    if os.path.exists(top):
        try:
            with open(top, encoding='utf-8', errors='ignore') as f:
                vs = f.read()
        except OSError:
            vs = ''
        got = {}
        for m in re.finditer(r"parameter\s*\[31:0\]\s*IMG(\d)_(OFFSET|LENGTH)\s*=\s*32'd(\d+)", vs):
            got[(m.group(1), m.group(2))] = int(m.group(3))
        print('    %s' % top)
        bad = False
        for i, c in enumerate(pick):
            for what, now in (('OFFSET', c.phy_off), ('LENGTH', c.ent.size)):
                old = got.get((str(i), what))
                if old is None:
                    print('    IMG%d_%s: 工程里没读到，请手动确认' % (i, what))
                    bad = True
                elif old == now:
                    print('    IMG%d_%s: 配置 %d == 卡上现在 %d   ✓ 一致' % (i, what, old, now))
                else:
                    print('    IMG%d_%s: 配置 %d != 卡上现在 %d   ✗★不一致，必须重烧★'
                          % (i, what, old, now))
                    bad = True
        if bad:
            print('    → 结论：**卡和配置对不上**（拷过/删过文件、换过卡、或重装过）。')
            print('      用 --apply 一键写回，或手工改上面那几组 parameter。')
        else:
            print('    → 结论：偏移/长度与工程完全一致 —— 这一步没问题，')
            print('      排查方向应转向"读回路径"（仲裁/SDRAM 读）而不是 SD 侧。')
    else:
        print('    找不到顶层文件 %s' % top)
    print()

    if args.json:
        with open(args.json, 'w', encoding='utf-8') as f:
            json.dump({'fs': fs.kind, 'part': part_desc, 'card_base': card_base,
                       'candidates': [{'path': c.ent.path, 'size': c.ent.size,
                                       'offset': c.phy_off, 'sector': c.sector,
                                       'valid': c.valid, 'contiguous': c.ent.contiguous,
                                       'selfcheck': c.selfcheck} for c in cands]},
                      f, ensure_ascii=False, indent=2)
        print('  已写出 JSON：%s' % args.json)

    if args.apply:
        apply_to_verilog(args.apply, pick)
    return 0


def rootdir_report(fs, verbose=True):
    """解析根目录，给出 FPGA 侧 fat_dir_lister 需要的全部参数。

    【它解决什么问题】
    以前 BMP 的偏移靠 --pick 手工挑 2~3 个文件写死参数。文件页要"按目录顺序
    列 4 个"，就必须知道**根目录在卡上的位置**，而不是某个文件的偏移。

    【输出什么】
        ROOT_LBA    根目录首簇的数据扇区号（卡上绝对扇区）
        FAT_LBA     FAT0 的起始扇区号
        SPC_SHIFT   log2(每簇扇区数)
        ROOT_CONTIG 根目录簇链是否连续（1 = 可以省掉 FAT 跟随逻辑）
        目录占用    簇数 × 每簇扇区数
        前 4 个文件 名字 / 扇区号 / 字节数（直接对照上板显示）

    【为什么只支持 FAT32】
    FAT12/16 的根目录是固定区，不占簇，地址算法完全不同。本工程只用 FAT32。
    """
    if fs.kind != 'FAT32':
        raise SystemExit('根目录参数只支持 FAT32，当前是 %s。\n'
                         '  FAT12/16 的根目录是固定区，算法不同，本工程不支持。' % fs.kind)

    spc = fs.spc
    if spc == 0 or (spc & (spc - 1)):
        raise SystemExit('每簇 %d 扇区不是 2 的幂，SPC_SHIFT 无法表示。' % spc)
    spc_shift = spc.bit_length() - 1

    # ---- 根目录簇链 ----
    chain = fs.chain(fs.root_clus)
    if not chain:
        raise SystemExit('根目录簇链为空（root_clus = %d）。' % fs.root_clus)
    contig = all(chain[i + 1] == chain[i] + 1 for i in range(len(chain) - 1))

    # ---- 卡上绝对扇区号 ----
    root_vol_off = fs.clus_vol_off(fs.root_clus)
    root_lba = (fs.card_base + root_vol_off) // SECTOR
    fat_lba = (fs.card_base + fs.fat_base) // SECTOR
    dir_secs = len(chain) * spc

    # ---- 逐项解析 ----
    data = fs.read_dir_clusters(fs.root_clus, False)
    files, dirs = [], []
    n_del = n_lfn = n_hidden = 0
    end_idx = None
    for i in range(0, len(data) - 31, 32):
        e = data[i:i + 32]
        if e[0] == 0x00:
            end_idx = i // 32
            break
        if e[0] == 0xE5:
            n_del += 1
            continue
        attr = e[11]
        if attr == 0x0F:
            n_lfn += 1
            continue
        if (attr & 0x08) and not (attr & 0x10):      # 卷标
            continue
        if attr & 0x02:                              # 隐藏（占位图）
            n_hidden += 1
            continue
        name = decode_short_name(e[0:11])
        if name in ('.', '..'):
            continue
        first = (u16(e, 20) << 16) | u16(e, 26)
        size = u32(e, 28)
        rec = dict(name=name, first=first, size=size,
                   is_dir=bool(attr & 0x10),
                   lba=(fs.card_base + fs.clus_vol_off(first)) // SECTOR if first >= 2 else None)
        (dirs if rec['is_dir'] else files).append(rec)

    n_bmp = sum(1 for f in files if f['name'].upper().endswith('.BMP'))
    n_wav = sum(1 for f in files if f['name'].upper().endswith('.WAV'))
    n_txt = sum(1 for f in files if f['name'].upper().endswith('.TXT'))

    res = dict(root_lba=root_lba, fat_lba=fat_lba, spc_shift=spc_shift,
               contig=contig, chain=chain, dir_secs=dir_secs,
               n_file=len(files), n_dir=len(dirs), n_bmp=n_bmp,
               n_wav=n_wav, n_txt=n_txt, n_del=n_del, n_lfn=n_lfn,
               n_hidden=n_hidden, end_slot=end_idx, files=files, dirs=dirs)

    if not verbose:
        return res

    print('  ' + '=' * 72)
    print('  根目录参数（直接贴进 top_tf_hdmi_audio.v）')
    print('  ' + '=' * 72)
    print('    parameter [31:0]  ROOT_LBA    = 32\'d%-8d // 根目录首扇区' % root_lba)
    print('    parameter [31:0]  FAT_LBA     = 32\'d%-8d // FAT0 起始扇区' % fat_lba)
    print('    parameter integer SPC_SHIFT   = %-10d // log2(%d 扇区/簇)' % (spc_shift, spc))
    print('    parameter         ROOT_CONTIG = 1\'b%d       // %s'
          % (1 if contig else 0,
             '簇链连续，可省 FAT 跟随' if contig else '簇链不连续，必须开 FAT 跟随'))
    print()
    print('  根目录：起始簇 %d，簇链 %s' % (fs.root_clus,
          ' → '.join(str(c) for c in chain[:8]) + (' …' if len(chain) > 8 else '')))
    print('          %d 个簇 × %d 扇区 = %d 扇区（%d 字节）'
          % (len(chain), spc, dir_secs, dir_secs * fs.bps))
    print('          簇链连续：%s' % ('✓ 是' if contig else '✗ 否（碎片化）'))
    print()
    print('  目录项统计：文件 %d（BMP %d / WAV %d / TXT %d），子目录 %d'
          % (len(files), n_bmp, n_wav, n_txt, len(dirs)))
    print('              已删 %d，LFN 槽 %d，隐藏 %d，结束于第 %s 槽'
          % (n_del, n_lfn, n_hidden, end_idx if end_idx is not None else '未找到'))
    print('              ⇒ 文件页可翻 %d 页（每页 4 个）'
          % ((len(files) + 3) // 4))
    print()

    if files:
        print('  前 4 个文件（对照上板显示）：')
        print('    %-4s %-14s %-11s %-12s %s' % ('#', '名字', '大小', '扇区号', '起始簇'))
        for k, f in enumerate(files[:4]):
            print('    %-4d %-14s %-11s %-12s %d'
                  % (k, f['name'], sizeof_str(f['size']),
                     f['lba'] if f['lba'] is not None else '-', f['first']))
        print()

    if not contig:
        print('  !! 根目录簇链不连续 —— FPGA 侧必须开 ROOT_CONTIG=0（多 ~60 LUT）。')
        print('     想省这 60 LUT：把根目录里的文件全删掉重新拷一遍，目录会重新连续分配。')
        print()

    return res


def apply_root_params(path, vals):
    """把 ROOT_LBA / FAT_LBA / SPC_SHIFT / ROOT_CONTIG 写回 .v 文件。

    只改数值，行尾注释和其它内容一律不动（与 apply_to_verilog 同样的策略）。
    newline='' 保留原文件 CRLF。
    """
    with open(path, encoding='utf-8', newline='') as f:
        src = f.read()

    pats = [
        (r"(parameter\s*\[31:0\]\s*ROOT_LBA\s*=\s*32'd)\s*\d+", str(vals['root_lba']), 'ROOT_LBA'),
        (r"(parameter\s*\[31:0\]\s*FAT_LBA\s*=\s*32'd)\s*\d+", str(vals['fat_lba']), 'FAT_LBA'),
        (r"(parameter\s*integer\s*SPC_SHIFT\s*=\s*)\d+", str(vals['spc_shift']), 'SPC_SHIFT'),
        (r"(parameter\s*ROOT_CONTIG\s*=\s*)1'b[01]",
         "1'b%d" % (1 if vals['contig'] else 0), 'ROOT_CONTIG'),
    ]
    n = 0
    for pat, rep, nm in pats:
        src, k = re.subn(pat, lambda m: m.group(1) + rep, src)
        n += k
        if k == 0:
            print('  !! 没找到 %s，请手动加' % nm)

    if n == 0:
        print('  !! 一处也没改（文件里还没有这些 parameter？）')
        return False

    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(src)
    print('  已改写 %d 处根目录参数：%s' % (n, path))
    return True


def apply_to_verilog(path, pick):
    """把偏移/长度写回顶层。

    只动这几组 parameter 的**数值**，行尾注释/其它内容一律不碰。
    ★ 用 newline='' 打开：保留原文件的 CRLF，避免"改一个数、整个文件换行全变" ★
    """
    with open(path, encoding='utf-8', newline='') as f:
        src = f.read()
    n = 0
    for i, c in enumerate(pick):
        # OFFSET 行：数值 + 行尾注释里的文件名**一起**换。
        #   注释里的名字是最容易过期的东西 —— 2026-10-18 就因为注释写着"草莓"、
        #   而卡上那个偏移其实是"西瓜"，白绕了一大圈。名字必须跟着工具走。
        pat = re.compile(r"(parameter\s*\[31:0\]\s*IMG%d_OFFSET\s*=\s*32'd)\s*\d+;([^\r\n]*)"
                         % i)
        src, k = pat.subn(lambda m: '%s%d;     // %s'
                          % (m.group(1), c.phy_off, os.path.basename(c.ent.path)), src)
        n += k
        if k == 0:
            print('  !! 没在 %s 里找到 IMG%d_OFFSET，请手动改' % (path, i))
        # LENGTH 行：只换数值，行尾注释（640×480×3+54）不动
        pat = re.compile(r"(parameter\s*\[31:0\]\s*IMG%d_LENGTH\s*=\s*32'd)\s*\d+" % i)
        src, k = pat.subn(lambda m: m.group(1) + str(c.ent.size), src)
        n += k
        if k == 0:
            print('  !! 没在 %s 里找到 IMG%d_LENGTH，请手动改' % (path, i))
    if n == 0:
        print('  !! 一处也没改（文件格式和预期不一样？），已放弃写入')
        return
    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(src)
    print('  已改写 %d 处参数：%s' % (n, path))
    for i, c in enumerate(pick):
        print('    IMG%d = %d 字节 @ 偏移 %d  ← %s' % (i, c.ent.size, c.phy_off, c.ent.path))


if __name__ == '__main__':
    sys.exit(main())
