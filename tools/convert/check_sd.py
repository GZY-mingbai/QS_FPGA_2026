#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_sd.py —— 按 RTL 的规则"重放"一次根目录扫描，告诉你为什么某张图扫不到 / 装上不显示

这个脚本**只读**，不改卡上任何字节。它把 `SD/fat_dir_lister.v` + `SD/bmp_loader.v`
的判据原样复刻一遍，于是板上"看不到"的原因可以在这里直接定位：

    fat_dir_lister 的限制（本脚本会逐条检查）
      · 只看**根目录**，从根目录首扇区起**连续 64 个扇区**（top: FAT_MAX_SECS）
        —— 不跟簇链，也不看第 65 个扇区之后的目录项
      · 扩展名逐字节比 **大写 'B''M''P'**，没有大小写折叠
      · 属性 0x02（隐藏）直接跳过；卷标、子目录也跳过
      · 遇到 byte0 == 0x00 立刻当成"目录结束"，**后面的项一律看不到**
      · 一页只出 4 项（N_ITEMS=4），相册一次只要 3 张

    bmp_loader 的限制
      · 'BM' / 24bpp / comp==0 / 0<w,h<=4096 / 高度为正
      · 按 file_offset + n **连续读**，不跟 FAT 簇链 ⇒ 碎片文件必花屏
      · 文件起点必须在前 4 GiB（23 位扇区号）

用法：
    python check_sd.py E:                       # 查卡（建议管理员运行，否则看不到碎片）
    python check_sd.py E: --all                 # 连 wav/txt/子目录一起列
    python check_sd.py sim_data/fat32_10img.img # 查磁盘镜像
"""

from __future__ import annotations

import argparse
import os
import re
import struct
import sys

SECTOR = 512
FAT_MAX_SECS = 64          # top_tf_hdmi_audio.v: parameter FAT_MAX_SECS = 64
N_ITEMS = 4                # fat_dir_lister: N_ITEMS = 4
ALBUM_WIN = 3              # album_src: 相册一次只要 3 张

ATTR_READONLY = 0x01
ATTR_HIDDEN = 0x02
ATTR_SYSTEM = 0x04
ATTR_VOLUME = 0x08
ATTR_DIR = 0x10
ATTR_ARCHIVE = 0x20
ATTR_LFN = 0x0F


def u16(b, o):
    return struct.unpack("<H", b[o:o + 2])[0]


def u32(b, o):
    return struct.unpack("<I", b[o:o + 4])[0]


# ----------------------------------------------------------------------------
# 裸盘 / 镜像读取
# ----------------------------------------------------------------------------
class Disk:
    """带缓存的小块随机读。镜像文件直接读；Windows 盘符走 \\\\.\\X:"""

    def __init__(self, path: str):
        self.path = path
        self.f = open(path, "rb", buffering=0)
        self._cache: dict[int, bytes] = {}

    def read_at(self, off: int, n: int) -> bytes:
        buf = bytearray()
        while len(buf) < n:
            sec = (off + len(buf)) // SECTOR
            if sec in self._cache:
                blk = self._cache[sec]
                start = (off + len(buf)) - sec * SECTOR
                buf += blk[start:start + (n - len(buf))]
            else:
                self.f.seek(sec * SECTOR)
                blk = self.f.read(SECTOR)
                if not blk:
                    break
                self._cache[sec] = blk
                if len(self._cache) > 40000:      # 别把内存吃光
                    self._cache.clear()
        return bytes(buf)

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass


# ----------------------------------------------------------------------------
# FAT32 布局
# ----------------------------------------------------------------------------
class Geo:
    def __init__(self):
        self.part_lba = 0
        self.bps = 0
        self.spc = 0
        self.spc_shift = 0
        self.reserved = 0
        self.nfat = 0
        self.fatsz = 0
        self.root_clus = 0
        self.fat_lba = 0
        self.data_lba = 0
        self.root_lba = 0
        self.total_sec = 0
        self.is_fat32 = False


def parse_geo(disk: Disk) -> Geo:
    g = Geo()
    s0 = disk.read_at(0, 512)
    if len(s0) < 512:
        raise SystemExit("读不到 0 号扇区（权限不足或不是磁盘）")

    # 与 RTL 相同的判断：LBA0 里 bytes[82..86] == "FAT32" ⇒ superfloppy
    if s0[3:11] == b"EXFAT   ":
        raise SystemExit("这是 **exFAT** 卡：RTL 只解析 FAT32 的 BPB，exFAT 上开机一张图都扫不到。\n"
                         "  修法：重新格式化成 FAT32（32GB 及以下；Windows 的格式化对话框里选 FAT32，\n"
                         "        更大的卡需要第三方工具如 guiformat 才能格成 FAT32）。")
    if s0[3:11] == b"NTFS    ":
        raise SystemExit("这是 **NTFS** 卡：RTL 只认 FAT32，请重新格式化为 FAT32。")

    if s0[82:87] == b"FAT32":
        g.part_lba = 0
        bpb = s0
    else:
        g.part_lba = u32(s0, 454)                 # RTL 也只认第 1 个分区项
        bpb = disk.read_at(g.part_lba * SECTOR, 512)

    if len(bpb) < 512:
        raise SystemExit("读不到分区里的 BPB —— 这不是一块 FAT32 卡/镜像。")
    g.bps = u16(bpb, 11) or SECTOR
    g.spc = bpb[13] or 1
    g.reserved = u16(bpb, 14)
    g.nfat = bpb[16]
    g.fatsz = u32(bpb, 36) or u16(bpb, 22)
    g.root_clus = u32(bpb, 44) or 2
    g.total_sec = u32(bpb, 32)
    g.is_fat32 = (bpb[82:87] == b"FAT32") or g.fatsz != 0

    if g.bps not in (512, 1024, 2048, 4096) or g.nfat == 0 or g.fatsz == 0:
        raise SystemExit("BPB 不合法（不是 FAT32 卷）：确认参数是盘符（E:）或整盘镜像，"
                         "而不是其中某个文件。")

    g.spc_shift = max(0, g.spc.bit_length() - 1) if g.spc else 0
    g.fat_lba = g.part_lba + g.reserved
    g.data_lba = g.fat_lba + g.nfat * g.fatsz
    # ★ RTL 把根目录写死成"数据区第 0 个扇区"（fat_dir_lister 注释：ROOT_LBA = DATA_LBA）
    g.root_lba = g.data_lba
    return g


class Fat:
    def __init__(self, disk: Disk, g: Geo):
        self.disk = disk
        self.base = g.fat_lba * SECTOR
        self.bytes = g.fatsz * SECTOR
        self._blk: dict[int, bytes] = {}

    def next_of(self, clus: int) -> int:
        off = clus * 4
        blk_i = off // 4096
        if blk_i not in self._blk:
            self._blk[blk_i] = self.disk.read_at(self.base + blk_i * 4096, 4096)
            if len(self._blk) > 20000:
                self._blk.clear()
                self._blk[blk_i] = self.disk.read_at(self.base + blk_i * 4096, 4096)
        blk = self._blk[blk_i]
        i = off % 4096
        if i + 4 > len(blk):
            return 0x0FFFFFFF
        return u32(blk, i) & 0x0FFFFFFF

    def chain(self, start: int, limit: int) -> list[int]:
        out, c = [], start
        while 2 <= c < 0x0FFFFFF8 and len(out) < limit:
            out.append(c)
            c = self.next_of(c)
            if c in out[-4:]:                  # 环
                break
        return out


# ----------------------------------------------------------------------------
# 目录项
# ----------------------------------------------------------------------------
class Entry:
    def __init__(self, raw: bytes, sector: int, index_in_sector: int):
        self.raw = raw
        self.sector = sector
        self.idx = index_in_sector
        self.attr = raw[11]
        self.ext = raw[8:11].rstrip(b" ").decode("latin1")
        self.base = raw[0:8].rstrip(b" ").decode("latin1")
        self.clus = (u16(raw, 20) << 16) | u16(raw, 26)
        self.size = u32(raw, 28)
        self.dead = raw[0] == 0xE5
        self.end = raw[0] == 0x00
        self.lfn = (self.attr & 0x3F) == ATTR_LFN

    @property
    def name(self) -> str:
        return f"{self.base}.{self.ext}" if self.ext else self.base

    @property
    def is_bmp_rtl(self) -> bool:
        # ★ 逐字节、区分大小写，和 RTL 的 f_bmp <= (data == "B") 完全一致
        return self.raw[8:11] == b"BMP"

    @property
    def is_bmp_any(self) -> bool:
        return self.raw[8:11].upper() == b"BMP"


def read_entries(disk: Disk, g: Geo, first_lba: int, nsec: int,
                 stop_on_end: bool = True) -> list[Entry]:
    """stop_on_end=True 复刻 RTL：byte0==0x00 立刻收工。False 用于看清"洞后面还有什么"。"""
    out: list[Entry] = []
    for s in range(nsec):
        lba = first_lba + s
        blk = disk.read_at(lba * SECTOR, SECTOR)
        if len(blk) < SECTOR:
            break
        for i in range(SECTOR // 32):
            raw = blk[i * 32:(i + 1) * 32]
            e = Entry(raw, lba, i)
            out.append(e)
            if e.end and stop_on_end:
                return out
    return out


def dir_all_sectors(disk: Disk, g: Geo, fat: Fat) -> list[int]:
    """根目录真实占用的扇区（跟簇链），用来找出 RTL 64 扇区窗口漏掉的部分。"""
    out = []
    for c in fat.chain(g.root_clus, 4096):
        base = g.data_lba + ((c - 2) << g.spc_shift)
        out += list(range(base, base + g.spc))
    return out


# ----------------------------------------------------------------------------
# BMP 头
# ----------------------------------------------------------------------------
def bmp_info(disk: Disk, off: int) -> dict | None:
    d = disk.read_at(off, 54)
    if len(d) < 54 or d[0:2] != b"BM":
        return None
    return {
        "file_len": u32(d, 2),
        "data_off": u32(d, 10),
        "hdr_size": u32(d, 14),
        "w": struct.unpack("<i", d[18:22])[0],
        "h": struct.unpack("<i", d[22:26])[0],
        "bpp": u16(d, 28),
        "comp": u32(d, 30),
    }


# ----------------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------------
def check(path: str, show_all: bool) -> int:
    # ---- Windows 盘符 → 裸设备 ----
    target = path
    m = re.fullmatch(r"([A-Za-z]):[\\/]?", path)
    if m:
        target = rf"\\.\{m.group(1).upper()}:"

    try:
        disk = Disk(target)
        g = parse_geo(disk)
        raw_ok = True
    except PermissionError:
        raw_ok = False
    except OSError as e:
        if m:                                   # 盘符但没权限
            raw_ok = False
        else:
            raise SystemExit(f"打不开 {path}：{e}")

    if not raw_ok:
        return fallback_mode(path)

    fat = Fat(disk, g)
    print(f"=== {path} 根目录体检（按 RTL 判据重放）===")
    print(f"分区起点 LBA {g.part_lba} | 保留 {g.reserved} | FAT×{g.nfat} @LBA {g.fat_lba}"
          f" | 数据区 @LBA {g.data_lba}")
    print(f"扇区 {g.bps}B | 簇 {g.spc} 扇区 ({g.spc * g.bps // 1024} KB) | "
          f"根目录首扇区 {g.root_lba} | RTL 只看前 {FAT_MAX_SECS} 个扇区 = {FAT_MAX_SECS * 16} 项")
    print()

    problems_hard: list[str] = []

    # ---- 与 RTL 假设的偏差 ----
    if g.bps != 512:
        problems_hard.append(f"扇区大小 {g.bps}B ≠ 512B：RTL 的 sd_sec_read 一次固定读 512 字节，"
                             f"所有偏移都会错位。换一张标准 512B 扇区的卡。")
    if g.spc & (g.spc - 1):
        problems_hard.append(f"每簇扇区数 {g.spc} 不是 2 的幂：RTL 的簇→扇区换算会错。")
    if g.root_clus != 2:
        problems_hard.append(f"根目录起始簇是 {g.root_clus}，不是 2：RTL 把根目录写死成数据区首扇区"
                             f"（ROOT_LBA = DATA_LBA），会从错误位置开始扫。")
    if g.part_lba >= (1 << 23):
        problems_hard.append("分区起点超过 4 GiB：RTL 的扇区号只有 23 位。")

    # ---- RTL 视角：从根目录首扇区起连续 64 个扇区 ----
    rtl = read_entries(disk, g, g.root_lba, FAT_MAX_SECS, stop_on_end=True)
    # ---- 全量目录（跟簇链、不因 0x00 停）----
    all_sec = dir_all_sectors(disk, g, fat)[:4096]
    real = read_entries(disk, g, all_sec[0], len(all_sec), stop_on_end=False) if all_sec else rtl
    rtl_keys = {(e.sector, e.idx) for e in rtl}

    # 目录里第一个 byte0==0x00 的位置（RTL 到这里就收工了）
    stop_idx = next((i for i, e in enumerate(real) if e.end), len(real))
    hole_reported = False

    hdr = f"{'#':>3} {'短名':<14} {'属性':<10} {'类型':<6} {'起点(B)':>13} {'大小':>10}  RTL"
    print(hdr)
    print("-" * len(hdr))
    notes: list[tuple[int, str, list[str]]] = []
    shown_i = 0
    after_hole = 0

    for i, e in enumerate(real):
        if e.dead or e.lfn or e.end or (e.attr & ATTR_VOLUME):
            continue
        is_dir = bool(e.attr & ATTR_DIR)
        hidden = bool(e.attr & ATTR_HIDDEN)
        key = (e.sector, e.idx)
        visible = key in rtl_keys

        if i > stop_idx:
            after_hole += 1
        if not show_all and not e.is_bmp_any and not is_dir:
            continue

        shown_i += 1
        attr_txt = ",".join([n for n, b in (("R", ATTR_READONLY), ("H", ATTR_HIDDEN),
                                            ("S", ATTR_SYSTEM), ("A", ATTR_ARCHIVE))
                             if e.attr & b]) or "-"
        kind = "DIR" if is_dir else ("BMP" if e.is_bmp_rtl else ("bmp?" if e.is_bmp_any else "other"))
        off = (g.data_lba + ((e.clus - 2) << g.spc_shift)) * SECTOR if e.clus >= 2 else 0
        if is_dir:
            mark = "子目录"
        elif hidden:
            mark = "跳过H"
        elif not visible:
            mark = "**漏**"
        else:
            mark = "可见"
        print(f"{shown_i:>3} {e.name:<14} {attr_txt:<10} {kind:<6} {off:>13,} {e.size:>10,}  {mark}")

        # ---------------- 逐项诊断 ----------------
        msgs: list[str] = []
        if is_dir:
            msgs.append("子目录：INCLUDE_DIR=0，目录扫描不递归 ⇒ 放进文件夹里的图永远扫不到，"
                        "必须放根目录。")
        if hidden:
            msgs.append("属性带 0x02（隐藏）：fat_dir_lister 的 is_hidden 直接跳过这一项。"
                        "（macOS 拷文件常带这个；Windows 下 `attrib -h 文件名` 可去掉）")
        if not e.is_bmp_rtl:
            if e.is_bmp_any:
                msgs.append(f"扩展名落盘是 '{e.ext}'：RTL 逐字节比 'B''M''P'（区分大小写），"
                            f"这一项被判成 type=0 —— 文件页能看到名字，相册/四宫格跳过，装载器也不启动。")
                msgs.append("  修法：在 Windows 上把扩展名改成 .bmp/.BMP 让它重写目录项，或整文件重拷一遍。")
            elif not is_dir:
                msgs.append("不是 .BMP 扩展名（相册只取 type=1 的项）。")
        if not visible and not hidden and not is_dir:
            if i > stop_idx:
                if not hole_reported:
                    msgs.append(f"它排在第一个 0x00 目录项（扇区 {real[stop_idx].sector} 第 {real[stop_idx].idx} 项）之后："
                                f"RTL 一见 0x00 就当成目录结束，后面的项全部扫不到。"
                                f"修法：删掉那个空洞项（chkdsk 或重新格式化再拷）。")
                    hole_reported = True
                else:
                    msgs.append("同上：位于 0x00 空洞之后，RTL 扫不到（这就是「第 N 张以后全没了」的典型原因）。")
            else:
                msgs.append(f"目录项在扇区 {e.sector}，超出 RTL 的 {FAT_MAX_SECS} 扇区扫描窗口"
                            f"（从根目录首扇区 {g.root_lba} 起算）—— 根目录项太多或目录簇不连续。")
        if e.clus < 2 and not is_dir:
            msgs.append("起始簇为 0（文件被截断/簇链损坏）。")

        info = bmp_info(disk, off) if (off and e.is_bmp_rtl and not is_dir) else None
        if e.is_bmp_rtl and not is_dir:
            if info is None:
                msgs.append(f"在 {off:,} 字节处读不到 'BM' 头 —— 起始簇指向了别处（碎片/簇链/目录项损坏）。")
            else:
                if info["bpp"] != 24:
                    msgs.append(f"位深 {info['bpp']}：要求 24 位（32 位带 alpha 的 BMP 会被拒）。")
                if info["comp"] != 0:
                    msgs.append(f"压缩方式 {info['comp']}：要求 0（BI_RGB）。RLE 压缩的不认。")
                if not (0 < info["w"] <= 4096 and 0 < info["h"] <= 4096):
                    msgs.append(f"尺寸 {info['w']}x{info['h']} 超出 1..4096，hdr_match 直接判非法。")
                if info["h"] == 4096 or info["w"] == 4096:
                    msgs.append("宽/高正好 4096：RTL 多处按 12 位截断，0x1000 会变成 0，建议降到 4095 以内。")
                stride = (info["w"] * 3 + 3) & ~3
                need = info["data_off"] + stride * max(info["h"], 0)
                if info["file_len"] != e.size:
                    msgs.append(f"头里的文件长度 {info['file_len']:,} 与目录项的 {e.size:,} 不一致。")
                if e.size < need:
                    msgs.append(f"文件只有 {e.size:,} 字节，按 {info['w']}x{info['h']} 至少要 {need:,}。")
                # ---- 碎片 ----
                clus_bytes = g.spc * SECTOR
                need_clus = max(1, -(-e.size // clus_bytes))
                chain = fat.chain(e.clus, need_clus + 8)
                segs, prev = 1, (chain[0] if chain else 0)
                for c in chain[1:]:
                    if c != prev + 1:
                        segs += 1
                    prev = c
                if len(chain) < need_clus:
                    msgs.append(f"簇链只走到 {len(chain)} 个簇，不够 {need_clus} 个（文件被截断或簇链损坏）"
                                + (f"；途中已经跳了 {segs - 1} 次" if segs > 1 else ""))
                elif segs > 1:
                    msgs.append(f"文件碎成 {segs} 段：装载器按 file_offset+n 连续读、**不跟簇链** ⇒ "
                                f"头能过，像素却从别的簇取（花屏/黑屏/半张图）。"
                                f"修法：卡上文件删干净→重新格式化→整批拷一次，或用 sync_to_sd.py。")
                if off + e.size > (1 << 32):
                    msgs.append("文件起点超过 4 GiB：RTL 的扇区号只有 23 位，读不到。")

        if msgs:
            notes.append((shown_i, e.name, msgs))

    print()
    # ---- 问题清单 ----
    if problems_hard:
        print("!! 卡本身不符合 RTL 假设：")
        for m_ in problems_hard:
            print("   - " + m_)
        print()
    if notes:
        print("!! 逐项问题：")
        for i, nm, msgs in notes:
            print(f"  [{i}] {nm}")
            for m_ in msgs:
                print("      " + m_)
        print()
    else:
        print("逐项检查没有发现问题。")
        print()

    # ---- 结论 ----
    rtl_items = [e for e in real if (e.sector, e.idx) in rtl_keys
                 and not (e.dead or e.lfn or e.end or (e.attr & ATTR_VOLUME)
                          or (e.attr & (ATTR_HIDDEN | ATTR_DIR)))]
    rtl_bmp = sum(1 for e in rtl_items if e.is_bmp_rtl)
    print(f"结论：RTL 这次扫描能看到 {len(rtl_items)} 个有效项，其中 {rtl_bmp} 个是 .BMP；"
          f"文件页一页 {N_ITEMS} 项，相册一次 {ALBUM_WIN} 张。")
    if rtl_bmp > ALBUM_WIN:
        print(f"      第 {ALBUM_WIN + 1} 张以后不会出现在相册首屏，要靠滑动窗口/进文件页翻页。")
    if rtl_bmp > N_ITEMS:
        print(f"      文件页每页 {N_ITEMS} 项，第 {N_ITEMS + 1} 项起要翻页（翻页键）。")
    hidden_files = [e.name for e in real
                    if (e.attr & ATTR_HIDDEN) and not (e.attr & (ATTR_DIR | ATTR_VOLUME))
                    and not (e.lfn or e.dead or e.end)]
    if hidden_files:
        print(f"      隐藏项 {len(hidden_files)} 个（跳过）：{', '.join(hidden_files[:6])}")
    if after_hole:
        print(f"      0x00 空洞之后还有 {after_hole} 项，RTL 全部扫不到。")
    missed = [e.name for e in real
              if (e.sector, e.idx) not in rtl_keys and not (e.dead or e.lfn or e.end)]
    if missed:
        print(f"      64 扇区窗口之外还有 {len(missed)} 项（RTL 扫不到）：{', '.join(missed[:6])}")

    disk.close()
    return 0


# ----------------------------------------------------------------------------
# 降级模式：没有裸盘权限时，只能按"文件系统视图"检查
# ----------------------------------------------------------------------------
def fallback_mode(path: str) -> int:
    m = re.fullmatch(r"([A-Za-z]):[\\/]?", path)
    root = f"{m.group(1).upper()}:\\" if m else path.rstrip("\\/") + "\\"
    print("!! 没有裸盘读权限（Windows 需要以管理员身份运行），只能做文件层面的检查：")
    print("   顺序、偏移、碎片程度这三项无法判断。要完整诊断请右键『以管理员身份运行』。\n")
    try:
        items = list(os.scandir(root))
    except OSError as e:
        raise SystemExit(f"打不开目录 {root}：{e}")

    print(f"{'名字':<28} {'大小':>12}  检查")
    print("-" * 72)
    bad = 0
    for it in items:
        try:
            st = it.stat()
        except OSError:
            continue
        if not it.is_file():
            continue
        hidden = bool(getattr(st, "st_file_attributes", 0) & 0x2)
        ext = os.path.splitext(it.name)[1]
        msgs = []
        if ext.lower() == ".bmp" and ext != ".BMP":
            msgs.append(f"扩展名 '{ext}'：RTL 只认落盘为 BMP 的大写形式（Windows 改名一次即可）")
        if hidden:
            msgs.append("隐藏属性 → 目录扫描跳过")
        if ext.lower() == ".bmp":
            try:
                with open(it.path, "rb") as f:
                    d = f.read(54)
                if len(d) < 54 or d[:2] != b"BM":
                    msgs.append("不是 'BM' 开头")
                else:
                    w, h = struct.unpack("<ii", d[18:26])
                    bpp = u16(d, 28)
                    comp = u32(d, 30)
                    if bpp != 24 or comp != 0:
                        msgs.append(f"bpp={bpp} comp={comp}（要求 24/0）")
                    if not (0 < w <= 4096 and 0 < h <= 4096):
                        msgs.append(f"{w}x{h} 超出 1..4096")
                    stride = (w * 3 + 3) & ~3
                    need = u32(d, 10) + stride * max(h, 0)
                    if st.st_size < need:
                        msgs.append(f"只有 {st.st_size:,} 字节，至少需要 {need:,}")
            except OSError as e:
                msgs.append(f"读失败：{e}")
        if msgs:
            bad += 1
            print(f"{it.name:<28} {st.st_size:>12,}  " + "; ".join(msgs))
        else:
            print(f"{it.name:<28} {st.st_size:>12,}  ok")
    print("-" * 72)
    print(f"共 {bad} 个文件存在可疑点（碎片/顺序仍需管理员权限才能查）。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="按 RTL 规则体检 SD 卡根目录（只读）")
    ap.add_argument("target", help="盘符（E:）或磁盘镜像路径")
    ap.add_argument("--all", action="store_true", help="连非 BMP 项一起列")
    args = ap.parse_args()
    return check(args.target, args.all)


if __name__ == "__main__":
    sys.exit(main())
