#!/usr/bin/env python3
"""rawdisk.py —— Windows 裸盘写入原语。

提供四件事：
  1. 盘符 / 磁盘号  与  \\\\.\\PhysicalDriveN  的互相解析
  2. 查询物理扇区大小与磁盘容量（IOCTL_DISK_GET_DRIVE_GEOMETRY_EX）
  3. 锁定 + 卸载盘上所有卷（不卸载会被系统的缓存回写覆盖，且引导区写入被拒）
  4. 扇区对齐的裸写入（FILE_FLAG_NO_BUFFERING 要求缓冲区地址/长度/偏移全部对齐）

注意：所有操作需要管理员权限。
"""
from __future__ import annotations

import contextlib
import ctypes
import ctypes.wintypes as wintypes
import json
import subprocess
import sys
import time

# ---------------------------------------------------------------- Win32 常量
k32 = ctypes.WinDLL("kernel32", use_last_error=True)

GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
FILE_SHARE_READ = 0x00000001
FILE_SHARE_WRITE = 0x00000002
OPEN_EXISTING = 3
FILE_ATTRIBUTE_NORMAL = 0x00000080
FILE_FLAG_NO_BUFFERING = 0x20000000
FILE_FLAG_WRITE_THROUGH = 0x80000000

IOCTL_DISK_GET_DRIVE_GEOMETRY_EX = 0x000700A0
IOCTL_DISK_UPDATE_PROPERTIES = 0x00070140
FSCTL_LOCK_VOLUME = 0x00090018
FSCTL_DISMOUNT_VOLUME = 0x00090020

MEM_COMMIT = 0x00001000
MEM_RESERVE = 0x00002000
MEM_RELEASE = 0x00008000
PAGE_READWRITE = 0x04

INVALID_HANDLE_VALUE = ctypes.c_void_p(-1).value
CLOSE_HANDLE = ctypes.wintypes.HANDLE


class DiskError(RuntimeError):
    pass


def _err(what: str) -> DiskError:
    return DiskError("%s 失败 (GetLastError=%d)" % (what, ctypes.get_last_error()))


k32.CreateFileW.restype = ctypes.c_void_p
k32.CreateFileW.argtypes = [
    wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, ctypes.c_void_p,
    wintypes.DWORD, wintypes.DWORD, ctypes.c_void_p,
]
k32.DeviceIoControl.restype = wintypes.BOOL
k32.DeviceIoControl.argtypes = [
    ctypes.c_void_p, wintypes.DWORD, ctypes.c_void_p, wintypes.DWORD,
    ctypes.c_void_p, wintypes.DWORD, ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p,
]
k32.WriteFile.restype = wintypes.BOOL
k32.WriteFile.argtypes = [
    ctypes.c_void_p, ctypes.c_void_p, wintypes.DWORD,
    ctypes.POINTER(wintypes.DWORD), ctypes.c_void_p,
]
k32.SetFilePointerEx.restype = wintypes.BOOL
k32.SetFilePointerEx.argtypes = [
    ctypes.c_void_p, ctypes.c_int64, ctypes.c_void_p, wintypes.DWORD,
]
k32.VirtualAlloc.restype = ctypes.c_void_p
k32.VirtualAlloc.argtypes = [ctypes.c_void_p, ctypes.c_size_t, wintypes.DWORD, wintypes.DWORD]
k32.VirtualFree.restype = wintypes.BOOL
k32.VirtualFree.argtypes = [ctypes.c_void_p, ctypes.c_size_t, wintypes.DWORD]
k32.CloseHandle.restype = wintypes.BOOL
k32.CloseHandle.argtypes = [ctypes.c_void_p]


class DISK_GEOMETRY_EX(ctypes.Structure):
    _fields_ = [
        ("Cylinders", ctypes.c_int64),
        ("MediaType", wintypes.DWORD),
        ("TracksPerCylinder", wintypes.DWORD),
        ("SectorsPerTrack", wintypes.DWORD),
        ("BytesPerSector", wintypes.DWORD),
        ("DiskSize", ctypes.c_int64),
    ]


# ------------------------------------------------------------------ 句柄工具
@contextlib.contextmanager
def physical_handle(disk_no: int, write: bool = True):
    path = r"\\.\PhysicalDrive%d" % disk_no
    access = GENERIC_READ | (GENERIC_WRITE if write else 0)
    flags = FILE_ATTRIBUTE_NORMAL | FILE_FLAG_NO_BUFFERING | FILE_FLAG_WRITE_THROUGH
    h = k32.CreateFileW(path, access, FILE_SHARE_READ | FILE_SHARE_WRITE,
                        None, OPEN_EXISTING, flags, None)
    if not h or h == INVALID_HANDLE_VALUE:
        raise _err("打开 %s（请用管理员身份运行）" % path)
    try:
        yield h
    finally:
        k32.CloseHandle(h)


def geometry(h) -> tuple[int, int]:
    """返回 (扇区字节数, 磁盘总字节数)。"""
    buf = ctypes.create_string_buffer(1024)
    ret = wintypes.DWORD()
    if not k32.DeviceIoControl(h, IOCTL_DISK_GET_DRIVE_GEOMETRY_EX, None, 0,
                               buf, len(buf), ctypes.byref(ret), None):
        raise _err("IOCTL_DISK_GET_DRIVE_GEOMETRY_EX")
    geo = ctypes.cast(buf, ctypes.POINTER(DISK_GEOMETRY_EX)).contents
    return geo.BytesPerSector, geo.DiskSize


def update_properties(h) -> None:
    """写完必须调用，否则资源管理器/系统仍显示旧分区信息。"""
    ret = wintypes.DWORD()
    k32.DeviceIoControl(h, IOCTL_DISK_UPDATE_PROPERTIES, None, 0, None, 0,
                        ctypes.byref(ret), None)


def flush(h) -> None:
    k32.FlushFileBuffers(h)


# ------------------------------------------------------- 盘符 / 磁盘号解析
def _powershell(script: str) -> str:
    r = subprocess.run(
        ["powershell", "-NoProfile", "-NonInteractive", "-Command", script],
        capture_output=True, text=True,
    )
    if r.returncode != 0:
        raise DiskError("PowerShell 失败：%s" % (r.stderr.strip() or r.stdout.strip()))
    return r.stdout


def disk_of_drive(letter: str) -> int:
    """'E' / 'E:' -> 磁盘号。"""
    letter = letter.strip().rstrip(":").upper()[:1]
    if not letter.isalpha():
        raise DiskError("非法盘符: %r" % letter)
    out = _powershell(
        "Get-Partition -DriveLetter %s | Get-Disk | Select-Object -ExpandProperty Number"
        % letter
    ).strip()
    if not out.isdigit():
        raise DiskError("无法解析盘符 %s 对应的磁盘号，输出=%r" % (letter, out))
    return int(out)


def volumes_of_disk(disk_no: int) -> list[str]:
    """返回该磁盘上所有已分配盘符的卷（如 ['E', 'F']）。"""
    out = _powershell(
        "$ErrorActionPreference='SilentlyContinue';"
        "Get-Partition -DiskNumber %d | Where-Object {$_.DriveLetter} |"
        "ForEach-Object { $_.DriveLetter }" % disk_no
    )
    return [ln.strip() for ln in out.splitlines() if ln.strip()]


def describe_disk(disk_no: int) -> str:
    """人类可读的磁盘摘要，用于二次确认。"""
    out = _powershell(
        "Get-Disk -Number %d | Select-Object Number,FriendlyName,SerialNumber,BusType,"
        "@{n='SizeGB';e={[math]::Round($_.Size/1GB,2)}} | Format-List" % disk_no
    )
    return out.strip()


def removable(disk_no: int) -> bool:
    out = _powershell(
        "Get-Disk -Number %d | Select-Object -ExpandProperty IsSystem" % disk_no
    ).strip()
    return out.lower() not in ("true", "1")


def is_system_disk(disk_no: int) -> bool:
    out = _powershell(
        "Get-Disk -Number %d | Select-Object -ExpandProperty IsSystem" % disk_no
    ).strip()
    return out.lower() in ("true", "1")


# ---------------------------------------------------------- 卷锁定 / 卸载
@contextlib.contextmanager
def locked_volumes(disk_no: int, retries: int = 10, delay: float = 0.5):
    """锁定并卸载该磁盘上所有卷，句柄在 with 退出前保持打开。

    Lock 会在句柄关闭时失效，所以必须一直持有到写入结束。
    """
    handles: list = []
    try:
        for letter in volumes_of_disk(disk_no):
            path = r"\\.\%s:" % letter
            h = k32.CreateFileW(path, GENERIC_READ | GENERIC_WRITE,
                                FILE_SHARE_READ | FILE_SHARE_WRITE,
                                None, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, None)
            if not h or h == INVALID_HANDLE_VALUE:
                raise _err("打开卷 %s:（是否被资源管理器/杀软占用？）" % path)
            handles.append(h)

            ret = wintypes.DWORD()
            ok = False
            for attempt in range(retries):
                if k32.DeviceIoControl(h, FSCTL_LOCK_VOLUME, None, 0, None, 0,
                                       ctypes.byref(ret), None):
                    ok = True
                    break
                time.sleep(delay)
            if not ok:
                raise DiskError(
                    "锁定卷 %s: 失败（请关闭该卡的所有资源管理器窗口/预览/杀软扫描）" % path
                )
            if not k32.DeviceIoControl(h, FSCTL_DISMOUNT_VOLUME, None, 0, None, 0,
                                       ctypes.byref(ret), None):
                raise _err("卸载卷 %s:" % path)
            print("[rawdisk] 已锁定并卸载卷 %s:" % letter, file=sys.stderr)
        yield
    finally:
        for h in handles:
            k32.CloseHandle(h)


# ------------------------------------------------------------------ 裸写入
def write_at(h, offset: int, data: bytes) -> None:
    """在 offset 处写入 data。要求 offset / len(data) 均为物理扇区整数倍。"""
    size = len(data)
    buf = k32.VirtualAlloc(None, size, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE)
    if not buf:
        raise _err("VirtualAlloc(%d)" % size)
    try:
        ctypes.memmove(buf, data, size)
        if not k32.SetFilePointerEx(h, offset, None, 0):
            raise _err("SetFilePointerEx(%d)" % offset)
        written = wintypes.DWORD()
        if not k32.WriteFile(h, buf, size, ctypes.byref(written), None):
            raise _err("WriteFile @%d" % offset)
        if written.value != size:
            raise DiskError("短写 @%d: %d/%d" % (offset, written.value, size))
    finally:
        k32.VirtualFree(buf, 0, MEM_RELEASE)


def image_layout(path: str) -> dict:
    """解析镜像的 FAT32 BPB，算出必须完整写入的元数据区长度。

    只有元数据区之前的字节（引导扇区 + 双 FAT + MBR）必须逐字节覆盖，
    数据区里未被使用的簇是全零，可以直接跳过以大幅缩短烧录时间。
    """
    with open(path, "rb") as f:
        head = f.read(4096)
    if len(head) < 512:
        raise DiskError("镜像太小: %s" % path)

    def u16(b, o):
        return int.from_bytes(b[o:o + 2], "little")

    def u32(b, o):
        return int.from_bytes(b[o:o + 4], "little")

    base = 0
    if head[510:512] != b"\x55\xAA":
        raise DiskError("镜像缺少引导扇区结束标记 0xAA55，不是有效的 FAT 镜像")

    # 0xEB/0xE9 开头且在偏移 82 处有 "FAT32   " 才算 FAT32 BPB
    is_fat32_at0 = head[82:90] == b"FAT32   "
    if not is_fat32_at0:
        # 尝试解析 MBR 分区表，找类型 0x0B/0x0C 的分区
        for i in range(4):
            e = 446 + i * 16
            ptype = head[e + 4]
            if ptype in (0x0B, 0x0C, 0x0E):
                lba = u32(head, e + 8)
                with open(path, "rb") as f:
                    f.seek(lba * 512)
                    bpb = f.read(512)
                if bpb[82:90] != b"FAT32   ":
                    continue
                base = lba * 512
                head = bpb + head[512:4096]
                break
        else:
            raise DiskError("未找到 FAT32 BPB（既不是 superfloppy 也不是带 0x0C 分区的 MBR）")

    byts_per_sec = u16(head, 11)
    rsvd = u16(head, 14)
    num_fats = head[16]
    fat_sz32 = u32(head, 36)
    if byts_per_sec not in (512, 1024, 2048, 4096) or rsvd == 0 or fat_sz32 == 0:
        raise DiskError("BPB 字段异常: bps=%d rsvd=%d fats=%d fatsz=%d"
                        % (byts_per_sec, rsvd, num_fats, fat_sz32))

    data_start = (rsvd + num_fats * fat_sz32) * byts_per_sec
    return {
        "bytes_per_sector": byts_per_sec,
        "partition_offset": base,
        "metadata_end": base + data_start,
        "fat_sectors": fat_sz32,
        "num_fats": num_fats,
    }


def burn_image(h, path: str, chunk: int = 4 << 20, progress=None) -> dict:
    """把镜像烧到已锁定的物理盘，跳过数据区里的全零块。

    跳零块的安全性：FAT 表已声明这些簇为空闲，卡上残留的旧数据不会被任何
    文件引用；而元数据区（MBR/引导扇区/双 FAT）永远整块写入，不会漏。
    """
    layout = image_layout(path)
    meta_end = layout["metadata_end"]
    bps = layout["bytes_per_sector"]

    total = 0
    written_blocks = 0
    skipped_blocks = 0
    total_blocks = 0

    with open(path, "rb") as f:
        f.seek(0, 2)
        total = f.tell()
        f.seek(0)
        offset = 0
        while True:
            buf = f.read(chunk)
            if not buf:
                break
            pad = (-len(buf)) % bps
            if pad:
                buf += b"\x00" * pad
            offset_end = offset + len(buf)
            total_blocks += 1

            must_write = offset < meta_end
            if not must_write and buf.count(0) != len(buf):
                must_write = True

            if must_write:
                write_at(h, offset, buf)
                written_blocks += 1
            else:
                skipped_blocks += 1

            offset = offset_end
            if progress:
                progress(min(offset, total), total)

    return {
        "image_bytes": total,
        "metadata_end": meta_end,
        "blocks_written": written_blocks,
        "blocks_skipped": skipped_blocks,
        "blocks_total": total_blocks,
    }
