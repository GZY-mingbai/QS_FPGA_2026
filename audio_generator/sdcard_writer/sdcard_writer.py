#!/usr/bin/env python3
"""sdcard_writer.py —— 按 JSON 清单格式化 SD 卡并按序写入文件。

分工：
    WSL2  : mkfs.vfat + mtools 构建 FAT32 镜像（顺序保证、簇布局可控）
    Windows: 裸写 \\\\.\\PhysicalDriveN（卷锁定 + 扇区对齐写 + 跳过全零块）

用法：
    python sdcard_writer.py spec.json --list          # 列出所有磁盘
    python sdcard_writer.py spec.json --dry-run --size 512M   # 只造镜像
    python sdcard_writer.py spec.json                 # 造镜像 + 烧卡

需要管理员权限（裸写物理盘）。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import rawdisk  # noqa: E402


# --------------------------------------------------------------------- 工具
def eprint(*a):
    print(*a, file=sys.stderr)


def human(n: int) -> str:
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if n < 1024 or unit == "TB":
            return "%.2f %s" % (n, unit) if unit != "B" else "%d B" % n
        n /= 1024.0
    return str(n)


def win_to_wsl(path: str) -> str:
    p = os.path.abspath(path).replace("\\", "/")
    if len(p) > 1 and p[1] == ":":
        return "/mnt/%s%s" % (p[0].lower(), p[2:])
    return p


def parse_size(text: str) -> int:
    """支持 512M / 32G / 123456789 三种写法。"""
    text = str(text).strip().upper()
    mult = 1
    for suffix, m in (("G", 1 << 30), ("M", 1 << 20), ("K", 1 << 10)):
        if text.endswith(suffix):
            mult = m
            text = text[:-1]
            break
    return int(float(text) * mult)


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def is_admin() -> bool:
    try:
        import ctypes
        return bool(ctypes.windll.shell32.IsUserAnAdmin())
    except Exception:
        return False


# ------------------------------------------------------------------ WSL 构建
def check_wsl() -> None:
    try:
        r = subprocess.run(
            ["wsl.exe", "-e", "bash", "-lc", "command -v mkfs.vfat && command -v mcopy"],
            capture_output=True, text=True,
        )
    except FileNotFoundError:
        sys.exit("未找到 wsl.exe。请安装 WSL2，或在 Linux 上直接运行。")
    if r.returncode != 0:
        sys.exit(
            "WSL 缺少依赖，请先执行:\n  wsl -e sudo apt install -y dosfstools mtools\n"
            "实际输出: %s" % (r.stderr.strip() or r.stdout.strip())
        )


def build_image(spec_path: str, img_win: str, size: int, cluster: int, mbr: bool) -> None:
    script = os.path.join(HERE, "build_img.sh")
    if not os.path.isfile(script):
        sys.exit("缺少 build_img.sh：%s" % script)

    # 镜像先落在 WSL 原生文件系统（drvfs 上建稀疏镜像会慢 5~10 倍），
    # 再由脚本 cp --sparse=always 到 Windows 侧。
    out_wsl = "/tmp/sdcard_build/card.img"
    cmd = [
        "wsl.exe", "-e", "bash", win_to_wsl(script),
        win_to_wsl(spec_path), out_wsl, str(size),
        win_to_wsl(img_win), str(cluster), "1" if mbr else "0",
    ]
    eprint("[1/3] 在 WSL 中构建镜像 ...")
    eprint("      " + " ".join(cmd))
    t0 = time.time()
    rc = subprocess.run(cmd).returncode
    if rc != 0:
        sys.exit("镜像构建失败 (exit=%d)" % rc)
    eprint("      完成，用时 %.1fs" % (time.time() - t0))

    if not os.path.isfile(img_win):
        sys.exit("镜像未出现在 Windows 侧：%s（检查 /mnt/c 映射与 cp 是否成功）" % img_win)


# ---------------------------------------------------------------------- 主流程
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="按 JSON 清单格式化 SD 卡（FAT32）并按序写入文件",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("spec", nargs="?", help="清单 JSON 路径")
    ap.add_argument("--target", help="目标盘符，如 E 或 E:，覆盖 spec.card.target")
    ap.add_argument("--image", help="镜像输出路径，覆盖 spec.build.image")
    ap.add_argument("--size", help="镜像大小，如 512M / 32G；默认取卡的物理容量")
    ap.add_argument("--fit", action="store_true",
                    help="镜像只按内容大小裁剪（不铺满整卡），速度快 5~8 倍")
    ap.add_argument("--cluster", type=int, help="每簇扇区数，默认 8（4KB 簇）")
    ap.add_argument("--mbr", dest="mbr", action="store_true", help="写 MBR 分区表")
    ap.add_argument("--no-mbr", dest="mbr", action="store_false", help="superfloppy（默认）")
    ap.add_argument("--dry-run", action="store_true", help="只造镜像，不写卡")
    ap.add_argument("--list", action="store_true", help="列出物理磁盘后退出")
    ap.add_argument("--yes", "-y", action="store_true", help="跳过二次确认")
    ap.add_argument("--keep-image", action="store_true", help="烧完保留镜像文件")
    ap.add_argument("--verify", action="store_true", help="烧完回读元数据区做 hash 比对")
    ap.set_defaults(mbr=None)
    args = ap.parse_args(argv)

    # --list ------------------------------------------------------------
    if args.list:
        out = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command",
             "Get-Disk | Select-Object Number,FriendlyName,BusType,"
             "@{n='SizeGB';e={[math]::Round($_.Size/1GB,2)}},IsSystem | Format-Table -AutoSize"],
            capture_output=True, text=True,
        ).stdout
        print(out)
        return 0

    if not args.spec:
        ap.error("需要 spec.json（或用 --list 查看磁盘）")

    # ---- 读 spec ----
    if not os.path.isfile(args.spec):
        sys.exit("spec 不存在: %s" % args.spec)
    with open(args.spec, encoding="utf-8") as f:
        spec = json.load(f)

    card = spec.setdefault("card", {})
    build = spec.setdefault("build", {})
    files = spec.get("files") or []
    if not files:
        sys.exit("spec.files 为空")

    target = args.target or card.get("target")
    cluster = args.cluster or int(card.get("clusterSectors") or 8)
    mbr = bool(card.get("mbr", False)) if args.mbr is None else args.mbr
    img_win = os.path.abspath(args.image or build.get("image")
                              or os.path.join(HERE, "build", "card.fat"))

    cluster_bytes = cluster * 512

    # ---- 解析目标磁盘 ----
    disk_no = None
    disk_bytes = 0
    bps = 512

    if target and not args.dry_run:
        disk_no = rawdisk.disk_of_drive(target)
        if rawdisk.is_system_disk(disk_no):
            sys.exit("拒绝操作：磁盘 %d 是系统盘" % disk_no)
        with rawdisk.physical_handle(disk_no, write=False) as h:
            bps, disk_bytes = rawdisk.geometry(h)

    # ---- 相对路径按 spec 所在目录解析，落一份 resolved spec 交给 WSL ----
    # （build_img.sh 在 WSL 里跑，相对路径会以 WSL 的 cwd 为基准，必须提前绝对化）
    spec_dir = os.path.dirname(os.path.abspath(args.spec))
    resolved = json.loads(json.dumps(spec))
    resolved.setdefault("card", {})["label"] = card.get("label", "NONAME")
    for it in resolved.get("files", []):
        if not os.path.isabs(it["src"]):
            it["src"] = os.path.normpath(os.path.join(spec_dir, it["src"]))
    os.makedirs(os.path.dirname(img_win), exist_ok=True)
    resolved_path = os.path.join(os.path.dirname(img_win), "spec.resolved.json")
    with open(resolved_path, "w", encoding="utf-8") as f:
        json.dump(resolved, f, ensure_ascii=False, indent=2)

    total_src = 0
    for it in resolved["files"]:
        p = it["src"]
        if not os.path.isfile(p):
            sys.exit("源文件不存在: %s" % p)
        total_src += os.path.getsize(p)

    # ---- 决定镜像大小 ----
    # fit 模式：镜像只按内容大小造（约 800MB），而不是铺满整张 8GB 卡。
    # 代价是卡上的卷容量变成 800MB（其余空间不可用），换来构建/拷贝/哈希都只处理 800MB。
    fit = args.fit or str(card.get("sizeMode") or "").lower() == "fit"
    if args.size and fit:
        sys.exit("--fit 与 --size 不能同时使用")

    if args.size:
        size = parse_size(args.size)
    elif fit:
        # 内容 + 5% 余量 + 32MB（簇对齐损耗、双 FAT、目录项）
        size = total_src * 105 // 100 + (32 << 20)
    elif card.get("sizeBytes"):
        size = int(card["sizeBytes"])
    elif disk_bytes:
        size = disk_bytes
    else:
        sys.exit("无法确定镜像大小：dry-run 模式下请用 --fit，或指定 --size 512M")

    size = size // cluster_bytes * cluster_bytes  # 必须簇对齐
    if disk_bytes and size > disk_bytes:
        sys.exit("镜像 %s 超过卡容量 %s" % (human(size), human(disk_bytes)))

    # ---- 打印计划并二次确认 ----

    print("=" * 68)
    print("  目标盘符 : %s" % (target or "(dry-run)"))
    if disk_no is not None:
        print("  磁盘号   : PhysicalDrive%d" % disk_no)
        for line in rawdisk.describe_disk(disk_no).splitlines():
            if line.strip():
                print("             " + line.strip())
    print("  镜像路径 : %s" % img_win)
    print("  镜像大小 : %s (簇 %d 扇区 = %d 字节)" % (human(size), cluster, cluster_bytes))
    print("  卷标     : %s" % card.get("label", "NONAME"))
    print("  分区表   : %s" % ("MBR (0x0C)" if mbr else "superfloppy（无分区表）"))
    print("  文件数量 : %d 个，共 %s" % (len(files), human(total_src)))
    if fit:
        print("  容量模式 : 按内容裁剪 — 卡上卷容量将是 %s（其余空间不可用）" % human(size))
    else:
        print("  容量模式 : 铺满整卡 — 卷容量 %s" % human(size))
        print("  余量     : %s" % human(max(size - total_src, 0)))
    print("=" * 68)
    print("!! 该操作会清空目标设备上的全部数据 !!")
    print("=" * 68)

    if not args.dry_run and not args.yes:
        answer = input("确认继续请输入盘符 %s: " % str(target).rstrip(":").upper()).strip()
        if answer.upper() != str(target).rstrip(":").upper():
            sys.exit("已取消")

    # ---- 造镜像 ----
    check_wsl()
    build_image(resolved_path, img_win, size, cluster, mbr)
    img_size = os.path.getsize(img_win)
    if args.verify:
        eprint("[1/3] 镜像 %s  sha256=%s" % (human(img_size), sha256_file(img_win)[:16] + "..."))
    else:
        eprint("[1/3] 镜像 %s（--verify 可附带 sha256）" % human(img_size))

    if args.dry_run:
        eprint("dry-run 结束，未写卡。可用 Win32DiskImager 手动验证该镜像。")
        return 0

    # ---- 锁定卷 + 裸写 ----
    if not is_admin():
        sys.exit("需要管理员权限才能裸写物理盘：请以管理员身份打开终端后重试")
    eprint("[2/3] 锁定并卸载目标盘上的卷 ...")
    t0 = time.time()
    last = [0.0]

    def progress(done, total):
        now = time.time()
        if now - last[0] < 0.25 and done < total:
            return
        last[0] = now
        sys.stderr.write("\r      已写入 %5.1f%%  %8s / %s"
                         % (done * 100.0 / max(total, 1), human(done), human(total)))
        if done >= total:
            sys.stderr.write("\n")

    with rawdisk.locked_volumes(disk_no):
        with rawdisk.physical_handle(disk_no, write=True) as h:
            eprint("[3/3] 烧录中（跳过全零块）...")
            stats = rawdisk.burn_image(h, img_win, progress=progress)
            rawdisk.flush(h)
            rawdisk.update_properties(h)

    elapsed = time.time() - t0
    eprint("      完成，用时 %.1fs，写入 %d / %d 块（跳过 %d 块全零）"
           % (elapsed, stats["blocks_written"], stats["blocks_total"],
              stats["blocks_skipped"]))

    # ---- 可选回读校验（只比元数据区，验证写偏移没算错）----
    if args.verify:
        eprint("[verify] 回读元数据区并比对 ...")
        meta_end = min(stats["metadata_end"], 64 << 20)
        path = r"\\.\PhysicalDrive%d" % disk_no
        try:
            with open(path, "rb") as f:
                disk_hash = hashlib.sha256(f.read(meta_end)).hexdigest()
            with open(img_win, "rb") as f:
                img_hash = hashlib.sha256(f.read(meta_end)).hexdigest()
            eprint("[verify] 磁盘=%s 镜像=%s -> %s"
                   % (disk_hash[:16], img_hash[:16],
                      "OK" if disk_hash == img_hash else "不一致！"))
        except OSError as exc:
            eprint("[verify] 回读失败（卷可能已被系统重新挂载）: %s" % exc)

    if not args.keep_image:
        try:
            os.remove(img_win)
            eprint("已删除镜像 %s（--keep-image 可保留）" % img_win)
        except OSError:
            pass

    print()
    print("烧录完成。请拔插一次读卡器，让系统重新枚举卷。")
    if fit:
        print("注意：本次为裁剪模式，卷容量 %s（未铺满 %s 的卡）。"
              % (human(size), human(disk_bytes) if disk_bytes else "整卡"))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit("已中断")
    except rawdisk.DiskError as exc:
        sys.exit("磁盘操作失败: %s" % exc)
