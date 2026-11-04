#!/usr/bin/env bash
# ============================================================================
# build_img.sh —— 在 WSL 内构建 FAT32 镜像（顺序保证）
#
# 用法:
#   build_img.sh <spec.json> <out.img> <size_bytes> <win_out_path> <cluster_sectors> <mbr 0|1>
#
# 顺序保证原理:
#   FAT32 的目录项没有排序字段，设备端看到的顺序就是目录项在目录簇里的物理先后。
#   因此"先按顺序建目录，再严格按 JSON 数组顺序 mcopy"是唯一可控手段。
#   同时 mkfs.vfat 之后分配器从低簇递增，文件数据在物理上也是连续且按序的。
# ============================================================================
set -euo pipefail

SPEC=${1:?需要 spec.json}
OUT=${2:?需要输出镜像路径}
SIZE=${3:?需要镜像字节数}
WIN_OUT=${4:?需要 Windows 侧输出路径}
SPC=${5:-8}
MBR=${6:-0}

CLUSTER_BYTES=$((SPC * 512))
# 尺寸必须是簇的整数倍，否则 BPB_TotSec32 与实际卡容量不符，部分设备会拒绝识别
SIZE=$(( SIZE / CLUSTER_BYTES * CLUSTER_BYTES ))

mkdir -p "$(dirname "$OUT")" "$(dirname "$WIN_OUT")"
rm -f "$OUT"
truncate -s "$SIZE" "$OUT"

# ---- 读取卷标与卷 ID（卷标做 ASCII 净化，FAT 卷标不支持中文/空格易出问题）----
read -r LABEL VOLID <<<"$(python3 - "$SPEC" <<'PY'
import json, re, sys
card = json.load(open(sys.argv[1], encoding="utf-8"))["card"]
label = re.sub(r"[^0-9A-Za-z]", "", str(card.get("label") or "NONAME")).upper()[:11] or "NONAME"
volid = str(card.get("volId") or "12345678")
if not re.fullmatch(r"[0-9A-Fa-f]{8}", volid):
    volid = "12345678"
print(label, volid)
PY
)"

echo "[build_img] size=$SIZE label=$LABEL volid=$VOLID cluster=${SPC}sector mbr=$MBR" >&2

# ---- 可选：写 MBR 分区表（部分工控/车机固件只认带分区表的卡）----
if [ "$MBR" = "1" ]; then
    printf 'label: dos\nunit: sectors\nstart=2048, size=+, type=c, bootable\n' | sfdisk -q "$OUT"
    OFF_ARGS=(--offset 2048)
    MTSUFFIX="@@$((2048 * 512))"   # mtools 的镜像偏移语法: <file>@@<byte offset>
else
    OFF_ARGS=()
    MTSUFFIX=""
fi

# ---- 格式化 ----
mkfs.vfat -F 32 -s "$SPC" -n "$LABEL" -i "$VOLID" \
    ${OFF_ARGS[@]+"${OFF_ARGS[@]}"} "$OUT" >/dev/null

# ---- 按 JSON 顺序建目录 + 拷文件 + 写清单 ----
python3 - "$SPEC" "${OUT}${MTSUFFIX}" <<'PY'
import hashlib
import json
import os
import subprocess
import sys
import tempfile

spec_path, img = sys.argv[1], sys.argv[2]
spec = json.load(open(spec_path, encoding="utf-8"))
files = spec.get("files") or []
if not files:
    sys.exit("spec.files 为空，拒绝构建空镜像")


def to_wsl(p: str) -> str:
    """把 Windows 路径转成 WSL 路径：D:/a/b -> /mnt/d/a/b。"""
    p = p.replace("\\", "/")
    if p.startswith("/"):
        return p
    if len(p) > 1 and p[1] == ":":
        return "/mnt/%s%s" % (p[0].lower(), p[2:])
    return os.path.abspath(p)


def parent_dirs(dst: str):
    """返回 /ROOT/A/B 需要的逐级目录 ['ROOT', 'ROOT/A', 'ROOT/A/B']。"""
    parts = dst.strip("/").split("/")[:-1]
    return ["/".join(parts[:i]) for i in range(1, len(parts) + 1)]


made = set()
for idx, item in enumerate(files, 1):
    dst = item["dst"]
    if not dst.startswith("/"):
        sys.exit("dst 必须以 / 开头: %r" % dst)
    for d in parent_dirs(dst):
        if d in made:
            continue
        subprocess.run(["mmd", "-i", img, "::/" + d], check=True)
        made.add(d)

    src = to_wsl(item["src"])
    if not os.path.isfile(src):
        sys.exit("源文件不存在: %s (%s)" % (item["src"], src))

    # -D o: 关闭文本模式转换（否则 \n 会被改成 \r\n，二进制静默损坏）
    # -m  -p: 保留 mtime / 属性
    subprocess.run(
        ["mcopy", "-i", img, "-D", "o", "-m", "-p", src, "::" + dst],
        check=True,
    )
    size = os.path.getsize(src)
    print("[build_img] %4d/%d  %-32s %8.2f MB" %
          (idx, len(files), dst, size / 1048576), file=sys.stderr)

# ---- 顺序清单：设备端可解析，避免只依赖目录项顺序 ----
# spec 里不写 manifest 字段 -> 默认 /ORDER.TXT；写 null -> 不生成
manifest = spec["manifest"] if "manifest" in spec else "/ORDER.TXT"
if manifest:
    lines = ["# SDCARD_ORDER_V1", "# idx\tsize\tsha256\tpath"]
    for idx, item in enumerate(files, 1):
        src = to_wsl(item["src"])
        digest = item.get("sha256")
        if not digest:
            h = hashlib.sha256()
            with open(src, "rb") as f:
                for chunk in iter(lambda: f.read(1 << 20), b""):
                    h.update(chunk)
            digest = h.hexdigest()
        lines.append("%d\t%d\t%s\t%s" % (idx, os.path.getsize(src), digest, item["dst"]))
    with tempfile.NamedTemporaryFile("w", encoding="ascii", suffix=".txt",
                                     delete=False, newline="\r\n") as tmp:
        tmp.write("\n".join(lines) + "\n")
        tmp_path = tmp.name
    for d in parent_dirs(manifest):
        if d not in made:
            subprocess.run(["mmd", "-i", img, "::/" + d], check=True)
            made.add(d)
    subprocess.run(["mcopy", "-i", img, "-D", "o", tmp_path, "::" + manifest], check=True)
    os.unlink(tmp_path)
    print("[build_img] manifest -> %s" % manifest, file=sys.stderr)
PY

# ---- 结构自检（superfloppy 模式才能直接 fsck 整镜像）----
if [ "$MBR" = "0" ] && command -v fsck.vfat >/dev/null 2>&1; then
    if ! fsck.vfat -n "$OUT" >/dev/null 2>&1; then
        echo "[build_img] 警告: fsck.vfat 报告结构异常，请检查后再烧录" >&2
    fi
fi

# ---- 落到 Windows 共享目录（稀疏拷贝：避免 32GB 镜像在 drvfs 上写满整盘）----
sync
cp --sparse=always -f "$OUT" "$WIN_OUT"
sync
echo "[build_img] done -> $WIN_OUT" >&2
