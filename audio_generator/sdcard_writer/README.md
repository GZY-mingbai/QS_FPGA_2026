# sdcard_writer —— 按 JSON 顺序格式化并写入 SD 卡（FAT32）

给 FPGA 音频发生器一类的固件准备 SD 卡：格式化 → 按 JSON 顺序写入 → 顺序可控、物理连续。

```
JSON ──> [WSL2] mkfs.vfat + mtools 造镜像 ──> card.fat
                                                │
                                                ▼
                              [Windows] 裸写 PhysicalDriveN ──> SD 卡
```

## 为什么这么分工

- **顺序只能靠造镜像来保证。** FAT32 的目录项没有排序字段，设备端看到的顺序就是目录项在目录簇里的物理先后。`mkfs.vfat` 之后按顺序 `mcopy`，分配器从低簇递增，文件数据和目录项都严格按 JSON 顺序落位。
- **写盘只能在 Windows 侧做。** WSL2 是虚拟机，默认看不到 `/dev/sdX`（要 `usbipd-win` 直通，插拔后得重新 attach）。Windows 用 `\\.\PhysicalDriveN` 裸写，和 `dd` 等价。
- 所以：**WSL 造镜像，Windows 烧卡**，两边各干最擅长的事，不装额外工具。

## 依赖

```bash
# WSL2（Debian/Ubuntu）
wsl -e sudo apt install -y dosfstools mtools
```

Windows 侧只需 Python 3.8+，无第三方包。烧卡需要**管理员权限**。

## 用法

```bash
# 0) 看清有哪些磁盘（确认目标盘号，别选错）
python sdcard_writer.py --list

# 1) 先只造镜像，不碰卡（推荐第一次这么跑）
python sdcard_writer.py spec.example.json --dry-run --size 512M

# 2) 正式烧卡（会二次确认，要求手输盘符）
python sdcard_writer.py spec.example.json

# 快 5~8 倍：镜像只按内容大小裁剪，不铺满整卡（推荐）
python sdcard_writer.py spec.example.json --fit

# 其他常用参数
python sdcard_writer.py spec.example.json --target F          # 指定盘符
python sdcard_writer.py spec.example.json --size 8G           # 自定义容量
python sdcard_writer.py spec.example.json --cluster 64        # 32KB 簇（大文件顺序读更快）
python sdcard_writer.py spec.example.json --mbr               # 写 MBR 分区表（0x0C）
python sdcard_writer.py spec.example.json --verify --keep-image
python sdcard_writer.py spec.example.json -y                  # 跳过确认（脚本化用）
```

## JSON 格式

```json
{
  "card": {
    "target": "E",
    "label": "AUDIO",
    "sizeBytes": 0,
    "clusterSectors": 8,
    "volId": "12345678",
    "mbr": false
  },
  "build": { "image": "./build/card.fat" },
  "manifest": "/ORDER.TXT",
  "files": [
    { "src": "../out/ACC_01.WAV", "dst": "/ACC_01.WAV" }
  ]
}
```

| 字段 | 说明 |
|---|---|
| `files[]` | **数组顺序即写入顺序**，也是目录项顺序。改顺序就改数组顺序。 |
| `src` | 源文件。相对路径以 **spec.json 所在目录**为基准（程序会绝对化后再交给 WSL）。 |
| `dst` | 卡内路径，必须以 `/` 开头。目录会自动逐级创建。**卡内路径统一大写**，避免设备端大小写不匹配。 |
| `card.target` | 盘符，可被 `--target` 覆盖。 |
| `card.sizeBytes` | 0 表示用卡的物理容量（推荐）。dry-run 时必须配合 `--size`。 |
| `card.clusterSectors` | 每簇**扇区数**。8 = 4KB 簇；64 = 32KB 簇。 |
| `card.volId` | 8 位十六进制卷序列号，固定它可以让镜像可复现。 |
| `card.mbr` | `false` = superfloppy（整卡一个 FAT32 卷，消费类设备 90% 可用）；`true` = 建 MBR 单分区（部分工控/车机固件要求）。 |
| `manifest` | 顺序清单文件路径，内容为 `idx / size / sha256 / path`，设备端可解析，避免只依赖目录项顺序。置 `null` 关闭。 |

## 实现要点

1. **顺序保证**：先按 JSON 顺序 `mmd` 建全部目录，再严格按数组顺序 `mcopy` 逐个写入。
2. **镜像尺寸簇对齐**：`size = size // (clusterSectors*512) * (clusterSectors*512)`。不对齐会让 `BPB_TotSec32` 与卡容量不符，部分设备直接不认卡。
3. **`mcopy -D o`**：关闭文本模式转换。不加这个开关，mtools 会把 `\n` 改成 `\r\n`，二进制文件静默损坏——最难排查的一类 bug。
4. **镜像先落 WSL 原生盘，再 `cp --sparse=always` 到 `/mnt/...`**：drvfs 上直接建稀疏大镜像慢 5~10 倍。
5. **卷锁定**：`FSCTL_LOCK_VOLUME` + `FSCTL_DISMOUNT_VOLUME`，句柄必须保持打开到写入结束（Lock 随句柄关闭失效）。不卸载的话系统缓存会回写覆盖，且引导区写入被拒。
6. **扇区对齐**：`FILE_FLAG_NO_BUFFERING` 要求缓冲区地址、长度、文件偏移全部是物理扇区整数倍，所以用 `VirtualAlloc` 分配（页对齐），而不是普通 `bytes`。
7. **跳过全零块**：从镜像 BPB 解析出「引导扇区 + 双 FAT」的结束位置，该区域**永远整块写入**；之后的纯零块直接跳过。64GB 卡 + 1GB 内容，烧录时间从约 40 分钟降到 1~2 分钟。安全性：被跳过的簇在 FAT 里标记为空闲，卡上残留的旧数据不会被任何文件引用。
8. **写后刷新**：`FlushFileBuffers` + `IOCTL_DISK_UPDATE_PROPERTIES`，否则系统仍显示旧分区信息。拔插一次读卡器最稳。

## 速度：为什么不烧满 8GB

两个机制叠加，实际 IO 只有 1GB 左右：

1. **`--fit` 裁剪镜像**。默认镜像尺寸取卡的物理容量（8GB），全程要构建、拷贝、读一遍 8GB。加 `--fit` 后镜像只按 `内容 × 1.05 + 32MB` 造，8GB 卡上的 785MB 内容 → 856MB 镜像，构建 19s、全程 28s。代价是卷容量变成 856MB，卡剩余空间不可用（对专用播放卡无影响）。
2. **跳全零块**。即使铺满整卡，`burn_image` 也只写"引导扇区 + 双 FAT"（必写）+ 非零数据块，其余直接跳过。

实测（134 个文件 / 785MB / 8GB 卡）：

| 模式 | 镜像大小 | 构建 | 总耗时 |
|---|---|---|---|
| 默认（铺满卡） | 8.0 GB | ~40s | ~3 分钟 |
| `--fit` | 856 MB | 19s | 28s |

## 故障排查

| 现象 | 原因 / 处理 |
|---|---|
| `锁定卷 E: 失败` | 资源管理器窗口/预览/杀软/索引服务占用。关掉该卡的窗口，或等几秒重试。 |
| `打开 \\.\PhysicalDriveN 失败` | 没用管理员权限。 |
| 卡插上但不识别 | 先试 `--mbr`；仍不行则查容量是否小于 32MB（FAT32 要求 ≥ 65525 个簇，太小 `mkfs.vfat` 会退化成 FAT16）。 |
| 设备只认出部分文件 | 设备按 **8.3 短名**排序或只支持短名。给 `dst` 换成 8.3 形式（`A0001.WAV`），或检查是否用了超过 11 字符的名字。 |
| 播放顺序不对 | 设备可能按文件名排序而非目录项顺序。此时顺序由命名决定，看 `/ORDER.TXT` 确认写入顺序是否正确。 |
| 镜像 build 后为 0 字节 | `cp` 前 `sync` 失败或 `/mnt/c` 空间不足。 |
| `未找到 FAT32 BPB` | 镜像损坏或 `--mbr` 分区类型不是 0x0B/0x0C/0x0E。 |

## 文件说明

| 文件 | 作用 |
|---|---|
| `sdcard_writer.py` | 主控：解析 JSON → 调 WSL 造镜像 → 锁定卷 → 裸写 → 校验 |
| `build_img.sh` | WSL 侧：`truncate` → `mkfs.vfat`（可选 MBR）→ 按序 `mmd`/`mcopy` → 写清单 → 稀疏拷回 |
| `rawdisk.py` | Win32 裸盘原语：几何查询、卷锁定/卸载、对齐写、跳零块、刷新属性 |
| `spec.example.json` | 清单模板（对应本项目 `out/` 里的 44 个 WAV） |

## 安全须知

程序有三道闸：`IsSystem` 检查（拒绝对系统盘操作）、二次确认需手输盘符、`--dry-run` 可全程不碰卡。但裸写物理盘本身不可逆，**烧录前请确认盘符对应的确实是 SD 卡**（`--list` 里看 `FriendlyName` 和 `SizeGB`）。
