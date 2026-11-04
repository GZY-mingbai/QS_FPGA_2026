# QS_FPGA

基于安路 EG4S20（HX4S20C 开发板）的 **TF 卡 BMP 相册 + HDMI 1.4b 显示 + 音频** 工程。
显示为 640×480p60，TF 卡使用 FAT32，工具链为 Anlogic Tang Dynasty + Icarus Verilog。

## 功能概览

- **TF 卡相册**：扫描 FAT32 根目录下的 BMP，按 8.3 短名读取；支持文件页、缩略图窗口、四宫格和高清槽。
- **图片查看**：适合屏幕的 24 位 BMP；支持等比缩放、补黑边、缩放/平移/翻页和过渡动画。
- **OSD**：菜单、文件页、系统页、信息栏、弹层、滑块、复选框、动态字幕。
- **音频**：WAV 播放、PCM 提示音、I2S / HDMI 音频输出、FFT 频谱条。
- **显示链路**：SDRAM 双缓冲帧缓存，50 MHz 场景域 / 100 MHz SDRAM 域，HDMI 1.4b + LVDS。
- **输入**：板载 4 键、五向键、环境光自动亮度（ALS）。
- **字库/图标**：字库、OSD 图标、菜单标签存放在外部 SPI Flash（W25Q64）。

## 目录结构

| 路径 | 内容 |
|---|---|
| `src/user_source/hdl_source/` | RTL 源码与测试台 |
| `src/user_source/hdl_source/APP/` | 应用层：场景合成、OSD、相册/文件页、四宫格、缩放 |
| `src/user_source/hdl_source/SD/` | TF 卡 FAT32、BMP 读写/缩放、SDRAM 与仲裁 |
| `src/user_source/hdl_source/audio/` | WAV 播放、音量/混音、频谱 FFT |
| `src/user_source/hdl_source/peripherals/` | 按键消抖/路由、ALS ADC 与自动亮度、USB HID |
| `src/user_source/hdl_source/ref/` | 等价测试用的冻结参考 RTL（`*_ref.v`） |
| `src/user_source/constraints_source/` | 引脚约束 `.adc`、时序 `.sdc`、EDID `.cwc` |
| `src/user_source/ip_source/` | PLL 等 IP 配置与生成源码 |
| `src/td_project/` | TD 工程入口 `HDMI1.4b_Transmitter_v1.0.al`、`.ipc`、综合脚本 |
| `tools/` | 镜像/字库/标签生成脚本、批跑测试台脚本 |
| `tools/convert/` | 图片转换、SD 卡检查与同步脚本（Pillow） |
| `Font/` | GB2312 字库源图与生成脚本 |
| `icons/` | OSD 图标源图（SVG/PNG）与打包结果 |
| `sim_data/` | 真卡导出的 `sd.hex` / `expect.txt` fixture |
| `audio_generator/` | TTS/BGM 音频生成与写卡脚本（产物不入库） |
| `PCB/` | 硬件工程 `QS_FPGA.eprj2` |
| `FlashProgrammer/` | SPI Flash 烧录器（FPGA + 上位机） |
| `skills/` | OSD 仿真验证 skill |

以下目录/文件是**本地产物，不入库**，按需生成：

- TD 综合/布局布线：`src/td_project/*_Runs/`、`src/td_project/log/`
- 仿真：`sim_out/`、`*.vvp`、`*.vcd`、`*.ppm`、`*.raw`
- 数据生成：`tools/out/`、`sim_data/*.img`、`sim_data/*.bmp`
- 音频素材：`audio_generator/out/`、`audio_generator/sdcard_writer/staging*/`

## 打开与构建

1. 用 Tang Dynasty 打开 `src/td_project/HDMI1.4b_Transmitter_v1.0.al`。
2. 综合/布局布线输出会落在 `src/td_project/HDMI1.4b_Transmitter_v1.0_Runs/`（已忽略）。
3. 引脚与时序约束在 `src/user_source/constraints_source/`。

### 烧录 SPI Flash

烧录器分两部分：`FlashProgrammer/FPGA/`（FPGA 工程）和 `FlashProgrammer/host/flash_tool.py`（上位机）。
上位机按 `FlashProgrammer/host/partitions.json` 里的分区表把文件写入 W25Q64：

```bash
python FlashProgrammer/host/flash_tool.py list
python FlashProgrammer/host/flash_tool.py id --port COM3
python FlashProgrammer/host/flash_tool.py check -p FlashProgrammer/host/partitions.json
python FlashProgrammer/host/flash_tool.py program -p FlashProgrammer/host/partitions.json --port COM3 --verify
```

## 运行仿真

需要 `bash`、`iverilog`、`vvp`、`python`：

```bash
bash tools/run_all_tb.sh -q          # 跳过长时间测试台
bash tools/run_all_tb.sh -c          # 只编译
bash tools/run_all_tb.sh -j 4 osd    # 并行 4 路，只跑名字含 osd 的台子
bash tools/run_all_tb.sh -j 4 quad   # 只跑名字含 quad 的台子
```

脚本会自动发现 `tb_*.v`，日志写到 `sim_out/tb_logs/<tb名>.log`（已忽略）。
`-q` 会跳过 `tb_osd_keys`、`tb_osd_render`、`tb_app_full`、`tb_chain_img`、`tb_osd_popup`、`tb_switch`、`tb_trans` 等长台子。
缺少测试数据时会自动生成：

- `sim_data/fat32_10img.img`（及 `.map` / `_map.vh`）：用 `tools/mk_fat32_img.py`
- `src/user_source/hdl_source/SD/out/card_img.bin`：用 `tools/gen_grid_bmp.py` 现画测试图，再交给 `tools/gen_sd_image.py`
- `src/user_source/hdl_source/SD/out/ref/sdr_port_sched_ref.v`：从固定 git 版本导出

想用自定义图片做 `card_img.bin`，可设置 `CARD_SRC=/path/to/image.bmp` 再运行脚本。

## 重新生成数据资产

```bash
# 全链路仿真的 FAT32 卡镜像
python tools/mk_fat32_img.py --out sim_data/fat32_10img.img --check

# OSD 图标 + logo ROM（输出 icons/osd_bitmaps.bin 和 OSD/tools/osd_bitmaps.hex）
# 需要 Pillow：pip install Pillow
python tools/gen_osd_bitmaps.py

# GB2312 字库（输出 Font/font_gb2312.bin）
# 需要 Pillow：pip install Pillow
python Font/gen_font.py

# WAV -> MIF（48 kHz / 16 或 24 bit）
python src/user_source/hdl_source/python/wav_to_mif.py input.wav output.mif
```

`sim_data/sd.hex` 和 `sim_data/expect.txt` 是从真实 TF 卡导出的 fixture，不能由脚本重建，已保留。

## 图片转换与 SD 卡工具

`tools/convert/` 下是配套的图片/卡工具，详见 [`tools/convert/README.md`](tools/convert/README.md)：

| 脚本 | 用途 |
|---|---|
| `convert_images_to_bmp.py` | 批量转成 640×480 / 24 位 / 无压缩 BMP |
| `keepres_to_bmp.py` | 保持原分辨率，只做色彩模式归一化 |
| `sync_to_sd.py` | 清理旧扇区残留并按顺序同步图片到 SD 卡 |
| `check_sd.py` | 按 RTL 规则只读检查卡或磁盘镜像的根目录 |

`convert_images_to_bmp.py` / `keepres_to_bmp.py` 会在脚本旁创建 `input_images/` 和 `output_bmp/`（均已忽略）；
`sync_to_sd.py` 默认从 `output_bmp/` 取图，`check_sd.py` 可直接检查 `sim_data/fat32_10img.img`。

FPGA 端要求：**FAT32、512 字节扇区、根目录簇号 = 2、24 位无压缩 BMP、文件名大写 8.3**。
使用 `keepres_to_bmp.py` 时高度建议不超过 4095（`bmp_loader` 使用 12 位行号）。

## 仓库约定

- 只提交人写的源码、约束、工程文件、脚本，以及构建/仿真必需的小数据（`.mif`、字库/图标 `.bin`、`*.hex`、`sd.hex`）。
- 波形、截图、日志、镜像、综合数据库、bitstream、备份目录不要提交。
- `audio_generator/` 只保留生成脚本和配置，生成的 WAV/BMP 由 `.gitignore` 排除。
- 修改生成逻辑后，同步更新本文件的再生成命令。
