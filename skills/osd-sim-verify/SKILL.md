---
name: osd-sim-verify
description: 用全分辨率仿真出图 + 状态断言来验证 FPGA OSD（菜单/控件/文字/弹层/动画）的做法。当要改 OSD 布局、控件外观、文字排布、下拉弹层、游标、分割线，或上板发现显示异常（缺字、错位、杂像素、圆角、背景黑、值串味）但不确定是哪一行代码时使用。包含 7 个仿真台的分工、出图工具的用法、断言写法模板，以及 iverilog/TD 的踩坑清单。
agent_created: true
---

# OSD 仿真验证（出图 + 断言）

## 用途

把「只有上板才看得出的 UI 问题」变成**可回归的检查**：一批 640×480 全分辨率仿真台
（状态断言 / 像素出图 / 纯组合探针），配合几个几十行的 Python 小工具做像素级量测。

这套东西的价值不在"能出图"，而在两条纪律：

1. **先让断言 FAIL，再改代码** —— 没有失败过的断言不算验证。
2. **能变成断言的，就不要靠眼睛** —— 眼睛只用来最后确认观感。

## 何时使用

- 改 `osd_menu.v` / `osd_widget.v` / `osd_gfx.v` / `ps_gen_text.v` 的布局、控件、文字
- 上板现象：文字缺失/错行、控件错位、下拉弹层异常、分割线杂像素、多余圆角、背景黑、多个值联动
- 需要回答"这个改动会不会把别处弄坏"
- 排查"仿真跑不出来"的问题（先读代码找设计缺陷，再把它固化成断言）

## 工程位置

```
src/user_source/hdl_source/APP/OSD/          # OSD 源码 + 7 个仿真台
src/user_source/hdl_source/APP/OSD/tools/    # ppm2png / crop / slice / chk_multidrv / probe
src/user_source/hdl_source/APP/OSD/out/      # 出图目录（*.ppm + *.png + *.log）
src/user_source/hdl_source/APP/viewer_ctrl.v # 场景侧状态机（OSD 只允许在 S_HD 打开）
```

## 台子分工（按耗时选台）

| 台子 | 耗时 | 用途 |
|---|---|---|
| `tb_osd_style` | 毫秒 | 纯组合：样式表 level/颜色、圆角切角判据。**改配色/圆角先跑它** |
| `tb_osd_smoke` | 数秒 | 结构探针：几个特征点上像素颜色/alpha 对不对 |
| `tb_osd_keys` | ~20s | **主力**：逐按键状态断言 + 控件几何断言（几百项） |
| `tb_osd_popup` | ~2min | 局部快渲：只出下拉弹层那几张图 |
| `tb_osd_render` | 5~8min | 全量出图：巡检 20+ 张全分辨率图 |
| `tb_osd_probe` / `tb_osd_anim` | 数十秒 | 单点探针 / 动画通道逐步观察 |

**迭代节奏**：改代码 → `tb_osd_style` → `tb_osd_keys` → `tb_osd_popup` 看图 →
最后才跑 `tb_osd_render` 全量确认。不要每次都用最慢的台。

## 快速开始

```bash
cd src/user_source/hdl_source/APP/OSD

# 1) 状态断言（最常用）
iverilog -g2012 -o k.vvp -s tb_osd_keys tb_osd_keys.v osd_top.v osd_menu.v \
    osd_gfx.v osd_cellmem.v osd_widget.v ../osd_font_cache.v ../ps_gen_text.v \
    ../gb_addr.v ../../spi_flash_read.v ../w25q64_model_24b.v && vvp k.vvp \
    | grep -aE "FAIL|ALL PASS"

# 2) 纯组合样式/圆角
iverilog -g2012 -o s.vvp -s tb_osd_style tb_osd_style.v osd_widget.v && vvp s.vvp

# 3) 出图（PPM → PNG，读图前必须校验）
for f in v_popup f_popup t_page0; do
  "$PY" tools/ppm2png.py out/$f.ppm; done

# 4) 静态检查（TD 报多驱动时必跑）
"$PY" tools/chk_multidrv.py osd_menu.v osd_gfx.v osd_cellmem.v osd_widget.v osd_top.v
```

`$PY` = `C:/Users/mingbai/.workbuddy/binaries/python/versions/3.13.12/python.exe`

## 铁律

1. **先复现再修**：加一条会 FAIL 的断言，把 got/expect 记到回复里，再动 RTL。
2. **断言不写死绝对坐标**，写不变量（"选中项必在 combo_y-2"、"任意两条文字行字格不相交"）。
3. **断言名用 ASCII**：`chk("H row0 visible", ...)`。`%0s` 打中文会变乱码。
4. **读图前校验 PNG**（CRC + 解压长度）。坏图会让整轮对话 400。
5. **慢台只做最终确认**，迭代用快台或局部快渲。
6. **"仿真不出来"的问题，最后必须变成一条断言或一个静态检查**，否则下次还会犯。

## 参考文件

| 文件 | 内容 |
|---|---|
| `references/tb-inventory.md` | 7 个台的完整构建命令、覆盖点、输出文件、加场景的方法 |
| `references/image-tools.md` | 出图工具用法、PNG 校验脚本、墨迹 bbox、差值恒定性检验 |
| `references/assertion-patterns.md` | `chk` 任务、不变量模板、文字监视器、`cell_r` 位布局表 |
| `references/pitfalls.md` | iverilog / TD 的 14 条坑：现象→原因→改法 |

## 三条最有价值的经验

- **文字层字格是 16px 高**：两条文字行 y 相差 < 16 就会互相擦掉 4 行像素。
  弹层行距必须满足 `2*PITCH <= 菜单行距 - 16`（本工程 32 ≤ 80-16）。
- **`always @(*)` 的敏感表不含函数体内读到的信号**（iverilog）。取多路值一律用
  显式 `case` 或行内表达式，不要用 `function val_of(item)`。
- **窗口行号 ≠ 绝对 item 号**：`sel_k/icx` 是窗口行（0..4），`vidx` 是 item 号，
  只有 `scroll_row==0` 时相等。两者混用会在"滚动之后"才暴露。
