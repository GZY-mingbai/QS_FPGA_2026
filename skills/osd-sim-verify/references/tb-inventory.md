# 仿真台清单

工作目录一律是 `src/user_source/hdl_source/APP/OSD/`。
公共源文件（除 style 台外都要带）：

```
osd_top.v osd_menu.v osd_gfx.v osd_cellmem.v osd_widget.v
flash_arb.v osd_icon_rom.v
tools/bram32k_stub.v tools/mult18_stub.v
../osd_font_cache.v ../ps_gen_text.v ../gb_addr.v
../../spi_flash_read.v ../w25q64_model_24b.v
```

编译要加 **`-DIVERILOG_SIM`**（osd_menu / osd_font_cache / osd_icon_rom 靠它走行为
RAM 分支，不加会去 elab 真正的 `EG_PHY_BRAM32K`）。

两个 stub 补的是 iverilog 没有的原语：
- `tools/bram32k_stub.v` —— `EG_PHY_BRAM32K`
- `tools/mult18_stub.v` —— `EG_PHY_MULT18`（2026-10-29 起 osd_menu 的 tween
  乘法改成显式例化这个原语，为的是把 25×25 的推断乘法从 4 个 DSP 降到 1 个）

字体由 `w25q64_model_24b #(.INIT_HEX("tools/font.hex"))` 提供 —— 少了它
`osd_font_cache` 永远不 ready，文字全空。

> ⚠️ 2026-10-29 现状：只有 **`tb_osd_keys`** 是活的。`tb_osd_smoke` / `tb_osd_anim`
> 还在引用 S6 重构前的 `u_mem.cell_r` 和 `u_fc.str_mem`，elaborate 阶段直接报
> 134 个 "Unable to bind"，改布局前要先修它们（或者别用）。

---

## 1. tb_osd_style.v（纯组合，毫秒）

```bash
iverilog -g2012 -o s.vvp -s tb_osd_style tb_osd_style.v osd_widget.v && vvp s.vvp
```

只例化 `osd_widget`，直接给 `wtype/wstyle/wrad/lx/ly/w/h` 看 `on/o_level/o_rgb`。
覆盖：

- A：`style10`（分割线）的 level 必须等于 `style1`（面板）= 6；亮度差 ≥ 24
- B：`grad=0` 时四角/边必须是样式色（判"有没有圆角"要看**颜色**，不能看 `on` ——
  圆角外侧是故意 `on=1` 并继承下层色的）
- C：`grad=8` 时角上必须是"继承色"（确认切角判据没被修死）

**改配色、改圆角、改边框，先跑这个台。**

---

## 2. tb_osd_smoke.v（数秒）

```bash
iverilog -g2012 -o /tmp/sm.vvp -s tb_osd_smoke tb_osd_smoke.v <公共源> && vvp /tmp/sm.vvp
```

结构探针：在若干特征点上直接算合成像素并比对 `{颜色, alpha}`。

```
navbg   (50,240)  0e0e0e/192   左栏
pagetop (500,10)  0e0e0e/192   页面板顶端（曾因 gy0=12 变成 000000/128）
pagebg  (300,300) 0e0e0e/192
divider (97,240)  3a3a3a/192   分割线（曾是不透明的 3a3a3a/255）
cursor  (28,40)   ffffff/255   游标外框
```

改页面板/分割线/遮罩的几何或 alpha，先跑它。

---

## 3. tb_osd_keys.v（~20s，主力）

```bash
iverilog -g2012 -o k.vvp -s tb_osd_keys tb_osd_keys.v <公共源> && vvp k.vvp > out/keys_test.log
grep -aE "FAIL|ALL PASS" out/keys_test.log
```

**这是投入产出比最高的台**：只跑按键脉冲 + 内部状态，不出图，所以快。
单个 vvp 跑完要 **10~12 分钟**（不是 20s），务必后台跑。

> ⚠️ **按键方向只有一个约定**：`KEY4`(4'h8)=上、`KEY3`(4'h4)=下；
> ComboBox 里"上"= 选项 **−1**、"下"= 选项 **+1**。
> 历史上 H 段与 O 段曾各写一套（一正一反），翻转方向后一次冒 30 项假 FAIL，
> 详见 `pitfalls.md` 第 19 条。新增段落请在这行注释下照抄这个约定。
按 A..W 分段，每段一个主题：

| 段 | 主题 |
|---|---|
| A–I | 逐按键：进入/上下/进入编辑/加减/退出 |
| L | KEY1 退出规则（导航栏=关 OSD，页面内=只返回） |
| N | 信息条 CheckBox |
| R | 上下换行不能改任何一行的显示值；编辑时值动画仍要收敛 |
| S | 滚到底 → 回导航栏 → 再进页面（几何归位） |
| T | CheckBox 用 KEY2 直接反选 |
| U | 滚到底 + 控件在第 0 行时打开弹层（`pop_a` 夹紧） |
| V | 系统页 4 个 ComboBox 的值互不影响 |
| W | Button 外观（type/半径/宽度/文字码） |
| D | 页面板/分割线几何 |
| H | 弹层几何（三种锚定位置） |

辅助任务：`kpress(k)` 单拍按键、`lay()` 排一帧、`lays(n)` 排 n 帧、
`to_nav` / `nav_to(p)`、`show_cell(i)` 打印控件属性。

### 加一个新场景的套路

```verilog
$display("--- X. 场景名 ---");
to_nav; nav_to(0); kpress(4'h2);          // 进 page0
begin : x_dn integer i; for (i=0;i<6;i=i+1) kpress(4'h4); end   // 滚到底
lays(22);                                  // 等动画收敛
chk("X1 at item6", `IT, 6);
```

- 按键编码：`KEY1=4'h1`（开/返回）、`KEY2=4'h2`（进入/确认）、`KEY3=4'h4`（下）、`KEY4=4'h8`（上）
- 每次改状态后 `lays(22)` 让 6 条动画通道走完，否则读到中间值
- 断言前先 `$display` 出关键内部值，失败时才能定位

---

## 4. tb_osd_popup.v（~2min，局部快渲）

```bash
iverilog -g2012 -o p.vvp -s tb_osd_popup tb_osd_popup.v <公共源> && vvp p.vvp
```

只渲染弹层相关的几块区域（`shot_rect` 的 st=1 是 1:1，区域越小越快）。
输出：`p_popup.ppm` / `p_zoom.ppm` / `p_full.ppm` / `f_corner.ppm` / `f_seam.ppm` /
`p_popup_up.ppm`。

`shot_rect(name, x0, y0, nx, ny, st)` 的 `nx,ny` 是**输出尺寸**，`st` 是采样步长
（1 = 1:1，2 = 缩小一半）。**想要放大必须用 `tools/crop.py` 二次裁**。

---

## 5. tb_osd_render.v（5~8min，全量出图）

```bash
iverilog -g2012 -o r.vvp -s tb_osd_render tb_osd_render.v <公共源> && vvp r.vvp > out/render.log
grep -a -c "^shot" out/render.log      # 30+ 张
```

场景序列：NAV → PAGE0 → 逐项 → 弹层 → 勾选框 → 四页巡检（每页 60 帧等文字重绑）→
滚到底 → 第 0 行开弹层 → 按钮行 → 回导航栏再进页面。

每步都有 `dump_state("TAG")`，日志里能查到当步的
`focus/nav/page/item/edit/scroll/pop/popsel`，出图异常时先对状态。

**注意**：巡检等待帧数要够（`NL` 从 6 加到 8 后，24 帧会拍到"半新半旧"的标签，
用 60 帧）。这是一类典型的假故障 —— 看起来像"标签错位"，其实是没等够。

---

## 6. tb_osd_probe.v / tb_osd_anim.v（数十秒）

单点像素轨迹探针 / 6 条动画通道逐步打印。改 cursor 动画、滚动、值动画时用。

---

## 7. tb_osd_icons.v（~1s，图标专项）

```bash
iverilog -g2012 -s tb_osd_icons -o /tmp/ic.vvp tb_osd_icons.v \
    osd_top.v osd_menu.v osd_gfx.v osd_cellmem.v osd_widget.v \
    osd_icon_rom.v flash_arb.v ../osd_font_cache.v ../ps_gen_text.v \
    ../gb_addr.v ../../spi_flash_read.v ../w25q64_model_24b.v && vvp /tmp/ic.vvp
```

Flash 模型同时灌字库与图标：

```verilog
w25q64_model_24b #(
    .INIT_HEX("tools/font.hex"),
    .INIT_HEX_ICONS("tools/icons32.hex"), .ICON_ADDR(24'h03D000)
) u_flash (...);
```

`tools/icons32.hex` = `icons/icons32.bin` 前 2560 字节，每行 1 字节。
**59 项断言**，分五组：

| 组 | 内容 |
|---|---|
| A/B | ROM ready、5 个槽 type=12、**1024 像素逐点等于文件解码值**（同时守住地址/字节/半字节/调色板/拼装方向） |
| C | 形状统计（颜色级数/墨迹像素数/bbox 不贴边）——防"纯色色块" |
| D | `icon_ready=0` ⇒ 整块 A=0（不能退化成 5 个色块） |
| E | 信息条：**文字捕获**（文件名 8 字不重复、分辨率紧凑 `640x480`、倒计时右对齐到第 17/18 列）+ 面板真圆角 + 进度条无圆角 |

E 组的倒计时断言需要把 `auto_remain` 接成 **reg**（原来是常量 `6'd0`）：

```verilog
reg [5:0] auto_remain = 6'd0;
...
.auto_remain(auto_remain), .auto_bar(9'd0),
...
force dut.u_menu.page_r = 3'd1;  force dut.u_menu.val3 = 7'd1;  // auto_mode=1
auto_remain = 6'd3;  nframe(30);
chk("E0-14 countdown ones at col18", ic7[18], 16'h0033);
chk("E0-15 countdown tens blank",    ic7[17], 16'h0000);
```

---

## 8. tb_info_trans.v / tb_info_overlay.v（毫秒~秒，信息条 × 切换动画）

- `tb_info_trans`：假帧源 + 假装载器驱动 `viewer_ctrl`，断言
  `info_show` 在 S_ARM / S_TRANS / S_LOW **全程为 1**（改前 15013 拍为 0）。
  ⚠️ 2026-11-05 现状：**elaborate 就报错**（`viewer_ctrl` 的端口表已变，
  tb 还在连 `I_have`）—— 要跑先按当前端口表重接。
- `tb_info_overlay`：例化 `viewer_scene`，`force` 内部 `pix_b/in_box_b_p`，
  逐值检查 `pix_b_eff / s2_b / pix_a_eff / alpha_eff` 共 **18 项**（A~G 七组）：
  透明 OSD 像素放行 B、不透明文字盖 B、面板 + B 落屏时 `s2=pix_b`、
  面板在 B 框外时 `s2=pix_a`、fx2 淡入（`alpha_b<255`）时不换 B、
  以及"框内不透明文字虽然 `s2_b=1`，但 `a9=256` ⇒ 输出仍是文字本身"。

  > 面板**不再**预混 `(3O+B)>>2`（2026-11-05 删）。那条路把面板混了两次，
  > 动画期间面板背景会掉一半亮度。机理见 `pitfalls.md` 第 16 条。

---

## 9. tb_scale_center.v（~数秒，缩放中心漂移）

工作目录 = `APP/`。纹理只留一行（`IMG_H=1`），放两个关于源中心对称的
白标记（源像素 40/280），按 fx3 的宽度序列逐级量亮度中心。
改前 max 1.5px、改后 max 0.5px。**不要用单个标记**——源像素本身可能
偏离中心 0.5px，会把"正常缩放"误判成漂移。

```bash
iverilog -g2012 -s tb_scale_center -o /tmp/sc.vvp \
    tb_scale_center.v img_layer_ctrl.v ps_gen_image.v img_afifo.v && vvp /tmp/sc.vvp
```

---

## 10. tb_bmp_scale.v / tb_loader.v（装载期纹理，SD/）

工作目录 = `SD/`。这两个台守的是**纹理内容**：`bmp_scale` 把任意源图渲进
640×480 888 / 320×240 565 两块固定画布，`tb_loader` 再套一层真实文件头 +
扇区流。

```bash
iverilog -g2012 -s tb_bmp_scale -o /tmp/b.vvp \
    tb_bmp_scale.v bmp_scale.v bmp_read.v sd_bmp_loader.v && vvp /tmp/b.vvp
iverilog -g2012 -s tb_loader -o /tmp/l.vvp \
    tb_loader.v sd_bmp_loader.v bmp_scale.v bmp_read.v ../APP/img_afifo.v && vvp /tmp/l.vvp
```

| 台 | 项数 | 覆盖 |
|---|---|---|
| `tb_bmp_scale` | **32** | 320×240→888（A）、1920×1080 contain（B）、100×400 窄条（C）、stretch（D）、**640×480→565 的 2:1 平均（E，含 E3/E4 通道串位）** |
| `tb_loader` | 19 | 真实 BMP 头 + 扇区流；**S2 = 320×240→565（1:1 + 行翻转）**、S1 = 640×480→888 |

> ⚠️ **期望函数必须逐通道写**（`avg2pix`）。曾经抄 RTL 的"整字相加再右移"，
> 把"满屏彩色麻点"的 bug 写成了期望，30/30 假绿、上板花屏。
> 详见 `pitfalls.md` 第 21 条。再加一条不依赖实现的兜底断言：
> `avg(纯红, 黑)` 的 G/B 必须为 0。

> `SD/tb_565_only.v` 是早期无断言的手工探针（激励已和握手对不上，
> `done=0 writes=0`），565 的正规覆盖在上面两个台里。

---

## 缩放链（640×480 源）的两个新台 —— 2026-10-28

上板"切到 HD 后画面整体右移、右边缘被截"，第一嫌疑是缩放。这两个台把
"缩放器有没有位移"彻底钉死了，结论是**没有**。

### `SD/tb_bmp_read640.v`（4 项）

`tb_bmp_read` 的 `mem` 只有 256KB，装不下 640×480 的 24bit BMP（≈900KB），
**所以 640×480 这条路从来没被 bmp_read 测过**。本台把 SD 扇区源改成
"按绝对地址现算"（`byte_at(a)`），不占存储 ⇒ 任意尺寸都能测。

- 源图 = 坐标编码图 `{x>>2, y>>2, x[1:0], y[1:0], 4'h0}`，24 位像素自带 (x,y)
- 断言：307200 点**逐点精确** + 每行"最佳位移"扫描
- 结果：`307200/307200`、`右移 0 列`

### `SD/tb_scale_shift.v`（6 项）

驱动真实 `bmp_scale`，画布**全量**捕获，然后反查"画布这一格的内容来自源图哪一点"。

| 用例 | 结果 |
|---|---|
| 640×480 → 888，stretch=0 | 307200 字、0 洞、**右移 0 列 / 下移 0 行** |
| 640×480 → 888，stretch=1 | 同上 |
| 640×480 → 565，stretch=0 | 38400 字、0 洞 |

> ⚠️ **写期望函数时的坑**：不能写 `{x[9:2], y[8:2], ...}` —— `integer`
> 表达式在拼接里按 32 位自决定宽度，整体会错位（实测 x=4 得 `0x008000`
> 而不是 `0x010000`）。必须先在函数内落成定宽 `reg` 再拼。
> 这一条让我第一版误判成"bmp_read 偏了 16 列"，详见 `pitfalls.md` 第 22 条。

> 565 画布**不能用坐标编码图查位移**：RGB565 量化会吃掉 `x[4:2]`，
> 同一行所有字看起来一样。565 的横向相位由 888 那条（同一份 `cprev/run`
> 代码）间接守住，内容由 `tb_loader` S3 全量守。


### `APP/tb_hd_stream.v`（2026-10-28 新增）

**回答"HD 模式下送进帧缓存的像素流对不对"**。直接把 `viewer_scene` 放在
`a_hd=1 / a_en=1 / b_en=0 / osd_on=0 / 满屏` 的 S_HD 姿态下跑一帧，
纹理模型是**坐标编码**的：HD 槽第 k 个字 = `{8'hFF, k%640, k/640, 8'h5A}`。

```bash
cd src/user_source/hdl_source/APP
iverilog -g2012 -s tb_hd_stream -o /tmp/hs.vvp tb_hd_stream.v \
    viewer_scene.v app_scan_gen.v img_layer_ctrl.v img_layer_hd.v \
    ps_gen_image.v img_afifo.v ps_merge_alpha_pp.v ps_to_rgb24.v
vvp /tmp/hs.vvp
```

结果：**按 `PIPE=4` 补偿后 0/307196 不匹配**，第 639 列逐行正确。

> ⚠️ **判据必须按 PIPE 补偿**：本台是按"每个 `pipe_ce` 拍取一个"抓的，
> 而 `frame_writer` 的 `write_en = take` 延迟 `PIPE=4` 拍。不补偿会看到
> "整幅延迟 4 像素、307200/307200 全错"——那是台的相位，不是 RTL 的错。
> 台里已经把补偿后的结果作为判据，未补偿的视图标了"仅供参考"。

### 两个坏掉的台（2026-10-28 发现）

| 台 | 状态 | 说明 |
|---|---|---|
| `APP/tb_app_full.v` | **编译不过** | `App` 已经没有 `st_state` 端口（诊断被删），`tb` 还在连 ⇒ `port 'st_state' is not a port of u_app`。这个台早就死了，所以"App → frame_writer"这段一直没人守 |
| `APP/tb_fx_equiv.v` | **静默假绿** | 需要 `../../../../.backup/work/viewer_fx_ref.v` 才能编译。缺文件时编译失败，但 `/tmp` 里的**旧二进制还在**，`vvp` 照样跑出 `ALL PASS` ⇒ 读到的是上一版结果。跑完必须确认 `iverilog` 真的成功（`rm -f` 先删再编） |


### `SD/tb_frame_wr_addr.v`（2026-10-28 新增，6 项）

**只测 `frame_fifo_write` 的写使能 / 写地址序列**，不需要 FIFO 模型：
`rdusedw` 恒给 500（数据永远够），`wr_ready=1`，然后逐拍记录
`App_wr_en` / `App_wr_addr`，与"教科书地址序列"逐字比：

```
row = 479 - k/行宽,  col = k%行宽,  addr = row*行宽 + col
```

| 用例 | 期望字数 | 结果 |
|---|---|---|
| 888（一行 640 字 = 正好 5 个 128 字突发） | 307200 | 字数 ✓、地址逐字 ✓、每行第 639 字 ✓ |
| 565（一行 320 字 = 2.5 个突发） | 153600 | 字数 ✓、地址逐字 ✓、每行第 319 字 ✓ |

> 建这个台的动机：上板"**HD 才有一整列 1px 黑线，565 没有**"。
> 888 的行尾正好压在 burst 边界上（640 = 5×128），565 的行尾落在 burst 中间
> ——如果突发收尾丢一个字，就只有 888 会每行丢第 639 列。**实测否掉了这条**：
> 地址与使能序列一字不差，所以黑列不是写侧丢字。

```bash
cd src/user_source/hdl_source/SD
iverilog -g2012 -s tb_frame_wr_addr -o /tmp/fwa.vvp tb_frame_wr_addr.v frame_fifo_write.v
vvp /tmp/fwa.vvp
```

> ⚠️ 判据里必须带 V-flip：第 q 个字属于内存行 `479 - q/行宽`。
> 我第一版写成 `y*640+639` 忘了翻转，白报 480 个 FAIL。

---

## 出图目录约定

```
out/*.ppm   仿真原始输出（P6，640×480 或局部）
out/*.png   ppm2png 转出来的（读图的入口）
out/*.log   各台的 stdout
```

`tb_auto_popup` 出 4 张（`a_popup5` / `a_popup4` / `a_infobar` / `a_infobar1`），
后两张是信息条排版证据：`a_infobar` = 倒计时 `30`（两位）、
`a_infobar1` = `7`（一位）。两张的个位必须**落在同一列**，否则右对齐没生效。

`out/` 会被反复覆盖。要保留某一轮的结果，整目录改名（如 `out1/`）再跑。
注意：**同一个文件名会被后跑的台覆盖**（`tb_osd_popup` 的 `f_seam.ppm` 曾被
`tb_osd_render` 的同名文件覆盖过）—— 跨台复用的图请用不同前缀。

---

## 改标签 / 改标题：别手改 osd_menu.v

标签和标题存在 4 个 `EG_PHY_BRAM32K` 并联成的 64 位 ROM 里（中文 0..39、英文 40..79），
`osd_menu.v` 里那段被两个标记夹住：

```
// ==== BEGIN AUTO-GEN: LABEL_ROM (tools/gen_label_rom.py) ====
...
// ==== END AUTO-GEN: LABEL_ROM ====
```

**改法**：改 `tools/gen_label_rom.py` 顶部的 `CN` / `EN` 两张表，然后

```bash
python tools/gen_label_rom.py --write    # 重写标记区（INIT_00..04 + 仿真数组）
python tools/gen_label_rom.py --check    # 反解校验：把 INIT 拼回去和数组逐位比
```

`--check` 必须在 `--write` 之后跑一次并通过，否则说明编码器坏了。
**不要手改那 20 个 256 位常量** —— 错一位就是某个标签变乱码，只有上板才看得出来。

编码约定（脚本里有注释，这里只记结论）：
- 中文：1 个 16 位槽放 1 个 GB2312 码，第 c 个字在 slot(3-c)
- 英文：2 个 ASCII 挤 1 个槽，第 c 个字在 slot(3-c/2) 的高/低字节（偶数位→高字节）
- INIT 位序：`INIT_w` 的第 k 项在 `[16k +: 16]`，**最低 16 位 = 最低地址**
- 仿真数组必须是 `[0:127]`：`tsrc_a` 是 7 位，越界/空行指向 80，+40 后是 120

---

## 11. 2026-11-02 信息条三件套（文件名 / 分辨率 / 读秒）

这一轮把信息条的三个"数据来源"都换了，对应的回归台都在 `APP/` 下，**各自几秒**：

### `APP/tb_res_slot.v`（8 项，<1s）—— 3 槽分辨率缓存

```bash
cd src/user_source/hdl_source/APP
iverilog -g2012 -s tb_res_slot -o /tmp/rs.vvp tb_res_slot.v img_switch.v && vvp /tmp/rs.vvp
```

只例化 `img_switch`，用"图 k 的源分辨率 = (320+16k)×(240+8k)"造出每张图不同的
W/H，装载模型照真机把 `I_src_w/h` 跟着**正在装的那张**变。断言：

- 换图当拍（提交后的下一拍）分辨率就是**目标图**的；
- 换图后邻居预取把 `ld_src_w/h` 覆盖掉，分辨率**不许**跟着变；
- **后退翻页**（最近一次装载 ≠ 目标图）也必须对 —— 这就是"用一份寄存器"会错的场景。

> 台里必须建模两件事，否则会把"没按动"误报成"分辨率不对"：
> ① `I_tl_take` 要**组合**（真机里 tl_take 与 tl_req 同拍），寄存器一拍之后
> `O_tl_rel` 已经翻到下一张；② base 滑窗后 album_src 会拉低 `O_ready` 再拉高，
> 预取在这期间停摆（见台里的相册重扫模型）。

### `APP/tb_auto_sw.v`（7 项，~1s）—— 读秒口径

```bash
iverilog -g2012 -s tb_auto_sw -o /tmp/as.vvp tb_auto_sw.v viewer_ctrl.v img_switch.v viewer_fx.v
vvp /tmp/as.vvp
```

`AUTO_SEC_CYC=400`（1 "秒" = 400 拍）把 3s 档压到可仿真；断言：

- A1 静止高清下 3s 后真的换图（实测 1199 拍）；
- A3 换图那一拍倒计时**归零重来**（`sw_go` 清零）；
- **A2 过渡/装高清期间倒计时继续走**（`rem_mask` 里必须出现 3/2/1/0）——
  改造前它被钉在满值上，要等高清上屏才开始掉；
- A4 动画里就到期的档位，高清一上屏下一个整秒就点火（`auto_sec` 饱和 ⇒ 不会锁死）。

### `OSD/tb_infobar_shot.v`（出图，~30s）—— 信息条截图

```bash
cd src/user_source/hdl_source/APP/OSD
iverilog -g2012 -DIVERILOG_SIM -s tb_infobar_shot -o /tmp/ibs.vvp \
    tb_infobar_shot.v osd_top.v osd_menu.v osd_gfx.v osd_cellmem.v osd_widget.v \
    osd_icon_rom.v flash_arb.v tools/bram32k_stub.v tools/mult18_stub.v \
    ../osd_font_cache.v ../ps_gen_text.v ../gb_addr.v ../../spi_flash_read.v ../w25q64_model_24b.v
vvp /tmp/ibs.vvp && python tools/ppm2png.py out/a_infobar_name.ppm
```

输出 `out/a_infobar_name.ppm`（340×90）：喂 8.3 名 `IMG_0001.BMP`（主名 8 ⇒ 最长 12 字）
+ 640×480 + 倒计时 30，用来肉眼确认"文件名带点、不截断、不循环"。
采样方式抄 `tb_osd_icons` 的 `sample`；**所有 osd_top input 都显式给值**
（漏接 zoom_* 会出全黑图，见 `pitfalls.md` 第 27 条）。

**★ 2026-11-05 起这台串三个场景**（同一个 `initial` 里 `force dut.u_menu.val3` 切档）：

| 场景 | auto_mode | 断言 | 出图 |
|---|---|---|---|
| S1 | 1（3s） | 轨道 (100,58) 有像素；面板到 y=74；数字矩形有墨 | `a_infobar_name.ppm` |
| S2 | 0（关） | 轨道 alpha==0；y=60 空、y=40 仍有面板；数字矩形**零墨**；文件名/分辨率仍在 | `a_infobar_off.ppm` |
| S3 | 5（随音乐） | 同 S2 | `a_infobar_music.ppm` |

共 16 项断言，`ALL PASS`。S2/S3 的"文件名/分辨率必须还在"是防过度修改的护栏
（别为了藏进度条把整条信息条关掉）。S2/S3 出图 sha256 **逐位相同** ⇒
"随音乐=关闭"这条口径在像素级成立。

⚠ **墨迹判据不能用 `alpha > 0`**：信息条面板本身就盖在数字那两格上
（A=0xC0 / RGB=0x202020）⇒ 会数出整整 512 个"墨点"（本次第一版就踩了这个，
S2/S3 假 FAIL）。要按**颜色**判：`cap[23:0] != 24'h202020`。

> `tb_equiv_menu_geom` 的"逐位等价"在这条改动之后不再天然成立（冻结快照里
> mode 0 也画轨道）⇒ 台子里把 `val3` 两边一起 force 成 1，等价性才继续有意义。
> 但**这台本身已经 elaborate 不过**（`txt_col` 端口 2026-11-04 改名成 `txt_ci`），
> 与本次改动无关；`tb_menu_equiv` 同病（还在用 `cur_w/cur_h`）。

`tb_osd_icons` 的 E0 组也跟着改了：现在喂 `IMG_01` + `BMP`（blen=6，故意不是 8），
断言"逐字内容 + 行宽 = nm_use(10) + 点在第 6 格 + 颜色号 1 + 第 10 格不许循环"。


> `tb_auto_popup` 2026-11-02 修了两个陈旧点：`NSLOT` 20→8（与 App 一致）、
> 补接 5 根 zoom 端口。修之前它拍的 4 张 PPM **全是纯黑**（一直没人看）。
> `tb_chain_full` 仍然 elaborate 不过（`O_have` / `I_alb_have` / `img_idx` 等
> 2026-11-04 重构前的端口），本轮没动它。

