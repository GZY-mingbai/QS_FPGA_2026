# 出图 / 读图 工具链

全部是纯标准库 Python，不需要 PIL。用托管解释器：

```bash
PY="C:/Users/mingbai/.workbuddy/binaries/python/versions/3.13.12/python.exe"
```

---

## 1. ppm2png.py —— PPM(P6) → PNG

```bash
$PY tools/ppm2png.py out/v_popup.ppm          # 单张
$PY tools/ppm2png.py out                      # 目录下所有 .ppm
```

输出 `<同名>.png`，同时打印 `xxx.ppm -> xxx.png (WxH)`。

---

## 2. crop.py —— 裁剪 + 整数倍放大

```bash
$PY tools/crop.py in.ppm out.png x y w h [倍率=6]
```

从 PPM 裁 `(x,y,w,h)` 再按整数倍放大成 PNG。看细节（圆角、1px 边框、勾选框间隙）
一律用它放大 4~8 倍，不要靠 `shot_rect` 的 `st`（那个只会缩小）。

**已知坑（踩过三次）**：裁剪矩形越界时，`write_png` 仍按 `w*h*k` 声明 IHDR，
但像素切片变短 ⇒ PNG 头尾不一致 ⇒ 读图工具报 `unsupported image`，整个请求 400。
现行版本已在 `crop.py` 里夹紧矩形并断言长度；**用旧版裁的图要重新生成**。

---

## 3. slice.py —— 采样一条线/一列的颜色

```bash
$PY tools/slice.py out/f_seam.ppm --h y=150 --from 94 --to 106 --step 1
$PY tools/slice.py out/f_seam.ppm --v x=200 --from 56 --to 176 --step 6
```

`--h` 横采、`--v` 竖采；`--ox/--oy` 是"图是从哪个屏幕坐标裁的"的偏移（便于换算）。

**这是替代肉眼的第二件工具**：判断"分割线是不是 3px 同色"、"面板顶端有没有亮带"，
直接打印数值比看图可靠。

---

## 4. 读图前必须校验 PNG

坏图会让整轮对话失败。20 行脚本：

```python
import glob, struct, zlib
bad = []
for p in sorted(glob.glob('out/*.png')):
    d = open(p, 'rb').read()
    try:
        assert d[:8] == b'\x89PNG\r\n\x1a\n'
        off = 8; idat = b''
        while off < len(d):
            ln = struct.unpack('>I', d[off:off+4])[0]; typ = d[off+4:off+8]
            body = d[off+8:off+8+ln]
            assert zlib.crc32(typ+body) & 0xffffffff == \
                   struct.unpack('>I', d[off+8+ln:off+12+ln])[0]
            if typ == b'IHDR': w, h, bd, ct = struct.unpack('>IIBB', body[:10])
            elif typ == b'IDAT': idat += body
            elif typ == b'IEND': break
            off += 12+ln
        ch = {2:3, 6:4, 0:1}[ct]
        assert len(zlib.decompress(idat)) == (w*ch+1)*h   # ← 头尾不一致会在这里炸
    except Exception as e:
        bad.append((p, str(e)))
print("PNG 校验:", "全部通过" if not bad else bad)
```

---

## 5. 三个"用数值代替眼睛"的量测套路

### 5.1 墨迹 bbox（字有没有被裁）

```python
W,H,px = read_ppm('out/f_popup_row0.ppm')
def get(x,y):
    i = (y*W + x)*3; return (px[i], px[i+1], px[i+2])
ink = [y for y in range(row_top, row_top+30)
       if max(get(x,y)[0] for x in range(332,356)) > 150]
print("墨迹", ink[0], ink[-1], "高", len(ink))     # 16px 字格应该出 16 行
```

用来抓"字被相邻行擦掉 4 行"这类问题（见 `pitfalls.md` 第 6 条）。

### 5.2 差值恒定性（判断两根细线会不会"忽明忽暗"）

不需要知道底图是什么，只看**同一列上两条线的差值是否恒定**：

```python
d = [get(97,y)[0] - get(104,y)[0] for y in range(60,260)]
print("min=%d max=%d 唯一值=%s" % (min(d), max(d), sorted(set(d))))
# 期望：唯一值只有 1~2 个（量化误差）；若跨度大 ⇒ 有一根是不透明的
```

本工程实测 `[31,32]`（恒定）——修复前分割线是不透明 `3A3A3A`，
差值会随底图明暗反号，上板看就是"虚线/杂像素"。

### 5.3 面板边界竖扫

```python
prev = None
for y in range(y0, y1):
    c = get(600, y)
    if c != prev: print("  y=%d %s" % (y, c)); prev = c
```

一次就能看出面板上下边、内边距是否对称（曾用这招发现 12/6 不对称）。

---

## 6. 读图时的分寸

- **先数值，后看图**：图像用来确认"观感"（间距、密度、颜色搭配），
  几何/颜色的一致性一律用数值断言。
- 一张图里内容超过 3 处判断点时，先 `crop.py` 放大局部，不要盯着整屏猜。
- 出图前先 `grep -a -E "^\[TAG"` 看 `dump_state`，确认拍的是**你以为的那个状态**
  （很多"看起来错位"其实是拍在了动画中途）。
