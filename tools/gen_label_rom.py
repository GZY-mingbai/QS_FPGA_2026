#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
gen_label_rom.py —— 生成 osd_menu.v 里标签/标题 ROM 的初始化数据

【为什么需要脚本】
标签表是「4 个 EG_PHY_BRAM32K 并联成 64 位宽」：80 条记录 × 4 个 block ×
5 个 INIT 字 = 20 个 256 位常量。手写必错，而且错一位就是"某个标签变成乱码"，
上板才看得出来。

【编码约定（与 osd_menu.v 的 T1 段一致）】
  每条记录 = 64 位 = 4 个 16 位槽：
      slot3 = bits[63:48]  slot2 = [47:32]  slot1 = [31:16]  slot0 = [15:0]
  中文：4 个 GB2312 码，1 槽 1 字，第 i 个字放 slot(3-i)
  英文：8 个 ASCII 码，2 字 1 槽，第 i 个字放 slot(3-i//2) 的高/低字节
        （i 偶数 → 高字节，i 奇数 → 低字节）
  地址：中文 0..39，英文 40..79（英文 = 中文同序号 + 40）

  INIT 位序：INIT_w 的第 k 项在 bits[16k +: 16]，最低 16 位 = 最低地址
  （2026-10-28 上板实测确认，见 osd_menu.v T1 段注释）

【自检】--check 会把 osd_menu.v 里现有的 txt_rom 数组重新编码成 INIT，
        与文件里现有的 INIT 常量逐位比对。相等才说明本脚本的编码是对的。

用法：
    python tools/gen_label_rom.py --check     # 只校验编码器
    python tools/gen_label_rom.py --write     # 重新生成并写回 osd_menu.v
"""

import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
MENU = os.path.join(ROOT, "src", "user_source", "hdl_source", "APP", "OSD", "osd_menu.v")

BEGIN = "// ==== BEGIN AUTO-GEN: LABEL_ROM (tools/gen_label_rom.py) ===="
END   = "// ==== END AUTO-GEN: LABEL_ROM ===="

# ==============================================================
#  标签表（key = page*7 + item，与 osd_menu.v 的 lab_base 一致）
#    0..6   显示页    7..13  相册页    14..20 文件页（空）
#    21..27 系统页    28..34 关于页    35..39 标题
# ==============================================================
CN = {
    0: "亮度",     1: "对比度",   2: "色温",   3: "饱和度", 4: "风格",
    5: "恢复默认", 6: "自动亮度",
    7: "上一张",   8: "下一张",   9: "切换效果", 10: "自动切换",
    11: "信息条",  12: "音随图切", 13: "",
    21: "语言",    22: "音量",    23: "静音",   24: "循环播放", 25: "频谱条",
    26: "", 27: "",
    28: "型号",    29: "芯片",    30: "编译日期", 31: "作者",
    32: "许可",    33: "文档",    34: "退出",
    35: "显示",    36: "相册",    37: "文件",   38: "系统",  39: "关于",
}

EN = {
    0: "Bright",   1: "Contrast", 2: "Temp",    3: "Saturate", 4: "Style",
    5: "Default",  6: "AutoBrt",
    7: "Prev",     8: "Next",     9: "Effect",  10: "AutoPlay",
    11: "Info",    12: "ImgAudio",13: "",
    21: "Language", 22: "Volume", 23: "Mute",   24: "Loop",   25: "Spectrum",
    26: "", 27: "",
    28: "Model",   29: "Chip",    30: "Build",  31: "Author",
    32: "License", 33: "Docs",    34: "Exit",
    35: "Display", 36: "Album",   37: "File",   38: "System", 39: "About",
}

# ==============================================================
#  关于页（page 4）：整行文本，不走 label 表
#    行格式：CN 一行 = 16 槽（每个 16 位槽 1 个字，GB 码或 ASCII 码）
#            EN 一行 = 32 字符（2 字符/槽，高字节在前）
#    地址：  CN 80..95  = {3'b101, 行 0..3, 字 0..3}
#            EN 96..111 = {3'b110, 行 0..3, 字 0..3}
#    ⇒ "标签 + 值"按屏幕从左到右的顺序写成一整行，值不占任何逻辑。
# ==============================================================
ABOUT_CN = [
    "名称 智能商品展示终端",     # 11 槽
    "芯片 EG4S20BG256",          # 15 槽
    "平台 HX4S20",               #  9 槽
    "输出 640x480 60fps",        # 16 槽（正好用满）
]
ABOUT_EN = [
    "Name  Smart Display Terminal",   # 28 字符
    "Chip  EG4S20BG256",              # 18 字符
    "Board HX4S20",                   # 13 字符
    "Out   640x480 60fps",            # 19 字符
]

CN_MAX = 4          # 中文最多 4 字（4 个 16 位槽）
EN_MAX = 8          # 英文最多 8 字（2 字/槽）
N_ENTRY = 112       # 40 中文 + 40 英文 + 16 关于页中文 + 16 关于页英文
N_INIT  = 7         # 112 / 16


def word_cn(s):
    """中文标签 → 64 位字。"""
    if len(s) > CN_MAX:
        raise ValueError("中文标签超过 %d 字：%r" % (CN_MAX, s))
    w = 0
    for i, ch in enumerate(s):
        enc = ch.encode("gb2312")
        if len(enc) != 2:
            raise ValueError("非 GB2312 双字节字符：%r" % ch)
        code = (enc[0] << 8) | enc[1]
        w |= code << (16 * (3 - i))
    return w


def word_en(s):
    """英文标签 → 64 位字（2 个 ASCII 占 1 个 16 位槽，高字节在前）。"""
    if len(s) > EN_MAX:
        raise ValueError("英文标签超过 %d 字：%r" % (EN_MAX, s))
    w = 0
    for i, ch in enumerate(s):
        c = ord(ch)
        if not (0x20 <= c <= 0x7E):
            raise ValueError("非可打印 ASCII：%r" % ch)
        slot = 3 - (i >> 1)
        half = 8 if (i & 1) == 0 else 0        # 偶数 → 高字节
        w |= c << (16 * slot + half)
    return w


def code16(ch):
    """一个界面字符 → 16 位码：ASCII 走 0x00XX，汉字走 GB2312。"""
    if ord(ch) < 0x80:
        return ord(ch)
    enc = ch.encode("gb2312")
    if len(enc) != 2:
        raise ValueError("非 GB2312 双字节字符：%r" % ch)
    return (enc[0] << 8) | enc[1]


def about_cn_words(s):
    """关于页一行（中英混排）→ 4 个 64 位字。

    16 槽 = 4 字/字，槽 3 存该字的第 1 个字符（与 word_cn 一致）。
    """
    if len(s) > 16:
        raise ValueError("关于页中文行超过 16 字：%r" % s)
    s = s.ljust(16)
    words = []
    for w in range(4):
        v = 0
        for k in range(4):
            v |= code16(s[w * 4 + k]) << (16 * (3 - k))
        words.append(v)
    return words


def about_en_words(s):
    """关于页一行（纯 ASCII）→ 4 个 64 位字：8 字符/字，2 字符/槽，高字节在前。"""
    if len(s) > 32:
        raise ValueError("关于页英文行超过 32 字：%r" % s)
    s = s.ljust(32)
    words = []
    for w in range(4):
        v = 0
        for k in range(8):
            c = ord(s[w * 8 + k])
            if not (0x20 <= c <= 0x7E):
                raise ValueError("非可打印 ASCII：%r" % c)
            slot = 3 - (k >> 1)
            half = 8 if (k & 1) == 0 else 0        # 偶数 → 高字节
            v |= c << (16 * slot + half)
        words.append(v)
    return words


def build_entries():
    """40 中文 + 40 英文 + 关于页 4 行 × 2 语言 = 112 个 64 位字。"""
    out = [0] * N_ENTRY
    for k in range(40):
        out[k] = word_cn(CN.get(k, ""))
        out[40 + k] = word_en(EN.get(k, ""))
    for r, s in enumerate(ABOUT_CN):
        out[80 + 4 * r: 84 + 4 * r] = about_cn_words(s)
    for r, s in enumerate(ABOUT_EN):
        out[96 + 4 * r: 100 + 4 * r] = about_en_words(s)
    return out


def gen_init(words, nword=None):
    """80 个 64 位字 → 4 个 block × nword 个 INIT 字（256 位，16 项/字）。"""
    if nword is None:
        nword = N_INIT
    inits = []
    for blk in range(4):
        per_blk = []
        for w in range(nword):
            v = 0
            for k in range(16):
                idx = 16 * w + k
                if idx >= len(words):
                    break
                v |= ((words[idx] >> (16 * blk)) & 0xFFFF) << (16 * k)
            per_blk.append(v)
        inits.append(per_blk)
    return inits


def fmt_init(inits):
    """渲染成 4 个 EG_PHY_BRAM32K 实例的文本。"""
    lines = []
    lines.append("`else")
    lines.append("// ---- 综合：4 个 32Kb 块并联成 64 位宽 ----")
    lines.append("//   ★ 本段由 tools/gen_label_rom.py 自动生成，不要手改。")
    lines.append("//   条目 0..39 = 中文标签/标题，40..79 = 英文（同序号 + 40）；")
    lines.append("//   80..95 = 关于页中文行，96..111 = 关于页英文行（每行 4 个字）。")
    lines.append("wire [63:0] rom_q64;")
    for blk in range(4):
        lines.append("EG_PHY_BRAM32K #(")
        lines.append('    .MODE("DP16K"), .DATA_WIDTH_A("16"), .DATA_WIDTH_B("16"),')
        lines.append('    .REGMODE_A("NOREG"), .REGMODE_B("NOREG"),')
        lines.append('    .WRITEMODE_A("NORMAL"), .WRITEMODE_B("NORMAL"),')
        for w in range(N_INIT):
            comma = "," if w < N_INIT - 1 else ""
            lines.append("    .INIT_%02d(256'h%064X)%s" % (w, inits[blk][w], comma))
        lines.append(") u_txt_rom%d (" % blk)
        lines.append("    .clka(clk), .csa(1'b1), .wea(1'b0), .addra(11'd0), .dia(16'd0),")
        lines.append("    .bytea(1'b0), .bytewea(1'b0), .rsta(1'b0), .ocea(1'b1),")
        lines.append("    .clkb(clk), .csb(1'b1), .web(1'b0), .addrb({4'd0, tsrc_a}),")
        lines.append("    .dib(16'd0), .byteb(1'b0), .byteweb(1'b0), .rstb(1'b0), .oceb(1'b1),")
        lines.append("    .dob(rom_q64[%d:%d])" % (16 * blk + 15, 16 * blk))
        lines.append(");")
    lines.append("`endif")
    return "\n".join(lines)


def fmt_sim(words):
    """渲染仿真分支：行为数组 128 项（0..111 有内容，112..127 恒 0）。

    ★ 为什么是 128 而不是 112：tsrc_a 是 7 位，越界/空行会被指到 80（见
      osd_menu 的 tsrc_a 注释），加上英文偏移后最大 111。数组必须覆盖到 127，
      否则仿真里会读到越界的 X（综合侧 INIT_07 以后不写就是 0，天然安全）。
    """
    lines = []
    lines.append("`ifdef IVERILOG_SIM")
    lines.append("// ---- 仿真行为模型（iverilog 没有 TD 原语）----")
    lines.append("//   时序与 BRAM 一致：地址组合、clk 上升沿采样、1 拍后数据到 rom_q64。")
    lines.append("reg [63:0] rom_q64;")
    lines.append("reg [63:0] txt_rom [0:127];")
    lines.append("integer ti;")
    lines.append("initial begin")
    lines.append("    for (ti = 0; ti < 128; ti = ti + 1) txt_rom[ti] = 64'd0;")
    for k, w in enumerate(words):
        lines.append("    txt_rom[%2d] = 64'h%016X;" % (k, w))
    lines.append("end")
    lines.append("always @(posedge clk) rom_q64 <= txt_rom[tsrc_a];")
    return "\n".join(lines)


# ==============================================================
#  校验：把现有的 txt_rom 重新编码成 INIT，与现有 INIT 常量逐位比对
# ==============================================================
def do_check():
    text = open(MENU, encoding="utf-8").read()

    sim = {}
    for m in re.finditer(r"txt_rom\[\s*(\d+)\]\s*=\s*64'h([0-9A-Fa-f_]+)", text):
        sim[int(m.group(1))] = int(m.group(2).replace("_", ""), 16)

    # 只取 `else 之后（综合分支）的 INIT 常量，顺序 = block0..3 × word0..2
    tail = text.split("`else", 1)[1]
    consts = [int(m.group(2), 16)
              for m in re.finditer(r"\.INIT_(\d\d)\(256'h([0-9A-Fa-f]+)\)", tail)]

    print("解析到 txt_rom 条目 %d 个，INIT 常量 %d 个" % (len(sim), len(consts)))
    if not sim:
        print("!! 没解析到 txt_rom，检查文件格式")
        return 1
    if len(consts) % 4 != 0:
        print("!! INIT 常量数 %d 不是 4 的倍数" % len(consts))
        return 1
    nw = len(consts) // 4                      # 每个 block 的 INIT 字数
    words = [sim.get(k, 0) for k in range(nw * 16)]
    inits = gen_init(words, nw)
    bad = 0
    for blk in range(4):
        for w in range(nw):
            got = consts[blk * nw + w]
            exp = inits[blk][w]
            if got != exp:
                bad += 1
                print("MISMATCH block%d INIT_%02d" % (blk, w))
                print("  file = %064X" % got)
                print("  gen  = %064X" % exp)
                print("  xor  = %064X" % (got ^ exp))
    if bad == 0:
        print("CHECK PASS：编码器与文件里现有的 INIT 逐位一致（%d 项 × 4 block）"
              % (nw * 16))
        return 0
    print("CHECK FAIL：%d 处不一致" % bad)
    return 1


def do_write():
    text = open(MENU, encoding="utf-8").read()
    if BEGIN not in text or END not in text:
        print("!! 找不到自动生成标记，先把 osd_menu.v 的 ROM 段用标记包起来")
        return 1
    words = build_entries()
    body = "\n".join([BEGIN, fmt_sim(words), fmt_init(gen_init(words)), END])
    head, rest = text.split(BEGIN, 1)
    _, tail = rest.split(END, 1)
    open(MENU, "w", encoding="utf-8", newline="\n").write(head + body + tail)
    print("已写回 %s" % MENU)
    print("  条目数 %d（中文 0..39 / 英文 40..79 / 关于页 80..111）" % N_ENTRY)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--write", action="store_true")
    a = ap.parse_args()
    if a.check:
        return do_check()
    if a.write:
        return do_write()
    ap.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
