"""
===============================================================================
文件名   : wav_to_mif.py
功能     : 【音频格式转换工具】把电脑上的 .wav 音频文件，转换成 FPGA 能读的
           .mif 文件（Memory Initialization File，存储器初始化文件）。

为什么需要这个工具？
    FPGA 里的 ROM/RAM 上电时是空的（或者说是随机值），
    我们想让 ROM 里装上一段音乐，就得把音频数据"写死"进一个文本文件，
    综合工具会把这个文件的内容烧进 ROM 的初始值里。
    .mif 就是这个"文本格式的存储器内容"。

.mif 文件长什么样？（本脚本生成的格式）
    WIDTH=48;              ← 每个存储单元 48bit（左声道 24bit + 右声道 24bit）
    DEPTH=12345;           ← 一共有多少个单元
    ADDRESS_RADIX=UNS;     ← 地址用无符号十进制
    DATA_RADIX=HEX;        ← 数据用十六进制
    CONTENT BEGIN
    0 : 00ABCDEF123456;    ← 地址 0 上的数据
    1 : 00ABCDEF123456;
    ...
    END;

    ★ 注意 WIDTH=48 的由来：本工程要用一个 ROM 同时存左右两个声道，
      每个声道 24bit，24 + 24 = 48bit，正好凑成一个存储单元。★

输入文件的要求（脚本会检查，不符合会直接报错）
    · 采样率必须是 48000 Hz（因为工程里音频是按 48kHz 设计的）
    · 声道数必须是 1（单声道）或 2（立体声）
    · 位深必须是 16bit 或 24bit PCM

怎么用？
    命令行执行：
        python wav_to_mif.py input.wav output.mif
    然后把生成的 .mif 文件挂到 TD 的 ROM IP 上重新生成即可。

本脚本相关的输出文件
    同目录下的 audio_1khz_24bit_48deep.mif、song_48k.mif 就是本脚本的产物。
    额外的 .dath 文件是 TD 生成 ROM IP 时自动产生的辅助文件，不用管。
===============================================================================
"""

import wave     # Python 标准库，专门用来读 .wav 文件
import sys      # 用来读命令行参数


def s24_to_hex(v):
    """把一个 24bit 有符号整数转成 6 位十六进制字符串（用于写进 .mif）。

    为什么要处理负数？因为 mif 里只能写无符号的十六进制数。
    做法是"补码表示"：负数加上 2^24 就变成了对应的无符号数。
    例如 -1 -> 0xFFFFFF，-8388608 -> 0x800000。
    """
    if v < 0:
        v = (1 << 24) + v      # 加上 2 的 24 次方（相当于取补码）
    return f"{v & 0xFFFFFF:06X}"  # 只保留低 24 位，格式化成 6 位大写十六进制


def read_s24le(data3):
    """把 3 个字节（小端序 little-endian）读成一个 24bit 有符号整数。

    WAV 文件里的 24bit 采样就是 3 个字节。
    小端序 = 低位字节排在前面。
    Python 没有内置的 24bit 整数类型，所以要手工判断符号位来还原负数。
    """
    v = int.from_bytes(data3, byteorder="little", signed=False)  # 先当无符号读
    if v & 0x800000:        # 最高位（第 23 位）是 1 -> 这是负数
        v -= 1 << 24        # 减去 2^24 还原成有符号负数
    return v


def wav_to_mif(wav_path, mif_path):
    """主函数：读 wav -> 转换 -> 写 mif"""

    # ---- 第一步：打开 wav 文件，读它的格式信息 ----
    with wave.open(wav_path, "rb") as wf:
        nch = wf.getnchannels()      # 声道数（1=单声道，2=立体声）
        sw  = wf.getsampwidth()      # 每个采样的字节数（2=16bit，3=24bit）
        sr  = wf.getframerate()      # 采样率（Hz）
        nfr = wf.getnframes()        # 总共有多少个采样帧

        # ---- 格式检查：不符合就直接报错，避免生成错误的 mif ----
        if sr != 48000:
            raise ValueError(f"采样率必须是 48000，当前是 {sr}")
        if nch not in (1, 2):
            raise ValueError(f"只支持 1 或 2 声道，当前是 {nch}")
        if sw not in (2, 3):
            raise ValueError(f"只支持 16bit 或 24bit PCM，当前字节宽度是 {sw}")

        raw = wf.readframes(nfr)     # 一次性把全部音频数据读成字节串

    # ---- 第二步：逐帧解析出左右声道的 24bit 采样 ----
    samples = []
    frame_size = nch * sw            # 一帧所占字节数（帧 = 同一时刻的所有声道）

    for i in range(nfr):
        base = i * frame_size        # 这一帧在字节串里的起始位置

        if sw == 2:
            # 16bit 情况：先读成 16bit，再左移 8 位扩展到 24bit
            if nch == 1:
                l16 = int.from_bytes(raw[base:base+2], "little", signed=True)
                r16 = l16                                   # 单声道：左右相同
            else:
                l16 = int.from_bytes(raw[base:base+2], "little", signed=True)
                r16 = int.from_bytes(raw[base+2:base+4], "little", signed=True)

            # 16bit 扩展到 24bit（左移 8 位，相当于乘以 256，音量不变）
            l24 = l16 << 8
            r24 = r16 << 8

        else:  # sw == 3，即 24bit
            if nch == 1:
                l24 = read_s24le(raw[base:base+3])
                r24 = l24                                   # 单声道：左右相同
            else:
                l24 = read_s24le(raw[base:base+3])
                r24 = read_s24le(raw[base+3:base+6])

        samples.append((l24, r24))    # 存成 (左, 右) 的元组

    depth = len(samples)              # 存储深度 = 采样帧数

    # ---- 第三步：写 mif 文件 ----
    with open(mif_path, "w", encoding="utf-8") as f:
        f.write(f"WIDTH=48;\n")       # 48bit = 左24 + 右24
        f.write(f"DEPTH={depth};\n\n")
        f.write("ADDRESS_RADIX=UNS;\n")   # 地址用无符号十进制
        f.write("DATA_RADIX=HEX;\n\n")    # 数据用十六进制
        f.write("CONTENT BEGIN\n")
        for addr, (l24, r24) in enumerate(samples):
            # 每个地址上写 48bit：先左声道 6 位十六进制，再右声道 6 位
            f.write(f"{addr} : {s24_to_hex(l24)}{s24_to_hex(r24)};\n")
        f.write("END;\n")


# 程序入口：从命令行拿参数并执行
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("用法: python wav_to_mif.py input.wav output.mif")
        sys.exit(1)     # 参数不对，退出并返回错误码

    wav_to_mif(sys.argv[1], sys.argv[2])
    print("转换完成")
