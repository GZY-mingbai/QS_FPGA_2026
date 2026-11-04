#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
wav_fat_offset.py —— 算出 SD 卡上 WAV 文件在**裸扇区**里的位置，并回写顶层参数

==============================================================================
【它解决什么问题】
==============================================================================
FPGA 侧的音频读取器（audio/sd_audio_reader.v）**不解析 FAT**，它只会
"从第 N 个扇区开始，连续读 M 个扇区，每 4 字节拼一个 {L16,R16} 样本"。
所以必须先在 PC 上把三件事算出来，写进顶层 parameter：

    WAV_BASE_SECTOR   data 数据块所在的物理扇区号（LBA，从卡的 0 字节算起）
    WAV_SKIP_BYTES    data 起点在该扇区内的字节偏移（0..511）
    WAV_TOTAL_WORDS   一共要播多少个 32bit 样本字（= data 字节数 / 4）

本脚本还会逐项校验格式（PCM / 48k / 16bit / 立体声）和"文件是否连续"，
不合格直接报错退出 —— 交给出错后上板听噪音要省事得多。

==============================================================================
【为什么 FAT 解析不是本脚本写的】
==============================================================================
完全是同一件事：定位文件在卡上的**物理偏移**。为了不重复实现（也就不会
出现两套不一致的解析结果），本脚本直接 import 同目录的 sd_fat_offset.py，
复用它的 RawDev / Fs / 簇链 / 分区识别。那边已经处理过 MBR / GPT /
超级软盘 / 盘符 vs 物理盘 / 簇链自检这些坑。

==============================================================================
【怎么用】
==============================================================================
    # 先看卡上有哪些 wav（自动选唯一的可移动盘）
    python tools/wav_fat_offset.py

    # 指定盘 / 镜像
    python tools/wav_fat_offset.py --dev \\.\E:
    python tools/wav_fat_offset.py --image card.img

    # 算完直接写回顶层，并把 AUDIO_SRC 切成 3
    python tools/wav_fat_offset.py --pick MUSIC.WAV --apply ^
        src/user_source/hdl_source/top_tf_hdmi_audio.v

    # 只看不写（默认行为）
    python tools/wav_fat_offset.py --pick MUSIC.WAV

==============================================================================
【对 WAV 的硬性要求】（不满足直接报错，不会"凑合"）
==============================================================================
    · RIFF/WAVE 容器，PCM（fmt tag = 1），未压缩
    · 48000 Hz（HDMI IP 里写死 AUDIO_SAMPLE_RATE="48K"）
    · 16 bit 有符号、小端
    · 2 声道（L/R 交织，4 字节一帧）
    · 文件在卡上**物理连续**（不碎片）—— 读取器是线性搬的

    转码命令（任意格式 → 合规）：
        ffmpeg -i input.mp3 -ar 48000 -ac 2 -sample_fmt s16 -c:a pcm_s16le music.wav
"""

import argparse
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sd_fat_offset as fat          # 复用 FAT / 分区 / 裸读的全部实现

SECTOR = 512
DEFAULT_TOP = os.path.join('src', 'user_source', 'hdl_source', 'top_tf_hdmi_audio.v')


#==============================================================================
#  WAV 头解析
#==============================================================================
def parse_wav(head):
    """解析 RIFF 头，返回 (info dict, 错误说明)

    head 只需要覆盖 fmt 和 data 两个 chunk 头（读到 data 头即可，不用读音频体）。
    """
    if len(head) < 12 or head[0:4] != b'RIFF' or head[8:12] != b'WAVE':
        return None, '不是 RIFF/WAVE（文件头前 12 字节不对）'

    info = {'fmt': None, 'data_off': None, 'data_size': None}
    p = 12
    while p + 8 <= len(head):
        cid = head[p:p + 4]
        csz = fat.u32(head, p + 4)
        body = p + 8

        if cid == b'fmt ':
            if body + 16 > len(head):
                return None, 'fmt chunk 被截断'
            info['fmt'] = {
                'tag':          fat.u16(head, body + 0),
                'channels':     fat.u16(head, body + 2),
                'sample_rate':  fat.u32(head, body + 4),
                'byte_rate':    fat.u32(head, body + 8),
                'block_align':  fat.u16(head, body + 12),
                'bits':         fat.u16(head, body + 14),
            }
        elif cid == b'data':
            info['data_off'] = body
            info['data_size'] = csz
            break

        # chunk 按偶数字节对齐
        p = body + csz + (csz & 1)

        # 防御：坏文件里 csz 可能是天文数字，别死循环
        if p > len(head):
            break

    if info['fmt'] is None:
        return None, '没找到 fmt chunk'
    if info['data_off'] is None:
        return None, '没找到 data chunk（可能文件还没写完？）'
    return info, None


def check_wav(info):
    """检查格式是否符合 HDL 侧假设，返回错误列表（空 = 合格）"""
    f = info['fmt']
    bad = []
    if f['tag'] != 1:
        bad.append('编码是 %d（只支持 1 = PCM 未压缩；ADPCM/MP3/AAC 都不行）' % f['tag'])
    if f['channels'] != 2:
        bad.append('声道数 %d（只支持 2；单声道请用 ffmpeg -ac 2 复制）' % f['channels'])
    if f['sample_rate'] != 48000:
        bad.append('采样率 %d Hz（必须是 48000；44.1k 会整体变调）' % f['sample_rate'])
    if f['bits'] != 16:
        bad.append('位深 %d bit（必须是 16）' % f['bits'])
    if f['block_align'] != 4:
        bad.append('block_align = %d（必须是 4 = 2ch × 16bit）' % f['block_align'])
    return bad


#==============================================================================
#  候选文件
#==============================================================================
def find_wavs(fs):
    out = []
    for ent in fs.walk():
        if ent.is_dir:
            continue
        name = ent.path.lower()
        if name.endswith('.wav'):
            out.append(ent)
    return out


def analyze_one(fs, ent, verbose=True):
    """算一个 wav 的三个参数，返回 dict 或 None"""
    fs.fill_runs(ent)                       # 填 ent.runs / ent.contiguous

    # 先读文件头。最多读 1MB —— 有些 wav 前面挂了很大的 LIST/INFO 块
    head = fs.read_file(ent, min(ent.size, 1 << 20))
    info, err = parse_wav(head)
    if info is None:
        print('  !! %-24s %s' % (os.path.basename(ent.path), err))
        return None

    bad = check_wav(info)
    if bad:
        print('  !! %-24s 格式不合格：' % os.path.basename(ent.path))
        for b in bad:
            print('       - %s' % b)
        return None

    f = info['fmt']
    data_off = info['data_off']
    data_size = info['data_size']
    words = data_size // 4

    if data_off + data_size > ent.size:
        print('  !! %-24s data chunk 声明 %d 字节，超出文件大小 %d —— 头坏了'
              % (os.path.basename(ent.path), data_size, ent.size))
        return None

    # data 起点在卡上的绝对字节偏移
    phy_file = fs.card_off(fs.clus_vol_off(ent.first))
    phy_data = phy_file + data_off
    base_sector = phy_data // SECTOR
    skip_bytes = phy_data % SECTOR

    if skip_bytes >= SECTOR:
        print('  !! %-24s 跳过字节 %d ≥ 512，工具和 HDL 的约定被打破了'
              % (os.path.basename(ent.path), skip_bytes))
        return None

    # 逐字节自检：FAT 路径读出来的与按物理偏移裸读的必须一致
    #   直接复用 sd_fat_offset 里那份（盘符模式/整盘模式的地址换算它都处理过了）
    sc = fat.do_selfcheck(fs, ent, phy_file)
    selfcheck = sc.startswith('OK')

    rec = {
        'name':        os.path.basename(ent.path),
        'path':        ent.path,
        'size':        ent.size,
        'data_off':    data_off,
        'data_size':   data_size,
        'base_sector': base_sector,
        'skip_bytes':  skip_bytes,
        'words':       words,
        'seconds':     words / 48000.0,
        'contiguous':  ent.contiguous,
        'selfcheck':   selfcheck,
        'selfcheck_msg': sc,
        'fmt':         f,
    }

    if verbose:
        print('  %-24s %6.1f s  %8.2f MB  扇区 %d + %d 字节  字 %d'
              % (rec['name'], rec['seconds'], data_size / 1e6,
                 base_sector, skip_bytes, words))
        if not ent.contiguous:
            print('     !! 文件在卡上是**碎片**的（簇链不连续）—— HDL 线性读会串音。')
            print('        处理：把文件复制出卡 → 删掉 → 再复制回去（FAT 会尽量连续分配），')
            print('              或先格式化卡再放文件。')
        if selfcheck is False:
            print('     !! 自检失败：%s' % sc)
        else:
            print('     自检 %s' % sc)

    return rec


#==============================================================================
#  回写顶层
#==============================================================================
def apply_to_verilog(path, rec, verbose=True):
    """只改数值，不动注释与换行风格（CRLF 保留）"""
    with open(path, encoding='utf-8', newline='') as f:
        src = f.read()

    n = 0

    def sub_num(pattern, value, text):
        new, k = re.subn(pattern, lambda m: m.group(1) + str(value), text)
        return new, k

    src, k = sub_num(r'(parameter\s+integer\s+WAV_BASE_SECTOR\s*=\s*)\d+',
                     rec['base_sector'], src)
    n += k
    src, k = sub_num(r'(parameter\s+integer\s+WAV_SKIP_BYTES\s*=\s*)\d+',
                     rec['skip_bytes'], src)
    n += k
    src, k = sub_num(r'(parameter\s+integer\s+WAV_TOTAL_WORDS\s*=\s*)\d+',
                     rec['words'], src)
    n += k
    # 顺手把音源切到 3（SD WAV）
    src, k = sub_num(r'(parameter\s+integer\s+AUDIO_SRC\s*=\s*)\d+', 3, src)
    n += k

    if n == 0:
        print('  !! 一处也没改（顶层参数名和预期不一样？）—— 放弃写入')
        return False

    with open(path, 'w', encoding='utf-8', newline='') as f:
        f.write(src)

    if verbose:
        print('\n  已写回 %d 处参数：%s' % (n, path))
        print('    WAV_BASE_SECTOR = %d' % rec['base_sector'])
        print('    WAV_SKIP_BYTES  = %d' % rec['skip_bytes'])
        print('    WAV_TOTAL_WORDS = %d   (%.1f 秒)' % (rec['words'], rec['seconds']))
        print('    AUDIO_SRC       = 3   （SD 卡 WAV）')
    return True


#==============================================================================
#  打开设备（与 sd_fat_offset 完全一致的做法）
#==============================================================================
def open_fs(args):
    if not args.dev and not args.image:
        rows = [r for r in fat.win_list_drives() if r['removable']]
        if len(rows) == 1:
            args.dev = rows[0]['dev']
            print('  自动选中可移动盘 %s（%s %s）'
                  % (args.dev, rows[0]['label'], rows[0]['fs']))
        elif not rows:
            raise SystemExit('没找到可移动盘。请先插卡，或用 --dev / --image 指定。')
        else:
            raise SystemExit('检测到多个可移动盘，请用 --dev 指定其中一个：%s'
                             % ', '.join(r['dev'] for r in rows))

    if args.image:
        dev = fat.RawDev(args.image, align=args.sector)
        if fat.look_like_fat_boot(dev.read_sector(0, 1)):
            vol_base = card_base = 0
            desc = '镜像：卷在第 0 字节'
        else:
            off, desc = fat.find_partition(dev)
            vol_base = card_base = off
    else:
        dev = fat.RawDev(args.dev, align=args.sector)
        if args.dev.upper().startswith('\\\\.\\PHYSICALDRIVE') or args.dev.startswith('/dev/'):
            off, desc = fat.find_partition(dev)
            vol_base = card_base = off
        else:
            vol_base = 0
            card_base = 0
            desc = '卷（盘符）模式'
            if os.name == 'nt':
                letter = args.dev.rstrip(':').lstrip('\\\\.')
                info = fat.win_volume_map(letter)
                if info and info[2] is not None:
                    card_base = info[2]
                    desc = '卷模式：物理盘 %d 第 %d 分区，起始 %d' % (info[0], info[1], info[2])
                else:
                    raise SystemExit(
                        '拿不到这个卷在物理盘上的起始字节（算出来的偏移会少一段）。\n'
                        r'   改用 --dev \\.\PhysicalDriveN，或先导出镜像用 --image。')

    return fat.Fs(dev, vol_base, card_base, desc)


#==============================================================================
def main():
    ap = argparse.ArgumentParser(
        description='算 SD 卡上 WAV 的裸扇区位置，并回写顶层参数',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dev', help=r'设备：Windows \\.\E: 或 \\.\PhysicalDrive2；Linux /dev/sdb')
    ap.add_argument('--image', help='卡镜像文件（离线分析）')
    ap.add_argument('--pick', help='选哪个 wav（名字的一部分，不区分大小写）')
    ap.add_argument('--apply', nargs='?', const=DEFAULT_TOP,
                    help='把结果写回顶层 .v（不给路径则用 %s）' % DEFAULT_TOP)
    ap.add_argument('--sector', type=int, default=512, help='逻辑扇区大小（默认 512）')
    args = ap.parse_args()

    fs = open_fs(args)
    print('  %s' % fs.part_desc)
    print('  分区类型 %s，簇 %d 字节\n' % (fs.kind, fs.bytes_per_clus))

    ents = find_wavs(fs)
    if not ents:
        raise SystemExit('  !! 卡上没找到 .wav 文件')

    if args.pick:
        key = args.pick.lower()
        ents = [e for e in ents if key in e.path.lower()]
        if not ents:
            raise SystemExit('  !! 没有名字包含 "%s" 的 wav' % args.pick)

    print('  找到 %d 个 wav：' % len(ents))
    print('  %-24s %8s %10s %s' % ('文件', '时长', '大小', '位置'))
    recs = []
    for e in ents:
        r = analyze_one(fs, e, verbose=True)
        if r:
            recs.append(r)

    if not recs:
        raise SystemExit('\n  !! 没有一个可用（见上面的原因）')

    if not args.apply:
        best = recs[0]
        print('\n  ---- 可以粘贴到顶层 ----')
        print('    parameter integer WAV_BASE_SECTOR = %d,' % best['base_sector'])
        print('    parameter integer WAV_SKIP_BYTES  = %d,' % best['skip_bytes'])
        print('    parameter integer WAV_TOTAL_WORDS = %d'  % best['words'])
        print('    parameter integer AUDIO_SRC       = 3')
        print('\n  或者加 --apply 让本脚本直接写回。')
        return 0

    # 多个候选时取第一个；建议用 --pick 明确指定
    best = recs[0]
    if len(recs) > 1:
        print('\n  !! 有 %d 个可用 wav，默认取第一个（%s）；'
              '要换请用 --pick 指定。' % (len(recs), best['name']))

    if best['contiguous'] is False:
        raise SystemExit('\n  !! 文件碎片，拒绝回写（上板一定串音）。先把文件整理成连续再跑。')
    if best['selfcheck'] is False:
        raise SystemExit('\n  !! 自检失败，拒绝回写。')

    ok = apply_to_verilog(args.apply, best)
    if ok:
        print('\n  下一步：重新综合下载。SW2 拨到 ON（播放）即可听到。')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
