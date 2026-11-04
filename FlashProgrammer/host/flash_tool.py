#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flash_tool.py —— HX4S20C (EG4S20BG256) SPI Flash 烧录上位机

配套 FPGA 工程：FlashProgrammer/FPGA（W25Q64，8MB）
通信方式：串口（CH340，默认 921600 8N1）

依赖：pyserial   （pip install pyserial）
"""

import argparse
import json
import os
import struct
import sys
import time

# ----------------------------------------------------------------------------
# 依赖检查
# ----------------------------------------------------------------------------
try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    serial = None
    HAS_SERIAL = False


def require_serial():
    """只有真正连板子的子命令才需要 pyserial；check / --help 不需要。"""
    if not HAS_SERIAL:
        sys.stderr.write(
            "\n[错误] 缺少依赖 pyserial，无法连接串口。请先安装：\n"
            "    pip install pyserial\n"
            "（如果只想检查分区表，用 `check` 子命令，不需要 pyserial）\n\n"
        )
        sys.exit(1)


# ============================================================================
#  协议常量（必须与 FPGA 侧 cmd_parser.v 保持一致）
# ============================================================================
FRAME_H0 = 0xAA
FRAME_H1 = 0x55

CMD_READ_ID    = 0x01
CMD_READ       = 0x02
CMD_WRITE      = 0x03
CMD_CHIP_ERASE = 0x04
CMD_PING       = 0x05

ST_OK      = 0x00
ST_CRC_ERR = 0x01
ST_BAD_CMD = 0x02
ST_BAD_LEN = 0x03

STATUS_TEXT = {
    ST_OK:      "OK",
    ST_CRC_ERR: "CRC 校验失败",
    ST_BAD_CMD: "命令不支持",
    ST_BAD_LEN: "长度非法",
}

FLASH_SIZE   = 8 * 1024 * 1024     # W25Q64 = 8MB
SECTOR_SIZE  = 4096                # 擦除最小单位
PAGE_SIZE    = 256                 # 编程最小单位
MAX_FRAME_DATA = 4096              # FPGA 侧缓冲上限（cmd_parser 的 4KB）

DEFAULT_BAUD = 921600


# ============================================================================
#  CRC-16/CCITT-FALSE（初值 0xFFFF，多项式 0x1021，无反转）
# ============================================================================
def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ============================================================================
#  组帧
# ============================================================================
def pack_request(cmd: int, addr: int = 0, data: bytes = b"", length: int = None) -> bytes:
    """
    按协议组一帧请求：AA 55 | CMD | ADDR(4,LE) | LEN(4,LE) | DATA | CRC16(2,LE)

    length：帧里的 LEN 字段，默认 = len(data)。**这里是最容易踩的坑：**
      · WRITE → LEN 就是数据长度，用默认即可
      · READ  → 请求帧本身没有数据，LEN 表示"要读多少字节"，必须显式传！
                若用默认（len(data)=0），板子会读回 0 字节。
      · PING / READ_ID / CHIP_ERASE → LEN = 0，用默认
    """
    if length is None:
        length = len(data)
    body = struct.pack("<BII", cmd, addr & 0xFFFFFFFF, length) + data
    crc = crc16(body)
    return bytes([FRAME_H0, FRAME_H1]) + body + struct.pack("<H", crc)


# ============================================================================
#  串口封装
# ============================================================================
class FlashLink:
    """负责组帧、发送、收响应、重试。"""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD, timeout: float = 5.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None

    # ---------------------------------------------------------------
    def open(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        # 打开串口会拉低 DTR/RTS，部分 CH340 会因此复位，等一下再清空
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    # ---------------------------------------------------------------
    def _read_exact(self, n: int, timeout: float):
        """读满 n 字节，超时抛异常。"""
        old = self.ser.timeout
        self.ser.timeout = timeout
        try:
            buf = self.ser.read(n)
        finally:
            self.ser.timeout = old
        if len(buf) != n:
            raise TimeoutError(
                "串口超时：期望 %d 字节，只收到 %d 字节" % (n, len(buf))
            )
        return buf

    # ---------------------------------------------------------------
    def request(self, cmd: int, addr: int = 0, data: bytes = b"",
                length: int = None, resp_timeout: float = 5.0):
        """
        发送一帧请求，收一帧响应。
        返回 (status:int, payload:bytes)

        length：帧里的 LEN 字段，默认 = len(data)。
          · WRITE → 用默认
          · READ  → 必须显式传"要读多少字节"，否则板子读回 0 字节
        """
        frame = pack_request(cmd, addr, data, length)

        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()

        # --- 收响应：AA 55 | STATUS(1) | LEN(4,LE) | DATA(LEN) | CRC16(2,LE) ---
        # 先找帧头，容忍少量脏字节
        hdr = self._sync_header(resp_timeout)
        if hdr is None:
            raise TimeoutError("等待响应帧头 AA 55 超时")

        status = self._read_exact(1, resp_timeout)[0]
        length = struct.unpack("<I", self._read_exact(4, resp_timeout))[0]
        if length > FLASH_SIZE:
            raise ValueError("响应长度异常：%d" % length)

        payload = self._read_exact(length, resp_timeout) if length else b""
        crc_rx = struct.unpack("<H", self._read_exact(2, resp_timeout))[0]

        # --- 校验响应 CRC（覆盖 STATUS..DATA）---
        calc = crc16(bytes([status]) + struct.pack("<I", length) + payload)
        if calc != crc_rx:
            raise ValueError(
                "响应 CRC 不符：收到 0x%04X，计算 0x%04X" % (crc_rx, calc)
            )
        return status, payload

    # ---------------------------------------------------------------
    def _sync_header(self, timeout: float):
        """扫描流直到出现 AA 55。返回 True/None。"""
        deadline = time.time() + timeout
        prev = None
        while time.time() < deadline:
            self.ser.timeout = max(0.05, deadline - time.time())
            b = self.ser.read(1)
            if not b:
                break
            cur = b[0]
            if prev == FRAME_H0 and cur == FRAME_H1:
                return True
            prev = cur
        return None

    # ---------------------------------------------------------------
    def read_id(self):
        st, p = self.request(CMD_READ_ID, resp_timeout=3.0)
        if st != ST_OK:
            raise IOError("读 ID 失败：" + STATUS_TEXT.get(st, "未知状态 %d" % st))
        return p

    def ping(self):
        st, p = self.request(CMD_PING, resp_timeout=3.0)
        return st == ST_OK and p == b"\x55"

    def chip_erase(self):
        # 整片擦除典型 20~100 秒，超时给足
        st, _ = self.request(CMD_CHIP_ERASE, resp_timeout=180.0)
        if st != ST_OK:
            raise IOError("整片擦除失败：" + STATUS_TEXT.get(st, "状态 %d" % st))

    def read_flash(self, addr: int, length: int) -> bytes:
        out = bytearray()
        while length > 0:
            n = min(length, MAX_FRAME_DATA)
            st, p = self.request(CMD_READ, addr=addr, length=n, resp_timeout=10.0)
            if st != ST_OK:
                raise IOError("读 Flash 失败：" + STATUS_TEXT.get(st, "状态 %d" % st))
            if len(p) != n:
                raise IOError("读回长度不符：期望 %d，收到 %d" % (n, len(p)))
            out += p
            addr += n
            length -= n
        return bytes(out)

    def write_flash(self, addr: int, blob: bytes, progress=None):
        total = len(blob)
        done = 0
        while done < total:
            chunk = blob[done:done + MAX_FRAME_DATA]
            st, _ = self.request(CMD_WRITE, addr=addr + done, data=chunk, resp_timeout=15.0)
            if st != ST_OK:
                raise IOError("写 Flash 失败：" + STATUS_TEXT.get(st, "状态 %d" % st))
            done += len(chunk)
            if progress:
                progress(done, total)


# ============================================================================
#  分区表
# ============================================================================
def parse_int(v):
    if isinstance(v, int):
        return v
    return int(str(v), 0)      # 支持 "0x1000" / "4096"


class Partition:
    def __init__(self, d, base_dir):
        self.name = str(d.get("name", "")).strip()
        self.offset = parse_int(d.get("offset", 0))
        f = d.get("file")
        if not f:
            raise ValueError("分区 %s 缺少 file 字段" % self.name)
        self.path = f if os.path.isabs(f) else os.path.join(base_dir, f)
        self.size = parse_int(d["size"]) if "size" in d else None

    @property
    def data(self):
        with open(self.path, "rb") as fp:
            return fp.read()

    @property
    def length(self):
        return os.path.getsize(self.path)


def load_partitions(path):
    if not os.path.isfile(path):
        raise FileNotFoundError("分区表不存在：%s" % path)
    with open(path, "r", encoding="utf-8") as fp:
        cfg = json.load(fp)

    base = os.path.dirname(os.path.abspath(path))
    raw = cfg.get("partitions")
    if not raw:
        raise ValueError("分区表里没有 partitions 数组")

    parts = [Partition(d, base) for d in raw]
    for p in parts:
        if not p.name:
            raise ValueError("有分区缺少 name")
    return cfg, parts


def check_partitions(parts):
    """返回 (errors, warnings)。不连板子，纯本地检查。"""
    errors, warnings = [], []

    # 1) 重叠 / 越界
    ordered = sorted(parts, key=lambda p: p.offset)
    for i, p in enumerate(ordered):
        if not os.path.isfile(p.path):
            errors.append("分区 [%s] 文件不存在：%s" % (p.name, p.path))
            continue
        end = p.offset + p.length
        if end > FLASH_SIZE:
            errors.append(
                "分区 [%s] 越界：0x%06X + %d = 0x%06X > 0x%06X (8MB)"
                % (p.name, p.offset, p.length, end, FLASH_SIZE)
            )
        if p.size is not None and p.length > p.size:
            errors.append(
                "分区 [%s] 超出声明容量：文件 %d 字节 > size %d 字节"
                % (p.name, p.length, p.size)
            )
        if i + 1 < len(ordered):
            nxt = ordered[i + 1]
            if end > nxt.offset:
                errors.append(
                    "分区 [%s] 与 [%s] 重叠：0x%06X~0x%06X vs 起始 0x%06X"
                    % (p.name, nxt.name, p.offset, end, nxt.offset)
                )
        # 2) 4KB 对齐
        if p.offset % SECTOR_SIZE != 0:
            warnings.append(
                "分区 [%s] 起始 0x%06X 未按 4KB 对齐（扇区 0x%06X 内还有别的内容会被擦除）"
                % (p.name, p.offset, p.offset & ~(SECTOR_SIZE - 1))
            )
        if p.length % SECTOR_SIZE != 0:
            warnings.append(
                "分区 [%s] 长度 %d 不是 4KB 整数倍（尾部扇区会被整块擦除）"
                % (p.name, p.length)
            )
    return errors, warnings


# ============================================================================
#  子命令
# ============================================================================
def cmd_list(args):
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("未发现任何串口。请检查 USB 线是否插好、CH340 驱动是否安装。")
        return 0
    print("可用串口：")
    for p in ports:
        print("  %-8s  %s" % (p.device, p.description))
    return 0


def cmd_id(args):
    with FlashLink(args.port, args.baud) as link:
        rid = link.read_id()
        print("JEDEC ID: %s" % " ".join("%02X" % b for b in rid))
        if len(rid) >= 3:
            mfr, mem_type, cap = rid[0], rid[1], rid[2]
            print("  厂商 = 0x%02X %s" % (mfr, "(Winbond)" if mfr == 0xEF else ""))
            if mfr == 0xEF and mem_type == 0x40:
                print("  型号 ≈ W25Q%02X（容量 %d Mbit）" % (cap, 1 << cap))
        return 0


def cmd_check(args):
    _, parts = load_partitions(args.partitions)
    errors, warnings = check_partitions(parts)
    print("分区表：%s" % args.partitions)
    print("共 %d 个分区\n" % len(parts))
    for p in sorted(parts, key=lambda x: x.offset):
        exist = os.path.isfile(p.path)
        print("  %-10s offset=0x%06X  %s" % (
            p.name, p.offset,
            ("%d 字节" % p.length) if exist else "!! 文件缺失"
        ))
    print()
    for w in warnings:
        print("[警告] " + w)
    for e in errors:
        print("[错误] " + e)
    if not errors and not warnings:
        print("检查通过：无重叠、无越界、对齐正常。")
    elif not errors:
        print("\n无致命错误，可以烧录（注意上面的对齐警告）。")
    return 1 if errors else 0


def cmd_program(args):
    cfg, parts = load_partitions(args.partitions)

    if args.only:
        sel = [p for p in parts if p.name == args.only]
        if not sel:
            sys.stderr.write("找不到分区：%s\n" % args.only)
            return 2
        parts = sel

    errors, warnings = check_partitions(parts)
    for w in warnings:
        print("[警告] " + w)
    if errors:
        for e in errors:
            print("[错误] " + e)
        sys.stderr.write("\n分区表有致命错误，已中止。请先跑 `check`。\n")
        return 1

    port = args.port or cfg.get("port")
    baud = args.baud or cfg.get("baud", DEFAULT_BAUD)
    if not port:
        sys.stderr.write("未指定串口。用 --port 指定，或在分区表里写 \"port\"。\n")
        return 2

    total_bytes = sum(p.length for p in parts)

    print("串口 %s @ %d   待写入 %d 字节" % (port, baud, total_bytes))
    print("烧录流程：整片擦除 → 逐分区写入\n")

    t0 = time.time()
    with FlashLink(port, baud) as link:
        # 0) 验证链路
        try:
            rid = link.read_id()
            print("链路正常，JEDEC ID = %s" % " ".join("%02X" % b for b in rid))
        except Exception as e:
            sys.stderr.write("读 ID 失败，请检查接线/串口号/波特率。\n  原因：%s\n" % e)
            return 1

        # 1) 整片擦除（--skip-erase 可跳过）
        if args.skip_erase:
            print("\n[1/2] 跳过整片擦除（--skip-erase）")
            print("      ！Flash 只能把 1 写成 0：若目标区域已有旧数据，")
            print("        新数据会与旧数据按位相与，结果可能不正确。")
        else:
            print("\n[1/2] 整片擦除中……（W25Q64 典型 20~100 秒，请勿断电）")
            te = time.time()
            link.chip_erase()
            print("      擦除完成，用时 %.1f 秒" % (time.time() - te))

        # 2) 逐分区写入
        print("\n[2/2] 写入分区：")
        for p in sorted(parts, key=lambda x: x.offset):
            blob = p.data
            print("      [%s] 0x%06X  %d 字节" % (p.name, p.offset, len(blob)))
            ts = time.time()

            def prog(done, total, _name=p.name):
                pct = 100.0 * done / total
                sys.stdout.write("\r        %s: %5.1f%%  (%d/%d)" % (_name, pct, done, total))
                sys.stdout.flush()

            link.write_flash(p.offset, blob, progress=prog)
            sys.stdout.write("\r        %s: 100.0%%  用时 %.2f 秒\n" % (p.name, time.time() - ts))

            # 写后校验
            if args.verify:
                back = link.read_flash(p.offset, len(blob))
                if back != blob:
                    diff = next(i for i in range(len(blob)) if back[i] != blob[i])
                    sys.stderr.write(
                        "\n[校验失败] 分区 [%s] 第 %d 字节不符：写 0x%02X，读回 0x%02X\n"
                        % (p.name, diff, blob[diff], back[diff])
                    )
                    return 1
                print("        校验通过")

    print("\n全部完成，总用时 %.1f 秒。" % (time.time() - t0))
    return 0


def cmd_dump(args):
    cfg, parts = load_partitions(args.partitions)
    port = args.port or cfg.get("port")
    baud = args.baud or cfg.get("baud", DEFAULT_BAUD)
    if not port:
        sys.stderr.write("未指定串口。用 --port 指定，或在分区表里写 \"port\"。\n")
        return 2

    outdir = args.dir
    os.makedirs(outdir, exist_ok=True)

    with FlashLink(port, baud) as link:
        print("读回 %d 个分区 → %s" % (len(parts), outdir))
        for p in sorted(parts, key=lambda x: x.offset):
            n = p.size if p.size else p.length
            data = link.read_flash(p.offset, n)
            out = os.path.join(outdir, p.name + ".bin")
            with open(out, "wb") as fp:
                fp.write(data)
            print("  [%s] 0x%06X  %d 字节 → %s" % (p.name, p.offset, n, out))
    return 0


def cmd_erase(args):
    cfg, _ = load_partitions(args.partitions)
    port = args.port or cfg.get("port")
    baud = args.baud or cfg.get("baud", DEFAULT_BAUD)
    if not port:
        sys.stderr.write("未指定串口。用 --port 指定，或在分区表里写 \"port\"。\n")
        return 2

    print("整片擦除 W25Q64（8MB）…… 典型 20~100 秒")
    with FlashLink(port, baud) as link:
        link.chip_erase()
    print("擦除完成。")
    return 0


# ============================================================================
#  帮助文本
# ============================================================================
EPILOG = r"""
────────────────────────────────────────────────────────────────────────
  使用教程
────────────────────────────────────────────────────────────────────────

【1】硬件接线（HX4S20C 开发板）
    · USB Type-C 线接板上「串口」口（板载 CH340）
    · 板子上的 W25Q64 已焊好，无需外接

【2】分区表格式（JSON，路径相对分区表所在目录）
    {
      "port": "COM3",
      "baud": 921600,
      "partitions": [
        { "name": "boot",  "offset": "0x000000", "file": "bin/boot.bin" },
        { "name": "app",   "offset": "0x010000", "file": "bin/app.bin" },
        { "name": "font",  "offset": "0x400000", "file": "bin/font.bin" },
        { "name": "param", "offset": "0x7F0000", "file": "bin/param.bin", "size": "0x1000" }
      ]
    }
    字段：name / offset / file 必填；size 可选（用于读回长度和容量检查）

【3】典型流程
    python flash_tool.py list                      # 看有哪些串口
    python flash_tool.py id --port COM3            # 读 JEDEC ID，验证接线
    python flash_tool.py check -p partitions.json  # 只检查分区表，不连板子
    python flash_tool.py program -p partitions.json --port COM3 --verify
    python flash_tool.py dump -p partitions.json -d out/

【4】烧录行为说明
    · program 默认 **整片擦除（C7h）后** 再逐分区写入 —— 保证每次都是干净状态
    · 加 --skip-erase 跳过整片擦除，直接写（约省 20 秒）
      注意：Flash 只能把 1 写成 0。跳过擦除时若目标区域有旧数据，
            新数据会与旧数据按位相与，结果可能不正确。
            只有"该区域本来就是 0xFF"时才适合跳过。
    · 每个分区按 offset 从小到大依次写入
    · 分区表里 offset 必须按 4KB 对齐；不对齐会警告（扇区是擦除最小单位）

【5】常见故障
    · 读 ID 失败 / 超时
        - 确认串口号（用 list 看），确认没有别的程序占用串口
        - 确认波特率与 FPGA 里 top.v 的 BAUD 一致（默认 921600）
        - 若还不行，把 pin.adc 里 uart_rx / uart_tx 的 LOCATION 对调再综合
    · CRC 校验失败
        - 多为波特率不匹配或线太长，换短线或降到 115200 试
    · 写入后读回不对
        - 加 --verify 复现；确认分区没重叠

【6】命令一览
    list                                    列出串口
    id                                      读 JEDEC ID
    check   -p FILE                         检查分区表（不连板子）
    program -p FILE [--only NAME] [--verify] 整片擦除 + 烧录
    dump    -p FILE -d DIR                  按分区读回为 bin
    erase   -p FILE                         只做整片擦除
────────────────────────────────────────────────────────────────────────
"""


# ============================================================================
#  入口
# ============================================================================
def build_parser():
    p = argparse.ArgumentParser(
        prog="flash_tool.py",
        description="HX4S20C / W25Q64 SPI Flash 烧录上位机（分区表驱动）",
        epilog=EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--port", help="串口号，如 COM3（也可写在分区表里）")
    p.add_argument("--baud", type=int, help="波特率，默认 %d" % DEFAULT_BAUD)

    sub = p.add_subparsers(dest="cmd", metavar="<命令>")

    sp = sub.add_parser("list", help="列出可用串口")
    sp.set_defaults(func=cmd_list)

    sp = sub.add_parser("id", help="读 JEDEC ID（验证接线）")
    sp.set_defaults(func=cmd_id)

    sp = sub.add_parser("check", help="检查分区表（不连板子）")
    sp.add_argument("-p", "--partitions", required=True, help="分区表 JSON 路径")
    sp.set_defaults(func=cmd_check)

    sp = sub.add_parser("program", help="整片擦除后按分区表烧录")
    sp.add_argument("-p", "--partitions", required=True, help="分区表 JSON 路径")
    sp.add_argument("--only", help="只烧某个分区（按 name）")
    sp.add_argument("--verify", action="store_true", help="写后读回逐字节校验")
    sp.add_argument("--skip-erase", action="store_true",
                    help="跳过整片擦除，直接写入（快，但目标区域必须已是 0xFF）")
    sp.set_defaults(func=cmd_program)

    sp = sub.add_parser("dump", help="按分区表读回为 bin")
    sp.add_argument("-p", "--partitions", required=True, help="分区表 JSON 路径")
    sp.add_argument("-d", "--dir", default="dump", help="输出目录（默认 dump/）")
    sp.set_defaults(func=cmd_dump)

    sp = sub.add_parser("erase", help="整片擦除")
    sp.add_argument("-p", "--partitions", required=True, help="分区表 JSON 路径（用于取串口配置）")
    sp.set_defaults(func=cmd_erase)

    return p


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    if not getattr(args, "func", None):
        parser.print_help()
        return 0
    if args.cmd in ("list", "id", "program", "dump", "erase"):
        require_serial()
    try:
        return args.func(args)
    except KeyboardInterrupt:
        sys.stderr.write("\n已中断。\n")
        return 130
    except Exception as e:
        sys.stderr.write("\n[错误] %s\n" % e)
        return 1


if __name__ == "__main__":
    sys.exit(main())
