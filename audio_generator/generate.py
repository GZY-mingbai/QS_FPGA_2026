#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
批量音频生成脚本（火山引擎 / 豆包 Seed-Audio 1.0，非流式 HTTP 接口）

流程：
  1. 读取 prompt.json，把 text / bgm / voice_style 拼成 text_prompt
  2. POST 到 https://openspeech.bytedance.com/api/v3/tts/create 生成音频（mp3）
  3. 用 ffmpeg 给音频前后各加 0.5s 静音，并在人声起止处各做 0.5s 淡入淡出
  4. 按指定参数转成 48kHz / 双声道 / s16le 的 PCM WAV，文件名与 prompt.json 的 file 字段一致

用法：
  set BYTEDANCE_API_KEY=xxxx            (Windows CMD)
  $env:BYTEDANCE_API_KEY="xxxx"          (PowerShell)
  export BYTEDANCE_API_KEY=xxxx          (bash)

  python generate.py                     # 全部生成（默认 3 并发 + 进度条）
  python generate.py --only TOP_01 SHOE   # 只处理文件名包含这些关键词的条目
  python generate.py --force             # 已存在也重新生成
  python generate.py --workers 5         # 并发数（注意接口 QPS 限制）
  python generate.py --interval 0        # 提交间隔，0 为不限制；默认 0.3s
  python generate.py --speech-rate 35    # 语速更快（默认 20，0 为标准，上限 100）
  python generate.py --keep-punct        # 保留句末标点（默认去掉，减少句间停顿）
  python generate.py --pace ""           # 不写节奏描述（默认强调"少停顿、别断句"）
  python generate.py --no-progress       # 关掉进度条，输出纯日志（适合重定向到文件）
  python generate.py --dry-run           # 只打印将要提交的请求体，不真正调用

依赖：Python 3.9+，ffmpeg / ffprobe 在 PATH 中（仅用标准库发请求，无第三方包）
"""

import argparse
import base64
import concurrent.futures
import json
import os
import shutil
import subprocess
import sys
import threading
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path

# ----------------------------------------------------------------------------- 配置

API_URL = "https://openspeech.bytedance.com/api/v3/tts/create"
MODEL = "seed-audio-1.0"

# 接口返回的音频格式（mp3 体积小，后面统一用 ffmpeg 转 WAV）
API_AUDIO_FORMAT = "mp3"
API_SAMPLE_RATE = 44100

# 声音风格：如果后续想固定某个音色，把音色 ID 填进来（会走 speaker 字段）
# 例：SPEAKER_ID = "zh_female_xxx_bigtts"
SPEAKER_ID = ""

# text_prompt 模板，{voice_style} / {emotion} / {pace} / {text} / {bgm} / {bgm_level} 会被替换
PROMPT_TEMPLATE = (
    "请生成一段中文带货配音。"
    "语气要求：{voice_style}。"
    "情感要求：{emotion}。"
    "节奏要求：{pace}。"
    "需要朗读的文案是：「{text}」"
    "背景音乐：{bgm}。{bgm_level}"
    "整体连贯自然。"
)

# 配乐存在感的档位。注意：配乐和人声是模型一次性混出来的，
# 后期无法单独提亮，所以只能靠这里把描述写足。
BGM_LEVELS = {
    "normal": "配乐音量适度，作为背景轻轻衬托人声，氛围自然。",
    "strong": (
        "配乐音量偏大、存在感明显，旋律和节奏清晰可辨，作为氛围主体之一；"
        "人声依然在音乐前面，咬字清楚不被盖住。"
    ),
    "max": (
        "配乐音量很大、非常突出，像广告片一样音乐铺满整个耳朵，"
        "鼓点与旋律层次分明、有冲击力；人声略微靠后但仍字字清楚。"
    ),
}
DEFAULT_BGM_LEVEL = "strong"

# 情感描述：避免平铺直叙的播报感
EMOTION_PROMPT = (
    "要有真感情，情绪饱满有感染力，像真人面对面热情推荐，"
    "语气有起伏和重音，避免平铺直叙和机械播报感"
)

# 节奏描述：目标是少停顿、不断句、一口气顺下来
PACE_PROMPT = "语速偏快，句子之间不要有明显停顿，保持连贯流畅，避免拖长尾音和逐字断开"

# 语速参数 audio_config.speech_rate：0 为默认，取值 [-50, 100]
# 100 ≈ 2.0 倍速，-50 ≈ 0.5 倍速；这里默认比标准快一档
SPEECH_RATE = 20

# 去掉文案里的句末标点（。！？；），减少模型在标点处的长停顿
STRIP_SENTENCE_PUNCT = True

# 并发提交时每条请求之间的最小间隔（秒）；并发 + 小间隔即可，太大反而拖慢
DEFAULT_INTERVAL = 0.3

# 静音 / 淡入淡出参数（秒）
PAD_SEC = 0.5
FADE_SEC = 0.5

# 输出 WAV 参数（对应你给的 ffmpeg 命令）
WAV_SAMPLE_RATE = 48000
WAV_CHANNELS = 2

FFMPEG = "ffmpeg"


# ----------------------------------------------------------------------------- 工具函数

def log(msg: str) -> None:
    print(msg, flush=True)


class Progress:
    """
    线程安全的单行进度条（纯标准库实现，不依赖 tqdm）。
    非 TTY（重定向到文件 / CI）时自动降级为逐行日志。
    进度条上只显示 ASCII 字符，避免 Windows 控制台 GBK 编码报错。
    """

    def __init__(self, total: int, enabled: bool = True):
        self.total = max(total, 1)
        self.done = 0
        self.ok = 0
        self.fail = 0
        self.skip = 0
        self.current = ""
        self.enabled = enabled and total > 0
        self.lock = threading.Lock()
        self.width = 26
        self.start = time.time()
        self._printed = 0

    def _bar(self) -> str:
        ratio = min(self.done / self.total, 1.0)
        filled = int(self.width * ratio)
        return "[" + "#" * filled + "-" * (self.width - filled) + "]"

    def _line(self) -> str:
        elapsed = time.time() - self.start
        rate = self.done / elapsed if elapsed > 0 and self.done else 0
        eta = (self.total - self.done) / rate if rate > 0 and self.done < self.total else 0
        pct = self.done / self.total * 100
        cur = self.current
        try:
            cols = shutil.get_terminal_size((100, 20)).columns
        except Exception:
            cols = 100
        tail = (f" {pct:5.1f}% {self.done}/{self.total}"
                f" ok:{self.ok} fail:{self.fail} skip:{self.skip} {elapsed:0.0f}s")
        if eta:
            tail += f" eta:{eta:0.0f}s"
        if cur:
            room = cols - len(self._bar()) - len(tail) - 2
            if room > 6:
                tail += " " + (cur if len(cur) <= room else cur[: room - 3] + "...")
        return self._bar() + tail

    def _draw(self) -> None:
        if not self.enabled:
            return
        line = self._line()
        sys.stdout.write("\r" + line + " " * max(0, self._printed - len(line)))
        sys.stdout.flush()
        self._printed = len(line)

    def update(self, name: str = "", status: str = "ok", message: str | None = None) -> None:
        """
        完成一条任务：done +1，按 status 计入 ok / fail / skip。
        message 非空时先清屏打印一条独立日志，再重画进度条，避免日志被 \\r 覆盖。
        """
        with self.lock:
            self.done += 1
            if status == "ok":
                self.ok += 1
            elif status == "skip":
                self.skip += 1
            else:
                self.fail += 1
            self.current = name
            if self.enabled:
                if message:
                    sys.stdout.write("\r" + " " * self._printed + "\r")
                    print(message, flush=True)
                    self._printed = 0
                self._draw()
            elif message:
                print(message, flush=True)

    def finish(self) -> None:
        with self.lock:
            if self.enabled:
                sys.stdout.write("\r" + " " * self._printed + "\r")
                self.current = ""
                self._printed = 0
                print(self._line(), flush=True)
            self.enabled = False


class Pacer:
    """控制请求发起的最小间隔，避免并发打满触发 45000003 限频。"""

    def __init__(self, interval: float):
        self.interval = max(interval, 0.0)
        self._next = 0.0
        self._lock = threading.Lock()

    def wait(self) -> None:
        if self.interval <= 0:
            return
        with self._lock:
            now = time.monotonic()
            if now < self._next:
                time.sleep(self._next - now)
                now = time.monotonic()
            self._next = now + self.interval


# 句末停顿的来源之一就是文案里的标点，去掉后模型会连着念
PUNCT_MAP = str.maketrans({c: "" for c in "。！？；!?"})


def build_prompt(item: dict, pace: str = PACE_PROMPT, strip_punct: bool = True,
                 emotion: str = EMOTION_PROMPT, bgm_level: str = DEFAULT_BGM_LEVEL) -> str:
    text = item.get("text", "").strip()
    if strip_punct:
        text = text.translate(PUNCT_MAP).strip()
    pace_desc = pace.strip().rstrip("。") if pace else "自然流畅"
    emotion_desc = emotion.strip().rstrip("。") if emotion else "自然真诚"
    if bgm_level in BGM_LEVELS:
        level_desc = BGM_LEVELS[bgm_level]
    else:
        level_desc = bgm_level  # 允许直接传一段自定义描述
    return PROMPT_TEMPLATE.format(
        voice_style=item.get("voice_style", "").strip(),
        emotion=emotion_desc,
        text=text,
        bgm=item.get("bgm", "").strip(),
        pace=pace_desc,
        bgm_level=level_desc,
    )


def build_payload(item: dict, args=None) -> dict:
    pace = getattr(args, "pace", PACE_PROMPT)
    emotion = getattr(args, "emotion", EMOTION_PROMPT)
    bgm_level = getattr(args, "bgm_level", DEFAULT_BGM_LEVEL)
    strip_punct = getattr(args, "strip_punct", STRIP_SENTENCE_PUNCT)
    speech_rate = getattr(args, "speech_rate", SPEECH_RATE)

    audio_config = {
        "format": API_AUDIO_FORMAT,
        "sample_rate": API_SAMPLE_RATE,
    }
    if speech_rate:
        audio_config["speech_rate"] = int(speech_rate)

    payload = {
        "model": MODEL,
        "text_prompt": build_prompt(item, pace=pace, strip_punct=strip_punct,
                                    emotion=emotion, bgm_level=bgm_level),
        "audio_config": audio_config,
        "watermark": {"aigc_watermark": False, "aigc_metadata": {"enable": False}},
    }
    if SPEAKER_ID:
        payload["speaker"] = SPEAKER_ID
    return payload


class ApiError(RuntimeError):
    """带业务码的接口错误。retryable 决定是否值得重试。"""

    def __init__(self, code, message: str, logid: str = "-", retryable: bool = False):
        self.code = code
        self.logid = logid
        self.retryable = retryable
        hint = ERROR_HINTS.get(code)
        text = f"code={code} message={message} (logid={logid})"
        if hint:
            text += "\n    可能原因：" + hint
        super().__init__(text)


# 出现这些码说明整批都跑不通，直接中止
FATAL_CODES = {45000001, 45000030}

# 常见错误码 -> 排查方向（45000030 是最容易踩的）
ERROR_HINTS = {
    45000030: (
        "该 API Key 所属项目未被授权使用本服务（resource not granted）。"
        "请检查：① 语音技术控制台是否已开通「音频生成/Seed-Audio」服务并有可用额度；"
        "② API Key 创建时选择的项目，与开通该服务的项目是否为同一个；"
        "③ 若用旧版控制台，需用 appid 对应的 X-Api-App-Id + X-Api-Access-Key 双头鉴权，"
        "或改用新版控制台 key。"
    ),
    45000001: "鉴权失败：API Key 无效、已删除或与当前接口不匹配。",
    45000002: "参数错误：检查 model / text_prompt / speaker 等字段是否合法。",
    45000003: "请求频率超限：降低 --workers 或增加间隔后重试。",
    45000004: "配额不足：资源包或免费额度已用尽，请在控制台续购。",
}


def build_headers(auth_mode: str, api_key: str, app_id: str, access_key: str) -> dict:
    """新版控制台用 X-Api-Key 单头；旧版用 X-Api-App-Id + X-Api-Access-Key 双头。"""
    headers = {
        "Content-Type": "application/json",
        "X-Api-Request-Id": str(uuid.uuid4()),
    }
    if auth_mode == "legacy":
        if not (app_id and access_key):
            raise SystemExit("legacy 鉴权需要同时提供 --app-id 和 --access-key")
        headers["X-Api-App-Id"] = app_id
        headers["X-Api-Access-Key"] = access_key
    else:
        if not api_key:
            raise SystemExit("缺少 API Key：设置环境变量 BYTEDANCE_API_KEY 或传 --api-key")
        headers["X-Api-Key"] = api_key
    return headers


def http_post_json(url: str, payload: dict, headers: dict, timeout: int = 180) -> dict:
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    req = urllib.request.Request(url, data=body, headers=headers, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        raw = resp.read()
        resp_headers = dict(resp.headers)
    try:
        data = json.loads(raw.decode("utf-8"))
    except json.JSONDecodeError:
        raise RuntimeError(f"响应不是合法 JSON：{raw[:500]!r}")
    return {"data": data, "headers": resp_headers}


def get_logid(headers: dict) -> str:
    for k, v in headers.items():
        if k.lower() == "x-tt-logid":
            return str(v)
    return "-"


def call_api(payload: dict, headers: dict, retries: int = 3, timeout: int = 180) -> dict:
    """提交生成请求。仅对网络抖动 / 429 / 5xx 重试；业务错误（如 45000030）直接抛出。"""
    last_err = None
    for attempt in range(1, retries + 1):
        try:
            result = http_post_json(API_URL, payload, headers, timeout=timeout)
            data = result["data"]
            logid = get_logid(result["headers"])
            code = data.get("code")
            if code in (0, 200, 20000000, None) and (data.get("audio") or data.get("url")):
                return data
            msg = data.get("message") or data.get("msg") or str(data)
            raise ApiError(code, msg, logid, retryable=code in (429,))

        except urllib.error.HTTPError as e:
            detail = ""
            try:
                body = json.loads(e.read().decode("utf-8", "replace"))
                detail = body.get("message", "")
                code = body.get("code", e.code)
            except Exception:
                code, detail = e.code, ""
            last_err = ApiError(code, detail or f"HTTP {e.code}",
                                retryable=e.code in (429, 500, 502, 503, 504))
            if not last_err.retryable:
                raise last_err
        except ApiError as e:
            last_err = e
            if not e.retryable:
                raise
        except Exception as e:  # 网络层错误（超时、连接重置）值得重试
            last_err = e

        if attempt < retries:
            wait = min(2 ** attempt, 20)
            log(f"  [重试 {attempt}/{retries - 1}] {last_err}，{wait}s 后重试")
            time.sleep(wait)
    raise RuntimeError(f"请求失败：{last_err}")


def check_auth(headers: dict, timeout: int = 60) -> int:
    """最小请求探活：判断 key 是否被授权调用本服务，不消耗额度（文案极短）。"""
    payload = {
        "model": MODEL,
        "text_prompt": "你好",
        "audio_config": {"format": "mp3", "sample_rate": 44100},
    }
    log("正在用最小请求探测鉴权与授权状态 ...")
    try:
        data = call_api(payload, headers, retries=1, timeout=timeout)
        log(f"授权正常，已返回音频；原始时长 {data.get('original_duration')}s")
        return 0
    except Exception as e:
        log(f"探测失败：{e}")
        return 1


def save_audio(data: dict, dst: Path) -> None:
    """把返回的 base64 音频写入文件；没有 base64 时退回下载 url。"""
    b64 = data.get("audio")
    if b64:
        dst.write_bytes(base64.b64decode(b64))
        return
    url = data.get("url")
    if not url:
        raise RuntimeError("响应里既没有 audio 也没有 url")
    with urllib.request.urlopen(url, timeout=120) as resp:
        dst.write_bytes(resp.read())


def run(cmd: list) -> None:
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if proc.returncode != 0:
        raise RuntimeError(
            "命令执行失败：\n  " + " ".join(cmd)
            + "\n  " + proc.stderr.decode("utf-8", "replace").strip()[:1000]
        )


def process_audio(src: Path, dst: Path, pad: float, fade: float) -> None:
    """
    静音 + 淡入淡出，全部作用在人声本身，不靠"读时长再算时间点"（mp3 时长常有误差）。

    思路：把音频翻转一次 → 尾静音跑到头部之外，此时"前 fade 秒"就是原始音频的最后 fade 秒；
    淡入后翻回来，就得到精确的尾部淡出。

    1) areverse + afade=in:st=0:d=fade + areverse    → 人声最后 fade 秒淡出
    2) adelay=pad                                     → 前置 pad 秒静音
    3) afade=in:st=pad:d=fade                         → 人声开头 fade 秒淡入（跳过静音区）
    4) apad=pad_dur=pad                              → 后置 pad 秒静音
    5) 直接输出 48kHz / 双声道 / pcm_s16le（等价于
       ffmpeg -i XXX.mp3 -ar 48000 -ac 2 -sample_fmt s16 -c:a pcm_s16le XXXXXX.WAV）
    """
    pad_ms = int(round(pad * 1000))
    af = (
        f"areverse,"
        f"afade=t=in:st=0:d={fade},"
        f"areverse,"
        f"adelay={pad_ms}:all=1,"
        f"afade=t=in:st={pad}:d={fade},"
        f"apad=pad_dur={pad}"
    )
    cmd = [
        FFMPEG, "-y", "-v", "error", "-i", str(src),
        "-af", af,
        "-ar", str(WAV_SAMPLE_RATE),
        "-ac", str(WAV_CHANNELS),
        "-sample_fmt", "s16",
        "-c:a", "pcm_s16le",
        str(dst),
    ]
    run(cmd)


# ----------------------------------------------------------------------------- 单条任务

def handle_item(item: dict, headers: dict, out_dir: Path, raw_dir: Path,
                args, pacer: "Pacer | None" = None) -> tuple:
    """处理单条任务。返回 (状态文字, 是否跳过)。"""
    stem = Path(item["file"]).stem            # TOP_01
    final_path = out_dir / f"{stem}.WAV"
    raw_path = raw_dir / f"{stem}.mp3"

    if final_path.exists() and not args.force:
        return f"[跳过] {final_path.name} 已存在（--force 可覆盖）", True

    payload = build_payload(item, args)
    if args.dry_run:
        return f"[dry-run] {stem}: {json.dumps(payload, ensure_ascii=False)}", False

    # 1. 生成
    duration = None
    if not (raw_path.exists() and not args.force):
        if pacer:
            pacer.wait()
        data = call_api(payload, headers, retries=args.retries)
        save_audio(data, raw_path)
        duration = data.get("original_duration")

    # 2. 加静音 + 淡入淡出 + 转 WAV
    process_audio(raw_path, final_path, args.pad, args.fade)
    suffix = f"，原始时长 {duration}s" if duration is not None else ""
    return f"[完成] {final_path.name}{suffix}", False


# ----------------------------------------------------------------------------- 主流程

def main() -> int:
    parser = argparse.ArgumentParser(description="批量生成音频并转 WAV")
    parser.add_argument("-i", "--input", default="prompt.json", help="任务清单 JSON")
    parser.add_argument("-o", "--out-dir", default="out", help="WAV 输出目录")
    parser.add_argument("--raw-dir", default="out/raw", help="接口原始 mp3 存放目录")
    parser.add_argument("--api-key", default=os.environ.get("BYTEDANCE_API_KEY", ""),
                        help="新版控制台 API Key，默认读环境变量 BYTEDANCE_API_KEY")
    parser.add_argument("--auth", choices=["new", "legacy"], default="new",
                        help="new=X-Api-Key 单头（默认）；legacy=旧版双头鉴权")
    parser.add_argument("--app-id", default=os.environ.get("VOLC_APP_ID", ""), help="旧版控制台 AppID")
    parser.add_argument("--access-key", default=os.environ.get("VOLC_ACCESS_KEY", ""),
                        help="旧版控制台 Access Key")
    parser.add_argument("--check", action="store_true",
                        help="只用一条最小请求探测鉴权/服务是否已开通，不批量生成")
    parser.add_argument("--only", nargs="*", default=None,
                        help="只处理文件名包含这些关键词的条目，如 TOP_01 SHOE")
    parser.add_argument("--force", action="store_true", help="已存在也重新生成")
    parser.add_argument("--dry-run", action="store_true", help="只打印请求体")
    parser.add_argument("--workers", type=int, default=3,
                        help="并发数（默认 3；接口限频时调小或加大 --interval）")
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL,
                        help=f"两次请求提交的最小间隔秒数（默认 {DEFAULT_INTERVAL}，0 为不限制）")
    parser.add_argument("--retries", type=int, default=3, help="失败重试次数")
    parser.add_argument("--speech-rate", type=int, default=SPEECH_RATE,
                        help=f"语速，[-50,100]，越大越快（默认 {SPEECH_RATE}，0 为标准）")
    parser.add_argument("--pace", default=PACE_PROMPT,
                        help="写进 text_prompt 的节奏描述，默认强调少停顿；传空串则不加")
    parser.add_argument("--emotion", default=EMOTION_PROMPT,
                        help="写进 text_prompt 的情感描述，默认强调有感情；传空串则不加")
    parser.add_argument("--bgm-level", default=DEFAULT_BGM_LEVEL,
                        choices=list(BGM_LEVELS) + ["custom"],
                        help="配乐存在感：normal / strong（默认）/ max；custom 需配 --bgm-desc")
    parser.add_argument("--bgm-desc", default="",
                        help="配合 --bgm-level custom 使用，直接给一段配乐描述")
    parser.add_argument("--keep-punct", dest="strip_punct", action="store_false",
                        help="保留文案里的句末标点（默认会去掉，减少句间停顿）")
    parser.add_argument("--no-progress", dest="progress", action="store_false",
                        help="关闭进度条，输出纯日志（便于重定向到文件）")
    parser.add_argument("--pad", type=float, default=PAD_SEC, help="前后静音秒数")
    parser.add_argument("--fade", type=float, default=FADE_SEC, help="淡入淡出秒数")
    args = parser.parse_args()

    if args.bgm_level == "custom":
        args.bgm_level = args.bgm_desc.strip() or BGM_LEVELS[DEFAULT_BGM_LEVEL]

    if args.check and not args.dry_run:
        return check_auth(build_headers(args.auth, args.api_key, args.app_id, args.access_key))

    src_json = Path(args.input)
    if not src_json.exists():
        log(f"找不到任务清单：{src_json}")
        return 2
    items = json.loads(src_json.read_text(encoding="utf-8"))

    if args.only:
        keys = [k.upper() for k in args.only]
        items = [it for it in items if any(k in it["file"].upper() for k in keys)]
        if not items:
            log("按 --only 过滤后没有任务")
            return 2

    headers = {}
    if not args.dry_run:
        headers = build_headers(args.auth, args.api_key, args.app_id, args.access_key)
        if shutil.which(FFMPEG) is None:
            log(f"找不到 {FFMPEG}，请先安装 ffmpeg 并加入 PATH")
            return 2

    out_dir = Path(args.out_dir)
    raw_dir = Path(args.raw_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    raw_dir.mkdir(parents=True, exist_ok=True)

    log(f"共 {len(items)} 条任务 → {out_dir.resolve()}  并发={args.workers}  间隔={args.interval}s")
    bar = Progress(len(items), enabled=(args.progress and not args.dry_run and sys.stdout.isatty()))
    pacer = Pacer(args.interval)

    def worker(item):
        """返回 (是否成功, 日志文本, 是否属于整批级致命错误, 是否跳过)。"""
        try:
            msg, skipped = handle_item(item, headers, out_dir, raw_dir, args, pacer)
            return True, msg, False, skipped
        except ApiError as e:
            # 授权/鉴权类错误对整批都成立，继续跑只是浪费时间和额度
            return False, f"[失败] {item.get('file')}: {e}", e.code in FATAL_CODES, False
        except Exception as e:
            return False, f"[失败] {item.get('file')}: {e}", False, False

    def report(item, success, msg, skipped):
        if success and skipped:
            bar.update(item["file"], "skip", msg)
        elif success:
            bar.update(item["file"], "ok", None if bar.enabled else msg)
        else:
            bar.update(item["file"], "fail", msg)

    fatal = False

    if args.workers <= 1:
        for item in items:
            if fatal:
                break
            success, msg, is_fatal, skipped = worker(item)
            report(item, success, msg, skipped)
            if is_fatal:
                fatal = True
                log("  → 授权类错误，已中止后续任务（修好后可用 --check 复验）")
    else:
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as pool:
            futures = {pool.submit(worker, it): it for it in items}
            for fut in concurrent.futures.as_completed(futures):
                item = futures[fut]
                success, msg, is_fatal, skipped = fut.result()
                report(item, success, msg, skipped)
                if is_fatal and not fatal:
                    fatal = True
                    for f in futures:
                        f.cancel()
                    log("  → 授权类错误，已取消尚未开始的任务（修好后可用 --check 复验）")

    bar.finish()
    ok, fail, skipped = bar.ok, bar.fail, bar.skip
    log(f"\n汇总：成功 {ok} 条 / 跳过 {skipped} 条 / 失败 {fail} 条；WAV 目录 {out_dir.resolve()}")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
