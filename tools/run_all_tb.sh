#!/usr/bin/env bash
#==============================================================================
#  run_all_tb.sh —— 一键编译 + 运行全部 iverilog 测试台，输出 PASS/FAIL 汇总
#==============================================================================
#  用法（在工程根目录）：
#      bash tools/run_all_tb.sh                 # 全部（长台子也跑，可能要 30+ min）
#      bash tools/run_all_tb.sh -q              # 跳过长台子（见 LONG）
#      bash tools/run_all_tb.sh -j 4 osd quad   # 4 路并行，只跑名字含 osd 或 quad 的
#      bash tools/run_all_tb.sh -c              # 只编译，不运行
#
#  不维护文件清单：每个 tb 只给自己的 .v，其余模块靠 iverilog 的 `-y` 库搜索
#  （按「模块名.v」在 APP / APP/OSD / SD / 源码根 里找）。新加的 tb 自动纳入。
#  厂商原语用 APP/OSD/tools 下的仿真桩补上。
#
#  判定（读运行日志）：
#      含 "FAIL"            ⇒ FAIL
#      含 "PASS"、不含 FAIL  ⇒ PASS
#      都没有               ⇒ NOVERDICT（出图类台子，需人看 out/ 下的产物）
#      超过时限             ⇒ TIMEOUT
#  [float=N]：被测模块有 N 个输入口没接（悬空 Z）⇒ 该台结论不可信，需补接线。
#  日志落在 $OUT（默认 sim_out/tb_logs/），每个台子一个 .log。
#==============================================================================
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="$ROOT/src/user_source/hdl_source"
OUT="${OUT:-$ROOT/sim_out/tb_logs}"
JOBS=1
QUICK=0
COMPILE_ONLY=0
FILTERS=()

# 长台子：墙钟分钟级，-q 时跳过
LONG="tb_osd_keys tb_osd_render tb_app_full tb_chain_img tb_osd_popup tb_switch tb_trans"
# 单台默认时限（秒）；长台子另给
T_DEFAULT=600
T_LONG=3600

while [ $# -gt 0 ]; do
    case "$1" in
        -j) JOBS="$2"; shift 2 ;;
        -q) QUICK=1; shift ;;
        -c) COMPILE_ONLY=1; shift ;;
        -h|--help) sed -n 2,22p "$0"; exit 0 ;;
        *) FILTERS+=("$1"); shift ;;
    esac
done

mkdir -p "$OUT"

# ---- 全链路仿真用的 FAT32 镜像（不入库，缺了就现做）----
# mk_fat32_img.py 同时生成 fat32_10img.img / .map / _map.vh
if [ ! -f "$ROOT/sim_data/fat32_10img.img" ]; then
    mkdir -p "$ROOT/sim_data"
    python "$ROOT/tools/mk_fat32_img.py" \
        --out "$ROOT/sim_data/fat32_10img.img" --check > /dev/null \
        && echo "== built sim_data/fat32_10img.img"
fi

# ---- 台子依赖的生成物（不入库，缺了就现做；否则 SD 模型全读 0xFF，台子白跑）----
# 默认用 gen_grid_bmp.py 现画一张 640×480 测试图；也可用 CARD_SRC=xxx.bmp 指定素材。
CARD="$SRC/SD/out/card_img.bin"
if [ ! -f "$CARD" ]; then
    mkdir -p "$SRC/SD/out"
    CARD_SRC="${CARD_SRC:-}"
    if [ -z "$CARD_SRC" ]; then
        python "$ROOT/tools/gen_grid_bmp.py" --outdir "$ROOT/tools/out" > /dev/null
        CARD_SRC="$ROOT/tools/out/grid_red_640x480.bmp"
    fi
    PYTHONIOENCODING=utf-8 python "$ROOT/tools/gen_sd_image.py" build "$CARD" \
        "0:$(cygpath -m "$CARD_SRC" 2>/dev/null || echo "$CARD_SRC")" > /dev/null \
        && echo "== built $CARD (tb_chain_img)"
fi

# 等价台的参考模块：面积优化前的版本（固定提交，提交后依然有意义；行为有意改动时改这个号）
#   tb_sched_equiv ← sdr_port_sched_ref（7e6ad32 = 2026-10-31 面积优化前）
REF_REV="${REF_REV:-7e6ad32}"
REFDIR="$SRC/SD/out/ref"
mkdir -p "$REFDIR"
git -C "$ROOT" show "$REF_REV:src/user_source/hdl_source/SD/sdr_port_sched.v" 2>/dev/null \
    | sed 's/^module sdr_port_sched/module sdr_port_sched_ref/' > "$REFDIR/sdr_port_sched_ref.v"

export SRC OUT COMPILE_ONLY T_DEFAULT T_LONG LONG

run_one() {
    f="$1"
    d="$(dirname "$f")"
    n="$(basename "$f" .v)"
    log="$OUT/$n.log"
    vvp_f="$OUT/$n.vvp"
    tl=$T_DEFAULT
    case " $LONG " in *" $n "*) tl=$T_LONG ;; esac

    t0=$(date +%s)
    if ! ( cd "$d" && iverilog -g2012 -Wall -DIVERILOG_SIM -Y .v \
            -y . -y "$SRC/APP" -y "$SRC/APP/OSD" -y "$SRC/SD" -y "$SRC" -y "$SRC/ref" -y "$SRC/SD/out/ref" \
            -s "$n" -o "$vvp_f" "$n.v" \
            "$SRC/APP/OSD/tools/mult18_stub.v" "$SRC/APP/OSD/tools/bram32k_stub.v" ) \
            > "$log" 2>&1; then
        printf "%-10s %-24s %5ss  %s\n" "NOCOMPILE" "$n" 0 "$(grep -m1 -i error "$log")"
        return
    fi
    # 悬空输入口 = Z ⇒ 结果可能无效（Q8 全黑就是这么来的），单独标出来
    nf=$(grep -c "dangling input port" "$log")
    fl=""; [ "$nf" -gt 0 ] && fl="[float=$nf] "
    if [ "$COMPILE_ONLY" = 1 ]; then
        printf "%-10s %-24s        %s\n" "COMPILED" "$n" "$fl"; return
    fi

    # 在 tb 自己的目录里跑：$readmemh / INIT_HEX 都是相对它写的
    ( cd "$d" && timeout "$tl" vvp -n "$vvp_f" ) >> "$log" 2>&1
    rc=$?
    dt=$(( $(date +%s) - t0 ))
    if [ $rc -eq 124 ]; then v=TIMEOUT
    elif grep -q "FAIL" "$log"; then v=FAIL
    elif grep -q "PASS" "$log"; then v=PASS
    else v=NOVERDICT
    fi
    detail=""
    [ "$v" = FAIL ] && detail="$(grep -m1 "FAIL" "$log" | sed 's/^ *//' | cut -c1-70)"
    printf "%-10s %-24s %5ss  %s%s\n" "$v" "$n" "$dt" "$fl" "$detail"
}
export -f run_one

# 收集台子
list=()
while IFS= read -r f; do
    n="$(basename "$f" .v)"
    if [ "$QUICK" = 1 ]; then case " $LONG " in *" $n "*) continue ;; esac; fi
    if [ ${#FILTERS[@]} -gt 0 ]; then
        hit=0; for p in "${FILTERS[@]}"; do case "$n" in *"$p"*) hit=1 ;; esac; done
        [ $hit = 1 ] || continue
    fi
    list+=("$f")
done < <(cd "$SRC" && find "$SRC" -name "tb_*.v" | sort)

echo "== ${#list[@]} testbenches, jobs=$JOBS, logs -> $OUT"
printf "%s\n" "${list[@]}" | xargs -P "$JOBS" -I{} bash -c 'run_one "$@"' _ {} | tee "$OUT/summary.txt"

echo "== summary:"
awk '{c[$1]++} END {for (k in c) printf "   %-10s %d\n", k, c[k]}' "$OUT/summary.txt" | sort
grep -qE "^(FAIL|NOCOMPILE|TIMEOUT)" "$OUT/summary.txt" && exit 1 || exit 0
