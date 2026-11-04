#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
chk_multidrv.py —— 查"同一个 reg 被两个 always 块驱动"。

为什么需要它：
    iverilog **不会**报这种错（仿真照跑，取哪个块的结果看运气），
    但 TD 综合会直接失败：
        ERROR: net 'ch_i[2]' is constantly driven from multiple places in xxx.v(451)
    这个错误在仿真里完全看不出来，只能靠静态检查提前抓。

用法:
    python chk_multidrv.py a.v b.v ...
    python chk_multidrv.py *.v
"""
import re
import sys
from collections import defaultdict


def blocks_of(path):
    """按 always 块切分，返回 [(起始行, {LHS 名: 首次出现行})]。"""
    src = open(path, encoding='utf-8', errors='replace').read().split('\n')
    out, cur = [], None
    for i, ln in enumerate(src):
        if re.match(r'\s*always\s*@', ln):
            if cur:
                out.append(cur)
            cur = {'line': i + 1, 'lhs': {}}
        elif cur is not None:
            if re.match(r'\s*endmodule', ln):
                out.append(cur)
                return out
            # 只认"语句开头"的 identifier <=，避开 (a <= b) 这类比较
            for m in re.finditer(
                    r'(?:^|[;]\s*|\bbegin\b|\belse\b)\s*([A-Za-z_]\w*)\s*(?:\[[^\]]*\]\s*)?<=', ln):
                cur['lhs'].setdefault(m.group(1), i + 1)
    if cur:
        out.append(cur)
    return out


def main():
    bad_total = 0
    for path in sys.argv[1:]:
        d = defaultdict(list)
        for b in blocks_of(path):
            for name, ln in b['lhs'].items():
                d[name].append((b['line'], ln))
        bad = {n: v for n, v in d.items() if len(v) > 1}
        if bad:
            bad_total += len(bad)
            print(f"!! {path}")
            for n, v in sorted(bad.items()):
                where = ', '.join(f"块@{b}(行{l})" for b, l in v)
                print(f"     {n}: {where}")
        else:
            print(f"ok {path}")
    return 1 if bad_total else 0


if __name__ == '__main__':
    sys.exit(main())
