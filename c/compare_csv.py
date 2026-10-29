#!/usr/bin/env python3
"""比对两份状态 CSV（参考序列 vs C 运行器输出）。

用法：
    python3 c/compare_csv.py c/ref/quad_step_ref.csv c/out/quad_step.csv

任一样本或通道超过容差就返回非零，供 CI 使用。
"""

import argparse
import csv
import sys


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    header = rows[0]
    data = [[float(v) for v in r] for r in rows[1:] if r]
    return header, data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ref")
    ap.add_argument("cand")
    ap.add_argument("--tol", type=float, default=1e-9)
    a = ap.parse_args()

    h_ref, d_ref = load(a.ref)
    h_cand, d_cand = load(a.cand)

    if h_ref != h_cand:
        print("header mismatch:\n  %s\n  %s" % (h_ref, h_cand))
        return 1
    if len(d_ref) != len(d_cand):
        print("row count mismatch: %d vs %d" % (len(d_ref), len(d_cand)))
        return 1

    worst = 0.0
    worst_at = None
    for i, (r1, r2) in enumerate(zip(d_ref, d_cand)):
        if len(r1) != len(r2):
            print("column count mismatch at row %d" % i)
            return 1
        for j, (x, y) in enumerate(zip(r1, r2)):
            e = abs(x - y)
            if e > worst:
                worst = e
                worst_at = (i, h_ref[j])
            if e > a.tol:
                print("FAIL row %d col %s: %.17g vs %.17g (%.3e)"
                      % (i, h_ref[j], x, y, e))
                return 1

    print("OK %d rows, worst |diff| = %.3e at %s"
          % (len(d_ref), worst, worst_at))
    return 0


if __name__ == "__main__":
    sys.exit(main())
