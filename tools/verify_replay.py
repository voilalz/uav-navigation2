"""以独立 CSV 读取核对在线与离线回放结果；真值列不参与回放。"""
import argparse
import csv
import math


def verify(estimate, replay):
    with open(estimate, encoding="utf-8") as left, open(replay, encoding="utf-8") as right:
        a, b = list(csv.DictReader(left)), list(csv.DictReader(right))
    if not a or len(a) != len(b):
        raise ValueError("output row counts differ")
    maximum = 0.0
    for x, y in zip(a, b):
        for key in y:
            actual, expected = float(x[key]), float(y[key])
            if not math.isfinite(actual) or not math.isfinite(expected):
                raise ValueError("nonfinite output")
            difference = abs(actual - expected)
            maximum = max(maximum, difference)
            if difference > 1e-8 * max(1, abs(actual)):
                raise ValueError(f"replay mismatch at {x['t_us']}: {key}")
    return len(a), maximum


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("estimate")
    parser.add_argument("replay")
    args = parser.parse_args()
    rows, difference = verify(args.estimate, args.replay)
    print(f"replay verified: {rows} rows; max difference {difference:.3g}")
