"""时间对齐真值/估计 CSV 后计算位置精度；不把估计跟踪误差充当定位真值。"""
import argparse
import csv
import json
import math
from pathlib import Path


def evaluate(estimated, truth):
    def read(path):
        with path.open(encoding="utf-8") as file:
            rows = list(csv.DictReader(file))
        stamps = [int(x["t_us"]) for x in rows]
        if not rows or any(a >= b for a, b in zip(stamps, stamps[1:])):
            raise ValueError("timestamps must be strictly increasing")
        return rows
    reference = {int(x["t_us"]): x for x in read(truth)}
    errors = []
    for item in read(estimated):
        true = reference.get(int(item["t_us"]))
        if true is None:
            continue
        lat = (float(item["lat_rad"]) + float(true["lat_rad"])) / 2
        height = (float(item["height_m"]) + float(true["height_m"])) / 2
        d = 1 - .0066943799901413165 * math.sin(lat)**2
        rn, re = 6378137 * (1-.0066943799901413165) / d**1.5, 6378137 / math.sqrt(d)
        north = (float(true["lat_rad"]) - float(item["lat_rad"])) * (rn+height)
        east = math.remainder(float(true["lon_rad"])-float(item["lon_rad"]), 2*math.pi) * (re+height) * math.cos(lat)
        error = math.hypot(north, east)
        if not math.isfinite(error):
            raise ValueError("nonfinite position")
        errors.append(error)
    if not errors:
        raise ValueError("no exact aligned independent truth samples")
    errors.sort()
    return {"samples": len(errors), "horizontal_rms_m": math.sqrt(sum(x*x for x in errors)/len(errors)),
            "cep_m": errors[len(errors)//2], "horizontal_p95_m": errors[min(len(errors)-1,int(.95*len(errors)))], "horizontal_max_m": max(errors)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("estimate", type=Path)
    parser.add_argument("truth", type=Path)
    args = parser.parse_args()
    print(json.dumps(evaluate(args.estimate, args.truth), indent=2))
