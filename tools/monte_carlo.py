"""多随机种子的数字仿真；使用 '-' 关闭大体积原始日志，仅汇总实际输出。"""
import argparse
import concurrent.futures
import json
import math
import statistics
import subprocess
from pathlib import Path


def run(executable, duration, seed, scenario):
    process = subprocess.run([str(executable), str(duration), str(seed), "-", scenario], capture_output=True, text=True, check=True)
    data = json.loads(process.stdout)
    for key in ("horizontal_rms_m", "horizontal_p95_m", "output_hz", "position_nees_mean"):
        if not math.isfinite(data[key]):
            raise ValueError(f"nonfinite {key} for seed {seed}")
    return data


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("executable", type=Path)
    parser.add_argument("--duration", type=float, default=1200)
    parser.add_argument("--runs", type=int, default=100)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--scenario", choices=["combined", "relative", "inertial"], default="combined")
    parser.add_argument("--output", type=Path, default=Path("out/monte-carlo.json"))
    args = parser.parse_args()
    if not 1 <= args.runs <= 10000 or not 1 <= args.workers <= 32:
        parser.error("runs 1..10000, workers 1..32")
    with concurrent.futures.ThreadPoolExecutor(args.workers) as pool:
        futures = [pool.submit(run, args.executable.resolve(), args.duration, seed + 1, args.scenario) for seed in range(args.runs)]
        results = [future.result() for future in futures]
    summary = {"runs": len(results), "duration_s": args.duration, "scenario": args.scenario,
               "rms_mean_m": statistics.mean(x["horizontal_rms_m"] for x in results),
               "rms_min_m": min(x["horizontal_rms_m"] for x in results),
               "rms_max_m": max(x["horizontal_rms_m"] for x in results),
               "worst_horizontal_max_m": max(x["horizontal_max_m"] for x in results),
               "output_hz_min": min(x["output_hz"] for x in results),
               "total_injected_outliers": sum(x["injected_outliers"] for x in results),
               "total_outliers_rejected": sum(x["injected_outliers_rejected"] for x in results),
               "position_nees_mean": statistics.mean(x["position_nees_mean"] for x in results),
               "results": results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(summary, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    print(json.dumps({k: v for k, v in summary.items() if k != "results"}, indent=2))
