"""解析误差预算，仅用于量级比较；不是可观测性或滤波精度保证。"""
import argparse
import json
import math


def budget(t_s, gyro_deg_h=10, accel_mg=2, speed_m_s=600/3.6, vo_error=0.01, heading_deg=.3, fix_sigma_m=15):
    gyro = math.radians(gyro_deg_h)/3600
    accel = accel_mg * 9.80665 / 1000
    position_accel = .5*accel*t_s*t_s
    position_tilt = 9.80665*gyro*t_s**3/6
    relative = speed_m_s*t_s*math.hypot(vo_error, math.radians(heading_deg))
    return {"time_s": t_s, "distance_km": speed_m_s*t_s/1000,
            "pure_ins_bias_rss_m": math.hypot(position_accel, position_tilt),
            "assisted_search_3sigma_m": 3*math.hypot(fix_sigma_m, relative),
            "gsd_m_at_1km_16mm": 1000*3.45e-6/.016,
            "blur_px_at_400us": speed_m_s*.0004/(1000*3.45e-6/.016)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--times", type=float, nargs="+", default=[60, 180, 300, 600, 1200])
    args = parser.parse_args()
    if any(not math.isfinite(t) or t <= 0 for t in args.times):
        parser.error("times must be positive finite seconds")
    print(json.dumps([budget(t) for t in args.times], indent=2))
