"""把视觉服务真实 JSONL 量测编码为 MCU 协议；保留曝光时间及二维 NED 语义。"""
import argparse
import json
import math
import socket
import struct
import sys
from pathlib import Path
from .nav_receiver import crc16


def frame(msg_id, payload, sequence):
    if not 0 <= sequence < 65536 or len(payload) > 255:
        raise ValueError("invalid sequence/payload length")
    body = struct.pack("<BBH", msg_id, len(payload), sequence) + payload
    return b"\xa5\x5a" + body + struct.pack("<H", crc16(body))


def covariance(values, dimensions):
    if len(values) != 9 or not all(math.isfinite(x) for x in values):
        raise ValueError("covariance must be finite row-major 3x3")
    if dimensions not in (2, 3):
        raise ValueError("only two or three observed dimensions")
    lower = [[0.0] * dimensions for _ in range(dimensions)]
    for i in range(dimensions):
        for j in range(i + 1):
            a, b = values[i * 3 + j], values[j * 3 + i]
            if not math.isclose(a, b, abs_tol=1e-12, rel_tol=1e-9):
                raise ValueError("covariance must be symmetric")
            value = (a + b) / 2 - sum(lower[i][k] * lower[j][k] for k in range(j))
            if i == j:
                if value <= 0:
                    raise ValueError("covariance must be positive definite")
                lower[i][j] = math.sqrt(value)
            else:
                lower[i][j] = value / lower[j][j]
    return values


def measurement(row, origin_lat, origin_lon, sequence):
    if row.get("valid") is not True or row.get("type") not in ("image_fix", "vo"):
        return None
    stamp = row["t_us"]
    if not isinstance(stamp, int) or isinstance(stamp, bool) or not 0 < stamp < 1 << 64:
        raise ValueError("exposure timestamp must be an integer MCU microsecond time")
    dimensions = row["dimensions"]
    noise = covariance(row["covariance"], dimensions)
    if row["type"] == "vo":
        if row.get("frame") != "NED" or row.get("covariance_frame") != "NED" or dimensions != 2:
            raise ValueError("planar VO must be two-dimensional NED with NED covariance")
        velocity = row["velocity_ned"]
        if len(velocity) != 3 or not all(math.isfinite(x) for x in velocity):
            raise ValueError("invalid velocity")
        # 第三分量只是数组占位，dimensions=2 禁止滤波器把它当垂直速度量测。
        return frame(0x10, struct.pack("<Q3d9dBBB", stamp, *velocity, *noise, 2, 2, 1), sequence)
    if dimensions != 2 or row.get("frame") != "local_NED":
        raise ValueError("image fix must be two-dimensional local NED")
    north, east, height = row["north_m"], row["east_m"], row["height_m"]
    if not all(math.isfinite(x) for x in (north, east, height, origin_lat, origin_lon)) or not -89.5 < origin_lat < 89.5 or not -180 <= origin_lon <= 180 or not -1000 < height < 100000 or math.hypot(north, east) > 50000:
        raise ValueError("invalid local georeference; use a tile-local origin within 50km")
    lat = math.radians(origin_lat)
    d = 1 - .0066943799901413165 * math.sin(lat)**2
    meridian = 6378137 * (1-.0066943799901413165) / d**1.5
    prime = 6378137 / math.sqrt(d)
    latitude = lat + north / (meridian + height)
    longitude = math.remainder(math.radians(origin_lon) + east / ((prime + height) * math.cos(latitude)), 2 * math.pi)
    if abs(latitude) >= math.radians(89.5):
        raise ValueError("local conversion leaves supported latitude range")
    return frame(0x11, struct.pack("<Q3d9dBB", stamp, math.degrees(latitude), math.degrees(longitude), height, *noise, 2, 2), sequence)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, help="JSONL file; omit to consume stdin")
    parser.add_argument("--origin-lat", type=float, required=True)
    parser.add_argument("--origin-lon", type=float, required=True)
    destination = parser.add_mutually_exclusive_group(required=True)
    destination.add_argument("--udp", help="explicit host:port")
    destination.add_argument("--output", type=Path, help="write protocol capture without transmitting")
    args = parser.parse_args()
    incoming = args.input.open(encoding="utf-8") if args.input else sys.stdin
    output = args.output.open("wb") if args.output else None
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.udp else None
    address = None
    if args.udp:
        host, port = args.udp.rsplit(":", 1)
        address = host, int(port)
    sequence = sent = rejected = 0
    try:
        for line in incoming:
            if not line.strip():
                continue
            try:
                row = json.loads(line)
                if not isinstance(row, dict):
                    raise ValueError("JSONL must contain objects")
                packet = measurement(row, args.origin_lat, args.origin_lon, sequence)
            except (ValueError, KeyError, TypeError, OverflowError) as error:
                rejected += 1
                print(f"measurement rejected: {error}", file=sys.stderr)
                continue
            if packet is None:
                continue
            if output:
                output.write(packet)
            else:
                sock.sendto(packet, address)
            sequence = (sequence + 1) & 65535
            sent += 1
    finally:
        if incoming is not sys.stdin:
            incoming.close()
        if output:
            output.close()
        if sock:
            sock.close()
    print(f"companion transport: sent {sent}, rejected {rejected}", file=sys.stderr)
    return 0 if sent and not rejected else 2


if __name__ == "__main__":
    raise SystemExit(main())
