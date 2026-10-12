#!/usr/bin/env python3
"""把已去畸变/已正射、已统一坐标的 PGM + 椭球高网格切片；仅用标准库。

本工具不会把任意航拍图片自动变成测绘正射图，也不猜测高程基准。
输入 local north/east 原点必须与导航模块相同；上游负责测绘坐标/大地水准面转换。
"""
import argparse
import datetime
import hashlib
import json
import math
import pathlib
import shutil
import statistics
import sys
import tempfile

MAX_PIXELS = 64 * 1024 * 1024
DATUM = "WGS84_ELLIPSOID"


def _dimensions(width, height):
    if not (2 <= width <= 16384 and 2 <= height <= 16384 and width * height <= MAX_PIXELS):
        raise ValueError("raster dimensions must be 2..16384 and at most 64M samples")


def read_pgm(path):
    """读 P2/P5，支持 8/16bit maxval，返回归一后的 8bit 灰度。

    P5 maxval 后只消费一个分隔符（或 CRLF），因此首像素可以是空白字节。
    """
    data = pathlib.Path(path).read_bytes()
    cursor = 0

    def token():
        nonlocal cursor
        while cursor < len(data):
            if data[cursor] in b" \t\r\n\v\f":
                cursor += 1
            elif data[cursor] == 35:
                newline = data.find(b"\n", cursor)
                cursor = len(data) if newline < 0 else newline + 1
            else:
                break
        start = cursor
        while cursor < len(data) and data[cursor] not in b" \t\r\n\v\f#":
            cursor += 1
        if cursor == start:
            raise ValueError("truncated PGM header/sample")
        return data[start:cursor].decode("ascii")

    magic = token()
    if magic not in ("P2", "P5"):
        raise ValueError("only P2/P5 PGM is supported")
    width, height, maximum = int(token()), int(token()), int(token())
    _dimensions(width, height)
    if not 1 <= maximum <= 65535:
        raise ValueError("invalid PGM maxval")
    count = width * height
    if magic == "P5":
        if cursor >= len(data) or data[cursor] not in b" \t\r\n":
            raise ValueError("PGM binary header has no delimiter")
        if data[cursor:cursor + 2] == b"\r\n":
            cursor += 2
        else:
            cursor += 1
        size = count * (2 if maximum > 255 else 1)
        raster = data[cursor:cursor + size]
        if len(raster) != size:
            raise ValueError("truncated PGM raster")
        if data[cursor + size:].strip(b" \t\r\n"):
            raise ValueError("extra PGM raster data")
        raw = [raster[i] * 256 + raster[i + 1] for i in range(0, size, 2)] if maximum > 255 else list(raster)
    else:
        raw = [int(token()) for _ in range(count)]
        try:
            token()
        except ValueError:
            pass
        else:
            raise ValueError("extra PGM text samples")
    if any(value < 0 or value > maximum for value in raw):
        raise ValueError("PGM sample exceeds maxval")
    return width, height, [int(math.floor(value * 255 / maximum + .5)) for value in raw]


def write_pgm(path, width, height, values):
    _dimensions(width, height)
    if len(values) != width * height or any(not math.isfinite(v) or v < 0 or v > 255 for v in values):
        raise ValueError("invalid PGM values")
    pathlib.Path(path).write_bytes(f"P5\n{width} {height}\n255\n".encode() + bytes(int(math.floor(v + .5)) for v in values))


def read_heights(path):
    fields = pathlib.Path(path).read_text(encoding="ascii").split()
    if len(fields) < 4 or fields[:2] != ["NAVHEIGHT", "1"]:
        raise ValueError("height file must start with NAVHEIGHT 1 width height")
    width, height = int(fields[2]), int(fields[3])
    _dimensions(width, height)
    if len(fields) != 4 + width * height:
        raise ValueError("height sample count mismatch")
    values = [float(v) for v in fields[4:]]
    if any(not math.isfinite(v) for v in values):
        raise ValueError("nonfinite height samples are forbidden; fill voids before packaging")
    return width, height, values


def write_heights(path, width, height, values):
    _dimensions(width, height)
    if len(values) != width * height or any(not math.isfinite(v) for v in values):
        raise ValueError("invalid height values")
    with pathlib.Path(path).open("w", encoding="ascii", newline="\n") as stream:
        stream.write(f"NAVHEIGHT 1 {width} {height}\n")
        for y in range(height):
            stream.write(" ".join(format(v, ".17g") for v in values[y * width:(y + 1) * width]) + "\n")


def _quality(width, height, image, heights):
    laplacian = [image[y * width + x - 1] + image[y * width + x + 1]
                 + image[(y - 1) * width + x] + image[(y + 1) * width + x] - 4 * image[y * width + x]
                 for y in range(1, height - 1) for x in range(1, width - 1)]
    return statistics.pstdev(image), statistics.pvariance(laplacian) if laplacian else 0, statistics.pstdev(heights)


def _ranges(length, tile_size):
    ranges = []
    start = 0
    while start < length:
        end = min(length, start + tile_size)
        if length - end == 1:
            end = length
        ranges.append((start, end))
        start = end
    return ranges


def _crop(values, stride, x0, y0, x1, y1):
    return [value for y in range(y0, y1) for value in values[y * stride + x0:y * stride + x1]]


def _digest(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def build_reference(image_path, height_path, output, *, origin_lat, origin_lon, north, east,
                    gsd, height_datum, acquired_utc, tile_size=1024, force=False):
    coordinates = [origin_lat, origin_lon, north, east, gsd]
    if any(not math.isfinite(v) for v in coordinates) or not (-90 <= origin_lat <= 90 and -180 <= origin_lon <= 180):
        raise ValueError("invalid WGS84 origin/bounds")
    if not 0 < gsd <= 10000 or not 2 <= tile_size <= 16384:
        raise ValueError("invalid GSD or tile size")
    if height_datum != DATUM:
        raise ValueError("height datum must be WGS84_ELLIPSOID; convert geoid/AMSL heights upstream")
    try:
        parsed = datetime.datetime.strptime(acquired_utc, "%Y-%m-%dT%H:%M:%SZ")
    except ValueError as error:
        raise ValueError("acquired UTC must be YYYY-MM-DDTHH:MM:SSZ") from error
    if parsed.year < 1970 or parsed.strftime("%Y-%m-%dT%H:%M:%SZ") != acquired_utc:
        raise ValueError("invalid acquisition UTC")
    width, height, image = read_pgm(image_path)
    hw, hh, heights = read_heights(height_path)
    if (hw, hh) != (width, height):
        raise ValueError("DOM/DSM dimensions must agree on the same grid")
    nmin, emax = north - (height - 1) * gsd, east + (width - 1) * gsd
    if not math.isfinite(nmin) or not math.isfinite(emax) or nmin >= north or emax <= east:
        raise ValueError("invalid raster bounds")
    output = pathlib.Path(output)
    if output.exists() and any(output.iterdir()) and not force:
        raise ValueError("output directory is nonempty; use --force to replace the reference package")
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = pathlib.Path(tempfile.mkdtemp(prefix=".navref-", dir=output.parent))
    try:
        entries = []
        for row, (y0, y1) in enumerate(_ranges(height, tile_size)):
            for column, (x0, x1) in enumerate(_ranges(width, tile_size)):
                tile_id = f"tile_r{row:04d}_c{column:04d}"
                tile_width, tile_height = x1 - x0, y1 - y0
                tile_image = _crop(image, width, x0, y0, x1, y1)
                tile_heights = _crop(heights, width, x0, y0, x1, y1)
                image_file, height_file = tile_id + ".pgm", tile_id + ".height"
                write_pgm(staging / image_file, tile_width, tile_height, tile_image)
                write_heights(staging / height_file, tile_width, tile_height, tile_heights)
                std, lap, hstd = _quality(tile_width, tile_height, tile_image, tile_heights)
                tile_north, tile_east = north - y0 * gsd, east + x0 * gsd
                metadata = {
                    "format": "NAVTILE_1", "id": tile_id, "image_file": image_file, "height_file": height_file,
                    "image_sha256": _digest(staging / image_file), "height_sha256": _digest(staging / height_file),
                    "width": tile_width, "height": tile_height, "origin_north_m": tile_north, "origin_east_m": tile_east,
                    "gsd_m": gsd, "north_min_m": tile_north - (tile_height - 1) * gsd, "north_max_m": tile_north,
                    "east_min_m": tile_east, "east_max_m": tile_east + (tile_width - 1) * gsd,
                    "origin_lat_deg": origin_lat, "origin_lon_deg": origin_lon, "height_datum": height_datum,
                    "acquired_utc": acquired_utc, "texture_std": std, "laplacian_variance": lap, "height_std_m": hstd,
                }
                metadata_file = tile_id + ".meta"
                text = "".join(key + "=" + (format(value, ".17g") if isinstance(value, float) else str(value)) + "\n" for key, value in metadata.items())
                (staging / metadata_file).write_text(text, encoding="ascii", newline="\n")
                entries.append(f"{tile_id} {metadata_file} {_digest(staging / metadata_file)}\n")
        index = "NAVREF 1\n" + "".join(entries)
        (staging / "index.tsv").write_text(index, encoding="ascii", newline="\n")
        (staging / "index.sha256").write_text(_digest(staging / "index.tsv") + "\n", encoding="ascii", newline="\n")
        if output.exists():
            shutil.rmtree(output)
        staging.rename(output)
        return {"tiles": len(entries), "width": width, "height": height, "height_datum": height_datum,
                "index_sha256": _digest(output / "index.tsv")}
    finally:
        if staging.exists():
            shutil.rmtree(staging)


def create_demo(output, force=False):
    output = pathlib.Path(output)
    if output.exists() and any(output.iterdir()) and not force:
        raise ValueError("demo output directory is nonempty; use --force to replace it")
    if output.exists() and force:
        shutil.rmtree(output)
    (output / "frames").mkdir(parents=True, exist_ok=True)
    width, height = 256, 192
    image, heights = [], []
    for y in range(height):
        for x in range(width):
            value = ((x * 73856093) & 0xffffffff) ^ ((y * 19349663) & 0xffffffff) ^ 92837111
            value ^= value >> 13
            value = (value * 1274126177) & 0xffffffff
            image.append(int(math.floor(40 + 15 * math.sin(x * .27) + 12 * math.cos(y * .31)
                + 8 * math.sin(x * .13 + y * .21) + value % 160 + .5)))
            heights.append(100 + 4 * math.sin(.23 * x) + 7 * math.cos(.14 * y) + 3 * math.sin(.15 * x + .3 * y))
    write_pgm(output / "source.pgm", width, height, image)
    write_heights(output / "source.height", width, height, heights)
    summary = build_reference(output / "source.pgm", output / "source.height", output / "reference",
        origin_lat=30., origin_lon=120., north=200., east=-100., gsd=1., height_datum=DATUM,
        acquired_utc="2026-01-01T00:00:00Z", tile_size=256)
    manifest = ["NAVREPLAY 1", "reference=reference", "fx=64", "fy=64", "cx=31.5", "cy=31.5",
        "ortho_gsd_m=1", "ortho_width=64", "ortho_height=64", "search_radius_m=12",
        "terrain_observed=observed.height", "terrain_north_m=141", "terrain_east_m=-14", "terrain_gsd_m=1",
        "terrain_search_radius_m=12", "height_datum=WGS84_ELLIPSOID", "FRAMES",
        "image,t_us,north_m,east_m,height_m,plane_height_m,roll_rad,pitch_rad,yaw_rad,altitude_sigma_m"]
    for i, (x0, y0) in enumerate([(72, 50), (77, 48), (82, 46)]):
        filename = f"frames/frame{i:03d}.pgm"
        write_pgm(output / filename, 64, 64, _crop(image, width, x0, y0, x0 + 64, y0 + 64))
        north, east = 200 - (y0 + 31.5), -100 + (x0 + 31.5)
        manifest.append(f"{filename},{1000200 + i * 100000},{north + 3},{east - 4},164,100,0,0,0,0.2")
    observed = [h + 7.5 for h in _crop(heights, width, 90, 65, 114, 85)]
    write_heights(output / "observed.height", 24, 20, observed)
    (output / "frames.manifest").write_text("\n".join(manifest) + "\n", encoding="ascii", newline="\n")
    summary["frames"] = 3
    summary["manifest"] = str(output / "frames.manifest")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    build = commands.add_parser("build", help="package already georeferenced DOM/DSM inputs")
    build.add_argument("--image", required=True, type=pathlib.Path)
    build.add_argument("--height", required=True, type=pathlib.Path)
    build.add_argument("--output", required=True, type=pathlib.Path)
    build.add_argument("--origin-lat", required=True, type=float)
    build.add_argument("--origin-lon", required=True, type=float)
    build.add_argument("--north", required=True, type=float, help="north coordinate of top-left pixel center")
    build.add_argument("--east", required=True, type=float, help="east coordinate of top-left pixel center")
    build.add_argument("--gsd", required=True, type=float)
    build.add_argument("--tile-size", type=int, default=1024)
    build.add_argument("--height-datum", required=True)
    build.add_argument("--acquired-utc", required=True)
    build.add_argument("--force", action="store_true")
    demo = commands.add_parser("demo", help="create deterministic real PGM/DSM replay fixtures")
    demo.add_argument("--output", type=pathlib.Path, required=True)
    demo.add_argument("--force", action="store_true")
    args = parser.parse_args(argv)
    try:
        if args.command == "demo":
            summary = create_demo(args.output, args.force)
        else:
            summary = build_reference(args.image, args.height, args.output, origin_lat=args.origin_lat,
                origin_lon=args.origin_lon, north=args.north, east=args.east, gsd=args.gsd,
                tile_size=args.tile_size, height_datum=args.height_datum, acquired_utc=args.acquired_utc, force=args.force)
        print(json.dumps(summary, sort_keys=True))
        return 0
    except (OSError, ValueError, UnicodeError) as error:
        print("reference build error: " + str(error), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
