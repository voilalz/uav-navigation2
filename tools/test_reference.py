#!/usr/bin/env python3
"""参考库生产的真实文件行为回归；仅依赖 Python 标准库。"""
import hashlib
import importlib.util
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest

SCRIPT = pathlib.Path(__file__).with_name("build_reference.py")
spec = importlib.util.spec_from_file_location("build_reference", SCRIPT)
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)
COMPANION = None
if "--companion" in sys.argv:
    position = sys.argv.index("--companion")
    COMPANION = sys.argv[position + 1]
    del sys.argv[position:position + 2]


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="uav-reference-")
        self.root = pathlib.Path(self.tmp.name)
        self.image = self.root / "input.pgm"
        self.height = self.root / "input.height"
        pixels = bytes((x * 31 + y * 53 + x * y * 7) % 256 for y in range(14) for x in range(18))
        self.image.write_bytes(b"P5\n18 14\n255\n" + pixels)
        self.height.write_text("NAVHEIGHT 1 18 14\n" + " ".join(str(100 + x * .1 + y * .2) for y in range(14) for x in range(18)) + "\n")

    def tearDown(self):
        self.tmp.cleanup()

    def build(self, *extra):
        return subprocess.run([sys.executable, str(SCRIPT), "build", "--image", str(self.image),
            "--height", str(self.height), "--output", str(self.root / "reference"),
            "--origin-lat", "30", "--origin-lon", "120", "--north", "500", "--east", "-150",
            "--gsd", "2", "--tile-size", "10", "--height-datum", "WGS84_ELLIPSOID",
            "--acquired-utc", "2026-01-01T00:00:00Z", *extra], capture_output=True, text=True)

    def test_real_tiles_bounds_metadata_and_hashes(self):
        result = self.build()
        self.assertEqual(result.returncode, 0, result.stderr)
        root = self.root / "reference"
        index = (root / "index.tsv").read_bytes()
        self.assertEqual(hashlib.sha256(index).hexdigest(), (root / "index.sha256").read_text().strip())
        entries = index.decode().splitlines()[1:]
        self.assertEqual(len(entries), 4)
        for entry in entries:
            tile_id, filename, digest = entry.split()
            raw = (root / filename).read_bytes()
            self.assertEqual(hashlib.sha256(raw).hexdigest(), digest)
            meta = dict(line.split("=", 1) for line in raw.decode().splitlines())
            self.assertEqual(meta["id"], tile_id)
            self.assertEqual(meta["height_datum"], "WGS84_ELLIPSOID")
            self.assertGreater(float(meta["texture_std"]), 1)
            self.assertGreater(float(meta["laplacian_variance"]), 1)
            w, h, _ = reference.read_pgm(root / meta["image_file"])
            self.assertEqual((w, h), (int(meta["width"]), int(meta["height"])))
            self.assertAlmostEqual(float(meta["north_max_m"]) - float(meta["north_min_m"]), (h - 1) * 2)
            self.assertAlmostEqual(float(meta["east_max_m"]) - float(meta["east_min_m"]), (w - 1) * 2)
            for kind in ("image", "height"):
                self.assertEqual(hashlib.sha256((root / meta[kind + "_file"]).read_bytes()).hexdigest(), meta[kind + "_sha256"])

    def test_bounds_and_height_datum_are_required(self):
        for extra in [("--origin-lat", "91"), ("--origin-lon", "181"), ("--gsd", "0"),
                      ("--height-datum", "AMSL"), ("--acquired-utc", "unknown")]:
            with self.subTest(extra=extra):
                result = self.build(*extra)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse((self.root / "reference" / "index.tsv").exists())

    def test_pgm_16bit_and_truncation(self):
        path = self.root / "sixteen.pgm"
        path.write_bytes(b"P5\n2 2\n65535\n" + bytes([0, 0, 255, 255, 128, 128, 1, 1]))
        w, h, pixels = reference.read_pgm(path)
        self.assertEqual((w, h), (2, 2))
        self.assertEqual(pixels, [0, 255, 128, 1])
        path.write_bytes(b"P5\n2 2\n255\n\x00")
        with self.assertRaises(ValueError):
            reference.read_pgm(path)

    def test_nonfinite_and_wrong_shape_heights_are_rejected(self):
        self.height.write_text("NAVHEIGHT 1 18 14\n" + "nan " * (18 * 14))
        self.assertNotEqual(self.build().returncode, 0)
        self.height.write_text("NAVHEIGHT 1 2 2\n0 1 2 3\n")
        self.assertNotEqual(self.build().returncode, 0)

    def test_demo_has_actual_images_and_exposure_timestamps(self):
        out = self.root / "demo"
        result = subprocess.run([sys.executable, str(SCRIPT), "demo", "--output", str(out)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        manifest = (out / "frames.manifest").read_text()
        self.assertIn("NAVREPLAY 1", manifest)
        frames = manifest.split("FRAMES\n", 1)[1].splitlines()[1:]
        self.assertEqual(len(frames), 3)
        times = []
        for row in frames:
            fields = row.split(",")
            self.assertTrue((out / fields[0]).is_file())
            w, h, data = reference.read_pgm(out / fields[0])
            self.assertEqual((w, h), (64, 64))
            self.assertGreater(max(data) - min(data), 100)
            times.append(int(fields[1]))
        self.assertEqual(times, [1000200, 1100200, 1200200])

    @unittest.skipUnless(COMPANION, "provide --companion to test C++ replay integration")
    def test_replay_reports_actual_matches_vo_and_terrain(self):
        out = self.root / "demo"
        reference.create_demo(out)
        result = subprocess.run([COMPANION, "--replay", str(out / "frames.manifest")], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        rows = [json.loads(line) for line in result.stdout.splitlines()]
        fixes = [row for row in rows if row["type"] == "image_fix"]
        self.assertEqual(len(fixes), 3)
        self.assertEqual([row["t_us"] for row in fixes], [1000200, 1100200, 1200200])
        for i, row in enumerate(fixes):
            self.assertTrue(row["valid"], row)
            self.assertAlmostEqual(row["north_m"], 118.5 + 2 * i, delta=.2)
            self.assertAlmostEqual(row["east_m"], 3.5 + 5 * i, delta=.2)
            self.assertEqual(row["dimensions"], 2)
        velocities = [row for row in rows if row["type"] == "vo"]
        self.assertEqual(len(velocities), 2)
        for row in velocities:
            self.assertTrue(row["valid"], row)
            self.assertAlmostEqual(row["velocity_ned"][0], 20, delta=1)
            self.assertAlmostEqual(row["velocity_ned"][1], 50, delta=1)
            self.assertEqual(row["covariance_frame"], "NED")
        terrain = [row for row in rows if row["type"] == "terrain_registration"]
        self.assertEqual(len(terrain), 1)
        self.assertTrue(terrain[0]["valid"], terrain[0])
        self.assertAlmostEqual(terrain[0]["delta_north_m"], -6, delta=.1)
        self.assertAlmostEqual(terrain[0]["delta_east_m"], 4, delta=.1)
        self.assertAlmostEqual(terrain[0]["vertical_bias_m"], 7.5, delta=.01)

    @unittest.skipUnless(COMPANION, "provide --companion to test C++ replay integration")
    def test_replay_rejects_nonmonotonic_exposure(self):
        out = self.root / "demo"
        reference.create_demo(out)
        manifest = out / "frames.manifest"
        manifest.write_text(manifest.read_text().replace(",1200200,", ",1100200,"))
        result = subprocess.run([COMPANION, "--replay", str(manifest)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("exposure", result.stderr)

    @unittest.skipUnless(COMPANION, "provide --companion to test C++ replay integration")
    def test_replay_rejects_corrupted_raster(self):
        out = self.root / "demo"
        reference.create_demo(out)
        image = next((out / "reference").glob("*.pgm"))
        raw = image.read_bytes()
        image.write_bytes(raw[:-1] + bytes([raw[-1] ^ 1]))
        result = subprocess.run([COMPANION, "--replay", str(out / "frames.manifest")], capture_output=True, text=True)
        self.assertEqual(result.returncode, 2)
        self.assertIn("checksum", result.stderr)


if __name__ == "__main__":
    unittest.main()
