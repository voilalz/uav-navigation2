import struct
import unittest
from integration.companion_sender import measurement
from integration.nav_receiver import crc16


class SenderTests(unittest.TestCase):
    def test_planar_velocity_keeps_frame_covariance_and_exposure(self):
        row = dict(type="vo", valid=True, t_us=1234500, dimensions=2, frame="NED",
                   covariance_frame="NED", velocity_ned=[20, 50, 0], covariance=[1, .1, 0, .1, 2, 0, 0, 0, 0])
        wire = measurement(row, 30, 120, 65535)
        self.assertEqual((wire[2], wire[3], len(wire)), (0x10, 107, 115))
        self.assertEqual(struct.unpack_from("<H", wire, 4)[0], 65535)
        self.assertEqual(struct.unpack_from("<Q", wire, 6)[0], 1234500)
        self.assertEqual(wire[-5:-2], bytes([2, 2, 1]))
        self.assertEqual(struct.unpack_from("<H", wire, len(wire)-2)[0], crc16(wire[2:-2]))

    def test_image_fix_uses_supplied_georeference(self):
        row = dict(type="image_fix", valid=True, t_us=1, frame="local_NED", dimensions=2,
                   north_m=0, east_m=0, height_m=1000, covariance=[4, 0, 0, 0, 4, 0, 0, 0, 0])
        wire = measurement(row, 30, 120, 0)
        values = struct.unpack("<Q3d9dBB", wire[6:-2])
        self.assertAlmostEqual(values[1], 30)
        self.assertAlmostEqual(values[2], 120)
        self.assertEqual(values[-2:], (2, 2))

    def test_invalid_or_unavailable_match_produces_no_wire_fix(self):
        self.assertIsNone(measurement(dict(type="image_fix", valid=False), 30, 120, 0))
        self.assertIsNone(measurement(dict(type="terrain_registration", valid=True), 30, 120, 0))

    def test_bad_covariance_and_body_assumption_are_rejected(self):
        row = dict(type="vo", valid=True, t_us=1, dimensions=2, frame="BODY",
                   covariance_frame="NED", velocity_ned=[0, 0, 0], covariance=[1, 0, 0, 0, 1, 0, 0, 0, 0])
        with self.assertRaises(ValueError):
            measurement(row, 30, 120, 0)
        row["frame"] = "NED"
        row["covariance"][0] = -1
        with self.assertRaises(ValueError):
            measurement(row, 30, 120, 0)


if __name__ == "__main__":
    unittest.main()
