"""Hand-calculated checks for d2d.geometry. Run: python -m unittest discover -s tests"""
import unittest

import numpy as np

from d2d.geometry import Camera, attitude, lla_to_enu, px4_attitude

CAM = Camera()  # 1280 x 720, 90 deg: f = 640 px
P0, EAST = np.zeros(3), attitude(0.0)  # at the origin, nose pointing east


class Frames(unittest.TestCase):
    def test_enu_axes_and_curvature(self):
        ref = (50.0, -120.0, 400.0)
        north = lla_to_enu(50.0 + 1000 / 111229.0, -120.0, 400.0, ref)  # ~1 km north (111.229 km per degree at 50 N)
        self.assertAlmostEqual(north[1], 1000.0, delta=0.5)
        self.assertAlmostEqual(north[0], 0.0, delta=1e-6)
        self.assertAlmostEqual(north[2], -1000.0 ** 2 / (2 * 6.37e6), delta=0.005)  # the earth curves away: -7.8 cm
        self.assertAlmostEqual(lla_to_enu(50.0, -120.0, 410.0, ref)[2], 10.0, places=6)
        self.assertGreater(lla_to_enu(50.0, -119.99, 400.0, ref)[0], 700.0)  # east is +x

    def test_attitude_conventions(self):
        np.testing.assert_allclose(attitude(np.pi / 2) @ [1, 0, 0], [0, 1, 0], atol=1e-12)  # yaw 90: nose north
        self.assertGreater((attitude(0, pitch=0.2) @ [1, 0, 0])[2], 0)  # pitch up lifts the nose
        self.assertLess((attitude(0, roll=0.2) @ [0, -1, 0])[2], 0)  # roll right drops the right wing

    def test_px4_quaternion(self):
        np.testing.assert_allclose(px4_attitude([1, 0, 0, 0]) @ [1, 0, 0], [0, 1, 0], atol=1e-12)  # NED identity: nose north
        s = np.sqrt(0.5)
        r = px4_attitude([s, 0, 0, s])  # +90 deg about down: nose east
        np.testing.assert_allclose(r @ [1, 0, 0], [1, 0, 0], atol=1e-12)
        np.testing.assert_allclose(r @ [0, 0, 1], [0, 0, 1], atol=1e-12)  # body up stays up
        np.testing.assert_allclose(r, attitude(0.0), atol=1e-12)


class Projection(unittest.TestCase):
    def test_centre_and_directions(self):
        uv, z = CAM.project(P0, EAST, [[25, 0, 0], [25, 5, 0], [25, 0, 5], [-25, 0, 0]])
        np.testing.assert_allclose(uv[0], [640, 360], atol=1e-9)
        self.assertAlmostEqual(z[0], 25.0)
        self.assertAlmostEqual(uv[1, 0], 640 - 640 * 5 / 25)  # a target to the left (north) appears left
        self.assertAlmostEqual(uv[2, 1], 360 - 640 * 5 / 25)  # a target above appears higher (smaller v)
        self.assertLess(z[3], 0)
        np.testing.assert_array_equal(CAM.in_view(uv, z), [True, True, True, False])

    def test_field_of_view_edge(self):
        uv, z = CAM.project(P0, EAST, [[10, -9.99, 0], [10, -10.01, 0]])  # 45 deg right is the edge of a 90 deg lens
        np.testing.assert_array_equal(CAM.in_view(uv, z), [True, False])

    def test_tilt(self):
        cam = Camera(tilt_down_deg=30)
        uv, _ = cam.project(P0, EAST, [[25, 0, -25 * np.tan(np.radians(30))]])
        np.testing.assert_allclose(uv[0], [640, 360], atol=1e-9)

    def test_bearing_inverts_project(self):
        rng = np.random.default_rng(0)
        r = attitude(0.7, 0.1, -0.2)
        pts = r @ np.array([30.0, 0, 0]) + rng.normal(0, 4, (20, 3)) + [3, -2, 50]
        uv, z = CAM.project([3, -2, 50], r, pts)
        d = pts - [3, -2, 50]
        np.testing.assert_allclose(CAM.bearing(r, uv), d / np.linalg.norm(d, axis=1, keepdims=True), atol=1e-9)


class Boxes(unittest.TestCase):
    def test_pixels_on_target(self):
        for rng_m, px in [(25, 19.7), (50, 9.9), (100, 4.9)]:  # doc 01 section 6, doc 02 section 5.1
            b = CAM.box(P0, EAST, [rng_m, 0, 0])
            self.assertAlmostEqual(b[2] - b[0], px, delta=0.03 * px)
        self.assertIsNone(CAM.box(P0, EAST, [-25, 0, 0]))

    def test_range_from_box(self):
        for p in ([25, 0, 0], [40, 20, 5], [15, -10, -3]):
            self.assertAlmostEqual(CAM.range_from_box(CAM.box(P0, EAST, p)), np.linalg.norm(p), delta=0.03 * np.linalg.norm(p))

    def test_angle_survives_turning_the_camera(self):
        p = np.array([30.0, 0, 0])
        centred, turned = CAM.box(P0, EAST, p), CAM.box(P0, attitude(np.radians(35)), p)
        self.assertGreater(turned[2] - turned[0], 1.2 * (centred[2] - centred[0]))  # wider in pixels near the edge
        self.assertAlmostEqual(CAM.subtended_angle(turned), CAM.subtended_angle(centred), delta=0.02 * CAM.subtended_angle(centred))

    def test_cue_error(self):
        for rng_m, px in [(10, 65.0), (25, 27.9), (100, 12.9)]:  # doc 02 section 5.2
            self.assertAlmostEqual(CAM.cue_sigma_px(rng_m, 1.0, np.radians(1.0)), px, delta=0.1)


if __name__ == "__main__":
    unittest.main()
