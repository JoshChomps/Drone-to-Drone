import unittest

import numpy as np

from d2d.cue import LABEL_AMBIGUOUS, LABEL_NONE, HeadingCalibrator
from d2d.geometry import Camera, attitude

DEG = np.pi / 180
CAM = Camera(3840, 2160, 90.0, 15.0)


def boxes_for(o, r_true, pts):
    return [CAM.box(o, r_true, p) for p in pts]


class TestCue(unittest.TestCase):
    def test_correct_undoes_the_bias(self):
        cal = HeadingCalibrator()
        cal.b = 2.0 * DEG
        np.testing.assert_allclose(cal.correct(attitude(0.3 + 2.0 * DEG, 0.02, -0.01)), attitude(0.3, 0.02, -0.01), atol=1e-12)

    def test_converges_to_the_bias(self):
        """Noise-free: teammates seen with a camera whose reported heading is 2 deg off."""
        o, yaw, bias = np.zeros(3), 0.4, 2.0 * DEG
        mates = np.array([[90.0, 50.0, 2.0], [80.0, 10.0, -3.0], [100.0, 70.0, 0.0]])
        cal = HeadingCalibrator(floor_deg=0.0)
        for _ in range(50):
            labels, _ = cal.step(0.1, CAM, o, attitude(yaw + bias), boxes_for(o, attitude(yaw), mates), mates)
        np.testing.assert_array_equal(labels, [0, 1, 2])
        self.assertAlmostEqual(cal.b / DEG, 2.0, delta=0.05)
        self.assertLess(cal.sigma / DEG, 0.2)

    def test_keeps_labelling_after_a_large_bias_is_learned(self):
        """6 deg: once learned, the cue must be projected with the corrected attitude or the teammate falls out of the
        narrowed gate (a 2 deg bias stays inside the gate's allowance for position error, so it cannot show this)."""
        o, yaw, bias = np.zeros(3), 0.4, 6.0 * DEG
        mates = np.array([[90.0, 50.0, 2.0], [80.0, 10.0, -3.0], [100.0, 70.0, 0.0]])
        cal = HeadingCalibrator(floor_deg=0.0)
        for _ in range(50):
            labels, _ = cal.step(0.1, CAM, o, attitude(yaw + bias), boxes_for(o, attitude(yaw), mates), mates)
        np.testing.assert_array_equal(labels, [0, 1, 2])
        self.assertAlmostEqual(cal.b / DEG, 6.0, delta=0.05)

    def test_other_drone_is_not_a_teammate(self):
        o, r = np.zeros(3), attitude(0.4)
        mates = np.array([[90.0, 50.0, 2.0]])
        other = np.array([[60.0, -10.0, -5.0]])  # well away from the cue
        labels, n = HeadingCalibrator().match(CAM, o, r, boxes_for(o, r, np.vstack([mates, other])), mates)
        self.assertEqual(n, 1)
        np.testing.assert_array_equal(labels, [0, LABEL_NONE])

    def test_box_size_rejects_a_drone_on_the_same_line(self):
        """A drone exactly on the teammate's line of sight but at half the range has a box twice as wide."""
        o, r = np.zeros(3), attitude(0.4)
        mate = np.array([90.0, 50.0, 2.0])
        labels, _ = HeadingCalibrator().match(CAM, o, r, boxes_for(o, r, [0.5 * mate]), mate[None])
        np.testing.assert_array_equal(labels, [LABEL_NONE])

    def test_cue_just_outside_the_image_still_labels(self):
        """Teammate 10 px inside the right edge, its shared position 1 m further right: the cue lands outside the
        image. Build log section 11: over half the leaked teammate boxes were this case."""
        o, r = np.zeros(3), attitude(0.0)
        mate = o + 60.0 * CAM.bearing(r, [CAM.width - 10.0, CAM.height / 2])[0]
        shared = mate + [0.0, -1.0, 0.0]  # facing +x, right is -y
        self.assertGreater(CAM.project(o, r, shared)[0][0, 0], CAM.width)
        for edge, want in ((0.0, LABEL_NONE), (3.0, 0)):
            cal = HeadingCalibrator(edge=edge)
            labels, n = cal.match(CAM, o, r, boxes_for(o, r, [mate]), shared[None])
            np.testing.assert_array_equal(labels, [want])
            self.assertEqual((n, cal.misses), (0, 0))  # a cue outside the image is neither counted nor missed

    def test_one_box_in_two_cues_is_ambiguous(self):
        o, r = np.zeros(3), attitude(0.4)
        mates = np.array([[90.0, 50.0, 2.0], [90.3, 50.2, 2.1]])  # two cues on top of each other
        labels, _ = HeadingCalibrator().match(CAM, o, r, boxes_for(o, r, mates[:1]), mates)
        np.testing.assert_array_equal(labels, [LABEL_AMBIGUOUS])

    def test_missed_cue_is_counted(self):
        o, r = np.zeros(3), attitude(0.4)
        mates = np.array([[90.0, 50.0, 2.0], [80.0, 10.0, -3.0]])
        cal = HeadingCalibrator()
        cal.match(CAM, o, r, boxes_for(o, r, mates[:1]), mates)
        self.assertEqual((cal.cues, cal.misses), (2, 1))


if __name__ == "__main__":
    unittest.main()
