"""Checks for the look scheduler in tools/stage4_checks.py. Run: python -m unittest discover -s tests"""
import unittest

import numpy as np

from d2d.geometry import Camera, attitude
from tools.stage4_checks import _cells, _schedule

CAM = Camera(3840, 2160, 90.0, 15.0)  # 6 x 4 tiles of 640 px
O, R = np.zeros(3), attitude(0.0)


def at(u, v, r=60.0):
    return O + r * CAM.bearing(R, [u, v])[0]


class Looks(unittest.TestCase):
    def test_windows_come_first_and_the_budget_holds(self):
        wins = [(at(100, 100), 20), (at(3700, 2000), 20)]  # one tile each, opposite corners
        hit, _, used = _schedule(CAM, O, R, wins, 1, 0)
        self.assertEqual((used, hit.sum()), (1, 1))
        self.assertTrue(hit[0, 0])  # the first window, not the second, not the sweep

    def test_a_window_that_does_not_fit_is_skipped_for_one_that_does(self):
        big, small = (at(1920, 1080), 700), (at(3700, 2000), 20)  # the big one spans several tiles
        self.assertGreater(len(_cells(CAM, O, R, *big)), 2)
        hit, _, used = _schedule(CAM, O, R, [big, small], 2, 0)
        self.assertTrue(hit[3, 5])
        self.assertEqual(used, 2)  # the small window plus one sweep tile

    def test_sweep_covers_the_frame_round_robin(self):
        seen, ptr = np.zeros((4, 6), int), 0
        for _ in range(8):  # 3 tiles a frame: the whole frame in 8 frames, each tile once
            hit, ptr, used = _schedule(CAM, O, R, [], 3, ptr)
            self.assertEqual(used, 3)
            seen += hit
        np.testing.assert_array_equal(seen, np.ones((4, 6)))


if __name__ == "__main__":
    unittest.main()
