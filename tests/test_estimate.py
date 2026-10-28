"""Checks for d2d.estimate. Run: python -m unittest discover -s tests"""
import unittest

import numpy as np

from d2d.estimate import TargetFilter, angle_model, bearing_model, triangulate


def unit(v):
    return np.asarray(v, float) / np.linalg.norm(v)


def orbit(t, centre=(-35.0, 0.0, 60.0), radius=20.0, speed=5.0):
    w = speed / radius
    return np.array(centre) + radius * np.array([np.cos(w * t), np.sin(w * t), 0.0])


def run(observer, use_angle, size=0.5, known=False, T=40.0, dt=0.1):
    """Noise-free measurements of a constant-velocity target; returns the filter and the true final position."""
    p0, v = np.array([5.0, -10.0, 50.0]), np.array([0.8, 0.5, 0.05])  # stays 35-75 m from the observer
    f = None
    for k in range(int(T / dt)):
        t = k * dt
        p, o = p0 + v * t, observer(t)
        g, theta = unit(p - o), 2 * np.arctan(size / (2 * np.linalg.norm(p - o)))
        if f is None:
            f = TargetFilter(o, g, theta if use_angle else None, *((size, 1e-3) if known else (0.3, 0.25)))
            continue
        f.predict(dt, accel_sigma=0.2)
        f.update_bearing(o, g, 2e-3)
        if use_angle:
            f.update_angle(o, theta, 2e-3)
    return f, p0 + v * (T - dt)


class Models(unittest.TestCase):
    def test_triangulate(self):
        p = np.array([10.0, 20.0, 50.0])
        o = np.array([[0.0, 0, 60], [40, 0, 60], [0, 40, 55]])
        np.testing.assert_allclose(triangulate(o, [unit(p - q) for q in o]), p, atol=1e-9)

    def test_jacobians_match_finite_differences(self):
        rng = np.random.default_rng(0)
        for back in (0.0, 0.7):
            x, o = np.array([20.0, -15, 48, 3, -2, 0.5, 0.6]), np.array([-5.0, 4, 60])
            _, _, E, Hb = bearing_model(x, o, back)
            th, Ha = angle_model(x, o, back)
            for _ in range(5):
                dx = rng.normal(0, 1e-5, 7)
                np.testing.assert_allclose(E @ bearing_model(x + dx, o, back)[0], Hb @ dx, atol=1e-11)
                self.assertAlmostEqual(angle_model(x + dx, o, back)[0] - th, Ha @ dx, delta=1e-12)


class Observability(unittest.TestCase):
    """Ning et al. 2024, Theorem 1: with the box size the observer needs higher-order motion than the target, in any
    direction. Without it the extra motion must be sideways. A hovering observer leaves range and size tied together."""

    def test_orbiting_observer_with_box_size_finds_range_and_size(self):
        f, p = run(orbit, use_angle=True)
        self.assertLess(np.linalg.norm(f.position - p), 1.0)
        self.assertAlmostEqual(f.size, 0.5, delta=0.03)  # started at 0.3

    def test_hovering_observer_cannot_separate_range_from_size(self):
        f, p = run(lambda t: orbit(0.0), use_angle=True)
        self.assertGreater(np.linalg.norm(f.position - p), 10.0)  # stays at the range its size prior implies
        self.assertLess(np.linalg.norm(np.cross(f.position - orbit(0.0), unit(p - orbit(0.0)))), 0.5)  # direction is right

    def test_known_size_gives_range_even_when_hovering(self):
        f, p = run(lambda t: orbit(0.0), use_angle=True, known=True)
        self.assertLess(np.linalg.norm(f.position - p), 1.0)


class LateData(unittest.TestCase):
    def test_late_bearing_is_right_only_at_its_own_time(self):
        p0, v, back = np.array([0.0, 0, 50]), np.array([5.0, 0, 0]), 0.5
        o1, o2 = np.array([-30.0, 0, 60]), np.array([0.0, -30, 60])
        err = {}
        for own_time in (True, False):
            f = TargetFilter(o1, unit(p0 - o1), range_guess=31.6, range_sigma=10.0)
            for k in range(1, 300):
                t = k * 0.1
                f.predict(0.1, accel_sigma=0.2)
                f.update_bearing(o1, unit(p0 + v * t - o1), 2e-3)
                f.update_bearing(o2, unit(p0 + v * (t - back) - o2), 2e-3, back=back if own_time else 0.0, gate=1e9)
            err[own_time] = np.linalg.norm(f.position - (p0 + v * t))
        self.assertLess(err[True], 0.2)
        self.assertGreater(err[False], 5 * err[True])


class Recovery(unittest.TestCase):
    def test_lost_track_is_reacquired_and_keeps_its_size(self):
        o, p = np.array([0.0, 0, 60]), np.array([30.0, 0, 50])
        f = TargetFilter(o, unit(p - o), 2 * np.arctan(0.5 / (2 * np.linalg.norm(p - o))), 0.5, 0.02)
        p = np.array([-10.0, 25, 50])  # it reappears somewhere else entirely
        g, th = unit(p - o), 2 * np.arctan(0.5 / (2 * np.linalg.norm(p - o)))
        for _ in range(5):
            self.assertFalse(f.update_bearing(o, g, 0.01))
        self.assertEqual(f.rejected, 5)
        f.reacquire(o, g, th)
        self.assertEqual(f.rejected, 0)
        self.assertLess(np.linalg.norm(f.position - p), 0.5)
        self.assertAlmostEqual(f.size, 0.5)
        self.assertTrue(f.update_bearing(o, g, 0.01))


if __name__ == "__main__":
    unittest.main()
