"""Checks for d2d.estimate.IMMFilter. Run: python -m unittest discover -s tests"""
import unittest

import numpy as np

from d2d.estimate import IMMFilter, TargetFilter

O1, O2 = np.array([-40.0, 0, 60]), np.array([0.0, -40, 60])


def unit(v):
    return np.asarray(v, float) / np.linalg.norm(v)


def straight_then_turn(t):
    """6 m/s east for 10 s, then a 0.8 rad/s left turn for 3 s (4.8 m/s^2), then straight again."""
    p0, v, w = np.array([-20.0, -5, 50]), 6.0, 0.8
    if t <= 10:
        return p0 + [v * t, 0, 0]
    c = p0 + [v * 10, v / w, 0]  # turn centre
    a = w * min(t - 10, 3.0)
    p = c + v / w * np.array([np.sin(a), -np.cos(a), 0])
    if t > 13:
        p = p + v * (t - 13) * np.array([np.cos(a), np.sin(a), 0])
    return p


def fly(make, T=16.0, dt=0.1, seed=0, sigma=2e-3):
    """Bearings from two observers, noise sigma; returns times, errors and the filter's model probabilities."""
    rng = np.random.default_rng(seed)
    p = straight_then_turn(0.0)
    f = make(O1, unit(p - O1))
    ts, err, mu = [], [], []
    for k in range(1, int(T / dt)):
        t = k * dt
        p = straight_then_turn(t)
        f.predict(dt, accel_sigma=getattr(f, "_a", 1.0))
        for o in (O1, O2):
            f.update_bearing(o, unit(unit(p - o) + rng.normal(0, sigma, 3)), sigma, gate=np.inf)
        ts.append(t)
        err.append(np.linalg.norm(f.position - p))
        mu.append(getattr(f, "mu", np.array([np.nan])).copy())
    return np.array(ts), np.array(err), np.array(mu)


def cv(a):
    def make(o, g):
        f = TargetFilter(o, g, range_guess=50.0, range_sigma=20.0)
        f._a = a
        return f
    return make


def imm(**kw):
    return lambda o, g: IMMFilter(o, g, range_guess=50.0, range_sigma=20.0, **kw)


def rms(e, ts, t0, t1):
    return np.sqrt(np.mean(e[(ts >= t0) & (ts < t1)] ** 2))


class IMM(unittest.TestCase):
    def test_identical_models_reduce_to_one_filter(self):
        ts, e_cv, _ = fly(cv(2.0))
        _, e_imm, _ = fly(imm(accels=(2.0, 2.0)))
        np.testing.assert_allclose(e_imm, e_cv, atol=1e-9)

    def test_model_probability_follows_the_manoeuvre(self):
        ts, _, mu = fly(imm())
        self.assertGreater(mu[(ts > 8) & (ts < 10), 0].mean(), 0.8)  # straight: the quiet model
        self.assertGreater(mu[(ts > 10.5) & (ts < 13), 1].max(), 0.5)  # turning: the manoeuvre model takes over

    def test_better_than_either_single_model_where_it_matters(self):
        e = {}
        for name, make in (("quiet", cv(0.5)), ("loose", cv(5.0)), ("imm", imm())):
            errs = [fly(make, seed=s) for s in range(5)]
            ts = errs[0][0]
            e[name] = (np.mean([rms(x[1], ts, 6, 10) for x in errs]), np.mean([rms(x[1], ts, 10, 14) for x in errs]))
        self.assertLess(e["imm"][0], 0.8 * e["loose"][0])  # straight flight: less noise than the loose model
        self.assertLess(e["imm"][1], 0.8 * e["quiet"][1])  # the turn: less lag than the quiet model

    def test_mixing_brings_the_quiet_model_back_after_a_turn(self):
        """Without mixing, the quiet model is left ~1.8 m behind after the turn while the output leans on the other
        model; with mixing it restarts from the combined estimate (0.2-0.3 m)."""
        rng = np.random.default_rng(0)
        p = straight_then_turn(0.0)
        f = IMMFilter(O1, unit(p - O1), range_guess=50.0, range_sigma=20.0)
        worst = 0.0
        for k in range(1, 160):
            t = k * 0.1
            p = straight_then_turn(t)
            f.predict(0.1)
            for o in (O1, O2):
                f.update_bearing(o, unit(unit(p - o) + rng.normal(0, 2e-3, 3)), 2e-3, gate=np.inf)
            if t > 13.5:
                worst = max(worst, np.linalg.norm(f.models[0].position - p))
        self.assertLess(worst, 1.0)


if __name__ == "__main__":
    unittest.main()
