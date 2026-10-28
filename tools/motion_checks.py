"""Motion model check (doc 03 section 9): is the constant-velocity model with 3 m/s^2 noise, chosen on the stage 3
scenario, also right for a different drone, and does an IMM (d2d.estimate.IMMFilter) do better without tuning?

Stage 3's trial (tools/stage3_checks.py, 3840 px camera) on two drones:
  wander : the stage 3 drone (4 m/s, gentle turns, at most 1.6 m/s^2), on which 3 m/s^2 was chosen
  legs   : a drone it was not tuned on (6 m/s straight legs of 3-8 s, then 1 rad/s turns of 60-150 deg, 6 m/s^2)
Each with two observers orbiting 90 deg apart, headings off by 2 deg (as stage 3) and calibrated to 0.5 deg (where the
motion model's share of the error is larger). The IMM's models (0.5 and 5 m/s^2) and its 5 s sojourn were set before
any run, not tuned.

Run: python tools/motion_checks.py [processes]   (~3 min with 8 processes)
"""
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from stage3_checks import CAMERAS, run  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from d2d.estimate import IMMFilter  # noqa: E402

CAM = CAMERAS["3840 px, 90 deg"]
MODELS = [("CV 1 m/s^2", dict(accel_sigma=1.0)), ("CV 3 m/s^2 (chosen in stage 3)", dict(accel_sigma=3.0)),
          ("CV 6 m/s^2", dict(accel_sigma=6.0)), ("IMM 0.5 + 5 m/s^2", dict(make_filter=IMMFilter))]


def _job(args):
    seed, kw = args
    return run(seed, CAM, ["orbit", "orbit"], **kw)


def main(seeds=40, procs=8):
    print(f"[motion] two observers orbiting 90 deg apart, bearing + box size, size unknown, 3840 px; {seeds} seeds x 40 s;")
    print("  RMS after 10 s, median over seeds [90th percentile]; paired: IMM minus CV 3 per seed, mean +- 95 % half-width")
    with Pool(procs) as pool:
        for motion in ("wander", "legs"):
            for bias in (2.0, 0.5):
                print(f"\n  drone: {motion}; heading bias {bias} deg")
                print(f"  {'model':32s} pos while seen    vel while seen   re-acq", flush=True)
                res = {}
                for name, kw in MODELS:
                    r = np.array(pool.map(_job, [(s, dict(kw, motion=motion, yaw_bias_deg=bias)) for s in range(seeds)]))
                    res[name] = r
                    q = lambda c, p: np.nanquantile(r[:, c], p)  # noqa: E731
                    print(f"  {name:32s} {q(1, .5):5.2f} [{q(1, .9):5.2f}]    {q(2, .5):4.2f} [{q(2, .9):4.2f}]      "
                          f"{r[:, 4].mean():.2f}", flush=True)
                for c, lbl in ((1, "position"), (2, "velocity")):
                    d = res["IMM 0.5 + 5 m/s^2"][:, c] - res["CV 3 m/s^2 (chosen in stage 3)"][:, c]
                    d = d[~np.isnan(d)]
                    print(f"    IMM - CV 3, {lbl}: {d.mean():+.3f} +- {1.96 * d.std(ddof=1) / np.sqrt(len(d)):.3f}", flush=True)


if __name__ == "__main__":
    main(procs=int(sys.argv[1]) if len(sys.argv) > 1 else 8)
