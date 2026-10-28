"""Cost per frame of the tracker and the heading calibrator (doc 03 section 9: "Pi 5 cost unmeasured").

Times Tracker.step and HeadingCalibrator.step inside the stage 7 cued + calibrated trial, one process, on THIS
machine (the laptop, i7-12650H). No Pi 5 was available: the Pi 5 figure is not measured here. Each drone runs one
tracker step and one calibrator step per frame (10 Hz). Run with nothing else heavy on the CPU.

Run: python tools/cost_checks.py   (~3 min)
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from d2d.cue import HeadingCalibrator  # noqa: E402
from d2d.estimate import IMMFilter  # noqa: E402
from d2d.track import Tracker  # noqa: E402
from stage4_checks import run  # noqa: E402

T = {}


def timed(cls, name):
    f = getattr(cls, name)

    def wrap(*a, **k):
        t0 = time.perf_counter()
        out = f(*a, **k)
        T.setdefault(f"{cls.__name__}.{name}", []).append(time.perf_counter() - t0)
        return out
    setattr(cls, name, wrap)


def main(seeds=3):
    timed(Tracker, "step")
    timed(HeadingCalibrator, "step")
    print(f"[cost] stage 7 cued + calibrated, five observers, 3840 px, {seeds} seeds x 60 s, one process on this laptop;")
    print("  per call, ms: median [99th percentile]; the tracker takes every camera's sightings each step")
    for name, extra in (("constant-velocity tracks", {}), ("IMM tracks", dict(make_filter=IMMFilter))):
        T.clear()
        t0 = time.perf_counter()
        for s in range(seeds):
            run(s, mates=True, cue="calibrate", assumed_bias_deg=2.0, tracker_kw=dict(confirm_cams=2, **extra))
        wall = time.perf_counter() - t0
        print(f"  {name}: whole trial {wall / seeds:5.1f} s per seed (600 steps, 5 cameras, scoring included)")
        for k, v in T.items():
            v = np.array(v) * 1e3
            print(f"    {k:26s} {np.median(v):6.2f} [{np.percentile(v, 99):6.2f}] ms over {len(v)} calls", flush=True)


if __name__ == "__main__":
    main()
