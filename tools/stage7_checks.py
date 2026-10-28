"""Stage 7 check (03_build_plan.md): teammate cueing and heading calibration in the five-on-five trial.

The stage 4 trial (tools/stage4_checks.py) with the observers now visible in each other's cameras. Each camera projects
its teammates' shared positions (cues), labels the boxes that match them as teammates, and (with "calibrate") learns
its own heading bias from them (d2d/cue.py). Unlabelled boxes go to the tracker as before.

Not modelled: teammates' shared positions arriving late or lost (they are always fresh here, even in the late-link
row, where only sightings are delayed); clock offset; heading bias that drifts during a run.

Run: python tools/stage7_checks.py [main|floor|all] [processes]   (main ~18 min, floor ~6 min with 6-8 processes)
"floor": one change from the cued + calibrated row, the least heading uncertainty declared to the tracker. The filter's
own sigma falls to ~0.05 deg while its real error is ~0.34 deg (GNSS errors are slow, not white noise).
"""
import os
import sys
import warnings
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from stage4_checks import POOR, run  # noqa: E402
from stage4_detector_checks import REAL  # noqa: E402

M = dict(mates=True)
CONFIGS = [
    ("teammates out of view (stage 4)", dict()),
    ("teammates out of view, headings 0.5 deg", dict(yaw_bias_deg=0.5)),
    ("teammates in view, no cueing", dict(M)),
    ("in view, cued, bias declared (2 deg)", dict(M, cue="fixed")),
    ("in view, cued + calibrated", dict(M, cue="calibrate")),
    ("cued + calibrated, bias 5 deg", dict(M, cue="calibrate", yaw_bias_deg=5.0)),
    ("cued, bias declared, realistic detector", dict(M, **REAL, cue="fixed")),
    ("cued + calibrated, realistic detector", dict(M, **REAL, cue="calibrate")),
    ("cued + calibrated, link 0.5 s late + bursty", dict(M, cue="calibrate", delay=0.5, bursty=True)),
    ("cued + calibrated, 3 observers", dict(M, cue="calibrate", n_obs=3)),
    ("cued + calibrated, 1280 px camera", dict(M, cue="calibrate", cam=POOR)),
]


def _job(args):
    seed, kw = args
    return run(seed, **kw)


FLOORS = [(f"cued + calibrated, declared floor {fl:4.2f} deg", dict(M, cue="calibrate", cal_kw=dict(floor_deg=fl)))
          for fl in (0.0, 0.25, 0.5, 1.0)]


def main(configs=CONFIGS, seeds=16, procs=8):
    print(f"[stage 7] five observers, five other drones; 3840 px, heading bias 2 deg, size unknown, 0.02 false alarms per "
          f"frame unless stated; {seeds} seeds x 60 s; means; count ok with a 95 % interval over seeds")
    print(f"  {'configuration':44s} found count ok        err m  swaps  false | heading deg   mate ok leak | other "
          f"held taken | tiles/frame (full scan 24; 1280 px: 4)", flush=True)
    with Pool(procs) as pool:
        for name, kw in configs:
            kw = dict(kw)
            kw.setdefault("assumed_bias_deg", kw.get("yaw_bias_deg", 2.0))
            kw["tracker_kw"] = {"confirm_cams": 2, **kw.get("tracker_kw", {})}
            r = np.array(pool.map(_job, [(s, kw) for s in range(seeds)]), float)
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)  # all-nan columns
                m = np.nanmean(r, 0)
            c = r[:, 1] * 100
            half = 1.96 * c.std(ddof=1) / np.sqrt(seeds)
            f = lambda x, fmt: "  -  " if np.isnan(x) else format(x, fmt)  # noqa: E731
            print(f"  {name:44s} {m[0] * 100:3.0f} %  {c.mean():3.0f} % [{c.mean() - half:3.0f}-{c.mean() + half:3.0f}]  "
                  f"{m[2]:4.2f}  {m[3]:4.2f}  {m[4]:4.2f} | {f(m[5], '4.2f')} -> {f(m[6], '4.2f')} | "
                  f"{f(m[7] * 100, '4.1f')} {f(m[8] * 100, '4.1f')} | {f(m[9] * 100, '4.1f')} {f(m[10] * 100, '4.1f')} | "
                  f"{f(m[11], '4.1f')}", flush=True)


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "main"
    procs = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    main({"main": CONFIGS, "floor": FLOORS, "all": CONFIGS + FLOORS}[which], procs=procs)
