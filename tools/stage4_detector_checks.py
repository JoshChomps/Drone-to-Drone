"""Stage 4 follow-up (build log section 9): does the five-on-five tracker survive a realistic detector, and do the rows
doc 03 section 6 could not separate hold up with more seeds?

Detector numbers borrowed from the sibling project ../.Drone Detection (its notes, 2026-10-04). They come from
ground cameras looking at sky (Anti-UAV), not air-to-air, so they are a stress test, not a measurement:
  * background false alarms at 90 % recall: 46-159 per 1000 frames, i.e. 0.05-0.16 per frame (stage 4 assumed 0.02);
  * recall under blur + noise + JPEG with its adopted recipe: ~0.32 at 8.5 px, 0.75 at 14 px, 0.85 at 34 px,
    fitted here by pd_max 0.85, half at 10 px (stage 4 assumed 0.95, half at 8 px);
  * box width +25 % on drones under 8 px.

Run: python tools/stage4_detector_checks.py [detector|seeds|all] [processes]   (~10 min with 8 processes)
"""
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from stage4_checks import GOOD, POOR, run  # noqa: E402

REAL = dict(clutter=0.1, pd_max=0.85, w50=10.0, width_bias=0.25)  # the middle of the sibling's false-alarm range

DETECTOR = [
    ("baseline (doc 03 section 6)", dict()),
    ("false alarms 0.05 per frame", dict(clutter=0.05)),
    ("false alarms 0.10 per frame", dict(clutter=0.1)),
    ("false alarms 0.16 per frame", dict(clutter=0.16)),
    ("degraded detection curve", dict(pd_max=0.85, w50=10.0)),
    ("box width +25 % under 8 px", dict(width_bias=0.25)),
    ("all three (FA 0.10)", dict(REAL)),
    ("all three, headings calibrated to 0.5 deg", dict(REAL, yaw_bias_deg=0.5)),
    ("FA 0.16, three cameras to confirm", dict(clutter=0.16, tracker_kw=dict(confirm_cams=3))),
    ("1280 px camera", dict(cam=POOR)),
    ("1280 px camera, all three (FA 0.10)", dict(REAL, cam=POOR)),
]

SEEDS = [
    ("baseline", dict()),
    ("drone size known", dict(known=True)),
    ("1280 px camera", dict(cam=POOR)),
]


def _job(args):
    seed, kw = args
    return run(seed, **kw)


def _defaults(kw):
    kw = dict(kw)
    kw.setdefault("assumed_bias_deg", kw.get("yaw_bias_deg", 2.0))
    kw["tracker_kw"] = {"confirm_cams": 2, **kw.get("tracker_kw", {})}
    return kw


def table(title, rows, seeds, pool):
    print(f"[{title}] five observers, 3840 px unless stated; {seeds} seeds x 60 s; mean [worst seed]; "
          f"count ok with a 95 % interval over seeds")
    print(f"  {'configuration':42s} found       count ok          error m      swaps/drone/min  false tracks", flush=True)
    for name, kw in rows:
        kw = _defaults(kw)
        r = np.array(pool.map(_job, [(s, kw) for s in range(seeds)]))
        c = r[:, 1] * 100
        half = 1.96 * c.std(ddof=1) / np.sqrt(seeds)
        print(f"  {name:42s} {r[:, 0].mean() * 100:3.0f} % [{r[:, 0].min() * 100:3.0f}]  {c.mean():3.0f} % [{c.mean() - half:3.0f}-{c.mean() + half:3.0f}]"
              f"   {np.nanmean(r[:, 2]):4.2f} [{np.nanmax(r[:, 2]):4.1f}]  {r[:, 3].mean():4.2f} [{r[:, 3].max():4.1f}]      "
              f"{r[:, 4].mean():4.2f} [{r[:, 4].max():4.1f}]", flush=True)


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    procs = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    with Pool(procs) as pool:
        if which in ("detector", "all"):
            table("realistic detector", DETECTOR, 16, pool)
        if which in ("seeds", "all"):
            table("more seeds", SEEDS, 32, pool)
