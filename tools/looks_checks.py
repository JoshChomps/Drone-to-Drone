"""Look scheduler check (doc 03 section 9): how many 640-px tiles per second must each camera run?

Stage 7 counted the tiles that cued looks need (~9 of 24 per frame) but always searched the whole frame. Here each
camera has a budget: windows around tracks first, then teammates' cues, then a round-robin sweep (stage4_checks.run,
looks=...). A full 4K frame is 24 tiles, so 240 tiles/s searches every frame whole at 10 Hz. Doc 02 section 5.2's
figure for a Pi 5 CPU is ~15 tiles/s; an accelerator is assumed to give ~60 (not measured, not checked).

Same seeds as stage 7; the random draws are the same as the full-frame run, so rows differ only through what is searched.
Run: python tools/looks_checks.py [main|confirm] [processes]   (main ~12 min, confirm ~6 min with 6 processes)
"confirm": searching less scored better than searching everything. If that is because false alarms and leaked teammate
boxes get fewer chances to reach 5 hits, a stricter confirmation with the whole frame should do the same.
"""
import os
import sys
import warnings
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from stage4_checks import run  # noqa: E402

C = dict(mates=True, cue="calibrate", assumed_bias_deg=2.0, tracker_kw=dict(confirm_cams=2))
ROWS = [
    ("whole frame every frame (240 tiles/s)", dict(C)),
    ("60 tiles/s", dict(C, looks=dict(rate=60))),
    ("30 tiles/s", dict(C, looks=dict(rate=30))),
    ("15 tiles/s", dict(C, looks=dict(rate=15))),
    ("15 tiles/s, cues every 5th frame", dict(C, looks=dict(rate=15, cue_every=5))),
    ("15 tiles/s, fixed track order (first version)", dict(C, looks=dict(rate=15, rotate=False))),
    ("15 tiles/s, sweep only (no track/cue windows)", dict(C, looks=dict(rate=15, cue_every=0, tracks=False))),
    ("60 tiles/s, sweep only", dict(C, looks=dict(rate=60, cue_every=0, tracks=False))),
]
CONFIRM = [
    ("whole frame, confirm after 10 hits", dict(C, tracker_kw=dict(confirm_cams=2, confirm_hits=10))),
    ("whole frame, confirm after 20 hits", dict(C, tracker_kw=dict(confirm_cams=2, confirm_hits=20))),
    ("whole frame, three cameras to confirm", dict(C, tracker_kw=dict(confirm_cams=3))),
]


def _job(args):
    seed, kw = args
    return run(seed, **kw)


def main(rows=ROWS, seeds=16, procs=6):
    print(f"[looks] stage 7 cued + calibrated, five observers, 3840 px (24 tiles of 640 px); {seeds} seeds x 60 s; means;")
    print("  count ok with a 95 % interval over seeds; time to find: until each drone first has a confirmed track within 8 m")
    print(f"  {'configuration':46s} found count ok        err m  swaps  false | heading after  leak | time to find s",
          flush=True)
    with Pool(procs) as pool:
        for name, kw in rows:
            r = np.array(pool.map(_job, [(s, kw) for s in range(seeds)]), float)
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)
                m = np.nanmean(r, 0)
            c = r[:, 1] * 100
            half = 1.96 * c.std(ddof=1) / np.sqrt(seeds)
            print(f"  {name:46s} {m[0] * 100:3.0f} %  {c.mean():3.0f} % [{c.mean() - half:3.0f}-{c.mean() + half:3.0f}]  "
                  f"{m[2]:4.2f}  {m[3]:4.2f}  {m[4]:4.2f} | {m[6]:4.2f}        {m[8] * 100:4.1f} | {m[12]:4.1f} "
                  f"[worst {np.max(r[:, 12]):4.1f}]", flush=True)


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "main"
    main({"main": ROWS, "confirm": CONFIRM}[which], procs=int(sys.argv[2]) if len(sys.argv) > 2 else 6)
