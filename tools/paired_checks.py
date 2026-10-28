"""Paired comparisons (doc 03 section 9): rows that were within seed spread of each other, compared on the same seeds.

Each pair runs both configurations on every seed and reports the mean per-seed difference with a 95 % interval
(1.96 sd / sqrt(n) of the differences). The seed-to-seed spread (scene difficulty) cancels in the difference, so a
paired interval is much narrower than the two unpaired ones. The scene (drone paths, orbit placement) comes from its own
random stream (run(split=True)), so a camera or detector change does not change the drones; noise still differs.

Run: python tools/paired_checks.py [processes]   (~20 min with 8 processes)
"""
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from stage4_checks import POOR, run  # noqa: E402

M7 = dict(mates=True, cue="calibrate")
PAIRS = [
    ("drone size known - baseline", dict(known=True), dict(), 64),
    ("1280 px camera - 3840 px camera", dict(cam=POOR), dict(), 64),
    ("stage 7: 3 observers - 5 observers", dict(M7, n_obs=3), dict(M7), 32),
]


def _job(args):
    seed, kw = args
    return run(seed, split=True, **kw)


def _defaults(kw):
    kw = dict(kw)
    kw.setdefault("assumed_bias_deg", kw.get("yaw_bias_deg", 2.0))
    kw["tracker_kw"] = {"confirm_cams": 2, **kw.get("tracker_kw", {})}
    return kw


def _ci(x):
    x = x[~np.isnan(x)]
    return x.mean(), 1.96 * x.std(ddof=1) / np.sqrt(len(x))


def main(procs=8):
    print("[paired] same seeds, scene from its own random stream; five observers, 3840 px, heading bias 2 deg declared,")
    print("  size unknown, 0.02 false alarms per frame unless stated; 60 s per seed")
    print("  A - B; each side mean +- 95 % half-width over seeds (unpaired); then the per-seed difference +- 95 % half-width",
          flush=True)
    with Pool(procs) as pool:
        for name, a, b, seeds in PAIRS:
            ra = np.array(pool.map(_job, [(s, _defaults(a)) for s in range(seeds)]), float)
            rb = np.array(pool.map(_job, [(s, _defaults(b)) for s in range(seeds)]), float)
            print(f"  {name} ({seeds} seeds)")
            for col, label, scale, unit in ((1, "count ok", 100, "%"), (0, "found", 100, "%"), (2, "error", 1, "m"),
                                            (3, "swaps/drone/min", 1, ""), (4, "false tracks", 1, "")):
                (ma, ha), (mb, hb), (md, hd) = _ci(ra[:, col] * scale), _ci(rb[:, col] * scale), \
                    _ci((ra[:, col] - rb[:, col]) * scale)
                worse = (ra[:, col] < rb[:, col]).mean() if col in (0, 1) else (ra[:, col] > rb[:, col]).mean()
                print(f"    {label:16s} A {ma:6.2f} +-{ha:5.2f}  B {mb:6.2f} +-{hb:5.2f}  A-B {md:+6.2f} +-{hd:5.2f} {unit:2s}"
                      f"  (A worse on {worse * 100:3.0f} % of seeds)", flush=True)
            if not np.isnan(ra[:, 8]).all():
                print(f"    teammate boxes leaked: A {np.nanmean(ra[:, 8]) * 100:4.2f} %  B {np.nanmean(rb[:, 8]) * 100:4.2f} %;"
                      f"  tiles A {np.nanmean(ra[:, 11]):4.1f}  B {np.nanmean(rb[:, 11]):4.1f}", flush=True)


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 8)
