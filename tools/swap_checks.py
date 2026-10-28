"""Why does knowing the drone's size double identity swaps (results/paired_checks.txt)? Logs every swap in the
baseline and size-known rows on the same seeds and sorts them by kind.

Run: python tools/swap_checks.py [main|calibrated] [processes]   (main ~12 min, calibrated ~6 min with 6 processes)
"calibrated": if a known size splits tracks because the cameras' heading biases disagree and the size pins the range,
then with headings at 0.5 deg the size-known row should no longer swap more than the baseline.
"""
import os
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from stage4_checks import run  # noqa: E402


def _job(args):
    seed, kw = args
    log = []
    kw = dict(kw)
    kw.setdefault("assumed_bias_deg", 2.0)
    r = run(seed, split=True, tracker_kw=dict(confirm_cams=2, **kw.pop("tracker_kw", {})),
            swap_log=log, **kw)
    return log, r


MAIN = [("baseline (size unknown)", dict()), ("drone size known", dict(known=True)),
        ("baseline, merge within 5 m", dict(tracker_kw=dict(merge_dist=5.0))),
        ("size known, merge within 5 m", dict(known=True, tracker_kw=dict(merge_dist=5.0)))]
CALIBRATED = [("baseline, headings 0.5 deg", dict(yaw_bias_deg=0.5, assumed_bias_deg=0.5)),
              ("size known, headings 0.5 deg", dict(known=True, yaw_bias_deg=0.5, assumed_bias_deg=0.5))]


def main(rows=MAIN, seeds=32, procs=8):
    print(f"[swaps] every identity swap, {seeds} seeds x 60 s, split scene stream (as results/paired_checks.txt)")
    with Pool(procs) as pool:
        for name, kw in rows:
            out = pool.map(_job, [(s, kw) for s in range(seeds)])
            logs, res = [o[0] for o in out], np.array([o[1][:5] for o in out], float)
            L = np.array([x for lg in logs for x in lg], float).reshape(-1, 11)
            flap = 0  # the drone goes back to the track it had two swaps ago (A -> B -> A)
            for lg in logs:
                last = {}
                for x in lg:
                    flap += last.get(x[6]) == x[8]
                    last[x[6]] = x[7]
            per = np.array([len(lg) for lg in logs])
            gone = L[:, 1] == 0
            print(f"  {name}: count ok {res[:, 1].mean() * 100:3.0f} %, error {np.nanmean(res[:, 2]):4.2f} m, swaps/drone/min "
                  f"{res[:, 3].mean():4.2f}, false tracks {res[:, 4].mean():4.2f}")
            print(f"    {len(L)} swaps; per seed median {np.median(per):.0f}, max {per.max()}, "
                  f"seeds with none {np.sum(per == 0)}; top 3 seeds hold {np.sort(per)[-3:].sum() / max(len(L), 1) * 100:.0f} %")
            print(f"    old track gone (died or merged away): {gone.sum()};  old track still alive: {(~gone).sum()};"
                  f"  back to the track held two swaps ago (flapping): {flap}")
            if (~gone).any():
                d = L[~gone, 2]
                print(f"      old still alive: its distance to the drone median {np.median(d):5.1f} m "
                      f"(10-90 % {np.percentile(d, 10):4.1f}-{np.percentile(d, 90):5.1f})")
                gap = L[~gone, 9]
                print(f"      the two tracks apart: median {np.median(gap):4.1f} m; under 5 m: {np.sum(gap < 5)}; the drone's "
                      f"nearest other drone: median {np.median(L[~gone, 10]):4.1f} m, under 8 m: {np.sum(L[~gone, 10] < 8)}")
            age = L[:, 3]
            print(f"    new track's age at the swap: median {np.median(age):5.1f} s; under 2 s: {np.sum(age < 2)}; "
                  f"new track's distance to the drone median {np.median(L[:, 4]):4.1f} m", flush=True)


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "main"
    main({"main": MAIN, "calibrated": CALIBRATED}[which], procs=int(sys.argv[2]) if len(sys.argv) > 2 else 8)
