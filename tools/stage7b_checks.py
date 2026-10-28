"""Stage 7b check (03_build_plan.md): where teammate boxes leak, and teammates' shared positions arriving late, lost,
with clock offsets, or with drifting GNSS error.

The stage 7 cued + calibrated row (tools/stage7_checks.py) with one change per row. Same seeds as stage 7, so the first
row reproduces its 87 %. Each change is described in stage4_checks.run (share=...). Observers fly 5 m/s on 15 m
circles, so a 0.5 s-old position is 2.5 m off.

Run: python tools/stage7b_checks.py [leaks|share|fix|drift|all] [processes]
  (leaks ~4 min, share ~20 min, fix ~15 min, drift ~10 min with 6-8 processes)
"fix": the two leak causes found by "leaks", one change per row: cues just outside the image still matched (edge, in
sigmas), then a wider gate (99.9 % instead of 99 %), with five and with three observers.
"""
import os
import sys
import warnings
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from stage4_checks import run  # noqa: E402
from stage7_checks import main as table  # noqa: E402

C = dict(mates=True, cue="calibrate")
SHARE = [
    ("cued + calibrated, positions fresh (stage 7)", dict(C)),
    ("positions 0.5 s old, used as they are", dict(C, share=dict(delay=0.5))),
    ("positions 0.5 s old, dead reckoned", dict(C, share=dict(delay=0.5, dr=True))),
    ("0.5 s old + bursty loss, as they are", dict(C, share=dict(delay=0.5, bursty=True))),
    ("0.5 s old + bursty loss, dead reckoned", dict(C, share=dict(delay=0.5, bursty=True, dr=True))),
    ("dead reckoned, clock offsets sd 0.05 s", dict(C, share=dict(dr=True, clock_sd=0.05))),
    ("dead reckoned, clock offsets sd 0.2 s", dict(C, share=dict(dr=True, clock_sd=0.2))),
    ("all link effects, clocks 0.05 s (see doc)", dict(C, delay=0.5, bursty=True,
                                                       share=dict(delay=0.5, bursty=True, dr=True, clock_sd=0.05))),
    ("GNSS error drifts, tau 60 s", dict(C, share=dict(gnss_tau=60.0))),
    ("GNSS error drifts, tau 10 s", dict(C, share=dict(gnss_tau=10.0))),
]

E3 = dict(edge=3.0)
G999 = dict(edge=3.0, gate=16.27)  # chi-square 3 dof, 99.9 %
FIX = [
    ("cued + calibrated (stage 7)", dict(C)),
    ("cues up to 3 sigma outside the image", dict(C, cal_kw=E3)),
    ("  + gate 99.9 %", dict(C, cal_kw=G999)),
    ("3 observers (stage 7)", dict(C, n_obs=3)),
    ("3 observers, cues outside the image", dict(C, n_obs=3, cal_kw=E3)),
    ("3 observers, + gate 99.9 %", dict(C, n_obs=3, cal_kw=G999)),
]
DRIFT = [  # GNSS drift cut the count from 87 % to 73 %: is it the cueing or the tracker?
    ("stage 4 scene (teammates out of view)", dict()),
    ("stage 4 scene, GNSS error drifts, tau 60 s", dict(share=dict(gnss_tau=60.0))),
    ("stage 4 scene, headings 0.5 deg", dict(yaw_bias_deg=0.5)),
    ("stage 4 scene, headings 0.5 deg, drift tau 60 s", dict(yaw_bias_deg=0.5, share=dict(gnss_tau=60.0))),
]
LEAK_DRIFT_ROWS = [("5 observers, GNSS drift tau 60 s", dict(C, share=dict(gnss_tau=60.0)))]
DR60 = dict(gnss_tau=60.0)
FIX_DRIFT = [  # stage 7's fixed GNSS errors were small by luck (build log 11.5); the drifting error visits the whole spread
    ("GNSS drift 60 s, cued + calibrated", dict(C, share=DR60)),
    ("GNSS drift 60 s, cues outside the image", dict(C, share=DR60, cal_kw=E3)),
    ("GNSS drift 60 s, + gate 99.9 %", dict(C, share=DR60, cal_kw=G999)),
    ("GNSS drift 60 s, + gate 99.9 %, sigma_pos 1.25 m", dict(C, share=DR60, cal_kw=dict(G999, sigma_pos=1.25))),
]
GATE_ONLY = [  # the edge change alone hurt under drift (start-up mis-locks): the wider gate without it
    ("GNSS drift 60 s, gate 99.9 % only (no edge)", dict(C, share=DR60, cal_kw=dict(gate=16.27))),
    ("fixed GNSS error, gate 99.9 % only (no edge)", dict(C, cal_kw=dict(gate=16.27))),
]
LEAK_ROWS = [("5 observers", C), ("3 observers", dict(C, n_obs=3))]
LEAK_FIX_ROWS = [("5 observers, cues outside the image", dict(C, cal_kw=E3)),
                 ("5 observers, + gate 99.9 %", dict(C, cal_kw=G999))]


def _leaks(args):
    seed, kw = args
    d = []
    run(seed, assumed_bias_deg=2.0, tracker_kw=dict(confirm_cams=2), diag=d, **kw)
    return d


def leaks(rows=LEAK_ROWS, seeds=16, procs=8):
    """Every teammate box that reached the tracker in the cued + calibrated row, sorted by cause."""
    print(f"[stage 7b leaks] teammate boxes not labelled, cued + calibrated, {seeds} seeds x 60 s")
    with Pool(procs) as pool:
        for name, kw in rows:
            d = sum(pool.map(_leaks, [(s, kw) for s in range(seeds)]), [])
            n = len(d)
            out = [x for x in d if not x["seen"]]
            inv = [x for x in d if x["seen"]]
            early = sum(x["t"] < 1.0 for x in d)
            print(f"  {name}: {n} leaked boxes; {early} in the first second (calibration not yet converged)")
            print(f"    cue out of view while the box is in view: {len(out)}" +
                  (f" (box within 100 px of the image edge: {sum(min(x['u'], 3840 - x['u'], x['v'], 2160 - x['v']) < 100 for x in out)})"
                   if out else ""))
            if inv:
                T = np.array([x["terms"] for x in inv])
                dom = np.argmax(T, 1)
                print(f"    cue in view, outside the gate: {len(inv)}; largest gate term horizontal {np.sum(dom == 0)}, "
                      f"vertical {np.sum(dom == 1)}, size {np.sum(dom == 2)}")
                for lbl, key in (("range m", "range"), ("width px", "width"), ("nearest other box px", "near_px"),
                                 ("heading error deg", "head_err_deg"), ("shared position error m", "pos_err")):
                    v = np.array([abs(x[key]) for x in inv])
                    print(f"      {lbl:24s} median {np.median(v):7.2f}  10-90 % {np.percentile(v, 10):7.2f} - "
                          f"{np.percentile(v, 90):7.2f}")
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore", RuntimeWarning)
                    print(f"      gate terms (median): horizontal {np.median(T[:, 0]):5.1f}  vertical {np.median(T[:, 1]):5.1f}"
                          f"  size {np.median(T[:, 2]):5.1f}  (gate 11.3 on the sum)", flush=True)


if __name__ == "__main__":
    which = sys.argv[1] if len(sys.argv) > 1 else "all"
    if which == "gateonly":
        table(GATE_ONLY, procs=int(sys.argv[2]))
        sys.exit()
    if which == "fixunderdrift":
        table(FIX_DRIFT, procs=int(sys.argv[2]))
        sys.exit()
    if which == "fixdrift":  # both follow-ups, one process pool size
        leaks(LEAK_DRIFT_ROWS, procs=int(sys.argv[2]))
        table(DRIFT, procs=int(sys.argv[2]))
        leaks(LEAK_FIX_ROWS, procs=int(sys.argv[2]))
        table(FIX, procs=int(sys.argv[2]))
        sys.exit()
    procs = int(sys.argv[2]) if len(sys.argv) > 2 else 8
    if which in ("leaks", "all"):
        leaks(procs=procs)
    if which in ("share", "all"):
        table(SHARE, procs=procs)
    if which in ("drift", "all"):
        leaks(LEAK_DRIFT_ROWS, procs=procs)
        table(DRIFT, procs=procs)
    if which in ("fix", "all"):
        leaks(LEAK_FIX_ROWS, procs=procs)
        table(FIX, procs=procs)
