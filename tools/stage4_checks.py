"""Stage 4 / 4b check (03_build_plan.md): a team's cameras keeping track of five other drones.

Five identical drones (0.77 m, 4 m/s, turning, 45-55 m altitude) wander in an 80 m-wide area. Up to five observers
orbit pre-planned circles around it at 60 m with fixed forward cameras pointed at the middle of the area. One tracker
(d2d.track) runs on observer 1; teammates' sightings reach it over the link. Run: python tools/stage4_checks.py (~20 min)

Modelled: field of view, detection probability by box width, pixel and attitude noise, heading bias, teammates' position
error, one box when two drones overlap in an image, false alarms, link delay and bursty loss.
Not modelled: image rendering, a real detector, teammates appearing in each other's cameras (stage 7 removes those
with the radio cue), gaze control, wind.

Scored against truth every step after 10 s (confirmed tracks paired with true drones, at most 8 m apart):
  found      share of (drone, time) with a track on it
  count ok   share of time the tracker reports exactly five drones
  error      RMS distance of paired tracks
  swaps      identity changes per drone per minute (the track on a drone is not the one that was there before)
  false      confirmed tracks on nothing, average at any time
"""
import os
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from d2d.cue import LABEL_NONE, HeadingCalibrator  # noqa: E402
from d2d.geometry import Camera, attitude  # noqa: E402
from d2d.track import Sighting, Tracker  # noqa: E402

DEG = np.pi / 180
GOOD, POOR = Camera(3840, 2160, 90.0, 15.0), Camera(1280, 720, 90.0, 15.0)
SPAN, W_MIN, N_OTHER = 0.77, 8.0, 5


def gilbert_elliott(rng, n, p_gb=0.02, p_bg=0.20):
    lost, state, u = np.zeros(n, bool), 0, rng.random(n)
    for k in range(n):
        state = (1 if u[k] < p_gb else 0) if state == 0 else (0 if u[k] < p_bg else 1)
        lost[k] = state == 1
    return lost


def run(seed, n_obs=5, cam=GOOD, known=False, yaw_bias_deg=2.0, delay=0.0, bursty=False, clutter=0.02,
        assumed_bias_deg=0.0, tracker_kw=None, T=60.0, dt=0.1, pd_max=0.95, w50=W_MIN, width_bias=0.0, mates=False,
        cue=None, cal_kw=None, split=False, diag=None, share=None, swap_log=None, looks=None):
    """pd_max, w50: detection probability pd_max / (1 + exp(-(width - w50) / 2)). width_bias: boxes of true width under
    W_MIN px come out this fraction too wide (the sibling detector measures +25 %). Defaults reproduce doc 03 section 6.

    Stage 7: mates puts the observers in each other's cameras. cue: None sends every box to the tracker; "fixed" labels
    boxes against teammates' shared positions, with the declared bias as the cue's uncertainty, and never learns it;
    "calibrate" also estimates each camera's heading bias (d2d.cue) and corrects that camera's sightings.

    Returns found, count ok, error, swaps, false tracks; then for stage 7: RMS heading bias before and error after
    calibration (deg), share of teammate boxes labelled right, share leaked to the tracker, share of other drones' boxes
    withheld or taken for a teammate, share taken for a teammate, and 640-px tiles per frame covering cues and tracks
    (camera 1).

    split: draw the scene (drone paths, orbit placement) from its own random stream, so configurations that change the
    number of boxes (another camera, another detector) still see the same drones on the same seed. Off by default: the
    documented numbers were made with one stream. diag: a list; each teammate box that leaks to the tracker appends a
    dict saying why (see _leak).

    share (stage 7b): how teammates' shared positions reach each camera, a dict with
      delay     s, constant latency of the position messages (one per teammate per step)
      bursty    True: Gilbert-Elliott loss on them (per sender); the last message received is used
      dr        True: extrapolate the last message with its reported velocity over its age (dead reckoning); False: use
                the stale position as it is
      clock_sd  s, each drone's clock offset (normal); the age computed from a message's time stamp, and a late
                sighting's age, are wrong by the difference of two offsets
      gnss_tau  s: each teammate's GNSS error drifts (first-order Gauss-Markov, same 0.7 m sd) instead of staying fixed
    Defaults (None) keep the stage 7 behaviour: fresh positions, no clock offset, fixed GNSS error.

    swap_log: a list; each identity swap appends (t, whether the old track still exists, its distance to the drone
    (m, nan if gone), the new track's age (s), its distance to the drone (m), whether old and new were both alive the
    step before, the drone, the old id, the new id, the distance between the two tracks (nan if the old is gone), the
    distance from the drone to the nearest other drone).

    looks (look scheduler): None searches every frame whole. Otherwise a dict: rate (640-px tiles per second per camera),
    cue_every (search teammates' cue windows every Nth frame; 0 never), tracks (False: no track windows, sweep only),
    rotate (default True: among tracks, the window this camera searched longest ago comes first; False: a fixed order,
    which starves all but the first tracks when the budget is short, build log section 11).
    Each frame a camera searches, in order and while its budget lasts: windows around confirmed tracks, then tentative
    tracks, then cues, then the next tiles of a round-robin sweep over the frame. Detections and false alarms outside the
    searched tiles are dropped (after all random draws, so the noise is the same as in the full-frame run). Every camera
    uses observer 1's tracks (in a real team each drone's tracker is near-identical, as sightings are shared).

    Returns, after the 12 numbers above, the mean time (s) until each drone first has a confirmed track within 8 m
    (T if never)."""
    rng = np.random.default_rng(seed)
    rt = np.random.default_rng([seed, 1]) if split else rng
    n = int(T / dt)
    s_bearing, s_angle = np.hypot(np.hypot(1.0 / cam.f, 0.5 * DEG), assumed_bias_deg * DEG), 1.5 / cam.f
    # the other drones
    p = np.column_stack([rt.uniform(-25, 25, (N_OTHER, 2)), 45 + 2.5 * np.arange(N_OTHER)])
    hd, w = rt.uniform(0, 2 * np.pi, N_OTHER), np.zeros(N_OTHER)
    # observers: orbit centres evenly spaced on a 55 m ring
    ring = 2 * np.pi * np.arange(n_obs) / n_obs + rt.uniform(0, 2 * np.pi)
    centres = np.column_stack([55 * np.cos(ring), 55 * np.sin(ring), np.full(n_obs, 60.0)])
    phase = rt.uniform(0, 2 * np.pi, n_obs)
    yaw_bias = rng.normal(0, yaw_bias_deg * DEG, n_obs)
    pos_bias = np.vstack([np.zeros((1, 3)), rng.normal(0, 0.7, (n_obs - 1, 3))])
    lost = np.stack([gilbert_elliott(rng, n) if bursty else np.zeros(n, bool) for _ in range(n_obs)])
    nd = int(round(delay / dt))
    sh = {"delay": 0.0, "bursty": False, "dr": False, "clock_sd": 0.0, "gnss_tau": None, **(share or {})}
    nds = int(round(sh["delay"] / dt))
    lost_sh = np.stack([gilbert_elliott(rng, n) for _ in range(n_obs)]) if sh["bursty"] else np.zeros((n_obs, n), bool)
    clk = rng.normal(0, sh["clock_sd"], n_obs) if sh["clock_sd"] else np.zeros(n_obs)
    last_rx, sent = np.zeros(n_obs, int), []  # step of the last position message received from each drone; what was sent
    lk = None if looks is None else {"rate": 24 / dt, "cue_every": 1, "tracks": True, **looks}
    credit, sweep = np.zeros(n_obs), np.zeros(n_obs, int)
    looked = [{} for _ in range(n_obs)]  # per camera: track id -> step it last searched that track's window
    first = np.full(N_OTHER, np.nan)
    trk = Tracker(**({"size": SPAN, "size_sigma": 0.01} if known else {}), **(tracker_kw or {}))
    queue, prev = [], [None] * N_OTHER
    found = count_ok = false = swaps = steps = 0
    sq = []
    cals = [HeadingCalibrator(**{"sigma0_deg": max(yaw_bias_deg, 0.5), **(cal_kw or {})}) for _ in range(n_obs)]
    if cue == "fixed":
        for c in cals:
            c.P = (assumed_bias_deg * DEG) ** 2
    lab = np.zeros(5)  # teammate boxes, labelled right, leaked; other drones' boxes withheld or mislabelled, mislabelled
    n_other_boxes = 0
    head_sq, bias_sq, tiles = [], [], []
    for k in range(n):
        t = k * dt
        if k % 40 == 0:
            w = rt.uniform(-0.25, 0.25, N_OTHER)
        out = np.hypot(p[:, 0], p[:, 1]) > 40
        w = np.where(out, 0.4 * np.sign(-np.cos(hd) * p[:, 1] + np.sin(hd) * p[:, 0]), w)
        hd = hd + w * dt
        p = p + np.column_stack([4 * np.cos(hd), 4 * np.sin(hd), np.zeros(N_OTHER)]) * dt
        batches = []
        obs = centres + 15 * np.column_stack([np.cos(0.33 * t + phase), np.sin(0.33 * t + phase), np.zeros(n_obs)])
        if sh["gnss_tau"]:
            a = dt / sh["gnss_tau"]
            pos_bias[1:] += -a * pos_bias[1:] + 0.7 * np.sqrt(2 * a) * rng.normal(size=(n_obs - 1, 3))
        vel = 15 * 0.33 * np.column_stack([-np.sin(0.33 * t + phase), np.cos(0.33 * t + phase), np.zeros(n_obs)])
        sent.append((obs + pos_bias, vel))
        if k >= nds:
            last_rx = np.where(lost_sh[:, k - nds], last_rx, k - nds)
        for i in range(n_obs):
            o = obs[i]
            yaw = np.arctan2(-o[1], -o[0])  # pre-planned gaze: the middle of the area
            r_true = attitude(yaw)
            others = [j for j in range(n_obs) if j != i]
            objs = np.vstack([p, obs[others]]) if mates else p  # rows from N_OTHER on are teammates
            boxes, src = [], []
            for j in np.argsort(np.linalg.norm(objs - o, axis=1)):  # nearest first
                b = cam.box(o, r_true, objs[j], SPAN, 0.25)  # the documented runs used 0.25 m; the model file says 0.29
                if b is None:
                    continue
                c = np.array([(b[0] + b[2]) / 2, (b[1] + b[3]) / 2])
                if not cam.in_view(c[None], np.ones(1))[0]:
                    continue
                if any(q[0] - 2 <= c[0] <= q[2] + 2 and q[1] - 2 <= c[1] <= q[3] + 2 for q in boxes):
                    continue  # hidden behind, or fused with, a nearer drone's box
                if rng.random() > pd_max / (1 + np.exp(-((b[2] - b[0]) - w50) / 2.0)):
                    continue
                if b[2] - b[0] < W_MIN:
                    b = b + np.array([-1, 0, 1, 0]) * width_bias * (b[2] - b[0]) / 2
                boxes.append(b)
                src.append(j)
            for _ in range(rng.poisson(clutter)):  # false alarms anywhere in the image
                c, wd = rng.uniform([0, 0], [cam.width, cam.height]), rng.uniform(6, 14)
                boxes.append(np.array([c[0] - wd / 2, c[1] - wd / 6, c[0] + wd / 2, c[1] + wd / 6]))
                src.append(-1)
            r_est = attitude(yaw + yaw_bias[i], rng.normal(0, 0.5 * DEG), rng.normal(0, 0.5 * DEG))
            boxes = [b + np.array([-1, 0, 1, 0]) * rng.normal(0, 0.75) + np.repeat(rng.normal(0, 1.0, 2), 2)[[0, 2, 1, 3]]
                     for b in boxes]
            labels, sb = np.full(len(boxes), LABEL_NONE), s_bearing
            if cue:
                shared = np.array([sent[last_rx[j]][0][j] for j in others])  # teammates' positions as received
                if sh["dr"]:
                    age = (k - last_rx[others]) * dt + clk[i] - clk[others]
                    shared = shared + np.array([sent[last_rx[j]][1][j] for j in others]) * age[:, None]
            if lk is not None:
                credit[i] += lk["rate"] * dt
                r_s = cals[i].correct(attitude(yaw + yaw_bias[i]))
                sb_s = np.hypot(np.hypot(1.0 / cam.f, 0.5 * DEG), cals[i].declared if cue == "calibrate" else
                                assumed_bias_deg * DEG)
                wins, ids = [], []
                if lk["tracks"]:  # confirmed first; among them, the one this camera searched longest ago first
                    order = sorted(trk.tracks, key=lambda tr: (not tr.confirmed, looked[i].get(tr.id, -1))
                                   if lk.get("rotate", True) else not tr.confirmed)
                    for tr in order:
                        wins.append((tr.f.position, 3 * cam.f * np.hypot(sb_s, np.sqrt(np.trace(tr.f.P[:3, :3]) / 3) /
                                                                        np.linalg.norm(tr.f.position - o)) + 20))
                        ids.append(tr.id)
                if cue and lk["cue_every"] and k % lk["cue_every"] == 0:
                    wins += [(q, 3 * cam.f * np.hypot(cals[i].declared, 1.0 / np.linalg.norm(q - o)) + 20) for q in shared]
                hit, sweep[i], used = _schedule(cam, o + pos_bias[i], r_s, wins, int(credit[i]), sweep[i])
                credit[i] -= used
                for tid, (q, half) in zip(ids, wins):
                    cells = _cells(cam, o + pos_bias[i], r_s, q, half)
                    if cells and all(hit[c] for c in cells):
                        looked[i][tid] = k
                keep = [kb for kb, b in enumerate(boxes) if hit[int(min(max((b[1] + b[3]) / 2, 0), cam.height - 1) // 640),
                                                              int(min(max((b[0] + b[2]) / 2, 0), cam.width - 1) // 640)]]
                boxes, src, labels = [boxes[kb] for kb in keep], [src[kb] for kb in keep], labels[keep]
            if cue:
                if cue == "calibrate":
                    labels, r_est = cals[i].step(dt, cam, o + pos_bias[i], r_est, boxes, shared)
                    sb = np.hypot(np.hypot(1.0 / cam.f, 0.5 * DEG), cals[i].declared)
                else:
                    labels, _ = cals[i].match(cam, o + pos_bias[i], r_est, boxes, shared)
                for kb, (lb, j) in enumerate(zip(labels, src)):
                    if j >= N_OTHER:
                        lab[0] += 1
                        lab[1] += lb == j - N_OTHER
                        lab[2] += lb == LABEL_NONE
                        if lb == LABEL_NONE and diag is not None:
                            diag.append(_leak(cam, cals[i], boxes, kb, j - N_OTHER, o, obs[others[j - N_OTHER]],
                                              cals[i].b - yaw_bias[i],
                                              shared[j - N_OTHER] - obs[others[j - N_OTHER]] - pos_bias[i], t))
                    elif j >= 0:
                        n_other_boxes += 1
                        lab[3] += lb != LABEL_NONE
                        lab[4] += lb >= 0
            if cue == "calibrate" and t >= 10.0:
                head_sq.append((cals[i].b - yaw_bias[i]) ** 2)
                bias_sq.append(yaw_bias[i] ** 2)
            batch = []
            for b, lb in zip(boxes, labels):
                if lb != LABEL_NONE:
                    continue  # a teammate, or possibly one: not sent to the tracker as an unknown drone
                g = cam.bearing(r_est, [(b[0] + b[2]) / 2, (b[1] + b[3]) / 2])[0]
                sbi = sb if i == 0 else np.hypot(sb, 0.7 / 50)
                batch.append(Sighting(o + pos_bias[i], g, cam.subtended_angle(b), sbi, s_angle, cam=i))
            if i == 0 and cue and t >= 10.0:
                tiles.append(_tiles(cam, o, r_est, shared, cals[0], trk.confirmed, sb))
            if i == 0:
                batches.append(batch)
            elif not lost[i, k]:
                queue.append((k + nd, k, batch))
        due = [q for q in queue if q[0] <= k]
        queue = [q for q in queue if q[0] > k]
        for _, k0, batch in due:
            for s in batch:
                s.back = max(0.0, (k - k0) * dt + clk[0] - clk[s.cam])  # age from the time stamps
            batches.append(batch)
        alive_before = {tr.id for tr in trk.tracks}
        trk.step(dt, batches)
        if trk.confirmed and np.isnan(first).any():
            d = np.linalg.norm(p[:, None] - np.array([tr.f.position for tr in trk.confirmed])[None], axis=2).min(1)
            first = np.where(np.isnan(first) & (d <= 8.0), t, first)
        if t < 10.0:
            continue
        steps += 1
        tracks = trk.confirmed
        pair = {}
        if tracks:
            d = np.linalg.norm(p[:, None] - np.array([tr.f.position for tr in tracks])[None], axis=2)
            for a, b in zip(*linear_sum_assignment(np.minimum(d, 1e3))):
                if d[a, b] <= 8.0:
                    pair[a] = tracks[b].id
                    sq.append(d[a, b] ** 2)
        found += len(pair)
        count_ok += len(tracks) == N_OTHER
        false += len(tracks) - len(pair)
        for a in range(N_OTHER):
            if a in pair:
                swaps += prev[a] is not None and prev[a] != pair[a]
                if swap_log is not None and prev[a] is not None and prev[a] != pair[a]:
                    old = next((tr for tr in trk.tracks if tr.id == prev[a]), None)
                    new = next(tr for tr in tracks if tr.id == pair[a])
                    swap_log.append((t, old is not None, np.linalg.norm(old.f.position - p[a]) if old else np.nan,
                                     trk.t - new.born, np.linalg.norm(new.f.position - p[a]),
                                     prev[a] in alive_before and pair[a] in alive_before, a, prev[a], pair[a],
                                     np.linalg.norm(old.f.position - new.f.position) if old else np.nan,
                                     np.delete(np.linalg.norm(p - p[a], axis=1), a).min()))
                prev[a] = pair[a]
    minutes = steps * dt / 60
    nan = np.nan
    return (found / (steps * N_OTHER), count_ok / steps, np.sqrt(np.mean(sq)) if sq else nan,
            swaps / N_OTHER / minutes, false / steps,
            np.sqrt(np.mean(bias_sq)) / DEG if bias_sq else nan, np.sqrt(np.mean(head_sq)) / DEG if head_sq else nan,
            lab[1] / lab[0] if lab[0] else nan, lab[2] / lab[0] if lab[0] else nan,
            lab[3] / n_other_boxes if n_other_boxes else nan, lab[4] / n_other_boxes if n_other_boxes else nan,
            np.mean(tiles) if tiles else nan, np.nanmean(np.where(np.isnan(first), T, first)))


def _cells(cam, o, r, q, half, tile=640):
    """(row, column) of the tiles a window of half-size `half` px around point q's projection overlaps."""
    uv, z = cam.project(o, r, q)
    u, v = uv[0]
    if z[0] <= 0 or u + half < 0 or u - half >= cam.width or v + half < 0 or v - half >= cam.height:
        return []
    c0, c1 = int(max(u - half, 0) // tile), int(min(u + half, cam.width - 1) // tile)
    r0, r1 = int(max(v - half, 0) // tile), int(min(v + half, cam.height - 1) // tile)
    return [(a, b) for a in range(r0, r1 + 1) for b in range(c0, c1 + 1)]


def _schedule(cam, o, r, windows, budget, ptr, tile=640):
    """Choose at most `budget` tiles: each window in order if all its new tiles fit, then a round-robin sweep from
    tile `ptr`. Returns the searched-tile mask, the new sweep pointer and the number of tiles used."""
    nx, ny = -(-cam.width // tile), -(-cam.height // tile)
    hit, used = np.zeros((ny, nx), bool), 0
    for q, half in windows:
        new = [c for c in _cells(cam, o, r, q, half, tile) if not hit[c]]
        if used + len(new) <= budget:
            for c in new:
                hit[c] = True
            used += len(new)
    while used < budget and not hit.all():
        c = divmod(ptr % (nx * ny), nx)
        ptr += 1
        if not hit[c]:
            hit[c] = True
            used += 1
    return hit, ptr, used


def _leak(cam, cal, boxes, k, m, o, q, head_err, pos_err, t):
    """Why teammate m's box k reached the tracker. cal.last holds match()'s gate terms, computed with the heading
    estimate before this frame's update. head_err: heading estimate minus true bias (rad); pos_err: the error of the
    shared position difference (m)."""
    b = boxes[k]
    c = np.array([(b[0] + b[2]) / 2, (b[1] + b[3]) / 2])
    near = min((np.hypot(*(c - [(a[0] + a[2]) / 2, (a[1] + a[3]) / 2])) for i, a in enumerate(boxes) if i != k),
               default=np.inf)
    seen = cal.last is not None and cal.last["seen"][m]
    terms = cal.last["terms"][m, k] if seen else np.full(3, np.nan)
    return dict(t=t, seen=bool(seen), terms=terms, range=float(np.linalg.norm(q - o)), width=float(b[2] - b[0]),
                u=float(c[0]), v=float(c[1]), near_px=float(near), head_err_deg=float(head_err / DEG),
                pos_err=float(np.linalg.norm(pos_err)), uv_cue=cal.last["uv"][m] if cal.last is not None else None)


def _tiles(cam, o, r, shared, cal, tracks, sb, tile=640):
    """640-px tiles one camera must run to look where teammates and confirmed tracks should be (3-sigma windows plus
    20 px for the box). A full scan is ceil(w / 640) x ceil(h / 640) tiles. New drones still need full scans now and
    then; this counts only the cued looks."""
    nx, ny = -(-cam.width // tile), -(-cam.height // tile)
    hit = np.zeros((ny, nx), bool)
    looks = [(q, 3 * cam.f * np.hypot(cal.declared, 1.0 / np.linalg.norm(q - o)) + 20) for q in shared]
    looks += [(tr.f.position, 3 * cam.f * np.hypot(sb, np.sqrt(np.trace(tr.f.P[:3, :3]) / 3) /
                                                  np.linalg.norm(tr.f.position - o)) + 20) for tr in tracks]
    for q, half in looks:
        uv, z = cam.project(o, r, q)
        u, v = uv[0]
        if z[0] <= 0 or u + half < 0 or u - half >= cam.width or v + half < 0 or v - half >= cam.height:
            continue
        c0, c1 = int(max(u - half, 0) // tile), int(min(u + half, cam.width - 1) // tile)
        r0, r1 = int(max(v - half, 0) // tile), int(min(v + half, cam.height - 1) // tile)
        hit[r0:r1 + 1, c0:c1 + 1] = True
    return int(hit.sum())


CONFIGS = [
    ("1 observer", dict(n_obs=1)),
    ("2 observers", dict(n_obs=2)),
    ("3 observers", dict(n_obs=3)),
    ("5 observers", dict(n_obs=5)),
    ("5 observers, drone size known", dict(known=True)),
    ("5 observers, headings calibrated to 0.5 deg", dict(yaw_bias_deg=0.5)),
    ("5 observers, link 0.5 s late + bursty loss", dict(delay=0.5, bursty=True)),
    ("5 observers, ten times the false alarms", dict(clutter=0.2)),
    ("5 observers, 1280 px camera", dict(cam=POOR)),
    ("5 observers, any one camera may confirm", dict(tracker_kw=dict(confirm_cams=1))),
    ("5 observers, tracker not told about the bias", dict(assumed_bias_deg=0.0)),
    ("5 observers, matching on distance alone", dict(tracker_kw=dict(use_likelihood=False))),
]


def main(seeds=8):
    print("[stage 4] Five identical drones (0.77 m, 4 m/s, turning) tracked by a team's cameras; 3840 px / 90 deg unless stated")
    print("  heading bias 2 deg and the tracker told to expect it; size unknown (prior 0.45 +- 0.25); 0.02 false alarms per")
    print("  camera frame; ideal link; a track needs two cameras to count (one when there is one observer); unless stated")
    print(f"  {seeds} seeds x 60 s; mean over seeds [worst seed]")
    print(f"  {'configuration':45s} found        count ok   error m      swaps/drone/min  false tracks", flush=True)
    for name, kw in CONFIGS:
        kw = dict(kw)
        kw.setdefault("assumed_bias_deg", kw.get("yaw_bias_deg", 2.0))
        kw["tracker_kw"] = {"confirm_cams": 1 if kw.get("n_obs", 5) == 1 else 2, **kw.get("tracker_kw", {})}
        r = np.array([run(s, **kw) for s in range(seeds)])
        print(f"  {name:45s} {r[:, 0].mean() * 100:3.0f} % [{r[:, 0].min() * 100:3.0f}]  {r[:, 1].mean() * 100:3.0f} %      "
              f"{np.nanmean(r[:, 2]):4.2f} [{np.nanmax(r[:, 2]):4.1f}]  {r[:, 3].mean():4.2f} [{r[:, 3].max():4.1f}]      "
              f"{r[:, 4].mean():4.2f} [{r[:, 4].max():4.1f}]", flush=True)


if __name__ == "__main__":
    main()
