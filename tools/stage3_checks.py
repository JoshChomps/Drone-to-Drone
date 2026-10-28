"""Stage 3 check (03_build_plan.md): locating a drone that shares nothing, in 3-D, with d2d.geometry and d2d.estimate.

Observers fly pre-planned paths (hover, straight leg, orbit) at 60 m and turn in place to keep the other drone in a
fixed forward camera. The other drone wanders at 4 m/s around 50 m. Run: python tools/stage3_checks.py   (~4 min)

Modelled: pinhole camera, box from the drone's true extent, pixel noise, detection probability by box width, field of
view, heading bias, roll/pitch noise, a teammate's position error, link delay and bursty loss, re-acquisition.
Not modelled: image rendering, a real detector's correlated errors, gaze control (the observer yaws towards the true
direction, rate-limited), banking, wind. Stage 6 replaces the detector model with measured numbers.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from d2d.estimate import IMMFilter, TargetFilter  # noqa: E402
from d2d.geometry import Camera, attitude  # noqa: E402

DEG = np.pi / 180
CAMERAS = {"1280 px, 90 deg": Camera(1280, 720, 90.0, 15.0), "3840 px, 90 deg": Camera(3840, 2160, 90.0, 15.0)}
W_MIN = 8.0  # box width (px) at which half the detections are lost; the sibling project's measured w_min


def observer_path(kind, centre, t):
    if kind == "hover":
        return centre
    if kind == "leg":  # straight and level at 3 m/s through the centre
        return centre + np.array([0.0, 3.0 * (t - 20.0), 0.0])
    w = 5.0 / 20.0  # orbit: 20 m radius at 5 m/s, 1.25 m/s^2
    return centre + 20.0 * np.array([np.cos(w * t), np.sin(w * t), 0.0])


def gilbert_elliott(rng, n, p_gb=0.02, p_bg=0.20):
    lost, state, u = np.zeros(n, bool), 0, rng.random(n)
    for k in range(n):
        state = (1 if u[k] < p_gb else 0) if state == 0 else (0 if u[k] < p_bg else 1)
        lost[k] = state == 1
    return lost


def run(seed, cam, paths, use_angle=True, known=False, sep_deg=90.0, yaw_bias_deg=2.0, delay=0.0, own_time=True,
        bursty=False, lost_after=5, noise=1.0, accel_sigma=3.0, turning=True, T=40.0, dt=0.1, make_filter=TargetFilter,
        motion="wander"):
    """Returns RMS position error over all time, RMS position and velocity error while seen in the last second,
    the share of frames in which observer 1 detected the drone, and the number of re-acquisitions.

    make_filter: TargetFilter (constant velocity with accel_sigma) or IMMFilter. motion: "wander" (the stage 3 drone:
    4 m/s, turn rate changing every 4 s within +-0.25 rad/s, 0.4 rad/s back towards the middle) or "legs" (a different
    drone for validation: 6 m/s straight legs of 3-8 s, then a 1 rad/s turn of 60-150 deg, 6 m/s^2)."""
    rng = np.random.default_rng(seed)
    n, n_obs = int(T / dt), len(paths)
    s_bearing, s_angle = np.hypot(1.0 / cam.f, 0.5 * DEG), 1.5 / cam.f  # 1 px centre + 0.5 deg roll/pitch; 1.5 px width
    span = 0.5 if known else rng.uniform(0.25, 0.8)
    a = np.pi + np.array([0.0, sep_deg * DEG])
    centres = [np.array([35 * np.cos(x), 35 * np.sin(x), 60.0]) for x in a]
    pT = np.array([*rng.uniform(-15, 15, 2), 50.0])
    hd, w = rng.uniform(0, 2 * np.pi), 0.0
    yaw = [None] * n_obs
    yaw_bias = rng.normal(0, yaw_bias_deg * DEG, n_obs)
    pos_bias = [np.zeros(3), noise * rng.normal(0, 0.7, 3)]  # observer 2's shared position, relative to observer 1
    lost = gilbert_elliott(rng, n) if bursty else np.zeros(n, bool)
    nd = int(round(delay / dt))
    f, queue, err, err_seen, verr_seen, seen, reacq, last = None, [], [], [], [], 0, 0, -10 ** 9
    for k in range(n):
        t = k * dt
        if motion == "legs":
            if k == 0:
                leg, turn_left = rng.uniform(3, 8), 0.0
            if turn_left > 0:  # turning
                turn_left -= abs(w) * dt
                if turn_left <= 0:
                    w, leg = 0.0, rng.uniform(3, 8)
            else:
                leg -= dt
                if leg <= 0 or (np.hypot(*pT[:2]) > 30 and np.cos(hd) * pT[0] + np.sin(hd) * pT[1] > 0):
                    toward = np.sign(-np.cos(hd) * pT[1] + np.sin(hd) * pT[0])  # +1: the middle is to the left
                    w = 1.0 * (toward if np.hypot(*pT[:2]) > 20 else rng.choice([-1.0, 1.0]))
                    turn_left = rng.uniform(60, 150) * DEG
            speed = 6.0
        else:
            if k % 40 == 0:
                w = rng.uniform(-0.25, 0.25) if turning else 0.0
            if turning and np.hypot(*pT[:2]) > 30:  # stay in the area: turn back towards the middle
                w = 0.4 * np.sign(-np.cos(hd) * pT[1] + np.sin(hd) * pT[0])
            speed = 4.0
        hd += w * dt
        vT = np.array([speed * np.cos(hd), speed * np.sin(hd), 0.3 * np.cos(0.3 * t)])
        pT = pT + vT * dt
        if f is not None:
            f.predict(dt, accel_sigma)
        for i in range(n_obs):
            o = observer_path(paths[i], centres[i], t)
            want = np.arctan2(pT[1] - o[1], pT[0] - o[0])
            if yaw[i] is None:
                yaw[i] = want
            yaw[i] += np.clip((want - yaw[i] + np.pi) % (2 * np.pi) - np.pi, -60 * DEG * dt, 60 * DEG * dt)
            box = cam.box(o, attitude(yaw[i]), pT, span, 0.33 * span)
            if box is None or not cam.in_view(np.array([[(box[0] + box[2]) / 2, (box[1] + box[3]) / 2]]), np.ones(1))[0]:
                continue
            if noise > 0 and rng.random() > 0.95 / (1 + np.exp(-((box[2] - box[0]) - W_MIN) / 2.0)):  # missed: too few pixels
                continue
            seen += i == 0
            box = box + noise * (np.array([-1, 0, 1, 0]) * rng.normal(0, 1.5 / 2) + np.repeat(rng.normal(0, 1.0, 2), 2)[[0, 2, 1, 3]])
            r_est = attitude(yaw[i] + yaw_bias[i], noise * rng.normal(0, 0.5 * DEG), noise * rng.normal(0, 0.5 * DEG))
            g = cam.bearing(r_est, [(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])[0]
            m = (o + pos_bias[i], g, cam.subtended_angle(box), k)
            if i == 0:
                last = k
                if f is None:
                    f = make_filter(m[0], g, m[2] if use_angle else None, *((0.5, 1e-3) if known else (0.45, 0.25)))
                    continue
                f.update_bearing(m[0], g, s_bearing)
                if f.rejected >= lost_after:  # lost: it was out of view and is not where the filter expected
                    f.reacquire(m[0], g, m[2] if use_angle else None)
                    reacq += 1
                if use_angle:
                    f.update_angle(m[0], m[2], s_angle)
            elif not lost[k]:
                queue.append(m)
        while queue and queue[0][3] + nd <= k and f is not None:  # the teammate's sightings arrive over the link
            o2, g, th, k0 = queue.pop(0)
            back = (k - k0) * dt if own_time else 0.0
            last = k
            f.update_bearing(o2, g, np.hypot(s_bearing, 0.7 / 40), back)
            if use_angle:
                f.update_angle(o2, th, s_angle, back)
        if f is not None and t >= 10.0:
            err.append(np.linalg.norm(f.position - pT))
            if k - last <= 10:
                err_seen.append(err[-1])
                verr_seen.append(np.linalg.norm(f.velocity - vT))
    rms = lambda e: np.sqrt(np.mean(np.square(e))) if e else np.nan
    return rms(err), rms(err_seen), rms(verr_seen), seen / n, reacq


CONFIGS = [
    ("1 orbiting, bearing + box size, NO re-acquisition", dict(paths=["orbit"], lost_after=10 ** 9)),
    ("1 hovering, bearing only", dict(paths=["hover"], use_angle=False)),
    ("1 hovering, bearing + box size", dict(paths=["hover"])),
    ("1 straight leg at 3 m/s, bearing + box size", dict(paths=["leg"])),
    ("1 orbiting, bearing only", dict(paths=["orbit"], use_angle=False)),
    ("1 orbiting, bearing + box size", dict(paths=["orbit"])),
    ("1 hovering, bearing + box size, size KNOWN", dict(paths=["hover"], known=True)),
    ("2 hovering 30 deg apart, bearing + box size", dict(paths=["hover", "hover"], sep_deg=30)),
    ("2 hovering 90 deg apart, bearing only", dict(paths=["hover", "hover"], use_angle=False)),
    ("2 hovering 90 deg apart, bearing + box size", dict(paths=["hover", "hover"])),
    ("2 orbiting 90 deg apart, bearing + box size", dict(paths=["orbit", "orbit"])),
    ("  same, headings calibrated to 0.5 deg", dict(paths=["orbit", "orbit"], yaw_bias_deg=0.5)),
    ("  same, motion model with less freedom (1 m/s^2)", dict(paths=["orbit", "orbit"], accel_sigma=1.0)),
    ("  same, link 0.5 s late, used as if fresh", dict(paths=["orbit", "orbit"], delay=0.5, own_time=False)),
    ("  same, link 0.5 s late, used at its own time", dict(paths=["orbit", "orbit"], delay=0.5)),
    ("  same, 0.5 s late + bursty loss (9 %), own time", dict(paths=["orbit", "orbit"], delay=0.5, bursty=True)),
]


def main():
    print("[stage 3] Locating a silent drone (4 m/s, turning, ~50 m altitude) from observers at 60 m, 15-70 m away")
    print("  heading bias 2 deg, roll/pitch noise 0.5 deg, box noise 1 px centre / 1.5 px width; size unknown")
    print("  (0.25-0.8 m, prior 0.45 +- 0.25) unless stated; 40 seeds; RMS after 10 s, median [90th pct]")
    print("  constant-velocity model with 3 m/s^2 acceleration noise (chosen on this scenario; 1 m/s^2 lags a turning drone)")
    print("  pipeline check, straight-flying drone, no noise, 20 s:", end="")
    for cfg, kw in [("1 hovering, size known", dict(paths=["hover"], known=True)), ("2 hovering, bearing only", dict(paths=["hover", "hover"], use_angle=False))]:
        r = np.array([run(s, CAMERAS["3840 px, 90 deg"], noise=0.0, yaw_bias_deg=0.0, turning=False, accel_sigma=1.0, T=20.0, **kw) for s in range(20)])
        print(f"  {cfg}: {np.nanmedian(r[:, 1]):.2f} m, {np.nanmedian(r[:, 2]):.2f} m/s;", end="")
    print()
    for name, cam in CAMERAS.items():
        print(f"\n  camera {name}, tilted 15 deg down (f = {cam.f:.0f} px; a 0.5 m drone is {W_MIN:.0f} px wide at {cam.f * 0.5 / W_MIN:.0f} m)")
        print(f"  {'configuration':52s} pos, all time   pos, while seen  vel, while seen  seen  re-acq")
        for cfg, kw in CONFIGS:
            r = np.array([run(s, cam, **kw) for s in range(40)])
            q = lambda c, p: np.nanquantile(r[:, c], p)
            print(f"  {cfg:52s} {q(0, .5):5.1f} [{q(0, .9):5.1f}]   {q(1, .5):5.2f} [{q(1, .9):5.1f}]    "
                  f"{q(2, .5):4.2f} [{q(2, .9):4.1f}]     {r[:, 3].mean() * 100:3.0f} %  {r[:, 4].mean():.1f}")


if __name__ == "__main__":
    main()
