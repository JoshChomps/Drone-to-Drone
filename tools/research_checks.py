"""Reproduce the numbers in 02_research_survey_and_novel_directions.md (section references in brackets).

Pure numpy, no simulator, seeded. Run: python tools/research_checks.py   (about 1-2 min)

All of this is 2-D point-mass modelling. It ranks ideas and sets expectations; it does not replace the
PX4/Gazebo experiments (doc 01) or real detector outputs.
"""
import numpy as np

F_PX = 640 / np.tan(np.radians(45))  # focal length in px: 1280-px wide, 90-deg HFOV camera (doc 01 section 6)
SPAN = 0.77  # x500 envelope diameter 2r (doc 01 section 4.2), the box width seen from the side
DEG = np.pi / 180


# ----------------------------------------------------------------------------- [5.1] error budget
def budget():
    print("[5.1] Where is a teammate? One-sigma error by source (x500, 1280 px / 90 deg camera)")
    s_w, s_att, s_gps = 1.5, 1.0 * DEG, 1.0  # box-width noise px, attitude error, relative GNSS error m (assumptions)
    print(f"  assumptions: box width noise {s_w} px, attitude error {s_att / DEG:.1f} deg, relative GNSS {s_gps} m")
    print("  range | box px | vision range err | vision cross-range err | GNSS (any direction)")
    for r in [5, 10, 20, 40, 60, 100]:
        w = F_PX * SPAN / r
        print(f"  {r:4} m | {w:5.1f}  | {r * s_w / w:7.2f} m        | {r * np.hypot(1 / F_PX, s_att):6.2f} m"
              f"               | {s_gps:.1f} m")
    print(f"  vision range beats GNSS inside {np.sqrt(s_gps * F_PX * SPAN / s_w):.0f} m; "
          f"vision cross-range beats GNSS inside {s_gps / np.hypot(1 / F_PX, s_att):.0f} m")

    print("\n[5.2] Projecting a teammate's shared position into the image (the 'radio cue')")
    print("  range | box px | cue error px (GNSS 1 m + attitude 1 deg) | with RTK 0.02 m | cue window (3 sigma + box)")
    for r in [10, 25, 50, 100]:
        w = F_PX * SPAN / r
        e = F_PX * np.hypot(s_gps / r, s_att)
        e_rtk = F_PX * np.hypot(0.02 / r, s_att)
        print(f"  {r:4} m | {w:5.1f}  | {e:6.1f}                                   | {e_rtk:5.1f}           | {6 * e + w:5.0f} px")
    tiles_full = int(np.ceil((1280 - 64) / (640 - 64))) * int(np.ceil((720 - 64) / (640 - 64)))
    print(f"  full 1280x720 scan = {tiles_full} tiles of 640 px (64 px overlap); a cue is 1 tile per teammate."
          f" At 15 tiles/s (sibling doc 18, Pi 5): {15 / tiles_full:.1f} full scans/s, or 15 cued looks/s")


# ----------------------------------------------------------------------------- [5.3] heading from teammates
def heading_calibration(n_runs=4000):
    print("\n[5.3] Heading from seeing teammates whose GNSS positions are shared (one epoch per teammate)")
    rng = np.random.default_rng(1)
    s_b = np.hypot(1 / F_PX, 0.5 * DEG)  # body-frame bearing noise: 1 px + 0.5 deg roll/pitch-induced
    print("  baseline | 1 teammate | 2 teammates | 4 teammates   (RMS heading error, deg; relative GNSS 1 m per axis)")
    for r in [10, 30, 60]:
        row = []
        for n in [1, 2, 4]:
            err = np.zeros(n_runs)
            for k in range(n_runs):
                phi = rng.uniform(0, 2 * np.pi, n)
                u = np.stack([np.cos(phi), np.sin(phi)], 1)
                rel = r * u + rng.normal(0, 1.0, (n, 2))  # shared relative position with GNSS error
                est = np.arctan2(rel[:, 1], rel[:, 0]) - (phi + rng.normal(0, s_b, n))  # = heading error per teammate
                err[k] = np.arctan2(np.sin(est).mean(), np.cos(est).mean())
            row.append(np.sqrt((err ** 2).mean()) / DEG)
        print(f"  {r:5} m  | " + " | ".join(f"{x:8.2f}  " for x in row))
    print("  rule of thumb: sigma_heading ~ sigma_GNSS / (baseline * sqrt(N));  RTK (0.02 m) at 30 m: "
          f"{np.hypot(0.02 / 30, s_b) / DEG:.2f} deg (bearing noise dominates)")


# ----------------------------------------------------------------------------- [5.4] target motion estimation
def gilbert_elliott(rng, n, p_gb=0.02, p_bg=0.20):
    lost, state = np.zeros(n, bool), 0
    u = rng.random(n)
    for k in range(n):
        state = (1 if u[k] < p_gb else 0) if state == 0 else (0 if u[k] < p_bg else 1)
        lost[k] = state == 1
    return lost


class EKF:
    """State [px, py, vx, vy, l]: target position, velocity, physical width. Constant-velocity model."""

    def __init__(self, o, beta, theta, l0, s_l, r0=30.0, s_r=20.0):
        r = l0 / theta if theta is not None else r0
        s_r = r * 0.6 if theta is not None else s_r
        u = np.array([np.cos(beta), np.sin(beta)])
        n = np.array([-u[1], u[0]])
        self.x = np.array([*(o + r * u), 0.0, 0.0, l0])
        Pp = s_r ** 2 * np.outer(u, u) + 2.0 ** 2 * np.outer(n, n)
        self.P = np.zeros((5, 5))
        self.P[:2, :2], self.P[2:4, 2:4], self.P[4, 4] = Pp, 5.0 ** 2 * np.eye(2), s_l ** 2

    def predict(self, dt, s_a=1.0, s_ldot=0.0):
        Fm = np.eye(5)
        Fm[0, 2] = Fm[1, 3] = dt
        G = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x = Fm @ self.x
        self.P = Fm @ self.P @ Fm.T
        self.P[:4, :4] += s_a ** 2 * G @ G.T
        self.P[4, 4] += s_ldot ** 2 * dt

    def _upd(self, innov, H, R):
        S = H @ self.P @ H + R
        if innov ** 2 / S > 25:  # 5-sigma gate
            return
        K = self.P @ H / S
        self.x = self.x + K * innov
        self.P = (np.eye(5) - np.outer(K, H)) @ self.P
        self.x[4] = max(self.x[4], 0.05)

    def bearing(self, o, beta, R, back=0.0):
        """Bearing taken `back` seconds ago from o. back > 0 evaluates it on the state propagated backwards
        (exact for constant velocity), which is how a late packet should be used."""
        d = self.x[:2] - self.x[2:4] * back - o
        r2 = d @ d
        Hp = np.array([-d[1], d[0]]) / r2
        H = np.array([*Hp, *(-back * Hp), 0.0])
        innov = (beta - np.arctan2(d[1], d[0]) + np.pi) % (2 * np.pi) - np.pi
        self._upd(innov, H, R)

    def angle(self, o, theta, R):
        d = self.x[:2] - o
        r = np.linalg.norm(d)
        H = np.array([*(-self.x[4] * d / r ** 3), 0.0, 0.0, 1 / r])
        self._upd(theta - self.x[4] / r, H, R)


def tma_run(seed, use_angle, known_l, weave, n_obs, sep_deg=90.0, delay=0.0, bursty=False, retro=True,
            yaw_bias_deg=2.0, T=40.0, dt=0.1, standoff=25.0):
    rng = np.random.default_rng(seed)
    n = int(T / dt)
    l_true = 0.5 if known_l else rng.uniform(0.25, 0.8)
    pT, hd, w = np.zeros(2), rng.uniform(0, 2 * np.pi), 0.0
    ang = rng.uniform(0, 2 * np.pi) + np.array([0.0, sep_deg * DEG])[:n_obs]
    po = np.stack([standoff * 1.6 * np.array([np.cos(a), np.sin(a)]) for a in ang])  # start far, close in
    vo = np.zeros((n_obs, 2))
    yaw_bias = rng.normal(0, yaw_bias_deg * DEG, n_obs)  # heading bias per observer (magnetometer)
    gps_bias = np.vstack([np.zeros(2), rng.normal(0, 0.7, (1, 2))])[:n_obs]  # observer 2's shared position vs observer 1
    lost = gilbert_elliott(rng, n) if bursty else np.zeros(n, bool)
    nd = int(round(delay / dt))
    queue, ekf, errs = [], None, []
    s_px, s_w = 1.0, 1.5
    R_b = (s_px / F_PX) ** 2 + (0.5 * DEG) ** 2
    R_t = (s_w / F_PX) ** 2
    for k in range(n):
        t = k * dt
        if k % 40 == 0:
            w = rng.uniform(-0.25, 0.25)  # target turn rate, changes every 4 s (up to 1 m/s^2 lateral)
        hd += w * dt
        vT = 4.0 * np.array([np.cos(hd), np.sin(hd)])
        pT = pT + vT * dt
        for i in range(n_obs):  # observers: P control to a slot around the target, no feed-forward, accel-limited
            des = pT + standoff * np.array([np.cos(ang[i]), np.sin(ang[i])])
            cmd = 1.0 * (des - po[i])
            if weave:
                los = (pT - po[i]) / np.linalg.norm(pT - po[i])
                cmd = cmd + 3.0 * np.sin(2 * np.pi * t / 6.0) * np.array([-los[1], los[0]])
            sp = np.linalg.norm(cmd)
            cmd = cmd * min(1.0, 10.0 / max(sp, 1e-9))
            dv = cmd - vo[i]
            vo[i] = vo[i] + dv * min(1.0, 3.0 * dt / max(np.linalg.norm(dv), 1e-9))
            po[i] = po[i] + vo[i] * dt
        if ekf is not None:
            ekf.predict(dt)
        meas = []
        for i in range(n_obs):
            d = pT - po[i]
            r = np.linalg.norm(d)
            theta = 2 * np.arctan(l_true / (2 * r))
            if rng.random() > 0.95 / (1 + np.exp(-(F_PX * theta - 8.0) / 2.0)):  # detection probability by box width
                continue
            beta = np.arctan2(d[1], d[0]) + yaw_bias[i] + rng.normal(0, np.sqrt(R_b))
            meas.append((i, po[i] + gps_bias[i], beta, theta + rng.normal(0, np.sqrt(R_t)), k))
        for m in meas:
            if m[0] == 0:
                if ekf is None:
                    l0, s_l = (0.5, 1e-3) if known_l else (0.45, 0.25)
                    ekf = EKF(m[1], m[2], m[3] if use_angle else None, l0, s_l)
                ekf.bearing(m[1], m[2], R_b)
                if use_angle:
                    ekf.angle(m[1], m[3], R_t)
            elif not lost[k]:
                queue.append(m)
        while queue and queue[0][4] + nd <= k and ekf is not None:  # observer 2's bearings arrive over the link
            m = queue.pop(0)
            ekf.bearing(m[1], m[2], R_b + (0.7 / standoff) ** 2, back=(k - m[4]) * dt if retro else 0.0)
        if ekf is not None and t >= 10.0:
            errs.append(np.linalg.norm(ekf.x[:2] - pT))
    return np.sqrt(np.mean(np.square(errs))) if errs else np.nan


def tma():
    print("\n[5.4] Locating a manoeuvring drone (4 m/s, turns up to 1 m/s^2) from ~25 m; RMS position error after 10 s")
    print("  detector noise 1 px centre / 1.5 px width, heading bias 2 deg, P_d by box size; 60 seeds; median [90th pct] m")
    cfgs = [
        ("1 observer, bearing only, following", dict(use_angle=False, known_l=False, weave=False, n_obs=1)),
        ("1 observer, bearing only, weaving +-3 m/s", dict(use_angle=False, known_l=False, weave=True, n_obs=1)),
        ("1 observer, bearing + box size, size unknown", dict(use_angle=True, known_l=False, weave=False, n_obs=1)),
        ("1 observer, bearing + box size, unknown, weaving", dict(use_angle=True, known_l=False, weave=True, n_obs=1)),
        ("1 observer, bearing + box size, size KNOWN (teammate)", dict(use_angle=True, known_l=True, weave=False, n_obs=1)),
        ("  same, heading calibrated to 0.5 deg (5.3)", dict(use_angle=True, known_l=True, weave=False, n_obs=1, yaw_bias_deg=0.5)),
        ("2 observers 30 deg apart, bearings, ideal link", dict(use_angle=False, known_l=False, weave=False, n_obs=2, sep_deg=30)),
        ("2 observers 90 deg apart, bearings, ideal link", dict(use_angle=False, known_l=False, weave=False, n_obs=2)),
        ("2 observers 90 deg, bearings + box size (unknown)", dict(use_angle=True, known_l=False, weave=False, n_obs=2)),
        ("  same, headings calibrated to 0.5 deg (5.3)", dict(use_angle=True, known_l=False, weave=False, n_obs=2, yaw_bias_deg=0.5)),
        ("  same, link 0.5 s late, used as if fresh", dict(use_angle=True, known_l=False, weave=False, n_obs=2, delay=0.5, retro=False)),
        ("  same, link 0.5 s late, applied at its own time", dict(use_angle=True, known_l=False, weave=False, n_obs=2, delay=0.5)),
        ("  same, 0.5 s late + bursty loss (9 %), own time", dict(use_angle=True, known_l=False, weave=False, n_obs=2, delay=0.5, bursty=True)),
    ]
    for name, kw in cfgs:
        e = np.array([tma_run(s, **kw) for s in range(60)])
        print(f"  {name:52s} {np.nanmedian(e):6.2f} [{np.nanquantile(e, 0.9):6.2f}]")


# ----------------------------------------------------------------------------- [5.5] tag
D_STATIC = SPAN + 2 * np.sqrt(2) * 0.5  # 2r + 2*sqrt(2)*sigma (doc 01 section 4.2 without the speed terms)
D_KEEP = D_STATIC + 0.6  # plus a reaction allowance


def tag_run(seed, n_p, ratio, p_pol, e_pol, safety=True, L=100.0, v_e=5.0, a_max=3.0, r_tag=6.0, hold=1.0, T=120.0, dt=0.1):
    """Everyone is acceleration-limited. Safety is mutual and sits under the game: each drone caps its own speed
    towards any other drone at sqrt(2 * 0.7 a * (d - D_KEEP) / 2), i.e. it can always stop inside its half of the gap."""
    rng = np.random.default_rng(seed)
    g = np.arange(1.25, L, 2.5)
    grid = np.stack(np.meshgrid(g, g), -1).reshape(-1, 2)
    hdg = np.arange(24) * 2 * np.pi / 24
    dirs = np.stack([np.cos(hdg), np.sin(hdg)], 1)
    x = np.zeros((n_p + 1, 2))  # row 0 is the evader
    x[0] = rng.uniform(20, 80, 2)
    x[1:] = rng.uniform(5, 95, (n_p, 2))
    while np.min(np.linalg.norm(x[1:] - x[0], axis=1)) < 25:
        x[1:] = rng.uniform(5, 95, (n_p, 2))
    v = np.zeros_like(x)
    vmax = np.array([v_e] + [ratio * v_e] * n_p)
    held, dmin, rel_at_tag = 0.0, 1e9, []

    def unit(u):
        return u / max(np.linalg.norm(u), 1e-9)

    for k in range(int(T / dt)):
        e, p = x[0], x[1:]
        cmd = np.zeros_like(x)
        if e_pol == "lookahead":  # the heading that keeps the nearest pursuer furthest away 1.5, 3 and 6 s ahead
            score = 0.3 * dirs @ unit(v[0])
            for h in (1.5, 3.0, 6.0):
                raw = e + v_e * h * dirs
                fut = np.clip(raw, 2, L - 2)
                gap = np.linalg.norm(fut[:, None] - p[None], axis=2) - vmax[1] * h  # pursuers run straight at that point
                score = score + np.maximum(gap.min(1), -5) / h - 3.0 * (np.abs(fut - raw).sum(1) > 0)
            cmd[0] = v_e * dirs[np.argmax(score)]
        else:  # flee: inverse-square repulsion from pursuers and walls
            f = sum((e - q) / np.linalg.norm(e - q) ** 3 for q in p)
            f = f + np.array([1 / max(e[0], 0.5) ** 2 - 1 / max(L - e[0], 0.5) ** 2,
                              1 / max(e[1], 0.5) ** 2 - 1 / max(L - e[1], 0.5) ** 2])
            cmd[0] = v_e * unit(f)
        if p_pol == "area-min":
            te = np.linalg.norm(grid - e, axis=1) / v_e
            tp = np.linalg.norm(grid[:, None] - p[None], axis=2) / vmax[1]
        for i in range(n_p):
            if p_pol == "pursuit":
                aim = e
            elif p_pol == "lead":  # aim where the evader will be when I could get there (constant-velocity guess)
                aim = np.clip(e + v[0] * np.linalg.norm(e - p[i]) / vmax[1], 0, L)
            else:  # area minimisation: the middle of the border between my cell and the evader's cell
                border = (np.abs(te - tp[:, i]) < 0.35) & (tp[:, i] <= tp.min(1) + 1e-9)
                aim = grid[border].mean(0) if border.any() and np.linalg.norm(e - p[i]) > 2 * r_tag else e
            to = aim - p[i]
            cmd[i + 1] = v[0] + 1.0 * to if (aim is e and np.linalg.norm(to) < r_tag) else vmax[i + 1] * unit(to)
            sp = np.linalg.norm(cmd[i + 1])
            cmd[i + 1] *= min(1.0, vmax[i + 1] / max(sp, 1e-9))
        if safety:
            for i in range(n_p + 1):
                for j in range(n_p + 1):
                    if i != j:
                        u = unit(x[j] - x[i])
                        cap = np.sqrt(2 * 0.7 * a_max * max(np.linalg.norm(x[j] - x[i]) - D_KEEP, 0.0) / 2)
                        if cmd[i] @ u > cap:
                            cmd[i] = cmd[i] - (cmd[i] @ u - cap) * u
        dv = cmd - v
        v = v + dv * np.minimum(1.0, a_max * dt / np.maximum(np.linalg.norm(dv, axis=1, keepdims=True), 1e-9))
        x = np.clip(x + v * dt, 0, L)
        dist = np.linalg.norm(x[:, None] - x[None], axis=2) + 1e9 * np.eye(n_p + 1)
        dmin = min(dmin, dist.min())
        near = dist[0, 1:].argmin()
        if dist[0, 1 + near] < r_tag:
            held += dt
            rel_at_tag.append(np.linalg.norm(v[1 + near] - v[0]))
        else:
            held, rel_at_tag = 0.0, []
        if held >= hold:
            return (k + 1) * dt, dmin, float(np.mean(rel_at_tag))
    return np.inf, dmin, np.nan


def tag():
    print("\n[5.5] Tag in a 100 m square: evader 5 m/s, everyone limited to 3 m/s^2; tag = within 6 m for 1 s")
    print(f"  static safety distance {D_STATIC:.2f} m (contact {SPAN:.2f} m); mutual braking rule keeps each drone able to stop")
    print("  40 seeds, 120 s limit: tag rate, median / 90th-pct time to tag (untagged = 120 s), closest pair distance over all runs,")
    print("  median relative speed during the tag second")
    print("  pursuers x speed | pursuer policy | evader    | safety | tagged | median t | p90 t  | min dist | rel speed")
    rows = [(1, 1.0, True), (1, 1.2, True), (2, 1.0, True), (3, 1.0, True), (2, 0.8, True), (3, 0.8, True), (2, 1.0, False)]
    for n_p, ratio, safe in rows:
        for e_pol in ["flee", "lookahead"]:
            for p_pol in ["pursuit", "lead", "area-min"]:
                res = [tag_run(s, n_p, ratio, p_pol, e_pol, safety=safe) for s in range(40)]
                t = np.minimum(np.array([r[0] for r in res]), 120.0)
                rel = np.array([r[2] for r in res])
                print(f"  {n_p} x {ratio:.1f}          | {p_pol:8s}       | {e_pol:9s} | {'on ' if safe else 'OFF'}    | "
                      f"{(t < 120).mean() * 100:4.0f} %  | {np.median(t):6.1f}   | {np.quantile(t, 0.9):6.1f} | "
                      f"{min(r[1] for r in res):6.2f}   | {np.nanmedian(rel) if np.isfinite(rel).any() else np.nan:5.1f}")


if __name__ == "__main__":
    budget()
    heading_calibration()
    tma()
    tag()
