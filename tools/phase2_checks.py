"""Reproduce the numbers in 01_phase2_simulation_plan.md (section references in brackets).

Pure numpy, no simulator. Run: python tools/phase2_checks.py   (about 1-2 min, the CBF grid is the slow part)
"""
import numpy as np

rng = np.random.default_rng(0)
R_BODY = np.hypot(0.174, 0.174) + 0.2792307692 / 2  # x500 rotor-hub radius + prop radius, PX4-gazebo-models x500_base SDF


def delayed_p_loop(K, tau, dt=0.001, T=60.0):
    """x' = K (1 - x(t - tau)); returns the late-time max |error|. Stable iff K*tau < pi/2."""
    n, d = int(T / dt), int(round(tau / dt))
    x = np.zeros(n)
    for k in range(1, n):
        x[k] = x[k - 1] + dt * K * (1.0 - (x[k - 1 - d] if k - 1 - d >= 0 else 0.0))
    return np.abs(x[-5000:] - 1).max()


def mean_age(lost, T):
    """Time-average age of information for a periodic stream with losses (sawtooth area / time)."""
    gaps = np.diff(np.nonzero(~lost)[0] * T)
    return (gaps ** 2).sum() / (2 * gaps.sum()), np.quantile(gaps, 0.999), gaps.max()


def gilbert_elliott(n, p_gb, p_bg):
    lost, state, u = np.zeros(n, bool), 0, rng.random(n)
    for k in range(n):
        state = (1 if u[k] < p_gb else 0) if state == 0 else (0 if u[k] < p_bg else 1)
        lost[k] = state == 1
    return lost


def cbf_head_on(v_nom=5.0, Ds=3.0, gamma=1.0, avoid=True, a_max=None, stale=0.0, predict=False, inflate=0.0, dt=0.005, T=25):
    """Two drones swap ends head-on (closing 2*v_nom). Decentralised reciprocal CBF, single-integrator form
    (Wang, Ames & Egerstedt 2017): h = |dp|^2 - D^2, each agent enforces 2 dp.v_i >= -gamma h / 2.
    Neighbour position is `stale` seconds old (optionally propagated at its old velocity). Returns d_min."""
    p = np.array([[-40.0, 0.25], [40.0, -0.25]])
    goal = np.array([[40.0, 0.25], [-40.0, -0.25]])
    v, hist, nd, dmin = np.zeros((2, 2)), [], int(round(stale / dt)), 1e9
    for _ in range(int(T / dt)):
        hist.append((p.copy(), v.copy()))
        pb, vb = hist[max(0, len(hist) - 1 - nd)]
        cmd = np.zeros((2, 2))
        for i in range(2):
            j = 1 - i
            u = goal[i] - p[i]
            u = u / max(np.linalg.norm(u), 1e-9) * min(v_nom, np.linalg.norm(u))
            if avoid:
                dp = p[i] - (pb[j] + (vb[j] * stale if predict else 0))
                a, b = 2 * dp, -gamma * (dp @ dp - (Ds + inflate) ** 2) / 2
                if a @ u < b:
                    u = u + (b - a @ u) / (a @ a) * a
            cmd[i] = u
        if a_max is None:
            v = cmd
        else:
            dv = cmd - v
            v = v + dv * np.minimum(1, a_max * dt / np.maximum(np.linalg.norm(dv, axis=1, keepdims=True), 1e-9))
        p = p + v * dt
        dmin = min(dmin, np.linalg.norm(p[0] - p[1]))
    return dmin


def main():
    print("[4.1] Delayed P loop, stable iff K*tau < pi/2 = 1.571")
    for K, tau in [(1, 0.5), (1, 1.0), (1, 1.5), (1, 1.6), (2, 0.5), (2, 0.8)]:
        print(f"  K={K} tau={tau}  K*tau={K * tau:.2f}  late |err| = {delayed_p_loop(K, tau):.2g}")

    print("\n[4.1] Leader on a circle R=20 m at 5 m/s; follower uses leader state tau seconds old")
    R, v = 20.0, 5.0
    w, acc = v / R, v * v / R
    t = np.arange(0, 60, 0.01)
    pos = lambda t: np.stack([R * np.cos(w * t), R * np.sin(w * t)], -1)
    vel = lambda t: np.stack([-v * np.sin(w * t), v * np.cos(w * t)], -1)
    for tau in [0.1, 0.25, 0.5, 1.0]:
        e0 = np.linalg.norm(pos(t - tau) - pos(t), axis=1).mean()
        e1 = np.linalg.norm(pos(t - tau) + vel(t - tau) * tau - pos(t), axis=1).mean()
        print(f"  tau={tau:4}: stale {e0:.2f} m (v*tau {v * tau:.2f}); predicted {e1:.3f} m (a*tau^2/2 {acc * tau * tau / 2:.3f})")

    print("\n[4.1] Ramp tracking with and without velocity feed-forward (K=1, 5 m/s)")
    for ff in [True, False]:
        p, dt = 0.0, 0.001
        for k in range(30000):
            p += dt * np.clip((5.0 if ff else 0.0) + (5.0 * k * dt - p), -12, 12)
        print(f"  feed-forward={ff}: steady error {5.0 * 30 - p:.2f} m (v/K = 5.0 without)")

    print("\n[3.3] Age of information, 10 Hz state stream (no fixed delay added)")
    T, n = 0.1, 500_000
    for p in [0.0, 0.1, 0.3, 0.5]:
        m, q, mx = mean_age(rng.random(n) < p, T)
        print(f"  Bernoulli p={p}: mean age {m * 1e3:.0f} ms (theory {T * (1 + p) / (2 * (1 - p)) * 1e3:.0f}); 99.9th pct gap {q:.1f} s; max {mx:.1f} s")
    lost = gilbert_elliott(n, 0.02, 0.20)
    m, q, mx = mean_age(lost, T)
    print(f"  Gilbert-Elliott (G->B 2%, B->G 20%): loss {lost.mean():.3f}; mean age {m * 1e3:.0f} ms; 99.9th pct gap {q:.1f} s; max {mx:.1f} s")

    print(f"\n[4.2] Safety distance 2r + 2*sqrt(2)*sigma + v_c*(tau + 0.2 s) + v_c^2/(4a); r = {R_BODY:.3f} m, sigma 0.5 m, a 3 m/s^2")
    for vc in [2, 5, 10]:
        ds = [2 * R_BODY + 2 * np.sqrt(2) * 0.5 + vc * (tau + 0.2) + vc * vc / 12 for tau in (0.1, 0.5, 1.0)]
        print(f"  v_close={vc:2} m/s: " + "  ".join(f"tau={tau}: {d:4.1f} m" for tau, d in zip((0.1, 0.5, 1.0), ds)))

    print("\n[4.2] CBF head-on, 5 m/s each, accel limit 3 m/s^2, Ds 3 m  (* below Ds, ! contact < 2r)")
    mark = lambda d: f"{d:5.2f}{'!' if d < 2 * R_BODY else ('*' if d < 3 - 1e-6 else ' ')}"
    print(f"  no avoidance: {mark(cbf_head_on(avoid=False))}   CBF, ideal tracking: {mark(cbf_head_on())}")
    print("  gamma stale | plain  | predict | predict + inflate v*tau")
    for g in [0.5, 1.0, 3.0]:
        for s in [0.0, 0.5, 1.0]:
            row = [cbf_head_on(a_max=3.0, gamma=g, stale=s, **kw) for kw in (dict(), dict(predict=True), dict(predict=True, inflate=5.0 * s))]
            print(f"  {g:4}  {s:4}  | " + " | ".join(mark(d) for d in row))

    print("\n[3.3] lat/lon -> local metres at 50 N: flat-earth with ellipsoid radii vs exact ECEF->ENU")
    A, F = 6378137.0, 1 / 298.257223563
    E2 = F * (2 - F)
    lat0 = np.radians(50.0)
    M = A * (1 - E2) / (1 - E2 * np.sin(lat0) ** 2) ** 1.5
    N = A / np.sqrt(1 - E2 * np.sin(lat0) ** 2)

    def ecef(lat, lon):
        n_ = A / np.sqrt(1 - E2 * np.sin(lat) ** 2)
        return np.array([n_ * np.cos(lat) * np.cos(lon), n_ * np.cos(lat) * np.sin(lon), n_ * (1 - E2) * np.sin(lat)])

    lon0 = np.radians(-120.0)
    Rm = np.array([[-np.sin(lon0), np.cos(lon0), 0],
                   [-np.sin(lat0) * np.cos(lon0), -np.sin(lat0) * np.sin(lon0), np.cos(lat0)]])
    for dist in [100, 1000, 5000]:
        dlat, dlon = dist / np.sqrt(2) / M, dist / np.sqrt(2) / (N * np.cos(lat0))
        exact = Rm @ (ecef(lat0 + dlat, lon0 + dlon) - ecef(lat0, lon0))
        flat = np.array([dlon * N * np.cos(lat0), dlat * M])
        sph = np.array([dlon * 6371000 * np.cos(lat0), dlat * 6371000])
        print(f"  {dist:5} m: ellipsoid radii {np.linalg.norm(flat - exact) * 100:.1f} cm; sphere 6371 km {np.linalg.norm(sph - exact):.2f} m")

    print("\n[4.3] Coverage of 1 km x 1 km, camera 60 m AGL, HFOV 45 deg, 8 m/s")
    swath = 2 * 60 * np.tan(np.radians(22.5))
    legs = int(np.ceil(1000 / swath))
    L = legs * 1000 + (legs - 1) * swath
    for n_ in [1, 2, 3]:
        print(f"  N={n_}: swath {swath:.1f} m, {legs} legs, {L / n_ / 1000:.1f} km and {L / n_ / 8 / 60:.1f} min per drone (turns not slowed)")

    print("\n[3.3] Shared-state bandwidth")
    payload = 4 + 3 * 4 + 3 * 2 + 2 + 2 + 2 + 12  # time, lat/lon/alt, vNED (int16 cm/s), yaw, flags, seq, goal lat/lon/alt
    pkt = payload + 10 + 2  # MAVLink 2 header + CRC, unsigned
    for n_, hz in [(3, 10), (5, 10), (3, 20)]:
        print(f"  {payload} B payload, {pkt} B packet; N={n_} at {hz} Hz: {n_ * hz * pkt * 8 / 1000:.1f} kbit/s")

    print("\n[6] Pixels on target for an x500 (span 2r) seen by a 1280-px, 90-deg HFOV camera")
    f = 640 / np.tan(np.radians(45))
    print("  " + "  ".join(f"{z} m: {f * 2 * R_BODY / z:.1f} px" for z in (25, 50, 100)) + f";  8 px (sibling E2 w_min) at {f * 2 * R_BODY / 8:.0f} m")


if __name__ == "__main__":
    main()
