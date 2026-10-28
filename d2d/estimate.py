"""Locating another drone from camera detections (build stage 3). numpy only, world frame ENU.

One filter per tracked drone. State x = [p (3), v (3), l]: position, velocity and physical width, constant-velocity
model. Measurements, from any observer and at any age:
  bearing : unit vector from the observer to the target (Camera.bearing of the box centre)
  angle   : the angle the box width subtends (Camera.subtended_angle); it ties range to l (Ning et al., IJRR 2024)
A measurement taken `back` seconds ago is evaluated on the state propagated backwards by `back`, which is exact for
constant velocity. That is how a late packet from a teammate should be used (doc 02 section 5.4).
"""
import numpy as np


def triangulate(origins, bearings):
    """Least-squares point closest to several rays (origins (N, 3), unit bearings (N, 3)). Needs non-parallel rays."""
    origins, bearings = np.atleast_2d(origins), np.atleast_2d(bearings)
    proj = np.eye(3) - bearings[:, :, None] * bearings[:, None, :]  # projector onto the plane normal to each ray
    return np.linalg.solve(proj.sum(0), (proj @ origins[:, :, None]).sum(0)).ravel()


def bearing_model(x, o, back=0.0):
    """Predicted direction u, range r, a 2x3 basis E of the plane normal to u, and the 2x7 Jacobian of E @ u(x)."""
    d = x[:3] - back * x[3:6] - o
    r = np.linalg.norm(d)
    u = d / r
    # e1 = u x z (or u x x when u is near vertical), e2 = u x e1; written out because np.cross is slow for single vectors
    e1 = np.array([u[1], -u[0], 0.0]) if abs(u[2]) < 0.9 else np.array([0.0, u[2], -u[1]])
    e1 /= np.sqrt(e1 @ e1)
    E = np.array([e1, [u[1] * e1[2] - u[2] * e1[1], u[2] * e1[0] - u[0] * e1[2], u[0] * e1[1] - u[1] * e1[0]]])
    return u, r, E, np.hstack([E / r, -back * E / r, np.zeros((2, 1))])


def angle_model(x, o, back=0.0):
    """Predicted subtended angle 2 atan(l / 2r) and its 1x7 Jacobian."""
    d = x[:3] - back * x[3:6] - o
    r, l = np.linalg.norm(d), x[6]
    k = r * r + l * l / 4
    hp = -l / k * d / r
    return 2 * np.arctan(l / (2 * r)), np.array([*hp, *(-back * hp), r / k])


class TargetFilter:
    """Extended Kalman filter for one drone seen by one or more cameras."""

    def __init__(self, o, g, theta=None, size=0.45, size_sigma=0.25, range_guess=40.0, range_sigma=30.0):
        """Start from one bearing g taken at o. With an angle, the size prior sets the first range
        (0.45 +- 0.25 m is 'a small multirotor'); pass the true width and a tiny sigma for a known teammate."""
        r, s_r = (size / (2 * np.tan(theta / 2)), None) if theta is not None else (range_guess, range_sigma)
        if s_r is None:
            s_r = r * max(size_sigma / size, 0.05)
        self.x = np.array([*(np.asarray(o, float) + r * g), 0.0, 0.0, 0.0, size])
        self.P = np.zeros((7, 7))
        self.P[:3, :3] = s_r ** 2 * np.outer(g, g) + (0.05 * r) ** 2 * (np.eye(3) - np.outer(g, g))
        self.P[3:6, 3:6] = 5.0 ** 2 * np.eye(3)
        self.P[6, 6] = size_sigma ** 2
        self.rejected = 0  # bearings gated out in a row; the caller decides when that means the track is lost

    def reacquire(self, o, g, theta=None):
        """The drone was lost (out of view, then somewhere unexpected). Restart position and velocity from a new
        sighting, keeping what was learned about its size. Without an angle the old range is reused, loosely."""
        o = np.asarray(o, float)
        size, s_size = self.x[6], np.sqrt(self.P[6, 6])
        r = size / (2 * np.tan(theta / 2)) if theta is not None else np.linalg.norm(self.x[:3] - o)
        s_r = r * max(s_size / size, 0.1) if theta is not None else 0.5 * r
        self.x[:6] = [*(o + r * g), 0.0, 0.0, 0.0]
        self.P[:6] = 0.0
        self.P[:, :6] = 0.0
        self.P[:3, :3] = s_r ** 2 * np.outer(g, g) + (0.05 * r) ** 2 * (np.eye(3) - np.outer(g, g))
        self.P[3:6, 3:6] = 5.0 ** 2 * np.eye(3)
        self.rejected = 0

    def predict(self, dt, accel_sigma=1.0, size_rate_sigma=0.0):
        F = np.eye(7)
        F[:3, 3:6] = dt * np.eye(3)
        G = np.vstack([dt * dt / 2 * np.eye(3), dt * np.eye(3), np.zeros((1, 3))])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + accel_sigma ** 2 * G @ G.T
        self.P[6, 6] += size_rate_sigma ** 2 * dt

    def _update(self, y, H, R, gate):
        S = H @ self.P @ H.T + R
        if y @ np.linalg.solve(S, y) > gate:
            return False
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        A = np.eye(7) - K @ H
        self.P = A @ self.P @ A.T + K @ R @ K.T
        self.x[6] = max(self.x[6], 0.05)
        return True

    def update_bearing(self, o, g, sigma, back=0.0, gate=16.0):
        """Bearing g (unit vector) from o, angular noise sigma (rad), taken `back` seconds ago. False if gated out."""
        _, _, E, H = bearing_model(self.x, np.asarray(o, float), back)
        ok = self._update(E @ g, H, sigma ** 2 * np.eye(2), gate)
        self.rejected = 0 if ok else self.rejected + 1
        return ok

    def update_angle(self, o, theta, sigma, back=0.0, gate=25.0):
        """Subtended angle theta (rad) seen from o, noise sigma (rad), taken `back` seconds ago."""
        pred, H = angle_model(self.x, np.asarray(o, float), back)
        return self._update(np.array([theta - pred]), H[None], np.array([[sigma ** 2]]), gate)

    def distance(self, o, g, sigma, theta=None, theta_sigma=None, back=0.0):
        """How far a sighting is from what this track predicts. Returns (d2, penalty): d2 is the squared Mahalanobis
        distance (2 degrees of freedom, 3 with an angle), used for the gate; penalty is the log-determinant of the
        innovation covariance. d2 + penalty is twice the negative log-likelihood up to a constant, and is what
        sightings should be matched on: d2 alone favours vague tracks, because anything is 'close' to them."""
        o = np.asarray(o, float)
        u, _, E, H = bearing_model(self.x, o, back)
        if u @ g < 0.5:  # more than 60 degrees off: the small-angle residual below would be meaningless
            return np.inf, 0.0
        y = E @ g
        S = H @ self.P @ H.T + sigma ** 2 * np.eye(2)
        d2, pen = y @ np.linalg.solve(S, y), np.log(S[0, 0] * S[1, 1] - S[0, 1] * S[1, 0])
        if theta is not None:
            pred, Ha = angle_model(self.x, o, back)
            sa = Ha @ self.P @ Ha + theta_sigma ** 2
            d2, pen = d2 + (theta - pred) ** 2 / sa, pen + np.log(sa)
        return d2, pen

    @property
    def position(self):
        return self.x[:3]

    @property
    def velocity(self):
        return self.x[3:6]

    @property
    def size(self):
        return self.x[6]


class IMMFilter:
    """Interacting multiple model filter (Blom and Bar-Shalom 1988): constant-velocity models that differ only in their
    acceleration noise, e.g. 0.5 m/s^2 for straight flight and 5 m/s^2 for turns, mixed by how well each has been
    predicting. A drop-in for TargetFilter (same constructor, updates and properties); predict() ignores the single
    accel_sigma it is given and uses its own models.

    sojourn: mean time (s) spent in a model before switching; sets the Markov switching probability per step. Not tuned."""

    def __init__(self, o, g, theta=None, size=0.45, size_sigma=0.25, range_guess=40.0, range_sigma=30.0,
                 accels=(0.5, 5.0), sojourn=5.0, mu0=None):
        first = TargetFilter(o, g, theta, size, size_sigma, range_guess, range_sigma)
        self.models = [first] + [TargetFilter.__new__(TargetFilter) for _ in accels[1:]]
        for m in self.models[1:]:
            m.x, m.P, m.rejected = first.x.copy(), first.P.copy(), 0
        self.accels, self.sojourn = np.asarray(accels, float), sojourn
        self.mu = np.full(len(accels), 1.0 / len(accels)) if mu0 is None else np.asarray(mu0, float)
        self.rejected = 0

    def _combined(self):
        x = sum(w * m.x for w, m in zip(self.mu, self.models))
        P = sum(w * (m.P + np.outer(m.x - x, m.x - x)) for w, m in zip(self.mu, self.models))
        return x, P

    @property
    def x(self):
        return self._combined()[0]

    @property
    def P(self):
        return self._combined()[1]

    def predict(self, dt, accel_sigma=None, size_rate_sigma=0.0):
        n = len(self.models)
        stay = np.exp(-dt / self.sojourn)
        Pi = np.full((n, n), (1 - stay) / max(n - 1, 1)) + np.eye(n) * (stay - (1 - stay) / max(n - 1, 1))
        c = Pi.T @ self.mu  # predicted model probabilities
        w = Pi * self.mu[:, None] / c[None, :]  # w[i, j]: probability the target was in i, given it is now in j
        xs = [m.x.copy() for m in self.models]
        Ps = [m.P.copy() for m in self.models]
        for j, m in enumerate(self.models):
            m.x = sum(w[i, j] * xs[i] for i in range(n))
            m.P = sum(w[i, j] * (Ps[i] + np.outer(xs[i] - m.x, xs[i] - m.x)) for i in range(n))
            m.predict(dt, self.accels[j], size_rate_sigma)
        self.mu = c

    def _apply(self, models_yHR, gate):
        """models_yHR: per model (y, H, R). Gate on the combined estimate, then update each model and reweight."""
        x, P = self._combined()
        y0, H0, R0 = models_yHR[-1][3]
        S0 = H0 @ P @ H0.T + R0
        if y0 @ np.linalg.solve(S0, y0) > gate:
            return False
        like = np.empty(len(self.models))
        for j, (m, (y, H, R, _)) in enumerate(zip(self.models, models_yHR)):
            S = H @ m.P @ H.T + R
            like[j] = np.exp(-0.5 * y @ np.linalg.solve(S, y)) / np.sqrt(np.linalg.det(2 * np.pi * S))
            m._update(y, H, R, np.inf)
        mu = self.mu * like
        self.mu = mu / mu.sum() if mu.sum() > 0 else np.full(len(mu), 1.0 / len(mu))
        return True

    def update_bearing(self, o, g, sigma, back=0.0, gate=16.0):
        o = np.asarray(o, float)
        x, _ = self._combined()
        _, _, Ec, Hc = bearing_model(x, o, back)
        R = sigma ** 2 * np.eye(2)
        terms = []
        for m in self.models:
            _, _, E, H = bearing_model(m.x, o, back)
            terms.append((E @ g, H, R, (Ec @ g, Hc, R)))
        ok = self._apply(terms, gate)
        self.rejected = 0 if ok else self.rejected + 1
        return ok

    def update_angle(self, o, theta, sigma, back=0.0, gate=25.0):
        o = np.asarray(o, float)
        x, _ = self._combined()
        pc, Hc = angle_model(x, o, back)
        R = np.array([[sigma ** 2]])
        terms = []
        for m in self.models:
            pred, H = angle_model(m.x, o, back)
            terms.append((np.array([theta - pred]), H[None], R, (np.array([theta - pc]), Hc[None], R)))
        return self._apply(terms, gate)

    def distance(self, o, g, sigma, theta=None, theta_sigma=None, back=0.0):
        """As TargetFilter.distance, on the moment-matched combination of the models."""
        tmp = TargetFilter.__new__(TargetFilter)
        tmp.x, tmp.P = self._combined()
        return tmp.distance(o, g, sigma, theta, theta_sigma, back)

    def reacquire(self, o, g, theta=None):
        for m in self.models:
            m.reacquire(o, g, theta)
        self.rejected = 0

    @property
    def position(self):
        return self.x[:3]

    @property
    def velocity(self):
        return self.x[3:6]

    @property
    def size(self):
        return self.x[6]
