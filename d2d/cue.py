"""Teammate cueing and heading calibration (build stage 7, doc 02 section 4). World frame ENU, numpy only.

Teammates share their positions over the radio. Projected into a camera, each shared position says where that teammate
should appear (a cue). A detection that matches a cue is that teammate, not an unknown drone; the gap between where it
is and where the cue says it should be measures the camera's heading error.

  cue       : project a teammate's shared position with the corrected attitude; gate on the pixel position and on the
              box size (a teammate's size is known, and so is its range)
  label     : a detection inside exactly one cue's gate, and in no other cue's, is that teammate. A detection in a gate
              but ambiguous is withheld from the tracker and not used for calibration. Everything else goes to the Tracker
  calibrate : each labelled teammate's azimuth residual updates a scalar Kalman filter on the camera's heading bias
  health    : a cue in view with no detection is a miss (teammate hidden, too small, or its shared position wrong)

Only heading (yaw) bias is estimated: it is the error that stages 3 and 4 found largest. Roll and pitch come from the
autopilot's gravity reference and are treated as noise. Clock offset is not estimated yet.
"""
import numpy as np

from .geometry import X500_SPAN, _rz

DEG = np.pi / 180
LABEL_NONE, LABEL_AMBIGUOUS = -1, -2


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


class HeadingCalibrator:
    """One per camera. b is the heading bias: the reported yaw minus the true yaw (rad)."""

    def __init__(self, sigma0_deg=3.0, drift_deg_per_s=0.01, floor_deg=0.5, sigma_pos=1.0, s_px=1.0, s_att_deg=0.5,
                 span=X500_SPAN, gate=11.34, edge=0.0):
        """sigma_pos: one-sigma error of the camera-to-teammate position difference (both GNSS fixes), m. floor_deg: the
        least heading uncertainty ever declared to the tracker; GNSS errors are slow, not white, so the filter's own
        variance is optimistic once it has averaged many looks at the same few teammates. gate: chi-square, 3 dof, 99 %.
        edge: a cue that projects up to this many sigmas outside the image is still matched, since its teammate's box
        can be inside (0: only cues inside the image, the stage 7 behaviour; build log section 11)."""
        self.b, self.P = 0.0, (sigma0_deg * DEG) ** 2
        self.q, self.floor = (drift_deg_per_s * DEG) ** 2, floor_deg * DEG
        self.sigma_pos, self.s_px, self.s_att, self.span, self.gate = sigma_pos, s_px, s_att_deg * DEG, span, gate
        self.edge = edge
        self.updates = self.cues = self.misses = 0

    @property
    def sigma(self):
        return np.sqrt(self.P)

    @property
    def declared(self):
        """Heading uncertainty to declare to the tracker (rad)."""
        return max(self.sigma, self.floor)

    def predict(self, dt):
        self.P += self.q * dt

    def correct(self, r_world_body):
        """The reported attitude with the estimated heading bias removed."""
        return _rz(-self.b) @ r_world_body

    def match(self, cam, o, r_corr, boxes, mates):
        """Label each box: index of the teammate it is, LABEL_NONE, or LABEL_AMBIGUOUS. o: own reported position;
        mates: (M, 3) teammates' shared positions; r_corr: corrected attitude. Keeps self.last: which cues were in view
        and the three gate terms (horizontal, vertical, size) of every cue-box pair, for diagnosis."""
        labels = np.full(len(boxes), LABEL_NONE)
        self.last = None
        if not len(boxes) or not len(mates):
            return labels, 0
        uv, z = cam.project(o, r_corr, mates)
        seen = cam.in_view(uv, z)
        c = np.array([[(b[0] + b[2]) / 2, (b[1] + b[3]) / 2] for b in boxes])
        th = np.array([cam.subtended_angle(b) for b in boxes])
        cand = np.zeros((len(mates), len(boxes)), bool)
        terms = np.full((len(mates), len(boxes), 3), np.nan)
        for m in np.flatnonzero(z > 0):
            r = np.linalg.norm(mates[m] - o)
            x = (uv[m, 0] - cam.width / 2) / cam.f  # du/dyaw grows as 1 + x^2 away from the centre
            su2 = (cam.f * (1 + x * x)) ** 2 * (self.P + self.s_att ** 2 + (self.sigma_pos / r) ** 2) + self.s_px ** 2
            sv2 = cam.f ** 2 * (self.s_att ** 2 + (self.sigma_pos / r) ** 2) + self.s_px ** 2
            mu, mv = self.edge * np.sqrt(su2), self.edge * np.sqrt(sv2)
            if not (seen[m] or (-mu <= uv[m, 0] < cam.width + mu and -mv <= uv[m, 1] < cam.height + mv)):
                continue
            th_m = 2 * np.arctan(self.span / (2 * r))
            st2 = (1.5 / cam.f) ** 2 + (th_m * self.sigma_pos / r) ** 2
            terms[m] = np.column_stack([(c[:, 0] - uv[m, 0]) ** 2 / su2, (c[:, 1] - uv[m, 1]) ** 2 / sv2,
                                        (th - th_m) ** 2 / st2])
            cand[m] = terms[m].sum(1) < self.gate
        for k in np.flatnonzero(cand.any(0)):
            m = np.flatnonzero(cand[:, k])
            labels[k] = m[0] if len(m) == 1 and cand[m[0]].sum() == 1 else LABEL_AMBIGUOUS
        self.last = dict(seen=seen, uv=uv, terms=terms)
        self.cues += int(seen.sum())
        self.misses += int((seen & ~cand.any(1)).sum())
        return labels, int(seen.sum())

    def update(self, g, o, q, f):
        """Bearing g (unit vector, computed with the corrected attitude) of a detection labelled as the teammate at q;
        f: focal length in px. Roll and pitch noise move a point mostly vertically; the 0.3 share of it that reaches
        azimuth (a point ~15 deg off the optical axis) is a rough figure, not derived."""
        d = q - o
        rh = np.hypot(d[0], d[1])
        y = _wrap(np.arctan2(g[1], g[0]) - np.arctan2(d[1], d[0]))
        R = (self.s_px / f) ** 2 + (self.sigma_pos / rh) ** 2 + (0.3 * self.s_att) ** 2
        k = self.P / (self.P + R)
        self.b += k * y
        self.P *= 1 - k
        self.updates += 1

    def step(self, dt, cam, o, r_est, boxes, mates):
        """Predict, label the boxes against the cues, update on the unambiguous teammates. Returns (labels, r_corr), where
        r_corr is the attitude corrected with the updated estimate."""
        self.predict(dt)
        labels, _ = self.match(cam, o, self.correct(r_est), boxes, mates)
        for k in np.flatnonzero(labels >= 0):
            b = boxes[k]
            g = cam.bearing(self.correct(r_est), [(b[0] + b[2]) / 2, (b[1] + b[3]) / 2])[0]
            self.update(g, o, mates[labels[k]], cam.f)
        return labels, self.correct(r_est)
