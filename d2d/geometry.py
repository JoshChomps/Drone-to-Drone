"""Frames and camera geometry (build stage 2). numpy only.

Frames
  world : ENU about a shared reference point (x east, y north, z up), metres
  body  : FLU (x forward, y left, z up)
  camera: optical (x right, y down, z forward); pixels (u right, v down), origin at the top-left corner

R_ab maps coordinates in frame b to coordinates in frame a.
"""
from dataclasses import dataclass

import numpy as np

WGS84_A, WGS84_F = 6378137.0, 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)
# x500 envelope (PX4 v1.17.0 gz model x500_base/model.sdf, collision boxes, read 2026-10-05): rotor centres at
# (+-0.174, +-0.174) m, 0.279 m props -> diameter 0.492 + 0.279 = 0.77 m (doc 01 section 4.2 agrees); skids bottom at
# -0.227 m, rotor plane at +0.060 m -> height 0.29 m. Meshes not measured; legs and props only, no payload.
X500_SPAN, X500_HEIGHT = 0.77, 0.29


def _ecef(lat, lon, alt):
    lat, lon = np.radians(lat), np.radians(lon)
    n = WGS84_A / np.sqrt(1 - WGS84_E2 * np.sin(lat) ** 2)
    return np.stack([(n + alt) * np.cos(lat) * np.cos(lon), (n + alt) * np.cos(lat) * np.sin(lon),
                     (n * (1 - WGS84_E2) + alt) * np.sin(lat)], -1)


def lla_to_enu(lat, lon, alt, ref):
    """Latitude, longitude (deg) and altitude (m) to ENU metres about ref = (lat0, lon0, alt0). Exact (doc 01 section 3.3)."""
    la, lo = np.radians(ref[0]), np.radians(ref[1])
    r = np.array([[-np.sin(lo), np.cos(lo), 0],
                  [-np.sin(la) * np.cos(lo), -np.sin(la) * np.sin(lo), np.cos(la)],
                  [np.cos(la) * np.cos(lo), np.cos(la) * np.sin(lo), np.sin(la)]])
    return (_ecef(np.asarray(lat, float), np.asarray(lon, float), np.asarray(alt, float)) - _ecef(*ref)) @ r.T


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def attitude(yaw, pitch=0.0, roll=0.0):
    """R_world_body from angles in radians: yaw anticlockwise from east, pitch nose-up, roll right-wing-down."""
    return _rz(yaw) @ _ry(-pitch) @ _rx(roll)


def px4_attitude(q):
    """R_world_body (ENU, FLU) from a PX4 attitude quaternion [w, x, y, z], which rotates FRD body into NED."""
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    r_ned_frd = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                          [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                          [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]]) @ r_ned_frd @ np.diag([1.0, -1.0, -1.0])


@dataclass(frozen=True)
class Camera:
    """Pinhole camera fixed to the body, looking forward, optionally tilted down."""
    width: int = 1280
    height: int = 720
    hfov_deg: float = 90.0
    tilt_down_deg: float = 0.0

    @property
    def f(self):
        return self.width / 2 / np.tan(np.radians(self.hfov_deg) / 2)

    @property
    def r_body_cam(self):
        return _ry(np.radians(self.tilt_down_deg)) @ np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0.0]])

    def project(self, p_cam, r_world_body, pts):
        """World points (N, 3) to pixels (N, 2) and depth along the optical axis (N,). Depth <= 0 is behind the camera."""
        pc = (np.atleast_2d(pts) - p_cam) @ (r_world_body @ self.r_body_cam)
        z = pc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            uv = self.f * pc[:, :2] / z[:, None] + [self.width / 2, self.height / 2]
        return uv, z

    def in_view(self, uv, z):
        return (z > 0) & (uv[:, 0] >= 0) & (uv[:, 0] < self.width) & (uv[:, 1] >= 0) & (uv[:, 1] < self.height)

    def bearing(self, r_world_body, uv):
        """Pixels (N, 2) to unit direction vectors in the world frame (N, 3)."""
        uv = np.atleast_2d(uv)
        d = np.column_stack([uv - [self.width / 2, self.height / 2], np.full(len(uv), self.f)])
        d = d @ (r_world_body @ self.r_body_cam).T
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def box(self, p_cam, r_world_body, p_target, span=X500_SPAN, height=X500_HEIGHT):
        """Bounding box (u0, v0, u1, v1) of an upright cylinder centred on p_target, or None if any of it is behind."""
        a = np.arange(16) * 2 * np.pi / 16
        rim = np.column_stack([span / 2 * np.cos(a), span / 2 * np.sin(a), np.zeros(16)])
        pts = np.vstack([rim + [0, 0, height / 2], rim - [0, 0, height / 2]]) + p_target
        uv, z = self.project(p_cam, r_world_body, pts)
        if (z <= 0).any():
            return None
        return np.array([*uv.min(0), *uv.max(0)])

    def subtended_angle(self, box):
        """Angle (rad) the box width subtends at the camera. Unlike the width in pixels it does not change when the
        camera turns (Ning et al., IJRR 2024, section 3)."""
        u0, v0, u1, v1 = box
        dy = (v0 + v1) / 2 - self.height / 2
        left, right = np.hypot(np.hypot(self.f, u0 - self.width / 2), dy), np.hypot(np.hypot(self.f, u1 - self.width / 2), dy)
        return np.arccos(np.clip((left ** 2 + right ** 2 - (u1 - u0) ** 2) / (2 * left * right), -1, 1))

    def range_from_box(self, box, span=X500_SPAN):
        """Range to a target of known width from its box."""
        return span / 2 / np.tan(self.subtended_angle(box) / 2)

    def cue_sigma_px(self, rng, sigma_pos, sigma_att):
        """One-sigma pixel error of a shared position projected into the image: position error (m) at range rng (m)
        plus attitude error (rad). Near the image centre (doc 02 section 5.2)."""
        return self.f * np.hypot(sigma_pos / rng, sigma_att)
