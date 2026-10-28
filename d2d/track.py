"""Tracking several drones at once from camera sightings (build stage 4 and 4b). World frame ENU.

One Tracker runs on each drone. It takes sightings from its own camera and, over the radio, from teammates' cameras:
sharing sightings rather than finished tracks means there is one estimate per drone and nothing to reconcile.
A sighting carries no identity, so each batch (one camera, one frame) is matched to the existing tracks.

  match    : each sighting to at most one track, inside a gate, maximising total likelihood
  birth    : an unmatched sighting starts a tentative track (range from the size prior)
  confirm  : a tentative track becomes real after `confirm_hits` matched sightings from `confirm_cams` cameras
  merge    : two tracks that have converged on the same place are one drone; the older identity is kept
  death    : a track with no matched sighting for `drop_after` seconds (tentative: `tentative_after`) is removed
"""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import linear_sum_assignment

from .estimate import TargetFilter


@dataclass
class Sighting:
    o: np.ndarray  # where the camera was, as reported by its drone (m)
    g: np.ndarray  # unit direction to the detection
    theta: float  # angle subtended by the box width (rad)
    s_bearing: float  # one-sigma direction noise (rad)
    s_angle: float  # one-sigma noise of theta (rad)
    back: float = 0.0  # age of the sighting when it is used (s)
    cam: int = 0  # which camera it came from


class Track:
    def __init__(self, tid, s, size, size_sigma, t, make_filter=TargetFilter):
        self.id, self.hits, self.born, self.last, self.confirmed, self.cams = tid, 1, t, t, False, {s.cam}
        self.f = make_filter(s.o, s.g, s.theta, size, size_sigma)


class Tracker:
    def __init__(self, size=0.45, size_sigma=0.25, accel_sigma=3.0, gate=16.0, confirm_hits=5, drop_after=3.0,
                 tentative_after=0.5, merge_dist=3.0, use_likelihood=True, confirm_cams=1, make_filter=TargetFilter):
        """make_filter: the per-track filter class, TargetFilter (constant velocity, accel_sigma) or IMMFilter."""
        self.size, self.size_sigma, self.accel_sigma, self.gate = size, size_sigma, accel_sigma, gate
        self.confirm_hits, self.drop_after, self.tentative_after, self.merge_dist = confirm_hits, drop_after, tentative_after, merge_dist
        self.confirm_cams = confirm_cams  # cameras that must have seen a track before it counts as real
        self.use_likelihood = use_likelihood  # False: match on distance alone (kept for comparison; see the build log)
        self.make_filter = make_filter
        self.tracks, self.t, self._next = [], 0.0, 1

    def step(self, dt, batches):
        """Advance by dt, then take in batches of sightings (each batch: one camera, one frame)."""
        self.t += dt
        for tr in self.tracks:
            tr.f.predict(dt, self.accel_sigma)
        for batch in batches:
            self._match(batch)
        self._merge()
        self.tracks = [tr for tr in self.tracks
                       if self.t - tr.last <= (self.drop_after if tr.confirmed else self.tentative_after)]

    def _match(self, batch):
        if not batch:
            return
        free = set(range(len(batch)))
        if self.tracks:
            dp = np.array([[tr.f.distance(s.o, s.g, s.s_bearing, s.theta, s.s_angle, s.back) for s in batch]
                           for tr in self.tracks])
            d2 = dp[:, :, 0]
            cost = np.where(d2 <= self.gate, d2 + (dp[:, :, 1] if self.use_likelihood else 0.0), 1e6)
            rows, cols = linear_sum_assignment(cost)
            for i, j in zip(rows, cols):
                if d2[i, j] > self.gate:
                    continue
                tr, s = self.tracks[i], batch[j]
                tr.f.update_bearing(s.o, s.g, s.s_bearing, s.back, gate=np.inf)
                tr.f.update_angle(s.o, s.theta, s.s_angle, s.back, gate=np.inf)
                tr.hits, tr.last = tr.hits + 1, self.t
                tr.cams.add(s.cam)
                tr.confirmed = tr.confirmed or (tr.hits >= self.confirm_hits and len(tr.cams) >= self.confirm_cams)
                free.discard(j)
        for j in sorted(free):
            self.tracks.append(Track(self._next, batch[j], self.size, self.size_sigma, self.t, self.make_filter))
            self._next += 1

    def _merge(self):
        keep = []
        for tr in sorted(self.tracks, key=lambda tr: (not tr.confirmed, tr.id)):  # confirmed first, then oldest
            twin = next((k for k in keep if np.linalg.norm(k.f.position - tr.f.position) < self.merge_dist), None)
            if twin is None:
                keep.append(tr)
            else:
                twin.last = max(twin.last, tr.last)
                twin.cams |= tr.cams
        self.tracks = keep

    @property
    def confirmed(self):
        return [tr for tr in self.tracks if tr.confirmed]
