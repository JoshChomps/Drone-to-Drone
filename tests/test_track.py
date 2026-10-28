"""Checks for d2d.track. Run: python -m unittest discover -s tests"""
import unittest

import numpy as np

from d2d.track import Sighting, Tracker

S_B, S_A = 0.005, 0.002


def sight(o, p, size=0.77, back=0.0):
    d = np.asarray(p, float) - o
    r = np.linalg.norm(d)
    return Sighting(np.asarray(o, float), d / r, 2 * np.arctan(size / (2 * r)), S_B, S_A, back)


O1, O2 = np.array([-40.0, 0, 60]), np.array([0.0, -40, 60])


class Basics(unittest.TestCase):
    def test_two_drones_two_tracks_stable_identities(self):
        trk = Tracker(size=0.77, size_sigma=0.01)
        ids = []
        for k in range(100):
            a, b = np.array([10 + 0.3 * k, 5, 50.0]), np.array([-5, -20 + 0.2 * k, 52.0])
            trk.step(0.1, [[sight(O1, a), sight(O1, b)][::(-1) ** k]])  # order within a batch must not matter
            if k > 10:
                ids.append(tuple(sorted(tr.id for tr in trk.confirmed)))
        self.assertEqual(set(ids), {(1, 2)})
        got = sorted(trk.confirmed, key=lambda tr: tr.f.position[1])
        self.assertLess(np.linalg.norm(got[1].f.position - a), 1.0)
        self.assertLess(np.linalg.norm(got[0].f.position - b), 1.0)

    def test_one_frame_false_alarms_never_become_tracks(self):
        trk = Tracker(size=0.77, size_sigma=0.01)
        rng = np.random.default_rng(0)
        for k in range(100):
            batch = [sight(O1, [10, 5, 50])]
            if k % 3 == 0:
                batch.append(sight(O1, rng.uniform([-30, -30, 40], [30, 30, 60])))
            trk.step(0.1, [batch])
        self.assertEqual([tr.id for tr in trk.confirmed], [1])

    def test_track_dies_when_unseen(self):
        trk = Tracker(size=0.77, size_sigma=0.01, drop_after=2.0)
        for _ in range(20):
            trk.step(0.1, [[sight(O1, [10, 5, 50])]])
        self.assertEqual(len(trk.confirmed), 1)
        for _ in range(25):
            trk.step(0.1, [[]])
        self.assertEqual(len(trk.tracks), 0)


class Matching(unittest.TestCase):
    def test_close_pair_needs_the_best_pairing_not_the_first(self):
        """Two drones 5 m apart and coarse sightings, so each sighting passes the gate of both tracks. Only choosing
        the pairing with the lowest total distance keeps the tracks on their own drones."""
        trk = Tracker(size=0.77, size_sigma=0.01)
        for k in range(150):
            a, b = np.array([10 + 0.2 * k, 0, 50.0]), np.array([10 + 0.2 * k, 5, 50.0])
            batch = [sight(O1, a), sight(O1, b)][::(-1) ** k]
            for s in batch:
                s.s_bearing = 0.04
            trk.step(0.1, [batch])
        got = sorted(trk.confirmed, key=lambda tr: tr.f.position[1])
        self.assertEqual(len(got), 2)
        self.assertLess(np.linalg.norm(got[0].f.position - a), 1.0)
        self.assertLess(np.linalg.norm(got[1].f.position - b), 1.0)

    def test_duplicate_tracks_merge_and_the_older_identity_survives(self):
        trk = Tracker(size=0.77, size_sigma=0.01, gate=0.0)  # gate 0: every sighting starts a new track
        trk.merge_dist = 0.0
        for _ in range(2):
            trk.step(0.1, [[sight(O1, [10, 5, 50])]])
        self.assertEqual([tr.id for tr in trk.tracks], [1, 2])
        trk.merge_dist = 3.0
        trk.step(0.1, [[]])
        self.assertEqual([tr.id for tr in trk.tracks], [1])


    def vague_track_scenario(self, use_likelihood):
        """A well-established track on a drone, plus a brand-new track from a false alarm almost in line with it.
        Returns how many of the next 10 sightings of the drone went to the established track."""
        trk = Tracker(use_likelihood=use_likelihood, tentative_after=5.0)
        p = np.array([5.0, 3, 50])
        for _ in range(60):
            trk.step(0.1, [[sight(O1, p)], [sight(O2, p)]])
        before = trk.tracks[0].hits
        d = p - O1
        ghost = O1 + 0.6 * d + [0, 0.15, 0]  # a false alarm 0.3 degrees off the drone's direction, looking bigger
        trk.step(0.1, [[sight(O1, p), sight(O1, ghost, size=0.9)]])
        self.assertEqual(len(trk.tracks), 2)
        for _ in range(10):
            trk.step(0.1, [[sight(O1, p + [0, 0.35, 0])]])  # an ordinary 1.5-sigma measurement error
        return trk.tracks[0].hits - before - 1

    def test_a_vague_new_track_must_not_steal_sightings(self):
        """On distance alone a vague track is 'close' to everything and takes the drone's sightings. On likelihood
        the established track keeps them."""
        self.assertEqual(self.vague_track_scenario(use_likelihood=True), 10)
        self.assertLess(self.vague_track_scenario(use_likelihood=False), 10)

    def test_two_cameras_needed_to_confirm(self):
        trk = Tracker(size=0.77, size_sigma=0.01, confirm_cams=2)
        for _ in range(20):
            trk.step(0.1, [[sight(O1, [10, 5, 50])]])
        self.assertEqual(len(trk.confirmed), 0)
        s2 = sight(O2, [10, 5, 50])
        s2.cam = 1
        trk.step(0.1, [[sight(O1, [10, 5, 50])], [s2]])
        self.assertEqual(len(trk.confirmed), 1)


class Team(unittest.TestCase):
    def test_two_cameras_one_drone_one_track(self):
        """With unknown size each camera alone would place the drone at a different range. They must still end up as
        one track, not two."""
        trk = Tracker()  # size prior 0.45 +- 0.25; the drone is 0.77
        p = np.array([5.0, 3, 50])
        for k in range(100):
            q = p + [0.2 * k, 0, 0]
            trk.step(0.1, [[sight(O1, q)], [sight(O2, q)]])
        self.assertEqual(len(trk.confirmed), 1)
        self.assertLess(np.linalg.norm(trk.confirmed[0].f.position - q), 1.0)
        self.assertAlmostEqual(trk.confirmed[0].f.size, 0.77, delta=0.05)

    def test_crossing_in_one_image_keeps_identities_with_a_second_camera(self):
        """Two drones at different ranges cross as seen from camera 1. Camera 2 sees them well apart."""
        trk = Tracker(size=0.77, size_sigma=0.01)
        for k in range(120):
            a = np.array([0.0, -12 + 0.2 * k, 50])  # near, moving north
            b = np.array([30.0, 12 - 0.2 * k, 50])  # far, moving south; they line up from O1 at k = 60
            trk.step(0.1, [[sight(O1, a), sight(O1, b)], [sight(O2, a), sight(O2, b)]])
        self.assertEqual(sorted(tr.id for tr in trk.confirmed), [1, 2])
        t1 = next(tr for tr in trk.confirmed if tr.id == 1)
        self.assertLess(np.linalg.norm(t1.f.position - a), 1.5)  # track 1 is still the near drone


if __name__ == "__main__":
    unittest.main()
