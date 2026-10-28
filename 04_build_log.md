# 04: Build Log: What We Intended, Did, Measured, Got Wrong, and Learned

> [!NOTE]
> **A chronological record, written to be learned from (user request, 2026-10-04).** One entry per step. Every entry says what the intent was, what was done, what was measured, what worked, what did not, and what to take from it. Wrong turns are kept, with the wrong number and the reason it was wrong. Add an entry for every step from now on; never rewrite an old entry, add a correction instead.
>
> Plans live in [doc 01](01_phase2_simulation_plan.md) and [doc 03](03_build_plan.md); the literature in [doc 02](02_research_survey_and_novel_directions.md). This file is the history.

**Entry format:** Intent → What was done → Measurements → Worked → Did not work → Lesson.

**Index**

| # | Date | Step | Outcome in one line |
| :-: | :-: | :--- | :--- |
| 1 | 09-28/29 | Phase-1 research and the phase-2 simulation plan | Stack chosen; five numerical facts that shaped the design |
| 2 | 10-03 | Literature sweep | 176 resources; only 12 read at the source |
| 3 | 10-03 | Five toy trials | Four useful results; the tag trial was wrong twice before it was right |
| 4 | 10-03 | Scope discussion | Observe and identify; no vision-driven chasing |
| 5 | 10-03 | Stage 2: camera geometry | 11 tests, passed first time, so the tests themselves were tested |
| 6 | 10-04 | Stage 3: estimators | Works; the first full run was ~20 m wrong for three separate reasons |
| 7 | 10-04 | Stage 4: multi-drone tracker and team fusion | Five cameras: all found, 1.7 m, count right 86 %; the first run held 11 tracks for 5 drones, for two separate reasons |
| 8 | 10-04 | Handoff: documents brought up to date | State, run instructions, open items and the pending choice of next stage are written down; three documentation slips found |
| 9 | 10-05 | Repository, reproduction, realistic detector, more seeds | Pushed; numbers reproduce; false alarms at 0.16/frame → count right 42 %, but calibrated headings hold 94 %; the 86 % baseline is 75 % on 32 seeds |
| 10 | 10-05 | Stage 7: teammate cueing and heading calibration | Heading 1.8° → 0.34°; teammates in view: 0 % without cueing, 87 % with cueing + calibration; one prediction wrong (the declared floor did not matter) |
| 11 | 10-05 | CPU follow-ups while WSL waits (stage 7b, leaks, scheduler, IMM, paired seeds, reading, hardware) | Leaks found (half an edge case) and the wider gate lifts the count to 86-94 %; dead-reckon shared positions; stage 7's 87 % was helped by lucky small GNSS errors (prediction wrong); a 15 tiles/s scheduler keeps 99 % once made fair, because the confirmation rule was too loose; IMM no gain; doc 02's G3 gap was wrong |

---

## 1. Phase-1 research and the phase-2 plan (2026-09-28/29)

**Intent.** Decide how to simulate several drones on this laptop and what to test first, before installing anything.

**What was done.** Facts checked at the source (PX4 docs and the v1.17.0 source tag, MAVSDK release notes, Microsoft WSL docs, NVIDIA requirements). Design numbers computed in `tools/phase2_checks.py` instead of being assumed.

**Measurements that shaped the design** (all reproducible with `python tools/phase2_checks.py`):

| Question | Number | Consequence |
| :--- | :--- | :--- |
| How big is an x500? | Envelope radius 0.386 m; contact below 0.77 m centre to centre | The definition of a collision in every later test |
| How far apart must two drones stay? | 5.8 m at 5 m/s closing; 13.5-22.5 m at 10 m/s | Closing speed dominates; head-on at cruise needs wide spacing or altitude layers |
| Does a simple safety filter (CBF) work with real acceleration limits? | With γ = 3 and 3 m/s², separation fell to 1.19 m with fresh data and to contact (0.63 m) with 0.5 s-old data. γ ≤ 1 held | γ is a safety parameter, not a tuning knob |
| Does average packet loss describe a radio? | At 9 % average loss, bursty loss leaves 1.5 s gaps; random loss 0.3 s | Design for bursts, not averages |
| Is a spherical-earth conversion good enough? | 2.1 m wrong at 1 km | Use the exact conversion (now `d2d.geometry.lla_to_enu`) |

**Worked.** Checking the no-avoidance baseline first: it collides (0.50 m), so the test scenario is hard enough to mean something.

**Did not work / ruled out.** IsaacLab (needs 16 GB of video memory and 32 GB of RAM; the laptop has 8 and 16).

**Lesson.** Do the arithmetic before choosing. Three of the five numbers contradicted what a quick guess would have assumed.

---

## 2. Literature sweep (2026-10-03)

**Intent.** Understand the whole field (papers, open-source projects, companies) before building, and find what has not been tried.

**What was done.** 72 web searches in nine batches; 15 pages fetched; the PDF text of three papers read for their equations and experiment details (the bearing-angle paper, the STT triangulation paper, Schilling's vision-based flocking). Result: [doc 02](02_research_survey_and_novel_directions.md) with 176 catalogued resources.

**Measurements.** 12 entries read at the primary source, 8 verified earlier, **156 search-level only**.

**Worked.**
* Reading the PDFs directly. The search summaries and the abstracts carried no numbers. The PDFs gave the bearing-angle observability theorem, the fact that STT flew on three 6.3 kg DJI M300s with gimbals, and Schilling's 2 m spacing at 0.5 m/s.
* Marking every entry with how far it was checked, so an unverified summary cannot be mistaken for a fact.

**Did not work.**
* The page-summarising tool could not read PDFs; they had to be saved and their text extracted with `pypdf`. One PDF exceeded the size limit and was never read (the GLAD paper body).
* Some search summaries were unreliable. One "source" for radio ranging accuracy was a shopping-site article; it was left out.
* An over-statement caught on re-reading: I wrote that in Sandia's tag game "a tag is a crash". The article says the evaders were trained to make pursuers collide; how a tag is registered was not stated. Corrected in doc 02.

**Lesson.** A long reference list is not a verified one. The honest count (12 of 176) belongs at the top of the document.

---

## 3. Five toy trials (2026-10-03)

**Intent.** Test the most promising ideas numerically before committing to them. `tools/research_checks.py`, 2-D point masses.

**Measurements and what they meant.**

| Trial | Result | Meaning |
| :--- | :--- | :--- |
| Vision against GNSS for a teammate | Vision range better inside 18 m; vision cross-range better out to 57 m | They are complementary; fuse them |
| Projecting a shared position into the image | Error 65 px at 10 m, 28 px at 25 m, 13 px at 100 m; always larger than the box | A shared position gives a search window, never a label. RTK does not fix it: attitude error is 11 px per degree |
| Heading from seeing a teammate | 2.0° per look at 30 m with 1 m GNSS | Worth doing: heading bias turned out to be the largest error everywhere |
| Locating a silent drone (2-D) | Follower, direction only: 24 m. With box size: 3.8 m. Two observers at 90°: 1.3 m | **The 3.8 m was later shown to be optimistic (entry 6)** |
| Tag | Without a safety layer, collisions (0.01 m) | See below |

**The tag trial was wrong twice.**

1. *Every configuration tagged 100 % of the time.* The evader policies were too weak, so no pursuit strategy could be told from another. A test in which everything passes measures nothing.
2. *After strengthening it, the simple "flee" evader suddenly escaped 43 % of 1-on-1 games.* I guessed the earlier tags had been unsafe fast fly-bys that the new speed-dependent safety distance now forbade. **A direct check showed 0 % were fly-bys** (median relative speed during a tag: 0.6 m/s). Tracing one run step by step showed the real cause: the evader always moved at full speed, so in a corner it jittered; the closing speed flipped between −9.9 and +9.9 m/s every step; the safety distance flipped between 2.2 m and 12.4 m; the pursuer stalled at 12 m. An artefact, not a strategy.
3. *The fix* (acceleration limits for everyone and a mutual braking rule) gave usable results, and one real warning: against an evader that turns into the pursuer, the braking rule let a pair reach **0.54 m**, which is contact.

**Lesson.** When a result surprises you, measure the explanation before writing it down. My first explanation was plausible and wrong. Also: a safety distance driven by an instantaneous, noisy quantity will chatter.

---

## 4. Scope discussion (2026-10-03)

**Intent (user).** Five drones recognising five others and each "marking" one efficiently, fast, predicting movement, by vision.

**What happened.** My first reply was stopped by a safety filter. Over several exchanges the scope was narrowed and agreed:

* **In:** detecting, sorting (teammate or other), tracking with identities, estimating position and velocity, merging the team's sightings; vision-only flocking among our own drones.
* **Out:** steering drones to chase or close on drones that are not sharing positions, using camera estimates; assigning pursuers to camera-detected drones.
* While identifying, drones fly pre-planned paths and may turn in place to look.

**Points corrected along the way** (worth keeping because they are easy to get wrong):

* *"YOLO already gives real-world speed."* It gives boxes in pixels. Speed demos work for a fixed camera over a calibrated road surface. From a moving, tilting camera looking at an object in open air, pixels become metres only with range and with the camera's own motion removed.
* *"Vision gives acceleration."* It is a noisy bearing differentiated twice. The bearing-angle paper leaves acceleration out of the state because the estimate diverges. Entry 6 measures velocity at about 1 m/s at best, which bounds what acceleration could be.
* *"Separation is only needed within each team."* The pairs that approach each other on purpose are from opposite sides.
* A good idea from the user: score proximity with a mutual range sensor (UWB), not with GNSS or a camera.

**Lesson.** The pre-planned-path rule changed the science, not only the scope: entry 6 shows a single observer on a pre-planned path cannot range an unknown drone, where a follower could.

---

## 5. Stage 2: camera geometry (2026-10-03)

**Intent.** One tested module that relates drones to pixels, so every later stage shares the same frames and camera model. Nothing installed.

**What was done.** `d2d/geometry.py`: exact GPS-to-local conversion; attitude from yaw, pitch, roll and from PX4's quaternion; a pinhole `Camera` (project, in view, pixel to direction, bounding box of a drone, subtended angle, range from box, cue error). `tests/test_geometry.py`: 11 tests against hand calculations.

**Frames chosen (and why they are written down).** World: east, north, up. Body: forward, left, up. Camera: right, down, forward. PX4 uses north-east-down and forward-right-down, so one function converts. Frame mix-ups are the classic silent bug in this kind of code.

**Measurements.** An x500 (0.77 m) is 19.7 px wide at 25 m, 9.9 px at 50 m, 4.9 px at 100 m with a 1280-px, 90° camera, matching doc 01. 1 km north at 50° N drops 7.8 cm from the local horizontal plane.

**Worked.** All 11 tests passed on the first run.

**Did not work.** The first test command failed (`-t .` needs the tests folder to be a package); the sibling project's form `python -m unittest discover -s tests` works.

**The check on the tests.** Tests that pass first time may be unable to fail. I broke the code on purpose three ways:

| Deliberate break | Caught by |
| :--- | :--- |
| Camera mirrored left-right | `test_centre_and_directions` |
| Pitch sign reversed | `test_attitude_conventions` |
| PX4 conversion without the body-axis flip | `test_px4_quaternion` |

**Known weakness.** The x500's height (0.25 m) is a guess; only the width comes from the model file. Only one test catches a mirrored camera.

**Lesson.** A test is evidence only if it fails on the bug it guards.

---

## 6. Stage 3: estimators (2026-10-04)

**Intent.** Locate a drone that shares nothing, from one or more cameras, in 3-D, with observers on pre-planned paths. Reproduce the 2-D toy numbers (24 m → 3.8 m → 1.3 m) with real camera geometry.

**What was done.** `d2d/estimate.py`: one extended Kalman filter per drone. State: position, velocity, and the drone's physical width. Two measurements: a direction (unit vector) and the angle the box width subtends. Both can come from any observer and can be old: a late measurement is evaluated on the state propagated back to its own time.

### 6.1 The noise-free tests found a bad test, not a bad filter

* Jacobians matched finite differences. Triangulation was exact.
* `test_orbiting_observer…` failed: error 1.13 m against a 1.0 m limit. Printing the run over time showed the target had flown **194 m away**; 1.13 m is 0.6 % of that and inside the filter's own stated uncertainty (3.3 m). The scenario let the target leave. Fixed the scenario (target stays 35-75 m away), not the threshold.
* After the fix, the theory showed up cleanly:

| Observer | Measurements | Position error | Size estimate (true 0.500, start 0.300) |
| :--- | :--- | :-: | :-: |
| Orbiting | direction + box size | 0.03 m | 0.500 |
| Hovering | direction + box size | 21.5 m | 0.299 (never moved) |
| Hovering | direction + box size, size known | 0.00 m | - |
| Orbiting | direction only | 0.01 m | - |

A hovering observer cannot separate range from size: a drone twice as big, twice as far and twice as fast looks identical. This is the bearing-angle theorem (the observer needs higher-order motion than the target).

### 6.2 The first realistic run was ~20 m wrong everywhere

First table (1280-px camera, re-acquisition not yet written, model freedom 1 m/s²):

| Configuration | Position error |
| :--- | :-: |
| 1 hovering, size **known** | 21.9 m |
| 2 hovering at 90° | 9.6 m |
| 2 orbiting at 90° | 10.1 m |

A known-size target should give 1-2 m. So this was a fault, not a finding. Found in this order:

1. **Checked the measurement against the model with no noise.** Agreement to 0.001° and 0.2 %. Not a geometry bug.
2. **Counted gate rejections.** 30 % of direction updates were being rejected. Removing heading bias changed nothing, so not bias.
3. **Traced one bad run (seed 5, 73 m error).** Early residuals were under 1°. Then the drone passed close beneath the observer, left the camera's view, turned while unseen, and on reappearing **every sighting was rejected for the rest of the run** (95 of 181; mean residual 14-21°). *Cause 1: no recovery from a lost track.* Added `reacquire` after 5 rejections in a row. Known-size error 21.9 m → 7.3 m.
4. **Still poor, and the drone was seen in only 46 % of frames.** Breakdown: out of view 5 %, **in view but too small to detect 50 %**, detected 45 %. Median range 44 m; median box width 7.5 px; 56 % of boxes under 8 px. *Cause 2: the camera.* A 0.5 m drone reaches the 8-px limit at 40 m with f = 640 px. With f = 1920 px the limit is 120 m and the drone is seen 89 % of the time.
5. **Still 3.8 m and 2.3 m/s for known size with the good camera. Ran with all noise off: 3.2 m and 2.3 m/s.** So not noise. A straight-flying drone with no noise gave 0.41 m and 0.10 m/s (two observers: 0.05 m, 0.07 m/s), so the pipeline is right. *Cause 3: lag.* A constant-velocity model with 1 m/s² of freedom cannot follow a drone that turns continuously when range updates are weak.

| Acceleration freedom | 1 observer, size known | 2 observers (velocity) |
| :-: | :-: | :-: |
| 1 m/s² | 3.63 m | 1.75 m/s |
| 2 m/s² | 2.50 m | 1.25 m/s |
| 3 m/s² | 2.21 m | 1.14 m/s |
| 5 m/s² | 2.01 m | 1.08 m/s |

Chose 3 m/s². **This is tuned on the test scenario, so it is not validated.** A turn-aware model is the proper fix.

### 6.3 Final measurements

Full table in [doc 03 §5](03_build_plan.md#5-stage-3-results-2026-10-04). The essentials, good camera: one observer and unknown size 7.3 m (the size-prior error; orbiting does not help against a drone that manoeuvres as hard as the observer); known size 2.5 m; two observers at 90° 2.3-2.7 m; with calibrated headings **1.1 m**; at 30° 5.8 m; velocity about 1 m/s at best; a 0.5 s-late sighting used as fresh 3.3 m, used at its own time 2.5 m.

**Worked.** The filter; using late data at its own time; two observers; splitting "error while seen" from "error over all time".

**Did not work.** One observer against an unknown manoeuvring drone; the 1280-px camera; the stiff motion model; running without track recovery.

**Unexplained.** Loosening the motion model also cut the one-observer, unknown-size error from 16.6 m to 7.6 m. I do not know why.

**Lessons.**
* One bad number can have several causes stacked on top of each other. Fixing the first (re-acquisition) made things better but not right; stopping there would have blamed the estimator for the camera's fault.
* A 2-D toy without a field of view cannot lose a track, so it never showed the need for recovery.
* The 2-D toy's single-observer result depended on the observer following the target. Changing the rules of motion changed the answer.
* Pixels on target decide the outcome before any algorithm does. This is the sibling project's main lesson, repeated.

---

## 7. Stage 4 and 4b: multi-drone tracker and team fusion (2026-10-04)

**Intent.** Five teammates' cameras keeping track of five other, identical-looking drones: how many there are, where each is, and which is which over time. One estimate per drone, even though each camera sees only some of them and no sighting says which drone it is.

**Design decisions and why.**

* **Share sightings, not tracks.** Each drone runs one tracker fed by its own camera and by teammates' sightings over the radio. Sharing finished tracks would mean reconciling five separate opinions about each drone. A sighting is small (camera position, direction, box angle, time: about 24 bytes), so 5 drones × 5 sightings × 10 Hz is about 48 kbit/s.
* **One camera frame is one batch.** Within a batch each sighting can belong to at most one track and each track can take at most one sighting. That constraint is what stops two drones collapsing into one.
* **Tentative, confirmed, merged, dropped.** A sighting that matches nothing starts a tentative track; enough matches confirm it; two tracks that converge on the same place are one drone and the older identity survives; an unseen track is eventually dropped.

**What was done.** `d2d/track.py` (`Sighting`, `Track`, `Tracker`), `TargetFilter.distance`, `tests/test_track.py` (9 tests), `tools/stage4_checks.py`.

### 7.1 The tests passed first time, and one of them proved nothing

Five tracker tests passed on the first run. Deliberate breaks:

| Deliberate break | Caught? |
| :--- | :--- |
| No merging | Only indirectly, by the false-alarm test |
| **Match sightings to tracks in arrival order, not by best fit** | **Not caught by any test** |
| Confirm a track on its first sighting | Caught |
| No gate | Caught |
| Tracks never die | Caught |

Why the worst break went unnoticed: the gate rejected the wrong pairings, the rejected sightings started new tracks, and merging folded those back into the right ones. The tracker limped to the right answer by a wasteful route, and the test only looked at the answer. Two tests were added that leave no such escape (a close pair with coarse sightings, where both sightings pass both gates; and a direct merge test). Both breaks are now caught directly.

**Lesson.** A system with several safety nets can hide a broken part. Test the part where the nets cannot help.

### 7.2 First 5-on-5 run: every drone found, almost everything else wrong

| Observers | Found | Count right | Error | Swaps per drone per minute | False tracks |
| :-: | :-: | :-: | :-: | :-: | :-: |
| 1 | 47 % | 45 % | 4.9 m | 2.1 | 3.0 |
| 2 | 94 % | 4 % | 2.9 m | 3.5 | 3.0 |
| 5 | 100 % | 3 % | 2.2 m | 4.5 | 5.6 |
| 5, size known | 100 % | 27 % | 1.9 m | 5.6 | 2.6 |

"Found 100 %" alone would have looked like success. The tracker was holding **11 confirmed tracks for 5 drones**.

**What the false tracks were.** Distance from each false track to the nearest real drone: under 3 m 14 %, 3-8 m 42 %, 8-20 m 32 %, over 20 m 12 %. So mostly duplicates of real drones, displaced by a few metres, just too far apart for the 3 m merge rule. Not random clutter.

### 7.3 Cause 1: heading bias the tracker was not told about

Hypothesis: each camera's 2° heading bias is four times the 0.5° noise the tracker assumed, so a drone's own sighting often fails its own track's gate, starts a second track, and that camera keeps feeding it. Tested directly (4 seeds, 40 s):

| Condition | Count right | Error | Swaps | False tracks |
| :--- | :-: | :-: | :-: | :-: |
| Bias 2°, tracker told 0.5° | 0 % | 2.28 m | 5.40 | 6.19 |
| No heading bias | 3 % | 0.82 m | 0.30 | 2.76 |
| Bias 2°, tracker told to expect 2° | 0 % | 1.73 m | 0.70 | 3.13 |
| Bias calibrated to 0.5°, told 0.5° | 0 % | 1.10 m | 0.20 | 3.40 |

Confirmed for identity swaps (5.4 → 0.2-0.7) and for half the false tracks. **But about 3 false tracks remained with no bias at all**, so there was a second cause. (A labelling slip of mine: the "no heading bias" row still had 0.7 m of position error on teammates.)

**Lesson.** A filter must be told the truth about its errors. An unmodelled bias does not just add error; it breaks association, and broken association multiplies tracks.

### 7.4 Cause 2: a vague track looks like a good match for everything

One change at a time from the no-bias baseline:

| Change | Count right | False tracks |
| :--- | :-: | :-: |
| Baseline | 3 % | 2.76 |
| No detector false alarms | 48 % | 0.95 |
| **Drone size known** | **88 %** | **0.13** |
| Merge within 6 m instead of 3 m | 8 % | 2.10 (and swaps rose to 1.1) |
| Confirm after 10 sightings instead of 5 | 5 % | 2.37 |
| Wider gate | 8 % | 2.51 |
| Drop unseen tracks after 1 s | 17 % | 1.93 |

Tuning the track-management numbers barely helped. Two things did: knowing the size, and having no false alarms. Together they point at one mechanism. A track born from a single sighting with unknown size is uncertain by ±55 % along its line of sight. I was matching on Mahalanobis distance alone, and by that measure **a vague track is close to anything near its line**, so a false alarm's track stole real drones' sightings and got confirmed.

The standard remedy is to match on likelihood: distance plus the log-determinant of the innovation covariance, which charges a track for being vague. Result with no bias: false tracks 1.61 → 0.08, count right 13 % → 91 %.

The unit test for this failed when first written, usefully. With perfect sightings the established track matches with distance exactly zero and cannot be beaten. The stealing needs ordinary measurement error. With a 1.5σ error added: on likelihood the established track keeps 10 of 10 sightings; on distance alone, 0 of 10.

**Lesson.** When sweeping parameters changes little and removing a *condition* changes a lot, the fault is in the method, not the settings.

### 7.5 A bug fixed by accident

The "distance alone" figure in §7.4 (1.61) did not match the same configuration's earlier figure (2.76). The only code change in between: pairings outside the gate used to keep their real, large distances in the matching problem; now they all get the same forbidden cost. Before, a far-out-of-gate pairing could tip the optimiser into a worse in-gate choice, and was then rejected anyway. I did not set out to fix that. It was found only because a number failed to reproduce.

**Lesson.** When a "before" number will not reproduce, something changed. Find out what.

### 7.6 Two cameras before a track counts

With the bias present and declared (5 observers):

| Rule | Count right | Swaps | False tracks |
| :--- | :-: | :-: | :-: |
| Any one camera may confirm | 55 % | 1.10 | 1.12 |
| Two cameras needed | 71 % | 0.80 | 0.85 |
| + drop unseen tracks after 1 s | 73 % | 0.80 | 0.81 |
| + merge within 5 m | 72 % (found 99 %) | 0.60 | 0.45 |

Adopted the two-camera rule (with one observer it has to stay at one). Left the merge distance at 3 m: 5 m starts merging real drones.

### 7.7 Final measurements

`python tools/stage4_checks.py` (~20 min). 8 seeds × 60 s; means over seeds. Defaults: 3840-px camera, heading bias 2° and declared, size unknown, 0.02 false alarms per camera frame, ideal link, two cameras to confirm.

| Configuration | Found | Count right | Error (m) | Swaps per drone per minute | False tracks |
| :--- | :-: | :-: | :-: | :-: | :-: |
| 1 observer | 12 % | 77 % | 6.10 | 0.69 | 4.09 |
| 2 observers | 92 % | 60 % | 2.61 | 1.08 | 0.85 |
| 3 observers | 99 % | 76 % | 1.99 | 0.42 | 0.33 |
| **5 observers** | **100 %** | **86 %** | **1.67** | **0.54** | **0.41** |
| 5, headings calibrated to 0.5° | 100 % | **93 %** | **1.01** | 0.21 | 0.08 |
| 5, link 0.5 s late + bursty loss | 99 % | 83 % | 2.02 | 0.39 | 0.48 |
| 5, drone size known | 100 % | 76 % | 1.63 | 0.93 | 0.54 |
| 5, 1280-px camera | 99 % | 84 % | 1.96 | 0.63 | 0.27 |
| 5, ten times the false alarms | 100 % | 30 % | 1.66 | 0.24 | 1.28 |
| 5, any one camera may confirm | 100 % | 79 % | 1.67 | 0.63 | 0.51 |
| 5, tracker not told about the bias | 100 % | 35 % | 2.12 | 2.61 | 1.66 |
| 5, matching on distance alone | 100 % | 28 % | 1.68 | 0.60 | 1.34 |

* **Five cameras do the job:** every drone found, 1.7 m error, the right count 86 % of the time, an identity swap about once every two minutes per drone.
* **Calibrating headings is the biggest single improvement** (count right 93 %, error 1.0 m, false tracks 0.08), as in stage 3. It is stage 7.
* **One observer counts the drones but misplaces them.** It holds about the right number of tracks (count right 77 %), yet only 12 % are within 8 m of a drone: the direction is right and the range rests on the size guess (prior 0.45 m, truth 0.77 m). The scoring calls those tracks "false", which undersells them: they are right in direction and wrong in range.
* **The lossy link costs little** (2.0 m against 1.7 m), because late sightings are used at their own time.
* **False alarms are the weak point.** Ten times the detector's false-alarm rate (0.2 per camera frame) drops the correct count to 30 %. Real detector false-alarm rates over ground clutter are unmeasured (stage 5).
* **The two method fixes matter as much as the hardware:** without the bias declared, 35 %; matching on distance alone, 28 %.
* **Not established:** knowing the drone's size scored *lower* on count (76 % against 86 %) and the 1280-px camera did about as well as the 4K one. With 8 seeds, differences of ten points are within the spread between seeds (worst-seed false tracks reach 3.2-3.4 in several rows). The camera result also differs from stage 3 because these drones are all 0.77 m (8 px at 62 m with the poor camera) and five cameras cover for each other; stage 3 used 0.25-0.8 m drones.

**Worked.** Sharing sightings instead of tracks; likelihood matching; declaring the bias; two-camera confirmation; late data at its own time (carried over from stage 3).

**Did not work.** Matching on distance alone; an undeclared bias; tuning track-management numbers to fix a method fault; a merge radius large enough to hide duplicates (it merges real drones).

**Not done / open.**
* Teammates also appear in each other's cameras. This trial leaves them out; stage 7 removes them with the radio cue.
* Bias is declared as extra noise. That is honest but crude: a bias is constant, not random. Estimating it (stage 7) is the proper treatment.
* The cameras stare at the middle of the area. No gaze control.
* The trial runs at about 12 s per simulated minute with five cameras in Python. The cost on a Pi 5 is unmeasured.
* 8 seeds is too few to separate rows that differ by ten points.

**Lessons.**
* "Every drone found" is not success. Count, identity and false tracks have to be scored too; the first run was 100 % found and wrong in every other way.
* Diagnose false tracks by asking what they are (here: where they sit relative to real drones) before changing anything.
* Removing a condition (no bias, known size, no false alarms) locates a cause faster than sweeping parameters.
* A test that passes while a core part is broken is worse than no test, because it is trusted.

---

## 8. Handoff: documents brought up to date (2026-10-04)

**Intent.** Leave the project so that a new chat, with no memory of this one, can continue: know the state, the rules, how to run things, what is unfinished and what the user has to decide.

**What was done.**
* Doc 03: §7, open items and the choice of next stage.
* README: status, the decisions table (outdoors decided; camera and UWB/RTK open; tag set aside).
* Docs 01 and 02: a "later status" line each, so their older status notes do not mislead; doc 02 §7 marked for which questions are answered or set aside.
* Tests re-run: 27 pass. Python 3.12.0, numpy 2.2.6, scipy 1.17.1.

**Did not work / mistakes found while doing it.**
* The docs said the tracker had **11** tests; it has **9** (11 is the geometry count). The docs were written from memory instead of from the test run. Corrected in doc 03 before this entry.
* The intermediate tables in §7.2-7.6 were produced by one-off calls to `run()` in `tools/stage4_checks.py` and the calling scripts were not kept. The final table reproduces from `main()`; the intermediate ones need the calls rebuilt from the row descriptions, and §7.2 cannot be reproduced at all without undoing the fixes.
* README still listed "indoors or outdoors" as open and described mode 2 as tag, a day after both were settled. Status lines had been updated; the body had not.

**Lessons.**
* Copy counts and numbers from program output, not from memory.
* Keep the script behind every table that goes into a document, including the intermediate ones.
* When a decision is made, search the documents for every place the old state is written, not only the status line.

---

## 9. Repository, reproduction, a realistic detector, and more seeds (2026-10-05)

**Intent.** Put the work in version control; check that every documented number still comes out of its script; ask whether the five-on-five tracker survives detector numbers measured on real images rather than assumed; and settle, with more seeds, the rows doc 03 §6 could not separate.

**What was done.**
* First commit, pushed to the private repository [JoshChomps/Drone-to-Drone](https://github.com/JoshChomps/Drone-to-Drone).
* All four reproduction scripts re-run; outputs kept in `results/` (`phase2_checks.txt`, `research_checks.txt`, `stage3_checks.txt`, `stage4_checks.txt`).
* `tools/stage4_checks.py`: `run()` gained `pd_max`, `w50` (the detection curve) and `width_bias` (boxes under 8 px drawn too wide). Defaults leave every earlier number unchanged; checked on two short runs before and after the edit.
* `tools/stage4_detector_checks.py` (new, parallel): 11 detector rows at 16 seeds and 3 rows at 32 seeds. Output: `results/stage4_detector_checks.txt`.

**Where the detector numbers come from.** The sibling project (`../.Drone Detection`, its notes of 2026-10-04): background false alarms at 90 % recall of 46-159 per 1000 frames (0.05-0.16 per frame; stage 4 assumed 0.02); recall under blur, noise and JPEG of ~0.32 at 8.5 px, 0.75 at 14 px, 0.85 at 34 px (fitted here as 0.85 at most, half at 10 px; stage 4 assumed 0.95, half at 8 px); box width +25 % on drones under 8 px. **These were measured on ground cameras looking at sky, not air to air.** They are a stress test, not a measurement of this system.

**A prediction made before running it.** With the 4K camera the farthest drone in this scenario is about 110 m away, so its box is about 13 px. The width bias (under 8 px only) should therefore change nothing. The short smoke run gave identical numbers with and without it, and so did the 16-seed row. With the 1280-px camera (f = 640 px; 8 px at 62 m) it does apply.

**Measurements** (5 observers, 16 seeds × 60 s; count right with a 95 % interval over seeds):

| Configuration | Found | Count right | Error (m) | Swaps | False tracks |
| :--- | :-: | :-: | :-: | :-: | :-: |
| Baseline (doc 03 §6) | 100 % | 82 % [67-97] | 1.68 | 0.87 | 0.33 |
| False alarms 0.05 per frame | 100 % | 76 % [60-92] | 1.70 | 0.61 | 0.33 |
| False alarms 0.10 | 100 % | 68 % [53-82] | 1.68 | 0.36 | 0.55 |
| False alarms 0.16 | 99 % | 42 % [30-55] | 1.70 | 0.72 | 0.93 |
| Degraded detection curve | 100 % | 79 % [62-97] | 1.68 | 0.83 | 0.49 |
| Box width +25 % under 8 px | 100 % | 82 % [67-97] | 1.68 | 0.87 | 0.33 |
| All three (FA 0.10) | 100 % | 65 % [47-82] | 1.74 | 0.63 | 0.64 |
| **All three, headings calibrated to 0.5°** | 100 % | **94 % [90-99]** | **0.97** | 0.07 | 0.06 |
| FA 0.16, three cameras to confirm | 99 % | 49 % [36-63] | 1.70 | 0.69 | 0.77 |
| 1280-px camera | 99 % | 81 % [68-93] | 2.06 | 1.12 | 0.29 |
| 1280-px camera, all three (FA 0.10) | 99 % | 69 % [57-80] | 2.16 | 0.51 | 0.45 |

More seeds (32 × 60 s):

| Configuration | Count right | Error (m) | Swaps | False tracks |
| :--- | :-: | :-: | :-: | :-: |
| Baseline | 75 % [64-87] | 1.70 | 0.74 | 0.46 |
| Drone size known | 64 % [51-78] | 1.66 | 1.16 | 0.57 |
| 1280-px camera | 75 % [63-87] | 2.08 | 0.97 | 0.45 |

**Worked.**
* **Calibrated headings make the tracker robust to a realistic detector:** 65 % → 94 % with all three detector effects. The gate shrinks when the heading error is small, so a false alarm rarely fits an existing track and a real sighting rarely misses its own. This supports stage 7 (entry 10), which was started in parallel at the user's request before this row came in.
* The reproduction scripts reproduce: stage 3, phase 2 and the research trials match the documents (e.g. stage 3: 21.1 m, 6.3 m, 1.10 m, as in doc 03 §5).

**Did not work.**
* **False alarms remain the weak point without calibration:** 0.16 per frame gives 42 %. Asking for three cameras instead of two before confirming a track recovers only 7 points (49 %).
* **The doc 03 baseline of 86 % was a lucky draw.** The same configuration gives 86 % on 8 seeds, 82 % on 16 and 75 % on 32. The 95 % interval on 16 seeds is ±15 points.

**Settled and not settled by the extra seeds.** The 1280-px camera counts as well as the 4K one (75 % each) but places drones worse (2.08 m against 1.70 m). That is consistent, since the 0.77 m drones here stay above 8 px out to 62 m even with the poor camera. "Size known" scores lower again (64 % against 75 %), but the intervals still overlap: **not established**. A paired comparison (same seeds, difference per seed) would be the next way to test it.

**Lesson.** Eight seeds were enough to see large effects (the method faults in §7) and not enough to quote a baseline to the nearest point. Report the interval with the number.

---

## 10. Stage 7: teammate cueing and heading calibration (2026-10-05)

**Intent.** Teammates also appear in each other's cameras, and stage 4 left them out. Use their shared positions to (1) recognise them, so they are not tracked as unknown drones, and (2) measure each camera's heading bias, the largest error in stages 3 and 4. This is doc 02 §4's "radio-cued vision", without clock offset for now.

**What was done.**
* `d2d/cue.py`: `HeadingCalibrator`, one per camera. It projects each teammate's shared position into the image with the bias-corrected attitude (the cue). It gates every box on pixel position and on box size, since a teammate's size and range are both known. A box in exactly one cue's gate, and in no other, is labelled as that teammate. A box in a gate but ambiguous is withheld from the tracker and not used for calibration. Each labelled teammate's azimuth residual updates a scalar Kalman filter on the heading bias. A cue in view with no box counts as a miss (the health flag).
* `tests/test_cue.py`: 7 tests.
* `tools/stage4_checks.py`: `run()` gained `mates` (observers visible to each other), `cue` (`None`, `"fixed"`, `"calibrate"`) and `cal_kw`, plus labelling, heading and tile scoring. Defaults still reproduce doc 03 §6 exactly (checked on two short runs, and by the full re-run in `results/stage4_checks.txt`).
* `tools/stage7_checks.py`: the trial (`main`, 11 rows) and the declared-floor experiment (`floor`, 4 rows), 16 seeds each. Outputs: `results/stage7_checks.txt`, `results/stage7_floor_checks.txt`.

### 10.1 The tests passed first time, so they were broken on purpose

| Deliberate break | Caught by |
| :--- | :--- |
| Residual sign flipped | `test_converges_to_the_bias` |
| No box-size check in the gate | `test_box_size_rejects_a_drone_on_the_same_line` |
| No ambiguity rule | `test_one_box_in_two_cues_is_ambiguous` |
| Misses not counted | `test_missed_cue_is_counted` |
| **Cues projected with the uncorrected attitude** | **Nothing, at first** |

Why the last one escaped: with a 2° bias the teammate still sits inside the gate, which allows for 1 m of position error (0.6° at 100 m) and 0.5° of roll/pitch noise even after the bias is learned. A test with a 6° bias (`test_keeps_labelling_after_a_large_bias_is_learned`) now catches it.

### 10.2 Results

Five observers, five other drones, 3840 px, heading bias 2°, size unknown, 0.02 false alarms per frame unless stated; 16 seeds × 60 s. *Mate ok / leak*: share of teammate boxes labelled as the right teammate, and share sent to the tracker as unknown. *Tiles*: 640-px tiles per frame needed to look at every cue and confirmed track (a full 4K scan is 24; a 1280 × 720 frame is 4).

| Configuration | Count right | Error (m) | Swaps | False tracks | Heading error (deg) | Mate ok | Leak | Tiles |
| :--- | :-: | :-: | :-: | :-: | :-: | :-: | :-: | :-: |
| Teammates out of view (stage 4) | 82 % [67-97] | 1.68 | 0.87 | 0.33 | 1.80 | | | |
| Teammates out of view, headings 0.5° | 94 % [90-98] | 0.95 | 0.16 | 0.07 | 0.5 (assumed) | | | |
| **Teammates in view, no cueing** | **0 %** | 1.71 | 0.80 | **6.69** | 1.80 | | | |
| In view, cued, bias declared (2°) | 54 % [33-75] | 1.73 | 0.65 | 1.16 | 1.80 | 97.7 % | 2.2 % | 12.4 |
| **In view, cued + calibrated** | **87 % [78-96]** | **0.77** | **0.14** | 0.14 | **1.80 → 0.34** | 99.2 % | 0.8 % | 9.2 |
| Cued + calibrated, bias 5° | 87 % [78-96] | 0.77 | 0.14 | 0.14 | 4.49 → 0.34 | 99.2 % | 0.8 % | 9.2 |
| Cued, bias declared, realistic detector | 50 % [33-67] | 1.73 | 0.38 | 1.05 | 1.80 | 97.6 % | 2.3 % | 12.5 |
| Cued + calibrated, realistic detector | 88 % [81-95] | 0.80 | 0.18 | 0.12 | 1.80 → 0.34 | 99.2 % | 0.8 % | 9.2 |
| Cued + calibrated, link 0.5 s late + bursty | 90 % [82-97] | 1.01 | 0.12 | 0.11 | 1.80 → 0.34 | 99.2 % | 0.8 % | 9.3 |
| Cued + calibrated, 3 observers | 95 % [90-100] | 1.15 | 0.12 | 0.07 | 1.76 → 0.28 | 99.4 % | 0.6 % | 8.0 |
| Cued + calibrated, 1280-px camera | 91 % [86-97] | 0.98 | 0.10 | 0.08 | 1.80 → 0.37 | 98.6 % | 1.4 % | 2.1 of 4 |

No other drone's box was ever taken for a teammate (0.0 % in every row).

**What this says.**
1. **Cueing is not optional.** With teammates visible and no cueing, the tracker holds about 6.7 extra tracks and never reports the right count.
2. **Cueing without calibration is not enough** (54 %). With the bias only declared, the cue windows must be wide (2° is 67 px), and 2.2 % of teammate boxes still fall outside them and leak into the tracker as unknown drones.
3. **Calibration does most of the work:** heading error 1.8° → 0.34°, position error 1.68 → 0.77 m, swaps 0.87 → 0.14 per drone per minute. The position error beats the stage 4 "headings 0.5°" row because the calibration reaches 0.34°.
4. **It is robust:** a 5° bias is learned as well as a 2° one (each camera converges within 0.5 s, long before scoring starts at 10 s). The realistic detector costs nothing measurable here (88 %), where without calibration it cost 17 points (entry 9). The late, lossy link and the 1280-px camera also hold up.
5. **Cued tiles save less than hoped:** about 9 of 24 tiles per frame (4 teammates plus 5 tracks, mostly in separate tiles), so cued looks cost ~40 % of a full scan, not the ~15 % doc 02 §5.2 implied. New drones still need a full scan now and then.

**Checked because it looked wrong.** The 5° row matched the 2° row in every column but the first heading figure. A direct comparison (seed 3) shows they differ only at the fourth to sixth decimal place: the filter forgets its starting point within 0.5 s. A real result, not a copy error.

### 10.3 The filter is overconfident, and it did not matter

A trace of one camera showed the calibrator's own uncertainty falling to 0.05° by 10 s while its real error is 0.34°, about seven times overconfident. That is expected: each teammate's GNSS error is slow (here constant for the whole run), and the filter treats it as fresh noise on every look. To protect the tracker, the calibrator declares at least `floor_deg` (0.5°). **I predicted that removing the floor would break matching the way the undeclared bias did in §7.3.** One change at a time:

| Declared floor | Count right | Error (m) | False tracks |
| :-: | :-: | :-: | :-: |
| 0° | 86 % [77-95] | 0.74 | 0.15 |
| 0.25° | 89 % [81-97] | 0.74 | 0.12 |
| 0.5° | 87 % [78-96] | 0.77 | 0.14 |
| 1° | 85 % [75-95] | 0.84 | 0.15 |

**The prediction was wrong.** The arithmetic shows why. The tracker's bearing noise already contains 0.5° of roll/pitch noise. With no floor it is told about 0.50° in total (0.03° pixel, 0.5° roll/pitch, 0.05° heading), against a true 0.60° (0.5° and 0.34° combined): an understatement of 1.2×, not the 4× of §7.3. The floor stays at 0.5° as cheap insurance, for example for drones with better roll/pitch than assumed here. The proper fix is to model GNSS error as slow (a state per teammate, or a fading memory).

### 10.4 Not established, and not modelled

* The remaining gap to the "teammates out of view, 0.5°" row (87 % against 94 %; false tracks 0.14 against 0.07) is probably the 0.8 % of teammate boxes that leak. **Not checked:** where the leaks happen (two drones fused in one box, image edges, occlusion) has not been looked at.
* Three observers scored higher than five (95 % against 87 %). One untested explanation: fewer teammates in view, so fewer leaks. The intervals overlap.
* Not modelled: shared positions arriving late or lost (they are always fresh here, even in the late-link row); clock offset; heading bias drifting during a run; teammates' GNSS error changing over time.
* The doc 02 sources behind cueing were not re-read at the source before this stage, as the project notes asked. That reading is still owed.

**Lessons.**
* A gate that is wide for good reasons (position error, roll/pitch noise) can hide a wrong projection. Test with an error larger than the gate.
* Predict before running, and keep the prediction when it fails. Here the arithmetic explained the failure in one line.
* What matters is the error downstream. The calibrator's own sigma was 7× wrong, yet the tracker was fine, because the tracker's error budget was dominated by something else.

---

## 11. CPU follow-ups while WSL waits for a reboot (2026-10-05)

**Intent.** The user enabled WSL but had not rebooted, and asked for "the CPU and minor tasks" in the meantime: the open items of doc 03 §9 that need no simulator and no GPU. One change per experiment, prediction before each run where there was one.

**What was done (code).**
* `tools/stage4_checks.py`, `run()` gained, all off by default (defaults reproduce every earlier number bit for bit; checked against the committed version on six short runs after each edit):
  * `split`: the scene (drone paths, orbit placement) drawn from its own random stream, so a camera or detector change keeps the same drones on the same seed.
  * `diag`: why each leaked teammate box leaked (`_leak`).
  * `share`: teammates' shared positions late (`delay`), lost (`bursty`), dead-reckoned (`dr`), with clock offsets (`clock_sd`), or with drifting GNSS error (`gnss_tau`). A late sighting's age now comes from the time stamps, so it carries the clock offsets too.
  * `swap_log`: every identity swap and its kind.
  * `looks`: a look scheduler (a tile budget per camera; track windows, then cues, then a round-robin sweep; `_schedule`, `_cells`). A 13th return value: time until each drone is first found.
* `d2d/cue.py`: `match()` keeps its gate terms (`last`) for diagnosis, and gained `edge`: a cue projecting up to that many sigmas outside the image is still matched (0 = the stage 7 behaviour).
* `d2d/estimate.py`: `IMMFilter`, an interacting multiple model filter (two constant-velocity models, 0.5 and 5 m/s², 5 s mean sojourn, all set before any run), a drop-in for `TargetFilter`. `d2d/track.py`: `Tracker(make_filter=...)`. `tools/stage3_checks.py`: `make_filter` and a second drone, `motion="legs"`.
* `d2d/geometry.py`: x500 height 0.25 → 0.29 m from the model file (below). The trial scripts keep 0.25 so the documented numbers stand.
* Tests: 34 → 42 (`test_imm.py` 4, `test_looks.py` 3, one more in `test_cue.py`).
* Scripts (all parallel, outputs in `results/`): `paired_checks.py`, `stage7b_checks.py`, `swap_checks.py`, `motion_checks.py`, `looks_checks.py`, `cost_checks.py`.

### 11.1 The x500's size, read from the model file

PX4 v1.17.0 pins `PX4-gazebo-models` at commit b6127f4; its `models/x500_base/model.sdf` is identical to `main` apart from magnetometer noise. From the collision boxes: rotor centres at (±0.174, ±0.174) m, so 0.492 m diagonal; 0.279 m props; **diameter 0.492 + 0.279 = 0.77 m** (doc 01's figure, now confirmed at the source); skids bottom at −0.227 m, rotor plane at +0.060 m: **height 0.29 m**, not the guessed 0.25 m. The height only sets the box's vertical extent here, which enters only the "two boxes fused" test (drones 2.5 m apart in altitude are ~96 px apart at 50 m against an 11 px box), so the documented numbers were not re-run.

Note for hardware: Holybro's X500 V2 kit (vendor page) ships 10-inch (1045) props, 500 mm wheelbase, 215 mm landing gear. With 10-inch props the real span is about 0.50 + 0.25 = 0.75 m, not the simulator's 0.77 m.

### 11.2 Paired seeds settle two rows and unsettle a third

Same seeds for both sides, scene from its own stream, difference per seed with a 95 % interval (`results/paired_checks.txt`):

| A − B | Seeds | Count right | Error (m) | Swaps/drone/min | False tracks |
| :--- | :-: | :-: | :-: | :-: | :-: |
| Size known − baseline | 64 | −2.6 ± 5.8 | **−0.05 ± 0.02** | **+0.81 ± 0.38** | +0.05 ± 0.11 |
| 1280-px − 3840-px camera | 64 | +4.0 ± 7.2 | **+0.43 ± 0.05** | +0.36 ± 0.42 | −0.06 ± 0.15 |
| Stage 7: 3 − 5 observers | 32 | +11.0 ± 12.1 | **+0.27 ± 0.08** | −0.04 ± 0.24 | −0.09 ± 0.26 |

* **Size known does not change the count** (entry 9's 64 % against 75 % was seed noise). It places drones 5 cm better, and **doubles identity swaps**, which needed explaining (§11.9).
* **The 1280-px camera counts as well and places 0.43 m worse**, now firmly.
* **"Three observers beat five" (entry 10) is not established:** +11 ± 12 points. Five observers place drones better (0.76 against 1.03 m).
* The unpaired intervals on the same data are ±6 to ±10 points; the paired ones on error are ±0.02-0.08 m. Pairing removes the scene's difficulty, which dominates the spread.

### 11.3 Where teammate boxes leak

`python tools/stage7b_checks.py leaks`: every teammate box that reached the tracker in the stage 7 cued + calibrated row, 16 seeds.

| | 5 observers | 3 observers |
| :--- | :-: | :-: |
| Leaked boxes | 903 | 272 |
| … cue just outside the image, box inside (box within 100 px of the edge) | **499** (486) | 106 (106) |
| … cue in view, box outside the gate: largest term vertical / size / horizontal | 404: 331 / 72 / 1 | 166: 153 / 13 / 0 |
| Shared position error of the in-view leaks, median | 2.1 m | 2.0 m |
| Leak share of teammate boxes | 0.8 % | 0.6 % |

* **Over half the leaks are an edge effect** the code had no case for: a teammate inside the image whose shared position projects just outside it. `match()` only considered cues inside the image.
* **The rest are the gate doing what a 99 % gate does:** pairs whose relative GNSS error is large (median 2.1 m, against the 1 m per axis the gate assumes) fall outside, mostly vertically. Not a bug.
* **Three against five observers:** the leak *rate* is similar, but every camera's sightings go to one tracker, and the number of camera-teammate pairs is 5·4 = 20 against 3·2 = 6. That is 3.3 times as many leaked boxes; measured 903/272 = 3.3. This fits the "fewer leaks" explanation of entry 10, but §11.2 shows the count difference itself is not established.

### 11.4 Shared positions late, lost, mistimed, drifting (stage 7b)

**Predictions, written before the run** (observers fly 5 m/s on 15 m circles, 1.63 m/s² towards the centre):
* 0.5 s stale: 2.5 m along track, 1.3-4.8° at 30-110 m, against a gate built for 1 m: labelling breaks.
* Dead reckoning over 0.5 s on a circle: ½ · 1.63 · 0.5² = 0.2 m: near the fresh row.
* Clock offsets, sd 0.05 s per drone: difference 0.07 s × 5 m/s = 0.35 m, small against GNSS: no effect. 0.2 s: 1.4 m, comparable to GNSS: a visible drop.
* GNSS error drifting with a 60 s time constant: ~0.013°/s at 50 m, close to the calibrator's assumed drift of 0.01°/s: little effect. 10 s: the filter lags; some drop.

`python tools/stage7b_checks.py share`, the stage 7 cued + calibrated row with one change per row, 16 seeds:

| Shared positions | Count right | Error (m) | False tracks | Heading after (deg) | Leak |
| :--- | :-: | :-: | :-: | :-: | :-: |
| Fresh (stage 7) | 87 % [78-96] | 0.77 | 0.14 | 0.34 | 0.8 % |
| 0.5 s old, used as they are | **44 %** [34-54] | 0.80 | 0.77 | 0.41 | 3.4 % |
| 0.5 s old, dead reckoned | 88 % [79-97] | 0.77 | 0.13 | 0.34 | 0.7 % |
| 0.5 s old + bursty loss, as they are | **18 %** [11-25] | 0.81 | 1.60 | 0.42 | 5.4 % |
| 0.5 s old + bursty loss, dead reckoned | 88 % [80-97] | 0.77 | 0.12 | 0.34 | 0.8 % |
| Dead reckoned, clock offsets sd 0.05 s | 90 % [84-96] | 0.77 | 0.11 | 0.34 | 0.8 % |
| Dead reckoned, clock offsets sd 0.2 s | 76 % [64-87] | 0.82 | 0.30 | 0.36 | 1.5 % |
| All link effects (sightings and positions 0.5 s late + bursty, DR, clocks 0.05 s) | 86 % [80-93] | 1.04 | 0.14 | 0.34 | 1.0 % |
| GNSS error drifts, τ = 60 s | **73 %** [58-87] | 0.79 | 0.41 | 0.33 | 1.2 % |
| GNSS error drifts, τ = 10 s | **74 %** [64-84] | 0.84 | 0.34 | 0.26 | 1.4 % |

* **Positions must be dead-reckoned** to the image time with their own velocity; then a 0.5 s-late, bursty link costs nothing measurable. Stale positions break labelling (44 %, 18 %), as predicted.
* **Clocks need to agree to well under 0.2 s**; 0.05 s is harmless. Stamping messages with GNSS time (a receiver's time pulse is good to well under a millisecond) meets this with a wide margin, so clock offset does not need estimating from the cue residuals as doc 02 §4 proposed.
* **The drift prediction was wrong.** A drifting GNSS error cut the count to 73-74 %, while the heading estimate stayed as good (0.33°, even 0.26° at τ = 10 s). Leaks went from 0.8 % to 1.2-1.4 % and false tracks from 0.14 to 0.34-0.41. Diagnosis in §11.5.

### 11.5 Why the drift prediction failed, and fixing the leaks

**First check: is it the tracker or the cueing?** The same drift in the stage 4 scene (teammates out of view, no cueing), one change per row: 82 % → 76 % with 2° headings, 94 % → 91 % with 0.5°. Both within seed noise. So drift acts through the cueing.

**The leak diagnosis with drift** (16 seeds): 1375 leaked boxes against 903; in-view gate failures doubled (823 against 404), again mostly on the vertical term, with larger position errors (median 2.6 m against 2.1 m).

**The cause: stage 7's seeds were lucky.** With fixed GNSS errors, the 16 seeds hold only 64 error draws, and leaks come from the tail. Re-drawing them: **no teammate pair in those seeds has a vertical error difference above 2 m, where 2.7 % of pairs would on average** (realized spread 0.79 m against 0.88 m expected). A drifting error visits the whole spread over a minute. So the drift rows (73-74 %) are closer to what this gate delivers, and **stage 7's 87 % was optimistic**. My prediction assumed the fixed-error seeds were representative.

**Fixing the two leak causes** (`python tools/stage7b_checks.py fix`, one change per row, 16 seeds):

| | Count right | False tracks | Leak | Other drones taken for a teammate |
| :--- | :-: | :-: | :-: | :-: |
| Stage 7 cued + calibrated | 87 % [78-96] | 0.14 | 0.8 % | 0.0 % |
| Cues up to 3σ outside the image (`edge=3`) | 94 % [90-99] | 0.05 | 0.5 % | 0.0 % |
| … + gate 99.9 % (16.27) | **97 % [95-100]** | 0.03 | **0.1 %** | 0.0 % (0.1 % withheld as ambiguous) |
| 3 observers (stage 7) | 95 % [90-100] | 0.07 | 0.6 % | 0.0 % |
| 3 observers, both fixes | 95 % [90-100] | 0.07 | 0.0 % | 0.0 % |

* The edge change removes the edge class (499 → 20 leaked boxes); the wider gate removes most of the rest (→ 141).
* **With the leaks fixed, five observers (97 %) are level with three (95 %).** That supports entry 10's untested "fewer leaks" explanation.
* **A risk the score cannot see:** with `edge=3`, one camera in seed 2 locked onto a wrong heading for the first 6 s (up to 5.3° off) before recovering. Scoring starts at 10 s, so this never shows in the table; 113 of the 539 leaked boxes in that diagnosis came from it. A cue matched just outside the image, while the heading is still unknown, can pull the estimate the wrong way. Not fixed; worth a start-up rule (e.g. no edge matches until the heading sigma is small).
**The fixes under drifting GNSS error** (`fixunderdrift`, `gateonly`; the honest test, since the drift visits the whole error spread):

| GNSS error drifts (τ = 60 s) | Count right | False tracks | Heading after | Leak | Others taken for a teammate |
| :--- | :-: | :-: | :-: | :-: | :-: |
| No fix | 73 % [58-87] | 0.41 | 0.33° | 1.2 % | 0.0 % |
| Edge only | **67 %** [50-83] | 0.95 | **0.76°** | 2.9 % | 0.0 % |
| Gate 99.9 % only | 86 % [75-97] | 0.16 | 0.33° | 0.6 % | 0.0 % |
| Edge + gate 99.9 % | 89 % [80-97] | 0.17 | 0.42° | 0.5 % | 0.1 % |
| Edge + gate 99.9 % + σ_pos 1.25 m | 92 % [86-99] | 0.11 | 0.42° | 0.4 % | 0.1 % |

(With fixed errors, the gate alone gives 94 % [90-99], against 97 % with the edge change too.)

* **The edge change alone is harmful once GNSS errors are realistic:** the start-up mis-lock seen in seed 2 now does lasting damage (heading 0.76°). With the wider gate it adds ~3 points but costs heading accuracy (0.42° against 0.33°).
* **The wider gate is the safe gain** (73 → 86 % under drift, 87 → 94 % fixed). A slightly larger assumed position error (1.25 m; two GNSS fixes of 0.7 m per axis differ by ~1 m per axis, plus drift) adds a little more. The first sign of the price: 0.1 % of other drones' boxes taken for a teammate.
* **Recommendation:** adopt gate 99.9 % now; keep `edge` off until a start-up guard exists. **Not done:** defaults are unchanged (`edge=0`, gate 99 %), so every documented row still reproduces; adopting means re-running the stage 7 tables.

### 11.6 The motion model: an IMM, checked on a drone it was not tuned on

Stage 3 chose a constant-velocity model with 3 m/s² noise on its own scenario (doc 03 §9: "tuned on the test scenario, not validated"). `IMMFilter` mixes a quiet model (0.5 m/s²) and a manoeuvre model (5 m/s²).

**Tests first, then broken on purpose** (`tests/test_imm.py`, a drone flying straight, turning at 4.8 m/s², then straight, seen by two observers):

| Deliberate break | Caught by |
| :--- | :--- |
| Model probabilities never reweighted | `test_model_probability_follows_the_manoeuvre` |
| Both models given the quiet noise | `test_model_probability_follows_the_manoeuvre`; **not** `test_better_than_either...` at first: with strict `<` it passed on floating-point noise, so it now asks for a 20 % margin |
| **No mixing** (each model runs alone; only the output is combined) | **Nothing, at first.** The output barely suffers (0.19 against 0.15 m after the turn), because it leans on the other model. The quiet model itself is left 1.8 m behind (0.2-0.3 m with mixing); `test_mixing_brings_the_quiet_model_back_after_a_turn` now checks that |
| State left unmixed, covariance still mixed | Nothing; the inflated covariance makes the quiet model catch up anyway. A partial break with little effect; accepted |

On the toy the IMM is best in every phase (RMS 0.10 m straight, 0.23 m turning, 0.15 m after, against 0.09 / 1.24 / 0.66 m for CV 0.5 and 0.15 / 0.23 / 0.20 m for CV 5).

**In the stage 3 trial** (`python tools/motion_checks.py`; two observers orbiting 90° apart, 40 seeds; "legs" is a new drone: 6 m/s straight legs of 3-8 s and 1 rad/s turns of 60-150°, i.e. 6 m/s²):

| Drone, heading bias | CV 1 | CV 3 (chosen) | CV 6 | IMM | IMM − CV 3, paired |
| :--- | :-: | :-: | :-: | :-: | :-: |
| Stage 3 drone, 2° | 3.10 m | 2.65 m | 2.53 m | 2.60 m | −0.03 ± 0.03 m |
| Stage 3 drone, 0.5° | 1.78 m | 1.10 m | 1.05 m | 1.12 m | +0.01 ± 0.02 m |
| Legs, 2° | 4.87 m | 2.84 m | 2.79 m | 2.84 m | +0.70 ± 1.52 m (one seed) |
| Legs, 0.5° | 4.27 m | 1.32 m | **1.16 m** | 1.23 m | **−0.10 ± 0.02 m**; velocity −0.24 ± 0.03 m/s |

(Median position error while seen.)
* **The 3 m/s² choice holds up on a drone it was not tuned on**, and is slightly too low: 6 m/s² is as good or better in all four cases. 1 m/s² fails badly on sharp turns (two re-acquisitions per run).
* **The IMM gives no clear gain here.** Only with calibrated headings on the sharp-turn drone does it beat CV 3 (by 0.10 m and 0.24 m/s), and CV 6 beats it there. At these noise levels the bearing errors, not the motion model, set the error. It costs ~2.5× the filter time.
* Kept: `IMMFilter` as an option (`Tracker(make_filter=IMMFilter)`); the default stays CV 3. Raising it to 6 is a one-line change that was not run through the stage 4/7 tables, so it is not made.

### 11.7 Reading owed: the sources behind cueing

Read at the source (doc 02 correction note of 2026-10-05):
* **Opromolla [72]** (Sensors 2020): heading (magnetometer) calibration of one UAV from a second vehicle seen by its camera, with **pseudorange** GNSS (DGPS or two fixes), offline: mean heading error 7.4° → 2.7°, 70-107 m apart. Doc 02's gap G3 ("only shown with carrier-phase GNSS") was wrong. What stays open for us: online, several teammates, inside the tracker; and our 0.34° is a simulation with modelled errors, not a flight result.
* **Omni-swarm [41]** (T-RO 2022): projects the swarm's estimated drone states into the image as boxes and matches them to detections (global nearest neighbour on box overlap), only for association; the detector runs on the whole image. So **cue-then-label is prior art**; "the cue decides where the detector looks" is still not found.
* **Wen et al. [59]** (arXiv 2023, abstract): clock offset estimated jointly with bearing-based mutual localisation. Given §11.4, GNSS time stamps make this unnecessary for us.
* [70], [71] (MDPI) refused the fetch (HTTP 403); still unread.

### 11.8 Hardware, checked at the source where possible

| Part | Finding | Level |
| :--- | :--- | :--- |
| Raspberry Pi Camera Module 3 Wide | IMX708, 4608 × 2592, 102° horizontal, **rolling shutter**. f = 2304 / tan 51° ≈ 1870 px: **on the 1700-1900 px target**. The fastest listed mode is 2304 × 1296 at 56 fps (f ≈ 930 px); the full-resolution frame rate is not listed | P (raspberrypi.com camera docs) |
| Raspberry Pi Global Shutter Camera | IMX296, 1456 × 1088, global shutter, 60 fps, C/CS lens. At 90° f ≈ 730 px (the "poor camera" class); f = 1870 px means a 43° lens | P (same page) |
| Raspberry Pi HQ Camera | IMX477, 4056 × 3040, rolling shutter, C/CS lens; 2028 × 1520 at 40 fps | P (same page) |
| Raspberry Pi AI HAT+ 26 TOPS (Hailo-8) | ~US$132 | S (retailer search) |
| Raspberry Pi 5 8 GB | US$80 at launch; a search summary says $125 after a 2026 memory-price rise, sources conflict | S, unresolved |
| Holybro X500 V2 ARF kit | €380; 500 mm wheelbase, 610 g frame, 2216 KV920 motors, 1045 props, ~18 min hover | S (retailer search) |
| u-blox ZED-F9P boards (SparkFun) | US$184-270; 0.01 m horizontal with RTK, 2.5 m without | S |
| Qorvo DWM3000 UWB module | ~€22.5 each; ~10 cm ranging | S |

**The camera choice is a real trade:** the only Pi camera that reaches the focal length at 90°+ is rolling-shutter (motion skew on a moving, vibrating drone; not modelled), and the global-shutter one is the 1280-px class. A third-party global-shutter sensor of ~4K, or two narrower cameras, are the alternatives.

### 11.9 Why a known size doubled identity swaps

`python tools/swap_checks.py` logs every swap (32 seeds, same seeds for every row):

| | Count right | Swaps/drone/min | Swaps | Flapping (A → B → A) | Old track still alive | The two tracks apart (median) |
| :--- | :-: | :-: | :-: | :-: | :-: | :-: |
| Baseline, heading bias 2° | 64 % | 1.16 | 154 | 79 | 118 | 6.0 m |
| Size known, 2° | 60 % | 2.17 | 290 | 173 | 243 | 5.2 m (97 under 5 m) |
| Baseline, merge within 5 m | 66 % | 1.35 | 180 | 46 | 82 | 6.3 m |
| Size known, merge within 5 m | 81 % | 1.40 | 186 | 45 | 84 | 6.2 m |
| Baseline, headings 0.5° | 85 % | 0.45 | 60 | 16 | 22 | 4.0 m |
| Size known, headings 0.5° | 93 % | 0.49 | 65 | 18 | 29 | 3.5 m |

* **Most "swaps" are not two drones exchanging identities.** The drone's nearest other drone is a median 16 m away. Two long-lived tracks, both 3-4 m from one drone and ~6 m apart, take turns being the closer one, so the score flips between them: one drone's sightings are split between two tracks.
* **Why a known size makes it worse.** Each camera has its own 2° heading bias, which puts the drone 2-3.5 m sideways at these ranges, differently for each camera. With the size unknown, the filter can absorb the disagreement in range. With the size pinned (σ = 1 cm), it cannot, so groups of cameras settle on separate tracks. **Prediction: with headings calibrated, the extra swaps disappear.** They do: 0.49 against 0.45 swaps per drone per minute.
* **Merging within 5 m is not a fix:** flapping halves, but merged tracks are reborn within a second (new tracks' median age at a swap 0.2-0.9 s), so total swaps rise for the baseline. It did lift the size-known count to 81 % and cut false tracks; one row, not explored further.
* Not established: size known scores 93 % against 85 % with calibrated headings (same seeds, difference not computed per seed).

### 11.10 The look scheduler: searching less scored better

Each camera gets a budget of 640-px tiles per second (24 tiles a frame is the whole 4K image at 10 Hz). Windows around tracks come first, then teammates' cues, then a round-robin sweep. Only searched tiles produce detections or false alarms. `python tools/looks_checks.py`, stage 7 cued + calibrated, 16 seeds:

**First version: starvation.** The track windows were served in a fixed order. At 15 tiles/s (1.5 a frame) the first track's window took the whole budget every frame, the others got no looks and died: 76 % [68-85], worse than a plain sweep (86 %). The fix: each camera serves the track it searched longest ago first. Both versions are kept (`results/looks_checks_v1_fixed_order.txt`; `rotate=False`). `tests/test_looks.py` checks the budget, the skip-if-it-does-not-fit rule and the sweep.

| Budget per camera | Count right | Error (m) | False tracks | Time to find (s) |
| :--- | :-: | :-: | :-: | :-: |
| Whole frame every frame (240 tiles/s) | 87 % [78-96] | 0.77 | 0.14 | 0.0 |
| 60 tiles/s | 96 % [92-100] | 0.77 | 0.04 | 0.2 |
| 30 tiles/s | 97 % [95-100] | 0.83 | 0.02 | 0.6 |
| **15 tiles/s** (doc 02's Pi 5 CPU figure) | **99 % [98-100]** | 0.96 | 0.00 | 1.0 |
| 15 tiles/s, cues every 5th frame | 99 % [98-100] | 0.97 | 0.00 | 0.9 |
| 15 tiles/s, fixed track order (first version) | 76 % [68-85] | 1.51 | 0.17 | 1.5 |
| 15 tiles/s, sweep only | 86 % [78-93] | 1.93 | 0.07 | 2.7 |
| 60 tiles/s, sweep only | 99 % [98-100] | 1.00 | 0.01 | 0.3 |

**Why less beats more. Hypothesis:** false alarms and leaked teammate boxes need 5 hits from 2 cameras to become a track; searching less gives them fewer chances, while real drones are searched where they are. **Test, one change: the whole frame every frame, but a stricter confirmation:**

| Whole frame, confirm after | Count right | False tracks |
| :--- | :-: | :-: |
| 5 hits, 2 cameras (stage 7) | 87 % | 0.14 |
| 10 hits | 97 % [94-100] | 0.03 |
| 20 hits | 97 % [95-99] | 0.02 |
| 5 hits, 3 cameras | 97 % [94-100] | 0.03 |

The hypothesis holds: the confirmation rule was too loose for five cameras at 10 Hz (5 hits arrive within a single frame), and the scheduler hid that. **What the scheduler really buys** is compute: 15 tiles/s per camera keeps the count and costs 0.2 m of error (0.96 against 0.77 m), because each drone is seen less often. Time to find stays under 2 s in this scenario, where every drone is in view from the start; a drone entering the view later is not tested.

### 11.11 Cost per frame (laptop only)

`python tools/cost_checks.py`: the stage 7 trial, one process, CPU otherwise idle (1 % load), i7-12650H. Per call, median [99th percentile]: **`Tracker.step` 19.9 [41.0] ms** (all five cameras' sightings, ~5 tracks), **`HeadingCalibrator.step` 0.79 [2.09] ms**. With IMM tracks the tracker takes 49.5 [88.0] ms. A frame is 100 ms at 10 Hz.

**Not measured on a Pi 5** (none available). The code is plain numpy on 7×7 matrices, so Python overhead dominates; a Pi 5 core is likely several times slower than this laptop's, which would put the constant-velocity tracker at a large share of the frame and the IMM over it. That is a guess, not a measurement. Vectorising the per-track loops, or a compiled core, are the options if the board confirms it.

### 11.12 Not established, and not done

* Every stage 7b result is 16 seeds on one scenario; the leak-fix rows are within each other's intervals in places (86 % [75-97] against 89 % [80-97]).
* The look scheduler's scenario has every drone in view from the start: finding a drone that arrives later is not tested.
* The leak fixes, the stricter confirmation and CV 6 m/s² are **not adopted as defaults**; each needs the stage 4/7 tables re-run.
* Gaze control: not attempted. Here the gaze is fixed on the middle of an area all drones stay in, so a gaze rule has nothing to win; it needs a scenario where drones leave the view.
* The x500 height change (0.29 m) was not run through the trial scripts (argued negligible in §11.1).
* History scripts for build log §7.2-7.6 were not rebuilt (the §7.5 fix cannot be undone by a flag; the rest are flag combinations of `run()`).

**Lessons.**
* **A diagnosis beats a guess.** The leak log turned "probably the 0.8 % leaks" into two causes, one of them a missing case in the code.
* **Fixed random errors are a sample, not the distribution.** Sixteen seeds gave 64 GNSS errors and none of them large; the drift run, meant to test something else, exposed it. When a result depends on a tail, check the tail was drawn.
* **A fix that helps on the lucky seeds can hurt on realistic ones** (the edge change). Test fixes under the hardest realistic condition, not only the reference row.
* **When less input scores better, look for a rule that is too permissive.** Searching fewer tiles "worked" because the confirmation rule was too loose; the stricter rule with the whole frame did the same.
* **A scheduler needs fairness.** A fixed priority order starves everything after the first item when the budget is short; the first version's numbers were worse than having no priorities at all.
* **Pairing seeds turned two "not established" rows into answers** and showed that a third ("3 beat 5") was noise.
* **Patches can fail silently in background jobs.** A patch script whose anchor text had changed raised an error in a background command's output, and the job then re-ran the old code for ten minutes. Check that a patch applied before launching the run that depends on it.

