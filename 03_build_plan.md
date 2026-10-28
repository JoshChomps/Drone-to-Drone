# 03: Build Plan: Swarm Detection and Identification

> [!NOTE]
> **Status 2026-10-05: stages 2, 3, 4, 4b and 7 are done, plus CPU follow-ups (42 tests). Results: §5 (locating one drone), §6 (tracking five), §8 (a realistic detector, more seeds, and stage 7: teammate cueing and heading calibration), §10 (stage 7b, leak fixes, look scheduler, motion model, paired seeds). Open items and what comes next: §10.2 (§7 and §9 are earlier versions, kept for the record). The step-by-step history, including what failed, is in the [build log](04_build_log.md). Stage 0 (install) is with the user.** Research behind this plan: [doc 02](02_research_survey_and_novel_directions.md). Simulator plan: [doc 01](01_phase2_simulation_plan.md).

## 1. Scope (agreed with the user, 2026-10-03)

The project is **drone-to-drone detection, with the emphasis on a swarm seeing another group of drones**: detect them, tell teammates from others, keep identities, estimate position and velocity, and merge what five cameras see into one picture.

| In scope | Out of scope |
| :--- | :--- |
| Detecting drones from a moving drone, on a small onboard computer | Steering drones to chase or close on drones that are not sharing positions, using camera estimates |
| Tracking several identical-looking drones with stable identities | Assigning pursuers to camera-detected drones |
| Position and velocity from one camera (box size) and from two (triangulation), scored against ground truth | |
| Sorting detections into teammates (they match a shared position) and others | |
| Merging sightings across the team over a lossy link | |
| Vision-only flocking among our own drones (spacing and avoidance from camera estimates) | |
| The safety layer and altitude bands under everything | |
| A catching game **on shared positions**, scored by a mutual range sensor (UWB), if the user returns to it | |

While identifying, drones fly their own pre-planned paths (formation, search pattern, flocking). They may turn in place to look. They do not navigate towards what they see.

**Set aside for now (user, 2026-10-03):** the tag game and the catcher-to-runner allocation problem. The user finds the shared-position version a plain control problem and prefers the vision work.

## 2. Stages

| Stage | What gets built | How we know it works | Needs | State |
| :-: | :--- | :--- | :--- | :--- |
| **0** | WSL2, PX4 v1.17, Gazebo Harmonic, Python venv | One simulated x500 takes off and lands (doc 01 S0) | User runs doc 01 §9 | With the user |
| **1** | Multi-drone simulation with logging (doc 01 S1-S2) | Real-time factor for 3 and 5 drones; ground truth read back | CPU | After stage 0 |
| **2** | Camera geometry: frames, projection, boxes, bearing, range from box, cue error | Hand calculations; tests fail when the code is broken on purpose | CPU | **Done** |
| **3** | Estimators: one-camera filter with box size, two-camera triangulation, late data used at its own time | Doc 02 §5.4 numbers reproduced in 3-D, then on simulated flights with PX4's attitude errors | CPU | **Done** (§5) |
| **4** | Multi-drone tracker with identities | Identity switches per minute when drones cross or leave view | CPU | **Done** (§6) |
| **4b** | Team fusion: five drones' sightings merged into one list of others | No double counts, no lost drones, under doc 01's bursty link | CPU | **Done** (§6) |
| **5** | Detector on real air-to-air images, importing the sibling's `cuas` | Recall by pixel size; false alarms over ground clutter | GPU or Colab; **ask first** | |
| **6** | Measured detector noise fed back into stages 3-4b | Estimates with realistic misses and box jitter | CPU | |
| **7** | Teammate cueing and heading calibration (doc 02 §4) | Tiles per second saved; heading error before and after | CPU | **Done** (§8); clock offset not yet |
| **8** | Vision-only flocking in simulation, 3 then 5 drones | Zero contacts over ≥ 10 seeds; spacing error | CPU | |
| **9** | Two real drones, then more | Vision estimates against GNSS truth in flight | Hardware | |

Stages 2-4b push simulated truth through the camera model, so they need no image rendering and no GPU.

**Known risk:** five simulated x500s may not run in real time on this laptop (doc 01 §2.3). Stage 1 measures it; the fallback is agents on simulation time.

## 3. Code

* Package `d2d/`; tests in `tests/` (`python -m unittest discover -s tests`); number-reproducing scripts in `tools/`.
* **Frames** (`d2d/geometry.py`): world = ENU about a shared reference; body = FLU; camera = optical (x right, y down, z forward). `px4_attitude` converts PX4's NED/FRD quaternion.
* **Stage 7 contents:** `d2d/cue.py`: `HeadingCalibrator` (cue, label, calibrate, health count).
* **Stage 4 contents:** `d2d/track.py`: `Sighting`, `Track`, `Tracker` (needs scipy for the assignment).
* **Stage 3 contents:** `d2d/estimate.py`: `TargetFilter`, `bearing_model`, `angle_model`, `triangulate`.
* **Stage 2 contents:** `lla_to_enu` (exact), `attitude`, `px4_attitude`, and `Camera` with `project`, `in_view`, `bearing`, `box`, `subtended_angle`, `range_from_box`, `cue_sigma_px`.
* **Assumption to replace:** the x500's height (0.25 m) in `box` is a guess. Width (0.77 m) is from the model file.
* The repository is initialised; nothing is committed until the user asks.

## 4. Hardware (nothing to buy before stage 8)

Current leaning, **prices and parts not yet checked at the source:** a PX4 kit airframe with a Pi 5 and a global-shutter camera **with a focal length of about 1700-1900 px** (§5, finding 1; a 1280-px, 90° camera is not enough), and possibly a detector accelerator (§5, finding 2); RTK on at least two drones for ground truth; UWB modules as an optional range reference. Operating height: above local tree height and below Canada's 122 m limit, in separate altitude bands.

## 5. Stage 3 Results (2026-10-04)

Code: [`d2d/estimate.py`](d2d/estimate.py) (`TargetFilter`: position, velocity and physical width; bearing and box-size updates from any observer at any age; `reacquire`; `triangulate`), 7 tests in [`tests/test_estimate.py`](tests/test_estimate.py). Numbers: `python tools/stage3_checks.py` (~4 min).

**Scenario:** a drone that shares nothing (0.25-0.8 m wide, 4 m/s, turning continuously, ~50 m altitude) seen by observers at 60 m, 15-70 m away (median 44 m), flying pre-planned paths and turning in place to look. Heading bias 2°, roll/pitch noise 0.5°, box noise 1 px centre and 1.5 px width, detection probability falling with box width (half at 8 px). 40 seeds; RMS error while seen, median [90th percentile].

| Configuration | 1280 px, 90° | | 3840 px, 90° | |
| :--- | :-: | :-: | :-: | :-: |
| | position (m) | velocity (m/s) | position (m) | velocity (m/s) |
| Share of frames in which the drone is detected | 46 % | | 89 % | |
| 1 observer, bearing only (hovering) | 21.1 [38.4] | 4.3 | 24.7 [36.4] | 4.6 |
| 1 observer, bearing + box size, hovering | 10.5 [27.0] | 3.1 | 7.3 [19.8] | 1.7 |
| 1 observer, bearing + box size, orbiting | 12.0 [34.9] | 3.2 | 7.6 [21.6] | 2.0 |
| 1 observer, bearing + box size, **size known** | 6.3 [12.7] | 2.9 | 2.5 [4.1] | 1.6 |
| 2 observers 30° apart, bearing + box size | 7.4 [14.3] | 2.1 | 5.8 [11.0] | 1.4 |
| 2 observers 90° apart, bearing only | 4.3 [11.5] | 1.6 | 2.9 [7.4] | 1.0 |
| 2 observers 90° apart, bearing + box size (hovering) | 3.4 [8.2] | 1.7 | 2.3 [4.0] | 1.0 |
| … orbiting | 3.9 [7.4] | 1.6 | 2.7 [5.1] | 1.2 |
| … orbiting, headings calibrated to 0.5° | 2.1 [4.2] | 1.4 | **1.1 [1.9]** | 1.0 |
| … link 0.5 s late, used as if fresh | 4.0 [8.7] | 2.0 | 3.3 [5.5] | 1.5 |
| … link 0.5 s late, used at its own time | 4.3 [7.9] | 2.0 | 2.5 [5.1] | 1.7 |
| … 0.5 s late + bursty loss (9 %) | 4.2 [8.1] | 2.1 | 2.9 [7.0] | 1.6 |

Pipeline check (straight-flying drone, no noise): 0.41 m and 0.10 m/s with one observer and known size; 0.05 m and 0.07 m/s with two.

**What this says:**

1. **The camera decides more than the estimator.** With 1280 px across 90° the drone is in view 95 % of the time but too small to detect in half the frames: a 0.5 m drone is at the 8-px limit at 40 m. Tripling the focal length in pixels (3840 px across 90°, f = 1920 px) moves that limit to 120 m and the detection share from 46 % to 89 %. **Target: f ≈ 1700-1900 px**, by resolution or by a narrower lens (1920 px across 60° gives f = 1663 px but only a 36° vertical view).
2. **Compute follows from that.** A 3840×2160 frame is about 24 tiles of 640 px, so a full scan takes ~1.6 s at the sibling's 15 tiles/s on a Pi 5. Tiles cued by existing tracks and by teammates' positions (stage 7) are required, not optional, or the onboard computer needs an accelerator (not yet researched).
3. **One observer cannot range an unknown drone from a pre-planned path.** Bearing only: ~21-25 m. Adding box size: 7-12 m, which is the error of the size prior (true width 0.25-0.8 m against a prior of 0.45 m). Orbiting does not help when the other drone manoeuvres as hard as the observer, as the bearing-angle theorem says (the observer needs higher-order motion than the target). **Doc 02 §5.4's 3.8 m was optimistic:** there the observer followed at 25 m.
4. **Known size fixes one observer** (2.5 m): that is the teammate case, and also any drone whose size has been learned.
5. **Two observers about 90° apart are the practical answer:** 2.3-2.7 m with no knowledge of size, 5.8 m at 30°. **With calibrated headings, 1.1 m.** Heading bias remains the largest single error.
6. **Velocity is known to about 1 m/s at best** against a 4 m/s drone that keeps turning. Predicting one second ahead therefore carries about 1 m of error from velocity alone, on top of the position error.
7. **A track gets lost and must be re-acquired.** When the drone passes out of view and turns, the gate rejects every later sighting. Without `reacquire` the affected runs never recover.
8. **Late data at its own time:** 3.3 m → 2.5 m with the good camera; no measurable difference with the poor one, where other errors dominate.

**Mistakes made and fixed here:**

* A test put the target 194 m away after 40 s and then demanded 1 m accuracy. The filter was right; the scenario was not.
* The first full run gave ~20 m everywhere. Three separate causes, found one at a time: no re-acquisition after track loss; a camera with too few pixels; and a constant-velocity model with 1 m/s² of freedom lagging a turning drone (3 m/s² cut one-observer error from 3.6 m to 2.2 m). **The 3 m/s² value was chosen on this scenario, so it is tuned, not validated.**

**Not explained:** with unknown size, raising the model freedom also cut the one-observer error (16.6 m → 7.6 m with the good camera). I have not worked out why.

**Open after stage 3:** a turn-rate or multi-model filter (the sibling's IMM) instead of a tuned constant-velocity model; gaze control (the observer currently yaws towards the true direction); real detector noise (stage 6).

## 6. Stage 4 and 4b Results (2026-10-04)

Code: [`d2d/track.py`](d2d/track.py) (`Tracker`: one per drone, fed by its own camera and by teammates' sightings over the link), 9 tests in [`tests/test_track.py`](tests/test_track.py). Numbers: `python tools/stage4_checks.py` (~20 min). How it got here, including what failed: [build log §7](04_build_log.md#7-stage-4-and-4b-multi-drone-tracker-and-team-fusion-2026-10-04).

**Scenario:** five identical drones (0.77 m, 4 m/s, turning, 45-55 m altitude) in an 80 m-wide area; up to five observers orbiting pre-planned circles around it at 60 m, cameras fixed on the middle of the area. Heading bias 2° (declared to the tracker), size unknown, 0.02 false alarms per camera frame, one box when two drones overlap in an image. 8 seeds × 60 s. *Found*: share of drones with a confirmed track within 8 m. *Count right*: share of time exactly five tracks are reported.

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

> **Correction (2026-10-05, §8):** on 32 seeds the baseline count is 75 % [64-87], not 86 %; the 8-seed figures in this table are a lucky draw. The comparisons between rows still hold where the gaps are large.

* **Five cameras do the job:** every drone found, 1.7 m error, the right count 86 % of the time, an identity swap about once every two minutes per drone.
* **Calibrating headings is the biggest single improvement** (count right 93 %, error 1.0 m, false tracks 0.08), as in stage 3. It is stage 7.
* **One observer counts the drones but misplaces them.** It holds about the right number of tracks (count right 77 %), yet only 12 % are within 8 m of a drone: the direction is right and the range rests on the size guess (prior 0.45 m, truth 0.77 m). The scoring calls those tracks "false", which undersells them: they are right in direction and wrong in range.
* **The lossy link costs little** (2.0 m against 1.7 m), because late sightings are used at their own time.
* **False alarms are the weak point.** Ten times the detector's false-alarm rate (0.2 per camera frame) drops the correct count to 30 %. Real detector false-alarm rates over ground clutter are unmeasured (stage 5).
* **The two method fixes matter as much as the hardware:** without the bias declared, 35 %; matching on distance alone, 28 %.
* **Not established:** knowing the drone's size scored *lower* on count (76 % against 86 %) and the 1280-px camera did about as well as the 4K one. With 8 seeds, differences of ten points are within the spread between seeds (worst-seed false tracks reach 3.2-3.4 in several rows). The camera result also differs from stage 3 because these drones are all 0.77 m (8 px at 62 m with the poor camera) and five cameras cover for each other; stage 3 used 0.25-0.8 m drones.

**Settings chosen by evidence (build log §7.3-7.6):** match on likelihood, not distance alone; declare the heading bias to the tracker; require two cameras before a track counts; merge duplicates within 3 m; drop a confirmed track unseen for 3 s.

## 7. Open Items and What Comes Next (2026-10-04; superseded by §9)

**The user chooses the next stage; neither has been started.**

| Option | Why | What it needs |
| :--- | :--- | :--- |
| **Stage 7: teammate cueing and heading calibration** (recommended first) | The largest measured gain in §5 and §6, and it adds the teammates that §6's trial leaves out of the cameras' view | CPU only |
| **Stage 5: a real detector on air-to-air images** | Measures the false-alarm rate, which §6 is most sensitive to and which is assumed (0.02 per camera frame) | GPU or Colab; ask first |

**Stage 7 outline:** project each teammate's shared position into the camera; match detections to those cues; a match is a teammate, and its residual estimates the camera's heading bias and clock offset; an unmatched detection goes to the `Tracker`; a cue with no detection is a health flag. Add teammates as visible drones in `tools/stage4_checks.py`, and replace the declared bias with the estimated one.

**Open items carried forward:**

| Item | Where it came from | Proper fix |
| :--- | :--- | :--- |
| Constant-velocity model with 3 m/s² noise, tuned on the test scenario | §5 | Turn-rate or multi-model (IMM) filter, checked on a different scenario |
| More model freedom also cut the unknown-size one-observer error; cause unknown | §5 | Trace one run |
| "Size known" and "1280-px camera" rows not separated from the baseline | §6 | More seeds (8 is too few) |
| Heading bias declared to the tracker as random noise | §6 | Estimate it (stage 7) |
| Teammates not present in each other's cameras | §6 | Stage 7 |
| No gaze control (stage 3 looks at the truth; stage 4 stares at the middle) | §5, §6 | A look-direction rule driven by the tracks |
| Assumed false-alarm rate and box noise | §5, §6 | Stages 5 and 6 |
| x500 height 0.25 m is a guess | §3 | Read it from the model file |
| Cost per frame on a Pi 5 unmeasured | §6 | Time `Tracker.step` on the target board |
| Hardware prices, parts and accelerators unchecked | §4 | Check at the source before stage 8 |

## 8. Realistic Detector, More Seeds, and Stage 7 Results (2026-10-05)

How each result was reached, including a failed prediction: [build log §9, §10](04_build_log.md#9-repository-reproduction-a-realistic-detector-and-more-seeds-2026-10-05). Outputs of every script: `results/`.

**Realistic detector** (`python tools/stage4_detector_checks.py`, ~10 min with 8 processes). Numbers borrowed from the sibling detector project, measured on ground cameras looking at sky, so a stress test rather than a measurement: false alarms 0.05-0.16 per frame, recall falling under blur/noise/JPEG, box width +25 % under 8 px. 16 seeds:

| Configuration | Count right | Error (m) | False tracks |
| :--- | :-: | :-: | :-: |
| Baseline | 82 % [67-97] | 1.68 | 0.33 |
| False alarms 0.10 / 0.16 per frame | 68 % / 42 % | 1.68 / 1.70 | 0.55 / 0.93 |
| All three detector effects (FA 0.10) | 65 % [47-82] | 1.74 | 0.64 |
| **All three, headings calibrated to 0.5°** | **94 % [90-99]** | **0.97** | **0.06** |
| FA 0.16, three cameras to confirm | 49 % [36-63] | 1.70 | 0.77 |

The width bias changes nothing with the 4K camera (no box is under 8 px at these ranges), as predicted before the run.

**More seeds (32):** the baseline is **75 % [64-87]**, not the 86 % of §6 (8 seeds). The 1280-px camera counts as well (75 %) but places worse (2.08 m against 1.70 m). "Size known" is lower again (64 % [51-78]) but still **not established**.

**Stage 7** (`python tools/stage7_checks.py main`, ~18 min; `floor`, ~6 min). Teammates now visible in each other's cameras; 16 seeds:

| Configuration | Count right | Error (m) | Swaps | False tracks | Heading (deg) | Tiles of 24 |
| :--- | :-: | :-: | :-: | :-: | :-: | :-: |
| Teammates out of view (stage 4) | 82 % | 1.68 | 0.87 | 0.33 | 1.80 | |
| Teammates out of view, headings 0.5° | 94 % | 0.95 | 0.16 | 0.07 | 0.5 assumed | |
| Teammates in view, no cueing | **0 %** | 1.71 | 0.80 | **6.69** | 1.80 | |
| In view, cued, bias declared | 54 % | 1.73 | 0.65 | 1.16 | 1.80 | 12.4 |
| **In view, cued + calibrated** | **87 % [78-96]** | **0.77** | **0.14** | 0.14 | **0.34** | 9.2 |
| … bias 5° | 87 % | 0.77 | 0.14 | 0.14 | 4.49 → 0.34 | 9.2 |
| … realistic detector | 88 % | 0.80 | 0.18 | 0.12 | 0.34 | 9.2 |
| … link 0.5 s late + bursty | 90 % | 1.01 | 0.12 | 0.11 | 0.34 | 9.3 |
| … 3 observers | 95 % | 1.15 | 0.12 | 0.07 | 0.28 | 8.0 |
| … 1280-px camera | 91 % | 0.98 | 0.10 | 0.08 | 0.37 | 2.1 of 4 |

* **Teammates must be cued:** without it, the count is never right.
* **Calibration is the gain:** heading 1.8° → 0.34° within 0.5 s, position error 1.68 → 0.77 m, and robustness to the realistic detector (88 % against 65 %).
* 99.2 % of teammate boxes are labelled correctly; no other drone's box was ever taken for a teammate; the 0.8 % that leak probably account for the gap to 94 % (not checked).
* **Cued looks cost ~40 % of a full scan** (9 of 24 tiles), not the ~15 % doc 02 implied: teammates and tracks fall in separate tiles.
* The calibrator's own uncertainty is ~7× optimistic (0.05° claimed, 0.34° real: GNSS errors are slow). Declaring a 0.5° floor to the tracker turned out not to matter (85-89 % for floors of 0-1°), because roll/pitch noise already dominates the tracker's bearing noise.

## 9. Open Items and What Comes Next (2026-10-05, morning; superseded by §10.2)

| Option | Why | Needs |
| :--- | :--- | :--- |
| **Stage 0 → 1: simulator** | Everything so far is a hand-written point-mass simulation. PX4 + Gazebo gives real flight dynamics, attitude errors and timing (doc 01) | WSL2 (one admin step and a reboot by the user), then CPU |
| **Stage 5: real detector on air-to-air images** | False alarms are still the weak point without calibration; their real rate is unmeasured | Colab (local GPU is shared); air-to-air training data, since the sibling detector transfers poorly to unseen data |
| **Stage 7b: finish cueing** | Clock offset; shared positions late or lost; drifting GNSS; where the 0.8 % leaks happen | CPU |
| **Stage 8: vision-only flocking** | Uses the calibrated relative positions | CPU, then the simulator |

**Open items carried forward:**

| Item | Proper fix |
| :--- | :--- |
| Constant-velocity model with 3 m/s² noise, tuned on the test scenario | Turn-rate or IMM filter, checked on a different scenario |
| Unknown-size one-observer error fell when the model was loosened; cause unknown | Trace one run |
| "Size known" lower than baseline (64 % against 75 %, 32 seeds), not established | Paired comparison over the same seeds |
| Calibrator treats slow GNSS error as white noise | A state per teammate or a fading memory |
| Leaked teammate boxes (0.8 %) not diagnosed | Log where they occur |
| Three observers beat five in stage 7 (95 % against 87 %), unexplained | Test the "fewer leaks" explanation |
| Cued tiles ~40 % of a full scan; full-scan schedule for new drones not designed | A look scheduler: cues every frame, a full scan every N frames; measure detection delay |
| No gaze control | A look-direction rule driven by tracks |
| Assumed false-alarm rate and box noise | Stages 5 and 6 |
| x500 height 0.25 m is a guess | Read it from the model file |
| Cost per frame on a Pi 5 unmeasured | Time `Tracker.step` and `HeadingCalibrator.step` on the board |
| Doc 02 S-entries behind cueing not re-read at the source | Read them |
| Hardware prices, parts and accelerators unchecked | Check at the source before stage 8 |

## 10. CPU Follow-ups (2026-10-05, while WSL waits for a reboot)

How each was reached, with the failed prediction and the scheduler's first version: [build log §11](04_build_log.md#11-cpu-follow-ups-while-wsl-waits-for-a-reboot-2026-10-05). Outputs: `results/`.

### 10.1 Results

| Question | Answer | Status |
| :--- | :--- | :--- |
| Where do teammate boxes leak? | Over half: teammate inside the image, its cue just outside. The rest: a 99 % gate rejecting large GNSS-error pairs | Measured |
| Fix | **Gate 99.9 %** (`HeadingCalibrator(gate=16.27)`): 87 → 94 % with fixed GNSS errors, **73 → 86 % with drifting ones**. Cues up to 3σ outside the image (`edge=3`) add ~3 points with the gate (97 % / 89 %) but **alone hurt under drift** (67 %, start-up heading mis-locks). Recommended: the gate now, `edge` after a start-up guard | Measured, 16 seeds; **defaults not changed yet** |
| Shared positions 0.5 s late, bursty | Used as they are: 44 % / 18 %. **Dead-reckoned with their velocity: 88 %** | Measured |
| Clock offset | sd 0.05 s harmless; 0.2 s → 76 %. **Stamp messages with GNSS time; no need to estimate offsets** | Measured |
| Drifting GNSS error | 73-74 %: stage 7's fixed-error seeds had luckily *small* errors (no pair above 2 m vertically, 2.7 % expected), so **stage 7's 87 % was optimistic**; the tracker side is barely affected (stage 4 scene 94 → 91 %) | Measured |
| Three observers beat five? | +11 ± 12 points paired: **not established**; with leaks fixed, five equal three. Five place drones 0.27 m better | Settled |
| Size known? | Count unchanged (paired −3 ± 6), error −5 cm, **swaps doubled**: per-camera heading biases split one drone's track in two when the size pins the range; gone with calibrated headings | Explained |
| 1280-px camera | Same count, **+0.43 ± 0.05 m** error (paired) | Settled |
| Look scheduler | 15 tiles/s per camera (Pi 5 CPU) with rotating track windows: **99 %**, 0.96 m (whole frame 87 %, 0.77 m). The gain is fewer chances for false sightings to confirm: confirming after 10 hits with the whole frame also gives 97 % | Measured; time to find a drone entering later not tested |
| Motion model | CV 3 m/s² holds on a new sharp-turn drone; CV 6 slightly better; **IMM no clear gain** (−0.10 m only with calibrated headings) | Measured; IMM kept as an option |
| x500 size | 0.77 m span confirmed, height 0.29 m (PX4 v1.17.0 model file) | Read at source |
| Prior art (doc 02 correction) | Heading calibration from a teammate with pseudorange GNSS exists (Opromolla 2020, offline, 2.7°); Omni-swarm projects estimates into the image for association. "The cue decides where the detector looks" still not found | Read at source |
| Camera | Pi Camera Module 3 Wide: f ≈ 1870 px at 102° but rolling shutter; the global-shutter Pi camera is the 1280-px class | Read at source |

### 10.2 Open items (2026-10-05, evening)

| Item | Proper fix |
| :--- | :--- |
| Leak fix not adopted: gate 99.9 % ready; `edge` mis-locks at start-up | Adopt the gate and re-run stage 7; a start-up rule for `edge` (no edge matches until the heading sigma is small) |
| Confirmation rule too loose for five cameras at 10 Hz (5 hits in one frame) | Count hits over time or per camera; re-run stage 4/7 tables |
| Split tracks (one drone, two tracks ~6 m apart) when heading biases disagree | Calibrated headings fix it here; a track-to-track merge on likelihood rather than distance |
| Calibrator treats slow GNSS error as white noise | A state per teammate or a fading memory |
| Look scheduler: drones entering the view later; real per-tile detector cost | A scenario with arrivals; stage 5 |
| Motion model: CV 6 m/s² looked slightly better | Run through the stage 4/7 tables before changing |
| Unknown-size one-observer error fell when the model was loosened; cause unknown | Trace one run |
| Assumed false-alarm rate and box noise | Stages 5 and 6 |
| Cost per frame on a Pi 5 | Laptop: `Tracker.step` 20 ms median (41 ms p99), calibrator 0.8 ms, IMM tracker 50 ms, of a 100 ms frame; the board itself unmeasured |
| Camera: rolling vs global shutter at f ≈ 1870 px | User decision (doc 02 question 3 also open: fixed camera or gimbal) |
| [70], [71] (MDPI) unread (fetch refused) | Read from another copy |
