# Drone-to-Drone: Detection, Connection & Pathing: Scope & Research Plan

Drones that **see and locate other drones from the air, share what they know, and plan paths**. It's a sibling of [`.Drone Detection`](https://github.com/JoshChomps/Drone-Detection) (ground camera → drone) and `Wildfire_Survey`.

> [!NOTE]
> **Status 2026-10-05: build stages 2-4b and 7 done ([doc 03](03_build_plan.md)): camera geometry, estimators, a five-on-five tracker, and teammate cueing with heading calibration (heading error 1.8° → 0.34°; with teammates in view, the right count 0 % without cueing and 87 % with it); 42 tests. A realistic-detector stress test and more seeds are in doc 03 §8. CPU follow-ups (doc 03 §10, build log §11): fixing two teammate-leak causes lifts the right count to 97 %; shared positions must be dead-reckoned (then a late, lossy link costs nothing) and GNSS time stamps suffice for clocks; a look scheduler at 15 tiles/s per camera (a Pi 5 CPU) keeps 99 %; an IMM motion model gives no clear gain. Nothing is installed yet; WSL2 waits on one admin step by the user.** Every step, with its mistakes and measurements, is in the [build log](04_build_log.md). **Next: see [doc 03 §9](03_build_plan.md#9-open-items-and-what-comes-next-2026-10-05).** Repository: [JoshChomps/Drone-to-Drone](https://github.com/JoshChomps/Drone-to-Drone).
>
> **Purpose (user, 2026-10-03):** a purely academic, in-house exercise. **Focus: drone-to-drone detection, especially a swarm seeing another group of drones** (detect, tell teammates from others, keep identities, estimate position and velocity, merge five cameras into one picture). Outdoors, above tree height. The tag game is set aside. Scope table: [doc 03 §1](03_build_plan.md#1-scope-agreed-with-the-user-2026-10-03).

| Doc | Contents |
| :--- | :--- |
| This README | Scope, constraints, phase-1 research findings (§7) |
| [01 Phase 2 simulation plan](01_phase2_simulation_plan.md) | PX4 v1.17 + Gazebo Harmonic in WSL2; MAVSDK agents; simulated radio link; formation, CBF collision avoidance, coverage; experiments S0-S9; setup steps |
| [02 Research survey and novel directions](02_research_survey_and_novel_directions.md) | The field by sub-problem (176 catalogued resources, each marked with how far it was checked); what has not been tested; the "radio-cued vision" package; five numerical trials; what ports from `.Drone Detection`; added experiments S2b-S2d and T1-T4 |
| [03 Build plan](03_build_plan.md) | Agreed scope (in and out), build stages 0-9 with their checks, code layout, hardware leaning, **results: §5 locating one drone, §6 tracking five** |
| [04 Build log](04_build_log.md) | **The history, written to be learned from:** each step's intent, what was done, measurements, what worked, what failed and why, and the lesson |
| [`d2d/`](d2d/geometry.py), [`tests/`](tests/test_geometry.py) | The package (stage 2: frames and camera geometry; stage 3: `estimate.py`, locating a drone from one or more cameras; stage 4: `track.py`, tracking several drones from a team's cameras; stage 7: `cue.py`, recognising teammates and calibrating heading; `estimate.IMMFilter`, a two-model motion filter) and its tests: `python -m unittest discover -s tests` |
| [`tools/phase2_checks.py`](tools/phase2_checks.py) | Reproduces doc 01's numbers (numpy only) |
| [`tools/research_checks.py`](tools/research_checks.py) | Reproduces doc 02's trial numbers (numpy only, ~90 s) |
| [`tools/stage3_checks.py`](tools/stage3_checks.py) | Reproduces doc 03 §5 (3-D, two cameras, ~4 min) |
| [`tools/stage4_checks.py`](tools/stage4_checks.py) | Reproduces doc 03 §6 (five observers, five drones, ~20 min) |
| [`tools/stage4_detector_checks.py`](tools/stage4_detector_checks.py) | Doc 03 §8: the tracker under a realistic detector, and more seeds (parallel, ~10 min) |
| [`tools/stage7_checks.py`](tools/stage7_checks.py) | Doc 03 §8: teammate cueing and heading calibration (`main` ~18 min, `floor` ~6 min) |
| [`tools/paired_checks.py`](tools/paired_checks.py) | Build log §11.2: rows compared on the same seeds (~20 min) |
| [`tools/stage7b_checks.py`](tools/stage7b_checks.py) | Build log §11.3-11.5: where teammate boxes leak, shared positions late/lost/mistimed/drifting, the leak fixes (`leaks`, `share`, `fix`, `drift`, `fixunderdrift`) |
| [`tools/swap_checks.py`](tools/swap_checks.py) | Build log §11.9: why a known drone size doubled identity swaps (`main`, `calibrated`) |
| [`tools/motion_checks.py`](tools/motion_checks.py) | Build log §11.6: constant-velocity vs IMM on two different drones (~3 min) |
| [`tools/looks_checks.py`](tools/looks_checks.py) | Build log §11.10: the look scheduler's tile budget, and the stricter-confirmation test (`main`, `confirm`) |
| [`tools/cost_checks.py`](tools/cost_checks.py) | Build log §11.11: time per frame of the tracker and calibrator on the laptop (the Pi 5 is not measured) |
| [`results/`](results/) | The saved output of every script above, from the 2026-10-05 runs |

---

## 1. Two Modes, Built in This Order

| Mode | What it does | Status of scope |
| :--- | :--- | :--- |
| **1. Cooperative swarm** (first) | Our own drones detect and locate each other (camera + radio), share positions, and plan paths together: formation keeping, collision avoidance, and dividing an area for coverage (e.g. the wildfire survey) | Practical, flyable with our own drones |
| **2. Theoretical defensive tracking** (later). Doc 02 studied it as **tag** between our own drones ([§5.5](02_research_survey_and_novel_directions.md#55-tag)); **the tag game is set aside (user, 2026-10-03)** and the current build observes and identifies only ([doc 03 §1](03_build_plan.md#1-scope-agreed-with-the-user-2026-10-03)) | A drone detects and tracks a *non-cooperative* drone and plans a path to **follow and keep it in view**, for safety only | **Theoretical and non-aggressive.** It targets the technology, never people; no contact, no weapons, no jamming; not military. **Simulation, plus our own cooperative drones as stand-in targets, only** |

Mode 2 builds on mode 1's components: detection from a moving camera, relative localisation, and path planning.

## 2. The Core Technical Problems

| Problem | Why it's hard | Starting points |
| :--- | :--- | :--- |
| **Detecting a drone from a moving drone** | The camera itself moves (6 degrees of freedom), so background subtraction breaks. The background is ground clutter as well as sky ([`.Drone Detection` doc 08](https://github.com/JoshChomps/Drone-Detection/blob/main/08_camera_angles_and_operational_pov_selection.md), Case 3) | Det-Fly dataset (air-to-air, verified in `.Drone Detection` doc 03). Reuse the detector interface, tiling and size-binned evaluation. Compensate motion with the drone's IMU and odometry |
| **Relative localisation** | One camera gives bearing, not range ([`.Drone Detection` doc 09](https://github.com/JoshChomps/Drone-Detection/blob/main/09_monocular_depth_frame_tracking_and_kinematics.md)) | Fuse camera bearing with shared GPS/IMU states (cooperative), ultra-wideband ranging, or two drones triangulating the same target |
| **Connection (communication)** | Links drop, add latency and have limited bandwidth, especially in mountains | MAVLink telemetry, mesh radios. Design for stale or missing messages from the start |
| **Path planning** | Coordinate several vehicles safely, in real time, with uncertain positions | Coverage planning, collision avoidance with safety margins; follow/keep-in-view pursuit (mode 2) studied in simulation |
| **Tracking + filtering** | Targets manoeuvre; measurements are noisy and intermittent | The Kalman/IMM work in `.Drone Detection` doc 09, done in world coordinates because the observer moves |

## 3. Simulation First

Both modes start in simulation: repeatable, free, and no risk to aircraft. Candidates (verified in [`.Drone Detection` doc 02](https://github.com/JoshChomps/Drone-Detection/blob/main/02_github_repositories_and_codebases.md)): **PX4-Autopilot** with software-in-the-loop Gazebo (multi-vehicle), and **IsaacLab** for GPU-accelerated learning. Simulated camera views of other drones provide perfectly labelled training data.

**Decided for phase 2 ([doc 01](01_phase2_simulation_plan.md)):** PX4 v1.17.0 + Gazebo Harmonic in WSL2 Ubuntu 24.04, headless, one MAVSDK agent per drone and a simulated, seeded radio link. **IsaacLab is ruled out on this laptop** (it needs 16 GB VRAM and 32 GB RAM; doc 01 §2.5).

## 4. Constraints

* **Mode 2 stays theoretical:** simulation plus our own cooperative drones only. Never pursue or interfere with other people's aircraft. It observes and tracks, nothing more (per the user's scope).
* **Flying multiple drones in Canada:** a future consideration (§7.1). Development and endorsement have free rein; confirm the authorisation for multi-drone and beyond-visual-line-of-sight flights before public or field deployment. The Transport Canada categories are summarised in [`.Drone Detection` doc 13 §5](https://github.com/JoshChomps/Drone-Detection/blob/main/13_system_requirements_and_geometry_budget.md#5-legal--operational-scope).
* **No radio jamming or spoofing**, ever: illegal in Canada, and outside the project's purpose.
* **Low cost:** small drones, a Raspberry Pi 5-class computer on board, a laptop RTX 4060 + Colab for training.

## 5. What Can Be Reused From `.Drone Detection`

* Detector interface, tiling, zoom-out (simulated distance) and size-binned evaluation (`cuas/`).
* Kinematics and IMM filtering maths (doc 09); time-to-collision as a cue.
* **Extract a shared package only when this project actually needs the code.**

## 6. Open Decisions

| Decision | Notes |
| :--- | :--- |
| Airframes | Two or three small quads for mode 1 (cheap, legal category) |
| On-board computer | Pi 5 vs a companion board with an accelerator; depends on `.Drone Detection` E3 (Pi 5 speed). Stage 3 found a 4K frame is ~24 tiles (1.6 s per full scan on a Pi 5), so cued tiles or an accelerator are needed ([doc 03 §5](03_build_plan.md#5-stage-3-results-2026-10-04)) |
| Camera | **Not chosen.** Target focal length ~1700-1900 px (doc 03 §5). Fixed or gimbal: open |
| Communication hardware | Telemetry radios vs mesh vs Wi-Fi; range in BC terrain |
| Simulator | **Decided (2026-09-29):** PX4 + Gazebo for phase 2; IsaacLab ruled out locally (VRAM/RAM), see [doc 01 §2](01_phase2_simulation_plan.md#2-simulator-stack-verified) |
| Mode 2 guidance approach | **Set aside with the tag game.** First toy result ([doc 02 §5.5](02_research_survey_and_novel_directions.md#55-tag)): lead pursuit tags in 90-100 % of games, pure pursuit in 15-55 %; area-minimisation needs equal speeds. Play in altitude layers. To be repeated in Gazebo (T1-T4) |
| Indoors or outdoors | **Decided (user, 2026-10-03): outdoors, above tree height** |
| UWB / RTK budget | Open |

## 7. Phase-1 Research Findings (Checked at the Source, 2026-09-28)

### 7.1 Regulation: how many drones one pilot may fly (Canada)
[Canadian Aviation Regulations 901.40](https://laws-lois.justice.gc.ca/eng/regulations/SOR-96-433/page-134.html): no pilot may operate more than one RPA at a time unless the operation is **visual line of sight**, it follows the systems' operating manuals, the system is **designed to let one control station fly several aircraft**, and **no more than five** fly at a time. More than five, or beyond visual line of sight, needs a special flight operations certificate (903.03).

**Status (user, 2026-09-28): a future consideration, not a current constraint.** For development and endorsement the project has free rein. Don't let 901.40 shape the phase-2 simulation design. Revisit it before public or field deployment.

**Consequences once it applies:** a 2-3 drone swarm is within the rules only if each is in sight and either (a) each drone has its own pilot, or (b) the ground control software is genuinely built for multi-aircraft control.

### 7.2 Simulator
* **PX4 multi-vehicle Gazebo simulation** ([PX4 docs](https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation)): one PX4 SITL process per vehicle (`-i <n>`), the first also starts the Gazebo server, the rest join with `PX4_GZ_STANDALONE=1`. **Linux only.**
* **On this Windows laptop that means WSL2**, which the PX4 team supports ([PX4 docs](https://docs.px4.io/main/en/dev_setup/dev_env_windows_wsl); Ubuntu 22.04 or 24.04). Its stated limits: no direct USB access to flight controllers, and the WSL IP changes on restart, so QGroundControl on Windows must be re-pointed.
* **IsaacLab** (in `.Drone Detection` doc 02): **checked 2026-09-28, too big for this laptop.** It needs 32 GB RAM and 16 GB VRAM; Isaac Sim's minimum is an RTX 4080 (16 GB). See [doc 01 §2.5](01_phase2_simulation_plan.md#25-isaaclab-checked-not-for-this-laptop).
* **Recommendation:** start with PX4 + Gazebo in WSL2 for flight logic, communication and path planning. Consider IsaacLab only for learning-based work, after checking its requirements.

### 7.3 Air-to-air data
| Dataset | What it is | Size | Licence | Fit |
| :--- | :--- | :--- | :--- | :--- |
| **Det-Fly** (verified in `.Drone Detection` [doc 01 §2.4](https://github.com/JoshChomps/Drone-Detection/blob/main/01_research_papers_compendium.md#24-air-to-air-visual-detection-of-micro-uavs-an-experimental-evaluation-of-deep-learning-det-fly)) | Micro-UAVs seen from another UAV | 13,000+ images | See the repo | Closest match to mode 1 |
| **Airborne Object Tracking (AOT)** (Amazon; [AWS Open Data](https://registry.opendata.aws/airborne-object-tracking/)) | Two instrumented aircraft flying planned encounters | **4,943 sequences of ~120 s at 10 Hz; 5.9M+ images, 3.3M+ boxes**; public S3 bucket, no account needed | CDLA-Permissive 1.0 | Large and sequential (tracking, closing speeds). Object types and sizes not yet checked; far too big to download whole |

### 7.4 What carries over from `.Drone Detection`, and what does not
| Carries over | Does not |
| :--- | :--- |
| Detector interface, 640-px tiling without downscaling, recall by size and `w_min`, zoom-out simulated distance, per-video bootstrap intervals (`cuas/`) | **Motion cueing.** `cuas.motion` assumes a fixed camera with small jitter (phase correlation plus a background model). A flying camera needs ego-motion compensation from the IMU/odometry first, or detection without motion cues |
| Pixels-on-target geometry ([doc 13 §2](https://github.com/JoshChomps/Drone-Detection/blob/main/13_system_requirements_and_geometry_budget.md#2-pixels-on-target-the-constraint-that-dominates-everything)): a drone seen by a drone is the same few-pixel problem | Sky-only background: from the air, the background is often ground clutter, the hard case in doc 08 |
| Kalman / IMM tracking, time to collision ([doc 09](https://github.com/JoshChomps/Drone-Detection/blob/main/09_monocular_depth_frame_tracking_and_kinematics.md)); here in world coordinates | Ground-station compute: on board, the budget is a Pi 5-class board ([doc 18](https://github.com/JoshChomps/Drone-Detection/blob/main/18_hardware_optimisation_and_realtime_latency.md): the Pi 5 CPU runs ~15 detector tiles/s) |
| Lessons: split by sequence, look at false alarms, "no label" is not "no object", one change per experiment ([doc 17 §7](https://github.com/JoshChomps/Drone-Detection/blob/main/17_design_process_and_lessons_learned.md#7-catalogue-of-what-went-wrong)) | |

**Code sharing:** nothing copied. When phase 3 starts, import `cuas` from `../.Drone Detection` rather than copying it.

---

## 8. Phases (Proposed)

> The working plan is now the stage table in [doc 03 §2](03_build_plan.md#2-stages). The list below is the original outline.

1. **Research:** simulator choice; air-to-air datasets (verify at the source); multi-drone authorisation.
2. **Simulation:** 2-3 drones in PX4 + Gazebo sharing state; formation and collision avoidance. **Plan: [doc 01](01_phase2_simulation_plan.md)** (experiments S0-S9; setup steps in §9).
3. **Air-to-air detection:** train on Det-Fly + simulated views; evaluate with the size-binned method.
4. **Real flights:** 2 drones, cooperative detection + relative localisation.
5. **Mode 2 (simulation, then our own drones as players):** detect → track → follow/keep-in-view, played as contact-free tag. Research and first trials: [doc 02](02_research_survey_and_novel_directions.md).
