---
task_id: task004
type: plan
status: ready_for_implementation
from: orchestrator
to: coder
revision: 2
requires_review: true
---
# Task004: collision-safe navigation runtime

## Goal

Prevent the navigation route from entering current Gazebo collision geometry,
and ensure a crashed/stopped navigation process cannot leave the diff-drive
plugin moving on its last command.

## Required behavior

- The navigation map must union the measured/hybrid map with the current
  world's static collision boxes.  A SLAM free cell cannot erase a physical
  wall.  The artifact must be labelled navigation-only, not pure SLAM.
- Global inflation is larger than local inflation so Navfn approaches corners
  with clearance before DWA takes over.  The local costmap must retain laser
  and depth obstacle layers and the measured axle footprint.
- `move_base` publishes only a private navigation velocity topic.  A wall
  clock dead-man watchdog is the sole navigation-mode publisher connected to
  Gazebo and outputs zero when navigation commands stop arriving.
- Runtime transitions purge dead ROS registrations, cancel action goals, and
  send a zero command before/after stopping nodes.
- Random goals require a conservative clearance from the live inflated global
  costmap and are sampled only from the birth-connected known component.

## Verification

- Static Python, YAML, shell, and XML checks pass.
- Safe map has no current-world occupied cell represented as free.
- Cold navigation start has one `/my_car/cmd_vel` publisher (watchdog), one
  `/my_car/cmd_vel_nav` publisher (move_base), one map publisher, and one TF
  authority per transform.
- Watchdog unit check: a fresh command is forwarded; after wall-clock timeout
  the output is zero.
- Controlled goal(s) keep the footprint outside lethal local-costmap cells and
  finish without a Gazebo wall penetration; five random goals are attempted
  with persistent per-goal result evidence.

## Out of scope

Pure SLAM coverage/provenance remains task003 and is not silently reclassified
as complete by this navigation safety overlay.

---

## Revision 1 (2026-09-19): guarantee no corner stalls

### Problem found during verification

With `local inflation 0.30 / global inflation 0.45` and reverse disabled
(`min_vel_x 0.0`), the robot reached several pinch points and stalled with
`DWA planner failed to produce path`, `local_plan n=1`, spinning in place
(state ACTIVE).  Diagnosis evidence:

- Physical corridors around the low-wall boxes are ~1.0-1.2 m wide.  The
  0.38 m footprint + 2 x inflation must fit the lane; at local 0.30 the lane
  cost reached 99/100 along the whole corridor and every forward sample was
  rejected (`corner_capture`, gradient dumps).
- In a corner pocket the footprint rotation sweep can exceed the free space to
  yaw in place; with reverse disabled and rotation blocked the robot is
  irrecoverable.

### Agreed changes (implemented)

- `local_costmap.yaml`: `inflation_radius 0.30 -> 0.18`,
  `cost_scaling_factor 5.0 -> 4.0` so ~1 m corridors keep a legal DWA lane;
  wall avoidance is carried by DWA weights, not by inflation that seals lanes.
- `move_base.yaml` (DWAPlannerROS): `path_distance_bias 18 -> 26`,
  `goal_distance_bias 24 -> 20` (follow the path, swing wide at corners),
  `sim_time 1.0 -> 1.2`, `occdist_scale 0.10 -> 0.15`,
  `min_vel_x 0.0 -> -0.12` (slow footprint-checked reverse escapes corner
  pockets; DWA only samples reverse arcs that are clear in the live local
  costmap, and the watchdog still zeroes the bus on any timeout).
- `global_costmap.yaml`: kept at 0.45 / 5.0 (path-only layer; Navfn is only
  blocked by lethal cells, large global inflation shapes clearance without
  sealing corridors, and DWA decides passability from the local map).

### Acceptance (revision 1)

With the stack cold-started from `start_sim.sh` + `start_navigation.sh` and
AMCL latched at the birth pose:

1. Controlled far-corner goal (0.825, 0.375) — the historically stuck corner —
   completes SUCCEEDED (state 3).
2. Two fresh seeds of `validate_navigation.py` (5 random, clearance-checked,
   birth-connected goals each): **10/10 SUCCEEDED**, including far west
   (-3.925, 0.175) and north (1.275, 3.825) targets.  Evidence:
   `docs/agent_context/tasks/task004/evidence/navigation_validation_20260919_seed374909340.json`
   and the printed run for `_seed:=20260919`.
3. No wall penetration / no lethal-cell footprint at the goal stamps (goal
   cells carry a conservatively clear radius in the live inflated map).

### Out of scope (unchanged)

- The AMCL-vs-move_base startup race (map frame occasionally appears late
  after a cold start) does not cause stalls during aligned runs; a pre-run
  map-transform readiness wait is acceptable for acceptance runs.

---

## Revision 2 (2026-09-20): direct routes, no corner spin

### Problem found during live demo

On a live Gazebo+RViz demonstration of the same revision-1 setup the robot
still spent ~60 s oscillating at the low-wall pocket near (3.80, 3.58)
before finally reversing out, then took the long north detour (~80 s total).
To the user this reads as "too slow / stuck".

Diagnosis:

- That pocket is geometrically unspinnable: a wall band is ~0.3 m on the
  west side of the robot, so an in-place yaw sweep collides; the only exit
  is a reverse escape.  Lowering the rotation speed does not help.
- The east-west gate between the low-wall row (bottom y 3.246) and the
  person-area row (top y 2.589) is only ~0.66 m physical.  Footprint is
  0.42 m wide, so **any inflation radius above ~0.12 m shears that gate
  shut at the local layer**.  At global inflation 0.45 Navfn also refuses
  it, so the planner detoured around the whole north side of the arena.

### Agreed changes (implemented)

- `local_costmap.yaml`: `inflation_radius 0.18 -> 0.10`,
  `cost_scaling_factor 4.0 -> 3.0`.  The physical gates are the credible
  gap test; the footprint layer remains the hard collision check, and DWA
  keeps the body off the walls.
- `global_costmap.yaml`: `inflation_radius 0.45 -> 0.30`.  Keeps Navfn's
  route out of the sealed-gate detour while still providing path clearance.
- `move_base.yaml` (DWAPlannerROS): `sim_time 1.2 -> 1.5`,
  `vx_samples 20 -> 30` (finer reverse sampling),
  `min_vel_x -0.12 -> -0.15`, `occdist_scale 0.15 -> 0.20`,
  `oscillation_reset_dist 0.10 -> 0.25` (break the flip-flop sooner),
  `max_vel_theta / acc_lim_theta 1.0/2.0 -> 1.4/3.0` (turns resolve faster).

### Acceptance (revision 2)

Cold start via `start_sim.sh` + `start_navigation.sh`, AMCL re-locked at the
birth pose (after a `set_model_state` teleport AMCL's belief does not follow
the teleport, so a `/initialpose` publish is part of the reset routine):

- Controlled corner goal (0.825, 0.375): the historical stuck corner now
  completes in **~30 s with 0 s stall** and a flat east->north->west path,
  versus ~80 s with ~60 s of corner oscillation before.
- Two fresh seeds (`_seed:=20260920`, `_seed:=478391055`): **10/10
  SUCCEEDED (state 3)** including far west (-3.825, 0.225).  Evidence:
  `docs/agent_context/tasks/task004/evidence/navigation_validation_20260920_seed478391055.json`
  and the printed run for `_seed:=20260920`.
- No wall penetration / lethal-cell footprint at any goal stamp.

### Notes for acceptance runs

After a `gazebo/set_model_state` teleport, publish `/initialpose` at the
birth pose and wait for `map -> base_footprint` to lock within ~0.15 m
before starting `validate_navigation.py` (its birth-pose precheck rejects a
stale AMCL belief otherwise).

### Acceptance (revision 3) — formal Gate B on the pure-SLAM map

After task003 Gate-A approval, Gate B navigation acceptance runs on the
approved pure-SLAM map `competition_slam_verified4` (no truth overlay) with
the revision-2 costmap/DWA parameters unchanged:

- `start_navigation.sh competition_slam_verified4` (single cold start),
  AMCL locked to birth `(4.0833,-3.9583,1.5708)` within ~0.004 m.
- Two independent fresh seeds, single fixed-birth no-teleport sequences:
  `_seed:=712345678` **5/5 SUCCEEDED**, `_seed:=777001133` **5/5
  SUCCEEDED**. Evidence JSONs and RViz screenshots in the task004 evidence dir.
- Exclusive authorities: `/map` by `/map_server`, `/my_car/cmd_vel_nav` by
  `/move_base`, `/my_car/cmd_vel` by `/cmd_vel_watchdog` (consumer `/gazebo`),
  TF `map->odom` by `/amcl`; `check_foundation.py` passed.
- Known residual: one run (`_seed:=998877665`) aborted a single east-lane
  goal 0.33 m short (state=2 near-goal DWA nuance); the identical goal order
  re-run later produced 3/3 SUCCEEDED. Documented, non-blocking.
- Patrol runtime acceptance and the laser/depth dynamic-obstacle proof are
  separate test items (patrol + dynamic obstacle).
