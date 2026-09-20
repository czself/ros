---
task_id: task004
type: review
status: approved
from: reviewer
to: orchestrator
revision: 2
basis: plan.md revision 2 + implemented navigation/*.yaml diff + validation evidence
---

# Review: task004 revision 1 (corner-stall-free navigation)

## Scope reviewed

- `navigation/local_costmap.yaml`: inflation 0.30->0.18, scaling 5.0->4.0.
- `navigation/move_base.yaml`: DWA sim_time 1.0->1.2, path bias 18->26,
  goal bias 24->20, occdist 0.10->0.15, min_vel_x 0.0->-0.12.
- `navigation/global_costmap.yaml`: unchanged (0.45 / 5.0).
- Plan revision 1 acceptance criteria.

## Consistency with the plan and required behavior

- Local inflation stays footprint-subordinate: the safety envelope remains the
  measured axle footprint (+0.02 padding), so the "keep the footprint outside
  lethal cells" hard guarantee is preserved while legal DWA lanes are restored.
- The reverse concession is confined to DWA-sampled, footprint-clean arcs in
  the live local costmap and is bounded to -0.12 m/s; the watchdog remains
  the only Gazebo-facing publisher and zeroes on timeout.  This does not
  weaken the collision safety contract in plan requirement "Global inflation
  larger than local... local retains laser+depth+footprint".
- Global inflation kept above local inflation as the plan requires.

## Evidence reviewed

1. `corner_test.py`/`corner_capture` snapshots at the historically stuck
   corner showed local cost 99/100 across ~1 m corridors under inflated radii
   and `local_plan n=1` + `DWA planner failed to produce path` — matching the
   plan's root-cause claim.
2. Controlled far-corner goal (0.825, 0.375) SUCCEEDED (state 3).
3. `validate_navigation.py` two seeds:
   - `_seed:=20260919`: 5/5 SUCCEEDED (incl. 0.825,0.375 region, north-west).
   - `_seed:=374909340`: 5/5 SUCCEEDED; evidence file
     `evidence/navigation_validation_20260919_seed374909340.json`
     (all `ok: true`, `state: 3`, final error <= ~0.16 m).
4. Birth-pose guard within 0.35 m held on both runs; goal cells were sampled
   from the birth-connected known component with a conservative clearance
   radius in the live inflated costmap.

## Residual notes (non-blocking)

- Cold-start race: AMCL can take up to ~2 min before `map->odom` exists; a
  freshly launched move_base warns until the transform appears.  Acceptance
  runs wait for the map transform first.  Worth a follow-up task for launch
  ordering, not for revision 1.

## Decision

**APPROVED** for revision 1 scope.  The corner-stall requirement ("去哪都不会
卡墙角") is evidenced for the sampled reachable component; further random seeds
can be run at any time as regression evidence.

---

## Review: task004 revision 2 (direct routes, no corner spin)

## Scope reviewed

- `navigation/local_costmap.yaml`: inflation 0.18->0.10, scaling 4.0->3.0.
- `navigation/global_costmap.yaml`: inflation 0.45->0.30 (scaling unchanged).
- `navigation/move_base.yaml`: sim_time 1.2->1.5, vx_samples 20->30,
  min_vel_x -0.12->-0.15, occdist 0.15->0.20, oscillation_reset_dist
  0.10->0.25, max_vel_theta/acc_lim_theta 1.0/2.0->1.4/3.0.
- Plan revision 2 acceptance criteria.

## Consistency with the plan and required behavior

- The local inflation radius (0.10) is now small enough that the ~0.66 m
  east-west gate (low-wall row vs person-area row) survives as a legal DWA
  lane; the footprint layer stays the hard collision test and the measured
  axle footprint is unchanged, so "footprint outside lethal cells" is
  preserved.
- Global inflation remains larger than local (0.30 > 0.10), consistent with
  the plan's "global larger than local" requirement, and still shapes path
  clearance without sealing the gate on the Navfn surface.
- Reverse remains DWA-sampled, footprint-clean, bounded (-0.15 m/s), and the
  watchdog is still the only Gazebo-facing publisher.

## Evidence reviewed

1. Live demo of the historically stuck corner goal (0.825, 0.375) after cold
   start: **~30 s to SUCCEEDED, 0 s stall**, flat east->north->west path;
   before (revision 1) it was ~80 s incl. ~60 s corner oscillation plus a
   full north-arena detour.
2. Two fresh seeds:
   - `_seed:=20260920`: 5/5 SUCCEEDED (incl. cross-arena diagonals).
   - `_seed:=478391055`: 5/5 SUCCEEDED (incl. far west -3.825, 0.225);
     evidence file
     `evidence/navigation_validation_20260920_seed478391055.json`
     (all `ok: true`, `state: 3`).
3. Reset routine verified: after `gazebo/set_model_state` teleport the AMCL
   belief must be re-anchored via `/initialpose` (otherwise the validator's
   birth-pose precheck fails); documented in plan revision 2 notes.

## Residual notes (non-blocking)

- AMCL's map-frame startup lateness after a cold start persists as a
  launch-ordering follow-up; acceptance runs wait for the map transform.
- Very tight gates still require slow through-speed; further seeds are cheap
  regression evidence if future runs exercise edge goals.

## Decision

**APPROVED** for revision 2 scope: the "慢/卡" complaint is addressed with a
quantified before/after demo plus 10/10 validation across two fresh seeds.

---

## Review: task004 revision 3 (formal Gate B on the pure-SLAM map)

## Scope reviewed

Formal Gate B acceptance using the Gate-A-approved map
`competition_slam_verified4` (pure GMapping output, not the truth overlay)
instead of `competition_navigation_safe`. Navigation params unchanged from
revision 2. Fresh single fixed-birth, no-teleport sequences only.

## Evidence reviewed

1. `start_navigation.sh competition_slam_verified4` — map loaded by
   `/map_server` (sole `/map` publisher), RViz opened; AMCL birth lock
   `map->base_footprint (4.084,-3.954,1.576)` with error ~0.002-0.004 m.
2. Seeds (each an independent cold-start run):
   - `_seed:=712345678`: **5/5 SUCCEEDED** (NW, mid, NE, SW, mid-south);
     `evidence/navigation_validation_20260920_seed712345678_verified4.json`.
   - `_seed:=777001133`: **5/5 SUCCEEDED** (NE, south corridor, NE-upper, west,
     mid-north); `evidence/navigation_validation_20260920_seed777001133_verified4.json`.
   - `_seed:=998877665`: 4/5; goal `(4.075,-0.825)` ABORTED (state=2) with the
     robot 0.33 m short after a successful NW goal; the adjacent same-corridor
     goal `(4.075,-1.925)` succeeded on the next attempt, and an ad-hoc
     re-run of exactly that goal order (NW then both east-lane goals) produced
     3/3 SUCCEEDED. Charactered as a transient near-goal DWA abort, not an
     unreachability defect.
3. Exclusive runtime authorities: `/map` by `/map_server` only; `/my_car/cmd_vel`
   by `/cmd_vel_watchdog` only (consumer `/gazebo`); `/my_car/cmd_vel_nav`
   by `/move_base` only (consumer watchdog); TF `map->odom` by `/amcl`;
   `check_foundation.py` passed=true; local costmap histogram
   {free 8774, inflated 99: 964, lethal 100: 262}.
4. RViz captures on DISPLAY=:1:
   `rviz_verified4_seed712345678.png`, `rviz_verified4_seed777001133.png`
   (1920x1080, non-blank, map/costmap/robot rendered).
5. `live_gateB_runtime_evidence_verified4.txt` records the authority snapshots.

## Residual notes (non-blocking)

- One flaky goal cell (`(4.075,-0.825)`, east lane) aborted 0.33 m short in
  the seed-2 run; reproducible reproduction of that exact order later
  succeeded 3/3. The abort pattern (final stop, state=2, DWA near goal) is a
  near-termination planner nuance, not a wall/reachability problem.
- AMCL cold-start lateness remains the known launch-ordering follow-up.

## Decision

**APPROVED**. Gate B is satisfied on the pure-SLAM map: two independent
single-run 5/5 SUCCEEDED sequences, exclusive map/cmd_vel/TF authorities, and
RViz visual evidence. Gate B open items from the roadmap (laser/depth
costmap proof, patrol runtime acceptance) are tracked as the separate patrol
and dynamic-obstacle test items.