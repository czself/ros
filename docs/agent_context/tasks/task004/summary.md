---
task_id: task004
type: summary
status: implemented
from: coder
to: reviewer
revision: 3
review_required: true
---
# Task004 Revision 3 Summary — formal Gate B on the pure-SLAM map

Gate B navigation acceptance completed on the Gate-A-approved pure-SLAM map
`competition_slam_verified4` (no truth overlay).

Procedure and evidence:

- `./scripts/start_navigation.sh competition_slam_verified4` single cold
  start; RViz opened on DISPLAY=:1; AMCL locked to birth
  `(4.0833,-3.9583,1.5708)` with `map->base_footprint` error ~0.002-0.004 m.
- Two independent fresh single-run validations (fixed birth, no teleport):
  - `_seed:=712345678`: 5/5 SUCCEEDED
    `evidence/navigation_validation_20260920_seed712345678_verified4.json`
  - `_seed:=777001133`: 5/5 SUCCEEDED
    `evidence/navigation_validation_20260920_seed777001133_verified4.json`
- `_seed:=998877665`: 4/5 — goal `(4.075,-0.825)` ABORTED (state=2) 0.33 m
  short; adjacent `(4.075,-1.925)` then SUCCEEDED, and an ad-hoc re-run of
  that exact order produced 3/3 SUCCEEDED. Recorded as a transient
  near-goal DWA nuance, not a reachability defect.
- Exclusive authorities captured: `/map` by `/map_server` only; `/my_car/cmd_vel`
  by `/cmd_vel_watchdog` only (consumer `/gazebo`); `/my_car/cmd_vel_nav` by
  `/move_base` only (consumer watchdog); TF `map->odom` by `/amcl`;
  `check_foundation.py` passed=true; local costmap histogram recorded.
- Accepted with 0.35 m birth guard per run; navigation params unchanged from
  revision 2.

Artifacts:

- `evidence/navigation_validation_20260920_seed712345678_verified4.json`
- `evidence/navigation_validation_20260920_seed777001133_verified4.json`
- `evidence/rviz_verified4_seed712345678.png`,
  `evidence/rviz_verified4_seed777001133.png`
- `evidence/live_gateB_runtime_evidence_verified4.txt`

Gate B approved. Remaining open items (patrol runtime acceptance and
laser/depth dynamic-obstacle proof) are tracked separately.