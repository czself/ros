---
task_id: task002
type: plan
status: ready_for_implementation
from: planner
to: coder
revision: 0
requires_review: true
---
# Task002: navigation foundation
## Goal
Fix audit B/C/D foundation issues before another map.
## Scope
worlds/competition_classic.world, models/my_car/model.sdf, scripts/model_state_odom.py, shared runtime/start helpers, navigation/*.launch and costmap/DWA parameters, sensor/robot diagnostics.
## Required behavior
Read architecture_repair.md. Preserve chassis birth, stop conflicting nodes on mode changes, one TF authority, actual axle-frame velocities/extrinsics, working AMCL and laser/depth costmaps. Footprint contains collision geometry. Cold startup cannot depend on stale parameters.
## Verification
Python/shell/XML checks; clean restart; rates/TF authorities; physical sensor vs TF; controlled forward/angular commands with measured axle movement and stopping; nonempty obstacle costmap; live RGB/depth/points. A current-world truth map may be used only as explicitly labeled diagnostic data.
## Boundaries
Mapping and five-goal acceptance are later tasks. Do not alter walls, reset per goal or delete old maps.
## Handoff
Coder writes summary and exact command results under artifacts. Reviewer independently checks architecture, then implementation and evidence. Root maintains state.
