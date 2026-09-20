---
task_id: task002
type: architecture_review
status: approved
from: reviewer
to: orchestrator
revision: 0
decision: APPROVED
next_action: implement_current_task
---

# Task002 architecture review

## Decision

APPROVED for foundation implementation. This does not approve runtime behavior,
SLAM coverage, or navigation delivery. No blocking architecture issues found.

## Findings and implementation checks

1. The axle-frame design addresses a measured kinematic error. For chassis yaw
   theta and offset a=0.125 m, axle world position must be
   `(x+a*cos(theta), y+a*sin(theta))`. Its world velocity is chassis velocity
   plus `omega cross R(theta)*(a,0,0)`, then rotated into axle body axes. In the
   planar case this adds `a*wz` to chassis body lateral velocity. Applying only
   the frame rotation would leave the audited error. Verify rotation about a
   stationary axle and straight travel in at least two headings.
2. The user birth pose explicitly remains the chassis pose. At yaw=1.5708 the
   corresponding axle is approximately `(4.0833,-3.9583)`. AMCL/GMapping initial
   poses and goal verification must distinguish these two frames. Define the
   base ground-plane Z convention explicitly and preserve the physical
   chassis/sensor height in `base_footprint -> chassis`; do not silently flatten
   every link to Z=0 or use an incorrect static chassis height after settling.
3. Lowering the physical lidar to chassis Z=0.30 m leaves approximately 0.117 m
   clearance below the 0.50 m wall tops at the nominal chassis birth height.
   This is geometrically feasible and avoids the existing grazing scan plane.
   Synchronize the sensor link, TF, and mast/head visuals in both robot copies.
   Validate settled world height and actual scan hits; changing TF alone is
   insufficient. Camera optical axes must remain a separate frame from the
   physical camera mounting frame.
4. The proposed map authorities correctly separate modes: only GMapping during
   mapping; map_server plus AMCL during localization. Killing ROS nodes alone
   is insufficient if old launch processes respawn them. Verify publishers by
   sampled TF connection authority and fresh sensor/map messages after a cold
   start and a mode switch, not by node-name existence.
5. The footprint must be offset with the axle: body collision X spans roughly
   `[-0.325,0.075]` and wheels extend to approximately Y=+/-0.176667 in this
   frame. Include margins and other collision geometry. Restoring both depth
   and laser obstacles is appropriate, but prove each independently contributes
   current obstacle cells and that ground/self returns do not obstruct the car.
6. World-derived odometry remains an explicit simulation assumption; it does
   not make the SLAM occupancy image synthetic. Later saved-map provenance
   must include the map-frame birth pose or map-to-odom relation required for
   AMCL initialization after a reset. A saved GMapping map is not guaranteed
   to retain an exactly identity map/world transform.
7. The task boundary is sensible: foundation validation must complete before
   new exploration, and a truth-map diagnostic must remain labeled as such.
   Tasks003/004 still owe reachable-region coverage, five dispersed goals,
   stopping/collision evidence, and live visual verification. No-frontier or
   timeout alone cannot establish full physical coverage.

## Reviewer verification

- Read the complete codgent-workflow skill and reviewer template.
- Read project_context.md, architecture_repair.md, task002/plan.md, and the
  complete navigation_audit_20260919.md.
- Inspected model_state_odom.py, navigation.launch, common_costmap.yaml,
  start_sim.sh, and robot/nearby-wall geometry in competition_classic.world.
- Confirmed the plan directly covers audited TF conflicts, axle offset,
  twist coordinate semantics, lidar height, AMCL isolation, and disabled
  obstacle layers. No implementation summary or test results exist for this
  new task yet; implementation/runtime verification is deliberately pending.
- Review is read-only except for this review document. No runtime mutation or
  implementation edits performed.

## Next action

Implement task002 and supply exact command results and measured motion/TF/
sensor evidence for a separate implementation review.
