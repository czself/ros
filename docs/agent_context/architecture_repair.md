---
type: architecture_analysis
status: proposed
updated_by: planner
review_required: true
---
# Navigation repair architecture

Supersedes architecture_analysis.md. Audit: ../navigation_audit_20260919.md. Existing task001 failed.

## Target and decisions
- Keep Gazebo chassis birth pose (4.0833,-4.0833,0.0833333,yaw=1.5708), scene walls and dimensions unchanged.
- Use base_footprint at the drive axle ground projection (chassis x +0.125 m). DWA/AMCL/GMapping use this frame. User birth pose remains chassis pose, not axle pose.
- One model_state_odom publishes odom -> base_footprint and base_footprint -> chassis; twist expressed at the axle in body axes, reset-safe simulation stamps, bounded publish rate.
- One sensor launch publishes chassis -> lidar_link and chassis -> camera_link -> camera_optical_frame. Move physical lidar to chassis z=0.30m, below 0.50m wall tops; synchronize TF. Depth messages use camera_optical_frame.
- Mapping: GMapping sole map -> odom and /map source. Navigation: map_server sole /map source, AMCL sole map -> odom source. No identity map transform or AMCL relay.
- Shared runtime launcher stops project mode nodes/launches using exact process/ROS identities and waits for health. Explicit world restart with fixed birth. One motion source; mapping/teleop/navigation/patrol/validation mutually exclusive.
- Local costmap uses laser and depth obstacles plus inflation; global uses static map and inflation. Depth range/height filters avoid ground/self returns. Footprint contains wheels and uses axle origin.
- Frontiers use inflated traversable global costmap and move_base with wall-clock watchdogs; complete requires stable coverage/no reachable remaining frontier, not elapsed time.
- Genuine SLAM save includes publisher provenance, metadata, pixel hash, scan/pose/TF recording. Preserve historical wrongly named maps but mark invalid for SLAM. Never copy truth as scanned data.
- Validate five dispersed reachable goals across mapped birth-connected robot-clear region. Check chassis birth once; verify action success, map distance/yaw, world consistency, wall-clock timeout and exclusive controller. Persist JSON/trajectories. No teleport in goal sequence.
- RViz shows map, scan, plans, footprint, RGB, depth and RGB point cloud. Capture runtime evidence.

## Sequence and gates
Task002: physical sensor/base geometry, odom/TF, lifecycle, AMCL/DWA/costmap and controlled movement checks.
Task003 revision 1: genuine SLAM provenance plus a birth-connected coverage report. The
task is not complete while a reachable frontier remains or runtime evidence is
missing. Closed rooms are reported as blocked/unreachable; they are not entered
by teleport and are not made reachable by changing walls.
Task004: only after Task003 review approval, start map_server + AMCL + move_base,
sample five dispersed goals from the inflated birth-connected component, and
record 5/5 runtime navigation evidence and RViz evidence.
One task plan at a time. Independent coding/review handoffs. No passing claim without measurements.

The detailed acceptance gates are in `goal_slam_navigation.md`.

## Risks
Axle origin requires offset-aware footprint/initial pose. Restart invalidates old clocks. Verify depth ground/self filtering. Closed rooms cannot be reached from birth; report them as blocked/unreachable and do not change walls to fake SLAM or navigation success.
