---
type: workflow_goal
status: active
updated_by: planner
updated_at: 2026-09-19T00:00:00+08:00
---
# Workflow Goals: Complete SLAM Coverage and Navigation

This document defines the gates for the current ROS/Gazebo delivery. A gate
cannot be marked complete from a node list or a screenshot alone; each claim
must have runtime evidence tied to the same session and artifact hashes.

## Goal A: Birth-connected SLAM coverage

Produce a map from the live `/slam_gmapping` publisher while the robot remains
at the fixed configured birth pose and is driven through every physically
reachable, robot-clear area connected to that pose.

### Coverage definition

- `reachable` means a route exists from the birth pose for the configured robot
  footprint with the measured walls and collision geometry.
- `observed` means the active laser has returned valid measurements for the
  area; unknown cells are not free cells.
- `blocked/unreachable` means a frontier is separated by a closed wall, a
  collision-clearance violation, or another physically verified constraint.
- Full coverage means there is no remaining reachable frontier after inflation
  and the coverage report lists every remaining unknown component as either
  blocked/unreachable with evidence or a failed coverage item.

### Gate A acceptance evidence

- Clean mapping runtime has exactly one `/map` publisher (`/slam_gmapping`),
  one odometry/TF authority, and one motion source.
- The map manifest records session id, start/end timestamps, measured chassis
  birth pose, derived axle/base pose, topic rates, TF caller IDs sampled from
  `/tf` and `/tf_static`, route/trajectory hash, mapper hash, completion log
  hash, and map image hash.
- The saved PGM is pixel- and normalized-content different from all truth maps.
- A coverage report contains map dimensions, known/free/occupied/unknown
  counts, inflated reachable component, frontier count before/after, and an
  explicit list of blocked/unreachable regions. `known_fraction` alone is not
  sufficient evidence of full coverage.
- RViz shows the live map and scan together at the final pose; no claim is made
  from a truth-map overlay.

### Gate A failure conditions

Fail and retain the artifact as partial if any publisher provenance is missing,
the trajectory/log hash does not match the manifest, the mapper stops on stale
data, or a reachable frontier remains. Do not teleport the robot or lower/remove
walls to make a frontier disappear.

## Goal B: Navigation on the verified map

Using only a Gate-A-approved map, start navigation from the same fixed birth
pose and reach five separated goals without teleporting or changing the scene.

### Gate B acceptance evidence

- Mapping, teleop, frontier mapper, and stale navigation processes are stopped.
- `map_server` is the sole `/map` publisher; AMCL is the sole `map -> odom`
  authority; `move_base` is the sole command owner.
- Global and local costmaps are live. Laser and depth obstacle observations are
  visible in the runtime costmap, and unknown cells are not treated as free.
- Five goals are sampled from the inflated birth-connected free component and
  recorded before execution. The robot starts once at the fixed birth pose and
  moves through the sequence with no teleport/reset between goals.
- All five action goals reach `SUCCEEDED` within wall-clock limits, with final
  position/yaw tolerances, no collision/contact event, and recorded goal,
  status, elapsed time, final pose, and runtime costmap evidence.
- RViz evidence includes map, robot pose/TF, scan, depth point cloud, global and
  local plans, costmaps, and footprint.

### Gate B failure conditions

Any goal timeout/abort, duplicate map or TF authority, missing obstacle-layer
observations, collision, stale pose, or birth-pose change fails the gate. A
successful action result without pose/error and runtime costmap evidence is not
an acceptance.

## Dependency and handoff

Goal B is blocked until Goal A is approved by an independent review. The
current `competition_slam_verified` map remains an interim measured artifact:
it has unknown cells and is not a Gate-A-approved full-coverage map.
