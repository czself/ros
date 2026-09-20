---
type: architecture_analysis
status: draft
updated_by: planner
review_required: true
---

# Architecture Analysis

Superseded by architecture_repair.md after the 2026-09-19 full audit. The claims below describe earlier intended behavior, not validated runtime facts.

## Current Shape

The world embeds the robot, RGB/depth camera, lidar, and differential-drive
plugin. Navigation consumes a static map, `/odom`, TF, laser scan, and camera
point cloud through costmap obstacle layers. RViz is started from a checked-in
config file.

## Target Shape

One depth-camera plugin publishes RGB, depth image, camera info, and PointCloud2
without duplicate camera namespaces. RViz displays RGB, depth, and point cloud.
The robot stays at the configured birth pose for every navigation run, and a
deterministic five-goal regression script verifies successful navigation.

## Key Decisions

- Keep the existing map and birth pose; do not solve navigation by teleporting
  the robot for each goal.
- Use the depth point cloud as the dynamic obstacle source.
- Keep laser clearing available while disabling its known misaligned marking.
- Validate action completion from `/move_base/status` and pose distance.

## Review Questions

- Are all depth topics publishing after a clean Gazebo restart?
- Does RViz subscribe to those topics rather than stale displays?
- Do five reachable goals reach `SUCCEEDED` without changing the birth pose?
