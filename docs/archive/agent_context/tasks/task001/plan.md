---
task_id: task001
type: plan
status: ready_for_implementation
from: planner
to: coder
revision: 0
requires_review: true
---

# Task 001 Plan: Depth RViz and five-goal navigation

## Goal

Make depth RGB/depth/point-cloud displays reliable and verify navigation to
five reachable goals from the unchanged birth pose.

## Relevant Files

- `worlds/competition_classic.world`
- `navigation/common_costmap.yaml`
- `navigation/global_costmap.yaml`
- `navigation/local_costmap.yaml`
- `navigation/navigation.launch`
- `scripts/competition.rviz`
- `scripts/start_navigation.sh`

## Required Changes

- Ensure one depth camera plugin publishes all camera topics without duplicate
  services.
- Ensure RViz config subscribes to active RGB, depth, and PointCloud2 topics.
- Add a deterministic validation script for five reachable map goals; reset to
  the fixed birth pose only once before the goal sequence.
- Do not modify the birth coordinates.

## Verification

```bash
./scripts/start_sim.sh
./scripts/start_navigation.sh /root/ros1_ws/maps/competition_ground_truth.yaml
./scripts/validate_navigation.sh
```

## Acceptance Criteria

- `/camera/image_raw`, `/camera/depth/image_raw`, and `/camera/depth/points`
  publish continuously.
- RViz has active Camera, DepthImage, and DepthPointCloud displays.
- Five goals report `SUCCEEDED`; no goal is reached by teleporting.
- The initial `/odom` pose matches `(4.0833, -4.0833, 1.5708)` before goals.
