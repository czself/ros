---
type: project_context
status: active
updated_by: planner
---

# Project Context

## Purpose

ROS Noetic/Gazebo Classic mobile-robot navigation demo with RViz visualization.

## Tech Stack

- Gazebo Classic 11 and ROS Noetic in Docker container `ros1_modeling`
- `move_base`, Navfn, DWA, map_server, AMCL
- SDF world and shell launch scripts

## Current Architecture Facts

- Gazebo world: `worlds/competition_classic.world`
- Navigation launch/config: `navigation/`
- Startup: `scripts/start_sim.sh`, `scripts/start_navigation.sh`
- Ground-truth map: `maps/competition_ground_truth.yaml`
- Ground-truth odom is published by `scripts/model_state_odom.py`.
- RViz config: `scripts/competition.rviz`

## Constraints

- Birth pose must remain fixed at `(4.0833, -4.0833, yaw=1.5708)`.
- Validation runs inside Docker container `ros1_modeling`.
- Preserve the existing static map and avoid changing unrelated scene geometry.

## Known Risks

- Gazebo and RViz can leave stale processes after crashes.
- Competing odom/TF publishers cause navigation oscillation.
- Depth and RGB plugins must not advertise duplicate camera services.
