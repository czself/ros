---
type: roadmap
status: active
updated_by: planner
---

# Roadmap

## Active repair sequence

task001 failed acceptance. task002 foundation is complete enough to support the
current runtime, but task003 still has reviewer changes requested. Do not enter
navigation acceptance until the SLAM gate is approved. See
`goal_slam_navigation.md` for the formal gates.

## Goal A: birth-connected SLAM coverage

- [ ] Revise task003 provenance and runtime evidence.
- [ ] Drive every birth-connected, robot-clear reachable frontier.
- [ ] Produce a coverage report separating observed, reachable-unknown, and
      blocked/unreachable components.
- [ ] Save a non-truth map whose trajectory, mapper, completion log, and TF
      evidence hashes match the manifest.
- [ ] Independent review approves the full-coverage claim or records the exact
      remaining blocker.

## Goal B: verified-map navigation

- [ ] Cleanly switch from mapping to navigation with exclusive map/TF/cmd_vel
      authorities.
- [ ] Confirm laser and depth observations enter runtime local costmap.
- [ ] Sample five dispersed goals in the inflated birth-connected component.
- [ ] Run one fixed-birth, no-teleport sequence and record 5/5 `SUCCEEDED`.
- [ ] Capture RViz map, scan, point cloud, plans, costmaps and footprint.

## Later

- [ ] Re-enable laser marking after TF/map alignment is corrected.
- [ ] Handle wall-height/camera visibility as a separate scene-change task;
      changing geometry requires a new SLAM map and both gates again.
