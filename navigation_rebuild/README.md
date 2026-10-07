# Navigation rebuild

This is a new navigation path following the reference project's structure:
saved map + AMCL + `move_base`, a declared goal list, one Action client, and
tasks that run after verified arrival. The old `route_executor.py` is not used.
AMCL receives the original SLAM map on `/map_localization`; planners and the
velocity gate receive the separate white-line occupancy overlay on `/map`.

Start from a fresh Gazebo birth pose with:

```bash
./scripts/start_navigation_rebuild.sh current_slam_preview_white_lines
```

For the first short navigation check, stop after POINT_1 without running
inspection tasks:

```bash
RUN_TASKS=false STOP_AFTER=POINT_1 ./scripts/start_navigation_rebuild.sh
```

Results are written to `~/ros1_ws/navigation_rebuild_runs/<timestamp>/`.
Any navigation or inspection failure cancels the active goal and ends the run.
The velocity gate is the only publisher to Gazebo during this mode. It checks
the saved occupancy map, command freshness, and traffic stop lines. Photos
are raw captures with pose metadata; semantic photo acceptance still requires
separate visual verification.
