# Navigation photo route progress — 2026-09-26

This branch records the route and localization work performed against the fixed
`POINT_1` through `POINT_10` photo route. It does not claim that every photo or
the final heading has passed acceptance.

## Verified progress

- The route executor uses the existing ten recorded photo poses in order and
  returns to the contract HOME pose. It records camera images, depth summaries,
  target/actual AMCL pose, and the configured acceptance criterion.
- Navigation startup checks simulated time, laser and wheel-odometry stamps,
  map-to-base TF, and the `move_base` action server before dispatching goals.
- Photo navigation allows the normal low-speed reverse sample
  (`min_vel_x=-0.08`) at the recorded points. The known P6-to-P7 short approach
  remains forward-only.
- With reverse available, the second complete run reached all ten points and
  completed the HOME action. P3-to-P4 changed from repeated DWA failure and a
  roughly 159-degree in-place turn to a successful 0.95 m traverse with about
  87 degrees of heading change.

## Acceptance still open

- P5 cuts off the fourth person at the right edge; P7 cuts off the top of the
  traffic-light unit.
- At P5, AMCL reported about 2 cm position error, while synchronized Gazebo
  truth placed the base about 11 cm from the recorded target. This explains the
  crop and points to localization/odometry drift rather than a bad camera or
  lidar mounting transform.
- The executor accepted HOME using AMCL at 2.4 cm / 1.91 degrees. Synchronized
  Gazebo truth was about 2.5 cm / 8.17 degrees from HOME, so the final heading
  did not meet the 0.06 rad requirement.
- The lidar and camera transforms relative to the chassis matched Gazebo link
  state during the run. Wheel-odometry calibration and AMCL correction are the
  remaining localization work; the photo points and sensor extrinsics were not
  changed in this run.

The full run telemetry is retained outside the repository at
`/home/sz/ros1_ws/navigation_diagnostics/20260926_photo_route_retry_reverse/`
(28 readable rosbag segments, about 29 GB). The photos are in
`/home/sz/ros1_ws/photo_stops/standee_route_runs/20260926_1328_route_retry_reverse/`.
