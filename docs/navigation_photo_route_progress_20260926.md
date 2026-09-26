# Navigation photo route progress — 2026-09-26

The route still uses the ten recorded photo poses in order and returns to the
original HOME pose. No photo coordinates were changed and no hidden alignment
or transition goals are inserted.

## Verified improvements

- Removed the executor's hidden in-place alignment goals before POINT_5 and
  POINT_7. Each navigation action now targets the recorded point directly.
- Added a positive DWA `twirling_scale` for photo navigation. In P3 testing,
  near-pure-rotation commands fell from about 62% to about 35% of samples.
- Reworked SafeEscape to subscribe to the active Navfn plan, score collision-
  checked arcs by path progress and cross-track error, and prefer endpoints
  outside the inscribed obstacle halo.
- P3 was reached and photographed after setting only P3's XY navigation and
  capture tolerances to 0.05 m. The resulting photo shows all three people
  fully in frame; its map-TF error was about 3.3 cm and heading error about
  2 degrees.
- Disabling continuous DWA trajectory-cloud publication restored Gazebo
  performance; the P1 navigation time returned to roughly 9 seconds after a
  diagnostic run with trajectory-cloud publishing took about 31 seconds.

## Round 15 result

The full route reached POINT_1 through POINT_10 in order, returned to HOME, and
stopped with zero commanded velocity. The run summary is `COMPLETE_PARKED`.
P5 and P8 each timed out once at 75 seconds and succeeded on the executor's
single retry. The other goals succeeded on their first attempt.

Visual review of the ten captures:

- P1–P4, P6, and P8–P10 passed the recorded framing criteria.
- P5 failed: only three of the required four people are visible; the rightmost
  person is cut off.
- P7 failed: the top of the traffic-light fixture is clipped.

AMCL reported HOME within 2.6 cm and 0.036 rad of the saved pose. A later
Gazebo-truth sample was within 2.7 cm and 0.10 rad, but it was taken 47 seconds
after completion; the same-stamp HOME truth comparison is still pending.

The full rosbag (533 MB, 571 seconds, 1,896,936 messages) and captures are in:

- `/home/sz/ros1_ws/navigation_diagnostics/20260926_photo_route_alpha20_min05_pathaware_01/`
- `/home/sz/ros1_ws/photo_stops/standee_route_runs/20260926_alpha20_min05_pathaware_01/`

## Cause of the repeated turning

The recovery plugin was choosing sharp arcs with angular speed up to 1.10
rad/s, then asking MoveBase to replan the same goal. Some accepted arcs moved
only 7 cm. At P8, it accepted a 7 cm arc with negative path-progress score
(-0.067) while cross-track error grew from 0.005 m to 0.054 m. The old rejection
threshold allowed that arc. P5 is especially sensitive because P4→P5 is only
about 0.17 m and the saved heading reverses by almost 180 degrees. There was no
DWA “failed to produce path” log at P5; its recovery sequence followed lack of
useful progress rather than a confirmed absence of local trajectories.

The recovery code now limits its candidate arcs to ±0.38 rad/s, samples only
durations of 1.2–2.0 seconds, requires at least 0.12 m of travel, and rejects
arcs with path score below 0.03 or more than 0.05 m extra cross-track error.
No route coordinates changed. The ROS package compiled successfully inside
the `ros1_modeling` container; the rebuilt plugin has not yet been loaded by a
restarted MoveBase or validated in a new route run.

Photo acceptance is still open until P5/P7 pass and HOME truth is confirmed.

The original round-8 bag was overwritten by a recorder started with a reused
directory. Its photos, P3 attempt samples, and report remain, but that bag is
unavailable. All later runs use unique directories.
