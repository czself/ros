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

The first recovery revision limited candidate arcs to ±0.38 rad/s. After the
P5 failure in Round 16, the current candidate restores higher-curvature arcs
for tight spaces, samples only durations of 1.2–2.0 seconds, requires at least
0.12 m travel, penalizes angular speed, and rejects path score below 0.03 or
more than 0.05 m extra cross-track error. No route coordinates changed. The
package compiles inside `ros1_modeling`; route validation is pending.

Photo acceptance is still open until P5/P7 pass and HOME truth is confirmed.

## Round 16: recovery constraint check

The first low-curvature-only revision reached P1–P4 faster (about 11, 14, 22,
and 22 seconds) and did not execute the old repeated tight loops. It then failed
at P5: attempt 1 ended `NO_PROGRESS`, attempt 2 aborted, and no P5 photo was
taken. Both recovery probes found no collision-free long low-curvature arc
(endpoint cost 254). P6–P10 were not attempted. The bag is preserved at
`/home/sz/ros1_ws/navigation_diagnostics/20260926_photo_route_alpha20_min12_pathaware_01/`.

The next candidate keeps the 0.12 m minimum travel and the path-score/cross-
track gates, starts duration sampling at 1.2 seconds, and restores higher
curvature candidates only for cases where the local costmap rejects gentler
arcs. It applies an angular-rate penalty so a useful tight arc can pass while
7 cm micro-arcs cannot. This revision compiles, but has not yet been loaded or
tested in a route run.

## Round 16 P5 diagnosis and HOME return

The Round 16 bag confirms this is a short-goal control oscillation, not a DWA
planner with no path. At P5 the Navfn path stayed valid and DWA published local
plans, usually only two poses long. During both attempts, forward and reverse
commands each appeared in roughly 40–50% of samples, while pure rotation was
only 6–8%; net forward progress was near zero. SafeEscape's low-curvature-only
revision found no collision-free long arc in the live costmap, so the route
stopped at P5. The current local map had lethal wall cells and scan returns
0.18–0.22 m away around the short P4→P5 turn, despite a roughly 0.7 m visual
corridor. See
`/home/sz/ros1_ws/navigation_diagnostics/20260926_photo_route_alpha20_min12_pathaware_01/p5_no_progress_diagnosis.md`.

As a targeted next test, POINT_5 now disables reverse velocity, matching the
existing POINT_7 exception. This addresses the measured forward/reverse
alternation without changing the route or inserting an alignment goal. It has
not yet been run.

After the failed Round 16 route, navigation was restarted from the measured
current Gazebo pose (no teleport) and the rebuilt higher-curvature plugin was
loaded. One HOME action succeeded under AMCL's 0.05 m / 0.10 rad tolerances,
but truth at arrival was about 2.5 cm and 6.1 degrees from HOME. Tightening
the final tolerances made MoveBase report success while truth still differed
by about 4 cm and 5 degrees. With zero commanded velocity, the Gazebo chassis
continued to yaw slowly (about 0.00042 rad/s). A later same-HOME correction
ended in a 2.3 rad heading error and MoveBase oscillation abort. The vehicle
was stationary afterward; the 10-point route and exact HOME acceptance remain
open. The full return-attempt bag is
`/home/sz/ros1_ws/navigation_diagnostics/20260926_round16_final_home_relocalize_return_04/home_return_0.bag`.

The original round-8 bag was overwritten by a recorder started with a reused
directory. Its photos, P3 attempt samples, and report remain, but that bag is
unavailable. All later runs use unique directories.
