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

## Acceptance still open

- The latest route stopped at P7 after two MoveBase recovery failures. P8–P10
  and HOME were not attempted.
- P2's latest image cuts or nearly cuts the three people's feet. P5 still crops
  the right-side person. P1, P3, P4, and P6 passed visual review in the latest
  run.
- At P2, map TF was about 2.1 cm from the recorded target, while calibrated
  Gazebo truth was about 9.9 cm from it. This mismatch is consistent with the
  P2 framing error. P5 still needs the same truth-versus-TF comparison.
- P7's path-aware escape candidates were safe and on the Navfn path, but the
  shortest candidate moved about 10.4 cm; the current minimum recovery-distance
  threshold is 12 cm, so the plugin rejected it. The next check will lower that
  threshold to 8 cm without changing any route point.

The latest complete diagnostics are in
`/home/sz/ros1_ws/navigation_diagnostics/20260926_photo_route_p3tol05_pathescape_01/`;
photos are in
`/home/sz/ros1_ws/photo_stops/standee_route_runs/20260926_p3tol05_pathescape_01/`.
The P3 images and route summary are preserved there; no full-route acceptance is
claimed.

The original round-8 bag was overwritten by a recorder started with a reused
directory. Its photos, P3 attempt samples, and report remain, but that bag is
unavailable. All later runs use unique directories.
