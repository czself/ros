---
task_id: task003
type: plan
status: ready_for_implementation
from: planner
to: coder
revision: 2
requires_review: true
---
# Task003 Revision 2: genuine laser SLAM map and coverage evidence

Revision 1 was implemented and independently re-surveyed. This revision records
the final acceptance criteria and the evidence a successful run must bind.

## Required behavior (unchanged from revision 1)

- Mapping mode has only model_state_odom, robot TF, GMapping, and one mapper motion source.
- No map_server, AMCL, explore, navigation, teleop, or stale TF publisher.
- Mapper uses current axle/chassis odometry and fresh `/scan`; it stops on stale
  scan/pose, near obstacle, timeout, invalid scan, blocked route, or route
  completion and records the actual stop reason.
- Use deterministic route segments in the actual world, inspect `/map` growth and trajectory. Do not teleport after mapping starts.
- Save only when the complete `/map` publisher set before and after save is
  exactly `['/slam_gmapping']`; compute SHA256 and compare against all
  historical truth hashes; write a JSON manifest with measured chassis birth,
  derived axle/base birth, topic rates, actual `/tf` and `/tf_static` caller IDs,
  route/trajectory hash, mapper hash, completion-log hash, and stop reason.
- Generate a coverage report from the inflated traversable map that separates
  observed/free/occupied/unknown cells, reachable frontiers, and
  blocked/unreachable components. Do not use canvas `known_fraction` as a
  physical-area coverage claim. Proof of "no unclassified reachable frontier" is
  the coverage gate, not a lane-known percentage.
- Reject map if it is byte/pixel identical to `competition_ground_truth.pgm` or if coverage remains unverified.

## Revision 2 evidence requirements

- The executed mapper must be the wall-clock / stale-sensor / blocked-route
  watchdog version (host `scripts/survey_mapper.py`), deployed at session start;
  its hash must match the manifest `mapper_sha256` and the archived
  `*_mapper.py`.
- The single survey run must complete in one uninterrupted autonomous route
  starting at measured birth `(4.0833,-4.0833,1.5708)`, recorded
  `stop_reason=route_complete`, 117 waypoints (97 original + south-corridor
  sweep), with a trajectory whose first point matches the configured chassis birth.
- `coverage_report_gate_a_candidate=true` and `coverage_report_frontier_cells=0`
  are required for acceptance.
- Live runtime evidence: `/map` sole publisher `/slam_gmapping`, `/map`~2.5 Hz,
  `/scan` 10 Hz, depth points present, no blocking nodes, sampled at the save
  session.

## Verification (as revision 1)

`start_autonomous_mapping.sh`; one mapper route at a time; `rostopic info /map /scan`;
sampled `/tf` and `/tf_static` caller IDs; manifest; PGM dimensions/hist/hash;
compare to truth; artifact hash re-computation; coverage report; RViz map/scan;
static parsing. Use `map_saver` only after provenance checks.

## Out of scope

AMCL/five-goal acceptance (task004), changing walls to make route reachable,
copying truth maps, declaring full-world coverage from elapsed time alone,
runtime dynamic-obstacle and patrol acceptance (separate later items).

## Acceptance

The map has a non-truth hash, `/map` was published only by GMapping during save,
the manifest binds map/trajectory/mapper/log hashes and measured birth frames,
the coverage report shows gate_a_candidate=true with zero reachable frontier,
and the reviewer can reproduce rejection if the map source or any evidence is
swapped. If reachable unknown remains, the task stays incomplete and the map is
marked partial.