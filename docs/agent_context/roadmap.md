---
type: roadmap
status: active
updated_by: orchestrator
---

# Roadmap

## Phase 1: Navigation And Line Compliance — task006

- [ ] Review the revised global/local costmap and watchdog architecture.
- [ ] Prove the live local costmap uses the same white-line map as the global costmap.
- [ ] Validate all ordered route legs with full-footprint checks; do not move if any leg or turn sweep is blocked.
- [ ] Run a bagged autonomous route with the white-line gate active and verify no line contact/crossing.
- [ ] Verify no keyboard/teleop publisher is active during the run.

## Phase 2: Perception And Reporting — task007

- [ ] Validate the provided best.pt classes on recorded camera frames.
- [ ] Integrate detector outputs for resident/stranger, traffic-light ON/OFF states, and license-plate boxes.
- [ ] Expose plate boxes, confidence, and annotated crops for the user's OCR stage (OCR implementation is out of scope).
- [ ] Publish ordered console output, annotated images, and street-wide person totals.

## Phase 3: End-To-End Mission — task008

- [ ] Run the diagram route and POINT_1 through POINT_10 in order.
- [ ] Require every photo acceptance criterion, including four people at P5 and full signal at P7.
- [ ] Obey red/green light rules at all measured stop lines.
- [ ] Return to the same HOME position and orientation, stop with zero velocity, and save independent evidence.

## Phase 4: Delivery

- [ ] Preserve plans, summaries, reviews, command logs, bag metadata, route images, and detector outputs.
- [ ] Push reviewed changes to GitHub on codex/navigation-photo-run-progress.
- [ ] Produce a final task report linking the code, photos, detector output, and compliance evidence.
