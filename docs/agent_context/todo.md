---
type: todo
status: active
updated_by: orchestrator
---

# Active Work

## task006 — White-line-safe navigation

- [ ] Architecture review for matching global/local white-line costmaps and fail-closed watchdog.
- [ ] Implement/review config and launcher checks; preserve current no-goal/stopped state.
- [ ] Recheck Navfn plans and full-footprint swept paths for all ordered route legs.
- [ ] Capture one autonomous run with white-line enforcement enabled and verify no paint contact/crossing.

## task007 — Detector and OCR

- [ ] Load best.pt only after confirming its class map and safe inference path.
- [ ] Validate detector on representative traffic-light, person, and plate frames.
- [ ] Add terminal/image output correlation and expose plate crops for user-managed OCR; do not implement OCR.

## task008 — Full acceptance run

- [ ] Re-run the fixed 10-point photo sequence and HOME after task006/007 reviews.
- [ ] Resolve P5/P7 framing by evidence while preserving the authorized point order.
