# Repair todo

## Goal A: complete birth-connected SLAM coverage

- [x] task002: foundation and controlled-motion evidence
- [ ] task003 revision 1: true SLAM coverage, frontier report, and provenance
- [ ] Reconcile each audit item with fix, retirement, or documented limit
- [ ] Independent review approves Gate A

## Goal B: verified-map navigation

- [ ] task004: five-goal navigation and RViz acceptance (blocked by Gate A)
- [ ] Confirm exclusive map/TF/cmd_vel authorities at runtime
- [ ] Confirm laser/depth obstacle evidence in local costmap
- [ ] Record five dispersed goals with 5/5 `SUCCEEDED`

## Explicit non-goals for these gates

- [ ] Do not copy or overlay the truth map as SLAM output
- [ ] Do not teleport between goals or into closed rooms
- [ ] Do not lower/remove scene walls during this acceptance cycle
