#!/usr/bin/env bash
set -eo pipefail
WORKSPACE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
set -u
RUN_ID="${1:-}"
[[ "$RUN_ID" =~ ^[A-Za-z0-9_-]+$ ]] || { echo '用法: ./native_audit.sh <RUN_ID>' >&2; exit 2; }
PROJECT="$WORKSPACE/src/community_inspection"
RUN="$WORKSPACE/runtime_data/$RUN_ID"
export COMPETITION_WORLD_PATH="$RUN/assets/competition_native.world"
export CAR_STANDEE_DIR="$RUN/assets/car_standees"
python3 "$PROJECT/scripts/audit_standee_photo_run.py" "$RUN" \
  --texture "$PROJECT/models/competition_ground/materials/textures/map.png" \
  --require-command-decisions --output "$RUN/independent_audit.json" > "$RUN/audit.log" 2>&1
python3 "$PROJECT/scripts/navigation_phase_metrics.py" "$RUN" \
  --route-config "$PROJECT/navigation/standee_photo_route.json" \
  --output "$RUN/navigation_phase_metrics.json" > "$RUN/phase_audit.log" 2>&1
python3 "$PROJECT/scripts/analyze_docs20_run.py" "$RUN" \
  > "$RUN/supplementary_audit.log" 2>&1
if [[ -f "$RUN/judge/recognition_terminal.txt" ]]; then
  python3 "$PROJECT/scripts/audit_judge_presentation.py" "$RUN" \
    --output "$RUN/judge/presentation_audit.json" > "$RUN/judge/presentation_audit.log" 2>&1
fi
echo "审计结果: $RUN/independent_audit.json"
