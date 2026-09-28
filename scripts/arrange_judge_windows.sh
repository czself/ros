#!/usr/bin/env bash
set -euo pipefail
export DISPLAY="${DISPLAY:-:1}"
for _ in $(seq 1 20); do
  WINDOWS="$(wmctrl -lG)"
  GAZEBO_ID="$(printf '%s\n' "$WINDOWS" | awk '/ Gazebo$/{print $1;exit}')"
  RVIZ_ID="$(printf '%s\n' "$WINDOWS" | awk '/RViz$/{print $1;exit}')"
  VISION_ID="$(printf '%s\n' "$WINDOWS" | awk '/JudgeVision$/{print $1;exit}')"
  TERMINAL_ID="$(printf '%s\n' "$WINDOWS" | awk '/比赛识别终端/{print $1;exit}')"
  if [[ -n "$GAZEBO_ID" && -n "$RVIZ_ID" && -n "$TERMINAL_ID" ]]; then break; fi
  sleep .5
done
for id in "$GAZEBO_ID" "$RVIZ_ID" "$TERMINAL_ID" "$VISION_ID"; do
  [[ -z "$id" ]] || wmctrl -ir "$id" -b remove,maximized_vert,maximized_horz
done
[[ -z "$GAZEBO_ID" ]] || wmctrl -ir "$GAZEBO_ID" -e 0,62,32,590,574
[[ -z "$RVIZ_ID" ]] || wmctrl -ir "$RVIZ_ID" -e 0,660,32,590,574
[[ -z "$TERMINAL_ID" ]] || wmctrl -ir "$TERMINAL_ID" -e 0,62,625,1848,425
[[ -z "$VISION_ID" ]] || wmctrl -ir "$VISION_ID" -e 0,1260,32,650,574
for id in "$GAZEBO_ID" "$RVIZ_ID" "$TERMINAL_ID" "$VISION_ID"; do
  [[ -z "$id" ]] || wmctrl -ia "$id"
done
