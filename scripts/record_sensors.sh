#!/usr/bin/env bash
# Run from a ROS/workspace-sourced Ubuntu terminal alongside the simulation.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${1:-atlas}"
OUTPUT="${2:-roam-${PROFILE}-$(date +%Y%m%d-%H%M%S)}"
TOPIC_TEXT="$(python3 "$ROOT/scripts/validate_sensors.py" --profile "$PROFILE" --print-topics)"
mapfile -t TOPICS <<< "$TOPIC_TEXT"
exec ros2 bag record --use-sim-time -o "$OUTPUT" "${TOPICS[@]}"
