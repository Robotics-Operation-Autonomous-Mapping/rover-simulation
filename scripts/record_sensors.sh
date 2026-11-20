#!/usr/bin/env bash
# Record topics from the exact running assembly, including instance overrides.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if (( $# < 1 || $# > 2 )); then
    echo "Usage: $0 ASSEMBLY_DIRECTORY [BAG_OUTPUT]" >&2
    exit 2
fi
OUTPUT="${2:-roam-$(date +%Y%m%d-%H%M%S)}"
TOPIC_TEXT="$(python3 "$ROOT/scripts/validate_sensors.py" --assembly "$1" --print-topics)"
mapfile -t TOPICS <<< "$TOPIC_TEXT"
exec ros2 bag record --use-sim-time -o "$OUTPUT" "${TOPICS[@]}"
