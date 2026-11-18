#!/usr/bin/env bash
# Compatibility launcher: source the environment, then use the shared Python runner.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
for arg in "$@"; do
    if [[ "$arg" == --help || "$arg" == -h ]]; then
        exec python3 "$ROOT/scripts/run.py" --help
    fi
done
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ -f "$ROOT/install/setup.bash" ]]; then source "$ROOT/install/setup.bash"; fi
set -u
# Explicit calibration/profile selections override the legacy default.
PROFILE=(--profile default)
for arg in "$@"; do
    if [[ "$arg" == --calibration || "$arg" == --profile || "$arg" == --calibration=* || "$arg" == --profile=* ]]; then PROFILE=(); fi
done
exec python3 "$ROOT/scripts/run.py" --control "${PROFILE[@]}" "$@"
