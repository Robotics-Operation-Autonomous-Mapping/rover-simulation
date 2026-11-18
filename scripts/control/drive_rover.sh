#!/usr/bin/env bash
# Send timed velocity commands through a temporary one-way ROS-Gazebo bridge.
set -euo pipefail

# Keep this workspace separate from other Gazebo simulations on this VM.
export GZ_PARTITION="${GZ_PARTITION:-roam_rover_wheels}"
if [[ "${1:-}" == -h || "${1:-}" == --help ]]; then
    echo "Usage: $0 LINEAR_M_PER_S ANGULAR_RAD_PER_S [DURATION_SECONDS]"
    echo "Example: $0 0.3 0 3"
    echo "Stop: $0 0 0 0"
    exit 0
fi
if (( $# < 2 || $# > 3 )); then
    echo "Usage: $0 LINEAR_M_PER_S ANGULAR_RAD_PER_S [DURATION_SECONDS]" >&2
    exit 2
fi
validated=$(/usr/bin/python3 - "$1" "$2" "${3:-2}" <<'PY'
import math
import sys
try:
    linear, angular, duration = map(float, sys.argv[1:])
    assert all(math.isfinite(x) for x in (linear, angular, duration))
    assert abs(linear) <= 1.0 and abs(angular) <= 1.5 and 0 <= duration <= 60
except (ValueError, AssertionError):
    sys.exit('Use finite numbers: linear +/-1 m/s, angular +/-1.5 rad/s, duration 0..60 s.')
print(linear, angular, duration)
PY
)
read -r linear angular duration <<< "$validated"
set +u
source /opt/ros/jazzy/setup.bash
set -u
command -v gz >/dev/null || { echo "Gazebo is missing." >&2; exit 1; }
# Do not silently publish into a simulation without the drive plugin.
subscriber_info=$(gz topic -i -t /rover/cmd_vel)
if ! [[ "$subscriber_info" == *"Subscribers ["* ]]; then
    echo "No rover drive subscriber in GZ_PARTITION=$GZ_PARTITION." >&2
    echo "Restart ~/rover-simulation/scripts/run_rover.sh, then retry this command." >&2
    exit 1
fi
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$script_dir/drive_rover.py" "$linear" "$angular" "$duration"
