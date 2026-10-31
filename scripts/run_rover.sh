#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ -f "$ROS_SETUP" ]]; then
    set +u
    source "$ROS_SETUP"
    set -u
fi
export GZ_SIM_RESOURCE_PATH="$ROOT/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_PARTITION="${GZ_PARTITION:-roam_rover_wheels}"
exec gz sim -r "$ROOT/worlds/roam_test.sdf"
