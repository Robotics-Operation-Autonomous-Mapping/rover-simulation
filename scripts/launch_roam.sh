#!/usr/bin/env bash
# Launch the instrumented rover, ROS sensor pipeline, controls, and RViz.
set -euo pipefail

PROJECT_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD=road
RENDER_ENGINE=""
SENSORS=true
SENSOR_VIEW=true
CONTROLS=true
HEADLESS=false
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

usage() {
    echo "Usage: $0 [road|maze|forest|roam_test] [options]"
    echo '  --render-engine ogre|ogre2  Gazebo GUI renderer (sensors use Ogre 2)'
    echo '  --no-view                  Run sensors without opening RViz'
    echo '  --no-controls              Do not open the keyboard control window'
    echo '  --no-sensors               Run Gazebo without ROS sensor nodes or RViz'
    echo '  --headless                 Run server and ROS sensors, without GUI windows'
}

while (( $# )); do
    case "$1" in
        road|maze|forest|roam_test) WORLD="$1"; shift ;;
        --render-engine)
            if (( $# < 2 )); then echo 'Missing rendering engine.' >&2; exit 2; fi
            RENDER_ENGINE="$2"; shift 2 ;;
        --no-view) SENSOR_VIEW=false; shift ;;
        --no-controls) CONTROLS=false; shift ;;
        --no-sensors) SENSORS=false; SENSOR_VIEW=false; shift ;;
        --headless) HEADLESS=true; CONTROLS=false; SENSOR_VIEW=false; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
    esac
done

ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ROS setup not found: $ROS_SETUP" >&2
    exit 1
fi
set +u
source "$ROS_SETUP"
set -u

if [[ "$SENSORS" == true ]]; then
    SENSOR_LAUNCH="$PROJECT_ROOT/install/my_robot/share/my_robot/launch/rover_sensors.launch.py"
    if [[ ! -f "$SENSOR_LAUNCH" ]]; then
        echo 'Build the sensor packages first, from the repository root:' >&2
        echo "  source $ROS_SETUP" >&2
        echo '  colcon build --packages-up-to my_robot' >&2
        exit 1
    fi
    set +u
    source "$PROJECT_ROOT/install/setup.bash"
    set -u
fi

for cmd in gz python3 ros2 setsid; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "Missing dependency: $cmd" >&2
        exit 1
    fi
done

if [[ -z "$RENDER_ENGINE" ]]; then
    RENDER_ENGINE=ogre2
    if command -v systemd-detect-virt >/dev/null 2>&1 && systemd-detect-virt -q; then
        RENDER_ENGINE=ogre
    fi
fi
if [[ "$RENDER_ENGINE" != ogre && "$RENDER_ENGINE" != ogre2 ]]; then
    echo "Invalid renderer: $RENDER_ENGINE" >&2
    exit 2
fi

if [[ "${XDG_SESSION_TYPE:-}" == wayland ]]; then
    export QT_QPA_PLATFORM=xcb
fi
export GZ_PARTITION="${GZ_PARTITION:-roam_rover_sensors}"
export GZ_SIM_RESOURCE_PATH="$PROJECT_ROOT/models:${GZ_SIM_RESOURCE_PATH:-}"

RUNTIME_DIR="$(mktemp -d "${TMPDIR:-/tmp}/roam-sensors.XXXXXX")"
SIM_PID=""
SENSOR_PID=""
CONTROL_PID=""
cleanup() {
    trap - EXIT INT TERM
    for pid in "$CONTROL_PID" "$SENSOR_PID"; do
        if [[ -n "$pid" ]]; then kill -TERM -- "-$pid" 2>/dev/null || true; fi
    done
    for pid in "$CONTROL_PID" "$SENSOR_PID"; do
        if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
    done
    # Each component has its own session, including any children it launches.
    if [[ -n "$SIM_PID" ]]; then
        kill -TERM -- "-$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi
    rm -rf -- "$RUNTIME_DIR"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

python3 "$PROJECT_ROOT/scripts/compose_rover_world.py" "$WORLD" "$RUNTIME_DIR/world.sdf"
echo "ROAM rover | world=$WORLD | renderer=$RENDER_ENGINE | partition=$GZ_PARTITION"
echo 'Attached sensors: RGB camera, 3D LiDAR, IMU, GNSS, wheel encoders'
echo 'INS is synthetic odometry with position noise, derived from simulation truth.'
SIM_OPTIONS=()
if [[ "$HEADLESS" == true ]]; then SIM_OPTIONS+=(-s); fi
setsid gz sim -r "$RUNTIME_DIR/world.sdf" --render-engine "$RENDER_ENGINE" "${SIM_OPTIONS[@]}" &
SIM_PID=$!

if [[ "$SENSORS" == true ]]; then
    setsid ros2 launch my_robot rover_sensors.launch.py "rviz:=$SENSOR_VIEW" &
    SENSOR_PID=$!
fi

if [[ "$CONTROLS" == true ]]; then
    echo 'Waiting for the rover drive controller...'
    READY=false
    for ((i=0; i<60; i++)); do
        if ! kill -0 "$SIM_PID" 2>/dev/null; then
            echo 'Gazebo exited before the rover was ready.' >&2
            exit 1
        fi
        if [[ -n "$SENSOR_PID" ]] && ! kill -0 "$SENSOR_PID" 2>/dev/null; then
            echo 'ROS sensor launch exited unexpectedly.' >&2
            exit 1
        fi
        if gz topic -i -t /rover/cmd_vel 2>/dev/null | awk '
            /^Subscribers/ { in_subscribers=1; next }
            in_subscribers && /gz.msgs.Twist/ { found=1 }
            END { exit !found }
        '; then
            READY=true
            break
        fi
        sleep 1
    done
    if [[ "$READY" != true ]]; then
        echo 'Timed out waiting for the rover drive controller.' >&2
        exit 1
    fi
    CONTROL_OPTIONS=()
    if [[ "$SENSORS" == true ]]; then CONTROL_OPTIONS+=(--no-bridge); fi
    setsid python3 "$PROJECT_ROOT/scripts/keyboard_drive.py" "${CONTROL_OPTIONS[@]}" &
    CONTROL_PID=$!
fi

echo 'Close Gazebo or press Ctrl+C here to stop the simulation and sensor nodes.'
if [[ -n "$SENSOR_PID" ]]; then
    wait -n "$SIM_PID" "$SENSOR_PID"
else
    wait "$SIM_PID"
fi
