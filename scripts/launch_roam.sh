
#!/usr/bin/env bash
set -euo pipefail

# ==========================================
# ROAM Simulation Launcher
#
# Usage:
#   ./scripts/launch_roam.sh road
#   ./scripts/launch_roam.sh maze
#   ./scripts/launch_roam.sh forest
#
# Options:
#   --render-engine ogre
#   --render-engine ogre2
# ==========================================

PROJECT_ROOT="$(
    cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd
)"

WORLD="road"
RENDER_ENGINE=""
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# ==========================================
# ARGUMENTS
# ==========================================

while [[ $# -gt 0 ]]; do
    case "$1" in
        road|maze|forest)
            WORLD="$1"
            shift
            ;;
        --render-engine)
            if [[ $# -lt 2 ]]; then
                echo "Missing rendering engine."
                exit 1
            fi
            RENDER_ENGINE="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 {road|maze|forest} [--render-engine ogre|ogre2]"
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            exit 1
            ;;
    esac
done

# ==========================================
# WORLD CONFIGURATION
# ==========================================

case "$WORLD" in
    road)
        POSE="0 0 0.23 0 0 0"
        ;;
    maze)
        POSE="-4.5 -4.5 0.23 0 0 0"
        ;;
    forest)
        POSE="0 0 0.35 0 -0.12 0"
        ;;
esac

WORLD_FILE="$PROJECT_ROOT/worlds/$WORLD.sdf"
MODEL_FILE="$PROJECT_ROOT/models/roam_rover/model.sdf"
CONTROLLER="$PROJECT_ROOT/scripts/keyboard_drive.py"
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"

# ==========================================
# DEPENDENCY CHECKS
# ==========================================

for file in "$WORLD_FILE" "$MODEL_FILE" "$CONTROLLER" "$ROS_SETUP"; do
    if [[ ! -f "$file" ]]; then
        echo "Required file missing: $file"
        exit 1
    fi
done

# Jazzy's vendor packages expose gz through the ROS environment hooks.
# ROS setup scripts may reference unset variables.
set +u
source "$ROS_SETUP"
set -u

for cmd in gz python3 ros2; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "Missing dependency: $cmd"
        exit 1
    fi
done

# ==========================================
# RENDER ENGINE SELECTION
# ==========================================

if [[ -z "$RENDER_ENGINE" ]]; then
    RENDER_ENGINE="ogre2"

    if command -v systemd-detect-virt >/dev/null 2>&1; then
        if systemd-detect-virt -q; then
            RENDER_ENGINE="ogre"
        fi
    fi
fi

if [[ "$RENDER_ENGINE" != "ogre" &&
      "$RENDER_ENGINE" != "ogre2" ]]; then
    echo "Invalid renderer: $RENDER_ENGINE"
    exit 1
fi

# Do not force CPU software rendering.
unset LIBGL_ALWAYS_SOFTWARE
unset GALLIUM_DRIVER

# Help resolve Wayland / Qt compatibility issues.
if [[ "${XDG_SESSION_TYPE:-}" == "wayland" ]]; then
    export QT_QPA_PLATFORM=xcb
fi

# ==========================================
# GAZEBO RESOURCE CONFIGURATION
# ==========================================

export GZ_SIM_RESOURCE_PATH="$PROJECT_ROOT/models:${GZ_SIM_RESOURCE_PATH:-}"

# ==========================================
# TERMINAL DETECTION
# ==========================================

find_terminal() {
    for terminal in \
        gnome-terminal \
        xfce4-terminal \
        konsole \
        mate-terminal \
        xterm
    do
        if command -v "$terminal" >/dev/null 2>&1; then
            echo "$terminal"
            return 0
        fi
    done

    return 1
}

TERMINAL="$(find_terminal || true)"

if [[ -z "$TERMINAL" ]]; then
    echo "No supported terminal emulator found."
    echo "Install gnome-terminal, xfce4-terminal, konsole, mate-terminal, or xterm."
    exit 1
fi

# ==========================================
# TEMPORARY WORLD GENERATION
# ==========================================

TEMP_WORLD="$(mktemp "$PROJECT_ROOT/worlds/.roam_${WORLD}_XXXXXX.sdf")"
SIM_PID=""

cleanup() {
    if [[ -n "$SIM_PID" ]]; then
        kill "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi

    rm -f "$TEMP_WORLD"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Inject rover into a temporary world.
# The original SDF world is never modified.

python3 - "$WORLD_FILE" "$TEMP_WORLD" "$POSE" <<'PY'
import sys
import xml.etree.ElementTree as ET

source, destination, pose = sys.argv[1:]

tree = ET.parse(source)
root = tree.getroot()

world = root.find("world")

if world is None:
    raise RuntimeError("World element not found")

# Avoid duplicate rover insertion.
for include in world.findall("include"):
    name = include.findtext("name")
    uri = include.findtext("uri")

    if name == "roam_rover" or uri == "model://roam_rover":
        raise RuntimeError(
            "World already contains the ROAM rover"
        )

include = ET.SubElement(world, "include")

ET.SubElement(include, "uri").text = "model://roam_rover"
ET.SubElement(include, "name").text = "roam_rover"
ET.SubElement(include, "pose").text = pose

tree.write(
    destination,
    encoding="utf-8",
    xml_declaration=True
)
PY

# ==========================================
# START GAZEBO
# ==========================================

echo "===================================="
echo "ROAM Autonomous Rover Simulation"
echo "===================================="
echo "World:     $WORLD"
echo "Renderer:  $RENDER_ENGINE"
echo "ROS:       $ROS_DISTRO"
echo "Terminal:  $TERMINAL"
echo "Spawn:     $POSE"
echo "===================================="

gz sim -r "$TEMP_WORLD" \
    --render-engine "$RENDER_ENGINE" &

SIM_PID=$!

# ==========================================
# WAIT FOR ROVER DRIVE SUBSCRIBER
# ==========================================

echo "Waiting for Gazebo rover controller..."

READY=false

for ((i=0; i<60; i++)); do

    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "Gazebo exited unexpectedly."
        exit 1
    fi

    INFO="$(gz topic -i -t /rover/cmd_vel 2>/dev/null || true)"

    # A DiffDrive subscription should appear in the
    # Subscribers section as gz.msgs.Twist.
    if printf '%s\n' "$INFO" | awk '
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
    echo "Timed out waiting for rover drive subscriber."
    echo "Check the Gazebo model and DiffDrive plugin."
    exit 1
fi

echo "Rover drive controller detected."

# ==========================================
# START KEYBOARD CONTROLLER
# ==========================================

echo "Launching keyboard controls..."

# This command is deliberately run in a new shell.
# It avoids the parent's strict 'set -u' settings.
CONTROLLER_COMMAND='
    source "$1"
    cd "$2"
    python3 "$3"
'

case "$TERMINAL" in

    gnome-terminal)
        gnome-terminal -- \
            bash -c "$CONTROLLER_COMMAND" \
            _ "$ROS_SETUP" "$PROJECT_ROOT" "$CONTROLLER"
        ;;

    xfce4-terminal)
        xfce4-terminal --disable-server \
            --command="bash -c 'source \"$ROS_SETUP\"; cd \"$PROJECT_ROOT\"; python3 \"$CONTROLLER\"'"
        ;;

    konsole)
        konsole -e \
            bash -c "$CONTROLLER_COMMAND" \
            _ "$ROS_SETUP" "$PROJECT_ROOT" "$CONTROLLER" &
        ;;

    mate-terminal)
        mate-terminal -- \
            bash -c "$CONTROLLER_COMMAND" \
            _ "$ROS_SETUP" "$PROJECT_ROOT" "$CONTROLLER"
        ;;

    xterm)
        xterm -e \
            bash -c "$CONTROLLER_COMMAND" \
            _ "$ROS_SETUP" "$PROJECT_ROOT" "$CONTROLLER" &
        ;;
esac

echo "===================================="
echo "ROAM simulation started."
echo "Close Gazebo or press Ctrl+C to exit."
echo "===================================="

wait "$SIM_PID" || true
