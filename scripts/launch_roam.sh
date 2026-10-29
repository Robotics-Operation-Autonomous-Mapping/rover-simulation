
#!/usr/bin/env bash
set -euo pipefail

# ==========================================
# ROAM Simulation Launcher
# Usage: ./scripts/launch_roam.sh maze
# ==========================================

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD="${1:-road}"

# Available worlds and rover spawn positions
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
    *)
        echo "Unknown world: $WORLD"
        echo "Usage: $0 {road|maze|forest}"
        exit 1
        ;;
esac

WORLD_FILE="$PROJECT_ROOT/worlds/$WORLD.sdf"

if [[ ! -f "$WORLD_FILE" ]]; then
    echo "World file not found: $WORLD_FILE"
    exit 1
fi

if ! command -v gnome-terminal >/dev/null; then
    echo "gnome-terminal is required."
    exit 1
fi

# Locate our custom Gazebo models
export GZ_SIM_RESOURCE_PATH="$PROJECT_ROOT/models:${GZ_SIM_RESOURCE_PATH:-}"

# Use OGRE 1 for VMware compatibility
unset LIBGL_ALWAYS_SOFTWARE
unset GALLIUM_DRIVER

# Set up ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# ==========================================
# Create temporary world with rover included
# ==========================================

TEMP_WORLD="$(mktemp "$PROJECT_ROOT/worlds/.roam_${WORLD}_XXXXXX.sdf")"
SIM_PID=""

cleanup() {
    if [[ -n "$SIM_PID" ]]; then
        kill "$SIM_PID" 2>/dev/null || true
    fi
    rm -f "$TEMP_WORLD"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

python3 - "$WORLD_FILE" "$TEMP_WORLD" "$POSE" <<'PY'
import sys
import xml.etree.ElementTree as ET

source, destination, pose = sys.argv[1:]

tree = ET.parse(source)
root = tree.getroot()

world = root.find("world")
if world is None:
    raise RuntimeError("No <world> element found")

# Insert the rover without modifying the original world
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
# Launch Gazebo
# ==========================================

echo "Starting ROAM simulation..."
echo "World: $WORLD"
echo "Rover spawn: $POSE"

gz sim -r "$TEMP_WORLD" --render-engine ogre &
SIM_PID=$!

# Wait until the drive command topic exists
echo "Waiting for rover drive controller..."

READY=false

for ((i=0; i<100; i++)); do
    if ! kill -0 "$SIM_PID" 2>/dev/null; then
        echo "Gazebo exited unexpectedly."
        exit 1
    fi

    if gz topic -l 2>/dev/null | grep -Fxq "/rover/cmd_vel"; then
        READY=true
        break
    fi

    sleep 0.5
done

if [[ "$READY" != true ]]; then
    echo "Rover drive topic was not detected."
    echo "Check Gazebo output for loading errors."
    exit 1
fi

# ==========================================
# Launch keyboard controller
# ==========================================

echo "Opening keyboard controls..."

gnome-terminal -- bash -lc \
    "source /opt/ros/jazzy/setup.bash; \
     cd \"$PROJECT_ROOT\"; \
     python3 scripts/keyboard_drive.py; \
     exec bash"

echo "ROAM simulation running."
echo "Close Gazebo or press Ctrl+C here to stop."

wait "$SIM_PID" || true
