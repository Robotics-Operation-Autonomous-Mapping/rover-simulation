#!/usr/bin/env bash
# Run the wheel model in one of the existing club worlds.
set -euo pipefail

export GZ_PARTITION="${GZ_PARTITION:-roam_rover_wheels}"
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
world_name=road
if [[ "${1:-}" == --world ]]; then
    if (( $# < 2 )); then
        echo 'Usage: run_rover.sh [--world road|maze|forest] [gz sim options]' >&2
        exit 2
    fi
    world_name="$2"
    shift 2
fi
if [[ "${1:-}" == -h || "${1:-}" == --help ]]; then
    echo "Usage: $0 [--world road|maze|forest] [gz sim options]"
    echo "Example: $0 --world maze -s"
    exit 0
fi
case "$world_name" in
    road|maze|forest) ;;
    *) echo "Unsupported world: $world_name. Choose road, maze or forest." >&2; exit 2 ;;
esac

set +u
source /opt/ros/jazzy/setup.bash
set -u
command -v gz >/dev/null || { echo 'Gazebo is missing. Install ros-jazzy-ros-gz.' >&2; exit 1; }

runtime_dir="$(mktemp -d "${TMPDIR:-/tmp}/roam_rover.XXXXXX")"
trap 'rm -rf -- "$runtime_dir"' EXIT

# Compose only the temporary file; keep the club worlds unchanged.
/usr/bin/python3 - "$workspace_dir" "$world_name" "$runtime_dir/rover.sdf" <<'PY'
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

workspace = Path(sys.argv[1])
world_name = sys.argv[2]
# maze.sdf has whitespace before its XML declaration.
root = ET.fromstring((workspace / 'worlds' / f'{world_name}.sdf').read_text().lstrip())
world = root.find('world')
model = ET.parse(workspace / 'models/basic_rover/model.sdf').getroot().find('model')

# Road surface is at z=0.05; forest ground is tilted and 0.5 m thick.
# Maze starts away from the walls that meet at the origin.
spawn_poses = {
    'road': '0 0 0.075 0 0 0',
    'maze': '-4.5 -4.5 0.025 0 0 0',
    'forest': '0 0 0.35 0 -0.12 0',
}
model.find('pose').text = spawn_poses[world_name]
world.append(model)

# Explicit world systems keep physics / GUI updates independent of user defaults.
for suffix, system in (
    ('physics', 'Physics'),
    ('user-commands', 'UserCommands'),
    ('scene-broadcaster', 'SceneBroadcaster'),
):
    name = f'gz::sim::systems::{system}'
    if not any(plugin.get('name') == name for plugin in world.findall('plugin')):
        ET.SubElement(world, 'plugin', filename=f'gz-sim-{suffix}-system', name=name)

if world.find('gui') is None:
    x, y, z, *_ = map(float, spawn_poses[world_name].split())
    gui = ET.SubElement(world, 'gui', fullscreen='false')
    scene = ET.SubElement(gui, 'plugin', filename='MinimalScene', name='3D View')
    ET.SubElement(scene, 'engine').text = 'ogre2'
    ET.SubElement(scene, 'scene').text = 'scene'
    ET.SubElement(scene, 'ambient_light').text = '0.5 0.5 0.5'
    ET.SubElement(scene, 'background_color').text = '0.69 0.80 0.89'
    ET.SubElement(scene, 'camera_pose').text = f'{x + 2.5} {y - 3} {z + 1.8} 0 0.372 2.266'
    for filename in ('GzSceneManager', 'InteractiveViewControl', 'CameraTracking'):
        ET.SubElement(gui, 'plugin', filename=filename, name=filename)
    controls = ET.SubElement(gui, 'plugin', filename='WorldControl', name='World control')
    ET.SubElement(controls, 'play_pause').text = 'true'
    ET.SubElement(controls, 'step').text = 'true'
    ET.SubElement(controls, 'use_event').text = 'true'

ET.ElementTree(root).write(sys.argv[3], encoding='utf-8', xml_declaration=True)
PY

gz sim -r "$runtime_dir/rover.sdf" "$@"
