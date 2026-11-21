# ROAM simulation — setup and commands

Run these commands in an Ubuntu 24.04 Bash terminal, from the repository root.
This guide can be copied elsewhere; paths in commands are relative to the repo.

## 1. Verify your installation

```bash
cd ~/Desktop/clubs/roam/rover-simulation
bash scripts/setup/verify_dependencies.sh
```

The checker makes no system changes. PASS means detected, FAIL means a required
check failed, WARN is non-blocking. It does not test graphics or live sensors.

- Required for the full workflow: ROS 2 Jazzy, Gazebo Harmonic (Sim 8), ROS–Gazebo
  bridge, Python, colcon and the build toolchain for workspace packages/messages.
- RViz is optional: use `--no-view` if unavailable.
- rosdep is optional dependency-management tooling. Its absence or lack of
  initialization does not block a build when dependencies are already installed.
- `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers` and
  `ros-jazzy-gz-ros2-control` are **not required**. The rover uses Gazebo's native
  DiffDrive system. They are no longer checked or requested by the installer.
- The checker no longer enforces a separate GCC 13/C++20 requirement. It still
  checks build-tool availability; Ubuntu 24.04 provides the target toolchain.

If required dependencies are missing, review and run the installer:

```bash
bash scripts/setup/install_dependencies.sh
```

It uses sudo/network access, may configure apt repositories, and installs the
full development/viewer environment. It also initializes/updates rosdep and runs
verification. Neither setup script takes command-line parameters. Existing
unused control packages are left installed; no uninstall is necessary.

## 2. Build after pulling code changes

Use a fresh terminal, then:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --build-base build-clean --install-base install-clean --packages-up-to my_robot
source install-clean/setup.bash
```

The first `source` exposes installed ROS tools. The build generates custom
messages and installs this project's nodes, launch files and assets.
`--packages-up-to my_robot` includes its workspace dependency `my_robot_msgs`.
The fresh directory names avoid old installed launch files from before cleanup.
Reuse these directories for subsequent builds. Stop if the build fails.

In **each new terminal** used for ROS commands:

```bash
cd ~/Desktop/clubs/roam/rover-simulation
source /opt/ros/jazzy/setup.bash
source install-clean/setup.bash
```

Rebuild after code/component changes; sourcing alone does not rebuild anything.
If you previously built into `install/`, use its setup file instead, consistently.

## 3. Launch

```bash
python3 scripts/run.py --world road --control
```

Default configuration: ATLAS with all its configured sensors. Starts Gazebo,
ROS bridges, TF, adapters and RViz. `--control` adds the keyboard controller:
W/S = left wheels, Up/Down = right wheels, Space = brake. Focus the controller
window to drive. Ctrl+C in the launching terminal stops the simulation.
Stop one simulation before starting another.

```bash
python3 scripts/run.py --world forest --control --no-view
python3 scripts/run.py --world maze --headless
python3 scripts/run.py --world camera_test --calibration calibration/camera.json
python3 scripts/run.py --world road --disable gnss --disable gnss_velocity
```

### Input parameters (run.py, assemble.py and validate.py)

| Parameter | Meaning / default |
| --- | --- |
| `--rover NAME` | Reusable rover directory name; default `atlas`. |
| `--world NAME` | World filename without `.sdf`; default `road`. Available: `road`, `forest`, `maze`, `roam_test`, `camera_test`, `empty`. |
| `--calibration PATH` | Sensor-instance configuration; default `calibration/atlas.json`. JSON or YAML (YAML needs PyYAML). |
| `--enable ID` | Enable a configured instance; repeat for multiple IDs. |
| `--disable ID` | Disable a configured instance; repeat for multiple IDs. |
| `--spawn-pose "X Y Z R P Y"` | Override world spawn: metres and roll/pitch/yaw in radians. Example: `--spawn-pose "0 0 0.23 0 0 0"`. |
| `-h`, `--help` | Show the command's parameters. |

`empty` has no ground plane, so a simulated rover falls under gravity. Use `road`
or `roam_test` for initial driving. Disabled native sensors keep their physical
housings but stop generating measurements. Dependency checks still apply:
disabling GNSS also requires disabling its velocity adapter.

### Additional run.py parameters

| Parameter | Meaning |
| --- | --- |
| `--control` | Start keyboard control; off by default and suppressed in headless mode. |
| `--no-view` | Do not open RViz. Gazebo GUI remains. |
| `--headless` | No Gazebo GUI, RViz or keyboard window. Sensor rendering still needs a working graphics environment. |
| `--no-ros` | Disable ROS sensor bridges, TF and adapters. Native Gazebo sensors still run. With `--control`, retain the command bridge. The runner still uses ROS launch. |
| `--render-engine ogre` or `ogre2` | GUI renderer override; automatic VM detection prefers `ogre`. GPU sensors still use Ogre 2. |
| `--output DIRECTORY` | Keep the generated bundle. Directory must be empty or nonexistent. Otherwise a temporary bundle is used and removed after shutdown. |
| `--assemble-only` | Generate without launching; requires `--output`. |

Configurations: `atlas.json` is the current five-camera setup. `legacy.json`
preserves the original single-camera/generic-LiDAR setup. `camera.json`,
`lidar.json` and `localization.json` select subsets of that **original** setup;
`camera.json` enables one camera, not ATLAS's five. All use the same runtime.

## 4. Inspect an assembly without launching

```bash
python3 scripts/assemble.py --world road --output build/inspect-road
python3 scripts/validate.py --assembly build/inspect-road
```

`assemble.py` requires `--output DIRECTORY`. Assembly/structural validation do
not need ROS or Gazebo. The bundle contains generated world/model SDF, resolved
configuration, bridges, transforms, RViz configuration and copied assets/adapters.
Use a fresh output name on subsequent assemblies. Regenerate after moving a
bundle because its mesh paths are absolute.

To launch that exact bundle from a sourced terminal:

```bash
ros2 launch my_robot simulation.launch.py assembly_dir:="$PWD/build/inspect-road" control:=true
```

ROS launch parameters use `name:=value`:

| Parameter | Default / purpose |
| --- | --- |
| `assembly_dir` | Required bundle directory; prefer an absolute path. |
| `gazebo` | `true`; `false` attaches ROS processes to an already running matching bundle. |
| `ros` | `true`; enable sensor bridge, TF and adapters. |
| `control` | `false`; keyboard control. |
| `view` | `true`; RViz when ROS is enabled and not headless. |
| `headless` | `false`; suppress windows. |
| `renderer` | `ogre2`; GUI rendering engine. |
| `log_readings` | `false`; log custom wheel encoder readings when that adapter is enabled. |

Do not attach a second ROS launch while the first already publishes the same
topics/TF. The default `GZ_PARTITION` is `roam_rover_sensors`; if you override it,
use the same value in every participating terminal.

## 5. Checks and recording

Quick checks without starting the simulator:

```bash
python3 src/my_robot/test/test_registry.py
python3 scripts/validate.py --world road
```

Start a run with a retained bundle:

```bash
python3 scripts/run.py --world road --control --output build/manual-road
```

In a second **sourced** terminal:

```bash
ros2 topic list
ros2 topic hz /rover/imu/data
# Ctrl+C stops topic hz; then run:
python3 scripts/validate.py --assembly build/manual-road --live --duration 20
bash scripts/record_sensors.sh build/manual-road my-sensor-bag
```

`validate.py --assembly DIRECTORY` inspects an existing bundle; omit it to check
the selected calibration/world. `--live` additionally checks running ROS outputs;
`--duration SECONDS` defaults to 20. Prefer the running bundle so overrides match.
This checks basic data, types, timestamps, frames and rates, not hardware fidelity.

`validate_sensors.py` can also run directly: it accepts `--assembly DIRECTORY`,
or `--rover NAME --calibration PATH` plus repeated `--enable/--disable ID`.
It accepts `--duration SECONDS` (20), `--min-rate-ratio FRACTION` (0.5; greater
than 0 and at most 1), and `--print-topics` (no ROS needed). It does not take
`--world` or `--spawn-pose`.

Recording syntax: `bash scripts/record_sensors.sh ASSEMBLY_DIRECTORY [BAG_OUTPUT]`.
The optional output defaults to a timestamped name; choose a nonexistent bag
directory. Ctrl+C stops recording. The generated topic list includes `/clock`,
`/tf`, `/tf_static` and declared sensor contracts, but currently omits the custom
`/rover/wheel_encoders` message and ground-truth odometry. Record those explicitly
with `ros2 bag record` when needed.

For ROS-dependent automated tests after building:

```bash
colcon test --build-base build-clean --install-base install-clean --packages-select my_robot
colcon test-result --test-result-base build-clean --verbose
```

If launch cannot find `my_robot`, source the matching workspace setup file.
If a bundle directory is nonempty, choose a new output directory. If Gazebo or
RViz crashes in VMware, save the terminal error; dependency checks alone do not
establish that the VM's graphics configuration can render the sensors.
