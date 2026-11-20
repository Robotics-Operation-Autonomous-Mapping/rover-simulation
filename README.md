# ROAM rover simulation

Gazebo Harmonic / ROS 2 Jazzy simulation assembled from reusable rover, sensor,
and world components. `calibration/atlas.json` selects sensor instances, mounts,
settings, topics and frames. Settings are provisional, not measured hardware calibration.

See [Architecture and sensor fidelity](docs/ARCHITECTURE.md) for editable diagrams,
source responsibilities, sensor contracts and the fidelity roadmap.
See [Cleanup and migration](docs/MIGRATION.md) for retired entry points.

## Setup and run

On Ubuntu 24.04, inspect dependencies with `bash scripts/setup/verify_dependencies.sh`.
If necessary, review and run `bash scripts/setup/install_dependencies.sh` (sudo/network).

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to my_robot
source install/setup.bash
python3 scripts/run.py --world forest --control
```

Worlds: `road`, `forest`, `maze`, `roam_test`, `camera_test`, `empty`.
The last is a minimal scene without a supporting ground plane; the rover falls
under gravity. It is useful for assembly inspection, not driving tests.

`--headless` suppresses windows; sensor rendering still requires an appropriate
graphics environment. `--no-view` disables RViz. `--no-ros` disables sensor
bridges, TF and adapters; `--control` can retain the command bridge. Native Gazebo
sensors remain active. Omit `--control` to omit the keyboard controller.
`--render-engine ogre|ogre2` selects the GUI renderer; GPU sensors use Ogre 2.
Ctrl+C shuts down launch and keeps temporary assets alive until it exits.

Keyboard: W/S for left wheels, Up/Down for right wheels, Space to brake.
Bounded timed-drive tools remain in `scripts/control/`. The default Gazebo
partition is `roam_rover_sensors`; custom `GZ_PARTITION` must match across terminals.

## Assemble and inspect independently

```bash
python3 scripts/assemble.py --rover atlas --world forest --calibration calibration/atlas.json --output build/atlas-inspect
python3 scripts/validate.py --assembly build/atlas-inspect
# After sourcing ROS and the workspace:
ros2 launch my_robot simulation.launch.py assembly_dir:="$PWD/build/atlas-inspect"
```

Output directories must be empty. Generated mesh paths are absolute: regenerate
when relocating a bundle. JSON calibration needs only Python; YAML also needs
PyYAML. Assembly does not require ROS or Gazebo.

Select configurations with `--calibration calibration/NAME.json`:

| File | Selection |
| --- | --- |
| `atlas.json` (default) | Five cameras, JT128-labelled GPU LiDAR, two IMUs, magnetometer, GNSS position/velocity, wheel measurements |
| `legacy.json` | Original single-camera, generic LiDAR, IMU, GNSS, encoders, synthetic INS contract |
| `camera.json`, `lidar.json`, `localization.json` | Original focused selections, expressed as ordinary enabled flags |

`legacy.json` is retained sensor data, not an alternate execution path.
Use `--enable INSTANCE` / `--disable INSTANCE` repeatedly for overrides. Disable
both `gnss` and `gnss_velocity` when removing that dependency.

## Lightweight verification and recording

```bash
python3 src/my_robot/test/test_registry.py
python3 scripts/validate.py --world road
# On a running, sourced ROS workspace:
python3 scripts/validate.py --assembly build/atlas-inspect --live
bash scripts/record_sensors.sh build/atlas-inspect my-sensor-bag
colcon test --packages-select my_robot
colcon test-result --verbose
```

Portable tests cover registry validation, original contract preservation,
supplied worlds, installed assets, adapter declarations and bundle consistency.
ROS-dependent tests exercise TF conversion, synthetic INS and encoder callbacks.
Structural checks do not establish live Gazebo validity, delivered frequencies,
or equivalence to physical sensors.

For ROS-only attachment, start the main bundle with `ros:=false`, then launch
the same bundle with `gazebo:=false`. Avoid duplicate bridges/controllers.
`--output build/run` on `scripts/run.py` retains a bundle for inspection/recording.
