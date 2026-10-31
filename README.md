# ROAM Simulation environment

Supported: **Ubuntu 24.04 LTS**, x86_64 or aarch64. Standard stack: **ROS 2 Jazzy**, **Gazebo Harmonic**, **ROS–Gazebo bridge**, **GCC/G++ 13+**, **CMake 3.22+**, **C++20**, RViz2, colcon, rosdep, ros2_control.

## Rover with all sensors

`sensors-integrated` combines the sensor branches with the updated rover and
worlds from `main`. The RGB camera, 3D LiDAR, IMU, and GNSS antenna are fixed to
the ROAM rover with mass and collision geometry. The wheel encoder nodes read
this rover's four wheel joints.

Build once from the repository root, and rebuild after changing the ROS package,
sensor mounts, or viewer configuration:

```bash
source /opt/ros/jazzy/setup.bash  # setup.zsh in a zsh terminal
colcon build --packages-up-to my_robot
bash scripts/launch_roam.sh road
```

The launcher loads ROS and the workspace, starts Gazebo, bridges all sensors,
opens the keyboard control window, and opens RViz with the front camera image,
3D LiDAR cloud, sensor frames, and synthetic INS poses. `maze`, `forest`, and
`roam_test` are also supported. `bash scripts/run_rover.sh` selects `roam_test`.
Use W/S for the left wheels, Up/Down for the right wheels, and Space to brake.
Close the old simulation before starting another one. Ctrl+C in the launch
terminal stops Gazebo, the sensor nodes, RViz, and the keyboard controller.

```bash
bash scripts/launch_roam.sh maze --no-view      # sensors without RViz
bash scripts/launch_roam.sh forest --headless  # server + sensors, no windows
bash scripts/launch_roam.sh road --no-sensors  # Gazebo + controls, no ROS sensors
```

The GUI renderer is selected automatically; override it with `--render-engine
ogre2` or `--render-engine ogre`. Camera and GPU LiDAR rendering always use
Ogre 2. `--headless` disables windows but still needs working sensor rendering
(for example, an available X display).

### Sensor topics

In another terminal, source `/opt/ros/jazzy/setup.bash` and `install/setup.bash`
(use `.zsh` files in zsh), then inspect the following ROS topics:

| Sensor / output | ROS topic | Message type | Configured rate in simulation time |
| --- | --- | --- | --- |
| Front RGB camera, 640 × 480 | `/cameras/surround_front/image` | `sensor_msgs/msg/Image` | 30 Hz |
| Camera calibration | `/cameras/surround_front/camera_info` | `sensor_msgs/msg/CameraInfo` | 30 Hz |
| 360° LiDAR, 1800 × 32 rays | `/lidar_points` | `sensor_msgs/msg/PointCloud2` | 10 Hz |
| IMU | `/imu` | `sensor_msgs/msg/Imu` | 100 Hz |
| GNSS | `/gnss/fix` | `sensor_msgs/msg/NavSatFix` | 20 Hz |
| Synthetic INS | `/ins/odom` | `nav_msgs/msg/Odometry` | 100 Hz |
| Four wheel encoders | `/rover/wheel_encoders` | `my_robot_msgs/msg/WheelEncoders` | Up to 50 Hz |
| Wheel odometry | `/rover/odom` | `nav_msgs/msg/Odometry` | 50 Hz |
| Ground truth | `/rover/ground_truth/odometry` | `nav_msgs/msg/Odometry` | 100 Hz |

```bash
ros2 topic echo /imu --once --qos-reliability best_effort
ros2 topic echo /gnss/fix --once --qos-reliability best_effort
ros2 topic echo /rover/wheel_encoders --once
ros2 topic hz /lidar_points
```

Camera and LiDAR delivery can fall below their configured rates when rendering
or transport cannot keep up. Simulation-time rates also differ from wall-clock
rates when the simulator runs slower than real time.

GNSS uses Gazebo's native NavSat sensor with a WGS84 origin at latitude
51.0447°, longitude -114.0719°, elevation 1045 m, in an east/north/up world.
Horizontal noise is 0.00000018 degrees (roughly 1–2 cm here), following the
[Gazebo Sensors 8 angular-noise convention](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/NavSatSensor.cc).
Altitude noise is 4 cm. The native ROS bridge leaves GNSS covariance unknown.
The IMU has Gaussian angular-velocity and acceleration noise. LiDAR uses a
generic 360° scan based on the branch settings; it does not reproduce Livox's
scanning pattern.

The INS output preserves the original testbed's approach: simulated 3D ground
truth plus 5 mm position noise. Orientation and velocity come from truth.
This is a synthetic test signal, not an inertial estimator or GNSS/IMU fusion.
The `world -> base_link` TF also follows truth so the sensor view stays aligned;
wheel odometry remains a separate measurement.

Sensor frames are read from `models/roam_rover/model.sdf`, including each
sensor's offset from its housing and the camera optical-frame rotation. Bridge
configuration and the RViz layout are in `src/my_robot/config/`. All sensor
processes use `GZ_PARTITION=roam_rover_sensors` by default and honor a custom
value. For a manually started viewer and bridge against this simulation:

```bash
export GZ_PARTITION=roam_rover_sensors
ros2 launch my_robot rover_sensors.launch.py
```

Use that command only when the main launcher was started with `--no-sensors`
to avoid duplicate bridges and publishers.

The original standalone camera, GNSS/INS cube testbed, basic wheel rover, and
[LiDAR setup notes](<LiDAR Informatioin.md>) are preserved as references.
Build outputs, logs, Python caches, and temporary worlds are ignored by Git.

## Before installing

Run from the repository root:

```bash
bash scripts/verify_dependencies.sh
```

PASS = installed and detectable. FAIL = required component missing / not verified. WARN = non-blocking concern. The script makes no system changes. Nonzero exit means at least one required check failed.

## Install missing dependencies

Review the installer before running: it adds the official ROS 2 and Gazebo apt repositories when needed, installs missing packages, and initializes / updates rosdep. Requires **sudo**, internet access, and may take disk space.

```bash
bash scripts/install_dependencies.sh
```

Then verify:

```bash
bash scripts/verify_dependencies.sh
source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep ros_gz
```

## Scope and limitations

- Installer skips already-installed apt packages; it does not attempt to repair broken existing installations or upgrade old compiler versions.
- ROS 2 itself is not required to be built as C++20; C++20 is a requirement for ROAM-owned C++ packages. For those packages set `CMAKE_CXX_STANDARD 20`, `CMAKE_CXX_STANDARD_REQUIRED ON`, and `CMAKE_CXX_EXTENSIONS OFF`.
- Verification checks package discovery and C++20 syntax support, not GUI rendering, simulation of physical sensors, message transfer over live topics, GPU acceleration, or functional hardware drivers.
- Before team-wide rollout, test on a fresh Ubuntu 24.04 VM and a machine with packages already installed. Review any apt conflicts before proceeding.
- On Macs, run these scripts **inside Ubuntu 24.04**, not the macOS host.

## Official installation references

- ROS 2 Jazzy: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- Gazebo Harmonic: https://gazebosim.org/docs/harmonic/install_ubuntu/

## Standalone basic wheel rover

The four-wheel skid-steer rover, timed drive helper, wheel encoder nodes and
custom encoder message are integrated into this colcon workspace. They use the club's
existing worlds; the launcher adds the rover and required Gazebo systems to a
temporary SDF. No build is needed to run the Gazebo model.

```bash
cd ~/rover-simulation
./scripts/run_wheel_rover.sh                       # road world
./scripts/run_wheel_rover.sh --world maze
./scripts/run_wheel_rover.sh --world forest -s      # server only
```

Choose one simulation at a time. `road` is the default; each world has a spawn
pose above its terrain and away from nearby walls. `empty.sdf` is a visual-only
scene without a collision ground and is not supported by this launcher.

In another terminal:

```bash
cd ~/rover-simulation
./scripts/drive_rover.sh 0.3 0 3     # forward: linear m/s, angular rad/s, duration s
./scripts/drive_rover.sh 0 0.5 2     # turn left
./scripts/drive_rover.sh -0.3 0 2    # reverse
./scripts/drive_rover.sh 0 0 0       # stop
```

The helper uses the installed ROS-Gazebo bridge and a persistent publisher,
then stops the rover on completion or Ctrl+C. It needs no workspace build.
Run one drive helper at a time. Limits are +/-1 m/s, +/-1.5 rad/s and 0..60
seconds. Duration is wall time after connection setup; a zero duration only
sends stop commands. Stop before pausing Gazebo. Direct `gz topic` velocity
commands remain active until an explicit stop.

Both scripts default to `GZ_PARTITION=roam_rover_wheels`, separate from the old
workspace. If overriding it, use the same value in every simulation, drive,
inspection and bridge terminal.

### Wheel measurements

```bash
source /opt/ros/jazzy/setup.bash  # use setup.zsh in zsh
export GZ_PARTITION=roam_rover_wheels
gz topic -e -t /rover/wheel_joint_states
gz topic -e -t /rover/odometry
```

| Gazebo topic | Type | Purpose |
| --- | --- | --- |
| `/rover/cmd_vel` | `gz.msgs.Twist` | Linear x / angular z commands |
| `/rover/wheel_joint_states` | `gz.msgs.Model` | Joint angles and angular velocities |
| `/rover/odometry` | `gz.msgs.Odometry` | Wheel-derived odometry |
| `/rover/tf` | `gz.msgs.Pose_V` | Odometry-to-chassis transform |

These are Gazebo Transport topics. Wheel angles are radians and velocities are
rad/s. Wheel-derived odometry can drift during skid steering.

### ROS encoder packages

```bash
cd ~/rover-simulation
source /opt/ros/jazzy/setup.bash  # use setup.zsh in zsh
colcon build --packages-up-to my_robot
source install/setup.bash       # use setup.zsh in zsh
ros2 interface show my_robot_msgs/msg/WheelEncoders
ros2 pkg executables my_robot
```

`my_robot_msgs/WheelEncoders` contains a header and four joint names, angles and
angular velocities, ordered front-left, front-right, rear-left, rear-right.
`my_robot` provides `wheel_encoder_publisher` and `wheel_encoder_subscriber`.
The publisher validates and reorders joint measurements; the subscriber logs
encoder readings once per second.

The simulator launcher runs Gazebo only. In a second terminal, source Jazzy and
this workspace's `install/setup.bash` (or `setup.zsh` in zsh), then start the
encoder pipeline:

```bash
ros2 launch my_robot wheel_encoders.launch.py
```

This launch file uses the same default `GZ_PARTITION=roam_rover_wheels` as the
simulator, bridges joint states and `/clock` from Gazebo to ROS, and runs both
encoder nodes with simulation time. It preserves a custom `GZ_PARTITION` if set.
Use `log_readings:=false` to disable the logging subscriber. Encoder timestamps
come from the simulated joint measurements; these are simulated joint angles
and velocities, without hardware tick quantization or sensor noise.

To check the output in another sourced terminal:

```bash
ros2 topic echo /rover/wheel_encoders --once
```

For manual debugging, the equivalent joint-state bridge is:

```bash
export GZ_PARTITION=roam_rover_wheels
ros2 run ros_gz_bridge parameter_bridge \
  '/rover/wheel_joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
```

Then run one node per sourced terminal. Use either the launch file or these
manual commands to avoid duplicate publishers:

```bash
ros2 run my_robot wheel_encoder_publisher
ros2 run my_robot wheel_encoder_subscriber
```

Run the encoder regression checks after building:

```bash
colcon test --packages-select my_robot
colcon test-result --verbose
```

### Model and source files

- `models/basic_rover/`: 8 kg rover, 6 kg chassis and four 0.5 kg wheels.
- `scripts/run_wheel_rover.sh`: rover launcher using the existing club worlds.
- `scripts/drive_rover.sh` and `scripts/drive_rover.py`: bounded, timed commands
  through a temporary ROS-Gazebo bridge, with stop on exit.
- `src/my_robot/`: encoder publisher and subscriber.
- `src/my_robot/launch/wheel_encoders.launch.py`: ROS bridge and encoder startup.
- `src/my_robot_msgs/`: fixed-size custom wheel encoder interface.

Chassis dimensions are 0.80 x 0.50 x 0.16 m. Wheels have radius 0.14 m and
width 0.07 m; separation is 0.58 m and wheelbase is 0.56 m. Front is +x, left
is +y, up is +z; the green strip marks the front. Both wheels on each side
share drive commands. Linear acceleration is limited to +/-1 m/s^2 and angular
acceleration to +/-2 rad/s^2.

Wheel joints use explicit finite +/-1e16 rad limits so DART can drive them;
these bounds allow effectively continuous rotation.

Joint names are `front_left_wheel_joint`, `front_right_wheel_joint`,
`rear_left_wheel_joint` and `rear_right_wheel_joint`.
