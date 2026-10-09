# ROAM Simulation environment

Supported: **Ubuntu 24.04 LTS**, x86_64 or aarch64. Standard stack: **ROS 2 Jazzy**, **Gazebo Harmonic**, **ROS–Gazebo bridge**, **GCC/G++ 13+**, **CMake 3.22+**, **C++20**, RViz2, colcon, rosdep, ros2_control.

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