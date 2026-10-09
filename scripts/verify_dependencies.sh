#!/usr/bin/env bash
# ROAM simulation: Ubuntu 24.04 dependency verification (no system changes).
set -uo pipefail
pass=0; fail=0; warn=0
ok(){ printf '[PASS] %s\n' "$1"; ((pass+=1)); }
bad(){ printf '[FAIL] %s\n' "$1"; ((fail+=1)); }
notice(){ printf '[WARN] %s\n' "$1"; ((warn+=1)); }
section(){ printf '\n== %s ==\n' "$1"; }
installed(){ dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -qx 'install ok installed'; }
package(){ if installed "$1"; then ok "$1 installed"; else bad "$1 missing"; fi; }
command_check(){ if command -v "$1" >/dev/null 2>&1; then ok "$1 available"; else bad "$1 missing"; fi; }

section 'Platform'
if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  printf 'Detected: %s (%s)\n' "${PRETTY_NAME:-unknown}" "$(uname -m)"
  [[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 24.04 ]] && ok 'Ubuntu 24.04' || bad 'Ubuntu 24.04 required'
else
  bad 'Linux /etc/os-release missing'
fi
case "$(uname -m)" in x86_64|aarch64) ok '64-bit CPU architecture';; *) bad 'Unsupported CPU architecture';; esac
if ! command -v dpkg-query >/dev/null 2>&1; then
  bad 'dpkg-query unavailable: run on Ubuntu 24.04'; exit 1
fi

section 'C/C++ build toolchain'
for tool in gcc g++ cmake make git python3; do command_check "$tool"; done
if command -v g++ >/dev/null 2>&1; then
  gcc_major=$(g++ -dumpfullversion -dumpversion 2>/dev/null | cut -d. -f1)
  if [[ "$gcc_major" =~ ^[0-9]+$ ]] && (( gcc_major >= 13 )); then ok "G++ $gcc_major >= 13"; else bad 'G++ 13+ required'; fi
  if printf '#if __cplusplus < 202002L\n#error C++20 unavailable\n#endif\nint main() {return 0;}\n' | g++ -std=c++20 -x c++ -fsyntax-only - >/dev/null 2>&1; then
    ok 'C++20 supported (syntax-only test)'
  else bad 'C++20 compilation check failed'; fi
fi
if command -v cmake >/dev/null 2>&1; then
  cmake_ver=$(cmake --version 2>/dev/null | head -n1 | awk '{print $3}')
  if [[ "${cmake_ver:-}" =~ ^([0-9]+)\.([0-9]+) ]]; then
    major=${BASH_REMATCH[1]}; minor=${BASH_REMATCH[2]}
    if (( major > 3 || (major == 3 && minor >= 22) )); then ok "CMake $cmake_ver >= 3.22"; else bad "CMake $cmake_ver is older than 3.22"; fi
  else bad 'Cannot detect CMake version'; fi
fi

section 'ROS 2 Jazzy'
package ros-jazzy-ros-base
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  ok 'ROS setup file exists'
  if bash -c 'source /opt/ros/jazzy/setup.bash && command -v ros2 >/dev/null && ros2 pkg prefix rclcpp >/dev/null 2>&1'; then
    ok 'ROS 2 CLI and rclcpp are discoverable'
  else bad 'ROS 2 CLI/rclcpp not working'; fi
else bad '/opt/ros/jazzy/setup.bash missing'; fi
package ros-jazzy-rviz2

section 'Gazebo Harmonic'
if command -v gz >/dev/null 2>&1; then
  gz_output=$(gz sim --version 2>&1 || true)
  printf 'Gazebo reports: %s\n' "$gz_output"
  if grep -Eq 'Gazebo Sim(, version| version)? 8([.]|$)' <<< "$gz_output"; then
    ok 'Gazebo Sim 8 (Harmonic)'
  else bad 'Gazebo Harmonic (Sim 8) not verified'; fi
else bad 'gz command missing'; fi

section 'ROS-Gazebo bridge and robot control'
package ros-jazzy-ros-gz
package ros-jazzy-ros2-control
package ros-jazzy-ros2-controllers
package ros-jazzy-gz-ros2-control
if [[ -f /opt/ros/jazzy/setup.bash ]] && bash -c 'source /opt/ros/jazzy/setup.bash && ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 && ros2 pkg prefix ros_gz_sim >/dev/null 2>&1'; then
  ok 'ros_gz_bridge and ros_gz_sim discoverable'
else bad 'ROS-Gazebo bridge packages not discoverable'; fi

section 'ROS development utilities'
for tool in colcon rosdep; do command_check "$tool"; done
if command -v rosdep >/dev/null 2>&1; then
  if [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then ok 'rosdep initialized'; else notice 'rosdep is not initialized (sudo rosdep init)'; fi
fi

section 'Summary'
printf 'PASS: %d  FAIL: %d  WARN: %d\n' "$pass" "$fail" "$warn"
printf 'Package checks only: no GUI, live ROS topics, or sensor simulation tested.\n'
(( fail == 0 ))
