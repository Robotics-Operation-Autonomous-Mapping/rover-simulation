#!/usr/bin/env bash
# Installs missing ROAM simulation dependencies on Ubuntu 24.04 (amd64 or arm64).
# Run as a regular user with sudo privileges; rerunning is supported.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -r /etc/os-release ]]; then echo 'ERROR: Ubuntu 24.04 required.' >&2; exit 1; fi
. /etc/os-release
if [[ "${ID:-}" != ubuntu || "${VERSION_ID:-}" != 24.04 ]]; then
  echo 'ERROR: Installer supports Ubuntu 24.04 only.' >&2; exit 1
fi
case "$(uname -m)" in x86_64|aarch64) ;; *) echo 'ERROR: Expected 64-bit x86 or ARM.' >&2; exit 1;; esac
if (( EUID == 0 )); then echo 'ERROR: Run as your normal user (sudo will be requested).' >&2; exit 1; fi
for tool in sudo apt-get dpkg-query curl; do
  if [[ "$tool" == curl ]]; then continue; fi
  command -v "$tool" >/dev/null || { echo "Missing required command: $tool" >&2; exit 1; }
done

installed(){ dpkg-query -W -f='${Status}' "$1" 2>/dev/null | grep -qx 'install ok installed'; }
install_missing(){
  local missing=() pkg
  for pkg in "$@"; do if installed "$pkg"; then printf '[SKIP] %s\n' "$pkg"; else missing+=("$pkg"); fi; done
  if ((${#missing[@]})); then sudo apt-get install -y "${missing[@]}"; fi
}

printf 'ROAM Ubuntu 24.04 installer: apt packages and repository configuration may change.\n'
printf 'Requires internet and sudo privileges. No full system upgrade will be run.\n'
sudo -v
sudo apt-get update
install_missing ca-certificates curl gnupg software-properties-common lsb-release

# ROS 2 official apt repository (ros2-apt-source manages its keys and configuration).
if ! installed ros2-apt-source; then
  echo 'Configuring official ROS 2 apt repository...'
  sudo add-apt-repository -y universe
  tmpdir=$(mktemp -d)
  trap 'rm -rf "$tmpdir"' EXIT
  release=$(curl -fsSL --retry 3 'https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest' |
    python3 -c 'import sys,json; print(json.load(sys.stdin)["tag_name"])')
  [[ "$release" =~ ^[0-9]+([.][0-9]+)*$ ]] || { echo "Unexpected ROS repo release tag: $release" >&2; exit 1; }
  curl -fL --retry 3 -o "$tmpdir/ros2-apt-source.deb" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${release}/ros2-apt-source_${release}.noble_all.deb"
  sudo apt-get install -y "$tmpdir/ros2-apt-source.deb"
  sudo apt-get update
else
  echo '[SKIP] ROS 2 apt repository package installed'
fi

# OSRF provides the full Gazebo Harmonic metapackage.
if ! installed gz-harmonic; then
  if ! grep -Rq 'packages.osrfoundation.org/gazebo/ubuntu-stable' /etc/apt/sources.list.d/ 2>/dev/null; then
    echo 'Configuring official Gazebo repository...'
    curl -fsSL --retry 3 https://packages.osrfoundation.org/gazebo.gpg |
      sudo tee /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg >/dev/null
    arch=$(dpkg --print-architecture)
    printf 'deb [arch=%s signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main\n' "$arch" |
      sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
    sudo apt-get update
  fi
fi

# GCC/G++ 13 default on Ubuntu 24.04; configure ROS and Gazebo together.
install_missing build-essential cmake git python3 python3-pip python3-venv \
  ros-dev-tools ros-jazzy-desktop ros-jazzy-ros-gz gz-harmonic \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control

if command -v rosdep >/dev/null 2>&1; then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    echo 'Initializing rosdep...'
    sudo rosdep init
  fi
  echo 'Updating rosdep dependency index...'
  rosdep update || echo 'WARNING: rosdep update failed; retry when network is available.' >&2
fi

printf '\nInstallation steps complete. Running verification...\n'
if [[ -f "$SCRIPT_DIR/verify_dependencies.sh" ]]; then
  bash "$SCRIPT_DIR/verify_dependencies.sh"
else
  echo 'WARNING: verify_dependencies.sh not found next to installer.'
fi
printf '\nTo use ROS 2 in a new terminal: source /opt/ros/jazzy/setup.bash\n'
