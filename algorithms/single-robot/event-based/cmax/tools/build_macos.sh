#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")/.."
if [[ "$(uname -s)" != Darwin || "$(uname -m)" != arm64 ]]; then
  echo 'This build entry point targets native macOS arm64.' >&2
  exit 1
fi
ros_prefix=/opt/homebrew/opt/ros2-jazzy
if [[ -f "$ros_prefix/setup.bash" ]]; then
  source "$ros_prefix/setup.bash"
elif [[ -f "$ros_prefix/local_setup.bash" ]]; then
  source "$ros_prefix/local_setup.bash"
elif [[ "${ROS_DISTRO:-}" != jazzy ]]; then
  echo 'Source your working Homebrew Jazzy environment first; no setup.bash was found at the supplied prefix.' >&2
  exit 1
fi
set -u
: "${ROS_PYTHON:=$(command -v python3)}"
"$ROS_PYTHON" tools/preflight.py
# Invoke colcon using the same Python interpreter checked for ROS2 extension compatibility.
"$ROS_PYTHON" -c 'from colcon_core.command import main; raise SystemExit(main())' build \
  --packages-select cmax_slam_ros2 --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64 \
  "-DCMAKE_PREFIX_PATH=$ros_prefix;/opt/homebrew" "-DPython3_EXECUTABLE=$ROS_PYTHON" "$@"
echo 'Build completed. In your zsh terminal: source install/setup.zsh'
