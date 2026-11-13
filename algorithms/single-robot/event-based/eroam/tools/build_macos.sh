#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/../../.."
set +u
source "$HOME/robotics2/robotics_env.zsh"
set -u
export CMAKE_PREFIX_PATH="/opt/homebrew:${CMAKE_PREFIX_PATH:-}"
colcon build --symlink-install --packages-select eroam_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
