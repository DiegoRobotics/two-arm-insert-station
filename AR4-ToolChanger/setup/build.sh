#!/usr/bin/env bash
# Copy project packages from this repo (source of truth, in git) into the WSL workspace, build, and test.
# Works on Jazzy (Ubuntu 24.04) and Humble (Ubuntu 22.04): the installed distro is detected.
# Run inside Ubuntu:  bash <repo>/AR4-ToolChanger/setup/build.sh
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS=~/ar4tc_ws
DISTRO=${ROS_DISTRO:-$(ls /opt/ros | grep -E '^(jazzy|humble)$' | head -1)}
[ -f /opt/ros/$DISTRO/setup.bash ] || { echo "No ROS 2 Jazzy or Humble found in /opt/ros"; exit 1; }
echo "Building for ROS 2 $DISTRO"
mkdir -p $WS/src/ar4tc
rsync -a --delete --exclude __pycache__ --exclude .pytest_cache "$REPO/ros_ws/src/" $WS/src/ar4tc/
source /opt/ros/$DISTRO/setup.bash
cd $WS
colcon build --symlink-install --packages-select ar4tc_toolchanger
source install/setup.bash
colcon test --packages-select ar4tc_toolchanger --pytest-args -q
colcon test-result --verbose --all | tail -5
