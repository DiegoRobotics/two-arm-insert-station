#!/usr/bin/env bash
# Copy project packages from OneDrive (source of truth, in git) into the WSL workspace, build, and test.
# Run inside Ubuntu:  bash "/mnt/c/Users/gael1/OneDrive/Documents/AR4 MK3 Engiworks Gael/AR4-ToolChanger/setup/build.sh"
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS=~/ar4tc_ws
mkdir -p $WS/src/ar4tc
rsync -a --delete --exclude __pycache__ "$REPO/ros_ws/src/" $WS/src/ar4tc/
source /opt/ros/jazzy/setup.bash
cd $WS
colcon build --symlink-install --packages-select ar4tc_toolchanger
source install/setup.bash
colcon test --packages-select ar4tc_toolchanger --pytest-args -q
colcon test-result --verbose --all | tail -5
