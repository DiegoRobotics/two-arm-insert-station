#!/usr/bin/env bash
# Phase 1 setup, run INSIDE WSL Ubuntu 24.04:  bash setup_wsl_ros.sh
# Installs ROS 2 Jazzy + MoveIt 2 + Gazebo Harmonic and builds the pinned AR4 driver in simulation only.
# Doesn't touch the robot, the Teensy, or Annin's HMI software.
set -euo pipefail

AR4_DRIVER_COMMIT=4b88db1c924515f7ce343105d46a637ecf2d7659   # ycheng517/ar4_ros_driver main, 2026-04-04
WS=~/ar4tc_ws

# 1. ROS 2 Jazzy apt source
sudo apt-get update
sudo apt-get install -y software-properties-common curl git
sudo add-apt-repository -y universe
ROS_APT_VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | cut -d'"' -f4)
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VER}/ros2-apt-source_${ROS_APT_VER}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt-get update && sudo apt-get -y upgrade

# 2. ROS 2, MoveIt 2, Gazebo Harmonic, IK and kinematics tools
sudo apt-get install -y \
  ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-moveit ros-jazzy-moveit-py \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-pick-ik ros-jazzy-pinocchio \
  python3-pytest

# 3. Workspace with the upstream MK3 driver at a pinned commit
mkdir -p $WS/src && cd $WS/src
[ -d ar4_ros_driver ] || git clone https://github.com/ycheng517/ar4_ros_driver.git
git -C ar4_ros_driver checkout $AR4_DRIVER_COMMIT
cd $WS
set +u   # ROS setup scripts reference unset variables
source /opt/ros/jazzy/setup.bash
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

grep -q ar4tc_ws ~/.bashrc || cat >> ~/.bashrc <<'EOF'
source /opt/ros/jazzy/setup.bash
[ -f ~/ar4tc_ws/install/setup.bash ] && source ~/ar4tc_ws/install/setup.bash
EOF
echo "Done. Smoke test:  ros2 launch annin_ar4_moveit_config moveit.launch.py ar_model:=mk3 include_gripper:=True"
