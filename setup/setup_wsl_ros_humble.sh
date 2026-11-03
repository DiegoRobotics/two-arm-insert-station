#!/usr/bin/env bash
# Humble variant of setup_wsl_ros.sh. Run INSIDE WSL Ubuntu 22.04:  bash setup_wsl_ros_humble.sh
# Installs ROS 2 Humble + MoveIt 2 + Gazebo (Ignition Fortress) and builds the upstream driver's
# `humble` branch at a pinned commit, simulation only. Leaves the robot, the Teensy and Annin's HMI alone.
# Same project packages as Jazzy: afterwards run setup/build.sh (it detects the distro).
set -euo pipefail

AR4_DRIVER_COMMIT=b9adb2a65be5d4bc7c7f5e25c421238528aeeae5   # ycheng517/ar4_ros_driver humble, 2025-06-08
WS=~/ar4tc_ws

. /etc/os-release
[ "$VERSION_CODENAME" = jammy ] || { echo "ROS 2 Humble binaries need Ubuntu 22.04 (jammy); this is $VERSION_CODENAME"; exit 1; }

# 1. ROS 2 apt source
sudo apt-get update
sudo apt-get install -y software-properties-common curl git rsync
sudo add-apt-repository -y universe
ROS_APT_VER=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | cut -d'"' -f4)
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_VER}/ros2-apt-source_${ROS_APT_VER}.${VERSION_CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt-get update && sudo apt-get -y upgrade

# 2. ROS 2, MoveIt 2, Gazebo Fortress (ros_gz + ign_ros2_control on Humble), IK and kinematics tools
sudo apt-get install -y \
  ros-humble-desktop ros-dev-tools \
  ros-humble-moveit \
  ros-humble-ros-gz ros-humble-ign-ros2-control \
  ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-pick-ik ros-humble-pinocchio \
  python3-pytest python3-serial python3-pybind11

# 3. Workspace with the upstream driver's humble branch at a pinned commit
mkdir -p $WS/src && cd $WS/src
[ -d ar4_ros_driver ] || git clone https://github.com/ycheng517/ar4_ros_driver.git
git -C ar4_ros_driver fetch -q origin humble
git -C ar4_ros_driver checkout $AR4_DRIVER_COMMIT
cd $WS
set +u   # ROS setup scripts reference unset variables
source /opt/ros/humble/setup.bash
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update --rosdistro humble
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install

grep -q ar4tc_ws ~/.bashrc || cat >> ~/.bashrc <<'EOF'
source /opt/ros/humble/setup.bash
[ -f ~/ar4tc_ws/install/setup.bash ] && source ~/ar4tc_ws/install/setup.bash
EOF
echo "Done. Smoke test:  ros2 launch annin_ar4_moveit_config demo.launch.py ar_model:=mk3"
echo "Then build our packages:  bash \"<repo>/setup/build.sh\""
