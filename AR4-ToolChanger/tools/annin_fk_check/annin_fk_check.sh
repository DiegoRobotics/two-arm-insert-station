#!/usr/bin/env bash
# Cross-check the ROS MK3 URDF against Annin HMI 6.0's own kinematics (kinematics.cpp).
# Compiles a COPY of Annin's source into ~/annin_kin; the Annin folder is only read.
# Needs: python3-pybind11 (sudo apt install python3-pybind11) and the built ar4tc workspace.
set -eo pipefail
ANNIN="/mnt/c/Users/gael1/OneDrive/Documents/AR4 MK3 Engiworks Gael/6.0/AR4-MK3-HMI-interface-6.0-exe/AR4-MK3 HMI interface 6.0 exe"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p ~/annin_kin
cp "$ANNIN/kinematics.cpp" "$ANNIN/bindings.cpp" ~/annin_kin/
cd ~/annin_kin
# Annin builds with MSVC; GCC needs -fpermissive for its template lookups.
g++ -O2 -std=c++14 -shared -fPIC -fpermissive -w $(python3 -m pybind11 --includes) bindings.cpp \
    -o robot_kinematics$(python3-config --extension-suffix)
source /opt/ros/jazzy/setup.bash
source ~/ar4tc_ws/install/setup.bash
xacro "$(ros2 pkg prefix annin_ar4_moveit_config)/share/annin_ar4_moveit_config/urdf/fake_ar.urdf.xacro" ar_model:=mk3 > /tmp/ar_mk3.urdf
PYTHONPATH=~/annin_kin:$PYTHONPATH python3 "$HERE/annin_fk_check.py" /tmp/ar_mk3.urdf
