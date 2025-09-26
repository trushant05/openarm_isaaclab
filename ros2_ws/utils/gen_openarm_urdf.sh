#!/bin/bash

# Exit if command fails
set -e

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Build openarm_description package
colcon build --symlink-install 

# Source overlay workspace
source install/setup.bash

# Extact URDF from xacro
ros2 run xacro xacro src/openarm_description/urdf/robot/v10.urdf.xacro bimanual:=true -o ../URDF/openarm.urdf
