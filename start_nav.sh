#!/bin/bash

cd /root/ros_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install
exec ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True "$@"