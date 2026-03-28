#!/bin/bash

set -e

MODEL="${1:-elfin5}"
WORKSPACE_DIR="/home/xpm/workspace/ros2_ws"
export ROS_LOG_DIR=/tmp/ros_logs

pkill -f "ros2 launch /home/xpm/workspace/ros2_ws/elfin_visualize.launch.py" || true
pkill -f "python3 /home/xpm/workspace/ros2_ws/control_elfin.py" || true
pkill -f "robot_state_publisher" || true
pkill -f "joint_state_publisher" || true
pkill -f "joint_state_publisher_gui" || true
pkill -f rviz2 || true
sleep 1

cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
source install/setup.bash

exec ros2 launch /home/xpm/workspace/ros2_ws/elfin_visualize.launch.py "model:=${MODEL}"
