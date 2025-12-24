#!/bin/bash

echo "=== START UR5 SIMULATION ==="

# Source ROS + workspace
source /opt/ros/noetic/setup.bash
source ~/ur5_ws/devel/setup.bash

# Mở Gazebo UR5 (chạy nền)
echo "Launching Gazebo UR5..."
roslaunch ur_gazebo ur5_bringup.launch &

# Đợi Gazebo load xong
sleep 8

# Chạy pick and place node
echo "Launching Pick & Place node..."
roslaunch ur5_control run_pick_place.launch


roslaunch ur5_moveit_config demo.launch

