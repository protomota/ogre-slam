#!/bin/bash
# Launch RViz for Isaac Sim SLAM mapping
# Uses same config as real robot - just set ROS_DOMAIN_ID=42

# Set ROS domain to match Isaac Sim and real robot
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "============================================"
echo "Launching RViz for Isaac Sim SLAM Mapping"
echo "============================================"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "Expected topics:"
echo "  - /scan (LIDAR)"
echo "  - /odom (odometry)"
echo "  - /map (SLAM map)"
echo ""
echo "Make sure Isaac Sim is running with:"
echo "  - ROS2 Context Domain ID = 42"
echo "  - LIDAR publishing to /scan topic"
echo "  - Simulation playing (Press Play)"
echo "============================================"
echo ""

# Launch RViz with universal SLAM config
rviz2 -d ~/ros2_ws/src/ogre-slam/rviz/slam_universal.rviz
