#!/bin/bash
# Unified launcher for SLAM mapping session
# Launches both ogre-slam (mapping) and ogre_teleop (manual control)

# Configuration
RPLIDAR_MODEL="${1:-a1}"  # Default to A1, can override: ./launch_mapping_session.sh a2m7

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Source ROS environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "🚀 Project Ogre SLAM Mapping Session"
echo "========================================"
echo ""

# Check if ogre_teleop package exists
if ! ros2 pkg list | grep -q "ogre_teleop"; then
    echo -e "${RED}❌ Error: ogre_teleop package not found!${NC}"
    echo "Please build ogre_teleop package first:"
    echo "  cd ~/ros2_ws && colcon build --packages-select ogre_teleop"
    exit 1
fi

# 1. Launch SLAM mapping system
echo -e "${GREEN}📊 [1/2] Launching SLAM mapping system...${NC}"
echo "  - RPLIDAR model: $RPLIDAR_MODEL"
echo "  - Odometry node (wheel encoders)"
echo "  - robot_localization EKF"
echo "  - slam_toolbox (async mapping)"
echo "  - RViz visualization"
echo ""

ros2 launch ogre_slam mapping.launch.py rplidar_model:=$RPLIDAR_MODEL > /tmp/ogre_slam_mapping.log 2>&1 &
SLAM_PID=$!

# Wait for SLAM system to initialize
echo -e "${YELLOW}⏳ Waiting for SLAM system to initialize...${NC}"
sleep 5

# Check if SLAM is still running
if ! kill -0 $SLAM_PID 2>/dev/null; then
    echo -e "${RED}❌ SLAM system failed to start! Check logs:${NC}"
    echo "  tail -f /tmp/ogre_slam_mapping.log"
    exit 1
fi

echo -e "${GREEN}   ✅ SLAM system ready (PID: $SLAM_PID)${NC}"
echo ""

# 2. Launch ogre_teleop for manual control
echo -e "${GREEN}🎮 [2/2] Launching ogre_teleop (manual control)...${NC}"
echo "  - Motor control node"
echo "  - Camera node"
echo "  - Web interface on port 8080"
echo ""

ros2 launch ogre_teleop web_teleop.launch.py > /tmp/ogre_teleop.log 2>&1 &
TELEOP_PID=$!

# Wait for teleop to initialize
sleep 3

# Check if teleop is still running
if ! kill -0 $TELEOP_PID 2>/dev/null; then
    echo -e "${RED}❌ Teleop failed to start! Check logs:${NC}"
    echo "  tail -f /tmp/ogre_teleop.log"
    kill $SLAM_PID 2>/dev/null
    exit 1
fi

echo -e "${GREEN}   ✅ Teleop ready (PID: $TELEOP_PID)${NC}"
echo ""

# Summary
echo "========================================"
echo -e "${GREEN}🎉 Mapping session ready!${NC}"
echo ""
echo "Active Processes:"
echo "  SLAM System: PID $SLAM_PID"
echo "  Teleop:      PID $TELEOP_PID"
echo ""
echo "Logs:"
echo "  tail -f /tmp/ogre_slam_mapping.log"
echo "  tail -f /tmp/ogre_teleop.log"
echo ""
echo -e "${YELLOW}🌐 Web Interface:${NC}"
echo "  http://10.21.21.45:8080"
echo ""
echo -e "${YELLOW}📋 Mapping Instructions:${NC}"
echo "  1. Open web interface in browser"
echo "  2. Drive robot slowly around the area"
echo "  3. Ensure good loop closures (revisit starting point)"
echo "  4. Watch RViz to see map being built"
echo "  5. When done, press Ctrl+C here to stop"
echo ""
echo -e "${YELLOW}💾 To save map when done:${NC}"
echo "  ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/ogre-slam/maps/my_map"
echo ""
echo "Press Ctrl+C to stop all systems..."
echo "========================================"

# Trap Ctrl+C for clean shutdown
trap 'echo ""; echo "🛑 Shutting down mapping session..."; \
      kill $SLAM_PID 2>/dev/null; \
      kill $TELEOP_PID 2>/dev/null; \
      sleep 2; \
      echo "✅ All systems stopped"; \
      exit 0' INT TERM

# Wait for processes
wait
