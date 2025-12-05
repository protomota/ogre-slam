# Quickstart

Quick reference for common commands. See README.md for full documentation.

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select ogre_slam ogre_policy_controller --symlink-install
source install/setup.bash
```

## SLAM Mapping (Real Robot)

```bash
# Launch full mapping session (SLAM + teleop)
cd ~/ros2_ws/src/ogre-slam
./scripts/launch_mapping_session.sh

# Save map after mapping
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/ogre-slam/maps/my_map
```

## Navigation (Real Robot)

```bash
ros2 launch ogre_slam navigation.launch.py map:=~/ros2_ws/src/ogre-slam/maps/my_map.yaml
```

## Isaac Sim Testing

```bash
# Terminal 1: Start Isaac Sim with ogre.usd, press Play

# Terminal 2: Launch RViz for visualization
export ROS_DOMAIN_ID=42
cd ~/ros2_ws/src/ogre-slam
./scripts/launch_isaac_sim_rviz.sh

# Terminal 3: Test with keyboard teleop
export ROS_DOMAIN_ID=42
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Policy Controller (RL-based velocity tracking)

```bash
# With policy enabled (uses trained neural network)
ros2 launch ogre_policy_controller policy_controller.launch.py use_policy:=true

# Pass-through mode (forwards Twist directly)
ros2 launch ogre_policy_controller policy_controller.launch.py use_policy:=false
```

## Training New Policy (ogre-lab)

```bash
cd ~/ogre-lab

# Train (1024 envs, 1000 iterations)
./scripts/train_ogre_navigation.sh 1024 1000

# Export to ONNX
./scripts/export_policy.sh --headless

# Deploy to ogre-slam
./scripts/deploy_model.sh --rebuild
```

## Debugging

```bash
# Check topics
ros2 topic list
ros2 topic hz /odom        # Should be ~50 Hz
ros2 topic hz /scan        # Should be ~5-6 Hz

# Echo topics
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel --once
ros2 topic echo /joint_command --once

# Check TF tree
ros2 run tf2_tools view_frames  # Creates frames.pdf
```

## Network Setup

```bash
# Robot IP: 10.21.21.45
export ROS_DOMAIN_ID=42
```
