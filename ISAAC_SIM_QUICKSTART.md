# Isaac Sim Quickstart

Step-by-step guide to run the Ogre robot in Isaac Sim with RL policy control and Nav2 navigation.

## Prerequisites

- Isaac Sim 5.0+ installed
- ROS2 Humble workspace built
- Trained policy deployed to `ogre_policy_controller/models/policy.onnx`

## Step 1: Start Isaac Sim

```bash
# Open Isaac Sim
~/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/isaacsim/isaacsim.sh
```

In Isaac Sim:
1. File → Open → `~/ros2_ws/src/ogre-slam/usds/ogre.usd`
2. Press **Play** ▶️

## Step 2: Launch ROS2 Stack

Open 4 terminals. In each, first run:
```bash
export ROS_DOMAIN_ID=42
source ~/ros2_ws/install/setup.bash
```

### Terminal 1: Policy Controller
```bash
ros2 launch ogre_policy_controller policy_controller.launch.py use_policy:=true
```

### Terminal 2: SLAM (builds map)
```bash
ros2 launch ogre_slam mapping.launch.py use_rviz:=false use_teleop:=false
```

### Terminal 3: RViz Visualization
```bash
cd ~/ros2_ws/src/ogre-slam
./scripts/launch_isaac_sim_rviz.sh
```

### Terminal 4: Teleop (optional, for manual control)
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Step 3: Navigate

In RViz:
1. Click **2D Pose Estimate** → Set robot's initial position on map
2. Click **2D Goal Pose** → Click destination on map
3. Robot will navigate autonomously

## Quick Test (Forward Motion)

To quickly test the policy without full Nav2:

```bash
# Terminal 1: Policy controller
export ROS_DOMAIN_ID=42
source ~/ros2_ws/install/setup.bash
ros2 launch ogre_policy_controller policy_controller.launch.py use_policy:=true

# Terminal 2: Send velocity command
export ROS_DOMAIN_ID=42
ros2 topic pub /cmd_vel_smoothed geometry_msgs/msg/Twist \
  "{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

Robot should move forward with symmetric wheel velocities.

## Verify Topics

```bash
# Check policy controller output
ros2 topic echo /joint_command

# Expected for forward motion: all 4 wheels similar velocities
# [FL, FR, RL, RR] should be approximately equal for straight motion
```

## Troubleshooting

**Robot doesn't move:**
- Check Isaac Sim is in Play mode
- Verify `/joint_command` topic is being published
- Check action graph in Isaac Sim subscribes to `/joint_command`

**Robot veers to one side:**
- Check wheel velocities are symmetric in `/joint_command`
- May need to retrain policy with symmetry penalty

**RViz shows no data:**
- Ensure `ROS_DOMAIN_ID=42` is set
- Check Isaac Sim is publishing `/odom`, `/scan`, `/tf`
