# Isaac Sim Quickstart

Run autonomous navigation in Isaac Sim with the trained RL policy.

## Step 1: Start Isaac Sim

1. Open Isaac Sim
2. File → Open → `~/ros2_ws/src/ogre-slam/usds/ogre.usd`
3. Press **Play** ▶️

## Step 2: Launch Navigation Stack

```bash
# Terminal 1: Navigation + Policy Controller
export ROS_DOMAIN_ID=42
source ~/ros2_ws/install/setup.bash
ros2 launch ogre_slam navigation.launch.py map:=~/ros2_ws/src/ogre-slam/maps/wide_maze.yaml use_rviz:=false
```

```bash
# Terminal 2: Policy Controller
export ROS_DOMAIN_ID=42
source ~/ros2_ws/install/setup.bash
ros2 launch ogre_policy_controller policy_controller.launch.py use_policy:=true
```

```bash
# Terminal 3: RViz
export ROS_DOMAIN_ID=42
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/ogre-slam
./scripts/launch_isaac_sim_rviz.sh
```

## Step 3: Navigate

In RViz:
1. Click **2D Pose Estimate** → Set robot's initial position
2. Click **2D Goal Pose** → Click destination
3. Robot navigates autonomously
