# Isaac Sim Quickstart

Run autonomous navigation in Isaac Sim with the trained RL policy.

## Step 1: Start Isaac Sim

1. Open Isaac Sim
2. File → Open → `~/ros2_ws/src/ogre-slam/usds/ogre.usd`
3. Press **Play** ▶️

## Step 2: Launch Navigation Stack

```bash
conda deactivate  # Exit conda if active
export ROS_DOMAIN_ID=42
source ~/ros2_ws/install/setup.bash

# CRITICAL: use_sim_time:=true is REQUIRED for Isaac Sim!
ros2 launch ogre_slam navigation.launch.py \
    map:=~/ros2_ws/src/ogre-slam/maps/isaac_sim_map.yaml \
    use_sim_time:=true
```

> **CRITICAL:** The `use_sim_time:=true` flag tells all ROS2 nodes to use Isaac Sim's `/clock` topic instead of wall clock time. Without this, you'll see TF_OLD_DATA errors and navigation will fail.

## Step 3: Launch Policy Controller

```bash
conda deactivate
export ROS_DOMAIN_ID=42
source ~/ros2_ws/install/setup.bash

ros2 launch ogre_policy_controller policy_controller.launch.py use_sim_time:=true
```

## Step 4: Navigate

In RViz:
1. Click **2D Pose Estimate** → Set robot's initial position
2. Click **2D Goal Pose** → Click destination
3. Robot navigates autonomously
