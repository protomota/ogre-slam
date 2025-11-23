# Maze Generator for Isaac Sim

Automatically generate mazes for SLAM testing in Isaac Sim.

## Quick Start

### Option 1: Run from Script Editor (Recommended)

1. **Open Isaac Sim**
2. **Window → Script Editor**
3. **File → Open** → Select `scripts/generate_maze.py`
4. **Click "Run"** button
5. **Reload ogre.usd** in Isaac Sim to see the maze

### Option 2: Run from Command Line

```bash
cd ~/ros2_ws/src/ogre-slam
./scripts/run_maze_generator.sh
```

Then reload ogre.usd in Isaac Sim.

## Default Configuration

- **Maze Size:** 5×5 cells (smaller, easier to map)
- **Cell Size:** 60cm × 60cm (open space for driving)
- **Wall Dimensions:** 60cm long × 38.5cm tall × 2cm thick
- **Total Physical Size:** 3.0m × 3.0m
- **Position:** Centered at origin (0, 0, 0) with open center cell
- **Physics:** Rigid body collision with kinematic mode (static walls)
- **Algorithm:** Recursive backtracking (guarantees solution)

**Note:** The center cell (2,2) is always cleared to provide an open starting area for the robot. Cell size controls the drivable space between walls.

## Customization

Edit `scripts/generate_maze.py`:

```python
# Change maze size
maze = MazeGenerator(width=10, height=10)  # 10x10 instead of 8x8

# Change cell size and wall dimensions
create_maze_usd(
    maze,
    cell_size=0.80,        # 80cm cells (more space for driving)
    wall_height=0.385,     # 38.5cm tall (matches real cardboard)
    wall_thickness=0.02,   # 2cm thick
    maze_x=-2.0,           # Maze Xform position X
    maze_y=-2.0,           # Maze Xform position Y
    maze_z=0.0             # Maze Xform position Z (ground level)
)

# Get same maze every time (reproducible)
random.seed(42)  # Add at top of main()
maze = MazeGenerator(width=8, height=8)
```

## Features

✅ **Physics Collision** - Walls have rigid body physics (kinematic mode)
✅ **Random Generation** - Different maze each time
✅ **Guaranteed Solution** - Always has a path from start to end
✅ **Customizable** - Easy to adjust size, dimensions, position
✅ **Static Walls** - Walls don't move when robot collides

## Backup & Restore

A backup is automatically created: `ogre_backup.usd`

**To restore original:**
```bash
cd ~/ros2_ws/src/ogre-slam
cp ogre_backup.usd ogre.usd
```

## Using the Maze

1. **Generate maze** (see Quick Start above)
2. **Load ogre.usd** in Isaac Sim
3. **Press Play** ▶️
4. **Run SLAM mapping:**
   ```bash
   cd ~/ros2_ws
   source install/setup.bash
   export ROS_DOMAIN_ID=42
   export ROS_USE_SIM_TIME=true

   ros2 launch ogre_slam mapping.launch.py \
     use_rviz:=true \
     use_teleop:=false \
     use_odometry:=false \
     use_ekf:=false \
     use_sim_time:=true
   ```
5. **Drive robot** with teleop keyboard
6. **Watch map build** in RViz!

## Troubleshooting

**"ModuleNotFoundError: No module named 'pxr'"**
- You're not using Isaac Sim's Python
- Use Script Editor method instead

**Maze too big/small?**
- Adjust `MazeGenerator(width=X, height=Y)`
- Default is 8x8 cells = ~2m × 2m

**Robot collides with walls?**
- Walls already have collision enabled
- Check robot radius vs. maze cell size
- Increase cell_size for wider paths

**Want different maze?**
- Run script again (generates random maze)
- OR use `random.seed(N)` for reproducible mazes

## Algorithm Details

**Recursive Backtracking (Depth-First Search):**
1. Start at cell (0,0)
2. Pick random unvisited neighbor
3. Remove wall between cells
4. Move to neighbor and repeat
5. Backtrack when no unvisited neighbors
6. Guarantees perfect maze (exactly one solution path)

## Files

- `scripts/generate_maze.py` - Main maze generator script
- `scripts/run_maze_generator.sh` - Launcher script (auto-finds Isaac Sim Python)
- `ogre.usd` - Your robot scene (maze will be added here)
- `ogre_backup.usd` - Backup created automatically

---

**Tips:**
- Start with default 8x8 maze for testing
- Smaller mazes (5x5) map faster
- Larger mazes (12x12+) test SLAM loop closure
- Thin walls (2cm) match real cardboard
