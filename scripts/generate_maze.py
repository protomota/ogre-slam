#!/usr/bin/env python3
"""
Generate a maze for Isaac Sim using cubes as walls.

Wall dimensions: 25.5cm wide x 38.5cm tall x 2cm thick
Maze size: 8x8 cells
Output: maze.usd (standalone USD file)

Usage:
    python3 generate_maze.py
"""

import random
from pxr import Usd, UsdGeom, Gf, UsdPhysics


class MazeGenerator:
    """Generate a maze using recursive backtracking algorithm."""

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = [[{'N': True, 'S': True, 'E': True, 'W': True}
                      for _ in range(width)] for _ in range(height)]
        self.generate()

    def generate(self):
        """Generate maze using depth-first search with backtracking."""
        visited = set()
        stack = [(0, 0)]
        visited.add((0, 0))

        while stack:
            x, y = stack[-1]
            neighbors = []

            # Check all four directions
            for dx, dy, direction, opposite in [
                (0, -1, 'N', 'S'),  # North
                (0, 1, 'S', 'N'),   # South
                (1, 0, 'E', 'W'),   # East
                (-1, 0, 'W', 'E')   # West
            ]:
                nx, ny = x + dx, y + dy
                if (0 <= nx < self.width and 0 <= ny < self.height and
                    (nx, ny) not in visited):
                    neighbors.append((nx, ny, direction, opposite))

            if neighbors:
                # Choose random unvisited neighbor
                nx, ny, direction, opposite = random.choice(neighbors)

                # Remove walls between current cell and chosen neighbor
                self.grid[y][x][direction] = False
                self.grid[ny][nx][opposite] = False

                visited.add((nx, ny))
                stack.append((nx, ny))
            else:
                stack.pop()


def create_maze_usd(maze, cell_size=0.255, wall_height=0.385, wall_thickness=0.02,
                    usd_file="ogre.usd", maze_x=-3.6, maze_y=-1.3, maze_z=0.6):
    """
    Add maze walls to an existing USD file (or create new one).

    Args:
        maze: MazeGenerator instance
        cell_size: Size of each cell in meters (0.255m = 25.5cm)
        wall_height: Height of walls in meters (0.385m = 38.5cm)
        wall_thickness: Thickness of walls in meters (0.02m = 2cm)
        usd_file: USD file to modify (creates if doesn't exist)
        maze_x: X position for maze Xform (meters)
        maze_y: Y position for maze Xform (meters)
        maze_z: Z position for maze Xform (meters)
    """
    # Open existing USD stage or create new one
    import os
    if os.path.exists(usd_file):
        print(f"📂 Opening existing file: {usd_file}")
        stage = Usd.Stage.Open(usd_file)
    else:
        print(f"📄 Creating new file: {usd_file}")
        stage = Usd.Stage.CreateNew(usd_file)

    # Remove old maze if it exists
    old_maze = stage.GetPrimAtPath('/Maze')
    if old_maze.IsValid():
        print(f"🗑️  Removing old maze...")
        stage.RemovePrim('/Maze')

    # Create root prim for maze with position
    maze_prim = UsdGeom.Xform.Define(stage, '/Maze')
    maze_prim.AddTranslateOp().Set(Gf.Vec3d(maze_x, maze_y, maze_z))

    # Enable physics scene
    scene = UsdPhysics.Scene.Define(stage, '/physicsScene')
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(9.81)

    wall_count = 0

    # Create walls
    for y in range(maze.height):
        for x in range(maze.width):
            cell = maze.grid[y][x]

            # Cell center position (relative to maze origin, no offset since Xform handles it)
            cx = x * cell_size
            cy = y * cell_size
            cz = wall_height / 2  # Center wall vertically

            # North wall (horizontal, along X axis)
            if cell['N']:
                wall_name = f'/Maze/Wall_{wall_count}'
                wall_count += 1

                cube = UsdGeom.Cube.Define(stage, wall_name)
                cube.CreateSizeAttr(1.0)  # Unit cube, will be scaled

                # Position at north edge of cell
                pos = Gf.Vec3d(cx + cell_size/2, cy, cz)
                cube.AddTranslateOp().Set(pos)

                # Scale: X=cell_size, Y=thickness, Z=height
                scale = Gf.Vec3f(cell_size, wall_thickness, wall_height)
                cube.AddScaleOp().Set(scale)

                # Add physics - collision and static rigid body
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
                rigid_body.CreateKinematicEnabledAttr(True)  # Static wall, doesn't move

            # West wall (horizontal, along Y axis)
            if cell['W']:
                wall_name = f'/Maze/Wall_{wall_count}'
                wall_count += 1

                cube = UsdGeom.Cube.Define(stage, wall_name)
                cube.CreateSizeAttr(1.0)

                # Position at west edge of cell
                pos = Gf.Vec3d(cx, cy + cell_size/2, cz)
                cube.AddTranslateOp().Set(pos)

                # Scale: X=thickness, Y=cell_size, Z=height
                scale = Gf.Vec3f(wall_thickness, cell_size, wall_height)
                cube.AddScaleOp().Set(scale)

                # Add physics - collision and static rigid body
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
                rigid_body.CreateKinematicEnabledAttr(True)  # Static wall, doesn't move

    # Add south boundary wall for last row
    y = maze.height - 1
    for x in range(maze.width):
        if maze.grid[y][x]['S']:
            wall_name = f'/Maze/Wall_{wall_count}'
            wall_count += 1

            cx = x * cell_size
            cy = (y + 1) * cell_size
            cz = wall_height / 2

            cube = UsdGeom.Cube.Define(stage, wall_name)
            cube.CreateSizeAttr(1.0)

            pos = Gf.Vec3d(cx + cell_size/2, cy, cz)
            cube.AddTranslateOp().Set(pos)

            scale = Gf.Vec3f(cell_size, wall_thickness, wall_height)
            cube.AddScaleOp().Set(scale)

            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            rigid_body.CreateKinematicEnabledAttr(True)

    # Add east boundary wall for last column
    x = maze.width - 1
    for y in range(maze.height):
        if maze.grid[y][x]['E']:
            wall_name = f'/Maze/Wall_{wall_count}'
            wall_count += 1

            cx = (x + 1) * cell_size
            cy = y * cell_size
            cz = wall_height / 2

            cube = UsdGeom.Cube.Define(stage, wall_name)
            cube.CreateSizeAttr(1.0)

            pos = Gf.Vec3d(cx, cy + cell_size/2, cz)
            cube.AddTranslateOp().Set(pos)

            scale = Gf.Vec3f(wall_thickness, cell_size, wall_height)
            cube.AddScaleOp().Set(scale)

            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            rigid_body = UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
            rigid_body.CreateKinematicEnabledAttr(True)

    # Save stage
    stage.GetRootLayer().Save()

    print(f"✅ Maze added successfully!")
    print(f"   File: {usd_file}")
    print(f"   Walls: {wall_count}")
    print(f"   Maze size: {maze.width}x{maze.height} cells")
    print(f"   Physical size: {maze.width * cell_size:.2f}m x {maze.height * cell_size:.2f}m")
    print(f"   Wall dimensions: {cell_size*100:.1f}cm long x {wall_height*100:.1f}cm tall x {wall_thickness*100:.1f}cm thick")
    print(f"   Maze position: ({maze_x:.2f}m, {maze_y:.2f}m, {maze_z:.2f}m)")
    print(f"   Physics: ✅ Rigid body collision enabled")
    print(f"\nTo view in Isaac Sim:")
    print(f"   File → Open → {usd_file}")


def main():
    """Generate maze and add to ogre.usd file."""
    print("🏗️  Generating 5x5 maze for Project Ogre...")

    # Generate maze (start from center for open middle)
    random.seed()  # Random each time
    maze = MazeGenerator(width=5, height=5)

    # Clear center cell walls to ensure open space for robot start
    center = 2  # Center of 5x5 is index 2
    maze.grid[center][center] = {'N': False, 'S': False, 'E': False, 'W': False}

    # Calculate centered position
    # Robot: 205mm wide × 95mm long (diagonal: 226mm)
    # Cell size: 0.60m = 600mm (374mm clearance for diagonal movement)
    # 5 cells × 0.6m = 3.0m total maze size
    # To center at origin: offset by -half_size = -1.5m
    maze_size = 5 * 0.60  # 3.0m
    center_offset = -maze_size / 2  # -1.5m

    # Add maze to ogre.usd (current robot configuration)
    create_maze_usd(
        maze,
        cell_size=0.60,       # 60cm open space (comfortable for 205mm wide robot)
        wall_height=0.385,    # 38.5cm tall
        wall_thickness=0.02,  # 2cm thick
        usd_file="/home/brad/ros2_ws/src/ogre-slam/ogre.usd",
        maze_x=center_offset, # Centered X (-1.5m)
        maze_y=center_offset, # Centered Y (-1.5m)
        maze_z=0.6            # Wall base at 60cm height
    )

    print("\n📝 Customization options:")
    print("   Edit the script to change:")
    print("   - Maze size: MazeGenerator(width=5, height=5)")
    print("   - Cell size: cell_size=0.60 (60cm paths for 205mm robot)")
    print("   - Wall height: wall_height=0.385 (38.5cm tall)")
    print("   - Wall thickness: wall_thickness=0.02 (2cm thick)")
    print("   - Position: Auto-centered at origin with open center")
    print("   - Random seed: random.seed(42) for reproducible mazes")
    print("\n⚠️  Note: This modifies ogre.usd - make a backup first!")
    print("💡 Tip: Center cell is always clear for robot starting position")
    print("💡 Robot: 205mm×95mm (diagonal 226mm) fits with 374mm clearance")
    print("💡 Physics: Walls have rigid body collision (kinematic mode)")
    print("💡 For wider robot: See OGRE_WIDE.md for 80cm cell configuration")


if __name__ == "__main__":
    main()
