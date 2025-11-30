#!/bin/bash
# Run maze generator using Isaac Sim's Python
# This will add a maze to the existing ogre.usd

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Find Isaac Sim installation
ISAAC_SIM_PATH=$(find ~/.local/share/ov/pkg -maxdepth 1 -type d -name "isaac-sim-*" | sort -V | tail -1)

if [ -z "$ISAAC_SIM_PATH" ]; then
    echo "❌ Isaac Sim not found!"
    echo ""
    echo "Alternative: Run from Isaac Sim Script Editor"
    echo "  1. Open Isaac Sim"
    echo "  2. Window → Script Editor"
    echo "  3. Open scripts/generate_maze.py"
    echo "  4. Click Run"
    exit 1
fi

echo "🔍 Found Isaac Sim: $ISAAC_SIM_PATH"
echo "🏗️  Adding maze to ogre.usd..."
echo ""

"$ISAAC_SIM_PATH/python.sh" "$SCRIPT_DIR/generate_maze.py"

echo ""
echo "✨ Done! Reload ogre.usd in Isaac Sim to see the maze."
