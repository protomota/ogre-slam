#!/bin/bash
# Debug Isaac Sim crash by launching from terminal
# This will show real-time error messages

set -e

ISAAC_SIM="$HOME/isaac-sim"
USD_FILE="$HOME/ros2_ws/src/ogre-slam/ogre_stable.usd"

echo "🔍 Launching Isaac Sim with debug output..."
echo "📂 Loading: $USD_FILE"
echo ""
echo "Watch for error messages below. Common crashes:"
echo "  - 'Overlap detected' = collision geometry overlap"
echo "  - 'Invalid joint' = bad articulation configuration"
echo "  - 'Mass must be positive' = physics mass issue"
echo ""
echo "Press Ctrl+C to cancel"
echo "========================================"
echo ""

cd "$ISAAC_SIM"
./isaac-sim.sh "$USD_FILE" 2>&1 | tee /tmp/isaac_sim_crash.log

echo ""
echo "========================================"
echo "Log saved to: /tmp/isaac_sim_crash.log"
