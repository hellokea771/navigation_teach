#!/bin/bash
# Pure Pursuit Demo Runner
# Kills old zombie processes first, then launches the demo.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== Pure Pursuit Demo ==="
echo "Cleaning up old processes..."

# Kill only processes matching the install lib path (not this script itself)
pkill -9 -f "install/pure_pursuit_rviz_demo/lib" 2>/dev/null
sleep 0.5

echo "Starting demo..."
echo ""

# Source environment and launch
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/install/setup.bash"

exec ros2 launch pure_pursuit_rviz_demo demo.launch.py "$@"
