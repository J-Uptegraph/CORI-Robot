#!/bin/bash

# CORI Robot Rebuild and Run Script
echo "🤖 CORI Robot - Rebuild and Run Script"
echo "======================================"

# Navigate to workspace
cd /home/juptegraph/Workspaces/Robotics/Projects/CORI/cori_ws

echo "📁 Current directory: $(pwd)"

# Clean previous build (optional)
echo "🧹 Cleaning previous build..."
rm -rf build/ devel/ install/

# Build the workspace
echo "🔨 Building workspace..."
colcon build --packages-select cori_description

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "✅ Build successful!"
else
    echo "❌ Build failed! Check the output above."
    exit 1
fi

# Source the workspace
echo "📦 Sourcing workspace..."
source install/setup.bash

# Launch the robot
echo "🚀 Launching CORI robot in Ignition Gazebo..."
echo "Press Ctrl+C to stop the simulation"
echo ""

ros2 launch cori_description spawn_cori_ignition.launch.py

echo "🏁 Simulation ended."
