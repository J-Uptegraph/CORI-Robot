#!/bin/bash

# CORI Workspace Build Script - Modular Architecture
echo "🤖 Building CORI Modular Workspace..."
echo "📦 Building packages: cori_core cori_control cori_description cori_gui cori_simulation cori_tools cori_vision"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build with specific package order for dependencies
echo "🔨 Building core utilities first..."
colcon build --packages-select cori_core --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "🔨 Building remaining packages..."
colcon build --packages-select cori_control cori_description cori_gui cori_simulation cori_tools cori_vision --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "✅ CORI workspace build complete!"
echo "🔄 Don't forget to source the workspace: source install/setup.bash"