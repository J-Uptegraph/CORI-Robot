#!/bin/bash
echo "🧹 CORI System Cleanup"
echo "====================="

echo "Killing all ROS and Gazebo processes..."

# Kill all ROS processes
pkill -f "ros2" 2>/dev/null || true
pkill -f "ign gazebo" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "v4l2_camera" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true
pkill -f "cori_cv" 2>/dev/null || true
pkill -f "laundry_color_detector" 2>/dev/null || true
pkill -f "simple_color_detector" 2>/dev/null || true
pkill -f "color_display" 2>/dev/null || true

# Force kill any stubborn processes
pkill -9 -f "v4l2" 2>/dev/null || true
pkill -9 -f "camera" 2>/dev/null || true

# Wait for processes to die
sleep 3

# Reset USB camera driver (simulates unplug/replug)
echo "🔄 Resetting USB camera driver..."
sudo modprobe -r uvcvideo 2>/dev/null || true
sleep 2
sudo modprobe uvcvideo 2>/dev/null || true
sleep 2

echo "✅ Cleanup complete!"
echo "📷 Camera devices available: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
echo "💡 System ready for restart"
echo ""
echo "🎯 To restart CORI system, run: ./build.bash"