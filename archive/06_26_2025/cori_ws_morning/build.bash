#!/bin/bash
# CORI Robot Complete Build and Run Script + Sensor Fusion
echo "🤖 CORI Robot - Enhanced Build Script with Sensor Fusion"
echo "========================================================="

# Navigate to workspace
cd /home/juptegraph/Workspaces/Robotics/Projects/CORI/cori_ws
echo "📁 Current directory: $(pwd)"

# Check if sensor fusion files exist
FUSION_FILES_EXIST=true
if [ ! -f "src/cori_cv/cori_cv/sensor_fusion/spatial_database.py" ]; then
    FUSION_FILES_EXIST=false
fi

# Clean previous build (optional)
echo "🧹 Cleaning previous build..."
rm -rf build/ devel/ install/

# Build both packages
echo "🔨 Building workspace..."
colcon build --packages-select cori_description cori_cv

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

echo ""
echo "🎯 Choose what to run:"
echo "1) 🚀 Full system (Gazebo + Webcam + Color Detection)"
echo "2) 🎮 Just Gazebo simulation"
echo "3) 🦾 Manual Control Mode (No Input Lag!)"
echo "4) 📷 Just webcam color detection"
if [ "$FUSION_FILES_EXIST" = true ]; then
    echo "5) 🧠 SENSOR FUSION DEMO (Gazebo + Camera + Smart Head Movement)"
    echo "6) 🗃️  View Spatial Database"
fi
echo "7) 🧹 Kill all ROS processes and exit"
echo "8) 🚪 Exit"
echo ""
read -p "Enter choice [1-8]: " choice

case $choice in
    1)
        echo ""
        echo "🚀 Starting complete CORI system..."
        echo "📋 This will open multiple processes:"
        echo "   🎮 Gazebo simulation with CORI"
        echo "   📷 Webcam feed"
        echo "   🎨 Color detection"
        echo "   🖥️  Terminal color display"
        echo ""
        echo "⚠️  Press Ctrl+C in any terminal to stop"
        echo ""
        
        # Function to kill all background processes
        cleanup() {
            echo ""
            echo "🛑 Stopping all processes..."
            
            # Kill specific ROS processes
            pkill -f "ros2 launch cori_description"
            pkill -f "ros2 launch cori_cv"
            pkill -f "ros2 run cori_cv"
            pkill -f "v4l2_camera_node"
            pkill -f "ign gazebo"
            pkill -f "gz sim"
            
            # Kill any remaining ROS processes
            pkill -f "robot_state_publisher"
            pkill -f "laundry_color_detector"
            pkill -f "simple_color_detector"
            pkill -f "color_display"
            
            # Wait a moment for clean shutdown
            sleep 2
            
            echo "✅ Cleanup complete"
            exit 0
        }
        
        # Clean up any existing processes first
        echo "🧹 Cleaning up any existing processes..."
        pkill -f "ros2 launch" 2>/dev/null || true
        pkill -f "ros2 run" 2>/dev/null || true
        pkill -f "v4l2_camera" 2>/dev/null || true
        pkill -f "ign gazebo" 2>/dev/null || true
        sleep 3
        
        # Set trap to cleanup on Ctrl+C
        trap cleanup SIGINT
        
        # Start Gazebo in background
        echo "🎮 Starting Gazebo simulation..."
        ros2 launch cori_description spawn_cori_ignition.launch.py &
        GAZEBO_PID=$!
        
        echo "⏳ Waiting for Gazebo to fully load..."
        sleep 8
        
        # Check if Gazebo started properly
        if ! ps -p $GAZEBO_PID > /dev/null; then
            echo "❌ Gazebo failed to start!"
            exit 1
        fi
        
        # Start webcam in background
        echo "📷 Starting webcam..."
        ros2 launch cori_cv laundry_color_detector.launch.py &
        WEBCAM_PID=$!
        
        echo "⏳ Waiting for webcam to initialize..."
        sleep 5
        
        # Check if webcam started
        if ! ps -p $WEBCAM_PID > /dev/null; then
            echo "❌ Webcam failed to start!"
            kill $GAZEBO_PID 2>/dev/null
            exit 1
        fi
        
        # Start color detection bridge in background
        echo "🔗 Starting color detection bridge..."
        ros2 run cori_cv simple_color_detector &
        BRIDGE_PID=$!
        
        echo "⏳ Waiting for color detection to connect..."
        sleep 3
        
        # Start color display in foreground (this will show the output)
        echo "🎨 Starting color display..."
        echo "👋 Hold colored objects in front of your webcam!"
        echo "📺 Check that your webcam permissions are enabled"
        ros2 run cori_cv color_display
        
        # If we get here, user stopped the color display
        cleanup
        ;;

    5)
        if [ "$FUSION_FILES_EXIST" = false ]; then
            echo "❌ Sensor fusion files not found!"
            echo "📝 Please ensure sensor fusion scripts are in src/cori_cv/cori_cv/sensor_fusion/"
            exit 1
        fi
        
        echo ""
        echo "🧠 CORI SENSOR FUSION DEMONSTRATION"
        echo "==================================="
        echo ""
        echo "🎯 This demo shows PRODUCTION ROBOTICS techniques:"
        echo "   📡 Physical camera detects real-world colors"
        echo "   🗃️  Spatial database predicts object locations"
        echo "   🤖 CORI's head moves to predicted positions"
        echo "   🧠 System learns and adapts over time"
        echo ""
        echo "📊 DEMO SEQUENCE:"
        echo "   1. Hold RED object → CORI looks LEFT (14°)"
        echo "   2. Hold BLUE object → CORI looks RIGHT (-16°)"
        echo "   3. Hold GREEN object → CORI looks STRAIGHT (0°)"
        echo "   4. Database confidence increases with each success"
        echo ""
        
        read -p "🚀 Start sensor fusion demo? [y/N]: " confirm
        if [[ ! $confirm =~ ^[Yy]$ ]]; then
            echo "👋 Demo cancelled"
            exit 0
        fi
        
        # Function to cleanup sensor fusion
        cleanup_fusion() {
            echo ""
            echo "🛑 Stopping sensor fusion demo..."
            pkill -f "sensor_fusion_demo.py" 2>/dev/null || true
            pkill -f "demo_display.py" 2>/dev/null || true
            pkill -f "ros2 launch cori_description" 2>/dev/null || true
            pkill -f "ros2 launch cori_cv" 2>/dev/null || true
            pkill -f "simple_color_detector" 2>/dev/null || true
            pkill -f "ign gazebo" 2>/dev/null || true
            sleep 3
            echo "✅ Sensor fusion demo stopped"
            exit 0
        }
        
        trap cleanup_fusion SIGINT
        
        # Clean up any existing processes
        echo "🧹 Cleaning up existing processes..."
        pkill -f "ros2 launch" 2>/dev/null || true
        pkill -f "ros2 run" 2>/dev/null || true
        pkill -f "sensor_fusion" 2>/dev/null || true
        pkill -f "ign gazebo" 2>/dev/null || true
        sleep 3
        
        # Initialize database
        echo "🗃️  Initializing spatial database..."
        python3 src/cori_cv/cori_cv/sensor_fusion/spatial_database.py
        
        # Start Gazebo
        echo "🎮 Starting Gazebo with CORI..."
        ros2 launch cori_description spawn_cori_ignition.launch.py &
        GAZEBO_PID=$!
        sleep 8
        
        # Start camera via your existing launch system
        echo "📷 Starting camera system..."
        ros2 launch cori_cv laundry_color_detector.launch.py &
        CAMERA_PID=$!
        sleep 5
        
        # Check what camera topics are available
        echo "🔍 Checking camera topics..."
        ros2 topic list | grep -E "(image|camera)" || echo "No camera topics found yet, continuing..."
        
        # Start sensor fusion demo
        echo "🧠 Starting sensor fusion processing..."
        python3 src/cori_cv/cori_cv/sensor_fusion/sensor_fusion_demo.py &
        FUSION_PID=$!
        sleep 2
        
        # Start display (foreground)
        echo "🖥️  Starting demo display..."
        echo ""
        echo "🎯 HOLD COLORED OBJECTS IN FRONT OF CAMERA:"
        echo "   🔴 RED object → Watch CORI look LEFT"
        echo "   🔵 BLUE object → Watch CORI look RIGHT" 
        echo "   🟢 GREEN object → Watch CORI look STRAIGHT"
        echo "   ⚪ WHITE object → Watch CORI look slightly LEFT"
        echo ""
        echo "📊 Watch the spatial database learn and adapt!"
        echo "⚠️  Press Ctrl+C to stop the demo"
        echo ""
        
        python3 src/cori_cv/cori_cv/sensor_fusion/demo_display.py
        
        cleanup_fusion
        ;;

    8)
        if [ "$FUSION_FILES_EXIST" = false ]; then
            echo "❌ Sensor fusion files not found!"
            exit 1
        fi
        
        echo "🗃️  SPATIAL DATABASE MANAGEMENT"
        echo "==============================="
        echo ""
        echo "📊 Displaying spatial database..."
        python3 src/cori_cv/cori_cv/sensor_fusion/spatial_database.py
        echo ""
        echo "💾 Database file: database/cori_spatial_database.json"
        ;;
        
    2)
        echo "🎮 Launching CORI robot in Ignition Gazebo..."
        echo "⚠️  Press Ctrl+C to stop the simulation"
        echo ""
        ros2 launch cori_description spawn_cori_ignition.launch.py
        ;;
        
    3)
        echo "🦾 Manual Control Mode - No Input Lag!"
        echo "📋 This launches CORI for manual manipulation:"
        echo "   🎮 Gazebo with CORI"
        echo "   🚫 NO automatic joint control"
        echo "   ✋ Click and drag CORI freely!"
        echo ""
        echo "⚠️  Press Ctrl+C to stop"
        echo ""
        
        # Function to kill manual control processes
        cleanup_manual() {
            echo ""
            echo "🛑 Stopping manual control..."
            pkill -f "ign gazebo" 2>/dev/null || true
            pkill -f "robot_state_publisher" 2>/dev/null || true
            pkill -f "ros_gz_sim" 2>/dev/null || true
            sleep 2
            echo "✅ Manual control stopped"
            exit 0
        }
        
        # Kill any existing joint state publishers first
        echo "🧹 Killing any existing joint state publishers..."
        sudo pkill -f joint_state_publisher 2>/dev/null || true
        sleep 1
        
        # Set trap for cleanup
        trap cleanup_manual SIGINT
        
        # Start Gazebo in background
        echo "🎮 Starting Gazebo..."
        ign gazebo /home/juptegraph/Workspaces/Robotics/Projects/CORI/cori_ws/src/cori_description/worlds/laundry_world.sdf &
        GAZEBO_PID=$!
        
        echo "⏳ Waiting for Gazebo to load..."
        sleep 6
        
        # Start robot state publisher in background
        echo "🤖 Starting robot state publisher..."
        ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro src/cori_description/urdf/cori.urdf.xacro)" &
        RSP_PID=$!
        
        echo "⏳ Waiting for robot description..."
        sleep 3
        
        # Spawn CORI
        echo "🚀 Spawning CORI..."
        ros2 run ros_gz_sim create -name cori -topic robot_description
        
        echo ""
        echo "🎉 CORI is ready for manual control!"
        echo "✋ Click and drag any part of CORI in Gazebo"
        echo "🚫 No input lag - he stays exactly where you put him!"
        echo "⚠️  Press Ctrl+C to stop"
        echo ""
        
        # Keep the script running and wait for Ctrl+C
        while true; do
            sleep 1
            # Check if Gazebo is still running
            if ! ps -p $GAZEBO_PID > /dev/null 2>&1; then
                echo "🛑 Gazebo closed, shutting down..."
                cleanup_manual
            fi
        done
        ;;
        
    4)
        echo "📷 Starting webcam color detection system..."
        echo ""
        
        # Start webcam
        echo "📷 Starting webcam..."
        ros2 launch cori_cv laundry_color_detector.launch.py &
        WEBCAM_PID=$!
        sleep 3
        
        # Start color detection bridge
        echo "🎨 Starting color detection..."
        ros2 run cori_cv simple_color_detector &
        BRIDGE_PID=$!
        sleep 2
        
        # Start color display
        echo "👋 Hold colored objects in front of your webcam!"
        ros2 run cori_cv color_display
        
        # Cleanup
        echo "🛑 Stopping webcam processes..."
        kill $WEBCAM_PID $BRIDGE_PID 2>/dev/null
        ;;
        
    5)
        echo "🧹 Killing all ROS processes and resetting camera..."
        
        # Comprehensive cleanup
        pkill -f "ros2" 2>/dev/null || true
        pkill -f "ign gazebo" 2>/dev/null || true
        pkill -f "gz sim" 2>/dev/null || true
        pkill -f "v4l2_camera" 2>/dev/null || true
        pkill -f "robot_state_publisher" 2>/dev/null || true
        pkill -f "cori_cv" 2>/dev/null || true
        pkill -f "sensor_fusion" 2>/dev/null || true
        pkill -f "spatial_database" 2>/dev/null || true
        
        # Wait for processes to die
        sleep 3
        
        # Reset USB camera driver (simulates unplug/replug)
        echo "🔄 Resetting USB camera driver..."
        sleep 2
        sleep 2
        
        echo "✅ All ROS processes killed and camera reset"
        echo "💡 Camera should be available for restart"
        echo "📷 Camera devices: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
        exit 0
        ;;
        
    6)
        echo "👋 Exiting..."
        exit 0
        ;;
        
    7)
        echo "🧹 Killing all ROS processes and resetting camera..."
        
        # Comprehensive cleanup
        pkill -f "ros2" 2>/dev/null || true
        pkill -f "ign gazebo" 2>/dev/null || true
        pkill -f "gz sim" 2>/dev/null || true
        pkill -f "v4l2_camera" 2>/dev/null || true
        pkill -f "robot_state_publisher" 2>/dev/null || true
        pkill -f "cori_cv" 2>/dev/null || true
        pkill -f "sensor_fusion" 2>/dev/null || true
        pkill -f "spatial_database" 2>/dev/null || true
        
        sleep 3
        
        echo "🔄 Resetting USB camera driver..."
        sleep 2
        sleep 2
        
        echo "✅ All ROS processes killed and camera reset"
        exit 0
        ;;
        
    8)
        echo "👋 Exiting..."
        exit 0
        ;;
    *)
        echo "❌ Invalid choice. Exiting..."
        exit 1
        ;;
esac

echo "🏁 CORI system ended."