#!/bin/bash
# CORI Robot Build and Run Script - Cooperative Organizational Robotic Intelligence
# Description: Unified build and execution for CORI's laundry sorting system

# Constants
WORKSPACE_DIR="/home/juptegraph/Workspaces/Robotics/Projects/CORI/cori_ws"
SENSOR_FUSION_PATH="src/cori_cv/cori_cv/sensor_fusion/spatial_database.py"
INTEGRATION_PATH="src/cori_tools/cori_tools/cori_ignition_integration.py"
WORLD_FILE="src/cori_description/worlds/laundry_world.sdf"
URDF_FILE="src/cori_description/urdf/cori.urdf.xacro"

# Function to display a loading bar
show_loading_bar() {
    local duration=$1
    local bar_length=20
    local sleep_time=$(echo "scale=2; $duration / $bar_length" | bc)

    echo -ne "   ["
    for i in $(seq 1 $bar_length); do
        printf "\e[32m█\e[0m"
        sleep $sleep_time
    done
    echo -e "]\n"
}

# Display startup sequence
show_startup_sequence() {
    echo -e "\n🤖 Initializing C.O.R.I. system...."
    show_loading_bar 1
    
    # --- Start of Banner Box (Using rounded borders) ---
    local TOTAL_BOX_WIDTH=70
    local banner_inner_width=$((TOTAL_BOX_WIDTH - 2)) 

    # Top border - now rounded
    echo -e "\n╭"$(printf '─%.0s' $(seq 1 $banner_inner_width))"╮"

    # Blank line inside banner
    printf "│%*s│\n" $banner_inner_width ""

local banner_lines=(
    "    ██████╗    ██████╗    ██████╗    ██╗    "
    "   ██╔════╝   ██╔═══██╗   ██╔══██╗   ██║    "
    "   ██║        ██║   ██║   ██████╔╝   ██║    "
    "   ██║        ██║   ██║   ██╔══██╗   ██║    "
    "   ╚██████╗██╗╚██████╔╝██╗██║  ██║██╗██║ ██╗"
    "    ╚═════╝╚═╝ ╚═════╝ ╚═╝╚═╝  ╚═╝╚═╝ ╚═╝"
)

    for line in "${banner_lines[@]}"; do
        local len=${#line}
        local padding_left=$(( (banner_inner_width - len) / 2 ))
        local padding_right=$(( banner_inner_width - len - padding_left ))
        printf "│%*s%s%*s│\n" $padding_left "" "$line" $padding_right ""
    done

    # Blank line inside banner
    printf "│%*s│\n" $banner_inner_width ""

    local text_lines=(
        "Cooperative Organizational Robotic Intelligence"
        "Developed by Johnathan Uptegraph - 2025"
        "Built to function, designed to matter."
    )
    for text_line in "${text_lines[@]}"; do
        local len=${#text_line}
        local padding_left=$(( (banner_inner_width - len) / 2 ))
        local padding_right=$(( banner_inner_width - len - padding_left ))
        printf "│%*s%s%*s│\n" $padding_left "" "$text_line" $padding_right ""
    done
    
    # Blank line inside banner
    printf "│%*s│\n" $banner_inner_width ""

    echo "╰"$(printf '─%.0s' $(seq 1 $banner_inner_width))"╯"
    # --- End of Banner Box ---
}

# Check if file exists
check_file() {
    local file_path="$1"
    [ -f "$file_path" ] || { echo "❌ File not found: $file_path"; return 1; }
    return 0
}

# Clean up processes
cleanup_processes() {
    local mode="$1"
    echo "🛑 Stopping $mode processes..."
    pkill -f "ros2 launch" 2>/dev/null || true
    pkill -f "ros2 run" 2>/dev/null || true
    pkill -f "ign gazebo" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "v4l2_camera" 2>/dev/null || true
    pkill -f "robot_state_publisher" 2>/dev/null || true
    pkill -f "cori_cv" 2>/dev/null || true
    pkill -f "sensor_fusion" 2>/dev/null || true
    pkill -f "spatial_database" 2>/dev/null || true
    pkill -f "cori_ignition_integration" 2>/dev/null || true
    pkill -f "laundry_color_detector" 2>/dev/null || true
    sleep 3
    echo "✅ $mode processes stopped"
}

# Build workspace
build_workspace() {
    echo "🧹 Cleaning previous build..."
    rm -rf build/ devel/ install/
    echo "🔨 Building workspace..."
    colcon build --packages-select cori_description cori_vision cori_control cori_simulation cori_core cori_gui cori_tools
    [ $? -eq 0 ] && echo "✅ Build successful!" || { echo "❌ Build failed!"; exit 1; }
    echo "📦 Sourcing workspace..."
    source install/setup.bash
}

# Start Gazebo simulation
start_gazebo() {
    local pid_var="$1"
    echo "🎮 Starting Gazebo simulation..."
    ros2 launch cori_description spawn_cori_ignition.launch.py &
    eval "$pid_var=\$!"
    sleep 8
    [ -z "$(ps -p ${!pid_var} -o pid=)" ] && { echo "❌ Gazebo failed to start!"; exit 1; }
}

# Start webcam
start_webcam() {
    local pid_var="$1"
    echo "📷 Starting webcam..."
    ros2 launch cori_cv laundry_color_detector.launch.py &
    eval "$pid_var=\$!"
    sleep 5
    [ -z "$(ps -p ${!pid_var} -o pid=)" ] && { echo "❌ Webcam failed to start!"; return 1; }
    return 0
}

# Run full system
run_full_system() {
    cleanup_processes "full system"
    trap 'cleanup_processes "full system"; exit 0' SIGINT
    start_gazebo GAZEBO_PID
    start_webcam WEBCAM_PID || { kill $GAZEBO_PID 2>/dev/null; exit 1; }
    echo "🔗 Starting color detection bridge..."
    ros2 run cori_cv simple_color_detector &
    BRIDGE_PID=$!
    sleep 3
    echo "🎨 Starting color display..."
    echo "👋 Hold colored objects in front of your webcam!"
    echo "📺 Ensure webcam permissions are enabled"
    ros2 run cori_cv color_display
    cleanup_processes "full system"
}

# Run Gazebo simulation only
run_gazebo_only() {
    cleanup_processes "Gazebo"
    trap 'cleanup_processes "Gazebo"; exit 0' SIGINT
    ros2 launch cori_description spawn_cori_ignition.launch.py
}

# Run manual control mode
run_manual_control() {
    cleanup_processes "manual control"
    trap 'cleanup_processes "manual control"; exit 0' SIGINT
    echo "🎮 Starting Gazebo..."
    ign gazebo "$WORLD_FILE" &
    GAZEBO_PID=$!
    sleep 6
    echo "🤖 Starting robot state publisher..."
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro "$URDF_FILE")" &
    RSP_PID=$!
    sleep 3
    echo "🚀 Spawning CORI..."
    ros2 run ros_gz_sim create -name cori -topic robot_description
    echo "🎉 CORI is ready for manual control!"
    echo "✋ Click and drag CORI in Gazebo"
    while true; do sleep 1; [ -z "$(ps -p $GAZEBO_PID -o pid=)" ] && cleanup_processes "manual control"; done
}

# Run webcam color detection
run_webcam_color() {
    cleanup_processes "webcam color detection"
    trap 'cleanup_processes "webcam color detection"; exit 0' SIGINT
    start_webcam WEBCAM_PID || exit 1
    echo "🎨 Starting color detection..."
    ros2 run cori_cv simple_color_detector &
    BRIDGE_PID=$!
    sleep 2
    echo "👋 Hold colored objects in front of your webcam!"
    ros2 run cori_cv color_display
    cleanup_processes "webcam color detection"
}

# Run sensor fusion demo
run_sensor_fusion() {
    [ $(check_file "$SENSOR_FUSION_PATH"; echo $?) -ne 0 ] && exit 1
    echo "🧠 CORI SENSOR FUSION DEMONSTRATION"
    echo "==================================="
    echo "🎯 DEMO SEQUENCE:"
    echo "   1. Hold RED object → CORI looks LEFT (14°)"
    echo "   2. Hold BLUE object → CORI looks RIGHT (-16°)"
    echo "   3. Hold GREEN object → CORI looks STRAIGHT (0°)"
    read -p "🚀 Start sensor fusion demo? [y/N]: " confirm
    [[ ! $confirm =~ ^[Yy]$ ]] && { echo "👋 Demo cancelled"; exit 0; }
    cleanup_processes "sensor fusion"
    trap 'cleanup_processes "sensor fusion"; exit 0' SIGINT
    echo "🗃️ Initializing spatial database..."
    python3 "$SENSOR_FUSION_PATH"
    start_gazebo GAZEBO_PID
    start_webcam CAMERA_PID || { kill $GAZEBO_PID 2>/dev/null; exit 1; }
    echo "🔍 Checking camera topics..."
    ros2 topic list | grep -E "(image|camera)" || echo "No camera topics found yet, continuing..."
    echo "🧠 Starting sensor fusion processing..."
    python3 src/cori_cv/cori_cv/sensor_fusion/sensor_fusion_demo.py &
    FUSION_PID=$!
    sleep 2
    echo "🖥️ Starting demo display..."
    echo "🎯 HOLD COLORED OBJECTS IN FRONT OF CAMERA:"
    echo "   🔴 RED → LEFT"
    echo "   🔵 BLUE → RIGHT"
    echo "   🟢 GREEN → STRAIGHT"
    python3 src/cori_cv/cori_cv/sensor_fusion/demo_display.py
    cleanup_processes "sensor fusion"
}

# View spatial database
view_spatial_database() {
    [ $(check_file "$SENSOR_FUSION_PATH"; echo $?) -ne 0 ] && exit 1
    echo "🗃️ SPATIAL DATABASE MANAGEMENT"
    echo "==============================="
    python3 "$SENSOR_FUSION_PATH"
    echo "💾 Database file: database/cori_spatial_database.json"
}

# Run laundry assistant
run_laundry_assistant() {
    local script_path="src/cori_cv/cori_cv/cori_simulator.py"
    [ $(check_file "$script_path"; echo $?) -ne 0 ] && { echo "❌ Laundry assistant not found!"; exit 1; }
    echo "🧺 CORI LAUNDRY SORTING ASSISTANT"
    echo "================================="
    echo "🤖 Features:"
    echo "   📚 Learns your preferences"
    echo "   🧠 Improves with each item"
    echo "   🗂️ Sorts: Lights, Darks, Colors"
    read -p "🚀 Start laundry sorting? [y/N]: " confirm
    [[ ! $confirm =~ ^[Yy]$ ]] && { echo "👋 Cancelled"; exit 0; }
    trap 'echo -e "\n🛑 Stopping...\n💾 Progress saved!"; exit 0' SIGINT
    cd src/cori_cv/cori_cv/
    echo "🚀 Launching Laundry Assistant..."
    echo "🎯 TIPS: Start with 'red shirt', 'blue jeans'; type 'quit' to stop"
    python3 cori_simulator.py
}

# NEW: Run CORI Smart Control (Your preferred mode - Camera Only)
run_cori_smart_camera() {
    [ $(check_file "$INTEGRATION_PATH"; echo $?) -ne 0 ] && exit 1
    echo "📷 CORI SMART CAMERA MODE"
    echo "========================"
    echo "🎯 Features:"
    echo "   📷 Webcam color detection"
    echo "   🧠 Smart database logging"
    echo "   ⚡ No robot movement (camera only)"
    read -p "🚀 Start smart camera mode? [y/N]: " confirm
    [[ ! $confirm =~ ^[Yy]$ ]] && { echo "👋 Cancelled"; exit 0; }
    
    cleanup_processes "smart camera"
    trap 'cleanup_processes "smart camera"; exit 0' SIGINT
    
    start_webcam CAMERA_PID || exit 1
    
    echo "🔍 Verifying camera integration..."
    sleep 2
    
    echo "🧠 Starting CORI smart camera system..."
    cd src/cori_tools/cori_tools/
    
    # Auto-select camera only mode (option 1)
    echo "1" | python3 cori_ignition_integration.py
    
    cleanup_processes "smart camera"
}

# NEW: Run CORI Full Control (Your working mode - Ignition Full)
run_cori_full_control() {
    [ $(check_file "$INTEGRATION_PATH"; echo $?) -ne 0 ] && exit 1
    echo "🤖 CORI FULL CONTROL MODE"
    echo "========================="
    echo "🎯 Features:"
    echo "   🎮 Gazebo simulation"
    echo "   📷 Webcam detection"
    echo "   🤖 Robot head movement"
    echo "   🧠 Unified database"
    read -p "🚀 Start full control mode? [y/N]: " confirm
    [[ ! $confirm =~ ^[Yy]$ ]] && { echo "👋 Cancelled"; exit 0; }
    
    cleanup_processes "full control"
    trap 'cleanup_processes "full control"; exit 0' SIGINT
    
    start_gazebo GAZEBO_PID
    start_webcam CAMERA_PID || { kill $GAZEBO_PID 2>/dev/null; exit 1; }
    
    echo "🔍 Verifying integration..."
    sleep 3
    
    ros2 topic list | grep -q "/camera/color/image_raw" && echo "   ✅ Camera topics found" || echo "   ⚠️ Camera topics missing"
    ros2 topic list | grep -q "/model/cori/joint/head_joint/cmd_pos" && echo "   ✅ Robot topics found" || echo "   ⚠️ Robot topics missing"
    
    echo "🤖 Starting CORI full control system..."
    cd src/cori_tools/cori_tools/
    
    # Auto-select ignition full mode (option 2) - THIS IS WHAT YOU WANT
    echo "2" | python3 cori_ignition_integration.py
    
    cleanup_processes "full control"
}

# NEW: Run CORI Laundry Mode (Camera + Smart Suggestions)
run_cori_laundry_mode() {
    [ $(check_file "$INTEGRATION_PATH"; echo $?) -ne 0 ] && exit 1
    echo "🧺 CORI LAUNDRY SORTING MODE"
    echo "============================"
    echo "🎯 Features:"
    echo "   📷 Webcam detection"
    echo "   🧺 Laundry category suggestions"
    echo "   📚 Learning your preferences"
    read -p "🚀 Start laundry mode? [y/N]: " confirm
    [[ ! $confirm =~ ^[Yy]$ ]] && { echo "👋 Cancelled"; exit 0; }
    
    cleanup_processes "laundry mode"
    trap 'cleanup_processes "laundry mode"; exit 0' SIGINT
    
    start_webcam CAMERA_PID || exit 1
    
    echo "🧺 Starting CORI laundry assistant..."
    cd src/cori_tools/cori_tools/
    
    # Auto-select laundry camera mode (option 3)
    echo "3" | python3 cori_ignition_integration.py
    
    cleanup_processes "laundry mode"
}

# OLD: Run unified integration (kept for backward compatibility)
run_unified_integration() {
    [ $(check_file "$INTEGRATION_PATH"; echo $?) -ne 0 ] && exit 1
    echo "🔗 CORI UNIFIED INTEGRATION SYSTEM"
    echo "=================================="
    echo "🎯 Features:"
    echo "   🎮 Gazebo simulation"
    echo "   📷 Camera detection"
    echo "   🧠 Unified database"
    echo "⚠️  NOTE: You'll need to select a mode after launch"
    read -p "🚀 Start integration? [y/N]: " confirm
    [[ ! $confirm =~ ^[Yy]$ ]] && { echo "👋 Cancelled"; exit 0; }
    cleanup_processes "unified integration"
    trap 'cleanup_processes "unified integration"; exit 0' SIGINT
    start_gazebo GAZEBO_PID
    start_webcam CAMERA_PID || { kill $GAZEBO_PID 2>/dev/null; exit 1; }
    echo "🔍 Verifying integration..."
    ros2 topic list | grep -q "/camera/color/image_raw" || echo "   ⚠️ Camera topic not found"
    ros2 topic list | grep -q "/model/cori/joint/head_joint/cmd_pos" || echo "   ⚠️ Joint topic not found"
    echo "🔗 Starting integration system..."
    cd src/cori_tools/cori_tools/
    python3 cori_ignition_integration.py
    cleanup_processes "unified integration"
}

# Kill all processes
kill_all_processes() {
    cleanup_processes "all ROS"
    echo "📷 Camera devices: $(ls /dev/video* 2>/dev/null | tr '\n' ' ')"
    exit 0
}

# Main execution
main() {
    cd "$WORKSPACE_DIR" || { echo "❌ Failed to navigate to $WORKSPACE_DIR"; exit 1; }
    echo "📁 Current directory: $(pwd)"
    build_workspace
    show_startup_sequence

    # --- Start of Menu Box ---
    local TOTAL_MENU_WIDTH=70
    local menu_inner_width=$((TOTAL_MENU_WIDTH))

    echo "╭"$(printf '─%.0s' $(seq 1 $menu_inner_width))"╮"

    # Menu title with centered alignment
    local menu_title="SELECT A PROGRAM TO RUN:"
    local title_len=${#menu_title}
    local title_pad_left=$(( (menu_inner_width - title_len) / 2 ))
    printf "│%*s%s%*s│\n" $title_pad_left "" "$menu_title" $((menu_inner_width - title_len - title_pad_left)) ""

    # Empty line after title
    printf "│%*s│\n" $menu_inner_width ""

    # Menu items with consistent padding
    local menu_items=(
        "1) 🚀 Full System (Gazebo + Webcam + Color Detection)"
        "2) 🎮 Gazebo Simulation Only"
        "3) 🧺 Laundry Sorting Assistant"
        "4) 📷 Webcam Color Detection"
        "5) 🦾 Manual Control Mode"
    )

    for item in "${menu_items[@]}"; do
        printf "│ %-*s │\n" $((menu_inner_width)) "$item"
    done

    local fusion_exists=$(check_file "$SENSOR_FUSION_PATH" && echo true || echo false)
    if [ "$fusion_exists" = true ]; then
        printf "│ %-*s │\n" $((menu_inner_width)) "6) 🧠 Sensor Fusion Demo"
        printf "│ %-*s │\n" $((menu_inner_width)) "7) 📁 View Spatial Database"
    fi

    local integration_exists=$(check_file "$INTEGRATION_PATH" && echo true || echo false)
    if [ "$integration_exists" = true ]; then
        printf "│ %-*s │\n" $((menu_inner_width)) "8) 📷 CORI Smart Camera"
        printf "│ %-*s │\n" $((menu_inner_width)) "9) 🤖 CORI Full Control"
        printf "│ %-*s │\n" $((menu_inner_width)) "10) 🧺 CORI Laundry Mode"
        printf "│ %-*s │\n" $((menu_inner_width)) "11) 🔗 Unified Integration"
    fi
    
    # Static menu items at the bottom
    printf "│ %-*s │\n" $((menu_inner_width)) "12) 🧹 Kill All ROS Processes"
    printf "│ %-*s │\n" $((menu_inner_width)) "13) 🚪 Exit"

    # Empty line before bottom border
    printf "│%*s│\n" $menu_inner_width ""

    echo "╰"$(printf '─%.0s' $(seq 1 $menu_inner_width))"╯"
    # --- End of Menu Box ---

    read -p "Enter choice [1-13]: " choice
    case $choice in
        1) run_full_system ;;
        2) run_gazebo_only ;;
        3) run_laundry_assistant ;;
        4) run_webcam_color ;;
        5) run_manual_control ;;
        6) [ "$fusion_exists" = true ] && run_sensor_fusion || echo "❌ Invalid choice" ;;
        7) [ "$fusion_exists" = true ] && view_spatial_database || echo "❌ Invalid choice" ;;
        8) [ "$integration_exists" = true ] && run_cori_smart_camera || echo "❌ Invalid choice" ;;
        9) [ "$integration_exists" = true ] && run_cori_full_control || echo "❌ Invalid choice" ;;
        10) [ "$integration_exists" = true ] && run_cori_laundry_mode || echo "❌ Invalid choice" ;;
        11) [ "$integration_exists" = true ] && run_unified_integration || echo "❌ Invalid choice" ;;
        12) kill_all_processes ;;
        13) echo "👋 Exiting..."; exit 0 ;;
        *) echo "❌ Invalid choice"; exit 1 ;;
    esac
    echo "🏁 CORI system ended."
}

main