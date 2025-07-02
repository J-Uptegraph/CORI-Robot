#!/bin/bash
# CORI Robot Build and Run Script - Cooperative Organizational Robotic Intelligence
# Description: Unified build and execution for CORI's laundry sorting system
# Constants
WORKSPACE_DIR="/home/juptegraph/Workspaces/Robotics/Projects/CORI/cori_ws"
INTEGRATION_PATH="src/cori_tools/cori_tools/cori_ignition_integration.py"
WORLD_FILE="src/cori_description/worlds/laundry_world.sdf"
URDF_FILE="src/cori_description/urdf/cori.urdf.xacro"

    # Function to display a loading bar
    show_loading_bar() {
        local duration=$1
        local bar_length=65
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
        echo -e "\n🤖 Initializing C.O.R.I. system..."
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
            "    ╚═════╝╚═╝ ╚═════╝ ╚═╝╚═╝  ╚═╝╚═╝╚═╝ ╚═╝"
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
        pkill -f "cori_vision" 2>/dev/null || true
        pkill -f "cori_gui" 2>/dev/null || true
        pkill -f "cori_simulation" 2>/dev/null || true
        pkill -f "cori_tools" 2>/dev/null || true
        pkill -f "spatial_database" 2>/dev/null || true
        pkill -f "cori_ignition_integration" 2>/dev/null || true
        sleep 3
        echo "✅ $mode processes stopped"
    }

# Build workspace
build_workspace() {
    echo "🧹 Cleaning previous build..."
    rm -rf build/ devel/ install/
    echo "🔨 Building workspace..."
    colcon build
    [ $? -eq 0 ] && echo "✅ Build successful!" || { echo "❌ Build failed!"; exit 1; }
    
    # Fix ROS 2 executable paths for cori_vision
    echo "🔧 Fixing ROS 2 executable paths..."
    if [ -d "install/cori_vision/bin" ] && [ -d "install/cori_vision/lib/cori_vision" ]; then
        cp install/cori_vision/bin/* install/cori_vision/lib/cori_vision/ 2>/dev/null || true
        echo "✅ Fixed cori_vision executable paths"
    fi
    
    echo "📦 Sourcing workspace..."
    source install/setup.bash
    
    # Add to bashrc for system-wide availability
    echo "🌐 Setting up system-wide access..."
    local setup_line="source $WORKSPACE_DIR/install/setup.bash"
    if ! grep -q "$setup_line" ~/.bashrc; then
        echo "# CORI Workspace Setup" >> ~/.bashrc
        echo "$setup_line" >> ~/.bashrc
        echo "✅ Added CORI workspace to ~/.bashrc"
    else
        echo "✅ CORI workspace already in ~/.bashrc"
    fi
}

# Start Gazebo simulation
start_gazebo() {
    local pid_var="$1"
    echo "🎮 Starting Gazebo simulation..."
    source install/setup.bash
    ros2 launch cori_description spawn_cori_ignition.launch.py &
    eval "$pid_var=\$!"
    sleep 8
    [ -z "$(ps -p ${!pid_var} -o pid=)" ] && { echo "❌ Gazebo failed to start!"; exit 1; }
}

# Start webcam
start_webcam() {
    local pid_var="$1"
    echo "📷 Starting webcam..."
    source install/setup.bash
    ros2 launch cori_vision laundry_color_detector.launch.py &
    eval "$pid_var=\$!"
    sleep 5
    [ -z "$(ps -p ${!pid_var} -o pid=)" ] && { echo "❌ Webcam failed to start!"; return 1; }
    return 0
}

# Run system test
run_system_test() {
    cleanup_processes "system test"
    trap 'cleanup_processes "system test"; exit 0' SIGINT
    start_gazebo GAZEBO_PID
    start_webcam WEBCAM_PID || { kill $GAZEBO_PID 2>/dev/null; exit 1; }
    echo "🔗 Starting color detection bridge..."
    ros2 run cori_vision simple_color_detector &
    BRIDGE_PID=$!
    sleep 3
    echo "🎨 Starting color display..."
    echo "👋 Hold colored objects in front of your webcam!"
    echo "📺 Ensure webcam permissions are enabled"
    ros2 run cori_gui color_display
    cleanup_processes "system test"
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
    ros2 run cori_vision simple_color_detector &
    BRIDGE_PID=$!
    sleep 2
    echo "👋 Hold colored objects in front of your webcam!"
    ros2 run cori_gui color_display
    cleanup_processes "webcam color detection"
}


# Run laundry assistant
run_laundry_assistant() {
    local script_path="src/cori_simulation/cori_simulation/cori_simulator.py"
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
    cd src/cori_simulation/cori_simulation/
    echo "🚀 Launching Laundry Assistant..."
    echo "🎯 TIPS: Start with 'red shirt', 'blue jeans'; type 'quit' to stop"
    python3 cori_simulator.py
}

# Run full system
run_full_system() {
    [ $(check_file "$INTEGRATION_PATH"; echo $?) -ne 0 ] && exit 1
    echo "🔗 CORI UNIFIED INTEGRATION SYSTEM"
    echo "=================================="
    echo "🎯 Features:"
    echo "   🎮 Gazebo simulation"
    echo "   📷 Camera detection with head movement"
    echo "   🧠 Unified database"
    echo "   🤖 Automatic mode selection (Ignition Full)"
    read -p "🚀 Start integration? [y/N]: " confirm
    [[ ! $confirm =~ ^[Yy]$ ]] && { echo "👋 Cancelled"; exit 0; }
    cleanup_processes "unified integration"
    trap 'cleanup_processes "unified integration"; exit 0' SIGINT
    start_gazebo GAZEBO_PID
    echo "🔍 Verifying Gazebo startup..."
    sleep 3
    ros2 topic list | grep -q "/model/cori/joint/head_joint/cmd_pos" || echo "   ⚠️ Head joint topic not found"
    echo "🔗 Starting CORI integration system..."
    cd src/cori_tools/cori_tools/
    # Auto-select Ignition Full mode (option 2) via command line argument
    python3 cori_ignition_integration.py 2
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
    local menu_padding=2  # Left padding for menu items

    echo "╭"$(printf '─%.0s' $(seq 1 $menu_inner_width))"╮"

    # Menu title with centered alignment
    local menu_title="SELECT A PROGRAM:"
    local title_len=${#menu_title}
    local title_pad_left=$(( (menu_inner_width - title_len) / 2 ))
    printf "│%*s%s%*s│\n" $title_pad_left "" "$menu_title" $((menu_inner_width - title_len - title_pad_left)) ""

    # Empty line after title
    printf "│%*s│\n" $menu_inner_width ""

    local menu_items=(
        "1) 🚀 Full System"
        "2) 🎮 Gazebo Simulation"
        "3) 🧺 Laundry Sorting Assistant"
        "4) 📷 Webcam Color Detection"
        "5) 🦾 Manual Robot Control"
        "6) 🧹 Kill All ROS Processes"
        "7) 🔗 System Test"
        "8) 🚪 Exit"
    )

    for item in "${menu_items[@]}"; do
        printf "│ %-*s │\n" $((menu_inner_width)) "$item"
    done

    # Empty line before bottom border
    printf "│%*s│\n" $menu_inner_width ""

    echo "╰"$(printf '─%.0s' $(seq 1 $menu_inner_width))"╯"
    # --- End of Menu Box ---

    read -p "Enter choice [1-8]: " choice
    case $choice in
        1) run_full_system ;;
        2) run_gazebo_only ;;
        3) run_laundry_assistant ;;
        4) run_webcam_color ;;
        5) run_manual_control ;;
        6) kill_all_processes ;;
        7) run_system_test ;;
        8) echo "👋 Exiting..."; exit 0 ;;
        *) echo "❌ Invalid choice"; exit 1 ;;
    esac
    echo "🏁 CORI system ended."
}

main