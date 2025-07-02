#!/bin/bash
# =============================================================================
# CORI Production Demo System
# =============================================================================
# Description: Professional demo launcher for CORI robotic system
# Author: Johnathan Uptegraph
# Version: 2.0
# License: Proprietary
# =============================================================================

set -euo pipefail

# =============================================================================
# CONSTANTS & CONFIGURATION
# =============================================================================

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly LOG_DIR="${WORKSPACE_ROOT}/logs"
readonly TIMESTAMP="$(date '+%Y%m%d_%H%M%S')"
readonly SESSION_LOG="${LOG_DIR}/demo_${TIMESTAMP}.log"

# Demo configuration
readonly CAMERA_DEVICE="/dev/video0"
readonly DEMO_TIMEOUT=300  # 5 minutes default timeout

# Colors for output
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly PURPLE='\033[0;35m'
readonly CYAN='\033[0;36m'
readonly BOLD='\033[1m'
readonly NC='\033[0m'

# Process tracking
declare -a DEMO_PIDS=()

# =============================================================================
# LOGGING & OUTPUT FUNCTIONS
# =============================================================================

log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp="$(date '+%Y-%m-%d %H:%M:%S')"
    
    echo "[$timestamp] [$level] $message" | tee -a "$SESSION_LOG"
}

print_banner() {
    clear
    echo -e "${CYAN}${BOLD}"
    cat << 'EOF'
╔════════════════════════════════════════════════════════════════════════════╗
║                                                                            ║
║     ██████╗ ██████╗ ██████╗ ██╗    ██████╗ ███████╗███╗   ███╗ ██████╗    ║
║    ██╔════╝██╔═══██╗██╔══██╗██║    ██╔══██╗██╔════╝████╗ ████║██╔═══██╗   ║
║    ██║     ██║   ██║██████╔╝██║    ██║  ██║█████╗  ██╔████╔██║██║   ██║   ║
║    ██║     ██║   ██║██╔══██╗██║    ██║  ██║██╔══╝  ██║╚██╔╝██║██║   ██║   ║
║    ╚██████╗╚██████╔╝██║  ██║██║    ██████╔╝███████╗██║ ╚═╝ ██║╚██████╔╝   ║
║     ╚═════╝ ╚═════╝ ╚═╝  ╚═╝╚═╝    ╚═════╝ ╚══════╝╚═╝     ╚═╝ ╚═════╝    ║
║                                                                            ║
║           Cooperative Organizational Robotic Intelligence                  ║
║                     Professional Demo System v2.0                         ║
║                                                                            ║
╚════════════════════════════════════════════════════════════════════════════╝
EOF
    echo -e "${NC}"
    echo -e "${BLUE}Session Log: ${SESSION_LOG}${NC}"
    echo -e "${BLUE}Workspace: ${WORKSPACE_ROOT}${NC}"
    echo ""
}

print_status() {
    local status="$1"
    local message="$2"
    case "$status" in
        "success") echo -e "${GREEN}[✓]${NC} $message" ;;
        "error")   echo -e "${RED}[✗]${NC} $message" ;;
        "warning") echo -e "${YELLOW}[⚠]${NC} $message" ;;
        "info")    echo -e "${BLUE}[ℹ]${NC} $message" ;;
        "step")    echo -e "${PURPLE}[→]${NC} $message" ;;
    esac
}

# =============================================================================
# SYSTEM VALIDATION
# =============================================================================

validate_system() {
    print_status "step" "Validating system requirements"
    
    # Check workspace is built
    if [ ! -d "${WORKSPACE_ROOT}/install" ]; then
        print_status "error" "Workspace not built. Run: ./scripts/cori_build.bash"
        return 1
    fi
    
    # Check ROS environment
    if [ -z "${ROS_DISTRO:-}" ]; then
        print_status "error" "ROS environment not sourced"
        return 1
    fi
    
    # Source workspace
    if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
        # shellcheck source=/dev/null
        source "${WORKSPACE_ROOT}/install/setup.bash"
        print_status "success" "Workspace environment loaded"
    else
        print_status "error" "Workspace setup file not found"
        return 1
    fi
    
    # Check camera device (optional)
    if [ -e "$CAMERA_DEVICE" ]; then
        print_status "success" "Camera device available: $CAMERA_DEVICE"
    else
        print_status "warning" "Camera device not found: $CAMERA_DEVICE"
        print_status "info" "Some demos may not work without camera"
    fi
    
    # Check required packages
    local required_packages=("cori_description" "cori_vision" "cori_gui" "cori_simulation")
    for package in "${required_packages[@]}"; do
        if ros2 pkg list | grep -q "^${package}$"; then
            print_status "success" "Package available: $package"
        else
            print_status "error" "Package missing: $package"
            return 1
        fi
    done
    
    print_status "success" "System validation complete"
    return 0
}

# =============================================================================
# PROCESS MANAGEMENT
# =============================================================================

cleanup_processes() {
    if [ ${#DEMO_PIDS[@]} -gt 0 ]; then
        print_status "step" "Cleaning up demo processes"
        for pid in "${DEMO_PIDS[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                log "INFO" "Terminating process $pid"
                kill "$pid" 2>/dev/null || true
            fi
        done
        
        # Wait for graceful shutdown
        sleep 2
        
        # Force kill if necessary
        for pid in "${DEMO_PIDS[@]}"; do
            if kill -0 "$pid" 2>/dev/null; then
                log "WARN" "Force killing process $pid"
                kill -9 "$pid" 2>/dev/null || true
            fi
        done
        
        DEMO_PIDS=()
    fi
    
    # Kill any remaining ROS processes
    pkill -f "ros2" 2>/dev/null || true
    pkill -f "gazebo" 2>/dev/null || true
    pkill -f "ign" 2>/dev/null || true
    
    print_status "success" "Process cleanup complete"
}

start_background_process() {
    local name="$1"
    local command="$2"
    
    print_status "step" "Starting $name"
    eval "$command" &
    local pid=$!
    DEMO_PIDS+=("$pid")
    
    # Give process time to start
    sleep 2
    
    if kill -0 "$pid" 2>/dev/null; then
        print_status "success" "$name started (PID: $pid)"
        return 0
    else
        print_status "error" "$name failed to start"
        return 1
    fi
}

wait_for_user() {
    echo ""
    echo -e "${YELLOW}Press ENTER to continue or Ctrl+C to exit...${NC}"
    read -r
}

# =============================================================================
# DEMO SCENARIOS
# =============================================================================

demo_robot_visualization() {
    print_status "step" "Starting Robot Visualization Demo"
    echo ""
    echo "This demo shows CORI's robot model in RViz2"
    echo "You can manipulate joints and view the 3D model"
    wait_for_user
    
    cleanup_processes
    
    if start_background_process "RViz2 Visualization" "ros2 launch cori_description display_rviz.launch.py"; then
        print_status "success" "Robot visualization demo running"
        echo ""
        echo -e "${GREEN}Demo Instructions:${NC}"
        echo "• RViz2 should open showing CORI's 3D model"
        echo "• Use the joint state publisher to move joints"
        echo "• Explore different viewing angles"
        echo ""
        echo -e "${YELLOW}Press ENTER when done viewing...${NC}"
        read -r
    else
        print_status "error" "Failed to start robot visualization demo"
        return 1
    fi
}

demo_gazebo_simulation() {
    print_status "step" "Starting Gazebo Simulation Demo"
    echo ""
    echo "This demo launches CORI in a physics simulation environment"
    echo "You can see CORI in a virtual laundry room with objects"
    wait_for_user
    
    cleanup_processes
    
    if start_background_process "Gazebo Simulation" "ros2 launch cori_description spawn_cori_ignition.launch.py"; then
        print_status "success" "Gazebo simulation demo running"
        echo ""
        echo -e "${GREEN}Demo Instructions:${NC}"
        echo "• Gazebo will open with CORI in a laundry environment"
        echo "• You can move the camera around to explore"
        echo "• Watch CORI's joints and physics interactions"
        echo "• Note the laundry items and sorting bins"
        echo ""
        echo -e "${YELLOW}Press ENTER when done exploring...${NC}"
        read -r
    else
        print_status "error" "Failed to start Gazebo simulation demo"
        return 1
    fi
}

demo_vision_system() {
    print_status "step" "Starting Computer Vision Demo"
    echo ""
    echo "This demo shows CORI's color detection and object recognition"
    
    if [ ! -e "$CAMERA_DEVICE" ]; then
        print_status "warning" "Camera not available - skipping vision demo"
        return 0
    fi
    
    echo "Make sure you have colored objects to show the camera"
    wait_for_user
    
    cleanup_processes
    
    if start_background_process "Camera Node" "ros2 launch cori_vision laundry_color_detector.launch.py"; then
        sleep 3
        if start_background_process "Color Display" "ros2 run cori_gui color_display"; then
            print_status "success" "Vision system demo running"
            echo ""
            echo -e "${GREEN}Demo Instructions:${NC}"
            echo "• Hold colored objects in front of the camera"
            echo "• Watch the color detection and classification"
            echo "• Try different colored clothing items"
            echo "• Observe how CORI categorizes: lights, darks, colors"
            echo ""
            echo -e "${YELLOW}Press ENTER when done testing...${NC}"
            read -r
        else
            print_status "error" "Failed to start color display"
            return 1
        fi
    else
        print_status "error" "Failed to start camera node"
        return 1
    fi
}

demo_integrated_system() {
    print_status "step" "Starting Integrated System Demo"
    echo ""
    echo "This demo runs the full CORI system with simulation and vision"
    echo "It demonstrates the complete perception-to-action pipeline"
    
    if [ ! -e "$CAMERA_DEVICE" ]; then
        print_status "warning" "Camera not available - running simulation only"
    fi
    
    wait_for_user
    
    cleanup_processes
    
    # Start Gazebo simulation
    if start_background_process "Gazebo Simulation" "ros2 launch cori_description spawn_cori_ignition.launch.py"; then
        sleep 5
        
        # Start vision system if camera available
        if [ -e "$CAMERA_DEVICE" ]; then
            if start_background_process "Vision System" "ros2 launch cori_vision laundry_color_detector.launch.py"; then
                sleep 3
                start_background_process "System Integration" "ros2 run cori_tools ignition_integration"
            fi
        fi
        
        print_status "success" "Integrated system demo running"
        echo ""
        echo -e "${GREEN}Demo Instructions:${NC}"
        echo "• Gazebo shows CORI in the laundry environment"
        echo "• Vision system processes camera input (if available)"
        echo "• Watch for coordinated behavior between perception and simulation"
        echo "• This represents the full CORI experience"
        echo ""
        echo -e "${YELLOW}Press ENTER when done with integrated demo...${NC}"
        read -r
    else
        print_status "error" "Failed to start integrated system demo"
        return 1
    fi
}

# =============================================================================
# DEMO MENU
# =============================================================================

show_demo_menu() {
    echo -e "${BOLD}Available Demos:${NC}"
    echo ""
    echo -e "${CYAN}1)${NC} Robot Visualization     - View CORI's 3D model in RViz2"
    echo -e "${CYAN}2)${NC} Gazebo Simulation      - See CORI in physics simulation"
    echo -e "${CYAN}3)${NC} Computer Vision        - Test color detection system"
    echo -e "${CYAN}4)${NC} Integrated System      - Full CORI experience"
    echo -e "${CYAN}5)${NC} System Status          - Check system health"
    echo -e "${CYAN}6)${NC} Exit                   - Clean shutdown"
    echo ""
}

show_system_status() {
    print_status "step" "Checking system status"
    echo ""
    
    # ROS environment
    echo -e "${BOLD}ROS Environment:${NC}"
    echo "  Distribution: $ROS_DISTRO"
    echo "  Domain ID: ${ROS_DOMAIN_ID:-0}"
    echo ""
    
    # Workspace packages
    echo -e "${BOLD}CORI Packages:${NC}"
    ros2 pkg list | grep "^cori_" | sed 's/^/  /'
    echo ""
    
    # Hardware status
    echo -e "${BOLD}Hardware Status:${NC}"
    if [ -e "$CAMERA_DEVICE" ]; then
        echo -e "  Camera: ${GREEN}Available${NC} ($CAMERA_DEVICE)"
    else
        echo -e "  Camera: ${RED}Not Available${NC} ($CAMERA_DEVICE)"
    fi
    
    # Check GPU
    if command -v nvidia-smi >/dev/null 2>&1; then
        echo -e "  GPU: ${GREEN}NVIDIA Available${NC}"
    else
        echo -e "  GPU: ${YELLOW}No NVIDIA GPU${NC}"
    fi
    echo ""
    
    # Recent logs
    echo -e "${BOLD}Recent Activity:${NC}"
    if [ -f "$SESSION_LOG" ]; then
        tail -n 5 "$SESSION_LOG" | sed 's/^/  /'
    else
        echo "  No session activity yet"
    fi
    echo ""
}

# =============================================================================
# MAIN EXECUTION
# =============================================================================

cleanup_on_exit() {
    echo ""
    print_status "step" "Shutting down demo system"
    cleanup_processes
    log "INFO" "Demo session ended"
    print_status "success" "Thank you for using CORI Demo System!"
    exit 0
}

main() {
    trap cleanup_on_exit EXIT INT TERM
    
    # Create log directory
    mkdir -p "$LOG_DIR"
    log "INFO" "Demo session started"
    
    # Initialize
    print_banner
    
    # Validate system
    if ! validate_system; then
        print_status "error" "System validation failed"
        exit 1
    fi
    
    # Main demo loop
    while true; do
        echo ""
        show_demo_menu
        echo -n "Select demo (1-6): "
        read -r choice
        
        case "$choice" in
            1) demo_robot_visualization ;;
            2) demo_gazebo_simulation ;;
            3) demo_vision_system ;;
            4) demo_integrated_system ;;
            5) show_system_status ;;
            6) break ;;
            *) print_status "error" "Invalid choice. Please select 1-6." ;;
        esac
        
        cleanup_processes
    done
}

# Execute main function if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi