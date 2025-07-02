#!/bin/bash
# =============================================================================
# CORI Environment Setup & Validation
# =============================================================================
# Description: Production environment setup and dependency validation
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
readonly SETUP_LOG="${LOG_DIR}/setup_${TIMESTAMP}.log"

# System requirements
readonly REQUIRED_ROS_DISTRO="humble"
readonly REQUIRED_UBUNTU_VERSION="22.04"
readonly MIN_PYTHON_VERSION="3.8"

# Colors for output
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly PURPLE='\033[0;35m'
readonly CYAN='\033[0;36m'
readonly BOLD='\033[1m'
readonly NC='\033[0m'

# =============================================================================
# LOGGING & OUTPUT
# =============================================================================

log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp="$(date '+%Y-%m-%d %H:%M:%S')"
    
    echo "[$timestamp] [$level] $message" | tee -a "$SETUP_LOG"
}

print_header() {
    echo -e "${CYAN}${BOLD}"
    echo "============================================================================="
    echo "  CORI Environment Setup & Validation"
    echo "  Ensuring optimal conditions for robotic development"
    echo "============================================================================="
    echo -e "${NC}"
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

print_section() {
    echo ""
    echo -e "${BOLD}$1${NC}"
    echo "$(printf '─%.0s' $(seq 1 ${#1}))"
}

# =============================================================================
# SYSTEM VALIDATION
# =============================================================================

check_operating_system() {
    print_section "Operating System Validation"
    
    if [ ! -f /etc/os-release ]; then
        print_status "error" "Cannot determine operating system"
        return 1
    fi
    
    # shellcheck source=/dev/null
    source /etc/os-release
    
    print_status "info" "OS: $PRETTY_NAME"
    print_status "info" "Kernel: $(uname -r)"
    print_status "info" "Architecture: $(uname -m)"
    
    if [ "$ID" = "ubuntu" ]; then
        if [ "$VERSION_ID" = "$REQUIRED_UBUNTU_VERSION" ]; then
            print_status "success" "Ubuntu version is compatible"
        else
            print_status "warning" "Expected Ubuntu $REQUIRED_UBUNTU_VERSION, found $VERSION_ID"
            print_status "info" "CORI may work but is only tested on Ubuntu $REQUIRED_UBUNTU_VERSION"
        fi
    else
        print_status "warning" "Non-Ubuntu system detected: $PRETTY_NAME"
        print_status "info" "CORI is designed for Ubuntu $REQUIRED_UBUNTU_VERSION"
    fi
    
    return 0
}

check_python_environment() {
    print_section "Python Environment Validation"
    
    if ! command -v python3 >/dev/null 2>&1; then
        print_status "error" "Python 3 not found"
        return 1
    fi
    
    local python_version
    python_version=$(python3 -c "import sys; print('.'.join(map(str, sys.version_info[:2])))")
    print_status "info" "Python version: $python_version"
    
    # Check minimum version
    if python3 -c "import sys; sys.exit(0 if sys.version_info >= (3, 8) else 1)"; then
        print_status "success" "Python version is compatible"
    else
        print_status "error" "Python $MIN_PYTHON_VERSION or higher required"
        return 1
    fi
    
    # Check pip
    if command -v pip3 >/dev/null 2>&1; then
        print_status "success" "pip3 available"
        print_status "info" "pip version: $(pip3 --version | cut -d' ' -f2)"
    else
        print_status "warning" "pip3 not found - may cause issues with Python packages"
    fi
    
    return 0
}

check_ros_environment() {
    print_section "ROS 2 Environment Validation"
    
    # Check if ROS is sourced
    if [ -z "${ROS_DISTRO:-}" ]; then
        print_status "warning" "ROS environment not sourced"
        
        # Try to source it
        if [ -f "/opt/ros/$REQUIRED_ROS_DISTRO/setup.bash" ]; then
            print_status "info" "Attempting to source ROS $REQUIRED_ROS_DISTRO"
            # shellcheck source=/dev/null
            source "/opt/ros/$REQUIRED_ROS_DISTRO/setup.bash"
            export ROS_DISTRO="$REQUIRED_ROS_DISTRO"
        else
            print_status "error" "ROS $REQUIRED_ROS_DISTRO not installed at /opt/ros/$REQUIRED_ROS_DISTRO"
            print_status "info" "Install with: sudo apt install ros-$REQUIRED_ROS_DISTRO-desktop"
            return 1
        fi
    fi
    
    print_status "info" "ROS Distribution: $ROS_DISTRO"
    print_status "info" "ROS Domain ID: ${ROS_DOMAIN_ID:-0}"
    
    if [ "$ROS_DISTRO" = "$REQUIRED_ROS_DISTRO" ]; then
        print_status "success" "ROS distribution is compatible"
    else
        print_status "warning" "Expected ROS $REQUIRED_ROS_DISTRO, found $ROS_DISTRO"
    fi
    
    # Check essential ROS commands
    local ros_commands=("ros2" "colcon" "xacro")
    for cmd in "${ros_commands[@]}"; do
        if command -v "$cmd" >/dev/null 2>&1; then
            print_status "success" "$cmd available"
        else
            print_status "error" "$cmd not found"
            return 1
        fi
    done
    
    return 0
}

check_robotics_dependencies() {
    print_section "Robotics Dependencies Validation"
    
    # Check Gazebo/Ignition
    if command -v ign >/dev/null 2>&1; then
        local ign_version
        ign_version=$(ign gazebo --version 2>/dev/null | head -1 || echo "Unknown")
        print_status "success" "Ignition Gazebo available: $ign_version"
    else
        print_status "warning" "Ignition Gazebo not found"
        print_status "info" "Install with: sudo apt install ignition-gazebo6"
    fi
    
    # Check OpenCV
    if python3 -c "import cv2" 2>/dev/null; then
        local opencv_version
        opencv_version=$(python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null || echo "Unknown")
        print_status "success" "OpenCV available: $opencv_version"
    else
        print_status "error" "OpenCV not available"
        print_status "info" "Install with: sudo apt install python3-opencv"
        return 1
    fi
    
    # Check camera support
    if [ -e "/dev/video0" ]; then
        print_status "success" "Camera device available: /dev/video0"
        
        # Check v4l2 tools
        if command -v v4l2-ctl >/dev/null 2>&1; then
            print_status "success" "v4l2-utils available"
        else
            print_status "warning" "v4l2-utils not found (optional for camera debugging)"
        fi
    else
        print_status "warning" "No camera device found at /dev/video0"
        print_status "info" "Vision demos will not work without a camera"
    fi
    
    return 0
}

check_workspace_structure() {
    print_section "Workspace Structure Validation"
    
    # Check if we're in the right directory
    if [ ! -f "$WORKSPACE_ROOT/current_dependencies.txt" ]; then
        print_status "error" "Not in CORI workspace root"
        print_status "info" "Expected to find: current_dependencies.txt"
        return 1
    fi
    
    print_status "success" "CORI workspace detected"
    print_status "info" "Workspace root: $WORKSPACE_ROOT"
    
    # Check essential directories
    local essential_dirs=("src" "scripts")
    for dir in "${essential_dirs[@]}"; do
        if [ -d "$WORKSPACE_ROOT/$dir" ]; then
            print_status "success" "$dir/ directory exists"
        else
            print_status "error" "$dir/ directory missing"
            return 1
        fi
    done
    
    # Check package structure
    local expected_packages=("cori_core" "cori_description" "cori_vision")
    for package in "${expected_packages[@]}"; do
        if [ -d "$WORKSPACE_ROOT/src/$package" ]; then
            print_status "success" "Package $package exists"
        else
            print_status "error" "Package $package missing"
            return 1
        fi
    done
    
    # Check if workspace is built
    if [ -d "$WORKSPACE_ROOT/install" ]; then
        print_status "info" "Workspace appears to be built"
    else
        print_status "warning" "Workspace not built yet"
        print_status "info" "Run: ./scripts/cori_build.bash"
    fi
    
    return 0
}

check_network_connectivity() {
    print_section "Network Connectivity Validation"
    
    # Check internet connectivity
    if ping -c 1 8.8.8.8 >/dev/null 2>&1; then
        print_status "success" "Internet connectivity available"
    else
        print_status "warning" "No internet connectivity"
        print_status "info" "Package updates and downloads may fail"
    fi
    
    # Check for ROS network issues
    local ros_localhost="${ROS_LOCALHOST_ONLY:-}"
    if [ "$ros_localhost" = "1" ]; then
        print_status "info" "ROS configured for localhost only"
    else
        print_status "info" "ROS configured for network discovery"
    fi
    
    return 0
}

# =============================================================================
# FIXES & RECOMMENDATIONS
# =============================================================================

generate_setup_script() {
    print_section "Environment Setup Script Generation"
    
    local setup_script="$WORKSPACE_ROOT/setup_cori_env.bash"
    
    cat > "$setup_script" << 'EOF'
#!/bin/bash
# CORI Environment Setup Script
# Auto-generated by setup_environment.bash

# Source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
    echo "✓ ROS 2 Humble sourced"
else
    echo "✗ ROS 2 Humble not found"
    exit 1
fi

# Source CORI workspace (if built)
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
    echo "✓ CORI workspace sourced"
else
    echo "⚠ CORI workspace not built - run ./scripts/cori_build.bash first"
fi

# Set optimal ROS configuration
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RCUTILS_LOGGING_SEVERITY=INFO

echo "🤖 CORI environment ready!"
echo "Available commands:"
echo "  ./scripts/cori_build.bash  - Build the workspace"
echo "  ./scripts/cori_demo.bash   - Run demos"

EOF
    
    chmod +x "$setup_script"
    print_status "success" "Setup script generated: setup_cori_env.bash"
    print_status "info" "Use: source ./setup_cori_env.bash"
    
    return 0
}

show_recommendations() {
    print_section "Recommendations & Next Steps"
    
    echo -e "${BOLD}To get started with CORI:${NC}"
    echo ""
    echo "1. Source the environment:"
    echo -e "   ${CYAN}source ./setup_cori_env.bash${NC}"
    echo ""
    echo "2. Build the workspace:"
    echo -e "   ${CYAN}./scripts/cori_build.bash${NC}"
    echo ""
    echo "3. Run demos:"
    echo -e "   ${CYAN}./scripts/cori_demo.bash${NC}"
    echo ""
    
    echo -e "${BOLD}For development:${NC}"
    echo ""
    echo "• Always source the environment before building/running"
    echo "• Check logs in the logs/ directory for debugging"
    echo "• Use the demo system to verify functionality"
    echo ""
    
    if [ ! -e "/dev/video0" ]; then
        echo -e "${BOLD}Camera Setup (Optional):${NC}"
        echo ""
        echo "• Connect a USB camera for vision demos"
        echo "• Install v4l2-utils: sudo apt install v4l2-utils"
        echo "• Test camera: v4l2-ctl --list-devices"
        echo ""
    fi
}

# =============================================================================
# MAIN EXECUTION
# =============================================================================

main() {
    mkdir -p "$LOG_DIR"
    log "INFO" "Environment setup started"
    
    print_header
    
    local validation_failed=false
    
    # Run all validation checks
    check_operating_system || validation_failed=true
    check_python_environment || validation_failed=true
    check_ros_environment || validation_failed=true
    check_robotics_dependencies || validation_failed=true
    check_workspace_structure || validation_failed=true
    check_network_connectivity || validation_failed=true
    
    # Generate setup script
    generate_setup_script
    
    # Show final status
    echo ""
    if [ "$validation_failed" = "true" ]; then
        print_status "warning" "Environment validation completed with issues"
        print_status "info" "Some functionality may be limited"
    else
        print_status "success" "Environment validation passed!"
        print_status "info" "CORI is ready for development"
    fi
    
    show_recommendations
    
    log "INFO" "Environment setup completed"
}

# Execute main function if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi