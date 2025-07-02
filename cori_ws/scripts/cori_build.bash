#!/bin/bash
# =============================================================================
# CORI Production Build System
# =============================================================================
# Description: Production-grade build system for CORI robotic workspace
# Author: Johnathan Uptegraph
# Version: 2.0
# License: Proprietary
# =============================================================================

set -euo pipefail  # Exit on error, undefined variables, pipe failures

# =============================================================================
# CONSTANTS & CONFIGURATION
# =============================================================================

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly LOG_DIR="${WORKSPACE_ROOT}/logs"
readonly TIMESTAMP="$(date '+%Y%m%d_%H%M%S')"
readonly LOG_FILE="${LOG_DIR}/build_${TIMESTAMP}.log"

# Build configuration
readonly ROS_DISTRO="humble"
readonly REQUIRED_PACKAGES=(
    "cori_core"
    "cori_control" 
    "cori_description"
    "cori_gui"
    "cori_simulation"
    "cori_tools"
    "cori_vision"
)

# Colors for output
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly PURPLE='\033[0;35m'
readonly CYAN='\033[0;36m'
readonly NC='\033[0m' # No Color

# =============================================================================
# LOGGING & OUTPUT FUNCTIONS
# =============================================================================

log() {
    local level="$1"
    shift
    local message="$*"
    local timestamp="$(date '+%Y-%m-%d %H:%M:%S')"
    
    echo "[$timestamp] [$level] $message" | tee -a "$LOG_FILE"
}

log_info() { log "INFO" "$@"; }
log_warn() { log "WARN" "$@"; }
log_error() { log "ERROR" "$@"; }
log_success() { log "SUCCESS" "$@"; }

print_header() {
    echo -e "${CYAN}"
    echo "============================================================================="
    echo "  CORI Production Build System v2.0"
    echo "  Cooperative Organizational Robotic Intelligence"
    echo "============================================================================="
    echo -e "${NC}"
}

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[⚠]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

check_command() {
    local cmd="$1"
    if ! command -v "$cmd" >/dev/null 2>&1; then
        log_error "Required command '$cmd' not found"
        return 1
    fi
    return 0
}

create_directories() {
    mkdir -p "$LOG_DIR"
    log_info "Created log directory: $LOG_DIR"
}

cleanup_on_exit() {
    local exit_code=$?
    if [ $exit_code -ne 0 ]; then
        print_error "Build failed with exit code $exit_code"
        log_error "Build process failed"
    fi
    exit $exit_code
}

# =============================================================================
# VALIDATION FUNCTIONS
# =============================================================================

validate_environment() {
    print_step "Validating build environment"
    
    # Check if we're in the correct directory
    if [ ! -f "${WORKSPACE_ROOT}/current_dependencies.txt" ]; then
        log_error "Not in CORI workspace root directory"
        return 1
    fi
    
    # Check ROS 2 environment
    if [ -z "${ROS_DISTRO:-}" ]; then
        log_error "ROS_DISTRO not set. Please source ROS 2 setup"
        return 1
    fi
    
    if [ "$ROS_DISTRO" != "humble" ]; then
        log_warn "Expected ROS Humble, found: $ROS_DISTRO"
    fi
    
    # Check required commands
    local required_commands=("colcon" "ros2" "python3" "xacro")
    for cmd in "${required_commands[@]}"; do
        if ! check_command "$cmd"; then
            return 1
        fi
    done
    
    print_success "Environment validation passed"
    return 0
}

validate_source_packages() {
    print_step "Validating source packages"
    
    local missing_packages=()
    for package in "${REQUIRED_PACKAGES[@]}"; do
        if [ ! -d "${WORKSPACE_ROOT}/src/${package}" ]; then
            missing_packages+=("$package")
        fi
    done
    
    if [ ${#missing_packages[@]} -gt 0 ]; then
        log_error "Missing packages: ${missing_packages[*]}"
        return 1
    fi
    
    # Check package.xml files
    for package in "${REQUIRED_PACKAGES[@]}"; do
        local package_xml="${WORKSPACE_ROOT}/src/${package}/package.xml"
        if [ ! -f "$package_xml" ]; then
            log_error "Missing package.xml for $package"
            return 1
        fi
    done
    
    print_success "Source package validation passed"
    return 0
}

# =============================================================================
# BUILD FUNCTIONS
# =============================================================================

clean_workspace() {
    print_step "Cleaning workspace"
    
    local clean_dirs=("build" "install" "log")
    for dir in "${clean_dirs[@]}"; do
        if [ -d "${WORKSPACE_ROOT}/${dir}" ]; then
            log_info "Removing ${dir} directory"
            rm -rf "${WORKSPACE_ROOT}/${dir}"
        fi
    done
    
    print_success "Workspace cleaned"
}

build_packages() {
    print_step "Building CORI packages"
    
    cd "$WORKSPACE_ROOT"
    
    # Source ROS 2 environment
    if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
        # shellcheck source=/dev/null
        source "/opt/ros/$ROS_DISTRO/setup.bash"
        log_info "Sourced ROS $ROS_DISTRO environment"
    else
        log_error "ROS $ROS_DISTRO setup file not found"
        return 1
    fi
    
    # Build with dependency order
    log_info "Building core package first"
    if ! colcon build --packages-select cori_core --cmake-args -DCMAKE_BUILD_TYPE=Release; then
        log_error "Failed to build cori_core"
        return 1
    fi
    
    log_info "Building remaining packages"
    local remaining_packages=()
    for pkg in "${REQUIRED_PACKAGES[@]}"; do
        if [ "$pkg" != "cori_core" ]; then
            remaining_packages+=("$pkg")
        fi
    done
    
    if ! colcon build --packages-select "${remaining_packages[@]}" --cmake-args -DCMAKE_BUILD_TYPE=Release; then
        log_error "Failed to build packages: ${remaining_packages[*]}"
        return 1
    fi
    
    print_success "All packages built successfully"
}

verify_build() {
    print_step "Verifying build artifacts"
    
    # Check install directory
    if [ ! -d "${WORKSPACE_ROOT}/install" ]; then
        log_error "Install directory not found"
        return 1
    fi
    
    # Check each package installation
    for package in "${REQUIRED_PACKAGES[@]}"; do
        local install_dir="${WORKSPACE_ROOT}/install/${package}"
        if [ ! -d "$install_dir" ]; then
            log_error "Package $package not installed"
            return 1
        fi
        
        # Check for package.xml in share directory
        local package_share="${install_dir}/share/${package}/package.xml"
        if [ ! -f "$package_share" ]; then
            log_warn "Package metadata missing for $package"
        fi
    done
    
    print_success "Build verification passed"
}

test_critical_files() {
    print_step "Testing critical file access"
    
    # Source the workspace
    if [ -f "${WORKSPACE_ROOT}/install/setup.bash" ]; then
        # shellcheck source=/dev/null
        source "${WORKSPACE_ROOT}/install/setup.bash"
    else
        log_error "Workspace setup file not found"
        return 1
    fi
    
    # Test URDF processing
    local urdf_file="${WORKSPACE_ROOT}/install/cori_description/share/cori_description/urdf/cori.urdf.xacro"
    if [ -f "$urdf_file" ]; then
        if xacro "$urdf_file" > /tmp/cori_test.urdf 2>/dev/null; then
            print_success "URDF processing test passed"
            rm -f /tmp/cori_test.urdf
        else
            log_error "URDF processing test failed"
            return 1
        fi
    else
        log_error "URDF file not found: $urdf_file"
        return 1
    fi
    
    # Test package discovery
    local missing_packages=()
    for package in "${REQUIRED_PACKAGES[@]}"; do
        if ! ros2 pkg list | grep -q "^${package}$"; then
            missing_packages+=("$package")
        fi
    done
    
    if [ ${#missing_packages[@]} -gt 0 ]; then
        log_error "Packages not discoverable: ${missing_packages[*]}"
        return 1
    fi
    
    print_success "Critical file tests passed"
}

# =============================================================================
# MAIN EXECUTION
# =============================================================================

main() {
    trap cleanup_on_exit EXIT
    
    print_header
    
    # Initialize
    create_directories
    log_info "Starting CORI build process"
    log_info "Workspace: $WORKSPACE_ROOT"
    log_info "Log file: $LOG_FILE"
    
    # Validation phase
    validate_environment || exit 1
    validate_source_packages || exit 1
    
    # Build phase
    clean_workspace || exit 1
    build_packages || exit 1
    
    # Verification phase
    verify_build || exit 1
    test_critical_files || exit 1
    
    # Success
    print_success "CORI workspace build completed successfully!"
    log_success "Build process completed"
    
    echo ""
    echo -e "${GREEN}Next steps:${NC}"
    echo "  1. Source the workspace: ${CYAN}source install/setup.bash${NC}"
    echo "  2. Run demos: ${CYAN}./scripts/cori_demo.bash${NC}"
    echo "  3. View logs: ${CYAN}cat ${LOG_FILE}${NC}"
    echo ""
}

# Execute main function if script is run directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi