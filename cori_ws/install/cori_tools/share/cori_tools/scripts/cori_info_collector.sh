#!/bin/bash
# CORI System Information Collector
# Run this script to gather all system info for integration planning

# Create output file with timestamp
OUTPUT_FILE="cori_system_info_$(date +%Y%m%d_%H%M%S).txt"

echo "🤖 CORI System Information Collector"
echo "📋 Gathering system information for integration planning..."
echo "💾 Output will be saved to: $OUTPUT_FILE"
echo ""

# Function to run command and capture output safely
run_cmd() {
    local description="$1"
    local command="$2"
    echo "▶ $description" | tee -a "$OUTPUT_FILE"
    echo "Command: $command" >> "$OUTPUT_FILE"
    
    # Run command and capture both stdout and stderr
    if output=$(eval "$command" 2>&1); then
        echo "$output" | tee -a "$OUTPUT_FILE"
        echo "✅ Success" >> "$OUTPUT_FILE"
    else
        echo "❌ Failed: $output" | tee -a "$OUTPUT_FILE"
    fi
    echo "" | tee -a "$OUTPUT_FILE"
}

# Function to check if file/directory exists
check_path() {
    local description="$1"
    local path="$2"
    echo "▶ $description" | tee -a "$OUTPUT_FILE"
    if [ -e "$path" ]; then
        ls -la "$path" | tee -a "$OUTPUT_FILE"
        echo "✅ Found" >> "$OUTPUT_FILE"
    else
        echo "❌ Not found: $path" | tee -a "$OUTPUT_FILE"
    fi
    echo "" | tee -a "$OUTPUT_FILE"
}

# Start collecting information
{
    echo "🤖 CORI SYSTEM INFORMATION REPORT"
    echo "Generated: $(date)"
    echo "Host: $(hostname)"
    echo "User: $(whoami)"
    echo "Current Directory: $(pwd)"
    echo "================================================================"
    echo ""
} > "$OUTPUT_FILE"

echo "🖥️  Collecting Operating System Information..."
run_cmd "Linux Distribution" "lsb_release -a"
run_cmd "Kernel Version" "uname -a"
run_cmd "System Architecture" "arch"
run_cmd "Available Memory" "free -h"
run_cmd "Disk Space" "df -h /"

echo "🐍 Collecting Python Information..."
run_cmd "Python3 Version" "python3 --version"
run_cmd "Python3 Location" "which python3"
run_cmd "Pip3 Version" "pip3 --version"
run_cmd "Pip3 Location" "which pip3"
run_cmd "Active Virtual Environment" "echo \$VIRTUAL_ENV"
run_cmd "Python Path" "python3 -c 'import sys; print(\"\\n\".join(sys.path))'"

echo "🤖 Collecting ROS2 Information..."
run_cmd "ROS2 Version" "ros2 --version"
run_cmd "ROS2 Distribution" "echo \$ROS_DISTRO"
run_cmd "ROS2 Location" "which ros2"
run_cmd "ROS2 Environment Variables" "env | grep ROS"
run_cmd "Available ROS2 Packages (first 10)" "ros2 pkg list | head -10"
run_cmd "ROS2 Topics (if any)" "timeout 3 ros2 topic list"
run_cmd "ROS2 Nodes (if any)" "timeout 3 ros2 node list"

echo "🎮 Collecting Gazebo Information..."
run_cmd "Gazebo Version" "gazebo --version"
run_cmd "Gazebo Location" "which gazebo"
run_cmd "Gazebo Models Path" "echo \$GAZEBO_MODEL_PATH"
run_cmd "Gazebo Plugins Path" "echo \$GAZEBO_PLUGIN_PATH"

echo "📷 Collecting Camera Information..."
run_cmd "Video Devices" "ls -la /dev/video* 2>/dev/null || echo 'No video devices found'"
run_cmd "USB Devices" "lsusb | grep -i camera || echo 'No USB cameras found'"
run_cmd "V4L2 Utils Check" "which v4l2-ctl"
run_cmd "Camera Device Info" "v4l2-ctl --list-devices 2>/dev/null || echo 'v4l-utils not installed'"

echo "📦 Collecting Python Package Information..."
run_cmd "OpenCV" "python3 -c 'import cv2; print(f\"OpenCV: {cv2.__version__}\")' 2>/dev/null || echo 'OpenCV not installed'"
run_cmd "NumPy" "python3 -c 'import numpy; print(f\"NumPy: {numpy.__version__}\")' 2>/dev/null || echo 'NumPy not installed'"
run_cmd "ROS2 Python (rclpy)" "python3 -c 'import rclpy; print(\"rclpy available\")' 2>/dev/null || echo 'rclpy not installed'"
run_cmd "CV Bridge" "python3 -c 'from cv_bridge import CvBridge; print(\"cv_bridge available\")' 2>/dev/null || echo 'cv_bridge not installed'"
run_cmd "All Installed Packages" "pip3 list"

echo "📁 Collecting Workspace Information..."
check_path "Current Directory Contents" "."
check_path "ROS2 Workspace (~/colcon_ws)" "$HOME/colcon_ws"
check_path "Alternative Workspace (~/ros2_ws)" "$HOME/ros2_ws"
check_path "Development Directory (~/dev)" "$HOME/dev"
check_path "CORI Files in Current Directory" "*.py"
check_path "CORI Text Files" "*.txt"
check_path "Database Directory" "database"

echo "🔍 Searching for CORI-related Files..."
run_cmd "Find CORI Directories" "find ~ -name '*cori*' -type d 2>/dev/null | head -10"
run_cmd "Find CORI Files" "find ~ -name '*cori*' -type f 2>/dev/null | head -10"
run_cmd "Find Launch Files" "find ~ -name '*.launch*' 2>/dev/null | grep -i cori | head -5"

echo "🔧 Collecting Build Tools Information..."
run_cmd "Colcon" "which colcon"
run_cmd "CMake" "cmake --version 2>/dev/null | head -1"
run_cmd "Make" "make --version 2>/dev/null | head -1"
run_cmd "GCC" "gcc --version 2>/dev/null | head -1"

echo "🌐 Collecting Network Information..."
run_cmd "Network Interfaces" "ip addr show | grep -E '^[0-9]+:' -A 2"
run_cmd "ROS Domain ID" "echo \$ROS_DOMAIN_ID"
run_cmd "Localhost Check" "ping -c 1 localhost 2>/dev/null || echo 'Localhost not reachable'"

echo "⚡ Testing CORI Files..."
if [ -f "paste.txt" ]; then
    echo "▶ Testing Database Manager (paste.txt)" | tee -a "$OUTPUT_FILE"
    python3 -c "
import sys
sys.path.append('.')
try:
    with open('paste.txt', 'r') as f:
        content = f.read()
    if 'CORIDatabaseManager' in content:
        print('✅ Database Manager file found and contains expected class')
    else:
        print('⚠️  Database Manager file found but may be incomplete')
except Exception as e:
    print(f'❌ Error reading Database Manager: {e}')
" | tee -a "$OUTPUT_FILE"
else
    echo "❌ paste.txt (Database Manager) not found" | tee -a "$OUTPUT_FILE"
fi

if [ -f "paste-2.txt" ]; then
    echo "▶ Testing Sensor Fusion (paste-2.txt)" | tee -a "$OUTPUT_FILE"
    python3 -c "
try:
    with open('paste-2.txt', 'r') as f:
        content = f.read()
    if 'SensorFusionDemo' in content:
        print('✅ Sensor Fusion file found and contains expected class')
    else:
        print('⚠️  Sensor Fusion file found but may be incomplete')
except Exception as e:
    print(f'❌ Error reading Sensor Fusion: {e}')
" | tee -a "$OUTPUT_FILE"
else
    echo "❌ paste-2.txt (Sensor Fusion) not found" | tee -a "$OUTPUT_FILE"
fi

if [ -f "paste-3.txt" ]; then
    echo "▶ Testing Laundry Assistant (paste-3.txt)" | tee -a "$OUTPUT_FILE"
    python3 -c "
try:
    with open('paste-3.txt', 'r') as f:
        content = f.read()
    if 'CORILaundryAssistant' in content:
        print('✅ Laundry Assistant file found and contains expected class')
    else:
        print('⚠️  Laundry Assistant file found but may be incomplete')
except Exception as e:
    print(f'❌ Error reading Laundry Assistant: {e}')
" | tee -a "$OUTPUT_FILE"
else
    echo "❌ paste-3.txt (Laundry Assistant) not found" | tee -a "$OUTPUT_FILE"
fi

echo "" | tee -a "$OUTPUT_FILE"

# Summary Section
{
    echo "================================================================"
    echo "📊 INTEGRATION READINESS SUMMARY"
    echo "================================================================"
    echo ""
    
    echo "🟢 READY COMPONENTS:"
    echo "-------------------"
} >> "$OUTPUT_FILE"

# Check for ready components
python3 --version >/dev/null 2>&1 && echo "✅ Python3 installed" >> "$OUTPUT_FILE"
ros2 --version >/dev/null 2>&1 && echo "✅ ROS2 installed" >> "$OUTPUT_FILE"
gazebo --version >/dev/null 2>&1 && echo "✅ Gazebo installed" >> "$OUTPUT_FILE"
[ -f "paste.txt" ] && echo "✅ Database Manager available" >> "$OUTPUT_FILE"
[ -f "paste-2.txt" ] && echo "✅ Sensor Fusion available" >> "$OUTPUT_FILE"
[ -f "paste-3.txt" ] && echo "✅ Laundry Assistant available" >> "$OUTPUT_FILE"

{
    echo ""
    echo "⚠️  POTENTIAL ISSUES:"
    echo "--------------------"
} >> "$OUTPUT_FILE"

# Check for potential issues
python3 -c "import cv2" >/dev/null 2>&1 || echo "❌ OpenCV not installed" >> "$OUTPUT_FILE"
python3 -c "import rclpy" >/dev/null 2>&1 || echo "❌ rclpy not installed" >> "$OUTPUT_FILE"
[ -z "$ROS_DISTRO" ] && echo "❌ ROS2 environment not sourced" >> "$OUTPUT_FILE"
[ ! -d "/dev/" ] || [ -z "$(ls /dev/video* 2>/dev/null)" ] && echo "⚠️  No camera devices detected" >> "$OUTPUT_FILE"

{
    echo ""
    echo "📝 RECOMMENDED NEXT STEPS:"
    echo "-------------------------"
    echo "1. Review the complete report above"
    echo "2. Install any missing packages identified"
    echo "3. Source ROS2 environment if needed"
    echo "4. Test individual CORI components"
    echo "5. Proceed with integration based on findings"
    echo ""
    echo "================================================================"
    echo "Report generated: $(date)"
    echo "Report saved to: $OUTPUT_FILE"
    echo "================================================================"
} >> "$OUTPUT_FILE"

echo ""
echo "✅ System information collection complete!"
echo "📄 Full report saved to: $OUTPUT_FILE"
echo ""
echo "📋 Quick Summary:"
python3 --version >/dev/null 2>&1 && echo "✅ Python3: Ready" || echo "❌ Python3: Missing"
ros2 --version >/dev/null 2>&1 && echo "✅ ROS2: Ready" || echo "❌ ROS2: Missing" 
gazebo --version >/dev/null 2>&1 && echo "✅ Gazebo: Ready" || echo "❌ Gazebo: Missing"
[ -f "paste.txt" ] && echo "✅ CORI Files: Found" || echo "❌ CORI Files: Missing"
echo ""
echo "📨 Share the file '$OUTPUT_FILE' for personalized integration guidance!"
