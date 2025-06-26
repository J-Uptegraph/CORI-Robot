#!/usr/bin/env python3
"""
CORI Sensor Fusion Demo Display
Clean terminal interface showing sensor fusion progress
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import time
from datetime import datetime

class DemoDisplay(Node):
    def __init__(self):
        super().__init__('demo_display')
        
        # Subscribe to demo status
        self.status_sub = self.create_subscription(
            String, '/cori/demo_status', self.status_callback, 10)
        
        # Subscribe to color detection
        self.color_sub = self.create_subscription(
            String, '/cori/color_detected', self.color_callback, 10)
        
        # Display state
        self.current_status = "System initializing..."
        self.current_color = "NONE"
        self.status_history = []
        self.demo_start_time = time.time()
        
        # Display timer
        self.display_timer = self.create_timer(0.5, self.update_display)
        
        self.get_logger().info("Demo display ready")
        self.clear_screen()
    
    def status_callback(self, msg):
        """Handle status updates"""
        self.current_status = msg.data
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        self.status_history.append(f"[{timestamp}] {msg.data}")
        
        # Keep only last 8 status messages
        if len(self.status_history) > 8:
            self.status_history.pop(0)
    
    def color_callback(self, msg):
        """Handle color detection updates"""
        self.current_color = msg.data
    
    def clear_screen(self):
        """Clear terminal screen"""
        os.system('clear' if os.name == 'posix' else 'cls')
    
    def get_colored_text(self, color):
        """Return colored text for terminal display"""
        color_codes = {
            'RED': '\033[91m',
            'GREEN': '\033[92m',
            'YELLOW': '\033[93m',
            'BLUE': '\033[94m',
            'PURPLE': '\033[95m',
            'CYAN': '\033[96m',
            'WHITE': '\033[97m',
            'BLACK': '\033[90m',
            'GRAY': '\033[37m',
            'ORANGE': '\033[33m',
            'NONE': '\033[37m'
        }
        
        reset = '\033[0m'
        color_code = color_codes.get(color, '\033[0m')
        
        return f"{color_code}{color}{reset}"
    
    def update_display(self):
        """Update the display"""
        self.clear_screen()
        
        # Calculate runtime
        runtime = time.time() - self.demo_start_time
        runtime_str = f"{int(runtime//60):02d}:{int(runtime%60):02d}"
        
        # Header
        print("=" * 80)
        print("🤖 CORI SENSOR FUSION DEMONSTRATION")
        print("   Combining Real-Time Detection with Spatial Memory")
        print("=" * 80)
        print()
        
        # Current status
        print("📊 CURRENT STATUS")
        print("-" * 40)
        print(f"   Detected Object: {self.get_colored_text(self.current_color)}")
        print(f"   System State:    {self.current_status}")
        print(f"   Runtime:         {runtime_str}")
        print()
        
        # System architecture
        print("🏗️  SYSTEM ARCHITECTURE")
        print("-" * 40)
        print("   Physical Camera → OpenCV Detection → ROS 2 Messages")
        print("                           ↓")
        print("   Spatial Database ← Learning System ← Head Movement")
        print("                           ↓")
        print("   Professional Robotics: Sensor Fusion in Action")
        print()
        
        # Expected behavior
        print("🎯 EXPECTED BEHAVIOR")
        print("-" * 40)
        print("   🔴 Hold RED object   → CORI looks LEFT (~14°)")
        print("   🔵 Hold BLUE object  → CORI looks RIGHT (~-16°)")
        print("   🟢 Hold GREEN object → CORI looks STRAIGHT (0°)")
        print("   ⚪ Hold WHITE object → CORI looks slightly LEFT (~5°)")
        print()
        
        # Demo instructions
        print("📋 DEMO INSTRUCTIONS")
        print("-" * 40)
        print("   1. Hold colored objects steadily in front of camera")
        print("   2. Wait for 'Physical sensor detects [COLOR]'")
        print("   3. Watch CORI's head move to predicted location")
        print("   4. Observe learning: confidence increases over time")
        print("   5. Press Ctrl+C to stop demo")
        print()
        
        # Live status history
        print("📡 LIVE STATUS FEED")
        print("-" * 40)
        if self.status_history:
            for status in self.status_history[-6:]:  # Show last 6 messages
                print(f"   {status}")
        else:
            print("   Waiting for sensor fusion events...")
        print()
        
        # Demo results for Dan
        print("🎓 WHY THIS MATTERS FOR PRODUCTION ROBOTICS")
        print("-" * 40)
        print("   ✅ Real-time sensor data fusion")
        print("   ✅ Spatial memory and prediction")
        print("   ✅ Adaptive learning algorithms")
        print("   ✅ Professional ROS 2 architecture")
        print("   ✅ Scalable to multiple sensors/objects")
        print()
        print("   This is exactly how warehouse robots and autonomous")
        print("   vehicles combine multiple data sources for intelligent")
        print("   decision making! 🚗 🤖")
        print()
        
        # Footer
        print("=" * 80)
        print("CORI Sensor Fusion Demo - Press Ctrl+C to exit")
        print("=" * 80)


def main():
    rclpy.init()
    display = DemoDisplay()
    
    try:
        rclpy.spin(display)
    except KeyboardInterrupt:
        display.get_logger().info("Demo display shutdown")
    finally:
        display.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()