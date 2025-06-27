#!/usr/bin/env python3
"""
CORI Demo Display Interface
Clean terminal display for the unified demo system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import time
from datetime import datetime

class CORIDemoDisplay(Node):
    def __init__(self):
        super().__init__('cori_demo_display')
        
        # Subscribe to demo status
        self.status_sub = self.create_subscription(
            String, '/cori/demo_status', self.status_callback, 10)
        
        # Subscribe to color detection
        self.color_sub = self.create_subscription(
            String, '/cori/color_detected', self.color_callback, 10)
        
        # Display state
        self.current_status = "System ready - waiting for colored objects..."
        self.current_color = "NONE"
        self.status_history = []
        self.demo_start_time = time.time()
        self.demo_step = 0
        
        # Demo steps
        self.demo_steps = [
            "🔍 Waiting for physical detection...",
            "🧠 Checking memory database...",
            "🌐 Searching virtual Gazebo world...",
            "🤖 Moving head to predicted location...",
            "🎓 Updating learning system..."
        ]
        
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
        
        # Update demo step based on status content
        if "I see" in msg.data:
            self.demo_step = 1
        elif "memory says" in msg.data or "No memory" in msg.data:
            self.demo_step = 2
        elif "Virtual search" in msg.data or "Searching Gazebo" in msg.data:
            self.demo_step = 3
        elif "Turning head" in msg.data or "Target acquired" in msg.data:
            self.demo_step = 4
        elif "Memory updated" in msg.data or "Demo complete" in msg.data:
            self.demo_step = 5
    
    def color_callback(self, msg):
        """Handle color detection updates"""
        self.current_color = msg.data
        if self.demo_step == 0:
            self.demo_step = 1
    
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
    
    def get_step_status(self, step_num):
        """Get status indicator for demo step"""
        if step_num < self.demo_step:
            return "✅"
        elif step_num == self.demo_step:
            return "🔄"
        else:
            return "⏳"
    
    def update_display(self):
        """Update the display"""
        self.clear_screen()
        
        # Calculate runtime
        runtime = time.time() - self.demo_start_time
        runtime_str = f"{int(runtime//60):02d}:{int(runtime%60):02d}"
        
        # Header
        print("=" * 80)
        print("🤖 CORI UNIFIED DEMO SYSTEM")
        print("   Physical Detection + Virtual Search + Head Movement + Learning")
        print("=" * 80)
        print()
        
        # Current detection
        print("📊 CURRENT STATUS")
        print("-" * 40)
        print(f"   Detected Color: {self.get_colored_text(self.current_color)}")
        print(f"   System Status:  {self.current_status}")
        print(f"   Runtime:        {runtime_str}")
        print()
        
        # Demo flow progress
        print("🎯 DEMO FLOW PROGRESS")
        print("-" * 40)
        for i, step_desc in enumerate(self.demo_steps, 1):
            status = self.get_step_status(i)
            print(f"   {status} Step {i}: {step_desc}")
        print()
        
        # Expected demo sequence
        print("🔄 EXPECTED DEMO SEQUENCE")
        print("-" * 40)
        print("   1. 👁️  'I see RED' (Physical camera detects object)")
        print("   2. 🧠 'My memory says RED is usually at 45°' (Database lookup)")
        print("   3. 🌐 'Found RED object in Gazebo!' (Virtual world search)")
        print("   4. 🤖 'Target acquired!' (Head turns to predicted location)")
        print("   5. 🎓 'Memory updated - I'm getting smarter' (Learning)")
        print()
        
        # Instructions
        print("📋 DEMO INSTRUCTIONS")
        print("-" * 40)
        print("   • Hold colored objects in front of your physical camera")
        print("   • Try: RED, BLUE, GREEN, WHITE objects for best results")
        print("   • Watch CORI's head movement in Gazebo simulation")
        print("   • Observe the learning progression over multiple detections")
        print("   • Each color creates and improves spatial memory")
        print()
        
        # Live status feed
        print("📡 LIVE STATUS FEED")
        print("-" * 40)
        if self.status_history:
            for status in self.status_history[-6:]:  # Show last 6 messages
                # Highlight important keywords
                display_status = status
                if "I see" in status:
                    display_status = status.replace("I see", "👁️  I see")
                elif "memory says" in status:
                    display_status = status.replace("memory says", "🧠 memory says")
                elif "Virtual search" in status:
                    display_status = status.replace("Virtual search", "🌐 Virtual search")
                elif "Target acquired" in status:
                    display_status = status.replace("Target acquired", "🎯 Target acquired")
                elif "Memory updated" in status:
                    display_status = status.replace("Memory updated", "🎓 Memory updated")
                
                print(f"   {display_status}")
        else:
            print("   Waiting for demo events...")
        print()
        
        # Technical details
        print("🔧 SYSTEM ARCHITECTURE")
        print("-" * 40)
        print("   Physical Camera → OpenCV Detection → ROS2 Messages")
        print("                           ↓")
        print("   Spatial Database ← Memory System → Virtual Object Search")
        print("                           ↓")
        print("   Head Controller ← Prediction Engine → Learning Updates")
        print()
        
        # Why this matters
        print("🏭 PRODUCTION ROBOTICS VALUE")
        print("-" * 40)
        print("   ✅ Multi-sensor data fusion (camera + simulation)")
        print("   ✅ Spatial memory and predictive positioning")
        print("   ✅ Adaptive learning from experience")
        print("   ✅ Real-time decision making pipeline")
        print("   ✅ Scalable to warehouse/manufacturing environments")
        print()
        
        # Footer
        print("=" * 80)
        print("CORI Demo - Press Ctrl+C to exit | Show colored objects to camera!")
        print("=" * 80)


def main():
    rclpy.init()
    display = CORIDemoDisplay()
    
    try:
        print("🖥️  Starting CORI Demo Display...")
        print("📺 Waiting for demo system to start...")
        rclpy.spin(display)
    except KeyboardInterrupt:
        display.get_logger().info("Demo display shutdown")
    finally:
        display.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    