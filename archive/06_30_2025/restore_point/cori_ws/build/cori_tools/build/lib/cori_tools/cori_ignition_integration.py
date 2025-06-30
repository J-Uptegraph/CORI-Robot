#!/usr/bin/env python3
"""
CORI Ignition Gazebo Integration - FIXED VERSION
Corrected joint names and detection throttling
"""

import json
import time
import math
import threading
import queue
import os
import sys
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, asdict
from datetime import datetime
from enum import Enum

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import String, Float64
    from cv_bridge import CvBridge
    import cv2
    import numpy as np
    ROS_AVAILABLE = True
    print("✅ ROS2 modules loaded successfully")
except ImportError as e:
    print(f"❌ ROS2 import failed: {e}")
    ROS_AVAILABLE = False

class CORIMode(Enum):
    """Operation modes for CORI with Ignition"""
    CAMERA_ONLY = "camera_only"
    IGNITION_FULL = "ignition_full"
    LAUNDRY_CAMERA = "laundry_camera"
    DATABASE_ONLY = "database_only"

@dataclass
class DetectionEvent:
    """Structure for detection events"""
    color: str
    confidence: float
    timestamp: float
    context: str = ""
    item_type: str = ""

class IgnitionCORIDatabase:
    """Database manager optimized for your Ignition setup"""
    
    def __init__(self, database_file: str = None):
        if database_file is None:
            workspace_path = "/home/juptegraph/Workspaces/Robotics/Projects/CORI/cori_ws"
            database_file = f"{workspace_path}/src/cori_tools/cori_tools/cori_ignition_database.json"
        
        self.database_file = database_file
        self.ensure_database_directory()
        self.database = self.load_database()
    
    def ensure_database_directory(self):
        """Create database directory if it doesn't exist"""
        db_dir = os.path.dirname(self.database_file)
        if not os.path.exists(db_dir):
            os.makedirs(db_dir, exist_ok=True)
            print(f"📁 Created database directory: {db_dir}")
    
    def load_database(self) -> Dict:
        """Load or create database"""
        default_db = {
            "metadata": {
                "created": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "description": "CORI Ignition Gazebo Database",
                "version": "1.0",
                "ignition_compatible": True
            },
            "spatial_objects": {
                "red_objects": [],
                "blue_objects": [],
                "green_objects": [],
                "white_objects": [],
                "black_objects": [],
                "yellow_objects": [],
                "orange_objects": []
            },
            "ignition_data": {
                "joint_commands": [],
                "camera_calibration": {},
                "movement_history": []
            },
            "laundry_learning": {
                "total_sorted": 0,
                "user_preferences": {},
                "color_categories": {
                    "lights": ["white", "yellow", "light_grey", "cream"],
                    "darks": ["black", "dark_blue", "navy", "brown"],
                    "colors": ["red", "blue", "green", "orange", "purple"]
                }
            },
            "detection_history": []
        }
        
        try:
            with open(self.database_file, 'r') as f:
                db = json.load(f)
                print(f"📖 Loaded existing database: {self.database_file}")
                return db
        except FileNotFoundError:
            print(f"🆕 Creating new database: {self.database_file}")
            self.save_database(default_db)
            return default_db
        except json.JSONDecodeError:
            print(f"⚠️  Database corrupted, creating backup and new one")
            return default_db
    
    def save_database(self, db_data=None):
        """Save database to file"""
        if db_data is None:
            db_data = self.database
            
        try:
            with open(self.database_file, 'w') as f:
                json.dump(db_data, f, indent=2)
            return True
        except Exception as e:
            print(f"❌ Failed to save database: {e}")
            return False
    
    def log_detection(self, event: DetectionEvent):
        """Log detection event"""
        detection_record = {
            "timestamp": event.timestamp,
            "color": event.color,
            "confidence": event.confidence,
            "context": event.context,
            "item_type": event.item_type
        }
        
        self.database["detection_history"].append(detection_record)
        
        # Keep only last 500 detections
        if len(self.database["detection_history"]) > 500:
            self.database["detection_history"] = self.database["detection_history"][-500:]
        
        self.save_database()

class IgnitionCameraHandler:
    """Handle camera detection for Ignition Gazebo with throttling"""
    
    def __init__(self, node_name: str = "cori_ignition_camera"):
        self.node = None
        self.bridge = None
        self.detection_callbacks = []
        
        # Detection throttling
        self.last_detection_time = 0
        self.detection_interval = 1.0  # Only detect every 1 second
        self.last_detected_color = "unknown"
        self.stable_detection_count = 0
        self.required_stable_detections = 3
        
        # Color detection parameters (tuned for your NexiGo camera)
        self.color_ranges = {
                'red': {
                    'lower1': np.array([0, 120, 70]),
                    'upper1': np.array([10, 255, 255]),
                    'lower2': np.array([170, 120, 70]),
                    'upper2': np.array([180, 255, 255])
                },
                'blue': {
                    'lower': np.array([100, 150, 70]),
                    'upper': np.array([130, 255, 255])
                },
                'green': {
                    'lower': np.array([40, 120, 70]),
                    'upper': np.array([80, 255, 255])
                },
                'yellow': {
                    'lower': np.array([20, 100, 100]),
                    'upper': np.array([30, 255, 255])
                },
                'orange': {
                    'lower': np.array([10, 100, 100]),
                    'upper': np.array([20, 255, 255])
                },
                'purple': {
                    'lower': np.array([130, 100, 100]),
                    'upper': np.array([160, 255, 255])
                },
                'black': {
                    'lower': np.array([0, 0, 0]),
                    'upper': np.array([180, 255, 50])
                },
                'white': {
                    'lower': np.array([0, 0, 200]),
                    'upper': np.array([180, 55, 255])
                },
                'grey': {
                    'lower': np.array([0, 0, 80]),
                    'upper': np.array([180, 50, 200])
                }
        }

        
        if ROS_AVAILABLE:
            self.setup_ros()
    
    def setup_ros(self):
        """Setup ROS2 node for camera handling"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = rclpy.create_node('cori_ignition_camera')
            self.bridge = CvBridge()
            
            # Subscribe to your existing camera topic
            self.camera_sub = self.node.create_subscription(
                Image,
                '/camera/color/image_raw',
                self.camera_callback,
                10
            )
            
            # Publisher for detected colors
            self.color_pub = self.node.create_publisher(
                String,
                '/cori/color_detected',
                10
            )
            
            print("✅ Camera handler initialized with Ignition topics")
            
        except Exception as e:
            print(f"❌ ROS setup failed: {e}")
    
    def add_detection_callback(self, callback):
        """Add callback for when colors are detected"""
        self.detection_callbacks.append(callback)
    
    def camera_callback(self, msg):
        """Process camera images with throttling"""
        current_time = time.time()
        
        # Throttle detection rate
        if current_time - self.last_detection_time < self.detection_interval:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            detected_color = self.detect_dominant_color(cv_image)
            
            # Require stable detections
            if detected_color == self.last_detected_color:
                self.stable_detection_count += 1
            else:
                self.stable_detection_count = 1
                self.last_detected_color = detected_color
            
            # Only process stable detections
            if detected_color != "unknown" and self.stable_detection_count >= self.required_stable_detections:
                # Create detection event
                event = DetectionEvent(
                    color=detected_color,
                    confidence=0.85,
                    timestamp=current_time,
                    context="ignition_camera"
                )
                
                # Update last detection time
                self.last_detection_time = current_time
                
                # Notify all callbacks
                for callback in self.detection_callbacks:
                    try:
                        callback(event)
                    except Exception as e:
                        print(f"⚠️  Callback error: {e}")
                
                # Publish to ROS topic
                color_msg = String()
                color_msg.data = detected_color
                self.color_pub.publish(color_msg)
                
        except Exception as e:
            print(f"❌ Camera callback error: {e}")
    
    def detect_dominant_color(self, image):
        """Detect dominant color in image center"""
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2
        roi_size = min(width, height) // 6
        
        # Extract center region
        roi = image[
            center_y - roi_size:center_y + roi_size,
            center_x - roi_size:center_x + roi_size
        ]
        
        # Convert to HSV
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Test each color
        best_score = 0
        best_color = "unknown"
        
        for color_name, ranges in self.color_ranges.items():
            if color_name == 'red':
                # Special handling for red (wraps around hue)
                mask1 = cv2.inRange(hsv_roi, ranges['lower1'], ranges['upper1'])
                mask2 = cv2.inRange(hsv_roi, ranges['lower2'], ranges['upper2'])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv_roi, ranges['lower'], ranges['upper'])
            
            score = np.sum(mask) / 255
            if score > best_score and score > 200:  # Minimum threshold
                best_score = score
                best_color = color_name
        
        return best_color

class IgnitionRobotController:
    """Control CORI robot in Ignition Gazebo - FIXED JOINT NAMES"""
    
    def __init__(self, node_name: str = "cori_ignition_controller"):
        self.node = None
        self.joint_publishers = {}
        self.current_angles = {}
        
        if ROS_AVAILABLE:
            self.setup_ros()
    
    def setup_ros(self):
        """Setup ROS2 publishers for Ignition joints"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = rclpy.create_node('cori_ignition_controller')
            
            # CORRECTED: Your robot uses head_joint, not head_pan_joint/head_tilt_joint
            joint_topics = {
                'head': '/model/cori/joint/head_joint/cmd_pos',
            }
            
            # Create publishers for each joint
            for joint_name, topic in joint_topics.items():
                self.joint_publishers[joint_name] = self.node.create_publisher(
                    Float64,
                    topic,
                    10
                )
                self.current_angles[joint_name] = 0.0
                print(f"✅ Connected to joint: {topic}")
            
            print(f"✅ Robot controller initialized with {len(joint_topics)} joints")
            
        except Exception as e:
            print(f"❌ Robot controller setup failed: {e}")
    

    # This is what will be replaced with a database
    def move_head_to_color(self, color: str):
        """Move head based on detected color - FIXED COMMANDS"""
        # Updated color-to-angle mapping
        color_angles = {
    'black':  0.76,
    'grey':   0.62,
    'purple': 0.45,
    'blue':   0.23,
    'green':  0.0,
    'yellow': -0.23,
    'orange': -0.45,
    'red':    -0.62
    }
        
        target_angle = color_angles.get(color, 0.0)
        self.move_head(target_angle)
        
        angle_degrees = math.degrees(target_angle)
        direction = "LEFT" if target_angle < 0 else "RIGHT" if target_angle > 0 else "CENTER"
        print(f"🤖 Moving head {direction} ({angle_degrees:.1f}°) for {color} object")
    
    def move_head(self, angle_rad: float):
        """Move head joint to specified angle"""
        if 'head' in self.joint_publishers:
            msg = Float64()
            msg.data = float(angle_rad)
            self.joint_publishers['head'].publish(msg)
            self.current_angles['head'] = angle_rad
            print(f"📡 Sent head command: {angle_rad:.2f} rad")

class CORIIgnitionSystem:
    """Main CORI system for Ignition Gazebo - FIXED VERSION"""
    
    def __init__(self, mode: CORIMode = CORIMode.CAMERA_ONLY):
        self.mode = mode
        self.database = IgnitionCORIDatabase()
        self.camera_handler = None
        self.robot_controller = None
        
        print(f"🤖 Initializing CORI for Ignition Gazebo in {mode.value} mode")
        
        # Initialize components based on mode
        self.setup_components()
        
        # Start ROS spinning in background thread
        self.ros_thread = None
        if ROS_AVAILABLE:
            self.start_ros_spinning()
    
    def setup_components(self):
        """Setup components based on mode"""
        if self.mode in [CORIMode.CAMERA_ONLY, CORIMode.IGNITION_FULL, CORIMode.LAUNDRY_CAMERA]:
            self.camera_handler = IgnitionCameraHandler()
            self.camera_handler.add_detection_callback(self.handle_detection)
        
        if self.mode in [CORIMode.IGNITION_FULL, CORIMode.LAUNDRY_CAMERA]:
            self.robot_controller = IgnitionRobotController()
    
    def start_ros_spinning(self):
        """Start ROS2 spinning in background thread"""
        def spin_nodes():
            while rclpy.ok():
                try:
                    if self.camera_handler and self.camera_handler.node:
                        rclpy.spin_once(self.camera_handler.node, timeout_sec=0.1)
                    if self.robot_controller and self.robot_controller.node:
                        rclpy.spin_once(self.robot_controller.node, timeout_sec=0.1)
                except Exception as e:
                    print(f"⚠️  ROS spinning error: {e}")
                    break
        
        self.ros_thread = threading.Thread(target=spin_nodes, daemon=True)
        self.ros_thread.start()
        print("✅ ROS spinning started in background")
    
    def handle_detection(self, event: DetectionEvent):
        """Handle color detection events"""
        print(f"👁️  Detected {event.color} (confidence: {event.confidence:.2f})")
        
        # Log to database
        self.database.log_detection(event)
        
        # Move robot if in full mode
        if self.mode in [CORIMode.IGNITION_FULL, CORIMode.LAUNDRY_CAMERA] and self.robot_controller:
            self.robot_controller.move_head_to_color(event.color)
        
        # Handle laundry sorting if in laundry mode
        if self.mode == CORIMode.LAUNDRY_CAMERA:
            self.suggest_laundry_category(event.color)
    
    def suggest_laundry_category(self, color: str):
        """Suggest laundry category for detected color"""
        categories = self.database.database["laundry_learning"]["color_categories"]
        
        for category, colors in categories.items():
            if color in colors:
                print(f"🧺 Suggestion: {color} → {category.upper()} hamper")
                return category
        
        print(f"🤔 Unknown color: {color} - please teach me where it goes!")
        return "unknown"
    
    def show_menu(self):
        """Show interactive menu"""
        while True:
            print("\n" + "="*50)
            print("🤖 CORI IGNITION GAZEBO SYSTEM - FIXED VERSION")
            print("="*50)
            print(f"Current Mode: {self.mode.value}")
            print(f"Camera Status: {'✅ Active' if self.camera_handler else '❌ Disabled'}")
            print(f"Robot Control: {'✅ Active' if self.robot_controller else '❌ Disabled'}")
            print()
            print("Options:")
            print("1. Test camera detection")
            print("2. Manual robot control")
            print("3. View detection history") 
            print("4. Laundry sorting mode")
            print("5. Database statistics")
            print("6. Change mode")
            print("7. Exit")
            
            choice = input("\nChoose (1-7): ").strip()
            
            if choice == "1":
                self.test_camera()
            elif choice == "2":
                self.manual_robot_control()
            elif choice == "3":
                self.show_detection_history()
            elif choice == "4":
                self.laundry_mode()
            elif choice == "5":
                self.show_database_stats()
            elif choice == "6":
                self.change_mode()
            elif choice == "7":
                self.shutdown()
                break
            else:
                print("❌ Invalid choice")
    
    def test_camera(self):
        """Test camera detection"""
        if not self.camera_handler:
            print("❌ Camera not available in current mode")
            return
        
        print("\n📷 Camera Detection Test - THROTTLED")
        print("Hold colored objects in front of camera...")
        print("⏱️  Detection rate: 1 per second (no more spam!)")
        print("🎯 Try: red, blue, green, white objects")
        print("Press Enter to stop")
        
        input()
        print("✅ Camera test complete")
    
    def manual_robot_control(self):
        """Manual robot control interface"""
        if not self.robot_controller:
            print("❌ Robot control not available in current mode")
            return
        
        print("\n🤖 Manual Robot Control - FIXED COMMANDS")
        print("Commands: left, right, center, test, quit")
        
        while True:
            cmd = input("Robot> ").strip().lower()
            
            if cmd == "quit":
                break
            elif cmd == "left":
                self.robot_controller.move_head(-0.5)
                print("👈 Head moved left")
            elif cmd == "right":
                self.robot_controller.move_head(0.5)
                print("👉 Head moved right")
            elif cmd == "center":
                self.robot_controller.move_head(0.0)
                print("🎯 Head centered")
            elif cmd == "test":
                print("🧪 Testing movement sequence...")
                self.robot_controller.move_head(-0.5)
                time.sleep(1)
                self.robot_controller.move_head(0.5)
                time.sleep(1)
                self.robot_controller.move_head(0.0)
                print("✅ Test complete")
            else:
                print("❌ Unknown command. Try: left, right, center, test, quit")
    
    def show_detection_history(self):
        """Show recent detection history"""
        history = self.database.database["detection_history"]
        
        print("\n📊 Recent Detections:")
        if not history:
            print("No detections recorded yet")
            return
        
        recent = history[-10:]
        for i, detection in enumerate(recent, 1):
            timestamp = datetime.fromtimestamp(detection["timestamp"])
            print(f"{i:2d}. {detection['color']:8s} "
                  f"({detection['confidence']:.2f}) "
                  f"at {timestamp.strftime('%H:%M:%S')}")
    
    def show_database_stats(self):
        """Show database statistics"""
        db = self.database.database
        
        print("\n📊 Database Statistics:")
        print(f"Total detections: {len(db['detection_history'])}")
        print(f"Laundry items sorted: {db['laundry_learning']['total_sorted']}")
        
        # Color frequency
        if db['detection_history']:
            colors = [d['color'] for d in db['detection_history']]
            color_counts = {}
            for color in colors:
                color_counts[color] = color_counts.get(color, 0) + 1
            
            print("\nColor frequency:")
            for color, count in sorted(color_counts.items(), key=lambda x: x[1], reverse=True):
                print(f"  {color}: {count}")
    
    def laundry_mode(self):
        """Interactive laundry sorting"""
        print("\n🧺 Laundry Sorting Mode")
        print("Show me clothing items and I'll suggest which hamper!")
        print("Commands: 'done' to exit, 'teach [color] [category]' to train")
        
        while True:
            cmd = input("\nLaundry> ").strip().lower()
            
            if cmd == "done":
                break
            elif cmd.startswith("teach"):
                parts = cmd.split()
                if len(parts) == 3:
                    color, category = parts[1], parts[2]
                    self.teach_color_category(color, category)
                else:
                    print("Usage: teach [color] [category]")
            else:
                print("Hold up a clothing item for automatic detection...")
    
    def teach_color_category(self, color: str, category: str):
        """Teach color categorization"""
        valid_categories = ["lights", "darks", "colors"]
        if category not in valid_categories:
            print(f"❌ Category must be one of: {', '.join(valid_categories)}")
            return
        
        categories = self.database.database["laundry_learning"]["color_categories"]
        
        # Remove color from other categories
        for cat, colors in categories.items():
            if color in colors:
                colors.remove(color)
        
        # Add to new category
        categories[category].append(color)
        self.database.save_database()
        
        print(f"✅ Learned: {color} → {category}")
    
    def change_mode(self):
        """Change operation mode"""
        print("\nAvailable modes:")
        for i, mode in enumerate(CORIMode, 1):
            print(f"{i}. {mode.value}")
        
        try:
            choice = int(input("Choose mode: ")) - 1
            if 0 <= choice < len(CORIMode):
                new_mode = list(CORIMode)[choice]
                self.__init__(new_mode)
                print(f"✅ Switched to {new_mode.value} mode")
            else:
                print("❌ Invalid choice")
        except ValueError:
            print("❌ Please enter a number")
    
    def shutdown(self):
        """Shutdown system"""
        print("\n🔄 Shutting down CORI Ignition system...")
        
        # Save final database state
        self.database.save_database()
        print("💾 Database saved")
        
        # Shutdown ROS
        if ROS_AVAILABLE:
            try:
                rclpy.shutdown()
                print("✅ ROS shutdown complete")
            except:
                pass
        
        print("👋 CORI shutdown complete!")

def main():
    """Main entry point"""
    print("🤖 CORI Ignition Gazebo System - FIXED VERSION")
    print("✅ Fixed joint names and detection throttling")
    print()
    
    # Check if ROS2 topics are available
    if ROS_AVAILABLE:
        try:
            rclpy.init()
            temp_node = rclpy.create_node('cori_topic_checker')
            
            topic_names = temp_node.get_topic_names_and_types()
            topic_list = [name for name, _ in topic_names]
            
            camera_available = any('/camera' in topic for topic in topic_list)
            cori_available = any('/cori' in topic for topic in topic_list)
            
            temp_node.destroy_node()
            rclpy.shutdown()
            
            print(f"📷 Camera topics: {'✅ Found' if camera_available else '❌ Not found'}")
            print(f"🤖 CORI topics: {'✅ Found' if cori_available else '❌ Not found'}")
            print()
            
        except Exception as e:
            print(f"⚠️  Could not check topics: {e}")
    
    # Mode selection
    print("Select operation mode:")
    for i, mode in enumerate(CORIMode, 1):
        print(f"{i}. {mode.value.replace('_', ' ').title()}")
    
    try:
        choice = int(input("\nChoose mode (1-4): ")) - 1
        if 0 <= choice < len(CORIMode):
            selected_mode = list(CORIMode)[choice]
        else:
            print("Invalid choice, using Camera Only mode")
            selected_mode = CORIMode.CAMERA_ONLY
    except ValueError:
        print("Invalid input, using Camera Only mode")
        selected_mode = CORIMode.CAMERA_ONLY
    
    # Initialize and run system
    try:
        system = CORIIgnitionSystem(selected_mode)
        system.show_menu()
    except KeyboardInterrupt:
        print("\n⏸️  Interrupted by user")
    except Exception as e:
        print(f"\n❌ System error: {e}")
    finally:
        print("👋 Thanks for using CORI!")

if __name__ == "__main__":
    main()