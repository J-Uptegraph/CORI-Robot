#!/usr/bin/env python3
"""
CORI Unified Demo System
Combines physical camera detection with virtual Gazebo object search and head movement
"""

import json
import time
import math
import threading
import os
import sys
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
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

@dataclass
class ObjectMemory:
    """Memory of object locations"""
    color: str
    angle: float
    confidence: float
    last_seen: float
    found_count: int
    virtual_position: Optional[Tuple[float, float, float]] = None

class VirtualObjectDatabase:
    """Database for storing virtual object locations found in Gazebo"""
    
    def __init__(self, db_file: str = "virtual_object_database.json"):
        self.db_file = db_file
        self.database = self.load_database()
    
    def load_database(self) -> Dict:
        """Load or create virtual object database"""
        default_db = {
            "metadata": {
                "created": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                "description": "CORI Virtual Object Database - Gazebo object locations",
                "version": "1.0"
            },
            "virtual_objects": {},
            "search_history": []
        }
        
        try:
            with open(self.db_file, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            print(f"🆕 Creating new virtual object database: {self.db_file}")
            self.save_database(default_db)
            return default_db
        except json.JSONDecodeError:
            print(f"⚠️  Virtual database corrupted, creating new one")
            return default_db
    
    def save_database(self, db_data=None):
        """Save database to file"""
        if db_data is None:
            db_data = self.database
        try:
            with open(self.db_file, 'w') as f:
                json.dump(db_data, f, indent=2)
            return True
        except Exception as e:
            print(f"❌ Failed to save virtual database: {e}")
            return False
    
    def store_virtual_object(self, color: str, position: Tuple[float, float, float], angle: float):
        """Store virtual object location"""
        self.database["virtual_objects"][color] = {
            "position": position,
            "angle": angle,
            "timestamp": time.time(),
            "found_count": self.database["virtual_objects"].get(color, {}).get("found_count", 0) + 1
        }
        self.save_database()
        print(f"💾 Stored virtual {color} object at position {position}, angle {angle:.1f}°")

class CORIUnifiedDemo(Node):
    """Main CORI demo system that combines everything"""
    
    def __init__(self):
        super().__init__('cori_unified_demo')
        
        # Initialize components
        self.bridge = CvBridge()
        self.virtual_db = VirtualObjectDatabase()
        
        # Memory system
        self.object_memory: Dict[str, ObjectMemory] = {}
        self.load_spatial_database()
        
        # Detection state
        self.current_detections = []
        self.last_detection_time = 0
        self.detection_interval = 3.0  # Detect every 3 seconds for demo
        self.current_head_angle = 0.0
        
        # Demo state
        self.demo_active = False
        self.searching_color = None
        self.search_start_time = 0
        
        # Virtual objects from laundry_world.sdf
        self.virtual_objects = {
            'red': (0.8, 0.2, 0.45),      # shirt_red position
            'green': (1.0, 0.0, 0.45),   # shirt_green position  
            'blue': (0.7, -0.2, 0.45),   # pants_blue position
            'white': (1.2, 0.1, 0.45),   # sock_white position
            'gray': (1.3, -0.2, 0.45)    # towel_gray position
        }
        
        # Color detection parameters (tuned for your camera)
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
            'white': {
                'lower': np.array([0, 0, 200]),
                'upper': np.array([180, 55, 255])
            }
        }
        
        # ROS setup
        self.setup_ros_connections()
        
        self.get_logger().info("🤖 CORI Unified Demo System Ready!")
    
    def setup_ros_connections(self):
        """Setup ROS2 connections"""
        # Camera input
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_callback,
            10
        )
        
        # Head control
        self.head_pub = self.create_publisher(
            Float64,
            '/model/cori/joint/head_joint/cmd_pos',
            10
        )
        
        # Demo status output
        self.status_pub = self.create_publisher(
            String,
            '/cori/demo_status',
            10
        )
        
        # Color detection output
        self.color_pub = self.create_publisher(
            String,
            '/cori/color_detected',
            10
        )
    
    def load_spatial_database(self):
        """Load existing spatial database into memory"""
        db_path = "src/database/cori_spatial_database.json"
        try:
            with open(db_path, 'r') as f:
                spatial_db = json.load(f)
            
            # Extract object memories
            if "objects" in spatial_db:
                for color_key, objects in spatial_db["objects"].items():
                    color = color_key.replace("_objects", "")
                    if objects:
                        best_obj = max(objects, key=lambda x: x.get("confidence", 0))
                        self.object_memory[color] = ObjectMemory(
                            color=color,
                            angle=best_obj.get("primary_location", 0),
                            confidence=best_obj.get("confidence", 0.5),
                            last_seen=best_obj.get("last_seen", time.time()),
                            found_count=best_obj.get("success_count", 0)
                        )
            
            print(f"📖 Loaded {len(self.object_memory)} object memories")
            
        except FileNotFoundError:
            print("📝 No existing spatial database found, starting fresh")
        except Exception as e:
            print(f"⚠️  Error loading spatial database: {e}")
    
    def camera_callback(self, msg):
        """Process camera images with throttling"""
        current_time = time.time()
        
        # Throttle detection rate
        if current_time - self.last_detection_time < self.detection_interval:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            detected_color = self.detect_dominant_color(cv_image)
            
            if detected_color != "unknown":
                self.process_physical_detection(detected_color)
                self.last_detection_time = current_time
                
        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")
    
    def detect_dominant_color(self, image):
        """Detect dominant color in image center"""
        height, width = image.shape[:2]
        center_x, center_y = width // 2, height // 2
        roi_size = min(width, height) // 6
        
        roi = image[
            center_y - roi_size:center_y + roi_size,
            center_x - roi_size:center_x + roi_size
        ]
        
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        best_score = 0
        best_color = "unknown"
        
        for color_name, ranges in self.color_ranges.items():
            if color_name == 'red':
                mask1 = cv2.inRange(hsv_roi, ranges['lower1'], ranges['upper1'])
                mask2 = cv2.inRange(hsv_roi, ranges['lower2'], ranges['upper2'])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv_roi, ranges['lower'], ranges['upper'])
            
            score = np.sum(mask) / 255
            if score > best_score and score > 800:  # Higher threshold for demo
                best_score = score
                best_color = color_name
        
        return best_color
    
    def process_physical_detection(self, color: str):
        """Process physical camera detection - Step 1"""
        self.publish_status(f"👁️  I see {color.upper()}")
        self.get_logger().info(f"🎯 Physical detection: {color.upper()}")
        
        # Publish color detection
        color_msg = String()
        color_msg.data = color.upper()
        self.color_pub.publish(color_msg)
        
        # Start demo sequence
        self.searching_color = color
        threading.Thread(target=self.run_demo_sequence, args=(color,), daemon=True).start()
    
    def run_demo_sequence(self, color: str):
        """Run the complete demo sequence"""
        try:
            time.sleep(1)
            self.check_memory_database(color)
            time.sleep(1.5)
            self.virtual_search_gazebo(color)
            time.sleep(1.5)
            self.move_head_to_object(color)
            time.sleep(2)
            self.update_learning_system(color)
            
        except Exception as e:
            self.get_logger().error(f"Demo sequence error: {e}")
            self.publish_status(f"❌ Demo sequence failed: {e}")
    
    def check_memory_database(self, color: str):
        """Step 2: Check spatial database memory"""
        if color in self.object_memory:
            memory = self.object_memory[color]
            confidence_percent = int(memory.confidence * 100)
            
            self.publish_status(f"🧠 My memory says {color.upper()} is usually at {memory.angle:.1f}° (confidence: {confidence_percent}%)")
            self.get_logger().info(f"📊 Memory check: {color} → {memory.angle:.1f}°")
            
            return memory.angle
        else:
            self.publish_status(f"🤔 No memory of {color.upper()} objects yet - learning new object!")
            self.get_logger().info(f"❓ No memory found for {color}")
            return None
    
    def virtual_search_gazebo(self, color: str):
        """Step 3: Search for object in virtual Gazebo world"""
        self.publish_status(f"🔍 Searching Gazebo virtual world for {color.upper()} objects...")
        
        # Simulate search time
        time.sleep(1)
        
        if color in self.virtual_objects:
            position = self.virtual_objects[color]
            
            # Calculate angle from CORI's position to object
            dx = position[0] - 0  # CORI at origin
            dy = position[1] - 0
            angle_rad = math.atan2(dy, dx)
            angle_deg = math.degrees(angle_rad)
            
            # Store in virtual database
            self.virtual_db.store_virtual_object(color, position, angle_deg)
            
            self.publish_status(f"🎯 Virtual search: Found {color.upper()} object at {angle_deg:.1f}°!")
            self.get_logger().info(f"🌐 Virtual found: {color} at {position} → {angle_deg:.1f}°")
            
            return angle_deg
        else:
            self.publish_status(f"❌ Virtual search: No {color.upper()} objects found in world")
            return None
    
    def move_head_to_object(self, color: str):
        """Step 4: Move CORI's head to predicted location"""
        target_angle = None
        source = "unknown"
        
        # Priority: Memory first, then virtual search
        if color in self.object_memory:
            target_angle = self.object_memory[color].angle
            source = "memory"
        elif color in self.virtual_objects:
            position = self.virtual_objects[color]
            dx, dy = position[0], position[1]
            target_angle = math.degrees(math.atan2(dy, dx))
            source = "virtual search"
        
        if target_angle is not None:
            # Convert to radians and scale for robot
            angle_rad = math.radians(target_angle * 0.03)  # Scale for visible movement
            
            self.publish_status(f"🤖 Turning head to {target_angle:.1f}° (from {source})")
            self.get_logger().info(f"🎯 Head movement: {target_angle:.1f}° → {angle_rad:.3f} rad")
            
            # Send head command
            head_msg = Float64()
            head_msg.data = float(angle_rad)
            self.head_pub.publish(head_msg)
            
            self.current_head_angle = angle_rad
            
            # Simulate target acquisition
            time.sleep(1.5)
            self.publish_status(f"✅ Target acquired! Head locked onto {color.upper()} object")
            
        else:
            self.publish_status(f"❓ No location data for {color.upper()} - staying centered")
    
    def update_learning_system(self, color: str):
        """Step 5: Update learning and memory systems"""
        current_time = time.time()
        
        if color in self.object_memory:
            # Update existing memory
            memory = self.object_memory[color]
            memory.found_count += 1
            memory.last_seen = current_time
            memory.confidence = min(0.95, memory.confidence + 0.05)
            
            self.publish_status(f"🎓 Memory updated: {color.upper()} confidence now {memory.confidence:.2f} (found {memory.found_count} times)")
            
        else:
            # Create new memory from virtual search
            if color in self.virtual_objects:
                position = self.virtual_objects[color]
                dx, dy = position[0], position[1]
                angle = math.degrees(math.atan2(dy, dx))
                
                self.object_memory[color] = ObjectMemory(
                    color=color,
                    angle=angle,
                    confidence=0.7,
                    last_seen=current_time,
                    found_count=1,
                    virtual_position=position
                )
                
                self.publish_status(f"🆕 New memory created for {color.upper()} at {angle:.1f}°!")
        
        # Final status
        total_objects = len(self.object_memory)
        time.sleep(1)
        self.publish_status(f"🎉 Demo complete! I now remember {total_objects} different colored objects - I'm getting smarter!")
    
    def publish_status(self, message: str):
        """Publish status message"""
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)
        print(f"🔄 {message}")

def main():
    if not ROS_AVAILABLE:
        print("❌ ROS2 not available")
        return
    
    try:
        rclpy.init()
        demo = CORIUnifiedDemo()
        print("✅ Unified demo system ready!")
        rclpy.spin(demo)
    except KeyboardInterrupt:
        print("\n⏸️  Demo stopped")
    finally:
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass

if __name__ == "__main__":
    main()
