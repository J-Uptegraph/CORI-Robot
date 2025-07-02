#!/usr/bin/env python3
"""
CORI Sensor Fusion Demo with Physics Wake-up Fix
Combines camera detection with spatial database for intelligent head movement
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float64
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math
import sys
import os

# Add the sensor_fusion directory to Python path
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from spatial_database import SpatialDatabase

class SensorFusionDemo(Node):
    def __init__(self):
        super().__init__('sensor_fusion_demo')
        
        # Initialize components
        self.bridge = CvBridge()
        self.spatial_db = SpatialDatabase()
        
        # Physics tracking
        self.physics_awakened = False
        self.last_movement_time = 0
        self.current_head_angle = 0.0
        
        # Detection parameters
        self.detection_threshold = 3  # Need 3 consistent detections
        self.current_detections = []
        self.last_color = "unknown"
        self.detection_confidence = 0
        
        # Color thresholds (BGR format for OpenCV)
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 50, 50]),     # Lower red range
                'upper1': np.array([10, 255, 255]),
                'lower2': np.array([170, 50, 50]),   # Upper red range  
                'upper2': np.array([180, 255, 255])
            },
            'blue': {
                'lower': np.array([100, 50, 50]),
                'upper': np.array([130, 255, 255])
            },
            'green': {
                'lower': np.array([40, 50, 50]),
                'upper': np.array([80, 255, 255])
            },
            'white': {
                'lower': np.array([0, 0, 200]),
                'upper': np.array([180, 30, 255])
            }
        }
        
        # Set up subscribers and publishers
        self.setup_ros_connections()
        
        # Wake up physics system
        self.get_logger().info("🤖 Initializing sensor fusion demo...")
        self.wake_up_physics()
        
        self.get_logger().info("🧠 Sensor fusion demo ready!")
        self.get_logger().info("📷 Hold colored objects in front of camera:")
        self.get_logger().info("   🔴 RED → Head turns LEFT")
        self.get_logger().info("   🔵 BLUE → Head turns RIGHT") 
        self.get_logger().info("   🟢 GREEN → Head looks STRAIGHT")
        
    def setup_ros_connections(self):
        """Set up ROS subscribers and publishers"""
        
        # Try multiple camera topic possibilities
        camera_topics = [
            '/camera/image_raw',
            '/image_raw', 
            '/camera/color/image_raw',
            '/usb_cam/image_raw'
        ]
        
        # Find which camera topic exists
        available_topics = self.get_topic_names_and_types()
        available_topic_names = [topic[0] for topic in available_topics]
        
        camera_topic = None
        for topic in camera_topics:
            if topic in available_topic_names:
                camera_topic = topic
                break
                
        if camera_topic:
            self.get_logger().info(f"📷 Found camera topic: {camera_topic}")
            self.camera_subscriber = self.create_subscription(
                Image,
                camera_topic,
                self.camera_callback,
                10
            )
        else:
            self.get_logger().warn("📷 No camera topic found! Available topics:")
            for topic in available_topic_names:
                if 'image' in topic or 'camera' in topic:
                    self.get_logger().warn(f"   {topic}")
            
            # Default to most common one
            self.camera_subscriber = self.create_subscription(
                Image,
                '/image_raw',
                self.camera_callback,
                10
            )
        
        # Color detection publisher
        self.color_publisher = self.create_publisher(
            String,
            '/cori/color_detected',
            10
        )
        
        # Head joint controller
        self.joint_publisher = self.create_publisher(
            Float64,
            '/model/cori/joint/head_joint/cmd_pos',
            10
        )
        
        # Status publisher  
        self.status_publisher = self.create_publisher(
            String,
            '/cori/sensor_fusion_status',
            10
        )

    def wake_up_physics(self):
        """Wake up Gazebo physics by sending small movements to joints"""
        self.get_logger().info("🔌 Waking up Gazebo physics...")
        
        # Wait for publishers to be ready
        time.sleep(1.0)
        
        # Send small movements to activate physics
        wake_up_sequence = [0.01, -0.01, 0.005, -0.005, 0.0]
        
        for angle in wake_up_sequence:
            msg = Float64()
            msg.data = angle
            self.joint_publisher.publish(msg)
            time.sleep(0.2)
            
        self.physics_awakened = True
        self.current_head_angle = 0.0
        self.get_logger().info("✅ Physics awakened - joints should now respond smoothly")

    def detect_object_color(self, cv_image):
        """Detect dominant color in image using HSV color space"""
        
        # Convert BGR to HSV for better color detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Get image center region (focus on what's being held up)
        height, width = hsv.shape[:2]
        center_x, center_y = width // 2, height // 2
        roi_size = min(width, height) // 4
        
        roi = hsv[
            center_y - roi_size:center_y + roi_size,
            center_x - roi_size:center_x + roi_size
        ]
        
        # Check each color
        color_scores = {}
        
        # Red (special case - needs two ranges)
        if 'red' in self.color_ranges:
            mask1 = cv2.inRange(roi, self.color_ranges['red']['lower1'], self.color_ranges['red']['upper1'])
            mask2 = cv2.inRange(roi, self.color_ranges['red']['lower2'], self.color_ranges['red']['upper2']) 
            red_mask = cv2.bitwise_or(mask1, mask2)
            color_scores['red'] = np.sum(red_mask) / 255
            
        # Other colors
        for color_name, ranges in self.color_ranges.items():
            if color_name == 'red':
                continue
                
            mask = cv2.inRange(roi, ranges['lower'], ranges['upper'])
            color_scores[color_name] = np.sum(mask) / 255
        
        # Find dominant color
        if color_scores:
            dominant_color = max(color_scores, key=color_scores.get)
            max_score = color_scores[dominant_color]
            
            # Threshold for detection confidence
            if max_score > 500:  # Adjust this threshold as needed
                return dominant_color
                
        return "unknown"

    def camera_callback(self, msg):
        """Process camera images for color detection"""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect color
            detected_color = self.detect_object_color(cv_image)
            
            # Add debug output
            if detected_color != "unknown":
                self.get_logger().info(f"🎨 DETECTED COLOR: {detected_color.upper()}")
            
            # Require consistent detections
            self.current_detections.append(detected_color)
            if len(self.current_detections) > self.detection_threshold:
                self.current_detections.pop(0)
            
            # Check for stable detection
            if len(self.current_detections) >= self.detection_threshold:
                unique_colors = set(self.current_detections)
                if len(unique_colors) == 1 and detected_color != "unknown":
                    stable_color = detected_color
                    
                    # Only process if it's a new color
                    if stable_color != self.last_color:
                        self.process_color_detection(stable_color)
                        self.last_color = stable_color
                        
                elif detected_color == "unknown":
                    # Reset if no color detected
                    if self.last_color != "unknown":
                        self.last_color = "unknown"
                        self.publish_status("🔍 Scanning for objects...")
                        
        except Exception as e:
            self.get_logger().error(f"Camera callback error: {e}")

    def process_color_detection(self, color):
        """Process detected color with spatial database"""
        
        self.get_logger().info(f"🎯 Physical sensor detects {color.upper()}")
        
        # Publish color detection
        color_msg = String()
        color_msg.data = color
        self.color_publisher.publish(color_msg)
        
        # Query spatial database
        location_data = self.spatial_db.query_object_location(color)
        
        if location_data:
            angle = location_data['angle']
            confidence = location_data['confidence']
            
            self.get_logger().info(f"🗃️  Environmental database: {color.upper()} typically at {angle}° (confidence: {confidence:.2f})")
            
            # Move head to predicted location
            self.move_head_to_angle(angle)
            
            # Update database with successful detection
            self.spatial_db.update_object_success(color, angle)
            
            self.publish_status(f"🎯 Target acquired: {color.upper()} at {angle}°")
            
        else:
            self.get_logger().info(f"❓ No spatial data for {color.upper()}")
            self.publish_status(f"🔍 Learning new object: {color.upper()}")

    def move_head_to_angle(self, angle_degrees):
        """Move CORI's head to specified angle with physics wake-up if needed"""
        
        # Check if physics needs wake-up (if it's been a while since last movement)
        current_time = time.time()
        if not self.physics_awakened or (current_time - self.last_movement_time) > 10:
            self.get_logger().info("🔌 Physics may be sleeping, sending wake-up sequence...")
            self.wake_up_physics()
        
        # Convert degrees to radians with scaling for visible movement
        # Make movements more dramatic for demo purposes
        angle_rad = math.radians(angle_degrees * 0.03)  # Scale down for safety
        
        self.get_logger().info(f"🤖 Turning head to {angle_degrees}° ({angle_rad:.3f} rad)")
        
        # Send movement command
        msg = Float64()
        msg.data = angle_rad
        self.joint_publisher.publish(msg)
        
        # Update tracking
        self.current_head_angle = angle_rad
        self.last_movement_time = current_time
        
        # Publish status
        direction = "right" if angle_degrees > 0 else "left" if angle_degrees < 0 else "straight"
        self.publish_status(f"🤖 Turning head to {direction} ({angle_degrees}°)")

    def publish_status(self, message):
        """Publish status message"""
        status_msg = String()
        status_msg.data = message
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        sensor_fusion_demo = SensorFusionDemo()
        rclpy.spin(sensor_fusion_demo)
        
    except KeyboardInterrupt:
        print("\n🛑 Sensor fusion demo stopped by user")
        
    except Exception as e:
        print(f"❌ Error in sensor fusion demo: {e}")
        
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()