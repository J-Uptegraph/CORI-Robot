#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class SimpleColorDetector(Node):
    def __init__(self):
        super().__init__('simple_color_detector')
        self.get_logger().info('CORI Color Detector Starting...')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Subscribe to real webcam
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Real webcam topic
            self.image_callback,
            10
        )
        
        # Publisher for color detection results
        self.color_publisher = self.create_publisher(String, '/cori/color_detected', 10)
        
        # Detection variables
        self.last_color = "No Color Detected"
        self.frame_count = 0
        
        self.get_logger().info('Color detector ready! Looking for colors...')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Process every 10th frame for performance
            self.frame_count += 1
            if self.frame_count % 10 != 0:
                return
            
            # Get center region for color detection
            height, width = cv_image.shape[:2]
            center_x, center_y = width // 2, height // 2
            roi_size = 50
            
            # Extract center region
            roi = cv_image[center_y-roi_size:center_y+roi_size, 
                          center_x-roi_size:center_x+roi_size]
            
            if roi.size == 0:
                return
            
            # Get dominant color
            dominant_color = self.get_dominant_color(roi)
            
            # Classify color
            color_name = self.classify_color(dominant_color)
            pile = self.classify_pile(color_name)
            
            # Only publish if color changed
            current_detection = f"{color_name.upper()}"
            if current_detection != self.last_color:
                self.last_color = current_detection
                
                # Publish to ROS topic for Gazebo UI
                color_msg = String()
                color_msg.data = current_detection
                self.color_publisher.publish(color_msg)
                
                # Print to terminal with pile classification
                self.get_logger().info(f"DETECTED: {color_name.upper()} -> {pile.upper()}")
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def get_dominant_color(self, roi):
        """Get the dominant color in the region of interest"""
        # Reshape to list of pixels
        pixels = roi.reshape(-1, 3)
        
        # Use median for robust color detection
        dominant_color = np.median(pixels, axis=0).astype(int)
        return dominant_color

    def classify_color(self, bgr_color):
        """Classify BGR color to color name - using your existing logic"""
        # Convert to HSV for better color classification
        hsv = cv2.cvtColor(np.uint8([[bgr_color]]), cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv

        # White detection
        if s < 30 and v > 200:
            return 'white'
        
        # Black detection
        if v < 50:
            return 'black'
        
        # Gray detection
        if s < 50:
            return 'gray'

        # Color classification based on hue
        if h < 10 or h >= 170:
            return 'red'
        elif 10 <= h < 25:
            return 'orange'
        elif 25 <= h < 40:
            return 'yellow'
        elif 40 <= h < 85:
            return 'green'
        elif 85 <= h < 130:
            return 'cyan'
        elif 130 <= h < 150:
            return 'blue'
        elif 150 <= h < 170:
            return 'purple'
        
        return 'unknown'

    def classify_pile(self, color_name):
        """Classify color into laundry piles - using your existing logic"""
        if color_name in ['white', 'yellow']:
            return 'lights'
        elif color_name in ['black', 'gray', 'purple']:
            return 'darks'
        elif color_name in ['red', 'orange', 'green', 'blue', 'cyan']:
            return 'colors'
        return 'unknown'

def main():
    rclpy.init()
    node = SimpleColorDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()