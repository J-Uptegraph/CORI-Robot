#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String, Bool, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
from collections import deque
import threading
import queue


class ProductionLaundryDetector(Node):
    def __init__(self):
        super().__init__('production_laundry_detector')
        self.get_logger().info('Production Laundry Detector v1.0 - Starting')
        
        # Core components
        self.bridge = CvBridge()
        self.setup_parameters()
        self.setup_vision_pipeline()
        self.setup_robot_interface()
        self.setup_performance_monitoring()
        
        # ROS interfaces
        self.setup_ros_interfaces()
        
        self.get_logger().info('Production Laundry Detector ready')

    def setup_parameters(self):
        """Production parameters - simple and effective"""
        self.declare_parameter('detection_fps', 10.0)
        self.declare_parameter('min_area', 3000)
        self.declare_parameter('max_area', 50000)
        self.declare_parameter('confidence_threshold', 0.75)
        self.declare_parameter('use_background_subtraction', True)
        self.declare_parameter('color_stability_frames', 5)
        
        self.detection_fps = self.get_parameter('detection_fps').value
        self.min_area = self.get_parameter('min_area').value
        self.max_area = self.get_parameter('max_area').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.use_background_subtraction = self.get_parameter('use_background_subtraction').value
        self.color_stability_frames = self.get_parameter('color_stability_frames').value

    def setup_vision_pipeline(self):
        """Efficient vision pipeline"""
        # Background subtraction for motion detection
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=16, detectShadows=True
        )
        
        # Color analysis
        self.color_history = deque(maxlen=self.color_stability_frames)
        
        # Frame management
        self.latest_rgb = None
        self.latest_depth = None
        self.frame_count = 0
        self.last_detection_time = 0
        
        # Detection state
        self.current_detections = []
        self.pick_queue = queue.Queue()
        self.processing_lock = threading.Lock()

    def setup_robot_interface(self):
        """Simple robot interface"""
        self.robot_busy = False
        self.pick_success_rate = 0.85
        self.items_processed = 0
        self.session_start = time.time()

    def setup_performance_monitoring(self):
        """Performance tracking"""
        self.frame_times = deque(maxlen=50)
        self.detection_times = deque(maxlen=50)
        self.avg_fps = 0.0
        self.shutdown_requested = False

    def setup_ros_interfaces(self):
        """Clean ROS interface"""
        # Inputs
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        
        # Outputs
        self.pick_target_pub = self.create_publisher(PoseStamped, '/pick_target', 5)
        self.sort_command_pub = self.create_publisher(String, '/sort_command', 5)
        self.status_pub = self.create_publisher(String, '/detector_status', 5)
        
        # Control
        self.start_sub = self.create_subscription(Bool, '/start_sorting', self.start_callback, 1)
        self.robot_status_sub = self.create_subscription(String, '/robot_status', self.robot_status_callback, 5)
        
        # Main processing timer
        detection_period = 1.0 / self.detection_fps
        self.detection_timer = self.create_timer(detection_period, self.detection_callback)
        self.display_timer = self.create_timer(0.033, self.display_callback)  # 30 FPS display

    def rgb_callback(self, msg):
        """High-performance RGB processing"""
        if self.shutdown_requested:
            return
            
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Basic preprocessing
            frame = cv2.bilateralFilter(frame, 5, 50, 50)  # Noise reduction
            
            with self.processing_lock:
                self.latest_rgb = frame
                self.frame_count += 1
                
        except Exception as e:
            if not self.shutdown_requested:
                self.get_logger().error(f"RGB callback error: {e}")

    def depth_callback(self, msg):
        """Efficient depth processing"""
        if self.shutdown_requested:
            return
            
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            
            # Convert to meters and filter
            depth_m = depth.astype(np.float32) / 1000.0
            depth_m[depth_m == 0] = np.nan
            depth_m[depth_m > 1.5] = np.nan  # 1.5m max range
            
            with self.processing_lock:
                self.latest_depth = depth_m
                
        except Exception as e:
            if not self.shutdown_requested:
                self.get_logger().error(f"Depth callback error: {e}")

    def detection_callback(self):
        """Main detection processing - optimized for speed"""
        if self.shutdown_requested:
            return
            
        start_time = time.time()
        
        # Skip if no new frames or robot is busy
        if self.latest_rgb is None or self.robot_busy:
            return
        
        with self.processing_lock:
            rgb_frame = self.latest_rgb.copy()
            depth_frame = self.latest_depth.copy() if self.latest_depth is not None else None
        
        # Detect garments
        detections = self.detect_garments(rgb_frame, depth_frame)
        
        # Process detections
        if detections:
            # Sort by confidence and pickability
            detections.sort(key=lambda x: x['total_score'], reverse=True)
            
            # Update current detections
            self.current_detections = detections
            
            # Send pick command for best target
            if not self.robot_busy and detections[0]['total_score'] > self.confidence_threshold:
                self.send_pick_command(detections[0])
        
        # Performance tracking
        detection_time = time.time() - start_time
        self.detection_times.append(detection_time)
        self.last_detection_time = time.time()
        
        # Update FPS
        if self.frame_times:
            self.avg_fps = len(self.frame_times) / sum(self.frame_times)

    def detect_garments(self, rgb_frame, depth_frame):
        """Efficient garment detection"""
        detections = []
        
        # Method 1: Background subtraction for new items
        if self.use_background_subtraction:
            motion_detections = self.detect_motion_objects(rgb_frame)
            detections.extend(motion_detections)
        
        # Method 2: Static detection for existing items
        static_detections = self.detect_static_objects(rgb_frame, depth_frame)
        detections.extend(static_detections)
        
        # Remove duplicates
        detections = self.remove_overlapping_detections(detections)
        
        return detections

    def detect_motion_objects(self, frame):
        """Fast motion-based detection"""
        # Apply background subtraction
        fg_mask = self.bg_subtractor.apply(frame)
        
        # Remove shadows
        fg_mask[fg_mask == 127] = 0
        
        # Morphological cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for contour in contours:
            detection = self.analyze_contour(contour, frame, 'motion')
            if detection:
                detections.append(detection)
        
        return detections

    def detect_static_objects(self, frame, depth_frame):
        """Efficient static object detection"""
        detections = []
        
        # Edge detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        
        # Morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            detection = self.analyze_contour(contour, frame, 'static', depth_frame)
            if detection:
                detections.append(detection)
        
        return detections

    def analyze_contour(self, contour, frame, detection_type, depth_frame=None):
        """Fast contour analysis"""
        # Basic filtering
        area = cv2.contourArea(contour)
        if area < self.min_area or area > self.max_area:
            return None
        
        # Bounding rectangle
        x, y, w, h = cv2.boundingRect(contour)
        
        # Aspect ratio check
        aspect_ratio = w / h
        if aspect_ratio > 4 or aspect_ratio < 0.25:
            return None
        
        # Extract region of interest
        roi = frame[y:y+h, x:x+w]
        if roi.size == 0:
            return None
        
        # Color analysis
        color_result = self.analyze_color_fast(roi)
        
        # Depth analysis (if available)
        depth_score = 0.5
        pickable_point = None
        
        if depth_frame is not None:
            depth_result = self.analyze_depth_fast(x, y, w, h, depth_frame)
            depth_score = depth_result['confidence']
            pickable_point = depth_result['pick_point']
        
        # Shape analysis
        shape_score = self.analyze_shape_fast(contour, area, w, h)
        
        # Detection type bonus
        type_bonus = 0.2 if detection_type == 'motion' else 0.0
        
        # Total confidence
        total_score = (color_result['confidence'] * 0.4 + 
                      depth_score * 0.3 + 
                      shape_score * 0.2 + 
                      type_bonus * 0.1)
        
        return {
            'contour': contour,
            'bbox': (x, y, w, h),
            'area': area,
            'color_name': color_result['color_name'],
            'pile': color_result['pile'],
            'color_confidence': color_result['confidence'],
            'depth_score': depth_score,
            'shape_score': shape_score,
            'total_score': total_score,
            'detection_type': detection_type,
            'pickable_point': pickable_point
        }

    def analyze_color_fast(self, roi):
        """Fast color analysis"""
        # Sample center region
        h, w = roi.shape[:2]
        center_h, center_w = h//2, w//2
        sample_size = min(w//4, h//4, 15)
        
        if sample_size < 3:
            center_roi = roi
        else:
            center_roi = roi[center_h-sample_size:center_h+sample_size,
                           center_w-sample_size:center_w+sample_size]
        
        # Get dominant color
        pixels = center_roi.reshape(-1, 3)
        dominant_color = np.median(pixels, axis=0).astype(int)
        
        # Classify color
        return self.classify_color_production(dominant_color)

    def classify_color_production(self, bgr_color):
        """Production color classification - fast and reliable"""
        # Convert to HSV
        hsv = cv2.cvtColor(np.uint8([[bgr_color]]), cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv
        
        confidence = 0.8  # Base confidence
        
        # White detection
        if s < 25 and v > 180:
            return {'color_name': 'white', 'pile': 'lights', 'confidence': 0.95}
        
        # Black detection
        if v < 50:
            return {'color_name': 'black', 'pile': 'darks', 'confidence': 0.9}
        
        # Gray detection
        if s < 35:
            if v > 140:
                return {'color_name': 'light_gray', 'pile': 'lights', 'confidence': 0.85}
            else:
                return {'color_name': 'gray', 'pile': 'darks', 'confidence': 0.85}
        
        # Color classification
        hue_360 = h * 2
        
        # Define color ranges
        if hue_360 < 20 or hue_360 >= 340:
            color_name, pile = 'red', 'colors'
        elif 20 <= hue_360 < 45:
            color_name, pile = 'orange', 'colors'
        elif 45 <= hue_360 < 65:
            # Yellow special case
            if v > 150:
                color_name, pile = 'yellow', 'lights'
            else:
                color_name, pile = 'yellow', 'colors'
        elif 65 <= hue_360 < 120:
            color_name, pile = 'green', 'colors'
        elif 120 <= hue_360 < 180:
            color_name, pile = 'cyan', 'colors'
        elif 180 <= hue_360 < 240:
            if v < 80:  # Dark blue
                color_name, pile = 'navy', 'darks'
            else:
                color_name, pile = 'blue', 'colors'
        elif 240 <= hue_360 < 300:
            if v < 90:  # Dark purple
                color_name, pile = 'purple', 'darks'
            else:
                color_name, pile = 'purple', 'colors'
        else:
            color_name, pile = 'pink', 'colors'
        
        # Adjust confidence based on saturation
        if s > 100:
            confidence = 0.9
        elif s < 50:
            confidence = 0.6
        
        return {'color_name': color_name, 'pile': pile, 'confidence': confidence}

    def analyze_depth_fast(self, x, y, w, h, depth_frame):
        """Fast depth analysis"""
        # Extract depth ROI
        depth_roi = depth_frame[y:y+h, x:x+w]
        valid_depths = depth_roi[~np.isnan(depth_roi)]
        
        if len(valid_depths) == 0:
            return {'confidence': 0.3, 'pick_point': None}
        
        # Calculate depth statistics
        mean_depth = np.mean(valid_depths)
        min_depth = np.min(valid_depths)  # Closest point (highest)
        depth_variance = np.var(valid_depths)
        
        # Confidence based on depth quality
        if depth_variance < 0.01:  # Good depth consistency
            confidence = 0.9
        elif depth_variance < 0.05:
            confidence = 0.7
        else:
            confidence = 0.4
        
        # Pick point at center of highest region
        center_x = x + w // 2
        center_y = y + h // 2
        pick_point = self.pixel_to_world(center_x, center_y, min_depth)
        
        return {'confidence': confidence, 'pick_point': pick_point}

    def analyze_shape_fast(self, contour, area, w, h):
        """Fast shape analysis"""
        # Aspect ratio score
        aspect_ratio = w / h
        if 0.5 <= aspect_ratio <= 2.0:  # Reasonable clothing ratios
            aspect_score = 1.0
        else:
            aspect_score = 0.5
        
        # Solidity score
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area if hull_area > 0 else 0
        
        if solidity > 0.6:  # Reasonably solid
            solidity_score = 1.0
        else:
            solidity_score = 0.6
        
        return (aspect_score + solidity_score) / 2

    def pixel_to_world(self, u, v, depth):
        """Simple pixel to world conversion"""
        # Simplified camera parameters
        fx, fy = 525.0, 525.0  # Replace with actual calibration
        cx, cy = 320.0, 240.0
        
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        
        return [x, y, z]

    def remove_overlapping_detections(self, detections):
        """Remove overlapping detections"""
        if len(detections) <= 1:
            return detections
        
        # Sort by confidence
        detections.sort(key=lambda x: x['total_score'], reverse=True)
        
        filtered = []
        for detection in detections:
            x1, y1, w1, h1 = detection['bbox']
            
            overlap = False
            for existing in filtered:
                x2, y2, w2, h2 = existing['bbox']
                
                # Calculate overlap
                overlap_x = max(0, min(x1 + w1, x2 + w2) - max(x1, x2))
                overlap_y = max(0, min(y1 + h1, y2 + h2) - max(y1, y2))
                overlap_area = overlap_x * overlap_y
                
                union_area = w1 * h1 + w2 * h2 - overlap_area
                iou = overlap_area / union_area if union_area > 0 else 0
                
                if iou > 0.3:  # 30% overlap threshold
                    overlap = True
                    break
            
            if not overlap:
                filtered.append(detection)
        
        return filtered

    def send_pick_command(self, detection):
        """Send pick command to robot"""
        if detection['pickable_point'] is None:
            return
        
        # Create pose message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        
        pick_point = detection['pickable_point']
        pose_msg.pose.position.x = pick_point[0]
        pose_msg.pose.position.y = pick_point[1]
        pose_msg.pose.position.z = pick_point[2] - 0.03  # Approach from 3cm above
        
        # Default orientation (gripper pointing down)
        pose_msg.pose.orientation.w = 1.0
        
        # Publish pick target
        self.pick_target_pub.publish(pose_msg)
        
        # Publish sort command
        sort_msg = String()
        sort_msg.data = detection['pile']
        self.sort_command_pub.publish(sort_msg)
        
        # Update state
        self.robot_busy = True
        
        self.get_logger().info(f"Pick command sent: {detection['color_name']} -> {detection['pile']} "
                              f"(confidence: {detection['total_score']:.2f})")

    def start_callback(self, msg):
        """Start/stop sorting"""
        if msg.data:
            self.get_logger().info("Starting laundry sorting operation")
            self.session_start = time.time()
            self.items_processed = 0
        else:
            self.get_logger().info("Stopping laundry sorting operation")

    def robot_status_callback(self, msg):
        """Handle robot status updates"""
        status = msg.data
        
        if status == 'pick_complete':
            self.robot_busy = False
            self.items_processed += 1
            self.get_logger().info(f"Pick completed. Total items: {self.items_processed}")
        elif status == 'pick_failed':
            self.robot_busy = False
            self.get_logger().warn("Pick operation failed")
        elif status in ['moving', 'picking', 'sorting']:
            self.robot_busy = True

    def display_callback(self):
        """Display results"""
        if self.latest_rgb is None or self.shutdown_requested:
            return
        
        # Create visualization
        vis_frame = self.latest_rgb.copy()
        
        # Draw detections
        for i, detection in enumerate(self.current_detections):
            self.draw_detection(vis_frame, detection, i == 0)
        
        # Draw status
        self.draw_status(vis_frame)
        
        # Show frame
        cv2.imshow('Production Laundry Detector v1.0', vis_frame)
        
        # Handle keyboard
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            self.get_logger().info("Shutdown requested by user")
            self.shutdown_requested = True
        elif key == ord(' '):  # Space - toggle background learning
            self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
                history=500, varThreshold=16, detectShadows=True
            )
            self.get_logger().info("Background model reset")

    def draw_detection(self, frame, detection, is_primary=False):
        """Draw detection visualization"""
        contour = detection['contour']
        x, y, w, h = detection['bbox']
        
        # Color coding
        if detection['total_score'] > 0.8:
            color = (0, 255, 0)  # Green - high confidence
        elif detection['total_score'] > 0.6:
            color = (0, 255, 255)  # Yellow - medium confidence
        else:
            color = (0, 165, 255)  # Orange - low confidence
        
        thickness = 3 if is_primary else 2
        
        # Draw contour
        cv2.drawContours(frame, [contour], -1, color, thickness)
        
        # Draw bounding box
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 1)
        
        # Label
        label = f"{detection['color_name'].upper()} -> {detection['pile'].upper()}"
        conf_label = f"Conf: {detection['total_score']:.2f}"
        type_label = f"Type: {detection['detection_type']}"
        
        # Label background
        labels = [label, conf_label, type_label]
        max_width = max([cv2.getTextSize(l, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0] for l in labels])
        
        bg_height = len(labels) * 20 + 10
        cv2.rectangle(frame, (x, y - bg_height), (x + max_width + 10, y), (0, 0, 0), -1)
        
        # Draw text
        for i, text in enumerate(labels):
            cv2.putText(frame, text, (x + 5, y - bg_height + 20 + i * 18), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def draw_status(self, frame):
        """Draw system status"""
        h, w = frame.shape[:2]
        
        # Status panel
        panel_h = 100
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (w, panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.7, frame, 0.3, 0, frame)
        
        # Status text
        status_lines = [
            f"Production Laundry Detector v1.0",
            f"FPS: {self.avg_fps:.1f} | Detections: {len(self.current_detections)}",
            f"Items Processed: {self.items_processed} | Robot: {'BUSY' if self.robot_busy else 'READY'}",
            f"Session Time: {time.time() - self.session_start:.0f}s"
        ]
        
        for i, line in enumerate(status_lines):
            cv2.putText(frame, line, (10, 25 + i * 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Performance indicators
        indicators = [
            ('RGB', (0, 255, 0) if self.latest_rgb is not None else (0, 0, 255)),
            ('DEPTH', (0, 255, 0) if self.latest_depth is not None else (0, 0, 255)),
            ('ROBOT', (0, 255, 0) if not self.robot_busy else (255, 255, 0))
        ]
        
        for i, (name, color) in enumerate(indicators):
            x = w - 200 + i * 60
            cv2.circle(frame, (x, 30), 8, color, -1)
            cv2.putText(frame, name, (x - 15, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    def publish_status(self):
        """Publish system status"""
        status = {
            'detections': len(self.current_detections),
            'items_processed': self.items_processed,
            'robot_busy': self.robot_busy,
            'avg_fps': self.avg_fps,
            'session_time': time.time() - self.session_start
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """Clean shutdown of the node"""
        try:
            self.get_logger().info("Cleaning up resources...")
            self.shutdown_requested = True
            
            # Stop timers
            if hasattr(self, 'detection_timer'):
                self.detection_timer.cancel()
            if hasattr(self, 'display_timer'):
                self.display_timer.cancel()
            
            # Clean up OpenCV
            cv2.destroyAllWindows()
            
            # Call parent destroy
            super().destroy_node()
            
        except Exception as e:
            print(f"Error during node cleanup: {e}")


def main():
    rclpy.init()
    detector = ProductionLaundryDetector()
    
    try:
        # Check for shutdown request in the spin loop
        while rclpy.ok() and not detector.shutdown_requested:
            rclpy.spin_once(detector, timeout_sec=0.1)
        
        if detector.shutdown_requested:
            detector.get_logger().info("Shutting down gracefully...")
            
    except KeyboardInterrupt:
        detector.get_logger().info("Keyboard interrupt received")
    except Exception as e:
        detector.get_logger().error(f"Unexpected error: {e}")
    finally:
        try:
            detector.destroy_node()
        except Exception as e:
            print(f"Error destroying node: {e}")
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            print(f"Error during RCL shutdown: {e}")


if __name__ == '__main__':
    main()