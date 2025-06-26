#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math
import time

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        
        # Publishers for CORI's ONLY movable joint: head_joint
        self.head_pub = self.create_publisher(Float64, '/model/cori/joint/head_joint/cmd_pos', 10)
        
        # Alternative topic formats to try
        self.head_pub2 = self.create_publisher(Float64, '/cori/head_joint/cmd_pos', 10)
        self.head_pub3 = self.create_publisher(Float64, '/head_joint/cmd_pos', 10)
        self.head_pub4 = self.create_publisher(Float64, '/joint_states/head_joint', 10)
        
        # Simple movement variables
        self.time_start = time.time()
        
        # Timer for smooth movement
        self.timer = self.create_timer(0.1, self.move_head)  # 10 Hz
        
        self.get_logger().info("🎯 Object Detection Ready!")
        self.get_logger().info("🔍 CORI scanning for objects - head moving left/right and up/down")
        self.get_logger().info("⏰ Pattern: 4 seconds left/right, 4 seconds up/down")
        
    def move_head(self):
        """Simple head movement pattern"""
        current_time = time.time() - self.time_start
        cycle_time = current_time % 8.0  # 8 second cycle
        
        if cycle_time < 4.0:
            # First 4 seconds: Rotate head left and right (Z-axis rotation)
            head_angle = math.sin(cycle_time * math.pi / 2) * 3.0  # -3.0 to +3.0 radians (HUGE!)
        else:
            # Next 4 seconds: Different rotation pattern
            head_angle = math.sin((cycle_time - 4.0) * math.pi / 1) * 2.0  # Faster oscillation
        
        # Create message for the single head joint
        head_msg = Float64()
        head_msg.data = head_angle
        
        # Publish to all possible topics for head_joint
        self.head_pub.publish(head_msg)
        self.head_pub2.publish(head_msg)
        self.head_pub3.publish(head_msg)
        self.head_pub4.publish(head_msg)
        
        # Log current position every half second for more feedback
        if int(current_time * 2) % 1 == 0 and int(current_time * 20) % 10 == 0:
            if cycle_time < 4.0:
                self.get_logger().info(f"🔍 HEAD_JOINT ROTATION - Angle: {head_angle:.2f} (±3.0 rad)")
            else:
                self.get_logger().info(f"🔍 HEAD_JOINT FAST SCAN - Angle: {head_angle:.2f} (±2.0 rad)")

def main(args=None):
    rclpy.init(args=args)
    mover = ObjectDetection()
    
    try:
        rclpy.spin(mover)
    except KeyboardInterrupt:
        print("\n🛑 Stopping Object Detection...")
    finally:
        mover.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
