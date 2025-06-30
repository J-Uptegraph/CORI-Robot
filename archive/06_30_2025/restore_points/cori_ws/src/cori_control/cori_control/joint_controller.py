#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class InteractiveJointController(Node):
    def __init__(self):
        super().__init__('interactive_joint_controller')
        
        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.publish_joints)  # 10 Hz
        
        # Joint positions (you can modify these!)
        self.joint_positions = {
            'head_joint': 0.0,
            'left_shoulder_joint': 0.0,
            'left_elbow_joint': 0.0,
            'left_wrist_joint': 0.0,
            'right_shoulder_joint': 0.0,
            'right_elbow_joint': 0.0,
            'right_wrist_joint': 0.0,
            'left_hip_joint': 0.0,
            'left_knee_joint': 0.0,
            'left_ankle_joint': 0.0,
            'right_hip_joint': 0.0,
            'right_knee_joint': 0.0,
            'right_ankle_joint': 0.0
        }
        
        self.get_logger().info("🤖 CORI Interactive Controller Ready!")
        self.get_logger().info("🎮 Manual control active - move CORI freely!")
        
    def publish_joints(self):
        """Publish current joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        msg.velocity = [0.0] * len(self.joint_positions)
        msg.effort = [0.0] * len(self.joint_positions)
        
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = InteractiveJointController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\n🛑 Stopping CORI Controller...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
