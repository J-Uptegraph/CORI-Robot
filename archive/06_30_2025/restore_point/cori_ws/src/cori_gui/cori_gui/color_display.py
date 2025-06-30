#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class ColorDisplay(Node):
    def __init__(self):
        super().__init__('color_display')
        
        # Subscribe to color detection
        self.subscription = self.create_subscription(
            String,
            '/cori/color_detected',
            self.color_callback,
            10
        )
        
        self.get_logger().info('Color Display Ready - Listening for colors...')
        self.print_header()

    def print_header(self):
        """Print a nice header"""
        os.system('clear')  # Clear terminal
        print("=" * 60)
        print("CORI ROBOT COLOR DETECTION SYSTEM")
        print("=" * 60)
        print("Looking at the world through CORI's camera...")
        print()

    def color_callback(self, msg):
        """Display detected color with fancy formatting"""
        color = msg.data
        
        # Color mappings for terminal colors
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
            'ORANGE': '\033[33m'
        }
        
        # Get pile classification
        pile = self.classify_pile(color)
        
        # Color code for display
        color_code = color_codes.get(color, '\033[0m')
        reset_code = '\033[0m'
        
        # Clear and redraw
        self.print_header()
        
        print(f"CURRENT DETECTION:")
        print(f"   Color: {color_code}{color}{reset_code}")
        print(f"   Pile:  {pile.upper()}")
        print()
        print("Hold different colored objects in front of CORI's camera!")
        print("Press Ctrl+C to stop")

    def classify_pile(self, color_name):
        """Classify color into laundry piles"""
        if color_name in ['WHITE', 'YELLOW']:
            return 'lights'
        elif color_name in ['BLACK', 'GRAY', 'PURPLE']:
            return 'darks'
        elif color_name in ['RED', 'ORANGE', 'GREEN', 'BLUE', 'CYAN']:
            return 'colors'
        return 'unknown'

def main():
    rclpy.init()
    node = ColorDisplay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nGoodbye!")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
