import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import time
from collections import Counter

class LaundryColorDetector(Node):
    def __init__(self):
        super().__init__('laundry_color_detector')
        self.get_logger().info('Initializing Laundry Color Detector...')

        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open webcam.')
        else:
            self.get_logger().info('Webcam initialized with full color.')

        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        self.class_counts = {
            'lights': 0,
            'darks': 0,
            'colors': 0,
            'unknown': 0
        }
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(history=300, varThreshold=50)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame from webcam.')
            return

        frame = cv2.resize(frame, (1920, 1080))
        fg_mask = self.bg_subtractor.apply(frame)
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        output_frame = frame.copy()

        self.class_counts = {k: 0 for k in self.class_counts}

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w * h < 800:
                continue

            roi = frame[y:y+h, x:x+w]
            avg_color = np.mean(roi.reshape(-1, 3), axis=0).astype(int)
            color_name = self.get_color_name(avg_color)
            pile = self.classify_pile(avg_color, color_name)
            self.class_counts[pile] += 1

            color_box = (int(avg_color[0]), int(avg_color[1]), int(avg_color[2]))
            label = f"{color_name.upper()} → {pile.upper()}"
            cv2.rectangle(output_frame, (x, y), (x+w, y+h), color_box, 2)
            cv2.putText(output_frame, label, (x, y - 10), self.font, 0.7, color_box, 2)

        self.draw_summary_box(output_frame)
        cv2.imshow('CORI Laundry Sorter [LIVE]', output_frame)
        cv2.waitKey(1)

    def classify_pile(self, bgr, name):
        if name in ['white', 'yellow', 'light_gray']:
            return 'lights'
        elif name in ['black', 'navy', 'dark_gray', 'brown', 'gray', 'purple']:
            return 'darks'
        elif name in ['red', 'orange', 'green', 'blue', 'cyan', 'pink']:
            return 'colors'
        else:
            return 'unknown'

    def get_color_name(self, bgr):
        hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv

        if v < 40:
            return 'black'
        if s < 40 and v > 180:
            return 'white'
        if s < 40:
            return 'gray'

        if h < 10 or h >= 170:
            return 'red'
        elif 10 <= h < 25:
            return 'orange'
        elif 25 <= h < 35:
            return 'yellow'
        elif 35 <= h < 85:
            return 'green'
        elif 85 <= h < 125:
            return 'cyan'
        elif 125 <= h < 145:
            return 'blue'
        elif 145 <= h < 170:
            return 'purple'
        else:
            return 'unknown'

    def draw_summary_box(self, frame):
        x, y, w, h = 1600, 30, 300, 200
        overlay = frame.copy()
        cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        lines = [
            f"Total Articles: {sum(self.class_counts.values())}",
            f"Lights: {self.class_counts['lights']}",
            f"Darks: {self.class_counts['darks']}",
            f"Colors: {self.class_counts['colors']}",
            f"Unknown: {self.class_counts['unknown']}"
        ]

        for i, line in enumerate(lines):
            cv2.putText(frame, line, (x + 15, y + 35 + i*30), self.font, 0.8, (255, 255, 255), 2)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main():
    rclpy.init()
    detector = LaundryColorDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
