import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time


class ColorClassifier:
    def __init__(self):
        pass

    def get_color_name(self, bgr):
        hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv

        if v < 50:
            return 'black'
        if s < 50 and v > 200:
            return 'white'
        if s < 50:
            return 'gray'

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
        if color_name in ['white', 'yellow', 'light_gray']:
            return 'lights'
        elif color_name in ['black', 'navy', 'dark_gray', 'brown', 'gray', 'purple']:
            return 'darks'
        elif color_name in ['red', 'orange', 'green', 'blue', 'cyan', 'pink']:
            return 'colors'
        return 'unknown'


class LaundryColorDetector(Node):
    def __init__(self):
        super().__init__('laundry_color_detector')
        self.get_logger().info('Starting Laundry Color Detector...')

        self.declare_parameter('min_area', 1500)
        self.min_area = self.get_parameter('min_area').value

        self.classifier = ColorClassifier()
        self.bridge = CvBridge()
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(history=300, varThreshold=50)

        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.image_buffer = None
        self.class_counts = {'lights': 0, 'darks': 0, 'colors': 0, 'unknown': 0}
        self.font = cv2.FONT_HERSHEY_SIMPLEX

        self.detecting = False
        self.button_state = 'detect'
        self.rerun_button = None
        self.latest_frame = None

        threading.Thread(target=self.display_loop, daemon=True).start()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            frame = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)
            self.latest_frame = frame
            if self.detecting:
                self.image_buffer = frame.copy()
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

    def start_detection(self):
        if not self.latest_frame.any():
            return

        self.detecting = True
        time.sleep(2)  # Capture buffer
        if self.image_buffer is not None:
            self.process_frame(self.image_buffer)
        self.detecting = False
        self.button_state = 'clear'

    def clear_detection(self):
        self.class_counts = {k: 0 for k in self.class_counts}
        self.button_state = 'detect'
        self.image_buffer = None

    def process_frame(self, frame):
        fg_mask = self.bg_subtractor.apply(frame)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
        fg_mask = cv2.dilate(fg_mask, kernel, iterations=1)

        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        self.class_counts = {k: 0 for k in self.class_counts}

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            if w * h < self.min_area:
                continue
            roi = frame[y:y+h, x:x+w]
            avg_color = np.mean(roi.reshape(-1, 3), axis=0).astype(np.uint8)
            color_name = self.classifier.get_color_name(avg_color)
            pile = self.classifier.classify_pile(color_name)
            self.class_counts[pile] += 1
            cv2.rectangle(frame, (x, y), (x+w, y+h), tuple(int(c) for c in avg_color), 2)
            cv2.putText(frame, f"{color_name.upper()} -> {pile.upper()}", (x, y - 10), self.font, 0.7, (255, 255, 255), 2)

    def draw_summary_box(self, frame):
        x, y, w, h = 30, 30, 320, 200
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
            cv2.putText(frame, line, (x + 15, y + 35 + i * 30), self.font, 0.8, (255, 255, 255), 2)

    def draw_button(self, frame):
        text = 'DETECT OBJECTS' if self.button_state == 'detect' else 'CLEAR'
        x, y, w, h = frame.shape[1] - 200, frame.shape[0] - 70, 180, 50
        color = (0, 100, 200) if self.button_state == 'detect' else (200, 50, 50)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, -1, cv2.LINE_AA)
        cv2.putText(frame, text, (x + 10, y + 35), self.font, 0.9, (255, 255, 255), 2)
        self.rerun_button = (x, y, x + w, y + h)

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.rerun_button:
            x1, y1, x2, y2 = self.rerun_button
            if x1 <= x <= x2 and y1 <= y <= y2:
                if self.button_state == 'detect':
                    threading.Thread(target=self.start_detection, daemon=True).start()
                else:
                    self.clear_detection()

    def display_loop(self):
        while True:
            if self.latest_frame is None:
                continue
            display = self.latest_frame.copy()
            self.draw_summary_box(display)
            self.draw_button(display)
            cv2.imshow('CORI Laundry Sorter [Interactive]', display)
            cv2.setMouseCallback('CORI Laundry Sorter [Interactive]', self.on_mouse_click)
            if cv2.waitKey(30) == 27:
                break

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = LaundryColorDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
