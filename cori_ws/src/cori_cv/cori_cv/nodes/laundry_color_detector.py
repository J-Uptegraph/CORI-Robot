import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sklearn.cluster import KMeans
import time

class LaundryColorDetector(Node):
    def __init__(self):
        super().__init__('laundry_color_detector')
        self.get_logger().info('🚀 Laundry Color Detector Initialized')

        # Attempt to connect to webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('Failed to open webcam.')
            return

        # Turn up exposure if supported
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # Manual mode
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -4)         # Value may vary based on camera driver

        # Timer for frame processing
        self.timer = self.create_timer(0.1, self.process_frame)

        # For temporal confidence tracking
        self.last_detected = None
        self.detection_start_time = None

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Webcam frame not available.')
            return

        # Preprocess image to find contours
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)
        edged = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_detection = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 5000:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            roi = frame[y:y+h, x:x+w]

            color = self.estimate_color(roi)
            color_name, pile = self.classify_color(color)

            aspect_ratio = h / float(w)
            clothing_type = 'Unknown'

            if area < 12000:
                clothing_type = 'Sock'
            elif 0.8 < aspect_ratio < 1.2:
                clothing_type = 'Shirt'
            elif aspect_ratio >= 1.5 and area > 30000:
                clothing_type = 'Pants'
            elif aspect_ratio < 0.8 and area > 20000:
                clothing_type = 'Shorts'
            elif area > 40000 and aspect_ratio < 1.5:
                clothing_type = 'Sweatshirt'
            elif aspect_ratio > 1.5 and area > 40000:
                clothing_type = 'Sweatpants'

            confidence = area / (frame.shape[0] * frame.shape[1]) * 100
            if confidence < 1:
                continue

            best_detection = {
                'x': x, 'y': y, 'w': w, 'h': h,
                'label': f"{clothing_type} | {color_name} | {confidence:.1f}% | {pile}",
                'pile': pile
            }
            break  # Only consider the first valid contour

        if best_detection:
            if self.last_detected != best_detection['label']:
                self.detection_start_time = time.time()
                self.last_detected = best_detection['label']
            elif time.time() - self.detection_start_time > 0.5:
                self.get_logger().info(f"🧺 Sort to: {best_detection['pile']}")

            x, y, w, h = best_detection['x'], best_detection['y'], best_detection['w'], best_detection['h']
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(frame, best_detection['label'], (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
        else:
            self.detection_start_time = None
            self.last_detected = None
            cv2.putText(frame, "No clothing detected", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2)

        cv2.imshow('Laundry Color Detector', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.cleanup()

    def estimate_color(self, image):
        image = cv2.resize(image, (50, 50))
        pixels = image.reshape((-1, 3))
        kmeans = KMeans(n_clusters=1, n_init='auto')
        kmeans.fit(pixels)
        return kmeans.cluster_centers_[0]

    def classify_color(self, bgr):
        hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv

        if v > 200 and s < 40:
            return 'Light', 'Whites/Lights'
        elif v < 50:
            return 'Dark', 'Darks'
        else:
            return 'Color', 'Colors'

    def cleanup(self):
        self.cap.release()
        cv2.destroyAllWindows()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LaundryColorDetector()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
