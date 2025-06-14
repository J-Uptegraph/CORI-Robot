#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
import tkinter as tk
from tkinter import ttk, messagebox
import threading
from collections import deque
from datetime import datetime


class LaundryDetectorUI:
    def __init__(self, detector_node):
        self.detector = detector_node
        self.setup_main_window()
        self.setup_controls()
        self.setup_display_panel()
        self.setup_statistics()
        
        # State
        self.detection_active = False
        
    def setup_main_window(self):
        self.root = tk.Tk()
        self.root.title("Professional Laundry Detection System")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2b2b2b')
        self.root.resizable(True, True)
        
        # Create main layout
        self.create_layout()
        
    def create_layout(self):
        # Control panel (top)
        self.control_frame = tk.Frame(self.root, bg='#1e1e1e', height=80)
        self.control_frame.pack(fill='x', padx=5, pady=5)
        self.control_frame.pack_propagate(False)
        
        # Main content
        self.main_frame = tk.Frame(self.root, bg='#2b2b2b')
        self.main_frame.pack(fill='both', expand=True, padx=5, pady=(0, 5))
        
        # Video area (left)
        self.video_frame = tk.Frame(self.main_frame, bg='#3c3c3c', relief='ridge', bd=2)
        self.video_frame.pack(side='left', fill='both', expand=True, padx=(0, 5))
        
        # Control sidebar (right)
        self.sidebar_frame = tk.Frame(self.main_frame, bg='#1e1e1e', width=300, relief='ridge', bd=2)
        self.sidebar_frame.pack(side='right', fill='y')
        self.sidebar_frame.pack_propagate(False)
        
    def setup_controls(self):
        # Title
        title_label = tk.Label(self.control_frame, text="LAUNDRY DETECTION SYSTEM", 
                              font=('Arial', 16, 'bold'), fg='#00ff88', bg='#1e1e1e')
        title_label.pack(side='left', padx=10, pady=10)
        
        # Status
        self.status_frame = tk.Frame(self.control_frame, bg='#1e1e1e')
        self.status_frame.pack(side='left', padx=20)
        
        self.status_light = tk.Label(self.status_frame, text="●", font=('Arial', 20), 
                                   fg='#ff4444', bg='#1e1e1e')
        self.status_light.pack()
        
        self.status_text = tk.Label(self.status_frame, text="OFFLINE", 
                                  font=('Arial', 10, 'bold'), fg='#ffffff', bg='#1e1e1e')
        self.status_text.pack()
        
        # Buttons
        self.button_frame = tk.Frame(self.control_frame, bg='#1e1e1e')
        self.button_frame.pack(side='right', padx=10)
        
        self.start_btn = tk.Button(self.button_frame, text="START DETECTION", 
                                  font=('Arial', 12, 'bold'), fg='white', bg='#007acc',
                                  command=self.toggle_detection, width=15)
        self.start_btn.pack(side='left', padx=5, pady=5)
        
        self.detect_btn = tk.Button(self.button_frame, text="DETECT ALL", 
                                   font=('Arial', 12, 'bold'), fg='white', bg='#ff6b35',
                                   command=self.detect_all_objects, width=12)
        self.detect_btn.pack(side='left', padx=5, pady=5)
        
        self.reset_btn = tk.Button(self.button_frame, text="RESET", 
                                  font=('Arial', 12, 'bold'), fg='white', bg='#6c757d',
                                  command=self.reset_system, width=8)
        self.reset_btn.pack(side='left', padx=5, pady=5)
        
    def setup_display_panel(self):
        # Detection Settings
        settings_frame = tk.LabelFrame(self.sidebar_frame, text="DETECTION SETTINGS", 
                                     font=('Arial', 12, 'bold'), fg='#00ff88', bg='#1e1e1e')
        settings_frame.pack(fill='x', padx=10, pady=10)
        
        # Confidence threshold
        tk.Label(settings_frame, text="Confidence Threshold:", 
                fg='white', bg='#1e1e1e').pack(anchor='w', padx=5, pady=2)
        
        self.confidence_var = tk.DoubleVar(value=0.75)
        self.confidence_scale = tk.Scale(settings_frame, from_=0.1, to=1.0, resolution=0.05,
                                       orient='horizontal', variable=self.confidence_var,
                                       bg='#1e1e1e', fg='white')
        self.confidence_scale.pack(fill='x', padx=5, pady=2)
        
        # Min area
        tk.Label(settings_frame, text="Min Detection Area:", 
                fg='white', bg='#1e1e1e').pack(anchor='w', padx=5, pady=2)
        
        self.min_area_var = tk.IntVar(value=2000)
        self.min_area_scale = tk.Scale(settings_frame, from_=500, to=10000, resolution=100,
                                     orient='horizontal', variable=self.min_area_var,
                                     bg='#1e1e1e', fg='white')
        self.min_area_scale.pack(fill='x', padx=5, pady=2)
        
        # Detection mode
        tk.Label(settings_frame, text="Detection Mode:", 
                fg='white', bg='#1e1e1e').pack(anchor='w', padx=5, pady=5)
        
        self.mode_var = tk.StringVar(value="hybrid")
        mode_frame = tk.Frame(settings_frame, bg='#1e1e1e')
        mode_frame.pack(fill='x', padx=5)
        
        tk.Radiobutton(mode_frame, text="Hybrid", variable=self.mode_var, value="hybrid",
                      fg='white', bg='#1e1e1e', selectcolor='#007acc').pack(side='left')
        tk.Radiobutton(mode_frame, text="Motion", variable=self.mode_var, value="motion",
                      fg='white', bg='#1e1e1e', selectcolor='#007acc').pack(side='left')
        tk.Radiobutton(mode_frame, text="Static", variable=self.mode_var, value="static",
                      fg='white', bg='#1e1e1e', selectcolor='#007acc').pack(side='left')
        
        # Display options
        display_frame = tk.LabelFrame(self.sidebar_frame, text="DISPLAY OPTIONS", 
                                    font=('Arial', 12, 'bold'), fg='#00ff88', bg='#1e1e1e')
        display_frame.pack(fill='x', padx=10, pady=10)
        
        self.show_conf_var = tk.BooleanVar(value=True)
        tk.Checkbutton(display_frame, text="Show Confidence", variable=self.show_conf_var,
                      fg='white', bg='#1e1e1e', selectcolor='#007acc').pack(anchor='w', padx=5)
        
        self.show_area_var = tk.BooleanVar(value=False)
        tk.Checkbutton(display_frame, text="Show Areas", variable=self.show_area_var,
                      fg='white', bg='#1e1e1e', selectcolor='#007acc').pack(anchor='w', padx=5)
        
        self.show_debug_var = tk.BooleanVar(value=False)
        tk.Checkbutton(display_frame, text="Debug Mode", variable=self.show_debug_var,
                      fg='white', bg='#1e1e1e', selectcolor='#007acc').pack(anchor='w', padx=5)
        
    def setup_statistics(self):
        # Results
        results_frame = tk.LabelFrame(self.sidebar_frame, text="DETECTION RESULTS", 
                                    font=('Arial', 12, 'bold'), fg='#00ff88', bg='#1e1e1e')
        results_frame.pack(fill='x', padx=10, pady=10)
        
        self.results_labels = {}
        categories = [('Total Items:', 'total', '#ffffff'),
                     ('Lights:', 'lights', '#ffffff'),
                     ('Darks:', 'darks', '#888888'),
                     ('Colors:', 'colors', '#ff6b35')]
        
        for text, key, color in categories:
            frame = tk.Frame(results_frame, bg='#1e1e1e')
            frame.pack(fill='x', padx=5, pady=2)
            
            tk.Label(frame, text=text, fg='white', bg='#1e1e1e').pack(side='left')
            self.results_labels[key] = tk.Label(frame, text="0", fg=color, bg='#1e1e1e', 
                                              font=('Arial', 12, 'bold'))
            self.results_labels[key].pack(side='right')
        
        # Performance
        perf_frame = tk.LabelFrame(self.sidebar_frame, text="PERFORMANCE", 
                                 font=('Arial', 12, 'bold'), fg='#00ff88', bg='#1e1e1e')
        perf_frame.pack(fill='x', padx=10, pady=10)
        
        # FPS
        fps_frame = tk.Frame(perf_frame, bg='#1e1e1e')
        fps_frame.pack(fill='x', padx=5, pady=2)
        tk.Label(fps_frame, text="FPS:", fg='white', bg='#1e1e1e').pack(side='left')
        self.fps_label = tk.Label(fps_frame, text="0.0", fg='#00ff88', bg='#1e1e1e', 
                                 font=('Arial', 12, 'bold'))
        self.fps_label.pack(side='right')
        
        # Processing time
        proc_frame = tk.Frame(perf_frame, bg='#1e1e1e')
        proc_frame.pack(fill='x', padx=5, pady=2)
        tk.Label(proc_frame, text="Processing:", fg='white', bg='#1e1e1e').pack(side='left')
        self.proc_label = tk.Label(proc_frame, text="0ms", fg='#ffaa00', bg='#1e1e1e', 
                                  font=('Arial', 12, 'bold'))
        self.proc_label.pack(side='right')
        
        # Export button
        export_btn = tk.Button(self.sidebar_frame, text="EXPORT RESULTS", 
                              font=('Arial', 12, 'bold'), fg='white', bg='#28a745',
                              command=self.export_results)
        export_btn.pack(fill='x', padx=10, pady=10)
        
    def toggle_detection(self):
        self.detection_active = not self.detection_active
        
        if self.detection_active:
            self.start_btn.configure(text="STOP DETECTION", bg='#dc3545')
            self.status_light.configure(fg='#00ff88')
            self.status_text.configure(text="ACTIVE")
            self.detector.start_detection()
        else:
            self.start_btn.configure(text="START DETECTION", bg='#007acc')
            self.status_light.configure(fg='#ff4444')
            self.status_text.configure(text="STOPPED")
            self.detector.stop_detection()
    
    def detect_all_objects(self):
        self.detector.detect_all_in_frame()
        
    def reset_system(self):
        result = messagebox.askyesno("Reset", "Reset all detection data?")
        if result:
            self.detector.reset_all()
            self.update_results({'lights': 0, 'darks': 0, 'colors': 0})
    
    def export_results(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"laundry_results_{timestamp}.json"
        
        results = {
            'timestamp': timestamp,
            'total_items': sum(self.detector.class_counts.values()),
            'class_counts': self.detector.class_counts,
            'session_duration': time.time() - self.detector.session_start
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(results, f, indent=2)
            messagebox.showinfo("Export", f"Results saved to {filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Export failed: {str(e)}")
    
    def update_results(self, counts):
        total = sum(counts.values())
        self.results_labels['total'].configure(text=str(total))
        self.results_labels['lights'].configure(text=str(counts.get('lights', 0)))
        self.results_labels['darks'].configure(text=str(counts.get('darks', 0)))
        self.results_labels['colors'].configure(text=str(counts.get('colors', 0)))
    
    def update_performance(self, fps, processing_time):
        self.fps_label.configure(text=f"{fps:.1f}")
        self.proc_label.configure(text=f"{processing_time:.1f}ms")
    
    def get_settings(self):
        return {
            'confidence_threshold': self.confidence_var.get(),
            'min_area': self.min_area_var.get(),
            'detection_mode': self.mode_var.get(),
            'show_confidence': self.show_conf_var.get(),
            'show_areas': self.show_area_var.get(),
            'debug_mode': self.show_debug_var.get()
        }


class ProfessionalLaundryDetector(Node):
    def __init__(self):
        super().__init__('laundry_detector')
        self.get_logger().info('Professional Laundry Detector starting...')
        
        # Core components
        self.bridge = CvBridge()
        self.setup_detection()
        self.setup_ros_interfaces()
        
        # State
        self.latest_frame = None
        self.current_detections = []
        self.class_counts = {'lights': 0, 'darks': 0, 'colors': 0}
        self.session_start = time.time()
        self.detection_active = False
        self.processing_times = deque(maxlen=30)
        
        # Font for OpenCV text
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Start UI
        self.ui_thread = threading.Thread(target=self.setup_ui, daemon=True)
        self.ui_thread.start()
        
        self.get_logger().info('Laundry Detector ready')

    def setup_detection(self):
        # Background subtractor
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=25, detectShadows=True
        )

    def setup_ros_interfaces(self):
        # Subscribers
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        # Publishers
        self.pick_target_pub = self.create_publisher(PoseStamped, '/pick_target', 5)
        self.sort_command_pub = self.create_publisher(String, '/sort_command', 5)
        self.status_pub = self.create_publisher(String, '/detector_status', 5)
        
        # Timer
        self.timer = self.create_timer(0.033, self.process_frame)

    def setup_ui(self):
        self.ui = LaundryDetectorUI(self)
        self.ui.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.ui.root.mainloop()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = self.enhance_image(frame)
            self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"Image callback error: {e}")

    def enhance_image(self, frame):
        # Noise reduction
        frame = cv2.bilateralFilter(frame, 9, 75, 75)
        
        # Brightness/contrast
        alpha = 1.3  # Contrast
        beta = 40    # Brightness
        frame = cv2.convertScaleAbs(frame, alpha=alpha, beta=beta)
        
        # CLAHE
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        l = clahe.apply(l)
        frame = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
        
        return frame

    def process_frame(self):
        if self.latest_frame is None or not hasattr(self, 'ui'):
            return
        
        start_time = time.time()
        
        # Get settings
        settings = self.ui.get_settings()
        
        # Create display frame
        display_frame = self.latest_frame.copy()
        
        # Detection
        if self.detection_active:
            detections = self.detect_objects(self.latest_frame, settings)
            self.current_detections = detections
            self.update_class_counts(detections)
            self.draw_detections(display_frame, detections, settings)
        
        # Draw overlays
        self.draw_overlays(display_frame, settings)
        
        # Performance
        processing_time = (time.time() - start_time) * 1000
        self.processing_times.append(processing_time)
        
        fps = 1.0 / max(0.001, time.time() - start_time)
        
        # Update UI
        try:
            self.ui.update_results(self.class_counts)
            self.ui.update_performance(fps, processing_time)
        except:
            pass
        
        # Display
        cv2.imshow('Professional Laundry Detection System', display_frame)
        
        # Keys
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            self.on_closing()
        elif key == ord(' '):  # Space
            if hasattr(self, 'ui'):
                self.ui.toggle_detection()
        elif key == ord('d'):  # D
            self.detect_all_in_frame()
        elif key == ord('r'):  # R
            self.reset_all()

    def detect_objects(self, frame, settings):
        detections = []
        
        if settings['detection_mode'] in ['hybrid', 'motion']:
            detections.extend(self.detect_motion_objects(frame, settings))
        
        if settings['detection_mode'] in ['hybrid', 'static']:
            detections.extend(self.detect_static_objects(frame, settings))
        
        # Remove overlaps
        detections = self.remove_overlaps(detections)
        
        # Filter by confidence
        detections = [d for d in detections if d['confidence'] >= settings['confidence_threshold']]
        
        return detections

    def detect_motion_objects(self, frame, settings):
        # Background subtraction
        fg_mask = self.bg_subtractor.apply(frame)
        fg_mask[fg_mask == 127] = 0  # Remove shadows
        
        # Morphology
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for contour in contours:
            detection = self.analyze_contour(contour, frame, 'motion', settings)
            if detection:
                detections.append(detection)
        
        return detections

    def detect_static_objects(self, frame, settings):
        # Edge detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        
        # Color segmentation
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create fabric mask
        mask1 = cv2.inRange(hsv, (0, 0, 50), (180, 255, 255))
        mask2 = cv2.inRange(hsv, (0, 0, 200), (180, 30, 255))
        color_mask = cv2.bitwise_or(mask1, mask2)
        
        combined_mask = cv2.bitwise_or(edges, color_mask)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel)
        
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detections = []
        for contour in contours:
            detection = self.analyze_contour(contour, frame, 'static', settings)
            if detection:
                detections.append(detection)
        
        return detections

    def analyze_contour(self, contour, frame, detection_type, settings):
        area = cv2.contourArea(contour)
        if area < settings['min_area'] or area > 100000:
            return None
        
        x, y, w, h = cv2.boundingRect(contour)
        
        aspect_ratio = w / h
        if aspect_ratio > 5 or aspect_ratio < 0.2:
            return None
        
        roi = frame[y:y+h, x:x+w]
        if roi.size == 0:
            return None
        
        # Color analysis
        color_result = self.analyze_color(roi)
        
        # Shape analysis
        hull = cv2.convexHull(contour)
        hull_area = cv2.contourArea(hull)
        solidity = area / hull_area if hull_area > 0 else 0
        extent = area / (w * h)
        
        shape_score = (solidity * 0.5 + extent * 0.5)
        
        # Confidence
        type_bonus = 0.1 if detection_type == 'motion' else 0.0
        confidence = color_result['confidence'] * 0.6 + shape_score * 0.3 + type_bonus * 0.1
        
        return {
            'contour': contour,
            'bbox': (x, y, w, h),
            'area': area,
            'color_name': color_result['color_name'],
            'pile': color_result['pile'],
            'confidence': confidence,
            'detection_type': detection_type,
            'shape_score': shape_score,
            'color_confidence': color_result['confidence']
        }

    def analyze_color(self, roi):
        h, w = roi.shape[:2]
        
        # Sample center region
        center_size = min(w//3, h//3, 20)
        center_roi = roi[h//2-center_size:h//2+center_size, w//2-center_size:w//2+center_size]
        
        if center_roi.size > 0:
            dominant_color = np.median(center_roi.reshape(-1, 3), axis=0).astype(int)
        else:
            dominant_color = np.median(roi.reshape(-1, 3), axis=0).astype(int)
        
        return self.classify_color(dominant_color)

    def classify_color(self, bgr_color):
        # Convert to HSV
        hsv = cv2.cvtColor(np.uint8([[bgr_color]]), cv2.COLOR_BGR2HSV)[0][0]
        h, s, v = hsv
        
        brightness = v
        saturation = s
        
        # White detection
        if s < 20 and v > 200:
            return {'color_name': 'white', 'pile': 'lights', 'confidence': 0.95}
        
        # Black detection
        if v < 45:
            return {'color_name': 'black', 'pile': 'darks', 'confidence': 0.92}
        
        # Gray detection
        if s < 30:
            if v > 160:
                return {'color_name': 'light_gray', 'pile': 'lights', 'confidence': 0.88}
            else:
                return {'color_name': 'gray', 'pile': 'darks', 'confidence': 0.85}
        
        # Color classification
        hue_360 = h * 2
        
        if hue_360 < 15 or hue_360 >= 345:
            color_name, pile = 'red', 'colors'
        elif 15 <= hue_360 < 35:
            color_name, pile = 'orange', 'colors'
        elif 35 <= hue_360 < 55:
            if brightness > 150 and saturation < 150:
                color_name, pile = 'yellow', 'lights'
            else:
                color_name, pile = 'yellow', 'colors'
        elif 55 <= hue_360 < 85:
            color_name, pile = 'green', 'colors'
        elif 85 <= hue_360 < 115:
            color_name, pile = 'cyan', 'colors'
        elif 115 <= hue_360 < 145:
            if brightness < 70:
                color_name, pile = 'navy', 'darks'
            else:
                color_name, pile = 'blue', 'colors'
        elif 145 <= hue_360 < 175:
            if brightness < 80:
                color_name, pile = 'purple', 'darks'
            else:
                color_name, pile = 'purple', 'colors'
        else:
            color_name, pile = 'pink', 'colors'
        
        confidence = min(0.9, max(0.5, saturation / 255 * 0.7 + 0.3))
        
        return {'color_name': color_name, 'pile': pile, 'confidence': confidence}

    def remove_overlaps(self, detections):
        if len(detections) <= 1:
            return detections
        
        detections.sort(key=lambda x: x['confidence'], reverse=True)
        
        filtered = []
        for detection in detections:
            x1, y1, w1, h1 = detection['bbox']
            
            overlap = False
            for existing in filtered:
                x2, y2, w2, h2 = existing['bbox']
                
                intersection = max(0, min(x1 + w1, x2 + w2) - max(x1, x2)) * \
                             max(0, min(y1 + h1, y2 + h2) - max(y1, y2))
                union = w1 * h1 + w2 * h2 - intersection
                
                if union > 0:
                    iou = intersection / union
                    if iou > 0.3:
                        overlap = True
                        break
            
            if not overlap:
                filtered.append(detection)
        
        return filtered

    def update_class_counts(self, detections):
        self.class_counts = {'lights': 0, 'darks': 0, 'colors': 0}
        
        for detection in detections:
            pile = detection['pile']
            if pile in self.class_counts:
                self.class_counts[pile] += 1

    def draw_detections(self, frame, detections, settings):
        for i, detection in enumerate(detections):
            self.draw_detection(frame, detection, i == 0, settings)

    def draw_detection(self, frame, detection, is_primary, settings):
        contour = detection['contour']
        x, y, w, h = detection['bbox']
        confidence = detection['confidence']
        
        # Colors
        pile_colors = {
            'lights': (255, 255, 255),
            'darks': (100, 100, 100),
            'colors': (100, 200, 255)
        }
        base_color = pile_colors.get(detection['pile'], (128, 128, 128))
        color = tuple(int(c * confidence) for c in base_color)
        
        # Thickness
        thickness = max(2, int(confidence * 4))
        if is_primary:
            thickness += 2
        
        # Draw contour
        cv2.drawContours(frame, [contour], -1, color, thickness)
        
        # Draw bounding box
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        
        # Labels
        labels = []
        labels.append(f"{detection['color_name'].upper()} -> {detection['pile'].upper()}")
        
        if settings['show_confidence']:
            labels.append(f"Conf: {confidence:.2f}")
        
        if settings['show_areas']:
            labels.append(f"Area: {detection['area']:.0f}")
        
        if settings['debug_mode']:
            labels.append(f"Type: {detection['detection_type']}")
        
        # Draw label background
        max_width = max([cv2.getTextSize(l, self.font, 0.6, 1)[0][0] for l in labels])
        bg_height = len(labels) * 20 + 10
        
        bg_y = max(0, y - bg_height - 5)
        
        overlay = frame.copy()
        cv2.rectangle(overlay, (x, bg_y), (x + max_width + 10, y), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, frame)
        
        cv2.rectangle(frame, (x, bg_y), (x + max_width + 10, y), color, 2)
        
        # Draw text
        for i, label in enumerate(labels):
            text_color = (255, 255, 255) if i == 0 else color
            cv2.putText(frame, label, (x + 5, bg_y + 20 + i * 18), 
                       self.font, 0.5, text_color, 1)

    def draw_overlays(self, frame, settings):
        h, w = frame.shape[:2]
        
        # Status
        status_color = (0, 255, 0) if self.detection_active else (0, 0, 255)
        status_text = "DETECTION ACTIVE" if self.detection_active else "DETECTION STOPPED"
        
        cv2.putText(frame, status_text, (10, h - 20), self.font, 0.8, status_color, 2)
        
        # Instructions
        instructions = [
            "SPACE: Toggle Detection",
            "D: Detect All", 
            "R: Reset",
            "ESC: Exit"
        ]
        
        for i, instruction in enumerate(instructions):
            cv2.putText(frame, instruction, (10, 30 + i * 25), self.font, 0.5, (200, 200, 200), 1)

    def start_detection(self):
        self.detection_active = True
        self.get_logger().info("Detection started")

    def stop_detection(self):
        self.detection_active = False
        self.get_logger().info("Detection stopped")

    def detect_all_in_frame(self):
        if self.latest_frame is not None:
            settings = self.ui.get_settings() if hasattr(self, 'ui') else {
                'confidence_threshold': 0.5, 'min_area': 2000, 'detection_mode': 'hybrid',
                'show_confidence': True, 'show_areas': False, 'debug_mode': False
            }
            
            detections = self.detect_objects(self.latest_frame, settings)
            self.current_detections = detections
            self.update_class_counts(detections)
            
            self.get_logger().info(f"Detected {len(detections)} objects")

    def reset_all(self):
        self.current_detections = []
        self.class_counts = {'lights': 0, 'darks': 0, 'colors': 0}
        self.session_start = time.time()
        
        self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
            history=500, varThreshold=25, detectShadows=True
        )
        
        self.get_logger().info("System reset")

    def on_closing(self):
        self.get_logger().info("Shutting down detector")
        cv2.destroyAllWindows()
        
        if hasattr(self, 'ui'):
            self.ui.root.quit()
        
        rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    detector = ProfessionalLaundryDetector()
    
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        detector.get_logger().info("Keyboard interrupt")
    except Exception as e:
        detector.get_logger().error(f"Error: {e}")
    finally:
        detector.destroy_node()


if __name__ == '__main__':
    main()