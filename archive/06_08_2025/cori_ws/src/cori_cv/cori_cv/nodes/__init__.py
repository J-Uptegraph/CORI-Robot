def __init__(self):
    super().__init__('laundry_color_detector')
    self.get_logger().info('Initializing Laundry Color Detector...')

    self.cap = cv2.VideoCapture(0)
    if not self.cap.isOpened():
        self.get_logger().error('Failed to open webcam.')
        return

    # === CAMERA SETTINGS ===
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    self.cap.set(cv2.CAP_PROP_FPS, 30)

    # Some drivers use 0.75 for auto, others use 1.0 — fallback if needed
    if not self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0):
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)

    # Tune these for better color reproduction if auto doesn't behave well
    self.cap.set(cv2.CAP_PROP_EXPOSURE, 0.3)
    self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)
    self.cap.set(cv2.CAP_PROP_CONTRAST, 0.5)
    self.cap.set(cv2.CAP_PROP_SATURATION, 0.5)

    self.get_logger().info('Webcam initialized with adjusted visual settings.')

    self.timer = self.create_timer(0.15, self.process_frame)  # Run at ~6.7 Hz
    self.last_detected = None
    self.detection_start_time = None
def __init__(self):
    super().__init__('laundry_color_detector')
    self.get_logger().info('Initializing Laundry Color Detector...')

    self.cap = cv2.VideoCapture(0)
    if not self.cap.isOpened():
        self.get_logger().error('Failed to open webcam.')
        return

    # === CAMERA SETTINGS ===
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    self.cap.set(cv2.CAP_PROP_FPS, 30)

    # Some drivers use 0.75 for auto, others use 1.0 — fallback if needed
    if not self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1.0):
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)

    # Tune these for better color reproduction if auto doesn't behave well
    self.cap.set(cv2.CAP_PROP_EXPOSURE, 0.3)
    self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5)
    self.cap.set(cv2.CAP_PROP_CONTRAST, 0.5)
    self.cap.set(cv2.CAP_PROP_SATURATION, 0.5)

    self.get_logger().info('Webcam initialized with adjusted visual settings.')

    self.timer = self.create_timer(0.15, self.process_frame)  # Run at ~6.7 Hz
    self.last_detected = None
    self.detection_start_time = None
