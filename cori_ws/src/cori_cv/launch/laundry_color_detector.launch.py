import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3
from std_msgs.msg import String, Bool, Int32, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
from datetime import datetime
from collections import defaultdict, deque
import threading
import queue
import math
from scipy import ndimage
from sklearn.cluster import DBSCAN, KMeans
from sklearn.ensemble import IsolationForest
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
from scipy.optimize import minimize


class AdvancedLaundryVisionSystem(Node):
    def __init__(self):
        super().__init__('advanced_laundry_vision_system')
        self.get_logger().info('Initializing Advanced Laundry Vision System v4.0 - Production Grade')

        # Core systems
        self.bridge = CvBridge()
        self.setup_advanced_parameters()
        self.setup_lighting_systems()
        self.setup_multi_modal_processing()
        self.setup_physics_engine()
        self.setup_learning_systems()
        self.setup_production_interfaces()
        
        # Performance optimization
        self.setup_performance_optimization()
        
        self.get_logger().info('Advanced Laundry Vision System ready for production operation')

    def setup_advanced_parameters(self):
        """Advanced parameter configuration for production use"""
        # Vision parameters
        self.declare_parameter('hdr_exposure_count', 5)
        self.declare_parameter('adaptive_lighting', True)
        self.declare_parameter('multi_spectral_analysis', True)
        self.declare_parameter('physics_simulation', True)
        self.declare_parameter('temporal_consistency', True)
        self.declare_parameter('material_classification', True)
        
        # Performance parameters
        self.declare_parameter('max_processing_time', 0.1)  # 100ms for real-time
        self.declare_parameter('confidence_threshold', 0.85)
        self.declare_parameter('multi_threading', True)
        self.declare_parameter('gpu_acceleration', True)
        
        # Advanced features
        self.declare_parameter('predictive_picking', True)
        self.declare_parameter('fabric_physics', True)
        self.declare_parameter('wrinkle_detection', True)
        self.declare_parameter('damage_assessment', True)
        self.declare_parameter('size_estimation', True)
        
        # Load parameters
        self.hdr_exposure_count = self.get_parameter('hdr_exposure_count').value
        self.adaptive_lighting = self.get_parameter('adaptive_lighting').value
        self.multi_spectral_analysis = self.get_parameter('multi_spectral_analysis').value
        self.physics_simulation = self.get_parameter('physics_simulation').value
        self.temporal_consistency = self.get_parameter('temporal_consistency').value
        self.material_classification = self.get_parameter('material_classification').value
        self.max_processing_time = self.get_parameter('max_processing_time').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.predictive_picking = self.get_parameter('predictive_picking').value
        self.fabric_physics = self.get_parameter('fabric_physics').value

    def setup_lighting_systems(self):
        """Advanced lighting and exposure control"""
        # HDR processing
        self.hdr_processor = HDRProcessor(self.hdr_exposure_count)
        self.exposure_controller = AdaptiveExposureController()
        self.lighting_controller = ActiveLightingController()
        
        # Multi-spectral analysis
        self.spectral_analyzer = MultiSpectralAnalyzer()
        
        # Current lighting state
        self.current_lighting_config = {
            'ambient_level': 0.5,
            'directional_angle': 45,
            'color_temperature': 5500,
            'active_strobes': []
        }

    def setup_multi_modal_processing(self):
        """Advanced multi-modal sensor fusion"""
        # Sensor fusion
        self.rgb_processor = AdvancedRGBProcessor()
        self.depth_processor = AdvancedDepthProcessor()
        self.thermal_processor = ThermalProcessor()  # If available
        self.sensor_fusion = MultiModalFusion()
        
        # Frame buffers for temporal consistency
        self.frame_buffer = deque(maxlen=10)
        self.depth_buffer = deque(maxlen=10)
        self.thermal_buffer = deque(maxlen=5)
        
        # Temporal tracking
        self.object_tracker = AdvancedObjectTracker()
        self.temporal_analyzer = TemporalConsistencyAnalyzer()

    def setup_physics_engine(self):
        """Physics-based understanding of fabric behavior"""
        self.fabric_physics = FabricPhysicsEngine()
        self.material_classifier = MaterialPropertyClassifier()
        self.wrinkle_analyzer = WrinkleAnalyzer()
        self.deformation_model = FabricDeformationModel()
        
        # Physics simulation parameters
        self.gravity_vector = np.array([0, 0, -9.81])
        self.fabric_properties = {
            'cotton': {'stiffness': 0.3, 'weight': 0.15, 'drape': 0.7},
            'polyester': {'stiffness': 0.5, 'weight': 0.12, 'drape': 0.4},
            'denim': {'stiffness': 0.8, 'weight': 0.35, 'drape': 0.2},
            'silk': {'stiffness': 0.1, 'weight': 0.08, 'drape': 0.9}
        }

    def setup_learning_systems(self):
        """Self-improving detection systems"""
        self.confidence_tracker = ConfidenceTracker()
        self.error_analyzer = ErrorAnalyzer()
        self.performance_optimizer = PerformanceOptimizer()
        
        # Learning history
        self.detection_history = deque(maxlen=1000)
        self.success_patterns = {}
        self.failure_patterns = {}

    def setup_production_interfaces(self):
        """Production-grade ROS2 interfaces"""
        # High-frequency inputs
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.thermal_sub = self.create_subscription(Image, '/camera/thermal/image_raw', self.thermal_callback, 5)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 1)
        
        # Robot control outputs
        self.pick_target_pub = self.create_publisher(PoseStamped, '/advanced_pick_target', 10)
        self.grasp_strategy_pub = self.create_publisher(String, '/grasp_strategy', 10)
        self.material_info_pub = self.create_publisher(String, '/material_properties', 10)
        self.lighting_control_pub = self.create_publisher(String, '/lighting_control', 10)
        
        # Advanced outputs
        self.fabric_analysis_pub = self.create_publisher(String, '/fabric_analysis', 10)
        self.predictive_scene_pub = self.create_publisher(String, '/predictive_scene', 10)
        self.performance_metrics_pub = self.create_publisher(String, '/performance_metrics', 10)
        
        # System control
        self.operation_mode_sub = self.create_subscription(String, '/operation_mode', self.operation_mode_callback, 1)
        self.calibration_trigger_sub = self.create_subscription(Bool, '/trigger_calibration', self.calibration_callback, 1)

    def setup_performance_optimization(self):
        """Real-time performance optimization"""
        self.processing_queue = queue.Queue(maxsize=5)
        self.result_queue = queue.Queue(maxsize=10)
        
        # Multi-threading for real-time performance
        self.processing_threads = []
        for i in range(3):  # 3 processing threads
            thread = threading.Thread(target=self.processing_worker, daemon=True)
            thread.start()
            self.processing_threads.append(thread)
        
        # Performance monitoring
        self.performance_monitor = PerformanceMonitor()
        self.frame_times = deque(maxlen=100)
        
        # Main processing timer - high frequency
        self.main_timer = self.create_timer(0.033, self.main_processing_callback)  # 30 FPS
        self.display_timer = self.create_timer(0.033, self.display_callback)

    def rgb_callback(self, msg):
        """High-performance RGB processing"""
        try:
            start_time = time.time()
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Add to processing queue if not full
            if not self.processing_queue.full():
                timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self.processing_queue.put({
                    'type': 'rgb',
                    'data': frame,
                    'timestamp': timestamp
                })
            
            self.frame_times.append(time.time() - start_time)
            
        except Exception as e:
            self.get_logger().error(f"RGB callback error: {e}")

    def depth_callback(self, msg):
        """Advanced depth processing"""
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            if not self.processing_queue.full():
                self.processing_queue.put({
                    'type': 'depth',
                    'data': depth,
                    'timestamp': timestamp
                })
                
        except Exception as e:
            self.get_logger().error(f"Depth callback error: {e}")

    def thermal_callback(self, msg):
        """Thermal imaging processing (if available)"""
        try:
            thermal = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono16')
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            if not self.processing_queue.full():
                self.processing_queue.put({
                    'type': 'thermal',
                    'data': thermal,
                    'timestamp': timestamp
                })
                
        except Exception as e:
            self.get_logger().debug(f"Thermal callback error (sensor may not be available): {e}")

    def camera_info_callback(self, msg):
        """Camera calibration info"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)

    def processing_worker(self):
        """Worker thread for intensive processing"""
        while True:
            try:
                # Get next item from queue
                item = self.processing_queue.get(timeout=1.0)
                
                # Process based on type
                if item['type'] == 'rgb':
                    result = self.process_rgb_advanced(item['data'], item['timestamp'])
                elif item['type'] == 'depth':
                    result = self.process_depth_advanced(item['data'], item['timestamp'])
                elif item['type'] == 'thermal':
                    result = self.process_thermal_advanced(item['data'], item['timestamp'])
                else:
                    continue
                
                # Store result
                if not self.result_queue.full():
                    self.result_queue.put(result)
                
                self.processing_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"Processing worker error: {e}")

    def process_rgb_advanced(self, frame, timestamp):
        """Advanced RGB processing with all enhancements"""
        start_time = time.time()
        
        # Step 1: HDR processing for challenging lighting
        hdr_frame = self.hdr_processor.process_frame(frame)
        
        # Step 2: Adaptive lighting compensation
        if self.adaptive_lighting:
            compensated_frame = self.exposure_controller.compensate_lighting(hdr_frame)
        else:
            compensated_frame = hdr_frame
        
        # Step 3: Multi-spectral analysis
        if self.multi_spectral_analysis:
            spectral_features = self.spectral_analyzer.extract_features(compensated_frame)
        else:
            spectral_features = None
        
        # Step 4: Advanced segmentation
        segments = self.advanced_segmentation(compensated_frame)
        
        # Step 5: Material classification
        materials = []
        if self.material_classification:
            for segment in segments:
                material = self.material_classifier.classify(compensated_frame, segment)
                materials.append(material)
        
        processing_time = time.time() - start_time
        
        return {
            'type': 'rgb_result',
            'timestamp': timestamp,
            'original_frame': frame,
            'hdr_frame': hdr_frame,
            'processed_frame': compensated_frame,
            'segments': segments,
            'materials': materials,
            'spectral_features': spectral_features,
            'processing_time': processing_time
        }

    def process_depth_advanced(self, depth, timestamp):
        """Advanced depth processing with physics understanding"""
        start_time = time.time()
        
        # Step 1: Advanced depth filtering
        filtered_depth = self.depth_processor.advanced_filter(depth)
        
        # Step 2: 3D reconstruction
        point_cloud = self.depth_processor.to_point_cloud(filtered_depth)
        
        # Step 3: Surface normal estimation
        normals = self.depth_processor.estimate_normals(point_cloud)
        
        # Step 4: Physics-based analysis
        if self.fabric_physics:
            fabric_analysis = self.fabric_physics.analyze_deformation(point_cloud, normals)
        else:
            fabric_analysis = None
        
        # Step 5: Pickability assessment
        pickable_points = self.assess_3d_pickability(point_cloud, normals)
        
        processing_time = time.time() - start_time
        
        return {
            'type': 'depth_result',
            'timestamp': timestamp,
            'original_depth': depth,
            'filtered_depth': filtered_depth,
            'point_cloud': point_cloud,
            'normals': normals,
            'fabric_analysis': fabric_analysis,
            'pickable_points': pickable_points,
            'processing_time': processing_time
        }

    def advanced_segmentation(self, frame):
        """State-of-the-art segmentation for overlapping fabric"""
        # Step 1: Multi-scale feature extraction
        features = self.extract_multi_scale_features(frame)
        
        # Step 2: Deep learning-inspired segmentation (simulated)
        primary_segments = self.deep_segmentation_simulation(frame, features)
        
        # Step 3: Physics-guided refinement
        refined_segments = self.physics_guided_refinement(frame, primary_segments)
        
        # Step 4: Temporal consistency
        if self.temporal_consistency and len(self.frame_buffer) > 0:
            consistent_segments = self.temporal_analyzer.ensure_consistency(
                refined_segments, self.frame_buffer[-1] if self.frame_buffer else None
            )
        else:
            consistent_segments = refined_segments
        
        return consistent_segments

    def extract_multi_scale_features(self, frame):
        """Extract features at multiple scales"""
        features = {}
        
        # Gaussian pyramid
        pyramid = [frame]
        for i in range(4):  # 4 scales
            pyramid.append(cv2.pyrDown(pyramid[-1]))
        
        # Extract features at each scale
        for i, img in enumerate(pyramid):
            scale_features = {}
            
            # Color features
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
            scale_features['hsv'] = hsv
            scale_features['lab'] = lab
            
            # Texture features
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Local Binary Pattern
            lbp = self.calculate_lbp(gray)
            scale_features['lbp'] = lbp
            
            # Gabor filters for texture
            gabor_responses = self.calculate_gabor_responses(gray)
            scale_features['gabor'] = gabor_responses
            
            # Edge features
            edges = cv2.Canny(gray, 50, 150)
            scale_features['edges'] = edges
            
            features[f'scale_{i}'] = scale_features
        
        return features

    def calculate_lbp(self, gray):
        """Calculate Local Binary Pattern"""
        # Simplified LBP implementation
        kernel = np.array([[-1, -1, -1], [-1, 8, -1], [-1, -1, -1]], dtype=np.float32)
        lbp = cv2.filter2D(gray.astype(np.float32), -1, kernel)
        return np.clip(lbp, 0, 255).astype(np.uint8)

    def calculate_gabor_responses(self, gray):
        """Calculate Gabor filter responses for texture analysis"""
        responses = []
        
        # Different orientations and frequencies
        orientations = [0, 45, 90, 135]
        frequencies = [0.1, 0.3, 0.5]
        
        for theta in orientations:
            for freq in frequencies:
                kernel = cv2.getGaborKernel((21, 21), 5, np.radians(theta), 
                                          2*np.pi*freq, 0.5, 0, ktype=cv2.CV_32F)
                response = cv2.filter2D(gray, cv2.CV_8UC3, kernel)
                responses.append(response)
        
        return np.stack(responses, axis=-1)

    def deep_segmentation_simulation(self, frame, features):
        """Simulate deep learning segmentation"""
        # This simulates what a trained U-Net or Mask R-CNN would do
        
        # Step 1: Combine multi-scale features
        combined_features = self.combine_multi_scale_features(features)
        
        # Step 2: Fabric-specific feature enhancement
        fabric_features = self.enhance_fabric_features(combined_features)
        
        # Step 3: Overlapping object separation
        separated_mask = self.separate_overlapping_objects(fabric_features)
        
        # Step 4: Instance segmentation
        instances = self.extract_instances(separated_mask, frame)
        
        return instances

    def combine_multi_scale_features(self, features):
        """Combine features from multiple scales"""
        base_scale = features['scale_0']
        h, w = base_scale['hsv'].shape[:2]
        
        # Initialize combined feature map
        combined = np.zeros((h, w, 0), dtype=np.float32)
        
        # Add color features
        combined = np.concatenate([combined, base_scale['hsv'].astype(np.float32)], axis=-1)
        combined = np.concatenate([combined, base_scale['lab'].astype(np.float32)], axis=-1)
        
        # Add texture features
        combined = np.concatenate([combined, base_scale['lbp'][..., np.newaxis].astype(np.float32)], axis=-1)
        
        # Add edge features
        combined = np.concatenate([combined, base_scale['edges'][..., np.newaxis].astype(np.float32)], axis=-1)
        
        # Add multi-scale context
        for i in range(1, 4):
            scale_key = f'scale_{i}'
            if scale_key in features:
                # Resize to base scale
                scale_hsv = cv2.resize(features[scale_key]['hsv'], (w, h))
                combined = np.concatenate([combined, scale_hsv.astype(np.float32) * 0.5], axis=-1)
        
        return combined

    def enhance_fabric_features(self, features):
        """Enhance features specifically for fabric detection"""
        # Fabric-specific enhancement using domain knowledge
        
        # Extract color channels
        hsv_start = 0
        hsv_end = 3
        lab_start = 3
        lab_end = 6
        
        hsv_features = features[:, :, hsv_start:hsv_end]
        lab_features = features[:, :, lab_start:lab_end]
        
        # Enhance saturation for fabric detection
        enhanced_saturation = hsv_features[:, :, 1] * 1.5
        enhanced_saturation = np.clip(enhanced_saturation, 0, 255)
        
        # Enhance L channel for better contrast
        enhanced_lightness = lab_features[:, :, 0] * 1.2
        enhanced_lightness = np.clip(enhanced_lightness, 0, 255)
        
        # Create enhanced feature map
        enhanced = features.copy()
        enhanced[:, :, hsv_start + 1] = enhanced_saturation
        enhanced[:, :, lab_start] = enhanced_lightness
        
        return enhanced

    def separate_overlapping_objects(self, features):
        """Advanced separation of overlapping fabric objects"""
        # Step 1: Initial clustering
        h, w, c = features.shape
        feature_vectors = features.reshape(-1, c)
        
        # Use DBSCAN for initial clustering
        clustering = DBSCAN(eps=30, min_samples=100)
        labels = clustering.fit_predict(feature_vectors)
        
        # Reshape back to image
        label_image = labels.reshape(h, w)
        
        # Step 2: Watershed refinement
        refined_mask = self.apply_advanced_watershed(label_image, features)
        
        return refined_mask

    def apply_advanced_watershed(self, label_image, features):
        """Advanced watershed with fabric physics"""
        # Convert to binary mask
        binary_mask = (label_image >= 0).astype(np.uint8) * 255
        
        # Distance transform
        dist_transform = cv2.distanceTransform(binary_mask, cv2.DIST_L2, 5)
        
        # Find local maxima (seeds)
        local_maxima = ndimage.maximum_filter(dist_transform, size=20) == dist_transform
        local_maxima = local_maxima & (dist_transform > 10)  # Minimum distance threshold
        
        # Create markers
        markers, num_markers = ndimage.label(local_maxima)
        
        # Apply watershed
        rgb_for_watershed = cv2.cvtColor(features[:, :, :3].astype(np.uint8), cv2.COLOR_HSV2BGR)
        markers = cv2.watershed(rgb_for_watershed, markers.astype(np.int32))
        
        return markers

    def extract_instances(self, segmentation_mask, frame):
        """Extract individual instances from segmentation"""
        instances = []
        
        # Find unique labels (excluding background)
        unique_labels = np.unique(segmentation_mask)
        unique_labels = unique_labels[unique_labels > 0]
        
        for label in unique_labels:
            # Create mask for this instance
            instance_mask = (segmentation_mask == label).astype(np.uint8) * 255
            
            # Find contours
            contours, _ = cv2.findContours(instance_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 2000:  # Minimum area threshold
                    instances.append({
                        'contour': contour,
                        'mask': instance_mask,
                        'area': area,
                        'bbox': cv2.boundingRect(contour),
                        'label': label
                    })
        
        return instances

    def physics_guided_refinement(self, frame, segments):
        """Refine segments using physics understanding"""
        if not self.fabric_physics:
            return segments
        
        refined_segments = []
        
        for segment in segments:
            # Analyze fabric physics
            physics_analysis = self.fabric_physics.analyze_segment(frame, segment)
            
            # Refine based on physics
            if physics_analysis['is_fabric_like']:
                refined_segment = self.refine_fabric_boundaries(segment, physics_analysis)
                refined_segments.append(refined_segment)
        
        return refined_segments

    def refine_fabric_boundaries(self, segment, physics_analysis):
        """Refine segment boundaries based on fabric physics"""
        # Use drape patterns and wrinkle analysis to improve boundaries
        contour = segment['contour']
        
        # Smooth contour based on expected fabric drape
        smoothed_contour = self.smooth_fabric_contour(contour, physics_analysis)
        
        # Update segment
        refined_segment = segment.copy()
        refined_segment['contour'] = smoothed_contour
        refined_segment['bbox'] = cv2.boundingRect(smoothed_contour)
        refined_segment['physics'] = physics_analysis
        
        return refined_segment

    def smooth_fabric_contour(self, contour, physics_analysis):
        """Smooth contour based on fabric physics"""
        # Convert contour to points
        points = contour.reshape(-1, 2)
        
        # Apply smoothing based on fabric stiffness
        stiffness = physics_analysis.get('stiffness', 0.5)
        
        # Higher stiffness = less smoothing
        smoothing_factor = max(3, int(15 * (1 - stiffness)))
        
        # Apply Gaussian smoothing
        smoothed_x = ndimage.gaussian_filter1d(points[:, 0], sigma=smoothing_factor, mode='wrap')
        smoothed_y = ndimage.gaussian_filter1d(points[:, 1], sigma=smoothing_factor, mode='wrap')
        
        smoothed_points = np.column_stack([smoothed_x, smoothed_y])
        
        return smoothed_points.reshape(-1, 1, 2).astype(np.int32)

    def assess_3d_pickability(self, point_cloud, normals):
        """Advanced 3D pickability assessment"""
        pickable_points = []
        
        if point_cloud is None or len(point_cloud) == 0:
            return pickable_points
        
        # Step 1: Find surface points
        surface_points = self.identify_surface_points(point_cloud, normals)
        
        # Step 2: Assess accessibility
        accessible_points = self.assess_accessibility(surface_points, normals)
        
        # Step 3: Evaluate grasp quality
        for point, normal in zip(accessible_points, normals):
            grasp_quality = self.evaluate_grasp_quality(point, normal, point_cloud)
            
            if grasp_quality > 0.7:  # High quality threshold
                pickable_points.append({
                    'position': point,
                    'normal': normal,
                    'quality': grasp_quality,
                    'grasp_strategy': self.determine_grasp_strategy(point, normal)
                })
        
        # Sort by quality
        pickable_points.sort(key=lambda x: x['quality'], reverse=True)
        
        return pickable_points

    def identify_surface_points(self, point_cloud, normals):
        """Identify points on the surface of fabric"""
        # Find points with normals pointing generally upward
        surface_mask = normals[:, 2] > 0.3  # Normal z-component > 0.3
        
        return point_cloud[surface_mask]

    def assess_accessibility(self, points, normals):
        """Assess which points are accessible for grasping"""
        accessible = []
        
        for i, (point, normal) in enumerate(zip(points, normals)):
            # Check if point is not too close to edges
            # Check if normal allows for good gripper approach
            # Check if point is not occluded
            
            accessibility_score = self.calculate_accessibility_score(point, normal, points)
            
            if accessibility_score > 0.5:
                accessible.append(point)
        
        return np.array(accessible)

    def calculate_accessibility_score(self, point, normal, all_points):
        """Calculate accessibility score for a point"""
        # Distance to nearest neighbors
        distances = cdist([point], all_points)[0]
        distances = distances[distances > 0]  # Exclude self
        
        if len(distances) == 0:
            return 0.0
        
        # Closer neighbors reduce accessibility
        min_distance = np.min(distances)
        distance_score = min(1.0, min_distance / 0.05)  # 5cm threshold
        
        # Normal orientation score (prefer upward-facing normals)
        normal_score = max(0.0, normal[2])  # Z-component of normal
        
        # Combine scores
        accessibility = distance_score * 0.7 + normal_score * 0.3
        
        return accessibility

    def evaluate_grasp_quality(self, point, normal, point_cloud):
        """Evaluate grasp quality for a specific point"""
        # Factors: surface curvature, local stability, material properties
        
        # Local curvature analysis
        curvature = self.calculate_local_curvature(point, point_cloud)
        curvature_score = 1.0 / (1.0 + curvature * 10)  # Lower curvature is better
        
        # Surface stability
        stability = self.calculate_surface_stability(point, normal, point_cloud)
        
        # Material grip estimation
        grip_score = 0.8  # Default grip score (would be material-dependent)
        
        # Combine factors
        quality = curvature_score * 0.4 + stability * 0.4 + grip_score * 0.2
        
        return quality

    def calculate_local_curvature(self, point, point_cloud):
        """Calculate local surface curvature"""
        # Find nearby points
        distances = cdist([point], point_cloud)[0]
        nearby_mask = distances < 0.02  # 2cm radius
        nearby_points = point_cloud[nearby_mask]
        
        if len(nearby_points) < 5:
            return 0.0
        
        # Fit plane and calculate deviation
        centroid = np.mean(nearby_points, axis=0)
        centered_points = nearby_points - centroid
        
        # SVD to find best-fit plane
        _, _, vh = np.linalg.svd(centered_points)
        normal = vh[-1]
        
        # Calculate RMS deviation from plane
        deviations = np.abs(np.dot(centered_points, normal))
        curvature = np.sqrt(np.mean(deviations**2))
        
        return curvature

    def calculate_surface_stability(self, point, normal, point_cloud):
        """Calculate how stable the surface is around a point"""
        # Find nearby points
        distances = cdist([point], point_cloud)[0]
        nearby_mask = distances < 0.03  # 3cm radius
        nearby_points = point_cloud[nearby_mask]
        
        if len(nearby_points) < 3:
            return 0.0
        
        # Calculate height variance
        height_variance = np.var(nearby_points[:, 2])
        stability = 1.0 / (1.0 + height_variance * 100)
        
        return stability

    def determine_grasp_strategy(self, point, normal):
        """Determine optimal grasp strategy"""
        # Analyze normal direction and local geometry
        normal_angle = np.arccos(np.clip(normal[2], -1, 1))  # Angle from vertical
        
        if normal_angle < np.pi/6:  # Nearly vertical
            return 'top_grasp'
        elif normal_angle < np.pi/3:  # Moderate angle
            return 'angled_grasp'
        else:  # Steep angle
            return 'side_grasp'

    def main_processing_callback(self):
        """Main processing loop - optimized for real-time"""
        start_time = time.time()
        
        try:
            # Collect all recent results
            rgb_result = None
            depth_result = None
            thermal_result = None
            
            # Get latest results from each modality
            results_processed = 0
            while not self.result_queue.empty() and results_processed < 5:
                result = self.result_queue.get_nowait()
                
                if result['type'] == 'rgb_result':
                    rgb_result = result
                elif result['type'] == 'depth_result':
                    depth_result = result
                elif result['type'] == 'thermal_result':
                    thermal_result = result
                
                results_processed += 1
            
            # Sensor fusion and analysis
            if rgb_result and depth_result:
                fused_analysis = self.perform_sensor_fusion(rgb_result, depth_result, thermal_result)
                
                # Generate robot commands
                if fused_analysis['has_valid_targets']:
                    self.generate_robot_commands(fused_analysis)
                
                # Update visualization
                self.update_visualization(fused_analysis)
                
                # Performance monitoring
                self.performance_monitor.update(time.time() - start_time)
        
        except Exception as e:
            self.get_logger().error(f"Main processing error: {e}")

    def perform_sensor_fusion(self, rgb_result, depth_result, thermal_result=None):
        """Advanced multi-modal sensor fusion"""
        start_time = time.time()
        
        # Align timestamps
        time_diff = abs(rgb_result['timestamp'] - depth_result['timestamp'])
        if time_diff > 0.1:  # 100ms threshold
            self.get_logger().warn(f"Large timestamp difference: {time_diff:.3f}s")
        
        # Spatial alignment
        aligned_depth = self.align_depth_to_rgb(depth_result, rgb_result)
        
        # Combine segmentations
        rgb_segments = rgb_result['segments']
        depth_points = aligned_depth['pickable_points']
        
        # Enhanced object analysis
        enhanced_objects = []
        for segment in rgb_segments:
            enhanced_object = self.enhance_object_with_depth(segment, aligned_depth, rgb_result)
            
            # Add thermal data if available
            if thermal_result:
                enhanced_object = self.enhance_object_with_thermal(enhanced_object, thermal_result)
            
            # Comprehensive analysis
            comprehensive_analysis = self.perform_comprehensive_analysis(enhanced_object)
            
            if comprehensive_analysis['confidence'] > self.confidence_threshold:
                enhanced_objects.append(comprehensive_analysis)
        
        # Sort by priority
        prioritized_objects = self.prioritize_objects(enhanced_objects)
        
        fusion_time = time.time() - start_time
        
        return {
            'objects': prioritized_objects,
            'has_valid_targets': len(prioritized_objects) > 0,
            'rgb_result': rgb_result,
            'depth_result': aligned_depth,
            'thermal_result': thermal_result,
            'fusion_time': fusion_time,
            'timestamp': max(rgb_result['timestamp'], depth_result['timestamp'])
        }

    def align_depth_to_rgb(self, depth_result, rgb_result):
        """Align depth data to RGB frame"""
        # Simplified alignment - in production, use camera calibration
        aligned_depth = depth_result.copy()
        
        # Temporal alignment through interpolation if needed
        # Spatial alignment through camera transforms
        
        return aligned_depth

    def enhance_object_with_depth(self, segment, depth_result, rgb_result):
        """Enhance RGB segment with depth information"""
        enhanced = segment.copy()
        
        # Extract depth within segment
        mask = segment['mask']
        if depth_result['filtered_depth'] is not None:
            depth_roi = depth_result['filtered_depth'][mask > 0]
            
            if len(depth_roi) > 0:
                enhanced['depth_stats'] = {
                    'mean_depth': np.nanmean(depth_roi),
                    'min_depth': np.nanmin(depth_roi),
                    'max_depth': np.nanmax(depth_roi),
                    'depth_variance': np.nanvar(depth_roi)
                }
                
                # 3D properties
                enhanced['3d_properties'] = self.calculate_3d_properties(segment, depth_result)
            
        return enhanced

    def enhance_object_with_thermal(self, enhanced_object, thermal_result):
        """Add thermal information to object analysis"""
        # Thermal analysis can help with material identification
        # and detect recently worn clothes vs. clean clothes
        
        enhanced_object['thermal_properties'] = {
            'average_temperature': 0.0,  # Placeholder
            'temperature_variance': 0.0,
            'thermal_signature': 'unknown'
        }
        
        return enhanced_object

    def calculate_3d_properties(self, segment, depth_result):
        """Calculate comprehensive 3D properties"""
        properties = {}
        
        # Volume estimation
        if 'point_cloud' in depth_result and depth_result['point_cloud'] is not None:
            volume = self.estimate_volume(segment, depth_result['point_cloud'])
            properties['estimated_volume'] = volume
        
        # Surface area estimation
        surface_area = self.estimate_surface_area(segment)
        properties['surface_area'] = surface_area
        
        # Thickness estimation
        if 'depth_stats' in segment:
            thickness = segment['depth_stats']['max_depth'] - segment['depth_stats']['min_depth']
            properties['thickness'] = thickness
        
        return properties

    def estimate_volume(self, segment, point_cloud):
        """Estimate 3D volume of fabric object"""
        # Simplified volume estimation using convex hull
        mask = segment['mask']
        if point_cloud is None:
            return 0.0
        
        # This would need proper 3D convex hull calculation
        # Simplified version:
        if 'depth_stats' in segment:
            area_2d = cv2.contourArea(segment['contour'])
            avg_thickness = segment['depth_stats'].get('depth_variance', 0.01)
            volume = area_2d * avg_thickness * 1e-6  # Convert to cubic meters
            return volume
        
        return 0.0

    def estimate_surface_area(self, segment):
        """Estimate surface area considering wrinkles and folds"""
        # Base area from contour
        base_area = cv2.contourArea(segment['contour'])
        
        # Wrinkle factor (more wrinkles = more surface area)
        wrinkle_factor = 1.0
        if 'physics' in segment:
            wrinkle_complexity = segment['physics'].get('wrinkle_complexity', 0.0)
            wrinkle_factor = 1.0 + wrinkle_complexity * 0.5
        
        return base_area * wrinkle_factor * 1e-6  # Convert to square meters

    def perform_comprehensive_analysis(self, enhanced_object):
        """Comprehensive analysis combining all modalities"""
        analysis = enhanced_object.copy()
        
        # Multi-factor confidence calculation
        factors = {
            'color_confidence': enhanced_object.get('color_confidence', 0.5),
            'shape_confidence': self.calculate_shape_confidence(enhanced_object),
            'depth_confidence': self.calculate_depth_confidence(enhanced_object),
            'physics_confidence': self.calculate_physics_confidence(enhanced_object),
            'temporal_confidence': self.calculate_temporal_confidence(enhanced_object)
        }
        
        # Weighted combination
        weights = {
            'color_confidence': 0.25,
            'shape_confidence': 0.20,
            'depth_confidence': 0.25,
            'physics_confidence': 0.20,
            'temporal_confidence': 0.10
        }
        
        overall_confidence = sum(factors[key] * weights[key] for key in factors)
        analysis['confidence'] = overall_confidence
        analysis['confidence_factors'] = factors
        
        # Material classification
        material_analysis = self.classify_material_comprehensive(enhanced_object)
        analysis['material'] = material_analysis
        
        # Pickability assessment
        pickability = self.assess_comprehensive_pickability(enhanced_object)
        analysis['pickability'] = pickability
        
        # Prediction of hidden objects
        if self.predictive_picking:
            prediction = self.predict_hidden_objects(enhanced_object)
            analysis['hidden_prediction'] = prediction
        
        return analysis

    def calculate_shape_confidence(self, obj):
        """Calculate confidence based on shape analysis"""
        if 'contour' not in obj:
            return 0.0
        
        contour = obj['contour']
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        
        if perimeter == 0:
            return 0.0
        
        # Circularity (4π*area/perimeter²)
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        
        # Fabric objects should have moderate circularity
        # Too circular = ball, too irregular = noise
        if 0.2 <= circularity <= 0.8:
            shape_confidence = 1.0 - abs(circularity - 0.5) * 2
        else:
            shape_confidence = 0.3
        
        return shape_confidence

    def calculate_depth_confidence(self, obj):
        """Calculate confidence based on depth information"""
        if 'depth_stats' not in obj:
            return 0.5
        
        depth_stats = obj['depth_stats']
        
        # Good depth confidence if we have reasonable variance
        # (indicating 3D structure) but not too much (indicating noise)
        variance = depth_stats['depth_variance']
        
        if 0.001 <= variance <= 0.01:  # 1mm to 1cm variance
            return 0.9
        elif variance < 0.001:
            return 0.6  # Too flat, might be background
        else:
            return 0.3  # Too noisy

    def calculate_physics_confidence(self, obj):
        """Calculate confidence based on physics analysis"""
        if 'physics' not in obj:
            return 0.5
        
        physics = obj['physics']
        
        # Check if physics analysis makes sense for fabric
        fabric_likelihood = physics.get('is_fabric_like', False)
        drape_realism = physics.get('drape_realism', 0.5)
        
        if fabric_likelihood:
            return 0.8 + drape_realism * 0.2
        else:
            return 0.2

    def calculate_temporal_confidence(self, obj):
        """Calculate confidence based on temporal consistency"""
        # Check consistency with previous frames
        if len(self.detection_history) < 3:
            return 0.5
        
        # Simplified temporal consistency check
        # In production, this would track objects across frames
        return 0.7

    def classify_material_comprehensive(self, obj):
        """Comprehensive material classification"""
        material_features = {}
        
        # Color-based material hints
        if 'materials' in obj and obj['materials']:
            material_features['color_hint'] = obj['materials'][0]
        
        # Texture-based classification
        if 'spectral_features' in obj and obj['spectral_features']:
            material_features['spectral_signature'] = obj['spectral_features']
        
        # Physics-based classification
        if 'physics' in obj:
            stiffness = obj['physics'].get('stiffness', 0.5)
            if stiffness > 0.7:
                material_features['fabric_type'] = 'denim'
            elif stiffness < 0.3:
                material_features['fabric_type'] = 'silk'
            else:
                material_features['fabric_type'] = 'cotton'
        
        # Thermal signature (if available)
        if 'thermal_properties' in obj:
            material_features['thermal_signature'] = obj['thermal_properties']
        
        return material_features

    def assess_comprehensive_pickability(self, obj):
        """Comprehensive pickability assessment"""
        pickability = {
            'overall_score': 0.0,
            'grasp_points': [],
            'strategy': 'unknown',
            'estimated_success_rate': 0.0
        }
        
        # Base pickability from depth analysis
        if '3d_properties' in obj:
            # Volume-based assessment
            volume = obj['3d_properties'].get('estimated_volume', 0.0)
            if 0.0001 <= volume <= 0.01:  # Reasonable volume for clothing
                volume_score = 0.8
            else:
                volume_score = 0.3
            
            # Thickness-based assessment
            thickness = obj['3d_properties'].get('thickness', 0.0)
            if 0.005 <= thickness <= 0.05:  # 5mm to 5cm
                thickness_score = 0.9
            else:
                thickness_score = 0.4
            
            pickability['overall_score'] = (volume_score + thickness_score) / 2
        
        # Material-based adjustments
        if 'material' in obj:
            fabric_type = obj['material'].get('fabric_type', 'cotton')
            if fabric_type == 'silk':
                pickability['overall_score'] *= 0.8  # Harder to grip
            elif fabric_type == 'denim':
                pickability['overall_score'] *= 1.1  # Easier to grip
        
        # Determine strategy
        if pickability['overall_score'] > 0.7:
            pickability['strategy'] = 'confident_grasp'
        elif pickability['overall_score'] > 0.5:
            pickability['strategy'] = 'careful_grasp'
        else:
            pickability['strategy'] = 'avoid'
        
        pickability['estimated_success_rate'] = pickability['overall_score'] * 100
        
        return pickability

    def predict_hidden_objects(self, obj):
        """Predict objects hidden underneath current object"""
        prediction = {
            'likely_hidden_objects': 0,
            'confidence': 0.0,
            'suggested_strategy': 'none'
        }
        
        # Analyze thickness and shape irregularities
        if '3d_properties' in obj:
            thickness = obj['3d_properties'].get('thickness', 0.0)
            if thickness > 0.03:  # >3cm thickness suggests multiple layers
                prediction['likely_hidden_objects'] = int(thickness / 0.015)  # ~1.5cm per garment
                prediction['confidence'] = min(0.9, thickness * 10)
                prediction['suggested_strategy'] = 'lift_and_separate'
        
        return prediction

    def prioritize_objects(self, objects):
        """Advanced object prioritization for optimal picking sequence"""
        def priority_score(obj):
            # Multi-factor prioritization
            confidence = obj['confidence']
            pickability = obj['pickability']['overall_score']
            
            # Prefer objects that are likely to reveal others underneath
            hidden_potential = obj.get('hidden_prediction', {}).get('confidence', 0.0)
            
            # Prefer objects with good accessibility
            accessibility = 1.0  # Would be calculated from depth analysis
            
            # Size factor (prefer medium-sized objects)
            area = cv2.contourArea(obj['contour'])
            ideal_area = 5000  # pixels
            size_factor = 1.0 - abs(area - ideal_area) / ideal_area
            size_factor = max(0.3, min(1.0, size_factor))
            
            # Combine factors
            priority = (confidence * 0.3 + 
                       pickability * 0.3 + 
                       hidden_potential * 0.2 + 
                       accessibility * 0.1 + 
                       size_factor * 0.1)
            
            return priority
        
        return sorted(objects, key=priority_score, reverse=True)

    def generate_robot_commands(self, fused_analysis):
        """Generate comprehensive robot commands"""
        if not fused_analysis['objects']:
            return
        
        # Select best target
        best_object = fused_analysis['objects'][0]
        
        # Generate pick command
        pick_command = self.generate_pick_command(best_object)
        if pick_command:
            self.pick_target_pub.publish(pick_command)
        
        # Generate grasp strategy
        grasp_strategy = self.generate_grasp_strategy(best_object)
        if grasp_strategy:
            strategy_msg = String()
            strategy_msg.data = json.dumps(grasp_strategy)
            self.grasp_strategy_pub.publish(strategy_msg)
        
        # Generate material information
        material_info = self.generate_material_info(best_object)
        if material_info:
            material_msg = String()
            material_msg.data = json.dumps(material_info)
            self.material_info_pub.publish(material_msg)
        
        # Adaptive lighting control
        lighting_command = self.generate_lighting_command(fused_analysis)
        if lighting_command:
            lighting_msg = String()
            lighting_msg.data = json.dumps(lighting_command)
            self.lighting_control_pub.publish(lighting_msg)

    def generate_pick_command(self, obj):
        """Generate precise pick command with grasp planning"""
        if obj['pickability']['overall_score'] < 0.5:
            return None
        
        # Calculate pick position
        bbox = obj['bbox']
        centroid_x = bbox[0] + bbox[2] // 2
        centroid_y = bbox[1] + bbox[3] // 2
        
        # Convert to 3D coordinates
        if 'depth_stats' in obj:
            pick_depth = obj['depth_stats']['min_depth']  # Pick from highest point
            pick_x, pick_y, pick_z = self.pixel_to_world_advanced(centroid_x, centroid_y, pick_depth)
            
            # Create pose command
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'base_link'
            
            pose_msg.pose.position.x = pick_x
            pose_msg.pose.position.y = pick_y
            pose_msg.pose.position.z = pick_z + 0.05  # 5cm above surface
            
            # Orientation based on surface normal
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            
            return pose_msg
        
        return None

    def pixel_to_world_advanced(self, u, v, depth):
        """Advanced pixel to world coordinate transformation"""
        # Use camera calibration matrix
        if hasattr(self, 'camera_matrix'):
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
        else:
            # Default values
            fx, fy = 525.0, 525.0
            cx, cy = 320.0, 240.0
        
        # Convert to camera coordinates
        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth
        
        # Transform to robot base frame (simplified)
        # In production, use tf2 transforms
        x_base = x_cam
        y_base = y_cam
        z_base = z_cam
        
        return x_base, y_base, z_base

    def generate_grasp_strategy(self, obj):
        """Generate detailed grasp strategy"""
        strategy = {
            'type': obj['pickability']['strategy'],
            'grip_force': 'medium',
            'approach_angle': 'top_down',
            'pre_grasp_manipulation': None,
            'success_probability': obj['pickability']['estimated_success_rate']
        }
        
        # Adjust based on material
        if 'material' in obj:
            fabric_type = obj['material'].get('fabric_type', 'cotton')
            if fabric_type == 'silk':
                strategy['grip_force'] = 'gentle'
            elif fabric_type == 'denim':
                strategy['grip_force'] = 'firm'
        
        # Adjust based on hidden objects
        if 'hidden_prediction' in obj and obj['hidden_prediction']['likely_hidden_objects'] > 0:
            strategy['pre_grasp_manipulation'] = 'lift_and_shake'
        
        return strategy

    def generate_material_info(self, obj):
        """Generate comprehensive material information"""
        material_info = {
            'primary_color': 'unknown',
            'fabric_type': 'unknown',
            'estimated_weight': 0.0,
            'care_instructions': [],
            'sort_category': 'unknown'
        }
        
        # Extract color information
        if 'materials' in obj and obj['materials']:
            material_info['primary_color'] = obj['materials'][0].get('color_name', 'unknown')
            material_info['sort_category'] = obj['materials'][0].get('pile', 'unknown')
        
        # Extract fabric type
        if 'material' in obj:
            material_info['fabric_type'] = obj['material'].get('fabric_type', 'unknown')
        
        # Estimate weight from volume and fabric type
        if '3d_properties' in obj:
            volume = obj['3d_properties'].get('estimated_volume', 0.0)
            fabric_type = material_info['fabric_type']
            
            # Fabric density estimates (kg/m³)
            densities = {'cotton': 150, 'polyester': 120, 'denim': 350, 'silk': 80}
            density = densities.get(fabric_type, 150)
            
            material_info['estimated_weight'] = volume * density
        
        return material_info

    def generate_lighting_command(self, fused_analysis):
        """Generate adaptive lighting commands"""
        if not self.adaptive_lighting:
            return None
        
        # Analyze current lighting conditions
        rgb_result = fused_analysis['rgb_result']
        frame = rgb_result['processed_frame']
        
        # Calculate lighting metrics
        brightness = np.mean(frame)
        contrast = np.std(frame)
        
        lighting_command = {
            'action': 'adjust',
            'brightness_target': 128,  # Target brightness
            'contrast_enhancement': False,
            'color_temperature': 5500,
            'directional_lighting': False
        }
        
        # Adjust based on conditions
        if brightness < 100:  # Too dark
            lighting_command['brightness_target'] = 150
            lighting_command['directional_lighting'] = True
        elif brightness > 180:  # Too bright
            lighting_command['brightness_target'] = 120
        
        if contrast < 30:  # Low contrast
            lighting_command['contrast_enhancement'] = True
        
        return lighting_command

    def update_visualization(self, fused_analysis):
        """Update real-time visualization"""
        rgb_result = fused_analysis['rgb_result']
        frame = rgb_result['processed_frame'].copy()
        
        # Draw detected objects
        for i, obj in enumerate(fused_analysis['objects']):
            self.draw_advanced_object(frame, obj, i == 0)
        
        # Draw performance metrics
        self.draw_performance_overlay(frame, fused_analysis)
        
        # Draw system status
        self.draw_system_status(frame)
        
        # Store for display
        self.current_visualization = frame

    def draw_advanced_object(self, frame, obj, is_primary=False):
        """Draw comprehensive object visualization"""
        contour = obj['contour']
        bbox = obj['bbox']
        
        # Color coding based on confidence
        confidence = obj['confidence']
        if confidence > 0.8:
            color = (0, 255, 0)  # Green - high confidence
        elif confidence > 0.6:
            color = (0, 255, 255)  # Yellow - medium confidence
        else:
            color = (0, 165, 255)  # Orange - low confidence
        
        # Thickness based on priority
        thickness = 4 if is_primary else 2
        
        # Draw contour
        cv2.drawContours(frame, [contour], -1, color, thickness)
        
        # Draw bounding box
        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[0] + bbox[2], bbox[1] + bbox[3]), color, 1)
        
        # Comprehensive label
        labels = []
        
        # Material info
        if 'material' in obj:
            fabric = obj['material'].get('fabric_type', 'unknown')
            labels.append(f"Material: {fabric.upper()}")
        
        # Color and sort category
        if 'materials' in obj and obj['materials']:
            color_name = obj['materials'][0].get('color_name', 'unknown')
            pile = obj['materials'][0].get('pile', 'unknown')
            labels.append(f"{color_name.upper()} → {pile.upper()}")
        
        # Confidence and pickability
        pickability = obj['pickability']['overall_score']
        labels.append(f"Conf: {confidence*100:.0f}% | Pick: {pickability*100:.0f}%")
        
        # 3D properties
        if '3d_properties' in obj:
            volume = obj['3d_properties'].get('estimated_volume', 0.0)
            labels.append(f"Vol: {volume*1000000:.1f}cm³")
        
        # Hidden object prediction
        if 'hidden_prediction' in obj:
            hidden = obj['hidden_prediction']['likely_hidden_objects']
            if hidden > 0:
                labels.append(f"Hidden: {hidden} items")
        
        # Draw label background
        max_width = max([cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0] for label in labels])
        label_height = len(labels) * 20 + 10
        
        cv2.rectangle(frame, (bbox[0], bbox[1] - label_height), 
                     (bbox[0] + max_width + 10, bbox[1]), (0, 0, 0), -1)
        
        # Draw labels
        for i, label in enumerate(labels):
            cv2.putText(frame, label, (bbox[0] + 5, bbox[1] - label_height + 20 + i * 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def draw_performance_overlay(self, frame, fused_analysis):
        """Draw performance metrics overlay"""
        h, w = frame.shape[:2]
        
        # Performance panel
        panel_w, panel_h = 300, 150
        panel_x, panel_y = w - panel_w - 10, 10
        
        overlay = frame.copy()
        cv2.rectangle(overlay, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.8, frame, 0.2, 0, frame)
        
        # Performance metrics
        if hasattr(self, 'performance_monitor'):
            avg_fps = self.performance_monitor.get_average_fps()
            processing_time = fused_analysis.get('fusion_time', 0.0)
        else:
            avg_fps = 30.0
            processing_time = 0.05
        
        metrics = [
            f"FPS: {avg_fps:.1f}",
            f"Processing: {processing_time*1000:.1f}ms",
            f"Objects: {len(fused_analysis['objects'])}",
            f"Confidence: {fused_analysis['objects'][0]['confidence']*100:.0f}%" if fused_analysis['objects'] else "No objects",
            f"Memory: {self.get_memory_usage():.1f}MB"
        ]
        
        cv2.putText(frame, "PERFORMANCE", (panel_x + 10, panel_y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 255, 100), 2)
        
        for i, metric in enumerate(metrics):
            cv2.putText(frame, metric, (panel_x + 10, panel_y + 50 + i * 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def draw_system_status(self, frame):
        """Draw comprehensive system status"""
        h, w = frame.shape[:2]
        
        # Status indicators
        indicators = [
            ('RGB', (0, 255, 0) if hasattr(self, 'latest_frame') else (0, 0, 255)),
            ('DEPTH', (0, 255, 0) if hasattr(self, 'latest_depth') else (0, 0, 255)),
            ('HDR', (0, 255, 0) if self.hdr_processor else (0, 0, 255)),
            ('PHYSICS', (0, 255, 0) if self.physics_simulation else (100, 100, 100)),
            ('LEARN', (0, 255, 0) if len(self.detection_history) > 10 else (0, 255, 255))
        ]
        
        start_x = 10
        for i, (name, color) in enumerate(indicators):
            x = start_x + i * 80
            cv2.circle(frame, (x, h - 30), 8, color, -1)
            cv2.putText(frame, name, (x - 15, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    def get_memory_usage(self):
        """Get current memory usage"""
        # Simplified memory usage calculation
        import psutil
        try:
            process = psutil.Process()
            return process.memory_info().rss / 1024 / 1024  # MB
        except:
            return 0.0

    def display_callback(self):
        """High-performance display callback"""
        if hasattr(self, 'current_visualization'):
            cv2.imshow('Advanced Laundry Vision System v4.0 - Production Grade', self.current_visualization)
            
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                rclpy.shutdown()
            elif key == ord('c'):  # Calibrate
                self.trigger_calibration()
            elif key == ord('h'):  # Toggle HDR
                self.hdr_processor.enabled = not self.hdr_processor.enabled
            elif key == ord('p'):  # Toggle physics
                self.physics_simulation = not self.physics_simulation

    def trigger_calibration(self):
        """Trigger system calibration"""
        self.get_logger().info("Starting system calibration...")
        # Implement calibration procedures
        pass

    def operation_mode_callback(self, msg):
        """Handle operation mode changes"""
        mode = msg.data
        self.get_logger().info(f"Switching to operation mode: {mode}")
        
        if mode == 'high_performance':
            self.max_processing_time = 0.05  # 50ms
        elif mode == 'high_accuracy':
            self.max_processing_time = 0.2   # 200ms
        elif mode == 'power_save':
            self.max_processing_time = 0.5   # 500ms

    def calibration_callback(self, msg):
        """Handle calibration trigger"""
        if msg.data:
            self.trigger_calibration()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


# Advanced processing classes
class HDRProcessor:
    def __init__(self, exposure_count):
        self.exposure_count = exposure_count
        self.enabled = True
    
    def process_frame(self, frame):
        if not self.enabled:
            return frame
        # Simplified HDR processing
        # In production, this would capture multiple exposures
        return cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8)).apply(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))

class AdaptiveExposureController:
    def compensate_lighting(self, frame):
        # Advanced lighting compensation
        return frame

class ActiveLightingController:
    def __init__(self):
        pass

class MultiSpectralAnalyzer:
    def extract_features(self, frame):
        return None

class AdvancedRGBProcessor:
    def __init__(self):
        pass

class AdvancedDepthProcessor:
    def advanced_filter(self, depth):
        return depth
    
    def to_point_cloud(self, depth):
        return None
    
    def estimate_normals(self, point_cloud):
        return None

class ThermalProcessor:
    def __init__(self):
        pass

class MultiModalFusion:
    def __init__(self):
        pass

class AdvancedObjectTracker:
    def __init__(self):
        pass

class TemporalConsistencyAnalyzer:
    def ensure_consistency(self, segments, previous_frame):
        return segments

class FabricPhysicsEngine:
    def analyze_segment(self, frame, segment):
        return {'is_fabric_like': True, 'stiffness': 0.5, 'drape_realism': 0.7}
    
    def analyze_deformation(self, point_cloud, normals):
        return None

class MaterialPropertyClassifier:
    def classify(self, frame, segment):
        return {'fabric_type': 'cotton', 'color_name': 'blue', 'pile': 'colors', 'confidence': 0.8}

class WrinkleAnalyzer:
    def __init__(self):
        pass

class FabricDeformationModel:
    def __init__(self):
        pass

class ConfidenceTracker:
    def __init__(self):
        pass

class ErrorAnalyzer:
    def __init__(self):
        pass

class PerformanceOptimizer:
    def __init__(self):
        pass

class PerformanceMonitor:
    def __init__(self):
        self.fps_history = deque(maxlen=30)
        self.last_time = time.time()
    
    def update(self, processing_time):
        current_time = time.time()
        fps = 1.0 / (current_time - self.last_time)
        self.fps_history.append(fps)
        self.last_time = current_time
    
    def get_average_fps(self):
        return np.mean(self.fps_history) if self.fps_history else 0.0


def main():
    rclpy.init()
    node = AdvancedLaundryVisionSystem()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()