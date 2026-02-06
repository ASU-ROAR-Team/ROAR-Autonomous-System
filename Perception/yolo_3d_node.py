#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Header
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import numpy as np
import cv2
import sys
import os

try:
    import pyzed.sl as sl
    ZED_AVAILABLE = True
    print("ZED SDK Python API is available")
except ImportError:
    ZED_AVAILABLE = False
    print("ZED SDK Python API not available. Install with: pip install pyzed")
    print("Running in simulation mode with mock data")


class ZEDYOLO3DNode(Node):
    def __init__(self):
        super().__init__('zed_yolo_3d_node')
        
        self.bridge = CvBridge()
        
        # Initialize ZED camera if available
        if ZED_AVAILABLE:
            self.zed = sl.Camera()
            self.init_zed_camera()
        else:
            self.zed = None
            self.get_logger().warn("Running in simulation mode - no real ZED camera")
        
        # Check CUDA
        if torch.cuda.is_available():
            self.device = "cuda"
            self.get_logger().info("Using CUDA for YOLO")
        else:
            self.device = "cpu"
            self.get_logger().info("Using CPU for YOLO")
        
        # Load YOLO model
        self.model = YOLO("yolov8n.pt")
        self.get_logger().info(f"✅ Loaded YOLO model with {len(self.model.names)} classes")
        
        # Camera parameters (will be filled from ZED)
        self.camera_matrix = None
        self.depth_scale = 0.001  # Default: depth in meters
        
        # 📏 FIXED DIMENSIONS FOR EACH CLASS (Width, Height, Depth) in meters
        self.FIXED_DIMENSIONS = {
            'person': [0.5, 1.7, 0.5],      # Width, Height, Depth
            'car': [1.8, 1.5, 4.0],         # Small car
            'truck': [2.5, 3.0, 8.0],       # Truck
            'bus': [2.5, 3.5, 12.0],        # Bus
            'bicycle': [0.7, 1.2, 1.8],     # Bicycle
            'motorcycle': [0.8, 1.3, 2.0],  # Motorcycle
            'cat': [0.3, 0.3, 0.5],         # Cat
            'dog': [0.4, 0.5, 0.8],         # Dog
            'chair': [0.5, 0.9, 0.5],       # Chair
            'bench': [1.5, 0.9, 0.5],       # Bench
            'couch': [2.0, 0.8, 0.9],       # Couch/Sofa
            'potted plant': [0.4, 0.6, 0.4], # Potted plant
            'bed': [2.0, 0.5, 1.5],         # Bed
            'dining table': [1.2, 0.8, 1.0], # Table
            'tv': [1.0, 0.6, 0.1],          # TV
            'laptop': [0.3, 0.02, 0.2],     # Laptop
            'cell phone': [0.08, 0.15, 0.01], # Phone
        }
        
        # 🎨 COLORS FOR EACH CLASS (BGR format)
        self.CLASS_COLORS = {
            'person': (0, 255, 0),      # Green
            'car': (255, 0, 0),         # Blue
            'truck': (0, 0, 255),       # Red
            'bus': (0, 255, 255),       # Yellow
            'bicycle': (255, 0, 255),   # Magenta
            'motorcycle': (255, 255, 0), # Cyan
            'cat': (255, 165, 0),       # Orange
            'dog': (128, 0, 128),       # Purple
        }
        
        # Subscribe to ZED topics (if not using direct ZED SDK)
        self.image_sub = self.create_subscription(
            Image,
            "/zed/zed_node/rgb/color/rect/image",
            self.image_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            "/zed/zed_node/depth/depth_registered",
            self.depth_callback,
            10
        )
        
        # Publishers
        self.image_pub = self.create_publisher(Image, "/yolo_3d/image_annotated", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/yolo_3d/markers", 10)
        self.bbox3d_pub = self.create_publisher(PoseArray, "/yolo_3d/bounding_boxes", 10)
        self.detections_pub = self.create_publisher(PoseArray, "/yolo_3d/detections", 10)
        
        # Storage
        self.latest_rgb = None
        self.latest_depth = None
        self.frame_count = 0
        
        self.get_logger().info("✅ ZED YOLO 3D Node started")
        self.get_logger().info("📏 Using FIXED dimensions for all objects")
        
    def init_zed_camera(self):
        """Initialize ZED camera if using direct SDK."""
        try:
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD1080
            init_params.depth_mode = sl.DEPTH_MODE.ULTRA
            init_params.coordinate_units = sl.UNIT.METER
            init_params.depth_maximum_distance = 20.0
            init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
            
            # Open the camera
            err = self.zed.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f"Failed to open ZED camera: {err}")
                self.zed = None
                return
            
            # Get camera information
            calibration_params = self.zed.get_camera_information().camera_configuration.calibration_parameters
            left_cam = calibration_params.left_cam
            
            # Camera matrix
            self.camera_matrix = np.array([
                [left_cam.fx, 0, left_cam.cx],
                [0, left_cam.fy, left_cam.cy],
                [0, 0, 1]
            ])
            
            self.get_logger().info("✅ ZED camera initialized successfully")
            self.get_logger().info(f"📐 Camera matrix:\n{self.camera_matrix}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ZED camera: {e}")
            self.zed = None
    
    def image_callback(self, msg):
        """Callback for RGB image."""
        self.latest_rgb = msg
    
    def depth_callback(self, msg):
        """Callback for depth image."""
        self.latest_depth = msg
        self.process_frame()
    
    def pixel_to_3d(self, u, v, depth_value):
        """Convert pixel coordinates and depth to 3D point."""
        if self.camera_matrix is None:
            # Default camera matrix if not available
            fx, fy = 525.0, 525.0  # Default focal length
            cx, cy = 320.0, 240.0  # Default principal point
        else:
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
        
        # Convert to 3D
        z = depth_value
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        return np.array([x, y, z])
    
    def get_3d_center_fixed(self, bbox_2d, depth_image):
        """Get 3D center point using fixed approach - only center point."""
        x1, y1, x2, y2 = bbox_2d
        
        # Calculate center of 2D bounding box
        center_u = int((x1 + x2) / 2)
        center_v = int((y1 + y2) / 2)
        
        # Get depth at center point
        if 0 <= center_u < depth_image.shape[1] and 0 <= center_v < depth_image.shape[0]:
            depth = depth_image[center_v, center_u]
            
            # Validate depth value
            if 0.3 < depth < 20.0:  # Valid range: 0.3m to 20m
                # Convert to 3D
                point_3d = self.pixel_to_3d(center_u, center_v, depth)
                return point_3d.tolist()
        
        return None
    
    def get_dimensions_for_class(self, class_name):
        """Get fixed dimensions for a specific class."""
        class_lower = class_name.lower()
        
        # Check exact match
        if class_lower in self.FIXED_DIMENSIONS:
            return self.FIXED_DIMENSIONS[class_lower]
        
        # Check for partial matches
        for key, value in self.FIXED_DIMENSIONS.items():
            if key in class_lower or class_lower in key:
                return value
        
        # Default dimensions for unknown classes
        self.get_logger().warn(f"⚠️ Unknown class '{class_name}', using default dimensions")
        return [1.0, 1.0, 1.0]  # Default: 1m cube
    
    def get_color_for_class(self, class_name):
        """Get BGR color for different object classes."""
        class_lower = class_name.lower()
        
        # Check exact match
        if class_lower in self.CLASS_COLORS:
            return self.CLASS_COLORS[class_lower]
        
        # Check for partial matches
        for key, color in self.CLASS_COLORS.items():
            if key in class_lower or class_lower in key:
                return color
        
        # Default color: White
        return (255, 255, 255)
    
    def create_3d_marker(self, class_name, center_3d, dimensions, conf, marker_id, header):
        """Create a 3D visualization marker with fixed dimensions."""
        # Main cube marker
        marker = Marker()
        marker.header = header
        marker.ns = f"{class_name}_3d"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = float(center_3d[0])
        marker.pose.position.y = float(center_3d[1])
        marker.pose.position.z = float(center_3d[2])
        marker.pose.orientation.w = 1.0
        
        # 🎯 SET FIXED DIMENSIONS
        marker.scale.x = float(dimensions[0])  # Width
        marker.scale.y = float(dimensions[1])  # Height
        marker.scale.z = float(dimensions[2])  # Depth
        
        # Set color based on class
        color = self.get_color_for_class(class_name)
        marker.color.r = color[2] / 255.0  # OpenCV BGR to ROS RGB
        marker.color.g = color[1] / 255.0
        marker.color.b = color[0] / 255.0
        marker.color.a = 0.4  # Semi-transparent
        
        marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
        
        return marker
    
    def create_text_marker(self, class_name, center_3d, dimensions, conf, marker_id, header):
        """Create text marker with class name and distance."""
        text_marker = Marker()
        text_marker.header = header
        text_marker.ns = f"{class_name}_text"
        text_marker.id = marker_id
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        
        # Position above the cube
        text_marker.pose.position.x = float(center_3d[0])
        text_marker.pose.position.y = float(center_3d[1])
        text_marker.pose.position.z = float(center_3d[2]) + float(dimensions[1]) / 2 + 0.2
        text_marker.pose.orientation.w = 1.0
        
        text_marker.scale.z = 0.15  # Text height
        
        # Calculate distance
        distance = np.sqrt(center_3d[0]**2 + center_3d[1]**2 + center_3d[2]**2)
        
        # Set text
        text_marker.text = f"{class_name}\n{distance:.1f}m ({conf:.2f})"
        
        # White color for text
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        
        text_marker.lifetime = rclpy.duration.Duration(seconds=1).to_msg()
        
        return text_marker
    
    def create_detection_pose(self, center_3d, dimensions, conf):
        """Create Pose message for detection."""
        pose = Pose()
        pose.position.x = float(center_3d[0])
        pose.position.y = float(center_3d[1])
        pose.position.z = float(center_3d[2])
        
        # Encode dimensions in orientation (as a hack)
        pose.orientation.x = float(dimensions[0])  # width
        pose.orientation.y = float(dimensions[1])  # height
        pose.orientation.z = float(dimensions[2])  # depth
        pose.orientation.w = float(conf)  # confidence
        
        return pose
    
    def draw_2d_annotations(self, frame, bbox_2d, class_name, conf, center_3d):
        """Draw 2D annotations on the image."""
        x1, y1, x2, y2 = bbox_2d
        
        # Get color for this class
        color = self.get_color_for_class(class_name)
        
        # Draw bounding box
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
        
        # Calculate distance
        distance = np.sqrt(center_3d[0]**2 + center_3d[1]**2 + center_3d[2]**2)
        
        # Create label
        label = f"{class_name} {conf:.2f} ({distance:.1f}m)"
        
        # Calculate text size
        (text_width, text_height), baseline = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
        )
        
        # Draw background rectangle for text
        cv2.rectangle(
            frame,
            (int(x1), int(y1) - text_height - 10),
            (int(x1) + text_width, int(y1)),
            color,
            -1  # Filled rectangle
        )
        
        # Draw text
        cv2.putText(
            frame,
            label,
            (int(x1), int(y1) - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),  # White text
            2
        )
        
        # Draw center point
        center_u = int((x1 + x2) / 2)
        center_v = int((y1 + y2) / 2)
        cv2.circle(frame, (center_u, center_v), 5, (0, 255, 255), -1)  # Yellow center
        
        return frame
    
    def process_frame(self):
        """Main processing function."""
        if self.latest_rgb is None or self.latest_depth is None:
            return
        
        self.frame_count += 1
        
        try:
            # Convert ROS images to OpenCV
            rgb_frame = self.bridge.imgmsg_to_cv2(self.latest_rgb, "bgr8")
            depth_frame = self.bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding="32FC1")
            
            # Convert depth to meters if needed
            if depth_frame.dtype != np.float32:
                depth_frame = depth_frame.astype(np.float32) * 0.001
            
            # Run YOLO inference
            results = self.model(rgb_frame, device=self.device, conf=0.5, verbose=False)[0]
            
            # Create output containers
            marker_array = MarkerArray()
            pose_array = PoseArray()
            pose_array.header = self.latest_rgb.header
            
            detections_array = PoseArray()
            detections_array.header = self.latest_rgb.header
            
            annotated_frame = rgb_frame.copy()
            
            if results.boxes is not None:
                boxes = results.boxes.cpu().numpy()
                
                if len(boxes) > 0:
                    self.get_logger().info(f"📦 Frame {self.frame_count}: Detected {len(boxes)} objects")
                
                for i in range(len(boxes)):
                    # Extract 2D bounding box
                    x1, y1, x2, y2 = boxes.xyxy[i]
                    conf = float(boxes.conf[i])
                    cls_id = int(boxes.cls[i])
                    class_name = self.model.names.get(cls_id, f"unknown_{cls_id}")
                    
                    # Filter by confidence
                    if conf < 0.4:  # Lower threshold for more detections
                        continue
                    
                    # Filter by interesting classes (optional)
                    interesting_classes = ['person', 'car', 'truck', 'bus', 'bicycle', 'motorcycle', 
                                          'cat', 'dog', 'chair', 'bench', 'couch']
                    if class_name.lower() not in [c.lower() for c in interesting_classes]:
                        continue
                    
                    # Convert to integers for pixel operations
                    bbox_2d = [int(x1), int(y1), int(x2), int(y2)]
                    
                    # 🎯 GET 3D CENTER WITH FIXED APPROACH
                    center_3d = self.get_3d_center_fixed(bbox_2d, depth_frame)
                    
                    if center_3d is None:
                        continue  # Skip if no valid depth
                    
                    # 📏 GET FIXED DIMENSIONS FOR THIS CLASS
                    dimensions = self.get_dimensions_for_class(class_name)
                    
                    # 🎨 CREATE 3D MARKER
                    cube_marker = self.create_3d_marker(
                        class_name, center_3d, dimensions, conf, i, self.latest_rgb.header
                    )
                    marker_array.markers.append(cube_marker)
                    
                    # 📝 CREATE TEXT MARKER
                    text_marker = self.create_text_marker(
                        class_name, center_3d, dimensions, conf, i + 1000, self.latest_rgb.header
                    )
                    marker_array.markers.append(text_marker)
                    
                    # 📦 CREATE POSE FOR BOUNDING BOX
                    bbox_pose = self.create_detection_pose(center_3d, dimensions, conf)
                    pose_array.poses.append(bbox_pose)
                    get_logger
                    # 🎯 CREATE SIMPLE DETECTION POSE (center only)
                    det_pose = Pose()
                    det_pose.position.x = float(center_3d[0])
                    det_pose.position.y = float(center_3d[1])
                    det_pose.position.z = float(center_3d[2])
                    det_pose.orientation.w = float(conf)
                    detections_array.poses.append(det_pose)
                    
                    # 🖼️ DRAW 2D ANNOTATIONS
                    annotated_frame = self.draw_2d_annotations(
                        annotated_frame, bbox_2d, class_name, conf, center_3d
                    )
                    
                    # 📊 LOG DETECTION INFO
                    distance = np.sqrt(center_3d[0]**2 + center_3d[1]**2 + center_3d[2]**2)
                    if distance ==0 :
                        
                    self.get_logger().info(
                        f"  📍 {class_name} (ID: {i}): "
                        f"Distance: {distance:.2f}m, "
                        f"Position: ({center_3d[0]:.2f}, {center_3d[1]:.2f}, {center_3d[2]:.2f}), "
                        f"Conf: {conf:.2f}, "
                        f"Dims: {dimensions[0]:.1f}x{dimensions[1]:.1f}x{dimensions[2]:.1f}m"
                    )
            
            # Publish annotated image
            img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            img_msg.header = self.latest_rgb.header
            self.image_pub.publish(img_msg)
            
            # Publish markers and poses
            if len(marker_array.markers) > 0:
                self.marker_pub.publish(marker_array)
                self.bbox3d_pub.publish(pose_array)
                self.detections_pub.publish(detections_array)
                
                # Log summary
                self.get_logger().info(f"✅ Published {len(marker_array.markers)//2} objects")
            
        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def destroy_node(self):
        """Clean shutdown."""
        if self.zed is not None and ZED_AVAILABLE:
            self.zed.close()
            self.get_logger().info("ZED camera closed")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ZEDYOLO3DNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Keyboard interrupt received")
    except Exception as e:
        node.get_logger().error(f"💥 Node error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
