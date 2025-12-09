#!/usr/bin/env python3

import sys
import time
import numpy as np
import cv2
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray

# Constants
MAKER_SIZE = 0.03 # meters
DICT_ID    = cv2.aruco.DICT_6X6_50

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node_custom')
        
        # 1. Realsense Setup
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.align = rs.align(rs.stream.color)
        
        try:
            self.pipeline.start(self.config)
            self.get_logger().info("RealSense Camera Started")
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense: {e}")
            sys.exit(1)

        # 2. ArUco Setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # 3. ROS Communications
        self.trigger_sub = self.create_subscription(
            Int32MultiArray,
            '/kinova/array/camera_trigger',
            self.trigger_callback,
            10
        )
        
        self.pose_pub = self.create_publisher(
            Float32MultiArray,
            '/kinova/array/detected_pose',
            10
        )
        
        # State variables
        self.target_ids = []      # List of IDs to find
        self.detected_poses = {}  # Dictionary to store found poses: {id: 4x4 matrix}
        self.searching = False    # Flag to enable detection loop
        
        # Camera Intrinsics (will be updated from the first frame)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.get_logger().info("Vision Node Custom Initialized. Waiting for trigger...")

    def get_camera_matrix(self, profile):
        intrinsics = profile.as_video_stream_profile().get_intrinsics()
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ])
        dist_coeffs = np.array(intrinsics.coeffs)
        return camera_matrix, dist_coeffs

    def trigger_callback(self, msg : Int32MultiArray):
        self.target_ids = list(msg.data)
        self.detected_poses = {} # Reset detected poses
        self.searching = True
        self.get_logger().info(f"Trigger received. Searching for Marker IDs: {self.target_ids}")

    def update_frame(self):
        # Get frames from Realsense
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
        except RuntimeError:
            return

        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        
        if not color_frame:
            return

        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())
        
        # Get intrinsics if not already set
        if self.camera_matrix is None:
            profile = color_frame.get_profile()
            self.camera_matrix, self.dist_coeffs = self.get_camera_matrix(profile)

        # Display image (copy for drawing)
        display_image = color_image.copy()

        # Process detection if searching
        if self.searching:
            self.process_detection(color_image, display_image)

        # Always show the feed
        cv2.imshow("RealSense Feed", display_image)
        cv2.waitKey(1)

    def process_detection(self, input_image, display_image):
        # Detect markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            input_image, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None:
            ids_flatten = ids.flatten()
            
            for i, marker_id in enumerate(ids_flatten):
                if marker_id in self.target_ids:
                    
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i], MAKER_SIZE, self.camera_matrix, self.dist_coeffs
                    )
                    
                    rvec = rvec[0][0]
                    tvec = tvec[0][0]
                    
                    cv2.aruco.drawDetectedMarkers(display_image, corners, ids, borderColor=(0, 255, 0))
                    cv2.drawFrameAxes(display_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.02)
                    
                    if marker_id not in self.detected_poses:
                        self.get_logger().info(f"Found Target Marker ID: {marker_id}")
                        
                        rotation_matrix, _ = cv2.Rodrigues(rvec)
                        transform_matrix = np.eye(4, dtype=np.float32)
                        transform_matrix[0:3, 0:3] = rotation_matrix
                        transform_matrix[0:3, 3] = tvec
                        
                        self.detected_poses[marker_id] = transform_matrix

        if len(self.detected_poses) == len(self.target_ids):
            self.publish_results()

    def publish_results(self):
        self.get_logger().info("All requested markers found. Publishing results...")
        
        final_data = []
        for tid in self.target_ids:
            pose_matrix = self.detected_poses[tid]
            final_data.extend(pose_matrix.flatten().tolist())
            
        msg = Float32MultiArray()
        msg.data = final_data
        self.pose_pub.publish(msg)
        
        self.searching = False
        self.target_ids = []
        self.detected_poses = {}
        self.get_logger().info("Search complete. Waiting for new trigger.")

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.001)
            node.update_frame()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()