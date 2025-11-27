#!/usr/bin/env python3

import sys
import time
import numpy as np
import cv2
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.declare_parameter('marker_size', 0.03)
        self.declare_parameter('dictionary_id', 'DICT_4X4_50')
        
        self.marker_size = self.get_parameter('marker_size').value
        self.dictionary_id_str = self.get_parameter('dictionary_id').value

        self.dictionaries = {
            "6x6": cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        }
        self.get_logger().info("Loaded ArUco Dictionaries: DICT_6X6_50")
        self.aruco_params = cv2.aruco.DetectorParameters()

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

        # 서브스크라이버
        self.trigger_sub = self.create_subscription(
            Int32MultiArray,
            '/kinova/array/camera_trigger',
            self.trigger_callback,
            10
        )

        # 퍼블리셔
        self.pose_pub = self.create_publisher(Float32MultiArray, '/kinova/array/detected_pose', 10)

        # 최신 프레임 저장
        self.latest_color_image = None
        self.latest_camera_matrix = None
        self.latest_dist_coeffs = None

        # Multi-Marker Search State
        self.target_ids = []
        self.detected_poses = {} # {id: 4x4_matrix}
        self.searching = False

        cv2.namedWindow("RealSense Feed", cv2.WINDOW_AUTOSIZE)
        self.get_logger().info("Vision Node Initialized. Waiting for trigger...")

    def get_camera_matrix(self, profile):
        intrinsics = profile.as_video_stream_profile().get_intrinsics()
        camera_matrix = np.array([
            [intrinsics.fx, 0, intrinsics.ppx],
            [0, intrinsics.fy, intrinsics.ppy],
            [0, 0, 1]
        ])
        dist_coeffs = np.array(intrinsics.coeffs)
        return camera_matrix, dist_coeffs

    def trigger_callback(self, msg):
        self.target_ids = list(msg.data)
        self.detected_poses = {}
        self.searching = True
        self.get_logger().info(f"Trigger received. Searching for Marker IDs: {self.target_ids}")

    def update_frame(self):
        # 일관된 프레임 쌍 대기: 깊이 및 컬러
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=100)
        except RuntimeError:
            return

        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        
        if not color_frame:
            return

        # 이미지를 numpy 배열로 변환
        self.latest_color_image = np.asanyarray(color_frame.get_data())

        # 카메라 내부 파라미터 가져오기
        profile = color_frame.get_profile()
        self.latest_camera_matrix, self.latest_dist_coeffs = self.get_camera_matrix(profile)

        # 디스플레이
        display_image = self.latest_color_image.copy()

        if self.searching:
            self.process_search(display_image)

        cv2.imshow("RealSense Feed", display_image)
        cv2.waitKey(1)

    def process_search(self, display_image):
        color_image = self.latest_color_image # Use raw image for detection
        camera_matrix = self.latest_camera_matrix
        dist_coeffs = self.latest_dist_coeffs

        # Detect markers using multiple dictionaries
        # We need to find ALL target_ids.
        
        # Optimization: Only search if we still need to find markers
        if len(self.detected_poses) == len(self.target_ids):
            self.publish_results()
            return

        for dict_name, dictionary in self.dictionaries.items():
            corners, ids, rejected = cv2.aruco.detectMarkers(
                color_image, dictionary, parameters=self.aruco_params
            )

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display_image, corners)
                
                ids_flatten = ids.flatten()
                for i, marker_id in enumerate(ids_flatten):
                    if marker_id in self.target_ids and marker_id not in self.detected_poses:
                        self.get_logger().info(f"Found Marker ID {marker_id} in {dict_name}")
                        
                        # Estimate pose
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                            corners[i], self.marker_size, camera_matrix, dist_coeffs
                        )
                        
                        rvec = rvec[0][0]
                        tvec = tvec[0][0]

                        # Draw axis
                        cv2.drawFrameAxes(display_image, camera_matrix, dist_coeffs, rvec, tvec, 0.01)

                        # Convert to Matrix
                        rotation_matrix, _ = cv2.Rodrigues(rvec)
                        transform_matrix = np.eye(4, dtype=np.float32)
                        transform_matrix[0:3, 0:3] = rotation_matrix
                        transform_matrix[0:3, 3] = tvec
                        
                        self.detected_poses[marker_id] = transform_matrix

        # Check if all found
        if len(self.detected_poses) == len(self.target_ids):
            self.get_logger().info("All requested markers found!")
            self.publish_results()

    def publish_results(self):
        # Construct 4x4xN array (flattened)
        # Order: based on self.target_ids
        
        final_data = []
        for tid in self.target_ids:
            pose_matrix = self.detected_poses[tid]
            final_data.extend(pose_matrix.flatten().tolist())
            
        matrix_msg = Float32MultiArray()
        matrix_msg.data = final_data
        
        self.pose_pub.publish(matrix_msg)
        self.get_logger().info(f"Published poses for IDs: {self.target_ids}")
        
        self.searching = False
        self.target_ids = []
        self.detected_poses = {}

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
