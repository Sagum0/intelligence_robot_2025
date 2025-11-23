#!/usr/bin/env python3

import sys
import time
import numpy as np
import cv2
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32MultiArray
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # 파라미터
        self.declare_parameter('marker_size', 0.03) # 미터 단위 (30mm)
        self.declare_parameter('dictionary_id', 'DICT_4X4_50')
        
        self.marker_size = self.get_parameter('marker_size').value
        self.dictionary_id_str = self.get_parameter('dictionary_id').value

        # ArUco 딕셔너리 설정
        # 4x4 및 6x6 딕셔너리 모두 로드
        self.dictionaries = {
            "4x4": cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
            "6x6": cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        }
        self.get_logger().info("Loaded ArUco Dictionaries: DICT_4X4_50, DICT_6X6_50")
        self.aruco_params = cv2.aruco.DetectorParameters()

        # RealSense 설정
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        # 깊이와 컬러 정렬
        self.align = rs.align(rs.stream.color)

        try:
            self.pipeline.start(self.config)
            self.get_logger().info("RealSense Camera Started")
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense: {e}")
            sys.exit(1)

        # 서브스크라이버
        self.trigger_sub = self.create_subscription(
            Int32,
            '/kinova/int/camera_trigger',
            self.trigger_callback,
            10
        )

        # 퍼블리셔
        self.pose_pub = self.create_publisher(Float32MultiArray, '/kinova/vision/detected_pose', 10)

        # 최신 프레임 저장
        self.latest_color_image = None
        self.latest_camera_matrix = None
        self.latest_dist_coeffs = None

        self.get_logger().info("Vision Node Initialized. Waiting for trigger...")

    def update_frame(self):
        # 일관된 프레임 쌍 대기: 깊이 및 컬러
        # 프레임이 없을 경우 ROS 스피닝을 너무 오래 차단하지 않도록 짧은 타임아웃 사용
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
        cv2.imshow("RealSense Feed", self.latest_color_image)
        cv2.waitKey(1)

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
        target_id = msg.data
        self.get_logger().info(f"Trigger received. Searching for Marker ID: {target_id}")

        if self.latest_color_image is None:
            self.get_logger().warn("No frame available yet")
            return

        color_image = self.latest_color_image.copy()
        camera_matrix = self.latest_camera_matrix
        dist_coeffs = self.latest_dist_coeffs

        # 여러 딕셔너리를 사용하여 마커 감지
        found_marker = False
        
        for dict_name, dictionary in self.dictionaries.items():
            corners, ids, rejected = cv2.aruco.detectMarkers(
                color_image, dictionary, parameters=self.aruco_params
            )

            if ids is not None and target_id in ids:
                self.get_logger().info(f"Found Marker ID {target_id} in {dict_name} dictionary")
                index = np.where(ids == target_id)[0][0]
                
                # 포즈 추정
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[index], self.marker_size, camera_matrix, dist_coeffs
                )

                # rvec과 tvec은 (1, 1, 3) 형태임
                rvec = rvec[0][0]
                tvec = tvec[0][0]

                self.get_logger().info(f"Found Marker {target_id} at {tvec}")

                # rvec(Rodrigues)을 회전 행렬(3x3)로 변환
                rotation_matrix, _ = cv2.Rodrigues(rvec)

                # 4x4 동차 변환 행렬 구성
                transform_matrix = np.eye(4, dtype=np.float32)
                transform_matrix[0:3, 0:3] = rotation_matrix
                transform_matrix[0:3, 3] = tvec

                # Float32MultiArray 발행 (평탄화된 4x4 행렬)
                matrix_msg = Float32MultiArray()
                matrix_msg.data = transform_matrix.flatten().tolist()

                self.pose_pub.publish(matrix_msg)
                self.get_logger().info("Pose matrix published")

                # 시각화를 위한 축 그리기
                cv2.drawFrameAxes(self.latest_color_image, camera_matrix, dist_coeffs, rvec, tvec, 0.01)
                # 마커 테두리 그리기
                cv2.aruco.drawDetectedMarkers(self.latest_color_image, corners)
                
                found_marker = True
                break # Stop searching other dictionaries if found

        if found_marker:
            # 그림을 즉시 표시하기 위해 디스플레이 강제 업데이트
            cv2.imshow("RealSense Feed", self.latest_color_image)
            cv2.waitKey(1)
            
            # 디버깅 시각화를 위해 일시 정지 (0.5초)
            self.get_logger().info("Pausing display for visualization...")
            time.sleep(0.5)

        else:
            self.get_logger().warn(f"Marker ID {target_id} not found in any dictionary")

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
