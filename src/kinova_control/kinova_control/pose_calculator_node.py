#!/usr/bin/env python3

import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from scipy.spatial.transform import Rotation as R

class PoseCalculatorNode(Node):
    def __init__(self):
        super().__init__('pose_calculator_node')
        self.current_joint_angles = None
        self.joint_sub = self.create_subscription(
            JointState,
            '/kinova/joint_states',
            self.joint_callback,
            10
        )
        
        self.vision_sub = self.create_subscription(
            Float32MultiArray,
            '/kinova/vision/detected_pose',
            self.vision_callback,
            10
        )

        self.get_logger().info("Pose Calculator Node Initialized")

        # Timer for FK Debug
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        if self.current_joint_angles is None:
            return

        if len(self.current_joint_angles) < 6:
            return

        try:
            T_base_camera = self.compute_fk(self.current_joint_angles)
            
            # Extract Translation
            x, y, z = T_base_camera[0:3, 3]

            # Extract Rotation (RPY)
            rotation = R.from_matrix(T_base_camera[0:3, 0:3])
            r, p, y_angle = rotation.as_euler('xyz', degrees=True)

            print("\n" + "="*50)
            print("[DEBUG] Base -> Camera FK (No Marker Needed)")
            print(f"x: {x:.4f} m")
            print(f"y: {y:.4f} m")
            print(f"z: {z:.4f} m")
            print(f"r: {r:.4f} deg")
            print(f"p: {p:.4f} deg")
            print(f"y: {y_angle:.4f} deg")
            print("="*50)
            
        except Exception as e:
            self.get_logger().error(f"FK Error: {e}")

    def joint_callback(self, msg):
        self.current_joint_angles = np.radians(np.array(msg.position))

    def dh_transform(self, theta, d, a, alpha):
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d],
            [0,   0,      0,     1]
        ])

    def compute_fk(self, joints):
        # 1: theta_1, 243.3, 0.0, pi/2
        T_0_1 = self.dh_transform(joints[0], 0.2433, 0.0, math.pi/2)

        # 2: theta_2 + pi/2, 30.0, 280.0, pi
        T_1_2 = self.dh_transform(joints[1] + math.pi/2, 0.030, 0.280, math.pi)

        # 3: theta_3 + pi/2, 20.0, 0.0, pi/2
        T_2_3 = self.dh_transform(joints[2] + math.pi/2, 0.020, 0.0, math.pi/2)

        # 4: theta_4 + pi/2, 245.0, 0.0, pi/2
        T_3_4 = self.dh_transform(joints[3] + math.pi/2, 0.245, 0.0, math.pi/2)

        # 5: theta_5 + pi, 57.0, 0.0, pi/2
        T_4_5 = self.dh_transform(joints[4] + math.pi, 0.057, 0.0, math.pi/2)

        # 6: (공식 DH 기준) theta_6 + pi/2, d6=0, a6=0, alpha6=0
        T_5_6 = self.dh_transform(joints[5] + math.pi/2, 0.0, 0.0, 0.0)

        T_base_6 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6

        T_6_c1 = self.dh_transform(0.0, 0.033636, 0.0, 0.0)

        T_c1_c2 = self.dh_transform(0.0, 0.0, -0.066577, 0.0)

        T_c2_c = self.dh_transform(-math.pi/2, 0.0, -0.0325, 0.0)

        T_6_camera = T_6_c1 @ T_c1_c2 @ T_c2_c

        T_optical_correction = np.eye(4)
        T_base_camera = T_base_6 @ T_6_camera @ T_optical_correction

        return T_base_camera


    def vision_callback(self, msg):
        if self.current_joint_angles is None:
            self.get_logger().warn("No joint angles received yet.")
            return

        if len(self.current_joint_angles) < 6:
            self.get_logger().warn(f"Not enough joint angles. Expected at least 6, got {len(self.current_joint_angles)}")
            return

        # 1. 카메라-마커 변환 가져오기
        T_camera_marker = np.array(msg.data).reshape(4, 4)

        # 2. 베이스-카메라 변환 계산 (FK)
        try:
            T_base_camera = self.compute_fk(self.current_joint_angles)
        except Exception as e:
            self.get_logger().error(f"FK Computation Error: {e}")
            return

        # 3. 베이스-마커 변환 계산
        T_base_marker = T_base_camera @ T_camera_marker

        # 4. 결과 출력
        print("\n" + "="*50)
        print("Base to Aruco Marker Pose Calculation")
        print("="*50)
        
        print("\n[1] 4x4 Transformation Matrix (Base -> Marker):")
        print(T_base_marker)

        # 이동(Translation) 추출
        x, y, z = T_base_marker[0:3, 3]

        # 회전(Rotation) 추출 (RPY)
        rotation = R.from_matrix(T_base_marker[0:3, 0:3])
        r, p, y_angle = rotation.as_euler('xyz', degrees=True)

        print("\n[2] Pose (x, y, z, r, p, y):")
        print(f"x: {x:.4f} m")
        print(f"y: {y:.4f} m")
        print(f"z: {z:.4f} m")
        print(f"r: {r:.4f} deg")
        print(f"p: {p:.4f} deg")
        print(f"y: {y_angle:.4f} deg")
        print("="*50 + "\n")

def main(args=None):
    rclpy.init(args=args)
    node = PoseCalculatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
