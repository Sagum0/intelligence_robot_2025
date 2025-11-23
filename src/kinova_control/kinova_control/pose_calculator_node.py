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

        # 최신 관절 각도 저장
        self.current_joint_angles = None

        # 서브스크라이버
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
        # 관절 각도 저장 (라디안 단위, DH 순서와 일치한다고 가정)
        # Gen3 관절은 보통 joint_1, joint_2, ... 로 명명됨
        # 리스트 순서가 올바르다고 가정함.
        # 드라이버가 도(degree) 단위로 발행한다면 변환이 필요함.
        # kortex_driver는 보통 Action에 대해 도 단위를 사용하지만, 우리 드라이버의 JointState는?
        # kinova_driver_node.py를 확인해보면 feedback.actuators[i].position을 가져옴. Kortex API는 보통 도 단위를 반환함.
        # JointState 표준은 라디안임.
        # 하지만 우리 드라이버는 actuator.position을 그대로 복사함.
        # 일반적인 Kortex 사용법에 따라 도 단위라고 가정하지만 확인이 필요함.
        # 잠시만, 표준 ROS JointState는 라디안임.
        # 만약 우리 드라이버가 도 단위를 발행한다면 수학 계산을 위해 라디안으로 변환해야 함.
        # 드라이버가 API가 주는 값을 그대로 발행한다고 가정함.
        # Kortex API 위치는 도 단위임.
        # 따라서 여기서 라디안으로 변환함.
        
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
        """
        Base -> Camera FK
        - 0~6 : Gen3 Lite 공식 DH (6번은 q6 + pi/2 사용)
        - 6 -> C1 -> C2 -> C : 카메라 마운트 고정 변환
        """

        # -------------------------------
        # 1. Base -> Joint6 (Flange)  DH
        # -------------------------------
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
        #    → 6번 조인트 축만 정의, 실제 마운트 길이는 아래 6->C1에서 처리
        T_5_6 = self.dh_transform(joints[5] + math.pi/2, 0.0, 0.0, 0.0)

        T_base_6 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_6

        # -------------------------------------
        # 2. Joint6 -> Camera Mount (고정변환)
        #    6 -> C1 -> C2 -> C
        # -------------------------------------

        # 6 -> C1 : z6 방향으로 +33.636mm 평행이동 (회전 없음)
        #  (그림에서 33.636 표시된 수직 거리)
        T_6_c1 = self.dh_transform(0.0, 0.033636, 0.0, 0.0)

        # C1 -> C2 : x_c1 방향으로 -66.577mm (그림의 66.577 길이, 부호는 사용자가 정한 축에 맞춤)
        T_c1_c2 = self.dh_transform(0.0, 0.0, -0.066577, 0.0)

        # C2 -> C : z_c2 축 기준 -90deg 회전, x_c2 방향으로 -32.5mm
        # (그림에서 마지막 수평 오프셋 32.5와 카메라 프레임의 회전 포함)
        T_c2_c = self.dh_transform(-math.pi/2, 0.0, -0.0325, 0.0)

        # 6에서 카메라까지
        T_6_camera = T_6_c1 @ T_c1_c2 @ T_c2_c

        # -------------------------------------
        # 3. (선택) 카메라 Optical Frame 보정
        # -------------------------------------
        # 현재는 물리 카메라 프레임 = 광학 프레임이라고 가정 (항등)
        T_optical_correction = np.eye(4)
        # 필요하면 예시처럼 조정
        # R_corr = R.from_euler('z', -90, degrees=True).as_matrix()
        # T_optical_correction[:3, :3] = R_corr

        # -------------------------------------
        # 4. 최종 Base -> Camera
        # -------------------------------------
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
