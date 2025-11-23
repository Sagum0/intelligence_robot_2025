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

        # Storage for latest joint angles
        self.current_joint_angles = None

        # Subscribers
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

    def joint_callback(self, msg):
        # Store joint angles (assuming they are in radians and order matches DH)
        # Gen3 joints are usually named joint_1, joint_2, ...
        # We assume the list is ordered correctly.
        # If the driver publishes in degrees, we need conversion. 
        # The kortex_driver usually publishes in degrees for the Action, but JointState from our driver?
        # Let's check kinova_driver_node.py. 
        # It gets feedback.actuators[i].position. Kortex API usually returns degrees.
        # JointState standard is radians. 
        # But our driver just copies actuator.position.
        # Let's assume degrees for now based on typical Kortex usage, but I should verify.
        # Wait, standard ROS JointState is radians. 
        # If our driver publishes degrees, we should convert to radians for math.
        # Let's assume the driver publishes what the API gives. 
        # Kortex API positions are in degrees.
        # So I will convert to radians here.
        
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
        # DH Parameters (from user)
        # theta, d, a, alpha
        # Units: d, a in mm -> convert to m
        # theta in radians
        
        # 1: theta_1, 243.3, 0.0, pi/2
        T_0_1 = self.dh_transform(joints[0], 0.2433, 0.0, math.pi/2)
        
        # 2: theta_2, 30.0, 280.0, pi
        T_1_2 = self.dh_transform(joints[1], 0.030, 0.280, math.pi)
        
        # 3: theta_3, 20.0, 0.0, pi/2
        T_2_3 = self.dh_transform(joints[2], 0.020, 0.0, math.pi/2)
        
        # 4: theta_4, 245.0, 0.0, pi/2
        T_3_4 = self.dh_transform(joints[3], 0.245, 0.0, math.pi/2)
        
        # 5: theta_5 + pi, 0.0, 101.405, pi
        T_4_5 = self.dh_transform(joints[4] + math.pi, 0.0, 0.101405, math.pi)
        
        # c1: 0.0, 26.885, 0, -pi/2
        T_5_c1 = self.dh_transform(0.0, 0.026885, 0.0, -math.pi/2)
        
        # c: -pi/2, 50.856, 0, -pi/2
        T_c1_c = self.dh_transform(-math.pi/2, 0.050856, 0.0, -math.pi/2)
        
        # Chain
        T_base_camera = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4 @ T_4_5 @ T_5_c1 @ T_c1_c
        return T_base_camera

    def vision_callback(self, msg):
        if self.current_joint_angles is None:
            self.get_logger().warn("No joint angles received yet.")
            return

        if len(self.current_joint_angles) < 5:
            self.get_logger().warn(f"Not enough joint angles. Expected at least 5, got {len(self.current_joint_angles)}")
            return

        # 1. Get Camera-to-Marker Transform
        T_camera_marker = np.array(msg.data).reshape(4, 4)

        # 2. Compute Base-to-Camera Transform (FK)
        try:
            T_base_camera = self.compute_fk(self.current_joint_angles)
        except Exception as e:
            self.get_logger().error(f"FK Computation Error: {e}")
            return

        # 3. Compute Base-to-Marker Transform
        T_base_marker = T_base_camera @ T_camera_marker

        # 4. Output Results
        print("\n" + "="*50)
        print("Base to Aruco Marker Pose Calculation")
        print("="*50)
        
        print("\n[1] 4x4 Transformation Matrix (Base -> Marker):")
        print(T_base_marker)

        # Extract Translation
        x, y, z = T_base_marker[0:3, 3]

        # Extract Rotation (RPY)
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
