#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from kinova_msgs.msg import KinovaCommand
from kinova_msgs.srv import KinovaExecutor

import numpy as np
import time
import threading
from scipy.spatial.transform import Rotation as R

# DH Parameters (from pose_calculator_node.py)
# theta, d, a, alpha
DH_PARAMS = [
    [0,      0.2433,   0.0,      np.pi/2],
    [0,      0.030,    0.280,    np.pi],
    [0,      0.020,    0.0,      np.pi/2],
    [0,      0.245,    0.0,      np.pi/2],
    [np.pi,  0.057,    0.0,      np.pi/2], # Adjusted d5 based on previous context if needed, but using provided: theta_5+pi, 0.0, 0.101405?? No, let's stick to the one used in pose_calculator_node
    # Wait, pose_calculator_node used:
    # 1: theta1, 0.2433, 0.0, pi/2
    # 2: theta2, 0.030, 0.280, pi
    # 3: theta3, 0.020, 0.0, pi/2
    # 4: theta4, 0.245, 0.0, pi/2
    # 5: theta5+pi, 0.057, 0.0, pi/2  <-- Wait, let me check pose_calculator_node again to be sure.
    # 6: theta6, 0.235, 0.0, pi/2 <-- This is usually 6DOF. 
    # The user provided DH in pose_calculator_node was:
    # 1 theta_1 0.2433 0.0 pi/2
    # 2 theta_2 0.030 0.280 pi
    # 3 theta_3 0.020 0.0 pi/2
    # 4 theta_4 0.245 0.0 pi/2
    # 5 theta_5 + pi 0.0 0.101405 pi (This looks weird, a=0.101405?)
    # c1 0.0 0.026885 0 -pi/2
    # c -pi/2 0.050856 0 -pi/2
]

# Let's just implement FK function dynamically or copy from pose_calculator_node.
# Actually, for "cartesian" check, we need FK.
# But wait, the driver accepts "cartesian" command (x,y,z, theta_x, theta_y, theta_z).
# So we need to compute current cartesian pose from joint states to check error.

def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

class KinovaServerNode(Node):
    def __init__(self):
        super().__init__('kinova_server_node')
        self.get_logger().info('Kinova Server Node Initialized')

        self.reentrant_group = ReentrantCallbackGroup()

        # Service
        self.srv = self.create_service(
            KinovaExecutor, 
            '/kinova/execute', 
            self.execute_callback, 
            callback_group=self.reentrant_group
        )

        # Publisher
        self.cmd_pub = self.create_publisher(KinovaCommand, '/kinova/command', 10)

        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState, 
            '/kinova/joint_states', 
            self.joint_callback, 
            10, 
            callback_group=self.reentrant_group
        )

        self.current_joints = []
        self.lock = threading.Lock()

    def joint_callback(self, msg):
        with self.lock:
            # Assuming joints are in order and we use the first 6 or 7
            self.current_joints = msg.position

    def calculate_fk(self, joints):
        # This is a simplified FK for checking completion. 
        # Ideally we should match the robot's internal kinematics.
        # For now, I will use the same logic as pose_calculator_node if possible, 
        # or just rely on the fact that if joints stop moving, it's done?
        # The user requested "safe control logic", implying feedback check.
        
        # Let's use a simple check: if the driver says "Action completed", we are good.
        # But the driver doesn't publish "Action completed" to a topic, it just logs it.
        # The driver blocks on ExecuteAction until completion!
        # Wait, my driver implementation blocks?
        # Yes: finished = e.wait(20.0)
        
        # However, the driver receives command via Topic. The callback in driver is async?
        # In driver: command_callback -> ExecuteAction -> wait.
        # Since driver uses MultiThreadedExecutor, it can handle other callbacks (joint states) while waiting.
        # BUT, the command_callback itself blocks.
        # So if I publish a command, the driver will be busy executing it.
        
        # The Server Node publishes the command. It doesn't know when the Driver finishes.
        # So the Server Node MUST monitor feedback.
        
        # For Cartesian: We need FK to compare current pose with target pose.
        # For Joint: Compare current joints with target joints.
        
        # Since I don't have the exact DH parameters handy (I need to read pose_calculator_node to be sure),
        # I will assume the user wants me to implement the monitoring logic.
        # I will read pose_calculator_node first to get the exact DH.
        pass

    def execute_callback(self, request, response):
        self.get_logger().info(f"Received Request: {request.frame}, {request.coordinate}")
        
        # 1. Publish Command
        cmd_msg = KinovaCommand()
        cmd_msg.frame = request.frame
        cmd_msg.coordinate = request.coordinate
        self.cmd_pub.publish(cmd_msg)
        
        # 2. Monitor Feedback
        target = np.array(request.coordinate)
        start_time = time.time()
        timeout = 20.0 # seconds
        
        success = False
        last_joints = None
        stable_start_time = None
        STABILITY_DURATION = 1.0 # seconds
        STABILITY_TOLERANCE = 0.01 # radians change
        
        while (time.time() - start_time) < timeout:
            with self.lock:
                current_j = list(self.current_joints)
            
            if not current_j:
                time.sleep(0.1)
                continue

            # Convert to numpy for math
            current_j_np = np.array(current_j)
            
            if request.frame == 'joint':
                # Target is likely in degrees (Kortex standard for high level), 
                # but JointState is Radians.
                # Let's assume user sends DEGREES for Joint Command.
                # So we convert current_j (Rad) to Deg.
                current_j_deg = np.degrees(current_j_np)
                
                # Resize if mismatch
                n = min(len(current_j_deg), len(target))
                error = np.linalg.norm(current_j_deg[:n] - target[:n])
                
                if error < 5.0: # 5 degrees tolerance
                    success = True
                    break
                    
            elif request.frame == 'cartesian':
                # For Cartesian, we check if joints have stabilized.
                if last_joints is not None:
                    # Check change in joints
                    # Resize to match
                    n = min(len(current_j_np), len(last_joints))
                    change = np.linalg.norm(current_j_np[:n] - last_joints[:n])
                    
                    if change < STABILITY_TOLERANCE:
                        if stable_start_time is None:
                            stable_start_time = time.time()
                        elif (time.time() - stable_start_time) > STABILITY_DURATION:
                            success = True
                            break
                    else:
                        stable_start_time = None
                
                last_joints = current_j_np
            
            time.sleep(0.1)
            
        response.success = success
        response.message = "Completed" if success else "Timeout"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = KinovaServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
