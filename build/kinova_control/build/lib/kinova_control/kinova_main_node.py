#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Float32MultiArray, Int32, Int32MultiArray
from kinova_msgs.msg import KinovaCommand
from sensor_msgs.msg import JointState
from kinova_control.module import *
import threading
import time, math
from scipy.spatial.transform import Rotation as R

import numpy as np

MAKER_OFFSET = 0.04

class KinovaMainNode(Node):
    def __init__(self):
        super().__init__('kinova_main_node')
        self.get_logger().info('Kinova Main Node Initialized')
        
        self.lock = threading.Lock()
        
        # 퍼블리셔
        self.cmd_pub = self.create_publisher(KinovaCommand, '/kinova/client_command', 10)
        self.grip_pub = self.create_publisher(Float32, '/kinova/float/grp_cmd', 10)

        self.id_pub = self.create_publisher(Int32MultiArray, '/kinova/array/camera_trigger', 10)

        # 서브스크라이버
        self.result_sub = self.create_subscription(Bool, '/kinova/service_result', self.result_callback, 10)
        self.action_sub = self.create_subscription(String, '/kinova/string/action_def', self.action_callback, 10)
        self.grip_sub   = self.create_subscription(Bool, '/kinova/bool/grp_done', self.grip_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/kinova/joint_states', self.joint_callback, 10)
        self.vision_sub = self.create_subscription(Float32MultiArray, '/kinova/array/detected_pose', self.vision_callback, 10)
        
        self.current_joint_angles = None
        
        # FSM 변수
        self.handler = None
        self.current_step = None
        self.next_step = None
        self.current_flag = 'idle'
        
        # 디버그 변수
        self.last_handler_name = None
        self.last_step = None
        self.last_flag = None
        self.last_next_step = None
        
        self.param_init()
        
        # 타이머
        self.timer = self.create_timer(0.05, self.loop)

    def param_init(self):
        with self.lock:
            self.id_len = None
            self.id_pose = None
            self.detected_poses = {}
            self.requested_ids = []
            self.next_block = None
        
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

    def joint_callback(self, msg):
        with self.lock:
            self.current_joint_angles = np.radians(np.array(msg.position))
            
    def compute_fk(self, joints):
        T_0_1 = self.dh_transform(joints[0], 0.2433, 0.0, math.pi/2)
        T_1_2 = self.dh_transform(joints[1] + math.pi/2, 0.030, 0.280, math.pi)
        T_2_3 = self.dh_transform(joints[2] + math.pi/2, 0.020, 0.0, math.pi/2)
        T_3_4 = self.dh_transform(joints[3] + math.pi/2, 0.245, 0.0, math.pi/2)
        T_4_5 = self.dh_transform(joints[4] + math.pi, 0.057, 0.0, math.pi/2)
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
        
        else:
            with self.lock:
                data = np.array(msg.data)
                expected_size = len(self.requested_ids) * 16
                if len(data) != expected_size:
                    self.get_logger().warn(f"Received data size {len(data)} does not match requested IDs count {len(self.requested_ids)} * 16")
                    return

                num_poses = len(self.requested_ids)
                
                if num_poses > 0:
                    self.detected_poses = {}

                    try:
                        T_base_camera = self.compute_fk(self.current_joint_angles)
                    except Exception as e:
                        self.get_logger().error(f"FK Computation Error: {e}")
                        return

                    for i, marker_id in enumerate(self.requested_ids):
                        pose_data = data[i*16 : (i+1)*16]
                        T_camera_marker = pose_data.reshape(4, 4)

                        T_base_marker = T_base_camera @ T_camera_marker
                        self.detected_poses[marker_id] = T_base_marker
                        
                        print("\n" + "="*50)
                        print(f"Base to Aruco Marker Pose Calculation (ID: {marker_id})")
                        print("="*50)
                        print(T_base_marker)
                        
                        P = T_base_marker[0:3, 3]
                        rotation = R.from_matrix(T_base_marker[0:3, 0:3])
                        r, p, y_angle = rotation.as_euler('xyz', degrees=True)
                        
                        print(f"Pose: x={P[0]:.4f}, y={P[1]:.4f}, z={P[2]:.4f}, r={r:.4f}, p={p:.4f}, y={y_angle:.4f}")
                        print("="*50 + "\n")

                if self.current_flag == 'waiting':
                        self.current_flag = 'done'

    def action_callback(self, msg):
        with self.lock:
            if self.current_flag != 'idle':
                self.get_logger().warn(f'[MAIN] Busy ({self.current_flag}). Ignoring command: {msg.data}')
                return

            command = msg.data
            self.get_logger().info(f'[MAIN] Received Action Command: {command}')
            
            if command == 'MIDTERM_ONE':
                self.handler = Midterm_1()
                self.current_step = 'step_1'
                self.current_flag = 'order'

            elif command == 'MIDTERM_TWO':
                self.handler = Midterm_2()
                self.current_step = 'step_1'
                self.current_flag = 'order'

            else:
                self.get_logger().warn(f'[MAIN] Unknown Command: {command}')

    def grip_callback(self, msg):
        with self.lock:

            if self.current_flag != 'idle' and self.current_flag != 'waiting':
                self.get_logger().warn(f'[MAIN] Busy ({self.current_flag}). Ignoring command: {msg.data}')
                return

            if msg.data:
                if self.current_flag == 'waiting':
                    self.current_flag = 'done'
                    self.get_logger().info('[MAIN] Gripper Completed Successfully')
            else:
                self.get_logger().warn('[MAIN] Gripper Failed')
                self.current_flag = 'fail'

    def result_callback(self, msg):
        with self.lock:
            if msg.data:
                if self.current_flag == 'waiting':
                    self.current_flag = 'done'
                    self.get_logger().info('[MAIN] Action Completed Successfully')
            else:
                self.get_logger().warn('[MAIN] Action Failed')
                self.current_flag = 'fail'

    def loop(self):
        with self.lock:
            handler = self.handler
            current_step = self.current_step
            current_flag = self.current_flag
            next_step = self.next_step
            
        self.check_state_change(handler, current_step, current_flag, next_step)

        if current_flag == 'idle':
            return

        if handler is None or current_step is None or current_step == 'None':
            with self.lock:
                self.current_flag = 'idle'
                self.handler = None
            return

        if current_flag == 'order':
            ans = handler.step(current_step)
            
            if ans is None:
                self.get_logger().warn(f'[MAIN] Wrong Step : {current_step}')
                with self.lock:
                    self.current_step = None
                return

            self.get_logger().info(f'[MAIN] Executing Step: {current_step}')
            
            self.module_translator(ans)
            
            requires_ack = ans.get('requires_ack', True)
            
            with self.lock:
                if requires_ack:
                    self.next_step = ans.get('next_step')
                    self.current_flag = 'waiting'
                else:
                    self.current_step = ans.get('next_step')
                    self.current_flag = 'order'
                    
        elif current_flag == 'waiting':
            pass
            
        elif current_flag == 'done':
            with self.lock:
                if next_step is None or next_step == 'None':
                    self.get_logger().info('[MAIN] Module Finished')
                    self.current_flag = 'idle'
                    self.handler = None
                    self.current_step = None
                else:
                    self.current_step = next_step
                    self.current_flag = 'order'
                    self.next_step = None
                
        elif current_flag == 'fail':
            self.get_logger().error('[MAIN] Stopped due to failure')
            with self.lock:
                self.current_flag = 'idle'
                self.handler = None
                self.current_step = None

    def module_translator(self, ans):
        if ans.get('position'):
            cmd_msg = KinovaCommand()
            cmd_msg.frame = ans['frame']
            cmd_msg.coordinate = ans['position']
            
            self.cmd_pub.publish(cmd_msg)
            self.get_logger().info(f"[MAIN] Published Command: {cmd_msg.frame}, {cmd_msg.coordinate}")

        if ans.get('gripper') is not None:
            grip_cmd = Float32()
            grip_cmd.data = float(ans['gripper'])

            self.grip_pub.publish(grip_cmd)
            self.get_logger().info(f"[MAIN] Published Grip Command: {grip_cmd.data}")
            
        if ans.get('camera_trigger') is not None:
            id_cmd = Int32MultiArray()
            req_id = ans['camera_trigger']
            
            with self.lock:
                self.requested_ids = req_id if isinstance(req_id, list) else [req_id]
                
            id_cmd.data = self.requested_ids
            self.id_pub.publish(id_cmd)
            self.get_logger().info(f"[MAIN] Published Camera Trigger IDs: {id_cmd.data}")


        if ans.get('move_pick_top'):
            with self.lock:
                offset = ans['idx_offset']
                id_pose = self.detected_poses.get(self.requested_ids[0+ offset])
            
            if id_pose is None:
                self.get_logger().warn("[MAIN] No pose available for move_pick_top")
                return

            P = id_pose[:3, 3]

            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            cmd_msg.coordinate = [P[0] + MAKER_OFFSET, P[1], 0.13, 179.0, 0.0, 90.0]

            self.cmd_pub.publish(cmd_msg)
            self.get_logger().info(f"[MAIN] Published Command: {cmd_msg.frame}, {cmd_msg.coordinate}")

        if ans.get('move_pick'):
            with self.lock:
                offset = ans['idx_offset']
                id_pose = self.detected_poses.get(self.requested_ids[0 + offset])
            
            if id_pose is None:
                self.get_logger().warn("[MAIN] No pose available for move_pick_top")
                return

            P = id_pose[:3, 3]

            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            cmd_msg.coordinate = [P[0] + MAKER_OFFSET, P[1], 0.030, 179.0, 0.0, 90.0]

            self.cmd_pub.publish(cmd_msg)
            self.get_logger().info(f"[MAIN] Published Command: {cmd_msg.frame}, {cmd_msg.coordinate}")

        if ans.get('move_pick_after'):
            with self.lock:
                offset = ans['idx_offset']
                id_pose = self.detected_poses.get(self.requested_ids[0 + offset])
            
            if id_pose is None:
                self.get_logger().warn("[MAIN] No pose available for move_pick_top")
                return

            P = id_pose[:3, 3]

            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            cmd_msg.coordinate = [P[0] + MAKER_OFFSET, P[1], 0.13, 179.0, 0.0, 90.0]

            self.cmd_pub.publish(cmd_msg)
            self.get_logger().info(f"[MAIN] Published Command: {cmd_msg.frame}, {cmd_msg.coordinate}")
        
        if ans.get('move_drop_top'):
            with self.lock:
                offset = ans['idx_offset']
                id_pose = self.detected_poses.get(self.requested_ids[1 + offset])
            
            if id_pose is None:
                self.get_logger().warn("[MAIN] No pose available for move_pick_top")
                return

            P = id_pose[:3, 3]

            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            cmd_msg.coordinate = [P[0] + MAKER_OFFSET, P[1], 0.13, 179.0, 0.0, 90.0]

            self.cmd_pub.publish(cmd_msg)
            self.get_logger().info(f"[MAIN] Published Command: {cmd_msg.frame}, {cmd_msg.coordinate}")

        if ans.get('move_drop'):
            with self.lock:
                offset = ans['idx_offset']
                id_pose = self.detected_poses.get(self.requested_ids[1 + offset])
            
            if id_pose is None:
                self.get_logger().warn("[MAIN] No pose available for move_pick_top")
                return

            P = id_pose[:3, 3]

            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            cmd_msg.coordinate = [P[0] + MAKER_OFFSET, P[1], 0.034, 179.0, 0.0, 90.0]

            self.cmd_pub.publish(cmd_msg)
            self.get_logger().info(f"[MAIN] Published Command: {cmd_msg.frame}, {cmd_msg.coordinate}")
            
        if ans.get('move_drop_top'):
            with self.lock:
                offset = ans['idx_offset']
                id_pose = self.detected_poses.get(self.requested_ids[1 + offset])
            
            if id_pose is None:
                self.get_logger().warn("[MAIN] No pose available for move_pick_top")
                return

            P = id_pose[:3, 3]

            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            cmd_msg.coordinate = [P[0] + MAKER_OFFSET, P[1], 0.13, 179.0, 0.0, 90.0]

            self.cmd_pub.publish(cmd_msg)
            self.get_logger().info(f"[MAIN] Published Command: {cmd_msg.frame}, {cmd_msg.coordinate}")


    def check_state_change(self, handler, step, flag, next_step):
        handler_name = handler.__class__.__name__ if handler else "None"
        
        if (handler_name != self.last_handler_name or 
            step != self.last_step or 
            flag != self.last_flag or 
            next_step != self.last_next_step):
            
            print(f"\n[DEBUG] State Changed:")
            print(f"  Module   : {handler_name}")
            print(f"  Step     : {step}")
            print(f"  Flag     : {flag}")
            print(f"  Next Step: {next_step}")
            print("-" * 30)
            
            self.last_handler_name = handler_name
            self.last_step = step
            self.last_flag = flag
            self.last_next_step = next_step

def main(args=None):
    rclpy.init(args=args)
    node = KinovaMainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
