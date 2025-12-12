#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32, Float32MultiArray, Int32, Int32MultiArray
from kinova_msgs.msg import KinovaCommand
from sensor_msgs.msg import JointState
from final_kinova.module import *
import threading
import time, math

import numpy as np

class KinovaMainNode(Node):
    def __init__(self):
        super().__init__('kinova_main_node')
        self.get_logger().info('Kinova Main Node Initialized')
        
        self.lock = threading.Lock()
        
        # 퍼블리셔
        self.cmd_pub = self.create_publisher(KinovaCommand, '/kinova/client_command', 10)
        self.grip_pub = self.create_publisher(Float32, '/kinova/float/grp_cmd', 10)

        self.obj_pub = self.create_publisher(String, '/kinova/string/obj_command', 10)
        self.id_pub = self.create_publisher(Int32, '/kinova/int/maker_command', 10)

        # 서브스크라이버
        self.result_sub = self.create_subscription(Bool, '/kinova/service_result', self.result_callback, 10)
        self.action_sub = self.create_subscription(String, '/kinova/string/action_def', self.action_callback, 10)
        self.grip_sub   = self.create_subscription(Bool, '/kinova/bool/grp_done', self.grip_callback, 10)
        
        # vision subscription
        self.maker_sub = self.create_subscription(Float32MultiArray, '/kinova/array/maker_pose', self.maker_callback, 10)
        self.obj_sub = self.create_subscription(Float32MultiArray, '/kinova/array/obj_pose', self.obj_callback, 10)
        
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
            self.id_pose = None
            self.detected_maker_pose = None
            self.detected_obj_pose = None
            self.requested_ids = []
            self.next_block = None
            self.last_handler_name = None
            self.last_step = None
            self.last_flag = None
            self.last_next_step = None
            self.handler = None
            self.current_step = None
            self.next_step = None
            self.current_flag = 'idle'

    def pose_init(self):
        with self.lock:
            self.detected_maker_pose = None
            self.detected_obj_pose = None
    
    def maker_callback(self, msg):
        with self.lock:
            data = np.array(msg.data)
            if len(data) != 4:
                self.get_logger().warn(f"Received data size {len(data)}, expected 3 (Base Frame XYZ)")
                return

            # Store the pose directly
            self.detected_maker_pose = data
            
            print("\n" + "="*50)
            print(f"Received ArUco Marker Base Position")
            print("="*50)
            print(f"Position: x={data[0]:.4f}, y={data[1]:.4f}, z={data[2]:.4f}")
            print("="*50 + "\n")
            
            if self.current_flag == 'waiting':
                self.current_flag = 'done'

    def obj_callback(self, msg):
        with self.lock:
            data = np.array(msg.data)
            if len(data) != 4:
                 self.get_logger().warn(f"Received ID 4 floats, got {len(data)}")
                 return

            self.detected_obj_pose = data
            
            print("\n" + "="*50)
            print(f"Received Object Base Pose")
            print("="*50)
            print(f"Pose: x={data[0]:.4f}, y={data[1]:.4f}, z={data[2]:.4f}, yaw={data[3]:.4f}")
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
            
            if command == 'FINAL':
                self.handler = Final()
                self.current_step = 'step_1'
                self.current_flag = 'order'
                
            elif command == 'HEX':
                self.handler = Final_Hex()
                self.current_step = 'step_1'
                self.current_flag = 'order'
                
            elif command == 'SQU':
                self.handler = Final_Squ()
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
                self.param_init()
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
            if ans['camera_trigger'] == 'obj':
                cam_msg = String()
                cam_msg.data = ans['camera_target']
                
                self.obj_pub.publish(cam_msg)
                self.get_logger().info(f"[MAIN] Published Object Trigger: {cam_msg.data}")
                
                
            elif ans['camera_trigger'] == 'maker':
                cam_msg = Int32()
                cam_msg.data = ans['camera_target']
                
                self.id_pub.publish(cam_msg)
                self.get_logger().info(f"[MAIN] Published Maker Trigger: {cam_msg.data}")
        
        ### block block block block block block block block block 
        # Block 중앙으로 이동하는 기능
        if ans.get('obj_move_top') is not None:
            with self.lock:
                obj_pose = self.detected_obj_pose
                
            x, y = float(obj_pose[0]), float(obj_pose[1])
            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            cmd_msg.coordinate = [x-0.04, y, 0.18, 179.0, 0.0, 90.0]
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[MAIN] Moving Above Object: {cmd_msg.coordinate}")
            
        # Block 집으러 내려가는 기능
        if ans.get('obj_move_pick') is not None:
            with self.lock:
                obj_pose = self.detected_obj_pose
                
            x, y, yaw = float(obj_pose[0]), float(obj_pose[1]), float(obj_pose[3])
            if ans.get('im_hex') and yaw < 0:
                yaw = yaw + 60.0
            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            cmd_msg.coordinate = [x, y, 0.028, 179.0, 0.0, 90.0 + yaw]
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[MAIN] Moving Above Object: {cmd_msg.coordinate}")
            
        if ans.get('obj_move_up') is not None:
            with self.lock:
                obj_pose = self.detected_obj_pose
                
            x, y = float(obj_pose[0]), float(obj_pose[1])
            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            cmd_msg.coordinate = [x, y, 0.15, 179.0, 0.0, 90.0]
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[MAIN] Moving Above Object: {cmd_msg.coordinate}")
                
        ### block block block block block block block block block 
        
        ### maker maker maker maker maker maker maker maker maker
        if ans.get('maker_move_top') is not None:
            with self.lock:
                maker_pose = self.detected_maker_pose
                
            x, y = float(maker_pose[0]), float(maker_pose[1])
            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            cmd_msg.coordinate = [x-0.04-0.045, y, 0.14, 179.0, 0.0, 90.0]
            
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[MAIN] Moving Above Maker: {cmd_msg.coordinate}")
            
        if ans.get('maker_move_drop_top') is not None:
            with self.lock:
                maker_pose = self.detected_maker_pose
                
            x, y, yaw = float(maker_pose[0]), float(maker_pose[1]), float(maker_pose[3])
            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            if ans['hex_offset']:
                cmd_msg.coordinate = [x, y, 0.14, 179.0, 0.0, 90.0 + yaw]
            else:
                cmd_msg.coordinate = [x, y, 0.14, 179.0, 0.0, 90.0 + yaw]
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[MAIN] Moving Above Maker: {cmd_msg.coordinate}")
        
        if ans.get('maker_move_drop') is not None:
            with self.lock:
                maker_pose = self.detected_maker_pose
                
            x, y, yaw = float(maker_pose[0]), float(maker_pose[1]), float(maker_pose[3])
            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            if ans['hex_offset']:
                cmd_msg.coordinate = [x, y, 0.083, 179.0, 0.0, 120.0 + yaw]
            else:
                cmd_msg.coordinate = [x, y, 0.087, 179.0, 0.0, 90.0 + yaw]
            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[MAIN] Moving Above Maker: {cmd_msg.coordinate}")
            
        if ans.get('maker_move_turn') is not None:
            with self.lock:
                maker_pose = self.detected_maker_pose
                
            x, y, yaw = float(maker_pose[0]), float(maker_pose[1]), float(maker_pose[3])
            cmd_msg = KinovaCommand()
            cmd_msg.frame = 'cartesian'
            cmd_msg.coordinate = [x, y, 0.14, 179.0, 0.0, 120.0 + yaw]

            self.cmd_pub.publish(cmd_msg)
            
            self.get_logger().info(f"[MAIN] Moving Above Maker: {cmd_msg.coordinate}")
        
        ### maker maker maker maker maker maker maker maker maker
        

        if ans.get('clear_param') is not None:
            self.param_init()
            self.get_logger().info("[MAIN] Parameters cleared")


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
