#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from kinova_msgs.msg import KinovaCommand
from kinova_control.module import *
import threading
import time

class KinovaMainNode(Node):
    def __init__(self):
        super().__init__('kinova_main_node')
        self.get_logger().info('Kinova Main Node Initialized')
        
        self.lock = threading.Lock()
        
        # Publishers
        self.cmd_pub = self.create_publisher(KinovaCommand, '/kinova/client_command', 10)
        
        # Subscribers
        self.result_sub = self.create_subscription(Bool, '/kinova/service_result', self.result_callback, 10)
        self.action_sub = self.create_subscription(String, '/kinova/string/action_def', self.action_callback, 10)
        
        # FSM Variables
        self.handler = None
        self.current_step = None
        self.next_step = None
        self.current_flag = 'idle'
        
        # Debug Variables
        self.last_handler_name = None
        self.last_step = None
        self.last_flag = None
        self.last_next_step = None
        
        # Timer
        self.timer = self.create_timer(0.05, self.loop) # 20Hz

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

    def result_callback(self, msg):
        with self.lock:
            if msg.data: # Success
                if self.current_flag == 'waiting':
                    self.current_flag = 'done'
                    self.get_logger().info('[MAIN] Action Completed Successfully')
            else: # Fail
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
            
            # Module Translator
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
            pass # Wait for result_callback
            
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
            # Handle failure (retry or stop)
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
            # Speed is not in KinovaCommand yet, ignoring for now or could extend message
            
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
