#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from kinova_msgs.srv import KinovaExecutor
from kinova_msgs.msg import KinovaCommand
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import sys

class KinovaClientNode(Node):
    def __init__(self):
        super().__init__('kinova_client_node')
        self.cli = self.create_client(KinovaExecutor, '/kinova/execute')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.sub = self.create_subscription(
            KinovaCommand,
            '/kinova/client_command',
            self.topic_callback,
            10
        )
        self.get_logger().info('Kinova Client Node Initialized. Waiting for commands on /kinova/client_command...')
        
        self.result_pub = self.create_publisher(Bool, '/kinova/service_result', 10)

    def topic_callback(self, msg):
        self.get_logger().info(f"Received Topic Command: {msg.frame}, {msg.coordinate}")
        self.send_request(msg.frame, msg.coordinate)

    def send_request(self, frame, coordinate):
        req = KinovaExecutor.Request()
        req.frame = frame
        req.coordinate = coordinate
        
        future = self.cli.call_async(req)
        future.add_done_callback(self.response_callback)
        
    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Service Result: {response.success}, Message: {response.message}")
            
            res_msg = Bool()
            res_msg.data = response.success
            self.result_pub.publish(res_msg)
            
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            res_msg = Bool()
            res_msg.data = False
            self.result_pub.publish(res_msg)

def main(args=None):
    rclpy.init(args=args)
    client = KinovaClientNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(client)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
