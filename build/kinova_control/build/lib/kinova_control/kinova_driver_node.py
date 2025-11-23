#!/usr/bin/env python3

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

# Custom Message
from kinova_msgs.msg import KinovaCommand

# Kortex API
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2

# Connection Helper
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2

class KinovaDriverNode(Node):
    def __init__(self):
        super().__init__('kinova_driver_node')

        # Parameters
        self.declare_parameter('ip_address', '192.168.1.10')
        self.declare_parameter('username', 'admin')
        self.declare_parameter('password', 'admin')

        self.ip_address = self.get_parameter('ip_address').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value

        # Kortex Connection
        self.transport = None
        self.router = None
        self.session_manager = None
        self.base = None
        self.base_cyclic = None

        self.connect_to_robot()

        # Callback Groups
        self.reentrant_group = ReentrantCallbackGroup()

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/kinova/joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.reentrant_group)

        # Subscribers
        self.subscription = self.create_subscription(
            KinovaCommand,
            '/kinova/command',
            self.command_callback,
            10,
            callback_group=self.reentrant_group
        )

        self.get_logger().info('Kinova Driver Node Initialized')

    def connect_to_robot(self):
        try:
            self.transport = TCPTransport()
            self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)
            self.transport.connect(self.ip_address, 10000)

            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.username
            session_info.password = self.password
            session_info.session_inactivity_timeout = 60000
            session_info.connection_inactivity_timeout = 2000

            self.session_manager = SessionManager(self.router)
            self.session_manager.CreateSession(session_info)

            self.base = BaseClient(self.router)
            self.base_cyclic = BaseCyclicClient(self.router)
            
            # Set servoing mode to Single Level Servoing
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)

            self.get_logger().info(f'Connected to robot at {self.ip_address}')

        except Exception as e:
            self.get_logger().error(f'Failed to connect to robot: {e}')
            sys.exit(1)

    def timer_callback(self):
        try:
            if self.base_cyclic:
                feedback = self.base_cyclic.RefreshFeedback()
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                
                for i, actuator in enumerate(feedback.actuators):
                    msg.name.append(f"joint_{i+1}")
                    msg.position.append(actuator.position)
                    msg.velocity.append(actuator.velocity)
                    msg.effort.append(actuator.torque)
                
                self.joint_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish joint states: {e}')

    def check_for_end_or_abort(self, e):
        def check(notification, e=e):
            event_name = Base_pb2.ActionEvent.Name(notification.action_event)
            self.get_logger().info(f"Event: {event_name}")
            if notification.action_event == Base_pb2.ACTION_END or \
               notification.action_event == Base_pb2.ACTION_ABORT:
                e.set()
        return check

    def command_callback(self, msg):
        self.get_logger().info(f"Received command: frame={msg.frame}, coord={msg.coordinate}")
        
        action = Base_pb2.Action()
        action.name = "ROS2 Command"
        action.application_data = ""

        if msg.frame == 'joint':
            # Validate length (assuming 7 DOF for Gen3, but could be 6)
            # We will just try to assign as many as provided
            for i, val in enumerate(msg.coordinate):
                joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
                joint_angle.joint_identifier = i
                joint_angle.value = val
        
        elif msg.frame == 'cartesian':
            if len(msg.coordinate) != 6:
                self.get_logger().error("Cartesian coordinate must have 6 values (x, y, z, theta_x, theta_y, theta_z)")
                return

            cartesian_pose = action.reach_pose.target_pose
            cartesian_pose.x = msg.coordinate[0]
            cartesian_pose.y = msg.coordinate[1]
            cartesian_pose.z = msg.coordinate[2]
            cartesian_pose.theta_x = msg.coordinate[3]
            cartesian_pose.theta_y = msg.coordinate[4]
            cartesian_pose.theta_z = msg.coordinate[5]

        else:
            self.get_logger().error(f"Unknown frame: {msg.frame}")
            return

        # Execute Action
        try:
            e = threading.Event()
            notification_handle = self.base.OnNotificationActionTopic(
                self.check_for_end_or_abort(e),
                Base_pb2.NotificationOptions()
            )

            self.get_logger().info("Executing action...")
            self.base.ExecuteAction(action)

            # Wait for completion
            finished = e.wait(20.0) # 20 seconds timeout
            self.base.Unsubscribe(notification_handle)

            if finished:
                self.get_logger().info("Action completed successfully")
            else:
                self.get_logger().error("Action timed out")

        except Exception as e:
            self.get_logger().error(f"Error executing action: {e}")

    def destroy_node(self):
        if self.session_manager:
            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000
            self.session_manager.CloseSession(router_options)
        if self.transport:
            self.transport.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KinovaDriverNode()
    
    # Use MultiThreadedExecutor to allow concurrent callbacks (timer + subscriber)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
