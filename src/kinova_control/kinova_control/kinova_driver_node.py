#!/usr/bin/env python3

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool

# Custom Message
from kinova_msgs.msg import KinovaCommand
from kinova_msgs.srv import KinovaExecutor

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

        self.gripper_sub = self.create_subscription(
            Float32,
            '/kinova/float/grp_cmd',
            self.gripper_callback,
            10,
            callback_group=self.reentrant_group
        )
        
        self.gripper_done_pub = self.create_publisher(Bool, '/kinova/bool/grp_done', 10)

        # Service
        self.srv = self.create_service(
            KinovaExecutor, 
            '/kinova/execute', 
            self.execute_service_callback, 
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



    def execute_service_callback(self, request, response):
        self.get_logger().info(f"Received Service Request: {request.frame}, {request.coordinate}")
        success = self.execute_robot_command(request.frame, request.coordinate)
        response.success = success
        response.message = "Completed" if success else "Failed or Timed out"
        return response

    def command_callback(self, msg):
        self.get_logger().info(f"Received Topic Command: frame={msg.frame}, coord={msg.coordinate}")
        self.execute_robot_command(msg.frame, msg.coordinate)

    def execute_robot_command(self, frame, coordinate):
        action = Base_pb2.Action()
        action.name = "ROS2 Command"
        action.application_data = ""

        if frame == 'joint':
            # Validate length (assuming 7 DOF for Gen3, but could be 6)
            # We will just try to assign as many as provided
            for i, val in enumerate(coordinate):
                joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
                joint_angle.joint_identifier = i
                joint_angle.value = val
        
        elif frame == 'cartesian':
            if len(coordinate) != 6:
                self.get_logger().error("Cartesian coordinate must have 6 values (x, y, z, theta_x, theta_y, theta_z)")
                return False

            cartesian_pose = action.reach_pose.target_pose
            cartesian_pose.x = coordinate[0]
            cartesian_pose.y = coordinate[1]
            cartesian_pose.z = coordinate[2]
            cartesian_pose.theta_x = coordinate[3]
            cartesian_pose.theta_y = coordinate[4]
            cartesian_pose.theta_z = coordinate[5]

        else:
            self.get_logger().error(f"Unknown frame: {frame}")
            return False

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
                return True
            else:
                self.get_logger().error("Action timed out")
                return False

        except Exception as e:
            self.get_logger().error(f"Error executing action: {e}")
            return False

    def gripper_callback(self, msg):
        target_pos = msg.data
        # Clamp target position to [0.0, 1.0]
        target_pos = max(0.0, min(1.0, target_pos))
        
        self.get_logger().info(f"Received gripper command: {target_pos}")

        try:
            # Create GripperCommand
            gripper_command = Base_pb2.GripperCommand()
            finger = gripper_command.gripper.finger.add()
            
            gripper_command.mode = Base_pb2.GRIPPER_POSITION
            finger.finger_identifier = 1
            finger.value = target_pos
            
            self.base.SendGripperCommand(gripper_command)
            
            # Wait for completion
            # We will poll the gripper status until it reaches the target or stops moving
            gripper_request = Base_pb2.GripperRequest()
            gripper_request.mode = Base_pb2.GRIPPER_POSITION
            
            start_time = time.time()
            timeout = 5.0 # seconds
            
            success = False
            
            while (time.time() - start_time) < timeout:
                gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
                if len(gripper_measure.finger):
                    current_pos = gripper_measure.finger[0].value
                    # Check if close enough (tolerance 0.01)
                    if abs(current_pos - target_pos) < 0.01:
                        success = True
                        break
                    
                    # Also check speed to see if it stopped (e.g. blocked)
                    # Note: GetMeasuredGripperMovement with GRIPPER_SPEED might be needed for speed
                    # But usually position feedback is enough. If it doesn't change for a while, it's done.
                    # For simplicity, we just wait for position match or timeout.
                    # If we want to detect "object grasped" (stopped before target), we need to check if position is stale.
                
                time.sleep(0.1)
            
            if success:
                self.get_logger().info("Gripper command completed successfully")
            else:
                self.get_logger().warn("Gripper command timed out or did not reach target")
                
            # Publish done status
            done_msg = Bool()
            done_msg.data = success # Or True if we consider "finished moving" as success regardless of position
            # The user asked for "success/failure", so let's return success if it reached target.
            # If it grasped an object, it might not reach target (e.g. closing to 1.0 but object at 0.5).
            # In that case, it is technically a "success" in terms of grasping, but "fail" in reaching target.
            # Usually for "done", we just want to know it finished acting. 
            # Let's refine: if it stops moving, it is done.
            
            # Refined logic: Check if stopped
            # But for now, simple target check is safer for "reached angle".
            # User said: "If Gripper reached user input angle". So strict check.
            
            self.gripper_done_pub.publish(done_msg)

        except Exception as e:
            self.get_logger().error(f"Error executing gripper command: {e}")
            done_msg = Bool()
            done_msg.data = False
            self.gripper_done_pub.publish(done_msg)

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
