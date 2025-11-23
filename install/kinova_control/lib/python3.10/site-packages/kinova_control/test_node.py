#!/usr/bin/env python3

import sys
import os
import threading
import time
import math

# ROS2 Imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

# Kortex API Imports
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2

# 유틸리티
from kinova_control import utilities 

# 이동 제한 시간 (초)
TIMEOUT_DURATION = 20

class KinovaCartesianMover(Node):
    def __init__(self):
        super().__init__('kinova_cartesian_mover')

        # Callback Groups: Timer와 Subscriber를 병렬 실행
        self.timer_group = ReentrantCallbackGroup()
        self.subscription_group = MutuallyExclusiveCallbackGroup()

        # 1. Subscriber (Cartesian Goal)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/kinova/array/goal_pose',
            self.listener_callback,
            10,
            callback_group=self.subscription_group
        )
        self.get_logger().info('Waiting for goal pose on /kinova/array/goal_pose...')

        # 2. Publisher (Joint State - 10Hz)
        self.joint_pub = self.create_publisher(JointState, '/kinova/joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.timer_group)

        # 3. Kortex API Connection
        self.declare_parameter('ip_address', '192.168.1.10')
        self.declare_parameter('username', 'admin')
        self.declare_parameter('password', 'admin')

        class ConnectionOptions:
            def __init__(self, ip, username, password):
                self.ip = ip
                self.username = username
                self.password = password

        self.args = ConnectionOptions(
            self.get_parameter('ip_address').get_parameter_value().string_value,
            self.get_parameter('username').get_parameter_value().string_value,
            self.get_parameter('password').get_parameter_value().string_value
        )
        self.router = None
        self.base = None
        self.base_cyclic = None

        try:
            self.connection = utilities.DeviceConnection.createTcpConnection(self.args)
            self.router = self.connection.__enter__()
            self.base = BaseClient(self.router)
            self.base_cyclic = BaseCyclicClient(self.router)
            self.get_logger().info('Connected to KINOVA Kortex API')
            
            self.clear_faults()
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to KINOVA: {e}')
            sys.exit(1)

    def clear_faults(self):
        try:
            self.base.ClearFaults()
            self.get_logger().info('Faults cleared (if any).')
        except Exception as e:
            self.get_logger().warn('Failed to clear faults.')

    def ensure_high_level_servoing(self):
        try:
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)
            time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f'Failed to set servoing mode: {e}')

    def timer_callback(self):
        """10Hz 주기로 Joint State 발행 (병렬 실행)"""
        # MultiThreadedExecutor와 ReentrantCallbackGroup 사용으로
        # listener_callback의 wait() 중에도 이 타이머는 계속 동작합니다.
        try:
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
            pass

    def print_validation_error(self, error_report):
        self.get_logger().error("❌ [ABORT PREDICTION] Trajectory Validation Failed!")
        for i, error in enumerate(error_report.trajectory_error_elements):
            error_type_str = Base_pb2.TrajectoryErrorType.Name(error.error_type)
            self.get_logger().error(f"  [Error #{i+1}] Reason: {error_type_str}")

    # [추가된 메서드] 액션 이벤트(종료/중단)를 감지하는 콜백 클로저
    def check_for_end_or_abort(self, e):
        def check(notification, e=e):
            # 이벤트 이름 로깅
            event_name = Base_pb2.ActionEvent.Name(notification.action_event)
            self.get_logger().info(f"🤖 EVENT RECEIVED: {event_name}")
            
            if notification.action_event == Base_pb2.ACTION_END:
                self.get_logger().info("✅ Movement Completed Successfully.")
                e.set()
            elif notification.action_event == Base_pb2.ACTION_ABORT:
                # Abort 세부 내용 출력
                error_details = Base_pb2.SubErrorCodes.Name(notification.abort_details)
                self.get_logger().error(f"❌ Movement ABORTED. Reason: {error_details}")
                e.set()
        return check

    def listener_callback(self, msg):
        if len(msg.data) != 6:
            self.get_logger().warn('Invalid array length. Expected 6.')
            return

        x, y, z = msg.data[0], msg.data[1], msg.data[2]
        tx, ty, tz = msg.data[3], msg.data[4], msg.data[5]

        self.get_logger().info(f'📥 Request: Pos({x:.2f}, {y:.2f}, {z:.2f}) Rot({tx:.2f}, {ty:.2f}, {tz:.2f})')

        waypoints = Base_pb2.WaypointList()
        waypoints.duration = 0.0
        waypoints.use_optimal_blending = False
        waypoint = waypoints.waypoints.add()
        waypoint.name = "ros_pose"
        waypoint.cartesian_waypoint.pose.x = x
        waypoint.cartesian_waypoint.pose.y = y
        waypoint.cartesian_waypoint.pose.z = z
        waypoint.cartesian_waypoint.pose.theta_x = tx
        waypoint.cartesian_waypoint.pose.theta_y = ty
        waypoint.cartesian_waypoint.pose.theta_z = tz
        waypoint.cartesian_waypoint.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_BASE
        waypoint.cartesian_waypoint.blending_radius = 0.0

        try:
            # 1. 유효성 검사
            result = self.base.ValidateWaypointList(waypoints)
            if len(result.trajectory_error_report.trajectory_error_elements) == 0:
                
                self.ensure_high_level_servoing()
                self.get_logger().info("✅ Validation Passed. Starting Execution...")
                
                # [핵심 수정] 알림 구독 및 대기 로직 추가
                e = threading.Event()
                notification_handle = self.base.OnNotificationActionTopic(
                    self.check_for_end_or_abort(e),
                    Base_pb2.NotificationOptions()
                )

                # 2. 실행
                self.base.ExecuteWaypointTrajectory(waypoints)

                # 3. 결과 대기 (Blocking Wait)
                # MultiThreadedExecutor 사용으로 Timer는 계속 동작합니다.
                finished = e.wait(TIMEOUT_DURATION)
                
                # 4. 구독 해제
                self.base.Unsubscribe(notification_handle)

                if not finished:
                    self.get_logger().error("⏳ Timeout: Action did not finish within duration.")
                
            else:
                self.print_validation_error(result.trajectory_error_report)

        except Exception as e:
            self.get_logger().error(f'Critical Error: {e}')

    def destroy_node(self):
        if self.connection:
            self.connection.__exit__(None, None, None)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = KinovaCartesianMover()

    # MultiThreadedExecutor 사용: Timer와 Subscriber를 병렬로 실행
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()