#!/usr/bin/env python3

import sys
import os
import threading
import time
import math

# ROS2 Imports
import rclpy
from rclpy.node import Node
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

        # 1. Subscriber (Cartesian Goal)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/kinova/array/goal_pose',
            self.listener_callback,
            10
        )
        self.get_logger().info('Waiting for goal pose on /kinova/array/goal_pose...')

        # 2. Publisher (Joint State - 10Hz)
        self.joint_pub = self.create_publisher(JointState, '/kinova/joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # 3. Kortex API Connection
        self.args = utilities.parseConnectionArguments()
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
        """10Hz 주기로 Joint State 발행"""
        # 주의: listener_callback에서 wait()가 걸리면 이 타이머도 일시 정지될 수 있습니다.
        # 이를 방지하려면 MultiThreadedExecutor를 사용해야 하지만, 
        # 현재 구조에서는 이동 중 JointState 업데이트가 멈출 수 있음을 참고하십시오.
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
                # 주의: 이 wait 동안에는 Node의 다른 콜백(Timer 등)이 멈춥니다.
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()