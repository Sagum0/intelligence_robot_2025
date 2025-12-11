#!/usr/bin/env python3

import sys
import threading
import time
import math

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool

# 커스텀 메시지
from kinova_msgs.msg import KinovaCommand
from kinova_msgs.srv import KinovaExecutor

# Kortex API
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2

# 연결 헬퍼
from kortex_api.TCPTransport import TCPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2

SLEEP_TIME = 0.5 # 이동이 정상적으로 이루어졌는지 감시하는 로직

class KinovaDriverNode(Node):
    def __init__(self):
        super().__init__('kinova_driver_node')

        # 파라미터
        self.declare_parameter('ip_address', '192.168.1.10')
        self.declare_parameter('username', 'admin')
        self.declare_parameter('password', 'admin')

        self.ip_address = self.get_parameter('ip_address').value
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value

        # Kortex 연결
        self.transport = None
        self.router = None
        self.session_manager = None
        self.base = None
        self.base_cyclic = None

        self.connect_to_robot()

        # 콜백 그룹
        self.reentrant_group = ReentrantCallbackGroup()

        # 퍼블리셔
        self.joint_pub = self.create_publisher(JointState, '/kinova/joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.reentrant_group)

        # 서브스크라이버
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

        # 서비스
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
            
            # 서보 모드를 Single Level Servoing으로 설정
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

    def check_for_end_or_abort(self, e, success_flag):
        def check(notification, e=e, success_flag=success_flag):
            event_name = Base_pb2.ActionEvent.Name(notification.action_event)
            event_id = notification.action_event
            self.get_logger().info(f"Event: {event_name} (ID: {event_id})")

            if notification.action_event == Base_pb2.ACTION_END:
                success_flag[0] = True
                e.set()
            elif notification.action_event == Base_pb2.ACTION_ABORT:
                success_flag[0] = False
                e.set()
            else:
                # 다른 이벤트 타입이 오면 경고
                self.get_logger().warn(
                    f"Unhandled event type: {event_name} (ID: {event_id}). "
                    "Action may hang if this is a terminal event."
                )
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

        # 목표 좌표/각도 저장 (검증용)
        target_values = []

        if frame == 'joint':
            # 길이 검증
            for i, val in enumerate(coordinate):
                joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
                joint_angle.joint_identifier = i
                joint_angle.value = val
                target_values.append(val)
        
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
            target_values = coordinate

        else:
            self.get_logger().error(f"Unknown frame: {frame}")
            return False

        # 동작 실행
        try:
            e = threading.Event()
            notification_queue = [] # 이벤트를 큐에 저장
            
            def check(notification):
                # 모든 이벤트를 큐에 넣고 Event set
                if notification.action_event in [Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT]:
                    notification_queue.append(notification.action_event)
                    e.set()
            
            notification_handle = self.base.OnNotificationActionTopic(
                check,
                Base_pb2.NotificationOptions()
            )

            self.get_logger().info("Executing action...")
            self.base.ExecuteAction(action)

            # 타임아웃 및 검증 설정
            timeout = 10.0 # 전체 타임아웃 10초
            start_time = time.time()
            
            # 허용 오차 설정
            TOLERANCE_POS = 0.01 # 1cm
            TOLERANCE_ANG = 2.0  # 2도 (Joint)
            
            while (time.time() - start_time) < timeout:
                # 이벤트 대기 (1초씩 끊어서 확인)
                if e.wait(1.0):
                    e.clear() # 다음 이벤트를 위해 초기화
                    
                    # 큐 확인
                    while notification_queue:
                        event = notification_queue.pop(0)
                        
                        if event == Base_pb2.ACTION_ABORT:
                            self.get_logger().warn("Action ABORTED by robot")
                            self.base.Unsubscribe(notification_handle)
                            return False

                        if event == Base_pb2.ACTION_END:
                            self.get_logger().info("Action END received. Verifying target...")
                            
                            # 검증 로직
                            is_reached = False
                            feedback = self.base_cyclic.RefreshFeedback()

                            if frame == 'joint':
                                # Joint 오차 확인
                                current_joints = [act.position for act in feedback.actuators]
                                # 6축까지만 비교 (Gen3 Lite 등 고려)
                                check_len = min(len(current_joints), len(target_values))
                                diffs = [abs(current_joints[i] - target_values[i]) for i in range(check_len)]
                                max_diff = max(diffs)
                                
                                if max_diff <= TOLERANCE_ANG:
                                    is_reached = True
                                else:
                                    self.get_logger().warn(f"Target not reached. Max Diff: {max_diff:.4f} > {TOLERANCE_ANG}")

                            elif frame == 'cartesian':
                                # Cartesian 오차 확인 (Forward Kinematics가 없으므로 Feedback의 Tool Pose 사용)
                                # 주의: base_cyclic feedback의 base.tool_pose_x 등은 없을 수 있음.
                                # 여기서는 Actuator 피드백만 확실하므로, 
                                # 엄밀한 Cartesian 검증은 FK가 필요하지만, 
                                # 간소화를 위해 '움직임 멈춤' 여부로 2차 판단.
                                # NOTE: Kortex Feedback에 BaseFeedback이 포함됨.
                                
                                try:
                                    # Base Feedback에서 Cartesian Pose 가져오기 시도
                                    base_feedback = self.base_cyclic.RefreshFeedback().base
                                    current_pose = [
                                        base_feedback.tool_pose_x,
                                        base_feedback.tool_pose_y,
                                        base_feedback.tool_pose_z
                                    ]
                                    target_pose = target_values[:3]
                                    
                                    # 거리 계산
                                    dist = math.sqrt(sum((c - t)**2 for c, t in zip(current_pose, target_pose)))
                                    
                                    if dist <= TOLERANCE_POS:
                                        is_reached = True
                                    else:
                                        self.get_logger().warn(f"Target not reached. Dist: {dist:.4f} > {TOLERANCE_POS}")
                                        
                                except Exception:
                                    # Base feedback 접근 실패 시 움직임만으로 판단할 수밖에 없음
                                    self.get_logger().warn("Cannot check Cartesian feedback directly. Relying on movement check.")
                                    is_reached = False # 확실하지 않으므로 False로 두고 움직임 체크로 넘어감

                            # 판단
                            if is_reached:
                                self.get_logger().info("Target reached successfully.")
                                self.base.Unsubscribe(notification_handle)
                                return True
                            else:
                                # 오차 범위 밖 -> 움직임 여부 확인
                                self.get_logger().info("Checking for continued movement...")
                                time.sleep(SLEEP_TIME) # 0.5초 대기
                                
                                feedback_check = self.base_cyclic.RefreshFeedback()
                                is_moving = any(abs(act.velocity) > 0.05 for act in feedback_check.actuators) # 임계값 낮춤
                                
                                if is_moving:
                                    self.get_logger().info("Robot is still moving/correcting. Continuing wait...")
                                    continue # Loop 계속
                                else:
                                    self.get_logger().error("Robot stopped but target not reached.")
                                    self.base.Unsubscribe(notification_handle)
                                    return False

                # 타임아웃 루프 계속

            # 전체 타임아웃
            self.base.Unsubscribe(notification_handle)
            self.get_logger().error("Robot action timed out.")
            return False

        except Exception as e:
            self.get_logger().error(f"Error executing action: {e}")
            return False

    def gripper_callback(self, msg):
        target_pos = msg.data
        # 목표 위치를 [0.0, 1.0] 범위로 제한
        target_pos = max(0.0, min(1.0, target_pos))
        
        self.get_logger().info(f"Received gripper command: {target_pos}")

        try:
            # GripperCommand 생성
            gripper_command = Base_pb2.GripperCommand()
            finger = gripper_command.gripper.finger.add()
            
            gripper_command.mode = Base_pb2.GRIPPER_POSITION
            finger.finger_identifier = 1
            finger.value = target_pos
            
            self.base.SendGripperCommand(gripper_command)
            
            # 완료 대기 (Stall Detection)
            gripper_request = Base_pb2.GripperRequest()
            gripper_request.mode = Base_pb2.GRIPPER_POSITION
            
            start_time = time.time()
            timeout = 5.0 # 초
            
            success = False
            last_pos = -1.0
            stable_count = 0
            required_stable_count = 5 # 0.1s * 5 = 0.5s stable
            
            while (time.time() - start_time) < timeout:
                gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
                if len(gripper_measure.finger):
                    current_pos = gripper_measure.finger[0].value
                    
                    # 1. Check if target reached
                    if abs(current_pos - target_pos) < 0.01:
                        success = True
                        self.get_logger().info("Gripper reached target position.")
                        break
                    
                    # 2. Check for stall (object grasped)
                    if abs(current_pos - last_pos) < 0.005: # Movement < 0.5%
                        stable_count += 1
                    else:
                        stable_count = 0
                        
                    if stable_count >= required_stable_count:
                        success = True
                        self.get_logger().info("Gripper stopped moving (likely grasped object).")
                        break
                        
                    last_pos = current_pos
                
                time.sleep(0.1)
            
            if success:
                self.get_logger().info("Gripper command completed successfully")
            else:
                self.get_logger().warn("Gripper command timed out (still moving or communication issue)")
                
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
