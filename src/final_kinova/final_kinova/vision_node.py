import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, Int32
import cv2
import numpy as np
import pyrealsense2 as rs
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from ultralytics import YOLO
import math
import time
from collections import deque

PT_PATH = '/home/pc/intelrb_ws/src/kinova_control/best.pt'
CLASS_NAMES = ['hex', 'squ']
CONF_THRES = 0.5
MAKER_SIZE = 0.03  # meters
DICT_ID = cv2.aruco.DICT_6X6_50
MASK_COLORS = np.array([
    [0, 0, 255],    # hex: red
    [255, 0, 0]     # squ: blue
], dtype=np.uint8)

ANGLE_RANGE = {
    'hex': 35,  # ±35°
    'squ': 47   # ±47°
}
LONGEST_COLORS = {
    'hex': (0, 0, 255),    # red (BGR)
    'squ': (255, 0, 0),    # blue (BGR)
}

class VisionNode(Node):
    def __init__(self):
        super().__init__('kinova_vision_node')

        self.sub = self.create_subscription(
            String, '/kinova/string/obj_command', self.trigger_callback, 10)
        
        self.aruco_trigger_sub = self.create_subscription(
            Int32, '/kinova/int/maker_command', self.aruco_trigger_callback, 10
        )
        
        self.pub = self.create_publisher(
            Float32MultiArray, '/kinova/array/obj_pose', 10)
            
        self.joint_sub = self.create_subscription(
            JointState, '/kinova/joint_states', self.joint_callback, 10
        )
            
        self.joint_sub = self.create_subscription(
            JointState, '/kinova/joint_states', self.joint_callback, 10
        )

        self.aruco_pose_pub = self.create_publisher(
            Float32MultiArray, '/kinova/array/maker_pose', 10
        )
        
        # RealSense 카메라 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # YOLO 로드
        self.model = YOLO(PT_PATH)
        
        # 연산 플래그 및 변수
        self.detection_target = None  # 'hex' or 'squ'
        self.idle_mode = True
        self.detecting = False
        self.detection_count = 0
        self.positions = []
        self.positions = []
        self.yaws = []
        
        # Delay related
        self.start_delay_time = None
        self.delay_duration = 2.0 # seconds

        self.depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()
        self.intrinsics = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        
        # ArUco 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_target_id = None
        self.detecting_aruco = False
        self.aruco_positions = []
        
        # Camera Matrix for ArUco (will be set in main_loop if needed, or derived from intrinsics)
        self.camera_matrix = np.array([
            [self.intrinsics.fx, 0, self.intrinsics.ppx],
            [0, self.intrinsics.fy, self.intrinsics.ppy],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.array(self.intrinsics.coeffs)

    def trigger_callback(self, msg):
        if self.detecting:
            # 이미 진행 중이면 무시
            return
        class_name = msg.data.strip().lower()
        if class_name not in CLASS_NAMES:
            self.get_logger().info(f"Unknown class name: {class_name}")
            return
        self.get_logger().info(f"Trigger received: {class_name}")

        if self.detecting:
            # 이미 실행 중이면 무시
            return

        # Trigger 감지 → YOLO On
        self.idle_mode = False
        self.detecting = True
        self.detection_target = class_name
        self.detection_count = 0
        self.positions = []
        self.yaws = []
        self.start_delay_time = time.time()
        self.get_logger().info(f"Trigger received. Waiting {self.delay_duration}s for robot stop...")

    def aruco_trigger_callback(self, msg):
        if self.detecting or self.detecting_aruco:
            return
            
        target_id = msg.data
        self.get_logger().info(f"ArUco Trigger received: ID {target_id}")
        
        self.idle_mode = False
        self.detecting_aruco = True
        self.aruco_target_id = target_id
        self.detection_count = 0
        self.aruco_target_id = target_id
        self.detection_count = 0
        self.aruco_positions = []
        self.aruco_rvecs = []
        self.start_delay_time = time.time()
        self.get_logger().info(f"ArUco Trigger received. Waiting {self.delay_duration}s for robot stop...")

    def joint_callback(self, msg):
        self.current_joint_angles = np.radians(np.array(msg.position))

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

    def get_pose_matrix_from_yaw(self, x, y, z, yaw_deg):
        # Yaw rotation around Z axis
        theta = np.radians(yaw_deg)
        c, s = np.cos(theta), np.sin(theta)
        
        # Rotation Matrix (Rotation around Z)
        # R = [[cos, -sin, 0], [sin, cos, 0], [0, 0, 1]]
        R = np.eye(3)
        R[0, 0] = c
        R[0, 1] = -s
        R[1, 0] = s
        R[1, 1] = c
        
        # 4x4 Matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    def get_pose_matrix_from_rvec_tvec(self, rvec, tvec):
        # Convert rvec to rotation matrix
        R, _ = cv2.Rodrigues(rvec)
        
        # 4x4 Matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec
        return T
    def get_depth_xyz(self, cx, cy, depth_frame):
        # 중심점 픽셀 좌표에서 depth→xyz 변환
        depth = depth_frame.get_distance(int(cx), int(cy))
        if depth == 0 or np.isnan(depth):
            return None  # invalid
        x, y, z = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [float(cx), float(cy)], depth)
        return (x, y, z)

    def calc_theta_y_axis(self, pt1, pt2):
        dx = pt2[0] - pt1[0]
        dy = pt2[1] - pt1[1]
        norm = math.hypot(dx, dy)
        if norm == 0:
            return 0.0
        theta_rad = math.atan2(dx, dy)
        theta_deg = np.degrees(theta_rad)
        if theta_deg > 90:
            theta_deg -= 180
        elif theta_deg < -90:
            theta_deg += 180
        return theta_deg

    def main_loop(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0)
                # 실시간 프레임 획득 (align)
                frames = self.pipeline.wait_for_frames()
                frames = self.align.process(frames)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                frame = np.asanyarray(color_frame.get_data())
                
                # Base Frame FK Calculation
                T_base_camera = None
                if self.current_joint_angles is not None and len(self.current_joint_angles) >= 6:
                    try:
                        T_base_camera = self.compute_fk(self.current_joint_angles)
                    except Exception as e:
                        self.get_logger().error(f"FK Error: {e}")


                # -----------------------------
                # ① 프리뷰 모드 (idle_mode = True)
                # -----------------------------
                if self.idle_mode and not self.detecting and not self.detecting_aruco:
                    cv2.imshow("YOLOv8n-seg-Realsense-ROS2", frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    continue
                
                # ArUco 모드일 때 YOLO 건너뛰기
                # -----------------------------
                # ② Trigger 후 YOLO 시작
                # -----------------------------
                if self.detecting_aruco:
                    results = []
                else:
                    results = self.model(frame)
                target_found = False

                # Delay Check
                if (self.detecting or self.detecting_aruco) and self.start_delay_time is not None:
                    if time.time() - self.start_delay_time < self.delay_duration:
                        # Still in delay period
                        # Just show preview
                        cv2.putText(frame, "Waiting for Robot Stop...", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                        cv2.imshow("YOLOv8n-seg-Realsense-ROS2", frame)
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                        continue
                    else:
                         pass # Delay finished

                for r in results:
                    if r.masks is not None and r.boxes is not None:
                        masks = r.masks.data.cpu().numpy()
                        boxes = r.boxes
                        classes = boxes.cls.cpu().numpy().astype(int)
                        confs = boxes.conf.cpu().numpy()
                        for i, mask in enumerate(masks):
                            class_idx = classes[i]
                            class_name = CLASS_NAMES[class_idx]
                            
                            # 타겟 클래스가 아니면 스킵 (YOLO 결과 자체를 필터링할 수도 있지만, 여기서는 순회하며 체크)
                            if self.detecting and class_name != self.detection_target:
                                continue

                            if confs[i] < CONF_THRES:
                                continue
                            
                            color = tuple(int(c) for c in MASK_COLORS[class_idx])
                            colored_mask = np.zeros_like(frame, dtype=np.uint8)
                            for ch in range(3):
                                colored_mask[:, :, ch] = mask * color[ch]
                            frame = cv2.addWeighted(frame, 1.0, colored_mask, 0.4, 0)

                            mask_uint8 = (mask * 255).astype(np.uint8)
                            contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            if len(contours) > 0:
                                max_contour = max(contours, key=cv2.contourArea)
                                epsilon = 0.02 * cv2.arcLength(max_contour, True)
                                approx = cv2.approxPolyDP(max_contour, epsilon, True)

                                cv2.polylines(frame, [approx], isClosed=True, color=color, thickness=1)
                                pts = [tuple(pt[0]) for pt in approx]
                                for pt in pts:
                                    cv2.circle(frame, pt, 3, color, -1)

                                # 중심점(무게중심) 계산 및 표시
                                M = cv2.moments(mask_uint8)
                                if M["m00"] != 0:
                                    cx = int(M["m10"] / M["m00"])
                                    cy = int(M["m01"] / M["m00"])
                                    cv2.circle(frame, (cx, cy), 6, (255, 255, 255), -1)
                                    cv2.putText(frame, f"({cx},{cy})", (cx+8, cy-8),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                                else:
                                    cx, cy = -1, -1

                                # === 연산 트리거된 경우만 아래 코드 수행 ===
                                if self.detecting and cx >= 0 and cy >= 0:
                                    # 1. 각도 후보 선정
                                    n = len(pts)
                                    angle_limit = ANGLE_RANGE[class_name]
                                    edge_info = []
                                    for j in range(n):
                                        pt1 = pts[j]
                                        pt2 = pts[(j+1)%n]
                                        length = math.hypot(pt2[0]-pt1[0], pt2[1]-pt1[1])
                                        theta_deg = self.calc_theta_y_axis(pt1, pt2)
                                        if -angle_limit <= theta_deg <= angle_limit:
                                            edge_info.append({'idx': j, 'length': length, 'theta': theta_deg, 'pt1': pt1, 'pt2': pt2})
                                    # 2. 그 중 가장 긴 모서리 선정
                                    if edge_info:
                                        max_edge = max(edge_info, key=lambda x: x['length'])
                                        pt1, pt2 = max_edge['pt1'], max_edge['pt2']
                                        theta_deg = max_edge['theta']
                                        # 모서리 강조
                                        cv2.line(frame, pt1, pt2, LONGEST_COLORS[class_name], thickness=3)
                                        mx = int((pt1[0] + pt2[0]) / 2)
                                        my = int((pt1[1] + pt2[1]) / 2)
                                        cv2.putText(frame, f"{theta_deg:.1f} deg", (mx, my),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, LONGEST_COLORS[class_name], 2, cv2.LINE_AA)
                                        yaw = theta_deg
                                    else:
                                        yaw = 0.0

                                    # 3D 좌표 변환 (Mask Median Depth 방식)
                                    pos3d = None
                                    depth_data = np.asanyarray(depth_frame.get_data())
                                    
                                    # 마스크 영역 내 Depth만 추출
                                    if mask_uint8 is not None:
                                        # 마스크가 0이 아닌 부분의 depth 값 추출
                                        masked_depth = depth_data[mask_uint8 > 0]
                                        # 유효 depth(0이 아닌 값)만 필터링
                                        valid_depth = masked_depth[masked_depth > 0]

                                        if len(valid_depth) > 0:
                                            # 중앙값(Median) 계산
                                            median_depth_mm = np.median(valid_depth)
                                            median_depth_m = median_depth_mm * self.depth_scale
                                            
                                            self.get_logger().info(f"Measured Depth: {median_depth_m:.4f} m")

                                            # Deprojection (cx, cy는 방향, depth는 Median값 사용)
                                            # intrinsics, pixel, depth -> point
                                            x, y, z = rs.rs2_deproject_pixel_to_point(
                                                self.intrinsics, [float(cx), float(cy)], median_depth_m)
                                            
                                            pos3d = (x, y, z)
                                    
                                    # 만약 위의 방식으로 실패했다면 기존 center 방식 fallback 혹은 None
                                    if pos3d is None:
                                         pos3d = self.get_depth_xyz(cx, cy, depth_frame)

                                    if pos3d is not None:
                                        self.positions.append(pos3d)
                                        self.yaws.append(yaw)
                                        self.detection_count += 1
                                        target_found = True
                                        self.get_logger().info(f"Detect {class_name} {self.detection_count}/10: xyz=({pos3d[0]:.3f}, {pos3d[1]:.3f}, {pos3d[2]:.3f}), yaw={yaw:.2f}")
                                    
                                    # Enough data, publish & reset
                                    if self.detection_count >= 10:
                                        if T_base_camera is None:
                                            self.get_logger().warn("Waiting for joints to compute FK...")
                                            self.detection_count = 0 
                                            self.positions = []
                                            self.yaws = []
                                            continue

                                        pos_np = np.array(self.positions)
                                        yaws_np = np.array(self.yaws)
                                        median_pos = np.median(pos_np, axis=0)
                                        median_yaw = np.median(yaws_np)
                                        
                                        # 1. Camera Frame Pose Matrix
                                        T_camera_obj = self.get_pose_matrix_from_yaw(
                                            median_pos[0], median_pos[1], median_pos[2], median_yaw
                                        )
                                        
                                        # 2. Base Frame Pose Matrix
                                        T_base_obj = T_base_camera @ T_camera_obj
                                        
                                        # 3. Extract [x, y, z] and use raw Yaw
                                        base_pos = T_base_obj[0:3, 3]
                                        
                                        # Use median_yaw directly as requested by user (Pixel coordinate yaw)
                                        final_data = [base_pos[0], base_pos[1], base_pos[2], median_yaw]
                                        
                                        msg = Float32MultiArray()
                                        msg.data = final_data
                                        self.pub.publish(msg)
                                        self.get_logger().info(f"Published Object Base Pose: {final_data}")
                                        
                                        # ----------- YOLO 분석 종료 -----------
                                        self.detecting = False
                                        self.idle_mode = True
                                        self.detection_target = None
                                        self.positions = []
                                        self.yaws = []
                                        self.detection_count = 0
                                        self.get_logger().info("YOLO detection finished. Back to idle mode.")
                                        self.detection_count = 0
                                        self.get_logger().info("YOLO detection finished. Back to idle mode.")

                # -----------------------------
                # ③ Trigger 후 ArUco 시작
                # -----------------------------
                if self.detecting_aruco:
                    corners, ids, rejected = cv2.aruco.detectMarkers(
                        frame, self.aruco_dict, parameters=self.aruco_params
                    )
                    
                    if ids is not None:
                        ids_flatten = ids.flatten()
                        for i, marker_id in enumerate(ids_flatten):
                            if marker_id == self.aruco_target_id:
                                # Pose Estimation
                                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                                    corners[i], MAKER_SIZE, self.camera_matrix, self.dist_coeffs
                                )
                                rvec = rvec[0][0]
                                tvec = tvec[0][0]
                                
                                # 시각화
                                cv2.aruco.drawDetectedMarkers(frame, corners, ids, borderColor=(0, 255, 0))
                                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.02)
                                
                                # rvec is [rx, ry, rz], tvec is [tx, ty, tz]
                                # Store raw rvec/tvec for median
                                self.aruco_positions.append(np.array([tvec[0], tvec[1], tvec[2]]))
                                self.aruco_rvecs.append(np.array([rvec[0], rvec[1], rvec[2]]))
                                
                                self.detection_count += 1
                                self.get_logger().info(f"ArUco {marker_id} detected {self.detection_count}/10")
                                
                                if self.detection_count >= 10:
                                    if T_base_camera is None:
                                        self.get_logger().warn("Waiting for joints to compute FK...")
                                        self.detection_count = 0 
                                        self.aruco_positions = []
                                        self.aruco_rvecs = []
                                        continue

                                    tvecs_np = np.array(self.aruco_positions)
                                    rvecs_np = np.array(self.aruco_rvecs)
                                    
                                    median_tvec = np.median(tvecs_np, axis=0)
                                    median_rvec = np.median(rvecs_np, axis=0)
                                    
                                    # 1. Camera Frame Pose Matrix
                                    T_camera_marker = self.get_pose_matrix_from_rvec_tvec(median_rvec, median_tvec)

                                    # [Refinement] Apply Offset relative to Marker Frame
                                    # User requested -45mm in Y-axis direction from the marker
                                    T_marker_offset = np.eye(4)
                                    T_marker_offset[1, 3] = -0.046 # -45mm Y offset

                                    # New Camera Frame Pose (Target Point)
                                    T_camera_target = T_camera_marker @ T_marker_offset
                                    
                                    # 2. Base Frame Pose Matrix (Transforming the Offset Target)
                                    T_base_marker = T_base_camera @ T_camera_target
                                    
                                    # 3. Extract [x, y, z] only
                                    base_pos = T_base_marker[0:3, 3]
                                    final_data = [base_pos[0], base_pos[1], base_pos[2]]
                                    
                                    msg = Float32MultiArray()
                                    msg.data = final_data
                                    self.aruco_pose_pub.publish(msg)
                                    self.get_logger().info(f"ArUco Published Base Position: {final_data}")
                                    
                                    # Reset
                                    self.detecting_aruco = False
                                    self.idle_mode = True
                                    self.aruco_target_id = None
                                    self.detection_count = 0
                                    self.aruco_positions = []
                                    self.aruco_rvecs = []
                                    self.get_logger().info("ArUco detection finished. Back to idle mode.")
                                break # Found target, stop checking other markers in this frame
                # imshow는 항상!
                cv2.imshow("YOLOv8n-seg-Realsense-ROS2", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            self.pipeline.stop()
            cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    node.main_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
