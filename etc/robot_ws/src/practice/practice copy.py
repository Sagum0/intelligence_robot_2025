import threading
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2

import pyrealsense2 as rs
import cv2
import numpy as np

# ==========================
# 설정값
# ==========================
MARKER_LENGTH_M = 0.03  # 마커 한 변 길이 (3cm = 0.03m)
ARUCO_DICT_TYPE = cv2.aruco.DICT_6X6_1000  # 6x6_1000 딕셔너리 사용
VALID_IDS = {0, 1}  # 사용할 마커 ID

TIMEOUT_DURATION = 20

TARGET_POSE = {
    "x" : 0.2,
    "y" : 0.3,
    "z" : 0.05,
    "theta_x" : 180,
    "theta_y" : 0, 
    "theta_z" : 0
}

def connect_to_kinova(router):
    base = BaseClient(router)
    base_cyclic = BaseCyclicClient(router)
    
    return base, base_cyclic

def check_for_end_or_abort(e):
    def check(notification, e=e):
        print(" EVNET :", Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event in [Base_pb2.ACTION_END, Base_pb2.ACTION_ABORT]:
            e.set()
        
    return check

def example_move_to_home_base(base):
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    print("Moving the arm to a safe position")
    
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle
            
    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False
    
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    
    base.Unsubscribe(notification_handle)
    
    if finished:
        print("Safe Position Reached!")
        
    else:
        print("Timeout on action notification wait")
        
    return finished

def move_to_pose(base, base_cyclic, target_pose):
    action = Base_pb2.Action()
    action.name = "MoveToPose"
    
    pose = action.reach_pose.target_pose
    pose.x = target_pose["x"]
    pose.y = target_pose["y"]
    pose.z = target_pose["z"]
    pose.theta_x = target_pose["theta_x"]
    pose.theta_y = target_pose["theta_y"]
    pose.theta_z = target_pose["theta_z"]
    
    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )
    
    base.ExecuteAction(action)
    finished = e.wait(TIMEOUT_DURATION)
    
    base.Unsubscribe(notification_handle)
    
    if not finished:
        print("Action time out!")
        return False
        
    return finished


def main():
    import utilities
    
    args = utilities.parseConnectionArguments()
    
    # --------------------------
    # 1. RealSense 파이프라인 설정
    # --------------------------
    pipeline = rs.pipeline()
    config = rs.config()

    # 컬러 스트림만 사용 (필요하면 depth도 추가 가능)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 스트리밍 시작
    profile = pipeline.start(config)

    # 카메라 내장 파라미터(내부 파라미터) 가져오기
    color_profile = profile.get_stream(rs.stream.color)
    color_intr = color_profile.as_video_stream_profile().get_intrinsics()
    fx = color_intr.fx
    fy = color_intr.fy
    cx = color_intr.ppx
    cy = color_intr.ppy

    camera_matrix = np.array([[fx, 0,  cx],
                              [0,  fy, cy],
                              [0,  0,  1]], dtype=np.float32)
    # 왜곡 계수 (RealSense intrinsics default setup)
    dist_coeffs = np.array(color_intr.coeffs, dtype=np.float32)

    print("Camera Matrix:\n", camera_matrix)
    print("Dist Coeffs:\n", dist_coeffs)

    # --------------------------
    # 2. ArUco 설정
    # --------------------------
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    aruco_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    
    with utilities.DeviceConnection.createTcpConnection(args) as router:
        base, base_cyclic = connect_to_kinova(router)
        
        # success = example_move_to_home_base(base)
        # time.sleep(1.0)
        
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.5)
        
        marker_mat = np.zeros((2,3))
        
        try:
            while True:
                # --------------------------
                # 3. 프레임 받아오기
                # --------------------------
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue

                color_img = np.asanyarray(color_frame.get_data())
                gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)

                # --------------------------
                # 4. ArUco 마커 탐지
                # --------------------------
                corners, ids, rejected = detector.detectMarkers(gray)

                if ids is not None and len(ids) > 0:
                    # ArUco 테두리 및 ID 표시
                    cv2.aruco.drawDetectedMarkers(color_img, corners, ids)

                    # pose estimation (마커별 R, t)
                    # rvecs, tvecs shape: (N, 1, 3)
                    rvecs, tvecs, _obj_points = cv2.aruco.estimatePoseSingleMarkers(
                        corners,
                        MARKER_LENGTH_M,
                        camera_matrix,
                        dist_coeffs
                    )

                    for i, marker_id in enumerate(ids.flatten()):
                        # 0~3번 마커만 사용
                        if marker_id not in VALID_IDS:
                            continue

                        rvec = rvecs[i]
                        tvec = tvecs[i]

                        # 좌표축 그리기 (단위: m)
                        cv2.drawFrameAxes(
                            color_img,
                            camera_matrix,
                            dist_coeffs,
                            rvec,
                            tvec,
                            MARKER_LENGTH_M * 1.5  # 축 길이
                        )

                        # 마커 중심에 텍스트 출력
                        c = corners[i][0]
                        center = c.mean(axis=0).astype(int)
                        cv2.putText(
                            color_img,
                            f"ID:{marker_id}",
                            (center[0] - 20, center[1] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (0, 255, 0),
                            2
                        )

                        # 위치/자세 정보 출력 (카메라 좌표계 기준)
                        tx, ty, tz = tvec[0]
                        # rotation vector -> rotation matrix -> Euler angle(optional)
                        R, _ = cv2.Rodrigues(rvec)
                        
                        H = np.eye(4)
                        H[:3, :3] = R
                        H[:3, 3] = [tx, ty, tx]
                        
                        print(f"[ID {marker_id}] T (m): x={tx:.3f}, y={ty:.3f}, z={tz:.3f}")
                        print(f'[ID {marker_id}] : ')
                        print(f'{H}')
                        
                        marker_mat[i,:] = np.array([tx, ty, tz])

                # --------------------------
                # 5. 화면 표시
                # --------------------------
                cv2.imshow("RealSense D435 - ArUco 6x6_1000 (ID 0~3)", color_img)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                
                elif np.all(marker_mat != 0):
                    break
                    

        finally:
            pipeline.stop()
            cv2.destroyAllWindows()
                
        TARGET_POSE['y'] += (marker_mat[0, 0] - marker_mat[1, 0])
        TARGET_POSE['x'] += (marker_mat[1, 2] - marker_mat[0, 2])
          
        print(TARGET_POSE)    
        success = move_to_pose(base, base_cyclic, TARGET_POSE)
        time.sleep(0.5)
        
            
if __name__ == "__main__":
    main()