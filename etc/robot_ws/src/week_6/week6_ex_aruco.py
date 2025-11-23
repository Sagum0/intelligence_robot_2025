import pyrealsense2 as rs
import cv2
import numpy as np

# ==========================
# 설정값
# ==========================
MARKER_LENGTH_M = 0.03  # 마커 한 변 길이 (3cm = 0.03m)
ARUCO_DICT_TYPE = cv2.aruco.DICT_6X6_1000  # 6x6_1000 딕셔너리 사용
VALID_IDS = {0, 1, 2, 3}  # 사용할 마커 ID

def main():
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

            # --------------------------
            # 5. 화면 표시
            # --------------------------
            cv2.imshow("RealSense D435 - ArUco 6x6_1000 (ID 0~3)", color_img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
