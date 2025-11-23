import pyrealsense2 as rs
import numpy as np
import cv2
import os

# 저장 디렉토리 (전역 변수)
OUTDIR = "/home/pc/image_data_set"  # 원하는 경로로 수정

# 저장 폴더가 없으면 생성
os.makedirs(OUTDIR, exist_ok=True)

# 카메라 해상도
WIDTH, HEIGHT = 640, 480

# 카메라 초기화
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, 30)
pipeline.start(config)

image_num = 31

try:
    while True:
        # 프레임 획득
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # 이미지 변환
        color_image = np.asanyarray(color_frame.get_data())

        # 화면에 이미지 출력
        cv2.imshow('RealSense Capture (Press "s" to Save, "q" to Quit)', color_image)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            # 파일 저장
            filename = f"week_2_{image_num}.jpg"
            filepath = os.path.join(OUTDIR, filename)
            cv2.imwrite(filepath, color_image)
            print(f"Saved: {filepath}")
            image_num += 1

        elif key == ord('q'):
            # 종료
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
