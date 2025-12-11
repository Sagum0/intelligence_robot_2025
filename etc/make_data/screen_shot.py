#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import cv2
import pyrealsense2 as rs

###############################
# 사용자가 설정하는 전역 변수
###############################
SAVE_DIR    = "/home/pc/intelrb_ws/etc/make_data/hole_both"  # 2. 저장 경로
FILE_PREFIX = "hole_both"                    # 3. 파일 이름 앞부분 (예: block_0.jpg)
START_INDEX = 0                        # 4. 시작 index 값
SAVE_EXT    = ".jpg"                     # ".jpg" 또는 ".png" 등

###############################
# 전역적으로 사용할 현재 index
###############################
IMG_INDEX = START_INDEX


def main():
    global IMG_INDEX

    # 저장 경로 생성
    os.makedirs(SAVE_DIR, exist_ok=True)

    # RealSense 파이프라인 설정
    pipeline = rs.pipeline()
    config = rs.config()

    # 컬러 스트림만 사용 (필요시 해상도/프레임 변경)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 스트리밍 시작
    pipeline.start(config)

    try:
        while True:
            # 프레임 가져오기
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # numpy 배열로 변환 (BGR)
            color_image = np.asanyarray(color_frame.get_data())

            # 화면 표시
            cv2.imshow("RealSense Color", color_image)

            # 키 입력 대기 (1ms)
            key = cv2.waitKey(1) & 0xFF

            # 's' 키를 누르면 캡쳐 및 저장
            if key == ord('s'):
                filename = f"{FILE_PREFIX}_{IMG_INDEX}{SAVE_EXT}"
                filepath = os.path.join(SAVE_DIR, filename)
                cv2.imwrite(filepath, color_image)
                print(f"Saved: {filepath}")
                IMG_INDEX += 1  # index 증가

            # 'q' 키를 누르면 종료
            if key == ord('q'):
                break

    finally:
        # 자원 해제
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    import numpy as np
    main()
