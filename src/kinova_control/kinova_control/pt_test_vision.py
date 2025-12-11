import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO
import math

PT_PATH = '/home/pc/intelrb_ws/src/kinova_control/best.pt'
CLASS_NAMES = ['hex', 'squ']
CONF_THRES = 0.5

model = YOLO(PT_PATH)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

np.random.seed(42)
MASK_COLORS = np.random.randint(0, 255, (len(CLASS_NAMES), 3), dtype=np.uint8)

LONGEST_COLORS = {
    'squ': (255, 0, 0),  # Blue
    'hex': (0, 0, 255),  # Red
}

ANGLE_RANGE = {
    'hex': 35,  # ±35°
    'squ': 47   # ±47°
}

def calc_theta_y_axis(pt1, pt2):
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

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        frame = np.asanyarray(color_frame.get_data())

        results = model(frame)
        conf_list = []

        for r in results:
            if r.masks is not None and r.boxes is not None:
                masks = r.masks.data.cpu().numpy()
                boxes = r.boxes
                classes = boxes.cls.cpu().numpy().astype(int)
                confs = boxes.conf.cpu().numpy()
                for i, mask in enumerate(masks):
                    if confs[i] < CONF_THRES:
                        continue
                    conf_list.append(confs[i])
                    # color 튜플화 (cv2 color 에러 방지)
                    color = tuple(int(c) for c in MASK_COLORS[classes[i]])
                    class_name = CLASS_NAMES[classes[i]]
                    color_longest = LONGEST_COLORS[class_name]
                    angle_limit = ANGLE_RANGE[class_name]

                    # --- Mask 오버레이 ---
                    colored_mask = np.zeros_like(frame, dtype=np.uint8)
                    for ch in range(3):
                        colored_mask[:, :, ch] = mask * color[ch]
                    frame = cv2.addWeighted(frame, 1.0, colored_mask, 0.4, 0)

                    # --- Mask contour 및 꼭지점 추출 ---
                    mask_uint8 = (mask * 255).astype(np.uint8)
                    contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if len(contours) > 0:
                        max_contour = max(contours, key=cv2.contourArea)
                        epsilon = 0.02 * cv2.arcLength(max_contour, True)
                        approx = cv2.approxPolyDP(max_contour, epsilon, True)

                        # 꼭지점 선 연결 (기본 얇은 선)
                        cv2.polylines(frame, [approx], isClosed=True, color=color, thickness=1)

                        pts = [tuple(pt[0]) for pt in approx]
                        for pt in pts:
                            cv2.circle(frame, pt, 3, color, -1)

                        # --- 조건에 맞는 모서리 후보 선별 ---
                        n = len(pts)
                        edge_info = []
                        for j in range(n):
                            pt1 = pts[j]
                            pt2 = pts[(j+1)%n]
                            length = math.hypot(pt2[0]-pt1[0], pt2[1]-pt1[1])
                            theta_deg = calc_theta_y_axis(pt1, pt2)
                            # 1. 각도 조건
                            if -angle_limit <= theta_deg <= angle_limit:
                                edge_info.append({'idx': j, 'length': length, 'theta': theta_deg, 'pt1': pt1, 'pt2': pt2})

                        # 2. 그 중 가장 긴 모서리만 강조
                        if edge_info:
                            max_edge = max(edge_info, key=lambda x: x['length'])
                            pt1, pt2 = max_edge['pt1'], max_edge['pt2']
                            theta_deg = max_edge['theta']
                            cv2.line(frame, pt1, pt2, color_longest, thickness=3)
                            mx = int((pt1[0] + pt2[0]) / 2)
                            my = int((pt1[1] + pt2[1]) / 2)
                            cv2.putText(frame, f"{theta_deg:.1f} deg", (mx, my),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_longest, 2, cv2.LINE_AA)
                        # 후보 없으면 강조선 없음

                    # --- 중심점(무게중심) 표시 ---
                    M = cv2.moments(mask_uint8)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(frame, (cx, cy), 6, (255, 255, 255), -1)
                        cv2.putText(frame, f"({cx},{cy})", (cx+8, cy-8),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 정확도 표시
        if len(conf_list) > 0:
            avg_conf = np.mean(conf_list)
            acc_text = f"Mean Confidence: {avg_conf:.2f}"
        else:
            acc_text = "Mean Confidence: N/A"
        cv2.putText(frame, acc_text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

        cv2.imshow('YOLOv8n-seg-Realsense-LongestEdge', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
