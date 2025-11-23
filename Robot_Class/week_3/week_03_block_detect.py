import pyrealsense2 as rs
import numpy as np  
import cv2
import os, sys
from ultralytics import YOLO

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))


pipeline = rs.pipeline()
config   = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)

device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

model = YOLO('/home/pc/Robot_Class/week_3/yolo11n-hexagon_week3.pt')

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        streo1_frame = frames.get_infrared_frame(1)
        streo2_frame = frames.get_infrared_frame(2)
        
        if not depth_frame or not color_frame:
            continue
        
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
        results = model.track(color_image, persist=True)
        annotated_frame = results[0].plot()
        
        images = np.hstack((color_image, depth_colormap))
        # color_path = os.path.join(OUT_DIR, f"color_{ts}.png")
        # cv2.imshow('Realsense', images)
        # cv2.imshow('YOLO 11 Tracking', annotated_frame)
        
        if results and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            # confidence ≥ 0.9만 선택
            mask = boxes.conf.cpu().numpy() >= 0.94
            boxes = boxes[mask]
            results[0].boxes = boxes

            annotated_frame = results[0].plot()
            cv2.imshow('YOLO 11 Tracking', annotated_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()