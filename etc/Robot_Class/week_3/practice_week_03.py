#!/usr/bin/env python3

import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from ultralytics import YOLO

pipeline = rs.pipeline()
config   = rs.config()

pipeline_wrapper = rs.pipeline.wrapper(pipeline)
pipeline_profile = rs.pipeline.resolve(pipeline_wrapper)

device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

pipeline.start(config)

align = rs.align(rs.stream.color)

vis = o3d.visualization.Visualizer()
vis.create_window("Segmented PointCloud")

pcd = o3d.geometry.PointCloud()
added = False

model = YOLO('/home/pc/Robot_Class/week_3/yolo11n-hexagon_week3.pt')

try:
    while True:
        frame = align.process(pipeline.wait_for_frames())
        depth_frame = frame.get_depth_frame()
        color_frame = frame.get_depth_frame()
        
        if not depth_frame or not color_frame:
            continue
        
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(depth_image.get_data())
        
        result = model.predict(source=color_image, task='segment', verbose=False)[0]
        
        if result.masks is not None:
            
        

except KeyboardInterrupt:
    pass

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    vis.destroy_window()
