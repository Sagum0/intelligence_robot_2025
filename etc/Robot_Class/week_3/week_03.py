import pyrealsense2 as rs
import open3d as o3d
import cv2
import numpy as np
import time

pipeline = rs.pipeline()
config   = rs.config()

pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)

device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()

depth_image = np.asanyarray(depth_frame.get_data())
color_image = np.asanyarray(color_frame.get_data())

timestamp = int(time.time())
depth_img_path = f"depth_{timestamp}.png"
color_img_path = f"color_{timestamp}.png"
cv2.imwrite(depth_img_path, depth_image)
cv2.imwrite(color_img_path, color_image)

rgb_img = o3d.io.read_image(color_img_path)
depth_img = o3d.io.read_image(depth_img_path)

rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    color=rgb_img,
    depth=depth_img,
    depth_scale=1000.0, # RealSense는 mm 단위 → m 단위로 바꾸기 위해 1000
    depth_trunc=3.0, # 3m 이상은 무시
    convert_rgb_to_intensity=False
)

pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
    rgbd_image,
    o3d.camera.PinholeCameraIntrinsic(
    o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))

# Flip it, otherwise the pointcloud will be upside down
pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
o3d.visualization.draw_geometries([pcd])