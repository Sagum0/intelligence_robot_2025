#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from ultralytics import YOLO

# ====== 설정 ======
MODEL_PATH = "week_3/yolo11n-hexagon_week3.pt"  # seg 가중치 사용 필수
CONF_THRES = 0.70            # 신뢰도 임계치
DEPTH_TRUNC_M = 10.0         # 장면 거리에 맞게 조정

model = YOLO(MODEL_PATH)

pipeline = rs.pipeline()
config   = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile  = pipeline.start(config)
align    = rs.align(rs.stream.color)

depth_sensor   = profile.get_device().first_depth_sensor()
depth_units_m  = depth_sensor.get_depth_scale()           # meters per unit (e.g., 0.001)
o3d_depth_scale = 1.0 / depth_units_m                     # units per meter (e.g., 1000.0)
print(f"depth_units_m={depth_units_m}, o3d_depth_scale={o3d_depth_scale}")

color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
intr = color_stream.get_intrinsics()  # fx, fy, ppx, ppy, width, height
intrinsics = o3d.camera.PinholeCameraIntrinsic(
    intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy
)

vis = o3d.visualization.Visualizer()
vis.create_window("Segmented PointCloud")

pcd = o3d.geometry.PointCloud()
added = False  # 유효 점군 확보 전까지 geometry 등록 금지

try:
    while True:
        frames = align.process(pipeline.wait_for_frames())
        d = frames.get_depth_frame()
        c = frames.get_color_frame()
        if not d or not c:
            continue

        depth_np = np.asanyarray(d.get_data())      # uint16
        color_np = np.asanyarray(c.get_data())      # BGR (H,W,3)

        # 유효성
        if depth_np.max() == 0:
            continue

        results = model.predict(color_np, verbose=False, conf=CONF_THRES)
        r = results[0]

        # === YOLO 주석 영상 창 ===
        yolo_annot = r.plot()                  # BGR 배열
        cv2.imshow("YOLO Seg", yolo_annot)     # 실시간 감지/세그 결과 확인

        mask_bin = None
        if hasattr(r, "masks") and r.masks is not None and len(r.masks.data) > 0:
            keep = np.arange(len(r.boxes))
            if r.boxes is not None and r.boxes.conf is not None:
                conf = r.boxes.conf.detach().cpu().numpy()
                keep = np.where(conf >= CONF_THRES)[0]
            if keep.size > 0:
                mlist = r.masks.data[keep].detach().cpu().numpy()
                m = (np.sum(mlist, axis=0) > 0.5).astype(np.uint8)
                mask_bin = cv2.resize(m, (color_np.shape[1], color_np.shape[0]),
                                      interpolation=cv2.INTER_NEAREST)

        mask_bin = None
        if hasattr(r, "masks") and r.masks is not None and len(r.masks.data) > 0:
            # conf 필터 반영
            keep = np.arange(len(r.boxes))  # 기본 전체
            if r.boxes is not None and r.boxes.conf is not None:
                conf = r.boxes.conf.detach().cpu().numpy()
                keep = np.where(conf >= CONF_THRES)[0]
            if keep.size > 0:
                mlist = r.masks.data[keep].detach().cpu().numpy()  # [N, Hm, Wm], float 0..1
                m = (np.sum(mlist, axis=0) > 0.5).astype(np.uint8) # 합집합
                mask_bin = cv2.resize(m, (color_np.shape[1], color_np.shape[0]),
                                      interpolation=cv2.INTER_NEAREST)

        if mask_bin is None:
            depth_masked = depth_np
            color_masked = color_np
        else:
            depth_masked = depth_np.copy()
            depth_masked[mask_bin == 0] = 0
            color_masked = color_np.copy()
            color_masked[mask_bin == 0] = 0

        # 디버그 뷰(선택)
        # cv2.imshow("Mask", (mask_bin*255) if mask_bin is not None else np.zeros(depth_np.shape, np.uint8))
        # cv2.imshow("Depth", cv2.applyColorMap(cv2.convertScaleAbs(depth_masked, alpha=0.03), cv2.COLORMAP_JET))

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # ====== RGBD → PointCloud (세그 영역만) ======
        color_rgb = cv2.cvtColor(color_masked, cv2.COLOR_BGR2RGB)
        rgb_img   = o3d.geometry.Image(color_rgb)
        depth_img = o3d.geometry.Image(depth_masked.astype(np.uint16))

        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            rgb_img, depth_img,
            depth_scale=o3d_depth_scale,
            depth_trunc=DEPTH_TRUNC_M,
            convert_rgb_to_intensity=False
        )

        pcd_new = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics)

        # 좌표계 플립
        pcd_new.transform([[1, 0, 0, 0],
                           [0,-1, 0, 0],
                           [0, 0,-1, 0],
                           [0, 0, 0, 1]])

        npts = len(pcd_new.points)
        if npts == 0:
            # 세그 결과가 없거나, 마스크 영역이 모두 depth 0인 경우
            continue

        if not added:
            pcd = pcd_new
            vis.add_geometry(pcd)   # 유효 점군을 얻은 뒤 최초 1회만 등록
            added = True
        else:
            pcd.points = pcd_new.points
            pcd.colors = pcd_new.colors
            vis.update_geometry(pcd)

        vis.poll_events()
        vis.update_renderer()
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    vis.destroy_window()
