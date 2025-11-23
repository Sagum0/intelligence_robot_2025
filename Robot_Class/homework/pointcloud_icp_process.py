# -*- coding: utf-8 -*-
import os
os.environ.setdefault("LIBGL_ALWAYS_SOFTWARE", "1")
os.environ.setdefault("OPEN3D_RENDERING_ENGINE", "EGL")
import cv2
import numpy as np
import pyrealsense2 as rs
import open3d as o3d
from ultralytics import YOLO
import copy

# ================== 설정 ==================
YOLO_MODEL_PATH   = '/home/pc/Robot_Class/week_3/yolo11n-hexagon_week3.pt'
TARGET_MODEL_PATH = '/home/pc/Robot_Class/homework/6-angle-40x60.STL'
CONF_THRESHOLD    = 0.7
ICP_VOXEL_SIZE    = 0.01
DEPTH_TRUNC_M     = 3.0
RENDER_W, RENDER_H = 1280, 720
AUTO_FOV_DEG      = 50.0
AUTO_MARGIN       = 1.15
# ==========================================

_renderer_cache = {"rnd": None, "w": None, "h": None}
def get_renderer(w, h):
    from open3d.visualization import rendering
    if _renderer_cache["rnd"] is None or _renderer_cache["w"] != w or _renderer_cache["h"] != h:
        _renderer_cache["rnd"] = rendering.OffscreenRenderer(w, h)
        _renderer_cache["w"], _renderer_cache["h"] = w, h
    return _renderer_cache["rnd"]

def render_items_offscreen_fit(items, w=RENDER_W, h=RENDER_H, bg=[1,1,1,1],
                               fov_deg=AUTO_FOV_DEG, margin=AUTO_MARGIN):
    from open3d.visualization import rendering
    rnd = get_renderer(w, h)
    scn = rnd.scene
    scn.clear_geometry()
    scn.set_background(bg)
    mat = rendering.MaterialRecord(); mat.shader = "defaultUnlit"

    min_b = np.array([ np.inf,  np.inf,  np.inf], float)
    max_b = np.array([-np.inf, -np.inf, -np.inf], float)
    added = 0

    for idx, (name, geom, color) in enumerate(items):
        g = copy.deepcopy(geom)
        if isinstance(g, o3d.geometry.PointCloud) and len(g.points) == 0: 
            continue
        if isinstance(g, o3d.geometry.TriangleMesh) and (not g.has_triangles()): 
            continue
        if color is not None: g.paint_uniform_color(color)
        scn.add_geometry(f"{name}_{idx}", g, mat); added += 1
        bb = g.get_axis_aligned_bounding_box()
        min_b = np.minimum(min_b, np.asarray(bb.get_min_bound(), float))
        max_b = np.maximum(max_b, np.asarray(bb.get_max_bound(), float))
    if added == 0: return np.zeros((h, w, 3), dtype=np.uint8)

    center = (min_b + max_b) / 2.0
    size   = (max_b - min_b) * margin
    r = float(0.5 * np.linalg.norm(size));  r = max(r, 1e-3)

    vfov = np.deg2rad(fov_deg)
    aspect = w / float(h)
    hfov = 2.0 * np.arctan(np.tan(vfov/2.0) * aspect)
    dist = max(r / np.tan(vfov/2.0), r / np.tan(hfov/2.0))

    eye = center + np.array([0, 0, dist])
    up  = np.array([0, -1, 0])
    near = max(1e-3, dist - 2.0*r)
    far  = dist + 2.0*r
    scn.camera.set_projection(fov_deg, aspect, near, far, rendering.Camera.FovType.Vertical)
    scn.camera.look_at(center, eye, up)

    img_o3d = rnd.render_to_image()
    return cv2.cvtColor(np.asarray(img_o3d), cv2.COLOR_RGB2BGR)

def np_img_contig_rgb(color_bgr):
    rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
    return np.ascontiguousarray(rgb, dtype=np.uint8)

def np_depth_contig_u16(depth_raw):
    return np.ascontiguousarray(depth_raw, dtype=np.uint16)

def make_rgbd(color_bgr, depth_u16, depth_scale_m, depth_trunc_m):
    color_o3d = o3d.geometry.Image(np_img_contig_rgb(color_bgr))
    depth_o3d = o3d.geometry.Image(np_depth_contig_u16(depth_u16))
    depth_scale_for_o3d = 1.0 / float(depth_scale_m)
    return o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d, depth_o3d, depth_scale=depth_scale_for_o3d,
        depth_trunc=depth_trunc_m, convert_rgb_to_intensity=False
    )

def rs_align_to_color():
    return rs.align(rs.stream.color)

def centroid(pcd: o3d.geometry.PointCloud):
    pts = np.asarray(pcd.points)
    if pts.size == 0: return np.zeros(3)
    return pts.mean(axis=0)

def initial_translation_T(source_pcd, target_pcd):
    """타겟과 소스의 중심 차이를 이용해 translation만 있는 초기 변환 생성"""
    c_s = centroid(source_pcd)
    c_t = centroid(target_pcd)
    T = np.eye(4)
    T[:3, 3] = c_t - c_s
    return T

def print_pose_matrix(name, T):
    np.set_printoptions(precision=6, suppress=True)
    print(f"[{name}] 4x4 pose matrix:\n{T}")

class YoloICPMatcher:
    def __init__(self, model_path, target_model_path):
        print("Initializing...")
        self.model = YOLO(model_path)
        print("YOLO loaded")

        print(f"Loading STL: {target_model_path}")
        mesh = o3d.io.read_triangle_mesh(target_model_path)
        if not mesh or not mesh.has_triangles():
            raise FileNotFoundError(f"Invalid STL: {target_model_path}")

        # 타겟을 원점 정렬 + mm→m 가정
        c = mesh.get_center()
        mesh.translate(-c)
        mesh.scale(0.001, center=(0,0,0))

        self.target_pcd = mesh.sample_points_poisson_disk(number_of_points=5000)
        self.target_pcd_down = self.target_pcd.voxel_down_sample(ICP_VOXEL_SIZE)
        self.target_pcd_down.estimate_normals(
            o3d.geometry.KDTreeSearchParamHybrid(radius=ICP_VOXEL_SIZE*2, max_nn=30))
        print("Target preprocessed")

        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        profile = self.pipeline.start(cfg)

        self.align = rs_align_to_color()
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale_m = float(depth_sensor.get_depth_scale())

        intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)
        print("RealSense ready")

        self.live_pcd = o3d.geometry.PointCloud()

    def draw_registration_result(self, source, target, T):
        src_t = copy.deepcopy(source); src_t.transform(T)
        img = render_items_offscreen_fit([
            ("target", target, [1.0, 0.2, 0.2]),   # 타깃을 빨간색으로
            ("source", src_t, [0.2, 0.8, 0.2]),    # ICP 결과를 초록색으로
        ])
        cv2.imshow("Result", img); cv2.waitKey(1)

    def run_icp(self, source_pcd):
        if source_pcd is None or not source_pcd.has_points():
            print("[WARN] empty source pcd"); return

        # 1) translation만 적용한 초기치
        T_init = initial_translation_T(source_pcd, self.target_pcd_down)
        print_pose_matrix("T_init (translation only)", T_init)

        # 2) 다운샘플 후 ICP
        src_down = source_pcd.voxel_down_sample(ICP_VOXEL_SIZE)
        thr = ICP_VOXEL_SIZE * 1.5
        reg = o3d.pipelines.registration.registration_icp(
            src_down, self.target_pcd_down, thr, T_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
        )
        T_final = reg.transformation
        print(reg)
        print_pose_matrix("T_final (after ICP)", T_final)

        # 3) 시각화
        self.draw_registration_result(source_pcd, self.target_pcd, T_final)

    def run(self):
        print("Keys: i=ICP, q=quit")
        source_pcd_for_icp = None
        global RENDER_W, RENDER_H
        try:
            while True:
                frames = self.pipeline.wait_for_frames()
                frames = self.align.process(frames)
                d = frames.get_depth_frame(); c = frames.get_color_frame()
                if not d or not c: continue

                depth_raw = np.asanyarray(d.get_data())
                color_bgr = np.asanyarray(c.get_data())

                # YOLO 세그멘트
                res = self.model.predict(source=color_bgr, task='segment', verbose=False)[0]
                mask_bool = np.zeros(depth_raw.shape[:2], bool); found = False
                if res.masks is not None:
                    confs = res.boxes.conf.cpu().numpy()
                    for i, m in enumerate(res.masks.data):
                        if confs[i] < CONF_THRESHOLD: 
                            continue
                        found = True
                        m_np = m.cpu().numpy().astype(np.uint8)
                        m_rs = cv2.resize(m_np, (depth_raw.shape[1], depth_raw.shape[0]),
                                          interpolation=cv2.INTER_NEAREST)
                        mask_bool |= m_rs.astype(bool)

                if found:
                    depth_masked = depth_raw.copy()
                    depth_masked[~mask_bool] = 0
                    kernel = np.ones((2,2), np.uint8)  # 반드시 uint8 배열
                    depth_masked = cv2.erode(depth_masked, kernel, iterations=4)
                    rgbd = make_rgbd(color_bgr, depth_masked, self.depth_scale_m, DEPTH_TRUNC_M)
                    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, self.intrinsic)
                    # 필요 시 좌표계 변환(사용 환경에 맞게 선택)
                    # pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
                    self.live_pcd = pcd
                    source_pcd_for_icp = pcd

                    from open3d.visualization import rendering
                    RENDER_W, RENDER_H = 1280, 720
                    rnd = get_renderer(RENDER_W, RENDER_H)
                    img_o3d = rnd.render_to_image()
                    img = np.asarray(img_o3d)
                    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    scn = rnd.scene

                    live_img = render_items_offscreen_fit([("live", self.live_pcd, None)])
                    
                    cv2.imshow("YOLO Masked PCD (fit)", img_bgr)
                else:
                    self.live_pcd.clear(); source_pcd_for_icp = None
                    cv2.imshow("YOLO Masked PCD (fit)", np.zeros((RENDER_H, RENDER_W, 3), np.uint8))

                cv2.imshow("Live Color", color_bgr)
                k = cv2.waitKey(1) & 0xFF
                if k == ord('q'): break
                if k == ord('i'): self.run_icp(source_pcd_for_icp)
        finally:
            self.pipeline.stop(); cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        YoloICPMatcher(YOLO_MODEL_PATH, TARGET_MODEL_PATH).run()
    except Exception as e:
        print(f"[ERROR] {e}")
