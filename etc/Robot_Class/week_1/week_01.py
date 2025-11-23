import open3d as o3d
import numpy as np

# ---------- 1. 타깃 점군 생성 ----------
theta = np.linspace(0, 2*np.pi, 100)
target = np.stack([np.cos(theta), np.sin(theta), np.zeros_like(theta)], axis=1)

# ---------- 2. 올바른 회전 함수 ----------
def pcd_rotate(pts, rx_deg, ry_deg, rz_deg):
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx),  np.cos(rx)]])
    Ry = np.array([[ np.cos(ry), 0,  np.sin(ry)],
                   [0,           1,           0],
                   [-np.sin(ry), 0,  np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz),  np.cos(rz), 0],
                   [0,          0,          1]])
    return (Rz @ Ry @ Rx @ pts.T).T

# ---------- 3. 소스 점군(노이즈·변환 포함) ----------
source = pcd_rotate(target, 110, 25, 10)
source += np.array([2, -5, 0.2])
source += np.random.normal(0, 0.01, source.shape)
np.random.shuffle(source)

# ---------- 4. ICP 한 스텝 ----------
def nearest_with_o3d(src, dst_pts):
    dst_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(dst_pts))
    kdtree  = o3d.geometry.KDTreeFlann(dst_pcd)
    idx = [kdtree.search_knn_vector_3d(p, 1)[1][0] for p in src]
    return dst_pts[np.array(idx)]

def icp_once(src, dst):
    match = nearest_with_o3d(src, dst)       # 최근접 대응
    s_c, d_c = src.mean(0), match.mean(0)

    H = (src - s_c).T @ (match - d_c)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:                 # 반사행렬 교정
        Vt[2] *= -1
        R = Vt.T @ U.T
    t = d_c - R @ s_c
    new_src = (R @ src.T).T + t
    err = np.linalg.norm(new_src - match, axis=1).mean()
    return new_src, err

# ---------- 5. 반복 루프 ----------
src = source.copy()
max_iter, eps = 20, 1e-5
for i in range(1, max_iter+1):
    src, mean_err = icp_once(src, target)
    print(f"Iter {i:2d}: mean err = {mean_err:.6f}")
    if mean_err < eps:
        print("Converged!")
        break
