import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

theta_sample_num = 100
theta = np.linspace(0.0, 2 * np.pi, theta_sample_num)

r = 1.0
x = r * np.cos(theta)
y = r * np.sin(theta)
z = np.zeros_like(x)
target = np.stack([x, y, z], axis=-1)

def pcd_rotate(points, rx_deg, ry_deg, rz_deg):
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(rx), -np.sin(rx)],
                   [0, np.sin(rx), np.cos(rx)]])
    
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                   [0, 1, 0],
                   [-np.sin(ry), 0, np.cos(ry)]])
    
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                   [np.sin(rz), np.cos(rz), 0],
                   [0, 0, 1]])
    
    R = Rz @ Ry @ Rx
    return (R @ points.T).T

source = pcd_rotate(target, 110.0, 25.0, 10.0)
source += np.array([0.4, -0.5, 0.2])
source += np.random.normal(scale=0.01, size=source.shape)
np.random.shuffle(source)

def ICP(source, target):
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)

    source_center = source - source_mean
    target_center = target - target_mean

    H = source_center.T @ target_center
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[2] *= -1
        R = Vt.T @ U.T

    t = target_mean - R @ source_mean

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

src = source.copy()
iter = 15

def apply_transform(pts, T):
    pts_h = np.hstack([pts, np.ones((pts.shape[0], 1))])   # (N,4)
    return (T @ pts_h.T).T[:, :3]

for i in range(1, iter + 1):
    T = ICP(src, target)
    src = apply_transform(src, T)
    mean_norm = np.linalg.norm(src - target, axis=1).mean()
    print(f"Iter {i}: mean norm = {mean_norm:.6f}")