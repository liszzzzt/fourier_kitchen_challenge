"""
gr3_compute_side/vision/head_camera.py —— 头部 RGB-D 相机：像素 + 深度 + 关节角 → base_link 下的 3D 坐标
=========================================================================================================
只依赖 numpy + PyYAML（docker 的 Aurora SDK 环境、conda 环境都能用；不需要 MuJoCo / OpenCV / ROS）。

标定结果 config/kitchen_head_camera.yaml 由 kitchen_head_calib 生成（J:\\robo@home\\kitchen_head_calib），里面有：
  * intrinsics          内参 K + 畸变 D（plumb_bob）
  * extrinsic           T_mount_optical：head_pitch_link → 相机光学系（手眼标定结果）
  * kinematics.head_chain   base_link → camera_link / torso_link 路径上的 body 和关节（GR3_V233 模型）
⚠ 只对 kitchen 的 MuJoCo 仿真有效：Aurora SDK GR3_V233 机型 + simulation/sensors/gr3v2_3_3.xml 里的 gr3_head_rgbd（320x240、fovy 60）。
  图像来自 /gr3/camera/color/image_raw 与 /gr3/camera/depth/image_raw（32FC1，单位米，沿光轴的 z 深度，与 RGB 对齐）。
  相机定义变了要重标；真机要用真机数据重新标定。

坐标链：  P_base = FK(q: base_link → head_pitch_link) · T_mount_optical · K⁻¹·(u, v, z)
          q = 腰 3（yaw, roll, pitch）+ 头 2（yaw, pitch），必须和这一帧图像是同一时刻的

用法：
    from gr3_compute_side.vision.head_camera import HeadCamera
    cam = HeadCamera.load()                                   # 默认读 config/kitchen_head_camera.yaml
    x, y, w, h, cls = inference(rgb)                          # vision/main.py 的输出：[x(左), y(上), w, h, cls]
    p_base = cam.bbox_to_base([x, y, w, h], depth, q)        # depth：米，沿光轴的 z 深度，与 rgb 对齐
    command = make_manipulation_command(2, cls, p_base)       # 和 center_control 发的 manipulation_command 同格式
"""
from __future__ import annotations

import os
from typing import Dict, List, Optional, Sequence

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CALIB = os.environ.get("GR3_HEAD_CAMERA_CALIB", os.path.join(HERE, "config", "kitchen_head_camera.yaml"))
WAIST_HEAD = ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint", "head_yaw_joint", "head_pitch_joint")


# ----------------------------------------------------------------------------- 刚体变换（T_a_b：把 b 系的点变到 a 系）
def _T(R=None, t=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if t is not None:
        T[:3, 3] = t
    return T


def _inv(T):
    R, t = T[:3, :3], T[:3, 3]
    return _T(R.T, -R.T @ t)


def _apply(T, P):
    P = np.asarray(P, float)
    return P @ T[:3, :3].T + T[:3, 3]


def _quat_R(q):
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _axis_R(axis, q):
    a = np.asarray(axis, float) / (np.linalg.norm(axis) + 1e-12)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


# ----------------------------------------------------------------------------- 头部运动学链（来自标定 yaml）
class HeadChain:
    def __init__(self, chain: List[dict]):
        self.bodies = {d["name"]: d for d in chain}
        self.root = next(d["name"] for d in chain if d["parent"] is None)

    def _T_root(self, body, q):
        path = []
        while body is not None:
            path.append(body)
            body = self.bodies[body]["parent"]
        T = np.eye(4)
        for name in reversed(path):
            d = self.bodies[name]
            if d["parent"] is not None:
                T = T @ _T(_quat_R(d["quat"]), d["pos"])
            for j in d["joints"]:
                v = float(q.get(j["name"], 0.0))
                if j["type"] == "hinge":
                    R = _axis_R(j["axis"], v)
                    p = np.asarray(j["pos"], float)
                    T = T @ _T(R, p - R @ p)
                else:
                    T = T @ _T(None, np.asarray(j["axis"], float) / np.linalg.norm(j["axis"]) * v)
        return T

    def fk(self, q: Dict[str, float], link: str, base: str = "base_link") -> np.ndarray:
        T = self._T_root(link, q)
        return T if base == self.root else _inv(self._T_root(base, q)) @ T


# ----------------------------------------------------------------------------- 相机
class HeadCamera:
    def __init__(self, K, D, width, height, T_mount_optical, chain: HeadChain, mount_link="head_pitch_link"):
        self.K = np.asarray(K, float)
        self.D = (list(np.asarray(D, float).ravel()) + [0.0] * 5)[:5]
        self.width, self.height = int(width), int(height)
        self.X = np.asarray(T_mount_optical, float)
        self.chain = chain
        self.mount_link = mount_link

    @classmethod
    def load(cls, calib_yaml: str = DEFAULT_CALIB):
        y = yaml.safe_load(open(calib_yaml, encoding="utf-8"))
        c, e = y["intrinsics"], y["extrinsic"]
        return cls(c["camera_matrix"], c["distortion_coefficients"], c["width"], c["height"], e["T_mount_optical"],
                   HeadChain(y["kinematics"]["head_chain"]), e["mount_link"])

    # ---- 位姿
    def T_base_optical(self, q: Dict[str, float]) -> np.ndarray:
        missing = [j for j in WAIST_HEAD if j not in q]
        if missing:
            raise KeyError(f"关节角缺少 {missing}：腰 3 + 头 2 都会影响相机位姿，必须给全")
        return self.chain.fk(q, self.mount_link) @ self.X

    # ---- 去畸变 / 投影
    def _undistort(self, u, v, iters=20):
        k1, k2, p1, p2, k3 = self.D
        xd, yd = (u - self.K[0, 2]) / self.K[0, 0], (v - self.K[1, 2]) / self.K[1, 1]
        x, y = xd.copy(), yd.copy()
        for _ in range(iters if any(abs(d) > 1e-12 for d in self.D) else 0):
            r2 = x * x + y * y
            rad = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
            dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
            dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
            x, y = (xd - dx) / rad, (yd - dy) / rad
        return x, y

    def project_base(self, P_base, q):
        """base_link 下的点 → 像素 (N,2)（画图/检查用）"""
        Pc = _apply(_inv(self.T_base_optical(q)), np.atleast_2d(P_base))
        x, y = Pc[:, 0] / Pc[:, 2], Pc[:, 1] / Pc[:, 2]
        k1, k2, p1, p2, k3 = self.D
        r2 = x * x + y * y
        rad = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
        xd, yd = x * rad + 2 * p1 * x * y + p2 * (r2 + 2 * x * x), y * rad + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
        return np.stack([self.K[0, 0] * xd + self.K[0, 2], self.K[1, 1] * yd + self.K[1, 2]], -1)

    # ---- 反投影
    def pixel_to_base(self, u, v, z, q) -> np.ndarray:
        """像素 (u=列, v=行) + z 深度（米）→ base_link 下 (N,3)"""
        u, v, z = (np.atleast_1d(np.asarray(a, float)).ravel() for a in (u, v, z))
        x, y = self._undistort(u, v)
        return _apply(self.T_base_optical(q), np.stack([x * z, y * z, z], -1))

    def depth_to_base(self, depth: np.ndarray, q, mask: Optional[np.ndarray] = None) -> np.ndarray:
        valid = depth > 0
        if mask is not None:
            valid &= mask
        v, u = np.nonzero(valid)
        return self.pixel_to_base(u, v, depth[v, u], q)

    def bbox_to_base(self, bbox: Sequence[float], depth: np.ndarray, q, shrink: float = 0.25,
                     mask: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        检测框 [x(左), y(上), w, h] → 框中心区域（四周各去掉 shrink）点云的中位数（base_link，米）。
        这是“可见表面”的中心，会偏向相机一侧；抓取前按物体几何补偿（比如方块：上表面中心 − 半边长）。
        """
        x, y, w, h = [float(t) for t in bbox[:4]]
        x0, x1 = max(int(round(x + shrink * w)), 0), int(round(x + (1 - shrink) * w))
        y0, y1 = max(int(round(y + shrink * h)), 0), int(round(y + (1 - shrink) * h))
        x1, y1 = min(max(x1, x0 + 1), self.width), min(max(y1, y0 + 1), self.height)
        roi = np.zeros(depth.shape, bool)
        roi[y0:y1, x0:x1] = True
        if mask is not None:
            roi &= mask
        P = self.depth_to_base(depth, q, roi)
        return None if len(P) < 5 else np.median(P, axis=0)

    # ---- 其它坐标系
    def base_to_link(self, p_base, link: str, q) -> np.ndarray:
        """base_link → 链上某个 link（目前 yaml 里有 torso_link / 头部各 link）"""
        return _apply(_inv(self.chain.fk(q, link)), p_base)

    @staticmethod
    def base_to_world(p_base, T_world_base) -> np.ndarray:
        return _apply(np.asarray(T_world_base, float), p_base)


def q_from_aurora(client) -> Dict[str, float]:
    """Aurora：控制组 waist = [yaw, roll, pitch]，head = [yaw, pitch]（GR3 hardware.json）"""
    w = client.get_group_state("waist", "position")
    h = client.get_group_state("head", "position")
    return dict(waist_yaw_joint=w[0], waist_roll_joint=w[1], waist_pitch_joint=w[2],
                head_yaw_joint=h[0], head_pitch_joint=h[1])


def make_manipulation_command(command_id, target: str, p_base, rpy=(0.0, 0.0, 0.0)) -> dict:
    """和 center_control.launch() 里 manipulation_command 相同的字段，坐标系改成 base_link"""
    return {"id": command_id, "target": target, "frame_id": "base_link",
            "coordinate": [round(float(v), 4) for v in p_base] + [float(a) for a in rpy]}


if __name__ == "__main__":     # 自测：不需要仿真，只检查标定文件能读、投影/反投影一致
    cam = HeadCamera.load()
    q = dict(waist_yaw_joint=0.0, waist_roll_joint=0.0, waist_pitch_joint=0.45, head_yaw_joint=0.0,
             head_pitch_joint=0.12)
    p = cam.pixel_to_base([320.0, 100.0], [240.0, 400.0], [0.9, 0.7], q)
    uv = cam.project_base(p, q)
    print("base_link 点：\n", np.round(p, 4))
    print("再投影回像素：\n", np.round(uv, 4), "（应为 [[320,240],[100,400]]）")
    assert np.allclose(uv, [[320, 240], [100, 400]], atol=1e-6)
    print(make_manipulation_command(2, "cube", p[0]))
    print("OK")
