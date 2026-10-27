"""
vision/rgbd_localizer.py —— 用头部相机标定结果，把检测框变成 base_link 坐标（ROS 2 节点）
==========================================================================================
订阅三个话题：
    /gr3/camera/color/image_raw    sensor_msgs/Image  rgb8
    /gr3/camera/depth/image_raw    sensor_msgs/Image  32FC1（米，沿光轴的 z 深度）
    /gr3/joint_calib_states        sensor_msgs/JointState  腰 3 + 头 2 的实测关节角（robot_side 从 Aurora 关节反馈读，50 Hz；
                                   真机上就是编码器读数，不用仿真的 qpos，免得和真机差太多）
RGB 和深度按 header.stamp（拍摄时刻）配成一对；关节角是另一路数据流，按图像的时间戳在前后两个读数之间线性插值
（最近的读数离图像超过 MAX_JOINT_DT 就不用这一帧）。
为什么要关节角：相机装在头上，相机在 base_link 下的位姿 = FK(腰 3 + 头 2) · 标定外参，
关节角必须和这一帧图像是同一时刻的，否则头一转，坐标就错了。
两路时间戳要是同一个时钟：仿真里图像和 robot_side 在同一台机器上；真机上相机驱动和 robot_side 要做时钟同步。

    localizer = RGBDLocalizer(); localizer.start()
    frame = localizer.get_latest_frame(timeout=2.0)          # (rgb, depth, q, stamp_ns) 或 None
    det = inference(frame[0])                                # [x(左), y(上), w, h, cls]
    command = localizer.make_pick_command(2, det, frame)     # {"id", "target", "frame_id": "base_link", "coordinate"}
    localizer.close()

只依赖 numpy + rclpy + sensor_msgs（不需要 OpenCV / MuJoCo）；标定数学在同目录的 head_camera.py。
"""

import bisect
import threading
import time
from collections import deque
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

from gr3_compute_side.vision.head_camera import DEFAULT_CALIB
from gr3_compute_side.vision.head_camera import HeadCamera
from gr3_compute_side.vision.head_camera import WAIST_HEAD
from gr3_compute_side.vision.head_camera import make_manipulation_command

Frame = Tuple[np.ndarray, np.ndarray, Dict[str, float], int]


class RGBDLocalizer(Node):
    RGB_TOPIC = "/gr3/camera/color/image_raw"
    DEPTH_TOPIC = "/gr3/camera/depth/image_raw"
    JOINT_TOPIC = "/gr3/joint_calib_states"
    MAX_JOINT_DT = 0.05              # s：图像时刻前后最近的关节读数离它最多这么远
    JOINT_BUFFER = 200               # 关节读数缓存（50 Hz → 4 s）

    def __init__(self, calib_yaml: str = DEFAULT_CALIB, max_pending: int = 6):
        super().__init__("gr3_rgbd_localizer")
        self.camera = HeadCamera.load(calib_yaml)
        self._max_pending = max_pending
        self._parts: Dict[int, Dict[str, object]] = {}          # stamp → {"rgb", "depth"}
        self._joints: deque = deque(maxlen=self.JOINT_BUFFER)    # (stamp_ns, {关节名: rad})，按时间排好
        self._waiting: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}   # 图像比最新的关节读数还新：等关节读数到了再配
        self._lock = threading.Lock()
        self._latest: Optional[Frame] = None
        self._new_frame = threading.Event()
        self._stop = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._warned_size = False

        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        joint_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Image, self.RGB_TOPIC, self._rgb_callback, qos)
        self.create_subscription(Image, self.DEPTH_TOPIC, self._depth_callback, qos)
        self.create_subscription(JointState, self.JOINT_TOPIC, self._joint_callback, joint_qos)

    # ------------------------------------------------------------------ 线程
    def start(self) -> None:
        if self._worker is None:
            self._worker = threading.Thread(target=self._run, name="gr3-rgbd-localizer", daemon=True)
            self._worker.start()

    def _run(self) -> None:
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self)
        try:
            while not self._stop.is_set() and rclpy.ok(context=self.context):
                executor.spin_once(timeout_sec=0.02)
        finally:
            executor.remove_node(self)
            executor.shutdown(timeout_sec=1.0)

    def close(self) -> None:
        """在 rclpy.shutdown() 之前调用"""
        if self._worker is not None:
            self._stop.set()
            self._worker.join()
            self._worker = None
        self.destroy_node()

    # ------------------------------------------------------------------ 对外接口
    def get_latest_frame(self, timeout: float = 0.0, newer_than_ns: int = -1) -> Optional[Frame]:
        """最新的一组 (rgb, depth, q, stamp_ns)；timeout > 0 时最多等这么久"""
        deadline = time.monotonic() + max(timeout, 0.0)
        while True:
            with self._lock:
                f = self._latest
            if f is not None and f[3] > newer_than_ns:
                rgb, depth, q, stamp = f
                return rgb.copy(), depth.copy(), dict(q), stamp
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._new_frame.wait(min(remaining, 0.1))
            self._new_frame.clear()

    def locate(self, bbox, frame: Frame, mask: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """检测框 [x(左), y(上), w, h] → base_link 下目标可见表面的中心（米）；没有有效深度返回 None"""
        _, depth, q, _ = frame
        return self.camera.bbox_to_base(bbox, depth, q, mask=mask)

    def make_pick_command(self, command_id, det, frame: Frame, object_half_height: float = 0.0) -> Optional[dict]:
        """det = inference() 的输出 [x, y, w, h, cls]；看到的是表面，抓取点往下补 object_half_height（按类别查表）"""
        p = self.locate(det[:4], frame)
        if p is None:
            return None
        p = p - np.array([0.0, 0.0, object_half_height])
        return make_manipulation_command(command_id, det[4], p)

    def locate_box(self, det, frame: Frame, half: float, mask: Optional[np.ndarray] = None):
        """方块：框内可见点云（base_link）→ 中心 + 绕竖直轴的朝向（模 90°）。
        直接取点云中位数会偏向相机一侧（侧面也看得见）；取上表面那层点的 xy 中位数，z = 上表面 − 半边长；
        朝向 = 上表面点的最小外接矩形。返回 (p_base, yaw) 或 None"""
        rgb, depth, q, _ = frame
        x, y, w, h = det[:4]
        roi = np.zeros(depth.shape, bool)
        roi[y:y + h, x:x + w] = True
        if mask is not None:
            roi &= mask
        P = self.camera.depth_to_base(depth, q, roi)
        if len(P) < 20:
            return None
        z_top = np.percentile(P[:, 2], 95)
        top = P[P[:, 2] > z_top - 0.006]
        c = np.array([np.median(top[:, 0]), np.median(top[:, 1]), np.median(top[:, 2]) - half])
        best = None
        for a in np.deg2rad(np.arange(0, 90, 2)):
            R = np.array([[np.cos(a), np.sin(a)], [-np.sin(a), np.cos(a)]])
            uv = (top[:, :2] - c[:2]) @ R.T
            area = np.ptp(uv[:, 0]) * np.ptp(uv[:, 1])
            if best is None or area < best[0]:
                best = (area, a)
        return c, float(best[1])

    # ------------------------------------------------------------------ 回调
    @staticmethod
    def _stamp(msg) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _check_size(self, msg) -> bool:
        if msg.width == self.camera.width and msg.height == self.camera.height:
            return True
        if not self._warned_size:
            self.get_logger().error(
                f"图像 {msg.width}x{msg.height} 与标定分辨率 {self.camera.width}x{self.camera.height} 不一致：要重新标定")
            self._warned_size = True
        return False

    def _rgb_callback(self, msg: Image) -> None:
        if msg.encoding.lower() != "rgb8" or not self._check_size(msg):
            return
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < msg.height * msg.step or msg.step < msg.width * 3:
            return
        rgb = raw[:msg.height * msg.step].reshape(msg.height, msg.step)[:, :msg.width * 3]
        self._put(self._stamp(msg), "rgb", rgb.reshape(msg.height, msg.width, 3).copy())

    def _depth_callback(self, msg: Image) -> None:
        if msg.encoding.upper() != "32FC1" or not self._check_size(msg) or msg.step != msg.width * 4:
            return
        raw = np.frombuffer(msg.data, dtype=">f4" if msg.is_bigendian else "<f4")
        if raw.size < msg.height * msg.width:
            return
        depth = raw[:msg.height * msg.width].reshape(msg.height, msg.width).astype(np.float32, copy=True)
        depth[~np.isfinite(depth)] = 0.0
        self._put(self._stamp(msg), "depth", depth)

    def _joint_callback(self, msg: JointState) -> None:
        q = dict(zip(msg.name, (float(v) for v in msg.position)))
        if not all(j in q for j in WAIST_HEAD):
            return
        stamp = self._stamp(msg)
        with self._lock:
            if self._joints and stamp <= self._joints[-1][0]:
                return                                             # 乱序 / 重复的读数不要
            self._joints.append((stamp, {j: q[j] for j in WAIST_HEAD}))
            for s in sorted(self._waiting):                       # 之前在等关节读数的图像
                rgb, depth = self._waiting[s]
                if self._try_complete(s, rgb, depth):
                    del self._waiting[s]

    def _joints_at(self, stamp: int) -> Optional[Dict[str, float]]:
        """图像时刻的关节角：前后两个读数线性插值；只有一边、且离得不超过 MAX_JOINT_DT 时用那一个；否则 None"""
        stamps = [s for s, _ in self._joints]
        i = bisect.bisect_left(stamps, stamp)
        max_dt = int(self.MAX_JOINT_DT * 1e9)
        before = self._joints[i - 1] if i > 0 else None
        after = self._joints[i] if i < len(stamps) else None
        if before is not None and after is not None and after[0] - before[0] <= 2 * max_dt:
            f = (stamp - before[0]) / max(after[0] - before[0], 1)
            return {j: before[1][j] + f * (after[1][j] - before[1][j]) for j in WAIST_HEAD}
        for near in (before, after):
            if near is not None and abs(near[0] - stamp) <= max_dt:
                return dict(near[1])
        return None

    def _try_complete(self, stamp: int, rgb: np.ndarray, depth: np.ndarray) -> bool:
        """（在锁里调用）给一对 RGB + 深度配上关节角；关节读数还没跟上就返回 False（继续等）"""
        if not self._joints or self._joints[-1][0] < stamp:
            return False
        q = self._joints_at(stamp)
        if q is not None and (self._latest is None or stamp > self._latest[3]):
            self._latest = (rgb, depth, q, stamp)
            self._new_frame.set()
        return True                                                # 配上了，或者确定配不上（这帧不用）

    def _put(self, stamp: int, key: str, value) -> None:
        with self._lock:
            parts = self._parts.setdefault(stamp, {})
            parts[key] = value
            if len(parts) == 2:
                del self._parts[stamp]
                for old in [s for s in self._parts if s < stamp]:      # 更早的不完整组不会再用了
                    del self._parts[old]
                if not self._try_complete(stamp, parts["rgb"], parts["depth"]):
                    self._waiting[stamp] = (parts["rgb"], parts["depth"])
                    while len(self._waiting) > self._max_pending:
                        del self._waiting[min(self._waiting)]
            while len(self._parts) > self._max_pending:
                del self._parts[min(self._parts)]
