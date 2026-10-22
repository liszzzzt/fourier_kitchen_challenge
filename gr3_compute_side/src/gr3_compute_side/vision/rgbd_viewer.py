"""ROS 2 RGB-D subscriber and optional OpenCV diagnostic viewer."""

import threading
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image


class RGBDCameraViewer(Node):
    """Receive synchronized RGB-D images without blocking compute-side input."""

    RGB_TOPIC = "/gr3/camera/color/image_raw"
    DEPTH_TOPIC = "/gr3/camera/depth/image_raw"
    RGB_WINDOW = "GR3 RGB Camera"
    DEPTH_WINDOW = "GR3 Depth Camera"

    def __init__(
        self,
        minimum_depth: float = 0.2,
        maximum_depth: float = 10.0,
    ):
        super().__init__("gr3_rgbd_viewer")

        if minimum_depth < 0.0:
            raise ValueError("minimum_depth cannot be negative")
        if maximum_depth <= minimum_depth:
            raise ValueError("maximum_depth must be greater than minimum_depth")

        self.minimum_depth = minimum_depth
        self.maximum_depth = maximum_depth

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.rgb_subscription = self.create_subscription(
            Image,
            self.RGB_TOPIC,
            self._rgb_callback,
            image_qos,
        )
        self.depth_subscription = self.create_subscription(
            Image,
            self.DEPTH_TOPIC,
            self._depth_callback,
            image_qos,
        )

        self._rgb_frames: Dict[int, np.ndarray] = {}
        self._depth_frames: Dict[int, np.ndarray] = {}
        self._maximum_pending_frames = 4

        self._frame_lock = threading.Lock()
        self._latest_pair: Optional[Tuple[np.ndarray, np.ndarray]] = None

        self._display_enabled = threading.Event()
        self._stop_requested = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._windows_open = False

    def start(self) -> None:
        """Start receiving ROS messages in a background thread."""
        if self._worker is not None:
            return

        self._worker = threading.Thread(
            target=self._run,
            name="gr3-rgbd-viewer",
            daemon=True,
        )
        self._worker.start()

    def enable_display(self) -> None:
        """Enable the RGB and depth diagnostic windows."""
        self._display_enabled.set()

    def disable_display(self) -> None:
        """Disable the diagnostic windows without stopping subscriptions."""
        self._display_enabled.clear()

    def get_latest_pair(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Return copies of the newest synchronized RGB and depth arrays."""
        with self._frame_lock:
            if self._latest_pair is None:
                return None
            rgb, depth = self._latest_pair
            return rgb.copy(), depth.copy()

    def _run(self) -> None:
        """Process subscriptions and OpenCV events on one worker thread."""
        executor = SingleThreadedExecutor(context=self.context)
        executor.add_node(self)
        try:
            while (
                not self._stop_requested.is_set()
                and rclpy.ok(context=self.context)
            ):
                executor.spin_once(timeout_sec=0.02)

                if self._display_enabled.is_set():
                    self._show_latest_pair()
                elif self._windows_open:
                    self._close_windows()
        finally:
            self._close_windows()
            executor.remove_node(self)
            executor.shutdown(timeout_sec=1.0)

    def _rgb_callback(self, message: Image) -> None:
        if message.encoding.lower() != "rgb8":
            self.get_logger().error(
                f"Unsupported RGB encoding: {message.encoding}"
            )
            return

        expected_size = message.height * message.step
        raw = np.frombuffer(message.data, dtype=np.uint8)
        if raw.size < expected_size or message.step < message.width * 3:
            self.get_logger().error("Invalid RGB image buffer")
            return

        rows = raw[:expected_size].reshape(message.height, message.step)
        rgb = rows[:, :message.width * 3].reshape(
            message.height,
            message.width,
            3,
        ).copy()

        timestamp = self._timestamp_key(message)
        self._rgb_frames[timestamp] = rgb
        self._accept_pair_if_ready(timestamp)
        self._trim_pending_frames(self._rgb_frames)

    def _depth_callback(self, message: Image) -> None:
        if message.encoding.upper() != "32FC1":
            self.get_logger().error(
                f"Unsupported depth encoding: {message.encoding}"
            )
            return

        bytes_per_pixel = np.dtype(np.float32).itemsize
        if message.step != message.width * bytes_per_pixel:
            self.get_logger().error("Unsupported padded depth image")
            return

        expected_values = message.height * message.width
        byte_order = ">f4" if message.is_bigendian else "<f4"
        raw = np.frombuffer(message.data, dtype=byte_order)
        if raw.size < expected_values:
            self.get_logger().error("Invalid depth image buffer")
            return

        depth = raw[:expected_values].reshape(
            message.height,
            message.width,
        ).astype(np.float32, copy=True)

        timestamp = self._timestamp_key(message)
        self._depth_frames[timestamp] = depth
        self._accept_pair_if_ready(timestamp)
        self._trim_pending_frames(self._depth_frames)

    @staticmethod
    def _timestamp_key(message: Image) -> int:
        return (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )

    def _accept_pair_if_ready(self, timestamp: int) -> None:
        rgb = self._rgb_frames.get(timestamp)
        depth = self._depth_frames.get(timestamp)
        if rgb is None or depth is None:
            return

        with self._frame_lock:
            self._latest_pair = (rgb, depth)

        del self._rgb_frames[timestamp]
        del self._depth_frames[timestamp]

    def _trim_pending_frames(self, frames: Dict[int, np.ndarray]) -> None:
        while len(frames) > self._maximum_pending_frames:
            oldest_timestamp = next(iter(frames))
            del frames[oldest_timestamp]

    def _show_latest_pair(self) -> None:
        with self._frame_lock:
            pair = self._latest_pair

        if pair is None:
            return

        rgb, depth = pair
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        depth_visualization = self._colorize_depth(depth)

        cv2.imshow(self.RGB_WINDOW, bgr)
        cv2.imshow(self.DEPTH_WINDOW, depth_visualization)
        self._windows_open = True
        cv2.waitKey(1)

    def _colorize_depth(self, depth: np.ndarray) -> np.ndarray:
        valid = np.isfinite(depth) & (depth > 0.0)
        clipped = np.clip(depth, self.minimum_depth, self.maximum_depth)

        normalized = np.zeros(depth.shape, dtype=np.uint8)
        scale = 255.0 / (self.maximum_depth - self.minimum_depth)
        normalized[valid] = (
            (clipped[valid] - self.minimum_depth) * scale
        ).astype(np.uint8)

        colorized = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        colorized[~valid] = 0
        return colorized

    def _close_windows(self) -> None:
        if not self._windows_open:
            return

        for window_name in (self.RGB_WINDOW, self.DEPTH_WINDOW):
            try:
                cv2.destroyWindow(window_name)
            except cv2.error:
                pass
        cv2.waitKey(1)
        self._windows_open = False

    def close(self) -> None:
        """Stop subscriptions and destroy this node.

        Call this before ``rclpy.shutdown()``.
        """
        worker = self._worker
        if worker is not None:
            self._stop_requested.set()
            worker.join()
            self._worker = None

        self.destroy_node()
