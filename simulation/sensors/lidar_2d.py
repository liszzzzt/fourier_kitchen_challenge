"""Asynchronous 2D LiDAR simulation for the GR3 torso sensor.

The physics thread submits copies of the latest MuJoCo state. A worker owns a
private MjData, casts horizontal rays from the LiDAR site, and publishes the
raw scan. At most one pending state is retained, so stale scans never form an
unbounded queue.
"""

import math
import random
import threading
import time
from typing import Optional

import mujoco
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class Lidar2D:
    """Cast 2D MuJoCo rays and publish them as a ROS LaserScan."""

    def __init__(
        self,
        model: mujoco.MjModel,
        site_name: str = "gr3_torso_lidar_2d",
        frame_id: str = "gr3_torso_lidar_2d_link",
        frequency_hz: float = 5.0,
        field_of_view_degrees: float = 270.0,
        angular_resolution_degrees: float = 0.5,
        range_min: float = 0.10,
        range_max: float = 10.0,
        # 0.03 seconds
        latency_mean_s: float = 0.0,
        # 0.01 seconds
        latency_jitter_s: float = 0.0,
    ):
        if frequency_hz <= 0:
            raise ValueError("LiDAR frequency must be positive")
        if field_of_view_degrees <= 0 or field_of_view_degrees > 360:
            raise ValueError("LiDAR field of view must be in (0, 360]")
        if angular_resolution_degrees <= 0:
            raise ValueError("LiDAR angular resolution must be positive")
        if range_min < 0 or range_max <= range_min:
            raise ValueError("LiDAR range limits are invalid")
        if latency_mean_s < 0 or latency_jitter_s < 0:
            raise ValueError("LiDAR latency values must be non-negative")

        site_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            site_name,
        )
        if site_id < 0:
            raise ValueError(f"MuJoCo site {site_name!r} was not found")

        self.model = model
        self.site_name = site_name
        self.site_id = site_id
        self.frame_id = frame_id
        self.capture_period = 1.0 / frequency_hz
        self.range_min = range_min
        self.range_max = range_max

        self.latency_mean_s = latency_mean_s
        self.latency_jitter_s = latency_jitter_s

        field_of_view = math.radians(field_of_view_degrees)
        requested_increment = math.radians(angular_resolution_degrees)
        self.ray_count = int(round(field_of_view / requested_increment)) + 1
        self.angle_min = -field_of_view / 2.0
        self.angle_max = field_of_view / 2.0
        self.angle_increment = field_of_view / (self.ray_count - 1)
        self.local_directions = self._make_local_directions()

        # Raycast against collision geometry used by this model and kitchen.
        # Robot geometry is intentionally included: this is a raw sensor model.
        self.geom_groups = np.array([1, 0, 0, 1, 1, 0], dtype=np.uint8)

        self.last_submitted_time: Optional[float] = None
        self._condition = threading.Condition()
        self._pending_state = None
        self._stop_requested = False
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._worker_error: Optional[Exception] = None

    def _make_local_directions(self) -> np.ndarray:
        """Create unit rays in the LiDAR site's local XY plane."""
        angles = self.angle_min + np.arange(self.ray_count) * self.angle_increment
        directions = np.zeros((self.ray_count, 3), dtype=np.float64)
        directions[:, 0] = np.cos(angles)
        directions[:, 1] = np.sin(angles)
        return directions

    def start(self) -> None:
        """Start the scan worker."""
        if self._thread is not None:
            return

        self._thread = threading.Thread(
            target=self._scan_loop,
            name="gr3-lidar-2d",
            daemon=True,
        )
        self._thread.start()

    def submit_state_if_due(self, data: mujoco.MjData) -> bool:
        """Submit the newest state when the 5 Hz scan period is due."""
        if self._thread is None:
            raise RuntimeError("LiDAR worker has not been started")
        if self._worker_error is not None:
            return False

        simulation_time = float(data.time)
        if (
            self.last_submitted_time is not None
            and simulation_time < self.last_submitted_time
        ):
            self.last_submitted_time = None

        if (
            self.last_submitted_time is not None
            and simulation_time - self.last_submitted_time < self.capture_period
        ):
            return False

        state = (
            simulation_time,
            time.time_ns(),
            data.qpos.copy(),
            data.qvel.copy(),
            data.act.copy(),
            data.mocap_pos.copy(),
            data.mocap_quat.copy(),
            data.userdata.copy(),
        )

        with self._condition:
            self._pending_state = state
            self._condition.notify()

        self.last_submitted_time = simulation_time
        return True

    def _scan_loop(self) -> None:
        """Generate and publish scans from private MuJoCo state."""
        ros_context = None
        ros_node = None
        try:
            ros_context = Context()
            rclpy.init(context=ros_context)
            ros_node = Node("gr3_lidar_2d", context=ros_context)

            scan_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            publisher = ros_node.create_publisher(
                LaserScan,
                "/gr3/mujoco_raw/lidar/scan",
                scan_qos,
            )
            scan_data = mujoco.MjData(self.model)

            while True:
                with self._condition:
                    while (
                        self._pending_state is None
                        and not self._stop_requested
                    ):
                        self._condition.wait()

                    if self._stop_requested:
                        break

                    state = self._pending_state
                    self._pending_state = None

                (
                    simulation_time,
                    acquisition_time_ns,
                    qpos,
                    qvel,
                    act,
                    mocap_pos,
                    mocap_quat,
                    userdata,
                ) = state

                scan_data.time = simulation_time
                scan_data.qpos[:] = qpos
                scan_data.qvel[:] = qvel
                scan_data.act[:] = act
                scan_data.mocap_pos[:] = mocap_pos
                scan_data.mocap_quat[:] = mocap_quat
                scan_data.userdata[:] = userdata
                mujoco.mj_forward(self.model, scan_data)

                ranges = self._cast_scan(scan_data)
                if self._stop_event.wait(self._sample_latency()):
                    break
                self._publish_scan(
                    ros_node,
                    publisher,
                    ranges,
                    acquisition_time_ns,
                )
        except Exception as error:  # Keep the main simulation alive.
            self._worker_error = error
            print(
                f"LiDAR worker failed: {type(error).__name__}: {error}",
                flush=True,
            )
        finally:
            if ros_node is not None:
                ros_node.destroy_node()
            if ros_context is not None and ros_context.ok():
                rclpy.shutdown(context=ros_context)

    def _cast_scan(self, data: mujoco.MjData) -> np.ndarray:
        """Cast one raw horizontal scan, including robot-body returns."""
        origin = data.site_xpos[self.site_id].copy()
        rotation = data.site_xmat[self.site_id].reshape(3, 3)
        ranges = np.full(self.ray_count, np.inf, dtype=np.float32)
        geom_id = np.array([-1], dtype=np.int32)

        for index in range(self.ray_count):
            direction = rotation @ self.local_directions[index]
            geom_id[0] = -1
            distance = mujoco.mj_ray(
                self.model,
                data,
                origin,
                direction,
                self.geom_groups,
                1,
                -1,
                geom_id,
            )
            if self.range_min <= distance <= self.range_max:
                ranges[index] = distance

        return ranges

    def _publish_scan(
        self,
        ros_node: Node,
        publisher,
        ranges: np.ndarray,
        acquisition_time_ns: int,
    ) -> None:
        """Publish one LaserScan stamped at acquisition time."""
        message = LaserScan()
        message.header.stamp = ros_node.get_clock().now().to_msg()
        message.header.stamp.sec = acquisition_time_ns // 1_000_000_000
        message.header.stamp.nanosec = acquisition_time_ns % 1_000_000_000
        message.header.frame_id = self.frame_id
        message.angle_min = self.angle_min
        message.angle_max = self.angle_max
        message.angle_increment = self.angle_increment
        message.time_increment = 0.0
        message.scan_time = self.capture_period
        message.range_min = self.range_min
        message.range_max = self.range_max
        message.ranges = ranges.tolist()
        message.intensities = []
        publisher.publish(message)

    def _sample_latency(self) -> float:
        """Sample one non-negative publication delay for a scan."""
        if self.latency_mean_s == 0.0 and self.latency_jitter_s == 0.0:
            return 0.0
        return max(
            0.0,
            random.gauss(self.latency_mean_s, self.latency_jitter_s),
        )

    def close(self) -> None:
        """Stop the worker."""
        thread = self._thread
        if thread is None:
            return

        with self._condition:
            self._stop_requested = True
            self._pending_state = None
            self._condition.notify_all()
        self._stop_event.set()

        thread.join()
        self._thread = None
