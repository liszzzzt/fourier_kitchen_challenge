"""Asynchronous RGB-D rendering for the simulated GR3 head camera.

The physics thread only submits copies of the latest robot state. A dedicated
worker owns the MuJoCo renderer and renders outside the viewer's simulation
lock. At most one pending state is retained, so slow rendering drops stale
frames instead of building an unbounded queue.
"""

import threading
from typing import Optional, Tuple

import mujoco
import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image


class RGBDCamera:
    """Render RGB and depth images on a dedicated worker thread."""

    def __init__(
        self,
        model: mujoco.MjModel,
        camera_name: str = "gr3_head_rgbd",
        width: int = 320,
        height: int = 240,
        frequency_hz: float = 5.0,
    ):
        if width <= 0 or height <= 0:
            raise ValueError("RGB-D image dimensions must be positive")
        if frequency_hz <= 0:
            raise ValueError("RGB-D frequency must be positive")

        camera_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            camera_name,
        )
        if camera_id < 0:
            raise ValueError(
                f"MuJoCo camera {camera_name!r} was not found in the model"
            )

        self.model = model
        self.camera_name = camera_name
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.capture_period = 1.0 / frequency_hz

        self.last_submitted_time: Optional[float] = None

        self._condition = threading.Condition()
        self._pending_state = None
        self._stop_requested = False
        self._thread: Optional[threading.Thread] = None
        self._worker_error: Optional[Exception] = None

    def start(self) -> None:
        """Start the rendering worker."""
        if self._thread is not None:
            return

        self._thread = threading.Thread(
            target=self._render_loop,
            name="gr3-rgbd-camera",
            daemon=True,
        )
        self._thread.start()

    def submit_state_if_due(self, data: mujoco.MjData) -> bool:
        """Copy the latest simulation state when the capture period is due.

        This method is intended to run while ``viewer.py`` holds the simulation
        lock. It performs no OpenGL operations and never waits for rendering.
        The return value indicates whether a state was submitted.
        """
        if self._thread is None:
            raise RuntimeError("RGB-D worker has not been started")
        if self._worker_error is not None:
            return False

        simulation_time = float(data.time)

        # A simulation reset can move time backwards.
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

        # Copies detach the camera worker from the live simulation MjData.
        state = (
            simulation_time,
            data.qpos.copy(),
            data.qvel.copy(),
            data.act.copy(),
            data.mocap_pos.copy(),
            data.mocap_quat.copy(),
            data.userdata.copy(),
        )

        with self._condition:
            # Replace an unrendered state: only the newest frame matters.
            self._pending_state = state
            self._condition.notify()

        self.last_submitted_time = simulation_time
        return True

    def _render_loop(self) -> None:
        """Own the OpenGL context and publish rendered RGB-D pairs."""
        renderer = None
        ros_context = None
        ros_node = None
        try:
            ros_context = Context()
            rclpy.init(context=ros_context)
            ros_node = Node("gr3_rgbd_camera", context=ros_context)

            image_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            rgb_publisher = ros_node.create_publisher(
                Image,
                "/gr3/camera/color/image_raw",
                image_qos,
            )
            depth_publisher = ros_node.create_publisher(
                Image,
                "/gr3/camera/depth/image_raw",
                image_qos,
            )

            render_data = mujoco.MjData(self.model)
            renderer = mujoco.Renderer(
                self.model,
                height=self.height,
                width=self.width,
            )

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
                    qpos,
                    qvel,
                    act,
                    mocap_pos,
                    mocap_quat,
                    userdata,
                ) = state

                render_data.time = simulation_time
                render_data.qpos[:] = qpos
                render_data.qvel[:] = qvel
                render_data.act[:] = act
                render_data.mocap_pos[:] = mocap_pos
                render_data.mocap_quat[:] = mocap_quat
                render_data.userdata[:] = userdata
                mujoco.mj_forward(self.model, render_data)

                rgb, depth = self._capture(renderer, render_data)
                self._publish_pair(
                    ros_node,
                    rgb_publisher,
                    depth_publisher,
                    rgb,
                    depth,
                )
        except Exception as error:  # Keep the main simulation alive.
            self._worker_error = error
            print(
                f"RGB-D worker failed: {type(error).__name__}: {error}",
                flush=True,
            )
        finally:
            if renderer is not None:
                close_renderer = getattr(renderer, "close", None)
                if close_renderer is not None:
                    close_renderer()
            if ros_node is not None:
                ros_node.destroy_node()
            if ros_context is not None and ros_context.ok():
                rclpy.shutdown(context=ros_context)

    def _capture(
        self,
        renderer: mujoco.Renderer,
        data: mujoco.MjData,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Render one synchronized RGB/depth pair from private data."""
        renderer.disable_depth_rendering()
        renderer.update_scene(data, camera=self.camera_name)
        self._disable_expensive_effects(renderer)
        rgb = renderer.render().copy()

        # Reuse the scene so RGB and depth correspond to exactly the same state.
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()

        if rgb.shape != (self.height, self.width, 3):
            raise RuntimeError(f"Unexpected RGB shape: {rgb.shape}")
        if depth.shape != (self.height, self.width):
            raise RuntimeError(f"Unexpected depth shape: {depth.shape}")

        return rgb, depth

    def _publish_pair(
        self,
        ros_node: Node,
        rgb_publisher,
        depth_publisher,
        rgb: np.ndarray,
        depth: np.ndarray,
    ) -> None:
        """Publish synchronized RGB and depth images with one ROS timestamp."""
        timestamp = ros_node.get_clock().now().to_msg()

        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        rgb_message = Image()
        rgb_message.header.stamp = timestamp
        rgb_message.header.frame_id = self.camera_name
        rgb_message.height = self.height
        rgb_message.width = self.width
        rgb_message.encoding = "rgb8"
        rgb_message.is_bigendian = 0
        rgb_message.step = self.width * 3
        rgb_message.data = rgb.tobytes()

        depth = np.ascontiguousarray(depth, dtype=np.float32)
        depth_message = Image()
        depth_message.header.stamp = timestamp
        depth_message.header.frame_id = self.camera_name
        depth_message.height = self.height
        depth_message.width = self.width
        depth_message.encoding = "32FC1"
        depth_message.is_bigendian = 0
        depth_message.step = self.width * depth.dtype.itemsize
        depth_message.data = depth.tobytes()

        rgb_publisher.publish(rgb_message)
        depth_publisher.publish(depth_message)

    @staticmethod
    def _disable_expensive_effects(renderer: mujoco.Renderer) -> None:
        """Disable effects that are costly inside VMware."""
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0

    def close(self) -> None:
        """Stop the worker and release its renderer in the worker thread."""
        thread = self._thread
        if thread is None:
            return

        with self._condition:
            self._stop_requested = True
            self._pending_state = None
            self._condition.notify_all()

        thread.join()
        self._thread = None
