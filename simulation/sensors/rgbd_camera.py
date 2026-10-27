"""Asynchronous RGB-D rendering for the simulated GR3 head camera.

The physics thread only submits copies of the latest robot state. A dedicated
worker owns the MuJoCo renderer and renders outside the viewer's simulation
lock. At most one pending state is retained, so slow rendering drops stale
frames instead of building an unbounded queue.

Rendering backend (GR3_CAMERA_GL environment variable):
  "osmesa" (default)  render in a child process that only uses OSMesa (CPU,
                      llvmpipe). Inside VMware every switch between the viewer's
                      OpenGL context and a second one stalls the whole virtual
                      GPU for ~0.4 s, which froze the entire host desktop; OSMesa
                      never touches the virtual GPU. Needs libosmesa6. If the
                      child cannot start, falls back to "gpu" with a warning.
  "gpu"               render in this process on the default MuJoCo backend.
"""

import os
import pickle
import random
import struct
import subprocess
import sys
import tempfile
import threading
import time
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
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image

CAMERA_GL = os.environ.get("GR3_CAMERA_GL", "osmesa")


def _send(stream, obj) -> None:
    payload = pickle.dumps(obj, protocol=pickle.HIGHEST_PROTOCOL)
    stream.write(struct.pack("<Q", len(payload)))
    stream.write(payload)
    stream.flush()


def _recv(stream):
    header = stream.read(8)
    if len(header) < 8:
        return None
    (size,) = struct.unpack("<Q", header)
    return pickle.loads(stream.read(size))


def _apply_state(model: mujoco.MjModel, data: mujoco.MjData, state) -> None:
    simulation_time, qpos, qvel, act, mocap_pos, mocap_quat, userdata = state
    data.time = simulation_time
    data.qpos[:] = qpos
    data.qvel[:] = qvel
    data.act[:] = act
    data.mocap_pos[:] = mocap_pos
    data.mocap_quat[:] = mocap_quat
    data.userdata[:] = userdata
    mujoco.mj_forward(model, data)


def _render_rgbd(
    renderer: mujoco.Renderer,
    data: mujoco.MjData,
    camera_name: str,
) -> Tuple[np.ndarray, np.ndarray]:
    """Render one synchronized RGB/depth pair (shadows / reflections off)."""
    renderer.disable_depth_rendering()
    renderer.update_scene(data, camera=camera_name)
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
    rgb = renderer.render().copy()

    # Reuse the scene so RGB and depth correspond to exactly the same state.
    renderer.enable_depth_rendering()
    depth = renderer.render().copy()
    renderer.disable_depth_rendering()
    return rgb, depth


class _OSMesaRenderer:
    """Head-camera renderer in a child process that only uses OSMesa.

    One process cannot mix OSMesa with the viewer's GLX context (MuJoCo's GL
    function pointers are process-wide), hence the child. It receives the
    model once (as an .mjb file) and then one state per frame over a pipe, and
    returns the RGB / depth arrays.
    """

    def __init__(self, model: mujoco.MjModel, camera_name: str, width: int, height: int):
        fd, self._mjb = tempfile.mkstemp(prefix="gr3_head_camera_", suffix=".mjb")
        os.close(fd)
        mujoco.mj_saveModel(model, self._mjb, None)
        to_child_r, self._to_child = os.pipe()
        self._from_child, from_child_w = os.pipe()
        env = dict(os.environ, MUJOCO_GL="osmesa", PYOPENGL_PLATFORM="osmesa")
        self._proc = subprocess.Popen(
            [sys.executable, os.path.abspath(__file__), "--osmesa-worker", self._mjb,
             camera_name, str(width), str(height), str(to_child_r), str(from_child_w)],
            pass_fds=(to_child_r, from_child_w),
            env=env,
        )
        os.close(to_child_r)
        os.close(from_child_w)
        self._out = os.fdopen(self._to_child, "wb")
        self._in = os.fdopen(self._from_child, "rb")
        reply = _recv(self._in)
        if reply is None or reply[0] != "ready":
            self.close()
            raise RuntimeError(f"OSMesa camera process did not start: {reply}")

    def render(self, state) -> Tuple[np.ndarray, np.ndarray]:
        _send(self._out, state)
        reply = _recv(self._in)
        if reply is None or reply[0] != "ok":
            raise RuntimeError(f"OSMesa camera process failed: {reply}")
        return reply[1], reply[2]

    def close(self) -> None:
        for stream in (getattr(self, "_out", None), getattr(self, "_in", None)):
            try:
                if stream is not None:
                    stream.close()
            except OSError:
                pass
        if self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        try:
            os.remove(self._mjb)
        except OSError:
            pass


def _osmesa_worker(mjb: str, camera_name: str, width: int, height: int, fd_in: int, fd_out: int) -> None:
    """Child process: MUJOCO_GL=osmesa was set before mujoco was imported."""
    inp = os.fdopen(fd_in, "rb")
    out = os.fdopen(fd_out, "wb")
    try:
        model = mujoco.MjModel.from_binary_path(mjb)
        data = mujoco.MjData(model)
        renderer = mujoco.Renderer(model, height=height, width=width)
    except Exception as error:
        _send(out, ("error", f"{type(error).__name__}: {error}"))
        return
    _send(out, ("ready",))
    while True:
        state = _recv(inp)
        if state is None:
            return
        try:
            _apply_state(model, data, state)
            rgb, depth = _render_rgbd(renderer, data, camera_name)
            _send(out, ("ok", rgb, depth))
        except Exception as error:
            _send(out, ("error", f"{type(error).__name__}: {error}"))
            return


class RGBDCamera:
    """Render RGB and depth images on a dedicated worker thread."""

    def __init__(
        self,
        model: mujoco.MjModel,
        camera_name: str = "gr3_head_rgbd",
        width: int = 320,
        height: int = 240,
        frequency_hz: float = 5.0,
        # 0.08 seconds
        latency_mean_s: float = 0.0,
        # 0.02 seconds
        latency_jitter_s: float = 0.0,
    ):
        if width <= 0 or height <= 0:
            raise ValueError("RGB-D image dimensions must be positive")
        if frequency_hz <= 0:
            raise ValueError("RGB-D frequency must be positive")
        if latency_mean_s < 0 or latency_jitter_s < 0:
            raise ValueError("RGB-D latency values must be non-negative")

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

        self.latency_mean_s = latency_mean_s
        self.latency_jitter_s = latency_jitter_s

        self.last_submitted_time: Optional[float] = None

        self._condition = threading.Condition()
        self._pending_state = None
        self._pending_stamp_ns = 0
        self._stop_requested = False
        self._stop_event = threading.Event()
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
            # Capture time (wall clock = ROS system time): the images are stamped with it, like a real camera's
            # exposure time, not with the later publish time after rendering. Consumers pair the images with the
            # robot's measured joint states by this stamp.
            self._pending_stamp_ns = time.time_ns()
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
            renderer = self._make_renderer()

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
                    stamp_ns = self._pending_stamp_ns
                    self._pending_state = None

                if isinstance(renderer, _OSMesaRenderer):
                    rgb, depth = renderer.render(state)
                else:
                    _apply_state(self.model, render_data, state)
                    rgb, depth = _render_rgbd(renderer, render_data, self.camera_name)
                if rgb.shape != (self.height, self.width, 3):
                    raise RuntimeError(f"Unexpected RGB shape: {rgb.shape}")
                if depth.shape != (self.height, self.width):
                    raise RuntimeError(f"Unexpected depth shape: {depth.shape}")
                if self._stop_event.wait(self._sample_latency()):
                    break
                self._publish_pair(
                    ros_node,
                    rgb_publisher,
                    depth_publisher,
                    rgb,
                    depth,
                    stamp_ns,
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

    def _make_renderer(self):
        """OSMesa child process (default) or an in-process renderer (GR3_CAMERA_GL=gpu / fallback)."""
        if CAMERA_GL == "osmesa":
            try:
                renderer = _OSMesaRenderer(self.model, self.camera_name, self.width, self.height)
                print("RGB-D camera: rendering with OSMesa in a separate process", flush=True)
                return renderer
            except Exception as error:
                print(f"RGB-D camera: OSMesa unavailable ({type(error).__name__}: {error}); "
                      "rendering on the GPU instead (inside VMware this stalls the whole desktop)", flush=True)
        return mujoco.Renderer(self.model, height=self.height, width=self.width)

    def _publish_pair(
        self,
        ros_node: Node,
        rgb_publisher,
        depth_publisher,
        rgb: np.ndarray,
        depth: np.ndarray,
        stamp_ns: int,
    ) -> None:
        """Publish synchronized RGB and depth images with one timestamp (the capture time)."""
        timestamp = Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)

        rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
        rgb_message = Image()
        rgb_message.header.stamp = timestamp
        rgb_message.header.frame_id = "camera_link"
        rgb_message.height = self.height
        rgb_message.width = self.width
        rgb_message.encoding = "rgb8"
        rgb_message.is_bigendian = 0
        rgb_message.step = self.width * 3
        rgb_message.data = rgb.tobytes()

        depth = np.ascontiguousarray(depth, dtype=np.float32)
        depth_message = Image()
        depth_message.header.stamp = timestamp
        depth_message.header.frame_id = "camera_link"
        depth_message.height = self.height
        depth_message.width = self.width
        depth_message.encoding = "32FC1"
        depth_message.is_bigendian = 0
        depth_message.step = self.width * depth.dtype.itemsize
        depth_message.data = depth.tobytes()

        rgb_publisher.publish(rgb_message)
        depth_publisher.publish(depth_message)

    def _sample_latency(self) -> float:
        """Sample one non-negative publication delay for an RGB-D pair."""
        if self.latency_mean_s == 0.0 and self.latency_jitter_s == 0.0:
            return 0.0
        return max(
            0.0,
            random.gauss(self.latency_mean_s, self.latency_jitter_s),
        )

    def close(self) -> None:
        """Stop the worker and release its renderer in the worker thread."""
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


if __name__ == "__main__" and len(sys.argv) == 8 and sys.argv[1] == "--osmesa-worker":
    _osmesa_worker(sys.argv[2], sys.argv[3], int(sys.argv[4]), int(sys.argv[5]), int(sys.argv[6]), int(sys.argv[7]))
