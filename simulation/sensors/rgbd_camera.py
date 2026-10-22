"""RGB-D rendering for the simulated GR3 head camera.

This module intentionally contains no ROS logic. ``RGBDCamera`` receives the
live MuJoCo model and data owned by ``viewer.py`` and renders synchronized RGB
and depth arrays from the named MJCF camera.

The object must be created and used from the same thread because MuJoCo's
OpenGL context is thread-affine.
"""

from typing import Optional, Tuple

import mujoco
import numpy as np


class RGBDCamera:
    """Render RGB and depth images from a MuJoCo camera."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        camera_name: str = "gr3_head_rgbd",
        width: int = 320,
        height: int = 240,
        frequency_hz: float = 5.0,
        print_first_frame: bool = True,
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
        self.data = data
        self.camera_name = camera_name
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.capture_period = 1.0 / frequency_hz
        self.print_first_frame = print_first_frame

        self.renderer = mujoco.Renderer(
            model,
            height=height,
            width=width,
        )
        self.last_capture_time: Optional[float] = None
        self.first_frame_printed = False

    def _disable_expensive_effects(self) -> None:
        """Disable rendering effects that are costly inside VMware."""
        scene = self.renderer.scene
        scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0

    def capture(self) -> Tuple[np.ndarray, np.ndarray]:
        """Render one synchronized RGB/depth pair from the current state.

        Returns:
            A tuple ``(rgb, depth)``. ``rgb`` has shape ``(H, W, 3)`` and
            dtype ``uint8``. ``depth`` has shape ``(H, W)`` and contains the
            floating-point camera depth values returned by MuJoCo.
        """
        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.camera_name)
        self._disable_expensive_effects()
        rgb = self.renderer.render().copy()

        self.renderer.enable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.camera_name)
        self._disable_expensive_effects()
        depth = self.renderer.render().copy()
        self.renderer.disable_depth_rendering()

        if rgb.shape != (self.height, self.width, 3):
            raise RuntimeError(f"Unexpected RGB shape: {rgb.shape}")
        if depth.shape != (self.height, self.width):
            raise RuntimeError(f"Unexpected depth shape: {depth.shape}")

        if self.print_first_frame and not self.first_frame_printed:
            self._print_first_pair(rgb, depth)
            self.first_frame_printed = True

        return rgb, depth

    def capture_if_due(
        self,
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Render at the configured rate, measured in simulation time."""
        simulation_time = float(self.data.time)

        # A model reset can move simulation time backwards.
        if (
            self.last_capture_time is not None
            and simulation_time < self.last_capture_time
        ):
            self.last_capture_time = None

        if (
            self.last_capture_time is not None
            and simulation_time - self.last_capture_time < self.capture_period
        ):
            return None

        rgb, depth = self.capture()
        self.last_capture_time = simulation_time
        return rgb, depth

    def _print_first_pair(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
    ) -> None:
        """Print representative values from the first synchronized pair."""
        finite_depth = depth[np.isfinite(depth)]
        if finite_depth.size:
            depth_min = float(finite_depth.min())
            depth_max = float(finite_depth.max())
        else:
            depth_min = float("nan")
            depth_max = float("nan")

        print(
            "RGB-D capture succeeded: "
            f"rgb={rgb.shape} {rgb.dtype}, "
            f"depth={depth.shape} {depth.dtype}, "
            f"depth_range=[{depth_min:.4f}, {depth_max:.4f}]"
        )

        sample_pixels = (
            (0, 0),
            (self.height // 2, self.width // 2),
            (self.height - 1, self.width - 1),
        )
        for row, column in sample_pixels:
            print(
                f"RGB-D pixel[{row}, {column}]: "
                f"rgb={rgb[row, column].tolist()}, "
                f"depth={float(depth[row, column]):.4f}"
            )

    def close(self) -> None:
        """Release the offscreen renderer."""
        close_renderer = getattr(self.renderer, "close", None)
        if close_renderer is not None:
            close_renderer()
