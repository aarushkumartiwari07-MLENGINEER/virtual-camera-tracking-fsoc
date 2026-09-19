"""
Master Simulation Engine orchestrator.
Provides the clean, frontend-agnostic Python API for controlling and stepping
the coarse alignment virtual camera simulation.
"""

from typing import Tuple, Optional
import numpy as np

from .config.simulation_config import SimulationConfig
from .models.state import GroundTruthState
from .simulation.world import World
from .simulation.beacon import Beacon
from .simulation.camera import Camera
from .simulation.projection import Projector
from .simulation.renderer import FrameRenderer


class SimulationEngine:
    """
    Core simulation engine. Owns the physical simulation clock, entity kinematics,
    projection geometry, frame rendering, and ground-truth telemetry generation.
    """

    def __init__(self, config: Optional[SimulationConfig] = None) -> None:
        self.config = config or SimulationConfig()
        self.fps = float(self.config.fps)
        self.dt = 1.0 / self.fps if self.fps > 0 else 1.0 / 30.0

        # Subsystem initialization
        self.world = World(
            width=self.config.world.width,
            height=self.config.world.height,
            boundary_behavior=self.config.world.boundary_behavior,
        )

        # Ensure beacon and camera default positions align with world center if unspecified
        beacon_config = self.config.beacon
        if beacon_config.initial_pos is None:
            beacon_config.initial_pos = self.world.center

        camera_config = self.config.camera
        if camera_config.initial_pos is None:
            camera_config.initial_pos = self.world.center

        self.beacon = Beacon(config=beacon_config)
        self.camera = Camera(config=camera_config)
        self.renderer = FrameRenderer(resolution=self.camera.resolution)

        # Simulation state
        self.sim_time: float = 0.0
        self.frame_index: int = 0
        self._last_state: Optional[GroundTruthState] = None

        self.reset()

    def reset(self) -> GroundTruthState:
        """
        Reset simulation to initial t=0 conditions.
        Returns the initial GroundTruthState.
        """
        self.sim_time = 0.0
        self.frame_index = 0

        self.beacon.reset(self.config.beacon.initial_pos or self.world.center)
        self.camera.reset(
            position=self.config.camera.initial_pos or self.world.center,
            pan_deg=self.config.camera.initial_pan_deg,
            tilt_deg=self.config.camera.initial_tilt_deg,
        )

        # Compute initial state at t=0
        is_visible, img_pos, bbox, ang_err, pix_err = Projector.project_beacon(
            self.beacon, self.camera
        )

        self._last_state = GroundTruthState(
            timestamp=self.sim_time,
            frame_index=self.frame_index,
            beacon_world_pos=self.beacon.position,
            beacon_world_velocity=self.beacon.velocity,
            camera_world_pos=self.camera.position,
            camera_pan_deg=self.camera.pan_deg,
            camera_tilt_deg=self.camera.tilt_deg,
            is_visible=is_visible,
            beacon_image_pos=img_pos if is_visible else None,
            beacon_bounding_box=bbox if is_visible else None,
            angular_error_deg=ang_err,
            pixel_error=pix_err,
        )
        return self._last_state

    def step(self, dt: Optional[float] = None) -> Tuple[np.ndarray, GroundTruthState]:
        """
        Advance simulation by one discrete timestep.
        
        Args:
            dt: Optional override for timestep in seconds (default: 1/fps).
            
        Returns:
            Tuple of (monochrome_frame: np.ndarray, ground_truth: GroundTruthState)
        """
        delta_t = dt if dt is not None else self.dt

        # Advance kinematics
        self.beacon.update(delta_t, self.sim_time, self.world.bounds)
        self.camera.update(delta_t)

        self.sim_time += delta_t
        self.frame_index += 1

        # Project beacon into camera view
        is_visible, img_pos, bbox, ang_err, pix_err = Projector.project_beacon(
            self.beacon, self.camera
        )

        # Render 2D monochrome camera sensor image
        frame = self.renderer.render(
            beacon=self.beacon,
            camera=self.camera,
            is_visible=is_visible,
            image_pos=img_pos,
        )

        # Construct immutable ground-truth telemetry
        self._last_state = GroundTruthState(
            timestamp=self.sim_time,
            frame_index=self.frame_index,
            beacon_world_pos=self.beacon.position,
            beacon_world_velocity=self.beacon.velocity,
            camera_world_pos=self.camera.position,
            camera_pan_deg=self.camera.pan_deg,
            camera_tilt_deg=self.camera.tilt_deg,
            is_visible=is_visible,
            beacon_image_pos=img_pos if is_visible else None,
            beacon_bounding_box=bbox if is_visible else None,
            angular_error_deg=ang_err,
            pixel_error=pix_err,
        )

        return frame, self._last_state

    def get_state(self) -> GroundTruthState:
        """Return the most recent ground-truth state."""
        if self._last_state is None:
            return self.reset()
        return self._last_state

    def set_camera_pan_tilt(self, pan_deg: float, tilt_deg: float) -> None:
        """Command target pan/tilt angles (rate-limited slewing)."""
        self.camera.set_target_pan_tilt(pan_deg, tilt_deg)

    def set_camera_pan_tilt_instant(self, pan_deg: float, tilt_deg: float) -> None:
        """Immediately reposition gimbal without slew rate limits."""
        self.camera.set_pan_tilt_instant(pan_deg, tilt_deg)

    def command_camera_velocity(self, pan_rate_deg_s: float, tilt_rate_deg_s: float) -> None:
        """Command raw pan/tilt velocity to gimbal."""
        self.camera.command_velocity(pan_rate_deg_s, tilt_rate_deg_s)
