"""
Virtual Tracking Camera and Gimbal Kinematics Model.
Represents the optical receiver camera with pan/tilt angular mechanism.
"""

import math
from typing import Tuple, Optional
from ..config.simulation_config import CameraConfig


class Camera:
    """
    Simulated coarse alignment camera mounted on a 2-axis Pan-Tilt gimbal.
    """

    def __init__(self, config: Optional[CameraConfig] = None) -> None:
        self.config = config or CameraConfig()
        self.resolution: Tuple[int, int] = self.config.resolution
        self.fov_deg: Tuple[float, float] = self.config.fov_deg
        self.width = self.resolution[0]
        self.height = self.resolution[1]

        # Derived optical scales (degrees per pixel)
        self.scale_x_deg_px = self.fov_deg[0] / float(self.width)
        self.scale_y_deg_px = self.fov_deg[1] / float(self.height)
        self.principal_point: Tuple[float, float] = (self.width / 2.0, self.height / 2.0)

        # Spatial position in world space
        self.initial_pos = self.config.initial_pos
        self.position: Tuple[float, float] = (
            self.initial_pos if self.initial_pos is not None else (1000.0, 1000.0)
        )

        # Pan-Tilt Gimbal angles (degrees)
        self.initial_pan_deg = float(self.config.initial_pan_deg)
        self.initial_tilt_deg = float(self.config.initial_tilt_deg)
        self.pan_deg = self.initial_pan_deg
        self.tilt_deg = self.initial_tilt_deg

        # Target angles for rate-limited slewing
        self.target_pan_deg = self.initial_pan_deg
        self.target_tilt_deg = self.initial_tilt_deg

        # Kinematic limits (deg/s)
        self.max_pan_speed = float(self.config.max_pan_speed_deg_s)
        self.max_tilt_speed = float(self.config.max_tilt_speed_deg_s)

        # Current commanded velocities
        self.pan_rate_deg_s = 0.0
        self.tilt_rate_deg_s = 0.0

    def reset(
        self,
        position: Optional[Tuple[float, float]] = None,
        pan_deg: Optional[float] = None,
        tilt_deg: Optional[float] = None,
    ) -> None:
        """Reset camera position and gimbal orientation."""
        if position is not None:
            self.position = (float(position[0]), float(position[1]))
        elif self.initial_pos is not None:
            self.position = (float(self.initial_pos[0]), float(self.initial_pos[1]))
        else:
            self.position = (1000.0, 1000.0)

        self.pan_deg = float(pan_deg if pan_deg is not None else self.initial_pan_deg)
        self.tilt_deg = float(tilt_deg if tilt_deg is not None else self.initial_tilt_deg)
        self.target_pan_deg = self.pan_deg
        self.target_tilt_deg = self.tilt_deg
        self.pan_rate_deg_s = 0.0
        self.tilt_rate_deg_s = 0.0

    def set_target_pan_tilt(self, pan_deg: float, tilt_deg: float) -> None:
        """Set target angles for the gimbal to slew towards."""
        self.target_pan_deg = float(pan_deg)
        self.target_tilt_deg = float(tilt_deg)

    def set_pan_tilt_instant(self, pan_deg: float, tilt_deg: float) -> None:
        """Immediately set gimbal angles without rate limiting (e.g. for testing/init)."""
        self.pan_deg = float(pan_deg)
        self.tilt_deg = float(tilt_deg)
        self.target_pan_deg = float(pan_deg)
        self.target_tilt_deg = float(tilt_deg)
        self.pan_rate_deg_s = 0.0
        self.tilt_rate_deg_s = 0.0

    def command_velocity(self, pan_rate_deg_s: float, tilt_rate_deg_s: float) -> None:
        """Command raw pan/tilt angular velocity with saturation at kinematic limits."""
        self.pan_rate_deg_s = max(-self.max_pan_speed, min(self.max_pan_speed, float(pan_rate_deg_s)))
        self.tilt_rate_deg_s = max(-self.max_tilt_speed, min(self.max_tilt_speed, float(tilt_rate_deg_s)))

    def update(self, dt: float) -> None:
        """
        Advance gimbal kinematics by timestep dt.
        If velocity was commanded, integrate velocity.
        Otherwise slew toward target_pan_deg / target_tilt_deg respecting max speed.
        """
        if self.pan_rate_deg_s != 0.0 or self.tilt_rate_deg_s != 0.0:
            self.pan_deg += self.pan_rate_deg_s * dt
            self.tilt_deg += self.tilt_rate_deg_s * dt
            self.target_pan_deg = self.pan_deg
            self.target_tilt_deg = self.tilt_deg
        else:
            # Slew towards target pan
            pan_diff = self.target_pan_deg - self.pan_deg
            max_pan_step = self.max_pan_speed * dt
            if abs(pan_diff) <= max_pan_step:
                self.pan_deg = self.target_pan_deg
            else:
                self.pan_deg += math.copysign(max_pan_step, pan_diff)

            # Slew towards target tilt
            tilt_diff = self.target_tilt_deg - self.tilt_deg
            max_tilt_step = self.max_tilt_speed * dt
            if abs(tilt_diff) <= max_tilt_step:
                self.tilt_deg = self.target_tilt_deg
            else:
                self.tilt_deg += math.copysign(max_tilt_step, tilt_diff)
