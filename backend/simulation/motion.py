"""
Motion models for the optical beacon target in the FSOC virtual simulation.
Implements:
1. Straight-Line Motion
2. Circular Motion
3. Figure-of-Eight Motion (Lemniscate)
4. Random Walk / Filtered Acceleration Drift
"""

import math
import numpy as np
from abc import ABC, abstractmethod
from typing import Tuple, Dict, Any, Optional


class MotionModel(ABC):
    """Abstract base class for beacon motion models."""

    @abstractmethod
    def reset(self, initial_pos: Tuple[float, float]) -> None:
        """Reset motion state to initial conditions."""
        pass

    @abstractmethod
    def update(
        self,
        current_pos: Tuple[float, float],
        dt: float,
        sim_time: float,
        world_bounds: Tuple[float, float],
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """
        Advance motion by dt.
        Returns:
            ((new_x, new_y), (vx, vy))
        """
        pass


class StraightLineMotion(MotionModel):
    """
    Linear motion with constant velocity vector and optional boundary bouncing.
    """

    def __init__(
        self,
        speed: float = 60.0,
        angle_deg: float = 45.0,
        bounce: bool = True,
    ) -> None:
        self.speed = float(speed)
        self.angle_rad = math.radians(angle_deg)
        self.bounce = bounce
        self.vx = self.speed * math.cos(self.angle_rad)
        self.vy = self.speed * math.sin(self.angle_rad)

    def reset(self, initial_pos: Tuple[float, float]) -> None:
        self.vx = self.speed * math.cos(self.angle_rad)
        self.vy = self.speed * math.sin(self.angle_rad)

    def update(
        self,
        current_pos: Tuple[float, float],
        dt: float,
        sim_time: float,
        world_bounds: Tuple[float, float],
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        x, y = current_pos
        x_new = x + self.vx * dt
        y_new = y + self.vy * dt

        if self.bounce:
            w, h = world_bounds
            if x_new < 0.0:
                x_new = -x_new
                self.vx = abs(self.vx)
            elif x_new > w:
                x_new = 2.0 * w - x_new
                self.vx = -abs(self.vx)

            if y_new < 0.0:
                y_new = -y_new
                self.vy = abs(self.vy)
            elif y_new > h:
                y_new = 2.0 * h - y_new
                self.vy = -abs(self.vy)

            x_new = max(0.0, min(w, x_new))
            y_new = max(0.0, min(h, y_new))

        return (x_new, y_new), (self.vx, self.vy)


class CircularMotion(MotionModel):
    """
    Circular orbital trajectory around a specified center point.
    x(t) = cx + R * cos(omega * t + phi0)
    y(t) = cy + R * sin(omega * t + phi0)
    """

    def __init__(
        self,
        center: Optional[Tuple[float, float]] = None,
        radius: float = 250.0,
        angular_speed_deg_s: float = 30.0,
        initial_phase_deg: float = 0.0,
    ) -> None:
        self.center = center
        self.radius = float(radius)
        self.angular_speed_rad_s = math.radians(angular_speed_deg_s)
        self.initial_phase_rad = math.radians(initial_phase_deg)
        self.current_phase_rad = self.initial_phase_rad

    def reset(self, initial_pos: Tuple[float, float]) -> None:
        self.current_phase_rad = self.initial_phase_rad
        if self.center is None:
            # If no center given, place center such that initial_pos is on circle at phase0
            cx = initial_pos[0] - self.radius * math.cos(self.initial_phase_rad)
            cy = initial_pos[1] - self.radius * math.sin(self.initial_phase_rad)
            self.center = (cx, cy)

    def update(
        self,
        current_pos: Tuple[float, float],
        dt: float,
        sim_time: float,
        world_bounds: Tuple[float, float],
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        if self.center is None:
            w, h = world_bounds
            self.center = (w / 2.0, h / 2.0)

        cx, cy = self.center
        phase = self.initial_phase_rad + self.angular_speed_rad_s * sim_time
        
        x_new = cx + self.radius * math.cos(phase)
        y_new = cy + self.radius * math.sin(phase)

        vx = -self.radius * self.angular_speed_rad_s * math.sin(phase)
        vy = self.radius * self.angular_speed_rad_s * math.cos(phase)

        return (x_new, y_new), (vx, vy)


class FigureEightMotion(MotionModel):
    """
    Figure-of-8 (Lemniscate of Gerono) trajectory.
    x(t) = cx + Ax * sin(omega * t + phi)
    y(t) = cy + (Ay / 2) * sin(2 * (omega * t + phi))
    """

    def __init__(
        self,
        center: Optional[Tuple[float, float]] = None,
        amplitude_x: float = 300.0,
        amplitude_y: float = 200.0,
        period_s: float = 12.0,
        initial_phase_deg: float = 0.0,
    ) -> None:
        self.center = center
        self.amplitude_x = float(amplitude_x)
        self.amplitude_y = float(amplitude_y)
        self.period_s = max(0.1, float(period_s))
        self.omega = 2.0 * math.pi / self.period_s
        self.initial_phase_rad = math.radians(initial_phase_deg)

    def reset(self, initial_pos: Tuple[float, float]) -> None:
        if self.center is None:
            self.center = initial_pos

    def update(
        self,
        current_pos: Tuple[float, float],
        dt: float,
        sim_time: float,
        world_bounds: Tuple[float, float],
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        if self.center is None:
            w, h = world_bounds
            self.center = (w / 2.0, h / 2.0)

        cx, cy = self.center
        theta = self.omega * sim_time + self.initial_phase_rad

        x_new = cx + self.amplitude_x * math.sin(theta)
        y_new = cy + (self.amplitude_y / 2.0) * math.sin(2.0 * theta)

        vx = self.amplitude_x * self.omega * math.cos(theta)
        vy = self.amplitude_y * self.omega * math.cos(2.0 * theta)

        return (x_new, y_new), (vx, vy)


class RandomMotion(MotionModel):
    """
    Smooth continuous random walk via Ornstein-Uhlenbeck stochastic acceleration.
    Simulates realistic optical target drift without jerky step discontinuities.
    """

    def __init__(
        self,
        max_speed: float = 80.0,
        acceleration_std: float = 50.0,
        drag: float = 0.8,
        seed: Optional[int] = None,
    ) -> None:
        self.max_speed = float(max_speed)
        self.acceleration_std = float(acceleration_std)
        self.drag = float(drag)
        self.seed = seed
        self.rng = np.random.default_rng(seed)
        self.vx = 0.0
        self.vy = 0.0

    def reset(self, initial_pos: Tuple[float, float]) -> None:
        self.rng = np.random.default_rng(self.seed)
        self.vx = float(self.rng.uniform(-self.max_speed / 2.0, self.max_speed / 2.0))
        self.vy = float(self.rng.uniform(-self.max_speed / 2.0, self.max_speed / 2.0))

    def update(
        self,
        current_pos: Tuple[float, float],
        dt: float,
        sim_time: float,
        world_bounds: Tuple[float, float],
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        # Ornstein-Uhlenbeck velocity step with drag and random kick
        noise_x = self.rng.normal(0.0, self.acceleration_std * math.sqrt(dt))
        noise_y = self.rng.normal(0.0, self.acceleration_std * math.sqrt(dt))

        self.vx = (1.0 - self.drag * dt) * self.vx + noise_x
        self.vy = (1.0 - self.drag * dt) * self.vy + noise_y

        # Speed clamping
        speed = math.hypot(self.vx, self.vy)
        if speed > self.max_speed:
            scale = self.max_speed / speed
            self.vx *= scale
            self.vy *= scale

        x, y = current_pos
        x_new = x + self.vx * dt
        y_new = y + self.vy * dt

        # Boundary soft rebound
        w, h = world_bounds
        margin = 50.0
        if x_new < margin:
            self.vx += abs(self.vx) * 0.5 + 10.0
        elif x_new > w - margin:
            self.vx -= abs(self.vx) * 0.5 + 10.0

        if y_new < margin:
            self.vy += abs(self.vy) * 0.5 + 10.0
        elif y_new > h - margin:
            self.vy -= abs(self.vy) * 0.5 + 10.0

        x_new = max(0.0, min(w, x_new))
        y_new = max(0.0, min(h, y_new))

        return (x_new, y_new), (self.vx, self.vy)


import inspect

def create_motion_model(motion_type: str, **kwargs: Any) -> MotionModel:
    """
    Factory function for instantiating beacon motion models.
    Filters keyword arguments to only those accepted by the model's constructor.
    """
    norm_type = motion_type.lower().strip().replace("-", "_").replace(" ", "_")
    model_cls: type[MotionModel]
    if norm_type in ("straight_line", "straight", "linear"):
        model_cls = StraightLineMotion
    elif norm_type in ("circular", "circle"):
        model_cls = CircularMotion
    elif norm_type in ("figure_eight", "figure_of_eight", "figure8", "lemniscate"):
        model_cls = FigureEightMotion
    elif norm_type in ("random", "random_walk", "drift"):
        model_cls = RandomMotion
    else:
        raise ValueError(
            f"Unknown motion type '{motion_type}'. Supported: 'straight_line', 'circular', 'figure_eight', 'random'"
        )

    # Filter kwargs to match constructor signature
    sig = inspect.signature(model_cls.__init__)
    valid_args = {k: v for k, v in kwargs.items() if k in sig.parameters}
    return model_cls(**valid_args)

