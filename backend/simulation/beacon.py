"""
Beacon entity module for the FSOC virtual simulation.
Represents the optical spot with spatial, optical, and kinematic properties.
"""

from typing import Tuple, Optional
from .motion import MotionModel, create_motion_model
from ..config.simulation_config import BeaconConfig


class Beacon:
    """
    Simulated optical beacon emitted by the remote FSOC terminal.
    """

    def __init__(
        self,
        config: Optional[BeaconConfig] = None,
        motion_model: Optional[MotionModel] = None,
    ) -> None:
        self.config = config or BeaconConfig()
        self.size = float(self.config.size)
        self.shape = self.config.shape.lower()
        self.intensity = float(self.config.intensity)
        
        # Position and kinematic state
        self.initial_pos = self.config.initial_pos
        self.position: Tuple[float, float] = self.initial_pos if self.initial_pos is not None else (1000.0, 1000.0)
        self.velocity: Tuple[float, float] = (0.0, 0.0)

        # Motion model initialization
        if motion_model is not None:
            self.motion_model = motion_model
        else:
            self.motion_model = create_motion_model(
                self.config.motion_type, **self.config.motion_params
            )
        
        self.reset(self.position)

    def reset(self, initial_pos: Optional[Tuple[float, float]] = None) -> None:
        """Reset beacon position, velocity, and internal motion clock."""
        if initial_pos is not None:
            self.position = (float(initial_pos[0]), float(initial_pos[1]))
        elif self.initial_pos is not None:
            self.position = (float(self.initial_pos[0]), float(self.initial_pos[1]))
        else:
            self.position = (1000.0, 1000.0)
        
        self.velocity = (0.0, 0.0)
        self.motion_model.reset(self.position)

    def update(
        self,
        dt: float,
        sim_time: float,
        world_bounds: Tuple[float, float],
    ) -> None:
        """Advance beacon trajectory by timestep dt."""
        (new_x, new_y), (vx, vy) = self.motion_model.update(
            self.position, dt, sim_time, world_bounds
        )
        self.position = (float(new_x), float(new_y))
        self.velocity = (float(vx), float(vy))

    @property
    def bounding_box_world(self) -> Tuple[float, float, float, float]:
        """Return (xmin, ymin, xmax, ymax) in world coordinates."""
        half = self.size / 2.0
        return (
            self.position[0] - half,
            self.position[1] - half,
            self.position[0] + half,
            self.position[1] + half,
        )
