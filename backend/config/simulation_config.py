"""
Configuration dataclasses for the FSOC Virtual Camera Simulation.
Defines all tunable physical, optical, and kinematic parameters.
"""

from dataclasses import dataclass, field
from typing import Tuple, Optional, Dict, Any


@dataclass
class WorldConfig:
    """Configuration for the virtual simulation world."""
    width: float = 2000.0   # Minimum 2000 px as per PS
    height: float = 2000.0  # Minimum 2000 px as per PS
    boundary_behavior: str = "bounce"  # 'bounce', 'wrap', or 'clamp'


@dataclass
class BeaconConfig:
    """Configuration for the optical beacon target."""
    initial_pos: Optional[Tuple[float, float]] = None  # (x, y) in world units; None -> center
    size: float = 10.0  # Size in pixels (5-20 px, default 10x10 as per PS)
    shape: str = "square"  # 'square', 'gaussian', 'circle'
    intensity: float = 255.0  # Peak intensity (0-255)
    motion_type: str = "straight_line"  # 'straight_line', 'circular', 'figure_eight', 'random'
    motion_params: Dict[str, Any] = field(default_factory=dict)


@dataclass
class CameraConfig:
    """Configuration for the virtual tracking camera and PTZ mechanism."""
    resolution: Tuple[int, int] = (640, 480)  # (width, height) default 640x480
    fov_deg: Tuple[float, float] = (4.0, 3.0)  # (horizontal, vertical) FOV in degrees
    initial_pos: Optional[Tuple[float, float]] = None  # (x, y) in world coords; None -> center
    initial_pan_deg: float = 0.0  # Initial pan (azimuth) offset
    initial_tilt_deg: float = 0.0  # Initial tilt (elevation) offset
    max_pan_speed_deg_s: float = 10.0  # Maximum pan angular speed (5-10 deg/s as per PS)
    max_tilt_speed_deg_s: float = 10.0  # Maximum tilt angular speed (5-10 deg/s as per PS)
    monochrome: bool = True  # Monochrome 8-bit sensor


@dataclass
class SimulationConfig:
    """Master simulation configuration container."""
    fps: float = 30.0  # Update rate (minimum 20-30 Hz as per PS)
    world: WorldConfig = field(default_factory=WorldConfig)
    beacon: BeaconConfig = field(default_factory=BeaconConfig)
    camera: CameraConfig = field(default_factory=CameraConfig)
    seed: Optional[int] = 42  # Seed for reproducible random motion
