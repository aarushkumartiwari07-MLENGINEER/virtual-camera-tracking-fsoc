"""
Ground-truth state and telemetry data definitions.
Strictly authoritative physical ground truth, separated from tracker output.
"""

from dataclasses import dataclass, asdict
from typing import Tuple, Optional, Dict, Any


@dataclass(frozen=True)
class GroundTruthState:
    """
    Authoritative physical ground-truth state produced at every simulation tick.
    Used for metrics calculation, error benchmarking, and evaluation without tracker bias.
    """
    timestamp: float                       # Simulation time elapsed in seconds (t >= 0.0)
    frame_index: int                       # Monotonically increasing frame sequence index
    beacon_world_pos: Tuple[float, float]  # True beacon position (x, y) in world units
    beacon_world_velocity: Tuple[float, float] # True beacon instantaneous velocity (vx, vy) in px/s
    camera_world_pos: Tuple[float, float]  # True camera position (x, y) in world units
    camera_pan_deg: float                  # True camera pan angle in degrees
    camera_tilt_deg: float                 # True camera tilt angle in degrees
    is_visible: bool                       # True if beacon is within the camera Field of View
    beacon_image_pos: Optional[Tuple[float, float]] = None # Subpixel centroid (u, v) if visible
    beacon_bounding_box: Optional[Tuple[float, float, float, float]] = None # (umin, vmin, umax, vmax)
    angular_error_deg: Optional[Tuple[float, float]] = None # (az_err, el_err) from bore-sight
    pixel_error: Optional[Tuple[float, float]] = None       # (u - u0, v - v0) from optical center

    def to_dict(self) -> Dict[str, Any]:
        """Convert state to serializable dictionary for logging and telemetry."""
        return asdict(self)
