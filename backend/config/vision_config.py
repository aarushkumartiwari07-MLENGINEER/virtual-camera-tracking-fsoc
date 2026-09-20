"""
Configuration dataclasses for Phase 2 Classical Computer Vision & Tracking.
Defines parameters for thresholding, candidate filtering, centroiding,
Kalman filtering, and the tracking state machine.
"""

from dataclasses import dataclass, field
from typing import Tuple, Optional


@dataclass
class DetectorConfig:
    """Parameters for classical beacon candidate detection and segmentation."""
    intensity_threshold: float = 30.0     # Minimum intensity threshold for binary segmentation (0-255)
    min_area: float = 2.0                 # Minimum blob area in pixels (to filter single-pixel noise)
    max_area: float = 1500.0              # Maximum blob area in pixels (to filter large background clusters)
    min_aspect_ratio: float = 0.2         # Min width/height ratio
    max_aspect_ratio: float = 5.0         # Max width/height ratio
    use_adaptive_threshold: bool = False  # If True, use Otsu/adaptive thresholding
    morphology_kernel_size: int = 0       # 0 = disabled, 3 = 3x3 open/close morphological cleanup


@dataclass
class CentroidConfig:
    """Parameters for sub-pixel centroid estimation."""
    method: str = "intensity_weighted"    # 'intensity_weighted' (moments) or 'geometric'
    crop_padding: int = 2                 # Pixel padding around candidate bounding box for moments


@dataclass
class KalmanConfig:
    """Parameters for 2D constant-velocity Kalman filter."""
    process_noise_std: float = 15.0       # Acceleration process noise std (px/s^2)
    measurement_noise_std: float = 1.0    # Centroid measurement noise std (px)
    initial_error_std: float = 10.0       # Initial position uncertainty std (px)
    initial_velocity_std: float = 50.0    # Initial velocity uncertainty std (px/s)


@dataclass
class StateMachineConfig:
    """Parameters for tracking finite state machine."""
    consecutive_acquire_frames: int = 3   # Consecutive detections needed to promote ACQUIRED -> TRACKING
    max_lost_coasting_frames: int = 5     # Maximum missing frames to coast in TRACKING before transitioning to LOST
    consecutive_reacquire_frames: int = 2 # Consecutive detections needed to transition REACQUIRED -> TRACKING


@dataclass
class VisionConfig:
    """Master configuration for the Phase 2 computer vision and tracking pipeline."""
    detector: DetectorConfig = field(default_factory=DetectorConfig)
    centroid: CentroidConfig = field(default_factory=CentroidConfig)
    kalman: KalmanConfig = field(default_factory=KalmanConfig)
    state_machine: StateMachineConfig = field(default_factory=StateMachineConfig)
