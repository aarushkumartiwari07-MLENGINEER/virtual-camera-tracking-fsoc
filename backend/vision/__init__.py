"""
Classical Computer Vision and Tracking Package for SIH26169.
"""

from .centroid import (
    calculate_intensity_weighted_centroid,
    calculate_geometric_centroid,
    extract_subpixel_centroid,
)
from .detector import BeaconDetector
from .kalman import BeaconKalmanFilter
from .state_machine import TrackingStateMachine
from .pipeline import VisionPipeline

__all__ = [
    "calculate_intensity_weighted_centroid",
    "calculate_geometric_centroid",
    "extract_subpixel_centroid",
    "BeaconDetector",
    "BeaconKalmanFilter",
    "TrackingStateMachine",
    "VisionPipeline",
]
