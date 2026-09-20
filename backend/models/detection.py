"""
Data models for classical computer vision candidate extraction and detection.
Completely independent from ground-truth simulation models.
"""

from dataclasses import dataclass, asdict
from typing import Tuple, List, Optional, Dict, Any


@dataclass(frozen=True)
class CandidateBlob:
    """
    Extracted candidate optical blob from the camera sensor frame.
    """
    centroid: Tuple[float, float]             # Sub-pixel (u, v) image coordinates
    bbox: Tuple[int, int, int, int]           # (umin, vmin, umax, vmax) integer bounding box
    area: float                               # Blob area in pixels
    peak_intensity: float                     # Maximum intensity inside blob (0-255)
    mean_intensity: float                     # Mean intensity inside blob
    aspect_ratio: float                       # Width / Height
    circularity: float                        # 4 * pi * area / perimeter^2
    score: float = 0.0                        # Candidate selection score

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class DetectionResult:
    """
    Output produced by the classical detector for a single frame.
    """
    is_detected: bool                         # True if a valid beacon candidate was selected
    selected_candidate: Optional[CandidateBlob] = None # The chosen beacon candidate
    all_candidates: Tuple[CandidateBlob, ...] = ()      # All extracted candidates for diagnostics
    frame_index: int = 0                      # Frame sequence index
    timestamp: float = 0.0                    # Frame timestamp in seconds
    processing_time_ms: float = 0.0           # Detector execution latency in milliseconds

    @property
    def centroid(self) -> Optional[Tuple[float, float]]:
        """Convenience property for the detected sub-pixel centroid."""
        return self.selected_candidate.centroid if self.selected_candidate is not None else None

    @property
    def bbox(self) -> Optional[Tuple[int, int, int, int]]:
        """Convenience property for the detected bounding box."""
        return self.selected_candidate.bbox if self.selected_candidate is not None else None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "is_detected": self.is_detected,
            "centroid": self.centroid,
            "bbox": self.bbox,
            "candidate_count": len(self.all_candidates),
            "frame_index": self.frame_index,
            "timestamp": self.timestamp,
            "processing_time_ms": self.processing_time_ms,
        }
