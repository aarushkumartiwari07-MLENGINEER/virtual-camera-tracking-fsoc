"""
Classical Computer Vision Beacon Detector.
Extracts optical candidate blobs using intensity thresholding, contour analysis,
morphological filtering, and intensity-weighted centroiding.
"""

import time
import math
from typing import Tuple, List, Optional
import numpy as np
import cv2

from ..config.vision_config import DetectorConfig, CentroidConfig
from ..models.detection import CandidateBlob, DetectionResult
from .centroid import extract_subpixel_centroid


class BeaconDetector:
    """
    Classical computer vision detector for acquiring and localizing the optical beacon spot.
    Operates strictly on the 2D image matrix without ground-truth knowledge.
    """

    def __init__(
        self,
        detector_config: Optional[DetectorConfig] = None,
        centroid_config: Optional[CentroidConfig] = None,
    ) -> None:
        self.detector_config = detector_config or DetectorConfig()
        self.centroid_config = centroid_config or CentroidConfig()

    def detect(
        self,
        frame: np.ndarray,
        predicted_pos: Optional[Tuple[float, float]] = None,
        frame_index: int = 0,
        timestamp: float = 0.0,
    ) -> DetectionResult:
        """
        Detect and locate the optical beacon in the given sensor frame.

        Args:
            frame: 2D monochrome camera frame (H, W) uint8
            predicted_pos: Optional (u_pred, v_pred) from Kalman filter for candidate gating
            frame_index: Sequence frame index
            timestamp: Simulation timestamp in seconds

        Returns:
            DetectionResult with selected candidate and all detected blobs.
        """
        t_start = time.perf_counter()

        if frame is None or frame.size == 0:
            return DetectionResult(
                is_detected=False,
                frame_index=frame_index,
                timestamp=timestamp,
                processing_time_ms=(time.perf_counter() - t_start) * 1000.0,
            )

        # Ensure single-channel 8-bit image
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        h_img, w_img = gray.shape[:2]

        # 1. Segmentation / Thresholding
        if self.detector_config.use_adaptive_threshold:
            _, thresh = cv2.threshold(
                gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
            )
        else:
            thresh_val = float(self.detector_config.intensity_threshold)
            _, thresh = cv2.threshold(
                gray, thresh_val, 255, cv2.THRESH_BINARY
            )

        # 2. Optional morphological cleanup
        ksize = self.detector_config.morphology_kernel_size
        if ksize > 1:
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
            thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        # 3. Contour & Connected Component Extraction
        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        candidates: List[CandidateBlob] = []

        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            contour_area = float(cv2.contourArea(cnt))
            box_area = float(w * h)
            area = max(contour_area, box_area if contour_area == 0 else contour_area)

            # Area filtering
            if area < self.detector_config.min_area or area > self.detector_config.max_area:
                continue

            # Aspect ratio filtering
            aspect_ratio = float(w) / float(max(1, h))
            if (
                aspect_ratio < self.detector_config.min_aspect_ratio
                or aspect_ratio > self.detector_config.max_aspect_ratio
            ):
                continue

            # Extract patch for photometric metrics
            patch = gray[y:y + h, x:x + w]
            if patch.size == 0:
                continue

            peak_intensity = float(np.max(patch))
            mean_intensity = float(np.mean(patch))

            # Shape circularity / compactness
            perimeter = float(cv2.arcLength(cnt, True))
            if perimeter > 0:
                circularity = float(4.0 * math.pi * area / (perimeter * perimeter))
            else:
                circularity = 1.0

            # Sub-pixel centroid extraction
            bbox = (int(x), int(y), int(x + w), int(y + h))
            centroid = extract_subpixel_centroid(
                gray,
                bbox=bbox,
                method=self.centroid_config.method,
                padding=self.centroid_config.crop_padding,
            )

            # Deterministic candidate scoring
            # High intensity + compactness + closeness to aspect ratio 1.0
            base_score = (
                peak_intensity * 1.5
                + mean_intensity * 0.8
                + min(circularity, 1.5) * 30.0
                - abs(aspect_ratio - 1.0) * 15.0
            )

            # Proximity bonus/penalty if temporal prior is available
            if predicted_pos is not None:
                dist = math.hypot(centroid[0] - predicted_pos[0], centroid[1] - predicted_pos[1])
                # Soft Gaussian gating penalty
                proximity_factor = math.exp(-dist / 80.0)
                score = base_score * (0.5 + 0.5 * proximity_factor)
            else:
                score = base_score

            candidate = CandidateBlob(
                centroid=centroid,
                bbox=bbox,
                area=area,
                peak_intensity=peak_intensity,
                mean_intensity=mean_intensity,
                aspect_ratio=aspect_ratio,
                circularity=circularity,
                score=score,
            )
            candidates.append(candidate)

        elapsed_ms = (time.perf_counter() - t_start) * 1000.0

        if not candidates:
            return DetectionResult(
                is_detected=False,
                selected_candidate=None,
                all_candidates=(),
                frame_index=frame_index,
                timestamp=timestamp,
                processing_time_ms=elapsed_ms,
            )

        # Sort candidates descending by deterministic score
        candidates.sort(key=lambda c: c.score, reverse=True)
        best_candidate = candidates[0]

        return DetectionResult(
            is_detected=True,
            selected_candidate=best_candidate,
            all_candidates=tuple(candidates),
            frame_index=frame_index,
            timestamp=timestamp,
            processing_time_ms=elapsed_ms,
        )
