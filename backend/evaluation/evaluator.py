"""
Tracking Evaluation and Ground-Truth Comparison Module.
Computes objective error metrics, RMSE, detection rates, and acquisition latency
by comparing TrackerOutput against authoritative GroundTruthState.
"""

import math
from typing import Dict, Any, List, Optional, Tuple
import numpy as np

from ..models.state import GroundTruthState
from ..models.tracking_state import TrackerOutput, TrackingState


class TrackingEvaluator:
    """
    Independent evaluation layer. Computes tracking metrics without biasing the tracker.
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        """Reset all evaluation accumulators."""
        self.total_frames: int = 0
        self.visible_frames: int = 0
        self.detected_frames: int = 0
        self.tracked_frames: int = 0
        self.false_positive_frames: int = 0
        self.missed_detection_frames: int = 0

        self.raw_errors: List[float] = []
        self.filtered_errors: List[float] = []
        self.processing_times_ms: List[float] = []

        self.acquisition_time_s: Optional[float] = None
        self._lost_timestamp: Optional[float] = None
        self.reacquisition_durations_s: List[float] = []

    def evaluate_step(
        self,
        tracker_output: TrackerOutput,
        ground_truth: GroundTruthState,
    ) -> Dict[str, Any]:
        """
        Evaluate a single simulation step comparing tracker output against ground truth.

        Returns:
            Dictionary of instantaneous step metrics.
        """
        self.total_frames += 1
        self.processing_times_ms.append(tracker_output.total_processing_time_ms)

        raw_err: Optional[float] = None
        filtered_err: Optional[float] = None

        if ground_truth.is_visible:
            self.visible_frames += 1
            gt_pos = ground_truth.beacon_image_pos

            # Check raw detection error
            if tracker_output.raw_detection.is_detected and tracker_output.raw_detection.centroid is not None and gt_pos is not None:
                self.detected_frames += 1
                det_u, det_v = tracker_output.raw_detection.centroid
                raw_err = float(math.hypot(det_u - gt_pos[0], det_v - gt_pos[1]))
                self.raw_errors.append(raw_err)
            else:
                self.missed_detection_frames += 1

            # Check filtered error
            if tracker_output.filtered_centroid is not None and gt_pos is not None:
                filt_u, filt_v = tracker_output.filtered_centroid
                filtered_err = float(math.hypot(filt_u - gt_pos[0], filt_v - gt_pos[1]))
                self.filtered_errors.append(filtered_err)

        else:
            # Target is outside FOV
            if tracker_output.raw_detection.is_detected:
                self.false_positive_frames += 1

        if tracker_output.state == TrackingState.TRACKING:
            self.tracked_frames += 1
            if self.acquisition_time_s is None:
                self.acquisition_time_s = float(ground_truth.timestamp)

            # Re-acquisition time calculation
            if self._lost_timestamp is not None:
                reacq_dt = float(ground_truth.timestamp - self._lost_timestamp)
                self.reacquisition_durations_s.append(reacq_dt)
                self._lost_timestamp = None

        elif tracker_output.state == TrackingState.LOST:
            if self._lost_timestamp is None:
                self._lost_timestamp = float(ground_truth.timestamp)

        return {
            "frame_index": ground_truth.frame_index,
            "timestamp": ground_truth.timestamp,
            "is_visible": ground_truth.is_visible,
            "is_detected": tracker_output.raw_detection.is_detected,
            "tracking_state": tracker_output.state.value,
            "raw_error_px": raw_err,
            "filtered_error_px": filtered_err,
            "processing_time_ms": tracker_output.total_processing_time_ms,
        }

    def get_summary_metrics(self) -> Dict[str, Any]:
        """
        Compute aggregate benchmark metrics over all processed frames.
        """
        mean_raw = float(np.mean(self.raw_errors)) if self.raw_errors else 0.0
        rmse_raw = float(np.sqrt(np.mean(np.square(self.raw_errors)))) if self.raw_errors else 0.0

        mean_filtered = float(np.mean(self.filtered_errors)) if self.filtered_errors else 0.0
        rmse_filtered = float(np.sqrt(np.mean(np.square(self.filtered_errors)))) if self.filtered_errors else 0.0

        det_rate = (
            (self.detected_frames / self.visible_frames * 100.0)
            if self.visible_frames > 0
            else 0.0
        )
        target_loss_rate = (
            (self.missed_detection_frames / self.visible_frames * 100.0)
            if self.visible_frames > 0
            else 0.0
        )

        mean_proc_time = float(np.mean(self.processing_times_ms)) if self.processing_times_ms else 0.0
        fps = (1000.0 / mean_proc_time) if mean_proc_time > 0 else 0.0

        mean_reacq = (
            float(np.mean(self.reacquisition_durations_s))
            if self.reacquisition_durations_s
            else None
        )

        return {
            "total_frames": self.total_frames,
            "visible_frames": self.visible_frames,
            "detected_frames": self.detected_frames,
            "detection_rate_pct": det_rate,
            "target_loss_rate_pct": target_loss_rate,
            "false_positives": self.false_positive_frames,
            "mean_raw_error_px": mean_raw,
            "rmse_raw_error_px": rmse_raw,
            "mean_filtered_error_px": mean_filtered,
            "rmse_filtered_error_px": rmse_filtered,
            "acquisition_time_s": self.acquisition_time_s,
            "mean_reacquisition_time_s": mean_reacq,
            "mean_processing_time_ms": mean_proc_time,
            "processing_fps": fps,
        }
