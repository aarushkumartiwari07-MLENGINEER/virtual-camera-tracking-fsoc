"""
Master Computer Vision and Tracking Pipeline orchestrator.
Coordinates BeaconDetector, BeaconKalmanFilter, and TrackingStateMachine.
Consumes ONLY the raw sensor image frame and produces TrackerOutput.
"""

import time
from typing import Optional, Tuple
import numpy as np

from ..config.vision_config import VisionConfig
from ..models.detection import DetectionResult
from ..models.tracking_state import TrackingState, TrackerOutput
from .detector import BeaconDetector
from .kalman import BeaconKalmanFilter
from .state_machine import TrackingStateMachine


class VisionPipeline:
    """
    Complete Phase 2 vision pipeline:
    Image Frame -> Classical Detector -> Subpixel Centroid -> 2D Kalman Filter -> FSM -> TrackerOutput
    """

    def __init__(self, config: Optional[VisionConfig] = None) -> None:
        self.config = config or VisionConfig()
        self.detector = BeaconDetector(
            detector_config=self.config.detector,
            centroid_config=self.config.centroid,
        )
        self.kalman = BeaconKalmanFilter(config=self.config.kalman)
        self.state_machine = TrackingStateMachine(config=self.config.state_machine)
        self.last_output: Optional[TrackerOutput] = None

    def reset(self) -> None:
        """Reset internal filter and state machine."""
        self.kalman.reset()
        self.state_machine.reset()
        self.last_output = None

    def process_frame(
        self,
        frame: np.ndarray,
        dt: float = 1.0 / 30.0,
        timestamp: float = 0.0,
        frame_index: int = 0,
    ) -> TrackerOutput:
        """
        Process a single 2D camera frame through detection, filtering, and state management.

        Args:
            frame: 2D monochrome camera frame (H, W) uint8
            dt: Discrete timestep elapsed since previous frame in seconds
            timestamp: Simulation time in seconds
            frame_index: Sequence frame index

        Returns:
            TrackerOutput containing raw detection, filtered centroid, velocity, and state.
        """
        t_start = time.perf_counter()

        # 1. Kalman Predict Step
        if self.kalman.is_initialized:
            pred_u, pred_v = self.kalman.predict(dt=dt)
            prior_pos: Optional[Tuple[float, float]] = (pred_u, pred_v)
        else:
            prior_pos = None

        # 2. Candidate Detection & Subpixel Centroiding
        # Provide prior position for candidate scoring/gating if active track exists
        detection = self.detector.detect(
            frame=frame,
            predicted_pos=prior_pos if self.state_machine.current_state in (TrackingState.TRACKING, TrackingState.ACQUIRED, TrackingState.REACQUIRED) else None,
            frame_index=frame_index,
            timestamp=timestamp,
        )

        # 3. State Machine Update
        state = self.state_machine.update(is_detected=detection.is_detected)

        # 4. Kalman Filter Update / Coasting
        is_predicted = False
        if detection.is_detected and detection.centroid is not None:
            # Update filter with fresh measurement
            filt_u, filt_v = self.kalman.update(detection.centroid)
            filtered_pos: Optional[Tuple[float, float]] = (filt_u, filt_v)
            filtered_vel: Optional[Tuple[float, float]] = self.kalman.get_velocity()
            is_predicted = False
        else:
            # No measurement in this frame
            if state == TrackingState.TRACKING and self.kalman.is_initialized:
                # Coast with predicted state
                filtered_pos = self.kalman.get_position()
                filtered_vel = self.kalman.get_velocity()
                is_predicted = True
            else:
                # Track lost or searching: reset filter
                if state in (TrackingState.LOST, TrackingState.SEARCHING):
                    self.kalman.reset()
                filtered_pos = None
                filtered_vel = None
                is_predicted = False

        is_tracking_bool = state in (TrackingState.ACQUIRED, TrackingState.TRACKING, TrackingState.REACQUIRED)
        total_time_ms = (time.perf_counter() - t_start) * 1000.0

        output = TrackerOutput(
            state=state,
            is_tracking=is_tracking_bool,
            raw_detection=detection,
            filtered_centroid=filtered_pos,
            filtered_velocity=filtered_vel,
            is_predicted=is_predicted,
            consecutive_detections=self.state_machine.consecutive_detections,
            consecutive_losses=self.state_machine.consecutive_losses,
            timestamp=timestamp,
            frame_index=frame_index,
            total_processing_time_ms=total_time_ms,
        )

        self.last_output = output
        return output
