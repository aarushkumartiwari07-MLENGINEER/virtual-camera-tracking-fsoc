"""
Tracking Finite State Machine (FSM) for FSOC coarse alignment beacon tracking.
Manages states: SEARCHING, ACQUIRED, TRACKING, LOST, REACQUIRED.
"""

from typing import Optional
from ..config.vision_config import StateMachineConfig
from ..models.tracking_state import TrackingState


class TrackingStateMachine:
    """
    Manages track lock lifecycle across discrete frame observations.
    """

    def __init__(self, config: Optional[StateMachineConfig] = None) -> None:
        self.config = config or StateMachineConfig()
        self.current_state: TrackingState = TrackingState.SEARCHING
        self.consecutive_detections: int = 0
        self.consecutive_losses: int = 0
        self.total_frames: int = 0
        self.total_detected_frames: int = 0

    def reset(self) -> None:
        """Reset state machine to initial SEARCHING state."""
        self.current_state = TrackingState.SEARCHING
        self.consecutive_detections = 0
        self.consecutive_losses = 0
        self.total_frames = 0
        self.total_detected_frames = 0

    def update(self, is_detected: bool) -> TrackingState:
        """
        Advance state machine with current frame detection outcome.

        Args:
            is_detected: True if beacon was successfully detected in current frame.

        Returns:
            Updated TrackingState.
        """
        self.total_frames += 1

        if is_detected:
            self.total_detected_frames += 1
            self.consecutive_detections += 1
            self.consecutive_losses = 0

            if self.current_state == TrackingState.SEARCHING:
                # First valid detection establishes acquisition candidate
                self.current_state = TrackingState.ACQUIRED

            elif self.current_state == TrackingState.ACQUIRED:
                if self.consecutive_detections >= self.config.consecutive_acquire_frames:
                    self.current_state = TrackingState.TRACKING

            elif self.current_state == TrackingState.TRACKING:
                # Maintain active tracking lock
                pass

            elif self.current_state == TrackingState.LOST:
                # Target reappeared after complete loss
                self.current_state = TrackingState.REACQUIRED

            elif self.current_state == TrackingState.REACQUIRED:
                if self.consecutive_detections >= self.config.consecutive_reacquire_frames:
                    self.current_state = TrackingState.TRACKING

        else:
            # Detection missed in this frame
            self.consecutive_losses += 1
            self.consecutive_detections = 0

            if self.current_state == TrackingState.SEARCHING:
                # Remain in search
                pass

            elif self.current_state == TrackingState.ACQUIRED:
                # Failed to confirm lock, revert to search
                self.current_state = TrackingState.SEARCHING

            elif self.current_state == TrackingState.TRACKING:
                # Coast with Kalman prediction up to max_lost_coasting_frames
                if self.consecutive_losses > self.config.max_lost_coasting_frames:
                    self.current_state = TrackingState.LOST

            elif self.current_state == TrackingState.LOST:
                # Remain lost
                pass

            elif self.current_state == TrackingState.REACQUIRED:
                # Failed to re-confirm lock, revert to lost
                self.current_state = TrackingState.LOST

        return self.current_state
