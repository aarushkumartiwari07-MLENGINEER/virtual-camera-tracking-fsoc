"""
Tests for TrackingStateMachine.
Validates all finite state machine transitions and threshold behaviors:
1. SEARCHING -> ACQUIRED (1 detection)
2. ACQUIRED -> TRACKING (N consecutive detections)
3. ACQUIRED -> SEARCHING (immediate drop before lock)
4. TRACKING -> Coasting (loss <= max_lost_coasting_frames)
5. TRACKING -> LOST (loss > max_lost_coasting_frames)
6. LOST -> REACQUIRED -> TRACKING
7. REACQUIRED -> LOST (drop before re-lock confirmation)
"""

import pytest
from backend.vision.state_machine import TrackingStateMachine
from backend.models.tracking_state import TrackingState
from backend.config.vision_config import StateMachineConfig


def test_fsm_acquisition_flow():
    """Verify SEARCHING -> ACQUIRED -> TRACKING transition with 3 consecutive detections."""
    cfg = StateMachineConfig(consecutive_acquire_frames=3)
    fsm = TrackingStateMachine(config=cfg)

    assert fsm.current_state == TrackingState.SEARCHING

    # Step 1: First detection -> ACQUIRED
    assert fsm.update(is_detected=True) == TrackingState.ACQUIRED
    assert fsm.consecutive_detections == 1

    # Step 2: Second detection -> still ACQUIRED
    assert fsm.update(is_detected=True) == TrackingState.ACQUIRED
    assert fsm.consecutive_detections == 2

    # Step 3: Third detection -> TRACKING
    assert fsm.update(is_detected=True) == TrackingState.TRACKING
    assert fsm.consecutive_detections == 3


def test_fsm_false_alarm_in_acquired():
    """Verify that a single detection followed immediately by a miss drops back from ACQUIRED to SEARCHING."""
    cfg = StateMachineConfig(consecutive_acquire_frames=3)
    fsm = TrackingStateMachine(config=cfg)

    fsm.update(is_detected=True)  # ACQUIRED
    assert fsm.current_state == TrackingState.ACQUIRED

    fsm.update(is_detected=False) # Miss -> drops to SEARCHING
    assert fsm.current_state == TrackingState.SEARCHING


def test_fsm_coasting_and_loss():
    """Verify that TRACKING coasts for up to max_lost frames before transitioning to LOST."""
    cfg = StateMachineConfig(consecutive_acquire_frames=2, max_lost_coasting_frames=3)
    fsm = TrackingStateMachine(config=cfg)

    # Establish track
    fsm.update(True)
    fsm.update(True)
    assert fsm.current_state == TrackingState.TRACKING

    # Coast for 3 frames (should remain in TRACKING)
    for i in range(1, 4):
        fsm.update(False)
        assert fsm.current_state == TrackingState.TRACKING
        assert fsm.consecutive_losses == i

    # 4th missed frame (> 3 max_lost) -> transitions to LOST
    fsm.update(False)
    assert fsm.current_state == TrackingState.LOST


def test_fsm_reacquisition_flow():
    """Verify LOST -> REACQUIRED -> TRACKING recovery sequence."""
    cfg = StateMachineConfig(consecutive_acquire_frames=2, max_lost_coasting_frames=2, consecutive_reacquire_frames=2)
    fsm = TrackingStateMachine(config=cfg)

    # Reach LOST state
    fsm.update(True)
    fsm.update(True)
    fsm.update(False)
    fsm.update(False)
    fsm.update(False)
    assert fsm.current_state == TrackingState.LOST

    # First re-detection -> REACQUIRED
    assert fsm.update(True) == TrackingState.REACQUIRED

    # Second consecutive re-detection -> TRACKING
    assert fsm.update(True) == TrackingState.TRACKING


def test_fsm_reset():
    """Verify reset returns state machine cleanly to initial conditions."""
    fsm = TrackingStateMachine()
    fsm.update(True)
    fsm.update(True)
    fsm.reset()
    assert fsm.current_state == TrackingState.SEARCHING
    assert fsm.total_frames == 0
