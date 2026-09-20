"""
Integration tests for VisionPipeline.
Validates the complete closed vision loop across moving beacon sequences.
"""

import pytest
import numpy as np
from backend.vision.pipeline import VisionPipeline
from backend.engine import SimulationEngine
from backend.config.simulation_config import SimulationConfig, BeaconConfig
from backend.models.tracking_state import TrackingState


def test_pipeline_static_beacon():
    """Verify pipeline locks onto stationary beacon and transitions to TRACKING."""
    sim_cfg = SimulationConfig(
        beacon=BeaconConfig(
            initial_pos=(1000.0, 1000.0),
            size=10.0,
            motion_type="straight_line",
            motion_params={"speed": 0.0},
        )
    )
    engine = SimulationEngine(sim_cfg)
    pipeline = VisionPipeline()

    for step in range(10):
        frame, gt_state = engine.step()
        output = pipeline.process_frame(frame, dt=engine.dt, timestamp=gt_state.timestamp, frame_index=step)

        if step == 0:
            assert output.state == TrackingState.ACQUIRED
        elif step >= 3:
            assert output.state == TrackingState.TRACKING
            assert output.filtered_centroid is not None
            assert pytest.approx(output.filtered_centroid[0], abs=0.5) == 320.0
            assert pytest.approx(output.filtered_centroid[1], abs=0.5) == 240.0


def test_pipeline_moving_circular_beacon():
    """Verify pipeline continuously tracks a moving circular beacon."""
    sim_cfg = SimulationConfig(
        beacon=BeaconConfig(
            initial_pos=(1000.0, 1000.0),
            size=10.0,
            motion_type="circular",
            motion_params={"radius": 100.0, "angular_speed_deg_s": 30.0},
        )
    )
    engine = SimulationEngine(sim_cfg)
    pipeline = VisionPipeline()

    # Step for 30 frames (1.0 sec)
    for step in range(30):
        frame, gt_state = engine.step()
        output = pipeline.process_frame(frame, dt=engine.dt, timestamp=gt_state.timestamp, frame_index=step)

        if step >= 3:
            assert output.state == TrackingState.TRACKING
            assert output.raw_detection.is_detected is True
            assert output.filtered_centroid is not None

            # Verify detected and filtered centroid closely follow ground truth
            gt_u, gt_v = gt_state.beacon_image_pos
            assert pytest.approx(output.raw_detection.centroid[0], abs=1.0) == gt_u
            assert pytest.approx(output.raw_detection.centroid[1], abs=1.0) == gt_v
            assert pytest.approx(output.filtered_centroid[0], abs=2.5) == gt_u
            assert pytest.approx(output.filtered_centroid[1], abs=2.5) == gt_v


def test_pipeline_coasting_and_recovery():
    """Verify pipeline coasts when artificial blank frames are fed, then recovers when target returns."""
    pipeline = VisionPipeline()
    dt = 1.0 / 30.0

    # 1. Feed 5 frames with target at (320, 240) -> reach TRACKING
    frame_target = np.zeros((480, 640), dtype=np.uint8)
    frame_target[235:245, 315:325] = 255

    for i in range(5):
        output = pipeline.process_frame(frame_target, dt=dt, frame_index=i)
    assert output.state == TrackingState.TRACKING
    assert output.is_predicted is False

    # 2. Feed 2 blank frames (simulate brief dropout/obstruction) -> coasting
    frame_blank = np.zeros((480, 640), dtype=np.uint8)
    out_coast1 = pipeline.process_frame(frame_blank, dt=dt, frame_index=5)
    assert out_coast1.state == TrackingState.TRACKING
    assert out_coast1.is_predicted is True
    assert out_coast1.filtered_centroid is not None

    # 3. Feed target again -> resumes active measurement tracking
    out_rec = pipeline.process_frame(frame_target, dt=dt, frame_index=7)
    assert out_rec.state == TrackingState.TRACKING
    assert out_rec.is_predicted is False
    assert out_rec.raw_detection.is_detected is True
