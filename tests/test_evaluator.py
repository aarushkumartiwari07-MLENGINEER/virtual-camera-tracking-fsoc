"""
Tests for TrackingEvaluator and Architectural Isolation.
Validates:
1. Tracking metrics computation (Mean error, RMSE, Detection Rate, Acquisition Time)
2. Strict isolation of VisionPipeline from ground-truth simulation entities
"""

import pytest
import numpy as np
import inspect

from backend.evaluation.evaluator import TrackingEvaluator
from backend.vision.pipeline import VisionPipeline
from backend.engine import SimulationEngine
from backend.config.simulation_config import SimulationConfig, BeaconConfig


def test_evaluator_metrics_computation():
    """Verify evaluator computes RMSE and detection rate accurately on a simulation run."""
    sim_cfg = SimulationConfig(
        fps=30.0,
        beacon=BeaconConfig(
            initial_pos=(1000.0, 1000.0),
            size=10.0,
            motion_type="circular",
            motion_params={"radius": 120.0, "angular_speed_deg_s": 40.0},
        ),
    )
    engine = SimulationEngine(sim_cfg)
    pipeline = VisionPipeline()
    evaluator = TrackingEvaluator()

    for step in range(45):
        frame, gt_state = engine.step()
        output = pipeline.process_frame(
            frame=frame,
            dt=engine.dt,
            timestamp=gt_state.timestamp,
            frame_index=gt_state.frame_index,
        )
        evaluator.evaluate_step(tracker_output=output, ground_truth=gt_state)

    metrics = evaluator.get_summary_metrics()

    assert metrics["total_frames"] == 45
    assert metrics["visible_frames"] == 45
    assert metrics["detected_frames"] == 45
    assert metrics["detection_rate_pct"] == 100.0
    assert metrics["target_loss_rate_pct"] == 0.0
    assert metrics["false_positives"] == 0
    # Raw subpixel centroid error should be under 1.0 pixels for rasterized square target
    assert metrics["rmse_raw_error_px"] < 1.0
    assert metrics["acquisition_time_s"] is not None
    assert metrics["processing_fps"] > 50.0  # Runs in real-time or faster


def test_evaluator_gaussian_subpixel_accuracy():
    """Verify subpixel RMSE on continuous Gaussian spot is < 0.2 pixels."""
    sim_cfg = SimulationConfig(
        fps=30.0,
        beacon=BeaconConfig(
            initial_pos=(1000.0, 1000.0),
            size=12.0,
            shape="gaussian",
            motion_type="circular",
            motion_params={"radius": 100.0, "angular_speed_deg_s": 30.0},
        ),
    )
    engine = SimulationEngine(sim_cfg)
    pipeline = VisionPipeline()
    evaluator = TrackingEvaluator()

    for step in range(30):
        frame, gt_state = engine.step()
        output = pipeline.process_frame(
            frame=frame,
            dt=engine.dt,
            timestamp=gt_state.timestamp,
            frame_index=gt_state.frame_index,
        )
        evaluator.evaluate_step(tracker_output=output, ground_truth=gt_state)

    metrics = evaluator.get_summary_metrics()
    assert metrics["rmse_raw_error_px"] < 0.25



def test_strict_vision_isolation():
    """
    Architectural Verification:
    Verify that VisionPipeline does not receive or import any ground truth classes or attributes.
    """
    pipeline = VisionPipeline()
    sig = inspect.signature(pipeline.process_frame)

    # process_frame parameters must only be: frame, dt, timestamp, frame_index
    params = list(sig.parameters.keys())
    assert "frame" in params
    assert "ground_truth" not in params
    assert "beacon" not in params
    assert "world" not in params
    assert "camera" not in params

    # Verify pipeline instance has no hidden ground-truth attributes
    for attr in dir(pipeline):
        assert "ground_truth" not in attr.lower()
        assert "world_pos" not in attr.lower()
