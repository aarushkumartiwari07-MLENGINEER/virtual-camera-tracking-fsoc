"""
Integration tests for SimulationEngine (Phase 1 end-to-end simulation loop).
"""

import pytest
import numpy as np
from backend.engine import SimulationEngine
from backend.config.simulation_config import (
    SimulationConfig,
    BeaconConfig,
    CameraConfig,
    WorldConfig,
)


def test_engine_initialization_and_reset():
    """Verify engine starts cleanly with correct initial state."""
    config = SimulationConfig(
        fps=30.0,
        world=WorldConfig(width=2000.0, height=2000.0),
        beacon=BeaconConfig(initial_pos=(1000.0, 1000.0), size=10.0),
        camera=CameraConfig(initial_pos=(1000.0, 1000.0)),
    )
    engine = SimulationEngine(config)
    state = engine.get_state()

    assert state.timestamp == 0.0
    assert state.frame_index == 0
    assert state.is_visible is True
    assert pytest.approx(state.beacon_image_pos[0], abs=1e-4) == 320.0
    assert pytest.approx(state.beacon_image_pos[1], abs=1e-4) == 240.0


def test_engine_step_loop():
    """Verify engine steps advance simulation time, frame counter, and generate valid image frames."""
    config = SimulationConfig(
        fps=30.0,
        beacon=BeaconConfig(
            initial_pos=(1000.0, 1000.0),
            motion_type="straight_line",
            motion_params={"speed": 30.0, "angle_deg": 0.0},
        ),
    )
    engine = SimulationEngine(config)

    for step in range(30):
        frame, state = engine.step()
        assert frame.shape == (480, 640)
        assert frame.dtype == np.uint8
        assert state.frame_index == step + 1
        assert pytest.approx(state.timestamp, abs=1e-5) == (step + 1) * (1.0 / 30.0)

    # After 30 steps (1.0 sec), beacon with speed 30 px/s should be at x = 1030
    assert pytest.approx(state.beacon_world_pos[0], abs=1e-3) == 1030.0
    assert pytest.approx(state.beacon_world_pos[1], abs=1e-3) == 1000.0
    assert pytest.approx(state.beacon_image_pos[0], abs=1e-3) == 350.0  # 320 + 30


def test_rendered_frame_contains_beacon():
    """Verify that a rendered frame contains non-zero intensity at the expected beacon image location."""
    config = SimulationConfig(
        beacon=BeaconConfig(
            initial_pos=(1000.0, 1000.0),
            size=10.0,
            shape="square",
            intensity=255.0,
        )
    )
    engine = SimulationEngine(config)
    frame, state = engine.step()

    assert state.is_visible is True
    u, v = int(round(state.beacon_image_pos[0])), int(round(state.beacon_image_pos[1]))
    assert frame[v, u] == 255
    # Corners of the frame should be black (background)
    assert frame[0, 0] == 0
    assert frame[479, 639] == 0


def test_gaussian_rendered_frame():
    """Verify Gaussian spot profile rendering."""
    config = SimulationConfig(
        beacon=BeaconConfig(
            initial_pos=(1000.0, 1000.0),
            size=12.0,
            shape="gaussian",
            intensity=255.0,
            motion_type="straight_line",
            motion_params={"speed": 0.0},
        )
    )
    engine = SimulationEngine(config)
    frame, state = engine.step()

    u, v = int(round(state.beacon_image_pos[0])), int(round(state.beacon_image_pos[1]))
    # Peak at center for stationary beacon on grid point
    assert frame[v, u] == 255
    # Gradual falloff nearby
    assert 0 < frame[v + 2, u + 2] < 255

