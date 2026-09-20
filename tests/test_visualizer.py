"""
Tests for Phase 1 Simulation Visualizer.
Verifies rendering pipeline, coordinate transformations, and composite frame dimensions.
"""

import pytest
import numpy as np
from visualization.viewer import SimulationVisualizer
from backend.config.simulation_config import SimulationConfig, BeaconConfig


def test_visualizer_initialization():
    """Verify visualizer initializes cleanly with correct panel dimensions."""
    viz = SimulationVisualizer(target_fps=30.0)
    assert viz.world_panel_size == 520
    assert viz.cam_width == 640
    assert viz.cam_height == 480
    assert viz.total_width == 520 + 640 + 3 * 15  # 1205 px
    assert viz.total_height == 520 + 200 + 3 * 15  # 765 px (520 world + 200 HUD + 45 margins)


def test_visualizer_world_coord_mapping():
    """Verify world-to-panel coordinate scaling is accurate."""
    viz = SimulationVisualizer()
    world_w, world_h = 2000.0, 2000.0
    
    # Origin (0, 0)
    px, py = viz.world_to_panel_coords(0.0, 0.0, world_w, world_h)
    assert px == 0 and py == 0

    # Center (1000, 1000)
    px, py = viz.world_to_panel_coords(1000.0, 1000.0, world_w, world_h)
    assert abs(px - 259.5) <= 0.5
    assert abs(py - 259.5) <= 0.5

    # Max (2000, 2000)
    px, py = viz.world_to_panel_coords(2000.0, 2000.0, world_w, world_h)
    assert px == viz.world_panel_size - 1
    assert py == viz.world_panel_size - 1


def test_visualizer_step_and_render():
    """Verify visualizer step_and_render produces valid composite dashboard image."""
    config = SimulationConfig(
        beacon=BeaconConfig(
            initial_pos=(1000.0, 1000.0),
            motion_type="circular",
        )
    )
    viz = SimulationVisualizer(config=config)
    dashboard, state, tracker_output = viz.step_and_render()

    assert isinstance(dashboard, np.ndarray)
    assert dashboard.shape == (viz.total_height, viz.total_width, 3)
    assert dashboard.dtype == np.uint8
    assert state.frame_index == 1
    assert tracker_output is not None
    assert len(viz.beacon_trail) == 1


def test_visualizer_motion_switching():
    """Verify switching motion models resets engine state and updates trajectory."""
    viz = SimulationVisualizer()
    for motion in ["straight_line", "circular", "figure_eight", "random"]:
        viz.reset(motion)
        assert viz.current_motion == motion
        dashboard, state, tracker_output = viz.step_and_render()
        assert dashboard is not None
        assert state.frame_index == 1
        assert tracker_output is not None
