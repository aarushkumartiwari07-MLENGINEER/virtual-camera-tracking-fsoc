"""
Tests for Configurable Camera Resolution and FOV.
Validates:
1. 640x480, 4.0° x 3.0° (Default)
2. 1280x720, 8.0° x 4.0° (Wide Aspect / High Resolution)
3. 800x600, 5.0° x 4.0° (4:3 Aspect / Custom Resolution)
"""

import pytest
import math
from backend.simulation.camera import Camera
from backend.simulation.beacon import Beacon
from backend.simulation.projection import Projector
from backend.simulation.renderer import FrameRenderer
from backend.engine import SimulationEngine
from backend.config.simulation_config import (
    SimulationConfig,
    CameraConfig,
    BeaconConfig,
    WorldConfig,
)
from visualization.viewer import SimulationVisualizer


CAMERA_TEST_CONFIGS = [
    {
        "name": "Default 640x480 (4°x3°)",
        "resolution": (640, 480),
        "fov_deg": (4.0, 3.0),
        "expected_scale_x": 4.0 / 640.0,
        "expected_scale_y": 3.0 / 480.0,
        "expected_pp": (320.0, 240.0),
        "test_angular_offset": (1.0, 0.5), # dAz=+1.0 deg, dEl=+0.5 deg
        "expected_pixel_shift": (1.0 / (4.0 / 640.0), 0.5 / (3.0 / 480.0)), # (160.0, 80.0)
    },
    {
        "name": "Wide HD 1280x720 (8°x4°)",
        "resolution": (1280, 720),
        "fov_deg": (8.0, 4.0),
        "expected_scale_x": 8.0 / 1280.0, # 0.00625 deg/px
        "expected_scale_y": 4.0 / 720.0,  # 0.00555555... deg/px
        "expected_pp": (640.0, 360.0),
        "test_angular_offset": (2.0, -1.0), # dAz=+2.0 deg, dEl=-1.0 deg
        "expected_pixel_shift": (2.0 / (8.0 / 1280.0), -1.0 / (4.0 / 720.0)), # (320.0, -180.0)
    },
    {
        "name": "Custom 800x600 (5°x4°)",
        "resolution": (800, 600),
        "fov_deg": (5.0, 4.0),
        "expected_scale_x": 5.0 / 800.0, # 0.00625 deg/px
        "expected_scale_y": 4.0 / 600.0, # 0.00666666... deg/px
        "expected_pp": (400.0, 300.0),
        "test_angular_offset": (-1.25, 1.0),
        "expected_pixel_shift": (-1.25 / (5.0 / 800.0), 1.0 / (4.0 / 600.0)), # (-200.0, 150.0)
    },
]


@pytest.mark.parametrize("cfg", CAMERA_TEST_CONFIGS, ids=lambda c: c["name"])
def test_camera_scales_and_principal_point(cfg):
    """Verify Sx, Sy, and principal point match resolution and FOV definitions."""
    camera = Camera(CameraConfig(
        resolution=cfg["resolution"],
        fov_deg=cfg["fov_deg"],
        initial_pos=(1000.0, 1000.0),
    ))

    assert camera.width == cfg["resolution"][0]
    assert camera.height == cfg["resolution"][1]
    assert pytest.approx(camera.scale_x_deg_px, abs=1e-7) == cfg["expected_scale_x"]
    assert pytest.approx(camera.scale_y_deg_px, abs=1e-7) == cfg["expected_scale_y"]
    assert pytest.approx(camera.principal_point[0], abs=1e-7) == cfg["expected_pp"][0]
    assert pytest.approx(camera.principal_point[1], abs=1e-7) == cfg["expected_pp"][1]


@pytest.mark.parametrize("cfg", CAMERA_TEST_CONFIGS, ids=lambda c: c["name"])
def test_camera_center_and_angular_projection(cfg):
    """Verify camera center projects to (W/2, H/2) and angular offsets project accurately."""
    camera = Camera(CameraConfig(
        resolution=cfg["resolution"],
        fov_deg=cfg["fov_deg"],
        initial_pos=(1000.0, 1000.0),
        initial_pan_deg=0.0,
        initial_tilt_deg=0.0,
    ))

    # 1. Target at camera center
    beacon_center = Beacon(BeaconConfig(initial_pos=(1000.0, 1000.0), size=10.0))
    is_vis, img_pos, bbox, ang_err, pix_err = Projector.project_beacon(beacon_center, camera)

    assert is_vis is True
    assert pytest.approx(img_pos[0], abs=1e-4) == cfg["expected_pp"][0]
    assert pytest.approx(img_pos[1], abs=1e-4) == cfg["expected_pp"][1]
    assert pytest.approx(pix_err[0], abs=1e-4) == 0.0
    assert pytest.approx(pix_err[1], abs=1e-4) == 0.0
    assert pytest.approx(ang_err[0], abs=1e-4) == 0.0
    assert pytest.approx(ang_err[1], abs=1e-4) == 0.0

    # 2. Known angular offset via beacon placement
    d_az, d_el = cfg["test_angular_offset"]
    dx_world = d_az / cfg["expected_scale_x"]
    dy_world = d_el / cfg["expected_scale_y"]
    beacon_offset = Beacon(BeaconConfig(initial_pos=(1000.0 + dx_world, 1000.0 + dy_world), size=10.0))

    is_vis, img_pos, bbox, ang_err, pix_err = Projector.project_beacon(beacon_offset, camera)

    expected_u = cfg["expected_pp"][0] + cfg["expected_pixel_shift"][0]
    expected_v = cfg["expected_pp"][1] + cfg["expected_pixel_shift"][1]

    assert is_vis is True
    assert pytest.approx(img_pos[0], abs=1e-3) == expected_u
    assert pytest.approx(img_pos[1], abs=1e-3) == expected_v
    assert pytest.approx(ang_err[0], abs=1e-4) == d_az
    assert pytest.approx(ang_err[1], abs=1e-4) == d_el


@pytest.mark.parametrize("cfg", CAMERA_TEST_CONFIGS, ids=lambda c: c["name"])
def test_projection_invertibility_configurable(cfg):
    """Verify image_to_angular and angular_to_image are strictly invertible for any resolution/FOV."""
    scale_x = cfg["expected_scale_x"]
    scale_y = cfg["expected_scale_y"]
    pp = cfg["expected_pp"]

    test_pixels = [
        (0.0, 0.0),
        (pp[0], pp[1]),
        (cfg["resolution"][0] - 1.0, cfg["resolution"][1] - 1.0),
        (pp[0] + 55.5, pp[1] - 42.2),
    ]

    for u, v in test_pixels:
        az, el = Projector.image_to_angular(u, v, scale_x, scale_y, pp)
        u_rec, v_rec = Projector.angular_to_image(az, el, scale_x, scale_y, pp)
        assert pytest.approx(u_rec, abs=1e-6) == u
        assert pytest.approx(v_rec, abs=1e-6) == v


@pytest.mark.parametrize("cfg", CAMERA_TEST_CONFIGS, ids=lambda c: c["name"])
def test_fov_visibility_and_footprint_configurable(cfg):
    """Verify FOV visibility boundaries and world footprint match resolution and FOV."""
    cam_cfg = CameraConfig(
        resolution=cfg["resolution"],
        fov_deg=cfg["fov_deg"],
        initial_pos=(1000.0, 1000.0),
    )
    camera = Camera(cam_cfg)

    # Footprint half-dimensions in world units
    half_w_world = (cfg["fov_deg"][0] / 2.0) / cfg["expected_scale_x"] # exactly resolution[0] / 2
    half_h_world = (cfg["fov_deg"][1] / 2.0) / cfg["expected_scale_y"] # exactly resolution[1] / 2

    assert pytest.approx(half_w_world, abs=1e-5) == cfg["resolution"][0] / 2.0
    assert pytest.approx(half_h_world, abs=1e-5) == cfg["resolution"][1] / 2.0

    # Inside test point: near top-left border (10 px inside)
    b_inside = Beacon(BeaconConfig(
        initial_pos=(1000.0 - half_w_world + 10.0, 1000.0 - half_h_world + 10.0),
        size=10.0,
    ))
    is_vis, img_pos, _, _, _ = Projector.project_beacon(b_inside, camera)
    assert is_vis is True
    assert 0 <= img_pos[0] < cfg["resolution"][0]
    assert 0 <= img_pos[1] < cfg["resolution"][1]

    # Outside test point: 50 px outside left border
    b_outside = Beacon(BeaconConfig(
        initial_pos=(1000.0 - half_w_world - 50.0, 1000.0),
        size=10.0,
    ))
    is_vis, img_pos, _, _, _ = Projector.project_beacon(b_outside, camera)
    assert is_vis is False


@pytest.mark.parametrize("cfg", CAMERA_TEST_CONFIGS, ids=lambda c: c["name"])
def test_renderer_and_visualizer_configurable(cfg):
    """Verify FrameRenderer and SimulationVisualizer generate correct frame dimensions."""
    sim_cfg = SimulationConfig(
        fps=30.0,
        camera=CameraConfig(
            resolution=cfg["resolution"],
            fov_deg=cfg["fov_deg"],
            initial_pos=(1000.0, 1000.0),
        ),
        beacon=BeaconConfig(initial_pos=(1000.0, 1000.0), size=10.0),
    )
    engine = SimulationEngine(sim_cfg)
    frame, state = engine.step()

    # Raw sensor frame dimensions
    assert frame.shape == (cfg["resolution"][1], cfg["resolution"][0])
    assert state.is_visible is True

    # Visualizer dimensions
    viz = SimulationVisualizer(config=sim_cfg)
    dashboard, _ = viz.step_and_render()
    expected_viz_w = viz.world_panel_size + cfg["resolution"][0] + 3 * viz.margin
    expected_viz_h = max(viz.world_panel_size, cfg["resolution"][1]) + viz.hud_height + 3 * viz.margin

    assert dashboard.shape == (expected_viz_h, expected_viz_w, 3)
