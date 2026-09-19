"""
Tests for Virtual Camera kinematics, pan/tilt gimbal limits, and update dynamics.
"""

import pytest
from backend.simulation.camera import Camera
from backend.config.simulation_config import CameraConfig


def test_camera_initialization():
    """Verify camera initializes with default PS parameters."""
    camera = Camera()
    assert camera.width == 640
    assert camera.height == 480
    assert camera.fov_deg == (4.0, 3.0)
    assert camera.pan_deg == 0.0
    assert camera.tilt_deg == 0.0
    assert camera.position == (1000.0, 1000.0)
    assert camera.principal_point == (320.0, 240.0)
    assert camera.scale_x_deg_px == pytest.approx(4.0 / 640.0)
    assert camera.scale_y_deg_px == pytest.approx(3.0 / 480.0)


def test_instant_pan_tilt():
    """Verify immediate gimbal repositioning."""
    camera = Camera()
    camera.set_pan_tilt_instant(1.5, -0.75)
    assert camera.pan_deg == 1.5
    assert camera.tilt_deg == -0.75


def test_rate_limited_pan_tilt():
    """Verify camera gimbal respects max pan/tilt slewing rate limits."""
    camera = Camera(CameraConfig(
        max_pan_speed_deg_s=10.0,
        max_tilt_speed_deg_s=10.0,
    ))
    camera.set_target_pan_tilt(pan_deg=5.0, tilt_deg=-5.0)

    # In dt = 0.1s, max angular step is 10.0 * 0.1 = 1.0 deg
    camera.update(dt=0.1)
    assert pytest.approx(camera.pan_deg, abs=1e-4) == 1.0
    assert pytest.approx(camera.tilt_deg, abs=1e-4) == -1.0

    # After 0.4 more seconds (total 0.5s), pan reaches 5.0 and stops
    for _ in range(4):
        camera.update(dt=0.1)
    assert pytest.approx(camera.pan_deg, abs=1e-4) == 5.0
    assert pytest.approx(camera.tilt_deg, abs=1e-4) == -5.0

    # Further updates do not overshoot target
    camera.update(dt=0.1)
    assert pytest.approx(camera.pan_deg, abs=1e-4) == 5.0
    assert pytest.approx(camera.tilt_deg, abs=1e-4) == -5.0


def test_command_velocity_saturation():
    """Verify commanded angular velocity saturates at max limits."""
    camera = Camera(CameraConfig(
        max_pan_speed_deg_s=10.0,
        max_tilt_speed_deg_s=10.0,
    ))
    # Command 50 deg/s (exceeds 10 deg/s limit)
    camera.command_velocity(pan_rate_deg_s=50.0, tilt_rate_deg_s=-25.0)
    assert camera.pan_rate_deg_s == 10.0
    assert camera.tilt_rate_deg_s == -10.0

    camera.update(dt=0.1)
    assert pytest.approx(camera.pan_deg, abs=1e-4) == 1.0
    assert pytest.approx(camera.tilt_deg, abs=1e-4) == -1.0
