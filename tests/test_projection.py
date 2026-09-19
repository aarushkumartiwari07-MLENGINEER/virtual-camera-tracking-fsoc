"""
Tests for World-to-Camera projection and coordinate transformation mathematics.
"""

import pytest
import math
from backend.simulation.camera import Camera
from backend.simulation.beacon import Beacon
from backend.simulation.projection import Projector
from backend.config.simulation_config import CameraConfig, BeaconConfig


def test_camera_center_projection():
    """Verify beacon at world center projects exactly to camera image center (320, 240)."""
    camera = Camera(CameraConfig(
        resolution=(640, 480),
        fov_deg=(4.0, 3.0),
        initial_pos=(1000.0, 1000.0),
        initial_pan_deg=0.0,
        initial_tilt_deg=0.0,
    ))
    beacon = Beacon(BeaconConfig(
        initial_pos=(1000.0, 1000.0),
        size=10.0,
    ))

    is_vis, img_pos, bbox, ang_err, pix_err = Projector.project_beacon(beacon, camera)

    assert is_vis is True
    assert img_pos is not None
    assert pytest.approx(img_pos[0], abs=1e-5) == 320.0
    assert pytest.approx(img_pos[1], abs=1e-5) == 240.0
    assert pytest.approx(ang_err[0], abs=1e-5) == 0.0
    assert pytest.approx(ang_err[1], abs=1e-5) == 0.0
    assert pytest.approx(pix_err[0], abs=1e-5) == 0.0
    assert pytest.approx(pix_err[1], abs=1e-5) == 0.0


def test_angular_shift_projection():
    """Verify angular gimbal shift moves projected pixel position predictably."""
    camera = Camera(CameraConfig(
        resolution=(640, 480),
        fov_deg=(4.0, 3.0),
        initial_pos=(1000.0, 1000.0),
    ))
    beacon = Beacon(BeaconConfig(
        initial_pos=(1000.0, 1000.0),
        size=10.0,
    ))

    # Scale: 4 deg / 640 px = 0.00625 deg/px.
    # A pan of +1.0 deg shifts the target to the left in the camera frame by 1.0 / 0.00625 = 160 px.
    camera.set_pan_tilt_instant(pan_deg=1.0, tilt_deg=0.5)

    is_vis, img_pos, bbox, ang_err, pix_err = Projector.project_beacon(beacon, camera)

    assert is_vis is True
    assert pytest.approx(img_pos[0], abs=1e-4) == 320.0 - 160.0  # 160.0
    assert pytest.approx(img_pos[1], abs=1e-4) == 240.0 - 80.0   # 160.0
    assert pytest.approx(ang_err[0], abs=1e-5) == -1.0
    assert pytest.approx(ang_err[1], abs=1e-5) == -0.5


def test_beacon_outside_fov():
    """Verify a beacon located far outside the FOV is detected as not visible."""
    camera = Camera(CameraConfig(
        resolution=(640, 480),
        fov_deg=(4.0, 3.0),
        initial_pos=(1000.0, 1000.0),
        initial_pan_deg=0.0,
        initial_tilt_deg=0.0,
    ))
    # Place beacon far away at (100.0, 100.0) -> dx = -900, dy = -900 px
    beacon = Beacon(BeaconConfig(
        initial_pos=(100.0, 100.0),
        size=10.0,
    ))

    is_vis, img_pos, bbox, ang_err, pix_err = Projector.project_beacon(beacon, camera)

    assert is_vis is False
    assert img_pos is not None
    # u = 320 - 900 = -580.0 px (well outside [0, 640))
    assert img_pos[0] < 0.0 or img_pos[0] >= 640.0
    assert img_pos[1] < 0.0 or img_pos[1] >= 480.0


def test_projection_invertibility():
    """Verify image_to_angular is exact inverse of angular_to_image."""
    scale_x = 4.0 / 640.0
    scale_y = 3.0 / 480.0
    pp = (320.0, 240.0)

    test_angles = [(-1.5, 0.8), (0.0, 0.0), (1.9, -1.4), (0.5, -0.2)]
    for az, el in test_angles:
        u, v = Projector.angular_to_image(az, el, scale_x, scale_y, pp)
        rec_az, rec_el = Projector.image_to_angular(u, v, scale_x, scale_y, pp)
        assert pytest.approx(rec_az, abs=1e-6) == az
        assert pytest.approx(rec_el, abs=1e-6) == el


def test_partial_edge_visibility():
    """Verify beacon partially intersecting boundary is marked visible."""
    camera = Camera(CameraConfig(resolution=(640, 480), fov_deg=(4.0, 3.0), initial_pos=(1000.0, 1000.0)))
    # Place beacon so its center is just outside right edge (u = 642) but size is 10 (extends 637 to 647)
    # dx = 322 px -> u = 320 + 322 = 642 px
    beacon = Beacon(BeaconConfig(initial_pos=(1322.0, 1000.0), size=10.0))

    is_vis, img_pos, bbox, ang_err, pix_err = Projector.project_beacon(beacon, camera)

    assert is_vis is True
    assert bbox[0] == 637.0
    assert bbox[2] == 647.0
    # Right border is 640, so part of the bbox [637, 640) is within the sensor!


def test_screenshot_equivalent_case():
    """
    Verify the exact benchmark case:
    Camera = (1000.0, 1000.0), Pan/Tilt = (0.0, 0.0)
    Beacon = (1043.1, 861.4)
    Yields image projection u = 363.1, v = 101.4.
    """
    camera = Camera(CameraConfig(
        resolution=(640, 480),
        fov_deg=(4.0, 3.0),
        initial_pos=(1000.0, 1000.0),
        initial_pan_deg=0.0,
        initial_tilt_deg=0.0,
    ))
    beacon = Beacon(BeaconConfig(
        initial_pos=(1043.1, 861.4),
        size=10.0,
    ))

    is_vis, img_pos, bbox, ang_err, pix_err = Projector.project_beacon(beacon, camera)

    assert is_vis is True
    assert pytest.approx(img_pos[0], abs=1e-3) == 363.1
    assert pytest.approx(img_pos[1], abs=1e-3) == 101.4
    assert pytest.approx(pix_err[0], abs=1e-3) == +43.1
    assert pytest.approx(pix_err[1], abs=1e-3) == -138.6
    assert pytest.approx(ang_err[0], abs=1e-4) == +43.1 * (4.0 / 640.0)  # +0.269375 deg
    assert pytest.approx(ang_err[1], abs=1e-4) == -138.6 * (3.0 / 480.0) # -0.86625 deg


def test_world_fov_footprint_consistency():
    """
    Verify that any beacon strictly inside the 640x480 world-plane FOV footprint
    is marked visible in image sensor space, and any point outside is marked not visible.
    """
    camera = Camera(CameraConfig(
        resolution=(640, 480),
        fov_deg=(4.0, 3.0),
        initial_pos=(1000.0, 1000.0),
        initial_pan_deg=0.0,
        initial_tilt_deg=0.0,
    ))

    # Corner 1: Inside top-left world footprint (1000-310, 1000-230) = (690, 770)
    b_inside = Beacon(BeaconConfig(initial_pos=(690.0, 770.0), size=10.0))
    is_vis, img_pos, _, _, _ = Projector.project_beacon(b_inside, camera)
    assert is_vis is True
    assert 0 <= img_pos[0] < 640 and 0 <= img_pos[1] < 480

    # Corner 2: Outside world footprint (1000-330, 1000-250) = (670, 750)
    b_outside = Beacon(BeaconConfig(initial_pos=(660.0, 740.0), size=10.0))
    is_vis, img_pos, _, _, _ = Projector.project_beacon(b_outside, camera)
    assert is_vis is False

