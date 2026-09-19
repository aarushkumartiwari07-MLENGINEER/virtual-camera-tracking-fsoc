"""
Tests for beacon motion models:
1. Straight-Line Motion
2. Circular Motion
3. Figure-of-Eight Motion
4. Random Motion
"""

import pytest
import math
import numpy as np
from backend.simulation.motion import (
    StraightLineMotion,
    CircularMotion,
    FigureEightMotion,
    RandomMotion,
    create_motion_model,
)


def test_straight_line_motion():
    """Verify straight line motion updates position predictably with constant velocity."""
    speed = 50.0  # px/s
    angle_deg = 0.0  # purely horizontal (+x)
    motion = StraightLineMotion(speed=speed, angle_deg=angle_deg, bounce=False)
    
    pos = (500.0, 500.0)
    dt = 0.1
    world_bounds = (2000.0, 2000.0)

    for i in range(10):
        pos, vel = motion.update(pos, dt, sim_time=i * dt, world_bounds=world_bounds)
    
    # After 1.0 second, moved 50 px in x, 0 px in y
    assert pytest.approx(pos[0], abs=1e-4) == 550.0
    assert pytest.approx(pos[1], abs=1e-4) == 500.0
    assert pytest.approx(vel[0], abs=1e-4) == 50.0
    assert pytest.approx(vel[1], abs=1e-4) == 0.0


def test_straight_line_bounce():
    """Verify straight line rebounds off world boundaries when bounce is enabled."""
    speed = 100.0
    angle_deg = 180.0  # moving left towards x = 0
    motion = StraightLineMotion(speed=speed, angle_deg=angle_deg, bounce=True)

    pos = (50.0, 500.0)
    dt = 1.0  # would move to -50.0, bounces to +50.0 with vx > 0
    world_bounds = (2000.0, 2000.0)

    pos, vel = motion.update(pos, dt, sim_time=0.0, world_bounds=world_bounds)
    assert pytest.approx(pos[0], abs=1e-4) == 50.0
    assert vel[0] > 0  # Rebounded


def test_circular_motion_radius_and_period():
    """Verify circular motion maintains constant radius R and completes period."""
    center = (1000.0, 1000.0)
    radius = 200.0
    angular_speed = 360.0 / 10.0  # 36 deg/s -> period = 10 s
    motion = CircularMotion(
        center=center,
        radius=radius,
        angular_speed_deg_s=angular_speed,
        initial_phase_deg=0.0,
    )
    motion.reset((1200.0, 1000.0))

    dt = 0.1
    pos = (1200.0, 1000.0)
    world_bounds = (2000.0, 2000.0)

    # Check radius preservation over 100 steps (1 full revolution)
    for step in range(101):
        t = step * dt
        pos, vel = motion.update(pos, dt, sim_time=t, world_bounds=world_bounds)
        dist = math.hypot(pos[0] - center[0], pos[1] - center[1])
        assert pytest.approx(dist, abs=1e-3) == radius

    # At t = 10.0 s (full circle), position should return to initial (1200, 1000)
    assert pytest.approx(pos[0], abs=1e-3) == 1200.0
    assert pytest.approx(pos[1], abs=1e-3) == 1000.0


def test_figure_eight_motion():
    """Verify figure-eight motion produces dual-lobe trajectory with correct period."""
    center = (1000.0, 1000.0)
    amp_x = 300.0
    amp_y = 200.0
    period = 8.0
    motion = FigureEightMotion(
        center=center,
        amplitude_x=amp_x,
        amplitude_y=amp_y,
        period_s=period,
        initial_phase_deg=0.0,
    )

    pos = center
    world_bounds = (2000.0, 2000.0)

    # At t=0: pos = center
    pos, vel = motion.update(pos, 0.0, sim_time=0.0, world_bounds=world_bounds)
    assert pytest.approx(pos[0], abs=1e-3) == 1000.0
    assert pytest.approx(pos[1], abs=1e-3) == 1000.0

    # At t = T/4 = 2.0s: x = cx + Ax, y = cy (since sin(2*pi/2) = sin(pi) = 0)
    pos, vel = motion.update(pos, 2.0, sim_time=2.0, world_bounds=world_bounds)
    assert pytest.approx(pos[0], abs=1e-3) == 1000.0 + amp_x
    assert pytest.approx(pos[1], abs=1e-3) == 1000.0

    # At t = T = 8.0s: returns to center
    pos, vel = motion.update(pos, 6.0, sim_time=8.0, world_bounds=world_bounds)
    assert pytest.approx(pos[0], abs=1e-3) == 1000.0
    assert pytest.approx(pos[1], abs=1e-3) == 1000.0


def test_random_motion_bounded():
    """Verify random walk stays bounded within the virtual world limits over 500 steps."""
    motion = RandomMotion(max_speed=80.0, acceleration_std=40.0, seed=123)
    pos = (1000.0, 1000.0)
    motion.reset(pos)

    dt = 1.0 / 30.0
    world_bounds = (2000.0, 2000.0)

    for step in range(500):
        pos, vel = motion.update(pos, dt, sim_time=step * dt, world_bounds=world_bounds)
        assert 0.0 <= pos[0] <= 2000.0
        assert 0.0 <= pos[1] <= 2000.0
        speed = math.hypot(vel[0], vel[1])
        assert speed <= 80.0 + 1e-3


def test_motion_factory():
    """Verify factory instantiation for all supported motion model names."""
    m1 = create_motion_model("straight_line")
    assert isinstance(m1, StraightLineMotion)

    m2 = create_motion_model("circular")
    assert isinstance(m2, CircularMotion)

    m3 = create_motion_model("figure_eight")
    assert isinstance(m3, FigureEightMotion)

    m4 = create_motion_model("random")
    assert isinstance(m4, RandomMotion)

    with pytest.raises(ValueError):
        create_motion_model("unknown_motion_xyz")
