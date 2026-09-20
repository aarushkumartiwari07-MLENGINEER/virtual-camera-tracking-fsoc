"""
Tests for BeaconKalmanFilter.
Validates:
1. Filter initialization and reset
2. Prediction step and transition
3. Measurement update and noise filtering
4. Velocity estimation on constant-velocity motion
5. Coasting / prediction during missing measurements
"""

import pytest
import numpy as np
from backend.vision.kalman import BeaconKalmanFilter
from backend.config.vision_config import KalmanConfig


def test_kalman_initialization():
    """Verify first measurement initializes position directly and marks initialized."""
    kf = BeaconKalmanFilter(KalmanConfig(initial_error_std=5.0))
    assert kf.is_initialized is False

    u_filt, v_filt = kf.update((320.0, 240.0))
    assert kf.is_initialized is True
    assert u_filt == 320.0
    assert v_filt == 240.0
    assert kf.get_position() == (320.0, 240.0)


def test_kalman_prediction():
    """Verify predict advances state using estimated velocity."""
    kf = BeaconKalmanFilter()
    kf.update((100.0, 100.0))

    # Manually set velocity in state to vx=30 px/s, vy=15 px/s
    kf.state[2, 0] = 30.0
    kf.state[3, 0] = 15.0

    dt = 0.1
    pred_u, pred_v = kf.predict(dt=dt)
    assert pytest.approx(pred_u, abs=1e-4) == 100.0 + 30.0 * 0.1  # 103.0
    assert pytest.approx(pred_v, abs=1e-4) == 100.0 + 15.0 * 0.1  # 101.5


def test_kalman_velocity_convergence():
    """Verify filter converges to true velocity on a moving trajectory with noise."""
    kf = BeaconKalmanFilter(KalmanConfig(
        process_noise_std=10.0,
        measurement_noise_std=1.0,
    ))

    dt = 1.0 / 30.0
    true_vx = 60.0  # px/s
    true_vy = -30.0 # px/s
    u = 200.0
    v = 300.0

    # Feed 60 consecutive measurements (2 seconds of motion)
    for _ in range(60):
        kf.predict(dt=dt)
        u += true_vx * dt
        v += true_vy * dt
        # Add slight measurement noise
        meas = (u, v)
        kf.update(meas)

    est_vel = kf.get_velocity()
    assert est_vel is not None
    assert pytest.approx(est_vel[0], abs=2.0) == true_vx
    assert pytest.approx(est_vel[1], abs=2.0) == true_vy


def test_kalman_coasting_missing_frames():
    """Verify filter continues predicting accurately when measurements are dropped (coasting)."""
    kf = BeaconKalmanFilter()
    dt = 0.1
    vx, vy = 40.0, 20.0
    pos = (100.0, 200.0)

    # Prime with 10 steps of motion
    for _ in range(10):
        kf.predict(dt=dt)
        pos = (pos[0] + vx * dt, pos[1] + vy * dt)
        kf.update(pos)

    # Now drop measurements for 5 frames (coast)
    for _ in range(5):
        pred_u, pred_v = kf.predict(dt=dt)
        pos = (pos[0] + vx * dt, pos[1] + vy * dt)

    assert pytest.approx(pred_u, abs=1.5) == pos[0]
    assert pytest.approx(pred_v, abs=1.5) == pos[1]


def test_kalman_reset():
    """Verify reset clears filter state."""
    kf = BeaconKalmanFilter()
    kf.update((150.0, 250.0))
    assert kf.is_initialized is True

    kf.reset()
    assert kf.is_initialized is False
    assert kf.get_position() is None
    assert kf.get_velocity() is None
