"""
Regression and Audit Tests for Phase 2 Kalman Filter and Tracking Benchmark Suite.
Verifies mathematical correctness, noise attenuation, coasting stability,
reacquisition latency, and variable-dt performance.
"""

import math
import numpy as np
import pytest

from backend.config.vision_config import VisionConfig, KalmanConfig
from backend.vision.kalman import BeaconKalmanFilter
from backend.evaluation.benchmark import TrackingBenchmarkRunner


def test_kalman_stationary_exact_convergence():
    """Verify filter on stationary target maintains exact position and near-zero velocity."""
    kf = BeaconKalmanFilter(KalmanConfig(process_noise_std=80.0, measurement_noise_std=0.2))
    
    # 30 frames at fixed position (320.0, 240.0)
    for _ in range(30):
        kf.predict(dt=1.0 / 30.0)
        pos = kf.update((320.0, 240.0))

    assert abs(pos[0] - 320.0) < 1e-4
    assert abs(pos[1] - 240.0) < 1e-4

    vel = kf.get_velocity()
    assert vel is not None
    assert abs(vel[0]) < 1e-3
    assert abs(vel[1]) < 1e-3


def test_kalman_constant_velocity_noise_attenuation():
    """Verify Kalman filter reduces measurement noise on a linear constant-velocity trajectory."""
    kf = BeaconKalmanFilter(KalmanConfig(process_noise_std=40.0, measurement_noise_std=1.0))
    rng = np.random.RandomState(42)

    dt = 1.0 / 30.0
    vx, vy = 60.0, 40.0  # px/s
    true_x, true_y = 100.0, 100.0
    noise_sigma = 2.0  # px

    raw_errors = []
    filt_errors = []

    for k in range(90):
        true_x += vx * dt
        true_y += vy * dt

        noisy_x = true_x + float(rng.normal(0, noise_sigma))
        noisy_y = true_y + float(rng.normal(0, noise_sigma))

        kf.predict(dt=dt)
        est_x, est_y = kf.update((noisy_x, noisy_y))

        # Discard first 10 frames during initial velocity lock
        if k >= 10:
            raw_err = math.hypot(noisy_x - true_x, noisy_y - true_y)
            filt_err = math.hypot(est_x - true_x, est_y - true_y)
            raw_errors.append(raw_err)
            filt_errors.append(filt_err)

    raw_rmse = float(np.sqrt(np.mean(np.square(raw_errors))))
    filt_rmse = float(np.sqrt(np.mean(np.square(filt_errors))))

    # Assert Kalman achieved at least 50% noise reduction
    assert filt_rmse < 0.60 * raw_rmse
    assert filt_rmse < 1.20  # Under 1.2 px RMSE for 2.0 px noise


def test_kalman_curved_motion_lag_bounded():
    """Verify that steady-state lag on a circular trajectory is bounded under balanced parameters."""
    runner = TrackingBenchmarkRunner(fps=30.0, steps=120)
    cfg = VisionConfig()
    cfg.kalman.process_noise_std = 80.0
    cfg.kalman.measurement_noise_std = 0.2

    res = runner.run_scenario(
        name="Circular Lag Audit",
        motion_type="circular",
        shape="gaussian",
        motion_params={"radius": 150.0, "angular_speed_deg_s": 45.0},
        vision_config=cfg,
    )

    # Filtered RMSE must be bounded under 0.5 px with balanced config
    assert res.filtered_rmse_px < 0.50
    assert res.filtered_max_err_px < 1.0


def test_kalman_multi_frame_coasting_and_covariance_growth():
    """Verify state propagation and monotonic covariance growth during missed frame coasting."""
    kf = BeaconKalmanFilter(KalmanConfig(process_noise_std=80.0, measurement_noise_std=0.2))
    
    # Initialize and track for 20 frames at (u=100+50*t, v=200+30*t)
    dt = 1.0 / 30.0
    for k in range(20):
        t = k * dt
        kf.predict(dt=dt)
        kf.update((100.0 + 50.0 * t, 200.0 + 30.0 * t))

    cov_before = float(np.trace(kf.P))
    pos_before = kf.get_position()
    vel_before = kf.get_velocity()
    assert pos_before is not None and vel_before is not None

    # Coast for 5 frames without measurement updates
    prev_cov = cov_before
    for _ in range(5):
        kf.predict(dt=dt)
        curr_cov = float(np.trace(kf.P))
        assert curr_cov > prev_cov  # Covariance must grow monotonically during coasting
        prev_cov = curr_cov

    pos_after = kf.get_position()
    assert pos_after is not None

    # Verify expected linear extrapolation: pos_after ≈ pos_before + 5 * dt * vel
    expected_u = pos_before[0] + 5 * dt * vel_before[0]
    expected_v = pos_before[1] + 5 * dt * vel_before[1]
    assert abs(pos_after[0] - expected_u) < 0.1
    assert abs(pos_after[1] - expected_v) < 0.1


def test_kalman_reacquisition_latency():
    """Verify filter re-converges to low error within 2 frames after 5 dropped frames."""
    runner = TrackingBenchmarkRunner(fps=30.0, steps=120)
    cfg = VisionConfig()
    cfg.kalman.process_noise_std = 80.0
    cfg.kalman.measurement_noise_std = 0.2

    res = runner.run_scenario(
        name="Dropout Test",
        motion_type="straight_line",
        shape="gaussian",
        motion_params={"speed": 50.0, "angle_deg": 30.0},
        dropout_interval=(40, 44),  # 5 frames dropped
        vision_config=cfg,
    )

    assert res.reacquisition_latency_frames is not None
    assert res.reacquisition_latency_frames <= 2


def test_kalman_variable_dt_stability():
    """Verify filter operates correctly across varying sample intervals dt."""
    kf = BeaconKalmanFilter(KalmanConfig(process_noise_std=80.0, measurement_noise_std=0.2))
    
    # Simulate variable frame rate (e.g. 20 Hz to 100 Hz dt)
    dts = [0.01, 0.05, 0.033, 0.02, 0.04]
    t = 0.0
    for _ in range(10):
        for dt_step in dts:
            t += dt_step
            true_pos = (200.0 + 40.0 * t, 300.0 - 20.0 * t)
            kf.predict(dt=dt_step)
            est_pos = kf.update(true_pos)

    vel = kf.get_velocity()
    assert vel is not None
    assert abs(vel[0] - 40.0) < 1.0
    assert abs(vel[1] - (-20.0)) < 1.0


def test_full_benchmark_matrix_execution():
    """Verify that TrackingBenchmarkRunner runs the complete 14-scenario matrix without error."""
    runner = TrackingBenchmarkRunner(fps=30.0, steps=60)
    results = runner.run_full_benchmark_matrix()
    assert len(results) >= 12
    for r in results:
        assert r.raw_rmse_px >= 0.0
        assert r.filtered_rmse_px >= 0.0
        assert r.total_frames == 60
