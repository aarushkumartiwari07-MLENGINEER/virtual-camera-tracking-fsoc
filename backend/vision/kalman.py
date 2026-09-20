"""
2D Constant-Velocity Discrete Kalman Filter for image-space beacon tracking.
Maintains state estimates: [u, v, u_dot, v_dot]^T
Provides temporal smoothing, velocity estimation, and coasting across missed frames.
"""

from typing import Tuple, Optional
import numpy as np

from ..config.vision_config import KalmanConfig


class BeaconKalmanFilter:
    """
    Linear discrete Kalman Filter operating on 2D image coordinates.
    State vector: x = [u, v, u_dot, v_dot]^T (dim 4)
    Measurement: z = [u, v]^T (dim 2)
    """

    def __init__(self, config: Optional[KalmanConfig] = None) -> None:
        self.config = config or KalmanConfig()
        self.state: np.ndarray = np.zeros((4, 1), dtype=np.float64)
        self.P: np.ndarray = np.eye(4, dtype=np.float64)
        self.is_initialized: bool = False

        # Measurement matrix H (2x4)
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ], dtype=np.float64)

        # Measurement noise covariance R (2x2)
        r_var = float(self.config.measurement_noise_std ** 2)
        self.R = np.eye(2, dtype=np.float64) * r_var

        self.q_var = float(self.config.process_noise_std ** 2)

    def reset(self, initial_pos: Optional[Tuple[float, float]] = None) -> None:
        """Reset filter covariance and state."""
        self.state = np.zeros((4, 1), dtype=np.float64)
        if initial_pos is not None:
            self.state[0, 0] = float(initial_pos[0])
            self.state[1, 0] = float(initial_pos[1])
            self.is_initialized = True
        else:
            self.is_initialized = False

        pos_var = float(self.config.initial_error_std ** 2)
        vel_var = float(self.config.initial_velocity_std ** 2)
        self.P = np.diag([pos_var, pos_var, vel_var, vel_var]).astype(np.float64)

    def _get_F_and_Q(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """Compute state transition matrix F and continuous white noise Q for dt."""
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ], dtype=np.float64)

        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt

        # Continuous white noise model integrated over dt
        q00 = 0.25 * dt4 * self.q_var
        q01 = 0.5 * dt3 * self.q_var
        q11 = dt2 * self.q_var

        Q = np.array([
            [q00, 0.0, q01, 0.0],
            [0.0, q00, 0.0, q01],
            [q01, 0.0, q11, 0.0],
            [0.0, q01, 0.0, q11],
        ], dtype=np.float64)

        return F, Q

    def predict(self, dt: float = 1.0 / 30.0) -> Tuple[float, float]:
        """
        Advance filter state and covariance by timestep dt (a priori estimate).
        Returns predicted position (u_pred, v_pred).
        """
        if not self.is_initialized:
            return 0.0, 0.0

        F, Q = self._get_F_and_Q(dt)
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + Q

        return float(self.state[0, 0]), float(self.state[1, 0])

    def update(self, measurement: Tuple[float, float]) -> Tuple[float, float]:
        """
        Update state estimate with a fresh sensor measurement (a posteriori estimate).
        
        Args:
            measurement: (u_meas, v_meas) sub-pixel centroid
            
        Returns:
            (u_filtered, v_filtered)
        """
        z = np.array([[measurement[0]], [measurement[1]]], dtype=np.float64)

        if not self.is_initialized:
            # First measurement initializes state directly
            self.reset(initial_pos=measurement)
            return float(self.state[0, 0]), float(self.state[1, 0])

        # Innovation: y = z - H * x
        y = z - self.H @ self.state

        # Innovation covariance: S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R

        # Optimal Kalman gain: K = P * H^T * S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # State update: x = x + K * y
        self.state = self.state + K @ y

        # Covariance update (Joseph form for numerical stability):
        # P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
        I = np.eye(4, dtype=np.float64)
        IKH = I - K @ self.H
        self.P = IKH @ self.P @ IKH.T + K @ self.R @ K.T

        return float(self.state[0, 0]), float(self.state[1, 0])

    def get_position(self) -> Optional[Tuple[float, float]]:
        """Return current filtered position (u, v)."""
        if not self.is_initialized:
            return None
        return float(self.state[0, 0]), float(self.state[1, 0])

    def get_velocity(self) -> Optional[Tuple[float, float]]:
        """Return current estimated velocity (u_dot, v_dot) in px/s."""
        if not self.is_initialized:
            return None
        return float(self.state[2, 0]), float(self.state[3, 0])
