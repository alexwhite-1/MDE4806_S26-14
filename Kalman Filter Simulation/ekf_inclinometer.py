"""Extended Kalman Filter for roll, pitch and gyro biases.

State vector (5D):

    x = [ roll, pitch, bx, by, bz ]^T

We integrate roll and pitch using the gyro measurements with bias
subtraction, and treat the biases as constant. Measurement is roll and
pitch from accelerometer-derived angles.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class EKFConfig:
    q_roll: float = 1e-5
    q_pitch: float = 1e-5
    q_bias: float = 1e-8
    r_roll: float = (0.01) ** 2
    r_pitch: float = (0.01) ** 2


class InclinometerEKF:
    def __init__(self, config: EKFConfig | None = None) -> None:
        self.config = config or EKFConfig()
        # State: roll, pitch, bx, by, bz
        self.x = np.zeros(5)
        self.P = np.eye(5)
        # Logs for ALS-style tuning: store innovations and innovation covariances
        self.innovation_log: list[np.ndarray] = []
        self.S_log: list[np.ndarray] = []

    # ------------------------------------------------------------------
    # Process / measurement models
    # ------------------------------------------------------------------
    def predict(self, gyro_meas: np.ndarray, dt: float) -> None:
        """Propagate state and covariance using gyro measurement.

        gyro_meas is a 3-element array [wx_meas, wy_meas, wz_meas].
        """

        roll, pitch, bx, by, bz = self.x
        wx_m, wy_m, wz_m = gyro_meas

        # Bias-corrected body rates
        wx = wx_m - bx
        wy = wy_m - by
        wz = wz_m - bz

        # Guard against gimbal lock
        eps = 1e-6
        if np.isclose(np.abs(pitch), np.pi / 2.0, atol=eps):
            pitch = np.sign(pitch) * (np.pi / 2.0 - eps)

        sr, cr = np.sin(roll), np.cos(roll)
        tp = np.tan(pitch)
        cp = np.cos(pitch)

        roll_dot = wx + wy * sr * tp + wz * cr * tp
        pitch_dot = wy * cr - wz * sr

        # Discrete-time state prediction (Euler integration)
        roll_pred = roll + roll_dot * dt
        pitch_pred = pitch + pitch_dot * dt
        bx_pred = bx
        by_pred = by
        bz_pred = bz

        self.x[:] = [roll_pred, pitch_pred, bx_pred, by_pred, bz_pred]

        # Jacobian F = df/dx
        F = np.eye(5)

        # Partial derivatives of roll_dot and pitch_dot wrt roll, pitch, biases
        # For small dt, we approximate using current state
        # roll_dot = f1(roll, pitch, bx, by, bz)
        # pitch_dot = f2(roll, pitch, bx, by, bz)

        # Derivatives wrt roll
        droll_dot_droll = (
            wy * cr * tp
            + wz * (-sr) * tp
        )
        dpitch_dot_droll = -wy * sr - wz * cr

        # Derivatives wrt pitch
        sec_p2 = 1.0 / (cp * cp)
        droll_dot_dpitch = wy * sr * sec_p2 + wz * cr * sec_p2
        dpitch_dot_dpitch = 0.0

        # Derivatives wrt biases (bx, by, bz) via wx, wy, wz
        # wx = wx_m - bx -> d(wx)/d(bx) = -1
        # wy = wy_m - by -> d(wy)/d(by) = -1
        # wz = wz_m - bz -> d(wz)/d(bz) = -1

        droll_dot_dbx = -1.0
        dpitch_dot_dbx = 0.0

        droll_dot_dby = -sr * tp
        dpitch_dot_dby = -cr

        droll_dot_dbz = -cr * tp
        dpitch_dot_dbz = sr

        # F entries for roll and pitch rows
        F[0, 0] += droll_dot_droll * dt
        F[0, 1] += droll_dot_dpitch * dt
        F[0, 2] += droll_dot_dbx * dt
        F[0, 3] += droll_dot_dby * dt
        F[0, 4] += droll_dot_dbz * dt

        F[1, 0] += dpitch_dot_droll * dt
        F[1, 1] += dpitch_dot_dpitch * dt
        F[1, 2] += dpitch_dot_dbx * dt
        F[1, 3] += dpitch_dot_dby * dt
        F[1, 4] += dpitch_dot_dbz * dt

        # Process noise covariance Q
        Q = np.zeros((5, 5))
        Q[0, 0] = self.config.q_roll * dt
        Q[1, 1] = self.config.q_pitch * dt
        Q[2, 2] = self.config.q_bias * dt
        Q[3, 3] = self.config.q_bias * dt
        Q[4, 4] = self.config.q_bias * dt

        self.P = F @ self.P @ F.T + Q

    def update(self, z: Tuple[float, float]) -> None:
        """Update with accelerometer-based roll and pitch measurement.

        z = (roll_meas, pitch_meas)
        """

        roll_meas, pitch_meas = z

        # Measurement model: h(x) = [roll, pitch]
        H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0, 0.0],
            ]
        )

        R = np.diag([self.config.r_roll, self.config.r_pitch])

        z_vec = np.array([roll_meas, pitch_meas])
        h = H @ self.x
        y = z_vec - h

        S = H @ self.P @ H.T + R

        # Log innovation and its covariance for ALS-style Q/R estimation
        self.innovation_log.append(y.copy())
        self.S_log.append(S.copy())

        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        I = np.eye(5)
        self.P = (I - K @ H) @ self.P

    def get_state(self) -> Tuple[float, float, float, float, float]:
        roll, pitch, bx, by, bz = self.x
        return float(roll), float(pitch), float(bx), float(by), float(bz)
