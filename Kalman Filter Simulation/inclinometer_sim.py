"""Inclinometer / IMU simulator.

This class simulates raw 3-axis accelerometer and gyroscope readings
given a body orientation (roll, pitch, yaw), a body-frame linear
acceleration, and a body-frame angular velocity. It includes:

- Configurable accelerometer and gyro white noise
- Fixed per-axis gyro bias
- Gravity vector in world frame [0, 0, -g]

Angles are in radians. Axes follow a standard aerospace convention:

- x: forward
- y: right
- z: up (opposite gravity, so gravity is [0, 0, -g])

Orientation is parameterised by roll (phi), pitch (theta), yaw (psi)
and we explicitly use rotation matrices instead of quaternions.
"""

from __future__ import annotations

import numpy as np


class InclinometerSim:
    """IMU / inclinometer simulator.

    State is held as roll, pitch, yaw and optionally integrated using
    the `step` method from gyro inputs. For many tests you'll simply
    set orientation directly and call `read_accel` / `read_gyro`.
    """

    def __init__(
        self,
        *,
        gravity: float = 9.81,
        accel_noise_std: tuple[float, float, float] = (0.01 * 9.81, 0.01 * 9.81, 0.01 * 9.81),
        gyro_noise_std: tuple[float, float, float] = (0.01, 0.01, 0.01),
        gyro_bias: tuple[float, float, float] = (0.04, 0.04, 0.04),
        orientation: tuple[float, float, float] = (0.0, 0.0, 0.0),
        linear_accel_body: tuple[float, float, float] = (0.0, 0.0, 0.0),
        angular_velocity_body: tuple[float, float, float] = (0.0, 0.0, 0.0),
        rng: np.random.Generator | None = None,
    ) -> None:
        # World-frame gravity vector [0, 0, -g] (z up, gravity down).
        # With roll=pitch=yaw=0 and R_bw = I, the accelerometer
        # measures approximately [0, 0, -g] in body frame at rest,
        # which matches the standard roll/pitch formulas used below.
        self.g = float(gravity)
        self.g_world = np.array([0.0, 0.0, -self.g])

        # Orientation (roll, pitch, yaw)
        self.roll, self.pitch, self.yaw = orientation

        # Current kinematics in body frame
        self.linear_accel_body = np.array(linear_accel_body, dtype=float)
        self.angular_velocity_body = np.array(angular_velocity_body, dtype=float)

        # Sensor characteristics
        self.accel_noise_std = np.array(accel_noise_std, dtype=float)
        self.gyro_noise_std = np.array(gyro_noise_std, dtype=float)
        self.gyro_bias = np.array(gyro_bias, dtype=float)

        # Random number generator
        self.rng = rng if rng is not None else np.random.default_rng()

    # ------------------------------------------------------------------
    # Orientation and kinematics helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Return body-from-world rotation matrix R_bw.

        We use the X-Y-Z (roll-pitch-yaw) convention:

            R_bw = R_x(roll) @ R_y(pitch) @ R_z(yaw)

        such that a vector in world coordinates v_w is transformed
        into body coordinates by v_b = R_bw @ v_w.
        """

        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        Rx = np.array(
            [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]],
            dtype=float,
        )
        Ry = np.array(
            [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
            dtype=float,
        )
        Rz = np.array(
            [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]],
            dtype=float,
        )

        return Rx @ Ry @ Rz

    def set_orientation(self, roll: float, pitch: float, yaw: float) -> None:
        self.roll = float(roll)
        self.pitch = float(pitch)
        self.yaw = float(yaw)

    def set_linear_accel_body(self, ax: float, ay: float, az: float) -> None:
        self.linear_accel_body[:] = [ax, ay, az]

    def set_angular_velocity_body(self, wx: float, wy: float, wz: float) -> None:
        self.angular_velocity_body[:] = [wx, wy, wz]

    # ------------------------------------------------------------------
    # Sensor simulation
    # ------------------------------------------------------------------
    def read_accel(self) -> np.ndarray:
        """Simulate raw accelerometer reading in body frame.

        a_meas = a_body_linear + R_bw @ g_world + noise
        """

        R_bw = self._rotation_matrix(self.roll, self.pitch, self.yaw)
        gravity_body = R_bw @ self.g_world
        accel_true = self.linear_accel_body + gravity_body
        noise = self.rng.normal(0.0, self.accel_noise_std, size=3)
        return accel_true + noise

    def read_gyro(self) -> np.ndarray:
        """Simulate raw gyroscope reading in body frame.

        omega_meas = omega_true + bias + noise
        """

        noise = self.rng.normal(0.0, self.gyro_noise_std, size=3)
        return self.angular_velocity_body + self.gyro_bias + noise

    # ------------------------------------------------------------------
    # Derived orientation from accelerometer
    # ------------------------------------------------------------------
    @staticmethod
    def roll_pitch_from_accel(accel_body: np.ndarray) -> tuple[float, float]:
        """Compute (roll, pitch) from accelerometer measurement.

        Uses the standard small-angle formulas assuming only gravity
        (or that linear acceleration is small / averaged out):

            roll  = atan2(ay, az)
            pitch = atan2(-ax, sqrt(ay^2 + az^2))
        """

        ax, ay, az = accel_body

        # With gravity approximately [0, 0, g] in body frame at rest,
        # these formulas produce roll,pitch ≈ 0 for level orientation.
        denom = np.sqrt(ay * ay + az * az)
        if denom < 1e-9:
            pitch = 0.0
        else:
            pitch = np.arctan2(-ax, denom)
        roll = -np.arctan2(-ay, -az)
        return float(roll), float(pitch)

    def read_roll_pitch_from_accel(self) -> tuple[float, float]:
        """Convenience wrapper: simulate accel then compute roll/pitch."""

        accel = self.read_accel()
        return self.roll_pitch_from_accel(accel)

    # ------------------------------------------------------------------
    # Simple integration of orientation from gyro
    # ------------------------------------------------------------------
    def step(self, dt: float) -> None:
        """Advance orientation by integrating current angular velocity.

        This uses the nonlinear mapping between body rates and Euler
        angle rates, then integrates with a simple Euler step. It's
        adequate for small dt in simulation.
        """

        wx, wy, wz = self.angular_velocity_body
        roll, pitch, yaw = self.roll, self.pitch, self.yaw

        # Guard against gimbal lock at pitch = +/- pi/2
        eps = 1e-6
        if np.isclose(np.abs(pitch), np.pi / 2.0, atol=eps):
            pitch = np.sign(pitch) * (np.pi / 2.0 - eps)

        sr, cr = np.sin(roll), np.cos(roll)
        tp = np.tan(pitch)
        cp = np.cos(pitch)

        roll_dot = wx + wy * sr * tp + wz * cr * tp
        pitch_dot = wy * cr - wz * sr
        yaw_dot = (wy * sr + wz * cr) / max(cp, 1e-6)

        self.roll += roll_dot * dt
        self.pitch += pitch_dot * dt
        self.yaw += yaw_dot * dt

    # ------------------------------------------------------------------
    # Utility
    # ------------------------------------------------------------------
    def get_orientation(self) -> tuple[float, float, float]:
        return float(self.roll), float(self.pitch), float(self.yaw)

