"""Motion profile helpers for the inclinometer simulator.

Each profile is a function that, given time t, returns:

- orientation (roll, pitch, yaw)
- linear acceleration in body frame (ax, ay, az)
- angular velocity in body frame (wx, wy, wz)
"""

from __future__ import annotations

from typing import Callable, Tuple

import numpy as np

Orientation = Tuple[float, float, float]
Accel = Tuple[float, float, float]
Omega = Tuple[float, float, float]


def static_orientation(orientation: Orientation) -> Callable[[float], tuple[Orientation, Accel, Omega]]:
    """Return a profile that is motionless with fixed orientation.

    Linear accel is zero (only gravity, which the simulator adds) and
    angular rates are zero.
    """

    def profile(t: float) -> tuple[Orientation, Accel, Omega]:  # noqa: ARG001
        return orientation, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    return profile


def sinusoidal_orientation(
    roll_amp: float = 0.1,
    roll_freq: float = 1.0,
    pitch_amp: float = 0.1,
    pitch_freq: float = 1.0,
    yaw_rate: float = 0.0,
) -> Callable[[float], tuple[Orientation, Accel, Omega]]:
    """Small oscillatory roll/pitch about zero, optional constant yaw rate.

    Orientation is defined as:

        roll(t)  = roll_amp  * sin(2*pi*roll_freq * t)
        pitch(t) = pitch_amp * sin(2*pi*pitch_freq * t)
        yaw(t)   = yaw_rate * t

    Angular velocities are computed by differentiating these.
    """

    def profile(t: float) -> tuple[Orientation, Accel, Omega]:
        w_r = 2.0 * np.pi * roll_freq
        w_p = 2.0 * np.pi * pitch_freq

        roll = roll_amp * np.sin(w_r * t)
        pitch = pitch_amp * np.sin(w_p * t)
        yaw = yaw_rate * t

        roll_dot = roll_amp * w_r * np.cos(w_r * t)
        pitch_dot = pitch_amp * w_p * np.cos(w_p * t)
        yaw_dot = yaw_rate

        # For small angles, approximate body rates as Euler angle rates
        wx = roll_dot
        wy = pitch_dot
        wz = yaw_dot

        return (roll, pitch, yaw), (0.0, 0.0, 0.0), (wx, wy, wz)

    return profile


def pure_linear_accel_world(
    accel_world: Accel,
    orientation: Orientation = (0.0, 0.0, 0.0),
) -> Callable[[float], tuple[Orientation, Accel, Omega]]:
    """Profile with fixed orientation and pure linear acceleration.

    The provided acceleration is in world frame. We convert it to body
    frame inside the simulator by keeping orientation fixed here and
    letting the user set `linear_accel_body` appropriately, so this
    helper just returns the orientation and world accel.
    """

    # We return accel in world; the notebook will convert using the
    # simulator's rotation matrices as needed. Here we keep API simple
    # and focused on orientation and high-level description.

    def profile(t: float) -> tuple[Orientation, Accel, Omega]:  # noqa: ARG001
        return orientation, accel_world, (0.0, 0.0, 0.0)

    return profile


def pure_angular_motion(
    base_rates: Omega = (0.0, 0.0, 0.5),
    oscillatory_rates: Omega = (0.0, 0.0, 0.0),
    osc_freq: float = 1.0,
) -> Callable[[float], tuple[Orientation, Accel, Omega]]:
    """Pure angular rotation with optional oscillatory component.

    Orientation is obtained by integrating the angular rates assuming
    initial orientation = 0. For convenience, we compute closed-form
    integrals for simple sinusoidal components.
    """

    bx, by, bz = base_rates
    ox, oy, oz = oscillatory_rates

    def integrated_angle(b: float, o: float, w: float, t: float) -> float:
        # angle(t) = b * t + (o / w) * (1 - cos(w t)) if w != 0
        if w == 0.0 or o == 0.0:
            return b * t
        return b * t + (o / (w)) * (1.0 - np.cos(w * t))

    def profile(t: float) -> tuple[Orientation, Accel, Omega]:
        wx = bx + ox * np.sin(2.0 * np.pi * osc_freq * t)
        wy = by + oy * np.sin(2.0 * np.pi * osc_freq * t)
        wz = bz + oz * np.sin(2.0 * np.pi * osc_freq * t)

        roll = integrated_angle(bx, ox, 2.0 * np.pi * osc_freq, t)
        pitch = integrated_angle(by, oy, 2.0 * np.pi * osc_freq, t)
        yaw = integrated_angle(bz, oz, 2.0 * np.pi * osc_freq, t)

        return (roll, pitch, yaw), (0.0, 0.0, 0.0), (wx, wy, wz)

    return profile


def mixed_linear_angular_demo() -> Callable[[float], tuple[Orientation, Accel, Omega]]:
    """A single mixed linear + angular acceleration demo profile.

    - Slow yawing motion
    - Small oscillatory roll
    - Forward/backward linear acceleration that oscillates
    """

    def profile(t: float) -> tuple[Orientation, Accel, Omega]:
        yaw_rate = 0.2  # rad/s
        yaw = yaw_rate * t
        roll = 0.05 * np.sin(2.0 * np.pi * 0.5 * t)
        pitch = 0.0

        wx = 0.05 * 2.0 * np.pi * 0.5 * np.cos(2.0 * np.pi * 0.5 * t)
        wy = 0.0
        wz = yaw_rate

        ax = 0.5 * np.sin(2.0 * np.pi * 0.5 * t)
        ay = 0.0
        az = 0.0

        return (roll, pitch, yaw), (ax, ay, az), (wx, wy, wz)

    return profile


def high_freq_vibration_profile(
    lin_accel_amp: float = 0.1,
    vib_freq: float = 10.0,
    pitch_rate: float = 0.005,
) -> Callable[[float], tuple[Orientation, Accel, Omega]]:
    """Small linear accelerations at high frequency with slow pitch drift.

    - Linear acceleration oscillates along x at high frequency ("vibration") in hertz.
    - Roll is always ~0.
    - Pitch slowly changes over time at a slow sinusoidal rate.
    - Yaw is fixed at 0.
    """

    def profile(t: float) -> tuple[Orientation, Accel, Omega]:
        roll = 0.0
        pitch = pitch_rate * np.sin(2.0 * np.pi * 0.1 * t)
        yaw = 0.0

        wx = 0.0
        wy = pitch_rate * 2.0 * np.pi * 0.1 * np.cos(2.0 * np.pi * 0.1 * t)
        wz = 0.0

        ax = lin_accel_amp * np.sin(2.0 * np.pi * vib_freq * t)
        ay = 0.0
        az = 0.0

        return (roll, pitch, yaw), (ax, ay, az), (wx, wy, wz)

    return profile
