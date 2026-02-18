"""Animation helpers for visualizing inclinometer motion.

Provides a function `animate_simulation` that takes the result dict
from `run_simulation` in `inclinometer_experiments.ipynb` and creates
an animated 3D cube along with live-updating plots of accelerometer,
gyroscope, and roll/pitch (true vs accel vs EKF).
"""

from __future__ import annotations

from typing import Dict, Any

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation


def _cube_vertices(size: float = 0.5) -> np.ndarray:
    """Return 8x3 array of cube vertices centered at origin."""

    s = size / 2.0
    # (x, y, z) corners
    return np.array(
        [
            [-s, -s, -s],
            [+s, -s, -s],
            [+s, +s, -s],
            [-s, +s, -s],
            [-s, -s, +s],
            [+s, -s, +s],
            [+s, +s, +s],
            [-s, +s, +s],
        ],
        dtype=float,
    )


def _cube_edges() -> list[tuple[int, int]]:
    """Return list of index pairs for cube edges."""

    return [
        (0, 1), (1, 2), (2, 3), (3, 0),  # bottom square
        (4, 5), (5, 6), (6, 7), (7, 4),  # top square
        (0, 4), (1, 5), (2, 6), (3, 7),  # verticals
    ]


def _rotation_matrix_xyz(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rotation matrix matching `InclinometerSim` (R_x * R_y * R_z).

    This maps from body to world if used as R = R_x * R_y * R_z and
    applied to body vectors. For visualization we want world->body or
    body->world consistently; we choose to work in body frame and
    then rotate cube vertices by the same R that maps world to body
    as used in the simulator's accelerometer model.
    """

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])

    return Rx @ Ry @ Rz


def animate_simulation(
    result: Dict[str, Any],
    title: str = "",
    speed: float = 1.0,
) -> animation.FuncAnimation:
    """Create an animation of a cube plus sensor/angle plots.

    Parameters
    ----------
    result : dict
        Dictionary returned by `run_simulation` containing keys like
        "t", "roll_true", "pitch_true", "roll_meas_acc",
        "pitch_meas_acc", "roll_est", "pitch_est".
    title : str
        Figure title.

    Returns
    -------
    matplotlib.animation.FuncAnimation
    """

    t = result["t"]
    roll_true = result["roll_true"]
    pitch_true = result["pitch_true"]
    yaw_true = result.get("yaw_true", np.zeros_like(roll_true))

    roll_meas_acc = result["roll_meas_acc"]
    pitch_meas_acc = result["pitch_meas_acc"]

    roll_est = result["roll_est"]
    pitch_est = result["pitch_est"]

    # Optional sensor components (if provided by run_simulation)
    accel = result.get("accel", None)  # shape (N,3)
    gyro = result.get("gyro", None)    # shape (N,3)

    rad2deg = np.rad2deg

    # Prepare figure: 3D cube + angle + accel + gyro subplots
    fig = plt.figure(figsize=(12, 10))
    gs = fig.add_gridspec(3, 2)

    ax3d = fig.add_subplot(gs[:, 0], projection="3d")
    ax_roll = fig.add_subplot(gs[0, 1])
    ax_pitch = fig.add_subplot(gs[1, 1])
    ax_accel = fig.add_subplot(gs[2, 1])

    # Cube vertices and edges
    verts0 = _cube_vertices(size=1.0)
    edges = _cube_edges()

    # Initialize 3D cube lines
    cube_lines = []
    for (i, j) in edges:
        (line,) = ax3d.plot(
            [verts0[i, 0], verts0[j, 0]],
            [verts0[i, 1], verts0[j, 1]],
            [verts0[i, 2], verts0[j, 2]],
            color="C0",
        )
        cube_lines.append(line)

    # 3D axis settings
    ax3d.set_xlabel("X")
    ax3d.set_ylabel("Y")
    ax3d.set_zlabel("Z")
    ax3d.set_title("Body orientation (cube)")
    ax3d.set_box_aspect([1, 1, 1])

    lim = 0.8
    ax3d.set_xlim(-lim, lim)
    ax3d.set_ylim(-lim, lim)
    ax3d.set_zlim(-lim, lim)

    # Angle plots (deg)
    ax_roll.plot(t, rad2deg(roll_true), label="roll true", color="C0")
    ax_roll.plot(t, rad2deg(roll_meas_acc), "--", label="roll accel", color="C1")
    ax_roll.plot(t, rad2deg(roll_est), ":", label="roll EKF", color="C2")
    ax_roll.set_ylabel("Roll [deg]")
    ax_roll.legend(loc="upper right")

    ax_pitch.plot(t, rad2deg(pitch_true), label="pitch true", color="C0")
    ax_pitch.plot(t, rad2deg(pitch_meas_acc), "--", label="pitch accel", color="C1")
    ax_pitch.plot(t, rad2deg(pitch_est), ":", label="pitch EKF", color="C2")
    ax_pitch.set_ylabel("Pitch [deg]")
    ax_pitch.legend(loc="upper right")

    # Accel / gyro components (if available)
    if accel is not None:
        ax_accel.plot(t, accel[:, 0], label="ax", color="C0")
        ax_accel.plot(t, accel[:, 1], label="ay", color="C1")
        ax_accel.plot(t, accel[:, 2], label="az", color="C2")
        ax_accel.set_ylabel("Accel [m/s^2]")
        ax_accel.legend(loc="upper right")

    if gyro is not None:
        ax_accel2 = ax_accel.twinx()
        ax_accel2.plot(t, gyro[:, 0], "--", label="wx", color="C3")
        ax_accel2.plot(t, gyro[:, 1], "--", label="wy", color="C4")
        ax_accel2.plot(t, gyro[:, 2], "--", label="wz", color="C5")
        ax_accel2.set_ylabel("Gyro [rad/s]")
        # Create a combined legend
        lines1, labels1 = ax_accel.get_legend_handles_labels()
        lines2, labels2 = ax_accel2.get_legend_handles_labels()
        ax_accel2.legend(lines1 + lines2, labels1 + labels2, loc="lower right")
    else:
        ax_accel.set_ylabel("Accel / Gyro")

    ax_accel.set_xlabel("Time [s]")

    # Time cursor lines on plots
    (roll_cursor,) = ax_roll.plot([t[0], t[0]], ax_roll.get_ylim(), "k-", alpha=0.3)
    (pitch_cursor,) = ax_pitch.plot([t[0], t[0]], ax_pitch.get_ylim(), "k-", alpha=0.3)
    (accel_cursor,) = ax_accel.plot([t[0], t[0]], ax_accel.get_ylim(), "k-", alpha=0.3)

    fig.suptitle(title)

    # Optional accel vector arrow on cube (body-frame accel direction)
    accel_arrow = None
    if accel is not None:
        # One arrow from origin; create once, update in-place
        accel_arrow = ax3d.quiver(
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            color="C3",
            length=0.0,
            normalize=False,
        )

    def update(frame: int) -> list:
        # Current orientation
        roll = roll_true[frame]
        pitch = pitch_true[frame]
        yaw = yaw_true[frame]

        # Rotation matrix consistent with simulator
        R = _rotation_matrix_xyz(roll, pitch, yaw)

        # Rotate cube vertices
        verts = (R @ verts0.T).T

        # Update cube edges
        for line, (i, j) in zip(cube_lines, edges):
            line.set_data([verts[i, 0], verts[j, 0]], [verts[i, 1], verts[j, 1]])
            line.set_3d_properties([verts[i, 2], verts[j, 2]])

        # Update accel arrow in body frame (if available)
        artists = []
        if accel is not None and accel_arrow is not None:
            ax3d.collections.remove(accel_arrow)
            vec = accel[frame]
            # Scale for display
            scale = 0.05
            accel_arrow = ax3d.quiver(
                0.0,
                0.0,
                0.0,
                vec[0] * scale,
                vec[1] * scale,
                vec[2] * scale,
                color="C3",
            )
            artists.append(accel_arrow)

        # Update time cursors
        x = t[frame]
        roll_cursor.set_xdata([x, x])
        pitch_cursor.set_xdata([x, x])
        accel_cursor.set_xdata([x, x])

        return cube_lines + [roll_cursor, pitch_cursor, accel_cursor] + artists

    anim = animation.FuncAnimation(
        fig,
        update,
        frames=len(t),
        interval=max(1, int(1000 * (t[1] - t[0]) / max(speed, 1e-3))),
        blit=False,
    )

    return anim
