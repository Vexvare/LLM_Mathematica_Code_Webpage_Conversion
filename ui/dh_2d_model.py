"""Planar two-revolute-joint model using STANDARD Denavit-Hartenberg transforms.

This deliberately fixes d_1 = d_2 = alpha_1 = alpha_2 = 0 to keep every
joint and link in the original XY plane. Editable variables are a_1, a_2
(link lengths) and theta_1, theta_2 (joint angles). The source demonstration's
link/obstacle half-widths are retained at 1/8 units each.

All math functions are independent of Streamlit and Plotly for testing.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin, pi
from typing import Sequence

import numpy as np

TAU = 2.0 * pi
LINK_HALF_WIDTH = 0.125
OBSTACLE_RADIUS = 0.125
COLLISION_CLEARANCE = LINK_HALF_WIDTH + OBSTACLE_RADIUS


@dataclass(frozen=True)
class PlanarDH:
    a1: float = 1.0
    a2: float = 1.0

    def __post_init__(self):
        for label, length in (("a1", self.a1), ("a2", self.a2)):
            if not np.isfinite(length) or length <= 0:
                raise ValueError(f"{label} must be a finite, positive link length")


@dataclass(frozen=True)
class PlanarPose:
    base: np.ndarray
    joint: np.ndarray
    end_effector: np.ndarray
    transforms: tuple[np.ndarray, np.ndarray]


def standard_dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """A_i = RotZ(theta) TransZ(d) TransX(a) RotX(alpha)."""
    values = (theta, d, a, alpha)
    if not all(np.isfinite(v) for v in values):
        raise ValueError("DH parameters must be finite")
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0.0,    sa,     ca,    d],
        [0.0,   0.0,    0.0,  1.0],
    ], dtype=float)


def forward_kinematics(theta1: float, theta2: float,
                       geometry: PlanarDH = PlanarDH()) -> PlanarPose:
    """Two standard DH transforms for a 2R manipulator in the XY plane."""
    t1 = standard_dh_transform(float(theta1), 0.0, geometry.a1, 0.0)
    t2 = t1 @ standard_dh_transform(float(theta2), 0.0, geometry.a2, 0.0)
    return PlanarPose(
        base=np.array([0., 0.]),
        joint=t1[:2, 3].copy(),
        end_effector=t2[:2, 3].copy(),
        transforms=(t1, t2),
    )


def point_to_segment_distance(point: Sequence[float], start: np.ndarray,
                              end: np.ndarray) -> float:
    """Euclidean distance from one obstacle center to a bounded link segment."""
    p = np.asarray(point, dtype=float)
    if p.shape != (2,) or not np.isfinite(p).all():
        raise ValueError("obstacle center must have finite (x, y) values")
    v = end - start
    length_sq = float(v @ v)
    if length_sq <= 0:
        return float(np.linalg.norm(p - start))
    t = np.clip(((p - start) @ v) / length_sq, 0., 1.)
    return float(np.linalg.norm(p - (start + t*v)))


def collision_distances(pose: PlanarPose,
                        obstacles: Sequence[Sequence[float]]) -> np.ndarray:
    """Return link-by-obstacle centerline distances, shape (2, N)."""
    obs = np.asarray(obstacles, dtype=float)
    if obs.ndim != 2 or obs.shape[1] != 2 or not np.isfinite(obs).all():
        raise ValueError("obstacles must be a finite (N,2) array")
    links = ((pose.base, pose.joint), (pose.joint, pose.end_effector))
    return np.array([[point_to_segment_distance(o, x, y) for o in obs]
                     for x, y in links], dtype=float)


def _grid_point_segment_distance(px: float, py: float,
                                x0: np.ndarray, y0: np.ndarray,
                                x1: np.ndarray, y1: np.ndarray) -> np.ndarray:
    dx, dy = x1-x0, y1-y0
    sq = dx*dx + dy*dy
    t = np.clip(((px-x0)*dx + (py-y0)*dy) / np.maximum(sq, 1e-15), 0., 1.)
    return np.hypot(px - (x0+t*dx), py - (y0+t*dy))


def configuration_space(geometry: PlanarDH, obstacles: Sequence[Sequence[float]],
                        resolution: int = 64) -> tuple[np.ndarray, np.ndarray]:
    """Compute [theta2 row, theta1 column] collision grid (0=free, 1=blocked).

    Samples are angular CELL CENTERS. Both links use theta1+theta2, which is
    the same convention used in the physical workspace drawing.
    """
    if not isinstance(resolution, (int, np.integer)) or isinstance(resolution, (bool, np.bool_)) or not 8 <= resolution <= 640:
        raise ValueError("resolution must be an integer in [8, 640]")
    obs = np.asarray(obstacles, dtype=float)
    if obs.ndim != 2 or obs.shape[1] != 2 or not np.isfinite(obs).all():
        raise ValueError("obstacles must be a finite (N,2) array")
    angles = (np.arange(resolution, dtype=float) + 0.5) * TAU / resolution
    th1, th2 = np.meshgrid(angles, angles, indexing="xy")
    jx = geometry.a1 * np.cos(th1)
    jy = geometry.a1 * np.sin(th1)
    ex = jx + geometry.a2 * np.cos(th1+th2)
    ey = jy + geometry.a2 * np.sin(th1+th2)
    zero = np.zeros_like(jx)
    blocked = np.zeros_like(jx, dtype=bool)
    for px, py in obs:
        blocked |= (_grid_point_segment_distance(px, py, zero, zero, jx, jy) < COLLISION_CLEARANCE)
        blocked |= (_grid_point_segment_distance(px, py, jx, jy, ex, ey) < COLLISION_CLEARANCE)
    return np.degrees(angles), blocked.astype(np.uint8)
