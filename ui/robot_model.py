"""Mathematical model for the "Robot Motion with Obstacles" demonstration.

This module intentionally contains no Streamlit or Plotly code.  It translates
the geometry and collision logic used by the source Mathematica demonstration
into NumPy so the model can be tested independently from the user interface.

Source-model facts preserved here
----------------------------------
* Two planar revolute links.
* Each link has length 1.
* The rendered arm has half-width w = 1/8.
* Each circular obstacle has radius w = 1/8.
* A collision occurs when an obstacle center is strictly closer than 2w to
  either link centerline.
* Joint angles are periodic modulo 2*pi.
* Configuration-space quality Q is an integer from 1 through 10.
* The high-resolution configuration-space grid contains (32*Q)^2 cells.
* The Mathematica source also uses an 8*Q resolution while controls are active.

One source-specific detail deserves explicit treatment: the displayed robot
pose uses theta1 + theta2 for link 2, while the Mathematica Table used to build
the raw ArrayPlot configuration-space array uses a1 - a2.  Rather than silently
"correcting" that difference, configuration_space() exposes a
source_compatible flag and defaults it to True.  This gives us something
concrete to compare against the original demonstration during testing.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import pi
from typing import Iterable

import numpy as np


TWO_PI = 2.0 * pi

QUALITY_MIN = 1
QUALITY_MAX = 10
HIGH_RESOLUTION_PER_QUALITY = 32
ACTIVE_RESOLUTION_PER_QUALITY = 8

DEFAULT_OBSTACLES = (
    (1.1, 1.1),
    (1.1, -1.1),
)

# Coordinates used by the phase-space locator in the Mathematica graphic.
PHASE_X_MIN = 2.9
PHASE_Y_MIN = -2.0
PHASE_SIDE_LENGTH = 3.8
PHASE_X_MAX = PHASE_X_MIN + PHASE_SIDE_LENGTH
PHASE_Y_MAX = PHASE_Y_MIN + PHASE_SIDE_LENGTH


@dataclass(frozen=True)
class RobotGeometry:
    """Fixed geometric parameters for the planar two-link robot."""

    link_length: float = 1.0
    link_half_width: float = 1.0 / 8.0
    obstacle_radius: float = 1.0 / 8.0

    @property
    def collision_clearance(self) -> float:
        """Centerline distance below which the source marks a collision."""
        return self.link_half_width + self.obstacle_radius


@dataclass(frozen=True)
class RobotPose:
    """Cartesian points describing one robot configuration."""

    base: np.ndarray
    joint: np.ndarray
    end_effector: np.ndarray

    @property
    def first_link(self) -> tuple[np.ndarray, np.ndarray]:
        return self.base, self.joint

    @property
    def second_link(self) -> tuple[np.ndarray, np.ndarray]:
        return self.joint, self.end_effector


@dataclass(frozen=True)
class CollisionResult:
    """Collision information for one robot configuration."""

    in_collision: bool
    distances: np.ndarray
    collision_mask: np.ndarray


@dataclass(frozen=True)
class ConfigurationSpace:
    """Sampled configuration-space collision map."""

    theta1: np.ndarray
    theta2: np.ndarray
    collision_grid: np.ndarray
    resolution: int
    source_compatible: bool


DEFAULT_GEOMETRY = RobotGeometry()


def angular_distance(a: float, b: float) -> float:
    """Return the smallest absolute distance between two periodic angles."""
    delta = (float(a) - float(b) + pi) % TWO_PI - pi
    return abs(delta)


def inverse_kinematics(
    x: float,
    y: float,
    current_theta1: float = 0.0,
    current_theta2: float = 0.0,
    geometry: RobotGeometry = DEFAULT_GEOMETRY,
    *,
    clamp_target: bool = True,
) -> tuple[float, float]:
    """Solve planar two-link inverse kinematics for an end-effector target.

    The two-link robot has two ordinary inverse-kinematic branches
    (elbow-up and elbow-down).  This function evaluates both and returns the
    solution that is closest to the robot's current joint configuration.

    If ``clamp_target`` is True and the requested point is outside the
    reachable workspace, the target is projected radially onto the nearest
    reachable boundary.  This makes drag interaction feel natural instead of
    failing when the pointer briefly leaves the arm's reachable disk.
    """
    x = float(x)
    y = float(y)

    if not np.isfinite(x) or not np.isfinite(y):
        raise ValueError("inverse-kinematics target must contain finite values")

    l1 = float(geometry.link_length)
    l2 = float(geometry.link_length)

    radius = float(np.hypot(x, y))
    max_reach = l1 + l2
    min_reach = abs(l1 - l2)

    target_x = x
    target_y = y

    if clamp_target:
        if radius > max_reach and radius > 0.0:
            scale = max_reach / radius
            target_x *= scale
            target_y *= scale
            radius = max_reach
        elif radius < min_reach:
            if radius > 0.0:
                scale = min_reach / radius
                target_x *= scale
                target_y *= scale
            else:
                target_x = min_reach
                target_y = 0.0
            radius = min_reach
    elif radius > max_reach + 1e-12 or radius < min_reach - 1e-12:
        raise ValueError("target is outside the robot's reachable workspace")

    r2 = target_x * target_x + target_y * target_y
    cos_theta2 = (r2 - l1 * l1 - l2 * l2) / (2.0 * l1 * l2)
    cos_theta2 = float(np.clip(cos_theta2, -1.0, 1.0))

    theta2_magnitude = float(np.arccos(cos_theta2))

    candidates = []
    for theta2_candidate in (theta2_magnitude, -theta2_magnitude):
        theta1_candidate = (
            np.arctan2(target_y, target_x)
            - np.arctan2(
                l2 * np.sin(theta2_candidate),
                l1 + l2 * np.cos(theta2_candidate),
            )
        )

        t1 = float(wrap_angle(theta1_candidate))
        t2 = float(wrap_angle(theta2_candidate))

        score = (
            angular_distance(t1, current_theta1)
            + angular_distance(t2, current_theta2)
        )

        candidates.append((score, t1, t2))

    candidates.sort(key=lambda item: item[0])
    _, theta1_solution, theta2_solution = candidates[0]

    return theta1_solution, theta2_solution


def wrap_angle(angle: float | np.ndarray) -> float | np.ndarray:
    """Wrap an angle to the interval [0, 2*pi)."""
    wrapped = np.mod(angle, TWO_PI)
    if np.isscalar(angle):
        return float(wrapped)
    return wrapped


def _as_obstacle_array(
    obstacles: Iterable[Iterable[float]] = DEFAULT_OBSTACLES,
) -> np.ndarray:
    """Validate and convert obstacle centers to an (N, 2) NumPy array."""
    arr = np.asarray(tuple(obstacles), dtype=float)

    if arr.ndim != 2 or arr.shape[1] != 2:
        raise ValueError("obstacles must contain one or more (x, y) coordinate pairs")

    if arr.shape[0] == 0:
        raise ValueError("at least one obstacle is required")

    if not np.all(np.isfinite(arr)):
        raise ValueError("obstacle coordinates must be finite numbers")

    return arr


def forward_kinematics(
    theta1: float,
    theta2: float,
    geometry: RobotGeometry = DEFAULT_GEOMETRY,
) -> RobotPose:
    """Return the base, elbow joint, and end-effector coordinates.

    This matches the robot pose drawn by the Mathematica demonstration:

        joint = L [cos(theta1), sin(theta1)]

        end = joint
              + L [cos(theta1 + theta2), sin(theta1 + theta2)]
    """
    t1 = wrap_angle(theta1)
    t2 = wrap_angle(theta2)
    length = float(geometry.link_length)

    base = np.array([0.0, 0.0], dtype=float)

    joint = length * np.array(
        [np.cos(t1), np.sin(t1)],
        dtype=float,
    )

    end_effector = joint + length * np.array(
        [np.cos(t1 + t2), np.sin(t1 + t2)],
        dtype=float,
    )

    return RobotPose(
        base=base,
        joint=joint,
        end_effector=end_effector,
    )


def point_to_segment_distance(
    point: Iterable[float],
    segment_start: Iterable[float],
    segment_end: Iterable[float],
) -> float:
    """Return the shortest Euclidean distance from a point to a line segment."""
    p = np.asarray(point, dtype=float)
    a = np.asarray(segment_start, dtype=float)
    b = np.asarray(segment_end, dtype=float)

    if p.shape != (2,) or a.shape != (2,) or b.shape != (2,):
        raise ValueError("point and segment endpoints must each be 2D coordinates")

    segment = b - a
    length_squared = float(np.dot(segment, segment))

    if length_squared == 0.0:
        return float(np.linalg.norm(p - a))

    projection = float(np.dot(p - a, segment) / length_squared)
    projection = float(np.clip(projection, 0.0, 1.0))
    nearest = a + projection * segment

    return float(np.linalg.norm(p - nearest))


def point_segment_distances(
    points: Iterable[Iterable[float]],
    segment_start: Iterable[float],
    segment_end: Iterable[float],
) -> np.ndarray:
    """Return the distance from every supplied point to one line segment."""
    obstacle_array = _as_obstacle_array(points)

    return np.array(
        [
            point_to_segment_distance(point, segment_start, segment_end)
            for point in obstacle_array
        ],
        dtype=float,
    )


def collision_details(
    theta1: float,
    theta2: float,
    obstacles: Iterable[Iterable[float]] = DEFAULT_OBSTACLES,
    geometry: RobotGeometry = DEFAULT_GEOMETRY,
) -> CollisionResult:
    """Evaluate collisions between both robot links and every obstacle.

    Returns a 2 x N distance matrix:

        row 0 -> link 1 to each obstacle center
        row 1 -> link 2 to each obstacle center

    The collision comparison is intentionally strict (<), matching the source.
    """
    obstacle_array = _as_obstacle_array(obstacles)
    pose = forward_kinematics(theta1, theta2, geometry)

    first_distances = point_segment_distances(
        obstacle_array,
        pose.base,
        pose.joint,
    )
    second_distances = point_segment_distances(
        obstacle_array,
        pose.joint,
        pose.end_effector,
    )

    distances = np.vstack((first_distances, second_distances))
    collision_mask = distances < geometry.collision_clearance

    return CollisionResult(
        in_collision=bool(np.any(collision_mask)),
        distances=distances,
        collision_mask=collision_mask,
    )


def is_in_collision(
    theta1: float,
    theta2: float,
    obstacles: Iterable[Iterable[float]] = DEFAULT_OBSTACLES,
    geometry: RobotGeometry = DEFAULT_GEOMETRY,
) -> bool:
    """Return True when either arm intersects at least one obstacle."""
    return collision_details(
        theta1=theta1,
        theta2=theta2,
        obstacles=obstacles,
        geometry=geometry,
    ).in_collision


def configuration_resolution(
    quality: int,
    *,
    controls_active: bool = False,
) -> int:
    """Translate Mathematica's quality Q into a grid resolution.

    High resolution: 32 * Q
    While controls are active in the source: 8 * Q
    """
    if isinstance(quality, bool) or not isinstance(quality, (int, np.integer)):
        raise TypeError("quality must be an integer")

    quality = int(quality)
    if not QUALITY_MIN <= quality <= QUALITY_MAX:
        raise ValueError(
            f"quality must be between {QUALITY_MIN} and {QUALITY_MAX}, inclusive"
        )

    scale = (
        ACTIVE_RESOLUTION_PER_QUALITY
        if controls_active
        else HIGH_RESOLUTION_PER_QUALITY
    )
    return scale * quality


def configuration_angle_samples(resolution: int) -> np.ndarray:
    """Return the cell-center angles used by the Mathematica Table.

    The source samples

        pi/resolution,
        3*pi/resolution,
        ...,
        2*pi - pi/resolution

    which is equivalent to the centers of `resolution` equal cells on [0, 2*pi).
    """
    if isinstance(resolution, bool) or not isinstance(
        resolution, (int, np.integer)
    ):
        raise TypeError("resolution must be an integer")

    resolution = int(resolution)
    if resolution <= 0:
        raise ValueError("resolution must be positive")

    return (np.arange(resolution, dtype=float) + 0.5) * (TWO_PI / resolution)


def _distance_from_point_to_segment_grid(
    point: np.ndarray,
    start_x: np.ndarray,
    start_y: np.ndarray,
    end_x: np.ndarray,
    end_y: np.ndarray,
) -> np.ndarray:
    """Vectorized point-to-segment distance for a full configuration grid."""
    px = float(point[0])
    py = float(point[1])

    seg_x = end_x - start_x
    seg_y = end_y - start_y

    length_squared = seg_x * seg_x + seg_y * seg_y

    # Link lengths are nonzero in the source model, but keep the helper robust.
    safe_length_squared = np.where(length_squared == 0.0, 1.0, length_squared)

    projection = (
        (px - start_x) * seg_x + (py - start_y) * seg_y
    ) / safe_length_squared
    projection = np.clip(projection, 0.0, 1.0)

    nearest_x = start_x + projection * seg_x
    nearest_y = start_y + projection * seg_y

    distances = np.hypot(px - nearest_x, py - nearest_y)

    if np.any(length_squared == 0.0):
        endpoint_distance = np.hypot(px - start_x, py - start_y)
        distances = np.where(length_squared == 0.0, endpoint_distance, distances)

    return distances


def configuration_space(
    obstacles: Iterable[Iterable[float]] = DEFAULT_OBSTACLES,
    quality: int = 2,
    geometry: RobotGeometry = DEFAULT_GEOMETRY,
    *,
    controls_active: bool = False,
    source_compatible: bool = True,
) -> ConfigurationSpace:
    """Compute the sampled obstacle region in robot configuration space.

    `collision_grid[j, i]` corresponds to theta2[j] and theta1[i].

    When source_compatible=True, link 2 uses theta1 - theta2 in this grid
    calculation because that is the expression present in the Mathematica
    source's Table that feeds ArrayPlot.

    The actual robot pose function, forward_kinematics(), uses theta1 + theta2,
    exactly as the source's workspace drawing does.  Keeping these behaviors
    separate lets the eventual test suite determine whether the difference is
    intentional display compensation or a conversion-sensitive inconsistency.
    """
    obstacle_array = _as_obstacle_array(obstacles)
    resolution = configuration_resolution(
        quality,
        controls_active=controls_active,
    )
    angles = configuration_angle_samples(resolution)

    theta1, theta2 = np.meshgrid(angles, angles, indexing="xy")

    length = float(geometry.link_length)

    joint_x = length * np.cos(theta1)
    joint_y = length * np.sin(theta1)

    second_link_angle = (
        theta1 - theta2
        if source_compatible
        else theta1 + theta2
    )

    end_x = joint_x + length * np.cos(second_link_angle)
    end_y = joint_y + length * np.sin(second_link_angle)

    zeros = np.zeros_like(theta1)
    collision_grid = np.zeros(theta1.shape, dtype=bool)

    clearance = geometry.collision_clearance

    for obstacle in obstacle_array:
        distance_link_1 = _distance_from_point_to_segment_grid(
            obstacle,
            zeros,
            zeros,
            joint_x,
            joint_y,
        )

        distance_link_2 = _distance_from_point_to_segment_grid(
            obstacle,
            joint_x,
            joint_y,
            end_x,
            end_y,
        )

        collision_grid |= (
            (distance_link_1 < clearance)
            | (distance_link_2 < clearance)
        )

    return ConfigurationSpace(
        theta1=angles.copy(),
        theta2=angles.copy(),
        collision_grid=collision_grid.astype(np.uint8),
        resolution=resolution,
        source_compatible=source_compatible,
    )


def phase_locator_to_angles(
    x: float,
    y: float,
) -> tuple[float, float]:
    """Map the Mathematica phase-space locator coordinates to joint angles."""
    theta1 = ((float(x) - PHASE_X_MIN) / PHASE_SIDE_LENGTH) * TWO_PI
    theta2 = ((float(y) - PHASE_Y_MIN) / PHASE_SIDE_LENGTH) * TWO_PI
    return wrap_angle(theta1), wrap_angle(theta2)


def angles_to_phase_locator(
    theta1: float,
    theta2: float,
) -> tuple[float, float]:
    """Map joint angles to the locator coordinates used in the source graphic."""
    t1 = wrap_angle(theta1)
    t2 = wrap_angle(theta2)

    x = PHASE_X_MIN + (t1 / TWO_PI) * PHASE_SIDE_LENGTH
    y = PHASE_Y_MIN + (t2 / TWO_PI) * PHASE_SIDE_LENGTH

    return float(x), float(y)
