"""2D serial R/P robotics model used by the interactive DH editor.

A planar revolute row is the standard-DH special case d=alpha=0:
    RotZ(theta) @ TransX(a)
A *planar prismatic* row uses
    RotZ(theta_fixed) @ TransX(a+s)
where s is the slider's in-plane extension, NOT standard-DH d. True
standard-DH prismatic d travels along local z and would leave this XY plane.

Joint arrays are ordered from robot base to tool tip. All configuration-space
plots are honest 2-variable slices: the remaining joints stay fixed.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import radians
from typing import Sequence
import numpy as np

LINK_HALF_WIDTH = 0.125
OBSTACLE_RADIUS = 0.125
COLLISION_CLEARANCE = LINK_HALF_WIDTH + OBSTACLE_RADIUS
MAX_JOINTS = 12
MAX_PARAMETER = 10000.0  # numerical/display safety guard, not a 5-unit UI limit


@dataclass(frozen=True)
class PlanarJoint:
    kind: str = "R"             # R or P
    a: float = 1.0               # fixed link portion
    theta_deg: float = 0.0        # R variable angle; P constant heading
    extension: float = 0.0        # P variable slider displacement
    stroke: float = 3.0           # P maximum displacement

    def __post_init__(self) -> None:
        if self.kind not in ("R", "P"):
            raise ValueError("Joint type must be R or P")
        if not np.isfinite([self.a, self.theta_deg, self.extension, self.stroke]).all():
            raise ValueError("All joint values must be finite")
        if not (0.01 <= self.a <= MAX_PARAMETER):
            raise ValueError("Link length a must be 0.01–10,000 units")
        if not (-360.0 <= self.theta_deg <= 360.0):
            raise ValueError("θ must be between -360° and 360°")
        if self.kind == "P" and (not 0 < self.stroke <= MAX_PARAMETER or not 0 <= self.extension <= self.stroke):
            raise ValueError("Prismatic extension must lie between 0 and its positive stroke")

    @property
    def current_length(self) -> float:
        return self.a + (self.extension if self.kind == "P" else 0.0)

    @property
    def max_length(self) -> float:
        return self.a + (self.stroke if self.kind == "P" else 0.0)


@dataclass(frozen=True)
class ChainPose:
    points: np.ndarray  # (N+1,2) includes base, every joint endpoint and TCP
    transforms: tuple[np.ndarray, ...]

    @property
    def base(self) -> np.ndarray:
        return self.points[0]

    @property
    def joint(self) -> np.ndarray:
        return self.points[1]

    @property
    def end_effector(self) -> np.ndarray:
        return self.points[-1]


def _validate_chain(joints: Sequence[PlanarJoint]) -> tuple[PlanarJoint, ...]:
    joints = tuple(joints)
    if not 2 <= len(joints) <= MAX_JOINTS or any(not isinstance(j, PlanarJoint) for j in joints):
        raise ValueError(f"Provide 2 to {MAX_JOINTS} valid planar joints")
    return joints


def planar_transform(theta_radians: float, effective_length: float) -> np.ndarray:
    c, s = np.cos(theta_radians), np.sin(theta_radians)
    return np.array([[c, -s, 0, effective_length * c],
                     [s,  c, 0, effective_length * s],
                     [0,  0, 1, 0],
                     [0,  0, 0, 1]], dtype=float)


def forward_chain(joints: Sequence[PlanarJoint]) -> ChainPose:
    joints = _validate_chain(joints)
    transform = np.eye(4)
    transforms = []
    points = [[0.0, 0.0]]
    for joint in joints:
        transform = transform @ planar_transform(radians(joint.theta_deg), joint.current_length)
        points.append(transform[:2, 3].copy())
        transforms.append(transform.copy())
    return ChainPose(points=np.asarray(points, dtype=float), transforms=tuple(transforms))


def point_to_segment_distance(point: Sequence[float], start: np.ndarray, end: np.ndarray) -> float:
    p = np.asarray(point, dtype=float)
    v = end - start
    d2 = float(v @ v)
    if d2 < 1e-15:
        return float(np.linalg.norm(p-start))
    t = float(np.clip((p-start) @ v / d2, 0.0, 1.0))
    return float(np.linalg.norm(p - (start + t*v)))


def collision_distances(pose: ChainPose, obstacles: Sequence[Sequence[float]]) -> np.ndarray:
    obs = np.asarray(obstacles, dtype=float)
    if obs.ndim != 2 or obs.shape[1] != 2 or obs.shape[0] == 0 or not np.isfinite(obs).all():
        raise ValueError("Expected one or more finite 2D obstacles")
    return np.array([
        [point_to_segment_distance(o, pose.points[i], pose.points[i+1]) for o in obs]
        for i in range(len(pose.points)-1)
    ], dtype=float)


def configuration_slice(joints: Sequence[PlanarJoint], obstacles: Sequence[Sequence[float]],
                        axis_a: int, axis_b: int, resolution: int = 64) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return x-axis samples, y-axis samples, collision_grid[y,x].

    R samples: [0,360] degrees; P samples: [0,stroke] distance units.
    All other joints remain at their current variable settings.
    """
    joints = _validate_chain(joints)
    if axis_a == axis_b or not (0 <= axis_a < len(joints)) or not (0 <= axis_b < len(joints)):
        raise ValueError("Select two different existing joint indices")
    if isinstance(resolution, bool) or not isinstance(resolution, (int, np.integer)) or not 8 <= resolution <= 320:
        raise ValueError("Resolution must be integer 8–320")
    obs = np.asarray(obstacles, dtype=float)
    if obs.ndim != 2 or obs.shape[1] != 2 or obs.shape[0] == 0 or not np.isfinite(obs).all():
        raise ValueError("Obstacles must contain finite coordinate pairs")

    def samples(index):
        end = 360.0 if joints[index].kind == "R" else joints[index].stroke
        return (np.arange(resolution, dtype=float) + .5) * end / resolution

    xs, ys = samples(axis_a), samples(axis_b)
    blocked = np.zeros((resolution, resolution), dtype=np.uint8)
    # Vectorize the whole plane; each link is a segment in an array of poses.
    xx, yy = np.meshgrid(xs, ys, indexing="xy")
    heading = np.zeros_like(xx)
    prev_x = np.zeros_like(xx)
    prev_y = np.zeros_like(xx)
    clear2 = COLLISION_CLEARANCE**2
    for i, joint in enumerate(joints):
        theta = xx if i == axis_a and joint.kind == "R" else yy if i == axis_b and joint.kind == "R" else joint.theta_deg
        ext = xx if i == axis_a and joint.kind == "P" else yy if i == axis_b and joint.kind == "P" else joint.extension
        heading = heading + np.deg2rad(theta)
        length = joint.a + (ext if joint.kind == "P" else 0)
        curr_x = prev_x + length * np.cos(heading)
        curr_y = prev_y + length * np.sin(heading)
        dx, dy = curr_x-prev_x, curr_y-prev_y
        sq = np.maximum(dx*dx+dy*dy, 1e-15)
        for ox, oy in obs:
            t = np.clip(((ox-prev_x)*dx+(oy-prev_y)*dy)/sq, 0, 1)
            blocked |= ((ox-(prev_x+t*dx))**2+(oy-(prev_y+t*dy))**2 < clear2)
        prev_x, prev_y = curr_x, curr_y
    return xs, ys, blocked


def rows_to_joints(rows: Sequence[dict]) -> tuple[PlanarJoint, ...]:
    if not 2 <= len(rows) <= MAX_JOINTS:
        raise ValueError(f"DH editor needs 2–{MAX_JOINTS} joints")
    joints = []
    for row in rows:
        kind = str(row["Type"]).upper().strip()
        joints.append(PlanarJoint(
            kind=kind,
            a=float(row["a"]),
            theta_deg=float(row["θ (deg)"]),
            extension=float(row["s"] if kind == "P" else 0.0),
            stroke=float(row["Stroke"] if kind == "P" else 0.0),
        ))
    return tuple(joints)
