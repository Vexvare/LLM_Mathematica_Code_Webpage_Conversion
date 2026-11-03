"""Editable 2D filled obstacle geometry and capsule-link collision checking.

Circle: radius. Square: width is the side length. Rectangle: width/height.
Triangle: width is the base and height is the height, centered at its centroid.
Every polygon may rotate counterclockwise around its own center.
A robot link has fixed half-width (capsule). Obstacles are filled, not outlines.
"""
from __future__ import annotations
from dataclasses import dataclass
from typing import Sequence
import numpy as np

SHAPES = ("Circle", "Square", "Rectangle", "Triangle")


@dataclass(frozen=True)
class Obstacle:
    x: float = 1.1
    y: float = 1.1
    shape: str = "Circle"
    radius: float = .125
    width: float = .5
    height: float = .5
    rotation_deg: float = 0.0

    def __post_init__(self):
        if self.shape not in SHAPES:
            raise ValueError(f"Shape must be one of {SHAPES}")
        vals = (self.x, self.y, self.radius, self.width, self.height, self.rotation_deg)
        if not np.isfinite(vals).all():
            raise ValueError("Obstacle values must be finite numbers")
        if not 0.001 <= self.radius <= 10000 or not 0.001 <= self.width <= 10000 or not 0.001 <= self.height <= 10000:
            raise ValueError("Obstacle dimensions must be between 0.001 and 10000")

    @property
    def extent(self) -> float:
        if self.shape == "Circle":
            return self.radius
        if self.shape == "Square":
            return np.sqrt(2.0) * self.width / 2
        return float(np.hypot(self.width, self.height) / 2)

    def to_dict(self) -> dict:
        return {"x": float(self.x), "y": float(self.y), "shape": self.shape,
                "radius": float(self.radius), "width": float(self.width),
                "height": float(self.height), "rotation_deg": float(self.rotation_deg)}


def parse_obstacles(obstacles: Sequence) -> tuple[Obstacle, ...]:
    """Accept Obstacle, dictionary, or legacy (x,y) point-circle coordinates."""
    out = []
    for ob in obstacles:
        if isinstance(ob, Obstacle):
            out.append(ob)
        elif isinstance(ob, dict):
            out.append(Obstacle(**ob))
        else:
            x, y = ob
            out.append(Obstacle(x=float(x), y=float(y)))
    if not out:
        raise ValueError("At least one obstacle required")
    return tuple(out)


def polygon_vertices(ob: Obstacle) -> np.ndarray:
    if ob.shape == "Circle":
        raise ValueError("Circle does not have polygon vertices")
    if ob.shape == "Square":
        w = h = ob.width
    else:
        w, h = ob.width, ob.height
    if ob.shape == "Triangle":
        pts = np.array([[-w/2, -h/3], [w/2, -h/3], [0, 2*h/3]], dtype=float)
    else:
        pts = np.array([[-w/2, -h/2], [w/2, -h/2], [w/2, h/2], [-w/2, h/2]], dtype=float)
    t = np.deg2rad(ob.rotation_deg)
    c, s = np.cos(t), np.sin(t)
    return pts @ np.array([[c, s], [-s, c]]) + [ob.x, ob.y]


def point_segment_sq(px, py, ax, ay, bx, by):
    """Broadcast-compatible squared distance of a point to a finite segment."""
    vx, vy = bx-ax, by-ay
    t = np.clip(((px-ax)*vx+(py-ay)*vy)/np.maximum(vx*vx+vy*vy, 1e-15), 0, 1)
    return (px-(ax+t*vx))**2+(py-(ay+t*vy))**2


def polygon_segment_sq(ax, ay, bx, by, vertices):
    """Squared distance of finite link segment to filled convex polygon.

    Supports scalar and array-valued segment coordinates (configuration-space grid).
    Returns zero when endpoints are inside or when the segment crosses polygon.
    """
    ax,ay,bx,by=np.broadcast_arrays(np.asarray(ax,float),np.asarray(ay,float),
                                     np.asarray(bx,float),np.asarray(by,float))
    d2 = np.full(ax.shape, np.inf, dtype=float)
    inside_a = np.ones(ax.shape, dtype=bool)
    inside_b = np.ones(ax.shape, dtype=bool)
    intersects = np.zeros(ax.shape, dtype=bool)
    count = len(vertices)
    for k in range(count):
        (cx,cy),(dx,dy) = vertices[k],vertices[(k+1)%count]
        # CCW polygon; all half-plane cross products must be nonnegative.
        ea=(dx-cx)*(ay-cy)-(dy-cy)*(ax-cx)
        eb=(dx-cx)*(by-cy)-(dy-cy)*(bx-cx)
        inside_a &= ea >= -1e-12
        inside_b &= eb >= -1e-12
        s1=(bx-ax)*(cy-ay)-(by-ay)*(cx-ax)
        s2=(bx-ax)*(dy-ay)-(by-ay)*(dx-ax)
        bbox_overlap = (
            np.maximum(np.minimum(ax,bx), min(cx,dx)) <=
            np.minimum(np.maximum(ax,bx), max(cx,dx)) + 1e-12
        ) & (
            np.maximum(np.minimum(ay,by), min(cy,dy)) <=
            np.minimum(np.maximum(ay,by), max(cy,dy)) + 1e-12
        )
        intersects |= (((s1 >= 0)&(s2 <= 0))|((s1 <= 0)&(s2 >= 0))) & (
            ((ea >= 0)&(eb <= 0))|((ea <= 0)&(eb >= 0))) & bbox_overlap
        d2 = np.minimum(d2,point_segment_sq(cx,cy,ax,ay,bx,by))
        d2 = np.minimum(d2,point_segment_sq(dx,dy,ax,ay,bx,by))
        d2 = np.minimum(d2,point_segment_sq(ax,ay,cx,cy,dx,dy))
        d2 = np.minimum(d2,point_segment_sq(bx,by,cx,cy,dx,dy))
    return np.where(intersects | inside_a | inside_b, 0.0, d2)


def segment_obstacle_gap(ax, ay, bx, by, ob: Obstacle, link_half_width: float = .125):
    """Signed separation from a capsule link to a filled obstacle.

    Negative = overlapping; positive = separated. For polygon penetration,
    return -link_half_width (penetration depth is not computed).
    """
    if ob.shape == "Circle":
        d = np.sqrt(point_segment_sq(ob.x,ob.y,ax,ay,bx,by)) - ob.radius
    else:
        d = np.sqrt(polygon_segment_sq(ax,ay,bx,by,polygon_vertices(ob)))
    return d - link_half_width


def obstacle_gaps(points: np.ndarray, obstacles: Sequence, link_half_width: float = .125) -> np.ndarray:
    obs=parse_obstacles(obstacles)
    return np.array([[float(segment_obstacle_gap(*points[i],*points[i+1], ob, link_half_width))
                      for ob in obs] for i in range(len(points)-1)],dtype=float)
