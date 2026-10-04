"""Plotly visualizations for the Robot Motion with Obstacles project.

This module is intentionally responsible only for presentation.  Robot
kinematics, collision tests, and configuration-space calculations live in
ui.robot_model.

The two primary figures correspond to the two graphical ideas in the source
Mathematica demonstration:

1. Workspace view:
   a two-link planar robot moving among circular obstacles.

2. Configuration-space view:
   a sampled collision map over (theta1, theta2), with the robot's current
   configuration shown as a point.

The plotting functions accept model outputs rather than storing Streamlit
session state.  That keeps the visualization layer independently testable and
lets demonstration_page.py decide how controls are organized.
"""

from __future__ import annotations

from typing import Iterable

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from ui.robot_model import (
    ConfigurationSpace,
    DEFAULT_GEOMETRY,
    DEFAULT_OBSTACLES,
    RobotGeometry,
    TWO_PI,
    collision_details,
    forward_kinematics,
    wrap_angle,
)


# ---------------------------------------------------------------------------
# Display defaults
# ---------------------------------------------------------------------------

WORKSPACE_RANGE = (-2.2, 2.2)

ROBOT_LINK_COLOR = "#59636e"
ROBOT_COLLISION_COLOR = "#d62728"
ROBOT_JOINT_COLOR = "#111827"
ROBOT_END_EFFECTOR_COLOR = "#2563eb"

OBSTACLE_FILL_COLOR = "rgba(249, 115, 22, 0.45)"
OBSTACLE_BORDER_COLOR = "#c2410c"

FREE_SPACE_COLOR = "#f8fafc"
COLLISION_SPACE_COLOR = "#334155"
CONFIGURATION_MARKER_COLOR = "#dc2626"

ANGLE_ARC_COLOR = "#64748b"


# ---------------------------------------------------------------------------
# Validation / geometry helpers
# ---------------------------------------------------------------------------

def _as_obstacle_array(
    obstacles: Iterable[Iterable[float]] = DEFAULT_OBSTACLES,
) -> np.ndarray:
    """Convert obstacle centers to an (N, 2) NumPy array."""
    arr = np.asarray(tuple(obstacles), dtype=float)

    if arr.ndim != 2 or arr.shape[1] != 2 or arr.shape[0] == 0:
        raise ValueError("obstacles must contain one or more (x, y) coordinate pairs")

    if not np.all(np.isfinite(arr)):
        raise ValueError("obstacle coordinates must be finite numbers")

    return arr


def _capsule_polygon(
    start: np.ndarray,
    end: np.ndarray,
    radius: float,
    samples_per_cap: int = 18,
) -> tuple[np.ndarray, np.ndarray]:
    """Create a 2D capsule polygon around a line segment.

    A capsule is a rectangle with semicircular ends.  It is a closer visual
    representation of a robot link with physical half-width than a Plotly line,
    whose thickness is measured in screen pixels rather than workspace units.
    """
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)

    delta = end - start
    length = float(np.linalg.norm(delta))

    if length == 0.0:
        angles = np.linspace(0.0, TWO_PI, 2 * samples_per_cap, endpoint=False)
        x = start[0] + radius * np.cos(angles)
        y = start[1] + radius * np.sin(angles)
        return x, y

    direction_angle = float(np.arctan2(delta[1], delta[0]))

    end_angles = np.linspace(
        direction_angle - np.pi / 2.0,
        direction_angle + np.pi / 2.0,
        samples_per_cap,
    )
    start_angles = np.linspace(
        direction_angle + np.pi / 2.0,
        direction_angle + 3.0 * np.pi / 2.0,
        samples_per_cap,
    )

    end_cap = np.column_stack(
        (
            end[0] + radius * np.cos(end_angles),
            end[1] + radius * np.sin(end_angles),
        )
    )
    start_cap = np.column_stack(
        (
            start[0] + radius * np.cos(start_angles),
            start[1] + radius * np.sin(start_angles),
        )
    )

    polygon = np.vstack((end_cap, start_cap))
    polygon = np.vstack((polygon, polygon[0]))

    return polygon[:, 0], polygon[:, 1]


def _arc_points(
    center: np.ndarray,
    start_angle: float,
    sweep_angle: float,
    radius: float,
    samples: int = 48,
) -> tuple[np.ndarray, np.ndarray]:
    """Return x/y points for an angle-indicator arc."""
    if abs(sweep_angle) < 1e-12:
        angles = np.array([start_angle, start_angle], dtype=float)
    else:
        angles = np.linspace(
            start_angle,
            start_angle + sweep_angle,
            samples,
        )

    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    return x, y


def _add_link_capsule(
    fig: go.Figure,
    start: np.ndarray,
    end: np.ndarray,
    half_width: float,
    *,
    color: str,
    name: str,
) -> None:
    """Draw one robot link as a filled capsule."""
    x, y = _capsule_polygon(start, end, half_width)

    fig.add_trace(
        go.Scatter(
            x=x,
            y=y,
            mode="lines",
            fill="toself",
            fillcolor=color,
            line=dict(color=color, width=1),
            hoverinfo="skip",
            name=name,
            showlegend=False,
        )
    )


def _angle_tick_text() -> tuple[list[float], list[str]]:
    """Common radian tick locations/labels for configuration-space axes."""
    values = [
        0.0,
        np.pi / 2.0,
        np.pi,
        3.0 * np.pi / 2.0,
        TWO_PI,
    ]
    labels = [
        "0",
        "π/2",
        "π",
        "3π/2",
        "2π",
    ]
    return values, labels


# ---------------------------------------------------------------------------
# Workspace visualization
# ---------------------------------------------------------------------------

def create_workspace_figure(
    theta1: float,
    theta2: float,
    obstacles: Iterable[Iterable[float]] = DEFAULT_OBSTACLES,
    geometry: RobotGeometry = DEFAULT_GEOMETRY,
    *,
    show_angle_arcs: bool = True,
    show_labels: bool = True,
    title: str = "Robot Workspace",
) -> go.Figure:
    """Create the Cartesian robot/obstacle workspace figure.

    The robot pose uses the same forward-kinematic convention as
    robot_model.forward_kinematics():

        link 1 angle = theta1
        link 2 angle = theta1 + theta2

    The robot changes color when the model reports a collision.
    """
    obstacle_array = _as_obstacle_array(obstacles)
    pose = forward_kinematics(theta1, theta2, geometry)
    collision = collision_details(theta1, theta2, obstacle_array, geometry)

    link_color = (
        ROBOT_COLLISION_COLOR
        if collision.in_collision
        else ROBOT_LINK_COLOR
    )

    fig = go.Figure()

    # Obstacles are plotted first so the robot remains visually prominent.
    for index, obstacle in enumerate(obstacle_array, start=1):
        ox, oy = float(obstacle[0]), float(obstacle[1])
        radius = float(geometry.obstacle_radius)

        fig.add_shape(
            type="circle",
            x0=ox - radius,
            x1=ox + radius,
            y0=oy - radius,
            y1=oy + radius,
            line=dict(
                color=OBSTACLE_BORDER_COLOR,
                width=2,
            ),
            fillcolor=OBSTACLE_FILL_COLOR,
            layer="below",
        )

        fig.add_trace(
            go.Scatter(
                x=[ox],
                y=[oy],
                mode="markers",
                marker=dict(
                    size=7,
                    color=OBSTACLE_BORDER_COLOR,
                ),
                name=f"Obstacle {index}",
                customdata=[[index]],
                hovertemplate=(
                    f"<b>Obstacle {index}</b><br>"
                    "x = %{x:.3f}<br>"
                    "y = %{y:.3f}<extra></extra>"
                ),
                showlegend=False,
            )
        )

        if show_labels:
            fig.add_annotation(
                x=ox,
                y=oy + radius + 0.12,
                text=f"O{index}",
                showarrow=False,
                font=dict(size=12),
            )

    _add_link_capsule(
        fig,
        pose.base,
        pose.joint,
        geometry.link_half_width,
        color=link_color,
        name="Link 1",
    )

    _add_link_capsule(
        fig,
        pose.joint,
        pose.end_effector,
        geometry.link_half_width,
        color=link_color,
        name="Link 2",
    )

    # Joint centers / end effector.
    fig.add_trace(
        go.Scatter(
            x=[
                pose.base[0],
                pose.joint[0],
                pose.end_effector[0],
            ],
            y=[
                pose.base[1],
                pose.joint[1],
                pose.end_effector[1],
            ],
            mode="markers",
            marker=dict(
                size=[13, 11, 10],
                color=[
                    ROBOT_JOINT_COLOR,
                    ROBOT_JOINT_COLOR,
                    ROBOT_END_EFFECTOR_COLOR,
                ],
                line=dict(width=1, color="white"),
            ),
            customdata=[
                ["Base"],
                ["Joint 2"],
                ["End effector"],
            ],
            hovertemplate=(
                "<b>%{customdata[0]}</b><br>"
                "x = %{x:.3f}<br>"
                "y = %{y:.3f}<extra></extra>"
            ),
            showlegend=False,
        )
    )

    # Optional joint-angle indicators.
    if show_angle_arcs:
        t1 = float(wrap_angle(theta1))

        arc1_x, arc1_y = _arc_points(
            pose.base,
            start_angle=0.0,
            sweep_angle=t1,
            radius=0.35,
        )
        fig.add_trace(
            go.Scatter(
                x=arc1_x,
                y=arc1_y,
                mode="lines",
                line=dict(
                    color=ANGLE_ARC_COLOR,
                    width=2,
                    dash="dot",
                ),
                hoverinfo="skip",
                showlegend=False,
            )
        )

        # theta2 is the relative joint angle from link 1 to link 2.
        relative_theta2 = float(wrap_angle(theta2))
        arc2_x, arc2_y = _arc_points(
            pose.joint,
            start_angle=t1,
            sweep_angle=relative_theta2,
            radius=0.28,
        )
        fig.add_trace(
            go.Scatter(
                x=arc2_x,
                y=arc2_y,
                mode="lines",
                line=dict(
                    color=ANGLE_ARC_COLOR,
                    width=2,
                    dash="dot",
                ),
                hoverinfo="skip",
                showlegend=False,
            )
        )

        if show_labels:
            fig.add_annotation(
                x=pose.base[0] + 0.48 * np.cos(t1 / 2.0),
                y=pose.base[1] + 0.48 * np.sin(t1 / 2.0),
                text="θ₁",
                showarrow=False,
            )

            theta2_mid = t1 + relative_theta2 / 2.0
            fig.add_annotation(
                x=pose.joint[0] + 0.40 * np.cos(theta2_mid),
                y=pose.joint[1] + 0.40 * np.sin(theta2_mid),
                text="θ₂",
                showarrow=False,
            )

    status_text = "COLLISION" if collision.in_collision else "Collision-free"

    fig.add_annotation(
        x=0.01,
        y=0.99,
        xref="paper",
        yref="paper",
        xanchor="left",
        yanchor="top",
        text=f"<b>{status_text}</b>",
        showarrow=False,
        bgcolor="rgba(255,255,255,0.8)",
        bordercolor=link_color,
        borderwidth=1,
        borderpad=5,
    )

    fig.update_layout(
        title=dict(
            text=title,
            x=0.5,
            xanchor="center",
        ),
        template="plotly_white",
        margin=dict(l=25, r=25, t=60, b=25),
        hovermode="closest",
        dragmode="pan",
        height=560,
        showlegend=False,
    )

    fig.update_xaxes(
        title_text="x",
        range=list(WORKSPACE_RANGE),
        scaleanchor="y",
        scaleratio=1,
        zeroline=True,
        showgrid=True,
    )

    fig.update_yaxes(
        title_text="y",
        range=list(WORKSPACE_RANGE),
        zeroline=True,
        showgrid=True,
    )

    return fig


# ---------------------------------------------------------------------------
# Configuration-space visualization
# ---------------------------------------------------------------------------

def create_configuration_space_figure(
    configuration: ConfigurationSpace,
    theta1: float,
    theta2: float,
    *,
    title: str = "Configuration Space",
    show_source_note: bool = True,
) -> go.Figure:
    """Create a collision-map figure for the sampled configuration space.

    The map uses:
        0 -> collision-free
        1 -> collision

    The current robot configuration is wrapped into [0, 2*pi) before plotting.
    """
    if configuration.collision_grid.ndim != 2:
        raise ValueError("configuration.collision_grid must be a 2D array")

    expected_shape = (
        configuration.resolution,
        configuration.resolution,
    )
    if configuration.collision_grid.shape != expected_shape:
        raise ValueError(
            "collision_grid shape does not match configuration.resolution"
        )

    current_theta1 = float(wrap_angle(theta1))
    current_theta2 = float(wrap_angle(theta2))

    colorscale = [
        [0.0, FREE_SPACE_COLOR],
        [0.499999, FREE_SPACE_COLOR],
        [0.5, COLLISION_SPACE_COLOR],
        [1.0, COLLISION_SPACE_COLOR],
    ]

    fig = go.Figure()

    fig.add_trace(
        go.Heatmap(
            x=configuration.theta1,
            y=configuration.theta2,
            z=configuration.collision_grid,
            zmin=0,
            zmax=1,
            colorscale=colorscale,
            showscale=False,
            hovertemplate=(
                "θ₁ = %{x:.3f} rad<br>"
                "θ₂ = %{y:.3f} rad<br>"
                "Collision cell = %{z}<extra></extra>"
            ),
            name="Collision map",
        )
    )

    fig.add_trace(
        go.Scatter(
            x=[current_theta1],
            y=[current_theta2],
            mode="markers",
            marker=dict(
                size=13,
                color=CONFIGURATION_MARKER_COLOR,
                symbol="x",
                line=dict(width=2),
            ),
            name="Current configuration",
            hovertemplate=(
                "<b>Current configuration</b><br>"
                "θ₁ = %{x:.3f} rad<br>"
                "θ₂ = %{y:.3f} rad<extra></extra>"
            ),
            showlegend=False,
        )
    )

    if show_source_note:
        convention = (
            "Source-compatible C-space convention"
            if configuration.source_compatible
            else "Workspace-consistent C-space convention"
        )

        fig.add_annotation(
            x=0.01,
            y=1.02,
            xref="paper",
            yref="paper",
            xanchor="left",
            yanchor="bottom",
            text=(
                f"{convention} • "
                f"{configuration.resolution}×{configuration.resolution} grid"
            ),
            showarrow=False,
            font=dict(size=11),
        )

    tick_values, tick_text = _angle_tick_text()

    fig.update_layout(
        title=dict(
            text=title,
            x=0.5,
            xanchor="center",
        ),
        template="plotly_white",
        margin=dict(l=25, r=25, t=75, b=25),
        height=560,
        hovermode="closest",
        dragmode="pan",
    )

    fig.update_xaxes(
        title_text="θ₁",
        range=[0.0, TWO_PI],
        tickmode="array",
        tickvals=tick_values,
        ticktext=tick_text,
        constrain="domain",
        showgrid=True,
        zeroline=False,
    )

    fig.update_yaxes(
        title_text="θ₂",
        range=[0.0, TWO_PI],
        tickmode="array",
        tickvals=tick_values,
        ticktext=tick_text,
        scaleanchor="x",
        scaleratio=1,
        showgrid=True,
        zeroline=False,
    )

    return fig


# ---------------------------------------------------------------------------
# Combined / convenience views
# ---------------------------------------------------------------------------

def create_side_by_side_figure(
    theta1: float,
    theta2: float,
    configuration: ConfigurationSpace,
    obstacles: Iterable[Iterable[float]] = DEFAULT_OBSTACLES,
    geometry: RobotGeometry = DEFAULT_GEOMETRY,
    *,
    title: str = "Robot Motion with Obstacles",
) -> go.Figure:
    """Create one Plotly figure containing workspace and C-space panels.

    The Streamlit page may instead render the two primary figures separately.
    This combined view is supplied as a convenience for screenshots and direct
    comparison with the original Mathematica demonstration.
    """
    obstacle_array = _as_obstacle_array(obstacles)
    pose = forward_kinematics(theta1, theta2, geometry)
    collision = collision_details(theta1, theta2, obstacle_array, geometry)

    link_color = (
        ROBOT_COLLISION_COLOR
        if collision.in_collision
        else ROBOT_LINK_COLOR
    )

    subplot = make_subplots(
        rows=1,
        cols=2,
        subplot_titles=("Robot Workspace", "Configuration Space"),
        horizontal_spacing=0.10,
    )

    # ---- Left: workspace -------------------------------------------------
    for index, obstacle in enumerate(obstacle_array, start=1):
        ox, oy = float(obstacle[0]), float(obstacle[1])
        radius = float(geometry.obstacle_radius)

        circle_angles = np.linspace(0.0, TWO_PI, 64)
        circle_x = ox + radius * np.cos(circle_angles)
        circle_y = oy + radius * np.sin(circle_angles)

        subplot.add_trace(
            go.Scatter(
                x=circle_x,
                y=circle_y,
                mode="lines",
                fill="toself",
                fillcolor=OBSTACLE_FILL_COLOR,
                line=dict(color=OBSTACLE_BORDER_COLOR, width=2),
                hoverinfo="skip",
                showlegend=False,
                name=f"Obstacle {index}",
            ),
            row=1,
            col=1,
        )

    for start, end in (
        (pose.base, pose.joint),
        (pose.joint, pose.end_effector),
    ):
        x, y = _capsule_polygon(
            start,
            end,
            geometry.link_half_width,
        )
        subplot.add_trace(
            go.Scatter(
                x=x,
                y=y,
                mode="lines",
                fill="toself",
                fillcolor=link_color,
                line=dict(color=link_color, width=1),
                hoverinfo="skip",
                showlegend=False,
            ),
            row=1,
            col=1,
        )

    subplot.add_trace(
        go.Scatter(
            x=[
                pose.base[0],
                pose.joint[0],
                pose.end_effector[0],
            ],
            y=[
                pose.base[1],
                pose.joint[1],
                pose.end_effector[1],
            ],
            mode="markers",
            marker=dict(
                size=[12, 10, 9],
                color=[
                    ROBOT_JOINT_COLOR,
                    ROBOT_JOINT_COLOR,
                    ROBOT_END_EFFECTOR_COLOR,
                ],
                line=dict(width=1, color="white"),
            ),
            hovertemplate="x=%{x:.3f}<br>y=%{y:.3f}<extra></extra>",
            showlegend=False,
        ),
        row=1,
        col=1,
    )

    # ---- Right: configuration space -------------------------------------
    cspace_colorscale = [
        [0.0, FREE_SPACE_COLOR],
        [0.499999, FREE_SPACE_COLOR],
        [0.5, COLLISION_SPACE_COLOR],
        [1.0, COLLISION_SPACE_COLOR],
    ]

    subplot.add_trace(
        go.Heatmap(
            x=configuration.theta1,
            y=configuration.theta2,
            z=configuration.collision_grid,
            zmin=0,
            zmax=1,
            colorscale=cspace_colorscale,
            showscale=False,
            hovertemplate=(
                "θ₁=%{x:.3f}<br>"
                "θ₂=%{y:.3f}<br>"
                "collision=%{z}<extra></extra>"
            ),
        ),
        row=1,
        col=2,
    )

    subplot.add_trace(
        go.Scatter(
            x=[float(wrap_angle(theta1))],
            y=[float(wrap_angle(theta2))],
            mode="markers",
            marker=dict(
                size=12,
                color=CONFIGURATION_MARKER_COLOR,
                symbol="x",
                line=dict(width=2),
            ),
            hovertemplate=(
                "θ₁=%{x:.3f}<br>"
                "θ₂=%{y:.3f}<extra></extra>"
            ),
            showlegend=False,
        ),
        row=1,
        col=2,
    )

    tick_values, tick_text = _angle_tick_text()

    subplot.update_xaxes(
        title_text="x",
        range=list(WORKSPACE_RANGE),
        scaleanchor="y",
        scaleratio=1,
        row=1,
        col=1,
    )
    subplot.update_yaxes(
        title_text="y",
        range=list(WORKSPACE_RANGE),
        row=1,
        col=1,
    )

    subplot.update_xaxes(
        title_text="θ₁",
        range=[0.0, TWO_PI],
        tickmode="array",
        tickvals=tick_values,
        ticktext=tick_text,
        row=1,
        col=2,
    )
    subplot.update_yaxes(
        title_text="θ₂",
        range=[0.0, TWO_PI],
        tickmode="array",
        tickvals=tick_values,
        ticktext=tick_text,
        scaleanchor="x2",
        scaleratio=1,
        row=1,
        col=2,
    )

    status = "COLLISION" if collision.in_collision else "Collision-free"

    subplot.update_layout(
        title=dict(
            text=f"{title} — {status}",
            x=0.5,
            xanchor="center",
        ),
        template="plotly_white",
        height=600,
        margin=dict(l=30, r=30, t=80, b=30),
        showlegend=False,
    )

    return subplot
