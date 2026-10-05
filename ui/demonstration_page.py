"""Interactive demonstration page for Robot Motion with Obstacles."""

from __future__ import annotations

from math import degrees, radians

import streamlit as st

from ui.interactive_plot import render_interactive_robot_system
from ui.robot_model import (
    DEFAULT_OBSTACLES,
    QUALITY_MAX,
    QUALITY_MIN,
    TWO_PI,
    collision_details,
    forward_kinematics,
    phase_locator_to_angles,
    wrap_angle,
)


DEFAULT_QUALITY = 2
DEFAULT_PHASE_LOCATOR = (5.0, 0.0)

_DEFAULT_THETA1_RAD, _DEFAULT_THETA2_RAD = phase_locator_to_angles(
    *DEFAULT_PHASE_LOCATOR
)

DEFAULT_THETA1_DEG = degrees(_DEFAULT_THETA1_RAD)
DEFAULT_THETA2_DEG = degrees(_DEFAULT_THETA2_RAD)

OBSTACLE_STEP = 0.05

INTERACTION_COMPONENT_KEY = "robot_motion_interaction"
INTERACTION_NONCE_KEY = "_last_robot_motion_interaction_nonce"


def _initialize_state() -> None:
    """Initialize demonstration state once per browser session."""
    defaults = {
        "demo_theta1_deg": float(DEFAULT_THETA1_DEG),
        "demo_theta2_deg": float(DEFAULT_THETA2_DEG),
        "demo_obstacle1_x": float(DEFAULT_OBSTACLES[0][0]),
        "demo_obstacle1_y": float(DEFAULT_OBSTACLES[0][1]),
        "demo_obstacle2_x": float(DEFAULT_OBSTACLES[1][0]),
        "demo_obstacle2_y": float(DEFAULT_OBSTACLES[1][1]),
        "demo_quality": DEFAULT_QUALITY,
        "demo_show_angle_arcs": True,
        "demo_show_labels": True,
    }

    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value


def _reset_demo() -> None:
    """Restore the default robot state."""
    st.session_state.demo_theta1_deg = float(DEFAULT_THETA1_DEG)
    st.session_state.demo_theta2_deg = float(DEFAULT_THETA2_DEG)

    st.session_state.demo_obstacle1_x = float(DEFAULT_OBSTACLES[0][0])
    st.session_state.demo_obstacle1_y = float(DEFAULT_OBSTACLES[0][1])

    st.session_state.demo_obstacle2_x = float(DEFAULT_OBSTACLES[1][0])
    st.session_state.demo_obstacle2_y = float(DEFAULT_OBSTACLES[1][1])

    st.session_state.demo_quality = DEFAULT_QUALITY
    st.session_state.demo_show_angle_arcs = True
    st.session_state.demo_show_labels = True


def _component_event() -> dict | None:
    """Return the latest committed browser interaction event."""
    component_state = st.session_state.get(INTERACTION_COMPONENT_KEY)

    if component_state is None:
        return None

    event = getattr(component_state, "event", None)

    if event is None and isinstance(component_state, dict):
        event = component_state.get("event")

    return event if isinstance(event, dict) else None


def _radians_to_degrees(angle: float) -> float:
    """Convert a radian angle to the 0..360 degree slider interval."""
    value = float(angle)

    if abs(value - TWO_PI) <= 1e-9:
        return 360.0

    return degrees(float(wrap_angle(value)))


def _consume_interaction_event() -> None:
    """Synchronize a committed browser interaction with Streamlit controls."""
    event = _component_event()

    if not event:
        return

    nonce = event.get("nonce")

    if (
        nonce is None
        or nonce
        == st.session_state.get(INTERACTION_NONCE_KEY)
    ):
        return

    st.session_state[INTERACTION_NONCE_KEY] = nonce

    try:
        theta1 = float(event["theta1"])
        theta2 = float(event["theta2"])

        obstacle1 = event["obstacle1"]
        obstacle2 = event["obstacle2"]

        obstacle1_x = float(obstacle1[0])
        obstacle1_y = float(obstacle1[1])
        obstacle2_x = float(obstacle2[0])
        obstacle2_y = float(obstacle2[1])
    except (KeyError, TypeError, ValueError, IndexError):
        return

    st.session_state.demo_theta1_deg = _radians_to_degrees(theta1)
    st.session_state.demo_theta2_deg = _radians_to_degrees(theta2)

    st.session_state.demo_obstacle1_x = max(-2.0, min(2.0, obstacle1_x))
    st.session_state.demo_obstacle1_y = max(-2.0, min(2.0, obstacle1_y))
    st.session_state.demo_obstacle2_x = max(-2.0, min(2.0, obstacle2_x))
    st.session_state.demo_obstacle2_y = max(-2.0, min(2.0, obstacle2_y))


def _robot_control_card() -> tuple[float, float]:
    """Render precise joint controls and return radians."""
    with st.container(border=True):
        st.markdown("### 🤖 Robot Configuration")
        st.caption(
            "Drag the robot or C-space crosshair, or use these sliders "
            "for precise values."
        )

        theta1_deg = st.slider(
            "Joint 1 — θ₁",
            min_value=0.0,
            max_value=360.0,
            step=1.0,
            key="demo_theta1_deg",
            format="%.1f°",
        )

        theta2_deg = st.slider(
            "Joint 2 — θ₂",
            min_value=0.0,
            max_value=360.0,
            step=1.0,
            key="demo_theta2_deg",
            format="%.1f°",
        )

        st.caption(
            f"Radians: θ₁ = {radians(theta1_deg):.3f}, "
            f"θ₂ = {radians(theta2_deg):.3f}"
        )

    return radians(theta1_deg), radians(theta2_deg)


def _obstacle_control_card(
    number: int,
    *,
    x_key: str,
    y_key: str,
) -> tuple[float, float]:
    """Render precise obstacle-position controls."""
    with st.container(border=True):
        st.markdown(f"### ⭕ Obstacle {number}")
        st.caption(
            "Drag the obstacle in the workspace or use these sliders "
            "for exact positioning."
        )

        x = st.slider(
            "X position",
            min_value=-2.0,
            max_value=2.0,
            step=OBSTACLE_STEP,
            key=x_key,
            format="%.2f",
        )

        y = st.slider(
            "Y position",
            min_value=-2.0,
            max_value=2.0,
            step=OBSTACLE_STEP,
            key=y_key,
            format="%.2f",
        )

        st.caption(f"Center: ({x:.2f}, {y:.2f})")

    return float(x), float(y)


def _model_settings_card() -> tuple[int, bool, bool]:
    """Render configuration-space and display settings."""
    with st.container(border=True):
        st.markdown("### ⚙️ Model Settings")

        quality = st.slider(
            "Configuration-space quality (Q)",
            min_value=QUALITY_MIN,
            max_value=QUALITY_MAX,
            step=1,
            key="demo_quality",
        )

        final_resolution = 32 * int(quality)
        preview_resolution = 8 * int(quality)

        st.caption(
            f"Live preview: {preview_resolution} × {preview_resolution} • "
            f"Final: {final_resolution} × {final_resolution}"
        )

        toggle_col1, toggle_col2 = st.columns(2)

        with toggle_col1:
            show_angle_arcs = st.toggle(
                "Angle arcs",
                key="demo_show_angle_arcs",
            )

        with toggle_col2:
            show_labels = st.toggle(
                "Labels",
                key="demo_show_labels",
            )

    return (
        int(quality),
        bool(show_angle_arcs),
        bool(show_labels),
    )


def _render_state_and_diagnostics(
    theta1: float,
    theta2: float,
    obstacles: tuple[tuple[float, float], tuple[float, float]],
    quality: int,
) -> None:
    """Render current state and numerical diagnostics in one visible row."""
    pose = forward_kinematics(theta1, theta2)
    collision = collision_details(theta1, theta2, obstacles)

    state_col, diagnostics_col = st.columns([1.0, 1.25], gap="large")

    with state_col:
        with st.container(border=True):
            st.markdown("### Current State")

            metric_left, metric_right = st.columns(2)

            with metric_left:
                st.metric(
                    "Robot state",
                    "COLLISION" if collision.in_collision else "Collision-free",
                )
                st.metric(
                    "End effector",
                    f"({pose.end_effector[0]:.2f}, {pose.end_effector[1]:.2f})",
                )

            with metric_right:
                st.metric(
                    "Joint angles",
                    f"{degrees(theta1):.1f}° / {degrees(theta2):.1f}°",
                )
                st.metric(
                    "C-space grid",
                    f"{32 * quality} × {32 * quality}",
                )

            st.markdown("#### Robot Geometry")
            st.markdown(
                f"""
- **Base:** `({pose.base[0]:.6f}, {pose.base[1]:.6f})`
- **Elbow:** `({pose.joint[0]:.6f}, {pose.joint[1]:.6f})`
- **End effector:** `({pose.end_effector[0]:.6f}, {pose.end_effector[1]:.6f})`
- **θ₁:** `{theta1:.6f}` rad / `{degrees(theta1):.3f}°`
- **θ₂:** `{theta2:.6f}` rad / `{degrees(theta2):.3f}°`
- **Collision clearance:** `0.250000`
"""
            )

    with diagnostics_col:
        with st.container(border=True):
            st.markdown("### Numerical Diagnostics")
            st.markdown("#### Link-to-obstacle distances")

            rows = []

            for link_index in range(collision.distances.shape[0]):
                for obstacle_index in range(collision.distances.shape[1]):
                    rows.append(
                        {
                            "Link": link_index + 1,
                            "Obstacle": obstacle_index + 1,
                            "Distance": float(
                                collision.distances[
                                    link_index,
                                    obstacle_index,
                                ]
                            ),
                            "Collision": bool(
                                collision.collision_mask[
                                    link_index,
                                    obstacle_index,
                                ]
                            ),
                        }
                    )

            st.dataframe(
                rows,
                use_container_width=True,
                hide_index=True,
            )

            st.caption(
                "Distances are measured from each obstacle center to the nearest "
                "point on each robot link centerline."
            )

def display_demonstration_page() -> None:
    """Render the interactive robot demonstration."""
    _initialize_state()

    # Browser drag events are committed once, then synchronized before the
    # Streamlit widgets using the same state keys are created.
    _consume_interaction_event()

    title_col, reset_col = st.columns([8, 1])

    with title_col:
        st.title("Robot Motion with Obstacles")
        st.caption(
            "Interactive workspace and configuration-space exploration"
        )

    with reset_col:
        st.write("")
        st.write("")

        if st.button(
            "↺ Reset",
            use_container_width=True,
            help="Restore the default robot and obstacle positions.",
        ):
            _reset_demo()
            st.rerun()

    st.markdown(
        """
Move the robot and obstacles directly in the visualizations, or use the
controls for exact values. Both views share the same robot state and remain
synchronized.
"""
    )

    st.subheader("Interactive Visualization")

    # Read the current state before rendering the visualization.
    theta1 = radians(float(st.session_state.demo_theta1_deg))
    theta2 = radians(float(st.session_state.demo_theta2_deg))

    obstacle1 = (
        float(st.session_state.demo_obstacle1_x),
        float(st.session_state.demo_obstacle1_y),
    )
    obstacle2 = (
        float(st.session_state.demo_obstacle2_x),
        float(st.session_state.demo_obstacle2_y),
    )
    obstacles = (obstacle1, obstacle2)

    quality = int(st.session_state.demo_quality)
    source_compatible = False
    show_angle_arcs = bool(st.session_state.demo_show_angle_arcs)
    show_labels = bool(st.session_state.demo_show_labels)

    render_interactive_robot_system(
        theta1=theta1,
        theta2=theta2,
        obstacles=obstacles,
        quality=quality,
        show_angle_arcs=show_angle_arcs,
        show_labels=show_labels,
        key=INTERACTION_COMPONENT_KEY,
    )

    st.caption(
        "The browser updates both views continuously while you drag. "
        "Obstacle motion uses an 8Q C-space preview and commits the final "
        "32Q map when the drag ends."
    )

    st.subheader("Controls")

    robot_col, obstacle1_col, obstacle2_col, settings_col = st.columns(
        [1.35, 1.0, 1.0, 1.25],
        gap="medium",
    )

    with robot_col:
        theta1, theta2 = _robot_control_card()

    with obstacle1_col:
        obstacle1 = _obstacle_control_card(
            1,
            x_key="demo_obstacle1_x",
            y_key="demo_obstacle1_y",
        )

    with obstacle2_col:
        obstacle2 = _obstacle_control_card(
            2,
            x_key="demo_obstacle2_x",
            y_key="demo_obstacle2_y",
        )

    with settings_col:
        (
            quality,
            show_angle_arcs,
            show_labels,
        ) = _model_settings_card()

        source_compatible = False

    obstacles = (obstacle1, obstacle2)

    st.subheader("Robot State & Diagnostics")

    _render_state_and_diagnostics(
        theta1=theta1,
        theta2=theta2,
        obstacles=obstacles,
        quality=quality,
    )
