"""Interactive Streamlit page for Robot Motion with Obstacles.

This version supports direct manipulation inside the Plotly visualizations:

Workspace
---------
* drag obstacle 1
* drag obstacle 2
* drag the elbow to change theta1
* drag the end effector to solve inverse kinematics

Configuration space
-------------------
* drag the current-configuration marker to change theta1/theta2

The sliders remain available as precise, accessible fallback controls.
"""

from __future__ import annotations

from math import atan2, degrees, radians

import streamlit as st

from ui.interactive_plot import (
    render_draggable_configuration_space,
    render_draggable_workspace,
)
from ui.robot_model import (
    DEFAULT_OBSTACLES,
    QUALITY_MAX,
    QUALITY_MIN,
    TWO_PI,
    angles_to_phase_locator,
    collision_details,
    configuration_space,
    forward_kinematics,
    inverse_kinematics,
    phase_locator_to_angles,
    wrap_angle,
)
from ui.visualization import (
    create_configuration_space_figure,
    create_workspace_figure,
)


DEFAULT_QUALITY = 2
DEFAULT_PHASE_LOCATOR = (5.0, 0.0)
_DEFAULT_THETA1_RAD, _DEFAULT_THETA2_RAD = phase_locator_to_angles(
    *DEFAULT_PHASE_LOCATOR
)
DEFAULT_THETA1_DEG = degrees(_DEFAULT_THETA1_RAD)
DEFAULT_THETA2_DEG = degrees(_DEFAULT_THETA2_RAD)

OBSTACLE_STEP = 0.05

WORKSPACE_COMPONENT_KEY = "workspace_drag_component"
CSPACE_COMPONENT_KEY = "cspace_drag_component"

WORKSPACE_EVENT_NONCE_KEY = "_last_workspace_drag_nonce"
CSPACE_EVENT_NONCE_KEY = "_last_cspace_drag_nonce"


@st.cache_data(show_spinner=False)
def _cached_configuration_space(
    obstacle_1: tuple[float, float],
    obstacle_2: tuple[float, float],
    quality: int,
    source_compatible: bool,
):
    """Cache the expensive C-space map independently from the current pose."""
    return configuration_space(
        obstacles=(obstacle_1, obstacle_2),
        quality=quality,
        source_compatible=source_compatible,
    )


def _initialize_state() -> None:
    """Seed widget state once per browser session."""
    defaults = {
        "demo_theta1_deg": float(DEFAULT_THETA1_DEG),
        "demo_theta2_deg": float(DEFAULT_THETA2_DEG),
        "demo_obstacle1_x": float(DEFAULT_OBSTACLES[0][0]),
        "demo_obstacle1_y": float(DEFAULT_OBSTACLES[0][1]),
        "demo_obstacle2_x": float(DEFAULT_OBSTACLES[1][0]),
        "demo_obstacle2_y": float(DEFAULT_OBSTACLES[1][1]),
        "demo_quality": DEFAULT_QUALITY,
        "demo_source_compatible": True,
        "demo_show_angle_arcs": True,
        "demo_show_labels": True,
    }

    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value


def _reset_demo() -> None:
    """Restore the source demonstration's default state."""
    st.session_state.demo_theta1_deg = float(DEFAULT_THETA1_DEG)
    st.session_state.demo_theta2_deg = float(DEFAULT_THETA2_DEG)

    st.session_state.demo_obstacle1_x = float(DEFAULT_OBSTACLES[0][0])
    st.session_state.demo_obstacle1_y = float(DEFAULT_OBSTACLES[0][1])

    st.session_state.demo_obstacle2_x = float(DEFAULT_OBSTACLES[1][0])
    st.session_state.demo_obstacle2_y = float(DEFAULT_OBSTACLES[1][1])

    st.session_state.demo_quality = DEFAULT_QUALITY
    st.session_state.demo_source_compatible = True
    st.session_state.demo_show_angle_arcs = True
    st.session_state.demo_show_labels = True


def _component_event(component_key: str) -> dict | None:
    """Read the latest custom-component event from Session State."""
    component_state = st.session_state.get(component_key)

    if component_state is None:
        return None

    event = getattr(component_state, "event", None)

    if event is None and isinstance(component_state, dict):
        event = component_state.get("event")

    if not isinstance(event, dict):
        return None

    return event


def _clip_workspace_coordinate(value: float) -> float:
    """Keep obstacle centers within the source workspace control range."""
    return max(-2.0, min(2.0, float(value)))


def _radians_to_slider_degrees(angle: float) -> float:
    """Convert a periodic angle to the 0..360 degree slider interval."""
    wrapped = float(wrap_angle(angle))
    value = degrees(wrapped)

    # Numerical roundoff near a complete revolution can produce values just
    # below/above the slider bounds.
    return max(0.0, min(360.0, value))


def _consume_workspace_drag_event() -> None:
    """Apply one unprocessed workspace drag event to widget state."""
    event = _component_event(WORKSPACE_COMPONENT_KEY)
    if not event:
        return

    nonce = event.get("nonce")
    if nonce is None or nonce == st.session_state.get(WORKSPACE_EVENT_NONCE_KEY):
        return

    # Mark it as consumed before processing so a malformed event cannot loop.
    st.session_state[WORKSPACE_EVENT_NONCE_KEY] = nonce

    kind = event.get("kind")

    try:
        x = float(event.get("x"))
        y = float(event.get("y"))
    except (TypeError, ValueError):
        return

    if kind == "obstacle1":
        st.session_state.demo_obstacle1_x = _clip_workspace_coordinate(x)
        st.session_state.demo_obstacle1_y = _clip_workspace_coordinate(y)
        return

    if kind == "obstacle2":
        st.session_state.demo_obstacle2_x = _clip_workspace_coordinate(x)
        st.session_state.demo_obstacle2_y = _clip_workspace_coordinate(y)
        return

    if kind == "elbow":
        # The elbow is constrained to the unit circle geometrically.  Only its
        # polar angle matters, so pointer radius is intentionally ignored.
        theta1 = float(wrap_angle(atan2(y, x)))
        st.session_state.demo_theta1_deg = _radians_to_slider_degrees(theta1)
        return

    if kind == "end_effector":
        current_theta1 = radians(float(st.session_state.demo_theta1_deg))
        current_theta2 = radians(float(st.session_state.demo_theta2_deg))

        theta1, theta2 = inverse_kinematics(
            x=x,
            y=y,
            current_theta1=current_theta1,
            current_theta2=current_theta2,
            clamp_target=True,
        )

        st.session_state.demo_theta1_deg = _radians_to_slider_degrees(theta1)
        st.session_state.demo_theta2_deg = _radians_to_slider_degrees(theta2)


def _consume_cspace_drag_event() -> None:
    """Apply one unprocessed C-space marker drag to the joint sliders."""
    event = _component_event(CSPACE_COMPONENT_KEY)
    if not event:
        return

    nonce = event.get("nonce")
    if nonce is None or nonce == st.session_state.get(CSPACE_EVENT_NONCE_KEY):
        return

    st.session_state[CSPACE_EVENT_NONCE_KEY] = nonce

    if event.get("kind") != "configuration":
        return

    try:
        theta1 = max(0.0, min(TWO_PI, float(event.get("x"))))
        theta2 = max(0.0, min(TWO_PI, float(event.get("y"))))
    except (TypeError, ValueError):
        return

    # Keep 2*pi representable as 360 degrees instead of wrapping it to zero.
    st.session_state.demo_theta1_deg = max(
        0.0,
        min(360.0, degrees(theta1)),
    )
    st.session_state.demo_theta2_deg = max(
        0.0,
        min(360.0, degrees(theta2)),
    )


def _consume_drag_events() -> None:
    """Synchronize visual drag events before Streamlit widgets are created."""
    _consume_workspace_drag_event()
    _consume_cspace_drag_event()


def _robot_control_card() -> tuple[float, float]:
    """Render precise degree-based joint controls and return radians."""
    with st.container(border=True):
        st.markdown("### 🤖 Robot Configuration")
        st.caption(
            "Drag the blue/purple robot handles in the workspace, drag the "
            "red point in C-space, or use these sliders for precise values."
        )

        theta1_deg = st.slider(
            "Joint 1 — θ₁",
            min_value=0.0,
            max_value=360.0,
            step=1.0,
            key="demo_theta1_deg",
            format="%.0f°",
        )

        theta2_deg = st.slider(
            "Joint 2 — θ₂",
            min_value=0.0,
            max_value=360.0,
            step=1.0,
            key="demo_theta2_deg",
            format="%.0f°",
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
    """Render precise Cartesian sliders for one draggable obstacle."""
    with st.container(border=True):
        st.markdown(f"### ⭕ Obstacle {number}")
        st.caption(
            "Drag the orange circle in the workspace or use these sliders "
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


def _model_settings_card() -> tuple[int, bool, bool, bool]:
    """Render quality and display controls in a compact settings card."""
    with st.container(border=True):
        st.markdown("### ⚙️ Model Settings")
        st.caption("Control C-space resolution and display details.")

        quality = st.slider(
            "Configuration-space quality (Q)",
            min_value=QUALITY_MIN,
            max_value=QUALITY_MAX,
            step=1,
            key="demo_quality",
        )

        resolution = 32 * int(quality)
        st.caption(
            f"{resolution} × {resolution} grid "
            f"({resolution * resolution:,} sampled configurations)"
        )

        source_compatible = st.toggle(
            "Source-compatible C-space",
            key="demo_source_compatible",
            help=(
                "Preserves the sign convention used by the Mathematica "
                "configuration-space raster."
            ),
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
        bool(source_compatible),
        bool(show_angle_arcs),
        bool(show_labels),
    )


def _render_summary_metrics(
    theta1: float,
    theta2: float,
    obstacles: tuple[tuple[float, float], tuple[float, float]],
    quality: int,
) -> None:
    """Render the current state as a compact row of metrics."""
    pose = forward_kinematics(theta1, theta2)
    collision = collision_details(theta1, theta2, obstacles)
    phase_x, phase_y = angles_to_phase_locator(theta1, theta2)

    state_col, angle_col, end_col, grid_col, locator_col = st.columns(5)

    with state_col:
        st.metric(
            "Robot state",
            "COLLISION" if collision.in_collision else "Collision-free",
        )

    with angle_col:
        st.metric(
            "Joint angles",
            f"{degrees(theta1):.0f}° / {degrees(theta2):.0f}°",
        )

    with end_col:
        st.metric(
            "End effector",
            f"({pose.end_effector[0]:.2f}, {pose.end_effector[1]:.2f})",
        )

    with grid_col:
        st.metric(
            "C-space grid",
            f"{32 * quality} × {32 * quality}",
        )

    with locator_col:
        st.metric(
            "Source locator",
            f"({phase_x:.2f}, {phase_y:.2f})",
        )


def _render_diagnostics(
    theta1: float,
    theta2: float,
    obstacles: tuple[tuple[float, float], tuple[float, float]],
) -> None:
    """Render detailed numerical values for comparison/testing."""
    pose = forward_kinematics(theta1, theta2)
    collision = collision_details(theta1, theta2, obstacles)
    
    with st.expander("Numerical diagnostics", expanded=False):
        left, right = st.columns([1, 1.3])

        with left:
            st.markdown("#### Robot geometry")
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

        with right:
            st.markdown("#### Link-to-obstacle distances")

            distance_rows = []

            for link_index in range(collision.distances.shape[0]):
                for obstacle_index in range(collision.distances.shape[1]):
                    distance_rows.append(
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
                distance_rows,
                use_container_width=True,
                hide_index=True,
            )


def display_demonstration_page() -> None:
    """Render the directly manipulable robotics conversion."""
    _initialize_state()

    # Custom-component values are stored in Session State after a drag-triggered
    # rerun. Consume them before creating sliders that use the same state keys.
    _consume_drag_events()

    title_col, reset_col = st.columns([8, 1])

    with title_col:
        st.title("Robot Motion with Obstacles")
        st.caption(
            "Interactive Python / Streamlit conversion of the Mathematica demonstration"
        )

    with reset_col:
        st.write("")
        st.write("")

        if st.button(
            "↺ Reset",
            use_container_width=True,
            help="Restore the original demonstration defaults.",
        ):
            _reset_demo()
            st.rerun()

    st.markdown(
        """
Directly manipulate the robot and obstacles in the plots, or use the controls
for exact values. The workspace and **configuration space (C-space)** remain
synchronized: changing either view updates the same robot state.
"""
    )

    # ------------------------------------------------------------------
    # Controls
    # ------------------------------------------------------------------
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
            source_compatible,
            show_angle_arcs,
            show_labels,
        ) = _model_settings_card()

    obstacles = (obstacle1, obstacle2)

    # ------------------------------------------------------------------
    # Model calculation
    # ------------------------------------------------------------------
    with st.spinner("Computing configuration space..."):
        configuration = _cached_configuration_space(
            obstacle_1=obstacle1,
            obstacle_2=obstacle2,
            quality=quality,
            source_compatible=source_compatible,
        )

    pose = forward_kinematics(theta1, theta2)

    workspace_figure = create_workspace_figure(
        theta1=theta1,
        theta2=theta2,
        obstacles=obstacles,
        show_angle_arcs=show_angle_arcs,
        show_labels=show_labels,
    )

    configuration_figure = create_configuration_space_figure(
        configuration=configuration,
        theta1=theta1,
        theta2=theta2,
        show_source_note=True,
    )

    # ------------------------------------------------------------------
    # Directly manipulable visualizations
    # ------------------------------------------------------------------
    st.subheader("Interactive Visualization")

    workspace_col, cspace_col = st.columns(2, gap="large")

    with workspace_col:
        with st.container(border=True):
            st.markdown("#### Robot Workspace")
            st.caption(
                "Drag obstacles or robot handles directly in Cartesian space."
            )

            render_draggable_workspace(
                figure=workspace_figure,
                pose=pose,
                obstacles=obstacles,
                key=WORKSPACE_COMPONENT_KEY,
            )

    with cspace_col:
        with st.container(border=True):
            st.markdown("#### Configuration Space")
            st.caption(
                "Drag the red handle to set θ₁ and θ₂ directly."
            )

            render_draggable_configuration_space(
                figure=configuration_figure,
                theta1=theta1,
                theta2=theta2,
                key=CSPACE_COMPONENT_KEY,
            )
    c1, c2 = st.columns([1,1])
    with c1:
        _render_diagnostics(
            theta1=theta1,
            theta2=theta2,
            obstacles=obstacles,
        )
    with c2:
        with st.expander("Conversion notes for testing", expanded=False):
            st.markdown(
                """
                - **Obstacle interaction:** the orange obstacle circles are now directly
                draggable in the workspace. The X/Y sliders remain available for exact
                values.
                - **C-space interaction:** the red configuration handle is directly draggable
                and updates θ₁/θ₂.
                - **Robot interaction:** dragging the blue elbow ring changes θ₁. Dragging the
                purple end-effector ring runs planar two-link inverse kinematics and selects
                the elbow branch closest to the current pose.
                - **Unreachable end-effector targets:** drag targets outside the reachable
                radius are projected to the nearest reachable boundary.
                - **Periodicity:** the robot model treats both joint angles modulo `2π`.
                - **Quality Q:** the source high-resolution rule of `32Q` samples per axis is
                preserved.
                - **Dynamic low-resolution behavior:** Mathematica temporarily uses `8Q`
                samples while a locator is actively moving. This build commits a drag on
                mouse release, then computes the final `32Q` grid.
                - **C-space sign convention:** the source raster uses `a₁ - a₂` for the second
                link even though the workspace drawing uses `θ₁ + θ₂`. The source-compatible
                toggle preserves that behavior for comparison.
                """
                )
