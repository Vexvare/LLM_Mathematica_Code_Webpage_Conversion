"""Landing page for the Robot Motion with Obstacles application."""

from __future__ import annotations

from collections.abc import Callable

import streamlit as st


NavigationCallback = Callable[[str], None]


def display_home_page(navigate: NavigationCallback | None = None) -> None:
    """Render the public landing page."""
    st.title("Robot Motion with Obstacles")
    st.caption("Interactive two-link planar robot visualization")

    st.markdown(
        """
Explore the motion of a **two-link planar robot arm** moving among circular
obstacles. The application connects the robot's physical workspace with its
two-dimensional **configuration space (C-space)** so you can see how joint
angles, obstacle placement, and collisions relate to one another.
"""
    )

    st.divider()

    demo_col, help_col = st.columns(2, gap="large")

    with demo_col:
        with st.container(border=True):
            st.subheader("🤖 Interactive Demonstration")
            st.write(
                "Move the robot, reposition obstacles, adjust configuration-space "
                "resolution, and watch the workspace and C-space update together."
            )

            if navigate is not None and st.button(
                "Open Demonstration",
                type="primary",
                use_container_width=True,
            ):
                navigate("Demonstration")

    with help_col:
        with st.container(border=True):
            st.subheader("📘 Help & Reference")
            st.write(
                "Learn how to use the controls, interpret the plots, and understand "
                "the robotics concepts behind the visualization."
            )

            if navigate is not None and st.button(
                "Open Help",
                use_container_width=True,
            ):
                navigate("Help")

    st.divider()

    st.subheader("What You Can Explore")

    workspace_col, cspace_col, interaction_col = st.columns(3, gap="medium")

    with workspace_col:
        with st.container(border=True):
            st.markdown("### Workspace")
            st.write(
                "View the robot in Cartesian space, including its base, elbow, "
                "end effector, and circular obstacles."
            )

    with cspace_col:
        with st.container(border=True):
            st.markdown("### Configuration Space")
            st.write(
                "Explore a sampled map over θ₁ and θ₂ showing collision-free "
                "and colliding robot configurations."
            )

    with interaction_col:
        with st.container(border=True):
            st.markdown("### Direct Manipulation")
            st.write(
                "Drag obstacles, the robot elbow, the end effector, and the "
                "configuration-space marker directly in the visualizations."
            )

    st.divider()

    st.subheader("Key Capabilities")

    left, right = st.columns(2, gap="large")

    with left:
        st.markdown(
            """
- Forward kinematics for a two-link planar manipulator
- Inverse kinematics for end-effector dragging
- Link-to-obstacle collision detection
- Periodic joint-angle handling
"""
        )

    with right:
        st.markdown(
            """
- Interactive Plotly workspace and C-space views
- Adjustable configuration-space sampling quality
- Precise joint and obstacle sliders
- Numerical diagnostics for robot state and geometry
"""
        )

    st.info(
        "Tip: use the draggable plot handles for exploration and the sliders "
        "when you need exact, repeatable values."
    )
