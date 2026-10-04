"""Landing page for the Robot Motion with Obstacles project."""

from __future__ import annotations

from collections.abc import Callable

import streamlit as st


NavigationCallback = Callable[[str], None]


def display_home_page(navigate: NavigationCallback | None = None) -> None:
    """Render the application's landing page."""
    st.title("Robot Motion with Obstacles")
    st.caption("Mathematica → Python / Streamlit")

    st.markdown(
        """
This application is an AI-assisted conversion of the Wolfram Demonstrations
Project example **Robot Motion with Obstacles** into an interactive Python /
Streamlit application.

The converted demonstration models a two-link planar robot arm moving around
two circular obstacles. It displays both the robot's physical workspace and
the corresponding two-dimensional configuration space, where collision and
collision-free joint configurations can be explored interactively.
"""
    )

    st.divider()

    demo_col, help_col = st.columns(2, gap="large")

    with demo_col:
        with st.container(border=True):
            st.subheader("🤖 Interactive Demonstration")
            st.write(
                "Adjust both robot joints, move the obstacle centers, change "
                "configuration-space quality, and compare the workspace with "
                "the generated collision map."
            )

            if navigate is not None and st.button(
                "Open Demonstration",
                type="primary",
                use_container_width=True,
            ):
                navigate("Demonstration")

    with help_col:
        with st.container(border=True):
            st.subheader("📘 Help & Evaluation")
            st.write(
                "Review how the controls work, what the visualizations mean, "
                "known conversion differences, and a checklist for testing the "
                "AI-generated implementation."
            )

            if navigate is not None and st.button(
                "Open Help",
                use_container_width=True,
            ):
                navigate("Help")

    st.divider()
    c1, c2 = st.columns([1,1])
    with c1:
        st.subheader("What the Application Contains")

        workspace_col, cspace_col, testing_col = st.columns(3, gap="medium")

        with workspace_col:
            st.markdown("#### Workspace")
            st.write(
                "A Cartesian view of the two-link robot, its revolute joints, "
                "end effector, and the two circular obstacles."
            )

        with cspace_col:
            st.markdown("#### Configuration Space")
            st.write(
                "A sampled map over θ₁ and θ₂ showing which robot configurations "
                "produce obstacle collisions."
            )

        with testing_col:
            st.markdown("#### Evaluation Tools")
            st.write(
                "Collision status, end-effector coordinates, source-locator "
                "coordinates, link-to-obstacle distances, and conversion notes."
            )



    with c2:
        st.subheader("Project Build Workflow")

        convert_col, test_col, evaluate_col = st.columns(3)

        with convert_col:
            st.markdown("#### 1. Convert")
            st.write(
                "Translate the Mathematica controls, robot geometry, collision "
                "logic, and configuration-space visualization into Python."
            )

        with test_col:
            st.markdown("#### 2. Test")
            st.write(
                "Exercise the converted controls and compare numerical and visual "
                "behavior against the original demonstration."
            )

        with evaluate_col:
            st.markdown("#### 3. Evaluate")
            st.write(
                "Document what works correctly as well as faults, missing "
                "functionality, interface differences, and mathematical errors."
            )

    st.success(
        "The interactive conversion is now implemented and ready for systematic testing."
    )
