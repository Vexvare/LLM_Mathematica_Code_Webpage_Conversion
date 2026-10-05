"""Help and reference page for the Robot Motion with Obstacles application."""

from __future__ import annotations

from collections.abc import Callable

import streamlit as st


NavigationCallback = Callable[[str], None]


GLOSSARY = {
    "Workspace": (
        "The Cartesian x-y plane containing the robot arm and circular obstacles."
    ),
    "Configuration Space (C-space)": (
        "A space whose coordinates are the robot joint variables. Here the axes "
        "are θ₁ and θ₂, and each point represents one complete robot pose."
    ),
    "Collision Cell": (
        "A sampled C-space configuration in which at least one robot link "
        "intersects an obstacle."
    ),
    "Forward Kinematics": (
        "The calculation used to determine the elbow and end-effector positions "
        "from θ₁ and θ₂."
    ),
    "Inverse Kinematics": (
        "The calculation used to determine joint angles from a desired "
        "end-effector position."
    ),
    "Quality (Q)": (
        "The configuration-space sampling setting. The displayed map uses "
        "32Q samples along each angular axis."
    ),
    "Periodic Angles": (
        "Joint angles repeat every 2π radians, so 0° and 360° describe the "
        "same physical orientation."
    ),
}


FAQS = [
    {
        "question": "What does this application show?",
        "answer": (
            "It models a two-link planar robot arm moving around two circular "
            "obstacles. The workspace shows the physical robot pose, while the "
            "configuration-space plot shows which combinations of θ₁ and θ₂ "
            "produce collisions."
        ),
    },
    {
        "question": "Why are the joint controls shown in degrees?",
        "answer": (
            "Degrees are easier to read and adjust interactively. The robot model "
            "converts those values to radians internally for its calculations."
        ),
    },
    {
        "question": "Can I manipulate the robot directly in the plots?",
        "answer": (
            "Yes. In the workspace you can drag either orange obstacle, the "
            "blue elbow handle, or the purple end-effector handle. In "
            "configuration space you can drag the red marker to change θ₁ and θ₂."
        ),
    },
    {
        "question": "What happens when I drag the end effector?",
        "answer": (
            "The application solves the two-link inverse-kinematics problem and "
            "selects the solution closest to the robot's current pose. Targets "
            "outside the reachable workspace are projected onto the nearest "
            "reachable boundary."
        ),
    },
    {
        "question": "What do the dark regions in configuration space mean?",
        "answer": (
            "Dark cells represent sampled joint configurations where at least "
            "one robot link intersects an obstacle. Light cells are sampled "
            "collision-free configurations."
        ),
    },
    {
        "question": "What does Q change?",
        "answer": (
            "Q changes the sampling density of the configuration-space map. "
            "Higher values produce a finer map but require more computation."
        ),
    },
    {
        "question": "Why might the C-space boundary look blocky?",
        "answer": (
            "The map is sampled on a finite grid. Increasing Q increases the "
            "number of samples and makes the displayed collision boundary finer."
        ),
    },
]


def display_help_page(navigate: NavigationCallback | None = None) -> None:
    """Render usage help and robotics reference information."""
    st.title("Help & Reference")
    st.caption("Using Robot Motion with Obstacles")

    if navigate is not None:
        home_col, demo_col, _ = st.columns([1, 1.2, 3.5])

        with home_col:
            if st.button("← Home", use_container_width=True):
                navigate("Home")

        with demo_col:
            if st.button(
                "Open Demonstration",
                type="primary",
                use_container_width=True,
            ):
                navigate("Demonstration")

    st.divider()

    st.subheader("Using the Demonstration")

    controls_col, plots_col = st.columns(2, gap="large")

    with controls_col:
        with st.container(border=True):
            st.markdown("### Controls")
            st.markdown(
                """
1. Drag an **orange obstacle** to reposition it in the workspace.
2. Drag the **blue elbow handle** to change θ₁.
3. Drag the **purple end-effector handle** to reposition the arm using inverse kinematics.
4. Drag the **red C-space marker** to change θ₁ and θ₂ together.
5. Use the joint and obstacle sliders when you need exact values.
6. Change **Q** to adjust configuration-space sampling resolution.
7. Use the display toggles to show or hide workspace annotations.
8. Use **Reset** to return to the default state.
"""
            )

    with plots_col:
        with st.container(border=True):
            st.markdown("### Reading the Plots")
            st.markdown(
                """
- **Robot Workspace:** the arm and obstacles in Cartesian coordinates.
- **Configuration Space:** joint-angle combinations mapped to collision state.
- **Dark C-space cells:** sampled collision configurations.
- **Light C-space cells:** sampled collision-free configurations.
- **Red marker:** the robot's current configuration.
"""
            )

    st.divider()

    st.subheader("Understanding the Model")

    model_left, model_right = st.columns(2, gap="large")

    with model_left:
        with st.container(border=True):
            st.markdown("### Robot Geometry")
            st.write(
                "The robot has two revolute links of equal length. The second "
                "link angle is relative to the first, so the end-effector position "
                "depends on both θ₁ and θ₂."
            )

            st.latex(r"x_1 = \cos(\theta_1)")
            st.latex(r"y_1 = \sin(\theta_1)")
            st.latex(r"x_2 = x_1 + \cos(\theta_1+\theta_2)")
            st.latex(r"y_2 = y_1 + \sin(\theta_1+\theta_2)")

    with model_right:
        with st.container(border=True):
            st.markdown("### Collision Checking")
            st.write(
                "Each obstacle is treated as a circle. A collision occurs when "
                "an obstacle center comes within the combined obstacle radius and "
                "link half-width of either robot link segment."
            )

            st.write(
                "The C-space plot repeats this collision test over a grid of "
                "joint-angle pairs."
            )

    st.divider()

    st.subheader("Frequently Asked Questions")

    for item in FAQS:
        with st.expander(item["question"]):
            st.write(item["answer"])

    st.divider()

    st.subheader("Glossary")
    glossary_cols = st.columns(2)

    for index, (term, definition) in enumerate(GLOSSARY.items()):
        with glossary_cols[index % 2]:
            with st.container(border=True):
                st.markdown(f"**{term}**")
                st.caption(definition)

    st.divider()

    st.subheader("Implementation Notes")

    with st.expander("View technical notes", expanded=False):
        st.markdown(
            """
- Drag interactions are synchronized back to the Python model when the drag is committed.
- Configuration space is sampled on a finite grid, so its visual resolution depends on **Q**.
- The application keeps joint and obstacle sliders available alongside direct manipulation for precise input.
- Plot navigation controls remain available for zooming, panning, and resetting the visualizations.
"""
        )
