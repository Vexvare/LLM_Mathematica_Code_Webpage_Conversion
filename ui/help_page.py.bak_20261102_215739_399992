"""Help and reference page for Robot Motion with Obstacles."""

from __future__ import annotations

from collections.abc import Callable

import streamlit as st


NavigationCallback = Callable[[str], None]

SOURCE_URL = "https://demonstrations.wolfram.com/RobotMotionWithObstacles/"


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
        "The configuration-space sampling setting. The final map uses 32Q "
        "samples along each angular axis."
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
        "question": "Can I manipulate the robot directly?",
        "answer": (
            "Yes. Drag either orange obstacle, the blue elbow handle, or the "
            "purple end-effector handle in the workspace. You can also click or "
            "drag the red C-space crosshair to change θ₁ and θ₂."
        ),
    },
    {
        "question": "Why does obstacle dragging feel faster than a normal Streamlit update?",
        "answer": (
            "The workspace and C-space preview update locally in the browser "
            "during the drag. The final state is synchronized back to Python "
            "when you release the pointer."
        ),
    },
    {
        "question": "What happens when I drag the end effector?",
        "answer": (
            "The browser solves the two-link inverse-kinematics problem and "
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
            "Q controls the C-space sampling density. While an obstacle is being "
            "dragged, the live preview uses 8Q samples per axis. When the drag "
            "ends, the final map uses 32Q samples per axis."
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
1. Drag an **orange obstacle** to reposition it.
2. Drag the **blue elbow handle** to change θ₁.
3. Drag the **purple end-effector handle** to reposition the arm using inverse kinematics.
4. Click or drag the **red C-space crosshair** to change θ₁ and θ₂ together.
5. Use the sliders when you need exact values.
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
- **Red crosshair:** the robot's current configuration.
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
                "link angle is relative to the first, so the end-effector "
                "position depends on both θ₁ and θ₂."
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
                "The C-space view repeats this collision test over a grid of "
                "joint-angle pairs."
            )

    st.divider()

    st.subheader("Original Demonstration")

    with st.container(border=True):
        st.markdown(
            """
This application is based on **Robot Motion with Obstacles** by
**Aaron T. Becker** and **Haoran Zhao**, published in the
**Wolfram Demonstrations Project**.
"""
        )

        st.link_button(
            "Open the original Wolfram demonstration ↗",
            SOURCE_URL,
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
- Workspace and C-space interaction run together in one browser component.
- C-space crosshair motion updates the robot immediately without a Streamlit rerun.
- Obstacle drags use an **8Q** live C-space preview and a **32Q** final map.
- The committed state is synchronized back to Python after the drag ends.
- The Python robot model remains available for diagnostics and independent calculations.
"""
        )
