"""Help and reference page for the Robot Motion with Obstacles project."""

from __future__ import annotations

from collections.abc import Callable
import platform
import sys

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
    "Quality (Q)": (
        "The source demonstration's configuration-space resolution setting. "
        "The converted high-resolution map uses 32Q samples on each angular axis."
    ),
    "Source-compatible C-space": (
        "The mode that preserves the second-link sign convention found in the "
        "Mathematica configuration-space raster."
    ),
    "Source Locator": (
        "The coordinate of Mathematica's original phase-space locator that "
        "corresponds to the current θ₁ and θ₂ values."
    ),
    "Conversion Fault": (
        "A missing, incorrect, inconsistent, or misleading behavior discovered "
        "when the Streamlit conversion is compared with the source demonstration."
    ),
}


FAQS = [
    {
        "question": "What does this demonstration show?",
        "answer": (
            "It models a two-link planar robot arm moving around two circular "
            "obstacles. The workspace shows the physical robot pose, while the "
            "configuration-space plot shows which combinations of θ₁ and θ₂ "
            "result in collision."
        ),
    },
    {
        "question": "Why are the joint controls in degrees when the model uses radians?",
        "answer": (
            "Degrees are used in the interface because they are easier to read "
            "and adjust interactively. The values are converted to radians "
            "before the robot model performs its calculations."
        ),
    },
    {
        "question": "Can I manipulate the robot directly in the plots?",
        "answer": (
            "Yes. In the workspace you can drag either orange obstacle, the "
            "blue elbow handle, or the purple end-effector handle. In "
            "configuration space you can drag the red marker to change θ₁ and "
            "θ₂. The ordinary sliders remain available for precise values."
        ),
    },
    {
        "question": "What does the dark region in configuration space mean?",
        "answer": (
            "Dark sampled cells represent joint configurations where the model "
            "detects a collision between an obstacle and at least one robot link. "
            "Light cells are sampled collision-free configurations."
        ),
    },
    {
        "question": "What does Q change?",
        "answer": (
            "Q changes the sampling resolution of the configuration-space map. "
            "The converted high-resolution grid contains 32Q samples along each "
            "joint-angle axis, so increasing Q produces a denser calculation."
        ),
    },
    {
        "question": "What is the source-compatible C-space option?",
        "answer": (
            "The Mathematica source draws the workspace second link using "
            "θ₁ + θ₂, but its configuration-space raster contains an a₁ - a₂ "
            "expression. The option preserves that source behavior rather than "
            "silently changing it."
        ),
    },
    {
        "question": "Should I assume the conversion is correct because it looks right?",
        "answer": (
            "No. The assignment requires testing. Compare controls, collision "
            "boundaries, robot poses, C-space patterns, numerical values, and "
            "edge cases against the original Mathematica demonstration."
        ),
    },
]


TESTING_GUIDE = [
    "Verify the default robot pose and obstacle positions against the Mathematica demonstration.",
    "Test θ₁ and θ₂ near 0°, 90°, 180°, 270°, and 360° and compare robot orientation.",
    "Move each obstacle independently and verify the workspace and C-space both update.",
    "Place an obstacle clearly on a link and confirm the application reports a collision.",
    "Move an obstacle clearly away from both links and confirm the collision state clears.",
    "Compare C-space patterns at several obstacle locations with the original demonstration.",
    "Change Q from low to high values and confirm the map resolution changes without changing the underlying geometry.",
    "Test periodic behavior near 0° and 360°.",
    "Compare the source-compatible C-space option against the alternate sign convention.",
    "Record successful behavior as carefully as faults; the assignment requires both.",
]


KNOWN_LIMITS = [
    (
        "The conversion now supports direct drag interaction, but a drag is "
        "committed when the mouse is released rather than continuously "
        "recomputing the full Python model during every pointer movement."
    ),
    (
        "The Mathematica demonstration can temporarily lower C-space resolution "
        "while a locator is actively moving; this Streamlit build commits the "
        "drag first and then calculates the final high-resolution grid."
    ),
    (
        "The source's workspace and configuration-space expressions use "
        "different second-link angle signs. The application exposes this "
        "difference through the source-compatible C-space option for testing."
    ),
    (
        "The configuration space is sampled on a finite grid, so its displayed "
        "collision boundary is an approximation whose visual resolution depends on Q."
    ),
]


def _system_info() -> None:
    """Display lightweight runtime information useful when reporting a problem."""
    st.caption(
        f"Runtime: Python {sys.version.split()[0]} • "
        f"{platform.system()} {platform.release()}"
    )


def display_help_page(navigate: NavigationCallback | None = None) -> None:
    """Render usage help, technical reference, and evaluation guidance."""
    st.title("Help & Reference")
    st.caption("Using and evaluating the Robot Motion with Obstacles conversion")

    if navigate is not None:
        home_col, demo_col, spacer = st.columns([1, 1.2, 3.5])

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

    usage_col, plot_col = st.columns(2, gap="large")

    with usage_col:
        with st.container(border=True):
            st.markdown("### Controls")
            st.markdown(
                """
1. Drag an **orange obstacle** in the workspace to reposition it directly.
2. Drag the **blue elbow ring** to change θ₁.
3. Drag the **purple end-effector ring** to reposition the arm using inverse kinematics.
4. Drag the **red C-space marker** to change θ₁ and θ₂ together.
5. Use the joint and obstacle sliders when you need exact values.
6. Change **Q** to adjust configuration-space sampling resolution.
7. Use the display toggles to show or hide workspace annotations.
8. Use **Reset** to return to the source default state.
"""
            )

    with plot_col:
        with st.container(border=True):
            st.markdown("### Reading the Plots")
            st.markdown(
                """
- **Robot Workspace:** shows the arm and obstacles in Cartesian coordinates.
- **Configuration Space:** maps joint-angle combinations to collision state.
- **Dark C-space cells:** sampled collision configurations.
- **Light C-space cells:** sampled collision-free configurations.
- **Red X marker:** the robot's current configuration.
"""
            )

    st.divider()

    st.subheader("Testing Checklist")
    for number, item in enumerate(TESTING_GUIDE, start=1):
        st.markdown(f"{number}. {item}")

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

    st.subheader("Known Conversion Limitations")
    with st.expander("View current limitations", expanded=True):
        for item in KNOWN_LIMITS:
            st.markdown(f"- {item}")

    st.info(
        "For the assignment report, record the exact input or control state, "
        "what this application produced, what Mathematica produced, and why "
        "you classified the result as correct or faulty."
    )

    _system_info()
