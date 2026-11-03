"""Home page for the original Mathematica conversion and Assignment 2 DH extension."""
from __future__ import annotations
from collections.abc import Callable
import streamlit as st

NavigationCallback = Callable[[str], None]


def display_home_page(navigate: NavigationCallback | None = None) -> None:
    st.title("Robot Motion with Obstacles")
    st.caption("Interactive 2D robotics demonstrations · Mathematica conversion and DH extension")
    st.markdown(
        "Explore **robot kinematics, obstacles, and configuration space** without Mathematica. "
        "The original two-link demonstration is preserved, and a separate DH Editor "
        "adds configurable planar robots and editable obstacles."
    )
    st.divider()

    original, editor, help_col = st.columns(3, gap="large")
    with original:
        with st.container(border=True):
            st.subheader("🤖 Demonstration")
            st.caption("Original conversion · two revolute joints")
            st.write("Drag the elbow, tool, obstacles, and C-space crosshair. "
                     "Study the original two-link robot with two circular obstacles.")
            if navigate is not None and st.button("Open Demonstration", type="primary",
                                                 use_container_width=True):
                navigate("Demonstration")
    with editor:
        with st.container(border=True):
            st.subheader("🧮 DH Editor")
            st.caption("Assignment 2 · editable planar serial robot")
            st.write("Add up to 12 revolute or in-plane prismatic joints; edit link "
                     "lengths, choose two C-space variables, and customize obstacle shapes.")
            if navigate is not None and st.button("Open DH Editor", type="primary",
                                                 use_container_width=True):
                navigate("DH Editor")
    with help_col:
        with st.container(border=True):
            st.subheader("📘 Help & Reference")
            st.caption("Controls, coordinate systems, conventions")
            st.write("Learn how dragging works, what a configuration-space slice means, "
                     "and how planar prismatic motion differs from standard DH d.")
            if navigate is not None and st.button("Open Help", use_container_width=True):
                navigate("Help")

    st.divider()
    st.subheader("What you can investigate")
    c1, c2, c3 = st.columns(3, gap="large")
    with c1:
        st.markdown("#### Robot workspace")
        st.write("Move robot joints and the end effector, adjust link lengths, "
                 "and observe the physical links in XY space.")
    with c2:
        st.markdown("#### Collision and obstacles")
        st.write("Drag obstacles or change their type to circle, square, rectangle, or triangle; "
                 "edit sizes and rotate polygons in the DH Editor.")
    with c3:
        st.markdown("#### Configuration space")
        st.write("See colliding joint configurations. In the DH Editor, select any two "
                 "joint variables as the plot axes while holding other joints fixed.")

    st.info("The DH Editor is an educational **planar R/P extension**. Prismatic joints "
            "extend along the link's local X axis using s; they do not implement "
            "the standard-DH Z-axis d displacement.")
