"""Help page for original demonstration and expanded, planar DH Editor."""
from __future__ import annotations
from collections.abc import Callable
import streamlit as st

NavigationCallback = Callable[[str], None]
SOURCE_URL = "https://demonstrations.wolfram.com/RobotMotionWithObstacles/"


def display_help_page(navigate: NavigationCallback | None = None) -> None:
    st.title("Help & Reference")
    st.caption("Robot Motion with Obstacles · Demonstration and DH Editor")
    if navigate is not None:
        a, b, c = st.columns([1, 1, 3])
        with a:
            if st.button("← Home", use_container_width=True, key="help_home"):
                navigate("Home")
        with b:
            if st.button("Demonstration", use_container_width=True, key="help_demo"):
                navigate("Demonstration")
        with c:
            if st.button("Open DH Editor", type="primary", key="help_dh_editor"):
                navigate("DH Editor")

    st.divider()
    st.subheader("Two different modes")
    left, right = st.columns(2, gap="large")
    with left:
        with st.container(border=True):
            st.markdown("### Demonstration — original 2R robot")
            st.write("A faithful two-link planar demonstration with fixed equal link lengths "
                     "and two circular obstacles. Move the blue elbow, purple tool, orange "
                     "obstacles, or red C-space crosshair. It is separate from the DH Editor.")
    with right:
        with st.container(border=True):
            st.markdown("### DH Editor — extended planar robot")
            st.write("Build a serial chain of 2–12 links, choose R or P for each joint, "
                     "change link sizes, and explore joint-variable C-space slices. "
                     "Obstacle shapes and dimensions can be customized here.")

    st.subheader("Using the DH Editor")
    with st.container(border=True):
        st.markdown("""
1. **Add or remove joints:** use **+ Add** / **− Last** above the editor table.
2. **Choose joint type:** use **R** (revolute) or **P** (in-plane sliding) in the table.
3. **Set the geometry:** `a` is the fixed link length; `θ (deg)` is the rotation/heading. For P, set `Stroke` and vary the in-plane extension `s`.
4. **Choose the C-space axes:** the **Horizontal axis (X)** and **Vertical axis (Y)** menus can select *any two distinct joints*; the other joint variables remain at their currently entered values.
5. **Manipulate graphically:** drag orange obstacles, blue intermediate joint handles, the purple tool, or the red C-space crosshair. Table and numerical values synchronize after a drag is released.
6. **Customize obstacles:** choose Circle, Square, Rectangle, or Triangle in each obstacle control card; enter radius or side/width/height. Rotate polygons with *Rotation (degrees CCW)*.
7. **Move beyond the original coordinates:** obstacle position fields accept larger coordinates and the viewport expands automatically. Increase *Minimum plot half-span* to drag farther in the plot.
8. **Tune detail:** quality Q controls C-space sampling, with an 8Q preview while dragging obstacles and 32Q final grid; toggles control labels and angle arcs.
9. **Reset:** returns the editor and obstacles to two unit-length revolute joints and the original circular obstacles.
""")

    st.subheader("Reading configuration-space slices")
    st.write("The complete configuration space of an N-joint manipulator is N-dimensional. "
             "The DH Editor therefore displays a **two-dimensional slice**, not a projection "
             "of all reachable configurations. The selected variables form the axes; "
             "every other joint is held fixed at its current table value.")
    st.markdown("""
- A **revolute (R) axis** runs from 0° to 360°.
- A **prismatic (P) axis** runs from 0 to the joint's specified Stroke.
- **Dark cells** represent sampled configurations in collision with at least one obstacle.
- **Light cells** represent sampled collision-free configurations.
- **Red crosshair** identifies the current values of the two selected joints; drag it to change them.
""")

    st.subheader("Mathematical conventions")
    col1, col2 = st.columns(2, gap="large")
    with col1:
        with st.container(border=True):
            st.markdown("#### Forward kinematics")
            st.latex(r"\phi_i=\sum_{k=1}^{i}\theta_k")
            st.latex(r"\mathbf p_i=\mathbf p_{i-1}+(a_i+s_i)\begin{bmatrix}\cos\phi_i\\\sin\phi_i\end{bmatrix}")
            st.write("For R joints, sᵢ is zero and θᵢ varies. For P joints, θᵢ is "
                     "fixed while sᵢ varies within its configured stroke.")
    with col2:
        with st.container(border=True):
            st.markdown("#### Important DH limitation")
            st.write("Standard DH prismatic motion changes **d** along a local Z axis. "
                     "This strictly 2D extension instead changes **s** along local X. "
                     "Thus it is a DH-inspired planar joint model, not a complete "
                     "3D DH implementation. d and α remain zero.")

    st.subheader("Obstacle collision model")
    st.write("Robot links are capsules of fixed half-width 0.125 units. Circles use their "
             "specified radius. Squares, rectangles and triangles are treated as **filled polygons**, "
             "including their rotation. A collision is detected when the link capsule "
             "overlaps an obstacle. C-space uses the same shapes and dimensions. "
             "A boundary is sampled, so very small obstacles may need a higher Q.")

    with st.expander("Frequently asked questions"):
        st.markdown("**Can I select θ₂ and θ₄ instead of θ₁ and θ₂?** Yes. Add enough "
                    "revolute joints, then choose J2 and J4 in the two axis menus.")
        st.markdown("**Can I use a prismatic axis?** Yes. It is plotted in extension units (s), "
                    "not in degrees; the other axis can be another P or an R joint.")
        st.markdown("**Does changing an obstacle size affect the collision map?** Yes. "
                    "The collision checks use the selected filled shape and dimensions.")
        st.markdown("**Why doesn't the tool always reach my mouse position?** Exact analytic "
                    "IK is available for a two-revolute-joint chain. Other chains use an "
                    "approximate numerical solver that can stop at local solutions or limits.")
        st.markdown("**Why is the C-space map slow at high quality?** Each cell requires "
                    "collision checking for all links and obstacle edges, and polygon shapes "
                    "are more expensive than circles.")

    st.divider()
    st.subheader("Original Mathematica demonstration")
    st.write("Based on *Robot Motion with Obstacles* by Aaron T. Becker and Haoran Zhao "
             "in the Wolfram Demonstrations Project.")
    st.link_button("View the source demonstration", SOURCE_URL)
