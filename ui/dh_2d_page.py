"""Editable 2D serial R/P robot, retaining the original demonstration's interaction.

This page and its canvas component are independent of demonstration_page.py.
"""
from __future__ import annotations

from math import degrees, radians
import numpy as np
import pandas as pd
import streamlit as st

from ui.dh_chain_model import (
    PlanarJoint, MAX_JOINTS, COLLISION_CLEARANCE, LINK_HALF_WIDTH,
    OBSTACLE_RADIUS, rows_to_joints, forward_chain, collision_distances,
)
from ui.dh_interactive_plot import render_interactive_dh_system
from ui.obstacle_geometry import Obstacle, SHAPES, parse_obstacles, obstacle_gaps

EDITOR_KEY = "dh2d_grid_editor"
COMPONENT_KEY = "dh2d_fast_robot_interaction"
NONCE_KEY = "_last_dh2d_interaction_nonce"
DEFAULT_OBSTACLES = ((1.1, 1.1), (1.1, -1.1))


def _new_row(number: int) -> dict:
    return {"Joint": str(number), "Type": "R", "θ (deg)": 0.0,
            "d": 0.0, "a": 1.0, "α (deg)": 0.0, "s": 0.0, "Stroke": 3.0}


def _initial_state() -> None:
    defaults = {
        "dh2d_rows": [_new_row(1), _new_row(2)],
        "dh2d_o1x": 1.1, "dh2d_o1y": 1.1,
        "dh2d_o2x": 1.1, "dh2d_o2y": -1.1,
        "dh2d_o1_shape": "Circle", "dh2d_o2_shape": "Circle",
        "dh2d_o1_radius": .125, "dh2d_o2_radius": .125,
        "dh2d_o1_width": .5, "dh2d_o2_width": .5,
        "dh2d_o1_height": .5, "dh2d_o2_height": .5,
        "dh2d_o1_rotation": 0., "dh2d_o2_rotation": 0.,
        "dh2d_quality": 2,
        "dh2d_show_angle_arcs": True,
        "dh2d_show_labels": True,
        "dh2d_min_view": 2.2,
        "dh2d_axis_x": 0,
        "dh2d_axis_y": 1,
    }
    for key, value in defaults.items():
        if key not in st.session_state:
            st.session_state[key] = value


def _reset_state() -> None:
    for key in list(st.session_state.keys()):
        if key.startswith("dh2d_") or key == NONCE_KEY:
            st.session_state.pop(key, None)
    _initial_state()


def _change_joint_count(delta: int) -> None:
    rows = [dict(row) for row in st.session_state["dh2d_rows"]]
    if delta == 1 and len(rows) < MAX_JOINTS:
        rows.append(_new_row(len(rows) + 1))
    elif delta == -1 and len(rows) > 2:
        rows.pop()
    st.session_state["dh2d_rows"] = rows
    st.session_state.pop(EDITOR_KEY, None)
    if st.session_state.get("dh2d_axis_x", 0) >= len(rows):
        st.session_state["dh2d_axis_x"] = 0
    if st.session_state.get("dh2d_axis_y", 1) >= len(rows):
        st.session_state["dh2d_axis_y"] = 1


def _variable_changed(index: int) -> None:
    """A slider changed; update table before it is rendered again."""
    rows = [dict(row) for row in st.session_state["dh2d_rows"]]
    field = "θ (deg)" if rows[index]["Type"] == "R" else "s"
    rows[index][field] = float(st.session_state[f"dh2d_var_{index}"])
    st.session_state["dh2d_rows"] = rows
    st.session_state.pop(EDITOR_KEY, None)


def _consume_interaction_event() -> None:
    current = st.session_state.get(COMPONENT_KEY)
    event = getattr(current, "event", None)
    if event is None and isinstance(current, dict):
        event = current.get("event")
    if not isinstance(event, dict):
        return
    nonce = event.get("nonce")
    if nonce is None or nonce == st.session_state.get(NONCE_KEY):
        return
    st.session_state[NONCE_KEY] = nonce
    try:
        values = [float(x) for x in event["values"]]
        obstacles = np.asarray(event["obstacles"], dtype=float)
        rows = [dict(row) for row in st.session_state["dh2d_rows"]]
        if len(values) != len(rows) or obstacles.shape != (2, 2):
            return
        if not np.isfinite(values).all() or not np.isfinite(obstacles).all():
            return
        for i, (row, value) in enumerate(zip(rows, values)):
            if row["Type"] == "R":
                row["θ (deg)"] = min(360.0, max(0.0, degrees(value)))
            else:
                row["s"] = min(float(row["Stroke"]), max(0.0, value))
            st.session_state[f"dh2d_var_{i}"] = float(row["θ (deg)" if row["Type"] == "R" else "s"])
        st.session_state["dh2d_rows"] = rows
        st.session_state.pop(EDITOR_KEY, None)
        for n in range(2):
            st.session_state[f"dh2d_o{n+1}x"] = float(obstacles[n, 0])
            st.session_state[f"dh2d_o{n+1}y"] = float(obstacles[n, 1])
    except (KeyError, ValueError, TypeError, OverflowError):
        return


def _editor() -> tuple[PlanarJoint, ...]:
    hdr, add, remove, reset = st.columns([5, 1, 1, 1])
    with hdr:
        st.subheader("Planar joint / DH editor")
    with add:
        st.button("+ Add", key="dh2d_add", use_container_width=True,
                  disabled=len(st.session_state["dh2d_rows"]) >= MAX_JOINTS,
                  on_click=_change_joint_count, args=(1,))
    with remove:
        st.button("− Last", key="dh2d_del", use_container_width=True,
                  disabled=len(st.session_state["dh2d_rows"]) <= 2,
                  on_click=_change_joint_count, args=(-1,))
    with reset:
        if st.button("↺ Reset", key="dh2d_reset", use_container_width=True):
            _reset_state()
            st.rerun()

    st.caption("R: θ is an active rotation (standard DH with d=α=0). "
               "P: θ is a fixed heading; s is in-plane slider extension, from 0 to Stroke. "
               "For P, s is **not** standard-DH d (which would translate along Z). "
               "All links stay in the XY plane. Link lengths may exceed 5 units.")
    edited = st.data_editor(
        pd.DataFrame(st.session_state["dh2d_rows"]),
        key=EDITOR_KEY, num_rows="fixed", hide_index=True, use_container_width=True,
        disabled=["Joint", "d", "α (deg)"],
        column_config={
            "Joint": st.column_config.TextColumn("Joint"),
            "Type": st.column_config.SelectboxColumn("Type", options=["R", "P"], required=True),
            "θ (deg)": st.column_config.NumberColumn("θ (deg)", min_value=-360., max_value=360., step=1., format="%.1f"),
            "d": st.column_config.NumberColumn("d = 0", format="%.1f"),
            "a": st.column_config.NumberColumn("a (length)", min_value=.01, step=.25, format="%.2f"),
            "α (deg)": st.column_config.NumberColumn("α = 0°", format="%.1f"),
            "s": st.column_config.NumberColumn("s (P only)", min_value=0., step=.25, format="%.2f"),
            "Stroke": st.column_config.NumberColumn("P max stroke", min_value=.01, step=.5, format="%.2f"),
        },
    )
    try:
        rows = edited.to_dict("records")
        for i, row in enumerate(rows):
            row["Joint"] = str(i + 1)
            row["Type"] = str(row["Type"]).upper()
            row["d"] = 0.0
            row["α (deg)"] = 0.0
            if row["Type"] == "R":
                row["s"] = 0.0
        joints = rows_to_joints(rows)
    except (KeyError, ValueError, TypeError, OverflowError) as exc:
        st.error(f"Invalid joint table: {exc}")
        st.stop()
    st.session_state["dh2d_rows"] = rows
    with st.expander("Kinematics convention (important for prismatic joints)"):
        st.latex(r"{}^{i-1}T_i = R_z(\theta_i)\,T_x(a_i+s_i)")
        st.write("For R, sᵢ = 0 and θᵢ is the variable. For P, θᵢ is fixed and "
                 "sᵢ varies along the link's local X direction. This is a "
                 "planar R/P extension, not a fully general standard-DH prismatic joint.")
    return joints


def _joint_controls(joints: tuple[PlanarJoint, ...]) -> None:
    with st.container(border=True):
        st.markdown("### 🤖 Joint controls")
        cols = st.columns(min(3, len(joints)))
        for i, joint in enumerate(joints):
            with cols[i % len(cols)]:
                key = f"dh2d_var_{i}"
                val = joint.theta_deg if joint.kind == "R" else joint.extension
                st.session_state[key] = float(val)
                if joint.kind == "R":
                    st.slider(f"J{i+1} θ (R)", -360., 360., step=1., format="%.0f°", key=key,
                              on_change=_variable_changed, args=(i,))
                else:
                    st.slider(f"J{i+1} s (P)", 0., float(joint.stroke), step=.1,
                              key=key, on_change=_variable_changed, args=(i,))
        st.caption("Drag any intermediate blue joint handle to change that joint. "
                   "The purple tool handle uses 2R analytic IK or numerical IK for larger/mixed chains.")


def _obstacle_controls(n: int) -> None:
    """Controls for editable filled obstacles, including their rotation."""
    prefix = f"dh2d_o{n}"
    with st.container(border=True):
        st.markdown(f"### 🟠 Obstacle {n}")
        shape = st.selectbox("Shape", options=SHAPES, key=f"{prefix}_shape")
        c1, c2 = st.columns(2)
        with c1:
            st.number_input("X position", step=.25, format="%.2f", key=f"{prefix}x")
        with c2:
            st.number_input("Y position", step=.25, format="%.2f", key=f"{prefix}y")
        if shape == "Circle":
            st.number_input("Radius", min_value=.001, max_value=10000., step=.125,
                            format="%.3f", key=f"{prefix}_radius")
        else:
            st.number_input("Side length" if shape == "Square" else "Width / base",
                            min_value=.001, max_value=10000., step=.25,
                            format="%.3f", key=f"{prefix}_width")
            if shape != "Square":
                st.number_input("Height", min_value=.001, max_value=10000., step=.25,
                                format="%.3f", key=f"{prefix}_height")
            st.number_input("Rotation (degrees CCW)", step=15., format="%.1f",
                            key=f"{prefix}_rotation")
        st.caption("Drag the orange shape by its center; edit dimensions here."
                   " The workspace scales to include its full extent.")


def _current_obstacles() -> tuple[Obstacle, Obstacle]:
    return tuple(Obstacle(
        x=float(st.session_state[f"dh2d_o{n}x"]),
        y=float(st.session_state[f"dh2d_o{n}y"]),
        shape=str(st.session_state[f"dh2d_o{n}_shape"]),
        radius=float(st.session_state[f"dh2d_o{n}_radius"]),
        width=float(st.session_state[f"dh2d_o{n}_width"]),
        height=float(st.session_state[f"dh2d_o{n}_height"]),
        rotation_deg=float(st.session_state[f"dh2d_o{n}_rotation"]),
    ) for n in (1, 2))


def _diagnostics(joints: tuple[PlanarJoint, ...], obstacles) -> None:
    pose = forward_chain(joints)
    gaps = obstacle_gaps(pose.points, obstacles, LINK_HALF_WIDTH)
    collision = bool(np.any(gaps < 0))
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Robot state", "COLLISION" if collision else "Collision-free")
    c2.metric("Tool (x, y)", f"({pose.end_effector[0]:.2f}, {pose.end_effector[1]:.2f})")
    c3.metric("Links", str(len(joints)))
    c4.metric("Maximum radial bound", f"{sum(j.max_length for j in joints):.2f}")
    with st.expander("Numerical diagnostics, transforms, and link distances"):
        st.dataframe(pd.DataFrame([{"Point": i, "x": float(p[0]), "y": float(p[1])}
                                     for i,p in enumerate(pose.points)]), hide_index=True)
        st.dataframe(pd.DataFrame([
            {"Link": i+1, "Obstacle": k+1, "Shape": obstacles[k].shape,
             "Signed clearance": float(gaps[i,k]), "Colliding": bool(gaps[i,k] < 0)}
            for i in range(len(joints)) for k in range(2)
        ]), hide_index=True)
        st.caption("Negative signed clearance means the link overlaps the filled obstacle. "
                   "The link has half-width 0.125 units. For circles, the radius is included "
                   "in the clearance. Polygons use segment-to-filled-polygon distance.")
        st.markdown("**Base-to-tool homogeneous transform (4 × 4)**")
        st.dataframe(pd.DataFrame(pose.transforms[-1]).round(6), hide_index=True)


def display_dh_2d_page() -> None:
    _initial_state()
    _consume_interaction_event()
    st.title("Interactive Planar Robot — DH Editor")
    st.caption("Assignment 2 extension · multiple links · revolute and in-plane prismatic joints")
    joints = _editor()
    n = len(joints)
    for key, default in (("dh2d_axis_x", 0), ("dh2d_axis_y", 1)):
        if st.session_state.get(key) not in range(n):
            st.session_state[key] = default
    if st.session_state["dh2d_axis_x"] == st.session_state["dh2d_axis_y"]:
        st.session_state["dh2d_axis_y"] = (st.session_state["dh2d_axis_x"] + 1) % n

    st.subheader("Configuration-space axis selection")
    st.caption("Pick ANY two joints as the horizontal and vertical variables. "
               "Other joints remain fixed at their current settings. "
               "R joints vary θ (0–360°); P joints vary s (0–Stroke).")
    xcol, ycol = st.columns(2)
    with xcol:
        st.selectbox("Horizontal axis (X)", options=list(range(n)), key="dh2d_axis_x",
                     format_func=lambda i: f"J{i+1} — {'θ (revolute)' if joints[i].kind == 'R' else 's (prismatic)'}")
    with ycol:
        st.selectbox("Vertical axis (Y)", options=list(range(n)), key="dh2d_axis_y",
                     format_func=lambda i: f"J{i+1} — {'θ (revolute)' if joints[i].kind == 'R' else 's (prismatic)'}")
    with st.expander("Viewport and C-space resolution settings", expanded=False):
        qcol, span_col = st.columns(2)
        with qcol:
            st.slider("C-space quality Q", 1, 10, key="dh2d_quality")
        with span_col:
            st.number_input("Minimum plot half-span", min_value=2.2, step=1.,
                            key="dh2d_min_view", help="Increase to drag obstacles farther away.")
        st.caption("The map uses 8Q × 8Q preview cells while obstacles move and "
                   "32Q × 32Q cells after release. Non-circular shapes and long chains "
                   "may make high Q settings slower.")
    ax_x, ax_y = st.session_state["dh2d_axis_x"], st.session_state["dh2d_axis_y"]
    if ax_x == ax_y:
        st.warning("Choose two different joints for the C-space axes.")
        st.stop()
    obstacles = _current_obstacles()
    limit = max(2.2, float(st.session_state["dh2d_min_view"]),
                sum(j.max_length for j in joints) + .35,
                *(max(abs(ob.x),abs(ob.y)) + ob.extent + .5 for ob in obstacles))
    st.subheader("Interactive visualization")
    render_interactive_dh_system(
        joints=joints, obstacles=obstacles, axis_x=ax_x, axis_y=ax_y,
        workspace_limit=limit, quality=int(st.session_state["dh2d_quality"]),
        show_angle_arcs=bool(st.session_state["dh2d_show_angle_arcs"]),
        show_labels=bool(st.session_state["dh2d_show_labels"]), key=COMPONENT_KEY,
    )
    st.caption("Drag orange obstacles, blue joint handles, the purple tool, or the red C-space crosshair. "
               "Use the obstacle controls below to change shape, size, or rotation. "
               "Numerical IK for 3+ or mixed joints is approximate.")
    st.subheader("Controls")
    _joint_controls(joints)
    o1, o2, settings = st.columns([1, 1, 1])
    with o1: _obstacle_controls(1)
    with o2: _obstacle_controls(2)
    with settings:
        with st.container(border=True):
            st.markdown("### ⚙️ Display")
            st.toggle("Angle arcs", key="dh2d_show_angle_arcs")
            st.toggle("Labels", key="dh2d_show_labels")
            st.caption(f"C-space grid: {32*int(st.session_state['dh2d_quality'])} × {32*int(st.session_state['dh2d_quality'])}")
    st.subheader("Current state and diagnostics")
    _diagnostics(joints, obstacles)
