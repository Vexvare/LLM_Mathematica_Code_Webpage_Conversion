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
    with st.container(border=True):
        st.markdown(f"### ⭕ Obstacle {n}")
        c1, c2 = st.columns(2)
        with c1:
            st.number_input("X position", step=.25, format="%.2f", key=f"dh2d_o{n}x")
        with c2:
            st.number_input("Y position", step=.25, format="%.2f", key=f"dh2d_o{n}y")
        st.caption("No ±2 coordinate limit. The viewport fits both obstacles.")


def _diagnostics(joints: tuple[PlanarJoint, ...], obstacles) -> None:
    pose = forward_chain(joints)
    dist = collision_distances(pose, obstacles)
    collision = bool(np.any(dist < COLLISION_CLEARANCE))
    c1, c2, c3, c4 = st.columns(4)
    c1.metric("Robot state", "COLLISION" if collision else "Collision-free")
    c2.metric("Tool (x, y)", f"({pose.end_effector[0]:.2f}, {pose.end_effector[1]:.2f})")
    c3.metric("Links", str(len(joints)))
    c4.metric("Maximum radial bound", f"{sum(j.max_length for j in joints):.2f}")
    with st.expander("Numerical diagnostics, transforms, and link distances"):
        st.dataframe(pd.DataFrame([{"Point": i, "x": float(p[0]), "y": float(p[1])}
                                     for i,p in enumerate(pose.points)]), hide_index=True)
        st.dataframe(pd.DataFrame([
            {"Link": i+1, "Obstacle": k+1, "Centerline distance": float(dist[i,k]),
             "Colliding": bool(dist[i,k] < COLLISION_CLEARANCE)}
            for i in range(len(joints)) for k in range(2)
        ]), hide_index=True)
        st.write(f"Collision threshold = {LINK_HALF_WIDTH:.3f} + {OBSTACLE_RADIUS:.3f} = {COLLISION_CLEARANCE:.3f} units")
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

    with st.expander("C-space slice and viewport settings", expanded=False):
        xcol, ycol, span_col, qcol = st.columns(4)
        with xcol:
            st.selectbox("Horizontal joint", options=list(range(n)), key="dh2d_axis_x",
                         format_func=lambda i: f"J{i+1} ({joints[i].kind})")
        with ycol:
            st.selectbox("Vertical joint", options=list(range(n)), key="dh2d_axis_y",
                         format_func=lambda i: f"J{i+1} ({joints[i].kind})")
        with span_col:
            st.number_input("Minimum plot half-span", min_value=2.2, step=1.,
                            key="dh2d_min_view", help="Increase to drag obstacles farther away.")
        with qcol:
            st.slider("C-space quality Q", 1, 10, key="dh2d_quality")
        st.caption("C-space varies only the selected two joints. The other joints "
                   "stay at the values in the editor. R axes cover 0–360°; P axes cover 0–Stroke.")
    ax_x, ax_y = st.session_state["dh2d_axis_x"], st.session_state["dh2d_axis_y"]
    if ax_x == ax_y:
        st.warning("Select different horizontal and vertical joints to plot C-space.")
        return
    obstacles = (
        (float(st.session_state["dh2d_o1x"]), float(st.session_state["dh2d_o1y"])),
        (float(st.session_state["dh2d_o2x"]), float(st.session_state["dh2d_o2y"])),
    )
    limit = max(2.2, float(st.session_state["dh2d_min_view"]),
                sum(j.max_length for j in joints) + .35,
                *(abs(v) + .5 for ob in obstacles for v in ob))
    st.subheader("Interactive visualization")
    render_interactive_dh_system(
        joints=joints, obstacles=obstacles, axis_x=ax_x, axis_y=ax_y,
        workspace_limit=limit, quality=int(st.session_state["dh2d_quality"]),
        show_angle_arcs=bool(st.session_state["dh2d_show_angle_arcs"]),
        show_labels=bool(st.session_state["dh2d_show_labels"]), key=COMPONENT_KEY,
    )
    st.caption("Drag obstacles, blue intermediate joints, purple tool, or the red C-space crosshair. "
               "Obstacle moves use an 8Q preview; final map uses 32Q. "
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
