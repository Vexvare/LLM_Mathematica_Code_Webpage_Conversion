"""Stable direct-manipulation layer for the robotics demonstration.

This version does NOT use Plotly editable ``layout.shapes`` for dragging.
Instead, it renders one normal Plotly graph and places a transparent SVG
interaction layer directly on top of that same graph.

Why
---
Plotly editable shapes worked inconsistently inside Streamlit custom components:
their edit layer could detach from the graph and component resizing could create
runaway height changes.  The SVG overlay avoids both issues:

* Plotly owns the chart, axes, grid, heatmap, robot traces, and mode bar.
* SVG handles are positioned from Plotly's own axis transforms.
* Handles therefore appear directly on top of the data coordinates.
* Only handle elements receive pointer events; the rest of the overlay is
  transparent, so Plotly hover/pan/zoom controls still work.
* The Plotly drawing height is fixed; only width responds to browser resizing.

The Python page still receives the same events:
``obstacle1``, ``obstacle2``, ``elbow``, ``end_effector``, and
``configuration``.
"""

from __future__ import annotations

import json
from typing import Iterable

import numpy as np
import plotly.graph_objects as go
import streamlit as st

from ui.robot_model import (
    DEFAULT_GEOMETRY,
    RobotPose,
    TWO_PI,
)


# ---------------------------------------------------------------------------
# Component frontend
# ---------------------------------------------------------------------------

_COMPONENT_HTML = """
<div class="robot-component">
    <div class="robot-stage">
        <div class="robot-plot"></div>
        <svg class="robot-overlay" aria-label="Draggable robot controls"></svg>
    </div>
    <div class="robot-hint"></div>
</div>
"""


_COMPONENT_CSS = """
.robot-component {
    width: 100%;
    min-width: 0;
    box-sizing: border-box;
    overflow: hidden;
}

.robot-stage {
    position: relative;
    width: 100%;
    height: 500px;
    min-width: 0;
    overflow: hidden;
    box-sizing: border-box;
}

.robot-plot {
    position: absolute;
    inset: 0;
    width: 100%;
    height: 500px;
    min-width: 0;
    box-sizing: border-box;
}

.robot-overlay {
    position: absolute;
    inset: 0;
    width: 100%;
    height: 500px;
    overflow: hidden;
    pointer-events: none;
    z-index: 8;
}

.robot-overlay .drag-handle {
    pointer-events: all;
    cursor: grab;
    touch-action: none;
}

.robot-overlay .drag-handle:active {
    cursor: grabbing;
}

.robot-overlay .handle-label {
    pointer-events: none;
    user-select: none;
    font-family: sans-serif;
    font-size: 12px;
    font-weight: 600;
}

.robot-hint {
    width: 100%;
    box-sizing: border-box;
    color: var(--st-text-color);
    opacity: 0.68;
    font-size: 0.78rem;
    line-height: 1.35;
    padding: 0.35rem 0.25rem 0.10rem 0.25rem;
    overflow-wrap: anywhere;
}
"""


_COMPONENT_JS = r"""
import Plotly from "https://cdn.jsdelivr.net/npm/plotly.js-dist-min@3.1.0/+esm";

export default function(component) {
    const {
        parentElement,
        data,
        setStateValue,
    } = component;

    const stage = parentElement.querySelector(".robot-stage");
    const plot = parentElement.querySelector(".robot-plot");
    const overlay = parentElement.querySelector(".robot-overlay");
    const hint = parentElement.querySelector(".robot-hint");

    if (!stage || !plot || !overlay) {
        throw new Error("Robot component could not find its rendering elements.");
    }

    hint.textContent = data.hint ?? "";

    // Clean listeners/observers from a prior render of this component instance.
    if (plot.__robotRelayoutHandler && plot.removeListener) {
        plot.removeListener("plotly_relayout", plot.__robotRelayoutHandler);
    }
    if (plot.__robotAfterPlotHandler && plot.removeListener) {
        plot.removeListener("plotly_afterplot", plot.__robotAfterPlotHandler);
    }
    if (stage.__robotResizeObserver) {
        stage.__robotResizeObserver.disconnect();
        stage.__robotResizeObserver = null;
    }

    const incomingFigure = data.figure ?? {data: [], layout: {}};

    const figure = {
        data: JSON.parse(JSON.stringify(incomingFigure.data ?? [])),
        layout: JSON.parse(JSON.stringify(incomingFigure.layout ?? {})),
    };

    // Fixed vertical size prevents recursive Streamlit/Plotly height growth.
    delete figure.layout.width;
    delete figure.layout.uirevision;
    figure.layout.height = 500;
    figure.layout.autosize = true;

    const config = {
        responsive: true,
        displaylogo: false,
        displayModeBar: true,
        scrollZoom: true,
        editable: false,
        ...data.config,
    };

    // Each handle lives in data coordinates. The overlay SVG is only the
    // interaction/rendering layer; its positions are derived from Plotly axes.
    const handles = (data.handles ?? []).map((handle) => ({...handle}));

    let cancelled = false;
    let activeDrag = null;

    const svgNS = "http://www.w3.org/2000/svg";

    const getAxes = () => {
        const full = plot._fullLayout;
        if (!full?.xaxis || !full?.yaxis) {
            return null;
        }
        return {
            xaxis: full.xaxis,
            yaxis: full.yaxis,
        };
    };

    const dataToPixel = (x, y) => {
        const axes = getAxes();
        if (!axes) {
            return null;
        }

        return {
            x: axes.xaxis._offset + axes.xaxis.l2p(Number(x)),
            y: axes.yaxis._offset + axes.yaxis.l2p(Number(y)),
        };
    };

    const pixelToData = (clientX, clientY) => {
        const axes = getAxes();
        if (!axes) {
            return null;
        }

        const rect = plot.getBoundingClientRect();
        const px = clientX - rect.left;
        const py = clientY - rect.top;

        return {
            x: axes.xaxis.p2l(px - axes.xaxis._offset),
            y: axes.yaxis.p2l(py - axes.yaxis._offset),
        };
    };

    const clampValue = (value, bounds) => {
        if (!Array.isArray(bounds) || bounds.length !== 2) {
            return value;
        }

        return Math.max(
            Number(bounds[0]),
            Math.min(Number(bounds[1]), value),
        );
    };

    const handleRadiusPx = (handle) => {
        if (Number.isFinite(Number(handle.radius_px))) {
            return Number(handle.radius_px);
        }

        if (Number.isFinite(Number(handle.radius_data))) {
            const center = dataToPixel(handle.x, handle.y);
            const edge = dataToPixel(
                Number(handle.x) + Number(handle.radius_data),
                handle.y,
            );

            if (center && edge) {
                return Math.max(5, Math.abs(edge.x - center.x));
            }
        }

        return 10;
    };

    const clearOverlay = () => {
        while (overlay.firstChild) {
            overlay.removeChild(overlay.firstChild);
        }
    };

    const positionHandleGroup = (group, handle) => {
        const pixel = dataToPixel(handle.x, handle.y);
        if (!pixel) {
            return;
        }

        const circle = group.querySelector("circle");
        const label = group.querySelector("text");

        circle.setAttribute("cx", String(pixel.x));
        circle.setAttribute("cy", String(pixel.y));
        circle.setAttribute("r", String(handleRadiusPx(handle)));

        if (label) {
            label.setAttribute("x", String(pixel.x));
            label.setAttribute(
                "y",
                String(pixel.y - handleRadiusPx(handle) - 8),
            );
        }
    };

    const emitHandle = (handle) => {
        setStateValue("event", {
            kind: handle.kind,
            x: Number(handle.x),
            y: Number(handle.y),
            nonce: Date.now(),
        });
    };

    const startDrag = (event, handle, group) => {
        event.preventDefault();
        event.stopPropagation();

        activeDrag = {
            pointerId: event.pointerId,
            handle,
            group,
        };

        const circle = group.querySelector("circle");
        if (circle?.setPointerCapture) {
            try {
                circle.setPointerCapture(event.pointerId);
            } catch (_) {
                // Pointer capture is an enhancement, not a requirement.
            }
        }
    };

    const continueDrag = (event) => {
        if (
            !activeDrag ||
            event.pointerId !== activeDrag.pointerId
        ) {
            return;
        }

        event.preventDefault();
        event.stopPropagation();

        const position = pixelToData(event.clientX, event.clientY);
        if (!position) {
            return;
        }

        const handle = activeDrag.handle;

        handle.x = clampValue(
            Number(position.x),
            handle.x_bounds,
        );
        handle.y = clampValue(
            Number(position.y),
            handle.y_bounds,
        );

        positionHandleGroup(activeDrag.group, handle);
    };

    const finishDrag = (event) => {
        if (
            !activeDrag ||
            event.pointerId !== activeDrag.pointerId
        ) {
            return;
        }

        event.preventDefault();
        event.stopPropagation();

        const handle = activeDrag.handle;
        activeDrag = null;

        emitHandle(handle);
    };

    const drawHandles = () => {
        if (cancelled || !plot._fullLayout) {
            return;
        }

        const rect = plot.getBoundingClientRect();
        const width = Math.max(1, rect.width);
        const height = 500;

        overlay.setAttribute("viewBox", `0 0 ${width} ${height}`);
        overlay.setAttribute("width", String(width));
        overlay.setAttribute("height", String(height));

        clearOverlay();

        for (const handle of handles) {
            const group = document.createElementNS(svgNS, "g");
            group.setAttribute("data-kind", handle.kind);

            const circle = document.createElementNS(svgNS, "circle");
            circle.classList.add("drag-handle");
            circle.setAttribute("fill", handle.fill ?? "rgba(0,0,0,0.12)");
            circle.setAttribute("stroke", handle.stroke ?? "#111827");
            circle.setAttribute(
                "stroke-width",
                String(handle.stroke_width ?? 3),
            );

            const title = document.createElementNS(svgNS, "title");
            title.textContent = handle.title ?? `Drag ${handle.kind}`;
            circle.appendChild(title);

            circle.addEventListener(
                "pointerdown",
                (event) => startDrag(event, handle, group),
            );
            circle.addEventListener("pointermove", continueDrag);
            circle.addEventListener("pointerup", finishDrag);
            circle.addEventListener("pointercancel", finishDrag);

            group.appendChild(circle);

            if (handle.label) {
                const label = document.createElementNS(svgNS, "text");
                label.classList.add("handle-label");
                label.setAttribute(
                    "fill",
                    handle.label_color ?? handle.stroke ?? "#111827",
                );
                label.setAttribute("text-anchor", "middle");
                label.textContent = handle.label;
                group.appendChild(label);
            }

            overlay.appendChild(group);
            positionHandleGroup(group, handle);
        }
    };

    const relayoutHandler = () => {
        // Background Plotly pan/zoom changes axis transforms; keep handles
        // attached to their data coordinates.
        requestAnimationFrame(drawHandles);
    };

    const afterPlotHandler = () => {
        requestAnimationFrame(drawHandles);
    };

    plot.__robotRelayoutHandler = relayoutHandler;
    plot.__robotAfterPlotHandler = afterPlotHandler;

    Plotly.react(
        plot,
        figure.data,
        figure.layout,
        config,
    ).then(() => {
        if (cancelled) {
            return;
        }

        if (plot.on) {
            plot.on("plotly_relayout", relayoutHandler);
            plot.on("plotly_afterplot", afterPlotHandler);
        }

        drawHandles();

        // Observe WIDTH only. Height is fixed, so this cannot create the old
        // vertical-resize feedback loop.
        if (typeof ResizeObserver !== "undefined") {
            let previousWidth = stage.getBoundingClientRect().width;

            const observer = new ResizeObserver(() => {
                const nextWidth = stage.getBoundingClientRect().width;

                if (
                    cancelled ||
                    Math.abs(nextWidth - previousWidth) < 1
                ) {
                    return;
                }

                previousWidth = nextWidth;

                requestAnimationFrame(() => {
                    try {
                        Plotly.Plots.resize(plot);
                    } catch (_) {
                        // Safe during component teardown.
                    }

                    requestAnimationFrame(drawHandles);
                });
            });

            observer.observe(stage);
            stage.__robotResizeObserver = observer;
        }
    });

    return () => {
        cancelled = true;
        activeDrag = null;

        if (plot.__robotRelayoutHandler && plot.removeListener) {
            plot.removeListener(
                "plotly_relayout",
                plot.__robotRelayoutHandler,
            );
        }

        if (plot.__robotAfterPlotHandler && plot.removeListener) {
            plot.removeListener(
                "plotly_afterplot",
                plot.__robotAfterPlotHandler,
            );
        }

        if (stage.__robotResizeObserver) {
            stage.__robotResizeObserver.disconnect();
            stage.__robotResizeObserver = null;
        }
    };
}
"""


_draggable_plot = st.components.v2.component(
    "robot_motion_overlay_plot",
    html=_COMPONENT_HTML,
    css=_COMPONENT_CSS,
    js=_COMPONENT_JS,
)


# ---------------------------------------------------------------------------
# Python helpers
# ---------------------------------------------------------------------------

def _figure_json(figure: go.Figure) -> dict:
    """Convert a Plotly figure into JSON-compatible Python data."""
    return json.loads(figure.to_json())


def _remove_workspace_obstacle_shapes(payload: dict) -> None:
    """Remove static Plotly obstacle circles from the interactive workspace.

    The SVG handles will render those same obstacles on top of the Plotly graph,
    so leaving the layout shapes in place would produce duplicates.
    """
    layout = payload.setdefault("layout", {})
    layout["shapes"] = []

    # Hide the tiny obstacle-center marker traces; the SVG obstacle circles are
    # both the visible objects and the drag targets.
    for trace in payload.get("data", []):
        if str(trace.get("name", "")).startswith("Obstacle "):
            trace["visible"] = False


def _hide_cspace_static_marker(payload: dict) -> None:
    """Hide Plotly's static current-configuration marker."""
    for trace in payload.get("data", []):
        if trace.get("name") == "Current configuration":
            trace["visible"] = False


# ---------------------------------------------------------------------------
# Workspace
# ---------------------------------------------------------------------------

def render_draggable_workspace(
    figure: go.Figure,
    pose: RobotPose,
    obstacles: Iterable[Iterable[float]],
    *,
    key: str,
):
    """Render the workspace with objects directly overlaid on the Plotly axes."""
    obstacle_array = np.asarray(tuple(obstacles), dtype=float)
    payload = _figure_json(figure)

    _remove_workspace_obstacle_shapes(payload)

    # Do not preserve stale browser layout state after Python reruns.
    payload.setdefault("layout", {}).pop("uirevision", None)

    handles = [
        {
            "kind": "obstacle1",
            "x": float(obstacle_array[0, 0]),
            "y": float(obstacle_array[0, 1]),
            "radius_data": float(DEFAULT_GEOMETRY.obstacle_radius),
            "fill": "rgba(249, 115, 22, 0.45)",
            "stroke": "#c2410c",
            "stroke_width": 3,
            "label": "O1",
            "label_color": "#9a3412",
            "title": "Drag obstacle 1",
            "x_bounds": [-2.0, 2.0],
            "y_bounds": [-2.0, 2.0],
        },
        {
            "kind": "obstacle2",
            "x": float(obstacle_array[1, 0]),
            "y": float(obstacle_array[1, 1]),
            "radius_data": float(DEFAULT_GEOMETRY.obstacle_radius),
            "fill": "rgba(249, 115, 22, 0.45)",
            "stroke": "#c2410c",
            "stroke_width": 3,
            "label": "O2",
            "label_color": "#9a3412",
            "title": "Drag obstacle 2",
            "x_bounds": [-2.0, 2.0],
            "y_bounds": [-2.0, 2.0],
        },
        {
            "kind": "elbow",
            "x": float(pose.joint[0]),
            "y": float(pose.joint[1]),
            "radius_px": 10,
            "fill": "rgba(37, 99, 235, 0.16)",
            "stroke": "#2563eb",
            "stroke_width": 3,
            "title": "Drag elbow to change theta 1",
        },
        {
            "kind": "end_effector",
            "x": float(pose.end_effector[0]),
            "y": float(pose.end_effector[1]),
            "radius_px": 11,
            "fill": "rgba(124, 58, 237, 0.16)",
            "stroke": "#7c3aed",
            "stroke_width": 3,
            "title": "Drag end effector",
        },
    ]

    return _draggable_plot(
        data={
            "figure": payload,
            "handles": handles,
            "hint": (
                "Drag either orange obstacle, the blue elbow ring, or the "
                "purple end-effector ring. Plotly pan/zoom remains available "
                "when you drag the background or use the mode bar."
            ),
            "config": {
                "displayModeBar": True,
            },
        },
        default={"event": None},
        key=key,
        on_event_change=lambda: None,
        width="stretch",
        height=545,
    )


# ---------------------------------------------------------------------------
# Configuration space
# ---------------------------------------------------------------------------

def render_draggable_configuration_space(
    figure: go.Figure,
    theta1: float,
    theta2: float,
    *,
    key: str,
):
    """Render C-space with a red handle directly over the heatmap."""
    payload = _figure_json(figure)

    _hide_cspace_static_marker(payload)
    payload.setdefault("layout", {}).pop("uirevision", None)

    handles = [
        {
            "kind": "configuration",
            "x": float(theta1),
            "y": float(theta2),
            "radius_px": 10,
            "fill": "rgba(220, 38, 38, 0.18)",
            "stroke": "#dc2626",
            "stroke_width": 3,
            "title": "Drag current configuration",
            "x_bounds": [0.0, float(TWO_PI)],
            "y_bounds": [0.0, float(TWO_PI)],
        }
    ]

    return _draggable_plot(
        data={
            "figure": payload,
            "handles": handles,
            "hint": (
                "Drag the red marker to change θ₁ and θ₂ directly. "
                "Plotly pan/zoom remains available elsewhere in the plot."
            ),
            "config": {
                "displayModeBar": True,
            },
        },
        default={"event": None},
        key=key,
        on_event_change=lambda: None,
        width="stretch",
        height=545,
    )
