"""Footer for the Robot Motion with Obstacles application."""

from __future__ import annotations

import streamlit as st


SOURCE_URL = "https://demonstrations.wolfram.com/RobotMotionWithObstacles/"


def render_footer() -> None:
    """Render application and source attribution."""
    st.markdown(
        f"""
        <div class="robot-footer">
            <strong>Robot Motion with Obstacles</strong><br>
            Interactive Python / Streamlit robotics visualization<br>
            Based on
            <a
                href="{SOURCE_URL}"
                target="_blank"
                rel="noopener noreferrer"
            ><em>Robot Motion with Obstacles</em></a>
            by Aaron T. Becker and Haoran Zhao,
            Wolfram Demonstrations Project
        </div>
        """,
        unsafe_allow_html=True,
    )
