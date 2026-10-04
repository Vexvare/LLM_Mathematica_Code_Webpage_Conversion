"""Footer for the Robot Motion with Obstacles Streamlit project."""

from __future__ import annotations

import streamlit as st


def render_footer() -> None:
    """Render lightweight project/source attribution at the bottom of the app."""
    st.markdown(
        """
        <div class="robot-footer">
            <strong>Robot Motion with Obstacles</strong><br>
            Python / Streamlit conversion for robotics coursework<br>
            Based on the Wolfram Demonstrations Project example
            <em>Robot Motion with Obstacles</em> by Aaron T. Becker and Haoran Zhao
        </div>
        """,
        unsafe_allow_html=True,
    )
