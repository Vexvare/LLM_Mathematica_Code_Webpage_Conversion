"""Main entry point for the Robot Motion with Obstacles Streamlit app.

This file owns only application-level presentation and navigation:
- page configuration
- sidebar branding
- Lottie animation
- option-menu navigation
- global styling
- centered page framing
- footer rendering

Robotics mathematics and visualization remain isolated in the ui package.
"""

from __future__ import annotations

import json
import os

import streamlit as st
from streamlit_lottie import st_lottie
from streamlit_option_menu import option_menu

from ui.demonstration_page import display_demonstration_page
from ui.footer import render_footer
from ui.help_page import display_help_page
from ui.home_page import display_home_page


# ---------------------------------------------------------------------------
# Streamlit page configuration
# ---------------------------------------------------------------------------

st.set_page_config(
    page_title="Robot Motion WIth Obstacles",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded",
)


PAGES = ("Home", "Demonstration", "Help")

PAGE_ICONS = {
    "Home": "house",
    "Demonstration": "robot",
    "Help": "question-circle",
}


# ---------------------------------------------------------------------------
# Styling
# ---------------------------------------------------------------------------

def _inject_global_styles() -> None:
    """Apply lightweight styling shared by all pages."""
    st.markdown(
        """
        <style>
            /* Main application spacing */
            .block-container {
                padding-top: 1.8rem;
                padding-bottom: 2.5rem;
            }

            /* Sidebar spacing */
            section[data-testid="stSidebar"] > div {
                padding-top: 1.2rem;
            }

            /* Sidebar project title */
            .robot-brand {
                text-align: center;
                margin-top: 0.1rem;
                margin-bottom: 0.9rem;
            }

            .robot-brand h1 {
                margin: 0;
                font-size: 1.65rem;
                line-height: 1.15;
                font-weight: 800;
            }

            .robot-brand p {
                margin-top: 0.35rem;
                margin-bottom: 0;
                font-size: 0.86rem;
                opacity: 0.70;
            }

            /* Slightly tighter headings */
            h1, h2, h3 {
                letter-spacing: -0.015em;
            }

            /* Improve metric-card separation without fighting Streamlit theme */
            div[data-testid="stMetric"] {
                border: 1px solid rgba(128, 128, 128, 0.18);
                border-radius: 0.7rem;
                padding: 0.75rem 0.9rem;
            }

            /* Keep footer visually separate */
            .robot-footer {
                margin-top: 2.5rem;
                padding-top: 1rem;
                border-top: 1px solid rgba(128, 128, 128, 0.22);
                text-align: center;
                font-size: 0.82rem;
                opacity: 0.72;
                line-height: 1.5;
            }
        </style>
        """,
        unsafe_allow_html=True,
    )


# ---------------------------------------------------------------------------
# Lottie
# ---------------------------------------------------------------------------

def _load_lottie_file(filepath: str) -> dict | None:
    """Load a local Lottie JSON file, returning None if it cannot be read."""
    try:
        with open(filepath, "r", encoding="utf-8") as file:
            return json.load(file)
    except (OSError, json.JSONDecodeError):
        return None


def _render_sidebar_lottie() -> None:
    """Render the placeholder sidebar animation when available."""
    lottie_path = os.path.join("ui", "welcome_lottie.json")
    animation = _load_lottie_file(lottie_path)

    if animation is not None:
        st_lottie(
            animation,
            height=135,
            key="robot_motion_sidebar_lottie",
        )


# ---------------------------------------------------------------------------
# Navigation
# ---------------------------------------------------------------------------

def navigate_to(page: str) -> None:
    """Navigate programmatically from page buttons."""
    if page not in PAGES:
        return

    st.session_state.current_page = page
    st.rerun()


def _initialize_navigation_state() -> None:
    """Guarantee a valid active page."""
    if "current_page" not in st.session_state:
        st.session_state.current_page = "Home"

    if st.session_state.current_page not in PAGES:
        st.session_state.current_page = "Home"


def _render_sidebar() -> str:
    """Render sidebar branding, animation, and option-menu navigation."""
    _initialize_navigation_state()

    with st.sidebar:
        _render_sidebar_lottie()

        st.markdown(
            """
            <div class="robot-brand">
                <h1>Robot Motion</h1>
                <h1>with Obstacles</h1>
                <p>Interactive robotics demonstration</p>
            </div>
            """,
            unsafe_allow_html=True,
        )

        current_page = st.session_state.current_page
        default_index = PAGES.index(current_page)

        selected_page = option_menu(
            menu_title=None,
            options=list(PAGES),
            icons=[PAGE_ICONS[page] for page in PAGES],
            default_index=default_index,
            styles={
                "container": {
                    "padding": "0.35rem 0rem",
                    "background-color": "transparent",
                },
                "icon": {
                    "font-size": "1.05rem",
                },
                "nav-link": {
                    "font-size": "0.95rem",
                    "text-align": "left",
                    "margin": "0.20rem 0rem",
                    "padding": "0.65rem 0.85rem",
                    "border-radius": "0.55rem",
                },
                "nav-link-selected": {
                    "background-color": "#2563eb",
                    "color": "white",
                    "font-weight": "600",
                },
            },
        )

        if selected_page != current_page:
            st.session_state.current_page = selected_page
            st.rerun()

        st.markdown("---")
        st.caption(
            "Mathematica → Python / Streamlit\n\n"
            "AI-assisted robotics conversion"
        )

    return st.session_state.current_page


# ---------------------------------------------------------------------------
# Main content routing
# ---------------------------------------------------------------------------

def _render_page(active_page: str) -> None:
    """Render the selected page inside a modest centered content frame."""
    left_margin, page_content, right_margin = st.columns([0.45, 12.0, 0.45])

    with page_content:
        if active_page == "Home":
            display_home_page(navigate=navigate_to)

        elif active_page == "Demonstration":
            display_demonstration_page()

        elif active_page == "Help":
            display_help_page(navigate=navigate_to)


def main() -> None:
    """Run the application."""
    _inject_global_styles()

    active_page = _render_sidebar()
    _render_page(active_page)
    render_footer()


if __name__ == "__main__":
    main()
