"""Main entry point for the Robot Motion with Obstacles Streamlit application."""

from __future__ import annotations

import json
from pathlib import Path

import streamlit as st
from streamlit_lottie import st_lottie
from streamlit_option_menu import option_menu

from ui.demonstration_page import display_demonstration_page
from ui.footer import render_footer
from ui.help_page import display_help_page
from ui.home_page import display_home_page


APP_TITLE = "Robot Motion with Obstacles"
PAGES = ("Home", "Demonstration", "Help")

PAGE_ICONS = {
    "Home": "house",
    "Demonstration": "robot",
    "Help": "question-circle",
}

APP_ROOT = Path(__file__).resolve().parent
LOTTIE_PATH = APP_ROOT / "ui" / "welcome_lottie.json"


st.set_page_config(
    page_title=APP_TITLE,
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded",
)


def _inject_global_styles() -> None:
    """Apply styling shared by all pages."""
    st.markdown(
        """
        <style>
            .block-container {
                padding-top: 1.8rem;
                padding-bottom: 2.5rem;
            }

            section[data-testid="stSidebar"] > div {
                padding-top: 1.2rem;
            }

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

            h1, h2, h3 {
                letter-spacing: -0.015em;
            }

            div[data-testid="stMetric"] {
                border: 1px solid rgba(128, 128, 128, 0.18);
                border-radius: 0.7rem;
                padding: 0.75rem 0.9rem;
            }

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


def _load_lottie_file(path: Path) -> dict | None:
    """Load a local Lottie animation if it is available."""
    try:
        with path.open("r", encoding="utf-8") as file:
            return json.load(file)
    except (OSError, json.JSONDecodeError):
        return None


def _render_sidebar_lottie() -> None:
    """Render the sidebar animation when available."""
    animation = _load_lottie_file(LOTTIE_PATH)

    if animation is not None:
        st_lottie(
            animation,
            height=135,
            key="robot_motion_sidebar_lottie",
        )


def navigate_to(page: str) -> None:
    """Navigate to one of the application's top-level pages."""
    if page not in PAGES:
        return

    st.session_state.current_page = page
    st.rerun()


def _initialize_navigation_state() -> None:
    """Ensure the active page stored in session state is valid."""
    if st.session_state.get("current_page") not in PAGES:
        st.session_state.current_page = "Home"


def _render_sidebar() -> str:
    """Render the application sidebar and return the selected page."""
    _initialize_navigation_state()

    with st.sidebar:
        _render_sidebar_lottie()

        st.markdown(
            """
            <div class="robot-brand">
                <h1>Robot Motion</h1>
                <h1>with Obstacles</h1>
                <p>Interactive planar robotics visualization</p>
            </div>
            """,
            unsafe_allow_html=True,
        )

        current_page = st.session_state.current_page

        selected_page = option_menu(
            menu_title=None,
            options=list(PAGES),
            icons=[PAGE_ICONS[page] for page in PAGES],
            default_index=PAGES.index(current_page),
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
        st.caption("Two-link robot motion and configuration-space exploration")

    return st.session_state.current_page


def _render_page(active_page: str) -> None:
    """Render the selected page in a centered content column."""
    _, page_content, _ = st.columns([0.45, 12.0, 0.45])

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
