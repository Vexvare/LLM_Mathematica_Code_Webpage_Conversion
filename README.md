# Robot Motion with Obstacles

An interactive **Python / Streamlit** recreation of the Wolfram Demonstrations Project example **Robot Motion with Obstacles**.

This project was created for a robotics course assignment focused on converting an existing Mathematica demonstration into a web application with the assistance of an AI coding tool. The goal is not only to reproduce the original behavior, but also to make the conversion easy to test, inspect, and evaluate.

## Features

- Two-link planar robot arm
- Forward kinematics
- Two circular obstacles
- Robot/obstacle collision detection
- Sampled configuration-space (C-space) collision map
- Adjustable configuration-space quality
- Direct manipulation of:
  - Obstacle 1
  - Obstacle 2
  - Robot elbow
  - Robot end effector
  - Current C-space configuration
- Inverse kinematics for end-effector dragging
- Exact slider controls for repeatable testing
- Workspace and C-space visualizations rendered with Plotly
- Streamlit-based interface with sidebar navigation
- Lottie animation support
- Numerical diagnostics for validating the conversion

## Project Structure

```text
robot-motion-streamlit/
│
├── app.py
├── requirements.txt
│
├── ui/
│   ├── home_page.py
│   ├── help_page.py
│   ├── demonstration_page.py
│   ├── robot_model.py
│   ├── visualization.py
│   ├── interactive_plot.py
│   ├── footer.py
│   └── welcome_lottie.json
│
├── tests/
│   └── test_robot_model.py
│
└── reference/
    └── Demonstration-Robot-Motion-with-Obstacles-1-0-0-definition.nb
```

## Requirements

- Python 3.10 or newer recommended
- pip
- A modern web browser

The project dependencies are listed in `requirements.txt`.

The interactive Plotly component requires **Streamlit 1.51 or newer**.

## Installation

### 1. Clone the repository

```bash
git clone <YOUR_REPOSITORY_URL>
cd robot-motion-streamlit
```

Replace `<YOUR_REPOSITORY_URL>` with the URL of your GitHub repository.

### 2. Create a virtual environment

Creating a virtual environment is strongly recommended.

#### Windows PowerShell

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
```

#### Windows Command Prompt

```cmd
python -m venv .venv
.venv\Scripts\activate
```

#### macOS / Linux

```bash
python3 -m venv .venv
source .venv/bin/activate
```

### 3. Install dependencies

```bash
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Using `python -m pip` helps ensure the packages are installed into the same Python environment that will run the application.

## Running the Application

From the repository root:

```bash
python -m streamlit run app.py
```

Streamlit should open the application automatically in your browser.

If it does not, the terminal will display a local URL, usually similar to:

```text
http://localhost:8501
```

Open that address manually in your browser.

> `python -m streamlit run app.py` is recommended instead of `streamlit run app.py` because it avoids problems caused by stale or mismatched Streamlit launchers on the system PATH.

## Using the Demonstration

Open **Demonstration** from the sidebar.

### Robot controls

The robot has two revolute joints:

- `θ1` — first joint angle
- `θ2` — second joint angle

The sliders use degrees for easier interaction, while the internal model performs the calculations in radians.

### Direct manipulation

The workspace supports direct interaction:

- Drag **Obstacle 1**
- Drag **Obstacle 2**
- Drag the **elbow handle** to change `θ1`
- Drag the **end-effector handle** to reposition the arm using inverse kinematics

The configuration-space visualization also includes a draggable marker that changes `θ1` and `θ2`.

The sliders remain available for exact and repeatable test values.

### Configuration-space quality

The `Q` control changes the sampling resolution of the collision map.

The high-resolution configuration-space grid uses:

```text
resolution = 32 × Q
```

samples along each angular axis.

For example:

| Q | Grid size |
|---:|---:|
| 1 | 32 × 32 |
| 2 | 64 × 64 |
| 5 | 160 × 160 |
| 10 | 320 × 320 |

Higher values provide a finer collision map but require more computation.

## Mathematical Model

The robot is a two-link planar manipulator.

For link length `L = 1`:

```text
x1 = L cos(θ1)
y1 = L sin(θ1)

x2 = x1 + L cos(θ1 + θ2)
y2 = y1 + L sin(θ1 + θ2)
```

The model also includes:

- point-to-line-segment distance
- link/obstacle collision detection
- periodic joint-angle handling
- inverse kinematics
- sampled C-space generation

The robot model is intentionally separated from the Streamlit interface in:

```text
ui/robot_model.py
```

This makes the mathematics easier to test independently from the web interface.

## Visualization Architecture

The project separates the application into three major layers:

```text
Streamlit interface
        ↓
robot_model.py
        ↓
visualization.py / interactive_plot.py
```

### `robot_model.py`

Contains the robotics mathematics:

- forward kinematics
- inverse kinematics
- collision detection
- angle wrapping
- configuration-space generation

### `visualization.py`

Builds the Plotly figures:

- robot workspace
- configuration-space heatmap

### `interactive_plot.py`

Adds direct drag interaction over the Plotly figures.

The draggable interaction layer is synchronized with Plotly's coordinate system so that the draggable objects remain attached to the correct robot/workspace coordinates while the page is resized.

## Testing

The assignment requires evaluating both what the AI-generated conversion gets **right** and what it gets **wrong**.

Recommended tests include:

1. Compare the default robot pose against the original Mathematica demonstration.
2. Test `θ1` and `θ2` at:
   - 0°
   - 90°
   - 180°
   - 270°
   - 360°
3. Move each obstacle independently.
4. Place an obstacle directly on a robot link and verify that collision is detected.
5. Move an obstacle away from the arm and verify that the collision clears.
6. Compare the C-space pattern against the Mathematica source.
7. Test several values of `Q`.
8. Test periodic behavior near 0° / 360°.
9. Compare slider-controlled and drag-controlled motion.
10. Record both successful behavior and faults.

If automated tests are included, run:

```bash
python -m pytest
```

## Known Conversion Differences

The Streamlit version is designed to reproduce the source demonstration closely, but it is not a byte-for-byte or UI-for-UI translation of Mathematica.

Important differences include:

- Mathematica uses native `Locator` controls.
- This project recreates direct manipulation through a custom Streamlit interaction layer.
- Drag changes are synchronized back to Python after the interaction event.
- Streamlit and Plotly use a web-based rendering model rather than Mathematica's notebook graphics system.
- The original Mathematica code uses a different second-link sign convention in part of the C-space calculation than in the workspace visualization. The project preserves a source-compatible option so that this behavior can be tested rather than silently changed.
- C-space is sampled on a finite grid, so collision boundaries depend visually on `Q`.

These differences are useful observations for the evaluation portion of the assignment.

## Source Demonstration

This project is based on:

**Robot Motion with Obstacles**  
Wolfram Demonstrations Project  
Aaron T. Becker and Haoran Zhao

Reference page:

```text
https://demonstrations.wolfram.com/RobotMotionWithObstacles/
```

The original Mathematica notebook is included only as reference material for the conversion and testing process.

## Attribution and Project Scope

This repository contains a Python/Streamlit implementation created specifically for the robotics conversion assignment.

The application structure, Python implementation, Streamlit interface, interaction layer, and testing utilities are separate from the original Mathematica notebook.

When distributing or publishing the project, retain appropriate attribution to the original Wolfram Demonstrations Project example and follow the licensing terms that apply to the original source material.

## Troubleshooting

### `streamlit` command fails but Streamlit is installed

Check the installed version:

```bash
python -m streamlit --version
```

Then launch with:

```bash
python -m streamlit run app.py
```

If necessary, reinstall dependencies:

```bash
python -m pip install -r requirements.txt
```

### Interactive plots do not appear

Verify that Streamlit is current enough:

```bash
python -m streamlit --version
```

The project requires Streamlit 1.51 or newer for the custom interactive component.

Then restart Streamlit completely.

### Changes to Python files do not appear

Stop the server:

```text
Ctrl+C
```

and restart it:

```bash
python -m streamlit run app.py
```

### Configuration-space updates are slow

Lower `Q`.

A value of:

```text
Q = 2
```

produces a `64 × 64` grid and is a good default for interactive use.

## Development Notes

When modifying the project, keep responsibilities separated:

- robotics math → `robot_model.py`
- static Plotly construction → `visualization.py`
- direct browser interaction → `interactive_plot.py`
- Streamlit page layout/state → `demonstration_page.py`
- top-level routing → `app.py`

This separation makes debugging and comparison with the Mathematica source substantially easier.

## License

Add the license selected for this repository here.

If the repository includes or redistributes source material derived from the Wolfram demonstration, make sure the repository's licensing and attribution remain compatible with the terms of the original work.
