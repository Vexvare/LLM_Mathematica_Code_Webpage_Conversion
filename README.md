# Robot Motion with Obstacles

An interactive **Python / Streamlit** visualization of a two-link planar robot moving among circular obstacles.

The application connects the robot's Cartesian workspace with its two-dimensional **configuration space (C-space)**, making it possible to explore robot motion, obstacle collisions, forward kinematics, and inverse kinematics interactively.

## Features

- Two-link planar robot arm
- Forward kinematics
- Inverse kinematics for end-effector dragging
- Two draggable circular obstacles
- Link-to-obstacle collision detection
- Interactive configuration-space collision map
- Draggable C-space crosshair for direct robot control
- Draggable elbow and end-effector controls
- Exact sliders for joint angles and obstacle positions
- Adjustable configuration-space quality
- Numerical robot-state and collision diagnostics
- Responsive browser-side interaction for smooth dragging
- Streamlit navigation and user interface

## Interactive Performance

The workspace and configuration-space views share one browser-side interaction component.

During interaction:

- Moving the **C-space crosshair** updates the robot immediately in the browser.
- Dragging the **elbow** or **end effector** updates the robot continuously.
- Dragging an **obstacle** recomputes a lower-resolution `8Q × 8Q` C-space preview.
- When an obstacle drag ends, the final map is recomputed at `32Q × 32Q`.
- The committed state is then synchronized back to Streamlit/Python.

This keeps direct manipulation responsive while retaining the Python robot model for application state, diagnostics, and independent calculations.

## Project Structure

```text
robot-motion-with-obstacles/
├── app.py
├── requirements.txt
├── README.md
│
├── ui/
│   ├── home_page.py
│   ├── help_page.py
│   ├── demonstration_page.py
│   ├── robot_model.py
│   ├── interactive_plot.py
│   ├── visualization.py
│   ├── footer.py
│   └── welcome_lottie.json
│
└── tests/
    └── test_robot_model.py
```

`visualization.py` contains standalone Plotly visualization utilities. The main live demonstration uses the browser-side canvas interaction implemented in `interactive_plot.py`.

## Requirements

- Python 3.13 recommended for the tested environment
- pip
- A modern web browser

The project has been tested locally with **Python 3.13.3**.

The application dependencies are listed in `requirements.txt`.

The custom Streamlit component requires:

```text
Streamlit >= 1.51 and < 2
```

For deployment, use **Python 3.13** in Streamlit Community Cloud to match the
tested local Python 3.13.3 environment as closely as the deployment interface allows.

You can confirm your local version with:

```bash
python --version
```

## Local Installation

### 1. Clone the repository

```bash
git clone https://github.com/Vexvare/LLM_Mathematica_Code_Webpage_Conversion.git
cd LLM_Mathematica_Code_Webpage_Conversion
```

### 2. Create a virtual environment

A virtual environment is recommended.

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

### 3. Install application dependencies

```bash
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

### 4. Run the application

From the repository root:

```bash
python -m streamlit run app.py
```

Streamlit will normally open the application automatically.

If it does not, open the local address shown in the terminal, usually:

```text
http://localhost:8501
```

Using `python -m streamlit` ensures Streamlit runs from the same Python environment in which the dependencies were installed.

## Using the Demonstration

Open **Demonstration** from the sidebar.

### Workspace

The workspace shows the robot in Cartesian coordinates.

You can:

- Drag **Obstacle 1**
- Drag **Obstacle 2**
- Drag the **blue elbow handle** to change `θ₁`
- Drag the **purple end-effector handle** to reposition the robot using inverse kinematics

### Configuration Space

The C-space view maps joint-angle pairs `(θ₁, θ₂)` to collision state.

- Dark cells represent sampled collision configurations.
- Light cells represent sampled collision-free configurations.
- The red crosshair represents the current robot configuration.
- Click or drag the crosshair to move the robot directly.

### Precise Controls

The sliders beneath the visualization can be used when exact, repeatable values are preferred.

The interface uses degrees for readability while the internal robot model performs its angular calculations in radians.

## Configuration-Space Quality

The `Q` control changes the C-space sampling resolution.

While an obstacle is being dragged, the live preview uses:

```text
resolution = 8 × Q
```

After the drag is committed, the final map uses:

```text
resolution = 32 × Q
```

Examples of final resolution:

| Q | Final grid |
|---:|---:|
| 1 | 32 × 32 |
| 2 | 64 × 64 |
| 5 | 160 × 160 |
| 10 | 320 × 320 |

Higher values produce a finer sampled collision map but require more computation.

## Mathematical Model

The robot is a planar two-revolute-joint manipulator with equal link lengths.

For link length `L = 1`:

```text
x1 = L cos(θ1)
y1 = L sin(θ1)

x2 = x1 + L cos(θ1 + θ2)
y2 = y1 + L sin(θ1 + θ2)
```

The model also implements:

- periodic joint-angle handling
- point-to-line-segment distance
- robot-link collision detection
- sampled C-space generation
- two-branch planar inverse kinematics
- nearest-branch selection for smooth end-effector dragging

The mathematical model is kept in:

```text
ui/robot_model.py
```

and does not depend on Streamlit or browser rendering.

## Architecture

The application separates the robotics model from the user interface and browser interaction layer:

```text
Streamlit application
        │
        ├── page layout / controls / session state
        │
        ├── Python robot model
        │
        └── browser interaction component
                    │
                    ├── workspace rendering
                    ├── C-space rendering
                    ├── direct dragging
                    ├── live inverse kinematics
                    └── live C-space preview
```

### `robot_model.py`

Contains the Python robotics model:

- forward kinematics
- inverse kinematics
- collision detection
- angle wrapping
- configuration-space generation

### `interactive_plot.py`

Contains the high-speed browser interaction layer.

The workspace and C-space are rendered together so both views can update during a pointer drag without waiting for a Streamlit rerun.

### `demonstration_page.py`

Coordinates Streamlit state, exact controls, the interactive component, and numerical diagnostics.

## Tests

If you install the development dependencies:

```bash
python -m pip install -r requirements-dev.txt
```

run the automated tests with:

```bash
python -m pytest
```

Useful manual checks include:

1. Move the C-space crosshair and verify the robot follows continuously.
2. Drag each obstacle and verify the C-space preview updates during motion.
3. Release an obstacle and verify the final higher-resolution C-space appears.
4. Place an obstacle on a robot link and verify collision detection.
5. Drag the end effector around the reachable workspace.
6. Compare slider-controlled and drag-controlled values.
7. Test periodic joint behavior near `0°` and `360°`.

## Deploying on Streamlit Community Cloud

This repository is structured for deployment directly from GitHub.

Before deploying:

1. Make sure `app.py` and `requirements.txt` are in the repository root.
2. Push the latest working version to the `main` branch of:

   ```text
   https://github.com/Vexvare/LLM_Mathematica_Code_Webpage_Conversion
   ```

3. Confirm the application runs locally with:

   ```bash
   python -m streamlit run app.py
   ```

4. Note your local Python version:

   ```bash
   python --version
   ```

Then deploy through [Streamlit Community Cloud](https://share.streamlit.io/):

1. Sign in and connect your GitHub account.
2. Select **Create app**.
3. Choose the GitHub repository.
4. Select the branch you want to deploy, normally `main`.
5. Set the entrypoint to:

   ```text
   app.py
   ```

6. Choose a memorable `streamlit.app` subdomain if desired.
7. Open **Advanced settings** and select **Python 3.13**.
8. Deploy the application.

Community Cloud installs the dependencies from the repository's `requirements.txt` and launches the Streamlit entrypoint.

After deployment, add the live Streamlit URL near the top of this README.

## Source Demonstration

This application is based on:

**Robot Motion with Obstacles**  
by **Aaron T. Becker** and **Haoran Zhao**  
Wolfram Demonstrations Project

Original demonstration:

https://demonstrations.wolfram.com/RobotMotionWithObstacles/

The original work remains attributable to its authors and the Wolfram Demonstrations Project.

## Troubleshooting

### Streamlit is installed but the command is not found

Use:

```bash
python -m streamlit --version
python -m streamlit run app.py
```

### The application does not reflect recent code changes

Stop Streamlit with:

```text
Ctrl+C
```

then restart it:

```bash
python -m streamlit run app.py
```

For a deployed app, verify that your changes were committed and pushed to the branch used by Community Cloud.

### A Community Cloud build fails

Check the deployment logs first.

Common causes include:

- a missing package in `requirements.txt`
- a Python-version mismatch between local development and deployment
- filename or path capitalization that works on Windows but fails on Linux
- a required local asset that was not committed to GitHub

### C-space interaction is slow

Lower `Q`.

`Q = 2` gives a final `64 × 64` grid and is a good interactive default.

## License

The Python/Streamlit implementation in this repository is licensed under the **MIT License**.

The original **Robot Motion with Obstacles** demonstration is credited separately above and remains subject to the terms applicable to the original Wolfram Demonstrations Project material.
