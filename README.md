# 3R Manipulator — Digital Twin and Writing Research Platform

A research workspace for a planar 3R manipulator, targeting reproducible CAD-derived digital-twin modelling, MuJoCo simulation, ROS 2 control, and trajectory learning for writing/drawing.

> **Model assumptions in the starter:** planar revolute joints, nominal link lengths 0.18 m, 0.12 m, and 0.07 m. These are provisional dimensions from the research notes, not measurements extracted from CAD. Replace them after CAD inspection and calibration.

## Architecture

- `mujoco/`: standalone MuJoCo XML model and Python simulation entry point.
- `ros2_ws/src/three_r_description/`: URDF/Xacro robot description and RViz launch.
- `ros2_ws/src/three_r_control/`: ROS 2 trajectory interface starter.
- `scripts/`: CAD mesh conversion and validation utilities.
- `training/`: dataset and behavior-cloning baseline scaffold.
- `web/`: static dashboard, deployable to GitHub Pages.
- `.github/workflows/pages.yml`: build/deploy the static dashboard to GitHub Pages.

## CAD workflow

1. Export each rigid link and bracket from CAD as an individual STL or OBJ (millimetres preferred; preserve the source assembly and joint coordinate system).
2. Put the exported files in `assets/cad/` and edit `config/robot.yaml` to map mesh filenames, scale, mass, centre of mass, and inertial tensor.
3. Run `python scripts/validate_meshes.py --input assets/cad --output assets/meshes`.
4. Inspect mesh orientation and scale before using meshes in simulation. Do not infer joint axes from mesh geometry alone; use the CAD assembly mates/origins.

Supported inputs in the conversion helper: STEP/STP (requires FreeCAD Python module or FreeCAD CLI), STL, OBJ, PLY. STEP conversion is intentionally delegated to FreeCAD; no geometry is fabricated when FreeCAD is unavailable.

## MuJoCo

Install Python dependencies and run:

```bash
python -m venv .venv
# Windows: .venv\\Scripts\\activate
# Linux/macOS: source .venv/bin/activate
pip install -r requirements.txt
python mujoco/run_simulation.py
```

The starter XML uses primitive visual/collision geometry until the real CAD meshes are supplied and validated.

## ROS 2

On a ROS 2 installation (Jazzy recommended):

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch three_r_description view_robot.launch.py
```

The URDF is a provisional planar chain. Update it from CAD-derived joint frames, then verify in RViz and with joint-state/TF inspection before commanding hardware.

## Web dashboard

The web dashboard is a static project status/control-design interface. It does not run ROS 2 or MuJoCo in the browser. For live interaction later, run a local ROS bridge/API on the workstation and add an authenticated WebSocket connection; never expose robot control endpoints publicly without authentication, limits, and an emergency-stop path.

GitHub Pages can host the static dashboard from the repository's **Settings → Pages → GitHub Actions**. The workflow deploys on pushes to `main`.

## Safety and validation

- Simulation-only until joint directions, limits, collision geometry, masses, inertias, actuator ranges, and calibration are verified.
- Keep physical hardware disabled during initial model validation.
- Validate in stages: static TF/URDF → joint sweeps → forward kinematics → inverse kinematics → trajectory tracking → obstacle logic → writing trajectories.
- Training data must include workspace/joint-limit filtering and train/validation/test separation by trajectory or scene, not random point split alone.

## Current status

This is the starter project foundation. CAD conversion, joint-frame alignment, exact inertial properties, and measured actuator calibration remain pending receipt and inspection of the CAD assembly.
