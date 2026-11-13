# 3R Manipulator — Digital Twin and Writing Research Platform

Research workspace for the planar 3R manipulator, covering CAD-derived geometry, MuJoCo dynamics, ROS 2 description/control, and learning-based trajectory generation for handwriting and drawing.

## CAD inspection — 13 November 2026

Five supplied component meshes were inspected: `Base_Ring.stl`, `Base_Lid.stl`, `Link1.stl`, `Link2.stl`, and `Link3_PenHolder.stl`. Source STL coordinates are millimetres. Approximate pivot-hole centre spacings measured from the link meshes are **120 mm, 100 mm, and 60 mm**.

Download the accompanying `3R_Manipulator_CAD_Assets.zip` project attachment and extract it at the repository root. It populates both `assets/meshes/` for MuJoCo and `ros2_ws/src/three_r_description/meshes/` for ROS 2.

**Important:** separate STL exports do not contain original assembly mate transforms, motor shaft datums, or tool insertion depth. The base joint height and tool-tip offset in the starter models are provisional. Link masses and inertias use an assumed uniform density of 1,200 kg/m³ because STL contains no material-density metadata. Verify these values against the native CAD assembly and physical measurements before drawing conclusions about dynamics or enabling hardware.

See `docs/CAD_INSPECTION.md` for measured mesh bounds, volumes, and assumptions.

## Architecture

- `mujoco/`: CAD-mesh MJCF model, primitive fallback model, and Python simulation smoke test.
- `ros2_ws/src/three_r_description/`: CAD mesh URDF/Xacro description and RViz launch.
- `ros2_ws/src/three_r_control/`: ROS 2 trajectory-message demo scaffold (not yet connected to a hardware controller).
- `kinematics/`: planar 3R forward kinematics, Jacobian, and DLS inverse kinematics.
- `tests/`: automated kinematics checks.
- `scripts/`: CAD mesh conversion and validation utilities.
- `training/`: geometric trajectory dataset generator and future learning pipeline.
- `web/`: static research dashboard.
- `.github/workflows/`: GitHub Pages deployment and CI test workflows.

## Python setup and tests

```powershell
python -m venv .venv
.venv\\Scripts\\activate
pip install -r requirements.txt
python -m pytest -q
```

The kinematics tests include a finite-difference Jacobian check, reachable-target DLS IK, and sequential path continuity.

## MuJoCo

After extracting the CAD asset archive at the repository root:

```powershell
python mujoco/run_simulation.py --no-viewer
python mujoco/run_simulation.py
```

The CAD-mesh model is the default. To run the mesh-free primitive fallback:

```powershell
python mujoco/run_simulation.py --primitive
```

The CAD model uses mesh visuals plus simplified collision capsules. These collision capsules are initial approximations; validate their coverage and clearance before obstacle experiments.

## ROS 2

On a ROS 2 installation (Jazzy recommended), extract the asset archive at the repository root, then:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch three_r_description view_robot.launch.py
```

The launch file opens a preconfigured RViz view. Check the TF tree, joint axes, limits, base height, and tool point before connecting any controller. The `three_r_control` package is a demo trajectory publisher, not a final `ros2_control` hardware interface.

## Web hosting

The dashboard is a static Vite site. It does not run MuJoCo, ROS 2, or hardware control in the browser.

**GitHub Pages:** open repository **Settings → Pages**, select **GitHub Actions** as the build/deploy source, and run the `Deploy static dashboard to GitHub Pages` workflow (or push a change under `web/`). The workflow builds `web/` and publishes `web/dist`. The workflow is committed, but hosting is not yet confirmed live.

**Vercel:** import this GitHub repository into Vercel and set the project Root Directory to `web`, Build Command to `npm run build`, and Output Directory to `dist`. No Vercel deployment is claimed from this workspace.

For live monitoring later, run a secured local ROS bridge/API on the workstation and connect the hosted dashboard via authenticated WebSocket. Do not expose robot-control endpoints publicly without authentication, command limits, and an emergency-stop path.

## Training roadmap

1. Validate CAD assembly frames, mesh units, joint directions/limits, base height, and pen-tip transform.
2. Validate MuJoCo/URDF forward kinematics against the independent analytical implementation in `kinematics/`.
3. Implement time-parameterized writing paths and joint-space tracking baselines.
4. Generate labelled datasets with workspace/joint-limit filtering and trajectory-level train/validation/test splits.
5. Compare analytical/numerical IK, ANN, and safety-filtered hybrid approaches.
6. Evaluate Cartesian tracking error, joint smoothness/jerk, limit violations, collision clearance, completion rate, and generalization to unseen letters/shapes.

A starter dataset can be generated with:

```bash
python training/generate_demo_dataset.py --output data/demo/geometric_dls.csv
```

It creates synthetic circle and square examples with joint positions, velocities, accelerations, and IK residuals. It is software-generated data, not measured hardware data.

## Safety and evidence status

This repository contains CAD-derived descriptions and starter algorithms. A successful MuJoCo runtime test, ROS 2 runtime test, full assembly-frame validation, or physical-hardware experiment has **not** been completed in this environment. Keep hardware disabled until model geometry, joint conventions, actuator ranges, calibration, collision checks, and emergency-stop behavior are verified.
