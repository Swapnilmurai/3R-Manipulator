# 3R Manipulator — Digital Twin and Writing Research Platform

Research workspace for the planar 3R manipulator, covering CAD-derived geometry, MuJoCo dynamics, ROS 2 description/control, and learning-based trajectory generation for handwriting and drawing.

## CAD inspection — 13 November 2026

Five supplied component meshes were inspected: `Base_Ring.stl`, `Base_Lid.stl`, `Link1.stl`, `Link2.stl`, and `Link3_PenHolder.stl`. The STL coordinates are millimetres. Approximate pivot-hole centre spacings measured from the link meshes are **120 mm, 100 mm, and 60 mm**.

The CAD asset archive is available as [3R_Manipulator_CAD_Assets.tar.xz](https://github.com/Swapnilmurai/3R-Manipulator/releases) when attached to a release, or from the CAD asset download shared with the project owner. Extract it at the repository root. It provides both `assets/meshes/` for MuJoCo and `ros2_ws/src/three_r_description/meshes/` for ROS 2.

**Important:** separate STL exports do not contain original assembly mate transforms, motor shaft datums, or tool insertion depth. The base joint height and tool-tip offset in the starter models are provisional. Link masses and inertias use an assumed uniform density of 1,200 kg/m³ because STL contains no material-density metadata. Verify all these values against the native CAD assembly and physical measurements before drawing conclusions about dynamics or enabling hardware.

See `docs/CAD_INSPECTION.md` for the measured mesh bounds, volumes, and assumptions.

## Architecture

- `mujoco/`: CAD-mesh MJCF model, primitive fallback model, and Python simulation smoke test.
- `ros2_ws/src/three_r_description/`: CAD mesh URDF/Xacro description and RViz launch.
- `ros2_ws/src/three_r_control/`: ROS 2 trajectory-message demo scaffold (not yet connected to a hardware controller).
- `scripts/`: CAD conversion and validation utilities.
- `training/`: planned dataset and learning pipeline.
- `web/`: static research dashboard.
- `.github/workflows/pages.yml`: GitHub Pages build/deploy workflow.

## MuJoCo

Install Python dependencies:

```powershell
python -m venv .venv
.venv\\Scripts\\activate
pip install -r requirements.txt
```

After extracting the CAD mesh archive at the repository root:

```powershell
python mujoco/run_simulation.py --no-viewer
python mujoco/run_simulation.py
```

To run the primitive fallback without CAD assets:

```powershell
python mujoco/run_simulation.py --primitive
```

The CAD model uses mesh visuals plus simplified collision capsules. Those capsules are initial collision approximations and must be reviewed for obstacle-clearance studies.

## ROS 2

On a ROS 2 installation (Jazzy recommended), extract the CAD asset archive at the repository root, then:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch three_r_description view_robot.launch.py
```

The URDF uses the CAD meshes and estimated link inertias. Check the TF tree, joint axes, limits, base height, and tool point in RViz before connecting any controller.

## Web hosting

The dashboard is a static Vite site. It does not run MuJoCo, ROS 2, or hardware control in the browser.

**GitHub Pages:** open repository **Settings → Pages**, select **GitHub Actions** as the build/deploy source, and run the `Deploy static dashboard to GitHub Pages` workflow (or push a change under `web/`). The workflow builds `web/` and publishes `web/dist`.

**Vercel:** import this GitHub repository into Vercel and set the project Root Directory to `web`, Build Command to `npm run build`, and Output Directory to `dist`. A Vercel connection is not yet configured from this workspace, so no Vercel deployment is claimed.

For live monitoring later, run a secured local ROS bridge/API on the workstation and connect the hosted dashboard via authenticated WebSocket. Do not expose robot-control endpoints publicly without authentication, command limits, and an emergency-stop path.

## Training roadmap

1. Validate CAD assembly frames, mesh units, joint directions/limits, base height, and pen-tip transform.
2. Validate MuJoCo/URDF forward kinematics against an independent analytical model.
3. Implement time-parameterized writing paths and joint-space tracking baselines.
4. Generate labelled datasets with workspace/joint-limit filtering and trajectory-level train/validation/test splits.
5. Compare analytical/numerical IK, ANN, and safety-filtered hybrid approaches.
6. Evaluate Cartesian tracking error, joint smoothness/jerk, limit violations, collision clearance, completion rate, and generalization to unseen letters/shapes.

## Safety and evidence status

This repository contains starter code and CAD-derived descriptions. A successful MuJoCo runtime test, ROS 2 runtime test, full assembly-frame validation, or physical-hardware experiment has **not** been performed in this environment. Keep hardware disabled until model geometry, joint conventions, actuator ranges, calibration, collision checks, and emergency-stop behavior are verified.
