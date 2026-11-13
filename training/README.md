# Trajectory generation and learning

## Working baseline

`generate_demo_dataset.py` currently generates two synthetic Cartesian drawing paths (circle and square) around a small reachable task-space region, then labels them with the sequential damped-least-squares IK implementation in `kinematics/planar_3r.py`.

Run from the repository root:

```bash
python training/generate_demo_dataset.py --output data/demo/geometric_dls.csv
```

CSV fields include Cartesian position/velocity, joint position/velocity/acceleration, pen state, IK residual, and IK iteration count. These are simulated synthetic labels, not physical-hardware measurements.

## Next steps

1. Add letters and digits as vector-stroke trajectories with explicit pen-up/pen-down states.
2. Filter targets against the joint-limit-aware reachable set.
3. Improve IK with adaptive damping, singularity metrics, orientation/posture objectives, and collision constraints.
4. Store robot calibration, mesh/model version, path source, random seed, and IK solver configuration with every dataset.
5. Split train/validation/test by complete trajectories, symbols, and obstacle scenes to prevent leakage.
6. Train an ANN baseline to map desired tool position, velocity, and task context to joint targets; compare against DLS and a safety-filtered hybrid.
7. Evaluate Cartesian tracking error, joint smoothness/jerk, limit violations, collision clearance, completion rate, and generalization to unseen symbols.

Do not treat this example dataset as a training corpus for hardware control. Validate the assembled digital twin and calibrate the real mechanism first.
