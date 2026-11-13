# Trajectory learning scaffold

Target pipeline (to implement after CAD/kinematics validation):

1. Generate task-space writing paths (letters, digits, geometric primitives) as time-parameterized Cartesian trajectories.
2. Filter targets against the joint-limit-aware reachable set.
3. Generate IK labels using a trusted analytical/numerical solver; store joint states, Cartesian targets, velocities, accelerations, and success flags.
4. Split train/validation/test by complete trajectories, writing symbols, and obstacle scenes to prevent leakage.
5. Train a baseline MLP to map desired tool pose and context to joint targets; compare against damped least-squares IK and a safety-filtered hybrid.
6. Evaluate Cartesian tracking error, joint smoothness, jerk, limit violations, collision clearance, completion rate, and generalization to unseen symbols.

Do not train on placeholder geometry. Keep model version, random seed, dataset configuration, and robot calibration with every experiment.
