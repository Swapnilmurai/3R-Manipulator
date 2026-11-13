"""Generate a small geometric-path dataset using the DLS IK baseline.

From repository root:
  python training/generate_demo_dataset.py --output data/demo/geometric_dls.csv

This is a synthetic software example, not measured hardware data.
"""
from __future__ import annotations
import argparse
from pathlib import Path
import sys
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from kinematics.planar_3r import solve_path_dls


def make_paths(samples: int) -> dict[str, np.ndarray]:
    center = np.array([0.170, 0.070])
    radius = 0.012
    t = np.linspace(0.0, 2.0 * np.pi, samples, endpoint=True)
    circle = np.column_stack([center[0] + radius * np.cos(t), center[1] + radius * np.sin(t)])

    half = 0.010
    vertices = center + np.array([
        [-half, -half], [half, -half], [half, half], [-half, half], [-half, -half]
    ])
    square_points: list[np.ndarray] = []
    per_edge = max(2, samples // 4)
    for start, end in zip(vertices[:-1], vertices[1:]):
        for alpha in np.linspace(0.0, 1.0, per_edge, endpoint=False):
            square_points.append(start + alpha * (end - start))
    square_points.append(vertices[-1])
    return {"circle": circle, "square": np.vstack(square_points)}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--samples", type=int, default=101)
    parser.add_argument("--dt", type=float, default=0.02)
    parser.add_argument("--output", type=Path, default=ROOT / "data/demo/geometric_dls.csv")
    args = parser.parse_args()
    if args.samples < 20 or args.dt <= 0:
        raise SystemExit("--samples must be at least 20 and --dt must be positive")

    seed = np.array([0.30, 0.50, -0.40])
    frames: list[pd.DataFrame] = []
    for name, path in make_paths(args.samples).items():
        q, results = solve_path_dls(path, seed)
        qd = np.gradient(q, args.dt, axis=0, edge_order=2)
        qdd = np.gradient(qd, args.dt, axis=0, edge_order=2)
        xy_d = np.gradient(path, args.dt, axis=0, edge_order=2)
        frames.append(pd.DataFrame({
            "trajectory_id": name,
            "sample": np.arange(len(path)),
            "time_s": np.arange(len(path)) * args.dt,
            "x_m": path[:, 0], "y_m": path[:, 1],
            "q1_rad": q[:, 0], "q2_rad": q[:, 1], "q3_rad": q[:, 2],
            "q1d_rad_s": qd[:, 0], "q2d_rad_s": qd[:, 1], "q3d_rad_s": qd[:, 2],
            "q1dd_rad_s2": qdd[:, 0], "q2dd_rad_s2": qdd[:, 1], "q3dd_rad_s2": qdd[:, 2],
            "xd_m_s": xy_d[:, 0], "yd_m_s": xy_d[:, 1],
            "pen_down": 1,
            "ik_position_error_m": [r.position_error for r in results],
            "ik_iterations": [r.iterations for r in results],
        }))

    data = pd.concat(frames, ignore_index=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    data.to_csv(args.output, index=False)
    print(f"Wrote {len(data)} samples to {args.output}")
    print(data.groupby("trajectory_id")["ik_position_error_m"].agg(["max", "mean"]))


if __name__ == "__main__":
    main()
