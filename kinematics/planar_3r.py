"""Planar 3R forward kinematics, Jacobian, and damped least-squares IK.

All lengths and Cartesian coordinates are metres; all joint angles are radians.
The default dimensions are CAD-derived approximate pivot spacings, not validated
assembly dimensions. This module is a simulation/research utility only.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable
import numpy as np

DEFAULT_LINK_LENGTHS = np.array([0.120, 0.100, 0.060], dtype=float)
DEFAULT_LIMITS = np.array([[-2.967, 2.967], [-2.443, 2.443], [-2.967, 2.967]], dtype=float)


@dataclass(frozen=True)
class IKResult:
    q: np.ndarray
    success: bool
    position_error: float
    iterations: int
    limit_hits: int


def _validate_lengths(lengths: Iterable[float]) -> np.ndarray:
    values = np.asarray(lengths, dtype=float).reshape(-1)
    if values.shape != (3,) or np.any(~np.isfinite(values)) or np.any(values <= 0):
        raise ValueError("lengths must contain three finite positive values")
    return values


def forward_kinematics(q: Iterable[float], lengths: Iterable[float] = DEFAULT_LINK_LENGTHS) -> np.ndarray:
    qv = np.asarray(q, dtype=float).reshape(-1)
    if qv.shape != (3,) or np.any(~np.isfinite(qv)):
        raise ValueError("q must contain three finite joint angles")
    l = _validate_lengths(lengths)
    cumulative = np.cumsum(qv)
    return np.array([np.dot(l, np.cos(cumulative)), np.dot(l, np.sin(cumulative))])


def translational_jacobian(q: Iterable[float], lengths: Iterable[float] = DEFAULT_LINK_LENGTHS) -> np.ndarray:
    qv = np.asarray(q, dtype=float).reshape(-1)
    if qv.shape != (3,) or np.any(~np.isfinite(qv)):
        raise ValueError("q must contain three finite joint angles")
    l = _validate_lengths(lengths)
    a = np.cumsum(qv)
    j = np.zeros((2, 3), dtype=float)
    for k in range(3):
        j[0, k] = -np.sum(l[k:] * np.sin(a[k:]))
        j[1, k] = np.sum(l[k:] * np.cos(a[k:]))
    return j


def solve_ik_dls(
    target_xy: Iterable[float],
    seed: Iterable[float],
    lengths: Iterable[float] = DEFAULT_LINK_LENGTHS,
    limits: Iterable[Iterable[float]] = DEFAULT_LIMITS,
    damping: float = 0.005,
    posture_gain: float = 0.0,
    max_iterations: int = 500,
    tolerance: float = 2e-5,
    max_step: float = 0.12,
) -> IKResult:
    """Solve a 2-D position target for a redundant 3R chain.

    The solver uses the previous solution as a seed when used through
    solve_path_dls(). It has no obstacle avoidance or end-effector yaw objective.
    """
    target = np.asarray(target_xy, dtype=float).reshape(-1)
    q = np.asarray(seed, dtype=float).reshape(-1).copy()
    l = _validate_lengths(lengths)
    lim = np.asarray(limits, dtype=float)
    if target.shape != (2,) or np.any(~np.isfinite(target)):
        raise ValueError("target_xy must contain two finite coordinates")
    if q.shape != (3,) or np.any(~np.isfinite(q)):
        raise ValueError("seed must contain three finite joint angles")
    if lim.shape != (3, 2) or np.any(lim[:, 0] >= lim[:, 1]):
        raise ValueError("limits must be a 3x2 array of lower/upper limits")
    if damping <= 0 or max_iterations < 1 or tolerance <= 0 or max_step <= 0:
        raise ValueError("damping, tolerance, max_step, and max_iterations must be positive")

    q = np.clip(q, lim[:, 0], lim[:, 1])
    q_posture = q.copy()
    hits = 0
    for iteration in range(1, max_iterations + 1):
        error = target - forward_kinematics(q, l)
        error_norm = float(np.linalg.norm(error))
        if error_norm <= tolerance:
            return IKResult(q=q, success=True, position_error=error_norm,
                            iterations=iteration - 1, limit_hits=hits)

        j = translational_jacobian(q, l)
        jj = j @ j.T + (damping ** 2) * np.eye(2)
        primary = j.T @ np.linalg.solve(jj, error)
        j_pinv = j.T @ np.linalg.solve(jj, np.eye(2))
        null_projector = np.eye(3) - j_pinv @ j
        posture = null_projector @ (q_posture - q)
        step = primary + posture_gain * posture
        step_norm = float(np.linalg.norm(step))
        if step_norm > max_step:
            step *= max_step / step_norm
        candidate = q + step
        clipped = np.clip(candidate, lim[:, 0], lim[:, 1])
        hits += int(np.count_nonzero(np.abs(clipped - candidate) > 1e-12))
        q = clipped

    error_norm = float(np.linalg.norm(target - forward_kinematics(q, l)))
    return IKResult(q=q, success=error_norm <= tolerance, position_error=error_norm,
                    iterations=max_iterations, limit_hits=hits)


def solve_path_dls(
    path_xy: Iterable[Iterable[float]],
    seed: Iterable[float],
    lengths: Iterable[float] = DEFAULT_LINK_LENGTHS,
    limits: Iterable[Iterable[float]] = DEFAULT_LIMITS,
) -> tuple[np.ndarray, list[IKResult]]:
    """Solve a Cartesian path sequentially, seeding each point from the previous one."""
    points = np.asarray(path_xy, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        raise ValueError("path_xy must have shape (N, 2) with N >= 1")
    current = np.asarray(seed, dtype=float).reshape(3)
    solutions: list[np.ndarray] = []
    results: list[IKResult] = []
    for point in points:
        result = solve_ik_dls(point, current, lengths=lengths, limits=limits)
        results.append(result)
        solutions.append(result.q.copy())
        if not result.success:
            raise RuntimeError(
                f"IK failed at Cartesian point {point.tolist()} with error {result.position_error:.6g} m"
            )
        current = result.q
    return np.vstack(solutions), results
