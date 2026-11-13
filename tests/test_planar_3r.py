import numpy as np
from kinematics.planar_3r import (
    DEFAULT_LINK_LENGTHS, forward_kinematics, solve_ik_dls,
    solve_path_dls, translational_jacobian,
)


def test_jacobian_matches_finite_difference():
    q = np.array([0.3, -0.7, 0.4])
    eps = 1e-7
    numerical = np.column_stack([
        (forward_kinematics(q + np.eye(3)[i] * eps) -
         forward_kinematics(q - np.eye(3)[i] * eps)) / (2 * eps)
        for i in range(3)
    ])
    assert np.allclose(translational_jacobian(q), numerical, atol=2e-8)


def test_dls_recovers_reachable_target():
    q_expected = np.array([0.35, -0.75, 0.55])
    target = forward_kinematics(q_expected)
    result = solve_ik_dls(target, seed=np.array([0.1, -0.4, 0.2]))
    assert result.success
    assert result.position_error <= 2e-5


def test_fk_total_reach_at_zero_pose():
    assert np.allclose(forward_kinematics([0, 0, 0]), [DEFAULT_LINK_LENGTHS.sum(), 0.0])


def test_circle_path_continuity():
    t = np.linspace(0, 2*np.pi, 41)
    path = np.column_stack([0.170 + 0.012*np.cos(t), 0.070 + 0.012*np.sin(t)])
    q, results = solve_path_dls(path, seed=[0.3, 0.5, -0.4])
    assert all(r.success for r in results)
    assert np.max(np.linalg.norm(np.diff(q, axis=0), axis=1)) < 0.05
