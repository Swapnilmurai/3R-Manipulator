# Planar 3R kinematics

`planar_3r.py` provides forward kinematics, a translational Jacobian, and a bounded damped-least-squares (DLS) inverse-kinematics solver. The default link dimensions are approximate CAD pivot-hole spacings of 0.120, 0.100, and 0.060 m; verify them against the assembly mate frames.

The Cartesian task is 2-D position with a redundant 3R chain. Sequential path solving uses the previous point's joint solution as the next seed. It does not yet enforce end-effector yaw, obstacle avoidance, or tool orientation constraints.
