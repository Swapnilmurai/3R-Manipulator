# ROS 2 workspace

Build on a machine with ROS 2 and colcon installed:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch three_r_description view_robot.launch.py
```

The URDF and joint origins are provisional until compared with the supplied CAD assembly. Do not connect the demo trajectory node to physical actuators before limits, direction conventions, emergency stop, and calibration are verified.
