# Hardware integration plan

This document defines the integration sequence for the physical writing manipulator. The current repository is simulation-first; none of the hardware steps below should be considered commissioned.

## Target hardware context

The planned system includes three servo-driven revolute joints, an STM32 F4VE controller/servo-driver interface, and two infrared proximity sensors mounted near the end effector. The native CAD assembly, exact servo model/driver protocol, sensor part numbers, sensor field-of-view, and measured calibration values must be confirmed before implementing the final hardware interface.

## ROS 2 software boundaries

Keep three layers separate:

1. **Motion planning:** Cartesian strokes, joint-space trajectories, joint-limit checking, collision/clearance checking, and writing state machine.
2. **Hardware abstraction:** ROS 2 controller interface with joint state feedback, trajectory command conversion, watchdog, and fault reporting.
3. **Embedded controller:** STM32-side servo communication, sensor acquisition, command timeout, and hardware emergency-stop handling.

The serial/CAN/USB transport and message format are intentionally unspecified until the actual servo driver protocol and MCU firmware are known.

## Writing and obstacle-relocation state machine

Suggested task states:

- `IDLE`: await a writing task and verified home state.
- `PLAN`: generate and validate the complete stroke sequence.
- `DRAW`: track the current pen-down path.
- `OBSTACLE_STOP`: stop/hold motion and latch the sensor event.
- `RELOCATE`: compute and validate an alternate reachable, collision-free writing region.
- `RESUME`: replan the remaining strokes at the new location.
- `DONE` / `FAULT`: report completion or enter a safe stopped state.

Sensor events must include calibrated range, timestamp, sensor identity, and validity. The two sensor frames and their ray directions need to be measured from the assembled end-effector CAD and verified against physical sensor tests.

## Required safety checks before hardware motion

- Confirm joint numbering, direction, zero offset, hard/soft limits, and position feedback for all three axes.
- Calibrate servo command units to radians and verify repeatability at low speed with the pen removed.
- Validate sensor mounting, range, blind zone, field of view, false-positive rate, and detection latency.
- Add command timeout, watchdog, fault latch, mechanical travel limits, and an independent emergency-stop circuit.
- Begin with low speed, low torque/current limits, guarded workspace, and pen lifted.
- Test emergency stop and communication-loss handling before enabling writing trajectories.

## Evidence separation

A software trajectory dataset, MuJoCo run, ROS 2 visualization, offline sensor-ray diagnostic, and physical hardware test are different evidence classes. Record the environment and evidence type with every experiment; do not label offline analysis as simulation or hardware validation.
