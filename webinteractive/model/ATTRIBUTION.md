# SO-101 model

URDF and mesh files from [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101), retrieved September 23, 2026. The upstream `so101_new_calib.urdf` is stored here as `so101.urdf` without geometry changes.

The upstream Apache 2.0 license is included in `LICENSE`. The model uses the new calibration convention: the midpoint of each arm joint's range is zero. The jaw uses closed = zero and its actual encoder angle, rather than interpreting LeRobot's 0–100 opening percentage as radians.
