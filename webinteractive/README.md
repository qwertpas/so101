# SO-101 Viser controller

For direct browser USB control without running a server, use **https://qwertpas.github.io/so101/**. See the [browser controller instructions](../README.md). This document describes the original local Python app.

Run from `/Users/chris/Code/so101`:

```sh
./webinteractive/run.sh
```

Open **http://127.0.0.1:8080**. The app discovers USB serial adapters, connects at 1 Mbps, verifies all six STS3215 servos against the LeRobot calibration, and starts in **Ghost only**. Initial connection only reads the robot. The macOS `/dev/cu.usbmodem…` and `/dev/tty.usbmodem…` paths refer to the same adapter.

## Controls

- **Ghost only:** drag the cyan ball to move the end-effector target, the orange ring to tilt it, or the purple ring to twist it. The translucent cyan arm previews the commanded pose; the yellow arm follows measured servo positions. The gripper slider previews jaw opening.
- **Track end effector:** resets the target to the current robot pose, enables torque, then follows subsequent target edits. A reachable position is executed even when the requested orientation cannot be matched; the remaining orientation error is shown. An unreachable position holds the measured pose.
- **Joint control:** resets targets to the measured pose and enables the slider in the main panel. All six joints can be controlled independently.
- **Stop & hold:** stops tracking at the measured pose, with torque retained to support the arm. Selecting Ghost only, closing the controlling browser, or normal server shutdown has the same hold behavior. **Free joints (torque off)** explicitly removes motor torque; support the arm first.

The arm has five pose joints plus the gripper, so arbitrary six-dimensional poses are not all reachable. The viewer displays both residuals. This is kinematic preview, not a dynamics simulator. The editable maximum joint speed defaults to 45°/s, and calibrated joint limits are enforced. When motion is armed, servo acceleration is set to 254, matching LeRobot's STS3215 configuration. Collision avoidance is not implemented. A killed process or unplugged adapter cannot send a stop command; servo goals are kept close to measured feedback, but the software stop is not a hardware emergency stop.

The single main panel keeps all six measured positions visible. Connection actions sit below the target controls, and the Servo registers section is at the bottom of the panel. Its details show every motor's angle, encoder position, goal, velocity, load, voltage, current, temperature, motion flag, torque state, and error/status, including documented STS/SMS control-table entries through address 86. Raw register values are preserved alongside converted summaries. Positions and motion commands run together at 50 Hz; all live telemetry is read together and displayed at 5 Hz. Configuration registers refresh every five seconds. Temperature checks use each servo’s configured limit, confirm any high sample through separate temperature and limit reads, and log the measured values. A confirmed thermal or voltage fault stops motion while keeping the connection and telemetry visible. The port uses the same Feetech reply timeout calculation as the adjacent LeRobot code (wire time plus 50 ms), which addresses the SDK timeout bug. The app uses the SDK’s synchronized bus reads, as LeRobot does for arm positions, instead of asking each servo separately on every tick. A missed reply is reported directly and stops motion.

Only one application loop accesses the serial port. An adapter lock and PySerial exclusive access prevent another instance from opening the same port. Other servo tools must be closed. A second browser can view data or stop motion; only the browser that enabled tracking may edit active targets. Reconnection always returns to Ghost only.

## Environment and calibration

The Python 3.12 environment is `/Users/chris/Code/so101/.venv`, independent of LeRobot. To recreate it from the pinned lockfile:

```sh
./webinteractive/setup.sh
```

The default calibration is auto-discovered when exactly one SO follower calibration exists in `~/.cache/huggingface/lerobot/calibration/robots/`. On this machine it is `so_follower/arm1.json`, created by the adjacent LeRobot project. The application reads the JSON directly and checks the servo homing offsets and limits without rewriting them. Joint conversion matches LeRobot's degree convention: `(raw - range_midpoint) * 360 / 4095`. The gripper is measured from its calibrated closed endpoint. The measured calibration endpoints are authoritative for motion limits; the stock URDF limits are generic and differ from this arm’s actual travel.

```sh
./webinteractive/run.sh --serial /dev/tty.usbmodem5B140335431
./webinteractive/run.sh --calibration /path/to/arm.json
./webinteractive/run.sh --demo --port 8081
```

Demo mode never opens a serial port. It uses the calibration limits and simulates joint feedback. Optional `--state-file /tmp/so101.json` writes local diagnostic snapshots. The default bind address is loopback; `--host` can be specified explicitly for a trusted network (there is no authentication).

## Verification

```sh
.venv/bin/python -m pytest webinteractive/tests -q
.venv/bin/ruff check webinteractive
```

Tests cover encoder conversion, signed telemetry, FK agreement with the displayed URDF, reachable/unreachable IK, calibration and fault rejection, stale feedback, goal seeding before torque enable, rate limits, ghost-only behavior, controller ownership, browser disconnect, and joint/Cartesian mode transitions. Hardware telemetry and browser interactions are also checked separately; automated tests do not move the physical robot.

The robot model and license are included in `model/`; see its attribution file. The radio control adapter in `controls.py` uses native HTML radio inputs and Viser's existing button-message transport because Viser 1.1.1 has no native radio widget.
