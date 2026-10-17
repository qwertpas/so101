# SO-101 controller

Open **https://qwertpas.github.io/so101/** in desktop Chrome or Edge. The browser connects directly to the USB servo adapter at 1 Mbps; no local Python server is needed.

1. Click **Choose port…** and select the USB adapter in the browser's permission dialog. Previously approved adapters appear in the serial dropdown on future visits. Browsers do not expose macOS `/dev/tty…` paths; adapters are listed by USB vendor/product and port number.
2. Click **Connect**. The app verifies all six STS3215 servos, position mode, homing offsets, and calibrated limits. Connection only reads the arm and starts in **Ghost only**.
3. Select **Track end effector** or **Joint control** to enable torque. Activation resets the target to the measured arm pose before motion. Use **Stop & hold** to stop or **Free joints (torque off)** to release it.

Your arm's calibration is saved in [webinteractive/calibrations/arm1.json](webinteractive/calibrations/arm1.json), versioned in this repository, and bundled into the published app by default. Both the browser and Python app use it without depending on the Hugging Face cache. **Override calibration** optionally imports another LeRobot JSON; overrides stay in that browser's local storage and are never uploaded.

Close any Python server or other servo application before connecting. Only one application can own the adapter. **Demo** simulates feedback without opening a serial port. Safari and Firefox can use the demo but do not support this app's Web Serial connection.

The 3D view shows the measured arm in yellow and a cyan ghost preview. Drag the target arrows or edit X/Y/Z in millimeters. Tilt and twist set the requested orientation; position takes priority if the five-axis arm cannot match both. Joint control exposes all six independent sliders. The gripper also has a separate target slider. Servo telemetry and register values appear below the controls.

Motion uses calibrated limits, a configurable speed cap (45°/s initially), fresh feedback, fault checks, and goals no more than about 3° ahead of measured joints. Feedback runs at up to 50 Hz, full telemetry at 5 Hz, and configuration reads every five seconds. One asynchronous loop owns all serial transactions. Switching tabs or a control-loop gap over 250 ms stops motion and holds position; resuming requires explicitly selecting a motion mode again. Browser background throttling means this page must stay visible while controlling the arm.

Support the arm before releasing torque. Closing/crashing the browser or unplugging USB cannot guarantee a stop command; the software stop is not a hardware emergency stop. Collision avoidance is not implemented.

## Development

```sh
npm ci
npm run dev
npm test
npm run build
```

The browser app is in `webinteractive/browser/` and uses the existing URDF/STL assets in `webinteractive/model/`. Tests exercise the real serial driver against a simulated six-servo byte stream, including fragmented packets and safety behavior, plus kinematics against Python-generated reference poses. They never access hardware.

Pushes to `main` run tests, build the static app, and deploy it with GitHub Actions. GitHub Pages must use **GitHub Actions** as its publishing source.

The original Python/Viser application remains available; see [its README](webinteractive/README.md). Its pinned environment and packages are unchanged. Model attribution and license are in [webinteractive/model/ATTRIBUTION.md](webinteractive/model/ATTRIBUTION.md).
