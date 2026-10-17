"""Run with ../.venv/bin/python app.py (or ./run.sh)."""

import argparse
import json
import logging
import signal
import time
from html import escape
from pathlib import Path
from queue import Empty, SimpleQueue

import numpy as np
import viser
from scipy.spatial.transform import Rotation
from viser.extras import ViserUrdf

from calibration import NAMES, TICK, Calibration, find_calibration
from controls import ModeRadio
from hardware import REGISTERS, Bus, MotionFault, candidates
from kinematics import MODEL, Kinematics, Solution

LOG = logging.getLogger("so101")
MODES = ("Ghost only", "Track end effector", "Joint control")
LABELS = tuple(n.replace("_", " ").title() for n in NAMES)
CONTROL_HZ = 50
CONTROL_PERIOD = 1 / CONTROL_HZ


def ring_mesh(radius, tube):
    around = np.linspace(0, 2 * np.pi, 65)[:-1]
    across = np.linspace(0, 2 * np.pi, 9)[:-1]
    vertices = np.array(
        [
            [
                (radius + tube * np.cos(v)) * np.cos(u),
                (radius + tube * np.cos(v)) * np.sin(u),
                tube * np.sin(v),
            ]
            for u in around
            for v in across
        ]
    )
    faces = []
    for u in range(len(around)):
        for v in range(len(across)):
            a = u * len(across) + v
            b = ((u + 1) % len(around)) * len(across) + v
            c = u * len(across) + (v + 1) % len(across)
            d = ((u + 1) % len(around)) * len(across) + (v + 1) % len(across)
            faces.extend(((a, b, c), (c, b, d)))
    return vertices, np.array(faces)


class App:
    def __init__(self, args):
        self.args = args
        self.calibration = Calibration(find_calibration(args.calibration))
        self.kin = Kinematics(self.calibration)
        self.bus = None
        self.events = SimpleQueue()
        self.running = True
        self.mode = MODES[0]
        self.max_speed = 45.0
        self.owner = None
        self.clients = set()
        self.drags = {}
        self.rotation_drags = {}
        self.actual = np.clip(np.zeros(6), self.kin.low, self.kin.high)
        self.ghost = self.actual.copy()
        self.tilt = float(sum(self.ghost[1:4]))
        self.twist = float(self.ghost[4])
        self.target = self.kin.pose(self.actual)
        self.request_angles = self.ghost.copy()
        self.solution = self.kin.solve(self.target, self.ghost)
        self.dirty = False
        self.match_rotation = True
        self.message = "Demo — no hardware" if args.demo else "Looking for six servos…"
        self.next_connect = 0.0
        self.next_display = 0.0
        self.next_state = 0.0
        self.last_loop = time.monotonic()
        self.server = viser.ViserServer(host=args.host, port=args.port, label="SO-101")
        self.build_scene()
        self.build_gui()

        @self.server.on_client_connect
        async def connect(client):
            client.camera.position = (0.65, 0.65, 0.45)
            client.camera.look_at = (0.0, -0.06, 0.16)
            client.camera.up_direction = (0, 0, 1)
            self.events.put(("connect", client.client_id, None))

        @self.server.on_client_disconnect
        async def disconnect(client):
            self.events.put(("disconnect", client.client_id, None))

    def build_scene(self):
        scene = self.server.scene
        scene.set_up_direction("+z")
        scene.add_grid(
            "/floor",
            width=1.4,
            height=1.4,
            cell_size=0.05,
            section_size=0.25,
            position=(0, 0, -0.004),
            cell_color=(65, 75, 90),
            section_color=(90, 105, 125),
        )
        scene.add_frame("/base", axes_length=0.07, axes_radius=0.002)
        self.real_root = scene.add_frame("/robot", show_axes=False)
        self.ghost_root = scene.add_frame("/ghost", show_axes=False)
        self.real = ViserUrdf(self.server, MODEL, root_node_name="/robot")
        self.preview = ViserUrdf(
            self.server, MODEL, root_node_name="/ghost", mesh_color_override=(0.1, 0.85, 1.0, 0.3)
        )
        self.order = [NAMES.index(n) for n in self.real.get_actuated_joint_names()]
        self.real_root.visible = self.args.demo
        self.tool = scene.add_frame("/tool", axes_length=0.035, axes_radius=0.0015)
        self.control = scene.add_frame(
            "/target",
            axes_length=0.045,
            axes_radius=0.0015,
            position=self.target[:3, 3],
            wxyz=Rotation.from_matrix(self.target[:3, :3]).as_quat(scalar_first=True),
        )
        self.handle_point = scene.add_icosphere("/target/handle", radius=0.012, color=(60, 220, 255))
        self.rings = []
        for index, (name, radius, color) in enumerate(
            (("tilt", 0.032, (255, 174, 61)), ("twist", 0.044, (199, 119, 255)))
        ):
            vertices, faces = ring_mesh(radius, 0.002)
            ring = scene.add_mesh_simple(
                f"/target/{name}", vertices, faces, color=color, side="double", cast_shadow=False
            )
            self.rings.append(ring)

            @ring.on_drag
            async def ring_changed(event, index=index):
                self.rotate_target(event, index)

        theta = np.linspace(0, 2 * np.pi, 65)[:-1]
        scene.add_spline_catmull_rom(
            "/target/circle",
            points=np.c_[0.019 * np.cos(theta), 0.019 * np.sin(theta), np.zeros_like(theta)],
            closed=True,
            color=(60, 220, 255),
            thickness=0.0015,
        )
        self.tip = scene.add_icosphere("/ghost_tip", radius=0.004, color=(60, 220, 255))

        @self.handle_point.on_drag
        async def target_changed(event):
            client = event.client_id
            if event.phase == "start":
                if self.owner is None or self.owner == client:
                    self.drags[client] = (self.target[:3, 3].copy(), np.array(event.start_position))
                return
            start = self.drags.get(client)
            if start is not None:
                pose = self.target.copy()
                pose[:3, 3] = start[0] + np.array(event.end_position) - start[1]
                self.events.put(("target", client, pose))
            if event.phase == "end":
                self.drags.pop(client, None)

        self.position_rings()

    def position_rings(self):
        for ring, axis in zip(self.rings, self.kin.rotation_axes(self.request_angles), strict=True):
            local_axis = self.target[:3, :3].T @ axis
            turn = Rotation.align_vectors([local_axis], [[0, 0, 1]])[0]
            ring.wxyz = turn.as_quat(scalar_first=True)

    def rotate_target(self, event, index):
        key = (event.client_id, index)
        if event.phase == "start":
            if self.mode == MODES[2] or not self.match_rotation or self.owner not in (None, event.client_id):
                return
            axis = self.kin.rotation_axes(self.request_angles)[index]
            tangent = np.cross(axis, np.array(event.start_position) - self.target[:3, 3])
            tangent /= np.linalg.norm(tangent)
            value = self.tilt if index == 0 else self.twist
            self.rotation_drags[key] = (value, np.array(event.start_position), tangent, (0.032, 0.044)[index])
            return
        start = self.rotation_drags.get(key)
        if start is not None:
            change = np.dot(np.array(event.end_position) - start[1], start[2]) / start[3]
            self.events.put(("orientation", event.client_id, (index, np.rad2deg(start[0] + change))))
        if event.phase == "end":
            self.rotation_drags.pop(key, None)

    def bind(self, handle, kind, value=None):
        @handle.on_update
        async def changed(event):
            if event.client is not None:
                self.events.put(
                    (kind, event.client.client_id, handle.value if value is None else (value, handle.value))
                )

    def button(self, label, kind, **kwargs):
        handle = self.server.gui.add_button(label, **kwargs)

        @handle.on_click
        async def clicked(event):
            if event.client is not None:
                self.events.put((kind, event.client.client_id, None))

        return handle

    def build_gui(self):
        gui = self.server.gui
        gui.configure_theme(
            dark_mode=True,
            control_width="large",
            show_logo=False,
            show_share_button=False,
            brand_color=(45, 195, 222),
        )
        gui.main_panel.dock_right()
        self.status = gui.add_markdown(self.message)
        gui.main_panel.set_width(390)
        self.mode_control = ModeRadio(gui, MODES, self.events)
        self.speed_control = gui.add_slider(
            "Max speed (°/s)", min=1, max=180, step=1, initial_value=self.max_speed
        )
        self.bind(self.speed_control, "speed")
        gui.add_html(
            '<div style="color:#aab8c4;font-size:12px;padding:0 12px 7px">'
            "Ghost previews motion · Track follows the target · Joint moves each servo</div>"
        )
        self.button("Stop & hold", "stop", color="red")
        gui.add_html('<div style="border-top:1px solid #34414c;margin:5px 10px 8px"></div>')
        self.overview = gui.add_html("")
        self.sliders, self.info, self.register_text = [], [], []
        gui.add_html('<div style="font-weight:600;padding:2px 12px 7px">Joint angles</div>')
        for i, label in enumerate(LABELS):
            slider = gui.add_slider(
                label.replace(" Shoulder", "").replace(" Wrist", ""),
                min=round(float(np.rad2deg(self.kin.low[i])), 1),
                max=round(float(np.rad2deg(self.kin.high[i])), 1),
                step=0.1,
                initial_value=float(np.rad2deg(self.ghost[i])),
                disabled=True,
            )
            self.sliders.append(slider)
            self.bind(slider, "joint", i)

        gui.add_html('<div style="font-weight:600;padding:2px 12px 7px">Target</div>')
        self.rotation_control = gui.add_checkbox("Match tool orientation", initial_value=True)
        self.bind(self.rotation_control, "rotation")
        self.orientation = []
        for i, (label, low, high, value) in enumerate(
            (
                ("Tilt (°)", sum(self.kin.low[1:4]), sum(self.kin.high[1:4]), self.tilt),
                ("Twist (°)", self.kin.low[4], self.kin.high[4], self.twist),
            )
        ):
            slider = gui.add_slider(
                label,
                min=round(float(np.rad2deg(low)), 1),
                max=round(float(np.rad2deg(high)), 1),
                step=0.1,
                initial_value=float(np.rad2deg(value)),
            )
            self.orientation.append(slider)
            self.bind(slider, "orientation", i)
        self.grip = gui.add_slider(
            "Gripper (°)",
            min=round(float(np.rad2deg(self.kin.low[5])), 1),
            max=round(float(np.rad2deg(self.kin.high[5])), 1),
            step=0.1,
            initial_value=0.0,
        )
        self.bind(self.grip, "joint", 5)
        self.button("Reset target to arm", "reset")
        self.pose_text = gui.add_html("")
        self.ik_text = gui.add_html("")
        self.show_ghost = gui.add_checkbox("Show ghost", initial_value=True)
        self.bind(self.show_ghost, "visibility")
        gui.add_html(
            '<div style="color:#aab8c4;font-size:12px;padding:4px 12px">'
            "Drag cyan to move · Orange ring tilts · Purple ring twists</div>"
        )
        gui.add_html('<div style="font-weight:600;padding:8px 12px 4px">Connection</div>')
        self.connection_text = gui.add_html("")
        self.button("Reconnect servos", "reconnect")
        self.button("Free joints (torque off)", "release", color="orange")

        with gui.add_folder("Servo registers", expand_by_default=False):
            for i, label in enumerate(LABELS):
                with gui.add_folder(label, expand_by_default=False):
                    self.info.append(gui.add_markdown("Waiting for telemetry…"))
                    self.register_text.append(gui.add_markdown("Waiting for telemetry…"))
            gui.add_markdown(f"Calibration: `{self.calibration.path}`")

    def sync_target(self):
        self.drags.clear()
        self.rotation_drags.clear()
        self.tilt = float(sum(self.ghost[1:4]))
        self.twist = float(self.ghost[4])
        self.target = self.kin.pose(self.ghost)
        self.request_angles = self.ghost.copy()
        self.control.position = self.target[:3, 3]
        self.control.wxyz = Rotation.from_matrix(self.target[:3, :3]).as_quat(scalar_first=True)
        self.position_rings()
        self.solution = Solution(self.ghost.copy(), 0.0, 0.0, True)
        self.dirty = False

    def stop(self, message="Holding position"):
        self.drags.clear()
        self.rotation_drags.clear()
        if self.bus:
            self.bus.hold()
        self.mode = MODES[0]
        self.mode_control.value = self.mode
        self.owner = None
        self.message = message

    def reset(self):
        self.ghost = np.clip(self.actual, self.kin.low, self.kin.high)
        self.sync_target()

    def handle(self, kind, client, value):
        if kind == "connect":
            self.clients.add(client)
            return
        if kind == "disconnect":
            self.clients.discard(client)
            self.drags.pop(client, None)
            for key in [key for key in self.rotation_drags if key[0] == client]:
                self.rotation_drags.pop(key)
            if self.owner == client:
                self.stop("Controller disconnected — holding position")
            return
        if kind in ("stop", "release"):
            self.stop()
            if kind == "release" and self.bus:
                self.bus.release()
                self.message = "Torque released"
            return
        if self.owner is not None and client != self.owner:
            self.mode_control.value = self.mode
            self.speed_control.value = self.max_speed
            self.message = "Another browser owns robot control; Stop is available to everyone"
            return
        if kind == "mode":
            self.next_display = 0
            if value == MODES[0]:
                self.stop("Ghost only — robot tracking stopped")
            elif value in MODES:
                if client not in self.clients:
                    return
                if not self.bus and not self.args.demo:
                    self.mode_control.value = self.mode
                    self.message = "Connect all six calibrated servos before enabling motion"
                    return
                self.stop()
                self.reset()  # Never execute a target staged before activation.
                if self.bus:
                    self.bus.arm()
                self.owner = client
                self.mode = value
                self.mode_control.value = value
                self.message = f"{'Demo · ' if self.args.demo else ''}{value} active"
        elif kind == "target" and self.mode != MODES[2]:
            self.target = value
            self.control.position = self.target[:3, 3]
            self.dirty = True
        elif kind == "orientation" and self.mode != MODES[2]:
            if value[0] == 0:
                self.tilt = float(
                    np.clip(np.deg2rad(value[1]), sum(self.kin.low[1:4]), sum(self.kin.high[1:4]))
                )
            else:
                self.twist = float(np.clip(np.deg2rad(value[1]), self.kin.low[4], self.kin.high[4]))
            self.dirty = True
        elif kind == "rotation":
            self.match_rotation = value
            self.dirty = True
        elif kind == "speed":
            if np.isfinite(value):
                self.max_speed = float(np.clip(value, 1, 180))
                if self.bus:
                    self.bus.max_speed = self.max_speed
            self.speed_control.value = self.max_speed
        elif kind == "reset":
            self.reset()
        elif kind == "joint":
            index, degrees = value
            if self.mode == MODES[2] or index == 5:
                self.ghost[index] = np.clip(np.deg2rad(degrees), self.kin.low[index], self.kin.high[index])
                if self.mode == MODES[2]:
                    self.sync_target()
        elif kind == "visibility":
            self.ghost_root.visible = value
            self.tip.visible = value
        elif kind == "reconnect":
            self.stop()
            if self.bus:
                self.bus.close()
                self.bus = None
            self.real_root.visible = self.args.demo
            self.next_connect = 0

    def connect(self):
        errors = []
        for path in candidates(self.args.serial):
            bus = Bus(path, self.calibration)
            try:
                bus.connect()
            except Exception as exc:
                errors.append(f"{path}: {exc}")
                continue
            self.bus = bus
            bus.max_speed = self.max_speed
            self.actual = bus.angles
            self.reset()
            self.real_root.visible = True
            self.message = "Connected · 6/6 servos · ghost only"
            LOG.info("Connected to %s; calibration matches all six servos", path)
            return
        self.message = "Connection: " + ("; ".join(errors) if errors else "No USB serial adapter found")

    def fault(self, exc):
        LOG.exception("Controller stopped")
        if isinstance(exc, MotionFault):
            self.stop(f"Motion stopped: {exc}")
            return
        if self.bus:
            try:
                self.bus.hold()
            except Exception:
                LOG.exception("Unable to send hold command")
            self.bus.close()
            self.bus = None
        self.mode = MODES[0]
        self.mode_control.value = self.mode
        self.owner = None
        self.real_root.visible = False
        self.message = f"Stopped: {exc}. Reconnecting in ghost mode…"
        self.next_connect = time.monotonic() + 3

    def step(self):
        now = time.monotonic()
        dt = now - self.last_loop
        self.last_loop = now
        if not self.args.demo and not self.bus and now >= self.next_connect:
            self.connect()
            self.next_connect = now + 3
        if self.bus:
            self.bus.read()
            self.actual = self.bus.angles
        # Coalesce drag events without delaying stop/mode events.
        pending_target = None
        for _ in range(1000):
            try:
                event = self.events.get_nowait()
            except Empty:
                break
            if event[0] == "target":
                pending_target = event
            else:
                if pending_target:
                    self.handle(*pending_target)
                    pending_target = None
                self.handle(*event)
        if pending_target:
            self.handle(*pending_target)
        if self.dirty:
            if self.match_rotation:
                requested = self.kin.solve_with_tilt(self.target[:3, 3], self.tilt, self.twist, self.ghost)
                self.request_angles = requested.angles
                self.target[:3, :3] = self.kin.pose(requested.angles)[:3, :3]
                self.solution = (
                    requested
                    if requested.reachable
                    else self.kin.solve_position_first(self.target, self.ghost)
                )
                if self.solution.reachable:
                    actual_rotation = self.kin.pose(self.solution.angles)[:3, :3]
                    self.solution.rotation_error = float(
                        np.linalg.norm(
                            Rotation.from_matrix(self.target[:3, :3].T @ actual_rotation).as_rotvec()
                        )
                    )
            else:
                self.solution = self.kin.solve(self.target, self.ghost, match_rotation=False)
                if self.solution.reachable:
                    self.tilt = float(sum(self.solution.angles[1:4]))
                    self.twist = float(self.solution.angles[4])
                self.request_angles = self.solution.angles
                self.target[:3, :3] = self.kin.pose(self.solution.angles)[:3, :3]
            self.control.wxyz = Rotation.from_matrix(self.target[:3, :3]).as_quat(scalar_first=True)
            self.position_rings()
            if self.solution.reachable:
                self.ghost = self.solution.angles
            self.dirty = False
        if not self.solution.reachable:
            self.ghost = self.actual.copy()
        active = self.mode == MODES[2] or (self.mode == MODES[1] and self.solution.reachable)
        if active:
            if self.bus:
                self.bus.move(self.ghost, dt)
            elif self.args.demo:
                self.actual += np.clip(
                    self.ghost - self.actual,
                    -np.deg2rad(self.max_speed) * min(dt, 0.05),
                    np.deg2rad(self.max_speed) * min(dt, 0.05),
                )
        elif self.bus and self.bus.armed:
            # An unreachable target immediately holds measured position.
            self.bus.goals([r["Position"] for r in self.bus.rows])
            self.bus.command = self.actual.copy()
        with self.server.atomic():
            self.real.update_cfg(self.actual[self.order])
            self.preview.update_cfg(self.ghost[self.order])
            actual_pose = self.kin.pose(self.actual)
            self.tool.position = actual_pose[:3, 3]
            self.tool.wxyz = Rotation.from_matrix(actual_pose[:3, :3]).as_quat(scalar_first=True)
            self.tool.visible = self.bus is not None or self.args.demo
            self.tip.position = self.kin.pose(self.ghost)[:3, 3]
        if now >= self.next_display:
            self.display()
            self.next_display = now + 0.2
        if self.args.state_file and now >= self.next_state:
            self.save_state()
            self.next_state = now + 0.5

    def display(self):
        self.status.content = f"**{self.message}**"
        port = self.bus.path if self.bus else "Demo" if self.args.demo else "Disconnected"
        self.connection_text.content = (
            '<div style="color:#aab8c4;font-size:11px;padding:6px 12px">'
            f"{escape(port)} · {len(self.clients)} browser(s) · {CONTROL_HZ} Hz position · 5 Hz telemetry</div>"
        )
        joint_mode = self.mode == MODES[2]
        for i, slider in enumerate(self.sliders):
            slider.disabled = not joint_mode
            slider.value = round(float(np.rad2deg(self.ghost[i])), 1)
        self.grip.value = round(float(np.rad2deg(self.ghost[5])), 1)
        self.grip.disabled = joint_mode
        self.control.visible = self.mode != MODES[2]
        for ring in self.rings:
            ring.visible = self.mode != MODES[2] and self.match_rotation
        for i, slider in enumerate(self.orientation):
            slider.disabled = joint_mode or not self.match_rotation
            slider.value = round(float(np.rad2deg(self.tilt if i == 0 else self.twist)), 1)
        p = self.target[:3, 3] * 1000
        self.pose_text.content = (
            '<div style="font-size:12px;padding:1px 12px;color:#aab8c4">'
            f"Target X {p[0]:.0f} · Y {p[1]:.0f} · Z {p[2]:.0f} mm</div>"
        )
        if not self.solution.reachable:
            reach = "Position unreachable · holding"
        elif self.match_rotation and self.solution.rotation_error >= np.deg2rad(15):
            reach = "Position reached · orientation limited"
        else:
            reach = "Reachable"
        color = "#7ce1b2" if reach == "Reachable" else "#ffb36b"
        self.ik_text.content = (
            f'<div style="font-size:12px;padding:1px 12px;color:{color}">{reach}'
            f" · {self.solution.position_error * 1000:.1f} mm · "
            f"{np.rad2deg(self.solution.rotation_error):.1f}° error</div>"
        )
        if self.bus:
            rows = self.bus.rows
            self.overview.content = (
                '<div style="padding:0 12px 8px"><div style="font-weight:600;margin-bottom:4px">Servos'
                '</div><table style="width:100%;font-size:12px;border-collapse:collapse">'
                '<tr style="color:#aab8c4"><th align="left">Joint</th><th align="right">Angle</th>'
                '<th align="right">Temp</th><th align="right">Torque</th></tr>'
                + "\n".join(
                    f'<tr><td>{escape(label)}</td><td align="right">{np.rad2deg(q):.1f}°</td>'
                    f'<td align="right">{row["Temperature"]}°C</td><td align="right">'
                    f"{'On' if row['Torque enabled'] else 'Off'}</td></tr>"
                    for label, q, row in zip(LABELS, self.actual, rows, strict=True)
                )
                + "</table></div>"
            )
            for i, row in enumerate(rows):
                speed = row["Velocity"] * np.rad2deg(TICK)
                self.info[
                    i
                ].content = f"**{np.rad2deg(self.actual[i]):.2f}°** · encoder **{row['Position']}**\n\n{speed:.1f}°/s · {row['Voltage'] / 10:.1f} V · {row['Temperature']}°C\n\nLoad {row['Load'] / 10:.1f}% · current {row['Current'] * 6.5:.0f} mA\n\nGoal {row['Goal position']} ticks · {'moving' if row['Moving'] else 'still'} · torque {'on' if row['Torque enabled'] else 'off'} · status {row['Status']} · packet error {row['Packet error']}"
                self.register_text[i].content = (
                    "| Register | Address | Raw value |\n|---|---:|---:|\n"
                    + "\n".join(f"| {name} | {addr} | {row[name]} |" for name, addr, _, _ in REGISTERS)
                )
        else:
            self.overview.content = (
                '<div style="font-size:12px;padding:0 12px 8px">Demo · simulated positions</div>'
                if self.args.demo
                else '<div style="font-size:12px;padding:0 12px 8px">Servo telemetry unavailable</div>'
            )
            for i in range(6):
                self.info[i].content = (
                    f"Simulated: {np.rad2deg(self.actual[i]):.2f}°"
                    if self.args.demo
                    else "No current servo feedback"
                )
                self.register_text[i].content = "No hardware data"

    def save_state(self):
        state = dict(
            time=time.time(),
            mode=self.mode,
            max_speed=self.max_speed,
            control_hz=CONTROL_HZ,
            connected=self.bus is not None,
            demo=self.args.demo,
            port=self.bus.path if self.bus else None,
            message=self.message,
            actual=self.actual.tolist(),
            ghost=self.ghost.tolist(),
            target=self.target.tolist(),
            ik=dict(
                position_error=self.solution.position_error,
                rotation_error=self.solution.rotation_error,
                reachable=self.solution.reachable,
            ),
            servos=self.bus.rows if self.bus else [],
            clients=len(self.clients),
        )
        path = Path(self.args.state_file)
        temp = path.with_suffix(".tmp")
        temp.write_text(json.dumps(state, indent=2))
        temp.replace(path)

    def run(self):
        try:
            while self.running:
                start = time.monotonic()
                try:
                    self.step()
                except Exception as exc:
                    self.fault(exc)
                    self.display()
                time.sleep(max(0, CONTROL_PERIOD - (time.monotonic() - start)))
        finally:
            try:
                self.stop("Server stopped — holding position")
            finally:
                if self.bus:
                    self.bus.close()
                self.server.stop()


def arguments():
    parser = argparse.ArgumentParser(description="SO-101 Viser viewer and controller")
    parser.add_argument("--serial", help="Serial port; default: automatically detect USB adapters")
    parser.add_argument("--calibration", help="LeRobot follower calibration JSON")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--demo", action="store_true", help="Simulate feedback without opening a serial port")
    parser.add_argument("--state-file", help="Optional JSON telemetry snapshot for local diagnostics")
    return parser.parse_args()


def main():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    app = App(arguments())
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: setattr(app, "running", False))
    app.run()


if __name__ == "__main__":
    main()
