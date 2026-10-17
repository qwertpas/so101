import asyncio
import json
import time
from argparse import Namespace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import yourdfpy

from app import MODES, App
from calibration import NAMES, Calibration
from hardware import REGISTERS, Bus, MotionFault, Port, decode
from kinematics import MODEL, Kinematics


@pytest.fixture
def calibration(tmp_path):
    data = {
        name: dict(id=i + 1, drive_mode=0, homing_offset=100 * i, range_min=500, range_max=3595)
        for i, name in enumerate(NAMES)
    }
    data["gripper"].update(range_min=2000, range_max=3500)
    path = tmp_path / "arm.json"
    path.write_text(json.dumps(data))
    return Calibration(path)


def rows(calibration):
    result = []
    for name in NAMES:
        saved = calibration.data[name]
        result.append(
            {
                **{name: 0 for name, *_ in REGISTERS},
                "ID": saved["id"],
                "Model number": 777,
                "Operating mode": 0,
                "Homing offset": saved["homing_offset"],
                "Min position limit": saved["range_min"],
                "Max position limit": saved["range_max"],
                "Position": (saved["range_min"] + saved["range_max"]) // 2,
                "Temperature": 25,
                "Max temperature": 70,
                "Voltage": 120,
                "Min voltage": 40,
                "Max voltage": 140,
                "Packet error": 0,
                "Status": 0,
            }
        )
    return result


def fake_bus(calibration):
    bus = Bus("/dev/test", calibration)
    bus.rows = rows(calibration)
    bus.last_read = time.monotonic()
    bus.port = Mock(is_open=True)
    bus.packet = Mock()
    bus.packet.syncWriteTxOnly.return_value = 0
    bus.packet.writeTxRx.return_value = (0, 0)

    def read(port, servo, address, length):
        row = next(r for r in bus.rows if r["ID"] == servo)
        values = {13: row["Max temperature"], 63: row["Temperature"]}
        return [values[address]], 0, 0

    bus.packet.readTxRx.side_effect = read
    return bus


def test_encoder_roundtrip_and_gripper_zero(calibration):
    raw = np.array([700, 1234, 2048, 2750, 3100, 2100])
    np.testing.assert_array_equal(calibration.ticks(calibration.angles(raw)), raw)
    assert calibration.angles(calibration.low)[5] == 0
    np.testing.assert_array_equal(calibration.ticks(np.ones(6) * 100), calibration.high)
    with pytest.raises(ValueError):
        calibration.ticks([np.nan] * 6)


def test_signed_registers():
    data = bytearray(87)
    for address, value in [
        (31, 0x800 | 641),
        (56, 0x8000 | 200),
        (58, 0x8000 | 23),
        (60, 0x400 | 10),
        (69, 30),
    ]:
        data[address : address + 2] = value.to_bytes(2, "little")
    row = decode(data)
    assert [row[n] for n in ["Homing offset", "Position", "Velocity", "Load", "Current"]] == [
        -641,
        -200,
        -23,
        -10,
        30,
    ]


def test_fk_matches_visual_urdf(calibration):
    kin = Kinematics(calibration)
    model = yourdfpy.URDF.load(MODEL, load_meshes=False)
    for q in [np.zeros(6), np.array([0.3, -0.7, 0.8, 0.2, -0.3, 0.5])]:
        model.update_cfg(dict(zip(NAMES, q)))
        np.testing.assert_allclose(kin.pose(q), model.get_transform("gripper_frame_link"), atol=1e-9)


@pytest.mark.parametrize(
    "q",
    [[0.2, -0.4, 0.5, 0.2, -0.3, 0.5], [-0.5, 0.6, -0.7, 0.4, 0.8, 0.3], [1.0, -0.5, 0.9, -0.3, -1.0, 0.1]],
)
def test_ik_reaches_known_pose(calibration, q):
    kin = Kinematics(calibration)
    q = np.array(q)
    solution = kin.solve(kin.pose(q), q + [0.15, -0.1, 0.1, -0.1, 0.1, 0])
    assert solution.reachable
    assert solution.position_error < 1e-4
    assert solution.rotation_error < 0.002
    assert np.all(solution.angles >= kin.low)
    assert np.all(solution.angles <= kin.high)


def test_unreachable_pose_is_bounded(calibration):
    kin = Kinematics(calibration)
    target = np.eye(4)
    target[:3, 3] = [2, 2, 2]
    solution = kin.solve(target, np.zeros(6))
    assert not solution.reachable
    assert np.all(solution.angles >= kin.low)
    assert np.all(solution.angles <= kin.high)


def test_position_only_ik(calibration):
    kin = Kinematics(calibration)
    q = np.array([0.2, -0.4, 0.5, 0.2, -0.3, 0])
    target = kin.pose(q)
    target[:3, :3] = np.eye(3)
    result = kin.solve(target, q + 0.1, match_rotation=False)
    assert result.reachable
    assert result.position_error < 1e-4


def test_orientation_uses_total_bend_and_twist(calibration):
    kin = Kinematics(calibration)
    q = np.array([0.2, -0.4, 0.5, 0.2, -0.3, 0.0])
    for change in ([0, 0.1, -0.1, 0, 0, 0], [0, 0, 0.1, -0.1, 0, 0]):
        np.testing.assert_allclose(kin.pose(q)[:3, :3], kin.pose(q + change)[:3, :3], atol=1e-10)

    position = kin.pose(q)[:3, 3]
    tilt = sum(q[1:4]) + 0.2
    twist = q[4] + 0.3
    solution = kin.solve_with_tilt(position, tilt, twist, q)
    assert solution.reachable
    assert solution.position_error < 1e-4
    assert sum(solution.angles[1:4]) == pytest.approx(tilt)
    assert solution.angles[4] == pytest.approx(twist)
    np.testing.assert_allclose(kin.pose(solution.angles)[:3, 3], position, atol=1e-4)


def test_calibration_mismatch_blocks_arm(calibration):
    bus = fake_bus(calibration)
    bus.rows[2]["Homing offset"] += 1
    with pytest.raises(ValueError, match="Homing offset"):
        bus.arm()
    bus.packet.syncWriteTxOnly.assert_not_called()
    bus.packet.writeTxRx.assert_not_called()


@pytest.mark.parametrize(
    "field,value", [("Status", 4), ("Packet error", 8), ("Temperature", 75), ("Voltage", 20)]
)
def test_fault_blocks_writes(calibration, field, value):
    bus = fake_bus(calibration)
    bus.rows[1][field] = value
    with pytest.raises(MotionFault):
        bus.arm()
    bus.packet.syncWriteTxOnly.assert_not_called()


def test_arm_seeds_positions_before_enabling_torque(calibration):
    bus = fake_bus(calibration)
    bus.arm()
    calls = bus.packet.mock_calls
    assert calls[0][0] == "syncWriteTxOnly"
    params = calls[0].args[3]
    assert [params[i + 1] + 256 * params[i + 2] for i in range(0, len(params), 7)] == [
        r["Position"] for r in bus.rows
    ]
    assert len([c for c in calls if c[0] == "writeTxRx" and c.args[2] == 40]) == 6


def test_motion_rate_limit_and_hold(calibration):
    bus = fake_bus(calibration)
    bus.arm()
    initial = bus.command.copy()
    bus.move(initial + 0.5, 10)  # A stalled loop must not create a large jump.
    assert np.max(bus.command - initial) <= np.deg2rad(2.25) + 1e-10
    bus.hold()
    assert not bus.armed
    np.testing.assert_allclose(bus.command, bus.angles)


def test_stale_feedback_never_moves(calibration):
    bus = fake_bus(calibration)
    bus.arm()
    bus.packet.reset_mock()
    bus.last_read -= 1
    with pytest.raises(OSError, match="stale"):
        bus.move(bus.angles + 0.1, 0.05)
    bus.packet.syncWriteTxOnly.assert_not_called()


@pytest.fixture
def app(calibration, tmp_path):
    app = App(
        Namespace(
            calibration=str(calibration.path),
            demo=True,
            host="127.0.0.1",
            port=0,
            serial=None,
            state_file=str(tmp_path / "state.json"),
        )
    )
    app.handle("connect", 1, None)
    try:
        yield app
    finally:
        app.server.stop()


def test_app_mode_target_sliders_and_disconnect(app):
    app.ghost += 0.1
    app.handle("mode", 1, MODES[1])
    np.testing.assert_allclose(app.ghost, app.actual)
    target = app.kin.pose(app.ghost + [0.1, -0.1, 0.1, -0.1, 0.1, 0])
    app.handle("target", 1, target)
    app.step()
    assert app.solution.reachable
    assert np.linalg.norm(app.ghost - app.actual) > 0.05
    app.handle("mode", 1, MODES[2])
    app.handle("joint", 1, (0, 15))
    app.step()
    assert np.rad2deg(app.ghost[0]) == pytest.approx(15)
    assert not app.sliders[0].disabled
    app.handle("disconnect", 1, None)
    assert app.mode == MODES[0]
    app.save_state()
    state = json.loads(Path(app.args.state_file).read_text())
    assert state["demo"] and not state["connected"]


def test_orientation_sliders_keep_target_position(app):
    app.actual = np.array([0.2, -0.4, 0.5, 0.2, -0.3, 0.0])
    app.reset()
    position = app.target[:3, 3].copy()
    app.handle("orientation", 1, (0, np.rad2deg(app.tilt) + 10))
    app.handle("orientation", 1, (1, np.rad2deg(app.twist) + 20))
    app.step()
    assert app.solution.reachable
    np.testing.assert_allclose(app.target[:3, 3], position)
    np.testing.assert_allclose(app.kin.pose(app.ghost)[:3, 3], position, atol=1e-4)
    assert sum(app.ghost[1:4]) == pytest.approx(app.tilt)
    assert app.ghost[4] == pytest.approx(app.twist)


def test_position_tracks_when_requested_orientation_is_unreachable(app):
    app.actual = np.array([0.2, -0.4, 0.5, 0.2, -0.3, 0.0])
    app.reset()
    target = app.kin.pose(app.actual + [0.1, 0, 0, 0, 0, 0])
    app.handle("target", 1, target)
    app.handle("orientation", 1, (0, 90))
    app.step()
    assert app.solution.reachable
    assert app.solution.position_error < 0.005
    assert app.solution.rotation_error > np.deg2rad(15)
    np.testing.assert_allclose(app.kin.pose(app.ghost)[:3, 3], target[:3, 3], atol=0.005)
    assert np.linalg.norm(app.ghost - app.actual) > 0.05


@pytest.mark.parametrize("index", [0, 1])
def test_rotation_ring_requires_drag_and_keeps_position(app, index):
    app.actual = np.array([0.2, -0.4, 0.5, 0.2, -0.3, 0.0])
    app.reset()
    callback = app.rings[index]._impl.drag_cb[0].callback
    original = app.tilt if index == 0 else app.twist
    position = app.target[:3, 3].copy()
    axis = app.kin.rotation_axes(app.request_angles)[index]
    radial = np.cross(axis, [0, 0, 1])
    radial /= np.linalg.norm(radial)
    radius = (0.032, 0.044)[index]
    start = position + radial * radius
    tangent = np.cross(axis, radial)
    end = start + tangent * radius * 0.1

    def send(phase):
        event = SimpleNamespace(client_id=1, phase=phase, start_position=start, end_position=end)
        asyncio.run(callback(event))
        app.step()

    send("update")
    assert (app.tilt if index == 0 else app.twist) == pytest.approx(original)
    send("start")
    assert (app.tilt if index == 0 else app.twist) == pytest.approx(original)
    send("update")
    assert (app.tilt if index == 0 else app.twist) == pytest.approx(original + 0.1)
    np.testing.assert_allclose(app.target[:3, 3], position)
    send("end")
    send("update")
    assert (app.tilt if index == 0 else app.twist) == pytest.approx(original + 0.1)


def test_second_browser_cannot_move_robot_but_can_stop(app):
    app.handle("mode", 1, MODES[2])
    before = app.ghost.copy()
    app.handle("joint", 2, (0, 30))
    np.testing.assert_array_equal(app.ghost, before)
    app.handle("stop", 2, None)
    assert app.mode == MODES[0]


def test_app_ghost_does_not_write_and_unreachable_holds(app, calibration):
    bus = fake_bus(calibration)
    bus.read = lambda: bus.rows
    app.bus = bus
    app.step()
    bus.packet.syncWriteTxOnly.assert_not_called()
    app.handle("mode", 1, MODES[1])
    far = np.eye(4)
    far[:3, 3] = [3, 3, 3]
    app.handle("target", 1, far)
    app.step()
    assert not app.solution.reachable
    np.testing.assert_allclose(app.ghost, app.actual)
    np.testing.assert_allclose(app.kin.pose(app.ghost), app.kin.pose(app.actual))
    params = bus.packet.syncWriteTxOnly.call_args.args[3]
    assert [params[i + 1] + 256 * params[i + 2] for i in range(0, len(params), 7)] == [
        r["Position"] for r in bus.rows
    ]
    bus.rows[0]["Position"] += 50
    app.step()
    np.testing.assert_allclose(app.ghost, app.actual)


def test_target_moves_only_during_drag(app):
    callback = app.handle_point._impl.drag_cb[0].callback
    original = app.target.copy()

    def send(phase, start, end):
        event = SimpleNamespace(client_id=1, phase=phase, start_position=start, end_position=end)
        asyncio.run(callback(event))
        app.step()

    send("update", (0, 0, 0), (1, 1, 1))
    np.testing.assert_allclose(app.target, original)
    send("start", (0.1, 0.2, 0.3), (0.1, 0.2, 0.3))
    np.testing.assert_allclose(app.target, original)
    send("update", (0.1, 0.2, 0.3), (0.12, 0.2, 0.3))
    np.testing.assert_allclose(app.target[:3, 3], original[:3, 3] + [0.02, 0, 0])
    send("end", (0.12, 0.2, 0.3), (0.13, 0.2, 0.3))
    send("update", (0.13, 0.2, 0.3), (1, 1, 1))
    np.testing.assert_allclose(app.target[:3, 3], original[:3, 3] + [0.03, 0, 0])


def test_control_activation_preserves_pose_outside_generic_urdf_limits(app):
    app.actual = np.deg2rad([0, -103.4, 96.6, 26.6, 2.9, 48])
    app.handle("mode", 1, MODES[1])
    np.testing.assert_allclose(app.ghost, app.actual)
    np.testing.assert_allclose(app.target, app.kin.pose(app.actual))
    assert app.solution.position_error < 1e-7


def test_temperature_spike_must_be_confirmed(calibration):
    bus = fake_bus(calibration)
    bus.rows[3]["Temperature"] = 98
    bus.packet.readTxRx.side_effect = [([70], 0, 0), ([33], 0, 0)]
    bus.arm()
    assert bus.armed
    assert bus.rows[3]["Temperature"] == 33
    assert [c.args[2:] for c in bus.packet.readTxRx.call_args_list] == [(13, 1), (63, 1)]


def test_temperature_uses_servo_limit(calibration):
    bus = fake_bus(calibration)
    bus.rows[2]["Temperature"] = 65
    bus.arm()
    assert bus.armed
    bus.packet.readTxRx.assert_not_called()


def test_confirmed_temperature_fault_explains_values(calibration):
    bus = fake_bus(calibration)
    bus.rows[3]["Temperature"] = 75
    with pytest.raises(MotionFault, match="Servo 4: confirmed 75°C ≥ 70°C limit"):
        bus.arm()
    bus.packet.writeTxRx.assert_not_called()
    bus.packet.syncWriteTxOnly.assert_not_called()


def test_sensor_fault_keeps_connection_for_telemetry(app, calibration):
    bus = fake_bus(calibration)
    app.bus = bus
    app.handle("mode", 1, MODES[1])
    app.fault(MotionFault("Servo 4: confirmed 75°C ≥ 70°C limit"))
    assert app.bus is bus
    assert app.mode == MODES[0]
    assert not bus.armed
    bus.port.closePort.assert_not_called()
    assert "confirmed 75°C" in app.message
    assert "Reconnecting" not in app.message


def test_grouped_position_and_full_telemetry_reads(calibration):
    bus = fake_bus(calibration)
    bus.rows = []
    current_address = [0]
    for (address, _), reader in bus.readers.items():
        reader.txPacket = Mock(
            side_effect=lambda address=address: current_address.__setitem__(0, address) or 0
        )
    memory = bytearray(87)
    memory[13] = 70
    memory[56:58] = (2050).to_bytes(2, "little")
    memory[63] = 33
    memory[80] = 1
    bus.packet.readRx.side_effect = lambda port, servo, length: (
        memory[current_address[0] : current_address[0] + length],
        0,
        0,
    )
    bus.read()
    assert len(bus.rows) == 6
    assert bus.rows[0]["Temperature"] == 33
    assert bus.rows[0]["Max temperature"] == 70
    assert bus.rows[0]["Position"] == 2050
    assert bus.rows[0]["Moving velocity threshold"] == 1
    bus.packet.reset_mock()
    memory[63] = 34
    memory[56:58] = (2100).to_bytes(2, "little")
    bus.read()
    assert bus.rows[0]["Temperature"] == 33
    assert bus.rows[0]["Position"] == 2100
    assert bus.rows[0]["Max temperature"] == 70
    assert [c.args[2] for c in bus.packet.readRx.call_args_list] == [2] * 6
    bus.next_telemetry_read = 0
    bus.read()
    assert bus.rows[0]["Temperature"] == 34


def test_short_telemetry_packet_is_rejected(calibration):
    bus = fake_bus(calibration)
    previous = bus.rows
    bus.next_config_read = time.monotonic() + 60
    bus.readers[40, 33].txPacket = Mock(return_value=0)
    bus.packet.readRx.return_value = ([0], 0, 0)
    with pytest.raises(OSError):
        bus.read()
    assert bus.rows is previous


def test_feetech_timeout_matches_lerobot_patch():
    port = Port("/dev/test")
    port.tx_time_per_byte = 0.01
    port.getCurrentTime = lambda: 1234.0
    port.setPacketTimeout(39)
    assert port.packet_start_time == 1234.0
    assert port.packet_timeout == pytest.approx(50.42)


def test_missed_reply_is_reported_directly(calibration):
    bus = fake_bus(calibration)
    bus.packet.readTxRx.side_effect = [([], -6, 0)]
    with pytest.raises(OSError, match="Servo 4"):
        bus.read_registers(4, 63, 1)
    assert bus.packet.readTxRx.call_count == 1
