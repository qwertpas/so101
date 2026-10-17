"""The main application loop is the sole owner of this serial connection."""

import fcntl
import logging
import os
import tempfile
import time
from pathlib import Path

import numpy as np
import scservo_sdk as sdk
import serial
from serial.tools import list_ports

from calibration import TICK, Calibration

LOG = logging.getLogger("so101")


class MotionFault(RuntimeError):
    """Motion must stop, but the connection can keep reporting telemetry."""


# (label, address, byte count, sign-magnitude bit). Values remain raw in the register table.
REGISTERS = [
    ("Firmware major", 0, 1, None),
    ("Firmware minor", 1, 1, None),
    ("Model number", 3, 2, None),
    ("ID", 5, 1, None),
    ("Baud rate", 6, 1, None),
    ("Return delay", 7, 1, None),
    ("Response status level", 8, 1, None),
    ("Min position limit", 9, 2, None),
    ("Max position limit", 11, 2, None),
    ("Max temperature", 13, 1, None),
    ("Max voltage", 14, 1, None),
    ("Min voltage", 15, 1, None),
    ("Max torque", 16, 2, None),
    ("Phase", 18, 1, None),
    ("Unloading condition", 19, 1, None),
    ("LED alarm condition", 20, 1, None),
    ("P gain", 21, 1, None),
    ("D gain", 22, 1, None),
    ("I gain", 23, 1, None),
    ("Minimum startup force", 24, 2, None),
    ("CW dead zone", 26, 1, None),
    ("CCW dead zone", 27, 1, None),
    ("Protection current", 28, 2, None),
    ("Angular resolution", 30, 1, None),
    ("Homing offset", 31, 2, 11),
    ("Operating mode", 33, 1, None),
    ("Protective torque", 34, 1, None),
    ("Protection time", 35, 1, None),
    ("Overload torque", 36, 1, None),
    ("Velocity P gain", 37, 1, None),
    ("Overcurrent protection time", 38, 1, None),
    ("Velocity I gain", 39, 1, None),
    ("Torque enabled", 40, 1, None),
    ("Acceleration", 41, 1, None),
    ("Goal position", 42, 2, 15),
    ("Goal time", 44, 2, None),
    ("Goal velocity", 46, 2, 15),
    ("Torque limit", 48, 2, None),
    ("Lock", 55, 1, None),
    ("Position", 56, 2, 15),
    ("Velocity", 58, 2, 15),
    ("Load", 60, 2, 10),
    ("Voltage", 62, 1, None),
    ("Temperature", 63, 1, None),
    ("Status", 65, 1, None),
    ("Moving", 66, 1, None),
    ("Current", 69, 2, None),
    ("Goal position 2", 71, 2, 15),
    ("Moving velocity threshold", 80, 1, None),
    ("DTs", 81, 1, None),
    ("Velocity unit factor", 82, 1, None),
    ("Hts", 83, 1, None),
    ("Maximum velocity limit", 84, 1, None),
    ("Maximum acceleration", 85, 1, None),
    ("Acceleration multiplier", 86, 1, None),
]


def decode(data, start=0):
    row = {}
    for name, address, size, sign in REGISTERS:
        offset = address - start
        if offset < 0 or offset + size > len(data):
            continue
        value = int.from_bytes(data[offset : offset + size], "little")
        if sign is not None and value & (1 << sign):
            value = -(value & ((1 << sign) - 1))
        row[name] = value
    return row


def candidates(explicit=None):
    if explicit:
        return [explicit]
    # macOS exposes tty/cu aliases; probe each physical adapter only once.
    found = {}
    for p in list_ports.comports():
        if p.vid is not None or any(s in p.device for s in ("usbmodem", "usbserial", "ttyUSB", "ttyACM")):
            key = p.device.replace("/dev/tty.", "/dev/cu.")
            found[key] = p.device
    return sorted(found.values(), key=lambda p: ("5B140335431" not in p, p))


class Port(sdk.PortHandler):
    def setPacketTimeout(self, packet_length):  # noqa: N802
        # Match LeRobot's workaround for the published Feetech SDK timeout bug.
        self.packet_start_time = self.getCurrentTime()
        self.packet_timeout = self.tx_time_per_byte * (packet_length + 3) + 50

    def setupPort(self, baud):
        if self.is_open:
            self.closePort()
        self.ser = serial.Serial(self.port_name, baudrate=baud, timeout=0, exclusive=True)
        self.ser.reset_input_buffer()
        self.tx_time_per_byte = 10000 / baud
        self.is_open = True
        return True


class Bus:
    def __init__(self, path, calibration: Calibration):
        self.path = path
        self.calibration = calibration
        self.port = Port(path)
        self.packet = sdk.PacketHandler(0)
        self.readers = {
            (address, length): sdk.GroupSyncRead(self.port, self.packet, address, length)
            for address, length in ((0, 40), (80, 7), (40, 33), (56, 2))
        }
        for reader in self.readers.values():
            for servo in calibration.ids:
                reader.addParam(servo)
        self.lock = None
        self.rows = []
        self.last_read = 0.0
        self.armed = False
        self.command = None
        self.max_speed = 45.0
        self.next_config_read = 0.0
        self.next_telemetry_read = 0.0

    def connect(self):
        key = Path(self.path).name.removeprefix("tty.").removeprefix("cu.")
        self.lock = open(Path(tempfile.gettempdir()) / f"so101-{os.getuid()}-{key}.lock", "w")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.port.setBaudRate(1_000_000)
            self.read()
            self.calibration.check(self.rows)
        except Exception:
            self.close()
            raise

    def read_registers(self, servo, address, length):
        data, result, error = self.packet.readTxRx(self.port, servo, address, length)
        if result != sdk.COMM_SUCCESS or len(data) != length:
            raise OSError(f"Servo {servo}: {self.packet.getTxRxResult(result)}")
        return decode(bytes(data), address), error

    def read_group(self, address, length):
        reader = self.readers[address, length]
        result = reader.txPacket()
        if result != sdk.COMM_SUCCESS:
            raise OSError(f"Group read @{address}: {self.packet.getTxRxResult(result)}")
        rows = []
        for servo in self.calibration.ids:
            data, result, error = self.packet.readRx(self.port, servo, length)
            if result != sdk.COMM_SUCCESS or len(data) != length:
                raise OSError(f"Servo {servo} group read @{address}: {self.packet.getTxRxResult(result)}")
            rows.append((decode(bytes(data), address), error))
        return rows

    def read(self):
        now = time.monotonic()
        rows = [row.copy() for row in self.rows] if self.rows else [{} for _ in self.calibration.ids]
        refresh = not self.rows or now >= self.next_config_read
        if refresh:
            for address, length in ((0, 40), (80, 7)):
                for row, (values, error) in zip(rows, self.read_group(address, length), strict=True):
                    row.update(values)
                    row["Packet error"] = row.get("Packet error", 0) | error
        # The app reads pose feedback at 50 Hz and full live telemetry at 5 Hz.
        address, length = (40, 33) if not self.rows or now >= self.next_telemetry_read else (56, 2)
        for row, (values, error) in zip(rows, self.read_group(address, length), strict=True):
            row.update(values)
            row["Packet error"] = error
        self.rows = rows
        self.last_read = time.monotonic()
        if refresh:
            self.next_config_read = self.last_read + 5
        if address == 40:
            self.next_telemetry_read = self.last_read + 0.2
        return rows

    @property
    def angles(self):
        return self.calibration.angles([r["Position"] for r in self.rows])

    def check_motion(self):
        if time.monotonic() - self.last_read > 0.25:
            raise OSError("Servo feedback is stale")
        self.calibration.check(self.rows)
        for row in self.rows:
            if row["Packet error"] or row["Status"]:
                raise MotionFault(f"Servo {row['ID']} fault: {row['Status']}, packet {row['Packet error']}")
            if row["Temperature"] >= row["Max temperature"]:
                sample = row["Temperature"]
                # Confirm an alarming bulk sample with independent byte reads.
                for address in (13, 63):
                    confirmed, error = self.read_registers(row["ID"], address, 1)
                    row.update(confirmed)
                    if error:
                        raise MotionFault(f"Servo {row['ID']} temperature read fault: {error}")
                LOG.warning(
                    "Servo %s temperature sample %s°C; confirmed %s°C; limit %s°C",
                    row["ID"],
                    sample,
                    row["Temperature"],
                    row["Max temperature"],
                )
                if row["Max temperature"] <= 0:
                    raise MotionFault(
                        f"Servo {row['ID']} has invalid temperature limit {row['Max temperature']}°C"
                    )
                if row["Temperature"] >= row["Max temperature"]:
                    raise MotionFault(
                        f"Servo {row['ID']}: confirmed {row['Temperature']}°C ≥ {row['Max temperature']}°C limit"
                    )
            if not row["Min voltage"] <= row["Voltage"] <= row["Max voltage"]:
                raise MotionFault(f"Servo {row['ID']} supply voltage is out of range")

    def write(self, servo, address, data):
        result, error = self.packet.writeTxRx(self.port, servo, address, len(data), list(data))
        if result != sdk.COMM_SUCCESS or error:
            raise OSError(f"Servo {servo} write failed: {self.packet.getTxRxResult(result)} / {error}")

    def goals(self, ticks):
        speed = round(np.deg2rad(self.max_speed) / TICK)
        params = []
        for servo, tick in zip(self.calibration.ids, ticks, strict=True):
            # Position, zero move time, and the configured encoder-tick speed cap.
            params.extend([servo, int(tick) & 255, int(tick) >> 8, 0, 0, speed & 255, speed >> 8])
        result = self.packet.syncWriteTxOnly(self.port, 42, 6, params, len(params))
        if result != sdk.COMM_SUCCESS:
            raise OSError(self.packet.getTxRxResult(result))

    def arm(self):
        self.check_motion()
        self.command = self.angles.copy()
        # Clear old goals before enabling any motor, including after partial activation.
        self.goals([r["Position"] for r in self.rows])
        self.armed = True
        for servo in self.calibration.ids:
            # Match LeRobot's STS3215 configuration. A value of 10 takes about
            # two seconds to ramp to 180 deg/s; 254 reaches it in about 80 ms.
            self.write(servo, 41, bytes([254]))
            self.write(servo, 40, bytes([1]))

    def move(self, target, dt):
        self.check_motion()
        if not self.armed:
            raise RuntimeError("Tracking is not armed")
        if not np.all(np.isfinite(target)):
            raise ValueError("Non-finite joint target")
        step = np.deg2rad(self.max_speed) * min(max(dt, 0), 0.05)
        # Do not advance more than 3 degrees ahead of measured feedback.
        self.command = np.clip(self.command, self.angles - np.deg2rad(3), self.angles + np.deg2rad(3))
        self.command += np.clip(target - self.command, -step, step)
        self.goals(self.calibration.ticks(self.command))

    def hold(self):
        if self.armed and self.port.is_open:
            self.goals([r["Position"] for r in self.rows])
            self.command = self.angles.copy()
        self.armed = False

    def release(self):
        for servo in self.calibration.ids:
            self.write(servo, 40, bytes([0]))
        self.armed = False

    def close(self):
        if self.port.is_open:
            self.port.closePort()
        if self.lock:
            self.lock.close()
            self.lock = None
