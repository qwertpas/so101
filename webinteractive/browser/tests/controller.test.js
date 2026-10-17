import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { DOMParser } from '@xmldom/xmldom';
import { Matrix4, Vector3, Quaternion, Euler } from 'three';
import { Calibration, names, tick } from '../calibration.js';
import { Kinematics } from '../kinematics.js';
import { Bus, packet, decode } from '../serial.js';
import { registers } from '../registers.js';

const fixtures = JSON.parse(readFileSync(new URL('./poses.json', import.meta.url)));
const calibration = new Calibration(fixtures.calibration);

// Simulate the adapter at the byte-stream boundary; exercise the real driver.
class Adapter {
  constructor() {
    this.writes = [];
    this.rows = names.map((name, i) => ({
      ID: i + 1, 'Model number': 777, 'Operating mode': 0,
      'Homing offset': fixtures.calibration[name].homing_offset,
      'Min position limit': calibration.low[i], 'Max position limit': calibration.high[i],
      Position: Math.round(calibration.zero[i]), Temperature: 25, 'Max temperature': 70,
      Voltage: 120, 'Min voltage': 40, 'Max voltage': 140, Status: 0,
    }));
  }
  async open(options) {
    assert.equal(options.baudRate, 1_000_000);
    this.readable = new ReadableStream({ start: controller => { this.input = controller; } });
    this.writable = new WritableStream({ write: bytes => this.request([...bytes]) });
  }
  async close() { this.closed = true; }
  bytes(row, address, length) {
    const data = Array(87).fill(0);
    for (const [name, offset, size, sign] of registers) {
      let value = row[name] ?? 0;
      if (value < 0) value = -value | (1 << sign);
      data[offset] = value & 255;
      if (size === 2) data[offset + 1] = value >> 8;
    }
    return data.slice(address, address + length);
  }
  request(bytes) {
    this.writes.push(bytes);
    const [,, id,, instruction, ...body] = bytes;
    const data = body.slice(0, -1);
    if (instruction === 0x83) return;
    const rows = instruction === 0x82 ? data.slice(2).map(id => this.rows[id - 1]) : [this.rows[id - 1]];
    for (const row of rows) {
      if (this.missing === row.ID) continue;
      if (instruction === 3) {
        const field = registers.find(([, address]) => address === data[0])[0];
        row[field] = data[1];
      }
      const reply = [...packet(row.ID, this.error ?? 0, instruction === 3 ? [] : this.bytes(row, data[0], data[1]))];
      if (this.corrupt) reply[reply.length - 1] ^= 1;
      // Header and body fragmentation verifies buffering across read() calls.
      this.input.enqueue(Uint8Array.from(reply.slice(0, 3)));
      this.input.enqueue(Uint8Array.from(reply.slice(3)));
    }
  }
}

test('connect verifies six servos without any writes to motor registers', async () => {
  const adapter = new Adapter();
  const bus = new Bus(adapter, calibration);
  await bus.connect();
  assert.equal(bus.rows.length, 6);
  assert.ok(adapter.writes.every(bytes => bytes[4] === 0x82));
  assert.equal(bus.armed, false);
  await bus.close();
  assert.ok(adapter.closed);
});

test('motion clears old goals before torque enable, applies speed/feedback limits, then holds and releases', async () => {
  const adapter = new Adapter();
  const bus = new Bus(adapter, calibration);
  await bus.connect();
  const start = adapter.writes.length;
  await bus.arm();
  assert.equal(adapter.writes[start][4], 0x83);
  assert.equal(adapter.writes[start + 1][5], 41);
  assert.equal(adapter.writes[start + 2][5], 40);
  const before = [...bus.command];
  await bus.move([1, 1, 1, 1, 1, 1], 0.02);
  assert.ok(bus.command.every((value, i) => Math.abs(value - before[i]) <= Math.PI / 4 * 0.02 + 1e-9));
  bus.command = [10, 10, 10, 10, 10, 10];
  await bus.move([1, 1, 1, 1, 1, 1], 1);
  assert.ok(bus.command.every((value, i) => Math.abs(value - bus.angles[i]) <= Math.PI / 60 + Math.PI / 4 * 0.05 + 1e-9));
  await bus.hold();
  assert.equal(bus.armed, false);
  const goal = adapter.writes.at(-1);
  assert.equal(goal[4], 0x83);
  assert.equal(goal[8] + goal[9] * 256, bus.rows[0].Position);
  await bus.release();
  assert.ok(adapter.rows.every(row => row['Torque enabled'] === 0));
  await bus.close();
});

test('bad calibration, missing servo, checksum errors, and servo faults reject the connection', async () => {
  for (const fault of ['calibration', 'missing', 'corrupt', 'error']) {
    const adapter = new Adapter();
    if (fault === 'calibration') adapter.rows[0]['Homing offset'] += 1;
    if (fault === 'missing') adapter.missing = 3;
    if (fault === 'corrupt') adapter.corrupt = true;
    if (fault === 'error') adapter.error = 4;
    await assert.rejects(new Bus(adapter, calibration).connect());
    assert.ok(adapter.closed);
    assert.ok(adapter.writes.every(bytes => bytes[4] === 0x82));
  }
});

test('stale feedback, temperature, voltage, status, and nonfinite targets prevent motion', async () => {
  const adapter = new Adapter();
  const bus = new Bus(adapter, calibration);
  await bus.connect();
  await bus.arm();
  const count = adapter.writes.length;
  await assert.rejects(bus.move([NaN, 0, 0, 0, 0, 0], 0.02));
  assert.equal(adapter.writes.length, count);
  bus.lastRead -= 1000;
  await assert.rejects(bus.move([0, 0, 0, 0, 0, 0], 0.02), /stale/);
  await bus.read();
  bus.rows[0].Status = 1;
  await assert.rejects(bus.checkMotion(), /fault/);
  bus.rows[0].Status = 0;
  bus.rows[0].Voltage = 255;
  await assert.rejects(bus.checkMotion(), /voltage/);
  bus.rows[0].Voltage = 120;
  bus.rows[0].Temperature = adapter.rows[0].Temperature = 80;
  await assert.rejects(bus.checkMotion(), /temperature/);
  await bus.hold();
  await bus.close();
});

test('sign-magnitude telemetry and calibrated conversion match Python conventions', () => {
  const data = Array(87).fill(0);
  for (const [address, value] of [[31, 0x800 | 641], [56, 0x8000 | 200], [58, 0x8000 | 23], [60, 0x400 | 10]]) {
    data[address] = value & 255;
    data[address + 1] = value >> 8;
  }
  const row = decode(data);
  assert.deepEqual([row['Homing offset'], row.Position, row.Velocity, row.Load], [-641, -200, -23, -10]);
  const raw = [700, 1234, 2048, 2750, 3100, 2100];
  assert.deepEqual(calibration.ticks(calibration.angles(raw)), raw);
  assert.equal(calibration.angles(calibration.low)[5], 0);
  assert.equal(tick, 2 * Math.PI / 4095);
  assert.throws(() => new Calibration({ ...fixtures.calibration, gripper: { ...fixtures.calibration.gripper, id: 1 } }));
});

function model() {
  const doc = new DOMParser().parseFromString(readFileSync(new URL('../../model/so101.urdf', import.meta.url), 'utf8'), 'text/xml');
  const joints = {};
  for (const node of Array.from(doc.getElementsByTagName('joint'))) {
    if (!node.hasAttribute('type')) continue;
    const origin = node.getElementsByTagName('origin')[0];
    const xyz = (origin?.getAttribute('xyz') || '0 0 0').split(/\s+/).map(Number);
    const rpy = (origin?.getAttribute('rpy') || '0 0 0').split(/\s+/).map(Number);
    const axis = node.getElementsByTagName('axis')[0];
    joints[node.getAttribute('name')] = {
      matrix: new Matrix4().compose(new Vector3(...xyz), new Quaternion().setFromEuler(new Euler(...rpy, 'ZYX')), new Vector3(1, 1, 1)),
      axis: new Vector3(...(axis?.getAttribute('xyz') || '1 0 0').split(/\s+/).map(Number)),
      updateMatrix() {},
    };
  }
  return { joints };
}

test('browser forward kinematics matches Python/scipy reference matrices', () => {
  const kin = new Kinematics(model(), calibration);
  for (const { angles, matrix } of fixtures.poses) {
    const result = kin.pose(angles).elements;
    matrix.forEach((row, i) => row.forEach((value, j) => assert.ok(Math.abs(result[j * 4 + i] - value) < 1e-9)));
  }
});

test('bounded IK reaches known poses and rejects unreachable positions', () => {
  const kin = new Kinematics(model(), calibration);
  for (const { angles } of fixtures.poses) {
    const position = new Vector3().setFromMatrixPosition(kin.pose(angles));
    const result = kin.solve(position, angles[1] + angles[2] + angles[3], angles[4], [0, 0, 0, 0, 0, 0], true);
    assert.ok(result.reachable, `IK residual ${result.distance}`);
    assert.ok(result.angles.every((value, i) => value >= calibration.min[i] && value <= calibration.max[i]));
  }
  assert.equal(kin.solve(new Vector3(10, 10, 10), 0, 0, [0, 0, 0, 0, 0, 0], false).reachable, false);
});
