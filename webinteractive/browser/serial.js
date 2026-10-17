import { registers } from './registers.js';
import { tick, clamp } from './calibration.js';

export function packet(id, instruction, data = []) {
  const body = [id, data.length + 2, instruction, ...data];
  return Uint8Array.from([255, 255, ...body, ~body.reduce((sum, value) => sum + value, 0) & 255]);
}

export function decode(data, start = 0) {
  const row = {};
  for (const [name, address, size, sign] of registers) {
    const offset = address - start;
    if (offset < 0 || offset + size > data.length) continue;
    let value = data[offset] + (size === 2 ? data[offset + 1] * 256 : 0);
    if (sign !== null && (value & (1 << sign))) value = -(value & ((1 << sign) - 1));
    row[name] = value;
  }
  return row;
}

export class Bus {
  constructor(port, calibration) {
    this.port = port;
    this.calibration = calibration;
    this.rows = [];
    this.bytes = [];
    this.armed = false;
    this.maxSpeed = 45;
    this.nextConfig = 0;
    this.nextTelemetry = 0;
    this.lastRead = 0;
    this.command = null;
  }

  async connect() {
    await this.port.open({ baudRate: 1_000_000, bufferSize: 4096 });
    this.reader = this.port.readable.getReader();
    this.writer = this.port.writable.getWriter();
    this.reading = this.receive();
    try {
      await this.read();
      this.calibration.check(this.rows);
    } catch (error) {
      await this.close();
      throw error;
    }
  }

  async receive() {
    try {
      while (true) {
        const { value, done } = await this.reader.read();
        if (done) break;
        this.bytes.push(...value);
        if (this.bytes.length > 4096) throw Error('Serial receive buffer overflow');
        this.wake?.();
      }
    } catch (error) { this.readError = error; }
    finally { this.readError ||= Error('Serial port disconnected'); this.wake?.(); }
  }

  async reply(id, length) {
    const deadline = performance.now() + 60;
    while (true) {
      if (this.readError) throw this.readError;
      while (this.bytes.length > 1 && (this.bytes[0] !== 255 || this.bytes[1] !== 255)) this.bytes.shift();
      if (this.bytes.length >= 4) {
        const size = this.bytes[3] + 4;
        if (size < 6 || size > 255) throw Error('Invalid servo packet length');
        if (this.bytes.length >= size) {
          const response = this.bytes.splice(0, size);
          if ((response.slice(2).reduce((sum, value) => sum + value, 0) & 255) !== 255) throw Error('Servo checksum error');
          if (response[2] !== id || size !== length + 6) throw Error(`Unexpected reply from servo ${response[2]}`);
          return { data: response.slice(5, -1), error: response[4] };
        }
      }
      const remaining = deadline - performance.now();
      if (remaining <= 0) throw Error(`Servo ${id}: reply timeout`);
      await new Promise(resolve => {
        const timer = setTimeout(() => { this.wake = null; resolve(); }, remaining);
        this.wake = () => { clearTimeout(timer); this.wake = null; resolve(); };
      });
    }
  }

  async group(address, length) {
    await this.writer.write(packet(254, 0x82, [address, length, ...this.calibration.ids]));
    const rows = [];
    for (const id of this.calibration.ids) {
      const result = await this.reply(id, length);
      rows.push({ ...decode(result.data, address), 'Packet error': result.error });
    }
    return rows;
  }

  async read() {
    const now = performance.now();
    const rows = this.rows.length ? this.rows.map(row => ({ ...row })) : this.calibration.ids.map(() => ({}));
    if (now >= this.nextConfig) {
      for (const [address, length] of [[0, 40], [80, 7]]) {
        (await this.group(address, length)).forEach((row, i) => {
          if (row['Packet error']) throw Error(`Servo ${this.calibration.ids[i]}: configuration read fault`);
          Object.assign(rows[i], row);
        });
      }
      this.nextConfig = now + 5000;
    }
    const telemetry = now >= this.nextTelemetry;
    (await this.group(telemetry ? 40 : 56, telemetry ? 33 : 2)).forEach((row, i) => Object.assign(rows[i], row));
    this.rows = rows;
    this.lastRead = performance.now();
    if (telemetry) this.nextTelemetry = now + 200;
    return rows;
  }

  get angles() { return this.calibration.angles(this.rows.map(row => row.Position)); }

  async write(id, address, data) {
    await this.writer.write(packet(id, 3, [address, ...data]));
    const result = await this.reply(id, 0);
    if (result.error) throw Error(`Servo ${id}: write error ${result.error}`);
  }

  async goals(ticks) {
    const speed = Math.round(this.maxSpeed * Math.PI / 180 / tick);
    const data = [42, 6];
    this.calibration.ids.forEach((id, i) => data.push(id, ticks[i] & 255, ticks[i] >> 8, 0, 0, speed & 255, speed >> 8));
    await this.writer.write(packet(254, 0x83, data));
  }

  async checkMotion() {
    if (performance.now() - this.lastRead > 250) throw Error('Servo feedback is stale');
    this.calibration.check(this.rows);
    for (const row of this.rows) {
      if (row['Packet error'] || row.Status) throw Error(`Servo ${row.ID}: hardware fault`);
      if (row.Temperature >= row['Max temperature']) {
        for (const address of [13, 63]) {
          await this.writer.write(packet(row.ID, 2, [address, 1]));
          const result = await this.reply(row.ID, 1);
          if (result.error) throw Error(`Servo ${row.ID}: temperature read fault`);
          Object.assign(row, decode(result.data, address));
        }
        if (row['Max temperature'] <= 0 || row.Temperature >= row['Max temperature']) throw Error(`Servo ${row.ID}: temperature limit`);
      }
      if (row.Voltage < row['Min voltage'] || row.Voltage > row['Max voltage']) throw Error(`Servo ${row.ID}: voltage limit`);
    }
  }

  async arm() {
    await this.checkMotion();
    this.command = this.angles;
    await this.goals(this.rows.map(row => row.Position));
    this.armed = true;
    for (const id of this.calibration.ids) {
      await this.write(id, 41, [254]);
      await this.write(id, 40, [1]);
    }
  }

  async move(target, dt) {
    await this.checkMotion();
    if (!this.armed) throw Error('Motion is not armed');
    this.calibration.ticks(target); // Validate before changing the command.
    const step = this.maxSpeed * Math.PI / 180 * clamp(dt, 0, 0.05);
    this.command = this.command.map((value, i) => {
      const near = clamp(value, this.angles[i] - Math.PI / 60, this.angles[i] + Math.PI / 60);
      return near + clamp(target[i] - near, -step, step);
    });
    await this.goals(this.calibration.ticks(this.command));
  }

  async hold() {
    if (this.armed) await this.goals(this.rows.map(row => row.Position));
    this.armed = false;
  }

  async release() {
    await this.hold();
    for (const id of this.calibration.ids) await this.write(id, 40, [0]);
  }

  async close() {
    if (this.reader) {
      await this.reader.cancel().catch(() => {});
      await this.reading;
      this.reader.releaseLock();
      this.writer.releaseLock();
      this.reader = null;
    }
    await this.port.close();
  }
}
