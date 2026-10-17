export const names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper'];
export const tick = 2 * Math.PI / 4095;
export const clamp = (value, low, high) => Math.max(low, Math.min(high, value));

export class Calibration {
  constructor(data) {
    this.data = data;
    this.ids = names.map(name => data[name]?.id);
    if (new Set(this.ids).size !== 6 || this.ids.some(id => !Number.isInteger(id) || id < 1 || id > 253)) {
      throw Error('Calibration must contain six unique servo IDs');
    }
    this.low = names.map(name => data[name].range_min);
    this.high = names.map(name => data[name].range_max);
    names.forEach((name, i) => {
      const row = data[name];
      if (row.drive_mode !== 0 || !Number.isInteger(row.homing_offset) || Math.abs(row.homing_offset) > 2047 ||
          !Number.isInteger(this.low[i]) || !Number.isInteger(this.high[i]) ||
          this.low[i] < 0 || this.high[i] > 4095 || this.low[i] >= this.high[i]) {
        throw Error(`Invalid calibration for ${name}`);
      }
    });
    this.zero = this.low.map((value, i) => i === 5 ? value : (value + this.high[i]) / 2);
    this.min = this.angles(this.low);
    this.max = this.angles(this.high);
  }

  angles(raw) { return raw.map((value, i) => (value - this.zero[i]) * tick); }

  ticks(angles) {
    if (angles.length !== 6 || !angles.every(Number.isFinite)) throw Error('Invalid joint target');
    return angles.map((angle, i) => clamp(Math.round(angle / tick + this.zero[i]), this.low[i], this.high[i]));
  }

  check(rows) {
    if (rows.length !== 6) throw Error('Six servos are required');
    rows.forEach((row, i) => {
      const saved = this.data[names[i]];
      if (row['ID'] !== saved.id || row['Model number'] !== 777 || row['Operating mode'] !== 0) {
        throw Error(`${names[i]}: expected STS3215 in position mode`);
      }
      for (const [key, field] of [['homing_offset', 'Homing offset'], ['range_min', 'Min position limit'], ['range_max', 'Max position limit']]) {
        if (row[field] !== saved[key]) throw Error(`${names[i]}: ${field} differs from calibration`);
      }
    });
  }
}
