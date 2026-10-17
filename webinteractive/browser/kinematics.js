import { Matrix4, Vector3, Quaternion } from 'three';
import { Matrix, solve } from 'ml-matrix';
import { names, clamp } from './calibration.js';

export class Kinematics {
  constructor(robot, calibration) {
    this.calibration = calibration;
    this.origins = names.slice(0, 5).map(name => {
      robot.joints[name].updateMatrix();
      return robot.joints[name].matrix.clone();
    });
    this.axes = names.slice(0, 5).map(name => robot.joints[name].axis.clone());
    robot.joints.gripper_frame_joint.updateMatrix();
    this.tool = robot.joints.gripper_frame_joint.matrix.clone();
  }

  pose(angles) {
    const pose = new Matrix4();
    this.origins.forEach((origin, i) => {
      pose.multiply(origin).multiply(new Matrix4().makeRotationAxis(this.axes[i], angles[i]));
    });
    return pose.multiply(this.tool);
  }

  solve(position, tilt, twist, seed, matchRotation) {
    if (![...position.toArray(), tilt, twist, ...seed].every(Number.isFinite)) throw Error('Invalid target');
    const { min, max } = this.calibration;
    const residual = q => {
      const point = new Vector3().setFromMatrixPosition(this.pose(q));
      const result = point.sub(position).toArray();
      if (matchRotation) result.push((q[1] + q[2] + q[3] - tilt) * 0.025, (q[4] - twist) * 0.025);
      return result;
    };
    const fit = (start, match) => {
      let q = start.map((value, i) => clamp(value, min[i], max[i]));
      let damping = 1e-5;
      const error = value => match ? residual(value) : residual(value).slice(0, 3);
      const cost = values => values.reduce((sum, value) => sum + value * value, 0);
      for (let step = 0; step < 100; step++) {
        const r = error(q);
        const columns = q.slice(0, 5).map((_, i) => {
          const moved = [...q];
          moved[i] += 1e-5;
          return error(moved).map((value, j) => (value - r[j]) / 1e-5);
        });
        const jacobian = new Matrix(columns).transpose();
        const transpose = jacobian.transpose();
        const system = transpose.mmul(jacobian).add(Matrix.eye(5).mul(damping));
        const delta = solve(system, transpose.mmul(Matrix.columnVector(r))).to1DArray();
        const next = q.map((value, i) => i === 5 ? value : clamp(value - clamp(delta[i], -0.3, 0.3), min[i], max[i]));
        if (cost(error(next)) < cost(r)) {
          q = next;
          damping = Math.max(1e-8, damping / 2);
        } else damping = Math.min(10, damping * 4);
        if (cost(error(q)) < 1e-10 || damping === 10) break;
      }
      return q;
    };
    let angles = fit(seed, matchRotation);
    let distance = new Vector3().setFromMatrixPosition(this.pose(angles)).distanceTo(position);
    if (distance >= 0.005) {
      const mirror = [...seed];
      mirror[1] *= -1;
      mirror[2] *= -1;
      const alternate = fit(mirror, matchRotation);
      const otherDistance = new Vector3().setFromMatrixPosition(this.pose(alternate)).distanceTo(position);
      if (otherDistance < distance) { angles = alternate; distance = otherDistance; }
    }
    // Position takes priority when this five-axis arm cannot match orientation.
    if (matchRotation && distance >= 0.005) {
      angles = fit(angles, false);
      distance = new Vector3().setFromMatrixPosition(this.pose(angles)).distanceTo(position);
    }
    const desired = [...angles];
    desired[3] = tilt - angles[1] - angles[2];
    desired[4] = twist;
    const rotationError = new Quaternion().setFromRotationMatrix(this.pose(angles))
      .angleTo(new Quaternion().setFromRotationMatrix(this.pose(desired)));
    return { angles, distance, rotationError: matchRotation ? rotationError : 0, reachable: distance < 0.005 };
  }
}
