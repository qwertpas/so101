import { Group, Mesh, MeshBasicMaterial, TorusGeometry, SphereGeometry, CylinderGeometry, Vector2, Vector3, Raycaster } from 'three';
import { clamp } from './calibration.js';

const zAxis = new Vector3(0, 0, 1);
const railLength = 0.065;

export class TargetHandles {
  constructor(scene, camera, canvas, orbit, change, hint) {
    this.camera = camera;
    this.canvas = canvas;
    this.orbit = orbit;
    this.change = change;
    this.hint = hint;
    this.group = new Group();
    this.group.visible = false;
    scene.add(this.group);
    this.rings = ['tilt', 'twist'].map((name, i) => {
      const ring = new Mesh(new TorusGeometry(0.033 + i * 0.015, 0.0025, 10, 80),
        new MeshBasicMaterial({ color: i ? 0xc777ff : 0xffae3d, depthTest: false }));
      ring.name = name;
      ring.renderOrder = 10;
      this.group.add(ring);
      return ring;
    });
    this.grip = new Group();
    this.group.add(this.grip);
    const green = new MeshBasicMaterial({ color: 0x7ce1a4, depthTest: false });
    const rail = new Mesh(new CylinderGeometry(0.0015, 0.0015, railLength, 8), green);
    rail.rotation.z = -Math.PI / 2;
    rail.position.x = railLength / 2;
    rail.renderOrder = 10;
    this.knob = new Mesh(new SphereGeometry(0.0065), green);
    this.knob.name = 'gripper';
    this.knob.renderOrder = 11;
    this.grip.add(rail, this.knob);
    this.raycaster = new Raycaster();
    canvas.addEventListener('pointerdown', event => this.down(event), true);
    canvas.addEventListener('pointermove', event => this.move(event), true);
    canvas.addEventListener('pointerup', event => this.up(event), true);
    canvas.addEventListener('pointercancel', () => this.cancel(), true);
    canvas.addEventListener('lostpointercapture', () => this.cancel(), true);
    canvas.addEventListener('pointerleave', () => { if (!this.drag) this.showHint(); });
  }

  update(position, pose, axes, values, min, max, enabled, rotationEnabled) {
    this.group.visible = enabled;
    this.rings.forEach((ring, i) => {
      ring.visible = rotationEnabled;
      ring.quaternion.setFromUnitVectors(zAxis, axes[i]);
    });
    this.group.position.copy(position);
    this.grip.quaternion.setFromRotationMatrix(pose);
    this.grip.position.set(0.065, 0, 0).applyQuaternion(this.grip.quaternion);
    this.knob.position.x = railLength * (values.gripper - min.gripper) / (max.gripper - min.gripper);
    this.values = values;
    this.min = min;
    this.max = max;
    if (!enabled || (this.drag?.name !== 'gripper' && !rotationEnabled)) this.cancel();
    this.group.updateMatrixWorld(true);
  }

  project(point) {
    const size = this.canvas.getBoundingClientRect();
    const projected = point.clone().project(this.camera);
    return new Vector2((projected.x + 1) * size.width / 2, (1 - projected.y) * size.height / 2);
  }

  pick(event) {
    if (!this.group.visible) return;
    const rect = this.canvas.getBoundingClientRect();
    this.raycaster.setFromCamera(new Vector2(
      (event.clientX - rect.left) / rect.width * 2 - 1,
      1 - (event.clientY - rect.top) / rect.height * 2,
    ), this.camera);
    return this.raycaster.intersectObjects([...this.rings.filter(ring => ring.visible), this.knob], false)[0];
  }

  down(event) {
    if (event.button !== 0 || this.drag) return;
    const hit = this.pick(event);
    if (!hit) return;
    const name = hit.object.name;
    let direction;
    if (name === 'gripper') {
      const start = this.grip.getWorldPosition(new Vector3());
      const end = this.grip.localToWorld(new Vector3(railLength, 0, 0));
      direction = this.project(end).sub(this.project(start)).divideScalar(this.max.gripper - this.min.gripper);
    } else {
      const axis = zAxis.clone().applyQuaternion(hit.object.quaternion);
      const radius = hit.point.clone().sub(this.group.position);
      const tangent = axis.cross(radius);
      direction = this.project(hit.point.clone().add(tangent)).sub(this.project(hit.point));
    }
    if (direction.length() < 8) return;
    this.drag = { name, pointer: event.pointerId, start: new Vector2(event.clientX, event.clientY), value: this.values[name], direction };
    event.stopImmediatePropagation();
    event.preventDefault();
    this.orbit.enabled = false;
    this.canvas.setPointerCapture(event.pointerId);
    this.showHint(name, event);
  }

  move(event) {
    if (!this.drag) {
      if (event.buttons === 0) this.showHint(this.pick(event)?.object.name, event);
      return;
    }
    if (event.pointerId !== this.drag.pointer) return;
    event.stopImmediatePropagation();
    const { name, start, value, direction } = this.drag;
    const delta = new Vector2(event.clientX, event.clientY).sub(start).dot(direction) / direction.lengthSq();
    this.change(name, clamp(value + delta, this.min[name], this.max[name]));
    this.showHint(name, event);
  }

  up(event) {
    if (!this.drag || event.pointerId !== this.drag.pointer) return;
    event.stopImmediatePropagation();
    this.cancel();
  }

  cancel() {
    if (!this.drag) return;
    const pointer = this.drag.pointer;
    this.drag = null;
    if (this.canvas.hasPointerCapture(pointer)) this.canvas.releasePointerCapture(pointer);
    this.orbit.enabled = true;
    this.showHint();
  }

  showHint(name, event) {
    this.hint.hidden = !name;
    this.canvas.style.cursor = name ? this.drag ? 'grabbing' : 'grab' : '';
    if (!name) return;
    this.hint.textContent = `${name[0].toUpperCase() + name.slice(1)} · ${(this.values[name] * 180 / Math.PI).toFixed(1)}°`;
    const rect = this.canvas.getBoundingClientRect();
    this.hint.style.left = `${Math.min(event.clientX - rect.left + 15, rect.width - 155)}px`;
    this.hint.style.top = `${Math.max(0, event.clientY - rect.top - 32)}px`;
  }
}
