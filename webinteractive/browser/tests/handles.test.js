import test from 'node:test';
import assert from 'node:assert/strict';
import { Group, Matrix4, PerspectiveCamera, Vector3 } from 'three';
import { TargetHandles } from '../handles.js';

class Canvas extends EventTarget {
  style = {};
  getBoundingClientRect() { return { left: 0, top: 0, width: 800, height: 600 }; }
  setPointerCapture(id) { this.pointer = id; }
  hasPointerCapture(id) { return this.pointer === id; }
  releasePointerCapture() { this.pointer = null; }
  pointerEvent(type, point) {
    const event = new Event(type, { cancelable: true });
    Object.assign(event, { pointerId: 1, button: 0, buttons: type === 'pointerup' ? 0 : 1, clientX: point.x, clientY: point.y });
    this.dispatchEvent(event);
  }
}

function setup() {
  const camera = new PerspectiveCamera(40, 800 / 600, 0.001, 10);
  camera.position.set(0.3, 0.4, 0.5);
  camera.up.set(0, 0, 1);
  camera.lookAt(0, 0, 0);
  camera.updateMatrixWorld();
  const canvas = new Canvas();
  const orbit = { enabled: true };
  const changes = [];
  const handles = new TargetHandles(new Group(), camera, canvas, orbit,
    (name, value) => changes.push({ name, value }), { style: {} });
  const update = (enabled = true, rotation = true) => handles.update(new Vector3(), new Matrix4(),
    [new Vector3(0, 1, 0), new Vector3(0, 0, 1)], { tilt: 0, twist: 0, gripper: 0 },
    { tilt: -1, twist: -1, gripper: 0 }, { tilt: 1, twist: 1, gripper: 1 }, enabled, rotation);
  update();
  return { handles, canvas, orbit, changes, update };
}

for (const name of ['tilt', 'twist', 'gripper']) test(`${name} handle responds to pointer dragging and clamps to its limit`, () => {
  const { handles, canvas, orbit, changes } = setup();
  const mesh = name === 'gripper' ? handles.knob : handles.rings.find(ring => ring.name === name);
  const local = new Vector3(name === 'tilt' ? 0.033 : name === 'twist' ? 0.048 : 0, 0, 0);
  const point = handles.project(mesh.localToWorld(local));
  canvas.pointerEvent('pointerdown', point);
  assert.equal(handles.drag?.name, name);
  assert.equal(orbit.enabled, false);
  const direction = handles.drag.direction;
  canvas.pointerEvent('pointermove', point.clone().addScaledVector(direction, 0.3));
  assert.equal(changes.at(-1).name, name);
  assert.ok(Math.abs(changes.at(-1).value - 0.3) < 1e-8);
  canvas.pointerEvent('pointermove', point.clone().addScaledVector(direction, 10));
  assert.equal(changes.at(-1).value, 1);
  canvas.pointerEvent('pointerup', point);
  assert.equal(handles.drag, null);
  assert.equal(orbit.enabled, true);
  assert.equal(canvas.pointer, null);
});

test('disabling handles cancels a drag and rotation can be disabled independently of the gripper', () => {
  const { handles, canvas, orbit, changes, update } = setup();
  const point = handles.project(handles.knob.getWorldPosition(new Vector3()));
  canvas.pointerEvent('pointerdown', point);
  update(false);
  assert.equal(handles.drag, null);
  assert.equal(orbit.enabled, true);
  canvas.pointerEvent('pointermove', point);
  assert.equal(changes.length, 0);
  update(true, false);
  assert.ok(handles.rings.every(ring => !ring.visible));
  canvas.pointerEvent('pointerdown', point);
  assert.equal(handles.drag?.name, 'gripper');
  canvas.pointerEvent('pointercancel', point);
  assert.equal(orbit.enabled, true);
});
