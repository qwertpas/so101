import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import { Bus } from './serial.js';
import { Calibration, names, clamp } from './calibration.js';
import { Kinematics } from './kinematics.js';
import { TargetHandles } from './handles.js';
import { registers } from './registers.js';
import defaultCalibration from '../calibrations/arm1.json';
import './style.css';

const $ = id => document.getElementById(id);
const degrees = value => value * 180 / Math.PI;
const radians = value => value * Math.PI / 180;
const labels = names.map(name => name.replaceAll('_', ' '));
let calibration, kin, robot, ghost, bus, ports = [], demo = false, mode = 'ghost';
let actual = [0, 0, 0, 0, 0, 0], target = [...actual], tilt = 0, twist = 0, reachable = true;
let dirty = false, connected = false, busy = false, lastStep = performance.now(), nextDisplay = 0;
const actions = [];
const status = (message, error = false) => { $('status').textContent = message; $('status').dataset.error = error; };
const enqueue = action => actions.push(action);

const viewer = $('viewer');
const scene = new THREE.Scene();
scene.background = new THREE.Color('#121a22');
const camera = new THREE.PerspectiveCamera(40, 1, 0.001, 10);
camera.up.set(0, 0, 1);
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
viewer.append(renderer.domElement);
const orbit = new OrbitControls(camera, renderer.domElement);
function home() { camera.position.set(0.65, 0.65, 0.45); orbit.target.set(0, -0.06, 0.16); orbit.update(); }
home();
$('home').onclick = home;
scene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 2.8));
const light = new THREE.DirectionalLight(0xffffff, 2);
light.position.set(1, 2, 3);
scene.add(light);
const grid = new THREE.GridHelper(1.4, 28, 0x526475, 0x2e3b49);
grid.rotation.x = Math.PI / 2;
grid.position.z = -0.004;
scene.add(grid, new THREE.AxesHelper(0.07));
const handle = new THREE.Mesh(new THREE.SphereGeometry(0.008), new THREE.MeshBasicMaterial({ color: 0x45cbdc }));
scene.add(handle);
const transform = new TransformControls(camera, renderer.domElement);
transform.setSize(0.65);
transform.attach(handle);
scene.add(transform.getHelper());
transform.addEventListener('dragging-changed', event => { orbit.enabled = !event.value; });
transform.addEventListener('objectChange', () => { dirty = true; updateCoordinates(); });
const targetHandles = new TargetHandles(scene, camera, renderer.domElement, orbit, (name, value) => {
  if (name === 'gripper') target[5] = value;
  else {
    if (name === 'tilt') tilt = value;
    else twist = value;
    dirty = true;
  }
}, $('handle-hint'));
new ResizeObserver(() => {
  const { width, height } = viewer.getBoundingClientRect();
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
}).observe(viewer);
renderer.setAnimationLoop(() => {
  if (kin) {
    const requested = [...target];
    requested[3] = tilt - target[1] - target[2];
    requested[4] = twist;
    targetHandles.update(handle.position, kin.pose(requested), kin.rotationAxes(requested),
      { tilt, twist, gripper: target[5] },
      { tilt: radians(Number($('tilt').min)), twist: calibration.min[4], gripper: calibration.min[5] },
      { tilt: radians(Number($('tilt').max)), twist: calibration.max[4], gripper: calibration.max[5] },
      mode !== 'joint' && !busy, $('match').checked);
  }
  orbit.update();
  renderer.render(scene, camera);
});

for (const [i, label] of labels.entries()) {
  const row = document.createElement('div');
  row.innerHTML = `<label for="joint-${i}">${label}<output id="joint-value-${i}"></output></label><input id="joint-${i}" type="range" step="0.1" disabled>`;
  $('joints').append(row);
  $(`joint-${i}`).oninput = event => {
    if (mode !== 'joint') return;
    target[i] = radians(Number(event.target.value));
    syncTarget();
  };
}

function updateCoordinates() {
  ['x', 'y', 'z'].forEach(axis => { $(axis).value = (handle.position[axis] * 1000).toFixed(1); });
}

function syncTarget() {
  if (!kin) return;
  handle.position.setFromMatrixPosition(kin.pose(target));
  tilt = target[1] + target[2] + target[3];
  twist = target[4];
  reachable = true;
  dirty = false;
  $('ik').textContent = 'Reachable';
  updateCoordinates();
}

function configure(data, label) {
  calibration = new Calibration(data);
  if (kin) kin.calibration = calibration;
  else kin = new Kinematics(robot, calibration);
  actual = actual.map((value, i) => clamp(value, calibration.min[i], calibration.max[i]));
  target = [...actual];
  names.forEach((_, i) => {
    $(`joint-${i}`).min = degrees(calibration.min[i]);
    $(`joint-${i}`).max = degrees(calibration.max[i]);
  });
  for (const [id, low, high] of [
    ['tilt', calibration.min.slice(1, 4).reduce((a, b) => a + b), calibration.max.slice(1, 4).reduce((a, b) => a + b)],
    ['twist', calibration.min[4], calibration.max[4]], ['gripper', calibration.min[5], calibration.max[5]],
  ]) { $(id).min = degrees(low); $(id).max = degrees(high); }
  $('calibration-name').textContent = label;
  syncTarget();
}

function setMode(value) {
  targetHandles.cancel();
  mode = value;
  document.querySelector(`input[name=mode][value=${mode}]`).checked = true;
}

async function stop(message = 'Holding position · ghost only') {
  setMode('ghost');
  if (bus) await bus.hold();
  status(message);
}

async function disconnect() {
  await stop('Disconnected');
  const previous = bus;
  bus = null;
  connected = false;
  if (previous) await previous.close();
}

async function fail(error) {
  setMode('ghost');
  if (bus) {
    try { await bus.hold(); } catch (holdError) { console.error('Hold failed', holdError); }
    const previous = bus;
    bus = null;
    connected = false;
    try { await previous.close(); } catch (closeError) { console.error(closeError); }
  }
  status(error.message, true);
  console.error(error);
}

async function refreshPorts() {
  if (!navigator.serial) return;
  const selected = ports[Number($('ports').value)];
  ports = await navigator.serial.getPorts();
  $('ports').replaceChildren();
  ports.forEach((port, i) => {
    const info = port.getInfo();
    const usb = info.usbVendorId === undefined ? 'Serial device' : `USB ${info.usbVendorId.toString(16).padStart(4, '0')}:${info.usbProductId.toString(16).padStart(4, '0')}`;
    $('ports').add(new Option(`${usb} · port ${i + 1}`, String(i)));
  });
  if (!ports.length) $('ports').add(new Option('No approved ports', ''));
  if (ports.includes(selected)) $('ports').value = String(ports.indexOf(selected));
  updateControls();
}

function updateControls() {
  const ready = !!kin;
  $('ports').disabled = connected || busy || !ports.length;
  $('choose').disabled = connected || busy || !navigator.serial;
  $('connect').disabled = connected || busy || demo || !ports.length || !calibration;
  $('disconnect').disabled = !connected || busy;
  $('calibration').disabled = connected || busy;
  $('demo').disabled = connected || busy || !ready;
  $('modes').disabled = !ready || busy;
  for (const input of document.querySelectorAll('input[name=mode]')) input.disabled = input.value !== 'ghost' && !connected && !demo;
  $('release').disabled = !connected || busy;
  $('target-controls').querySelectorAll('input, button').forEach(input => { input.disabled = !ready || mode === 'joint'; });
  $('tilt').disabled ||= !$('match').checked;
  $('twist').disabled ||= !$('match').checked;
  transform.enabled = ready && mode !== 'joint';
  transform.getHelper().visible = ready && mode !== 'joint';
  names.forEach((_, i) => { $(`joint-${i}`).disabled = mode !== 'joint'; });
}

$('choose').onclick = async () => {
  try {
    const port = await navigator.serial.requestPort();
    await refreshPorts();
    $('ports').value = String(ports.indexOf(port));
    status('Port selected · click Connect');
  } catch (error) { if (error.name !== 'NotFoundError') status(error.message, true); }
};
$('connect').onclick = () => {
  const port = ports[Number($('ports').value)];
  enqueue(async () => {
    if (connected || !port || !calibration || demo) return;
    status('Connecting at 1 Mbps · checking six servos…');
    const connection = new Bus(port, calibration);
    await connection.connect();
    bus = connection;
    connected = true;
    actual = bus.angles;
    target = [...actual];
    setMode('ghost');
    syncTarget();
    status('Connected · 6/6 calibrated servos · ghost only');
  });
};
$('disconnect').onclick = () => enqueue(disconnect);
$('stop').onclick = () => { setMode('ghost'); actions.length = 0; enqueue(() => stop()); };
$('release').onclick = () => enqueue(async () => { await stop(); if (bus) await bus.release(); status('Torque released'); });
$('reset').onclick = () => { target = [...actual]; syncTarget(); };
$('speed').oninput = () => { $('speed-value').textContent = `${$('speed').value}°/s`; };
$('show-ghost').onchange = () => { if (ghost) ghost.visible = $('show-ghost').checked; };
for (const axis of ['x', 'y', 'z']) $(axis).onchange = () => {
  const value = Number($(axis).value);
  if (Number.isFinite(value)) { handle.position[axis] = value / 1000; dirty = true; }
};
$('match').onchange = () => { dirty = true; updateControls(); };
$('tilt').oninput = () => { tilt = radians(Number($('tilt').value)); dirty = true; };
$('twist').oninput = () => { twist = radians(Number($('twist').value)); dirty = true; };
$('gripper').oninput = () => { target[5] = radians(Number($('gripper').value)); };
document.querySelectorAll('input[name=mode]').forEach(input => {
  input.onchange = () => {
    const requested = input.value;
    enqueue(async () => {
      await stop();
      if (requested === 'ghost') return;
      if ((!bus && !demo) || document.hidden) throw Error('Connect the calibrated robot before enabling motion');
      target = [...actual];
      syncTarget();
      if (bus) await bus.arm();
      setMode(requested);
      status(`${demo ? 'Demo · ' : ''}${requested === 'joint' ? 'Joint control' : 'Track end effector'} active`);
    });
  };
});

$('calibration').onchange = async () => {
  const file = $('calibration').files[0];
  if (!file) return;
  try {
    const data = JSON.parse(await file.text());
    new Calibration(data);
    enqueue(async () => {
      if (connected) throw Error('Disconnect before changing calibration');
      configure(data, file.name);
      try { localStorage.setItem('so101-calibration', JSON.stringify({ data, name: file.name })); }
      catch { status('Calibration loaded; browser storage is unavailable'); return; }
      status('Calibration saved in this browser · select a port and connect');
    });
  } catch (error) { status(error.message, true); }
};
$('demo').onchange = () => enqueue(async () => {
  await stop();
  demo = $('demo').checked;
  target = [...actual];
  syncTarget();
  status(demo ? 'Demo · simulated feedback · ghost only' : 'Select a serial port to connect');
});
document.addEventListener('visibilitychange', () => {
  if (document.hidden) { setMode('ghost'); actions.length = 0; enqueue(() => stop('Tab hidden · holding position')); }
});
window.addEventListener('pagehide', () => { setMode('ghost'); enqueue(disconnect); });

function display() {
  if (!kin) return;
  for (const [i, name] of names.entries()) {
    robot.setJointValue(name, actual[i]);
    ghost.setJointValue(name, target[i]);
    $(`joint-${i}`).value = degrees(target[i]);
    $(`joint-value-${i}`).textContent = `${degrees(target[i]).toFixed(1)}°`;
  }
  robot.visible = connected || demo;
  for (const [id, value] of [['tilt', tilt], ['twist', twist], ['gripper', target[5]]]) {
    $(id).value = degrees(value);
    $(`${id}-value`).textContent = `${degrees(value).toFixed(1)}°`;
  }
  $('telemetry').innerHTML = labels.map((label, i) => {
    const row = bus?.rows[i];
    return `<tr><td>${label}</td><td>${degrees(actual[i]).toFixed(1)}°</td><td>${row ? `${row.Temperature}°C` : '—'}</td><td>${row ? row['Torque enabled'] ? 'On' : 'Off' : '—'}</td></tr>`;
  }).join('');
  if ($('registers').open) {
    const open = new Set([...$('register-values').querySelectorAll('details[open]')].map(node => node.dataset.id));
    $('register-values').innerHTML = labels.map((label, i) => `<details data-id="${i}" ${open.has(String(i)) ? 'open' : ''}><summary>${label}</summary><table><thead><tr><th>Register</th><th>Address</th><th>Value</th></tr></thead><tbody>${registers.map(([name, address]) => `<tr><td>${name}</td><td>${address}</td><td>${bus?.rows[i]?.[name] ?? '—'}</td></tr>`).join('')}</tbody></table></details>`).join('');
  }
  updateControls();
}

async function step() {
  const started = performance.now();
  const dt = (started - lastStep) / 1000;
  lastStep = started;
  try {
    if (bus) { await bus.read(); actual = bus.angles; }
    if (actions.length) {
      busy = true;
      updateControls();
      while (actions.length) await actions.shift()();
      busy = false;
      updateControls();
    }
    if (kin && dirty) {
      const solution = kin.solve(handle.position, tilt, twist, target, $('match').checked);
      reachable = solution.reachable;
      if (reachable) target = solution.angles;
      else target = [...actual];
      $('ik').textContent = `${reachable ? solution.rotationError >= radians(15) ? 'Position reached · orientation limited' : 'Reachable' : 'Position unreachable · holding'} · ${(solution.distance * 1000).toFixed(1)} mm · ${degrees(solution.rotationError).toFixed(1)}° error`;
      dirty = false;
    }
    if (mode !== 'ghost' && (document.hidden || dt > 0.25)) await stop('Control paused · holding position');
    if (mode !== 'ghost' && (mode === 'joint' || reachable)) {
      if (bus) { bus.maxSpeed = Number($('speed').value); await bus.move(target, dt); }
      if (demo) actual = actual.map((value, i) => value + clamp(target[i] - value, -radians(Number($('speed').value)) * Math.min(dt, 0.05), radians(Number($('speed').value)) * Math.min(dt, 0.05)));
    } else if (bus?.armed) await bus.goals(bus.rows.map(row => row.Position));
    if (performance.now() >= nextDisplay) { display(); nextDisplay = performance.now() + 100; }
  } catch (error) { busy = false; actions.length = 0; await fail(error); updateControls(); }
  setTimeout(step, Math.max(0, 20 - (performance.now() - started)));
}

try {
  const manager = new THREE.LoadingManager();
  let meshError;
  manager.onError = url => { meshError = Error(`Unable to load robot mesh: ${url}`); };
  const meshes = new Promise(resolve => { manager.onLoad = resolve; });
  robot = await new URDFLoader(manager).loadAsync(`${import.meta.env.BASE_URL}so101.urdf`);
  await meshes;
  if (meshError) throw meshError;
  for (const joint of Object.values(robot.joints)) joint.ignoreLimits = true;
  ghost = robot.clone();
  ghost.traverse(node => {
    if (node.isMesh) node.material = new THREE.MeshStandardMaterial({ color: 0x45cbdc, transparent: true, opacity: 0.28, depthWrite: false });
  });
  scene.add(robot, ghost);
  configure(defaultCalibration, 'arm1.json (default)');
  try {
    const saved = JSON.parse(localStorage.getItem('so101-calibration'));
    if (saved) configure(saved.data, saved.name);
  } catch (error) { status(`Stored calibration unavailable: ${error.message}`, true); }
  if (navigator.serial) {
    navigator.serial.addEventListener('connect', refreshPorts);
    navigator.serial.addEventListener('disconnect', () => { enqueue(async () => { if (connected) await disconnect(); await refreshPorts(); }); });
    await refreshPorts();
    status('Select a serial port to connect · ghost only');
  } else {
    $('browser-help').textContent = 'Web Serial requires desktop Chrome or Edge over HTTPS. Demo is available in this browser.';
    status('Serial unavailable · use desktop Chrome or Edge', true);
  }
  updateControls();
  step();
} catch (error) { status(error.message, true); console.error(error); }
