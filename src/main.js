import * as THREE from 'three';
import { CameraGrabber } from './xr/camera-grabber.js';
import { MarkerAnchor } from './xr/marker-anchor.js';
import { createWorkspace } from './scene/workspace.js';
import { intrinsicsFromProjection } from './cv/pose.js';

// ---------------------------------------------------------------- config (URL overridable)
const params = new URLSearchParams(location.search);
const CONFIG = {
  markerSize: parseFloat(params.get('size') ?? '0.10'), // metres, edge of the black square
  markerId: parseInt(params.get('id') ?? '0', 10), // -1 = any id
  dictionary: 'ARUCO_MIP_36h12',
  detectWidth: parseInt(params.get('w') ?? '640', 10),
  lostMode: params.get('lost') === '3dof' ? '3dof' : 'hold',
};

// ---------------------------------------------------------------- DOM
const $ = (id) => document.getElementById(id);
const ui = {
  start: $('start'),
  support: $('support'),
  overlay: $('overlay'),
  mode: $('hud-mode'),
  stats: $('hud-stats'),
  lost: $('btn-lost'),
  recenter: $('btn-recenter'),
  exit: $('btn-exit'),
  debug: $('btn-debug'),
  preview: $('preview'),
};
$('cfg').textContent = `marker id ${CONFIG.markerId < 0 ? 'any' : CONFIG.markerId}, ${CONFIG.markerSize * 100} cm`;

// ---------------------------------------------------------------- three.js
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance' });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType('local');
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.01, 50);

const stats = { mode: '-', marker: '-', detect: '-', fps: '-' };
const ws = createWorkspace(() => stats);
scene.add(ws.root, ws.markerGizmo);

const tracker = new MarkerAnchor({ lostMode: CONFIG.lostMode });

// ---------------------------------------------------------------- detection worker
const worker = new Worker(new URL('./cv/detector.worker.js', import.meta.url), { type: 'module' });
worker.postMessage({ type: 'init', dictionary: CONFIG.dictionary });
let workerBusy = false;
const spareBuffers = [];
const inflight = new Map(); // frameId -> capture meta
let frameCounter = 0;
let detectMs = 0;
let detectRate = 0;
let detectCount = 0;
let seenIds = []; // ids found by the detector recently (any id, for diagnosing a wrong ?id=)
let seenAt = -Infinity;

// flip auto-calibration: if we never find a marker, alternate the vertical flip until
// one is found, then lock it (the camera texture orientation differs between devices).
let flipLocked = false;
let missesSinceFlip = 0;

const tmpMat = new THREE.Matrix4();
const camWorld = new THREE.Matrix4();
const markerCv = new THREE.Matrix4();
// CV camera (x right, y down, z fwd) -> GL camera (x right, y up, z back)
const CV_TO_GL = new THREE.Matrix4().makeScale(1, -1, -1);
// marker frame used by the app (y = normal out of paper, z = towards bottom edge)
// -> CV marker frame (y down the page, z into the paper)
const APP_TO_CV_MARKER = new THREE.Matrix4().set(1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1);

worker.onmessage = (e) => {
  const { frameId, ms, markers, gray } = e.data;
  workerBusy = false;
  spareBuffers.push(gray);
  const meta = inflight.get(frameId);
  inflight.delete(frameId);
  detectMs = detectMs * 0.9 + ms * 0.1;
  detectCount++;
  if (markers.length) { seenIds = markers.map((k) => k.id); seenAt = performance.now(); }
  if (!meta) return;

  const m = markers.find((k) => CONFIG.markerId < 0 || k.id === CONFIG.markerId);
  if (!m) {
    if (!flipLocked && ++missesSinceFlip > 20) {
      grabber.flipY = !grabber.flipY;
      missesSinceFlip = 0;
    }
    return;
  }
  if (!flipLocked && meta.flipY === grabber.flipY) flipLocked = true;
  if (m.rmsPx > 2.5) return; // poor fit, likely a bad corner

  const R = m.R, t = m.t;
  markerCv.set(R[0], R[1], R[2], t[0], R[3], R[4], R[5], t[1], R[6], R[7], R[8], t[2], 0, 0, 0, 1);
  camWorld.fromArray(meta.camWorld);
  // world <- cam(GL) <- cam(CV) <- marker(CV) <- marker(app)
  tmpMat.copy(camWorld).multiply(CV_TO_GL).multiply(markerCv).multiply(APP_TO_CV_MARKER);

  tracker.ingest(tmpMat, meta.time);
  ws.markerGizmo.matrixAutoUpdate = false;
  ws.markerGizmo.matrix.copy(tmpMat).multiply(new THREE.Matrix4().makeScale(CONFIG.markerSize, 1, CONFIG.markerSize));
  ws.markerGizmo.userData.seen = meta.time;
  stats.marker = `#${m.id} ${(t[2] * 100).toFixed(0)}cm ${m.rmsPx.toFixed(2)}px`;
};

// ---------------------------------------------------------------- session
let grabber = null;
let glBinding = null;
let session = null;

async function checkSupport() {
  if (!navigator.xr) return 'WebXR is not available in this browser. Use Chrome on an ARCore-capable Android phone.';
  if (!window.isSecureContext) return 'This page must be served over HTTPS (or localhost via adb reverse).';
  const ok = await navigator.xr.isSessionSupported('immersive-ar').catch(() => false);
  if (!ok) return 'immersive-ar is not supported on this device/browser.';
  return null;
}

checkSupport().then((err) => {
  if (err) {
    ui.support.textContent = err;
    ui.start.disabled = true;
  }
});

ui.start.addEventListener('click', async () => {
  try {
    session = await navigator.xr.requestSession('immersive-ar', {
      requiredFeatures: ['camera-access'],
      optionalFeatures: ['dom-overlay', 'anchors', 'local'],
      domOverlay: { root: ui.overlay },
    });
  } catch (err) {
    ui.support.textContent = `Could not start AR: ${err.message}. Raw camera access needs Chrome 107+ on Android with ARCore.`;
    return;
  }
  session.addEventListener('end', onSessionEnd);
  await renderer.xr.setSession(session);
  glBinding = renderer.xr.getBinding();
  grabber = new CameraGrabber(renderer.getContext(), { width: CONFIG.detectWidth });
  tracker.reset();
  document.body.classList.add('in-ar');
});

function onSessionEnd() {
  document.body.classList.remove('in-ar');
  session = null;
  grabber = null;
  glBinding = null;
  inflight.clear();
}

ui.exit.addEventListener('click', () => session?.end());
ui.recenter.addEventListener('click', () => tracker.reset());
ui.lost.addEventListener('click', () => {
  tracker.lostMode = tracker.lostMode === 'hold' ? '3dof' : 'hold';
  updateLostButton();
});
ui.debug.addEventListener('click', () => ui.preview.classList.toggle('show'));
function updateLostButton() {
  ui.lost.textContent = tracker.lostMode === 'hold' ? 'Marker lost: hold 6DoF' : 'Marker lost: 3DoF';
}
updateLostButton();
// taps on the HUD must not also count as taps in the 3D scene
ui.overlay.addEventListener('beforexrselect', (e) => {
  if (e.target.closest('button')) e.preventDefault();
});

// tap a window to focus it (screen taps arrive as transient XR input)
const controller = renderer.xr.getController(0);
scene.add(controller);
const raycaster = new THREE.Raycaster();
controller.addEventListener('select', () => {
  raycaster.ray.origin.setFromMatrixPosition(controller.matrixWorld);
  raycaster.ray.direction.set(0, 0, -1).transformDirection(controller.matrixWorld);
  const hit = raycaster.intersectObjects(ws.meshes, false)[0];
  ws.toggleFocus(hit ? hit.object.userData.panel : null);
});

// ---------------------------------------------------------------- per-frame
let last = 0;
let fpsAcc = 0, fpsFrames = 0, rateTimer = 0;
let previewTimer = 0;
const previewCtx = ui.preview.getContext('2d');

renderer.setAnimationLoop((time, frame) => {
  const now = time / 1000;
  const dt = Math.min(0.1, last ? now - last : 0.016);
  last = now;

  if (frame && grabber) {
    const refSpace = renderer.xr.getReferenceSpace();
    const pose = frame.getViewerPose(refSpace);
    if (pose) {
      handleCamera(frame, pose, now, dt);
      const d = tracker.update(frame, refSpace, pose, dt, now);
      ws.root.position.copy(d.pos);
      ws.root.rotation.set(0, d.yaw, 0);
      ws.setSurface(tracker.target.surface);
      ws.markerGizmo.visible = now - (ws.markerGizmo.userData.seen ?? -1) < 0.3;
    }
  }

  ws.tick(dt);
  updateHud(dt);
  renderer.render(scene, camera);
});

function handleCamera(frame, pose, now, dt) {
  let touchedGl = false;
  // 1) collect a finished readback and hand it to the worker (only when it is idle, so the
  //    newest frame is always the one being processed)
  const ready = !workerBusy && grabber.busy && (touchedGl = true) && grabber.poll((n) => {
    const buf = spareBuffers.pop();
    return buf && buf.byteLength === n ? new Uint8Array(buf) : new Uint8Array(n);
  });
  if (ready) {
    const id = ++frameCounter;
    inflight.set(id, ready);
    workerBusy = true;
    const K = intrinsicsFromProjection(ready.projection, ready.w, ready.h);
    worker.postMessage(
      { gray: ready.pixels.buffer, w: ready.w, h: ready.h, K, markerSize: CONFIG.markerSize, frameId: id },
      [ready.pixels.buffer],
    );
    if (ui.preview.classList.contains('show') && (previewTimer += dt) > 0.5) {
      previewTimer = 0;
      drawPreview(ready);
    }
  }

  // 2) start the next capture while the worker is busy with the previous one: at most one
  //    readback + one detection in flight keeps latency low and leaves the GPU to rendering
  const view = pose.views[0];
  if (!grabber.busy && view?.camera) {
    const tex = glBinding.getCameraImage(view.camera);
    if (tex) {
      grabber.capture(tex, view.camera.width, view.camera.height, {
        camWorld: Array.from(view.transform.matrix),
        projection: Array.from(view.projectionMatrix),
        time: now,
      });
      touchedGl = true;
    }
  }
  if (touchedGl) {
    // We touched GL state behind three.js's back, so its state cache must be reset.
    // resetState() also forgets the current render target, which in XR is the session's
    // framebuffer (set by three before this callback). Without restoring it, every frame
    // that touched the camera rendered to the invisible default framebuffer → flicker.
    const rt = renderer.getRenderTarget();
    renderer.resetState();
    renderer.setRenderTarget(rt);
  }
}

function drawPreview(meta) {
  const { w, h } = meta;
  const pw = ui.preview.width = 240, ph = ui.preview.height = Math.round((240 * h) / w);
  const img = previewCtx.createImageData(pw, ph);
  const g = meta.pixels;
  for (let y = 0; y < ph; y++) {
    const sy = Math.floor((y * h) / ph) * w;
    for (let x = 0; x < pw; x++) {
      const v = g[sy + Math.floor((x * w) / pw)];
      const o = (y * pw + x) * 4;
      img.data[o] = img.data[o + 1] = img.data[o + 2] = v;
      img.data[o + 3] = 255;
    }
  }
  previewCtx.putImageData(img, 0, 0);
}

function updateHud(dt) {
  fpsAcc += dt; fpsFrames++;
  rateTimer += dt;
  if (rateTimer < 0.5) return;
  const fps = fpsFrames / fpsAcc;
  detectRate = detectCount / rateTimer;
  fpsAcc = 0; fpsFrames = 0; detectCount = 0; rateTimer = 0;

  const labels = {
    searching: ['3DoF · looking for marker', 'searching'],
    tracking: ['6DoF · locked to marker', 'tracking'],
    held: ['6DoF · held by world tracking', 'held'],
    '3dof': ['3DoF · marker lost', 'threedof'],
  };
  const [label, cls] = labels[tracker.state];
  stats.mode = tracker.state;
  stats.fps = fps.toFixed(0);
  stats.detect = `${detectMs.toFixed(1)}ms ${detectRate.toFixed(0)}Hz`;
  if (tracker.state !== 'tracking') stats.marker = '-';
  ui.mode.textContent = tracker.trackingLost ? '3DoF · device tracking lost' : label;
  ui.mode.className = cls;
  const seen = performance.now() - seenAt < 1000 ? `sees #${seenIds.join(',#')}` : 'no marker in view';
  ui.stats.textContent = `${stats.fps} fps · detect ${stats.detect} · ${grabber ? `${grabber.w}×${grabber.h}` : ''} · ${seen}${flipLocked ? '' : ' · calibrating'}`;
}

window.addEventListener('resize', () => {
  if (renderer.xr.isPresenting) return;
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
