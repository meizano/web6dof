// Renders a synthetic ArUco marker at known poses, runs the real detector + pose
// solver on it and checks the recovered pose. Run: npm test
import { MarkerDetector } from '../src/cv/detect.js';
import { solveSquarePose, intrinsicsFromProjection } from '../src/cv/pose.js';
import { AR } from '../src/vendor/aruco.js';

const W = 640, H = 360;
const SIZE = 0.1; // metres
const ID = 7;
const code = new AR.Dictionary('ARUCO_MIP_36h12').codeList[ID];

// A WebGL-style projection (column-major) like a phone camera: ~60deg vertical FOV,
// slightly off-centre principal point, to exercise intrinsicsFromProjection.
const fovY = (60 * Math.PI) / 180, aspect = W / H, near = 0.05, far = 100;
const f = 1 / Math.tan(fovY / 2);
const P = new Float64Array(16);
P[0] = f / aspect; P[5] = f; P[8] = 0.02; P[9] = -0.015;
P[10] = (far + near) / (near - far); P[11] = -1; P[14] = (2 * far * near) / (near - far);
const K = intrinsicsFromProjection(P, W, H);

let failures = 0;
const check = (cond, msg) => { if (!cond) { failures++; console.error('  FAIL', msg); } };

// 1) intrinsics agree with the GL projection for a random camera-space point
{
  const X = 0.13, Y = -0.07, Z = 0.9; // CV camera coords
  const gl = [X, -Y, -Z, 1];
  const clip = [0, 1, 3].map((r) => P[r] * gl[0] + P[4 + r] * gl[1] + P[8 + r] * gl[2] + P[12 + r] * gl[3]);
  const u = ((clip[0] / clip[2] + 1) / 2) * W;
  const v = ((1 - clip[1] / clip[2]) / 2) * H;
  const u2 = K.fx * (X / Z) + K.cx, v2 = K.fy * (Y / Z) + K.cy;
  check(Math.abs(u - u2) < 1e-6 && Math.abs(v - v2) < 1e-6, `intrinsics mismatch ${u},${v} vs ${u2},${v2}`);
}

function rotFromEuler(rx, ry, rz) {
  const [cx, sx, cy, sy, cz, sz] = [Math.cos(rx), Math.sin(rx), Math.cos(ry), Math.sin(ry), Math.cos(rz), Math.sin(rz)];
  const Rx = [1, 0, 0, 0, cx, -sx, 0, sx, cx];
  const Ry = [cy, 0, sy, 0, 1, 0, -sy, 0, cy];
  const Rz = [cz, -sz, 0, sz, cz, 0, 0, 0, 1];
  const m = (A, B) => [0, 1, 2].flatMap((i) => [0, 1, 2].map((j) => A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j]));
  return m(Rz, m(Ry, Rx));
}

function markerValue(X, Y) {
  const cell = SIZE / 8;
  const c = Math.floor((X + SIZE / 2) / cell), r = Math.floor((Y + SIZE / 2) / cell);
  if (c < -1 || r < -1 || c > 8 || r > 8) return null; // outside the printed sheet area
  if (c < 0 || r < 0 || c > 7 || r > 7) return 255; // white quiet zone
  if (c === 0 || r === 0 || c === 7 || r === 7) return 0;
  return code[(r - 1) * 6 + (c - 1)] === '1' ? 255 : 0;
}

function render(R, t, seed = 1) {
  // inverse homography: pixel -> marker plane
  const Hm = [
    [K.fx * R[0] + K.cx * R[6], K.fx * R[1] + K.cx * R[7], K.fx * t[0] + K.cx * t[2]],
    [K.fy * R[3] + K.cy * R[6], K.fy * R[4] + K.cy * R[7], K.fy * t[1] + K.cy * t[2]],
    [R[6], R[7], t[2]],
  ];
  const inv = inv3(Hm);
  const img = new Uint8Array(W * H);
  let s = seed;
  const rnd = () => ((s = (s * 1103515245 + 12345) & 0x7fffffff) / 0x7fffffff);
  for (let v = 0; v < H; v++) {
    for (let u = 0; u < W; u++) {
      let acc = 0;
      for (const [ox, oy] of [[0.25, 0.25], [0.75, 0.25], [0.25, 0.75], [0.75, 0.75]]) {
        const x = u + ox, y = v + oy;
        const p = [0, 1, 2].map((i) => inv[i][0] * x + inv[i][1] * y + inv[i][2]);
        const val = markerValue(p[0] / p[2], p[1] / p[2]);
        acc += val === null ? 110 + 30 * Math.sin(u * 0.05) * Math.cos(v * 0.07) : 20 + val * 0.85;
      }
      img[v * W + u] = Math.max(0, Math.min(255, acc / 4 + (rnd() - 0.5) * 12));
    }
  }
  return img;
}

function inv3(m) {
  const [a, b, c] = m[0], [d, e, f2] = m[1], [g, h, i] = m[2];
  const A = e * i - f2 * h, B = -(d * i - f2 * g), C = d * h - e * g;
  const det = a * A + b * B + c * C;
  return [
    [A / det, -(b * i - c * h) / det, (b * f2 - c * e) / det],
    [B / det, (a * i - c * g) / det, -(a * f2 - c * d) / det],
    [C / det, -(a * h - b * g) / det, (a * e - b * d) / det],
  ];
}

function rotAngleDeg(A, B) {
  // angle of A^T B
  let tr = 0;
  for (let i = 0; i < 3; i++) for (let k = 0; k < 3; k++) tr += A[k * 3 + i] * B[k * 3 + i];
  return (Math.acos(Math.max(-1, Math.min(1, (tr - 1) / 2))) * 180) / Math.PI;
}

const cases = [
  { name: 'frontal 40cm', e: [0, 0, 0], t: [0, 0, 0.4] },
  { name: 'tilted 35deg, 60cm, off-centre', e: [0.6, 0.2, 0.3], t: [0.08, -0.04, 0.6] },
  { name: 'table view 55deg, 50cm', e: [-0.95, 0.1, -0.4], t: [-0.05, 0.05, 0.5] },
  { name: 'rotated 170deg, 80cm', e: [0.3, -0.4, 2.97], t: [0.1, 0.06, 0.8] },
  { name: 'far 1.2m', e: [0.4, 0.3, 1.2], t: [0, 0, 1.2] },
];

const detector = new MarkerDetector('ARUCO_MIP_36h12');
for (const c of cases) {
  const R = rotFromEuler(...c.e);
  const img = render(R, c.t);
  if (process.env.DUMP) {
    const { writeFileSync } = await import('node:fs');
    writeFileSync(`${process.env.DUMP}/${c.name.replace(/\W+/g, '_')}.pgm`, Buffer.concat([Buffer.from(`P5 ${W} ${H} 255\n`), Buffer.from(img)]));
  }
  const t0 = performance.now();
  const found = detector.detect(img, W, H);
  const ms = performance.now() - t0;
  const m = found.find((x) => x.id === ID);
  if (!m) { check(false, `${c.name}: marker not detected (found ${found.map((x) => x.id)})`); continue; }
  const pose = solveSquarePose(m.corners, K, SIZE);
  if (!pose) { check(false, `${c.name}: pose failed`); continue; }
  const dt = Math.hypot(...pose.t.map((v, i) => v - c.t[i]));
  const dr = rotAngleDeg(R, pose.R);
  console.log(`${c.name.padEnd(34)} detect ${ms.toFixed(1).padStart(5)} ms  pos err ${(dt * 1000).toFixed(1).padStart(5)} mm  rot err ${dr.toFixed(2).padStart(5)} deg  reproj ${pose.rmsPx.toFixed(2)} px`);
  check(dt < 0.01 + 0.01 * c.t[2], `${c.name}: position error ${(dt * 1000).toFixed(1)} mm`);
  check(dr < 3, `${c.name}: rotation error ${dr.toFixed(2)} deg`);
}

// Detector must not invent markers in an empty scene
{
  const empty = new Uint8Array(W * H).map((_, i) => 100 + 40 * Math.sin(i * 0.01));
  check(detector.detect(empty, W, H).length === 0, 'false positive on empty frame');
}

if (failures) { console.error(`\n${failures} failure(s)`); process.exit(1); }
console.log('\nall pipeline checks passed');
