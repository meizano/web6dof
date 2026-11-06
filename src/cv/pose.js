// Pose of a square marker from its 4 image corners (planar PnP):
// homography initialisation + Levenberg-Marquardt refinement of reprojection error.
//
// Conventions (OpenCV style):
//   camera: x right, y down, z forward
//   marker: origin at centre, x right, y down (towards the printed bottom edge),
//           z into the paper. Corner order: TL, TR, BR, BL.

/** Intrinsics for a w*h image from a WebGL/WebXR projection matrix (column-major). */
export function intrinsicsFromProjection(P, w, h) {
  return {
    fx: (P[0] * w) / 2,
    fy: (P[5] * h) / 2,
    cx: ((1 - P[8]) * w) / 2,
    cy: ((1 + P[9]) * h) / 2,
  };
}

/**
 * @param corners  [{x,y}] x4, pixel coords
 * @param K        {fx,fy,cx,cy}
 * @param size     marker black-square edge length in metres
 * @returns {{R:number[], t:number[], rmsPx:number} | null}  R row-major 3x3, marker->camera
 */
export function solveSquarePose(corners, K, size) {
  const s = size / 2;
  const obj = [[-s, -s], [s, -s], [s, s], [-s, s]];
  const img = corners.map((c) => [(c.x - K.cx) / K.fx, (c.y - K.cy) / K.fy]);

  const H = homography(obj, img);
  if (!H) return null;
  let { R, t } = decomposeHomography(H);
  if (!R) return null;

  // Levenberg-Marquardt over a rotation increment (axis-angle, left-multiplied) and t.
  let lambda = 1e-3;
  let r = residuals(obj, img, R, t);
  let cost = dot(r, r);
  for (let iter = 0; iter < 15; iter++) {
    const J = jacobian(obj, img, R, t);
    const JtJ = new Float64Array(36);
    const Jtr = new Float64Array(6);
    for (let i = 0; i < 8; i++) {
      for (let a = 0; a < 6; a++) {
        Jtr[a] += J[i * 6 + a] * r[i];
        for (let b = 0; b < 6; b++) JtJ[a * 6 + b] += J[i * 6 + a] * J[i * 6 + b];
      }
    }
    let improved = false;
    for (let tries = 0; tries < 6; tries++) {
      const A = Float64Array.from(JtJ);
      for (let a = 0; a < 6; a++) A[a * 6 + a] *= 1 + lambda;
      const delta = solve(A, Array.from(Jtr, (v) => -v), 6);
      if (!delta) break;
      const R2 = mul3(rodrigues(delta[0], delta[1], delta[2]), R);
      const t2 = [t[0] + delta[3], t[1] + delta[4], t[2] + delta[5]];
      const r2 = residuals(obj, img, R2, t2);
      const c2 = dot(r2, r2);
      if (c2 < cost) {
        R = R2; t = t2; r = r2;
        const rel = (cost - c2) / (cost + 1e-30);
        cost = c2;
        lambda = Math.max(lambda / 10, 1e-9);
        improved = rel > 1e-10;
        break;
      }
      lambda *= 10;
    }
    if (!improved) break;
  }
  if (t[2] <= 0) return null;
  const f = (K.fx + K.fy) / 2;
  return { R, t, rmsPx: Math.sqrt(cost / 4) * f };
}

// ---------------------------------------------------------------- helpers

function homography(src, dst) {
  // Solve for h (8 unknowns, h33 = 1) mapping src (X,Y) -> dst (x,y).
  const A = new Float64Array(64);
  const b = new Array(8);
  for (let i = 0; i < 4; i++) {
    const [X, Y] = src[i];
    const [x, y] = dst[i];
    A.set([X, Y, 1, 0, 0, 0, -x * X, -x * Y], (2 * i) * 8);
    A.set([0, 0, 0, X, Y, 1, -y * X, -y * Y], (2 * i + 1) * 8);
    b[2 * i] = x;
    b[2 * i + 1] = y;
  }
  const h = solve(A, b, 8);
  if (!h) return null;
  return [h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7], 1];
}

function decomposeHomography(H) {
  const h1 = [H[0], H[3], H[6]];
  const h2 = [H[1], H[4], H[7]];
  const h3 = [H[2], H[5], H[8]];
  let l = 2 / (norm(h1) + norm(h2));
  if (h3[2] * l < 0) l = -l; // marker must be in front of the camera
  let r1 = scale(h1, l);
  let r2 = scale(h2, l);
  const t = scale(h3, l);
  // make r1, r2 orthonormal symmetrically
  const e = dot(r1, r2) / 2;
  const a = normalize(sub(r1, scale(r2, e)));
  const c = normalize(sub(r2, scale(r1, e)));
  r1 = a; r2 = c;
  const r3 = cross(r1, r2);
  // columns r1 r2 r3 -> row-major
  const R = [r1[0], r2[0], r3[0], r1[1], r2[1], r3[1], r1[2], r2[2], r3[2]];
  return { R, t };
}

function residuals(obj, img, R, t) {
  const r = new Array(8);
  for (let i = 0; i < 4; i++) {
    const [X, Y] = obj[i];
    const x = R[0] * X + R[1] * Y + t[0];
    const y = R[3] * X + R[4] * Y + t[1];
    const z = R[6] * X + R[7] * Y + t[2];
    r[2 * i] = x / z - img[i][0];
    r[2 * i + 1] = y / z - img[i][1];
  }
  return r;
}

function jacobian(obj, img, R, t) {
  const J = new Float64Array(48);
  const base = residuals(obj, img, R, t);
  const eps = 1e-7;
  for (let a = 0; a < 6; a++) {
    const d = [0, 0, 0, 0, 0, 0];
    d[a] = eps;
    const R2 = a < 3 ? mul3(rodrigues(d[0], d[1], d[2]), R) : R;
    const t2 = [t[0] + d[3], t[1] + d[4], t[2] + d[5]];
    const r2 = residuals(obj, img, R2, t2);
    for (let i = 0; i < 8; i++) J[i * 6 + a] = (r2[i] - base[i]) / eps;
  }
  return J;
}

function rodrigues(x, y, z) {
  const th = Math.hypot(x, y, z);
  if (th < 1e-12) return [1, -z, y, z, 1, -x, -y, x, 1];
  const kx = x / th, ky = y / th, kz = z / th;
  const c = Math.cos(th), s = Math.sin(th), v = 1 - c;
  return [
    kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s,
    ky * kx * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s,
    kz * kx * v - ky * s, kz * ky * v + kx * s, kz * kz * v + c,
  ];
}

function mul3(A, B) {
  const C = new Array(9);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      C[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
  return C;
}

/** Gaussian elimination with partial pivoting; A is n*n row-major (modified). */
function solve(A, b, n) {
  const M = Float64Array.from(A);
  const x = Float64Array.from(b);
  for (let col = 0; col < n; col++) {
    let piv = col;
    for (let r = col + 1; r < n; r++) if (Math.abs(M[r * n + col]) > Math.abs(M[piv * n + col])) piv = r;
    if (Math.abs(M[piv * n + col]) < 1e-14) return null;
    if (piv !== col) {
      for (let k = 0; k < n; k++) {
        const tmp = M[col * n + k]; M[col * n + k] = M[piv * n + k]; M[piv * n + k] = tmp;
      }
      const tmp = x[col]; x[col] = x[piv]; x[piv] = tmp;
    }
    for (let r = col + 1; r < n; r++) {
      const f = M[r * n + col] / M[col * n + col];
      if (f === 0) continue;
      for (let k = col; k < n; k++) M[r * n + k] -= f * M[col * n + k];
      x[r] -= f * x[col];
    }
  }
  for (let r = n - 1; r >= 0; r--) {
    let s = x[r];
    for (let k = r + 1; k < n; k++) s -= M[r * n + k] * x[k];
    x[r] = s / M[r * n + r];
  }
  return Array.from(x);
}

const dot = (a, b) => a.reduce((s, v, i) => s + v * b[i], 0);
const norm = (a) => Math.sqrt(dot(a, a));
const scale = (a, k) => a.map((v) => v * k);
const sub = (a, b) => a.map((v, i) => v - b[i]);
const normalize = (a) => scale(a, 1 / norm(a));
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
