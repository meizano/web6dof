// Marker detection on a grayscale frame: js-aruco2 candidate search + decoding,
// followed by sub-pixel corner refinement (the stock detector returns integer
// contour pixels, which makes the pose jitter).
import { AR, CV } from '../vendor/aruco.js';

export class MarkerDetector {
  constructor(dictionaryName = 'ARUCO_MIP_36h12') {
    // The library default tolerance is the dictionary's full tau (12 bits for 36h12), which
    // lets random texture decode as a marker. Accept at most 2 flipped bits.
    this.det = new AR.Detector({ dictionaryName, maxHammingDistance: 3 });
    this.w = 0;
    this.h = 0;
  }

  _ensure(w, h) {
    if (w === this.w && h === this.h) return;
    const d = this.det;
    d.grey = new CV.Image(w, h, null);
    d.thres = new CV.Image(w, h, new Uint8ClampedArray(w * h));
    // contour labels can exceed 255, so this must not be a byte array
    d.binary = new Int32Array((w + 2) * (h + 2));
    this.w = w;
    this.h = h;
  }

  /**
   * @param {Uint8Array|Uint8ClampedArray} gray  w*h luminance, row 0 = top of image
   * @returns {{id:number, corners:{x:number,y:number}[], hamming:number}[]}
   *   corners in continuous pixel coordinates (0,0 = top-left edge of the image),
   *   ordered top-left, top-right, bottom-right, bottom-left of the printed marker.
   */
  detect(gray, w, h) {
    this._ensure(w, h);
    const d = this.det;
    d.grey.data = gray;
    d.grey.width = w;
    d.grey.height = h;

    CV.adaptiveThreshold(d.grey, d.thres, 2, 7);
    d.contours = CV.findContours(d.thres, d.binary);
    let candidates = d.findCandidates(d.contours, w * 0.01, 0.05, 10);
    candidates = d.clockwiseCorners(candidates);
    // Note: the stock notTooNear() filter is skipped on purpose. It drops the smaller of two
    // nested quads, which throws away the real black square whenever the white quiet zone
    // around it is thin. Instead decode everything and de-duplicate by id + position.
    const decoded = d.findMarkers(d.grey, candidates, 49);
    const markers = [];
    for (const m of decoded) {
      const cx = (m.corners[0].x + m.corners[2].x) / 2, cy = (m.corners[0].y + m.corners[2].y) / 2;
      const dup = markers.find((o) => o.id === m.id && Math.hypot(o.cx - cx, o.cy - cy) < 10);
      if (dup) {
        if (m.hammingDistance < dup.hammingDistance) Object.assign(dup, m, { cx, cy });
        continue;
      }
      markers.push(Object.assign(m, { cx, cy }));
    }

    return markers.map((m) => {
      // refinement window must stay smaller than one marker cell, or it locks onto inner bits
      const edge = Math.min(...m.corners.map((c, i) => {
        const n = m.corners[(i + 1) % 4];
        return Math.hypot(n.x - c.x, n.y - c.y);
      }));
      const win = Math.max(1, Math.min(4, Math.round(edge / 14)));
      return {
      id: m.id,
      hamming: m.hammingDistance,
      corners: m.corners.map((c) => {
        const r = refineCorner(gray, w, h, c.x, c.y, win);
        // pixel index -> continuous coordinate (pixel centres sit at +0.5)
        return { x: r.x + 0.5, y: r.y + 0.5 };
      }),
      };
    });
  }
}

/**
 * Gradient-based sub-pixel corner refinement (same idea as OpenCV cornerSubPix):
 * the corner q is the point where every image gradient g_i in the window is
 * orthogonal to (p_i - q), i.e. solve  sum(g g^T) q = sum(g g^T p).
 */
export function refineCorner(img, w, h, x0, y0, win) {
  let x = x0;
  let y = y0;
  for (let iter = 0; iter < 4; iter++) {
    const cx = Math.round(x);
    const cy = Math.round(y);
    if (cx - win - 1 < 0 || cy - win - 1 < 0 || cx + win + 1 >= w || cy + win + 1 >= h) break;
    let a = 0, b = 0, c = 0, bx = 0, by = 0;
    const s2 = 2 * (win * 0.75) ** 2;
    for (let dy = -win; dy <= win; dy++) {
      const py = cy + dy;
      const row = py * w;
      for (let dx = -win; dx <= win; dx++) {
        const px = cx + dx;
        const gx = (img[row + px + 1] - img[row + px - 1]) * 0.5;
        const gy = (img[row + w + px] - img[row - w + px]) * 0.5;
        const wt = Math.exp(-(dx * dx + dy * dy) / s2);
        const gxx = gx * gx * wt, gxy = gx * gy * wt, gyy = gy * gy * wt;
        a += gxx; b += gxy; c += gyy;
        bx += gxx * px + gxy * py;
        by += gxy * px + gyy * py;
      }
    }
    const det = a * c - b * b;
    if (Math.abs(det) < 1e-6) break;
    const nx = (c * bx - b * by) / det;
    const ny = (a * by - b * bx) / det;
    // reject runaway solutions: keep the original corner
    if (Math.abs(nx - x0) > win || Math.abs(ny - y0) > win) return { x: x0, y: y0 };
    const moved = Math.hypot(nx - x, ny - y);
    x = nx;
    y = ny;
    if (moved < 0.01) break;
  }
  return { x, y };
}
