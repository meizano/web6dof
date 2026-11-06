// Runs marker detection + pose estimation off the main thread.
// In:  { gray: ArrayBuffer (w*h bytes, row 0 = top), w, h, K, markerSize, frameId }
// Out: { frameId, ms, markers: [{ id, corners, R, t, rmsPx }], gray } (buffer handed back for reuse)
import { MarkerDetector } from './detect.js';
import { solveSquarePose } from './pose.js';

let detector = null;

self.onmessage = (e) => {
  const msg = e.data;
  if (msg.type === 'init') {
    detector = new MarkerDetector(msg.dictionary);
    return;
  }
  const { gray, w, h, K, markerSize, frameId } = msg;
  const t0 = performance.now();
  const found = detector.detect(new Uint8Array(gray), w, h);
  const markers = [];
  for (const m of found) {
    const pose = solveSquarePose(m.corners, K, markerSize);
    if (pose) markers.push({ id: m.id, corners: m.corners, ...pose });
  }
  self.postMessage({ frameId, ms: performance.now() - t0, markers, gray }, [gray]);
};
