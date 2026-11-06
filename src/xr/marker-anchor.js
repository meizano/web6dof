// Decides where the workspace lives in the XR world, frame by frame.
//
//   searching  no marker seen yet: workspace floats in front of you, rotation-only (3DoF)
//   tracking   marker visible: workspace locked to the marker (6DoF)
//   held       marker gone: workspace stays where the marker was, kept there by the
//              device's own SLAM (+ an XRAnchor for drift correction). Still 6DoF.
//   3dof       marker gone and lostMode = '3dof', or device tracking lost:
//              workspace follows your head position, rotation stays world-fixed.
import * as THREE from 'three';

const UP = new THREE.Vector3(0, 1, 0);

export class MarkerAnchor {
  constructor({ lostMode = 'hold', visibleTimeout = 0.35, anchors = true } = {}) {
    this.lostMode = lostMode;
    this.visibleTimeout = visibleTimeout;
    this.useAnchors = anchors;
    this.state = 'searching';

    this.target = { pos: new THREE.Vector3(), yaw: 0, surface: 'table' };
    this.display = { pos: new THREE.Vector3(), yaw: 0 };
    this.hasTarget = false;
    this.lastSeen = -Infinity;
    this.outliers = [];
    this.headOffset = new THREE.Vector3();
    this.needsPlacement = true;

    this.anchor = null;
    this.anchorPending = false;
    this.anchorAt = { pos: new THREE.Vector3(), yaw: 0, time: -Infinity };
    this.trackingLost = false;
  }

  /** Re-place the workspace in front of the user and forget the marker. */
  reset() {
    this.state = 'searching';
    this.hasTarget = false;
    this.needsPlacement = true;
    this.outliers.length = 0;
    this.anchor?.delete?.();
    this.anchor = null;
  }

  /**
   * Feed a marker detection.
   * @param {THREE.Matrix4} markerWorld  marker frame in XR world space (x right, y = normal
   *                                     out of the paper, z towards the printed bottom edge)
   */
  ingest(markerWorld, now) {
    const pos = new THREE.Vector3().setFromMatrixPosition(markerWorld);
    const ax = new THREE.Vector3(), normal = new THREE.Vector3(), down = new THREE.Vector3();
    markerWorld.extractBasis(ax, normal, down);

    // Which way should the workspace face? Lying on a table: towards the printed bottom
    // edge (you read the marker upright). On a wall: out of the wall.
    const flat = normal.y > 0.6;
    const forward = flat ? down : normal;
    forward.y = 0;
    if (forward.lengthSq() < 1e-6) return;
    const yaw = Math.atan2(forward.x, forward.z);
    const surface = flat ? 'table' : 'wall';

    if (!this.hasTarget) {
      this._accept(pos, yaw, surface, 1);
    } else {
      const jump = pos.distanceTo(this.target.pos);
      const turn = Math.abs(angleDiff(yaw, this.target.yaw));
      if (jump > 0.15 || turn > 0.5) {
        // Either a bad detection or the marker was really moved. Accept the jump only
        // after 3 detections that agree with each other.
        this.outliers.push({ pos, yaw });
        if (this.outliers.length > 3) this.outliers.shift();
        const agree = this.outliers.length === 3 &&
          this.outliers.every((o) => o.pos.distanceTo(pos) < 0.03 && Math.abs(angleDiff(o.yaw, yaw)) < 0.1);
        if (!agree) return;
        this._accept(pos, yaw, surface, 1);
      } else {
        this._accept(pos, yaw, surface, 0.35);
      }
    }
    this.outliers.length = 0;
    this.lastSeen = now;
  }

  _accept(pos, yaw, surface, alpha) {
    if (alpha >= 1 || !this.hasTarget) {
      this.target.pos.copy(pos);
      this.target.yaw = yaw;
    } else {
      this.target.pos.lerp(pos, alpha);
      this.target.yaw += angleDiff(yaw, this.target.yaw) * alpha;
    }
    this.target.surface = surface;
    this.hasTarget = true;
    this.state = 'tracking';
  }

  /**
   * @param {XRFrame} frame
   * @param {XRReferenceSpace} refSpace
   * @param {XRViewerPose} viewerPose
   */
  update(frame, refSpace, viewerPose, dt, now) {
    const head = new THREE.Vector3();
    const headQ = new THREE.Quaternion();
    const p = viewerPose.transform.position, o = viewerPose.transform.orientation;
    head.set(p.x, p.y, p.z);
    headQ.set(o.x, o.y, o.z, o.w);
    this.trackingLost = viewerPose.emulatedPosition;

    const visible = now - this.lastSeen < this.visibleTimeout;

    if (this.needsPlacement) {
      // place 0.8 m in front of the user, slightly below eye level, facing them
      const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(headQ);
      fwd.y = 0;
      if (fwd.lengthSq() < 1e-6) fwd.set(0, 0, -1);
      fwd.normalize();
      this.display.pos.copy(head).addScaledVector(fwd, 0.8).add(new THREE.Vector3(0, -0.15, 0));
      this.display.yaw = Math.atan2(-fwd.x, -fwd.z);
      this.headOffset.subVectors(this.display.pos, head);
      this.needsPlacement = false;
    }

    // ---- state transitions
    if (this.hasTarget && visible && !this.trackingLost) {
      this.state = 'tracking';
    } else if (this.state === 'tracking' || (this.state === 'held' && this.trackingLost)) {
      const goTo3dof = this.lostMode === '3dof' || this.trackingLost;
      this.state = goTo3dof ? '3dof' : 'held';
      if (goTo3dof) this.headOffset.subVectors(this.display.pos, head);
    } else if (this.state === '3dof' && this.lostMode === 'hold' && !this.trackingLost && this.hasTarget) {
      this.state = 'held';
    }

    // ---- where should the workspace be?
    if (this.state === 'tracking') {
      this._maybeAnchor(frame, refSpace, now);
      this._smoothTo(this.target.pos, this.target.yaw, dt, 14);
    } else if (this.state === 'held') {
      const a = this._anchorPose(frame, refSpace);
      if (a) {
        // the anchor was created at target pose; follow SLAM's corrections to it
        this.target.pos.copy(a.pos);
        this.target.yaw = a.yaw;
      }
      this._smoothTo(this.target.pos, this.target.yaw, dt, 8);
    } else {
      // searching / 3dof: translation follows the head, rotation stays put in the world
      this.display.pos.addVectors(head, this.headOffset);
    }

    return this.display;
  }

  _smoothTo(pos, yaw, dt, rate) {
    const k = 1 - Math.exp(-rate * dt);
    this.display.pos.lerp(pos, k);
    this.display.yaw += angleDiff(yaw, this.display.yaw) * k;
  }

  _maybeAnchor(frame, refSpace, now) {
    if (!this.useAnchors || this.anchorPending || typeof frame.createAnchor !== 'function') return;
    const moved = this.anchorAt.pos.distanceTo(this.target.pos) > 0.01 ||
      Math.abs(angleDiff(this.anchorAt.yaw, this.target.yaw)) > 0.03;
    if (!(this.anchor === null || (moved && now - this.anchorAt.time > 1.0))) return;

    const q = new THREE.Quaternion().setFromAxisAngle(UP, this.target.yaw);
    const t = this.target.pos;
    const pose = new XRRigidTransform({ x: t.x, y: t.y, z: t.z }, { x: q.x, y: q.y, z: q.z, w: q.w });
    const at = { pos: t.clone(), yaw: this.target.yaw, time: now };
    this.anchorPending = true;
    let promise;
    try {
      promise = frame.createAnchor(pose, refSpace);
    } catch {
      this.useAnchors = false;
      this.anchorPending = false;
      return;
    }
    promise.then(
      (anchor) => {
        this.anchor?.delete?.();
        this.anchor = anchor;
        this.anchorAt = at;
        this.anchorPending = false;
      },
      () => {
        this.anchorPending = false;
        this.anchorAt.time = now; // back off, retry later
      },
    );
  }

  _anchorPose(frame, refSpace) {
    if (!this.anchor || !frame.trackedAnchors?.has(this.anchor)) return null;
    const pose = frame.getPose(this.anchor.anchorSpace, refSpace);
    if (!pose) return null;
    const { position: p, orientation: o } = pose.transform;
    const fwd = new THREE.Vector3(0, 0, 1).applyQuaternion(new THREE.Quaternion(o.x, o.y, o.z, o.w));
    return { pos: new THREE.Vector3(p.x, p.y, p.z), yaw: Math.atan2(fwd.x, fwd.z) };
  }
}

function angleDiff(a, b) {
  let d = a - b;
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return d;
}
