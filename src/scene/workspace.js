// The "apps": flat windows drawn into canvas textures. Canvas texture uploads are the
// main per-frame cost here, so each panel redraws only when its content changes.
import * as THREE from 'three';

const PX_PER_M = 1600;

export class Panel {
  constructor({ title, width = 0.34, height = 0.22, draw, accent = '#4f8cff' }) {
    this.title = title;
    this.drawBody = draw;
    this.accent = accent;
    this.focused = false;
    this.canvas = document.createElement('canvas');
    this.canvas.width = Math.round(width * PX_PER_M);
    this.canvas.height = Math.round(height * PX_PER_M);
    this.ctx = this.canvas.getContext('2d');
    this.texture = new THREE.CanvasTexture(this.canvas);
    this.texture.colorSpace = THREE.SRGBColorSpace;
    this.texture.anisotropy = 4;
    this.texture.generateMipmaps = false;
    this.texture.minFilter = THREE.LinearFilter;
    this.mesh = new THREE.Mesh(
      new THREE.PlaneGeometry(width, height),
      new THREE.MeshBasicMaterial({ map: this.texture, transparent: true, depthWrite: true }),
    );
    this.mesh.userData.panel = this;
    this.object = new THREE.Group();
    this.object.add(this.mesh);
    this.home = { position: new THREE.Vector3(), rotationY: 0 };
    this.redraw();
  }

  setHome(x, y, z, rotY) {
    this.home.position.set(x, y, z);
    this.home.rotationY = rotY;
    this.object.position.copy(this.home.position);
    this.object.rotation.y = rotY;
  }

  redraw() {
    const { ctx, canvas } = this;
    const W = canvas.width, H = canvas.height, r = 28;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = 'rgba(18, 22, 30, 0.88)';
    roundRect(ctx, 0, 0, W, H, r);
    ctx.fill();
    ctx.lineWidth = this.focused ? 10 : 4;
    ctx.strokeStyle = this.focused ? this.accent : 'rgba(255,255,255,0.18)';
    roundRect(ctx, ctx.lineWidth / 2, ctx.lineWidth / 2, W - ctx.lineWidth, H - ctx.lineWidth, r);
    ctx.stroke();
    // title bar
    ctx.fillStyle = this.accent;
    ctx.beginPath();
    ctx.arc(46, 46, 14, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = '#e8ecf4';
    ctx.font = '600 40px system-ui, sans-serif';
    ctx.textBaseline = 'middle';
    ctx.fillText(this.title, 76, 48);
    ctx.fillStyle = 'rgba(255,255,255,0.1)';
    ctx.fillRect(0, 92, W, 2);
    ctx.save();
    ctx.translate(32, 120);
    this.drawBody(ctx, W - 64, H - 150);
    ctx.restore();
    this.texture.needsUpdate = true;
  }

  /** animate towards home or the focused pose */
  tick(dt) {
    const k = 1 - Math.exp(-10 * dt);
    const s = this.focused ? 1.25 : 1;
    const z = this.home.position.z + (this.focused ? 0.06 : 0);
    this.object.scale.setScalar(THREE.MathUtils.lerp(this.object.scale.x, s, k));
    this.object.position.z = THREE.MathUtils.lerp(this.object.position.z, z, k);
  }
}

export function createWorkspace(getStats) {
  const root = new THREE.Group(); // placed by MarkerAnchor (origin = marker, +Z towards you)
  const layout = new THREE.Group(); // per-surface offset (table vs wall)
  root.add(layout);

  const clock = new Panel({
    title: 'Clock',
    accent: '#ffb547',
    draw(ctx, w, h) {
      const d = new Date();
      ctx.fillStyle = '#fff';
      ctx.font = '700 150px system-ui, sans-serif';
      ctx.textBaseline = 'top';
      ctx.fillText(d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }), 0, 10);
      ctx.fillStyle = '#9aa4b8';
      ctx.font = '400 44px system-ui, sans-serif';
      ctx.fillText(d.toLocaleDateString([], { weekday: 'long', month: 'long', day: 'numeric' }), 4, 190);
    },
  });

  const status = new Panel({
    title: 'Tracking',
    accent: '#3ddc97',
    draw(ctx) {
      const s = getStats();
      const rows = [
        ['Mode', s.mode],
        ['Marker', s.marker],
        ['Detect', s.detect],
        ['FPS', s.fps],
      ];
      ctx.textBaseline = 'top';
      rows.forEach(([k, v], i) => {
        ctx.fillStyle = '#9aa4b8';
        ctx.font = '400 40px system-ui, sans-serif';
        ctx.fillText(k, 0, i * 60);
        ctx.fillStyle = '#fff';
        ctx.font = '600 40px ui-monospace, monospace';
        ctx.fillText(String(v), 190, i * 60);
      });
    },
  });

  const notes = new Panel({
    title: 'Notes',
    accent: '#b58cff',
    draw(ctx, w) {
      ctx.fillStyle = '#d6dbe6';
      ctx.font = '400 36px system-ui, sans-serif';
      ctx.textBaseline = 'top';
      const text = 'Tap a window to bring it forward. Hide the marker: windows stay put (world tracking). ' +
        'Show it again: they re-lock to it.';
      wrap(ctx, text, w).forEach((line, i) => ctx.fillText(line, 0, i * 48));
    },
  });

  const panels = [notes, status, clock];
  const radius = 0.55;
  const spread = 0.62; // radians between panels
  panels.forEach((p, i) => {
    const a = (i - 1) * spread;
    // arc centred behind the marker, curving towards the viewer (+Z)
    p.setHome(Math.sin(a) * radius, 0, -Math.cos(a) * radius + radius, -a);
    layout.add(p.object);
  });

  // small marker outline so you can see what the system locked onto
  const markerGizmo = new THREE.LineSegments(
    new THREE.EdgesGeometry(new THREE.PlaneGeometry(1, 1).rotateX(-Math.PI / 2)),
    new THREE.LineBasicMaterial({ color: 0x3ddc97 }),
  );
  markerGizmo.visible = false;

  let surface = null;
  function setSurface(s) {
    if (s === surface) return;
    surface = s;
    // table: windows stand behind and above the marker. wall: windows centred on it.
    if (s === 'wall') layout.position.set(0, 0, 0.03);
    else layout.position.set(0, 0.2, -0.12);
  }
  setSurface('table');

  let clockMinute = -1;
  let statusTimer = 0;
  function tick(dt) {
    const m = new Date().getMinutes();
    if (m !== clockMinute) { clockMinute = m; clock.redraw(); }
    statusTimer += dt;
    if (statusTimer > 0.25) { statusTimer = 0; status.redraw(); }
    for (const p of panels) p.tick(dt);
  }

  function toggleFocus(panel) {
    for (const p of panels) {
      const f = p === panel ? !p.focused : false;
      if (f !== p.focused) { p.focused = f; p.redraw(); }
    }
  }

  return { root, panels, markerGizmo, setSurface, tick, toggleFocus, meshes: panels.map((p) => p.mesh) };
}

function roundRect(ctx, x, y, w, h, r) {
  ctx.beginPath();
  ctx.roundRect(x, y, w, h, r);
}

function wrap(ctx, text, maxW) {
  const lines = [];
  let line = '';
  for (const word of text.split(' ')) {
    const t = line ? line + ' ' + word : word;
    if (ctx.measureText(t).width > maxW && line) { lines.push(line); line = word; } else line = t;
  }
  if (line) lines.push(line);
  return lines;
}
