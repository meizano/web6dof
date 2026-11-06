# web6dof: marker-anchored WebXR workspace

Floating "app" windows that lock to a printed ArUco marker in 6DoF, built with WebXR + three.js.
The target is Android Chrome with ARCore.

| State | When | Behaviour |
|---|---|---|
| `searching` | before the marker is seen | windows float 0.8 m in front of you, 3DoF (rotation only) |
| `tracking` | marker visible | windows locked to the marker, 6DoF |
| `held` | marker gone (default) | windows stay where the marker was, kept in place by ARCore SLAM + an `XRAnchor`, still 6DoF |
| `3dof` | marker gone with `?lost=3dof`, or ARCore loses tracking | windows follow your head position, rotation stays world-fixed |

## Run on a phone

```bash
npm install
npm run dev
```

**Option A: USB.** No certificates needed, because localhost counts as a secure origin.
```bash
adb reverse tcp:5173 tcp:5173
```
Then open `http://localhost:5173` in Chrome on the phone.

**Option B: Wi-Fi.** `HTTPS=1 npm run dev` serves self-signed HTTPS. Open `https://<your-PC-LAN-IP>:5173` and accept the certificate warning.

Print `marker.html` at 100% scale and measure the black square. If it isn't 10 cm, pass the real size: `?size=0.085`.

URL options: `size` (metres), `id` (marker id, `-1` = any), `lost=3dof`, `w` (detection width, default 640).

## How it works (performance notes)

1. **Camera frames:** WebXR `camera-access` provides the camera image as a GPU texture. A shader downsamples it to 640 px wide, converts it to grayscale and packs 4 pixels per RGBA texel. The image is then read back through a PBO + fence, so the GPU→CPU copy (~230 KB) never stalls rendering ([camera-grabber.js](src/xr/camera-grabber.js)).
2. **Detection:** runs in a Web Worker: js-aruco2 finds candidate squares, then sub-pixel corner refinement, then planar PnP (homography + Levenberg-Marquardt) ([detect.js](src/cv/detect.js), [pose.js](src/cv/pose.js)). At most one readback and one detection are in flight at a time.
3. **Latency compensation:** every frame carries the camera pose *at capture time*, so the marker's world pose is right even though detection finishes a few frames later.
4. **Anchoring:** only position + yaw are taken from the marker. Gravity comes from ARCore, which removes most marker tilt jitter. Detections are smoothed, outliers are rejected, and a real move needs 3 detections that agree ([marker-anchor.js](src/xr/marker-anchor.js)).
5. **App windows:** canvas textures that redraw only when their content changes ([workspace.js](src/scene/workspace.js)).

## Tests

```bash
npm test
```
The test renders synthetic markers at known poses and checks the full detection + pose path (about 1–9 mm error up to 1.2 m).

## Known limits

- Needs Chrome 107+ on an ARCore device (raw camera access). iOS Safari and Quest do not support `camera-access`.
- Apps are WebGL panels. Real web pages cannot be rendered inside an immersive WebXR session.
- The camera texture's vertical orientation is auto-calibrated: the HUD shows `calibrating` until the first detection. The **Camera** button shows what the detector sees.
