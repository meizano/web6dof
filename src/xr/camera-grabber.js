// Pulls WebXR raw-camera frames to the CPU cheaply:
//  1. draw the camera texture into a small FBO, converting to grayscale and packing
//     4 pixels per RGBA texel (4x less bandwidth than RGBA readback)
//  2. readPixels into a PIXEL_PACK_BUFFER + fence, so the GPU->CPU copy is async
//  3. a later frame polls the fence and copies the bytes out without stalling.

const VS = `#version 300 es
out vec2 vUv;
void main() {
  vec2 p = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
  vUv = p;
  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}`;

const FS = `#version 300 es
precision highp float;
uniform sampler2D uCam;
uniform float uWidth;   // output image width in pixels (= 4 * FBO width)
uniform float uFlipY;   // 1.0: FBO row 0 (bottom) samples the top of the camera image
in vec2 vUv;
out vec4 outColor;
float lum(float px, float v) {
  vec3 c = texture(uCam, vec2((px + 0.5) / uWidth, v)).rgb;
  return dot(c, vec3(0.299, 0.587, 0.114));
}
void main() {
  float base = floor(gl_FragCoord.x) * 4.0;
  float v = mix(vUv.y, 1.0 - vUv.y, uFlipY);
  outColor = vec4(lum(base, v), lum(base + 1.0, v), lum(base + 2.0, v), lum(base + 3.0, v));
}`;

export class CameraGrabber {
  /** @param {WebGL2RenderingContext} gl */
  constructor(gl, { width = 640 } = {}) {
    this.gl = gl;
    this.targetWidth = width;
    this.flipY = true;
    this.w = 0;
    this.h = 0;
    this.pending = null; // { sync, meta }
    this.prog = this._program();
    this.uCam = gl.getUniformLocation(this.prog, 'uCam');
    this.uWidth = gl.getUniformLocation(this.prog, 'uWidth');
    this.uFlipY = gl.getUniformLocation(this.prog, 'uFlipY');
    this.vao = gl.createVertexArray();
  }

  get busy() {
    return this.pending !== null;
  }

  _program() {
    const gl = this.gl;
    const sh = (type, src) => {
      const s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
      return s;
    };
    const p = gl.createProgram();
    gl.attachShader(p, sh(gl.VERTEX_SHADER, VS));
    gl.attachShader(p, sh(gl.FRAGMENT_SHADER, FS));
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
    return p;
  }

  _resize(w, h) {
    const gl = this.gl;
    if (this.fbo) {
      gl.deleteFramebuffer(this.fbo);
      gl.deleteTexture(this.tex);
      gl.deleteBuffer(this.pbo);
    }
    this.w = w;
    this.h = h;
    this.tex = gl.createTexture();
    gl.bindTexture(gl.TEXTURE_2D, this.tex);
    gl.texStorage2D(gl.TEXTURE_2D, 1, gl.RGBA8, w / 4, h);
    this.fbo = gl.createFramebuffer();
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.fbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.tex, 0);
    this.pbo = gl.createBuffer();
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, this.pbo);
    gl.bufferData(gl.PIXEL_PACK_BUFFER, w * h, gl.STREAM_READ);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
  }

  /**
   * Start an async capture. Must be called inside the XR frame callback.
   * @param {WebGLTexture} camTex   from XRWebGLBinding.getCameraImage()
   * @param {number} camW, camH     XRCamera width/height
   * @param {object} meta           anything to carry along (pose, projection, ...)
   */
  capture(camTex, camW, camH, meta) {
    if (this.pending) return false;
    const gl = this.gl;
    const w = Math.min(this.targetWidth, camW) & ~3;
    const h = Math.round((w * camH) / camW);
    if (w !== this.w || h !== this.h) this._resize(w, h);

    gl.bindFramebuffer(gl.FRAMEBUFFER, this.fbo);
    gl.viewport(0, 0, w / 4, h);
    gl.disable(gl.DEPTH_TEST);
    gl.disable(gl.BLEND);
    gl.disable(gl.CULL_FACE);
    gl.disable(gl.SCISSOR_TEST);
    gl.colorMask(true, true, true, true);
    gl.useProgram(this.prog);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, camTex);
    gl.uniform1i(this.uCam, 0);
    gl.uniform1f(this.uWidth, w);
    gl.uniform1f(this.uFlipY, this.flipY ? 1 : 0);
    gl.bindVertexArray(this.vao);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
    gl.bindVertexArray(null);

    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, this.pbo);
    gl.readPixels(0, 0, w / 4, h, gl.RGBA, gl.UNSIGNED_BYTE, 0);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
    const sync = gl.fenceSync(gl.SYNC_GPU_COMMANDS_COMPLETE, 0);
    gl.flush();
    this.pending = { sync, meta: { ...meta, w, h, flipY: this.flipY } };
    return true;
  }

  /**
   * If the last capture finished, copy it into `dst` (Uint8Array, length >= w*h).
   * Rows come out bottom-up from readPixels; because the shader already flipped,
   * row 0 is the top of the camera image.
   * @returns meta of the finished capture, or null if not ready.
   */
  poll(dstFactory) {
    if (!this.pending) return null;
    const gl = this.gl;
    const status = gl.getSyncParameter(this.pending.sync, gl.SYNC_STATUS);
    if (status !== gl.SIGNALED) return null;
    gl.deleteSync(this.pending.sync);
    const { meta } = this.pending;
    this.pending = null;
    if (meta.w !== this.w || meta.h !== this.h) return null; // resized meanwhile
    const dst = dstFactory(meta.w * meta.h);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, this.pbo);
    gl.getBufferSubData(gl.PIXEL_PACK_BUFFER, 0, dst, 0, meta.w * meta.h);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
    meta.pixels = dst;
    return meta;
  }
}
