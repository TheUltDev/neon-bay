// WebGL 2 backend.
//
// One program, one vertex format, one blend mode. Everything the renderer can
// draw is a mode switch in the fragment shader, so a frame is a handful of
// `drawArrays` calls with a uniform change between them and no pipeline churn
// at all.
//
// Multisampling comes free here: the default framebuffer is asked for it at
// context creation, which is what keeps the barrier ribbons and the car
// outlines from looking like stairs next to the 2D renderer's antialiased
// strokes.

import { MARK_SIZE } from './base';
import { Mode, shortenGpu, Tex, VERTEX_STRIDE, type GpuDevice, type GpuMesh, type Projection } from './device';
import type { Verts } from './mesh';
import { ASPHALT_TILE, asphaltPixels } from './palette';

const VERT = `#version 300 es
in vec2 aPos;
in vec2 aUV;
in vec4 aCol;
uniform vec4 uX;
uniform vec2 uT;
out vec2 vUV;
out vec4 vCol;
void main() {
  vUV = aUV;
  vCol = aCol;
  gl_Position = vec4(uX.xy * aPos.x + uX.zw * aPos.y + uT, 0.0, 1.0);
}`;

const FRAG = `#version 300 es
precision highp float;
in vec2 vUV;
in vec4 vCol;
uniform int uMode;
uniform vec2 uP;
uniform sampler2D uTex;
out vec4 frag;
void main() {
  vec4 c = vCol;
  if (uMode == 1) {
    c *= texture(uTex, vUV);
  } else if (uMode == 2) {
    // A disc with one pixel of falloff at its rim, which is what a filled
    // arc in a 2D context comes out as.
    float d = length(vUV);
    c *= 1.0 - smoothstep(1.0 - fwidth(d) * 1.5, 1.0, d);
  } else if (uMode == 3) {
    c *= 1.0 - clamp((length(vUV) - uP.x) / max(uP.y - uP.x, 1e-4), 0.0, 1.0);
  }
  frag = c;
}`;

interface GlMesh extends GpuMesh {
  buf: WebGLBuffer;
  vao: WebGLVertexArrayObject;
  bytes: number;
}

export function createWebgl2Device(canvas: HTMLCanvasElement): GpuDevice | null {
  const gl = canvas.getContext('webgl2', {
    alpha: false,
    antialias: true,
    depth: false,
    stencil: false,
    powerPreference: 'high-performance',
  });
  if (!gl) return null;
  try {
    return new Webgl2Device(gl);
  } catch (e) {
    console.warn('webgl2 setup failed', e);
    return null;
  }
}

class Webgl2Device implements GpuDevice {
  readonly backend = 'webgl2' as const;
  readonly api = 'WebGL 2';
  readonly device: string;
  readonly detail: string;
  /** A GL render target's first row of texels sits at clip y = -1. */
  readonly targetFlipY = 1;

  private gl: WebGL2RenderingContext;
  private prog: WebGLProgram;
  private uX: WebGLUniformLocation;
  private uT: WebGLUniformLocation;
  private uMode: WebGLUniformLocation;
  private uP: WebGLUniformLocation;

  private asphalt: WebGLTexture;
  private blank: WebGLTexture;
  private marksTex: WebGLTexture;
  private marksFbo: WebGLFramebuffer;
  private fadeMesh: GlMesh;
  private fadeAlpha = -1;

  private viewW = 1;
  private viewH = 1;
  private contextLost: string | null = null;

  constructor(gl: WebGL2RenderingContext) {
    this.gl = gl;
    canvasOf(gl).addEventListener('webglcontextlost', () => {
      this.contextLost = 'WebGL context lost — reload to get the renderer back';
    });

    this.prog = link(gl, VERT, FRAG);
    gl.useProgram(this.prog);
    this.uX = loc(gl, this.prog, 'uX');
    this.uT = loc(gl, this.prog, 'uT');
    this.uMode = loc(gl, this.prog, 'uMode');
    this.uP = loc(gl, this.prog, 'uP');
    gl.uniform1i(loc(gl, this.prog, 'uTex'), 0);

    gl.disable(gl.DEPTH_TEST);
    gl.disable(gl.CULL_FACE);
    gl.enable(gl.BLEND);
    gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA);

    this.asphalt = tex2d(gl, ASPHALT_TILE, ASPHALT_TILE, new Uint8Array(asphaltPixels().buffer), gl.REPEAT);
    // Bound by every draw that samples nothing. Without it the mark layer is
    // still on the texture unit while the mark pass renders into it, which is
    // a feedback loop: the driver is entitled to drop the draw, and does.
    this.blank = tex2d(gl, 1, 1, new Uint8Array([255, 255, 255, 255]), gl.CLAMP_TO_EDGE);
    this.marksTex = tex2d(gl, MARK_SIZE, MARK_SIZE, null, gl.CLAMP_TO_EDGE);
    const fbo = gl.createFramebuffer();
    if (!fbo) throw new Error('no framebuffer for the mark layer');
    this.marksFbo = fbo;
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.marksFbo);
    gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.marksTex, 0);
    gl.clearColor(0, 0, 0, 0);
    gl.clear(gl.COLOR_BUFFER_BIT);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);

    this.fadeMesh = this.createMesh('mark-fade') as GlMesh;

    const dbg = gl.getExtension('WEBGL_debug_renderer_info');
    const name = String(
      (dbg && gl.getParameter(dbg.UNMASKED_RENDERER_WEBGL)) || gl.getParameter(gl.RENDERER) || 'unknown',
    );
    const vendor = String((dbg && gl.getParameter(dbg.UNMASKED_VENDOR_WEBGL)) || gl.getParameter(gl.VENDOR) || '');
    this.device = shortenGpu(name);
    this.detail = [name, vendor, gl.getParameter(gl.VERSION)].filter(Boolean).join(' · ');
  }

  get lost(): string | null {
    return this.contextLost;
  }

  // -------------------------------------------------------------- buffers --

  createMesh(_label: string): GpuMesh {
    const gl = this.gl;
    const buf = gl.createBuffer();
    const vao = gl.createVertexArray();
    if (!buf || !vao) throw new Error('out of GL objects');
    gl.bindVertexArray(vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, buf);
    // x y | u v | r g b a
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 2, gl.FLOAT, false, VERTEX_STRIDE, 0);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 2, gl.FLOAT, false, VERTEX_STRIDE, 8);
    gl.enableVertexAttribArray(2);
    gl.vertexAttribPointer(2, 4, gl.FLOAT, false, VERTEX_STRIDE, 16);
    gl.bindVertexArray(null);
    return { verts: 0, buf, vao, bytes: 0 } as GlMesh;
  }

  upload(mesh: GpuMesh, data: Verts, floats: number) {
    const gl = this.gl;
    const m = mesh as GlMesh;
    m.verts = floats / 8;
    if (floats === 0) return;
    gl.bindBuffer(gl.ARRAY_BUFFER, m.buf);
    const need = floats * 4;
    if (need > m.bytes) {
      // Round up so a buffer that grows a little does not reallocate a lot.
      m.bytes = Math.max(need, m.bytes * 2, 4096);
      gl.bufferData(gl.ARRAY_BUFFER, m.bytes, gl.DYNAMIC_DRAW);
    }
    gl.bufferSubData(gl.ARRAY_BUFFER, 0, data, 0, floats);
  }

  // ---------------------------------------------------------------- frame --

  beginFrame(pxW: number, pxH: number) {
    const gl = this.gl;
    this.viewW = pxW;
    this.viewH = pxH;
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, pxW, pxH);
    gl.useProgram(this.prog);
  }

  paintMarks(mesh: GpuMesh, verts: number, proj: Projection, fade: number) {
    const gl = this.gl;
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.marksFbo);
    gl.viewport(0, 0, MARK_SIZE, MARK_SIZE);

    if (fade > 0) {
      // Scale what is already there towards nothing: the GL spelling of the 2D
      // renderer's `destination-out` wash.
      if (this.fadeAlpha !== fade) {
        this.fadeAlpha = fade;
        this.upload(this.fadeMesh, clipQuad(fade), 48);
      }
      gl.blendFunc(gl.ZERO, gl.ONE_MINUS_SRC_ALPHA);
      this.setProjection(IDENTITY);
      this.draw(this.fadeMesh, 0, 6, Mode.Solid, Tex.None);
      gl.blendFunc(gl.ONE, gl.ONE_MINUS_SRC_ALPHA);
    }
    if (verts > 0) {
      this.setProjection(proj);
      this.draw(mesh, 0, verts, Mode.Solid, Tex.None);
    }

    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.viewport(0, 0, this.viewW, this.viewH);
  }

  setProjection(p: Projection) {
    this.gl.uniform4f(this.uX, p[0], p[2], p[1], p[3]);
    this.gl.uniform2f(this.uT, p[4], p[5]);
  }

  draw(mesh: GpuMesh, first: number, count: number, mode: Mode, tex: Tex, p0 = 0, p1 = 1) {
    if (count <= 0) return;
    const gl = this.gl;
    const m = mesh as GlMesh;
    gl.uniform1i(this.uMode, mode);
    if (mode === Mode.Ramp) gl.uniform2f(this.uP, p0, p1);
    gl.activeTexture(gl.TEXTURE0);
    gl.bindTexture(gl.TEXTURE_2D, tex === Tex.Asphalt ? this.asphalt : tex === Tex.Marks ? this.marksTex : this.blank);
    gl.bindVertexArray(m.vao);
    gl.drawArrays(gl.TRIANGLES, first, count);
  }

  endFrame() {
    this.gl.bindVertexArray(null);
  }

  dispose() {
    const gl = this.gl;
    gl.deleteProgram(this.prog);
    gl.deleteTexture(this.asphalt);
    gl.deleteTexture(this.blank);
    gl.deleteTexture(this.marksTex);
    gl.deleteFramebuffer(this.marksFbo);
    gl.getExtension('WEBGL_lose_context')?.loseContext();
  }
}

// ------------------------------------------------------------------ utils --

const IDENTITY = new Float32Array([1, 0, 0, 1, 0, 0]);

/** Six vertices covering clip space, in black at `alpha`. */
function clipQuad(alpha: number): Verts {
  const v = new Float32Array(48);
  const corners = [
    [-1, -1], [1, -1], [1, 1],
    [-1, -1], [1, 1], [-1, 1],
  ];
  for (let i = 0; i < 6; i++) {
    v[i * 8] = corners[i][0];
    v[i * 8 + 1] = corners[i][1];
    v[i * 8 + 7] = alpha;
  }
  return v;
}

function canvasOf(gl: WebGL2RenderingContext): HTMLCanvasElement {
  return gl.canvas as HTMLCanvasElement;
}

function link(gl: WebGL2RenderingContext, vs: string, fs: string): WebGLProgram {
  const prog = gl.createProgram();
  if (!prog) throw new Error('no GL program');
  for (const [type, src] of [
    [gl.VERTEX_SHADER, vs],
    [gl.FRAGMENT_SHADER, fs],
  ] as const) {
    const sh = gl.createShader(type);
    if (!sh) throw new Error('no GL shader');
    gl.shaderSource(sh, src);
    gl.compileShader(sh);
    if (!gl.getShaderParameter(sh, gl.COMPILE_STATUS)) {
      throw new Error(`shader: ${gl.getShaderInfoLog(sh)}`);
    }
    gl.attachShader(prog, sh);
    gl.deleteShader(sh);
  }
  // Fixed slots, so every mesh can share one attribute layout.
  gl.bindAttribLocation(prog, 0, 'aPos');
  gl.bindAttribLocation(prog, 1, 'aUV');
  gl.bindAttribLocation(prog, 2, 'aCol');
  gl.linkProgram(prog);
  if (!gl.getProgramParameter(prog, gl.LINK_STATUS)) {
    throw new Error(`link: ${gl.getProgramInfoLog(prog)}`);
  }
  return prog;
}

function loc(gl: WebGL2RenderingContext, prog: WebGLProgram, name: string): WebGLUniformLocation {
  const l = gl.getUniformLocation(prog, name);
  if (!l) throw new Error(`uniform ${name} went missing`);
  return l;
}

function tex2d(
  gl: WebGL2RenderingContext,
  w: number,
  h: number,
  pixels: Uint8Array | null,
  wrap: number,
): WebGLTexture {
  const t = gl.createTexture();
  if (!t) throw new Error('no GL texture');
  gl.bindTexture(gl.TEXTURE_2D, t);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA8, w, h, 0, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, wrap);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, wrap);
  return t;
}
