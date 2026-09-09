// The seam between "what to draw" and "which API draws it".
//
// `GpuRenderer` builds one frame's worth of triangles and hands them over
// through this interface; the WebGL 2 and WebGPU backends behind it do nothing
// but move those triangles onto the screen. Keeping the seam this thin is what
// stops the two backends drifting into two different-looking games.

import type { Verts } from './mesh';

/** Which of the four things the fragment shader can do with a vertex. */
export const Mode = {
  /** Straight premultiplied colour. */
  Solid: 0,
  /** Colour times a texture sample at the vertex uv. */
  Tex: 1,
  /** An antialiased disc cut out of the quad, uv running -1..1 across it. */
  Disc: 2,
  /** Radial ramp from full colour to nothing, between params x and y. */
  Ramp: 3,
} as const;
export type Mode = (typeof Mode)[keyof typeof Mode];

export const Tex = {
  None: 0,
  Asphalt: 1,
  Marks: 2,
} as const;
export type Tex = (typeof Tex)[keyof typeof Tex];

/** An opaque vertex buffer. Backends put whatever they need behind it. */
export interface GpuMesh {
  /** Vertices currently uploaded. */
  verts: number;
}

/**
 * A 2D affine straight to clip space:
 *   clip.x = m[0]*x + m[1]*y + m[4]
 *   clip.y = m[2]*x + m[3]*y + m[5]
 */
export type Projection = Float32Array;

export interface GpuDevice {
  readonly backend: 'webgpu' | 'webgl2';
  readonly api: string;
  /** The GPU, abbreviated for the panel. */
  readonly device: string;
  /** Everything the driver would say, for the tooltip. */
  readonly detail: string;
  /**
   * Sign the mark-layer projection needs on its y axis.
   *
   * A render target's first row of texels is at clip y = -1 in GL and at
   * clip y = +1 in WebGPU. One number here is the whole difference, and it
   * saves the shared code from sampling the layer upside down on one backend.
   */
  readonly targetFlipY: number;
  /** Set when the GPU went away underneath us; the panel reports it. */
  readonly lost: string | null;

  createMesh(label: string): GpuMesh;
  upload(mesh: GpuMesh, data: Verts, floats: number): void;

  /** Resize to the drawing buffer and start recording this frame. */
  beginFrame(pxW: number, pxH: number): void;
  /**
   * Accumulate ink into the world-space mark layer, first fading what is
   * already there by `fade` (0 to leave it alone). Must come before the first
   * `draw` of the frame: the visible pass samples what this leaves behind.
   */
  paintMarks(mesh: GpuMesh, verts: number, proj: Projection, fade: number): void;
  setProjection(proj: Projection): void;
  draw(mesh: GpuMesh, first: number, count: number, mode: Mode, tex: Tex, p0?: number, p1?: number): void;
  endFrame(): void;
  dispose(): void;
}

/** Shared by both backends, so the two shaders cannot disagree about layout. */
export const VERTEX_STRIDE = 32; // 8 floats

/**
 * Trim a driver's renderer string down to something a 240-pixel panel can
 * show. The full version stays in `detail` for the tooltip.
 */
export function shortenGpu(name: string): string {
  let t = name.trim();
  // "ANGLE (NVIDIA, NVIDIA GeForce RTX 4070 Direct3D11 vs_5_0 ps_5_0, D3D11)"
  const angle = /^ANGLE \((.+)\)$/.exec(t);
  if (angle) {
    const parts = angle[1].split(', ');
    t = parts.length >= 2 ? parts[1] : angle[1];
  }
  t = t
    .replace(/\s+Direct3D\d*.*$/i, '')
    .replace(/\s+vs_\d_\d.*$/i, '')
    .replace(/\s+\(0x[0-9A-Fa-f]+\)/g, '')
    .replace(/\s+/g, ' ')
    .trim();
  return t.length > 44 ? `${t.slice(0, 43)}…` : t;
}

/**
 * The GPU's own name, out of a throwaway GL context.
 *
 * WebGPU redacts most of `GPUAdapter.info` unless the browser is told
 * otherwise, while WebGL has handed the unmasked string out for years. When
 * the adapter will not say what it is, this asks the other API about the same
 * card rather than showing the panel a blank.
 */
export function probeGpuName(): string | null {
  try {
    const probe = document.createElement('canvas');
    const gl = probe.getContext('webgl2') ?? probe.getContext('webgl');
    if (!gl) return null;
    const dbg = gl.getExtension('WEBGL_debug_renderer_info');
    const name = dbg ? gl.getParameter(dbg.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER);
    gl.getExtension('WEBGL_lose_context')?.loseContext();
    return name ? String(name) : null;
  } catch {
    return null;
  }
}
