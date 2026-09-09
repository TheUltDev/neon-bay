// WebGPU backend: the top tier, and the default wherever the browser has it.
//
// The same single shader as the GL path, in WGSL, with the mode switch moved
// into a uniform block. What differs is the plumbing rather than the picture:
//
//   * uniforms come from one buffer addressed with dynamic offsets, so a frame
//     writes every projection once, up front, and the pass just re-points at
//     them. No per-draw uniform traffic.
//   * multisampling is explicit here where GL hands it over with the default
//     framebuffer, so the frame renders into a 4x target and resolves into the
//     swapchain texture on the way out.
//   * a render target's first row of texels is at clip y = +1 rather than -1,
//     which `targetFlipY` tells the shared code so the mark layer is not
//     sampled upside down.

import { MARK_SIZE } from './base';
import {
  Mode,
  probeGpuName,
  shortenGpu,
  Tex,
  VERTEX_STRIDE,
  type GpuDevice,
  type GpuMesh,
  type Projection,
} from './device';
import type { Verts } from './mesh';
import { ASPHALT_TILE, asphaltPixels } from './palette';

const SHADER = `
struct U {
  xform: vec4f,   // clip = xform.xy * pos.x + xform.zw * pos.y + off.xy
  off: vec4f,     // off.z carries the draw mode
  params: vec4f,  // params.xy bound the radial ramp
};

@group(0) @binding(0) var<uniform> u: U;
@group(0) @binding(1) var samp: sampler;
@group(0) @binding(2) var tex: texture_2d<f32>;

struct VSOut {
  @builtin(position) pos: vec4f,
  @location(0) uv: vec2f,
  @location(1) col: vec4f,
};

@vertex
fn vs(@location(0) pos: vec2f, @location(1) uv: vec2f, @location(2) col: vec4f) -> VSOut {
  var out: VSOut;
  out.pos = vec4f(u.xform.xy * pos.x + u.xform.zw * pos.y + u.off.xy, 0.0, 1.0);
  out.uv = uv;
  out.col = col;
  return out;
}

@fragment
fn fs(in: VSOut) -> @location(0) vec4f {
  let mode = i32(u.off.z);
  var c = in.col;
  // Sampled unconditionally and selected afterwards: a texture fetch inside a
  // branch is only legal in uniform control flow, and this costs nothing.
  let t = textureSample(tex, samp, in.uv);
  c = select(c, c * t, mode == 1);
  let d = length(in.uv);
  let disc = 1.0 - smoothstep(1.0 - fwidth(d) * 1.5, 1.0, d);
  c = select(c, c * disc, mode == 2);
  let ramp = 1.0 - clamp((d - u.params.x) / max(u.params.y - u.params.x, 1e-4), 0.0, 1.0);
  c = select(c, c * ramp, mode == 3);
  return c;
}
`;

/** Uniform blocks per frame, at the 256-byte offset alignment WebGPU wants. */
const BLOCK_FLOATS = 64;
const BLOCKS = 64;
const SAMPLES = 4;

interface WgpuMesh extends GpuMesh {
  buf: GPUBuffer | null;
  bytes: number;
}

export async function createWebgpuDevice(canvas: HTMLCanvasElement): Promise<GpuDevice> {
  const gpu = navigator.gpu;
  if (!gpu) throw new Error('not available in this browser');
  const adapter = await gpu.requestAdapter({ powerPreference: 'high-performance' });
  if (!adapter) throw new Error('no adapter offered');
  const device = await adapter.requestDevice();
  const ctx = canvas.getContext('webgpu');
  if (!ctx) throw new Error('the canvas would not give up a context');
  return new WebgpuDevice(adapter, device, ctx);
}

class WebgpuDevice implements GpuDevice {
  readonly backend = 'webgpu' as const;
  readonly api = 'WebGPU';
  readonly device: string;
  readonly detail: string;
  /** A WebGPU render target's first row of texels sits at clip y = +1. */
  readonly targetFlipY = -1;

  private gpu: GPUDevice;
  private ctx: GPUCanvasContext;
  private format: GPUTextureFormat;

  private mainPipe: GPURenderPipeline;
  private markPipe: GPURenderPipeline;
  private erasePipe: GPURenderPipeline;

  private uniforms: GPUBuffer;
  private staging = new Float32Array(BLOCK_FLOATS * BLOCKS);
  private block = 0;

  private groups: GPUBindGroup[] = [];
  private marksTex: GPUTexture;
  private msaa: GPUTexture | null = null;
  private msaaW = 0;
  private msaaH = 0;

  private fade: WgpuMesh;
  private fadeAlpha = -1;

  private encoder: GPUCommandEncoder | null = null;
  private pass: GPURenderPassEncoder | null = null;
  private proj = new Float32Array(6);
  private gone: string | null = null;

  constructor(adapter: GPUAdapter, device: GPUDevice, ctx: GPUCanvasContext) {
    this.gpu = device;
    this.ctx = ctx;
    this.format = navigator.gpu.getPreferredCanvasFormat();
    ctx.configure({ device, format: this.format, alphaMode: 'opaque' });

    device.lost.then((info) => {
      this.gone = `WebGPU device lost: ${info.reason ?? 'unknown'} — reload to get the renderer back`;
    });
    device.addEventListener('uncapturederror', (e) => {
      console.error('webgpu:', (e as GPUUncapturedErrorEvent).error.message);
    });

    const module = device.createShaderModule({ code: SHADER, label: 'neon' });
    const layout = device.createBindGroupLayout({
      entries: [
        { binding: 0, visibility: GPUShaderStage.VERTEX | GPUShaderStage.FRAGMENT, buffer: { type: 'uniform', hasDynamicOffset: true, minBindingSize: 48 } },
        { binding: 1, visibility: GPUShaderStage.FRAGMENT, sampler: { type: 'filtering' } },
        { binding: 2, visibility: GPUShaderStage.FRAGMENT, texture: { sampleType: 'float' } },
      ],
    });
    const pipelineLayout = device.createPipelineLayout({ bindGroupLayouts: [layout] });
    const buffers: GPUVertexBufferLayout[] = [
      {
        arrayStride: VERTEX_STRIDE,
        attributes: [
          { shaderLocation: 0, offset: 0, format: 'float32x2' },
          { shaderLocation: 1, offset: 8, format: 'float32x2' },
          { shaderLocation: 2, offset: 16, format: 'float32x4' },
        ],
      },
    ];
    const premultiplied: GPUBlendState = {
      color: { srcFactor: 'one', dstFactor: 'one-minus-src-alpha', operation: 'add' },
      alpha: { srcFactor: 'one', dstFactor: 'one-minus-src-alpha', operation: 'add' },
    };
    // Scale the destination towards nothing and add none of the source: the
    // WebGPU spelling of `destination-out`, which is how the mark layer fades.
    const erase: GPUBlendState = {
      color: { srcFactor: 'zero', dstFactor: 'one-minus-src-alpha', operation: 'add' },
      alpha: { srcFactor: 'zero', dstFactor: 'one-minus-src-alpha', operation: 'add' },
    };
    const pipe = (format: GPUTextureFormat, blend: GPUBlendState, samples: number) =>
      device.createRenderPipeline({
        layout: pipelineLayout,
        vertex: { module, entryPoint: 'vs', buffers },
        fragment: { module, entryPoint: 'fs', targets: [{ format, blend }] },
        primitive: { topology: 'triangle-list' },
        multisample: { count: samples },
      });
    this.mainPipe = pipe(this.format, premultiplied, SAMPLES);
    this.markPipe = pipe('rgba8unorm', premultiplied, 1);
    this.erasePipe = pipe('rgba8unorm', erase, 1);

    this.uniforms = device.createBuffer({
      size: BLOCK_FLOATS * BLOCKS * 4,
      usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST,
      label: 'draw-state',
    });

    // Textures: the road tile, the mark layer, and a white pixel for every
    // draw that does not sample anything (the binding still has to exist).
    const asphalt = device.createTexture({
      size: [ASPHALT_TILE, ASPHALT_TILE],
      format: 'rgba8unorm',
      usage: GPUTextureUsage.TEXTURE_BINDING | GPUTextureUsage.COPY_DST,
    });
    device.queue.writeTexture(
      { texture: asphalt },
      asphaltPixels(),
      { bytesPerRow: ASPHALT_TILE * 4 },
      [ASPHALT_TILE, ASPHALT_TILE],
    );
    this.marksTex = device.createTexture({
      size: [MARK_SIZE, MARK_SIZE],
      format: 'rgba8unorm',
      usage: GPUTextureUsage.TEXTURE_BINDING | GPUTextureUsage.RENDER_ATTACHMENT,
    });
    const blank = device.createTexture({
      size: [1, 1],
      format: 'rgba8unorm',
      usage: GPUTextureUsage.TEXTURE_BINDING | GPUTextureUsage.COPY_DST,
    });
    device.queue.writeTexture({ texture: blank }, new Uint8Array([255, 255, 255, 255]), { bytesPerRow: 4 }, [1, 1]);

    const sampler = device.createSampler({
      magFilter: 'linear',
      minFilter: 'linear',
      addressModeU: 'repeat',
      addressModeV: 'repeat',
    });
    const group = (t: GPUTexture) =>
      device.createBindGroup({
        layout,
        entries: [
          { binding: 0, resource: { buffer: this.uniforms, size: 48 } },
          { binding: 1, resource: sampler },
          { binding: 2, resource: t.createView() },
        ],
      });
    this.groups[Tex.None] = group(blank);
    this.groups[Tex.Asphalt] = group(asphalt);
    this.groups[Tex.Marks] = group(this.marksTex);

    this.fade = this.createMesh('mark-fade') as WgpuMesh;

    // Chrome hands out a redacted adapter; the same card will still introduce
    // itself properly through WebGL, so ask there when it does.
    const info = adapter.info as GPUAdapterInfo | undefined;
    const named = [info?.description, info?.device, info?.architecture, info?.vendor].filter(
      (s): s is string => !!s && s.length > 0,
    );
    const probed = named.length === 0 || (named.length <= 2 && !named[0].includes(' ')) ? probeGpuName() : null;
    this.device = shortenGpu(probed ?? named[0] ?? 'unnamed adapter');
    this.detail = [probed, ...named].filter(Boolean).join(' · ') || 'adapter details withheld';
  }

  get lost(): string | null {
    return this.gone;
  }

  // -------------------------------------------------------------- buffers --

  createMesh(label: string): GpuMesh {
    return { verts: 0, buf: null, bytes: 0, label } as WgpuMesh & { label: string };
  }

  upload(mesh: GpuMesh, data: Verts, floats: number) {
    const m = mesh as WgpuMesh;
    m.verts = floats / 8;
    if (floats === 0) return;
    const need = floats * 4;
    if (!m.buf || need > m.bytes) {
      m.buf?.destroy();
      m.bytes = Math.max(need, m.bytes * 2, 4096);
      m.buf = this.gpu.createBuffer({
        size: Math.ceil(m.bytes / 4) * 4,
        usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST,
      });
    }
    this.gpu.queue.writeBuffer(m.buf, 0, data, 0, floats);
  }

  // ---------------------------------------------------------------- frame --

  beginFrame(pxW: number, pxH: number) {
    if (this.gone) return;
    this.block = 0;
    if (!this.msaa || this.msaaW !== pxW || this.msaaH !== pxH) {
      this.msaa?.destroy();
      this.msaaW = pxW;
      this.msaaH = pxH;
      this.msaa = this.gpu.createTexture({
        size: [pxW, pxH],
        format: this.format,
        sampleCount: SAMPLES,
        usage: GPUTextureUsage.RENDER_ATTACHMENT,
      });
    }
    this.encoder = this.gpu.createCommandEncoder();
  }

  paintMarks(mesh: GpuMesh, verts: number, proj: Projection, fade: number) {
    if (!this.encoder) return;
    const pass = this.encoder.beginRenderPass({
      colorAttachments: [{ view: this.marksTex.createView(), loadOp: 'load', storeOp: 'store' }],
    });
    if (fade > 0) {
      if (this.fadeAlpha !== fade) {
        this.fadeAlpha = fade;
        this.upload(this.fade, clipQuad(fade), 48);
      }
      pass.setPipeline(this.erasePipe);
      this.setProjection(IDENTITY);
      this.record(pass, this.fade, 0, 6, Mode.Solid, Tex.None, 0, 1);
    }
    if (verts > 0) {
      pass.setPipeline(this.markPipe);
      this.setProjection(proj);
      this.record(pass, mesh, 0, verts, Mode.Solid, Tex.None, 0, 1);
    }
    pass.end();
  }

  setProjection(p: Projection) {
    this.proj.set(p);
  }

  draw(mesh: GpuMesh, first: number, count: number, mode: Mode, tex: Tex, p0 = 0, p1 = 1) {
    if (count <= 0 || !this.encoder) return;
    if (!this.pass) {
      if (!this.msaa) return;
      this.pass = this.encoder.beginRenderPass({
        colorAttachments: [
          {
            view: this.msaa.createView(),
            resolveTarget: this.ctx.getCurrentTexture().createView(),
            clearValue: { r: 0, g: 0, b: 0, a: 1 },
            loadOp: 'clear',
            // Only the resolve is ever read, so the 4x samples can go.
            storeOp: 'discard',
          },
        ],
      });
      this.pass.setPipeline(this.mainPipe);
    }
    this.record(this.pass, mesh, first, count, mode, tex, p0, p1);
  }

  /** One draw, with its own slice of the uniform buffer. */
  private record(
    pass: GPURenderPassEncoder,
    mesh: GpuMesh,
    first: number,
    count: number,
    mode: Mode,
    tex: Tex,
    p0: number,
    p1: number,
  ) {
    const m = mesh as WgpuMesh;
    if (!m.buf || this.block >= BLOCKS) return;
    const o = this.block * BLOCK_FLOATS;
    const s = this.staging;
    s[o] = this.proj[0];
    s[o + 1] = this.proj[2];
    s[o + 2] = this.proj[1];
    s[o + 3] = this.proj[3];
    s[o + 4] = this.proj[4];
    s[o + 5] = this.proj[5];
    s[o + 6] = mode;
    s[o + 7] = 0;
    s[o + 8] = p0;
    s[o + 9] = p1;
    pass.setBindGroup(0, this.groups[tex], [this.block * BLOCK_FLOATS * 4]);
    pass.setVertexBuffer(0, m.buf);
    pass.draw(count, 1, first, 0);
    this.block++;
  }

  endFrame() {
    if (!this.encoder) return;
    this.pass?.end();
    this.pass = null;
    // The uniform data lands on the queue ahead of the commands that read it.
    this.gpu.queue.writeBuffer(this.uniforms, 0, this.staging, 0, Math.max(1, this.block) * BLOCK_FLOATS);
    this.gpu.queue.submit([this.encoder.finish()]);
    this.encoder = null;
  }

  dispose() {
    this.msaa?.destroy();
    this.marksTex.destroy();
    this.uniforms.destroy();
    this.gpu.destroy();
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
