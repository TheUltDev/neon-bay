// Canvas2D renderer: the fallback tier, and the reference the other two match.
//
// Everything here is drawn from the same wasm world the netcode maintains, so
// what you see is literally the simulation state -- including the "server ghost"
// overlay, which draws the authority's pose next to the prediction.
//
// Static geometry (track surface, curbs, start line) is baked into Path2D
// objects once at load. Skid marks accumulate into an offscreen world-space
// canvas so they persist without a growing list of quads.
//
// Two rules keep it quick. Canvas2D hides an enormous range of cost behind
// calls that all look alike, and how much it hides varies by browser -- what
// Chrome absorbs on the GPU, a software rasterizer pays for line by line:
//
//   * no `shadowBlur` on anything larger than a car. A blurred shadow is
//     rendered into a scratch surface the size of the path's bounding box, and
//     the track's bounding box is the entire circuit. Stroking the two barriers
//     that way measured at 16 ms a frame on its own; the stacked strokes that
//     replaced it come to 0.5 ms.
//   * geometry that never moves is baked at load and drawn as a path, not
//     rebuilt from a loop every frame. That is the chequered flag, and it is
//     why the road surface is one fill of a pre-tinted tile rather than a fill
//     plus a glaze.
//
// What is *not* worth doing, measured rather than assumed: caching the
// full-screen gradients (the fill dominates, not building the ramp), and
// clipping the skid layer's blit to the visible corner (the cost is
// destination pixels, not source ones).

import type { Sim } from '../sim';
import { BaseRenderer, MARK_SIZE, TIRE_WIDTH } from './base';
import { ASPHALT_TILE, asphaltPixels, CAR_OUTLINE, EDGE_GLOW, shade } from './palette';
import type { DrawCar, GhostCar, RendererInfo } from './types';

export class Canvas2DRenderer extends BaseRenderer {
  readonly info: RendererInfo;
  private ctx: CanvasRenderingContext2D;

  // baked geometry
  private surface = new Path2D();
  private edgeL = new Path2D();
  private edgeR = new Path2D();
  private curbA = new Path2D();
  private curbB = new Path2D();
  private centerLine = new Path2D();
  private startLine = new Path2D();
  private chequerA = new Path2D();
  private chequerB = new Path2D();

  // world-space layers
  private marks: HTMLCanvasElement;
  private marksCtx: CanvasRenderingContext2D;
  private asphalt: CanvasPattern | null = null;

  // cached paint
  private bodyGrad = new Map<number, CanvasGradient>();
  private beamGrad: CanvasGradient | null = null;

  private lastMarkFade = 0;

  constructor(canvas: HTMLCanvasElement, sim: Sim, fallback: string | null) {
    super(canvas, sim);
    const ctx = canvas.getContext('2d', { alpha: false });
    if (!ctx) throw new Error('canvas 2d unavailable');
    this.ctx = ctx;
    this.info = {
      backend: 'canvas2d',
      api: 'Canvas 2D',
      device: 'browser 2D rasterizer',
      detail: 'CanvasRenderingContext2D — paths and fills, wherever this browser rasterizes them',
      fallback,
    };

    this.marks = document.createElement('canvas');
    this.marks.width = MARK_SIZE;
    this.marks.height = MARK_SIZE;
    this.marksCtx = this.marks.getContext('2d')!;

    this.bakeTrack();
    this.makeAsphalt();
  }

  // ------------------------------------------------------------ geometry --

  private bakeTrack() {
    const { points, tangents, halfWidth, samples } = this.track;
    const edge = (i: number, side: number) => {
      const nx = -tangents[i * 2 + 1];
      const ny = tangents[i * 2];
      const w = halfWidth[i] * side;
      return [points[i * 2] + nx * w, points[i * 2 + 1] + ny * w] as const;
    };

    // Surface: out along the left edge, back along the right.
    for (let i = 0; i <= samples; i++) {
      const [x, y] = edge(i % samples, 1);
      i === 0 ? this.surface.moveTo(x, y) : this.surface.lineTo(x, y);
    }
    for (let i = samples; i >= 0; i--) {
      const [x, y] = edge(i % samples, -1);
      this.surface.lineTo(x, y);
    }
    this.surface.closePath();

    for (const [path, side] of [
      [this.edgeL, 1],
      [this.edgeR, -1],
    ] as const) {
      for (let i = 0; i <= samples; i++) {
        const [x, y] = edge(i % samples, side);
        i === 0 ? path.moveTo(x, y) : path.lineTo(x, y);
      }
      path.closePath();
    }

    // Curbs: alternating blocks just inside each edge, only where it bends.
    const KERB = 1.25;
    for (let i = 0; i < samples; i++) {
      const bend = Math.abs(this.track.curvature[i]);
      if (bend < 0.0055) continue;
      const path = (i >> 2) % 2 === 0 ? this.curbA : this.curbB;
      for (const side of [1, -1]) {
        const j = (i + 1) % samples;
        const [ax, ay] = edge(i, side);
        const [bx, by] = edge(j, side);
        const [cx, cy] = edgeInset(edge(j, side), points, j, KERB);
        const [dx, dy] = edgeInset(edge(i, side), points, i, KERB);
        path.moveTo(ax, ay);
        path.lineTo(bx, by);
        path.lineTo(cx, cy);
        path.lineTo(dx, dy);
        path.closePath();
      }
    }

    for (let i = 0; i <= samples; i += 2) {
      const k = i % samples;
      const x = points[k * 2];
      const y = points[k * 2 + 1];
      i === 0 ? this.centerLine.moveTo(x, y) : this.centerLine.lineTo(x, y);
    }

    // Start/finish: a band across the track at s = 0.
    const [lx, ly] = edge(0, 1);
    const [rx, ry] = edge(0, -1);
    const tx = tangents[0];
    const ty = tangents[1];
    const D = 2.6;
    this.startLine.moveTo(lx, ly);
    this.startLine.lineTo(rx, ry);
    this.startLine.lineTo(rx + tx * D, ry + ty * D);
    this.startLine.lineTo(lx + tx * D, ly + ty * D);
    this.startLine.closePath();

    // The chequer, laid out along the band rather than clipped out of an
    // axis-aligned grid. Same picture; two fills a frame instead of clipping
    // and then stamping several hundred squares whether or not the start line
    // is even on screen.
    const across = Math.hypot(rx - lx, ry - ly);
    const ux = (rx - lx) / across;
    const uy = (ry - ly) / across;
    const SQ = 1.3;
    const cols = Math.ceil(across / SQ);
    const rows = Math.ceil(D / SQ);
    for (let c = 0; c < cols; c++) {
      for (let r = 0; r < rows; r++) {
        const path = (c + r) % 2 === 0 ? this.chequerA : this.chequerB;
        const a = Math.min(c * SQ, across);
        const b = Math.min((c + 1) * SQ, across);
        const d0 = Math.min(r * SQ, D);
        const d1 = Math.min((r + 1) * SQ, D);
        path.moveTo(lx + ux * a + tx * d0, ly + uy * a + ty * d0);
        path.lineTo(lx + ux * b + tx * d0, ly + uy * b + ty * d0);
        path.lineTo(lx + ux * b + tx * d1, ly + uy * b + ty * d1);
        path.lineTo(lx + ux * a + tx * d1, ly + uy * a + ty * d1);
        path.closePath();
      }
    }
  }

  /** The road surface tile, as a repeating pattern in world units. */
  private makeAsphalt() {
    const c = document.createElement('canvas');
    c.width = c.height = ASPHALT_TILE;
    const g = c.getContext('2d')!;
    g.putImageData(new ImageData(asphaltPixels(), ASPHALT_TILE, ASPHALT_TILE), 0, 0);
    this.asphalt = this.ctx.createPattern(c, 'repeat');
  }

  // ------------------------------------------------------------- effects --

  /**
   * The mark canvas is Y-down and the world is Y-up, so this mapping has to
   * flip Y to match what `drawMarks` undoes on the way back out -- without it
   * every mark lands as a mirrored ghost of the racing line somewhere else on
   * the map instead of under the car that laid it.
   */
  protected paintSkid(x0: number, y0: number, x1: number, y1: number, alpha: number, joined: boolean) {
    const g = this.marksCtx;
    const sx = (x1 + this.markOx) * this.markScale;
    const sy = MARK_SIZE - (y1 + this.markOy) * this.markScale;
    const ink = `rgba(8,8,12,${alpha})`;
    const w = TIRE_WIDTH * this.markScale;

    if (joined) {
      g.strokeStyle = ink;
      g.lineWidth = w;
      g.lineCap = 'round';
      g.beginPath();
      g.moveTo((x0 + this.markOx) * this.markScale, MARK_SIZE - (y0 + this.markOy) * this.markScale);
      g.lineTo(sx, sy);
      g.stroke();
    } else {
      // First contact of a new slide: nothing to join up to yet.
      g.fillStyle = ink;
      g.beginPath();
      g.arc(sx, sy, w / 2, 0, Math.PI * 2);
      g.fill();
    }
  }

  // ----------------------------------------------------------------- draw --

  draw(cars: DrawCar[], ghost: GhostCar | null, dt: number, localSpeed: number) {
    const ctx = this.ctx;
    const { w, h, dpr } = this.resizeBacking();

    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    // No clear: the backdrop is opaque and covers the canvas.
    this.drawBackdrop(w, h);

    const [shakeX, shakeY] = this.shakeOffset();

    // A full grid is two dozen cars, and the camera holds about a tenth of the
    // circuit -- most of them are somewhere else entirely.
    const reach = this.viewReach(w, h);

    ctx.save();
    ctx.translate(w / 2 + shakeX, h / 2 + shakeY);
    ctx.rotate(this.camRot);
    ctx.scale(this.camZoom, -this.camZoom); // flip Y: physics is Y-up
    ctx.translate(-this.camX, -this.camY);

    this.drawTrack(ctx);
    this.drawMarks(ctx);
    for (const c of cars) if (!c.isLocal && this.inView(c.x, c.y, reach)) this.drawCar(ctx, c);
    if (ghost && this.showGhost) this.drawGhost(ctx, ghost);
    for (const c of cars) if (c.isLocal) this.drawCar(ctx, c);
    this.drawParticles(ctx, dt, reach);

    ctx.restore();

    this.drawNameplates(ctx, cars, w, h, shakeX, shakeY);
    this.drawSpeedLines(ctx, w, h, localSpeed);
    // The vignette is #vignette in the stylesheet, not a fill here. It never
    // changes with anything but the window size, and blending a full-screen
    // radial gradient into the canvas every frame cost more than the entire
    // track did. As a static layer over the canvas the compositor owns it.

    // Marks fade slowly so a long session does not end up solid black.
    const now = performance.now();
    if (now - this.lastMarkFade > 900) {
      this.lastMarkFade = now;
      this.marksCtx.globalCompositeOperation = 'destination-out';
      this.marksCtx.fillStyle = 'rgba(0,0,0,0.035)';
      this.marksCtx.fillRect(0, 0, MARK_SIZE, MARK_SIZE);
      this.marksCtx.globalCompositeOperation = 'source-over';
    }
  }

  private drawBackdrop(w: number, h: number) {
    const ctx = this.ctx;
    const g = ctx.createRadialGradient(w / 2, h * 0.42, 40, w / 2, h * 0.5, Math.max(w, h) * 0.78);
    g.addColorStop(0, '#0b1220');
    g.addColorStop(1, '#04060b');
    ctx.fillStyle = g;
    ctx.fillRect(0, 0, w, h);

    // World-locked grid, so motion reads even off the racing surface. Drawn
    // under the camera transform rather than in screen space: with the camera
    // rotating, an axis-aligned grid slides the wrong way.
    const step = 10;
    if (step * this.camZoom < 6) return;
    const reach = Math.hypot(w, h) / 2 / this.camZoom;
    const x0 = Math.floor((this.camX - reach) / step) * step;
    const y0 = Math.floor((this.camY - reach) / step) * step;
    const x1 = this.camX + reach;
    const y1 = this.camY + reach;
    ctx.save();
    ctx.translate(w / 2, h / 2);
    ctx.rotate(this.camRot);
    ctx.scale(this.camZoom, -this.camZoom);
    ctx.translate(-this.camX, -this.camY);
    ctx.strokeStyle = 'rgba(64,120,180,0.055)';
    ctx.lineWidth = 1 / this.camZoom;
    ctx.beginPath();
    for (let x = x0; x <= x1; x += step) {
      ctx.moveTo(x, y0);
      ctx.lineTo(x, y1);
    }
    for (let y = y0; y <= y1; y += step) {
      ctx.moveTo(x0, y);
      ctx.lineTo(x1, y);
    }
    ctx.stroke();
    ctx.restore();
  }

  private drawTrack(ctx: CanvasRenderingContext2D) {
    ctx.save();

    // Surface. One fill: the tile is opaque and already the right colour.
    ctx.fillStyle = this.asphalt ?? '#14181f';
    ctx.fill(this.surface, 'evenodd');

    // Curbs.
    ctx.fillStyle = '#c8323f';
    ctx.fill(this.curbA, 'evenodd');
    ctx.fillStyle = '#e8eaee';
    ctx.fill(this.curbB, 'evenodd');

    // Center line.
    ctx.setLineDash([2.2, 3.6]);
    ctx.lineWidth = 0.16;
    ctx.strokeStyle = 'rgba(180,210,255,0.16)';
    ctx.stroke(this.centerLine);
    ctx.setLineDash([]);

    // Neon barriers, glowing by stacked stroke rather than by shadow.
    for (const [path, rgb] of [
      [this.edgeL, '56,232,255'],
      [this.edgeR, '255,77,157'],
    ] as const) {
      for (const [width, alpha] of EDGE_GLOW) {
        ctx.lineWidth = width;
        ctx.strokeStyle = `rgba(${rgb},${alpha})`;
        ctx.stroke(path);
      }
    }

    // Start/finish chequer.
    ctx.fillStyle = '#f8fafc';
    ctx.fill(this.chequerA);
    ctx.fillStyle = '#0f172a';
    ctx.fill(this.chequerB);

    ctx.restore();
  }

  private drawMarks(ctx: CanvasRenderingContext2D) {
    ctx.save();
    ctx.globalAlpha = 0.85;
    const s = 1 / this.markScale;
    ctx.translate(-this.markOx, -this.markOy);
    ctx.scale(s, s);
    // The marks canvas is Y-down; the world transform is Y-up, so flip back.
    ctx.translate(0, MARK_SIZE);
    ctx.scale(1, -1);
    ctx.drawImage(this.marks, 0, 0);
    ctx.restore();
  }

  private drawCar(ctx: CanvasRenderingContext2D, c: DrawCar) {
    const col = `#${c.color.toString(16).padStart(6, '0')}`;
    ctx.save();
    ctx.translate(c.x, c.y);
    ctx.rotate(c.heading);

    // Contact shadow.
    ctx.save();
    ctx.translate(-0.25, -0.3);
    ctx.fillStyle = 'rgba(0,0,0,0.45)';
    roundRect(ctx, -2.15, -1.05, 4.3, 2.1, 0.55);
    ctx.fill();
    ctx.restore();

    // Wheels.
    ctx.fillStyle = '#0b0d12';
    for (const [wx, wy, turn] of [
      [1.28, 0.92, 1],
      [1.28, -0.92, 1],
      [-1.32, 0.95, 0],
      [-1.32, -0.95, 0],
    ] as const) {
      ctx.save();
      ctx.translate(wx, wy);
      if (turn) ctx.rotate(c.steer);
      roundRect(ctx, -0.42, -0.2, 0.84, 0.4, 0.14);
      ctx.fill();
      ctx.restore();
    }

    // Body. The gradient is in the car's own frame, which is the same frame
    // for every car, so one per colour is enough for the whole grid.
    let g = this.bodyGrad.get(c.color);
    if (!g) {
      g = ctx.createLinearGradient(0, -1, 0, 1);
      g.addColorStop(0, shade(col, -0.35));
      g.addColorStop(0.45, col);
      g.addColorStop(1, shade(col, -0.55));
      this.bodyGrad.set(c.color, g);
    }
    ctx.fillStyle = g;
    ctx.shadowColor = col;
    ctx.shadowBlur = c.isLocal ? 22 : 12;
    carBody(ctx);
    ctx.fill();
    ctx.shadowBlur = 0;

    // Cockpit and accents.
    ctx.fillStyle = 'rgba(10,16,26,0.85)';
    roundRect(ctx, -0.55, -0.62, 1.15, 1.24, 0.28);
    ctx.fill();
    ctx.fillStyle = 'rgba(255,255,255,0.16)';
    ctx.fillRect(-1.9, -0.12, 3.4, 0.24);
    // Rear wing.
    ctx.fillStyle = shade(col, -0.6);
    roundRect(ctx, -2.15, -0.95, 0.34, 1.9, 0.1);
    ctx.fill();

    // Lights.
    if (c.speed > 0.5 || c.throttle > 0) {
      ctx.fillStyle = 'rgba(255,244,214,0.95)';
      ctx.fillRect(1.92, -0.78, 0.2, 0.4);
      ctx.fillRect(1.92, 0.38, 0.2, 0.4);
      if (!this.beamGrad) {
        this.beamGrad = ctx.createLinearGradient(2.1, 0, 12, 0);
        this.beamGrad.addColorStop(0, 'rgba(255,240,200,0.13)');
        this.beamGrad.addColorStop(1, 'rgba(255,240,200,0)');
      }
      ctx.fillStyle = this.beamGrad;
      ctx.beginPath();
      ctx.moveTo(2.0, -0.8);
      ctx.lineTo(13, -3.6);
      ctx.lineTo(13, 3.6);
      ctx.lineTo(2.0, 0.8);
      ctx.closePath();
      ctx.fill();
    }
    ctx.fillStyle = c.braking ? '#ff2d55' : 'rgba(150,30,50,0.75)';
    if (c.braking) {
      ctx.shadowColor = '#ff2d55';
      ctx.shadowBlur = 14;
    }
    ctx.fillRect(-2.12, -0.8, 0.18, 0.5);
    ctx.fillRect(-2.12, 0.3, 0.18, 0.5);
    ctx.shadowBlur = 0;

    if (c.isLocal) {
      ctx.strokeStyle = 'rgba(255,255,255,0.55)';
      ctx.lineWidth = 0.07;
      carBody(ctx);
      ctx.stroke();
    }
    ctx.restore();
  }

  /** The authority's pose, drawn as a wireframe next to the prediction. */
  private drawGhost(ctx: CanvasRenderingContext2D, g: GhostCar) {
    ctx.save();
    ctx.translate(g.x, g.y);
    ctx.rotate(g.heading);
    ctx.strokeStyle = 'rgba(120,255,214,0.75)';
    ctx.lineWidth = 0.09;
    ctx.setLineDash([0.45, 0.32]);
    carBody(ctx);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = 'rgba(120,255,214,0.9)';
    ctx.beginPath();
    ctx.arc(0, 0, 0.16, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  private drawParticles(ctx: CanvasRenderingContext2D, dt: number, reach: number) {
    ctx.save();
    this.stepParticles(dt, (p, t) => {
      if (!this.inView(p.x, p.y, reach)) return;
      if (p.kind === 0) {
        const r = p.size * (2.2 - t * 1.2);
        ctx.globalAlpha = t * t * 0.16;
        ctx.fillStyle = '#c9d4e4';
        ctx.beginPath();
        ctx.arc(p.x, p.y, r, 0, Math.PI * 2);
        ctx.fill();
      } else {
        ctx.globalAlpha = t;
        ctx.fillStyle = `hsl(${p.hue} 100% ${55 + t * 30}%)`;
        ctx.beginPath();
        ctx.arc(p.x, p.y, p.size * t, 0, Math.PI * 2);
        ctx.fill();
      }
    });
    ctx.globalAlpha = 1;
    ctx.restore();
  }

  private drawNameplates(
    ctx: CanvasRenderingContext2D,
    cars: DrawCar[],
    w: number,
    h: number,
    sx: number,
    sy: number,
  ) {
    ctx.save();
    ctx.font = '600 11px ui-monospace, "SF Mono", Menlo, monospace';
    ctx.textAlign = 'center';
    for (const c of cars) {
      if (c.isLocal) continue;
      // Same transform the world layer uses -- translate, flip Y, then rotate
      // -- collapsed into one step, so a plate stays pinned to its car once
      // the camera is turning.
      const dx = c.x - this.camX;
      const dy = c.y - this.camY;
      const cos = Math.cos(this.camRot);
      const sin = Math.sin(this.camRot);
      const px = (dx * cos + dy * sin) * this.camZoom + w / 2 + sx;
      const py = (dx * sin - dy * cos) * this.camZoom + h / 2 + sy;
      if (px < -80 || px > w + 80 || py < -60 || py > h + 60) continue;
      const label = `${c.name}${c.isBot ? '' : ' ●'}`;
      const ty = py - this.camZoom * 2.6;
      ctx.fillStyle = 'rgba(4,7,12,0.6)';
      const tw = ctx.measureText(label).width;
      ctx.fillRect(px - tw / 2 - 5, ty - 11, tw + 10, 15);
      ctx.fillStyle = `#${c.color.toString(16).padStart(6, '0')}`;
      ctx.fillText(label, px, ty);
    }
    ctx.restore();
  }

  private drawSpeedLines(ctx: CanvasRenderingContext2D, w: number, h: number, speed: number) {
    const t = Math.max(0, (speed - 34) / 34);
    if (t <= 0.01) return;
    ctx.save();
    ctx.globalAlpha = Math.min(0.3, t * 0.3);
    ctx.strokeStyle = '#dbeafe';
    ctx.lineWidth = 1.4;
    const cx = w / 2;
    const cy = h / 2;
    const r = Math.max(w, h) * 0.42;
    ctx.beginPath();
    for (let i = 0; i < 26; i++) {
      const a = (i / 26) * Math.PI * 2 + performance.now() * 0.0004;
      const len = 30 + Math.random() * 70 * t;
      ctx.moveTo(cx + Math.cos(a) * r, cy + Math.sin(a) * r);
      ctx.lineTo(cx + Math.cos(a) * (r + len), cy + Math.sin(a) * (r + len));
    }
    ctx.stroke();
    ctx.restore();
  }
}

// ------------------------------------------------------------------ utils --

function carBody(ctx: CanvasRenderingContext2D) {
  ctx.beginPath();
  for (let i = 0; i < CAR_OUTLINE.length; i += 2) {
    const x = CAR_OUTLINE[i];
    const y = CAR_OUTLINE[i + 1];
    i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
  }
  ctx.closePath();
}

function roundRect(ctx: CanvasRenderingContext2D, x: number, y: number, w: number, h: number, r: number) {
  ctx.beginPath();
  ctx.moveTo(x + r, y);
  ctx.arcTo(x + w, y, x + w, y + h, r);
  ctx.arcTo(x + w, y + h, x, y + h, r);
  ctx.arcTo(x, y + h, x, y, r);
  ctx.arcTo(x, y, x + w, y, r);
  ctx.closePath();
}

export function edgeInset(
  pt: readonly [number, number],
  points: Float32Array,
  i: number,
  amount: number,
): [number, number] {
  const cx = points[i * 2];
  const cy = points[i * 2 + 1];
  const dx = cx - pt[0];
  const dy = cy - pt[1];
  const len = Math.hypot(dx, dy) || 1;
  return [pt[0] + (dx / len) * amount, pt[1] + (dy / len) * amount];
}
