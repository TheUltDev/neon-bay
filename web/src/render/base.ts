// Everything a renderer needs that has nothing to do with how it draws.
//
// The camera, the particle pool, the skid-mark bookkeeping and the minimap are
// identical whether the frame ends up in a 2D context, a GL command stream or a
// WebGPU pass -- so they live here once, and each backend inherits them and
// implements only `draw` and `paintSkid`.

import type { Sim, TrackData } from '../sim';
import type { DrawCar, GhostCar, Renderer, RendererInfo, RenderSize } from './types';

export const MAX_PARTICLES = 700;
/** Tire contact patch width, in metres. Matches the wheels drawn on the car. */
export const TIRE_WIDTH = 0.4;
/** A streak breaks if its wheel stopped marking for longer than this, in ms. */
const CONTACT_GAP = 120;
/** Side of the world-space layer skid marks accumulate into, in texels. */
export const MARK_SIZE = 2048;

export interface Particle {
  x: number;
  y: number;
  vx: number;
  vy: number;
  life: number;
  maxLife: number;
  size: number;
  kind: 0 | 1; // 0 smoke, 1 spark
  hue: number;
}

export abstract class BaseRenderer implements Renderer {
  camX = 0;
  camY = 0;
  camZoom = 15;
  camRot = 0;
  rotateCamera = true;
  showGhost = true;

  abstract readonly info: RendererInfo;
  readonly size: RenderSize = { w: 0, h: 0, dpr: 1 };

  protected track: TrackData;
  protected shake = 0;
  protected particles: Particle[] = [];
  private partIdx = 0;

  /** Last contact point of each tire, in world space, keyed by wheel id. */
  private contacts = new Map<number, { x: number; y: number; t: number }>();

  /** World to mark-layer scale and origin, shared by every backend so a skid
   *  lands in the same texel whichever one is drawing it. */
  protected markScale: number;
  protected markOx: number;
  protected markOy: number;

  constructor(
    readonly canvas: HTMLCanvasElement,
    sim: Sim,
  ) {
    this.track = sim.track;
    const b = this.track.bounds;
    this.markScale = MARK_SIZE / Math.max(b.maxX - b.minX, b.maxY - b.minY);
    this.markOx = -b.minX;
    this.markOy = -b.minY;
  }

  // -------------------------------------------------------------- camera --

  updateCamera(
    target: { x: number; y: number; heading: number } | null,
    vx: number,
    vy: number,
    dt: number,
    snap = false,
  ) {
    if (!target) return;
    const speed = Math.hypot(vx, vy);

    // Rotation first: the chase offset below rides on the angle we settle on.
    if (this.rotateCamera) {
      // Screen up is the car's nose, so the view is parked behind it looking
      // forward. Track the heading rather than the velocity -- a spin should
      // swing the camera round with the car, not chase where it is sliding.
      let d = target.heading - Math.PI / 2 - this.camRot;
      while (d > Math.PI) d -= Math.PI * 2;
      while (d < -Math.PI) d += Math.PI * 2;
      this.camRot += d * (1 - Math.pow(0.02, dt));
    } else {
      this.camRot *= Math.pow(0.02, dt);
    }

    // Look where you are going, and pull back as the speed rises.
    const lead = Math.min(1, speed / 45);
    let tx: number;
    let ty: number;
    if (this.rotateCamera) {
      // Push the focus straight up the screen (the camera's forward axis), so
      // the car sits low in frame with the road ahead of it. Keeping the
      // offset on that axis and not on the heading holds the car on the
      // centreline while the rotation is still catching up.
      const ahead = 4 + lead * 10;
      tx = target.x - Math.sin(this.camRot) * ahead;
      ty = target.y + Math.cos(this.camRot) * ahead;
    } else {
      tx = target.x + vx * 0.42 * lead;
      ty = target.y + vy * 0.42 * lead;
    }
    if (snap) {
      // Spectating: we have no velocity for the car, so smoothing would only
      // trail behind it. Sit on the mark instead.
      this.camX = tx;
      this.camY = ty;
    } else {
      const k = 1 - Math.pow(0.0025, dt);
      this.camX += (tx - this.camX) * k;
      this.camY += (ty - this.camY) * k;
    }

    const zoomTarget = 17.5 - Math.min(7.5, speed * 0.135);
    this.camZoom += (zoomTarget - this.camZoom) * (1 - Math.pow(0.05, dt));

    this.shake *= Math.pow(0.02, dt);
  }

  impulse(strength: number) {
    this.shake = Math.min(26, this.shake + strength);
  }

  worldFromScreen() {
    return { x: this.camX, y: this.camY, zoom: this.camZoom };
  }

  /** World-space radius that certainly covers the viewport, whatever the
   *  camera rotation, plus room for a car straddling the edge. */
  protected viewReach(w: number, h: number): number {
    return Math.hypot(w, h) / 2 / this.camZoom + 6;
  }

  protected inView(x: number, y: number, reach: number): boolean {
    const dx = x - this.camX;
    const dy = y - this.camY;
    return dx * dx + dy * dy < reach * reach;
  }

  /** Screen shake for this frame, in CSS pixels. */
  protected shakeOffset(): [number, number] {
    return this.shake > 0.2 ? [rnd(this.shake * 0.5), rnd(this.shake * 0.5)] : [0, 0];
  }

  /**
   * Match the drawing buffer to the element, and record what that came to.
   * Returns the CSS-pixel size everything is laid out in.
   */
  protected resizeBacking(): { w: number; h: number; dpr: number } {
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = this.canvas.clientWidth || 1;
    const h = this.canvas.clientHeight || 1;
    const pw = Math.max(1, Math.round(w * dpr));
    const ph = Math.max(1, Math.round(h * dpr));
    if (this.canvas.width !== pw || this.canvas.height !== ph) {
      this.canvas.width = pw;
      this.canvas.height = ph;
    }
    this.size.w = pw;
    this.size.h = ph;
    this.size.dpr = dpr;
    return { w, h, dpr };
  }

  // ------------------------------------------------------------- effects --

  addSmoke(x: number, y: number, vx: number, vy: number, strength: number) {
    this.spawn(x, y, vx * 0.12 + rnd(0.9), vy * 0.12 + rnd(0.9), 0.32 + strength * 0.5, 0.4 + strength * 0.75, 0, 0);
  }

  addSpark(x: number, y: number, vx: number, vy: number) {
    const a = Math.random() * Math.PI * 2;
    const s = 6 + Math.random() * 14;
    this.spawn(x, y, vx * 0.2 + Math.cos(a) * s, vy * 0.2 + Math.sin(a) * s, 0.18 + Math.random() * 0.2, 0.22, 1, 30 + Math.random() * 25);
  }

  private spawn(x: number, y: number, vx: number, vy: number, life: number, size: number, kind: 0 | 1, hue: number) {
    const p = this.particles[this.partIdx] ?? ({} as Particle);
    p.x = x;
    p.y = y;
    p.vx = vx;
    p.vy = vy;
    p.life = life;
    p.maxLife = life;
    p.size = size;
    p.kind = kind;
    p.hue = hue;
    this.particles[this.partIdx] = p;
    this.partIdx = (this.partIdx + 1) % MAX_PARTICLES;
  }

  /**
   * Age the pool by `dt` and hand every particle still alive to `visit`.
   *
   * Smoke from a bot on the far side of the circuit still has to age, so the
   * culling is the visitor's business and not this loop's.
   */
  protected stepParticles(dt: number, visit: (p: Particle, fade: number) => void) {
    for (const p of this.particles) {
      if (!p || p.life <= 0) continue;
      p.life -= dt;
      p.x += p.vx * dt;
      p.y += p.vy * dt;
      p.vx *= 0.965;
      p.vy *= 0.965;
      if (p.life <= 0) continue;
      visit(p, p.life / p.maxLife);
    }
  }

  /**
   * Dark streak under a sliding tire.
   *
   * `wheel` identifies one tire across frames: the streak is drawn from that
   * wheel's previous contact point, so a car at speed leaves one unbroken line
   * rather than the dotted trail a per-frame stamp would give.
   */
  addSkid(wheel: number, x: number, y: number, alpha: number) {
    const now = performance.now();
    const prev = this.contacts.get(wheel);
    const joined = !!prev && now - prev.t < CONTACT_GAP;
    this.paintSkid(prev ? prev.x : x, prev ? prev.y : y, x, y, Math.min(0.5, alpha), joined);
    if (prev) {
      prev.x = x;
      prev.y = y;
      prev.t = now;
    } else {
      this.contacts.set(wheel, { x, y, t: now });
    }
  }

  /**
   * Lay one segment of tire ink into the world-space mark layer, in world
   * coordinates. With `joined` false there is no slide to continue and only
   * the second point gets a stamp.
   */
  protected abstract paintSkid(
    x0: number,
    y0: number,
    x1: number,
    y1: number,
    alpha: number,
    joined: boolean,
  ): void;

  abstract draw(cars: DrawCar[], ghost: GhostCar | null, dt: number, localSpeed: number): void;

  dispose() {}

  // ------------------------------------------------------------- minimap --

  /**
   * The whole circuit in a corner panel, on its own little 2D context.
   *
   * Deliberately not on the GPU backends' pipeline: it is a couple of hundred
   * pixels of polyline on a separate element, and standing a second device up
   * to feed it would cost more than it draws.
   */
  drawMinimap(canvas: HTMLCanvasElement, sim: Sim, cars: DrawCar[]) {
    const ctx = canvas.getContext('2d');
    if (!ctx) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (canvas.width !== w * dpr) {
      canvas.width = w * dpr;
      canvas.height = h * dpr;
    }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);

    const b = sim.track.bounds;
    const pad = 6;
    const scale = Math.min((w - pad * 2) / (b.maxX - b.minX), (h - pad * 2) / (b.maxY - b.minY));
    const ox = w / 2 - ((b.minX + b.maxX) / 2) * scale;
    const oy = h / 2 + ((b.minY + b.maxY) / 2) * scale;
    const px = (x: number) => ox + x * scale;
    const py = (y: number) => oy - y * scale;

    ctx.beginPath();
    const { points, samples } = sim.track;
    for (let i = 0; i <= samples; i++) {
      const k = i % samples;
      const x = px(points[k * 2]);
      const y = py(points[k * 2 + 1]);
      i === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    }
    ctx.closePath();
    ctx.strokeStyle = 'rgba(120,190,255,0.32)';
    ctx.lineWidth = Math.max(2.5, sim.track.halfWidth[0] * scale * 1.6);
    ctx.stroke();
    ctx.strokeStyle = 'rgba(56,232,255,0.5)';
    ctx.lineWidth = 1;
    ctx.stroke();

    for (const c of cars) {
      ctx.beginPath();
      ctx.arc(px(c.x), py(c.y), c.isLocal ? 3.4 : 2.4, 0, Math.PI * 2);
      ctx.fillStyle = c.isLocal ? '#ffffff' : `#${c.color.toString(16).padStart(6, '0')}`;
      ctx.fill();
      if (c.isLocal) {
        ctx.strokeStyle = '#22d3ee';
        ctx.lineWidth = 1.5;
        ctx.stroke();
      }
    }
  }
}

export function rnd(mag: number): number {
  return (Math.random() - 0.5) * 2 * mag;
}
