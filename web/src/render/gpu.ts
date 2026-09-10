// The GPU renderer, written once and run on either API.
//
// Same picture as the 2D renderer, assembled the way a GPU wants it: the track
// is triangulated at load into two buffers that never change, and each frame
// appends the things that move into three more. That comes to eight draw calls
// for a full grid of cars -- the 2D version issues a call per car per detail.
//
// Everything is premultiplied and blended with `ONE, ONE_MINUS_SRC_ALPHA`,
// there is no depth buffer, and nothing is sorted: triangles land in the order
// they were written, which is the order the 2D renderer paints in. That is the
// whole reason the two agree.
//
// Two things are deliberately not on the GPU. Nameplates go on a 2D overlay
// canvas over the top, because a glyph atlas to draw a dozen short strings
// would be more machinery than the strings are worth; and the minimap keeps its
// own little 2D context, for the same reason.

import type { Sim } from '../sim';
import { BaseRenderer, MARK_SIZE, TIRE_WIDTH } from './base';
import { Mode, Tex, type GpuDevice, type GpuMesh } from './device';
import { MeshBuilder } from './mesh';
import {
  ASPHALT_TILE, CAR_HULL, CAR_HULL_N, crushOutline,
  EDGE_GLOW, EDGE_RGB, hslRgb, lampOut, shadeRgb, WHEELS, WING_LOST,
} from './palette';
import type { DrawCar, GhostCar, RendererInfo } from './types';

/** Where the two brake lights sit along the tail. */
const BRAKE_LIGHTS = [-0.8, 0.3];

/** Grid spacing on the ground plane, in metres. */
const GRID_STEP = 10;

export class GpuRenderer extends BaseRenderer {
  readonly info: RendererInfo;

  // Static: uploaded once at load and drawn whole.
  private surfaceMesh: GpuMesh;
  private trackMesh: GpuMesh;

  // Per frame: built on the CPU, uploaded, then drawn in ranges.
  private world = new MeshBuilder(4096);
  private discs = new MeshBuilder(1024);
  private screen = new MeshBuilder(256);
  private ink = new MeshBuilder(256);
  private worldMesh: GpuMesh;
  private discMesh: GpuMesh;
  private screenMesh: GpuMesh;
  private inkMesh: GpuMesh;

  private proj = new Float32Array(6);
  private markProj = new Float32Array(6);
  private lastMarkFade = 0;

  // Nameplates, over the top.
  private overlay: HTMLCanvasElement;
  private plates: CanvasRenderingContext2D | null;
  private platesDirty = false;

  private gcol = new Float32Array(3);
  private quadPts = new Float32Array(8);

  constructor(
    canvas: HTMLCanvasElement,
    sim: Sim,
    private dev: GpuDevice,
    fallback: string | null,
  ) {
    super(canvas, sim);
    this.info = {
      backend: dev.backend,
      api: dev.api,
      device: dev.device,
      detail: dev.detail,
      fallback,
    };

    const surface = new MeshBuilder(5120);
    const solid = new MeshBuilder(32768);
    this.bakeTrack(surface, solid);
    this.surfaceMesh = dev.createMesh('track-surface');
    this.trackMesh = dev.createMesh('track-detail');
    dev.upload(this.surfaceMesh, surface.data, surface.floats);
    dev.upload(this.trackMesh, solid.data, solid.floats);

    this.worldMesh = dev.createMesh('world');
    this.discMesh = dev.createMesh('particles');
    this.screenMesh = dev.createMesh('screen');
    this.inkMesh = dev.createMesh('ink');

    // World to mark layer. The layer is square and covers the track's bounding
    // box; `targetFlipY` is the one place the two APIs disagree about which way
    // up a render target is.
    const span = MARK_SIZE / this.markScale;
    const f = dev.targetFlipY;
    this.markProj.set([2 / span, 0, 0, (f * 2) / span, (2 * this.markOx) / span - 1, f * ((2 * this.markOy) / span - 1)]);

    this.overlay = document.createElement('canvas');
    this.overlay.className = 'gfx-overlay';
    canvas.parentElement?.insertBefore(this.overlay, canvas.nextSibling);
    this.plates = this.overlay.getContext('2d');
  }

  dispose() {
    this.overlay.remove();
    this.dev.dispose();
  }

  // ------------------------------------------------------------ geometry --

  /**
   * The circuit, triangulated once.
   *
   * `surface` is the road itself, textured with the asphalt tile; `solid` is
   * everything drawn over it, in the order the 2D renderer paints them --
   * curbs, centre line, barriers, chequer -- because painter's order is the
   * only sorting either backend does.
   */
  private bakeTrack(surface: MeshBuilder, solid: MeshBuilder) {
    const { points, tangents, halfWidth, curvature, samples } = this.track;
    const edge = (i: number, side: number) => {
      const nx = -tangents[i * 2 + 1];
      const ny = tangents[i * 2];
      const w = halfWidth[i] * side;
      return [points[i * 2] + nx * w, points[i * 2 + 1] + ny * w] as const;
    };

    // Road surface: one quad per sample between the two edges. The texture
    // coordinate is the world position over the tile size, so the grain is
    // locked to the ground exactly as the 2D pattern is.
    const T = ASPHALT_TILE;
    surface.color(1, 1, 1, 1);
    for (let i = 0; i < samples; i++) {
      const j = (i + 1) % samples;
      const [lx, ly] = edge(i, 1);
      const [rx, ry] = edge(i, -1);
      const [lx2, ly2] = edge(j, 1);
      const [rx2, ry2] = edge(j, -1);
      surface.texQuad(
        lx, ly, lx / T, ly / T,
        rx, ry, rx / T, ry / T,
        rx2, ry2, rx2 / T, ry2 / T,
        lx2, ly2, lx2 / T, ly2 / T,
      );
    }

    // Curbs: alternating blocks just inside each edge, only where it bends.
    const KERB = 1.25;
    for (let i = 0; i < samples; i++) {
      if (Math.abs(curvature[i]) < 0.0055) continue;
      (i >> 2) % 2 === 0 ? solid.colorHex(0xc8323f) : solid.colorHex(0xe8eaee);
      for (const side of [1, -1]) {
        const j = (i + 1) % samples;
        const a = edge(i, side);
        const b = edge(j, side);
        const c = inset(b, points, j, KERB);
        const d = inset(a, points, i, KERB);
        solid.quad(a[0], a[1], b[0], b[1], c[0], c[1], d[0], d[1]);
      }
    }

    // Centre line: every second sample, dashed in world units.
    const mid = new Float32Array(Math.ceil(samples / 2) * 2);
    let m = 0;
    for (let i = 0; i < samples; i += 2) {
      mid[m++] = points[i * 2];
      mid[m++] = points[i * 2 + 1];
    }
    solid.color255(180, 210, 255, 0.16);
    solid.dashed(mid, m / 2, true, 2.2, 3.6, 0.16);

    // Neon barriers: the same three stacked strokes the 2D renderer uses,
    // extruded into ribbons instead of stroked.
    for (let s = 0; s < 2; s++) {
      const side = s === 0 ? 1 : -1;
      const line = new Float32Array(samples * 2);
      for (let i = 0; i < samples; i++) {
        const [x, y] = edge(i, side);
        line[i * 2] = x;
        line[i * 2 + 1] = y;
      }
      const [r, g, b] = EDGE_RGB[s];
      for (const [width, alpha] of EDGE_GLOW) {
        solid.color255(r, g, b, alpha);
        solid.ribbon(line, samples, width, true);
      }
    }

    // Start/finish chequer, laid out along the band rather than clipped out of
    // an axis-aligned grid.
    const [lx, ly] = edge(0, 1);
    const [rx, ry] = edge(0, -1);
    const tx = tangents[0];
    const ty = tangents[1];
    const D = 2.6;
    const across = Math.hypot(rx - lx, ry - ly);
    const ux = (rx - lx) / across;
    const uy = (ry - ly) / across;
    const SQ = 1.3;
    for (let c = 0; c < Math.ceil(across / SQ); c++) {
      for (let r = 0; r < Math.ceil(D / SQ); r++) {
        (c + r) % 2 === 0 ? solid.colorHex(0xf8fafc) : solid.colorHex(0x0f172a);
        const a0 = Math.min(c * SQ, across);
        const a1 = Math.min((c + 1) * SQ, across);
        const d0 = Math.min(r * SQ, D);
        const d1 = Math.min((r + 1) * SQ, D);
        solid.quad(
          lx + ux * a0 + tx * d0, ly + uy * a0 + ty * d0,
          lx + ux * a1 + tx * d0, ly + uy * a1 + ty * d0,
          lx + ux * a1 + tx * d1, ly + uy * a1 + ty * d1,
          lx + ux * a0 + tx * d1, ly + uy * a0 + ty * d1,
        );
      }
    }
  }

  // ------------------------------------------------------------- effects --

  protected paintSkid(x0: number, y0: number, x1: number, y1: number, alpha: number, joined: boolean) {
    this.ink.color255(8, 8, 12, alpha);
    if (joined) {
      this.ink.segment(x0, y0, x1, y1, TIRE_WIDTH);
    } else {
      // First contact of a new slide: nothing to join up to yet. At six texels
      // to the metre a square and a round cap are the same three pixels.
      this.ink.rect(x1 - TIRE_WIDTH / 2, y1 - TIRE_WIDTH / 2, TIRE_WIDTH, TIRE_WIDTH);
    }
  }

  // ---------------------------------------------------------------- draw --

  draw(cars: DrawCar[], ghost: GhostCar | null, dt: number, localSpeed: number) {
    const { w, h } = this.resizeBacking();
    const dev = this.dev;
    const [shakeX, shakeY] = this.shakeOffset();
    const reach = this.viewReach(w, h);

    this.world.reset();
    this.discs.reset();
    this.screen.reset();

    // --- screen space: the backdrop, and later the speed lines -------------
    // Two quads: the outer colour flat across the frame, then the inner colour
    // over it fading out radially. Together that is the radial gradient the 2D
    // renderer fills, without a per-frame gradient object.
    const R = Math.max(w, h) * 0.78;
    const bx = w / 2;
    const by = h * 0.42;
    const flatAt = this.screen.n;
    this.screen.color255(4, 6, 11, 1).rect(0, 0, w, h);
    const rampAt = this.screen.n;
    this.screen.color255(11, 18, 32, 1);
    this.screen.texQuad(
      0, 0, -bx / R, -by / R,
      w, 0, (w - bx) / R, -by / R,
      w, h, (w - bx) / R, (h - by) / R,
      0, h, -bx / R, (h - by) / R,
    );

    // --- world space -------------------------------------------------------
    // Grid first, under the track: world-locked, so motion reads even off the
    // racing surface.
    const gridAt = this.world.n;
    if (GRID_STEP * this.camZoom >= 6) {
      const span = Math.hypot(w, h) / 2 / this.camZoom;
      const x0 = Math.floor((this.camX - span) / GRID_STEP) * GRID_STEP;
      const y0 = Math.floor((this.camY - span) / GRID_STEP) * GRID_STEP;
      const x1 = this.camX + span;
      const y1 = this.camY + span;
      const lw = 1 / this.camZoom;
      this.world.color255(64, 120, 180, 0.055);
      for (let x = x0; x <= x1; x += GRID_STEP) this.world.segment(x, y0, x, y1, lw);
      for (let y = y0; y <= y1; y += GRID_STEP) this.world.segment(x0, y, x1, y, lw);
    }
    const gridVerts = this.world.n - gridAt;

    // The mark layer, as one quad over the track's bounding box.
    const marksAt = this.world.n;
    const span = MARK_SIZE / this.markScale;
    const mx = -this.markOx;
    const my = -this.markOy;
    this.world.color(1, 1, 1, 0.85);
    this.world.texQuad(
      mx, my, 0, 0,
      mx + span, my, 1, 0,
      mx + span, my + span, 1, 1,
      mx, my + span, 0, 1,
    );

    // Cars, in the 2D renderer's order: everyone else, the authority's ghost,
    // then the local car on top.
    const carsAt = this.world.n;
    for (const c of cars) if (!c.isLocal && this.inView(c.x, c.y, reach)) this.emitCar(c);
    if (ghost && this.showGhost) this.emitGhost(ghost);
    for (const c of cars) if (c.isLocal) this.emitCar(c);
    const carVerts = this.world.n - carsAt;

    // Particles age whether or not they are on screen; only the visible ones
    // are worth a quad.
    this.stepParticles(dt, (p, t) => {
      if (!this.inView(p.x, p.y, reach)) return;
      if (p.kind === 0) {
        this.discs.color255(201, 212, 228, t * t * 0.16);
        this.discs.disc(p.x, p.y, p.size * (2.2 - t * 1.2));
      } else {
        const [r, g, b] = hslRgb(p.hue, 1, 0.55 + t * 0.3);
        this.discs.color(r, g, b, t);
        this.discs.disc(p.x, p.y, p.size * t);
      }
    });

    // Speed lines, back in screen space, at the end of the buffer.
    const speedAt = this.screen.n;
    this.emitSpeedLines(w, h, localSpeed);
    const speedVerts = this.screen.n - speedAt;

    // --- submit ------------------------------------------------------------
    dev.beginFrame(this.size.w, this.size.h);

    // Marks accumulate into their own layer, so they have to be laid down
    // before the pass that samples it. They also fade, slowly, or a long
    // session ends up driving on solid black.
    const now = performance.now();
    const fade = now - this.lastMarkFade > 900 ? 0.035 : 0;
    if (fade > 0) this.lastMarkFade = now;
    if (this.ink.n > 0 || fade > 0) {
      dev.upload(this.inkMesh, this.ink.data, this.ink.floats);
      dev.paintMarks(this.inkMesh, this.ink.n, this.markProj, fade);
      this.ink.reset();
    }

    dev.upload(this.worldMesh, this.world.data, this.world.floats);
    dev.upload(this.discMesh, this.discs.data, this.discs.floats);
    dev.upload(this.screenMesh, this.screen.data, this.screen.floats);

    this.screenProjection(w, h);
    dev.setProjection(this.proj);
    dev.draw(this.screenMesh, flatAt, 6, Mode.Solid, Tex.None);
    dev.draw(this.screenMesh, rampAt, 6, Mode.Ramp, Tex.None, 40 / R, 1);

    this.worldProjection(w, h, shakeX, shakeY);
    dev.setProjection(this.proj);
    if (gridVerts) dev.draw(this.worldMesh, gridAt, gridVerts, Mode.Solid, Tex.None);
    dev.draw(this.surfaceMesh, 0, this.surfaceMesh.verts, Mode.Tex, Tex.Asphalt);
    dev.draw(this.trackMesh, 0, this.trackMesh.verts, Mode.Solid, Tex.None);
    dev.draw(this.worldMesh, marksAt, 6, Mode.Tex, Tex.Marks);
    if (carVerts) dev.draw(this.worldMesh, carsAt, carVerts, Mode.Solid, Tex.None);
    if (this.discs.n) dev.draw(this.discMesh, 0, this.discs.n, Mode.Disc, Tex.None);

    if (speedVerts) {
      this.screenProjection(w, h);
      dev.setProjection(this.proj);
      dev.draw(this.screenMesh, speedAt, speedVerts, Mode.Solid, Tex.None);
    }
    dev.endFrame();

    this.drawNameplates(cars, w, h, shakeX, shakeY);

    // A lost device stops producing frames rather than throwing; say so in the
    // panel instead of leaving a still picture and no explanation.
    if (dev.lost) this.info.fallback = dev.lost;
  }

  /** World metres straight to clip space, camera rotation and shake included. */
  private worldProjection(w: number, h: number, shakeX: number, shakeY: number) {
    const z = this.camZoom;
    const cos = Math.cos(this.camRot);
    const sin = Math.sin(this.camRot);
    const a = (2 * z * cos) / w;
    const b = (2 * z * sin) / w;
    const c = (-2 * z * sin) / h;
    const d = (2 * z * cos) / h;
    const ox = (2 * (w / 2 + shakeX)) / w - 1;
    const oy = 1 - (2 * (h / 2 + shakeY)) / h;
    this.proj.set([a, b, c, d, ox - (a * this.camX + b * this.camY), oy - (c * this.camX + d * this.camY)]);
  }

  /** CSS pixels, y down, the way the 2D renderer lays out its overlays. */
  private screenProjection(w: number, h: number) {
    this.proj.set([2 / w, 0, 0, -2 / h, -1, 1]);
  }

  // ------------------------------------------------------------ the cars --

  private emitCar(c: DrawCar) {
    const mb = this.world;
    // The shape this car is now, in its own frame. See `crushOutline`.
    const hull = crushOutline(c.dmgFront, c.dmgRear, c.dmgLeft, c.dmgRight);
    const grade = Math.round(c.damage * 4);
    const r = ((c.color >> 16) & 255) / 255;
    const g = ((c.color >> 8) & 255) / 255;
    const b = (c.color & 255) / 255;
    // `shadowBlur` is a screen-space radius, so the world-space glow has to
    // shrink as the camera pulls back or a car at speed grows a halo. Only the
    // reach changes with it: a wider blur spreads the same light further, it
    // does not put more of it against the car, which is why both tiers of car
    // glow start at the same value below.
    const spread = (c.isLocal ? 22 : 12) / this.camZoom;

    mb.push(c.x, c.y, c.heading);

    // Contact shadow.
    mb.color(0, 0, 0, 0.45).push(-0.25, -0.3);
    mb.roundRect(-2.15, -1.05, 4.3, 2.1, 0.55);
    mb.pop();

    // Wheels.
    mb.colorHex(0x0b0d12);
    for (const [wx, wy, turn] of WHEELS) {
      mb.push(wx, wy, turn ? c.steer : 0);
      mb.roundRect(-0.42, -0.2, 0.84, 0.4, 0.14);
      mb.pop();
    }

    // The glow the 2D renderer gets from `shadowBlur`, then the body over it.
    // 0.45 is what a canvas shadow actually leaves against the body: measured
    // off the 2D renderer, not guessed. A blurred edge would sit at half the
    // shadow's own alpha; a car is narrow enough for its far side to take some
    // of that back.
    mb.halo(hull, CAR_HULL_N, spread, r, g, b, 0.45);
    this.emitBody(hull, c.color, grade);

    // Cockpit and accents.
    mb.color255(10, 16, 26, 0.85);
    mb.roundRect(-0.55, -0.62, 1.15, 1.24, 0.28);
    mb.color255(255, 255, 255, 0.16);
    mb.rect(-1.9, -0.12, 3.4, 0.24);
    // Rear wing, until something takes it off.
    if (c.dmgRear < WING_LOST) {
      const wing = shadeRgb(c.color, -0.6);
      mb.color(wing[0], wing[1], wing[2], 1);
      mb.roundRect(-2.15, -0.95, 0.34, 1.9, 0.1);
    }

    // Headlights and their beam, which fades out over ten metres. A folded
    // nose takes the lamp on that corner with it.
    if (c.speed > 0.5 || c.throttle > 0) {
      mb.color255(255, 244, 214, 0.95);
      if (!lampOut(c.dmgFront, c.dmgLeft, c.dmgRight, 1)) mb.rect(1.92, -0.78, 0.2, 0.4);
      if (!lampOut(c.dmgFront, c.dmgLeft, c.dmgRight, -1)) mb.rect(1.92, 0.38, 0.2, 0.4);
      const br = 1;
      const bg = 240 / 255;
      const bb = 200 / 255;
      mb.vertC(2.0, -0.8, 0, 0, br, bg, bb, 0.13);
      mb.vertC(13, -3.6, 0, 0, br, bg, bb, 0);
      mb.vertC(13, 3.6, 0, 0, br, bg, bb, 0);
      mb.vertC(2.0, -0.8, 0, 0, br, bg, bb, 0.13);
      mb.vertC(13, 3.6, 0, 0, br, bg, bb, 0);
      mb.vertC(2.0, 0.8, 0, 0, br, bg, bb, 0.13);
    }

    // Brake lights, glowing when they are on.
    for (const y of BRAKE_LIGHTS) {
      if (c.braking) {
        rectPts(this.quadPts, -2.12, y, 0.18, 0.5);
        mb.halo(this.quadPts, 4, 0.3, 1, 45 / 255, 85 / 255, 0.5);
      }
      c.braking ? mb.color255(255, 45, 85, 1) : mb.color255(150, 30, 50, 0.75);
      mb.rect(-2.12, y, 0.18, 0.5);
    }

    if (c.isLocal) {
      mb.color255(255, 255, 255, 0.55);
      mb.ribbon(hull, CAR_HULL_N, 0.07, true);
    }
    // Buckled metal catches the light along the fold.
    if (grade > 0) {
      mb.color255(12, 14, 20, 0.2 + 0.13 * grade);
      mb.ribbon(hull, CAR_HULL_N, 0.06, true);
    }
    mb.pop();
  }

  /**
   * The body, as a fan carrying the 2D renderer's vertical gradient in its
   * vertices: lighter along the top edge, the car's own colour just above the
   * middle, darkest at the bottom.
   */
  private emitBody(hull: Float32Array, color: number, grade: number) {
    const mb = this.world;
    // Paint does not survive an accident either.
    const dirt = -0.09 * grade;
    const top = shadeRgb(color, -0.35 + dirt);
    const mid = shadeRgb(color, dirt);
    const bot = shadeRgb(color, -0.55 + dirt);
    const at = (y: number) => {
      const t = Math.min(1, Math.max(0, (y + 1) / 2));
      const low = t < 0.45;
      const from = low ? top : mid;
      const to = low ? mid : bot;
      const k = low ? t / 0.45 : (t - 0.45) / 0.55;
      this.gcol[0] = from[0] + (to[0] - from[0]) * k;
      this.gcol[1] = from[1] + (to[1] - from[1]) * k;
      this.gcol[2] = from[2] + (to[2] - from[2]) * k;
      return this.gcol;
    };
    const put = (x: number, y: number) => {
      const c = at(y);
      mb.vertC(x, y, 0, 0, c[0], c[1], c[2], 1);
    };
    for (let i = 0; i < CAR_HULL_N; i++) {
      const j = (i + 1) % CAR_HULL_N;
      put(0, 0);
      put(hull[i * 2], hull[i * 2 + 1]);
      put(hull[j * 2], hull[j * 2 + 1]);
    }
  }


  /** The authority's pose, drawn as a dashed wireframe next to the prediction. */
  private emitGhost(g: GhostCar) {
    const mb = this.world;
    mb.push(g.x, g.y, g.heading);
    mb.color255(120, 255, 214, 0.75);
    mb.dashed(CAR_HULL, CAR_HULL_N, true, 0.45, 0.32, 0.09);
    mb.color255(120, 255, 214, 0.9);
    mb.circle(0, 0, 0.16, 12);
    mb.pop();
  }

  private emitSpeedLines(w: number, h: number, speed: number) {
    const t = Math.max(0, (speed - 34) / 34);
    if (t <= 0.01) return;
    const cx = w / 2;
    const cy = h / 2;
    const r = Math.max(w, h) * 0.42;
    this.screen.color255(219, 234, 254, Math.min(0.3, t * 0.3));
    for (let i = 0; i < 26; i++) {
      const a = (i / 26) * Math.PI * 2 + performance.now() * 0.0004;
      const len = 30 + Math.random() * 70 * t;
      this.screen.segment(
        cx + Math.cos(a) * r, cy + Math.sin(a) * r,
        cx + Math.cos(a) * (r + len), cy + Math.sin(a) * (r + len),
        1.4,
      );
    }
  }

  // ---------------------------------------------------------- nameplates --

  private drawNameplates(cars: DrawCar[], w: number, h: number, sx: number, sy: number) {
    const ctx = this.plates;
    if (!ctx) return;
    const dpr = this.size.dpr;
    if (this.overlay.width !== this.size.w || this.overlay.height !== this.size.h) {
      this.overlay.width = this.size.w;
      this.overlay.height = this.size.h;
    }
    // One composited clear, and only when there was something on it.
    if (this.platesDirty) ctx.clearRect(0, 0, this.overlay.width, this.overlay.height);
    this.platesDirty = false;

    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.font = '600 11px ui-monospace, "SF Mono", Menlo, monospace';
    ctx.textAlign = 'center';
    const cos = Math.cos(this.camRot);
    const sin = Math.sin(this.camRot);
    for (const c of cars) {
      if (c.isLocal) continue;
      // Same transform the world pass uses -- translate, flip Y, then rotate --
      // collapsed into one step, so a plate stays pinned to its car once the
      // camera is turning.
      const dx = c.x - this.camX;
      const dy = c.y - this.camY;
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
      this.platesDirty = true;
    }
  }
}

// ------------------------------------------------------------------ utils --

/** Pull a point on the track edge towards the centreline, for the curbs. */
function inset(
  pt: readonly [number, number],
  points: Float32Array,
  i: number,
  amount: number,
): [number, number] {
  const dx = points[i * 2] - pt[0];
  const dy = points[i * 2 + 1] - pt[1];
  const len = Math.hypot(dx, dy) || 1;
  return [pt[0] + (dx / len) * amount, pt[1] + (dy / len) * amount];
}

/** The four corners of a rectangle, for something that wants an outline. */
function rectPts(out: Float32Array, x: number, y: number, w: number, h: number) {
  out[0] = x;
  out[1] = y;
  out[2] = x + w;
  out[3] = y;
  out[4] = x + w;
  out[5] = y + h;
  out[6] = x;
  out[7] = y + h;
}
