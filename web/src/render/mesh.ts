// Triangles, and the small amount of state it takes to write them comfortably.
//
// Both GPU backends speak the same vertex: position, texture coordinate, and a
// premultiplied colour. Premultiplied throughout means one blend mode for the
// whole renderer (`ONE, ONE_MINUS_SRC_ALPHA`), correct compositing into the
// skid-mark texture, and a soft edge that fades by scaling the whole colour
// rather than only its alpha.
//
// The transform stack is here for one reason: a car is drawn in the car's own
// frame in the 2D renderer, and it should read the same way here rather than
// being retyped as two dozen rotate-and-add expressions.

/** x, y, u, v, r, g, b, a. */
export const VERT_FLOATS = 8;

/** Vertex data on its way to a GPU buffer, which will not take a shared one. */
export type Verts = Float32Array<ArrayBuffer>;

/**
 * What a blurred edge has left at each third of its reach, as a fraction of the
 * value against the shape: `0.5 * erfc(d / (sigma * sqrt2))` sampled at 2/3,
 * 4/3 and 2 sigma, with the tail rounded down to nothing. The bands `halo`
 * emits interpolate between these.
 */
const HALO_FALLOFF = [1, 0.505, 0.182, 0] as const;

export class MeshBuilder {
  data: Verts;
  /** Vertices written so far. */
  n = 0;

  private cr = 1;
  private cg = 1;
  private cb = 1;
  private ca = 1;
  // 2x2 plus translation, applied to every vertex on the way in.
  private xx = 1;
  private xy = 0;
  private yx = 0;
  private yy = 1;
  private tx = 0;
  private ty = 0;
  private saved: number[] = [];
  // Per-vertex normals for `ribbon` and `halo`. Reused: a car outline is
  // extruded twice a frame per car, and neither keeps the buffer past the
  // call, so a fresh pair of typed arrays each time is pure garbage.
  private nx = new Float32Array(16);
  private ny = new Float32Array(16);

  constructor(capacity = 4096) {
    this.data = new Float32Array(capacity * VERT_FLOATS);
  }

  reset() {
    this.n = 0;
    this.xx = this.yy = 1;
    this.xy = this.yx = this.tx = this.ty = 0;
    this.saved.length = 0;
  }

  /** Vertices written, as a count of floats -- what an upload wants. */
  get floats(): number {
    return this.n * VERT_FLOATS;
  }

  /** Grow the extrusion scratch to hold `count` vertices. */
  private normals(count: number) {
    if (this.nx.length >= count) return;
    this.nx = new Float32Array(count);
    this.ny = new Float32Array(count);
  }

  private room(verts: number) {
    const need = (this.n + verts) * VERT_FLOATS;
    if (need <= this.data.length) return;
    let cap = this.data.length || VERT_FLOATS;
    while (cap < need) cap *= 2;
    const next = new Float32Array(cap);
    next.set(this.data.subarray(0, this.n * VERT_FLOATS));
    this.data = next;
  }

  // ---------------------------------------------------------------- state --

  color(r: number, g: number, b: number, a = 1): this {
    this.cr = r;
    this.cg = g;
    this.cb = b;
    this.ca = a;
    return this;
  }

  /** 0xrrggbb, the form car colours and the palette arrive in. */
  colorHex(hex: number, a = 1): this {
    return this.color(((hex >> 16) & 255) / 255, ((hex >> 8) & 255) / 255, (hex & 255) / 255, a);
  }

  /** 0..255 channels, the form the 2D renderer's rgba() strings are written in. */
  color255(r: number, g: number, b: number, a = 1): this {
    return this.color(r / 255, g / 255, b / 255, a);
  }

  push(x = 0, y = 0, rot = 0): this {
    this.saved.push(this.xx, this.xy, this.yx, this.yy, this.tx, this.ty);
    const c = Math.cos(rot);
    const s = Math.sin(rot);
    // Concatenate: this = this * translate(x, y) * rotate(rot).
    const nxx = this.xx * c + this.yx * s;
    const nxy = this.xy * c + this.yy * s;
    const nyx = this.xx * -s + this.yx * c;
    const nyy = this.xy * -s + this.yy * c;
    this.tx += this.xx * x + this.yx * y;
    this.ty += this.xy * x + this.yy * y;
    this.xx = nxx;
    this.xy = nxy;
    this.yx = nyx;
    this.yy = nyy;
    return this;
  }

  pop(): this {
    this.ty = this.saved.pop()!;
    this.tx = this.saved.pop()!;
    this.yy = this.saved.pop()!;
    this.yx = this.saved.pop()!;
    this.xy = this.saved.pop()!;
    this.xx = this.saved.pop()!;
    return this;
  }

  // ------------------------------------------------------------ primitives --

  /** One vertex in the current colour and frame. */
  vert(x: number, y: number, u = 0, v = 0) {
    this.room(1);
    const d = this.data;
    let o = this.n * VERT_FLOATS;
    d[o++] = this.xx * x + this.yx * y + this.tx;
    d[o++] = this.xy * x + this.yy * y + this.ty;
    d[o++] = u;
    d[o++] = v;
    d[o++] = this.cr * this.ca;
    d[o++] = this.cg * this.ca;
    d[o++] = this.cb * this.ca;
    d[o] = this.ca;
    this.n++;
  }

  /** One vertex carrying its own colour, for gradients. */
  vertC(x: number, y: number, u: number, v: number, r: number, g: number, b: number, a: number) {
    this.room(1);
    const d = this.data;
    let o = this.n * VERT_FLOATS;
    d[o++] = this.xx * x + this.yx * y + this.tx;
    d[o++] = this.xy * x + this.yy * y + this.ty;
    d[o++] = u;
    d[o++] = v;
    d[o++] = r * a;
    d[o++] = g * a;
    d[o++] = b * a;
    d[o] = a;
    this.n++;
  }

  tri(x0: number, y0: number, x1: number, y1: number, x2: number, y2: number) {
    this.vert(x0, y0);
    this.vert(x1, y1);
    this.vert(x2, y2);
  }

  /** Four corners in order round the shape. */
  quad(
    x0: number, y0: number,
    x1: number, y1: number,
    x2: number, y2: number,
    x3: number, y3: number,
  ) {
    this.tri(x0, y0, x1, y1, x2, y2);
    this.tri(x0, y0, x2, y2, x3, y3);
  }

  /** Axis-aligned, in the current frame, the way `fillRect` takes it. */
  rect(x: number, y: number, w: number, h: number) {
    this.quad(x, y, x + w, y, x + w, y + h, x, y + h);
  }

  /** A quad carrying texture coordinates, corners in the same order. */
  texQuad(
    x0: number, y0: number, u0: number, v0: number,
    x1: number, y1: number, u1: number, v1: number,
    x2: number, y2: number, u2: number, v2: number,
    x3: number, y3: number, u3: number, v3: number,
  ) {
    this.vert(x0, y0, u0, v0);
    this.vert(x1, y1, u1, v1);
    this.vert(x2, y2, u2, v2);
    this.vert(x0, y0, u0, v0);
    this.vert(x2, y2, u2, v2);
    this.vert(x3, y3, u3, v3);
  }

  /**
   * Rounded rectangle, as a fan around its centre.
   *
   * `SEG` corner segments is where a 0.14 m wheel radius stops reading as a
   * polygon on a screen that shows a metre in fifteen pixels.
   */
  roundRect(x: number, y: number, w: number, h: number, r: number) {
    const SEG = 4;
    const rad = Math.min(r, Math.min(w, h) / 2);
    const cx = x + w / 2;
    const cy = y + h / 2;
    const corners = [
      [x + w - rad, y + h - rad, 0],
      [x + rad, y + h - rad, Math.PI / 2],
      [x + rad, y + rad, Math.PI],
      [x + w - rad, y + rad, Math.PI * 1.5],
    ] as const;
    let px = 0;
    let py = 0;
    let first = true;
    for (const [ox, oy, a0] of corners) {
      for (let s = 0; s <= SEG; s++) {
        const a = a0 + (s / SEG) * (Math.PI / 2);
        const nx = ox + Math.cos(a) * rad;
        const ny = oy + Math.sin(a) * rad;
        if (!first) this.tri(cx, cy, px, py, nx, ny);
        px = nx;
        py = ny;
        first = false;
      }
    }
    // Close the loop back to where the first corner started.
    this.tri(cx, cy, px, py, x + w, y + h - rad);
  }

  /**
   * A disc as one quad with unit-circle texture coordinates: the fragment
   * shader cuts the circle out of it, which is both rounder and cheaper than a
   * fan with enough segments to look round.
   */
  disc(cx: number, cy: number, r: number) {
    this.texQuad(
      cx - r, cy - r, -1, -1,
      cx + r, cy - r, 1, -1,
      cx + r, cy + r, 1, 1,
      cx - r, cy + r, -1, 1,
    );
  }

  /** Convex polygon as a fan, xy pairs. */
  fan(pts: ArrayLike<number>, count = pts.length / 2) {
    for (let i = 1; i < count - 1; i++) {
      this.tri(pts[0], pts[1], pts[i * 2], pts[i * 2 + 1], pts[i * 2 + 2], pts[i * 2 + 3]);
    }
  }

  /**
   * A polyline extruded to `width`, mitred at the joints -- the triangle
   * equivalent of a stroke. Used for the barriers, whose glow is three of these
   * stacked, and for outlines.
   */
  ribbon(pts: ArrayLike<number>, count: number, width: number, closed: boolean) {
    const half = width / 2;
    this.normals(count);
    const nx = this.nx;
    const ny = this.ny;
    for (let i = 0; i < count; i++) {
      const p = i === 0 ? (closed ? count - 1 : 0) : i - 1;
      const n = i === count - 1 ? (closed ? 0 : count - 1) : i + 1;
      let ax = pts[i * 2] - pts[p * 2];
      let ay = pts[i * 2 + 1] - pts[p * 2 + 1];
      let bx = pts[n * 2] - pts[i * 2];
      let by = pts[n * 2 + 1] - pts[i * 2 + 1];
      const al = Math.hypot(ax, ay) || 1;
      const bl = Math.hypot(bx, by) || 1;
      ax /= al;
      ay /= al;
      bx /= bl;
      by /= bl;
      // Average the two segment normals, then lengthen the miter so the
      // ribbon keeps its width through the corner. Clamped, or a hairpin
      // throws a spike halfway across the track.
      let mx = -(ay + by);
      let my = ax + bx;
      const ml = Math.hypot(mx, my);
      if (ml < 1e-6) {
        mx = -ay;
        my = ax;
      } else {
        mx /= ml;
        my /= ml;
      }
      const cos = Math.max(0.35, mx * -by + my * bx);
      nx[i] = (mx / cos) * half;
      ny[i] = (my / cos) * half;
    }
    const last = closed ? count : count - 1;
    for (let i = 0; i < last; i++) {
      const j = (i + 1) % count;
      const x0 = pts[i * 2];
      const y0 = pts[i * 2 + 1];
      const x1 = pts[j * 2];
      const y1 = pts[j * 2 + 1];
      this.quad(
        x0 + nx[i], y0 + ny[i],
        x1 + nx[j], y1 + ny[j],
        x1 - nx[j], y1 - ny[j],
        x0 - nx[i], y0 - ny[i],
      );
    }
  }

  /** Filled circle, for the handful of dots that are not particles. */
  circle(cx: number, cy: number, r: number, seg = 16) {
    let px = cx + r;
    let py = cy;
    for (let i = 1; i <= seg; i++) {
      const a = (i / seg) * Math.PI * 2;
      const nx = cx + Math.cos(a) * r;
      const ny = cy + Math.sin(a) * r;
      this.tri(cx, cy, px, py, nx, ny);
      px = nx;
      py = ny;
    }
  }

  /**
   * A glow hugging the outside of a closed outline, fading out over `spread`.
   *
   * This is what the 2D renderer gets from `shadowBlur`, and the shape of the
   * falloff is the whole game. A canvas shadow is a Gaussian blur of the
   * silhouette, so its edge follows an erfc: half the light is gone a third of
   * the way out and there is almost none left at the end. A single band ramping
   * straight from `a` to nothing keeps far too much of it across the middle,
   * and on the local car -- whose blur is nearly twice as wide -- that reads as
   * a floodlight rather than a glow.
   *
   * So the ring is emitted as bands, at the levels the real profile passes
   * through. Still no blur, still a handful of triangles, and now it falls off
   * like the thing it is standing in for.
   */
  halo(pts: ArrayLike<number>, count: number, spread: number, r: number, g: number, b: number, a: number) {
    // Which way is out depends on the winding, and an outline is written in
    // whichever order read best when it was drawn.
    let area = 0;
    for (let i = 0; i < count; i++) {
      const j = (i + 1) % count;
      area += pts[i * 2] * pts[j * 2 + 1] - pts[j * 2] * pts[i * 2 + 1];
    }
    const sign = area > 0 ? -1 : 1;
    this.normals(count);
    const ox = this.nx;
    const oy = this.ny;
    for (let i = 0; i < count; i++) {
      const p = (i + count - 1) % count;
      const n = (i + 1) % count;
      let ax = pts[i * 2] - pts[p * 2];
      let ay = pts[i * 2 + 1] - pts[p * 2 + 1];
      let bx = pts[n * 2] - pts[i * 2];
      let by = pts[n * 2 + 1] - pts[i * 2 + 1];
      const al = Math.hypot(ax, ay) || 1;
      const bl = Math.hypot(bx, by) || 1;
      ax /= al;
      ay /= al;
      bx /= bl;
      by /= bl;
      let mx = -(ay + by) * sign;
      let my = (ax + bx) * sign;
      const ml = Math.hypot(mx, my) || 1;
      ox[i] = mx / ml;
      oy[i] = my / ml;
    }
    const bands = HALO_FALLOFF.length - 1;
    for (let band = 0; band < bands; band++) {
      const d0 = (band / bands) * spread;
      const d1 = ((band + 1) / bands) * spread;
      const a0 = a * HALO_FALLOFF[band];
      const a1 = a * HALO_FALLOFF[band + 1];
      for (let i = 0; i < count; i++) {
        const j = (i + 1) % count;
        const xi = pts[i * 2];
        const yi = pts[i * 2 + 1];
        const xj = pts[j * 2];
        const yj = pts[j * 2 + 1];
        this.vertC(xi + ox[i] * d0, yi + oy[i] * d0, 0, 0, r, g, b, a0);
        this.vertC(xj + ox[j] * d0, yj + oy[j] * d0, 0, 0, r, g, b, a0);
        this.vertC(xj + ox[j] * d1, yj + oy[j] * d1, 0, 0, r, g, b, a1);
        this.vertC(xi + ox[i] * d0, yi + oy[i] * d0, 0, 0, r, g, b, a0);
        this.vertC(xj + ox[j] * d1, yj + oy[j] * d1, 0, 0, r, g, b, a1);
        this.vertC(xi + ox[i] * d1, yi + oy[i] * d1, 0, 0, r, g, b, a1);
      }
    }
  }

  /** One straight segment of a stroke: the dashes and the skid marks. */
  segment(x0: number, y0: number, x1: number, y1: number, width: number) {
    const dx = x1 - x0;
    const dy = y1 - y0;
    const len = Math.hypot(dx, dy);
    if (len < 1e-6) {
      this.rect(x0 - width / 2, y0 - width / 2, width, width);
      return;
    }
    const px = (-dy / len) * (width / 2);
    const py = (dx / len) * (width / 2);
    this.quad(x0 + px, y0 + py, x1 + px, y1 + py, x1 - px, y1 - py, x0 - px, y0 - py);
  }

  /** A dashed polyline, walked in world units the way `setLineDash` walks it. */
  dashed(pts: ArrayLike<number>, count: number, closed: boolean, on: number, off: number, width: number) {
    let phase = 0;
    const last = closed ? count : count - 1;
    for (let i = 0; i < last; i++) {
      const j = (i + 1) % count;
      const x0 = pts[i * 2];
      const y0 = pts[i * 2 + 1];
      const dx = pts[j * 2] - x0;
      const dy = pts[j * 2 + 1] - y0;
      const len = Math.hypot(dx, dy);
      let t = 0;
      // Both floors are load-bearing. The walk stops a hair before the end of
      // the segment, or the last sliver rounds to a step of zero and it never
      // arrives; and a phase that lands exactly on a dash boundary has nothing
      // left of that dash, so it steps over the boundary rather than standing
      // on it -- which would freeze `phase` and leave the rest of the line
      // undrawn.
      while (len - t > 1e-6) {
        const cycle = phase % (on + off);
        const inking = cycle < on;
        const left = Math.max(inking ? on - cycle : on + off - cycle, 1e-4);
        const run = Math.min(left, len - t);
        if (inking) {
          this.segment(x0 + (dx * t) / len, y0 + (dy * t) / len, x0 + (dx * (t + run)) / len, y0 + (dy * (t + run)) / len, width);
        }
        t += run;
        phase += run;
      }
    }
  }
}
