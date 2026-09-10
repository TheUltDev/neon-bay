// The few numbers all three backends have to agree on.
//
// Everything else about a renderer is its own business, but a car drawn with a
// different outline or a barrier lit with a different falloff would show up the
// moment anyone switched tiers -- so the shape and the glow live here, once.

/**
 * The neon barrier glow, widest and faintest first, as [line width, alpha].
 *
 * Three stacked strokes rather than one stroke plus a blur: same falloff, and
 * it costs three passes over a polyline instead of a Gaussian blur across a
 * scratch surface the size of the entire track. The GPU backends extrude the
 * same three widths into triangle ribbons.
 */
export const EDGE_GLOW = [
  [2.4, 0.06],
  [1.2, 0.14],
  [0.42, 1],
] as const;

/** Left barrier, right barrier: cyan out, pink back. */
export const EDGE_RGB = [
  [56, 232, 255],
  [255, 77, 157],
] as const;

/** Wheel positions in the car frame, and whether the wheel steers. */
export const WHEELS = [
  [1.28, 0.92, 1],
  [1.28, -0.92, 1],
  [-1.32, 0.95, 0],
  [-1.32, -0.95, 0],
] as const;

/** The car silhouette in its own frame, x forward, in metres. */
export const CAR_OUTLINE = [
  2.1, -0.62, 1.55, -0.95, -1.65, -0.95, -2.1, -0.66,
  -2.1, 0.66, -1.65, 0.95, 1.55, 0.95, 2.1, 0.62,
] as const;

/**
 * The silhouette again, subdivided: every corner plus the midpoint of every
 * edge, sixteen points.
 *
 * On an undamaged car this is the same shape -- a midpoint of a straight edge
 * is on the edge -- which is the point of building it this way rather than
 * drawing a second outline. What it buys is somewhere for a dent to go. Eight
 * points can only be pulled inwards, which reads as a car that has shrunk;
 * sixteen can pucker an edge in the middle and leave its ends where they were,
 * which reads as one that has been hit.
 */
export const CAR_HULL_N = 16;
export const CAR_HULL = (() => {
  const h = new Float32Array(CAR_HULL_N * 2);
  for (let i = 0; i < 8; i++) {
    const j = (i + 1) % 8;
    h[i * 4] = CAR_OUTLINE[i * 2];
    h[i * 4 + 1] = CAR_OUTLINE[i * 2 + 1];
    h[i * 4 + 2] = (CAR_OUTLINE[i * 2] + CAR_OUTLINE[j * 2]) / 2;
    h[i * 4 + 3] = (CAR_OUTLINE[i * 2 + 1] + CAR_OUTLINE[j * 2 + 1]) / 2;
  }
  return h;
})();

/**
 * How far each hull point is thrown off the fold, as a fraction of how deep the
 * fold is there. Fixed, not random: a car's dents must not shimmer between
 * frames, and every backend has to agree on the same wreck.
 */
const CREASE = [
  0.31, -0.62, 0.18, 0.74, -0.45, 0.27, 0.66, -0.21,
  -0.34, 0.58, -0.71, 0.15, 0.42, -0.5, -0.23, 0.69,
] as const;

/** Reused: one car's outline is built and consumed before the next one's. */
const CRUSHED = new Float32Array(CAR_HULL_N * 2);

/**
 * [`CAR_HULL`] with the crush from `physics::damage` folded into it.
 *
 * Each point is pulled in by how much it *faces* each crushed face -- the
 * square of its position along that axis, so a nose-on hit flattens the nose
 * and leaves the doors alone, while a corner hit takes both. Then the fold is
 * thrown sideways by [`CREASE`], because metal that has nowhere to go buckles
 * rather than scaling.
 *
 * Returns a shared buffer. Draw with it before asking for another.
 */
export function crushOutline(front: number, rear: number, left: number, right: number): Float32Array {
  for (let i = 0; i < CAR_HULL_N; i++) {
    const x = CAR_HULL[i * 2];
    const y = CAR_HULL[i * 2 + 1];
    const fx = x / 2.1;
    const fy = y / 0.95;
    const sx = fx * fx;
    const sy = fy * fy;
    const dx = fx > 0 ? sx * front : -sx * rear;
    const dy = fy > 0 ? sy * left : -sy * right;
    const c = CREASE[i];
    CRUSHED[i * 2] = x - dx + c * dy * 0.55;
    CRUSHED[i * 2 + 1] = y - dy + c * dx * 0.6;
  }
  return CRUSHED;
}

/** Rear crush past which the wing is somewhere behind you on the track, m. */
export const WING_LOST = 0.18;

/**
 * Whether the headlight on the `+y` (`side` 1) or `-y` (-1) corner is out.
 *
 * A nose has to be properly folded before a lamp goes, and it goes on the side
 * that took the hit: `physics::damage` splits a corner impact between the front
 * face and the side it came in on, so the two together say which corner.
 */
export function lampOut(front: number, left: number, right: number, side: 1 | -1): boolean {
  if (front < 0.15) return false;
  return side > 0 ? left >= right : right >= left;
}

/** Lighten (positive) or darken (negative) a `#rrggbb` string. */
export function shade(hex: string, amount: number): string {
  const n = parseInt(hex.slice(1), 16);
  const f = (shift: number) => {
    const v = (n >> shift) & 255;
    const out = amount < 0 ? v * (1 + amount) : v + (255 - v) * amount;
    return Math.round(Math.max(0, Math.min(255, out)));
  };
  return `rgb(${f(16)},${f(8)},${f(0)})`;
}

/** The same, on a packed 0xrrggbb, returning 0..1 channels for a shader. */
export function shadeRgb(color: number, amount: number): [number, number, number] {
  const f = (shift: number) => {
    const v = ((color >> shift) & 255) / 255;
    return Math.max(0, Math.min(1, amount < 0 ? v * (1 + amount) : v + (1 - v) * amount));
  };
  return [f(16), f(8), f(0)];
}

/** HSL to 0..1 RGB, for the sparks -- the one colour that is computed, not picked. */
export function hslRgb(h: number, s: number, l: number): [number, number, number] {
  const c = (1 - Math.abs(2 * l - 1)) * s;
  const hp = (((h % 360) + 360) % 360) / 60;
  const x = c * (1 - Math.abs((hp % 2) - 1));
  const m = l - c / 2;
  const t: [number, number, number] =
    hp < 1 ? [c, x, 0] : hp < 2 ? [x, c, 0] : hp < 3 ? [0, c, x] : hp < 4 ? [0, x, c] : hp < 5 ? [x, 0, c] : [c, 0, x];
  return [t[0] + m, t[1] + m, t[2] + m];
}

/** Side of the road-surface noise tile, in texels and in metres: the pattern
 *  is laid out in world units, so one tile covers 128 m of track. */
export const ASPHALT_TILE = 128;

/**
 * The road surface, as one opaque noise tile.
 *
 * The base colour is baked into the tile rather than laid down first and then
 * glazed with translucent noise -- the track outline is a 768-segment path and
 * filling it is not cheap, so it is worth filling once. The numbers are what
 * #14181f glazed with the old translucent grain actually came out as, mean and
 * amplitude both.
 */
export function asphaltPixels(): Uint8ClampedArray<ArrayBuffer> {
  const px = new Uint8ClampedArray(ASPHALT_TILE * ASPHALT_TILE * 4);
  for (let i = 0; i < ASPHALT_TILE * ASPHALT_TILE; i++) {
    const n = (Math.random() * 46 - 23) * 0.0392;
    px[i * 4] = 24 + n;
    px[i * 4 + 1] = 28 + n;
    px[i * 4 + 2] = 35 + n;
    px[i * 4 + 3] = 255;
  }
  return px;
}
