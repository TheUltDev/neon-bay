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

/** The car silhouette in its own frame, x forward, in metres. */
export const CAR_OUTLINE = [
  2.1, -0.62, 1.55, -0.95, -1.65, -0.95, -2.1, -0.66,
  -2.1, 0.66, -1.65, 0.95, 1.55, 0.95, 2.1, 0.62,
] as const;

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
