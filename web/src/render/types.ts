// What every renderer agrees on: the frame it is handed, and what it can say
// about itself afterwards.
//
// Three backends implement this -- WebGPU, WebGL 2 and Canvas2D -- and the
// game does not know which one it got. `createRenderer` picks the best the
// browser will give it; everything else here talks to the interface.

import type { Sim } from '../sim';

export interface DrawCar {
  slot: number;
  x: number;
  y: number;
  heading: number;
  steer: number;
  color: number;
  name: string;
  isLocal: boolean;
  isBot: boolean;
  speed: number;
  wheelSpin: number;
  braking: boolean;
  throttle: number;
  lap: number;
}

export interface GhostCar {
  x: number;
  y: number;
  heading: number;
}

export type Backend = 'webgpu' | 'webgl2' | 'canvas2d';

/** What the renderer panel reports. Live: fields are updated in place. */
export interface RendererInfo {
  backend: Backend;
  /** How the API calls itself: "WebGPU", "WebGL 2", "Canvas 2D". */
  api: string;
  /** The hardware or engine the frames actually come out of, abbreviated. */
  device: string;
  /** The unabridged string behind `device`, for the tooltip. */
  detail: string;
  /** Why this tier and not the one above it. Null when we got the best one. */
  fallback: string | null;
}

/** Drawing-buffer size, in device pixels, plus the ratio it was derived from. */
export interface RenderSize {
  w: number;
  h: number;
  dpr: number;
}

export interface Renderer {
  camX: number;
  camY: number;
  camZoom: number;
  camRot: number;
  rotateCamera: boolean;
  showGhost: boolean;
  readonly info: RendererInfo;
  readonly size: RenderSize;
  /** The element being drawn into. It changes when the tier does. */
  readonly canvas: HTMLCanvasElement;

  updateCamera(
    target: { x: number; y: number; heading: number } | null,
    vx: number,
    vy: number,
    dt: number,
    snap?: boolean,
  ): void;
  draw(cars: DrawCar[], ghost: GhostCar | null, dt: number, localSpeed: number): void;
  drawMinimap(canvas: HTMLCanvasElement, sim: Sim, cars: DrawCar[]): void;

  addSkid(wheel: number, x: number, y: number, alpha: number): void;
  addSmoke(x: number, y: number, vx: number, vy: number, strength: number): void;
  addSpark(x: number, y: number, vx: number, vy: number): void;
  impulse(strength: number): void;

  worldFromScreen(): { x: number; y: number; zoom: number };
  dispose(): void;
}
