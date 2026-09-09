// Which renderer the browser gets.
//
// WebGPU if it is there, WebGL 2 if it is not, and the Canvas2D renderer if
// neither -- picked at boot, reported in the HUD, and overridable with
// `?renderer=webgl` (or `webgpu`, or `canvas`) when you want to see one of the
// other two on a machine that would not have chosen it.
//
// The order matters more than the tiers do. A canvas element can only ever have
// one kind of context, so an attempt that got as far as `getContext` has spent
// the element whether or not it went on to work -- hence the swap for a fresh
// one after every failure, which is cheap and is the only way the fallback can
// be honest rather than hopeful. `switchRenderer` swaps the element for the
// same reason, which is what lets the panel change tiers without a reload.

import type { Sim } from '../sim';
import { Canvas2DRenderer } from './canvas2d';
import { GpuRenderer } from './gpu';
import type { Backend, Renderer } from './types';
import { createWebgl2Device } from './webgl2';
import { createWebgpuDevice } from './webgpu';

export type { Backend, DrawCar, GhostCar, Renderer, RendererInfo, RenderSize } from './types';

export type RendererChoice = 'auto' | 'webgpu' | 'webgl2' | 'canvas2d';

/** `?renderer=webgpu|webgl|canvas`, for looking at the other two on purpose. */
export function rendererPreference(search: string = location.search): RendererChoice {
  const q = (new URLSearchParams(search).get('renderer') ?? '').toLowerCase();
  if (q === 'webgpu' || q === 'gpu') return 'webgpu';
  if (q === 'webgl' || q === 'webgl2' || q === 'gl') return 'webgl2';
  if (q === 'canvas' || q === 'canvas2d' || q === '2d') return 'canvas2d';
  return 'auto';
}

export async function createRenderer(
  canvas: HTMLCanvasElement,
  sim: Sim,
  choice: RendererChoice = rendererPreference(),
): Promise<Renderer> {
  let target = canvas;
  // Only the tiers that were tried and would not have us. Which tier is running
  // is the picker's job to show, and it shows it by lighting a button -- a line
  // of prose repeating that would be noise, so the note under the panel answers
  // one question only: why not the tier above.
  const missed: string[] = [];

  if (choice === 'auto' || choice === 'webgpu') {
    try {
      const dev = await createWebgpuDevice(target);
      return new GpuRenderer(target, sim, dev, note(missed));
    } catch (e) {
      missed.push(`WebGPU: ${reason(e)}`);
      target = fresh(target);
    }
  }

  if (choice !== 'canvas2d') {
    const dev = createWebgl2Device(target);
    if (dev) return new GpuRenderer(target, sim, dev, note(missed));
    missed.push('WebGL 2: no context on this canvas');
    target = fresh(target);
  }

  return new Canvas2DRenderer(target, sim, note(missed));
}

/**
 * Swap the live renderer for another tier, without reloading the page.
 *
 * The replacement is built on a canvas staged behind the running one and only
 * takes its place once it exists, so the picture never blanks mid-switch and a
 * failure leaves the renderer that is working alone. Camera and toggles come
 * across; the skid-mark layer does not, because it lives in the old backend's
 * texture and every way of carrying it over costs more than a fresh lap's
 * worth of tire marks.
 */
export async function switchRenderer(
  from: Renderer,
  sim: Sim,
  choice: RendererChoice,
): Promise<Renderer> {
  const staged = stage(from.canvas);
  let next: Renderer;
  try {
    next = await createRenderer(staged, sim, choice);
  } catch (e) {
    staged.remove();
    throw e;
  }
  next.camX = from.camX;
  next.camY = from.camY;
  next.camZoom = from.camZoom;
  next.camRot = from.camRot;
  next.rotateCamera = from.rotateCamera;
  next.showGhost = from.showGhost;
  promote(from.canvas, next.canvas);
  from.dispose();
  return next;
}

/**
 * Which tiers this browser would actually hand over, and why not for the rest.
 *
 * The one already running needs no test -- it is running. The others are asked
 * the cheapest question that has a real answer: an adapter request for WebGPU,
 * a context on a throwaway canvas for WebGL 2. Guessing from the user agent
 * would be quicker and would be wrong on exactly the machines that matter.
 */
export async function backendSupport(have: Backend): Promise<Record<Backend, string | null>> {
  const out: Record<Backend, string | null> = { webgpu: null, webgl2: null, canvas2d: null };

  if (have !== 'webgpu') {
    if (!navigator.gpu) {
      out.webgpu = 'this browser has no WebGPU';
    } else {
      try {
        out.webgpu = (await navigator.gpu.requestAdapter()) ? null : 'WebGPU offered no adapter';
      } catch (e) {
        out.webgpu = `WebGPU: ${reason(e)}`;
      }
    }
  }

  if (have !== 'webgl2') {
    const probe = document.createElement('canvas').getContext('webgl2');
    if (!probe) out.webgl2 = 'this browser has no WebGL 2';
    else probe.getExtension('WEBGL_lose_context')?.loseContext();
  }

  return out;
}

function note(missed: string[]): string | null {
  return missed.length ? missed.join(' · ') : null;
}

function reason(e: unknown): string {
  return e instanceof Error ? e.message : String(e);
}

/**
 * A replacement for a canvas whose context type is already spoken for, in the
 * same place in the document with the same attributes.
 */
function fresh(old: HTMLCanvasElement): HTMLCanvasElement {
  const next = document.createElement('canvas');
  for (const a of Array.from(old.attributes)) next.setAttribute(a.name, a.value);
  old.replaceWith(next);
  return next;
}

/**
 * A second canvas in the same box as the stage, waiting its turn.
 *
 * It carries the stage's geometry inline rather than its id: the id is what the
 * stylesheet sizes, and two elements answering to it would be two stages. A
 * renderer built on this one can still measure itself correctly before anyone
 * can see it.
 */
function stage(live: HTMLCanvasElement): HTMLCanvasElement {
  const next = document.createElement('canvas');
  next.className = live.className;
  next.style.cssText = 'position:fixed;inset:0;width:100%;height:100%;visibility:hidden';
  live.after(next);
  return next;
}

/** Retire the old canvas and hand its identity to the new one. */
function promote(old: HTMLCanvasElement, next: HTMLCanvasElement) {
  const id = old.id;
  old.remove();
  next.id = id;
  next.removeAttribute('style');
}
