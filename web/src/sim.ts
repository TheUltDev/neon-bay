// Client-side simulation: the same Rust physics as the sidecar, compiled to
// wasm, plus the netcode that keeps it honest.
//
// The loop is the classic three-part arrangement:
//
//   1. PREDICT   every tick, step the local car immediately from local input,
//                so steering feels instant regardless of ping.
//   2. RECONCILE when an authoritative snapshot arrives for tick T, compare it
//                with what we predicted for T. If they differ, rewind to the
//                server's state and replay the inputs from T+1 to now -- with
//                the other cars put back where they were at each replayed
//                tick, so the contact is the one the authority resolved.
//   3. SMOOTH    a rewind moves the car. Rather than teleport, keep the visual
//                error as an offset and decay it to zero over ~200 ms.
//   4. INTERPOLATE the simulation moves in 60 Hz jumps and the display does not.
//                Draw the fraction of a tick the frame actually falls on, or
//                the car stutters against a camera that moves every frame.
//
// Because the wasm and the sidecar run bit-identical code (see
// scripts/verify-determinism.mjs), step 2 usually finds an error of exactly
// zero and step 3 has nothing to do. The machinery only earns its keep when
// packets are late, dropped, or the client is lying.

export const TICK_HZ = 60;
export const DT = 1 / TICK_HZ;

/** Ticks of input/state history kept for replay. ~4 s at 60 Hz. */
const HISTORY = 256;
/** Position error below this is treated as a perfect prediction. */
const EPSILON_POS = 0.0015;
const EPSILON_ANG = 0.0008;
/** Beyond this the correction is too big to hide; snap and flag a resync. */
const SNAP_DISTANCE = 9.0;
/** Time constant for bleeding off a visual correction. */
const SMOOTH_TIME = 0.22;

/** Field offsets inside a CarState record. Mirrors physics::car::CarState. */
export const F = {
  x: 0, y: 1, heading: 2, vx: 3, vy: 4, omega: 5, steer: 6, ax: 7,
  slipF: 8, slipR: 9, wheelSpin: 10, rpm: 11, gear: 12, s: 13, lat: 14,
  seg: 15, lap: 16, cp: 17, lapStart: 18, lastLap: 19, bestLap: 20,
  impact: 21, wall: 22, active: 23,
} as const;

export interface WasmExports {
  memory: WebAssembly.Memory;
  phys_init(): number;
  phys_cars_ptr(): number;
  phys_inputs_ptr(): number;
  phys_track_ptr(): number;
  phys_consts_ptr(): number;
  phys_consts_len(): number;
  phys_set_active(mask: number): void;
  phys_active(): number;
  phys_step(mask: number): void;
  phys_spawn(index: number, gridSlot: number): void;
  phys_despawn(index: number): void;
  phys_respawn(index: number): void;
  phys_set_tick(tick: number): void;
  phys_tick(): number;
  phys_reproject(index: number): void;
  phys_get(index: number, out: number): void;
  phys_set(index: number, src: number): void;
  phys_scratch_ptr(): number;
  phys_fingerprint(): number;
  phys_car_floats(): number;
  phys_input_floats(): number;
  phys_max_cars(): number;
  phys_track_samples(): number;
  phys_track_stride(): number;
  phys_grid_slot(slot: number, out: number): void;
}

export interface TrackData {
  /** Centerline points, 2 floats per sample. */
  points: Float32Array;
  /** Unit tangents, 2 floats per sample. */
  tangents: Float32Array;
  halfWidth: Float32Array;
  curvature: Float32Array;
  samples: number;
  length: number;
  ds: number;
  checkpoints: number;
  bounds: { minX: number; minY: number; maxX: number; maxY: number };
}

export interface CarConsts {
  halfLen: number;
  halfWid: number;
  maxCars: number;
  carFloats: number;
}

export type Input = { throttle: number; steer: number; brake: number; handbrake: number };

/** What actually gets drawn: prediction plus the residual being smoothed away. */
export interface RenderPose {
  x: number;
  y: number;
  heading: number;
  steer: number;
}

export interface NetStats {
  /** Distance between prediction and authority at the last snapshot, meters. */
  error: number;
  /** Rolling peak, for the telemetry graph. */
  errorPeak: number;
  corrections: number;
  resyncs: number;
  replayTicks: number;
  lastSnapshotTick: number;
}

export function wrapPi(a: number): number {
  while (a > Math.PI) a -= Math.PI * 2;
  while (a < -Math.PI) a += Math.PI * 2;
  return a;
}

export class Sim {
  readonly wasm: WasmExports;
  readonly cars: Float32Array;
  readonly inputs: Float32Array;
  readonly track: TrackData;
  readonly consts: CarConsts;
  readonly stride: number;

  /**
   * What this `physics.wasm` computes, as one number. The authority publishes
   * its own; if the two differ, this client and the sidecar are not running the
   * same simulation and every prediction below is a guess. See
   * `physics/src/fingerprint.rs`.
   */
  readonly fingerprint: number;

  /** Physics slot this client owns, or -1 while spectating. */
  localSlot = -1;
  /** Tick the client is currently simulating. Runs ahead of the server. */
  localTick = 0;

  // --- rollback bookkeeping ---
  private histTick = new Int32Array(HISTORY).fill(-1);
  private histInput: Float32Array;
  private histState: Float32Array;
  /** Pointer + view into the wasm-side staging buffer used for rollback. */
  private scratch: number;
  private scratchView!: Float32Array;
  private lastApplied = -1;

  // --- visual error smoothing ---
  private offX = 0;
  private offY = 0;
  private offHeading = 0;

  // --- sub-tick render interpolation ---
  // The pose one tick behind `localTick`. Frames do not land on tick
  // boundaries, so the renderer draws somewhere between this and the current
  // pose rather than holding the current one until the next step lands.
  private prevX = 0;
  private prevY = 0;
  private prevHeading = 0;
  private prevSteer = 0;
  private havePrev = false;

  /**
   * Called before each replayed tick, so the caller can put the remote cars
   * where the authority had them *then*.
   *
   * Without it a rollback re-runs the last few ticks against rivals frozen at
   * wherever they are now, which is a different collision from the one the
   * authority resolved -- so the replay disagrees, and the next snapshot
   * corrects it again.
   */
  onReplayTick: ((tick: number) => void) | null = null;

  stats: NetStats = {
    error: 0,
    errorPeak: 0,
    corrections: 0,
    resyncs: 0,
    replayTicks: 0,
    lastSnapshotTick: 0,
  };

  constructor(wasm: WasmExports) {
    this.wasm = wasm;
    wasm.phys_init();
    // Scored inside `phys_init`, where the module is still allowed to allocate;
    // this is only reading the number back.
    this.fingerprint = wasm.phys_fingerprint() >>> 0;
    this.stride = wasm.phys_car_floats();
    const maxCars = wasm.phys_max_cars();
    this.cars = new Float32Array(wasm.memory.buffer, wasm.phys_cars_ptr(), maxCars * this.stride);
    this.inputs = new Float32Array(
      wasm.memory.buffer,
      wasm.phys_inputs_ptr(),
      maxCars * wasm.phys_input_floats(),
    );

    const c = new Float32Array(wasm.memory.buffer, wasm.phys_consts_ptr(), wasm.phys_consts_len());
    this.consts = { halfLen: c[0], halfWid: c[1], maxCars, carFloats: this.stride };

    this.track = readTrack(wasm, c);

    this.histInput = new Float32Array(HISTORY * 4);
    this.histState = new Float32Array(HISTORY * this.stride);
    this.scratch = wasm.phys_scratch_ptr();
    this.scratchView = new Float32Array(wasm.memory.buffer, this.scratch, this.stride);
  }

  setLocalSlot(slot: number) {
    if (this.localSlot === slot) return;
    this.localSlot = slot;
    this.histTick.fill(-1);
    this.lastApplied = -1;
    this.offX = this.offY = this.offHeading = 0;
    this.havePrev = false;
  }

  setActive(mask: number) {
    this.wasm.phys_set_active(mask);
  }

  /** Park a non-simulated car so it still collides at its interpolated pose. */
  placeRemote(slot: number, x: number, y: number, heading: number, vx: number, vy: number) {
    const b = slot * this.stride;
    this.cars[b + F.x] = x;
    this.cars[b + F.y] = y;
    this.cars[b + F.heading] = heading;
    this.cars[b + F.vx] = vx;
    this.cars[b + F.vy] = vy;
    this.cars[b + F.active] = 1;
  }

  /** Advance the local car one tick under `input`, recording it for replay. */
  step(input: Input) {
    if (this.localSlot < 0) return;
    const slot = this.localSlot;
    const hi = (this.localTick % HISTORY) * 4;
    const si = slot * 4;

    // The controls go to the simulation and into the replay log together.
    this.inputs[si] = this.histInput[hi] = input.throttle;
    this.inputs[si + 1] = this.histInput[hi + 1] = input.steer;
    this.inputs[si + 2] = this.histInput[hi + 2] = input.brake;
    this.inputs[si + 3] = this.histInput[hi + 3] = input.handbrake;

    const b = slot * this.stride;
    this.prevX = this.cars[b + F.x];
    this.prevY = this.cars[b + F.y];
    this.prevHeading = this.cars[b + F.heading];
    this.prevSteer = this.cars[b + F.steer];
    this.havePrev = true;

    this.wasm.phys_set_tick(this.localTick);
    this.wasm.phys_step(1 << slot);
    this.localTick++;
    this.record(this.localTick);
  }

  /** File the local car's current state as what was simulated for `tick`. */
  private record(tick: number) {
    const h = tick % HISTORY;
    const b = this.localSlot * this.stride;
    this.histTick[h] = tick;
    this.histState.set(this.cars.subarray(b, b + this.stride), h * this.stride);
  }

  /**
   * Reconcile against the authority.
   *
   * `state` is a full CarState record as the sidecar had it at `tick`.
   */
  reconcile(tick: number, state: Float32Array) {
    if (this.localSlot < 0) return;
    if (tick <= this.lastApplied) return;
    this.lastApplied = tick;
    this.stats.lastSnapshotTick = tick;

    const slot = this.localSlot;
    const h = tick % HISTORY;
    const known = this.histTick[h] === tick && tick <= this.localTick;

    if (!known) {
      // No history for that tick: either we just joined, or we fell so far
      // behind that the ring buffer wrapped. Accept the server wholesale.
      this.hardSet(slot, tick, state);
      return;
    }

    const mine = this.histState.subarray(h * this.stride, (h + 1) * this.stride);
    const dx = state[F.x] - mine[F.x];
    const dy = state[F.y] - mine[F.y];
    const dh = wrapPi(state[F.heading] - mine[F.heading]);
    const err = Math.hypot(dx, dy);
    this.stats.error = err;
    this.stats.errorPeak = Math.max(this.stats.errorPeak * 0.97, err);

    if (err < EPSILON_POS && Math.abs(dh) < EPSILON_ANG) {
      // Prediction was exact. Adopt the authoritative record anyway so tiny
      // differences in fields we do not compare cannot accumulate.
      this.histState.set(state, h * this.stride);
      return;
    }

    if (err > SNAP_DISTANCE) {
      this.hardSet(slot, tick, state);
      this.stats.resyncs++;
      return;
    }

    // Remember where the car *appeared* to be, rewind, replay, then keep the
    // difference as a visual offset so the picture never jumps.
    const b = slot * this.stride;
    const wasX = this.cars[b + F.x];
    const wasY = this.cars[b + F.y];
    const wasH = this.cars[b + F.heading];
    const wasSteer = this.cars[b + F.steer];
    const preX = wasX + this.offX;
    const preY = wasY + this.offY;
    const preH = wasH + this.offHeading;

    this.scratchView.set(state);
    this.wasm.phys_set(slot, this.scratch);

    let replayed = 0;
    for (let t = tick; t < this.localTick; t++) {
      const j = t % HISTORY;
      if (this.histTick[j] !== t) break;
      this.inputs.set(this.histInput.subarray(j * 4, j * 4 + 4), slot * 4);
      this.onReplayTick?.(t);
      this.wasm.phys_set_tick(t);
      this.wasm.phys_step(1 << slot);
      this.record(t + 1);
      replayed++;
    }

    this.offX = preX - this.cars[b + F.x];
    this.offY = preY - this.cars[b + F.y];
    this.offHeading = wrapPi(preH - this.cars[b + F.heading]);
    // The replay moved the current pose. Carry the previous-tick pose the same
    // distance, so what the renderer interpolates across is still one tick of
    // motion rather than one tick plus the whole correction.
    this.prevX += this.cars[b + F.x] - wasX;
    this.prevY += this.cars[b + F.y] - wasY;
    this.prevHeading += wrapPi(this.cars[b + F.heading] - wasH);
    this.prevSteer += this.cars[b + F.steer] - wasSteer;
    this.stats.corrections++;
    this.stats.replayTicks = replayed;
  }

  private hardSet(slot: number, tick: number, state: Float32Array) {
    this.scratchView.set(state);
    this.wasm.phys_set(slot, this.scratch);
    this.wasm.phys_reproject(slot);
    this.localTick = tick;
    this.histTick.fill(-1);
    this.record(tick);
    this.offX = this.offY = this.offHeading = 0;
    this.havePrev = false;
  }

  /** Bleed the visual correction away. Call once per rendered frame. */
  decaySmoothing(dt: number) {
    const k = Math.pow(0.001, Math.min(dt, 0.1) / SMOOTH_TIME);
    this.offX *= k;
    this.offY *= k;
    this.offHeading *= k;
    if (Math.abs(this.offX) < 1e-4) this.offX = 0;
    if (Math.abs(this.offY) < 1e-4) this.offY = 0;
    if (Math.abs(this.offHeading) < 1e-5) this.offHeading = 0;
  }

  /** Magnitude of the correction currently being hidden, in meters. */
  get smoothingResidual(): number {
    return Math.hypot(this.offX, this.offY);
  }

  /**
   * Pose to draw for the local car.
   *
   * `alpha` is how far into the current tick this frame falls: the fixed-step
   * accumulator's remainder over the step size. Without it the car only moves
   * on the frames that happen to run a tick, which reads as a stutter against a
   * camera and a track that move every frame -- and at 120 Hz or above, as the
   * car holding still for every second frame. On top of the interpolation goes
   * the residual from the last correction, decaying away.
   */
  localPose(alpha = 1): RenderPose {
    const b = this.localSlot * this.stride;
    const x = this.cars[b + F.x];
    const y = this.cars[b + F.y];
    const heading = this.cars[b + F.heading];
    const steer = this.cars[b + F.steer];
    if (!this.havePrev) {
      return { x: x + this.offX, y: y + this.offY, heading: heading + this.offHeading, steer };
    }
    const t = alpha < 0 ? 0 : alpha > 1 ? 1 : alpha;
    return {
      x: this.prevX + (x - this.prevX) * t + this.offX,
      y: this.prevY + (y - this.prevY) * t + this.offY,
      heading: this.prevHeading + wrapPi(heading - this.prevHeading) * t + this.offHeading,
      steer: this.prevSteer + (steer - this.prevSteer) * t,
    };
  }

  field(slot: number, f: number): number {
    return this.cars[slot * this.stride + f];
  }

  /**
   * Cheat: shove the local car forward with speed physics would never allow.
   * The sidecar never sees it -- inputs are still just throttle and steering --
   * so the next snapshot disagrees and drags the car back.
   */
  cheat(boost: number) {
    if (this.localSlot < 0) return;
    const b = this.localSlot * this.stride;
    const h = this.cars[b + F.heading];
    this.cars[b + F.vx] += Math.cos(h) * boost;
    this.cars[b + F.vy] += Math.sin(h) * boost;
  }
}

function readTrack(wasm: WasmExports, consts: Float32Array): TrackData {
  const n = wasm.phys_track_samples();
  const stride = wasm.phys_track_stride();
  const raw = new Float32Array(wasm.memory.buffer, wasm.phys_track_ptr(), n * stride);
  const points = new Float32Array(n * 2);
  const tangents = new Float32Array(n * 2);
  const halfWidth = new Float32Array(n);
  const curvature = new Float32Array(n);
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (let i = 0; i < n; i++) {
    const b = i * stride;
    const x = raw[b];
    const y = raw[b + 1];
    points[i * 2] = x;
    points[i * 2 + 1] = y;
    tangents[i * 2] = raw[b + 2];
    tangents[i * 2 + 1] = raw[b + 3];
    halfWidth[i] = raw[b + 4];
    curvature[i] = raw[b + 5];
    const w = halfWidth[i] + 2;
    minX = Math.min(minX, x - w);
    minY = Math.min(minY, y - w);
    maxX = Math.max(maxX, x + w);
    maxY = Math.max(maxY, y + w);
  }
  return {
    points,
    tangents,
    halfWidth,
    curvature,
    samples: n,
    length: consts[6],
    ds: consts[7],
    checkpoints: consts[9],
    bounds: { minX, minY, maxX, maxY },
  };
}

export async function loadSim(url: string): Promise<Sim> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`could not load ${url}: ${res.status}`);
  const { instance } = await WebAssembly.instantiate(await res.arrayBuffer(), {});
  return new Sim(instance.exports as unknown as WasmExports);
}
