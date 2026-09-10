// Client-side simulation: the same Rust physics as the sidecar, compiled to
// wasm, plus the netcode that keeps it honest.
//
// The loop is the classic arrangement, in four parts:
//
//   1. PREDICT   every tick, step the whole grid immediately: the local car
//                from local input, so steering feels instant regardless of
//                ping, and every rival from the controls the authority
//                published with its last snapshot, held.
//   2. RECONCILE when an authoritative snapshot arrives for tick T, rewind
//                every car to what the authority had at T and replay to now --
//                the local car on its recorded inputs, the rivals on their
//                held ones. The contact that gets replayed is then the contact
//                the authority resolved.
//   3. SMOOTH    a rewind moves the cars. Rather than teleport, keep the visual
//                error as an offset and decay it to zero over ~200 ms.
//   4. INTERPOLATE the simulation moves in 60 Hz jumps and the display does not.
//                Draw the fraction of a tick the frame actually falls on, or
//                the cars stutter against a camera that moves every frame.
//
// The two halves of step 2 are not the same claim and should not be read as
// one. The local car's replay is a *replay*: the wasm and the sidecar run
// bit-identical code (see scripts/verify-determinism.mjs) on identical inputs,
// so it lands on identical bits and step 3 has nothing to do. A rival's is a
// *prediction*: the client knows what the other driver was doing at the last
// snapshot and not what they are doing now, so it will be a few centimetres
// out and the next snapshot will say so. `stats.error` is the first; it is the
// number the demo is about, and it stays pinned at zero on a healthy link.
// `stats.rivalError` is the second, and never will be.

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

/**
 * Ticks a rival is carried on controls the authority has stopped confirming.
 *
 * Three-tenths of a second, against a snapshot that should arrive every three
 * ticks, so in normal running this never fires -- it is the guard for a stream
 * that has stopped rather than a limit on the prediction. There is no accuracy
 * argument for letting go sooner: `physics/examples/predict.rs` scores a held
 * input as the best of its four schemes at every lead it measures, 400 ms
 * included. There is a different argument for letting go eventually. A rival
 * whose news has stopped arriving and whose throttle is still buried drives
 * itself into a barrier and stays there, which is a worse picture than one that
 * lifts off and coasts to a halt roughly where it was last seen.
 */
const HOLD_TICKS = 18;

/** A rival correction bigger than this is a teleport -- a respawn, or a slot
 *  changing hands -- and gets shown as one instead of slid into place. */
const RIVAL_SNAP = 4;

/**
 * Field offsets inside a CarState record. Mirrors `physics::car::CarState`
 * field for field, in declaration order -- the wasm module hands this over as
 * raw memory, so an offset that is wrong here reads a neighbouring quantity
 * rather than failing.
 *
 * Most of it is the vehicle's internal state rather than its pose: four wheel
 * speeds, four tires part way through building up their cornering force, the
 * body's roll and pitch, and the drivetrain. Rollback replays from all of it.
 */
export const F = {
  x: 0, y: 1, heading: 2, vx: 3, vy: 4, omega: 5, steer: 6,
  wFl: 7, wFr: 8, wRl: 9, wRr: 10,
  fyFl: 11, fyFr: 12, fyRl: 13, fyRr: 14,
  roll: 15, rollRate: 16, pitch: 17, pitchRate: 18,
  engine: 19, gear: 20, shift: 21, clutch: 22,
  ax: 23, ay: 24,
  slipF: 25, slipR: 26, wheelSpin: 27, rpm: 28,
  s: 29, lat: 30, seg: 31, lap: 32, cp: 33, lapStart: 34, lastLap: 35,
  bestLap: 36, impact: 37, wall: 38,
  dmgFront: 39, dmgRear: 40, dmgLeft: 41, dmgRight: 42,
  active: 43,
  pedal: 44, brakePedal: 45,
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
  /** Rolling radius, meters. Needed to turn a road speed into a wheel speed. */
  wheelRadius: number;
  /** Road speed below which the gearbox will change direction, m/s. */
  reverseBelow: number;
  /** Crush at which a panel has nothing left to give, metres. */
  maxCrush: number;
}

export type Input = { throttle: number; steer: number; brake: number; handbrake: number };

/**
 * One car this client does not own, as the authority last described it.
 *
 * Both halves are needed and neither is enough. `state` is where the car was
 * and what it was doing; `input` is what its driver had their hands on at the
 * time, which is the one thing the client cannot work out for itself and the
 * thing that decides where the car goes next.
 */
export interface RivalState {
  slot: number;
  /** Full CarState record, in `F` order. */
  state: Float32Array;
  /** Four floats: throttle, steer, brake, handbrake. */
  input: Float32Array;
}

/** What actually gets drawn: prediction plus the residual being smoothed away. */
export interface RenderPose {
  x: number;
  y: number;
  heading: number;
  steer: number;
}

export interface NetStats {
  /** Distance between prediction and authority at the last snapshot, meters.
   *  The local car's, and a replay rather than a guess: zero on a healthy
   *  link, and a real fault when it is not. */
  error: number;
  /** Rolling peak, for the telemetry graph. */
  errorPeak: number;
  /** Worst rival mispredicted by the last snapshot, meters. A guess rather
   *  than a replay, so never zero -- see the note at the top of this file. */
  rivalError: number;
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
  /** Slots holding a car this world simulates: the local one, plus every rival
   *  a snapshot has been seeded from. Mirrors the wasm world's own mask. */
  activeMask = 0;

  // --- rollback bookkeeping ---
  private histTick = new Int32Array(HISTORY).fill(-1);
  private histInput: Float32Array;
  private histState: Float32Array;
  /** Pointer + view into the wasm-side staging buffer used for rollback. */
  private scratch: number;
  private scratchView!: Float32Array;
  private lastApplied = -1;
  /** Tick each rival's held controls are released at. See [`HOLD_TICKS`]. */
  private holdUntil: Int32Array;

  // --- per-car render bookkeeping ---
  // Two things the renderer needs that the physics does not keep, both per
  // slot rather than per client: every car on the grid is stepped now, so
  // every car needs them.
  //
  // `prev` is the pose one tick back. Frames do not land on tick boundaries, so
  // the renderer draws somewhere between it and the current pose rather than
  // holding the current one until the next step lands. Rivals used to get this
  // for free by being sampled at a fractional tick; simulated, they move in
  // 60 Hz jumps like anything else.
  //
  // `off` is the correction a rewind revealed, kept and decayed rather than
  // shown. The simulation is corrected at once; the picture catches up.
  /** x, y, heading, steer at `localTick - 1`, four floats per slot. */
  private prev: Float32Array;
  /** Which slots have one. A car that has just arrived does not. */
  private havePrev = 0;
  /** x, y, heading still being smoothed away, three floats per slot. */
  private off: Float32Array;
  /** Scratch: each car's pose immediately before a rewind, so [`absorb`] can
   *  work out what the rewind moved. */
  private before: Float32Array;
  /** x, y, heading of every simulated car at each recent tick. Only the local
   *  car needs to be *replayed* from its past, but every rival needs to be
   *  scored against it: this is what the next snapshot judges the guess by. */
  private histPose: Float32Array;

  stats: NetStats = {
    error: 0,
    errorPeak: 0,
    rivalError: 0,
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
    this.consts = {
      halfLen: c[0],
      halfWid: c[1],
      maxCars,
      carFloats: this.stride,
      wheelRadius: c[12],
      reverseBelow: c[13],
      maxCrush: c[14],
    };

    this.track = readTrack(wasm, c);

    this.histInput = new Float32Array(HISTORY * 4);
    this.histState = new Float32Array(HISTORY * this.stride);
    this.scratch = wasm.phys_scratch_ptr();
    this.scratchView = new Float32Array(wasm.memory.buffer, this.scratch, this.stride);
    this.prev = new Float32Array(maxCars * 4);
    this.off = new Float32Array(maxCars * 3);
    this.before = new Float32Array(maxCars * 4);
    this.histPose = new Float32Array(HISTORY * maxCars * 3);
    this.holdUntil = new Int32Array(maxCars);
  }

  setLocalSlot(slot: number) {
    if (this.localSlot === slot) return;
    this.localSlot = slot;
    // Taking a seat starts an empty grid holding one car -- this one, which is
    // put right by the first snapshot. The rivals join it as `reconcile` hears
    // from them, because there is nothing to seed a slot from until then and a
    // slot nobody has written still holds whoever sat in it last.
    //
    // Losing a seat empties it again. Nothing steps without a local car, so a
    // slot left in the mask would be a rival frozen at whatever tick the seat
    // was lost on; a spectator draws the field from the network buffer
    // instead, and `main.ts` decides which of the two it is looking at by
    // asking [`simulates`].
    this.setActive(slot >= 0 ? 1 << slot : 0);
    this.histTick.fill(-1);
    this.lastApplied = -1;
    this.havePrev = 0;
    this.off.fill(0);
  }

  private setActive(mask: number) {
    this.activeMask = mask;
    this.wasm.phys_set_active(mask);
  }

  /** Drop every slot no longer held by a car. Cars leave at once; they only
   *  arrive with a snapshot in hand. */
  retainActive(live: number) {
    this.setActive(this.activeMask & live);
  }

  /** Is this slot one this client is simulating? */
  simulates(slot: number): boolean {
    return slot >= 0 && (this.activeMask & (1 << slot)) !== 0;
  }

  /** Forget what was being drawn for a slot: no pose to interpolate from, and
   *  no correction to smooth away. For a slot that has just changed hands. */
  private forget(slot: number) {
    this.havePrev &= ~(1 << slot);
    this.off[slot * 3] = this.off[slot * 3 + 1] = this.off[slot * 3 + 2] = 0;
  }

  /**
   * Advance the whole grid one tick: the local car under `input`, recorded for
   * replay, and every rival on the controls its last snapshot came with.
   *
   * The rivals used to be parked here as immovable colliders at an extrapolated
   * pose, which is a car that corners without a steering wheel and brakes
   * without a brake pedal -- and it is precisely mid-corner and under braking
   * that a client most needs to know where a rival is about to be. Stepping
   * them costs the grid instead of one car; what it buys is in `predict.rs`.
   */
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

    this.release(this.localTick);
    this.markPrev();
    this.wasm.phys_set_tick(this.localTick);
    this.wasm.phys_step(this.activeMask);
    this.localTick++;
    this.record(this.localTick);
  }

  /** Take every simulated car's current pose as the one to interpolate away
   *  from on the frames between this tick and the next. */
  private markPrev() {
    for (let slot = 0; slot < this.consts.maxCars; slot++) {
      if (!(this.activeMask & (1 << slot))) continue;
      const b = slot * this.stride;
      const p = slot * 4;
      this.prev[p] = this.cars[b + F.x];
      this.prev[p + 1] = this.cars[b + F.y];
      this.prev[p + 2] = this.cars[b + F.heading];
      this.prev[p + 3] = this.cars[b + F.steer];
      this.havePrev |= 1 << slot;
    }
  }

  /** Let go of the controls of any rival whose news has gone stale. */
  private release(tick: number) {
    for (let slot = 0; slot < this.consts.maxCars; slot++) {
      if (slot === this.localSlot || !(this.activeMask & (1 << slot))) continue;
      if (tick < this.holdUntil[slot]) continue;
      const i = slot * 4;
      this.inputs[i] = this.inputs[i + 1] = this.inputs[i + 2] = this.inputs[i + 3] = 0;
    }
  }

  /**
   * File what was simulated for `tick`: the local car in full, because it may
   * have to be replayed from, and every other car's pose, because the next
   * snapshot is going to say how good a guess it was.
   */
  private record(tick: number) {
    const h = tick % HISTORY;
    const b = this.localSlot * this.stride;
    this.histTick[h] = tick;
    this.histState.set(this.cars.subarray(b, b + this.stride), h * this.stride);
    const base = h * this.consts.maxCars * 3;
    for (let slot = 0; slot < this.consts.maxCars; slot++) {
      if (!(this.activeMask & (1 << slot))) continue;
      const c = slot * this.stride;
      const p = base + slot * 3;
      this.histPose[p] = this.cars[c + F.x];
      this.histPose[p + 1] = this.cars[c + F.y];
      this.histPose[p + 2] = this.cars[c + F.heading];
    }
  }

  /**
   * Reconcile the whole grid against the authority.
   *
   * `state` is a full CarState record as the sidecar had it at `tick`, and
   * `rivals` is the same for every other car on the grid, each with the
   * controls its driver had their hands on at the time.
   *
   * The two halves are reconciled differently because they mean different
   * things. The local car is *rewound and replayed*: this client owns it, has
   * every input it gave it since `tick`, and has to arrive back at its own
   * answer. A rival is simply *overwritten* -- no blending, no smoothing of
   * the state itself. The client has no stake in a car it does not own, the
   * snapshot is not an opinion, and the residual it reveals is an apology for
   * a guess that has now been corrected: feeding that back into the next
   * contact would re-introduce exactly the error it was hiding. It goes into
   * the picture ([`off`]) and nowhere near the physics.
   */
  reconcile(tick: number, state: Float32Array, rivals: RivalState[]) {
    if (this.localSlot < 0) return;
    if (tick <= this.lastApplied) return;
    this.lastApplied = tick;
    this.stats.lastSnapshotTick = tick;

    const slot = this.localSlot;
    const h = tick % HISTORY;
    const known = this.histTick[h] === tick && tick <= this.localTick;

    // How far out the last round of guesses turned out to be, scored before
    // any of them is overwritten.
    this.stats.rivalError = this.measure(tick, rivals);
    // And where every car was about to be drawn, so what follows can be kept
    // as a fading offset rather than shown as a jump.
    const drawn = this.markBefore();

    // Adopt the authority's word on every rival, and admit any car this is the
    // first news of -- a slot nobody has written still holds whoever sat in it
    // last, and stepping that would be simulating a ghost.
    let mask = 1 << slot;
    for (const r of rivals) {
      if (!(this.activeMask & (1 << r.slot))) this.forget(r.slot);
      this.scratchView.set(r.state);
      this.wasm.phys_set(r.slot, this.scratch);
      this.inputs.set(r.input, r.slot * 4);
      this.holdUntil[r.slot] = tick + HOLD_TICKS;
      mask |= 1 << r.slot;
    }
    this.setActive(mask);

    if (!known) {
      // No history for that tick: either we just joined, or we fell so far
      // behind that the ring buffer wrapped. Accept the server wholesale. The
      // rivals already have, and the clock goes back to meet them.
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

    if (err > SNAP_DISTANCE) {
      this.hardSet(slot, tick, state);
      this.stats.resyncs++;
      return;
    }

    // Rewind and replay -- even when the local car was predicted exactly.
    // There is no fast path out of here any more: the rivals have just been
    // put back to where the authority had them at `tick` and the world is
    // still due at `localTick`, so somebody has to walk them there. The local
    // car comes along for free, and adopting the authoritative record for it
    // keeps tiny differences in fields that are not compared from
    // accumulating.
    this.scratchView.set(state);
    this.wasm.phys_set(slot, this.scratch);
    this.histState.set(state, h * this.stride);

    let replayed = 0;
    for (let t = tick; t < this.localTick; t++) {
      const j = t % HISTORY;
      if (this.histTick[j] !== t) break;
      this.inputs.set(this.histInput.subarray(j * 4, j * 4 + 4), slot * 4);
      this.release(t);
      this.wasm.phys_set_tick(t);
      this.wasm.phys_step(mask);
      this.record(t + 1);
      replayed++;
    }

    this.absorb(drawn & mask);
    // A correction is a disagreement about the local car, which is the number
    // the demo is about. The replay above now happens twenty times a second
    // whatever the local car did, and counting those as corrections would turn
    // a headline reading into a constant.
    if (err >= EPSILON_POS || Math.abs(dh) >= EPSILON_ANG) this.stats.corrections++;
    this.stats.replayTicks = replayed;
  }

  /**
   * How far out the guesses about the other cars turned out to be, in metres:
   * the worst of them.
   *
   * Not an error to be corrected -- the snapshot *is* the correction -- but the
   * number that says whether carrying a rival forward through the physics is
   * working, and the one to watch instead of [`stats.error`] when reading a
   * rival's behaviour. Zero would mean the client had guessed what another
   * driver was about to do, which it cannot.
   */
  private measure(tick: number, rivals: RivalState[]): number {
    const h = tick % HISTORY;
    if (this.histTick[h] !== tick) return 0;
    const base = h * this.consts.maxCars * 3;
    let worst = 0;
    for (const r of rivals) {
      // A car this client had not yet heard of made no guess to score.
      if (!(this.activeMask & (1 << r.slot))) continue;
      const p = base + r.slot * 3;
      const d = Math.hypot(r.state[F.x] - this.histPose[p], r.state[F.y] - this.histPose[p + 1]);
      if (d > worst) worst = d;
    }
    return worst;
  }

  /** Save every simulated car's pose, and return the slots saved. */
  private markBefore(): number {
    for (let slot = 0; slot < this.consts.maxCars; slot++) {
      if (!(this.activeMask & (1 << slot))) continue;
      const b = slot * this.stride;
      const p = slot * 4;
      this.before[p] = this.cars[b + F.x];
      this.before[p + 1] = this.cars[b + F.y];
      this.before[p + 2] = this.cars[b + F.heading];
      this.before[p + 3] = this.cars[b + F.steer];
    }
    return this.activeMask;
  }

  /**
   * Keep the correction a rewind revealed instead of showing it.
   *
   * The offset is where the car appeared to be minus where it now is, decaying
   * to nothing over [`SMOOTH_TIME`]; whatever was still fading is folded in, so
   * a run of small corrections does not restart the fade each time. The
   * previous-tick pose is carried the same distance, so what the renderer
   * interpolates across is still one tick of motion rather than one tick plus
   * the whole correction.
   */
  private absorb(slots: number) {
    for (let slot = 0; slot < this.consts.maxCars; slot++) {
      if (!(slots & (1 << slot))) continue;
      const b = slot * this.stride;
      const p = slot * 4;
      const o = slot * 3;
      const x = this.cars[b + F.x];
      const y = this.cars[b + F.y];
      const heading = this.cars[b + F.heading];
      const steer = this.cars[b + F.steer];
      this.off[o] += this.before[p] - x;
      this.off[o + 1] += this.before[p + 1] - y;
      this.off[o + 2] = wrapPi(this.off[o + 2] + this.before[p + 2] - heading);
      this.prev[p] += x - this.before[p];
      this.prev[p + 1] += y - this.before[p + 1];
      this.prev[p + 2] += wrapPi(heading - this.before[p + 2]);
      this.prev[p + 3] += steer - this.before[p + 3];
      // A rival that moved this far did not mispredict, it teleported: a
      // respawn, or a slot changing hands. Show it rather than sliding the car
      // across four metres of track over a fifth of a second. The local car
      // reaches the same conclusion through [`SNAP_DISTANCE`] and a resync.
      if (slot !== this.localSlot && Math.hypot(this.off[o], this.off[o + 1]) > RIVAL_SNAP) {
        this.off[o] = this.off[o + 1] = this.off[o + 2] = 0;
      }
    }
  }

  private hardSet(slot: number, tick: number, state: Float32Array) {
    this.scratchView.set(state);
    this.wasm.phys_set(slot, this.scratch);
    this.wasm.phys_reproject(slot);
    this.localTick = tick;
    this.histTick.fill(-1);
    this.record(tick);
    // A hard set is a discontinuity for the whole picture and not just for the
    // local car: the clock has moved, and every rival has just been put back to
    // where the authority had it at `tick`. Nothing on screen has a previous
    // pose worth interpolating from or a correction worth hiding.
    this.havePrev = 0;
    this.off.fill(0);
  }

  /** Bleed the visual corrections away. Call once per rendered frame. */
  decaySmoothing(dt: number) {
    const k = Math.pow(0.001, Math.min(dt, 0.1) / SMOOTH_TIME);
    for (let i = 0; i < this.off.length; i++) {
      const v = this.off[i] * k;
      // A tenth of a millimetre, or six thousandths of a degree. Below either
      // the offset is not being hidden any more, it is just still there.
      this.off[i] = Math.abs(v) < 1e-4 ? 0 : v;
    }
  }

  /** Magnitude of the correction currently being hidden on the local car, in
   *  meters. */
  get smoothingResidual(): number {
    if (this.localSlot < 0) return 0;
    const o = this.localSlot * 3;
    return Math.hypot(this.off[o], this.off[o + 1]);
  }

  /**
   * Pose to draw for a simulated car.
   *
   * `alpha` is how far into the current tick this frame falls: the fixed-step
   * accumulator's remainder over the step size. Without it a car only moves on
   * the frames that happen to run a tick, which reads as a stutter against a
   * camera and a track that move every frame -- and at 120 Hz or above, as the
   * car holding still for every second frame. On top of the interpolation goes
   * the residual from the last correction, decaying away.
   *
   * The local car used to be the only one that needed this, rivals being drawn
   * at whatever fractional tick was asked for. Now that every car on the grid
   * is stepped, every car needs it.
   */
  pose(slot: number, alpha = 1): RenderPose {
    const b = slot * this.stride;
    const p = slot * 4;
    const o = slot * 3;
    const x = this.cars[b + F.x];
    const y = this.cars[b + F.y];
    const heading = this.cars[b + F.heading];
    const steer = this.cars[b + F.steer];
    if (!(this.havePrev & (1 << slot))) {
      return {
        x: x + this.off[o],
        y: y + this.off[o + 1],
        heading: heading + this.off[o + 2],
        steer,
      };
    }
    const t = alpha < 0 ? 0 : alpha > 1 ? 1 : alpha;
    return {
      x: this.prev[p] + (x - this.prev[p]) * t + this.off[o],
      y: this.prev[p + 1] + (y - this.prev[p + 1]) * t + this.off[o + 1],
      heading: this.prev[p + 2] + wrapPi(heading - this.prev[p + 2]) * t + this.off[o + 2],
      steer: this.prev[p + 3] + (steer - this.prev[p + 3]) * t,
    };
  }

  /** Pose to draw for the local car. */
  localPose(alpha = 1): RenderPose {
    return this.pose(this.localSlot, alpha);
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
    // Spin the wheels up to match. A body suddenly doing 26 m/s more than its
    // tires are turning is not a cheating client, it is a car with all four
    // wheels locked: the tire model reads the mismatch as a large negative slip
    // ratio, and a saturated contact patch has nothing left for cornering. The
    // car would bleed the invented speed back and understeer into the barrier
    // rather than reach the authority for it to disagree with. A client lying
    // about where it is lies consistently.
    const dw = boost / this.consts.wheelRadius;
    this.cars[b + F.wFl] += dw;
    this.cars[b + F.wFr] += dw;
    this.cars[b + F.wRl] += dw;
    this.cars[b + F.wRr] += dw;
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
