// SpacetimeDB plumbing: subscriptions in, controller state out.
//
// The client subscribes to four small tables and never sees another player's
// inputs or any simulation internals. Everything it sends is a controller
// reading. That is the whole trust boundary.

import { DbConnection, type EventContext } from './module_bindings';
import type { Car as CarRow, CarState as CarStateRow } from './module_bindings/types';
import { Identity } from 'spacetimedb';
import { F } from './sim';

export interface Snapshot {
  tick: number;
  /** Full 24-float CarState record, ready to hand straight to the wasm sim. */
  state: Float32Array;
  ackSeq: number;
  /** performance.now() when this landed, for latency estimation. */
  received: number;
}

export interface CarMeta {
  carId: number;
  slot: number;
  name: string;
  color: number;
  isBot: boolean;
  mine: boolean;
}

/** One row of the module's best-lap table, with client-side colour attached. */
export interface LapRecordRow {
  name: string;
  bestLap: number;
  isBot: boolean;
  /** null once the car that set the time has left the race. */
  color: number | null;
  mine: boolean;
}

/** Artificial network conditions, applied to outgoing input only. */
export interface NetSim {
  latencyMs: number;
  jitterMs: number;
  lossPct: number;
}

const SNAPSHOT_BUFFER = 40;

export class Net {
  conn!: DbConnection;
  identity: Identity | null = null;
  connected = false;
  subscribed = false;

  /** car_id -> metadata. */
  cars = new Map<number, CarMeta>();
  /** car_id -> recent authoritative poses, oldest first. */
  buffers = new Map<number, Snapshot[]>();
  myCarId = 0;
  mySlot = -1;

  serverTick = 0;
  /** Freshest tick seen from any car, with the local time it landed. Together
   *  they give a running estimate of where the authority's clock is now. */
  lastSnapTick = 0;
  lastSnapAt = 0;
  /** Low-passed `tick - performance.now()`, in ticks: the clock estimate's
   *  free-running base. Re-deriving it from whichever snapshot landed last
   *  imports that snapshot's arrival jitter into everything downstream. */
  private clockOffset = 0;
  private clockLocked = false;
  sidecarOnline = false;
  tickHz = 60;
  snapshotHz = 20;

  // --- measured link quality ---
  rttMs = 0;
  private sentAt = new Map<number, number>();
  private seq = 1;
  snapshotsPerSec = 0;
  private snapCount = 0;
  private snapWindow = performance.now();

  netSim: NetSim = { latencyMs: 0, jitterMs: 0, lossPct: 0 };
  droppedInputs = 0;

  onLocalSnapshot: ((s: Snapshot) => void) | null = null;
  onCarsChanged: (() => void) | null = null;
  onStatus: ((msg: string) => void) | null = null;

  async connect(uri: string, dbName: string): Promise<void> {
    const tokenKey = `stdb-sidecar-token:${dbName}`;
    const saved = localStorage.getItem(tokenKey) ?? undefined;

    await new Promise<void>((resolve, reject) => {
      const builder = DbConnection.builder()
        .withUri(uri)
        .withDatabaseName(dbName)
        .withToken(saved)
        .onConnect((conn, identity, token) => {
          this.conn = conn;
          this.identity = identity;
          this.connected = true;
          try {
            localStorage.setItem(tokenKey, token);
          } catch {
            /* private browsing; a fresh identity per session is fine */
          }
          this.subscribe(resolve);
        })
        .onConnectError((_ctx, err) => reject(err))
        .onDisconnect((_ctx, err) => {
          this.connected = false;
          this.onStatus?.(err ? `disconnected: ${err.message}` : 'disconnected');
        });
      this.conn = builder.build();
    });
  }

  private subscribe(ready: () => void) {
    const db = this.conn.db;

    db.car.onInsert((_ctx, row) => this.upsertCar(row));
    db.car.onUpdate((_ctx, _old, row) => this.upsertCar(row));
    db.car.onDelete((_ctx, row) => {
      this.cars.delete(row.carId);
      this.buffers.delete(row.carId);
      if (row.carId === this.myCarId) {
        this.myCarId = 0;
        this.mySlot = -1;
      }
      this.onCarsChanged?.();
    });

    db.carState.onInsert((_ctx, row) => this.onState(row));
    db.carState.onUpdate((_ctx, _old, row) => this.onState(row));

    db.config.onInsert((_ctx, row) => this.onConfig(row));
    db.config.onUpdate((_ctx, _old, row) => this.onConfig(row));

    db.player.onUpdate((_ctx: EventContext, _old, row) => {
      if (this.identity && row.identity.isEqual(this.identity)) {
        this.myCarId = row.carId;
        const meta = this.cars.get(row.carId);
        this.mySlot = meta ? meta.slot : -1;
        this.onCarsChanged?.();
      }
    });

    this.conn
      .subscriptionBuilder()
      .onApplied(() => {
        this.subscribed = true;
        for (const c of this.conn.db.car.iter()) this.upsertCar(c);
        for (const s of this.conn.db.carState.iter()) this.onState(s);
        for (const cfg of this.conn.db.config.iter()) this.onConfig(cfg);
        ready();
      })
      .onError(() => this.onStatus?.('subscription failed'))
      .subscribe([
        'SELECT * FROM config',
        'SELECT * FROM car',
        'SELECT * FROM car_state',
        'SELECT * FROM player',
        'SELECT * FROM lap_record',
      ]);
  }

  private onConfig(row: { sidecarOnline: boolean; serverTick: bigint; tickHz: number; snapshotHz: number }) {
    this.sidecarOnline = row.sidecarOnline;
    this.serverTick = Number(row.serverTick);
    this.tickHz = row.tickHz;
    this.snapshotHz = row.snapshotHz;
  }

  private upsertCar(row: CarRow) {
    const mine = !!(this.identity && row.owner && row.owner.isEqual(this.identity));
    this.cars.set(row.carId, {
      carId: row.carId,
      slot: row.slot,
      name: row.name,
      color: row.color,
      isBot: row.isBot,
      mine,
    });
    if (mine) {
      this.myCarId = row.carId;
      this.mySlot = row.slot;
    }
    this.onCarsChanged?.();
  }

  /**
   * Turn a row into the exact 24-float record the wasm simulation uses.
   *
   * Every field the integrator reads back is on the wire; the two that are not
   * (`slip_f`/`slip_r`) are overwritten before they are ever read, so
   * reconstructing them as zero is exact rather than approximate.
   */
  private toRecord(row: CarStateRow): Float32Array {
    const s = new Float32Array(24);
    s[F.x] = row.x;
    s[F.y] = row.y;
    s[F.heading] = row.heading;
    s[F.vx] = row.vx;
    s[F.vy] = row.vy;
    s[F.omega] = row.omega;
    s[F.steer] = row.steer;
    s[F.ax] = row.ax;
    s[F.slipF] = 0;
    s[F.slipR] = 0;
    s[F.wheelSpin] = row.wheelSpin;
    s[F.rpm] = row.rpm;
    s[F.gear] = row.gear;
    s[F.s] = row.s;
    s[F.lat] = row.lat;
    s[F.seg] = row.seg;
    s[F.lap] = row.lap;
    s[F.cp] = row.cp;
    s[F.lapStart] = row.lapStart;
    s[F.lastLap] = row.lastLap;
    s[F.bestLap] = row.bestLap;
    s[F.impact] = row.impact;
    s[F.wall] = row.wall ? 1 : 0;
    s[F.active] = 1;
    return s;
  }

  private onState(row: CarStateRow) {
    const tick = Number(row.tick);
    if (tick === 0) return; // placeholder inserted at join

    const snap: Snapshot = {
      tick,
      state: this.toRecord(row),
      ackSeq: row.ackSeq,
      received: performance.now(),
    };

    let buf = this.buffers.get(row.carId);
    if (!buf) {
      buf = [];
      this.buffers.set(row.carId, buf);
    }
    if (buf.length && tick <= buf[buf.length - 1].tick) return; // stale
    buf.push(snap);
    if (tick > this.lastSnapTick) {
      this.lastSnapTick = tick;
      this.lastSnapAt = snap.received;
      this.syncClock(tick, snap.received);
    }
    while (buf.length > SNAPSHOT_BUFFER) buf.shift();

    if (row.carId === this.myCarId) {
      const sent = this.sentAt.get(row.ackSeq);
      if (sent !== undefined) {
        const rtt = snap.received - sent;
        // Exponential average; the raw figure is noisy at 20 Hz.
        this.rttMs = this.rttMs === 0 ? rtt : this.rttMs * 0.85 + rtt * 0.15;
        for (const k of this.sentAt.keys()) if (k <= row.ackSeq) this.sentAt.delete(k);
      }
      this.onLocalSnapshot?.(snap);
    }

    this.snapCount++;
    const now = performance.now();
    if (now - this.snapWindow > 1000) {
      const cars = Math.max(1, this.cars.size);
      this.snapshotsPerSec = (this.snapCount / cars / (now - this.snapWindow)) * 1000;
      this.snapCount = 0;
      this.snapWindow = now;
    }
  }

  // ------------------------------------------------------------- commands --

  async join(name: string, color: number) {
    await this.conn.reducers.joinRace({ name, color });
  }

  async leave() {
    await this.conn.reducers.leaveRace({});
  }

  async respawn() {
    await this.conn.reducers.requestRespawn({});
  }

  /**
   * Send controller state. Optionally delayed, jittered or dropped first, so the
   * reconciliation path can be exercised without a lossy network to hand.
   */
  sendInput(tick: number, throttle: number, steer: number, brake: number, handbrake: boolean) {
    if (!this.connected || this.myCarId === 0) return;
    const seq = this.seq++;
    this.sentAt.set(seq, performance.now());
    if (this.sentAt.size > 512) {
      const oldest = this.sentAt.keys().next().value;
      if (oldest !== undefined) this.sentAt.delete(oldest);
    }

    const fire = () => {
      this.conn.reducers
        .setInput({ seq, tick: BigInt(tick), throttle, steer, brake, handbrake })
        .catch(() => {});
    };

    if (this.netSim.lossPct > 0 && Math.random() * 100 < this.netSim.lossPct) {
      this.droppedInputs++;
      return;
    }
    const delay = this.netSim.latencyMs + (Math.random() - 0.5) * 2 * this.netSim.jitterMs;
    if (delay > 0.5) setTimeout(fire, delay);
    else fire();
  }

  /**
   * Fold one snapshot arrival into the clock estimate.
   *
   * `tick - arrival` is a constant for as long as the link delay is, whatever
   * the snapshot rate, so the spread in it is precisely the arrival jitter. A
   * low pass on that is a clock that advances at a steady 60 Hz rather than one
   * yanked twenty times a second -- and every remote car on screen is drawn
   * relative to it.
   */
  private syncClock(tick: number, at: number) {
    const sample = tick - (at / 1000) * 60;
    if (!this.clockLocked) {
      this.clockOffset = sample;
      this.clockLocked = true;
      return;
    }
    const d = sample - this.clockOffset;
    // Half a second of disagreement is a restarted sidecar or a tab that was
    // asleep, not jitter. Nothing to ease towards; take it whole.
    if (Math.abs(d) > 30) this.clockOffset = sample;
    else this.clockOffset += d * 0.1;
  }

  /**
   * Where the authority's clock is right now, in ticks. Deliberately ignores
   * one-way latency: this is the newest tick whose data could already be here,
   * which is exactly the reference the interpolator wants. Continuous in `now`,
   * so sampling it twice in a frame -- or on consecutive frames -- can never
   * step backwards.
   */
  estimatedServerTick(now: number): number {
    if (!this.clockLocked) return this.serverTick;
    return this.clockOffset + (now / 1000) * 60;
  }

  /** Interpolated pose for a remote car at a fractional server tick. */
  sampleRemote(carId: number, atTick: number): Float32Array | null {
    const buf = this.buffers.get(carId);
    if (!buf || buf.length === 0) return null;
    if (buf.length === 1 || atTick <= buf[0].tick) return buf[0].state;

    const last = buf[buf.length - 1];
    if (atTick >= last.tick) {
      // Ran out of buffer: dead reckon briefly rather than freeze.
      const ahead = Math.min((atTick - last.tick) / 60, 0.25);
      const out = new Float32Array(last.state);
      out[F.x] += out[F.vx] * ahead;
      out[F.y] += out[F.vy] * ahead;
      out[F.heading] += out[F.omega] * ahead;
      return out;
    }

    for (let i = buf.length - 1; i > 0; i--) {
      const b = buf[i];
      const a = buf[i - 1];
      if (atTick >= a.tick && atTick <= b.tick) {
        const t = (atTick - a.tick) / Math.max(1, b.tick - a.tick);
        return lerpState(a.state, b.state, t);
      }
    }
    return buf[0].state;
  }

  /** Best-lap table, quickest first, joined with whoever is on track now.
   *  The module keeps the times; colours and "that's me" only exist client-side,
   *  and a name can outlive its car, so both are optional. */
  records(): LapRecordRow[] {
    if (!this.conn?.db) return [];
    const live = new Map([...this.cars.values()].map((c) => [c.name, c]));
    return [...this.conn.db.lapRecord.iter()]
      .map((r) => {
        const car = live.get(r.name);
        return {
          name: r.name,
          bestLap: r.bestLap,
          isBot: r.isBot,
          color: car?.color ?? null,
          mine: car?.mine ?? false,
        };
      })
      .sort((a, b) => a.bestLap - b.bestLap);
  }
}

function shortAngle(a: number, b: number): number {
  let d = b - a;
  while (d > Math.PI) d -= Math.PI * 2;
  while (d < -Math.PI) d += Math.PI * 2;
  return d;
}

function lerpState(a: Float32Array, b: Float32Array, t: number): Float32Array {
  const out = new Float32Array(b);
  for (const f of [F.x, F.y, F.vx, F.vy, F.omega, F.steer, F.wheelSpin, F.rpm, F.s, F.lat]) {
    out[f] = a[f] + (b[f] - a[f]) * t;
  }
  out[F.heading] = a[F.heading] + shortAngle(a[F.heading], b[F.heading]) * t;
  return out;
}
