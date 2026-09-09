// SpacetimeDB plumbing: subscriptions in, controller state out.
//
// The client subscribes to four small tables and never sees another player's
// inputs or any simulation internals. Everything it sends is a controller
// reading. That is the whole trust boundary.

import { DbConnection, type EventContext } from './module_bindings';
import type {
  Car as CarRow,
  CarState as CarStateRow,
  Config as ConfigRow,
} from './module_bindings/types';
import { Identity } from 'spacetimedb';
import { F, wrapPi } from './sim';

export interface Snapshot {
  tick: number;
  /** Full 24-float CarState record, ready to hand straight to the wasm sim. */
  state: Float32Array;
  ackSeq: number;
  /** performance.now() when this landed, for latency estimation. */
  received: number;
}

/** Everything known about one car this client does not simulate. */
interface Remote {
  /** Authoritative poses, oldest first. */
  snaps: Snapshot[];
  /** Where the last snapshot said this car was, minus where this client had
   *  guessed: carried as a fading offset so a correction is never a jump. */
  offX: number;
  offY: number;
  offH: number;
  /** `performance.now()` the offset was set, which is when it starts fading. */
  offAt: number;
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

/** Longest a remote car is carried forward from its newest snapshot, seconds.
 *  Past this the guess is worse than the lag it is hiding. */
const MAX_LEAD = 0.3;
/** Time constant for fading out a remote car's prediction error. */
const RESIDUAL_FADE = 0.1;
/** A correction bigger than this is a teleport -- a respawn, a resync -- and
 *  gets shown as one instead of slid into place over a tenth of a second. */
const RESIDUAL_SNAP = 4;

/** Reconnect backoff: first retry, and the ceiling it doubles up to. */
const RECONNECT_BASE_MS = 700;
const RECONNECT_MAX_MS = 8000;
/** How often the connection is checked for the failure modes in `watch`. */
const WATCHDOG_MS = 2000;
/**
 * Silence from an authority that still claims to be running for longer than
 * this is a dead socket, not a quiet moment. The sidecar publishes at 20 Hz,
 * and had it gone away the module would have said so on this same connection.
 */
const STALL_MS = 5000;
/** A dial that has neither connected nor failed by now has hung. */
const DIAL_TIMEOUT_MS = 15000;

/**
 * Did this dial fail because the database refused the token we presented?
 *
 * Worth telling apart from an unreachable server, because retrying is useless:
 * every attempt would present the same dead credential and be refused
 * identically. Tokens are signed with a key the server holds, and a client has
 * no way to know it has been rotated -- which happens whenever the deployment's
 * volume is replaced, or the database it is talking to moves somewhere new.
 * Everyone who has been here before then arrives holding a token signed by a key
 * that is gone. The browser SDK trades a saved token for a short-lived one
 * before it opens the socket, so this surfaces as a failed exchange rather than
 * as a refused connection: no socket is ever attempted, and the dial fails
 * looking exactly like a server that is down.
 */
function isTokenRejected(err: Error | undefined): boolean {
  return /verify token|unauthoriz|forbidden|\b40[13]\b/i.test(err?.message ?? '');
}

export class Net {
  conn!: DbConnection;
  identity: Identity | null = null;
  connected = false;
  subscribed = false;

  /** car_id -> metadata. */
  cars = new Map<number, CarMeta>();
  /** car_id -> the poses and prediction state for a car this client watches. */
  remotes = new Map<number, Remote>();
  /** The tick this client is currently simulating and drawing, which is what
   *  remote cars are predicted to. Written once a frame by the render loop. */
  viewTick = 0;
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
  /** Fingerprint of the physics the current authority is running, or 0 before
   *  any sidecar has claimed. */
  physicsFingerprint = 0;

  // --- measured link quality ---
  rttMs = 0;
  private sentAt = new Map<number, number>();
  private seq = 1;
  snapshotsPerSec = 0;
  private snapCount = 0;
  private snapWindow = performance.now();

  netSim: NetSim = { latencyMs: 0, jitterMs: 0, lossPct: 0 };
  droppedInputs = 0;

  /** Scratch for `sampleRemote`. A full grid sampled on every fixed step is a
   *  couple of hundred short-lived typed arrays a frame, which Firefox collects
   *  as visible hitches; every caller reads the result out before asking for
   *  the next one, so one buffer does. */
  private sampleOut = new Float32Array(24);
  private recordsCache: LapRecordRow[] = [];
  private recordsAt = 0;

  onLocalSnapshot: ((s: Snapshot) => void) | null = null;
  onCarsChanged: (() => void) | null = null;
  onStatus: ((msg: string) => void) | null = null;
  /** The `config` row landed or changed -- which is when the authority's
   *  physics fingerprint becomes known. */
  onConfigChanged: (() => void) | null = null;
  /** Fired once a *re*-connection is subscribed and usable again. The first
   *  connection resolves `connect()` instead. */
  onReconnect: (() => void) | null = null;

  // --- reconnection ---
  private uri = '';
  private dbName = '';
  /** Bumped once per dial. Callbacks arriving from a superseded socket carry a
   *  stale one and are ignored, so a late `close` cannot tear down the
   *  connection that replaced it. */
  private epoch = 0;
  private dialing = false;
  private dialingSince = 0;
  private retryTimer = 0;
  private retryDelay = RECONNECT_BASE_MS;

  async connect(uri: string, dbName: string): Promise<void> {
    this.uri = uri;
    this.dbName = dbName;
    await this.open(true);
    this.watch();
  }

  /**
   * Dial the database.
   *
   * The first attempt rejects if it fails, so the page can fall back to the
   * offline view. Every attempt after that is the reconnect loop and never
   * rejects: it queues another try and returns.
   *
   * `useSaved` is how the one retry in `fail` comes back around without the
   * stored token, and the reason that retry cannot loop: the second pass has
   * no token to have rejected.
   */
  private open(initial: boolean, useSaved = true): Promise<void> {
    const tokenKey = `stdb-sidecar-token:${this.dbName}`;
    const saved = useSaved ? (localStorage.getItem(tokenKey) ?? undefined) : undefined;
    const epoch = ++this.epoch;
    this.dialing = true;
    this.dialingSince = performance.now();

    return new Promise<void>((resolve, reject) => {
      let settled = false;
      const succeed = () => {
        if (settled) return;
        settled = true;
        this.dialing = false;
        resolve();
      };
      const fail = (err: Error) => {
        if (settled) return;
        settled = true;
        this.dialing = false;
        // Held a credential the database will not take: forget it and dial
        // straight back as a stranger. Waiting for the backoff would only
        // present it again, so the offline screen the player would otherwise
        // be looking at is permanent -- and a wrong one, since the server is
        // up and would take them without it.
        if (saved && isTokenRejected(err)) {
          try {
            localStorage.removeItem(tokenKey);
          } catch {
            /* private browsing; there was nothing durable to forget */
          }
          resolve(this.open(initial, false));
          return;
        }
        if (initial) {
          reject(err);
        } else {
          this.scheduleReconnect();
          resolve();
        }
      };

      const builder = DbConnection.builder()
        .withUri(this.uri)
        .withDatabaseName(this.dbName)
        .withToken(saved)
        .onConnect((conn, identity, token) => {
          if (epoch !== this.epoch) return;
          this.conn = conn;
          this.identity = identity;
          this.connected = true;
          this.retryDelay = RECONNECT_BASE_MS;
          try {
            localStorage.setItem(tokenKey, token);
          } catch {
            /* private browsing; a fresh identity per session is fine */
          }
          this.subscribe(conn, epoch, () => {
            succeed();
            if (!initial) this.onReconnect?.();
          });
        })
        .onConnectError((_ctx, err) => {
          if (epoch !== this.epoch) return;
          fail(err);
        })
        .onDisconnect((_ctx, err) => {
          if (epoch !== this.epoch) return;
          this.dropped(err ? `disconnected: ${err.message}` : 'disconnected');
          // A socket that dies before its subscription applied still has to
          // settle this promise, or the retry chain ends here.
          fail(err ?? new Error('disconnected'));
        });
      this.conn = builder.build();
    });
  }

  /** Forget the session that just ended, and queue a redial. */
  private dropped(why: string) {
    if (!this.connected && !this.subscribed) return; // already handled
    this.connected = false;
    this.subscribed = false;
    this.cars.clear();
    this.remotes.clear();
    this.myCarId = 0;
    this.mySlot = -1;
    this.sentAt.clear();
    this.clockLocked = false;
    this.lastSnapTick = 0;
    this.lastSnapAt = 0;
    this.sidecarOnline = false;
    this.snapshotsPerSec = 0;
    this.onStatus?.(why);
    this.onCarsChanged?.();
    this.scheduleReconnect();
  }

  private scheduleReconnect() {
    if (this.retryTimer !== 0 || this.dialing) return;
    const delay = this.retryDelay;
    this.retryDelay = Math.min(this.retryDelay * 2, RECONNECT_MAX_MS);
    this.retryTimer = window.setTimeout(() => {
      this.retryTimer = 0;
      void this.open(false);
    }, delay);
  }

  /**
   * Two ways a session dies without the page being told, and they look
   * identical from the driver's seat -- every other car stops dead and never
   * moves again until a reload:
   *
   * * the socket was torn down while the tab was frozen or the machine asleep,
   *   and no `close` event was ever delivered;
   * * the socket is nominally open but has stopped carrying anything.
   *
   * Neither one fires `onDisconnect`, so both are polled for here.
   */
  private watch() {
    const check = () => {
      if (this.dialing) {
        // Everything below waits for a dial in flight to finish, so one that
        // never does -- a socket that opens and then goes quiet before its
        // subscription applies -- would wedge the retry loop for good.
        if (performance.now() - this.dialingSince < DIAL_TIMEOUT_MS) return;
        this.dialing = false;
        this.epoch++; // orphan the stuck attempt: its callbacks are stale now
        this.retryDelay = RECONNECT_BASE_MS;
        this.dropped('connection timed out — reconnecting');
        this.scheduleReconnect(); // in case `dropped` had nothing to tear down
        try {
          this.conn.disconnect();
        } catch {
          /* it never finished opening; there may be nothing to close */
        }
        return;
      }
      if (this.retryTimer !== 0) return;
      if (!this.connected) {
        this.scheduleReconnect();
        return;
      }
      const zombie = !this.conn.isActive || this.conn.isSocketClosed;
      const stalled =
        this.subscribed &&
        this.sidecarOnline &&
        this.lastSnapAt > 0 &&
        performance.now() - this.lastSnapAt > STALL_MS;
      if (!zombie && !stalled) return;
      // Redial straight away: this link has already been down a while, it did
      // not just fail this instant.
      this.retryDelay = RECONNECT_BASE_MS;
      this.dropped(zombie ? 'connection lost — reconnecting' : 'stream stalled — reconnecting');
      try {
        this.conn.disconnect();
      } catch {
        /* the socket is the thing being complained about; it may already be gone */
      }
    };
    window.setInterval(check, WATCHDOG_MS);
    // Coming back to the foreground is the likeliest moment to be holding a
    // socket the OS closed while the tab was away.
    document.addEventListener('visibilitychange', () => {
      if (!document.hidden) check();
    });
    window.addEventListener('online', check);
  }

  private subscribe(conn: DbConnection, epoch: number, ready: () => void) {
    const db = conn.db;
    const live = () => this.epoch === epoch;

    db.car.onInsert((_ctx, row) => live() && this.upsertCar(row));
    db.car.onUpdate((_ctx, _old, row) => live() && this.upsertCar(row));
    db.car.onDelete((_ctx, row) => {
      if (!live()) return;
      this.cars.delete(row.carId);
      this.remotes.delete(row.carId);
      if (row.carId === this.myCarId) {
        this.myCarId = 0;
        this.mySlot = -1;
      }
      this.onCarsChanged?.();
    });

    db.carState.onInsert((_ctx, row) => live() && this.onState(row));
    db.carState.onUpdate((_ctx, _old, row) => live() && this.onState(row));

    db.config.onInsert((_ctx, row) => live() && this.onConfig(row));
    db.config.onUpdate((_ctx, _old, row) => live() && this.onConfig(row));

    db.player.onUpdate((_ctx: EventContext, _old, row) => {
      if (!live()) return;
      if (this.identity && row.identity.isEqual(this.identity)) {
        this.myCarId = row.carId;
        const meta = this.cars.get(row.carId);
        this.mySlot = meta ? meta.slot : -1;
        this.onCarsChanged?.();
      }
    });

    conn
      .subscriptionBuilder()
      .onApplied(() => {
        if (!live()) return;
        this.subscribed = true;
        for (const c of db.car.iter()) this.upsertCar(c);
        for (const s of db.carState.iter()) this.onState(s);
        for (const cfg of db.config.iter()) this.onConfig(cfg);
        ready();
      })
      .onError(() => live() && this.onStatus?.('subscription failed'))
      .subscribe([
        'SELECT * FROM config',
        'SELECT * FROM car',
        'SELECT * FROM car_state',
        'SELECT * FROM player',
        'SELECT * FROM lap_record',
      ]);
  }

  private onConfig(row: ConfigRow) {
    this.sidecarOnline = row.sidecarOnline;
    this.serverTick = Number(row.serverTick);
    this.physicsFingerprint = row.physicsFingerprint >>> 0;
    this.onConfigChanged?.();
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
   * Positional, in `F` order -- the same `#[repr(C)]` layout the sim maps over
   * wasm memory. The two fields not on the wire (`slip_f`, `slip_r`) are
   * overwritten before they are ever read, so reconstructing them as zero is
   * exact rather than approximate.
   */
  private toRecord(row: CarStateRow): Float32Array {
    return Float32Array.of(
      row.x, row.y, row.heading, row.vx, row.vy, row.omega, row.steer, row.ax,
      0, 0, row.wheelSpin, row.rpm, row.gear, row.s, row.lat, row.seg,
      row.lap, row.cp, row.lapStart, row.lastLap, row.bestLap, row.impact,
      row.wall ? 1 : 0, 1,
    );
  }

  private onState(row: CarStateRow) {
    const tick = Number(row.tick);
    if (tick === 0) return; // placeholder inserted at join

    let r = this.remotes.get(row.carId);
    if (!r) {
      r = { snaps: [], offX: 0, offY: 0, offH: 0, offAt: 0 };
      this.remotes.set(row.carId, r);
    }
    const buf = r.snaps;
    if (buf.length && tick <= buf[buf.length - 1].tick) return; // stale

    const snap: Snapshot = {
      tick,
      state: this.toRecord(row),
      ackSeq: row.ackSeq,
      received: performance.now(),
    };
    // Where this car was about to be drawn, before the news arrived.
    const at = this.viewTick;
    let guessed: [number, number, number] | null = null;
    if (at > 0 && buf.length > 0) {
      const p = this.predict(r, at);
      guessed = [p[F.x], p[F.y], p[F.heading]];
    }
    buf.push(snap);
    if (tick > this.lastSnapTick) {
      this.lastSnapTick = tick;
      this.lastSnapAt = snap.received;
      this.syncClock(tick, snap.received);
    }
    while (buf.length > SNAPSHOT_BUFFER) buf.shift();
    if (guessed) this.absorb(r, guessed, at, snap.received);

    if (row.carId === this.myCarId) {
      const sent = this.sentAt.get(row.ackSeq);
      if (sent !== undefined) {
        const rtt = snap.received - sent;
        // Exponential average; the raw figure is noisy at 20 Hz.
        this.rttMs = this.rttMs === 0 ? rtt : this.rttMs * 0.85 + rtt * 0.15;
        // Sequence numbers only go up and a Map iterates in insertion order,
        // so the first key past the ack ends the sweep -- the whole 512-entry
        // window was being walked to drop the one or two that were acked.
        for (const k of this.sentAt.keys()) {
          if (k > row.ackSeq) break;
          this.sentAt.delete(k);
        }
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
    if (!this.connected) throw new Error('not connected');
    await this.conn.reducers.joinRace({ name, color });
  }

  async leave() {
    if (!this.connected) return;
    await this.conn.reducers.leaveRace({});
  }

  async respawn() {
    if (!this.connected) return;
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

  /** The newest authoritative pose for a car, or null. Not predicted: this is
   *  the authority's own word, which is what the ghost marker draws. */
  authoritative(carId: number): Float32Array | null {
    const r = this.remotes.get(carId);
    return r && r.snaps.length ? r.snaps[r.snaps.length - 1].state : null;
  }

  /**
   * Where a remote car is at `atTick`, on this client's clock.
   *
   * Not "where it was 70 ms ago", which is what an interpolation buffer would
   * give you. The local car is predicted several ticks
   * into the *future* -- far enough ahead that its input arrives before the
   * authority needs it -- so a rival drawn from the past is a rival in the
   * wrong place: metres behind where the authority will resolve the contact,
   * at racing speed. Carrying the newest snapshot forward to the tick actually
   * being simulated puts every car on one clock, and it is the clock the
   * authority used. What it costs is a guess, and [`absorb`] fades away the
   * difference between the guess and the snapshot that settles it.
   */
  predictRemote(carId: number, atTick: number): Float32Array | null {
    const r = this.remotes.get(carId);
    if (!r || r.snaps.length === 0) return null;
    return this.predict(r, atTick);
  }

  /**
   * The same pose with the last correction still fading out of it: what to
   * *draw*. The physics gets [`predictRemote`] instead, because the residual is
   * an apology for a guess that has already been corrected, and re-introducing
   * it would only make the next collision disagree with the authority again.
   */
  sampleRemote(carId: number, atTick: number): Float32Array | null {
    const r = this.remotes.get(carId);
    if (!r || r.snaps.length === 0) return null;
    const out = this.predict(r, atTick);
    const age = (performance.now() - r.offAt) / 1000;
    if (age < RESIDUAL_FADE * 5) {
      const k = Math.exp(-age / RESIDUAL_FADE);
      out[F.x] += r.offX * k;
      out[F.y] += r.offY * k;
      out[F.heading] += r.offH * k;
    }
    return out;
  }

  /**
   * The pose the buffer implies at `atTick`, written into the shared scratch.
   *
   * Between snapshots this interpolates. Past the newest one -- which is where
   * the client normally is, being ahead of the authority -- it carries the car
   * forward at a constant turn rate: a car mid-corner keeps turning, so
   * rotating its velocity as it goes follows the arc instead of flying off the
   * tangent. Over the ten-odd ticks of lead that is the difference between
   * centimetres of error and half a metre.
   */
  private predict(r: Remote, atTick: number): Float32Array {
    const buf = r.snaps;
    const out = this.sampleOut;
    if (buf.length === 1 || atTick <= buf[0].tick) {
      out.set(buf[0].state);
      return out;
    }

    const last = buf[buf.length - 1];
    if (atTick >= last.tick) {
      out.set(last.state);
      const dt = Math.min((atTick - last.tick) / 60, MAX_LEAD);
      const w = out[F.omega];
      const th = w * dt;
      const straight = Math.abs(w) < 1e-3;
      const s = straight ? dt : Math.sin(th) / w;
      const c = straight ? 0 : (1 - Math.cos(th)) / w;
      const vx = out[F.vx];
      const vy = out[F.vy];
      out[F.x] += vx * s - vy * c;
      out[F.y] += vx * c + vy * s;
      out[F.heading] += th;
      const cs = Math.cos(th);
      const sn = Math.sin(th);
      out[F.vx] = vx * cs - vy * sn;
      out[F.vy] = vx * sn + vy * cs;
      return out;
    }

    for (let i = buf.length - 1; i > 0; i--) {
      const b = buf[i];
      const a = buf[i - 1];
      if (atTick >= a.tick && atTick <= b.tick) {
        const t = (atTick - a.tick) / Math.max(1, b.tick - a.tick);
        return lerpState(out, a.state, b.state, t);
      }
    }
    out.set(buf[0].state);
    return out;
  }

  /**
   * Keep the error a new snapshot reveals, rather than showing it.
   *
   * The same trick the local car plays after a rollback: the correction is
   * subtracted from where the car is drawn and decays to nothing over
   * [`RESIDUAL_FADE`], so twenty snapshots a second land as a steady pose
   * instead of a shudder. Whatever is still fading is folded into the new
   * offset, so a run of small corrections does not restart the fade each time.
   */
  private absorb(r: Remote, guessed: [number, number, number], at: number, now: number) {
    const p = this.predict(r, at);
    const k = Math.exp(-(now - r.offAt) / 1000 / RESIDUAL_FADE);
    r.offX = r.offX * k + (guessed[0] - p[F.x]);
    r.offY = r.offY * k + (guessed[1] - p[F.y]);
    r.offH = r.offH * k + wrapPi(guessed[2] - p[F.heading]);
    r.offAt = now;
    if (Math.abs(r.offX) > RESIDUAL_SNAP || Math.abs(r.offY) > RESIDUAL_SNAP) {
      r.offX = r.offY = r.offH = 0;
    }
  }

  /** Best-lap table, quickest first, joined with whoever is on track now.
   *  The module keeps the times; colours and "that's me" only exist client-side,
   *  and a name can outlive its car, so both are optional. */
  records(): LapRecordRow[] {
    // Walking the table, joining it against the grid and sorting is not a
    // per-frame job: a best lap changes a few times a minute at most.
    const now = performance.now();
    if (now - this.recordsAt < 250) return this.recordsCache;
    this.recordsAt = now;
    if (!this.conn?.db || !this.subscribed) return (this.recordsCache = []);
    const live = new Map([...this.cars.values()].map((c) => [c.name, c]));
    return (this.recordsCache = [...this.conn.db.lapRecord.iter()]
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
      .sort((a, b) => a.bestLap - b.bestLap));
  }
}

/** Fields that interpolate linearly. Hoisted out of `lerpState`: it runs for
 *  every remote car on every fixed step. */
const LERP = [F.x, F.y, F.vx, F.vy, F.omega, F.steer, F.wheelSpin, F.rpm, F.s, F.lat];

function lerpState(out: Float32Array, a: Float32Array, b: Float32Array, t: number): Float32Array {
  out.set(b);
  for (const f of LERP) out[f] = a[f] + (b[f] - a[f]) * t;
  out[F.heading] = a[F.heading] + wrapPi(b[F.heading] - a[F.heading]) * t;
  return out;
}
