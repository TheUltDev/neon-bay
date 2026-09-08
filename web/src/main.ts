// Entry point: fixed-step prediction loop, entity interpolation for everyone
// else, and the clock sync that keeps the client running just far enough ahead
// of the sidecar that its inputs arrive on time.

import { GameAudio, type Bus } from './audio';
import { Hud, type HudState, type LeaderRow } from './hud';
import { Controls } from './input';
import { Net } from './net';
import { Renderer, type DrawCar } from './render';
import { DT, F, loadSim, type Sim } from './sim';

/** Extra ticks of headroom on top of the measured round trip. */
const LEAD_MARGIN = 2;
/** How far behind the authority remote cars are drawn, in ticks. */
const INTERP_TICKS = 4.5;
/** Inputs per second sent upstream. */
const INPUT_HZ = 30;
/** Instrument panel repaints per second. It is telemetry, not a mirror, and
 *  every repaint is a layout pass. */
const HUD_HZ = 30;

const PALETTE = [0x38e8ff, 0xff4d9d, 0xffc857, 0x5ef2a8, 0xc77dff, 0xff8c42, 0x60a5fa, 0xf472b6];

const $ = <T extends HTMLElement = HTMLElement>(id: string) => document.getElementById(id) as T;

// Most specific wins: `?uri=` points one tab at someone else's machine, then
// whatever was baked in at build time (how the deployed bundle finds its
// server, since it is not on the page's own host), then the dev default of a
// SpacetimeDB sitting on port 3000 of wherever this page came from.
function serverUri(): { uri: string; db: string } {
  const q = new URLSearchParams(location.search);
  return {
    uri:
      q.get('uri') ??
      import.meta.env.VITE_STDB_URI ??
      `${location.protocol}//${location.hostname}:3000`,
    db: q.get('db') ?? import.meta.env.VITE_STDB_DB ?? 'physics-sidecar',
  };
}

async function boot() {
  const canvas = $<HTMLCanvasElement>('stage');
  const sim = await loadSim('/physics.wasm');
  const renderer = new Renderer(canvas, sim);
  const hud = new Hud();
  const controls = new Controls();
  const audio = new GameAudio();
  const net = new Net();

  let chosenColor = PALETTE[0];
  buildColorPicker(chosenColor, (c) => (chosenColor = c));

  // --- keygen radio --------------------------------------------------------
  // Bound before the mixer, whose faders repaint the strip: a fader at zero is
  // the usual reason the meter is not moving.
  const npBar = $('nowplaying');
  const paintRadio = () => {
    const s = audio.nowPlaying();
    $('np-title').textContent = s.name;
    $('np-meta').textContent = `${s.bpm} BPM · ${s.index + 1}/${s.count}`;
    npBar.classList.toggle('live', audio.musicAudible);
  };
  const startAudio = () => audio.start().then(paintRadio).catch(() => {});
  const tune = (step: number) => {
    // A keypress is a gesture, so the radio keys double as the audio unlock.
    startAudio();
    audio.setStation(audio.station + step);
    paintRadio();
    npBar.classList.remove('tuned');
    void npBar.offsetWidth; // restart the flash even on a second press mid-fade
    npBar.classList.add('tuned');
  };
  controls.onTap('BracketLeft', () => tune(-1));
  controls.onTap('BracketRight', () => tune(1));
  npBar.addEventListener('click', () => tune(1));
  paintRadio();

  // --- mixer ---------------------------------------------------------------
  // Bound before the connection attempt, so the faders still work on the
  // offline page -- where touching the panel is the gesture that unlocks audio.
  const fader = (id: string, out: string, bus: Bus) =>
    bindSlider(
      id,
      out,
      '%',
      (v) => {
        audio.setVolume(bus, v / 100);
        paintRadio();
      },
      Math.round(audio.volume[bus] * 100),
    );
  const setMaster = fader('s-master', 'v-master', 'master');
  fader('s-sfx', 'v-sfx', 'sfx');
  fader('s-music', 'v-music', 'music');
  $('audio-panel').addEventListener('pointerdown', () => startAudio(), { once: true });

  controls.onTap('KeyM', () => {
    setMaster(Math.round(audio.toggleMute() * 100));
    hud.toast(audio.muted ? 'audio muted' : 'audio on');
    paintRadio();
  });
  // Nothing should be playing into a tab nobody is looking at.
  document.addEventListener('visibilitychange', () => audio.setHidden(document.hidden));

  const { uri, db } = serverUri();
  try {
    await net.connect(uri, db);
    $('g-status').textContent = `connected to ${db} at ${uri}`;
  } catch (e) {
    $('g-status').innerHTML = `<b style="color:var(--red)">could not reach ${uri}</b><br>start SpacetimeDB and publish the module, then reload.`;
    startRenderOnly(renderer, sim, hud);
    return;
  }

  // --- reconciliation hook -------------------------------------------------
  let pendingToast = 0;
  net.onLocalSnapshot = (snap) => {
    const before = sim.stats.resyncs;
    sim.reconcile(snap.tick, snap.state);
    hud.pushError(sim.stats.error);
    if (sim.stats.resyncs > before && performance.now() - pendingToast > 1500) {
      pendingToast = performance.now();
      hud.toast('authority resync');
    }
  };
  let joined = false;
  let rejoinAt = 0;
  net.onCarsChanged = () => {
    sim.setLocalSlot(net.mySlot);
    refreshActiveMask(sim, net);
  };
  net.onStatus = (msg) => {
    $('g-status').textContent = msg;
    if (joined) hud.toast(msg, 2400);
  };

  /**
   * Put the player back on the grid if they are not on it.
   *
   * The module reaps a car when its owner's socket closes, so surviving a
   * reconnect means asking for a new one. Asking repeatedly, at that: a join
   * sent the instant the new connection lands can arrive before the module has
   * finished tearing the old one down, in which case it is a no-op and the car
   * is reaped a moment later anyway.
   */
  const ensureCar = (now: number) => {
    if (!joined || !net.connected || net.myCarId !== 0) {
      rejoinAt = now + 600;
      return;
    }
    if (now < rejoinAt) return;
    rejoinAt = now + 2000;
    net.join(lastName, chosenColor).catch(() => {});
  };
  net.onReconnect = () => {
    hud.toast('reconnected');
    // Do not wait for the next frame: a tab in the background is not getting
    // one, and coming back to a race you have been dropped out of is the exact
    // thing this is here to prevent.
    rejoinAt = 0;
    ensureCar(performance.now());
  };

  // --- join flow -----------------------------------------------------------
  const nameInput = $<HTMLInputElement>('g-name');
  nameInput.value = localStorage.getItem('driver-name') ?? '';
  let lastName = nameInput.value;
  const join = async () => {
    // Joining is the first gesture on the page, which is also the browser's cue
    // that we are allowed to make noise.
    startAudio();
    const name = nameInput.value.trim() || `DRIVER${Math.floor(Math.random() * 90 + 10)}`;
    localStorage.setItem('driver-name', name);
    $<HTMLButtonElement>('g-join').disabled = true;
    try {
      await net.join(name, chosenColor);
      joined = true;
      lastName = name;
      $('gate').classList.add('hidden');
      canvas.focus();
    } catch (e) {
      $('g-status').textContent = `join failed: ${e}`;
      $<HTMLButtonElement>('g-join').disabled = false;
    }
  };
  $('g-join').addEventListener('click', join);
  nameInput.addEventListener('keydown', (e) => {
    if ((e as KeyboardEvent).key === 'Enter') join();
  });

  // --- demo controls -------------------------------------------------------
  bindSlider('s-lat', 'v-lat', 'ms', (v) => (net.netSim.latencyMs = v));
  bindSlider('s-jit', 'v-jit', 'ms', (v) => (net.netSim.jitterMs = v));
  bindSlider('s-loss', 'v-loss', '%', (v) => (net.netSim.lossPct = v));

  $('b-cheat').addEventListener('click', () => {
    sim.cheat(26);
    hud.toast('client desync injected');
  });
  $('b-respawn').addEventListener('click', () => net.respawn());
  $<HTMLInputElement>('t-ghost').addEventListener('change', (e) => {
    renderer.showGhost = (e.target as HTMLInputElement).checked;
  });
  $<HTMLInputElement>('t-rotate').addEventListener('change', (e) => {
    renderer.rotateCamera = (e.target as HTMLInputElement).checked;
  });

  controls.onTap('KeyR', () => net.respawn());
  controls.onTap('KeyG', () => {
    renderer.showGhost = !renderer.showGhost;
    $<HTMLInputElement>('t-ghost').checked = renderer.showGhost;
  });
  controls.onTap('KeyC', () => {
    renderer.rotateCamera = !renderer.rotateCamera;
    $<HTMLInputElement>('t-rotate').checked = renderer.rotateCamera;
  });

  // The authority publishes what its physics computes; this one knows what its
  // own does. They are built from the same source, so they agree unless the two
  // halves were deployed apart -- which otherwise shows up only as prediction
  // error, and reads as a network problem rather than a stale bundle.
  let warnedMismatch = false;
  const checkPhysics = () => {
    if (warnedMismatch || !physicsMismatch(sim, net)) return;
    warnedMismatch = true;
    const mine = hex(sim.fingerprint);
    const theirs = hex(net.physicsFingerprint);
    console.error(
      `physics mismatch: this client computes ${mine}, the authority computes ${theirs}. ` +
        'They were built from different sources, so prediction will not hold and every ' +
        'corner will end in a correction. Redeploy physics.wasm and the sidecar together.',
    );
    hud.toast('physics mismatch — see console', 6000);
    $('g-status').innerHTML =
      `<b style="color:var(--red)">physics mismatch</b><br>client ${mine} vs authority ${theirs}. ` +
      'Prediction is off until both are redeployed from the same source.';
  };
  net.onConfigChanged = checkPhysics;
  checkPhysics();

  // Handy from the devtools console when poking at the netcode:
  //   __neon.sim.stats, __neon.net.rttMs, __neon.sim.cheat(30)
  (window as unknown as Record<string, unknown>).__neon = { sim, net, renderer, hud, controls, audio };

  // --- loop state ----------------------------------------------------------
  let last = performance.now();
  let acc = 0;
  let sinceInput = 0;
  let correctionWindow = performance.now();
  let correctionsAtWindow = 0;
  let correctionsPerSec = 0;
  let hudAt = 0;
  const measureFps = frameCounter();

  function frame(now: number) {
    const dt = Math.min((now - last) / 1000, 0.25);
    last = now;

    // Losing the car mid-session -- a disconnect race, or the grid being reset
    // -- should not strand the player staring at a track they cannot drive on.
    ensureCar(now);

    // ---- clock sync -------------------------------------------------------
    // Run far enough ahead that an input for tick T reaches the sidecar before
    // it simulates T: half a round trip out, plus a little margin.
    const rttTicks = (net.rttMs / 1000) * 60;
    const serverNow = net.estimatedServerTick(now);
    const targetTick = serverNow + rttTicks + LEAD_MARGIN;
    let timeScale = 1;
    if (sim.localSlot >= 0 && net.lastSnapTick > 0) {
      const drift = targetTick - sim.localTick;
      if (Math.abs(drift) > 30) {
        sim.localTick = Math.round(targetTick);
      } else {
        // Gentle time dilation instead of a jump; you cannot feel 6%.
        timeScale = 1 + Math.max(-0.06, Math.min(0.06, drift * 0.02));
      }
    }

    // ---- fixed-step prediction -------------------------------------------
    acc += dt * timeScale;
    let steps = 0;
    while (acc >= DT && steps < 6) {
      acc -= DT;
      steps++;
      placeRemotes(sim, net, serverNow - INTERP_TICKS);
      if (sim.localSlot >= 0) {
        const input = controls.read(forwardSpeed(sim));
        // Stamp the tick this input *drives*, i.e. the one before the step it is
        // about to take. The sidecar holds it until its own clock reaches that
        // tick, which is what keeps the two simulations in lockstep.
        const inputTick = sim.localTick;
        sim.step(input);
        sinceInput += DT;
        if (sinceInput >= 1 / INPUT_HZ) {
          sinceInput = 0;
          net.sendInput(inputTick, input.throttle, input.steer, input.brake, input.handbrake > 0.5);
        }
      }
    }
    if (steps === 0) placeRemotes(sim, net, serverNow - INTERP_TICKS);
    if (acc > DT * 6) acc = 0;

    sim.decaySmoothing(dt);

    // ---- assemble the frame ----------------------------------------------
    // Whatever is left in the accumulator is the part of a tick this frame
    // falls past the last one simulated. Remote cars already get it for free,
    // being sampled at a fractional tick; the local car needs it handed over,
    // or it is the only thing on screen that moves in 60 Hz steps.
    const alpha = acc / DT;
    const cars = collectCars(sim, net, serverNow - INTERP_TICKS, alpha);
    const local = cars.find((c) => c.isLocal) ?? null;
    const followed = local ?? leader(cars);

    if (followed) {
      const vx = followed.isLocal ? sim.field(sim.localSlot, F.vx) : 0;
      const vy = followed.isLocal ? sim.field(sim.localSlot, F.vy) : 0;
      renderer.updateCamera(followed, vx, vy, dt, !followed.isLocal);
    }

    spawnEffects(renderer, cars, dt);

    // Impact feedback for the local car.
    if (sim.localSlot >= 0) {
      const impact = sim.field(sim.localSlot, F.impact);
      if (impact > 400) {
        renderer.impulse(Math.min(16, impact / 900));
        const p = sim.localPose(alpha);
        for (let i = 0; i < Math.min(14, impact / 700); i++) {
          renderer.addSpark(p.x, p.y, sim.field(sim.localSlot, F.vx), sim.field(sim.localSlot, F.vy));
        }
      }
    }

    const ghost = ghostPose(net);
    renderer.draw(cars, ghost, dt, local ? local.speed : 0);
    renderer.drawMinimap($<HTMLCanvasElement>('minimap'), sim, cars);

    // ---- telemetry --------------------------------------------------------
    // Every frame, not every HUD repaint: this is counting them.
    const fps = measureFps(now);
    if (now - correctionWindow > 1000) {
      correctionsPerSec = ((sim.stats.corrections - correctionsAtWindow) * 1000) / (now - correctionWindow);
      correctionsAtWindow = sim.stats.corrections;
      correctionWindow = now;
    }
    if (now - hudAt >= 1000 / HUD_HZ) {
      hudAt = now;
      hud.update(buildHudState(sim, net, correctionsPerSec, serverNow, fps));
    }

    if (audio.running && sim.localSlot >= 0) {
      audio.engine(
        sim.field(sim.localSlot, F.rpm),
        Math.abs(sim.inputs[sim.localSlot * 4]),
        sim.field(sim.localSlot, F.wheelSpin),
      );
    }

    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
}

// ---------------------------------------------------------------- helpers --

function forwardSpeed(sim: Sim): number {
  if (sim.localSlot < 0) return 0;
  const h = sim.field(sim.localSlot, F.heading);
  return sim.field(sim.localSlot, F.vx) * Math.cos(h) + sim.field(sim.localSlot, F.vy) * Math.sin(h);
}

/** Tell the wasm world which slots hold a car, so collisions include them. */
function refreshActiveMask(sim: Sim, net: Net) {
  let mask = 0;
  for (const c of net.cars.values()) mask |= 1 << c.slot;
  sim.setActive(mask);
}

/**
 * Park every remote car at its interpolated pose. They are never integrated by
 * this client -- but they are solid, so you can lean on a rival through a
 * corner and the local prediction reacts immediately.
 */
function placeRemotes(sim: Sim, net: Net, atTick: number) {
  for (const meta of net.cars.values()) {
    if (meta.carId === net.myCarId) continue;
    const s = net.sampleRemote(meta.carId, atTick);
    if (!s) continue;
    sim.placeRemote(meta.slot, s[F.x], s[F.y], s[F.heading], s[F.vx], s[F.vy]);
  }
}

function collectCars(sim: Sim, net: Net, atTick: number, alpha: number): DrawCar[] {
  const out: DrawCar[] = [];
  for (const meta of net.cars.values()) {
    if (meta.carId === net.myCarId && sim.localSlot >= 0) {
      const pose = sim.localPose(alpha);
      out.push({
        slot: meta.slot,
        x: pose.x,
        y: pose.y,
        heading: pose.heading,
        steer: pose.steer,
        color: meta.color,
        name: meta.name,
        isLocal: true,
        isBot: false,
        speed: Math.hypot(sim.field(meta.slot, F.vx), sim.field(meta.slot, F.vy)),
        wheelSpin: sim.field(meta.slot, F.wheelSpin),
        braking: sim.inputs[meta.slot * 4 + 2] > 0.1,
        throttle: sim.inputs[meta.slot * 4],
        lap: sim.field(meta.slot, F.lap),
      });
      continue;
    }
    const s = net.sampleRemote(meta.carId, atTick);
    if (!s) continue;
    out.push({
      slot: meta.slot,
      x: s[F.x],
      y: s[F.y],
      heading: s[F.heading],
      steer: s[F.steer],
      color: meta.color,
      name: meta.name,
      isLocal: false,
      isBot: meta.isBot,
      speed: Math.hypot(s[F.vx], s[F.vy]),
      wheelSpin: s[F.wheelSpin],
      braking: false,
      throttle: 1,
      lap: s[F.lap],
    });
  }
  return out;
}

function leader(cars: DrawCar[]): DrawCar | null {
  let best: DrawCar | null = null;
  for (const c of cars) if (!best || c.lap > best.lap) best = c;
  return best;
}

/**
 * Whether this client and the authority are running the same simulation.
 *
 * They have to be, or none of the rest of this means anything: the prediction
 * loop assumes that stepping the same inputs here and there lands on the same
 * bits, and if it does not, every corner ends in a correction that looks like
 * packet loss and is really a stale `physics.wasm`. Zero means no sidecar has
 * claimed yet, so there is nothing to compare against.
 */
function physicsMismatch(sim: Sim, net: Net): boolean {
  return net.physicsFingerprint !== 0 && net.physicsFingerprint !== sim.fingerprint;
}

/** Latest authoritative pose of the local car, for the ghost overlay. */
function ghostPose(net: Net) {
  const buf = net.buffers.get(net.myCarId);
  if (!buf || buf.length === 0) return null;
  const s = buf[buf.length - 1].state;
  return { x: s[F.x], y: s[F.y], heading: s[F.heading] };
}

// Rear tires lay the marks: `wheel_spin` is the rear axle's share of the
// friction budget, so a car only streaks once it is actually sliding.
const REAR_AXLE = 1.32;
const REAR_TRACK = 0.95;

function spawnEffects(renderer: Renderer, cars: DrawCar[], dt: number) {
  for (const c of cars) {
    if (c.speed < 6 || c.wheelSpin < 0.3) continue;
    const cs = Math.cos(c.heading);
    const sn = Math.sin(c.heading);
    for (const side of [1, -1]) {
      const wx = c.x - cs * REAR_AXLE - sn * REAR_TRACK * side;
      const wy = c.y - sn * REAR_AXLE + cs * REAR_TRACK * side;
      // One id per tire, stable across frames, so each wheel's streak joins up
      // to its own previous contact point and not the other side's.
      renderer.addSkid(c.slot * 2 + (side > 0 ? 0 : 1), wx, wy, c.wheelSpin * 0.5);
      if (Math.random() < c.wheelSpin * dt * 16) {
        renderer.addSmoke(wx, wy, -cs * 2, -sn * 2, c.wheelSpin);
      }
    }
  }
}

function buildHudState(
  sim: Sim,
  net: Net,
  correctionsPerSec: number,
  serverNow: number,
  fps: number,
): HudState {
  const slot = sim.localSlot;
  const has = slot >= 0;
  const speed = has ? Math.hypot(sim.field(slot, F.vx), sim.field(slot, F.vy)) : 0;
  const lapStart = has ? sim.field(slot, F.lapStart) : 0;
  const lapTime = has ? Math.max(0, (sim.localTick - lapStart) / 60) : 0;

  // Ten quickest laps the module has on file. If you are not among them, your
  // own row rides along underneath at its true rank rather than vanishing.
  const board = net.records();
  const top: LeaderRow[] = board.slice(0, 10).map((r, i) => ({ ...r, rank: i + 1 }));
  const mineAt = board.findIndex((r) => r.mine);
  if (mineAt >= 10) top.push({ ...board[mineAt], rank: mineAt + 1, cut: true });

  return {
    authority: !net.connected
      ? 'connecting'
      : physicsMismatch(sim, net)
        ? 'mismatch'
        : net.sidecarOnline
          ? 'online'
          : 'offline',
    fps,
    serverTick: Math.round(serverNow),
    clientTick: sim.localTick,
    lead: has ? sim.localTick - Math.round(serverNow) : 0,
    rttMs: net.rttMs,
    error: sim.stats.error,
    smoothing: sim.smoothingResidual,
    correctionsPerSec,
    replayTicks: sim.stats.replayTicks,
    snapshotHz: net.snapshotsPerSec,
    droppedInputs: net.droppedInputs,
    resyncs: sim.stats.resyncs,
    speedKph: speed * 3.6,
    gear: has ? sim.field(slot, F.gear) : 1,
    rpm: has ? sim.field(slot, F.rpm) : 0,
    throttle: has ? sim.inputs[slot * 4] : 0,
    brake: has ? sim.inputs[slot * 4 + 2] : 0,
    steer: has ? sim.inputs[slot * 4 + 1] : 0,
    lap: has ? sim.field(slot, F.lap) : 0,
    lapTime,
    lastLap: has ? sim.field(slot, F.lastLap) : 0,
    bestLap: has ? sim.field(slot, F.bestLap) : 0,
    leaderboard: top,
  };
}

const hex = (v: number) => `0x${(v >>> 0).toString(16).padStart(8, '0')}`;

/** Wires a range input to its readout. Returns a setter for driving it in code. */
function bindSlider(
  id: string,
  out: string,
  unit: string,
  apply: (v: number) => void,
  initial?: number,
) {
  const el = $<HTMLInputElement>(id);
  const label = $(out);
  const sync = () => {
    const v = Number(el.value);
    label.textContent = `${v} ${unit}`;
    apply(v);
  };
  if (initial !== undefined) el.value = String(initial);
  el.addEventListener('input', sync);
  sync();
  return (v: number) => {
    el.value = String(v);
    sync();
  };
}

function buildColorPicker(initial: number, onPick: (c: number) => void) {
  const host = $('g-colors');
  PALETTE.forEach((c, i) => {
    const s = document.createElement('span');
    s.style.background = `#${c.toString(16).padStart(6, '0')}`;
    if (c === initial) s.classList.add('sel');
    s.addEventListener('click', () => {
      host.querySelectorAll('span').forEach((n) => n.classList.remove('sel'));
      s.classList.add('sel');
      onPick(c);
    });
    host.append(s);
    if (i === 0) s.classList.add('sel');
  });
}

/**
 * Rendered frames per second, averaged over half-second windows.
 *
 * A shorter window than the corrections counter uses, deliberately: this is the
 * number you watch while changing something, so it has to react. Call it once
 * per frame -- it is counting the calls.
 */
function frameCounter(): (now: number) => number {
  let windowStart = performance.now();
  let frames = 0;
  let fps = 0;
  return (now) => {
    frames++;
    if (now - windowStart >= 500) {
      fps = (frames * 1000) / (now - windowStart);
      frames = 0;
      windowStart = now;
    }
    return fps;
  };
}

/** Offline fallback: still show the circuit so the page is not a blank void. */
function startRenderOnly(renderer: Renderer, sim: Sim, hud: Hud) {
  let last = performance.now();
  const measureFps = frameCounter();
  const loop = (now: number) => {
    const dt = Math.min((now - last) / 1000, 0.1);
    last = now;
    renderer.camX = Math.cos(now / 9000) * 120;
    renderer.camY = Math.sin(now / 9000) * 120;
    renderer.camZoom = 6;
    renderer.draw([], null, dt, 0);
    renderer.drawMinimap($<HTMLCanvasElement>('minimap'), sim, []);
    // Nothing else on this page has a number in it, and the renderer is the
    // only thing still running -- so the frame rate is worth showing even here.
    hud.update({ ...offlineHud(), fps: measureFps(now) });
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}

function offlineHud(): HudState {
  return {
    authority: 'offline',
    fps: 0,
    serverTick: 0,
    clientTick: 0,
    lead: 0,
    rttMs: 0,
    error: 0,
    smoothing: 0,
    correctionsPerSec: 0,
    replayTicks: 0,
    snapshotHz: 0,
    resyncs: 0,
    speedKph: 0,
    gear: 1,
    rpm: 0,
    throttle: 0,
    brake: 0,
    steer: 0,
    lap: 0,
    lapTime: 0,
    lastLap: 0,
    bestLap: 0,
    droppedInputs: 0,
    leaderboard: [],
  };
}

boot().catch((e) => {
  console.error(e);
  const s = document.getElementById('g-status');
  if (s) s.textContent = `startup failed: ${e}`;
});
