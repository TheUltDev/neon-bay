// HUD: the instrument panel, and the telemetry that makes the netcode visible.
//
// Roughly thirty readouts, repainted continuously. Writing to `textContent` or
// to `style` invalidates layout even when the value has not moved, so every
// write here goes through `set`/`css`, which remember what they last wrote and
// do nothing if it still holds. On a steady lap that is most of them.
//
// The prediction-error graph is the point of the whole demo. On a healthy link
// it sits pinned at zero, because the browser and the sidecar are running the
// same instructions on the same inputs. Add latency or loss and it starts to
// breathe; press "Desync me" and it spikes, then the correction gets smoothed
// away over a couple of hundred milliseconds.

const $ = <T extends HTMLElement = HTMLElement>(id: string): T => {
  const el = document.getElementById(id);
  if (!el) throw new Error(`missing element #${id}`);
  return el as T;
};

export interface HudState {
  authority: 'online' | 'offline' | 'connecting' | 'mismatch';
  /** Rendered frames per second, measured over the last half second. */
  fps: number;
  /** Which renderer the browser gave us, and what it is drawing into. */
  gfx: GfxState;
  serverTick: number;
  clientTick: number;
  lead: number;
  rttMs: number;
  error: number;
  smoothing: number;
  correctionsPerSec: number;
  replayTicks: number;
  snapshotHz: number;
  droppedInputs: number;
  resyncs: number;
  speedKph: number;
  gear: number;
  rpm: number;
  /** Lateral acceleration in g. Real, now that load transfer is: it is
   *  what the four contact patches actually managed between them. */
  latG: number;
  /** Worst crush on the body, 0 (straight) to 1 (written off). */
  damage: number;
  throttle: number;
  brake: number;
  steer: number;
  lap: number;
  lapTime: number;
  lastLap: number;
  bestLap: number;
  /** Top ten best laps as aggregated by the module, quickest first. Your own
   *  row is appended out of rank order when you did not make the cut, so the
   *  board can always answer "where am I". */
  leaderboard: LeaderRow[];
}

export interface GfxState {
  /** "WebGPU", "WebGL 2", "Canvas 2D". */
  api: string;
  /** The GPU, short enough for the panel. */
  device: string;
  /** Everything the driver would say, for the tooltip. */
  detail: string;
  /** Why this tier and not the one above it, when there is a reason. */
  fallback: string | null;
  /** Drawing buffer, in device pixels, and the ratio it came from. */
  w: number;
  h: number;
  dpr: number;
  /** False on the Canvas2D fallback: the picture is the same, the path is not. */
  accelerated: boolean;
}

export interface LeaderRow {
  rank: number;
  name: string;
  bestLap: number;
  /** null once the car that set the time has left the race. */
  color: number | null;
  mine: boolean;
  /** True for the pinned row below the cut. */
  cut?: boolean;
}

const GRAPH_SAMPLES = 168;

export class Hud {
  private errHistory = new Float32Array(GRAPH_SAMPLES);
  private errHead = 0;
  private graph: CanvasRenderingContext2D;
  private tacho: CanvasRenderingContext2D;
  private toastTimer = 0;
  private lastLeaderboard: string | null = null;

  /** Element cache, so a repaint is not thirty `getElementById` calls. */
  private els = new Map<string, HTMLElement>();
  /** What each of those was last set to, keyed by element id plus property. */
  private painted = new Map<string, string>();
  private graphGrad: CanvasGradient | null = null;
  private tachoGrad: CanvasGradient | null = null;

  constructor() {
    this.graph = $<HTMLCanvasElement>('errgraph').getContext('2d')!;
    this.tacho = $<HTMLCanvasElement>('tacho').getContext('2d')!;
  }

  private el(id: string): HTMLElement {
    let e = this.els.get(id);
    if (!e) {
      e = $(id);
      this.els.set(id, e);
    }
    return e;
  }

  /** Text, written only if it changed. */
  private set(id: string, text: string) {
    if (this.painted.get(id) === text) return;
    this.painted.set(id, text);
    this.el(id).textContent = text;
  }

  /** One style property, written only if it changed. */
  private css(id: string, prop: 'color' | 'width' | 'left' | 'background', value: string) {
    const key = `${id}.${prop}`;
    if (this.painted.get(key) === value) return;
    this.painted.set(key, value);
    this.el(id).style[prop] = value;
  }

  /** One attribute, written only if it changed. */
  private attr(id: string, name: string, value: string) {
    const key = `${id}@${name}`;
    if (this.painted.get(key) === value) return;
    this.painted.set(key, value);
    this.el(id).setAttribute(name, value);
  }

  /** A line that is there when there is something to say, and gone when not. */
  private note(id: string, text: string | null) {
    const key = `${id}.note`;
    const v = text ?? '';
    if (this.painted.get(key) === v) return;
    this.painted.set(key, v);
    const el = this.el(id);
    el.textContent = v;
    el.hidden = v === '';
  }

  pushError(e: number) {
    this.errHistory[this.errHead] = e;
    this.errHead = (this.errHead + 1) % GRAPH_SAMPLES;
  }

  toast(msg: string, ms = 1400) {
    const el = $('toast');
    el.textContent = msg;
    el.classList.add('show');
    clearTimeout(this.toastTimer);
    this.toastTimer = window.setTimeout(() => el.classList.remove('show'), ms);
  }

  update(s: HudState) {
    const dot = `dot ${s.authority === 'online' ? 'ok' : s.authority === 'connecting' ? 'warn' : 'bad'}`;
    if (this.painted.get('auth-dot.class') !== dot) {
      this.painted.set('auth-dot.class', dot);
      this.el('auth-dot').className = dot;
    }
    this.set(
      'auth-state',
      s.authority === 'online'
        ? 'sidecar authoritative'
        : s.authority === 'offline'
          ? 'no authority'
          : s.authority === 'mismatch'
            ? 'physics mismatch'
            : 'connecting…',
    );

    this.set('k-servertick', s.serverTick.toLocaleString());
    this.set('k-clienttick', s.clientTick.toLocaleString());
    this.set('k-lead', `${s.lead >= 0 ? '+' : ''}${s.lead} ticks`);

    // --- renderer ---
    // The dot is green on a GPU backend and amber on the 2D fallback: not a
    // fault, but the slow path, and worth seeing before reading the frame rate
    // underneath it.
    const gdot = `dot ${s.gfx.accelerated ? 'ok' : 'warn'}`;
    if (this.painted.get('gfx-dot.class') !== gdot) {
      this.painted.set('gfx-dot.class', gdot);
      this.el('gfx-dot').className = gdot;
    }
    this.set('gfx-api', s.gfx.api);
    this.set('gfx-device', s.gfx.device);
    this.attr('gfx-device', 'title', s.gfx.detail);
    this.set('gfx-res', `${s.gfx.w}×${s.gfx.h}${s.gfx.dpr === 1 ? '' : ` @${trim(s.gfx.dpr)}x`}`);
    this.note('gfx-note', s.gfx.fallback);

    this.set('k-fps', s.fps > 0 ? Math.round(s.fps).toString() : '—');
    // The simulation is fixed-step, so a slow frame rate does not change what
    // happens -- but it is the first thing to check when the demo feels wrong,
    // and it is the one number here the client alone is responsible for.
    this.css('k-fps', 'color', s.fps === 0 || s.fps >= 50 ? 'var(--text)' : s.fps >= 30 ? 'var(--amber)' : 'var(--pink)');

    this.set('k-rtt', s.rttMs > 0 ? `${s.rttMs.toFixed(0)} ms` : '—');
    this.set('k-err', `${s.error.toFixed(3)} m`);
    this.css('k-err', 'color', s.error > 0.25 ? 'var(--pink)' : s.error > 0.02 ? 'var(--amber)' : 'var(--green)');
    this.set('k-smooth', `${s.smoothing.toFixed(3)} m`);
    this.set('k-corr', `${s.correctionsPerSec.toFixed(1)} /s`);
    this.set('k-replay', `${s.replayTicks}`);
    this.set('k-snaps', s.snapshotHz > 0 ? `${s.snapshotHz.toFixed(1)} Hz` : '—');
    this.set('k-dropped', `${s.droppedInputs}`);
    this.css('k-dropped', 'color', s.droppedInputs > 0 ? 'var(--amber)' : 'var(--text)');
    this.set('k-resync', `${s.resyncs}`);

    this.set('k-speed', Math.round(s.speedKph).toString());
    this.set('k-gear', gearLabel(s.gear));
    this.set('k-latg', Math.abs(s.latG).toFixed(2));
    // The car peaks somewhere between 1.3 g and 1.7 g depending on how
    // much downforce the speed is worth, so 1.2 is 'leaning on it'.
    this.css('k-latg', 'color', Math.abs(s.latG) > 1.2 ? 'var(--amber)' : 'var(--text)');
    // Bodywork. Not cosmetic: past about a third of this the car has visibly
    // less downforce, less lock, less power and less grip, and the driver
    // deserves to be told which of those they are now driving around.
    this.set('k-dmg', s.damage < 0.02 ? 'OK' : `${Math.round(s.damage * 100)}%`);
    this.css(
      'k-dmg',
      'color',
      s.damage > 0.6 ? 'var(--pink)' : s.damage > 0.2 ? 'var(--amber)' : 'var(--text)',
    );
    this.css('bar-thr', 'width', `${(Math.max(0, s.throttle) * 100).toFixed(1)}%`);
    this.css('bar-brk', 'width', `${(s.brake * 100).toFixed(1)}%`);
    this.css('bar-str', 'width', `${(Math.abs(s.steer) * 50).toFixed(1)}%`);
    this.css('bar-str', 'left', s.steer > 0 ? `${(50 - Math.abs(s.steer) * 50).toFixed(1)}%` : '50%');

    this.set('k-lap', s.lap > 0 ? s.lap.toString() : '–');
    this.set('k-laptime', fmtTime(s.lapTime));
    this.set('k-lastlap', s.lastLap > 0 ? fmtTime(s.lastLap) : '—');
    this.set('k-bestlap', s.bestLap > 0 ? fmtTime(s.bestLap) : '—');

    this.drawGraph(s.error);
    this.drawTacho(s.rpm, s.speedKph, s.gear);
    this.drawLeaderboard(s.leaderboard);
  }

  private drawGraph(current: number) {
    const g = this.graph;
    const c = g.canvas;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = c.clientWidth || 252;
    const h = c.clientHeight || 62;
    if (c.width !== Math.round(w * dpr) || c.height !== Math.round(h * dpr)) {
      c.width = Math.round(w * dpr);
      c.height = Math.round(h * dpr);
    }
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, w, h);

    // Log scale: interesting values span 1 mm to 10 m.
    const toY = (v: number) => {
      const t = Math.log10(Math.max(v, 0.001) / 0.001) / Math.log10(10 / 0.001);
      return h - 3 - Math.min(1, t) * (h - 8);
    };

    g.strokeStyle = 'rgba(120,170,220,0.12)';
    g.lineWidth = 1;
    g.beginPath();
    for (const v of [0.01, 0.1, 1]) {
      const y = Math.round(toY(v)) + 0.5;
      g.moveTo(0, y);
      g.lineTo(w, y);
    }
    g.stroke();

    // Below about 50px the decade labels collide with the gridlines they name.
    if (h >= 50) {
      g.font = '8px ui-monospace, monospace';
      g.fillStyle = 'rgba(120,170,220,0.4)';
      g.fillText('1 m', 3, toY(1) - 2);
      g.fillText('1 cm', 3, toY(0.01) - 2);
    }

    g.beginPath();
    for (let i = 0; i < GRAPH_SAMPLES; i++) {
      const v = this.errHistory[(this.errHead + i) % GRAPH_SAMPLES];
      const x = (i / (GRAPH_SAMPLES - 1)) * w;
      const y = toY(v);
      i === 0 ? g.moveTo(x, y) : g.lineTo(x, y);
    }
    if (!this.graphGrad) {
      const grad = g.createLinearGradient(0, 0, 0, h);
      grad.addColorStop(0, '#ff4d9d');
      grad.addColorStop(0.55, '#ffc857');
      grad.addColorStop(1, '#5ef2a8');
      this.graphGrad = grad;
    }
    const grad = this.graphGrad;
    g.strokeStyle = grad;
    g.lineWidth = 1.4;
    g.stroke();

    g.lineTo(w, h);
    g.lineTo(0, h);
    g.closePath();
    g.globalAlpha = 0.13;
    g.fillStyle = grad;
    g.fill();
    g.globalAlpha = 1;

    if (current < 0.0015) {
      g.fillStyle = 'rgba(94,242,168,0.85)';
      g.font = '600 9px ui-monospace, monospace';
      g.textAlign = 'right';
      g.fillText('PREDICTION EXACT', w - 5, 11);
      g.textAlign = 'left';
    }
  }

  private drawTacho(rpm: number, kph: number, gear: number) {
    const g = this.tacho;
    const c = g.canvas;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    // The stylesheet decides how big the dial is; everything below is drawn in a
    // fixed 150-unit space and scaled onto whatever that turns out to be.
    const size = c.clientWidth || 150;
    const px = Math.round(size * dpr);
    if (c.width !== px) {
      c.width = px;
      c.height = px;
    }
    const s = px / 150;
    g.setTransform(s, 0, 0, s, 0, 0);
    g.clearRect(0, 0, 150, 150);

    const cx = 75;
    const cy = 75;
    const r = 58;
    const a0 = Math.PI * 0.78;
    const a1 = Math.PI * 2.22;

    g.lineCap = 'round';
    g.strokeStyle = 'rgba(255,255,255,0.07)';
    g.lineWidth = 9;
    g.beginPath();
    g.arc(cx, cy, r, a0, a1);
    g.stroke();

    // Ticks.
    g.strokeStyle = 'rgba(150,190,230,0.35)';
    g.lineWidth = 1.2;
    for (let i = 0; i <= 8; i++) {
      const a = a0 + ((a1 - a0) * i) / 8;
      const inner = i % 2 === 0 ? r - 13 : r - 9;
      g.beginPath();
      g.moveTo(cx + Math.cos(a) * inner, cy + Math.sin(a) * inner);
      g.lineTo(cx + Math.cos(a) * (r - 6), cy + Math.sin(a) * (r - 6));
      g.stroke();
    }

    const t = Math.max(0, Math.min(1, rpm));
    const aNow = a0 + (a1 - a0) * t;
    if (!this.tachoGrad) {
      const grad = g.createLinearGradient(0, 0, 150, 150);
      grad.addColorStop(0, '#38e8ff');
      grad.addColorStop(0.7, '#ffc857');
      grad.addColorStop(1, '#ff4d9d');
      this.tachoGrad = grad;
    }
    g.strokeStyle = this.tachoGrad;
    g.lineWidth = 9;
    g.shadowColor = t > 0.85 ? '#ff4d9d' : '#38e8ff';
    g.shadowBlur = 16;
    g.beginPath();
    g.arc(cx, cy, r, a0, aNow);
    g.stroke();
    g.shadowBlur = 0;

    // Needle.
    g.strokeStyle = '#fff';
    g.lineWidth = 2;
    g.beginPath();
    g.moveTo(cx + Math.cos(aNow) * 14, cy + Math.sin(aNow) * 14);
    g.lineTo(cx + Math.cos(aNow) * (r - 4), cy + Math.sin(aNow) * (r - 4));
    g.stroke();

    // On a small dial the centre stack is unreadable, and the panel spells the
    // same numbers out beside it anyway — so let the dial be just a dial.
    if (size >= 100) {
      g.textAlign = 'center';
      g.fillStyle = 'rgba(107,127,153,0.9)';
      g.font = '9px ui-monospace, monospace';
      g.fillText('RPM', cx, cy + 30);
      g.fillStyle = '#fff';
      g.font = '600 15px ui-monospace, monospace';
      g.fillText(`${Math.round(kph)}`, cx, cy + 6);
      g.fillStyle = '#ffc857';
      g.font = '10px ui-monospace, monospace';
      g.fillText(gearLabel(gear), cx, cy + 18);
      g.textAlign = 'left';
    }
  }

  private drawLeaderboard(rows: LeaderRow[]) {
    const key = rows.map((r) => `${r.rank}${r.name}${r.bestLap.toFixed(2)}${r.mine}${r.color}`).join('|');
    if (key === this.lastLeaderboard) return;
    this.lastLeaderboard = key;

    const ol = $('leaderboard');
    if (rows.length === 0) {
      const li = document.createElement('li');
      li.className = 'empty';
      li.textContent = 'no laps yet';
      ol.replaceChildren(li);
      return;
    }

    ol.replaceChildren(
      ...rows.map((r) => {
        const li = document.createElement('li');
        const flags = [r.mine && 'me', r.cut && 'cut', r.color === null && 'gone'];
        li.className = flags.filter(Boolean).join(' ');
        const pos = document.createElement('span');
        pos.textContent = `${r.rank}`;
        const sw = document.createElement('span');
        sw.className = 'swatch';
        if (r.color !== null) sw.style.background = `#${r.color.toString(16).padStart(6, '0')}`;
        const nm = document.createElement('span');
        nm.textContent = r.name;
        const t = document.createElement('b');
        t.textContent = fmtTime(r.bestLap);
        li.append(pos, sw, nm, t);
        return li;
      }),
    );
  }
}

/** A device pixel ratio without the trailing zeroes: 2, 1.5, 1.25. */
function trim(n: number): string {
  return n.toFixed(2).replace(/\.?0+$/, '');
}

export /** What the gearbox is in. `-1` is reverse on the wire; nobody wants to read
 *  "G-1" through a corner. */
function gearLabel(gear: number): string {
  if (gear < 0) return 'R';
  if (gear < 1) return 'N';
  return `G${Math.round(gear)}`;
}

function fmtTime(seconds: number): string {
  if (!isFinite(seconds) || seconds <= 0) return '0:00.00';
  const m = Math.floor(seconds / 60);
  const s = seconds - m * 60;
  return `${m}:${s.toFixed(2).padStart(5, '0')}`;
}
