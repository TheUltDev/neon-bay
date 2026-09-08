// The mixer. Three faders -- master, game effects, music -- over one
// AudioContext, with the engine synth on the effects bus and the keygen radio
// on the music bus. Nothing starts until the browser has seen a gesture, so the
// whole graph is built on the first `start()`.

import { KeygenMusic, stationInfo, wrapStation, type StationInfo } from './music';

export type Bus = 'master' | 'sfx' | 'music';

export const BUSES: Bus[] = ['master', 'sfx', 'music'];

const DEFAULTS: Record<Bus, number> = { master: 0.9, sfx: 0.85, music: 0.6 };
const STORE_KEY = 'audio-mix';
const STATION_KEY = 'audio-station';

/** Faders are linear; ears are not. Squaring is the cheap fix. */
const curve = (v: number) => (v <= 0 ? 0 : v * v);

function loadMix(): Record<Bus, number> {
  const mix = { ...DEFAULTS };
  try {
    const saved = JSON.parse(localStorage.getItem(STORE_KEY) ?? '{}') as Partial<Record<Bus, number>>;
    for (const bus of BUSES) {
      const v = saved[bus];
      if (typeof v === 'number' && v >= 0 && v <= 1) mix[bus] = v;
    }
  } catch {
    // A corrupt entry is not worth failing the page over.
  }
  return mix;
}

/** Which station the dial was left on last time. */
function loadStation(): number {
  try {
    return wrapStation(Number(localStorage.getItem(STATION_KEY)) || 0);
  } catch {
    return 0;
  }
}

export class GameAudio {
  private ctx: AudioContext | null = null;
  private gains = new Map<Bus, GainNode>();
  private engineSynth: EngineSynth | null = null;
  private music: KeygenMusic | null = null;
  private premute = DEFAULTS.master;
  readonly volume = loadMix();
  station = loadStation();
  running = false;
  muted = false;

  /** Build the graph and start the music. Must be called from a user gesture. */
  async start() {
    if (this.ctx) {
      await this.ctx.resume();
      this.running = true;
      return;
    }
    const ctx = new AudioContext();
    this.ctx = ctx;

    const master = ctx.createGain();
    master.connect(ctx.destination);
    for (const bus of ['sfx', 'music'] as const) {
      const g = ctx.createGain();
      g.connect(master);
      this.gains.set(bus, g);
    }
    this.gains.set('master', master);
    for (const bus of BUSES) this.apply(bus);

    this.engineSynth = new EngineSynth(ctx, this.gains.get('sfx')!);
    this.music = new KeygenMusic(ctx, this.gains.get('music')!, this.station);
    this.music.start();
    this.running = true;
  }

  setVolume(bus: Bus, v: number) {
    this.volume[bus] = Math.max(0, Math.min(1, v));
    if (bus === 'master' && this.volume.master > 0) this.muted = false;
    this.apply(bus);
    try {
      localStorage.setItem(STORE_KEY, JSON.stringify(this.volume));
    } catch {
      // Private browsing. The mix still works, it just will not be remembered.
    }
  }

  /**
   * Turn the radio dial. Works before the graph exists -- the station is just a
   * number until then, and the tracker is handed it when the audio starts.
   */
  setStation(index: number): StationInfo {
    this.station = wrapStation(index);
    this.music?.setStation(this.station);
    try {
      localStorage.setItem(STATION_KEY, String(this.station));
    } catch {
      // Same deal as the mix: not worth failing over.
    }
    return this.nowPlaying();
  }

  /** What the radio strip shows. */
  nowPlaying(): StationInfo {
    return stationInfo(this.station);
  }

  /** Whether the music bus can actually be heard right now. */
  get musicAudible(): boolean {
    return this.running && !this.muted && this.volume.master > 0 && this.volume.music > 0;
  }

  /** Returns the master level to show on the slider afterwards. */
  toggleMute(): number {
    if (this.muted) {
      const back = this.premute > 0 ? this.premute : DEFAULTS.master;
      this.setVolume('master', back);
    } else {
      this.premute = this.volume.master;
      this.setVolume('master', 0);
      this.muted = true;
    }
    return this.volume.master;
  }

  /** Park the audio clock while the tab is hidden; the tracker resumes in step. */
  setHidden(hidden: boolean) {
    if (!this.ctx || !this.running) return;
    void (hidden ? this.ctx.suspend() : this.ctx.resume());
  }

  /** `rpm` 0..1.15, `load` 0..1, `slide` 0..1. */
  engine(rpm: number, load: number, slide: number) {
    this.engineSynth?.update(rpm, load, slide);
  }

  private apply(bus: Bus) {
    const node = this.gains.get(bus);
    if (!node || !this.ctx) return;
    node.gain.setTargetAtTime(curve(this.volume[bus]), this.ctx.currentTime, 0.02);
  }
}

// -------------------------------------------------------------- engine note --

// Two detuned saws through a low-pass whose cutoff tracks load, plus a noise bed
// for tire scrub.
class EngineSynth {
  private readonly ctx: AudioContext;
  private readonly osc: OscillatorNode[] = [];
  private readonly gain: GainNode;
  private readonly filter: BiquadFilterNode;
  private readonly scrub: GainNode;

  constructor(ctx: AudioContext, out: AudioNode) {
    this.ctx = ctx;

    this.gain = ctx.createGain();
    this.gain.gain.value = 0;
    this.filter = ctx.createBiquadFilter();
    this.filter.type = 'lowpass';
    this.filter.frequency.value = 700;
    this.filter.Q.value = 3;
    this.filter.connect(this.gain);
    this.gain.connect(out);

    for (const detune of [0, 7, -5]) {
      const o = ctx.createOscillator();
      o.type = 'sawtooth';
      o.frequency.value = 60;
      o.detune.value = detune;
      o.connect(this.filter);
      o.start();
      this.osc.push(o);
    }

    // Tire scrub: filtered white noise.
    const len = ctx.sampleRate * 2;
    const buf = ctx.createBuffer(1, len, ctx.sampleRate);
    const data = buf.getChannelData(0);
    for (let i = 0; i < len; i++) data[i] = Math.random() * 2 - 1;
    const noise = ctx.createBufferSource();
    noise.buffer = buf;
    noise.loop = true;
    const nf = ctx.createBiquadFilter();
    nf.type = 'bandpass';
    nf.frequency.value = 2400;
    nf.Q.value = 0.8;
    this.scrub = ctx.createGain();
    this.scrub.gain.value = 0;
    noise.connect(nf).connect(this.scrub).connect(out);
    noise.start();
  }

  update(rpm: number, load: number, slide: number) {
    const t = this.ctx.currentTime;
    const f = 42 + rpm * 118;
    for (const o of this.osc) o.frequency.setTargetAtTime(f, t, 0.03);
    this.filter.frequency.setTargetAtTime(420 + rpm * 2100 + load * 900, t, 0.05);
    this.gain.gain.setTargetAtTime(0.05 + load * 0.06, t, 0.08);
    this.scrub.gain.setTargetAtTime(slide * 0.07, t, 0.05);
  }
}
