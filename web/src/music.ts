// Keygen radio: a handful of stations, all synthesized. No sample, no download
// -- a tiny tracker running on four channels (octave-jumping bass, a pulse-wave
// arpeggio, a detuned saw lead through a delay, and drums built out of noise
// bursts) with a different song loaded per station.
//
// Each song is written the way a MOD is: patterns of rows, an order list that
// stitches them into a song, and a scheduler that walks it. Rows are sixteenth
// notes; a pattern is two bars, one chord each. A station carries its own tempo
// and voicing on top of that -- pulse duty, delay length, swing -- so the
// stations differ in more than their notes.

const ROWS_PER_BAR = 16;
const ROWS_PER_PART = 32;

/** How far ahead of the audio clock rows are queued, in seconds. */
const LOOKAHEAD = 0.25;

/** Sixteenth notes: four rows to the beat. */
const rowDur = (bpm: number) => 60 / bpm / 4;

// ------------------------------------------------------------------ notes --

const SEMITONE: Record<string, number> = {
  c: 0, 'c#': 1, d: 2, 'd#': 3, e: 4, f: 5, 'f#': 6, g: 7, 'g#': 8, a: 9, 'a#': 10, b: 11,
};

/** `a4` -> 69, `g#3` -> 56. Middle C is `c4`. */
function midi(token: string): number {
  const m = /^([a-g]#?)(\d)$/.exec(token);
  if (!m) throw new Error(`bad note: ${token}`);
  return (Number(m[2]) + 1) * 12 + SEMITONE[m[1]];
}

const hz = (note: number) => 440 * 2 ** ((note - 69) / 12);

/** Split a written line into one token per row, spelling checked on the spot. */
function notes(src: string): string[] {
  const out = src.trim().split(/\s+/);
  if (out.length !== ROWS_PER_PART) {
    throw new Error(`lead line has ${out.length} rows, expected ${ROWS_PER_PART}`);
  }
  for (const token of out) if (token !== '.') midi(token);
  return out;
}

/** One character per row, so a miscounted pattern says so instead of drifting. */
function check<T extends Record<string, string>>(table: T, len: number): T {
  for (const [name, pattern] of Object.entries(table)) {
    if (pattern.length !== len) {
      throw new Error(`pattern ${name} "${pattern}" has ${pattern.length} rows, expected ${len}`);
    }
  }
  return table;
}

/** A typo in an order list should not survive to playback. */
function lookup<T>(table: Record<string, T>, key: string, kind: string): T {
  const found = table[key];
  if (!found) throw new Error(`unknown ${kind}: ${key}`);
  return found;
}

// ------------------------------------------------------------- song pieces --

/**
 * Chords voiced close together so the arpeggio stays in one register. The first
 * tone is the root: the bass line reads it as such.
 */
const CHORDS: Record<string, number[]> = {
  Am: [57, 60, 64], // a3 c4 e4
  F: [53, 57, 60], // f3 a3 c4
  C: [60, 64, 67], // c4 e4 g4
  G: [55, 59, 62], // g3 b3 d4
  E: [52, 56, 59], // e3 g#3 b3
  Dm: [50, 53, 57], // d3 f3 a3
  Cm: [60, 63, 67], // c4 d#4 g4
  Fm: [53, 56, 60], // f3 g#3 c4
  Ab: [56, 60, 63], // g#3 c4 d#4
  Eb: [51, 55, 58], // d#3 g3 a#3
  Bb: [58, 62, 65], // a#3 d4 f4
  // Sevenths give the arp a fourth step and the bass a colour tone to sit on.
  Am7: [57, 60, 64, 67],
  Dm7: [50, 53, 57, 60],
  Em7: [52, 55, 59, 62],
  G7: [55, 59, 62, 65],
  Cmaj7: [60, 64, 67, 71],
  Fmaj7: [53, 57, 60, 64],
};

/** Arp index into a chord, wrapping upward an octave at a time. */
const arpNote = (chord: number[], i: number) =>
  chord[i % chord.length] + 12 * ((i / chord.length) | 0);

/** Bass sits an octave under the arp. `8` is the octave jump that makes it walk. */
const BASS_TONE: Record<string, (c: number[]) => number> = {
  '1': (c) => c[0] - 12,
  '3': (c) => c[1] - 12,
  '5': (c) => c[2] - 12,
  '7': (c) => (c[3] ?? c[2]) - 12,
  '8': (c) => c[0],
};

// One bar each, replayed under whichever chord the row lands on.
const BASS: Record<string, string> = check({
  intro: '1.......1.....5.',
  drive: '1...1...1.8.1.5.',
  push: '1.1.8.1.1.5.8.5.',
  pump: '1.1.1.1.1.1.1.1.',
  walk: '1...5...8...5...',
  octave: '1...8...1...8...',
  roll: '1.11.1.11.1.5.8.',
  sub: '1.......7.......',
}, ROWS_PER_BAR);

const ARPS: Record<string, string> = check({
  sparse: '0.1.2.3.4.3.2.1.',
  steady: '0123012301230123',
  updown: '0123432101234321',
  wide: '0.2.4.2.6.4.2.0.',
  climb: '0123456765432101',
  spark: '4.3.2.1.0.1.2.3.',
  pulse: '0.0.2.2.4.4.2.2.',
}, ROWS_PER_BAR);

interface Kit {
  k: string;
  s: string;
  h: string;
}

const SILENT = '.'.repeat(ROWS_PER_PART);
const kit = (k: string, s: string, h: string): Kit => check({ k, s, h }, ROWS_PER_PART);

//                    beat 1   2   3   4   1   2   3   4
const KITS: Record<string, Kit> = {
  off: kit(SILENT, SILENT, SILENT),
  hats: kit(SILENT, SILENT, '..x...x...x...x...x...x...x...x.'),
  light: kit(
    'x.......x.......x.......x.......',
    '............................x...',
    '..x...x...x...x...x...x...x...x.',
  ),
  full: kit(
    'x..x....x.......x..x....x...x...',
    '....x.......x.......x.......x..x',
    'x.x.x.x.x.x.x.x.x.x.x.x.x.x.xxxo',
  ),
  fill: kit(
    'x.......x.......x.......x.......',
    '....x.......x.......x...x.x.xxxx',
    'x.x.x.x.x.x.x.x.x.x.x.x.........',
  ),
  four: kit(
    'x...x...x...x...x...x...x...x...',
    '....x.......x.......x.......x...',
    '..x...x...x...x...x...x...x...x.',
  ),
  punch: kit(
    'x.....x...x.....x.....x...x.....',
    '....x.......x.......x.......x...',
    'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxo',
  ),
  // Sparse enough to leave a wide-open delay some room to breathe.
  soft: kit(
    'x.............x.x.............x.',
    '........x...............x.......',
    '..x...x...x...x...x...x...x...x.',
  ),
  // Hats in threes rather than fours, which is what the swing leans into.
  shuffle: kit(
    'x.......x...x...x.......x.......',
    '........x...............x.......',
    'x..x..x.x..x..x.x..x..x.x..x..x.',
  ),
};

interface Part {
  chords: number[][];
  bass: string;
  arp: string | null;
  drums: Kit;
  lead: number;
}

const part = (chords: string, bass: string, arp: string | null, drums: string, lead = -1): Part => {
  const voiced = chords.trim().split(/\s+/).map((c) => lookup(CHORDS, c, 'chord'));
  if (voiced.length !== 2) throw new Error(`part "${chords}" needs two chords, one per bar`);
  return {
    chords: voiced,
    bass: lookup(BASS, bass, 'bass line'),
    arp: arp ? lookup(ARPS, arp, 'arp') : null,
    drums: lookup(KITS, drums, 'kit'),
    lead,
  };
};

// ---------------------------------------------------------------- stations --

/** What a station changes about the synth itself, on top of its notes. */
interface Tone {
  bpm: number;
  /** Duty cycle of the arp's pulse wave: narrow is nasal, 0.5 is hollow. */
  duty: number;
  /** Delay length in rows. 3 is the classic dotted eighth. */
  delayRows: number;
  delayFeedback: number;
  /** How dark the repeats get, in Hz. */
  delayDamp: number;
  /** Lead vibrato depth, in cents. */
  vibrato: number;
  /** Cutoff the lead's filter opens to, in Hz. */
  leadOpen: number;
  /** How long the bass rings, in rows. */
  bassRing: number;
  /** Fraction of a row that odd rows arrive late. 0 is straight sixteenths. */
  swing: number;
  /** Level for the station as a whole, so a busy song does not shout. */
  level: number;
}

const BASE_TONE: Tone = {
  bpm: 140,
  duty: 0.25,
  delayRows: 3,
  delayFeedback: 0.33,
  delayDamp: 2600,
  vibrato: 7,
  leadOpen: 3200,
  bassRing: 1.7,
  swing: 0,
  level: 0.9,
};

interface Station extends Tone {
  name: string;
  leads: string[][];
  order: Part[];
}

const station = (
  name: string,
  tone: Partial<Tone> & { bpm: number },
  leads: string[][],
  order: Part[],
): Station => {
  for (const p of order) {
    if (p.lead >= 0 && !leads[p.lead]) throw new Error(`${name}: no lead line ${p.lead}`);
  }
  return { name, leads, order, ...BASE_TONE, ...tone };
};

const STATIONS: Station[] = [
  // The one this demo shipped with: eight bars over Am F C G / Am F C E, the
  // second half climbing an octave so the loop has somewhere to go.
  station(
    'HARD-WIRED',
    { bpm: 140 },
    [
      notes(`a4 .  .  .  c5 .  b4 .  a4 .  .  .  .  e4 .  .
             f4 .  .  .  a4 .  c5 .  a4 .  .  .  .  .  .  .`),
      notes(`g4 .  .  .  e5 .  d5 .  c5 .  .  .  .  g4 .  .
             d5 .  .  .  b4 .  d5 .  g4 .  .  .  .  .  .  .`),
      notes(`a4 .  c5 .  e5 .  .  .  a5 .  .  .  g5 .  e5 .
             f5 .  .  .  e5 .  c5 .  a4 .  .  .  .  c5 .  .`),
      notes(`e5 .  .  .  g5 .  e5 .  c5 .  b4 .  c5 .  d5 .
             e5 .  .  .  d5 .  b4 .  g#4 .  .  .  b4 .  .  .`),
    ],
    // Intro, hook, breakdown, hook again, outro -- then back to the top.
    [
      part('Am F', 'intro', 'sparse', 'off'),
      part('C  G', 'intro', 'sparse', 'off'),
      part('Am F', 'drive', 'sparse', 'hats'),
      part('C  E', 'drive', 'steady', 'fill'),

      part('Am F', 'drive', 'steady', 'full', 0),
      part('C  G', 'drive', 'steady', 'full', 1),
      part('Am F', 'drive', 'steady', 'full', 2),
      part('C  E', 'drive', 'steady', 'full', 3),

      part('Am F', 'intro', 'updown', 'light'),
      part('C  E', 'drive', 'updown', 'fill'),

      part('Am F', 'push', 'updown', 'full', 0),
      part('C  G', 'push', 'updown', 'full', 1),
      part('Am F', 'push', 'updown', 'full', 2),
      part('C  E', 'push', 'updown', 'full', 3),

      part('Am F', 'drive', 'steady', 'light'),
      part('C  E', 'intro', 'sparse', 'off'),
    ],
  ),

  // Major, fast and thin: the tune scrolling behind a 1994 cracktro.
  station(
    'CRACKTRO BLUE',
    {
      bpm: 168, duty: 0.125, delayFeedback: 0.28, delayDamp: 3400,
      vibrato: 5, leadOpen: 4200, bassRing: 1.3, level: 0.86,
    },
    [
      notes(`c5 .  e5 .  g5 .  .  .  e5 .  g5 .  a5 .  g5 .
             b4 .  d5 .  g5 .  .  .  d5 .  b4 .  g4 .  .  .`),
      notes(`a4 .  c5 .  e5 .  .  .  c5 .  e5 .  a5 .  .  .
             f5 .  e5 .  c5 .  a4 .  c5 .  .  .  a4 .  .  .`),
      notes(`g5 .  .  .  e5 .  g5 .  c6 .  .  .  b5 .  g5 .
             d5 .  g5 .  b5 .  .  .  a5 .  g5 .  d5 .  b4 .`),
      notes(`f5 .  a5 .  c6 .  a5 .  f5 .  .  .  e5 .  c5 .
             g5 .  b5 .  d6 .  b5 .  g5 .  .  .  f5 .  d5 .`),
    ],
    [
      part('C  G', 'intro', 'sparse', 'off'),
      part('Am F', 'intro', 'sparse', 'hats'),
      part('C  G', 'pump', 'steady', 'light'),
      part('F  G', 'pump', 'climb', 'fill'),

      part('C  G', 'pump', 'steady', 'four', 0),
      part('Am F', 'pump', 'steady', 'four', 1),
      part('C  G', 'pump', 'climb', 'four', 2),
      part('F  G', 'push', 'climb', 'fill', 3),

      part('Am F', 'walk', 'wide', 'light'),
      part('C  G', 'walk', 'wide', 'full'),

      part('C  G', 'push', 'climb', 'full', 0),
      part('Am F', 'push', 'climb', 'full', 1),
      part('C  G', 'push', 'wide', 'full', 2),
      part('F  G', 'push', 'wide', 'fill', 3),
    ],
  ),

  // Slow, seventh chords, a quarter-note delay left wide open. The one to leave
  // on while you are still learning the track.
  station(
    'NIGHT DRIVE',
    {
      bpm: 104, duty: 0.5, delayRows: 4, delayFeedback: 0.42, delayDamp: 1800,
      vibrato: 11, leadOpen: 2400, bassRing: 2.6, level: 0.92,
    },
    [
      notes(`e4 .  .  .  .  .  a4 .  .  .  .  .  g4 .  .  .
             a4 .  .  .  .  .  c5 .  .  .  .  .  .  .  .  .`),
      notes(`g4 .  .  .  b4 .  .  .  c5 .  .  .  .  .  e5 .
             d5 .  .  .  .  .  b4 .  .  .  g4 .  .  .  .  .`),
      notes(`f4 .  .  .  a4 .  .  .  d5 .  .  .  c5 .  a4 .
             b4 .  .  .  e5 .  .  .  d5 .  b4 .  .  .  .  .`),
      notes(`c5 .  .  .  .  .  a4 .  f4 .  .  .  .  .  .  .
             g#4 .  .  .  b4 .  .  .  e5 .  .  .  b4 .  .  .`),
    ],
    [
      part('Am7 Fmaj7', 'sub', 'sparse', 'off'),
      part('Cmaj7 G7', 'sub', 'sparse', 'hats'),

      part('Am7 Fmaj7', 'walk', 'sparse', 'soft', 0),
      part('Cmaj7 G7', 'walk', 'sparse', 'soft', 1),
      part('Dm7 Em7', 'walk', 'pulse', 'light', 2),
      part('Fmaj7 E', 'walk', 'pulse', 'light', 3),

      part('Am7 Fmaj7', 'octave', 'wide', 'light'),
      part('Cmaj7 G7', 'octave', 'wide', 'soft'),

      part('Dm7 Em7', 'walk', 'sparse', 'soft', 2),
      part('Fmaj7 E', 'sub', 'sparse', 'off', 3),
    ],
  ),

  // Andalusian cadence, sixteenth-note bass rolls, drums that never let up.
  station(
    'OVERDRIVE',
    {
      bpm: 172, delayRows: 2, delayFeedback: 0.24, delayDamp: 3000,
      vibrato: 9, leadOpen: 4600, bassRing: 1.2, level: 0.85,
    },
    [
      notes(`a4 .  b4 .  c5 .  b4 .  a4 .  .  .  e5 .  .  .
             d5 .  b4 .  g4 .  b4 .  d5 .  g5 .  .  .  .  .`),
      notes(`f5 .  e5 .  c5 .  a4 .  f5 .  .  .  e5 .  .  .
             e5 .  .  .  g#4 .  b4 .  e5 .  d5 .  b4 .  g#4 .`),
      notes(`a5 .  .  .  g5 .  e5 .  a5 .  .  .  c6 .  b5 .
             g5 .  d5 .  b4 .  d5 .  g5 .  b5 .  d6 .  .  .`),
      notes(`c6 .  a5 .  f5 .  a5 .  c6 .  .  .  a5 .  f5 .
             b5 .  g#5 .  e5 .  g#5 .  b5 .  e6 .  b5 .  g#5 .`),
    ],
    [
      part('Am G', 'octave', 'steady', 'hats'),
      part('F  E', 'octave', 'steady', 'fill'),

      part('Am G', 'roll', 'steady', 'punch', 0),
      part('F  E', 'roll', 'steady', 'punch', 1),
      part('Am G', 'roll', 'climb', 'punch', 2),
      part('F  E', 'roll', 'climb', 'fill', 3),

      part('Dm E', 'push', 'spark', 'full'),
      part('Am E', 'push', 'spark', 'full'),

      part('Am G', 'roll', 'climb', 'punch', 2),
      part('F  E', 'roll', 'spark', 'fill', 3),
    ],
  ),

  // C minor with a limp in it: swung sixteenths under a narrow pulse, which is
  // about as close to a SID as a plain oscillator gets.
  station(
    'SECTOR SHUFFLE',
    {
      bpm: 124, duty: 0.125, swing: 0.17, delayFeedback: 0.3, delayDamp: 2200,
      vibrato: 6, leadOpen: 2900, bassRing: 1.5,
    },
    [
      notes(`c5 .  d#5 .  g5 .  .  .  d#5 .  d5 .  c5 .  .  .
             g#4 .  c5 .  d#5 .  c5 .  g#4 .  .  .  .  .  .  .`),
      notes(`d#5 .  g5 .  a#5 .  g5 .  d#5 .  .  .  d5 .  .  .
             d5 .  f5 .  a#5 .  f5 .  d5 .  a#4 .  .  .  .  .`),
      notes(`g5 .  .  .  d#5 .  c5 .  d5 .  d#5 .  g5 .  a#5 .
             c6 .  .  .  g#5 .  f5 .  c5 .  .  .  g#4 .  .  .`),
      notes(`d5 .  .  .  b4 .  d5 .  g5 .  .  .  f5 .  d5 .
             b4 .  d5 .  f5 .  d5 .  b4 .  g4 .  .  .  .  .`),
    ],
    [
      part('Cm Ab', 'intro', 'sparse', 'hats'),
      part('Eb Bb', 'intro', 'sparse', 'light'),

      part('Cm Ab', 'walk', 'pulse', 'shuffle', 0),
      part('Eb Bb', 'walk', 'pulse', 'shuffle', 1),
      part('Cm Fm', 'drive', 'steady', 'full', 2),
      part('G  G', 'drive', 'steady', 'fill', 3),

      part('Cm Ab', 'push', 'wide', 'full', 0),
      part('Eb Bb', 'push', 'wide', 'full', 1),
      part('Cm Fm', 'push', 'spark', 'full', 2),
      part('G  G', 'roll', 'spark', 'fill', 3),

      part('Cm Ab', 'walk', 'sparse', 'light'),
      part('Eb Bb', 'sub', 'sparse', 'off'),
    ],
  ),
];

export interface StationInfo {
  index: number;
  count: number;
  name: string;
  bpm: number;
}

/** The dial wraps in both directions, so prev/next never runs out of stations. */
export const wrapStation = (i: number) => ((i % STATIONS.length) + STATIONS.length) % STATIONS.length;

export function stationInfo(index: number): StationInfo {
  const i = wrapStation(index);
  return { index: i, count: STATIONS.length, name: STATIONS[i].name, bpm: STATIONS[i].bpm };
}

// ------------------------------------------------------------------ synth --

/** Fourier series for a pulse train, which is where the chip buzz comes from. */
function pulseWave(ctx: AudioContext, duty: number): PeriodicWave {
  const n = 32;
  const real = new Float32Array(n);
  const imag = new Float32Array(n);
  for (let k = 1; k < n; k++) real[k] = (2 / (k * Math.PI)) * Math.sin(Math.PI * k * duty);
  return ctx.createPeriodicWave(real, imag);
}

/** Attack, hold, exponential tail: a sampled one-shot in three moves. */
function shape(p: AudioParam, t: number, peak: number, attack: number, hold: number, decay: number) {
  p.setValueAtTime(0.0001, t);
  p.linearRampToValueAtTime(peak, t + attack);
  p.setValueAtTime(peak, t + attack + hold);
  p.exponentialRampToValueAtTime(0.0001, t + attack + hold + decay);
}

/** How long a lead note rings: until the next event on the line. */
function gate(line: string[], row: number): number {
  for (let i = row + 1; i < line.length; i++) if (line[i] !== '.') return i - row;
  return Math.min(8, line.length - row);
}

/**
 * One station's voice bus and its delay line. Switching stations builds a new
 * rig and fades the old one out, so the tail of the outgoing song never
 * inherits the tempo of the incoming one.
 */
interface Rig {
  voices: GainNode;
  send: GainNode;
}

export class KeygenMusic {
  private readonly ctx: AudioContext;
  private readonly mix: GainNode;
  private readonly noise: AudioBuffer;
  private readonly waves = new Map<number, PeriodicWave>();
  private readonly vibrato: GainNode;
  private station: Station;
  private stationIndex: number;
  private rig: Rig;
  private timer = 0;
  private rowIndex = 0;
  private nextRowTime = 0;
  playing = false;

  constructor(ctx: AudioContext, out: AudioNode, station = 0) {
    this.ctx = ctx;

    // Voices land on `mix`; the compressor keeps things glued when the lead,
    // the arp and a kick all land on the same row.
    this.mix = ctx.createGain();
    const comp = ctx.createDynamicsCompressor();
    comp.threshold.value = -14;
    comp.knee.value = 6;
    comp.ratio.value = 4;
    comp.attack.value = 0.004;
    comp.release.value = 0.18;
    // Drum transients outrun the compressor's attack, so trim after it rather
    // than before: this is what leaves the engine room on the master bus.
    const trim = ctx.createGain();
    trim.gain.value = 0.8;
    this.mix.connect(comp).connect(trim).connect(out);

    const len = ctx.sampleRate * 2;
    this.noise = ctx.createBuffer(1, len, ctx.sampleRate);
    const data = this.noise.getChannelData(0);
    for (let i = 0; i < len; i++) data[i] = Math.random() * 2 - 1;

    // One LFO for the whole radio, so lead notes share a vibrato phase.
    const lfo = ctx.createOscillator();
    lfo.frequency.value = 5.4;
    this.vibrato = ctx.createGain();
    lfo.connect(this.vibrato);
    lfo.start();

    this.stationIndex = wrapStation(station);
    this.station = STATIONS[this.stationIndex];
    this.vibrato.gain.value = this.station.vibrato;
    this.rig = this.buildRig(this.station);
  }

  start() {
    if (this.playing) return;
    this.playing = true;
    this.nextRowTime = this.ctx.currentTime + 0.08;
    this.timer = window.setInterval(this.schedule, 40);
    this.schedule();
  }

  stop() {
    this.playing = false;
    clearInterval(this.timer);
    this.timer = 0;
  }

  /**
   * Turn the dial. The change lands at the end of what is already queued: the
   * outgoing song plays out its scheduled rows into its own rig, which fades
   * away under a burst of tuning static while the next one starts from its
   * intro. Nothing overlaps, and no delay tail is left running at the wrong
   * tempo.
   */
  setStation(index: number): StationInfo {
    const i = wrapStation(index);
    if (i !== this.stationIndex) {
      const at = this.playing
        ? Math.max(this.nextRowTime, this.ctx.currentTime)
        : this.ctx.currentTime;
      const old = this.rig;
      old.voices.gain.setTargetAtTime(0.0001, at, 0.04);
      old.send.gain.setTargetAtTime(0.0001, at, 0.04);
      // Once it is silent the whole subgraph is unreachable, so it can go.
      window.setTimeout(() => {
        old.voices.disconnect();
        old.send.disconnect();
      }, 3000);

      this.stationIndex = i;
      this.station = STATIONS[i];
      this.vibrato.gain.value = this.station.vibrato;
      this.rig = this.buildRig(this.station);
      this.rowIndex = 0;
      this.nextRowTime = at;
      this.tuningStatic(at);
    }
    return stationInfo(i);
  }

  private buildRig(s: Station): Rig {
    const ctx = this.ctx;
    const voices = ctx.createGain();
    voices.gain.value = s.level;
    voices.connect(this.mix);

    // Delay damped in the feedback path, so repeats fade dark rather than hiss.
    const send = ctx.createGain();
    const delay = ctx.createDelay(1);
    delay.delayTime.value = rowDur(s.bpm) * s.delayRows;
    const damp = ctx.createBiquadFilter();
    damp.type = 'lowpass';
    damp.frequency.value = s.delayDamp;
    const fb = ctx.createGain();
    fb.gain.value = s.delayFeedback;
    send.connect(delay);
    delay.connect(damp).connect(fb).connect(delay);
    delay.connect(voices);
    return { voices, send };
  }

  /** A sweep of static across the seam, so the cut reads as a change of station. */
  private tuningStatic(t: number) {
    const n = this.burst(t);
    const bp = this.ctx.createBiquadFilter();
    bp.type = 'bandpass';
    bp.Q.value = 1.1;
    bp.frequency.setValueAtTime(800, t);
    bp.frequency.exponentialRampToValueAtTime(4400, t + 0.24);
    const g = this.ctx.createGain();
    shape(g.gain, t, 0.14, 0.012, 0.04, 0.18);
    n.connect(bp).connect(g).connect(this.mix);
    n.stop(t + 0.34);
  }

  private get rowDur(): number {
    return rowDur(this.station.bpm);
  }

  private wave(duty: number): PeriodicWave {
    let w = this.waves.get(duty);
    if (!w) {
      w = pulseWave(this.ctx, duty);
      this.waves.set(duty, w);
    }
    return w;
  }

  private schedule = () => {
    if (!this.playing) return;
    const until = this.ctx.currentTime + LOOKAHEAD;
    // A suspended context freezes currentTime, so this simply stops queueing.
    while (this.nextRowTime < until) {
      // Swing drags the off-sixteenths late without moving the grid itself.
      const late = this.rowIndex % 2 ? this.station.swing * this.rowDur : 0;
      this.playRow(this.rowIndex, this.nextRowTime + late);
      this.nextRowTime += this.rowDur;
      this.rowIndex++;
    }
  };

  private playRow(index: number, t: number) {
    const song = this.station;
    const p = song.order[((index / ROWS_PER_PART) | 0) % song.order.length];
    const row = index % ROWS_PER_PART;
    const col = row % ROWS_PER_BAR;
    const chord = p.chords[(row / ROWS_PER_BAR) | 0];

    const bass = BASS_TONE[p.bass[col]];
    if (bass) this.bass(t, bass(chord));

    if (p.arp) {
      const step = p.arp[col];
      if (step !== '.') this.arp(t, arpNote(chord, Number(step)), row % 2 ? 0.42 : -0.42);
    }

    if (p.lead >= 0) {
      const line = song.leads[p.lead];
      if (line[row] !== '.') this.lead(t, midi(line[row]), gate(line, row) * this.rowDur);
    }

    if (p.drums.k[row] === 'x') this.kick(t);
    if (p.drums.s[row] === 'x') this.snare(t);
    const h = p.drums.h[row];
    if (h === 'x' || h === 'o') this.hat(t, h === 'o');
  }

  // --- voices --------------------------------------------------------------

  private bass(t: number, note: number) {
    const f = hz(note);
    const ring = this.rowDur * this.station.bassRing;
    const g = this.ctx.createGain();
    const lp = this.ctx.createBiquadFilter();
    lp.type = 'lowpass';
    lp.Q.value = 7;
    lp.frequency.setValueAtTime(Math.min(6000, f * 16), t);
    lp.frequency.exponentialRampToValueAtTime(Math.max(140, f * 3), t + 0.11);
    lp.connect(g).connect(this.rig.voices);
    shape(g.gain, t, 0.34, 0.004, 0.02, ring);

    for (const detune of [0, -11]) {
      const o = this.ctx.createOscillator();
      if (detune === 0) o.setPeriodicWave(this.wave(0.5));
      else o.type = 'sawtooth';
      o.frequency.value = f;
      o.detune.value = detune;
      o.connect(lp);
      o.start(t);
      o.stop(t + ring + 0.1);
    }
  }

  private arp(t: number, note: number, pan: number) {
    const o = this.ctx.createOscillator();
    o.setPeriodicWave(this.wave(this.station.duty));
    o.frequency.value = hz(note);
    const g = this.ctx.createGain();
    // Hard-panned arp channels are an Amiga habit worth keeping.
    const pn = this.ctx.createStereoPanner();
    pn.pan.value = pan;
    o.connect(g).connect(pn);
    pn.connect(this.rig.voices);
    const send = this.ctx.createGain();
    send.gain.value = 0.22;
    pn.connect(send).connect(this.rig.send);
    shape(g.gain, t, 0.15, 0.003, 0.008, this.rowDur * 0.85);
    o.start(t);
    o.stop(t + this.rowDur * 1.2);
  }

  private lead(t: number, note: number, dur: number) {
    const f = hz(note);
    const g = this.ctx.createGain();
    const lp = this.ctx.createBiquadFilter();
    lp.type = 'lowpass';
    lp.Q.value = 1.2;
    lp.frequency.setValueAtTime(1400, t);
    lp.frequency.linearRampToValueAtTime(this.station.leadOpen, t + 0.09);
    const pn = this.ctx.createStereoPanner();
    pn.pan.value = 0.18;
    lp.connect(g).connect(pn);
    pn.connect(this.rig.voices);
    const send = this.ctx.createGain();
    send.gain.value = 0.34;
    pn.connect(send).connect(this.rig.send);
    shape(g.gain, t, 0.21, 0.014, Math.max(0.02, dur * 0.7), 0.16);

    for (const detune of [6, -7]) {
      const o = this.ctx.createOscillator();
      if (detune > 0) o.type = 'sawtooth';
      else o.setPeriodicWave(this.wave(0.5));
      o.frequency.value = f;
      o.detune.value = detune;
      this.vibrato.connect(o.detune);
      o.connect(lp);
      o.start(t);
      o.stop(t + dur + 0.2);
    }
  }

  private burst(t: number): AudioBufferSourceNode {
    const n = this.ctx.createBufferSource();
    n.buffer = this.noise;
    n.loop = true;
    n.start(t);
    return n;
  }

  private kick(t: number) {
    const o = this.ctx.createOscillator();
    o.frequency.setValueAtTime(190, t);
    o.frequency.exponentialRampToValueAtTime(47, t + 0.075);
    const g = this.ctx.createGain();
    g.gain.setValueAtTime(0.95, t);
    g.gain.exponentialRampToValueAtTime(0.0001, t + 0.24);
    o.connect(g).connect(this.rig.voices);
    o.start(t);
    o.stop(t + 0.26);

    const click = this.burst(t);
    const hp = this.ctx.createBiquadFilter();
    hp.type = 'highpass';
    hp.frequency.value = 2600;
    const cg = this.ctx.createGain();
    shape(cg.gain, t, 0.22, 0.001, 0, 0.02);
    click.connect(hp).connect(cg).connect(this.rig.voices);
    click.stop(t + 0.05);
  }

  private snare(t: number) {
    const n = this.burst(t);
    const bp = this.ctx.createBiquadFilter();
    bp.type = 'bandpass';
    bp.frequency.value = 1900;
    bp.Q.value = 0.6;
    const g = this.ctx.createGain();
    shape(g.gain, t, 0.42, 0.001, 0.01, 0.13);
    n.connect(bp).connect(g).connect(this.rig.voices);
    n.stop(t + 0.18);

    const body = this.ctx.createOscillator();
    body.type = 'triangle';
    body.frequency.setValueAtTime(210, t);
    body.frequency.exponentialRampToValueAtTime(150, t + 0.08);
    const bg = this.ctx.createGain();
    shape(bg.gain, t, 0.17, 0.001, 0.005, 0.08);
    body.connect(bg).connect(this.rig.voices);
    body.start(t);
    body.stop(t + 0.12);
  }

  private hat(t: number, open: boolean) {
    const n = this.burst(t);
    const hp = this.ctx.createBiquadFilter();
    hp.type = 'highpass';
    hp.frequency.value = 7400;
    const g = this.ctx.createGain();
    const decay = open ? 0.17 : 0.032;
    shape(g.gain, t, open ? 0.13 : 0.16, 0.001, 0, decay);
    n.connect(hp).connect(g).connect(this.rig.voices);
    n.stop(t + decay + 0.03);
  }
}
