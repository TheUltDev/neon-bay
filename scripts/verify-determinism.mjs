// Cross-checks the wasm physics build against the native one, bit for bit.
//
//   node scripts/verify-determinism.mjs
//
// Both sides run the same integer-derived input script (so the *driver* is
// identical and only the simulation is under test) and print the raw f32 bits
// of two cars. Any difference means client prediction would drift away from the
// sidecar on its own, and every corner would need a visible correction.

import { execFileSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const WASM = join(root, 'target/wasm32-unknown-unknown/release/physics.wasm');

function scripted(tick) {
  return [
    Math.floor(tick / 37) % 3 === 0 ? 0.0 : 1.0,
    ((tick % 240) - 120) / 120,
    Math.floor(tick / 53) % 5 === 0 ? 0.5 : 0.0,
    Math.floor(tick / 97) % 7 === 0 ? 1.0 : 0.0,
  ];
}

function runWasm() {
  const e = new WebAssembly.Instance(new WebAssembly.Module(readFileSync(WASM)), {}).exports;
  e.phys_init();
  e.phys_spawn(0, 0);
  e.phys_spawn(1, 3);
  const stride = e.phys_car_floats();
  const cars = new Float32Array(e.memory.buffer, e.phys_cars_ptr(), e.phys_max_cars() * stride);
  const inputs = new Float32Array(e.memory.buffer, e.phys_inputs_ptr(), e.phys_max_cars() * 4);
  const bits = new Uint32Array(cars.buffer, cars.byteOffset, cars.length);
  const out = [];
  const row = (tick) => {
    for (let i = 0; i < 2; i++) {
      const b = i * stride;
      const hex = [0, 1, 2, 3, 4, 5].map((k) => bits[b + k].toString(16).padStart(8, '0'));
      out.push(`t${String(tick).padStart(4, '0')} car${i} ${hex.join(' ')}`);
    }
  };
  for (let tick = 0; tick < 1200; tick++) {
    inputs.set(scripted(tick), 0);
    inputs.set(scripted(tick + 511), 4);
    e.phys_step(0b11);
    if (tick % 300 === 299) row(tick);
  }
  row(1199);
  return out;
}

const native = execFileSync(
  'cargo',
  ['run', '-p', 'physics', '--release', '--quiet', '--example', 'trace'],
  { cwd: root, encoding: 'utf8' },
)
  .trim()
  .split(/\r?\n/);
const wasm = runWasm();

let bad = 0;
for (let i = 0; i < Math.max(native.length, wasm.length); i++) {
  const same = native[i] === wasm[i];
  if (!same) bad++;
  console.log(`${same ? 'ok  ' : 'DIFF'} native ${native[i] ?? '-'}\n     wasm   ${wasm[i] ?? '-'}`);
}

if (bad) {
  console.error(`\n${bad} of ${native.length} rows differ -- prediction will drift.`);
  process.exit(1);
}
console.log(`\nAll ${native.length} checkpoints identical, bit for bit.`);
console.log('x86-64 and wasm32 agree, so a healthy client predicts with zero error.');
