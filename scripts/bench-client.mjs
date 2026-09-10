// What a browser tick costs, on the same wasm the browser runs.
//
//   node scripts/bench-client.mjs
//
// Two numbers, and they are not the same one:
//
// * the **forward step**, once per tick at 60 Hz;
// * the **rollback**, once per snapshot at 20 Hz, which replays however many
//   ticks of lead the client is holding.
//
// Both used to be one car's worth of work. Predicting rivals through the
// physics makes both of them the whole grid's, which is the cost the accuracy
// is bought with -- so it is worth being able to say what it is rather than
// guessing. The table prints one car and the full grid side by side.
//
// This is a desktop, and a desktop is not the machine to worry about. A
// mid-range phone runs wasm somewhere between three and eight times slower, so
// read the last column with that multiplier applied; the frame budget is the
// same 16.7 ms either way.

import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const WASM = join(root, 'target/wasm32-unknown-unknown/release/physics.wasm');

/** Snapshots per second, and so rollbacks per second: `SNAPSHOT_EVERY` = 3. */
const SNAPSHOT_HZ = 20;
/** Ticks a rollback replays. A 200 ms round trip plus the clock-sync margin. */
const LEAD = 12;
const TICK_HZ = 60;
const FRAME_MS = 1000 / 60;

const GRIDS = [1, 4, 8, 16, 24];
const WARMUP = 600;
const TICKS = 3000;

function load() {
  const e = new WebAssembly.Instance(new WebAssembly.Module(readFileSync(WASM)), {}).exports;
  e.phys_init();
  return e;
}

/** Put `n` cars on the grid, each steering to its own rhythm so they do not
 *  all follow one line and pile into the same barrier. */
function fill(e, n) {
  const inputs = new Float32Array(e.memory.buffer, e.phys_inputs_ptr(), e.phys_max_cars() * 4);
  for (let i = 0; i < n; i++) e.phys_spawn(i, i);
  const mask = n >= 32 ? -1 >>> 0 : (1 << n) - 1;
  e.phys_set_active(mask);
  return { inputs, mask };
}

function drive(inputs, n, tick) {
  for (let i = 0; i < n; i++) {
    const t = tick / TICK_HZ + i * 1.7;
    inputs[i * 4] = 0.85;
    inputs[i * 4 + 1] = Math.sin(t * 0.8) * 0.55;
    inputs[i * 4 + 2] = 0;
    inputs[i * 4 + 3] = 0;
  }
}

/** Microseconds per call, over `TICKS` of it. */
function timeStep(n) {
  const e = load();
  const { inputs, mask } = fill(e, n);
  for (let tick = 0; tick < WARMUP; tick++) {
    drive(inputs, n, tick);
    e.phys_step(mask);
  }
  // The inputs are written every tick in the real loop too, so they are inside
  // the measurement rather than hoisted out of it.
  const t0 = process.hrtime.bigint();
  for (let tick = 0; tick < TICKS; tick++) {
    drive(inputs, n, WARMUP + tick);
    e.phys_step(mask);
  }
  const t1 = process.hrtime.bigint();
  return Number(t1 - t0) / 1000 / TICKS;
}

/** Microseconds for one rollback: seed every car from a snapshot, then replay
 *  `LEAD` ticks of the whole grid. */
function timeRollback(n) {
  const e = load();
  const { inputs, mask } = fill(e, n);
  const stride = e.phys_car_floats();
  const cars = new Float32Array(e.memory.buffer, e.phys_cars_ptr(), e.phys_max_cars() * stride);
  const scratch = new Float32Array(e.memory.buffer, e.phys_scratch_ptr(), stride);
  const scratchPtr = e.phys_scratch_ptr();

  for (let tick = 0; tick < WARMUP; tick++) {
    drive(inputs, n, tick);
    e.phys_step(mask);
  }
  // A snapshot of the whole grid to rewind to, which is what actually arrives.
  const snap = cars.slice(0, n * stride);

  const rounds = 400;
  const t0 = process.hrtime.bigint();
  for (let r = 0; r < rounds; r++) {
    for (let i = 0; i < n; i++) {
      scratch.set(snap.subarray(i * stride, (i + 1) * stride));
      e.phys_set(i, scratchPtr);
    }
    for (let tick = 0; tick < LEAD; tick++) {
      drive(inputs, n, tick);
      e.phys_step(mask);
    }
  }
  const t1 = process.hrtime.bigint();
  return Number(t1 - t0) / 1000 / rounds;
}

console.log(`physics.wasm, ${LEAD}-tick lead, ${SNAPSHOT_HZ} Hz snapshots\n`);
console.log('  cars    step      rollback    per second        of a frame');
for (const n of GRIDS) {
  const step = timeStep(n);
  const back = timeRollback(n);
  const perSec = step * TICK_HZ + back * SNAPSHOT_HZ;
  console.log(
    `  ${String(n).padStart(4)}  ${step.toFixed(1).padStart(6)} us  ${back.toFixed(1).padStart(7)} us  ` +
      `${(perSec / 1000).toFixed(2).padStart(6)} ms/s     ${((perSec / 1000 / TICK_HZ / FRAME_MS) * 100).toFixed(1).padStart(5)} %`,
  );
}
console.log(
  '\n"per second" is the whole netcode cost of a second of racing: sixty forward\n' +
    'steps and twenty rollbacks. The last column is that spread over the frames it\n' +
    'happens in, on this machine. Multiply by three to eight for a phone.',
);
