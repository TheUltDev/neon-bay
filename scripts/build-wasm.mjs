// Build the physics core for the browser and drop it where Vite serves it.
//
//   node scripts/build-wasm.mjs        (or: cd web && npm run wasm)
//
// This exists as a Node script rather than a line in each shell script so the
// build and the copy have exactly one implementation. `cp` and `Copy-Item`
// disagree about almost everything.

import { execFileSync } from 'node:child_process';
import { copyFileSync, mkdirSync, statSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const OUT_DIR = join(root, 'web', 'public');
const OUT = join(OUT_DIR, 'physics.wasm');
const BUILT = join(root, 'target', 'wasm32-unknown-unknown', 'release', 'physics.wasm');

try {
  execFileSync(
    'cargo',
    ['build', '-p', 'physics', '--release', '--target', 'wasm32-unknown-unknown'],
    { cwd: root, stdio: 'inherit' },
  );
} catch (err) {
  console.error(
    '\ncargo build failed. If the target is missing:\n' +
      '  rustup target add wasm32-unknown-unknown\n',
  );
  process.exit(typeof err.status === 'number' ? err.status : 1);
}

mkdirSync(OUT_DIR, { recursive: true });
copyFileSync(BUILT, OUT);
console.log(`physics.wasm -> web/public (${(statSync(OUT).size / 1024).toFixed(1)} KB)`);
