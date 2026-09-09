#!/usr/bin/env bash
# One-time (or after changing the module schema): publish, regenerate bindings,
# build the wasm physics core, install the web client's dependencies.
#
#   ./scripts/setup.sh [--server local] [--db physics-sidecar] [--fresh]
#
# --fresh wipes the database, which you want whenever the schema changes shape.

set -euo pipefail

SERVER=local
DB=physics-sidecar
FRESH=0

while [ $# -gt 0 ]; do
  case "$1" in
    --server) SERVER="${2:?--server needs a value}"; shift 2 ;;
    --db)     DB="${2:?--db needs a value}"; shift 2 ;;
    --fresh)  FRESH=1; shift ;;
    -h|--help) sed -n '2,7p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

step() { printf '\n\033[36m== %s\033[0m\n' "$1"; }

step 'checking toolchain'
for cmd in cargo rustup spacetime npm node; do
  command -v "$cmd" >/dev/null 2>&1 || { echo "$cmd not found on PATH" >&2; exit 1; }
done
if ! rustup target list --installed | grep -qx 'wasm32-unknown-unknown'; then
  step 'installing the wasm32-unknown-unknown target'
  rustup target add wasm32-unknown-unknown
fi

# The bindings `spacetime generate` writes have to compile against the SDK
# version this project pins. If the CLI has moved on, say so now rather than
# letting cargo fail later with a wall of type errors.
pinned="$(grep -oE 'spacetimedb = "[^"]+"' module/Cargo.toml | grep -oE '[0-9]+\.[0-9]+' | head -1)"
cli="$(spacetime --version 2>/dev/null | grep -oE 'version [0-9]+\.[0-9]+' | head -1 | grep -oE '[0-9]+\.[0-9]+')"
if [ -n "$pinned" ] && [ -n "$cli" ] && [ "$pinned" != "$cli" ]; then
  printf '\033[33mwarning: spacetime CLI is %s, this project pins SDK %s.\n' "$cli" "$pinned"
  printf '         If the generated bindings do not compile, bump the version in\n'
  printf '         module/Cargo.toml, sidecar/Cargo.toml and web/package.json.\033[0m\n'
fi

step 'publishing the SpacetimeDB module'
publish=(publish --server "$SERVER" --module-path module --yes)
[ "$FRESH" -eq 1 ] && publish+=(--delete-data=always)
publish+=("$DB")
spacetime "${publish[@]}"

step 'generating client bindings'
# `--include-private` only for the sidecar: `input` is a private table, and the
# sidecar is the one client allowed to read it. The browser's bindings are
# generated without it, so the web bundle does not even carry the accessor.
spacetime generate --lang rust --include-private -y \
  --out-dir sidecar/src/module_bindings --module-path module
spacetime generate --lang typescript -y \
  --out-dir web/src/module_bindings --module-path module

step 'building the shared physics core (native + wasm)'
cargo build --release
node scripts/build-wasm.mjs

step 'installing web dependencies'
( cd web && npm install --no-fund --no-audit )

step 'verifying native and wasm agree bit for bit'
node scripts/verify-determinism.mjs | tail -2

printf '\n\033[32mReady. Start it with: ./scripts/dev.sh\033[0m\n'
