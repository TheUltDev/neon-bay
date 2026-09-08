#!/usr/bin/env bash
# Run the demo: the authoritative sidecar in the background, the web client in
# the foreground. Assumes `spacetime start` is already running and
# scripts/setup.sh has been run at least once.
#
#   ./scripts/dev.sh [--bots 6] [--db physics-sidecar] [--uri http://127.0.0.1:3000]
#
# Ctrl-C stops both.

set -euo pipefail

BOTS=6
DB=physics-sidecar
URI=http://127.0.0.1:3000

while [ $# -gt 0 ]; do
  case "$1" in
    --bots) BOTS="${2:?--bots needs a value}"; shift 2 ;;
    --db)   DB="${2:?--db needs a value}"; shift 2 ;;
    --uri)  URI="${2:?--uri needs a value}"; shift 2 ;;
    -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if command -v curl >/dev/null 2>&1; then
  if ! curl -fsS -m 3 "$URI/v1/ping" >/dev/null 2>&1; then
    echo "No SpacetimeDB at $URI. Start one with:  spacetime start" >&2
    exit 1
  fi
fi

# Windows shells (Git Bash, MSYS) build sidecar.exe; everyone else builds sidecar.
SIDECAR="$ROOT/target/release/sidecar"
[ -x "$SIDECAR" ] || SIDECAR="$ROOT/target/release/sidecar.exe"
if [ ! -x "$SIDECAR" ]; then
  echo 'Building the sidecar...'
  ( cd "$ROOT" && cargo build -p sidecar --release )
  SIDECAR="$ROOT/target/release/sidecar"
  [ -x "$SIDECAR" ] || SIDECAR="$ROOT/target/release/sidecar.exe"
fi

echo 'Starting the authoritative sidecar...'
"$SIDECAR" --bots "$BOTS" --db "$DB" --uri "$URI" &
SIDECAR_PID=$!

cleanup() {
  trap - EXIT INT TERM
  kill "$SIDECAR_PID" 2>/dev/null || true
  wait "$SIDECAR_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo 'Starting the web client on http://localhost:5173 ...'
cd "$ROOT/web"
npm run dev
