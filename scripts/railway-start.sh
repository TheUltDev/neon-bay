#!/usr/bin/env bash
# Container entrypoint: SpacetimeDB, then the module, then the authority.
#
# Railway gives a service one process tree and one public port, so this script
# is the supervisor. It brings the database up, waits for it to answer,
# publishes the module into it, starts the sidecar, and then takes the whole
# container down if either half dies -- a sidecar without a database has nothing
# to write to, and a database without a sidecar is a frozen race. Restarting the
# pair is always the right answer, and the sidecar is built for it: it re-claims
# authority and resumes the tick clock from `config.server_tick`.

set -euo pipefail

# Railway injects PORT. Falling back to 3000 keeps `docker run -p 3000:3000`
# working unchanged.
PORT="${PORT:-3000}"
DATA_DIR="${STDB_DATA_DIR:-/stdb}"
DB="${STDB_DB:-physics-sidecar}"
BOTS="${SIDECAR_BOTS:-6}"
WASM="${MODULE_WASM:-/app/module.wasm}"
LOCAL="http://127.0.0.1:${PORT}"

log() { printf '[boot] %s\n' "$*"; }
die() { printf '[boot] %s\n' "$*" >&2; exit 1; }

log "SpacetimeDB on 0.0.0.0:${PORT}, data dir ${DATA_DIR}"
spacetime start --listen-addr "0.0.0.0:${PORT}" --data-dir "$DATA_DIR" --non-interactive &
STDB_PID=$!

# Railway's healthcheck is watching the same endpoint, so anything that goes
# wrong here should be loud and fast rather than a deploy that hangs.
log 'waiting for the database'
ready=0
for attempt in $(seq 1 120); do
  if curl -fsS -m 2 "${LOCAL}/v1/ping" >/dev/null 2>&1; then
    log "database answering after ${attempt} attempt(s)"
    ready=1
    break
  fi
  kill -0 "$STDB_PID" 2>/dev/null || die 'SpacetimeDB exited during startup'
  sleep 0.5
done
[ "$ready" -eq 1 ] || die 'database did not come up within 60s'

# Nothing survives a deploy, so this is a create every time rather than an
# update; --delete-data=always says so instead of depending on what happens to
# be in the data directory. `--yes` implies `skip-login`, which is what keeps
# the CLI from stopping to ask about spacetimedb.com in a container with no tty.
log "publishing ${DB}"
spacetime publish --server "$LOCAL" --bin-path "$WASM" \
  --delete-data=always --yes "$DB" \
  || die "could not publish ${DB}"

log "sidecar -> ${LOCAL} / ${DB}, ${BOTS} bots"
sidecar --uri "$LOCAL" --db "$DB" --bots "$BOTS" ${SIDECAR_QUIET:+--quiet} &
SIDECAR_PID=$!

shutdown() {
  trap - TERM INT
  kill "$SIDECAR_PID" "$STDB_PID" 2>/dev/null || true
  wait "$SIDECAR_PID" "$STDB_PID" 2>/dev/null || true
}
trap shutdown TERM INT

# Whichever exits first ends the deployment.
status=0
wait -n "$STDB_PID" "$SIDECAR_PID" || status=$?
log "a process exited with status ${status}; stopping the container"
shutdown
exit "$status"
