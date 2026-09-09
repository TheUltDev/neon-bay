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
# The volume, and the three separate things on it that outlive a deploy. The
# database sits in its own subdirectory rather than at the root so that
# discarding it is `rm -rf` on one path, and the two credentials beside it --
# which it would be a mistake to discard -- are not in the blast radius.
STATE_DIR="${STDB_STATE_DIR:-/stdb}"
DATA_DIR="${STDB_DATA_DIR:-${STATE_DIR}/data}"
# The keypair identities are signed with. SpacetimeDB keeps it next to the CLI
# config by default, which is in the image rather than on the volume, so it
# would be regenerated on every deploy -- and every player would come back
# holding a token signed by a key that no longer exists, be refused, and be
# handed a new identity with none of their lap records attached to it. Keeping
# it beside the data it authenticates is what makes the volume worth attaching.
# `spacetime start` generates the pair if these paths are empty, so first boot
# needs no special case.
KEY_DIR="${STDB_KEY_DIR:-${STATE_DIR}/keys}"
# The CLI's own identity, for the same reason one step further along. A database
# belongs to the identity that created it and only that identity may publish to
# it again, and the CLI keeps its identity in cli.toml under $HOME -- in the
# image. Left there it is new on every deploy, so the second deploy onto a
# surviving database is a stranger to it and gets a 403 from the pre-publish
# check, with nothing in the container able to grant itself the rights back.
CLI_CONFIG="${STDB_CLI_CONFIG:-${STATE_DIR}/cli.toml}"
DB="${STDB_DB:-physics-sidecar}"
BOTS="${SIDECAR_BOTS:-6}"
WASM="${MODULE_WASM:-/app/module.wasm}"
LOCAL="http://127.0.0.1:${PORT}"

log() { printf '[boot] %s\n' "$*"; }
die() { printf '[boot] %s\n' "$*" >&2; exit 1; }

mkdir -p "$DATA_DIR" "$KEY_DIR" || die "could not create directories under ${STATE_DIR}"

log "SpacetimeDB on 0.0.0.0:${PORT}, data dir ${DATA_DIR}, keys ${KEY_DIR}"
spacetime --config-path "$CLI_CONFIG" start --listen-addr "0.0.0.0:${PORT}" --data-dir "$DATA_DIR" \
  --jwt-priv-key-path "${KEY_DIR}/id_ecdsa" \
  --jwt-pub-key-path "${KEY_DIR}/id_ecdsa.pub" \
  --non-interactive &
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

# The data directory outlives the deploy now, so this is an update rather than a
# create: the module is replaced and the lap records, identities and registry
# rows underneath it stay. The same command creates the database when the volume
# is empty, so first boot needs no special case. `--yes` implies `skip-login`,
# which is what keeps the CLI from stopping to ask about spacetimedb.com in a
# container with no tty.
log "publishing ${DB}"
published=0
for attempt in 1 2; do
  if spacetime --config-path "$CLI_CONFIG" publish \
      --server "$LOCAL" --bin-path "$WASM" --yes "$DB"; then
    published=1
    break
  fi
  log "publish attempt ${attempt} did not take"
  sleep 2
done

# Getting here means the database on the volume cannot be migrated to this
# module -- a changed table, almost always, since the retry above has already
# covered a database that answered /v1/ping a moment before it was ready. That
# leaves a container that will not start or a deploy that costs the lap times,
# and for a demo whose durable state is a lap time the deploy wins. Loudly,
# because it is the one path here that destroys anything.
if [ "$published" -eq 0 ]; then
  log "WARNING: ${DB} cannot take this module in place -- recreating it, and its data is going with it"
  spacetime --config-path "$CLI_CONFIG" publish --server "$LOCAL" --bin-path "$WASM" \
    --delete-data=always --yes "$DB" \
    || die "could not publish ${DB}. A 403 here means this container is not the
identity that owns the database, which is what ${CLI_CONFIG} exists to prevent:
if that file was lost while ${DATA_DIR} survived, nothing in here can win the
rights back. Delete ${DATA_DIR} -- and only that, the keypair and cli.toml
beside it are what the next boot needs to stay the same server."
fi

# The sidecar connects as the identity that owns the database, because `input`
# is a private table and SpacetimeDB shows one of those to its owner and to
# nobody else. That identity is the CLI's, and its token is in the config file
# on the volume -- the same file that lets the next deploy publish at all.
SIDECAR_TOKEN="$(awk -F'"' '/^spacetimedb_token/ { print $2; exit }' "$CLI_CONFIG")"
[ -n "$SIDECAR_TOKEN" ] || die "no spacetimedb_token in ${CLI_CONFIG}; the sidecar cannot
read the input table without it, and the race would never start."

log "sidecar -> ${LOCAL} / ${DB}, ${BOTS} bots"
STDB_TOKEN="$SIDECAR_TOKEN" sidecar --uri "$LOCAL" --db "$DB" --bots "$BOTS" ${SIDECAR_QUIET:+--quiet} &
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
