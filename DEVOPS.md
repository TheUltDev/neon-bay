# Building, running and deploying

Everything needed to build, run, test and deploy the demo. The
[README](README.md) explains what it is and how it performs; this file is how to
operate it.

## Running it

Needs [SpacetimeDB](https://spacetimedb.com/install) 2.8, Rust with the
`wasm32-unknown-unknown` target, and Node 20+. Everything below works the same
on Linux, macOS and Windows.

```bash
spacetime start                    # in its own terminal

./scripts/setup.sh --fresh         # publish, generate bindings, build wasm, npm install
./scripts/dev.sh --bots 6          # sidecar + web client
```

On Windows, the same two scripts in PowerShell:

```powershell
powershell -File scripts/setup.ps1 -Fresh
powershell -File scripts/dev.ps1 -Bots 6
```

Then open <http://localhost:5173>, pick a name and color, and drive with
`WASD`. `Space` is the handbrake, `R` respawns, `G` toggles the server ghost,
`C` toggles the rotating camera, which starts on.

**Version pinning matters.** `spacetime generate` emits bindings for the CLI's
own version, and they have to compile against the SDK this project pins. Those
pins live in three files and move together:

| | |
|---|---|
| `module/Cargo.toml` | `spacetimedb = "2.8"` |
| `sidecar/Cargo.toml` | `spacetimedb-sdk = "2.8"` |
| `web/package.json` | `"spacetimedb": "~2.8.3"` |

Both setup scripts compare your CLI against those pins and warn if they have
drifted. Generated bindings are not committed, so run `setup` before `cargo
build` at the workspace root. `cargo test -p physics` needs nothing generated
and works on a fresh clone.

<details>
<summary>Running the pieces by hand</summary>

```bash
spacetime publish --server local --module-path module --delete-data=always --yes physics-sidecar
spacetime generate --lang rust --include-private -y --out-dir sidecar/src/module_bindings --module-path module
spacetime generate --lang typescript -y             --out-dir web/src/module_bindings     --module-path module

node scripts/build-wasm.mjs                     # cargo build + copy into web/public
export STDB_TOKEN="$(spacetime login show --token | awk '/auth token/ { print $NF }')"
cargo run -p sidecar --release -- --bots 6      # terminal 1
cd web && npm install && npm run dev            # terminal 2
```

Two of those need saying out loud. `--include-private` is what puts the `input`
table in the sidecar's bindings, because a private table is invisible to an
ordinary client and the codegen leaves it out; the browser's bindings are
generated without the flag, so the web bundle does not even carry the accessor.
And `STDB_TOKEN` is the module publisher's token, which is the only identity
allowed to read that table or to claim the authority. Without it the sidecar
exits on its first subscription and says which flag it wanted.

While iterating on the physics, `cd web && npm run wasm` rebuilds and copies the
wasm on its own; Vite picks it up on reload.
</details>

<details>
<summary>Pointing the pieces at a different machine</summary>

The client defaults to `<page host>:3000`. Override it per tab:

```
http://localhost:5173/?uri=http://192.168.1.20:3000&db=physics-sidecar
```

The sidecar takes `--uri` and `--db` (or `STDB_URI` / `STDB_DB` / `STDB_TOKEN`),
and both dev scripts forward them. Vite binds to localhost only; to reach it
from another device on the network use `npm run dev -- --host`.
</details>

## Deploying it

Two hosts, because the two halves want different things. The database and the
sidecar go to **Railway as one container**, so the link between them stays on
loopback rather than becoming a network: they talk on the hot path, every input
as it lands and twenty snapshots a second. The browser client is static, so it
goes to **Cloudflare Workers** at the edge. Its only conversation is one `wss://`
back to Railway.

### The game server

`Dockerfile` builds all three native pieces in one pass: module wasm, generated
bindings, sidecar. `scripts/railway-start.sh` supervises the pair inside the
container, starting SpacetimeDB, waiting for `/v1/ping`, publishing the module
into it, starting the authority, and bringing the whole container down if either
half exits. Railway restarts it and the sidecar re-claims, resuming the tick
clock from `config.server_tick`.

```bash
railway login
railway init                # or `railway link` for an existing project
railway up                  # builds the Dockerfile and deploys
railway domain              # the public https:// URL
```

`railway.json` health-checks `/v1/ping`, restarts always, and pins the service
to `us-east4` at **one replica**. One replica is not a limitation waiting to be
fixed: `push_states` is guarded by a single registered identity, so the module
would turn a second sidecar away even if Railway ran one.

**Storage persists.** A Railway volume at `/stdb` holds three things: the
database in `data/`, the keypair identities are signed with in `keys/`, and the
CLI's own identity in `cli.toml`. They are kept apart because the database is
the only one it is ever right to throw away.

Four things have to be true for that to mean anything, and all four live in
`scripts/railway-start.sh`:

- **The module is published in place**, not recreated. `spacetime publish`
  without `--delete-data` updates the module and keeps the tables underneath it,
  and creates the database when the volume is empty, so first boot is not a
  special case. A module the running database cannot migrate to is retried once,
  then recreated from scratch, losing the data loudly: a demo that will not start
  is worse than a demo that lost its lap times.
- **The signing keypair lives on the volume**, at `/stdb/keys`, via
  `--jwt-priv-key-path` and `--jwt-pub-key-path`. SpacetimeDB otherwise keeps it
  beside the CLI config, which is in the image rather than on the volume. A
  keypair that changes every deploy hands every returning player a token signed
  by a key that is gone: their records survive, their claim on them does not.
- **The CLI's identity lives on the volume too**, at `/stdb/cli.toml`, via
  `--config-path`. This is the one that bites, because nothing goes wrong until
  the *second* deploy. A database belongs to the identity that created it, and
  the CLI keeps its identity under `$HOME`, which is in the image. Leave it there
  and deploy two arrives a stranger to the database deploy one created:
  publishing answers `403 ... is not authorized`, and the recreate fallback
  cannot save it either, because resetting a database is also something only its
  owner may do. The container crashloops with no way to grant itself the rights
  back, and the only fix is from outside: discard `data/`, and nothing else,
  which is what `STDB_WIPE_DATA` below does.
- **The sidecar connects as that identity too**, because `input` is private and
  only the database's owner can read it. The entrypoint lifts
  `spacetimedb_token` out of the same `cli.toml` and hands it over in the
  environment rather than on the command line. Without it the sidecar's
  subscription is refused and it exits saying which token it wanted, which is
  the right failure: loud, immediate, and impossible to mistake for a bug in the
  physics.

The volume needs `RAILWAY_RUN_UID=0` alongside it: the image runs as a non-root
user and Railway mounts volumes root-owned. Set the variable before attaching
the volume and the deploy in between still comes up.

A connecting client has no way to know a key was rotated, so it treats a refused
token as a credential to discard rather than a server to give up on. See
[The client](#the-client).

Service variables:

| | |
|---|---|
| `PORT` | Set to `3000`. What the server listens on and what Railway's proxy forwards to; they have to agree. |
| `SIDECAR_BOTS` | AI drivers on the grid. Default 6, capped at 24, which is the whole grid and leaves no room for players. |
| `SIDECAR_QUIET` | Set to anything to silence the once-a-second status line. |
| `STDB_DB` | Database name. Default `physics-sidecar`. |
| `STDB_WIPE_DATA` | Set to anything to discard the database on the next boot and nothing else, leaving the keypair and `cli.toml` in place. For a volume that has filled or a database that will not open, either of which stops SpacetimeDB before the container is able to publish. Remove it and redeploy once the service is up: deleting the variable does not by itself restart anything, so the flag stays in the running container until a new deployment replaces it, and while it is there every restart starts an empty race. |

A public instance accepts public connections, which is why the authority is not
first-come: `claim_authority` refuses everyone but the identity that published
the module, recorded in `config` by `init`. The restart window after a deploy is
not an opening for a stranger, only for the next sidecar holding that token.

### The client

The deployed client is not on the same host as the server, so it has to be told
where the server is. That lives in `web/.env.production`, which is committed
rather than ignored: it is a public address, and keeping it in the repo is what
makes the deployed client reproducible from a clone.

```bash
cd web
npm run deploy      # type check, Vite build, wrangler deploy
```

`wrangler.toml` declares an assets-only Worker, no script, just `dist/`, and
`npm run deploy` type checks and builds before uploading so a broken build never
ships. Two pages come out: the game at `/` and the write-up at `/tech`. Anything
that is not a real file falls through to the game, so a deep link does not 404.

`?uri=` overrides that at runtime, which is the quickest way to point a deployed
page at a server on your desk:

```
https://<your-worker>.workers.dev/?uri=http://192.168.1.20:3000
```

**A refused token is forgotten, not retried.** The client keeps its identity
token in `localStorage` and presents it on every dial, and the browser SDK trades
it for a short-lived one *before* opening the socket. So a token the database
will not verify fails the dial outright and looks like a server that is down,
when the truth is the opposite: the server is up and would take the same player
without it. `Net.open` tells the two apart, drops the token and redials as a
stranger. The retry cannot loop, because the second pass has no token to reject.

This matters even with the volume attached, because a key rotates whenever the
volume is replaced or the database moves. Without it the reconnect loop
re-presents the dead token forever.

**Ship the sidecar and `physics.wasm` together.** They are two halves of one
simulation deployed to different places, so it is easy to update one and not the
other. If you do, the authority readout turns red and reads *physics mismatch*
instead of leaving you to work out why prediction went bad. See
[Checking it at runtime too](README.md#checking-it-at-runtime-too).

## Failover

Run a second sidecar against the same database and it does not fight the first
one. It stands by: connected, subscribed, its world adopting every snapshot as
it lands, publishing nothing. Costed on the status line, standing by is 20 µs a
tick against the authority's 62, and it says `STANDBY` at the end of the line so
you can tell which process is which. Adopting a row is the same work whatever is
in it, and a standby runs no physics at all, so what it costs does not move with
the simulation.

The database decides who holds the seat, because it is the only thing that sees
both processes:

- **`claim_authority` grants** when nobody holds the lease, when you already do,
  or when the holder has not published for two seconds. Otherwise it refuses.
  Two sidecars claiming at once are two transactions, and one of them commits
  first; the loser reads `config` and keeps standing by.
- **`push_states` checks the connection**, not the identity. That is the fence.
  A sidecar that was partitioned long enough to lose its lease finds its writes
  refused the instant it reconnects, sees it is no longer the holder, and demotes
  itself, rather than publishing a race that moved on without it.
- **Losing the socket frees the seat immediately.** `client_disconnected` clears
  the holder, so the ordinary case of a deploy, a crash or a `kill` does not wait
  out the lease at all.

Killed outright with `kill -9`, and watching `car_state` from a third
connection: the longest the authoritative stream went quiet across four runs was
**103 to 123 ms**, against the 50 ms that separates two snapshots anyway. So the
race pauses for about one extra snapshot, and resumes from the same tick the
dead sidecar left, `config.server_tick + 1`, with every car adopted from its
last published pose. Drivers keep their momentum, their lap and their position
through a handover they mostly cannot see, and with it the gear they were in,
the revs they were pulling, the speed each wheel was turning at and how far the
body had rolled. All of it is on the wire for exactly this reason, the controls
each driver had their hands on included, so the new authority resumes with the
throttle where the old one left it instead of releasing every pedal on the grid
for a tick.

## Testing and poking at it

[PHYSICS.md](PHYSICS.md) says what the physics tests measure; the netcode
invariants live in the same suite.

```bash
cargo test -p physics --release -- --nocapture   # physics + netcode invariants
cargo run -p physics --example probe   --release  # performance and handling envelope
cargo run -p physics --example crash   --release  # what a hit costs, and predicts as
cargo run -p physics --example wear    --release  # how battered a field of bots gets
cargo run -p physics --example predict --release  # four ways to guess a rival, scored
node scripts/verify-determinism.mjs              # native vs wasm, bit for bit
node scripts/bench-client.mjs                    # what a browser tick costs, by grid size
node scripts/build-wasm.mjs                      # rebuild just the browser core
spacetime sql --server local physics-sidecar "SELECT car_id, tick, x, y, dmg_front FROM car_state"
spacetime logs --server local physics-sidecar

# Failover: start a second sidecar beside the one dev.sh runs. It prints
# STANDBY once a second until you kill the first, then takes the race over
# from the tick it left. Both need the module publisher's token.
export STDB_TOKEN="$(spacetime login show --token | awk '/auth token/ { print $NF }')"
cargo run -p sidecar --release -- --bots 6
```

In the browser console, `__neon` exposes `{ sim, net, renderer, hud, controls,
audio }`. Try `__neon.sim.stats`, `__neon.net.rttMs`, `__neon.sim.cheat(30)`, or
`__neon.sim.fingerprint` against `__neon.net.physicsFingerprint` to see the two
physics builds agree.

