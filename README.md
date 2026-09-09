# Physics sidecar for SpacetimeDB

A proof of concept for keeping **server authority** over a real-time physics
simulation while running none of that physics inside the database.

You can run the simulation inside the module, and SpacetimeDB handles it fine.
The question is what it costs. Every tick is a transaction, so at 60 Hz per car
the database spends its throughput on tire forces and collisions instead of on
everything else you asked it to do. The usual fix is to let clients own their
positions, which trades the security model for frame rate.

There is a third option. The physics moves into a **sidecar**: an ordinary
SpacetimeDB client that happens to be trusted, running the simulation in its own
process at 60 Hz and writing the results back through a reducer only it may
call. Players still send nothing but controller readings, and the database still
decides where every car is. It just no longer does the arithmetic.

The browser runs the same simulation, compiled from the same Rust source to
wasm, and predicts locally so steering is instant. When the authority disagrees,
the client rewinds, replays and slides back into line without a visible jump.

**Prediction error on a healthy local link: 0.000 m.** Not small. Zero, because
both sides run identical instructions on identical inputs.

**[Play it](https://neon-bay.ult.workers.dev).** The client is on Cloudflare's
edge, the database and authority on Railway in `us-east4`. The zero holds across
that ~90 ms round trip too, because distance does not matter, only running the
same instructions. See [Deploying it](#deploying-it).

**[How it works](https://neon-bay.ult.workers.dev/tech).** The architecture
written up as a page, for anyone who wants to build a sidecar of their own
rather than clone this one.

---

## What to try

Open the demo and find the **Prediction & reconciliation** panel. Everything
below is designed to move a number on it.

- **Just drive.** Prediction error sits at `0.000 m` and the graph reads
  `PREDICTION EXACT`. The rollback machinery is running; it never finds anything
  to correct.
- **Drag *added latency* to 250 ms.** Watch "lead" climb from +5 to +19 ticks:
  the client runs further ahead so its inputs still land on time. The car keeps
  steering instantly and the error stays in millimeters.
- **Add *packet loss*.** Dropped inputs leave the authority holding the last
  controller state it heard, so it briefly disagrees. Corrections tick up, and
  "smoothing out" shows the error being absorbed rather than snapped.
- **Press *Desync me*.** The client cheats and adds 26 m/s the physics never
  gave it. The sidecar never sees it, because a client can only say "throttle
  down", so a round trip later the authority disagrees by over a meter and pulls
  the car back. Nothing teleports.
- **Watch the server ghost** (the dashed outline). That is the authority's most
  recent published pose, drawn next to your prediction. On a good link it trails
  by exactly the lead the client is holding: a round trip plus two ticks.
- **Change renderer mid-lap.** The *Renderer* panel names the backend, the GPU
  and the frame rate. Its three buttons swap tier without a reload, so you can
  watch the same frame come out of WebGPU, WebGL 2 and Canvas2D one after the
  other.

## The shape of it

```
   BROWSER                    SPACETIMEDB                  SIDECAR
   ────────                   ───────────                  ───────
   physics.wasm  ──inputs──▶  ┌───────────┐  ──inputs──▶   physics (rlib)
   (prediction)   30 Hz       │  module   │    subscribe    (authority)
        ▲                     │           │                     │
        │                     │  inbox    │                     │ 60 Hz
        │                     │  registry │                     │ fixed step
        │                     │  outbox   │                     ▼
        └───snapshots───────  └───────────┘  ◀──push_states──  20 Hz
             subscribe                          (authority only)

   ── the module contains no integrator, no collision code, no track ──
```

Both ends of that diagram are the **same crate**, `physics/`, compiled twice:
an `rlib` for the sidecar and a `cdylib` for the browser. Not a port and not a
reimplementation. One source of truth for what a car does.

### Layout

| Path        | What it is |
|-------------|------------|
| `physics/`  | The simulation. Deterministic `f32`, no dependencies. Vehicle dynamics, track, collisions, bot driver, and a bare C-ABI wasm bridge. |
| `module/`   | The SpacetimeDB module. Tables and reducers only: an inbox for inputs, a registry of who is racing, an outbox for authoritative state. |
| `sidecar/`  | The authority. A SpacetimeDB client that steps the world at 60 Hz, drives the bots, and publishes snapshots at 20 Hz. `src/bin/load.rs` is a second binary: synthetic players, for measuring what a client costs the database. |
| `web/`      | The game. Vite + TypeScript, no framework, drawn on WebGPU, WebGL 2 or Canvas2D depending on the browser. Prediction, rollback, prediction of the other cars too, and a telemetry HUD that shows all of it working. Sound is synthesized, so there are no audio files to ship. `public/tech.html` is the architecture write-up served at `/tech`. |
| `scripts/`  | Setup, dev launcher, the native-vs-wasm determinism check, and the container entrypoint. |
| `Dockerfile` | The database and the sidecar in one image, for [deploying](#deploying-it). `railway.json` and `web/wrangler.toml` are the rest of it. |

## What it costs

Sidecar, 7 cars: six AI drivers and one connected player. Measured on the status
line it prints once a second:

```
[t   20663] 60.0 Hz sim | 20.0 Hz snap | 7 cars (6 bots) |  34.8 us/tick |  0.2% of budget | 20 inputs/s
```

Thirty-five microseconds of a 16.7 ms budget, a fifth of one percent. That is
the whole tick: pumping the connection, reading the inbox, driving the bots,
stepping the world, and publishing every third tick. The integrator is about
5 µs of it and the bot AI another 6. Most of the rest is the SDK.

The wasm build runs 10 000 ticks in under 10 ms in Node, so a 20-tick rollback
costs well under 20 µs, cheap enough that the client can afford one on every
snapshot without thinking about it.

The whole physics core is a **38 KB** `.wasm` with zero imports: no
wasm-bindgen, no wasm-pack, no build plugin. `CarState` is `#[repr(C)]` and all
`f32`, so the browser maps it with one `Float32Array` over wasm memory and reads
and writes the simulation in place. Nothing is serialized on the hot path.

**The database's cost does not grow with the simulation.** Every car rides in one
`push_states` call, so 24 cars and 1 car cost the same 20 write transactions a
second, and across that range the database never rises above the noise of its own
idle work. The module has nothing to compute about a car in any case, because it
does not depend on the physics crate.

Players do cost it. The full sweep, up to a 24-car grid with eighteen of them
connected, is in [The numbers](#the-numbers).

## The trust model

A client can call exactly one hot-path reducer, `set_input`, and it carries
throttle, steering, brake and handbrake. There is no reducer that lets a player
assert a position, a lap, or a time. It is rate limited to 45 writes a second
per identity with a burst of 15 banked, which the browser's 30 never comes near;
anything past that is dropped before the row is written, and so before any
subscriber pays to hear about it.

A player cannot *read* the inbox either. `input` is a private table, and
SpacetimeDB shows one of those to the database's owner and to nobody else -- a
subscription from anyone else is refused with "no such table". The sidecar is
that owner, connecting as the identity that published the module.

Two checks guard the outbox, and they answer different questions. *May* you be
the authority: `claim_authority` compares the caller against the owner recorded
in `config` when the module was published, so nobody can appoint themselves
during a handover. *Are* you the authority right now: `push_states` compares the
caller's **connection** against the recorded lease holder. A connection rather
than an identity, because a standby runs on the same credentials as the sidecar
it is standing by for -- and because it makes the check a fencing token, so a
sidecar that was replaced while it was partitioned has its writes refused the
moment it comes back rather than scribbling over a race it no longer runs.

The whole mechanism is **112 lines**: `claim_authority`, `release_authority`,
`require_authority`, `spend`, `set_input` and `push_states`. Twenty-three of
those are `push_states`, which only has to check the caller, drop anything stale and
write. You can read all of it before deciding to trust it.

The trust boundary did not disappear, it *moved*: from "the database is the only
thing that can be trusted" to "the database plus the identity that published the
module". That is the trade. In exchange the simulation becomes a normal process,
one you can profile, shard, or restart without touching the data.

## How the netcode works

Three pieces, in the order they run.

**1. Predict.** Every tick, the client samples the controls, steps its local car
in wasm immediately, and stores `(tick, input, resulting state)` in a 256-entry
ring buffer. Steering has zero input lag regardless of ping.

**2. Reconcile.** Snapshots arrive at 20 Hz carrying the authority's state for
a tick `T` in the recent past. The client compares it against what it recorded
for `T`. If they differ by more than a millimeter it rewinds to the authoritative
state and replays the stored inputs from `T+1` to now, typically 5 to 20 ticks
and about 20 µs of work.

`physics::tests::rollback_replay_is_bit_exact` pins the invariant this rests on:
re-simulating from an older state with the same inputs reproduces the same bits.

**3. Smooth.** A rewind moves the car, which would read as a stutter. So the
client keeps the gap between where the car appeared to be and where it now is as
a visual offset, and decays it to zero over ~220 ms. The simulation is corrected
at once; the picture catches up. Corrections over 9 m, meaning a respawn or a
long stall, snap instead and raise the "resyncs" counter.

**The clock.** For prediction to be exact, an input stamped for tick `T` has to
be applied by the authority *at* tick `T`. The client estimates where the
sidecar's clock is, adds a round trip plus two ticks of margin, and steers its
own tick rate by up to ±6 % to hold that lead. You cannot feel it. The sidecar
schedules inputs rather than applying them on arrival: an early one waits for its
tick, and a late one is applied at once, since there is no rewinding the
authority. The client absorbs the difference on its next rollback.

**Other cars.** The client never simulates them, but it does predict them. The
local car is deliberately in the future -- far enough ahead that its input
reaches the authority before the tick it belongs to -- so holding rivals a few
ticks in the *past*, which is what an interpolation buffer does, puts them wrong
by the sum of the two.
At racing speed that is several metres, systematically, in the direction of
travel: you would be leaning on a car the authority had somewhere else. So each
rival is carried forward from its newest published pose to the tick actually
being simulated, at a constant turn rate, which follows a car through a corner
instead of flying off the tangent. Measured against the snapshots that arrive
next, that guess is out by 0.14 m over a tenth of a second of lead. Whatever
error a snapshot does reveal is kept as a fading offset rather than a jump --
the same trick the local car plays after a rollback.

Rivals are parked in the wasm world as immovable colliders, so you can lean on
one mid-corner and your prediction reacts at once without ever claiming to own
its state. `World::step` takes a simulation mask for this: the sidecar passes
every car, the browser passes only its own. A rollback puts the rivals back
where they were at each replayed tick, so what gets replayed is the contact the
authority actually resolved.

## Determinism

Prediction is only exact if both builds agree bit for bit. Two things get in the
way, and both are handled in `physics/src/math.rs`:

- **Transcendentals.** `libm`'s `sinf` on x86-64 and the one linked into a wasm
  module need not agree in the last ulp, and one ulp compounds over a few hundred
  ticks into a visible desync. So `sin`, `cos`, `atan` and `atan2` are
  hand-rolled polynomials using only `+`, `-` and `*`, which are IEEE-754 exact
  everywhere. `sqrt` is correctly rounded by the standard, so it is used directly.
- **FMA contraction.** If the compiler fuses `a * b + c` into one multiply-add,
  the native build rounds once where wasm, which has no FMA instruction, rounds
  twice. Rust does not contract by default, so never build this workspace with
  `-C target-cpu=native`.

`node scripts/verify-determinism.mjs` runs the same 1200-tick scripted race
through both builds and diffs the raw `f32` bits:

```
ok   native fp   8338134d
     wasm   fp   8338134d
...
All 11 checkpoints identical, bit for bit.
x86-64 and wasm32 agree, so a healthy client predicts with zero error.
```

### Checking it at runtime too

That verifies the two builds on the machine that built them. It says nothing
about the pair actually talking to each other: the sidecar ships to a container
and `physics.wasm` ships to a CDN, and nothing makes those happen together. A
cached bundle, a rolled-back service or a half-succeeded deploy leaves the client
predicting with one physics while the authority decides with another.

Nothing errors when that happens. Every corner ends in a correction that looks
like packet loss, which is the worst kind of bug here: silent, and pointing at
the network.

So each side scores itself at startup. Not a version string, which only records
what someone remembered to bump, but a 240-tick scripted race hashed to one
`u32` (`physics/src/fingerprint.rs`). Two builds match only if they agree on the
arithmetic: a refactor that changes nothing observable still matches, and moving
one constant does not.

The sidecar hands its number to `claim_authority`, the module publishes it in
`config` without judging it, and the browser compares it against what its own
wasm computes. On a mismatch the authority readout goes red and reads **physics
mismatch**, and the console names both numbers and says to redeploy the two
together.

## The car

Not a dot with a velocity. A lateral-slip bicycle model with longitudinal load
transfer, a friction ellipse per axle, speed-sensitive steering, aero downforce,
grip-proportioned braking, and a handbrake that unloads the rear so you can hold
a slide. Bodies collide as two circles per car, so clipping a barrier with the
nose spins you the way it should.

Tuned against tests, not feel (`cargo test -p physics --release`):

| | |
|---|---|
| 0–100 km/h | ≈ 3.0 s |
| top speed | 238 km/h |
| steady-state cornering | 1.6 g, understeer-limited at the edge |
| circuit | 1424.7 m, 15–20 m wide, 12 checkpoints |
| bot lap times | 43.1 – 45.1 s, spread by driver skill |

The skidpad test pins that 1.6 g rather than just reporting it, because an
oversteer-limited car is undriveable with a keyboard.

## Drawing it

Three renderers, one picture. The client asks for **WebGPU**, falls back to
**WebGL 2**, then to **Canvas2D**. The Renderer panel says which one you got and
on what hardware, and its three buttons change tier in place: no reload, the
camera does not move, and a tier this browser will not give you is disabled and
says why on hover. `?renderer=` pins a tier from the URL.

The two GPU tiers are one renderer, not two. `web/src/render/gpu.ts` builds a
frame's worth of triangles and hands them to a device interface that WebGL 2 and
WebGPU each implement in about 350 lines. The track is triangulated once at load
into two buffers that never change; everything that moves is appended to three
more every frame. A full grid comes to eight draw calls, where the 2D renderer
issues one per car per detail.

Nothing is sorted and there is no depth buffer. Triangles land in the order they
were written, which is the order the 2D renderer paints in, and that is why the
three agree down to the stacked strokes that make the barriers glow. The one
place the APIs differ, which end of a render target counts as the top, comes to
a single sign in a projection.

Text is the exception on both GPU tiers. Nameplates go on a 2D canvas over the
top, because a glyph atlas for a dozen short strings is more machinery than the
strings are worth.

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
  back, and the only fix is from outside: delete `data/`, and nothing else.
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
[Checking it at runtime too](#checking-it-at-runtime-too).

## Failover

Run a second sidecar against the same database and it does not fight the first
one. It stands by: connected, subscribed, its world adopting every snapshot as
it lands, publishing nothing. Costed on the status line, standing by is 18 µs a
tick against the authority's 35, and it says `STANDBY` at the end of the line so
you can tell which process is which.

The database decides who holds the seat, because it is the only thing that sees
both processes:

- **`claim_authority` grants** when nobody holds the lease, when you already do,
  or when the holder has not published for two seconds. Otherwise it refuses.
  Two sidecars claiming at once are two transactions, and one of them commits
  first; the loser reads `config` and keeps standing by.
- **`push_states` checks the connection**, not the identity. That is the fence.
  A sidecar that was partitioned long enough to lose its lease finds its writes
  refused the instant it reconnects, sees it is no longer the holder, and demotes
  itself -- rather than publishing a race that moved on without it.
- **Losing the socket frees the seat immediately.** `client_disconnected` clears
  the holder, so the ordinary case -- a deploy, a crash, a `kill` -- does not
  wait out the lease at all.

Killed outright with `kill -9`, and watching `car_state` from a third
connection: the longest the authoritative stream went quiet across four runs was
**103 to 123 ms**, against the 50 ms that separates two snapshots anyway. So the
race pauses for about one extra snapshot, and resumes from the same tick the
dead sidecar left -- `config.server_tick + 1`, every car adopted from its last
published pose. Drivers keep their momentum, their lap and their position
through a handover they mostly cannot see.

## Known limitations

This is a proof of concept. Honest gaps:

- **A standby has to be running to be one.** The arbitration above is real, but
  this deployment ships a single container with a single sidecar in it, so a
  restart is still a gap of however long the process takes to come up. Two
  containers against one database is the fix, and nothing in the module or the
  sidecar is in the way of it.
- **Contact is still resolved on the authority's timeline.** Every car in the
  browser lives on one clock, and rivals are predicted onto it rather than
  interpolated behind it, so what you see, what you lean on and what the
  authority computes are the same arrangement of cars. What is left is that the
  prediction is a guess -- 0.14 m out at a tenth of a second of lead, more when a
  rival brakes hard or is hit. Real lag compensation, where the authority rewinds
  every rival into the view the toucher had, is a much larger piece of work, and
  one most racing games decline: two cars cannot both be right about a mutual
  impulse.
- **The rate limiter still runs the reducer.** A flood is dropped before it
  writes a row or fans one out, which is the part that costs everyone else, but
  the transaction is still opened. Bounding *that* is the host's job, not the
  module's.
- **The tick boundary spins.** The sidecar measures how late its own sleeps
  land and spins only that much -- 1.5 % of a core on the machine these numbers
  come from -- but a spin is a spin. Getting to zero needs a timer the operating
  system does not portably offer.
- **One track, 24 slots, one database.** The grid size is a constant shared by
  the module and the physics crate, and the sidecar refuses to start if the two
  disagree. Sharding across databases is not attempted.

## Poking at it

```bash
cargo test -p physics --release -- --nocapture   # physics + netcode invariants
node scripts/verify-determinism.mjs              # native vs wasm, bit for bit
node scripts/build-wasm.mjs                      # rebuild just the browser core
spacetime sql --server local physics-sidecar "SELECT car_id, tick, x, y, lap FROM car_state"
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

## The numbers

### Scaling

Players are the term that grows, because a player is a connection that writes:
30 Hz of `set_input` each, plus a subscription to the snapshots coming back.

| cars | players | inputs/s | sidecar tick | of budget | database |
|---|---|---|---|---|---|
| 1 | 0 | 0 | 26 µs | 0.16 % | below its own idle noise |
| 6 | 0 | 0 | 38 µs | 0.23 % | below its own idle noise |
| 12 | 0 | 0 | 45 µs | 0.27 % | below its own idle noise |
| 24 | 0 | 0 | 56 µs | 0.34 % | below its own idle noise |
| 12 | 6 | 180 | 45 µs | 0.27 % | 0.6 % of a core |
| 18 | 12 | 360 | 61 µs | 0.37 % | 1.8 % of a core |
| 24 | 18 | 540 | 76 µs | 0.46 % | 2.2 % of a core |

**The simulation held 60.0 Hz on every row**, including the last: a full grid
with eighteen connected players. Twenty-four times the cars costs the sidecar
about twice the tick and costs the database nothing measurable. Players cost
both, roughly a percentage point of a core per six. The percentages are one
machine and one build, so read the shape rather than the digits.

Reproduce the bottom rows with the load client. It holds one identity per
player; browser tabs share a stored token and would collide on a single row:

```bash
cargo run -p sidecar --release --bin load -- --players 18
```

### Holding 60 Hz

The loop sleeps to each tick boundary and spins the last little bit, because no
mainstream scheduler wakes a thread accurately enough on its own. How much to
leave for the spin is measured rather than guessed: the sidecar watches how late
its own sleeps land and shrinks the margin until they start landing late.

| spin margin | spin | tick rate | tick's own work | tick lands late, p99 |
|---|---|---|---|---|
| fixed 1.5 ms | 7.1 % of a core | 60.0 Hz | 32.7 µs | 35 µs |
| measured, settles near 0.5 ms | 1.5 % of a core | 60.0 Hz | 31.8 µs | 130 µs |

Windows 11, six bots, the spin timed inside the sidecar itself rather than read
off the operating system -- Windows charges CPU on a 15.6 ms sampling clock and
systematically under-reports a thread that runs for one millisecond and sleeps
for fifteen. A machine whose sleeps are tighter, which is most Linux boxes,
settles lower again and spins less.

The work inside a tick is the same either way; what changes is the core burnt
waiting for the next one. The trade is in the last column and it is the right
way round: a quarter of the CPU for a tick that occasionally lands a tenth of a
millisecond late, which nothing downstream can tell from one that did not. The
tick a snapshot carries is a number, not a timestamp.

### Predicting the other cars

Every remote car is carried forward from its newest snapshot to the tick the
client is simulating. Measured against the snapshots that arrive next, over six
bots racing:

| lead | straight line | constant turn rate |
|---|---|---|
| 3 ticks (50 ms) | 0.015 m | 0.009 m |
| 6 ticks (100 ms) | 0.056 m | 0.035 m |
| 12 ticks (200 ms) | 0.217 m | 0.139 m |

Rotating the velocity as the car is carried forward buys about a third, and it
buys most where it matters: mid-corner, at the lead a real round trip needs.
Both columns are far better than not predicting at all, which is not an error of
centimetres but of whole car lengths -- and a systematic one, always behind.

### Lines of code

The other cost is how much of this you have to write. `scc` over the
hand-written files, generated bindings excluded:

| | Code | |
|---|---|---|
| `module/src/lib.rs` | 431 | tables, the inbox, the registry, the outbox, the two authority checks |
| `sidecar/src/authority.rs` | 355 | one tick: sync the grid, schedule inputs, simulate and publish -- or stand by |
| `sidecar/src/main.rs` | 191 | connect, subscribe, and hold 60 Hz |
| **server side** | **977** | the module plus both sidecar files above |
| `web/src/sim.ts` | 332 | predict, rewind, replay, smooth |
| `web/src/net.ts` | 504 | subscriptions, clock estimate, remote prediction, reconnection |
| `web/src/main.ts` | 12 | the clock steering that holds the lead |
| **browser netcode** | **848** | the three web files above |

```bash
scc module/src/lib.rs sidecar/src/main.rs sidecar/src/authority.rs
scc web/src/sim.ts web/src/net.ts
```

For scale, `physics/src` is **1727** lines, roughly twice the server side. The
simulation is the biggest piece of the project, and the part you would write
wherever you ran it. `spacetime generate` emits another **3134** across 42 files
nobody reads.

## License

MIT.
