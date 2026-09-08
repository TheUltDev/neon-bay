# Physics sidecar for SpacetimeDB

A proof of concept for keeping **server authority** over a real-time physics
simulation while running none of that physics inside the database.

Running the simulation inside the module works, and SpacetimeDB is well able to
host it. The question is what it costs. Every tick is a transaction, and at
60 Hz per car that is throughput spent integrating tire forces, resolving
collisions and running bot AI rather than on everything else the database is
doing for you. The usual way to buy it back is to let clients own their own
positions, which trades the security model for frame rate.

This demo takes the third option. The physics moves into a **sidecar**: an
ordinary SpacetimeDB client that happens to be trusted, running the simulation
in its own process at a fixed 60 Hz and writing the results back through a
reducer only it is allowed to call. Players still send nothing but controller
readings. The database still holds the one true answer. It just stopped being
the thing that computes it.

The browser runs **the same simulation**, compiled from the same Rust source to
wasm, and predicts locally so steering is instant. When the authority disagrees,
the client rewinds, replays and slides back into line without a visible jump.

**Measured on a healthy local link: prediction error 0.000 m.** Not "small":
zero, because both sides execute identical instructions on identical inputs.

**[Play it](https://neon-bay.ult.workers.dev)** — client on Cloudflare's
edge, database and authority on Railway in `us-east4`. The zero holds there
too: 0.000 m across a ~90 ms round trip, because determinism does not care how
far away the authority is, only that it runs the same instructions. See
[Deploying it](#deploying-it).

**[How it works](https://neon-bay.ult.workers.dev/tech)** — the architecture
written up as a page, aimed at anyone who wants to build a sidecar of their own
rather than clone this one. It ships with the client, from `web/public/tech.html`.

---

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
an `rlib` linked into the native sidecar, and a `cdylib` for
`wasm32-unknown-unknown` loaded by the browser. Not a port, not a
reimplementation: one source of truth for what a car does.

## Layout

| Path        | What it is |
|-------------|------------|
| `physics/`  | The simulation. Deterministic `f32`, no dependencies. Vehicle dynamics, track, collisions, bot driver, and a bare C-ABI wasm bridge. |
| `module/`   | The SpacetimeDB module. Tables and reducers only: an inbox for inputs, a registry of who is racing, an outbox for authoritative state. |
| `sidecar/`  | The authority. A SpacetimeDB client that steps the world at 60 Hz, drives the bots, and publishes snapshots at 20 Hz. |
| `web/`      | The game. Vite + TypeScript, Canvas2D, no framework. Prediction, rollback, entity interpolation, and a telemetry HUD that shows all of it working. Sound is synthesized in the browser — an engine note and a chiptune soundtrack, no audio files to ship. `public/tech.html` is the architecture write-up served at `/tech`: one standalone file, no build step, linked from the join screen. |
| `scripts/`  | Setup, dev launcher, the native-vs-wasm determinism check, and the container entrypoint. |
| `Dockerfile` | The database and the sidecar in one image, for [deploying](#deploying-it). `railway.json` and `web/wrangler.toml` are the rest of it. |

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
`C` toggles a rotating camera.

**Version pinning matters here.** `spacetime generate` emits bindings for the
CLI's own version, and they have to compile against the SDK this project pins.
Those pins live in three files and move together:

| | |
|---|---|
| `module/Cargo.toml` | `spacetimedb = "2.8"` |
| `sidecar/Cargo.toml` | `spacetimedb-sdk = "2.8"` |
| `web/package.json` | `"spacetimedb": "~2.8.3"` |

Both setup scripts compare your CLI against those pins and warn if they have
drifted apart. Generated bindings are not committed, so run `setup` before
`cargo build` at the workspace root. `cargo test -p physics` needs nothing
generated and works on a fresh clone.

<details>
<summary>Running the pieces by hand</summary>

```bash
spacetime publish --server local --module-path module --delete-data=always --yes physics-sidecar
spacetime generate --lang rust       --out-dir sidecar/src/module_bindings --module-path module
spacetime generate --lang typescript --out-dir web/src/module_bindings     --module-path module

node scripts/build-wasm.mjs                     # cargo build + copy into web/public
cargo run -p sidecar --release -- --bots 6      # terminal 1
cd web && npm install && npm run dev            # terminal 2
```

While iterating on the physics, `cd web && npm run wasm` rebuilds and copies the
wasm without touching anything else; Vite picks it up on reload.
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
sidecar go to **Railway as a single container**: they talk on the hot path --
every input as it lands, twenty snapshots a second, at a fixed 60 Hz that
cannot slip -- so the link between them stays on loopback rather than becoming
a network. The browser client is static, so it goes to **Cloudflare Workers**
and is served from the edge; its only conversation is one `wss://` back to
Railway.

### The game server

`Dockerfile` builds all three native pieces in one pass -- module wasm,
generated bindings, sidecar -- and `scripts/railway-start.sh` supervises the
pair inside the container: start SpacetimeDB, wait for `/v1/ping`, publish the
module into it, start the authority, and bring the whole container down if
either half exits. Railway restarts it and the sidecar re-claims, resuming the
tick clock from `config.server_tick`.

```bash
railway login
railway init                # or `railway link` for an existing project
railway up                  # builds the Dockerfile and deploys
railway domain              # the public https:// URL
```

`railway.json` health-checks `/v1/ping`, restarts always, and pins the service
to `us-east4` at **one replica**. That last part is not a limitation waiting to
be fixed: `push_states` is guarded by a single registered identity, so a second
sidecar would be turned away by the module even if Railway ran one.

**Storage is deliberately ephemeral.** No volume is attached, so every deploy
starts from an empty data directory and the entrypoint republishes into it. Lap
records and player identities do not survive a push, which is the right trade
for a demo whose durable state is a lap time. To keep them, attach a Railway
volume at `/stdb` and set `RAILWAY_RUN_UID=0` -- the image runs as a non-root
user, and Railway mounts volumes root-owned.

Service variables:

| | |
|---|---|
| `PORT` | Set to `3000`. What the server listens on and what Railway's proxy forwards to; they have to agree. |
| `SIDECAR_BOTS` | AI drivers on the grid. Default 6, capped at 12 by the name list. |
| `SIDECAR_QUIET` | Set to anything to silence the once-a-second status line. |
| `STDB_DB` | Database name. Default `physics-sidecar`. |

One thing to be clear-eyed about: a public instance accepts public connections.
`claim_authority` is first-come and only transferable once the holder has
disconnected, so while the sidecar is up nobody else can take it -- but during
the restart window after a deploy, whoever connects first wins. On a demo that
is a curiosity. On anything real, sign the sidecar's identity into the module
rather than letting it be claimed.

### The client

The deployed client is not on the same host as the server, so it has to be told
where the server is. That lives in `web/.env.production`, committed rather than
ignored: it is a public address, and having it in the repo is what makes the
deployed client reproducible from a clone.

```bash
cd web
npm run deploy      # type check, Vite build, wrangler deploy
```

`wrangler.toml` declares an assets-only Worker -- no script, just `dist/` -- and
`npm run deploy` runs the type check and the Vite build before uploading, so a
broken build never ships. Two pages come out of it: the game at `/`, and the
write-up at `/tech`, which is `public/tech.html` copied through by Vite and
resolved without its extension by `html_handling = "auto-trailing-slash"`.
Everything that is not a real file still falls through to the game, so a deep
link into the client does not 404.

`?uri=` still overrides everything at runtime, which is the quickest way to
point a deployed page at a server on your desk:

```
https://<your-worker>.workers.dev/?uri=http://192.168.1.20:3000
```

## What to try

The HUD's **Prediction & reconciliation** panel is the demo. Everything below
is designed to move a number on it.

- **Just drive.** Prediction error sits at `0.000 m` and the graph reads
  `PREDICTION EXACT`. The rollback machinery is running, and "replayed ticks"
  is non-zero; it simply never finds anything to correct.
- **Drag *added latency* to 250 ms.** Watch "lead" climb from +5 to +19 ticks:
  the client automatically runs further ahead so its inputs still land on time.
  The car keeps steering instantly. Error stays in the millimeters.
- **Add *packet loss*.** Dropped inputs mean the authority holds the last
  controller state it heard, so it briefly disagrees. Corrections tick up, and
  the "smoothing out" figure shows the error being absorbed rather than snapped.
- **Press *Desync me*.** The client cheats: it adds 26 m/s of velocity the
  physics never granted it. The sidecar never sees it, because a client can
  only ever say "throttle down", so one round trip later the authority
  disagrees by over a meter and pulls the car back. Nothing teleports.
- **Watch the server ghost** (the dashed outline). That is the authority's most
  recent published pose, drawn next to your prediction. On a good link it trails
  by exactly the interpolation delay.

## How the netcode works

Three pieces, in the order they run.

**1. Predict.** Every tick, the client samples the controls, steps its local car
in wasm immediately, and stores `(tick, input, resulting state)` in a 256-entry
ring buffer. Steering has zero input lag regardless of ping.

**2. Reconcile.** Snapshots arrive at 20 Hz carrying the authority's state for
some tick `T` in the recent past. The client compares that against what it had
recorded for `T`. If they differ by more than a millimeter it rewinds the car to
the authoritative state and replays the stored inputs from `T+1` up to the
present, typically 5 to 20 ticks and about 20 µs of work.

`physics::tests::rollback_replay_is_bit_exact` pins the invariant this rests on:
re-simulating from an older state with the same inputs reproduces the same bits.

**3. Smooth.** A rewind moves the car, which would look like a stutter. Instead
the client keeps the difference between where the car *appeared* to be and where
it now *is* as a visual offset, and decays that offset to zero over ~220 ms. The
simulation is corrected instantly; the picture catches up smoothly. Corrections
larger than 9 m (a respawn, a long stall) snap and raise the "resyncs" counter,
because there is no hiding those.

**The clock.** For prediction to be exact, an input stamped for tick `T` must be
applied by the authority *at* tick `T`. So the client estimates where the
sidecar's clock is now, adds a round trip plus two ticks of margin, and steers
its own tick rate by up to ±6 % to hold that lead, a time dilation you cannot
feel. The sidecar, in turn, *schedules* inputs rather than applying them on
arrival: an input that shows up early waits for its tick. An input that shows up
late is applied immediately (there is no rewinding the authority) and the client
absorbs the difference on its next rollback.

**Other cars.** The client never simulates them. It interpolates their published
poses ~4 ticks in the past and parks them in the wasm world as immovable
colliders, so you can lean on a rival mid-corner and your own prediction reacts
at once, while never claiming to own their state. `World::step` takes a
simulation mask for exactly this: the sidecar passes every car, the browser
passes only its own.

## Determinism, and why it needed work

Prediction is only exact if both builds agree bit for bit. Two things get in the
way, and both are handled in `physics/src/math.rs`:

- **Transcendentals.** `libm`'s `sinf` on x86-64 and the one linked into a wasm
  module are not guaranteed to agree in the last ulp, and one ulp compounds over
  a few hundred ticks into a visible desync. So `sin`, `cos`, `atan` and `atan2`
  are hand-rolled polynomials evaluated with nothing but `+`, `-` and `*`, which
  are IEEE-754 exact everywhere. `sqrt` is correctly rounded by the standard and
  is used directly.
- **FMA contraction.** If the compiler fuses `a * b + c` into a single
  multiply-add, the native build rounds once where wasm (which has no FMA
  instruction) rounds twice. Rust does not contract by default, so just never
  build this workspace with `-C target-cpu=native`.

`node scripts/verify-determinism.mjs` runs the same 1200-tick scripted race
through both builds and diffs the raw `f32` bits:

```
All 10 checkpoints identical, bit for bit.
x86-64 and wasm32 agree, so a healthy client predicts with zero error.
```

## The car

Not a dot with a velocity. A lateral-slip bicycle model with longitudinal load
transfer, a friction ellipse per axle, speed-sensitive steering, aero downforce,
grip-proportioned braking, and a handbrake that unloads the rear so you can hold
a slide. Body collisions use two circles per car, so clipping a barrier with the
nose spins you the way it should.

Tuned against tests rather than vibes (`cargo test -p physics --release`):

| | |
|---|---|
| 0–100 km/h | ≈ 3.0 s |
| top speed | 238 km/h |
| steady-state cornering | 1.6 g, understeer-limited at the edge |
| circuit | 1424.7 m, 15–20 m wide, 12 checkpoints |
| bot lap times | 43.6 – 45.5 s, spread by driver skill |

The skidpad test asserts the car *understeers* at the limit. An
oversteer-limited car is undriveable with a keyboard, and it is a mistake you
make silently: an early version quietly canceled half its own cornering force
by applying the `ω × v` transport terms *and* re-projecting velocity through the
world frame each substep. It felt "twitchy" and every bot spun. The test now
pins 1.6 g so it cannot come back.

Nothing to configure for that: `claim_authority` deliberately leaves the bot
cars alone, and the sidecar reconciles the field to its `--bots` count itself.

## What it costs

Sidecar, 7 cars including 6 AI drivers, measured on the status line it prints
once a second:

```
[t   25483] 60.0 Hz sim | 20.0 Hz snap | 7 cars (6 bots) | 6.1 us/tick | 0.0% of budget
```

Six microseconds against a 16.7 ms budget. The wasm build runs 10 000 ticks in
10 ms in Node, so a 20-tick rollback costs about 20 µs, cheap enough that the
client can afford one on every snapshot without thinking about it.

The whole physics core is a **38 KB** `.wasm` with zero imports: no
wasm-bindgen, no wasm-pack, no build plugin. `CarState` is `#[repr(C)]` and
entirely `f32`, so the browser maps it with a single `Float32Array` over wasm
linear memory and reads and writes the simulation in place, with no serialization
on the hot path.

### And in lines

The other cost worth quoting is how much of this you have to write. `scc` over
the hand-written files, generated bindings excluded:

| | Code | |
|---|---|---|
| `module/src/lib.rs` | 465 | tables, the inbox, the registry, the outbox, the identity check |
| `sidecar/src/authority.rs` | 303 | one tick: sync the grid, schedule inputs, simulate, publish |
| `sidecar/src/main.rs` | 226 | connect, subscribe, claim, and hold 60 Hz |
| **the authority** | **994** | everything that makes the server the server |
| `web/src/sim.ts` | 337 | predict, rewind, replay, smooth |
| `web/src/net.ts` | 317 | subscriptions, clock estimate, entity interpolation |
| `web/src/main.ts` | 12 | the clock steering that holds the lead |
| **the netcode** | **666** | everything that hides the round trip |

```bash
scc module/src/lib.rs sidecar/src/main.rs sidecar/src/authority.rs
scc web/src/sim.ts web/src/net.ts
```

For scale: `physics/src` is **1609** lines. The simulation is larger than
everything that makes it authoritative, and it is the part you would have to
write wherever you chose to run it. `spacetime generate` emits another **3100**
across 42 files that nobody reads.

Narrow it further and the trust model itself -- `claim_authority`,
`release_authority`, `require_authority`, `set_input` and `push_states`
together -- is **104 lines**, a quarter of them `push_states` copying a struct
field by field. The security of the whole arrangement rests on that much code
rather than on a codebase, so you can read all of it before deciding to trust it.

## The trust model

A client can call exactly one hot-path reducer, `set_input`, and it carries
throttle, steering, brake and handbrake. There is no reducer that lets a player
assert a position, a lap, or a time. `push_states` checks the caller against the
identity registered in `config`, which is claimed first-come and only
transferable once the holder has disconnected.

So the trust boundary did not disappear, it *moved*: from "the database is the
only thing that can be trusted" to "the database plus one identity you control".
That is the trade this architecture makes, and it is worth being explicit about
it. In exchange, the simulation becomes a normal process, one you can profile,
scale horizontally by sharding the world, or restart without touching the data.

## Known limitations

This is a proof of concept. Honest gaps:

- **One authority, but handover is clean.** Kill the sidecar and start it again:
  it re-claims, resumes the tick clock from `config.server_tick`, and adopts
  every car from its last published pose: position, velocity, steering angle,
  lap, the lot. Because the snapshot carries the whole simulation state, drivers
  keep their momentum through a restart. What is missing is a *hot* standby:
  there is a visible gap while the process comes up, and nothing arbitrates
  between two sidecars racing to claim after a network partition.
- **`input` is a public table** so the sidecar can subscribe to it. Real
  deployments would put a row-level security filter on it so players cannot read
  each other's controls.
- **Rollback ignores contact history.** Replayed ticks collide against remote
  cars at their *current* interpolated poses, not their historical ones. Standard
  practice, and the resulting error is small and smoothed, but it is an
  approximation, unlike everything else here.
- **No lag compensation for contact.** Cars are interpolated ~70 ms in the past;
  a side-by-side pass is resolved by the authority against its own timeline.
- **No anti-cheat on input rate.** A client could spam `set_input` faster than
  30 Hz. The scheduler bounds the damage to "very responsive controls".
- **Busy-waiting on the tick boundary.** No mainstream scheduler wakes a thread
  accurately enough for 60 Hz, so the sidecar sleeps until 1.5 ms out and then
  spins. That buys a rock-steady 60.0 Hz for a little CPU; Windows needs it
  most, but Linux and macOS overshoot too.

## Poking at it

```bash
cargo test -p physics --release -- --nocapture   # physics + netcode invariants
node scripts/verify-determinism.mjs              # native vs wasm, bit for bit
node scripts/build-wasm.mjs                      # rebuild just the browser core
spacetime sql --server local physics-sidecar "SELECT car_id, tick, x, y, lap FROM car_state"
spacetime logs --server local physics-sidecar
```

In the browser console, `__neon` exposes `{ sim, net, renderer, hud, controls }`:
`__neon.sim.stats`, `__neon.net.rttMs`, `__neon.sim.cheat(30)`.

## License

MIT.
