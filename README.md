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
same instructions.

**[How it works](https://neon-bay.ult.workers.dev/tech).** The architecture
written up as a page, for anyone who wants to build a sidecar of their own
rather than clone this one.

**[The car](PHYSICS.md).** What the simulation itself models: tires,
drivetrain, load transfer, contact and damage, the driver aids, and the numbers
the tests pin.

**[The netcode](NETCODE.md).** How the browser predicts, reconciles and
smooths, how it simulates the other cars rather than guessing at them, and what
each of those costs.

**[Running and deploying it](DEVOPS.md).** Setup, the dev loop, the container,
the edge deploy, failover, and the commands for testing and poking at it.

**[What could be better](IMPROVEMENTS.md).** The gaps in this proof of concept,
as work that could be done.

The rest of this file is the sidecar pattern: the shape, what it costs, what
it trusts, why the two builds agree, and how it scales.

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
  gave it, and spins its wheels up to match, because a body doing 26 m/s more
  than its tires are turning is not a cheat, it is four locked wheels. The
  sidecar never sees any of it, because a client can only say "throttle down", so
  by the time a round trip is out the authority disagrees by two or three metres
  and pulls the car back. Nothing teleports.
- **Hit somebody.** The dent stays for the rest of the lap, and it is on the
  wire, so every browser in the race sees the same wreck. It costs you
  downforce, steering lock, engine power and grip until you cross the line, and
  the *BODY* readout on the dash says how much of the car is left. The contact
  and crush models are in [PHYSICS.md](PHYSICS.md).
- **Follow somebody into a corner and watch "rival guess out by".** That is the
  other half of the panel and a different kind of number from the one above it.
  Prediction error is the client re-running its own car and having to agree with
  the authority to the millimetre; this is the client guessing what somebody
  *else* was about to do with the pedals. It sits in the centimetres and it is
  never zero, because it cannot be.
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
| `physics/`  | The simulation. Deterministic `f32`, no dependencies: a four-wheel vehicle model, its drivetrain, load transfer and aero, box contacts with a crush model, the track, the bot driver, and a bare C-ABI wasm bridge. [PHYSICS.md](PHYSICS.md) describes it. |
| `module/`   | The SpacetimeDB module. Tables and reducers only: an inbox for inputs, a registry of who is racing, an outbox for authoritative state. |
| `sidecar/`  | The authority. A SpacetimeDB client that steps the world at 60 Hz, drives the bots, and publishes snapshots at 20 Hz. `src/bin/load.rs` is a second binary: synthetic players, for measuring what a client costs the database. |
| `web/`      | The game. Vite + TypeScript, no framework, drawn on WebGPU, WebGL 2 or Canvas2D depending on the browser. Prediction, rollback, prediction of the other cars too, and a telemetry HUD that shows all of it working. Sound is synthesized, so there are no audio files to ship. `public/tech.html` is the architecture write-up served at `/tech`. |
| `scripts/`  | Setup, dev launcher, the native-vs-wasm determinism check, and the container entrypoint. |
| `Dockerfile` | The database and the sidecar in one image, for [deploying](DEVOPS.md#deploying-it). `railway.json` and `web/wrangler.toml` are the rest of it. |

## What it costs

Sidecar, 7 cars: six AI drivers and one connected player at 30 inputs a second.
Measured on the status line it prints once a second:

```
[t   31367] 60.0 Hz sim | 19.7 Hz snap | 7 cars (6 bots) |  71.1 us/tick |  0.4% of budget | 30 inputs/s
```

Seventy-one microseconds of a 16 667 µs budget, four tenths of one percent. That
is the whole tick: pumping the connection, reading the inbox, driving the bots,
stepping the world, and publishing every third tick. **The simulation is 31 µs
of it and the bot AI another 2.** Most of the rest is the SDK.

Those 31 µs are the price of a four-wheel vehicle model with Magic Formula
tires and contacts solved on the same 480 Hz clock, and it is not a small one:

| cars | physics | bot AI | total | of a 60 Hz budget |
|---|---|---|---|---|
| 1  |   4.2 µs | 0.4 µs |   4.6 µs | 0.03% |
| 7  |  30.5 µs | 2.5 µs |  33.1 µs | 0.20% |
| 16 |  72.4 µs | 6.1 µs |  78.5 µs | 0.47% |
| 24 | 111.5 µs | 9.0 µs | 120.6 µs | 0.72% |

Which is the argument, not a caveat. That is exactly the kind of work you do not
want inside a transaction, and moving it out costs the module nothing at all: it
has no opinion about tire models, because it does not depend on the physics
crate. `cargo run -p physics --example probe --release` reproduces the table.

**The database's cost does not grow with the simulation's complexity.** A car
five times harder to simulate changes nothing about what the database does:
24 cars and 1 car are the same 20 write transactions a second,
because every car rides in one `push_states` call. What it does change is the
size of that call: the row is the 46-float record plus a handful of scalars,
about 190 bytes, and it costs tenths of a percent of a core, 0.2 % for a six-car
grid against 0.09 % idle and 0.6 % for twenty-four.

Players cost it more than cars do. The full sweep, up to a 24-car grid with
eighteen of them connected, is in [The numbers](#the-numbers).

## The trust model

A client can call exactly one hot-path reducer, `set_input`, and it carries
throttle, steering, brake and handbrake. There is no reducer that lets a player
assert a position, a lap, or a time. It is rate limited to 45 writes a second
per identity with a burst of 15 banked, which the browser's 30 never comes near;
anything past that is dropped before the row is written, and so before any
subscriber pays to hear about it.

A player cannot *read* the inbox either. `input` is a private table, and
SpacetimeDB shows one of those to the database's owner and to nobody else. A
subscription from anyone else is refused with "no such table". The sidecar is
that owner, connecting as the identity that published the module.

Two checks guard the outbox, and they answer different questions. *May* you be
the authority: `claim_authority` compares the caller against the owner recorded
in `config` when the module was published, so nobody can appoint themselves
during a handover. *Are* you the authority right now: `push_states` compares the
caller's **connection** against the recorded lease holder. A connection rather
than an identity, because a standby runs on the same credentials as the sidecar
it is standing by for. It also makes the check a fencing token, so a sidecar
that was replaced while it was partitioned has its writes refused the moment it
comes back rather than scribbling over a race it no longer runs.

The whole mechanism is **112 lines**: `claim_authority`, `release_authority`,
`require_authority`, `spend`, `set_input` and `push_states`. Twenty-three of
those are `push_states`, which only has to check the caller, drop anything stale and
write. You can read all of it before deciding to trust it.

The trust boundary did not disappear, it *moved*: from "the database is the only
thing that can be trusted" to "the database plus the identity that published the
module". That is the trade. In exchange the simulation becomes a normal process,
one you can profile, shard, or restart without touching the data.

## Determinism

Prediction is only exact if both builds agree bit for bit. Two things get in the
way, and both are handled in `physics/src/math.rs`:

- **Transcendentals.** `libm`'s `sinf` on x86-64 and the one linked into a wasm
  module need not agree in the last ulp, and one ulp compounds over a few hundred
  ticks into a visible desync. So `sin`, `cos`, `tan`, `atan` and `atan2` are
  hand-rolled polynomials using only `+`, `-` and `*`, which are IEEE-754 exact
  everywhere. `sqrt` is correctly rounded by the standard, so it is used directly.
  `max` and `min` are spelled out as comparisons for the same reason: the library
  versions disagree with wasm's `f32.max` about NaN, so one target gets a fix-up
  the other does not.
- **FMA contraction.** If the compiler fuses `a * b + c` into one multiply-add,
  the native build rounds once where wasm, which has no FMA instruction, rounds
  twice. Rust does not contract by default, so never build this workspace with
  `-C target-cpu=native`.

`node scripts/verify-determinism.mjs` runs the same 1200-tick scripted race
through both builds and diffs the raw `f32` bits:

```
ok   native fp   7fffd801
     wasm   fp   7fffd801
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

## The numbers

### Scaling

Players are the term that grows, because a player is a connection that writes:
30 Hz of `set_input` each, plus a subscription to the snapshots coming back.

| cars | players | inputs/s | sidecar tick | of budget | database |
|---|---|---|---|---|---|
| 1 | 0 | 0 | 32 µs | 0.19 % | 0.27 % of a core |
| 6 | 0 | 0 | 62 µs | 0.37 % | 0.18 % of a core |
| 12 | 0 | 0 | 96 µs | 0.57 % | 0.36 % of a core |
| 24 | 0 | 0 | 152 µs | 0.91 % | 0.62 % of a core |
| 12 | 6 | 180 | 104 µs | 0.62 % | 0.85 % of a core |
| 18 | 12 | 360 | 165 µs | 0.99 % | 0.51 % of a core |
| 24 | 18 | 540 | 235 µs | 1.41 % | 1.03 % of a core |

The sidecar column was measured without the four controller fields `car_state`
carries. They add thirteen bytes to a roughly 190-byte row and four field copies
per car per snapshot, comfortably inside the factor-of-two spread described
below.

**The simulation held 60.0 Hz on every row**, including the last: a full grid
with eighteen connected players, at one and a half percent of the tick budget.
Twenty-four times the cars costs the sidecar about five times the tick, which is
what it looks like when the simulation is most of what a tick does rather than a
rounding error next to the SDK. The sidecar column is the median of a minute's
status lines; the spread on a single row is a few microseconds either way, so
read the last digit as noise.

Treat the database column as an order of magnitude rather than a digit. Every
figure is a fraction of a percent of a core against an idle floor of **0.09 %**,
which is close enough to the resolution of the operating system's own CPU
accounting that the rows do not come out reliably ordered: one car reads above
six, and eighteen cars with twelve players read below twelve cars with six, over
70-second windows. Repeating a row moves it by up to a factor of two. What
survives that noise is the shape, and the shape is the point: cars are nearly
free because however many there are they arrive in one transaction, and players
are not, because each one writes thirty times a second.

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
| fixed 1.5 ms | 7.2 % of a core | 60.0 Hz | 58.0 µs | 0 µs |
| measured, settles near 0.5 ms | 1.5 % of a core | 60.0 Hz | 57.7 µs | 119 µs |

Windows 11, six bots, the spin timed inside the sidecar itself rather than read
off the operating system, which charges CPU on a 15.6 ms sampling clock and
systematically under-reports a thread that runs for one millisecond and sleeps
for fifteen. A machine whose sleeps are tighter, which is most Linux boxes,
settles lower again and spins less.

The work inside a tick is the same either way, the two rows landing a third of a
microsecond apart, which is the point of measuring both. What changes is the core
burnt waiting for the next one. The trade is in the last column and it
is the right way round: a fifth of the CPU for a tick that occasionally lands a
tenth of a millisecond late, which nothing downstream can tell from one that did
not. The tick a snapshot carries is a number, not a timestamp.

### Lines of code

The other cost is how much of this you have to write. `scc` over the
hand-written files, generated bindings excluded:

| | Code | |
|---|---|---|
| `module/src/lib.rs` | 457 | tables, the inbox, the registry, the outbox, the two authority checks |
| `sidecar/src/authority.rs` | 420 | one tick: sync the grid, schedule inputs, simulate and publish, or stand by |
| `sidecar/src/main.rs` | 189 | connect, subscribe, and hold 60 Hz |
| **server side** | **1066** | the module plus both sidecar files above |
| `web/src/sim.ts` | 448 | predict the grid, rewind, replay, smooth |
| `web/src/net.ts` | 525 | subscriptions, clock estimate, the spectator view, reconnection |
| `web/src/main.ts` | 12 | the clock steering that holds the lead |
| **browser netcode** | **985** | the three web files above |

```bash
scc module/src/lib.rs sidecar/src/main.rs sidecar/src/authority.rs
scc web/src/sim.ts web/src/net.ts
```

For scale, `physics/src` is **4276** lines across fifteen files, four times
the server side. The simulation is by far the biggest piece of the project,
and the part you would write wherever you ran it, which is the argument for being
able to run it wherever you like. It is also the argument for running the *same*
one on both sides: the browser predicts every rival through it, not around it.
`spacetime generate` emits another **3264** across 42 files nobody reads.

## License

MIT.
