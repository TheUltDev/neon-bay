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
  gave it, and spins its wheels up to match, because a body doing 26 m/s more
  than its tires are turning is not a cheat, it is four locked wheels. The
  sidecar never sees any of it, because a client can only say "throttle down", so
  by the time a round trip is out the authority disagrees by two or three metres
  and pulls the car back. Nothing teleports.
- **Hit somebody.** Contact is nearly plastic, the way a car-to-car impact
  really is: at 70 km/h of closing speed one percent of it comes back and the
  rest goes into the shape of both cars. The dent stays for the rest of the lap,
  and it is on the wire, so every browser in the race sees the same wreck. It
  costs you downforce, steering lock, engine power and grip on the corner that
  took it, and the *BODY* readout on the dash says how much of the car is left.
  Cross the line and you get a fresh one.
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
| `physics/`  | The simulation. Deterministic `f32`, no dependencies. Pacejka tires (`tire.rs`), wheel and slip dynamics (`wheel.rs`), engine and gearbox (`drivetrain.rs`), load transfer (`suspension.rs`), aero (`aero.rs`), box contacts (`collide.rs`), crush and damage (`damage.rs`), the track, the bot driver, and a bare C-ABI wasm bridge. |
| `module/`   | The SpacetimeDB module. Tables and reducers only: an inbox for inputs, a registry of who is racing, an outbox for authoritative state. |
| `sidecar/`  | The authority. A SpacetimeDB client that steps the world at 60 Hz, drives the bots, and publishes snapshots at 20 Hz. `src/bin/load.rs` is a second binary: synthetic players, for measuring what a client costs the database. |
| `web/`      | The game. Vite + TypeScript, no framework, drawn on WebGPU, WebGL 2 or Canvas2D depending on the browser. Prediction, rollback, prediction of the other cars too, and a telemetry HUD that shows all of it working. Sound is synthesized, so there are no audio files to ship. `public/tech.html` is the architecture write-up served at `/tech`. |
| `scripts/`  | Setup, dev launcher, the native-vs-wasm determinism check, and the container entrypoint. |
| `Dockerfile` | The database and the sidecar in one image, for [deploying](#deploying-it). `railway.json` and `web/wrangler.toml` are the rest of it. |

## What it costs

Sidecar, 7 cars: six AI drivers and one connected player at 30 inputs a second.
Measured on the status line it prints once a second:

```
[t   31367] 60.0 Hz sim | 19.7 Hz snap | 7 cars (6 bots) |  71.1 us/tick |  0.4% of budget | 30 inputs/s
```

Seventy-one microseconds of a 16 667 µs budget, four tenths of one percent. That
is the whole tick: pumping the connection, reading the inbox, driving the bots,
stepping the world, and publishing every third tick. **The simulation is 27 µs
of it and the bot AI another 2.** Most of the rest is the SDK.

Those 27 µs are the price of the vehicle model above, and it is not a small
one. A lumped car, one bicycle with a friction circle per axle stepped twice a
tick, does the same seven in **5.6 µs** on this machine. Four Magic Formula
evaluations per wheel-substep, at eight substeps, with the contacts solved on the
same clock as the tires, is about **five times** the arithmetic:

| cars | physics | bot AI | total | of a 60 Hz budget |
|---|---|---|---|---|
| 1  |  3.7 µs | 0.4 µs |  4.1 µs | 0.02% |
| 7  | 27.4 µs | 2.4 µs | 29.8 µs | 0.18% |
| 16 | 63.1 µs | 5.6 µs | 68.7 µs | 0.41% |
| 24 | 99.9 µs | 8.8 µs |108.7 µs | 0.65% |

Which is the argument, not a caveat. Five times the arithmetic is exactly the
kind of work you do not want inside a transaction, and moving it out costs the
module nothing at all: it has no opinion about tire models, because it does not
depend on the physics crate. `cargo run -p physics --example probe --release`
reproduces the table.

The wasm build replays 10 000 single-car ticks in 70 ms in Node, so a 20-tick
rollback costs about **140 µs**: a replayed tick resolves contact eight times as
well as stepping the car, and it is still cheap enough that the client can afford
one on every snapshot without thinking about it.

The whole physics core is a **56 KB** `.wasm` with zero imports: no
wasm-bindgen, no wasm-pack, no build plugin. `CarState` is `#[repr(C)]` and all
`f32`, forty-four of them, so the browser maps it with one `Float32Array` over
wasm memory and reads and writes the simulation in place. Nothing is serialized
on the hot path.

**The database's cost does not grow with the simulation's complexity.** A car
five times harder to simulate changes nothing about what the database does:
24 cars and 1 car are the same 20 write transactions a second,
because every car rides in one `push_states` call. What it does change is the
size of that call: the row is 44 floats, and it costs tenths of a percent of a
core, 0.2 % for a six-car grid against 0.09 % idle and 0.6 % for twenty-four.

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

## How the netcode works

Three pieces, in the order they run.

**1. Predict.** Every tick, the client samples the controls, steps its local car
in wasm immediately, and stores `(tick, input, resulting state)` in a 256-entry
ring buffer. Steering has zero input lag regardless of ping.

**2. Reconcile.** Snapshots arrive at 20 Hz carrying the authority's state for
a tick `T` in the recent past. The client compares it against what it recorded
for `T`. If they differ by more than a millimeter it rewinds to the authoritative
state and replays the stored inputs from `T+1` to now, typically 5 to 20 ticks
and 20 to 90 µs of work.

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
local car is deliberately in the future, far enough ahead that its input reaches
the authority before the tick it belongs to, so holding rivals a few ticks in the
*past*, which is what an interpolation buffer does, puts them wrong by the sum of
the two.
At racing speed that is several metres, systematically, in the direction of
travel: you would be leaning on a car the authority had somewhere else. So each
rival is carried forward from its newest published pose to the tick actually
being simulated, at a constant turn rate, which follows a car through a corner
instead of flying off the tangent. Measured against the snapshots that arrive
next, that guess is out by 0.02 m over a tenth of a second of lead. Whatever
error a snapshot does reveal is kept as a fading offset rather than a jump --
the same trick the local car plays after a rollback.

Rivals are in the wasm world as colliders the client may not move, so you can
lean on one mid-corner and your prediction reacts at once without ever claiming
to own its state. `World::step` takes a simulation mask for this: the sidecar
passes every car, the browser passes only its own. A rollback puts the rivals
back where they were at each replayed tick, so what gets replayed is the contact
the authority actually resolved.

**A rival the client may not move is not a rival that weighs nothing**, and the
two are easy to confuse. Saying the first to a solver by giving the car infinite
mass says the second as well, and an infinite mass returns the whole impulse: the
local car rebounds off something it should have shoved aside. So the solve uses
both cars' real mass and discards the answer for the one this process does not
own, because the authority's answer for that car is already in flight. Only the
positional repair, which is arithmetic rather than physics, treats it as
immovable. What the hit did to the rival is remembered for one tick, so a single
contact cannot fire on all eight substeps against a car that never reacts.

How much of that matters depends entirely on how fresh the rival's pose is, and
it is worth being plain about: carried onto the tick being simulated, which is
what the paragraph above buys, a plastic impact leaves two equal cars at the same
speed and either treatment lands in the same place. Against a rival six ticks
stale, real mass is 9.4 m/s and 3.97 m out where infinite mass is 10.2 m/s and
4.13 m. Both are bad, which is the argument for predicting rivals forward rather
than interpolating them behind.

So, with rivals on the tick being simulated, one round trip after the hit,
against what the authority resolved:

| closing | authority | client predicted | error |
|---|---|---|---|
|  6 m/s | 22.49 m/s | 22.25 m/s | 0.24 m/s, 0.08 m apart |
| 12 m/s | 25.86 m/s | 25.50 m/s | 0.36 m/s, 0.07 m apart |
| 20 m/s | 30.48 m/s | 29.64 m/s | 0.84 m/s, 0.03 m apart |
| 30 m/s | 34.94 m/s | 34.73 m/s | 0.20 m/s, 0.02 m apart |

`cargo run -p physics --example crash --release` reproduces it, and
`physics::tests::a_client_predicts_the_hit_the_authority_resolves` fails if it
ever stops being true.

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

## The car

Not a dot with a velocity, and not a lumped approximation of one either. Four
wheels, each with its own angular velocity, each pressed into the road by a
load that four separate things are arguing over.

The tire is a **Pacejka Magic Formula**, evaluated per wheel, per substep, for
combined slip. Three properties of real rubber come out of it that a linear
cornering-stiffness model cannot produce at all, and every one of them is
something a driver feels:

- **Grip peaks and then falls.** Past about eight degrees of slip, asking for
  more gives you less. That is the difference between a slide and a larger
  cornering force.
- **Sliding grip is below peak grip**: 71% of it longitudinally, 85%
  laterally. This is the entire reason ABS is worth having, and the reason
  locking the fronts means you go straight.
- **Grip is sub-linear in load.** Doubling the weight on a tire does not double
  what it can do, so *how* the load is spread across four patches decides the
  car's balance and not just its total grip.

Everything else exists to feed that. Load transfer runs through roll and pitch
as real second-order systems, so weight takes about 90 ms to move and the
front-to-rear split is set by the **roll stiffness distribution**. Stiffen the
front bar and the car understeers, for the reason a real one does. Each wheel
has rotational inertia, so **slip ratio is a real quantity**: wheelspin is the
rear wheels genuinely outrunning the road, lock-up is a wheel at zero rad/s
while the car keeps moving, and the handbrake needs no special case at all.
Behind it is an **engine torque curve** through a slipping clutch, a six-speed
box that cuts torque to shift, and a **limited-slip differential** that sends
torque away from the wheel that is spinning up. ABS and traction control are
modelled because the car has them, not to paper over anything.

None of the behaviour above is a case in a list. Understeer, power oversteer,
lift-off oversteer, engine braking and the way all of them change with speed are
consequences of those parts, and the tests measure them rather than assert them
(`cargo test -p physics --release`):

| | |
|---|---|
| 0-100 km/h | 4.1 s, traction-limited off the line |
| 0-200 km/h | 12.6 s |
| top speed | 277 km/h, where power meets drag rather than where the gearing runs out |
| 50 m/s to a standstill | 82 m, 1.48 g average, ABS holding the wheels short of lock |
| peak lateral | 1.28 g at 20 m/s, 1.43 g at 60 m/s; the difference is downforce |
| at the limit | front slip 9.5°, rear 5.3°: understeer-limited, on purpose |
| body movement | 2.97° of roll and 1.81° of pitch per g, 94 ms to get there |
| engine | 425 N·m, 275 kW at 7200 rpm, six speeds and a reverse |
| circuit | 1424.7 m, 15-20 m wide, 12 checkpoints |
| bot lap times | 48.5 to 52.0 s, spread by driver skill and by how much of the car is left |

The skidpad test *sweeps* the steering to find the limit rather than reading one
fixed input, because with a real tire curve more lock past the peak buys less
lateral force: a fixed input measures understeer, not grip. It pins the
understeer balance too, because an oversteer-limited car is undriveable with a
keyboard.

`cargo run -p physics --example probe --release` prints the whole envelope:
standing start, peak lateral g against speed, and what a tick costs.

### Two aids, and only two

Both are there because a keyboard is not a car, and both are named as aids in
the source so nobody mistakes them for physics.

`steer_lock` tapers the available lock with speed. A real rack has a fixed
ratio, but a real driver also has a wheel with 900 degrees of travel and two
hands on it; a key is down or up.

`reverse_assist` brakes for you when you ask for a direction the car is not
going in. A gearbox will not select reverse above a crawl, correctly, since the
ratio is short enough that engaging it at speed would put the engine past the
limiter backwards. A negative pedal is a request for a *direction* and not a
negative torque. Between those two facts sits a car that will not do what it is
told: hold the reverse key while still rolling forward and nothing happens at
all, the car coasting on rolling resistance until it happens to be slow enough,
with nothing about that to suggest that what is wanted is the brake. So the car
brakes itself until it is not going anywhere, and the gearbox does the rest.

It lives in the physics rather than in the browser's key mapping for two reasons.
The bots need it, because a bot nose-first into a barrier asks for exactly this.
And an aid that lives only in the client is an aid the authority has to be
trusted to agree with.

Everything else that looks like an aid, ABS and traction control, is modelled
because the car has it.

## Crashing it

A car is not a billiard ball. Steel that folds does not give the energy back, so
a real car-to-car impact is *mostly plastic*, and the harder it is the more
plastic it gets. One coefficient of restitution for every speed gets that exactly
backwards where it matters, by making a 100 km/h shunt bounce like a 5 km/h
one.

What actually happens to steel is in `physics/src/damage.rs`, and it is the
model accident reconstruction uses. Campbell's observation is that residual
crush is **linear in impact speed**, `v = B0 + B1·C`, where `B0` is the speed a
car shrugs off entirely (2 m/s, near enough the 5 mph bumper standard) and `B1`
is the crush per metre per second past it. Integrating the force that implies
gives the energy a given depth of crush has absorbed; inverting it gives the
depth a given amount of absorbed energy produces. Damage accumulates through
the *energy* rather than by adding depths, so the structure stiffens as it folds
and two 30 kJ hits leave the dent that one 60 kJ hit does.

Restitution then falls out of the same sentence instead of being a second,
independent knob. If everything up to `B0` is elastic and everything past it
goes into bending metal, the fraction of the energy returned is `(B0/v)²`, so
**e = B0/v**. That is derived rather than fitted, and it lands within a few
hundredths of the published curves: 0.20 at 10 m/s against Antonetti's 0.24,
0.10 at 20 m/s against 0.10.

Two cars, one rear-ending the other, both coasting:

| closing | separation | of the closing speed | at a fixed e = 0.35 | absorbed | nose |
|---|---|---|---|---|---|
|  6 m/s | 0.29 m/s | 5 % |  8 % |  10 kJ | 0.05 m |
| 12 m/s | 0.19 m/s | 2 % | 20 % |  37 kJ | 0.15 m |
| 20 m/s | 0.17 m/s | 1 % | 24 % | 111 kJ | 0.29 m |
| 30 m/s | 0.04 m/s | 0 % | 27 % | 256 kJ | 0.45 m, which is all of it |

The fourth column is what a fixed coefficient gives for the same four hits, and
it is the whole problem in one column: it gets *bouncier* the harder the impact.
At 108 km/h of closing speed it throws the two cars apart at 8 m/s.

Bodies meet as **oriented boxes**: a separating-axis test, the incident face
clipped against the reference one for a two-point manifold, and a
sequential-impulse solver with accumulated clamping. Two points is what a flat
contact needs: one can only push, two can push *and* resist a twist, which is
the difference between scraping along a barrier and pirouetting off it.

Those contacts are solved **every substep**, on the same 480 Hz clock as the
tires. Resolving them once at the end of a tick instead leaves two cars closing
at 30 m/s half a metre inside each other before anything is done about it, and
half a metre in, the shallowest separating axis is not reliably the one you drove
in along. Solved with the tires, the deepest overlap a hit reaches is 5 mm at
30 m/s of closing speed and 13 mm at 45. The one thing still sampled at 60 Hz is
*where the barrier is*: that is a search through the centreline, it is the most
expensive thing a tick does, and a barrier does not move, so the search runs once
and the contact it feeds runs eight times.

**Damage is simulation state, not decoration.** Four numbers on the wire, the
residual crush in metres on each face of the body, and every one of them changes
what the car can do:

- a folded nose has no splitter left, so it makes a fraction of the front
  downforce and a good deal of drag it did not have before;
- the rack loses lock, and bent geometry pulls towards the side that took the
  hit, so the car has to be held straight;
- the radiator is wearing its own condenser, so the engine gets less air;
- a bent corner rubs its own bodywork, so that tire has less grip.

It is on the wire because rollback needs it, since a client replaying without it
is replaying a car that is not the one being corrected, and because it is also,
directly, the shape the renderer draws. Sixteen points instead of eight, so a
dent can pucker an edge in the middle rather than shrink the whole car, and all
three backends get it from the same function.

**Completing a lap repairs the car.** The start/finish straight is where a pit
lane would be and this circuit has not got one, so crossing the line is the stop
you never had to make. Without it damage is a one-way ratchet: *Respawn* clears
it on demand, but a bot has no thumbs, and a driver who has not found the button
spends the rest of the race in whatever they made of the first corner. A lap is
the right clock for it. Fifty seconds is long enough that a shunt is something
you have to drive around and short enough that nobody is stuck with one, and five
fields of eight bots racing for two and a half minutes finish at a mean severity
of 0.00 to 0.03.

A scrape is not a crash, and the model has to know the difference or a car is
written off by a long graze down a barrier. The solver measures the two
separately: the energy a contact destroys *head-on*, which is normal impulse
doing work and is zero for a contact that is merely holding station, and the
energy it destroys *sliding*. The first folds panels. The second wears them, at
a fraction of the rate and under a ceiling of 10 cm, because sliding contact
takes your flank off and not your width.

`cargo run -p physics --example crash --release` prints all of it.

## Drawing it

Three renderers, one picture. The client asks for **WebGPU**, falls back to
**WebGL 2**, then to **Canvas2D**. The Renderer panel says which one you got and
on what hardware, and its three buttons change tier in place: no reload, the
camera does not move, and a tier this browser will not give you is disabled and
says why on hover. `?renderer=` pins a tier from the URL.

The two GPU tiers are one renderer, not two. `web/src/render/gpu.ts` builds a
frame's worth of triangles and hands them to a device interface that WebGL 2 and
WebGPU implement in 269 and 333 lines respectively. The track is triangulated once at load
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
body had rolled. All of it is on the wire for exactly this reason.

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
  authority computes are the same arrangement of cars. That is what keeps a
  predicted shunt within a metre a second of the one the authority resolves; a
  client working from a rival six ticks stale is nine metres a second out, which
  is the same accident with a different outcome. What is left is that the
  prediction is a guess: 0.02 m out at a tenth of a second of lead, and more when
  a rival brakes hard or is hit. Real lag compensation, where the authority
  rewinds every rival into the view the toucher had, is a much larger piece of
  work, and one most racing games decline: two cars cannot both be right about a
  mutual impulse.
- **The rate limiter still runs the reducer.** A flood is dropped before it
  writes a row or fans one out, which is the part that costs everyone else, but
  the transaction is still opened. Bounding *that* is the host's job, not the
  module's.
- **The tick boundary spins.** The sidecar measures how late its own sleeps
  land and spins only that much, 1.5 % of a core on the machine these numbers
  come from against 7.2 % for a fixed margin wide enough to cover it. A spin is
  still a spin, and getting to zero needs a timer the operating system does not
  portably offer.
- **One track, 24 slots, one database.** The grid size is a constant shared by
  the module and the physics crate, and the sidecar refuses to start if the two
  disagree. Sharding across databases is not attempted.

## Poking at it

```bash
cargo test -p physics --release -- --nocapture   # physics + netcode invariants
cargo run -p physics --example probe   --release  # performance and handling envelope
cargo run -p physics --example crash   --release  # what a hit costs, and predicts as
cargo run -p physics --example wear    --release  # how battered a field of bots gets
cargo run -p physics --example predict --release  # how well a rival can be guessed
node scripts/verify-determinism.mjs              # native vs wasm, bit for bit
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

### Predicting the other cars

Every remote car is carried forward from its newest snapshot to the tick the
client is simulating. Measured against the snapshots that arrive next, over six
bots racing:

| lead | straight line | constant turn rate | worst case |
|---|---|---|---|
| 3 ticks (50 ms) | 0.010 m | 0.005 m | 0.24 m |
| 6 ticks (100 ms) | 0.039 m | 0.021 m | 0.87 m |
| 12 ticks (200 ms) | 0.154 m | 0.082 m | 2.78 m |

Rotating the velocity as the car is carried forward buys about **half**, and it
buys most where it matters: mid-corner, at the lead a real round trip needs.
Both columns are far better than not predicting at all, which is not an error of
centimetres but of whole car lengths, and a systematic one, always behind.

The two ends of that table say different things and both are worth having. The
*average* rival is easy: real tires, real wheel inertia and load that takes 90 ms
to move cannot change direction abruptly, so a straight line through the last
snapshot stays true for a surprisingly long time. The *worst* rival is not, and
the difference is contact. A car that is hit changes direction inside a single
tick, by an impulse that had not happened when the snapshot being carried forward
was taken. No extrapolation can follow that, and none should be expected to: it
is precisely the case lag compensation exists for, and the case this demo
declines to solve.

```bash
cargo run -p physics --example predict --release
```

### Lines of code

The other cost is how much of this you have to write. `scc` over the
hand-written files, generated bindings excluded:

| | Code | |
|---|---|---|
| `module/src/lib.rs` | 451 | tables, the inbox, the registry, the outbox, the two authority checks |
| `sidecar/src/authority.rs` | 398 | one tick: sync the grid, schedule inputs, simulate and publish, or stand by |
| `sidecar/src/main.rs` | 189 | connect, subscribe, and hold 60 Hz |
| **server side** | **1038** | the module plus both sidecar files above |
| `web/src/sim.ts` | 355 | predict, rewind, replay, smooth |
| `web/src/net.ts` | 511 | subscriptions, clock estimate, remote prediction, reconnection |
| `web/src/main.ts` | 12 | the clock steering that holds the lead |
| **browser netcode** | **878** | the three web files above |

```bash
scc module/src/lib.rs sidecar/src/main.rs sidecar/src/authority.rs
scc web/src/sim.ts web/src/net.ts
```

For scale, `physics/src` is **3767** lines across fifteen files, three and a
half times the server side. The simulation is by far the biggest piece of the
project, and the part you would write wherever you ran it, which is the argument
for being able to run it wherever you like. `spacetime generate` emits another
**3234** across 42 files nobody reads.

## License

MIT.
