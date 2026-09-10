# The netcode

How the browser half of the demo keeps a car under the driver's hands while the
authority is a round trip away. The [README](README.md) covers the sidecar
pattern this sits on and [PHYSICS.md](PHYSICS.md) the car being simulated; this
file is the client's side of the wire, and it is specific to this
implementation.

## How it works

Three pieces, in the order they run.

**1. Predict.** Every tick, the client samples the controls and steps the whole
grid in wasm immediately: its own car on what the driver is doing, every rival on
the controls the authority published with that car's last snapshot, held. It
stores `(tick, input, resulting state)` for its own car in a 256-entry ring
buffer. Steering has zero input lag regardless of ping.

**2. Reconcile.** Snapshots arrive at 20 Hz carrying the authority's state for a
tick `T` in the recent past, for every car at once. Each rival is overwritten
with its own, since there is nothing in a car this client does not own to
reconcile, only news to adopt. Then the world is rewound to `T` and replayed
forward to now, the local car on its stored inputs and the rivals on their held
ones. Typically 5 to 20 ticks, and the grid's worth of work rather than one
car's: a full field costs about 1.8 ms of it per snapshot on a desktop.

The local car's half of that is a *replay* and the rivals' half is a
*prediction*, and the difference is worth keeping straight. The replay lands on
identical bits, which is what `physics::tests::rollback_replay_is_bit_exact`
pins: re-simulating from an older state with the same inputs reproduces the same
bits. The prediction does not and never will, because the client does not know
what the other driver did next. The HUD shows both, on separate lines, for
exactly that reason.

**3. Smooth.** A rewind moves the cars, which would read as a stutter. So the
client keeps the gap between where each car appeared to be and where it now is
as a visual offset, and decays it to zero over ~220 ms. The simulation is
corrected at once; the picture catches up. Corrections over 9 m on the local car,
meaning a respawn or a long stall, snap instead and raise the "resyncs" counter;
a rival that jumps more than 4 m has respawned or changed hands and is likewise
shown moving rather than slid across the track.

**The clock.** For prediction to be exact, an input stamped for tick `T` has to
be applied by the authority *at* tick `T`. The client estimates where the
sidecar's clock is, adds a round trip plus two ticks of margin, and steers its
own tick rate by up to ±6 % to hold that lead. You cannot feel it. The sidecar
schedules inputs rather than applying them on arrival: an early one waits for its
tick, and a late one is applied at once, since there is no rewinding the
authority. The client absorbs the difference on its next rollback.

**Other cars.** The client simulates them too. The local car is
deliberately in the future, far enough ahead that its input reaches the authority
before the tick it belongs to, so holding rivals a few ticks in the *past*, which
is what an interpolation buffer does, puts them wrong by the sum of the two. At
racing speed that is several metres, systematically, in the direction of travel:
you would be leaning on a car the authority had somewhere else. Every car on the
grid therefore lives on one clock, and it is the tick the local car is
predicting.

Each rival is seeded from its newest snapshot and stepped forward on the
controls the authority published beside it, held until the next one arrives.
`car_state` carries those four channels for exactly this: they are the one thing
about another driver that a client cannot work out for itself.

The cheap alternative is to extrapolate the pose, carrying the car along its
velocity and rotating it as it goes so a car mid-corner follows the arc instead
of flying off the tangent. That is a good guess about a car doing nothing and a
poor one about a car doing something, which is a problem, because a car doing
something is the only kind you ever hit. An extrapolated pose knows nothing about
tires saturating, about load taking ninety milliseconds to move, or about drag,
and nothing at all about the pedals. Over eight bots at two hundred milliseconds
of lead, scored against rivals that were braking or cornering when the snapshot
was taken:

| scheme | mean | p99 | worst |
|---|---|---|---|
| constant turn rate | 0.114 m | 0.278 m | 2.52 m |
| held input | 0.009 m | 0.064 m | 0.15 m |
| zero input (control) | 0.085 m | 0.130 m | 0.18 m |

The tail is the column that matters. The mean is dominated by cars going in a
straight line, where every scheme agrees and none of this makes any difference.
The third row separates the two halves of the claim: real dynamics does most of
the work, and the real pedals halve what is left, which is what earns the input
its place on the wire.

Bots are a hard test for held input and an easy one for the extrapolation. A
pure-pursuit controller re-decides its steering sixty times a second, and a human
holding a key does not. `cargo run -p physics --example predict --release`.

The mask `World::step` takes says which cars are being *integrated*, and both
processes pass the whole grid. It is not a statement about ownership. The client
integrates cars it does not own, and should, because the authority is integrating
those same cars from the same state with the same code. A clear bit is for a car
there is nothing to integrate *from*: one no snapshot has arrived for yet, which
is solid but has no state worth advancing.

**A rival is reconciled by being overwritten**, not by being blended. The client
has no stake in a car it does not own and the snapshot is not an opinion, so when
one lands it goes straight into the slot and the held input goes with it. The
error it reveals is an apology for a guess that has just been corrected: feeding
that back into the next contact would re-introduce exactly what it was hiding. So
it goes into the *picture*, as a fading offset, which is the same trick the local
car plays after a rollback, and nowhere near the physics.

**A rival is not a rival that weighs nothing** either, and that is easy to
confuse with not being allowed to move it. Saying the first to a solver by giving
the car infinite mass says the second as well, and an infinite mass returns the
whole impulse: the local car rebounds off something it should have shoved aside.
So the solve uses both cars' real mass. With both of them in the mask the
positional repair is split between them exactly as the authority splits it, which
is the point of being in the same arrangement as the authority. A car outside the
mask keeps its mass and takes none of that push, and what the hit did to it is
remembered for one tick, so a single contact cannot fire on all eight substeps
against something that never reacts.

Handed the same tick's state the authority has, a client that steps its rival
runs the authority's own arithmetic on the authority's own numbers and lands on
the same bits. One that parks it as scenery does not:

| closing | authority | parked client | stepped client |
|---|---|---|---|
|  6 m/s | 22.09 m/s | 2.03 m/s backwards, 0.70 m out | exact |
| 12 m/s | 25.47 m/s | 3.02 m/s backwards, 0.91 m out | exact |
| 20 m/s | 29.88 m/s | 4.78 m/s backwards, 1.37 m out | exact |
| 30 m/s | 35.06 m/s | 7.34 m/s backwards, 1.91 m out | exact |

`cargo run -p physics --example crash --release` reproduces it. That is the
contact model with the connection taken out of it. What a *stale* snapshot costs
on top is the netcode's question, and
`physics::tests::a_client_predicts_the_hit_the_authority_resolves` is where it
gets asked: a rival braking, cornering, or braking at the last moment, nine ticks
of lead, and the worst the client is ever out by about its own car.

| the rival is | parked ghost | stepped rival |
|---|---|---|
| travelling | 1.52 m, 9.75 m/s | 0.00 m |
| braking hard | 1.75 m, 8.55 m/s | 0.00 m |
| leaning on the wheel | 1.48 m, 7.68 m/s | 0.00 m |
| braking mid-corner | 1.76 m, 10.76 m/s | 0.00 m |
| braking late | 1.50 m, 9.65 m/s | 0.02 m, 0.21 m/s |

The zeros are not rounded. When a driver holds an input, which is what a driver
mostly does, the client reproduces the authority to the bit and there is nothing
left to be out by. The last row is where the scheme costs something and always
will: a car that changes its mind inside the round trip cannot be followed, only
corrected.

**What it costs** is the grid instead of one car, on the forward step and on
every replayed tick of a rollback. Twelve ticks of lead, twenty rollbacks a
second, `node scripts/bench-client.mjs`:

| cars | forward step | rollback | per second of racing | of a frame |
|---|---|---|---|---|
| 1 | 5.8 us | 62 us | 1.6 ms | 0.2 % |
| 8 | 51.0 us | 507 us | 13.2 ms | 1.3 % |
| 24 | 160 us | 1832 us | 46.2 ms | 4.6 % |

That is a desktop, and a desktop is not the machine to worry about: a mid-range
phone runs wasm three to eight times slower against the same 16.7 ms frame. A
full grid is the number to watch, and a full grid is rare.

## Predicting the other cars

Every remote car is carried forward from its newest snapshot to the tick the
client is simulating. There are four ways to do that, and the example scores all
four against where the car really went, over eight bots racing. Two of them are
extrapolations of the pose. The other two run the physics: one on the controls
the authority published with the snapshot, one with the pedals released. That
last is the control, and it says how much of the accuracy is the pedals and how
much is merely having real dynamics.

Position error at the 99th percentile, which is the number this is judged on:

| lead | straight line | constant turn rate | held input | zero input |
|---|---|---|---|---|
| 3 ticks (50 ms) | 0.017 m | 0.013 m | **0.002 m** | 0.004 m |
| 6 ticks (100 ms) | 0.067 m | 0.050 m | **0.011 m** | 0.025 m |
| 12 ticks (200 ms) | 0.265 m | 0.195 m | **0.064 m** | 0.125 m |
| 18 ticks (300 ms) | 0.592 m | 0.433 m | **0.176 m** | 0.307 m |

Rotating the velocity as the car is carried forward buys about a quarter over
a straight line, and it buys it mid-corner, which is where it matters. Running
the physics buys another factor of three on top, and the pedals are half of
that: dynamics without them still has the car braking when it is not, and
coasting when it is. All four are far better than not predicting at all, which is
not an error of centimetres but of whole car lengths, and a systematic one,
always behind.

The *average* rival is easy for every scheme: real tires, real wheel inertia and
load that takes 90 ms to move cannot change direction abruptly, so even a
straight line through the last snapshot stays true for a surprisingly long time.
The *worst* rival is where they separate, and the worst rival is almost always
one that has just been hit by somebody else. Scoring only those, meaning a car
that takes a hit from another car *during* the lead, so the impulse is not in the
snapshot being carried forward, at 200 ms, over the eighteen hundred of them in
this race:

| scheme | mean | p99 | worst |
|---|---|---|---|
| straight line | 0.173 m | 0.513 m | 1.74 m |
| constant turn rate | 0.115 m | 0.592 m | 1.90 m |
| held input | 0.011 m | 0.068 m | 0.12 m |
| zero input | 0.074 m | 0.136 m | 0.18 m |

An extrapolation cannot follow a collision that has not happened yet, and its
whole tail is made of them. A grid stepped together can, and not by guessing the
impulse better: the car doing the hitting is in the same world running the same
physics on the same tick, so the client is not guessing at all. It arrives at the
collision from both sides, the way the authority does.

What is left in the tail after that is the thing no amount of simulation
reaches: a driver who changes their mind inside the round trip. The client is
holding an input that was true when it was published and is not true now, and
nothing carried forward from before the change can know about it.

Bots make this table pessimistic for held input and flattering to the two
extrapolations: a pure-pursuit controller re-decides its steering sixty times a
second, and a driver holding a key does not. Whatever margin the third column
wins here, it wins by more against a person.

```bash
cargo run -p physics --example predict --release
```

## The core in the browser

The whole physics core is a **58 KB** `.wasm` with zero imports: no
wasm-bindgen, no wasm-pack, no build plugin. `CarState` is `#[repr(C)]` and all
`f32`, forty-six of them, so the browser maps it with one `Float32Array` over
wasm memory and reads and writes the simulation in place. Nothing is serialized
on the hot path.
