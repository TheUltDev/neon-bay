# The car

The vehicle model behind the demo. It is one Rust crate, `physics/`, with no
dependencies, compiled once as an `rlib` for the sidecar and once as a `cdylib`
for the browser, and it runs the same instructions on both sides: every
transcendental is a fixed polynomial in `f32`, so the two builds agree bit for
bit. The [README](README.md) covers why that matters and how it is checked. This
file covers what the crate simulates.

The simulation advances at 60 Hz. Each tick is eight substeps of 2.08 ms, and
each substep does the same things in the same order: resolve the steering into
a road-wheel angle per side, work out the load on each tire, ask each tire what
it is doing, step the engine and the driven wheels together and the front
wheels on their own, sum the forces onto the body and integrate it, then
resolve contacts between bodies. Understeer, oversteer, lift-off oversteer,
engine braking, wheelspin, lock-up and the way each of them changes with speed
are consequences of those parts rather than cases in a list, and the tests
measure them.

## Measured envelope

| | |
|---|---|
| 0-100 km/h | 3.7 s, traction-limited, the rear held at the tire's peak by traction control |
| 0-200 km/h | 11.8 s |
| top speed | 277 km/h, where power meets drag rather than where the gearing runs out |
| 50 m/s to a standstill | 81 m, 1.56 g average, ABS holding the wheels just past the peak |
| peak lateral | 1.28 g at 20 m/s, 1.44 g at 60 m/s; the difference is downforce |
| at the limit, 32 m/s | front slip 10.4°, rear 5.4°: understeer-limited |
| body movement | 2.97° of roll and 1.81° of pitch per g, 94 ms to get there |
| engine | 425 N·m, 275 kW, six speeds and a reverse |
| circuit | 1424.7 m, 15 to 20 m wide, 12 checkpoints |
| bot lap times | 48.3 to 52.2 s on a clean lap |

`cargo test -p physics --release -- --nocapture` prints every one of these.
`cargo run -p physics --example probe --release` prints the launch, the lateral
limit against speed and the cost of a tick.

## Tires (`tire.rs`)

Each contact patch is a Pacejka Magic Formula,

```text
F(s) = D sin(C atan(B s - E (B s - atan(B s))))
```

evaluated per wheel, per substep, for combined slip. `D` is the peak force,
`C` sets how far the curve falls past it, `B` how quickly it gets there and `E`
the shape of the peak. The coefficients are quoted at a reference load of
3000 N and scale from there.

| | longitudinal | lateral |
|---|---|---|
| peak friction at 3000 N | 1.62 | 1.55 |
| shape factor C | 1.50 | 1.35 |
| curvature E | -1.50 | -2.00 |
| sliding grip as a share of the peak | 71% | 85% |

Three properties of rubber follow from the curve, and all three are something
a driver feels. Grip peaks and then falls: past about eight degrees of slip
angle, or eight percent of slip ratio, asking for more gives less. Sliding
grip is below peak grip, which is the reason ABS is worth having and the reason
a locked front goes straight. And grip is sub-linear in load: the peak friction
coefficient gives up 18% of itself for every 3000 N the load rises, down to a
floor of 45%, so how the load is spread across the four patches decides the
car's balance and not only its total grip.

Slip stiffness rises with load and then flattens, as `BCD = K1 sin(2 atan(Fz /
K2))` with `K1` of 125 000 longitudinally and 78 000 laterally and knees at
5000 and 5200 N. Over the operating range of 1 to 6 kN, cornering stiffness
climbs with load while peak friction falls, which is what makes the peak slip
angle grow with load: 6.7° at 1500 N, 9° at 4500 N. The longitudinal peak sits
at 7.7% slip on a typical rear load of 2600 N.

Combined slip is handled by normalizing each slip by its own peak, taking the
resulting vector's length as the combined slip, evaluating both curves there
and splitting the force along the vector's direction. Pure slip in either axis
reproduces that axis's curve exactly, and braking and cornering compete for one
budget without either being clamped by hand.

The lateral force acts behind the wheel centre, on a pneumatic trail of 38 mm
that collapses with slip plus a mechanical trail of 22 mm from caster that does
not. The aligning moment that produces peaks well before the grip does and is
nearly gone by the limit.

A carcass builds cornering force over a distance rolled, not instantly. Each
tire's developed lateral force lags the formula's answer by a relaxation length
of 0.45 m, which is 15 ms at 30 m/s. That force is state: it is in `CarState`
and on the wire.

## Wheels (`wheel.rs`)

Each wheel carries its own angular velocity, with an inertia of 1.2 kg·m² and
a rolling radius of 0.32 m. Slip ratio is `(wR - u) / max(|u|, 2)` and slip
angle is `atan(-v / max(|u|, 2))`, with `u` and `v` the contact patch's
velocity along and across the wheel in m/s. The 2 m/s floor keeps both finite
at a standstill, and it is the same floor the assists measure against.
Wheelspin is the rear wheels outrunning the road, lock-up is a wheel at zero
while the car keeps moving, and the handbrake needs no special case: enough
torque to stop the rear wheels turning is enough to break the rear away,
because a saturated patch has nothing left for cornering.

Rolling resistance is a torque at the patch of 0.014 times the vertical load,
so it scales with downforce.

A 1.2 kg·m² wheel against a tire that pushes back with tens of kilonewtons per
unit of slip is a mode with a sub-millisecond time constant. Every wheel step
is therefore implicit in the tire's slope, the resisting torque per rad/s the
wheel gains on the road, taken from the linear region. The road's own
acceleration over the step, which is the body's longitudinal acceleration from
the previous substep, is folded into that term, because under steady
acceleration the road speeds up with the wheel and the net slip does not
change. An undriven wheel under a car accelerating at 7 m/s² drags with the
82 N its own inertia requires, at any substep rate.

Brakes are 2600 N·m per front wheel and 1500 per rear; the handbrake is a
3200 N·m cable to the rear calipers alone. A brake can stop a wheel within a
substep and never reverse it.

## Drivetrain (`drivetrain.rs`)

The engine has a torque curve sampled every 800 rpm, peaking at 425 N·m at
4800 rpm with about 275 kW just under the 7400 rpm fuel cut, an inertia of
0.22 kg·m², an idle governor at 900 rpm and a closed-throttle drag that rises
with speed, which is where engine braking comes from. It is never allowed
below 400 rpm.

The clutch is a stiff damper of 90 N·m per rad/s of slip, capped at 700 N·m.
Driving, that cap tapers to nothing below 950 rpm and is all there above 3400,
which keeps the engine alive under a load and the car from creeping at idle,
and makes a standing start flare the revs and then bite. On the overrun, with
the road turning the engine, the full cap applies: an engine being dragged up
cannot stall, and that is engine braking.

The gearbox is automatic, because the wire carries a throttle axis and not a
shifter. Six ratios of 3.55, 2.60, 1.91, 1.40, 1.03 and 0.76 through a 4.10
final drive, a 3.60 reverse, and 92% driveline efficiency. It shifts up at
7250 rpm under throttle and down at 3300, reading those points off the road
speed in the current gear rather than off the crankshaft, so neither wheelspin
nor a locked rear moves the box. A shift takes 120 ms with the torque cut and
the clutch out, after which the clutch comes back in at 8 per second. Reverse
is selected only below 0.6 m/s.

The limited-slip differential has 90 N·m of preload, adds lock at 32% of the
torque through it on power and 14% on the overrun, and resists a speed
difference across the axle at 60 N·m per rad/s up to that cap.

The engine and both driven wheels are stepped together. The clutch is a stiff
coupling between the engine and the axle and the differential another between
the two wheels, so each substep solves one 3x3 linear system for the three new
speeds: backward Euler on every coupling, with each coupling linear in the
speeds until a clutch or a diff reaches what it can hold, at which point it is
a constant torque and drops out of the Jacobian. The brakes are inside the same
solve, so the rear brakes slow the flywheel through the clutch as well as the
wheel, and a wheel that has stopped, with a brake that can hold whatever is
trying to turn it, is bolted for the substep. The answer does not depend on the
substep rate: the launch is 3.68 s to 100 km/h at 480 Hz and 3.68 s at
1920 Hz, and `the_launch_does_not_depend_on_the_substep` keeps it that way.

## Load transfer (`suspension.rs`)

The tire takes one number from the chassis, the vertical load, and grip is
sub-linear in it, so how the load moves is most of the car's balance. Roll and
pitch are second-order systems with real inertia, stiffness and damping:
320 kg·m² against 94 000 N·m/rad in roll at about 0.55 of critical, and
1400 kg·m² against 139 000 N·m/rad in pitch. Load moves across the car over
about a tenth of a second rather than instantly, which is what makes lifting
off mid-corner something a driver can meter.

Lateral transfer through the springs is split front to rear by the roll
stiffness distribution, 52 000 N·m/rad at the front against 42 000 at the rear.
The front axle takes more of it, its inside wheel goes lighter, and the pair
loses more to load sensitivity than the rear does, so the car understeers at
the limit. That is deliberate: an oversteer-limited car cannot be driven with a
keyboard. The geometric share through the roll centres, 55 mm at the front and
85 mm at the rear, and the unsprung mass move instantly. Longitudinally a
quarter of the transfer goes through the links at once, as anti-dive and
anti-squat, and the rest waits for the body to pitch.

The sprung mass is 1062 kg of the car's 1150 kg, at 0.5365 m. The whole car's
centre of gravity is 0.52 m up, 1.25 m behind the front axle and 1.45 m ahead
of the rear, on tracks of 1.60 and 1.58 m. At 1 g the body rolls 2.97° and
pitches 1.81°, reaching 63% of that in 94 ms, and the inside front lifts at
1.9 g.

## Aerodynamics (`aero.rs`)

Drag is 0.34 v² plus 0.22 N for every newton of downforce, and downforce is
0.75 v² with 42% of it on the front axle. At 60 m/s the wings press with a
quarter of the car's weight; downforce would equal weight at 123 m/s, which the
car never reaches. The rear-biased balance is deliberate: a car that gains grip
at the front faster than at the rear as it speeds up turns into high-speed
oversteer.

## Steering

The rack reaches 0.58 rad at the road wheel and moves at 5 rad/s, with 65%
Ackermann, so the inner wheel takes some extra angle in a corner but not all
of it.

## Assists modelled because the car has them

Both sit just past the tire's longitudinal peak, and `assists_sit_at_the_peak`
reads that peak off the tire model and holds them to it.

**ABS** releases the pedal's brake torque past 0.10 slip ratio, down to a floor
of 12% by 0.21, as a proportional release rather than the on-off cycling of a
hydraulic unit. The handbrake is a cable and gets no help, which is what makes
it useful for putting the car sideways.

**Traction control** starts closing the throttle past 0.09 slip ratio and has
it closed by 0.19, watching the worse of the two rear wheels so that a single
spinning inside wheel on corner exit is enough. Wheelspin is measured in the
direction the gearbox is driving, against the road only where the road is
moving that way: a car rolling forward on stopped wheels while the driver holds
reverse is waiting to be braked, not spinning its wheels. What traction control
decides is a scale on the pedal, kept apart from the driver's intent, and the
gearbox reads the intent for which way to go.

## Input aids

Three, all there because a keyboard is not a car, and all in the physics rather
than in the browser so that the authority applies the same ones and the bots
get them too.

**The steering aid** stands in for a wheel with 900 degrees of travel and two
hands on it. The lock a key commands tapers with speed, `0.58 / (1 + 0.105 v)`
radians: 33° at rest, 12.9° at 15 m/s, 8° at 30 m/s, because a tire makes its
peak lateral force at six to nine degrees of slip and a keyboard handed
seventeen degrees at 30 m/s would spend three quarters of its range past the
peak. Once the rear has let go, its slip angle past eight degrees and fully by
twelve, opposite lock is measured from where the front tires are travelling
rather than from straight ahead, in proportion to the input: full opposite lock
puts the fronts at the taper's slip angle relative to the road, and a key held
into the slide unwinds towards straight ahead but never past it. The slide is
also damped, with a wheel angle of 0.3 rad per rad/s against `omega - ay / u`,
the rate at which the body slip angle is growing, which is zero in any steady
corner at any speed and needs no model of the car. The damping is capped at
the lock a key has; it acts only once the car is sideways, meaning body slip
past six degrees, or the rear is past its peak, or the slip rate is past
0.35 rad/s, where turning in to a fast corner never takes it; and it fades out
below 6 m/s. The bot driver has yaw-rate feedback of its own and subtracts the
damping.

**The reverse assist** brakes for a driver who asks for a direction the car is
not going in, at half of full pressure per m/s of speed the wrong way beyond
the 0.6 m/s at which the gearbox will change direction. A negative pedal is a
request for a direction and not a negative torque, and a gearbox will not
select reverse at speed.

**Pedal travel** gives both pedals a stroke. The throttle takes 250 ms to floor
and 83 ms to release, the brake 100 ms and 67 ms, with pressing defined as
moving away from rest in either direction. An analog trigger is left alone
unless it moves faster than a foot. The pedal positions are simulation state,
on the wire.

## Contact (`collide.rs`)

Bodies are oriented boxes of 4.2 by 1.9 m. Two boxes are separated with the
separating-axis test, and the incident face is clipped against the reference
face for a manifold of up to two points, which is what a flat contact needs:
one point can only push, two can push and resist a twist, and that is the
difference between scraping along a barrier and pirouetting off it. The
constraints are solved as accumulated impulses over six passes in a fixed
order, normal impulses clamped non-negative and friction clamped to the Coulomb
cone against the normal impulse that contact has built up. Penetration beyond
5 mm is pushed out at a quarter per pass, and approaches below 0.6 m/s are rests
rather than bounces. Friction is 0.65 car to car and 0.40 against a barrier.

Contacts are solved every substep, on the same 480 Hz clock as the tires. The
deepest overlap a hit reaches is 5 mm at 30 m/s of closing speed and 13 mm at
45. The one thing sampled once a tick is where the barrier is: that is a search
through the centreline, the most expensive thing a tick does, and a barrier
does not move, so the search runs once and the contact it feeds runs eight
times.

## Crush and damage (`damage.rs`)

Steel that folds does not give the energy back, so a car-to-car impact is
mostly plastic, and the harder it is the more plastic it gets. The model is
Campbell's: residual crush is linear in impact speed, `v = B0 + B1 C`, with
`B0` of 2 m/s the speed a car shrugs off entirely and `B1` of 28 m/s per metre
of crush. Integrating the force that implies gives the energy a depth of crush
has absorbed, and inverting it gives the depth a given energy produces. Damage
accumulates through the energy, so the structure stiffens as it folds and two
30 kJ hits leave the dent of one 60 kJ hit. Crush is capped at 0.45 m per face.

Restitution follows from the same statement rather than being a second knob.
If everything up to `B0` is elastic and everything past it goes into bending
metal, the returned fraction of the energy is `(B0/v)²` and `e = B0/v`, capped
at 0.45 for impacts too gentle to bend anything. It lands within a few
hundredths of the published curves: 0.20 at 10 m/s against Antonetti's 0.24,
and 0.10 at 20 m/s against 0.10.

Two cars, one rear-ending the other at 20 m/s, both coasting:

| closing | separation | of the closing speed | absorbed | chaser's nose |
|---|---|---|---|---|
| 2 m/s | 0.05 m/s | 2% | 7.5 kJ | 0.007 m |
| 6 m/s | 0.61 m/s | 10% | 16 kJ | 0.054 m |
| 12 m/s | 0.48 m/s | 4% | 43 kJ | 0.152 m |
| 20 m/s | 0.11 m/s | 1% | 110 kJ | 0.291 m |
| 30 m/s | 0.00 m/s | 0% | 255 kJ | 0.450 m, which is all of it |

A scrape is not a crash. The solver measures the energy a contact destroys
head-on, normal impulse doing work, separately from the energy it destroys
sliding. The first folds panels at the full rate; the second wears them at 15%
of it, under a ceiling of 10 cm, because sliding contact takes the flank off
and not the width. Half of a car-to-car hit's energy goes into the car's own
structure, and 70% of a barrier hit's.

Damage is four numbers on the wire, the residual crush in metres on each face
of the body, and every one of them changes what the car can do: a folded nose
makes a fraction of the front downforce and more drag; the rack loses lock and
bent geometry pulls towards the side that took the hit; the engine gets less
air; a bent corner rubs its own bodywork and that tire has less grip. It is on
the wire because rollback needs it, and because it is also the shape the
renderer draws. Completing a lap repairs the car: the start line is where a pit
lane would be, and a lap is long enough that a shunt has to be driven around
and short enough that nobody is stuck with one.

`cargo run -p physics --example crash --release` prints the tables above and
what a browser predicts of the same hits; `wear` prints how battered a field of
bots gets over a race.

## The bot driver (`bot.rs`)

Pure pursuit on the centreline, aiming 9 m plus 0.72 s of travel up the road,
with an apex-seeking offset, a personal line bias and a slow wobble so that no
two bots take the same line. The desired yaw rate is clipped to what the tire
model says the car can deliver at this speed, planning for 60% of that limit
scaled by a skill of 0.86 to 1.05, and converted back to a road-wheel angle
through the same taper a keyboard gets, with 0.16 rad of yaw-rate feedback per
rad/s of error and the steering aid's damping subtracted. Corner-entry speed
comes from the track's curvature over a horizon that grows with speed, and the
throttle is limited by speed, by steering angle and by rear slip. A bot that is
stopped against a barrier, or pointing the wrong way at low speed, rocks the
car out in 75-tick phases rather than pushing further in.

Bots run only in the sidecar. The browser never runs their brains; it carries
their cars forward on the controls the sidecar published.

## The track (`track.rs`)

A closed polar curve of three harmonics about a 200 m radius, star-shaped by
construction so that it can never self-intersect and the walls can never pinch,
resampled at uniform arc length into 768 samples about 1.86 m apart. The
resampling is what makes the rest cheap: the index is `s / ds`, so mapping a
car's distance along the lap to a wall segment is a multiply rather than a
search. The lap is 1424.7 m, 15 to 20 m wide, with 12 checkpoints that have to
be crossed in order.

## What the tests pin

`cargo test -p physics --release` runs 85 tests. Beyond the envelope above,
the ones worth knowing about:

- **Tire shape.** The baked peak positions match the curve, pure slip in either
  axis reproduces that axis exactly, braking eats into cornering grip, and the
  aligning moment peaks before the grip does.
- **The skidpad sweeps the lock** rather than reading one fixed input, because
  with a real tire curve more lock past the peak buys less force, and it pins
  the understeer balance.
- **A keyboard throttle mid-corner does not spin the car**, at 10, 15 and
  22 m/s: the pedal goes from holding speed to flat and stays there.
- **A keyboard driver can catch a slide**: a handbrake flick mid-corner at 20
  and 30 m/s, opposite lock a quarter of a second late, and the car calm again
  within the run.
- **The launch does not depend on the substep.**
- **The assists sit at the peak**, read off the tire model.
- **Rollback replay is bit-exact**, and the collision a client predicts is the
  one the authority resolves.

## What it costs

| cars | physics | bot AI | total | of a 60 Hz budget |
|---|---|---|---|---|
| 1 | 4.2 µs | 0.4 µs | 4.6 µs | 0.03% |
| 7 | 30.5 µs | 2.5 µs | 33.1 µs | 0.20% |
| 16 | 72.4 µs | 6.1 µs | 78.5 µs | 0.47% |
| 24 | 111.5 µs | 9.0 µs | 120.6 µs | 0.72% |

Four Magic Formula evaluations per substep per car, eight substeps a tick, and
the contacts on the same clock. `cargo run -p physics --example probe --release`
reproduces the table.
