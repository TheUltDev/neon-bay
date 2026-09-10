# What could be better

This is a proof of concept, and these are the places where it stops. Each one
is written as the work that would take it further, with a concrete way in. The
[README](README.md) describes what is there today; nothing below is started.

## A hot standby

Today one container runs one sidecar, so a deploy or a crash is a gap of
however long the process takes to come up. The arbitration for two sidecars is
already in the module: `claim_authority` grants the seat when it is empty,
already yours, or its holder has been quiet for two seconds; `push_states`
fences on the connection that won the claim; `client_disconnected` frees the
seat at once. A standby costs 20 µs a tick and a measured handover is 103 to
123 ms against the 50 ms between snapshots.

To make it real, run the standby as a second container against the same
database:

- Give the image a sidecar-only mode. `scripts/railway-start.sh` starts
  SpacetimeDB, publishes and then starts the authority; a `SIDECAR_ONLY=1` path
  would skip the first two and point `--uri` at the primary's public address,
  with the same publisher token supplied through the environment.
- Deploy it as a second Railway service in the same project, restart-always,
  health-checked on its own status line rather than on `/v1/ping`, which it
  does not serve.
- Shorten the lease. A clean exit or a crash closes the socket and
  `client_disconnected` frees the seat at once, so the two-second lease only
  covers a partitioned holder. Calling `release_authority` on `SIGTERM` makes
  the clean case explicit; the reducer exists and the sidecar does not call it
  today. Measure the false-handover rate against the snapshot cadence before
  going below one second.

## Exact prediction of the bots

The one tail no amount of simulation reaches is a driver who changes their
mind inside the round trip: the client holds an input that was true when it was
published and is not true now. For a human that is the honest limit. For a bot
it is not, because a bot's mind is deterministic. `BotBrain::drive` is a pure
function of the world, the track and the tick, seeded by the bot's slot, and it
is in the same crate the browser already loads.

- Export it across the wasm bridge as `phys_drive_bot(slot, tick)`, writing the
  result into the input block the way `set_input` does for the local car.
- The `car` row, which the client already subscribes to, carries `is_bot`, and
  the sidecar derives each bot's personality seed from the row's `car_id`, so
  the client can do the same.
- In `sim.ts`, run the brain for each bot on the forward step and on every
  replayed tick instead of holding its last published input. The prediction
  becomes a replay, and it lands on the authority's bits whenever the cars the
  bot is reacting to are where the authority has them, which for a grid of
  bots is all of the time.

That moves every bot out of the "changes their mind" column and leaves only
humans in it. The bot AI costs 0.4 µs per car per tick natively; in wasm, a
full grid of bots is well under a millisecond per second of racing on top of
the physics already being run for them.

## Contact resolved from the toucher's point of view

Every car in the browser lives on one clock and is simulated onto it, so a hit
the client predicts is the hit the authority resolves whenever both drivers
held their inputs. What remains is a rival whose input changed inside the
round trip, and a bounded form of lag compensation would cover most of it:

- Keep a ring of the last few hundred milliseconds of snapshots in the sidecar,
  which produces them anyway and today forgets each one as it is published.
- When an input for tick `T` arrives late from a client whose lead was `L`,
  and the authority finds that client's car in contact, resolve that contact
  against the rivals as they were at `T` rather than at now, and apply the
  resulting impulse to the rival at now. Bound it to one snapshot of history so
  the rival is never moved by a hit from further in its past than a driver can
  perceive.
- Prefer the toucher when the two accounts disagree, which is what shooters do
  for hit registration, and accept that the touched car sees a small correction
  it did not cause. The visual offset in `sim.ts` already hides corrections of
  that size.

Mutual impulses cannot both be right, so this is a policy rather than a
solution, and it should be measured with the `predict` example's collision
tail before and after: the p99 of a rival struck during the lead is 0.068 m
today with held inputs.

## Replays and ghost laps from the wire

`car_state` carries the controls the authority applied beside every pose, and
the physics is deterministic, so a recording of snapshots is a recording of the
race. That is enough for:

- **Deterministic replays.** Record the 20 Hz stream to IndexedDB during a
  session. Replay by seeding the wasm world from any snapshot and stepping it
  on the recorded inputs, which reproduces the authority's bits between
  snapshots rather than interpolating between them. Scrubbing is a re-seed.
- **Ghost laps.** Store the best lap's snapshot stream against the lap record
  and draw it as a rival that nobody can hit: the renderer already draws the
  server ghost as a dashed outline.
- **A telemetry overlay.** Everyone's pedals, slip angles and gear are already
  on the wire; a spectator view is a subscription and some drawing.

## Bounding floods before the transaction

`set_input` drops anything past 45 writes a second per identity before it
writes a row, so a flood costs nobody else a fan-out. It still opens a
transaction per call, and bounding that is the host's job. What the module and
the sidecar can do on top:

- Count violations on the `player` row inside the same reducer, since the
  bucket is already being spent there.
- Have the sidecar, which reads every table on every tick, despawn the car of
  an identity whose count keeps climbing, so its inputs drive nothing, and
  clear the count on a clean reconnect.
- Cut the honest client's own rate. The browser sends 30 inputs a second;
  sending only on change, with a 10 Hz keepalive so packet loss still shows up
  as a stale input rather than a lost one, drops the write rate of a car
  holding a key to a third of what it is now.

## A tick boundary that does not spin

The loop sleeps towards each tick and spins the last measured margin, 1.5 % of
a core against 7.2 % for a fixed margin. Two ways to zero:

- **A better timer.** On Linux, `clock_nanosleep` with `TIMER_ABSTIME` on
  `CLOCK_MONOTONIC` lands within tens of microseconds on a current kernel; on
  Windows, a waitable timer created with
  `CREATE_WAITABLE_TIMER_HIGH_RESOLUTION` does the same from Windows 10 1803.
  Keep the measured margin as the fallback where neither is available.
- **Accept the lateness.** The tick a snapshot carries is a number, not a
  timestamp, so a tick that lands 100 µs late costs nothing downstream. Sleep
  the whole way, drop the spin, and measure p99 lateness against the 16 667 µs
  budget. If the scheduler keeps it under a millisecond the spin was never
  needed.

## More tracks, bigger grids, more races

The grid is 24 slots on one track in one database. Each of the three is a
constant with an obvious seam:

- **Tracks.** `track.rs` builds the circuit from three harmonics on a 200 m
  radius, so a track is five numbers. Put a seed on the `config` row, have the
  sidecar and the client both build the track from it at startup, and fold it
  into the physics fingerprint so a mismatch is caught the same way a physics
  mismatch is.
- **Grid size.** `MAX_CARS` is shared by the module and the physics crate, and
  the sidecar refuses to start if the two disagree. Sidecar cost is linear at
  5 µs a car, so 48 cars is
  about 240 µs a tick, still under two percent of the budget. The browser is
  the real limit: a 24-car rollback is 1.8 ms and it happens twenty times a
  second, so a bigger grid wants a cheaper rollback, either fewer replayed
  ticks through a shorter lead or rivals replayed at a coarser substep.
- **Races.** A database is a race. A lobby database that only holds who is
  racing where, and one database per race with its own sidecar container, is
  sharding with nothing new in the module. The authority check is per
  database already.
