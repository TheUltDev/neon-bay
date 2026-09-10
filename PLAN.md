# sim-rivals: predict rivals with the physics, not with a straight line

Working notes for the next session. Branched from `master` at `cf96a8d`.

## Where this came from

`master` now carries a rival's yaw rate into the client's world, which halved
how far a predicted shunt lands from the authority's answer. That fixed a field
the client had and was dropping. What is left is harder: the client does not
have the rival's *future*, and the way it guesses at one is a straight line.

`Net.predict` carries a rival forward at constant velocity and constant turn
rate, for up to `MAX_LEAD = 0.3 s`. It knows nothing about tires saturating,
load transfer, or drag. So it is at its worst exactly where it matters --
a rival braking hard, or one that has just been hit -- which is what the README
means by "the prediction is a guess: 0.02 m out at a tenth of a second of lead,
and more when a rival brakes hard or is hit."

The client already owns the whole simulation. The one thing it is missing is
what the other driver is doing with the pedals.

## Step 0: measure before building anything

**Do not skip this.** The full change costs up to 24x the client's per-tick
physics, and that may well be the thing that kills it. Find out whether the
accuracy is there first.

1. Publish the input (step 1 below) and nothing else.
2. In `net.ts`, when a snapshot for tick T lands, compare three predictions of
   it made from the *previous* snapshot:
   - what `predict` actually produced (constant turn rate),
   - what a physics step with the rival's last known input produces,
   - what a physics step with a zero input produces (the control: is any
     benefit just from having real dynamics, or from having the real pedals?).
3. Log the position and heading error of each against the snapshot that
   arrived. Drive a few laps in traffic, and look at the tail rather than the
   mean -- the mean is dominated by cars going in a straight line, where all
   three agree and none of this matters.

If held-input physics is not clearly better than constant turn rate at the
tail, stop. The rest is not worth the cost.

## Step 1: put the last-applied input on the wire

Cheap, and the prerequisite for measuring.

- `sidecar/src/authority.rs` already holds it: `Slot.current` is documented as
  "input currently being applied, held until a newer one comes due", and it is
  in scope where the `CarState` rows are built for `push_states`. Four more
  fields sourced straight from `s.current`.
- `module/src/lib.rs`: add `in_throttle`, `in_steer`, `in_brake`,
  `in_handbrake` to `car_state`.
- `CarState` in `physics/src/car.rs` does **not** need them -- they are inputs,
  not state, and the physics takes them as a separate `CarInput`. Keep them out
  of the `#[repr(C)]` record so `CAR_FLOATS` does not move. Carry them beside
  the state in `net.ts` instead.
- Regenerate both binding sets.

Ships as a destructive deploy, for the usual reason: `car_state` reshapes.

## Step 2: step rivals instead of parking them

Today `Sim.step` runs `phys_step(1 << localSlot)` and `placeRemotes` parks
everyone else as immovable colliders at their predicted pose.

- Write each rival's newest full snapshot into its slot with `phys_set`.
- Load its last known input into `inputs[slot * 4 ..]`.
- Step the whole active mask rather than one bit.

`World::step` already takes an arbitrary mask, so no physics change is needed
for this part. The sidecar has always run it this way.

## Step 3: hold the input across the gap

A rival's input is known up to its newest snapshot and unknown after it. Hold
the last one for the remaining lead, still bounded by `MAX_LEAD`.

The bet is that driver input is strongly autocorrelated over 100-300 ms, so a
held real input beats a constant turn rate badly. Step 0 is what settles
whether the bet pays.

## Step 4: reconcile rivals by overwriting

When a newer snapshot for a rival lands, write it into the slot outright. No
blending and no smoothing -- it is authoritative, and the client has no stake
in a car it does not own. Keep `sampleRemote` (with the residual, for drawing)
and `predictRemote` (without it, for the physics) distinct exactly as they are
now; the residual is an apology for a guess that has already been corrected,
and feeding it back into a contact re-introduces the error it was hiding.

## What will bite

- **Cost.** One car to N, per tick *and* per replayed tick. The sidecar's own
  table says 3.7 us for one car and 99.9 us for 24; a rollback is currently
  ~140 us for 20 ticks and would scale with the grid. Measure on a phone, not
  on a desktop.
- **`separate()` moves cars.** In `world.rs` the positional push is deliberately
  restricted to cars the process owns -- "a numerical repair and not a force".
  Putting rivals in the mask makes the client start moving them, which is a new
  divergence source rather than a fix. That guard needs re-reading before the
  mask changes, and it may need to stay keyed on ownership rather than on the
  mask.
- **Bots have no `Input` rows.** Their input is generated on the sidecar by
  `BotBrain`. `Slot.current` should still hold it, but check: if bots publish a
  stale or empty input, every bot on the grid gets predicted with no pedals.
- **Determinism is not at risk, but the claim is.** Rivals stepped from a held
  input will not match the authority, and that is expected -- it is a
  prediction, not a replay. Make sure the *local* car's bit-exactness is still
  what `verify-determinism.mjs` and the HUD's "PREDICTION EXACT" refer to, and
  that nobody reads the rival error as a determinism failure.

## Done looks like

- `a_client_predicts_the_hit_the_authority_resolves` extended with a case where
  the rival is braking or turning *during* the approach, not just travelling.
  That is the case constant-turn-rate cannot do and this is supposed to fix.
- The tail of the rival prediction error measurably smaller than what step 0
  recorded for the straight-line guess.
- Client tick cost on a full grid still inside frame budget on a mid-range
  phone.
