//! SpacetimeDB module for the physics-sidecar demo.
//!
//! Note what is *not* here: there is no simulation, no integrator, no collision
//! code, not even a track. The module is three things and nothing more:
//!
//! * an **inbox** -- clients write their controller state into [`input`], which
//!   only the authority can read back;
//! * a **registry** -- who is connected, which car is theirs, who is allowed to
//!   be the authority and which process holds it;
//! * an **outbox** -- the sidecar writes authoritative poses into [`car_state`]
//!   and SpacetimeDB fans them out to every subscriber.
//!
//! The security model is unchanged from running physics inside the module: a
//! client can only ever say "I am holding the throttle down". It cannot say
//! where its car is. The difference is that the code deciding what that means
//! runs in a process that can be scaled, profiled and hot-restarted separately
//! from the database.

use spacetimedb::{reducer, table, ConnectionId, Identity, ReducerContext, Table, Timestamp};

/// Physics world slots. Must match `physics::MAX_CARS`; the sidecar refuses to
/// start if [`config`] disagrees with the crate it was built against.
const MAX_CARS: u32 = 24;

/// How long an authority may go quiet before its seat is declared free.
/// Snapshots land twenty times a second, so this is forty missed ones: long
/// enough that a stutter is not a handover, short enough that a wedged process
/// is not a frozen race.
const LEASE_MICROS: i64 = 2_000_000;

/// Controller writes a client may sustain, and the burst it may bank.
/// The browser sends thirty a second; the slack absorbs a hiccup that arrives
/// as a clump without letting anyone hold this reducer open at a kilohertz.
const INPUT_RATE_HZ: f32 = 45.0;
const INPUT_BURST: f32 = 15.0;

// ---------------------------------------------------------------- registry --

#[table(accessor = config, public)]
pub struct Config {
    #[primary_key]
    pub id: u32,
    /// Identity currently holding the authority. Empty when nobody does.
    pub sidecar: Option<Identity>,
    pub sidecar_online: bool,
    pub tick_hz: u32,
    pub snapshot_hz: u32,
    pub max_cars: u32,
    /// Most recent simulation tick the authority has published.
    pub server_tick: u64,
    pub updated_at: Timestamp,
    /// What the current authority's physics build computes, as one number.
    /// Clients compare it against their own `physics.wasm` and stop trusting
    /// their prediction if it differs -- see `physics::fingerprint`.
    ///
    /// Last in the struct and carrying a default so that adding it is an
    /// automatic migration: a live database should not have to be wiped, and
    /// its leaderboard thrown away, to gain a health check. Zero means no
    /// sidecar has claimed yet, which clients read as "nothing to compare
    /// against" rather than as a mismatch.
    #[default(0)]
    pub physics_fingerprint: u32,
    /// The only identity allowed to be the authority: whoever published the
    /// module. Pinned by [`init`], because a database's owner is the one thing
    /// here that is not first-come. `None` on a database created before this
    /// column existed, where the first claim pins it instead.
    #[default(None)]
    pub owner: Option<Identity>,
    /// Which *connection* holds the authority right now.
    ///
    /// Not the identity: a standby runs on the same credentials as the
    /// authority it is standing by for, so the identity cannot tell the two
    /// apart. The connection can, which makes this the fencing token --
    /// [`push_states`] checks it, so a sidecar that was replaced while it was
    /// partitioned has its writes refused the moment it comes back, instead of
    /// scribbling over a race it no longer runs.
    #[default(None)]
    pub holder: Option<ConnectionId>,
}

#[table(accessor = player, public)]
pub struct Player {
    #[primary_key]
    pub identity: Identity,
    pub name: String,
    /// 0 when spectating.
    pub car_id: u32,
    pub online: bool,
    pub joined: Timestamp,
}

/// Identity and appearance of a car. Changes rarely, so it is kept apart from
/// the pose that churns 20 times a second.
#[table(accessor = car, public)]
pub struct Car {
    #[primary_key]
    #[auto_inc]
    pub car_id: u32,
    /// `None` for bots, which belong to the sidecar.
    pub owner: Option<Identity>,
    pub name: String,
    /// 0xRRGGBB.
    pub color: u32,
    pub is_bot: bool,
    /// Index into the sidecar's physics world.
    #[unique]
    pub slot: u32,
    pub spawned_at: Timestamp,
}

// ------------------------------------------------------------------- inbox --

/// What a client is doing with the controls right now. This is the *only* thing
/// a player is trusted to assert.
///
/// Deliberately **not** `public`. SpacetimeDB shows a private table to the
/// database's owner and to nobody else, which is exactly the split this needs:
/// the sidecar authenticates as the owner and subscribes to every row, while a
/// player -- who has no business reading a rival's throttle a few milliseconds
/// before it takes effect -- is told the table does not exist.
#[table(accessor = input)]
pub struct Input {
    #[primary_key]
    pub identity: Identity,
    pub car_id: u32,
    /// Monotonic per-client counter; the sidecar echoes it back so the client
    /// can measure the full round trip and know what is safe to discard.
    pub seq: u32,
    /// Client-predicted simulation tick this input belongs to.
    pub tick: u64,
    pub throttle: f32,
    pub steer: f32,
    pub brake: f32,
    pub handbrake: bool,
    /// Bumped by [`request_respawn`]; the sidecar acts on the change.
    pub respawn_seq: u32,
    /// When the last accepted write landed. Half of the rate limiter's state.
    pub at: Timestamp,
    /// The other half: writes banked but not spent. See [`spend`].
    #[default(0.0)]
    pub credits: f32,
}

// ------------------------------------------------------------------ outbox --

/// Authoritative pose. Written only by the sidecar, and the payload
/// [`push_states`] carries: the sidecar sends whole rows, so there is no
/// second copy of this shape to be kept in step with it.
///
/// Most of what follows is not pose at all -- it is the vehicle's internal
/// state, and it is on the wire because rollback needs it. A client that
/// rewinds to an authoritative tick and replays has to start from *exactly*
/// the car the authority had: four wheels turning at their own speeds, four
/// tires part way through building up their cornering force, a body part way
/// through rolling, and an engine at some particular speed in some particular
/// gear. Restore the position and velocity alone and the replay diverges
/// within a few ticks, because the car it is replaying is not the same car.
///
/// See `physics/src/car.rs` for what each one means.
#[table(accessor = car_state, public)]
#[derive(Default)]
pub struct CarState {
    #[primary_key]
    pub car_id: u32,
    pub slot: u32,
    /// Simulation tick this pose is from.
    pub tick: u64,
    /// Last [`Input::seq`] from this car's owner that the sidecar had applied.
    pub ack_seq: u32,

    pub x: f32,
    pub y: f32,
    pub heading: f32,
    pub vx: f32,
    pub vy: f32,
    pub omega: f32,
    pub steer: f32,

    /// Wheel angular velocities, rad/s: front-left, front-right, rear-left,
    /// rear-right. Slip ratio is computed from these, so wheelspin and lock-up
    /// only replay correctly if they come across.
    pub w_fl: f32,
    pub w_fr: f32,
    pub w_rl: f32,
    pub w_rr: f32,

    /// Lateral force each tire has built up so far. A carcass takes a
    /// relaxation length to develop it, which makes it state and not output.
    pub fy_fl: f32,
    pub fy_fr: f32,
    pub fy_rl: f32,
    pub fy_rr: f32,

    /// Body attitude and how fast it is changing. Load transfer lags the
    /// driver's input through these, so they decide what grip each tire has.
    pub roll: f32,
    pub roll_rate: f32,
    pub pitch: f32,
    pub pitch_rate: f32,

    /// Engine speed, rad/s.
    pub engine: f32,
    /// -1 reverse, 1..=6 forward.
    pub gear: i8,
    /// Seconds left of the current shift, during which the clutch is out.
    pub shift: f32,
    /// Clutch engagement, 0..1.
    pub clutch: f32,

    /// Body-frame acceleration. Feeds the instantaneous paths of load
    /// transfer, so like `ax` before it, a client that rolled back without it
    /// would re-simulate a subtly different car.
    pub ax: f32,
    pub ay: f32,

    /// Drives tire smoke and the tachometer. Not simulation state -- but the
    /// browser never simulates anyone else's car, so without these the rest of
    /// the field would drive around in silence with still needles.
    pub wheel_spin: f32,
    pub rpm: f32,

    pub lap: u32,
    pub cp: u32,
    /// Distance around the current lap, for standings.
    pub s: f32,
    /// Offset from the centerline, and the cached nearest-sample hint. Both are
    /// part of the simulation state proper.
    pub lat: f32,
    pub seg: u32,
    pub lap_start: f32,
    pub last_lap: f32,
    pub best_lap: f32,

    /// Collision impulse this tick, for hit effects.
    pub impact: f32,
    pub wall: bool,

    /// Permanent crush on each face of the body, metres.
    ///
    /// Simulation state like the rest of this row, and the least optional part
    /// of it: a damaged car has less downforce, less steering lock, less power
    /// and less grip, so a client that rolled back without knowing about the
    /// damage would replay a car that no longer exists. It is also what the
    /// renderer bends the silhouette by, which is why a wreck looks like one
    /// from every browser watching.
    pub dmg_front: f32,
    pub dmg_rear: f32,
    pub dmg_left: f32,
    pub dmg_right: f32,
}

/// Best laps. Aggregating these is bookkeeping, not simulation, so it belongs
/// in the database rather than the sidecar.
#[table(accessor = lap_record, public)]
pub struct LapRecord {
    #[primary_key]
    pub name: String,
    pub best_lap: f32,
    pub is_bot: bool,
    pub set_at: Timestamp,
}

// -------------------------------------------------------------- lifecycle ---

#[reducer(init)]
pub fn init(ctx: &ReducerContext) {
    ctx.db.config().insert(Config {
        id: 0,
        sidecar: None,
        sidecar_online: false,
        tick_hz: 60,
        snapshot_hz: 20,
        max_cars: MAX_CARS,
        server_tick: 0,
        updated_at: ctx.timestamp,
        physics_fingerprint: 0,
        // `init` runs as the identity that published the module, so this is
        // the one place the owner can be learned without being told.
        owner: Some(ctx.sender()),
        holder: None,
    });
    log::info!(
        "physics-sidecar module initialized, {MAX_CARS} car slots, owner {}",
        ctx.sender()
    );
}

#[reducer(client_connected)]
pub fn client_connected(ctx: &ReducerContext) {
    let id = ctx.sender();
    if let Some(mut p) = ctx.db.player().identity().find(&id) {
        p.online = true;
        ctx.db.player().identity().update(p);
    } else {
        ctx.db.player().insert(Player {
            identity: id,
            name: String::new(),
            car_id: 0,
            online: true,
            joined: ctx.timestamp,
        });
    }
}

#[reducer(client_disconnected)]
pub fn client_disconnected(ctx: &ReducerContext) {
    let id = ctx.sender();

    // If the authority dropped, free the seat immediately: a standby takes it
    // on its next tick, and clients switch to dead reckoning until it does.
    // Matched on the connection, so the authority losing its socket is not
    // confused with a standby on the same credentials losing its.
    let mut cfg = config(ctx);
    if cfg.holder.is_some() && cfg.holder == ctx.connection_id() {
        cfg.sidecar = None;
        cfg.holder = None;
        cfg.sidecar_online = false;
        cfg.updated_at = ctx.timestamp;
        ctx.db.config().id().update(cfg);
        log::warn!("sidecar disconnected -- no authority");
    }

    unseat(ctx, id, false);
}

// ------------------------------------------------------------- authority ----

/// Claim the right to publish authoritative state.
///
/// Two questions, and they are different ones. *May* you be the authority: only
/// the identity that published the module, so a player cannot appoint itself
/// during a handover. *Can* you have it right now: only if the seat is empty,
/// already yours, or its holder has gone quiet for [`LEASE_MICROS`].
///
/// That last clause is the arbitration. Two sidecars on the same credentials --
/// an authority and the standby waiting to replace it -- both pass the identity
/// check, and the database is the only thing that sees both of them, so it is
/// where the tie is broken: whoever's claim commits first becomes the holder,
/// and the loser reads the row and stands by. A partitioned authority does not
/// get a vote, and its writes stop counting the moment it stops holding
/// [`Config::holder`].
///
/// `physics_fingerprint` is what the incoming authority's simulation computes.
/// The module does not check it, having no physics to check it against, which
/// is the entire point. It just publishes it, so every client can check its own
/// copy against the one now deciding the race.
#[reducer]
pub fn claim_authority(ctx: &ReducerContext, physics_fingerprint: u32) -> Result<(), String> {
    let mut cfg = config(ctx);
    let me = ctx
        .connection_id()
        .ok_or("authority must be claimed over a live connection")?;

    match cfg.owner {
        Some(owner) if owner != ctx.sender() => return Err("not the database owner".into()),
        Some(_) => {}
        // A database that predates the column has nobody pinned; the first
        // claim pins it. Loud, because on a fresh database this never happens.
        None => {
            log::warn!("no owner recorded; pinning authority to {}", ctx.sender());
            cfg.owner = Some(ctx.sender());
        }
    }

    if let Some(held) = cfg.holder {
        let quiet = ctx.timestamp.to_micros_since_unix_epoch()
            - cfg.updated_at.to_micros_since_unix_epoch();
        if held != me && quiet < LEASE_MICROS {
            return Err("another sidecar holds the lease".into());
        }
        if held != me {
            log::warn!("lease expired after {quiet} us; taking authority from {held}");
        }
    }

    cfg.sidecar = Some(ctx.sender());
    cfg.holder = Some(me);
    cfg.sidecar_online = true;
    cfg.physics_fingerprint = physics_fingerprint;
    cfg.updated_at = ctx.timestamp;
    ctx.db.config().id().update(cfg);
    // Bot cars are deliberately left alone. The incoming sidecar adopts them
    // from their last published pose and reconciles the count itself, so a
    // handover does not reset the race.
    log::info!("authority claimed by {} on {me}", ctx.sender());
    Ok(())
}

#[reducer]
pub fn release_authority(ctx: &ReducerContext) -> Result<(), String> {
    let mut cfg = require_authority(ctx)?;
    cfg.sidecar = None;
    cfg.holder = None;
    cfg.sidecar_online = false;
    cfg.updated_at = ctx.timestamp;
    ctx.db.config().id().update(cfg);
    Ok(())
}

// ------------------------------------------------------------------ racing --

#[reducer]
pub fn join_race(ctx: &ReducerContext, name: String, color: u32) -> Result<(), String> {
    let id = ctx.sender();
    let mut player = ctx
        .db
        .player()
        .identity()
        .find(&id)
        .ok_or("not connected")?;

    // A reconnect can race the previous connection's disconnect handler, which
    // leaves `car_id` pointing at a car that has already been reaped. Trust the
    // car table, not the stale pointer.
    let still_racing = player.car_id != 0
        && ctx
            .db
            .car()
            .car_id()
            .find(&player.car_id)
            .is_some_and(|c| c.owner == Some(id));
    if still_racing {
        return Ok(()); // joining twice is a no-op, not an error
    }
    if player.car_id != 0 {
        remove_car(ctx, player.car_id);
        player.car_id = 0;
    }
    // Clear any leftover input row: its sequence counter would reject
    // everything a freshly loaded client sends.
    ctx.db.input().identity().delete(&id);

    let display = sanitize_name(&name, "Driver");
    let slot = free_slot(ctx).ok_or("grid is full")?;
    let car = ctx.db.car().insert(Car {
        car_id: 0,
        owner: Some(id),
        name: display.clone(),
        color: color & 0x00ff_ffff,
        is_bot: false,
        slot,
        spawned_at: ctx.timestamp,
    });

    ctx.db.car_state().insert(blank_state(car.car_id, slot));
    ctx.db.input().insert(Input {
        identity: id,
        car_id: car.car_id,
        seq: 0,
        tick: 0,
        throttle: 0.0,
        steer: 0.0,
        brake: 0.0,
        handbrake: false,
        respawn_seq: 0,
        at: ctx.timestamp,
        credits: INPUT_BURST,
    });

    player.name = display;
    player.car_id = car.car_id;
    ctx.db.player().identity().update(player);
    Ok(())
}

#[reducer]
pub fn leave_race(ctx: &ReducerContext) {
    unseat(ctx, ctx.sender(), true);
}

/// The hot path from the client's side: one row, overwritten ~30 times a second.
#[reducer]
pub fn set_input(
    ctx: &ReducerContext,
    seq: u32,
    tick: u64,
    throttle: f32,
    steer: f32,
    brake: f32,
    handbrake: bool,
) -> Result<(), String> {
    let id = ctx.sender();
    let mut row = ctx.db.input().identity().find(&id).ok_or("not racing")?;
    // Late packets are worse than no packet: they would walk the sidecar's view
    // of the controls backwards.
    if seq <= row.seq && row.seq != 0 {
        return Ok(());
    }
    // Nothing stops a client sending faster than the thirty a second the
    // browser sends at. Dropping the excess silently keeps a flood costing its
    // sender a reducer call and everyone else nothing.
    if !spend(&mut row, ctx.timestamp) {
        return Ok(());
    }
    row.seq = seq;
    row.tick = tick;
    row.throttle = clamp(throttle, -1.0, 1.0);
    row.steer = clamp(steer, -1.0, 1.0);
    row.brake = clamp(brake, 0.0, 1.0);
    row.handbrake = handbrake;
    ctx.db.input().identity().update(row);
    Ok(())
}

#[reducer]
pub fn request_respawn(ctx: &ReducerContext) -> Result<(), String> {
    let id = ctx.sender();
    let mut row = ctx.db.input().identity().find(&id).ok_or("not racing")?;
    if !spend(&mut row, ctx.timestamp) {
        return Ok(());
    }
    row.respawn_seq = row.respawn_seq.wrapping_add(1);
    ctx.db.input().identity().update(row);
    Ok(())
}

// ------------------------------------------------------- authority writes ---

/// The sidecar's snapshot. One transaction, every car, ~20 times a second.
///
/// Also the authority's heartbeat: `updated_at` is what [`claim_authority`]
/// measures the lease against, so an authority that is still publishing can
/// never be displaced, and one that has stopped always is.
#[reducer]
pub fn push_states(ctx: &ReducerContext, tick: u64, states: Vec<CarState>) -> Result<(), String> {
    let mut cfg = require_authority(ctx)?;

    for s in states {
        let Some(old) = ctx.db.car_state().car_id().find(&s.car_id) else {
            continue; // car left between the sidecar's read and this write
        };
        if old.tick > tick {
            continue; // out-of-order snapshot
        }
        let (car_id, lap) = (s.car_id, s.best_lap);
        // Which slot a car holds and which tick a pose is from are the
        // module's to say, not the sidecar's; everything else is the physics.
        let row = CarState { slot: old.slot, tick, ..s };
        ctx.db.car_state().car_id().update(row);
        if lap > 0.0 && (old.best_lap <= 0.0 || lap < old.best_lap) {
            record_lap(ctx, car_id, lap);
        }
    }

    cfg.server_tick = tick;
    cfg.sidecar_online = true;
    cfg.updated_at = ctx.timestamp;
    ctx.db.config().id().update(cfg);
    Ok(())
}

#[reducer]
pub fn spawn_bot(ctx: &ReducerContext, name: String, color: u32) -> Result<(), String> {
    require_authority(ctx)?;
    let slot = free_slot(ctx).ok_or("grid is full")?;
    let car = ctx.db.car().insert(Car {
        car_id: 0,
        owner: None,
        name: sanitize_name(&name, "Bot"),
        color: color & 0x00ff_ffff,
        is_bot: true,
        slot,
        spawned_at: ctx.timestamp,
    });
    ctx.db.car_state().insert(blank_state(car.car_id, slot));
    Ok(())
}

#[reducer]
pub fn despawn_bot(ctx: &ReducerContext, car_id: u32) -> Result<(), String> {
    require_authority(ctx)?;
    match ctx.db.car().car_id().find(&car_id) {
        Some(c) if c.is_bot => {
            remove_car(ctx, car_id);
            Ok(())
        }
        Some(_) => Err("that car belongs to a player".into()),
        None => Ok(()),
    }
}

// ------------------------------------------------------------------ helpers -

fn config(ctx: &ReducerContext) -> Config {
    ctx.db
        .config()
        .id()
        .find(&0)
        .expect("config row missing; module was not initialized")
}

/// The fence. Holding the right identity is not enough, because a sidecar that
/// was replaced while it was away still has it, so what is checked is the
/// connection recorded by the winning [`claim_authority`].
fn require_authority(ctx: &ReducerContext) -> Result<Config, String> {
    let cfg = config(ctx);
    if cfg.holder.is_some() && cfg.holder == ctx.connection_id() {
        Ok(cfg)
    } else {
        Err("only the sidecar holding the authority lease may write simulation state".into())
    }
}

/// Take one write out of a client's token bucket, or refuse it.
///
/// `credits` is what was left after the last accepted write and `at` is when
/// that was, so the bucket refills by itself: nothing has to run on a timer,
/// and a client that goes quiet banks up to [`INPUT_BURST`]. One that floods
/// runs dry and is dropped here -- before the row is written, and so before
/// every subscriber pays for the fan-out, which is the cost worth avoiding.
fn spend(row: &mut Input, now: Timestamp) -> bool {
    let elapsed =
        (now.to_micros_since_unix_epoch() - row.at.to_micros_since_unix_epoch()).max(0) as f32;
    let credits = (row.credits + elapsed * 1e-6 * INPUT_RATE_HZ).min(INPUT_BURST);
    if credits < 1.0 {
        return false;
    }
    row.credits = credits - 1.0;
    row.at = now;
    true
}

/// Lowest unused physics slot.
fn free_slot(ctx: &ReducerContext) -> Option<u32> {
    let mut used = 0u32;
    for c in ctx.db.car().iter() {
        if c.slot < MAX_CARS {
            used |= 1 << c.slot;
        }
    }
    (0..MAX_CARS).find(|s| used & (1 << s) == 0)
}

fn blank_state(car_id: u32, slot: u32) -> CarState {
    CarState { car_id, slot, gear: 1, ..Default::default() }
}

/// Take a player off the grid: the car, its pose and its input row all go.
/// Whoever is leaving is still connected; whoever dropped is not.
fn unseat(ctx: &ReducerContext, id: Identity, online: bool) {
    if let Some(mut p) = ctx.db.player().identity().find(&id) {
        let car_id = p.car_id;
        p.car_id = 0;
        p.online = online;
        ctx.db.player().identity().update(p);
        remove_car(ctx, car_id);
    }
    ctx.db.input().identity().delete(&id);
}

fn remove_car(ctx: &ReducerContext, car_id: u32) {
    if car_id == 0 {
        return;
    }
    ctx.db.car().car_id().delete(&car_id);
    ctx.db.car_state().car_id().delete(&car_id);
}

fn record_lap(ctx: &ReducerContext, car_id: u32, lap: f32) {
    let Some(car) = ctx.db.car().car_id().find(&car_id) else {
        return;
    };
    match ctx.db.lap_record().name().find(&car.name) {
        Some(rec) if rec.best_lap <= lap => {}
        Some(mut rec) => {
            rec.best_lap = lap;
            rec.set_at = ctx.timestamp;
            ctx.db.lap_record().name().update(rec);
        }
        None => {
            ctx.db.lap_record().insert(LapRecord {
                name: car.name,
                best_lap: lap,
                is_bot: car.is_bot,
                set_at: ctx.timestamp,
            });
        }
    }
}

fn sanitize_name(raw: &str, fallback: &str) -> String {
    let cleaned: String = raw
        .trim()
        .chars()
        .filter(|c| !c.is_control())
        .take(16)
        .collect();
    if cleaned.is_empty() {
        fallback.to_string()
    } else {
        cleaned
    }
}

/// `f32::clamp` with an answer for NaN, which a client is free to send.
fn clamp(v: f32, lo: f32, hi: f32) -> f32 {
    if v.is_nan() { 0.0 } else { v.clamp(lo, hi) }
}
