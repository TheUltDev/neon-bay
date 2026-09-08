//! SpacetimeDB module for the physics-sidecar demo.
//!
//! Note what is *not* here: there is no simulation, no integrator, no collision
//! code, not even a track. The module is three things and nothing more:
//!
//! * an **inbox** -- clients write their controller state into [`input`];
//! * a **registry** -- who is connected, which car is theirs, who is allowed to
//!   be the authority;
//! * an **outbox** -- the sidecar writes authoritative poses into [`car_state`]
//!   and SpacetimeDB fans them out to every subscriber.
//!
//! The security model is unchanged from running physics inside the module: a
//! client can only ever say "I am holding the throttle down". It cannot say
//! where its car is. The difference is that the code deciding what that means
//! runs in a process that can be scaled, profiled and hot-restarted separately
//! from the database.

use spacetimedb::{reducer, table, Identity, ReducerContext, Table, Timestamp};

/// Physics world slots. Must match `physics::MAX_CARS`; the sidecar refuses to
/// start if [`config`] disagrees with the crate it was built against.
const MAX_CARS: u32 = 24;

// ---------------------------------------------------------------- registry --

#[table(accessor = config, public)]
pub struct Config {
    #[primary_key]
    pub id: u32,
    /// Identity allowed to call [`push_states`]. Empty until a sidecar claims it.
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
#[table(accessor = input, public)]
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
    pub at: Timestamp,
}

// ------------------------------------------------------------------ outbox --

/// Authoritative pose. Written only by the sidecar.
#[table(accessor = car_state, public)]
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
    /// Filtered longitudinal acceleration. Cosmetically irrelevant, but the
    /// integrator feeds it back into load transfer, so a client that rolls back
    /// without it would re-simulate a subtly different car.
    pub ax: f32,
    pub wheel_spin: f32,
    pub rpm: f32,
    pub gear: u8,
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
}

/// One car's pose inside a [`push_states`] batch.
#[derive(spacetimedb::SpacetimeType, Clone)]
pub struct StateUpdate {
    pub car_id: u32,
    pub ack_seq: u32,
    pub x: f32,
    pub y: f32,
    pub heading: f32,
    pub vx: f32,
    pub vy: f32,
    pub omega: f32,
    pub steer: f32,
    pub ax: f32,
    pub wheel_spin: f32,
    pub rpm: f32,
    pub gear: u8,
    pub lap: u32,
    pub cp: u32,
    pub s: f32,
    pub lat: f32,
    pub seg: u32,
    pub lap_start: f32,
    pub last_lap: f32,
    pub best_lap: f32,
    pub impact: f32,
    pub wall: bool,
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
    });
    log::info!("physics-sidecar module initialized, {MAX_CARS} car slots");
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

    // If the authority dropped, say so loudly: clients switch to dead reckoning
    // and stop trusting their own prediction.
    let mut cfg = config(ctx);
    if cfg.sidecar == Some(id) {
        cfg.sidecar_online = false;
        cfg.updated_at = ctx.timestamp;
        ctx.db.config().id().update(cfg);
        log::warn!("sidecar disconnected -- no authority");
    }

    if let Some(mut p) = ctx.db.player().identity().find(&id) {
        p.online = false;
        let car_id = p.car_id;
        p.car_id = 0;
        ctx.db.player().identity().update(p);
        remove_car(ctx, car_id);
    }
    ctx.db.input().identity().delete(&id);
}

// ------------------------------------------------------------- authority ----

/// Claim the right to publish authoritative state.
///
/// First caller wins. A second sidecar can only take over once the first has
/// disconnected, which makes a restart seamless but blocks a rogue client from
/// stealing the simulation out from under a live one.
///
/// `physics_fingerprint` is what the incoming authority's simulation computes.
/// The module does not check it -- it has no physics to check it against, which
/// is the entire point of the sidecar -- it just publishes it so that every
/// client can check its own copy against the one now deciding the race.
#[reducer]
pub fn claim_authority(ctx: &ReducerContext, physics_fingerprint: u32) -> Result<(), String> {
    let mut cfg = config(ctx);
    match cfg.sidecar {
        Some(existing) if existing != ctx.sender() && cfg.sidecar_online => {
            return Err("another sidecar already holds authority".into());
        }
        _ => {}
    }
    cfg.sidecar = Some(ctx.sender());
    cfg.sidecar_online = true;
    cfg.physics_fingerprint = physics_fingerprint;
    cfg.updated_at = ctx.timestamp;
    ctx.db.config().id().update(cfg);
    // Bot cars are deliberately left alone. The incoming sidecar adopts them
    // from their last published pose and reconciles the count itself, so a
    // restart does not reset the race.
    log::info!("authority claimed by {}", ctx.sender());
    Ok(())
}

#[reducer]
pub fn release_authority(ctx: &ReducerContext) -> Result<(), String> {
    let mut cfg = require_authority(ctx)?;
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
    });

    player.name = display;
    player.car_id = car.car_id;
    ctx.db.player().identity().update(player);
    Ok(())
}

#[reducer]
pub fn leave_race(ctx: &ReducerContext) {
    let id = ctx.sender();
    if let Some(mut p) = ctx.db.player().identity().find(&id) {
        let car_id = p.car_id;
        p.car_id = 0;
        ctx.db.player().identity().update(p);
        remove_car(ctx, car_id);
    }
    ctx.db.input().identity().delete(&id);
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
    row.seq = seq;
    row.tick = tick;
    row.throttle = clamp(throttle, -1.0, 1.0);
    row.steer = clamp(steer, -1.0, 1.0);
    row.brake = clamp(brake, 0.0, 1.0);
    row.handbrake = handbrake;
    row.at = ctx.timestamp;
    ctx.db.input().identity().update(row);
    Ok(())
}

#[reducer]
pub fn request_respawn(ctx: &ReducerContext) -> Result<(), String> {
    let id = ctx.sender();
    let mut row = ctx.db.input().identity().find(&id).ok_or("not racing")?;
    row.respawn_seq = row.respawn_seq.wrapping_add(1);
    row.at = ctx.timestamp;
    ctx.db.input().identity().update(row);
    Ok(())
}

// ------------------------------------------------------- authority writes ---

/// The sidecar's snapshot. One transaction, every car, ~20 times a second.
#[reducer]
pub fn push_states(ctx: &ReducerContext, tick: u64, states: Vec<StateUpdate>) -> Result<(), String> {
    let mut cfg = require_authority(ctx)?;

    for s in &states {
        let Some(existing) = ctx.db.car_state().car_id().find(&s.car_id) else {
            continue; // car left between the sidecar's read and this write
        };
        if existing.tick > tick {
            continue; // out-of-order snapshot
        }
        let improved = s.best_lap > 0.0 && (existing.best_lap <= 0.0 || s.best_lap < existing.best_lap);
        ctx.db.car_state().car_id().update(CarState {
            car_id: s.car_id,
            slot: existing.slot,
            tick,
            ack_seq: s.ack_seq,
            x: s.x,
            y: s.y,
            heading: s.heading,
            vx: s.vx,
            vy: s.vy,
            omega: s.omega,
            steer: s.steer,
            ax: s.ax,
            wheel_spin: s.wheel_spin,
            rpm: s.rpm,
            gear: s.gear,
            lap: s.lap,
            cp: s.cp,
            s: s.s,
            lat: s.lat,
            seg: s.seg,
            lap_start: s.lap_start,
            last_lap: s.last_lap,
            best_lap: s.best_lap,
            impact: s.impact,
            wall: s.wall,
        });
        if improved {
            record_lap(ctx, s.car_id, s.best_lap);
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

fn require_authority(ctx: &ReducerContext) -> Result<Config, String> {
    let cfg = config(ctx);
    if cfg.sidecar == Some(ctx.sender()) {
        Ok(cfg)
    } else {
        Err("only the registered sidecar may write simulation state".into())
    }
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
    CarState {
        car_id,
        slot,
        tick: 0,
        ack_seq: 0,
        x: 0.0,
        y: 0.0,
        heading: 0.0,
        vx: 0.0,
        vy: 0.0,
        omega: 0.0,
        steer: 0.0,
        ax: 0.0,
        wheel_spin: 0.0,
        rpm: 0.0,
        gear: 1,
        lap: 0,
        cp: 0,
        s: 0.0,
        lat: 0.0,
        seg: 0,
        lap_start: 0.0,
        last_lap: 0.0,
        best_lap: 0.0,
        impact: 0.0,
        wall: false,
    }
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

fn clamp(v: f32, lo: f32, hi: f32) -> f32 {
    if v.is_nan() {
        0.0
    } else if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}
