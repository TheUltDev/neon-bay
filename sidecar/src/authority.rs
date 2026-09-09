//! The authoritative simulation.
//!
//! Each tick this pulls the current controller state out of the SpacetimeDB
//! client cache, drives the bots, advances the physics world and -- every third
//! tick -- publishes every car's pose back through the module.
//!
//! It is deliberately *pull-based*: rather than wiring up `on_insert`/`on_delete`
//! callbacks and marshalling events across threads, the loop just reads the
//! materialized view it already has. With a couple of dozen rows that costs
//! nothing and removes an entire class of ordering bug.

use std::collections::VecDeque;
use std::time::{Duration, Instant};

use physics::bot::BotBrain;
use physics::{CarInput, CarState as PhysicsCar, World, MAX_CARS};
use spacetimedb_sdk::{ConnectionId, Table};

use crate::module_bindings::*;

/// Ticks between snapshots. 60 Hz simulation / 3 = 20 Hz on the wire.
pub const SNAPSHOT_EVERY: u64 = 3;

/// How long to wait between claims when the seat is empty, and when it is not.
/// An empty seat is the gap a standby exists to close, so it is worth one
/// attempt per round trip; a held one is worth checking in case its lease runs
/// out, and nothing more.
const CLAIM_FAST: Duration = Duration::from_millis(100);
const CLAIM_SLOW: Duration = Duration::from_secs(1);

const BOT_NAMES: [&str; 24] = [
    "VECTOR", "NITRO", "HALCYON", "RIPTIDE", "ZEPHYR", "OBSIDIAN", "QUASAR", "MAVERICK", "TEMPEST",
    "CINDER", "ONYX", "VAPOR", "PHANTOM", "COBALT", "MERIDIAN", "VORTEX", "EMBER", "HALIDE",
    "SABLE", "KESTREL", "AURORA", "BASALT", "TALON", "MIRAGE",
];
const BOT_COLORS: [u32; 24] = [
    0xff4d6d, 0x4dd2ff, 0xffd166, 0x8affc1, 0xc77dff, 0xff9f45, 0x5fa8ff, 0xff6ec7, 0x9dff5f,
    0x00e5c0, 0xffe066, 0xff5f5f, 0xa78bfa, 0x34d399, 0xf59e0b, 0x38bdf8, 0xfb7185, 0x84cc16,
    0x22d3ee, 0xe879f9, 0xfacc15, 0x60a5fa, 0xf97316, 0x2dd4bf,
];

/// Rebuild the simulation's view of a car from a published row. The inverse of
/// the mapping in [`Authority::publish`].
///
/// The four fields left at their default -- `slip_f`, `slip_r`, `impact`,
/// `wall` -- are the ones not on the wire. Each is overwritten before it is
/// next read, so resuming without them is exact rather than approximate.
fn from_row(r: &CarState) -> PhysicsCar {
    PhysicsCar {
        x: r.x,
        y: r.y,
        heading: r.heading,
        vx: r.vx,
        vy: r.vy,
        omega: r.omega,
        steer: r.steer,
        ax: r.ax,
        wheel_spin: r.wheel_spin,
        rpm: r.rpm,
        gear: r.gear as f32,
        s: r.s,
        lat: r.lat,
        seg: r.seg as f32,
        lap: r.lap as f32,
        cp: r.cp as f32,
        lap_start: r.lap_start,
        last_lap: r.last_lap,
        best_lap: r.best_lap,
        active: 1.0,
        ..Default::default()
    }
}

#[derive(Clone, Default)]
struct Slot {
    car_id: u32,
    is_bot: bool,
    brain: Option<BotBrain>,
    /// Last input sequence consumed, echoed back so clients can measure the
    /// full client -> module -> sidecar -> module -> client round trip.
    ack_seq: u32,
    last_respawn: u32,
    /// Inputs that have arrived but whose tick has not come round yet, oldest
    /// first. See [`Authority::pull_inputs`].
    pending: VecDeque<(u64, CarInput)>,
    /// Input currently being applied, held until a newer one comes due.
    current: CarInput,
}

pub struct Authority {
    pub world: World,
    slots: [Option<Slot>; MAX_CARS],
    /// Rotating grid position so late joiners do not spawn inside each other.
    next_grid: usize,
    /// This process's connection, which is what the module's lease is held
    /// against. Not the identity: a standby runs on the same credentials.
    connection: ConnectionId,
    /// How many bots this sidecar wants on the grid once it is the authority.
    bots: usize,
    has_authority: bool,
    /// Newest tick adopted while standing by. See [`Authority::follow`].
    followed: u64,
    /// Who held the lease last tick, so the moment it comes free is noticed
    /// rather than waited for.
    held_by: Option<ConnectionId>,
    last_claim: Instant,
    /// Cached: scoring it runs a scripted race, and re-claiming is a loop.
    fingerprint: u32,
    // --- telemetry ---
    /// Everything one tick costs: pumping the connection, reading the cache,
    /// driving the bots, simulating and publishing. Timed by the caller, which
    /// is the only place that can see the whole tick.
    pub tick_time: Duration,
    pub ticks_this_second: u32,
    pub snapshots_this_second: u32,
    pub inputs_applied: u32,
    last_report: Instant,
}

impl Authority {
    pub fn new(connection: ConnectionId, bots: usize) -> Self {
        Authority {
            world: World::new(),
            slots: [const { None }; MAX_CARS],
            next_grid: 0,
            connection,
            bots: bots.min(BOT_NAMES.len()),
            has_authority: false,
            followed: 0,
            held_by: None,
            last_claim: Instant::now() - Duration::from_secs(5),
            fingerprint: physics::fingerprint(),
            tick_time: Duration::ZERO,
            ticks_this_second: 0,
            snapshots_this_second: 0,
            inputs_applied: 0,
            last_report: Instant::now(),
        }
    }

    /// One simulation tick, or one tick of standing by.
    pub fn step(&mut self, conn: &DbConnection) {
        self.sync_cars(conn);
        self.check_authority(conn);
        self.pull_inputs(conn);
        self.ticks_this_second += 1;
        // Standing by: the holder's snapshots are the truth, and simulating a
        // second opinion would only be thrown away when the next one lands.
        if !self.has_authority {
            return;
        }
        self.drive_bots();
        self.world.step(self.world.active);
        if self.world.tick % SNAPSHOT_EVERY == 0 {
            self.publish(conn);
        }
    }

    /// Hold the authority, take it, or stand by ready to.
    ///
    /// Every process runs this, and the one holding the lease is simply the one
    /// whose claim the database committed first. The loser is not idle: it
    /// tracks the race it is not running, so that when the seat comes free --
    /// because the holder exited, or because it went quiet long enough for its
    /// lease to lapse -- it can carry on rather than start.
    fn check_authority(&mut self, conn: &DbConnection) {
        let Some(cfg) = conn.db.config().id().find(&0) else {
            return;
        };
        if cfg.holder == Some(self.connection) {
            if !self.has_authority {
                self.has_authority = true;
                self.world.tick = cfg.server_tick + 1;
                println!("[authority] granted; publishing from tick {}", self.world.tick);
                self.reconcile_bots(conn);
            }
            return;
        }
        if self.has_authority {
            println!("[authority] lease lost to another sidecar; standing by");
            self.has_authority = false;
            self.followed = 0;
        }
        self.follow(conn, cfg.server_tick);
        // A seat that has just come free is worth taking on this tick rather
        // than on the next poll: the gap a standby exists to close is measured
        // in milliseconds, and waiting out a retry interval would dominate it.
        if cfg.holder.is_none() && self.held_by.is_some() {
            self.last_claim = Instant::now() - CLAIM_SLOW;
        }
        self.held_by = cfg.holder;
        let wait = if cfg.holder.is_none() { CLAIM_FAST } else { CLAIM_SLOW };
        if self.last_claim.elapsed() > wait {
            self.last_claim = Instant::now();
            let _ = conn.reducers.claim_authority(self.fingerprint);
        }
    }

    /// Track the published race without writing to it.
    ///
    /// A standby that only watched would still be a cold start: the moment a
    /// player touched a control it never saw, its world would be somewhere
    /// else. Adopting each snapshot as it lands costs one copy per car per
    /// 50 ms and makes a promotion a continuation instead of a restart.
    fn follow(&mut self, conn: &DbConnection, server_tick: u64) {
        if server_tick <= self.followed {
            return;
        }
        self.followed = server_tick;
        for st in conn.db.car_state().iter() {
            if (st.slot as usize) < MAX_CARS && st.tick > 0 {
                self.world.adopt(st.slot as usize, from_row(&st));
            }
        }
        self.world.tick = server_tick + 1;
    }

    /// Bring the bot field to the requested count, keeping whoever is already
    /// out there. A sidecar that has just taken over inherits the previous
    /// one's bots -- it resumed them from their last published pose -- so this
    /// only adds or removes the difference.
    fn reconcile_bots(&self, conn: &DbConnection) {
        let existing: Vec<(u32, String)> = conn
            .db
            .car()
            .iter()
            .filter(|c| c.is_bot)
            .map(|c| (c.car_id, c.name.clone()))
            .collect();

        for (car_id, name) in existing.iter().skip(self.bots) {
            println!("[grid] retiring bot {name}");
            let _ = conn.reducers.despawn_bot(*car_id);
        }
        let mut added = 0;
        for (i, name) in BOT_NAMES.iter().enumerate() {
            if existing.len() + added >= self.bots {
                break;
            }
            if existing.iter().any(|(_, n)| n == name) {
                continue;
            }
            let _ = conn.reducers.spawn_bot((*name).to_string(), BOT_COLORS[i]);
            added += 1;
        }
        println!(
            "[grid] bots: {} already racing, {added} added, {} retired",
            existing.len().min(self.bots),
            existing.len().saturating_sub(self.bots)
        );
    }

    /// Reconcile the physics world with the `car` table.
    fn sync_cars(&mut self, conn: &DbConnection) {
        let mut seen = [false; MAX_CARS];

        for car in conn.db.car().iter() {
            let slot = car.slot as usize;
            if slot >= MAX_CARS {
                continue;
            }
            seen[slot] = true;
            let fresh = match &self.slots[slot] {
                Some(s) => s.car_id != car.car_id,
                None => true,
            };
            if !fresh {
                continue;
            }
            // If the outbox already holds a pose for this car, resume from it.
            // Everything the integrator reads back is on the wire, so a sidecar
            // restart is invisible to the driver: they keep their position,
            // their momentum and their lap.
            let resumed = conn
                .db
                .car_state()
                .car_id()
                .find(&car.car_id)
                .filter(|st| st.tick > 0)
                .map(|st| {
                    self.world.adopt(slot, from_row(&st));
                    st.tick
                });
            if resumed.is_none() {
                let grid = self.next_grid % MAX_CARS;
                self.next_grid += 1;
                self.world.spawn(slot, grid);
            }
            self.slots[slot] = Some(Slot {
                car_id: car.car_id,
                is_bot: car.is_bot,
                brain: car.is_bot.then(|| BotBrain::new(car.car_id.wrapping_mul(2654435761))),
                ..Default::default()
            });
            println!(
                "[grid] slot {slot} <- {} \"{}\" (car {}) {}",
                if car.is_bot { "bot" } else { "player" },
                car.name,
                car.car_id,
                match resumed {
                    Some(tick) => format!("resumed from tick {tick}"),
                    None => "on the grid".into(),
                }
            );
        }

        for slot in 0..MAX_CARS {
            if !seen[slot] && self.slots[slot].is_some() {
                println!("[grid] slot {slot} released");
                self.slots[slot] = None;
                self.world.despawn(slot);
            }
        }
    }

    /// Copy controller state out of the `input` table into the simulation.
    ///
    /// Inputs are *scheduled*, not applied on arrival. A client runs a few ticks
    /// ahead of the authority and stamps each input with the tick it belongs to;
    /// holding it until that tick comes round is what lets the browser's
    /// prediction match the authority exactly instead of approximately. An input
    /// that shows up late is applied immediately -- there is no rewinding the
    /// authority -- and the client absorbs the difference on its next rollback.
    fn pull_inputs(&mut self, conn: &DbConnection) {
        for row in conn.db.input().iter() {
            let Some((slot, state)) = self.slots.iter_mut().enumerate().find_map(|(i, s)| {
                s.as_mut().filter(|s| s.car_id == row.car_id).map(|s| (i, s))
            }) else {
                continue;
            };
            if row.seq != state.ack_seq {
                self.inputs_applied += 1;
                state.ack_seq = row.seq;
                state.pending.push_back((
                    row.tick,
                    CarInput {
                        throttle: row.throttle,
                        steer: row.steer,
                        brake: row.brake,
                        handbrake: if row.handbrake { 1.0 } else { 0.0 },
                    },
                ));
                // A client with a wildly wrong clock could otherwise queue
                // forever; a second of backlog is already far too much.
                while state.pending.len() > 64 {
                    state.pending.pop_front();
                }
            }
            if row.respawn_seq != state.last_respawn {
                state.last_respawn = row.respawn_seq;
                self.world.respawn_in_place(slot);
                state.pending.clear();
                println!("[respawn] slot {slot}");
            }
        }

        // Promote whatever is now due. `world.tick` is still the pre-step value,
        // and the input for tick T is the one applied during the step that takes
        // the world from T to T+1 -- the same convention the client uses.
        let now = self.world.tick;
        for slot in 0..MAX_CARS {
            let Some(state) = self.slots[slot].as_mut() else {
                continue;
            };
            while state.pending.front().is_some_and(|(t, _)| *t <= now) {
                state.current = state.pending.pop_front().unwrap().1;
            }
            // Nothing due but something waiting far in the past means the client
            // clock drifted behind us; take the oldest rather than stall.
            if state.pending.len() > 12 {
                state.current = state.pending.pop_front().unwrap().1;
            }
            self.world.inputs[slot] = state.current;
        }
    }

    /// The part that actually justifies a sidecar: AI that would otherwise be
    /// burning transaction time inside the database.
    fn drive_bots(&mut self) {
        let tick = self.world.tick;
        for slot in 0..MAX_CARS {
            let Some(s) = &self.slots[slot] else { continue };
            let Some(brain) = &s.brain else { continue };
            if !self.world.is_active(slot) {
                continue;
            }
            self.world.inputs[slot] =
                brain.drive(slot, &self.world.cars, self.world.active, &self.world.track, tick);
        }
    }

    /// Publish every live car in a single transaction.
    fn publish(&mut self, conn: &DbConnection) {
        let mut states = Vec::with_capacity(MAX_CARS);
        for slot in 0..MAX_CARS {
            let Some(s) = &self.slots[slot] else { continue };
            if !self.world.is_active(slot) {
                continue;
            }
            let c = &self.world.cars[slot];
            states.push(CarState {
                car_id: s.car_id,
                slot: slot as u32,
                tick: self.world.tick,
                ack_seq: s.ack_seq,
                x: c.x,
                y: c.y,
                heading: c.heading,
                vx: c.vx,
                vy: c.vy,
                omega: c.omega,
                steer: c.steer,
                ax: c.ax,
                wheel_spin: c.wheel_spin,
                rpm: c.rpm,
                gear: c.gear as u8,
                lap: c.lap as u32,
                cp: c.cp as u32,
                s: c.s,
                lat: c.lat,
                seg: c.seg as u32,
                lap_start: c.lap_start,
                last_lap: c.last_lap,
                best_lap: c.best_lap,
                impact: c.impact,
                wall: c.wall > 0.5,
            });
        }
        // An empty grid still publishes, because this call is also the lease's
        // heartbeat: a sidecar that went quiet because there was nothing to say
        // would look exactly like one that had wedged, and lose the seat to a
        // standby every two seconds.
        if let Err(e) = conn.reducers.push_states(self.world.tick, states) {
            eprintln!("[publish] {e}");
        }
        self.snapshots_this_second += 1;
    }

    /// Once-a-second status line. Also the load figure that makes the case for
    /// moving this work out of the database in the first place.
    pub fn report(&mut self) {
        if self.last_report.elapsed() < Duration::from_secs(1) {
            return;
        }
        let elapsed = self.last_report.elapsed().as_secs_f32();
        self.last_report = Instant::now();
        let ticks = self.ticks_this_second.max(1);
        let cars = (0..MAX_CARS).filter(|i| self.world.is_active(*i)).count();
        let bots = self
            .slots
            .iter()
            .filter(|s| s.as_ref().is_some_and(|s| s.is_bot))
            .count();
        println!(
            "[t{:>8}] {:>4.1} Hz sim | {:>4.1} Hz snap | {} cars ({} bots) | {:>5.1} us/tick | {:>4.1}% of budget | {} inputs/s{}",
            self.world.tick,
            self.ticks_this_second as f32 / elapsed,
            self.snapshots_this_second as f32 / elapsed,
            cars,
            bots,
            self.tick_time.as_secs_f32() * 1e6 / ticks as f32,
            self.tick_time.as_secs_f32() / elapsed * 100.0,
            self.inputs_applied,
            if self.has_authority { "" } else { " | STANDBY" },
        );
        self.tick_time = Duration::ZERO;
        self.ticks_this_second = 0;
        self.snapshots_this_second = 0;
        self.inputs_applied = 0;
    }
}
