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

use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};

use physics::bot::BotBrain;
use physics::{CarInput, CarState as PhysicsCar, World, MAX_CARS};
use spacetimedb_sdk::{Identity, Table};

use crate::module_bindings::*;

/// Ticks between snapshots. 60 Hz simulation / 3 = 20 Hz on the wire.
pub const SNAPSHOT_EVERY: u64 = 3;

/// Rebuild the simulation's view of a car from a published row. The inverse of
/// the mapping in [`Authority::publish`].
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
        slip_f: 0.0,
        slip_r: 0.0,
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
        impact: 0.0,
        wall: 0.0,
        active: 1.0,
    }
}

#[derive(Clone)]
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
    identity: Identity,
    has_authority: bool,
    last_claim: Instant,
    /// Cached: scoring it runs a scripted race, and re-claiming is a loop.
    fingerprint: u32,
    // --- telemetry ---
    pub sim_time: Duration,
    pub ticks_this_second: u32,
    pub snapshots_this_second: u32,
    pub inputs_applied: u32,
    last_report: Instant,
}

impl Authority {
    pub fn new(identity: Identity) -> Self {
        Authority {
            world: World::new(),
            slots: [const { None }; MAX_CARS],
            next_grid: 0,
            identity,
            has_authority: false,
            last_claim: Instant::now() - Duration::from_secs(5),
            fingerprint: physics::fingerprint(),
            sim_time: Duration::ZERO,
            ticks_this_second: 0,
            snapshots_this_second: 0,
            inputs_applied: 0,
            last_report: Instant::now(),
        }
    }

    /// One simulation tick.
    pub fn step(&mut self, conn: &DbConnection) {
        self.check_authority(conn);
        self.sync_cars(conn);
        self.pull_inputs(conn);
        self.drive_bots();

        let t0 = Instant::now();
        self.world.step(self.world.active);
        self.sim_time += t0.elapsed();
        self.ticks_this_second += 1;

        if self.has_authority && self.world.tick % SNAPSHOT_EVERY == 0 {
            self.publish(conn);
        }
    }

    /// Claim, or re-claim, the right to write simulation state.
    fn check_authority(&mut self, conn: &DbConnection) {
        let Some(cfg) = conn.db.config().id().find(&0) else {
            return;
        };
        let mine = cfg.sidecar == Some(self.identity) && cfg.sidecar_online;
        if mine {
            if !self.has_authority {
                println!("[authority] granted; publishing from tick {}", self.world.tick);
            }
            self.has_authority = true;
            return;
        }
        if self.has_authority {
            println!("[authority] lost, re-claiming");
        }
        self.has_authority = false;
        if self.last_claim.elapsed() > Duration::from_secs(1) {
            self.last_claim = Instant::now();
            let _ = conn.reducers.claim_authority(self.fingerprint);
        }
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
                ack_seq: 0,
                last_respawn: 0,
                pending: VecDeque::new(),
                current: CarInput::default(),
            });
            match resumed {
                Some(tick) => println!(
                    "[grid] slot {slot} <- {} \"{}\" (car {}) resumed from tick {tick}",
                    if car.is_bot { "bot" } else { "player" },
                    car.name,
                    car.car_id
                ),
                None => println!(
                    "[grid] slot {slot} <- {} \"{}\" (car {}) on the grid",
                    if car.is_bot { "bot" } else { "player" },
                    car.name,
                    car.car_id
                ),
            }
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
        let by_car: HashMap<u32, usize> = (0..MAX_CARS)
            .filter_map(|i| self.slots[i].as_ref().map(|s| (s.car_id, i)))
            .collect();

        for row in conn.db.input().iter() {
            let Some(&slot) = by_car.get(&row.car_id) else {
                continue;
            };
            let Some(state) = self.slots[slot].as_mut() else {
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
            states.push(StateUpdate {
                car_id: s.car_id,
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
        if states.is_empty() {
            return;
        }
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
            self.sim_time.as_secs_f32() * 1e6 / ticks as f32,
            self.sim_time.as_secs_f32() / elapsed * 100.0,
            self.inputs_applied,
            if self.has_authority { "" } else { " | NO AUTHORITY" },
        );
        self.sim_time = Duration::ZERO;
        self.ticks_this_second = 0;
        self.snapshots_this_second = 0;
        self.inputs_applied = 0;
    }
}
