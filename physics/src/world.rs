//! The simulated world: N cars, the circuit, collisions and lap bookkeeping.
//!
//! [`World::step`] takes a *simulation mask*. Cars whose bit is set are
//! integrated; cars whose bit is clear are treated as immovable colliders. That
//! single knob is what lets the same code run in two very different roles:
//!
//! * the sidecar steps with every car in the mask -- it is the authority;
//! * the browser steps with only the local player in the mask, and parks the
//!   other cars at their interpolated network positions, so you can still lean
//!   on a rival mid-corner without the client ever claiming to own their state.

use crate::car::{self, CarInput, CarState, CIRCLE_OFF, CIRCLE_R, INV_MASS, IZ, MASS};
use crate::math::{abs, clamp, signum, V2};
use crate::track::{Track, CHECKPOINTS, NO_HINT};

pub const MAX_CARS: usize = 24;

/// The slots set in `bits`, lowest first.
///
/// Same order as scanning `0..MAX_CARS` and testing each one -- which is what
/// the collision result depends on -- but without walking the empty slots. A
/// world holding seven cars in twenty-four slots spends most of a step's
/// bookkeeping on the seventeen that are not there.
#[inline]
fn slots(mut bits: u32) -> impl Iterator<Item = usize> {
    core::iter::from_fn(move || {
        if bits == 0 {
            return None;
        }
        let i = bits.trailing_zeros() as usize;
        bits &= bits - 1;
        Some(i)
    })
}

/// Restitution against the barriers. Low: walls eat your speed.
const WALL_RESTITUTION: f32 = 0.26;
/// Restitution car vs car. Higher: contact is bouncy and readable.
const CAR_RESTITUTION: f32 = 0.42;
const WALL_FRICTION: f32 = 0.55;

pub struct World {
    pub track: Track,
    pub cars: [CarState; MAX_CARS],
    pub inputs: [CarInput; MAX_CARS],
    /// Bit `i` set means slot `i` holds a live car.
    pub active: u32,
    pub tick: u64,
}

impl Default for World {
    fn default() -> Self {
        Self::new()
    }
}

impl World {
    pub fn new() -> World {
        World {
            track: Track::new(),
            cars: [CarState::default(); MAX_CARS],
            inputs: [CarInput::default(); MAX_CARS],
            active: 0,
            tick: 0,
        }
    }

    #[inline]
    pub fn is_active(&self, i: usize) -> bool {
        i < MAX_CARS && (self.active & (1 << i)) != 0
    }

    /// Put a car on the grid and mark its slot live.
    pub fn spawn(&mut self, i: usize, grid_slot: usize) {
        if i >= MAX_CARS {
            return;
        }
        let (pos, heading) = self.track.grid_slot(grid_slot);
        let car = &mut self.cars[i];
        *car = CarState::default();
        car.place(pos, heading);
        car.active = 1.0;
        car.lap = 1.0;
        car.cp = 1.0;
        car.lap_start = self.tick as f32;
        car.best_lap = 0.0;
        // Prime the track hint and arc position.
        let hit = self.track.nearest(pos, NO_HINT);
        car.seg = hit.idx as f32;
        car.s = hit.s;
        car.lat = hit.lat;
        self.active |= 1 << i;
        self.inputs[i] = CarInput::default();
    }

    /// Take over a car whose state came from somewhere else -- a snapshot the
    /// previous authority published, say. The track hint is recomputed from
    /// scratch because the incoming one cannot be trusted.
    pub fn adopt(&mut self, i: usize, state: CarState) {
        if i >= MAX_CARS {
            return;
        }
        self.cars[i] = state;
        self.cars[i].active = 1.0;
        let hit = self.track.nearest(self.cars[i].pos(), NO_HINT);
        self.cars[i].seg = hit.idx as f32;
        self.cars[i].s = hit.s;
        self.cars[i].lat = hit.lat;
        self.active |= 1 << i;
        self.inputs[i] = CarInput::default();
    }

    pub fn despawn(&mut self, i: usize) {
        if i < MAX_CARS {
            self.active &= !(1 << i);
            self.cars[i] = CarState::default();
            self.inputs[i] = CarInput::default();
        }
    }

    /// Drop a car back on the racing line where it is, pointing the right way.
    pub fn respawn_in_place(&mut self, i: usize) {
        if !self.is_active(i) {
            return;
        }
        let pos = self.cars[i].pos();
        let hit = self.track.nearest(pos, NO_HINT);
        let (p, tan, _) = self.track.sample(hit.s);
        let heading = crate::math::atan2(tan.y, tan.x);
        let keep = self.cars[i];
        let car = &mut self.cars[i];
        car.place(p, heading);
        car.lap = keep.lap;
        car.cp = keep.cp;
        car.lap_start = keep.lap_start;
        car.last_lap = keep.last_lap;
        car.best_lap = keep.best_lap;
        car.s = hit.s;
        car.active = 1.0;
    }

    /// Advance the world one tick (1/60 s).
    ///
    /// `sim_mask` selects which cars are integrated; the rest still collide but
    /// never move.
    pub fn step(&mut self, sim_mask: u32) {
        let mask = sim_mask & self.active;
        self.tick = self.tick.wrapping_add(1);

        for i in slots(mask) {
            self.cars[i].impact = 0.0;
            self.cars[i].wall = 0.0;
        }

        for _ in 0..car::SUBSTEPS {
            for i in slots(mask) {
                let input = self.inputs[i];
                car::integrate(&mut self.cars[i], &input, car::H);
            }
        }

        for i in slots(mask) {
            self.resolve_walls(i);
        }

        // Fixed pair order keeps the result independent of iteration whims.
        // Both ends have to be live and at least one of them has to be moving.
        for i in slots(self.active) {
            for j in slots(self.active & !((2 << i) - 1)) {
                let a = mask & (1 << i) != 0;
                let b = mask & (1 << j) != 0;
                if a || b {
                    self.resolve_pair(i, j, a, b);
                }
            }
        }

        for i in slots(mask) {
            self.update_progress(i);
        }
    }

    /// Push a car back inside the barriers, once per body circle so that
    /// clipping a wall with the nose spins you the way it should.
    fn resolve_walls(&mut self, i: usize) {
        let mut car = self.cars[i];
        let hint = car.seg as u16;
        let fwd = V2::from_angle(car.heading);

        for k in 0..2 {
            let off = if k == 0 { CIRCLE_OFF } else { -CIRCLE_OFF };
            let cp = car.pos().add(fwd.scale(off));
            let hit = self.track.nearest(cp, hint);
            let limit = hit.half_width - CIRCLE_R;
            let d = hit.lat;
            if abs(d) <= limit {
                continue;
            }

            let pen = abs(d) - limit;
            // Inward normal: back toward the centerline.
            let n = hit.normal.scale(-signum(d));

            car.x += n.x * pen;
            car.y += n.y * pen;
            car.wall = 1.0;

            // Contact point relative to the CG, after the correction.
            let r = car.pos().add(fwd.scale(off)).sub(car.pos());
            // Velocity of the contact point, v + omega x r.
            let vpt = V2::new(
                car.vx - car.omega * r.y,
                car.vy + car.omega * r.x,
            );
            let vn = vpt.dot(n);
            if vn >= 0.0 {
                continue;
            }
            let rxn = r.cross(n);
            let denom = INV_MASS + rxn * rxn / IZ;
            let jn = -(1.0 + WALL_RESTITUTION) * vn / denom;

            // Scrub friction along the wall, capped by the normal impulse.
            let t = n.perp();
            let vt = vpt.dot(t);
            let rxt = r.cross(t);
            let denom_t = INV_MASS + rxt * rxt / IZ;
            let mut jt = -vt / denom_t * WALL_FRICTION;
            let cap = jn * 0.85;
            jt = clamp(jt, -cap, cap);

            let imp = n.scale(jn).add(t.scale(jt));
            car.vx += imp.x * INV_MASS;
            car.vy += imp.y * INV_MASS;
            car.omega += r.cross(imp) / IZ;
            car.impact += abs(jn) + abs(jt) * 0.5;
        }

        self.cars[i] = car;
    }

    /// Circle-vs-circle body contact. `a_dyn`/`b_dyn` say which cars may move;
    /// a static car acts as infinite mass.
    fn resolve_pair(&mut self, i: usize, j: usize, a_dyn: bool, b_dyn: bool) {
        // Cheap reject on the bounding radius of the whole body.
        let da = self.cars[i].pos().sub(self.cars[j].pos());
        let reach = CIRCLE_OFF + CIRCLE_R;
        if da.len_sq() > (2.0 * reach) * (2.0 * reach) {
            return;
        }

        let mut a = self.cars[i];
        let mut b = self.cars[j];
        let fa = V2::from_angle(a.heading);
        let fb = V2::from_angle(b.heading);

        for ka in 0..2 {
            for kb in 0..2 {
                let oa = if ka == 0 { CIRCLE_OFF } else { -CIRCLE_OFF };
                let ob = if kb == 0 { CIRCLE_OFF } else { -CIRCLE_OFF };
                let pa = a.pos().add(fa.scale(oa));
                let pb = b.pos().add(fb.scale(ob));
                let delta = pa.sub(pb);
                let dist = delta.len();
                let min_d = 2.0 * CIRCLE_R;
                if dist >= min_d || dist < 1e-4 {
                    continue;
                }

                let n = delta.scale(1.0 / dist);
                let pen = min_d - dist;

                // Positional correction, shared by whoever is allowed to move.
                let (wa, wb) = match (a_dyn, b_dyn) {
                    (true, true) => (0.5, 0.5),
                    (true, false) => (1.0, 0.0),
                    (false, true) => (0.0, 1.0),
                    (false, false) => continue,
                };
                a.x += n.x * pen * wa;
                a.y += n.y * pen * wa;
                b.x -= n.x * pen * wb;
                b.y -= n.y * pen * wb;

                let ra = pa.sub(a.pos());
                let rb = pb.sub(b.pos());
                let va = V2::new(a.vx - a.omega * ra.y, a.vy + a.omega * ra.x);
                let vb = V2::new(b.vx - b.omega * rb.y, b.vy + b.omega * rb.x);
                let rel = va.sub(vb).dot(n);
                if rel >= 0.0 {
                    continue;
                }

                let rxa = ra.cross(n);
                let rxb = rb.cross(n);
                let inv_a = if a_dyn { INV_MASS } else { 0.0 };
                let inv_b = if b_dyn { INV_MASS } else { 0.0 };
                let ia = if a_dyn { rxa * rxa / IZ } else { 0.0 };
                let ib = if b_dyn { rxb * rxb / IZ } else { 0.0 };
                let denom = inv_a + inv_b + ia + ib;
                if denom < 1e-6 {
                    continue;
                }
                let jn = -(1.0 + CAR_RESTITUTION) * rel / denom;
                let imp = n.scale(jn);

                if a_dyn {
                    a.vx += imp.x * INV_MASS;
                    a.vy += imp.y * INV_MASS;
                    a.omega += ra.cross(imp) / IZ;
                    a.impact += abs(jn);
                }
                if b_dyn {
                    b.vx -= imp.x * INV_MASS;
                    b.vy -= imp.y * INV_MASS;
                    b.omega -= rb.cross(imp) / IZ;
                    b.impact += abs(jn);
                }
            }
        }

        // Angular velocity can run away in a multi-contact pile-up.
        a.omega = clamp(a.omega, -6.0, 6.0);
        b.omega = clamp(b.omega, -6.0, 6.0);
        self.cars[i] = a;
        self.cars[j] = b;
    }

    /// Track position, checkpoint order and lap timing.
    fn update_progress(&mut self, i: usize) {
        let car = &mut self.cars[i];
        let prev_s = car.s;
        let hit = self.track.nearest(car.pos(), car.seg as u16);
        car.seg = hit.idx as f32;
        car.s = hit.s;
        car.lat = hit.lat;

        let len = self.track.length;
        let forward = {
            let d = hit.s - prev_s;
            if d < 0.0 {
                d + len
            } else {
                d
            }
        };
        // A big "forward" jump is really a backward move; ignore it.
        if forward > len * 0.5 {
            return;
        }

        let cp = car.cp as usize % CHECKPOINTS;
        let target = self.track.checkpoint_s(cp);
        let to_target = {
            let d = target - prev_s;
            if d < 0.0 {
                d + len
            } else {
                d
            }
        };
        if to_target > forward {
            return;
        }

        let next = (cp + 1) % CHECKPOINTS;
        car.cp = next as f32;
        if next == 1 {
            // Just crossed the start/finish line with every checkpoint hit.
            let secs = (self.tick as f32 - car.lap_start) / car::TICK_HZ as f32;
            if car.lap >= 1.0 {
                car.last_lap = secs;
                if car.best_lap <= 0.0 || secs < car.best_lap {
                    car.best_lap = secs;
                }
            }
            car.lap += 1.0;
            car.lap_start = self.tick as f32;
        }
    }

}

/// Total simulated distance, used by the tests as a coarse determinism hash.
pub fn checksum(cars: &[CarState]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for c in cars {
        for v in [c.x, c.y, c.heading, c.vx, c.vy, c.omega] {
            h ^= v.to_bits() as u64;
            h = h.wrapping_mul(0x100_0000_01b3);
        }
    }
    h
}

/// Guardrail against the layout assumption the wasm bridge relies on.
const _: () = {
    assert!(core::mem::size_of::<CarState>() == car::CAR_FLOATS * 4);
    assert!(core::mem::size_of::<CarInput>() == car::INPUT_FLOATS * 4);
    assert!(MASS > 0.0);
};
