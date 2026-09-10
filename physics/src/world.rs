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

use crate::car::{self, CarInput, CarState, BOUND_R, HALF_LEN, HALF_WID, INV_MASS, IZ, MASS};
use crate::collide::{self, Body, Constraint, Manifold, Obb};
use crate::damage::{self, Damage};
use crate::math::{abs, signum, V2};
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

/// How much each surface springs back, as a multiple of what bending sheet
/// metal gives back on its own.
///
/// The *speed* dependence belongs to the crush model and not to these -- see
/// [`crate::damage::restitution`], where a hard hit is nearly plastic because
/// the energy went into the shape of the car rather than back into its
/// velocity. All these two say is that a barrier keeps rather more of a hit
/// than another car's flank does.
const WALL_BOUNCE: f32 = 0.7;
const CAR_BOUNCE: f32 = 1.0;
/// Coulomb friction at a contact.
///
/// The barrier is steel and is meant to be slippery -- guiding a car back onto
/// the road rather than catching it is the entire reason it is shaped like
/// that -- but it is not frictionless, and what is left is enough to spin a car
/// that arrives at an angle. It came down from 0.55 when contacts started being
/// solved every substep: a barrier used to flick a car that leant on it away
/// once a tick, which is not a thing barriers do, and with that artifact gone
/// the friction that had been tuned around it pinned cars against the wall.
///
/// Panel on panel is the other way round. It used to be far slipperier than
/// this, which turned every side-by-side moment into a clean reflection; real
/// bodywork catches on the car it is rubbing, and that is what makes contact
/// scrub speed off both of them rather than exchange it.
const WALL_FRICTION: f32 = 0.40;
const CAR_FRICTION: f32 = 0.65;
/// Ceiling on the spin a contact may leave behind, rad/s. A car caught between
/// a barrier and another car can otherwise be handed a whole revolution.
const MAX_SPIN: f32 = 6.0;

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

    /// Drop a car back on the racing line where it is, pointing the right way,
    /// and straight: [`CarState::place`] repairs the bodywork.
    ///
    /// A deliberate call rather than a physical one. This is the button a
    /// driver presses when they are wedged somewhere facing a barrier, and
    /// making them serve the rest of the race in the car that got them there
    /// would turn one bad corner into a retirement. Completing a lap repairs a
    /// car too, in [`World::update_progress`] -- this is the same repair for
    /// somebody who is not going to reach the line.
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

        // Where the barrier is under each corner of each car. Sampled once --
        // see [`Barrier`] for why that is enough for a whole tick.
        let mut barrier = [[Barrier::default(); 4]; MAX_CARS];
        for i in slots(mask) {
            barrier[i] = self.sample_barriers(i);
        }

        // How fast the cars this process does *not* own are travelling, for the
        // length of this tick, so that being hit can change it.
        //
        // Their state is not ours to write and stays exactly as it was; this is
        // a scratch copy that lives for one tick and is thrown away. Without it
        // a client hits a rival that never reacts, and the same contact fires
        // again on every one of the eight substeps, each pass pulling the local
        // car further towards a velocity the rival is no longer travelling at.
        // With it the contact is over after the substep that resolved it, which
        // is what happens on the authority.
        //
        // Worth measuring rather than assuming: while the rival's pose is
        // current it is worth nothing at all, because a nearly plastic impact
        // leaves two equal cars at the same speed and the snapshot is already
        // showing it. It earns its keep when the pose is stale -- 10.7 m/s of
        // error against 13.0 at a three-tick-old snapshot and 30 m/s of closing
        // speed -- which is the condition this has to survive.
        let mut ghost = [(V2::ZERO, 0.0f32); MAX_CARS];
        for i in slots(self.active & !mask) {
            ghost[i] = (self.cars[i].vel(), self.cars[i].omega);
        }

        // Dynamics and contacts advance together.
        //
        // Contacts used to be resolved once, after all eight substeps had run.
        // That put the tires on a 480 Hz clock and the panels on a 60 Hz one,
        // and it showed: two cars closing at 30 m/s were already half a metre
        // into each other before anything was done about it, and half a metre
        // in, the shallowest separating axis is not reliably the one you drove
        // in along. Solving them at the same rate costs a broad-phase test per
        // pair per substep and bounds the deepest overlap by the distance a car
        // covers in one, which is centimetres.
        for _ in 0..car::SUBSTEPS {
            for i in slots(mask) {
                let input = self.inputs[i];
                car::integrate(&mut self.cars[i], &input, car::H);
            }

            for i in slots(mask) {
                self.resolve_walls(i, &barrier[i]);
            }

            // Fixed pair order keeps the result independent of iteration whims.
            // Both ends have to be live and at least one has to be moving.
            for i in slots(self.active) {
                for j in slots(self.active & !((2 << i) - 1)) {
                    let a = mask & (1 << i) != 0;
                    let b = mask & (1 << j) != 0;
                    if a || b {
                        self.resolve_pair(i, j, a, b, &mut ghost);
                    }
                }
            }
        }

        for i in slots(mask) {
            self.update_progress(i);
        }
    }

    /// Sample the barrier under each of a car's four corners, in the order
    /// [`Obb::corners`] returns them.
    fn sample_barriers(&self, i: usize) -> [Barrier; 4] {
        let car = self.cars[i];
        let body = Obb::new(car.pos(), car.heading, HALF_LEN, HALF_WID);
        let hint = car.seg as u16;
        let mut out = [Barrier::default(); 4];
        for (k, corner) in body.corners().iter().enumerate() {
            let hit = self.track.nearest(*corner, hint);
            // Outward: away from the centreline, on the side this corner is.
            let n = hit.normal.scale(signum(hit.lat));
            out[k] = Barrier {
                out: n,
                d: (abs(hit.lat) - hit.half_width) - corner.dot(n),
            };
        }
        out
    }

    /// Push a car back inside the barriers.
    ///
    /// Each of the four corners is tested against the plane [`sample_barriers`]
    /// found under it. A car broadside into a wall reports two corners and gets
    /// a flat, stable contact; one that clips it with a front corner reports
    /// one, and spins.
    fn resolve_walls(&mut self, i: usize, bar: &[Barrier; 4]) {
        let car = self.cars[i];
        let body = Obb::new(car.pos(), car.heading, HALF_LEN, HALF_WID);

        let mut m = Manifold::default();
        for (k, corner) in body.corners().iter().enumerate() {
            let pen = bar[k].depth(*corner);
            if pen <= 0.0 {
                continue;
            }
            // Inward normal: back towards the centerline.
            m.push(*corner, bar[k].out.scale(-1.0), pen);
        }
        if m.count == 0 {
            return;
        }

        let mut a = Body {
            pos: car.pos(),
            vel: car.vel(),
            omega: car.omega,
            inv_m: INV_MASS,
            inv_i: 1.0 / IZ,
        };
        let mut wall = Body::fixed(car.pos());
        collide::prepare(&a, &wall, m.as_slice_mut(), WALL_BOUNCE);
        let hit = collide::solve(&mut a, &mut wall, m.as_slice_mut(), WALL_FRICTION);
        collide::separate(&mut a, &mut wall, m.as_slice());

        let car = &mut self.cars[i];
        car.x = a.pos.x;
        car.y = a.pos.y;
        car.vx = a.vel.x;
        car.vy = a.vel.y;
        car.omega = collide::clamp_spin(a.omega, MAX_SPIN);
        car.wall = 1.0;
        car.impact += hit.severity();
        crush(car, m.as_slice(), &hit, damage::SHARE_WALL, 1.0);
    }

    /// Body-vs-body contact between two cars. `a_dyn`/`b_dyn` say which of them
    /// this process owns; the other one's *position* is not ours to move, and
    /// its velocity is borrowed from `ghost` for the length of the tick.
    fn resolve_pair(
        &mut self,
        i: usize,
        j: usize,
        a_dyn: bool,
        b_dyn: bool,
        ghost: &mut [(V2, f32); MAX_CARS],
    ) {
        // Cheap reject on the bounding radius of the whole body, before the
        // separating-axis test earns its keep.
        let apart = self.cars[i].pos().sub(self.cars[j].pos());
        if apart.len_sq() > (2.0 * BOUND_R) * (2.0 * BOUND_R) {
            return;
        }

        let ca = self.cars[i];
        let cb = self.cars[j];
        let oa = Obb::new(ca.pos(), ca.heading, HALF_LEN, HALF_WID);
        let ob = Obb::new(cb.pos(), cb.heading, HALF_LEN, HALF_WID);
        let mut m = match collide::box_box(&oa, &ob) {
            Some(m) => m,
            None => return,
        };

        // Both cars weigh what they weigh, whether or not this process owns
        // them.
        //
        // A car this process may not *move* is not a car that weighs nothing,
        // and the two are easy to confuse. The browser simulates only the local
        // car, so it used to hand every rival infinite mass -- and an infinite
        // mass returns the whole impulse, so you rebounded off a car you should
        // have shoved out of the way. Giving the other car its real mass and
        // then discarding its half of the answer costs nothing, because the
        // authority's answer for that car is already in flight, and it makes
        // the impulse the client predicts *for itself* the one the sidecar
        // computed rather than one that happens to land nearby.
        //
        // How much that is worth depends entirely on how fresh the rival's pose
        // is, and it is worth saying so: with it predicted onto the current
        // tick the two treatments agree to a few hundredths, because a plastic
        // impact leaves two equal cars at the same speed either way. Against a
        // rival six ticks old at 20 m/s of closing speed, real mass is 9.4 m/s
        // and 3.97 m out where infinite mass is 10.2 m/s and 4.13 m.
        let (va, wa) = if a_dyn { (ca.vel(), ca.omega) } else { ghost[i] };
        let (vb, wb) = if b_dyn { (cb.vel(), cb.omega) } else { ghost[j] };
        let mut a = Body { pos: ca.pos(), vel: va, omega: wa, inv_m: INV_MASS, inv_i: 1.0 / IZ };
        let mut b = Body { pos: cb.pos(), vel: vb, omega: wb, inv_m: INV_MASS, inv_i: 1.0 / IZ };

        collide::prepare(&a, &b, m.as_slice_mut(), CAR_BOUNCE);
        let hit = collide::solve(&mut a, &mut b, m.as_slice_mut(), CAR_FRICTION);

        // Pushing overlapping bodies apart, on the other hand, is a numerical
        // repair and not a force -- so it may only move a car this process owns,
        // and a client that owns one of the two therefore takes all of it.
        a.inv_m = if a_dyn { INV_MASS } else { 0.0 };
        b.inv_m = if b_dyn { INV_MASS } else { 0.0 };
        collide::separate(&mut a, &mut b, m.as_slice());

        if a_dyn {
            let car = &mut self.cars[i];
            car.x = a.pos.x;
            car.y = a.pos.y;
            car.vx = a.vel.x;
            car.vy = a.vel.y;
            car.omega = collide::clamp_spin(a.omega, MAX_SPIN);
            car.impact += hit.severity();
            crush(car, m.as_slice(), &hit, damage::SHARE_CAR, 1.0);
        }
        if b_dyn {
            let car = &mut self.cars[j];
            car.x = b.pos.x;
            car.y = b.pos.y;
            car.vx = b.vel.x;
            car.vy = b.vel.y;
            car.omega = collide::clamp_spin(b.omega, MAX_SPIN);
            car.impact += hit.severity();
            crush(car, m.as_slice(), &hit, damage::SHARE_CAR, -1.0);
        }
        // What the hit did to a car we do not own, remembered until the end of
        // the tick and no longer. The next snapshot is the truth about it.
        if !a_dyn {
            ghost[i] = (a.vel, collide::clamp_spin(a.omega, MAX_SPIN));
        }
        if !b_dyn {
            ghost[j] = (b.vel, collide::clamp_spin(b.omega, MAX_SPIN));
        }
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
            // And a fresh car to start it in.
            //
            // The start/finish straight is where a pit lane would be, and this
            // circuit does not have one, so completing a lap is the stop you
            // never had to make. Without it damage is a one-way ratchet: the
            // *Respawn* button repairs a car, but a bot has no thumbs and a
            // driver who has not found the button spends the rest of the race
            // in whatever they made of the first corner. A lap is the right
            // clock for it -- long enough that a shunt is something you have to
            // drive around for the best part of a minute, short enough that
            // nobody is stuck with one forever.
            car.set_damage(&Damage::default());
        }
    }

}

/// The barrier under one corner of a car, as a plane.
///
/// The track edge is a moving wall -- it is wherever the centreline says it is
/// -- so there is no polygon to intersect, only a query. That query walks 49
/// centreline samples per corner and is the most expensive thing a tick does,
/// which is why it happens once a tick and the contact it feeds happens eight
/// times.
///
/// Sampling it that rarely is not an approximation of much. The barrier is
/// piecewise straight, a car covers about a metre in a whole tick at racing
/// speed, and over that metre the plane a corner is measured against is the
/// same plane. What moves inside the tick is the car, and the car is measured
/// against it every substep.
#[derive(Clone, Copy, Default)]
struct Barrier {
    /// Outward unit normal: away from the centreline, into the wall.
    out: V2,
    /// Plane offset, so `p . out + d` is how far `p` is past the barrier.
    d: f32,
}

impl Barrier {
    #[inline]
    fn depth(&self, p: V2) -> f32 {
        p.dot(self.out) + self.d
    }
}

/// Turn the energy a contact destroyed into residual crush on the faces of the
/// car that absorbed it.
///
/// `push` is +1 for the car the manifold's normals point towards and -1 for the
/// other one, which is the direction each was shoved. The face that took the
/// hit is the one the shove came through, so a contact square on the nose
/// crushes only the nose, and one that came in at an angle splits its energy
/// between two faces by the *squares* of the normal's components -- which sum
/// to one, so a corner impact invents nothing and loses nothing.
///
/// The two kinds of energy the contact reports travel the same route and land
/// on the same faces; what differs is how far they are allowed to fold them.
fn crush(car: &mut CarState, cs: &[Constraint], hit: &collide::Impact, share: f32, push: f32) {
    let folding = hit.crush * share;
    let sliding = hit.scrape * share;
    if folding <= 0.0 && sliding <= 0.0 {
        return;
    }
    let mut total = 0.0;
    for c in cs {
        total += c.impulse();
    }
    if total <= 1e-3 {
        return;
    }
    let mut d = car.damage();
    for c in cs {
        let part = c.impulse() / total;
        if part <= 0.0 {
            continue;
        }
        // Which way this car was pushed, in its own frame.
        let n = c.normal.scale(push).to_local(car.heading);
        let (wx, wy) = (part * n.x * n.x, part * n.y * n.y);
        if n.x < 0.0 {
            d.front = damage::scuff(damage::accumulate(d.front, folding * wx), sliding * wx);
        } else {
            d.rear = damage::scuff(damage::accumulate(d.rear, folding * wx), sliding * wx);
        }
        if n.y < 0.0 {
            d.left = damage::scuff(damage::accumulate(d.left, folding * wy), sliding * wy);
        } else {
            d.right = damage::scuff(damage::accumulate(d.right, folding * wy), sliding * wy);
        }
    }
    car.set_damage(&d);
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
