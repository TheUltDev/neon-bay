//! The car: four wheels, a body, and the loop that ties them together.
//!
//! This file owns no physics of its own. It holds the chassis geometry, and
//! then each substep it does the same five things in the same order:
//!
//! 1. resolve the steering into a road-wheel angle per side ([`ackermann`]);
//! 2. ask [`crate::aero`] what the air is doing and [`crate::suspension`] what
//!    that plus the body's attitude means for the load on each tire;
//! 3. work out where each contact patch is going, hand it to [`crate::wheel`],
//!    and get a force back from [`crate::tire`];
//! 4. put the drivetrain's torque into the rear wheels ([`crate::drivetrain`]);
//! 5. sum the four forces and moments and integrate the body.
//!
//! What comes out is not a car that has been *told* how to behave. Understeer,
//! power oversteer, lift-off oversteer, lock-up, wheelspin, engine braking and
//! the way all of them change with speed are consequences of those five steps,
//! not cases in a list. The one place the model deliberately stops being a
//! simulation is [`steer_lock`], which is an input aid for people driving with
//! a keyboard rather than a steering wheel.
//!
//! Everything is plain `f32` and every transcendental comes from
//! [`crate::math`], so the sidecar (x86-64) and the browser (wasm32) produce
//! identical bits.

use crate::aero;
use crate::damage::Damage;
use crate::drivetrain::{self, Drivetrain};
use crate::math::{abs, atan, clamp, max, min, tan, V2};
use crate::suspension::{self, Attitude};
use crate::tire;
use crate::wheel::{self, Contact, Wheel};

/// Simulation rate. Both the sidecar and the client advance at exactly this.
pub const TICK_HZ: u32 = 60;
pub const DT: f32 = 1.0 / 60.0;

/// Dynamics substeps per tick.
///
/// Eight, where the old lumped model needed two. A wheel carrying real angular
/// momentum against a tire making tens of kilonewtons per unit of slip is a
/// genuinely stiff system; the implicit steps in [`crate::wheel`] and
/// [`crate::drivetrain`] handle the stability, and this is what is left for
/// *accuracy* -- enough resolution that a lock-up, a shift and a kerb strike
/// all land on the right side of the same millisecond.
pub const SUBSTEPS: u32 = 8;
pub const H: f32 = DT / SUBSTEPS as f32;

pub const G: f32 = 9.81;

// --- chassis ------------------------------------------------------------
pub const MASS: f32 = 1150.0;
pub const INV_MASS: f32 = 1.0 / MASS;
/// Yaw inertia. About `m * (1.2 m)^2`, which is where a car this size lands.
pub const IZ: f32 = 1600.0;
/// CG to front axle / rear axle. Slightly rear-biased, as a mid-engined car is.
pub const LF: f32 = 1.25;
pub const LR: f32 = 1.45;
pub const WHEELBASE: f32 = LF + LR;
/// Track width. The lever the lateral load transfer in [`crate::suspension`]
/// works across, so a wider car transfers less of its weight for the same
/// cornering force.
pub const TRACK_F: f32 = 1.60;
pub const TRACK_R: f32 = 1.58;
pub const CG_HEIGHT: f32 = 0.52;

/// Body half extents, for rendering and collision.
pub const HALF_LEN: f32 = 2.10;
pub const HALF_WID: f32 = 0.95;
/// Bounding circles along the body axis, still exported to the client for its
/// own cheap proximity tests. The simulation's narrow phase is a proper
/// oriented box -- see [`crate::collide`].
pub const CIRCLE_OFF: f32 = 1.06;
pub const CIRCLE_R: f32 = 1.04;
/// Radius of the circle that contains the whole body, for the broad phase.
/// Rounded up from `hypot(HALF_LEN, HALF_WID)`; `bound_radius_contains_the_body`
/// checks that it still does. Over-estimating here only costs a wasted SAT
/// call, but under-estimating drops real corner-to-corner contacts.
pub const BOUND_R: f32 = 2.31;

/// Maximum road-wheel angle at the steering rack's limit.
pub const MAX_STEER: f32 = 0.58;
/// How fast the road wheels can be moved, rad/s. A hurried input, not a lazy
/// one -- this is the rack, not the driver.
const STEER_RATE: f32 = 5.0;
/// How much of full Ackermann the steering geometry has. Real racks are
/// partial, so the inner wheel takes some extra angle but not all of it.
const ACKERMANN: f32 = 0.65;

/// Grip left at the rear once the handbrake is pulled. Almost all of the effect
/// is the tire model reacting to two locked wheels; this is the mechanical
/// part, the rear brake being a cable that does not modulate.
const HANDBRAKE_GRIP: f32 = 0.88;

/// Below this speed a coasting car is simply stopped. The tire model has no
/// opinion at zero slip velocity and would otherwise let it drift forever.
const SETTLE_SPEED: f32 = 0.15;

/// Wheel order, everywhere: front-left, front-right, rear-left, rear-right.
pub const FL: usize = 0;
pub const FR: usize = 1;
pub const RL: usize = 2;
pub const RR: usize = 3;

/// One car, laid out for zero-copy sharing with JavaScript.
///
/// The whole struct is `f32` -- including small integers like the lap counter --
/// so the browser can map it with a single `Float32Array` over wasm memory and
/// the sidecar can memcpy it straight into a network snapshot. Values below
/// 2^24 are exact in `f32`, which covers every counter here.
///
/// Nearly all of it is *simulation state* rather than pose: four wheel speeds,
/// four relaxed tire forces, the body's roll and pitch, and the drivetrain.
/// Rollback has to restore every one of them or a replay diverges, which is why
/// they are on the wire in `module/src/lib.rs` and not just in memory here.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CarState {
    pub x: f32,
    pub y: f32,
    pub heading: f32,
    pub vx: f32,
    pub vy: f32,
    pub omega: f32,
    /// Nominal road-wheel angle, radians, before Ackermann splits it per side.
    pub steer: f32,

    /// Wheel angular velocities, rad/s, in [`FL`]..[`RR`] order.
    pub w_fl: f32,
    pub w_fr: f32,
    pub w_rl: f32,
    pub w_rr: f32,

    /// Lateral force each tire has actually built up, newtons. Carried because
    /// a carcass takes a relaxation length to develop it -- see
    /// [`crate::wheel`].
    pub fy_fl: f32,
    pub fy_fr: f32,
    pub fy_rl: f32,
    pub fy_rr: f32,

    /// Body attitude. Positive roll leans right, positive pitch is nose-down.
    pub roll: f32,
    pub roll_rate: f32,
    pub pitch: f32,
    pub pitch_rate: f32,

    /// Engine speed, rad/s.
    pub engine: f32,
    /// -1 reverse, 1..=6 forward.
    pub gear: f32,
    /// Seconds left of the current shift.
    pub shift: f32,
    /// Clutch engagement, 0..1.
    pub clutch: f32,

    /// Body-frame acceleration. Fed back into the geometric and unsprung paths
    /// of load transfer, which are instantaneous and so cannot wait for the
    /// body to roll -- and read by the HUD as a g-meter.
    pub ax: f32,
    pub ay: f32,

    /// Mean slip angle per axle, radians. Telemetry: their difference is the
    /// difference between understeer and oversteer.
    pub slip_f: f32,
    pub slip_r: f32,
    /// 0..1 traction loss -- drives tire smoke and marks.
    pub wheel_spin: f32,
    /// 0..1 normalized engine speed.
    pub rpm: f32,

    /// Distance along the lap, meters.
    pub s: f32,
    /// Signed offset from the centerline, meters.
    pub lat: f32,
    /// Cached nearest-centerline index (search hint).
    pub seg: f32,
    pub lap: f32,
    /// Next checkpoint index that must be crossed.
    pub cp: f32,
    pub lap_start: f32,
    pub last_lap: f32,
    pub best_lap: f32,

    /// Collision impulse magnitude accumulated this tick.
    pub impact: f32,
    /// 1 while scraping a wall.
    pub wall: f32,

    /// Residual crush on each face of the body, metres. See [`crate::damage`].
    ///
    /// Simulation state, not decoration: it decides how much downforce the car
    /// still makes, how much lock the rack still has, how much air the engine
    /// still gets and how much grip each corner is left with. A rollback that
    /// dropped it would replay an undamaged car and diverge immediately -- and
    /// it is also, directly, the shape the renderer draws.
    pub dmg_front: f32,
    pub dmg_rear: f32,
    pub dmg_left: f32,
    pub dmg_right: f32,

    pub active: f32,
}

/// Number of `f32`s in [`CarState`]. Asserted against the real layout in tests.
pub const CAR_FLOATS: usize = 44;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CarInput {
    /// -1..1. Negative is reverse.
    pub throttle: f32,
    /// -1..1. Positive steers left.
    pub steer: f32,
    /// 0..1.
    pub brake: f32,
    /// 0 or 1.
    pub handbrake: f32,
}

pub const INPUT_FLOATS: usize = 4;

impl CarInput {
    #[inline]
    pub fn sanitize(self) -> CarInput {
        CarInput {
            throttle: clamp(nan_to_zero(self.throttle), -1.0, 1.0),
            steer: clamp(nan_to_zero(self.steer), -1.0, 1.0),
            brake: clamp(nan_to_zero(self.brake), 0.0, 1.0),
            handbrake: if self.handbrake > 0.5 { 1.0 } else { 0.0 },
        }
    }
}

#[inline]
fn nan_to_zero(v: f32) -> f32 {
    if v.is_nan() {
        0.0
    } else {
        v
    }
}

impl CarState {
    /// The record as the flat block of `f32` it is laid out as -- the same view
    /// the browser takes over wasm memory, and what the fingerprint hashes.
    #[inline]
    pub fn as_floats(&self) -> &[f32; CAR_FLOATS] {
        // SAFETY: `#[repr(C)]` with `CAR_FLOATS` fields, every one an `f32`.
        // `layout_matches_the_wasm_bridge_contract` asserts exactly that, and
        // the client would be reading garbage if it ever stopped holding.
        unsafe { &*(self as *const CarState as *const [f32; CAR_FLOATS]) }
    }

    #[inline]
    pub fn pos(&self) -> V2 {
        V2::new(self.x, self.y)
    }

    #[inline]
    pub fn vel(&self) -> V2 {
        V2::new(self.vx, self.vy)
    }

    #[inline]
    pub fn speed(&self) -> f32 {
        self.vel().len()
    }

    /// Forward speed, signed. Negative while reversing.
    #[inline]
    pub fn forward_speed(&self) -> f32 {
        self.vel().to_local(self.heading).x
    }

    /// Lateral acceleration in g, for the telemetry readout.
    #[inline]
    pub fn lateral_g(&self) -> f32 {
        self.ay / G
    }

    /// The four wheels, in [`FL`]..[`RR`] order.
    #[inline]
    fn wheels(&self) -> [Wheel; 4] {
        [
            Wheel { omega: self.w_fl, fy: self.fy_fl },
            Wheel { omega: self.w_fr, fy: self.fy_fr },
            Wheel { omega: self.w_rl, fy: self.fy_rl },
            Wheel { omega: self.w_rr, fy: self.fy_rr },
        ]
    }

    #[inline]
    fn set_wheels(&mut self, w: &[Wheel; 4]) {
        self.w_fl = w[FL].omega;
        self.w_fr = w[FR].omega;
        self.w_rl = w[RL].omega;
        self.w_rr = w[RR].omega;
        self.fy_fl = w[FL].fy;
        self.fy_fr = w[FR].fy;
        self.fy_rl = w[RL].fy;
        self.fy_rr = w[RR].fy;
    }

    /// The crush on each face, as the damage model sees it.
    #[inline]
    pub fn damage(&self) -> Damage {
        Damage {
            front: self.dmg_front,
            rear: self.dmg_rear,
            left: self.dmg_left,
            right: self.dmg_right,
        }
    }

    #[inline]
    pub fn set_damage(&mut self, d: &Damage) {
        self.dmg_front = d.front;
        self.dmg_rear = d.rear;
        self.dmg_left = d.left;
        self.dmg_right = d.right;
    }

    #[inline]
    fn attitude(&self) -> Attitude {
        Attitude {
            roll: self.roll,
            roll_rate: self.roll_rate,
            pitch: self.pitch,
            pitch_rate: self.pitch_rate,
        }
    }

    #[inline]
    fn set_attitude(&mut self, a: &Attitude) {
        self.roll = a.roll;
        self.roll_rate = a.roll_rate;
        self.pitch = a.pitch;
        self.pitch_rate = a.pitch_rate;
    }

    #[inline]
    fn drivetrain(&self) -> Drivetrain {
        Drivetrain {
            engine: self.engine,
            gear: self.gear,
            shift: self.shift,
            clutch: self.clutch,
        }
    }

    #[inline]
    fn set_drivetrain(&mut self, d: &Drivetrain) {
        self.engine = d.engine;
        self.gear = d.gear;
        self.shift = d.shift;
        self.clutch = d.clutch;
    }

    /// Place the car, stopped, at a pose. Also invalidates the track hint,
    /// puts the drivetrain back at idle in first, and straightens the panels:
    /// this is where a car comes from, and a car comes from a factory.
    pub fn place(&mut self, pos: V2, heading: f32) {
        let idle = Drivetrain::default();
        *self = CarState {
            x: pos.x,
            y: pos.y,
            heading,
            seg: crate::track::NO_HINT as f32,
            engine: idle.engine,
            gear: idle.gear,
            clutch: idle.clutch,
            rpm: idle.engine / (drivetrain::REDLINE_RPM * drivetrain::RPM_TO_RAD),
            lap: self.lap,
            cp: self.cp,
            lap_start: self.lap_start,
            last_lap: self.last_lap,
            best_lap: self.best_lap,
            active: self.active,
            ..Default::default()
        };
    }

    /// Make the drivetrain consistent with the body's current speed: wheels
    /// rolling rather than locked, a gear that suits the speed, and an engine
    /// turning at what that gear implies.
    ///
    /// For a car given a velocity directly rather than driven up to it. Every
    /// part of this matters -- wheels left at zero are four locked tires, and a
    /// gear left at first is an engine being driven to five figures. Neither is
    /// what a caller writing `vx = 30.0` is asking for.
    pub fn sync_drivetrain(&mut self) {
        let u = self.forward_speed();
        self.w_fl = u / wheel::RADIUS;
        self.w_fr = self.w_fl;
        self.w_rl = self.w_fl;
        self.w_rr = self.w_fl;
        let (gear, engine) = drivetrain::gear_for(u);
        self.gear = gear;
        self.engine = engine;
        self.shift = 0.0;
        self.clutch = 1.0;
        self.rpm = engine / (drivetrain::REDLINE_RPM * drivetrain::RPM_TO_RAD);
    }
}

/// Road-wheel angle a full-lock input produces at a given speed.
///
/// The one input aid in the model. A real rack has a fixed ratio, but a real
/// driver also has a wheel with 900 degrees of travel and both hands on it; a
/// keyboard has a key that is down or up. Tapering the lock with speed is what
/// stands in for the fine control that is missing, and the bot driver reads the
/// same curve so it steers the same car.
///
/// The taper is far steeper than it was under the old linear tires, and it has
/// to be. A real tire makes peak force at six to nine degrees of slip and less
/// beyond that, so at 30 m/s the front axle wants about four degrees of steer
/// to pull maximum lateral g -- and anything past that is not more cornering,
/// it is understeer. Handing a keyboard seventeen degrees there would mean the
/// useful part of the input range was the first quarter of it.
#[inline]
pub fn steer_lock(speed: f32) -> f32 {
    MAX_STEER / (1.0 + speed * 0.105)
}

/// Split a nominal steering angle into left and right road-wheel angles.
///
/// Both wheels are on the same steering rack but describe different circles, so
/// the inner one has to take more angle or it drags. Real geometry gets part of
/// the way there; [`ACKERMANN`] says how much.
#[inline]
pub fn ackermann(delta: f32) -> (f32, f32) {
    if abs(delta) < 1e-4 {
        return (delta, delta);
    }
    let t = tan(delta);
    let k = TRACK_F / (2.0 * WHEELBASE) * t;
    // A left turn has positive `t`, which makes the left denominator smaller
    // and the left wheel the inner one. A right turn flips both by itself.
    let full_l = atan(t / (1.0 - k));
    let full_r = atan(t / (1.0 + k));
    (
        delta + (full_l - delta) * ACKERMANN,
        delta + (full_r - delta) * ACKERMANN,
    )
}

/// Body-frame position of each wheel centre, `+x` forward and `+y` left.
#[inline]
fn wheel_pos(i: usize) -> V2 {
    match i {
        FL => V2::new(LF, TRACK_F * 0.5),
        FR => V2::new(LF, -TRACK_F * 0.5),
        RL => V2::new(-LR, TRACK_R * 0.5),
        _ => V2::new(-LR, -TRACK_R * 0.5),
    }
}

/// Advance one car by `h` seconds of dynamics. Collisions and lap bookkeeping
/// are handled by [`crate::world::World::step`].
pub fn integrate(car: &mut CarState, input: &CarInput, h: f32) {
    let inp = input.sanitize();
    let speed = car.speed();
    // Everything below asks the wreckage what it is still allowed to do. On an
    // undamaged car every one of these multipliers is exactly 1, so the model
    // is the model and damage is a set of coefficients on it -- not a fork.
    let dmg = car.damage();

    // --- steering ---------------------------------------------------------
    // A bent rack has less lock, and bent geometry has an opinion of its own
    // about where straight ahead is. The pull is added to the target rather
    // than to the output, so the driver has to hold against it exactly as long
    // as they want to go straight.
    let lock = steer_lock(speed) * dmg.steer_lock();
    let target = clamp(inp.steer * lock + dmg.steer_pull(), -MAX_STEER, MAX_STEER);
    let max_delta = STEER_RATE * h;
    car.steer += clamp(target - car.steer, -max_delta, max_delta);
    let (steer_l, steer_r) = ackermann(car.steer);
    let steer = [steer_l, steer_r, 0.0, 0.0];

    // --- aero and vertical loads ------------------------------------------
    let air = aero::evaluate(speed).crushed(&dmg);
    let mut att = car.attitude();
    let load = suspension::loads(&att, car.ax, car.ay, air.down_f, air.down_r);
    let fz = [load.fl, load.fr, load.rl, load.rr];

    // --- where each contact patch is going --------------------------------
    let v_body = car.vel().to_local(car.heading);
    let (u, w) = (v_body.x, v_body.y);

    let mut wheels = car.wheels();
    let mut contact = [Contact::default(); 4];
    let mut kappa = [0.0f32; 4];
    for i in 0..4 {
        let r = wheel_pos(i);
        // Contact point velocity: v + omega x r, then into the wheel's frame.
        let vc = V2::new(u - car.omega * r.y, w + car.omega * r.x).to_local(steer[i]);
        let hand = if i >= RL && inp.handbrake > 0.5 {
            HANDBRAKE_GRIP
        } else {
            1.0
        };
        // A corner that has been folded is rubbing its own bodywork and is no
        // longer pointing where the other three are. Wheels 0 and 2 are the
        // left-hand pair, 0 and 1 the front one.
        let end = if i <= FR { dmg.front } else { dmg.rear };
        let side = if i % 2 == 0 { dmg.left } else { dmg.right };
        let grip = hand * dmg.wheel_grip(end, side);
        contact[i] = Contact { u: vc.x, v: vc.y, fz: fz[i], grip };
        // Slip ratio is known before any force is: it needs only the wheel's
        // speed and the road's. Both assists read it here, ahead of the tires,
        // against the same floor the tires will use.
        let u_ref = max(abs(vc.x), wheel::V_MIN);
        kappa[i] = (wheels[i].omega * wheel::RADIUS - vc.x) / u_ref;
    }

    // --- drivetrain --------------------------------------------------------
    // Traction control watches the driven wheels' slip in whichever direction
    // the gearbox is trying to move the car. The direction matters: reverse is
    // a ratio of nearly fifteen to one, so it lights the rear tires up more
    // readily than first does, and a slip ratio that means wheelspin going
    // forwards is signed the other way going backwards. Left unsigned, a car
    // backing out of a barrier sits on the rev limiter going nowhere.
    let dir = if car.gear < 0.0 { -1.0 } else { 1.0 };
    let pedal = inp.throttle * dir;
    let spin = max(kappa[RL] * dir, kappa[RR] * dir);
    let throttle = if pedal > 0.0 {
        // Back to the wire's convention on the way out, where a negative
        // throttle is the request for reverse rather than a negative torque.
        wheel::traction_control(pedal, spin) * dir
    } else {
        inp.throttle
    };
    let mut dt = car.drivetrain();
    // Damage reaches the engine as less air, which is what a radiator wearing
    // its own condenser actually does to one -- so it scales the pedal rather
    // than the torque curve, and idle stays idle.
    let drive = dt.step(throttle * dmg.power(), wheels[RL].omega, wheels[RR].omega, u, h);
    let torque = [0.0, 0.0, drive.torque_l, drive.torque_r];
    let coupling = [0.0, 0.0, drive.coupling, drive.coupling];

    // --- brakes ------------------------------------------------------------
    // ABS on the pedal; the handbrake is a cable to the rear calipers and gets
    // no help at all, which is exactly why it is useful for putting the car
    // sideways. The pedal itself may not be the driver's -- see
    // [`reverse_assist`].
    let pressure = reverse_assist(inp.throttle, inp.brake, u);
    let hand = inp.handbrake * wheel::BRAKE_HAND;
    let brake = [
        wheel::anti_lock(pressure * wheel::BRAKE_F, kappa[FL]),
        wheel::anti_lock(pressure * wheel::BRAKE_F, kappa[FR]),
        wheel::anti_lock(pressure * wheel::BRAKE_R, kappa[RL]) + hand,
        wheel::anti_lock(pressure * wheel::BRAKE_R, kappa[RR]) + hand,
    ];

    // --- tire forces, summed onto the body ---------------------------------
    let mut fx_body = 0.0;
    let mut fy_body = 0.0;
    let mut mz = 0.0;
    let mut slip = [0.0f32; 4];
    let mut sat = [0.0f32; 4];
    for i in 0..4 {
        let out = wheels[i].step(&contact[i], torque[i], brake[i], coupling[i], h);
        // Back out of the wheel's frame and onto the chassis.
        let f = V2::new(out.fx, out.fy).to_world(steer[i]);
        let r = wheel_pos(i);
        fx_body += f.x;
        fy_body += f.y;
        mz += r.cross(f) + out.mz;
        slip[i] = out.alpha;
        sat[i] = out.saturation;
    }

    // Drag opposes travel. Applied to the body rather than the patches because
    // it acts on the shell, not the rubber.
    if speed > 1e-3 {
        let k = air.drag / speed;
        fx_body -= u * k;
        fy_body -= w * k;
    }

    // --- integrate ---------------------------------------------------------
    let acc_u = fx_body * INV_MASS;
    let acc_w = fy_body * INV_MASS;
    car.ax = acc_u;
    car.ay = acc_w;

    // The body rolls and pitches under what the tires just did, so the next
    // substep's loads already know about it.
    att.integrate(acc_u, acc_w, h);
    car.set_attitude(&att);

    car.omega += mz / IZ * h;

    // No `omega x v` transport terms here: velocity is *stored* in world space
    // and re-projected into the body frame every substep, so the rotation of the
    // frame is already accounted for. Adding them as well double-counts it and
    // quietly cancels half of the cornering force.
    let world_v = V2::new(u + acc_u * h, w + acc_w * h).to_world(car.heading);
    car.vx = world_v.x;
    car.vy = world_v.y;
    car.x += car.vx * h;
    car.y += car.vy * h;
    car.heading = crate::math::wrap_pi(car.heading + car.omega * h);

    car.set_wheels(&wheels);
    car.set_drivetrain(&dt);

    // Settle: a coasting car below walking pace has no slip for the tire model
    // to work with, so stop it rather than let it drift.
    if car.speed() < SETTLE_SPEED && inp.throttle.abs() < 0.02 && pressure < 0.02 {
        car.vx = 0.0;
        car.vy = 0.0;
        car.omega *= 0.5;
        car.w_fl = 0.0;
        car.w_fr = 0.0;
        car.w_rl = 0.0;
        car.w_rr = 0.0;
    }

    // --- telemetry ---------------------------------------------------------
    car.slip_f = (slip[FL] + slip[FR]) * 0.5;
    car.slip_r = (slip[RL] + slip[RR]) * 0.5;
    let rear = max(sat[RL], sat[RR]);
    let front = max(sat[FL], sat[FR]);
    car.wheel_spin = clamp((max(rear, front * 0.8) - 0.9) * 1.2, 0.0, 1.0);
    car.rpm = drive.rpm;
}

/// How quickly the reverse assist winds the brakes on, per m/s of travel in
/// the wrong direction. Full pressure two metres a second above the point the
/// gearbox will change direction at.
const REVERSE_BRAKE: f32 = 0.5;

/// Brake pressure the car applies for you when you ask it to go the way it is
/// not currently going.
///
/// The second input aid, and the same kind of thing as [`steer_lock`]. A
/// keyboard has one key for a direction, where a car has a brake pedal to hold
/// with your left foot and a lever for your right hand, and a gearbox that will
/// not select reverse at speed -- correctly, since the ratio is short enough
/// that doing so would put the engine past the limiter backwards. Between those
/// facts sits a car that will not do what it is told: hold reverse while still
/// rolling forwards and nothing happens at all, because a negative pedal is a
/// request for a direction and not a negative torque. The car coasts on rolling
/// resistance alone for as long as that takes, and nothing about it suggests
/// that what was wanted was the brake.
///
/// So asking for a direction the car is not going brakes it until it is not
/// going anywhere, and then the gearbox does the rest. The mirror case is the
/// same case: throttle while rolling backwards brakes too.
///
/// This is physics and not a key mapping, which matters twice over. The bots
/// need it -- one nose-first into a barrier was doing exactly the above, asking
/// for reverse at walking pace and coasting at it for the rest of the race --
/// and an aid that lives in the browser is an aid the authority has to be
/// trusted to agree with.
pub fn reverse_assist(throttle: f32, brake: f32, forward: f32) -> f32 {
    // How fast the car is travelling the wrong way for the pedal that is down.
    let wrong = if throttle < -0.02 {
        forward
    } else if throttle > 0.02 {
        -forward
    } else {
        return brake;
    };
    if wrong <= drivetrain::REVERSE_BELOW {
        return brake;
    }
    max(brake, min((wrong - drivetrain::REVERSE_BELOW) * REVERSE_BRAKE, 1.0))
}

/// Peak lateral force the whole car can make at a given speed, newtons.
///
/// Not used by the simulation. The bot driver needs to know how fast it can go
/// through a corner, and asking the tire model is a great deal more honest than
/// the fixed grip number it used to assume -- this one knows about downforce,
/// and about whether the car is still in one piece. A driver who has just lost
/// a corner of their car and goes on planning for the grip it used to have
/// crashes again immediately, and then again, which is a spiral rather than a
/// consequence.
pub fn grip_limit(speed: f32, dmg: &Damage) -> f32 {
    let air = aero::evaluate(speed).crushed(dmg);
    let total = MASS * G + air.down_f + air.down_r;
    // Four patches, each carrying a quarter, at the friction their load gives.
    let per = total * 0.25;
    let worst = dmg.wheel_grip(max(dmg.front, dmg.rear), max(dmg.left, dmg.right));
    4.0 * tire::mu(tire::MU_Y0, per) * per * worst
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ackermann_turns_the_inner_wheel_further() {
        let (l, r) = ackermann(0.35);
        println!("left turn at 0.35 rad: inner {l:.4}, outer {r:.4}");
        assert!(l > 0.35 && r < 0.35, "Ackermann is backwards");
        // Mirrored for a right-hander.
        let (l2, r2) = ackermann(-0.35);
        assert!(abs(l2 + r) < 1e-5 && abs(r2 + l) < 1e-5, "steering is not symmetric");
        // And it does nothing at all going straight.
        assert_eq!(ackermann(0.0), (0.0, 0.0));
    }

    #[test]
    fn wheels_are_where_the_geometry_says() {
        assert_eq!(wheel_pos(FL).x, LF);
        assert_eq!(wheel_pos(RR).x, -LR);
        assert!(wheel_pos(FL).y > 0.0, "front left is not on the left");
        assert_eq!(wheel_pos(FL).y - wheel_pos(FR).y, TRACK_F);
    }

    /// The broad phase rejects pairs on this radius, so a body corner may never
    /// stick out past it.
    #[test]
    fn bound_radius_contains_the_body() {
        let corner = crate::math::sqrt(HALF_LEN * HALF_LEN + HALF_WID * HALF_WID);
        println!("body corner at {corner:.4} m, broad phase reaches {BOUND_R:.4} m");
        assert!(BOUND_R >= corner, "broad phase would miss a corner contact");
        assert!(BOUND_R < corner * 1.1, "broad phase is needlessly loose");
    }

    /// Holding the reverse key from speed has to actually stop the car and
    /// then reverse it, without the driver being expected to know that a brake
    /// was also required.
    #[test]
    fn asking_for_reverse_brakes_first_and_then_reverses() {
        let mut c = CarState::default();
        c.vx = 20.0;
        c.sync_drivetrain();
        let inp = CarInput { throttle: -1.0, ..Default::default() };

        let mut stopped_at = 0.0f32;
        for i in 0..60 * 12 {
            for _ in 0..SUBSTEPS {
                integrate(&mut c, &inp, H);
            }
            if stopped_at == 0.0 && c.forward_speed() < 0.05 {
                stopped_at = i as f32 * DT;
            }
        }
        println!(
            "20 m/s, reverse held: stopped after {stopped_at:.2} s, then {:.1} m/s backwards in gear {}",
            c.forward_speed(),
            c.gear as i32
        );
        assert!(stopped_at > 0.0 && stopped_at < 3.0, "took {stopped_at:.2} s to stop");
        assert!(c.gear < 0.0, "never selected reverse");
        assert!(c.forward_speed() < -3.0, "only reached {:.2} m/s backwards", c.forward_speed());
    }

    /// And the assist has to keep out of the way the rest of the time.
    #[test]
    fn the_reverse_assist_does_nothing_to_a_car_going_the_right_way() {
        // Full throttle, going forwards, at every speed that matters.
        for v in [0.0f32, 0.5, 5.0, 40.0, 78.0] {
            assert_eq!(reverse_assist(1.0, 0.0, v), 0.0, "braked at {v} m/s under power");
        }
        // Coasting is not a request for a direction.
        assert_eq!(reverse_assist(0.0, 0.0, 30.0), 0.0);
        // It may never take the driver's own brake away.
        assert_eq!(reverse_assist(1.0, 0.7, 30.0), 0.7);
        // Rolling backwards with the throttle down brakes just the same.
        assert!(reverse_assist(1.0, 0.0, -5.0) > 0.9);
    }

    #[test]
    fn damage_makes_the_car_worse_at_every_job_it_has() {
        let straight = Damage::default();
        let wrecked = Damage { front: 0.35, rear: 0.0, left: 0.0, right: 0.0 };
        let a = aero::evaluate(60.0);
        let b = a.crushed(&wrecked);
        println!(
            "0.35 m of nose: front downforce {:.0} -> {:.0} N, drag {:.0} -> {:.0} N, lock x{:.2}, power x{:.2}",
            a.down_f, b.down_f, a.drag, b.drag, wrecked.steer_lock(), wrecked.power()
        );
        assert!(b.down_f < a.down_f * 0.6, "a folded nose still makes downforce");
        assert!(b.drag > a.drag * 1.3, "a folded nose is not draggy");
        assert!(grip_limit(60.0, &wrecked) < grip_limit(60.0, &straight) * 0.9);
        // And it pulls: crushed on one side only, the geometry has an opinion.
        let lopsided = Damage { front: 0.2, left: 0.3, ..Default::default() };
        println!("crushed on the left: pulls {:.3} rad", lopsided.steer_pull());
        assert!(lopsided.steer_pull() > 0.02, "a bent corner steers straight");
    }

    #[test]
    fn grip_limit_climbs_with_downforce() {
        let straight = Damage::default();
        let slow = grip_limit(10.0, &straight) / (MASS * G);
        let fast = grip_limit(70.0, &straight) / (MASS * G);
        println!("grip: {slow:.2} g at 10 m/s, {fast:.2} g at 70 m/s");
        assert!(fast > slow * 1.2, "downforce bought no grip");
        assert!(slow > 1.2 && slow < 1.8, "low-speed grip is {slow:.2} g");
    }
}
