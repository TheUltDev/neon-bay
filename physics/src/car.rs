//! Vehicle dynamics: a lateral-slip bicycle model with longitudinal load
//! transfer and a friction circle, which is what makes it possible to lose the
//! back end and hold a slide instead of just steering a dot around.
//!
//! Everything is plain `f32` and every transcendental comes from [`crate::math`],
//! so the sidecar (x86-64) and the browser (wasm32) produce identical bits.

use crate::math::{abs, atan2, clamp, signum, sqrt, wrap_pi, V2};

/// Simulation rate. Both the sidecar and the client advance at exactly this.
pub const TICK_HZ: u32 = 60;
pub const DT: f32 = 1.0 / 60.0;
/// Dynamics substeps per tick. Two is enough to keep stiff tires stable.
pub const SUBSTEPS: u32 = 2;
pub const H: f32 = DT / SUBSTEPS as f32;

pub const G: f32 = 9.81;

// --- chassis ------------------------------------------------------------
pub const MASS: f32 = 1150.0;
pub const INV_MASS: f32 = 1.0 / MASS;
/// Yaw inertia.
pub const IZ: f32 = 1350.0;
/// CG to front axle / rear axle.
pub const LF: f32 = 1.25;
pub const LR: f32 = 1.45;
pub const WHEELBASE: f32 = LF + LR;
pub const CG_HEIGHT: f32 = 0.52;

/// Body half extents, for rendering and collision.
pub const HALF_LEN: f32 = 2.10;
pub const HALF_WID: f32 = 0.95;
/// The body is approximated by two circles at +/-[`CIRCLE_OFF`] along its axis.
pub const CIRCLE_OFF: f32 = 1.06;
pub const CIRCLE_R: f32 = 1.04;

// --- tires --------------------------------------------------------------
/// Cornering stiffness, N per radian of slip. Rear is stiffer for stability.
pub const C_ALPHA_F: f32 = 78_000.0;
pub const C_ALPHA_R: f32 = 90_000.0;
pub const GRIP_F: f32 = 1.55;
pub const GRIP_R: f32 = 1.62;
/// Rear grip multiplier while the handbrake is pulled.
const HANDBRAKE_GRIP: f32 = 0.42;

// --- drivetrain ---------------------------------------------------------
pub const POWER: f32 = 300_000.0;
pub const FORCE_MAX: f32 = 10_000.0;
const REVERSE_FORCE: f32 = 4_200.0;
pub const BRAKE_FORCE: f32 = 17_000.0;
const ENGINE_BRAKE: f32 = 900.0;
pub const DRAG: f32 = 0.85;
pub const ROLL_RESIST: f32 = 12.0;
/// Downforce coefficient, N per (m/s)^2. Grip climbs with speed.
pub const DOWNFORCE: f32 = 1.55;

pub const MAX_STEER: f32 = 0.58;
const STEER_RATE: f32 = 5.0;
const YAW_DAMP: f32 = 620.0;

/// Gear ratios purely for the tachometer -- they do not affect the physics.
const GEAR_TOP: [f32; 6] = [12.0, 22.0, 33.0, 45.0, 57.0, 90.0];

/// One car, laid out for zero-copy sharing with JavaScript.
///
/// The whole struct is `f32` -- including small integers like the lap counter --
/// so the browser can map it with a single `Float32Array` over wasm memory and
/// the sidecar can memcpy it straight into a network snapshot. Values below
/// 2^24 are exact in `f32`, which covers every counter here.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CarState {
    pub x: f32,
    pub y: f32,
    pub heading: f32,
    pub vx: f32,
    pub vy: f32,
    pub omega: f32,
    /// Road-wheel angle, radians.
    pub steer: f32,
    /// Filtered longitudinal acceleration, kept across ticks for load transfer.
    pub ax: f32,
    pub slip_f: f32,
    pub slip_r: f32,
    /// 0..1 traction loss at the rear -- drives tire smoke and marks.
    pub wheel_spin: f32,
    /// 0..1 normalized engine speed.
    pub rpm: f32,
    pub gear: f32,
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
    pub active: f32,
}

/// Number of `f32`s in [`CarState`]. Asserted against the real layout in tests.
pub const CAR_FLOATS: usize = 24;

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

    /// Place the car, stopped, at a pose. Also invalidates the track hint.
    pub fn place(&mut self, pos: V2, heading: f32) {
        *self = CarState {
            x: pos.x,
            y: pos.y,
            heading,
            seg: crate::track::NO_HINT as f32,
            lap: self.lap,
            cp: self.cp,
            lap_start: self.lap_start,
            last_lap: self.last_lap,
            best_lap: self.best_lap,
            active: self.active,
            ..Default::default()
        };
    }
}

/// Road-wheel angle a full-lock input produces at a given speed. Steering lock
/// tapers off with speed, and the bot driver needs the same curve to convert a
/// desired steering angle back into an input value.
#[inline]
pub fn steer_lock(speed: f32) -> f32 {
    MAX_STEER / (1.0 + speed * 0.032)
}

/// Advance one car by `h` seconds of dynamics. Collisions and lap bookkeeping
/// are handled by [`crate::world::World::step`].
pub fn integrate(car: &mut CarState, input: &CarInput, h: f32) {
    let inp = input.sanitize();

    let speed = car.speed();

    // --- steering actuator: less lock the faster you go -------------------
    let lock = steer_lock(speed);
    let target = inp.steer * lock;
    let max_delta = STEER_RATE * h;
    car.steer += clamp(target - car.steer, -max_delta, max_delta);

    // --- body-frame velocity ---------------------------------------------
    let v_body = car.vel().to_local(car.heading);
    let u = v_body.x; // forward
    let w = v_body.y; // left
    let dir = if u >= 0.0 { 1.0 } else { -1.0 };
    // Tire slip is undefined at a standstill; clamp the denominator and fade
    // lateral force out at walking pace so a parked car does not creep.
    let ud = if abs(u) > 3.0 { abs(u) } else { 3.0 };
    let lat_fade = clamp(speed * 0.55, 0.0, 1.0);

    // --- vertical loads ---------------------------------------------------
    let df = DOWNFORCE * speed * speed;
    let transfer = MASS * car.ax * CG_HEIGHT / WHEELBASE;
    let fz_f = clamp(
        MASS * G * (LR / WHEELBASE) - transfer + df * 0.45,
        600.0,
        45_000.0,
    );
    let fz_r = clamp(
        MASS * G * (LF / WHEELBASE) + transfer + df * 0.55,
        600.0,
        45_000.0,
    );

    // --- slip angles ------------------------------------------------------
    let alpha_f = atan2(w + LF * car.omega, ud) - car.steer * dir;
    let alpha_r = atan2(w - LR * car.omega, ud);

    // --- grip budget -------------------------------------------------------
    let cap_f = GRIP_F * fz_f;
    let cap_r = GRIP_R * fz_r;

    // --- longitudinal forces ---------------------------------------------
    // Constant-power curve, then limited to just over what the rear tires can
    // actually put down. Without that ceiling a keyboard's all-or-nothing
    // throttle would spin the car on every corner exit.
    let drive = if inp.throttle >= 0.0 {
        let by_power = POWER / if abs(u) > 8.0 { abs(u) } else { 8.0 };
        let f = if by_power < FORCE_MAX { by_power } else { FORCE_MAX };
        let traction = cap_r * 1.15;
        inp.throttle * if f < traction { f } else { traction }
    } else {
        inp.throttle * REVERSE_FORCE
    };
    let coast = if inp.throttle.abs() < 0.02 {
        -signum(u) * ENGINE_BRAKE
    } else {
        0.0
    };
    let braking = inp.brake * BRAKE_FORCE * -signum(u);

    // Brake distribution follows the grip available at each axle, so the rear
    // does not snap loose the moment weight transfers forward. Real cars have a
    // proportioning valve or EBD for exactly this reason.
    let bias_f = cap_f / (cap_f + cap_r);
    let fx_f = braking * bias_f;
    let mut fx_r = drive + coast + braking * (1.0 - bias_f);

    // Lateral demand from the tire model, before the friction limit.
    let fy_f = -C_ALPHA_F * alpha_f * dir * lat_fade;
    let mut fy_r = -C_ALPHA_R * alpha_r * dir * lat_fade;

    if inp.handbrake > 0.5 {
        // Locked rear wheels: most of the budget goes into scrubbing off speed
        // and what lateral grip remains is a fraction of normal.
        fx_r = -signum(u) * cap_r * 0.85;
        fy_r *= HANDBRAKE_GRIP;
    }

    // Friction ellipse. Scaling the *whole* force vector back into the circle
    // (rather than clamping the components independently) is what makes power
    // oversteer controllable: at full throttle the rear keeps a usable slice of
    // lateral grip instead of dropping to exactly zero and spinning the car.
    let (fx_f, fy_f, sat_f) = saturate(fx_f, fy_f, cap_f);
    let (fx_r, fy_r, sat_r) = saturate(fx_r, fy_r, cap_r);

    car.slip_f = alpha_f;
    car.slip_r = alpha_r;
    // Smoke comes from the rear axle running out of grip, mostly.
    car.wheel_spin = clamp(sat_r * 2.2 + sat_f * 0.4, 0.0, 1.0);

    // --- resistance -------------------------------------------------------
    let resist = -signum(u) * (DRAG * u * u + ROLL_RESIST * abs(u));

    // --- equations of motion (body frame, +x forward, +y left, +w CCW) -----
    let (sn, cs) = crate::math::sin_cos(car.steer);
    let fx_body = fx_r + fx_f * cs - fy_f * sn + resist;
    let fy_body = fy_r + fy_f * cs + fx_f * sn;
    let mz = LF * (fy_f * cs + fx_f * sn) - LR * fy_r - YAW_DAMP * car.omega;

    // No `omega x v` transport terms here: velocity is *stored* in world space
    // and re-projected into the body frame every substep, so the rotation of the
    // frame is already accounted for. Adding them as well double-counts it and
    // quietly cancels half of the cornering force.
    let acc_u = fx_body * INV_MASS;
    let acc_w = fy_body * INV_MASS;

    let nu = u + acc_u * h;
    let nw = w + acc_w * h;
    car.omega += mz / IZ * h;

    // Filtered, because raw acceleration fed back into load transfer rings.
    car.ax += (fx_body * INV_MASS - car.ax) * 0.25;

    let world_v = V2::new(nu, nw).to_world(car.heading);
    car.vx = world_v.x;
    car.vy = world_v.y;
    car.x += car.vx * h;
    car.y += car.vy * h;
    car.heading = wrap_pi(car.heading + car.omega * h);

    // Settle: kill the residual crawl of a stopped car.
    if car.speed() < 0.12 && inp.throttle.abs() < 0.02 {
        car.vx = 0.0;
        car.vy = 0.0;
        car.omega *= 0.5;
    }

    update_tacho(car, nu);
}

/// Clip a tire force vector into its friction circle, returning how much had
/// to be given up (0 = inside the circle, ->1 = fully saturated).
#[inline]
fn saturate(fx: f32, fy: f32, cap: f32) -> (f32, f32, f32) {
    let mag = sqrt(fx * fx + fy * fy);
    if mag <= cap || mag < 1e-3 {
        (fx, fy, 0.0)
    } else {
        let k = cap / mag;
        (fx * k, fy * k, 1.0 - k)
    }
}

fn update_tacho(car: &mut CarState, forward: f32) {
    let sp = abs(forward);
    let mut gear = 0usize;
    while gear + 1 < GEAR_TOP.len() && sp > GEAR_TOP[gear] {
        gear += 1;
    }
    let lo = if gear == 0 { 0.0 } else { GEAR_TOP[gear - 1] };
    let hi = GEAR_TOP[gear];
    let frac = clamp((sp - lo) / (hi - lo), 0.0, 1.0);
    let target = 0.18 + 0.82 * frac + 0.35 * car.wheel_spin;
    // Needles have mass.
    car.rpm += (clamp(target, 0.0, 1.15) - car.rpm) * 0.25;
    car.gear = (gear + 1) as f32;
}
