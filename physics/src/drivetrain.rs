//! Engine, clutch, gearbox and differential: everything between the throttle
//! pedal and the two rear contact patches.
//!
//! The old model multiplied throttle by a force and called it a drivetrain.
//! This one carries the engine's rotational speed as real state and lets the
//! road push back on it, which changes the car in ways worth having:
//!
//! * **Torque follows the curve, not the pedal.** [`TORQUE`] peaks in the
//!   middle of the range and falls off at both ends, so the same throttle
//!   opening does different things at 2000 rpm and at 7000, and being in the
//!   wrong gear costs you real time.
//! * **The engine has inertia.** Spinning it up absorbs torque that is then not
//!   reaching the road, which is most of why first gear is slower than the
//!   gearing suggests.
//! * **The clutch slips.** [`cap`] limits what it can carry as a function of
//!   engine speed, so pulling away flares the revs and lets them bite -- and
//!   the engine can never be dragged below idle and stalled.
//! * **The differential is not a solid axle.** A limited-slip unit sends torque
//!   away from a wheel that is spinning up, which is what lets a rear-drive car
//!   put power down on corner exit with the inside wheel light.
//!
//! Gear selection is automatic because the client sends a throttle axis and not
//! a shifter. The shift itself takes [`SHIFT_TIME`] with the clutch out, so an
//! upshift mid-corner really does unload the rear for a moment.
//!
//! # One step for three bodies
//!
//! The clutch is a stiff damper between the engine and the axle, and the
//! differential another between the two wheels. At 90 N.m per rad/s against
//! 0.22 kg.m^2 the engine's mode alone would need ~2.4 kHz to step explicitly,
//! so every coupling is taken implicitly. The first version did that one body
//! at a time: the engine damped by the clutch's slope with the wheels held
//! still, each wheel by the same slope with the engine held still. That is
//! stable, and it is wrong. Under steady acceleration the engine and the wheels
//! speed up *together* and the clutch torque does not change at all, but each
//! body was still being charged for the change it would have seen had the other
//! stood still. Through first gear that phantom came to a fifth of a g: the
//! launch was slower at 480 Hz than at 2 kHz, the tires never reached their
//! peak, and traction control never had a reason to fire on a straight.
//!
//! [`Drivetrain::step`] now solves the engine and both driven wheels as one
//! 3x3 linear system per substep. The couplings are linear in the three speeds
//! (until a clutch or a diff reaches what it can hold, at which point it is a
//! constant and drops out of the Jacobian), so this is exact for them, and it
//! costs two divides. The tire's own slope stays on the diagonal, with the
//! road's expected acceleration folded in for the same reason -- see
//! [`crate::wheel::Wheel::spin`].

use crate::math::{abs, clamp, max, signum};
use crate::wheel::{self, Patch, Wheel};

/// Radians per second per rpm.
pub const RPM_TO_RAD: f32 = 0.104_719_75;

/// Engine torque, N.m, sampled every 800 rpm from 0 to 8000. Peak torque of
/// 425 N.m at 4800 rpm; peak power of about 275 kW just under the limiter.
const TORQUE: [f32; 11] = [
    0.0, 250.0, 310.0, 350.0, 385.0, 410.0, 425.0, 420.0, 400.0, 365.0, 300.0,
];
/// Spacing of the samples above, in rad/s.
const TORQUE_STEP: f32 = 800.0 * RPM_TO_RAD;

/// Engine and flywheel rotational inertia.
const I_ENGINE: f32 = 0.22;

pub const IDLE_RPM: f32 = 900.0;
pub const REDLINE_RPM: f32 = 7600.0;
/// Fuel cut. Slightly under the redline so the needle never pins.
const LIMITER: f32 = 7400.0 * RPM_TO_RAD;
const IDLE: f32 = IDLE_RPM * RPM_TO_RAD;
/// The engine is never allowed below this, which is what "it cannot stall"
/// means in practice. The clutch releasing at [`CLUTCH_LO`] is what makes it
/// true rather than enforced.
const STALL_FLOOR: f32 = 400.0 * RPM_TO_RAD;
/// Idle governor: a little torque, proportional to how far under idle it is.
const IDLE_GAIN: f32 = 3.0;
const IDLE_MAX: f32 = 90.0;

/// Closed-throttle drag: pumping losses and friction, rising with speed.
const BRAKE_BASE: f32 = 20.0;
const BRAKE_RATE: f32 = 0.055;

/// Forward gears, then the final drive.
///
/// Roughly geometric, each ratio about 0.73 of the one below it, which is what
/// keeps the engine landing back in the same part of its torque curve after
/// every shift. First is short enough to be over with by 60 km/h -- the
/// previous set ran it to 74 km/h, which meant a standing start spent three
/// seconds in one gear. Sixth is an overdrive tall enough that the limiter in
/// top sits past the speed the drag in [`crate::aero`] allows, so top speed is
/// set by power against air and not by running out of gearbox.
pub const GEARS: [f32; 6] = [3.55, 2.60, 1.91, 1.40, 1.03, 0.76];
pub const FINAL_DRIVE: f32 = 4.10;
const REVERSE: f32 = -3.60;

/// Road speed below which the gearbox will change direction, m/s.
///
/// A real one refuses at any more than a crawl, and is right to: a reverse
/// ratio is short enough that engaging it at speed would put the engine well
/// past the limiter backwards. [`crate::car::reverse_assist`] is what gets the
/// car down to here in the first place.
pub const REVERSE_BELOW: f32 = 0.6;

/// Driveline efficiency. The missing 8% is heat in the gears and bearings.
const EFFICIENCY: f32 = 0.92;

/// Shift points. Up near peak power, down far enough below it that a shift can
/// never immediately undo itself -- `gearbox_never_hunts` proves that for every
/// ratio pair rather than trusting the arithmetic here.
const UP_SHIFT: f32 = 7250.0 * RPM_TO_RAD;
const DOWN_SHIFT: f32 = 3300.0 * RPM_TO_RAD;
/// Seconds the clutch is out for. Long enough to feel, short enough to race.
pub const SHIFT_TIME: f32 = 0.12;
/// How fast the clutch comes back in afterwards, in engagement per second.
const CLUTCH_RATE: f32 = 8.0;

/// Torque the clutch can hold when fully engaged and spinning freely.
const CLUTCH_CAP: f32 = 700.0;
/// Stiffness of the slipping clutch, N.m per rad/s of slip. High enough that a
/// "locked" clutch gives up only tens of rpm under full load; the integrator
/// takes it implicitly, so the stiffness costs nothing in stability.
const CLUTCH_K: f32 = 90.0;
/// The clutch carries nothing below `LO` and everything above `HI`.
///
/// `LO` sits just *above* idle, which does two jobs: the engine can always
/// escape a load by dropping below it, so it cannot be stalled, and a car
/// sitting at idle in gear does not creep. The span up to `HI` is what makes a
/// standing start flare the revs and then bite, instead of bogging -- the
/// engine settles where its torque equals what the clutch can hold, which
/// `engine_revs_against_a_stalled_car` pins at a realistic launch speed. `HI`
/// is below the gearbox's downshift point, so under power the clutch is always
/// fully locked and none of this costs the car any torque.
const CLUTCH_LO: f32 = 950.0 * RPM_TO_RAD;
const CLUTCH_HI: f32 = 3_400.0 * RPM_TO_RAD;

/// Limited-slip differential. Preload is always there; the ramps add lock in
/// proportion to the torque going through it, and there is more of it on power
/// than on the overrun so that lifting mid-corner does not tie the axle
/// together and push the car straight on.
const LSD_PRELOAD: f32 = 90.0;
const LSD_RAMP_POWER: f32 = 0.32;
const LSD_RAMP_COAST: f32 = 0.14;
/// How hard the unit resists a speed difference, N.m per rad/s.
const LSD_K: f32 = 60.0;

/// A driven wheel turning slower than this, with a brake on it that can hold
/// whatever is trying to turn it, is locked: held at exactly zero rather than
/// stepped, so it neither creeps nor chatters against the brake.
const LOCKED: f32 = 0.5;
/// The diagonal a locked wheel gets in the step: bolted to the floor.
const BOLTED: f32 = 1e12;

/// What the drivetrain did to the rear axle this substep. Telemetry now: the
/// wheels themselves are moved inside [`Drivetrain::step`].
#[derive(Clone, Copy, Debug, Default)]
pub struct Output {
    /// Torque handed to each wheel at the start of the step, N.m.
    pub torque_l: f32,
    pub torque_r: f32,
    /// Engine speed as a fraction of the redline, for the tachometer.
    pub rpm: f32,
}

/// One driven wheel, as [`Drivetrain::step`] sees it: the wheel to move, what
/// its tire is doing, and the brake torque on it this substep.
pub struct Driven<'a> {
    pub wheel: &'a mut Wheel,
    pub patch: &'a Patch,
    pub brake: f32,
}

/// Everything between the pedal and the axle that has to be remembered from one
/// tick to the next.
#[derive(Clone, Copy, Debug)]
pub struct Drivetrain {
    /// Engine speed, rad/s.
    pub engine: f32,
    /// -1 reverse, 1..=6 forward.
    pub gear: f32,
    /// Seconds remaining of the current shift; 0 when engaged.
    pub shift: f32,
    /// Clutch engagement, 0..1.
    pub clutch: f32,
}

impl Default for Drivetrain {
    fn default() -> Self {
        Drivetrain {
            engine: IDLE,
            gear: 1.0,
            shift: 0.0,
            clutch: 1.0,
        }
    }
}

/// Engine torque at a given speed and throttle opening. Blends the wide-open
/// curve against closed-throttle drag, which is how engine braking arrives.
pub fn engine_torque(omega: f32, throttle: f32) -> f32 {
    // Fuel cut on the limiter: only the drag term survives.
    let open = if omega >= LIMITER { 0.0 } else { clamp(throttle, 0.0, 1.0) };

    let t = omega / TORQUE_STEP;
    let i = clamp(t, 0.0, (TORQUE.len() - 1) as f32) as usize;
    let j = if i + 1 < TORQUE.len() { i + 1 } else { i };
    let frac = clamp(t - i as f32, 0.0, 1.0);
    let wot = TORQUE[i] + (TORQUE[j] - TORQUE[i]) * frac;

    let drag = -(BRAKE_BASE + BRAKE_RATE * omega);
    let idle_assist = clamp((IDLE - omega) * IDLE_GAIN, 0.0, IDLE_MAX);

    open * wot + (1.0 - open) * drag + idle_assist
}

/// Torque the clutch can hold at this engine speed.
#[inline]
fn cap(engine: f32) -> f32 {
    CLUTCH_CAP * clamp((engine - CLUTCH_LO) / (CLUTCH_HI - CLUTCH_LO), 0.0, 1.0)
}

/// Overall ratio from the engine to the wheels, sign included.
#[inline]
fn ratio(gear: f32) -> f32 {
    if gear < 0.0 {
        REVERSE * FINAL_DRIVE
    } else {
        let i = clamp(gear, 1.0, GEARS.len() as f32) as usize - 1;
        GEARS[i] * FINAL_DRIVE
    }
}

/// The gear a driver accelerating through `speed` would be in, and the engine
/// speed that implies.
///
/// Needed whenever a car is handed a road speed without a drivetrain to match
/// -- a test setting up a corner, or a pose written in from outside. Getting it
/// wrong is not a cosmetic problem: leave a car in first at 126 km/h and the
/// engine is being driven at 15,000 rpm, the clutch hands the rear axle its
/// full capacity backwards through a ratio of fourteen, and the back end snaps
/// round before the gearbox can shift out of it. Which is exactly what a real
/// car would do, and exactly not what the caller meant.
pub fn gear_for(speed: f32) -> (f32, f32) {
    let w_wheel = abs(speed) / crate::wheel::RADIUS;
    let mut gear = GEARS.len();
    for (i, g) in GEARS.iter().enumerate() {
        if w_wheel * g * FINAL_DRIVE <= UP_SHIFT {
            gear = i + 1;
            break;
        }
    }
    let engine = clamp(
        w_wheel * GEARS[gear - 1] * FINAL_DRIVE,
        IDLE,
        REDLINE_RPM * RPM_TO_RAD,
    );
    (gear as f32, engine)
}

impl Drivetrain {
    /// Advance the engine, gearbox, differential and both driven wheels by `h`
    /// seconds, together. See the module docs for why together.
    ///
    /// `intent` is the driver's key or stick, signed, and `pedal` where the
    /// throttle pedal actually is after the foot, traction control and the
    /// state of the engine bay have all had their say -- signed the same way.
    /// They arrive separately because they mean different things: the gearbox
    /// reads the intent for which way the driver wants to go, and the engine
    /// gets the pedal. Handing the gearbox the pedal used to mean that
    /// traction control closing the throttle in reverse read as the driver
    /// letting go of the reverse key, and the box hunted between first and
    /// reverse four times a second on a car that was asking, the whole time,
    /// to back up.
    ///
    /// `forward` is the car's signed forward speed; `a_road` its longitudinal
    /// acceleration over the previous substep, which the tire terms need for
    /// the reason given at [`Wheel::spin`].
    pub fn step(
        &mut self,
        intent: f32,
        pedal: f32,
        left: Driven,
        right: Driven,
        forward: f32,
        a_road: f32,
        h: f32,
    ) -> Output {
        // A `CarState` that came from `Default` -- an empty slot, or a snapshot
        // written in from outside -- has a stopped engine in no gear. Rather
        // than let every caller remember to prime it, normalise here.
        if self.gear == 0.0 {
            self.gear = 1.0;
        }
        if self.engine < STALL_FLOOR {
            self.engine = IDLE;
        }

        // --- gear selection ------------------------------------------------
        if self.shift > 0.0 {
            self.shift = max(self.shift - h, 0.0);
        } else if self.clutch > 0.9 {
            // Only decide once the clutch is properly back in. Deciding while
            // it is still feeding would read an engine speed that the road is
            // not yet connected to, and shift again on the strength of it.
            self.select(intent, forward);
        }

        // The clutch comes out for a shift and eases back in afterwards.
        let target = if self.shift > 0.0 { 0.0 } else { 1.0 };
        let step = CLUTCH_RATE * h;
        self.clutch = clamp(self.clutch + clamp(target - self.clutch, -step, step), 0.0, 1.0);

        // Torque is cut for the duration of the shift, the way every automated
        // gearbox does it. Without the cut, a disconnected engine at full
        // throttle simply revs to the limiter during the 120 ms it is out of
        // gear -- and arrives back over the upshift threshold, which is a
        // gearbox that goes second, fourth, sixth in half a second.
        //
        // The sign is the gearbox's business, not the engine's: an engine only
        // ever turns one way, and `-1` on the wire means "reverse", not "run
        // the crankshaft backwards". Reverse is a negative *ratio* below, and
        // the pedal here is how far it is pressed.
        let pedal = if self.shift > 0.0 {
            0.0
        } else if self.gear < 0.0 {
            -pedal
        } else {
            pedal
        };
        let t_eng = engine_torque(self.engine, pedal);

        // --- clutch, at the start of the step ------------------------------
        // Linear in the slip until it reaches what it can hold, then a
        // constant. Only the linear side goes into the Jacobian: a clutch at
        // its cap is not a spring, and treating it as one would couple the
        // engine to an axle it is in fact sliding over.
        let r = ratio(self.gear);
        let w_l = left.wheel.omega;
        let w_r = right.wheel.omega;
        let w_diff = (w_l + w_r) * 0.5;
        let w_in = w_diff * r;
        // What the plate can hold depends on which way it is being asked to.
        // Driving, it tapers to nothing below `CLUTCH_LO`, which is what keeps
        // the engine alive under a load and the car from creeping at idle. On
        // the overrun -- the road turning the engine rather than the engine
        // the road -- there is nothing to protect, an engine being dragged
        // *up* cannot stall, and this is where engine braking comes from. It
        // used to taper both ways, which left a car that had been spun with
        // the handbrake coasting in gear at idle, the engine never picking the
        // axle back up because the axle was the faster of the two.
        let held = if self.engine >= w_in { cap(self.engine) } else { CLUTCH_CAP } * self.clutch;
        let t_free = CLUTCH_K * self.clutch * (self.engine - w_in);
        let (t_clutch, k_c) = if abs(t_free) >= held {
            (signum(t_free) * held, 0.0)
        } else {
            (t_free, CLUTCH_K * self.clutch)
        };

        // --- differential ---------------------------------------------------
        let t_diff = t_clutch * r * EFFICIENCY;
        let base = t_diff * 0.5;

        // Power or overrun? The ramps differ, so the unit locks harder under
        // acceleration than it does off the throttle.
        let driving = if abs(w_diff) > 0.5 {
            signum(t_diff) == signum(w_diff)
        } else {
            true
        };
        let ramp = if driving { LSD_RAMP_POWER } else { LSD_RAMP_COAST };
        let lock_cap = LSD_PRELOAD + ramp * abs(t_diff);
        let l_free = LSD_K * (w_l - w_r);
        let (lock, k_l) = if abs(l_free) >= lock_cap {
            (signum(l_free) * lock_cap, 0.0)
        } else {
            (l_free, LSD_K)
        };

        // --- brakes -----------------------------------------------------------
        // A Coulomb torque against each wheel's rotation, and it goes *into*
        // the step below rather than being applied after it. The rear brakes
        // are slowing the flywheel as well as the wheel, through the clutch;
        // applied afterwards as a step of their own they slowed only the
        // wheel, and the clutch then dragged it straight back up to the
        // engine's speed, so that under full braking the rear tires sat at one
        // percent slip with 1500 N.m going nowhere. A wheel that has already
        // stopped, and whose brake can hold whatever is trying to turn it, is
        // simply held -- bolted to the floor for this substep, which is what a
        // locked wheel is.
        let push_l = base - lock + left.patch.torque;
        let push_r = base + lock + right.patch.torque;
        let locked_l = abs(w_l) < LOCKED && abs(push_l) <= left.brake;
        let locked_r = abs(w_r) < LOCKED && abs(push_r) <= right.brake;
        let brake_l = if locked_l { -push_l } else { -signum(w_l) * left.brake };
        let brake_r = if locked_r { -push_r } else { -signum(w_r) * right.brake };

        // --- the step -------------------------------------------------------
        // Backward Euler on the three speeds, with every coupling linearised
        // about the state above. Written out, with `d` the change in each
        // speed over the step:
        //
        //   (I_e + h k_c) d_e    - b (d_l + d_r)                  = h (T_eng - T_c)
        //   -c d_e + D_l d_l + e d_r                              = h (T_l + road_l)
        //   -c d_e + e d_l + D_r d_r                              = h (T_r + road_r)
        //
        // where `b` and `c` carry the clutch through the gearing to and from
        // the axle, `g` is the clutch's grip on one wheel through the gearing
        // squared, and each wheel's diagonal is its inertia plus what the
        // clutch, the diff and its own tire resist a change in its speed with.
        // Eliminate the engine and the pair that is left is symmetric.
        let g = k_c * r * r * EFFICIENCY * 0.25;
        let a = I_ENGINE + h * k_c;
        let b = h * k_c * r * 0.5;
        let c = b * EFFICIENCY;
        let d_l = if locked_l { BOLTED } else { wheel::INERTIA + h * (g + k_l + left.patch.slope) };
        let d_r = if locked_r { BOLTED } else { wheel::INERTIA + h * (g + k_l + right.patch.slope) };
        let e = h * (g - k_l);
        // The road under each patch speeds up too; see `Wheel::spin`.
        let road = h * a_road / wheel::RADIUS;
        let b_e = h * (t_eng - t_clutch);
        let b_l = h * (push_l + brake_l + left.patch.slope * road);
        let b_r = h * (push_r + brake_r + right.patch.slope * road);

        let inv_a = 1.0 / a;
        let q = c * b * inv_a;
        let m_ll = d_l - q;
        let m_rr = d_r - q;
        let m_lr = e - q;
        let s_l = b_l + c * b_e * inv_a;
        let s_r = b_r + c * b_e * inv_a;
        let inv_det = 1.0 / (m_ll * m_rr - m_lr * m_lr);
        let d_wl = (m_rr * s_l - m_lr * s_r) * inv_det;
        let d_wr = (m_ll * s_r - m_lr * s_l) * inv_det;
        let d_we = (b_e + b * (d_wl + d_wr)) * inv_a;

        self.engine = clamp(self.engine + d_we, STALL_FLOOR, REDLINE_RPM * RPM_TO_RAD);
        // A brake can stop a wheel inside the step but never reverse it.
        left.wheel.omega = if locked_l || w_l * (w_l + d_wl) < 0.0 { 0.0 } else { w_l + d_wl };
        right.wheel.omega = if locked_r || w_r * (w_r + d_wr) < 0.0 { 0.0 } else { w_r + d_wr };

        Output {
            torque_l: base - lock,
            torque_r: base + lock,
            rpm: self.engine / (REDLINE_RPM * RPM_TO_RAD),
        }
    }

    /// Pick a gear. Automatic, because the wire carries a throttle axis and not
    /// a shifter.
    ///
    /// Shift points are read off the road, not the crankshaft: the engine speed
    /// the car's own speed implies in the current gear. The two agree whenever
    /// the tires are gripping, and where they do not is exactly where the
    /// engine is the wrong thing to read. Wheelspin off the line used to rev
    /// the engine over the upshift point while the car was still doing
    /// 30 km/h; the handbrake used to drag the engine to idle, and the box
    /// down to first, on a car doing 140.
    fn select(&mut self, throttle: f32, forward: f32) {
        // Reverse is only available from a near-standstill, in both directions.
        if throttle < -0.02 && forward < REVERSE_BELOW && self.gear > 0.0 {
            self.gear = -1.0;
            self.shift = SHIFT_TIME;
            return;
        }
        if self.gear < 0.0 {
            if throttle > -0.02 && forward > -REVERSE_BELOW {
                self.gear = 1.0;
                self.shift = SHIFT_TIME;
            }
            return;
        }

        let top = GEARS.len() as f32;
        let road = abs(forward) / wheel::RADIUS * abs(ratio(self.gear));
        if road > UP_SHIFT && self.gear < top && throttle > 0.1 {
            self.gear += 1.0;
            self.shift = SHIFT_TIME;
        } else if road < DOWN_SHIFT && self.gear > 1.0 {
            self.gear -= 1.0;
            self.shift = SHIFT_TIME;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A patch that will not let its wheel move: the axle bolted to the floor,
    /// which is what "wheels held" means for the tests below.
    const BOLTED: Patch = Patch {
        fx: 0.0,
        fy: 0.0,
        mz: 0.0,
        kappa: 0.0,
        alpha: 0.0,
        saturation: 0.0,
        torque: 0.0,
        slope: 1e12,
    };

    /// Step the drivetrain against wheels held at fixed speeds.
    fn held(d: &mut Drivetrain, throttle: f32, w_rl: f32, w_rr: f32, forward: f32, h: f32) -> Output {
        let mut l = Wheel { omega: w_rl, fy: 0.0 };
        let mut r = Wheel { omega: w_rr, fy: 0.0 };
        d.step(
            throttle,
            throttle,
            Driven { wheel: &mut l, patch: &BOLTED, brake: 0.0 },
            Driven { wheel: &mut r, patch: &BOLTED, brake: 0.0 },
            forward,
            0.0,
            h,
        )
    }

    /// Gearing has to put a usable road speed in every gear, and let top gear
    /// out past what the car can actually reach.
    #[test]
    fn the_ratios_cover_the_speed_range() {
        let wheel_speed = |gear: usize, engine: f32| {
            engine / (GEARS[gear] * FINAL_DRIVE) * crate::wheel::RADIUS
        };
        let first = wheel_speed(0, UP_SHIFT);
        let top = wheel_speed(GEARS.len() - 1, LIMITER);
        println!(
            "first gear runs to {:.0} km/h, sixth to {:.0} km/h on the limiter",
            first * 3.6,
            top * 3.6
        );
        assert!(first * 3.6 > 45.0 && first * 3.6 < 70.0, "first gear tops at {:.0} km/h", first * 3.6);
        // Comfortably past the ~76 m/s the drag in `aero.rs` allows, so the
        // gearbox is never what limits top speed.
        assert!(top > 79.0, "sixth gear caps the car at {:.0} m/s", top);
    }

    /// The engine must not be able to rev itself back over the upshift point
    /// while it is disconnected, or one shift triggers the next.
    #[test]
    fn a_shift_cuts_torque_so_it_cannot_chain() {
        let mut d = Drivetrain::default();
        d.gear = 2.0;
        d.engine = UP_SHIFT * 0.99;
        d.shift = SHIFT_TIME;
        d.clutch = 0.0;
        // Full throttle, wheels turning as they would be at that speed.
        let w = d.engine / (GEARS[1] * FINAL_DRIVE);
        let mut ticks = 0;
        while d.shift > 0.0 && ticks < 480 {
            held(&mut d, 1.0, w, w, w * crate::wheel::RADIUS, 1.0 / 480.0);
            ticks += 1;
        }
        println!(
            "engine through a full-throttle shift: {:.0} -> {:.0} rpm (upshift at {:.0})",
            UP_SHIFT * 0.99 / RPM_TO_RAD,
            d.engine / RPM_TO_RAD,
            UP_SHIFT / RPM_TO_RAD
        );
        assert_eq!(d.gear, 2.0, "gearbox shifted again mid-shift");
        assert!(d.engine < UP_SHIFT, "engine flared past the upshift point while out of gear");
    }

    /// A shift must never put the engine straight back over the opposite
    /// threshold, or the gearbox oscillates between two ratios forever.
    #[test]
    fn gearbox_never_hunts() {
        for i in 0..GEARS.len() - 1 {
            let after_up = UP_SHIFT * (GEARS[i + 1] / GEARS[i]);
            println!(
                "{}->{}: engine falls to {:.0} rpm (downshift at {:.0})",
                i + 1,
                i + 2,
                after_up / RPM_TO_RAD,
                DOWN_SHIFT / RPM_TO_RAD
            );
            assert!(after_up > DOWN_SHIFT * 1.1, "upshift {} lands in a downshift", i + 1);
            let after_down = DOWN_SHIFT * (GEARS[i] / GEARS[i + 1]);
            assert!(after_down < UP_SHIFT * 0.9, "downshift {} lands in an upshift", i + 2);
        }
    }

    /// The torque curve has to have a peak in the middle, or gear choice does
    /// not matter and the gearbox above is decoration.
    #[test]
    fn torque_curve_peaks_and_falls() {
        let mut best = (0usize, 0.0f32);
        for (i, t) in TORQUE.iter().enumerate() {
            if *t > best.1 {
                best = (i, *t);
            }
        }
        assert!(best.0 > 2 && best.0 < TORQUE.len() - 2, "peak torque is at an end of the range");
        assert!(
            *TORQUE.last().unwrap() < best.1 * 0.85,
            "engine does not fall off towards the limiter"
        );
        // Peak power, for the record.
        let (mut pw, mut at) = (0.0f32, 0.0f32);
        for (i, t) in TORQUE.iter().enumerate() {
            let w = i as f32 * TORQUE_STEP;
            if t * w > pw {
                pw = t * w;
                at = w / RPM_TO_RAD;
            }
        }
        println!("peak torque {:.0} N.m, peak power {:.0} kW at {at:.0} rpm", best.1, pw / 1000.0);
        assert!(pw > 200_000.0 && pw < 340_000.0, "peak power is {:.0} kW", pw / 1000.0);
    }

    /// Holding the throttle open with the wheels held still must rev the engine
    /// out rather than bogging it -- that is the clutch doing its job.
    #[test]
    fn engine_revs_against_a_stalled_car() {
        let mut d = Drivetrain::default();
        for _ in 0..480 {
            held(&mut d, 1.0, 0.0, 0.0, 0.0, 1.0 / 480.0);
        }
        let rpm = d.engine / RPM_TO_RAD;
        println!("one second at full throttle, wheels held: {rpm:.0} rpm");
        assert!(rpm > 2_000.0, "engine bogged to {rpm:.0} rpm instead of slipping the clutch");
        assert!(rpm <= REDLINE_RPM + 1.0, "engine passed the redline");

        // And the clutch is fully locked by the time the gearbox would ever
        // ask for drive, so slipping it never costs the car torque on track.
        assert!(cap(DOWN_SHIFT) > CLUTCH_CAP * 0.95, "clutch still slipping at the downshift point");
    }

    /// A car left in gear at idle must not creep away on its own.
    #[test]
    fn idle_in_gear_does_not_creep() {
        let mut d = Drivetrain::default();
        let out = held(&mut d, 0.0, 0.0, 0.0, 0.0, 1.0 / 480.0);
        println!("at idle, in first, off the throttle: {:.1} N.m to the axle", out.torque_l + out.torque_r);
        assert_eq!(cap(IDLE), 0.0, "the clutch is holding torque at idle");
        assert!(abs(out.torque_l + out.torque_r) < 1.0);
    }

    /// And it must never stall, however hard the driveline pulls on it.
    #[test]
    fn engine_cannot_be_stalled() {
        let mut d = Drivetrain::default();
        for _ in 0..2400 {
            held(&mut d, 0.0, 0.0, 0.0, 0.0, 1.0 / 480.0);
        }
        let rpm = d.engine / RPM_TO_RAD;
        println!("five seconds closed-throttle against a stopped axle: {rpm:.0} rpm");
        assert!(rpm > 500.0, "engine stalled at {rpm:.0} rpm");
    }

    /// A wheel spinning up has to lose torque to the one that is not.
    #[test]
    fn the_diff_sends_torque_to_the_slower_wheel() {
        let mut d = Drivetrain::default();
        // Left wheel spinning far faster than the right.
        let out = held(&mut d, 1.0, 90.0, 40.0, 20.0, 1.0 / 480.0);
        println!("torque split with 50 rad/s across the axle: l {:.0} r {:.0}", out.torque_l, out.torque_r);
        assert!(out.torque_r > out.torque_l, "diff fed the spinning wheel");
    }

    /// Reverse has to actually go backwards. The engine only turns one way, so
    /// a negative throttle means "press the pedal, in reverse gear" -- if that
    /// signed value reaches the torque curve unchanged it clamps to zero and
    /// the car sits there with the engine idling.
    #[test]
    fn reverse_drives_the_car_backwards() {
        let mut d = Drivetrain::default();
        // Stopped, asking for reverse.
        let mut out = Output::default();
        for _ in 0..480 {
            out = held(&mut d, -0.7, 0.0, 0.0, 0.0, 1.0 / 480.0);
        }
        println!(
            "in reverse at {:.0} rpm: {:.0} N.m to the axle",
            d.engine / RPM_TO_RAD,
            out.torque_l + out.torque_r
        );
        assert_eq!(d.gear, -1.0, "never selected reverse");
        assert!(d.engine / RPM_TO_RAD > 1_200.0, "engine idled instead of revving");
        assert!(
            out.torque_l + out.torque_r < -200.0,
            "reverse made {:.0} N.m, which is not going anywhere",
            out.torque_l + out.torque_r
        );
    }

    #[test]
    fn closed_throttle_brakes_the_engine() {
        assert!(engine_torque(500.0, 0.0) < 0.0, "no engine braking at 4800 rpm");
        assert!(engine_torque(500.0, 1.0) > 0.0, "no drive at 4800 rpm");
    }

    /// Whatever speed a car is handed, the gear picked for it has to leave the
    /// engine somewhere it could actually be running.
    #[test]
    fn every_road_speed_has_a_sensible_gear() {
        let mut v = 0.0f32;
        while v <= 85.0 {
            let (gear, engine) = gear_for(v);
            let rpm = engine / RPM_TO_RAD;
            assert!(gear >= 1.0 && gear <= GEARS.len() as f32, "gear {gear} at {v} m/s");
            assert!(rpm >= IDLE_RPM - 1.0 && rpm <= REDLINE_RPM + 1.0, "{rpm:.0} rpm at {v} m/s");
            v += 1.0;
        }
        for v in [0.0f32, 15.0, 30.0, 50.0, 76.0] {
            let (gear, engine) = gear_for(v);
            println!("  {v:4.0} m/s -> gear {gear:.0} at {:.0} rpm", engine / RPM_TO_RAD);
        }
    }

    #[test]
    fn the_limiter_cuts_fuel() {
        let over = LIMITER + 10.0;
        assert!(engine_torque(over, 1.0) < 0.0, "full throttle past the limiter still drives");
    }

    /// The whole reason for the coupled step: first gear bolts the engine to
    /// the wheels through a ratio of fourteen, and the result has to sit still
    /// at 480 Hz with the tires pushing back at tens of kN.m per unit slip.
    #[test]
    fn stays_stable_in_first_gear_under_load() {
        let mut d = Drivetrain::default();
        let (gear, engine) = gear_for(20.0);
        d.gear = gear;
        d.engine = engine;
        let mut l = Wheel::default();
        let mut r = Wheel::default();
        l.roll_at(20.0);
        r.roll_at(20.0);
        let c = crate::wheel::Contact { u: 20.0, v: 0.0, fz: 2600.0, grip: 1.0 };
        for _ in 0..2400 {
            let pl = l.patch(&c, 1.0 / 480.0);
            let pr = r.patch(&c, 1.0 / 480.0);
            d.step(
                1.0,
                1.0,
                Driven { wheel: &mut l, patch: &pl, brake: 0.0 },
                Driven { wheel: &mut r, patch: &pr, brake: 0.0 },
                20.0,
                0.0,
                1.0 / 480.0,
            );
            assert!(l.omega.is_finite() && abs(l.omega) < 1e4, "wheel blew up: {}", l.omega);
            assert!(d.engine.is_finite(), "engine blew up");
        }
        println!("full throttle in first at 20 m/s for 5 s: wheel {:.1} rad/s, engine {:.0} rpm", l.omega, d.engine / RPM_TO_RAD);
    }

    /// Under steady acceleration the engine and the axle speed up together and
    /// the clutch torque does not change, so the step must hand the road the
    /// engine's torque less what its own inertia takes -- and give the same
    /// answer at any substep. The one-body-at-a-time version lost a fifth of a
    /// g here and did not.
    #[test]
    fn steady_acceleration_loses_nothing_to_the_integrator() {
        // A heavily loaded tire so it can carry the whole engine without
        // saturating: the drive force is then read straight off the patch.
        let drive_at = |div: u32| {
            let h = 1.0 / (480.0 * div as f32);
            let mut d = Drivetrain::default();
            d.gear = 1.0;
            let mut l = Wheel::default();
            let mut r = Wheel::default();
            let mut u = 8.0f32;
            l.roll_at(u);
            r.roll_at(u);
            d.engine = u / crate::wheel::RADIUS * ratio(1.0);
            let a = 7.0f32;
            let mut fx = 0.0;
            for _ in 0..240 * div {
                let c = crate::wheel::Contact { u, v: 0.0, fz: 12_000.0, grip: 1.0 };
                let pl = l.patch(&c, h);
                let pr = r.patch(&c, h);
                fx = pl.fx + pr.fx;
                d.step(
                    1.0,
                    1.0,
                    Driven { wheel: &mut l, patch: &pl, brake: 0.0 },
                    Driven { wheel: &mut r, patch: &pr, brake: 0.0 },
                    u,
                    a,
                    h,
                );
                u += a * h;
            }
            (fx, d.engine)
        };
        let (coarse, engine) = drive_at(1);
        let (fine, _) = drive_at(16);
        // What the engine has to give at that speed, less its own inertia,
        // less the two wheels', through the gearing.
        let alpha = 7.0 / crate::wheel::RADIUS;
        let expected = (engine_torque(engine, 1.0) - I_ENGINE * ratio(1.0) * alpha) * ratio(1.0) * EFFICIENCY
            / crate::wheel::RADIUS
            - 2.0 * wheel::INERTIA * alpha / crate::wheel::RADIUS
            - 2.0 * crate::aero::ROLL_RESIST * 12_000.0;
        println!(
            "first gear, 7 m/s^2: {coarse:.0} N at 480 Hz, {fine:.0} N at 7680 Hz, torque balance says {expected:.0} N"
        );
        assert!(abs(coarse - fine) < expected * 0.05, "drive force depends on the substep");
        assert!(abs(coarse - expected) < expected * 0.08, "integrator is eating torque");
    }
}
