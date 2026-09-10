//! One wheel: an angular velocity, and what it does to the road.
//!
//! Giving each wheel its own rotational state is the change that makes the
//! longitudinal side of the tire model mean anything. Slip ratio stops being a
//! number invented from the throttle position and becomes what it actually is
//! -- the mismatch between how fast the tire is turning and how fast the car is
//! travelling. Everything downstream then arrives for free:
//!
//! * **Wheelspin** is the rear wheels genuinely turning faster than the road.
//! * **Lock-up** is a wheel at zero rad/s while the car keeps moving, which
//!   drives slip ratio to -1, saturates the patch and takes the steering away
//!   with it -- so a locked front really does mean the car goes straight.
//! * **The handbrake** needs no special case at all. Enough torque to stop the
//!   rear wheels turning is enough to break the rear away, because a saturated
//!   contact patch has nothing left for cornering. See [`crate::tire`].
//!
//! The catch is stiffness. Coupling a 1.2 kg.m^2 wheel to a tire that makes
//! tens of kilonewtons per unit of slip gives a mode with a sub-millisecond
//! time constant, and through first gear the engine is bolted to it as well.
//! Stepping that explicitly would need several kHz. So each substep is split in
//! two: [`Wheel::patch`] asks the tire what it is doing and hands back the
//! torque *and its slope*, and then whoever integrates the wheel takes that
//! slope implicitly. For an undriven wheel that is [`Wheel::spin`]; for the
//! driven pair it is [`crate::drivetrain`], which steps the engine and both
//! rear wheels as one system, because the couplings between them are the stiff
//! part and cannot be taken one body at a time. See the note on that.

use crate::aero::ROLL_RESIST;
use crate::math::{abs, atan, clamp, signum};
use crate::tire;

/// Rolling radius, meters.
pub const RADIUS: f32 = 0.32;
/// Wheel, tire and hub rotational inertia.
pub const INERTIA: f32 = 1.2;

/// Floor on the speed that slip is measured against. Slip ratio and slip angle
/// are both undefined at a standstill; this is the standard way to keep them
/// finite without special-casing the whole model.
///
/// Public because the assists in [`crate::car`] read slip ratio one step ahead
/// of the tires, to decide the throttle and brake the tires are then given. If
/// the two ever measured it against different speeds they would disagree about
/// what the wheel is doing, which is a hard thing to notice and an easy thing
/// to prevent.
pub const V_MIN: f32 = 2.0;

/// Tire relaxation length, meters. A carcass builds cornering force over a
/// distance rolled, not instantly, so a fast steering input arrives at the road
/// a few hundredths of a second late.
const RELAX_LEN: f32 = 0.45;
/// Added to the speed in the relaxation rate so that force still converges when
/// the car is barely moving.
const RELAX_FLOOR: f32 = 1.5;

/// Maximum brake torque per wheel, N.m. The front/rear split is the brake bias.
pub const BRAKE_F: f32 = 2_600.0;
pub const BRAKE_R: f32 = 1_500.0;
/// The handbrake, which is a rear-only cable and does not care about ABS.
pub const BRAKE_HAND: f32 = 3_200.0;

/// Slip ratio past which ABS starts releasing. Just beyond the peak of the
/// longitudinal curve, which is where a real system aims to sit -- and the
/// peak is at 0.08, not the 0.13 this used to be. `assists_sit_at_the_peak`
/// reads it off the tire so the two cannot drift apart again.
const ABS_SLIP: f32 = 0.10;
const ABS_GAIN: f32 = 8.0;
/// ABS never releases the brake entirely.
const ABS_FLOOR: f32 = 0.12;

/// Slip ratio past which traction control starts closing the throttle, and
/// how hard. Just past the peak, closed entirely a tenth beyond it. It used to
/// start at 0.14 and take until 0.39 to close, which is not traction control
/// so much as a comment about it: on a keyboard, whose throttle is a switch,
/// the inside rear ran at a quarter slip on every corner exit with more than
/// half the pedal still in, and a tire at a quarter slip has a fifth of its
/// cornering grip left.
const TC_SLIP: f32 = 0.09;
const TC_GAIN: f32 = 10.0;

/// One wheel's persistent state.
#[derive(Clone, Copy, Debug, Default)]
pub struct Wheel {
    /// Angular velocity, rad/s. Positive drives the car forward.
    pub omega: f32,
    /// Lateral force actually developed, after relaxation. Newtons.
    pub fy: f32,
}

/// What the chassis is doing to this contact patch this substep.
#[derive(Clone, Copy, Debug, Default)]
pub struct Contact {
    /// Contact point velocity along the wheel's heading.
    pub u: f32,
    /// Contact point velocity to the wheel's left.
    pub v: f32,
    /// Vertical load, newtons.
    pub fz: f32,
    /// Grip scale, for surface or compound. 1.0 is nominal.
    pub grip: f32,
}

/// What this patch did about it: the forces for the chassis, and the torque
/// and its slope for whoever integrates the wheel.
#[derive(Clone, Copy, Debug, Default)]
pub struct Patch {
    /// In the wheel frame: `fx` along its heading, `fy` to its left.
    pub fx: f32,
    pub fy: f32,
    /// Self-aligning moment about the contact patch.
    pub mz: f32,
    pub kappa: f32,
    pub alpha: f32,
    /// Combined slip, 1.0 at the grip peak.
    pub saturation: f32,
    /// Net torque the road puts on the wheel: rolling resistance plus the
    /// tire's reaction to being driven or dragged. Positive spins it forward.
    pub torque: f32,
    /// How much harder the road pushes back for every rad/s the wheel gains on
    /// it, from the tire's slope in the linear region. N.m per rad/s. This is
    /// what makes the wheel step implicit, and stable at any rate.
    pub slope: f32,
}

impl Wheel {
    /// Put the wheel at the speed a free roll at `u` would give it. Used when a
    /// car is placed rather than driven there.
    #[inline]
    pub fn roll_at(&mut self, u: f32) {
        self.omega = u / RADIUS;
        self.fy = 0.0;
    }

    /// Ask the tire what it is doing under this contact, and advance the
    /// relaxed lateral force by `h` seconds. The wheel itself does not move
    /// here: that is [`Wheel::spin`], or the drivetrain's coupled step.
    pub fn patch(&mut self, c: &Contact, h: f32) -> Patch {
        let u_ref = if abs(c.u) > V_MIN { abs(c.u) } else { V_MIN };

        let kappa = (self.omega * RADIUS - c.u) / u_ref;
        let alpha = atan(-c.v / u_ref);
        let f = tire::evaluate(c.fz, kappa, alpha, c.grip);

        // Lateral force lags by a relaxation length. Implicit, so it is stable
        // however fast the car is going.
        let rate = (abs(c.u) + RELAX_FLOOR) / RELAX_LEN;
        self.fy = (self.fy + h * rate * f.fy) / (1.0 + h * rate);

        // Rolling resistance is a torque at the patch, so it scales with the
        // load this tire is carrying -- downforce included.
        let rolling = -signum(self.omega) * ROLL_RESIST * c.fz * RADIUS;

        Patch {
            fx: f.fx,
            fy: self.fy,
            // The lagged force acts on the same trail the unlagged one did.
            mz: -f.trail * self.fy,
            kappa,
            alpha,
            saturation: f.saturation,
            torque: rolling - f.fx * RADIUS,
            slope: f.stiffness_x * RADIUS * RADIUS / u_ref,
        }
    }

    /// Advance an undriven wheel by `h` seconds under its patch and a brake.
    ///
    /// Implicit in the tire's slope. `a_road` is how fast the road under the
    /// patch is expected to speed up over the step -- the body's longitudinal
    /// acceleration, near enough -- and it is not optional. The slope says how
    /// much harder the tire pushes back if the wheel gains on the road, but
    /// under steady acceleration the road is gaining too, and the net slip does
    /// not change at all. Charging the wheel as if the road stood still is a
    /// resisting torque that never existed: at 480 Hz and 8 m/s it came to
    /// 0.04 g across the two front wheels alone. Predicting the road's share
    /// from the previous substep's acceleration cancels it exactly at steady
    /// state and leaves a second-order remainder during transients.
    pub fn spin(&mut self, p: &Patch, brake: f32, a_road: f32, h: f32) {
        let inertia = INERTIA + h * p.slope;
        self.omega += h * (p.torque + p.slope * h * a_road / RADIUS) / inertia;
        self.brake(brake, inertia, h);
    }

    /// The brake can stop the wheel but never reverse it. Clamping at the
    /// crossing is what lets a wheel actually lock and stay locked. `inertia`
    /// is whatever the wheel's step just resisted its other torques with.
    #[inline]
    pub fn brake(&mut self, brake: f32, inertia: f32, h: f32) {
        let step = brake * h / inertia;
        self.omega = if abs(self.omega) <= step {
            0.0
        } else {
            self.omega - signum(self.omega) * step
        };
    }
}

/// Anti-lock: back the brake off a wheel that has gone past the slip its tire
/// makes peak force at. Modelled as a proportional release rather than the
/// on-off cycling of a real hydraulic unit, which lands in the same place
/// without putting a 15 Hz buzz into a 60 Hz simulation.
#[inline]
pub fn anti_lock(brake: f32, kappa: f32) -> f32 {
    if kappa >= -ABS_SLIP {
        return brake;
    }
    brake * clamp(1.0 - (-kappa - ABS_SLIP) * ABS_GAIN, ABS_FLOOR, 1.0)
}

/// Traction control: how much of the pedal to let through when the driven
/// wheels light up. `spin` is slip ratio measured in the direction the gearbox
/// is trying to move the car, against the road only where the road is going
/// that way too -- see [`crate::car`] for why. The caller hands it the worse of
/// the two rear wheels, so a single spinning inside wheel on corner exit is
/// enough to trigger it.
#[inline]
pub fn traction_control(spin: f32) -> f32 {
    if spin <= TC_SLIP {
        return 1.0;
    }
    clamp(1.0 - (spin - TC_SLIP) * TC_GAIN, 0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contact(u: f32, fz: f32) -> Contact {
        Contact { u, v: 0.0, fz, grip: 1.0 }
    }

    /// One substep of an undriven wheel: evaluate the patch, then spin.
    fn step(w: &mut Wheel, c: &Contact, brake: f32, h: f32) -> Patch {
        let p = w.patch(c, h);
        w.spin(&p, brake, 0.0, h);
        p
    }

    /// An undriven, unbraked wheel under a rolling car has to settle at the
    /// speed the road is turning it, and stay there.
    #[test]
    fn a_free_wheel_finds_rolling_speed() {
        let mut w = Wheel::default();
        let c = contact(30.0, 3000.0);
        for _ in 0..480 {
            step(&mut w, &c, 0.0, 1.0 / 480.0);
        }
        let slip = (w.omega * RADIUS - c.u) / c.u;
        println!("free rolling at 30 m/s: {:.2} rad/s, slip {slip:.4}", w.omega);
        // Not exactly zero: rolling resistance has to be paid for with a little
        // slip, which is precisely what rolling resistance is.
        assert!(abs(slip) < 0.01, "free wheel settled at {slip:.4} slip");
    }

    /// Enough brake torque has to stop the wheel dead and keep it there,
    /// without it ever spinning backwards.
    #[test]
    fn a_wheel_locks_and_stays_locked() {
        let mut w = Wheel::default();
        w.roll_at(30.0);
        let c = contact(30.0, 3000.0);
        let mut out = Patch::default();
        for _ in 0..240 {
            out = step(&mut w, &c, 9_000.0, 1.0 / 480.0);
        }
        println!("under 9 kN.m of brake: omega {:.3}, kappa {:.3}, fx {:.0} N", w.omega, out.kappa, out.fx);
        assert_eq!(w.omega, 0.0, "locked wheel is turning at {}", w.omega);
        assert!(out.kappa < -0.99, "locked wheel reports slip {:.3}", out.kappa);
        assert!(out.fx < 0.0, "locked wheel makes no braking force");
        // And a locked tire is worse than one at the peak -- the reason for ABS.
        let peak = tire::evaluate(3000.0, -0.11, 0.0, 1.0).fx;
        assert!(out.fx > peak, "sliding {:.0} N beat peak {:.0} N", out.fx, peak);
    }

    /// ABS has to hold the wheel near the peak instead of letting it lock.
    #[test]
    fn anti_lock_keeps_the_wheel_turning() {
        let mut w = Wheel::default();
        w.roll_at(30.0);
        let c = contact(30.0, 3000.0);
        let mut kappa = 0.0;
        for _ in 0..480 {
            let braked = anti_lock(9_000.0, kappa);
            kappa = step(&mut w, &c, braked, 1.0 / 480.0).kappa;
        }
        println!("with ABS: omega {:.2}, kappa {kappa:.3}", w.omega);
        assert!(w.omega > 1.0, "ABS still let the wheel lock");
        assert!(kappa > -0.35, "ABS held {kappa:.3} slip, well past the peak");
    }

    /// Traction control has to shut a big slip down and leave a clean launch
    /// alone. It is written in terms of slip in the direction of drive, so
    /// reverse -- where it is needed most, the ratio being nearly fifteen to
    /// one -- is the caller's business.
    #[test]
    fn traction_control_cuts_spin_and_nothing_else() {
        assert!(traction_control(0.6) < 0.01, "a wheel at 60% slip still had throttle");
        assert!(traction_control(0.12) > 0.5 && traction_control(0.12) < 0.9);
        assert_eq!(traction_control(0.04), 1.0, "traction control cut a clean launch");
        assert_eq!(traction_control(-0.5), 1.0, "traction control cut a braking wheel");
    }

    /// Both assists claim to sit just past the tire's longitudinal peak. Read
    /// the peak off the tire model and hold them to it, so that retuning one
    /// without the other is a failing test and not a slower car.
    #[test]
    fn assists_sit_at_the_peak() {
        // A rear wheel's static load, roughly; the peak moves a little with it.
        let fz = 2600.0;
        let d = tire::mu(tire::MU_X0, fz) * fz;
        let b = tire::stiffness_x(fz) / (tire::C_X * d);
        let peak = tire::Z_PEAK_X / b;
        println!("longitudinal peak at {peak:.3} slip; TC from {TC_SLIP}, ABS from {ABS_SLIP}");
        assert!(TC_SLIP > peak && TC_SLIP < peak * 1.4, "traction control does not start just past the peak");
        assert!(ABS_SLIP > peak && ABS_SLIP < peak * 1.5, "ABS does not start just past the peak");
        // And both are all the way in within a tenth of that.
        assert!(traction_control(TC_SLIP + 0.1) < 0.01, "traction control is still open a tenth past its threshold");
        assert!(anti_lock(1.0, -(ABS_SLIP + 0.12)) <= ABS_FLOOR + 1e-6, "ABS is still holding a tenth past its threshold");
    }

    /// Cornering force has to build over a distance rolled, not instantly.
    #[test]
    fn lateral_force_lags_by_a_relaxation_length() {
        let mut w = Wheel::default();
        w.roll_at(30.0);
        let c = Contact { u: 30.0, v: -2.0, fz: 3000.0, grip: 1.0 };
        let h = 1.0 / 480.0;
        let first = step(&mut w, &c, 0.0, h);
        let steady = {
            let mut w2 = w;
            let mut o = first;
            for _ in 0..480 {
                o = step(&mut w2, &c, 0.0, h);
            }
            o
        };
        println!("fy after one substep {:.0} N, settled {:.0} N", first.fy, steady.fy);
        assert!(abs(first.fy) < abs(steady.fy) * 0.35, "force arrived instantly");
        // ~0.45 m at 30 m/s is 15 ms, so it should be there well inside 100 ms.
        let mut w3 = w;
        let mut o = first;
        for _ in 0..48 {
            o = step(&mut w3, &c, 0.0, h);
        }
        assert!(abs(o.fy) > abs(steady.fy) * 0.9, "force still had not arrived after 100 ms");
    }

    /// An undriven wheel under a car that is accelerating must not drag: the
    /// only force it may put on the road is the little that spins its own
    /// inertia up. Without the road-speed prediction in [`Wheel::spin`] the
    /// implicit step charged it for slip it never had, and the answer moved
    /// with the substep.
    #[test]
    fn a_dragged_wheel_costs_only_its_own_inertia() {
        let a = 7.0f32;
        let drag_at = |div: u32| {
            let h = 1.0 / (480.0 * div as f32);
            let mut w = Wheel::default();
            w.roll_at(8.0);
            let mut u = 8.0f32;
            let mut p = Patch::default();
            for _ in 0..480 * div {
                let c = contact(u, 3000.0);
                p = w.patch(&c, h);
                w.spin(&p, 0.0, a, h);
                u += a * h;
            }
            // Subtract rolling resistance, which is real and not the point.
            -p.fx - ROLL_RESIST * 3000.0
        };
        let ideal = INERTIA * a / (RADIUS * RADIUS);
        let coarse = drag_at(1);
        let fine = drag_at(16);
        println!(
            "wheel dragged at {a} m/s^2: {coarse:.0} N at 480 Hz, {fine:.0} N at 7680 Hz, inertia alone {ideal:.0} N"
        );
        assert!(abs(coarse - ideal) < ideal * 0.25 + 5.0, "480 Hz drags {coarse:.0} N, should be {ideal:.0}");
        assert!(abs(coarse - fine) < ideal * 0.25 + 5.0, "answer depends on the substep");
    }
}
