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
//! Stepping that explicitly would need several kHz. [`Wheel::step`] instead
//! takes both couplings implicitly, using each one's slope in the linear
//! region, which is stable at any rate and costs one divide.

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
/// longitudinal curve, which is where a real system aims to sit.
const ABS_SLIP: f32 = 0.13;
const ABS_GAIN: f32 = 6.0;
/// ABS never releases the brake entirely.
const ABS_FLOOR: f32 = 0.12;

/// Slip ratio past which traction control starts closing the throttle.
const TC_SLIP: f32 = 0.14;
const TC_GAIN: f32 = 4.0;

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

/// What this patch did about it.
#[derive(Clone, Copy, Debug, Default)]
pub struct Output {
    /// In the wheel frame: `fx` along its heading, `fy` to its left.
    pub fx: f32,
    pub fy: f32,
    /// Self-aligning moment about the contact patch.
    pub mz: f32,
    pub kappa: f32,
    pub alpha: f32,
    /// Combined slip, 1.0 at the grip peak.
    pub saturation: f32,
}

impl Wheel {
    /// Put the wheel at the speed a free roll at `u` would give it. Used when a
    /// car is placed rather than driven there.
    #[inline]
    pub fn roll_at(&mut self, u: f32) {
        self.omega = u / RADIUS;
        self.fy = 0.0;
    }

    /// Advance this wheel by `h` seconds.
    ///
    /// `drive` is drivetrain torque, `brake` a torque magnitude that always
    /// opposes rotation, and `coupling` the drivetrain's resistance to a change
    /// in this wheel's speed (zero for an undriven wheel).
    pub fn step(&mut self, c: &Contact, drive: f32, brake: f32, coupling: f32, h: f32) -> Output {
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

        // Implicit in both stiff couplings: the tire's slope through the
        // contact patch, and the drivetrain's through the gearing.
        let slope = f.stiffness_x * RADIUS * RADIUS / u_ref + coupling;
        let denom = INERTIA + h * slope;
        let mut omega = self.omega + h * (drive + rolling - f.fx * RADIUS) / denom;

        // The brake can stop the wheel but never reverse it. Clamping at the
        // crossing is what lets a wheel actually lock and stay locked.
        let step = brake * h / denom;
        omega = if abs(omega) <= step {
            0.0
        } else {
            omega - signum(omega) * step
        };
        self.omega = omega;

        Output {
            fx: f.fx,
            fy: self.fy,
            // The lagged force acts on the same trail the unlagged one did.
            mz: -f.trail * self.fy,
            kappa,
            alpha,
            saturation: f.saturation,
        }
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

/// Traction control: close the throttle when the driven wheels light up. Takes
/// the worse of the two rear slips, so a single spinning inside wheel on corner
/// exit is enough to trigger it.
#[inline]
pub fn traction_control(throttle: f32, kappa: f32) -> f32 {
    if kappa <= TC_SLIP || throttle <= 0.0 {
        return throttle;
    }
    throttle * clamp(1.0 - (kappa - TC_SLIP) * TC_GAIN, 0.0, 1.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contact(u: f32, fz: f32) -> Contact {
        Contact { u, v: 0.0, fz, grip: 1.0 }
    }

    /// An undriven, unbraked wheel under a rolling car has to settle at the
    /// speed the road is turning it, and stay there.
    #[test]
    fn a_free_wheel_finds_rolling_speed() {
        let mut w = Wheel::default();
        let c = contact(30.0, 3000.0);
        for _ in 0..480 {
            w.step(&c, 0.0, 0.0, 0.0, 1.0 / 480.0);
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
        let mut out = Output::default();
        for _ in 0..240 {
            out = w.step(&c, 0.0, 9_000.0, 0.0, 1.0 / 480.0);
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
            kappa = w.step(&c, 0.0, braked, 0.0, 1.0 / 480.0).kappa;
        }
        println!("with ABS: omega {:.2}, kappa {kappa:.3}", w.omega);
        assert!(w.omega > 1.0, "ABS still let the wheel lock");
        assert!(kappa > -0.35, "ABS held {kappa:.3} slip, well past the peak");
    }

    /// Drive torque beyond what the tire can take has to show up as the wheel
    /// outrunning the car, and traction control has to notice.
    #[test]
    fn drive_torque_spins_the_wheel_up() {
        let mut w = Wheel::default();
        w.roll_at(10.0);
        let c = contact(10.0, 2600.0);
        let mut out = Output::default();
        for _ in 0..240 {
            out = w.step(&c, 4_000.0, 0.0, 0.0, 1.0 / 480.0);
        }
        println!("4 kN.m into one wheel at 10 m/s: kappa {:.2}", out.kappa);
        assert!(out.kappa > 0.3, "wheel did not spin up: slip {:.2}", out.kappa);
        assert!(traction_control(1.0, out.kappa) < 0.2, "traction control ignored a big slip");
        assert_eq!(traction_control(1.0, 0.05), 1.0, "traction control cut a clean launch");
    }

    /// The assist is written in terms of "slip in the direction of drive", so
    /// the caller can hand it reverse by flipping the sign of both. Reverse is
    /// where it is needed most: the ratio is nearly fifteen to one.
    #[test]
    fn traction_control_works_in_either_direction() {
        // Forwards: wheel outrunning the road.
        assert!(traction_control(1.0, 0.6) < 0.3);
        // Backwards, as `car.rs` presents it: pedal and slip both negated.
        let reverse_slip = -0.6;
        let dir = -1.0;
        assert!(traction_control(0.7, reverse_slip * dir) < 0.3);
        // And it leaves a clean launch alone in both.
        assert_eq!(traction_control(1.0, 0.04), 1.0);
        assert_eq!(traction_control(0.7, -0.04 * dir), 0.7);
    }

    /// Cornering force has to build over a distance rolled, not instantly.
    #[test]
    fn lateral_force_lags_by_a_relaxation_length() {
        let mut w = Wheel::default();
        w.roll_at(30.0);
        let c = Contact { u: 30.0, v: -2.0, fz: 3000.0, grip: 1.0 };
        let h = 1.0 / 480.0;
        let first = w.step(&c, 0.0, 0.0, 0.0, h);
        let steady = {
            let mut w2 = w;
            let mut o = first;
            for _ in 0..480 {
                o = w2.step(&c, 0.0, 0.0, 0.0, h);
            }
            o
        };
        println!("fy after one substep {:.0} N, settled {:.0} N", first.fy, steady.fy);
        assert!(abs(first.fy) < abs(steady.fy) * 0.35, "force arrived instantly");
        // ~0.45 m at 30 m/s is 15 ms, so it should be there well inside 100 ms.
        let mut w3 = w;
        let mut o = first;
        for _ in 0..48 {
            o = w3.step(&c, 0.0, 0.0, 0.0, h);
        }
        assert!(abs(o.fy) > abs(steady.fy) * 0.9, "force still had not arrived after 100 ms");
    }

    /// The whole reason for the implicit step: first gear bolts the engine to
    /// the wheel through a ratio of eleven, and the result still has to sit
    /// still at 480 Hz.
    #[test]
    fn stays_stable_under_a_stiff_drivetrain_coupling() {
        let mut w = Wheel::default();
        w.roll_at(20.0);
        let c = contact(20.0, 2600.0);
        for _ in 0..2400 {
            w.step(&c, 500.0, 0.0, 12_000.0, 1.0 / 480.0);
            assert!(w.omega.is_finite() && abs(w.omega) < 1e4, "wheel blew up: {}", w.omega);
        }
        println!("stable under 12 kN.m/(rad/s) of coupling: omega {:.2}", w.omega);
    }
}
