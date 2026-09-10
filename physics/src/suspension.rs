//! Load transfer: what each of the four tires is actually being pressed into
//! the road with, at this instant.
//!
//! [`crate::tire`] only cares about one number from this file, `Fz`, but it
//! cares enormously -- grip is sub-linear in load, so *how* the total weight is
//! distributed across four patches decides the car's balance, not just how much
//! total grip there is. Two things follow, and both are the point of modelling
//! this at all rather than filtering an acceleration:
//!
//! * **Balance is a setup parameter.** Lateral transfer is split front to rear
//!   by the *roll stiffness distribution*, [`K_ROLL_F`] against [`K_ROLL_R`].
//!   Stiffen the front bar and the front axle takes more of the transfer, its
//!   inside wheel goes lighter, the pair loses grip to load sensitivity, and
//!   the car understeers. That is how a real anti-roll bar works and it falls
//!   straight out of the arithmetic below.
//! * **Transfer takes time.** Roll and pitch are second-order systems with real
//!   inertia, stiffness and damping, so load moves across the car over roughly
//!   a tenth of a second rather than instantly. Lift off mid-corner and the
//!   rear unloads *progressively*; that lag is what makes trail-braking and
//!   lift-off oversteer feel like techniques rather than switches.
//!
//! Signs: `roll` is positive leaning right, which is what a left-hand corner
//! does; `pitch` is positive nose-down, which is what braking does.

use crate::car::{CG_HEIGHT, MASS, TRACK_F, TRACK_R, WHEELBASE};
use crate::math::{clamp, max};

/// Mass carried on the springs, and the mass that is not (wheels, uprights,
/// roughly half of each arm). Only the sprung part rolls.
pub const SPRUNG_MASS: f32 = 1062.0;
/// Per axle, i.e. two wheels' worth.
pub const UNSPRUNG_AXLE: f32 = 44.0;
/// Height of the unsprung mass. It is the wheel, so this is the tire radius.
const H_UNSPRUNG: f32 = 0.32;

/// Sprung CG height. Chosen so `MASS * CG_HEIGHT` equals the sprung and
/// unsprung moments added up -- `mass_budget_is_consistent` checks it.
const H_SPRUNG: f32 = 0.5365;

/// Roll centre heights. The rear is higher, which is conventional and which
/// hands the rear axle a little more of the geometric transfer.
const RC_F: f32 = 0.055;
const RC_R: f32 = 0.085;
/// Roll axis height directly under the sprung CG.
const ROLL_AXIS_Z: f32 = RC_F + (RC_R - RC_F) * (crate::car::LF / WHEELBASE);
/// Moment arm of the sprung mass about the roll axis. This is the lever that
/// makes the body roll, and everything elastic scales with it.
const H_ROLL: f32 = H_SPRUNG - ROLL_AXIS_Z;

/// Roll stiffness per axle, N.m per radian: springs in roll plus the anti-roll
/// bar. Their *ratio* is the main balance adjustment on the car; their sum sets
/// how far it leans.
pub const K_ROLL_F: f32 = 52_000.0;
pub const K_ROLL_R: f32 = 42_000.0;
const K_ROLL: f32 = K_ROLL_F + K_ROLL_R;
/// Roll inertia of the sprung mass about the roll axis.
const I_ROLL: f32 = 320.0;
/// Roll damping, from the dampers working in roll. About 0.55 of critical --
/// underdamped enough to have a transient at all, damped enough not to wallow.
const C_ROLL: f32 = 6_030.0;

/// Pitch stiffness and inertia, same idea in the other plane.
const K_PITCH: f32 = 139_000.0;
const I_PITCH: f32 = 1_400.0;
const C_PITCH: f32 = 19_530.0;
/// Share of longitudinal transfer that goes through the suspension *links*
/// rather than the springs -- anti-dive and anti-squat geometry, plus the
/// unsprung mass. That share is instant; the rest waits for the body to pitch.
const ANTI_SHARE: f32 = 0.25;
/// Moment arm driving the pitch DOF, calibrated so the elastic path carries
/// exactly `1 - ANTI_SHARE` of the rigid-body answer at steady state.
/// `transfer_totals_match_a_rigid_body` is the check.
const H_PITCH: f32 = (1.0 - ANTI_SHARE) * MASS * CG_HEIGHT / SPRUNG_MASS;

/// Sprung mass over each axle.
const SPRUNG_F: f32 = SPRUNG_MASS * crate::car::LR / WHEELBASE;
const SPRUNG_R: f32 = SPRUNG_MASS * crate::car::LF / WHEELBASE;

/// Attitude cannot run away, however hard the car is hit.
const MAX_ANGLE: f32 = 0.30;
const MAX_RATE: f32 = 6.0;

/// Vertical load on each contact patch, newtons. Never negative: a wheel that
/// has run out of load has left the road, and [`crate::tire::evaluate`] gives
/// it no grip at all.
#[derive(Clone, Copy, Debug, Default)]
pub struct Loads {
    pub fl: f32,
    pub fr: f32,
    pub rl: f32,
    pub rr: f32,
}

/// The body's roll and pitch, and how fast they are changing.
#[derive(Clone, Copy, Debug, Default)]
pub struct Attitude {
    pub roll: f32,
    pub roll_rate: f32,
    pub pitch: f32,
    pub pitch_rate: f32,
}

impl Attitude {
    /// Advance roll and pitch by `h` seconds under the body-frame accelerations
    /// the tires just produced.
    ///
    /// Semi-implicit in the damping term, which lets a 480 Hz substep hold a
    /// 2.7 Hz roll mode with plenty of margin and never rings.
    pub fn integrate(&mut self, ax: f32, ay: f32, h: f32) {
        let roll_acc = (SPRUNG_MASS * ay * H_ROLL - K_ROLL * self.roll) / I_ROLL;
        self.roll_rate = (self.roll_rate + roll_acc * h) / (1.0 + h * C_ROLL / I_ROLL);
        self.roll = clamp(self.roll + self.roll_rate * h, -MAX_ANGLE, MAX_ANGLE);
        self.roll_rate = clamp(self.roll_rate, -MAX_RATE, MAX_RATE);

        let pitch_acc = (-SPRUNG_MASS * ax * H_PITCH - K_PITCH * self.pitch) / I_PITCH;
        self.pitch_rate = (self.pitch_rate + pitch_acc * h) / (1.0 + h * C_PITCH / I_PITCH);
        self.pitch = clamp(self.pitch + self.pitch_rate * h, -MAX_ANGLE, MAX_ANGLE);
        self.pitch_rate = clamp(self.pitch_rate, -MAX_RATE, MAX_RATE);
    }
}

/// Vertical load on each wheel.
///
/// `ay` and `ax` are the body-frame accelerations; `df_f` and `df_r` the aero
/// downforce already resolved onto each axle. Three paths carry load across the
/// car and all three are here: the elastic one through the springs (which lags,
/// because it is driven by the body's actual attitude), and the geometric and
/// unsprung ones through the links and the wheels themselves (which do not).
pub fn loads(att: &Attitude, ax: f32, ay: f32, df_f: f32, df_r: f32) -> Loads {
    // --- fore and aft ------------------------------------------------------
    let elastic_long = K_PITCH * att.pitch / WHEELBASE;
    let direct_long = ANTI_SHARE * MASS * -ax * CG_HEIGHT / WHEELBASE;
    let d_long = elastic_long + direct_long;

    let axle_f = MASS * crate::car::G * (crate::car::LR / WHEELBASE) + d_long + df_f;
    let axle_r = MASS * crate::car::G * (crate::car::LF / WHEELBASE) - d_long + df_r;

    // --- side to side ------------------------------------------------------
    // The elastic term is split by roll stiffness, which is why the bars are a
    // balance adjustment. The other two are pure geometry and mass.
    let d_lat_f = (K_ROLL_F * att.roll + SPRUNG_F * ay * RC_F + UNSPRUNG_AXLE * ay * H_UNSPRUNG)
        / TRACK_F;
    let d_lat_r = (K_ROLL_R * att.roll + SPRUNG_R * ay * RC_R + UNSPRUNG_AXLE * ay * H_UNSPRUNG)
        / TRACK_R;

    // Left turn: ay and roll are positive, and the right-hand wheels take it.
    Loads {
        fl: max(axle_f * 0.5 - d_lat_f, 0.0),
        fr: max(axle_f * 0.5 + d_lat_f, 0.0),
        rl: max(axle_r * 0.5 - d_lat_r, 0.0),
        rr: max(axle_r * 0.5 + d_lat_r, 0.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::car::G;
    use crate::math::abs;

    #[test]
    fn mass_budget_is_consistent() {
        let split = SPRUNG_MASS + 2.0 * UNSPRUNG_AXLE;
        assert!(abs(split - MASS) < 1.0, "sprung + unsprung is {split}, not {MASS}");
        let moment = SPRUNG_MASS * H_SPRUNG + 2.0 * UNSPRUNG_AXLE * H_UNSPRUNG;
        let whole = MASS * CG_HEIGHT;
        println!("cg moment: parts {moment:.1}, whole {whole:.1}");
        assert!(abs(moment - whole) < 3.0, "sprung CG height is inconsistent with CG_HEIGHT");
    }

    /// However the transfer is routed internally, the totals have to come out
    /// where a rigid body on four contact patches would put them.
    #[test]
    fn transfer_totals_match_a_rigid_body() {
        // Settle the attitude under a steady 1 g in each plane.
        let settle = |ax: f32, ay: f32| {
            let mut a = Attitude::default();
            for _ in 0..8000 {
                a.integrate(ax, ay, 1.0 / 480.0);
            }
            a
        };

        let braking = settle(-G, 0.0);
        let l = loads(&braking, -G, 0.0, 0.0, 0.0);
        let front = l.fl + l.fr;
        let moved = front - MASS * G * (crate::car::LR / WHEELBASE);
        let rigid = MASS * G * CG_HEIGHT / WHEELBASE;
        println!(
            "1 g braking: pitch {:.2} deg, front axle gains {moved:.0} N (rigid {rigid:.0} N)",
            braking.pitch * 180.0 / core::f32::consts::PI
        );
        assert!(abs(moved - rigid) < rigid * 0.03, "longitudinal transfer is off");

        let cornering = settle(0.0, G);
        let l = loads(&cornering, 0.0, G, 0.0, 0.0);
        let moved_f = (l.fr - l.fl) * 0.5;
        let moved_r = (l.rr - l.rl) * 0.5;
        let rigid_lat = MASS * G * CG_HEIGHT / ((TRACK_F + TRACK_R) * 0.5) * 0.5;
        println!(
            "1 g cornering: roll {:.2} deg, transfer f {moved_f:.0} N r {moved_r:.0} N (rigid total {:.0} N)",
            cornering.roll * 180.0 / core::f32::consts::PI,
            rigid_lat * 2.0
        );
        assert!(
            abs(moved_f + moved_r - rigid_lat * 2.0) < rigid_lat * 0.06,
            "lateral transfer is off"
        );
        // And the split follows the bars, which is the whole point.
        assert!(moved_f > moved_r, "front bar is stiffer but takes less transfer");
    }

    /// Roll should reach a couple of degrees at 1 g -- a car that leans ten
    /// degrees is a bus, and one that leans none has no transient to lag.
    #[test]
    fn body_leans_a_realistic_amount() {
        let mut a = Attitude::default();
        for _ in 0..8000 {
            a.integrate(-G, G, 1.0 / 480.0);
        }
        let roll_deg = a.roll * 180.0 / core::f32::consts::PI;
        let pitch_deg = a.pitch * 180.0 / core::f32::consts::PI;
        println!("at 1 g: roll {roll_deg:.2} deg, pitch {pitch_deg:.2} deg");
        assert!(roll_deg > 1.5 && roll_deg < 4.5, "roll of {roll_deg:.2} deg at 1 g");
        assert!(pitch_deg > 0.8 && pitch_deg < 3.0, "pitch of {pitch_deg:.2} deg at 1 g");
    }

    /// Load has to take time to move. If it were instant, lifting off mid
    /// corner would be a step change rather than something a driver can meter.
    #[test]
    fn transfer_lags_the_input() {
        let mut a = Attitude::default();
        let h = 1.0 / 480.0;
        let mut reached = 0;
        for i in 1..=480 {
            a.integrate(0.0, G, h);
            if reached == 0 && a.roll > 0.632 * (SPRUNG_MASS * G * H_ROLL / K_ROLL) {
                reached = i;
            }
        }
        let ms = reached as f32 * h * 1000.0;
        println!("roll reaches 63% of steady state in {ms:.0} ms");
        assert!(ms > 20.0 && ms < 250.0, "roll transient is {ms:.0} ms");
    }

    /// At high enough lateral g the inside wheels have to come off the ground.
    /// The tire model returns nothing for them, which is correct and is why
    /// [`Loads`] is clamped rather than allowed to go negative.
    #[test]
    fn inside_wheel_lifts_under_enough_load() {
        let mut a = Attitude::default();
        let hard = 1.9 * G;
        for _ in 0..8000 {
            a.integrate(0.0, hard, 1.0 / 480.0);
        }
        let l = loads(&a, 0.0, hard, 0.0, 0.0);
        println!("at 1.9 g: fl {:.0} fr {:.0} rl {:.0} rr {:.0}", l.fl, l.fr, l.rl, l.rr);
        assert!(l.fl <= 0.0, "front inside wheel still loaded at 1.9 g");
        assert!(l.fr > 0.0 && l.rr > 0.0, "outside wheels lost their load");
    }
}
