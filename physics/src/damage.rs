//! Crush: what a car keeps after a crash, and what it costs.
//!
//! A car is not a billiard ball, and the difference is the whole of this file.
//! Steel that folds does not give the energy back, so a real car-to-car impact
//! is *mostly plastic* -- and the harder the hit, the more plastic it gets. A
//! constant coefficient of restitution gets that exactly backwards at the speeds
//! where it matters: it makes a 100 km/h shunt bounce like a 5 km/h one.
//!
//! The model here is the one accident reconstruction uses, which is convenient
//! because it is also the one that gives us the damage for free. Campbell's
//! observation is that residual crush depth is *linear* in impact speed:
//!
//! ```text
//!     v = B0 + B1 * C
//! ```
//!
//! with [`B0`] the speed below which the car springs back undamaged and [`B1`]
//! the metres of permanent crush per metre-per-second beyond it. Integrating the
//! force that implies gives the energy a given crush depth has absorbed
//! ([`energy_for`]), and inverting that gives the crush a given amount of
//! absorbed energy produces ([`crush_for`]). Those two are exact inverses, which
//! is what lets damage accumulate honestly: two 30 kJ hits leave the same dent
//! as one 60 kJ hit, and the structure gets stiffer as it folds, so the second
//! one adds less than the first.
//!
//! Restitution falls out of the same statement rather than being a second,
//! independent knob. If everything up to [`B0`] is elastic and everything past
//! it goes into bending metal, then the fraction of energy returned is
//! `(B0/v)^2`, so `e = B0/v` -- see [`restitution`]. That is not a curve fit, but
//! it lands within a few hundredths of the published ones: 0.20 at 10 m/s
//! against Antonetti's 0.24, 0.10 at 20 m/s against 0.10.
//!
//! What damage then *does* to the car is the one part of this file that is
//! tuning rather than physics. The mechanisms are real -- a folded nose has no
//! splitter left, a bent wishbone will not point its wheel straight, a radiator
//! full of its own condenser does not cool -- but their magnitudes are picked so
//! that a damaged car is a worse car to drive and still a car.

use crate::car::MASS;
use crate::math::{clamp, max, min, sqrt};

/// Impact speed a car shrugs off entirely, m/s. Below this the bumper springs
/// back and there is nothing left to see -- about 7 km/h, which is roughly the
/// old 5 mph bumper standard and not a coincidence.
pub const B0: f32 = 2.0;

/// Metres per second of impact speed per metre of residual crush. Around 28 for
/// a passenger car across the full-frontal range, which is what puts a 54 km/h
/// barrier hit at roughly half a metre of nose.
pub const B1: f32 = 28.0;

/// Crush past which the structure is spent, metres. The model would happily
/// keep folding; a car that has lost this much of its nose has nothing left to
/// fold, so beyond here further energy goes somewhere this simulation does not
/// model.
pub const MAX_CRUSH: f32 = 0.45;

/// Force the structure resists with at zero crush, newtons. `m * B0 * B1`.
const A: f32 = MASS * B0 * B1;
/// How fast that force rises with crush, N/m. `m * B1^2`.
const B: f32 = MASS * B1 * B1;

/// Ceiling on the coefficient of restitution, for impacts too gentle to bend
/// anything. Without it [`restitution`] would exceed 1 below [`B0`].
const E_MAX: f32 = 0.45;

/// Coefficient of restitution at a given approach speed, m/s.
///
/// Everything up to [`B0`] is elastic and everything past it is not, so the
/// returned fraction of the energy is `(B0/v)^2` and the returned fraction of
/// the *speed* is the root of that.
pub fn restitution(approach: f32) -> f32 {
    if approach <= B0 {
        return E_MAX;
    }
    min(E_MAX, B0 / approach)
}

/// Energy a given depth of crush has absorbed, joules.
pub fn energy_for(crush: f32) -> f32 {
    A * crush + 0.5 * B * crush * crush
}

/// Depth of crush a given amount of absorbed energy produces, metres. The
/// inverse of [`energy_for`], by the quadratic formula.
pub fn crush_for(energy: f32) -> f32 {
    if energy <= 0.0 {
        return 0.0;
    }
    (sqrt(A * A + 2.0 * B * energy) - A) / B
}

/// Add `energy` joules of absorbed impact to a panel already crushed by
/// `crush` metres, and return the new depth.
///
/// Through the energy rather than by adding depths, so the structure stiffens
/// as it folds: the second half of a dent costs far more than the first.
pub fn accumulate(crush: f32, energy: f32) -> f32 {
    min(MAX_CRUSH, crush_for(energy_for(crush) + energy))
}

/// How much of the energy a *sliding* contact destroys reaches the shape of the
/// panel rather than going into heat and paint.
const SCRAPE: f32 = 0.15;

/// How deep a scrape can get on its own, metres.
///
/// The ceiling matters more than the rate. Dragging a car down a barrier for a
/// hundred metres destroys an enormous amount of energy -- comparable with a
/// serious impact -- but it does it against the *outside* of the car. Paint,
/// then panel, then it is rubbing on structure that does not move. A graze
/// takes your flank off; it does not take your width off, and without this the
/// bots write themselves off inside a lap on contact nobody would even file a
/// report about.
pub const MAX_SCRAPE: f32 = 0.10;

/// Wear a panel by sliding along something, and return the new depth.
///
/// Same curve as [`accumulate`] -- it is the same metal -- under a much lower
/// ceiling, and a panel already folded past that ceiling by a real impact is
/// left exactly as it was rather than being polished back out.
pub fn scuff(crush: f32, energy: f32) -> f32 {
    if crush >= MAX_SCRAPE {
        return crush;
    }
    min(MAX_SCRAPE, accumulate(crush, energy * SCRAPE))
}

/// How much of a two-car impact's energy each car absorbs. Equal cars, equal
/// structures, so they share it.
pub const SHARE_CAR: f32 = 0.5;
/// The same against a barrier. Not one, because a race barrier is built to
/// deform -- but most of it still comes out of the car.
pub const SHARE_WALL: f32 = 0.7;

// --- what it costs to drive ---------------------------------------------

/// Downforce lost per metre of crush at that end of the car. A folded nose has
/// no splitter and a folded tail has no diffuser, and both were making more
/// than their share.
const DOWNFORCE_LOSS: f32 = 1.6;
/// Extra drag per metre of crush, as a fraction. A caved-in panel is a brick.
const DRAG_GAIN: f32 = 1.2;
/// Steering lock lost per metre of front crush. The rack survives; the arms
/// and the uprights it pushes on do not.
const STEER_LOSS: f32 = 0.8;
/// Radians of permanent steer per metre of *asymmetric* front crush. A bent
/// wishbone is shorter than it was, which toes its wheel in, and the car pulls
/// towards the side that took the hit.
const STEER_PULL: f32 = 0.16;
/// Engine torque lost per metre of front crush. Radiator, intercooler, and the
/// air that used to reach both.
const POWER_LOSS: f32 = 1.1;
/// Grip lost per metre of crush at a wheel's own corner. Bodywork on rubber,
/// and a wheel that is no longer pointing where the other three are.
const GRIP_LOSS: f32 = 0.55;

/// Residual crush, in metres, on each face of the car.
///
/// Four faces rather than four corners: a corner impact crushes two of them at
/// once in proportion to how square it was, which reconstructs the corner
/// without the state to store it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Damage {
    pub front: f32,
    pub rear: f32,
    pub left: f32,
    pub right: f32,
}

impl Damage {
    /// Overall condition, 0 (straight) to 1 (written off). What the HUD shows
    /// and what the renderer smokes about.
    pub fn severity(&self) -> f32 {
        let worst = max(max(self.front, self.rear), max(self.left, self.right));
        clamp(worst / MAX_CRUSH, 0.0, 1.0)
    }

    /// Multiplier on the downforce each axle still makes.
    pub fn downforce(&self) -> (f32, f32) {
        (
            max(1.0 - DOWNFORCE_LOSS * self.front, 0.15),
            max(1.0 - DOWNFORCE_LOSS * self.rear, 0.15),
        )
    }

    /// Multiplier on drag. Bent panels at both ends both cost.
    pub fn drag(&self) -> f32 {
        1.0 + DRAG_GAIN * (self.front + self.rear)
    }

    /// Multiplier on the steering lock still available.
    pub fn steer_lock(&self) -> f32 {
        max(1.0 - STEER_LOSS * self.front, 0.25)
    }

    /// Permanent steer angle the damage has left in the geometry, radians.
    /// Positive steers left, so a car with a crushed left front pulls left.
    pub fn steer_pull(&self) -> f32 {
        STEER_PULL * (self.left - self.right)
    }

    /// Multiplier on engine torque.
    pub fn power(&self) -> f32 {
        max(1.0 - POWER_LOSS * self.front, 0.35)
    }

    /// Multiplier on the grip available at one wheel, given the crush at the
    /// end of the car it sits at and on the side it sits on. The worse of the
    /// two: one bent corner is one bent corner however it got that way.
    pub fn wheel_grip(&self, end: f32, side: f32) -> f32 {
        max(1.0 - GRIP_LOSS * max(end, side), 0.4)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::abs;

    /// The two halves of the crush model have to be exact inverses, or damage
    /// stops accumulating honestly.
    #[test]
    fn energy_and_crush_invert_each_other() {
        for c in [0.0f32, 0.01, 0.05, 0.2, 0.45] {
            let back = crush_for(energy_for(c));
            println!("{c:.3} m -> {:.1} kJ -> {back:.3} m", energy_for(c) / 1000.0);
            assert!(abs(back - c) < 1e-4, "{c} came back as {back}");
        }
    }

    /// Campbell's relation is the definition of the model, so it had better
    /// come back out of it: a car hitting a rigid barrier at `v` should be left
    /// with `(v - B0) / B1` metres of crush.
    #[test]
    fn barrier_crush_follows_the_speed_it_came_from() {
        println!("  barrier speed   predicted crush   model crush");
        for v in [4.0f32, 7.5, 11.0, 15.0] {
            // All of the kinetic energy above the elastic part goes into it.
            let e = 0.5 * MASS * (v * v - B0 * B0);
            let c = crush_for(e);
            let campbell = (v - B0) / B1;
            println!("  {:5.1} m/s      {campbell:.3} m           {c:.3} m", v);
            assert!(abs(c - campbell) < 0.01, "{c:.3} m against Campbell's {campbell:.3} m");
        }
    }

    /// The published empirical fits are exponential in speed. Ours is a
    /// hyperbola derived from the crush model, and the point of this test is
    /// that over the range a race produces they are the same curve.
    #[test]
    fn restitution_tracks_the_measured_curve() {
        println!("  approach   ours   Antonetti");
        for v in [4.0f32, 6.0, 10.0, 15.0, 20.0, 30.0] {
            let ours = restitution(v);
            // 0.574 * exp(-0.0886 v), the fit to staged collisions, evaluated
            // here with the standard library because this is a test.
            let measured = 0.574 * (-0.0886 * v).exp();
            println!("  {v:5.1}     {ours:.3}   {measured:.3}");
            assert!(
                abs(ours - measured) < 0.09,
                "at {v} m/s: {ours:.3} against a measured {measured:.3}"
            );
        }
        assert!(restitution(1.0) > restitution(20.0) * 4.0, "hard hits are not plastic");
    }

    /// Damage has to saturate, and it has to get harder to do as it goes.
    #[test]
    fn crush_stiffens_as_it_accumulates() {
        let first = accumulate(0.0, 20_000.0);
        let second = accumulate(first, 20_000.0) - first;
        println!("first 20 kJ: {first:.3} m, second 20 kJ adds {second:.3} m");
        assert!(second < first, "the structure did not stiffen");
        assert!(accumulate(MAX_CRUSH, 1e7) <= MAX_CRUSH);
    }

    /// A graze must not write a car off the way a square hit does, however
    /// long it goes on for.
    #[test]
    fn a_scrape_wears_a_panel_but_never_folds_one() {
        let square = accumulate(0.0, 50_000.0);
        let mut graze = 0.0f32;
        // Ten seconds of barrier at racing speed, in 50 kJ helpings.
        for _ in 0..40 {
            graze = scuff(graze, 50_000.0);
        }
        println!("50 kJ head-on -> {square:.3} m; 2 MJ of barrier -> {graze:.3} m");
        assert!(square > 0.25, "a square hit left {square:.3} m");
        assert!(graze <= MAX_SCRAPE + 1e-6, "a graze folded the car to {graze:.3} m");
        // And a panel that is already folded is not repaired by rubbing it.
        assert_eq!(scuff(0.3, 1e6), 0.3);
    }

    #[test]
    fn a_written_off_car_is_still_a_car() {
        let d = Damage { front: MAX_CRUSH, rear: MAX_CRUSH, left: MAX_CRUSH, right: MAX_CRUSH };
        println!(
            "at maximum crush: steer x{:.2}, power x{:.2}, grip x{:.2}, drag x{:.2}",
            d.steer_lock(),
            d.power(),
            d.wheel_grip(MAX_CRUSH, MAX_CRUSH),
            d.drag()
        );
        assert!(d.severity() == 1.0);
        assert!(d.steer_lock() > 0.2 && d.power() > 0.3 && d.wheel_grip(1.0, 1.0) > 0.3);
        // Symmetric damage pulls nowhere.
        assert_eq!(d.steer_pull(), 0.0);
    }
}
