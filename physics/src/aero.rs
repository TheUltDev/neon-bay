//! Aerodynamics: drag, downforce, and the price the one charges for the other.
//!
//! Both scale with the square of speed, which is what makes a fast corner a
//! different problem from a slow one. At 30 km/h the wings do nothing and the
//! car has only its own weight to grip with; at 250 km/h they are pressing an
//! extra third of a car onto the road and it will take far more speed through a
//! bend than the tires alone could ever justify.
//!
//! Downforce is not free. A wing making lift makes induced drag with it, so
//! [`INDUCED`] charges the engine for the grip -- which is why the top speed
//! that falls out of [`crate::drivetrain`] is well short of what the gearing
//! alone would allow.

use crate::car::G;
use crate::damage::Damage;

/// `0.5 * rho * Cd * A`, so drag is just this times `v^2`. About Cd 0.27 on
/// 2.05 m^2 of frontal area, before the induced part below.
pub const DRAG_AREA: f32 = 0.34;

/// `0.5 * rho * Cl * A` for the whole car. At 76 m/s this is pressing down with
/// nearly 40% of the car's weight again.
pub const LIFT_AREA: f32 = 0.75;

/// Share of downforce carried by the front axle. Below 50% because a car that
/// gains grip at the front faster than at the rear as it speeds up turns into
/// high-speed oversteer, and that is a bad way to find out about aero balance.
pub const BALANCE_F: f32 = 0.42;

/// Drag per newton of downforce -- a lift-to-drag ratio of about 4.5, which is
/// what a whole car manages once the body is counted and not just the wing.
const INDUCED: f32 = 0.22;

/// Rolling resistance coefficient. Applied at the contact patch as a torque on
/// the wheel, not as a body force, so it scales with the load each tire is
/// actually carrying -- downforce included.
pub const ROLL_RESIST: f32 = 0.014;

/// What the air is doing to the car at this speed.
#[derive(Clone, Copy, Debug, Default)]
pub struct Aero {
    /// Downforce on the front axle, newtons.
    pub down_f: f32,
    /// Downforce on the rear axle, newtons.
    pub down_r: f32,
    /// Drag, newtons. Always positive; the caller applies it against travel.
    pub drag: f32,
}

impl Aero {
    /// The same air, over a car that has been in an accident.
    ///
    /// A splitter that is folded under the car is not making downforce, a
    /// diffuser with the rear crash structure in it is not either, and both of
    /// them are now making drag they were not making before. This is where a
    /// damaged car stops being able to lean on its aero in the fast corners,
    /// which is exactly where it was leaning on it hardest.
    pub fn crushed(self, d: &Damage) -> Aero {
        let (kf, kr) = d.downforce();
        Aero {
            down_f: self.down_f * kf,
            down_r: self.down_r * kr,
            drag: self.drag * d.drag(),
        }
    }
}

/// Resolve the aero loads at a given speed.
pub fn evaluate(speed: f32) -> Aero {
    let q = speed * speed;
    let down = LIFT_AREA * q;
    Aero {
        down_f: down * BALANCE_F,
        down_r: down * (1.0 - BALANCE_F),
        drag: DRAG_AREA * q + INDUCED * down,
    }
}

/// Speed at which downforce equals the car's own weight. Not used by the
/// simulation -- it is here because it is the one number that says how much
/// aero a car has, and the tests quote it.
pub fn neutral_speed() -> f32 {
    crate::math::sqrt(crate::car::MASS * G / LIFT_AREA)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn downforce_is_meaningful_but_not_a_racing_car() {
        let v = neutral_speed();
        println!(
            "downforce equals weight at {v:.0} m/s ({:.0} km/h)",
            v * 3.6
        );
        // A road car never gets there; a Le Mans prototype does it by 50 m/s.
        assert!(v > 100.0 && v < 145.0, "aero level is off: neutral at {v:.0} m/s");

        let a = evaluate(60.0);
        let share = (a.down_f + a.down_r) / (crate::car::MASS * G);
        println!(
            "at 60 m/s: {:.0} N front, {:.0} N rear, {:.0} N drag ({:.0}% of weight)",
            a.down_f,
            a.down_r,
            a.drag,
            share * 100.0
        );
        assert!(share > 0.15 && share < 0.40, "downforce at 60 m/s is {share:.2} of weight");
    }

    #[test]
    fn drag_is_dominated_by_the_body_at_speed() {
        let a = evaluate(70.0);
        let induced = a.drag - DRAG_AREA * 70.0 * 70.0;
        println!("at 70 m/s: {:.0} N drag, of which {induced:.0} N is the price of downforce", a.drag);
        assert!(induced > 0.0 && induced < a.drag * 0.5);
    }

    #[test]
    fn nothing_happens_at_a_standstill() {
        let a = evaluate(0.0);
        assert_eq!(a.drag, 0.0);
        assert_eq!(a.down_f, 0.0);
        assert_eq!(a.down_r, 0.0);
    }
}
