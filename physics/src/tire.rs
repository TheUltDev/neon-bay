//! The tire: a Pacejka Magic Formula, evaluated per wheel, per substep.
//!
//! This is the piece that decides what the car actually does. Everything else
//! -- the drivetrain, the suspension, the aero -- exists to put a vertical load
//! and a slip on four contact patches, and then this file answers with a force.
//!
//! The formula itself is Pacejka's:
//!
//! ```text
//! F(s) = D sin(C atan(B s - E (B s - atan(B s))))
//! ```
//!
//! `D` is the peak, `C` decides how far the curve falls off past it, `B` sets
//! how quickly it gets there and `E` bends the shape around the peak. Three
//! properties of real rubber fall out of it that a linear `F = -C_alpha * alpha`
//! model cannot produce at all, and all three are things a driver feels:
//!
//! * **There is a peak.** Grip rises with slip, tops out, and then *falls*.
//!   Past the peak, asking for more slip gives you less force. That is what
//!   makes a slide a slide rather than just a larger cornering force.
//! * **Sliding grip is lower than peak grip.** `sin(C pi/2)` is the ratio, so
//!   [`C_X`] and [`C_Y`] set it directly: a locked tire keeps 71% of its peak,
//!   a fully sideways one 85%. This is the entire reason ABS is worth having.
//! * **Load sensitivity.** Doubling the vertical load does *not* double the
//!   grip -- [`MU_LOAD`] takes some of it back. That single coefficient is why
//!   weight transfer changes a car's balance, why an inside wheel going light
//!   costs more than the outside one gains, and why the roll stiffness split in
//!   [`crate::suspension`] is a handling adjustment rather than a comfort one.
//!
//! Everything here is plain `f32` and every transcendental comes from
//! [`crate::math`], so the sidecar (x86-64) and the browser (wasm32) produce
//! identical bits. See the crate docs.

use crate::math::{atan, clamp, cos, sin, sqrt};

/// Reference vertical load. The coefficients below are quoted here and scale
/// away from it. Roughly the static load on one front wheel.
pub const FZ0: f32 = 3000.0;

/// Peak friction coefficient at [`FZ0`].
pub const MU_X0: f32 = 1.62;
pub const MU_Y0: f32 = 1.55;
/// Load sensitivity: the fraction of peak `mu` given up when the load rises by
/// [`FZ0`]. Small number, enormous consequences -- see the module docs.
pub const MU_LOAD: f32 = 0.18;
/// Floor on the load-sensitivity falloff, so a wildly overloaded tire keeps
/// *some* grip instead of crossing zero and driving the car backwards.
const MU_FLOOR: f32 = 0.45;

/// Shape factor. `sin(C * PI/2)` is the sliding-to-peak ratio: 0.707 for the
/// longitudinal curve, 0.853 for the lateral one. Tires really do lose more
/// under lock-up than they do sideways.
pub const C_X: f32 = 1.50;
pub const C_Y: f32 = 1.35;
/// Curvature factor. Negative sharpens the peak and brings it in to a realistic
/// slip; near +1 the curve goes flat and the peak wanders off to nonsense.
pub const E_X: f32 = -1.50;
pub const E_Y: f32 = -2.00;

/// Slip stiffness, as `BCD = K1 sin(2 atan(Fz / K2))`.
///
/// Not a constant times load: a carcass stiffens with load, more and more
/// slowly, and eventually gives up. `K1` is the ceiling and `K2` sets where the
/// knee is. The operating range here (1-6 kN) sits on the rising part, so
/// cornering stiffness climbs with load while `mu` falls -- which is exactly
/// the pair of trends that makes peak slip angle grow with load.
pub const K_X1: f32 = 125_000.0;
pub const K_X2: f32 = 5_000.0;
pub const K_Y1: f32 = 78_000.0;
pub const K_Y2: f32 = 5_200.0;

/// Where the magic formula peaks, in units of `B * s`.
///
/// Solves `(1 - E) z + E atan(z) = tan(PI / 2C)`, which is the `z = B s` that
/// drives the outer `sin` to 1. Constant per `(C, E)` pair, so it is baked here
/// rather than solved every substep; `peak_matches_the_curve` re-derives both
/// numerically and fails if a coefficient above is edited without updating them.
pub const Z_PEAK_X: f32 = 1.224_4;
pub const Z_PEAK_Y: f32 = 1.408_3;

/// Pneumatic trail at zero slip, meters. The contact patch loads up behind its
/// centre, so the lateral force acts aft of the wheel and yaws the car.
const TRAIL_0: f32 = 0.038;
/// Mechanical trail from caster. Unlike the pneumatic part it does not collapse
/// with slip -- it is geometry, not rubber.
const TRAIL_MECH: f32 = 0.022;
/// Shape of the trail's collapse with slip. At the grip peak the trail is
/// already near zero; past it, it inverts.
const TRAIL_B: f32 = 1.40;
const TRAIL_C: f32 = 1.60;

/// Below this the slip vector is too small to have a direction worth using.
const SLIP_EPS: f32 = 1e-6;

/// What one contact patch is doing.
#[derive(Clone, Copy, Debug, Default)]
pub struct Force {
    /// Longitudinal, along the wheel's heading. Positive drives the car forward.
    pub fx: f32,
    /// Lateral, to the wheel's left.
    pub fy: f32,
    /// Self-aligning moment about the contact patch.
    pub mz: f32,
    /// Distance behind the wheel centre that the lateral force acts at. Handed
    /// back so that a caller applying a *lagged* lateral force can put it on
    /// the same lever, instead of carrying a second relaxation state for the
    /// moment. See [`crate::wheel`].
    pub trail: f32,
    /// `d fx / d kappa` in the linear region, which the wheel spin integrator
    /// needs to stay stable at a sane step size. See [`crate::wheel`].
    pub stiffness_x: f32,
    /// Combined slip normalized so that 1.0 is the grip peak. Drives tire smoke
    /// and marks, and it is what the traction and brake assists watch.
    pub saturation: f32,
}

/// The magic formula itself.
#[inline]
fn magic(z: f32, c: f32, d: f32, e: f32) -> f32 {
    // z is already `B * s`; folding B in at the call site saves a multiply and
    // keeps the peak constants above in terms of z.
    let arg = z - e * (z - atan(z));
    d * sin(c * atan(arg))
}

/// Peak friction at a given load, after load sensitivity.
#[inline]
pub fn mu(mu0: f32, fz: f32) -> f32 {
    mu0 * clamp(1.0 - MU_LOAD * (fz / FZ0 - 1.0), MU_FLOOR, 2.0)
}

/// Force and moment from one contact patch.
///
/// `kappa` is the longitudinal slip ratio, `alpha` the slip angle in radians,
/// `fz` the vertical load. `grip` scales both peaks -- the handbrake pulls it
/// down at the rear, and it is where a surface or a tire compound would go.
///
/// Combined slip is handled by normalizing each slip by *its own* peak, taking
/// the resulting vector's length as a single combined slip, and evaluating both
/// curves there. Braking and cornering then compete for one budget the way they
/// do in reality: stand on the brakes mid-corner and the lateral force drops,
/// without either component ever being clamped by hand.
pub fn evaluate(fz: f32, kappa: f32, alpha: f32, grip: f32) -> Force {
    // A wheel in the air carries nothing. This is reachable: lift the inside
    // front over a kerb at 1.4 g and `fz` really does hit zero.
    if fz <= 0.0 {
        return Force::default();
    }

    let d_x = mu(MU_X0, fz) * grip * fz;
    let d_y = mu(MU_Y0, fz) * grip * fz;
    let bcd_x = K_X1 * sin(2.0 * atan(fz / K_X2));
    let bcd_y = K_Y1 * sin(2.0 * atan(fz / K_Y2));
    let b_x = bcd_x / (C_X * d_x);
    let b_y = bcd_y / (C_Y * d_y);

    // Slip at which each curve peaks, at this load.
    let k_peak = Z_PEAK_X / b_x;
    let a_peak = Z_PEAK_Y / b_y;

    // Normalized slip vector: 1.0 in either axis means "at the peak".
    let nx = kappa / k_peak;
    let ny = alpha / a_peak;
    let n = sqrt(nx * nx + ny * ny);
    if n < SLIP_EPS {
        return Force {
            stiffness_x: bcd_x,
            trail: TRAIL_MECH + TRAIL_0,
            ..Force::default()
        };
    }

    // Both curves evaluated at the *combined* slip, then split along the slip
    // direction. Pure slip in either axis recovers that axis's curve exactly.
    let fx = (nx / n) * magic(n * Z_PEAK_X, C_X, d_x, E_X);
    let fy = (ny / n) * magic(n * Z_PEAK_Y, C_Y, d_y, E_Y);

    // Trail collapses as the patch saturates and inverts past the peak, so the
    // aligning moment stops helping right when the car starts to let go.
    let trail = TRAIL_MECH + TRAIL_0 * cos(TRAIL_C * atan(TRAIL_B * n));

    Force {
        fx,
        fy,
        mz: -trail * fy,
        trail,
        stiffness_x: bcd_x,
        saturation: n,
    }
}

/// Longitudinal slip stiffness alone, for the substep that has no force to
/// evaluate yet but still needs a Jacobian.
#[inline]
pub fn stiffness_x(fz: f32) -> f32 {
    if fz <= 0.0 {
        0.0
    } else {
        K_X1 * sin(2.0 * atan(fz / K_X2))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::abs;

    /// The baked peak constants have to match the coefficients they were
    /// derived from, or every combined-slip split is normalized against the
    /// wrong number.
    #[test]
    fn peak_matches_the_curve() {
        // Scan for the argmax of the outer sin, which is where D is reached.
        let scan = |c: f32, e: f32| {
            let mut best = (0.0f32, -1.0f32);
            let mut z = 0.0f32;
            while z < 20.0 {
                let v = magic(z, c, 1.0, e);
                if v > best.1 {
                    best = (z, v);
                }
                z += 1e-4;
            }
            best.0
        };
        let zx = scan(C_X, E_X);
        let zy = scan(C_Y, E_Y);
        println!("z_peak: x {zx:.4} (baked {Z_PEAK_X:.4}), y {zy:.4} (baked {Z_PEAK_Y:.4})");
        assert!(
            abs(zx - Z_PEAK_X) < 5e-3,
            "Z_PEAK_X is {Z_PEAK_X}, curve peaks at {zx}"
        );
        assert!(
            abs(zy - Z_PEAK_Y) < 5e-3,
            "Z_PEAK_Y is {Z_PEAK_Y}, curve peaks at {zy}"
        );
    }

    /// Pure slip in one axis must reproduce that axis's curve untouched --
    /// the combined-slip split is only allowed to matter when both are nonzero.
    #[test]
    fn pure_slip_hits_the_peak_exactly() {
        let fz = 3000.0;
        let d_x = mu(MU_X0, fz) * fz;
        let b_x = stiffness_x(fz) / (C_X * d_x);
        let f = evaluate(fz, Z_PEAK_X / b_x, 0.0, 1.0);
        println!("pure long: fx {:.0} of D_x {:.0}", f.fx, d_x);
        assert!(f.fx > d_x * 0.999, "pure longitudinal slip missed the peak");
        assert!(abs(f.fy) < 1e-3, "lateral force with no slip angle");
    }

    /// Braking has to cost cornering force. This is the friction ellipse, and
    /// it is the property the old hand-clamped friction circle was standing in
    /// for.
    #[test]
    fn combined_slip_trades_grip() {
        let fz = 3000.0;
        let alpha = 0.12;
        let pure = evaluate(fz, 0.0, alpha, 1.0);
        let combined = evaluate(fz, -0.15, alpha, 1.0);
        println!(
            "fy pure {:.0} -> combined {:.0}, fx {:.0}",
            pure.fy, combined.fy, combined.fx
        );
        assert!(
            abs(combined.fy) < abs(pure.fy) * 0.8,
            "braking did not eat into cornering grip"
        );
        assert!(combined.fx < 0.0, "braking slip produced no braking force");
        // And the total still lives inside the ellipse.
        let d = mu(MU_Y0, fz) * fz;
        let mag = sqrt(combined.fx * combined.fx + combined.fy * combined.fy);
        assert!(
            mag < d * 1.12,
            "combined force {mag:.0} escaped the budget {d:.0}"
        );
    }

    /// Grip must be sub-linear in load, or weight transfer would be free and
    /// the whole balance model underneath it would be pointless.
    #[test]
    fn grip_is_sublinear_in_load() {
        let light = evaluate(1500.0, 0.0, 0.25, 1.0);
        let heavy = evaluate(4500.0, 0.0, 0.25, 1.0);
        let ratio = abs(heavy.fy) / abs(light.fy);
        println!(
            "3x the load buys {ratio:.2}x the grip (mu {:.2} -> {:.2})",
            mu(MU_Y0, 1500.0),
            mu(MU_Y0, 4500.0)
        );
        assert!(ratio > 1.5 && ratio < 2.85, "load sensitivity is {ratio:.2}x");
    }

    /// Past the peak, more slip has to mean less force. Without this there is
    /// no such thing as losing the back end.
    #[test]
    fn force_falls_off_past_the_peak() {
        let fz = 3000.0;
        let peak = abs(evaluate(fz, 0.0, 0.17, 1.0).fy);
        let slide = abs(evaluate(fz, 0.0, 1.2, 1.0).fy);
        println!(
            "lateral: peak {peak:.0} N, full slide {slide:.0} N ({:.0}%)",
            slide / peak * 100.0
        );
        assert!(
            slide < peak * 0.92,
            "sliding grip {slide:.0} is not below peak {peak:.0}"
        );
        assert!(slide > peak * 0.7, "sliding grip fell off a cliff");
    }

    /// The aligning moment has to peak *before* the grip does and then fall
    /// away. That ordering is the whole warning system: the wheel goes light in
    /// the driver's hands while there is still grip left to save it with.
    #[test]
    fn aligning_moment_peaks_before_the_grip_does() {
        let fz = 3000.0;
        let (mut a_mz, mut mz_peak) = (0.0f32, 0.0f32);
        let (mut a_fy, mut fy_peak) = (0.0f32, 0.0f32);
        let mut a = 0.0f32;
        while a < 0.6 {
            let f = evaluate(fz, 0.0, a, 1.0);
            if abs(f.mz) > mz_peak {
                mz_peak = abs(f.mz);
                a_mz = a;
            }
            if abs(f.fy) > fy_peak {
                fy_peak = abs(f.fy);
                a_fy = a;
            }
            a += 1e-3;
        }
        println!(
            "Mz peaks at {a_mz:.3} rad ({mz_peak:.0} N.m), Fy at {a_fy:.3} rad ({fy_peak:.0} N)"
        );
        assert!(a_mz < a_fy * 0.8, "aligning moment does not lead the grip peak");
        let at_limit = abs(evaluate(fz, 0.0, a_fy, 1.0).mz);
        println!("Mz down to {at_limit:.0} N.m by the grip limit");
        assert!(at_limit < mz_peak * 0.8, "trail had not collapsed by the grip limit");
        // And it restores rather than diverges: the moment always turns the
        // wheel back towards its direction of travel, whichever way it is
        // slipping. Positive alpha means the wheel is pointed left of where it
        // is going, so the couple that undoes that is negative.
        assert!(evaluate(fz, 0.0, 0.03, 1.0).mz < 0.0, "aligning moment does not restore");
        assert!(evaluate(fz, 0.0, -0.03, 1.0).mz > 0.0, "aligning moment is not symmetric");
    }

    #[test]
    fn a_wheel_in_the_air_carries_nothing() {
        let f = evaluate(0.0, 0.3, 0.3, 1.0);
        assert_eq!(f.fx, 0.0);
        assert_eq!(f.fy, 0.0);
        assert_eq!(f.mz, 0.0);
    }
}
