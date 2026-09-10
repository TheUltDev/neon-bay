//! Small deterministic math kernel.
//!
//! Every transcendental used by the simulation lives here and is implemented in
//! plain `f32` arithmetic. That is not premature optimization -- it is the whole
//! reason client prediction can be bit-exact: `libm`'s `sinf` on x86-64 and the
//! one linked into a wasm module are *not* guaranteed to agree in the last ulp,
//! and a one-ulp difference compounds over a few hundred ticks into a visible
//! desync. Polynomials evaluated with `+`, `-` and `*` are IEEE-754 exact on
//! both targets, so both builds produce identical bits.

pub const PI: f32 = 3.141_592_7;
pub const TAU: f32 = 6.283_185_5;
pub const INV_TAU: f32 = 0.159_154_94;
pub const HALF_PI: f32 = 1.570_796_3;

#[inline]
pub fn clamp(v: f32, lo: f32, hi: f32) -> f32 {
    if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

#[inline]
pub fn lerp(a: f32, b: f32, t: f32) -> f32 {
    a + (b - a) * t
}

/// Explicit comparisons rather than `f32::max` / `f32::min`.
///
/// The library versions disagree with wasm's `f32.max` about NaN, so Rust emits
/// a fix-up on that target and not on x86-64. The answers still match for
/// ordinary inputs, but "ordinary inputs" is not a claim this crate is willing
/// to make about arithmetic the netcode depends on being bit-identical.
#[inline]
pub fn max(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

#[inline]
pub fn min(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

#[inline]
pub fn signum(v: f32) -> f32 {
    if v > 0.0 {
        1.0
    } else if v < 0.0 {
        -1.0
    } else {
        0.0
    }
}

/// Wrap an angle into `[-PI, PI]` without `fmod`.
///
/// The `as i32` cast is saturating (and matches wasm's `i32.trunc_sat_f32_s`
/// exactly), so this is deterministic on every target.
#[inline]
pub fn wrap_pi(a: f32) -> f32 {
    let k = (a * INV_TAU + if a >= 0.0 { 0.5 } else { -0.5 }) as i32;
    a - (k as f32) * TAU
}

/// `sin` for any angle. Max error ~6e-8, i.e. below `f32` resolution.
#[inline]
pub fn sin(a: f32) -> f32 {
    let mut x = wrap_pi(a);
    // Fold [-PI, PI] onto [-PI/2, PI/2]; sin(PI - x) == sin(x).
    if x > HALF_PI {
        x = PI - x;
    } else if x < -HALF_PI {
        x = -PI - x;
    }
    poly_sin(x)
}

#[inline]
pub fn cos(a: f32) -> f32 {
    sin(a + HALF_PI)
}

/// Both at once -- the simulation almost always wants the pair.
#[inline]
pub fn sin_cos(a: f32) -> (f32, f32) {
    (sin(a), cos(a))
}

/// Taylor series of `sin` truncated after x^11, valid on `[-PI/2, PI/2]`.
#[inline]
fn poly_sin(x: f32) -> f32 {
    const S1: f32 = -1.666_666_7e-1;
    const S2: f32 = 8.333_333_3e-3;
    const S3: f32 = -1.984_127e-4;
    const S4: f32 = 2.755_731_4e-6;
    const S5: f32 = -2.505_21e-8;
    let x2 = x * x;
    let p = S1 + x2 * (S2 + x2 * (S3 + x2 * (S4 + x2 * S5)));
    x + x * x2 * p
}

/// `atan` on the whole line. Minimax-ish rational-free polynomial on `[-1, 1]`
/// plus the reciprocal identity outside it. Max error ~2e-7.
#[inline]
pub fn atan(x: f32) -> f32 {
    let ax = abs(x);
    if ax <= 1.0 {
        poly_atan(x)
    } else {
        let r = poly_atan(1.0 / ax);
        let v = HALF_PI - r;
        if x < 0.0 {
            -v
        } else {
            v
        }
    }
}

#[inline]
fn poly_atan(x: f32) -> f32 {
    const A1: f32 = 0.999_977_25;
    const A3: f32 = -0.332_623_47;
    const A5: f32 = 0.193_543_46;
    const A7: f32 = -0.116_432_87;
    const A9: f32 = 0.052_653_32;
    const A11: f32 = -0.011_721_2;
    let x2 = x * x;
    x * (A1 + x2 * (A3 + x2 * (A5 + x2 * (A7 + x2 * (A9 + x2 * A11)))))
}

/// `tan` via the pair, which is all the steering geometry needs. Blows up near
/// +/-PI/2, which no steering angle here goes anywhere near.
#[inline]
pub fn tan(a: f32) -> f32 {
    let (s, c) = sin_cos(a);
    s / c
}

#[inline]
pub fn atan2(y: f32, x: f32) -> f32 {
    if x > 0.0 {
        atan(y / x)
    } else if x < 0.0 {
        if y >= 0.0 {
            atan(y / x) + PI
        } else {
            atan(y / x) - PI
        }
    } else if y > 0.0 {
        HALF_PI
    } else if y < 0.0 {
        -HALF_PI
    } else {
        0.0
    }
}

#[inline]
pub fn abs(v: f32) -> f32 {
    f32::from_bits(v.to_bits() & 0x7fff_ffff)
}

/// IEEE-754 `sqrt` is correctly rounded on every target we build for, so it is
/// safe to use directly (unlike `sin`/`atan`).
#[inline]
pub fn sqrt(v: f32) -> f32 {
    if v <= 0.0 {
        0.0
    } else {
        v.sqrt()
    }
}

/// Shortest signed angular distance from `a` to `b`.
#[inline]
pub fn angle_delta(a: f32, b: f32) -> f32 {
    wrap_pi(b - a)
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct V2 {
    pub x: f32,
    pub y: f32,
}

impl V2 {
    pub const ZERO: V2 = V2 { x: 0.0, y: 0.0 };

    #[inline]
    pub fn new(x: f32, y: f32) -> Self {
        V2 { x, y }
    }

    /// Unit vector at `a` radians (0 = +X, counter-clockwise).
    #[inline]
    pub fn from_angle(a: f32) -> Self {
        let (s, c) = sin_cos(a);
        V2 { x: c, y: s }
    }

    #[inline]
    pub fn add(self, o: V2) -> V2 {
        V2::new(self.x + o.x, self.y + o.y)
    }

    #[inline]
    pub fn sub(self, o: V2) -> V2 {
        V2::new(self.x - o.x, self.y - o.y)
    }

    #[inline]
    pub fn scale(self, k: f32) -> V2 {
        V2::new(self.x * k, self.y * k)
    }

    #[inline]
    pub fn dot(self, o: V2) -> f32 {
        self.x * o.x + self.y * o.y
    }

    /// 2D cross product (z component of the 3D cross).
    #[inline]
    pub fn cross(self, o: V2) -> f32 {
        self.x * o.y - self.y * o.x
    }

    /// Rotated 90 degrees counter-clockwise.
    #[inline]
    pub fn perp(self) -> V2 {
        V2::new(-self.y, self.x)
    }

    #[inline]
    pub fn len_sq(self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    #[inline]
    pub fn len(self) -> f32 {
        sqrt(self.len_sq())
    }

    #[inline]
    pub fn normalize(self) -> V2 {
        let l = self.len();
        if l > 1e-6 {
            self.scale(1.0 / l)
        } else {
            V2::ZERO
        }
    }

    /// World -> body frame, where the body's +X axis points along `angle`.
    #[inline]
    pub fn to_local(self, angle: f32) -> V2 {
        let (s, c) = sin_cos(angle);
        V2::new(self.x * c + self.y * s, -self.x * s + self.y * c)
    }

    /// Body -> world frame.
    #[inline]
    pub fn to_world(self, angle: f32) -> V2 {
        let (s, c) = sin_cos(angle);
        V2::new(self.x * c - self.y * s, self.x * s + self.y * c)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32, eps: f32) {
        assert!(abs(a - b) < eps, "{a} != {b}");
    }

    #[test]
    fn trig_matches_libm() {
        let mut a = -20.0f32;
        while a < 20.0 {
            approx(sin(a), a.sin(), 2e-6);
            approx(cos(a), a.cos(), 2e-6);
            a += 0.013;
        }
    }

    #[test]
    fn atan2_matches_libm() {
        for yi in -20..=20 {
            for xi in -20..=20 {
                let (y, x) = (yi as f32 * 0.37, xi as f32 * 0.41);
                if x == 0.0 && y == 0.0 {
                    continue;
                }
                approx(atan2(y, x), y.atan2(x), 3e-6);
            }
        }
    }

    #[test]
    fn tan_matches_libm() {
        let mut a = -1.4f32;
        while a < 1.4 {
            approx(tan(a), a.tan(), 2e-5);
            a += 0.011;
        }
    }

    #[test]
    fn wrap_is_stable() {
        approx(wrap_pi(PI + 0.1), -PI + 0.1, 1e-5);
        approx(wrap_pi(-PI - 0.1), PI - 0.1, 1e-5);
        approx(wrap_pi(0.3), 0.3, 1e-6);
        approx(wrap_pi(100.0 * TAU + 0.25), 0.25, 1e-3);
    }

    #[test]
    fn local_world_roundtrip() {
        let v = V2::new(3.0, -7.0);
        let r = v.to_local(0.7).to_world(0.7);
        approx(r.x, v.x, 1e-4);
        approx(r.y, v.y, 1e-4);
    }
}
